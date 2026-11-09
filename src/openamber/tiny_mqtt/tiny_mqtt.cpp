#include "tiny_mqtt.h"

#include "esphome/components/network/util.h"
#include "esphome/core/application.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include <cerrno>
#include <cstring>

namespace esphome::tiny_mqtt {

static const char *const TAG = "tiny_mqtt";

// MQTT 3.1.1 control packet types (fixed header, upper nibble).
static constexpr uint8_t PKT_CONNECT = 0x10;
static constexpr uint8_t PKT_CONNACK = 0x20;
static constexpr uint8_t PKT_PUBLISH = 0x30;
static constexpr uint8_t PKT_PINGREQ = 0xC0;
static constexpr uint8_t PKT_DISCONNECT = 0xE0;

static constexpr uint8_t CONNECT_FLAG_USERNAME = 0x80;
static constexpr uint8_t CONNECT_FLAG_PASSWORD = 0x40;
static constexpr uint8_t CONNECT_FLAG_CLEAN_SESSION = 0x02;
static constexpr uint16_t KEEPALIVE_S = 60;
// While lingering, send a PINGREQ after this much idle time, well within the keepalive,
// so the broker keeps the session open however long `linger` is.
static constexpr uint32_t PING_INTERVAL_MS = KEEPALIVE_S * 1000 / 2;
// MQTT remaining length is at most 4 bytes (268,435,455); our packets are far smaller.
static constexpr size_t MAX_REMAINING_LENGTH = 268435455;

static void put_remaining_length(std::vector<uint8_t> &out, size_t len) {
  do {
    uint8_t byte = len % 128;
    len /= 128;
    if (len > 0)
      byte |= 0x80;
    out.push_back(byte);
  } while (len > 0);
}

static void put_u16(std::vector<uint8_t> &out, uint16_t v) {
  out.push_back(static_cast<uint8_t>(v >> 8));
  out.push_back(static_cast<uint8_t>(v & 0xFF));
}

static void put_string(std::vector<uint8_t> &out, const char *s, size_t len) {
  put_u16(out, static_cast<uint16_t>(len));
  out.insert(out.end(), s, s + len);
}

static size_t remaining_length_size(size_t len) {
  size_t n = 1;
  while (len >= 128) {
    len /= 128;
    n++;
  }
  return n;
}

void TinyMqttClient::setup() {
  if (this->client_id_.empty()) {
    char mac[MAC_ADDRESS_BUFFER_SIZE];
    get_mac_address_into_buffer(mac);
    this->client_id_ = App.get_name() + "-" + mac;
  }
  // Nothing to do until the first publish().
  this->disable_loop();
}

void TinyMqttClient::dump_config() {
  ESP_LOGCONFIG(TAG,
                "Tiny MQTT publisher:\n"
                "  Broker: %s:%u\n"
                "  Client ID: %s\n"
                "  Username: %s\n"
                "  Linger: %" PRIu32 " ms\n"
                "  Max queue: %u bytes",
                this->host_, this->port_, this->client_id_.c_str(), this->username_, this->linger_ms_,
                (unsigned) this->max_queue_size_);
}

bool TinyMqttClient::can_publish() const { return network::is_connected(); }

bool TinyMqttClient::publish(const char *topic, const char *payload, size_t payload_len, bool retain) {
  if (!network::is_connected()) {
    return false;
  }
  const size_t topic_len = strlen(topic);
  if (topic_len == 0 || topic_len > 0xFFFF) {
    ESP_LOGW(TAG, "Invalid topic length %u", (unsigned) topic_len);
    return false;
  }
  // Wildcards are only valid in subscriptions; a broker closes the connection on such a PUBLISH.
  if (strpbrk(topic, "+#") != nullptr) {
    ESP_LOGW(TAG, "Wildcards are not allowed in publish topic '%s'", topic);
    return false;
  }
  const size_t remaining = 2 + topic_len + payload_len;
  if (remaining > MAX_REMAINING_LENGTH) {
    return false;
  }
  const size_t packet_len = 1 + remaining_length_size(remaining) + remaining;
  if (this->queued_bytes_() + packet_len > this->max_queue_size_) {
    this->dropped_++;
    ESP_LOGW(TAG, "Queue full, dropped message for '%s' (%" PRIu32 " dropped)", topic, this->dropped_);
    return false;
  }

  // While a session is open, append straight to the transmit buffer.
  if (this->state_ == State::SESSION) {
    if (this->tx_offset_ > 0) {
      // Drop the already written prefix so tx_ never holds more than max_queue_size_ bytes.
      this->tx_.erase(this->tx_.begin(), this->tx_.begin() + this->tx_offset_);
      this->tx_offset_ = 0;
    }
    if (this->tx_.empty())
      this->last_tx_progress_ms_ = millis();
  }
  std::vector<uint8_t> &out = this->state_ == State::SESSION ? this->tx_ : this->pending_;
  out.reserve(out.size() + packet_len);
  out.push_back(PKT_PUBLISH | (retain ? 0x01 : 0x00));
  put_remaining_length(out, remaining);
  put_string(out, topic, topic_len);
  out.insert(out.end(), payload, payload + payload_len);

  this->last_publish_ms_ = millis();
  this->enable_loop();
  return true;
}

bool TinyMqttClient::publish_json(const std::string &topic, const json::json_build_t &f, bool retain) {
  auto message = json::build_json(f);
  return this->publish(topic.c_str(), message.data(), message.size(), retain);
}

void TinyMqttClient::loop() {
  switch (this->state_) {
    case State::IDLE:
      if (this->pending_.empty()) {
        this->disable_loop();
        return;
      }
      this->start_session_();
      break;
    case State::RESOLVING:
      this->poll_resolving_();
      break;
    case State::CONNECTING:
      this->poll_connecting_();
      break;
    case State::WAIT_CONNACK:
      this->poll_connack_();
      break;
    case State::SESSION:
      this->poll_session_();
      break;
  }
}

void TinyMqttClient::start_session_() {
  if (!network::is_connected()) {
    this->fail_("Network down", 0);
    return;
  }
  this->state_ = State::RESOLVING;
  this->state_start_ms_ = millis();
  this->poll_resolving_();
}

void TinyMqttClient::poll_resolving_() {
  if (this->resolve_.consume_failure()) {
    this->fail_("DNS lookup failed", 0);
    return;
  }
  this->resolve_.start(this->host_, this->port_, TAG);
  if (!this->resolve_.ready()) {
    if (millis() - this->state_start_ms_ > this->connect_timeout_ms_) {
      this->fail_("DNS lookup timed out", ETIMEDOUT);
    }
    return;
  }

  struct sockaddr_storage dest;
  socklen_t dest_len = this->resolve_.to_sockaddr(reinterpret_cast<struct sockaddr *>(&dest), sizeof(dest), this->port_);
  if (dest_len == 0) {
    this->fail_("No broker address", 0);
    return;
  }
  this->sock_ = socket::socket(dest.ss_family, SOCK_STREAM, IPPROTO_TCP);
  if (this->sock_ == nullptr) {
    this->fail_("Socket create failed", errno);
    return;
  }
  this->sock_->setblocking(false);
  int yes = 1;
  this->sock_->setsockopt(IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));

  this->state_ = State::CONNECTING;
  this->state_start_ms_ = millis();
  if (this->sock_->connect(reinterpret_cast<struct sockaddr *>(&dest), dest_len) != 0 && errno != EINPROGRESS) {
    this->fail_("Connect failed", errno);
  }
}

void TinyMqttClient::poll_connecting_() {
  int err = 0;
  switch (socket::poll_connect(*this->sock_, err)) {
    case socket::ConnectPollResult::CONNECT_POLL_RESULT_PENDING:
      if (millis() - this->state_start_ms_ > this->connect_timeout_ms_) {
        this->fail_("Connect timed out", ETIMEDOUT);
      }
      return;
    case socket::ConnectPollResult::CONNECT_POLL_RESULT_ERROR:
      this->fail_("Connect failed", err);
      return;
    default:
      break;
  }
  this->tx_.clear();
  this->tx_offset_ = 0;
  this->encode_connect_(this->tx_);
  this->connack_len_ = 0;
  this->state_ = State::WAIT_CONNACK;
  this->state_start_ms_ = millis();
  this->flush_tx_();
}

void TinyMqttClient::poll_connack_() {
  if (!this->flush_tx_())
    return;
  while (this->connack_len_ < sizeof(this->connack_)) {
    ssize_t n = this->sock_->read(this->connack_ + this->connack_len_, sizeof(this->connack_) - this->connack_len_);
    if (n > 0) {
      this->connack_len_ += static_cast<uint8_t>(n);
      continue;
    }
    if (n == 0) {
      this->fail_("Broker closed connection", 0);
      return;
    }
    if (errno != EAGAIN && errno != EWOULDBLOCK) {
      this->fail_("Read failed", errno);
      return;
    }
    if (millis() - this->state_start_ms_ > this->connect_timeout_ms_) {
      this->fail_("No CONNACK", ETIMEDOUT);
    }
    return;
  }
  if (this->connack_[0] != PKT_CONNACK || this->connack_[1] != 0x02) {
    this->fail_("Unexpected reply to CONNECT", 0);
    return;
  }
  if (this->connack_[3] != 0) {
    // 4 = bad username/password, 5 = not authorized.
    ESP_LOGW(TAG, "Broker refused connection, return code %u", this->connack_[3]);
    this->fail_("Connection refused", 0);
    return;
  }
  ESP_LOGD(TAG, "Connected to %s:%u", this->host_, this->port_);

  // Hand everything queued so far to the socket; later publishes append to tx_ directly.
  this->tx_.swap(this->pending_);
  this->tx_offset_ = 0;
  std::vector<uint8_t>().swap(this->pending_);
  this->state_ = State::SESSION;
  this->last_tx_progress_ms_ = millis();
  this->flush_tx_();
}

void TinyMqttClient::poll_session_() {
  // Drain anything the broker sends (QoS 0 publishes get no reply, so normally nothing).
  uint8_t scratch[16];
  for (;;) {
    ssize_t n = this->sock_->read(scratch, sizeof(scratch));
    if (n > 0)
      continue;
    if (n == 0) {
      this->fail_("Broker closed connection", 0);
      return;
    }
    if (errno != EAGAIN && errno != EWOULDBLOCK) {
      this->fail_("Read failed", errno);
      return;
    }
    break;
  }

  if (!this->flush_tx_())
    return;
  if (this->tx_offset_ < this->tx_.size()) {
    // Socket busy, try again next loop. Give up when the broker stops reading altogether,
    // so a later publish can start a fresh session instead of filling a dead one.
    if (millis() - this->last_tx_progress_ms_ > this->connect_timeout_ms_)
      this->fail_("Write stalled", ETIMEDOUT);
    return;
  }
  if (millis() - this->last_publish_ms_ < this->linger_ms_) {
    if (millis() - this->last_tx_progress_ms_ >= PING_INTERVAL_MS) {
      // The PINGRESP is drained by the read loop above.
      this->tx_.push_back(PKT_PINGREQ);
      this->tx_.push_back(0x00);
      this->tx_offset_ = 0;
      this->last_tx_progress_ms_ = millis();
      this->flush_tx_();
    }
    return;
  }

  ESP_LOGD(TAG, "Session done, disconnecting");
  this->close_(true);
}

bool TinyMqttClient::flush_tx_() {
  while (this->tx_offset_ < this->tx_.size()) {
    ssize_t n = this->sock_->write(this->tx_.data() + this->tx_offset_, this->tx_.size() - this->tx_offset_);
    if (n > 0) {
      this->tx_offset_ += n;
      this->last_tx_progress_ms_ = millis();
      continue;
    }
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
      return true;
    this->fail_("Write failed", n < 0 ? errno : 0);
    return false;
  }
  // Everything written: release the buffer instead of keeping its capacity around.
  std::vector<uint8_t>().swap(this->tx_);
  this->tx_offset_ = 0;
  return true;
}

void TinyMqttClient::encode_connect_(std::vector<uint8_t> &out) const {
  const size_t client_id_len = this->client_id_.size();
  const size_t username_len = strlen(this->username_);
  const size_t password_len = strlen(this->password_);

  uint8_t flags = CONNECT_FLAG_CLEAN_SESSION;
  size_t remaining = 10 + 2 + client_id_len;  // variable header + client id
  if (username_len > 0) {
    flags |= CONNECT_FLAG_USERNAME;
    remaining += 2 + username_len;
    if (password_len > 0) {
      flags |= CONNECT_FLAG_PASSWORD;
      remaining += 2 + password_len;
    }
  }

  out.reserve(out.size() + 1 + remaining_length_size(remaining) + remaining);
  out.push_back(PKT_CONNECT);
  put_remaining_length(out, remaining);
  put_string(out, "MQTT", 4);
  out.push_back(0x04);  // protocol level 3.1.1
  out.push_back(flags);
  put_u16(out, KEEPALIVE_S);
  put_string(out, this->client_id_.data(), client_id_len);
  if (flags & CONNECT_FLAG_USERNAME)
    put_string(out, this->username_, username_len);
  if (flags & CONNECT_FLAG_PASSWORD)
    put_string(out, this->password_, password_len);
}

void TinyMqttClient::fail_(const char *what, int err) {
  const size_t lost = this->queued_bytes_();
  ESP_LOGW(TAG, "%s (%d), discarding %u queued bytes", what, err, (unsigned) lost);
  this->close_(false);
  // QoS 0: a failed session drops its messages; the next publish starts a fresh attempt.
  std::vector<uint8_t>().swap(this->pending_);
  this->resolve_.forget();
}

void TinyMqttClient::close_(bool send_disconnect) {
  if (this->sock_ != nullptr) {
    if (send_disconnect) {
      const uint8_t disconnect[2] = {PKT_DISCONNECT, 0x00};
      this->sock_->write(disconnect, sizeof(disconnect));
    }
    this->sock_->shutdown(SHUT_RDWR);
    this->sock_->close();
    this->sock_.reset();
  }
  std::vector<uint8_t>().swap(this->tx_);
  this->tx_offset_ = 0;
  // loop() starts a new session if pending_ is non-empty, otherwise it disables itself.
  this->state_ = State::IDLE;
}

}  // namespace esphome::tiny_mqtt
