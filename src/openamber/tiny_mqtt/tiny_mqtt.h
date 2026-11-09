#pragma once

#include "esphome/components/json/json_util.h"
#include "esphome/components/socket/ipv4_resolve.h"
#include "esphome/components/socket/socket.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"

#include <memory>
#include <string>
#include <vector>

namespace esphome::tiny_mqtt {

/// Minimal MQTT 3.1.1 publisher, QoS 0 only, no subscriptions.
///
/// A connection is only opened while there is something to send:
///   publish() -> DNS -> TCP connect -> CONNECT/CONNACK -> PUBLISH... -> linger -> DISCONNECT -> close
/// Between sessions no socket, task or buffer is kept, so the idle heap cost is just this object.
class TinyMqttClient : public Component {
 public:
  // Config strings come from codegen literals (flash), so they are stored as pointers.
  void set_broker(const char *host, uint16_t port) {
    this->host_ = host;
    this->port_ = port;
  }
  void set_credentials(const char *username, const char *password) {
    this->username_ = username;
    this->password_ = password;
  }
  void set_client_id(const std::string &client_id) { this->client_id_ = client_id; }
  void set_linger(uint32_t ms) { this->linger_ms_ = ms; }
  void set_connect_timeout(uint32_t ms) { this->connect_timeout_ms_ = ms; }
  void set_max_queue_size(size_t bytes) { this->max_queue_size_ = bytes; }

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  /// Queue a QoS 0 message. Returns false when the network is down or the queue is full.
  /// true means "accepted", not "delivered" (QoS 0 semantics).
  bool publish(const std::string &topic, const std::string &payload, bool retain = false) {
    return this->publish(topic.c_str(), payload.data(), payload.size(), retain);
  }
  /// Compatibility overload with ESPHome's MQTTClientComponent::publish(topic, payload, qos, retain).
  bool publish(const std::string &topic, const std::string &payload, uint8_t qos, bool retain) {
    (void) qos;
    return this->publish(topic.c_str(), payload.data(), payload.size(), retain);
  }
  bool publish(const char *topic, const char *payload, size_t payload_len, bool retain);
  bool publish_json(const std::string &topic, const json::json_build_t &f, bool retain = false);

  /// True when a publish would be accepted (network up). There is no persistent connection.
  bool can_publish() const;
  /// True while a broker session is open.
  bool is_session_active() const { return this->state_ == State::SESSION; }

 protected:
  enum class State : uint8_t {
    IDLE,
    RESOLVING,
    CONNECTING,
    WAIT_CONNACK,
    SESSION,
  };

  void start_session_();
  void poll_resolving_();
  void poll_connecting_();
  void poll_connack_();
  void poll_session_();
  /// Write as much of tx_ as the socket accepts. Returns false on a fatal socket error.
  bool flush_tx_();
  void fail_(const char *what, int err);
  void close_(bool send_disconnect);
  void encode_connect_(std::vector<uint8_t> &out) const;
  size_t queued_bytes_() const { return this->pending_.size() + (this->tx_.size() - this->tx_offset_); }

  const char *host_{""};
  const char *username_{""};
  const char *password_{""};
  std::string client_id_;
  uint16_t port_{1883};
  uint32_t linger_ms_{2000};
  uint32_t connect_timeout_ms_{10000};
  size_t max_queue_size_{8192};

  State state_{State::IDLE};
  uint32_t state_start_ms_{0};
  uint32_t last_publish_ms_{0};
  /// Last time the socket accepted bytes (or tx_ became non-empty); used to detect a stalled session.
  uint32_t last_tx_progress_ms_{0};
  uint32_t dropped_{0};

  socket::Ipv4Resolve resolve_;
  std::unique_ptr<socket::Socket> sock_;
  /// Encoded PUBLISH packets waiting for a session.
  std::vector<uint8_t> pending_;
  /// Bytes on their way to the socket; tx_offset_ marks what is already written.
  std::vector<uint8_t> tx_;
  size_t tx_offset_{0};
  uint8_t connack_[4]{};
  uint8_t connack_len_{0};
};

template<typename... Ts> class PublishAction final : public Action<Ts...> {
 public:
  explicit PublishAction(TinyMqttClient *parent) : parent_(parent) {}
  TEMPLATABLE_VALUE(std::string, topic)
  TEMPLATABLE_VALUE(std::string, payload)
  TEMPLATABLE_VALUE(bool, retain)

  void play(const Ts &...x) override {
    this->parent_->publish(this->topic_.value(x...), this->payload_.value(x...), this->retain_.value(x...));
  }

 protected:
  TinyMqttClient *parent_;
};

template<typename... Ts> class PublishJsonAction final : public Action<Ts...> {
 public:
  explicit PublishJsonAction(TinyMqttClient *parent) : parent_(parent) {}
  TEMPLATABLE_VALUE(std::string, topic)
  TEMPLATABLE_VALUE(bool, retain)

  void set_payload(std::function<void(Ts..., JsonObject)> payload) { this->payload_ = payload; }

  void play(const Ts &...x) override {
    this->parent_->publish_json(
        this->topic_.value(x...), [this, x...](JsonObject root) { this->payload_(x..., root); },
        this->retain_.value(x...));
  }

 protected:
  std::function<void(Ts..., JsonObject)> payload_;
  TinyMqttClient *parent_;
};

}  // namespace esphome::tiny_mqtt
