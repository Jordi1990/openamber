/*
 * Open Amber - native EEBus PoC
 *
 * SHIP TLS listener implementation.
 *
 * A dedicated FreeRTOS task accepts incoming TLS connections and dispatches
 * each to the SHIP/spine handshake layer (connection handler).  In this PoC
 * stage the dispatcher validates the TLS session and closes it; the full SHIP
 * messaging handshake is implemented in the spine layer.
 */

#include "ship_listener.h"
#include "eebus_websocket.h"
#include "eebus_ship.h"
#include "esphome/core/log.h"

#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "errno.h"

#include "esp_heap_caps.h"
#include "mbedtls/ssl.h"
#include "mbedtls/net_sockets.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/x509_crt.h"
#include "mbedtls/pk.h"

namespace esphome {
namespace openamber_eebus {

static const char *const TAG = "eebus_ship";
static const int kShipPort = 44328;
static const int kAcceptQueueSize = 4;

static EEBusShipListener *active_listener = nullptr;

// Custom RNG for mbedTLS using the ESP32 hardware RNG directly.  This avoids
// the mbedTLS entropy pool, which in ESP-IDF uses a lazily-created FreeRTOS
// mutex and can assert (xQueueSemaphoreTake) when triggered during early boot.
static int esp_mbedtls_rng(void *, unsigned char *buf, size_t len) {
  esp_fill_random(buf, len);
  return 0;
}

// Write the whole buffer, retrying on WANT_WRITE and continuing on partial
// writes.  Larger WS frames (the ~1.4 KB discovery reply) need this, otherwise
// a single mbedtls_ssl_write can return WANT_WRITE/partial and the peer never
// sees the frame.
static bool ssl_write_all(mbedtls_ssl_context *ssl, const unsigned char *buf, size_t len) {
  size_t written = 0;
  while (written < len) {
    int ret = mbedtls_ssl_write(ssl, buf + written, len - written);
    if (ret > 0) {
      written += static_cast<size_t>(ret);
      continue;
    }
    if (ret == MBEDTLS_ERR_SSL_WANT_READ || ret == MBEDTLS_ERR_SSL_WANT_WRITE) {
      vTaskDelay(2);
      continue;
    }
    return false;
  }
  return true;
}

// Signature-hash list, guaranteed non-empty so mbedtls_ssl_setup does not fail
// with MBEDTLS_ERR_SSL_BAD_CONFIG (see ssl_handshake_init).  ECDSA+SHA-256
// covers the ECDHE-ECDSA-...-SHA256 ciphersuite evcc uses.
static const uint16_t kEebusSigAlgs[] = {MBEDTLS_TLS1_3_SIG_ECDSA_SECP256R1_SHA256,
                                         MBEDTLS_TLS1_3_SIG_NONE};

// The accept+handshake task runs its stack from PSRAM to spare the scarce
// internal SRAM (internal RAM is tight on the S3 with LVGL + WiFi).
static StaticTask_t ship_task_tcb;
static StackType_t *ship_task_stack = nullptr;
static const uint32_t kShipTaskStackBytes = 32 * 1024;

static void ship_accept_task(void *arg);

void EEBusShipListener::teardown() {
  this->is_connected_ = false;
  if (this->queue_mutex_ != nullptr) {
    vSemaphoreDelete(static_cast<SemaphoreHandle_t>(this->queue_mutex_));
    this->queue_mutex_ = nullptr;
  }
  if (this->ssl_ready_) {
    mbedtls_ssl_free(&this->ssl_);
    this->ssl_ready_ = false;
  }
  if (!this->ready_)
    return;
  mbedtls_ssl_config_free(&this->ssl_conf_);
  mbedtls_x509_crt_free(&this->own_cert_);
  mbedtls_pk_free(&this->own_key_);
  mbedtls_ctr_drbg_free(&this->ctr_drbg_);
  this->ready_ = false;
}

void EEBusShipListener::begin(const EEBusCertificateStore &cert, uint16_t port) {
  this->port_ = port ? port : kShipPort;
  active_listener = this;
  if (this->queue_mutex_ == nullptr) {
    this->queue_mutex_ = xSemaphoreCreateMutex();
  }

  mbedtls_ssl_config_init(&this->ssl_conf_);
  mbedtls_x509_crt_init(&this->own_cert_);
  mbedtls_pk_init(&this->own_key_);
  mbedtls_ctr_drbg_init(&this->ctr_drbg_);

  const unsigned char pers[] = "openamber-ship";
  if (mbedtls_ctr_drbg_seed(&this->ctr_drbg_, esp_mbedtls_rng, nullptr, pers,
                            sizeof(pers)) != 0) {
    ESP_LOGE(TAG, "CTR_DRBG seed failed");
    return;
  }

  if (mbedtls_ssl_config_defaults(&this->ssl_conf_, MBEDTLS_SSL_IS_SERVER,
                                  MBEDTLS_SSL_TRANSPORT_STREAM,
                                  MBEDTLS_SSL_PRESET_DEFAULT) != 0) {
    ESP_LOGE(TAG, "SSL config defaults failed");
    return;
  }
  mbedtls_ssl_conf_authmode(&this->ssl_conf_, MBEDTLS_SSL_VERIFY_OPTIONAL);
  mbedtls_ssl_conf_rng(&this->ssl_conf_, mbedtls_ctr_drbg_random, &this->ctr_drbg_);
  mbedtls_ssl_conf_read_timeout(&this->ssl_conf_, 4000);

  // Force TLS 1.2 and set an explicit (non-empty) signature-algorithm list;
  // without this, mbedtls_ssl_setup can fail with MBEDTLS_ERR_SSL_BAD_CONFIG.
  mbedtls_ssl_conf_min_tls_version(&this->ssl_conf_, MBEDTLS_SSL_VERSION_TLS1_2);
  mbedtls_ssl_conf_max_tls_version(&this->ssl_conf_, MBEDTLS_SSL_VERSION_TLS1_2);
  mbedtls_ssl_conf_sig_algs(&this->ssl_conf_, kEebusSigAlgs);

  int ret = mbedtls_x509_crt_parse(&this->own_cert_,
                                   reinterpret_cast<const unsigned char *>(cert.get_cert_pem().c_str()),
                                   cert.get_cert_pem().size() + 1);
  if (ret != 0) {
    ESP_LOGE(TAG, "Failed to parse certificate: -0x%04x", static_cast<unsigned>(-ret));
    return;
  }
  ret = mbedtls_pk_parse_key(&this->own_key_,
                             reinterpret_cast<const unsigned char *>(cert.get_key_pem().c_str()),
                             cert.get_key_pem().size() + 1, nullptr, 0,
                             mbedtls_ctr_drbg_random, &this->ctr_drbg_);
  if (ret != 0) {
    ESP_LOGE(TAG, "Failed to parse private key: -0x%04x", static_cast<unsigned>(-ret));
    return;
  }
  ret = mbedtls_ssl_conf_own_cert(&this->ssl_conf_, &this->own_cert_, &this->own_key_);
  if (ret != 0) {
    ESP_LOGE(TAG, "Failed to set own cert: -0x%04x", static_cast<unsigned>(-ret));
    return;
  }

  this->ready_ = true;

  // Prepare the reused TLS session (one connection at a time).  Doing the
  // (heap-allocating) mbedtls_ssl_setup once avoids the intermittent
  // MBEDTLS_ERR_SSL_BAD_CONFIG seen with per-connection setup.
  mbedtls_ssl_init(&this->ssl_);
  int setup_ret = mbedtls_ssl_setup(&this->ssl_, &this->ssl_conf_);
  if (setup_ret != 0) {
    ESP_LOGE(TAG, "mbedtls_ssl_setup (reused) failed -0x%04x", static_cast<unsigned>(-setup_ret));
    return;
  }
  this->ssl_ready_ = true;

  // Run the accept/handshake task from PSRAM to keep internal SRAM free.
  size_t stack_words = kShipTaskStackBytes / sizeof(StackType_t);
  ship_task_stack = static_cast<StackType_t *>(
      heap_caps_malloc(kShipTaskStackBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (ship_task_stack == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate %u bytes PSRAM task stack",
             static_cast<unsigned>(kShipTaskStackBytes));
    return;
  }
  xTaskCreateStatic(ship_accept_task, "eebus_ship", stack_words, nullptr,
                    tskIDLE_PRIORITY + 3, ship_task_stack, &ship_task_tcb);
}

static void ship_accept_task(void *arg) {
  (void)arg;
  int listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (listen_fd < 0) {
    ESP_LOGE(TAG, "socket() failed: %d", errno);
    vTaskDelete(nullptr);
    return;
  }

  int opt = 1;
  setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(active_listener->get_port());
  addr.sin_addr.s_addr = htonl(INADDR_ANY);

  if (bind(listen_fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    ESP_LOGE(TAG, "bind() to port %u failed: %d", static_cast<unsigned>(active_listener->get_port()), errno);
    close(listen_fd);
    vTaskDelete(nullptr);
    return;
  }
  if (listen(listen_fd, kAcceptQueueSize) < 0) {
    ESP_LOGE(TAG, "listen() failed: %d", errno);
    close(listen_fd);
    vTaskDelete(nullptr);
    return;
  }
  ESP_LOGI(TAG, "SHIP TLS listener ready on port %u",
           static_cast<unsigned>(active_listener->get_port()));

  socklen_t addr_len = sizeof(addr);
  while (true) {
    int conn = accept(listen_fd, reinterpret_cast<struct sockaddr *>(&addr), &addr_len);
    if (conn < 0)
      continue;
    if (active_listener != nullptr)
      active_listener->handle_connection(conn);
  }
}

void EEBusShipListener::handle_connection(int fd) {
  ESP_LOGI(TAG, "Accepted SHIP connection");
  this->close_requested_ = false;

  // Set socket timeouts so TLS/WS reads can't block forever (evcc times out).
  struct timeval tv;
  tv.tv_sec = 2;
  tv.tv_usec = 0;
  setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

  // Wrap the accepted socket in an mbedTLS net context for the TLS session.
  mbedtls_net_context net;
  mbedtls_net_init(&net);
  net.fd = fd;

  if (!this->ssl_ready_) {
    close(fd);
    return;
  }
  if (mbedtls_ssl_session_reset(&this->ssl_) != 0) {
    ESP_LOGW(TAG, "mbedtls_ssl_session_reset failed");
    close(fd);
    return;
  }
  mbedtls_ssl_set_bio(&this->ssl_, &net, mbedtls_net_send, mbedtls_net_recv, nullptr);

  int ret = -1;
  mbedtls_ssl_context &ssl = this->ssl_;
  uint32_t hs_start = millis();
  while (millis() - hs_start < 4000) {
    ret = mbedtls_ssl_handshake(&ssl);
    if (ret == 0)
      break;
    if (ret == MBEDTLS_ERR_SSL_WANT_READ || ret == MBEDTLS_ERR_SSL_WANT_WRITE) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    break;
  }
  if (ret != 0) {
    ESP_LOGW(TAG, "TLS handshake failed: -0x%04x (want_read=%d want_write=%d)",
             static_cast<unsigned>(-ret),
             ret == MBEDTLS_ERR_SSL_WANT_READ, ret == MBEDTLS_ERR_SSL_WANT_WRITE);
    close(fd);
    return;
  }
  ESP_LOGI(TAG, "TLS handshake OK (version=%s cipher=%s)",
           mbedtls_ssl_get_version(&ssl), mbedtls_ssl_get_ciphersuite(&ssl));

  // Read the client WebSocket upgrade request.
  std::string req;
  char buf[256];
  while (req.find("\r\n\r\n") == std::string::npos) {
    memset(buf, 0, sizeof(buf));
    ret = mbedtls_ssl_read(&ssl, reinterpret_cast<unsigned char *>(buf), sizeof(buf) - 1);
    if (ret == MBEDTLS_ERR_SSL_WANT_READ || ret == MBEDTLS_ERR_SSL_WANT_WRITE) {
      vTaskDelay(2);
      continue;
    }
    if (ret <= 0) {
      ESP_LOGW(TAG, "WS upgrade read failed: -0x%04x / %d",
               ret < 0 ? static_cast<unsigned>(-ret) : 0, ret);
      close(fd);
      return;
    }
    req.append(buf, static_cast<size_t>(ret));
    if (req.size() > 8192)
      break;
  }
  ESP_LOGD(TAG, "WS request (first 200): %.200s", req.c_str());

  std::string ws_resp;
  if (!eebus_websocket_server_response(req, ws_resp)) {
    ESP_LOGW(TAG, "Not a WebSocket upgrade request; raw=<%.160s>", req.c_str());
    close(fd);
    return;
  }
  ssl_write_all(&ssl, reinterpret_cast<const unsigned char *>(ws_resp.data()), ws_resp.size());
  ESP_LOGI(TAG, "WebSocket upgrade accepted");
  this->is_connected_ = true;

  // Short timeout so we can poll outbound_queue_
  tv.tv_sec = 0;
  tv.tv_usec = 100000;
  setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  // Frame loop: read, decode WS frames, echo frame data to the handler.
  EebusWsDecoder decoder;
  std::vector<uint8_t> rbuf(4096);
  while (true) {
    // 1. Drain outbound queue
    if (this->queue_mutex_ != nullptr) {
      std::vector<std::vector<uint8_t>> to_send;
      if (xSemaphoreTake(static_cast<SemaphoreHandle_t>(this->queue_mutex_), pdMS_TO_TICKS(5)) == pdTRUE) {
        to_send.swap(this->outbound_queue_);
        xSemaphoreGive(static_cast<SemaphoreHandle_t>(this->queue_mutex_));
      }
      for (const auto &out_frame : to_send) {
        if (!out_frame.empty()) {
          if (!ssl_write_all(&ssl, out_frame.data(), out_frame.size())) {
            ESP_LOGE(TAG, "Failed to send outbound queued frame (%u bytes)",
                     static_cast<unsigned>(out_frame.size()));
          } else {
            ESP_LOGD(TAG, "Sent outbound queued frame (%u bytes)",
                     static_cast<unsigned>(out_frame.size()));
          }
        }
      }
    }

    ret = mbedtls_ssl_read(&ssl, rbuf.data(), rbuf.size());
    if (this->close_requested_) {
      break;
    }
    if (ret == MBEDTLS_ERR_SSL_WANT_READ || ret == MBEDTLS_ERR_SSL_WANT_WRITE ||
        ret == MBEDTLS_ERR_SSL_TIMEOUT) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    if (ret <= 0) {
      if (ret == MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY) {
        ESP_LOGI(TAG, "Peer closed TLS session cleanly (close_notify)");
      } else {
        ESP_LOGW(TAG, "WS read ended ret=%d phase=%d", ret, static_cast<int>(decoder.phase));
      }
      break;
    }
    // Log the raw bytes we just read only at verbose log level
    ESP_LOGV(TAG, "read ret=%d phase=%d", ret, static_cast<int>(decoder.phase));

    std::vector<EebusWsFrame> frames;
    std::vector<uint8_t> chunk(rbuf.begin(), rbuf.begin() + ret);
    if (!decoder.feed(chunk, frames)) {
      ESP_LOGW(TAG, "WS frame decode error");
      break;
    }
    for (auto &frame : frames) {
      if (frame.opcode == 0x9) {  // WebSocket PING
        ESP_LOGD(TAG, "WebSocket PING received (%u bytes payload) -> sending PONG",
                 static_cast<unsigned>(frame.payload.size()));
        std::vector<uint8_t> pong;
        eebus_websocket_encode_server_frame(0x0A /*Pong*/, frame.payload, pong);
        if (ssl_write_all(&ssl, pong.data(), pong.size())) {
          ESP_LOGD(TAG, "WebSocket PONG sent");
        } else {
          ESP_LOGE(TAG, "Failed to send WebSocket PONG");
        }
        continue;
      }
      if (frame.opcode == 0x8) {  // WebSocket CLOSE
        ESP_LOGI(TAG, "WebSocket CLOSE received");
        this->close_requested_ = true;
        break;
      }
      ESP_LOGD(TAG, "WS frame received (opcode=0x%02x, %u bytes)",
               frame.opcode, static_cast<unsigned>(frame.payload.size()));
      if (this->frame_handler_) {
        std::vector<uint8_t> out = this->frame_handler_(frame.payload);
        if (!out.empty()) {
          if (ssl_write_all(&ssl, out.data(), out.size())) {
            ESP_LOGD(TAG, "reply sent (%u bytes)", static_cast<unsigned>(out.size()));
          } else {
            ESP_LOGE(TAG, "ssl_write_all failed (%u bytes)",
                     static_cast<unsigned>(out.size()));
          }
        }
      }
    }
    if (this->close_requested_ || decoder.phase == EebusWsDecoder::Phase::DONE)
      break;
  }

  this->is_connected_ = false;
  if (this->queue_mutex_ != nullptr) {
    if (xSemaphoreTake(static_cast<SemaphoreHandle_t>(this->queue_mutex_), pdMS_TO_TICKS(50)) == pdTRUE) {
      this->outbound_queue_.clear();
      xSemaphoreGive(static_cast<SemaphoreHandle_t>(this->queue_mutex_));
    }
  }
  close(fd);
  mbedtls_ssl_session_reset(&this->ssl_);
  ESP_LOGI(TAG, "SHIP connection closed");
}

bool EEBusShipListener::queue_outbound_frame(const std::vector<uint8_t> &ws_frame) {
  if (!this->is_connected_ || this->queue_mutex_ == nullptr)
    return false;
  if (xSemaphoreTake(static_cast<SemaphoreHandle_t>(this->queue_mutex_), pdMS_TO_TICKS(50)) == pdTRUE) {
    if (this->outbound_queue_.size() < 16) {
      this->outbound_queue_.push_back(ws_frame);
      xSemaphoreGive(static_cast<SemaphoreHandle_t>(this->queue_mutex_));
      return true;
    }
    xSemaphoreGive(static_cast<SemaphoreHandle_t>(this->queue_mutex_));
  }
  return false;
}

bool EEBusShipListener::send_ship_data(const std::string &spine_json) {
  if (!this->is_connected_)
    return false;
  std::vector<uint8_t> ship_payload = ship_data_frame(spine_json);
  std::vector<uint8_t> ws_frame;
  eebus_websocket_encode_server_frame(0x2 /*binary*/, ship_payload, ws_frame);
  return this->queue_outbound_frame(ws_frame);
}

bool EEBusShipListener::send_ship_control(const std::string &control_json) {
  if (!this->is_connected_)
    return false;
  std::vector<uint8_t> ship_payload = ship_control_frame(control_json);
  std::vector<uint8_t> ws_frame;
  eebus_websocket_encode_server_frame(0x2 /*binary*/, ship_payload, ws_frame);
  return this->queue_outbound_frame(ws_frame);
}

}  // namespace openamber_eebus
}  // namespace esphome
