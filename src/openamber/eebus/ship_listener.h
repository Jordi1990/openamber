/*
 * Open Amber - native EEBus PoC
 *
 * SHIP transport: TLS server accepted over a local TCP socket, advertised via
 * mDNS.  Each accepted connection is given to the SHIP/spine handshake layer.
 *
 * Uses lwIP sockets + mbedTLS (ESP-IDF).
 */

#pragma once

#include "esphome/core/component.h"
#include "cert.h"
#include <functional>
#include <string>
#include <vector>

#include "mbedtls/ssl.h"
#include "mbedtls/x509_crt.h"
#include "mbedtls/pk.h"
#include "mbedtls/ctr_drbg.h"
#include "esp_random.h"

namespace esphome {
namespace openamber_eebus {

class EEBusShipListener {
 public:
  ~EEBusShipListener() { this->teardown(); }

  void begin(const EEBusCertificateStore &cert, uint16_t port);
  uint16_t get_port() const { return this->port_; }

  // Called with each decoded SHIP WS-frame payload (msg-type byte first).
  // Return value: bytes to send back on this connection (already WS-encoded
  // server frame), or empty to send nothing.
  void set_frame_handler(
      const std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> &fn) {
    this->frame_handler_ = fn;
  }

  // Serves a single accepted connection (TLS + WebSocket + SHIP frame loop).
  void handle_connection(int fd);

  bool is_connected() const { return this->is_connected_; }

  // Queue a WS-encoded frame to be sent out over the active connection.
  bool queue_outbound_frame(const std::vector<uint8_t> &ws_frame);

  // High-level helpers: frame as SHIP DATA / CONTROL and queue for sending.
  bool send_ship_data(const std::string &spine_json);
  bool send_ship_control(const std::string &control_json);

  // Request active connection to be closed immediately after sending current frame.
  void request_close() { this->close_requested_ = true; }

 private:
  void teardown();

  uint16_t port_{0};
  bool ready_{false};
  bool is_connected_{false};
  bool close_requested_{false};
  mbedtls_ssl_config ssl_conf_;
  mbedtls_x509_crt own_cert_;
  mbedtls_pk_context own_key_;
  mbedtls_ctr_drbg_context ctr_drbg_;
  // Single, reused TLS session (one connection at a time); mbedtls_ssl_setup
  // is done once so per-connection heap churn can't cause intermittent
  // MBEDTLS_ERR_SSL_BAD_CONFIG.
  mbedtls_ssl_context ssl_;
  bool ssl_ready_{false};
  std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> frame_handler_;

  void *queue_mutex_{nullptr};  // SemaphoreHandle_t created on begin()
  std::vector<std::vector<uint8_t>> outbound_queue_;
};

}  // namespace openamber_eebus
}  // namespace esphome
