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

 private:
  void teardown();

  uint16_t port_{0};
  bool ready_{false};
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
};

}  // namespace openamber_eebus
}  // namespace esphome
