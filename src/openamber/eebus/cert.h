/*
 * Open Amber - native EEBus PoC
 *
 * Certificate & key material for the SHIP (TLS) connection.
 * - EC P-256 keypair + self-signed X.509 certificate.
 * - SKI = SubjectKeyIdentifier (hex) used by evcc for pairing.
 * - A freshly generated pair is embedded at build time; persisted to NVS so
 *   the SKI stays stable across reboots.
 *
 * See docs/eebus-native-poc.md.
 */

#pragma once

#include "esphome/core/component.h"
#include <cstdint>
#include <string>
#include <vector>

namespace esphome {
namespace openamber_eebus {

class EEBusCertificateStore {
 public:
  bool load_or_create();
  bool is_loaded() const { return !this->cert_pem_.empty(); }

  const std::string &get_cert_pem() const { return this->cert_pem_; }
  const std::string &get_key_pem() const { return this->key_pem_; }
  const std::string &get_ski() const { return this->ski_; }

 private:
  bool generate_and_save();
  bool use_embedded();
  bool save_to_nvs();
  bool load_from_nvs();

  std::string cert_pem_;
  std::string key_pem_;
  std::string ski_;
};

}  // namespace openamber_eebus
}  // namespace esphome
