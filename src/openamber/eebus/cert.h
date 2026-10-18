/*
 * Native EEBUS component for ESPHome
 *
 * Certificate & key material for the SHIP (TLS) connection.
 * - EC P-256 keypair + self-signed X.509 certificate.
 * - SKI = SubjectKeyIdentifier (hex) used by EMS/CEM (e.g. evcc) for pairing.
 * - Generated dynamically on first boot and persisted to NVS so the SKI
 *   stays stable across reboots.
 */

#pragma once

#include "esphome/core/component.h"
#include <cstdint>
#include <string>
#include <vector>

namespace esphome {
namespace eebus {

class EEBusCertificateStore {
 public:
  void set_common_name(const std::string &common_name) { this->common_name_ = common_name; }
  bool load_or_create();
  bool is_loaded() const { return !this->cert_pem_.empty(); }

  const std::string &get_cert_pem() const { return this->cert_pem_; }
  const std::string &get_key_pem() const { return this->key_pem_; }
  const std::string &get_ski() const { return this->ski_; }

 private:
  bool generate_and_save();
  bool save_to_nvs();
  bool load_from_nvs();

  std::string common_name_{"EEBUS-Device"};
  std::string cert_pem_;
  std::string key_pem_;
  std::string ski_;
};

}  // namespace eebus
}  // namespace esphome
