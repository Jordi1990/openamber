#pragma once

#include "esphome/core/component.h"
#include "cert.h"
#include "ship_listener.h"
#include "eebus_ohpcf.h"
#include "eebus_lpc.h"
#include "eebus_measurements.h"
#include "eebus_node.h"
#include <functional>
#include <string>

namespace esphome {
namespace openamber_eebus {

class EEBusComponent : public PollingComponent {
 public:
  void set_device_sku(const std::string &sku) { this->device_sku_ = sku; }
  void set_brand(const std::string &brand) { this->brand_ = brand; }
  void set_model(const std::string &model) { this->model_ = model; }
  void set_failsafe_duration(uint32_t seconds) { this->failsafe_duration_s_ = seconds; }

  // OpenAmber bridge (wired from YAML lambdas that can access id(...)).
  void set_read_power(const std::function<float()> &fn) { this->read_power_ = fn; }
  void set_read_dhw_temp(const std::function<float()> &fn) { this->read_dhw_temp_ = fn; }
  void set_apply_optional(const std::function<bool(bool)> &fn) { this->apply_optional_ = fn; }
  void set_apply_dim(const std::function<bool(bool)> &fn) { this->apply_dim_ = fn; }

  void setup() override;
  void update() override;

  // Getters exposed to the SHIP/SPINE layer for outbound use-case data.
  float get_dhw_temperature_c() const { return this->mdt_.get_temperature_c(); }
  float get_power_w() const { return this->mpc_.get_power_w(); }

  OhpcfServer &ohpcf() { return this->ohpcf_; }
  LpcServer &lpc() { return this->lpc_; }
  EebusNode &node() { return this->node_; }

 private:
  std::string device_sku_;
  std::string brand_;
  std::string model_;
  uint32_t failsafe_duration_s_{7200};
  std::string ship_id_;
  bool listener_started_{false};

  EEBusCertificateStore cert_store_;
  EEBusShipListener ship_listener_;
  OhpcfServer ohpcf_;
  LpcServer lpc_;
  MpcServer mpc_;
  MdtServer mdt_;
  EebusNode node_;

  std::function<float()> read_power_;
  std::function<float()> read_dhw_temp_;
  std::function<bool(bool)> apply_optional_;
  std::function<bool(bool)> apply_dim_;

  void init_certificate_and_keys();
  void start_ship_listener();
  void advertise_mdns();
  void wire_use_cases();
  void check_failsafe();
};

}  // namespace openamber_eebus
}  // namespace esphome
