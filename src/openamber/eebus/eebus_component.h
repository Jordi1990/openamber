#pragma once

#include "esphome/core/component.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
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
  EEBusComponent();
  ~EEBusComponent();

  void disconnect_ems();

  void set_device_sku(const std::string &sku) { this->device_sku_ = sku; }
  void set_brand(const std::string &brand) { this->brand_ = brand; }
  void set_model(const std::string &model) { this->model_ = model; }
  void set_failsafe_duration(uint32_t seconds) { this->failsafe_duration_s_ = seconds; }

  // Diagnostic sensors for Home Assistant
  void set_status_sensor(text_sensor::TextSensor *sens) { this->status_sensor_ = sens; }
  void set_last_action_sensor(text_sensor::TextSensor *sens) { this->last_action_sensor_ = sens; }
  void set_connected_sensor(binary_sensor::BinarySensor *sens) { this->connected_sensor_ = sens; }
  void set_power_limit_sensor(sensor::Sensor *sens) { this->power_limit_sensor_ = sens; }
  void set_ski_sensor(text_sensor::TextSensor *sens) { this->ski_sensor_ = sens; }
  void set_ems_sensor(text_sensor::TextSensor *sens) { this->ems_sensor_ = sens; }

  void record_action(const std::string &action);

  // OpenAmber bridge (wired from YAML lambdas that can access id(...)).
  void set_read_model(const std::function<std::string()> &fn) { this->read_model_ = fn; }
  void set_read_power(const std::function<float()> &fn) { this->read_power_ = fn; }
  void set_read_dhw_temp(const std::function<float()> &fn) { this->read_dhw_temp_ = fn; }
  void set_read_power_estimate(const std::function<float()> &fn) { this->read_power_estimate_ = fn; }
  void set_read_voltage(const std::function<float()> &fn) { this->read_voltage_ = fn; }
  void set_read_current(const std::function<float()> &fn) { this->read_current_ = fn; }
  void set_read_energy(const std::function<float()> &fn) { this->read_energy_ = fn; }
  void set_is_boost_active(const std::function<bool()> &fn) { this->is_boost_active_ = fn; }
  void set_is_enabled(const std::function<bool()> &fn) { this->is_enabled_ = fn; }
  void set_apply_optional(const std::function<bool(bool)> &fn) { this->apply_optional_ = fn; }
  void set_apply_dim(const std::function<bool(bool)> &fn) { this->apply_dim_ = fn; }
  void set_apply_limit(const std::function<bool(bool, float)> &fn) { this->apply_limit_ = fn; }

  bool is_enabled() const { return this->is_enabled_ ? this->is_enabled_() : true; }
  void start_eebus_subsystem();
  void stop_eebus_subsystem();

  void setup() override;
  void loop() override;
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
  bool mdns_advertised_{false};
  uint32_t last_mdns_reannounce_ms_{0};
  uint32_t last_measurement_notify_ms_{0};
  uint32_t last_ohpcf_notify_ms_{0};
  uint32_t last_use_case_notify_ms_{0};
  uint32_t last_boost_start_ms_{0};
  float last_power_estimate_w_{0.0f};

  text_sensor::TextSensor *status_sensor_{nullptr};
  text_sensor::TextSensor *last_action_sensor_{nullptr};
  binary_sensor::BinarySensor *connected_sensor_{nullptr};
  sensor::Sensor *power_limit_sensor_{nullptr};
  text_sensor::TextSensor *ski_sensor_{nullptr};
  text_sensor::TextSensor *ems_sensor_{nullptr};
  std::string last_action_{"Initialisatie"};
  std::string current_status_{"Initialisatie"};
  std::string current_ems_{"–"};
  bool last_connected_{false};

  // Thread-safe dispatch from ship_accept_task to main loop
  std::string pending_ems_state_;
  bool pending_ems_update_{false};
  std::string pending_action_state_;
  bool pending_action_update_{false};
  std::string pending_status_state_;
  bool pending_status_update_{false};
  bool pending_connected_state_{false};
  bool pending_connected_update_{false};
  float pending_limit_state_{NAN};
  bool pending_limit_update_{false};
  bool pending_remote_changed_{false};

  EEBusCertificateStore cert_store_;
  EEBusShipListener ship_listener_;
  OhpcfServer ohpcf_;
  LpcServer lpc_;
  MpcServer mpc_;
  MdtServer mdt_;
  EebusNode node_;

  std::function<std::string()> read_model_{nullptr};
  std::function<float()> read_power_;
  std::function<float()> read_dhw_temp_;
  std::function<float()> read_power_estimate_;
  std::function<float()> read_voltage_;
  std::function<float()> read_current_;
  std::function<float()> read_energy_;
  std::function<bool()> is_boost_active_;
  std::function<bool()> is_enabled_;
  std::function<bool(bool)> apply_optional_;
  std::function<bool(bool)> apply_dim_;
  std::function<bool(bool, float)> apply_limit_;
  bool initialized_{false};
  bool model_detected_{false};

  void init_certificate_and_keys();
  void start_ship_listener();
  bool advertise_mdns();
  void wire_use_cases();
  void check_failsafe();
  void check_connection_state();
  void update_ems_status();
};

void eebus_disconnect();

}  // namespace openamber_eebus
}  // namespace esphome
