#pragma once

#include "esphome/core/component.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include <functional>
#include <string>
#include <cmath>

namespace esphome {
namespace eebus {

/**
 * Headless mock implementation of the EEBus component for host simulation and testing.
 * Provides simulated EMS pairing, SG-Ready/§14a boost, dimming, and limit flows without
 * hardware dependencies (mbedTLS, FreeRTOS, ESP-IDF mDNS).
 */
class EEBusComponent : public PollingComponent {
 public:
  EEBusComponent();
  ~EEBusComponent();

  void disconnect_ems();

  void set_device_sku(const std::string &sku) { this->device_sku_ = sku; }
  void set_brand(const std::string &brand) { this->brand_ = brand; }
  void set_model(const std::string &model) { this->model_ = model; }
  void set_device_name(const std::string &name) { this->device_name_ = name; }
  void set_device_category(const std::string &cat) { this->device_category_ = cat; }
  void set_device_type(const std::string &type) { this->device_type_ = type; }
  void set_entity_type(const std::string &type) { this->entity_type_ = type; }
  void set_enable_ohpcf(bool enable) { this->enable_ohpcf_ = enable; }
  void set_enable_mdt(bool enable) { this->enable_mdt_ = enable; }
  void set_failsafe_duration(uint32_t seconds) { this->failsafe_duration_s_ = seconds; }

  // Diagnostic sensors for Home Assistant
  void set_status_sensor(text_sensor::TextSensor *sens) { this->status_sensor_ = sens; }
  void set_last_action_sensor(text_sensor::TextSensor *sens) { this->last_action_sensor_ = sens; }
  void set_connected_sensor(binary_sensor::BinarySensor *sens) { this->connected_sensor_ = sens; }
  void set_power_limit_sensor(sensor::Sensor *sens) { this->power_limit_sensor_ = sens; }
  void set_ski_sensor(text_sensor::TextSensor *sens) { this->ski_sensor_ = sens; }
  void set_ems_sensor(text_sensor::TextSensor *sens) { this->ems_sensor_ = sens; }

  void record_action(const std::string &action);

  // Host application bridge
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

  float get_dhw_temperature_c() const { return 0.0f; }
  float get_power_w() const { return 0.0f; }

  void fake_request_boost(bool active);
  void fake_request_dim(bool active);
  void fake_set_power_limit(float limit_w);
  void fake_set_connected(bool connected, const std::string &ems_name = "EVCC (192.168.1.100)");

 private:
  std::string device_sku_{"EEBUS-Device"};
  std::string brand_{"Generic"};
  std::string model_{"Device"};
  std::string device_name_{""};
  std::string device_category_{"heatPump"};
  std::string device_type_{"HeatGenerationSystem"};
  std::string entity_type_{"heatPumpAppliance"};
  bool enable_ohpcf_{true};
  bool enable_mdt_{true};
  uint32_t failsafe_duration_s_{7200};

  text_sensor::TextSensor *status_sensor_{nullptr};
  text_sensor::TextSensor *last_action_sensor_{nullptr};
  binary_sensor::BinarySensor *connected_sensor_{nullptr};
  sensor::Sensor *power_limit_sensor_{nullptr};
  text_sensor::TextSensor *ski_sensor_{nullptr};
  text_sensor::TextSensor *ems_sensor_{nullptr};
  std::string last_action_{"Initialization"};

  bool last_enabled_{false};
  bool is_boost_requested_{false};
  bool is_dim_requested_{false};

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
};

void eebus_disconnect();
void fake_eebus_boost(bool active);
void fake_eebus_dim(bool active);
void fake_eebus_limit(float limit_w);
void fake_eebus_connected(bool connected, const std::string &ems_name = "EVCC (192.168.1.100)");

}  // namespace eebus

namespace openamber_eebus = eebus;
}  // namespace esphome
