/*
 * Open Amber - native EEBus PoC
 *
 * MPC / MDT server data (Monitoring Appliance):
 *  - MPC: measured electrical power consumption (the "integrated meter").
 *  - MDT: domestic hot water temperature.
 *
 * These are the *device* values that evcc's eebus-ohpcf driver reads via
 * CurrentPower() and Soc().  In this PoC they are simple value holders
 * refreshed from the OpenAmber bridge every update() tick.
 */

#pragma once

#include "esphome/core/component.h"

namespace esphome {
namespace openamber_eebus {

class MpcServer {
 public:
  void set_power_w(float w) { this->power_w_ = w; }
  float get_power_w() const { return this->power_w_; }

 private:
  float power_w_{0.0f};
};

class MdtServer {
 public:
  void set_temperature_c(float c) { this->temperature_c_ = c; }
  float get_temperature_c() const { return this->temperature_c_; }

 private:
  float temperature_c_{0.0f};
};

}  // namespace openamber_eebus
}  // namespace esphome
