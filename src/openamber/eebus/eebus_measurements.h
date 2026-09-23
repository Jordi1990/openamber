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
  void set_power_w(float w) {
    this->power_w_ = w;
    uint32_t now = millis();
    if (this->last_energy_calc_ms_ > 0 && now > this->last_energy_calc_ms_) {
      float dt_hours = (now - this->last_energy_calc_ms_) / 3600000.0f;
      if (w > 0.0f) {
        this->energy_wh_ += w * dt_hours;
      }
    }
    this->last_energy_calc_ms_ = now;
  }
  float get_power_w() const { return this->power_w_; }

  void set_energy_wh(float wh) {
    if (wh > this->energy_wh_) {
      this->energy_wh_ = wh;
    }
  }
  float get_energy_wh() const { return this->energy_wh_; }

  void set_voltage_v(float v) { this->voltage_v_ = v; }
  float get_voltage_v() const { return this->voltage_v_; }

  void set_current_a(float a) { this->current_a_ = a; }
  float get_current_a() const { return this->current_a_; }

 private:
  float power_w_{0.0f};
  float energy_wh_{100.0f};
  float voltage_v_{230.0f};
  float current_a_{0.0f};
  uint32_t last_energy_calc_ms_{0};
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
