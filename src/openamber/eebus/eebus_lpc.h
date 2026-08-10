/*
 * Open Amber - native EEBus PoC
 *
 * LPC server (Energy Guard: Load Power Control).  This is the device side of
 * the §14a / consumption-limit use case that evcc's eebus-ohpcf driver writes
 * to via Dim().
 *
 * evcc writes a fixed 0 W active limit when dimming.  The device maps "active
 * 0 W limit" on to its SG-Ready block / compressor-stop path, subject to hard
 * safety overrides (frost protection, errors).
 */

#pragma once

#include "esphome/core/component.h"
#include <cstdint>
#include <functional>

namespace esphome {
namespace openamber_eebus {

class LpcServer {
 public:
  using Dimmer = std::function<void(bool dim)>;

  LpcServer() = default;

  void set_dimmer(Dimmer dimmer) { this->dimmer_ = std::move(dimmer); }

  // WriteConsumptionLimit from the CEM.
  void write_limit(float value_w, bool active);
  void release_limit();

  bool is_dimmed() const { return this->limit_active_; }
  float get_limit_value_w() const { return this->limit_value_w_; }

 private:
  void apply();

  bool limit_active_{false};
  float limit_value_w_{0.0f};
  Dimmer dimmer_;
};

}  // namespace openamber_eebus
}  // namespace esphome
