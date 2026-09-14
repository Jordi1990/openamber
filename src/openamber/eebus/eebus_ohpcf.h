/*
 * Open Amber - native EEBus PoC
 *
 * OHPCF server (Optimization of Self-Consumption by Heat Pump Compressor
 * Flexibility).  This is the *device/compressor* side of the use case that
 * evcc's eebus-ohpcf driver controls.
 *
 * evcc models this as an on/off "optional consumption": it schedules/resumes
 * to enable and pauses/aborts to disable.  The device owns the actual
 * decision (how much optional power it can take, within min run/pause
 * durations and safety constraints).
 */

#pragma once

#include "esphome/core/component.h"
#include <cstdint>
#include <functional>

namespace esphome {
namespace openamber_eebus {

enum class OhpcfState : uint8_t {
  AVAILABLE,  // optional consumption announced, not scheduled
  SCHEDULED,  // scheduled (starts at ConsumptionStartTime)
  RUNNING,    // currently consuming optional power
  PAUSED,     // paused by the CEM
};

class OhpcfServer {
 public:
  // Callbacks driven by the OpenAmber bridge / integration layer.
  using Applier = std::function<bool(bool enabled)>;
  using ChangeCallback = std::function<void(OhpcfState state)>;

  OhpcfServer() = default;

  void set_applier(Applier applier) { this->applier_ = std::move(applier); }
  void set_change_callback(ChangeCallback cb) { this->change_cb_ = std::move(cb); }

  // Data exposed to the CEM.
  float get_requested_power_estimate_w() const { return this->requested_power_w_; }
  float get_requested_power_max_w() const { return this->requested_power_max_w_; }
  bool get_consumption_is_stoppable() const { return this->is_stoppable_; }
  bool get_consumption_is_pausable() const { return this->is_pausable_; }
  uint32_t get_minimal_run_duration_s() const { return this->min_run_s_; }
  uint32_t get_minimal_pause_duration_s() const { return this->min_pause_s_; }
  OhpcfState get_state() const { return this->state_; }

  // A newly announced optional consumption (e.g. DHW boost) is made public so
  // the CEM can schedule it.
  void announce(float power_w, uint32_t min_run_s, uint32_t min_pause_s);
  void set_requested_power(float estimate_w, float max_w = 0.0f);

  // Commands from the CEM (evcc).
  void schedule(uint32_t start_delta_s);
  void resume();
  void pause();
  void abort();

  // Returns current optional draw (0 when not running), to be summed with base
  // load for the MPC meter.
  float get_current_optional_power_w() const {
    return this->state_ == OhpcfState::RUNNING ? this->requested_power_w_ : 0.0f;
  }

 private:
  void set_state(OhpcfState state);
  void apply();

  OhpcfState state_{OhpcfState::AVAILABLE};
  float requested_power_w_{0.0f};
  float requested_power_max_w_{0.0f};
  bool is_stoppable_{true};
  bool is_pausable_{true};
  uint32_t min_run_s_{600};
  uint32_t min_pause_s_{600};
  Applier applier_;
  ChangeCallback change_cb_;
};

}  // namespace openamber_eebus
}  // namespace esphome
