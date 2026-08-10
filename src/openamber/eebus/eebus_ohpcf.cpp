/*
 * Open Amber - native EEBus PoC
 *
 * OHPCF server state machine.
 */

#include "eebus_ohpcf.h"
#include "esphome/core/log.h"

namespace esphome {
namespace openamber_eebus {

void OhpcfServer::announce(float power_w, uint32_t min_run_s, uint32_t min_pause_s) {
  this->requested_power_w_ = power_w;
  this->requested_power_max_w_ = power_w;
  this->min_run_s_ = min_run_s;
  this->min_pause_s_ = min_pause_s;
  // A fresh announcement returns to the available state.
  this->set_state(OhpcfState::AVAILABLE);
  ESP_LOGI("eebus_ohpcf", "Announced optional consumption %.0f W (min run %us)",
           power_w, static_cast<unsigned>(min_run_s));
}

void OhpcfServer::schedule(uint32_t start_delta_s) {
  if (this->state_ != OhpcfState::AVAILABLE)
    return;
  ESP_LOGI("eebus_ohpcf", "Scheduled optional consumption (start delta %us)",
           static_cast<unsigned>(start_delta_s));
  // PoC: start_delta_s==0 means start immediately; extended scheduling with an
  // async start timer is added together with the SPINE layer.
  this->set_state(OhpcfState::RUNNING);
  this->apply();
}

void OhpcfServer::resume() {
  if (this->state_ == OhpcfState::PAUSED) {
    this->set_state(OhpcfState::RUNNING);
    this->apply();
  }
}

void OhpcfServer::pause() {
  if (this->state_ == OhpcfState::RUNNING && this->is_pausable_) {
    this->set_state(OhpcfState::PAUSED);
    this->apply();
  }
}

void OhpcfServer::abort() {
  if (this->state_ == OhpcfState::RUNNING || this->state_ == OhpcfState::SCHEDULED) {
    this->set_state(OhpcfState::AVAILABLE);
    this->apply();
  }
}

void OhpcfServer::set_state(OhpcfState state) {
  this->state_ = state;
  ESP_LOGD("eebus_ohpcf", "ConsumptionState -> %d", static_cast<int>(state));
}

void OhpcfServer::apply() {
  if (this->applier_) {
    // The OpenAmber bridge decides whether the optional demand is actually
    // granted (safety/frost/min-on-time take precedence).  applier receives
    // the requested on/off; the bridge returns the granted result via a later
    // state reconciliation, so keep the local state authoritative only if the
    // request was honoured.
    this->applier_(this->state_ == OhpcfState::RUNNING);
  }
}

}  // namespace openamber_eebus
}  // namespace esphome
