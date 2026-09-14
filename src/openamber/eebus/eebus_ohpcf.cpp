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
  this->requested_power_w_ = power_w > 0.0f ? power_w : 2500.0f;
  this->requested_power_max_w_ = power_w > 0.0f ? std::max(3000.0f, power_w * 1.5f) : 3500.0f;
  this->min_run_s_ = min_run_s;
  this->min_pause_s_ = min_pause_s;
  // A fresh announcement returns to the available state.
  this->set_state(OhpcfState::AVAILABLE);
  ESP_LOGI("eebus_ohpcf", "Announced optional consumption %.0f W (max %.0f W, min run %us)",
           this->requested_power_w_, this->requested_power_max_w_, static_cast<unsigned>(min_run_s));
}

void OhpcfServer::set_requested_power(float estimate_w, float max_w) {
  if (estimate_w > 0.0f) {
    this->requested_power_w_ = estimate_w;
  } else {
    this->requested_power_w_ = 2500.0f;
  }
  if (max_w > 0.0f) {
    this->requested_power_max_w_ = max_w;
  } else {
    this->requested_power_max_w_ = std::max(3000.0f, this->requested_power_w_ * 1.5f);
  }
}

void OhpcfServer::schedule(uint32_t start_delta_s) {
  ESP_LOGI("eebus_ohpcf", "Scheduled optional consumption (start delta %us, current state %d)",
           static_cast<unsigned>(start_delta_s), static_cast<int>(this->state_));
  this->set_state(OhpcfState::RUNNING);
  this->apply();
}

void OhpcfServer::resume() {
  ESP_LOGI("eebus_ohpcf", "Resume optional consumption (current state %d)", static_cast<int>(this->state_));
  this->set_state(OhpcfState::RUNNING);
  this->apply();
}

void OhpcfServer::pause() {
  ESP_LOGI("eebus_ohpcf", "Pause optional consumption (current state %d)", static_cast<int>(this->state_));
  this->set_state(OhpcfState::PAUSED);
  this->apply();
}

void OhpcfServer::abort() {
  ESP_LOGI("eebus_ohpcf", "Abort optional consumption (current state %d)", static_cast<int>(this->state_));
  this->set_state(OhpcfState::AVAILABLE);
  this->apply();
}

void OhpcfServer::set_state(OhpcfState state) {
  if (this->state_ != state) {
    this->state_ = state;
    ESP_LOGD("eebus_ohpcf", "ConsumptionState -> %d", static_cast<int>(state));
    if (this->change_cb_) {
      this->change_cb_(state);
    }
  }
}

void OhpcfServer::apply() {
  if (this->applier_) {
    bool wanted = (this->state_ == OhpcfState::RUNNING);
    bool accepted = this->applier_(wanted);
    if (wanted && !accepted) {
      ESP_LOGW("eebus_ohpcf", "Optional consumption request rejected by OpenAmber (conditions not met)");
      this->set_state(OhpcfState::AVAILABLE);
    }
  }
}

}  // namespace openamber_eebus
}  // namespace esphome
