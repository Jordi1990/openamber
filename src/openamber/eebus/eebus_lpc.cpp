/*
 * Open Amber - native EEBus PoC
 *
 * LPC server implementation.
 */


#include "eebus_lpc.h"
#include "esphome/core/log.h"

namespace esphome {
namespace eebus {

void LpcServer::write_limit(float value_w, bool active) {
  this->limit_value_w_ = value_w;
  this->limit_active_ = active;
  ESP_LOGI("eebus_lpc", "Consumption limit %s (%.0f W)", active ? "ACTIVE" : "inactive",
           value_w);
  this->apply();
}

void LpcServer::release_limit() {
  this->limit_active_ = false;
  this->apply();
}

void LpcServer::apply() {
  if (this->limit_applier_) {
    this->limit_applier_(this->limit_active_, this->limit_value_w_);
  }
  if (this->dimmer_) {
    bool dim_active = this->limit_active_ && (this->limit_value_w_ <= 0.0f);
    this->dimmer_(dim_active);
  }
  if (this->change_cb_) {
    this->change_cb_(this->limit_active_, this->limit_value_w_);
  }
}

}  // namespace eebus
}  // namespace esphome

