/*
 * Open Amber - native EEBus PoC
 *
 * LPC server implementation.
 */

#include "eebus_lpc.h"
#include "esphome/core/log.h"

namespace esphome {
namespace openamber_eebus {

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
  if (this->dimmer_) {
    this->dimmer_(this->limit_active_);
  }
}

}  // namespace openamber_eebus
}  // namespace esphome
