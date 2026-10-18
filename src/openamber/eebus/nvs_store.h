/*
 * Open Amber - native EEBus PoC
 *
 * Thin wrapper over ESP-IDF NVS for the EEBus persistent state
 * (certificate/keys, paired SKIs).
 */

#pragma once

#include "esphome/core/component.h"
#include <string>

namespace esphome {
namespace eebus {

class EEBusNvsStore {
 public:
  bool write_string(const char *key, const std::string &value);
  bool read_string(const char *key, std::string &out);
};

}  // namespace eebus
}  // namespace esphome
