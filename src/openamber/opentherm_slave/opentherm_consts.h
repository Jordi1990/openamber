/*
 * Open Amber - Itho Daalderop Amber heat pump controller for ESPHome
 *
 * Copyright (C) 2025 Jordi Epema
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <cstdint>

#ifdef USE_ESP32
#include "esp_attr.h"
#endif
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

namespace esphome {
namespace opentherm_slave {

enum class MessageType : uint8_t {
  READ_DATA = 0b000,
  WRITE_DATA = 0b001,
  INVALID_DATA = 0b010,
  READ_ACK = 0b100,
  WRITE_ACK = 0b101,
  DATA_INVALID = 0b110,
  UNKNOWN_DATAID = 0b111,
};

enum class MessageId : uint8_t {
  STATUS = 0,
  CH_SETPOINT = 1,
  MASTER_CONFIG = 2,
  SLAVE_CONFIG = 3,
  FAULT_FLAGS = 5,
  MAX_REL_MOD_LEVEL_SETTING = 14,
  ROOM_SETPOINT = 16,
  REL_MOD_LEVEL = 17,
  CH_WATER_PRESSURE = 18,
  ROOM_TEMP = 24,
  FLOW_TEMP = 25,
  DHW_TEMP = 26,
  OUTSIDE_TEMP = 27,
  RETURN_TEMP = 28,
  DHW_SETPOINT = 56,
  MAX_CH_SETPOINT = 57,
};

struct OpenthermData {
  MessageType type;
  uint8_t id;
  uint8_t value_hb;
  uint8_t value_lb;

  float f88() const {
    return static_cast<float>(static_cast<int8_t>(value_hb)) +
           static_cast<float>(value_lb) / 256.0f;
  }

  void f88(float value) {
    if (value >= 0) {
      value_hb = static_cast<uint8_t>(static_cast<int>(value));
      value_lb = static_cast<uint8_t>((value - static_cast<int>(value)) * 256.0f);
    } else {
      int int_part = static_cast<int>(value) - 1;
      float frac = value - int_part;
      value_hb = static_cast<uint8_t>(static_cast<int8_t>(int_part));
      value_lb = static_cast<uint8_t>(frac * 256.0f);
    }
  }

  uint16_t u16() const {
    return (static_cast<uint16_t>(value_hb) << 8) | value_lb;
  }

  void u16(uint16_t value) {
    value_hb = static_cast<uint8_t>(value >> 8);
    value_lb = static_cast<uint8_t>(value & 0xFF);
  }

  int16_t s16() const { return static_cast<int16_t>(u16()); }

  void s16(int16_t value) { u16(static_cast<uint16_t>(value)); }
};

struct MasterStatus {
  bool ch_enable;
  bool dhw_enable;
  bool cooling_enable;
  bool otc_active;
  bool ch2_active;

  static MasterStatus from_hb(uint8_t hb) {
    return {
        .ch_enable = static_cast<bool>(hb & 0x01),
        .dhw_enable = static_cast<bool>(hb & 0x02),
        .cooling_enable = static_cast<bool>(hb & 0x04),
        .otc_active = static_cast<bool>(hb & 0x08),
        .ch2_active = static_cast<bool>(hb & 0x10),
    };
  }
};

struct SlaveStatus {
  bool fault;
  bool ch_active;
  bool dhw_active;
  bool flame_on;
  bool cooling_active;
  bool ch2_active;
  bool diagnostic;

  uint8_t to_lb() const {
    uint8_t lb = 0;
    if (fault) lb |= 0x01;
    if (ch_active) lb |= 0x02;
    if (dhw_active) lb |= 0x04;
    if (flame_on) lb |= 0x08;
    if (cooling_active) lb |= 0x10;
    if (ch2_active) lb |= 0x20;
    if (diagnostic) lb |= 0x40;
    return lb;
  }
};

inline IRAM_ATTR bool parity(uint32_t frame) {
  frame ^= frame >> 16;
  frame ^= frame >> 8;
  frame ^= frame >> 4;
  frame ^= frame >> 2;
  frame ^= frame >> 1;
  return frame & 1;
}

}  // namespace opentherm_slave
}  // namespace esphome
