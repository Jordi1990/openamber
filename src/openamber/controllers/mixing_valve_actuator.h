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

#include <algorithm>
#include <cmath>

#include "esphome.h"
#include "constants.h"

using namespace esphome;

class MixingValveActuator
{
private:
  float pid_output_ = 0.0f;  // PID output [0..1] -> valve position

public:
  MixingValveActuator() = default;

  void SetPidOutput(float value)
  {
    pid_output_ = std::max(0.0f, std::min(1.0f, value));
  }

  float GetPidOutput() const { return pid_output_; }

  /// Returns the raw valve position as percentage (0-100%) based on PID output.
  float GetValvePositionPercent() const
  {
    return pid_output_ * 100.0f;
  }

  /// Returns the clamped valve position as percentage (0-100%),
  /// taking min/max settings into account.
  /// @param min_percent Minimum valve position (0-100%)
  /// @param max_percent Maximum valve position (0-100%)
  float GetClampedValvePositionPercent(float min_percent, float max_percent) const
  {
    float raw_percent = GetValvePositionPercent();
    return std::max(min_percent, std::min(max_percent, raw_percent));
  }

  /// Translates PID output (0.0–1.0) to a clamped modbus value (0–100)
  /// corresponding to 0-10V on the mixing valve actuator (0 = 0%, 100 = 100%).
  /// The output is clamped between min_percent and max_percent.
  /// 0 = valve fully closed (bypass/return water only)
  /// 100 = valve fully open (maximum hot supply water)
  int GetClampedModbusValue(float min_percent, float max_percent) const
  {
    float clamped = GetClampedValvePositionPercent(min_percent, max_percent);
    return static_cast<int>(roundf(clamped));
  }

  /// Writes the current valve position to the modbus register,
  /// clamped within the configured min/max range.
  /// @param modbus_number The modbus number entity to write to
  /// @param min_percent Minimum valve position setting (0-100%)
  /// @param max_percent Maximum valve position setting (0-100%)
  void ApplyPosition(esphome::number::Number& modbus_number,
                     float min_percent, float max_percent)
  {
    int value = GetClampedModbusValue(min_percent, max_percent);
    if (static_cast<int>(modbus_number.state) != value)
    {
      ESP_LOGI("amber", "Mixing valve position: %.1f%% (modbus: %d, range: %.0f-%.0f%%)", 
               GetClampedValvePositionPercent(min_percent, max_percent), value,
               min_percent, max_percent);
      auto call = modbus_number.make_call();
      call.set_value(value);
      call.perform();
    }
  }

  void ApplyValvePosition(esphome::number::Number& modbus_number,
                          float min_percent, float max_percent)
  {
    ApplyPosition(modbus_number, min_percent, max_percent);
  }

  /// Closes the valve actuator to its minimum position
  /// @param min_percent Minimum valve position setting (0-100%)
  void Close(esphome::number::Number& modbus_number,
             float min_percent)
  {
    pid_output_ = 0.0f;
    int min_value = static_cast<int>(roundf(min_percent));
    if (static_cast<int>(modbus_number.state) != min_value)
    {
      ESP_LOGI("amber", "Closing mixing valve to minimum (%.0f%%)", min_percent);
      auto call = modbus_number.make_call();
      call.set_value(min_value);
      call.perform();
    }
  }

  void CloseValve(esphome::number::Number& modbus_number,
                  float min_percent)
  {
    Close(modbus_number, min_percent);
  }
};
