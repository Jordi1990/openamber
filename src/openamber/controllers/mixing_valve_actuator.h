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
  esphome::number::Number* output_number_ = nullptr;
  esphome::number::Number* min_position_ = nullptr;
  esphome::number::Number* max_position_ = nullptr;
  float pid_output_ = 0.0f;  // PID output [0..1] -> valve position

public:
  MixingValveActuator() = default;

  MixingValveActuator(esphome::number::Number& output_number,
                      esphome::number::Number& min_position,
                      esphome::number::Number& max_position)
    : output_number_(&output_number), min_position_(&min_position), max_position_(&max_position)
  {
  }

  MixingValveActuator(esphome::number::Number* output_number,
                      esphome::number::Number* min_position,
                      esphome::number::Number* max_position)
    : output_number_(output_number), min_position_(min_position), max_position_(max_position)
  {
  }

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
  float GetClampedValvePositionPercent() const
  {
    float raw_percent = GetValvePositionPercent();
    if (min_position_ && max_position_)
    {
      return std::max(min_position_->state, std::min(max_position_->state, raw_percent));
    }
    return raw_percent;
  }

  /// Translates PID output (0.0–1.0) to a clamped modbus value (0–100)
  /// corresponding to 0-10V on the mixing valve actuator (0 = 0%, 100 = 100%).
  /// The output is clamped between min_percent and max_percent.
  /// 0 = valve fully closed (bypass/return water only)
  /// 100 = valve fully open (maximum hot supply water)
  int GetClampedModbusValue() const
  {
    return static_cast<int>(roundf(GetClampedValvePositionPercent()));
  }

  /// Writes the current valve position to the modbus register,
  /// clamped within the configured min/max range.
  void ApplyValvePosition()
  {
    int value = GetClampedModbusValue();
    if (static_cast<int>(output_number_->state) != value)
    {
      ESP_LOGI("amber", "Mixing valve position: %d", value);
      auto call = output_number_->make_call();
      call.set_value(value);
      call.perform();
    }
  }

  /// Opens the valve actuator to its maximum position (e.g. during cooling mode)
  void OpenValve()
  {
    float max_percent = max_position_->state;
    pid_output_ = std::max(0.0f, std::min(1.0f, max_percent / 100.0f));
    int max_value = static_cast<int>(roundf(max_percent));
    if (static_cast<int>(output_number_->state) != max_value)
    {
      ESP_LOGI("amber", "Opening mixing valve to maximum (%.0f%%)", max_percent);
      auto call = output_number_->make_call();
      call.set_value(max_value);
      call.perform();
    }
  }

  /// Closes the valve actuator to its minimum position
  void CloseValve()
  {
    pid_output_ = 0.0f;
    int min_value = static_cast<int>(roundf(min_position_->state));
    if (static_cast<int>(output_number_->state) != min_value)
    {
      ESP_LOGI("amber", "Closing mixing valve to minimum (%.0f%%)", min_value);
      auto call = output_number_->make_call();
      call.set_value(min_value);
      call.perform();
    }
  }
};
