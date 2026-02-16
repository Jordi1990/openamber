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

#include "esphome.h"
#include "constants.h"

using namespace esphome;

class CompressorController
{
private:
  uint32_t last_compressor_start_ms_ = 0;
  uint32_t last_compressor_stop_ms_ = 0;
  uint32_t last_compressor_mode_change_ms_ = 0;

public:
  CompressorController() {}

  uint32_t GetStartTime()
  {
    return last_compressor_start_ms_;
  }

  void ApplyCompressorMode(int compressor_mode)
  {
    auto current_compressor_mode = id(compressor_control_select).active_index().value();

    // On initial start, allow any mode.
    if(current_compressor_mode != 0)
    {
      // Limit to a single frequency step per update
      if (compressor_mode > current_compressor_mode + 1)
        compressor_mode = current_compressor_mode + 1;
      if (compressor_mode < current_compressor_mode - 1)
        compressor_mode = current_compressor_mode - 1;

      if (current_compressor_mode == compressor_mode)
      {
        return;
      }

      const uint32_t min_interval = (current_compressor_mode > compressor_mode ? FREQUENCY_CHANGE_INTERVAL_DOWN_S : FREQUENCY_CHANGE_INTERVAL_UP_S) * 1000;
      if ((millis() - last_compressor_mode_change_ms_) < min_interval)
      {
        ESP_LOGI("amber", "Frequency change skipped (interval not elapsed)");
        return;
      }
    }

    auto compressor_set_call = id(compressor_control_select).make_call();
    compressor_set_call.set_index(compressor_mode);
    compressor_set_call.perform();
    ESP_LOGI("amber", "Applying compressor mode change to %d", compressor_mode);
    last_compressor_mode_change_ms_ = millis();
  }

  void Stop()
  {
    last_compressor_start_ms_ = 0;
    last_compressor_stop_ms_ = millis();
    auto compressor_set_call = id(compressor_control_select).make_call();
    compressor_set_call.select_first();
    compressor_set_call.perform();
  }

  bool IsRunning()
  {
    return id(current_compressor_frequency).state > 0;
  }

  bool HasPassedMinOnTime()
  {
    const uint32_t min_on_ms = COMPRESSOR_MIN_ON_S * 1000UL;
    return (millis() - last_compressor_start_ms_) > min_on_ms;
  }

  bool HasPassedMinOffTime()
  {
    const uint32_t min_off_ms = COMPRESSOR_MIN_OFF_S * 1000UL;
    return millis() >= last_compressor_stop_ms_ + min_off_ms;
  }

  bool HasPassedSoftStartDuration()
  {
    return millis() - last_compressor_start_ms_ >= COMPRESSOR_SOFT_START_DURATION_S * 1000UL;
  }

  void RecordStartTime()
  {
    last_compressor_start_ms_ = millis();
  }

  void ApplyDefrostRecoveryMode()
  {
    auto current_compressor_mode = id(compressor_control_select).active_index().value();
    // Increase compressor mode by 3 steps to speed up recovery after defrost
    int defrost_recovery_mode = current_compressor_mode + 3;
    auto compressor_set_call = id(compressor_control_select).make_call();
    int mode_offset = id(compressor_control_select).size() - id(heat_compressor_mode).size();
    int capped_mode_index = std::min(defrost_recovery_mode, (int)id(heat_compressor_mode).active_index().value() + mode_offset);
    compressor_set_call.set_index(capped_mode_index);
    compressor_set_call.perform();
    last_compressor_mode_change_ms_ = millis();
  }
};
