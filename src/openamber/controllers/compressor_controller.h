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

#include <cmath>

#include "esphome.h"
#include "constants.h"

using namespace esphome;


class CompressorController
{
private:
  uint32_t last_compressor_start_ms_ = 0;
  uint32_t last_compressor_stop_ms_ = 0;
  uint32_t last_compressor_mode_change_ms_ = 0;
  uint32_t start_wait_started_ms_ = 0;
  uint32_t last_discharge_limit_step_ms_ = 0;
  bool discharge_limit_active_ = false;

  /// @brief Returns the outdoor unit's threshold when it was read and is plausible, otherwise the default.
  static float GetUnitLimit(float value, float fallback, float min_value, float max_value)
  {
    if (std::isnan(value) || value < min_value || value > max_value)
    {
      return fallback;
    }
    return value;
  }

  /// @brief Keeps Pd and Td below the outdoor unit's own protection: hold the level when close,
  /// step down one level per DISCHARGE_LIMIT_STEP_DOWN_INTERVAL_S when closer.
  int ApplyDischargeSideLimit(int compressor_mode, int current_compressor_mode)
  {
    if (current_compressor_mode <= 0)
    {
      if (discharge_limit_active_)
      {
        discharge_limit_active_ = false;
        id(discharge_side_limit_active_sensor).publish_state(false);
      }
      return compressor_mode;
    }

    const float pd = id(discharge_pressure_pd).state;
    const float td = id(discharge_temperature_td).state;
    const bool pd_valid = !std::isnan(pd) && pd > 0.0f;
    const bool td_valid = !std::isnan(td) && td > 0.0f;
    if (!pd_valid && !td_valid)
    {
      // No readings: keep the current cap while latched instead of letting the level rise.
      return discharge_limit_active_ ? std::min(compressor_mode, current_compressor_mode) : compressor_mode;
    }

    const float unit_pd = GetUnitLimit(id(outdoor_unit_pd_no_increase_limit).state, DISCHARGE_LIMIT_DEFAULT_UNIT_PD_BAR, 15.0f, 45.0f);
    const float unit_td = GetUnitLimit(id(outdoor_unit_td_no_increase_limit).state, DISCHARGE_LIMIT_DEFAULT_UNIT_TD_C, 70.0f, 130.0f);

    const bool step_down = (pd_valid && pd >= unit_pd - DISCHARGE_LIMIT_STEP_DOWN_PD_MARGIN_BAR) || (td_valid && td >= unit_td - DISCHARGE_LIMIT_STEP_DOWN_TD_MARGIN_C);
    const bool hold = (pd_valid && pd >= unit_pd - DISCHARGE_LIMIT_HOLD_PD_MARGIN_BAR) || (td_valid && td >= unit_td - DISCHARGE_LIMIT_HOLD_TD_MARGIN_C);
    const bool released = (!pd_valid || pd < unit_pd - DISCHARGE_LIMIT_RELEASE_PD_MARGIN_BAR) && (!td_valid || td < unit_td - DISCHARGE_LIMIT_RELEASE_TD_MARGIN_C);

    if (step_down || hold)
    {
      if (!discharge_limit_active_)
      {
        ESP_LOGI("amber", "Discharge side limiter active (Pd %.1f bar, Td %.1f°C).", pd, td);
      }
      discharge_limit_active_ = true;
    }
    else if (discharge_limit_active_ && released)
    {
      ESP_LOGI("amber", "Discharge side limiter released (Pd %.1f bar, Td %.1f°C).", pd, td);
      discharge_limit_active_ = false;
    }
    id(discharge_side_limit_active_sensor).publish_state(discharge_limit_active_);

    if (!discharge_limit_active_)
    {
      return compressor_mode;
    }

    const uint32_t now = App.get_loop_component_start_time();
    if (step_down && current_compressor_mode > DISCHARGE_LIMIT_MIN_MODE &&
        (now - last_discharge_limit_step_ms_) >= DISCHARGE_LIMIT_STEP_DOWN_INTERVAL_S * 1000UL &&
        (now - last_compressor_mode_change_ms_) >= FREQUENCY_CHANGE_INTERVAL_DOWN_S * 1000UL)
    {
      last_discharge_limit_step_ms_ = now;
      ESP_LOGI("amber", "Discharge side limiter: stepping down to %d (Pd %.1f bar, Td %.1f°C).", current_compressor_mode - 1, pd, td);
      return std::min(compressor_mode, current_compressor_mode - 1);
    }

    // Close to the limit, or between two step-downs: never go up.
    return std::min(compressor_mode, current_compressor_mode);
  }

public:
  CompressorController() {}

  uint32_t GetStartTime()
  {
    return last_compressor_start_ms_;
  }

  void ApplyCompressorMode(int compressor_mode)
  {
    auto current_compressor_mode = id(compressor_control_select).active_index().value();
    compressor_mode = ApplyDischargeSideLimit(compressor_mode, current_compressor_mode);

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
      if ((App.get_loop_component_start_time() - last_compressor_mode_change_ms_) < min_interval)
      {
        ESP_LOGI("amber", "Frequency change skipped (interval not elapsed)");
        return;
      }
    }

    auto compressor_set_call = id(compressor_control_select).make_call();
    compressor_set_call.set_index(compressor_mode);
    compressor_set_call.perform();
    ESP_LOGI("amber", "Applying compressor mode change to %d", compressor_mode);
    last_compressor_mode_change_ms_ = App.get_loop_component_start_time();
  }

  void Stop()
  {
    ClearStartWaitTimer();
    last_compressor_start_ms_ = 0;
    last_compressor_stop_ms_ = App.get_loop_component_start_time();
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
    return (App.get_loop_component_start_time() - last_compressor_start_ms_) > min_on_ms;
  }

  bool HasPassedMinOffTime()
  {
    const uint32_t min_off_ms = COMPRESSOR_MIN_OFF_S * 1000UL;
    return (App.get_loop_component_start_time() - last_compressor_stop_ms_) > min_off_ms;
  }

  bool HasPassedSoftStartDuration()
  {
    return (App.get_loop_component_start_time() - last_compressor_start_ms_) >= COMPRESSOR_SOFT_START_DURATION_S * 1000UL;
  }

  void RecordStartTime()
  {
    ClearStartWaitTimer();
    last_compressor_start_ms_ = App.get_loop_component_start_time();
  }

  void ArmStartWaitTimer()
  {
    start_wait_started_ms_ = App.get_loop_component_start_time();
  }

  void ClearStartWaitTimer()
  {
    start_wait_started_ms_ = 0;
  }

  uint32_t GetStartWaitStartedTime()
  {
    return start_wait_started_ms_;
  }

  void ApplyDefrostRecoveryMode()
  {
    auto current_compressor_mode = id(compressor_control_select).active_index().value();
    // Increase compressor mode by 3 steps to speed up recovery after defrost
    int defrost_recovery_mode = current_compressor_mode + 3;
    auto compressor_set_call = id(compressor_control_select).make_call();
    int mode_offset = id(compressor_control_select).size() - id(heat_compressor_mode).size();
    int capped_mode_index = std::min(defrost_recovery_mode, (int)id(heat_compressor_mode).active_index().value() + mode_offset);
    if (id(silent_mode_active_sensor).state)
    {
      int silent_offset = id(compressor_control_select).size() - id(silent_mode_compressor_mode).size();
      int silent_limit = (int)id(silent_mode_compressor_mode).active_index().value_or(3) + silent_offset;
      capped_mode_index = std::min(capped_mode_index, silent_limit);
    }
    capped_mode_index = ApplyDischargeSideLimit(capped_mode_index, current_compressor_mode);
    if (capped_mode_index == (int)current_compressor_mode)
    {
      return;
    }
    compressor_set_call.set_index(capped_mode_index);
    compressor_set_call.perform();
    last_compressor_mode_change_ms_ = App.get_loop_component_start_time();
  }
};
