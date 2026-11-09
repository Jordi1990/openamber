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
  // Start/stop times use 64-bit millis_64() timestamps, which never wrap. A compressor can run
  // or stay off for longer than the ~49.7 days after which a 32-bit elapsed time wraps, which
  // would make it look freshly started or stopped again (min on/off time, soft start).
  uint64_t last_compressor_start_ms_ = 0;
  uint64_t last_compressor_stop_ms_ = 0;
  uint32_t last_compressor_mode_change_ms_ = 0;
  uint32_t start_wait_started_ms_ = 0;
  uint32_t last_discharge_limit_step_ms_ = 0;
  bool discharge_limit_active_ = false;
  int heating_ambient_band_ = -1;
  int cooling_ambient_band_ = -1;
  int last_logged_ambient_limit_ = -1;

  /// @brief Ta offset for a step from one band to another: only a step to a lower, non-zero limit is
  /// delayed. The step to a band outside the operating range (level 1) is never delayed.
  static float StepOffset(const float *max_hz, int from, int to)
  {
    return (max_hz[to] > 0.0f && max_hz[to] < max_hz[from]) ? AMBIENT_LIMIT_TA_OFFSET_C : 0.0f;
  }

  /// @brief Follows the outdoor unit's Ta band with its 1 K hysteresis. A step to a band with a lower limit
  /// needs AMBIENT_LIMIT_TA_OFFSET_C extra, so OpenAmber caps a little later than the stock unit would.
  /// low_restart_c is extra hysteresis to leave band 0 (heating low ambient limit).
  static int UpdateAmbientBand(int band, float ta, const float *bounds, int bound_count, const float *max_hz,
                               float low_restart_c)
  {
    int raw_band = 0;
    while (raw_band < bound_count && ta >= bounds[raw_band])
    {
      raw_band++;
    }

    if (band < 0)
    {
      return raw_band;
    }

    while (band < raw_band)
    {
      const float extra = StepOffset(max_hz, band, band + 1) + (band == 0 ? low_restart_c : 0.0f);
      if (ta < bounds[band] + OUTDOOR_UNIT_BAND_HYSTERESIS_C + extra)
      {
        break;
      }
      band++;
    }
    while (band > raw_band)
    {
      const float extra = StepOffset(max_hz, band, band - 1);
      if (ta >= bounds[band - 1] - OUTDOOR_UNIT_BAND_HYSTERESIS_C - extra)
      {
        break;
      }
      band--;
    }
    return band;
  }

  /// @brief Maximum compressor frequency for the current outdoor temperature, with the fixed Ta bands and
  /// limits of the outdoor unit (firmware v1.18). 0 outside the operating range, NaN when Ta is unknown.
  float GetAmbientMaxFrequency(bool cooling)
  {
    const float ta = id(temperature_outside_ta).state;
    if (std::isnan(ta))
    {
      return NAN;
    }

    const float *bounds = cooling ? OUTDOOR_UNIT_COOLING_BAND_BOUNDS_C : OUTDOOR_UNIT_HEATING_BAND_BOUNDS_C;
    const float *max_hz = cooling ? OUTDOOR_UNIT_COOLING_BAND_MAX_HZ : OUTDOOR_UNIT_HEATING_BAND_MAX_HZ;
    const int bound_count = cooling
      ? sizeof(OUTDOOR_UNIT_COOLING_BAND_BOUNDS_C) / sizeof(OUTDOOR_UNIT_COOLING_BAND_BOUNDS_C[0])
      : sizeof(OUTDOOR_UNIT_HEATING_BAND_BOUNDS_C) / sizeof(OUTDOOR_UNIT_HEATING_BAND_BOUNDS_C[0]);
    int &band = cooling ? cooling_ambient_band_ : heating_ambient_band_;
    band = UpdateAmbientBand(band, ta, bounds, bound_count, max_hz,
                             cooling ? 0.0f : OUTDOOR_UNIT_HEATING_LOW_LIMIT_RESTART_C);
    return max_hz[band];
  }

  /// @brief Frequency the outdoor unit runs for a compressor level, from the table OpenAmber has written
  /// (3001-3010 cooling, 3012-3021 heating). NaN until the table has been read.
  static float GetLevelFrequency(bool cooling, int level)
  {
    if (cooling)
    {
      switch (level)
      {
        case 1: return id(cooling_frequency_index_1).state;
        case 2: return id(cooling_frequency_index_2).state;
        case 3: return id(cooling_frequency_index_3).state;
        case 4: return id(cooling_frequency_index_4).state;
        case 5: return id(cooling_frequency_index_5).state;
        case 6: return id(cooling_frequency_index_6).state;
        case 7: return id(cooling_frequency_index_7).state;
        case 8: return id(cooling_frequency_index_8).state;
        case 9: return id(cooling_frequency_index_9).state;
        case 10: return id(cooling_frequency_index_10).state;
        default: return NAN;
      }
    }
    switch (level)
    {
      case 1: return id(heating_frequency_index_1).state;
      case 2: return id(heating_frequency_index_2).state;
      case 3: return id(heating_frequency_index_3).state;
      case 4: return id(heating_frequency_index_4).state;
      case 5: return id(heating_frequency_index_5).state;
      case 6: return id(heating_frequency_index_6).state;
      case 7: return id(heating_frequency_index_7).state;
      case 8: return id(heating_frequency_index_8).state;
      case 9: return id(heating_frequency_index_9).state;
      case 10: return id(heating_frequency_index_10).state;
      default: return NAN;
    }
  }

  /// @brief Caps the requested level to the highest level whose frequency in the active table stays within
  /// the maximum frequency for the current outdoor temperature. The cap never goes below level 1: outside
  /// the operating range (0 Hz) the compressor runs at level 1 instead of stopping.
  int CapToAmbientFrequencyLimit(int compressor_mode)
  {
    if (compressor_mode <= 1)
    {
      return compressor_mode;
    }

    const bool cooling = id(working_mode_switch).active_index().value_or(WORKING_MODE_STANDBY) == WORKING_MODE_COOLING;
    const float max_frequency = GetAmbientMaxFrequency(cooling);
    if (std::isnan(max_frequency))
    {
      last_logged_ambient_limit_ = -1;
      return compressor_mode;
    }

    int capped = compressor_mode;
    while (capped > 1)
    {
      const float frequency = GetLevelFrequency(cooling, capped);
      if (std::isnan(frequency))
      {
        // Frequency table not read yet: do not guess.
        last_logged_ambient_limit_ = -1;
        return compressor_mode;
      }
      if (frequency <= max_frequency)
      {
        break;
      }
      capped--;
    }

    if (capped == compressor_mode)
    {
      last_logged_ambient_limit_ = -1;
      return compressor_mode;
    }

    if (last_logged_ambient_limit_ != capped)
    {
      ESP_LOGI("amber", "Ambient limit %.0f Hz at Ta %.1f°C caps compressor level to %d (%.0f Hz) instead of %d.",
               max_frequency, id(temperature_outside_ta).state, capped, GetLevelFrequency(cooling, capped), compressor_mode);
      last_logged_ambient_limit_ = capped;
    }
    return capped;
  }


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

  uint64_t GetStartTime()
  {
    return last_compressor_start_ms_;
  }

  void ApplyCompressorMode(int compressor_mode)
  {
    auto current_compressor_mode = id(compressor_control_select).active_index().value();
    compressor_mode = CapToAmbientFrequencyLimit(compressor_mode);
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
    last_compressor_stop_ms_ = millis_64();
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
    const uint64_t min_on_ms = COMPRESSOR_MIN_ON_S * 1000ULL;
    return (millis_64() - last_compressor_start_ms_) > min_on_ms;
  }

  bool HasPassedMinOffTime()
  {
    const uint64_t min_off_ms = COMPRESSOR_MIN_OFF_S * 1000ULL;
    return (millis_64() - last_compressor_stop_ms_) > min_off_ms;
  }

  bool HasPassedSoftStartDuration()
  {
    return (millis_64() - last_compressor_start_ms_) >= COMPRESSOR_SOFT_START_DURATION_S * 1000ULL;
  }

  void RecordStartTime()
  {
    ClearStartWaitTimer();
    last_compressor_start_ms_ = millis_64();
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
    capped_mode_index = CapToAmbientFrequencyLimit(capped_mode_index);
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
