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
  int heating_ambient_band_ = -1;
  int cooling_ambient_band_ = -1;
  int last_logged_ambient_limit_ = -1;

  static int UpdateAmbientBand(int band, float ta, const float *bounds, int bound_count)
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

    while (band < raw_band && ta >= bounds[band] + OUTDOOR_UNIT_BAND_HYSTERESIS_C)
    {
      band++;
    }
    while (band > raw_band && ta < bounds[band - 1] - OUTDOOR_UNIT_BAND_HYSTERESIS_C)
    {
      band--;
    }
    return band;
  }

  /// @brief Maximum compressor frequency for the current outdoor temperature, using the Ta bands of the
  /// outdoor unit (firmware v1.18). Returns NaN when there is no cap: Ta unknown, or a band outside the
  /// operating range where the unit blocks the compressor itself (P11).
  float GetAmbientMaxFrequency(bool cooling)
  {
    const float ta = id(temperature_outside_ta).state;
    if (std::isnan(ta))
    {
      return NAN;
    }

    if (cooling)
    {
      const int bound_count = sizeof(OUTDOOR_UNIT_COOLING_BAND_BOUNDS_C) / sizeof(OUTDOOR_UNIT_COOLING_BAND_BOUNDS_C[0]);
      cooling_ambient_band_ = UpdateAmbientBand(cooling_ambient_band_, ta, OUTDOOR_UNIT_COOLING_BAND_BOUNDS_C, bound_count);
      const float limits[OUTDOOR_UNIT_COOLING_LIMITED_BANDS] = {
        id(ambient_limit_cooling_band_1_max_frequency).state,
        id(ambient_limit_cooling_band_2_max_frequency).state,
        id(ambient_limit_cooling_band_3_max_frequency).state,
        id(ambient_limit_cooling_band_4_max_frequency).state,
        id(ambient_limit_cooling_band_5_max_frequency).state,
        id(ambient_limit_cooling_band_6_max_frequency).state,
        id(ambient_limit_cooling_band_7_max_frequency).state,
        id(ambient_limit_cooling_band_8_max_frequency).state,
      };
      const int band = cooling_ambient_band_ - 1;
      return (band >= 0 && band < OUTDOOR_UNIT_COOLING_LIMITED_BANDS) ? limits[band] : NAN;
    }

    const int bound_count = sizeof(OUTDOOR_UNIT_HEATING_BAND_BOUNDS_C) / sizeof(OUTDOOR_UNIT_HEATING_BAND_BOUNDS_C[0]);
    heating_ambient_band_ = UpdateAmbientBand(heating_ambient_band_, ta, OUTDOOR_UNIT_HEATING_BAND_BOUNDS_C, bound_count);
    const float limits[OUTDOOR_UNIT_HEATING_LIMITED_BANDS] = {
      id(ambient_limit_heating_band_1_max_frequency).state,
      id(ambient_limit_heating_band_2_max_frequency).state,
      id(ambient_limit_heating_band_3_max_frequency).state,
      id(ambient_limit_heating_band_4_max_frequency).state,
      id(ambient_limit_heating_band_5_max_frequency).state,
    };
    const int band = heating_ambient_band_ - 1;
    return (band >= 0 && band < OUTDOOR_UNIT_HEATING_LIMITED_BANDS) ? limits[band] : NAN;
  }

  /// @brief Frequency the outdoor unit runs for a compressor level, from the table that is active in the
  /// unit (3001-3010 cooling, 3012-3021 heating). NaN until the table has been read.
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

  /// @brief Caps the requested level so its frequency stays within the maximum frequency for the current
  /// outdoor temperature. The cap is in Hz, so it keeps matching the stock unit when the frequency table
  /// is changed. Level 1 is never capped further: below that the compressor would stop.
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

public:
  CompressorController() {}

  uint32_t GetStartTime()
  {
    return last_compressor_start_ms_;
  }

  void ApplyCompressorMode(int compressor_mode)
  {
    auto current_compressor_mode = id(compressor_control_select).active_index().value();
    compressor_mode = CapToAmbientFrequencyLimit(compressor_mode);

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
    capped_mode_index = CapToAmbientFrequencyLimit(capped_mode_index);
    compressor_set_call.set_index(capped_mode_index);
    compressor_set_call.perform();
    last_compressor_mode_change_ms_ = App.get_loop_component_start_time();
  }
};
