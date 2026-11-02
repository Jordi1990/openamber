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

  /// @brief Level cap of a band; bands where the unit blocks the compressor (0) are not capped here.
  static int BandCap(const int *max_levels, int band)
  {
    return max_levels[band] > 0 ? max_levels[band] : 10;  // 10 = highest level, no cap
  }

  /// @brief Follows the outdoor unit's Ta band with its 1 K hysteresis. A step to a band with a lower limit
  /// needs AMBIENT_LIMIT_TA_OFFSET_C extra, so OpenAmber caps a little later than the unit does.
  static int UpdateAmbientBand(int band, float ta, const float *bounds, int bound_count, const int *max_levels)
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
      const float extra = BandCap(max_levels, band + 1) < BandCap(max_levels, band) ? AMBIENT_LIMIT_TA_OFFSET_C : 0.0f;
      if (ta < bounds[band] + OUTDOOR_UNIT_BAND_HYSTERESIS_C + extra)
      {
        break;
      }
      band++;
    }
    while (band > raw_band)
    {
      const float extra = BandCap(max_levels, band - 1) < BandCap(max_levels, band) ? AMBIENT_LIMIT_TA_OFFSET_C : 0.0f;
      if (ta >= bounds[band - 1] - OUTDOOR_UNIT_BAND_HYSTERESIS_C - extra)
      {
        break;
      }
      band--;
    }
    return band;
  }

  /// @brief Maximum compressor level for the current outdoor temperature, with the fixed Ta bands and
  /// levels of the outdoor unit (firmware v1.18). Returns -1 when there is no cap: Ta unknown, or a band
  /// outside the operating range where the unit blocks the compressor itself (P11).
  int GetAmbientMaxLevel(bool cooling)
  {
    const float ta = id(temperature_outside_ta).state;
    if (std::isnan(ta))
    {
      return -1;
    }

    const float *bounds = cooling ? OUTDOOR_UNIT_COOLING_BAND_BOUNDS_C : OUTDOOR_UNIT_HEATING_BAND_BOUNDS_C;
    const int *max_levels = cooling ? OUTDOOR_UNIT_COOLING_BAND_MAX_LEVEL : OUTDOOR_UNIT_HEATING_BAND_MAX_LEVEL;
    const int bound_count = cooling
      ? sizeof(OUTDOOR_UNIT_COOLING_BAND_BOUNDS_C) / sizeof(OUTDOOR_UNIT_COOLING_BAND_BOUNDS_C[0])
      : sizeof(OUTDOOR_UNIT_HEATING_BAND_BOUNDS_C) / sizeof(OUTDOOR_UNIT_HEATING_BAND_BOUNDS_C[0]);
    int &band = cooling ? cooling_ambient_band_ : heating_ambient_band_;
    band = UpdateAmbientBand(band, ta, bounds, bound_count, max_levels);
    return max_levels[band] > 0 ? max_levels[band] : -1;
  }

  /// @brief Caps the requested level to the outdoor unit's maximum level for the current outdoor temperature.
  int CapToAmbientLevelLimit(int compressor_mode)
  {
    if (compressor_mode <= 1)
    {
      return compressor_mode;
    }

    const bool cooling = id(working_mode_switch).active_index().value_or(WORKING_MODE_STANDBY) == WORKING_MODE_COOLING;
    const int max_level = GetAmbientMaxLevel(cooling);
    if (max_level < 0 || compressor_mode <= max_level)
    {
      last_logged_ambient_limit_ = -1;
      return compressor_mode;
    }

    const int capped = std::max(1, max_level);
    if (last_logged_ambient_limit_ != capped)
    {
      ESP_LOGI("amber", "Ambient limit at Ta %.1f°C caps compressor level to %d instead of %d.",
               id(temperature_outside_ta).state, capped, compressor_mode);
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
    compressor_mode = CapToAmbientLevelLimit(compressor_mode);

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
    capped_mode_index = CapToAmbientLevelLimit(capped_mode_index);
    compressor_set_call.set_index(capped_mode_index);
    compressor_set_call.perform();
    last_compressor_mode_change_ms_ = App.get_loop_component_start_time();
  }
};
