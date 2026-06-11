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
  uint32_t last_frequency_limit_deactivation_time_ms_ = 0;
  bool frequency_limit_active_ = false;

  static float Lerp(float x, float x0, float y0, float x1, float y1)
  {
    if (fabsf(x1 - x0) < 0.0001f)
    {
      return y0;
    }
    const float t = (x - x0) / (x1 - x0);
    return y0 + t * (y1 - y0);
  }

  static float ComputeTuoMaxFromTa(float ta)
  {
    if (TUO_LIMIT_CURVE_POINT_COUNT == 0)
    {
      return 70.0f;
    }

    if (ta <= TUO_LIMIT_CURVE_POINTS[0].ta_c)
    {
      return TUO_LIMIT_CURVE_POINTS[0].tuo_max_c;
    }

    const uint8_t last = TUO_LIMIT_CURVE_POINT_COUNT - 1;
    if (ta >= TUO_LIMIT_CURVE_POINTS[last].ta_c)
    {
      return TUO_LIMIT_CURVE_POINTS[last].tuo_max_c;
    }

    for (uint8_t i = 1; i < TUO_LIMIT_CURVE_POINT_COUNT; ++i)
    {
      const auto &left = TUO_LIMIT_CURVE_POINTS[i - 1];
      const auto &right = TUO_LIMIT_CURVE_POINTS[i];
      if (ta <= right.ta_c)
      {
        return Lerp(ta, left.ta_c, left.tuo_max_c, right.ta_c, right.tuo_max_c);
      }
    }

    return TUO_LIMIT_CURVE_POINTS[last].tuo_max_c;
  }

public:
  CompressorController() {}

  uint32_t GetStartTime()
  {
    return last_compressor_start_ms_;
  }

  int ApplyFrequencyLimit(int desired_mode)
  {
    const float ta = id(temperature_outside_ta_avg_1h).state;
    const float tuo = id(outlet_temperature_tuo).state;
    const int current_mode = id(compressor_control_select).active_index().value();
    const bool frequency_limit_enabled = id(frequency_limit_mode).active_index().value() == 0;

    if (!frequency_limit_enabled ||
        std::isnan(ta) ||
        std::isnan(tuo))
    {
      return desired_mode;
    }

    const float tuo_max = ComputeTuoMaxFromTa(ta);
    const uint32_t now = App.get_loop_component_start_time();
    const int cutoff_mode = FREQUENCY_LIMIT_CUTOFF_MODE_INDEX;

    const uint32_t frequency_limit_lock_duration_ms = FREQUENCY_LIMIT_UP_LOCK_S * 1000UL;

    if (tuo > tuo_max && (now - last_frequency_limit_deactivation_time_ms_) > frequency_limit_lock_duration_ms)
    {
      frequency_limit_active_ = true;
      const int down_target = std::max(cutoff_mode, current_mode - 1);
      const int capped_mode = std::min(desired_mode, down_target);
      ESP_LOGI("amber", "Limiter active: Ta=%.2f Tuo=%.2f Tuo_max=%.2f cutoff_mode=%d mode %d->%d", ta, tuo, tuo_max, cutoff_mode, current_mode, capped_mode);
      return capped_mode;
    }

    if(frequency_limit_active_ && tuo <= tuo_max)
    {
      frequency_limit_active_ = false;
      last_frequency_limit_deactivation_time_ms_ = now;
      ESP_LOGI("amber", "Limiter deactivated: Ta=%.2f Tuo=%.2f Tuo_max=%.2f cutoff_mode=%d mode %d->%d", ta, tuo, tuo_max, cutoff_mode, current_mode, desired_mode);
    }

    return desired_mode;
  }

  void ApplyCompressorMode(int compressor_mode)
  {
    auto current_compressor_mode = id(compressor_control_select).active_index().value();
    int capped_compressor_mode = ApplyFrequencyLimit(compressor_mode);
    // On initial start, allow any mode.
    if(current_compressor_mode != 0)
    {
      // Limit to a single frequency step per update
      if (capped_compressor_mode > current_compressor_mode + 1)
        capped_compressor_mode = current_compressor_mode + 1;
      if (capped_compressor_mode < current_compressor_mode - 1)
        capped_compressor_mode = current_compressor_mode - 1;

      if (current_compressor_mode == capped_compressor_mode)
      {
        return;
      }

      const uint32_t min_interval = (current_compressor_mode > capped_compressor_mode ? FREQUENCY_CHANGE_INTERVAL_DOWN_S : FREQUENCY_CHANGE_INTERVAL_UP_S) * 1000;
      if ((App.get_loop_component_start_time() - last_compressor_mode_change_ms_) < min_interval)
      {
        ESP_LOGI("amber", "Frequency change skipped (interval not elapsed)");
        return;
      }
    }

    auto compressor_set_call = id(compressor_control_select).make_call();
    compressor_set_call.set_index(capped_compressor_mode);
    compressor_set_call.perform();
    ESP_LOGI("amber", "Applying compressor mode change to %d", capped_compressor_mode);
    last_compressor_mode_change_ms_ = App.get_loop_component_start_time();
  }

  void Stop()
  {
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
    last_compressor_start_ms_ = App.get_loop_component_start_time();
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
    last_compressor_mode_change_ms_ = App.get_loop_component_start_time();
  }
};
