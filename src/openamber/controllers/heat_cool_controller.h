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
#include "controller.h"
#include "constants.h"

using namespace esphome;

class HeatCoolController : Controller
{
private:
  uint32_t backup_degmin_last_ms_ = 0;
  float accumulated_backup_degmin_ = 0.0f;
  float temperature_rate_c_per_min_ = 0.0f;
  float last_temperature_for_rate_ = 0.0f;
  float compressor_pid_ = 0.0f;
  float start_current_temperature = 0.0f;

  int MapPIDToCompressorMode(float pid) const
  {
    float raw = fabsf(pid);
    if (raw > 1.0f)
      raw = 1.0f;

    int mode_offset = id(compressor_control_select).size() - id(heat_compressor_mode).size();
    int amount_of_modes = id(heat_compressor_mode).active_index().value() + mode_offset;
    int desired_mode = (int)roundf(raw * (amount_of_modes - 1)) + 1;
    return std::max(1, std::min(desired_mode, (int)amount_of_modes));
  }

  int DetermineSoftStartMode(float supply_temperature_delta) const
  {
    int start_compressor_frequency_mode;
    float delta = fabs(supply_temperature_delta);
    if (delta < 5.0f)
      start_compressor_frequency_mode = 1;
    else if (delta < 7.0f)
      start_compressor_frequency_mode = 3;
    else if (delta < 10.0f)
      start_compressor_frequency_mode = 4;
    else
      start_compressor_frequency_mode = 5;

    return start_compressor_frequency_mode;
  }
public:
  HeatCoolController() {}

  bool HasDemand() override {
    float current_temperature = id(heat_cool_temperature_tc).state;
    float target_temperature = id(pid_heat_cool_temperature_control).target_temperature;

    bool has_active_demand = id(heat_demand_active_sensor).state || id(cool_demand_active_sensor).state || id(frost_protection_stage2_active).state;
    if (!has_active_demand)
    {
      return false;
    }

    // Start condition based on target temperature and start_compressor_delta
    float start_temperature = target_temperature - id(compressor_start_delta).state;
    if (current_temperature >= start_temperature && !id(frost_protection_stage2_active).state)
    {
      ESP_LOGI("amber", "Not starting compressor because target temperature (%.2f) is higher than the start temperature(%.2f)", current_temperature, start_temperature);
      return false;
    }

    return true;
  }

  void OnCompressorStarted() override {
    start_current_temperature = id(heat_cool_temperature_tc).state;
  }

  float GetPreferredPumpSpeed() override {
    return id(pump_speed_heating_number).state;
  }

  void CalculateAccumulatedDegreeMinutes()
  {
    float tc = id(heat_cool_temperature_tc).state;
    float target = id(pid_heat_cool_temperature_control).target_temperature;
    const float diff = std::max(0.0f, target - tc);
    uint32_t dt_ms = millis() - backup_degmin_last_ms_;
    backup_degmin_last_ms_ = millis();
    const float dt_min = (float)dt_ms / 60000.0f;

    int mode_offset = id(compressor_control_select).size() - id(heat_compressor_mode).size();
    int max_compressor_mode = id(heat_compressor_mode).active_index().value() + mode_offset;
    bool is_at_max_mode = id(compressor_control_select).active_index().value() >= max_compressor_mode;

    // Reset accumulated degmin if there is no difference or we are not at max compressor mode
    if (diff <= 0.0f || !is_at_max_mode)
    {
      accumulated_backup_degmin_ = 0.0f;
    }

    if (dt_min <= 0.0f)
    {
      return;
    }

    accumulated_backup_degmin_ += diff * dt_min;
    id(backup_heater_degmin_current_sensor).publish_state(accumulated_backup_degmin_);
    ESP_LOGD("amber", "Backup heater degree/minutes updated: +%.2f -> %.2f", diff * dt_min, accumulated_backup_degmin_);
  }

  void ResetAccumulatedDegMin()
  {
    accumulated_backup_degmin_ = 0.0f;
  }

  void InitializeBackupDegMinTracking()
  {
    backup_degmin_last_ms_ = millis();
  }

  int DetermineCompressorMode() override
  {
    const uint32_t now = millis();
    int desired_compressor_mode = MapPIDToCompressorMode(compressor_pid_);
    auto current_compressor_mode = id(compressor_control_select).active_index().value();
    float current_temperature = id(heat_cool_temperature_tc).state;
    float target = id(pid_heat_cool_temperature_control).target_temperature;
    float dt = current_temperature - target;

    // If compressor is not running use soft start mode based on ΔT.
    if (current_compressor_mode == 0)
    {
      return DetermineSoftStartMode(dt);
    }

    // Deadband on Tc to avoid too frequent changes around setpoint
    if (fabsf(dt) < DEAD_BAND_DT)
    {
      ESP_LOGD("amber", "ΔT=%.2f°C within deadband, compressor mode remains at %d",
               dt, current_compressor_mode);
      return current_compressor_mode;
    }

    // If Tc is above setpoint, then never modulate up
    const float TEMP_MARGIN = 0.3f;
    if (current_temperature > (target + TEMP_MARGIN) && desired_compressor_mode > current_compressor_mode)
    {
      ESP_LOGW("amber", "Temperature is above target and PID controller wanted to move to a higher compressor mode.");
      return current_compressor_mode;
    }

    int mode_offset = id(compressor_control_select).size() - id(heat_compressor_mode).size();
    int capped_mode_index = std::min(desired_compressor_mode, (int)id(heat_compressor_mode).active_index().value() + mode_offset);

    ESP_LOGI("amber", "PID %.2f -> mode %d (previous mode %d)", compressor_pid_, capped_mode_index, current_compressor_mode);
    return capped_mode_index;
  }

  bool ShouldStopCompressor() override
  {
    if(!HasDemand())
    {
      return true;
    }

    // Stop when we overshoot above target + delta
    float target_temperature = id(pid_heat_cool_temperature_control).target_temperature;
    float current_temperature = id(heat_cool_temperature_tc).state;
    float supply_temperature_delta = current_temperature - target_temperature;
    
    if (supply_temperature_delta >= id(compressor_stop_delta).state)
    {
      if (id(oil_return_cycle_active).state)
      {
        ESP_LOGI("amber", "Not stopping compressor because oil return cycle is active.");
        return false;
      }

      ESP_LOGI("amber", "Stopping compressor because it reached delta %.2f°C (ΔT=%.2f°C).", id(compressor_stop_delta).state, supply_temperature_delta);
      return true;
    }
  }
};
