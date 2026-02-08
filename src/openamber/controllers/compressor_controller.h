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
#include "utility_functions.h"
#include "three_way_valve_controller.h"

using namespace esphome;

class CompressorController
{
private:
  uint32_t last_compressor_start_ms_ = 0;
  uint32_t last_compressor_stop_ms_ = 0;
  uint32_t last_compressor_mode_change_ms_ = 0;
  float compressor_pid_ = 0.0f;
  float start_target_temp_ = 0.0f;
  const ThreeWayValveController& valve_controller_;

  void SetWorkingMode(int workingMode)
  {
    auto working_mode_call = id(working_mode_switch).make_call();
    working_mode_call.set_index(workingMode);
    working_mode_call.perform();
  }

  void SetCompressorMode(int mode_index)
  {
    int mode_offset = id(compressor_control_select).size() - id(heat_compressor_mode).size();
    int capped_mode_index = std::min(mode_index, (int)id(heat_compressor_mode).active_index().value() + mode_offset);
    auto compressor_set_call = id(compressor_control_select).make_call();
    compressor_set_call.set_index(capped_mode_index);
    compressor_set_call.perform();
    last_compressor_mode_change_ms_ = millis();
  }

  int MapPIDToCompressorMode(float pid) const
  {
    float raw = fabsf(pid);
    if (raw > 1.0f)
      raw = 1.0f;

    int mode_offset = id(compressor_control_select).size() - id(heat_compressor_mode).size();
    int amount_of_modes = id(heat_compressor_mode).active_index().value() + mode_offset;
    int desired_mode = (int)roundf(raw * (amount_of_modes - 1)) + 1;
    desired_mode = std::max(1, std::min(desired_mode, (int)amount_of_modes));

    auto current_mode = id(compressor_control_select).active_index().value();
    // Limit to a single frequency step per update
    int new_mode = desired_mode;
    if (desired_mode > current_mode + 1)
      new_mode = current_mode + 1;
    if (desired_mode < current_mode - 1)
      new_mode = current_mode - 1;

    return new_mode;
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

  int GetDhwCompressorMode() const
  {
    float temperature_ta = id(temperature_outside_ta).state;
    float ta_threshold = id(dhw_temperature_threshold_max_compressor_mode).state;
    
    int dhw_mode_offset = id(compressor_control_select).size() - id(dhw_compressor_mode).size();
    int dhw_mode_max_offset = id(compressor_control_select).size() - id(dhw_compressor_mode_max).size();
    
    int selected_dhw_compressor_mode = id(dhw_compressor_mode).active_index().value() + dhw_mode_offset;    
    if (temperature_ta <= ta_threshold)
    {
      selected_dhw_compressor_mode = id(dhw_compressor_mode_max).active_index().value() + dhw_mode_max_offset;
    }
    return selected_dhw_compressor_mode;
  }

  bool IsCompressorDemandForCurrentMode() const
  {
    if (valve_controller_.IsInDhwMode())
    {
      return id(dhw_demand_active_sensor).state;
    }
    else
    {
      return id(heat_demand_active_sensor).state || id(cool_demand_active_sensor).state || id(frost_protection_stage2_active).state;
    }
  }

public:
  CompressorController(const ThreeWayValveController& valve_controller) 
    : valve_controller_(valve_controller) {}

  uint32_t GetLastStartTime() const { return last_compressor_start_ms_; }
  uint32_t GetLastStopTime() const { return last_compressor_stop_ms_; }
  float GetStartTargetTemp() const { return start_target_temp_; }

  void SetPIDValue(float pid_value)
  {
    ESP_LOGD("amber", "Writing PID value: %.2f", pid_value);
    compressor_pid_ = pid_value;
  }

  void ManageModulation()
  {
    const uint32_t now = millis();
    int desired_compressor_mode = MapPIDToCompressorMode(compressor_pid_);
    auto current_compressor_mode = id(compressor_control_select).active_index().value();
    const uint32_t min_interval = (current_compressor_mode > desired_compressor_mode ? FREQUENCY_CHANGE_INTERVAL_DOWN_S : FREQUENCY_CHANGE_INTERVAL_UP_S) * 1000;
    if ((now - last_compressor_mode_change_ms_) < min_interval)
    {
      ESP_LOGI("amber", "Frequency change skipped (interval not elapsed)");
      return;
    }

    // Deadband on Tc to avoid too frequent changes around setpoint
    float current_temperature = id(heat_cool_temperature_tc).state;
    float target = GetTargetTemperature(valve_controller_);
    float dt = GetTemperatureDelta(valve_controller_);

    if (fabsf(dt) < DEAD_BAND_DT)
    {
      ESP_LOGD("amber", "ΔT=%.2f°C within deadband, compressor mode remains at %d",
               dt, current_compressor_mode);
      return;
    }

    // If Tc is above setpoint, then never modulate up
    const float TEMP_MARGIN = 0.3f;
    if (current_temperature > (target + TEMP_MARGIN) && desired_compressor_mode > current_compressor_mode)
    {
      ESP_LOGW("amber", "Temperature is above target and PID controller wanted to move to a higher compressor mode.");
      return;
    }

    ESP_LOGI("amber", "PID %.2f -> mode %d (previous mode %d)", compressor_pid_, desired_compressor_mode, current_compressor_mode);

    if (desired_compressor_mode != current_compressor_mode)
    {
      SetCompressorMode(desired_compressor_mode);
      ESP_LOGI("amber", "Compressor mode updated to %d", desired_compressor_mode);
    }
  }

  void Start(bool is_switching_modes)
  {
    start_target_temp_ = GetTargetTemperature(valve_controller_);
    if (valve_controller_.IsInDhwMode())
    {
      int dhw_compressor_mode = GetDhwCompressorMode();
      ESP_LOGI("amber", "Starting compressor in DHW mode, setting compressor to mode %d", dhw_compressor_mode);
      SetCompressorMode(dhw_compressor_mode);
    }
    else
    {
      float supply_temperature_delta = GetTemperatureDelta(valve_controller_);
      ESP_LOGI("amber", "Starting compressor (ΔT=%.2f°C)", supply_temperature_delta);
      int compressor_frequency_mode = DetermineSoftStartMode(supply_temperature_delta);
      ESP_LOGI("amber", "Soft start mode: %d (ΔT=%.1f°C)", compressor_frequency_mode, supply_temperature_delta);
      id(pid_heat_cool_temperature_control).reset_integral_term();
      SetCompressorMode(compressor_frequency_mode);
    }
  }

  void Stop(uint32_t pump_interval_ms)
  {
    last_compressor_start_ms_ = 0;
    last_compressor_stop_ms_ = millis();
    auto compressor_set_call = id(compressor_control_select).make_call();
    compressor_set_call.select_first();
    compressor_set_call.perform();

    SetWorkingMode(WORKING_MODE_STANDBY);
  }

  bool ShouldStart(uint32_t pump_start_time) const
  {
    if (valve_controller_.IsInDhwMode())
    {
      return id(dhw_demand_active_sensor).state;
    }

    float current_temperature = id(heat_cool_temperature_tc).state;
    float target_temperature = GetTargetTemperature(valve_controller_);

    if (!IsCompressorDemandForCurrentMode())
    {
      ESP_LOGI("amber", "Not starting compressor because there is no demand.");
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

  bool ShouldStop() const
  {
    if(valve_controller_.IsInDhwMode())
    {
      // In DHW mode, stop when target temperature is reached
      float current_temperature = GetCurrentTemperature(valve_controller_);
      float target_temperature = GetTargetTemperature(valve_controller_);
      ESP_LOGI("amber", "DHW mode: Checking if current temperature %.2f°C >= target temperature %.2f°C to stop compressor.", current_temperature, target_temperature);
      return current_temperature >= target_temperature;
    }
    else
    {
      // Stop when there's no demand
      return !IsCompressorDemandForCurrentMode();
    }
  }

  bool IsRunning() const
  {
    return id(current_compressor_frequency).state > 0;
  }

  bool HasPassedMinOnTime() const
  {
    const uint32_t min_on_ms = COMPRESSOR_MIN_ON_S * 1000UL;
    return (millis() - last_compressor_start_ms_) > min_on_ms;
  }

  bool HasPassedMinOffTime(bool is_switching_modes) const
  {
    const uint32_t min_off_ms = COMPRESSOR_MIN_OFF_S * 1000UL;
    return millis() >= last_compressor_stop_ms_ + min_off_ms || is_switching_modes;
  }

  bool HasPassedSoftStartDuration() const
  {
    return millis() - last_compressor_start_ms_ >= COMPRESSOR_SOFT_START_DURATION_S * 1000UL;
  }

  void RecordStartTime()
  {
    last_compressor_start_ms_ = millis();
  }

  void ApplyDhwCompressorMode()
  {
    int dhw_compressor_mode = GetDhwCompressorMode();
    auto current_compressor_mode = id(compressor_control_select).active_index().value();
    if (current_compressor_mode != dhw_compressor_mode)
    {
      ESP_LOGI("amber", "Applying compressor mode change for DHW to %d", dhw_compressor_mode);
      auto compressor_set_call = id(compressor_control_select).make_call();
      compressor_set_call.set_index(dhw_compressor_mode);
      compressor_set_call.perform();
      last_compressor_mode_change_ms_ = millis();
    }
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

  int DetermineWorkingMode() const
  {
    return valve_controller_.IsInDhwMode() ? WORKING_MODE_HEATING 
         : id(heat_demand_active_sensor).state ? WORKING_MODE_HEATING
         : WORKING_MODE_COOLING;
  }
};
