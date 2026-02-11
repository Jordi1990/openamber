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

class BackupHeaterController
{
private:
  uint32_t backup_degmin_last_ms_ = 0;
  float accumulated_backup_degmin_ = 0.0f;
  float temperature_rate_c_per_min_ = 0.0f;
  float last_temperature_for_rate_ = 0.0f;
  const ThreeWayValveController& valve_controller_;

  float CalculatePredictedTemperature()
  {
    float current_temperature = GetCurrentTemperature(valve_controller_);
    float target = GetTargetTemperature(valve_controller_);

    uint32_t dt_ms = millis() - backup_degmin_last_ms_;
    const float dt_min = (float)dt_ms / 60000.0f;
    if (dt_min <= 0.0f)
    {
      ESP_LOGW("amber", "Delta time for backup heater degree/min rate calculation is zero or negative, skipping update.");
      return 0.0f;
    }
    // Calculate degree/minute rate
    float rate = (current_temperature - last_temperature_for_rate_) / dt_min;
    // Low-pass filter the rate to avoid spikes
    double a = 0.2f;
    temperature_rate_c_per_min_ = (1 - a) * temperature_rate_c_per_min_ + a * rate;
    last_temperature_for_rate_ = current_temperature;
    // Predict future Tc based on current rate
    float predicted_temperature = current_temperature + temperature_rate_c_per_min_ * ((float)BACKUP_HEATER_LOOKAHEAD_S / 60.0f);
    backup_degmin_last_ms_ = millis();
    ESP_LOGI("amber", "Backup heater temperature rate/min updated: %.2f (Predicted Temperature: %.2f°C)", temperature_rate_c_per_min_, predicted_temperature);
    return predicted_temperature;
  }

public:
  BackupHeaterController(const ThreeWayValveController& valve_controller) 
    : valve_controller_(valve_controller) {}

  void CheckActivation(uint32_t compressor_start_time, float start_target_temp)
  {
    if (!valve_controller_.IsInDhwMode())
    {
      CalculateAccumulatedDegreeMinutes();

      if (accumulated_backup_degmin_ >= id(backup_heater_degmin_threshold).state)
      {
        ESP_LOGI("amber", "Enabling backup heater (degree/min limit reached: %.2f)", accumulated_backup_degmin_);
        accumulated_backup_degmin_ = 0.0f;
        id(backup_heater_relay).turn_on();
      }
    }
    else
    {
      // DHW mode backup heater logic
      const uint32_t now = millis();
      const float elapsed_min = (float)(now - compressor_start_time) / 60000.0f;
      const float gained = GetCurrentTemperature(valve_controller_) - start_target_temp;
      const float avg_rate = (elapsed_min > 0.1f) ? (gained / elapsed_min) : 0.0f;
      id(dhw_backup_current_avg_rate_sensor).publish_state(avg_rate);
      if (avg_rate < id(dhw_backup_min_avg_rate).state && elapsed_min >= DHW_BACKUP_HEATER_GRACE_PERIOD_S / 60.0f)
      {
        ESP_LOGI("amber", "DHW backup enable: avg_rate=%.3f°C/min < min=%.3f°C/min", avg_rate, id(dhw_backup_min_avg_rate).state);
        id(backup_heater_relay).turn_on();
      }
    }
  }

  bool ShouldDeactivateBasedOnPrediction()
  {
    float predicted_temperature = CalculatePredictedTemperature();
    float target_temperature = GetTargetTemperature(valve_controller_);
    
    // Disable backup heater if predicted Temperature is above target
    if (predicted_temperature >= target_temperature + 2.0f)
    { 
      ESP_LOGI("amber", "Disabling backup heater (predicted Temperature %.2f°C above target %.2f°C)", predicted_temperature, target_temperature);
      id(backup_heater_relay).turn_off();
      return true;
    }
    return false;
  }
};
