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
#include "pump_controller.h"
#include "compressor_controller.h"
#include <vector>
#include <algorithm>

using namespace esphome;

enum class DHWState
{
  UNKNOWN,
  IDLE,
  WAIT_PUMP_RUNNING,
  PUMP_RUNNING,
  WAIT_COMPRESSOR_RUNNING,
  COMPRESSOR_RUNNING,
  WAIT_DHW_PUMP_RUNNING,
  WAIT_BACKUP_HEATER_RUNNING,
  BACKUP_HEATER_RUNNING,
  WAIT_COMPRESSOR_STOP,
  WAIT_PUMP_STOP,
  WAIT_FOR_STATE_SWITCH,
};

class DHWController
{
private:
  DHWState state_ = DHWState::UNKNOWN;
  DHWState deferred_machine_state_;
  uint32_t defer_state_change_until_ms_;
  uint32_t backup_degmin_last_ms_ = 0;
  float temperature_rate_c_per_min_ = 0.0f;
  float last_temperature_for_rate_ = 0.0f;
  bool requested_to_stop_ = false;
  PumpController *pump_controller_;
  CompressorController *compressor_controller_;
  float dhw_pump_start_time_ = 0.0f;
  uint32_t last_dhw_avg_sample_ms_ = 0;
  bool defrost_pause_active_ = false;
  std::vector<std::pair<uint32_t, float>> dhw_temperature_samples_;

  const char* DHWStateToString(DHWState state) const
  {
    switch (state)
    {
      case DHWState::IDLE:
        return "Idle";
      case DHWState::WAIT_PUMP_RUNNING:
        return "Wait pump running";
      case DHWState::PUMP_RUNNING:
        return "Pump running";
      case DHWState::WAIT_COMPRESSOR_RUNNING:
        return "Wait compressor running";
      case DHWState::WAIT_DHW_PUMP_RUNNING:
        return "Wait DHW pump running";
      case DHWState::COMPRESSOR_RUNNING:
        return "Compressor running";
      case DHWState::WAIT_BACKUP_HEATER_RUNNING:
        return "Wait backup heater running";
      case DHWState::BACKUP_HEATER_RUNNING:
        return "Backup heater running";
      case DHWState::WAIT_COMPRESSOR_STOP:
        return "Wait compressor stop";
      case DHWState::WAIT_PUMP_STOP:
        return "Wait pump stop";
      case DHWState::WAIT_FOR_STATE_SWITCH:
        return "Wait for state switch";
      default:
        return "Unknown";
    }
  }

  void SetNextState(DHWState new_state)
  {
    state_ = new_state;
    const char* txt = DHWStateToString(new_state);
    
    id(state_machine_state_dhw).publish_state(txt);
    ESP_LOGI("amber", "DHW state changed: %s", txt);
  }

  void LeaveStateAndSetNextStateAfterWaitTime(DHWState new_state, uint32_t defer_ms)
  {
    deferred_machine_state_ = new_state;
    defer_state_change_until_ms_ = millis() + defer_ms;
    SetNextState(DHWState::WAIT_FOR_STATE_SWITCH);
  }

  float GetPreferredPumpSpeed() {
    return id(pump_speed_dhw_number).state;
  }

  int DetermineCompressorMode()
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

  bool IsTargetTemperatureReached()
  {
    // In DHW mode, stop when target temperature is reached
    float current_temperature = id(dhw_temperature_tw_sensor).state;
    float target_temperature = id(current_dhw_setpoint_sensor).state;
    return current_temperature >= target_temperature;
  }

  void StopDhwPump()
  {
    ESP_LOGI("amber", "Stopping DHW pump.");
    id(dhw_pump_relay_switch).turn_off();
    dhw_pump_start_time_ = 0.0f;
    last_dhw_avg_sample_ms_ = 0;
    defrost_pause_active_ = false;
    dhw_temperature_samples_.clear();
  }

  bool IsPredictedTemperatureAboveTarget()
  {
    float predicted_temperature = CalculatePredictedTemperature();
    float target_temperature = id(current_dhw_setpoint_sensor).state;
    
    // Disable backup heater if predicted Temperature is above target
    if (predicted_temperature >= target_temperature + 2.0f)
    { 
      return true;
    }
    return false;
  }

  float CalculatePredictedTemperature()
  {
    float current_temperature = id(dhw_temperature_tw_sensor).state;
    float target = id(current_dhw_setpoint_sensor).state;

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

  bool ShouldStartDhwPump()
  {
    int pump_start_mode = id(dhw_pump_start_mode_select).active_index().value();
    if(id(dhw_pump_relay_switch).state)
    {
      return false;
    }

    if (pump_start_mode == DHW_START_PUMP_MODE_DIRECT)
    {
      return true;
    }

    if(pump_start_mode == DHW_START_PUMP_MODE_DELTA_T)
    {
      float dhw_temp = id(dhw_temperature_tw_sensor).state;
      float temperature_output = id(outlet_temperature_tuo).state;
      if (temperature_output >= dhw_temp)
      {
        return true;
      }
    }

    return false;
  }

  bool IsHeatingSlowerThanMinimumAverageRate()
  {
    const uint32_t now = millis();

    // Don't enable backup heater based when DHW pump is not started yet.
    if(dhw_pump_start_time_ == 0.0f)
    {
      return false;
    }

    // During defrost, don't evaluate DHW average. Defrost can flatten/drop TW temporarily.
    if(id(defrost_active_sensor).state)
    {
      if(!defrost_pause_active_)
      {
        ESP_LOGI("amber", "DHW backup average paused during defrost.");
      }
      defrost_pause_active_ = true;
      return false;
    }

    // After defrost, restart rolling window to avoid defrost dip affecting backup activation.
    if(defrost_pause_active_)
    {
      defrost_pause_active_ = false;
      last_dhw_avg_sample_ms_ = now;
      dhw_temperature_samples_.clear();
      dhw_temperature_samples_.emplace_back(now, id(dhw_temperature_tw_sensor).state);
      ESP_LOGI("amber", "DHW backup average resumed after defrost; rolling window restarted.");
      return false;
    }

    // Evaluate a new rolling average sample once per minute.
    if(last_dhw_avg_sample_ms_ != 0 && now - last_dhw_avg_sample_ms_ < 60000UL)
    {
      return false;
    }
    last_dhw_avg_sample_ms_ = now;

    const float configured_window_min = id(dhw_backup_avg_window_minutes).state;
    const uint32_t window_ms = (uint32_t)std::max(1.0f, configured_window_min) * 60000UL;
    const float current_temperature = id(dhw_temperature_tw_sensor).state;

    dhw_temperature_samples_.emplace_back(now, current_temperature);
    while(!dhw_temperature_samples_.empty() && now - dhw_temperature_samples_.front().first > window_ms)
    {
      dhw_temperature_samples_.erase(dhw_temperature_samples_.begin());
    }

    if(dhw_temperature_samples_.empty())
    {
      return false;
    }

    const auto oldest_sample = dhw_temperature_samples_.front();
    const uint32_t elapsed_ms = now - oldest_sample.first;
    if(elapsed_ms < window_ms)
    {
      ESP_LOGD("amber", "DHW rolling average window not filled yet (%lu/%lu ms).", elapsed_ms, window_ms);
      return false;
    }

    const float elapsed_min = (float)elapsed_ms / 60000.0f;
    const float gained = current_temperature - oldest_sample.second;
    const float avg_rate = (elapsed_min > 0.0f) ? (gained / elapsed_min) : 0.0f;

    id(dhw_backup_current_avg_rate_sensor).publish_state(avg_rate);
    if (avg_rate < id(dhw_backup_min_avg_rate).state && id(dhw_backup_min_avg_rate).state > 0.0f)
    {
      ESP_LOGI("amber", "DHW backup enable: rolling avg_rate=%.3f°C/min over %.1f min < min=%.3f°C/min", avg_rate, configured_window_min, id(dhw_backup_min_avg_rate).state);
      return true;
    }

    return false;
  }

  bool IsBackupHeaterActive()
  {
    return id(backup_heater_active_sensor).state;
  }

  void TurnOnBackupHeater()
  {
    id(backup_heater_relay).turn_on();
  }

  void TurnOffBackupHeater()
  {
    id(backup_heater_relay).turn_off();
  }

  void DoSafetyChecks()
  {
    // If pump is not active while compressor is running, stop compressor to avoid damage
    if (compressor_controller_->IsRunning() && !pump_controller_->IsRunning())
    {
      ESP_LOGW("amber", "Safety check: Pump is not active while compressor is running, stopping compressor to avoid damage.");
      compressor_controller_->Stop();
      SetNextState(DHWState::IDLE);
      return;
    }

    // If Tuo - Tui is above 8 degrees while compressor is running, stop compressor to avoid damage
    if (id(outlet_temperature_tuo).state - id(inlet_temperature_tui).state > 8.0f && compressor_controller_->IsRunning())
    {
      ESP_LOGW("amber", "Safety check: Temperature difference between Tuo and Tui is above 8 degrees while compressor is running, stopping compressor to avoid damage.");
      compressor_controller_->Stop();
      pump_controller_->Stop();
      SetNextState(DHWState::IDLE);
      return;
    }
  }

  void SetWorkingMode(int working_mode)
  {
    if(id(working_mode_switch).active_index().value() == working_mode)
    {
      return;
    }

    auto working_mode_call = id(working_mode_switch).make_call();
    working_mode_call.set_index(working_mode);
    working_mode_call.perform();
  }

  bool HasDemand()
  {
    return id(dhw_demand_active_sensor).state;
  }
public:
    DHWController(PumpController *pump_controller, CompressorController *compressor_controller)
        : pump_controller_(pump_controller), compressor_controller_(compressor_controller) {}

  void RequestToStop()
  {
    requested_to_stop_ = true;
    ESP_LOGI("amber", "DHW controller requested to stop. Current state: %s", DHWStateToString(state_));
  }

  bool IsRequestedToStop()
  {
    return requested_to_stop_;
  }

  bool IsInIdleState()
  {
    return state_ == DHWState::IDLE;
  }

  void UpdateStateMachine()
  {
    DoSafetyChecks();

    switch (state_)
    {
      case DHWState::UNKNOWN:
        // Restore state based on current conditions on startup.
        if (compressor_controller_->IsRunning())
        {
          SetNextState(DHWState::COMPRESSOR_RUNNING);
        }
        else if (pump_controller_->IsRunning())
        {
          SetNextState(DHWState::PUMP_RUNNING);
        }
        else if (IsBackupHeaterActive())
        {
          SetNextState(DHWState::BACKUP_HEATER_RUNNING);
        }
        else if(!pump_controller_->IsInitialized())
        {
          // Initialize pump to Off
          pump_controller_->Stop();
          // Short settle time before we start logic.
          LeaveStateAndSetNextStateAfterWaitTime(DHWState::IDLE, 60000);
        }
        else 
        {
          SetNextState(DHWState::IDLE);
        }
        break;
      case DHWState::IDLE:
        requested_to_stop_ = false;

        if(!HasDemand())
        {
          break;
        }

        pump_controller_->Start(GetPreferredPumpSpeed());
        SetNextState(DHWState::WAIT_PUMP_RUNNING);
        break;

      case DHWState::WAIT_PUMP_RUNNING:
      {
        if (pump_controller_->IsRunning())
        {
          SetNextState(DHWState::PUMP_RUNNING);
        }
        else 
        {
          // TODO: Implement timeout for when pump does not start.
        }
        break;
      }

      case DHWState::PUMP_RUNNING:
        SetWorkingMode(WORKING_MODE_HEATING);
        compressor_controller_->ApplyCompressorMode(DetermineCompressorMode());
        SetNextState(DHWState::WAIT_COMPRESSOR_RUNNING);
        break;

      case DHWState::WAIT_COMPRESSOR_RUNNING:
        if (compressor_controller_->IsRunning())
        {
          compressor_controller_->RecordStartTime();
          SetNextState(DHWState::COMPRESSOR_RUNNING);
        }
        else 
        {
          // TODO: Implement timeout for when compressor does not start.
        }
        break;

      case DHWState::COMPRESSOR_RUNNING:
        pump_controller_->ApplySpeedChangeIfNeeded(GetPreferredPumpSpeed());

        if(ShouldStartDhwPump())
        {
          ESP_LOGI("amber", "Starting DHW pump");
          id(dhw_pump_relay_switch).turn_on();
          SetNextState(DHWState::WAIT_DHW_PUMP_RUNNING);
          break;
        }

        if (compressor_controller_->HasPassedMinOnTime())
        {
          if(IsTargetTemperatureReached())
          {
            ESP_LOGI("amber", "Target temperature reached, stopping compressor.");
            compressor_controller_->Stop();
            SetNextState(DHWState::WAIT_COMPRESSOR_STOP);
            break;
          }
        }
        else 
        {
          ESP_LOGI("amber", "Minimum compressor on time not reached, not checking for potential stop conditions yet.");
        }

        if (id(sg_ready_max_boost_mode_active_sensor).state)
        {
          ESP_LOGI("amber", "Enabling backup heater (SG Ready max boost active)");
          TurnOnBackupHeater();
          SetNextState(DHWState::WAIT_BACKUP_HEATER_RUNNING);
          break;
        }

        if(IsHeatingSlowerThanMinimumAverageRate())
        {
          ESP_LOGI("amber", "Enabling backup heater");
          TurnOnBackupHeater();
          SetNextState(DHWState::WAIT_BACKUP_HEATER_RUNNING);
          break;
        }

        compressor_controller_->ApplyCompressorMode(DetermineCompressorMode());
        break;
  
      case DHWState::WAIT_DHW_PUMP_RUNNING:
        if(id(dhw_pump_relay_switch).state)
        {
          dhw_pump_start_time_ = millis();
          last_dhw_avg_sample_ms_ = dhw_pump_start_time_;
          defrost_pause_active_ = false;
          dhw_temperature_samples_.clear();
          dhw_temperature_samples_.emplace_back(dhw_pump_start_time_, id(dhw_temperature_tw_sensor).state);
          SetNextState(DHWState::COMPRESSOR_RUNNING);
        }
        break;

      case DHWState::WAIT_COMPRESSOR_STOP:
        if (!compressor_controller_->IsRunning())
        {
          pump_controller_->Stop();
          StopDhwPump();
          SetNextState(DHWState::WAIT_PUMP_STOP);
        }
        break;

      case DHWState::WAIT_PUMP_STOP:
        if (!pump_controller_->IsRunning())
        {
          SetWorkingMode(WORKING_MODE_STANDBY);
          SetNextState(DHWState::IDLE);
        }
        else 
        {
          // TODO: Implement timeout for when pump does not stop.
        }
        break;

      case DHWState::WAIT_BACKUP_HEATER_RUNNING:
        if (IsBackupHeaterActive())
        {
          SetNextState(DHWState::BACKUP_HEATER_RUNNING);
        }
        break;

      case DHWState::BACKUP_HEATER_RUNNING:
        if (IsPredictedTemperatureAboveTarget())
        {
          ESP_LOGI("amber", "Disabling backup heater because predicted temperature is above target");
          TurnOffBackupHeater();
          LeaveStateAndSetNextStateAfterWaitTime(DHWState::COMPRESSOR_RUNNING, BACKUP_HEATER_OFF_SETTLE_TIME_S * 1000UL);
          return;
        }
        break;

      case DHWState::WAIT_FOR_STATE_SWITCH:
      {
        if (defer_state_change_until_ms_ <= millis())
        {
          defer_state_change_until_ms_ = 0;
          SetNextState(deferred_machine_state_);
          deferred_machine_state_ = DHWState::UNKNOWN;
        }
        break;
      }
    }
  }

  bool CanStartDhw()
  {
    if(!HasDemand())
    {
      return false;
    }

    if(id(dhw_legionella_run_active_sensor).state)
    {
      ESP_LOGI("amber", "DHW legionella run active, allowing DHW start regardless of current temperature.");
      return true;
    }

    float current_temperature = id(dhw_temperature_tw_sensor).state;
    float target_temperature = id(current_dhw_setpoint_sensor).state;
    float restart_delta = id(dhw_restart_dhw_delta).state;

    // Only start DHW when the current temperature is below the target temperature minus the restart delta.
    if (current_temperature >= target_temperature - restart_delta)
    {
      return false;
    }

    return true;
  }

  void CheckLegionellaCycle()
  {
    if(!id(legio_enabled_switch).state)
    {
      return;
    }

    auto now = id(my_time).now();
    auto legionella_time = id(next_legionella_run).state_as_esptime();
    if(!id(dhw_legionella_run_active_sensor).state && now >= legionella_time)
    {
      ESP_LOGI("amber", "Starting legionella cycle.");
      id(dhw_legionella_run_active_sensor).publish_state(true);
      auto next_run = id(next_legionella_run).state_as_esptime();
      for(int i=1; i<=id(legio_repeat_days_number).state; i++)
      {
        next_run.increment_day();
      }
      auto legio_start_call = id(next_legionella_run).make_call();
      legio_start_call.set_datetime(next_run);
      legio_start_call.perform();
      ESP_LOGI("amber", "Next legionella cycle scheduled at %04d-%02d-%02d %02d:%02d", 
               id(next_legionella_run).year, id(next_legionella_run).month, 
               id(next_legionella_run).day, id(next_legionella_run).hour, 
               id(next_legionella_run).minute);
    }
    else if(id(dhw_legionella_run_active_sensor).state && !id(dhw_demand_active_sensor).state)
    {
      ESP_LOGI("amber", "Legionella cycle completed, target temperature %.2f°C reached.", id(legio_target_temperature_number).state);
      id(dhw_legionella_run_active_sensor).publish_state(false);
    }
  }
};
