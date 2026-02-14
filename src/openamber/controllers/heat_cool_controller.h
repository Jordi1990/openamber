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

using namespace esphome;

enum class HeatCoolState
{
  UNKNOWN,
  IDLE,
  WAIT_PUMP_RUNNING,
  PUMP_RUNNING,
  WAIT_COMPRESSOR_RUNNING,
  COMPRESSOR_SOFTSTART,
  COMPRESSOR_RUNNING,
  WAIT_BACKUP_HEATER_RUNNING,
  BACKUP_HEATER_RUNNING,
  WAIT_COMPRESSOR_STOP,
  WAIT_PUMP_STOP,
  DEFROSTING,
  WAIT_FOR_STATE_SWITCH,
};

class HeatCoolController
{
private:
  uint32_t backup_degmin_last_ms_ = 0;
  float accumulated_backup_degmin_ = 0.0;
  float temperature_rate_c_per_min_ = 0.0;
  float last_temperature_for_rate_ = 0.0;
  float compressor_pid_ = 0.0f;
  float start_current_temperature_ = 0.0f;

  HeatCoolState state_ = HeatCoolState::UNKNOWN;
  HeatCoolState deferred_machine_state_;
  uint32_t defer_state_change_until_ms_;
  bool requested_to_stop_ = false;

  CompressorController* compressor_controller_;
  PumpController* pump_controller_;

  const char* StateToString(HeatCoolState state) const
  {
    switch (state)
    {
      case HeatCoolState::IDLE:
        return "Idle";
      case HeatCoolState::WAIT_PUMP_RUNNING:
        return "Wait pump running";
      case HeatCoolState::PUMP_RUNNING:
        return "Pump running";
      case HeatCoolState::WAIT_COMPRESSOR_RUNNING:
        return "Wait compressor running";
      case HeatCoolState::COMPRESSOR_RUNNING:
        return "Compressor running";
      case HeatCoolState::WAIT_BACKUP_HEATER_RUNNING:
        return "Wait backup heater running";
      case HeatCoolState::BACKUP_HEATER_RUNNING:
        return "Backup heater running";
      case HeatCoolState::DEFROSTING:
        return "Defrosting";
      case HeatCoolState::COMPRESSOR_SOFTSTART:
        return "Compressor softstart";
      case HeatCoolState::WAIT_COMPRESSOR_STOP:
        return "Wait compressor stop";
      case HeatCoolState::WAIT_PUMP_STOP:
        return "Wait pump stop";
      case HeatCoolState::WAIT_FOR_STATE_SWITCH:
        return "Wait for state switch";
      default:
        return "Unknown";
    }
  }

  void SetNextState(HeatCoolState new_state)
  {
    state_ = new_state;
    const char* txt = StateToString(new_state);
    
    id(state_machine_state).publish_state(txt);
    ESP_LOGI("amber", "HP state changed: %s", txt);
  }

  void LeaveStateAndSetNextStateAfterWaitTime(HeatCoolState new_state, uint32_t defer_ms)
  {
    deferred_machine_state_ = new_state;
    defer_state_change_until_ms_ = millis() + defer_ms;
    SetNextState(HeatCoolState::WAIT_FOR_STATE_SWITCH);
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

  int MapPIDToCompressorMode(float pid)
  {
    float raw = fabsf(pid);
    if (raw > 1.0f)
      raw = 1.0f;

    int mode_offset = id(compressor_control_select).size() - id(heat_compressor_mode).size();
    int amount_of_modes = id(heat_compressor_mode).active_index().value() + mode_offset;
    int desired_mode = (int)roundf(raw * (amount_of_modes - 1)) + 1;
    return std::max(1, std::min(desired_mode, (int)amount_of_modes));
  }

  int DetermineSoftStartMode()
  {
    float current_temperature = id(heat_cool_temperature_tc).state;
    float target = id(pid_heat_cool_temperature_control).target_temperature;
    float supply_temperature_delta = current_temperature - target;

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

  bool IsAccumulatedDegreeMinutesReached()
  {
      if (accumulated_backup_degmin_ >= id(backup_heater_degmin_threshold).state)
      {
        ESP_LOGI("amber", "Enabling backup heater (degree/min limit reached: %.2f)", accumulated_backup_degmin_);
        return true;
      }
      return false; 
  }

  float GetPreferredPumpSpeed() {
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
      if(id(backup_heater_degmin_current_sensor).state != 0.0f)
      {
        id(backup_heater_degmin_current_sensor).publish_state(0.0f);
        ESP_LOGI("amber", "Resetting accumulated degree/minutes because diff=%.2f or not at max compressor mode.", diff);
      }
      return;
    }

    if (dt_min <= 0.0f)
    {
      return;
    }

    accumulated_backup_degmin_ += diff * dt_min;
    id(backup_heater_degmin_current_sensor).publish_state(accumulated_backup_degmin_);
    ESP_LOGD("amber", "Backup heater degree/minutes updated: +%.2f -> %.2f", diff * dt_min, accumulated_backup_degmin_);
  }

  float CalculateBackupHeaterPredictedTemperature()
  {
    float current_temperature = id(heat_cool_temperature_tc).state;
    float target = id(pid_heat_cool_temperature_control).target_temperature;

    uint32_t dt_ms = millis() - backup_degmin_last_ms_;
    const float dt_min = (float)dt_ms / 60000.0f;
    if (dt_min <= 0.0f)
    {
      ESP_LOGW("amber", "Delta time for backup heater degree/min rate calculation is zero or negative, skipping update.");
      return 0.0f;
    }
    // Calculate degree/minute rate.
    float rate = (current_temperature - last_temperature_for_rate_) / dt_min;
    // Low-pass filter the rate to avoid spikes.
    double a = 0.2f;
    temperature_rate_c_per_min_ = (1 - a) * temperature_rate_c_per_min_ + a * rate;
    last_temperature_for_rate_ = current_temperature;
    // Predict future Tc based on current rate.
    float predicted_temperature = current_temperature + temperature_rate_c_per_min_ * ((float)BACKUP_HEATER_LOOKAHEAD_S / 60.0f);
    backup_degmin_last_ms_ = millis();
    ESP_LOGI("amber", "Backup heater temperature rate/min updated: %.2f (Predicted Temperature: %.2f°C)", temperature_rate_c_per_min_, predicted_temperature);
    return predicted_temperature;
  }

  void ResetAccumulatedDegMin()
  {
    accumulated_backup_degmin_ = 0.0f;
  }

  bool HasPumpDemand()
  {
      return id(heat_demand_active_sensor).state ||
             id(cool_demand_active_sensor).state ||
             id(frost_protection_stage1_active).state ||
             id(frost_protection_stage2_active).state;
  }

  bool HasCompressorDemand()
  {
    float current_temperature = id(heat_cool_temperature_tc).state;
    float target_temperature = id(pid_heat_cool_temperature_control).target_temperature;

    bool has_active_demand = id(heat_demand_active_sensor).state || id(cool_demand_active_sensor).state || id(frost_protection_stage2_active).state;
    if (!has_active_demand)
    {
      return false;
    }

    if(requested_to_stop_)
    {
      ESP_LOGI("amber", "Compressor stop requested, ignoring compressor demand until stopped.", current_temperature, target_temperature);
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

  void DoSafetyChecks()
  {
    // If pump is not active while compressor is running, stop compressor to avoid damage
    if (compressor_controller_->IsRunning() && !pump_controller_->IsRunning())
    {
      ESP_LOGW("amber", "Safety check: Pump is not active while compressor is running, stopping compressor to avoid damage.");
      compressor_controller_->Stop();
      SetNextState(HeatCoolState::IDLE);
      return;
    }

    // If Tuo - Tui is above 8 degrees while compressor is running, stop compressor to avoid damage
    if (id(outlet_temperature_tuo).state - id(inlet_temperature_tui).state > 8.0f && compressor_controller_->IsRunning())
    {
      ESP_LOGW("amber", "Safety check: Temperature difference between Tuo and Tui is above 8 degrees while compressor is running, stopping compressor to avoid damage.");
      compressor_controller_->Stop();
      pump_controller_->Stop();
      SetNextState(HeatCoolState::IDLE);
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
public:
  HeatCoolController(PumpController* pump_controller, CompressorController* compressor_controller)
    : pump_controller_(pump_controller), compressor_controller_(compressor_controller) {}

  void SetPIDValue(float pid_value)
  {
    //ESP_LOGD("amber", "Writing PID value: %.2f", pid_value);
    compressor_pid_ = pid_value;
  }

  void InitializeBackupDegMinTracking()
  {
    backup_degmin_last_ms_ = millis();
  }

  int DetermineCompressorMode()
  {
    const uint32_t now = millis();
    int desired_compressor_mode = MapPIDToCompressorMode(compressor_pid_);
    auto current_compressor_mode = id(compressor_control_select).active_index().value();
    float current_temperature = id(heat_cool_temperature_tc).state;
    float target = id(pid_heat_cool_temperature_control).target_temperature;
    float dt = current_temperature - target;

    // Deadband on Tc to avoid too frequent changes around setpoint
    if (fabsf(dt) < DEAD_BAND_DT)
    {
      ESP_LOGD("amber", "ΔT=%.2f°C within deadband, compressor mode remains at %d", dt, current_compressor_mode);
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

  bool ShouldStopCompressor()
  {
    if(!HasCompressorDemand())
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

    return false;
  }

  void Stop()
  {
    ESP_LOGI("amber", "Stopping compressor as requested by controller.");
    compressor_controller_->Stop();
    SetNextState(HeatCoolState::WAIT_COMPRESSOR_STOP);
  }

  void UpdateStateMachine()
  {
    DoSafetyChecks();

    switch (state_)
    {
      case HeatCoolState::UNKNOWN:
      {
        // Restore state based on current conditions on startup.
        if (compressor_controller_->IsRunning())
        {
          SetNextState(HeatCoolState::COMPRESSOR_RUNNING);
        }
        else if (pump_controller_->IsRunning())
        {
          SetNextState(HeatCoolState::PUMP_RUNNING);
        }
        else if (IsBackupHeaterActive())
        {
          SetNextState(HeatCoolState::BACKUP_HEATER_RUNNING);
        }
        else
        {
          // Initialize pump to Off
          pump_controller_->Stop();
          SetNextState(HeatCoolState::IDLE);
        }
        break;
      }

      case HeatCoolState::IDLE:
      {
        requested_to_stop_ = false;

        if(!HasPumpDemand())
        {
          break;
        }

        // Start pump on interval or if there is compressor demand.
        if (pump_controller_->ShouldStartNextPumpCycle() || HasCompressorDemand())
        {
          pump_controller_->Start(GetPreferredPumpSpeed());
          SetNextState(HeatCoolState::WAIT_PUMP_RUNNING);
        }
        break;
      }

      case HeatCoolState::WAIT_PUMP_RUNNING:
      {
        if (pump_controller_->IsRunning())
        {
          SetNextState(HeatCoolState::PUMP_RUNNING);
        }
        else 
        {
          // TODO: Implement timeout for when pump does not start.
        }
        break;
      }

      case HeatCoolState::PUMP_RUNNING:
      {
        pump_controller_->ApplySpeedChangeIfNeeded(GetPreferredPumpSpeed());

        // Stop if there is no demand and pump interval is finished.
        if (!HasCompressorDemand() && pump_controller_->IsIntervalCycleFinished())
        {
          ESP_LOGI("amber", "Stopping pump (interval cycle finished)");
          pump_controller_->Stop();
          SetNextState(HeatCoolState::WAIT_PUMP_STOP);
          break;
        }

        // Settle temperature before starting compressor.
        if (!pump_controller_->IsPumpSettled())
        {
          ESP_LOGI("amber", "Not starting compressor because temperature needs to stabilize (pump on time too short)");
          break;
        }

        if (!compressor_controller_->HasPassedMinOffTime())
        {
          ESP_LOGI("amber", "Not starting compressor because minimum compressor time off is not reached.");
          break;
        }

        if(!HasCompressorDemand())
        {
          break;
        }

        SetWorkingMode(id(heat_demand_active_sensor).state ? WORKING_MODE_HEATING : WORKING_MODE_COOLING);
        compressor_controller_->ApplyCompressorMode(DetermineSoftStartMode());
        SetNextState(HeatCoolState::WAIT_COMPRESSOR_RUNNING);
        break;
      }

      case HeatCoolState::WAIT_COMPRESSOR_RUNNING:
      {
        if (compressor_controller_->IsRunning())
        {
          id(pid_heat_cool_temperature_control).reset_integral_term();
          compressor_controller_->RecordStartTime();
          SetNextState(HeatCoolState::COMPRESSOR_SOFTSTART);
        }
        else 
        {
          // TODO: Implement timeout for when compressor does not start.
        }
        break;
      }

      case HeatCoolState::COMPRESSOR_SOFTSTART:
      {
        if (compressor_controller_->HasPassedSoftStartDuration())
        {
          backup_degmin_last_ms_ = millis();
          SetNextState(HeatCoolState::COMPRESSOR_RUNNING);
        }
        break;
      }

      case HeatCoolState::COMPRESSOR_RUNNING:
      {
        pump_controller_->ApplySpeedChangeIfNeeded(GetPreferredPumpSpeed());
        if(start_current_temperature_ == 0.0f)
        {
          start_current_temperature_ = id(heat_cool_temperature_tc).state;
        }

        if (id(defrost_active_sensor).state)
        {
          SetNextState(HeatCoolState::DEFROSTING);
          break;
        }

        if (compressor_controller_->HasPassedMinOnTime())
        {
          if(ShouldStopCompressor())
          {
            ESP_LOGI("amber", "Stopping compressor because there is no demand or a temperature overshoot.");
            compressor_controller_->Stop();
            SetNextState(HeatCoolState::WAIT_COMPRESSOR_STOP);
            break;
          }
        }
        else 
        {
          ESP_LOGI("amber", "Minimum compressor on time not reached, cannot stop compressor even if there is no demand or temperature overshoot.");
        }

        if (id(sg_ready_max_boost_mode_active_sensor).state)
        {
          ESP_LOGI("amber", "Enabling backup heater (SG Ready max boost active)");
          TurnOnBackupHeater();
          SetNextState(HeatCoolState::WAIT_BACKUP_HEATER_RUNNING);
          break;
        }

        CalculateAccumulatedDegreeMinutes();
        if(IsAccumulatedDegreeMinutesReached())
        {
          TurnOnBackupHeater();
          ResetAccumulatedDegMin();
          SetNextState(HeatCoolState::WAIT_BACKUP_HEATER_RUNNING);
          break;
        }
        
        compressor_controller_->ApplyCompressorMode(DetermineCompressorMode());
        break;
      }

      case HeatCoolState::WAIT_COMPRESSOR_STOP:
      {
        if (!compressor_controller_->IsRunning())
        {
          start_current_temperature_ = 0;
          pump_controller_->RestartPumpInterval();
          SetNextState(HeatCoolState::PUMP_RUNNING);
        }
        break;
      }

      case HeatCoolState::WAIT_PUMP_STOP:
      {
        if (!pump_controller_->IsRunning())
        {
          SetWorkingMode(WORKING_MODE_STANDBY);
          SetNextState(HeatCoolState::IDLE);
        }
        else 
        {
          // TODO: Implement timeout for when pump does not stop.
        }
        break;
      }

      case HeatCoolState::WAIT_BACKUP_HEATER_RUNNING:
      {
        if (IsBackupHeaterActive())
        {
          SetNextState(HeatCoolState::BACKUP_HEATER_RUNNING);
        }
        break;
      }

      case HeatCoolState::BACKUP_HEATER_RUNNING:
      {
        float predicted_temperature = CalculateBackupHeaterPredictedTemperature();
        float target_temperature = id(pid_heat_cool_temperature_control).target_temperature;
        // Disable backup heater if predicted Temperature is above target.
        if (predicted_temperature >= target_temperature + 2.0f)
        { // Margin to buffer some heat to reduce undershoot.
          ESP_LOGI("amber", "Disabling backup heater (predicted Temperature %.2f°C above target %.2f°C)", predicted_temperature, target_temperature);
          TurnOffBackupHeater();
          LeaveStateAndSetNextStateAfterWaitTime(HeatCoolState::COMPRESSOR_RUNNING, BACKUP_HEATER_OFF_SETTLE_TIME_S * 1000UL);
        }
        break;
      }

      case HeatCoolState::DEFROSTING:
      {
        if (id(defrost_active_sensor).state)
        {
          ESP_LOGI("amber", "Defrost busy, waiting before making changes to compressor.");
          break;
        }

        if (id(temperature_outside_ta).state <= id(defrost_backup_heater_boost_temperature_sensor).state)
        {
          ESP_LOGI("amber", "Defrost ended, enabling backup heater as configured for outside temperature %.2f°C to boost temperature back to setpoint.", id(temperature_outside_ta).state);
          TurnOnBackupHeater();
          SetNextState(HeatCoolState::WAIT_BACKUP_HEATER_RUNNING);
        }
        else
        {
          compressor_controller_->ApplyDefrostRecoveryMode();
          LeaveStateAndSetNextStateAfterWaitTime(HeatCoolState::COMPRESSOR_RUNNING, COMPRESSOR_SETTLE_TIME_AFTER_DEFROST_S * 1000UL);
        }
        id(pid_heat_cool_temperature_control).reset_integral_term();
        break;
      }

      case HeatCoolState::WAIT_FOR_STATE_SWITCH:
      {
        if (defer_state_change_until_ms_ > millis())
        {
          ESP_LOGD("amber", "Waiting for state switch, transitioning to next state in %lu ms", defer_state_change_until_ms_ - millis());
        }
        else
        {
          defer_state_change_until_ms_ = 0;
          SetNextState(deferred_machine_state_);
          deferred_machine_state_ = HeatCoolState::UNKNOWN;
        }
        break;
      }
    }
  }

  void RequestToStop()
  {
    requested_to_stop_ = true;
    ESP_LOGI("amber", "Heat/Cool controller requested to stop. Current state: %s", StateToString(state_));
  }

  bool IsRequestedToStop()
  {
    return requested_to_stop_;
  }

  bool IsInIdleState()
  {
    return state_ == HeatCoolState::IDLE;
  }
};
