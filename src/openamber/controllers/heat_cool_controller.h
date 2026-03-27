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
  float compressor_heat_pid_ = 0.0f;
  float compressor_cool_pid_ = 0.0f;
  float start_current_temperature_ = 0.0f;

  HeatCoolState state_ = HeatCoolState::IDLE;
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
    
    id(state_machine_state_heat_cool).publish_state(txt);
    ESP_LOGI("amber", "HEAT_COOL state changed: %s", txt);
  }

  void LeaveStateAndSetNextStateAfterWaitTime(HeatCoolState new_state, uint32_t defer_ms)
  {
    deferred_machine_state_ = new_state;
    defer_state_change_until_ms_ = App.get_loop_component_start_time() + defer_ms;
    SetNextState(HeatCoolState::WAIT_FOR_STATE_SWITCH);
  }

  bool IsCoolingDemand()
  {
    return id(cool_demand_active_sensor).state;
  }

  auto& GetActiveCompressorMode()
  {
    return IsCoolingDemand() ? id(cool_compressor_mode) : id(heat_compressor_mode);
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

  int MapPIDToCompressorMode()
  {
    float pid = GetPIDValue();
    float raw = fabsf(pid);
    if (raw > 1.0f)
      raw = 1.0f;

    auto& active_mode = GetActiveCompressorMode();
    int mode_offset = id(compressor_control_select).size() - active_mode.size();
    int amount_of_modes = active_mode.active_index().value() + mode_offset;
    int desired_mode = (int)roundf(raw * (amount_of_modes - 1)) + 1;
    return std::max(1, std::min(desired_mode, (int)amount_of_modes));
  }

  int DetermineSoftStartMode()
  {
    int start_compressor_frequency_mode;

    if (IsCoolingDemand())
    {
      // Cooling: always start at the lowest compressor mode
      start_compressor_frequency_mode = 1;
    }
    else
    {
      // Heating: start higher when further from target
      float current_temperature = id(heat_cool_temperature_tc).state;
      float target = id(pid_heat_temperature_control).target_temperature;
      float delta = fabs(current_temperature - target);
      if (delta < 5.0f)
        start_compressor_frequency_mode = 1;
      else if (delta < 7.0f)
        start_compressor_frequency_mode = 3;
      else if (delta < 10.0f)
        start_compressor_frequency_mode = 4;
      else
        start_compressor_frequency_mode = 5;
    }

    // Cap by the user-selected compressor mode
    auto& active_mode = GetActiveCompressorMode();
    int mode_offset = id(compressor_control_select).size() - active_mode.size();
    int max_allowed = (int)active_mode.active_index().value() + mode_offset;
    return std::min(start_compressor_frequency_mode, max_allowed);
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
    return IsCoolingDemand() ? id(pump_speed_cooling_number).state : id(pump_speed_heating_number).state;
  }

  void CalculateAccumulatedDegreeMinutes()
  {
    uint32_t now = App.get_loop_component_start_time();
    float tc = id(heat_cool_temperature_tc).state;
    float target = id(pid_heat_temperature_control).target_temperature;
    const float diff = std::max(0.0f, target - tc);
    uint32_t dt_ms = now - backup_degmin_last_ms_;
    backup_degmin_last_ms_ = now;
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
    uint32_t now = App.get_loop_component_start_time();
    float current_temperature = id(heat_cool_temperature_tc).state;
    float target = id(pid_heat_temperature_control).target_temperature;

    uint32_t dt_ms = now - backup_degmin_last_ms_;
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
    backup_degmin_last_ms_ = now;
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
    bool has_active_demand = false;
  
    // When compressor is running only consider the active demand for the current mode.
    if(compressor_controller_->IsRunning())
    {
      has_active_demand = IsCoolingDemand() ? id(cool_demand_active_sensor).state : id(heat_demand_active_sensor).state || id(frost_protection_stage2_active).state;
    }
    else 
    {
      has_active_demand = id(heat_demand_active_sensor).state || id(cool_demand_active_sensor).state || id(frost_protection_stage2_active).state;
    }

    if (!has_active_demand)
    {
      return false;
    }

    if(requested_to_stop_)
    {
      ESP_LOGI("amber", "Compressor stop requested, ignoring compressor demand until stopped.");
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

    // If temperature difference between Tuo and Tui is above 8 degrees while compressor is running, stop compressor to avoid damage
    if (fabsf(id(outlet_temperature_tuo).state - id(inlet_temperature_tui).state) > 8.0f && compressor_controller_->IsRunning())
    {
      ESP_LOGW("amber", "Safety check: Temperature difference between Tuo and Tui is above 8 degrees while compressor is running, stopping compressor to avoid damage.");
      compressor_controller_->Stop();
      StopPumps();
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

    SetPidController(working_mode);
  }
  
  void SetPidController(int working_mode)
  {
    auto heat_call = id(pid_heat_temperature_control).make_call();
    heat_call.set_mode(working_mode == WORKING_MODE_HEAT ? climate::CLIMATE_MODE_HEAT : climate::CLIMATE_MODE_OFF);
    heat_call.perform();
    ESP_LOGI("amber", "PID climate mode switched to HEATING");
    
    auto cool_call = id(pid_cool_temperature_control).make_call();
    cool_call.set_mode(working_mode == WORKING_MODE_COOLING ? climate::CLIMATE_MODE_COOL : climate::CLIMATE_MODE_OFF);
    cool_call.perform();
    ESP_LOGI("amber", "PID climate mode switched to COOLING");
   }
public:
  HeatCoolController(PumpController* pump_controller, CompressorController* compressor_controller)
    : pump_controller_(pump_controller), compressor_controller_(compressor_controller) {}

  void SetHeatPIDValue(float pid_value)
  {
    compressor_heat_pid_ = pid_value;
  }

  void SetCoolPIDValue(float pid_value)
  {
    compressor_cool_pid_ = pid_value;
  }

  float GetPIDValue()
  {
    return IsCoolingDemand() ? compressor_cool_pid_ : compressor_heat_pid_;
  }

  void InitializeBackupDegMinTracking()
  {
    backup_degmin_last_ms_ = App.get_loop_component_start_time();
  }

  int DetermineCompressorMode()
  {
    const uint32_t now = App.get_loop_component_start_time();
    int desired_compressor_mode = MapPIDToCompressorMode();
    auto current_compressor_mode = id(compressor_control_select).active_index().value();
    float current_temperature = id(heat_cool_temperature_tc).state;
    float target = GetPidController().target_temperature;
    float dt = current_temperature - target;

    // Deadband on Tc to avoid too frequent changes around setpoint
    if (fabsf(dt) < DEAD_BAND_DT)
    {
      ESP_LOGD("amber", "ΔT=%.2f°C within deadband, compressor mode remains at %d", dt, current_compressor_mode);
      return current_compressor_mode;
    }

    // Guard: never modulate up when temperature has already overshot the setpoint
    const float TEMP_MARGIN = 0.3f;
    bool overshoot = IsCoolingDemand()
      ? (current_temperature < (target - TEMP_MARGIN))
      : (current_temperature > (target + TEMP_MARGIN));
    if (overshoot && desired_compressor_mode > current_compressor_mode)
    {
      ESP_LOGW("amber", "Temperature has overshot target and PID controller wanted to move to a higher compressor mode.");
      return current_compressor_mode;
    }

    auto& active_mode = GetActiveCompressorMode();
    int mode_offset = id(compressor_control_select).size() - active_mode.size();
    int capped_mode_index = std::min(desired_compressor_mode, (int)active_mode.active_index().value() + mode_offset);

    ESP_LOGI("amber", "PID %.2f -> mode %d (previous mode %d)", GetPIDValue(), capped_mode_index, current_compressor_mode);
    return capped_mode_index;
  }

  bool ShouldStopCompressor()
  {
    if(!HasCompressorDemand())
    {
      return true;
    }

    float target_temperature = GetPidController().target_temperature;
    float current_temperature = id(heat_cool_temperature_tc).state;

    // Overshoot is positive when temp has passed the setpoint in the working direction
    // Heating: too hot (Tc > target), Cooling: too cold (Tc < target)
    float stop_delta = IsCoolingDemand() ? id(compressor_stop_delta_cooling).state : id(compressor_stop_delta).state;
    float overshoot = IsCoolingDemand() ? (target_temperature - current_temperature) : (current_temperature - target_temperature);

    if (overshoot >= stop_delta)
    {
      if (id(oil_return_cycle_active).state)
      {
        ESP_LOGI("amber", "Not stopping compressor because oil return cycle is active.");
        return false;
      }

      ESP_LOGI("amber", "Stopping compressor because it reached delta %.2f°C (overshoot=%.2f°C).", stop_delta, overshoot);
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
          StartPumpP1IfNeeded();
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
          StopPumps();
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

        // Start condition based on target temperature and start_compressor_delta
        float current_temperature = id(heat_cool_temperature_tc).state;
        float target_temperature = GetPidController().target_temperature;
        bool should_start;
        if (IsCoolingDemand())
        {
          // Cooling: start when supply temp is above target + start_delta
          float start_temperature = target_temperature + id(compressor_start_delta).state;
          should_start = current_temperature > start_temperature;
          if (!should_start)
          {
            ESP_LOGI("amber", "Not starting compressor because supply temperature (%.2f) is below cooling start temperature (%.2f)", current_temperature, start_temperature);
            break;
          }
        }
        else
        {
          // Heating: start when supply temp is below target - start_delta
          float start_temperature = target_temperature - id(compressor_start_delta).state;
          if (current_temperature >= start_temperature && !id(frost_protection_stage2_active).state)
          {
            ESP_LOGI("amber", "Not starting compressor because supply temperature (%.2f) is above heating start temperature (%.2f)", current_temperature, start_temperature);
            break;
          }
        }

        SetWorkingMode(IsCoolingDemand() ? WORKING_MODE_COOLING : WORKING_MODE_HEATING);
        compressor_controller_->ApplyCompressorMode(DetermineSoftStartMode());
        SetNextState(HeatCoolState::WAIT_COMPRESSOR_RUNNING);
        break;
      }

      case HeatCoolState::WAIT_COMPRESSOR_RUNNING:
      {
        if (compressor_controller_->IsRunning())
        {
          GetPidController().reset_integral_term();
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
          backup_degmin_last_ms_ = App.get_loop_component_start_time();
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
          StopPumpP1IfNeeded();
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
          ESP_LOGI("amber", "Minimum compressor on time not reached, not checking for potential stop conditions yet.");
        }

        // Backup heater logic only applies to heating mode
        if (!IsCoolingDemand())
        {
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
        }
        
        compressor_controller_->ApplyCompressorMode(DetermineCompressorMode());
        break;
      }

      case HeatCoolState::WAIT_COMPRESSOR_STOP:
      {
        if (!compressor_controller_->IsRunning())
        {
          start_current_temperature_ = 0;

          // When not requested to stop, let the pump run for another cycle.
          if(!requested_to_stop_)
          {
            pump_controller_->RestartPumpInterval();
          }
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
        float target_temperature = id(pid_heat_temperature_control).target_temperature;
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

        GetPidController().reset_integral_term();
        StartPumpP1IfNeeded();

        if (id(temperature_outside_ta).state <= id(defrost_backup_heater_boost_temperature_sensor).state)
        {
          ESP_LOGI("amber", "Defrost ended, enabling backup heater as configured for outside temperature %.2f°C to boost temperature back to setpoint.", id(temperature_outside_ta).state);
          TurnOnBackupHeater();
          SetNextState(HeatCoolState::WAIT_BACKUP_HEATER_RUNNING);
        }
        else
        {
          float current_temperature = id(heat_cool_temperature_tc).state;
          float target_temperature = id(pid_heat_temperature_control).target_temperature;
          bool should_use_defrost_recovery_mode = current_temperature < target_temperature - 3.0f;
          if(should_use_defrost_recovery_mode)
          {
            compressor_controller_->ApplyDefrostRecoveryMode();
          }
          LeaveStateAndSetNextStateAfterWaitTime(HeatCoolState::COMPRESSOR_RUNNING, COMPRESSOR_SETTLE_TIME_AFTER_DEFROST_S * 1000UL);
        }
        break;
      }

      case HeatCoolState::WAIT_FOR_STATE_SWITCH:
      {
        uint32_t now = App.get_loop_component_start_time();
        if (defer_state_change_until_ms_ > now)
        {
          ESP_LOGD("amber", "Waiting for state switch, transitioning to next state in %lu ms", defer_state_change_until_ms_ - now);
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

  void StopPumps()
  {
    pump_controller_->Stop();
    StopPumpP1IfNeeded();
  }

  void StartPumpP1IfNeeded()
  {
    if(id(pump_p1_enabled).state && !id(pump_p1_relay_switch).state)
    {
      ESP_LOGI("amber", "Starting pump P1");
      id(pump_p1_relay_switch).turn_on();
    }
  }

  void StopPumpP1IfNeeded()
  {
    if(id(pump_p1_enabled).state && id(pump_p1_relay_switch).state)
    {
      ESP_LOGI("amber", "Stopping pump P1");
      id(pump_p1_relay_switch).turn_off();
    }
  }

  pid::PIDClimate& GetPidController()
  {
    return IsCoolingDemand() ? id(pid_cool_temperature_control) : id(pid_heat_temperature_control);
  }
};
