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

#include "base_controller.h"
#include "mixing_valve_actuator.h"

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

class HeatCoolController : public BaseController<HeatCoolState>
{
private:
  uint32_t backup_degmin_last_ms_ = 0;
  float accumulated_backup_degmin_ = 0.0;
  float temperature_rate_c_per_min_ = 0.0;
  float last_temperature_for_rate_ = 0.0;
  float compressor_heat_pid_ = 0.0f;
  float compressor_cool_pid_ = 0.0f;
  float start_current_temperature_ = 0.0f;
  uint32_t pump_flow_missing_since_ms_ = 0;
  MixingValveActuator* mixing_valve_zone1_ = nullptr;
  MixingValveActuator* mixing_valve_zone2_ = nullptr;

  void ApplyMixingValvePositions()
  {
    bool mixing_z1_enabled = id(mixing_valve_zone1_enabled).state;
    bool mixing_z2_enabled = id(mixing_valve_zone2_enabled).state;
    if (!mixing_z1_enabled && !mixing_z2_enabled) return;

    bool z1_demand = id(heat_demand_active_sensor_zone_1).state;
    bool z2_demand = id(heat_demand_active_sensor_zone_2).state;
    bool frost_protection_active = id(frost_protection_stage1_active).state || id(frost_protection_stage2_active).state;

    float target_z1 = (id(heat_mode_select).current_option() == "Stooklijn")
                        ? id(heat_curve_calculated_setpoint).state
                        : id(manual_setpoint).state;
    float target_z2 = id(heat_curve_z2_calculated_setpoint).state;

    // Zone 1
    if (mixing_z1_enabled)
    {
      if (!z1_demand)
      {
        if (frost_protection_active)
        {
          mixing_valve_zone1_->OpenValve();
        }
        else
        {
          mixing_valve_zone1_->CloseValve();
        }
        id(pid_mixing_valve_zone1).reset_integral_term();
      }
      else if (z2_demand && target_z1 < (target_z2 - 0.1f))
      {
        mixing_valve_zone1_->ApplyValvePosition();
      }
      else
      {
        mixing_valve_zone1_->OpenValve();
        id(pid_mixing_valve_zone1).reset_integral_term();
      }
    }

    // Zone 2
    if (mixing_z2_enabled)
    {
      if (!z2_demand)
      {
        if (frost_protection_active)
        {
          mixing_valve_zone2_->OpenValve();
        }
        else
        {
          mixing_valve_zone2_->CloseValve();
        }
        id(pid_mixing_valve_zone2).reset_integral_term();
      }
      else if (z1_demand && target_z2 < (target_z1 - 0.1f))
      {
        mixing_valve_zone2_->ApplyValvePosition();
      }
      else
      {
        mixing_valve_zone2_->OpenValve();
        id(pid_mixing_valve_zone2).reset_integral_term();
      }
    }
  }

  void ResetMixingValvesIntegralTerm()
  {
    if (id(mixing_valve_zone1_enabled).state)
    {
      id(pid_mixing_valve_zone1).reset_integral_term();
    }
    if (id(mixing_valve_zone2_enabled).state)
    {
      id(pid_mixing_valve_zone2).reset_integral_term();
    }
  }

  void OpenMixingValves()
  {
    ResetMixingValvesIntegralTerm();
    if (id(mixing_valve_zone1_enabled).state)
    {
      mixing_valve_zone1_->OpenValve();
    }
    if (id(mixing_valve_zone2_enabled).state)
    {
      mixing_valve_zone2_->OpenValve();
    }
  }

  void CloseMixingValves()
  {
    ResetMixingValvesIntegralTerm();
    if (id(mixing_valve_zone1_enabled).state)
    {
      mixing_valve_zone1_->CloseValve();
    }
    if (id(mixing_valve_zone2_enabled).state)
    {
      mixing_valve_zone2_->CloseValve();
    }
  }

  const char* StateToString(HeatCoolState state) const override
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

  void PublishState(const char* state_text) override
  {
    id(state_machine_state_heat_cool).publish_state(state_text);
  }

  const char* LogTag() const override
  {
    return "HEAT_COOL";
  }

  bool IsCoolingDemand()
  {
    return id(cool_demand_active_sensor).state;
  }

  auto& GetActiveCompressorMode()
  {
    return IsCoolingDemand() ? id(cool_compressor_mode) : id(heat_compressor_mode);
  }

  float GetControlTemperature()
  {
    // When mixing valve are enabled, always use Tc
    bool mixing_valves_enabled = id(mixing_valve_zone1_enabled).state || id(mixing_valve_zone2_enabled).state;
    if (mixing_valves_enabled)
    {
      return id(heat_cool_temperature_tc).state;
    }

    auto selected_source = id(heat_cool_control_temperature_source_select).active_index();
    int source_index = (selected_source.has_value() && selected_source.value() == 1) ? 1 : 0;
    if (source_index == 1)
    {
      return id(heat_cool_temperature_tv1).state;
    }

    return id(heat_cool_temperature_tc).state;
  }

  int GetMaxAllowedCompressorMode()
  {
    auto& active_mode = GetActiveCompressorMode();
    int mode_offset = id(compressor_control_select).size() - active_mode.size();
    if (IsCoolingDemand())
    {
      // Cooling maximum range is limited to make the mode selection based on the lower frequencies, maximum compressor modes should not be needed for cooling.
      mode_offset -= 3;
    }
    int max_allowed = (int)active_mode.active_index().value() + mode_offset;
    if (id(silent_mode_active_sensor).state)
    {
      int silent_offset = id(compressor_control_select).size() - id(silent_mode_compressor_mode).size();
      if (IsCoolingDemand())
      {
        silent_offset -= 3;
      }
      int silent_limit = (int)id(silent_mode_compressor_mode).active_index().value_or(3) + silent_offset;
      max_allowed = std::min(max_allowed, silent_limit);
    }
    return max_allowed;
  }

  int MapPIDToCompressorMode()
  {
    float pid = GetPIDValue();
    float raw = fabsf(pid);
    if (raw > 1.0f)
      raw = 1.0f;

    int amount_of_modes = GetMaxAllowedCompressorMode();
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
      float current_temperature = GetControlTemperature();
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
    int max_allowed = GetMaxAllowedCompressorMode();
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

  void CalculateAccumulatedDegreeMinutes()
  {
    uint32_t now = App.get_loop_component_start_time();
    float control_temperature = GetControlTemperature();
    float target = id(pid_heat_temperature_control).target_temperature;
    const float diff = std::max(0.0f, target - control_temperature);
    uint32_t dt_ms = now - backup_degmin_last_ms_;
    backup_degmin_last_ms_ = now;
    const float dt_min = (float)dt_ms / 60000.0f;

    int max_compressor_mode = GetMaxAllowedCompressorMode();
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
    float current_temperature = GetControlTemperature();
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
    // Predict future control temperature based on current rate.
    float predicted_temperature = current_temperature + temperature_rate_c_per_min_ * ((float)BACKUP_HEATER_LOOKAHEAD_S / 60.0f);
    backup_degmin_last_ms_ = now;
    ESP_LOGI("amber", "Backup heater temperature rate/min updated: %.2f (Predicted Temperature: %.2f°C)", temperature_rate_c_per_min_, predicted_temperature);
    return predicted_temperature;
  }

  void ResetAccumulatedDegMin()
  {
    accumulated_backup_degmin_ = 0.0f;
    id(backup_heater_degmin_current_sensor).publish_state(0.0f);
  }

  bool HasPumpDemand()
  {
      return id(heat_demand_active_sensor).state ||
             id(cool_demand_active_sensor).state ||
             id(frost_protection_stage1_active).state ||
             id(frost_protection_stage2_active).state;
  }

  /// @brief True when the control temperature asks for the compressor (start delta). The minimum off time is
  /// checked separately: it only delays the start, it is no reason to end the pump cycle.
  bool IsStartTemperatureReached()
  {
      // Start condition based on target temperature and start_compressor_delta
      float current_temperature = GetControlTemperature();
      if (IsCoolingDemand())
      {
        // Cooling: start when supply temp is above target + start_delta
        float target_temperature = id(pid_cool_temperature_control).target_temperature;
        float start_temperature = target_temperature + id(compressor_start_delta_cooling).state;
        if (current_temperature <= start_temperature)
        {
          ESP_LOGI("amber", "Not starting compressor because supply temperature (%.2f) is below cooling start temperature (%.2f)", current_temperature, start_temperature);
          return false;
        }
      }
      else
      {
        // Heating: start when supply temp is below target - start_delta
        float target_temperature = id(pid_heat_temperature_control).target_temperature;
        float start_temperature = target_temperature - id(compressor_start_delta_heating).state;
        if (current_temperature >= start_temperature && !id(frost_protection_stage2_active).state)
        {
          ESP_LOGI("amber", "Not starting compressor because supply temperature (%.2f) is above heating start temperature (%.2f)", current_temperature, start_temperature);
          return false;
        }
      }

      return true;
  }

  bool HasCompressorDemand()
  {
    bool has_active_demand = false;
  
    // When compressor is running only consider the active demand for the current mode.
    if(compressor_controller_->IsRunning())
    {
      has_active_demand = IsWorkingMode(WORKING_MODE_COOLING) ? id(cool_demand_active_sensor).state : id(heat_demand_active_sensor).state || id(frost_protection_stage2_active).state;
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


  bool PerformSafetyChecks()
  {
    if(id(error_active).state && state_ != HeatCoolState::IDLE)
    {
      ESP_LOGI("amber", "Error active, stopping heatpump.");
      return true;
    }

    // Stop compressor only if pump flow stays missing for the configured delay while compressor runs.
    if (state_ != HeatCoolState::IDLE && compressor_controller_->IsRunning() && !pump_controller_->IsRunning())
    {
      const uint32_t now = App.get_loop_component_start_time();
      const uint32_t flow_switch_delay_ms = static_cast<uint32_t>(id(flow_switch_safety_delay_minutes).state * 60000.0f);

      if (pump_flow_missing_since_ms_ == 0)
      {
        pump_flow_missing_since_ms_ = now;
        ESP_LOGW("amber", "Safety check: Pump flow missing while compressor is running, waiting %.0f min before shutdown.", id(flow_switch_safety_delay_minutes).state);
      }

      if ((now - pump_flow_missing_since_ms_) >= flow_switch_delay_ms)
      {
        ESP_LOGW("amber", "Safety check: Pump flow missing for %.0f min while compressor is running, stopping compressor to avoid damage.", id(flow_switch_safety_delay_minutes).state);
        return true;
      }
    }
    else
    {
      pump_flow_missing_since_ms_ = 0;
    }

    // If temperature difference between Tuo and Tui is above 8 degrees while compressor is running, stop compressor to avoid damage
    if (fabsf(id(outlet_temperature_tuo).state - id(inlet_temperature_tui).state) > 8.0f && (compressor_controller_->IsRunning() || IsBackupHeaterActive()) && state_ != HeatCoolState::IDLE)
    {
      ESP_LOGW("amber", "Safety check: Temperature difference between Tuo and Tui is above 8 degrees while compressor is running, stopping compressor to avoid damage.");
      return true;
    }

    return false;
  }
  
  void SetPidController(climate::ClimateMode climate_mode)
  {
    auto heat_call = id(pid_heat_temperature_control).make_call();
    heat_call.set_mode(climate_mode);
    heat_call.perform();
    
    auto cool_call = id(pid_cool_temperature_control).make_call();
    cool_call.set_mode(climate_mode);
    cool_call.perform();
   }
  
public:
  HeatCoolController(PumpController* pump_controller, CompressorController* compressor_controller,
                     MixingValveActuator* mv_zone1, MixingValveActuator* mv_zone2)
    : BaseController(pump_controller, compressor_controller, HeatCoolState::WAIT_FOR_STATE_SWITCH, HeatCoolState::UNKNOWN, HeatCoolState::IDLE) {
      mixing_valve_zone1_ = mv_zone1;
      mixing_valve_zone2_ = mv_zone2;
      PublishState(StateToString(state_));
    }

  void StopAndSetIdleState()
  {
    compressor_controller_->Stop();
    TurnOffBackupHeater();
    StopPumps();
    CloseMixingValves();
    // Safety stop: leave heating/cooling at once, also during a defrost.
    SetWorkingMode(WORKING_MODE_STANDBY);
    SetNextState(HeatCoolState::IDLE);
  }

  /// @brief Takes over units that kept running while the ESP restarted, instead of stopping them.
  /// Only called with the 3-way valve in the heat/cool position and pump P0 running.
  void ResumeFromRunningUnits()
  {
    const bool compressor_running = compressor_controller_->IsRunning();
    const bool compressor_commanded = compressor_controller_->IsCommanded();
    requested_to_stop_ = false;
    start_current_temperature_ = 0.0f;
    pump_flow_missing_since_ms_ = 0;
    pump_controller_->AdoptRunningPump();
    const bool cooling = IsWorkingMode(WORKING_MODE_COOLING);

    if (compressor_running && compressor_commanded)
    {
      SetPidController(cooling ? climate::CLIMATE_MODE_COOL : climate::CLIMATE_MODE_HEAT);
      GetPidController().reset_integral_term();
      ResetMixingValvesIntegralTerm();
      if (cooling)
      {
        // Cooling runs with the enabled mixing valves fully open, normally set on PUMP_RUNNING entry which the resume
        // skips. Reapply it (no write when a valve is already open) so a closed valve does not block the cooled water.
        OpenMixingValves();
      }
      // The run time before the restart is unknown: keep the compressor on for at least the minimum on time.
      compressor_controller_->RecordStartTime();
      if (id(defrost_running_sensor).state)
      {
        // Actual defrost: P1 is off during a defrost, DEFROSTING starts it again when the defrost ends.
        StopPumpP1IfNeeded();
        SetNextState(HeatCoolState::DEFROSTING);
      }
      else
      {
        // No defrost or still in the pre-phase: COMPRESSOR_RUNNING keeps heating and handles the switch to
        // DEFROSTING (and stops P1) once the actual defrost starts. P1 may have stayed off when a defrost ended
        // during the restart, run it again on demand as IDLE does.
        if (id(heat_demand_active_sensor).state || id(cool_demand_active_sensor).state)
        {
          StartPumpP1IfNeeded();
        }
        SetNextState(HeatCoolState::COMPRESSOR_RUNNING);
      }
      return;
    }

    if (compressor_commanded)
    {
      // Level set but compressor not (yet) running: it was starting. Cancel the start, the normal flow
      // starts it again with the minimum off time and pump settle time.
      ESP_LOGI("amber", "Resume: compressor was starting, cancelling the start.");
      compressor_controller_->Stop();
      SetNextState(HeatCoolState::WAIT_COMPRESSOR_STOP);
      return;
    }

    compressor_controller_->RecordStopTime();
    if (compressor_running)
    {
      // Level 0 but still running: the compressor was stopping.
      SetNextState(HeatCoolState::WAIT_COMPRESSOR_STOP);
      return;
    }

    // Only the pump was running: continue the pump cycle.
    SetNextState(HeatCoolState::PUMP_RUNNING);
  }

  /// @brief Evaluates the safety checks without running the state machine (used while an OTA update holds it).
  bool HasSafetyStopCondition()
  {
    return PerformSafetyChecks();
  }

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
    int max_allowed = GetMaxAllowedCompressorMode();
    float current_temperature = GetControlTemperature();
    float target = GetPidController().target_temperature;

    // Hysteresis: hold the compressor mode while the control temperature stays close to the setpoint.
    // Inside the PID deadband the PID output still moves a little (kp_multiplier), which is enough to
    // flip the rounded compressor mode back and forth between two adjacent modes.
    const float hysteresis = id(compressor_hysteresis).state;
    const float dt = current_temperature - target;
    if (hysteresis > 0.0f && fabsf(dt) < hysteresis)
    {
      int clamped_mode = std::min((int)current_compressor_mode, max_allowed);
      ESP_LOGI("amber", "ΔT=%.2f°C within compressor hysteresis %.1f°C, compressor mode remains at %d", dt, hysteresis, clamped_mode);
      return clamped_mode;
    }

    // Guard: never modulate up when temperature has already overshot the setpoint
    const float TEMP_MARGIN = 0.3f;
    bool overshoot = IsWorkingMode(WORKING_MODE_COOLING)
      ? (current_temperature < (target - TEMP_MARGIN))
      : (current_temperature > (target + TEMP_MARGIN));
    if (overshoot && desired_compressor_mode > current_compressor_mode)
    {
      ESP_LOGW("amber", "Temperature has overshot target and PID controller wanted to move to a higher compressor mode.");
      return current_compressor_mode;
    }

    int capped_mode_index = std::min(desired_compressor_mode, max_allowed);

    ESP_LOGI("amber", "PID %.2f -> mode %d (previous mode %d)", GetPIDValue(), capped_mode_index, current_compressor_mode);
    return capped_mode_index;
  }

  bool ShouldStopCompressor()
  {
    if(!HasCompressorDemand())
    {
      // A stop request (switch to DHW, maintenance mode, controlled stop) lets a running oil return cycle or defrost
      // finish first, like the normal stop on temperature overshoot does.
      if (requested_to_stop_ && (id(oil_return_cycle_active).state || id(defrost_active_sensor).state))
      {
        ESP_LOGI("amber", "Stop requested: waiting for the oil return cycle or defrost to finish before stopping.");
        return false;
      }
      return true;
    }

    float target_temperature = GetPidController().target_temperature;
    float current_temperature = GetControlTemperature();

    // Overshoot is positive when temp has passed the setpoint in the working direction
    // Heating: too hot (Tc > target), Cooling: too cold (Tc < target)
    float stop_delta = IsWorkingMode(WORKING_MODE_COOLING) ? id(compressor_stop_delta_cooling).state : id(compressor_stop_delta_heating).state;
    float overshoot = IsWorkingMode(WORKING_MODE_COOLING) ? (target_temperature - current_temperature) : (current_temperature - target_temperature);

    if (overshoot >= stop_delta)
    {
      if (id(oil_return_cycle_active_settled).state)
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
    CloseMixingValves();
    SetNextState(HeatCoolState::WAIT_COMPRESSOR_STOP);
  }

  void OnStateEnter(HeatCoolState new_state) override
  {
    switch (new_state)
    {
      case HeatCoolState::PUMP_RUNNING:
      {
        bool frost_protection = id(frost_protection_stage1_active).state || id(frost_protection_stage2_active).state;
        bool frost_protection_without_heat_demand = frost_protection && !id(heat_demand_active_sensor_zone_1).state && !id(heat_demand_active_sensor_zone_2).state;
        if (IsCoolingDemand() || frost_protection_without_heat_demand)
        {
          OpenMixingValves();
        }
        break;
      }

      case HeatCoolState::COMPRESSOR_RUNNING:
        backup_degmin_last_ms_ = App.get_loop_component_start_time();
        break;

      case HeatCoolState::WAIT_BACKUP_HEATER_RUNNING:
        ResetAccumulatedDegMin();
        break;

      default:
        break;
    }
  }

  void UpdateStateMachine()
  {
    uint32_t now = App.get_loop_component_start_time();
    if(PerformSafetyChecks())
    {
      StopAndSetIdleState();
      return;
    }

    switch (state_)
    {
      case HeatCoolState::UNKNOWN:
        break;

      case HeatCoolState::IDLE:
      {
        requested_to_stop_ = false;

        if(!HasPumpDemand())
        {
          StopPumpP1IfNeeded();
          break;
        }

        // P1 runs continuously on heat/cool demand; frost protection alone only runs P1 together with the P0 interval cycles.
        if (id(heat_demand_active_sensor).state || id(cool_demand_active_sensor).state)
        {
          StartPumpP1IfNeeded();
        }
        else
        {
          StopPumpP1IfNeeded();
        }

        // Start pump on interval, without the pump running we cannot determine if there is actual compressor demand as Tc can change.
        if (pump_controller_->ShouldStartNextPumpCycle())
        {
          pump_controller_->Start();
          StartPumpP1IfNeeded();
          SetNextState(HeatCoolState::WAIT_PUMP_RUNNING);
        }
        break;
      }

      case HeatCoolState::WAIT_PUMP_RUNNING:
      {
        if (pump_controller_->IsRunning())
        {
          pump_controller_->ClearStartWaitTimer();
          SetNextState(HeatCoolState::PUMP_RUNNING);
        }
        else 
        {
          const uint32_t timeout_ms = PUMP_START_TIMEOUT_S * 1000UL;
          if ((now - pump_controller_->GetStartWaitStartedTime()) >= timeout_ms)
          {
            ESP_LOGE("amber", "Heat/cool pump start timeout reached after %lu seconds, stopping system.", (unsigned long) PUMP_START_TIMEOUT_S);
            id(error_pump_start_timeout).publish_state(true);
            StopPumps();
            SetNextState(HeatCoolState::WAIT_PUMP_STOP);
          }
        }
        break;
      }

      case HeatCoolState::PUMP_RUNNING:
      {
        pump_controller_->ApplySpeedChangeIfNeeded(false);

        // Stop if there is no compressor demand or the temperature does not ask for the compressor and pump interval is finished.
        const bool compressor_wanted = HasCompressorDemand() && IsStartTemperatureReached();
        if (!compressor_wanted && pump_controller_->IsIntervalCycleFinished())
        {
          ESP_LOGI("amber", "Stopping pump (interval cycle finished)");
          // P1 keeps running between pump cycles while there is demand, idle stops it when the demand is gone.
          pump_controller_->Stop();
          SetNextState(HeatCoolState::WAIT_PUMP_STOP);
          break;
        }

        if (!compressor_wanted)
        {
          break;
        }

        // The compressor is wanted: keep the pump running while the minimum off time passes. Ending the pump
        // cycle here would delay the start by a full pump interval.
        if (!compressor_controller_->HasPassedMinOffTime())
        {
          ESP_LOGI("amber", "Not starting compressor because minimum compressor time off is not reached.");
          break;
        }

        // Settle temperature before starting compressor.
        if (!pump_controller_->IsPumpSettled())
        {
          ESP_LOGI("amber", "Not starting compressor because temperature needs to stabilize (pump on time too short)");
          break;
        }

        SetWorkingMode(IsCoolingDemand() ? WORKING_MODE_COOLING : WORKING_MODE_HEATING);
        SetPidController(IsCoolingDemand() ? climate::CLIMATE_MODE_COOL : climate::CLIMATE_MODE_HEAT);

        if(id(emergency_mode_enabled).state)
        {
          ESP_LOGW("amber", "Emergency mode enabled, not starting compressor but switching on backup heater");
          TurnOnBackupHeater();
          SetNextState(HeatCoolState::WAIT_BACKUP_HEATER_RUNNING);
        }
        else
        {
          compressor_controller_->ApplyCompressorMode(DetermineSoftStartMode());
          compressor_controller_->ArmStartWaitTimer();
          SetNextState(HeatCoolState::WAIT_COMPRESSOR_RUNNING);
        }
        break;
      }

      case HeatCoolState::WAIT_COMPRESSOR_RUNNING:
      {
        if (compressor_controller_->IsRunning())
        {
          GetPidController().reset_integral_term();
          ResetMixingValvesIntegralTerm();
          compressor_controller_->RecordStartTime();
          SetNextState(HeatCoolState::COMPRESSOR_SOFTSTART);
        }
        else 
        {
          const uint32_t timeout_ms = COMPRESSOR_START_TIMEOUT_S * 1000UL;
          if ((now - compressor_controller_->GetStartWaitStartedTime()) >= timeout_ms)
          {
            ESP_LOGE("amber", "Heat/cool compressor start timeout reached after %lu seconds, stopping system.", (unsigned long) COMPRESSOR_START_TIMEOUT_S);
            id(error_compressor_start_timeout).publish_state(true);
            compressor_controller_->Stop();
            SetNextState(HeatCoolState::WAIT_COMPRESSOR_STOP);
          }
        }
        break;
      }

      case HeatCoolState::COMPRESSOR_SOFTSTART:
      {
        pump_controller_->ApplySpeedChangeIfNeeded(false);
        
        if(IsWorkingMode(WORKING_MODE_HEATING))
        {
          ApplyMixingValvePositions();
        }

        // A controlled stop does not wait for the soft start or the minimum on time.
        if (IsControlledStopRequested() && ShouldStopCompressor())
        {
          ESP_LOGI("amber", "Controlled stop: stopping compressor during soft start.");
          Stop();
          break;
        }

        if (compressor_controller_->HasPassedSoftStartDuration())
        {
          SetNextState(HeatCoolState::COMPRESSOR_RUNNING);
        }
        break;
      }

      case HeatCoolState::COMPRESSOR_RUNNING:
      {
        pump_controller_->ApplySpeedChangeIfNeeded(true);

        if(IsWorkingMode(WORKING_MODE_HEATING))
        {
          ApplyMixingValvePositions();
        }

        if(start_current_temperature_ == 0.0f)
        {
          start_current_temperature_ = GetControlTemperature();
        }

        // Keep heating during the outdoor unit's pre-phase, react once the actual defrost starts.
        if (id(defrost_running_sensor).state)
        {
          StopPumpP1IfNeeded();
          SetNextState(HeatCoolState::DEFROSTING);
          break;
        }

        if (compressor_controller_->HasPassedMinOnTime() || IsControlledStopRequested())
        {
          if(ShouldStopCompressor())
          {
            ESP_LOGI("amber", "Stopping compressor because there is no demand or a temperature overshoot.");
            Stop();
            break;
          }
        }
        else 
        {
          ESP_LOGI("amber", "Minimum compressor on time not reached, not checking for potential stop conditions yet.");
        }

        // Backup heater logic only applies to heating mode, and not while a stop request waits (e.g. for oil return).
        if (!IsWorkingMode(WORKING_MODE_COOLING) && !requested_to_stop_)
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
            pump_controller_->RestartPumpRunCycle();
          }
          SetNextState(HeatCoolState::PUMP_RUNNING);
        }
        break;
      }

      case HeatCoolState::WAIT_PUMP_STOP:
      {
        if (!pump_controller_->IsRunning())
        {
          // Leaving heating aborts a running defrost without its recovery phase, so wait for 2118 to drop.
          if (id(defrost_active_sensor).state)
          {
            ESP_LOGI("amber", "Defrost active, waiting before switching to standby.");
            StopPumpP1IfNeeded();
            break;
          }
          pump_controller_->ClearStopWaitTimer();
          SetWorkingMode(WORKING_MODE_STANDBY);
          SetPidController(climate::CLIMATE_MODE_OFF);
          CloseMixingValves();
          SetNextState(HeatCoolState::IDLE);
        }
        else 
        {
          const uint32_t timeout_ms = PUMP_STOP_TIMEOUT_S * 1000UL;
          if ((now - pump_controller_->GetStopWaitStartedTime()) >= timeout_ms)
          {
            ESP_LOGE("amber", "Heat/cool pump stop timeout reached after %lu seconds, forcing idle state.", (unsigned long) PUMP_STOP_TIMEOUT_S);
            id(error_pump_stop_timeout).publish_state(true);
            StopPumps();
            SetWorkingMode(WORKING_MODE_STANDBY);
            SetPidController(climate::CLIMATE_MODE_OFF);
            CloseMixingValves();
            SetNextState(HeatCoolState::IDLE);
          }
        }
        break;
      }

      case HeatCoolState::WAIT_BACKUP_HEATER_RUNNING:
      {
        if (IsBackupHeaterActive())
        {
          last_temperature_for_rate_ = GetControlTemperature();
          temperature_rate_c_per_min_ = 0.0f;
          backup_degmin_last_ms_ = backup_heater_start_time_ms_;
          SetNextState(HeatCoolState::BACKUP_HEATER_RUNNING);
        }
        break;
      }

      case HeatCoolState::BACKUP_HEATER_RUNNING:
      {
        ApplyMixingValvePositions();
        float current_temperature = GetControlTemperature();
        float target_temperature = id(pid_heat_temperature_control).target_temperature;

        if (current_temperature >= target_temperature + id(compressor_stop_delta_heating).state)
        {
          ESP_LOGI("amber", "Disabling backup heater because measured temperature %.2f°C exceeded stop temperature delta %.2f°C.", current_temperature, target_temperature + id(compressor_stop_delta_heating).state);
          TurnOffBackupHeater();
          LeaveStateAndSetNextStateAfterWaitTime(id(emergency_mode_enabled).state ? HeatCoolState::PUMP_RUNNING : HeatCoolState::COMPRESSOR_RUNNING, BACKUP_HEATER_OFF_SETTLE_TIME_S * 1000UL);
          break;
        }

        if(!id(emergency_mode_enabled).state)
        {
          uint32_t now = App.get_loop_component_start_time();
          float predicted_temperature = CalculateBackupHeaterPredictedTemperature();
          // Disable backup heater if predicted Temperature is above target.
          uint32_t settle_elapsed_ms = now - backup_heater_start_time_ms_;
          uint32_t settle_time_ms = BACKUP_HEATER_PREDICTION_SETTLE_TIME_S * 1000UL;
          // Only check predicition after a minimum settle time to avoid disabling backup heater too early due to initial temperature fluctuations when turning it on.
          if (predicted_temperature >= target_temperature + 2.0f && settle_elapsed_ms >= settle_time_ms)
          { // Margin to buffer some heat to reduce undershoot.
            ESP_LOGI("amber", "Disabling backup heater (predicted Temperature %.2f°C above target %.2f°C)", predicted_temperature, target_temperature);
            TurnOffBackupHeater();
            LeaveStateAndSetNextStateAfterWaitTime(HeatCoolState::COMPRESSOR_RUNNING, BACKUP_HEATER_OFF_SETTLE_TIME_S * 1000UL);
            break;
          }
        }

        if(ShouldStopCompressor())
        {
          ESP_LOGI("amber", "Stopping compressor and backup heater because there is no demand or a temperature overshoot.");
          compressor_controller_->Stop();
          TurnOffBackupHeater();
          SetNextState(id(emergency_mode_enabled).state ? HeatCoolState::PUMP_RUNNING : HeatCoolState::WAIT_COMPRESSOR_STOP);
          break;
        }
        break;
      }

      case HeatCoolState::DEFROSTING:
      {
        if (id(defrost_active_sensor).state)
        {          
          pump_controller_->ApplySpeedChangeIfNeeded(false);
          ESP_LOGI("amber", "Defrost busy, waiting before making changes to compressor.");
          break;
        }

        pump_controller_->ResetHeatingPidState();
        GetPidController().reset_integral_term();
        ResetMixingValvesIntegralTerm();
        StartPumpP1IfNeeded();

        if (id(temperature_outside_ta_avg_1h).state <= id(defrost_backup_heater_boost_temperature_sensor).state && !requested_to_stop_)
        {
          ESP_LOGI("amber", "Defrost ended, enabling backup heater as configured for outside temperature %.2f°C to boost temperature back to setpoint.", id(temperature_outside_ta_avg_1h).state);
          TurnOnBackupHeater();
          SetNextState(HeatCoolState::WAIT_BACKUP_HEATER_RUNNING);
        }
        else
        {
          float current_temperature = GetControlTemperature();
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
        ProcessDeferredStateChange();
        break;
      }
    }
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
    return IsWorkingMode(WORKING_MODE_COOLING) ? id(pid_cool_temperature_control) : id(pid_heat_temperature_control);
  }
};
