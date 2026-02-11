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
#include "pump_controller.h"
#include "compressor_controller.h"
#include "backup_heater_controller.h"
#include "dhw_controller.h"

using namespace esphome;


/**
 * Heat Pump Controller - Core logic for heat pump operation
 */
class HeatPumpController {
private:
  HPState machine_state_;
  HPState deferred_machine_state_;
  uint32_t defer_state_change_until_ms_;
  bool is_switching_modes_;

  ThreeWayValveController* valve_controller_;
  PumpController* pump_controller_;
  CompressorController* compressor_controller_;
  BackupHeaterController* backup_heater_controller_;
  DHWController* dhw_controller_;
  IModeController* active_mode_controller_;
  IModeController* heat_cool_controller_;

  void SetNextState(HPState new_state);
  void LeaveStateAndSetNextStateAfterWaitTime(HPState new_state, uint32_t defer_ms);
  void DoSafetyChecks();
  void UpdateStateMachine();
  ThreeWayValvePosition GetDesiredThreeWayValvePosition();

public:
  HeatPumpController();
  ~HeatPumpController();

  void Loop();
  void SetPIDValue(float value);
  void ResetPumpInterval();
};

HeatPumpController::HeatPumpController()
{
  valve_controller_ = new ThreeWayValveController();
  pump_controller_ = new PumpController(*valve_controller_);
  compressor_controller_ = new CompressorController(*valve_controller_);
  backup_heater_controller_ = new BackupHeaterController(*valve_controller_);
  dhw_controller_ = new DHWController(*valve_controller_);
  heat_cool_controller_ = new HeatCoolController(*valve_controller_);
  active_mode_controller_ = heat_cool_controller_;
  deferred_machine_state_ = HPState::UNKNOWN;
  defer_state_change_until_ms_ = 0;
  is_switching_modes_ = false;
}

HeatPumpController::~HeatPumpController()
{
  delete dhw_controller_;
  delete backup_heater_controller_;
  delete compressor_controller_;
  delete pump_controller_;
  delete valve_controller_;
}

void HeatPumpController::SetNextState(HPState new_state)
{
  machine_state_ = new_state;
  const char* txt = HPStateToString(new_state);
  
  id(state_machine_state).publish_state(txt);
  StateEntered(new_state);
  ESP_LOGI("amber", "HP state changed: %s", txt);
}

void HeatPumpController::LeaveStateAndSetNextStateAfterWaitTime(HPState new_state, uint32_t defer_ms)
{
  deferred_machine_state_ = new_state;
  defer_state_change_until_ms_ = millis() + defer_ms;
  SetNextState(HPState::WAIT_FOR_STATE_SWITCH);
}

void HeatPumpController::DoSafetyChecks()
{
  // If pump is not active while compressor is running, stop compressor to avoid damage
  if (compressor_controller_->IsRunning() && !pump_controller_->IsRunning())
  {
    ESP_LOGW("amber", "Safety check: Pump is not active while compressor is running, stopping compressor to avoid damage.");
    uint32_t pump_interval_ms = (uint32_t)id(pump_interval).state * 60000UL;
    compressor_controller_->Stop(pump_interval_ms);
    SetNextState(HPState::IDLE);
    return;
  }

  // If Tuo - Tui is above 8 degrees while compressor is running, stop compressor to avoid damage
  if (id(outlet_temperature_tuo).state - id(inlet_temperature_tui).state > 8.0f && compressor_controller_->IsRunning())
  {
    ESP_LOGW("amber", "Safety check: Temperature difference between Tuo and Tui is above 8 degrees while compressor is running, stopping compressor to avoid damage.");
    uint32_t pump_interval_ms = (uint32_t)id(pump_interval).state * 60000UL;
    compressor_controller_->Stop(pump_interval_ms);
    pump_controller_->Stop();
    SetNextState(HPState::IDLE);
    return;
  }
}

void HeatPumpController::StateEntered(HPState state)
{
  switch (state)
  {
    case HPState::WAIT_COMPRESSOR_STOP:
      pump_controller_->RestartPumpInterval();
      break;
    case HPState::COMPRESSOR_RUNNING:
      compressor_controller_->RecordStartTime();
      active_mode_controller_->StartTrackingTemperatureChanges();

      // Maybe move to heat_cool_controller if we have a compressor started method or something.
      id(pid_heat_cool_temperature_control).reset_integral_term();
      break;
}

void HeatPumpController::UpdateStateMachine()
{
  const uint32_t now = millis();

  bool dhw_demand = dhw_controller_->HasDemand();
  bool frost_protection_stage2 = id(frost_protection_stage2_active).state;
  bool frost_protection_stage1 = id(frost_protection_stage1_active).state;
  bool heating_or_cooling_demand = id(heat_demand_active_sensor).state || id(cool_demand_active_sensor).state;
  bool pump_demand = dhw_demand || heating_or_cooling_demand || frost_protection_stage1 || frost_protection_stage2;

  switch (machine_state_)
  {
  case HPState::INITIALIZE:
  {
      // Initialize relays
      id(initialize_relay_switch).turn_on();
      id(pump_p0_relay_switch).turn_on();

      // Restore 3-way valve state based on relay position
      valve_controller_->RestoreStateFromRelay();

      // Restore state whenever initialized while compressor is running
      if (compressor_controller_->IsRunning())
      {
        SetNextState(HPState::COMPRESSOR_RUNNING);
      }
      else if (pump_controller_->IsRunning())
      {
        SetNextState(HPState::PUMP_INTERVAL_RUNNING);
      }
      else if (backup_heater_controller_->IsActive())
      {
        SetNextState(HPState::BACKUP_HEATER_RUNNING);
      }
      else
      {
        // Initialize pump to Off
        pump_controller_->Stop();
        SetNextState(HPState::IDLE);
      }
      break;
  }
  case HPState::WAIT_FOR_STATE_SWITCH:
  {
    if (defer_state_change_until_ms_ > now)
    {
      ESP_LOGD("amber", "Waiting for state switch, transitioning to next state in %lu ms", defer_state_change_until_ms_ - now);
    }
    else
    {
      defer_state_change_until_ms_ = 0;
      SetNextState(deferred_machine_state_);
      deferred_machine_state_ = HPState::UNKNOWN;
    }
    break;
  }
  case HPState::IDLE:
  {
    ThreeWayValvePosition desired_position = GetDesiredThreeWayValvePosition();

    if (!valve_controller_->IsInPosition(desired_position))
    {
      valve_controller_->SetPosition(desired_position);
      is_switching_modes_ = true;
      LeaveStateAndSetNextStateAfterWaitTime(HPState::IDLE, THREE_WAY_VALVE_SWITCH_TIME_S * 1000UL);
      break;
    }

    // If interval is elapsed or there is compressor demand, start pump immediately
    if (pump_controller_->ShouldStartNextPumpCycle() || active_mode_controller_->HasDemand())
    {
      pump_controller_->Start(active_mode_controller_->GetPreferredPumpSpeed());
      SetNextState(HPState::WAIT_PUMP_RUNNING);
    }
    break;
  }
  case HPState::WAIT_PUMP_RUNNING:
  {
    if (pump_controller_->IsRunning())
    {
      SetNextState(HPState::PUMP_INTERVAL_RUNNING);
    }
    // TODO: Implement timeout for when pump does not start.
    break;
  }
  case HPState::PUMP_INTERVAL_RUNNING:
  {
    pump_controller_->ApplySpeedChangeIfNeeded(active_mode_controller_->GetPreferredPumpSpeed());

    // Stop if there is no demand and pump interval is finished.
    if (!active_mode_controller_->HasDemand() && pump_controller_->IsIntervalCycleFinished())
    {
      ESP_LOGI("amber", "Stopping pump (interval cycle finished)");
      pump_controller_->Stop();
      SetNextState(HPState::WAIT_PUMP_STOP);
      break;
    }

    // Settle temperature before starting compressor.
    if (!pump_controller_->IsPumpSettled() && active_mode_controller_->ShouldWaitForTemperatureStabilizationBeforeCompressorStart())
    {
      ESP_LOGI("amber", "Not starting compressor because temperature needs to stabilize (pump on time too short)");
      break;
    }

    // TODO: Not after switching
    if (!compressor_controller_->HasPassedMinOffTime())
    {
      ESP_LOGI("amber", "Not starting compressor because minimum compressor time off is not reached.");
      return false;
    }

    ApplyWorkingMode();
    compressor_controller_->ApplyCompressorMode(active_mode_controller_->DetermineCompressorMode());
    is_switching_modes_ = false;
    SetNextState(HPState::WAIT_COMPRESSOR_RUNNING);
    break;
  }
  case HPState::WAIT_PUMP_STOP:
  {
    if (!pump_controller_->IsRunning())
    {
      SetNextState(HPState::IDLE);
    }
    else 
    {
      // TODO: Implement timeout for when pump does not stop.
    }
    break;
  }
  case HPState::WAIT_COMPRESSOR_RUNNING:
  {
    if (compressor_controller_->IsRunning())
    {
      SetNextState(active_mode_controller_->ShouldSoftStart() ? HPState::HEAT_COOL_COMPRESSOR_SOFTSTART : HPState::COMPRESSOR_RUNNING);
    }
    else 
    {
      // TODO: Implement timeout for when compressor does not start.
    }
    break;
  }
  case HPState::HEAT_COOL_COMPRESSOR_SOFTSTART:
  {
    if (compressor_controller_->HasPassedSoftStartDuration())
    {
      SetNextState(HPState::COMPRESSOR_RUNNING);
    }
    break;
  }
  case HPState::COMPRESSOR_RUNNING:
  {
    pump_controller_->ApplySpeedChangeIfNeeded(active_mode_controller_->GetPreferredPumpSpeed());

    if (id(defrost_active_sensor).state)
    {
      SetNextState(HPState::DEFROSTING);
      break;
    }

    dhw_controller_->HandlePumpStart();

    // Only check for stop conditions when min on time is passed
    if (compressor_controller_->HasPassedMinOnTime())
    {
      if(active_mode_controller_->ShouldStopCompressor())
      {
        ESP_LOGI("amber", "Stopping compressor because controller has determined it should stop.");
        SetNextState(HPState::WAIT_COMPRESSOR_STOP);
        break;
      }

      // Check if we need to stop compressor because there is demand for the other mode that has priority (DHW > heating/cooling).
      if (active_mode_controller != dhw_controller && dhw_controller->HasDemand())
      {
        ESP_LOGI("amber", "Stopping compressor because there is demand for DHW.");
        compressor_controller_->Stop();
        SetNextState(HPState::WAIT_COMPRESSOR_STOP);
        break;
      }
    }

    if (id(sg_ready_max_boost_mode_active_sensor).state)
    {
      ESP_LOGI("amber", "Enabling backup heater (SG Ready max boost active)");
      backup_heater_controller_->TurnOn();
      SetNextState(HPState::WAIT_BACKUP_HEATER_RUNNING);
      break;
    }

    backup_heater_controller_->CheckActivation(compressor_controller_->GetLastStartTime(), compressor_controller_->GetStartTargetTemp());
    
    if(active_mode_controller_->ShouldActivateBackupHeater())
    {
      ESP_LOGI("amber", "Enabling backup heater (active mode controller requested activation)");
      TurnOnBackupHeater();
      SetNextState(HPState::WAIT_BACKUP_HEATER_RUNNING);
      break;
    }

    compressor_controller_->ApplyCompressorMode(active_mode_controller_->DetermineCompressorMode());
  }
  case HPState::WAIT_COMPRESSOR_STOP:
  {
    if (!compressor_controller_->IsRunning())
    {
      // If we stopped the compressor without extending the pump cycle, immediately stop before the logic to start compressor kicks in again
      if(pump_controller_->IsIntervalCycleFinished())
      {
        pump_controller_->Stop();
        SetNextState(HPState::WAIT_PUMP_STOP);
        break;
      }
      
      SetNextState(HPState::PUMP_INTERVAL_RUNNING);
    }
    break;
  }
  case HPState::WAIT_BACKUP_HEATER_RUNNING:
  {
    if (backup_heater_controller_->IsActive())
    {
      SetNextState(HPState::BACKUP_HEATER_RUNNING);
    }
    break;
  }
  case HPState::BACKUP_HEATER_RUNNING:
  {
    if (backup_heater_controller_->ShouldDeactivateBasedOnPrediction())
    {
      LeaveStateAndSetNextStateAfterWaitTime(HPState::COMPRESSOR_RUNNING, BACKUP_HEATER_OFF_SETTLE_TIME_S * 1000UL);
      return;
    }
    break;
  }
  case HPState::DEFROSTING:
  {
    if (id(defrost_active_sensor).state)
    {
      ESP_LOGI("amber", "Defrost busy, waiting before making changes to compressor.");
      break;
    }

    // Defrost is no longer active

    // No need to boost in DHW mode
    if (valve_controller_->IsInDhwMode())
    {
      LeaveStateAndSetNextStateAfterWaitTime(HPState::COMPRESSOR_RUNNING, COMPRESSOR_SETTLE_TIME_AFTER_DEFROST_S * 1000UL);
      break;
    }

    if (id(temperature_outside_ta).state <= id(defrost_backup_heater_boost_temperature_sensor).state)
    {
      ESP_LOGI("amber", "Defrost ended, enabling backup heater as configured for outside temperature %.2f°C to boost temperature back to setpoint.", 
           id(temperature_outside_ta).state);
      backup_heater_controller_->TurnOn();
      SetNextState(HPState::WAIT_BACKUP_HEATER_RUNNING);
    }
    else
    {
      compressor_controller_->ApplyDefrostRecoveryMode();
      LeaveStateAndSetNextStateAfterWaitTime(HPState::COMPRESSOR_RUNNING, COMPRESSOR_SETTLE_TIME_AFTER_DEFROST_S * 1000UL);
    }
    id(pid_heat_cool_temperature_control).reset_integral_term();
    break;
  }
  }
}

void HeatPumpController::Loop()
{
  if(!id(modbus_inside_online).state || !id(modbus_outside_online).state)
  {
    return;
  }

  DoSafetyChecks();
  dhw_controller_->CheckLegionellaCycle();
  UpdateStateMachine();
}

void HeatPumpController::SetPIDValue(float pid_value)
{
  compressor_controller_->SetPIDValue(pid_value);
}

void HeatPumpController::ResetPumpInterval()
{
  pump_controller_->ResetInterval();
}

/// @brief Determines the desired position of the 3-way valve based on active demands. DHW has priority over heating/cooling.
/// @return 
ThreeWayValvePosition HeatPumpController::GetDesiredThreeWayValvePosition()
{
  // DHW has priority
  if (id(dhw_demand_active_sensor).state)
  {
    return ThreeWayValvePosition::DHW;
  }
  
  // Otherwise use heating/cooling position
  return ThreeWayValvePosition::HEATING_COOLING;
}

void HeatPumpController::ApplyWorkingMode()
{
  int workingMode = valve_controller_->IsInDhwMode() ? WORKING_MODE_HEATING 
         : id(heat_demand_active_sensor).state ? WORKING_MODE_HEATING
         : WORKING_MODE_COOLING;

  if(id(working_mode_switch).active_index().value() == workingMode)
  {
    return;
  }

  auto working_mode_call = id(working_mode_switch).make_call();
  working_mode_call.set_index(workingMode);
  working_mode_call.perform();
}

bool HeatPumpController::IsBackupHeaterActive()
{
  return id(backup_heater_active_sensor).state;
}

void HeatPumpController::TurnOnBackupHeater()
{
  id(backup_heater_relay).turn_on();
}

void HeatPumpController::TurnOffBackupHeater()
{
  id(backup_heater_relay).turn_off();
}