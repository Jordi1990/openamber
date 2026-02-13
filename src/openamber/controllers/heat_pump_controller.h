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
#include "dhw_controller.h"
#include "heat_cool_controller.h"

using namespace esphome;


/**
 * Heat Pump Controller - Core logic for heat pump operation
 */
class HeatPumpController {
private:

void HeatPumpController::RefreshActiveModeController()
{
  if (valve_controller_->IsInDhwMode())
  {
    active_mode_controller_ = static_cast<ModeController*>(dhw_controller_);
    return;
  }

  active_mode_controller_ = static_cast<ModeController*>(heat_cool_controller_);
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
      // Restore state whenever initialized while compressor is running
      if (compressor_controller_->IsRunning())
      {
        SetNextState(HPState::COMPRESSOR_RUNNING);
      }
      else if (pump_controller_->IsRunning())
      {
        SetNextState(HPState::PUMP_INTERVAL_RUNNING);
      }
      else if (IsBackupHeaterActive())
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
    break;
  }
  case HPState::COMPRESSOR_RUNNING:
  {
    // Only check for stop conditions when min on time is passed
    if (compressor_controller_->HasPassedMinOnTime())
    {
      // Check if we need to stop compressor because there is demand for the other mode that has priority (DHW > heating/cooling).
      if (active_mode_controller_ != dhw_controller_ && dhw_controller_->HasDemand())
      {
        ESP_LOGI("amber", "Stopping compressor because there is demand for DHW.");
        compressor_controller_->Stop();
        SetNextState(HPState::WAIT_COMPRESSOR_STOP);
        break;
      }
    }
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
}