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
#include "three_way_valve_controller.h"

using namespace esphome;


// ============================================================================
// HEAT PUMP STATE TO STRING CONVERSION
// ============================================================================

inline const char* HPStateToString(HPState state)
{
  switch (state)
  {
  case HPState::IDLE:
    return "Idle";
  case HPState::PUMP_INTERVAL_RUNNING:
    return "Pump interval running";
  case HPState::WAIT_PUMP_RUNNING:
    return "Wait for pump running";
  case HPState::WAIT_COMPRESSOR_RUNNING:
    return "Wait for compressor running";
  case HPState::HEAT_COOL_COMPRESSOR_SOFTSTART:
    return "Compressor soft start";
  case HPState::COMPRESSOR_RUNNING:
    return "Compressor running";
  case HPState::WAIT_COMPRESSOR_STOP:
    return "Waiting for compressor to stop";
  case HPState::WAIT_BACKUP_HEATER_RUNNING:
    return "Wait for backup heater running";
  case HPState::BACKUP_HEATER_RUNNING:
    return "Backup heater running";
  case HPState::DEFROSTING:
    return "Defrosting";
  case HPState::WAIT_FOR_STATE_SWITCH:
    return "Waiting for wait time to elapse";
  case HPState::WAIT_PUMP_STOP:
    return "Waiting for pump to stop";
  default:
    return "Unknown";
  }
}


// ============================================================================
// TEMPERATURE UTILITY FUNCTIONS
// ============================================================================

inline float GetTargetTemperature(const ThreeWayValveController& valve_controller)
{
  if (valve_controller.IsInDhwMode())
  {
    return id(current_dhw_setpoint_sensor).state;
  }
  else
  {
    return id(pid_heat_cool_temperature_control).target_temperature;
  }
}

inline float GetCurrentTemperature(const ThreeWayValveController& valve_controller)
{
  return valve_controller.IsInDhwMode() 
    ? id(dhw_temperature_tw_sensor).state 
    : id(heat_cool_temperature_tc).state;
}

inline float GetTemperatureDelta(const ThreeWayValveController& valve_controller)
{
  return GetCurrentTemperature(valve_controller) - GetTargetTemperature(valve_controller);
}
