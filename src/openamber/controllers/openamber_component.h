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
 * This program is distributed it in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "constants.h"
#include "dhw_controller.h"
#include "heat_cool_controller.h"

using namespace esphome;

namespace esphome {
namespace openamber {

OpenAmberComponent::OpenAmberComponent()
{
  pump_controller_ = new PumpController();
  compressor_controller_ = new CompressorController();
  dhw_controller_ = new DHWController(pump_controller_, compressor_controller_);
  heat_cool_controller_ = new HeatCoolController(pump_controller_, compressor_controller_);
}

OpenAmberComponent::~OpenAmberComponent()
{
  delete dhw_controller_;
  delete heat_cool_controller_;
  delete pump_controller_;
  delete compressor_controller_;
}

void OpenAmberComponent::setup()
{
  ESP_LOGI("amber", "OpenAmberController initialized");
}

void OpenAmberComponent::loop()
{
  if(!initialized)
  {
    id(initialize_relay_switch).turn_on();
    id(pump_p0_relay_switch).turn_on();
    initialized = true;
    ESP_LOGI("amber", "Initialized heat pump controller");
    return;
  }

  ThreeWayValvePosition current_valve_position = GetThreeWayValvePosition();
  
  if(current_valve_position == ThreeWayValvePosition::DHW)
  {
    dhw_controller_->UpdateStateMachine();
  }
  else
  {
    heat_cool_controller_->UpdateStateMachine();
  }
}

void OpenAmberComponent::write_heat_pid_value(float value)
{
  heat_cool_controller_->SetPIDValue(value);
}

void OpenAmberComponent::reset_pump_interval()
{
  pump_controller_->ResetInterval();
}

// Privates
void OpenAmberComponent::SetThreeWayValve(ThreeWayValvePosition position)
{
  if(position == ThreeWayValvePosition::DHW)
  {
    id(three_way_valve_dhw_switch).turn_on();
    id(three_way_valve_heat_cool_switch).turn_off();
  }
  else
  {
    id(three_way_valve_dhw_switch).turn_off();
    id(three_way_valve_heat_cool_switch).turn_on();
  }
  ESP_LOGI("amber", "Setting 3-way valve to %s.", position == ThreeWayValvePosition::DHW ? "DHW" : "Heating/Cooling");
}

ThreeWayValvePosition OpenAmberComponent::GetThreeWayValvePosition()
{
  if (id(three_way_valve_dhw_switch).state)
  {
    return ThreeWayValvePosition::DHW;
  }
  else
  {
    return ThreeWayValvePosition::HEATING_COOLING;
  }
}

void OpenAmberComponent::ApplyWorkingMode()
{
  int workingMode = GetThreeWayValvePosition() == ThreeWayValvePosition::DHW ? WORKING_MODE_HEATING 
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

/// @brief Determines the desired position of the 3-way valve based on active demands. DHW has priority over heating/cooling.
/// @return 
ThreeWayValvePosition OpenAmberComponent::GetDesiredThreeWayValvePosition()
{
  // DHW has priority
  if (id(dhw_demand_active_sensor).state)
  {
    return ThreeWayValvePosition::DHW;
  }
  
  // Otherwise use heating/cooling position
  return ThreeWayValvePosition::HEATING_COOLING;
}

}  // namespace openamber
}  // namespace esphome
