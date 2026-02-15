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
  // Always set PID controller to HEAT mode.
  auto call = id(pid_heat_cool_temperature_control).make_call();
  call.set_mode("HEAT");
  call.perform();
  ESP_LOGI("amber", "OpenAmberController initialized");
}

void OpenAmberComponent::loop()
{

}

void OpenAmberComponent::update()
{
  if(!id(modbus_inside_online).state || !id(modbus_outside_online).state)
  {
    return;
  }

  ThreeWayValvePosition current_valve_position = GetThreeWayValvePosition();
  ThreeWayValvePosition desired_valve_position = GetDesiredThreeWayValvePosition();

  switch(state_)
  {
    case State::INITIALIZING:
    {
      id(initialize_relay_switch).turn_on();
      id(pump_p0_relay_switch).turn_on();
      ESP_LOGI("amber", "Initialized heat pump controller");
      SetNextState(current_valve_position == ThreeWayValvePosition::DHW ? State::DHW_HEAT : State::HEAT_COOL);
      break;
    }

    case State::HEAT_COOL:
    {
      if(desired_valve_position == ThreeWayValvePosition::DHW && !heat_cool_controller_->IsRequestedToStop())
      {
        heat_cool_controller_->RequestToStop();
      }

      if(heat_cool_controller_->IsInIdleState() && desired_valve_position != current_valve_position)
      {
        SetThreeWayValve(desired_valve_position);
        SetNextState(State::SWITCHING);
        break;
      }

      heat_cool_controller_->UpdateStateMachine();
      break;
    }

    case State::DHW_HEAT:
    {
      // TODO: Potentially stop DHW when priority heating is required.

      if(dhw_controller_->IsInIdleState() && desired_valve_position != current_valve_position)
      {
        SetThreeWayValve(desired_valve_position);
        SetNextState(State::SWITCHING);
        break;
      }

      dhw_controller_->UpdateStateMachine();
      break;
    }

    case State::SWITCHING:
    {
      if(last_three_way_valve_switch_ms_ + THREE_WAY_VALVE_SWITCH_TIME_S * 1000UL < millis())
      {
        SetNextState(desired_valve_position == ThreeWayValvePosition::DHW ? State::DHW_HEAT : State::HEAT_COOL);
        ESP_LOGI("amber", "3-way valve switch complete, new mode: %s", state_ == State::DHW_HEAT ? "DHW" : "HEAT/COOL");
      }
      break;
    }
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
void OpenAmberComponent::SetNextState(State state)
{
  state_ = state;
  const char* txt = StateToString(state);
  id(state_machine_state_heat_cool).publish_state(txt);
  ESP_LOGI("amber", "HEAT_COOL state changed: %s", txt);
}

const char* OpenAmberComponent::StateToString(State state)
{
  switch (state)
  {
    case State::INITIALIZING:
      return "Initializing";
    case State::SWITCHING:
      return "Switching";
    case State::DHW_HEAT:
      return "DHW";
    case State::HEAT_COOL:
      return "Heat/Cool";
    default:
      return "Unknown";
  }
}

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

  last_three_way_valve_switch_ms_ = millis();
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
