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
  state_ = State::INITIALIZING;
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

  switch (state_)
  {
    case State::INITIALIZING:
    {
      id(initialize_relay_switch).turn_on();
      id(pump_p0_relay_switch).turn_on();
      id(pump_p1_relay_switch).turn_off();
      id(dhw_pump_relay_switch).turn_off();
      id(three_way_valve_dhw_switch).turn_off();
      id(three_way_valve_heat_cool_switch).turn_off();
      id(backup_heater_stage_1).turn_off();
      id(backup_heater_stage_2).turn_off();
      auto working_mode_call = id(working_mode_switch).make_call();
      working_mode_call.set_index(WORKING_MODE_STANDBY);
      working_mode_call.perform();
      compressor_controller_->Stop();
      pump_controller_->Stop();
      WriteHeatingFrequencyTable();
      ESP_LOGI("amber", "Initialized heat pump controller");
      LeaveStateAndSetNextStateAfterWaitTime(State::WAIT_INITIALIZATION, INITIALIZATION_DELAY_S * 1000UL);
      break;
    }

    case State::WAIT_INITIALIZATION:
    {
      SetThreeWayValve(desired_valve_position);
      LeaveStateAndSetNextStateAfterWaitTime(desired_valve_position == ThreeWayValvePosition::DHW ? State::DHW_HEAT : State::HEAT_COOL, THREE_WAY_VALVE_SWITCH_TIME_S * 1000UL);
      break;
    }

    case State::HEAT_COOL:
    {
      dhw_controller_->CheckLegionellaCycle();

      if(desired_valve_position == ThreeWayValvePosition::DHW && !heat_cool_controller_->IsRequestedToStop())
      {
        heat_cool_controller_->RequestToStop();
      }

      if(heat_cool_controller_->IsInIdleState() && desired_valve_position != current_valve_position)
      {
        SetThreeWayValve(desired_valve_position);
        LeaveStateAndSetNextStateAfterWaitTime(desired_valve_position == ThreeWayValvePosition::DHW ? State::DHW_HEAT : State::HEAT_COOL, THREE_WAY_VALVE_SWITCH_TIME_S * 1000UL);
        break;
      }

      heat_cool_controller_->UpdateStateMachine();
      break;
    }

    case State::DHW_HEAT:
    {
      dhw_controller_->CheckLegionellaCycle();

      if(dhw_controller_->IsInIdleState() && desired_valve_position != current_valve_position)
      {
        SetThreeWayValve(desired_valve_position);
        LeaveStateAndSetNextStateAfterWaitTime(desired_valve_position == ThreeWayValvePosition::DHW ? State::DHW_HEAT : State::HEAT_COOL, THREE_WAY_VALVE_SWITCH_TIME_S * 1000UL);
        break;
      }

      dhw_controller_->UpdateStateMachine();
      break;
    }

    case State::WAIT_FOR_STATE_SWITCH:
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
        deferred_machine_state_ = State::UNKNOWN;
      }
      break;
    }
  }
}

void OpenAmberComponent::write_heat_pid_value(float value)
{
  heat_cool_controller_->SetHeatPIDValue(value);
}

void OpenAmberComponent::write_cool_pid_value(float value)
{
  heat_cool_controller_->SetCoolPIDValue(value);
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
  id(state_machine_state_main).publish_state(txt);
  ESP_LOGI("amber", "State changed: %s", txt);
}

void OpenAmberComponent::LeaveStateAndSetNextStateAfterWaitTime(State new_state, uint32_t defer_ms)
{
  deferred_machine_state_ = new_state;
  defer_state_change_until_ms_ = App.get_loop_component_start_time() + defer_ms;
  SetNextState(State::WAIT_FOR_STATE_SWITCH);
}

const char* OpenAmberComponent::StateToString(State state)
{
  switch (state)
  {
    case State::INITIALIZING:
      return "Initializing";
    case State::WAIT_INITIALIZATION:
      return "Waiting for initialization";
    case State::WAIT_FOR_STATE_SWITCH:
      return "Waiting for state switch";
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
  // TODO: Potentially prioritize heating when in certain conditions.

  // DHW has priority
  if (dhw_controller_->CanStartDhw())
  {
    return ThreeWayValvePosition::DHW;
  }
  
  // Otherwise use heating/cooling position
  return ThreeWayValvePosition::HEATING_COOLING;
}

void OpenAmberComponent::WriteHeatingFrequencyTable()
{
      // Patch heating frequency table to have more control in low load situations.
      id(heating_frequency_index_1).make_call().set_value(30).perform();
      id(heating_frequency_index_2).make_call().set_value(36).perform();
      id(heating_frequency_index_3).make_call().set_value(43).perform();
      id(heating_frequency_index_4).make_call().set_value(49).perform();
      id(heating_frequency_index_5).make_call().set_value(55).perform();
      id(heating_frequency_index_6).make_call().set_value(61).perform();
      id(heating_frequency_index_7).make_call().set_value(69).perform();
      id(heating_frequency_index_8).make_call().set_value(74).perform();
      id(heating_frequency_index_9).make_call().set_value(82).perform();
      id(heating_frequency_index_10).make_call().set_value(90).perform();
}
}  // namespace openamber
}  // namespace esphome
