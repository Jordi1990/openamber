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

template <typename StateEnum>
class BaseController
{
protected:
  StateEnum state_;
  StateEnum deferred_machine_state_;
  uint32_t defer_state_change_until_ms_ = 0;
  bool requested_to_stop_ = false;
  PumpController* pump_controller_;
  CompressorController* compressor_controller_;

  StateEnum wait_for_state_switch_state_;
  StateEnum unknown_state_;
  StateEnum idle_state_;

  virtual const char* StateToString(StateEnum state) const = 0;
  virtual void PublishState(const char* state_text) = 0;
  virtual const char* LogTag() const = 0;

  void SetNextState(StateEnum new_state)
  {
    state_ = new_state;
    const char* txt = StateToString(new_state);
    PublishState(txt);
    ESP_LOGI("amber", "%s state changed: %s", LogTag(), txt);
  }

  void LeaveStateAndSetNextStateAfterWaitTime(StateEnum new_state, uint32_t defer_ms)
  {
    deferred_machine_state_ = new_state;
    defer_state_change_until_ms_ = App.get_loop_component_start_time() + defer_ms;
    SetNextState(wait_for_state_switch_state_);
  }

  bool ProcessDeferredStateChange()
  {
    uint32_t now = App.get_loop_component_start_time();
    if (defer_state_change_until_ms_ > now)
    {
      ESP_LOGD("amber", "Waiting for state switch, transitioning to next state in %lu ms", defer_state_change_until_ms_ - now);
      return false;
    }
    defer_state_change_until_ms_ = 0;
    SetNextState(deferred_machine_state_);
    deferred_machine_state_ = unknown_state_;
    return true;
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
  BaseController(PumpController* pump_controller, CompressorController* compressor_controller,
                 StateEnum wait_for_state_switch, StateEnum unknown_state, StateEnum idle_state)
    : state_(idle_state),
      pump_controller_(pump_controller),
      compressor_controller_(compressor_controller),
      wait_for_state_switch_state_(wait_for_state_switch),
      unknown_state_(unknown_state),
      idle_state_(idle_state) {}

  void RequestToStop()
  {
    requested_to_stop_ = true;
    ESP_LOGI("amber", "%s controller requested to stop. Current state: %s", LogTag(), StateToString(state_));
  }

  bool IsRequestedToStop()
  {
    return requested_to_stop_;
  }

  bool IsInIdleState()
  {
    return state_ == idle_state_;
  }
};
