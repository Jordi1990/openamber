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

using namespace esphome;

/// @brief Lightweight base class for custom routines.
template <typename StateEnum>
class RoutineController
{
protected:
  StateEnum state_;
  StateEnum idle_state_;
  StateEnum wait_for_state_switch_state_;
  StateEnum deferred_machine_state_;
  uint32_t defer_state_change_until_ms_ = 0;
  bool requested_to_stop_ = false;

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
      ESP_LOGD("amber", "%s waiting for state switch, transitioning in %lu ms", LogTag(), defer_state_change_until_ms_ - now);
      return false;
    }
    defer_state_change_until_ms_ = 0;
    SetNextState(deferred_machine_state_);
    return true;
  }

  /// @brief Called by derived Start() methods to perform common initialization.
  void StartRoutine()
  {
    requested_to_stop_ = false;
  }

public:
  RoutineController(StateEnum idle_state, StateEnum wait_state)
    : state_(idle_state),
      idle_state_(idle_state),
      wait_for_state_switch_state_(wait_state) {}

  virtual ~RoutineController() = default;

  void RequestToStop()
  {
    if (state_ == idle_state_) return;
    requested_to_stop_ = true;
    ESP_LOGI("amber", "%s routine requested to stop. Current state: %s", LogTag(), StateToString(state_));
  }

  bool IsRequestedToStop() const
  {
    return requested_to_stop_;
  }

  bool IsIdle() const
  {
    return state_ == idle_state_;
  }

  bool IsRunning() const
  {
    return state_ != idle_state_;
  }

  virtual void UpdateStateMachine() = 0;
};
