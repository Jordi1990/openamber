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
#include "timing.h"
#include "pump_controller.h"
#include "compressor_controller.h"

using namespace esphome;

template <typename StateEnum>
class BaseController
{
protected:
  StateEnum state_;
  StateEnum deferred_machine_state_;
  uint32_t defer_state_change_started_ms_ = 0;
  uint32_t defer_state_change_duration_ms_ = 0;
  uint32_t backup_heater_start_time_ms_ = 0;
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
    OnStateEnter(state_);
    ESP_LOGI("amber", "%s state changed: %s", LogTag(), txt);
  }

  virtual void OnStateEnter(StateEnum new_state) {}

  void LeaveStateAndSetNextStateAfterWaitTime(StateEnum new_state, uint32_t defer_ms)
  {
    deferred_machine_state_ = new_state;
    defer_state_change_started_ms_ = App.get_loop_component_start_time();
    defer_state_change_duration_ms_ = defer_ms;
    SetNextState(wait_for_state_switch_state_);
  }

  bool ProcessDeferredStateChange()
  {
    uint32_t now = App.get_loop_component_start_time();
    if (!HasElapsedMs(defer_state_change_started_ms_, defer_state_change_duration_ms_, now))
    {
      ESP_LOGD("amber", "Waiting for state switch, transitioning to next state in %lu ms",
               (unsigned long) RemainingMs(defer_state_change_started_ms_, defer_state_change_duration_ms_, now));
      return false;
    }
    defer_state_change_duration_ms_ = 0;
    SetNextState(deferred_machine_state_);
    deferred_machine_state_ = unknown_state_;
    return true;
  }

  /// @brief True while switching to standby has to wait for the outdoor unit's defrost (2118) to end.
  /// Standby aborts a running defrost without its recovery phase, but the unit only runs its pre-phase and
  /// defrost timers while the compressor runs (firmware v1.10/v1.18: the pre-phase counter needs the continuous
  /// compressor run time, the defrost timer only counts during the execution, and a compressor stop clears the
  /// execution but not 2118). With the compressor off 2118 therefore stays set until standby is written, so
  /// waiting for it would never end. Only wait while the compressor still runs, and never longer than
  /// DEFROST_STANDBY_WAIT_MAX_S after the pump stop.
  bool ShouldWaitForDefrostBeforeStandby(uint32_t now)
  {
    if (!id(defrost_active_sensor).state)
    {
      return false;
    }

    if (!compressor_controller_->IsRunning())
    {
      ESP_LOGW("amber", "%s: defrost flag (2118) set while the compressor is off, the outdoor unit cannot finish it. "
               "Switching to standby, which clears it.", LogTag());
      return false;
    }

    const uint32_t stop_started_ms = pump_controller_->GetStopWaitStartedTime();
    if (stop_started_ms != 0 && HasElapsedMs(stop_started_ms, DEFROST_STANDBY_WAIT_MAX_S * 1000UL, now))
    {
      ESP_LOGW("amber", "%s: defrost still active after %lu seconds, switching to standby anyway.", LogTag(),
               (unsigned long) DEFROST_STANDBY_WAIT_MAX_S);
      return false;
    }

    return true;
  }

  bool IsBackupHeaterActive()
  {
    if(id(backup_heating_mode).active_index().value() == 0)
    {
      return id(backup_heater_active_sensor).state;
    }

    return true; // Assume external backup heater is active when enabled, as we don't have a sensor for it.
  }

  void TurnOnBackupHeater()
  {
    backup_heater_start_time_ms_ = App.get_loop_component_start_time();
    if(id(backup_heating_mode).active_index().value() == 0)
    {      
      id(backup_heater_relay).turn_on();
    }
    else
    {
      id(external_backup_heating_relay).turn_on();
    }
  }

  void TurnOffBackupHeater()
  {
    backup_heater_start_time_ms_ = 0;
    if(id(backup_heater_relay).state)
    {      
      id(backup_heater_relay).turn_off();
    }
    else if(id(external_backup_heating_relay).state)
    {
      id(external_backup_heating_relay).turn_off();
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

  virtual ~BaseController() = default;

  void RequestToStop()
  {
    requested_to_stop_ = true;
    ESP_LOGI("amber", "%s controller requested to stop. Current state: %s", LogTag(), StateToString(state_));
  }

  bool IsRequestedToStop()
  {
    return requested_to_stop_;
  }

  /// @brief A stop requested by the user's controlled stop: it does not wait for the minimum compressor on time.
  bool IsControlledStopRequested()
  {
    // Maintenance mode wins over a controlled stop when both are on; it waits for the minimum on time.
    return requested_to_stop_ && id(controlled_stop_active) && !id(service_mode_enabled).state;
  }

  bool IsInIdleState()
  {
    return state_ == idle_state_;
  }

  bool IsWorkingMode(int working_mode)
  {
    return id(working_mode_switch).active_index().value() == working_mode;
  }

  void Init()
  {
    SetNextState(idle_state_);
  }
};
