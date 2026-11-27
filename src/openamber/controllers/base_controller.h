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

#include <cmath>

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
  uint32_t pump_flow_missing_since_ms_ = 0;
  uint32_t water_delta_t_exceeded_since_ms_ = 0;
  PumpController* pump_controller_;
  CompressorController* compressor_controller_;

  StateEnum wait_for_state_switch_state_;
  StateEnum unknown_state_;
  StateEnum idle_state_;
  StateEnum compressor_running_state_;

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

  /// @brief Hot switch: what this controller stops before the 3-way valve moves (secondary pumps, mixing valves).
  virtual void PrepareHandOver() = 0;

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

  bool IsBackupHeaterActive()
  {
    if(id(backup_heating_mode).active_index().value() == 0)
    {
      return id(backup_heater_active_sensor).state;
    }

    // Report the commanded state of the external relay (holding register 1108) instead of
    // assuming it is on, so callers see the heater as inactive once it has been turned off.
    return id(external_backup_heating_relay).state;
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
                 StateEnum wait_for_state_switch, StateEnum unknown_state, StateEnum idle_state,
                 StateEnum compressor_running_state)
    : state_(idle_state),
      pump_controller_(pump_controller),
      compressor_controller_(compressor_controller),
      wait_for_state_switch_state_(wait_for_state_switch),
      unknown_state_(unknown_state),
      idle_state_(idle_state),
      compressor_running_state_(compressor_running_state) {}

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

  /// @brief Hot switch between heating and DHW: true when the running compressor can be handed over to the other
  /// controller without stopping. Heating mode only (cooling needs the 4-way valve), and no backup heater,
  /// emergency mode, error, defrost or oil return cycle, and only with pump P0 running.
  bool CanHandOverRunningCompressor()
  {
    return state_ == compressor_running_state_ && IsWorkingMode(WORKING_MODE_HEATING) &&
           compressor_controller_->IsRunning() && compressor_controller_->IsCommanded() &&
           pump_controller_->IsRunning() && !IsBackupHeaterActive() &&
           !id(emergency_mode_enabled).state && !id(error_active).state && !id(defrost_active_sensor).state &&
           !id(oil_return_cycle_active).state;
  }

  /// @brief Leaves the compressor, pump P0 and working mode as they are and goes to idle, so the other controller
  /// can take over once the 3-way valve has moved.
  void HandOverRunningCompressor()
  {
    PrepareHandOver();
    requested_to_stop_ = false;
    pump_flow_missing_since_ms_ = 0;
    water_delta_t_exceeded_since_ms_ = 0;
    // The other controller takes over: this controller's checks stop running, so its valve check starts over (full run
    // time) when it gets the compressor back, instead of counting the time before the hand-over.
    ResetValvePositionCheck();
    SetNextState(idle_state_);
  }

  bool IsWorkingMode(int working_mode)
  {
    return id(working_mode_switch).active_index().value() == working_mode;
  }

  void Init()
  {
    SetNextState(idle_state_);
  }

protected:
  /// @brief 3-way valve position check: true when the outlet water stays far above the temperature of the circuit
  /// the valve should serve (the water comes from a warmer circuit).
  /// @param active Compressor running in a state where the valve position is settled (not defrosting).
  bool IsOutletFarAboveCircuit(bool active, float circuit_temperature)
  {
    return UpdateValvePositionCheck(active, circuit_temperature, true);
  }

  /// @brief 3-way valve position check: true when the outlet water stays far below the temperature of the circuit
  /// the valve should serve (the water comes from a colder circuit).
  /// @param active Compressor running in a state where the valve position is settled (not defrosting).
  bool IsOutletFarBelowCircuit(bool active, float circuit_temperature)
  {
    return UpdateValvePositionCheck(active, circuit_temperature, false);
  }

private:
  bool valve_check_running_ = false;
  uint32_t valve_check_running_since_ms_ = 0;
  bool valve_check_deviating_ = false;
  uint32_t valve_check_deviating_since_ms_ = 0;

  /// @brief After the compressor has run for THREE_WAY_VALVE_CHECK_RUN_TIME_S, the outlet water (Tuo minus
  /// THREE_WAY_VALVE_CHECK_TUO_OFFSET_C) must stay within THREE_WAY_VALVE_CHECK_DEVIATION_C of the circuit
  /// temperature; exactly that deviation is still allowed. A larger deviation in the checked direction that lasts
  /// THREE_WAY_VALVE_PROTECTION_HIGH_TEMPERATURE_TIME_S means the valve is stuck or in the wrong position.
  bool UpdateValvePositionCheck(bool active, float circuit_temperature, bool check_above)
  {
    const float tuo = id(outlet_temperature_tuo).state;
    const uint32_t now = App.get_loop_component_start_time();
    if (!active || std::isnan(tuo) || std::isnan(circuit_temperature))
    {
      ResetValvePositionCheck();
      return false;
    }

    if (!valve_check_running_)
    {
      valve_check_running_ = true;
      valve_check_running_since_ms_ = now;
    }
    if (!HasElapsedMs(valve_check_running_since_ms_, THREE_WAY_VALVE_CHECK_RUN_TIME_S * 1000UL, now))
    {
      valve_check_deviating_ = false;
      return false;
    }

    const float deviation = (tuo - THREE_WAY_VALVE_CHECK_TUO_OFFSET_C) - circuit_temperature;
    const bool deviating = check_above ? deviation > THREE_WAY_VALVE_CHECK_DEVIATION_C
                                       : deviation < -THREE_WAY_VALVE_CHECK_DEVIATION_C;
    if (!deviating)
    {
      valve_check_deviating_ = false;
      return false;
    }

    if (!valve_check_deviating_)
    {
      valve_check_deviating_ = true;
      valve_check_deviating_since_ms_ = now;
    }
    if (!HasElapsedMs(valve_check_deviating_since_ms_, THREE_WAY_VALVE_PROTECTION_HIGH_TEMPERATURE_TIME_S * 1000UL, now))
    {
      return false;
    }

    // Start over after a trip: once the error is reset, the full run and hold time apply again.
    ResetValvePositionCheck();
    return true;
  }

  void ResetValvePositionCheck()
  {
    valve_check_running_ = false;
    valve_check_deviating_ = false;
  }
};
