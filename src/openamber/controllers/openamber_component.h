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
#include "timing.h"
#include "dhw_controller.h"
#include "mixing_valve_actuator.h"
#include "heat_cool_controller.h"
#include "deaeration_routine.h"

using namespace esphome;

namespace esphome {
namespace openamber {

OpenAmberComponent::OpenAmberComponent()
{
  pump_controller_ = new PumpController();
  compressor_controller_ = new CompressorController();
  mixing_valve_zone1_ = new MixingValveActuator(
    id(mixing_valve_zone1_output),
    id(mixing_valve_zone1_min_position),
    id(mixing_valve_zone1_max_position));
  mixing_valve_zone2_ = new MixingValveActuator(
    id(mixing_valve_zone2_output),
    id(mixing_valve_zone2_min_position),
    id(mixing_valve_zone2_max_position));
  dhw_controller_ = new DHWController(pump_controller_, compressor_controller_);
  heat_cool_controller_ = new HeatCoolController(pump_controller_, compressor_controller_, mixing_valve_zone1_, mixing_valve_zone2_);
  deaeration_routine_ = new DeaerationRoutine();
}

OpenAmberComponent::~OpenAmberComponent()
{
  delete dhw_controller_;
  delete heat_cool_controller_;
  delete deaeration_routine_;
  delete pump_controller_;
  delete compressor_controller_;
  delete mixing_valve_zone1_;
  delete mixing_valve_zone2_;
}

void OpenAmberComponent::setup()
{
  ESP_LOGI("amber", "OpenAmberController initialized");
  dhw_controller_->Init();
  heat_cool_controller_->Init();
  SetNextState(State::WAIT_MODBUS_CONNECTION);
}

void OpenAmberComponent::loop()
{

}

void OpenAmberComponent::update()
{
  if ((state_ == State::HEAT_COOL || state_ == State::DHW_HEAT) && !id(pump_p0_relay_switch).state)
  {
    ota_in_progress_ = false;
    id(error_inside_unit_failsafe).publish_state(true);
    StopControllers();
    ESP_LOGI("amber", "Inside unit pump relay switch unexpectedly turned off. Re-initializing controller.");
    SetNextState(State::WAIT_MODBUS_CONNECTION);
    return;
  }

  // The outdoor unit stopped a running compressor by itself (protection, power or communication loss). We do not
  // know why, so stop everything, re-initialize and wait for the user to reset the error.
  if ((state_ == State::HEAT_COOL || state_ == State::DHW_HEAT) && compressor_controller_->HasStoppedUnexpectedly())
  {
    ota_in_progress_ = false;
    id(error_compressor_unexpected_stop).publish_state(true);
    ESP_LOGE("amber", "Compressor reports 0 Hz for %u s while it should run. Stopping and re-initializing controller.",
             (unsigned) COMPRESSOR_UNEXPECTED_STOP_S);
    StopControllers();
    SetNextState(State::WAIT_MODBUS_CONNECTION);
    return;
  }

  // Emergency mode is only checked when a controller would start the compressor. When it is enabled while the
  // compressor is requested, stop everything and re-initialize: regulation then starts again in emergency mode
  // (backup heater only). The request, not the frequency, is checked, so the run-down after the stop does not
  // trigger this again.
  if ((state_ == State::HEAT_COOL || state_ == State::DHW_HEAT) && id(emergency_mode_enabled).state &&
      id(compressor_control_select).active_index().value_or(0) > 0)
  {
    ota_in_progress_ = false;
    ESP_LOGW("amber", "Emergency mode enabled while the compressor is requested. Stopping and re-initializing controller.");
    StopControllers();
    SetNextState(State::WAIT_MODBUS_CONNECTION);
    return;
  }

  // The other way around: emergency mode is switched off while a controller regulates on the backup heater
  // alone. Those states assume a compressor that was started the normal way (pump settle time, minimum off time,
  // soft start, start time), so stop and re-initialize; regulation then starts again with the compressor.
  const bool emergency_mode_enabled_now = id(emergency_mode_enabled).state;
  const bool emergency_mode_switched_off = emergency_mode_was_enabled_ && !emergency_mode_enabled_now;
  emergency_mode_was_enabled_ = emergency_mode_enabled_now;
  if ((state_ == State::HEAT_COOL || state_ == State::DHW_HEAT) && emergency_mode_switched_off &&
      (id(backup_heater_relay).state || id(external_backup_heating_relay).state) &&
      id(compressor_control_select).active_index().value_or(0) == 0)
  {
    ota_in_progress_ = false;
    ESP_LOGW("amber", "Emergency mode disabled while regulating on the backup heater. Stopping and re-initializing controller.");
    StopControllers();
    SetNextState(State::WAIT_MODBUS_CONNECTION);
    return;
  }

  // Also during the OTA hold: a lost Modbus connection must still raise its error (error_active), which the
  // safety check below turns into a stop.
  bool modbus_connected = id(modbus_inside_online).state && id(modbus_outside_online).state && id(outside_unit_eeprom_version).has_state();

  if(!modbus_connected)
  {
    CheckModbusConnectionTimeout();
  }
  else 
  {
    modbus_disconnected_since_ms_ = 0;
  }

  if (ota_in_progress_)
  {
    // Freeze the state machine while the new firmware is written: the units keep their current state and the
    // controller takes them over again after the reboot (RESUME_CHECK). No new starts or stops just before a reboot.
    // Every OTA progress report extends the hold, so it only times out when the update stopped reporting.
    if (App.get_loop_component_start_time() - ota_last_activity_ms_ >= OTA_HOLD_TIMEOUT_S * 1000UL)
    {
      ESP_LOGW("amber", "No OTA progress for %lu s, resuming regulation.", (unsigned long) OTA_HOLD_TIMEOUT_S);
      ota_in_progress_ = false;
    }
    else if (HasSafetyStopCondition())
    {
      // Safety stops are not held: release the hold, the regular state machine below performs the stop.
      ESP_LOGW("amber", "Safety condition during OTA update, releasing the hold to stop the units.");
      ota_in_progress_ = false;
    }
    else
    {
      return;
    }
  }

  ThreeWayValvePosition current_valve_position = GetThreeWayValvePosition();
  ThreeWayValvePosition desired_valve_position = GetDesiredThreeWayValvePosition();
  State desired_state = modbus_disconnected_error_occurred_ ? State::WAIT_MODBUS_CONNECTION : (desired_valve_position == ThreeWayValvePosition::DHW ? State::DHW_HEAT : State::HEAT_COOL);
  bool maintenance_requested = id(service_mode_enabled).state;
  // Maintenance mode wins over a controlled stop when both are on.
  bool controlled_stop_requested = id(controlled_stop_active) && !maintenance_requested;

  switch (state_)
  {
    case State::UNKNOWN:
    {
      break;
    }

    case State::WAIT_MODBUS_CONNECTION:
    {
      if(modbus_connected)
      {
        modbus_disconnected_error_occurred_ = false;
        // Pump P0 relay read as off (inside unit restarted, failsafe, maintenance): nothing to take over.
        // has_state(): an unread switch is false by default, which must not stop running units.
        if (id(pump_p0_relay_switch).has_state() && !id(pump_p0_relay_switch).state)
        {
          ESP_LOGI("amber", "Modbus connection established, transitioning to initialization.");
          SetNextState(State::INITIALIZING);
        }
        else
        {
          ESP_LOGI("amber", "Modbus connection established, checking whether the units are still running.");
          resume_check_started_ms_ = App.get_loop_component_start_time();
          SetNextState(State::RESUME_CHECK);
        }
      }
      break;
    }

    case State::RESUME_CHECK:
    {
      // After a restart of the ESP the units may still run on the commands from before the restart.
      // The online flag of the inside unit is set by 1303; other ranges (e.g. valve feedback 1305, 1312) can
      // arrive later. Wait for those readbacks: an unread entity looks like "off" and would stop running units.
      if (!AreResumeReadbacksAvailable())
      {
        if (!HasElapsedMs(resume_check_started_ms_, RESUME_READBACK_TIMEOUT_S * 1000UL, App.get_loop_component_start_time()))
        {
          ESP_LOGD("amber", "Resume: waiting for the Modbus readbacks of the units.");
          break;
        }
        ESP_LOGW("amber", "Resume: readbacks not available after %lu s, normal initialization.", (unsigned long) RESUME_READBACK_TIMEOUT_S);
        SetNextState(State::INITIALIZING);
        break;
      }
      if (CanResumeRunningUnits())
      {
        ResumeRunningUnits();
      }
      else
      {
        SetNextState(State::INITIALIZING);
      }
      break;
    }

    case State::INITIALIZING:
    {
      bool is_hp_s_model = id(outside_unit_eeprom_version).state == EEPROM_VERSION_HPS;
      if (!is_hp_s_model)
      {
        id(initialize_relay_switch).turn_on();
      }
      id(pump_p0_relay_switch).turn_on();
      id(pump_p1_relay_switch).turn_off();
      id(dhw_pump_relay_switch).turn_off();
      id(three_way_valve_dhw_switch).turn_off();
      id(three_way_valve_heat_cool_switch).turn_off();
      id(backup_heater_stage_1).turn_off();
      id(backup_heater_stage_2).turn_off();
      // Close mixing valves during init.
      id(mixing_valve_zone1_output).make_call().set_value(0).perform();
      id(mixing_valve_zone2_output).make_call().set_value(0).perform();
      auto working_mode_call = id(working_mode_switch).make_call();
      working_mode_call.set_index(WORKING_MODE_STANDBY);
      working_mode_call.perform();
      compressor_controller_->Stop();
      pump_controller_->Stop();
      // Stop() restarts the pump interval; after (re)initialization the pump must start right away on pump demand.
      pump_controller_->ResetInterval();
      WriteEEPROMConfiguration();
      ESP_LOGI("amber", "Initialized heat pump controller");
      LeaveStateAndSetNextStateAfterWaitTime(State::WAIT_INITIALIZATION, INITIALIZATION_DELAY_S * 1000UL);
      break;
    }

    case State::WAIT_INITIALIZATION:
    {
      SwitchThreeWayValveAndWait(desired_valve_position, desired_state);
      break;
    }

    case State::HEAT_COOL:
    {
      dhw_controller_->CheckLegionellaCycle();

      if((desired_valve_position == ThreeWayValvePosition::DHW || maintenance_requested || controlled_stop_requested) && !heat_cool_controller_->IsRequestedToStop())
      {
        heat_cool_controller_->RequestToStop();
      }

      if(heat_cool_controller_->IsInIdleState())
      {
        if(maintenance_requested)
        {
          auto working_mode_call = id(working_mode_switch).make_call();
          working_mode_call.set_index(WORKING_MODE_MAINTENANCE);
          working_mode_call.perform();
          id(pump_p0_relay_switch).turn_off();
          heat_cool_controller_->StopPumpP1IfNeeded();
          SetNextState(State::MAINTENANCE);
          break;
        }
        else if(controlled_stop_requested)
        {
          EnterStandby();
          break;
        }
        else if(desired_valve_position != current_valve_position)
        {
          // P1 keeps running while the heat/cool controller idles with heat demand. Running it during DHW
          // pulls water through the CV side of the 3-way valve, so hot DHW water ends up in the floor.
          heat_cool_controller_->StopPumpP1IfNeeded();
          SwitchThreeWayValveAndWait(desired_valve_position, desired_state);
          break;
        }
      }

      heat_cool_controller_->UpdateStateMachine();
      break;
    }

    case State::DHW_HEAT:
    {
      dhw_controller_->CheckLegionellaCycle();

      if((maintenance_requested || controlled_stop_requested) && !dhw_controller_->IsRequestedToStop())
      {
        dhw_controller_->RequestToStop();
      }

      if(dhw_controller_->IsInIdleState())
      {
        if(maintenance_requested)
        {
          auto working_mode_call = id(working_mode_switch).make_call();
          working_mode_call.set_index(WORKING_MODE_MAINTENANCE);
          working_mode_call.perform();
          SetNextState(State::MAINTENANCE);          
          break;
        }
        else if(controlled_stop_requested)
        {
          EnterStandby();
          break;
        }
        else if(desired_valve_position != current_valve_position)
        {
          SwitchThreeWayValveAndWait(desired_valve_position, desired_state);
          break;
        }
      }

      dhw_controller_->UpdateStateMachine();
      break;
    }

    case State::STANDBY:
    {
      // A deliberate stop: no frost protection while stopped. A forgotten stop resumes by itself after 30 minutes.
      const bool timed_out = App.get_loop_component_start_time() - standby_started_ms_ >= CONTROLLED_STOP_AUTO_RESUME_S * 1000UL;
      if (!controlled_stop_requested || timed_out)
      {
        if (timed_out)
        {
          ESP_LOGW("amber", "Controlled stop ends (30 min passed), resuming regulation.");
        }
        else
        {
          ESP_LOGI("amber", "Controlled stop ends, resuming regulation.");
        }
        id(controlled_stop_active) = false;
        // Maintenance mode requested while stopped: re-initialize, it takes over from there.
        SetNextState(State::WAIT_MODBUS_CONNECTION);
      }
      break;
    }

    case State::MAINTENANCE:
    {
      if (!maintenance_requested)
      {
        if (!deaeration_routine_->IsIdle())
        {
          deaeration_routine_->Stop();
        }
        SetNextState(State::WAIT_MODBUS_CONNECTION);
        break;
      }

      // Update active routines
      if (!deaeration_routine_->IsIdle())
      {
        deaeration_routine_->UpdateStateMachine();
      }
      break;
    }

    case State::WAIT_THREE_WAY_VALVE:
    {
      if (App.get_loop_component_start_time() - valve_switch_started_ms_ < THREE_WAY_VALVE_SWITCH_TIME_S * 1000UL)
      {
        break;
      }

      if (!IsThreeWayValveFeedbackConfirmed(valve_target_position_))
      {
        // The active error suppresses all demand, so the next state stays idle until the error is reset.
        ESP_LOGE("amber", "3-way valve relay feedback did not confirm the requested position, stopping system.");
        id(error_three_way_valve_state_timeout).publish_state(true);
      }
      SetNextState(valve_target_state_);
      break;
    }

    case State::WAIT_FOR_STATE_SWITCH:
    {
      uint32_t now = App.get_loop_component_start_time();
      if (!HasElapsedMs(defer_state_change_started_ms_, defer_state_change_duration_ms_, now) && !maintenance_requested)
      {
        ESP_LOGD("amber", "Waiting for state switch, transitioning to next state in %lu ms",
                 (unsigned long) RemainingMs(defer_state_change_started_ms_, defer_state_change_duration_ms_, now));
      }
      else
      {
        defer_state_change_duration_ms_ = 0;
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

void OpenAmberComponent::write_pump_p0_pid_value(float value)
{
  pump_controller_->SetPumpP0PidOutput(value);
}

void OpenAmberComponent::reset_pump_interval()
{
  pump_controller_->ResetInterval();
}

bool OpenAmberComponent::is_maintenance_state() const
{
  return state_ == State::MAINTENANCE;
}

void OpenAmberComponent::start_deaeration_routine(bool extended)
{
  if (state_ == State::MAINTENANCE && deaeration_routine_->IsIdle())
  {
    deaeration_routine_->Start(extended);
  }
}

void OpenAmberComponent::stop_deaeration_routine()
{
  if (!deaeration_routine_->IsIdle())
  {
    deaeration_routine_->Stop();
  }
}

bool OpenAmberComponent::is_deaeration_running() const
{
  return !deaeration_routine_->IsIdle();
}

bool OpenAmberComponent::is_deaeration_extended() const
{
  return deaeration_routine_->is_extended();
}

int OpenAmberComponent::get_deaeration_state() const
{
  return deaeration_routine_->GetStateId();
}

bool OpenAmberComponent::is_deaeration_dhw_circuit() const
{
  return deaeration_routine_->IsDhwCircuit();
}

int OpenAmberComponent::get_deaeration_current_cycle() const
{
  return deaeration_routine_->GetCurrentCycle();
}

int OpenAmberComponent::get_deaeration_cycle_count() const
{
  return deaeration_routine_->GetCycleCount();
}

int OpenAmberComponent::get_deaeration_progress_percent() const
{
  return deaeration_routine_->GetProgressPercent();
}

uint32_t OpenAmberComponent::get_deaeration_remaining_seconds() const
{
  return deaeration_routine_->GetRemainingSeconds();
}

std::string OpenAmberComponent::get_deaeration_phase_text() const
{
  return deaeration_routine_->GetPhaseText();
}

uint32_t OpenAmberComponent::get_deaeration_duration_seconds(bool extended) const
{
  return deaeration_routine_->GetTotalDurationS(extended);
}

void OpenAmberComponent::on_ota_started()
{
  if (!ota_in_progress_)
  {
    ESP_LOGI("amber", "OTA update started, holding the state machine. Main state: %s", StateToString(state_));
  }
  ota_in_progress_ = true;
  ota_last_activity_ms_ = App.get_loop_component_start_time();
}

void OpenAmberComponent::on_ota_progress()
{
  // Each progress report extends the hold. It also restores a hold that another OTA source released by
  // failing while this upload continues.
  if (!ota_in_progress_)
  {
    ESP_LOGI("amber", "OTA update in progress, holding the state machine. Main state: %s", StateToString(state_));
  }
  ota_in_progress_ = true;
  ota_last_activity_ms_ = App.get_loop_component_start_time();
}

void OpenAmberComponent::on_ota_finished(bool success)
{
  if (!ota_in_progress_)
  {
    return;
  }
  if (success)
  {
    // The device reboots right after this. Keep holding: the units keep running and are taken over after boot.
    ESP_LOGI("amber", "OTA update written, the controller resumes the running units after the reboot.");
    return;
  }
  ESP_LOGW("amber", "OTA update failed or aborted, resuming regulation.");
  ota_in_progress_ = false;
}

#ifdef USE_HOST
void OpenAmberComponent::simulate_restart()
{
  ESP_LOGI("amber", "Simulating a restart of the ESP (units keep their state).");
  delete dhw_controller_;
  delete heat_cool_controller_;
  delete pump_controller_;
  delete compressor_controller_;
  pump_controller_ = new PumpController();
  compressor_controller_ = new CompressorController();
  dhw_controller_ = new DHWController(pump_controller_, compressor_controller_);
  heat_cool_controller_ = new HeatCoolController(pump_controller_, compressor_controller_, mixing_valve_zone1_, mixing_valve_zone2_);
  ota_in_progress_ = false;
  modbus_disconnected_since_ms_ = 0;
  modbus_disconnected_error_occurred_ = false;
  defer_state_change_started_ms_ = 0;
  defer_state_change_duration_ms_ = 0;
  deferred_machine_state_ = State::UNKNOWN;
  setup();
}
#endif

// Privates

/// @brief True when the controller that owns the current main state wants a safety stop (error, missing flow,
/// Tuo-Tui difference). Used during the OTA hold, where the regular state machine does not run.
bool OpenAmberComponent::HasSafetyStopCondition()
{
  switch (state_)
  {
    case State::HEAT_COOL:
      return heat_cool_controller_->HasSafetyStopCondition();
    case State::DHW_HEAT:
      return dhw_controller_->HasSafetyStopCondition();
    default:
      return false;
  }
}

/// @brief True when the readbacks that CanResumeRunningUnits() relies on have been read at least once.
bool OpenAmberComponent::AreResumeReadbacksAvailable()
{
  return id(internal_pump_active).has_state() &&
         id(current_compressor_frequency).has_state() &&
         id(compressor_control_select).has_state() &&
         id(working_mode_switch).has_state() &&
         id(three_way_valve_dhw_switch).has_state() &&
         id(three_way_valve_heat_cool_switch).has_state() &&
         id(three_way_valve_active_sensor).has_state() &&
         id(three_way_valve_dhw_active_sensor).has_state() &&
         id(dhw_pump_relay_switch).has_state();
}

/// @brief Writes the unit configuration that lives in RAM of the units. Safe to write while the units run.
void OpenAmberComponent::WriteEEPROMConfiguration()
{
  if (id(outside_unit_eeprom_version).state == EEPROM_VERSION_HPS)
  {
    ESP_LOGI("amber", "Detected HPS unit based on EEPROM version, not applying outdoor unit optimizations.");
    return;
  }

  if (!id(outdoor_unit_optimizations_enabled).state)
  {
    ESP_LOGI("amber", "Outdoor unit optimizations disabled, not writing any outdoor unit parameters.");
    return;
  }

  WriteHeatingFrequencyTable();
  WriteCoolingFrequencyTable();

  // OpenAmber limits the compressor per outdoor temperature in Hz (CompressorController), including
  // outside the operating range. Turn off the unit's own level-based limit (RAM-only).
  id(outdoor_unit_ambient_limit_control).make_call().set_value(0).perform();
}

/// @brief After a restart of the ESP (OTA update, crash, manual restart) the units can still run on the last
/// commands. Checks from the Modbus readbacks whether that state can be taken over. Only reads, writes nothing.
/// @return false when there is nothing to take over or the state is not consistent; INITIALIZING then runs as usual.
bool OpenAmberComponent::CanResumeRunningUnits()
{
  if (id(service_mode_enabled).state)
  {
    ESP_LOGI("amber", "Resume: maintenance mode requested, normal initialization.");
    return false;
  }
  if (id(error_active).state)
  {
    ESP_LOGI("amber", "Resume: an error is active, normal initialization.");
    return false;
  }
  if (id(emergency_mode_enabled).state)
  {
    // Emergency mode must not take over a running compressor; initialization starts it on the backup heater.
    ESP_LOGI("amber", "Resume: emergency mode enabled, normal initialization.");
    return false;
  }

  const int working_mode = id(working_mode_switch).active_index().value_or(WORKING_MODE_STANDBY);

  ESP_LOGI("amber", "Resume: P0 relay=%d, pump=%d, compressor running=%d, level=%d, working mode=%d, defrost=%d, DHW pump=%d",
           id(pump_p0_relay_switch).state, id(internal_pump_active).state, compressor_controller_->IsRunning(),
           (int) id(compressor_control_select).active_index().value_or(0), working_mode, id(defrost_active_sensor).state,
           id(dhw_pump_relay_switch).state);

  if (!id(pump_p0_relay_switch).state)
  {
    ESP_LOGI("amber", "Resume: pump P0 relay is off (inside unit restarted or failsafe), normal initialization.");
    return false;
  }
  if (!id(internal_pump_active).state)
  {
    // Nothing runs, or a compressor without flow: let INITIALIZING put everything in a known state.
    ESP_LOGI("amber", "Resume: pump P0 not running, normal initialization.");
    return false;
  }
  if (working_mode != WORKING_MODE_HEATING && working_mode != WORKING_MODE_COOLING)
  {
    ESP_LOGI("amber", "Resume: working mode %d is not heating or cooling, normal initialization.", working_mode);
    return false;
  }

  const bool valve_dhw = id(three_way_valve_dhw_switch).state;
  const bool valve_heat_cool = id(three_way_valve_heat_cool_switch).state;
  if (valve_dhw == valve_heat_cool)
  {
    ESP_LOGW("amber", "Resume: 3-way valve outputs are inconsistent (DHW=%d, heat/cool=%d), normal initialization.", valve_dhw, valve_heat_cool);
    return false;
  }
  const ThreeWayValvePosition position = valve_dhw ? ThreeWayValvePosition::DHW : ThreeWayValvePosition::HEATING_COOLING;
  if (!IsThreeWayValveFeedbackConfirmed(position))
  {
    ESP_LOGW("amber", "Resume: 3-way valve feedback does not match the output, normal initialization.");
    return false;
  }
  if (position == ThreeWayValvePosition::DHW && working_mode != WORKING_MODE_HEATING)
  {
    ESP_LOGW("amber", "Resume: DHW position with working mode %d, normal initialization.", working_mode);
    return false;
  }
  return true;
}

/// @brief Takes over the running units instead of INITIALIZING (only after CanResumeRunningUnits()).
/// Writes the non-stopping part of INITIALIZING (configuration in RAM of the units), then puts the
/// controllers in the state that matches what the units are doing.
void OpenAmberComponent::ResumeRunningUnits()
{
  WriteEEPROMConfiguration();
  // The backup heater timing (prediction, settle time) is lost with the restart; the controllers switch it on again when needed.
  id(backup_heater_relay).turn_off();
  // backup_heater_relay only forwards a change to the stages; after a reboot it is already off, so switch the
  // stages off directly in case they were left on.
  id(backup_heater_stage_1).turn_off();
  id(backup_heater_stage_2).turn_off();
  id(external_backup_heating_relay).turn_off();

  const ThreeWayValvePosition position = GetThreeWayValvePosition();
  valve_target_position_ = position;

  if (position == ThreeWayValvePosition::DHW)
  {
    dhw_controller_->ResumeFromRunningUnits();
    ESP_LOGI("amber", "Resume: taking over running DHW cycle.");
    SetNextState(State::DHW_HEAT);
  }
  else
  {
    heat_cool_controller_->ResumeFromRunningUnits();
    ESP_LOGI("amber", "Resume: taking over running heat/cool cycle.");
    SetNextState(State::HEAT_COOL);
  }
}

void OpenAmberComponent::SetNextState(State state)
{
  state_ = state;
  const char* txt = StateToString(state);
  id(state_machine_state_main).publish_state(txt);
  ESP_LOGI("amber", "Main State changed: %s", txt);
}

void OpenAmberComponent::LeaveStateAndSetNextStateAfterWaitTime(State new_state, uint32_t defer_ms)
{
  deferred_machine_state_ = new_state;
  defer_state_change_started_ms_ = App.get_loop_component_start_time();
  defer_state_change_duration_ms_ = defer_ms;
  SetNextState(State::WAIT_FOR_STATE_SWITCH);
}

const char* OpenAmberComponent::StateToString(State state)
{
  switch (state)
  {
    case State::WAIT_MODBUS_CONNECTION:
      return "Waiting for Modbus connection";
    case State::INITIALIZING:
      return "Initializing";
    case State::WAIT_INITIALIZATION:
      return "Waiting for initialization";
    case State::WAIT_FOR_STATE_SWITCH:
      return "Waiting for state switch";
    case State::WAIT_THREE_WAY_VALVE:
      return "Waiting for 3-way valve";
    case State::DHW_HEAT:
      return "DHW";
    case State::HEAT_COOL:
      return "Heat/Cool";
    case State::MAINTENANCE:
      return "Maintenance";
    case State::STANDBY:
      return "Standby";
    case State::RESUME_CHECK:
      return "Checking running units";
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

/// @brief Sends the 3-way valve command and waits for the valve travel time before entering next_state.
/// The inside unit relay feedback is checked at the end of that wait.
void OpenAmberComponent::SwitchThreeWayValveAndWait(ThreeWayValvePosition position, State next_state)
{
  SetThreeWayValve(position);
  valve_target_position_ = position;
  valve_target_state_ = next_state;
  valve_switch_started_ms_ = App.get_loop_component_start_time();
  SetNextState(State::WAIT_THREE_WAY_VALVE);
}

/// @brief Checks the inside unit relay feedback registers: 1305 mirrors the heat/cool valve output (1105),
/// 1312 mirrors the DHW valve output (1112). They only confirm the output was switched, not the valve end position.
bool OpenAmberComponent::IsThreeWayValveFeedbackConfirmed(ThreeWayValvePosition position)
{
  if (!id(three_way_valve_active_sensor).has_state() || !id(three_way_valve_dhw_active_sensor).has_state())
  {
    return false;
  }
  bool want_dhw = position == ThreeWayValvePosition::DHW;
  return id(three_way_valve_dhw_active_sensor).state == want_dhw && id(three_way_valve_active_sensor).state == !want_dhw;
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
  if (id(dhw_demand_active_sensor).state)
  {
    return ThreeWayValvePosition::DHW;
  }
  
  // Otherwise use heating/cooling position
  return ThreeWayValvePosition::HEATING_COOLING;
}

void OpenAmberComponent::WriteHeatingFrequencyTable()
{
    // Patch heating frequency table to have more control in low load situations.
    id(heating_frequency_index_1).make_call().set_value(20).perform();
    id(heating_frequency_index_2).make_call().set_value(28).perform();
    id(heating_frequency_index_3).make_call().set_value(36).perform();
    id(heating_frequency_index_4).make_call().set_value(43).perform();
    id(heating_frequency_index_5).make_call().set_value(55).perform();
    id(heating_frequency_index_6).make_call().set_value(63).perform();
    id(heating_frequency_index_7).make_call().set_value(72).perform();
    id(heating_frequency_index_8).make_call().set_value(79).perform();
    id(heating_frequency_index_9).make_call().set_value(85).perform();
    id(heating_frequency_index_10).make_call().set_value(90).perform();
}

void OpenAmberComponent::WriteCoolingFrequencyTable()
{
    id(cooling_frequency_index_0).make_call().set_value(0).perform();
    id(cooling_frequency_index_1).make_call().set_value(20).perform();
    id(cooling_frequency_index_2).make_call().set_value(26).perform();
    id(cooling_frequency_index_3).make_call().set_value(30).perform();
    id(cooling_frequency_index_4).make_call().set_value(36).perform();
    id(cooling_frequency_index_5).make_call().set_value(43).perform();
    id(cooling_frequency_index_6).make_call().set_value(48).perform();
    id(cooling_frequency_index_7).make_call().set_value(55).perform();
    id(cooling_frequency_index_8).make_call().set_value(69).perform();
    id(cooling_frequency_index_9).make_call().set_value(74).perform();
    id(cooling_frequency_index_10).make_call().set_value(82).perform();
}

void OpenAmberComponent::CheckModbusConnectionTimeout()
{
  // Check for timeout
  if(modbus_disconnected_since_ms_ == 0)
  {
    modbus_disconnected_since_ms_ = App.get_loop_component_start_time();
  }

  const uint32_t timeout_ms = MODBUS_CONNECTION_TIMEOUT_S * 1000UL;
  const uint32_t now = App.get_loop_component_start_time();
  if((now - modbus_disconnected_since_ms_) >= timeout_ms)
  {
    if(!id(error_modbus_connection_timeout).state)
    {
      ESP_LOGE("amber", "Modbus connection timeout reached after %lu seconds.", (unsigned long) MODBUS_CONNECTION_TIMEOUT_S);
      id(error_modbus_connection_timeout).publish_state(true);
    }

    modbus_disconnected_error_occurred_ = true;
  }
}

void OpenAmberComponent::write_mixing_valve_zone1_pid_value(float value)
{
  if (mixing_valve_zone1_)
    mixing_valve_zone1_->SetPidOutput(value);
}

void OpenAmberComponent::write_mixing_valve_zone2_pid_value(float value)
{
  if (mixing_valve_zone2_)
    mixing_valve_zone2_->SetPidOutput(value);
}

float OpenAmberComponent::get_mixing_valve_zone1_position() const
{
  return mixing_valve_zone1_ ? mixing_valve_zone1_->GetValvePositionPercent() : 0.0f;
}

float OpenAmberComponent::get_mixing_valve_zone2_position() const
{
  return mixing_valve_zone2_ ? mixing_valve_zone2_->GetValvePositionPercent() : 0.0f;
}

/// @brief The controller is idle after a controlled stop, so the compressor and the pumps are off.
void OpenAmberComponent::EnterStandby()
{
  // The heat/cool controller keeps P1 running in IDLE on heat demand, and STANDBY no longer updates it.
  heat_cool_controller_->StopPumpP1IfNeeded();
  standby_started_ms_ = App.get_loop_component_start_time();
  ESP_LOGI("amber", "Controlled stop: units stopped, regulation paused.");
  SetNextState(State::STANDBY);
}

void OpenAmberComponent::StopControllers()
{
  ESP_LOGE("amber", "Stopping regulation (inside unit failsafe/reboot or unexpected compressor stop).");
  compressor_controller_->Stop();
  pump_controller_->Stop();
  heat_cool_controller_->StopAndSetIdleState();
  dhw_controller_->StopAndSetIdleState();
  auto working_mode_call = id(working_mode_switch).make_call();
  working_mode_call.set_index(WORKING_MODE_STANDBY);
  working_mode_call.perform();
}

}  // namespace openamber
}  // namespace esphome
