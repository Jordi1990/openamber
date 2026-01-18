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
  
using namespace esphome;

// Frequency cannot change faster than given amount of seconds.
// Outside unit manages this aswell.
static const uint32_t FREQUENCY_CHANGE_INTERVAL_DOWN_S = 10;
static const uint32_t FREQUENCY_CHANGE_INTERVAL_UP_S = 5 * 60;
// Minimum compressor off time between cycles (anti short-cycling)
static const uint32_t COMPRESSOR_MIN_OFF_S = 5 * 60;
// Minimum compressor on time before it may stop
static const uint32_t COMPRESSOR_MIN_ON_S = 10 * 60;
// Duration of soft-start phase (no strict PID yet)
static const uint32_t COMPRESSOR_SOFT_START_DURATION_S = 3 * 60;
// Minimum time in seconds the pump has to be on before starting the compressor.
// Start compressor only after the pump has been running for this duration to allow Tc to stabilize.
static const uint32_t COMPRESSOR_MIN_TIME_PUMP_ON = 5 * 60;
// Working mode standby.
static const uint32_t WORKING_MODE_STANDBY = 0;
// Working mode cooling.
static const uint32_t WORKING_MODE_COOLING = 1;
// Working mode heating.
static const uint32_t WORKING_MODE_HEATING = 2;
/// Deadband on Tc to avoid too frequent changes around setpoint
static const float DEAD_BAND_DT = 1.0f;
// Settle time after a defrost before allowing compressor modulation and stop checks.
static const uint32_t COMPRESSOR_SETTLE_TIME_AFTER_DEFROST_S = 5 * 60;
// Lookahead time for backup heater to calculate when to stop based on degree/minute rate.
static const uint32_t BACKUP_HEATER_LOOKAHEAD_S = 2 * 60;
// Settle time after backup heater is turned off before compressor PID loop starts again.
static const uint32_t BACKUP_HEATER_OFF_SETTLE_TIME_S = 2 * 60;
// Time required to switch the 3-way valve position.
static const uint32_t THREE_WAY_VALVE_SWITCH_TIME_S = 1 * 60;

enum class HPState {
    UNKNOWN,
    IDLE,
    WAIT_PUMP_RUNNING,
    WAIT_PUMP_STOP,
    PUMP_INTERVAL_RUNNING,
    WAIT_COMPRESSOR_RUNNING,
    HEAT_COOL_COMPRESSOR_SOFTSTART,
    COMPRESSOR_RUNNING,
    WAIT_BACKUP_HEATER_RUNNING,
    BACKUP_HEATER_RUNNING,
    DEFROSTING,
    WAIT_FOR_STATE_SWITCH,
    SWITCH_TO_DHW,
    WAIT_COMPRESSOR_STOP,
    WAIT_3WAY_VALVE_SWITCH,
};

enum ThreeWayValvePosition {
    HEATING_COOLING,
    DHW,
};

struct AmberState {
  uint32_t last_compressor_start_ms = 0;
  uint32_t last_compressor_stop_ms = 0;
  uint32_t last_compressor_mode_change_ms = 0;
  float compressor_pid = 0.0;
  uint32_t next_pump_cycle = 0;
  uint32_t pump_start_time = 0;

  // Backup heater degree / minute tracking.
  uint32_t backup_degmin_last_ms = 0;
  float accumulated_backup_degmin = 0.0;
  float temperature_rate_c_per_min = 0.0;
  float last_temperature_for_rate = 0.0;

  HPState hp_state = HPState::IDLE;
  HPState deferred_hp_state = HPState::UNKNOWN;
  uint32_t defer_state_change_until_ms = 0;
  bool is_switching_modes = false;
};

class OpenAmberController {
 public:
  AmberState state;
  select::Select *compressor_control = nullptr;
  select::Select *working_mode = nullptr;
  sensor::Sensor *temp_tc = nullptr;
  sensor::Sensor *temp_tui = nullptr;
  sensor::Sensor *temp_tuo = nullptr;
  sensor::Sensor *temp_outside = nullptr;
  sensor::Sensor *room_temp = nullptr;
  sensor::Sensor *dhw_temperature_tw = nullptr;
  number::Number *pump_interval_min = nullptr;
  number::Number *pump_duration_min = nullptr;
  sensor::Sensor *compressor_current_frequency = nullptr;
  pid::PIDClimate *pid_heat_cool = nullptr;
  climate::Climate *thermostat_climate = nullptr;
  number::Number *pump_control = nullptr;
  binary_sensor::BinarySensor *pump_active = nullptr;
  binary_sensor::BinarySensor *frost_protection_stage1 = nullptr;
  binary_sensor::BinarySensor *frost_protection_stage2 = nullptr;  
  number::Number *start_compressor_delta = nullptr;
  number::Number *stop_compressor_delta = nullptr;
  text_sensor::TextSensor *hp_state_text_sensor = nullptr;
  binary_sensor::BinarySensor *oil_return_cycle = nullptr;
  number::Number *pump_speed_heating = nullptr;
  number::Number *pump_speed_dhw = nullptr;
  binary_sensor::BinarySensor *heat_demand = nullptr;
  binary_sensor::BinarySensor *cool_demand = nullptr;
  binary_sensor::BinarySensor *defrost_active = nullptr;
  switch_::Switch *backup_heater = nullptr;
  binary_sensor::BinarySensor *backup_heater_active = nullptr;
  number::Number *backup_heater_degmin_limit = nullptr;
  sensor::Sensor *backup_heater_degmin_current = nullptr;
  switch_::Switch *pump_p0_relay = nullptr;
  select::Select *heat_compressor_max_mode = nullptr;
  sensor::Sensor *pump_p0_current_pwm = nullptr;
  number::Number *defrost_backup_heater_boost_temperature = nullptr;

  select::Select *three_way_valve_position = nullptr;
  switch_::Switch *three_way_valve_dhw = nullptr;
  number::Number *dhw_setpoint = nullptr;
  binary_sensor::BinarySensor *dhw_demand = nullptr;
  select::Select *dhw_compressor = nullptr;
  select::Select *dhw_compressor_max = nullptr;
  number::Number *dhw_ta_temperature_compressor_max = nullptr;
  switch_::Switch *dhw_enabled = nullptr;
  switch_::Switch *dhw_pump = nullptr;  
  int mode_offset = 0;
  int dhw_mode_offset = 0;
  int dhw_mode_max_offset = 0;

  void loop() {
    if(this->initialized)
    {
      DoSafetyChecks();

      UpdateStateMachine();
    }
    else
    {
      Initialize();
    }
  }

  void WriteHeatPIDValue(float pid_value) {
    ESP_LOGD("amber", "Writing PID value: %.2f", pid_value);
    state.compressor_pid = pid_value;
  }
  
  void ResetPumpInterval()
  {
	  state.next_pump_cycle = 0;
  }

 private:
  bool initialized = false;

  void Initialize()
  {
    this->compressor_control = &id(compressor_control_select);
    this->working_mode = &id(working_mode_switch);

    this->temp_tc = &id(heat_cool_temperature_tc);
    this->temp_tui = &id(inlet_temperature_tui);
    this->temp_tuo = &id(outlet_temperature_tuo);
    this->temp_outside = &id(temperature_outside_ta);
    this->room_temp = &id(room_temperature);

    this->pump_interval_min = &id(pump_interval);
    this->pump_duration_min = &id(pump_duration);

    this->compressor_current_frequency = &id(current_compressor_frequency);
    this->pid_heat_cool = &id(pid_heat_cool_temperature_control);
    this->dhw_temperature_tw = &id(dhw_temperature_tw_sensor);
    this->thermostat_climate = &id(climate_controller);
    this->heat_demand = &id(heat_demand_active_sensor);
    this->cool_demand = &id(cool_demand_active_sensor);

    this->pump_control = &id(pump_control_pwm_number);
    this->pump_speed_heating = &id(pump_speed_heating_number);
    this->pump_speed_dhw = &id(pump_speed_dhw_number);
    this->pump_active = &id(internal_pump_active);
    this->frost_protection_stage1 = &id(frost_protection_stage1_active);
    this->frost_protection_stage2 = &id(frost_protection_stage2_active);
	
	  this->start_compressor_delta = &id(compressor_start_delta);
    this->stop_compressor_delta = &id(compressor_stop_delta);

    this->hp_state_text_sensor = &id(state_machine_state);
    this->oil_return_cycle = &id(oil_return_cycle_active);
    this->defrost_active = &id(defrost_active_sensor);

    this->backup_heater = &id(backup_heater_relay);
    this->backup_heater_degmin_limit = &id(backup_heater_degmin_threshold);
    this->backup_heater_active = &id(backup_heater_active_sensor);
    this->backup_heater_degmin_current = &id(backup_heater_degmin_current_sensor);
    this->pump_p0_relay = &id(pump_p0_relay_switch);
    this->heat_compressor_max_mode = &id(heat_compressor_mode);
    this->pump_p0_current_pwm = &id(pump_p0_current_pwm_sensor);

    this->defrost_backup_heater_boost_temperature = &id(defrost_backup_heater_boost_temperature_sensor);

    this->three_way_valve_position = &id(three_way_valve_select);
    this->dhw_demand = &id(dhw_demand_active_sensor);
    this->dhw_setpoint = &id(dhw_setpoint_temperature);
    this->dhw_compressor = &id(dhw_compressor_mode);
    this->dhw_compressor_max = &id(dhw_compressor_mode_max);
    this->dhw_ta_temperature_compressor_max = &id(dhw_temperature_threshold_max_compressor_mode);
    this->dhw_enabled = &id(dhw_enabled_switch);
    this->dhw_pump = &id(dhw_pump_relay_switch);
    this->three_way_valve_dhw = &id(three_way_valve_dhw_switch);

    this->mode_offset = this->compressor_control->size() - this->heat_compressor_max_mode->size();
    this->dhw_mode_offset = this->compressor_control->size() - this->dhw_compressor->size();
    this->dhw_mode_max_offset = this->compressor_control->size() - this->dhw_compressor_max->size();
    
    // Initialize relays
    id(initialize_relay_switch).turn_on();
    this->pump_p0_relay->turn_on();

    // Restore 3-way valve state based on relay position.
    if (this->dhw_enabled->state)
    {
      bool is_valve_in_dhw_position = this->three_way_valve_dhw->state;
      SetThreeWayValve(is_valve_in_dhw_position ? ThreeWayValvePosition::DHW : ThreeWayValvePosition::HEATING_COOLING);
    }
    else 
    {
      SetThreeWayValve(ThreeWayValvePosition::HEATING_COOLING);
    }

    // Restore state whenever initialized while compressor is running.
    if(this->compressor_current_frequency->state > 0) {
      state.last_compressor_start_ms = millis();
      SetNextState(HPState::COMPRESSOR_RUNNING);
    }
    else if(this->pump_active->state)
    {
      state.pump_start_time = millis();
      SetNextState(HPState::PUMP_INTERVAL_RUNNING);
    }
    else if (this->backup_heater_active->state)
    {
      SetNextState(HPState::BACKUP_HEATER_RUNNING);
    }
    else
    {
      // Initalize pump to Off, it starts on 950 mode.
      StopPump();
      SetNextState(HPState::IDLE);
    }
    ESP_LOGI("amber", "OpenAmberController initialized");
    this->initialized = true;
  }

  void DoSafetyChecks()
  {
    // If pump is not active while compressor is running, stop compressor to avoid damage.
    if(this->compressor_current_frequency->state > 0 && !this->pump_active->state)
    {
      ESP_LOGW("amber", "Safety check: Pump is not active while compressor is running, stopping compressor to avoid damage.");
      StopCompressor(false);
      SetNextState(HPState::IDLE);
      return;
    }

    // If Tuo - Tui is above 8 degrees while compressor is running, stop compressor to avoid damage.
    if (this->temp_tuo->state - this->temp_tui->state > 8.0f && this->compressor_current_frequency->state > 0)
    {
      ESP_LOGW("amber", "Safety check: Temperature difference between Tuo and Tui is above 15 degrees while compressor is running, stopping compressor to avoid damage.");
      StopCompressor(false);
      SetNextState(HPState::IDLE);
      return;
    }
  }

  void UpdateStateMachine()
  {
      const uint32_t now = millis();

       // TODO: This probably needs more logic around priorities and heating time on or post defrost.
      bool dhw_demand = this->dhw_demand->state && this->dhw_enabled->state;

      float current_temperature = GetCurrentTemperature();
      float target_temperature = GetTargetTemperature();
      float supply_temperature_delta = GetTemperatureDelta();

      bool frost_protection_stage2_active = frost_protection_stage2->state;
      bool frost_protection_stage1_active = frost_protection_stage1->state;

      bool heating_or_cooling_demand = heat_demand->state || cool_demand->state;

      bool pump_demand = dhw_demand || heating_or_cooling_demand || frost_protection_stage1_active || frost_protection_stage2_active;
      bool pump_running = pump_active->state;
      bool compressor_running = compressor_current_frequency->state > 0;

      const uint32_t min_on_ms = COMPRESSOR_MIN_ON_S  * 1000UL;

      switch (state.hp_state)
      {
        case HPState::WAIT_FOR_STATE_SWITCH:
        {
          if(state.defer_state_change_until_ms > now)
          {
            ESP_LOGD("amber", "Waiting for state switch, transitioning to next state in %lu ms", now - state.defer_state_change_until_ms);
            break;
          }
          else
          {
            state.defer_state_change_until_ms = 0;
            SetNextState(state.deferred_hp_state);
            state.deferred_hp_state = HPState::UNKNOWN;
            break;
          }
        }
        case HPState::IDLE:
        {
            if (!pump_demand) break;

            if (dhw_demand && !IsThreeWayValveInPosition(ThreeWayValvePosition::DHW))
            {
              SetThreeWayValve(ThreeWayValvePosition::DHW);
              state.is_switching_modes = true;
              LeaveStateAndSetNextStateAfterWaitTime(HPState::IDLE, THREE_WAY_VALVE_SWITCH_TIME_S * 1000UL);
              break;
            }
            else if (!dhw_demand && !IsThreeWayValveInPosition(ThreeWayValvePosition::HEATING_COOLING))
            {
			  SetThreeWayValve(ThreeWayValvePosition::HEATING_COOLING);
              state.is_switching_modes = true;
              LeaveStateAndSetNextStateAfterWaitTime(HPState::IDLE, THREE_WAY_VALVE_SWITCH_TIME_S * 1000UL);
              break;   
            }

            // If interval is elapsed or there is compressor demand, start pump immediately.
            if (now >= state.next_pump_cycle || ShouldStartCompressor())
            {
                StartPump();
                SetNextState(HPState::WAIT_PUMP_RUNNING);
            }
            break;
        }
        case HPState::WAIT_PUMP_RUNNING:
        {
          if(pump_running)
          {
            SetNextState(HPState::PUMP_INTERVAL_RUNNING);
          }
          // TODO: Implement timeout for when pump does not start.
          break;
        }
        case HPState::PUMP_INTERVAL_RUNNING:
        {
          bool should_start_compressor = ShouldStartCompressor();
          ApplyPumpSpeedChangeIfNeeded();

          // Stop pump when duration expired.
          uint32_t duration_ms = (uint32_t) pump_duration_min->state * 60000UL;  
          if (now >= state.pump_start_time + duration_ms && !should_start_compressor) {
            ESP_LOGI("amber", "Stopping pump (interval cycle finished)");
            StopPump();
            uint32_t interval_ms = (uint32_t) pump_interval_min->state * 60000UL;
            state.next_pump_cycle = now + interval_ms;
            SetNextState(HPState::WAIT_PUMP_STOP);
            break;
          }

          if(!should_start_compressor) {
            break;
          }

          bool pump_on_long_enough = millis() - state.pump_start_time >= (COMPRESSOR_MIN_TIME_PUMP_ON * 1000UL);
          if(!pump_on_long_enough && !IsInDhwMode()) {
            ESP_LOGI("amber", "Not starting compressor because Tc needs to stabilize (pump on time too short)");
            break;
          }

          // After mode switch, always start when there is demand.
          const uint32_t min_off_ms = COMPRESSOR_MIN_OFF_S * 1000UL;
          if (millis() < state.last_compressor_stop_ms + min_off_ms && !state.is_switching_modes) {
            ESP_LOGI("amber", "Not starting compressor because minimum compressor time off is not reached.");
            break;
          }

          int workingMode = IsInDhwMode() ? WORKING_MODE_HEATING :
                            this->heat_demand->state ? WORKING_MODE_HEATING : WORKING_MODE_COOLING;

          SetWorkingMode(workingMode);
          StartCompressor();
          SetNextState(HPState::WAIT_COMPRESSOR_RUNNING);
          break;
        }
        case HPState::WAIT_PUMP_STOP:
        {
          if (!pump_running)
          {
            if(IsInDhwMode())
            {
              // If we are in DHW mode, always switch back to heating/cooling after pump is stopped.
              SetThreeWayValve(ThreeWayValvePosition::HEATING_COOLING);
            }

            SetNextState(HPState::IDLE);
          }
          else {
            // TODO: Implement timeout for when pump does not stop.
          }
          break;
        }
        case HPState::WAIT_COMPRESSOR_RUNNING:
        {
          if (compressor_running)
          {
            state.last_compressor_start_ms = now;
            SetNextState(IsInDhwMode() ? HPState::COMPRESSOR_RUNNING : HPState::HEAT_COOL_COMPRESSOR_SOFTSTART);
          }
          // TODO: Implement timeout for when compressor does not start.
          break;
        }
        case HPState::HEAT_COOL_COMPRESSOR_SOFTSTART:
        {
          if (now - state.last_compressor_start_ms >= COMPRESSOR_SOFT_START_DURATION_S * 1000UL)
          {
            state.backup_degmin_last_ms = now;
            SetNextState(HPState::COMPRESSOR_RUNNING);
          }

          break;
        }
        case HPState::COMPRESSOR_RUNNING:
        {
          ApplyPumpSpeedChangeIfNeeded();

          if (this->defrost_active->state) {
            SetNextState(HPState::DEFROSTING);
            break;
          }

          CalculateAccumulatedBackupHeaterDegreeMinutes();

          if(state.accumulated_backup_degmin >= backup_heater_degmin_limit->state) {
            ESP_LOGI("amber", "Enabling backup heater (degree/min limit reached: %.2f)", state.accumulated_backup_degmin);
            backup_heater->turn_on();
            SetNextState(HPState::WAIT_BACKUP_HEATER_RUNNING);
            break;
          }

          bool hasPassedMinOnTime = (now - state.last_compressor_start_ms) > min_on_ms;

          // Only check for stop conditions when min on time is passed.
          if (hasPassedMinOnTime)
          {
            if (!IsCompressorDemandForCurrentMode()) {
              ESP_LOGI("amber", "Stopping compressor because there is no demand.");
              StopCompressor();
              SetNextState(HPState::WAIT_COMPRESSOR_STOP);
              break;
            }

            if (dhw_demand && !IsInDhwMode())
            {
                ESP_LOGI("amber", "Stopping compressor because there is demand for DHW.");
                StopCompressor();
                SetNextState(HPState::WAIT_COMPRESSOR_STOP);
                break;
            }

            // Stop when we overshoot above target + delta.
            if (supply_temperature_delta >= stop_compressor_delta->state && !IsInDhwMode()) {
              if(this->oil_return_cycle->state)
              {
                // Every 2 hours of compressor running in low frequency it will do an oil return cycle, this will mess up the delta T based stopping.
                // TODO: Improve this if needed by adding a timer do make sure Tc is stable before checking the delta again.
                ESP_LOGI("amber", "Not stopping compressor because oil return cycle is active.");
                break;
              }

              ESP_LOGI("amber", "Stopping compressor because it reached delta %.2f°C (ΔT=%.2f°C).", stop_compressor_delta->state, supply_temperature_delta);
              StopCompressor();
              SetNextState(HPState::WAIT_COMPRESSOR_STOP);
              break;
            }
          }

          if(IsInDhwMode())
          {
            int dhw_compressor_mode = GetDhwCompressorMode();
            auto current_compressor_mode = this->compressor_control->active_index().value();
            if(current_compressor_mode != dhw_compressor_mode)
            {
              ESP_LOGI("amber", "Applying compressor mode change for DHW to %d", dhw_compressor_mode);
              SetCompressorMode(dhw_compressor_mode);
			}
          }
          else 
          {
            ManageCompressorModulation();
          }
          break;
        }
        case HPState::WAIT_COMPRESSOR_STOP:
        {
          if (compressor_current_frequency->state == 0)
          {
            SetNextState(HPState::PUMP_INTERVAL_RUNNING);
          }
          else 
          {
            // TODO: Implement timeout for when compressor does not stop.
          }
          break;
        }
        case HPState::WAIT_BACKUP_HEATER_RUNNING:
        {
          if (this->backup_heater_active->state) {
            SetNextState(HPState::BACKUP_HEATER_RUNNING);
          }
          // TODO: Implement timeout for when backup heater does not start.
          break;
        }
        case HPState::BACKUP_HEATER_RUNNING:
        {
          float predicted_temperature = CalculateBackupHeaterPredictedTemperature();
          // Disable backup heater if predicted Temperature is above target.
          if(predicted_temperature >= target_temperature + 2.0f) { // Margin to buffer some heat to reduce undershoot.
            ESP_LOGI("amber", "Disabling backup heater (predicted Temperature %.2f°C above target %.2f°C)", predicted_temperature, target_temperature);
            backup_heater->turn_off();
            LeaveStateAndSetNextStateAfterWaitTime(HPState::COMPRESSOR_RUNNING, BACKUP_HEATER_OFF_SETTLE_TIME_S * 1000UL);
            return;
          }
          break;
        }
        case HPState::DEFROSTING:
        {
          if(this->defrost_active->state)
          {
            ESP_LOGI("amber", "Defrost busy, waiting before making changes to compressor.");
            break;
          }

          // Defrost is no longer active

          // No need to boost in DHW mode.
          if(IsInDhwMode())
          {
            LeaveStateAndSetNextStateAfterWaitTime(HPState::COMPRESSOR_RUNNING, COMPRESSOR_SETTLE_TIME_AFTER_DEFROST_S * 1000UL);
            break;
          }

          if(this->temp_outside->state <= this->defrost_backup_heater_boost_temperature->state) {
            ESP_LOGI("amber", "Defrost ended, enabling backup heater as configured for outside temperature %.2f°C to boost temperature back to setpoint.", this->temp_outside->state);
            backup_heater->turn_on();
            SetNextState(HPState::WAIT_BACKUP_HEATER_RUNNING);
          }
          else {
            auto current_compressor_mode = this->compressor_control->active_index().value();
            // Increase compressor mode by 3 steps to speed up recovery after defrost.
            int defrost_recovery_mode = current_compressor_mode + 3; // Max step will be limited in SetCompressorMode.
            SetCompressorMode(defrost_recovery_mode);
            LeaveStateAndSetNextStateAfterWaitTime(HPState::COMPRESSOR_RUNNING, COMPRESSOR_SETTLE_TIME_AFTER_DEFROST_S * 1000UL);
          }
          pid_heat_cool->reset_integral_term();
        }
      }
  }

  bool IsInDhwMode()
  {
    return IsThreeWayValveInPosition(ThreeWayValvePosition::DHW);
  }

  bool IsThreeWayValveInPosition(ThreeWayValvePosition position)
  {
    return this->three_way_valve_position->active_index() == (int)position;
  }

  void SetThreeWayValve(ThreeWayValvePosition position)
  {
      auto three_way_valve_call = this->three_way_valve_position->make_call();
      three_way_valve_call.set_index((int)position);
      three_way_valve_call.perform();
      ESP_LOGI("amber", "Setting 3-way valve to %s.", this->three_way_valve_position->current_option());
  }

  float GetCurrentPumpSpeedSetting()
  {
    return (IsInDhwMode() ? this->pump_speed_dhw : this->pump_speed_heating)->state;
  }

  void ApplyPumpSpeedChangeIfNeeded()
  {
    if(!this->pump_active->state)
    {
      return;
    }

    SetPumpPwmDutyCycle(GetCurrentPumpSpeedSetting());
  }

  /// @brief Calculate and update the accumulated backup heater degree/minutes.
  /// Will not accumulate if not at max compressor mode.
  void CalculateAccumulatedBackupHeaterDegreeMinutes()
  {
    float tc = temp_tc->state;
    float target = pid_heat_cool->target_temperature;
    const float diff = std::max(0.0f, target - tc);
    uint32_t dt_ms = millis() - state.backup_degmin_last_ms;
    state.backup_degmin_last_ms = millis();
    const float dt_min = (float)dt_ms / 60000.0f;
    if (dt_min <= 0.0f)
    {
      ESP_LOGW("amber", "Delta time for backup heater degree/min calculation is zero or negative, skipping update.");
      return;
    }
    state.accumulated_backup_degmin += diff * dt_min;

    int max_compressor_mode = this->heat_compressor_max_mode->active_index().value() + this->mode_offset - 1;
    bool is_at_max_mode = this->compressor_control->active_index().value() >= max_compressor_mode;

    // Reset accumulated degmin if there is no difference or we are not at max compressor mode.
    if (diff <= 0.0f || !is_at_max_mode)
    {
      state.accumulated_backup_degmin = 0.0f;
    }
    this->backup_heater_degmin_current->publish_state(state.accumulated_backup_degmin);
    ESP_LOGD("amber", "Backup heater degree/minutes updated: +%.2f -> %.2f", diff * dt_min, state.accumulated_backup_degmin);
  }

  float CalculateBackupHeaterPredictedTemperature()
  {
    float current_temperature = GetCurrentTemperature();
    float target = GetTargetTemperature();

    uint32_t dt_ms = millis() - state.backup_degmin_last_ms;
    const float dt_min = (float)dt_ms / 60000.0f;
    if (dt_min <= 0.0f)
    {
      ESP_LOGW("amber", "Delta time for backup heater degree/min rate calculation is zero or negative, skipping update.");
      return 0.0f;
    }
    // Calculate degree/minute rate.
    float rate = (current_temperature - state.last_temperature_for_rate) / dt_min;
    // Low-pass filter the rate to avoid spikes.
    double a = 0.2f;
    state.temperature_rate_c_per_min = (1-a) * state.temperature_rate_c_per_min + a * rate;
    state.last_temperature_for_rate = current_temperature;
    // Predict future Tc based on current rate.
    float predicted_temperature = current_temperature + state.temperature_rate_c_per_min * ((float)BACKUP_HEATER_LOOKAHEAD_S / 60.0f);
    state.backup_degmin_last_ms = millis();
    ESP_LOGI("amber", "Backup heater temperature rate/min updated: %.2f (Predicted Temperature: %.2fÂ°C)", state.temperature_rate_c_per_min, predicted_temperature);
    return predicted_temperature;
  }

  bool ShouldStartCompressor()
  {
    if(IsInDhwMode())
    {
      return this->dhw_demand->state;
    }

    float current_temperature = GetCurrentTemperature();
    float target_temperature = GetTargetTemperature();
  
    if (!IsCompressorDemandForCurrentMode()) {
      ESP_LOGI("amber", "Not starting compressor because there is no demand.");
      return false;
    }

    // Start condition based on target temperature and start_compressor_delta.
    float start_temperature = target_temperature - start_compressor_delta->state;
    if (current_temperature >= start_temperature && !frost_protection_stage2->state) {
      ESP_LOGI("amber", "Not starting compressor because target temperature (%.2f) is higher than the start temperature(%.2f)", current_temperature, start_temperature);
      return false;
    }

    return true;
  }

  void ManageCompressorModulation() {
    const uint32_t now = millis();
    int desired_compressor_mode = MapPIDToCompressorMode(state.compressor_pid);
    auto current_compressor_mode = this->compressor_control->active_index().value();
    const uint32_t min_interval = (current_compressor_mode > desired_compressor_mode ? FREQUENCY_CHANGE_INTERVAL_DOWN_S : FREQUENCY_CHANGE_INTERVAL_UP_S) * 1000;
    if ((now - state.last_compressor_mode_change_ms) < min_interval) {
      ESP_LOGI("amber", "Frequency change skipped (interval not elapsed)");
      return;
    }

    // Deadband on Tc to avoid too frequent changes around setpoint
    float current_temperature = GetCurrentTemperature();
    float target = GetTargetTemperature();
    float dt = GetTemperatureDelta();

    if (fabsf(dt) < DEAD_BAND_DT) {
      ESP_LOGD("amber", "ΔT=%.2f°C within deadband, compressor mode remains at %d",
              dt, current_compressor_mode);
      return;
    }

    // If Tc is above setpoint, then never modulate up.
    const float TEMP_MARGIN = 0.3f; // Minor margin to reduce noise
    if (current_temperature > (target + TEMP_MARGIN) && desired_compressor_mode > current_compressor_mode) {
        ESP_LOGW("amber", "Temperature is above target and PID controller wanted to move to a higher compressor mode.");
        return;
    }
    
    ESP_LOGI("amber", "PID %.2f -> mode %d (previous mode %d)", state.compressor_pid, desired_compressor_mode, current_compressor_mode);

    if (desired_compressor_mode != current_compressor_mode) {
      SetCompressorMode(desired_compressor_mode);
      ESP_LOGI("amber", "Compressor mode updated to %d", desired_compressor_mode);
    }
  }

  void StopCompressor(bool run_pump_interval = true)
  {
    // Let pump run for another interval cycle seconds after stopping the compressor.
    state.pump_start_time = millis();

    state.last_compressor_start_ms = 0;
    state.last_compressor_stop_ms = millis();
    auto compressor_set_call = compressor_control->make_call();
    compressor_set_call.select_first();
    compressor_set_call.perform();

    SetWorkingMode(WORKING_MODE_STANDBY);

    if (run_pump_interval)
    {
      // Let pump run for another 60 seconds.
      state.next_pump_cycle = millis() + (uint32_t)pump_interval_min->state * 60000UL;
    }
  }

  void StartCompressor() {
    state.is_switching_modes = false;
    if(IsInDhwMode())
    { 
      int dhw_compressor_mode = GetDhwCompressorMode();
      ESP_LOGI("amber", "Starting compressor in DHW mode, setting compressor to mode %d", dhw_compressor_mode);
      SetCompressorMode(dhw_compressor_mode);
    }
    else
    {
      float supply_temperature_delta = GetTemperatureDelta();
      ESP_LOGI("amber", "Starting compressor (ΔT=%.2f°C)", supply_temperature_delta);
      int compressor_frequency_mode = DetermineSoftStartMode(supply_temperature_delta);
      ESP_LOGI("amber", "Soft start mode: %d (ΔT=%.1f°C)", compressor_frequency_mode, supply_temperature_delta);
      pid_heat_cool->reset_integral_term();
      SetCompressorMode(compressor_frequency_mode);
    }
  }

  int GetDhwCompressorMode()
  {
    float temperature_ta = temp_outside->state;
    float ta_threshold = dhw_ta_temperature_compressor_max->state;
    int dhw_compressor_mode = this->dhw_compressor->active_index().value() + this->dhw_mode_offset;
    if(temperature_ta <= ta_threshold)
    {
        dhw_compressor_mode = this->dhw_compressor_max->active_index().value() + this->dhw_mode_max_offset;
    }
    return dhw_compressor_mode;
  }

  /// @brief Determine the compressor mode at which to start the compressor.
  /// In this soft start period the PID loop can act on karakteristics of the measured supply temperature(Tc).
  /// @param supply_temperature_delta Temperature delta between Tc and target temperature.
  /// @return Compressor mode.
  int DetermineSoftStartMode(float supply_temperature_delta) {
    int start_compressor_frequency_mode;
    float delta = fabs(supply_temperature_delta);
    if (delta < 5.0f) start_compressor_frequency_mode = 1;
    else if (delta < 7.0f) start_compressor_frequency_mode = 3;
    else if (delta < 10.0f) start_compressor_frequency_mode = 4;
    else start_compressor_frequency_mode = 5;

    return start_compressor_frequency_mode;
  }

  /// @brief Map the PID output value to a discrete compressor mode.
  /// Limits the mode change to a single step per update.
  int MapPIDToCompressorMode(float pid) {
    float raw = fabsf(pid);
    if (raw > 1.0f) raw = 1.0f;

    int amount_of_modes = this->heat_compressor_max_mode->active_index().value() + this->mode_offset; 
    int desired_mode = (int) roundf(raw * (amount_of_modes - 1)) + 1;
    desired_mode = std::max(1, std::min(desired_mode, (int)amount_of_modes));

    auto current_mode = this->compressor_control->active_index().value();
    // Limit to a single frequency step per update
    int new_mode = desired_mode;
    if (desired_mode > current_mode + 1) new_mode = current_mode + 1;
    if (desired_mode < current_mode - 1) new_mode = current_mode - 1;

    return new_mode;
  }

  void SetCompressorMode(int mode_index)
  {
    int capped_mode_index = std::min(mode_index, (int)this->heat_compressor_max_mode->active_index().value() + this->mode_offset - 1);
    auto compressor_set_call = compressor_control->make_call();
    compressor_set_call.set_index(capped_mode_index);
    compressor_set_call.perform();
    state.last_compressor_mode_change_ms = millis();
  }

  void StartPump() {
    ESP_LOGI("amber", "Starting pump (interval cycle)");
    if(!this->pump_p0_relay->state) {
      ESP_LOGW("amber", "Pump P0 relay is not active, activating it now.");
      this->pump_p0_relay->turn_on();
    }

    SetPumpPwmDutyCycle(GetCurrentPumpSpeedSetting());

    if(IsInDhwMode()) {
      ESP_LOGI("amber", "Starting DHW pump.");
      this->dhw_pump->turn_on();
    }

    state.pump_start_time = millis();
  }

  void StopPump()
  {
    if(IsInDhwMode()) {
      ESP_LOGI("amber", "Stopping DHW pump.");
      this->dhw_pump->turn_off();
    }

    SetPumpPwmDutyCycle(0);
  }

  void SetWorkingMode(int workingMode)
  {
    auto working_mode_call = working_mode->make_call();
    working_mode_call.set_index(workingMode);
    working_mode_call.perform();
  }

  float GetTargetTemperature()
  {
    bool is_dhw_mode = IsThreeWayValveInPosition(ThreeWayValvePosition::DHW);
    return is_dhw_mode ? this->dhw_setpoint->state : this->pid_heat_cool->target_temperature;
  }

  float GetCurrentTemperature()
  {
    bool is_dhw_mode = IsThreeWayValveInPosition(ThreeWayValvePosition::DHW);
    return is_dhw_mode ? this->dhw_temperature_tw->state : this->temp_tc->state;
  }

  float GetTemperatureDelta()
  {
    return GetCurrentTemperature() - GetTargetTemperature();
  }

  bool IsCompressorDemandForCurrentMode()
  {
    if(IsInDhwMode())
    {
        return this->dhw_demand->state;
    }
    else
    {
        return this->heat_demand->state || this->cool_demand->state || frost_protection_stage2->state;
    }
  }

  void LeaveStateAndSetNextStateAfterWaitTime(HPState new_state, uint32_t defer_ms)
  {
    state.deferred_hp_state = new_state;
    state.defer_state_change_until_ms = millis() + defer_ms;
    SetNextState(HPState::WAIT_FOR_STATE_SWITCH);
  }

  void SetPumpPwmDutyCycle(float duty_cycle)
  {
    float control_speed = ((duty_cycle*10)*-1)+1000;
    if(this->pump_p0_current_pwm->raw_state != control_speed)
    {
      ESP_LOGI("amber", "Applying pump speed change to %.0f (Duty cycle: %.0f)", control_speed, duty_cycle);
      auto pump_call = this->pump_control->make_call();
      pump_call.set_value(control_speed);
      pump_call.perform();
    }
  }

  void SetNextState(HPState new_state)
  {
    const char *txt = "Unknown";
    state.hp_state = new_state;

    switch (new_state) {
      case HPState::IDLE:
        txt = "Idle";
        break;
      case HPState::PUMP_INTERVAL_RUNNING:
        txt = "Pump interval running";
        break;
      case HPState::WAIT_PUMP_RUNNING:
        txt = "Wait for pump running";
        break;
      case HPState::WAIT_COMPRESSOR_RUNNING:
        txt = "Wait for compressor running";
        break;
      case HPState::HEAT_COOL_COMPRESSOR_SOFTSTART:
        txt = "Compressor soft start";
        break;
      case HPState::COMPRESSOR_RUNNING:
        txt = "Compressor running";
        break;
      case HPState::WAIT_COMPRESSOR_STOP:
        txt = "Waiting for compressor to stop";
        break;
      case HPState::WAIT_BACKUP_HEATER_RUNNING:
        txt = "Wait for backup heater running";
        break;
      case HPState::BACKUP_HEATER_RUNNING:
        txt = "Backup heater running";
        break;
      case HPState::DEFROSTING:
        txt = "Defrosting";
        break;
      case HPState::WAIT_FOR_STATE_SWITCH:
        txt = "Waiting for wait time to elapse";
        break;
      case HPState::WAIT_PUMP_STOP:
        txt = "Waiting for pump to stop";
        break;
    }

    hp_state_text_sensor->publish_state(txt);
    ESP_LOGI("amber", "HP state changed: %s", txt);
  }
};

inline OpenAmberController AMBER;
