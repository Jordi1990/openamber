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

enum class HPState {
    IDLE,
    WAIT_PUMP_RUNNING,
    PUMP_INTERVAL_RUNNING,
    WAIT_COMPRESSOR_RUNNING,
    COMPRESSOR_SOFTSTART,
    COMPRESSOR_RUNNING,
    COMPRESSOR_STOP
};

struct AmberState {
  uint32_t last_compressor_start = 0;
  uint32_t last_compressor_stop = 0;
  uint32_t last_compressor_mode_change = 0;
  float compressor_pid = 0.0;
  uint32_t next_pump_cycle = 0;
  uint32_t pump_start_time = 0;

  HPState hp_state = HPState::IDLE;
};

class OpenAmberController {
 public:
  AmberState state;
  select::Select *compressor_control = nullptr;
  select::Select *working_mode = nullptr;
  sensor::Sensor *temp_tc = nullptr;
  sensor::Sensor *temp_tui = nullptr;
  sensor::Sensor *temp_outside = nullptr;
  sensor::Sensor *room_temp = nullptr;
  number::Number *pump_interval_min = nullptr;
  number::Number *pump_duration_min = nullptr;
  sensor::Sensor *compressor_current_frequency = nullptr;
  pid::PIDClimate *pid_climate = nullptr;
  climate::Climate *thermostat_climate = nullptr;
  binary_sensor::BinarySensor *external_heat_demand = nullptr;
  binary_sensor::BinarySensor *external_cool_demand = nullptr;
  select::Select *thermostat_mode = nullptr;
  select::Select *pump_control = nullptr;
  binary_sensor::BinarySensor *pump_active = nullptr;
  binary_sensor::BinarySensor *frost_protection_stage1 = nullptr;
  binary_sensor::BinarySensor *frost_protection_stage2 = nullptr;  
  number::Number *start_compressor_delta = nullptr;
  number::Number *stop_compressor_delta = nullptr;
  text_sensor::TextSensor *hp_state_text_sensor = nullptr;
  binary_sensor::BinarySensor *oil_return_cycle = nullptr;
  select::Select *pump_speed = nullptr;

  void loop() {
    if(this->initialized)
    {
      ApplyPumpSpeedChangeIfNeeded();
      UpdateStateMachine();
    }
    else
    {
      Initialize();
    }
  }

  void WritePIDValue(float pid_value) {
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
    this->temp_outside = &id(temperature_outside_ta);
    this->room_temp = &id(room_temperature);

    this->pump_interval_min = &id(pump_interval);
    this->pump_duration_min = &id(pump_duration);

    this->compressor_current_frequency = &id(current_compressor_frequency);
    this->pid_climate = &id(compressor_pid_climate);
    this->thermostat_climate = &id(climate_controller);
    this->thermostat_mode = &id(thermostat_mode_select);
    this->external_heat_demand = &id(external_heat_demand_sensor);
    this->external_cool_demand = &id(external_cool_demand_sensor);

    this->pump_control = &id(pump_control_select);
    this->pump_speed = &id(pump_speed_select);
    this->pump_active = &id(internal_pump_active);
    this->frost_protection_stage1 = &id(frost_protection_stage1_active);
    this->frost_protection_stage2 = &id(frost_protection_stage2_active);
	
	  this->start_compressor_delta = &id(compressor_start_delta);
    this->stop_compressor_delta = &id(compressor_stop_delta);

    this->hp_state_text_sensor = &id(state_machine_state);
    this->oil_return_cycle = &id(oil_return_cycle_active);

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
    else
    {
      // Initalize pump to Off, it starts on 950 mode.
      StopPump();
      SetNextState(HPState::IDLE);
    }
    ESP_LOGI("amber", "OpenAmberController initialized");
    this->initialized = true;
  }

  void UpdateStateMachine()
  {
      const uint32_t now = millis();

      float tc = temp_tc->state;
      float target_temperature = pid_climate->target_temperature;
      float supply_temperature_delta = tc - target_temperature;

      bool frost_protection_stage2_active = frost_protection_stage2->state;
      bool frost_protection_stage1_active = frost_protection_stage1->state;

      bool heating_or_cooling_demand = IsThermostatCooling() || IsThermostatHeating();

      bool pump_demand = heating_or_cooling_demand || frost_protection_stage1_active || frost_protection_stage2_active;
      bool compressor_demand = heating_or_cooling_demand || frost_protection_stage2_active;
      bool pump_running = pump_active->state;
      bool compressor_running = compressor_current_frequency->state > 0;

      const uint32_t min_on_ms  = COMPRESSOR_MIN_ON_S  * 1000UL;

      switch (state.hp_state)
      {
          case HPState::IDLE:
          {
              if (!pump_demand) break;

              if (now >= state.next_pump_cycle)
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
            bool should_start_compressor = ShouldStartCompressor(compressor_demand);

            // Stop pump when duration expired.
            uint32_t duration_ms = (uint32_t) pump_duration_min->state * 60000UL;  
            if (now >= state.pump_start_time + duration_ms && !should_start_compressor) {
              ESP_LOGI("amber", "Stopping pump (interval cycle finished)");
              StopPump();
              uint32_t interval_ms = (uint32_t) pump_interval_min->state * 60000UL;
              state.next_pump_cycle = now + interval_ms;
              SetNextState(HPState::IDLE);
              break;
            }

            if(!should_start_compressor) {
              break;
            }

            bool pump_on_long_enough = millis() - state.pump_start_time >= (COMPRESSOR_MIN_TIME_PUMP_ON * 1000UL);
            if(!pump_on_long_enough) {
              ESP_LOGI("amber", "Not starting compressor because Tc needs to stabilize (pump on time too short)");
              break;
            }

            SetWorkingModeFromClimate();
            StartCompressor(supply_temperature_delta);
            SetNextState(HPState::WAIT_COMPRESSOR_RUNNING);
            break;
          }
          case HPState::WAIT_COMPRESSOR_RUNNING:
          {
            if (compressor_running)
            {
              state.last_compressor_start = now;
              SetNextState(HPState::COMPRESSOR_SOFTSTART);
            }
            // TODO: Implement timeout for when compressor does not start.
            break;
          }
          case HPState::COMPRESSOR_SOFTSTART:
          {
            if (now - state.last_compressor_start >= COMPRESSOR_SOFT_START_DURATION_S * 1000UL)
            {
                SetNextState(HPState::COMPRESSOR_RUNNING);
            }

            break;
          }
          case HPState::COMPRESSOR_RUNNING:
          {
            bool hasPassedMinOnTime = (now - state.last_compressor_start) > min_on_ms;

            // Only check for stop conditions when min on time is passed.
            if (hasPassedMinOnTime)
            {
              if (!compressor_demand) {
                ESP_LOGI("amber", "Stopping compressor because there is no demand.");
                SetNextState(HPState::COMPRESSOR_STOP);
                break;
              }

              // Stop when we are close enough to target
              if (supply_temperature_delta >= stop_compressor_delta->state) {
                if(this->oil_return_cycle->state)
                {
                  // Every 2 hours of compressor running in low frequency it will do an oil return cycle, this will mess up the delta T based stopping.
                  // TODO: Improve this if needed by adding a timer do make sure Tc is stable before checking the delta again.
                  ESP_LOGI("amber", "Not stopping compressor because oil return cycle is active.");
                  break;
                }

                ESP_LOGI("amber", "Stopping compressor because it reached delta %.2f°C (ΔT=%.2f°C).", stop_compressor_delta->state, supply_temperature_delta);
                SetNextState(HPState::COMPRESSOR_STOP);
                break;
              }
            }

            ManageCompressorModulation();
            break;
          }
          case HPState::COMPRESSOR_STOP:
          {
              StopCompressor();
              SetWorkingMode(WORKING_MODE_STANDBY);
              // Let pump run for another 60 seconds.
              state.next_pump_cycle = now + (uint32_t)pump_interval_min->state * 60000UL;
              SetNextState(HPState::PUMP_INTERVAL_RUNNING);
              break;
          }
      }
  }

  void ApplyPumpSpeedChangeIfNeeded()
  {
    if(!this->pump_active->state)
    {
      return;
    }

    if(this->pump_control->current_option() != this->pump_speed->current_option())
    {
      ESP_LOGI("amber", "Applying pump speed change to %s", this->pump_speed->current_option());
      auto pump_call = pump_control->make_call();
      pump_call.set_option(this->pump_speed->current_option());
      pump_call.perform();
    }
  }

  bool ShouldStartCompressor(bool compressor_demand)
  {
    float tc = temp_tc->state;
    const uint32_t min_off_ms = COMPRESSOR_MIN_OFF_S * 1000UL;
  
    if (!compressor_demand) {
      ESP_LOGI("amber", "Not starting compressor because there is no demand.");
      return false;
    }

    // Start condition based on Tc and start_compressor_delta.
    float start_temperature = pid_climate->target_temperature - start_compressor_delta->state;
    if (tc >= start_temperature && !frost_protection_stage2->state) {
      ESP_LOGI("amber", "Not starting compressor because Tc(%.2f) is higher than the start temperature(%.2f)", tc, start_temperature);
      return false;
    }

    if (millis() - state.last_compressor_stop < min_off_ms) {
      ESP_LOGI("amber", "Not starting compressor because minimum compressor time off is not reached.");
      return false;
    }

    return true;
  }

  void ManageCompressorModulation() {
    const uint32_t now = millis();
    int desired_compressor_index = MapPIDToCompressorMode(state.compressor_pid);
    auto current_compressor_mode = this->compressor_control->active_index().value();
    const uint32_t min_interval = (current_compressor_mode > desired_compressor_index ? FREQUENCY_CHANGE_INTERVAL_DOWN_S : FREQUENCY_CHANGE_INTERVAL_UP_S) * 1000;
    if ((now - state.last_compressor_mode_change_ms) < min_interval) {
      ESP_LOGI("amber", "Frequency change skipped (interval not elapsed)");
      return;
    }

    // Deadband on Tc to avoid too frequent changes around setpoint
    float tc = temp_tc->state;
    float target = pid_climate->target_temperature;
    float dt = tc - target;

    if (fabsf(dt) < DEAD_BAND_DT) {
      ESP_LOGD("amber", "ΔT=%.2f°C within deadband, compressor mode remains at %d",
              dt, current_compressor_mode);
      return;
    }
    
    ESP_LOGI("amber", "PID %.2f -> mode %d (previous mode %d)", state.compressor_pid, desired_compressor_index, current_compressor_mode);

    if (desired_compressor_index != current_compressor_mode) {
      SetCompressorMode(desired_compressor_index);
      ESP_LOGI("amber", "Compressor mode updated to %d", desired_compressor_index);
    }
  }

  void StopCompressor()
  {
    // Let pump run for another interval cycle seconds after stopping the compressor.
    state.pump_start_time = millis();

    state.last_compressor_start = 0;
    state.last_compressor_stop = millis();
    auto compressor_set_call = compressor_control->make_call();
    compressor_set_call.select_first();
    compressor_set_call.perform();

    pid_climate->reset_integral_term();
  }

  void StartCompressor(float supply_temperature_delta) {
    ESP_LOGI("amber", "Starting compressor (ΔT=%.2f°C)", supply_temperature_delta);
    int compressor_frequency_mode = DetermineSoftStartMode(supply_temperature_delta);
    ESP_LOGI("amber", "Soft start mode: %d (ΔT=%.1f°C)", compressor_frequency_mode, supply_temperature_delta);

    pid_climate->reset_integral_term();
    SetCompressorMode(compressor_frequency_mode);
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

    auto amount_of_modes = this->compressor_control->size();
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
    auto compressor_set_call = compressor_control->make_call();
    compressor_set_call.set_index(mode_index);
    compressor_set_call.perform();
    state.last_compressor_mode_change_ms = millis();
  }

  void StartPump() {
    ESP_LOGI("amber", "Starting pump (interval cycle)");
    auto pump_call = pump_control->make_call();
    pump_call.set_option(this->pump_speed->current_option());
    pump_call.perform();
    state.pump_start_time = millis();
  }

  void StopPump()
  {
    auto pump_call = pump_control->make_call();
    pump_call.set_index(0);
    pump_call.perform();
  }

  bool IsThermostatHeating()
  {
    if (this->thermostat_mode->current_option() == "Intern")
    {
      climate::ClimateAction action = thermostat_climate->action;
      return action == climate::CLIMATE_ACTION_HEATING;
    }
    else
    {
      return this->external_heat_demand->state;
    }
  }

  bool IsThermostatCooling()
  {
    if (this->thermostat_mode->current_option() == "Intern")
    {
      climate::ClimateAction action = thermostat_climate->action;
      return action == climate::CLIMATE_ACTION_COOLING;
    }
    else
    {
      return this->external_cool_demand->state;
    }
  }

  void SetWorkingModeFromClimate()
  {
      if (IsThermostatHeating())
      {
          SetWorkingMode(WORKING_MODE_HEATING);
      }
      else if (IsThermostatCooling())
      {
          SetWorkingMode(WORKING_MODE_COOLING);
      }
  }

  void SetWorkingMode(int workingMode)
  {
    auto working_mode_call = working_mode->make_call();
    working_mode_call.set_index(workingMode);
    working_mode_call.perform();
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
        case HPState::COMPRESSOR_SOFTSTART:
          txt = "Compressor soft start";
          break;
        case HPState::COMPRESSOR_RUNNING:
          txt = "Compressor running";
          break;
        case HPState::COMPRESSOR_STOP:
          txt = "Compressor stopping";
          break;
      }

    hp_state_text_sensor->publish_state(txt);
    ESP_LOGI("amber", "HP state changed: %s", txt);
  }
};

inline OpenAmberController AMBER;
