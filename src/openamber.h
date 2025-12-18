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
static const float DEAD_BAND_DT = 0.5f;

// Supported discrete compressor frequencies
static const float SUPPORTED_FREQUENCIES[] = {36, 43, 49, 55, 61, 67, 72, 79, 85, 90};
static const int FREQUENCY_COUNT = sizeof(SUPPORTED_FREQUENCIES) / sizeof(SUPPORTED_FREQUENCIES[0]);

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
  uint32_t last_compressor_freq_change = 0;
  float compressor_pid = 0.0;
  int commanded_freq_index = -1;
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
  output::FloatOutput *pid_heat_output = nullptr;
  output::FloatOutput *pid_cool_output = nullptr;
  climate::Climate *thermostat_climate = nullptr;
  select::Select *pump_control = nullptr;
  binary_sensor::BinarySensor *pump_active = nullptr;
  binary_sensor::BinarySensor *frost_protection_stage1 = nullptr;
  binary_sensor::BinarySensor *frost_protection_stage2 = nullptr;  
  number::Number *start_compressor_delta = nullptr;
  number::Number *stop_compressor_delta = nullptr;
  text_sensor::TextSensor *hp_state_text_sensor = nullptr;
  binary_sensor::BinarySensor *oil_return_cycle_active = nullptr;

  void init(
      select::Select *working_mode,
      select::Select *compressor_control,
      sensor::Sensor *tc,
      sensor::Sensor *tui,
      sensor::Sensor *ta,
      sensor::Sensor *room,
      number::Number *pump_interval,
      number::Number *pump_duration,
      sensor::Sensor *compressor_current_frequency,
      pid::PIDClimate *pid,
      output::FloatOutput *pid_heat_output,
      output::FloatOutput *pid_cool_output,
      climate::Climate *thermo,
      select::Select *pump_control,
      binary_sensor::BinarySensor *pump_active,
      binary_sensor::BinarySensor *frost_protection_stage1,
      binary_sensor::BinarySensor *frost_protection_stage2,
      number::Number *start_compressor_delta,
      number::Number *stop_compressor_delta,
      text_sensor::TextSensor *hp_state_text_sensor,
      binary_sensor::BinarySensor *oil_return_cycle_active) {
    this->compressor_control = compressor_control;
    this->working_mode = working_mode;

    temp_tc = tc;
    temp_tui = tui;
    temp_outside = ta;
    room_temp = room;

    pump_interval_min = pump_interval;
    pump_duration_min = pump_duration;

    this->compressor_current_frequency = compressor_current_frequency;
    pid_climate = pid;
    this->pid_heat_output = pid_heat_output;
    this->pid_cool_output = pid_cool_output;
    thermostat_climate = thermo;

    this->pump_control = pump_control;
    this->pump_active = pump_active;
    this->frost_protection_stage1 = frost_protection_stage1;
    this->frost_protection_stage2 = frost_protection_stage2;
	
	this->start_compressor_delta = start_compressor_delta;
    this->stop_compressor_delta = stop_compressor_delta;

    this->hp_state_text_sensor = hp_state_text_sensor;
    this->oil_return_cycle_active = oil_return_cycle_active;

    SetNextState(HPState::IDLE);
    ESP_LOGI("amber", "OpenAmberController initialized");
  }

  void loop() {
    UpdateStateMachine();
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
  void UpdateStateMachine()
  {
      const uint32_t now = millis();

      float tc = temp_tc->state;
      float target_temperature = pid_climate->target_temperature;
      float supply_temperature_delta = tc - target_temperature;

      climate::ClimateAction action = thermostat_climate->action;
      bool frost_protection_stage2_active = frost_protection_stage2->state;
      bool frost_protection_stage1_active = frost_protection_stage1->state;

      bool heating_or_cooling_demand =
          (action == climate::CLIMATE_ACTION_HEATING ||
          action == climate::CLIMATE_ACTION_COOLING);

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
                if(!this->oil_return_cycle_active->state)
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
    float current_compressor_frequency = SUPPORTED_FREQUENCIES[state.commanded_freq_index];
    int desired_compressor_index = MapPIDToCompressorFrequencyIndex(state.compressor_pid);
    float desired_compressor_frequency = SUPPORTED_FREQUENCIES[desired_compressor_index];
    
    const uint32_t min_interval = (current_compressor_frequency > desired_compressor_frequency ? FREQUENCY_CHANGE_INTERVAL_DOWN_S : FREQUENCY_CHANGE_INTERVAL_UP_S) * 1000;
    if ((now - state.last_compressor_freq_change) < min_interval) {
      ESP_LOGI("amber", "Frequency change skipped (interval not elapsed)");
      return;
    }

    // Deadband on Tc to avoid too frequent changes around setpoint
    float tc = temp_tc->state;
    float target = pid_climate->target_temperature;
    float dt = tc - target;

    if (fabsf(dt) < DEAD_BAND_DT) {
      ESP_LOGD("amber", "ΔT=%.2f°C within deadband, frequency remains at %.1f Hz",
              dt, current_compressor_frequency);
      return;
    }
    
    ESP_LOGI("amber", "PID %.2f -> %.1f Hz (previous %.1f Hz)", state.compressor_pid, desired_compressor_frequency, current_compressor_frequency);

    if (desired_compressor_index != state.commanded_freq_index) {
      SetCompressorFrequency(desired_compressor_index);
      ESP_LOGI("amber", "Compressor frequency updated to %.1f Hz", desired_compressor_frequency);
    }
  }

  void StopCompressor()
  {
    // Let pump run for another interval cycle seconds after stopping the compressor.
    state.pump_start_time = millis();

    state.last_compressor_start = 0;
    state.last_compressor_stop = millis();
    state.commanded_freq_index = -1;
    auto compressor_set_call = compressor_control->make_call();
    compressor_set_call.select_first();
    compressor_set_call.perform();

    pid_climate->reset_integral_term();
  }

  void StartCompressor(float supply_temperature_delta) {
    ESP_LOGI("amber", "Starting compressor (ΔT=%.2f°C)", supply_temperature_delta);
    int compressor_frequency_index = DetermineSoftStartFrequencyIndex(supply_temperature_delta);
    float start_compressor_frequency = SUPPORTED_FREQUENCIES[compressor_frequency_index];
    ESP_LOGI("amber", "Soft start frequency: %.1f Hz (ΔT=%.1f°C)", start_compressor_frequency, supply_temperature_delta);

    pid_climate->reset_integral_term();
    SetCompressorFrequency(compressor_frequency_index);
  }

  /// @brief Determine the frequency at which to start the compressor.
  /// In this soft start period the PID loop can act on karakteristics of the measured supply temperature(Tc).
  /// @param supply_temperature_delta Temperature delta between Tc and target temperature.
  /// @return Compressor frequency index.
  int DetermineSoftStartFrequencyIndex(float supply_temperature_delta) {
    int start_compressor_frequency_index;
    float delta = fabs(supply_temperature_delta);
    if (delta < 2.0f) start_compressor_frequency_index = 1;
    else if (delta < 5.0f) start_compressor_frequency_index = 3;
    else if (delta < 8.0f) start_compressor_frequency_index = 4;
    else start_compressor_frequency_index = 5;
    return start_compressor_frequency_index;
  }
  /// @brief Map the PID output value to a discrete compressor frequency index.
  /// Limits the frequency change to a single step per update.
  int MapPIDToCompressorFrequencyIndex(float pid) {
    float raw = fabsf(pid);
    if (raw > 1.0f) raw = 1.0f;

    int desired_index = (int) roundf(raw * (FREQUENCY_COUNT - 1));
    desired_index = std::max(0, std::min(desired_index, FREQUENCY_COUNT - 1));

    // Limit to a single frequency step per update
    int new_index = desired_index;
    if (desired_index > state.commanded_freq_index + 1) new_index = state.commanded_freq_index + 1;
    if (desired_index < state.commanded_freq_index - 1) new_index = state.commanded_freq_index - 1;

    return new_index;
  }

  void SetCompressorFrequency(int frequency_index)
  {
    auto compressor_set_call = compressor_control->make_call();
    compressor_set_call.set_index(frequency_index + 1);
    state.commanded_freq_index = frequency_index;
    compressor_set_call.perform();
    state.last_compressor_freq_change = millis();
  }

  void StartPump() {
    ESP_LOGI("amber", "Starting pump (interval cycle)");
    auto pump_call = pump_control->make_call();
    pump_call.set_index(4);
    pump_call.perform();
    state.pump_start_time = millis();
  }

  void StopPump()
  {
    auto pump_call = pump_control->make_call();
    pump_call.set_index(0);
    pump_call.perform();
  }

  void SetWorkingModeFromClimate()
  {
      climate::ClimateAction action = thermostat_climate->action;

      if (action == climate::CLIMATE_ACTION_HEATING)
      {
          SetWorkingMode(WORKING_MODE_HEATING);
      }
      else if (action == climate::CLIMATE_ACTION_COOLING)
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
