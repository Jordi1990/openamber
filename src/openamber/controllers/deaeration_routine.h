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

#include "routine_controller.h"

using namespace esphome;

enum class DeaerationState
{
  IDLE,
  STARTING_PUMP,
  HIGH_SPEED,
  LOW_SPEED,
  SWITCHING_VALVE,
  STOPPING_PUMP,
  WAIT_FOR_STATE_SWITCH,
};

/// @brief Deaeration routine.
/// Cycles the circulation pump between high and low speed to purge air from
/// the hydraulic system. Deaerates both circuits (heating/cooling and DHW)
/// by switching the 3-way valve between runs. Supports short and extended modes.
class DeaerationRoutine : public RoutineController<DeaerationState>
{
private:
  int cycles_remaining_ = 0;
  bool first_circuit_done_ = false;
  bool extended_mode_ = false;
  uint32_t start_wait_started_ms_ = 0;
  uint32_t routine_start_time_ms_ = 0;

  int GetCycles() const
  {
    return extended_mode_ ? DEAERATION_EXTENDED_CYCLES : DEAERATION_SHORT_CYCLES;
  }

  uint32_t GetHighSpeedDurationS() const
  {
    return extended_mode_ ? DEAERATION_EXTENDED_HIGH_SPEED_DURATION_S : DEAERATION_SHORT_HIGH_SPEED_DURATION_S;
  }

  uint32_t GetLowSpeedDurationS() const
  {
    return extended_mode_ ? DEAERATION_EXTENDED_LOW_SPEED_DURATION_S : DEAERATION_SHORT_LOW_SPEED_DURATION_S;
  }

  bool IsDhwEnabled() const
  {
    return id(dhw_enabled_switch).state;
  }

  uint32_t GetTotalDurationS() const
  {
    uint32_t cycles = GetCycles();
    uint32_t high_s = GetHighSpeedDurationS();
    uint32_t low_s = GetLowSpeedDurationS();
    uint32_t circuits = IsDhwEnabled() ? 2 : 1;
    uint32_t valve_time = IsDhwEnabled() ? THREE_WAY_VALVE_SWITCH_TIME_S : 0;
    return (circuits * cycles * (high_s + low_s)) + valve_time;
  }

  /// @brief Sets the pump PWM duty cycle via the Modbus register.
  void SetPwmDutyCycle(uint32_t duty_cycle_percent)
  {
    float control_speed = ((duty_cycle_percent * 10.0f) * -1.0f) + 1000.0f;
    auto pump_call = id(pump_control_pwm_number).make_call();
    pump_call.set_value(control_speed);
    pump_call.perform();
  }

  void StartPumps()
  {
    if (!id(pump_p0_relay_switch).state)
    {
      id(pump_p0_relay_switch).turn_on();
    }
    if (id(pump_p1_enabled).state && !id(pump_p1_relay_switch).state)
    {
      id(pump_p1_relay_switch).turn_on();
    }
  }

  void StopPumps()
  {
    SetPwmDutyCycle(0);
    id(pump_p0_relay_switch).turn_off();
    if (id(pump_p1_enabled).state)
    {
      id(pump_p1_relay_switch).turn_off();
    }
  }

  void SetValveToHeatingCooling()
  {
    id(three_way_valve_dhw_switch).turn_off();
    id(three_way_valve_heat_cool_switch).turn_on();
    ESP_LOGI("amber", "DEAERATION: 3-way valve set to Heating/Cooling");
  }

  void SetValveToDhw()
  {
    id(three_way_valve_heat_cool_switch).turn_off();
    id(three_way_valve_dhw_switch).turn_on();
    ESP_LOGI("amber", "DEAERATION: 3-way valve set to DHW");
  }

  const char* StateToString(DeaerationState state) const override
  {
    switch (state)
    {
      case DeaerationState::IDLE:                   return "Idle";
      case DeaerationState::STARTING_PUMP:          return "Starting pump";
      case DeaerationState::HIGH_SPEED:             return "High speed";
      case DeaerationState::LOW_SPEED:              return "Low speed";
      case DeaerationState::SWITCHING_VALVE:        return "Switching valve";
      case DeaerationState::STOPPING_PUMP:          return "Stopping pump";
      case DeaerationState::WAIT_FOR_STATE_SWITCH:  return "Waiting";
      default:                                      return "Unknown";
    }
  }

  void PublishState(const char* state_text) override
  {
    if (state_ == DeaerationState::WAIT_FOR_STATE_SWITCH)
    {
      return; // Retain current phase description during deferred transitions
    }
    if (state_ == DeaerationState::IDLE)
    {
      id(state_machine_state_routine).publish_state("Inactief");
      return;
    }
    std::string text = GetPhaseText() + " • " + GetTimeText();
    id(state_machine_state_routine).publish_state(text);
  }

  const char* LogTag() const override
  {
    return "DEAERATION";
  }

public:
  bool is_extended() const { return extended_mode_; }

  uint32_t GetElapsedSeconds() const
  {
    if (IsIdle() || routine_start_time_ms_ == 0) return 0;
    return (App.get_loop_component_start_time() - routine_start_time_ms_) / 1000UL;
  }

  uint32_t GetRemainingSeconds() const
  {
    uint32_t el = GetElapsedSeconds();
    uint32_t tot = GetTotalDurationS();
    return (el >= tot) ? 0 : (tot - el);
  }

  int GetProgressPercent() const
  {
    if (IsIdle()) return 0;
    uint32_t el = GetElapsedSeconds();
    uint32_t tot = GetTotalDurationS();
    if (tot == 0) return 0;
    int pct = (el * 100) / tot;
    return pct > 100 ? 100 : pct;
  }

  std::string GetPhaseText() const
  {
    switch (state_)
    {
      case DeaerationState::IDLE:
        return "Inactief";
      case DeaerationState::STARTING_PUMP:
        return "Pomp starten...";
      case DeaerationState::HIGH_SPEED:
      {
        const char* circuit = first_circuit_done_ ? "Tapwater" : "CV/Koelen";
        int current_cycle = GetCycles() - cycles_remaining_ + 1;
        char buf[64];
        snprintf(buf, sizeof(buf), "%s: Hoog (%d/%d)", circuit, current_cycle, GetCycles());
        return std::string(buf);
      }
      case DeaerationState::LOW_SPEED:
      {
        const char* circuit = first_circuit_done_ ? "Tapwater" : "CV/Koelen";
        int current_cycle = GetCycles() - cycles_remaining_;
        char buf[64];
        snprintf(buf, sizeof(buf), "%s: Laag (%d/%d)", circuit, current_cycle, GetCycles());
        return std::string(buf);
      }
      case DeaerationState::SWITCHING_VALVE:
        return "Klep wisselen naar Tapwater...";
      case DeaerationState::STOPPING_PUMP:
        return "Pomp uitschakelen...";
      default:
        return "Wachten...";
    }
  }

  std::string GetTimeText() const
  {
    if (IsIdle()) return "Resterend: -";
    uint32_t rem = GetRemainingSeconds();
    int pct = GetProgressPercent();
    char buf[48];
    snprintf(buf, sizeof(buf), "Nog %02u:%02u (%d%%)", static_cast<unsigned int>(rem / 60),
             static_cast<unsigned int>(rem % 60), pct);
    return std::string(buf);
  }

  DeaerationRoutine()
    : RoutineController(DeaerationState::IDLE, DeaerationState::WAIT_FOR_STATE_SWITCH) {}

  /// @brief Start the deaeration routine.
  /// @param extended If true, uses extended mode (more cycles, longer durations).
  void Start(bool extended)
  {
    if (!IsIdle()) return;
    StartRoutine();

    extended_mode_ = extended;
    cycles_remaining_ = GetCycles();
    first_circuit_done_ = false;
    routine_start_time_ms_ = App.get_loop_component_start_time();

    ESP_LOGI("amber", "DEAERATION: Starting %s mode (%d cycles per circuit)",
             extended ? "extended" : "short", cycles_remaining_);

    // Start on heating/cooling circuit
    SetValveToHeatingCooling();
    StartPumps();
    SetPwmDutyCycle(DEAERATION_HIGH_SPEED_PWM);

    start_wait_started_ms_ = App.get_loop_component_start_time();
    SetNextState(DeaerationState::STARTING_PUMP);
  }

  void UpdateStateMachine() override
  {
    switch (state_)
    {
      case DeaerationState::IDLE:
      {
        break;
      }

      case DeaerationState::STARTING_PUMP:
      {
        if (id(internal_pump_active).state)
        {
          start_wait_started_ms_ = 0;
          ESP_LOGI("amber", "DEAERATION: Pump running, starting cycles (%d remaining)", cycles_remaining_);
          SetNextState(DeaerationState::HIGH_SPEED);
          break;
        }

        // Timeout — continue anyway, pump may be running below flow switch threshold
        uint32_t now = App.get_loop_component_start_time();
        if (start_wait_started_ms_ > 0 &&
            now - start_wait_started_ms_ >= DEAERATION_PUMP_START_TIMEOUT_S * 1000UL)
        {
          ESP_LOGW("amber", "DEAERATION: Pump start timeout, continuing anyway");
          start_wait_started_ms_ = 0;
          SetNextState(DeaerationState::HIGH_SPEED);
        }
        break;
      }

      case DeaerationState::HIGH_SPEED:
      {
        if (requested_to_stop_)
        {
          SetNextState(DeaerationState::STOPPING_PUMP);
          break;
        }

        SetPwmDutyCycle(DEAERATION_HIGH_SPEED_PWM);
        ESP_LOGI("amber", "DEAERATION: High speed phase (%lu s), cycles remaining: %d, circuit: %s",
                 GetHighSpeedDurationS(), cycles_remaining_, first_circuit_done_ ? "DHW" : "Heat/Cool");
        LeaveStateAndSetNextStateAfterWaitTime(DeaerationState::LOW_SPEED, GetHighSpeedDurationS() * 1000UL);
        break;
      }

      case DeaerationState::LOW_SPEED:
      {
        if (requested_to_stop_)
        {
          SetNextState(DeaerationState::STOPPING_PUMP);
          break;
        }

        SetPwmDutyCycle(DEAERATION_LOW_SPEED_PWM);
        cycles_remaining_--;

        // Determine what happens after this low-speed phase
        DeaerationState next_state;
        if (cycles_remaining_ > 0)
        {
          next_state = DeaerationState::HIGH_SPEED;
        }
        else if (!first_circuit_done_ && IsDhwEnabled())
        {
          next_state = DeaerationState::SWITCHING_VALVE;
        }
        else
        {
          next_state = DeaerationState::STOPPING_PUMP;
        }

        ESP_LOGI("amber", "DEAERATION: Low speed phase (%lu s), cycles remaining: %d",
                 GetLowSpeedDurationS(), cycles_remaining_);
        LeaveStateAndSetNextStateAfterWaitTime(next_state, GetLowSpeedDurationS() * 1000UL);
        break;
      }

      case DeaerationState::SWITCHING_VALVE:
      {
        if (requested_to_stop_)
        {
          SetNextState(DeaerationState::STOPPING_PUMP);
          break;
        }

        first_circuit_done_ = true;
        cycles_remaining_ = GetCycles();
        SetValveToDhw();

        ESP_LOGI("amber", "DEAERATION: Switching to DHW circuit, waiting %lu s for valve",
                 THREE_WAY_VALVE_SWITCH_TIME_S);
        LeaveStateAndSetNextStateAfterWaitTime(DeaerationState::HIGH_SPEED, THREE_WAY_VALVE_SWITCH_TIME_S * 1000UL);
        break;
      }

      case DeaerationState::STOPPING_PUMP:
      {
        ESP_LOGI("amber", "DEAERATION: Stopping pumps and restoring valve position");
        StopPumps();
        // Restore valve to heating/cooling position
        SetValveToHeatingCooling();
        requested_to_stop_ = false;
        // Use timed wait instead of flow switch (may already be inactive at low PWM)
        LeaveStateAndSetNextStateAfterWaitTime(DeaerationState::IDLE, DEAERATION_PUMP_STOP_WAIT_S * 1000UL);
        break;
      }

      case DeaerationState::WAIT_FOR_STATE_SWITCH:
      {
        // Check for stop request during wait — override deferred transition
        if (requested_to_stop_)
        {
          defer_state_change_until_ms_ = 0;
          SetNextState(DeaerationState::STOPPING_PUMP);
          break;
        }
        ProcessDeferredStateChange();
        break;
      }
    }
  }
};
