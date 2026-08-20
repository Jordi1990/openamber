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

#include "esphome/core/component.h"

// Forward declarations
class DHWController;
class HeatCoolController;
class PumpController;
class CompressorController;
class DeaerationRoutine;
enum ThreeWayValvePosition
{
  HEATING_COOLING,
  DHW,
};
enum State {
  UNKNOWN,
  WAIT_INITIALIZATION,
  INITIALIZING,
  DHW_HEAT,
  HEAT_COOL,
  MAINTENANCE,
  WAIT_FOR_STATE_SWITCH,
};
namespace esphome {
namespace openamber {

/**
 * ESPHome Component wrapper for Open Amber heat pump controller.
 */
class OpenAmberComponent : public PollingComponent {
private:
  DHWController* dhw_controller_;
  HeatCoolController* heat_cool_controller_;
  PumpController *pump_controller_;
  CompressorController *compressor_controller_;
  DeaerationRoutine *deaeration_routine_;
  State deferred_machine_state_;
  uint32_t defer_state_change_until_ms_;
  State state_ = State::INITIALIZING;
  void SetThreeWayValve(ThreeWayValvePosition position);
  ThreeWayValvePosition GetThreeWayValvePosition();
  ThreeWayValvePosition GetDesiredThreeWayValvePosition();

  void SetNextState(State state);
  void LeaveStateAndSetNextStateAfterWaitTime(State new_state, uint32_t defer_ms);
  const char* StateToString(State state);
  void WriteHeatingFrequencyTable();
  void WriteCoolingFrequencyTable();
public:
  OpenAmberComponent();
  ~OpenAmberComponent();
  
  void setup() override;
  void loop() override;
  void update() override;
  
  void write_heat_pid_value(float value);
  void write_cool_pid_value(float value);
  void write_pump_p0_pid_value(float value);
  void reset_pump_interval();
  bool is_maintenance_state() const;
  void start_deaeration_routine(bool extended);
  void stop_deaeration_routine();
  bool is_deaeration_running() const;
  bool is_deaeration_extended() const;
  int get_deaeration_state() const;
  bool is_deaeration_dhw_circuit() const;
  int get_deaeration_current_cycle() const;
  int get_deaeration_cycle_count() const;
  int get_deaeration_progress_percent() const;
  uint32_t get_deaeration_remaining_seconds() const;
  std::string get_deaeration_phase_text() const;
  uint32_t get_deaeration_duration_seconds(bool extended) const;
};

}  // namespace openamber
}  // namespace esphome
