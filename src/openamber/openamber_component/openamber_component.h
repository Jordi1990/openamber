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
class ThreeWayValveController;
class PumpController;
class CompressorController;
class BackupHeaterController;
class DHWController;
enum class HPState;

namespace esphome {
namespace openamber {

/**
 * ESPHome Component interface for Open Amber heat pump controller.
 * 
 * NOTE: Implementation is in controllers/heat_pump_controller.h, not here!
 * This is necessary because the implementation needs access to ESPHome's id() macro,
 * which only works in the main ESPHome context, not in component subdirectories.
 */
class OpenAmberComponent : public Component {
private:
  HPState hp_state_;
  HPState deferred_hp_state_;
  uint32_t defer_state_change_until_ms_;
  bool is_switching_modes_;

  ThreeWayValveController *valve_controller_;
  PumpController *pump_controller_;
  CompressorController *compressor_controller_;
  BackupHeaterController *backup_heater_controller_;
  DHWController *dhw_controller_;

  void SetNextState(HPState new_state);
  void LeaveStateAndSetNextStateAfterWaitTime(HPState new_state, uint32_t defer_ms);
  void DoSafetyChecks();
  void UpdateStateMachine();

public:
  OpenAmberComponent();
  ~OpenAmberComponent();
  
  void setup() override;
  void loop() override;
  void write_heat_pid_value(float value);
  void reset_pump_interval();
};

}  // namespace openamber
}  // namespace esphome
