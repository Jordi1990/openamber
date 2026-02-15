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
enum ThreeWayValvePosition
{
  HEATING_COOLING,
  DHW,
};
enum State {
  INITIALIZING,
  SWITCHING,
  DHW_HEAT,
  HEAT_COOL,
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
  uint32_t last_three_way_valve_switch_ms_ = 0;
  State state_ = State::INITIALIZING;
  void SetThreeWayValve(ThreeWayValvePosition position);
  ThreeWayValvePosition GetThreeWayValvePosition();
  ThreeWayValvePosition GetDesiredThreeWayValvePosition();
  void ApplyWorkingMode();
  void SetNextState(State state);
  const char* StateToString(State state);
public:
  OpenAmberComponent();
  ~OpenAmberComponent();
  
  void setup() override;
  void loop() override;
  void update() override;
  
  void write_heat_pid_value(float value);
  void reset_pump_interval();
};

}  // namespace openamber
}  // namespace esphome
