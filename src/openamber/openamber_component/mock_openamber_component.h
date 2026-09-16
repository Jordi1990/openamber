/*
 * Open Amber - Itho Daalderop Amber heat pump controller for ESPHome
 * Mock OpenAmberComponent for host SDL simulation.
 */

#pragma once

#ifdef USE_HOST

#include "esphome/core/component.h"
#include <string>

namespace esphome {
namespace openamber {

class OpenAmberComponent : public PollingComponent {
 public:
  OpenAmberComponent() = default;

  void setup() override {}
  void loop() override {}
  void update() override {}

  void write_heat_pid_value(float value) {}
  void write_cool_pid_value(float value) {}
  void write_pump_p0_pid_value(float value) {}
  void reset_pump_interval() {}

  bool is_maintenance_state() const { return false; }
  void start_deaeration_routine(bool extended) {}
  void stop_deaeration_routine() {}
  bool is_deaeration_running() const { return false; }
  bool is_deaeration_extended() const { return false; }
  int get_deaeration_state() const { return 0; }
  bool is_deaeration_dhw_circuit() const { return false; }
  int get_deaeration_current_cycle() const { return 0; }
  int get_deaeration_cycle_count() const { return 0; }
  int get_deaeration_progress_percent() const { return 0; }
  uint32_t get_deaeration_remaining_seconds() const { return 0; }
  std::string get_deaeration_phase_text() const { return "Idle"; }
  uint32_t get_deaeration_duration_seconds(bool extended) const { return 0; }
};

}  // namespace openamber
}  // namespace esphome

#endif

