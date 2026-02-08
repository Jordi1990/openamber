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
#include "constants.h"
#include "three_way_valve_controller.h"

using namespace esphome;

class PumpController
{
private:
  uint32_t pump_start_time_ = 0;
  uint32_t next_pump_cycle_ = 0;
  const ThreeWayValveController& valve_controller_;

public:
  PumpController(const ThreeWayValveController& valve_controller) 
    : valve_controller_(valve_controller) {}

  uint32_t GetPumpStartTime() const { return pump_start_time_; }
  uint32_t GetNextPumpCycle() const { return next_pump_cycle_; }

  float GetCurrentSpeedSetting() const
  {
    return (valve_controller_.IsInDhwMode() ? id(pump_speed_dhw_number) : id(pump_speed_heating_number)).state;
  }

  void SetPwmDutyCycle(float duty_cycle)
  {
    float control_speed = ((duty_cycle * 10) * -1) + 1000;
    if (id(pump_p0_current_pwm_sensor).raw_state != control_speed)
    {
      ESP_LOGI("amber", "Applying pump speed change to %.0f (Duty cycle: %.0f)", control_speed, duty_cycle);
      auto pump_call = id(pump_control_pwm_number).make_call();
      pump_call.set_value(control_speed);
      pump_call.perform();
    }
  }

  void ApplySpeedChangeIfNeeded()
  {
    if (!id(internal_pump_active).state)
    {
      return;
    }

    SetPwmDutyCycle(GetCurrentSpeedSetting());
  }

  void Start()
  {
    ESP_LOGI("amber", "Starting pump (interval cycle)");
    if (!id(pump_p0_relay_switch).state)
    {
      ESP_LOGW("amber", "Pump P0 relay is not active, activating it now.");
      id(pump_p0_relay_switch).turn_on();
    }

    SetPwmDutyCycle(GetCurrentSpeedSetting());
    pump_start_time_ = millis();
  }

  void Stop()
  {
    if (valve_controller_.IsInDhwMode())
    {
      ESP_LOGI("amber", "Stopping DHW pump.");
      id(dhw_pump_relay_switch).turn_off();
    }

    SetPwmDutyCycle(0);

    uint32_t interval_ms = (uint32_t)id(pump_interval).state * 60000UL;
    next_pump_cycle_ = millis() + interval_ms;
  }

  bool IsIntervalCycleFinished() const
  {
    uint32_t duration_ms = (uint32_t)id(pump_duration).state * 60000UL;
    return millis() >= pump_start_time_ + duration_ms;
  }

  void ResetInterval()
  {
    next_pump_cycle_ = 0;
  }

  bool IsRunning() const
  {
    return id(internal_pump_active).state;
  }
};
