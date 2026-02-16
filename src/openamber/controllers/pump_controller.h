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

using namespace esphome;

class PumpController
{
private:
  uint32_t pump_start_time_ = 0;
  uint32_t next_pump_cycle_ = 0;
  bool initialized = false;

public:
  PumpController() {}

  bool ShouldStartNextPumpCycle()
  {
    return millis() >= next_pump_cycle_;
  }

  bool IsPumpSettled()
  {
    return millis() - pump_start_time_ >= COMPRESSOR_MIN_TIME_PUMP_ON * 1000UL;
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

  void ApplySpeedChangeIfNeeded(float pump_speed_preference)
  {
    if (!id(internal_pump_active).state)
    {
      return;
    }

    SetPwmDutyCycle(pump_speed_preference);
  }

  void Start(float pump_speed_preference)
  {
    ESP_LOGI("amber", "Starting pump (interval cycle)");
    if (!id(pump_p0_relay_switch).state)
    {
      ESP_LOGW("amber", "Pump P0 relay is not active, activating it now.");
      id(pump_p0_relay_switch).turn_on();
    }

    SetPwmDutyCycle(pump_speed_preference);
    pump_start_time_ = millis();
  }

  void Stop()
  {
    initialized = true;
    SetPwmDutyCycle(0);

    RestartPumpInterval();
  }

  bool IsIntervalCycleFinished()
  {
    uint32_t duration_ms = (uint32_t)id(pump_duration).state * 60000UL;
    return millis() >= pump_start_time_ + duration_ms;
  }

  void ResetInterval()
  {
    next_pump_cycle_ = 0;
  }

  void RestartPumpInterval()
  {
    uint32_t interval_ms = (uint32_t)id(pump_interval).state * 60000UL;
    next_pump_cycle_ = millis() + interval_ms;
  }

  bool IsRunning()
  {
    return id(internal_pump_active).state;
  }

  bool IsInitialized()
  {
    return initialized;
  }
};
