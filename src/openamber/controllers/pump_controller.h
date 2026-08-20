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

#include <algorithm>
#include <cmath>

#include "esphome.h"
#include "constants.h"

using namespace esphome;

class PumpController
{
private:
  uint32_t pump_start_time_ = 0;
  uint32_t next_pump_cycle_ = 0;
  uint32_t start_wait_started_ms_ = 0;
  uint32_t stop_wait_started_ms_ = 0;
  float pump_p0_pid_output_ = 0.0f;

  float CalculateHeatingPidPumpSpeed()
  {
    float min_pwm = id(pump_p0_pid_min_pwm).state;
    float max_pwm = id(pump_p0_pid_max_pwm).state;
    if (min_pwm > max_pwm)
    {
      ESP_LOGW("amber", "Pump P0 PID min PWM (%.0f) is greater than max PWM (%.0f), falling back to 100%% PWM instead.", min_pwm, max_pwm);
      min_pwm = 100.0f;
      max_pwm = 100.0f;
    }

    const float normalized_output = 1.0f - pump_p0_pid_output_;
    const float requested_pwm = min_pwm + ((max_pwm - min_pwm) * normalized_output);
    ESP_LOGD("amber", "P0 climate PID output=%.2f -> %.0f%% PWM", pump_p0_pid_output_, requested_pwm);
    return requested_pwm;
  }

  bool IsCoolingDemand()
  {
    return id(cool_demand_active_sensor).state;
  }

  float GetPreferredPumpSpeed() {
    if(id(dhw_active).state)
    {
      return id(pump_speed_dhw_number).state;
    }

    return IsCoolingDemand() ? id(pump_speed_cooling_number).state : id(pump_speed_heating_number).state;
  }

public:
  PumpController() {}

  bool ShouldStartNextPumpCycle()
  {
    return App.get_loop_component_start_time() >= next_pump_cycle_;
  }

  bool IsPumpSettled()
  {
    return App.get_loop_component_start_time() - pump_start_time_ >= COMPRESSOR_MIN_TIME_PUMP_ON * 1000UL;
  }

  void SetPwmDutyCycle(float duty_cycle)
  {
    const uint32_t now = App.get_loop_component_start_time();

    const float control_speed = ((duty_cycle * 10.0f) * -1.0f) + 1000.0f;
    if (id(pump_p0_current_pwm_sensor).get_raw_state() != control_speed)
    {
      ESP_LOGI("amber", "Applying pump speed change to %.0f (Duty cycle: %.0f%%)", control_speed, duty_cycle);
      auto pump_call = id(pump_control_pwm_number).make_call();
      pump_call.set_value(control_speed);
      pump_call.perform();
    }
  }

  void ApplySpeedChangeIfNeeded(bool compressor_settled)
  {
    if (!id(internal_pump_active).state)
    {
      return;
    }

    if(id(pump_p0_pid_enabled).state && compressor_settled && !id(dhw_active).state && !IsCoolingDemand())
    {
      SetPwmDutyCycle(CalculateHeatingPidPumpSpeed());
    }
    else
    {
      SetPwmDutyCycle(GetPreferredPumpSpeed());
    }
  }

  void SetPumpP0PidOutput(float output)
  {
    pump_p0_pid_output_ = std::max(0.0f, std::min(1.0f, output));
  }

  void ResetHeatingPidState()
  {
    pump_p0_pid_output_ = 0.0f;
    id(pump_p0_pid_temperature_control).reset_integral_term();
  }

  void Start()
  {
    ESP_LOGI("amber", "Starting pump (interval cycle)");
    if (!id(pump_p0_relay_switch).state)
    {
      ESP_LOGW("amber", "Pump P0 relay is not active, activating it now.");
      id(pump_p0_relay_switch).turn_on();
    }

    pump_start_time_ = App.get_loop_component_start_time();
    start_wait_started_ms_ = pump_start_time_;
    ResetHeatingPidState();
    SetPwmDutyCycle(GetPreferredPumpSpeed());
    ClearStopWaitTimer();
  }

  void Stop()
  {
    ClearStartWaitTimer();
    stop_wait_started_ms_ = App.get_loop_component_start_time();
    ResetHeatingPidState();
    SetPwmDutyCycle(0);

    RestartPumpInterval();
  }

  void ClearStartWaitTimer()
  {
    start_wait_started_ms_ = 0;
  }

  uint32_t GetStartWaitStartedTime()
  {
    return start_wait_started_ms_;
  }

  void ClearStopWaitTimer()
  {
    stop_wait_started_ms_ = 0;
  }

  uint32_t GetStopWaitStartedTime()
  {
    return stop_wait_started_ms_;
  }

  bool IsIntervalCycleFinished()
  {
    uint32_t duration_ms = (uint32_t)id(pump_duration).state * 60000UL;
    return App.get_loop_component_start_time() >= pump_start_time_ + duration_ms;
  }

  void ResetInterval()
  {
    next_pump_cycle_ = 0;
  }

  void RestartPumpInterval()
  {
    uint32_t interval_ms = (uint32_t)id(pump_interval).state * 60000UL;
    next_pump_cycle_ = App.get_loop_component_start_time() + interval_ms;
  }

  bool IsRunning()
  {
    return id(internal_pump_active).state;
  }
};
