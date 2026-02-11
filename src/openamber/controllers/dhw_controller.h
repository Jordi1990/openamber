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
#include "controller.h"
#include "constants.h"

using namespace esphome;

class DHWController : Controller
{
private:
  float start_current_temperature = 0.0f;
public:
  DHWController() {}

  void HandlePumpStart()
  {
    if(!id(dhw_pump_relay_switch).state && valve_controller_.IsInDhwMode())
    {
      int pump_start_mode = id(dhw_pump_start_mode_select).active_index().value();
      if (pump_start_mode == DHW_START_PUMP_MODE_DIRECT)
      {
        ESP_LOGI("amber", "Starting DHW pump since compressor is running and DHW pump start mode is set to compressor active.");
        id(dhw_pump_relay_switch).turn_on();
      }
      else if (pump_start_mode == DHW_START_PUMP_MODE_DELTA_T)
      {
        float dhw_temp = id(dhw_temperature_tw_sensor).state;
        float temperature_output = id(outlet_temperature_tuo).state;
        if (temperature_output >= dhw_temp)
        {
          ESP_LOGI("amber", "Starting DHW pump since Tuo(%.2f°C) is higher than Tw(%.2f°C) and DHW pump start mode is set to ΔT.", temperature_output, dhw_temp);
          id(dhw_pump_relay_switch).turn_on();
        }
      }
    }
  }

  void CheckLegionellaCycle()
  {
    if(!id(legio_enabled_switch).state || !id(dhw_enabled_switch).state)
    {
      return;
    }

    auto now = id(my_time).now();
    auto legionella_time = id(next_legionella_run).state_as_esptime();
    if(!id(dhw_legionella_run_active_sensor).state && now >= legionella_time)
    {
      ESP_LOGI("amber", "Starting legionella cycle.");
      id(dhw_legionella_run_active_sensor).publish_state(true);
      auto next_run = id(next_legionella_run).state_as_esptime();
      for(int i=1; i<=id(legio_repeat_days_number).state; i++)
      {
        next_run.increment_day();
      }
      auto legio_start_call = id(next_legionella_run).make_call();
      legio_start_call.set_datetime(next_run);
      legio_start_call.perform();
      ESP_LOGI("amber", "Next legionella cycle scheduled at %04d-%02d-%02d %02d:%02d", 
               id(next_legionella_run).year, id(next_legionella_run).month, 
               id(next_legionella_run).day, id(next_legionella_run).hour, 
               id(next_legionella_run).minute);
    }
    else if(id(dhw_legionella_run_active_sensor).state && !id(dhw_demand_active_sensor).state)
    {
      ESP_LOGI("amber", "Legionella cycle completed, target temperature %.2f°C reached.", id(legio_target_temperature_number).state);
      id(dhw_legionella_run_active_sensor).publish_state(false);
    }
  }

  bool HasDemand() override
  {
    return id(dhw_demand_active_sensor).state;
  }

  void OnCompressorStarted() override {
    start_current_temperature = id(heat_cool_temperature_tc).state;
    HandlePumpStart();
  }

  float GetPreferredPumpSpeed() override {
    return id(pump_speed_dhw_number).state;
  }

  bool ShouldWaitForTemperatureStabilizationBeforeCompressorStart() override {
    return false;
  }

  bool ShouldSoftStart() override {
    return false;
  }

  int DetermineCompressorMode() override
  {
    float temperature_ta = id(temperature_outside_ta).state;
    float ta_threshold = id(dhw_temperature_threshold_max_compressor_mode).state;
    
    int dhw_mode_offset = id(compressor_control_select).size() - id(dhw_compressor_mode).size();
    int dhw_mode_max_offset = id(compressor_control_select).size() - id(dhw_compressor_mode_max).size();
    
    int selected_dhw_compressor_mode = id(dhw_compressor_mode).active_index().value() + dhw_mode_offset;    
    if (temperature_ta <= ta_threshold)
    {
      selected_dhw_compressor_mode = id(dhw_compressor_mode_max).active_index().value() + dhw_mode_max_offset;
    }
    return selected_dhw_compressor_mode;
  }

  bool ShouldStopCompressor() override
  {
    // In DHW mode, stop when target temperature is reached
    float current_temperature = id(dhw_temperature_tw_sensor).state;
    float target_temperature = id(current_dhw_setpoint_sensor).state;
    ESP_LOGI("amber", "DHW mode: Checking if current temperature %.2f°C >= target temperature %.2f°C to stop compressor.", current_temperature, target_temperature);
    return current_temperature >= target_temperature;
  }
};