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

class DHWController
{
private:
  const ThreeWayValveController& valve_controller_;

public:
  DHWController(const ThreeWayValveController& valve_controller) 
    : valve_controller_(valve_controller) {}

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

  bool HasDemand() const
  {
    return id(dhw_demand_active_sensor).state;
  }
};
