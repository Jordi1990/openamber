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

using namespace esphome;

// 3-way valve positions
enum ThreeWayValvePosition
{
  HEATING_COOLING,
  DHW,
};

class ThreeWayValveController
{
public:
  ThreeWayValveController() = default;

  bool IsInPosition(ThreeWayValvePosition position) const
  {
    return id(three_way_valve_select).active_index() == (int)position;
  }

  bool IsInDhwMode() const
  {
    return IsInPosition(ThreeWayValvePosition::DHW);
  }

  void SetPosition(ThreeWayValvePosition position)
  {
    auto three_way_valve_call = id(three_way_valve_select).make_call();
    three_way_valve_call.set_index((int)position);
    three_way_valve_call.perform();
    ESP_LOGI("amber", "Setting 3-way valve to %s.", id(three_way_valve_select).current_option());
  }

  void RestoreStateFromRelay()
  {
    if (id(dhw_enabled_switch).state)
    {
      bool is_valve_in_dhw_position = id(three_way_valve_dhw_switch).state;
      SetPosition(is_valve_in_dhw_position ? ThreeWayValvePosition::DHW : ThreeWayValvePosition::HEATING_COOLING);
    }
    else
    {
      SetPosition(ThreeWayValvePosition::HEATING_COOLING);
    }
  }
};
