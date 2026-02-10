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
 * This program is distributed it in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "heat_pump_controller.h"

using namespace esphome;

namespace esphome {
namespace openamber {

OpenAmberComponent::OpenAmberComponent()
{
  heat_pump_controller_ = new HeatPumpController();
}

OpenAmberComponent::~OpenAmberComponent()
{
  delete heat_pump_controller_;
}

void OpenAmberComponent::setup()
{
  ESP_LOGI("amber", "OpenAmberController initialized");
}

void OpenAmberComponent::loop()
{
  heat_pump_controller_->Loop();
}

void OpenAmberComponent::write_heat_pid_value(float value)
{
  heat_pump_controller_->SetPIDValue(value);
}

void OpenAmberComponent::reset_pump_interval()
{
  heat_pump_controller_->ResetPumpInterval();
}

}  // namespace openamber
}  // namespace esphome
