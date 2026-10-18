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

class OffDelayTracker
{
public:
  bool update(bool raw_demand, float delay_minutes, const char* name = nullptr)
  {
    if (raw_demand)
    {
      off_delay_start_ms_ = 0;
      was_active_ = true;
      return true;
    }

    if (delay_minutes <= 0.0f || !was_active_)
    {
      reset();
      return false;
    }

    uint32_t now = millis();
    if (off_delay_start_ms_ == 0)
    {
      off_delay_start_ms_ = now;
      if (name != nullptr)
      {
        ESP_LOGI("amber", "Externe vraag %s weggevallen, uitschakelvertraging van %.0f min gestart", name, delay_minutes);
      }
    }

    uint32_t delay_ms = static_cast<uint32_t>(delay_minutes * 60000.0f);
    if ((now - off_delay_start_ms_) < delay_ms)
    {
      return true;
    }

    if (name != nullptr)
    {
      ESP_LOGI("amber", "Externe vraag %s uitschakelvertraging verstreken", name);
    }
    reset();
    return false;
  }

  void reset()
  {
    off_delay_start_ms_ = 0;
    was_active_ = false;
  }

  bool is_delay_active() const
  {
    return off_delay_start_ms_ != 0;
  }

private:
  uint32_t off_delay_start_ms_{0};
  bool was_active_{false};
};
