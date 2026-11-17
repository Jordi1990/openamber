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

#include <cmath>
#include <cstdint>

/// @brief Checks from temperatures that the water goes through the circuit the 3-way valve should serve.
///
/// After the compressor has run for THREE_WAY_VALVE_CHECK_RUN_TIME_S, the outlet water (Tuo minus
/// THREE_WAY_VALVE_CHECK_TUO_OFFSET_C) must stay within THREE_WAY_VALVE_CHECK_DEVIATION_C of the temperature of that
/// circuit (Tw for DHW, Tc for heating); exactly that deviation is still allowed. A larger deviation that lasts THREE_WAY_VALVE_PROTECTION_HIGH_TEMPERATURE_TIME_S
/// means the valve is stuck or in the wrong position.
class ThreeWayValveCheck
{
private:
  bool running_ = false;
  uint32_t running_since_ms_ = 0;
  bool deviating_ = false;
  uint32_t deviating_since_ms_ = 0;
  float last_deviation_ = 0.0f;

public:
  void Reset()
  {
    running_ = false;
    deviating_ = false;
  }

  /// @brief Signed deviation of the last update: (Tuo - offset) - reference.
  float LastDeviation() const { return last_deviation_; }

  /// @param active Compressor running in a state where the valve position is settled (not defrosting).
  /// @param check_high Report Tuo far above the reference (water from a warmer circuit).
  /// @param check_low Report Tuo far below the reference (water from a colder circuit).
  /// @return true once the deviation has lasted long enough.
  bool Update(bool active, float tuo, float reference, bool check_high, bool check_low, uint32_t now)
  {
    if (!active || std::isnan(tuo) || std::isnan(reference))
    {
      Reset();
      return false;
    }

    if (!running_)
    {
      running_ = true;
      running_since_ms_ = now;
    }

    if (now - running_since_ms_ < THREE_WAY_VALVE_CHECK_RUN_TIME_S * 1000UL)
    {
      deviating_ = false;
      return false;
    }

    last_deviation_ = (tuo - THREE_WAY_VALVE_CHECK_TUO_OFFSET_C) - reference;
    const bool deviation = (check_high && last_deviation_ > THREE_WAY_VALVE_CHECK_DEVIATION_C) ||
                           (check_low && last_deviation_ < -THREE_WAY_VALVE_CHECK_DEVIATION_C);
    if (!deviation)
    {
      deviating_ = false;
      return false;
    }

    if (!deviating_)
    {
      deviating_ = true;
      deviating_since_ms_ = now;
    }
    if ((now - deviating_since_ms_) < THREE_WAY_VALVE_PROTECTION_HIGH_TEMPERATURE_TIME_S * 1000UL)
    {
      return false;
    }
    // Start over after a trip: once the error is reset, the full run and hold time apply again.
    Reset();
    return true;
  }
};
