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

#include <cstdint>

// Rollover-safe timer helpers.
//
// 32-bit millisecond counters (millis(), App.get_loop_component_start_time()) wrap after
// ~49.7 days. Never store "now + duration" deadlines or compare absolute timestamps; store
// the start time and compare the unsigned elapsed time instead. That stays correct across
// the wrap as long as the timer is checked before ~49.7 days have passed since it started.
//
// Timers that can legitimately run, or sit unchecked, for longer than that (for example a
// pump interval that waits through a long idle period) should use millis_64() instead.

inline bool HasElapsedMs(uint32_t start_ms, uint32_t duration_ms, uint32_t now_ms)
{
  return static_cast<uint32_t>(now_ms - start_ms) >= duration_ms;
}

inline uint32_t RemainingMs(uint32_t start_ms, uint32_t duration_ms, uint32_t now_ms)
{
  const uint32_t elapsed_ms = now_ms - start_ms;
  return elapsed_ms >= duration_ms ? 0 : duration_ms - elapsed_ms;
}
