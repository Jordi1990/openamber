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

// ============================================================================
// HEAT PUMP STATE MACHINE STATES
// ============================================================================

enum class HPState
{
  UNKNOWN,
  INITIALIZE,
  IDLE,
  WAIT_PUMP_RUNNING,
  WAIT_PUMP_STOP,
  PUMP_INTERVAL_RUNNING,
  WAIT_COMPRESSOR_RUNNING,
  HEAT_COOL_COMPRESSOR_SOFTSTART,
  COMPRESSOR_RUNNING,
  WAIT_BACKUP_HEATER_RUNNING,
  BACKUP_HEATER_RUNNING,
  DEFROSTING,
  WAIT_FOR_STATE_SWITCH,
  SWITCH_TO_DHW,
  WAIT_COMPRESSOR_STOP,
  WAIT_3WAY_VALVE_SWITCH,
};


// ============================================================================
// TIMING CONSTANTS (in seconds)
// ============================================================================

// Three-way valve
static const uint32_t THREE_WAY_VALVE_SWITCH_TIME_S = 1 * 60;

// Compressor timing
static const uint32_t COMPRESSOR_MIN_OFF_S = 5 * 60;
static const uint32_t COMPRESSOR_MIN_ON_S = 10 * 60;
static const uint32_t COMPRESSOR_SOFT_START_DURATION_S = 3 * 60;
static const uint32_t COMPRESSOR_MIN_TIME_PUMP_ON = 5 * 60;
static const uint32_t COMPRESSOR_SETTLE_TIME_AFTER_DEFROST_S = 5 * 60;
static const uint32_t FREQUENCY_CHANGE_INTERVAL_DOWN_S = 10;
static const uint32_t FREQUENCY_CHANGE_INTERVAL_UP_S = 5 * 60;

// Backup heater timing
static const uint32_t BACKUP_HEATER_LOOKAHEAD_S = 2 * 60;
static const uint32_t BACKUP_HEATER_OFF_SETTLE_TIME_S = 2 * 60;
static const uint32_t DHW_BACKUP_HEATER_GRACE_PERIOD_S = 10 * 60;


// ============================================================================
// TEMPERATURE CONSTANTS
// ============================================================================

static const float DEAD_BAND_DT = 1.0f;


// ============================================================================
// WORKING MODES
// ============================================================================

static const uint32_t WORKING_MODE_STANDBY = 0;
static const uint32_t WORKING_MODE_COOLING = 1;
static const uint32_t WORKING_MODE_HEATING = 2;


// ============================================================================
// DHW PUMP START MODES
// ============================================================================

static const uint32_t DHW_START_PUMP_MODE_DELTA_T = 0;
static const uint32_t DHW_START_PUMP_MODE_DIRECT = 1;
