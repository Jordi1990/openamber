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
// TIMING CONSTANTS (in seconds)
// ============================================================================

static const uint32_t INITIALIZATION_DELAY_S = 10;

// Modbus connection
static const uint32_t MODBUS_CONNECTION_TIMEOUT_S = 2 * 60;

// Three-way valve
static const uint32_t THREE_WAY_VALVE_SWITCH_TIME_S = 1 * 60;
static const uint32_t THREE_WAY_VALVE_PROTECTION_DELTA_TEMPERATURE_C = 5.0f;
static const uint32_t THREE_WAY_VALVE_PROTECTION_HIGH_TEMPERATURE_TIME_S = 2 * 60;
static const float THREE_WAY_VALVE_PROTECTION_TUO_TC_THRESHOLD_C = 5.0f;

// Compressor timing
static const uint32_t COMPRESSOR_MIN_OFF_S = 2 * 60;
static const uint32_t COMPRESSOR_MIN_ON_S = 10 * 60;
static const uint32_t COMPRESSOR_SOFT_START_DURATION_S = 3 * 60;
static const uint32_t COMPRESSOR_START_TIMEOUT_S = 2 * 60;
static const uint32_t COMPRESSOR_MIN_TIME_PUMP_ON = 2 * 60;
static const uint32_t COMPRESSOR_SETTLE_TIME_AFTER_DEFROST_S = 5 * 60;
static const uint32_t FREQUENCY_CHANGE_INTERVAL_DOWN_S = 10;
static const uint32_t FREQUENCY_CHANGE_INTERVAL_UP_S = 5 * 60;

// Ambient temperature bands the outdoor unit uses to cap the compressor (firmware v1.18, same as v1.10).
// A band is entered 1 K above and left 1 K below its boundary. Band 0 is below the first boundary,
// band N is at or above the last one.
static const float OUTDOOR_UNIT_HEATING_BAND_BOUNDS_C[] = {-25.0f, 12.0f, 20.0f, 24.0f, 28.0f, 46.0f};
static const float OUTDOOR_UNIT_COOLING_BAND_BOUNDS_C[] = {0.0f, 12.0f, 18.0f, 24.0f, 30.0f, 41.0f, 45.0f, 51.0f, 65.0f};
// The first and last band of each table are outside the operating range: the unit blocks the compressor
// there (P11) and OpenAmber leaves that to the unit. The bands in between have a configurable maximum
// compressor level (core/numbers.yaml).
static const int OUTDOOR_UNIT_HEATING_LIMITED_BANDS = 5;
static const int OUTDOOR_UNIT_COOLING_LIMITED_BANDS = 8;
static const float OUTDOOR_UNIT_BAND_HYSTERESIS_C = 1.0f;

// Pump timing
static const uint32_t PUMP_START_TIMEOUT_S = 2 * 60;
static const uint32_t PUMP_STOP_TIMEOUT_S = 2 * 60;

// Backup heater timing
static const uint32_t BACKUP_HEATER_LOOKAHEAD_S = 2 * 60;
static const uint32_t BACKUP_HEATER_PREDICTION_SETTLE_TIME_S = 5 * 60;
static const uint32_t BACKUP_HEATER_OFF_SETTLE_TIME_S = 2 * 60;
static const uint32_t DHW_BACKUP_HEATER_GRACE_PERIOD_S = 10 * 60;

// Dhw
static const uint32_t DHW_PUMP_TEMPERATURE_SETTLE_TIME_S = 2 * 60;
// Window over which the DHW heating rate (°C/min) is averaged. Tw is polled every 15s over Modbus
// with limited resolution, so a rate taken between two 5s controller updates is mostly 0 with
// occasional spikes; averaging over a few minutes gives the actual heating speed.
static const uint32_t DHW_HEATING_RATE_WINDOW_S = 3 * 60;


// ============================================================================
// WORKING MODES
// ============================================================================

static const uint32_t WORKING_MODE_STANDBY = 0;
static const uint32_t WORKING_MODE_COOLING = 1;
static const uint32_t WORKING_MODE_HEATING = 2;
static const uint32_t WORKING_MODE_MAINTENANCE = 3;


// ============================================================================
// DHW PUMP START MODES
// ============================================================================

static const uint32_t DHW_START_PUMP_MODE_DELTA_T = 0;
static const uint32_t DHW_START_PUMP_MODE_DIRECT = 1;

// ============================================================================
// EEPROM VERSIONS
// ============================================================================
static const uint32_t EEPROM_VERSION_AMBER = 114;
static const uint32_t EEPROM_VERSION_HPS = 29184;

// ============================================================================
// DEAERATION ROUTINE
// ============================================================================

// Shared
static const uint32_t DEAERATION_HIGH_SPEED_PWM = 100;      // % pump speed high phase
static const uint32_t DEAERATION_LOW_SPEED_PWM = 30;        // % pump speed low phase
static const uint32_t DEAERATION_PUMP_START_TIMEOUT_S = 2 * 60;
static const uint32_t DEAERATION_PUMP_STOP_WAIT_S = 5;      // Timed wait instead of flow switch

// Short mode (~10 min total)
static const uint32_t DEAERATION_SHORT_CYCLES = 3;
static const uint32_t DEAERATION_SHORT_HIGH_SPEED_DURATION_S = 60;
static const uint32_t DEAERATION_SHORT_LOW_SPEED_DURATION_S = 30;

// Extended mode (~31 min total)
static const uint32_t DEAERATION_EXTENDED_CYCLES = 5;
static const uint32_t DEAERATION_EXTENDED_HIGH_SPEED_DURATION_S = 2 * 60;
static const uint32_t DEAERATION_EXTENDED_LOW_SPEED_DURATION_S = 60;
