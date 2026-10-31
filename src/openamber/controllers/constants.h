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
static const uint32_t WORKING_MODE_FORCED_DEFROST = 4;


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

// ============================================================================
// CUSTOM DEFROST
// ============================================================================

// Defrost control mode (EEPROM 3274) select indices
static const uint32_t DEFROST_CONTROL_BY_MASTER = 0;
static const uint32_t DEFROST_CONTROL_BY_OUTDOOR_UNIT = 1;

// Custom defrost mode select indices
static const uint32_t CUSTOM_DEFROST_MODE_OUTDOOR_UNIT = 0;
static const uint32_t CUSTOM_DEFROST_MODE_DRY_RUN = 1;
static const uint32_t CUSTOM_DEFROST_MODE_OPENAMBER = 2;

// Start condition after the Mitsubishi SRK-ZSX algorithm (technical manual 17-SRK-T-228, chapter 17c):
// the coil temperature must stay below the start line for 3 minutes while the compressor runs.
static const uint32_t CUSTOM_DEFROST_CONFIRM_S = 3 * 60;
static const uint32_t CUSTOM_DEFROST_MIN_COMPRESSOR_ON_S = 5 * 60;
static const float CUSTOM_DEFROST_COLD_REFERENCE_TA_C = -15.0f;
// Cycling compressor route: at least 10 compressor stops since the last defrost and Ta at most 3 °C.
static const uint32_t CUSTOM_DEFROST_CYCLING_MIN_STOPS = 10;
static const float CUSTOM_DEFROST_CYCLING_MAX_TA_C = 3.0f;
// After a forced defrost the outdoor unit only accepts a new one after 30 min compressor time.
static const uint32_t CUSTOM_DEFROST_OUTDOOR_UNIT_LOCK_S = 30 * 60;
// Interval between defrosts adapts to the duration of the last defrost, like the outdoor unit's Smart mode.
static const uint32_t CUSTOM_DEFROST_INTERVAL_MAX_MIN = 120;
static const uint32_t CUSTOM_DEFROST_INTERVAL_STEP_UP_MIN = 20;
static const uint32_t CUSTOM_DEFROST_INTERVAL_STEP_DOWN_MIN = 10;
static const uint32_t CUSTOM_DEFROST_SHORT_DEFROST_S = 3 * 60;
static const uint32_t CUSTOM_DEFROST_LONG_DEFROST_S = 5 * 60;
static const uint32_t CUSTOM_DEFROST_MANUAL_REQUEST_EXPIRY_S = 40 * 60;

static const uint32_t CUSTOM_DEFROST_CONTROL_MODE_WRITE_INTERVAL_S = 60;
static const uint32_t CUSTOM_DEFROST_RESTORE_WORKING_MODE_DELAY_S = 2 * 60;
static const uint32_t CUSTOM_DEFROST_DRY_RUN_MATCH_WINDOW_S = 60 * 60;
static const uint32_t CUSTOM_DEFROST_MAX_REQUEST_ATTEMPTS = 2;
static const uint32_t CUSTOM_DEFROST_OVERRUN_MARGIN_S = 5 * 60;
static const uint32_t CUSTOM_DEFROST_TRIGGER_PULSE_S = 10;
