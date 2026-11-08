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
// A started compressor that reports 0 Hz this long, outside a defrost or oil return cycle, has been
// stopped by the outdoor unit itself (protection, power or communication loss).
static const uint32_t COMPRESSOR_UNEXPECTED_STOP_S = 3 * 60;
// This many unexpected stops within the window raise an error instead of another restart.
static const uint8_t COMPRESSOR_UNEXPECTED_STOP_ERROR_COUNT = 3;
static const uint32_t COMPRESSOR_UNEXPECTED_STOP_WINDOW_S = 60 * 60;
// A longer gap between two unexpected stop checks restarts the 0 Hz period.
static const uint32_t COMPRESSOR_UNEXPECTED_STOP_CHECK_GAP_S = 10;
// The outdoor unit raises 2118 up to 300 s before the actual defrost when it decides itself (3274 = 1).
// The actual defrost starts once the compressor runs at the defrost standby frequency (3279).
static const uint32_t DEFROST_PREPHASE_MAX_S = 330;
static const float DEFROST_STANDBY_FREQUENCY_MARGIN_HZ = 5.0f;
// After 2118 drops the unit switches the 4-way valve back and restarts the fan within 30 s.
static const uint32_t DEFROST_RECOVERY_PHASE_S = 30;
static const uint32_t FREQUENCY_CHANGE_INTERVAL_DOWN_S = 10;
static const uint32_t FREQUENCY_CHANGE_INTERVAL_UP_S = 5 * 60;

// Ambient temperature limit (outdoor unit firmware v1.18, same as v1.10, registers 3282-3298). OpenAmber
// writes 3281 = 0 at start, which turns the unit's own level-based limit off, and applies the limit itself.
// A band is entered 1 K above and left 1 K below its boundary. Band 0 is below the first boundary, band N
// is at or above the last one.
static const float OUTDOOR_UNIT_HEATING_BAND_BOUNDS_C[] = {-25.0f, 12.0f, 20.0f, 24.0f, 28.0f, 46.0f};
static const float OUTDOOR_UNIT_COOLING_BAND_BOUNDS_C[] = {0.0f, 12.0f, 18.0f, 24.0f, 30.0f, 41.0f, 45.0f, 51.0f, 65.0f};
// Maximum frequency per band, so the limit does not move when OpenAmber's frequency table changes. The
// values are the stock v1.18 level limits (heating 0/10/8/5/3/2/0, cooling 0/2/3/6/10/10/7/4/1/0) looked up
// in the stock v1.18 frequency table (Amber parameter set, heating 30-90 Hz, cooling 30-79 Hz).
// 0 Hz is outside the operating range (P11 in the stock unit): OpenAmber caps to level 1 there, never to 0.
static const float OUTDOOR_UNIT_HEATING_BAND_MAX_HZ[] = {0.0f, 90.0f, 74.0f, 55.0f, 43.0f, 36.0f, 0.0f};
static const float OUTDOOR_UNIT_COOLING_BAND_MAX_HZ[] = {0.0f, 36.0f, 42.0f, 56.0f, 79.0f, 79.0f, 61.0f, 47.0f, 30.0f, 0.0f};
static const float OUTDOOR_UNIT_BAND_HYSTERESIS_C = 1.0f;
// Heating below the low ambient limit (-25 °C, 3498 - 3499 - 30) only restarts 3500 (4 K) higher on top of
// the normal hysteresis, around -20 °C.
static const float OUTDOOR_UNIT_HEATING_LOW_LIMIT_RESTART_C = 4.0f;
// OpenAmber moves to a band with a lower (non-zero) limit this much later in Ta than the stock unit.
// Moving to a band with a higher limit, and to a band outside the operating range, happen at the stock Ta.
static const float AMBIENT_LIMIT_TA_OFFSET_C = 1.0f;

// Discharge side limiter: stay below the outdoor unit's own Pd/Td protection instead of running
// into it. The limits are margins below the unit's "no further frequency increase" thresholds,
// read from 3269 (Pd) and 3262 (Td); the defaults are used until those are read (EEPROM 114).
static const float DISCHARGE_LIMIT_DEFAULT_UNIT_PD_BAR = 26.0f;
static const float DISCHARGE_LIMIT_DEFAULT_UNIT_TD_C = 95.0f;
static const float DISCHARGE_LIMIT_HOLD_PD_MARGIN_BAR = 2.0f;
static const float DISCHARGE_LIMIT_STEP_DOWN_PD_MARGIN_BAR = 1.0f;
static const float DISCHARGE_LIMIT_RELEASE_PD_MARGIN_BAR = 3.0f;
static const float DISCHARGE_LIMIT_HOLD_TD_MARGIN_C = 5.0f;
static const float DISCHARGE_LIMIT_STEP_DOWN_TD_MARGIN_C = 2.0f;
static const float DISCHARGE_LIMIT_RELEASE_TD_MARGIN_C = 8.0f;
static const uint32_t DISCHARGE_LIMIT_STEP_DOWN_INTERVAL_S = 60;
static const int DISCHARGE_LIMIT_MIN_MODE = 1;

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
