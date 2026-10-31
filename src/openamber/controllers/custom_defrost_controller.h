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
#include <string>
#include "esphome.h"
#include "constants.h"

using namespace esphome;

enum class CustomDefrostState
{
  OFF,
  MONITORING,
  BLOCKED,
  DRY_RUN_TRIGGERED,
  REQUESTED,
  DEFROSTING,
};

/// @brief Decides when the outdoor unit should defrost.
///
/// The start condition follows the Mitsubishi SRK-ZSX algorithm (technical manual 17-SRK-T-228, chapter 17c).
/// All of these must hold at the same time:
///  1. The compressor ran at least the interval since the previous defrost (35 min, adapts to the last defrost).
///  2. The compressor runs continuously for at least 5 min.
///  3. The coil temperature Tp stays below the start line for 3 min. The start line is
///     Ta - (required difference), where the required difference is 7 K at Ta >= 0 °C and 4 K at Ta = -15 °C,
///     and never above -5 °C. All three values are settings, because the Amber is a bigger monoblock than
///     the Mitsubishi split units and may need other values.
/// Cycling route (Mitsubishi): the compressor stopped at least 10 times, Ta <= 3 °C and Tp <= -5 °C for 3 min.
/// Safety net: after the maximum runtime with Tp below the safety net threshold for 3 min.
/// Optional humidity sensor: when the coil is above the frost point of the air, no ice can grow,
/// so the normal and cycling route wait (the safety net does not).
///
/// Modes (custom_defrost_mode_select):
///  - Buitenunit: the outdoor unit decides itself, only statistics are kept here.
///  - Dry-run: the algorithm runs and logs its trigger moments, the outdoor unit still defrosts itself.
///  - OpenAmber: EEPROM 3274 is set to master control and a defrost is started by writing working mode 4.
/// The heat/cool and DHW controllers pick up a pending request in COMPRESSOR_RUNNING via ShouldRequestDefrost().
class CustomDefrostController
{
private:
  static const int HISTORY_SIZE = 24;

  CustomDefrostState state_ = CustomDefrostState::OFF;
  uint32_t last_update_ms_ = 0;
  uint32_t runtime_remainder_ms_ = 0;
  uint32_t compressor_running_since_ms_ = 0;
  uint32_t compressor_stops_ = 0;
  uint32_t below_start_line_since_ms_ = 0;
  uint32_t below_max_coil_since_ms_ = 0;
  uint32_t below_safety_net_since_ms_ = 0;
  uint32_t return_temperature_block_since_ms_ = 0;
  uint32_t dry_run_trigger_ms_ = 0;
  uint32_t request_started_ms_ = 0;
  uint32_t manual_request_ms_ = 0;
  uint32_t defrost_started_ms_ = 0;
  uint32_t forced_mode_seen_since_ms_ = 0;
  uint32_t last_control_mode_write_ms_ = 0;
  uint32_t trigger_pulse_until_ms_ = 0;
  uint32_t request_attempts_ = 0;
  uint32_t history_ms_[HISTORY_SIZE] = {0};
  int history_index_ = 0;
  int last_published_runtime_min_ = -1;
  int last_published_count_ = -1;
  int previous_mode_ = -1;
  float previous_min_interval_ = NAN;
  bool defrost_active_previous_ = false;
  bool request_pending_ = false;
  bool manual_request_ = false;
  bool started_by_openamber_ = false;
  bool overrun_warned_ = false;
  std::string pending_reason_;
  std::string last_status_;

  int GetMode()
  {
    auto idx = id(custom_defrost_mode_select).active_index();
    return idx.has_value() ? (int)idx.value() : (int)CUSTOM_DEFROST_MODE_OUTDOOR_UNIT;
  }

  bool IsWorkingModeIndex(uint32_t working_mode)
  {
    auto idx = id(working_mode_switch).active_index();
    return idx.has_value() && idx.value() == working_mode;
  }

  void WriteWorkingMode(uint32_t working_mode)
  {
    auto call = id(working_mode_switch).make_call();
    call.set_index(working_mode);
    call.perform();
  }

  bool IsCompressorRunning()
  {
    return id(current_compressor_frequency).state > 0;
  }

  static uint32_t NonZero(uint32_t now)
  {
    return now == 0 ? 1 : now;
  }

  static bool IsValid(float value)
  {
    return !std::isnan(value);
  }

  /// @brief Keeps track of how long a condition holds without interruption. Returns the duration in seconds.
  static uint32_t HoldSeconds(bool condition, uint32_t* since_ms, uint32_t now)
  {
    if (!condition)
    {
      *since_ms = 0;
      return 0;
    }
    if (*since_ms == 0)
    {
      *since_ms = NonZero(now);
    }
    return (now - *since_ms) / 1000;
  }

  void PublishStatus(const std::string& status)
  {
    if (status == last_status_)
    {
      return;
    }
    last_status_ = status;
    id(custom_defrost_status).publish_state(status);
  }

  void SetState(CustomDefrostState state)
  {
    if (state_ == state)
    {
      return;
    }
    state_ = state;
    ESP_LOGI("amber", "CustomDefrost state changed: %s", StateToString(state));
  }

  static const char* StateToString(CustomDefrostState state)
  {
    switch (state)
    {
      case CustomDefrostState::OFF:               return "Off";
      case CustomDefrostState::MONITORING:        return "Monitoring";
      case CustomDefrostState::BLOCKED:           return "Blocked";
      case CustomDefrostState::DRY_RUN_TRIGGERED: return "Dry-run triggered";
      case CustomDefrostState::REQUESTED:         return "Requested";
      case CustomDefrostState::DEFROSTING:        return "Defrosting";
      default:                                    return "Unknown";
    }
  }

  void ResetTriggerTimers()
  {
    below_start_line_since_ms_ = 0;
    below_max_coil_since_ms_ = 0;
    below_safety_net_since_ms_ = 0;
    return_temperature_block_since_ms_ = 0;
  }

  // ── Start line (Mitsubishi) ──

  /// @brief Required difference Ta - Tp: constant above 0 °C, linear down to the cold reference (-15 °C), constant below.
  float GetRequiredDelta(float ta)
  {
    float delta_zero = id(custom_defrost_delta_at_zero).state;
    float delta_cold = id(custom_defrost_delta_at_cold).state;
    if (ta >= 0.0f)
    {
      return delta_zero;
    }
    if (ta <= CUSTOM_DEFROST_COLD_REFERENCE_TA_C)
    {
      return delta_cold;
    }
    float fraction = ta / CUSTOM_DEFROST_COLD_REFERENCE_TA_C;
    return delta_zero + (delta_cold - delta_zero) * fraction;
  }

  /// @brief Coil temperature at or below which the coil counts as iced.
  float GetStartTemperature(float ta)
  {
    return std::min(id(custom_defrost_max_coil_temperature).state, ta - GetRequiredDelta(ta));
  }

  // ── Optional humidity sensor ──

  /// @brief Frost point of the outdoor air, or NAN when the humidity sensor is not used or has no valid value.
  float GetFrostPoint(float ta)
  {
    if (!id(custom_defrost_humidity_switch).state)
    {
      return NAN;
    }
    float rh = id(outdoor_humidity).state;
    if (!IsValid(rh) || rh < 1.0f || rh > 100.0f)
    {
      return NAN;
    }
    // Magnus formula: vapour pressure of the air (over water), then the temperature where that pressure saturates over ice.
    float vapour_pressure = rh / 100.0f * 6.112f * expf(17.62f * ta / (243.12f + ta));
    float l = logf(vapour_pressure / 6.112f);
    return 272.62f * l / (22.46f - l);
  }

  // ── Interval between defrosts ──

  uint32_t GetMinIntervalMinutes()
  {
    return std::max((uint32_t)id(custom_defrost_min_runtime).state, CUSTOM_DEFROST_OUTDOOR_UNIT_LOCK_S / 60);
  }

  uint32_t GetIntervalMinutes()
  {
    uint32_t min_interval = GetMinIntervalMinutes();
    uint32_t interval = id(custom_defrost_interval_min);
    if (interval < min_interval)
    {
      return min_interval;
    }
    return std::min(interval, std::max(CUSTOM_DEFROST_INTERVAL_MAX_MIN, min_interval));
  }

  /// @brief A changed minimum interval setting restarts the adaptive interval from that value.
  void CheckMinIntervalChanged()
  {
    float min_interval = id(custom_defrost_min_runtime).state;
    if (min_interval == previous_min_interval_ || std::isnan(min_interval))
    {
      return;
    }
    if (!std::isnan(previous_min_interval_))
    {
      id(custom_defrost_interval_min) = GetMinIntervalMinutes();
    }
    previous_min_interval_ = min_interval;
    id(custom_defrost_interval_sensor).publish_state(GetIntervalMinutes());
  }

  /// @brief Short defrost = little ice, so wait longer next time. Long defrost = wait shorter. Same rule as the outdoor unit's Smart mode.
  void AdaptInterval(uint32_t duration_s)
  {
    uint32_t interval = GetIntervalMinutes();
    uint32_t max_defrost_s = IsValid(id(max_defrost_time).state) ? (uint32_t)(id(max_defrost_time).state * 60.0f) : 8 * 60;
    if (duration_s + 10 >= max_defrost_s)
    {
      interval = GetMinIntervalMinutes();
    }
    else if (duration_s <= CUSTOM_DEFROST_SHORT_DEFROST_S)
    {
      interval += CUSTOM_DEFROST_INTERVAL_STEP_UP_MIN;
    }
    else if (duration_s > CUSTOM_DEFROST_LONG_DEFROST_S)
    {
      interval = interval > CUSTOM_DEFROST_INTERVAL_STEP_DOWN_MIN ? interval - CUSTOM_DEFROST_INTERVAL_STEP_DOWN_MIN : 0;
    }
    id(custom_defrost_interval_min) = interval;
    interval = GetIntervalMinutes();
    id(custom_defrost_interval_min) = interval;
    id(custom_defrost_interval_sensor).publish_state(interval);
    ESP_LOGI("amber", "CustomDefrost: defrost took %u s, interval is now %u min.", (unsigned) duration_s, (unsigned) interval);
  }

  /// @brief The outdoor unit postpones a forced defrost until 30 min compressor time after the previous one.
  bool IsOutdoorUnitLocked()
  {
    if (id(outside_unit_internal_registers_supported).state && IsValid(id(outside_unit_defrost_compressor_time).state))
    {
      return id(outside_unit_defrost_compressor_time).state < CUSTOM_DEFROST_OUTDOOR_UNIT_LOCK_S;
    }
    return GetRuntimeSeconds() < CUSTOM_DEFROST_OUTDOOR_UNIT_LOCK_S;
  }

  uint32_t GetRuntimeSeconds()
  {
    return id(custom_defrost_runtime_s);
  }

  void UpdateRuntime(uint32_t dt_ms, bool defrost_active)
  {
    if (!IsCompressorRunning() || defrost_active || !IsWorkingModeIndex(WORKING_MODE_HEATING))
    {
      return;
    }

    runtime_remainder_ms_ += dt_ms;
    uint32_t seconds = runtime_remainder_ms_ / 1000;
    runtime_remainder_ms_ %= 1000;
    id(custom_defrost_runtime_s) += seconds;
    PublishRuntime();
  }

  /// @brief Tracks how long the compressor runs and counts its stops while heating (for the cycling route).
  void UpdateCompressorStops(uint32_t now)
  {
    if (IsCompressorRunning())
    {
      if (compressor_running_since_ms_ == 0)
      {
        compressor_running_since_ms_ = NonZero(now);
      }
      return;
    }
    if (compressor_running_since_ms_ != 0 && IsWorkingModeIndex(WORKING_MODE_HEATING))
    {
      compressor_stops_++;
    }
    compressor_running_since_ms_ = 0;
  }

  uint32_t GetCompressorOnSeconds(uint32_t now)
  {
    return compressor_running_since_ms_ == 0 ? 0 : (now - compressor_running_since_ms_) / 1000;
  }

  void PublishRuntime()
  {
    int minutes = (int)(id(custom_defrost_runtime_s) / 60);
    if (minutes != last_published_runtime_min_)
    {
      last_published_runtime_min_ = minutes;
      id(custom_defrost_runtime_sensor).publish_state(minutes);
    }
  }

  void PublishCount(uint32_t now)
  {
    int count = 0;
    for (int i = 0; i < HISTORY_SIZE; i++)
    {
      if (history_ms_[i] != 0 && (now - history_ms_[i]) < 24UL * 3600UL * 1000UL)
      {
        count++;
      }
    }
    if (count != last_published_count_)
    {
      last_published_count_ = count;
      id(custom_defrost_count_24h).publish_state(count);
    }
  }

  static void PublishRounded(sensor::Sensor* target, float value)
  {
    float rounded = roundf(value * 10.0f) / 10.0f;
    float current = target->state;
    if (std::isnan(rounded) != std::isnan(current) || (!std::isnan(rounded) && fabsf(current - rounded) >= 0.1f))
    {
      target->publish_state(rounded);
    }
  }

  void PublishCoilValues()
  {
    float ta = id(temperature_outside_ta).state;
    float tp = id(evaporation_temperature_tp).state;
    if (!IsValid(ta))
    {
      return;
    }
    PublishRounded(&id(custom_defrost_start_temperature), GetStartTemperature(ta));
    PublishRounded(&id(custom_defrost_frost_point), GetFrostPoint(ta));
    if (IsValid(tp))
    {
      PublishRounded(&id(custom_defrost_ta_tp), ta - tp);
    }
  }

  void ManageDefrostControlMode(int mode, uint32_t now)
  {
    uint32_t desired = mode == (int)CUSTOM_DEFROST_MODE_OPENAMBER ? DEFROST_CONTROL_BY_MASTER : DEFROST_CONTROL_BY_OUTDOOR_UNIT;
    auto idx = id(defrost_control_mode_select).active_index();
    if (idx.has_value() && idx.value() == desired)
    {
      return;
    }

    if (last_control_mode_write_ms_ != 0 && (now - last_control_mode_write_ms_) < CUSTOM_DEFROST_CONTROL_MODE_WRITE_INTERVAL_S * 1000UL)
    {
      return;
    }

    last_control_mode_write_ms_ = NonZero(now);
    ESP_LOGI("amber", "CustomDefrost: writing defrost control mode (3274) = %s", desired == DEFROST_CONTROL_BY_MASTER ? "master (OpenAmber)" : "outdoor unit");
    auto call = id(defrost_control_mode_select).make_call();
    call.set_index(desired);
    call.perform();
  }

  void PublishTrigger(uint32_t now, const std::string& text)
  {
    trigger_pulse_until_ms_ = NonZero(now + CUSTOM_DEFROST_TRIGGER_PULSE_S * 1000UL);
    id(custom_defrost_trigger_active).publish_state(true);
    id(custom_defrost_last_trigger).publish_state(text);
  }

  void RestoreHeatingWorkingMode()
  {
    if (IsWorkingModeIndex(WORKING_MODE_FORCED_DEFROST))
    {
      ESP_LOGI("amber", "CustomDefrost: restoring working mode to heating.");
      WriteWorkingMode(WORKING_MODE_HEATING);
    }
  }

  void FallbackToOutdoorUnit(const char* reason)
  {
    ESP_LOGE("amber", "CustomDefrost: %s, falling back to defrost control by the outdoor unit.", reason);
    id(warning_custom_defrost_failed).publish_state(true);
    request_pending_ = false;
    manual_request_ = false;
    request_attempts_ = 0;
    auto call = id(custom_defrost_mode_select).make_call();
    call.set_index(CUSTOM_DEFROST_MODE_OUTDOOR_UNIT);
    call.perform();
    previous_mode_ = CUSTOM_DEFROST_MODE_OUTDOOR_UNIT;
    SetState(CustomDefrostState::OFF);
    PublishStatus("Teruggevallen op buitenunit");
  }

  void OnDefrostStarted(uint32_t now)
  {
    defrost_started_ms_ = NonZero(now);
    overrun_warned_ = false;
    started_by_openamber_ = state_ == CustomDefrostState::REQUESTED;
    request_pending_ = false;
    manual_request_ = false;
    request_attempts_ = 0;

    history_ms_[history_index_] = NonZero(now);
    history_index_ = (history_index_ + 1) % HISTORY_SIZE;
    PublishCount(now);

    float tp = id(evaporation_temperature_tp).state;
    float ta = id(temperature_outside_ta).state;
    id(custom_defrost_start_tp).publish_state(tp);
    id(custom_defrost_start_ta).publish_state(ta);

    if (started_by_openamber_)
    {
      ESP_LOGI("amber", "CustomDefrost: defrost started by OpenAmber (Tp %.1f°C, Ta %.1f°C).", tp, ta);
    }
    else if (state_ == CustomDefrostState::DRY_RUN_TRIGGERED)
    {
      float delta_min = (float)(now - dry_run_trigger_ms_) / 60000.0f;
      id(custom_defrost_dry_run_delta).publish_state(roundf(delta_min * 10.0f) / 10.0f);
      ESP_LOGI("amber", "CustomDefrost dry-run: outdoor unit started defrost %.1f min after dry-run trigger (Tp %.1f°C, Ta %.1f°C).", delta_min, tp, ta);
    }
    else
    {
      if (GetMode() == (int)CUSTOM_DEFROST_MODE_DRY_RUN)
      {
        id(custom_defrost_dry_run_delta).publish_state(NAN);
        id(custom_defrost_last_trigger).publish_state("Unit ontdooide zonder dry-run trigger");
      }
      ESP_LOGI("amber", "CustomDefrost: defrost started by outdoor unit (Tp %.1f°C, Ta %.1f°C).", tp, ta);
    }

    dry_run_trigger_ms_ = 0;
    SetState(CustomDefrostState::DEFROSTING);
  }

  void OnDefrostEnded(uint32_t now)
  {
    if (defrost_started_ms_ != 0)
    {
      uint32_t duration_s = (now - defrost_started_ms_) / 1000;
      id(custom_defrost_last_duration).publish_state(duration_s);
      ESP_LOGI("amber", "CustomDefrost: defrost ended after %u s (%s).", (unsigned) duration_s, started_by_openamber_ ? "OpenAmber" : "outdoor unit");
      // Only a forced defrost has no pre-phase, so only then the duration is the real defrost time.
      if (started_by_openamber_)
      {
        AdaptInterval(duration_s);
      }
    }
    // The outdoor unit returns to heating by itself but leaves 3999 on 4, which would start a new defrost after 30 min.
    RestoreHeatingWorkingMode();
    defrost_started_ms_ = 0;
    started_by_openamber_ = false;
    id(custom_defrost_runtime_s) = 0;
    runtime_remainder_ms_ = 0;
    compressor_stops_ = 0;
    PublishRuntime();
    ResetTriggerTimers();
    forced_mode_seen_since_ms_ = 0;
    SetState(GetMode() == (int)CUSTOM_DEFROST_MODE_OUTDOOR_UNIT ? CustomDefrostState::OFF : CustomDefrostState::MONITORING);
  }

  /// @brief Backup: restores the working mode when the outdoor unit is still in forced defrost mode without a defrost.
  void CheckForcedModeRestore(uint32_t now, bool defrost_active)
  {
    if (defrost_active || state_ == CustomDefrostState::REQUESTED || !IsWorkingModeIndex(WORKING_MODE_FORCED_DEFROST))
    {
      forced_mode_seen_since_ms_ = 0;
      return;
    }

    if (forced_mode_seen_since_ms_ == 0)
    {
      forced_mode_seen_since_ms_ = NonZero(now);
      return;
    }

    if ((now - forced_mode_seen_since_ms_) >= CUSTOM_DEFROST_RESTORE_WORKING_MODE_DELAY_S * 1000UL)
    {
      ESP_LOGW("amber", "CustomDefrost: outdoor unit still in forced defrost mode without active defrost.");
      RestoreHeatingWorkingMode();
      forced_mode_seen_since_ms_ = 0;
    }
  }

  void CheckDefrostOverrun(uint32_t now, bool defrost_active)
  {
    if (!defrost_active || defrost_started_ms_ == 0 || overrun_warned_)
    {
      return;
    }
    float max_time_min = id(max_defrost_time).state;
    if (std::isnan(max_time_min))
    {
      max_time_min = 8.0f;
    }
    uint32_t limit_ms = (uint32_t)(max_time_min * 60000.0f) + CUSTOM_DEFROST_OVERRUN_MARGIN_S * 1000UL;
    if ((now - defrost_started_ms_) > limit_ms)
    {
      overrun_warned_ = true;
      ESP_LOGW("amber", "CustomDefrost: defrost is taking longer than %.0f min + margin, leaving it to the outdoor unit.", max_time_min);
    }
  }

  /// @brief Checks the three start routes. Returns the trigger reason, or an empty string with *waiting set to why not.
  std::string EvaluateTriggers(uint32_t now, std::string* waiting)
  {
    float ta = id(temperature_outside_ta).state;
    float tp = id(evaporation_temperature_tp).state;
    if (!IsValid(ta) || !IsValid(tp))
    {
      ResetTriggerTimers();
      *waiting = "geen Ta of Tp";
      return "";
    }

    char buffer[96];
    uint32_t runtime_min = GetRuntimeSeconds() / 60;
    uint32_t interval_min = GetIntervalMinutes();
    uint32_t compressor_on_s = GetCompressorOnSeconds(now);
    bool soft_start_done = compressor_on_s >= COMPRESSOR_SOFT_START_DURATION_S;

    // Safety net: long runtime with a cold coil, regardless of the start line and humidity.
    bool safety_net_cold = soft_start_done && runtime_min >= (uint32_t)id(custom_defrost_max_runtime).state && tp < id(custom_defrost_fallback_tp).state;
    if (HoldSeconds(safety_net_cold, &below_safety_net_since_ms_, now) >= CUSTOM_DEFROST_CONFIRM_S)
    {
      snprintf(buffer, sizeof(buffer), "Vangnet: %u min draaitijd, Tp %.1f°C", (unsigned) runtime_min, tp);
      return buffer;
    }

    // Normal route: Tp below the start line, compressor running continuously.
    float start_temperature = GetStartTemperature(ta);
    bool below_start_line = compressor_on_s >= CUSTOM_DEFROST_MIN_COMPRESSOR_ON_S && tp <= start_temperature;
    uint32_t below_start_line_s = HoldSeconds(below_start_line, &below_start_line_since_ms_, now);

    // Cycling route: many compressor stops, cold weather and a cold coil.
    bool cycling = compressor_stops_ >= CUSTOM_DEFROST_CYCLING_MIN_STOPS && ta <= CUSTOM_DEFROST_CYCLING_MAX_TA_C;
    bool below_max_coil = cycling && soft_start_done && tp <= id(custom_defrost_max_coil_temperature).state;
    uint32_t below_max_coil_s = HoldSeconds(below_max_coil, &below_max_coil_since_ms_, now);

    if (runtime_min < interval_min)
    {
      snprintf(buffer, sizeof(buffer), "interval %u / %u min", (unsigned) runtime_min, (unsigned) interval_min);
      *waiting = buffer;
      return "";
    }

    float frost_point = GetFrostPoint(ta);
    if (IsValid(frost_point) && tp > frost_point)
    {
      snprintf(buffer, sizeof(buffer), "droge lucht, Tp boven rijppunt %.1f°C", frost_point);
      *waiting = buffer;
      return "";
    }

    if (below_start_line_s >= CUSTOM_DEFROST_CONFIRM_S)
    {
      snprintf(buffer, sizeof(buffer), "Startlijn: Tp %.1f <= %.1f°C (Ta %.1f°C)", tp, start_temperature, ta);
      return buffer;
    }

    if (below_max_coil_s >= CUSTOM_DEFROST_CONFIRM_S)
    {
      snprintf(buffer, sizeof(buffer), "Pendelen: %u keer gestopt, Tp %.1f°C", (unsigned) compressor_stops_, tp);
      return buffer;
    }

    *waiting = "";
    return "";
  }

  /// @brief Returns the block reason or an empty string.
  std::string EvaluateBlocks(uint32_t now)
  {
    if (IsOutdoorUnitLocked())
    {
      return "slot buitenunit (30 min na defrost)";
    }

    float return_temperature = id(inlet_temperature_tui).state;
    float min_return_temperature = id(custom_defrost_min_return_temperature).state;
    if (IsValid(return_temperature) && return_temperature < min_return_temperature)
    {
      if (return_temperature_block_since_ms_ == 0)
      {
        return_temperature_block_since_ms_ = NonZero(now);
      }
      uint32_t max_wait_ms = (uint32_t)(id(custom_defrost_max_return_wait).state * 60000.0f);
      if ((now - return_temperature_block_since_ms_) < max_wait_ms)
      {
        char buffer[64];
        snprintf(buffer, sizeof(buffer), "retour %.1f < %.1f°C", return_temperature, min_return_temperature);
        return buffer;
      }
      ESP_LOGW("amber", "CustomDefrost: return temperature %.1f°C still below %.1f°C after maximum wait, defrosting anyway.", return_temperature, min_return_temperature);
    }
    else
    {
      return_temperature_block_since_ms_ = 0;
    }

    return "";
  }

  bool IsHeatingWithCompressor(bool defrost_active)
  {
    return IsCompressorRunning() && !defrost_active && IsWorkingModeIndex(WORKING_MODE_HEATING);
  }

public:
  CustomDefrostController() {}

  /// @brief Called every controller update, in every main state.
  void Update(bool modbus_connected)
  {
    uint32_t now = App.get_loop_component_start_time();
    uint32_t dt_ms = last_update_ms_ == 0 ? 0 : now - last_update_ms_;
    last_update_ms_ = NonZero(now);

    int mode = GetMode();
    bool defrost_active = id(defrost_active_sensor).state;

    if (mode != previous_mode_)
    {
      ESP_LOGI("amber", "CustomDefrost: mode changed to %s", id(custom_defrost_mode_select).current_option().c_str());
      if (mode == (int)CUSTOM_DEFROST_MODE_OPENAMBER)
      {
        id(warning_custom_defrost_failed).publish_state(false);
      }
      if (state_ == CustomDefrostState::REQUESTED && !defrost_active)
      {
        RestoreHeatingWorkingMode();
      }
      request_pending_ = false;
      manual_request_ = false;
      request_attempts_ = 0;
      dry_run_trigger_ms_ = 0;
      ResetTriggerTimers();
      if (!defrost_active)
      {
        SetState(mode == (int)CUSTOM_DEFROST_MODE_OUTDOOR_UNIT ? CustomDefrostState::OFF : CustomDefrostState::MONITORING);
      }
      previous_mode_ = mode;
    }

    CheckMinIntervalChanged();

    UpdateCompressorStops(now);
    UpdateRuntime(dt_ms, defrost_active);
    PublishCoilValues();
    PublishCount(now);

    if (trigger_pulse_until_ms_ != 0 && static_cast<int32_t>(now - trigger_pulse_until_ms_) >= 0)
    {
      trigger_pulse_until_ms_ = 0;
      id(custom_defrost_trigger_active).publish_state(false);
    }

    if (modbus_connected)
    {
      ManageDefrostControlMode(mode, now);
    }

    if (defrost_active && !defrost_active_previous_)
    {
      OnDefrostStarted(now);
    }
    else if (!defrost_active && defrost_active_previous_)
    {
      OnDefrostEnded(now);
    }
    defrost_active_previous_ = defrost_active;

    CheckForcedModeRestore(now, defrost_active);
    CheckDefrostOverrun(now, defrost_active);

    if (defrost_active)
    {
      PublishStatus(started_by_openamber_ ? "Ontdooien" : "Ontdooien (buitenunit)");
      return;
    }

    if (mode == (int)CUSTOM_DEFROST_MODE_OUTDOOR_UNIT)
    {
      SetState(CustomDefrostState::OFF);
      PublishStatus("Uit");
      return;
    }

    if (state_ == CustomDefrostState::REQUESTED)
    {
      uint32_t timeout_ms = (uint32_t)(id(custom_defrost_confirm_timeout).state * 1000.0f);
      if ((now - request_started_ms_) < timeout_ms)
      {
        PublishStatus("Aangevraagd");
        return;
      }

      request_attempts_++;
      ESP_LOGW("amber", "CustomDefrost: no defrost started within %.0f s after forced defrost request (attempt %u).", id(custom_defrost_confirm_timeout).state, (unsigned) request_attempts_);
      // Take the 4 back, otherwise the outdoor unit would still start the defrost later.
      RestoreHeatingWorkingMode();
      if (request_attempts_ >= CUSTOM_DEFROST_MAX_REQUEST_ATTEMPTS)
      {
        FallbackToOutdoorUnit("forced defrost was not confirmed by the outdoor unit");
        return;
      }
      // Retry on the next opportunity, the trigger timers keep running.
      SetState(CustomDefrostState::MONITORING);
      request_pending_ = true;
      PublishStatus("Opnieuw aanvragen");
      return;
    }

    if (state_ == CustomDefrostState::DRY_RUN_TRIGGERED)
    {
      if ((now - dry_run_trigger_ms_) >= CUSTOM_DEFROST_DRY_RUN_MATCH_WINDOW_S * 1000UL)
      {
        ESP_LOGI("amber", "CustomDefrost dry-run: no defrost by outdoor unit within %u min after trigger.", (unsigned) (CUSTOM_DEFROST_DRY_RUN_MATCH_WINDOW_S / 60));
        id(custom_defrost_last_trigger).publish_state(id(custom_defrost_last_trigger).state + " (geen defrost gevolgd)");
        dry_run_trigger_ms_ = 0;
        ResetTriggerTimers();
        SetState(CustomDefrostState::MONITORING);
      }
      else
      {
        PublishStatus("Dry-run: trigger");
        return;
      }
    }

    if (manual_request_)
    {
      if ((now - manual_request_ms_) >= CUSTOM_DEFROST_MANUAL_REQUEST_EXPIRY_S * 1000UL)
      {
        ESP_LOGW("amber", "CustomDefrost: manual defrost request expired.");
        manual_request_ = false;
        request_pending_ = false;
      }
      else if (IsOutdoorUnitLocked())
      {
        request_pending_ = false;
        PublishStatus("Handmatig: wacht op slot buitenunit");
        return;
      }
      else
      {
        request_pending_ = true;
        PublishStatus("Handmatig aangevraagd");
        return;
      }
    }

    if (!IsHeatingWithCompressor(defrost_active) || id(temperature_outside_ta).state > id(custom_defrost_max_ta).state)
    {
      ResetTriggerTimers();
      request_pending_ = false;
      SetState(CustomDefrostState::MONITORING);
      PublishStatus("Bewaken");
      return;
    }

    std::string waiting;
    std::string reason = EvaluateTriggers(now, &waiting);
    if (reason.empty())
    {
      request_pending_ = false;
      return_temperature_block_since_ms_ = 0;
      SetState(CustomDefrostState::MONITORING);
      PublishStatus(waiting.empty() ? std::string("Bewaken") : "Bewaken: " + waiting);
      return;
    }

    std::string block = EvaluateBlocks(now);
    if (!block.empty())
    {
      request_pending_ = false;
      SetState(CustomDefrostState::BLOCKED);
      PublishStatus("Geblokkeerd: " + block);
      return;
    }

    if (mode == (int)CUSTOM_DEFROST_MODE_DRY_RUN)
    {
      ESP_LOGI("amber", "CustomDefrost dry-run: would start defrost now (%s).", reason.c_str());
      PublishTrigger(now, "Dry-run: " + reason);
      dry_run_trigger_ms_ = NonZero(now);
      SetState(CustomDefrostState::DRY_RUN_TRIGGERED);
      PublishStatus("Dry-run: trigger");
      return;
    }

    if (!request_pending_)
    {
      ESP_LOGI("amber", "CustomDefrost: defrost wanted (%s).", reason.c_str());
    }
    request_pending_ = true;
    pending_reason_ = reason;
    SetState(CustomDefrostState::MONITORING);
    PublishStatus("Defrost gewenst");
  }

  /// @brief True when the heat/cool or DHW controller should start a forced defrost now.
  bool ShouldRequestDefrost()
  {
    if (!request_pending_ || state_ == CustomDefrostState::REQUESTED || GetMode() != (int)CUSTOM_DEFROST_MODE_OPENAMBER)
    {
      return false;
    }
    return IsHeatingWithCompressor(id(defrost_active_sensor).state);
  }

  /// @brief Writes working mode 4 (forced defrost). Called by the active controller after preparing pumps.
  void StartRequest()
  {
    uint32_t now = App.get_loop_component_start_time();
    std::string reason = manual_request_ ? std::string("Handmatig") : pending_reason_;
    ESP_LOGI("amber", "CustomDefrost: requesting forced defrost (%s).", reason.c_str());
    PublishTrigger(now, reason);
    request_pending_ = false;
    manual_request_ = false;
    request_started_ms_ = NonZero(now);
    SetState(CustomDefrostState::REQUESTED);
    PublishStatus("Aangevraagd");
    WriteWorkingMode(WORKING_MODE_FORCED_DEFROST);
  }

  bool IsRequestActive() const
  {
    return state_ == CustomDefrostState::REQUESTED;
  }

  bool IsDefrostOrRequestActive()
  {
    return state_ == CustomDefrostState::REQUESTED || id(defrost_active_sensor).state;
  }

  void RequestManualDefrost()
  {
    if (GetMode() != (int)CUSTOM_DEFROST_MODE_OPENAMBER)
    {
      ESP_LOGW("amber", "CustomDefrost: manual defrost only possible when defrost control is set to OpenAmber.");
      return;
    }
    if (state_ == CustomDefrostState::REQUESTED || id(defrost_active_sensor).state)
    {
      ESP_LOGW("amber", "CustomDefrost: defrost already requested or active.");
      return;
    }
    ESP_LOGI("amber", "CustomDefrost: manual defrost requested.");
    manual_request_ = true;
    manual_request_ms_ = NonZero(App.get_loop_component_start_time());
  }
};
