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
  REQUESTED,
  DEFROSTING,
};

/// @brief Frost point of the outdoor air, NAN without a valid humidity (1-100 %).
inline float CustomDefrostFrostPoint(float ta, float rh)
{
  if (std::isnan(ta) || std::isnan(rh) || rh < 1.0f || rh > 100.0f)
  {
    return NAN;
  }
  // Magnus formula: vapour pressure of the air (over water), then the temperature where that pressure saturates over ice.
  float vapour_pressure = rh / 100.0f * 6.112f * expf(17.62f * ta / (243.12f + ta));
  float l = logf(vapour_pressure / 6.112f);
  return 272.62f * l / (22.46f - l);
}

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
/// Deep frost: Tp 3 min at least 5 K below the start line defrosts without waiting for the interval (the 30 min lock
/// of the outdoor unit still applies).
/// Optional humidity sensor: when the coil is above the frost point of the air, no ice can grow,
/// so the normal and cycling route wait (the safety net does not).
///
/// Modes (defrost_control_select), switchable at any time:
///  - Buitenunit: the outdoor unit decides itself, only statistics are kept here.
///  - OpenAmber: EEPROM 3274 is set to master control and a defrost is started by writing working mode 4.
/// A running defrost always finishes; 3274 is only rewritten when no defrost is active.
/// The heat/cool and DHW controllers pick up a pending request in COMPRESSOR_RUNNING via ShouldRequestDefrost().
class CustomDefrostController
{
private:
  CustomDefrostState state_ = CustomDefrostState::OFF;
  uint32_t last_update_ms_ = 0;
  uint32_t runtime_remainder_ms_ = 0;
  uint32_t compressor_running_since_ms_ = 0;
  uint32_t compressor_stops_ = 0;
  uint32_t below_start_line_since_ms_ = 0;
  uint32_t below_max_coil_since_ms_ = 0;
  uint32_t below_safety_net_since_ms_ = 0;
  uint32_t below_deep_frost_since_ms_ = 0;
  uint32_t return_temperature_block_since_ms_ = 0;
  uint32_t request_started_ms_ = 0;
  uint32_t manual_request_ms_ = 0;
  uint32_t defrost_started_ms_ = 0;
  uint32_t forced_mode_seen_since_ms_ = 0;
  uint32_t last_control_mode_write_ms_ = 0;
  uint32_t request_attempts_ = 0;
  uint32_t failed_since_ms_ = 0;
  uint32_t control_mode_writes_ = 0;
  int last_published_runtime_min_ = -1;
  int previous_mode_ = -1;
  float previous_min_interval_ = NAN;
  bool defrost_active_previous_ = false;
  bool request_pending_ = false;
  bool manual_request_ = false;
  bool started_by_openamber_ = false;
  bool overrun_warned_ = false;
  bool control_mode_write_failed_logged_ = false;
  bool dry_air_delay_logged_ = false;
  std::string pending_reason_;

  // Measurement of the current defrost (see OnDefrostStarted / TrackDefrost / OnDefrostEnded).
  float tp_samples_[CUSTOM_DEFROST_TP_SAMPLES] = {NAN, NAN, NAN, NAN, NAN, NAN};
  int tp_sample_index_ = 0;
  uint32_t last_tp_sample_ms_ = 0;
  uint32_t execution_started_ms_ = 0;
  uint32_t melt_ms_ = 0;
  float energy_kj_ = 0.0f;
  float event_ta_ = NAN;
  float event_tp_ = NAN;
  float event_start_line_ = NAN;
  float event_humidity_ = NAN;
  float event_frost_point_ = NAN;
  float event_runtime_min_ = NAN;
  uint32_t event_stops_ = 0;
  int event_source_ = 0;
  std::string last_status_;

  int GetMode()
  {
    auto idx = id(defrost_control_select).active_index();
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
    below_deep_frost_since_ms_ = 0;
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

  /// @brief Frost point of the outdoor air, or NAN without a valid humidity value (no sensor: humidity is ignored).
  float GetFrostPoint(float ta)
  {
    return CustomDefrostFrostPoint(ta, id(outdoor_humidity).state);
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
    if (IsValid(tp))
    {
      PublishRounded(&id(custom_defrost_ta_tp), ta - tp);
    }
  }

  void ManageDefrostControlMode(int mode, uint32_t now)
  {
    uint32_t desired = mode == (int)CUSTOM_DEFROST_MODE_OPENAMBER ? DEFROST_CONTROL_BY_MASTER : DEFROST_CONTROL_BY_OUTDOOR_UNIT;
    auto idx = id(defrost_control_mode_select).active_index();
    // Only write after a valid readback, so a restart does not cause an EEPROM write before 3274 was read.
    if (!idx.has_value())
    {
      return;
    }
    if (idx.value() == desired)
    {
      control_mode_writes_ = 0;
      control_mode_write_failed_logged_ = false;
      return;
    }

    if (last_control_mode_write_ms_ != 0 && (now - last_control_mode_write_ms_) < CUSTOM_DEFROST_CONTROL_MODE_WRITE_INTERVAL_S * 1000UL)
    {
      return;
    }

    // 3274 is an EEPROM parameter: stop rewriting it when the readback never follows, to spare the EEPROM.
    if (control_mode_writes_ >= CUSTOM_DEFROST_MAX_CONTROL_MODE_WRITES)
    {
      if (!control_mode_write_failed_logged_)
      {
        control_mode_write_failed_logged_ = true;
        ESP_LOGE("amber", "CustomDefrost: defrost control mode (3274) does not follow after %u writes, giving up until the mode changes.", (unsigned) control_mode_writes_);
      }
      return;
    }

    control_mode_writes_++;
    last_control_mode_write_ms_ = NonZero(now);
    ESP_LOGI("amber", "CustomDefrost: writing defrost control mode (3274) = %s", desired == DEFROST_CONTROL_BY_MASTER ? "master (OpenAmber)" : "outdoor unit");
    auto call = id(defrost_control_mode_select).make_call();
    call.set_index(desired);
    call.perform();
  }

  void PublishTrigger(uint32_t now, const std::string& text)
  {
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

  /// @brief Control stays with OpenAmber: raise the warning and try again after the retry delay.
  void OnRequestFailed()
  {
    ESP_LOGE("amber", "CustomDefrost: forced defrost was not confirmed by the outdoor unit, retrying in %u min.",
             (unsigned) (CUSTOM_DEFROST_FAILED_RETRY_S / 60));
    id(warning_custom_defrost_failed).publish_state(true);
    request_pending_ = false;
    manual_request_ = false;
    request_attempts_ = 0;
    failed_since_ms_ = NonZero(App.get_loop_component_start_time());
    ResetTriggerTimers();
    SetState(CustomDefrostState::MONITORING);
  }

  /// @brief Keeps one Tp sample per 10 s, so the lowest Tp of the last minute is known when the hot gas arrives.
  void SampleTp(uint32_t now)
  {
    if (last_tp_sample_ms_ != 0 && (now - last_tp_sample_ms_) < CUSTOM_DEFROST_TP_SAMPLE_S * 1000UL)
    {
      return;
    }
    last_tp_sample_ms_ = NonZero(now);
    tp_samples_[tp_sample_index_] = id(evaporation_temperature_tp).state;
    tp_sample_index_ = (tp_sample_index_ + 1) % CUSTOM_DEFROST_TP_SAMPLES;
  }

  float GetRecentMinTp()
  {
    float result = NAN;
    for (float sample : tp_samples_)
    {
      if (IsValid(sample) && (!IsValid(result) || sample < result))
      {
        result = sample;
      }
    }
    return result;
  }

  /// @brief Measures the running defrost: start of the execution, melt time and the heat taken from the water.
  void TrackDefrost(uint32_t now, uint32_t dt_ms)
  {
    float tp = id(evaporation_temperature_tp).state;
    if (execution_started_ms_ == 0)
    {
      float min_tp = GetRecentMinTp();
      if (IsValid(tp) && IsValid(min_tp) && tp >= min_tp + CUSTOM_DEFROST_EXECUTION_TP_RISE_K)
      {
        execution_started_ms_ = NonZero(now);
        ESP_LOGI("amber", "CustomDefrost: execution started after %u s (Tp %.1f°C, was %.1f°C).",
                 (unsigned) ((now - defrost_started_ms_) / 1000), tp, min_tp);
      }
      else
      {
        // Keep sampling during the outdoor unit's pre-phase, the compressor still heats normally then.
        SampleTp(now);
      }
      return;
    }

    if (IsValid(tp) && tp >= CUSTOM_DEFROST_MELT_MIN_C && tp <= CUSTOM_DEFROST_MELT_MAX_C)
    {
      melt_ms_ += dt_ms;
    }

    // The defrost takes its heat from the water: flow (l/min) x (Tui - Tuo).
    float flow = id(water_flow_rate).state;
    float delta = id(inlet_temperature_tui).state - id(outlet_temperature_tuo).state;
    if (IsValid(flow) && IsValid(delta) && flow > 0.0f && delta > 0.0f)
    {
      energy_kj_ += flow / 60.0f * 0.998f * 4.19f * delta * (float) dt_ms / 1000.0f;
    }
  }

  void OnDefrostStarted(uint32_t now)
  {
    defrost_started_ms_ = NonZero(now);
    overrun_warned_ = false;
    started_by_openamber_ = state_ == CustomDefrostState::REQUESTED;
    request_pending_ = false;
    manual_request_ = false;
    request_attempts_ = 0;

    float tp = id(evaporation_temperature_tp).state;
    float ta = id(temperature_outside_ta).state;

    execution_started_ms_ = 0;
    melt_ms_ = 0;
    energy_kj_ = 0.0f;
    event_ta_ = ta;
    event_tp_ = tp;
    event_start_line_ = IsValid(ta) ? GetStartTemperature(ta) : NAN;
    event_humidity_ = id(outdoor_humidity).state;
    event_frost_point_ = IsValid(ta) ? GetFrostPoint(ta) : NAN;
    event_runtime_min_ = GetRuntimeSeconds() / 60.0f;
    event_stops_ = compressor_stops_;
    event_source_ = started_by_openamber_ ? 1 : 0;
    failed_since_ms_ = 0;
    if (started_by_openamber_)
    {
      id(warning_custom_defrost_failed).publish_state(false);
    }

    ESP_LOGI("amber", "CustomDefrost: defrost started by %s (Tp %.1f°C, Ta %.1f°C).",
             started_by_openamber_ ? "OpenAmber" : "outdoor unit", tp, ta);

    SetState(CustomDefrostState::DEFROSTING);
  }

  void OnDefrostEnded(uint32_t now)
  {
    if (defrost_started_ms_ != 0)
    {
      // 2118 also covers the outdoor unit's pre-phase (about 5 min, firmware dependent), so the execution time is
      // measured from the detected Tp rise. Without that detection only a forced defrost (no pre-phase) has a duration.
      float total_s = (float) ((now - defrost_started_ms_) / 1000);
      float execution_s = execution_started_ms_ != 0 ? (float) ((now - execution_started_ms_) / 1000)
                                                     : (started_by_openamber_ ? total_s : NAN);
      float melt_s = execution_started_ms_ != 0 ? (float) (melt_ms_ / 1000) : NAN;
      float energy_kj = execution_started_ms_ != 0 ? roundf(energy_kj_) : NAN;
      id(custom_defrost_last_duration).publish_state(execution_s);
      ESP_LOGI("amber", "CustomDefrost: defrost ended (%s): 2118 %.0f s, execution %.0f s, melt %.0f s, %.0f kJ.",
               started_by_openamber_ ? "OpenAmber" : "outdoor unit", total_s, execution_s, melt_s, energy_kj);
      // Only adapt the interval to a defrost OpenAmber started itself.
      if (started_by_openamber_ && IsValid(execution_s))
      {
        AdaptInterval((uint32_t) execution_s);
      }
      id(send_defrost_event).execute(event_source_, total_s, execution_s, melt_s, energy_kj, event_ta_, event_tp_,
                                     event_start_line_, event_humidity_, event_frost_point_, event_runtime_min_,
                                     (int) event_stops_);
    }
    // The outdoor unit returns to heating by itself but leaves 3999 on 4, which would start a new defrost after 30 min.
    RestoreHeatingWorkingMode();
    defrost_started_ms_ = 0;
    started_by_openamber_ = false;
    id(custom_defrost_runtime_s) = 0;
    runtime_remainder_ms_ = 0;
    compressor_stops_ = 0;
    dry_air_delay_logged_ = false;
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
    bool safety_net_cold = soft_start_done && runtime_min >= CUSTOM_DEFROST_SAFETY_NET_RUNTIME_MIN && tp < CUSTOM_DEFROST_SAFETY_NET_TP_C;
    if (HoldSeconds(safety_net_cold, &below_safety_net_since_ms_, now) >= CUSTOM_DEFROST_CONFIRM_S)
    {
      snprintf(buffer, sizeof(buffer), "Vangnet: %u min draaitijd, Tp %.1f°C", (unsigned) runtime_min, tp);
      return buffer;
    }

    // Normal route: Tp below the start line, compressor running continuously.
    float start_temperature = GetStartTemperature(ta);
    bool below_start_line = compressor_on_s >= CUSTOM_DEFROST_MIN_COMPRESSOR_ON_S && tp <= start_temperature;
    uint32_t below_start_line_s = HoldSeconds(below_start_line, &below_start_line_since_ms_, now);

    // Deep frost: far below the start line the coil is clearly iced, so don't wait for the interval or the humidity.
    float deep_frost_temperature = start_temperature - CUSTOM_DEFROST_DEEP_FROST_K;
    bool deep_frost = below_start_line && tp <= deep_frost_temperature;
    if (HoldSeconds(deep_frost, &below_deep_frost_since_ms_, now) >= CUSTOM_DEFROST_CONFIRM_S)
    {
      snprintf(buffer, sizeof(buffer), "Diepe bevriezing: Tp %.1f <= %.1f°C (Ta %.1f°C)", tp, deep_frost_temperature, ta);
      return buffer;
    }

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

    // Dry air: the coil must be at least the margin below the frost point before ice can grow. The delay is
    // limited, so a wrong or drifting humidity sensor cannot postpone defrosts for long.
    float frost_point = GetFrostPoint(ta);
    if (IsValid(frost_point))
    {
      float margin = id(custom_defrost_frost_point_margin).state;
      float icing_limit = frost_point - (IsValid(margin) ? margin : 0.0f);
      bool delay_passed = runtime_min >= interval_min + CUSTOM_DEFROST_HUMIDITY_MAX_DELAY_MIN;
      if (tp > icing_limit)
      {
        if (!delay_passed)
        {
          snprintf(buffer, sizeof(buffer), "droge lucht, Tp boven %.1f°C (rijppunt %.1f°C)", icing_limit, frost_point);
          *waiting = buffer;
          return "";
        }
        if (!dry_air_delay_logged_)
        {
          dry_air_delay_logged_ = true;
          ESP_LOGW("amber", "CustomDefrost: dry air delayed the defrost for %u min, ignoring the humidity now.", (unsigned) CUSTOM_DEFROST_HUMIDITY_MAX_DELAY_MIN);
        }
      }
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
    if (failed_since_ms_ != 0)
    {
      uint32_t elapsed_s = (now - failed_since_ms_) / 1000;
      if (elapsed_s < CUSTOM_DEFROST_FAILED_RETRY_S)
      {
        char buffer[64];
        snprintf(buffer, sizeof(buffer), "mislukt, opnieuw over %u min", (unsigned) ((CUSTOM_DEFROST_FAILED_RETRY_S - elapsed_s + 59) / 60));
        return buffer;
      }
      failed_since_ms_ = 0;
    }

    if (IsOutdoorUnitLocked())
    {
      return "slot buitenunit (30 min na defrost)";
    }

    float return_temperature = id(inlet_temperature_tui).state;
    float min_return_temperature = CUSTOM_DEFROST_MIN_RETURN_TEMPERATURE_C;
    if (IsValid(return_temperature) && return_temperature < min_return_temperature)
    {
      if (return_temperature_block_since_ms_ == 0)
      {
        return_temperature_block_since_ms_ = NonZero(now);
      }
      uint32_t max_wait_ms = CUSTOM_DEFROST_MAX_RETURN_WAIT_S * 1000UL;
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
      ESP_LOGI("amber", "CustomDefrost: mode changed to %s", id(defrost_control_select).current_option().c_str());
      id(warning_custom_defrost_failed).publish_state(false);
      failed_since_ms_ = 0;
      if (state_ == CustomDefrostState::REQUESTED && !defrost_active)
      {
        RestoreHeatingWorkingMode();
      }
      request_pending_ = false;
      manual_request_ = false;
      request_attempts_ = 0;
      control_mode_writes_ = 0;
      control_mode_write_failed_logged_ = false;
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

    // Switching during a defrost: let the defrost finish first, then hand 3274 over.
    if (modbus_connected && !defrost_active)
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
      TrackDefrost(now, dt_ms);
    }
    else
    {
      SampleTp(now);
    }

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
      uint32_t timeout_ms = CUSTOM_DEFROST_CONFIRM_TIMEOUT_S * 1000UL;
      if ((now - request_started_ms_) < timeout_ms)
      {
        PublishStatus("Aangevraagd");
        return;
      }

      request_attempts_++;
      ESP_LOGW("amber", "CustomDefrost: no defrost started within %u s after forced defrost request (attempt %u).", (unsigned) CUSTOM_DEFROST_CONFIRM_TIMEOUT_S, (unsigned) request_attempts_);
      // Take the 4 back, otherwise the outdoor unit would still start the defrost later.
      RestoreHeatingWorkingMode();
      if (request_attempts_ >= CUSTOM_DEFROST_MAX_REQUEST_ATTEMPTS)
      {
        OnRequestFailed();
        PublishStatus("Mislukt, wacht op nieuwe poging");
        return;
      }
      // Retry on the next opportunity, the trigger timers keep running.
      SetState(CustomDefrostState::MONITORING);
      request_pending_ = true;
      PublishStatus("Opnieuw aanvragen");
      return;
    }

    if (manual_request_)
    {
      if ((now - manual_request_ms_) >= CUSTOM_DEFROST_MANUAL_REQUEST_EXPIRY_S * 1000UL)
      {
        ESP_LOGW("amber", "CustomDefrost: manual defrost request expired.");
        manual_request_ = false;
        request_pending_ = false;
      }
      else if (!IsCompressorRunning())
      {
        ESP_LOGW("amber", "CustomDefrost: compressor stopped, manual defrost request cancelled.");
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

    if (!IsHeatingWithCompressor(defrost_active) || id(temperature_outside_ta).state > CUSTOM_DEFROST_MAX_TA_C)
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
    if (!IsHeatingWithCompressor(false))
    {
      ESP_LOGW("amber", "CustomDefrost: manual defrost only possible while heating with the compressor running.");
      return;
    }
    ESP_LOGI("amber", "CustomDefrost: manual defrost requested.");
    manual_request_ = true;
    manual_request_ms_ = NonZero(App.get_loop_component_start_time());
  }
};
