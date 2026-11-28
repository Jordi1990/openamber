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

/// @brief Saturation vapour pressure over water at t (°C), in Pa (Magnus formula).
inline float CustomDefrostVapourPressureWater(float t)
{
  return 611.2f * expf(17.62f * t / (243.12f + t));
}

/// @brief Saturation vapour pressure over ice at t (°C), in Pa (Magnus formula).
inline float CustomDefrostVapourPressureIce(float t)
{
  return 611.2f * expf(22.46f * t / (272.62f + t));
}

inline bool CustomDefrostValidHumidity(float rh)
{
  return !std::isnan(rh) && rh >= 1.0f && rh <= 100.0f;
}

/// @brief Vapour pressure excess of the outdoor air over ice at the coil temperature, in Pa.
/// This drives frost growth on the coil: 0 when the coil is above the frost point, NAN without a valid humidity.
inline float CustomDefrostIcingRate(float ta, float rh, float tp)
{
  if (std::isnan(ta) || std::isnan(tp) || !CustomDefrostValidHumidity(rh))
  {
    return NAN;
  }
  return std::max(0.0f, rh / 100.0f * CustomDefrostVapourPressureWater(ta) - CustomDefrostVapourPressureIce(tp));
}

/// @brief Decides when the outdoor unit should defrost.
///
/// Without humidity there is one start route: after the adaptive interval (at least the outdoor unit's 30 min lock)
/// with Tp below -2 °C for 3 min. The interval grows after short defrosts and shrinks after long ones, like the outdoor
/// unit's Smart mode, so wet weather defrosts more often than dry weather. It is at most 120 min.
/// With a valid outdoor humidity (BLE sensor) the ice counter does the work: the vapour pressure excess of the air over
/// ice at Tp, summed over the compressor time since the last defrost (Pa·h), defrosts at its threshold after the minimum
/// interval. With the compressor stopped and the coil above +1 °C for 10 min the frost has melted and the counter goes
/// to 0. The interval route then only acts after 4 h compressor time, as an emergency limit for a wrong sensor value.
/// There is no start line on Ta - Tp (Mitsubishi SRK-ZSX): on the Amber Ta - Tp stayed 2.4-3.6 K with and without frost
/// (camera, 27/28 Nov 2026), so it does not show ice.
///
/// Modes (defrost_control_select), switchable at any time:
///  - Buitenunit: the outdoor unit decides itself, only statistics are kept here.
///  - OpenAmber: 3274 is set to master control and a defrost is started by writing working mode 4.
/// The mode is restored after a restart. 3274 is written once after the start, after every mode change and after the Modbus
/// connection with the outdoor unit returns (a power cycle of the unit loses it), each time only when the readback
/// differs and never during a running defrost.
/// When the outdoor unit does not start a requested defrost within the confirmation time, working mode 4 is taken back and
/// error TO10 (error_defrost_start_timeout) stops OpenAmber like the other timeout errors; no new request until it is reset.
/// The heat/cool and DHW controllers pick up a pending request in COMPRESSOR_RUNNING via ShouldRequestDefrost().
class CustomDefrostController
{
private:
  /// @brief Conditions at the start of a defrost, sent with the defrost event when it ends.
  struct DefrostEvent
  {
    float ta = NAN;
    float tp = NAN;
    float humidity = NAN;
    float runtime_min = NAN;
    float ice_load = NAN;
    int source = 0;
  };

  uint32_t last_update_ms_ = 0;
  uint32_t runtime_remainder_ms_ = 0;
  uint32_t compressor_running_since_ms_ = 0;
  uint32_t below_interval_tp_since_ms_ = 0;
  uint32_t ice_melt_since_ms_ = 0;
  uint32_t return_temperature_block_since_ms_ = 0;
  // Non-zero while working mode 4 was written and the outdoor unit has not started the defrost yet.
  uint32_t request_started_ms_ = 0;
  uint32_t defrost_started_ms_ = 0;
  uint32_t forced_mode_seen_since_ms_ = 0;
  int last_published_runtime_min_ = -1;
  int previous_mode_ = -1;
  int last_published_interval_min_ = -1;
  bool defrost_active_previous_ = false;
  bool modbus_connected_previous_ = false;
  // Set at the start, on a mode change and when the Modbus connection returns: 3274 is checked and written once.
  bool control_mode_write_pending_ = true;
  bool request_pending_ = false;
  bool manual_request_ = false;
  bool started_by_openamber_ = false;
  std::string pending_reason_;

  // Measurement of the current defrost (see OnDefrostStarted / TrackDefrost / OnDefrostEnded).
  float tp_samples_[CUSTOM_DEFROST_TP_SAMPLES] = {NAN, NAN, NAN, NAN, NAN, NAN};
  int tp_sample_index_ = 0;
  uint32_t last_tp_sample_ms_ = 0;
  uint32_t execution_started_ms_ = 0;
  uint32_t melt_ms_ = 0;
  float energy_kj_ = 0.0f;
  DefrostEvent event_;
  float last_published_ice_load_ = NAN;
  std::string last_status_;

  int GetMode()
  {
    auto idx = id(defrost_control_select).active_index();
    return idx.has_value() ? (int)idx.value() : (int)CUSTOM_DEFROST_MODE_OUTDOOR_UNIT;
  }

  bool IsOpenAmberMode()
  {
    return GetMode() == (int)CUSTOM_DEFROST_MODE_OPENAMBER;
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

  bool IsHeatingWithCompressor(bool defrost_active)
  {
    return IsCompressorRunning() && !defrost_active && IsWorkingModeIndex(WORKING_MODE_HEATING);
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

  void ClearRequest()
  {
    request_pending_ = false;
    manual_request_ = false;
  }

  void ResetTriggerTimers()
  {
    below_interval_tp_since_ms_ = 0;
    return_temperature_block_since_ms_ = 0;
  }

  // ── Interval between defrosts ──

  uint32_t GetMinIntervalMinutes()
  {
    uint32_t setting = (uint32_t)id(custom_defrost_min_runtime).state;
    return std::min(std::max(setting, CUSTOM_DEFROST_OUTDOOR_UNIT_LOCK_S / 60), CUSTOM_DEFROST_INTERVAL_MAX_MIN);
  }

  uint32_t ClampInterval(uint32_t interval)
  {
    return std::min(std::max(interval, GetMinIntervalMinutes()), CUSTOM_DEFROST_INTERVAL_MAX_MIN);
  }

  uint32_t GetIntervalMinutes()
  {
    return ClampInterval(id(custom_defrost_interval_min));
  }

  /// @brief Publishes the interval when it changes (also when a changed minimum interval setting clamps it).
  void PublishInterval()
  {
    int minutes = (int) GetIntervalMinutes();
    if (minutes != last_published_interval_min_)
    {
      last_published_interval_min_ = minutes;
      id(custom_defrost_interval_sensor).publish_state(minutes);
    }
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
    id(custom_defrost_interval_min) = ClampInterval(interval);
    PublishInterval();
    ESP_LOGI("amber", "CustomDefrost: defrost took %u s, interval is now %u min.", (unsigned) duration_s, (unsigned) GetIntervalMinutes());
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
    if (!IsHeatingWithCompressor(defrost_active))
    {
      return;
    }

    runtime_remainder_ms_ += dt_ms;
    uint32_t seconds = runtime_remainder_ms_ / 1000;
    runtime_remainder_ms_ %= 1000;
    id(custom_defrost_runtime_s) += seconds;
    PublishRuntime();
  }

  // ── Ice counter (optional humidity sensor) ──

  /// @brief Sums the icing rate over the heating compressor time since the last defrost (Pa·h). With the compressor
  /// stopped and the coil above +1 °C for 10 min the frost has melted, so the counter goes back to 0.
  void UpdateIceLoad(uint32_t dt_ms, bool defrost_active, uint32_t now)
  {
    float tp = id(evaporation_temperature_tp).state;
    bool melting = !IsCompressorRunning() && !defrost_active && IsValid(tp) && tp > CUSTOM_DEFROST_ICE_MELT_TP_C;
    if (HoldSeconds(melting, &ice_melt_since_ms_, now) >= CUSTOM_DEFROST_ICE_MELT_S && id(custom_defrost_ice_load) > 0.0f)
    {
      ESP_LOGI("amber", "CustomDefrost: coil above %.0f°C with the compressor off, ice counter %.0f -> 0 Pa·h.",
               CUSTOM_DEFROST_ICE_MELT_TP_C, id(custom_defrost_ice_load));
      id(custom_defrost_ice_load) = 0.0f;
      PublishIceLoad();
    }
    if (dt_ms == 0 || !IsHeatingWithCompressor(defrost_active))
    {
      return;
    }
    float rate = CustomDefrostIcingRate(id(temperature_outside_ta).state, id(outdoor_humidity).state,
                                        id(evaporation_temperature_tp).state);
    if (!IsValid(rate))
    {
      return;
    }
    id(custom_defrost_ice_load) += rate * (float) dt_ms / 3600000.0f;
    PublishIceLoad();
  }

  void PublishIceLoad()
  {
    float value = roundf(id(custom_defrost_ice_load));
    if (value != last_published_ice_load_)
    {
      last_published_ice_load_ = value;
      id(custom_defrost_ice_load_sensor).publish_state(value);
    }
  }

  /// @brief How long the compressor runs without interruption, in seconds.
  uint32_t GetCompressorOnSeconds(uint32_t now)
  {
    return HoldSeconds(IsCompressorRunning(), &compressor_running_since_ms_, now);
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

  /// @brief Puts 3274 on the control that matches the mode, once per request (see control_mode_write_pending_). A Modbus
  /// write only changes the outdoor unit's RAM; the unit loses it after a power cycle.
  void ManageDefrostControlMode()
  {
    uint32_t desired = IsOpenAmberMode() ? DEFROST_CONTROL_BY_MASTER : DEFROST_CONTROL_BY_OUTDOOR_UNIT;
    auto idx = id(defrost_control_mode_select).active_index();
    // Wait for a valid readback, so nothing is written before 3274 was read.
    if (!idx.has_value())
    {
      return;
    }
    control_mode_write_pending_ = false;
    if (idx.value() == desired)
    {
      return;
    }
    ESP_LOGI("amber", "CustomDefrost: writing defrost control mode (3274) = %s", desired == DEFROST_CONTROL_BY_MASTER ? "master (OpenAmber)" : "outdoor unit");
    auto call = id(defrost_control_mode_select).make_call();
    call.set_index(desired);
    call.perform();
  }

  void RestoreHeatingWorkingMode()
  {
    if (IsWorkingModeIndex(WORKING_MODE_FORCED_DEFROST))
    {
      ESP_LOGI("amber", "CustomDefrost: restoring working mode to heating.");
      WriteWorkingMode(WORKING_MODE_HEATING);
    }
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
    started_by_openamber_ = request_started_ms_ != 0;
    request_started_ms_ = 0;
    ClearRequest();

    float tp = id(evaporation_temperature_tp).state;
    float ta = id(temperature_outside_ta).state;

    execution_started_ms_ = 0;
    melt_ms_ = 0;
    energy_kj_ = 0.0f;
    event_ = DefrostEvent{};
    event_.ta = ta;
    event_.tp = tp;
    event_.humidity = id(outdoor_humidity).state;
    event_.runtime_min = GetRuntimeSeconds() / 60.0f;
    event_.ice_load = id(custom_defrost_ice_load);
    event_.source = started_by_openamber_ ? 1 : 0;

    ESP_LOGI("amber", "CustomDefrost: defrost started by %s (Tp %.1f°C, Ta %.1f°C).",
             started_by_openamber_ ? "OpenAmber" : "outdoor unit", tp, ta);
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
      id(send_defrost_event).execute(event_.source, total_s, execution_s, melt_s, energy_kj, event_.ta, event_.tp,
                                     event_.humidity, event_.runtime_min, event_.ice_load);
    }
    // The outdoor unit returns to heating by itself but leaves 3999 on 4, which would start a new defrost after 30 min.
    RestoreHeatingWorkingMode();
    defrost_started_ms_ = 0;
    started_by_openamber_ = false;
    id(custom_defrost_runtime_s) = 0;
    id(custom_defrost_ice_load) = 0.0f;
    runtime_remainder_ms_ = 0;
    PublishRuntime();
    PublishIceLoad();
    ResetTriggerTimers();
    forced_mode_seen_since_ms_ = 0;
  }

  /// @brief Backup: restores the working mode when the outdoor unit is still in forced defrost mode without a defrost.
  void CheckForcedModeRestore(uint32_t now, bool defrost_active)
  {
    bool stuck = !defrost_active && request_started_ms_ == 0 && IsWorkingModeIndex(WORKING_MODE_FORCED_DEFROST);
    if (HoldSeconds(stuck, &forced_mode_seen_since_ms_, now) >= CUSTOM_DEFROST_RESTORE_WORKING_MODE_DELAY_S)
    {
      ESP_LOGW("amber", "CustomDefrost: outdoor unit still in forced defrost mode without active defrost.");
      RestoreHeatingWorkingMode();
      forced_mode_seen_since_ms_ = 0;
    }
  }

  /// @brief Checks the start route. Returns the trigger reason, or an empty string with *waiting set to why not.
  std::string EvaluateTriggers(uint32_t now, std::string* waiting)
  {
    float tp = id(evaporation_temperature_tp).state;
    if (!IsValid(tp))
    {
      ResetTriggerTimers();
      *waiting = "geen Tp";
      return "";
    }

    char buffer[96];
    uint32_t runtime_min = GetRuntimeSeconds() / 60;
    // With a humidity value the ice counter does the work; the interval route is then only an emergency limit (4 h).
    bool humidity = CustomDefrostValidHumidity(id(outdoor_humidity).state);
    uint32_t interval_min = humidity ? CUSTOM_DEFROST_HUMIDITY_EMERGENCY_MIN : GetIntervalMinutes();

    // Once the interval has passed, a coil below -2 °C for 3 min (after the compressor's soft start) defrosts.
    bool interval_cold = GetCompressorOnSeconds(now) >= COMPRESSOR_SOFT_START_DURATION_S && tp < CUSTOM_DEFROST_INTERVAL_TP_C;
    uint32_t interval_cold_s = HoldSeconds(interval_cold, &below_interval_tp_since_ms_, now);

    // Ice counter: enough frost has grown according to the humidity, after the minimum interval.
    float ice_threshold = id(custom_defrost_ice_threshold).state;
    float ice_load = id(custom_defrost_ice_load);
    if (humidity && runtime_min >= GetMinIntervalMinutes() && IsValid(ice_threshold) && ice_threshold > 0.0f &&
        ice_load >= ice_threshold)
    {
      snprintf(buffer, sizeof(buffer), "IJsteller: %.0f >= %.0f Pa·h (Tp %.1f°C)", ice_load, ice_threshold, tp);
      return buffer;
    }

    if (runtime_min < interval_min)
    {
      snprintf(buffer, sizeof(buffer), "interval %u / %u min", (unsigned) runtime_min, (unsigned) interval_min);
      *waiting = buffer;
      return "";
    }

    if (interval_cold_s >= CUSTOM_DEFROST_CONFIRM_S)
    {
      snprintf(buffer, sizeof(buffer), "Interval: %u min draaitijd, Tp %.1f°C", (unsigned) runtime_min, tp);
      return buffer;
    }

    *waiting = "";
    return "";
  }

  /// @brief Returns the block reason or an empty string.
  std::string EvaluateBlocks(uint32_t now)
  {
    char buffer[64];
    if (IsOutdoorUnitLocked())
    {
      return "slot buitenunit (30 min na defrost)";
    }

    float return_temperature = id(inlet_temperature_tui).state;
    bool too_cold = IsValid(return_temperature) && return_temperature < CUSTOM_DEFROST_MIN_RETURN_TEMPERATURE_C;
    if (too_cold)
    {
      if (HoldSeconds(true, &return_temperature_block_since_ms_, now) < CUSTOM_DEFROST_MAX_RETURN_WAIT_S)
      {
        snprintf(buffer, sizeof(buffer), "retour %.1f < %.1f°C", return_temperature, CUSTOM_DEFROST_MIN_RETURN_TEMPERATURE_C);
        return buffer;
      }
      ESP_LOGW("amber", "CustomDefrost: return temperature %.1f°C still below %.1f°C after maximum wait, defrosting anyway.", return_temperature, CUSTOM_DEFROST_MIN_RETURN_TEMPERATURE_C);
    }
    else
    {
      return_temperature_block_since_ms_ = 0;
    }

    return "";
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
    // Set again below when a defrost is wanted now.
    request_pending_ = false;

    if (mode != previous_mode_)
    {
      ESP_LOGI("amber", "CustomDefrost: mode changed to %s", id(defrost_control_select).current_option().c_str());
      if (request_started_ms_ != 0 && !defrost_active)
      {
        RestoreHeatingWorkingMode();
        request_started_ms_ = 0;
      }
      ClearRequest();
      ResetTriggerTimers();
      control_mode_write_pending_ = true;
      previous_mode_ = mode;
    }
    if (modbus_connected && !modbus_connected_previous_)
    {
      control_mode_write_pending_ = true;
    }
    modbus_connected_previous_ = modbus_connected;

    PublishInterval();

    GetCompressorOnSeconds(now);  // keeps the continuous run time up to date every update
    UpdateRuntime(dt_ms, defrost_active);
    UpdateIceLoad(dt_ms, defrost_active, now);

    // Switching during a defrost: let the defrost finish first, then hand 3274 over.
    if (control_mode_write_pending_ && modbus_connected && !defrost_active)
    {
      ManageDefrostControlMode();
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

    if (defrost_active)
    {
      TrackDefrost(now, dt_ms);
      PublishStatus(started_by_openamber_ ? "Ontdooien" : "Ontdooien (buitenunit)");
      return;
    }
    SampleTp(now);

    if (!IsOpenAmberMode())
    {
      PublishStatus("Uit");
      return;
    }

    if (request_started_ms_ != 0)
    {
      if ((now - request_started_ms_) < CUSTOM_DEFROST_CONFIRM_TIMEOUT_S * 1000UL)
      {
        PublishStatus("Aangevraagd");
        return;
      }

      request_started_ms_ = 0;
      ESP_LOGE("amber", "CustomDefrost: no defrost started within %u s after the forced defrost request, raising TO10.", (unsigned) CUSTOM_DEFROST_CONFIRM_TIMEOUT_S);
      // Take the 4 back, otherwise the outdoor unit would still start the defrost later.
      RestoreHeatingWorkingMode();
      ClearRequest();
      ResetTriggerTimers();
      id(error_defrost_start_timeout).publish_state(true);
    }

    // TO10 is latched: no new request until the error is reset.
    if (id(error_defrost_start_timeout).state)
    {
      ClearRequest();
      PublishStatus("Fout: defrost startte niet");
      return;
    }

    // A manual request goes out at the next opportunity; it is dropped when heating with the compressor ends.
    if (manual_request_)
    {
      if (IsHeatingWithCompressor(defrost_active))
      {
        request_pending_ = true;
        PublishStatus("Handmatig aangevraagd");
        return;
      }
      ESP_LOGW("amber", "CustomDefrost: no longer heating with the compressor, manual defrost request cancelled.");
      manual_request_ = false;
    }

    if (!IsHeatingWithCompressor(defrost_active) || id(temperature_outside_ta).state > CUSTOM_DEFROST_MAX_TA_C)
    {
      ResetTriggerTimers();
      PublishStatus("Bewaken");
      return;
    }

    std::string waiting;
    std::string reason = EvaluateTriggers(now, &waiting);
    if (reason.empty())
    {
      return_temperature_block_since_ms_ = 0;
      PublishStatus(waiting.empty() ? std::string("Bewaken") : "Bewaken: " + waiting);
      return;
    }

    std::string block = EvaluateBlocks(now);
    if (!block.empty())
    {
      PublishStatus("Geblokkeerd: " + block);
      return;
    }

    if (last_status_ != "Defrost gewenst")
    {
      ESP_LOGI("amber", "CustomDefrost: defrost wanted (%s).", reason.c_str());
    }
    request_pending_ = true;
    pending_reason_ = reason;
    PublishStatus("Defrost gewenst");
  }

  /// @brief True when the heat/cool or DHW controller should start a forced defrost now.
  bool ShouldRequestDefrost()
  {
    if (!request_pending_ || request_started_ms_ != 0 || !IsOpenAmberMode() || id(error_defrost_start_timeout).state)
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
    id(custom_defrost_last_trigger).publish_state(reason);
    request_pending_ = false;
    manual_request_ = false;
    request_started_ms_ = NonZero(now);
    PublishStatus("Aangevraagd");
    WriteWorkingMode(WORKING_MODE_FORCED_DEFROST);
  }

  bool IsRequestActive() const
  {
    return request_started_ms_ != 0;
  }

  void RequestManualDefrost()
  {
    if (!IsOpenAmberMode())
    {
      ESP_LOGW("amber", "CustomDefrost: manual defrost only possible when defrost control is set to OpenAmber.");
      return;
    }
    if (request_started_ms_ != 0 || id(defrost_active_sensor).state)
    {
      ESP_LOGW("amber", "CustomDefrost: defrost already requested or active.");
      return;
    }
    if (!IsHeatingWithCompressor(false))
    {
      ESP_LOGW("amber", "CustomDefrost: manual defrost only possible while heating with the compressor running.");
      return;
    }
    // The outdoor unit ignores a forced defrost within 30 min compressor time after the previous one.
    if (IsOutdoorUnitLocked())
    {
      ESP_LOGW("amber", "CustomDefrost: manual defrost refused, the outdoor unit accepts a new defrost %u min after the previous one.",
               (unsigned) (CUSTOM_DEFROST_OUTDOOR_UNIT_LOCK_S / 60));
      return;
    }
    ESP_LOGI("amber", "CustomDefrost: manual defrost requested.");
    manual_request_ = true;
  }
};
