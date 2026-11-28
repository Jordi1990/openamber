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

/// @brief Vapour pressure excess of the outdoor air over ice at the coil temperature, in Pa (sensor "Defrost ijsgroei").
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
/// With a valid outdoor humidity (BLE sensor) the ice counter does the work: the sensor "Defrost ijsteller" integrates
/// "Defrost ijsgroei" (the vapour pressure excess of the air over ice at Tp while heating with the compressor) and goes
/// to 0 after a defrost or when the frost has melted (YAML). The interval route then only acts after 4 h compressor
/// time, as an emergency limit for a wrong sensor value. The compressor time since the last defrost is the
/// integration sensor "Defrost draaitijd sinds defrost".
/// There is no start line on Ta - Tp (Mitsubishi SRK-ZSX): on the Amber Ta - Tp stayed 2.4-3.6 K with and without frost
/// (camera, 27/28 Nov 2026), so it does not show ice.
///
/// Modes (defrost_control_select), switchable at any time:
///  - Buitenunit: the outdoor unit decides itself, only statistics are kept here.
///  - OpenAmber: 3274 is on master control (script sync_defrost_control_mode) and a defrost is started by writing
///    working mode 4 while heating with the compressor running. The heat/cool and DHW controllers keep the compressor
///    running while a request is open (IsRequestActive) and react to 2118 as for any defrost.
/// When the outdoor unit does not start a requested defrost within the confirmation time, working mode 4 is taken back and
/// error TO10 (error_defrost_start_timeout) stops OpenAmber like the other timeout errors; no new request until it is reset.
/// A request is cancelled without error when the compressor stops before the defrost starts.
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

  uint32_t compressor_running_since_ms_ = 0;
  uint32_t below_interval_tp_since_ms_ = 0;
  // Non-zero while working mode 4 was written and the outdoor unit has not started the defrost yet.
  uint32_t request_started_ms_ = 0;
  uint32_t defrost_started_ms_ = 0;
  int last_published_interval_min_ = -1;
  int previous_mode_ = -1;
  bool defrost_active_previous_ = false;
  bool manual_request_ = false;
  bool started_by_openamber_ = false;
  DefrostEvent event_;
  std::string last_status_;

  bool IsOpenAmberMode()
  {
    return id(defrost_control_select).active_index().value_or(CUSTOM_DEFROST_MODE_OUTDOOR_UNIT) == CUSTOM_DEFROST_MODE_OPENAMBER;
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
    if (status != last_status_)
    {
      last_status_ = status;
      id(custom_defrost_status).publish_state(status);
    }
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

  /// @brief Short defrost = little ice, so wait longer next time. Long defrost = wait shorter. Same rule as the
  /// outdoor unit's Smart mode, on the 2118 time of a forced defrost.
  void AdaptInterval(uint32_t duration_s)
  {
    uint32_t interval = GetIntervalMinutes();
    float max_time_min = id(max_defrost_time).state;
    uint32_t max_defrost_s = std::isnan(max_time_min) ? 8 * 60 : (uint32_t)(max_time_min * 60.0f);
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

  // ── Compressor time since the last defrost (integration sensor, reset by a defrost) ──

  uint32_t GetRuntimeSeconds()
  {
    float minutes = id(custom_defrost_runtime_sensor).state;
    return std::isnan(minutes) || minutes < 0.0f ? 0 : (uint32_t)(minutes * 60.0f);
  }

  /// @brief The outdoor unit postpones a forced defrost until 30 min compressor time after the previous one.
  bool IsOutdoorUnitLocked()
  {
    return GetRuntimeSeconds() < CUSTOM_DEFROST_OUTDOOR_UNIT_LOCK_S;
  }

  // ── Request ──

  void RestoreHeatingWorkingMode()
  {
    if (IsWorkingModeIndex(WORKING_MODE_FORCED_DEFROST))
    {
      ESP_LOGI("amber", "CustomDefrost: restoring working mode to heating.");
      WriteWorkingMode(WORKING_MODE_HEATING);
    }
  }

  /// @brief Writes working mode 4 (forced defrost).
  void StartRequest(const std::string& reason, uint32_t now)
  {
    ESP_LOGI("amber", "CustomDefrost: requesting forced defrost (%s).", reason.c_str());
    id(custom_defrost_last_trigger).publish_state(reason);
    manual_request_ = false;
    request_started_ms_ = NonZero(now);
    PublishStatus("Aangevraagd");
    WriteWorkingMode(WORKING_MODE_FORCED_DEFROST);
  }

  /// @brief Takes the 4 back, otherwise the outdoor unit would still start the defrost later.
  void CancelRequest()
  {
    request_started_ms_ = 0;
    below_interval_tp_since_ms_ = 0;
    RestoreHeatingWorkingMode();
  }

  // ── Measurement: the 2118 time and the heat taken from the water (custom_defrost_energy) ──

  void OnDefrostStarted(uint32_t now)
  {
    defrost_started_ms_ = NonZero(now);
    started_by_openamber_ = request_started_ms_ != 0;
    request_started_ms_ = 0;
    manual_request_ = false;
    event_ = DefrostEvent{id(temperature_outside_ta).state, id(evaporation_temperature_tp).state,
                          id(outdoor_humidity).state, GetRuntimeSeconds() / 60.0f,
                          id(custom_defrost_ice_load_sensor).state, started_by_openamber_ ? 1 : 0};
    id(custom_defrost_energy).reset();
    ESP_LOGI("amber", "CustomDefrost: defrost started by %s (Tp %.1f°C, Ta %.1f°C).",
             started_by_openamber_ ? "OpenAmber" : "outdoor unit", event_.tp, event_.ta);
  }

  void OnDefrostEnded(uint32_t now)
  {
    if (defrost_started_ms_ != 0)
    {
      // The 2118 time. A defrost the outdoor unit starts itself includes its pre-phase (about 5 min), so only a forced
      // defrost adapts the interval.
      float duration_s = (float) ((now - defrost_started_ms_) / 1000);
      // The defrost takes its heat from the water, so the thermal power is negative during it.
      float energy_kj = roundf(-id(custom_defrost_energy).state);
      id(custom_defrost_last_duration).publish_state(duration_s);
      ESP_LOGI("amber", "CustomDefrost: defrost ended (%s): %.0f s, %.0f kJ.",
               started_by_openamber_ ? "OpenAmber" : "outdoor unit", duration_s, energy_kj);
      if (started_by_openamber_)
      {
        AdaptInterval((uint32_t) duration_s);
      }
      // Execution and melt time are no longer measured (NAN, the event keys stay for the backend).
      id(send_defrost_event).execute(event_.source, duration_s, NAN, NAN, energy_kj, event_.ta, event_.tp,
                                     event_.humidity, event_.runtime_min, event_.ice_load);
    }
    // The outdoor unit returns to heating by itself but leaves 3999 on 4, which would start a new defrost after 30 min.
    RestoreHeatingWorkingMode();
    defrost_started_ms_ = 0;
    started_by_openamber_ = false;
    id(custom_defrost_runtime_sensor).reset();
    id(custom_defrost_ice_load_sensor).reset();
    below_interval_tp_since_ms_ = 0;
  }

public:
  CustomDefrostController() {}

  /// @brief Called every controller update, in every main state.
  void Update()
  {
    uint32_t now = App.get_loop_component_start_time();

    int mode = id(defrost_control_select).active_index().value_or(CUSTOM_DEFROST_MODE_OUTDOOR_UNIT);
    bool defrost_active = id(defrost_active_sensor).state;
    if (mode != previous_mode_)
    {
      ESP_LOGI("amber", "CustomDefrost: mode changed to %s", id(defrost_control_select).current_option().c_str());
      if (request_started_ms_ != 0 && !defrost_active)
      {
        CancelRequest();
      }
      manual_request_ = false;
      below_interval_tp_since_ms_ = 0;
      previous_mode_ = mode;
    }

    PublishInterval();
    uint32_t compressor_on_s = HoldSeconds(IsCompressorRunning(), &compressor_running_since_ms_, now);

    if (defrost_active && !defrost_active_previous_)
    {
      OnDefrostStarted(now);
    }
    else if (!defrost_active && defrost_active_previous_)
    {
      OnDefrostEnded(now);
    }
    defrost_active_previous_ = defrost_active;

    if (defrost_active)
    {
      PublishStatus(started_by_openamber_ ? "Ontdooien" : "Ontdooien (buitenunit)");
      return;
    }
    if (!IsOpenAmberMode())
    {
      PublishStatus("Uit");
      return;
    }

    if (request_started_ms_ != 0)
    {
      if (!IsCompressorRunning())
      {
        ESP_LOGW("amber", "CustomDefrost: compressor stopped before the defrost started, request cancelled.");
        CancelRequest();
      }
      else if ((now - request_started_ms_) < CUSTOM_DEFROST_CONFIRM_TIMEOUT_S * 1000UL)
      {
        PublishStatus("Aangevraagd");
        return;
      }
      else
      {
        ESP_LOGE("amber", "CustomDefrost: no defrost started within %u s after the forced defrost request, raising TO10.", (unsigned) CUSTOM_DEFROST_CONFIRM_TIMEOUT_S);
        CancelRequest();
        id(error_defrost_start_timeout).publish_state(true);
      }
    }

    // TO10 is latched: no new request until the error is reset.
    if (id(error_defrost_start_timeout).state)
    {
      manual_request_ = false;
      PublishStatus("Fout: defrost startte niet");
      return;
    }

    float tp = id(evaporation_temperature_tp).state;
    if (!IsHeatingWithCompressor(defrost_active) || id(controlled_stop_active) ||
        id(temperature_outside_ta).state > CUSTOM_DEFROST_MAX_TA_C || std::isnan(tp))
    {
      if (manual_request_)
      {
        ESP_LOGW("amber", "CustomDefrost: no longer heating with the compressor, manual defrost request cancelled.");
        manual_request_ = false;
      }
      below_interval_tp_since_ms_ = 0;
      PublishStatus("Bewaken");
      return;
    }

    char reason[96];
    uint32_t runtime_min = GetRuntimeSeconds() / 60;
    // With a humidity value the ice counter does the work; the interval route is then only an emergency limit (4 h).
    bool humidity = CustomDefrostValidHumidity(id(outdoor_humidity).state);
    uint32_t interval_min = humidity ? CUSTOM_DEFROST_HUMIDITY_EMERGENCY_MIN : GetIntervalMinutes();
    float ice_threshold = id(custom_defrost_ice_threshold).state;
    float ice_load = id(custom_defrost_ice_load_sensor).state;
    // Once the interval has passed, a coil below -2 °C for 3 min (after the compressor's soft start) defrosts.
    bool cold = compressor_on_s >= COMPRESSOR_SOFT_START_DURATION_S && tp < CUSTOM_DEFROST_INTERVAL_TP_C;
    uint32_t cold_s = HoldSeconds(cold, &below_interval_tp_since_ms_, now);
    if (manual_request_)
    {
      snprintf(reason, sizeof(reason), "Handmatig");
    }
    else if (humidity && runtime_min >= GetMinIntervalMinutes() && ice_threshold > 0.0f && ice_load >= ice_threshold)
    {
      snprintf(reason, sizeof(reason), "IJsteller: %.0f >= %.0f Pa·h (Tp %.1f°C)", ice_load, ice_threshold, tp);
    }
    else if (runtime_min < interval_min)
    {
      snprintf(reason, sizeof(reason), "Bewaken: interval %u / %u min", (unsigned) runtime_min, (unsigned) interval_min);
      PublishStatus(reason);
      return;
    }
    else if (cold_s >= CUSTOM_DEFROST_CONFIRM_S)
    {
      snprintf(reason, sizeof(reason), "Interval: %u min draaitijd, Tp %.1f°C", (unsigned) runtime_min, tp);
    }
    else
    {
      PublishStatus("Bewaken");
      return;
    }

    if (IsOutdoorUnitLocked())
    {
      PublishStatus("Geblokkeerd: slot buitenunit (30 min na defrost)");
      return;
    }
    StartRequest(reason, now);
  }

  /// @brief True while working mode 4 is written and the outdoor unit has not started the defrost yet. The heat/cool and
  /// DHW controllers keep the compressor running and write no working mode then.
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
