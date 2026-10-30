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
/// Modes (custom_defrost_mode_select):
///  - Buitenunit: outdoor unit decides itself, nothing happens here except statistics.
///  - Dry-run: the algorithm runs and logs its trigger moments, the outdoor unit still defrosts itself.
///  - OpenAmber: EEPROM 3274 is set to master control and a defrost is started by writing working mode 4 (forced defrost).
/// The heat/cool and DHW controllers pick up a pending request in COMPRESSOR_RUNNING via ShouldRequestDefrost().
class CustomDefrostController
{
private:
  static const int HISTORY_SIZE = 24;

  CustomDefrostState state_ = CustomDefrostState::OFF;
  uint32_t last_update_ms_ = 0;
  uint32_t runtime_remainder_ms_ = 0;
  uint32_t compressor_running_since_ms_ = 0;
  uint32_t tp_below_since_ms_ = 0;
  uint32_t dt_above_since_ms_ = 0;
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
    tp_below_since_ms_ = 0;
    dt_above_since_ms_ = 0;
    return_temperature_block_since_ms_ = 0;
  }

  float GetTpThreshold(float ta, int* band)
  {
    if (ta > 3.0f)
    {
      *band = 1;
      return id(custom_defrost_tp_band1).state;
    }
    if (ta > -3.0f)
    {
      *band = 2;
      return id(custom_defrost_tp_band2).state;
    }
    if (ta > -10.0f)
    {
      *band = 3;
      return id(custom_defrost_tp_band3).state;
    }
    *band = 4;
    return id(custom_defrost_tp_band4).state;
  }

  float GetEvaporatorTemperature()
  {
    float te = id(evaporating_temperature_te).state;
    return std::isnan(te) ? id(evaporation_temperature_tp).state : te;
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

  void PublishTaTe()
  {
    float ta = id(temperature_outside_ta).state;
    float evap = GetEvaporatorTemperature();
    if (std::isnan(ta) || std::isnan(evap))
    {
      return;
    }
    float delta = roundf((ta - evap) * 10.0f) / 10.0f;
    float current = id(custom_defrost_ta_te).state;
    if (std::isnan(current) || fabsf(current - delta) >= 0.1f)
    {
      id(custom_defrost_ta_te).publish_state(delta);
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
      ESP_LOGW("amber", "CustomDefrost: restoring working mode to heating.");
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
      float duration_s = (float)(now - defrost_started_ms_) / 1000.0f;
      id(custom_defrost_last_duration).publish_state(roundf(duration_s));
      ESP_LOGI("amber", "CustomDefrost: defrost ended after %.0f s (%s).", duration_s, started_by_openamber_ ? "OpenAmber" : "outdoor unit");
    }
    defrost_started_ms_ = 0;
    started_by_openamber_ = false;
    id(custom_defrost_runtime_s) = 0;
    runtime_remainder_ms_ = 0;
    PublishRuntime();
    ResetTriggerTimers();
    forced_mode_seen_since_ms_ = 0;
    SetState(GetMode() == (int)CUSTOM_DEFROST_MODE_OUTDOOR_UNIT ? CustomDefrostState::OFF : CustomDefrostState::MONITORING);
  }

  /// @brief Restores the working mode when the outdoor unit did not switch back from forced defrost by itself.
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

  /// @brief Returns the trigger reason or an empty string. Sets *fallback when the max runtime safety net fired.
  std::string EvaluateTriggers(uint32_t now, bool* fallback)
  {
    *fallback = false;
    float ta = id(temperature_outside_ta).state;
    float tp = id(evaporation_temperature_tp).state;
    if (std::isnan(ta) || std::isnan(tp))
    {
      ResetTriggerTimers();
      return "";
    }

    char buffer[96];
    uint32_t runtime_s = GetRuntimeSeconds();
    if (runtime_s >= (uint32_t)(id(custom_defrost_max_runtime).state * 60.0f) && tp < id(custom_defrost_fallback_tp).state)
    {
      *fallback = true;
      snprintf(buffer, sizeof(buffer), "Vangnet: %u min draaitijd, Tp %.1f°C", (unsigned) (runtime_s / 60), tp);
      return buffer;
    }

    std::string reason;
    if (id(custom_defrost_tp_trigger_switch).state)
    {
      int band = 0;
      float threshold = GetTpThreshold(ta, &band);
      if (tp < threshold)
      {
        if (tp_below_since_ms_ == 0)
        {
          tp_below_since_ms_ = NonZero(now);
        }
        if ((now - tp_below_since_ms_) >= (uint32_t)(id(custom_defrost_tp_debounce).state * 1000.0f))
        {
          snprintf(buffer, sizeof(buffer), "Tp %.1f < %.1f°C (band %d)", tp, threshold, band);
          reason = buffer;
        }
      }
      else
      {
        tp_below_since_ms_ = 0;
      }
    }
    else
    {
      tp_below_since_ms_ = 0;
    }

    if (id(custom_defrost_dt_trigger_switch).state)
    {
      float delta = ta - GetEvaporatorTemperature();
      float threshold = id(custom_defrost_dt_threshold).state;
      if (tp < 0.0f && delta >= threshold)
      {
        if (dt_above_since_ms_ == 0)
        {
          dt_above_since_ms_ = NonZero(now);
        }
        if (reason.empty() && (now - dt_above_since_ms_) >= (uint32_t)(id(custom_defrost_dt_debounce).state * 1000.0f))
        {
          snprintf(buffer, sizeof(buffer), "Ta-Te %.1f >= %.1f K", delta, threshold);
          reason = buffer;
        }
      }
      else
      {
        dt_above_since_ms_ = 0;
      }
    }
    else
    {
      dt_above_since_ms_ = 0;
    }

    return reason;
  }

  /// @brief Returns the block reason or an empty string.
  std::string EvaluateBlocks(uint32_t now, bool fallback)
  {
    char buffer[96];
    uint32_t runtime_s = GetRuntimeSeconds();
    uint32_t min_runtime_s = (uint32_t)(id(custom_defrost_min_runtime).state * 60.0f);
    if (!fallback && runtime_s < min_runtime_s)
    {
      snprintf(buffer, sizeof(buffer), "draaitijd %u < %u min", (unsigned) (runtime_s / 60), (unsigned) (min_runtime_s / 60));
      return buffer;
    }

    uint32_t min_on_s = std::max((uint32_t)(id(custom_defrost_min_compressor_on).state * 60.0f), COMPRESSOR_SOFT_START_DURATION_S);
    if (compressor_running_since_ms_ == 0 || (now - compressor_running_since_ms_) < min_on_s * 1000UL)
    {
      return "compressor net gestart";
    }

    float return_temperature = id(inlet_temperature_tui).state;
    float min_return_temperature = id(custom_defrost_min_return_temperature).state;
    if (!std::isnan(return_temperature) && return_temperature < min_return_temperature)
    {
      if (return_temperature_block_since_ms_ == 0)
      {
        return_temperature_block_since_ms_ = NonZero(now);
      }
      uint32_t max_wait_ms = (uint32_t)(id(custom_defrost_max_return_wait).state * 60000.0f);
      if ((now - return_temperature_block_since_ms_) < max_wait_ms)
      {
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

    if (IsCompressorRunning())
    {
      if (compressor_running_since_ms_ == 0)
      {
        compressor_running_since_ms_ = NonZero(now);
      }
    }
    else
    {
      compressor_running_since_ms_ = 0;
    }

    UpdateRuntime(dt_ms, defrost_active);
    PublishTaTe();
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
      if ((now - manual_request_ms_) >= 10UL * 60UL * 1000UL)
      {
        ESP_LOGW("amber", "CustomDefrost: manual defrost request expired, compressor was not running in heating mode.");
        manual_request_ = false;
        request_pending_ = false;
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

    bool fallback = false;
    std::string reason = EvaluateTriggers(now, &fallback);
    if (reason.empty())
    {
      request_pending_ = false;
      return_temperature_block_since_ms_ = 0;
      SetState(CustomDefrostState::MONITORING);
      PublishStatus("Bewaken");
      return;
    }

    std::string block = EvaluateBlocks(now, fallback);
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
