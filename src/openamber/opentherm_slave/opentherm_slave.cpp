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

#include "opentherm_slave.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include <cmath>
#include <cinttypes>

namespace esphome {
namespace opentherm_slave {

static const char *const SLAVE_TAG = "opentherm.slave";

void OpenthermSlaveHub::setup() {
  ESP_LOGI(SLAVE_TAG, "Setting up OpenTherm slave...");

  this->protocol_ = new OpenthermProtocol(this->in_pin_, this->out_pin_);

  if (!this->protocol_->initialize()) {
    ESP_LOGE(SLAVE_TAG, "Failed to initialize OpenTherm protocol");
    this->mark_failed();
    return;
  }
  this->protocol_->set_event_component(this);
  this->protocol_->listen();
  this->publish_diagnostics_(millis());
  ESP_LOGI(SLAVE_TAG, "OpenTherm slave ready, listening for thermostat");
}

void OpenthermSlaveHub::loop() {
  if (this->protocol_ == nullptr)
    return;

  uint32_t now = millis();
  if (this->relisten_pending_ && static_cast<int32_t>(now - this->relisten_at_ms_) >= 0) {
    this->relisten_pending_ = false;
    this->protocol_->listen();
  }

  if (now - this->last_diag_publish_ms_ >= 1000) {
    this->publish_diagnostics_(now);
  }

  if (now - this->last_status_log_ > 10000) {
    this->last_status_log_ = now;
    ESP_LOGD(SLAVE_TAG, "Status: mode=%s, rx=%u, tx=%u, err=%u",
             mode_to_str_(this->protocol_->get_mode()),
             this->frames_received_, this->frames_sent_, this->errors_);
  }

  switch (this->protocol_->get_mode()) {
    case ProtocolMode::RECEIVED: {
      OpenthermData request;
      if (this->protocol_->get_message(request)) {
        this->frames_received_++;
        this->last_message_ms_ = now;
        this->last_error_type_ = ProtocolErrorType::NO_ERROR;
        this->process_request_(request);
        OpenthermData response = this->build_response_(request);

        ESP_LOGD(SLAVE_TAG, "RX [id=%d] type=%s hb=0x%02X lb=0x%02X (f88=%.1f) -> TX type=%s hb=0x%02X lb=0x%02X",
                 request.id,
                 msg_type_to_str_(request.type), request.value_hb, request.value_lb, request.f88(),
                 msg_type_to_str_(response.type), response.value_hb, response.value_lb);

        this->protocol_->send(response);
      }
      break;
    }

    case ProtocolMode::SENT:
      this->frames_sent_++;
      this->schedule_relisten_(now, POST_TX_IGNORE_MS);
      break;

    case ProtocolMode::ERROR_PROTOCOL:
      this->errors_++;
      this->last_error_type_ = this->protocol_->get_error_type();
      if (this->errors_ <= 3 || (this->errors_ % 10) == 0) {
        ESP_LOGW(SLAVE_TAG,
                 "Protocol error %s (total: %u, bit=%d, clock=%u, capture=0x%08" PRIX32 ", data=0x%08" PRIX32 "), restarting listener",
                 error_type_to_str_(this->protocol_->get_error_type()), this->errors_, this->protocol_->get_bit_pos(),
                 this->protocol_->get_clock(), this->protocol_->get_capture(), this->protocol_->get_raw_data());
      }
      this->schedule_relisten_(now, ERROR_RECOVERY_DELAY_MS);
      break;

    case ProtocolMode::ERROR_TIMEOUT:
      this->errors_++;
      this->last_error_type_ = ProtocolErrorType::NO_CHANGE_TOO_LONG;
      ESP_LOGW(SLAVE_TAG, "Timeout error (total: %u, bit=%d, clock=%u, capture=0x%08" PRIX32 ") restarting listener",
               this->errors_, this->protocol_->get_bit_pos(), this->protocol_->get_clock(), this->protocol_->get_capture());
      this->protocol_->listen();
      break;

    case ProtocolMode::ERROR_TIMER:
      this->errors_++;
      this->last_error_type_ = ProtocolErrorType::NO_CHANGE_TOO_LONG;
      ESP_LOGW(SLAVE_TAG, "Timer error (total: %u, bit=%d, clock=%u), restarting listener", this->errors_,
               this->protocol_->get_bit_pos(), this->protocol_->get_clock());
      this->schedule_relisten_(now, ERROR_RECOVERY_DELAY_MS);
      break;

    default:
      break;
  }
}

void OpenthermSlaveHub::dump_config() {
  ESP_LOGCONFIG(SLAVE_TAG, "OpenTherm Slave:");
  log_pin(SLAVE_TAG, "  In Pin: ", this->in_pin_);
  log_pin(SLAVE_TAG, "  Out Pin: ", this->out_pin_);
}

void OpenthermSlaveHub::process_request_(const OpenthermData &request) {
  auto msg_id = static_cast<MessageId>(request.id);

  switch (msg_id) {
    case MessageId::STATUS:
      break;

    case MessageId::CH_SETPOINT:
      this->publish_sensor_(this->ch_setpoint_sensor_, request.f88());
      break;

    case MessageId::ROOM_SETPOINT:
      this->publish_sensor_(this->room_setpoint_sensor_, request.f88());
      break;

    case MessageId::ROOM_TEMP:
      this->publish_sensor_(this->room_temp_sensor_, request.f88());
      break;

    case MessageId::OUTSIDE_TEMP:
      if (request.type == MessageType::WRITE_DATA) {
        this->publish_sensor_(this->outside_temp_sensor_, request.f88());
      }
      break;

    case MessageId::DHW_SETPOINT:
      if (request.type == MessageType::WRITE_DATA) {
        this->publish_sensor_(this->dhw_setpoint_sensor_, request.f88());
      }
      break;

    case MessageId::MAX_REL_MOD_LEVEL_SETTING:
      this->publish_sensor_(this->max_rel_mod_level_sensor_, request.f88());
      break;

    default:
      break;
  }
}

OpenthermData OpenthermSlaveHub::build_response_(const OpenthermData &request) {
  OpenthermData response;
  response.id = request.id;
  response.value_hb = 0;
  response.value_lb = 0;

  auto msg_id = static_cast<MessageId>(request.id);

  if (request.type == MessageType::READ_DATA) {
    response.type = MessageType::READ_ACK;
  } else if (request.type == MessageType::WRITE_DATA) {
    response.type = MessageType::WRITE_ACK;
  } else {
    response.type = MessageType::DATA_INVALID;
    return response;
  }

  switch (msg_id) {
    case MessageId::STATUS: {
      SlaveStatus status;
      status.fault = this->read_binary_(this->fault_input_);
      status.ch_active = this->read_binary_(this->ch_active_input_);
      status.dhw_active = this->read_binary_(this->dhw_active_input_);
      status.flame_on = this->read_binary_(this->flame_on_input_);
      status.cooling_active = false;
      status.ch2_active = false;
      status.diagnostic = false;
      response.value_hb = request.value_hb;
      response.value_lb = status.to_lb();
      break;
    }

    case MessageId::CH_SETPOINT:
      response.value_hb = request.value_hb;
      response.value_lb = request.value_lb;
      break;

    case MessageId::SLAVE_CONFIG:
      response.value_hb = this->slave_flags_;
      response.value_lb = this->slave_member_id_;
      break;

    case MessageId::FAULT_FLAGS:
      response.value_hb = this->read_binary_(this->fault_input_) ? 0x01 : 0x00;
      response.value_lb = 0;
      break;

    case MessageId::MAX_REL_MOD_LEVEL_SETTING:
      response.value_hb = request.value_hb;
      response.value_lb = request.value_lb;
      break;

    case MessageId::ROOM_SETPOINT:
      response.value_hb = request.value_hb;
      response.value_lb = request.value_lb;
      break;

    case MessageId::REL_MOD_LEVEL:
      response.f88(this->read_sensor_(this->rel_mod_level_input_, 0.0f));
      break;

    case MessageId::CH_WATER_PRESSURE:
      response.f88(this->read_sensor_(this->ch_water_pressure_input_, 0.0f));
      break;

    case MessageId::ROOM_TEMP:
      response.value_hb = request.value_hb;
      response.value_lb = request.value_lb;
      break;

    case MessageId::FLOW_TEMP:
      response.f88(this->read_sensor_(this->flow_temp_input_, 0.0f));
      break;

    case MessageId::DHW_TEMP:
      response.f88(this->read_sensor_(this->dhw_temp_input_, 0.0f));
      break;

    case MessageId::OUTSIDE_TEMP:
      response.f88(this->read_sensor_(this->outside_temp_input_, 0.0f));
      break;

    case MessageId::RETURN_TEMP:
      response.f88(this->read_sensor_(this->return_temp_input_, 0.0f));
      break;

    case MessageId::DHW_SETPOINT:
      if (request.type == MessageType::READ_DATA) {
        response.f88(this->read_sensor_(this->dhw_setpoint_sensor_, 0.0f));
      } else {
        response.value_hb = request.value_hb;
        response.value_lb = request.value_lb;
      }
      break;

    case MessageId::MAX_CH_SETPOINT:
      if (request.type == MessageType::READ_DATA) {
        response.f88(this->read_sensor_(this->max_ch_setpoint_input_, 0.0f));
      } else {
        response.value_hb = request.value_hb;
        response.value_lb = request.value_lb;
      }
      break;

    default:
      response.type = MessageType::UNKNOWN_DATAID;
      response.value_hb = 0;
      response.value_lb = 0;
      break;
  }

  return response;
}

void OpenthermSlaveHub::publish_sensor_(sensor::Sensor *sensor, float value) {
  if (sensor != nullptr) {
    sensor->publish_state(value);
  }
}

void OpenthermSlaveHub::publish_binary_(binary_sensor::BinarySensor *sensor, bool value) {
  if (sensor != nullptr) {
    sensor->publish_state(value);
  }
}

void OpenthermSlaveHub::publish_text_(text_sensor::TextSensor *sensor, const char *value) {
  if (sensor != nullptr) {
    sensor->publish_state(value);
  }
}

float OpenthermSlaveHub::read_sensor_(sensor::Sensor *sensor, float default_value) {
  if (sensor != nullptr && sensor->has_state()) {
    return sensor->state;
  }
  return default_value;
}

bool OpenthermSlaveHub::read_binary_(binary_sensor::BinarySensor *sensor, bool default_value) {
  if (sensor != nullptr && sensor->has_state()) {
    return sensor->state;
  }
  return default_value;
}

void OpenthermSlaveHub::schedule_relisten_(uint32_t now, uint32_t delay_ms) {
  this->protocol_->stop();
  this->relisten_at_ms_ = now + delay_ms;
  this->relisten_pending_ = true;
}

void OpenthermSlaveHub::publish_diagnostics_(uint32_t now) {
  this->last_diag_publish_ms_ = now;

  const bool has_seen_message = this->last_message_ms_ != 0;
  const uint32_t age_ms = has_seen_message ? (now - this->last_message_ms_) : UINT32_MAX;
  const bool link_online = has_seen_message && age_ms <= 15000;

  float link_quality = 0.0f;
  if (has_seen_message) {
    float freshness = 100.0f;
    if (age_ms > 15000) {
      freshness = 0.0f;
    } else if (age_ms > 10000) {
      freshness = 25.0f;
    } else if (age_ms > 5000) {
      freshness = 70.0f;
    }

    const uint32_t total_attempts = this->frames_received_ + this->errors_;
    float success_ratio = 100.0f;
    if (total_attempts > 0) {
      success_ratio = 100.0f * static_cast<float>(this->frames_received_) / static_cast<float>(total_attempts);
    }
    link_quality = std::fmax(0.0f, std::fmin(freshness, success_ratio));
  }

  this->publish_sensor_(this->link_quality_sensor_, link_quality);
  this->publish_sensor_(this->frames_received_sensor_, static_cast<float>(this->frames_received_));
  this->publish_sensor_(this->frames_sent_sensor_, static_cast<float>(this->frames_sent_));
  this->publish_sensor_(this->errors_total_sensor_, static_cast<float>(this->errors_));
  this->publish_sensor_(this->last_message_age_sensor_, has_seen_message ? age_ms / 1000.0f : NAN);
  this->publish_binary_(this->link_online_binary_sensor_, link_online);
  this->publish_text_(this->status_text_sensor_, status_key_(this->protocol_->get_mode(), link_online, age_ms, has_seen_message));
  this->publish_text_(this->last_error_text_sensor_, diagnostic_error_key_(this->last_error_type_));
}

const char *OpenthermSlaveHub::mode_to_str_(ProtocolMode mode) {
  switch (mode) {
    case ProtocolMode::IDLE: return "IDLE";
    case ProtocolMode::LISTENING: return "LISTENING";
    case ProtocolMode::RECEIVING: return "RECEIVING";
    case ProtocolMode::RECEIVED: return "RECEIVED";
    case ProtocolMode::WRITING: return "WRITING";
    case ProtocolMode::SENT: return "SENT";
    case ProtocolMode::ERROR_TIMEOUT: return "ERR_TIMEOUT";
    case ProtocolMode::ERROR_PROTOCOL: return "ERR_PROTOCOL";
    default: return "UNKNOWN";
  }
}

const char *OpenthermSlaveHub::msg_type_to_str_(MessageType type) {
  switch (type) {
    case MessageType::READ_DATA: return "READ";
    case MessageType::WRITE_DATA: return "WRITE";
    case MessageType::INVALID_DATA: return "INVALID";
    case MessageType::READ_ACK: return "READ_ACK";
    case MessageType::WRITE_ACK: return "WRITE_ACK";
    case MessageType::DATA_INVALID: return "DATA_INV";
    case MessageType::UNKNOWN_DATAID: return "UNK_ID";
    default: return "???";
  }
}

const char *OpenthermSlaveHub::error_type_to_str_(ProtocolErrorType type) {
  switch (type) {
    case ProtocolErrorType::NO_ERROR: return "NO_ERROR";
    case ProtocolErrorType::NO_TRANSITION: return "NO_TRANSITION";
    case ProtocolErrorType::INVALID_STOP_BIT: return "INVALID_STOP_BIT";
    case ProtocolErrorType::PARITY_ERROR: return "PARITY_ERROR";
    case ProtocolErrorType::NO_CHANGE_TOO_LONG: return "NO_CHANGE_TOO_LONG";
    case ProtocolErrorType::INVALID_MESSAGE: return "INVALID_MESSAGE";
    default: return "UNKNOWN";
  }
}

const char *OpenthermSlaveHub::status_key_(ProtocolMode mode, bool link_online, uint32_t age_ms, bool has_seen_message) {
  switch (mode) {
    case ProtocolMode::ERROR_PROTOCOL:
      return "protocol_error";
    case ProtocolMode::ERROR_TIMEOUT:
      return "timeout";
    case ProtocolMode::ERROR_TIMER:
      return "timer_error";
    default:
      break;
  }

  if (link_online) {
    return "connected";
  }
  if (!has_seen_message) {
    return "waiting_for_thermostat";
  }
  if (age_ms > 15000) {
    return "connection_lost";
  }
  return "listening";
}

const char *OpenthermSlaveHub::diagnostic_error_key_(ProtocolErrorType type) {
  switch (type) {
    case ProtocolErrorType::NO_ERROR:
      return "none";
    case ProtocolErrorType::NO_TRANSITION:
      return "no_transition";
    case ProtocolErrorType::INVALID_STOP_BIT:
      return "invalid_stop_bit";
    case ProtocolErrorType::PARITY_ERROR:
      return "parity_error";
    case ProtocolErrorType::NO_CHANGE_TOO_LONG:
      return "no_change_too_long";
    case ProtocolErrorType::INVALID_MESSAGE:
      return "invalid_message";
    default:
      return "unknown";
  }
}

}  // namespace opentherm_slave
}  // namespace esphome
