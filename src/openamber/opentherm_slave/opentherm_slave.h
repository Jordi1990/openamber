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

#include "opentherm_consts.h"
#include "opentherm_protocol.h"
#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

namespace esphome {
namespace opentherm_slave {

class OpenthermSlaveHub : public Component {
 public:
  void set_in_pin(InternalGPIOPin *pin) { this->in_pin_ = pin; }
  void set_out_pin(InternalGPIOPin *pin) { this->out_pin_ = pin; }

  void set_ch_setpoint_sensor(sensor::Sensor *s) { this->ch_setpoint_sensor_ = s; }
  void set_room_setpoint_sensor(sensor::Sensor *s) { this->room_setpoint_sensor_ = s; }
  void set_room_temp_sensor(sensor::Sensor *s) { this->room_temp_sensor_ = s; }
  void set_outside_temp_sensor(sensor::Sensor *s) { this->outside_temp_sensor_ = s; }
  void set_dhw_setpoint_sensor(sensor::Sensor *s) { this->dhw_setpoint_sensor_ = s; }
  void set_max_rel_mod_level_sensor(sensor::Sensor *s) { this->max_rel_mod_level_sensor_ = s; }

  void set_flow_temp_input(sensor::Sensor *s) { this->flow_temp_input_ = s; }
  void set_dhw_temp_input(sensor::Sensor *s) { this->dhw_temp_input_ = s; }
  void set_return_temp_input(sensor::Sensor *s) { this->return_temp_input_ = s; }
  void set_outside_temp_input(sensor::Sensor *s) { this->outside_temp_input_ = s; }
  void set_rel_mod_level_input(sensor::Sensor *s) { this->rel_mod_level_input_ = s; }
  void set_ch_water_pressure_input(sensor::Sensor *s) { this->ch_water_pressure_input_ = s; }
  void set_max_ch_setpoint_input(sensor::Sensor *s) { this->max_ch_setpoint_input_ = s; }

  void set_flame_on_input(binary_sensor::BinarySensor *s) { this->flame_on_input_ = s; }
  void set_ch_active_input(binary_sensor::BinarySensor *s) { this->ch_active_input_ = s; }
  void set_dhw_active_input(binary_sensor::BinarySensor *s) { this->dhw_active_input_ = s; }
  void set_fault_input(binary_sensor::BinarySensor *s) { this->fault_input_ = s; }

  void set_link_quality_sensor(sensor::Sensor *s) { this->link_quality_sensor_ = s; }
  void set_frames_received_sensor(sensor::Sensor *s) { this->frames_received_sensor_ = s; }
  void set_frames_sent_sensor(sensor::Sensor *s) { this->frames_sent_sensor_ = s; }
  void set_errors_total_sensor(sensor::Sensor *s) { this->errors_total_sensor_ = s; }
  void set_last_message_age_sensor(sensor::Sensor *s) { this->last_message_age_sensor_ = s; }

  void set_link_online_binary_sensor(binary_sensor::BinarySensor *s) { this->link_online_binary_sensor_ = s; }

  void set_status_text_sensor(text_sensor::TextSensor *s) { this->status_text_sensor_ = s; }
  void set_last_error_text_sensor(text_sensor::TextSensor *s) { this->last_error_text_sensor_ = s; }

  void set_slave_member_id(uint8_t id) { this->slave_member_id_ = id; }
  void set_slave_flags(uint8_t flags) { this->slave_flags_ = flags; }

  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void setup() override;
  void loop() override;
  void dump_config() override;

 protected:
  static const uint32_t ERROR_RECOVERY_DELAY_MS = 25;
  static const uint32_t POST_TX_IGNORE_MS = 4;

  InternalGPIOPin *in_pin_{nullptr};
  InternalGPIOPin *out_pin_{nullptr};
  OpenthermProtocol *protocol_{nullptr};

  uint32_t last_status_log_{0};
  uint32_t relisten_at_ms_{0};
  uint32_t frames_received_{0};
  uint32_t frames_sent_{0};
  uint32_t errors_{0};
  uint32_t last_message_ms_{0};
  uint32_t last_diag_publish_ms_{0};
  bool relisten_pending_{false};

  sensor::Sensor *ch_setpoint_sensor_{nullptr};
  sensor::Sensor *room_setpoint_sensor_{nullptr};
  sensor::Sensor *room_temp_sensor_{nullptr};
  sensor::Sensor *outside_temp_sensor_{nullptr};
  sensor::Sensor *dhw_setpoint_sensor_{nullptr};
  sensor::Sensor *max_rel_mod_level_sensor_{nullptr};

  sensor::Sensor *flow_temp_input_{nullptr};
  sensor::Sensor *dhw_temp_input_{nullptr};
  sensor::Sensor *return_temp_input_{nullptr};
  sensor::Sensor *outside_temp_input_{nullptr};
  sensor::Sensor *rel_mod_level_input_{nullptr};
  sensor::Sensor *ch_water_pressure_input_{nullptr};
  sensor::Sensor *max_ch_setpoint_input_{nullptr};

  binary_sensor::BinarySensor *flame_on_input_{nullptr};
  binary_sensor::BinarySensor *ch_active_input_{nullptr};
  binary_sensor::BinarySensor *dhw_active_input_{nullptr};
  binary_sensor::BinarySensor *fault_input_{nullptr};

  sensor::Sensor *link_quality_sensor_{nullptr};
  sensor::Sensor *frames_received_sensor_{nullptr};
  sensor::Sensor *frames_sent_sensor_{nullptr};
  sensor::Sensor *errors_total_sensor_{nullptr};
  sensor::Sensor *last_message_age_sensor_{nullptr};

  binary_sensor::BinarySensor *link_online_binary_sensor_{nullptr};

  text_sensor::TextSensor *status_text_sensor_{nullptr};
  text_sensor::TextSensor *last_error_text_sensor_{nullptr};

  uint8_t slave_member_id_{0};
  uint8_t slave_flags_{0x01};
  ProtocolErrorType last_error_type_{ProtocolErrorType::NO_ERROR};

  void process_request_(const OpenthermData &request);
  OpenthermData build_response_(const OpenthermData &request);
  void publish_diagnostics_(uint32_t now);

  void publish_sensor_(sensor::Sensor *sensor, float value);
  void publish_binary_(binary_sensor::BinarySensor *sensor, bool value);
  void publish_text_(text_sensor::TextSensor *sensor, const char *value);
  float read_sensor_(sensor::Sensor *sensor, float default_value);
  bool read_binary_(binary_sensor::BinarySensor *sensor, bool default_value = false);
  void schedule_relisten_(uint32_t now, uint32_t delay_ms);

  static const char *mode_to_str_(ProtocolMode mode);
  static const char *msg_type_to_str_(MessageType type);
  static const char *error_type_to_str_(ProtocolErrorType type);
  static const char *status_key_(ProtocolMode mode, bool link_online, uint32_t age_ms, bool has_seen_message);
  static const char *diagnostic_error_key_(ProtocolErrorType type);
};

}  // namespace opentherm_slave
}  // namespace esphome
