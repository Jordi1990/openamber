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

#include "opentherm_protocol.h"

namespace esphome {
namespace opentherm_slave {

static const char *const TAG = "opentherm.protocol";

bool OpenthermProtocol::initialize() {
  this->in_pin_->pin_mode(gpio::FLAG_INPUT);
  this->in_pin_->setup();
  this->out_pin_->pin_mode(gpio::FLAG_OUTPUT);
  this->out_pin_->setup();
  this->out_pin_->digital_write(true);

  bool in_level = this->in_pin_->digital_read();
  this->idle_level_ = in_level;
  ESP_LOGD(TAG, "Initial line levels: in_pin=%d (idle expected HIGH), out_pin forced HIGH", in_level);
  if (!in_level) {
    ESP_LOGW(TAG, "in_pin is LOW at startup; check OT adapter pin mapping or signal inversion");
  }

#ifdef USE_ESP32
  if (!this->init_esp32_timer_()) {
    return false;
  }
#endif

  this->mode_ = ProtocolMode::IDLE;
  this->error_type_ = ProtocolErrorType::NO_ERROR;
  this->capture_ = 0;
  this->clock_ = 0;
  this->data_ = 0;
  this->bit_pos_ = 0;
  this->listen_last_value_ = this->in_pin_->digital_read() == this->idle_level_;
  this->listen_low_ticks_ = this->listen_last_value_ ? 0 : 1;
  return true;
}

void OpenthermProtocol::listen() {
  this->stop_timer_();
  this->timeout_counter_ = this->device_timeout_ < 0 ? -1 : this->device_timeout_ * 5;
  this->listen_guard_ticks_ = LISTEN_GUARD_TICKS;
  this->mode_ = ProtocolMode::LISTENING;
  this->error_type_ = ProtocolErrorType::NO_ERROR;
  this->data_ = 0;
  this->bit_pos_ = 0;
  this->capture_ = 0;
  this->clock_ = 0;
  this->listen_last_value_ = this->isr_in_pin_.digital_read() == this->idle_level_;
  this->listen_low_ticks_ = this->listen_last_value_ ? 0 : 1;
  this->start_read_timer_();
}

void OpenthermProtocol::send(const OpenthermData &data) {
  this->stop_timer_();
  this->data_ = static_cast<uint32_t>(data.type);
  this->data_ = (this->data_ << 12) | data.id;
  this->data_ = (this->data_ << 8) | data.value_hb;
  this->data_ = (this->data_ << 8) | data.value_lb;
  if (parity(this->data_)) {
    this->data_ |= 0x80000000UL;
  }

  this->clock_ = 1;
  this->bit_pos_ = TX_STOP_BIT_POS;
  this->error_type_ = ProtocolErrorType::NO_ERROR;
  this->mode_ = ProtocolMode::WRITING;
  this->start_write_timer_();
}

bool OpenthermProtocol::get_message(OpenthermData &data) {
  if (this->mode_ != ProtocolMode::RECEIVED) {
    return false;
  }

  uint8_t type = (this->data_ >> 28) & 0x7;
  if (type != static_cast<uint8_t>(MessageType::READ_DATA) &&
      type != static_cast<uint8_t>(MessageType::WRITE_DATA)) {
    this->mode_ = ProtocolMode::ERROR_PROTOCOL;
    this->error_type_ = ProtocolErrorType::INVALID_MESSAGE;
    return false;
  }

  data.type = static_cast<MessageType>(type);
  data.id = (this->data_ >> 16) & 0xFF;
  data.value_hb = (this->data_ >> 8) & 0xFF;
  data.value_lb = this->data_ & 0xFF;
  return true;
}

void OpenthermProtocol::stop() {
  this->stop_timer_();
  this->mode_ = ProtocolMode::IDLE;
}

#ifdef USE_ESP32
bool OpenthermProtocol::init_esp32_timer_() {
  timer_config_t const config = {
      .alarm_en = TIMER_ALARM_EN,
      .counter_en = TIMER_PAUSE,
      .intr_type = TIMER_INTR_LEVEL,
      .counter_dir = TIMER_COUNT_UP,
      .auto_reload = TIMER_AUTORELOAD_EN,
      .clk_src = TIMER_SRC_CLK_DEFAULT,
      .divider = 80,
  };

  // Find a free timer first; fixed timer slots can conflict with other components.
  int cur_timer = 0;
  timer_group_t timer_group = TIMER_GROUP_0;
  timer_idx_t timer_idx = TIMER_0;
  bool timer_found = false;

  for (; cur_timer < SOC_TIMER_GROUP_TOTAL_TIMERS; cur_timer++) {
    timer_group = cur_timer < 2 ? TIMER_GROUP_0 : TIMER_GROUP_1;
    timer_idx = cur_timer < 2 ? (timer_idx_t) cur_timer : (timer_idx_t) (cur_timer - 2);

    auto err = timer_init(timer_group, timer_idx, &config);
    if (err == ESP_OK) {
      timer_found = true;
      break;
    }

    ESP_LOGD(TAG, "Timer %d:%d not available, trying next", timer_group, timer_idx);
  }

  if (!timer_found) {
    ESP_LOGE(TAG, "No free timer found for OpenTherm");
    return false;
  }

  this->timer_group_ = timer_group;
  this->timer_idx_ = timer_idx;

  auto result = ESP_OK;

  result = timer_set_counter_value(this->timer_group_, this->timer_idx_, 0);
  if (result != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set timer counter: %s", esp_err_to_name(result));
    return false;
  }

  result = timer_isr_callback_add(this->timer_group_, this->timer_idx_,
                                  reinterpret_cast<bool (*)(void *)>(timer_isr), this, 0);
  if (result != ESP_OK) {
    ESP_LOGE(TAG, "Failed to register timer ISR: %s", esp_err_to_name(result));
    return false;
  }

  return true;
}

void IRAM_ATTR OpenthermProtocol::start_esp32_timer_(uint64_t alarm_value) {
  InterruptLock const lock;
  this->timer_error_ = timer_set_alarm_value(this->timer_group_, this->timer_idx_, alarm_value);
  if (this->timer_error_ != ESP_OK) {
    this->mode_ = ProtocolMode::ERROR_TIMER;
    this->notify_event_component_();
    return;
  }

  this->timer_error_ = timer_start(this->timer_group_, this->timer_idx_);
  if (this->timer_error_ != ESP_OK) {
    this->mode_ = ProtocolMode::ERROR_TIMER;
    this->notify_event_component_();
  }
}

void IRAM_ATTR OpenthermProtocol::stop_timer_() {
  InterruptLock const lock;
  timer_pause(this->timer_group_, this->timer_idx_);
  timer_set_counter_value(this->timer_group_, this->timer_idx_, 0);
}
#endif

}  // namespace opentherm_slave
}  // namespace esphome
