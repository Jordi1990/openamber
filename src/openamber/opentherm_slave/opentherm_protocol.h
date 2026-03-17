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
#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#ifdef USE_ESP32
#include "driver/timer.h"
#include "esp_err.h"
#endif

namespace esphome {
namespace opentherm_slave {

enum class ProtocolMode : uint8_t {
  IDLE = 0,
  LISTENING = 1,
  RECEIVING = 2,
  RECEIVED = 3,
  WRITING = 4,
  SENT = 5,
  ERROR_PROTOCOL = 8,
  ERROR_TIMEOUT = 9,
  ERROR_TIMER = 10,
};

enum class ProtocolErrorType : uint8_t {
  NO_ERROR = 0,
  NO_TRANSITION = 1,
  INVALID_STOP_BIT = 2,
  PARITY_ERROR = 3,
  NO_CHANGE_TOO_LONG = 4,
  INVALID_MESSAGE = 5,
};

static const uint8_t RX_STOP_BIT_POS = 32;
static const uint8_t TX_STOP_BIT_POS = 33;
static const int32_t LISTEN_GUARD_TICKS = 15;
// Keep start qualification permissive; noisy false starts are filtered in RECEIVING.
static const uint8_t LISTEN_MIN_LOW_TICKS = 1;
static const uint8_t FALSE_START_MAX_BIT_POS = 1;
// Mid-bit transition threshold. Wider values regressed the previously working RX/TX path.
static const uint8_t MID_BIT_CAPTURE_MAX = 0x0F;

class OpenthermProtocol {
 public:
  OpenthermProtocol(InternalGPIOPin *in_pin, InternalGPIOPin *out_pin, int32_t device_timeout_ms = -1)
      : in_pin_(in_pin),
        out_pin_(out_pin),
        isr_in_pin_(in_pin->to_isr()),
        isr_out_pin_(out_pin->to_isr()),
#ifdef USE_ESP32
        timer_group_(TIMER_GROUP_0),
        timer_idx_(TIMER_0),
#endif
        device_timeout_(device_timeout_ms) {}

  bool initialize();
  void set_event_component(Component *component) { this->event_component_ = component; }
  void listen();
  void send(const OpenthermData &data);
  bool get_message(OpenthermData &data);
  void stop();

  bool has_message() const { return this->mode_ == ProtocolMode::RECEIVED; }
  bool is_idle() const { return this->mode_ == ProtocolMode::IDLE; }
  bool is_sent() const { return this->mode_ == ProtocolMode::SENT; }
  bool is_error() const {
    return this->mode_ == ProtocolMode::ERROR_TIMEOUT ||
           this->mode_ == ProtocolMode::ERROR_PROTOCOL ||
           this->mode_ == ProtocolMode::ERROR_TIMER;
  }

  ProtocolMode get_mode() const { return this->mode_; }
  ProtocolErrorType get_error_type() const { return this->error_type_; }
  uint32_t get_capture() const { return this->capture_; }
  uint8_t get_clock() const { return this->clock_; }
  int8_t get_bit_pos() const { return this->bit_pos_; }
  uint32_t get_raw_data() const { return this->data_; }

  static bool IRAM_ATTR timer_isr(OpenthermProtocol *arg) {
    if (arg->mode_ == ProtocolMode::LISTENING) {
      if (arg->timeout_counter_ == 0) {
        arg->mode_ = ProtocolMode::ERROR_TIMEOUT;
        arg->stop_timer_();
        arg->notify_event_component_();
        return false;
      }

      if (arg->listen_guard_ticks_ > 0) {
        bool raw_value = arg->isr_in_pin_.digital_read();
        bool value = raw_value == arg->idle_level_;
        arg->listen_last_value_ = value;
        arg->listen_low_ticks_ = value ? 0 : 1;
        arg->listen_guard_ticks_--;
        if (arg->timeout_counter_ > 0) {
          arg->timeout_counter_--;
        }
        return false;
      }

      bool raw_value = arg->isr_in_pin_.digital_read();
      bool value = raw_value == arg->idle_level_;
      bool had_low_phase = arg->listen_low_ticks_ >= LISTEN_MIN_LOW_TICKS;
      if (!value) {
        if (arg->listen_low_ticks_ < 0xFF) {
          arg->listen_low_ticks_++;
        }
      } else {
        if (had_low_phase) {
          arg->listen_last_value_ = value;
          arg->listen_low_ticks_ = 0;
          arg->read_();
          if (arg->timeout_counter_ > 0) {
            arg->timeout_counter_--;
          }
          return false;
        }
        arg->listen_low_ticks_ = 0;
      }
      arg->listen_last_value_ = value;

      if (arg->timeout_counter_ > 0) {
        arg->timeout_counter_--;
      }
      return false;
    }

    if (arg->mode_ == ProtocolMode::RECEIVING) {
      bool raw_value = arg->isr_in_pin_.digital_read();
      bool value = raw_value == arg->idle_level_;
      uint8_t last = arg->capture_ & 1;

      if (value != last) {
        if (arg->clock_ == 1 && arg->capture_ > MID_BIT_CAPTURE_MAX) {
          if (arg->bit_pos_ <= FALSE_START_MAX_BIT_POS) {
            arg->resume_listening_from_isr_(value);
            return false;
          }

          arg->mode_ = ProtocolMode::ERROR_PROTOCOL;
          arg->error_type_ = ProtocolErrorType::NO_TRANSITION;
          arg->stop_timer_();
          arg->notify_event_component_();
          return false;
        }

        if (arg->clock_ == 1 || arg->capture_ > MID_BIT_CAPTURE_MAX) {
          if (arg->bit_pos_ == RX_STOP_BIT_POS) {
            auto stop_error = arg->verify_stop_bit_(value);
            if (stop_error == ProtocolErrorType::NO_ERROR) {
              arg->mode_ = ProtocolMode::RECEIVED;
              arg->stop_timer_();
              arg->notify_event_component_();
              return false;
            }

            arg->mode_ = ProtocolMode::ERROR_PROTOCOL;
            arg->error_type_ = stop_error;
            arg->stop_timer_();
            arg->notify_event_component_();
            return false;
          }

          arg->bit_read_(value);
          arg->clock_ = 0;
        } else {
          arg->clock_ = 1;
        }
        arg->capture_ = 1;
      } else if (arg->capture_ > 0xFF) {
        // No decoded bit yet: treat as a false start and silently resume listening.
        if (arg->bit_pos_ <= FALSE_START_MAX_BIT_POS) {
          arg->resume_listening_from_isr_(value);
          return false;
        }

        arg->mode_ = ProtocolMode::ERROR_PROTOCOL;
        arg->error_type_ = ProtocolErrorType::NO_CHANGE_TOO_LONG;
        arg->stop_timer_();
        arg->notify_event_component_();
        return false;
      }

      arg->capture_ = (arg->capture_ << 1) | value;
      return false;
    }

    if (arg->mode_ != ProtocolMode::WRITING) {
      return false;
    }

    if (arg->bit_pos_ == TX_STOP_BIT_POS || arg->bit_pos_ == 0) {
      arg->write_bit_(1, arg->clock_);
    } else {
      arg->write_bit_((arg->data_ >> (arg->bit_pos_ - 1)) & 0x01, arg->clock_);
    }

    if (arg->clock_ == 0) {
      if (arg->bit_pos_ <= 0) {
        arg->mode_ = ProtocolMode::SENT;
        arg->stop_timer_();
        arg->notify_event_component_();
        return false;
      }
      arg->bit_pos_--;
      arg->clock_ = 1;
    } else {
      arg->clock_ = 0;
    }
    return false;
  }

 protected:
  InternalGPIOPin *in_pin_;
  InternalGPIOPin *out_pin_;
  ISRInternalGPIOPin isr_in_pin_;
  ISRInternalGPIOPin isr_out_pin_;

#ifdef USE_ESP32
  timer_group_t timer_group_;
  timer_idx_t timer_idx_;
  esp_err_t timer_error_{ESP_OK};
  bool init_esp32_timer_();
  void start_esp32_timer_(uint64_t alarm_value);
#endif

  volatile ProtocolMode mode_{ProtocolMode::IDLE};
  volatile ProtocolErrorType error_type_{ProtocolErrorType::NO_ERROR};
  volatile uint32_t capture_{0};
  volatile uint8_t clock_{0};
  volatile uint32_t data_{0};
  volatile int8_t bit_pos_{0};
  volatile int32_t timeout_counter_{-1};
  volatile int32_t listen_guard_ticks_{0};
  volatile bool listen_last_value_{false};
  volatile uint8_t listen_low_ticks_{0};
  volatile bool idle_level_{true};
  int32_t device_timeout_{-1};
  Component *event_component_{nullptr};

  void IRAM_ATTR read_() {
    this->data_ = 0;
    this->bit_pos_ = 0;
    this->mode_ = ProtocolMode::RECEIVING;
    this->capture_ = 1;
    // The start-bit mid transition was already consumed by LISTENING.
    this->clock_ = 0;
    this->start_read_timer_();
  }

  void IRAM_ATTR resume_listening_from_isr_(bool value) {
    this->mode_ = ProtocolMode::LISTENING;
    this->error_type_ = ProtocolErrorType::NO_ERROR;
    this->data_ = 0;
    this->bit_pos_ = 0;
    this->capture_ = 0;
    this->clock_ = 0;
    this->listen_last_value_ = value;
    this->listen_low_ticks_ = value ? 0 : 1;
  }

  void IRAM_ATTR bit_read_(uint8_t value) {
    this->data_ = (this->data_ << 1) | value;
    this->bit_pos_++;
  }

  ProtocolErrorType IRAM_ATTR verify_stop_bit_(uint8_t value) {
    if (!value) {
      return ProtocolErrorType::INVALID_STOP_BIT;
    }
    return parity(this->data_) == 0 ? ProtocolErrorType::NO_ERROR : ProtocolErrorType::PARITY_ERROR;
  }

  void IRAM_ATTR write_bit_(uint8_t high, uint8_t clock) {
    if (clock == 1) {
      this->isr_out_pin_.digital_write(!high);
    } else {
      this->isr_out_pin_.digital_write(high);
    }
  }

  void IRAM_ATTR notify_event_component_() {
    if (this->event_component_ != nullptr) {
      this->event_component_->enable_loop_soon_any_context();
    }
  }

  void IRAM_ATTR start_read_timer_() {
#ifdef USE_ESP32
    this->start_esp32_timer_(200);
#endif
  }

  void IRAM_ATTR start_write_timer_() {
#ifdef USE_ESP32
    this->start_esp32_timer_(500);
#endif
  }

  void stop_timer_();
};

}  // namespace opentherm_slave
}  // namespace esphome
