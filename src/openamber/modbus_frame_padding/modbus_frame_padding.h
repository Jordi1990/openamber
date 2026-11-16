#pragma once

#include <vector>

#include "esphome/core/component.h"
#include "esphome/components/uart/uart_component.h"

namespace esphome::modbus_frame_padding {

// Wraps a real UART and forwards everything to it. After a write that starts with one of the
// configured Modbus addresses, it sends one extra pad byte, so the CRC is no longer the last byte
// on the wire. See __init__.py and issue #322 for why the outdoor unit needs this.
class ModbusFramePadding : public uart::UARTComponent, public Component {
 public:
  explicit ModbusFramePadding(uart::UARTComponent *parent) : parent_(parent) {}

  void add_address(uint8_t address) { this->addresses_.push_back(address); }
  void set_pad_byte(uint8_t pad_byte) { this->pad_byte_ = pad_byte; }

  // Between the real UART (BUS) and the Modbus hub (BUS - 1), which reads the line settings.
  float get_setup_priority() const override { return setup_priority::BUS - 0.5f; }
  void setup() override { this->sync_settings_(); }
  void dump_config() override;

  void write_array(const uint8_t *data, size_t len) override;
  bool peek_byte(uint8_t *data) override { return this->parent_->peek_byte(data); }
  bool read_array(uint8_t *data, size_t len) override { return this->parent_->read_array(data, len); }
  size_t available() override { return this->parent_->available(); }
  size_t available_for_write() override { return this->parent_->available_for_write(); }
  uart::UARTFlushResult flush() override { return this->parent_->flush(); }
  bool is_connected() override { return this->parent_->is_connected(); }
  void set_flush_timeout(uint32_t flush_timeout_ms) override { this->parent_->set_flush_timeout(flush_timeout_ms); }
  void set_rx_full_threshold(size_t rx_full_threshold) override {
    this->parent_->set_rx_full_threshold(rx_full_threshold);
    this->sync_settings_();
  }
  void set_rx_timeout(size_t rx_timeout) override {
    this->parent_->set_rx_timeout(rx_timeout);
    this->sync_settings_();
  }
#if defined(USE_ESP8266) || defined(USE_ESP32)
  void load_settings(bool dump_config) override { this->parent_->load_settings(dump_config); }
#endif

  uint32_t get_pad_count() const { return this->pad_count_; }

 protected:
  void check_logger_conflict() override {}
  // The getters on UARTComponent are not virtual, so mirror the real UART's line settings here.
  void sync_settings_();
  bool should_pad_(const uint8_t *data, size_t len) const;

  uart::UARTComponent *parent_;
  std::vector<uint8_t> addresses_;
  uint8_t pad_byte_{0xFF};
  uint32_t pad_count_{0};
};

}  // namespace esphome::modbus_frame_padding
