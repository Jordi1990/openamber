#include "modbus_frame_padding.h"

#include <algorithm>

#include "esphome/core/log.h"

namespace esphome::modbus_frame_padding {

static const char *const TAG = "modbus_frame_padding";

// Shortest Modbus RTU frame: address, function code, CRC (2 bytes).
static constexpr size_t MIN_FRAME_SIZE = 4;

void ModbusFramePadding::sync_settings_() {
  this->set_baud_rate(this->parent_->get_baud_rate());
  this->set_data_bits(this->parent_->get_data_bits());
  this->set_stop_bits(this->parent_->get_stop_bits());
  this->set_parity(this->parent_->get_parity());
  this->set_rx_buffer_size(this->parent_->get_rx_buffer_size());
  this->rx_full_threshold_ = this->parent_->get_rx_full_threshold();
  this->rx_timeout_ = this->parent_->get_rx_timeout();
}

bool ModbusFramePadding::should_pad_(const uint8_t *data, size_t len) const {
  if (len < MIN_FRAME_SIZE)
    return false;
  return std::find(this->addresses_.begin(), this->addresses_.end(), data[0]) != this->addresses_.end();
}

void ModbusFramePadding::write_array(const uint8_t *data, size_t len) {
  if (!this->should_pad_(data, len)) {
    this->parent_->write_array(data, len);
    return;
  }
  // One write keeps the pad byte directly behind the CRC, without an idle gap in between.
  std::vector<uint8_t> buffer(data, data + len);
  buffer.push_back(this->pad_byte_);
  this->parent_->write_array(buffer.data(), buffer.size());
  this->pad_count_++;
}

void ModbusFramePadding::dump_config() {
  ESP_LOGCONFIG(TAG, "Modbus frame padding:");
  ESP_LOGCONFIG(TAG, "  Pad byte: 0x%02X", this->pad_byte_);
  for (uint8_t address : this->addresses_)
    ESP_LOGCONFIG(TAG, "  Address: %u", address);
}

}  // namespace esphome::modbus_frame_padding
