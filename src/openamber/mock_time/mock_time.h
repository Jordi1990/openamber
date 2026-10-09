#pragma once

#include "esphome/components/time/real_time_clock.h"

namespace esphome {
namespace mock_time {

class MockTime : public time::RealTimeClock {
 public:
  void setup() override {
    // Synchronize to a valid timestamp (e.g. 2026-10-09 12:00:00 UTC)
    this->synchronize_epoch_(1791547200);
  }

  void update() override {}
};

}  // namespace mock_time
}  // namespace esphome
