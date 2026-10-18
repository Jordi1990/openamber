#pragma once

#include "esphome/components/time/real_time_clock.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace mock_time {

class MockTime : public time::RealTimeClock {
 public:
  void setup() override {
    start_millis_ = millis_64();
    // Synchronize to a valid timestamp (e.g. 2026-10-09 12:00:00 UTC)
    this->synchronize_epoch_(1791547200);
    auto t = this->now();
    ESP_LOGI("mock_time", "MockTime setup: %04d-%02d-%02d %02d:%02d:%02d, day_of_week=%d, ts=%lld",
             t.year, t.month, t.day_of_month, t.hour, t.minute, t.second, t.day_of_week,
             (long long)this->timestamp_now());
  }

  time_t timestamp_now() {
    return 1791547200 + static_cast<time_t>((millis_64() - start_millis_) / 1000ULL);
  }

  ESPTime now() {
    time_t epoch = this->timestamp_now();
#ifdef USE_TIME_TIMEZONE
    struct tm local_tm;
    if (epoch_to_local_tm(epoch, time::get_global_tz(), &local_tm)) {
      return ESPTime::from_c_tm(&local_tm, epoch);
    }
    return ESPTime::from_epoch_utc(epoch);
#else
    return ESPTime::from_epoch_local(epoch);
#endif
  }

  ESPTime utcnow() { return ESPTime::from_epoch_utc(this->timestamp_now()); }

  void update() override {}

 protected:
  uint64_t start_millis_{0};
};

}  // namespace mock_time
}  // namespace esphome
