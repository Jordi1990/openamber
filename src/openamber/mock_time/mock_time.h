#pragma once

#include "esphome/components/time/real_time_clock.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace mock_time {

class MockTime : public time::RealTimeClock {
 public:
  /// Fixed start of the simulated wall clock: Friday 2026-10-09 12:00:00 UTC.
  static constexpr time_t START_EPOCH = 1791547200;
  /// The instance, so the test bridge can set the wall clock.
  static inline MockTime *instance = nullptr;

  void setup() override {
    instance = this;
    start_millis_ = millis_64();
    // Synchronize to a valid timestamp (e.g. 2026-10-09 12:00:00 UTC)
    this->synchronize_epoch_(START_EPOCH);
    auto t = this->now();
    ESP_LOGI("mock_time", "MockTime setup: %04d-%02d-%02d %02d:%02d:%02d, day_of_week=%d, ts=%lld",
             t.year, t.month, t.day_of_month, t.hour, t.minute, t.second, t.day_of_week,
             (long long)this->timestamp_now());
  }

  time_t timestamp_now() {
    return base_epoch_ + static_cast<time_t>((millis_64() - start_millis_) / 1000ULL);
  }

  /// Set the wall clock (date and time) without touching millis(), so controller timers keep running normally.
  void set_clock(time_t epoch) {
    base_epoch_ = epoch;
    start_millis_ = millis_64();
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
  time_t base_epoch_{START_EPOCH};
};

}  // namespace mock_time
}  // namespace esphome
