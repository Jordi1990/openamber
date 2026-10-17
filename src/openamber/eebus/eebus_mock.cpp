#include "eebus_mock.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome {
namespace eebus {

static const char *const TAG = "eebus";
static EEBusComponent *global_eebus_component = nullptr;

EEBusComponent::EEBusComponent() {
  global_eebus_component = this;
}

EEBusComponent::~EEBusComponent() {
  if (global_eebus_component == this) {
    global_eebus_component = nullptr;
  }
}

void EEBusComponent::record_action(const std::string &action) {
  this->last_action_ = action;
  if (this->last_action_sensor_ != nullptr) {
    this->last_action_sensor_->publish_state(action);
  }
  ESP_LOGI(TAG, "EEBUS [MOCK] action: %s", action.c_str());
}

void EEBusComponent::start_eebus_subsystem() {
  ESP_LOGI(TAG, "EEBus [MOCK] subsystem started.");
  if (this->status_sensor_ != nullptr) {
    this->status_sensor_->publish_state("Disconnected");
  }
  if (this->connected_sensor_ != nullptr) {
    this->connected_sensor_->publish_state(false);
  }
  if (this->last_action_sensor_ != nullptr) {
    this->last_action_sensor_->publish_state("Initialization complete");
  }
  if (this->power_limit_sensor_ != nullptr) {
    this->power_limit_sensor_->publish_state(NAN);
  }
  if (this->ski_sensor_ != nullptr) {
    this->ski_sensor_->publish_state("MOCK-0123456789ABCDEF0123456789ABCDEF01234567");
  }
  if (this->ems_sensor_ != nullptr) {
    this->ems_sensor_->publish_state("–");
  }
}

void EEBusComponent::stop_eebus_subsystem() {
  ESP_LOGI(TAG, "EEBus [MOCK] subsystem stopped.");
  if (this->is_boost_requested_) {
    if (this->apply_optional_) {
      this->apply_optional_(false);
    }
    this->is_boost_requested_ = false;
  }
  if (this->is_dim_requested_) {
    if (this->apply_dim_) {
      this->apply_dim_(false);
    }
    this->is_dim_requested_ = false;
  }
  if (this->is_limit_active_) {
    if (this->apply_limit_) {
      this->apply_limit_(false, 0.0f);
    }
    this->is_limit_active_ = false;
  }
  if (this->status_sensor_ != nullptr) {
    this->status_sensor_->publish_state("Disabled");
  }
  if (this->connected_sensor_ != nullptr) {
    this->connected_sensor_->publish_state(false);
  }
  if (this->last_action_sensor_ != nullptr) {
    this->last_action_sensor_->publish_state("Disabled");
  }
  if (this->power_limit_sensor_ != nullptr) {
    this->power_limit_sensor_->publish_state(NAN);
  }
  if (this->ski_sensor_ != nullptr) {
    this->ski_sensor_->publish_state("–");
  }
  if (this->ems_sensor_ != nullptr) {
    this->ems_sensor_->publish_state("–");
  }
}

void EEBusComponent::setup() {
  this->last_enabled_ = this->is_enabled();
  if (!this->last_enabled_) {
    this->stop_eebus_subsystem();
    return;
  }
  this->start_eebus_subsystem();
}

void EEBusComponent::loop() {
  bool enabled = this->is_enabled();
  if (enabled != this->last_enabled_) {
    this->last_enabled_ = enabled;
    if (enabled) {
      this->start_eebus_subsystem();
    } else {
      this->stop_eebus_subsystem();
    }
  }
}

void EEBusComponent::update() {
  // Periodic update
}

void EEBusComponent::disconnect_ems() {
  ESP_LOGI(TAG, "EEBus [MOCK] Disconnect requested");
  this->fake_set_connected(false);
}

void EEBusComponent::fake_request_boost(bool active) {
  if (!this->is_enabled()) {
    ESP_LOGW(TAG, "EEBus [MOCK] Cannot request boost: EEBUS is disabled");
    return;
  }
  if (!this->enable_ohpcf_) {
    ESP_LOGW(TAG, "EEBus [MOCK] Cannot request boost: OHPCF is disabled");
    return;
  }
  this->is_boost_requested_ = active;
  if (this->apply_optional_) {
    bool ok = this->apply_optional_(active);
    ESP_LOGI(TAG, "EEBus [MOCK] apply_optional(%d) -> %d", active, ok);
  }
  if (active) {
    if (this->connected_sensor_ != nullptr && !this->connected_sensor_->state) {
      this->fake_set_connected(true, "EVCC (192.168.1.100)");
    }
    this->record_action("EMS Boost started (OHPCF)");
    if (this->status_sensor_ != nullptr) {
      this->status_sensor_->publish_state("Boost active");
    }
  } else {
    this->record_action("EMS Boost ended (OHPCF)");
    if (this->status_sensor_ != nullptr) {
      bool conn = this->connected_sensor_ != nullptr && this->connected_sensor_->state;
      this->status_sensor_->publish_state(conn ? "Normal operation" : "Disconnected");
    }
  }
}

void EEBusComponent::fake_request_dim(bool active) {
  if (!this->is_enabled()) {
    ESP_LOGW(TAG, "EEBus [MOCK] Cannot request dim: EEBUS is disabled");
    return;
  }
  this->is_dim_requested_ = active;
  if (this->apply_dim_) {
    bool ok = this->apply_dim_(active);
    ESP_LOGI(TAG, "EEBus [MOCK] apply_dim(%d) -> %d", active, ok);
  }
  if (this->apply_limit_) {
    this->apply_limit_(active, 0.0f);
  }
  if (active) {
    if (this->connected_sensor_ != nullptr && !this->connected_sensor_->state) {
      this->fake_set_connected(true, "EVCC (192.168.1.100)");
    }
    this->record_action("Grid Dim active (LPC §14a)");
    if (this->status_sensor_ != nullptr) {
      this->status_sensor_->publish_state("Dimmed (§14a)");
    }
    if (this->power_limit_sensor_ != nullptr) {
      this->power_limit_sensor_->publish_state(0.0f);
    }
  } else {
    this->record_action("Grid Dim released (LPC §14a)");
    if (this->status_sensor_ != nullptr) {
      bool conn = this->connected_sensor_ != nullptr && this->connected_sensor_->state;
      this->status_sensor_->publish_state(conn ? "Normal operation" : "Disconnected");
    }
    if (this->power_limit_sensor_ != nullptr) {
      this->power_limit_sensor_->publish_state(NAN);
    }
  }
}

void EEBusComponent::fake_set_power_limit(float limit_w) {
  if (!this->is_enabled()) {
    ESP_LOGW(TAG, "EEBus [MOCK] Cannot set limit: EEBUS is disabled");
    return;
  }
  bool active = (limit_w > 0.0f);
  this->is_limit_active_ = active;
  if (this->apply_limit_) {
    bool ok = this->apply_limit_(active, limit_w);
    ESP_LOGI(TAG, "EEBus [MOCK] apply_limit(%d, %.0f) -> %d", active, limit_w, ok);
  }
  if (active) {
    if (this->connected_sensor_ != nullptr && !this->connected_sensor_->state) {
      this->fake_set_connected(true, "EVCC (192.168.1.100)");
    }
    char act_buf[64];
    snprintf(act_buf, sizeof(act_buf), "Power limit set to %.0f W (LPC)", limit_w);
    this->record_action(act_buf);
    if (this->status_sensor_ != nullptr) {
      char st_buf[64];
      snprintf(st_buf, sizeof(st_buf), "Limited to %.0f W", limit_w);
      this->status_sensor_->publish_state(st_buf);
    }
    if (this->power_limit_sensor_ != nullptr) {
      this->power_limit_sensor_->publish_state(limit_w);
    }
  } else {
    this->record_action("Power limit removed (LPC)");
    if (this->status_sensor_ != nullptr) {
      bool conn = this->connected_sensor_ != nullptr && this->connected_sensor_->state;
      this->status_sensor_->publish_state(conn ? "Normal operation" : "Disconnected");
    }
    if (this->power_limit_sensor_ != nullptr) {
      this->power_limit_sensor_->publish_state(NAN);
    }
  }
}

void EEBusComponent::fake_set_connected(bool connected, const std::string &ems_name) {
  if (!this->is_enabled()) {
    ESP_LOGW(TAG, "EEBus [MOCK] Cannot change connection: EEBUS is disabled");
    return;
  }
  if (connected) {
    if (this->connected_sensor_ != nullptr) {
      this->connected_sensor_->publish_state(true);
    }
    if (this->ems_sensor_ != nullptr) {
      this->ems_sensor_->publish_state(ems_name);
    }
    this->record_action("Connected to " + ems_name);
    if (this->status_sensor_ != nullptr) {
      this->status_sensor_->publish_state("Normal operation");
    }
  } else {
    if (this->connected_sensor_ != nullptr) {
      this->connected_sensor_->publish_state(false);
    }
    if (this->ems_sensor_ != nullptr) {
      this->ems_sensor_->publish_state("–");
    }
    if (this->power_limit_sensor_ != nullptr) {
      this->power_limit_sensor_->publish_state(NAN);
    }
    this->record_action("Disconnected from EMS");
    if (this->status_sensor_ != nullptr) {
      this->status_sensor_->publish_state("Disconnected");
    }
    if (this->is_boost_requested_) {
      if (this->apply_optional_) {
        this->apply_optional_(false);
      }
      this->is_boost_requested_ = false;
    }
    if (this->is_dim_requested_) {
      if (this->apply_dim_) {
        this->apply_dim_(false);
      }
      this->is_dim_requested_ = false;
    }
    if (this->is_limit_active_) {
      if (this->apply_limit_) {
        this->apply_limit_(false, 0.0f);
      }
      this->is_limit_active_ = false;
    }
  }
}

void eebus_disconnect() {
  if (global_eebus_component != nullptr) {
    global_eebus_component->disconnect_ems();
  }
}

void fake_eebus_boost(bool active) {
  if (global_eebus_component != nullptr) {
    global_eebus_component->fake_request_boost(active);
  }
}

void fake_eebus_dim(bool active) {
  if (global_eebus_component != nullptr) {
    global_eebus_component->fake_request_dim(active);
  }
}

void fake_eebus_limit(float limit_w) {
  if (global_eebus_component != nullptr) {
    global_eebus_component->fake_set_power_limit(limit_w);
  }
}

void fake_eebus_connected(bool connected, const std::string &ems_name) {
  if (global_eebus_component != nullptr) {
    global_eebus_component->fake_set_connected(connected, ems_name);
  }
}

}  // namespace eebus
}  // namespace esphome
