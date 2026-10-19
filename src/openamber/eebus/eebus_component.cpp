/*
 * Native EEBUS component for ESPHome
 *
 * Copyright (C) 2025 Jordi Epema
 *
 * The SHIP/SPINE transport wires into ohpcf_/lpc_/mpc_/mdt_ servers;
 * the host application bridge callbacks (set_*) route into host controllers with
 * safety priority.
 */

#include "eebus_component.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

#include "mdns.h"
#include "eebus_ship.h"
#include "eebus_websocket.h"
#include "esphome/components/network/util.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <algorithm>
#include <cctype>

namespace esphome {
namespace eebus {

static const char *const TAG = "eebus";

void EEBusComponent::record_action(const std::string &action) {
  this->last_action_ = action;
  this->pending_action_state_ = action;
  this->pending_action_update_ = true;
  ESP_LOGI(TAG, "EEBUS action: %s", action.c_str());
}

void EEBusComponent::setup() {
  if (!this->is_enabled()) {
    ESP_LOGI(TAG, "EEBus component is disabled; skipping initialization.");
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
    return;
  }

  this->start_eebus_subsystem();
}

void EEBusComponent::start_eebus_subsystem() {
  if (this->initialized_)
    return;

  // Check if model is already known via callback
  if (this->read_model_) {
    std::string detected = this->read_model_();
    if (!detected.empty() && detected != "Onbekend model" && detected != "Unknown") {
      this->model_ = detected;
      this->device_sku_ = this->brand_ + "-" + detected;
      this->model_detected_ = true;
      ESP_LOGI(TAG, "EEBUS model via callback: %s", this->model_.c_str());
    } else {
      this->model_detected_ = false;
    }
  } else {
    this->model_detected_ = true;
  }

  ESP_LOGI(TAG, "EEBus component initializing (sku=%s brand=%s model=%s)",
           this->device_sku_.c_str(), this->brand_.c_str(), this->model_.c_str());
  this->init_certificate_and_keys();
  this->wire_use_cases();
  this->initialized_ = true;

  this->current_status_ = "Initializing...";
  if (this->status_sensor_ != nullptr) {
    this->status_sensor_->publish_state(this->current_status_);
  }
  if (this->connected_sensor_ != nullptr) {
    this->connected_sensor_->publish_state(false);
  }
  this->record_action("Starting");
  if (this->power_limit_sensor_ != nullptr) {
    this->power_limit_sensor_->publish_state(NAN);
  }
  if (this->ski_sensor_ != nullptr) {
    this->ski_sensor_->publish_state(this->cert_store_.get_ski());
  }
  if (this->ems_sensor_ != nullptr) {
    this->ems_sensor_->publish_state("–");
  }
}

void EEBusComponent::stop_eebus_subsystem() {
  if (!this->initialized_)
    return;
  ESP_LOGI(TAG, "EEBus component stopping (disabled)...");

  // Stop SHIP listener & mDNS
  this->ship_listener_.stop();
  eebus_mdns_stop();
  this->listener_started_ = false;
  this->mdns_advertised_ = false;
  this->model_detected_ = false;
  this->last_mdns_reannounce_ms_ = 0;

  // Release any active consumption limits / dimming / boost
  if (this->lpc_.is_dimmed()) {
    this->lpc_.release_limit();
  }
  if (this->apply_dim_) {
    this->apply_dim_(false);
  }
  if (this->apply_optional_) {
    this->apply_optional_(false);
  }

  this->last_connected_ = false;
  this->initialized_ = false;
  this->current_status_ = "Disabled";
  this->current_ems_ = "–";

  if (this->connected_sensor_ != nullptr) {
    this->connected_sensor_->publish_state(false);
  }
  if (this->status_sensor_ != nullptr) {
    this->status_sensor_->publish_state("Disabled");
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

void EEBusComponent::loop() {
  bool enabled = this->is_enabled();
  if (enabled && !this->initialized_) {
    this->start_eebus_subsystem();
  } else if (!enabled && this->initialized_) {
    this->stop_eebus_subsystem();
  }

  if (this->initialized_) {
    // Process queued inbound SPINE messages on main ESPHome thread
    if (this->inbound_mutex_ != nullptr) {
      std::vector<std::string> to_process;
      if (xSemaphoreTake(static_cast<SemaphoreHandle_t>(this->inbound_mutex_), pdMS_TO_TICKS(10)) == pdTRUE) {
        to_process.swap(this->inbound_spine_queue_);
        xSemaphoreGive(static_cast<SemaphoreHandle_t>(this->inbound_mutex_));
      }
      for (const auto &msg : to_process) {
        std::string reply = this->node_.handle_inbound(msg);
        ESP_LOGI(TAG, "SPINE handled in loop -> reply size=%u", static_cast<unsigned>(reply.size()));
        if (!reply.empty()) {
          this->ship_listener_.send_ship_data(reply);
        }
      }
    }

    bool is_conn = this->ship_listener_.is_connected();
    if (is_conn != this->last_connected_) {
      this->check_connection_state();
    }
  }

  if (this->pending_remote_changed_) {
    this->pending_remote_changed_ = false;
    this->update_ems_status();
  }
  if (this->pending_action_update_) {
    this->pending_action_update_ = false;
    if (this->last_action_sensor_ != nullptr) {
      this->last_action_sensor_->publish_state(this->pending_action_state_);
    }
  }
  if (this->pending_connected_update_) {
    this->pending_connected_update_ = false;
    if (this->connected_sensor_ != nullptr) {
      this->connected_sensor_->publish_state(this->pending_connected_state_);
    }
  }
  if (this->pending_status_update_) {
    this->pending_status_update_ = false;
    if (this->status_sensor_ != nullptr) {
      this->status_sensor_->publish_state(this->pending_status_state_);
    }
  }
  if (this->pending_limit_update_) {
    this->pending_limit_update_ = false;
    if (this->power_limit_sensor_ != nullptr) {
      this->power_limit_sensor_->publish_state(this->pending_limit_state_);
    }
  }
  if (this->pending_ems_update_) {
    this->pending_ems_update_ = false;
    if (this->ems_sensor_ != nullptr) {
      this->ems_sensor_->publish_state(this->pending_ems_state_);
    }
  }
}

void EEBusComponent::update() {
  bool enabled = this->is_enabled();
  if (!enabled) {
    if (this->initialized_) {
      this->stop_eebus_subsystem();
    }
    return;
  }

  if (!this->initialized_) {
    this->start_eebus_subsystem();
  }

  // Check / update model from callback
  if (this->read_model_) {
    std::string detected = this->read_model_();
    if (!detected.empty() && detected != "Onbekend model" && detected != "Unknown") {
      if (this->model_ != detected) {
        this->model_ = detected;
        this->device_sku_ = this->brand_ + "-" + detected;
        this->node_.set_model(this->model_);
        ESP_LOGI(TAG, "EEBUS model configured via callback: %s (sku=%s)",
                 this->model_.c_str(), this->device_sku_.c_str());
        this->mdns_advertised_ = false;  // Force re-advertisement with new model
      }
      this->model_detected_ = true;
    }
  } else {
    this->model_detected_ = true;
  }

  if (!this->listener_started_) {
    this->listener_started_ = true;
    this->start_ship_listener();
  }

  // Update action message
  if (!this->model_detected_) {
    if (this->last_action_ == "Starting" || this->last_action_ == "Initialization") {
      this->record_action("Waiting for device model...");
    }
  } else if (this->last_action_ == "Waiting for device model..." || this->last_action_ == "Starting") {
    this->record_action("EEBUS listener started, waiting for EMS");
  }

  // Only advertise mDNS once network/WiFi is connected AND model is known!
  if (network::is_connected() && this->model_detected_) {
    uint32_t now = millis();
    bool should_advertise = !this->mdns_advertised_ ||
                            (!this->ship_listener_.is_connected() && (now - this->last_mdns_reannounce_ms_ >= 45000UL));
    if (should_advertise) {
      if (this->advertise_mdns()) {
        this->mdns_advertised_ = true;
        this->last_mdns_reannounce_ms_ = now;
      }
    }
  } else {
    this->mdns_advertised_ = false;
  }

  this->check_connection_state();

  // Refresh MPC (power) + MDT (temperature) from host application callbacks.
  float power_w = 0.0f;
  if (this->read_power_) {
    power_w = this->read_power_();
    this->mpc_.set_power_w(power_w);
  }
  float temp_c = 0.0f;
  if (this->enable_mdt_ && this->read_dhw_temp_) {
    temp_c = this->read_dhw_temp_();
    this->mdt_.set_temperature_c(temp_c);
  }
  if (this->read_voltage_) {
    this->mpc_.set_voltage_v(this->read_voltage_());
  }
  if (this->read_current_) {
    this->mpc_.set_current_a(this->read_current_());
  }
  if (this->read_energy_) {
    float en = this->read_energy_();
    if (en > 0.0f) {
      this->mpc_.set_energy_wh(en);
    }
  }

  // Refresh dynamic power estimate if configured (OHPCF).
  if (this->enable_ohpcf_) {
    if (this->read_power_estimate_) {
      float est = this->read_power_estimate_();
      if (est >= 0.0f) {
        this->ohpcf_.set_requested_power(est);
        if (fabsf(est - this->last_power_estimate_w_) >= 50.0f) {
          this->last_power_estimate_w_ = est;
          if (this->ship_listener_.is_connected() && this->node_.has_subscriptions()) {
            ESP_LOGD(TAG, "EEBUS power estimate updated to %.0f W", est);
            this->node_.notify_ohpcf();
          }
        }
      }
    }

    // If OHPCF is RUNNING, verify if host boost mode is still active
    if (this->ohpcf_.get_state() == OhpcfState::RUNNING) {
      uint32_t now = millis();
      // Allow a 60-second grace period after boost starts before checking if boost ended
      if (now - this->last_boost_start_ms_ >= 60000UL) {
        if (this->is_boost_active_ && !this->is_boost_active_()) {
          ESP_LOGI(TAG, "Boost mode ended externally; updating EEBUS OHPCF state to AVAILABLE (inactive)");
          this->record_action("Boost ended");
          this->ohpcf_.abort();
        }
      }
    }
  }

  // If connected, push measurement updates periodically (every 5s)
  if (this->ship_listener_.is_connected() && this->node_.has_remote_device()) {
    uint32_t now = millis();
    if (now - this->last_measurement_notify_ms_ >= 5000) {
      this->last_measurement_notify_ms_ = now;
      this->node_.notify_measurements(power_w, temp_c);
    }
  }

  // Push OHPCF state periodically (every 10s) when connected
  if (this->enable_ohpcf_ && this->ship_listener_.is_connected() && this->node_.has_remote_device()) {
    uint32_t now = millis();
    if (now - this->last_ohpcf_notify_ms_ >= 10000) {
      this->last_ohpcf_notify_ms_ = now;
      this->node_.notify_ohpcf();
    }
  }

  // Push Use Case announcements periodically (every 30s) so EVCC never loses compressor entity
  if (this->ship_listener_.is_connected() && this->node_.has_remote_device()) {
    uint32_t now = millis();
    if (now - this->last_use_case_notify_ms_ >= 30000) {
      this->last_use_case_notify_ms_ = now;
      this->node_.notify_use_cases();
    }
  }

  this->check_failsafe();
}

void EEBusComponent::init_certificate_and_keys() {
  std::string cn = this->device_name_.empty() ? this->brand_ : this->device_name_;
  if (cn.empty()) cn = "EEBUS-Device";
  this->cert_store_.set_common_name(cn);
  if (!this->cert_store_.load_or_create())
    ESP_LOGE(TAG, "Failed to initialise EEBus certificate store");
  this->ship_id_ = this->cert_store_.get_ski();
}

void EEBusComponent::wire_use_cases() {
  if (this->enable_ohpcf_) {
    float initial_power = 0.0f;
    if (this->read_power_estimate_) {
      initial_power = this->read_power_estimate_();
    }
    this->ohpcf_.announce(initial_power, 600, 600);
    this->last_power_estimate_w_ = initial_power;

    this->ohpcf_.set_applier([this](bool enabled) -> bool {
      // Route the CEM's optional-consumption request through the bridge; the
      // bridge decides whether the request is granted (safety first).
      // An active LPC limit (e.g. §14a dim) takes precedence over an OHPCF
      // boost request: never switch to Boost when the limit is below the
      // power the boost would draw (always the case for a 0 W block).
      if (enabled && this->lpc_.is_dimmed() &&
          this->lpc_.get_limit_value_w() < this->ohpcf_.get_requested_power_estimate_w()) {
        this->record_action("EMS Boost rejected (LPC limit active)");
        return false;
      }
      if (this->apply_optional_) {
        bool ok = this->apply_optional_(enabled);
        if (!ok) {
          this->record_action("EMS Boost rejected (lockout/safety)");
          return false;
        }
      }
      if (enabled) {
        this->last_boost_start_ms_ = millis();
        this->record_action("EMS Boost started (OHPCF)");
      } else {
        this->record_action("EMS Boost ended (OHPCF)");
      }
      return true;
    });

    this->ohpcf_.set_change_callback([this](OhpcfState state) {
      this->node_.notify_ohpcf();
    });
  }

  this->lpc_.set_dimmer([this](bool dim) {
    // Dim -> SG-Ready block path, again safety-gated by the bridge.
    if (this->apply_dim_) {
      this->apply_dim_(dim);
    }
    // A running OHPCF boost cannot coexist with a block: report it as paused
    // so the CEM sees the real state (it may resume once the limit is lifted).
    if (dim && this->enable_ohpcf_ && this->ohpcf_.get_state() == OhpcfState::RUNNING) {
      this->ohpcf_.pause();
    }
    if (dim) {
      this->record_action("EMS Dim/Block active (LPC 0W)");
    } else {
      this->record_action("EMS Dim/Block released");
    }
  });

  this->lpc_.set_limit_applier([this](bool active, float limit_w) {
    if (this->apply_limit_) {
      this->apply_limit_(active, limit_w);
    }
    if (active) {
      char buf[64];
      snprintf(buf, sizeof(buf), "EMS Limit active: %.0f W", limit_w);
      this->record_action(buf);
    } else {
      this->record_action("EMS Limit released");
    }
  });

  this->lpc_.set_change_callback([this](bool active, float limit_w) {
    this->node_.notify_lpc(active, limit_w);
    this->pending_limit_state_ = active ? limit_w : NAN;
    this->pending_limit_update_ = true;
  });

  this->node_.set_ship_sender([this](const std::string &spine_json) -> bool {
    return this->ship_listener_.send_ship_data(spine_json);
  });

  this->node_.set_ohpcf(this->enable_ohpcf_ ? &this->ohpcf_ : nullptr);
  this->node_.set_lpc(&this->lpc_);
  this->node_.set_mpc(&this->mpc_);
  this->node_.set_mdt(this->enable_mdt_ ? &this->mdt_ : nullptr);
  this->node_.set_device_id(this->ship_id_);
  this->node_.set_brand(this->brand_);
  this->node_.set_model(this->model_);
  this->node_.set_device_name(this->device_name_.empty() ? this->brand_ : this->device_name_);
  this->node_.set_device_category(this->device_category_);
  this->node_.set_device_type(this->device_type_);
  this->node_.set_entity_type(this->entity_type_);
  this->node_.set_enable_ohpcf(this->enable_ohpcf_);
  this->node_.set_enable_mdt(this->enable_mdt_);
  this->node_.set_failsafe_duration_s(this->failsafe_duration_s_);
  this->node_.set_on_remote_device_changed([this](const std::string &) {
    this->pending_remote_changed_ = true;
  });

  this->ship_listener_.set_frame_handler([this](const std::vector<uint8_t> &frame)
                                             -> std::vector<uint8_t> {
    std::vector<uint8_t> out;  // WS-encoded bytes to send back (empty = none)
    auto send_data = [&out](const std::vector<uint8_t> &payload) {
      if (!payload.empty()) {
        if (payload[0] == SHIP_MSG_CONTROL || payload[0] == SHIP_MSG_DATA) {
          if (payload.size() < 128) {
            std::string raw(payload.begin() + 1, payload.end());
            ESP_LOGI(TAG, "SHIP SEND msg_type=%u payload=[%s]", payload[0], raw.c_str());
          } else {
            ESP_LOGI(TAG, "SHIP SEND msg_type=%u (%u bytes)", payload[0], static_cast<unsigned>(payload.size()));
          }
        } else {
          ESP_LOGI(TAG, "SHIP SEND msg_type=%u (%u bytes)", payload[0], static_cast<unsigned>(payload.size()));
        }
      }
      eebus_websocket_encode_server_frame(0x2 /*binary*/, payload, out);
    };

    ESP_LOGI(TAG, "SHIP frame (%u bytes, type=%u)", static_cast<unsigned>(frame.size()),
             frame.empty() ? 0 : frame[0]);

    uint8_t msg_type = 0;
    std::string json;
    if (!ship_decode_frame(frame, msg_type, json)) {
      ESP_LOGW(TAG, "Could not decode SHIP frame");
      return out;
    }
    if (json.size() < 128) {
      ESP_LOGI(TAG, "SHIP msg_type=%u json=[%s]", static_cast<unsigned>(msg_type), json.c_str());
    } else {
      ESP_LOGI(TAG, "SHIP msg_type=%u (%u bytes)", static_cast<unsigned>(msg_type), static_cast<unsigned>(json.size()));
    }

    if (msg_type == 0) {
      // SHIP CMI (connection management initiation): ShipInit = {0x00,0x00}.
      // As server we must reply with the same ShipInit, then the client sends
      // its HELLO/pairing phase.  (ship-go/model: MsgTypeInit=0.)
      send_data({0x00, 0x00});
      ESP_LOGI(TAG, "Replied to SHIP INIT (CMI) with ShipInit");
      return out;
    }

    if (msg_type == SHIP_MSG_DATA) {
      if (this->inbound_mutex_ != nullptr) {
        if (xSemaphoreTake(static_cast<SemaphoreHandle_t>(this->inbound_mutex_), pdMS_TO_TICKS(50)) == pdTRUE) {
          this->inbound_spine_queue_.push_back(json);
          xSemaphoreGive(static_cast<SemaphoreHandle_t>(this->inbound_mutex_));
        } else {
          ESP_LOGE(TAG, "Failed to acquire inbound mutex; dropped SPINE DATA frame");
        }
      }
      return out;
    }

    // Control (handshake) messages.
    if (json.find("connectionHello") != std::string::npos) {
      // Do not answer an abort; the peer is tearing the connection down.
      if (json.find("\"aborted\"") != std::string::npos) {
        ESP_LOGW(TAG, "Peer aborted handshake (connectionHello aborted)");
        this->ship_listener_.request_close();
        return out;
      }
      // HELLO: reply ready -> HELLO_OK on the peer, it then runs the protocol
      // handshake phase.
      send_data(ship_control_frame(
          "{\"connectionHello\":{\"phase\":\"ready\",\"waiting\":60000}}"));
      ESP_LOGI(TAG, "Replied to connectionHello (ready)");
      return out;
    }

    if (json.find("messageProtocolHandshake") != std::string::npos) {
      if (json.find("\"announceMax\"") != std::string::npos) {
        // PROT phase: peer announces its max protocol; select it back.
        // NOTE: HandshakeType MUST come first according to EEBUS SHIP schema order!
        send_data(ship_control_frame(
            "{\"messageProtocolHandshake\":{\"handshakeType\":\"select\","
            "\"version\":{\"major\":1,\"minor\":0},"
            "\"formats\":{\"format\":[\"JSON-UTF8\"]}}}"));
        ESP_LOGI(TAG, "Replied to messageProtocolHandshake (select)");
        return out;
      }
      // The peer's select echo confirms our choice; wait for peer to initiate next phase.
      ESP_LOGI(TAG, "Received messageProtocolHandshake confirm (select echo)");
      return out;
    }

    if (json.find("connectionPinState") != std::string::npos) {
      // PIN check phase: announce our own (no PIN required) state.
      send_data(ship_control_frame("{\"connectionPinState\":{\"pinState\":\"none\"}}"));
      ESP_LOGI(TAG, "Replied to connectionPinState (none)");
      return out;
    }

    if (json.find("accessMethodsRequest") != std::string::npos) {      // Respond with our SHIP ID so evcc/Venus can identify the device.
      std::string model = "{\"accessMethods\":{\"id\":\"" + this->ship_id_ + "\"}}";
      send_data(ship_control_frame(model));
      ESP_LOGI(TAG, "Responded to accessMethodsRequest (shipId=%s)", this->ship_id_.c_str());
      return out;
    } else if (json.find("accessMethods") != std::string::npos) {
      ESP_LOGI(TAG, "Received peer accessMethods: %s", json.c_str());
      return out;
    } else if (json.find("connectionClose") != std::string::npos) {
      ESP_LOGI(TAG, "Peer closed connection (connectionClose)");
      this->node_.reset_subscriptions();
      if (json.find("\"confirm\"") == std::string::npos) {
        send_data(ship_control_frame("{\"connectionClose\":{\"phase\":\"confirm\"}}"));
      }
      this->ship_listener_.request_close();
    } else {
      // HS_hello/init/prot/pin/access etc. logged for the next iteration.
      ESP_LOGI(TAG, "SHIP control received (handshake TODO): %s", json.c_str());
    }
    return out;
  });
}

void EEBusComponent::start_ship_listener() {
  if (!this->cert_store_.is_loaded()) {
    ESP_LOGE(TAG, "Cannot start SHIP listener: certificate store is not loaded");
    return;
  }
  this->ship_listener_.begin(this->cert_store_, 4712);
}

bool EEBusComponent::advertise_mdns() {
  return eebus_mdns_start(this->device_sku_, this->brand_, this->model_,
                          this->device_category_, this->device_type_,
                          this->cert_store_.get_ski(), this->ship_id_,
                          this->ship_listener_.get_port());
}

void EEBusComponent::check_failsafe() {
  // When the EEBus link is down for longer than the configured failsafe, any
  // active dim/block must be released so the heat pump returns to normal
  // operation.
  if (this->lpc_.is_dimmed() && !this->ship_listener_.is_connected()) {
    uint32_t now = millis();
    uint32_t last_act = this->node_.get_last_activity_ms();
    if (last_act > 0 && (now - last_act > this->failsafe_duration_s_ * 1000UL)) {
      ESP_LOGW(TAG, "EEBUS failsafe duration elapsed without peer activity; releasing consumption limit");
      this->record_action("Failsafe triggered: limit released");
      this->lpc_.release_limit();
    }
  }
}

void EEBusComponent::check_connection_state() {
  bool is_conn = this->ship_listener_.is_connected();
  if (is_conn != this->last_connected_) {
    this->last_connected_ = is_conn;
    eebus_mdns_set_registered(is_conn);
    this->pending_connected_state_ = is_conn;
    this->pending_connected_update_ = true;
    if (is_conn) {
      this->record_action("Connected to EMS");
      this->last_measurement_notify_ms_ = 0;
      this->last_ohpcf_notify_ms_ = 0;
      this->last_use_case_notify_ms_ = 0;
    } else {
      this->record_action("Disconnected from EMS");
      this->node_.reset_subscriptions();
    }
  }

  this->update_ems_status();

  std::string new_status = "Disconnected";
  if (is_conn) {
    if (this->enable_ohpcf_ && this->ohpcf_.get_state() == OhpcfState::RUNNING) {
      new_status = "Boost active (OHPCF)";
    } else if (this->lpc_.is_dimmed()) {
      if (this->lpc_.get_limit_value_w() <= 0.0f) {
        new_status = "Dimmed / Blocked (0W)";
      } else {
        char buf[64];
        snprintf(buf, sizeof(buf), "Limited to %.0f W", this->lpc_.get_limit_value_w());
        new_status = buf;
      }
    } else {
      new_status = "Normal operation";
    }
  }

  if (new_status != this->current_status_) {
    this->current_status_ = new_status;
    this->pending_status_state_ = new_status;
    this->pending_status_update_ = true;
  }
}

void EEBusComponent::update_ems_status() {
  std::string ems_desc = "–";
  if (this->ship_listener_.is_connected()) {
    std::string name;
    std::string rdev = this->node_.remote_device();
    std::string ip = this->ship_listener_.get_remote_ip();

    std::string lower_rdev = rdev;
    for (auto &c : lower_rdev) c = std::tolower(c);
    std::string lower_peer = this->ship_listener_.get_peer_name();
    for (auto &c : lower_peer) c = std::tolower(c);

    if (lower_rdev.find("evcc") != std::string::npos || lower_peer.find("evcc") != std::string::npos) {
      name = "EVCC";
    } else if (!rdev.empty()) {
      std::string clean = rdev;
      if (clean.rfind("d:_n:", 0) == 0) clean = clean.substr(5);
      else if (clean.rfind("d:_i:", 0) == 0) clean = clean.substr(5);
      name = clean;
    } else if (!this->ship_listener_.get_peer_name().empty()) {
      name = this->ship_listener_.get_peer_name();
    } else {
      name = "EMS";
    }

    if (!ip.empty()) {
      ems_desc = name + " (" + ip + ")";
    } else {
      ems_desc = name;
    }
  }

  if (ems_desc != this->current_ems_) {
    this->current_ems_ = ems_desc;
    this->pending_ems_state_ = ems_desc;
    this->pending_ems_update_ = true;
  }
}

static EEBusComponent *global_eebus_component = nullptr;

EEBusComponent::EEBusComponent() {
  global_eebus_component = this;
  this->inbound_mutex_ = xSemaphoreCreateMutex();
}

EEBusComponent::~EEBusComponent() {
  if (global_eebus_component == this) {
    global_eebus_component = nullptr;
  }
  if (this->inbound_mutex_ != nullptr) {
    vSemaphoreDelete(static_cast<SemaphoreHandle_t>(this->inbound_mutex_));
    this->inbound_mutex_ = nullptr;
  }
}

void EEBusComponent::disconnect_ems() {
  if (this->ship_listener_.is_connected()) {
    ESP_LOGI(TAG, "Disconnect requested by user from UI");
    this->record_action("Disconnected by user");
    this->ship_listener_.send_ship_control("{\"connectionClose\":{\"phase\":\"announce\"}}");
    this->ship_listener_.request_close();
  }
}

void eebus_disconnect() {
  if (global_eebus_component != nullptr) {
    global_eebus_component->disconnect_ems();
  }
}

}  // namespace eebus
}  // namespace esphome
