/*
 * Open Amber - native EEBus device PoC
 *
 * Copyright (C) 2025 Jordi Epema
 *
 * See docs/eebus-native-poc.md for the protocol contract and implementation
 * plan.  The SHIP/SPINE transport wires into ohpcf_/lpc_/mpc_/mdt_ servers;
 * the OpenAmber bridge callbacks (set_*) route into the real controllers with
 * safety priority.
 */

#include "eebus_component.h"
#include "mdns.h"
#include "eebus_ship.h"
#include "eebus_websocket.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <algorithm>
#include <cctype>

namespace esphome {
namespace openamber_eebus {

static const char *const TAG = "eebus";

void EEBusComponent::record_action(const std::string &action) {
  this->last_action_ = action;
  ESP_LOGI(TAG, "EEBUS Actie: %s", action.c_str());
  if (this->last_action_sensor_ != nullptr) {
    this->last_action_sensor_->publish_state(action);
  }
}

void EEBusComponent::setup() {
  if (!this->is_enabled()) {
    ESP_LOGI(TAG, "EEBus component is uitgeschakeld; initialisatie overgeslagen.");
    if (this->status_sensor_ != nullptr) {
      this->status_sensor_->publish_state("Uitgeschakeld");
    }
    if (this->connected_sensor_ != nullptr) {
      this->connected_sensor_->publish_state(false);
    }
    if (this->last_action_sensor_ != nullptr) {
      this->last_action_sensor_->publish_state("Uitgeschakeld");
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
  ESP_LOGI(TAG, "EEBus component initializing (sku=%s brand=%s model=%s)",
           this->device_sku_.c_str(), this->brand_.c_str(), this->model_.c_str());
  this->init_certificate_and_keys();
  this->wire_use_cases();
  this->initialized_ = true;

  if (this->status_sensor_ != nullptr) {
    this->status_sensor_->publish_state("Initialiseren...");
  }
  if (this->connected_sensor_ != nullptr) {
    this->connected_sensor_->publish_state(false);
  }
  if (this->last_action_sensor_ != nullptr) {
    this->last_action_sensor_->publish_state("Opstarten");
  }
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
  ESP_LOGI(TAG, "EEBus component stopping (uitgeschakeld)...");

  // Stop SHIP listener & mDNS
  this->ship_listener_.stop();
  eebus_mdns_stop();
  this->listener_started_ = false;

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
  this->current_status_ = "Uitgeschakeld";
  this->current_ems_ = "–";

  if (this->connected_sensor_ != nullptr) {
    this->connected_sensor_->publish_state(false);
  }
  if (this->status_sensor_ != nullptr) {
    this->status_sensor_->publish_state("Uitgeschakeld");
  }
  if (this->last_action_sensor_ != nullptr) {
    this->last_action_sensor_->publish_state("Uitgeschakeld");
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
    bool is_conn = this->ship_listener_.is_connected();
    if (is_conn != this->last_connected_) {
      this->check_connection_state();
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

  if (!this->listener_started_) {
    this->listener_started_ = true;
    this->start_ship_listener();
    this->advertise_mdns();
    if (this->last_action_ == "Opstarten") {
      this->record_action("EEBUS listener gestart, wacht op EMS");
    }
  }

  this->check_connection_state();

  // Refresh MPC (power) + MDT (temperature) from the OpenAmber bridge.
  float power_w = 0.0f;
  if (this->read_power_) {
    power_w = this->read_power_() + this->ohpcf_.get_current_optional_power_w();
    this->mpc_.set_power_w(power_w);
  }
  float temp_c = 0.0f;
  if (this->read_dhw_temp_) {
    temp_c = this->read_dhw_temp_();
    this->mdt_.set_temperature_c(temp_c);
  }

  // Refresh dynamic power estimate if configured.
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

  // If OHPCF is RUNNING, verify if OpenAmber's boost mode is still active
  if (this->ohpcf_.get_state() == OhpcfState::RUNNING) {
    uint32_t now = millis();
    // Allow a 60-second grace period after boost starts before checking if boost ended
    if (now - this->last_boost_start_ms_ >= 60000UL) {
      if (this->is_boost_active_ && !this->is_boost_active_()) {
        ESP_LOGI(TAG, "Boost mode ended in OpenAmber; updating EEBUS OHPCF state to AVAILABLE (inactive)");
        this->record_action("Boost beëindigd door OpenAmber");
        this->ohpcf_.abort();
      }
    }
  }

  // If connected and peer has subscribed, push measurement updates periodically
  if (this->ship_listener_.is_connected() && this->node_.is_subscribed_measurements()) {
    uint32_t now = millis();
    if (now - this->last_measurement_notify_ms_ >= 5000) {
      this->last_measurement_notify_ms_ = now;
      this->node_.notify_measurements(power_w, temp_c);
    }
  }

  // Push OHPCF state periodically (every 10s) when connected
  if (this->ship_listener_.is_connected() && this->node_.has_remote_device()) {
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
  if (!this->cert_store_.load_or_create())
    ESP_LOGE(TAG, "Failed to initialise EEBus certificate store");
  this->ship_id_ = this->cert_store_.get_ski();
}

void EEBusComponent::wire_use_cases() {
  float initial_power = 0.0f;
  if (this->read_power_estimate_) {
    initial_power = this->read_power_estimate_();
  }
  this->ohpcf_.announce(initial_power, 600, 600);
  this->last_power_estimate_w_ = initial_power;

  this->ohpcf_.set_applier([this](bool enabled) -> bool {
    // Route the CEM's optional-consumption request through the bridge; the
    // bridge decides whether the request is granted (safety first).
    if (this->apply_optional_) {
      bool ok = this->apply_optional_(enabled);
      if (!ok) {
        this->record_action("EVCC Boost geweigerd (OpenAmber blokkade/veiligheid)");
        return false;
      }
    }
    if (enabled) {
      this->last_boost_start_ms_ = millis();
      this->record_action("EVCC Boost gestart (OHPCF)");
    } else {
      this->record_action("EVCC Boost beëindigd (OHPCF)");
    }
    return true;
  });

  this->ohpcf_.set_change_callback([this](OhpcfState state) {
    this->node_.notify_ohpcf();
  });

  this->lpc_.set_dimmer([this](bool dim) {
    // Dim -> SG-Ready block path, again safety-gated by the bridge.
    if (this->apply_dim_) {
      this->apply_dim_(dim);
    }
    if (dim) {
      this->record_action("EVCC Dim/Blokkade actief (LPC 0W)");
    } else {
      this->record_action("EVCC Dim/Blokkade opgeheven");
    }
  });

  this->lpc_.set_limit_applier([this](bool active, float limit_w) {
    if (this->apply_limit_) {
      this->apply_limit_(active, limit_w);
    }
    if (active) {
      char buf[64];
      snprintf(buf, sizeof(buf), "EVCC Limiet actief: %.0f W", limit_w);
      this->record_action(buf);
    } else {
      this->record_action("EVCC Limiet opgeheven");
    }
  });

  this->lpc_.set_change_callback([this](bool active, float limit_w) {
    this->node_.notify_lpc(active, limit_w);
    if (this->power_limit_sensor_ != nullptr) {
      if (active) {
        this->power_limit_sensor_->publish_state(limit_w);
      } else {
        this->power_limit_sensor_->publish_state(NAN);
      }
    }
  });

  this->node_.set_ship_sender([this](const std::string &spine_json) -> bool {
    return this->ship_listener_.send_ship_data(spine_json);
  });

  this->node_.set_ohpcf(&this->ohpcf_);
  this->node_.set_lpc(&this->lpc_);
  this->node_.set_mpc(&this->mpc_);
  this->node_.set_mdt(&this->mdt_);
  this->node_.set_device_id(this->ship_id_);
  this->node_.set_on_remote_device_changed([this](const std::string &) {
    this->update_ems_status();
  });

  this->ship_listener_.set_frame_handler([this](const std::vector<uint8_t> &frame)
                                             -> std::vector<uint8_t> {
    std::vector<uint8_t> out;  // WS-encoded bytes to send back (empty = none)
    auto send_data = [&out](const std::vector<uint8_t> &payload) {
      eebus_websocket_encode_server_frame(0x2 /*binary*/, payload, out);
    };

    ESP_LOGV(TAG, "SHIP frame (%u bytes)", static_cast<unsigned>(frame.size()));

    uint8_t msg_type = 0;
    std::string json;
    if (!ship_decode_frame(frame, msg_type, json)) {
      ESP_LOGW(TAG, "Could not decode SHIP frame");
      return out;
    }
    ESP_LOGD(TAG, "SHIP msg_type=%u json=[%s]", static_cast<unsigned>(msg_type), json.c_str());

    if (msg_type == 0) {
      // SHIP CMI (connection management initiation): ShipInit = {0x00,0x00}.
      // As server we must reply with the same ShipInit, then the client sends
      // its HELLO/pairing phase.  (ship-go/model: MsgTypeInit=0.)
      send_data({0x00, 0x00});
      ESP_LOGI(TAG, "Replied to SHIP INIT (CMI) with ShipInit");
      return out;
    }

    if (msg_type == SHIP_MSG_DATA) {
      std::string reply = this->node_.handle_inbound(json);
      ESP_LOGD(TAG, "SPINE handled -> reply size=%u", static_cast<unsigned>(reply.size()));
      if (!reply.empty()) {
        std::vector<uint8_t> payload = ship_data_frame(reply);
        ESP_LOGD(TAG, "SPINE reply framed (%u bytes)", static_cast<unsigned>(payload.size()));
        send_data(payload);
      } else {
        ESP_LOGW(TAG, "SPINE DATA frame not handled (no reply)");
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
        // Note: the SHIP protocol format value is literally "JSON-UTF8"
        // (model.MessageProtocolFormatTypeUTF8).
        send_data(ship_control_frame(
            "{\"messageProtocolHandshake\":{\"version\":{\"major\":1,\"minor\":0},"
            "\"formats\":{\"format\":[\"JSON-UTF8\"]},\"handshakeType\":\"select\"}}"));
        ESP_LOGI(TAG, "Replied to messageProtocolHandshake (select)");
        return out;
      }
      // The peer's select echo confirms our choice; no reply needed.
      ESP_LOGI(TAG, "Received messageProtocolHandshake confirm (select echo)");
      return out;
    }

    if (json.find("connectionPinState") != std::string::npos) {
      // PIN check phase: announce our own (no PIN required) state.
      send_data(ship_control_frame("{\"connectionPinState\":{\"pinState\":\"none\"}}"));
      ESP_LOGI(TAG, "Replied to connectionPinState (none)");
      return out;
    }

    if (json.find("accessMethodsRequest") != std::string::npos) {      // Respond with our SHIP ID so evcc can identify the device.
      std::string model = "{\"accessMethods\":{\"id\":\"" + this->ship_id_ + "\"}}";
      send_data(ship_control_frame(model));
      ESP_LOGI(TAG, "Responded to accessMethodsRequest (shipId=%s)", this->ship_id_.c_str());
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
  this->ship_listener_.begin(this->cert_store_, 44328);
}

void EEBusComponent::advertise_mdns() {
  eebus_mdns_start(this->device_sku_, this->brand_, this->model_,
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
      this->record_action("Failsafe geactiveerd: limiet opgeheven");
      this->lpc_.release_limit();
    }
  }
}

void EEBusComponent::check_connection_state() {
  bool is_conn = this->ship_listener_.is_connected();
  if (is_conn != this->last_connected_) {
    this->last_connected_ = is_conn;
    if (this->connected_sensor_ != nullptr) {
      this->connected_sensor_->publish_state(is_conn);
    }
    if (is_conn) {
      this->record_action("Verbonden met EMS");
      this->last_measurement_notify_ms_ = 0;
      this->last_ohpcf_notify_ms_ = 0;
      this->last_use_case_notify_ms_ = 0;
    } else {
      this->record_action("Verbinding verbroken met EMS");
    }
  }

  this->update_ems_status();

  std::string new_status = "Niet verbonden";
  if (is_conn) {
    if (this->ohpcf_.get_state() == OhpcfState::RUNNING) {
      new_status = "Boost actief (OHPCF)";
    } else if (this->lpc_.is_dimmed()) {
      if (this->lpc_.get_limit_value_w() <= 0.0f) {
        new_status = "Gedimd / Geblokkeerd (0W)";
      } else {
        char buf[64];
        snprintf(buf, sizeof(buf), "Begrensd op %.0f W", this->lpc_.get_limit_value_w());
        new_status = buf;
      }
    } else {
      new_status = "Normaal bedrijf";
    }
  }

  if (new_status != this->current_status_) {
    this->current_status_ = new_status;
    if (this->status_sensor_ != nullptr) {
      this->status_sensor_->publish_state(new_status);
    }
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
    if (this->ems_sensor_ != nullptr) {
      this->ems_sensor_->publish_state(ems_desc);
    }
  }
}

}  // namespace openamber_eebus
}  // namespace esphome
