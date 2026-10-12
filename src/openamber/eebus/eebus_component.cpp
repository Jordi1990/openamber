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
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

#ifndef USE_EEBUS_MOCK
#include "mdns.h"
#include "eebus_ship.h"
#include "eebus_websocket.h"
#include "esphome/components/network/util.h"
#include <algorithm>
#include <cctype>
#endif

namespace esphome {
namespace openamber_eebus {

static const char *const TAG = "eebus";

#ifndef USE_EEBUS_MOCK

void EEBusComponent::record_action(const std::string &action) {
  this->last_action_ = action;
  this->pending_action_state_ = action;
  this->pending_action_update_ = true;
  ESP_LOGI(TAG, "EEBUS Actie: %s", action.c_str());
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

  // Check if model is already known via Modbus
  if (this->read_model_) {
    std::string detected = this->read_model_();
    if (!detected.empty() && detected != "Onbekend model") {
      this->model_ = detected;
      this->device_sku_ = "OpenAmber-" + detected;
      this->model_detected_ = true;
      ESP_LOGI(TAG, "EEBUS model via Modbus: %s", this->model_.c_str());
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

  this->current_status_ = "Initialiseren...";
  if (this->status_sensor_ != nullptr) {
    this->status_sensor_->publish_state(this->current_status_);
  }
  if (this->connected_sensor_ != nullptr) {
    this->connected_sensor_->publish_state(false);
  }
  this->record_action("Opstarten");
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

  // Check / update model from Modbus
  if (this->read_model_) {
    std::string detected = this->read_model_();
    if (!detected.empty() && detected != "Onbekend model") {
      if (this->model_ != detected) {
        this->model_ = detected;
        this->device_sku_ = "OpenAmber-" + detected;
        this->node_.set_model(this->model_);
        ESP_LOGI(TAG, "EEBUS model geconfigureerd via Modbus: %s (sku=%s)",
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
    if (this->last_action_ == "Opstarten" || this->last_action_ == "Initialisatie") {
      this->record_action("Wachten op warmtepomp model via Modbus...");
    }
  } else if (this->last_action_ == "Wachten op warmtepomp model via Modbus..." || this->last_action_ == "Opstarten") {
    this->record_action("EEBUS listener gestart, wacht op EMS");
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

  // If connected, push measurement updates periodically (every 5s)
  if (this->ship_listener_.is_connected() && this->node_.has_remote_device()) {
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
    this->pending_limit_state_ = active ? limit_w : NAN;
    this->pending_limit_update_ = true;
  });

  this->node_.set_ship_sender([this](const std::string &spine_json) -> bool {
    return this->ship_listener_.send_ship_data(spine_json);
  });

  this->node_.set_ohpcf(&this->ohpcf_);
  this->node_.set_lpc(&this->lpc_);
  this->node_.set_mpc(&this->mpc_);
  this->node_.set_mdt(&this->mdt_);
  this->node_.set_device_id(this->ship_id_);
  this->node_.set_brand(this->brand_);
  this->node_.set_model(this->model_);
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
      std::string reply = this->node_.handle_inbound(json);
      ESP_LOGI(TAG, "SPINE handled -> reply size=%u", static_cast<unsigned>(reply.size()));
      if (!reply.empty()) {
        std::vector<uint8_t> payload = ship_data_frame(reply);
        ESP_LOGI(TAG, "SPINE reply framed (%u bytes)", static_cast<unsigned>(payload.size()));
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
  this->ship_listener_.begin(this->cert_store_, 4712);
}

bool EEBusComponent::advertise_mdns() {
  return eebus_mdns_start(this->device_sku_, this->brand_, this->model_,
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
    eebus_mdns_set_registered(is_conn);
    this->pending_connected_state_ = is_conn;
    this->pending_connected_update_ = true;
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
}

EEBusComponent::~EEBusComponent() {
  if (global_eebus_component == this) {
    global_eebus_component = nullptr;
  }
}

void EEBusComponent::disconnect_ems() {
  if (this->ship_listener_.is_connected()) {
    ESP_LOGI(TAG, "Disconnect requested by user from UI");
    this->record_action("Verbinding verbroken door gebruiker");
    this->ship_listener_.send_ship_control("{\"connectionClose\":{\"phase\":\"announce\"}}");
    this->ship_listener_.request_close();
  }
}

void eebus_disconnect() {
  if (global_eebus_component != nullptr) {
    global_eebus_component->disconnect_ems();
  }
}

void fake_eebus_boost(bool active) {}
void fake_eebus_dim(bool active) {}
void fake_eebus_limit(float limit_w) {}
void fake_eebus_connected(bool connected, const std::string &ems_name) {}

#else  // USE_EEBUS_MOCK

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
  ESP_LOGI(TAG, "EEBUS [MOCK] Actie: %s", action.c_str());
}

void EEBusComponent::start_eebus_subsystem() {
  ESP_LOGI(TAG, "EEBus [MOCK] subsystem gestart.");
  if (this->status_sensor_ != nullptr) {
    this->status_sensor_->publish_state("Niet verbonden");
  }
  if (this->connected_sensor_ != nullptr) {
    this->connected_sensor_->publish_state(false);
  }
  if (this->last_action_sensor_ != nullptr) {
    this->last_action_sensor_->publish_state("Initialisatie voltooid");
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
  ESP_LOGI(TAG, "EEBus [MOCK] subsystem gestopt.");
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
  this->is_boost_requested_ = active;
  if (this->apply_optional_) {
    bool ok = this->apply_optional_(active);
    ESP_LOGI(TAG, "EEBus [MOCK] apply_optional(%d) -> %d", active, ok);
  }
  if (active) {
    if (this->connected_sensor_ != nullptr && !this->connected_sensor_->state) {
      this->fake_set_connected(true, "EVCC (192.168.1.100)");
    }
    this->record_action("EVCC Boost gestart (OHPCF)");
    if (this->status_sensor_ != nullptr) {
      this->status_sensor_->publish_state("Boost actief");
    }
  } else {
    this->record_action("EVCC Boost beëindigd (OHPCF)");
    if (this->status_sensor_ != nullptr) {
      bool conn = this->connected_sensor_ != nullptr && this->connected_sensor_->state;
      this->status_sensor_->publish_state(conn ? "Normaal bedrijf" : "Niet verbonden");
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
  if (active) {
    if (this->connected_sensor_ != nullptr && !this->connected_sensor_->state) {
      this->fake_set_connected(true, "EVCC (192.168.1.100)");
    }
    this->record_action("EVCC Dimmen gestart (LPC)");
    if (this->status_sensor_ != nullptr) {
      this->status_sensor_->publish_state("Gedimd (§14a)");
    }
  } else {
    this->record_action("EVCC Dimmen beëindigd (LPC)");
    if (this->status_sensor_ != nullptr) {
      bool conn = this->connected_sensor_ != nullptr && this->connected_sensor_->state;
      this->status_sensor_->publish_state(conn ? "Normaal bedrijf" : "Niet verbonden");
    }
  }
}

void EEBusComponent::fake_set_power_limit(float limit_w) {
  if (!this->is_enabled()) {
    ESP_LOGW(TAG, "EEBus [MOCK] Cannot set power limit: EEBUS is disabled");
    return;
  }
  bool active = !std::isnan(limit_w) && limit_w >= 0.0f;
  if (this->apply_limit_) {
    this->apply_limit_(active, limit_w);
  }
  if (this->power_limit_sensor_ != nullptr) {
    this->power_limit_sensor_->publish_state(limit_w);
  }
  if (active) {
    if (this->connected_sensor_ != nullptr && !this->connected_sensor_->state) {
      this->fake_set_connected(true, "EVCC (192.168.1.100)");
    }
    this->record_action("Vermogenslimiet ingesteld");
  } else {
    this->record_action("Vermogenslimiet opgeheven");
  }
}

void EEBusComponent::fake_set_connected(bool connected, const std::string &ems_name) {
  if (connected) {
    if (this->connected_sensor_ != nullptr) {
      this->connected_sensor_->publish_state(true);
    }
    if (this->ems_sensor_ != nullptr) {
      this->ems_sensor_->publish_state(ems_name);
    }
    if (this->status_sensor_ != nullptr) {
      if (this->is_boost_requested_) {
        this->status_sensor_->publish_state("Boost actief");
      } else if (this->is_dim_requested_) {
        this->status_sensor_->publish_state("Gedimd (§14a)");
      } else {
        this->status_sensor_->publish_state("Normaal bedrijf");
      }
    }
    this->record_action("Verbonden met " + ems_name);
  } else {
    if (this->connected_sensor_ != nullptr) {
      this->connected_sensor_->publish_state(false);
    }
    if (this->ems_sensor_ != nullptr) {
      this->ems_sensor_->publish_state("–");
    }
    if (this->status_sensor_ != nullptr) {
      this->status_sensor_->publish_state("Niet verbonden");
    }
    if (this->power_limit_sensor_ != nullptr) {
      this->power_limit_sensor_->publish_state(NAN);
    }
    this->record_action("Verbinding verbroken");
    if (this->is_boost_requested_) {
      this->fake_request_boost(false);
    }
    if (this->is_dim_requested_) {
      this->fake_request_dim(false);
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

#endif  // USE_EEBUS_MOCK

}  // namespace openamber_eebus
}  // namespace esphome
