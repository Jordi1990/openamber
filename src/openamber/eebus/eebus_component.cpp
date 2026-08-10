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

namespace esphome {
namespace openamber_eebus {

static const char *const TAG = "eebus";

void EEBusComponent::setup() {
  ESP_LOGI(TAG, "EEBus component initializing (sku=%s brand=%s model=%s)",
           this->device_sku_.c_str(), this->brand_.c_str(), this->model_.c_str());
  this->init_certificate_and_keys();
  this->wire_use_cases();
  // TLS-listener/mDNS startup is deferred to the first update() so all
  // ESPHome/ESP-IDF subsystems are fully initialised.
}

void EEBusComponent::update() {
  if (!this->listener_started_) {
    this->listener_started_ = true;
    this->start_ship_listener();
    this->advertise_mdns();
  }

  // Refresh MPC (power) + MDT (temperature) from the OpenAmber bridge.
  if (this->read_power_) {
    this->mpc_.set_power_w(this->read_power_() + this->ohpcf_.get_current_optional_power_w());
  }
  if (this->read_dhw_temp_) {
    this->mdt_.set_temperature_c(this->read_dhw_temp_());
  }
  this->check_failsafe();
}

void EEBusComponent::init_certificate_and_keys() {
  if (!this->cert_store_.load_or_create())
    ESP_LOGE(TAG, "Failed to initialise EEBus certificate store");
  this->ship_id_ = this->cert_store_.get_ski();
}

void EEBusComponent::wire_use_cases() {
  this->ohpcf_.set_applier([this](bool enabled) {
    // Route the CEM's optional-consumption request through the bridge; the
    // bridge decides whether the request is granted (safety first).
    if (this->apply_optional_) {
      this->apply_optional_(enabled);
    }
  });

  this->lpc_.set_dimmer([this](bool dim) {
    // Dim -> SG-Ready block path, again safety-gated by the bridge.
    if (this->apply_dim_) {
      this->apply_dim_(dim);
    }
  });

  this->node_.set_ohpcf(&this->ohpcf_);
  this->node_.set_lpc(&this->lpc_);
  this->node_.set_device_id(this->ship_id_);

  this->ship_listener_.set_frame_handler([this](const std::vector<uint8_t> &frame)
                                             -> std::vector<uint8_t> {
    std::vector<uint8_t> out;  // WS-encoded bytes to send back (empty = none)
    auto send_data = [&out](const std::vector<uint8_t> &payload) {
      eebus_websocket_encode_server_frame(0x2 /*binary*/, payload, out);
    };

    // Hex dump the first bytes for diagnosis (SHIP framing is debugged live).
    {
      std::string hex;
      size_t n = frame.size() > 96 ? 96 : frame.size();
      for (size_t i = 0; i < n; i++) {
        char b[4];
        snprintf(b, sizeof(b), "%02x ", frame[i]);
        hex += b;
      }
      ESP_LOGI(TAG, "SHIP frame (%u bytes) hex: %s", static_cast<unsigned>(frame.size()),
               hex.c_str());
    }

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
      ESP_LOGI(TAG, "SPINE handled -> reply size=%u", static_cast<unsigned>(reply.size()));
      if (!reply.empty()) {
        std::vector<uint8_t> payload = ship_data_frame(reply);
        ESP_LOGI(TAG, "SPINE reply framed (%u bytes)", static_cast<unsigned>(payload.size()));
        send_data(payload);
        // Also announce our use cases as a separate NOTIFY; evcc's use-case
        // layer activates OHPCF/MPC/MDT/LPC on a nodeManagementUseCaseData event.
        std::string ucn = this->node_.build_use_case_data_notify(0, "");
        std::vector<uint8_t> ucp = ship_data_frame(ucn);
        ESP_LOGI(TAG, "Use-case notify framed (%u bytes)", static_cast<unsigned>(ucp.size()));
        send_data(ucp);
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
  // operation.  The SPINE connection state drives this in the next layer; the
  // hook is kept here for the failsafe_duration wiring.
}

}  // namespace openamber_eebus
}  // namespace esphome
