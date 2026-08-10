/*
 * Open Amber - native EEBus PoC
 *
 * SHIP message framing over the WebSocket transport (see
 * docs/eebus-native-poc.md §10 and ship-go/ship/connection_serialization.go):
 *
 *   WS binary frame payload = [msgType:1 byte][EEBUS-JSON payload]
 *   msgType: 1 = control (handshake/models), 2 = data (SPINE datagram)
 *   Data envelope: {"data":{"header":{"protocolId":"ee1.0"},"payload":<spine>}}
 *
 * EEBUS-JSON: every object is encoded as an array of {"key":value} elements.
 */

#pragma once

#include "esphome/core/component.h"
#include "eebus_websocket.h"
#include <functional>
#include <string>
#include <vector>

namespace esphome {
namespace openamber_eebus {

enum ShipMsgType : uint8_t {
  SHIP_MSG_CONTROL = 1,
  SHIP_MSG_DATA = 2,
};

// Standard JSON <-> EEBUS JSON conversions (mirror of ship-go helper.go).
std::string eebus_json_into(const std::string &json);  // standard -> eebus
std::string eebus_json_from(const std::string &json);  // eebus -> standard

// Build the SPINE data frame payload: [SHIP_MSG_DATA] + eebus-json(spine datagram).
std::vector<uint8_t> ship_data_frame(const std::string &spine_datagram_json);

// Build a control frame payload: [SHIP_MSG_CONTROL] + eebus-json(model object).
std::vector<uint8_t> ship_control_frame(const std::string &model_object_json);

// Decode a frame payload: strips the msg-type byte and returns the EEBUS-JSON
// body (standard JSON) plus the message type.
bool ship_decode_frame(const std::vector<uint8_t> &frame, uint8_t &msg_type, std::string &standard_json);

}  // namespace openamber_eebus
}  // namespace esphome
