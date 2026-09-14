/*
 * Open Amber - native EEBus PoC
 *
 * Minimal RFC 6455 WebSocket server for the SHIP transport.
 *
 * Pure buffer/framing logic (no socket dependency) so it can be unit-tested
 * and reused by any byte-stream transport (TLS).  The server handshake
 * response and frame encode/decode are implemented per RFC 6455.
 *
 * SHIP sends binary frames; the first payload byte is the SHIP message type.
 */

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace esphome {
namespace openamber_eebus {

// Builds the 101 Switching Protocols response for a WS upgrade.
// Returns false if the client handshake is not a valid websocket upgrade.
bool eebus_websocket_server_response(const std::string &client_request, std::string &response_out);

// Server -> client: build a single unmasked data frame.
// msg_type is the SHIP message-type byte prepended inside the payload.
void eebus_websocket_encode_server_frame(uint8_t opcode, const std::vector<uint8_t> &payload,
                                         std::vector<uint8_t> &frame_out);

struct EebusWsFrame {
  uint8_t opcode{0};
  std::vector<uint8_t> payload;
};

// Holds an incremental client->server frame decode state (masked frames).
struct EebusWsDecoder {
  // State
  enum class Phase { HEADER, LENGTH, EXT_LENGTH, MASK, PAYLOAD, DONE } phase{Phase::HEADER};
  uint8_t opcode{0};
  uint64_t payload_len{0};
  uint8_t mask[4]{0, 0, 0, 0};
  uint32_t i{0};  // index within current phase
  std::vector<uint8_t> payload;
  bool fin{false};

  // Feed raw TCP bytes; appends completed frames (opcode + payload) to `out_frames`.
  // Returns false on protocol error.
  bool feed(const std::vector<uint8_t> &bytes, std::vector<EebusWsFrame> &out_frames);
  void reset();
};

}  // namespace openamber_eebus
}  // namespace esphome
