/*
 * Open Amber - native EEBus PoC
 *
 * Minimal RFC 6455 WebSocket server (see header).
 */

#include "eebus_websocket.h"
#include "esphome/core/log.h"

#include <cstdio>
#include <cstring>

// SHA-1, required for the Sec-WebSocket-Accept handshake.  ESP-IDF provides
// mbedtls SHA-1; include it directly (the header is available via mbedtls).
#include "mbedtls/sha1.h"

namespace esphome {
namespace openamber_eebus {

static const char *const TAG = "eebus_ws";
static const char kGUID[] = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

bool eebus_websocket_server_response(const std::string &req, std::string &response_out) {
  // Locate Sec-WebSocket-Key header value.
  auto key_pos = req.find("Sec-WebSocket-Key:");
  if (key_pos == std::string::npos)
    return false;
  auto a = key_pos + 18;
  while (a < req.size() && (req[a] == ' '))
    a++;
  auto b = req.find("\r\n", a);
  if (b == std::string::npos)
    b = req.size();
  std::string key = req.substr(a, b - a);

  std::string input = key + kGUID;

  uint8_t digest[20];
  int ret = mbedtls_sha1(reinterpret_cast<const unsigned char *>(input.data()), input.size(), digest);
  if (ret != 0)
    return false;

  static const char b64[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string accept;
  for (int c = 0; c < 20; c += 3) {
    uint32_t n = (uint32_t)digest[c] << 16;
    if (c + 1 < 20) n |= (uint32_t)digest[c + 1] << 8;
    if (c + 2 < 20) n |= (uint32_t)digest[c + 2];
    accept += b64[(n >> 18) & 63];
    accept += b64[(n >> 12) & 63];
    accept += c + 1 < 20 ? b64[(n >> 6) & 63] : '=';
    accept += c + 2 < 20 ? b64[n & 63] : '=';
  }

  response_out = "HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\n"
                 "Connection: Upgrade\r\nSec-WebSocket-Accept: " + accept + "\r\n\r\n";
  return true;
}

void eebus_websocket_encode_server_frame(uint8_t opcode, const std::vector<uint8_t> &payload,
                                         std::vector<uint8_t> &frame_out) {
  frame_out.push_back(0x80 | opcode);  // FIN + opcode
  size_t len = payload.size();
  if (len < 126) {
    frame_out.push_back(static_cast<uint8_t>(len));
  } else if (len < 65536) {
    frame_out.push_back(126);
    frame_out.push_back(static_cast<uint8_t>((len >> 8) & 0xff));
    frame_out.push_back(static_cast<uint8_t>(len & 0xff));
  } else {
    frame_out.push_back(127);
    for (int i = 7; i >= 0; i--)
      frame_out.push_back(static_cast<uint8_t>((len >> (8 * i)) & 0xff));
  }
  frame_out.insert(frame_out.end(), payload.begin(), payload.end());  // server frames are unmasked
}

void EebusWsDecoder::reset() {
  *this = EebusWsDecoder();
}

bool EebusWsDecoder::feed(const std::vector<uint8_t> &bytes,
                          std::vector<EebusWsFrame> &out_frames) {
  size_t pos = 0;
  while (pos < bytes.size()) {
    uint8_t byte = bytes[pos++];
    switch (this->phase) {
      case Phase::HEADER: {
        if (this->i == 0) {
          this->fin = (byte & 0x80) != 0;
          this->opcode = byte & 0x0f;
          if (this->opcode == 0x8) {  // close
            out_frames.push_back({0x08, {}});
            this->phase = Phase::DONE;
            return true;
          }
          this->i++;
        } else {
          bool masked = (byte & 0x80) != 0;
          if (!masked) {
            // client->server frames MUST be masked
            ESP_LOGW(TAG, "Received unmasked client frame");
          }
          uint8_t len7 = byte & 0x7f;
          this->payload_len = len7;
          this->payload.clear();
          if (len7 == 126) {
            // 16-bit extended length; payload_len recomputed below.
            this->payload_len = 0;
            this->phase = Phase::LENGTH;
            this->i = 0;
          } else if (len7 == 127) {
            // 64-bit extended length.
            this->payload_len = 0;
            this->phase = Phase::EXT_LENGTH;
            this->i = 0;
          } else {
            this->phase = masked ? Phase::MASK : Phase::PAYLOAD;
            this->i = 0;
            if (this->payload_len == 0 && !masked) {
              out_frames.push_back({this->opcode, this->payload});
              this->phase = Phase::HEADER;
            }
          }
        }
        break;
      }
      case Phase::LENGTH: {
        this->payload_len = (this->payload_len << 8) | byte;
        if (++this->i >= 2) {
          this->phase = Phase::MASK;
          this->i = 0;
        }
        break;
      }
      case Phase::EXT_LENGTH: {
        this->payload_len = (this->payload_len << 8) | byte;
        if (++this->i >= 8) {
          this->phase = Phase::MASK;
          this->i = 0;
        }
        break;
      }
      case Phase::MASK: {
        this->mask[this->i++] = byte;
        if (this->i >= 4) {
          if (this->payload_len == 0) {
            out_frames.push_back({this->opcode, this->payload});
            this->phase = Phase::HEADER;
            this->i = 0;
          } else {
            this->phase = Phase::PAYLOAD;
            this->i = 0;
          }
        }
        break;
      }
      case Phase::PAYLOAD: {
        // apply mask
        this->payload.push_back(byte ^ this->mask[(this->i++) & 3]);
        if (this->i >= this->payload_len) {
          out_frames.push_back({this->opcode, this->payload});
          this->phase = Phase::HEADER;
          this->i = 0;
        }
        break;
      }
      default:
        return false;
    }
  }
  return true;
}

}  // namespace openamber_eebus
}  // namespace esphome
