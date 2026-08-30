/*
 * Open Amber - native EEBus PoC
 *
 * SPINE core (minimal): JSON datagram model + feature-address handling +
 * command routing.  Encoded per spine-go's model (see
 * docs/eebus-native-poc.md §10).
 *
 * No external JSON library is used: a tiny self-contained builder/raster is
 * provided so the component compiles standalone and stays debuggable against
 * real evcc traffic.
 */

#pragma once

#include "esphome/core/component.h"
#include <cstdint>
#include <string>
#include <vector>

namespace esphome {
namespace openamber_eebus {

struct SpineAddress {
  uint16_t device{0};
  uint16_t entity{0};
  uint16_t feature{0};
  std::string device_str;  // SHIP device id (string form, e.g. "d:_n:...")

  bool empty() const { return device == 0 && entity == 0 && feature == 0 && device_str.empty(); }
  void append_json(std::string &out) const;
};

enum class CmdClassifier : uint8_t { CALL, RESULT, REPLY, WRITE, READ, NOTIFY, ERROR };

const char *cmd_classifier_str(CmdClassifier c);

// A single SPINE command (one element of "payload.cmd").
struct SpineCommand {
  std::string data_class;  // e.g. "nodeManagementBindingData", "measurementListData"
  std::string json;  // already-encoded body for this command (without the class key)
};

// Builds an outbound SPINE datagram JSON.
class SpineDatagram {
 public:
  SpineDatagram() = default;

  void set_source(const SpineAddress &a) { this->source_ = a; }
  void set_destination(const SpineAddress &a) { this->destination_ = a; }
  void set_classifier(CmdClassifier c) { this->classifier_ = c; }
  void set_msg_counter(uint32_t n) { this->msg_counter_ = n; }
  void set_msg_counter_reference(uint32_t n) { this->msg_counter_reference_ = n; }

  void add_command(const SpineCommand &cmd) { this->cmds_.push_back(cmd); }

  std::string to_json() const;

 private:
  SpineAddress source_;
  SpineAddress destination_;
  CmdClassifier classifier_{CmdClassifier::WRITE};
  uint32_t msg_counter_{1};
  uint32_t msg_counter_reference_{0};
  std::vector<SpineCommand> cmds_;
};

// Minimal JSON writer (object/array + string/number helpers).
class TinyJson {
 public:
  explicit TinyJson(std::string &out) : out_(out) {}
  void begin_object();
  void end_object();
  void begin_array();
  void end_array();
  void key(const char *k);
  void value(const char *s);
  void value(uint64_t n);
  void value(bool b, bool quoted = false);
  void raw(const char *s);  // append raw JSON text

 private:
  std::string &out_;
  bool need_comma_{false};
  bool in_value_{false};
};

// Inbound command extracted from "datagram.payload.cmd[]".
struct SpineInboundCommand {
  SpineAddress source;
  SpineAddress destination;
  CmdClassifier classifier{CmdClassifier::WRITE};
  uint32_t msg_counter{0};
  uint32_t msg_counter_reference{0};
  bool valid{false};
  std::vector<SpineCommand> cmds;  // raw <dataClass,json> pairs

  // Look up a raw JSON string for a data class.
  bool find(const char *data_class, std::string &out) const;
};

// Splits a received datagram JSON into its header + payload commands.
// Returns false when the frame is not a SPINE datagram we recognise.
bool spine_parse_dispatch(const std::string &json, SpineInboundCommand &out);

}  // namespace openamber_eebus
}  // namespace esphome
