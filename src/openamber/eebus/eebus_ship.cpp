/*
 * Open Amber - native EEBus PoC
 *
 * SHIP message framing (see header).
 */

#include "eebus_ship.h"
#include "esphome/core/log.h"

#include <cstring>

namespace esphome {
namespace openamber_eebus {

// Convert standard JSON into EEBUS JSON: wrap each object's fields into a
// single-element array of {"key":value}.  Implemented with a lightweight
// recursive transform that walks the JSON text.
// (Faithful to ship-go/ship/helper.go JsonIntoEEBUSJson.)

// Minimal recursive walker operating on the JSON string using a tiny parser.
namespace {

struct JsonCursor {
  const std::string &s;
  size_t i = 0;
  bool skip_ws() {
    while (i < s.size() && (s[i] == ' ' || s[i] == '\t' || s[i] == '\n' || s[i] == '\r'))
      i++;
    return i < s.size();
  }
};

// Emit an EEBUS encoded value (object->array, arrays flattened, primitives as-is).
// Appends to `out`.
void encode_value(JsonCursor &c, std::string &out);

void encode_string(JsonCursor &c, std::string &out) {
  out += '"';
  c.i++;  // skip opening quote
  while (c.i < c.s.size()) {
    char ch = c.s[c.i++];
    if (ch == '"')
      break;
    if (ch == '\\') {
      out += ch;
      if (c.i < c.s.size())
        out += c.s[c.i++];
      continue;
    }
    out += ch;
  }
  out += '"';  // closing quote
}

void encode_object(JsonCursor &c, std::string &out) {
  // output as array of {"key":value}
  out += '[';
  bool first = true;
  c.i++;  // skip '{'
  if (!c.skip_ws() || c.s[c.i] == '}') {
    // empty object -> empty array singleton (EEBUS form)
    if (c.i < c.s.size())
      c.i++;
    out += ']';
    return;
  }
  while (true) {
    if (!first)
      out += ',';
    out += '{';
    // key
    encode_string(c, out);
    // consume the input ':' between key and value, then emit our ':'.
    c.skip_ws();
    if (c.i < c.s.size() && c.s[c.i] == ':')
      c.i++;
    out += ':';
    // value
    if (!c.skip_ws())
      break;
    encode_value(c, out);
    out += '}';
    first = false;
    c.skip_ws();
    if (c.i >= c.s.size())
      break;
    if (c.s[c.i] == ',') {
      c.i++;
      continue;
    }
    if (c.s[c.i] == '}') {
      c.i++;
      break;
    }
  }
  out += ']';
}

void encode_array(JsonCursor &c, std::string &out) {
  out += '[';
  bool first = true;
  c.i++;  // skip '['
  while (true) {
    if (!c.skip_ws())
      break;
    if (c.s[c.i] == ']') {
      c.i++;
      break;
    }
    if (!first)
      out += ',';
    encode_value(c, out);
    first = false;
    c.skip_ws();
    if (c.i < c.s.size() && c.s[c.i] == ',')
      c.i++;
  }
  out += ']';
}

void encode_value(JsonCursor &c, std::string &out) {
  if (!c.skip_ws())
    return;
  char ch = c.s[c.i];
  if (ch == '{') {
    encode_object(c, out);
  } else if (ch == '[') {
    encode_array(c, out);
  } else if (ch == '"') {
    encode_string(c, out);
  } else {
    // number / literal
    size_t start = c.i;
    while (c.i < c.s.size() && c.s[c.i] != ',' && c.s[c.i] != '}' && c.s[c.i] != ']')
      c.i++;
    out.append(c.s, start, c.i - start);
  }
}

}  // namespace

std::string eebus_json_into(const std::string &json) {
  std::string out;
  JsonCursor c{json, 0};
  encode_value(c, out);
  // Trim root-level '[' ']'
  if (out.size() >= 2 && out.front() == '[' && out.back() == ']')
    return out.substr(1, out.size() - 2);
  return out;
}

std::string eebus_json_from(const std::string &json) {
  // Mirror of ship-go JsonFromEEBUSJson: object arrays -> objects.
  std::string s = json;
  std::string r;
  r.reserve(s.size());
  r = s;
  auto replace_all = [&r](const char *from, const char *to) {
    std::string needle = from;
    size_t at = 0;
    while ((at = r.find(needle, at)) != std::string::npos) {
      r.replace(at, needle.size(), to);
      at += std::string(to).size();
    }
  };
  replace_all("[{", "{");
  replace_all("},{", ",");
  replace_all("}]", "}");
  replace_all("[]", "{}");
  return r;
}

std::vector<uint8_t> ship_data_frame(const std::string &spine_datagram_json) {
  std::string body = "{\"data\":{\"header\":{\"protocolId\":\"ee1.0\"},\"payload\":" +
                     spine_datagram_json + "}}";
  std::string eb = eebus_json_into(body);
  std::vector<uint8_t> f;
  f.reserve(eb.size() + 1);
  f.push_back(SHIP_MSG_DATA);
  f.insert(f.end(), eb.begin(), eb.end());
  return f;
}

std::vector<uint8_t> ship_control_frame(const std::string &model_object_json) {
  std::string eb = eebus_json_into(model_object_json);
  std::vector<uint8_t> f;
  f.reserve(eb.size() + 1);
  f.push_back(SHIP_MSG_CONTROL);
  f.insert(f.end(), eb.begin(), eb.end());
  return f;
}

bool ship_decode_frame(const std::vector<uint8_t> &frame, uint8_t &msg_type, std::string &standard_json) {
  if (frame.empty())
    return false;
  msg_type = frame[0];
  std::string eebus(frame.begin() + 1, frame.end());
  standard_json = eebus_json_from(eebus);
  return true;
}

}  // namespace openamber_eebus
}  // namespace esphome
