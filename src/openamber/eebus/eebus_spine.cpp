/*
 * Open Amber - native EEBus PoC
 *
 * SPINE core implementation (minimal JSON datagram model + routing).  See
 * header for details.
 */

#include "eebus_spine.h"
#include "esphome/core/log.h"

#include <cstdio>
#include <cstring>

namespace esphome {
namespace openamber_eebus {

static const char *const TAG = "eebus_spine";

const char *cmd_classifier_str(CmdClassifier c) {
  switch (c) {
    case CmdClassifier::CALL: return "call";
    case CmdClassifier::RESULT: return "result";
    case CmdClassifier::REPLY: return "reply";
    case CmdClassifier::WRITE: return "write";
    case CmdClassifier::READ: return "read";
    case CmdClassifier::NOTIFY: return "notify";
    case CmdClassifier::ERROR: return "error";
    default: return "write";
  }
}

void SpineAddress::append_json(std::string &out) const {
  out += "{";
  if (!this->device_str.empty()) {
    out += "\"device\":\"";
    out += this->device_str;
    out += "\",";
  }
  out += "\"entity\":[";
  char ent[16];
  snprintf(ent, sizeof(ent), "%u", static_cast<unsigned>(this->entity));
  out += ent;
  out += "],\"feature\":";
  char feat[16];
  snprintf(feat, sizeof(feat), "%u", static_cast<unsigned>(this->feature));
  out += feat;
  out += "}";
}

// ---- TinyJson ----

void TinyJson::begin_object() {
  if (this->in_value_)
    this->out_ += ',';
  this->out_ += '{';
  this->in_value_ = false;
  this->need_comma_ = false;
}

void TinyJson::end_object() {
  this->out_ += '}';
  this->in_value_ = true;
  this->need_comma_ = true;
}

void TinyJson::begin_array() {
  if (this->in_value_)
    this->out_ += ',';
  this->out_ += '[';
  this->in_value_ = false;
  this->need_comma_ = false;
}

void TinyJson::end_array() {
  this->out_ += ']';
  this->in_value_ = true;
  this->need_comma_ = true;
}

void TinyJson::key(const char *k) {
  if (this->need_comma_)
    this->out_ += ',';
  this->out_ += '"';
  this->out_ += k;
  this->out_ += "\":";
  this->need_comma_ = true;
  this->in_value_ = false;
}

void TinyJson::value(const char *s) {
  this->out_ += '"';
  this->out_ += s;
  this->out_ += '"';
  this->need_comma_ = true;
  this->in_value_ = true;
}

void TinyJson::value(uint64_t n) {
  char buf[24];
  snprintf(buf, sizeof(buf), "%llu", static_cast<unsigned long long>(n));
  this->out_ += buf;
  this->need_comma_ = true;
  this->in_value_ = true;
}

void TinyJson::value(bool b, bool quoted) {
  if (quoted) {
    this->out_ += b ? "\"true\"" : "\"false\"";
  } else {
    this->out_ += b ? "true" : "false";
  }
  this->need_comma_ = true;
  this->in_value_ = true;
}

void TinyJson::raw(const char *s) {
  this->out_ += s;
  this->need_comma_ = true;
  this->in_value_ = true;
}

// ---- SpineDatagram ----

std::string SpineDatagram::to_json() const {
  std::string out;
  TinyJson j(out);
  j.begin_object();
  j.key("datagram");
  j.begin_object();
  j.key("header");
  j.begin_object();
  j.key("specificationVersion");
  j.value("1.3.0");
  j.key("addressSource");
  this->source_.append_json(out);
  j.key("addressDestination");
  this->destination_.append_json(out);
  j.key("msgCounter");
  j.value(static_cast<uint64_t>(this->msg_counter_));
  if (this->msg_counter_reference_ != 0) {
    j.key("msgCounterReference");
    j.value(static_cast<uint64_t>(this->msg_counter_reference_));
  }
  j.key("cmdClassifier");
  j.value(cmd_classifier_str(this->classifier_));
  j.end_object();  // header
  j.key("payload");
  j.begin_object();
  j.key("cmd");
  // Standard SPINE: "cmd":[{cmd1},{cmd2}]  (single array)
  // eebus_json_into will transform each {cmd} object into [{cmd}],
  // producing the EEBUS wire format: "cmd":[[{cmd1}],[{cmd2}]]
  j.begin_array();
  for (const auto &cmd : this->cmds_) {
    j.begin_object();
    j.key(cmd.data_class.c_str());
    j.raw(cmd.json.c_str());
    j.end_object();
  }
  j.end_array();  // cmd
  j.end_object();  // payload
  j.end_object();  // datagram
  j.end_object();  // root
  return out;
}

static size_t find_matching_brace(const std::string &s, size_t start) {
  if (start >= s.size() || s[start] != '{') return std::string::npos;
  int depth = 0;
  bool in_str = false;
  bool esc = false;
  for (size_t i = start; i < s.size(); ++i) {
    char c = s[i];
    if (esc) { esc = false; continue; }
    if (c == '\\') { if (in_str) esc = true; continue; }
    if (c == '"') { in_str = !in_str; continue; }
    if (!in_str) {
      if (c == '{') depth++;
      else if (c == '}') {
        depth--;
        if (depth == 0) return i;
      }
    }
  }
  return std::string::npos;
}

static size_t find_matching_bracket(const std::string &s, size_t start) {
  if (start >= s.size() || s[start] != '[') return std::string::npos;
  int depth = 0;
  bool in_str = false;
  bool esc = false;
  for (size_t i = start; i < s.size(); ++i) {
    char c = s[i];
    if (esc) { esc = false; continue; }
    if (c == '\\') { if (in_str) esc = true; continue; }
    if (c == '"') { in_str = !in_str; continue; }
    if (!in_str) {
      if (c == '[') depth++;
      else if (c == ']') {
        depth--;
        if (depth == 0) return i;
      }
    }
  }
  return std::string::npos;
}

bool SpineInboundCommand::find(const char *data_class, std::string &out) const {
  for (const auto &cmd : this->cmds) {
    if (cmd.data_class == data_class) {
      out = cmd.json;
      return true;
    }
  }
  // Fallback: search raw_json for key "\"data_class\":"
  std::string needle = std::string("\"") + data_class + "\"";
  size_t p = 0;
  while ((p = this->raw_json.find(needle, p)) != std::string::npos) {
    size_t after = p + needle.size();
    size_t col = this->raw_json.find_first_not_of(" \t\r\n", after);
    if (col != std::string::npos && this->raw_json[col] == ':') {
      auto val_start = this->raw_json.find_first_not_of(" \t\r\n", col + 1);
      if (val_start != std::string::npos) {
        char ch = this->raw_json[val_start];
        if (ch == '{') {
          auto end = find_matching_brace(this->raw_json, val_start);
          if (end != std::string::npos) {
            out = this->raw_json.substr(val_start, end - val_start + 1);
            return true;
          }
        } else if (ch == '[') {
          auto end = find_matching_bracket(this->raw_json, val_start);
          if (end != std::string::npos) {
            out = this->raw_json.substr(val_start, end - val_start + 1);
            return true;
          }
        } else {
          auto end = this->raw_json.find_first_of(",}]", val_start);
          if (end != std::string::npos) {
            out = this->raw_json.substr(val_start, end - val_start);
            return true;
          }
        }
      }
    }
    p = after;
  }
  return false;
}

// ---- Parse dispatch ----
// strtol is only safe when `from <= json.size()` (c_str() is null-terminated
// there); guard the call to avoid reading past the terminator.
static uint64_t safe_atoi(const std::string &s, size_t from) {
  if (from > s.size())
    return 0;
  return static_cast<uint64_t>(strtoll(s.c_str() + from, nullptr, 10));
}

bool spine_split_addr(const std::string &json, const char *key, SpineAddress &addr) {
  std::string needle = std::string("\"") + key + "\":";
  auto p = json.find(needle);
  if (p == std::string::npos)
    return false;
  // In EEBus JSON wire format, an address can be serialized either as an array of
  // single-key objects: [{"device":"..."},{"entity":[1]},{"feature":1}]
  // or as a single object: {"device":"...","entity":[1],"feature":1}.
  // Find the opening '[' or '{' and set bounds to the matching closing bracket/brace.
  size_t start_search = p + needle.size();
  auto first_struct = json.find_first_of("[{", start_search);
  size_t bounds = json.size();
  if (first_struct != std::string::npos) {
    if (json[first_struct] == '[') {
      auto arr_end = find_matching_bracket(json, first_struct);
      if (arr_end != std::string::npos)
        bounds = arr_end;
    } else if (json[first_struct] == '{') {
      auto obj_end = find_matching_brace(json, first_struct);
      if (obj_end != std::string::npos)
        bounds = obj_end;
    }
  }

  // Device may be a string id ("device":"d:_n:...") or numeric.
  auto dstr = json.find("\"device\":\"", p);
  if (dstr != std::string::npos && dstr < bounds) {
    auto v = dstr + 10;  // skip "device":"
    auto ve = json.find('"', v);
    if (ve != std::string::npos && ve < bounds && v < json.size() && ve <= json.size()) {
      addr.device_str = json.substr(v, ve - v);
    }
  } else {
    auto dev = json.find("\"device\":", p);
    if (dev != std::string::npos && dev < bounds) {
      auto v = dev + 9;
      if (v <= json.size())
        addr.device = static_cast<uint16_t>(safe_atoi(json, v));
    }
  }

  auto ent = json.find("\"entity\":", p);
  if (ent != std::string::npos && ent < bounds) {
    // entity is typically an array [N]; parse first element
    auto opb = json.find('[', ent);
    if (opb != std::string::npos && opb < bounds) {
      auto v = json.find_first_not_of(" \t\r\n", opb + 1);
      if (v != std::string::npos && v < bounds)
        addr.entity = static_cast<uint16_t>(safe_atoi(json, v));
    } else {
      auto v = json.find_first_not_of(" \t\r\n", ent + 9);
      if (v != std::string::npos && v < bounds)
        addr.entity = static_cast<uint16_t>(safe_atoi(json, v));
    }
  }

  auto feat = json.find("\"feature\":", p);
  if (feat != std::string::npos && feat < bounds) {
    auto v = json.find_first_not_of(" \t\r\n", feat + 10);
    if (v != std::string::npos && v < bounds)
      addr.feature = static_cast<uint16_t>(safe_atoi(json, v));
  }
  return true;
}

bool spine_parse_dispatch(const std::string &json, SpineInboundCommand &out) {
  out = SpineInboundCommand();
  out.raw_json = json;
  if (json.find("\"datagram\"") == std::string::npos)
    return false;

  auto cc = json.find("\"cmdClassifier\":");
  if (cc != std::string::npos && cc + 17 <= json.size()) {
    auto vs = json.find('"', cc + 16);
    auto ve = (vs == std::string::npos) ? std::string::npos : json.find('"', vs + 1);
    if (vs != std::string::npos && ve != std::string::npos && vs + 1 <= ve) {
      std::string v = json.substr(vs + 1, ve - vs - 1);
      if (v == "call") out.classifier = CmdClassifier::CALL;
      else if (v == "result") out.classifier = CmdClassifier::RESULT;
      else if (v == "reply") out.classifier = CmdClassifier::REPLY;
      else if (v == "read") out.classifier = CmdClassifier::READ;
      else if (v == "notify") out.classifier = CmdClassifier::NOTIFY;
      else if (v == "error") out.classifier = CmdClassifier::ERROR;
      else out.classifier = CmdClassifier::WRITE;
    }
  }

  auto mc = json.find("\"msgCounter\":");
  if (mc != std::string::npos)
    out.msg_counter = static_cast<uint32_t>(safe_atoi(json, mc + 13));
  auto mr = json.find("\"msgCounterReference\":");
  if (mr != std::string::npos)
    out.msg_counter_reference = static_cast<uint32_t>(safe_atoi(json, mr + 21));

  spine_split_addr(json, "addressSource", out.source);
  spine_split_addr(json, "addressDestination", out.destination);

  // Extract each "cmd":[ {...} ] entry as <dataClass>:<rawbody>.
  auto arr = json.find("\"cmd\":");
  if (arr != std::string::npos) {
    auto i = json.find('[', arr);
    if (i != std::string::npos) {
      size_t pos = i + 1;
      while (pos < json.size()) {
        auto ob = json.find('{', pos);
        if (ob == std::string::npos) break;
        auto cb = find_matching_brace(json, ob);
        if (cb == std::string::npos) break;

        // Inside this command object { ... }, extract the key
        auto kq = json.find('"', ob);
        if (kq != std::string::npos && kq < cb) {
          auto kq2 = json.find('"', kq + 1);
          if (kq2 != std::string::npos && kq2 < cb) {
            SpineCommand cmd;
            cmd.data_class = json.substr(kq + 1, kq2 - kq - 1);
            auto col = json.find(':', kq2);
            if (col != std::string::npos && col < cb) {
              size_t body_start = json.find_first_not_of(" \t\r\n", col + 1);
              if (body_start != std::string::npos && body_start < cb) {
                char ch = json[body_start];
                if (ch == '{') {
                  size_t body_end = find_matching_brace(json, body_start);
                  if (body_end != std::string::npos && body_end <= cb) {
                    cmd.json = json.substr(body_start, body_end - body_start + 1);
                  } else {
                    cmd.json = json.substr(body_start, cb - body_start);
                  }
                } else if (ch == '[') {
                  size_t body_end = find_matching_bracket(json, body_start);
                  if (body_end != std::string::npos && body_end <= cb) {
                    cmd.json = json.substr(body_start, body_end - body_start + 1);
                  } else {
                    cmd.json = json.substr(body_start, cb - body_start);
                  }
                } else {
                  cmd.json = json.substr(body_start, cb - body_start);
                }
              }
            }
            out.cmds.push_back(cmd);
          }
        }
        pos = cb + 1;
      }
    }
  }

  out.valid = !out.cmds.empty() || cc != std::string::npos;
  if (!out.valid)
    return false;
  ESP_LOGD(TAG, "Parsed datagram classifier=%s src=%u/%u/%u dst=%u/%u/%u cmds=%u",
           cmd_classifier_str(out.classifier), static_cast<unsigned>(out.source.device),
           static_cast<unsigned>(out.source.entity), static_cast<unsigned>(out.source.feature),
           static_cast<unsigned>(out.destination.device), static_cast<unsigned>(out.destination.entity),
           static_cast<unsigned>(out.destination.feature),
           static_cast<unsigned>(out.cmds.size()));
  return true;
}

}  // namespace openamber_eebus
}  // namespace esphome
