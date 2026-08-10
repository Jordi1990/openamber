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
  char buf[64];
  if (!this->device_str.empty()) {
    out += "{\"device\":\"";
    out += this->device_str;
    out += "\",\"entity\":[";
    // SPINE uses an entity path array
    char ent[16];
    snprintf(ent, sizeof(ent), "%u", static_cast<unsigned>(this->entity));
    out += ent;
    out += "],\"feature\":";
    char feat[16];
    snprintf(feat, sizeof(feat), "%u", static_cast<unsigned>(this->feature));
    out += feat;
    out += "}";
    return;
  }
  snprintf(buf, sizeof(buf), "{\"device\":%u,\"entity\":[%u],\"feature\":%u}",
           static_cast<unsigned>(this->device), static_cast<unsigned>(this->entity),
           static_cast<unsigned>(this->feature));
  out += buf;
}

// ---- TinyJson ----

void TinyJson::comma_or_space() {
  if (this->in_value_) {
    this->out_ += ',';
    this->in_value_ = false;
  }
  this->need_comma_ = false;
}

void TinyJson::begin_object() {
  this->comma_or_space();
  this->out_ += '{';
  this->in_value_ = true;
  this->need_comma_ = false;
}

void TinyJson::end_object() {
  this->out_ += '}';
}

void TinyJson::begin_array() {
  this->comma_or_space();
  this->out_ += '[';
  this->in_value_ = true;
  this->need_comma_ = false;
}

void TinyJson::end_array() {
  this->out_ += ']';
}

void TinyJson::key(const char *k) {
  if (this->need_comma_)
    this->out_ += ',';
  this->out_ += '"';
  this->out_ += k;
  this->out_ += "\":";
  this->need_comma_ = true;
}

void TinyJson::value(const char *s) {
  this->out_ += '"';
  this->out_ += s;
  this->out_ += '"';
  this->need_comma_ = true;
}

void TinyJson::value(uint64_t n) {
  char buf[24];
  snprintf(buf, sizeof(buf), "%llu", static_cast<unsigned long long>(n));
  this->out_ += buf;
  this->need_comma_ = true;
}

void TinyJson::value(bool b, bool quoted) {
  if (quoted) {
    this->out_ += b ? "\"true\"" : "\"false\"";
  } else {
    this->out_ += b ? "true" : "false";
  }
  this->need_comma_ = true;
}

void TinyJson::raw(const char *s) {
  this->out_ += s;
  this->need_comma_ = true;
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
  j.value("1.8");
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
  // SPINE nests each command as an array element: "cmd":[[ {...}, {...} ]]
  j.begin_array();
  j.begin_array();
  for (const auto &cmd : this->cmds_) {
    j.begin_object();
    j.key(cmd.data_class.c_str());
    j.raw(cmd.json.c_str());
    j.end_object();
  }
  j.end_array();  // inner array of commands
  j.end_array();  // cmd
  j.end_object();  // payload
  j.end_object();  // datagram
  j.end_object();  // root
  return out;
}

bool SpineInboundCommand::find(const char *data_class, std::string &out) const {
  for (const auto &cmd : this->cmds) {
    if (cmd.data_class == data_class) {
      out = cmd.json;
      return true;
    }
  }
  return false;
}

// ---- Parse dispatch ----
// A pragmatic, dependency-free splitter: it locates the header fields and the
// "cmd":[{ "<class>": <body> }] entries by string scanning.  All positions are
// bounds-checked; no C++ exceptions are available on this platform, so any
// out-of-range access would abort the device.

// strtol is only safe when `from <= json.size()` (c_str() is null-terminated
// there); guard the call to avoid reading past the terminator.
static uint64_t safe_atoi(const std::string &s, size_t from) {
  if (from > s.size())
    return 0;
  return static_cast<uint64_t>(strtoll(s.c_str() + from, nullptr, 10));
}

static bool split_addr(const std::string &json, const char *key, SpineAddress &addr) {
  std::string needle = std::string("\"") + key + "\":";
  auto p = json.find(needle);
  if (p == std::string::npos)
    return false;
  // Only read fields within this address object (up to the closing brace).
  auto obj_begin = json.find('{', p);
  auto obj_end = json.find('}', obj_begin == std::string::npos ? p : obj_begin);
  auto bounds = (obj_end == std::string::npos) ? json.size() : obj_end;
  // Device may be a string id ("device":"d:_n:...") or numeric.
  auto dstr = json.find("\"device\":\"", p);
  if (dstr != std::string::npos && dstr < bounds) {
    auto v = dstr + 10;  // skip "device":"
    auto ve = json.find('"', v);
    if (ve != std::string::npos && ve < bounds && v < json.size() && ve <= json.size()) {
      addr.device_str = json.substr(v, ve - v);
    }
  }
  auto dev = json.find("\"device\":", p);
  if (dev != std::string::npos && dev < bounds) {
    auto v = dev + 9;
    if (v <= json.size())
      addr.device = static_cast<uint16_t>(safe_atoi(json, v));
  }
  auto ent = json.find("\"entity\":", p);
  if (ent != std::string::npos && ent < bounds) {
    // entity is an array [N]; parse first element
    auto opb = json.find('[', ent);
    if (opb != std::string::npos && opb < bounds) {
      auto v = opb + 1;
      if (v <= json.size())
        addr.entity = static_cast<uint16_t>(safe_atoi(json, v));
    }
  }
  auto feat = json.find("\"feature\":", p);
  if (feat != std::string::npos && feat < bounds) {
    auto v = feat + 10;
    if (v <= json.size())
      addr.feature = static_cast<uint16_t>(safe_atoi(json, v));
  }
  return true;
}

bool spine_parse_dispatch(const std::string &json, SpineInboundCommand &out) {
  out = SpineInboundCommand();
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

  split_addr(json, "addressSource", out.source);
  split_addr(json, "addressDestination", out.destination);

  // Extract each "cmd":[ {...} ] entry as <dataClass>:<rawbody>.
  auto arr = json.find("\"cmd\":");
  if (arr != std::string::npos) {
    auto i = json.find('[', arr);
    while (i != std::string::npos && i < json.size()) {
      auto ob = json.find('{', i);
      if (ob == std::string::npos || ob >= json.size())
        break;
      auto cb = json.find('}', ob);
      if (cb == std::string::npos || cb >= json.size())
        break;
      auto kq = json.find('"', ob);
      if (kq == std::string::npos || kq >= cb || kq + 2 > json.size())
        break;
      auto kq2 = json.find('"', kq + 1);
      if (kq2 == std::string::npos || kq2 >= cb)
        break;
      SpineCommand cmd;
      cmd.data_class = json.substr(kq + 1, kq2 - kq - 1);
      auto col = json.find(':', kq2);
      if (col != std::string::npos && col < cb && col + 1 < json.size()) {
        cmd.json = json.substr(col + 1, cb - col - 1);
      }
      out.cmds.push_back(cmd);
      // advance past this entry
      auto next = json.find(',', cb);
      if (next != std::string::npos) {
        i = next + 1;
      } else {
        auto nb = json.find('[', cb);
        if (nb == std::string::npos || nb >= json.size())
          break;
        i = nb + 1;
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
