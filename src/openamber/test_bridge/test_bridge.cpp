#include "test_bridge.h"
#include <cerrno>

#ifndef _WIN32
extern "C" {
uint32_t __real__ZN7esphome6millisEv();
uint64_t __real__ZN7esphome9millis_64Ev();
uint32_t __real__ZN7esphome6microsEv();
}

static uint64_t s_virtual_time_offset_ms = 0;

extern "C" uint32_t __wrap__ZN7esphome6millisEv() {
  return __real__ZN7esphome6millisEv() + static_cast<uint32_t>(s_virtual_time_offset_ms);
}

extern "C" uint64_t __wrap__ZN7esphome9millis_64Ev() {
  return __real__ZN7esphome9millis_64Ev() + s_virtual_time_offset_ms;
}

extern "C" uint32_t __wrap__ZN7esphome6microsEv() {
  return __real__ZN7esphome6microsEv() + static_cast<uint32_t>(s_virtual_time_offset_ms * 1000ULL);
}
#else
static uint64_t s_virtual_time_offset_ms = 0;
#endif

namespace esphome {
namespace test_bridge {

static const char *const TAG = "test_bridge";

void TestBridge::setup() {
#ifdef _WIN32
  WSADATA wsa_data;
  WSAStartup(MAKEWORD(2, 2), &wsa_data);
#endif

  this->server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (this->server_fd_ < 0) {
    ESP_LOGE(TAG, "Failed to create socket: %d", errno);
    return;
  }

  int opt = 1;
#ifdef _WIN32
  setsockopt(this->server_fd_, SOL_SOCKET, SO_REUSEADDR, (const char *)&opt, sizeof(opt));
  u_long mode = 1;
  ioctlsocket(this->server_fd_, FIONBIO, &mode);
#else
  setsockopt(this->server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
  fcntl(this->server_fd_, F_SETFL, O_NONBLOCK);
#endif

  struct sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = inet_addr("127.0.0.1");
  address.sin_port = htons(this->port_);

  if (bind(this->server_fd_, (struct sockaddr *)&address, sizeof(address)) < 0) {
    ESP_LOGE(TAG, "Failed to bind to 127.0.0.1:%d: %d", this->port_, errno);
#ifdef _WIN32
    closesocket(this->server_fd_);
#else
    close(this->server_fd_);
#endif
    this->server_fd_ = -1;
    return;
  }

  if (listen(this->server_fd_, 1) < 0) {
    ESP_LOGE(TAG, "Failed to listen on socket");
    return;
  }

  ESP_LOGI(TAG, "Test automation bridge listening on 127.0.0.1:%d (%zu widgets, %zu sensors, %zu switches registered)", 
           this->port_, this->widgets_.size(), this->sensors_.size(), this->switches_.size());
}

void TestBridge::dump_config() {
  ESP_LOGCONFIG(TAG, "TestBridge:");
  ESP_LOGCONFIG(TAG, "  Port: %d", this->port_);
  ESP_LOGCONFIG(TAG, "  Registered widgets: %zu", this->widgets_.size());
  ESP_LOGCONFIG(TAG, "  Registered sensors: %zu", this->sensors_.size());
  ESP_LOGCONFIG(TAG, "  Registered numbers: %zu", this->numbers_.size());
  ESP_LOGCONFIG(TAG, "  Registered switches: %zu", this->switches_.size());
  ESP_LOGCONFIG(TAG, "  Registered binary sensors: %zu", this->binary_sensors_.size());
}

void TestBridge::close_client() {
  if (this->client_fd_ >= 0) {
#ifdef _WIN32
    closesocket(this->client_fd_);
#else
    close(this->client_fd_);
#endif
    this->client_fd_ = -1;
    this->rx_buffer_.clear();
    ESP_LOGD(TAG, "Client disconnected");
  }
}

void TestBridge::loop() {
  if (this->server_fd_ < 0) return;

  if (this->client_fd_ < 0) {
    struct sockaddr_in client_addr{};
    socklen_t client_len = sizeof(client_addr);
    this->client_fd_ = accept(this->server_fd_, (struct sockaddr *)&client_addr, &client_len);
    if (this->client_fd_ >= 0) {
#ifdef _WIN32
      u_long mode = 1;
      ioctlsocket(this->client_fd_, FIONBIO, &mode);
#else
      fcntl(this->client_fd_, F_SETFL, O_NONBLOCK);
#endif
      ESP_LOGI(TAG, "Test client connected");
    }
  }

  if (this->client_fd_ >= 0) {
    char buf[1024];
#ifdef _WIN32
    int n = recv(this->client_fd_, buf, sizeof(buf) - 1, 0);
#else
    ssize_t n = recv(this->client_fd_, buf, sizeof(buf) - 1, 0);
#endif
    if (n > 0) {
      this->rx_buffer_.append(buf, n);
      size_t pos;
      while ((pos = this->rx_buffer_.find('\n')) != std::string::npos) {
        std::string line = this->rx_buffer_.substr(0, pos);
        this->rx_buffer_.erase(0, pos + 1);
        if (!line.empty() && line.back() == '\r') {
          line.pop_back();
        }
        if (!line.empty()) {
          this->process_line(line);
        }
      }
    } else if (n == 0) {
      this->close_client();
    } else {
#ifdef _WIN32
      int err = WSAGetLastError();
      if (err != WSAEWOULDBLOCK) {
        this->close_client();
      }
#else
      if (errno != EAGAIN && errno != EWOULDBLOCK) {
        this->close_client();
      }
#endif
    }
  }
}

void TestBridge::send_response(const std::string &json_str) {
  if (this->client_fd_ >= 0) {
    std::string out = json_str + "\n";
#ifdef _WIN32
    send(this->client_fd_, out.c_str(), static_cast<int>(out.length()), 0);
#else
    send(this->client_fd_, out.c_str(), out.length(), 0);
#endif
  }
}

lv_obj_t *TestBridge::resolve_widget(const std::string &id) {
  auto it = this->widgets_.find(id);
  if (it != this->widgets_.end() && it->second != nullptr) {
    return *(it->second);
  }
  return nullptr;
}

void TestBridge::process_line(const std::string &line) {
  json::parse_json(line, [this](JsonObject root) {
    std::string cmd = root["cmd"] | "";

    if (cmd == "ping") {
      this->send_response("{\"status\":\"ok\"}");
      return true;
    }

    if (cmd == "list_widgets") {
      std::string resp = json::build_json([this](JsonObject out) {
        out["status"] = "ok";
        JsonArray arr = out["widgets"].to<JsonArray>();
        for (const auto &kv : this->widgets_) {
          arr.add(kv.first);
        }
      });
      this->send_response(resp);
      return true;
    }

    if (cmd == "click") {
      std::string id = root["id"] | "";
      lv_obj_t *obj = this->resolve_widget(id);
      if (obj == nullptr) {
        this->send_response("{\"status\":\"error\",\"message\":\"Widget not found: " + id + "\"}");
        return true;
      }
      lv_obj_send_event(obj, LV_EVENT_CLICKED, nullptr);
      this->send_response("{\"status\":\"ok\"}");
      return true;
    }

    if (cmd == "get_widget") {
      std::string id = root["id"] | "";
      lv_obj_t *obj = this->resolve_widget(id);
      if (obj == nullptr) {
        this->send_response("{\"status\":\"error\",\"message\":\"Widget not found: " + id + "\"}");
        return true;
      }
      bool hidden = lv_obj_has_flag(obj, LV_OBJ_FLAG_HIDDEN);
      bool disabled = lv_obj_has_state(obj, LV_STATE_DISABLED);
      bool checked = lv_obj_has_state(obj, LV_STATE_CHECKED);

      std::string text = "";
      if (lv_obj_check_type(obj, &lv_label_class)) {
        const char *t = lv_label_get_text(obj);
        if (t) text = t;
      } else {
        uint32_t cnt = lv_obj_get_child_count(obj);
        for (uint32_t i = 0; i < cnt; i++) {
          lv_obj_t *child = lv_obj_get_child(obj, i);
          if (child && lv_obj_check_type(child, &lv_label_class)) {
            const char *t = lv_label_get_text(child);
            if (t) { text = t; break; }
          }
        }
      }

      std::string resp = json::build_json([&](JsonObject out) {
        out["status"] = "ok";
        out["visible"] = !hidden;
        out["hidden"] = hidden;
        out["disabled"] = disabled;
        out["checked"] = checked;
        out["text"] = text;
      });
      this->send_response(resp);
      return true;
    }

    if (cmd == "set_sensor") {
      std::string id = root["id"] | "";
      float val = root["value"] | 0.0f;
      auto it = this->sensors_.find(id);
      if (it != this->sensors_.end() && it->second != nullptr) {
        it->second->publish_state(val);
        this->send_response("{\"status\":\"ok\"}");
        return true;
      }
      // Fallback: search App sensors
      for (auto *s : App.get_sensors()) {
        char buf[128];
        s->write_object_id_to(buf, sizeof(buf));
        if (id == buf || id == s->get_name().c_str()) {
          s->publish_state(val);
          this->send_response("{\"status\":\"ok\"}");
          return true;
        }
      }
      this->send_response("{\"status\":\"error\",\"message\":\"Sensor not found: " + id + "\"}");
      return true;
    }

    if (cmd == "set_number") {
      std::string id = root["id"] | "";
      float val = root["value"] | 0.0f;
      auto it = this->numbers_.find(id);
      if (it != this->numbers_.end() && it->second != nullptr) {
        it->second->publish_state(val);
        this->send_response("{\"status\":\"ok\"}");
        return true;
      }
      for (auto *n : App.get_numbers()) {
        char buf[128];
        n->write_object_id_to(buf, sizeof(buf));
        if (id == buf || id == n->get_name().c_str()) {
          n->publish_state(val);
          this->send_response("{\"status\":\"ok\"}");
          return true;
        }
      }
      this->send_response("{\"status\":\"error\",\"message\":\"Number not found: " + id + "\"}");
      return true;
    }

    if (cmd == "set_switch") {
      std::string id = root["id"] | "";
      bool val = root["value"] | false;
      auto it = this->switches_.find(id);
      if (it != this->switches_.end() && it->second != nullptr) {
        if (val) it->second->turn_on(); else it->second->turn_off();
        this->send_response("{\"status\":\"ok\"}");
        return true;
      }
      for (auto *sw : App.get_switches()) {
        char buf[128];
        sw->write_object_id_to(buf, sizeof(buf));
        if (id == buf || id == sw->get_name().c_str()) {
          if (val) sw->turn_on(); else sw->turn_off();
          this->send_response("{\"status\":\"ok\"}");
          return true;
        }
      }
      this->send_response("{\"status\":\"error\",\"message\":\"Switch not found: " + id + "\"}");
      return true;
    }

    if (cmd == "set_binary_sensor") {
      std::string id = root["id"] | "";
      bool val = root["value"] | false;
      auto it = this->binary_sensors_.find(id);
      if (it != this->binary_sensors_.end() && it->second != nullptr) {
        it->second->publish_state(val);
        this->send_response("{\"status\":\"ok\"}");
        return true;
      }
      for (auto *bs : App.get_binary_sensors()) {
        char buf[128];
        bs->write_object_id_to(buf, sizeof(buf));
        if (id == buf || id == bs->get_name().c_str()) {
          bs->publish_state(val);
          this->send_response("{\"status\":\"ok\"}");
          return true;
        }
      }
      this->send_response("{\"status\":\"error\",\"message\":\"Binary sensor not found: " + id + "\"}");
      return true;
    }

    if (cmd == "get_entity") {
      std::string id = root["id"] | "";
      // Check sensor map
      auto it_s = this->sensors_.find(id);
      if (it_s != this->sensors_.end() && it_s->second != nullptr) {
        std::string resp = json::build_json([&](JsonObject out) {
          out["status"] = "ok";
          out["type"] = "sensor";
          out["state"] = it_s->second->state;
        });
        this->send_response(resp);
        return true;
      }
      // Check switch map
      auto it_sw = this->switches_.find(id);
      if (it_sw != this->switches_.end() && it_sw->second != nullptr) {
        std::string resp = json::build_json([&](JsonObject out) {
          out["status"] = "ok";
          out["type"] = "switch";
          out["state"] = it_sw->second->state;
        });
        this->send_response(resp);
        return true;
      }
      // Check number map
      auto it_n = this->numbers_.find(id);
      if (it_n != this->numbers_.end() && it_n->second != nullptr) {
        std::string resp = json::build_json([&](JsonObject out) {
          out["status"] = "ok";
          out["type"] = "number";
          out["state"] = it_n->second->state;
        });
        this->send_response(resp);
        return true;
      }
      // Check binary sensor map
      auto it_bs = this->binary_sensors_.find(id);
      if (it_bs != this->binary_sensors_.end() && it_bs->second != nullptr) {
        std::string resp = json::build_json([&](JsonObject out) {
          out["status"] = "ok";
          out["type"] = "binary_sensor";
          out["state"] = it_bs->second->state;
        });
        this->send_response(resp);
        return true;
      }

      // App fallbacks
      for (auto *s : App.get_sensors()) {
        char buf[128];
        s->write_object_id_to(buf, sizeof(buf));
        if (id == buf || id == s->get_name().c_str()) {
          std::string resp = json::build_json([&](JsonObject out) {
            out["status"] = "ok";
            out["type"] = "sensor";
            out["state"] = s->state;
          });
          this->send_response(resp);
          return true;
        }
      }
      for (auto *sw : App.get_switches()) {
        char buf[128];
        sw->write_object_id_to(buf, sizeof(buf));
        if (id == buf || id == sw->get_name().c_str()) {
          std::string resp = json::build_json([&](JsonObject out) {
            out["status"] = "ok";
            out["type"] = "switch";
            out["state"] = sw->state;
          });
          this->send_response(resp);
          return true;
        }
      }
      for (auto *n : App.get_numbers()) {
        char buf[128];
        n->write_object_id_to(buf, sizeof(buf));
        if (id == buf || id == n->get_name().c_str()) {
          std::string resp = json::build_json([&](JsonObject out) {
            out["status"] = "ok";
            out["type"] = "number";
            out["state"] = n->state;
          });
          this->send_response(resp);
          return true;
        }
      }
      for (auto *bs : App.get_binary_sensors()) {
        char buf[128];
        bs->write_object_id_to(buf, sizeof(buf));
        if (id == buf || id == bs->get_name().c_str()) {
          std::string resp = json::build_json([&](JsonObject out) {
            out["status"] = "ok";
            out["type"] = "binary_sensor";
            out["state"] = bs->state;
          });
          this->send_response(resp);
          return true;
        }
      }
      auto it_sel = this->selects_.find(id);
      if (it_sel != this->selects_.end() && it_sel->second != nullptr) {
        std::string resp = json::build_json([&](JsonObject out) {
          out["status"] = "ok";
          out["type"] = "select";
          out["state"] = it_sel->second->current_option().c_str();
        });
        this->send_response(resp);
        return true;
      }
      auto it_cl = this->climates_.find(id);
      if (it_cl != this->climates_.end() && it_cl->second != nullptr) {
        std::string resp = json::build_json([&](JsonObject out) {
          out["status"] = "ok";
          out["type"] = "climate";
          out["current_temperature"] = it_cl->second->current_temperature;
          out["target_temperature"] = it_cl->second->target_temperature;
          out["action"] = LOG_STR_ARG(climate::climate_action_to_string(it_cl->second->action));
          out["mode"] = LOG_STR_ARG(climate::climate_mode_to_string(it_cl->second->mode));
        });
        this->send_response(resp);
        return true;
      }
      auto it_ts = this->text_sensors_.find(id);
      if (it_ts != this->text_sensors_.end() && it_ts->second != nullptr) {
        std::string resp = json::build_json([&](JsonObject out) {
          out["status"] = "ok";
          out["type"] = "text_sensor";
          out["state"] = it_ts->second->state.c_str();
        });
        this->send_response(resp);
        return true;
      }

      for (auto *sel : App.get_selects()) {
        char buf[128];
        sel->write_object_id_to(buf, sizeof(buf));
        if (id == buf || id == sel->get_name().c_str()) {
          std::string resp = json::build_json([&](JsonObject out) {
            out["status"] = "ok";
            out["type"] = "select";
            out["state"] = sel->current_option().c_str();
          });
          this->send_response(resp);
          return true;
        }
      }
      for (auto *cl : App.get_climates()) {
        char buf[128];
        cl->write_object_id_to(buf, sizeof(buf));
        if (id == buf || id == cl->get_name().c_str()) {
          std::string resp = json::build_json([&](JsonObject out) {
            out["status"] = "ok";
            out["type"] = "climate";
            out["current_temperature"] = cl->current_temperature;
            out["target_temperature"] = cl->target_temperature;
            out["action"] = LOG_STR_ARG(climate::climate_action_to_string(cl->action));
            out["mode"] = LOG_STR_ARG(climate::climate_mode_to_string(cl->mode));
          });
          this->send_response(resp);
          return true;
        }
      }
      for (auto *ts : App.get_text_sensors()) {
        char buf[128];
        ts->write_object_id_to(buf, sizeof(buf));
        if (id == buf || id == ts->get_name().c_str()) {
          std::string resp = json::build_json([&](JsonObject out) {
            out["status"] = "ok";
            out["type"] = "text_sensor";
            out["state"] = ts->state;
          });
          this->send_response(resp);
          return true;
        }
      }

      this->send_response("{\"status\":\"error\",\"message\":\"Entity not found: " + id + "\"}");
      return true;
    }

    if (cmd == "set_select") {
      std::string id = root["id"] | "";
      std::string opt = root["option"] | "";
      auto it = this->selects_.find(id);
      if (it != this->selects_.end() && it->second != nullptr) {
        it->second->make_call().set_option(opt).perform();
        this->send_response("{\"status\":\"ok\"}");
        return true;
      }
      for (auto *sel : App.get_selects()) {
        char buf[128];
        sel->write_object_id_to(buf, sizeof(buf));
        if (id == buf || id == sel->get_name().c_str()) {
          sel->make_call().set_option(opt).perform();
          this->send_response("{\"status\":\"ok\"}");
          return true;
        }
      }
      this->send_response("{\"status\":\"error\",\"message\":\"Select not found: " + id + "\"}");
      return true;
    }

    if (cmd == "set_climate") {
      std::string id = root["id"] | "";
      auto it = this->climates_.find(id);
      if (it != this->climates_.end() && it->second != nullptr) {
        auto call = it->second->make_call();
        if (root["target_temperature"].is<float>()) {
          call.set_target_temperature(root["target_temperature"].as<float>());
        }
        if (root["mode"].is<const char *>()) {
          std::string mode_str = root["mode"] | "";
          if (mode_str == "OFF") call.set_mode(climate::CLIMATE_MODE_OFF);
          else if (mode_str == "HEAT") call.set_mode(climate::CLIMATE_MODE_HEAT);
          else if (mode_str == "COOL") call.set_mode(climate::CLIMATE_MODE_COOL);
          else if (mode_str == "HEAT_COOL") call.set_mode(climate::CLIMATE_MODE_HEAT_COOL);
        }
        call.perform();
        this->send_response("{\"status\":\"ok\"}");
        return true;
      }
      this->send_response("{\"status\":\"error\",\"message\":\"Climate not found: " + id + "\"}");
      return true;
    }

    if (cmd == "advance_time") {
      uint32_t ms = root["ms"] | 0;
      s_virtual_time_offset_ms += ms;
      std::string resp = json::build_json([&](JsonObject out) {
        out["status"] = "ok";
        out["offset_ms"] = s_virtual_time_offset_ms;
        out["millis"] = millis();
      });
      this->send_response(resp);
      return true;
    }

    if (cmd == "reset_time") {
      s_virtual_time_offset_ms = 0;
      std::string resp = json::build_json([&](JsonObject out) {
        out["status"] = "ok";
        out["millis"] = millis();
      });
      this->send_response(resp);
      return true;
    }

    if (cmd == "get_time") {
      std::string resp = json::build_json([&](JsonObject out) {
        out["status"] = "ok";
        out["offset_ms"] = s_virtual_time_offset_ms;
        out["millis"] = millis();
      });
      this->send_response(resp);
      return true;
    }

    if (cmd == "step") {
      this->send_response("{\"status\":\"ok\"}");
      return true;
    }

    if (cmd == "exit") {
      this->send_response("{\"status\":\"ok\"}");
      ESP_LOGI(TAG, "Exit requested by test runner");
      ::exit(0);
      return true;
    }

    this->send_response("{\"status\":\"error\",\"message\":\"Unknown command: " + cmd + "\"}");
    return true;
  });
}

}  // namespace test_bridge
}  // namespace esphome
