#pragma once

#include "esphome/core/component.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"
#include "esphome/components/json/json_util.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/number/number.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/select/select.h"
#include "esphome/components/climate/climate.h"
#include "lvgl.h"

#include <string>
#include <vector>
#include <unordered_map>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
typedef int socklen_t;
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#endif

namespace esphome {
namespace test_bridge {

class TestBridge : public Component {
 public:
  void set_port(int port) { this->port_ = port; }

  void register_widget(const std::string &id, lv_obj_t **ptr) {
    this->widgets_[id] = ptr;
  }
  void register_sensor(const std::string &id, sensor::Sensor *s) {
    this->sensors_[id] = s;
  }
  void register_number(const std::string &id, number::Number *n) {
    this->numbers_[id] = n;
  }
  void register_switch(const std::string &id, switch_::Switch *sw) {
    this->switches_[id] = sw;
  }
  void register_binary_sensor(const std::string &id, binary_sensor::BinarySensor *bs) {
    this->binary_sensors_[id] = bs;
  }
  void register_select(const std::string &id, select::Select *sel) {
    this->selects_[id] = sel;
  }
  void register_climate(const std::string &id, climate::Climate *clim) {
    this->climates_[id] = clim;
  }
  void register_text_sensor(const std::string &id, text_sensor::TextSensor *ts) {
    this->text_sensors_[id] = ts;
  }

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::LATE; }

 private:
  int port_{8888};
  int server_fd_{-1};
  int client_fd_{-1};
  std::string rx_buffer_;
  std::unordered_map<std::string, lv_obj_t **> widgets_;
  std::unordered_map<std::string, sensor::Sensor *> sensors_;
  std::unordered_map<std::string, number::Number *> numbers_;
  std::unordered_map<std::string, switch_::Switch *> switches_;
  std::unordered_map<std::string, binary_sensor::BinarySensor *> binary_sensors_;
  std::unordered_map<std::string, select::Select *> selects_;
  std::unordered_map<std::string, climate::Climate *> climates_;
  std::unordered_map<std::string, text_sensor::TextSensor *> text_sensors_;

  void process_line(const std::string &line);
  void send_response(const std::string &json_str);
  void close_client();
  lv_obj_t *resolve_widget(const std::string &id);
};

}  // namespace test_bridge
}  // namespace esphome
