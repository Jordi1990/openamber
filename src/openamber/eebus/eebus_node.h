/*
 * Open Amber - native EEBus PoC
 *
 * SPINE device node: owns the local device/entity/feature addresses, the
 * message counter, the outbound datagram builders for the use-cases evcc
 * expects (node management, device classification, MPC/MDT measurement, OHPCF,
 * LPC), and routes inbound commands to the OHPCF/LPC servers.
 *
 * The heat-pump presents itself to evcc as a "Compressor" entity with the
 * OHPCF use-case; see docs/eebus-native-poc.md §10.
 */

#pragma once

#include "esphome/core/component.h"
#include "eebus_spine.h"
#include "eebus_ohpcf.h"
#include "eebus_lpc.h"
#include "eebus_measurements.h"
#include <functional>
#include <string>

namespace esphome {
namespace openamber_eebus {

class EebusNode {
 public:
  EebusNode() = default;

  void set_ohpcf(OhpcfServer *ohpcf) { this->ohpcf_ = ohpcf; }
  void set_lpc(LpcServer *lpc) { this->lpc_ = lpc; }
  void set_mpc(MpcServer *mpc) { this->mpc_ = mpc; }
  void set_mdt(MdtServer *mdt) { this->mdt_ = mdt; }
  void set_device_id(const std::string &id) { this->device_id_ = id; }

  void set_ship_sender(const std::function<bool(const std::string &)> &sender) { this->ship_sender_ = sender; }

  // Local addressing.
  const std::string &device_id() const { return this->device_id_; }
  const SpineAddress &compressor_entity() const { return this->compressor_; }
  const SpineAddress &cem_entity() const { return this->cem_; }
  const std::string &remote_device() const { return this->remote_device_; }

  // Outbound datagrams (ready to send over SHIP).
  std::string build_device_classification_notification();
  std::string build_device_classification_device_reply(uint32_t reply_to, const SpineAddress &dst, const SpineAddress &src_addr = {});
  std::string build_device_classification_manufacturer_reply(uint32_t reply_to, const SpineAddress &dst, const SpineAddress &src_addr = {});
  std::string build_device_configuration_description_reply(uint32_t reply_to, const SpineAddress &dst, const SpineAddress &src_addr = {});
  std::string build_device_configuration_key_value_reply(uint32_t reply_to, const SpineAddress &dst, const SpineAddress &src_addr = {});
  std::string build_measurement_notification(float power_w, float dhw_temp_c);
  std::string build_measurement_reply(uint32_t reply_to, const SpineAddress &dst, const SpineAddress &src_addr = {});
  std::string build_measurement_description_reply(uint32_t reply_to, const SpineAddress &dst, const SpineAddress &src_addr = {});
  std::string build_electrical_connection_description_reply(uint32_t reply_to, const SpineAddress &dst, const SpineAddress &src_addr = {});
  std::string build_electrical_connection_parameter_reply(uint32_t reply_to, const SpineAddress &dst, const SpineAddress &src_addr = {});
  std::string build_smart_energy_management_reply(uint32_t reply_to, const SpineAddress &dst, const SpineAddress &src_addr = {});
  std::string build_load_control_limit_reply(uint32_t reply_to, const SpineAddress &dst, const SpineAddress &src_addr = {});
  std::string build_load_control_description_reply(uint32_t reply_to, const SpineAddress &dst, const SpineAddress &src_addr = {});
  std::string build_load_control_limit_notification(bool active, float limit_w);
  std::string build_ohpcf_state_notification();
  std::string build_ohpcf_state_reply(uint32_t reply_to, const SpineAddress &dst, const SpineAddress &src_addr = {});
  std::string build_lpc_confirmation(uint32_t reply_to);
  std::string build_result(uint32_t reply_to, const SpineAddress &dst, const SpineAddress &src);
  std::string build_node_discovery_reply(uint32_t reply_to, const SpineAddress &dst);
  std::string build_use_case_data_reply(uint32_t reply_to, const SpineAddress &dst);
  std::string build_use_case_data_notify(uint32_t reply_to, const SpineAddress &dst);

  // High-level notification helpers (sends via ship_sender_ if connected/subscribed).
  bool notify_measurements(float power_w, float dhw_temp_c);
  bool notify_ohpcf();
  bool notify_lpc(bool active, float limit_w);
  bool notify_use_cases();

  bool has_remote_device() const { return !this->remote_device_.empty(); }
  bool has_subscriptions() const {
    return this->subscribed_measurements_ || this->subscribed_ohpcf_ || this->subscribed_lpc_ || this->subscribed_nodemgmt_;
  }
  bool is_subscribed_measurements() const { return this->subscribed_measurements_; }
  bool is_subscribed_ohpcf() const { return this->subscribed_ohpcf_; }
  bool is_subscribed_lpc() const { return this->subscribed_lpc_; }
  bool is_subscribed_nodemgmt() const { return this->subscribed_nodemgmt_; }

  void reset_subscriptions() {
    this->subscribed_measurements_ = false;
    this->subscribed_ohpcf_ = false;
    this->subscribed_lpc_ = false;
    this->subscribed_nodemgmt_ = false;
    this->subscriber_measurement_addr_ = SpineAddress();
    this->subscriber_ohpcf_addr_ = SpineAddress();
    this->subscriber_lpc_addr_ = SpineAddress();
    this->subscriber_nodemgmt_addr_ = SpineAddress();
    this->remote_device_.clear();
  }

  uint32_t get_last_activity_ms() const { return this->last_activity_ms_; }

  // Handles an inbound datagram: routes OHPCF/LPC commands, returns an outbound
  // reply/confirmation string (empty if none).
  std::string handle_inbound(const std::string &json);

 private:
  uint32_t next_counter() { return ++this->msg_counter_; }

  uint32_t msg_counter_{0};
  std::string device_id_{"OpenAmber"};
  SpineAddress compressor_{1, 1, 1};
  SpineAddress cem_{2, 1, 1};

  OhpcfServer *ohpcf_{nullptr};
  LpcServer *lpc_{nullptr};
  MpcServer *mpc_{nullptr};
  MdtServer *mdt_{nullptr};
  std::string remote_device_;  // peer (CEM) device address, learned from inbound discovery

  std::function<bool(const std::string &)> ship_sender_;
  uint32_t last_activity_ms_{0};
  bool subscribed_measurements_{false};
  bool subscribed_ohpcf_{false};
  bool subscribed_lpc_{false};
  bool subscribed_nodemgmt_{false};

  SpineAddress subscriber_measurement_addr_;
  SpineAddress subscriber_ohpcf_addr_;
  SpineAddress subscriber_lpc_addr_;
  SpineAddress subscriber_nodemgmt_addr_;
};

}  // namespace openamber_eebus
}  // namespace esphome
