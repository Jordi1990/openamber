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
#include <functional>
#include <string>

namespace esphome {
namespace openamber_eebus {

class EebusNode {
 public:
  EebusNode() = default;

  void set_ohpcf(OhpcfServer *ohpcf) { this->ohpcf_ = ohpcf; }
  void set_lpc(LpcServer *lpc) { this->lpc_ = lpc; }
  void set_device_id(const std::string &id) { this->device_id_ = id; }

  // Local addressing.
  const std::string &device_id() const { return this->device_id_; }
  const SpineAddress &compressor_entity() const { return this->compressor_; }
  const SpineAddress &cem_entity() const { return this->cem_; }

  // Outbound datagrams (ready to send over SHIP).
  std::string build_device_classification_notification();
  std::string build_measurement_notification(float power_w, float dhw_temp_c);
  std::string build_ohpcf_state_notification();
  std::string build_lpc_confirmation(uint32_t reply_to);
  std::string build_node_discovery_reply(uint32_t reply_to, const std::string &remote_device);
  std::string build_use_case_data_notify(uint32_t reply_to, const std::string &remote_device);

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
};

}  // namespace openamber_eebus
}  // namespace esphome
