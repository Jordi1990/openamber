/*
 * Open Amber - native EEBus PoC
 *
 * SPINE device node + outbound datagram builders (see header).
 */

#include "eebus_node.h"
#include "esphome/core/log.h"

#include <cstdio>

namespace esphome {
namespace openamber_eebus {

std::string EebusNode::build_device_classification_notification() {
  SpineDatagram d;
  d.set_source(this->compressor_);
  d.set_destination(SpineAddress());
  d.set_classifier(CmdClassifier::NOTIFY);
  d.set_msg_counter(this->next_counter());

  SpineCommand dev_cat;
  dev_cat.data_class = "deviceClassificationDeviceData";
  dev_cat.json = "{\"deviceCategory\":\"heatPump\",\"entityType\":\"compressor\"}";
  d.add_command(dev_cat);

  SpineCommand mfr;
  mfr.data_class = "deviceClassificationManufacturerData";
  mfr.json = "{\"deviceName\":\"OpenAmber\",\"brandName\":\"OpenAmber\",\"model\":\"Amber\"}";
  d.add_command(mfr);

  return d.to_json();
}

std::string EebusNode::build_measurement_notification(float power_w, float dhw_temp_c) {
  SpineDatagram d;
  d.set_source(this->compressor_);
  d.set_destination(SpineAddress());
  d.set_classifier(CmdClassifier::NOTIFY);
  d.set_msg_counter(this->next_counter());

  char buf[512];
  SpineCommand mpc;
  mpc.data_class = "measurementListData";
  snprintf(buf, sizeof(buf),
           "{\"measurementData\":[{\"measurementId\":0,\"measurementType\":\"power\","
           "\"unit\":\"W\",\"value\":%.1f},{\"measurementId\":1,"
           "\"measurementType\":\"temperature\",\"unit\":\"C\",\"value\":%.1f}]}",
           static_cast<double>(power_w), static_cast<double>(dhw_temp_c));
  mpc.json = buf;
  d.add_command(mpc);

  return d.to_json();
}

std::string EebusNode::build_ohpcf_state_notification() {
  SpineDatagram d;
  d.set_source(this->compressor_);
  d.set_destination(SpineAddress());
  d.set_classifier(CmdClassifier::NOTIFY);
  d.set_msg_counter(this->next_counter());

  const char *state = "available";
  if (this->ohpcf_ != nullptr) {
    switch (this->ohpcf_->get_state()) {
      case OhpcfState::SCHEDULED: state = "scheduled"; break;
      case OhpcfState::RUNNING: state = "running"; break;
      case OhpcfState::PAUSED: state = "paused"; break;
      default: state = "available";
    }
  }

  char buf[512];
  SpineCommand ops;
  ops.data_class = "deviceOperationsOperationalStateData";
  snprintf(buf, sizeof(buf),
           "{\"operationalState\":\"%s\",\"requestedPowerMaximum\":%.1f,"
           "\"minimalRunDuration\":%u,\"minimalPauseDuration\":%u}",
           state,
           static_cast<double>(this->ohpcf_ ? this->ohpcf_->get_requested_power_max_w() : 0.0f),
           static_cast<unsigned>(this->ohpcf_ ? this->ohpcf_->get_minimal_run_duration_s() : 600u),
           static_cast<unsigned>(this->ohpcf_ ? this->ohpcf_->get_minimal_pause_duration_s() : 600u));
  ops.json = buf;
  d.add_command(ops);

  return d.to_json();
}

std::string EebusNode::build_lpc_confirmation(uint32_t reply_to) {
  SpineDatagram d;
  d.set_source(this->compressor_);
  d.set_destination(this->cem_);
  d.set_classifier(CmdClassifier::REPLY);
  d.set_msg_counter(this->next_counter());
  d.set_msg_counter_reference(reply_to);

  SpineCommand res;
  res.data_class = "resultData";
  res.json = "{\"result\":\"success\"}";
  d.add_command(res);

  return d.to_json();
}

std::string EebusNode::build_node_discovery_reply(uint32_t reply_to,
                                                  const std::string &remote_device) {
  SpineDatagram d;
  SpineAddress src;
  src.device_str = "d:_n:OPENAMBER-" + this->device_id_;
  src.entity = 0;  // node management feature lives on entity[0]/feature[0]
  src.feature = 0;
  d.set_source(src);

  SpineAddress dst;
  dst.device_str = remote_device;
  dst.entity = 0;
  dst.feature = 0;
  d.set_destination(dst);
  d.set_classifier(CmdClassifier::REPLY);
  d.set_msg_counter(this->next_counter());
  d.set_msg_counter_reference(reply_to);

  // Reply with a valid SPINE discovery.  Per spine-go's model the discovery data
  // uses `description` wrappers (deviceInformation/entityInformation/
  // featureInformation each with a "description" object) and valid enum strings.
  std::string dev = src.device_str;
  char buf[2200];
  snprintf(buf, sizeof(buf),
           "{\"deviceInformation\":{\"description\":{\"deviceAddress\":\"%s\"}},"
           "\"entityInformation\":["
           "{\"description\":{\"entityAddress\":{\"device\":\"%s\",\"entity\":[1]},"
           "\"entityType\":\"Compressor\"}},"
           "{\"description\":{\"entityAddress\":{\"device\":\"%s\",\"entity\":[2]},"
           "\"entityType\":\"DHWCircuit\"}}"
           "],"
           "\"featureInformation\":["
           "{\"description\":{\"featureAddress\":{\"device\":\"%s\",\"entity\":[1],\"feature\":1},"
           "\"featureType\":\"measurement\",\"role\":\"server\"}},"
           "{\"description\":{\"featureAddress\":{\"device\":\"%s\",\"entity\":[2],\"feature\":1},"
           "\"featureType\":\"measurement\",\"role\":\"server\"}}"
           "]}",
           dev.c_str(), dev.c_str(), dev.c_str(), dev.c_str(), dev.c_str());

  SpineCommand disc;
  disc.data_class = "nodeManagementDetailedDiscoveryData";
  disc.json = buf;
  d.add_command(disc);

  // Use-case data (required for evcc to activate OHPCF/MPC/MDT/LPC).
  char ubuf[1600];
  snprintf(ubuf, sizeof(ubuf),
           "{\"useCaseInformation\":["
           "{\"address\":{\"device\":\"%s\",\"entity\":[1],\"feature\":1},"
           "\"actor\":\"compressor\",\"useCaseSupport\":[{\"useCase\":"
           "\"OptimizationOfSelfConsumptionByHeatPumpCompressorFlexibility\","
           "\"scenarioSupport\":[1,2]}]},"
           "{\"address\":{\"device\":\"%s\",\"entity\":[1],\"feature\":1},"
           "\"actor\":\"monitoringAppliance\",\"useCaseSupport\":[{\"useCase\":"
           "\"MonitoringOfPowerConsumption\",\"scenarioSupport\":[1,2,3]}]},"
           "{\"address\":{\"device\":\"%s\",\"entity\":[1],\"feature\":1},"
           "\"actor\":\"energyGuard\",\"useCaseSupport\":[{\"useCase\":"
           "\"LimitationOfPowerConsumption\",\"scenarioSupport\":[1,2,3,4]}]},"
           "{\"address\":{\"device\":\"%s\",\"entity\":[2],\"feature\":1},"
           "\"actor\":\"monitoringAppliance\",\"useCaseSupport\":[{\"useCase\":"
           "\"MonitoringOfDhwTemperature\",\"scenarioSupport\":[1]}]}"
           "]}",
           dev.c_str(), dev.c_str(), dev.c_str(), dev.c_str());

  SpineCommand ucdata;
  ucdata.data_class = "nodeManagementUseCaseData";
  ucdata.json = ubuf;
  d.add_command(ucdata);

  return d.to_json();
}

// A dedicated nodeManagementUseCaseData NOTIFY, which is what triggers
// evcc's use-case layer to activate OHPCF/MPC/MDT/LPC (UseCaseSupportUpdate).
std::string EebusNode::build_use_case_data_notify(uint32_t reply_to,
                                                  const std::string &remote_device) {
  SpineDatagram d;
  SpineAddress src;
  src.device_str = "d:_n:OPENAMBER-" + this->device_id_;
  src.entity = 0;
  src.feature = 0;
  d.set_source(src);
  SpineAddress dst;
  dst.device_str = remote_device;
  dst.entity = 0;
  dst.feature = 0;
  d.set_destination(dst);
  d.set_classifier(CmdClassifier::NOTIFY);
  d.set_msg_counter(this->next_counter());
  d.set_msg_counter_reference(reply_to);

  std::string dev = src.device_str;
  char ubuf[1600];
  snprintf(ubuf, sizeof(ubuf),
           "{\"useCaseInformation\":["
           "{\"address\":{\"device\":\"%s\",\"entity\":[1],\"feature\":1},"
           "\"actor\":\"compressor\",\"useCaseSupport\":[{\"useCase\":"
           "\"OptimizationOfSelfConsumptionByHeatPumpCompressorFlexibility\","
           "\"scenarioSupport\":[1,2]}]},"
           "{\"address\":{\"device\":\"%s\",\"entity\":[1],\"feature\":1},"
           "\"actor\":\"monitoringAppliance\",\"useCaseSupport\":[{\"useCase\":"
           "\"MonitoringOfPowerConsumption\",\"scenarioSupport\":[1,2,3]}]},"
           "{\"address\":{\"device\":\"%s\",\"entity\":[1],\"feature\":1},"
           "\"actor\":\"energyGuard\",\"useCaseSupport\":[{\"useCase\":"
           "\"LimitationOfPowerConsumption\",\"scenarioSupport\":[1,2,3,4]}]},"
           "{\"address\":{\"device\":\"%s\",\"entity\":[2],\"feature\":1},"
           "\"actor\":\"monitoringAppliance\",\"useCaseSupport\":[{\"useCase\":"
           "\"MonitoringOfDhwTemperature\",\"scenarioSupport\":[1]}]}"
           "]}",
           dev.c_str(), dev.c_str(), dev.c_str(), dev.c_str());

  SpineCommand ucdata;
  ucdata.data_class = "nodeManagementUseCaseData";
  ucdata.json = ubuf;
  d.add_command(ucdata);

  return d.to_json();
}

std::string EebusNode::handle_inbound(const std::string &json) {
  SpineInboundCommand in;
  if (!spine_parse_dispatch(json, in) || !in.valid)
    return "";

  // Node management: answer a detailed discovery read with our device/entity
  // description so evcc can bind the OHPCF use case.
  std::string discovery;
  if (in.find("nodeManagementDetailedDiscoveryData", discovery)) {
    ESP_LOGI("eebus_node", "Discovery request detected (classifier=%d, reply_to=%u, remote=%s)",
             static_cast<int>(in.classifier), static_cast<unsigned>(in.msg_counter),
             in.source.device_str.c_str());
    return this->build_node_discovery_reply(in.msg_counter, in.source.device_str);
  }

  // LPC WriteConsumptionLimit (dim).
  std::string lpc_json;
  if (in.find("consumptionLimitListData", lpc_json)) {
    // Body: [{"consumptionLimit":{"value":..,"isActive":..}}]
    bool active = lpc_json.find("\"isActive\":true") != std::string::npos;
    float value = 0.0f;
    auto vp = lpc_json.find("\"value\":");
    if (vp != std::string::npos)
      value = static_cast<float>(std::atof(lpc_json.c_str() + vp + 8));
    if (this->lpc_ != nullptr)
      this->lpc_->write_limit(value, active);
    return this->build_lpc_confirmation(in.msg_counter);
  }

  // OHPCF commands: schedule / resume / pause / abort.
  if (in.find("operationStateControlData", lpc_json)) {
    // Body carries the requested state/action.
    if (this->ohpcf_ != nullptr) {
      if (lpc_json.find("start") != std::string::npos || lpc_json.find("schedule") != std::string::npos)
        this->ohpcf_->schedule(0);
      else if (lpc_json.find("resume") != std::string::npos)
        this->ohpcf_->resume();
      else if (lpc_json.find("pause") != std::string::npos)
        this->ohpcf_->pause();
      else if (lpc_json.find("abort") != std::string::npos)
        this->ohpcf_->abort();
    }
    return this->build_lpc_confirmation(in.msg_counter);
  }

  return "";
}

}  // namespace openamber_eebus
}  // namespace esphome
