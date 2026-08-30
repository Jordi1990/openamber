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
  SpineAddress src;
  src.device_str = "d:_n:OPENAMBER-" + this->device_id_;
  src.entity = 1;
  src.feature = 1;
  d.set_source(src);
  SpineAddress dst;
  dst.device_str = this->remote_device_;
  dst.entity = 0;
  dst.feature = 0;
  d.set_destination(dst);
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

std::string EebusNode::build_device_classification_reply(uint32_t reply_to, const std::string &remote_device) {
  SpineDatagram d;
  SpineAddress src;
  src.device_str = "d:_n:OPENAMBER-" + this->device_id_;
  src.entity = 1;
  src.feature = 4;
  d.set_source(src);
  SpineAddress dst;
  dst.device_str = remote_device;
  dst.entity = 0;
  dst.feature = 0;
  d.set_destination(dst);
  d.set_classifier(CmdClassifier::REPLY);
  d.set_msg_counter(this->next_counter());
  d.set_msg_counter_reference(reply_to);

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
  SpineAddress src;
  src.device_str = "d:_n:OPENAMBER-" + this->device_id_;
  src.entity = 1;
  src.feature = 1;
  d.set_source(src);
  SpineAddress dst;
  dst.device_str = this->remote_device_;
  dst.entity = 0;
  dst.feature = 0;
  d.set_destination(dst);
  d.set_classifier(CmdClassifier::NOTIFY);
  d.set_msg_counter(this->next_counter());

  int64_t p_num = static_cast<int64_t>(power_w);
  int64_t t_num = static_cast<int64_t>(dhw_temp_c * 10.0f);

  char buf[512];
  SpineCommand mpc;
  mpc.data_class = "measurementListData";
  snprintf(buf, sizeof(buf),
           "{\"measurementData\":[{\"measurementId\":0,\"value\":{\"number\":%lld,\"scale\":0},\"valueState\":\"normal\"},"
           "{\"measurementId\":1,\"value\":{\"number\":%lld,\"scale\":-1},\"valueState\":\"normal\"}]}",
           static_cast<long long>(p_num), static_cast<long long>(t_num));
  mpc.json = buf;
  d.add_command(mpc);

  return d.to_json();
}

std::string EebusNode::build_measurement_reply(uint32_t reply_to, const std::string &remote_device) {
  SpineDatagram d;
  SpineAddress src;
  src.device_str = "d:_n:OPENAMBER-" + this->device_id_;
  src.entity = 1;
  src.feature = 1;
  d.set_source(src);
  SpineAddress dst;
  dst.device_str = remote_device;
  dst.entity = 0;
  dst.feature = 0;
  d.set_destination(dst);
  d.set_classifier(CmdClassifier::REPLY);
  d.set_msg_counter(this->next_counter());
  d.set_msg_counter_reference(reply_to);

  float power_w = this->mpc_ ? this->mpc_->get_power_w() : 0.0f;
  float dhw_temp = this->mdt_ ? this->mdt_->get_temperature_c() : 0.0f;
  int64_t p_num = static_cast<int64_t>(power_w);
  int64_t t_num = static_cast<int64_t>(dhw_temp * 10.0f);

  char buf[512];
  SpineCommand mpc;
  mpc.data_class = "measurementListData";
  snprintf(buf, sizeof(buf),
           "{\"measurementData\":[{\"measurementId\":0,\"value\":{\"number\":%lld,\"scale\":0},\"valueState\":\"normal\"},"
           "{\"measurementId\":1,\"value\":{\"number\":%lld,\"scale\":-1},\"valueState\":\"normal\"}]}",
           static_cast<long long>(p_num), static_cast<long long>(t_num));
  mpc.json = buf;
  d.add_command(mpc);

  return d.to_json();
}

std::string EebusNode::build_measurement_description_reply(uint32_t reply_to, const std::string &remote_device) {
  SpineDatagram d;
  SpineAddress src;
  src.device_str = "d:_n:OPENAMBER-" + this->device_id_;
  src.entity = 1;
  src.feature = 1;
  d.set_source(src);
  SpineAddress dst;
  dst.device_str = remote_device;
  dst.entity = 0;
  dst.feature = 0;
  d.set_destination(dst);
  d.set_classifier(CmdClassifier::REPLY);
  d.set_msg_counter(this->next_counter());
  d.set_msg_counter_reference(reply_to);

  SpineCommand desc;
  desc.data_class = "measurementDescriptionListData";
  desc.json = "{\"measurementDescriptionData\":["
              "{\"measurementId\":0,\"measurementType\":\"power\",\"commodityType\":\"electricity\",\"unit\":\"W\",\"scopeType\":\"acPowerTotal\"},"
              "{\"measurementId\":1,\"measurementType\":\"temperature\",\"commodityType\":\"hotWater\",\"unit\":\"degC\",\"scopeType\":\"heatAmount\"}"
              "]}";
  d.add_command(desc);

  return d.to_json();
}

std::string EebusNode::build_electrical_connection_reply(uint32_t reply_to, const std::string &remote_device) {
  SpineDatagram d;
  SpineAddress src;
  src.device_str = "d:_n:OPENAMBER-" + this->device_id_;
  src.entity = 1;
  src.feature = 2;
  d.set_source(src);
  SpineAddress dst;
  dst.device_str = remote_device;
  dst.entity = 0;
  dst.feature = 0;
  d.set_destination(dst);
  d.set_classifier(CmdClassifier::REPLY);
  d.set_msg_counter(this->next_counter());
  d.set_msg_counter_reference(reply_to);

  SpineCommand cmd1;
  cmd1.data_class = "electricalConnectionDescriptionListData";
  cmd1.json = "{\"electricalConnectionDescriptionData\":[{\"electricalConnectionId\":0,\"positiveEnergyDirection\":\"consume\"}]}";
  d.add_command(cmd1);

  SpineCommand cmd2;
  cmd2.data_class = "electricalConnectionParameterDescriptionListData";
  cmd2.json = "{\"electricalConnectionParameterDescriptionData\":[{\"electricalConnectionId\":0,\"parameterId\":0,\"measurementId\":0,\"acMeasuredPhases\":\"ABC\"}]}";
  d.add_command(cmd2);

  return d.to_json();
}

std::string EebusNode::build_smart_energy_management_reply(uint32_t reply_to, const std::string &remote_device) {
  SpineDatagram d;
  SpineAddress src;
  src.device_str = "d:_n:OPENAMBER-" + this->device_id_;
  src.entity = 1;
  src.feature = 3;
  d.set_source(src);
  SpineAddress dst;
  dst.device_str = remote_device;
  dst.entity = 0;
  dst.feature = 0;
  d.set_destination(dst);
  d.set_classifier(CmdClassifier::REPLY);
  d.set_msg_counter(this->next_counter());
  d.set_msg_counter_reference(reply_to);

  const char *state = "available";
  if (this->ohpcf_ != nullptr) {
    switch (this->ohpcf_->get_state()) {
      case OhpcfState::SCHEDULED: state = "scheduled"; break;
      case OhpcfState::RUNNING: state = "running"; break;
      case OhpcfState::PAUSED: state = "paused"; break;
      default: state = "available";
    }
  }

  char buf[768];
  snprintf(buf, sizeof(buf),
           "{\"smartEnergyManagementPsData\":{\"alternatives\":[{\"powerSequence\":[{"
           "\"description\":{\"sequenceId\":0},"
           "\"powerTimeSlot\":[{\"timeSlotId\":0,\"valueList\":{\"value\":["
           "{\"valueType\":\"power\",\"value\":{\"number\":%lld,\"scale\":0}},"
           "{\"valueType\":\"powerMax\",\"value\":{\"number\":%lld,\"scale\":0}}"
           "]}}],"
           "\"operatingConstraintsInterrupt\":{\"isPausable\":true,\"isStoppable\":true},"
           "\"operatingConstraintsDuration\":{\"activeDurationMin\":\"PT10M\",\"pauseDurationMin\":\"PT10M\"},"
           "\"state\":{\"state\":\"%s\"}"
           "}]}]}}",
           static_cast<long long>(this->ohpcf_ ? this->ohpcf_->get_requested_power_estimate_w() : 1500.0f),
           static_cast<long long>(this->ohpcf_ ? this->ohpcf_->get_requested_power_max_w() : 3000.0f),
           state);

  SpineCommand cmd;
  cmd.data_class = "smartEnergyManagementPsData";
  cmd.json = buf;
  d.add_command(cmd);

  return d.to_json();
}

std::string EebusNode::build_load_control_reply(uint32_t reply_to, const std::string &remote_device) {
  SpineDatagram d;
  SpineAddress src;
  src.device_str = "d:_n:OPENAMBER-" + this->device_id_;
  src.entity = 1;
  src.feature = 4;
  d.set_source(src);
  SpineAddress dst;
  dst.device_str = remote_device;
  dst.entity = 0;
  dst.feature = 0;
  d.set_destination(dst);
  d.set_classifier(CmdClassifier::REPLY);
  d.set_msg_counter(this->next_counter());
  d.set_msg_counter_reference(reply_to);

  bool active = this->lpc_ ? this->lpc_->is_dimmed() : false;
  int64_t limit_val = static_cast<int64_t>(this->lpc_ ? this->lpc_->get_limit_value_w() : 0.0f);

  char buf[512];
  snprintf(buf, sizeof(buf),
           "{\"loadControlLimitData\":[{\"limitId\":0,\"isLimitActive\":%s,\"value\":{\"number\":%lld,\"scale\":0}}]}",
           active ? "true" : "false", static_cast<long long>(limit_val));

  SpineCommand cmd1;
  cmd1.data_class = "loadControlLimitListData";
  cmd1.json = buf;
  d.add_command(cmd1);

  SpineCommand cmd2;
  cmd2.data_class = "loadControlLimitDescriptionListData";
  cmd2.json = "{\"loadControlLimitDescriptionData\":[{\"limitId\":0,\"limitType\":\"signOffCategory\",\"limitCategory\":\"obligation\",\"unit\":\"W\",\"scopeType\":\"heatPump\"}]}";
  d.add_command(cmd2);

  return d.to_json();
}

std::string EebusNode::build_ohpcf_state_notification() {
  SpineDatagram d;
  SpineAddress src;
  src.device_str = "d:_n:OPENAMBER-" + this->device_id_;
  src.entity = 1;
  src.feature = 3;
  d.set_source(src);
  SpineAddress dst;
  dst.device_str = this->remote_device_;
  dst.entity = 0;
  dst.feature = 0;
  d.set_destination(dst);
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

  char buf[768];
  snprintf(buf, sizeof(buf),
           "{\"smartEnergyManagementPsData\":{\"alternatives\":[{\"powerSequence\":[{"
           "\"description\":{\"sequenceId\":0},"
           "\"powerTimeSlot\":[{\"timeSlotId\":0,\"valueList\":{\"value\":["
           "{\"valueType\":\"power\",\"value\":{\"number\":%lld,\"scale\":0}},"
           "{\"valueType\":\"powerMax\",\"value\":{\"number\":%lld,\"scale\":0}}"
           "]}}],"
           "\"operatingConstraintsInterrupt\":{\"isPausable\":true,\"isStoppable\":true},"
           "\"operatingConstraintsDuration\":{\"activeDurationMin\":\"PT10M\",\"pauseDurationMin\":\"PT10M\"},"
           "\"state\":{\"state\":\"%s\"}"
           "}]}]}}",
           static_cast<long long>(this->ohpcf_ ? this->ohpcf_->get_requested_power_estimate_w() : 1500.0f),
           static_cast<long long>(this->ohpcf_ ? this->ohpcf_->get_requested_power_max_w() : 3000.0f),
           state);

  SpineCommand cmd;
  cmd.data_class = "smartEnergyManagementPsData";
  cmd.json = buf;
  d.add_command(cmd);

  return d.to_json();
}

std::string EebusNode::build_ohpcf_state_reply(uint32_t reply_to, const std::string &remote_device) {
  SpineDatagram d;
  SpineAddress src;
  src.device_str = "d:_n:OPENAMBER-" + this->device_id_;
  src.entity = 1;
  src.feature = 2;
  d.set_source(src);
  SpineAddress dst;
  dst.device_str = remote_device;
  dst.entity = 0;
  dst.feature = 0;
  d.set_destination(dst);
  d.set_classifier(CmdClassifier::REPLY);
  d.set_msg_counter(this->next_counter());
  d.set_msg_counter_reference(reply_to);

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

std::string EebusNode::build_result(uint32_t reply_to, const SpineAddress &dst, const SpineAddress &src) {
  SpineDatagram d;
  d.set_source(src);
  d.set_destination(dst);
  d.set_classifier(CmdClassifier::RESULT);
  d.set_msg_counter(this->next_counter());
  d.set_msg_counter_reference(reply_to);

  SpineCommand res;
  res.data_class = "resultData";
  res.json = "{\"errorNumber\":0}";
  d.add_command(res);

  return d.to_json();
}

std::string EebusNode::build_lpc_confirmation(uint32_t reply_to) {
  return this->build_result(reply_to, this->cem_, this->compressor_);
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
  // featureInformation each with a "description" object) and valid enum strings
  // including supportedFunction / possibleOperations so operations are permitted.
  std::string dev = src.device_str;
  std::string buf;
  buf.reserve(4096);
  buf += "{\"deviceInformation\":{\"description\":{\"deviceAddress\":{\"device\":\"" + dev + "\"},\"deviceType\":\"HeatPumpAppliance\",\"networkFeatureSet\":\"smart\"}},";
  buf += "\"entityInformation\":[";
  buf += "{\"description\":{\"entityAddress\":{\"device\":\"" + dev + "\",\"entity\":[0]},\"entityType\":\"DeviceInformation\"}},";
  buf += "{\"description\":{\"entityAddress\":{\"device\":\"" + dev + "\",\"entity\":[1]},\"entityType\":\"Compressor\"}},";
  buf += "{\"description\":{\"entityAddress\":{\"device\":\"" + dev + "\",\"entity\":[2]},\"entityType\":\"DHWCircuit\"}}";
  buf += "],";
  buf += "\"featureInformation\":[";
  buf += "{\"description\":{\"featureAddress\":{\"device\":\"" + dev + "\",\"entity\":[0],\"feature\":0},\"featureType\":\"NodeManagement\",\"role\":\"special\",\"supportedFunction\":[";
  buf += "{\"function\":\"nodeManagementDetailedDiscoveryData\",\"possibleOperations\":{\"read\":{}}},";
  buf += "{\"function\":\"nodeManagementSubscriptionRequestCall\",\"possibleOperations\":{}},";
  buf += "{\"function\":\"nodeManagementBindingRequestCall\",\"possibleOperations\":{}},";
  buf += "{\"function\":\"nodeManagementUseCaseData\",\"possibleOperations\":{\"read\":{}}}";
  buf += "]}},";
  buf += "{\"description\":{\"featureAddress\":{\"device\":\"" + dev + "\",\"entity\":[1],\"feature\":1},\"featureType\":\"Measurement\",\"role\":\"server\",\"supportedFunction\":[";
  buf += "{\"function\":\"measurementDescriptionListData\",\"possibleOperations\":{\"read\":{}}},";
  buf += "{\"function\":\"measurementListData\",\"possibleOperations\":{\"read\":{}}},";
  buf += "{\"function\":\"measurementConstraintsListData\",\"possibleOperations\":{\"read\":{}}}";
  buf += "]}},";
  buf += "{\"description\":{\"featureAddress\":{\"device\":\"" + dev + "\",\"entity\":[1],\"feature\":2},\"featureType\":\"ElectricalConnection\",\"role\":\"server\",\"supportedFunction\":[";
  buf += "{\"function\":\"electricalConnectionDescriptionListData\",\"possibleOperations\":{\"read\":{}}},";
  buf += "{\"function\":\"electricalConnectionParameterDescriptionListData\",\"possibleOperations\":{\"read\":{}}}";
  buf += "]}},";
  buf += "{\"description\":{\"featureAddress\":{\"device\":\"" + dev + "\",\"entity\":[1],\"feature\":3},\"featureType\":\"SmartEnergyManagementPs\",\"role\":\"server\",\"supportedFunction\":[";
  buf += "{\"function\":\"smartEnergyManagementPsData\",\"possibleOperations\":{\"read\":{},\"write\":{}}}";
  buf += "]}},";
  buf += "{\"description\":{\"featureAddress\":{\"device\":\"" + dev + "\",\"entity\":[1],\"feature\":4},\"featureType\":\"LoadControl\",\"role\":\"server\",\"supportedFunction\":[";
  buf += "{\"function\":\"loadControlLimitDescriptionListData\",\"possibleOperations\":{\"read\":{}}},";
  buf += "{\"function\":\"loadControlLimitListData\",\"possibleOperations\":{\"read\":{},\"write\":{}}}";
  buf += "]}},";
  buf += "{\"description\":{\"featureAddress\":{\"device\":\"" + dev + "\",\"entity\":[1],\"feature\":5},\"featureType\":\"DeviceClassification\",\"role\":\"server\",\"supportedFunction\":[";
  buf += "{\"function\":\"deviceClassificationManufacturerData\",\"possibleOperations\":{\"read\":{}}},";
  buf += "{\"function\":\"deviceClassificationDeviceData\",\"possibleOperations\":{\"read\":{}}}";
  buf += "]}},";
  buf += "{\"description\":{\"featureAddress\":{\"device\":\"" + dev + "\",\"entity\":[1],\"feature\":6},\"featureType\":\"DeviceConfiguration\",\"role\":\"server\",\"supportedFunction\":[";
  buf += "{\"function\":\"deviceConfigurationKeyValueDescriptionListData\",\"possibleOperations\":{\"read\":{}}},";
  buf += "{\"function\":\"deviceConfigurationKeyValueListData\",\"possibleOperations\":{\"read\":{}}}";
  buf += "]}},";
  buf += "{\"description\":{\"featureAddress\":{\"device\":\"" + dev + "\",\"entity\":[2],\"feature\":1},\"featureType\":\"Measurement\",\"role\":\"server\",\"supportedFunction\":[";
  buf += "{\"function\":\"measurementDescriptionListData\",\"possibleOperations\":{\"read\":{}}},";
  buf += "{\"function\":\"measurementListData\",\"possibleOperations\":{\"read\":{}}},";
  buf += "{\"function\":\"measurementConstraintsListData\",\"possibleOperations\":{\"read\":{}}}";
  buf += "]}},";
  buf += "{\"description\":{\"featureAddress\":{\"device\":\"" + dev + "\",\"entity\":[2],\"feature\":2},\"featureType\":\"ElectricalConnection\",\"role\":\"server\",\"supportedFunction\":[";
  buf += "{\"function\":\"electricalConnectionDescriptionListData\",\"possibleOperations\":{\"read\":{}}},";
  buf += "{\"function\":\"electricalConnectionParameterDescriptionListData\",\"possibleOperations\":{\"read\":{}}}";
  buf += "]}}";
  buf += "]}";

  SpineCommand disc;
  disc.data_class = "nodeManagementDetailedDiscoveryData";
  disc.json = std::move(buf);
  d.add_command(disc);

  // Use-case data (required for evcc to activate OHPCF/MPC/MDT/LPC).
  std::string ubuf;
  ubuf.reserve(1600);
  ubuf += "{\"useCaseInformation\":[";
  ubuf += "{\"address\":{\"device\":\"" + dev + "\",\"entity\":[1],\"feature\":1},\"actor\":\"Compressor\",\"useCaseSupport\":[{\"useCaseName\":\"optimizationOfSelfConsumptionByHeatPumpCompressorFlexibility\",\"useCaseVersion\":\"1.0.0\",\"useCaseAvailable\":true,\"scenarioSupport\":[1,2]}]},";
  ubuf += "{\"address\":{\"device\":\"" + dev + "\",\"entity\":[1],\"feature\":1},\"actor\":\"MonitoredUnit\",\"useCaseSupport\":[{\"useCaseName\":\"monitoringOfPowerConsumption\",\"useCaseVersion\":\"1.0.0\",\"useCaseAvailable\":true,\"scenarioSupport\":[1,2,3]}]},";
  ubuf += "{\"address\":{\"device\":\"" + dev + "\",\"entity\":[1],\"feature\":1},\"actor\":\"ControllableSystem\",\"useCaseSupport\":[{\"useCaseName\":\"limitationOfPowerConsumption\",\"useCaseVersion\":\"1.0.0\",\"useCaseAvailable\":true,\"scenarioSupport\":[1,2,3,4]}]},";
  ubuf += "{\"address\":{\"device\":\"" + dev + "\",\"entity\":[2],\"feature\":1},\"actor\":\"DHWCircuit\",\"useCaseSupport\":[{\"useCaseName\":\"monitoringOfDhwTemperature\",\"useCaseVersion\":\"1.0.0\",\"useCaseAvailable\":true,\"scenarioSupport\":[1]}]}";
  ubuf += "]}";

  SpineCommand ucdata;
  ucdata.data_class = "nodeManagementUseCaseData";
  ucdata.json = std::move(ubuf);
  d.add_command(ucdata);

  return d.to_json();
}

std::string EebusNode::build_use_case_data_reply(uint32_t reply_to,
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
  d.set_classifier(CmdClassifier::REPLY);
  d.set_msg_counter(this->next_counter());
  d.set_msg_counter_reference(reply_to);

  std::string dev = src.device_str;
  std::string ubuf;
  ubuf.reserve(1600);
  ubuf += "{\"useCaseInformation\":[";
  ubuf += "{\"address\":{\"device\":\"" + dev + "\",\"entity\":[1],\"feature\":1},\"actor\":\"Compressor\",\"useCaseSupport\":[{\"useCaseName\":\"optimizationOfSelfConsumptionByHeatPumpCompressorFlexibility\",\"useCaseVersion\":\"1.0.0\",\"useCaseAvailable\":true,\"scenarioSupport\":[1,2]}]},";
  ubuf += "{\"address\":{\"device\":\"" + dev + "\",\"entity\":[1],\"feature\":1},\"actor\":\"MonitoredUnit\",\"useCaseSupport\":[{\"useCaseName\":\"monitoringOfPowerConsumption\",\"useCaseVersion\":\"1.0.0\",\"useCaseAvailable\":true,\"scenarioSupport\":[1,2,3]}]},";
  ubuf += "{\"address\":{\"device\":\"" + dev + "\",\"entity\":[1],\"feature\":1},\"actor\":\"ControllableSystem\",\"useCaseSupport\":[{\"useCaseName\":\"limitationOfPowerConsumption\",\"useCaseVersion\":\"1.0.0\",\"useCaseAvailable\":true,\"scenarioSupport\":[1,2,3,4]}]},";
  ubuf += "{\"address\":{\"device\":\"" + dev + "\",\"entity\":[2],\"feature\":1},\"actor\":\"DHWCircuit\",\"useCaseSupport\":[{\"useCaseName\":\"monitoringOfDhwTemperature\",\"useCaseVersion\":\"1.0.0\",\"useCaseAvailable\":true,\"scenarioSupport\":[1]}]}";
  ubuf += "]}";

  SpineCommand ucdata;
  ucdata.data_class = "nodeManagementUseCaseData";
  ucdata.json = std::move(ubuf);
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
  std::string ubuf;
  ubuf.reserve(1600);
  ubuf += "{\"useCaseInformation\":[";
  ubuf += "{\"address\":{\"device\":\"" + dev + "\",\"entity\":[1],\"feature\":1},\"actor\":\"Compressor\",\"useCaseSupport\":[{\"useCaseName\":\"optimizationOfSelfConsumptionByHeatPumpCompressorFlexibility\",\"useCaseVersion\":\"1.0.0\",\"useCaseAvailable\":true,\"scenarioSupport\":[1,2]}]},";
  ubuf += "{\"address\":{\"device\":\"" + dev + "\",\"entity\":[1],\"feature\":1},\"actor\":\"MonitoredUnit\",\"useCaseSupport\":[{\"useCaseName\":\"monitoringOfPowerConsumption\",\"useCaseVersion\":\"1.0.0\",\"useCaseAvailable\":true,\"scenarioSupport\":[1,2,3]}]},";
  ubuf += "{\"address\":{\"device\":\"" + dev + "\",\"entity\":[1],\"feature\":1},\"actor\":\"ControllableSystem\",\"useCaseSupport\":[{\"useCaseName\":\"limitationOfPowerConsumption\",\"useCaseVersion\":\"1.0.0\",\"useCaseAvailable\":true,\"scenarioSupport\":[1,2,3,4]}]},";
  ubuf += "{\"address\":{\"device\":\"" + dev + "\",\"entity\":[2],\"feature\":1},\"actor\":\"DHWCircuit\",\"useCaseSupport\":[{\"useCaseName\":\"monitoringOfDhwTemperature\",\"useCaseVersion\":\"1.0.0\",\"useCaseAvailable\":true,\"scenarioSupport\":[1]}]}";
  ubuf += "]}";

  SpineCommand ucdata;
  ucdata.data_class = "nodeManagementUseCaseData";
  ucdata.json = std::move(ubuf);
  d.add_command(ucdata);

  return d.to_json();
}

std::string EebusNode::handle_inbound(const std::string &json) {
  SpineInboundCommand in;
  if (!spine_parse_dispatch(json, in) || !in.valid)
    return "";

  if (!in.source.device_str.empty()) {
    this->remote_device_ = in.source.device_str;
  }

  // Node management: answer a detailed discovery read with our device/entity
  // description so evcc can bind the OHPCF use case.
  std::string dummy;
  if (in.find("nodeManagementDetailedDiscoveryData", dummy)) {
    ESP_LOGI("eebus_node", "Discovery request detected (classifier=%d, reply_to=%u, remote=%s)",
             static_cast<int>(in.classifier), static_cast<unsigned>(in.msg_counter),
             in.source.device_str.c_str());
    return this->build_node_discovery_reply(in.msg_counter, in.source.device_str);
  }

  // Node management: subscription or binding request calls (e.g. EVCC subscribing to NodeManagement).
  if (in.find("nodeManagementSubscriptionRequestCall", dummy) ||
      in.find("nodeManagementBindingRequestCall", dummy)) {
    ESP_LOGI("eebus_node", "Subscription/Binding request detected (reply_to=%u, remote=%s)",
             static_cast<unsigned>(in.msg_counter), in.source.device_str.c_str());
    SpineAddress src;
    src.device_str = "d:_n:OPENAMBER-" + this->device_id_;
    src.entity = in.destination.entity;
    src.feature = in.destination.feature;
    return this->build_result(in.msg_counter, in.source, src);
  }

  // Node management: use case data read.
  if (in.find("nodeManagementUseCaseData", dummy) && in.classifier == CmdClassifier::READ) {
    ESP_LOGI("eebus_node", "UseCaseData read request detected (reply_to=%u, remote=%s)",
             static_cast<unsigned>(in.msg_counter), in.source.device_str.c_str());
    return this->build_use_case_data_reply(in.msg_counter, in.source.device_str);
  }

  // Measurement: description list data read.
  if (in.find("measurementDescriptionListData", dummy) && in.classifier == CmdClassifier::READ) {
    ESP_LOGI("eebus_node", "Measurement description read detected (reply_to=%u, remote=%s)",
             static_cast<unsigned>(in.msg_counter), in.source.device_str.c_str());
    return this->build_measurement_description_reply(in.msg_counter, in.source.device_str);
  }

  // Measurement: measurement list data read.
  if (in.find("measurementListData", dummy) && in.classifier == CmdClassifier::READ) {
    ESP_LOGI("eebus_node", "Measurement data read detected (reply_to=%u, remote=%s)",
             static_cast<unsigned>(in.msg_counter), in.source.device_str.c_str());
    return this->build_measurement_reply(in.msg_counter, in.source.device_str);
  }

  // Measurement: constraints list data read.
  if (in.find("measurementConstraintsListData", dummy) && in.classifier == CmdClassifier::READ) {
    ESP_LOGI("eebus_node", "Measurement constraints read detected (reply_to=%u, remote=%s)",
             static_cast<unsigned>(in.msg_counter), in.source.device_str.c_str());
    SpineDatagram d;
    SpineAddress src;
    src.device_str = "d:_n:OPENAMBER-" + this->device_id_;
    src.entity = 1;
    src.feature = 1;
    d.set_source(src);
    SpineAddress dst;
    dst.device_str = in.source.device_str;
    dst.entity = 0;
    dst.feature = 0;
    d.set_destination(dst);
    d.set_classifier(CmdClassifier::REPLY);
    d.set_msg_counter(this->next_counter());
    d.set_msg_counter_reference(in.msg_counter);
    SpineCommand cmd;
    cmd.data_class = "measurementConstraintsListData";
    cmd.json = "{\"measurementConstraintsData\":[{\"measurementId\":0}]}";
    d.add_command(cmd);
    return d.to_json();
  }

  // Device configuration read.
  if ((in.find("deviceConfigurationKeyValueDescriptionListData", dummy) ||
       in.find("deviceConfigurationKeyValueListData", dummy)) && in.classifier == CmdClassifier::READ) {
    ESP_LOGI("eebus_node", "Device configuration read detected (reply_to=%u, remote=%s)",
             static_cast<unsigned>(in.msg_counter), in.source.device_str.c_str());
    SpineDatagram d;
    SpineAddress src;
    src.device_str = "d:_n:OPENAMBER-" + this->device_id_;
    src.entity = 1;
    src.feature = 6;
    d.set_source(src);
    SpineAddress dst;
    dst.device_str = in.source.device_str;
    dst.entity = 0;
    dst.feature = 0;
    d.set_destination(dst);
    d.set_classifier(CmdClassifier::REPLY);
    d.set_msg_counter(this->next_counter());
    d.set_msg_counter_reference(in.msg_counter);
    SpineCommand cmd;
    cmd.data_class = "deviceConfigurationKeyValueListData";
    cmd.json = "{\"deviceConfigurationKeyValueData\":[{\"keyId\":0,\"value\":{\"string\":\"OpenAmber\"}}]}";
    d.add_command(cmd);
    return d.to_json();
  }

  // Device classification data read.
  if ((in.find("deviceClassificationManufacturerData", dummy) ||
       in.find("deviceClassificationDeviceData", dummy)) && in.classifier == CmdClassifier::READ) {
    ESP_LOGI("eebus_node", "Device classification read detected (reply_to=%u, remote=%s)",
             static_cast<unsigned>(in.msg_counter), in.source.device_str.c_str());
    return this->build_device_classification_reply(in.msg_counter, in.source.device_str);
  }

  // Device operational state read.
  if (in.find("deviceOperationsOperationalStateData", dummy) && in.classifier == CmdClassifier::READ) {
    ESP_LOGI("eebus_node", "Operational state read detected (reply_to=%u, remote=%s)",
             static_cast<unsigned>(in.msg_counter), in.source.device_str.c_str());
    return this->build_ohpcf_state_reply(in.msg_counter, in.source.device_str);
  }

  // Electrical connection data read.
  if ((in.find("electricalConnectionDescriptionListData", dummy) ||
       in.find("electricalConnectionParameterDescriptionListData", dummy)) && in.classifier == CmdClassifier::READ) {
    ESP_LOGI("eebus_node", "Electrical connection read detected (reply_to=%u, remote=%s)",
             static_cast<unsigned>(in.msg_counter), in.source.device_str.c_str());
    return this->build_electrical_connection_reply(in.msg_counter, in.source.device_str);
  }

  // SmartEnergyManagementPs (OHPCF) data read.
  if (in.find("smartEnergyManagementPsData", dummy) && in.classifier == CmdClassifier::READ) {
    ESP_LOGI("eebus_node", "SmartEnergyManagementPs read detected (reply_to=%u, remote=%s)",
             static_cast<unsigned>(in.msg_counter), in.source.device_str.c_str());
    return this->build_smart_energy_management_reply(in.msg_counter, in.source.device_str);
  }

  // LoadControl (LPC) data read.
  if ((in.find("loadControlLimitDescriptionListData", dummy) ||
       in.find("loadControlLimitListData", dummy)) && in.classifier == CmdClassifier::READ) {
    ESP_LOGI("eebus_node", "LoadControl read detected (reply_to=%u, remote=%s)",
             static_cast<unsigned>(in.msg_counter), in.source.device_str.c_str());
    return this->build_load_control_reply(in.msg_counter, in.source.device_str);
  }

  // SmartEnergyManagementPs (OHPCF) write (schedule / resume / pause / abort).
  std::string semps_json;
  if (in.find("smartEnergyManagementPsData", semps_json) && (in.classifier == CmdClassifier::WRITE || in.classifier == CmdClassifier::CALL)) {
    ESP_LOGI("eebus_node", "SmartEnergyManagementPs write/call received");
    if (this->ohpcf_ != nullptr) {
      if (semps_json.find("\"schedule\"") != std::string::npos || semps_json.find("\"startTime\"") != std::string::npos)
        this->ohpcf_->schedule(0);
      else if (semps_json.find("\"running\"") != std::string::npos)
        this->ohpcf_->resume();
      else if (semps_json.find("\"paused\"") != std::string::npos)
        this->ohpcf_->pause();
      else if (semps_json.find("\"invalid\"") != std::string::npos)
        this->ohpcf_->abort();
    }
    SpineAddress src;
    src.device_str = "d:_n:OPENAMBER-" + this->device_id_;
    src.entity = in.destination.entity;
    src.feature = in.destination.feature;
    return this->build_result(in.msg_counter, in.source, src);
  }

  // LoadControl (LPC) write (dim).
  std::string lpc_json;
  if (in.find("loadControlLimitListData", lpc_json) && (in.classifier == CmdClassifier::WRITE || in.classifier == CmdClassifier::CALL)) {
    bool active = lpc_json.find("\"isLimitActive\":true") != std::string::npos;
    float value = 0.0f;
    auto vp = lpc_json.find("\"number\":");
    if (vp != std::string::npos)
      value = static_cast<float>(std::atof(lpc_json.c_str() + vp + 9));
    if (this->lpc_ != nullptr)
      this->lpc_->write_limit(value, active);
    SpineAddress src;
    src.device_str = "d:_n:OPENAMBER-" + this->device_id_;
    src.entity = in.destination.entity;
    src.feature = in.destination.feature;
    return this->build_result(in.msg_counter, in.source, src);
  }

  // LPC WriteConsumptionLimit (dim legacy fallback).
  if (in.find("consumptionLimitListData", lpc_json)) {
    // Body: [{"consumptionLimit":{"value":..,"isActive":..}}]
    bool active = lpc_json.find("\"isActive\":true") != std::string::npos;
    float value = 0.0f;
    auto vp = lpc_json.find("\"value\":");
    if (vp != std::string::npos)
      value = static_cast<float>(std::atof(lpc_json.c_str() + vp + 8));
    if (this->lpc_ != nullptr)
      this->lpc_->write_limit(value, active);
    SpineAddress src;
    src.device_str = "d:_n:OPENAMBER-" + this->device_id_;
    src.entity = in.destination.entity;
    src.feature = in.destination.feature;
    return this->build_result(in.msg_counter, in.source, src);
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
    SpineAddress src;
    src.device_str = "d:_n:OPENAMBER-" + this->device_id_;
    src.entity = in.destination.entity;
    src.feature = in.destination.feature;
    return this->build_result(in.msg_counter, in.source, src);
  }

  // General fallback for any other CALL or WRITE commands requiring acknowledgement
  if (in.classifier == CmdClassifier::CALL || in.classifier == CmdClassifier::WRITE) {
    ESP_LOGI("eebus_node", "Acknowledging generic command (reply_to=%u)",
             static_cast<unsigned>(in.msg_counter));
    SpineAddress src;
    src.device_str = "d:_n:OPENAMBER-" + this->device_id_;
    src.entity = in.destination.entity;
    src.feature = in.destination.feature;
    return this->build_result(in.msg_counter, in.source, src);
  }

  return "";
}

}  // namespace openamber_eebus
}  // namespace esphome
