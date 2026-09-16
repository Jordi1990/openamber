#pragma once

#include "esphome/core/component.h"
#include "esphome/components/update/update_entity.h"

namespace esphome {
namespace mock_update {

class MockUpdate : public update::UpdateEntity, public Component {
 public:
  void setup() override {
    this->update_info_.current_version = "1.0.0";
    this->update_info_.latest_version = "1.0.0";
    this->update_info_.title = "OpenAmber Mock";
    this->publish_state();
  }
  void perform(bool force) override {}
  void check() override {}
};

}  // namespace mock_update
}  // namespace esphome
