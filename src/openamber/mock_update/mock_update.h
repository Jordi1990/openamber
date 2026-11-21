#pragma once

#include <string>

#include "esphome/core/component.h"
#include "esphome/components/update/update_entity.h"

namespace esphome {
namespace mock_update {

// Simulator stand-in for the http_request update entity. It serves one version per update channel: a source URL
// ending in ".beta.manifest.json" gets the beta manifest version, any other URL the stable one. Like ESPHome's http_request
// update, any version that differs from the installed one counts as available.
class MockUpdate : public update::UpdateEntity, public Component {
 public:
  void setup() override {
    this->update_info_.title = "OpenAmber Mock";
    this->set_mock_versions("1.0.0", "1.0.0", "1.0.0");
  }
  void perform(bool force) override { this->perform_count_++; }
  void check() override {
    bool beta = this->source_url_.find(".beta.manifest.json") != std::string::npos;
    this->update_info_.current_version = this->current_;
    this->update_info_.latest_version = beta ? this->beta_latest_ : this->stable_latest_;
    this->state_ = this->update_info_.latest_version.empty() ||
                           this->update_info_.latest_version == this->update_info_.current_version
                       ? update::UPDATE_STATE_NO_UPDATE
                       : update::UPDATE_STATE_AVAILABLE;
    this->publish_state();
  }

  void set_source_url(const char *url) { this->source_url_ = url; }
  void set_source_url(const std::string &url) { this->source_url_ = url; }
  const std::string &get_source_url() const { return this->source_url_; }

  // Simulator only: the installed version and the version each channel's manifest offers.
  void set_mock_versions(const std::string &current, const std::string &stable_latest,
                         const std::string &beta_latest) {
    this->current_ = current;
    this->stable_latest_ = stable_latest;
    this->beta_latest_ = beta_latest;
    this->check();
  }
  int get_perform_count() const { return this->perform_count_; }

 protected:
  std::string source_url_;
  std::string current_;
  std::string stable_latest_;
  std::string beta_latest_;
  int perform_count_{0};
};

}  // namespace mock_update
}  // namespace esphome
