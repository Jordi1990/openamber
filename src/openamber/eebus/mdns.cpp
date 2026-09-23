/*
 * Open Amber - native EEBus PoC
 *
 * mDNS advertisement of the EEBus SHIP service (_ship._tcp).
 *
 * Uses the ESP-IDF mdns (managed `espressif__mdns` component), same as
 * ESPHome's own mdns component.  The TXT record set follows SHIP 7.3.2 and what
 * eebus-go/evcc parses as mandatory: txtvers, id, path, ski, register.
 */

#include "mdns.h"
#include "esphome/core/defines.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

#include <cstring>

#ifdef USE_MDNS
#include <mdns.h>
#endif

namespace esphome {
namespace openamber_eebus {

bool eebus_mdns_start(const std::string &device_sku, const std::string &brand,
                      const std::string &model, const std::string &ski,
                      const std::string &ship_id, uint16_t ship_port) {
#ifdef USE_MDNS
  esp_err_t err = mdns_init();
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    ESP_LOGW("eebus_mdns", "mdns_init failed: %d", err);
    return false;
  }

  // Ensure hostname is configured on the mDNS server
  const char *hostname = App.get_name().c_str();
  mdns_hostname_set(hostname);
  mdns_instance_name_set(hostname);

  // If the service was previously registered, remove it first to allow updating records cleanly
  mdns_service_remove("_ship", "_tcp");

  std::string ski_lower = ski;
  for (auto &c : ski_lower) c = std::tolower(static_cast<unsigned char>(c));

  std::string id_lower = ship_id;
  for (auto &c : id_lower) c = std::tolower(static_cast<unsigned char>(c));

  std::string serial_val = brand.empty() ? "OpenAmber" : brand;
  serial_val += "-";
  serial_val += model.empty() ? "Amber" : model;

  // EEBUS SHIP 7.3.2 DNS-SD TXT records:
  // txtvers=1, path=/ship/, id=<shipid>, ski=<cert SKI>, register=true,
  // brand=<brand>, model=<model>, type=HeatPump, cat=3 (HVAC), serial=<serial>
  mdns_txt_item_t txt[] = {
      {"txtvers", "1"},
      {"path", "/ship/"},
      {"id", id_lower.c_str()},
      {"ski", ski_lower.c_str()},
      {"register", "true"},
      {"brand", brand.empty() ? "OpenAmber" : brand.c_str()},
      {"model", model.empty() ? "Amber" : model.c_str()},
      {"type", "HeatPump"},
      {"cat", "3"},
      {"serial", serial_val.c_str()},
  };

  err = mdns_service_add(nullptr, "_ship", "_tcp", ship_port, txt,
                         sizeof(txt) / sizeof(txt[0]));
  if (err != ESP_OK) {
    ESP_LOGW("eebus_mdns", "mdns_service_add(_ship._tcp) failed: %d", err);
    return false;
  }
  ESP_LOGI("eebus_mdns", "Advertised _ship._tcp on port %u (ski=%s id=%s brand=%s model=%s type=HeatPump cat=3 register=true)",
           static_cast<unsigned>(ship_port), ski_lower.c_str(), id_lower.c_str(),
           brand.empty() ? "OpenAmber" : brand.c_str(),
           model.empty() ? "Amber" : model.c_str());
  return true;
#else
  ESP_LOGI("eebus_mdns", "mDNS unavailable (USE_MDNS not enabled); "
                         "_ship._tcp not advertised");
  return false;
#endif
}

void eebus_mdns_set_registered(bool registered) {
#ifdef USE_MDNS
  // In SHIP 7.3.2: register=true means device is in pairing / registration mode.
  // When an EMS is connected (registered == true), pairing mode is closed (register=false).
  // When no EMS is connected (registered == false), pairing mode is open (register=true).
  const char *val = registered ? "false" : "true";
  esp_err_t err = mdns_service_txt_item_set("_ship", "_tcp", "register", val);
  if (err == ESP_OK) {
    ESP_LOGI("eebus_mdns", "mDNS register TXT record set to %s", val);
  } else {
    ESP_LOGW("eebus_mdns", "mdns_service_txt_item_set(register=%s) failed: %d", val, err);
  }
#endif
}

void eebus_mdns_stop() {
#ifdef USE_MDNS
  mdns_service_remove("_ship", "_tcp");
  ESP_LOGI("eebus_mdns", "Stopped advertising _ship._tcp on mDNS");
#endif
}

}  // namespace openamber_eebus
}  // namespace esphome
