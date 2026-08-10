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

#include <cstring>

#ifdef USE_MDNS
#include <mdns.h>
#endif

namespace esphome {
namespace openamber_eebus {

void eebus_mdns_start(const std::string &device_sku, const std::string &brand,
                      const std::string &model, const std::string &ski,
                      const std::string &ship_id, uint16_t ship_port) {
#ifdef USE_MDNS
  esp_err_t err = mdns_init();
  if (err != ESP_OK) {
    // ESP_OK_INVALID_STATE means already initialised (e.g. by ESPHome mdns).
    if (err == ESP_ERR_INVALID_STATE) {
      // already running, fine
    } else {
      ESP_LOGW("eebus_mdns", "mdns_init failed: %d", err);
      return;
    }
  }

  // txtvers=1, id=<shipid>, path=/ship/, ski=<cert SKI>, register=false
  mdns_txt_item_t txt[] = {
      {"txtvers", "1"},
      {"id", ship_id.c_str()},
      {"path", "/ship/"},
      {"ski", ski.c_str()},
      {"register", "false"},
  };

  err = mdns_service_add(nullptr, "_ship", "_tcp", ship_port, txt,
                         sizeof(txt) / sizeof(txt[0]));
  if (err != ESP_OK) {
    ESP_LOGW("eebus_mdns", "mdns_service_add(_ship._tcp) failed: %d", err);
    return;
  }
  ESP_LOGI("eebus_mdns", "Advertised _ship._tcp on port %u (ski=%s shipid=%s)",
           static_cast<unsigned>(ship_port), ski.c_str(), ship_id.c_str());
#else
  ESP_LOGI("eebus_mdns", "mDNS unavailable (USE_MDNS not enabled); "
                         "_ship._tcp not advertised");
#endif
}

}  // namespace openamber_eebus
}  // namespace esphome
