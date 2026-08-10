/*
 * Open Amber - native EEBus PoC
 *
 * ESP-IDF NVS (non-volatile storage) persistence.
 */

#include "nvs_store.h"
#include "esphome/core/log.h"
#include "nvs.h"
#include "nvs_flash.h"

namespace esphome {
namespace openamber_eebus {

static const char *const TAG = "eebus_nvs";
static const char *const kNvsNamespace = "openamber_eebus";

static bool ensure_init() {
  static bool inited = false;
  static bool ok = false;
  if (inited)
    return ok;
  inited = true;
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    nvs_flash_erase();
    err = nvs_flash_init();
  }
  ok = (err == ESP_OK);
  if (!ok)
    ESP_LOGE(TAG, "NVS init failed: %d", err);
  return ok;
}

bool EEBusNvsStore::write_string(const char *key, const std::string &value) {
  if (!ensure_init())
    return false;
  nvs_handle_t handle;
  if (nvs_open(kNvsNamespace, NVS_READWRITE, &handle) != ESP_OK)
    return false;
  esp_err_t err = nvs_set_str(handle, key, value.c_str());
  if (err == ESP_OK)
    err = nvs_commit(handle);
  nvs_close(handle);
  return err == ESP_OK;
}

bool EEBusNvsStore::read_string(const char *key, std::string &out) {
  if (!ensure_init())
    return false;
  nvs_handle_t handle;
  if (nvs_open(kNvsNamespace, NVS_READONLY, &handle) != ESP_OK)
    return false;
  size_t len = 0;
  esp_err_t err = nvs_get_str(handle, key, nullptr, &len);
  if (err != ESP_OK || len == 0) {
    nvs_close(handle);
    return false;
  }
  std::string tmp(len - 1, '\0');
  err = nvs_get_str(handle, key, &tmp[0], &len);
  nvs_close(handle);
  if (err != ESP_OK)
    return false;
  out.swap(tmp);
  return true;
}

}  // namespace openamber_eebus
}  // namespace esphome
