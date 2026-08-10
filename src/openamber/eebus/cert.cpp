/*
 * Open Amber - native EEBus PoC
 *
 * Certificate store: persists the generated pair to NVS and falls back to an
 * embedded build-time pair otherwise.  A stable SKI is required for evcc
 * pairing, so the pair must not change across reboots.
 *
 * Runtime regeneration with mbedTLS x509write is not available in the
 * default ESP-IDF build (the x509write module is excluded), so the PoC embeds
 * a pre-generated EC P-256 self-signed pair.  Regenerate and replace the
 * constants below before deployment; SKI must equal the certificate's
 * SubjectKeyIdentifier (hex).
 */

#include "cert.h"
#include "nvs_store.h"
#include "esphome/core/log.h"

namespace esphome {
namespace openamber_eebus {

static const char *const TAG = "eebus_cert";

// Embedded build-time pair (EC P-256, self-signed, CN=OpenAmber).
static const char kEmbeddedCertPem[] =
    "-----BEGIN CERTIFICATE-----\n"
    "MIIBSDCB76ADAgECAgEBMAoGCCqGSM49BAMCMBQxEjAQBgNVBAMMCU9wZW5BbWJl\n"
    "cjAeFw0yNjA4MDYyMDQzMTNaFw0zMTA4MDYyMDQzMTNaMBQxEjAQBgNVBAMMCU9w\n"
    "ZW5BbWJlcjBZMBMGByqGSM49AgEGCCqGSM49AwEHA0IABJRZslFPH87kxibrj8Gb\n"
    "ndNd4IuOUcoACsx+WZiaAMbRl9xjpOYzyQbBrazSetoXwdYmZNfd9Spa9yy6XkuV\n"
    "cwmjMjAwMA8GA1UdEwEB/wQFMAMBAf8wHQYDVR0OBBYEFPLzw8xm6WJgARescRB1\n"
    "dAHcEXjFMAoGCCqGSM49BAMCA0gAMEUCIQCe97zeqclE73RxWXo95PbtVXKhBwH7\n"
    "skc2lGSeKqLrqAIgbW3Lq++TYsEN10naR1RrGjTQhyQW8iIsM0BlnL2YAKI=\n"
    "-----END CERTIFICATE-----\n";

static const char kEmbeddedKeyPem[] =
    "-----BEGIN PRIVATE KEY-----\n"
    "MIGHAgEAMBMGByqGSM49AgEGCCqGSM49AwEHBG0wawIBAQQgj8P/Dc7+oRKpDcyJ\n"
    "7FAiSkDlJPPTmr8WBn1q79SFRbOhRANCAASUWbJRTx/O5MYm64/Bm53TXeCLjlHK\n"
    "AArMflmYmgDG0ZfcY6TmM8kGwa2s0nraF8HWJmTX3fUqWvcsul5LlXMJ\n"
    "-----END PRIVATE KEY-----\n";

static const char kEmbeddedSki[] = "F2F3C3CC66E962600117AC7110757401DC1178C5";

bool EEBusCertificateStore::load_or_create() {
  if (this->load_from_nvs()) {
    ESP_LOGI(TAG, "Loaded EEBus certificate from NVS");
    return true;
  }
  return this->use_embedded();
}

bool EEBusCertificateStore::use_embedded() {
  this->cert_pem_ = kEmbeddedCertPem;
  this->key_pem_ = kEmbeddedKeyPem;
  this->ski_ = kEmbeddedSki;
  if (!this->save_to_nvs()) {
    ESP_LOGW(TAG, "Could not persist embedded certificate; will re-embed on next boot");
  }
  ESP_LOGI(TAG, "Using embedded EEBus certificate (SKI %s)", this->ski_.c_str());
  return true;
}

bool EEBusCertificateStore::save_to_nvs() {
  EEBusNvsStore store;
  return store.write_string("eebus_cert", this->cert_pem_) &&
         store.write_string("eebus_key", this->key_pem_);
}

bool EEBusCertificateStore::load_from_nvs() {
  EEBusNvsStore store;
  if (!store.read_string("eebus_cert", this->cert_pem_) ||
      !store.read_string("eebus_key", this->key_pem_)) {
    return false;
  }
  this->ski_ = kEmbeddedSki;
  ESP_LOGI(TAG, "Loaded EEBus certificate from NVS (SKI %s)", this->ski_.c_str());
  return true;
}

}  // namespace openamber_eebus
}  // namespace esphome
