/*
 * Native EEBUS component for ESPHome
 *
 * Certificate store: persists the generated pair to NVS and falls back to an
 * embedded build-time pair otherwise.  A stable SKI is required for EMS/CEM
 * pairing, so the pair must not change across reboots.
 *
 * Each installation dynamically generates a unique EC P-256 keypair
 * and self-signed X.509 certificate on first boot, storing them in NVS along
 * with the RFC 5280 Subject Key Identifier (SKI).
 */


#include "cert.h"
#include "nvs_store.h"
#include "esphome/core/log.h"

#include <cstdio>
#include <cstring>
#include <vector>

#include "esp_random.h"
#include "mbedtls/pk.h"
#include "mbedtls/ecp.h"
#include "mbedtls/x509_crt.h"
#include "mbedtls/sha1.h"

namespace esphome {
namespace eebus {

static const char *const TAG = "eebus_cert";

static int cert_mbedtls_rng(void *, unsigned char *buf, size_t len) {
  esp_fill_random(buf, len);
  return 0;
}

bool EEBusCertificateStore::load_or_create() {
  if (this->load_from_nvs()) {
    if (!this->cert_pem_.empty() && !this->ski_.empty()) {
      ESP_LOGI(TAG, "Loaded unique EEBus certificate from NVS (SKI %s)", this->ski_.c_str());
      return true;
    }
  }

  ESP_LOGI(TAG, "Generating unique EEBus EC P-256 keypair and certificate...");
  if (this->generate_and_save()) {
    return true;
  }

  ESP_LOGE(TAG, "Dynamic certificate generation failed; clearing credentials");
  this->cert_pem_.clear();
  this->key_pem_.clear();
  this->ski_.clear();
  return false;
}

bool EEBusCertificateStore::generate_and_save() {
  mbedtls_pk_context key;
  mbedtls_pk_init(&key);

  const mbedtls_pk_info_t *pk_info = mbedtls_pk_info_from_type(MBEDTLS_PK_ECKEY);
  int ret = mbedtls_pk_setup(&key, pk_info);
  if (ret != 0) {
    ESP_LOGE(TAG, "mbedtls_pk_setup failed: -0x%04x", -ret);
    mbedtls_pk_free(&key);
    return false;
  }

  ret = mbedtls_ecp_gen_key(MBEDTLS_ECP_DP_SECP256R1, mbedtls_pk_ec(key),
                            cert_mbedtls_rng, nullptr);
  if (ret != 0) {
    ESP_LOGE(TAG, "mbedtls_ecp_gen_key failed: -0x%04x", -ret);
    mbedtls_pk_free(&key);
    return false;
  }

  std::vector<unsigned char> key_buf(1024, 0);
  ret = mbedtls_pk_write_key_pem(&key, key_buf.data(), key_buf.size());
  if (ret != 0) {
    ESP_LOGE(TAG, "mbedtls_pk_write_key_pem failed: -0x%04x", -ret);
    mbedtls_pk_free(&key);
    return false;
  }
  std::string generated_key_pem = reinterpret_cast<char *>(key_buf.data());

  // Compute Subject Key Identifier (SKI) = SHA-1(uncompressed public key point)
  std::vector<unsigned char> pubkey_buf(128, 0);
  unsigned char *pub_ptr = pubkey_buf.data() + pubkey_buf.size();
  int pub_len = mbedtls_pk_write_pubkey(&pub_ptr, pubkey_buf.data(), &key);
  if (pub_len <= 0) {
    ESP_LOGE(TAG, "mbedtls_pk_write_pubkey failed: -0x%04x", -pub_len);
    mbedtls_pk_free(&key);
    return false;
  }

  unsigned char sha1_out[20];
  ret = mbedtls_sha1(pub_ptr, static_cast<size_t>(pub_len), sha1_out);
  if (ret != 0) {
    ESP_LOGE(TAG, "mbedtls_sha1 failed: -0x%04x", -ret);
    mbedtls_pk_free(&key);
    return false;
  }

  char ski_buf[41];
  for (size_t i = 0; i < 20; i++) {
    snprintf(&ski_buf[i * 2], 3, "%02x", sha1_out[i]);
  }
  ski_buf[40] = '\0';
  std::string generated_ski = ski_buf;

  // Build X.509 certificate
  mbedtls_x509write_cert crt;
  mbedtls_x509write_crt_init(&crt);
  mbedtls_x509write_crt_set_version(&crt, MBEDTLS_X509_CRT_VERSION_3);
  mbedtls_x509write_crt_set_md_alg(&crt, MBEDTLS_MD_SHA256);
  mbedtls_x509write_crt_set_subject_key(&crt, &key);
  mbedtls_x509write_crt_set_issuer_key(&crt, &key);

  std::string dn = "CN=" + (this->common_name_.empty() ? "EEBUS-Device" : this->common_name_);
  ret = mbedtls_x509write_crt_set_subject_name(&crt, dn.c_str());
  if (ret != 0) {
    ESP_LOGE(TAG, "set_subject_name failed: -0x%04x", -ret);
    mbedtls_x509write_crt_free(&crt);
    mbedtls_pk_free(&key);
    return false;
  }

  ret = mbedtls_x509write_crt_set_issuer_name(&crt, dn.c_str());
  if (ret != 0) {
    ESP_LOGE(TAG, "set_issuer_name failed: -0x%04x", -ret);
    mbedtls_x509write_crt_free(&crt);
    mbedtls_pk_free(&key);
    return false;
  }

  unsigned char serial[16];
  esp_fill_random(serial, sizeof(serial));
  serial[0] &= 0x7F;
  if (serial[0] == 0)
    serial[0] = 1;
  ret = mbedtls_x509write_crt_set_serial_raw(&crt, serial, sizeof(serial));
  if (ret != 0) {
    ESP_LOGE(TAG, "set_serial_raw failed: -0x%04x", -ret);
    mbedtls_x509write_crt_free(&crt);
    mbedtls_pk_free(&key);
    return false;
  }

  ret = mbedtls_x509write_crt_set_validity(&crt, "20260101000000", "20360101000000");
  if (ret != 0) {
    ESP_LOGE(TAG, "set_validity failed: -0x%04x", -ret);
    mbedtls_x509write_crt_free(&crt);
    mbedtls_pk_free(&key);
    return false;
  }

  ret = mbedtls_x509write_crt_set_basic_constraints(&crt, 1, -1);
  if (ret != 0) {
    ESP_LOGE(TAG, "set_basic_constraints failed: -0x%04x", -ret);
    mbedtls_x509write_crt_free(&crt);
    mbedtls_pk_free(&key);
    return false;
  }

  ret = mbedtls_x509write_crt_set_subject_key_identifier(&crt);
  if (ret != 0) {
    ESP_LOGE(TAG, "set_subject_key_identifier failed: -0x%04x", -ret);
    mbedtls_x509write_crt_free(&crt);
    mbedtls_pk_free(&key);
    return false;
  }

  std::vector<unsigned char> cert_buf(2048, 0);
  ret = mbedtls_x509write_crt_pem(&crt, cert_buf.data(), cert_buf.size(),
                                 cert_mbedtls_rng, nullptr);
  mbedtls_x509write_crt_free(&crt);
  mbedtls_pk_free(&key);

  if (ret != 0) {
    ESP_LOGE(TAG, "mbedtls_x509write_crt_pem failed: -0x%04x", -ret);
    return false;
  }

  std::string generated_cert_pem = reinterpret_cast<char *>(cert_buf.data());

  this->cert_pem_ = std::move(generated_cert_pem);
  this->key_pem_ = std::move(generated_key_pem);
  this->ski_ = std::move(generated_ski);

  if (!this->save_to_nvs()) {
    ESP_LOGW(TAG, "Could not persist dynamically generated certificate to NVS!");
  } else {
    ESP_LOGI(TAG, "Generated and saved unique EEBus certificate to NVS (SKI %s)", this->ski_.c_str());
  }
  return true;
}

bool EEBusCertificateStore::save_to_nvs() {
  EEBusNvsStore store;
  return store.write_string("eebus_cert", this->cert_pem_) &&
         store.write_string("eebus_key", this->key_pem_) &&
         store.write_string("eebus_ski", this->ski_);
}

bool EEBusCertificateStore::load_from_nvs() {
  EEBusNvsStore store;
  if (!store.read_string("eebus_cert", this->cert_pem_) ||
      !store.read_string("eebus_key", this->key_pem_)) {
    return false;
  }
  if (!store.read_string("eebus_ski", this->ski_) || this->ski_.empty()) {
    mbedtls_x509_crt crt;
    mbedtls_x509_crt_init(&crt);
    int ret = mbedtls_x509_crt_parse(&crt,
                                     reinterpret_cast<const unsigned char *>(this->cert_pem_.c_str()),
                                     this->cert_pem_.size() + 1);
    if (ret == 0 && crt.subject_key_id.len == 20) {
      char ski_buf[41];
      for (size_t i = 0; i < 20; i++) {
        snprintf(&ski_buf[i * 2], 3, "%02x", crt.subject_key_id.p[i]);
      }
      ski_buf[40] = '\0';
      this->ski_ = ski_buf;
      store.write_string("eebus_ski", this->ski_);
    } else {
      mbedtls_x509_crt_free(&crt);
      this->cert_pem_.clear();
      this->key_pem_.clear();
      this->ski_.clear();
      return false;
    }
    mbedtls_x509_crt_free(&crt);
  }
  for (auto &c : this->ski_) {
    c = std::tolower(static_cast<unsigned char>(c));
  }
  return true;
}


}  // namespace eebus
}  // namespace esphome

