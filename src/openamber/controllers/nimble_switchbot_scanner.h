#pragma once

#include <atomic>
#include <cstring>
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"

static const char *NIMBLE_TAG = "nimble_switchbot";
static const uint32_t NIMBLE_SCAN_RETRY_MS = 120000;
static const uint32_t NIMBLE_ACTIVE_PROBE_MS = 30000;
static const uint32_t NIMBLE_READING_TIMEOUT_MS = 300000;
static uint32_t nimble_reading_timeout_ms = NIMBLE_READING_TIMEOUT_MS;

// Only the main loop manages scans and accesses ESPHome components. The host
// task copies readings into fixed storage; callbacks never allocate memory.
struct NimbleReading
{
    float temperature = NAN;
    float humidity = NAN;
    uint32_t received_ms = 0;
    bool valid = false;
};

static portMUX_TYPE nimble_mux = portMUX_INITIALIZER_UNLOCKED;
static std::atomic<bool> nimble_ready{false};
enum class NimbleState
{
    OFF,
    RUNNING,
    STOPPING,
    FAILED
};
static std::atomic<NimbleState> nimble_state{NimbleState::OFF};
static std::atomic<bool> nimble_host_exited{false};
static bool nimble_boot_ready = false;
static ble_addr_t nimble_expected_addr = {};
static ble_addr_t nimble_found_addr = {};
static bool nimble_target_enabled = false;
static bool nimble_target_found = false;
static uint8_t nimble_target_model = 0;
static NimbleReading nimble_unverified_reading;
static bool nimble_unverified_advertisement = false;
static NimbleReading nimble_pending_reading;
static NimbleReading nimble_reading;
static std::string nimble_tracked_mac;
static bool nimble_tracking = false;
static uint32_t nimble_scan_start_ms = 0;
static bool nimble_scan_started = false;
enum class NimbleScanMode
{
    PASSIVE,
    IDENTIFY,
    ACTIVE_PROBE,
    ACTIVE_TRACKING
};
static NimbleScanMode nimble_scan_mode = NimbleScanMode::PASSIVE;
// Evidence for the current scan, copied under nimble_mux by the main loop.
static bool nimble_advertisement_received = false;
static uint8_t nimble_scan_responses = 0;

static const unsigned NIMBLE_DISCOVERY_LIMIT = 8;
static const uint32_t NIMBLE_DISCOVERY_MS = 60000;
struct NimbleDevice
{
    ble_addr_t address = {};
    NimbleReading reading;
    uint8_t model = 0;
};
static NimbleDevice nimble_devices[NIMBLE_DISCOVERY_LIMIT];
static bool nimble_discovering = false;
static uint32_t nimble_discovery_started_ms = 0;

static bool nimble_is_discovering()
{
    portENTER_CRITICAL(&nimble_mux);
    bool active = nimble_discovering;
    portEXIT_CRITICAL(&nimble_mux);
    return active;
}

static unsigned nimble_discovery_seconds()
{
    uint32_t elapsed = uint32_t(millis() - nimble_discovery_started_ms);
    return nimble_is_discovering() && elapsed < NIMBLE_DISCOVERY_MS
               ? (NIMBLE_DISCOVERY_MS - elapsed + 999) / 1000
               : 0;
}

// SwitchBot's service UUID and model identify supported thermometer packets;
// company ID alone would also admit unrelated SwitchBot products.
static int nimble_switchbot_model(const uint8_t *data, uint8_t len)
{
    for (unsigned pos = 0; pos + 1 < len;)
    {
        unsigned size = data[pos];
        if (size == 0 || pos + size >= len)
            break;
        if (data[pos + 1] == 0x16 && size >= 4 &&
            ((data[pos + 2] == 0x3D && data[pos + 3] == 0xFD) ||
             (data[pos + 2] == 0x0D && data[pos + 3] == 0x00)))
        {
            return data[pos + 4] & 0x7F;
        }
        pos += size + 1;
    }
    return -1;
}

static bool nimble_is_thermometer(int model)
{
    return model == 0x54 || model == 0x74 || model == 0x69 || model == 0x49 || model == 0x77 ||
           model == 0x57;
}

static void nimble_discovery_snapshot(NimbleDevice *devices)
{
    portENTER_CRITICAL(&nimble_mux);
    memcpy(devices, nimble_devices, sizeof(nimble_devices));
    portEXIT_CRITICAL(&nimble_mux);
}

static void nimble_init();
static void nimble_log_heap();

static bool nimble_parse_mac(const std::string &mac, ble_addr_t &addr)
{
    if (mac.size() != 17)
        return false;
    for (int i = 0; i < 6; i++)
    {
        unsigned value = 0;
        for (int j = 0; j < 2; j++)
        {
            char c = mac[i * 3 + j];
            unsigned digit;
            if (c >= '0' && c <= '9')
                digit = c - '0';
            else if (c >= 'a' && c <= 'f')
                digit = c - 'a' + 10;
            else if (c >= 'A' && c <= 'F')
                digit = c - 'A' + 10;
            else
                return false;
            value = value * 16 + digit;
        }
        if (i < 5 && mac[i * 3 + 2] != ':')
            return false;
        addr.val[5 - i] = value;
    }
    return true;
}

static bool nimble_parse_switchbot_data(const uint8_t *data, uint8_t len, NimbleReading &reading)
{
    unsigned pos = 0;
    while (pos + 1 < len)
    {
        unsigned ad_len = data[pos];
        if (ad_len == 0 || pos + ad_len >= len)
            break;
        if (data[pos + 1] == 0xFF && ad_len >= 14)
        {
            const uint8_t *mfr = data + pos + 2;
            if (mfr[0] == 0x69 && mfr[1] == 0x09 && (mfr[10] & 0x0F) <= 9 && (mfr[12] & 0x7F) <= 100)
            {
                reading.temperature = (mfr[10] & 0x0F) * 0.1f + (mfr[11] & 0x7F);
                if ((mfr[11] & 0x80) == 0)
                    reading.temperature = -reading.temperature;
                reading.humidity = mfr[12] & 0x7F;
                reading.received_ms = millis();
                reading.valid = true;
                return true;
            }
        }
        // Meter/Meter Plus also put temperature and humidity in service data.
        if (data[pos + 1] == 0x16 && ad_len >= 9 &&
            ((data[pos + 2] == 0x3D && data[pos + 3] == 0xFD) ||
             (data[pos + 2] == 0x0D && data[pos + 3] == 0x00)))
        {
            const uint8_t *service = data + pos + 4;
            uint8_t model = service[0] & 0x7F;
            if ((model == 0x54 || model == 0x74 || model == 0x69 || model == 0x49) &&
                (service[3] & 0x0F) <= 9 && (service[5] & 0x7F) <= 100)
            {
                reading.temperature = (service[3] & 0x0F) * 0.1f + (service[4] & 0x7F);
                if ((service[4] & 0x80) == 0)
                    reading.temperature = -reading.temperature;
                reading.humidity = service[5] & 0x7F;
                reading.received_ms = millis();
                reading.valid = true;
                return true;
            }
        }
        pos += ad_len + 1;
    }
    return false;
}

static int nimble_gap_event_cb(struct ble_gap_event *event, void *arg)
{
    if (event->type != BLE_GAP_EVENT_DISC)
        return 0;
    // The allowlist applies during tracking; checking here also rejects late
    // advertisements from an old scan after the configured MAC changes.
    portENTER_CRITICAL(&nimble_mux);
    bool matches = nimble_target_enabled && memcmp(event->disc.addr.val, nimble_expected_addr.val, 6) == 0;
    bool discover =
        nimble_discovering && uint32_t(millis() - nimble_discovery_started_ms) < NIMBLE_DISCOVERY_MS;
    portEXIT_CRITICAL(&nimble_mux);
    if (!matches && !discover)
        return 0;

    NimbleReading reading;
    nimble_parse_switchbot_data(event->disc.data, event->disc.length_data, reading);
    int packet_model = nimble_switchbot_model(event->disc.data, event->disc.length_data);
    uint8_t model = nimble_is_thermometer(packet_model) ? packet_model : 0;
    bool advertisement = event->disc.event_type != BLE_HCI_ADV_RPT_EVTYPE_SCAN_RSP;
    portENTER_CRITICAL(&nimble_mux);
    if (discover && nimble_discovering)
    {
        int slot = -1, empty = -1;
        uint32_t oldest = 0;
        for (unsigned i = 0; i < NIMBLE_DISCOVERY_LIMIT; i++)
        {
            auto &device = nimble_devices[i];
            if ((device.model || device.reading.valid) &&
                memcmp(device.address.val, event->disc.addr.val, 6) == 0)
                slot = i;
            // ADV normally precedes SCAN_RSP. Keep its reading provisionally,
            // but always reclaim unidentified slots before identified sensors.
            uint32_t age =
                device.reading.valid ? uint32_t(millis() - device.reading.received_ms) : UINT32_MAX;
            if (!device.model && (empty < 0 || age > oldest))
            {
                empty = i;
                oldest = age;
            }
        }
        if (slot < 0 && (model || reading.valid))
        {
            slot = empty;
            if (slot >= 0)
                nimble_devices[slot] = {};
        }
        if (slot >= 0)
        {
            nimble_devices[slot].address = event->disc.addr;
            if (model)
                nimble_devices[slot].model = model;
            if (reading.valid)
                nimble_devices[slot].reading = reading;
        }
    }
    if (nimble_target_enabled && memcmp(event->disc.addr.val, nimble_expected_addr.val, 6) == 0)
    {
        if (packet_model >= 0)
            nimble_target_model = model;
        if (packet_model >= 0 && !model)
        {
            nimble_unverified_reading = {};
            nimble_pending_reading = {};
            nimble_target_found = false;
        }
        else
        {
            // ADV can precede the identifying SCAN_RSP. Keep that reading
            // private until the model is known; never refresh its timestamp.
            if (reading.valid)
            {
                nimble_unverified_reading = reading;
                nimble_unverified_advertisement = advertisement;
            }
            if (nimble_target_model && nimble_unverified_reading.valid)
            {
                nimble_found_addr = event->disc.addr;
                nimble_target_found = true;
                nimble_pending_reading = nimble_unverified_reading;
                if (nimble_unverified_advertisement)
                    nimble_advertisement_received = true;
                else if (nimble_scan_responses < 2)
                    nimble_scan_responses++;
                nimble_unverified_reading = {};
            }
        }
    }
    portEXIT_CRITICAL(&nimble_mux);
    return 0;
}

static void nimble_stop_scan()
{
    if (nimble_ready.load() && ble_gap_disc_active())
        ble_gap_disc_cancel();
}

static void nimble_end_discovery()
{
    if (!nimble_is_discovering())
        return;
    portENTER_CRITICAL(&nimble_mux);
    nimble_discovering = false;
    portEXIT_CRITICAL(&nimble_mux);
    nimble_stop_scan();
}

static void nimble_begin_discovery()
{
    nimble_stop_scan();
    portENTER_CRITICAL(&nimble_mux);
    for (auto &device : nimble_devices)
        device = {};
    nimble_discovery_started_ms = millis();
    nimble_discovering = true;
    portEXIT_CRITICAL(&nimble_mux);
}

static void nimble_start_scan(bool tracking, const ble_addr_t &target,
                              NimbleScanMode mode = NimbleScanMode::PASSIVE)
{
    nimble_stop_scan();
    if (tracking)
    {
        int rc = ble_gap_wl_set(&target, 1);
        if (rc != 0)
        {
            ESP_LOGW(NIMBLE_TAG, "Allowlist failed: %d; retrying discovery", rc);
            tracking = false;
        }
    }
    struct ble_gap_disc_params params = {};
    params.passive = mode == NimbleScanMode::PASSIVE;
    params.itvl = 160;
    params.window = params.itvl; // Wi-Fi coexistence already limits the available scan time.
    // Address-based duplicate filtering suppresses changed readings forever.
    params.filter_duplicates = 0;
    params.filter_policy = tracking ? 1 : 0;
    int duration = mode == NimbleScanMode::ACTIVE_PROBE || mode == NimbleScanMode::IDENTIFY
                       ? NIMBLE_ACTIVE_PROBE_MS : BLE_HS_FOREVER;
    if (nimble_is_discovering())
    {
        // SCAN_RSP identifies the model. Keep selected-sensor updates flowing
        // during discovery, then return to the single-address allowlist.
        params.passive = 0;
        params.filter_duplicates = 0;
        params.filter_policy = 0;
        uint32_t elapsed = uint32_t(millis() - nimble_discovery_started_ms);
        if (elapsed >= NIMBLE_DISCOVERY_MS)
            return;
        duration = NIMBLE_DISCOVERY_MS - elapsed;
    }
    portENTER_CRITICAL(&nimble_mux);
    nimble_advertisement_received = false;
    nimble_scan_responses = 0;
    portEXIT_CRITICAL(&nimble_mux);
    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, duration, &params, nimble_gap_event_cb, nullptr);
    nimble_scan_start_ms = millis();
    nimble_scan_started = true;
    nimble_tracking = tracking;
    nimble_scan_mode = mode;
    if (rc != 0)
        ESP_LOGW(NIMBLE_TAG, "Scan start failed: %d", rc);
    else
        ESP_LOGI(NIMBLE_TAG, "%s %s (%s)", tracking ? "Tracking" : "Discovering",
                 nimble_tracked_mac.c_str(), params.passive ? "passive" : "active");
}

static bool nimble_reading_fresh()
{
    return nimble_reading.valid &&
           uint32_t(millis() - nimble_reading.received_ms) < nimble_reading_timeout_ms;
}

static void nimble_clear_reading()
{
    portENTER_CRITICAL(&nimble_mux);
    nimble_target_enabled = false;
    nimble_target_found = false;
    nimble_target_model = 0;
    nimble_unverified_reading = {};
    nimble_pending_reading = {};
    nimble_advertisement_received = false;
    nimble_scan_responses = 0;
    portEXIT_CRITICAL(&nimble_mux);
    nimble_reading = {};
    id(ble_temp_value) = NAN;
    id(ble_hum_value) = NAN;
    nimble_tracked_mac.clear();
    nimble_tracking = false;
    nimble_scan_started = false;
    nimble_scan_mode = NimbleScanMode::PASSIVE;
}

static void nimble_stop_task(void *param)
{
    // nimble_port_stop waits for the host task. Keep that wait off the
    // ESPHome loop so Modbus, thermostat and OTA remain responsive.
    int rc = nimble_port_stop();
    if (rc == 0)
    {
        while (!nimble_host_exited.load())
            vTaskDelay(1);
        rc = nimble_port_deinit();
    }
    nimble_ready.store(false);
    if (rc == 0)
        ESP_LOGI(NIMBLE_TAG, "BLE stopped; host/controller allocations released");
    else
        ESP_LOGE(NIMBLE_TAG, "BLE shutdown failed: %d; reboot required", rc);
    nimble_log_heap();
    // Do not call esp_bt_mem_release: BLE must be restartable without reboot.
    nimble_state.store(rc == 0 ? NimbleState::OFF : NimbleState::FAILED);
    vTaskDelete(nullptr);
}

static void nimble_update(bool enabled)
{
    if (!nimble_boot_ready)
        return;
    if (!enabled ||
        (nimble_is_discovering() && uint32_t(millis() - nimble_discovery_started_ms) >= NIMBLE_DISCOVERY_MS))
        nimble_end_discovery();
    if (!enabled)
    {
        if (nimble_state.load() == NimbleState::RUNNING)
        {
            nimble_stop_scan();
            bool was_ready = nimble_ready.exchange(false);
            nimble_clear_reading();
            nimble_state.store(NimbleState::STOPPING);
            if (xTaskCreate(nimble_stop_task, "nimble_stop", 4096, nullptr, 1, nullptr) != pdPASS)
            {
                nimble_state.store(NimbleState::RUNNING);
                nimble_ready.store(was_ready);
                ESP_LOGE(NIMBLE_TAG, "Cannot allocate BLE shutdown task; retrying");
            }
        }
        return;
    }
    if (nimble_state.load() == NimbleState::OFF)
        nimble_init();
    if (nimble_state.load() != NimbleState::RUNNING)
        return;
    const std::string &mac = id(ble_device_mac).state;
    if (mac != nimble_tracked_mac)
    {
        nimble_stop_scan();
        ble_addr_t target = {};
        bool enabled = nimble_parse_mac(mac, target);
        portENTER_CRITICAL(&nimble_mux);
        nimble_expected_addr = target;
        nimble_target_enabled = enabled;
        nimble_target_found = false;
        nimble_target_model = 0;
        nimble_unverified_reading = {};
        nimble_advertisement_received = false;
        nimble_scan_responses = 0;
        for (auto &device : nimble_devices)
            if (enabled && device.model && memcmp(device.address.val, target.val, 6) == 0)
                nimble_target_model = device.model;
        nimble_pending_reading = {};
        portEXIT_CRITICAL(&nimble_mux);
        nimble_reading = {};
        id(ble_temp_value) = NAN;
        id(ble_hum_value) = NAN;
        nimble_tracking = false;
        nimble_scan_started = false;
        nimble_scan_mode = NimbleScanMode::IDENTIFY;
        nimble_tracked_mac = mac;
        if (!enabled && !mac.empty())
            ESP_LOGW(NIMBLE_TAG, "Invalid SwitchBot MAC");
    }

    portENTER_CRITICAL(&nimble_mux);
    bool target_enabled = nimble_target_enabled;
    bool found = nimble_target_found;
    bool identified = nimble_target_model != 0;
    ble_addr_t target = nimble_found_addr;
    if (nimble_pending_reading.valid &&
        uint32_t(millis() - nimble_pending_reading.received_ms) >= nimble_reading_timeout_ms)
        nimble_pending_reading.valid = false;
    nimble_reading = nimble_pending_reading;
    if (nimble_unverified_reading.valid &&
        uint32_t(millis() - nimble_unverified_reading.received_ms) >= nimble_reading_timeout_ms)
        nimble_unverified_reading.valid = false;
    bool advertisement_received = nimble_advertisement_received;
    uint8_t scan_responses = nimble_scan_responses;
    portEXIT_CRITICAL(&nimble_mux);
    id(ble_temp_value) = nimble_reading_fresh() ? nimble_reading.temperature : NAN;
    id(ble_hum_value) = nimble_reading_fresh() ? nimble_reading.humidity : NAN;
    if (!nimble_ready.load())
        return;
    if (nimble_is_discovering())
    {
        if (!ble_gap_disc_active())
            nimble_start_scan(false, target);
        return;
    }
    if (!target_enabled)
        return;

    uint32_t now = millis();
    uint32_t retry_ms = nimble_reading_timeout_ms / 3;
    if (retry_ms > NIMBLE_SCAN_RETRY_MS)
        retry_ms = NIMBLE_SCAN_RETRY_MS;
    uint32_t reading_age = uint32_t(now - nimble_reading.received_ms);
    // Reserve a full probe plus main-loop scheduling margin before expiry,
    // even if discovery or an allowlist change just restarted the scan.
    bool expiring = nimble_reading.valid &&
                    nimble_reading_timeout_ms - reading_age <= NIMBLE_ACTIVE_PROBE_MS + 5000;
    bool no_readings = !nimble_reading.valid || reading_age >= retry_ms;
    if (nimble_scan_mode == NimbleScanMode::IDENTIFY &&
        (identified || (nimble_scan_started && (!ble_gap_disc_active() ||
                        uint32_t(now - nimble_scan_start_ms) >= NIMBLE_ACTIVE_PROBE_MS))))
    {
        nimble_start_scan(found, target);
    }
    else if (nimble_scan_mode != NimbleScanMode::PASSIVE && advertisement_received)
    {
        nimble_start_scan(found, target);
    }
    else if (nimble_scan_mode == NimbleScanMode::ACTIVE_PROBE &&
             (uint32_t(now - nimble_scan_start_ms) >= NIMBLE_ACTIVE_PROBE_MS || !ble_gap_disc_active()))
    {
        // One response during discovery is not evidence that active scanning
        // is necessary. Require repeated responses after the passive trial.
        nimble_start_scan(found, target, scan_responses >= 2 ? NimbleScanMode::ACTIVE_TRACKING
                                                          : NimbleScanMode::PASSIVE);
    }
    else if (nimble_scan_mode == NimbleScanMode::PASSIVE && no_readings &&
             (uint32_t(now - nimble_scan_start_ms) >= retry_ms || expiring))
    {
        nimble_start_scan(found, target, identified ? NimbleScanMode::ACTIVE_PROBE
                                                  : NimbleScanMode::IDENTIFY);
    }
    else if (nimble_scan_mode == NimbleScanMode::ACTIVE_TRACKING &&
             (!nimble_reading.valid || reading_age >= NIMBLE_SCAN_RETRY_MS))
    {
        nimble_start_scan(found, target);
    }
    else if (!ble_gap_disc_active() ||
             (found && !nimble_tracking && nimble_scan_mode != NimbleScanMode::ACTIVE_PROBE))
    {
        nimble_start_scan(found, target, nimble_scan_mode);
    }
}

static void nimble_shutdown()
{
    // Stop BLE before esp_restart() shuts down the shared WiFi/Bluetooth radio.
    nimble_update(false);
    nimble_boot_ready = false;
    uint32_t started = millis();
    while (nimble_state.load() == NimbleState::STOPPING && uint32_t(millis() - started) < 5000)
        vTaskDelay(pdMS_TO_TICKS(10));
    if (nimble_state.load() != NimbleState::OFF)
        ESP_LOGW(NIMBLE_TAG, "BLE did not finish stopping before reboot");
}

static void nimble_log_heap()
{
    unsigned caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
    unsigned free = heap_caps_get_free_size(caps);
    unsigned minimum = heap_caps_get_minimum_free_size(caps);
    unsigned largest = heap_caps_get_largest_free_block(caps);
    ESP_LOGI(NIMBLE_TAG, "Internal heap: free=%u minimum=%u largest=%u; PSRAM free=%u", free, minimum,
             largest, unsigned(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)));
    if (free < 16384 || largest < 8192)
        ESP_LOGW(NIMBLE_TAG, "Low internal heap headroom");
}

static void nimble_host_task(void *param)
{
    nimble_port_run();
    nimble_host_exited.store(true);
    vTaskDelete(nullptr);
}

static void nimble_on_sync_cb()
{
    nimble_ready.store(true);
}
static void nimble_on_reset_cb(int reason)
{
    nimble_ready.store(false);
}

static void nimble_init()
{
    if (nimble_state.load() != NimbleState::OFF)
        return;
    nimble_log_heap();
    // Keep a reserve before starting the controller/host. This is a startup
    // guard, not proof of runtime headroom: observe the diagnostics on hardware.
    unsigned caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
    if (heap_caps_get_free_size(caps) < 49152 || heap_caps_get_largest_free_block(caps) < 16384)
    {
        ESP_LOGE(NIMBLE_TAG, "Not starting BLE: insufficient internal heap");
        return;
    }
    nimble_ready.store(false);
    int rc = nimble_port_init();
    if (rc != 0)
    {
        ESP_LOGE(NIMBLE_TAG, "NimBLE init failed: %d", rc);
        nimble_state.store(NimbleState::FAILED);
        return;
    }
    ble_hs_cfg.sync_cb = nimble_on_sync_cb;
    ble_hs_cfg.reset_cb = nimble_on_reset_cb;
    nimble_host_exited.store(false);
    // Own the task lifecycle: check allocation failure and let the stop task
    // wait until the host has returned before deinitializing its queues.
    if (xTaskCreatePinnedToCore(nimble_host_task, "nimble_host", NIMBLE_HS_STACK_SIZE, nullptr,
                                configMAX_PRIORITIES - 4, nullptr, NIMBLE_CORE) != pdPASS)
    {
        nimble_port_deinit();
        nimble_state.store(NimbleState::FAILED);
        ESP_LOGE(NIMBLE_TAG, "Cannot allocate BLE host task; reboot required");
        return;
    }
    nimble_state.store(NimbleState::RUNNING);
    ESP_LOGI(NIMBLE_TAG, "BLE started");
    nimble_log_heap();
}
