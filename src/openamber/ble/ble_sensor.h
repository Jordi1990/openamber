#pragma once

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <type_traits>
// The host simulator (USE_HOST) has no Bluetooth stack: the sensor logic below
// runs as on the device, fed by mock/ble_sensor_mock.h instead of a scanner.
#ifndef USE_HOST
#include "sdkconfig.h"
#include "esp_heap_caps.h"
// Set when the Bluetooth controller is on a co-processor (esp32_hosted, as on
// the ESP32-P4): this chip then has no controller to start.
#ifndef CONFIG_BT_CONTROLLER_DISABLED
#include "esp_bt.h"
#endif
#endif

// BLE thermometers via ESPHome's esp32_ble / esp32_ble_tracker: pairing
// slots, the search list, the BLE stack and the scan mode. Nothing here knows
// a sensor brand or advertisement layout: that is in ble_sensor_formats.h and
// the ble_format_*.h parsers behind it. A sensor's "model" is the value those
// give it.
// ESPHome delivers advertisements in its main loop, so all state below is
// only touched from the loop: no locks, tasks or atomics are needed.

static const char *BLE_SENSOR_TAG = "ble_sensor";
// Sensors advertise every few seconds. Readings stay valid for 10 minutes,
// so short gaps (WiFi coexistence, flash erases with the controller in flash)
// never drop the room temperature, while a dead or out-of-range sensor still
// raises B01 within reasonable time.
static const uint32_t BLE_SENSOR_READING_TIMEOUT_MS = 600000;
// Scanning is passive: sensors broadcast their readings in every
// advertisement. Active scanning makes every BLE device in range answer with a
// scan response, which costs the sensors battery, so it only runs while it is
// needed: during discovery, for a paired sensor whose model is not known yet
// (some sensors only identify themselves in the scan response), and for a
// paired sensor that sent nothing for this long (some only report readings
// there). A model that ble_sensor_model_passive() lists never needs it.
static const uint32_t BLE_SENSOR_PASSIVE_GRACE_MS = 120000;
// A paired sensor that needs scan responses gets them in bursts: active for
// BLE_SENSOR_ACTIVE_BURST_MS once every BLE_SENSOR_ACTIVE_PERIOD_MS, starting
// right away. A working sensor answers within the first burst; a dead or
// out-of-range one does not keep every other device in range answering.
static const uint32_t BLE_SENSOR_ACTIVE_BURST_MS = 30000;
static const uint32_t BLE_SENSOR_ACTIVE_PERIOD_MS = 600000;

struct BleSensorReading
{
    float temperature = NAN;
    float humidity = NAN;
    uint32_t received_ms = 0;
    bool valid = false;
};

// Most significant byte first, as printed ("AA:BB:..." -> val[0] == 0xAA).
struct BleSensorAddress
{
    uint8_t val[6] = {};
};

static const unsigned BLE_SENSOR_DISCOVERY_LIMIT = 8;
static const uint32_t BLE_SENSOR_DISCOVERY_MS = 60000;
struct BleSensorDevice
{
    BleSensorAddress address;
    BleSensorReading reading;
    uint8_t model = 0;
};

static bool ble_sensor_boot_ready = false;
static bool ble_sensor_stack_requested = false;
// Up to three paired sensors (room, outdoor, a future zone 2 thermostat). Each
// slot follows one text entity holding its MAC; the room temperature source
// picks one slot or the wired Tr sensor.
static const unsigned BLE_SENSOR_SLOTS = 3;
struct BleSensorSlot
{
    std::string mac;            // MAC currently tracked ("" = empty slot)
    BleSensorAddress address;
    bool valid = false;         // mac parsed
    BleSensorReading reading;
    uint8_t model = 0;          // model byte, 0 until its scan response is seen
    uint32_t tracking_since_ms = 0; // when tracking started (0: BLE off)
    bool timed_out = false;     // no reading for the full timeout; cleared by a fresh reading
    uint32_t active_wanted_since_ms = 0; // when it started needing scan responses (0: it does not)
};
static BleSensorSlot ble_sensor_slots[BLE_SENSOR_SLOTS];
// Slot used as room temperature source, -1 for the wired Tr sensor.
static int ble_sensor_source_slot = -1;
static BleSensorDevice ble_sensor_devices[BLE_SENSOR_DISCOVERY_LIMIT];
static bool ble_sensor_discovering = false;
static uint32_t ble_sensor_discovery_started_ms = 0;

static bool ble_sensor_is_discovering()
{
    return ble_sensor_discovering;
}

static unsigned ble_sensor_discovery_seconds()
{
    uint32_t elapsed = uint32_t(millis() - ble_sensor_discovery_started_ms);
    return ble_sensor_discovering && elapsed < BLE_SENSOR_DISCOVERY_MS
               ? (BLE_SENSOR_DISCOVERY_MS - elapsed + 999) / 1000
               : 0;
}

static void ble_sensor_discovery_snapshot(BleSensorDevice *devices)
{
    memcpy(devices, ble_sensor_devices, sizeof(ble_sensor_devices));
}

static bool ble_sensor_parse_mac(const std::string &mac, BleSensorAddress &addr)
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
        addr.val[i] = value;
    }
    return true;
}

static void ble_sensor_format_mac(const BleSensorAddress &address, char *text)
{
    snprintf(text, 18, "%02X:%02X:%02X:%02X:%02X:%02X", address.val[0], address.val[1], address.val[2],
             address.val[3], address.val[4], address.val[5]);
}

// The model of a paired sensor is stored together with its MAC (MAC << 8 |
// model), so it survives a reboot and is never applied to another sensor.
static uint64_t ble_sensor_model_key(const BleSensorAddress &address, uint8_t model)
{
    uint64_t key = 0;
    for (uint8_t byte : address.val)
        key = (key << 8) | byte;
    return (key << 8) | model;
}

static uint8_t ble_sensor_stored_model(uint64_t key, const BleSensorAddress &address)
{
    uint8_t model = key & 0xFF;
    return (key >> 8) == (ble_sensor_model_key(address, 0) >> 8) && ble_sensor_model_known(model) ? model : 0;
}

// "BTHome (A1B2)": the model name plus the last two MAC bytes, the way
// Home Assistant names a discovered device. Falls back to "Sensor (A1B2)".
static void ble_sensor_format_title(const BleSensorAddress &address, uint8_t model, char *text, size_t size)
{
    snprintf(text, size, "%s (%02X%02X)", ble_sensor_model_name(model), address.val[4], address.val[5]);
}

// Title for narrow widgets: the name part is cut to at most
// max_name characters, preferably at a word boundary, and marked with "...",
// while the short ID always stays visible: "SwitchBot Indoor/Outdoor... (A1B2)".
static void ble_sensor_format_short_title(const BleSensorAddress &address, uint8_t model, char *text, size_t size,
                                         size_t max_name)
{
    std::string name = ble_sensor_model_name(model);
    if (name.size() > max_name)
    {
        size_t cut = max_name;
        size_t boundary = name.find_last_of(" /-", cut);
        if (boundary != std::string::npos && boundary > max_name / 2)
            cut = boundary;
        name = name.substr(0, cut);
        while (!name.empty() && (name.back() == ' ' || name.back() == '/' || name.back() == '-'))
            name.pop_back();
        name += "...";
    }
    snprintf(text, size, "%s (%02X%02X)", name.c_str(), address.val[4], address.val[5]);
}

// During a search, log thermometer-like advertisements that no parser
// accepted (once per device), so an unsupported firmware or format shows up
// in the log with its payload:
// (BLE_SENSOR_KNOWN_SERVICE_UUIDS).
static const unsigned BLE_SENSOR_UNKNOWN_LOG_LIMIT = 16;
static BleSensorAddress ble_sensor_unknown_logged[BLE_SENSOR_UNKNOWN_LOG_LIMIT];
static unsigned ble_sensor_unknown_logged_count = 0;

template <typename Datas> static void ble_sensor_log_unknown(const BleSensorAddress &address, const Datas &services)
{
    for (const auto &service : services)
    {
        using Uuid = typename std::decay<decltype(service.uuid)>::type;
        uint16_t uuid = 0;
        for (uint16_t candidate : BLE_SENSOR_KNOWN_SERVICE_UUIDS)
            if (service.uuid == Uuid::from_uint16(candidate))
                uuid = candidate;
        if (!uuid)
            continue;
        for (unsigned i = 0; i < ble_sensor_unknown_logged_count; i++)
            if (memcmp(ble_sensor_unknown_logged[i].val, address.val, 6) == 0)
                return;
        if (ble_sensor_unknown_logged_count >= BLE_SENSOR_UNKNOWN_LOG_LIMIT)
            return;
        ble_sensor_unknown_logged[ble_sensor_unknown_logged_count++] = address;
        char mac[18], hex[2 * 31 + 1] = "";
        ble_sensor_format_mac(address, mac);
        for (size_t i = 0; i < service.data.size() && i < 31; i++)
            snprintf(hex + 2 * i, 3, "%02X", service.data[i]);
        ESP_LOGI(BLE_SENSOR_TAG, "Not supported: %s service 0x%04X data %s", mac, uuid, hex);
        return;
    }
}

static void ble_sensor_remember_device(const BleSensorAddress &address, uint8_t model,
                                      const BleSensorReading &reading)
{
    int slot = -1, empty = -1;
    uint32_t oldest = 0;
    for (unsigned i = 0; i < BLE_SENSOR_DISCOVERY_LIMIT; i++)
    {
        auto &device = ble_sensor_devices[i];
        if ((device.model || device.reading.valid) && memcmp(device.address.val, address.val, 6) == 0)
            slot = i;
        // Keep provisional readings and sensors heard in this search, but
        // reclaim unidentified entries and identified sensors without a reading
        // (kept from an earlier search) first.
        uint32_t age = device.reading.valid ? uint32_t(millis() - device.reading.received_ms) : UINT32_MAX;
        if ((!device.model || !device.reading.valid) && (empty < 0 || age > oldest))
        {
            empty = i;
            oldest = age;
        }
    }
    if (slot < 0 && (model || reading.valid))
    {
        slot = empty;
        if (slot >= 0)
            ble_sensor_devices[slot] = {};
    }
    if (slot < 0)
        return;
    ble_sensor_devices[slot].address = address;
    if (model)
        ble_sensor_devices[slot].model = model;
    if (reading.valid)
        ble_sensor_devices[slot].reading = reading;
}

// Whether an advertisement from this address matters: a paired sensor, or any
// device during a search (*discover).
static bool ble_sensor_wanted(const BleSensorAddress &address, bool *discover)
{
    bool paired = false;
    for (const auto &slot : ble_sensor_slots)
        paired |= slot.valid && memcmp(address.val, slot.address.val, 6) == 0;
    *discover = ble_sensor_discovering &&
                uint32_t(millis() - ble_sensor_discovery_started_ms) < BLE_SENSOR_DISCOVERY_MS;
    return paired || *discover;
}

// Stores a parsed advertisement in the search list and the paired slots.
static void ble_sensor_apply(const BleSensorAddress &address, uint8_t model, const BleSensorReading &reading,
                             bool discover)
{
    if (discover)
        ble_sensor_remember_device(address, model, reading);
    // The same MAC may be paired in more than one slot; all of them follow it.
    for (auto &slot : ble_sensor_slots)
    {
        if (!slot.valid || memcmp(address.val, slot.address.val, 6) != 0)
            continue;
        if (model)
            slot.model = model;
        if (reading.valid)
            slot.reading = reading;
    }
}

// Called from esp32_ble_tracker's on_ble_advertise with x.address() (MSB first).
template <typename Datas>
static void ble_sensor_on_advertise(const uint8_t *mac, const Datas &manufacturers, const Datas &services)
{
    BleSensorAddress address;
    memcpy(address.val, mac, 6);
    bool discover;
    if (!ble_sensor_wanted(address, &discover))
        return;

    BleSensorReading reading;
    uint8_t model =
        ble_sensor_parse(mac, manufacturers, services, reading.temperature, reading.humidity, reading.valid);
    if (reading.valid)
        reading.received_ms = millis();
    if (discover && !model && !reading.valid)
        ble_sensor_log_unknown(address, services);
    ble_sensor_apply(address, model, reading, discover);
}

static bool ble_sensor_slot_fresh(unsigned slot)
{
    if (slot >= BLE_SENSOR_SLOTS)
        return false;
    // Only an identified thermometer counts: a reading without a model may
    // come from another kind of device at a paired MAC.
    const BleSensorReading &r = ble_sensor_slots[slot].reading;
    return ble_sensor_model_known(ble_sensor_slots[slot].model) && r.valid &&
           uint32_t(millis() - r.received_ms) < BLE_SENSOR_READING_TIMEOUT_MS;
}

// Reading of the room temperature source slot (an empty one for Tr).
static const BleSensorReading &ble_sensor_source_reading()
{
    static const BleSensorReading none;
    return ble_sensor_source_slot >= 0 ? ble_sensor_slots[ble_sensor_source_slot].reading : none;
}

static bool ble_sensor_reading_fresh()
{
    return ble_sensor_source_slot >= 0 && ble_sensor_slot_fresh(ble_sensor_source_slot);
}

// True once a BLE slot is the source and it has supplied no reading for the
// full timeout: the last reading expired, or none arrived since tracking
// started (boot, Bluetooth switched on, or another sensor paired). Drives B01.
// The state is latched in ble_sensor_update(), so a millis() rollover during a
// long outage cannot clear it.
static bool ble_sensor_reading_missing()
{
    if (!ble_sensor_boot_ready || ble_sensor_source_slot < 0 || ble_sensor_reading_fresh())
        return false;
    return ble_sensor_slots[ble_sensor_source_slot].timed_out;
}

static void ble_sensor_end_discovery()
{
    ble_sensor_discovering = false;
}

// A new search only lists sensors heard during it, but keeps the models already
// learned: the model byte is only in the scan response, which can take tens of
// seconds to catch, while a reading arrives with every advertisement.
static void ble_sensor_begin_discovery()
{
    for (auto &device : ble_sensor_devices)
    {
        if (device.model)
            device.reading = {};
        else
            device = {};
    }
    ble_sensor_unknown_logged_count = 0;
    ble_sensor_discovery_started_ms = millis();
    ble_sensor_discovering = true;
}

// Set when BLE is selected but the controller cannot be started until reboot.
static bool ble_sensor_restart_required = false;
// Set when ESPHome's BLE host (Bluedroid) failed to initialise; esp32_ble then
// stays failed until the next boot.
static bool ble_sensor_host_failed = false;

#if defined(USE_HOST) || defined(CONFIG_BT_CONTROLLER_DISABLED)
// The co-processor runs the controller; ESPHome's esp32_ble brings it up over
// esp32_hosted when it is enabled. The host simulator has no controller.
static bool ble_sensor_controller_start()
{
    return true;
}
#else
// The S3 controller allocates its exchange memory (0x7800 bytes with one BLE
// activity) as one internal DMA-capable block, from its own task. If that
// fails it asserts (emi.c 164) instead of returning an error, so only start it
// when such a block exists. After WiFi and the UI are up the internal heap is
// usually too fragmented; ble_sensor_early_start() covers the boot case.
static const size_t BLE_SENSOR_CONTROLLER_BLOCK = 0x7800 + 2048;

static bool ble_sensor_controller_fits()
{
    return heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA) >= BLE_SENSOR_CONTROLLER_BLOCK;
}

static bool ble_sensor_controller_start()
{
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE)
    {
        if (!ble_sensor_controller_fits())
            return false;
        esp_bt_controller_config_t cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
        esp_err_t err = esp_bt_controller_init(&cfg);
        if (err != ESP_OK)
        {
            ESP_LOGE(BLE_SENSOR_TAG, "esp_bt_controller_init failed: %s", esp_err_to_name(err));
            return false;
        }
        while (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE)
            vTaskDelay(1);
    }
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_INITED)
    {
        esp_err_t err = esp_bt_controller_enable(ESP_BT_MODE_BLE);
        if (err != ESP_OK)
        {
            ESP_LOGE(BLE_SENSOR_TAG, "esp_bt_controller_enable failed: %s", esp_err_to_name(err));
            return false;
        }
    }
    return esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED;
}
#endif

// Bluedroid is only brought up while the Bluetooth switch is on, so without
// BLE the internal heap keeps the memory the stack would otherwise use.
// ESPHome's esp32_ble skips controller init when it is already enabled.
#ifdef USE_HOST
static bool ble_sensor_stack_enable()
{
    return true;
}
static void ble_sensor_stack_disable() {}
static bool ble_sensor_stack_failed()
{
    return false;
}
#else
static bool ble_sensor_stack_enable()
{
    if (esphome::esp32_ble::global_ble == nullptr || !ble_sensor_controller_start())
        return false;
    esphome::esp32_ble::global_ble->enable();
    return true;
}
static void ble_sensor_stack_disable()
{
    if (esphome::esp32_ble::global_ble != nullptr)
        esphome::esp32_ble::global_ble->disable();
}
static bool ble_sensor_stack_failed()
{
    return esphome::esp32_ble::global_ble != nullptr && esphome::esp32_ble::global_ble->is_failed();
}
#endif

static void ble_sensor_set_ble(bool enabled)
{
    if (!enabled)
        ble_sensor_restart_required = false;
    if (ble_sensor_stack_requested && !ble_sensor_host_failed && ble_sensor_stack_failed())
    {
        ESP_LOGE(BLE_SENSOR_TAG, "BLE host failed to start; restart required");
        ble_sensor_host_failed = true;
    }
    if (enabled == ble_sensor_stack_requested)
        return;
    if (enabled)
    {
        if (!ble_sensor_stack_enable())
        {
            // Retried on every update; memory may become available again.
            if (!ble_sensor_restart_required)
                ESP_LOGW(BLE_SENSOR_TAG, "Not starting BLE: no contiguous internal block for the controller; "
                                        "restart to start BLE during boot");
            ble_sensor_restart_required = true;
            return;
        }
        ble_sensor_restart_required = false;
        ble_sensor_stack_requested = true;
        ESP_LOGI(BLE_SENSOR_TAG, "BLE enabled");
    }
    else
    {
        ble_sensor_stack_requested = false;
        ble_sensor_stack_disable();
        ESP_LOGI(BLE_SENSOR_TAG, "BLE disabled");
    }
}

// Runs from on_boot between esp32_ble's setup and WiFi's, while the internal
// heap still has a large free block for the controller.
static void ble_sensor_early_start(bool enabled)
{
    if (!enabled)
        return;
    ble_sensor_set_ble(true);
}

static void ble_sensor_publish()
{
    bool fresh = ble_sensor_reading_fresh();
    const BleSensorReading &r = ble_sensor_source_reading();
    id(ble_temp_value) = fresh ? r.temperature : NAN;
    id(ble_hum_value) = fresh ? r.humidity : NAN;
}

static uint32_t ble_sensor_now_nonzero()
{
    return millis() ? millis() : 1;
}

// True while a paired sensor needs scan responses (its model is unknown, or it
// has sent nothing for BLE_SENSOR_PASSIVE_GRACE_MS) and its burst is running.
static bool ble_sensor_slot_needs_active_scan(BleSensorSlot &slot)
{
    bool wanted = false;
    if (slot.valid && !ble_sensor_model_passive(slot.model))
    {
        uint32_t since = slot.reading.received_ms ? slot.reading.received_ms : slot.tracking_since_ms;
        wanted = !ble_sensor_model_known(slot.model) || slot.timed_out ||
                 uint32_t(millis() - since) >= BLE_SENSOR_PASSIVE_GRACE_MS;
    }
    if (!wanted)
    {
        slot.active_wanted_since_ms = 0;
        return false;
    }
    if (slot.active_wanted_since_ms == 0)
        slot.active_wanted_since_ms = ble_sensor_now_nonzero();
    return uint32_t(millis() - slot.active_wanted_since_ms) % BLE_SENSOR_ACTIVE_PERIOD_MS < BLE_SENSOR_ACTIVE_BURST_MS;
}

// Switch the tracker between passive and active scanning, the way ESPHome's
// bluetooth_proxy does: the scan restarts with the new mode once it stopped.
static void ble_sensor_set_scan_active(bool active)
{
#ifdef USE_HOST
    (void) active;
#else
    auto *tracker = esphome::esp32_ble_tracker::global_esp32_ble_tracker;
    if (tracker == nullptr || tracker->get_scan_active() == active)
        return;
    ESP_LOGD(BLE_SENSOR_TAG, "Scanning %s", active ? "active" : "passive");
    tracker->set_scan_active(active);
    tracker->stop_scan();
    tracker->set_scan_continuous(true);
#endif
}

// `enabled` is the Bluetooth switch, `source` the slot used as room temperature
// source (-1: Tr), `macs` the three slot MACs and `models` their stored models
// (ble_sensor_model_key), updated once a model is learned. Runs every couple of
// seconds.
static void ble_sensor_update(bool enabled, int source, const std::string *const macs[BLE_SENSOR_SLOTS],
                             uint64_t *const models[BLE_SENSOR_SLOTS])
{
    if (!ble_sensor_boot_ready)
        return;
    if (!enabled ||
        (ble_sensor_discovering && uint32_t(millis() - ble_sensor_discovery_started_ms) >= BLE_SENSOR_DISCOVERY_MS))
        ble_sensor_end_discovery();
    ble_sensor_set_ble(enabled);
    ble_sensor_source_slot = enabled && source >= 0 && source < int(BLE_SENSOR_SLOTS) ? source : -1;
    for (unsigned i = 0; i < BLE_SENSOR_SLOTS; i++)
    {
        BleSensorSlot &slot = ble_sensor_slots[i];
        if (!enabled)
        {
            slot = {};
            continue;
        }
        const std::string &mac = *macs[i];
        if (mac != slot.mac || slot.tracking_since_ms == 0)
        {
            slot = {};
            slot.mac = mac;
            slot.valid = ble_sensor_parse_mac(mac, slot.address);
            // The stored model, or the discovery list for a sensor paired from it.
            if (slot.valid)
                slot.model = ble_sensor_stored_model(*models[i], slot.address);
            for (const auto &device : ble_sensor_devices)
                if (slot.valid && !slot.model && device.model && memcmp(device.address.val, slot.address.val, 6) == 0)
                    slot.model = device.model;
            slot.tracking_since_ms = ble_sensor_now_nonzero();
            if (!slot.valid && !mac.empty())
                ESP_LOGW(BLE_SENSOR_TAG, "Invalid sensor MAC in slot %u", i + 1);
        }
        if (slot.reading.valid && uint32_t(millis() - slot.reading.received_ms) >= BLE_SENSOR_READING_TIMEOUT_MS)
            slot.reading.valid = false;
        if (ble_sensor_slot_fresh(i))
        {
            slot.timed_out = false;
        }
        else if (!slot.timed_out)
        {
            bool identified = ble_sensor_model_known(slot.model) && slot.reading.received_ms;
            uint32_t since = identified ? slot.reading.received_ms : slot.tracking_since_ms;
            slot.timed_out = uint32_t(millis() - since) >= BLE_SENSOR_READING_TIMEOUT_MS;
        }
        if (slot.valid && ble_sensor_model_known(slot.model) &&
            *models[i] != ble_sensor_model_key(slot.address, slot.model))
            *models[i] = ble_sensor_model_key(slot.address, slot.model);
    }
    if (enabled)
    {
        bool active = ble_sensor_discovering;
        // No short-circuit: every slot keeps its own burst timing.
        for (auto &slot : ble_sensor_slots)
            active |= ble_sensor_slot_needs_active_scan(slot);
        ble_sensor_set_scan_active(active);
    }
    ble_sensor_publish();
}

// Slot holding this address, or -1.
static int ble_sensor_slot_of(const BleSensorAddress &address)
{
    for (unsigned i = 0; i < BLE_SENSOR_SLOTS; i++)
        if (ble_sensor_slots[i].valid && memcmp(ble_sensor_slots[i].address.val, address.val, 6) == 0)
            return i;
    return -1;
}
