#pragma once

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <type_traits>
#include "esp_heap_caps.h"
#include "esp_bt.h"

// SwitchBot thermometers via ESPHome's esp32_ble / esp32_ble_tracker.
// ESPHome delivers advertisements in its main loop, so all state below is
// only touched from the loop: no locks, tasks or atomics are needed.

static const char *SWITCHBOT_TAG = "switchbot_ble";
// A SwitchBot advertises every few seconds. Readings stay valid for 10 minutes,
// so short gaps (WiFi coexistence, flash erases with the controller in flash)
// never drop the room temperature, while a dead or out-of-range sensor still
// raises B01 within reasonable time.
static const uint32_t SWITCHBOT_READING_TIMEOUT_MS = 600000;
// Scanning is passive: Meter, Meter Plus and Outdoor Meter broadcast their
// readings in the manufacturer data of every advertisement. Active scanning
// makes every BLE device in range answer with a scan response, which costs the
// sensors battery, so it only runs while it is needed: during discovery, for a
// paired sensor whose model is not known yet (the model byte is only in the
// scan response), and for a paired sensor that sent nothing for this long
// (older Meter firmware only reports readings in the scan response).
static const uint32_t SWITCHBOT_PASSIVE_GRACE_MS = 120000;

struct SwitchbotReading
{
    float temperature = NAN;
    float humidity = NAN;
    uint32_t received_ms = 0;
    bool valid = false;
};

// Most significant byte first, as printed ("AA:BB:..." -> val[0] == 0xAA).
struct SwitchbotAddress
{
    uint8_t val[6] = {};
};

static const unsigned SWITCHBOT_DISCOVERY_LIMIT = 8;
static const uint32_t SWITCHBOT_DISCOVERY_MS = 60000;
struct SwitchbotDevice
{
    SwitchbotAddress address;
    SwitchbotReading reading;
    uint8_t model = 0;
};

static bool switchbot_boot_ready = false;
static bool switchbot_ble_requested = false;
// Up to three paired sensors (room, outdoor, a future zone 2 thermostat). Each
// slot follows one text entity holding its MAC; the room temperature source
// picks one slot or the wired Tr sensor.
static const unsigned SWITCHBOT_SLOTS = 3;
struct SwitchbotSlot
{
    std::string mac;            // MAC currently tracked ("" = empty slot)
    SwitchbotAddress address;
    bool valid = false;         // mac parsed
    SwitchbotReading reading;
    uint8_t model = 0;          // model byte, 0 until its scan response is seen
    uint32_t tracking_since_ms = 0; // when tracking started (0: BLE off)
    bool timed_out = false;     // no reading for the full timeout; cleared by a fresh reading
};
static SwitchbotSlot switchbot_slots[SWITCHBOT_SLOTS];
// Slot used as room temperature source, -1 for the wired Tr sensor.
static int switchbot_source_slot = -1;
static SwitchbotDevice switchbot_devices[SWITCHBOT_DISCOVERY_LIMIT];
static bool switchbot_discovering = false;
static uint32_t switchbot_discovery_started_ms = 0;

static const uint16_t SWITCHBOT_COMPANY_ID = 0x0969;
static const uint16_t SWITCHBOT_SERVICE_UUID = 0xFD3D;
static const uint16_t SWITCHBOT_LEGACY_SERVICE_UUID = 0x000D;

static bool switchbot_is_discovering()
{
    return switchbot_discovering;
}

static unsigned switchbot_discovery_seconds()
{
    uint32_t elapsed = uint32_t(millis() - switchbot_discovery_started_ms);
    return switchbot_discovering && elapsed < SWITCHBOT_DISCOVERY_MS
               ? (SWITCHBOT_DISCOVERY_MS - elapsed + 999) / 1000
               : 0;
}

static void switchbot_discovery_snapshot(SwitchbotDevice *devices)
{
    memcpy(devices, switchbot_devices, sizeof(switchbot_devices));
}

static bool switchbot_parse_mac(const std::string &mac, SwitchbotAddress &addr)
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

static void switchbot_format_mac(const SwitchbotAddress &address, char *text)
{
    snprintf(text, 18, "%02X:%02X:%02X:%02X:%02X:%02X", address.val[0], address.val[1], address.val[2],
             address.val[3], address.val[4], address.val[5]);
}

static bool switchbot_is_meter_model(uint8_t model)
{
    return model == 0x54 || model == 0x74 || model == 0x69 || model == 0x49 || model == 0x77 || model == 0x57;
}

// The model of a paired sensor is stored together with its MAC (MAC << 8 |
// model), so it survives a reboot and is never applied to another sensor.
static uint64_t switchbot_model_key(const SwitchbotAddress &address, uint8_t model)
{
    uint64_t key = 0;
    for (uint8_t byte : address.val)
        key = (key << 8) | byte;
    return (key << 8) | model;
}

static uint8_t switchbot_stored_model(uint64_t key, const SwitchbotAddress &address)
{
    uint8_t model = key & 0xFF;
    return (key >> 8) == (switchbot_model_key(address, 0) >> 8) && switchbot_is_meter_model(model) ? model : 0;
}

// Product names as pySwitchbot / Home Assistant use them. The high bit of the
// model byte is the encryption flag, so lower/upper case are the same product.
static const char *switchbot_model_name(uint8_t model)
{
    switch (model)
    {
    case 0x54: // 'T'
    case 0x74: // 't'
        return "Meter";
    case 0x69: // 'i'
    case 0x49: // 'I'
        return "Meter Plus";
    case 0x77: // 'w'
    case 0x57: // 'W'
        return "Indoor/Outdoor Thermo-Hygrometer";
    default:
        return nullptr;
    }
}

// "SwitchBot Meter Plus (A1B2)": the model plus the last two MAC bytes, the way
// Home Assistant names a discovered device. Falls back to "SwitchBot (A1B2)".
static void switchbot_format_title(const SwitchbotAddress &address, uint8_t model, char *text, size_t size)
{
    const char *name = switchbot_model_name(model);
    snprintf(text, size, "SwitchBot%s%s (%02X%02X)", name ? " " : "", name ? name : "", address.val[4],
             address.val[5]);
}

// Title for narrow widgets: the name part ("SwitchBot <model>") is cut to at most
// max_name characters, preferably at a word boundary, and marked with "...",
// while the short ID always stays visible: "SwitchBot Indoor/Outdoor... (A1B2)".
static void switchbot_format_short_title(const SwitchbotAddress &address, uint8_t model, char *text, size_t size,
                                         size_t max_name)
{
    const char *model_name = switchbot_model_name(model);
    std::string name = std::string("SwitchBot") + (model_name ? " " : "") + (model_name ? model_name : "");
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

// Decode the shared temperature/humidity layout: decimal nibble, integer with
// sign bit (set = positive), humidity percentage.
static bool switchbot_decode(uint8_t decimal, uint8_t integer, uint8_t humidity, SwitchbotReading &reading)
{
    if ((decimal & 0x0F) > 9 || (humidity & 0x7F) > 100)
        return false;
    reading.temperature = (decimal & 0x0F) * 0.1f + (integer & 0x7F);
    if ((integer & 0x80) == 0)
        reading.temperature = -reading.temperature;
    reading.humidity = humidity & 0x7F;
    reading.received_ms = millis();
    reading.valid = true;
    return true;
}

// `Datas` is ESPHome's std::vector<ServiceData>: each entry has a 16-bit
// company ID or service UUID in `uuid` and the payload after it in `data`.
// SwitchBot's service UUID plus model identify thermometers; the company ID
// alone would also admit unrelated SwitchBot products.
template <typename Datas> static uint8_t switchbot_meter_model(const Datas &services)
{
    for (const auto &service : services)
    {
        using Uuid = typename std::decay<decltype(service.uuid)>::type;
        if (!(service.uuid == Uuid::from_uint16(SWITCHBOT_SERVICE_UUID) ||
              service.uuid == Uuid::from_uint16(SWITCHBOT_LEGACY_SERVICE_UUID)) ||
            service.data.empty())
            continue;
        uint8_t model = service.data[0] & 0x7F;
        if (switchbot_is_meter_model(model))
            return model;
    }
    return 0;
}

template <typename Datas>
static bool switchbot_parse(const Datas &manufacturers, const Datas &services, SwitchbotReading &reading)
{
    for (const auto &mfr : manufacturers)
    {
        using Uuid = typename std::decay<decltype(mfr.uuid)>::type;
        if (mfr.uuid == Uuid::from_uint16(SWITCHBOT_COMPANY_ID) && mfr.data.size() >= 11 &&
            switchbot_decode(mfr.data[8], mfr.data[9], mfr.data[10], reading))
            return true;
    }
    // Meter/Meter Plus also put temperature and humidity in service data.
    for (const auto &service : services)
    {
        using Uuid = typename std::decay<decltype(service.uuid)>::type;
        if (!(service.uuid == Uuid::from_uint16(SWITCHBOT_SERVICE_UUID) ||
              service.uuid == Uuid::from_uint16(SWITCHBOT_LEGACY_SERVICE_UUID)) ||
            service.data.size() < 6)
            continue;
        uint8_t model = service.data[0] & 0x7F;
        if ((model == 0x54 || model == 0x74 || model == 0x69 || model == 0x49) &&
            switchbot_decode(service.data[3], service.data[4], service.data[5], reading))
            return true;
    }
    return false;
}

static void switchbot_remember_device(const SwitchbotAddress &address, uint8_t model,
                                      const SwitchbotReading &reading)
{
    int slot = -1, empty = -1;
    uint32_t oldest = 0;
    for (unsigned i = 0; i < SWITCHBOT_DISCOVERY_LIMIT; i++)
    {
        auto &device = switchbot_devices[i];
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
            switchbot_devices[slot] = {};
    }
    if (slot < 0)
        return;
    switchbot_devices[slot].address = address;
    if (model)
        switchbot_devices[slot].model = model;
    if (reading.valid)
        switchbot_devices[slot].reading = reading;
}

// Called from esp32_ble_tracker's on_ble_advertise with x.address() (MSB first).
template <typename Datas>
static void switchbot_on_advertise(const uint8_t *mac, const Datas &manufacturers, const Datas &services)
{
    SwitchbotAddress address;
    memcpy(address.val, mac, 6);
    bool paired = false;
    for (const auto &slot : switchbot_slots)
        paired |= slot.valid && memcmp(address.val, slot.address.val, 6) == 0;
    bool discover = switchbot_discovering &&
                    uint32_t(millis() - switchbot_discovery_started_ms) < SWITCHBOT_DISCOVERY_MS;
    if (!paired && !discover)
        return;

    SwitchbotReading reading;
    switchbot_parse(manufacturers, services, reading);
    uint8_t model = switchbot_meter_model(services);
    if (discover)
        switchbot_remember_device(address, model, reading);
    // The same MAC may be paired in more than one slot; all of them follow it.
    for (auto &slot : switchbot_slots)
    {
        if (!slot.valid || memcmp(address.val, slot.address.val, 6) != 0)
            continue;
        if (model)
            slot.model = model;
        if (reading.valid)
            slot.reading = reading;
    }
}

static bool switchbot_slot_fresh(unsigned slot)
{
    if (slot >= SWITCHBOT_SLOTS)
        return false;
    // Only an identified thermometer counts: manufacturer data of another
    // SwitchBot product at a paired MAC could otherwise decode as a reading.
    const SwitchbotReading &r = switchbot_slots[slot].reading;
    return switchbot_is_meter_model(switchbot_slots[slot].model) && r.valid &&
           uint32_t(millis() - r.received_ms) < SWITCHBOT_READING_TIMEOUT_MS;
}

// Reading of the room temperature source slot (an empty one for Tr).
static const SwitchbotReading &switchbot_source_reading()
{
    static const SwitchbotReading none;
    return switchbot_source_slot >= 0 ? switchbot_slots[switchbot_source_slot].reading : none;
}

static bool switchbot_reading_fresh()
{
    return switchbot_source_slot >= 0 && switchbot_slot_fresh(switchbot_source_slot);
}

// True once a BLE slot is the source and it has supplied no reading for the
// full timeout: the last reading expired, or none arrived since tracking
// started (boot, Bluetooth switched on, or another sensor paired). Drives B01.
// The state is latched in switchbot_update(), so a millis() rollover during a
// long outage cannot clear it.
static bool switchbot_reading_missing()
{
    if (!switchbot_boot_ready || switchbot_source_slot < 0 || switchbot_reading_fresh())
        return false;
    return switchbot_slots[switchbot_source_slot].timed_out;
}

static void switchbot_end_discovery()
{
    switchbot_discovering = false;
}

// A new search only lists sensors heard during it, but keeps the models already
// learned: the model byte is only in the scan response, which can take tens of
// seconds to catch, while a reading arrives with every advertisement.
static void switchbot_begin_discovery()
{
    for (auto &device : switchbot_devices)
    {
        if (device.model)
            device.reading = {};
        else
            device = {};
    }
    switchbot_discovery_started_ms = millis();
    switchbot_discovering = true;
}

// Set when BLE is selected but the controller cannot be started until reboot.
static bool switchbot_restart_required = false;
// Set when ESPHome's BLE host (Bluedroid) failed to initialise; esp32_ble then
// stays failed until the next boot.
static bool switchbot_host_failed = false;

// The S3 controller allocates its exchange memory (0x7800 bytes with one BLE
// activity) as one internal DMA-capable block, from its own task. If that
// fails it asserts (emi.c 164) instead of returning an error, so only start it
// when such a block exists. After WiFi and the UI are up the internal heap is
// usually too fragmented; switchbot_early_start() covers the boot case.
static const size_t SWITCHBOT_CONTROLLER_BLOCK = 0x7800 + 2048;

static bool switchbot_controller_fits()
{
    return heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA) >= SWITCHBOT_CONTROLLER_BLOCK;
}

static bool switchbot_controller_start()
{
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE)
    {
        if (!switchbot_controller_fits())
            return false;
        esp_bt_controller_config_t cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
        esp_err_t err = esp_bt_controller_init(&cfg);
        if (err != ESP_OK)
        {
            ESP_LOGE(SWITCHBOT_TAG, "esp_bt_controller_init failed: %s", esp_err_to_name(err));
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
            ESP_LOGE(SWITCHBOT_TAG, "esp_bt_controller_enable failed: %s", esp_err_to_name(err));
            return false;
        }
    }
    return esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED;
}

// Bluedroid is only brought up while the Bluetooth switch is on, so without
// BLE the internal heap keeps the memory the stack would otherwise use.
// ESPHome's esp32_ble skips controller init when it is already enabled.
static bool switchbot_ble_enable()
{
    if (esphome::esp32_ble::global_ble == nullptr || !switchbot_controller_start())
        return false;
    esphome::esp32_ble::global_ble->enable();
    return true;
}
static void switchbot_ble_disable()
{
    if (esphome::esp32_ble::global_ble != nullptr)
        esphome::esp32_ble::global_ble->disable();
}

static void switchbot_set_ble(bool enabled)
{
    if (!enabled)
        switchbot_restart_required = false;
    if (switchbot_ble_requested && !switchbot_host_failed && esphome::esp32_ble::global_ble != nullptr &&
        esphome::esp32_ble::global_ble->is_failed())
    {
        ESP_LOGE(SWITCHBOT_TAG, "BLE host failed to start; restart required");
        switchbot_host_failed = true;
    }
    if (enabled == switchbot_ble_requested)
        return;
    if (enabled)
    {
        if (!switchbot_ble_enable())
        {
            // Retried on every update; memory may become available again.
            if (!switchbot_restart_required)
                ESP_LOGW(SWITCHBOT_TAG, "Not starting BLE: no contiguous internal block for the controller; "
                                        "restart to start BLE during boot");
            switchbot_restart_required = true;
            return;
        }
        switchbot_restart_required = false;
        switchbot_ble_requested = true;
        ESP_LOGI(SWITCHBOT_TAG, "BLE enabled");
    }
    else
    {
        switchbot_ble_requested = false;
        switchbot_ble_disable();
        ESP_LOGI(SWITCHBOT_TAG, "BLE disabled");
    }
}

// Runs from on_boot between esp32_ble's setup and WiFi's, while the internal
// heap still has a large free block.
static void switchbot_early_start(bool enabled)
{
    if (!enabled)
        return;
    switchbot_set_ble(true);
}

static void switchbot_publish()
{
    bool fresh = switchbot_reading_fresh();
    const SwitchbotReading &r = switchbot_source_reading();
    id(ble_temp_value) = fresh ? r.temperature : NAN;
    id(ble_hum_value) = fresh ? r.humidity : NAN;
}

static uint32_t switchbot_now_nonzero()
{
    return millis() ? millis() : 1;
}

// True while a paired sensor needs scan responses: its model is unknown, or it
// has sent nothing for SWITCHBOT_PASSIVE_GRACE_MS.
static bool switchbot_slot_needs_active_scan(const SwitchbotSlot &slot)
{
    if (!slot.valid)
        return false;
    if (!switchbot_is_meter_model(slot.model) || slot.timed_out)
        return true;
    uint32_t since = slot.reading.received_ms ? slot.reading.received_ms : slot.tracking_since_ms;
    return uint32_t(millis() - since) >= SWITCHBOT_PASSIVE_GRACE_MS;
}

// Switch the tracker between passive and active scanning, the way ESPHome's
// bluetooth_proxy does: the scan restarts with the new mode once it stopped.
static void switchbot_set_scan_active(bool active)
{
    auto *tracker = esphome::esp32_ble_tracker::global_esp32_ble_tracker;
    if (tracker == nullptr || tracker->get_scan_active() == active)
        return;
    ESP_LOGD(SWITCHBOT_TAG, "Scanning %s", active ? "active" : "passive");
    tracker->set_scan_active(active);
    tracker->stop_scan();
    tracker->set_scan_continuous(true);
}

// `enabled` is the Bluetooth switch, `source` the slot used as room temperature
// source (-1: Tr), `macs` the three slot MACs and `models` their stored models
// (switchbot_model_key), updated once a model is learned. Runs every couple of
// seconds.
static void switchbot_update(bool enabled, int source, const std::string *const macs[SWITCHBOT_SLOTS],
                             uint64_t *const models[SWITCHBOT_SLOTS])
{
    if (!switchbot_boot_ready)
        return;
    if (!enabled ||
        (switchbot_discovering && uint32_t(millis() - switchbot_discovery_started_ms) >= SWITCHBOT_DISCOVERY_MS))
        switchbot_end_discovery();
    switchbot_set_ble(enabled);
    switchbot_source_slot = enabled && source >= 0 && source < int(SWITCHBOT_SLOTS) ? source : -1;
    for (unsigned i = 0; i < SWITCHBOT_SLOTS; i++)
    {
        SwitchbotSlot &slot = switchbot_slots[i];
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
            slot.valid = switchbot_parse_mac(mac, slot.address);
            // The stored model, or the discovery list for a sensor paired from it.
            if (slot.valid)
                slot.model = switchbot_stored_model(*models[i], slot.address);
            for (const auto &device : switchbot_devices)
                if (slot.valid && !slot.model && device.model && memcmp(device.address.val, slot.address.val, 6) == 0)
                    slot.model = device.model;
            slot.tracking_since_ms = switchbot_now_nonzero();
            if (!slot.valid && !mac.empty())
                ESP_LOGW(SWITCHBOT_TAG, "Invalid SwitchBot MAC in slot %u", i + 1);
        }
        if (slot.reading.valid && uint32_t(millis() - slot.reading.received_ms) >= SWITCHBOT_READING_TIMEOUT_MS)
            slot.reading.valid = false;
        if (switchbot_slot_fresh(i))
        {
            slot.timed_out = false;
        }
        else if (!slot.timed_out)
        {
            bool identified = switchbot_is_meter_model(slot.model) && slot.reading.received_ms;
            uint32_t since = identified ? slot.reading.received_ms : slot.tracking_since_ms;
            slot.timed_out = uint32_t(millis() - since) >= SWITCHBOT_READING_TIMEOUT_MS;
        }
        if (slot.valid && switchbot_is_meter_model(slot.model) &&
            *models[i] != switchbot_model_key(slot.address, slot.model))
            *models[i] = switchbot_model_key(slot.address, slot.model);
    }
    if (enabled)
    {
        bool active = switchbot_discovering;
        for (const auto &slot : switchbot_slots)
            active |= switchbot_slot_needs_active_scan(slot);
        switchbot_set_scan_active(active);
    }
    switchbot_publish();
}

// Slot holding this address, or -1.
static int switchbot_slot_of(const SwitchbotAddress &address)
{
    for (unsigned i = 0; i < SWITCHBOT_SLOTS; i++)
        if (switchbot_slots[i].valid && memcmp(switchbot_slots[i].address.val, address.val, 6) == 0)
            return i;
    return -1;
}
