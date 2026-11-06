#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <new>
#include <string>

static bool callback_running = false;
static unsigned callback_allocations = 0;
void *operator new(std::size_t size)
{
    if (callback_running)
        callback_allocations++;
    if (void *p = std::malloc(size))
        return p;
    throw std::bad_alloc();
}
void operator delete(void *p) noexcept
{
    std::free(p);
}
void operator delete(void *p, std::size_t) noexcept
{
    std::free(p);
}
#define ESP_LOGD(...) ((void)0)
#define ESP_LOGI(...) ((void)0)
#define ESP_LOGW(...) ((void)0)
#define ESP_LOGE(...) ((void)0)
template <class T> T &main_loop_id(T &value)
{
    assert(!callback_running);
    return value;
}
#define id(x) main_loop_id(x)
using portMUX_TYPE = std::mutex;
#define portMUX_INITIALIZER_UNLOCKED                                                                         \
    {                                                                                                        \
    }
#define portENTER_CRITICAL(m) (m)->lock()
#define portEXIT_CRITICAL(m) (m)->unlock()
static constexpr int BLE_OWN_ADDR_PUBLIC = 0, BLE_HS_FOREVER = -1, BLE_GAP_EVENT_DISC = 1;
static constexpr uint8_t BLE_HCI_ADV_RPT_EVTYPE_SCAN_RSP = 4;
static constexpr unsigned MALLOC_CAP_INTERNAL = 1, MALLOC_CAP_8BIT = 2, MALLOC_CAP_SPIRAM = 4;
struct ble_addr_t
{
    uint8_t type;
    uint8_t val[6];
};
struct ble_gap_disc_params
{
    int passive, itvl, window, filter_duplicates, filter_policy;
};
struct ble_gap_disc_desc
{
    ble_addr_t addr;
    const uint8_t *data;
    uint8_t length_data;
    uint8_t event_type;
};
struct ble_gap_event
{
    int type;
    ble_gap_disc_desc disc;
};
static struct
{
    void (*sync_cb)();
    void (*reset_cb)(int);
} ble_hs_cfg;
static struct
{
    std::string state;
} ble_device_mac;
static struct
{
    bool state = false;
} ble_enabled_switch;
static float ble_temp_value = NAN, ble_hum_value = NAN;
static uint32_t clock_ms = 0;
static uint32_t millis()
{
    return clock_ms;
}
static unsigned free_heap = 65536, largest_block = 32768;
static bool initialized = false, active = false;
static unsigned scan_starts = 0;
static int allowlist_result = 0, scan_result = 0;
static unsigned init_calls = 0, deinit_calls = 0;
static int init_result = 0, stop_result = 0, deinit_result = 0;
static int task_result = 1, host_task_result = 1;
static void (*host_entry)(void *) = nullptr;
static void (*stop_entry)(void *) = nullptr;
static constexpr int pdPASS = 1, NIMBLE_HS_STACK_SIZE = 4096, configMAX_PRIORITIES = 25, NIMBLE_CORE = 0;
static int xTaskCreate(void (*entry)(void *), const char *, int, void *, int, void *)
{
    if (task_result == pdPASS)
        stop_entry = entry;
    return task_result;
}
static int xTaskCreatePinnedToCore(void (*entry)(void *), const char *, int, void *, int, void *, int)
{
    if (host_task_result == pdPASS)
        host_entry = entry;
    return host_task_result;
}
static void vTaskDelete(void *) {}
#define pdMS_TO_TICKS(ms) (ms)
static bool shutdown_wait = false, finish_stop_on_delay = false;
static void vTaskDelay(int ticks)
{
    assert(shutdown_wait && "host must exit before deinit");
    clock_ms += ticks;
    if (finish_stop_on_delay && stop_entry)
    {
        auto entry = stop_entry;
        stop_entry = nullptr;
        entry(nullptr);
    }
}
static int nimble_port_init()
{
    init_calls++;
    initialized = init_result == 0;
    return init_result;
}
static void nimble_port_run() {}
static int nimble_port_stop()
{
    assert(initialized && !active);
    if (stop_result == 0)
        host_entry(nullptr);
    return stop_result;
}
static int nimble_port_deinit()
{
    deinit_calls++;
    initialized = false;
    return deinit_result;
}
static unsigned heap_caps_get_free_size(unsigned)
{
    return free_heap;
}
static unsigned heap_caps_get_largest_free_block(unsigned)
{
    return largest_block;
}
static unsigned heap_caps_get_minimum_free_size(unsigned)
{
    return free_heap;
}
static ble_gap_disc_params scan_params;
static ble_addr_t allowed_addr;
static int (*scan_callback)(ble_gap_event *, void *);
static bool seen_address = false;
static int scan_duration;
static int ble_gap_disc_active()
{
    return active;
}
static int ble_gap_disc_cancel()
{
    active = false;
    return 0;
}
static int ble_gap_wl_set(const ble_addr_t *addr, int count)
{
    assert(count == 1);
    allowed_addr = *addr;
    return allowlist_result;
}
static int ble_gap_disc(int, int duration, ble_gap_disc_params *params,
                        int (*callback)(ble_gap_event *, void *), void *)
{
    assert(duration == BLE_HS_FOREVER || (duration > 0 && duration <= 60000));
    scan_params = *params;
    scan_duration = duration;
    scan_callback = callback;
    active = scan_result == 0;
    seen_address = false;
    scan_starts++;
    return scan_result;
}
#include "nimble_switchbot_scanner.h"
static ble_addr_t target = {1, {0x01, 0, 0, 0, 0, 0x02}};
static void report(ble_addr_t addr, const uint8_t *payload, uint8_t length, uint8_t event_type = 0)
{
    assert(active);
    if (scan_params.passive && event_type == BLE_HCI_ADV_RPT_EVTYPE_SCAN_RSP)
        return;
    if (scan_params.filter_policy == 1 &&
        (addr.type != allowed_addr.type || memcmp(addr.val, allowed_addr.val, 6)))
        return;
    if (scan_params.filter_duplicates && seen_address)
        return;
    seen_address = true;
    ble_gap_event event{BLE_GAP_EVENT_DISC, {addr, payload, length, event_type}};
    callback_running = true;
    scan_callback(&event, nullptr);
    callback_running = false;
}
static void advertise(ble_addr_t addr, uint8_t decimal, uint8_t integer, uint8_t humidity,
                      uint8_t event_type = 0)
{
    uint8_t payload[] = {14, 0xFF, 0x69, 0x09, 0, 0, 0, 0, 0, 0, 0, 0, decimal, integer, humidity};
    report(addr, payload, sizeof(payload), event_type);
}
static void meter_reading(ble_addr_t addr, uint8_t event_type = BLE_HCI_ADV_RPT_EVTYPE_SCAN_RSP)
{
    uint8_t payload[] = {9, 0x16, 0x3D, 0xFD, 0x69, 0, 100, 3, 0x80 | 21, 57};
    report(addr, payload, sizeof(payload), event_type);
}
static void expect(float temperature, float humidity)
{
    assert(!std::isnan(ble_temp_value));
    assert(fabs(ble_temp_value - temperature) < 0.01f);
    assert(ble_hum_value == humidity);
}
static void identify(ble_addr_t address, uint8_t model)
{
    uint8_t payload[] = {6, 0x16, 0x3D, 0xFD, model, 0, 100};
    report(address, payload, sizeof(payload), BLE_HCI_ADV_RPT_EVTYPE_SCAN_RSP);
}
static void test_discovery()
{
    nimble_begin_discovery();
    nimble_update(true);
    assert(nimble_is_discovering() && nimble_discovery_seconds() == 60);
    assert(scan_params.passive == 0 && scan_params.filter_duplicates == 0 && scan_params.filter_policy == 0);
    assert(scan_params.window == scan_params.itvl);
    identify(target, 0x35);
    // Unsupported products and generic SwitchBot manufacturer data do not
    // appear in the list. The scan response must identify a thermometer.
    auto unrelated = target;
    unrelated.val[1]++;
    identify(unrelated, 0x48);
    advertise(unrelated, 0, 0x80 | 19, 50);
    NimbleDevice snapshot[NIMBLE_DISCOVERY_LIMIT];
    nimble_discovery_snapshot(snapshot);
    for (auto &device : snapshot)
        assert(!device.model);
    // Real active-scan order: temperature ADV first, identity SCAN_RSP next.
    // A single advertisement must be selectable; waiting for another can
    // exceed the 60-second discovery window for a quiet sensor.
    advertise(target, 0, 0x80 | 19, 57);
    nimble_discovery_snapshot(snapshot);
    for (auto &device : snapshot)
        assert(!device.model);
    identify(target, 0x77);
    nimble_discovery_snapshot(snapshot);
    unsigned identified = 0;
    for (auto &device : snapshot)
    {
        if (device.model)
        {
            identified++;
            assert(device.reading.valid && device.reading.humidity == 57);
        }
    }
    assert(identified == 1);
    // Unidentified packets cannot exhaust slots or evict an identified sensor.
    for (unsigned i = 0; i < 20; i++)
    {
        auto address = unrelated;
        address.val[0] += i;
        advertise(address, 0, 0x80 | 18, 50);
    }
    nimble_discovery_snapshot(snapshot);
    identified = 0;
    for (auto &device : snapshot)
        if (device.model)
            identified++;
    assert(identified == 1);
    nimble_begin_discovery();
    nimble_update(true);
    for (unsigned i = 0; i < 20; i++)
    {
        auto address = target;
        address.val[0] += i;
        identify(address, 0x77);
        advertise(address, i % 10, 0x80 | 20, 60);
    }
    nimble_discovery_snapshot(snapshot);
    for (unsigned i = 0; i < NIMBLE_DISCOVERY_LIMIT; i++)
    {
        assert(snapshot[i].model == 0x77);
        assert(snapshot[i].address.val[0] == target.val[0] + i);
        assert(snapshot[i].reading.valid && snapshot[i].reading.humidity == 60);
    }
    for (unsigned i = 0; i < 100000; i++)
        advertise(target, i % 10, 0x80 | 21, 57);
    nimble_update(true);
    expect(21.9f, 57);
    assert(callback_allocations == 0);
    // Deadline is rollover-safe and resumes tracking the saved target.
    clock_ms += 60000;
    nimble_update(true);
    assert(!nimble_is_discovering() && scan_params.filter_policy == 1);
    clock_ms = UINT32_MAX - 10000;
    nimble_begin_discovery();
    nimble_update(true);
    clock_ms += 60000;
    assert(nimble_discovery_seconds() == 0);
    identify(unrelated, 0x77); // callback arriving after the deadline is ignored
    nimble_discovery_snapshot(snapshot);
    for (auto &device : snapshot)
        assert(!device.model);
    nimble_update(true);
    assert(!nimble_is_discovering());
    // Empty MAC still permits discovery, but cannot create room readings.
    ble_device_mac.state = "";
    nimble_begin_discovery();
    nimble_update(true);
    identify(target, 0x77);
    advertise(target, 0, 0x80 | 19, 55);
    nimble_update(true);
    assert(!nimble_reading_fresh());
    nimble_end_discovery();
    nimble_update(true);
    assert(!active);
    ble_device_mac.state = "02:00:00:00:00:01";
    nimble_update(true);
    advertise(target, 0, 0x80 | 20, 60);
    nimble_update(true);
}

static void test_scan_modes()
{
    // A Meter Plus ADV followed by a temperature-bearing SCAN_RSP during
    // discovery must still lead to passive tracking, regardless of order.
    for (bool response_first : {false, true})
    {
        nimble_begin_discovery();
        nimble_update(true);
        if (response_first)
            meter_reading(target);
        advertise(target, 3, 0x80 | 21, 57);
        if (!response_first)
            meter_reading(target);
        nimble_update(true);
        clock_ms += 60000;
        active = false; // controller ends the finite discovery scan
        nimble_update(true);
        assert(scan_params.passive && scan_params.filter_policy == 1);
        // Even service data needs no active scan when it is in an ADV.
        for (unsigned i = 0; i < 4; i++)
        {
            clock_ms += 60000;
            meter_reading(target, 0);
            nimble_update(true);
            expect(21.3f, 57);
            assert(scan_params.passive);
        }
    }

    // Start the retry window near millis rollover; no early active scan.
    clock_ms = UINT32_MAX - 10000;
    meter_reading(target, 0);
    nimble_update(true);
    clock_ms += 99999;
    nimble_update(true);
    assert(scan_params.passive);
    clock_ms++;
    nimble_update(true);
    assert(!scan_params.passive && scan_params.filter_policy == 1 && scan_duration == 30000);

    // One response is insufficient, and fallback does not extend freshness.
    meter_reading(target);
    nimble_update(true);
    clock_ms += 29999;
    nimble_update(true);
    assert(!scan_params.passive);
    clock_ms++;
    active = false;
    nimble_update(true);
    assert(scan_params.passive);
    assert(nimble_reading.received_ms == clock_ms - 30000);

    // Repeated responses after a failed passive trial support active-only
    // sensors. Learning an address must not extend the 30-second probe.
    ble_device_mac.state = "";
    nimble_update(true);
    ble_device_mac.state = "02:00:00:00:00:01";
    nimble_update(true);
    assert(scan_params.passive && scan_params.filter_policy == 0);
    clock_ms += 120000;
    nimble_update(true);
    assert(!scan_params.passive && scan_duration == 30000);
    for (unsigned i = 0; i < 2; i++)
    {
        clock_ms += 1000;
        meter_reading(target);
        nimble_update(true);
    }
    clock_ms += 28000;
    allowlist_result = 5;
    active = false;
    nimble_update(true);
    assert(!scan_params.passive && scan_duration == BLE_HS_FOREVER && scan_params.filter_policy == 0);
    clock_ms += 119999;
    nimble_update(true);
    assert(scan_params.passive); // allowlist retries cannot keep stale active tracking alive
    allowlist_result = 0;
    clock_ms += 120000;
    nimble_update(true);
    // Packet type, not manufacturer versus service data, controls fallback.
    advertise(target, 3, 0x80 | 21, 57, BLE_HCI_ADV_RPT_EVTYPE_SCAN_RSP);
    meter_reading(target);
    clock_ms += 30000;
    nimble_update(true);
    assert(!scan_params.passive && scan_duration == BLE_HS_FOREVER && scan_params.filter_policy == 1);
    for (unsigned i = 0; i < 4; i++)
    {
        clock_ms += 60000;
        meter_reading(target);
        nimble_update(true);
        expect(21.3f, 57);
        assert(!scan_params.passive);
    }

    // An ADV cannot be hidden by a later response before the main loop runs.
    advertise(target, 3, 0x80 | 21, 57);
    meter_reading(target);
    nimble_update(true);
    assert(scan_params.passive);

    // A failed scan start cannot leave the fallback stuck in active mode.
    clock_ms += 120000;
    scan_result = 5;
    nimble_update(true);
    assert(!active);
    scan_result = 0;
    nimble_update(true);
    assert(active && scan_params.passive);

    // Probe expiry also survives millis rollover. Responses from another
    // address cannot establish a need for active tracking.
    clock_ms = UINT32_MAX - 130000;
    advertise(target, 3, 0x80 | 21, 57);
    nimble_update(true);
    clock_ms += 120000;
    nimble_update(true);
    assert(!scan_params.passive);
    auto other = target;
    other.val[0]++;
    meter_reading(other);
    meter_reading(other);
    clock_ms += 30000;
    active = false;
    nimble_update(true);
    assert(scan_params.passive);

    // No responses: the probe is bounded and the reading expires normally.
    advertise(target, 3, 0x80 | 21, 57);
    nimble_update(true);
    clock_ms += 120000;
    nimble_update(true);
    assert(!scan_params.passive);
    clock_ms += 30000;
    active = false;
    nimble_update(true);
    assert(scan_params.passive);
    clock_ms += 150000;
    nimble_update(true);
    assert(!nimble_reading_fresh() && std::isnan(ble_temp_value));

    // Changing the selected sensor discards the old active fallback mode.
    ble_device_mac.state = "02:00:00:00:00:02";
    nimble_update(true);
    assert(!scan_params.passive && scan_duration == 30000 && std::isnan(ble_temp_value));
    ble_device_mac.state = "02:00:00:00:00:01";
    nimble_update(true);
    advertise(target, 3, 0x80 | 21, 57);
    nimble_update(true);
    assert(scan_params.passive && callback_allocations == 0);

    clock_ms += 120000;
    nimble_update(true);
    meter_reading(target);
    meter_reading(target);
    clock_ms += 30000;
    nimble_update(true);
    assert(!scan_params.passive && scan_duration == BLE_HS_FOREVER);
    // Loss of an active-only sensor returns to passive; it does not keep
    // transmitting indefinitely or refresh the old reading.
    clock_ms += 120000;
    nimble_update(true);
    assert(scan_params.passive);
    expect(21.3f, 57);

    // Explicit discovery also ends with a fresh passive trial, even when
    // the selected sensor was previously confirmed to need active scans.
    clock_ms += 120000;
    nimble_update(true);
    meter_reading(target);
    meter_reading(target);
    clock_ms += 30000;
    nimble_update(true);
    assert(!scan_params.passive && scan_duration == BLE_HS_FOREVER);
    nimble_begin_discovery();
    nimble_update(true);
    meter_reading(target);
    clock_ms += 60000;
    nimble_update(true);
    assert(scan_params.passive);
}

static void select_unidentified_target()
{
    ble_device_mac.state = "";
    nimble_update(true);
    nimble_begin_discovery();
    nimble_end_discovery();
    ble_device_mac.state = "02:00:00:00:00:01";
    nimble_update(true);
}

static void test_target_identity()
{
    select_unidentified_target();
    assert(!scan_params.passive && scan_duration == 30000);
    identify(target, 0x48);
    advertise(target, 3, 0x80 | 21, 57);
    nimble_update(true);
    assert(!nimble_reading_fresh() && std::isnan(ble_temp_value));
    // Manufacturer data alone cannot validate a manually configured MAC.
    for (unsigned i = 0; i < 3; i++)
    {
        advertise(target, 4, 0x80 | 22, 50);
        nimble_update(true);
        assert(!nimble_reading_fresh());
    }
    clock_ms += 30000;
    active = false;
    nimble_update(true);
    assert(scan_params.passive); // unidentified devices do not keep scanning active

    // A separate identity packet can authorize a preceding advertisement,
    // without changing the age of that reading, including over rollover.
    clock_ms = UINT32_MAX - 2000;
    select_unidentified_target();
    advertise(target, 3, 0x80 | 21, 57);
    nimble_update(true);
    assert(!nimble_reading_fresh());
    clock_ms += 5000;
    identify(target, 0x69);
    nimble_update(true);
    expect(21.3f, 57);
    assert(uint32_t(clock_ms - nimble_reading.received_ms) == 5000);
    nimble_end_discovery();
    nimble_update(true);
    assert(scan_params.passive);
    advertise(target, 4, 0x80 | 22, 50);
    nimble_update(true);
    expect(22.4f, 50);

    // A known identity from another address must not authorize this target.
    select_unidentified_target();
    auto other = target;
    other.val[0]++;
    identify(other, 0x69);
    advertise(target, 3, 0x80 | 21, 57);
    nimble_update(true);
    assert(!nimble_reading_fresh());
    identify(target, 0x69);
    nimble_update(true);
    expect(21.3f, 57);
    ble_device_mac.state = "02:00:00:00:00:02";
    nimble_update(true);
    advertise(other, 4, 0x80 | 22, 50);
    nimble_update(true);
    assert(!nimble_reading_fresh());

    // Expired provisional data cannot become fresh when identity arrives.
    select_unidentified_target();
    nimble_reading_timeout_ms = 60000;
    advertise(target, 3, 0x80 | 21, 57);
    clock_ms += 60000;
    nimble_update(true);
    clock_ms += 20000;
    nimble_update(true);
    assert(!scan_params.passive);
    identify(target, 0x69);
    nimble_update(true);
    assert(scan_params.passive && !nimble_reading_fresh());
    advertise(target, 3, 0x80 | 21, 57);
    nimble_update(true);
    expect(21.3f, 57);
    // A contradictory, unsupported identity revokes the old reading too.
    nimble_begin_discovery();
    nimble_update(true);
    identify(target, 0x48);
    advertise(target, 4, 0x80 | 22, 50);
    nimble_update(true);
    assert(!nimble_reading_fresh());
    nimble_end_discovery();
    nimble_reading_timeout_ms = NIMBLE_READING_TIMEOUT_MS;
}

static void test_recovery_before_expiry()
{
    for (uint32_t timeout : {60000U, 120000U, 300000U, 900000U})
    {
        nimble_reading_timeout_ms = timeout;
        clock_ms = UINT32_MAX - 10000;
        select_unidentified_target();
        meter_reading(target);
        nimble_update(true);
        assert(scan_params.passive && nimble_reading_fresh());
        uint32_t retry = timeout / 3;
        if (retry > 120000)
            retry = 120000;
        clock_ms += retry - 1;
        nimble_update(true);
        assert(scan_params.passive && nimble_reading_fresh());
        clock_ms++;
        nimble_update(true);
        assert(!scan_params.passive && scan_duration == 30000 && nimble_reading_fresh());
        for (unsigned i = 0; i < 2; i++)
        {
            clock_ms += 2000;
            meter_reading(target);
            nimble_update(true);
            assert(nimble_reading_fresh());
        }
        clock_ms += 26000;
        active = false;
        nimble_update(true);
        assert(!scan_params.passive && scan_duration == BLE_HS_FOREVER && nimble_reading_fresh());
        // A response-only sensor keeps working through several short timeout
        // periods. Switching modes never substitutes for a real new reading.
        for (unsigned i = 0; i < 6; i++)
        {
            clock_ms += 30000;
            meter_reading(target);
            nimble_update(true);
            assert(nimble_reading_fresh() && !scan_params.passive);
        }
        advertise(target, 3, 0x80 | 21, 57);
        nimble_update(true);
        assert(scan_params.passive);
        clock_ms += timeout;
        nimble_update(true);
        assert(!nimble_reading_fresh() && std::isnan(ble_temp_value));
    }

    nimble_reading_timeout_ms = 60000;
    select_unidentified_target();
    meter_reading(target);
    nimble_update(true);
    clock_ms += 24000;
    nimble_begin_discovery();
    nimble_update(true);
    nimble_end_discovery();
    nimble_update(true);
    assert(scan_params.passive);
    clock_ms += 2000;
    nimble_update(true);
    assert(!scan_params.passive && nimble_reading_fresh());
    // Recent scan restarts cannot postpone recovery past the reading deadline.
    nimble_reading_timeout_ms = NIMBLE_READING_TIMEOUT_MS;
}

int main()
{
    // Disabled boot does not allocate the stack. Startup callbacks cannot start
    // it before the other components have finished setting up.
    ble_enabled_switch.state = true;
    nimble_update(id(ble_enabled_switch).state);
    assert(init_calls == 0);
    ble_enabled_switch.state = false;
    nimble_boot_ready = true;
    nimble_update(id(ble_enabled_switch).state);
    assert(init_calls == 0);

    // Refuse controller startup when internal memory is already exhausted/fragmented.
    free_heap = 40000;
    nimble_init();
    assert(!initialized);
    free_heap = 65536;
    largest_block = 8000;
    nimble_init();
    assert(!initialized);
    largest_block = 32768;
    nimble_init();
    assert(initialized);
    ble_enabled_switch.state = true;
    nimble_on_sync_cb();
    ble_device_mac.state = "02:00:00:00:00:01";
    nimble_update(id(ble_enabled_switch).state);
    identify(target, 0x77);
    advertise(target, 4, 0x80 | 21, 57);
    assert(std::isnan(ble_temp_value)); // host cannot publish ESPHome globals
    nimble_update(id(ble_enabled_switch).state);
    expect(21.4f, 57);
    assert(scan_params.filter_policy == 1);
    advertise(target, 4, 0x80 | 21, 57);
    clock_ms = 60000;
    advertise(target, 2, 0x80 | 19, 63);
    nimble_update(id(ble_enabled_switch).state);
    expect(19.2f, 63);
    auto other = target;
    other.val[0]++;
    advertise(other, 0, 0x80 | 30, 40);
    nimble_update(id(ble_enabled_switch).state);
    expect(19.2f, 63);
    for (unsigned i = 0; i < 100000; i++)
    {
        clock_ms += 1000;
        advertise(target, i % 10, 0x80 | 20, 60);
        nimble_update(id(ble_enabled_switch).state);
    }
    assert(callback_allocations == 0);
    expect(20.9f, 60);
    auto starts = scan_starts;
    clock_ms += 120000;
    nimble_update(id(ble_enabled_switch).state);
    assert(scan_starts > starts); // running scan with no data recovers
    clock_ms += 60000;
    nimble_update(id(ble_enabled_switch).state);
    expect(20.9f, 60); // hold the last real reading beyond three minutes
    clock_ms += 119999;
    nimble_update(id(ble_enabled_switch).state);
    expect(20.9f, 60);
    clock_ms++;
    nimble_update(id(ble_enabled_switch).state);
    assert(std::isnan(ble_temp_value)); // expires at exactly five minutes
    nimble_reading_timeout_ms = 900000;
    nimble_update(id(ble_enabled_switch).state);
    assert(std::isnan(ble_temp_value)); // extending timeout cannot revive expired data
    nimble_reading_timeout_ms = 60000;
    advertise(target, 3, 4, 64);
    nimble_update(id(ble_enabled_switch).state);
    expect(-4.3f, 64);
    clock_ms += 60000;
    nimble_update(id(ble_enabled_switch).state);
    assert(std::isnan(ble_temp_value));
    nimble_reading_timeout_ms = NIMBLE_READING_TIMEOUT_MS;
    advertise(target, 3, 4, 64);
    nimble_update(id(ble_enabled_switch).state);
    expect(-4.3f, 64);

    clock_ms = UINT32_MAX - 10000;
    advertise(target, 0, 0x80 | 20, 60);
    nimble_update(id(ble_enabled_switch).state);
    clock_ms += 20000;
    nimble_update(id(ble_enabled_switch).state);
    expect(20, 60); // unsigned timer rollover
    clock_ms += NIMBLE_READING_TIMEOUT_MS;
    nimble_update(id(ble_enabled_switch).state);
    assert(!nimble_reading_fresh());
    clock_ms = UINT32_MAX - 10000;
    nimble_update(id(ble_enabled_switch).state);
    assert(!nimble_reading_fresh());
    // Cached values cannot reappear a full timer wrap after expiry.

    ble_device_mac.state = "invalid";
    nimble_update(id(ble_enabled_switch).state);
    assert(!active && std::isnan(ble_temp_value));
    ble_device_mac.state = "02:00:00:00:00:02";
    nimble_update(id(ble_enabled_switch).state);
    assert(std::isnan(ble_temp_value) && !nimble_tracking);
    identify(other, 0x69);
    advertise(other, 1, 0x80 | 22, 61);
    nimble_update(id(ble_enabled_switch).state);
    expect(22.1f, 61);
    nimble_on_reset_cb(1);
    active = false;
    nimble_update(id(ble_enabled_switch).state);
    assert(!active);
    nimble_on_sync_cb();
    nimble_update(id(ble_enabled_switch).state);
    assert(active);
    ble_device_mac.state = "";
    nimble_update(id(ble_enabled_switch).state);
    assert(!active && std::isnan(ble_temp_value));

    ble_device_mac.state = "02:00:00:00:00:01";
    nimble_update(id(ble_enabled_switch).state);
    identify(target, 0x77);
    advertise(target, 0, 0x80 | 20, 60);
    nimble_update(id(ble_enabled_switch).state);
    test_discovery();
    test_scan_modes();
    test_target_identity();
    test_recovery_before_expiry();
    for (unsigned cycle = 0; cycle < 100; cycle++)
    {
        nimble_begin_discovery();
        nimble_update(true);
        auto calls = init_calls;
        ble_enabled_switch.state = false;
        nimble_update(id(ble_enabled_switch).state);
        assert(nimble_state.load() == NimbleState::STOPPING);
        assert(!active && !nimble_ready && !nimble_reading_fresh());
        assert(!nimble_is_discovering());
        assert(std::isnan(ble_temp_value) && std::isnan(ble_hum_value));
        // Late host callbacks must not resurrect a reading while stopping.
        uint8_t payload[] = {14, 0xFF, 0x69, 0x09, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x80 | 30, 50};
        ble_gap_event late{BLE_GAP_EVENT_DISC, {target, payload, sizeof(payload)}};
        nimble_gap_event_cb(&late, nullptr);
        assert(!nimble_pending_reading.valid);
        // Rapid OFF -> ON is deferred until asynchronous cleanup has finished.
        ble_enabled_switch.state = true;
        nimble_update(id(ble_enabled_switch).state);
        assert(init_calls == calls);
        stop_entry(nullptr);
        assert(!initialized && nimble_state.load() == NimbleState::OFF);
        assert(deinit_calls == cycle + 1);
        nimble_update(id(ble_enabled_switch).state);
        assert(initialized && init_calls == calls + 1);
        assert(!active && !nimble_reading_fresh());
        nimble_on_sync_cb();
        nimble_update(id(ble_enabled_switch).state);
        identify(target, 0x77);
        advertise(target, 3, 0x80 | 21, 58);
        nimble_update(id(ble_enabled_switch).state);
        expect(21.3f, 58);
    }

    // Reboot must finish the worker before WiFi/IDF shutdown, without saving Tr.
    shutdown_wait = finish_stop_on_delay = true;
    auto previous_mac = ble_device_mac.state;
    nimble_shutdown();
    assert(nimble_state.load() == NimbleState::OFF && !initialized && !active);
    assert(!nimble_boot_ready && ble_enabled_switch.state && ble_device_mac.state == previous_mac);
    auto stopped_calls = init_calls;
    nimble_update(true);
    nimble_shutdown(); // Already off; no double stop or delayed reinitialization.
    assert(init_calls == stopped_calls);

    // A stalled worker cannot hang the shutdown hook, including millis rollover.
    nimble_boot_ready = true;
    nimble_update(true);
    nimble_on_sync_cb();
    nimble_update(true);
    finish_stop_on_delay = false;
    clock_ms = UINT32_MAX - 1000;
    auto shutdown_started = clock_ms;
    nimble_shutdown();
    assert(uint32_t(clock_ms - shutdown_started) == 5000);
    assert(nimble_state.load() == NimbleState::STOPPING);
    stop_entry(nullptr);
    assert(nimble_state.load() == NimbleState::OFF);
    shutdown_wait = false;
    nimble_boot_ready = true;
    nimble_update(true);
    nimble_on_sync_cb();
    nimble_update(true);

    // A failed temporary-task allocation is retryable, with no double init.
    ble_enabled_switch.state = false;
    task_result = 0;
    nimble_update(id(ble_enabled_switch).state);
    assert(nimble_state.load() == NimbleState::RUNNING && nimble_ready);
    task_result = pdPASS;
    nimble_update(id(ble_enabled_switch).state);
    stop_entry(nullptr);
    assert(nimble_state.load() == NimbleState::OFF);
    nimble_update(id(ble_enabled_switch).state);
    assert(!initialized); // stays unloaded while disabled

    // Never re-enter a partially failed stack until reboot.
    ble_enabled_switch.state = true;
    host_task_result = 0;
    nimble_update(id(ble_enabled_switch).state);
    assert(!initialized && nimble_state.load() == NimbleState::FAILED);
    auto calls = init_calls;
    nimble_update(id(ble_enabled_switch).state);
    assert(init_calls == calls);
    host_task_result = pdPASS;
    nimble_state.store(NimbleState::OFF);
    nimble_update(id(ble_enabled_switch).state);
    nimble_on_sync_cb();
    nimble_update(id(ble_enabled_switch).state);
    ble_enabled_switch.state = false;
    nimble_update(id(ble_enabled_switch).state);
    stop_result = 5;
    auto deinits = deinit_calls;
    stop_entry(nullptr);
    assert(deinit_calls == deinits && nimble_state.load() == NimbleState::FAILED);
    ble_enabled_switch.state = true;
    calls = init_calls;
    nimble_update(id(ble_enabled_switch).state);
    assert(init_calls == calls);

    // Bounds checks for every payload length, malformed AD length, and short manufacturer record.
    uint8_t garbage[255] = {};
    for (unsigned len = 0; len <= 255; len++)
    {
        for (unsigned ad_len = 0; ad_len <= 255; ad_len++)
        {
            garbage[0] = ad_len;
            garbage[1] = 0xFF;
            garbage[2] = 0x69;
            garbage[3] = 0x09;
            NimbleReading reading;
            nimble_parse_switchbot_data(garbage, len, reading);
        }
    }
    uint8_t short_payload[] = {3, 0xFF, 0x69, 0x09};
    NimbleReading reading;
    assert(!nimble_parse_switchbot_data(short_payload, sizeof(short_payload), reading));
    uint8_t meter[] = {9, 0x16, 0x3D, 0xFD, 0x54, 0, 100, 3, 0x80 | 21, 57};
    NimbleReading meter_reading;
    assert(nimble_parse_switchbot_data(meter, sizeof(meter), meter_reading));
    assert(fabs(meter_reading.temperature - 21.3f) < 0.01f && meter_reading.humidity == 57);
    for (unsigned length = 0; length < sizeof(meter); length++)
    {
        NimbleReading truncated;
        assert(!nimble_parse_switchbot_data(meter, length, truncated));
    }
    meter[4] = 0x48;
    assert(!nimble_parse_switchbot_data(meter, sizeof(meter), meter_reading));
    puts("PASS: parsing, passive tracking, bounded active fallback, allowlist, allocation-free callbacks, "
         "stale recovery, rollover, MAC changes, "
         "bounded discovery, reset, low-heap guard, malformed packets, 100 stop/start cycles, deferred "
         "restart, lifecycle failures");
}
