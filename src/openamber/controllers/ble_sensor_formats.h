#pragma once

#include <cmath>
#include <cstdint>

// The one place that knows every supported thermometer format; ble_sensor.h
// only uses the functions below. To add a format: write its parser in a
// ble_format_<name>.h (included before this file in ble_package.yaml), give it
// a model value here and add it to each function.
//
// A model value identifies the kind of sensor and is stored with a paired
// sensor's MAC, so the values must stay as they are. SwitchBot uses its own
// 7-bit model bytes; the other formats get values above 0x7F.
static const uint8_t BLE_SENSOR_MODEL_BTHOME = 0x80;
static const uint8_t BLE_SENSOR_MODEL_ATC = 0x81;

// A supported thermometer (0 and other values: not identified).
static bool ble_sensor_model_known(uint8_t model)
{
    return switchbot_is_meter_model(model) || model == BLE_SENSOR_MODEL_BTHOME || model == BLE_SENSOR_MODEL_ATC;
}

// Passive models identify themselves and report in every advertisement. The
// others (SwitchBot, and a sensor that is not identified yet) may need scan
// responses, so ble_sensor.h scans actively for them when they stay silent.
static bool ble_sensor_model_passive(uint8_t model)
{
    return model == BLE_SENSOR_MODEL_BTHOME || model == BLE_SENSOR_MODEL_ATC;
}

static const char *ble_sensor_model_name(uint8_t model)
{
    if (model == BLE_SENSOR_MODEL_BTHOME)
        return "BTHome";
    if (model == BLE_SENSOR_MODEL_ATC)
        return "Xiaomi ATC";
    const char *name = switchbot_model_name(model);
    return name ? name : "Sensor";
}

// Service UUIDs of thermometer formats, supported or not. An advertisement
// with one of these that no parser accepts is logged during a search
// (0xFE95 is Xiaomi MiBeacon).
static const uint16_t BLE_SENSOR_KNOWN_SERVICE_UUIDS[] = {BTHOME_SERVICE_UUID, BTHOME_V1_SERVICE_UUID,
                                                          BTHOME_V1_ENCRYPTED_SERVICE_UUID, ATC_SERVICE_UUID, 0xFE95};

static bool ble_sensor_values_plausible(float temperature, float humidity)
{
    return temperature >= -50.0f && temperature <= 100.0f &&
           (std::isnan(humidity) || (humidity >= 0.0f && humidity <= 100.0f));
}

// Run the parsers on one advertisement. Returns the model when the
// advertisement identifies one (0 otherwise) and sets `has_reading` when
// temperature and humidity were filled (humidity NAN when the sensor has
// none). A SwitchBot gives these separately: readings in its advertisement,
// the model in its scan response. `mac` is the advertiser address, most
// significant byte first; `Datas` is ESPHome's std::vector<ServiceData>.
template <typename Datas>
static uint8_t ble_sensor_parse(const uint8_t *mac, const Datas &manufacturers, const Datas &services,
                                float &temperature, float &humidity, bool &has_reading)
{
    uint8_t model = switchbot_meter_model(services);
    has_reading = switchbot_parse(manufacturers, services, temperature, humidity);
    if (!model && !has_reading)
    {
        if ((has_reading = bthome_parse(services, temperature, humidity)))
            model = BLE_SENSOR_MODEL_BTHOME;
        else if ((has_reading = atc_parse(mac, services, temperature, humidity)))
            model = BLE_SENSOR_MODEL_ATC;
    }
    if (has_reading && !ble_sensor_values_plausible(temperature, humidity))
    {
        has_reading = false;
        if (ble_sensor_model_passive(model))
            model = 0;
    }
    return model;
}
