#pragma once

#include <cstdint>
#include <type_traits>

// SwitchBot Meter, Meter Plus and Indoor/Outdoor Thermo-Hygrometer.
//
// The reading is in the manufacturer data of every advertisement. The model
// byte that identifies a thermometer is only in the service data of the scan
// response, so it takes active scanning to learn it; older Meter firmware
// also only reports readings there.
//
// `Datas` is ESPHome's std::vector<ServiceData>: each entry has a 16-bit
// company ID or service UUID in `uuid` and the payload after it in `data`.

static const uint16_t SWITCHBOT_COMPANY_ID = 0x0969;
static const uint16_t SWITCHBOT_SERVICE_UUID = 0xFD3D;
static const uint16_t SWITCHBOT_LEGACY_SERVICE_UUID = 0x000D;

// Model bytes are 7 bits: the high bit is the encryption flag, so lower/upper
// case are the same product.
static bool switchbot_is_meter_model(uint8_t model)
{
    return model == 0x54 || model == 0x74 || model == 0x69 || model == 0x49 || model == 0x77 || model == 0x57;
}

// Product names as pySwitchbot / Home Assistant use them.
static const char *switchbot_model_name(uint8_t model)
{
    switch (model)
    {
    case 0x54: // 'T'
    case 0x74: // 't'
        return "SwitchBot Meter";
    case 0x69: // 'i'
    case 0x49: // 'I'
        return "SwitchBot Meter Plus";
    case 0x77: // 'w'
    case 0x57: // 'W'
        return "SwitchBot Indoor/Outdoor Thermo-Hygrometer";
    default:
        return nullptr;
    }
}

// Decode the shared temperature/humidity layout: decimal nibble, integer with
// sign bit (set = positive), humidity percentage.
static bool switchbot_decode(uint8_t decimal, uint8_t integer, uint8_t humidity_byte, float &temperature,
                             float &humidity)
{
    if ((decimal & 0x0F) > 9 || (humidity_byte & 0x7F) > 100)
        return false;
    temperature = (decimal & 0x0F) * 0.1f + (integer & 0x7F);
    if ((integer & 0x80) == 0)
        temperature = -temperature;
    humidity = humidity_byte & 0x7F;
    return true;
}

// The thermometer model in the service data, 0 when there is none. SwitchBot's
// service UUID plus model identify thermometers; the company ID alone would
// also admit unrelated SwitchBot products.
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
static bool switchbot_parse(const Datas &manufacturers, const Datas &services, float &temperature, float &humidity)
{
    for (const auto &mfr : manufacturers)
    {
        using Uuid = typename std::decay<decltype(mfr.uuid)>::type;
        if (mfr.uuid == Uuid::from_uint16(SWITCHBOT_COMPANY_ID) && mfr.data.size() >= 11 &&
            switchbot_decode(mfr.data[8], mfr.data[9], mfr.data[10], temperature, humidity))
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
            switchbot_decode(service.data[3], service.data[4], service.data[5], temperature, humidity))
            return true;
    }
    return false;
}
