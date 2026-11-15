#pragma once

#include <cstddef>
#include <cstdint>
#include <type_traits>

// ATC_MiThermometer firmware on Xiaomi thermometers: service data of the
// Environmental Sensing service, unencrypted. Both layouts repeat the
// advertiser address, which keeps other devices that use this service out.
//   ATC1441 (13 bytes): MAC, temperature int16 BE x0.1, humidity %, battery %,
//                       battery mV BE, counter
//   pvvx custom (15 bytes): MAC reversed, temperature int16 LE x0.01,
//                       humidity uint16 LE x0.01, battery mV, battery %,
//                       counter, flags

static const uint16_t ATC_SERVICE_UUID = 0x181A;

// `mac` is the advertiser address, most significant byte first.
static bool atc_parse_data(const uint8_t *data, size_t size, const uint8_t *mac, float &temperature, float &humidity)
{
    if (size == 13)
    {
        for (int i = 0; i < 6; i++)
            if (data[i] != mac[i])
                return false;
        temperature = int16_t((data[6] << 8) | data[7]) * 0.1f;
        humidity = data[8];
        return true;
    }
    if (size == 15)
    {
        for (int i = 0; i < 6; i++)
            if (data[i] != mac[5 - i])
                return false;
        temperature = int16_t(data[6] | (data[7] << 8)) * 0.01f;
        humidity = uint16_t(data[8] | (data[9] << 8)) * 0.01f;
        return true;
    }
    return false;
}

// `Datas` is ESPHome's std::vector<ServiceData>.
template <typename Datas>
static bool atc_parse(const uint8_t *mac, const Datas &services, float &temperature, float &humidity)
{
    for (const auto &service : services)
    {
        using Uuid = typename std::decay<decltype(service.uuid)>::type;
        if (service.uuid == Uuid::from_uint16(ATC_SERVICE_UUID) &&
            atc_parse_data(service.data.data(), service.data.size(), mac, temperature, humidity))
            return true;
    }
    return false;
}
