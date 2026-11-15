#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <type_traits>

// BTHome thermometers, unencrypted: v2 (https://bthome.io/format/) and v1
// ("HA BLE", older pvvx firmware). Everything is in the service data of every
// advertisement. The first temperature and humidity objects are used;
// humidity stays NAN when the sensor does not send one.

static const uint16_t BTHOME_SERVICE_UUID = 0xFCD2;
static const uint16_t BTHOME_V1_SERVICE_UUID = 0x181C;
static const uint16_t BTHOME_V1_ENCRYPTED_SERVICE_UUID = 0x181E;

// Value size of BTHome v2 object IDs 0x00..0x65.
// 0: unknown or not skippable (parsing stops there, as the spec prescribes),
// 0xFF: text/raw, a length byte follows the ID.
static const uint8_t BTHOME_OBJECT_SIZE[] = {
    1, 1, 2, 2, 3, 3, 2, 2, 2, 1, 3, 3, 2, 2, 2, 1,       // 0x00
    1, 1, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,       // 0x10
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,       // 0x20
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 2, 2, 4, 2,       // 0x30
    2, 2, 3, 2, 2, 2, 1, 2, 2, 2, 2, 3, 4, 4, 4, 4,       // 0x40
    4, 2, 2, 0xFF, 0xFF, 4, 2, 1, 1, 1, 2, 4, 4, 2, 2, 2, // 0x50
    1, 2, 4, 4, 1, 1,                                     // 0x60
};

static bool bthome_parse_v2(const uint8_t *data, size_t size, float &temperature, float &humidity)
{
    // Device info: bit 0 = encrypted, bits 5-7 = version (2).
    if (size < 2 || (data[0] & 0x01) != 0 || (data[0] >> 5) != 2)
        return false;
    temperature = NAN;
    humidity = NAN;
    size_t pos = 1;
    while (pos < size)
    {
        uint8_t object = data[pos++];
        if (object >= sizeof(BTHOME_OBJECT_SIZE))
            break;
        size_t length = BTHOME_OBJECT_SIZE[object];
        if (length == 0xFF)
        {
            if (pos >= size)
                break;
            length = data[pos++];
        }
        else if (length == 0)
            break;
        if (pos + length > size)
            break;
        const uint8_t *value = data + pos;
        pos += length;
        if (std::isnan(temperature))
        {
            if (object == 0x02)
                temperature = int16_t(value[0] | (value[1] << 8)) * 0.01f;
            else if (object == 0x45)
                temperature = int16_t(value[0] | (value[1] << 8)) * 0.1f;
            else if (object == 0x57)
                temperature = int8_t(value[0]);
            else if (object == 0x58)
                temperature = int8_t(value[0]) * 0.35f;
        }
        if (std::isnan(humidity))
        {
            if (object == 0x03)
                humidity = uint16_t(value[0] | (value[1] << 8)) * 0.01f;
            else if (object == 0x2E)
                humidity = value[0];
        }
    }
    return !std::isnan(temperature);
}

// v1: every object is one byte with the length of what follows in bits 0-4
// and the data type in bits 5-7 (0 unsigned, 1 signed), the object ID, then
// the little-endian value. Temperature (0x02) and humidity (0x03) are x0.01.
static bool bthome_parse_v1(const uint8_t *data, size_t size, float &temperature, float &humidity)
{
    temperature = NAN;
    humidity = NAN;
    size_t pos = 0;
    while (pos + 1 < size)
    {
        size_t length = data[pos] & 0x1F;
        uint8_t type = data[pos] >> 5;
        if (length < 1 || pos + 1 + length > size)
            break;
        uint8_t object = data[pos + 1];
        size_t bytes = length - 1;
        if (type <= 1 && bytes >= 1 && bytes <= 4)
        {
            uint32_t raw = 0;
            for (size_t i = 0; i < bytes; i++)
                raw |= uint32_t(data[pos + 2 + i]) << (8 * i);
            int32_t value = int32_t(raw);
            if (type == 1 && bytes < 4 && (raw & (1u << (8 * bytes - 1))))
                value = int32_t(raw | (0xFFFFFFFFu << (8 * bytes)));
            if (object == 0x02 && std::isnan(temperature))
                temperature = value * 0.01f;
            else if (object == 0x03 && std::isnan(humidity))
                humidity = value * 0.01f;
        }
        pos += 1 + length;
    }
    return !std::isnan(temperature);
}

// `Datas` is ESPHome's std::vector<ServiceData>.
template <typename Datas> static bool bthome_parse(const Datas &services, float &temperature, float &humidity)
{
    for (const auto &service : services)
    {
        using Uuid = typename std::decay<decltype(service.uuid)>::type;
        if (service.uuid == Uuid::from_uint16(BTHOME_SERVICE_UUID) &&
            bthome_parse_v2(service.data.data(), service.data.size(), temperature, humidity))
            return true;
        if (service.uuid == Uuid::from_uint16(BTHOME_V1_SERVICE_UUID) &&
            bthome_parse_v1(service.data.data(), service.data.size(), temperature, humidity))
            return true;
    }
    return false;
}
