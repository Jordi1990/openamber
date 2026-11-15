#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>

// Parsers for open thermometer advertisement formats. They only look at the
// service data bytes (the part after the 16-bit UUID) and have no ESP
// dependencies. Each returns true when a plausible temperature was found;
// humidity stays NAN when the sensor does not send one.

static const uint16_t BLE_BTHOME_SERVICE_UUID = 0xFCD2;
static const uint16_t BLE_BTHOME_V1_SERVICE_UUID = 0x181C;
// Environmental Sensing, used by the ATC1441 and pvvx custom formats.
static const uint16_t BLE_ATC_SERVICE_UUID = 0x181A;

static bool ble_reading_plausible(float temperature, float humidity)
{
    return temperature >= -50.0f && temperature <= 100.0f &&
           (std::isnan(humidity) || (humidity >= 0.0f && humidity <= 100.0f));
}

// Value size of BTHome v2 object IDs 0x00..0x65 (https://bthome.io/format/).
// 0: unknown or not skippable (parsing stops there, as the spec prescribes),
// 0xFF: text/raw, a length byte follows the ID.
static const uint8_t BLE_BTHOME_OBJECT_SIZE[] = {
    1, 1, 2, 2, 3, 3, 2, 2, 2, 1, 3, 3, 2, 2, 2, 1,       // 0x00
    1, 1, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,       // 0x10
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,       // 0x20
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 2, 2, 4, 2,       // 0x30
    2, 2, 3, 2, 2, 2, 1, 2, 2, 2, 2, 3, 4, 4, 4, 4,       // 0x40
    4, 2, 2, 0xFF, 0xFF, 4, 2, 1, 1, 1, 2, 4, 4, 2, 2, 2, // 0x50
    1, 2, 4, 4, 1, 1,                                     // 0x60
};

// BTHome v2, unencrypted. The first temperature and humidity objects are used.
static bool ble_parse_bthome(const uint8_t *data, size_t size, float &temperature, float &humidity)
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
        if (object >= sizeof(BLE_BTHOME_OBJECT_SIZE))
            break;
        size_t length = BLE_BTHOME_OBJECT_SIZE[object];
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
    return !std::isnan(temperature) && ble_reading_plausible(temperature, humidity);
}

// BTHome v1 ("HA BLE"), unencrypted, as older pvvx firmware sends it. Every
// object is: one byte with the length of what follows in bits 0-4 and the
// data type in bits 5-7 (0 unsigned, 1 signed), the object ID, then the
// little-endian value. Temperature (0x02) and humidity (0x03) are x0.01.
static bool ble_parse_bthome_v1(const uint8_t *data, size_t size, float &temperature, float &humidity)
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
    return !std::isnan(temperature) && ble_reading_plausible(temperature, humidity);
}

// ATC_MiThermometer firmware on Xiaomi thermometers. `mac` is the advertiser
// address, most significant byte first; both formats repeat it, which keeps
// other devices that use the 0x181A service out.
//   ATC1441 (13 bytes): MAC, temperature int16 BE x0.1, humidity %, battery %,
//                       battery mV BE, counter
//   pvvx custom (15 bytes): MAC reversed, temperature int16 LE x0.01,
//                       humidity uint16 LE x0.01, battery mV, battery %,
//                       counter, flags
static bool ble_parse_atc(const uint8_t *data, size_t size, const uint8_t *mac, float &temperature, float &humidity)
{
    if (size == 13)
    {
        for (int i = 0; i < 6; i++)
            if (data[i] != mac[i])
                return false;
        temperature = int16_t((data[6] << 8) | data[7]) * 0.1f;
        humidity = data[8];
    }
    else if (size == 15)
    {
        for (int i = 0; i < 6; i++)
            if (data[i] != mac[5 - i])
                return false;
        temperature = int16_t(data[6] | (data[7] << 8)) * 0.01f;
        humidity = uint16_t(data[8] | (data[9] << 8)) * 0.01f;
    }
    else
        return false;
    return ble_reading_plausible(temperature, humidity);
}
