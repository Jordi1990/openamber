#pragma once

#include <cmath>

// Simulated BLE thermometers for the host simulator (mock/ble_mock.yaml). Each
// one "advertises" like a BTHome sensor and goes through the same path as a real
// advertisement (ble_sensor_wanted / ble_sensor_apply in ble/ble_sensor.h), so
// searching, pairing, readings and timeouts behave as on the device.
static const unsigned BLE_MOCK_SENSORS = 3;
static const char *const BLE_MOCK_MACS[BLE_MOCK_SENSORS] = {"A4:C1:38:00:00:01", "A4:C1:38:00:00:02",
                                                            "A4:C1:38:00:00:03"};

// A NAN temperature means the sensor is silent (out of range, empty battery).
static void ble_sensor_mock_advertise(unsigned index, float temperature, float humidity)
{
    BleSensorAddress address;
    if (index >= BLE_MOCK_SENSORS || std::isnan(temperature) || !ble_sensor_parse_mac(BLE_MOCK_MACS[index], address))
        return;
    bool discover;
    if (!ble_sensor_wanted(address, &discover))
        return;
    BleSensorReading reading;
    reading.temperature = temperature;
    reading.humidity = humidity;
    reading.valid = ble_sensor_values_plausible(temperature, humidity);
    if (reading.valid)
        reading.received_ms = millis();
    ble_sensor_apply(address, BLE_SENSOR_MODEL_BTHOME, reading, discover);
}
