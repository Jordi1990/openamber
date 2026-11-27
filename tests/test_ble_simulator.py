"""Simulated BLE thermometers in the virtual display (mock/ble_mock.yaml): the real BLE logic and UI, fed by three
simulated BTHome sensors. Sensor 1 (A4:C1:38:00:00:01, 5 °C / 85 %) is paired in slot 1; nothing is read until the
Bluetooth switch is on."""

import pytest


def test_ble_room_temperature_source(clean_system):
    openamber = clean_system
    # The wired Tr value as the fixture left it (not changed here, so the thermostat keeps its state for later tests)
    tr = openamber.get_entity("room_temperature")

    # Bluetooth off: the effective room temperature is the wired Tr sensor
    openamber.advance_time(seconds=15, step_s=1)
    assert openamber.get_entity("effective_room_temperature") == pytest.approx(tr, abs=0.05)

    # Bluetooth on and BLE 1 as source: the simulated sensor's values
    openamber.set_switch("ble_enabled_switch", True)
    openamber.set_select("room_temperature_source_select", "BLE 1")
    openamber.advance_time(seconds=15, step_s=1)
    assert openamber.get_entity("effective_room_temperature") == pytest.approx(5.0, abs=0.05)
    assert openamber.get_entity("effective_room_humidity") == pytest.approx(85.0, abs=0.5)
    assert openamber.get_widget("ble_source_dropdown")["text"].startswith("BTHome")

    # The mock values can be changed
    openamber.set_number("mock_ble_1_temperature", 21.5)
    openamber.advance_time(seconds=15, step_s=1)
    assert openamber.get_entity("effective_room_temperature") == pytest.approx(21.5, abs=0.05)

    # Bluetooth off again: back to Tr
    openamber.set_switch("ble_enabled_switch", False)
    openamber.advance_time(seconds=15, step_s=1)
    assert openamber.get_entity("effective_room_temperature") == pytest.approx(tr, abs=0.05)
