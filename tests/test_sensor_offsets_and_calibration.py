"""Automated tests for sensor calibration offsets and general system settings.

Covers settings:
1. tc_offset, tr_offset, tw_offset, tui_offset, tuo_offset:
   Calibration offset numbers (-5.0°C to +5.0°C).
2. heat_cool_control_temperature_source_select:
   Selects process control temperature between Tc (heating/cooling flow) and Tv1 (zone 1 flow).
3. flow_sensor_enabled:
   Toggles hardware flow sensor presence.
4. flow_sensor_calibration:
   Number configuring pulses per liter calibration for flow sensor (e.g. 476 p/l).
5. advanced_settings_enabled:
   Switch enabling advanced configuration pages/settings.
6. analytics_enabled_switch:
   Switch enabling anonymous telemetry reporting.
"""

import pytest


def test_temperature_sensor_offset_numbers(clean_system):
    """
    Verify calibration offset numbers:
    - tc_offset, tr_offset, tw_offset, tui_offset, tuo_offset
    Verify that each setting accepts positive and negative offsets and persists state.
    """
    openamber = clean_system

    offset_entities = [
        ("tc_offset", 1.5, -2.0),
        ("tr_offset", 0.8, -1.2),
        ("tw_offset", 2.0, -1.5),
        ("tui_offset", -0.5, 1.0),
        ("tuo_offset", 1.2, -0.8),
    ]

    for entity_id, val1, val2 in offset_entities:
        # Set positive offset
        assert openamber.set_number(entity_id, val1) is True
        openamber.step(ms=50)
        current = float(openamber.get_entity(entity_id) or 0)
        assert current == pytest.approx(val1, 0.05), f"{entity_id} failed to set {val1}"

        # Set negative offset
        assert openamber.set_number(entity_id, val2) is True
        openamber.step(ms=50)
        current = float(openamber.get_entity(entity_id) or 0)
        assert current == pytest.approx(val2, 0.05), f"{entity_id} failed to set {val2}"

        # Reset to 0.0
        openamber.set_number(entity_id, 0.0)
        openamber.step(ms=50)


def test_heat_cool_control_temperature_source_selection(clean_system):
    """
    Verify heat_cool_control_temperature_source_select:
    - Option 'CV-aanvoer (Tc)': heat_cool_control_temperature reflects Tc.
    - Option 'CV-aanvoer zone 1 (Tv1)': heat_cool_control_temperature reflects Tv1.
    """
    openamber = clean_system

    # Set distinct temperatures for Tc and Tv1
    openamber.set_sensor("current_water_temperature_tc_sensor", 38.0)
    openamber.set_sensor("heat_cool_temperature_tc", 38.0)
    openamber.set_sensor("heat_cool_temperature_tv1", 31.0)
    openamber.step(ms=50)

    # 1. Source = Tc
    openamber.set_select("heat_cool_control_temperature_source_select", "CV-aanvoer (Tc)")
    openamber.advance_time(seconds=6, step_s=1)
    openamber.step(ms=100)

    ctrl_temp = float(openamber.get_entity("heat_cool_control_temperature") or 0)
    assert ctrl_temp == pytest.approx(38.0, 0.5), f"Expected control temp to track Tc (38°C), got {ctrl_temp}"

    # 2. Source = Tv1
    openamber.set_select("heat_cool_control_temperature_source_select", "CV-aanvoer zone 1 (Tv1)")
    openamber.advance_time(seconds=6, step_s=1)
    openamber.step(ms=100)

    ctrl_temp = float(openamber.get_entity("heat_cool_control_temperature") or 0)
    assert ctrl_temp == pytest.approx(31.0, 0.5), f"Expected control temp to track Tv1 (31°C), got {ctrl_temp}"

    # Reset back to Tc
    openamber.set_select("heat_cool_control_temperature_source_select", "CV-aanvoer (Tc)")
    openamber.advance_time(seconds=6, step_s=1)
    openamber.step(ms=50)


def test_flow_sensor_and_general_switches(clean_system):
    """
    Verify general hardware & feature settings:
    - flow_sensor_enabled switch
    - flow_sensor_calibration number
    - advanced_settings_enabled switch
    - analytics_enabled_switch
    """
    openamber = clean_system

    # Flow sensor enable switch
    assert openamber.set_switch("flow_sensor_enabled", True)
    openamber.step(ms=50)
    assert openamber.get_entity("flow_sensor_enabled") is True, "flow_sensor_enabled should be True"

    assert openamber.set_switch("flow_sensor_enabled", False)
    openamber.step(ms=50)
    assert openamber.get_entity("flow_sensor_enabled") is False, "flow_sensor_enabled should be False"

    # Flow sensor calibration
    assert openamber.set_number("flow_sensor_calibration", 476.0)
    openamber.step(ms=50)
    assert float(openamber.get_entity("flow_sensor_calibration") or 0) == 476.0, "flow_sensor_calibration should be 476.0"

    assert openamber.set_number("flow_sensor_calibration", 512.0)
    openamber.step(ms=50)
    assert float(openamber.get_entity("flow_sensor_calibration") or 0) == 512.0, "flow_sensor_calibration should be 512.0"

    # Advanced settings toggle
    assert openamber.set_switch("advanced_settings_enabled", True)
    openamber.step(ms=50)
    assert openamber.get_entity("advanced_settings_enabled") is True, "advanced_settings_enabled should be True"

    assert openamber.set_switch("advanced_settings_enabled", False)
    openamber.step(ms=50)
    assert openamber.get_entity("advanced_settings_enabled") is False, "advanced_settings_enabled should be False"

    # Analytics enabled toggle
    assert openamber.set_switch("analytics_enabled_switch", True)
    openamber.step(ms=50)
    assert openamber.get_entity("analytics_enabled_switch") is True, "analytics_enabled_switch should be True"

    assert openamber.set_switch("analytics_enabled_switch", False)
    openamber.step(ms=50)
    assert openamber.get_entity("analytics_enabled_switch") is False, "analytics_enabled_switch should be False"

