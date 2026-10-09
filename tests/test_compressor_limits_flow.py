"""Automated tests for compressor mode limiting settings across DHW, DHW Winter, Heating, and Cooling.
Covers:
1. DHW base compressor modes (Beperkt..Maximaal -> modes 4..10)
2. DHW winter compressor modes (Gemiddeld..Maximaal -> modes 7..10 when Ta <= threshold)
3. DHW winter temperature threshold crossing and threshold adjustment
4. Space Heating compressor mode limits & soft-start capping
5. Space Cooling compressor mode limits (Beperkt..Maximaal -> modes 1..7)
6. Dynamic mode limit adjustments during active operation
"""

import pytest


@pytest.mark.parametrize("mode_name,expected_mode_index", [
    ("Beperkt", 4),
    ("Zeer laag", 5),
    ("Laag", 6),
    ("Gemiddeld", 7),
    ("Verhoogd", 8),
    ("Hoog", 9),
    ("Maximaal", 10),
])
def test_dhw_base_compressor_modes(clean_system, mode_name, expected_mode_index):
    """
    Verify all 7 selectable DHW base compressor modes configure the compressor to the corresponding mode index (4..10).
    """
    openamber = clean_system

    openamber.set_select("dhw_compressor_mode", mode_name)
    openamber.set_number("dhw_temperature_threshold_max_compressor_mode", 5.0)
    openamber.set_sensor("temperature_outside_ta", 10.0)  # Ta > threshold -> base mode active
    openamber.set_sensor("dhw_temperature_tw_sensor", 38.0)
    openamber.step(ms=50)

    # Advance time for 3-way valve switch (60s) + pump wait/settle (130s) + softstart (190s)
    openamber.advance_time(seconds=400, step_s=20)
    openamber.step(ms=50)

    assert openamber.get_entity("state_machine_state_main") == "DHW"
    assert openamber.get_entity("state_machine_state_dhw") == "Compressor running"

    actual_mode = int(openamber.get_entity("compressor_control_select") or 0)
    assert actual_mode == expected_mode_index, (
        f"DHW base mode '{mode_name}' should set compressor to mode {expected_mode_index}, got {actual_mode}"
    )


@pytest.mark.parametrize("winter_mode_name,expected_mode_index", [
    ("Gemiddeld", 7),
    ("Verhoogd", 8),
    ("Hoog", 9),
    ("Maximaal", 10),
])
def test_dhw_winter_compressor_modes(clean_system, winter_mode_name, expected_mode_index):
    """
    Verify all 4 selectable DHW winter compressor modes configure the compressor to the corresponding mode index (7..10)
    when outside temperature is at or below the winter threshold.
    """
    openamber = clean_system

    openamber.set_number("dhw_temperature_threshold_max_compressor_mode", 5.0)
    openamber.set_select("dhw_compressor_mode", "Beperkt")  # Base mode would be 4
    openamber.set_select("dhw_compressor_mode_max", winter_mode_name)
    openamber.set_sensor("temperature_outside_ta", 2.0)  # Ta (2.0°C) <= threshold (5.0°C) -> winter mode active
    openamber.set_sensor("dhw_temperature_tw_sensor", 38.0)
    openamber.step(ms=50)

    # Advance time for 3-way valve switch (60s) + pump wait/settle (130s) + softstart (190s)
    openamber.advance_time(seconds=400, step_s=20)
    openamber.step(ms=50)

    assert openamber.get_entity("state_machine_state_main") == "DHW"
    assert openamber.get_entity("state_machine_state_dhw") == "Compressor running"

    actual_mode = int(openamber.get_entity("compressor_control_select") or 0)
    assert actual_mode == expected_mode_index, (
        f"DHW winter mode '{winter_mode_name}' at Ta=2.0°C should set compressor to mode {expected_mode_index}, got {actual_mode}"
    )


def test_dhw_winter_temperature_threshold_crossing_and_adjustment(clean_system):
    """
    Verify dynamic transitions between DHW base mode and winter mode:
    1. System runs DHW with Ta = 10.0°C (> threshold 5.0°C) -> Base mode (Beperkt = mode 4).
    2. Ta drops to 0.0°C (<= threshold 5.0°C) -> Ramps up to Winter mode (Maximaal = mode 10)
       respecting the 5-minute (300s) up-frequency step interval.
    3. Ta rises to 8.0°C (> threshold 5.0°C) -> Ramps down to Base mode (Beperkt = mode 4)
       respecting the 10-second down-frequency step interval.
    4. Threshold adjusted to 12.0°C (now Ta 8.0°C <= threshold 12.0°C) -> Ramps back up to Winter mode (mode 10).
    """
    openamber = clean_system

    openamber.set_number("dhw_temperature_threshold_max_compressor_mode", 5.0)
    openamber.set_select("dhw_compressor_mode", "Beperkt")  # mode 4
    openamber.set_select("dhw_compressor_mode_max", "Maximaal")  # mode 10
    openamber.set_sensor("temperature_outside_ta", 10.0)  # Base mode initially
    openamber.set_sensor("dhw_temperature_tw_sensor", 38.0)
    openamber.step(ms=50)

    openamber.advance_time(seconds=400, step_s=20)
    openamber.step(ms=50)

    # 1. Base mode active
    assert int(openamber.get_entity("compressor_control_select") or 0) == 4

    # 2. Temperature drops below winter threshold
    openamber.set_sensor("temperature_outside_ta", 0.0)
    # Ramping from mode 4 to mode 10 is 6 steps * 300s/step = 1800s
    openamber.advance_time(seconds=1850, step_s=30)
    openamber.step(ms=50)

    # Winter mode active at max mode 10
    assert int(openamber.get_entity("compressor_control_select") or 0) == 10

    # 3. Temperature rises above winter threshold
    openamber.set_sensor("temperature_outside_ta", 8.0)
    # Ramping down from mode 10 to mode 4 is 6 steps * 10s/step = 60s
    openamber.advance_time(seconds=80, step_s=5)
    openamber.step(ms=50)

    # Reverted to base mode 4
    assert int(openamber.get_entity("compressor_control_select") or 0) == 4

    # 4. User changes winter threshold to 12.0°C
    openamber.set_number("dhw_temperature_threshold_max_compressor_mode", 12.0)
    # Now 8.0°C <= 12.0°C -> Ramps back up to mode 10 (6 * 300s)
    openamber.advance_time(seconds=1850, step_s=30)
    openamber.step(ms=50)

    assert int(openamber.get_entity("compressor_control_select") or 0) == 10


def test_heating_compressor_softstart_capping(clean_system):
    """
    Verify Space Heating soft-start mode is capped by heat_compressor_mode:
    1. Large delta T (|Tc - Target| = 15°C) would normally request soft-start mode 5.
    2. When heat_compressor_mode is 'Beperkt' (max allowed mode = 4), soft-start is capped to mode 4.
    """
    openamber = clean_system

    openamber.set_select("thermostat_mode_select", "Extern")
    openamber.set_select("heat_mode_select", "Extern setpoint")
    openamber.set_number("manual_setpoint", 35.0)
    openamber.set_climate("pid_heat_temperature_control", target_temperature=35.0)
    openamber.set_select("heat_compressor_mode", "Beperkt")  # Max mode = 4
    openamber.set_switch("heat_demand_switch", True)
    openamber.set_sensor("current_water_temperature_tc_sensor", 20.0)  # delta = 15°C (> 10°C -> wants mode 5)
    openamber.set_sensor("heat_cool_temperature_tc", 20.0)
    openamber.set_sensor("outlet_temperature_tuo", 20.0)
    openamber.set_sensor("inlet_temperature_tui", 18.0)
    openamber.step(ms=50)

    # Advance through pump interval (900s) + pump settle (140s) to enter compressor start
    openamber.advance_time(seconds=900, step_s=30)
    openamber.advance_time(seconds=140, step_s=10)
    openamber.step(ms=50)

    # Softstart mode must be capped at 4 (Beperkt) instead of 5
    softstart_mode = int(openamber.get_entity("compressor_control_select") or 0)
    assert softstart_mode == 4, f"Softstart mode should be capped to 4 by 'Beperkt', got {softstart_mode}"


@pytest.mark.parametrize("limit_setting,max_expected_mode", [
    ("Beperkt", 4),
    ("Laag", 6),
    ("Gemiddeld", 7),
    ("Hoog", 9),
    ("Maximaal", 10),
])
def test_heating_compressor_mode_limits(clean_system, limit_setting, max_expected_mode):
    """
    Verify Space Heating compressor mode is capped at the selected heat_compressor_mode limit
    even under maximum PID heating demand (Tc far below setpoint).
    """
    openamber = clean_system

    openamber.set_select("thermostat_mode_select", "Extern")
    openamber.set_select("heat_mode_select", "Extern setpoint")
    openamber.set_number("manual_setpoint", 40.0)
    openamber.set_climate("pid_heat_temperature_control", target_temperature=40.0)
    openamber.set_number("backup_heater_degmin_threshold", 999.0)
    openamber.set_select("heat_compressor_mode", limit_setting)
    openamber.set_switch("heat_demand_switch", True)
    openamber.set_sensor("current_water_temperature_tc_sensor", 25.0)  # High demand
    openamber.set_sensor("heat_cool_temperature_tc", 25.0)
    openamber.set_sensor("outlet_temperature_tuo", 25.0)
    openamber.set_sensor("inlet_temperature_tui", 22.0)
    openamber.step(ms=50)

    # Advance through pump interval + settle + softstart into COMPRESSOR_RUNNING
    openamber.advance_time(seconds=900, step_s=30)
    openamber.advance_time(seconds=140, step_s=10)
    openamber.advance_time(seconds=190, step_s=10)
    openamber.step(ms=50)

    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"

    current_mode = int(openamber.get_entity("compressor_control_select") or 0)
    assert current_mode <= max_expected_mode, (
        f"Heating mode '{limit_setting}' must not exceed mode {max_expected_mode}, got {current_mode}"
    )


@pytest.mark.parametrize("limit_setting,max_expected_mode", [
    ("Beperkt", 1),
    ("Laag", 3),
    ("Gemiddeld", 4),
    ("Hoog", 6),
    ("Maximaal", 7),
])
def test_cooling_compressor_mode_limits(clean_system, limit_setting, max_expected_mode):
    """
    Verify Space Cooling compressor mode is capped at the selected cool_compressor_mode limit
    (which maps to lower frequencies 1..7) under cooling demand.
    """
    openamber = clean_system

    openamber.set_select("thermostat_mode_select", "Extern")
    openamber.set_select("cool_mode_select", "Intern setpoint")
    openamber.set_number("cooling_setpoint_number", 18.0)
    openamber.set_climate("pid_cool_temperature_control", target_temperature=18.0)
    openamber.set_select("cool_compressor_mode", limit_setting)
    openamber.set_switch("cool_demand_switch", True)
    openamber.set_sensor("current_water_temperature_tc_sensor", 28.0)  # 28°C > 18°C -> cooling demand
    openamber.set_sensor("heat_cool_temperature_tc", 28.0)
    openamber.set_sensor("outlet_temperature_tuo", 28.0)
    openamber.set_sensor("inlet_temperature_tui", 30.0)
    openamber.step(ms=50)

    # Advance through pump interval + settle + softstart into COMPRESSOR_RUNNING
    openamber.advance_time(seconds=900, step_s=30)
    openamber.advance_time(seconds=140, step_s=10)
    openamber.advance_time(seconds=190, step_s=10)
    openamber.step(ms=50)

    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"

    current_mode = int(openamber.get_entity("compressor_control_select") or 0)
    assert current_mode <= max_expected_mode, (
        f"Cooling mode '{limit_setting}' must not exceed mode {max_expected_mode}, got {current_mode}"
    )
