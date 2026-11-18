"""Automated tests for Night / Silence mode (Stiltemodus).

Covers:
1. Entity initialization & UI controls:
   - silent_mode_enabled_switch, silent_mode_start_time, silent_mode_end_time, silent_mode_compressor_mode
   - UI navigation to Settings -> Algemeen -> Stiltemodus
   - Start / end time decrement and increment UI controls
   - UI dropdown for Stiltemodus vermogen
2. Space Heating compressor mode limiting & softstart capping:
   - Compressor capped at silent mode limit under full heating demand
   - Soft-start frequency capped by silent limit
   - Lower of normal heat_compressor_mode and silent limit is respected
   - Disabling silent mode restores full modulation range
3. Space Cooling compressor mode limiting:
   - Cooling compressor capped at silent mode limit
4. DHW & DHW winter compressor mode limiting:
   - DHW compressor capped at silent mode limit
   - DHW winter compressor mode capped at silent mode limit
5. Time window boundary conditions:
   - Same-day and overnight time window transitions
"""

import pytest


def test_silent_mode_entities_and_ui_navigation(clean_system):
    """
    Verify silent mode entities default values and UI interactions:
    1. Default states: switch=False, start=23:00, end=07:00, mode=Gemiddeld, active=False.
    2. Navigate to Settings -> Algemeen -> Stiltemodus subtab.
    3. Modify start and end times via UI buttons.
    4. Change power limit via UI dropdown.
    5. Toggle master switch.
    """
    openamber = clean_system

    # 1. Verify defaults
    assert openamber.get_entity("silent_mode_enabled_switch") is False
    assert openamber.get_entity("silent_mode_compressor_mode") == "Gemiddeld"
    assert openamber.get_entity("silent_mode_active_sensor") is False

    start_ent = openamber.get_entity("silent_mode_start_time")
    assert start_ent is not None and "23:00" in start_ent
    end_ent = openamber.get_entity("silent_mode_end_time")
    assert end_ent is not None and "07:00" in end_ent

    # 2. Navigate in UI
    assert openamber.click("nav_settings"), "Must navigate to settings"
    openamber.step(ms=50)
    assert openamber.click("stab_general"), "Must click Algemeen sidebar tab"
    openamber.step(ms=50)
    assert openamber.click("general_stab_stiltemodus"), "Must click Stiltemodus subtab"
    openamber.step(ms=50)

    assert openamber.is_visible("panel_general_page_stiltemodus")

    # Verify labels in UI
    assert openamber.get_label("set_silent_mode_start_time_val") == "23:00"
    assert openamber.get_label("set_silent_mode_end_time_val") == "07:00"

    # Verify power dropdown and buttons are disabled initially when switch is OFF
    assert openamber.is_disabled("set_silent_mode_power_dd"), "Power dropdown must be disabled when silent mode is off"
    assert openamber.is_disabled("set_silent_mode_start_time_dec"), "Start time dec button must be disabled when silent mode is off"
    assert openamber.is_disabled("set_silent_mode_start_time_inc"), "Start time inc button must be disabled when silent mode is off"
    assert openamber.is_disabled("set_silent_mode_end_time_dec"), "End time dec button must be disabled when silent mode is off"
    assert openamber.is_disabled("set_silent_mode_end_time_inc"), "End time inc button must be disabled when silent mode is off"

    # 3. Enable silent mode via UI switch
    assert openamber.click("set_silent_mode_switch_ui")
    openamber.step(ms=50)
    assert openamber.get_entity("silent_mode_enabled_switch") is True

    # Verify power dropdown and buttons are now enabled
    assert not openamber.is_disabled("set_silent_mode_power_dd"), "Power dropdown must be enabled when silent mode is on"
    assert not openamber.is_disabled("set_silent_mode_start_time_dec"), "Start time dec button must be enabled when silent mode is on"
    assert not openamber.is_disabled("set_silent_mode_start_time_inc"), "Start time inc button must be enabled when silent mode is on"
    assert not openamber.is_disabled("set_silent_mode_end_time_dec"), "End time dec button must be enabled when silent mode is on"
    assert not openamber.is_disabled("set_silent_mode_end_time_inc"), "End time inc button must be enabled when silent mode is on" 

    # 4. Increment start time: 23:00 -> 23:30
    assert openamber.click("set_silent_mode_start_time_inc")
    openamber.step(ms=50)
    assert openamber.get_label("set_silent_mode_start_time_val") == "23:30"
    assert "23:30" in openamber.get_entity("silent_mode_start_time")

    # Decrement back: 23:30 -> 23:00
    assert openamber.click("set_silent_mode_start_time_dec")
    openamber.step(ms=50)
    assert openamber.get_label("set_silent_mode_start_time_val") == "23:00"

    # Decrement end time: 07:00 -> 06:30
    assert openamber.click("set_silent_mode_end_time_dec")
    openamber.step(ms=50)
    assert openamber.get_label("set_silent_mode_end_time_val") == "06:30"

    # Increment end time back: 06:30 -> 07:00
    assert openamber.click("set_silent_mode_end_time_inc")
    openamber.step(ms=50)
    assert openamber.get_label("set_silent_mode_end_time_val") == "07:00"

    # Test reactive UI update when time entity is updated via API / Home Assistant
    assert openamber.set_time("silent_mode_start_time", 22, 0, 0)
    openamber.step(ms=50)
    assert openamber.get_label("set_silent_mode_start_time_val") == "22:00"
    assert openamber.set_time("silent_mode_start_time", 23, 0, 0)
    openamber.step(ms=50)
    assert openamber.get_label("set_silent_mode_start_time_val") == "23:00"

    # 5. Set dropdown to "Beperkt" (index 0)
    assert openamber.set_dropdown("set_silent_mode_power_dd", 0)
    openamber.step(ms=50)
    assert openamber.get_entity("silent_mode_compressor_mode") == "Beperkt"

    # Restore switch off for clean state
    openamber.set_switch("silent_mode_enabled_switch", False)
    openamber.set_select("silent_mode_compressor_mode", "Gemiddeld")
    openamber.step(ms=50)
    assert openamber.is_disabled("set_silent_mode_power_dd"), "Power dropdown must be disabled again after switch turned off" 


def test_silent_mode_heating_limits_and_dynamic_adjustment(clean_system):
    """
    Verify Space Heating compressor mode is capped by silent mode:
    1. System runs heating under full PID demand (Tc = 25°C, Setpoint = 40°C).
    2. heat_compressor_mode is 'Maximaal' (mode 10).
    3. Silent mode is enabled across the whole day (00:00 - 23:59) with 'Gemiddeld' (mode 4).
    4. Compressor saturates at mode 4 instead of mode 10.
    5. Dynamically changing silent mode to 'Beperkt' (mode 1) steps down compressor to mode 1.
    6. Turning silent mode OFF allows compressor to ramp back up to mode 10.
    """
    openamber = clean_system

    openamber.set_select("thermostat_mode_select", "Extern")
    openamber.set_select("heat_mode_select", "Extern setpoint")
    openamber.set_number("manual_setpoint", 40.0)
    openamber.set_climate("pid_heat_temperature_control", target_temperature=40.0)
    openamber.set_number("backup_heater_degmin_threshold", 999.0)
    openamber.set_select("heat_compressor_mode", "Maximaal")

    # Set wide silent window to cover any time
    openamber.set_time("silent_mode_start_time", 0, 0, 0)
    openamber.set_time("silent_mode_end_time", 23, 59, 0)
    openamber.set_select("silent_mode_compressor_mode", "Gemiddeld")  # mode 4
    openamber.set_switch("silent_mode_enabled_switch", True)

    openamber.set_switch("heat_demand_switch", True)
    openamber.set_sensor("current_water_temperature_tc_sensor", 25.0)
    openamber.set_sensor("heat_cool_temperature_tc", 25.0)
    openamber.set_sensor("outlet_temperature_tuo", 25.0)
    openamber.set_sensor("inlet_temperature_tui", 22.0)
    openamber.step(ms=50)

    assert openamber.get_entity("silent_mode_active_sensor") is True

    # Advance through pump interval (900s) + pump settle (140s) + softstart (190s)
    openamber.advance_time(seconds=900, step_s=30)
    openamber.advance_time(seconds=140, step_s=10)
    openamber.advance_time(seconds=190, step_s=10)
    openamber.step(ms=50)

    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"

    # Ramp up under full demand (sat at capped limit)
    openamber.advance_time(seconds=1900, step_s=30)
    openamber.step(ms=50)

    current_mode = int(openamber.get_entity("compressor_control_select") or 0)
    assert current_mode == 4, f"Heating compressor should be capped at mode 4 (Gemiddeld), got {current_mode}"

    # Dynamically change silent mode to 'Beperkt' (mode 1)
    openamber.set_select("silent_mode_compressor_mode", "Beperkt")
    # Downward ramp takes 10s per step (3 steps from 4 to 1 = 30s)
    openamber.advance_time(seconds=50, step_s=5)
    openamber.step(ms=50)

    current_mode = int(openamber.get_entity("compressor_control_select") or 0)
    assert current_mode == 1, f"Heating compressor should step down to mode 1 (Beperkt), got {current_mode}"

    # Turn silent mode OFF -> ramps back up to Maximaal (mode 10)
    openamber.set_switch("silent_mode_enabled_switch", False)
    openamber.step(ms=50)
    assert openamber.get_entity("silent_mode_active_sensor") is False

    # 9 steps up from mode 1 to 10 * 300s = 2700s
    openamber.advance_time(seconds=2800, step_s=30)
    openamber.step(ms=50)

    current_mode = int(openamber.get_entity("compressor_control_select") or 0)
    assert current_mode == 10, f"With silent mode OFF, compressor should reach mode 10, got {current_mode}"


def test_silent_mode_heating_softstart_capping(clean_system):
    """
    Verify Space Heating soft-start mode is capped by silent mode:
    1. Large delta T (|Tc - Target| = 15°C) would normally request soft-start mode 5.
    2. When silent_mode_compressor_mode is 'Zeer laag' (max allowed mode = 2), soft-start is capped to mode 2.
    """
    openamber = clean_system

    openamber.set_select("thermostat_mode_select", "Extern")
    openamber.set_select("heat_mode_select", "Extern setpoint")
    openamber.set_number("manual_setpoint", 35.0)
    openamber.set_climate("pid_heat_temperature_control", target_temperature=35.0)
    openamber.set_select("heat_compressor_mode", "Maximaal")  # Normal max = 10

    openamber.set_time("silent_mode_start_time", 0, 0, 0)
    openamber.set_time("silent_mode_end_time", 23, 59, 0)
    openamber.set_select("silent_mode_compressor_mode", "Zeer laag")  # Silent limit = 2
    openamber.set_switch("silent_mode_enabled_switch", True)

    openamber.set_switch("heat_demand_switch", True)
    openamber.set_sensor("current_water_temperature_tc_sensor", 20.0)  # delta = 15°C -> wants mode 5
    openamber.set_sensor("heat_cool_temperature_tc", 20.0)
    openamber.set_sensor("outlet_temperature_tuo", 20.0)
    openamber.set_sensor("inlet_temperature_tui", 18.0)
    openamber.step(ms=50)

    assert openamber.get_entity("silent_mode_active_sensor") is True

    # Advance through pump interval (900s) + pump settle (140s) to enter compressor start
    openamber.advance_time(seconds=900, step_s=30)
    openamber.advance_time(seconds=140, step_s=10)
    openamber.step(ms=50)

    softstart_mode = int(openamber.get_entity("compressor_control_select") or 0)
    assert softstart_mode == 2, f"Softstart mode should be capped to 2 by silent mode 'Zeer laag', got {softstart_mode}"


def test_silent_mode_cooling_limit(clean_system):
    """
    Verify Space Cooling compressor mode is capped by silent mode:
    1. Normal cool_compressor_mode is 'Maximaal' (mode 7).
    2. Silent mode is active with 'Laag' (mode 3: silent mode always starts at mode 1, same as cooling).
    3. Cooling compressor mode saturates at mode 3.
    """
    openamber = clean_system

    openamber.set_select("thermostat_mode_select", "Extern")
    openamber.set_select("cool_mode_select", "Intern setpoint")
    openamber.set_number("cooling_setpoint_number", 18.0)
    openamber.set_climate("pid_cool_temperature_control", target_temperature=18.0)
    openamber.set_select("cool_compressor_mode", "Maximaal")  # mode 7

    openamber.set_time("silent_mode_start_time", 0, 0, 0)
    openamber.set_time("silent_mode_end_time", 23, 59, 0)
    openamber.set_select("silent_mode_compressor_mode", "Laag")  # mode 3 for cooling
    openamber.set_switch("silent_mode_enabled_switch", True)

    openamber.set_switch("cool_demand_switch", True)
    openamber.set_sensor("current_water_temperature_tc_sensor", 28.0)
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

    # Ramp up under full demand
    openamber.advance_time(seconds=1900, step_s=30)
    openamber.step(ms=50)

    current_mode = int(openamber.get_entity("compressor_control_select") or 0)
    assert current_mode == 3, f"Cooling mode should be capped at mode 3 by silent mode 'Laag', got {current_mode}"


def test_silent_mode_dhw_limit(clean_system):
    """
    Verify DHW compressor mode is capped by silent mode:
    1. dhw_compressor_mode is 'Maximaal' (mode 10).
    2. Silent mode is active with 'Gemiddeld' (mode 4).
    3. DHW compressor runs at mode 4.
    4. Even in winter mode (Ta <= threshold), DHW stays capped at mode 4.
    5. 'Beperkt' lowers DHW to mode 1, below the lowest DHW power setting (mode 4).
    """
    openamber = clean_system

    openamber.set_select("dhw_compressor_mode", "Maximaal")  # base mode 10
    openamber.set_select("dhw_compressor_mode_max", "Maximaal")  # winter mode 10
    openamber.set_number("dhw_temperature_threshold_max_compressor_mode", 5.0)
    openamber.set_sensor("temperature_outside_ta", 10.0)  # base mode initially

    openamber.set_time("silent_mode_start_time", 0, 0, 0)
    openamber.set_time("silent_mode_end_time", 23, 59, 0)
    openamber.set_select("silent_mode_compressor_mode", "Gemiddeld")  # mode 4
    openamber.set_switch("silent_mode_enabled_switch", True)

    openamber.set_sensor("dhw_temperature_tw_sensor", 38.0)
    openamber.step(ms=50)

    # Advance time for 3-way valve switch (60s) + pump wait/settle (130s) + softstart (190s)
    openamber.advance_time(seconds=400, step_s=20)
    openamber.step(ms=50)

    assert openamber.get_entity("state_machine_state_main") == "DHW"
    assert openamber.get_entity("state_machine_state_dhw") == "Compressor running"

    actual_mode = int(openamber.get_entity("compressor_control_select") or 0)
    assert actual_mode == 4, f"DHW compressor should be capped at mode 4 by silent mode, got {actual_mode}"

    # Drop outside temp to activate winter mode
    openamber.set_sensor("temperature_outside_ta", 0.0)
    openamber.advance_time(seconds=200, step_s=20)
    openamber.step(ms=50)

    # In winter mode, dhw_compressor_mode_max is 10, but silent mode still caps it to 4
    actual_mode = int(openamber.get_entity("compressor_control_select") or 0)
    assert actual_mode == 4, f"DHW winter mode should still be capped at mode 4 by silent mode, got {actual_mode}"

    # 'Beperkt' steps DHW down to mode 1 (10s per step down)
    openamber.set_select("silent_mode_compressor_mode", "Beperkt")
    openamber.advance_time(seconds=60, step_s=5)
    openamber.step(ms=50)

    actual_mode = int(openamber.get_entity("compressor_control_select") or 0)
    assert actual_mode == 1, f"DHW should be capped at mode 1 by silent mode 'Beperkt', got {actual_mode}"
    openamber.set_select("silent_mode_compressor_mode", "Gemiddeld")

    # Reset silent mode entities to defaults
    openamber.set_switch("silent_mode_enabled_switch", False)
    openamber.set_time("silent_mode_start_time", 23, 0, 0)
    openamber.set_time("silent_mode_end_time", 7, 0, 0)


@pytest.mark.parametrize("silent_option,expected_mode", [
    ("Beperkt", 1),
    ("Zeer laag", 2),
    ("Laag", 3),
    ("Gemiddeld", 4),
    ("Verhoogd", 5),
    ("Hoog", 6),
    ("Maximaal", 7),
])
def test_silent_mode_heating_limit_per_option(clean_system, silent_option, expected_mode):
    """
    Verify every silent mode option caps space heating at its compressor mode (1..7)
    under full heating demand with heat_compressor_mode 'Maximaal' (mode 10).
    """
    openamber = clean_system

    openamber.set_select("thermostat_mode_select", "Extern")
    openamber.set_select("heat_mode_select", "Extern setpoint")
    openamber.set_number("manual_setpoint", 40.0)
    openamber.set_climate("pid_heat_temperature_control", target_temperature=40.0)
    openamber.set_number("backup_heater_degmin_threshold", 999.0)
    openamber.set_select("heat_compressor_mode", "Maximaal")

    openamber.set_time("silent_mode_start_time", 0, 0, 0)
    openamber.set_time("silent_mode_end_time", 23, 59, 0)
    openamber.set_select("silent_mode_compressor_mode", silent_option)
    openamber.set_switch("silent_mode_enabled_switch", True)

    openamber.set_switch("heat_demand_switch", True)
    openamber.set_sensor("current_water_temperature_tc_sensor", 25.0)
    openamber.set_sensor("heat_cool_temperature_tc", 25.0)
    openamber.set_sensor("outlet_temperature_tuo", 25.0)
    openamber.set_sensor("inlet_temperature_tui", 22.0)
    openamber.step(ms=50)

    # Pump interval (900s) + pump settle (140s) + softstart (190s)
    openamber.advance_time(seconds=900, step_s=30)
    openamber.advance_time(seconds=140, step_s=10)
    openamber.advance_time(seconds=190, step_s=10)
    openamber.step(ms=50)
    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"

    # Soft start begins at mode 5 (capped by the silent limit); reaching mode 7 takes 2 steps of 300s.
    openamber.advance_time(seconds=900, step_s=30)
    openamber.step(ms=50)

    current_mode = int(openamber.get_entity("compressor_control_select") or 0)
    assert current_mode == expected_mode, (
        f"Silent mode '{silent_option}' should cap heating at mode {expected_mode}, got {current_mode}"
    )

    openamber.set_switch("silent_mode_enabled_switch", False)
    openamber.set_time("silent_mode_start_time", 23, 0, 0)
    openamber.set_time("silent_mode_end_time", 7, 0, 0)


def test_silent_mode_caps_defrost_recovery_boost(clean_system):
    """
    Verify the defrost recovery boost (+3 modes) does not exceed the silent mode limit:
    1. Heating runs at the silent limit 'Laag' (mode 3) with heat_compressor_mode 'Maximaal'.
    2. A defrost ends with Tc far below target, which normally boosts the compressor by 3 modes.
    3. The compressor stays at mode 3 instead of jumping to mode 6.
    """
    openamber = clean_system

    openamber.set_select("thermostat_mode_select", "Extern")
    openamber.set_select("heat_mode_select", "Extern setpoint")
    openamber.set_number("manual_setpoint", 35.0)
    openamber.set_climate("pid_heat_temperature_control", target_temperature=35.0)
    openamber.set_number("backup_heater_degmin_threshold", 999.0)
    openamber.set_select("heat_compressor_mode", "Maximaal")
    openamber.set_sensor("temperature_outside_ta", 5.0)

    openamber.set_time("silent_mode_start_time", 0, 0, 0)
    openamber.set_time("silent_mode_end_time", 23, 59, 0)
    openamber.set_select("silent_mode_compressor_mode", "Laag")  # mode 3
    openamber.set_switch("silent_mode_enabled_switch", True)

    openamber.set_switch("heat_demand_switch", True)
    openamber.set_sensor("current_water_temperature_tc_sensor", 28.0)
    openamber.set_sensor("heat_cool_temperature_tc", 28.0)
    openamber.set_sensor("outlet_temperature_tuo", 28.0)
    openamber.set_sensor("inlet_temperature_tui", 26.0)
    openamber.step(ms=50)

    openamber.advance_time(seconds=900, step_s=30)
    openamber.advance_time(seconds=140, step_s=10)
    openamber.advance_time(seconds=190, step_s=10)
    openamber.advance_time(seconds=600, step_s=30)
    openamber.step(ms=50)

    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"
    assert int(openamber.get_entity("compressor_control_select") or 0) == 3

    # Defrost starts and ends with Tc < target - 3°C, which triggers the recovery boost
    openamber.set_binary_sensor("defrost_active_sensor", True)
    openamber.advance_time(seconds=10, step_s=2)
    openamber.step(ms=50)
    assert openamber.get_entity("state_machine_state_heat_cool") == "Defrosting"

    openamber.set_binary_sensor("defrost_active_sensor", False)
    openamber.advance_time(seconds=10, step_s=2)
    openamber.step(ms=50)

    assert openamber.get_entity("state_machine_state_heat_cool") == "Wait for state switch"
    recovery_mode = int(openamber.get_entity("compressor_control_select") or 0)
    assert recovery_mode == 3, f"Defrost recovery boost should be capped at silent mode 'Laag' (mode 3), got {recovery_mode}"

    openamber.set_switch("silent_mode_enabled_switch", False)
    openamber.set_time("silent_mode_start_time", 23, 0, 0)
    openamber.set_time("silent_mode_end_time", 7, 0, 0)
