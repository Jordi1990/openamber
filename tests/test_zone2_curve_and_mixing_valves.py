"""Automated tests for Zone 2 Heating Curve and Mixing Valve settings.

Covers settings:
1. Zone 2 Heating Curve (Stooklijn zone 2):
   - heat_curve_z2_m10 (-10°C anchor)
   - heat_curve_z2_0 (0°C anchor)
   - heat_curve_z2_p5 (+5°C anchor)
   - heat_curve_z2_p10 (+10°C anchor)
   - heat_curve_z2_p15 (+15°C anchor)
"""

import pytest


def test_zone2_heating_curve_anchors_and_interpolation(clean_system):
    """
    Verify Zone 2 Heating Curve (Stooklijn Zone 2):
    1. Set anchor points:
       - Ta = -10°C: 38.0°C
       - Ta =   0°C: 32.0°C
       - Ta =  +5°C: 28.0°C
       - Ta = +10°C: 25.0°C
       - Ta = +15°C: 22.0°C
    2. Test discrete points and outer clamping.
    3. Test linear interpolation at midpoint Ta = 2.5°C -> (32.0 + 28.0)/2 = 30.0°C.
    """
    openamber = clean_system

    openamber.set_number("heat_curve_z2_m10", 38.0)
    openamber.set_number("heat_curve_z2_0", 32.0)
    openamber.set_number("heat_curve_z2_p5", 28.0)
    openamber.set_number("heat_curve_z2_p10", 25.0)
    openamber.set_number("heat_curve_z2_p15", 22.0)
    openamber.step(ms=50)

    test_points = [
        (18.0, 22.0),   # Ta >= 15°C -> clamped to p15 (22.0)
        (15.0, 22.0),   # Ta = 15°C -> 22.0
        (10.0, 25.0),   # Ta = 10°C -> 25.0
        (5.0, 28.0),    # Ta = 5°C  -> 28.0
        (2.5, 30.0),    # Ta = 2.5°C -> lerp(2.5, 5, 28, 0, 32) = 30.0
        (0.0, 32.0),    # Ta = 0°C  -> 32.0
        (-5.0, 35.0),   # Ta = -5°C -> lerp(-5, 0, 32, -10, 38) = 35.0
        (-10.0, 38.0),  # Ta = -10°C -> 38.0
        (-15.0, 38.0),  # Ta <= -10°C -> clamped to m10 (38.0)
    ]

    for ta, expected_sp in test_points:
        openamber.set_sensor("temperature_outside_ta", ta)
        openamber.advance_time(seconds=6, step_s=1)
        openamber.step(ms=100)

        calculated = float(openamber.get_entity("heat_curve_z2_calculated_setpoint") or 0)
        assert calculated == pytest.approx(expected_sp, 0.2), (
            f"At Ta={ta}°C, expected Zone 2 setpoint {expected_sp}°C, got {calculated}°C"
        )


def test_two_zones_enabled_highest_setpoint_selection(clean_system):
    """
    Verify that when 2 zones are enabled, the highest setpoint among them is used
    as current_setpoint and forwarded to the heat temperature control climate.
    """
    openamber = clean_system

    openamber.set_select("heat_mode_select", "Stooklijn")
    openamber.set_switch("mixing_valve_zone1_enabled", True)
    openamber.set_switch("mixing_valve_zone2_enabled", True)

    # Set outside temp to 7°C (between 5°C and 10°C)
    openamber.set_sensor("temperature_outside_ta", 7.0)

    # Case 1: Zone 1 setpoint is higher (35°C) than Zone 2 (28°C)
    openamber.set_number("heat_curve_m10", 35.0)
    openamber.set_number("heat_curve_0", 35.0)
    openamber.set_number("heat_curve_p5", 35.0)
    openamber.set_number("heat_curve_p10", 35.0)
    openamber.set_number("heat_curve_p15", 35.0)

    openamber.set_number("heat_curve_z2_m10", 28.0)
    openamber.set_number("heat_curve_z2_0", 28.0)
    openamber.set_number("heat_curve_z2_p5", 28.0)
    openamber.set_number("heat_curve_z2_p10", 28.0)
    openamber.set_number("heat_curve_z2_p15", 28.0)

    openamber.advance_time(seconds=6, step_s=1)
    openamber.step(ms=100)

    sp_z1 = float(openamber.get_entity("heat_curve_calculated_setpoint") or 0)
    sp_z2 = float(openamber.get_entity("heat_curve_z2_calculated_setpoint") or 0)
    current_sp = float(openamber.get_entity("current_setpoint") or 0)

    assert sp_z1 == pytest.approx(35.0, 0.2)
    assert sp_z2 == pytest.approx(28.0, 0.2)
    assert current_sp == pytest.approx(35.0, 0.2), f"Expected Zone 1 setpoint 35.0°C to be selected, got {current_sp}°C"

    climate_info = openamber.get_entity("pid_heat_temperature_control")
    assert climate_info.get("target_temperature") == pytest.approx(35.0, 0.2)

    # Case 2: Zone 2 setpoint is higher (33°C) than Zone 1 (25°C)
    openamber.set_number("heat_curve_p5", 25.0)
    openamber.set_number("heat_curve_p10", 25.0)
    openamber.set_number("heat_curve_z2_p5", 33.0)
    openamber.set_number("heat_curve_z2_p10", 33.0)

    openamber.advance_time(seconds=6, step_s=1)
    openamber.step(ms=100)

    sp_z1 = float(openamber.get_entity("heat_curve_calculated_setpoint") or 0)
    sp_z2 = float(openamber.get_entity("heat_curve_z2_calculated_setpoint") or 0)
    current_sp = float(openamber.get_entity("current_setpoint") or 0)

    assert sp_z1 == pytest.approx(25.0, 0.2)
    assert sp_z2 == pytest.approx(33.0, 0.2)
    assert current_sp == pytest.approx(33.0, 0.2), f"Expected Zone 2 setpoint 33.0°C to be selected, got {current_sp}°C"

    climate_info = openamber.get_entity("pid_heat_temperature_control")
    assert climate_info.get("target_temperature") == pytest.approx(33.0, 0.2)

    # Case 3: Only Zone 1 enabled -> uses Zone 1 setpoint even if Zone 2 is higher
    openamber.set_switch("mixing_valve_zone1_enabled", True)
    openamber.set_switch("mixing_valve_zone2_enabled", False)
    openamber.advance_time(seconds=6, step_s=1)
    openamber.step(ms=100)

    current_sp = float(openamber.get_entity("current_setpoint") or 0)
    assert current_sp == pytest.approx(25.0, 0.2), f"Expected only Zone 1 (25.0°C), got {current_sp}°C"

    # Case 4: Only Zone 2 enabled -> uses Zone 2 setpoint even if Zone 1 is higher
    openamber.set_number("heat_curve_p5", 36.0)
    openamber.set_number("heat_curve_p10", 36.0)
    openamber.set_switch("mixing_valve_zone1_enabled", False)
    openamber.set_switch("mixing_valve_zone2_enabled", True)
    openamber.advance_time(seconds=6, step_s=1)
    openamber.step(ms=100)

    current_sp = float(openamber.get_entity("current_setpoint") or 0)
    assert current_sp == pytest.approx(33.0, 0.2), f"Expected only Zone 2 (33.0°C), got {current_sp}°C"

    # Cleanup
    openamber.set_switch("mixing_valve_zone1_enabled", False)
    openamber.set_switch("mixing_valve_zone2_enabled", False)
    openamber.step(ms=50)


def _start_heating_and_reach_compressor_running(openamber):
    openamber.set_select("thermostat_mode_select", "Extern")
    openamber.set_select("heat_mode_select", "Extern setpoint")
    openamber.set_number("manual_setpoint", 35.0)
    openamber.set_climate("pid_heat_temperature_control", target_temperature=35.0)
    openamber.set_select("heat_compressor_mode", "Maximaal")
    openamber.set_switch("heat_demand_switch", True)
    openamber.set_sensor("current_water_temperature_tc_sensor", 28.0)
    openamber.set_sensor("heat_cool_temperature_tc", 28.0)
    openamber.set_sensor("outlet_temperature_tuo", 28.0)
    openamber.set_sensor("inlet_temperature_tui", 26.0)
    openamber.set_sensor("temperature_outside_ta", 7.0)
    openamber.step(ms=50)

    openamber.advance_time(seconds=900, step_s=30)
    openamber.advance_time(seconds=140, step_s=10)
    openamber.advance_time(seconds=190, step_s=10)
    openamber.step(ms=50)

    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"


def test_mixing_valve_temperature_control_tv1_tv2_and_anti_jitter(clean_system):
    """
    Verify mixing valve control based on Tv1 / Tv2:
    - Valve opens more when temperature is below target.
    - Valve closes more when temperature is above target.
    - Valve keeps steady and does not jitter within the deadband.
    """
    openamber = clean_system

    # Enable mixing valves for both zones
    openamber.set_switch("mixing_valve_zone1_enabled", True)
    openamber.set_switch("mixing_valve_zone2_enabled", True)

    # Set explicit curve anchors so target is 28.0°C for both zones
    openamber.set_number("heat_curve_m10", 28.0)
    openamber.set_number("heat_curve_0", 28.0)
    openamber.set_number("heat_curve_p5", 28.0)
    openamber.set_number("heat_curve_p10", 28.0)
    openamber.set_number("heat_curve_p15", 28.0)
    openamber.set_number("heat_curve_z2_m10", 28.0)
    openamber.set_number("heat_curve_z2_0", 28.0)
    openamber.set_number("heat_curve_z2_p5", 28.0)
    openamber.set_number("heat_curve_z2_p10", 28.0)
    openamber.set_number("heat_curve_z2_p15", 28.0)

    # Set initial temperatures matching target 28°C
    openamber.set_sensor("heat_cool_temperature_tv1", 28.0)
    openamber.set_sensor("heat_cool_temperature_tv2", 28.0)
    openamber.step(ms=50)

    _start_heating_and_reach_compressor_running(openamber)

    # 1. Zone 1: Lower Tv1 well below target (20°C vs target 28°C) -> expect valve 1 to open
    openamber.set_sensor("heat_cool_temperature_tv1", 20.0)
    openamber.advance_time(seconds=5, step_s=1)
    openamber.step(ms=100)
    val1_low = int(openamber.get_entity("mixing_valve_zone1_output"))
    assert val1_low > 50, f"Expected valve 1 to open when Tv1 is low, got {val1_low}%"

    # 2. Zone 1: Raise Tv1 well above target (36°C vs target 28°C) -> expect valve 1 to close
    openamber.set_sensor("heat_cool_temperature_tv1", 36.0)
    openamber.advance_time(seconds=5, step_s=1)
    openamber.step(ms=100)
    val1_high = int(openamber.get_entity("mixing_valve_zone1_output"))
    assert val1_high < 50, f"Expected valve 1 to close when Tv1 is high, got {val1_high}%"

    # 3. Zone 1: Bring Tv1 to target -> verify deadband and no jitter
    target_z1 = float(openamber.get_entity("pid_mixing_valve_zone1").get("target_temperature"))
    openamber.set_sensor("heat_cool_temperature_tv1", target_z1)
    openamber.advance_time(seconds=10, step_s=1)
    openamber.step(ms=100)
    val1_settled = int(openamber.get_entity("mixing_valve_zone1_output"))

    # Small fluctuation within 1°C deadband (+0.4°C) -> steady, no jitter
    openamber.set_sensor("heat_cool_temperature_tv1", target_z1 + 0.4)
    openamber.advance_time(seconds=5, step_s=1)
    openamber.step(ms=100)
    val1_fluc1 = int(openamber.get_entity("mixing_valve_zone1_output"))
    assert val1_fluc1 == val1_settled, f"Valve 1 jittered on +0.4°C: was {val1_settled}%, now {val1_fluc1}%"

    # Small fluctuation within 1°C deadband (-0.4°C) -> steady, no jitter
    openamber.set_sensor("heat_cool_temperature_tv1", target_z1 - 0.4)
    openamber.advance_time(seconds=5, step_s=1)
    openamber.step(ms=100)
    val1_fluc2 = int(openamber.get_entity("mixing_valve_zone1_output"))
    assert val1_fluc2 == val1_settled, f"Valve 1 jittered on -0.4°C: was {val1_settled}%, now {val1_fluc2}%"

    # 4. Zone 2: Lower Tv2 well below target -> expect valve 2 to open
    openamber.set_sensor("heat_cool_temperature_tv2", 18.0)
    openamber.advance_time(seconds=5, step_s=1)
    openamber.step(ms=100)
    val2_low = int(openamber.get_entity("mixing_valve_zone2_output"))
    assert val2_low > 50, f"Expected valve 2 to open when Tv2 is low, got {val2_low}%"

    # 5. Zone 2: Raise Tv2 well above target -> expect valve 2 to close
    openamber.set_sensor("heat_cool_temperature_tv2", 38.0)
    openamber.advance_time(seconds=5, step_s=1)
    openamber.step(ms=100)
    val2_high = int(openamber.get_entity("mixing_valve_zone2_output"))
    assert val2_high < 50, f"Expected valve 2 to close when Tv2 is high, got {val2_high}%"

    # 6. Zone 2: Bring Tv2 to target -> verify deadband and no jitter
    target_z2 = float(openamber.get_entity("pid_mixing_valve_zone2").get("target_temperature"))
    openamber.set_sensor("heat_cool_temperature_tv2", target_z2)
    openamber.advance_time(seconds=10, step_s=1)
    openamber.step(ms=100)
    val2_settled = int(openamber.get_entity("mixing_valve_zone2_output"))

    # Small fluctuation within deadband (+0.4°C) -> steady, no jitter (no proportional spike)
    openamber.set_sensor("heat_cool_temperature_tv2", target_z2 + 0.4)
    openamber.advance_time(seconds=5, step_s=1)
    openamber.step(ms=100)
    val2_fluc1 = int(openamber.get_entity("mixing_valve_zone2_output"))
    assert abs(val2_fluc1 - val2_settled) <= 2, f"Valve 2 jittered on +0.4°C: was {val2_settled}%, now {val2_fluc1}%"

    # Small fluctuation within deadband (-0.4°C) -> steady, no jitter (no proportional spike)
    openamber.set_sensor("heat_cool_temperature_tv2", target_z2 - 0.4)
    openamber.advance_time(seconds=5, step_s=1)
    openamber.step(ms=100)
    val2_fluc2 = int(openamber.get_entity("mixing_valve_zone2_output"))
    assert abs(val2_fluc2 - val2_settled) <= 2, f"Valve 2 jittered on -0.4°C: was {val2_settled}%, now {val2_fluc2}%"


def test_mixing_valves_clamped_within_min_and_max_positions(clean_system):
    """
    Verify that mixing valves are strictly clamped within configured min and max positions:
    1. Set Zone 1 min=20%, max=75%
    2. Set Zone 2 min=15%, max=85%
    3. Under extreme cold demand (low Tv1/Tv2), valve positions cannot exceed max_position.
    4. Under extreme hot demand (high Tv1/Tv2), valve positions cannot drop below min_position.
    5. When space heating stops and enters Idle, valves are closed to their min_position.
    """
    openamber = clean_system

    openamber.set_switch("mixing_valve_zone1_enabled", True)
    openamber.set_switch("mixing_valve_zone2_enabled", True)

    # Configure custom min and max limits
    openamber.set_number("mixing_valve_zone1_min_position", 20.0)
    openamber.set_number("mixing_valve_zone1_max_position", 75.0)
    openamber.set_number("mixing_valve_zone2_min_position", 15.0)
    openamber.set_number("mixing_valve_zone2_max_position", 85.0)
    openamber.step(ms=50)

    _start_heating_and_reach_compressor_running(openamber)

    # Demand full heat: very low Tv1 and Tv2 (10°C) -> would demand 100%
    openamber.set_sensor("heat_cool_temperature_tv1", 10.0)
    openamber.set_sensor("heat_cool_temperature_tv2", 10.0)
    openamber.advance_time(seconds=5, step_s=1)
    openamber.step(ms=100)

    val1 = int(openamber.get_entity("mixing_valve_zone1_output"))
    val2 = int(openamber.get_entity("mixing_valve_zone2_output"))
    assert val1 == 75, f"Expected Zone 1 clamped to max 75%, got {val1}%"
    assert val2 == 85, f"Expected Zone 2 clamped to max 85%, got {val2}%"

    # Demand minimum heat: very high Tv1 and Tv2 (50°C) -> would demand 0%
    openamber.set_sensor("heat_cool_temperature_tv1", 50.0)
    openamber.set_sensor("heat_cool_temperature_tv2", 50.0)
    openamber.advance_time(seconds=5, step_s=1)
    openamber.step(ms=100)

    val1 = int(openamber.get_entity("mixing_valve_zone1_output"))
    val2 = int(openamber.get_entity("mixing_valve_zone2_output"))
    assert val1 == 20, f"Expected Zone 1 clamped to min 20%, got {val1}%"
    assert val2 == 15, f"Expected Zone 2 clamped to min 15%, got {val2}%"

    # Turn off heating demand -> controller transitions to Idle and calls CloseMixingValves()
    openamber.set_switch("heat_demand_switch", False)
    # Stop delay: advance time through compressor shutdown and pump stop
    openamber.advance_time(seconds=30, step_s=5)
    openamber.step(ms=100)

    val1_idle = int(openamber.get_entity("mixing_valve_zone1_output"))
    val2_idle = int(openamber.get_entity("mixing_valve_zone2_output"))
    assert val1_idle == 20, f"In idle, Zone 1 valve must close to min_position (20%), got {val1_idle}%"
    assert val2_idle == 15, f"In idle, Zone 2 valve must close to min_position (15%), got {val2_idle}%"


def test_mixing_valves_held_in_previous_position_during_defrost(clean_system):
    """
    Verify that during defrosting the mixing valves are kept in their previous position:
    1. Heat pump runs in space heating mode (Compressor running).
    2. Valves are active at specific positions (e.g. Zone 1 = 75%, Zone 2 = 15%).
    3. Defrost starts (defrost_active_sensor = True -> state transitions to Defrosting).
    4. Sensor temperatures change radically during defrosting.
    5. Valves do not update and remain held at their pre-defrost positions.
    6. Defrost ends (defrost_active_sensor = False -> returns to Compressor running).
    7. Valves resume active regulation.
    """
    openamber = clean_system

    openamber.set_switch("mixing_valve_zone1_enabled", True)
    openamber.set_switch("mixing_valve_zone2_enabled", True)
    openamber.set_number("mixing_valve_zone1_min_position", 20.0)
    openamber.set_number("mixing_valve_zone1_max_position", 75.0)
    openamber.set_number("mixing_valve_zone2_min_position", 15.0)
    openamber.set_number("mixing_valve_zone2_max_position", 85.0)

    # Establish initial conditions: Tv1 low (demand 75%), Tv2 high (demand 15%)
    openamber.set_sensor("heat_cool_temperature_tv1", 18.0)
    openamber.set_sensor("heat_cool_temperature_tv2", 42.0)
    openamber.step(ms=50)

    _start_heating_and_reach_compressor_running(openamber)

    openamber.advance_time(seconds=5, step_s=1)
    openamber.step(ms=100)

    val1_before = int(openamber.get_entity("mixing_valve_zone1_output"))
    val2_before = int(openamber.get_entity("mixing_valve_zone2_output"))
    assert val1_before == 75, f"Pre-defrost Valve 1 should be 75%, got {val1_before}%"
    assert val2_before == 15, f"Pre-defrost Valve 2 should be 15%, got {val2_before}%"

    # Step 3: Trigger defrost cycle
    openamber.set_binary_sensor("defrost_active_sensor", True)
    openamber.advance_time(seconds=10, step_s=2)
    openamber.step(ms=50)

    assert openamber.get_entity("state_machine_state_heat_cool") == "Defrosting", (
        f"Expected 'Defrosting' state, got {openamber.get_entity('state_machine_state_heat_cool')}"
    )

    # Invert temperatures during defrost: Tv1 is now hot (45°C), Tv2 is now cold (15°C)
    # Under normal heating, this would invert valve positions (Valve 1 -> 20%, Valve 2 -> 85%)
    openamber.set_sensor("heat_cool_temperature_tv1", 45.0)
    openamber.set_sensor("heat_cool_temperature_tv2", 15.0)
    openamber.advance_time(seconds=30, step_s=5)
    openamber.step(ms=50)

    assert openamber.get_entity("state_machine_state_heat_cool") == "Defrosting"

    # Verify valves did NOT move during defrosting
    val1_during = int(openamber.get_entity("mixing_valve_zone1_output"))
    val2_during = int(openamber.get_entity("mixing_valve_zone2_output"))
    assert val1_during == val1_before, f"Valve 1 must remain frozen at {val1_before}% during defrost, got {val1_during}%"
    assert val2_during == val2_before, f"Valve 2 must remain frozen at {val2_before}% during defrost, got {val2_during}%"

    # Step 4: End defrost cycle
    openamber.set_binary_sensor("defrost_active_sensor", False)
    openamber.advance_time(seconds=10, step_s=2)
    openamber.step(ms=50)

    assert openamber.get_entity("state_machine_state_heat_cool") == "Wait for state switch"

    # Advance through defrost settle period (300s)
    openamber.advance_time(seconds=310, step_s=15)
    openamber.step(ms=50)

    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"

    # Now that defrost is finished, valves must resume active modulation and reflect the inverted temperatures
    val1_after = int(openamber.get_entity("mixing_valve_zone1_output"))
    val2_after = int(openamber.get_entity("mixing_valve_zone2_output"))
    assert val1_after == 20, f"Post-defrost Valve 1 should now reflect Tv1=45°C (closed to 20%), got {val1_after}%"
    assert val2_after == 85, f"Post-defrost Valve 2 should now reflect Tv2=15°C (opened to 85%), got {val2_after}%"
