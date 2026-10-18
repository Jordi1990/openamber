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
        assert calculated == pytest.approx(expected_sp, abs=0.2), (
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

    assert sp_z1 == pytest.approx(35.0, abs=0.2)
    assert sp_z2 == pytest.approx(28.0, abs=0.2)
    assert current_sp == pytest.approx(35.0, abs=0.2), f"Expected Zone 1 setpoint 35.0°C to be selected, got {current_sp}°C"

    climate_info = openamber.get_entity("pid_heat_temperature_control")
    assert climate_info.get("target_temperature") == pytest.approx(35.0, abs=0.2)

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

    assert sp_z1 == pytest.approx(25.0, abs=0.2)
    assert sp_z2 == pytest.approx(33.0, abs=0.2)
    assert current_sp == pytest.approx(33.0, abs=0.2), f"Expected Zone 2 setpoint 33.0°C to be selected, got {current_sp}°C"

    climate_info = openamber.get_entity("pid_heat_temperature_control")
    assert climate_info.get("target_temperature") == pytest.approx(33.0, abs=0.2)

    # Case 3: Only Zone 1 enabled -> uses Zone 1 setpoint even if Zone 2 is higher
    openamber.set_switch("mixing_valve_zone1_enabled", True)
    openamber.set_switch("mixing_valve_zone2_enabled", False)
    openamber.advance_time(seconds=6, step_s=1)
    openamber.step(ms=100)

    current_sp = float(openamber.get_entity("current_setpoint") or 0)
    assert current_sp == pytest.approx(25.0, abs=0.2), f"Expected only Zone 1 (25.0°C), got {current_sp}°C"

    # Case 4: Only Zone 2 enabled -> uses Zone 2 setpoint even if Zone 1 is higher
    openamber.set_number("heat_curve_p5", 36.0)
    openamber.set_number("heat_curve_p10", 36.0)
    openamber.set_switch("mixing_valve_zone1_enabled", False)
    openamber.set_switch("mixing_valve_zone2_enabled", True)
    openamber.advance_time(seconds=6, step_s=1)
    openamber.step(ms=100)

    current_sp = float(openamber.get_entity("current_setpoint") or 0)
    assert current_sp == pytest.approx(33.0, abs=0.2), f"Expected only Zone 2 (33.0°C), got {current_sp}°C"

    # Cleanup
    openamber.set_switch("mixing_valve_zone1_enabled", False)
    openamber.set_switch("mixing_valve_zone2_enabled", False)
    openamber.step(ms=50)


def _start_heating_and_reach_compressor_running(openamber, demand_z1=True, demand_z2=False, heat_mode=None):
    openamber.set_select("thermostat_mode_select", "Extern")
    if heat_mode:
        openamber.set_select("heat_mode_select", heat_mode)
    openamber.set_select("heat_compressor_mode", "Maximaal")
    openamber.set_number("compressor_start_delta_heating", 2.0)
    openamber.set_number("backup_heater_degmin_threshold", 9999.0)

    # Ensure minimum compressor off-time (120s) has elapsed before starting demand
    openamber.advance_time(seconds=130, step_s=10)
    openamber.step(ms=50)

    # Enable heat demand for requested zones
    openamber.set_switch("heat_demand_switch", demand_z1)
    openamber.set_switch("heat_demand_zone_2_switch", demand_z2)
    openamber.step(ms=50)

    # Settle setpoint calculation
    openamber.advance_time(seconds=6, step_s=1)
    openamber.step(ms=100)
    target = float(openamber.get_entity("current_setpoint") or 35.0)
    tc_init = target - 3.0

    openamber.set_sensor("current_water_temperature_tc_sensor", tc_init)
    openamber.set_sensor("heat_cool_temperature_tc", tc_init)
    openamber.set_sensor("outlet_temperature_tuo", tc_init)
    openamber.set_sensor("inlet_temperature_tui", tc_init - 2.0)
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

    # Both zones demand heat
    # Phase 1: Zone 1 has lower target (28°C) than Zone 2 (35°C) -> Zone 1 modulates via PID
    openamber.set_select("heat_mode_select", "Stooklijn")
    openamber.set_number("heat_curve_m10", 28.0)
    openamber.set_number("heat_curve_0", 28.0)
    openamber.set_number("heat_curve_p5", 28.0)
    openamber.set_number("heat_curve_p10", 28.0)
    openamber.set_number("heat_curve_p15", 28.0)

    openamber.set_number("heat_curve_z2_m10", 35.0)
    openamber.set_number("heat_curve_z2_0", 35.0)
    openamber.set_number("heat_curve_z2_p5", 35.0)
    openamber.set_number("heat_curve_z2_p10", 35.0)
    openamber.set_number("heat_curve_z2_p15", 35.0)

    # Set initial temperatures matching targets
    openamber.set_sensor("heat_cool_temperature_tv1", 28.0)
    openamber.set_sensor("heat_cool_temperature_tv2", 35.0)
    openamber.step(ms=50)

    _start_heating_and_reach_compressor_running(openamber, demand_z1=True, demand_z2=True, heat_mode="Stooklijn")

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

    # Phase 2: Invert targets so Zone 2 has lower target (28°C) than Zone 1 (35°C) -> Zone 2 modulates via PID
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

    # 4. Zone 2: Lower Tv2 well below target (18°C vs target 28°C) -> expect valve 2 to open
    openamber.set_sensor("heat_cool_temperature_tv2", 18.0)
    openamber.advance_time(seconds=5, step_s=1)
    openamber.step(ms=100)
    val2_low = int(openamber.get_entity("mixing_valve_zone2_output"))
    assert val2_low > 50, f"Expected valve 2 to open when Tv2 is low, got {val2_low}%"

    # 5. Zone 2: Raise Tv2 well above target (38°C vs target 28°C) -> expect valve 2 to close
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

    # Both zones demand heat: Zone 1 target 25°C < Zone 2 target 35°C -> Zone 1 mixes, Zone 2 open to max (85%)
    openamber.set_select("heat_mode_select", "Stooklijn")
    openamber.set_number("heat_curve_m10", 25.0)
    openamber.set_number("heat_curve_0", 25.0)
    openamber.set_number("heat_curve_p5", 25.0)
    openamber.set_number("heat_curve_p10", 25.0)
    openamber.set_number("heat_curve_p15", 25.0)
    openamber.set_number("heat_curve_z2_m10", 35.0)
    openamber.set_number("heat_curve_z2_0", 35.0)
    openamber.set_number("heat_curve_z2_p5", 35.0)
    openamber.set_number("heat_curve_z2_p10", 35.0)
    openamber.set_number("heat_curve_z2_p15", 35.0)

    _start_heating_and_reach_compressor_running(openamber, demand_z1=True, demand_z2=True)

    # 1. Extreme cold Tv1 (10°C): Zone 1 cannot exceed max 75%, Zone 2 is open at max 85%
    openamber.set_sensor("heat_cool_temperature_tv1", 10.0)
    openamber.set_sensor("heat_cool_temperature_tv2", 10.0)
    openamber.advance_time(seconds=5, step_s=1)
    openamber.step(ms=100)

    val1 = int(openamber.get_entity("mixing_valve_zone1_output"))
    val2 = int(openamber.get_entity("mixing_valve_zone2_output"))
    assert val1 == 75, f"Expected Zone 1 clamped to max 75%, got {val1}%"
    assert val2 == 85, f"Expected Zone 2 clamped to max 85%, got {val2}%"

    # 2. Extreme hot Tv1 (50°C): Zone 1 cannot drop below min 20%
    openamber.set_sensor("heat_cool_temperature_tv1", 50.0)
    openamber.advance_time(seconds=5, step_s=1)
    openamber.step(ms=100)

    val1_min = int(openamber.get_entity("mixing_valve_zone1_output"))
    assert val1_min == 20, f"Expected Zone 1 clamped to min 20%, got {val1_min}%"

    # 3. Swap: Zone 2 target 25°C < Zone 1 target 35°C -> Zone 2 mixes, Zone 1 is open to max 75%
    openamber.set_number("heat_curve_m10", 35.0)
    openamber.set_number("heat_curve_0", 35.0)
    openamber.set_number("heat_curve_p5", 35.0)
    openamber.set_number("heat_curve_p10", 35.0)
    openamber.set_number("heat_curve_p15", 35.0)
    openamber.set_number("heat_curve_z2_m10", 25.0)
    openamber.set_number("heat_curve_z2_0", 25.0)
    openamber.set_number("heat_curve_z2_p5", 25.0)
    openamber.set_number("heat_curve_z2_p10", 25.0)
    openamber.set_number("heat_curve_z2_p15", 25.0)
    openamber.advance_time(seconds=6, step_s=1)
    openamber.step(ms=100)

    # Extreme hot Tv2 (50°C): Zone 2 cannot drop below min 15%, Zone 1 open at max 75%
    openamber.set_sensor("heat_cool_temperature_tv2", 50.0)
    openamber.advance_time(seconds=5, step_s=1)
    openamber.step(ms=100)

    val2_min = int(openamber.get_entity("mixing_valve_zone2_output"))
    val1_open = int(openamber.get_entity("mixing_valve_zone1_output"))
    assert val2_min == 15, f"Expected Zone 2 clamped to min 15%, got {val2_min}%"
    assert val1_open == 75, f"Expected Zone 1 open at max 75%, got {val1_open}%"

    # Turn off heating demand -> controller transitions to Idle and calls CloseMixingValves()
    openamber.set_switch("heat_demand_switch", False)
    openamber.set_switch("heat_demand_zone_2_switch", False)
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
    2. Valves are active at specific positions (e.g. Zone 1 = 75%, Zone 2 = 85%).
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

    # Establish initial conditions: Zone 1 target 28°C < Zone 2 target 35°C
    openamber.set_select("heat_mode_select", "Stooklijn")
    openamber.set_number("heat_curve_m10", 28.0)
    openamber.set_number("heat_curve_0", 28.0)
    openamber.set_number("heat_curve_p5", 28.0)
    openamber.set_number("heat_curve_p10", 28.0)
    openamber.set_number("heat_curve_p15", 28.0)
    openamber.set_number("heat_curve_z2_m10", 35.0)
    openamber.set_number("heat_curve_z2_0", 35.0)
    openamber.set_number("heat_curve_z2_p5", 35.0)
    openamber.set_number("heat_curve_z2_p10", 35.0)
    openamber.set_number("heat_curve_z2_p15", 35.0)

    # Tv1 low (demand 75%), Tv2 matched (Zone 2 open at max 85%)
    openamber.set_sensor("heat_cool_temperature_tv1", 18.0)
    openamber.set_sensor("heat_cool_temperature_tv2", 35.0)
    openamber.step(ms=50)

    _start_heating_and_reach_compressor_running(openamber, demand_z1=True, demand_z2=True)

    openamber.advance_time(seconds=5, step_s=1)
    openamber.step(ms=100)

    val1_before = int(openamber.get_entity("mixing_valve_zone1_output"))
    val2_before = int(openamber.get_entity("mixing_valve_zone2_output"))
    assert val1_before == 75, f"Pre-defrost Valve 1 should be 75%, got {val1_before}%"
    assert val2_before == 85, f"Pre-defrost Valve 2 should be 85%, got {val2_before}%"

    # Step 3: Trigger defrost cycle
    openamber.set_binary_sensor("defrost_active_sensor", True)
    openamber.advance_time(seconds=10, step_s=2)
    openamber.step(ms=50)

    assert openamber.get_entity("state_machine_state_heat_cool") == "Defrosting", (
        f"Expected 'Defrosting' state, got {openamber.get_entity('state_machine_state_heat_cool')}"
    )

    # Change temperatures during defrost: Tv1 is now hot (45°C), Tv2 is cold
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

    # Now that defrost is finished, Zone 1 resumes modulation and reflects Tv1=45°C (closed to min 20%)
    val1_after = int(openamber.get_entity("mixing_valve_zone1_output"))
    val2_after = int(openamber.get_entity("mixing_valve_zone2_output"))
    assert val1_after == 20, f"Post-defrost Valve 1 should now reflect Tv1=45°C (closed to 20%), got {val1_after}%"
    assert val2_after == 85, f"Post-defrost Valve 2 should remain at max (85%), got {val2_after}%"


def test_control_temperature_forced_to_tc_when_mixing_valves_enabled(clean_system):
    """
    Verify GetControlTemperature():
    - When mixing valves are enabled, controller strictly forces Tc as control temperature
      for compressor regulation, even if Tv1 temperature spikes high.
    - When Tc itself exceeds the target + stop_delta, compressor stops as expected.
    """
    openamber = clean_system

    # Enable mixing valve zone 1 and set control source to Tv1
    openamber.set_switch("mixing_valve_zone1_enabled", True)
    openamber.set_switch("mixing_valve_zone2_enabled", False)
    openamber.set_select("heat_cool_control_temperature_source_select", "CV-aanvoer zone 1 (Tv1)")
    openamber.set_number("compressor_stop_delta_heating", 2.0)
    openamber.set_number("manual_setpoint", 35.0)
    openamber.set_climate("pid_heat_temperature_control", target_temperature=35.0)
    openamber.step(ms=50)

    # Start compressor running (target = 35°C, stop delta = 2°C -> stop threshold = 37°C)
    _start_heating_and_reach_compressor_running(openamber, heat_mode="Extern setpoint")
    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"

    # Stop conditions are only evaluated after COMPRESSOR_MIN_ON_S (600s). Without passing it,
    # the "keeps running" check below would pass even if Tv1 were used as control temperature.
    openamber.advance_time(seconds=620, step_s=20)
    openamber.step(ms=50)
    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"

    # Spike Tv1 well above stop threshold (45°C >> 37°C) while keeping Tc at 30°C (< 37°C)
    openamber.set_sensor("heat_cool_temperature_tv1", 45.0)
    openamber.set_sensor("heat_cool_temperature_tc", 30.0)
    openamber.advance_time(seconds=10, step_s=1)
    openamber.step(ms=50)

    # Since mixing valve is enabled, controller must use Tc (30°C) and NOT stop the compressor
    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running", (
        "Compressor must keep running because GetControlTemperature() forces Tc (30°C), ignoring hot Tv1 (45°C)"
    )

    # Now raise Tc above stop threshold (38°C > 37°C)
    openamber.set_sensor("heat_cool_temperature_tc", 38.0)
    openamber.set_sensor("current_water_temperature_tc_sensor", 38.0)
    openamber.advance_time(seconds=10, step_s=1)
    openamber.step(ms=50)

    # Now compressor must stop because Tc exceeded target + stop_delta
    # Normal stop path: Compressor running -> Wait compressor stop -> Pump running (-> Wait pump stop).
    # 'Idle' is deliberately not accepted: that would be a safety stop, not the stop-delta path.
    assert openamber.get_entity("state_machine_state_heat_cool") in [
        "Wait compressor stop", "Pump running", "Wait pump stop"
    ], (
        "Compressor must stop when Tc exceeds target + stop_delta"
    )
    assert int(openamber.get_entity("compressor_control_select")) == 0



def test_mixing_valves_open_to_max_in_cooling_mode(clean_system):
    """
    Verify:
    1. mixing_valve_zone1_position and zone2_position sensors return NaN (None) when disabled.
    2. In cooling mode, enabled mixing valves are fully opened to their max_position.
    3. When cooling ends and system enters Idle, valves are closed to their min_position.
    """
    openamber = clean_system

    # Step 1: Disabled valves -> sensors report None
    openamber.set_switch("mixing_valve_zone1_enabled", False)
    openamber.set_switch("mixing_valve_zone2_enabled", False)
    openamber.advance_time(seconds=6, step_s=1)
    openamber.step(ms=50)

    assert openamber.get_entity("mixing_valve_zone1_position") is None, "Disabled MV Zone 1 sensor should be None"
    assert openamber.get_entity("mixing_valve_zone2_position") is None, "Disabled MV Zone 2 sensor should be None"

    # Step 2: Enable mixing valves with custom min/max positions
    openamber.set_switch("mixing_valve_zone1_enabled", True)
    openamber.set_switch("mixing_valve_zone2_enabled", True)
    openamber.set_number("mixing_valve_zone1_min_position", 20.0)
    openamber.set_number("mixing_valve_zone1_max_position", 80.0)
    openamber.set_number("mixing_valve_zone2_min_position", 10.0)
    openamber.set_number("mixing_valve_zone2_max_position", 95.0)

    # Configure cooling settings
    openamber.set_select("thermostat_mode_select", "Extern")
    openamber.set_select("cool_mode_select", "Intern setpoint")
    openamber.set_number("cooling_setpoint_number", 18.0)
    openamber.set_climate("pid_cool_temperature_control", target_temperature=18.0)
    openamber.set_number("compressor_start_delta_cooling", 1.0)
    openamber.set_number("compressor_stop_delta_cooling", 2.0)
    openamber.set_select("cool_compressor_mode", "Maximaal")

    # Initial warm water (23°C) to start cooling easily
    openamber.set_sensor("current_water_temperature_tc_sensor", 23.0)
    openamber.set_sensor("heat_cool_temperature_tc", 23.0)
    openamber.set_sensor("outlet_temperature_tuo", 23.0)
    openamber.set_sensor("inlet_temperature_tui", 24.0)
    openamber.set_sensor("temperature_outside_ta", 25.0)
    openamber.step(ms=50)

    # Advance past min off time
    openamber.advance_time(seconds=130, step_s=10)
    openamber.step(ms=50)

    # Trigger cool demand
    openamber.set_binary_sensor("external_cool_demand_wired", True)
    openamber.step(ms=100)
    assert openamber.get_entity("cool_demand_active_sensor") is True

    # Advance to compressor running: pump interval (900s) + pump settle (140s) + softstart (190s)
    openamber.advance_time(seconds=920, step_s=30)
    openamber.advance_time(seconds=140, step_s=10)
    openamber.advance_time(seconds=190, step_s=10)
    openamber.step(ms=50)

    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"

    # In active cooling, both enabled mixing valves must be opened to their configured max positions
    val1_cool = int(openamber.get_entity("mixing_valve_zone1_output"))
    val2_cool = int(openamber.get_entity("mixing_valve_zone2_output"))
    assert val1_cool == 80, f"During cooling, Zone 1 valve must open to max_position (80%), got {val1_cool}%"
    assert val2_cool == 95, f"During cooling, Zone 2 valve must open to max_position (95%), got {val2_cool}%"

    # Step 3: Stop cooling demand -> advance past min on time (600s) to allow shutdown
    openamber.set_binary_sensor("external_cool_demand_wired", False)
    openamber.advance_time(seconds=610, step_s=20)
    openamber.step(ms=100)

    val1_idle = int(openamber.get_entity("mixing_valve_zone1_output"))
    val2_idle = int(openamber.get_entity("mixing_valve_zone2_output"))
    assert val1_idle == 20, f"After cooling ends, Zone 1 valve must close to min_position (20%), got {val1_idle}%"
    assert val2_idle == 10, f"After cooling ends, Zone 2 valve must close to min_position (10%), got {val2_idle}%"


def test_flow_zone1_demands_heat_zone2_no_demand_zone1_opens_zone2_closes(clean_system):
    """
    Flow requirement:
    - Zone 1 demands heat, zone 2 does not demand heat.
    - Zone 1 has mixing enabled so valve can open completely.
    - If zone 2 has mixing the valve needs to be closed.
    - Heatpump Tc setpoint is determined by Zone 1 setpoint.
    """
    openamber = clean_system

    openamber.set_switch("mixing_valve_zone1_enabled", True)
    openamber.set_switch("mixing_valve_zone2_enabled", True)
    openamber.set_number("mixing_valve_zone1_min_position", 20.0)
    openamber.set_number("mixing_valve_zone1_max_position", 80.0)
    openamber.set_number("mixing_valve_zone2_min_position", 15.0)
    openamber.set_number("mixing_valve_zone2_max_position", 85.0)

    # Set curve anchors: Zone 1 = 30°C, Zone 2 = 45°C
    openamber.set_select("heat_mode_select", "Stooklijn")
    openamber.set_number("heat_curve_m10", 30.0)
    openamber.set_number("heat_curve_0", 30.0)
    openamber.set_number("heat_curve_p5", 30.0)
    openamber.set_number("heat_curve_p10", 30.0)
    openamber.set_number("heat_curve_p15", 30.0)

    openamber.set_number("heat_curve_z2_m10", 45.0)
    openamber.set_number("heat_curve_z2_0", 45.0)
    openamber.set_number("heat_curve_z2_p5", 45.0)
    openamber.set_number("heat_curve_z2_p10", 45.0)
    openamber.set_number("heat_curve_z2_p15", 45.0)
    openamber.step(ms=50)

    # Zone 1 demands heat, Zone 2 does NOT demand heat
    _start_heating_and_reach_compressor_running(openamber, demand_z1=True, demand_z2=False)

    openamber.advance_time(seconds=5, step_s=1)
    openamber.step(ms=100)

    assert openamber.get_entity("heat_demand_active_sensor_zone_1") is True
    assert openamber.get_entity("heat_demand_active_sensor_zone_2") is False

    # Tc setpoint must be Zone 1 setpoint (30°C), ignoring Zone 2 (45°C)
    current_sp = float(openamber.get_entity("current_setpoint") or 0)
    assert current_sp == pytest.approx(30.0, abs=0.2)

    # Zone 1 valve opens completely to max (80%)
    val1 = int(openamber.get_entity("mixing_valve_zone1_output"))
    assert val1 == 80, f"Zone 1 valve must open completely to max (80%), got {val1}%"

    # Zone 2 valve closes to min (15%) because Zone 2 has no heat demand
    val2 = int(openamber.get_entity("mixing_valve_zone2_output"))
    assert val2 == 15, f"Zone 2 valve must close to min (15%), got {val2}%"


def test_flow_zone2_demands_heat_zone1_no_demand_zone2_opens_zone1_closes(clean_system):
    """
    Flow requirement:
    - Zone 2 demands heat, zone 1 does not demand heat.
    - Zone 2 has mixing enabled so valve can open completely.
    - If zone 1 has mixing the valve needs to be closed.
    - Heatpump Tc setpoint is determined by Zone 2 setpoint.
    """
    openamber = clean_system

    openamber.set_switch("mixing_valve_zone1_enabled", True)
    openamber.set_switch("mixing_valve_zone2_enabled", True)
    openamber.set_number("mixing_valve_zone1_min_position", 20.0)
    openamber.set_number("mixing_valve_zone1_max_position", 80.0)
    openamber.set_number("mixing_valve_zone2_min_position", 15.0)
    openamber.set_number("mixing_valve_zone2_max_position", 85.0)

    # Set curve anchors: Zone 1 = 45°C, Zone 2 = 32°C
    openamber.set_select("heat_mode_select", "Stooklijn")
    openamber.set_number("heat_curve_m10", 45.0)
    openamber.set_number("heat_curve_0", 45.0)
    openamber.set_number("heat_curve_p5", 45.0)
    openamber.set_number("heat_curve_p10", 45.0)
    openamber.set_number("heat_curve_p15", 45.0)

    openamber.set_number("heat_curve_z2_m10", 32.0)
    openamber.set_number("heat_curve_z2_0", 32.0)
    openamber.set_number("heat_curve_z2_p5", 32.0)
    openamber.set_number("heat_curve_z2_p10", 32.0)
    openamber.set_number("heat_curve_z2_p15", 32.0)
    openamber.step(ms=50)

    # Zone 2 demands heat, Zone 1 does NOT demand heat
    _start_heating_and_reach_compressor_running(openamber, demand_z1=False, demand_z2=True)

    openamber.advance_time(seconds=5, step_s=1)
    openamber.step(ms=100)

    assert openamber.get_entity("heat_demand_active_sensor_zone_1") is False
    assert openamber.get_entity("heat_demand_active_sensor_zone_2") is True

    # Tc setpoint must be Zone 2 setpoint (32°C), ignoring Zone 1 (45°C)
    current_sp = float(openamber.get_entity("current_setpoint") or 0)
    assert current_sp == pytest.approx(32.0, abs=0.2)

    # Zone 2 valve opens completely to max (85%)
    val2 = int(openamber.get_entity("mixing_valve_zone2_output"))
    assert val2 == 85, f"Zone 2 valve must open completely to max (85%), got {val2}%"

    # Zone 1 valve closes to min (20%) because Zone 1 has no heat demand
    val1 = int(openamber.get_entity("mixing_valve_zone1_output"))
    assert val1 == 20, f"Zone 1 valve must close to min (20%), got {val1}%"


def test_flow_both_zones_demand_heat_lowest_temp_mixes_highest_temp_opens(clean_system):
    """
    Flow requirement:
    - Zone 1 demands heat, zone 2 demands heat, both zones have mixing.
    - Mix the zone which has the lowest temperature.
    - The other valve can be opened as Tc already is set to the highest setpoint.
    """
    openamber = clean_system

    openamber.set_switch("mixing_valve_zone1_enabled", True)
    openamber.set_switch("mixing_valve_zone2_enabled", True)
    openamber.set_number("mixing_valve_zone1_min_position", 20.0)
    openamber.set_number("mixing_valve_zone1_max_position", 80.0)
    openamber.set_number("mixing_valve_zone2_min_position", 15.0)
    openamber.set_number("mixing_valve_zone2_max_position", 85.0)

    # ── Case A: Zone 1 higher (35°C), Zone 2 lower (28°C) ──
    openamber.set_select("heat_mode_select", "Stooklijn")
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

    _start_heating_and_reach_compressor_running(openamber, demand_z1=True, demand_z2=True)

    # Tc setpoint must be highest of both (35°C)
    current_sp = float(openamber.get_entity("current_setpoint") or 0)
    assert current_sp == pytest.approx(35.0, abs=0.2)

    # Zone 1 (highest setpoint) opens completely to max (80%)
    openamber.advance_time(seconds=5, step_s=1)
    openamber.step(ms=100)
    val1 = int(openamber.get_entity("mixing_valve_zone1_output"))
    assert val1 == 80, f"Zone 1 valve must open completely to max (80%), got {val1}%"

    # Zone 2 (lowest setpoint) mixes via PID:
    # Under low Tv2 (20°C < 28°C), Zone 2 valve modulates open
    openamber.set_sensor("heat_cool_temperature_tv2", 20.0)
    openamber.advance_time(seconds=5, step_s=1)
    openamber.step(ms=100)
    val2_low = int(openamber.get_entity("mixing_valve_zone2_output"))
    assert val2_low > 50, f"Zone 2 valve must mix open when Tv2 is low, got {val2_low}%"

    # Under high Tv2 (36°C > 28°C), Zone 2 valve modulates closed
    openamber.set_sensor("heat_cool_temperature_tv2", 36.0)
    openamber.advance_time(seconds=5, step_s=1)
    openamber.step(ms=100)
    val2_high = int(openamber.get_entity("mixing_valve_zone2_output"))
    assert val2_high < 50, f"Zone 2 valve must mix closed when Tv2 is high, got {val2_high}%"

    # ── Case B: Zone 2 higher (38°C), Zone 1 lower (27°C) ──
    openamber.set_number("heat_curve_m10", 27.0)
    openamber.set_number("heat_curve_0", 27.0)
    openamber.set_number("heat_curve_p5", 27.0)
    openamber.set_number("heat_curve_p10", 27.0)
    openamber.set_number("heat_curve_p15", 27.0)

    openamber.set_number("heat_curve_z2_m10", 38.0)
    openamber.set_number("heat_curve_z2_0", 38.0)
    openamber.set_number("heat_curve_z2_p5", 38.0)
    openamber.set_number("heat_curve_z2_p10", 38.0)
    openamber.set_number("heat_curve_z2_p15", 38.0)
    openamber.advance_time(seconds=6, step_s=1)
    openamber.step(ms=100)

    # Tc setpoint must be highest of both (38°C)
    current_sp = float(openamber.get_entity("current_setpoint") or 0)
    assert current_sp == pytest.approx(38.0, abs=0.2)

    # Zone 2 (highest setpoint) opens completely to max (85%)
    val2 = int(openamber.get_entity("mixing_valve_zone2_output"))
    assert val2 == 85, f"Zone 2 valve must open completely to max (85%), got {val2}%"

    # Zone 1 (lowest setpoint) mixes via PID:
    # Under low Tv1 (19°C < 27°C), Zone 1 valve modulates open
    openamber.set_sensor("heat_cool_temperature_tv1", 19.0)
    openamber.advance_time(seconds=5, step_s=1)
    openamber.step(ms=100)
    val1_low = int(openamber.get_entity("mixing_valve_zone1_output"))
    assert val1_low > 50, f"Zone 1 valve must mix open when Tv1 is low, got {val1_low}%"

    # Under high Tv1 (35°C > 27°C), Zone 1 valve modulates closed
    openamber.set_sensor("heat_cool_temperature_tv1", 35.0)
    openamber.advance_time(seconds=5, step_s=1)
    openamber.step(ms=100)
    val1_high = int(openamber.get_entity("mixing_valve_zone1_output"))
    assert val1_high < 50, f"Zone 1 valve must mix closed when Tv1 is high, got {val1_high}%"

    # ── Case C: Both zones equal setpoint (32°C) ──
    openamber.set_number("heat_curve_m10", 32.0)
    openamber.set_number("heat_curve_0", 32.0)
    openamber.set_number("heat_curve_p5", 32.0)
    openamber.set_number("heat_curve_p10", 32.0)
    openamber.set_number("heat_curve_p15", 32.0)

    openamber.set_number("heat_curve_z2_m10", 32.0)
    openamber.set_number("heat_curve_z2_0", 32.0)
    openamber.set_number("heat_curve_z2_p5", 32.0)
    openamber.set_number("heat_curve_z2_p10", 32.0)
    openamber.set_number("heat_curve_z2_p15", 32.0)
    openamber.advance_time(seconds=6, step_s=1)
    openamber.step(ms=100)

    # When both are equal, both can open completely
    val1_eq = int(openamber.get_entity("mixing_valve_zone1_output"))
    val2_eq = int(openamber.get_entity("mixing_valve_zone2_output"))
    assert val1_eq == 80, f"Zone 1 valve must open to max (80%) when equal, got {val1_eq}%"
    assert val2_eq == 85, f"Zone 2 valve must open to max (85%) when equal, got {val2_eq}%"


def test_flow_both_zones_demand_heat_only_one_mixing_valve_enabled(clean_system):
    """
    Verify behavior when both zones demand heat but only one has mixing enabled:
    - If mixed zone has lower setpoint than unmixed zone -> mixed zone modulates via PID.
    - If mixed zone has higher or equal setpoint -> mixed zone opens completely.
    """
    openamber = clean_system

    # Case A: Only Zone 1 has mixing enabled (Zone 2 unmixed)
    openamber.set_switch("mixing_valve_zone1_enabled", True)
    openamber.set_switch("mixing_valve_zone2_enabled", False)
    openamber.set_number("mixing_valve_zone1_min_position", 20.0)
    openamber.set_number("mixing_valve_zone1_max_position", 80.0)

    openamber.set_select("heat_mode_select", "Stooklijn")
    openamber.set_number("heat_curve_m10", 28.0)
    openamber.set_number("heat_curve_0", 28.0)
    openamber.set_number("heat_curve_p5", 28.0)
    openamber.set_number("heat_curve_p10", 28.0)
    openamber.set_number("heat_curve_p15", 28.0)

    openamber.set_number("heat_curve_z2_m10", 35.0)
    openamber.set_number("heat_curve_z2_0", 35.0)
    openamber.set_number("heat_curve_z2_p5", 35.0)
    openamber.set_number("heat_curve_z2_p10", 35.0)
    openamber.set_number("heat_curve_z2_p15", 35.0)

    _start_heating_and_reach_compressor_running(openamber, demand_z1=True, demand_z2=True)

    # Zone 1 target 28°C < Zone 2 target 35°C -> Zone 1 mixes down via PID
    openamber.set_sensor("heat_cool_temperature_tv1", 20.0)
    openamber.advance_time(seconds=5, step_s=1)
    openamber.step(ms=100)
    val1_mix = int(openamber.get_entity("mixing_valve_zone1_output"))
    assert val1_mix > 50, f"Zone 1 must mix when lower than unmixed Zone 2, got {val1_mix}%"

    # Zone 1 target 38°C >= Zone 2 target 35°C -> Zone 1 opens completely
    openamber.set_number("heat_curve_m10", 38.0)
    openamber.set_number("heat_curve_0", 38.0)
    openamber.set_number("heat_curve_p5", 38.0)
    openamber.set_number("heat_curve_p10", 38.0)
    openamber.set_number("heat_curve_p15", 38.0)
    openamber.advance_time(seconds=6, step_s=1)
    openamber.step(ms=100)
    val1_open = int(openamber.get_entity("mixing_valve_zone1_output"))
    assert val1_open == 80, f"Zone 1 must open completely when higher than unmixed Zone 2, got {val1_open}%"

    # Case B: Only Zone 2 has mixing enabled (Zone 1 unmixed)
    openamber.set_switch("mixing_valve_zone1_enabled", False)
    openamber.set_switch("mixing_valve_zone2_enabled", True)
    openamber.set_number("mixing_valve_zone2_min_position", 15.0)
    openamber.set_number("mixing_valve_zone2_max_position", 85.0)

    # Zone 1 target = 35°C, Zone 2 target = 28°C -> Zone 2 mixes down via PID
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

    openamber.set_sensor("heat_cool_temperature_tv2", 20.0)
    openamber.advance_time(seconds=5, step_s=1)
    openamber.step(ms=100)
    val2_mix = int(openamber.get_entity("mixing_valve_zone2_output"))
    assert val2_mix > 50, f"Zone 2 must mix when lower than unmixed Zone 1, got {val2_mix}%"

    # Zone 2 target 38°C >= Zone 1 target 35°C -> Zone 2 opens completely
    openamber.set_number("heat_curve_z2_m10", 38.0)
    openamber.set_number("heat_curve_z2_0", 38.0)
    openamber.set_number("heat_curve_z2_p5", 38.0)
    openamber.set_number("heat_curve_z2_p10", 38.0)
    openamber.set_number("heat_curve_z2_p15", 38.0)
    openamber.advance_time(seconds=6, step_s=1)
    openamber.step(ms=100)
    val2_open = int(openamber.get_entity("mixing_valve_zone2_output"))
    assert val2_open == 85, f"Zone 2 must open completely when higher than unmixed Zone 1, got {val2_open}%"


def test_flow_wired_external_heat_demand_zone2(clean_system):
    """
    Verify wired contact and manual switch for Zone 2:
    - When neither Zone 1 nor Zone 2 mixing is enabled, there is no Zone 2 in the system:
      external_heat_demand_wired_zone_2 = True or heat_demand_zone_2_switch = True
      must NOT activate heat_demand_active_sensor_zone_2 or heat_demand_active_sensor.
    - When mixing is enabled (Zone 2 or Zone 1), Zone 2 heat demand triggers normally.
    """
    openamber = clean_system

    openamber.set_select("thermostat_mode_select", "Extern")
    openamber.set_binary_sensor("sg_ready_block_mode_active_sensor", False)
    openamber.set_binary_sensor("error_active", False)
    openamber.set_binary_sensor("external_heat_demand_wired", False)
    openamber.set_switch("heat_demand_switch", False)
    openamber.set_binary_sensor("external_heat_demand_wired_zone_2", False)
    openamber.set_switch("heat_demand_zone_2_switch", False)
    openamber.set_switch("mixing_valve_zone1_enabled", False)
    openamber.set_switch("mixing_valve_zone2_enabled", False)
    openamber.step(ms=150)

    assert openamber.get_entity("heat_demand_active_sensor_zone_2") is False
    assert openamber.get_entity("heat_demand_active_sensor") is False
    assert openamber.is_hidden("nav_status_heat_icon")

    # 1. Activate wired Zone 2 contact while mixing is disabled -> MUST REMAIN FALSE
    openamber.set_binary_sensor("external_heat_demand_wired_zone_2", True)
    openamber.step(ms=150)

    assert openamber.get_entity("heat_demand_active_sensor_zone_2") is False, (
        "Zone 2 heat demand must be False when neither mixing valve is enabled even if wired input is high"
    )
    assert openamber.get_entity("heat_demand_active_sensor") is False, (
        "Overall heat demand must be False when neither mixing valve is enabled and Zone 1 has no demand"
    )
    assert openamber.is_hidden("nav_status_heat_icon")

    # Also test manual Zone 2 heat demand switch while mixing is disabled -> MUST REMAIN FALSE
    openamber.set_binary_sensor("external_heat_demand_wired_zone_2", False)
    openamber.set_switch("heat_demand_zone_2_switch", True)
    openamber.step(ms=150)

    assert openamber.get_entity("heat_demand_active_sensor_zone_2") is False
    assert openamber.get_entity("heat_demand_active_sensor") is False
    assert openamber.is_hidden("nav_status_heat_icon")

    openamber.set_switch("heat_demand_zone_2_switch", False)
    openamber.step(ms=150)

    # 2. Enable mixing valve Zone 2 -> Now wired Zone 2 contact triggers demand
    openamber.set_switch("mixing_valve_zone2_enabled", True)
    openamber.set_binary_sensor("external_heat_demand_wired_zone_2", True)
    openamber.step(ms=150)

    assert openamber.get_entity("heat_demand_active_sensor_zone_2") is True
    assert openamber.get_entity("heat_demand_active_sensor") is True
    assert openamber.is_visible("nav_status_heat_icon")

    # 3. Deactivate wired Zone 2 contact -> deactivates
    openamber.set_binary_sensor("external_heat_demand_wired_zone_2", False)
    openamber.step(ms=150)

    assert openamber.get_entity("heat_demand_active_sensor_zone_2") is False
    assert openamber.get_entity("heat_demand_active_sensor") is False
    assert openamber.is_hidden("nav_status_heat_icon")

    # 4. Enable mixing valve Zone 1 instead of Zone 2 -> Zone 2 demand also works (multi-zone)
    openamber.set_switch("mixing_valve_zone2_enabled", False)
    openamber.set_switch("mixing_valve_zone1_enabled", True)
    openamber.set_binary_sensor("external_heat_demand_wired_zone_2", True)
    openamber.step(ms=150)

    assert openamber.get_entity("heat_demand_active_sensor_zone_2") is True
    assert openamber.get_entity("heat_demand_active_sensor") is True
    assert openamber.is_visible("nav_status_heat_icon")

    # Cleanup
    openamber.set_binary_sensor("external_heat_demand_wired_zone_2", False)
    openamber.set_switch("mixing_valve_zone1_enabled", False)
    openamber.step(ms=150)


def test_control_temperature_source_dropdown_disabled_and_forced_to_tc_when_mixing_enabled(clean_system):
    """
    Verify LVGL UI dropdown set_heat_control_temp_source_dd:
    1. Initially mixing valves are disabled -> dropdown is active (not disabled).
    2. Set control source to Tv1 ("CV-aanvoer zone 1 (Tv1)") -> dropdown shows index 1 / Tv1.
    3. Enable mixing valve Zone 1 -> dropdown is disabled (disabled == True) and forced to index 0 ("CV-aanvoer (Tc)").
    4. Enable mixing valve Zone 2 (disable Zone 1) -> dropdown remains disabled and forced to Tc.
    5. Disable both mixing valves -> dropdown is re-enabled (disabled == False).
    """
    openamber = clean_system

    # Navigate to Settings page -> Systeem
    assert openamber.click("nav_settings")
    openamber.step(ms=150)
    assert openamber.click("general_stab_systeem")
    openamber.step(ms=150)

    # 1. Initially mixing valves disabled -> dropdown is not disabled
    openamber.set_switch("mixing_valve_zone1_enabled", False)
    openamber.set_switch("mixing_valve_zone2_enabled", False)
    openamber.step(ms=100)

    w = openamber.get_widget("set_heat_control_temp_source_dd")
    assert w.get("disabled") is False, "Control temp source dropdown should be enabled when mixing valves are disabled"

    # Set to Tv1
    openamber.set_select("heat_cool_control_temperature_source_select", "CV-aanvoer zone 1 (Tv1)")
    openamber.step(ms=100)
    w_tv1 = openamber.get_widget("set_heat_control_temp_source_dd")
    assert w_tv1.get("selected_index") == 1 or "Tv1" in w_tv1.get("text", "")

    # 2. Enable mixing valve Zone 1 -> dropdown becomes disabled and forced to index 0 (Tc)
    openamber.set_sensor("heat_cool_temperature_tc", 38.0)
    openamber.set_sensor("heat_cool_temperature_tv1", 32.0)
    openamber.set_switch("mixing_valve_zone1_enabled", True)
    openamber.advance_time(seconds=6, step_s=1)
    openamber.step(ms=150)

    w_dis1 = openamber.get_widget("set_heat_control_temp_source_dd")
    assert w_dis1.get("disabled") is True, "Dropdown must be disabled when Zone 1 mixing valve is enabled"
    assert w_dis1.get("selected_index") == 0 or "Tc" in w_dis1.get("text", ""), "Dropdown must be forced to Tc when Zone 1 mixing is enabled"
    assert openamber.get_entity("heat_cool_control_temperature") == 38.0, "Control temperature must be Tc when mixing is enabled"

    # Attempting to change dropdown to Tv1 via UI while mixing is active should be ignored/reset to Tc
    openamber.set_dropdown("set_heat_control_temp_source_dd", 1)
    openamber.advance_time(seconds=6, step_s=1)
    openamber.step(ms=150)
    w_dis1_after = openamber.get_widget("set_heat_control_temp_source_dd")
    assert w_dis1_after.get("selected_index") == 0 or "Tc" in w_dis1_after.get("text", "")
    assert openamber.get_entity("heat_cool_control_temperature") == 38.0

    # 3. Enable mixing valve Zone 2 (and disable Zone 1) -> dropdown remains disabled and forced to Tc
    openamber.set_switch("mixing_valve_zone1_enabled", False)
    openamber.set_switch("mixing_valve_zone2_enabled", True)
    openamber.advance_time(seconds=6, step_s=1)
    openamber.step(ms=150)

    w_dis2 = openamber.get_widget("set_heat_control_temp_source_dd")
    assert w_dis2.get("disabled") is True, "Dropdown must be disabled when Zone 2 mixing valve is enabled"
    assert w_dis2.get("selected_index") == 0 or "Tc" in w_dis2.get("text", ""), "Dropdown must be forced to Tc when Zone 2 mixing is enabled"
    assert openamber.get_entity("heat_cool_control_temperature") == 38.0

    # 4. Disable both mixing valves -> dropdown is re-enabled and reflects select entity (Tv1)
    openamber.set_switch("mixing_valve_zone1_enabled", False)
    openamber.set_switch("mixing_valve_zone2_enabled", False)
    openamber.advance_time(seconds=6, step_s=1)
    openamber.step(ms=150)

    w_en = openamber.get_widget("set_heat_control_temp_source_dd")
    assert w_en.get("disabled") is False, "Dropdown must be re-enabled when both mixing valves are disabled"
    assert w_en.get("selected_index") == 1 or "Tv1" in w_en.get("text", ""), "Dropdown must restore Tv1 when mixing is disabled"
    assert openamber.get_entity("heat_cool_control_temperature") == 32.0, "Control temperature must revert to Tv1 when mixing is disabled"



