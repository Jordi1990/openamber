"""Automated tests for Zone 2 Heating Curve and Mixing Valve settings.

Covers settings:
1. Zone 2 Heating Curve (Stooklijn zone 2):
   - heat_curve_z2_m10 (-10°C anchor)
   - heat_curve_z2_0 (0°C anchor)
   - heat_curve_z2_p5 (+5°C anchor)
   - heat_curve_z2_p10 (+10°C anchor)
   - heat_curve_z2_p15 (+15°C anchor)
2. Mixing Valve Enable Switches:
   - mixing_valve_zone1_enabled
   - mixing_valve_zone2_enabled
3. Mixing Valve Position Limits:
   - mixing_valve_zone1_min_position
   - mixing_valve_zone1_max_position
   - mixing_valve_zone2_min_position
   - mixing_valve_zone2_max_position
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


def test_mixing_valve_switches_and_position_limits(clean_system):
    """
    Verify Mixing Valve settings for Zone 1 and Zone 2:
    1. Toggle mixing_valve_zone1_enabled and mixing_valve_zone2_enabled.
    2. Configure min and max position limits:
       - mixing_valve_zone1_min_position: 10%
       - mixing_valve_zone1_max_position: 90%
       - mixing_valve_zone2_min_position: 15%
       - mixing_valve_zone2_max_position: 85%
    3. Verify states and ranges are persisted.
    """
    openamber = clean_system

    # Zone 1 switch
    openamber.set_switch("mixing_valve_zone1_enabled", True)
    openamber.step(ms=50)
    assert openamber.get_entity("mixing_valve_zone1_enabled") is True

    openamber.set_switch("mixing_valve_zone1_enabled", False)
    openamber.step(ms=50)
    assert openamber.get_entity("mixing_valve_zone1_enabled") is False

    # Zone 2 switch
    openamber.set_switch("mixing_valve_zone2_enabled", True)
    openamber.step(ms=50)
    assert openamber.get_entity("mixing_valve_zone2_enabled") is True

    openamber.set_switch("mixing_valve_zone2_enabled", False)
    openamber.step(ms=50)
    assert openamber.get_entity("mixing_valve_zone2_enabled") is False

    # Zone 1 Min/Max limits
    openamber.set_number("mixing_valve_zone1_min_position", 10.0)
    openamber.set_number("mixing_valve_zone1_max_position", 90.0)
    openamber.step(ms=50)
    assert float(openamber.get_entity("mixing_valve_zone1_min_position") or 0) == 10.0
    assert float(openamber.get_entity("mixing_valve_zone1_max_position") or 0) == 90.0

    # Zone 2 Min/Max limits
    openamber.set_number("mixing_valve_zone2_min_position", 15.0)
    openamber.set_number("mixing_valve_zone2_max_position", 85.0)
    openamber.step(ms=50)
    assert float(openamber.get_entity("mixing_valve_zone2_min_position") or 0) == 15.0
    assert float(openamber.get_entity("mixing_valve_zone2_max_position") or 0) == 85.0

    # Reset defaults
    openamber.set_number("mixing_valve_zone1_min_position", 0.0)
    openamber.set_number("mixing_valve_zone1_max_position", 100.0)
    openamber.set_number("mixing_valve_zone2_min_position", 0.0)
    openamber.set_number("mixing_valve_zone2_max_position", 100.0)
    openamber.step(ms=50)
