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
