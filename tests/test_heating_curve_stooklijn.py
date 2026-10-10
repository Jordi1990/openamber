"""Automated tests for Heating Curve (Stooklijn) setpoint calculation and settings.

Covers:
1. Zone 1 heating curve setpoint evaluation across all anchor points (-10, 0, +5, +10, +15°C).
2. Boundary clamping when outside temperature exceeds upper (+15°C) or lower (-10°C) calibration limits.
3. Linear interpolation between calibration points (e.g. at Ta = 7.5°C and 2.5°C).
4. Dynamic adjustment of individual stooklijn temperature numbers (heat_curve_m10, 0, p5, p10, p15).
5. Heat mode selection: Stooklijn vs Extern setpoint (manual_setpoint).
"""

import pytest


def test_heating_curve_zone1_discrete_anchors_and_clamping(clean_system):
    """Verify Zone 1 stooklijn calculated setpoint at exact anchor points and clamp boundaries."""
    openamber = clean_system

    # Select Stooklijn mode
    assert openamber.set_select("heat_mode_select", "Stooklijn")

    # Configure distinct anchor temperatures for Zone 1
    assert openamber.set_number("heat_curve_m10", 45.0)
    assert openamber.set_number("heat_curve_0", 40.0)
    assert openamber.set_number("heat_curve_p5", 35.0)
    assert openamber.set_number("heat_curve_p10", 30.0)
    assert openamber.set_number("heat_curve_p15", 25.0)
    openamber.step(ms=50)

    # Test anchor point: Ta = 15°C -> should yield 25°C
    assert openamber.set_sensor("temperature_outside_ta", 15.0)
    openamber.advance_time(seconds=6, step_s=2)
    openamber.step(ms=50)
    sp = float(openamber.get_entity("heat_curve_calculated_setpoint") or 0)
    assert sp == pytest.approx(25.0, abs=0.5), f"Ta=15°C anchor should calculate setpoint 25°C, got {sp}"

    # Test anchor point: Ta = 10°C -> should yield 30°C
    assert openamber.set_sensor("temperature_outside_ta", 10.0)
    openamber.advance_time(seconds=6, step_s=2)
    openamber.step(ms=50)
    sp = float(openamber.get_entity("heat_curve_calculated_setpoint") or 0)
    assert sp == pytest.approx(30.0, abs=0.5), f"Ta=10°C anchor should calculate setpoint 30°C, got {sp}"

    # Test anchor point: Ta = 5°C -> should yield 35°C
    assert openamber.set_sensor("temperature_outside_ta", 5.0)
    openamber.advance_time(seconds=6, step_s=2)
    openamber.step(ms=50)
    sp = float(openamber.get_entity("heat_curve_calculated_setpoint") or 0)
    assert sp == pytest.approx(35.0, abs=0.5), f"Ta=5°C anchor should calculate setpoint 35°C, got {sp}"

    # Test anchor point: Ta = 0°C -> should yield 40°C
    assert openamber.set_sensor("temperature_outside_ta", 0.0)
    openamber.advance_time(seconds=6, step_s=2)
    openamber.step(ms=50)
    sp = float(openamber.get_entity("heat_curve_calculated_setpoint") or 0)
    assert sp == pytest.approx(40.0, abs=0.5), f"Ta=0°C anchor should calculate setpoint 40°C, got {sp}"

    # Test anchor point: Ta = -10°C -> should yield 45°C
    assert openamber.set_sensor("temperature_outside_ta", -10.0)
    openamber.advance_time(seconds=6, step_s=2)
    openamber.step(ms=50)
    sp = float(openamber.get_entity("heat_curve_calculated_setpoint") or 0)
    assert sp == pytest.approx(45.0, abs=0.5), f"Ta=-10°C anchor should calculate setpoint 45°C, got {sp}"

    # Test upper clamp: Ta = 20°C (> 15°C) -> should clamp to heat_curve_p15 (25°C)
    assert openamber.set_sensor("temperature_outside_ta", 20.0)
    openamber.advance_time(seconds=6, step_s=2)
    openamber.step(ms=50)
    sp = float(openamber.get_entity("heat_curve_calculated_setpoint") or 0)
    assert sp == pytest.approx(25.0, abs=0.5), f"Ta=20°C upper clamp should yield 25°C, got {sp}"

    # Test lower clamp: Ta = -18°C (< -10°C) -> should clamp to heat_curve_m10 (45°C)
    assert openamber.set_sensor("temperature_outside_ta", -18.0)
    openamber.advance_time(seconds=6, step_s=2)
    openamber.step(ms=50)
    sp = float(openamber.get_entity("heat_curve_calculated_setpoint") or 0)
    assert sp == pytest.approx(45.0, abs=0.5), f"Ta=-18°C lower clamp should yield 45°C, got {sp}"


def test_heating_curve_linear_interpolation(clean_system):
    """Verify linear interpolation between calibration anchors."""
    openamber = clean_system

    openamber.set_select("heat_mode_select", "Stooklijn")
    openamber.set_number("heat_curve_p15", 24.0)
    openamber.set_number("heat_curve_p10", 30.0)
    openamber.set_number("heat_curve_p5", 36.0)
    openamber.set_number("heat_curve_0", 42.0)
    openamber.step(ms=50)

    # Midpoint between 5°C (36°C) and 10°C (30°C): Ta = 7.5°C -> expected setpoint = 33.0°C
    openamber.set_sensor("temperature_outside_ta", 7.5)
    openamber.advance_time(seconds=6, step_s=2)
    openamber.step(ms=50)
    sp = float(openamber.get_entity("heat_curve_calculated_setpoint") or 0)
    assert sp == pytest.approx(33.0, abs=0.5)

    # Midpoint between 0°C (42°C) and 5°C (36°C): Ta = 2.5°C -> expected setpoint = 39.0°C
    openamber.set_sensor("temperature_outside_ta", 2.5)
    openamber.advance_time(seconds=6, step_s=2)
    openamber.step(ms=50)
    sp = float(openamber.get_entity("heat_curve_calculated_setpoint") or 0)
    assert sp == pytest.approx(39.0, abs=0.5)


def test_heating_curve_setting_dynamic_adjustment(clean_system):
    """Verify that changing individual stooklijn setting numbers immediately updates the calculated setpoint."""
    openamber = clean_system

    openamber.set_select("heat_mode_select", "Stooklijn")
    openamber.set_sensor("temperature_outside_ta", 5.0)
    openamber.set_number("heat_curve_p5", 33.0)
    openamber.advance_time(seconds=6, step_s=2)
    openamber.step(ms=50)

    assert float(openamber.get_entity("heat_curve_calculated_setpoint") or 0) == pytest.approx(33.0, abs=0.5)

    # User adjusts heat_curve_p5 by +5°C (to 38.0°C)
    openamber.set_number("heat_curve_p5", 38.0)
    openamber.advance_time(seconds=6, step_s=2)
    openamber.step(ms=50)

    assert float(openamber.get_entity("heat_curve_calculated_setpoint") or 0) == pytest.approx(38.0, abs=0.5)


def test_heat_mode_switching_stooklijn_vs_manual_setpoint(clean_system):
    """Verify system setpoint toggles cleanly between Stooklijn and Extern setpoint (manual_setpoint)."""
    openamber = clean_system

    openamber.set_sensor("temperature_outside_ta", 5.0)
    openamber.set_number("heat_curve_p5", 34.0)
    openamber.set_number("manual_setpoint", 42.0)

    # Mode 1: Stooklijn active -> current_setpoint tracks curve (34°C)
    openamber.set_select("heat_mode_select", "Stooklijn")
    openamber.advance_time(seconds=14, step_s=2)
    openamber.step(ms=50)
    assert float(openamber.get_entity("current_setpoint") or 0) == pytest.approx(34.0, abs=0.5)

    # Mode 2: Switch to Extern setpoint -> current_setpoint tracks manual_setpoint (42°C)
    openamber.set_select("heat_mode_select", "Extern setpoint")
    openamber.advance_time(seconds=14, step_s=2)
    openamber.step(ms=50)
    assert float(openamber.get_entity("current_setpoint") or 0) == pytest.approx(42.0, abs=0.5)

    # Change manual setpoint to 48°C
    openamber.set_number("manual_setpoint", 48.0)
    openamber.advance_time(seconds=6, step_s=2)
    openamber.step(ms=50)
    assert float(openamber.get_entity("current_setpoint") or 0) == pytest.approx(48.0, abs=0.5)

    # Switch back to Stooklijn -> returns to 34°C
    openamber.set_select("heat_mode_select", "Stooklijn")
    openamber.advance_time(seconds=6, step_s=2)
    openamber.step(ms=50)
    assert float(openamber.get_entity("current_setpoint") or 0) == pytest.approx(34.0, abs=0.5)
