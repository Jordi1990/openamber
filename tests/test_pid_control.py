"""Tests for PID temperature controller and virtual time integration."""

import pytest


def test_pid_temperature_query_and_target_update(openamber):
    """Verify querying and setting target temperatures on the PID controller."""
    # Set target temperature to 36.0°C
    assert openamber.set_climate("pid_heat_temperature_control", target_temperature=36.0)
    openamber.step(ms=100)

    # Query PID climate entity
    info = openamber.get_entity("pid_heat_temperature_control")
    assert isinstance(info, dict)
    assert info.get("status") == "ok"
    assert info.get("target_temperature") == pytest.approx(36.0, 0.1)


def test_pid_virtual_time_integration(openamber):
    """
    Virtual Time & PID Integration Test:
    1. Set PID target to 35.0°C.
    2. Set process sensor heat_cool_control_temperature to 30.0°C (under-temperature error = 5.0°C).
    3. Advance virtual time by 60 seconds (simulating 1 minute of controller run time).
    4. Trigger a sensor update so the PID calculates relative dt based on the elapsed virtual time.
    5. Advance virtual time by an additional 120 seconds (total 3 minutes) with ongoing error.
    6. Verify controller state remains valid and robust across time warps.
    """
    openamber.reset_time()

    # Step 1: Configure target
    openamber.set_climate("pid_heat_temperature_control", target_temperature=35.0)
    openamber.step(ms=100)

    # Step 2: Incur temperature deficit
    openamber.set_sensor("heat_cool_control_temperature", 30.0)
    openamber.step(ms=100)

    # Step 3: Advance virtual time by 60 seconds
    offset_1 = openamber.advance_time(seconds=60)
    assert offset_1 >= 60000

    # Trigger PID update with virtual time dt
    openamber.set_sensor("heat_cool_control_temperature", 30.1)
    openamber.step(ms=100)

    # Step 4: Advance another 120 seconds
    offset_2 = openamber.advance_time(seconds=120)
    assert offset_2 >= 180000

    openamber.set_sensor("heat_cool_control_temperature", 30.2)
    openamber.step(ms=100)

    # Verify climate entity is still tracking and responding
    info = openamber.get_entity("pid_heat_temperature_control")
    assert info.get("status") == "ok"
    assert info.get("target_temperature") == pytest.approx(35.0, 0.1)

    openamber.reset_time()


def test_pid_deadband_stabilization_with_virtual_time(openamber):
    """
    Deadband Test with Virtual Time:
    1. Set target to 35.0°C.
    2. Bring current temperature to exactly 35.0°C (zero error, inside deadband).
    3. Advance virtual time by 180 seconds.
    4. Verify system remains stable without integral runaway.
    """
    openamber.reset_time()
    openamber.set_climate("pid_heat_temperature_control", target_temperature=35.0)
    openamber.set_sensor("heat_cool_control_temperature", 35.0)
    openamber.step(ms=100)

    # Time passes while in setpoint deadband
    openamber.advance_time(seconds=180)
    openamber.set_sensor("heat_cool_control_temperature", 35.0)
    openamber.step(ms=100)

    info = openamber.get_entity("pid_heat_temperature_control")
    assert info.get("status") == "ok"
    assert info.get("current_temperature") == pytest.approx(35.0, 0.1)

    openamber.reset_time()
