"""Tests for PID temperature controller and virtual time integration."""

import pytest


def test_pid_temperature_query_and_target_update(clean_system):
    """Verify querying and setting target temperatures on the PID controller."""
    openamber = clean_system

    # Set target temperature to 36.0°C
    assert openamber.set_climate("pid_heat_temperature_control", target_temperature=36.0, mode="HEAT"), (
        "Setting target temperature and HEAT mode on pid_heat_temperature_control should succeed"
    )
    openamber.step(ms=100)

    # Query PID climate entity
    info = openamber.get_entity("pid_heat_temperature_control")
    assert isinstance(info, dict), "Climate entity info should be a dict"
    assert info.get("status") == "ok", "Climate query status should be 'ok'"
    assert info.get("target_temperature") == pytest.approx(36.0, 0.1), (
        f"Expected target_temperature 36.0°C, got {info.get('target_temperature')}"
    )


def test_pid_virtual_time_integration(clean_system):
    """
    Virtual Time & PID Integration Test:
    1. Set PID target to 35.0°C and mode to HEAT.
    2. Set process sensor heat_cool_control_temperature to 30.0°C (under-temperature error = 5.0°C).
    3. Advance virtual time by 60 seconds (simulating 1 minute of controller run time).
    4. Trigger a sensor update so the PID calculates relative dt based on the elapsed virtual time.
    5. Advance virtual time by an additional 120 seconds (total 3 minutes) with ongoing error.
    6. Verify controller active heating response across time warps.
    """
    openamber = clean_system

    # Step 1: Configure target and mode
    assert openamber.set_select("heat_mode_select", "Extern setpoint"), "Setting Extern setpoint mode must succeed"
    assert openamber.set_number("manual_setpoint", 35.0), "Setting manual setpoint must succeed"
    assert openamber.set_climate("pid_heat_temperature_control", target_temperature=35.0, mode="HEAT"), (
        "Configuring climate target and HEAT mode must succeed"
    )
    openamber.step(ms=100)

    # Step 2: Incur temperature deficit
    assert openamber.set_sensor("heat_cool_control_temperature", 30.0), "Setting process sensor must succeed"
    openamber.step(ms=100)

    # Step 3: Advance virtual time by 60 seconds
    t_start = openamber.get_time().get("offset_ms", 0)
    offset_1 = openamber.advance_time(seconds=60)
    assert offset_1 - t_start >= 60000, "Virtual time should advance by at least 60s"

    # Trigger PID update with virtual time dt
    assert openamber.set_sensor("heat_cool_control_temperature", 30.1), "Setting sensor update must succeed"
    openamber.step(ms=100)

    # Step 4: Advance another 120 seconds
    offset_2 = openamber.advance_time(seconds=120)
    assert offset_2 - t_start >= 180000, "Virtual time should advance by at least 180s total"

    assert openamber.set_sensor("heat_cool_control_temperature", 30.2), "Setting sensor update must succeed"
    openamber.step(ms=100)

    # Verify climate entity is still tracking and responding
    info = openamber.get_entity("pid_heat_temperature_control")
    assert info.get("status") == "ok", "Climate query status must be 'ok'"
    assert info.get("target_temperature") == pytest.approx(35.0, 0.1), (
        f"Expected target_temperature 35.0°C, got {info.get('target_temperature')}"
    )
    assert info.get("current_temperature") == pytest.approx(30.2, 0.1), (
        f"Expected current_temperature 30.2°C, got {info.get('current_temperature')}"
    )
    assert info.get("mode") == "HEAT", f"Expected mode 'HEAT', got '{info.get('mode')}'"


def test_pid_deadband_stabilization_with_virtual_time(clean_system):
    """
    Deadband Test with Virtual Time:
    1. Set target to 35.0°C in HEAT mode.
    2. Bring current temperature to exactly 35.0°C (zero error, inside deadband).
    3. Advance virtual time by 180 seconds.
    4. Verify system remains stable in IDLE action without runaway.
    """
    openamber = clean_system

    assert openamber.set_climate("pid_heat_temperature_control", target_temperature=35.0, mode="HEAT"), (
        "Configuring climate target must succeed"
    )
    assert openamber.set_sensor("heat_cool_control_temperature", 35.0), "Setting process sensor must succeed"
    openamber.step(ms=100)

    # Time passes while in setpoint deadband
    openamber.advance_time(seconds=180)
    assert openamber.set_sensor("heat_cool_control_temperature", 35.0), "Re-publishing sensor must succeed"
    openamber.step(ms=100)

    info = openamber.get_entity("pid_heat_temperature_control")
    assert info.get("status") == "ok", "Climate query status must be 'ok'"
    assert info.get("current_temperature") == pytest.approx(35.0, 0.1), (
        f"Expected current_temperature 35.0°C, got {info.get('current_temperature')}"
    )
    assert info.get("action") == "IDLE", (
        f"PID action should stabilize in 'IDLE' inside deadband, got '{info.get('action')}'"
    )


