"""Tests for PID temperature controller and virtual time integration."""

import pytest


def test_pid_virtual_time_integration(clean_system):
    """
    Virtual Time & PID Integration Test:
    1. Set PID target to 35.0°C and mode to HEAT.
    2. Set Tc (source of heat_cool_control_temperature) to 30.0°C (under-temperature error = 5.0°C).
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
    assert openamber.set_sensor("heat_cool_temperature_tc", 30.0), "Setting process sensor must succeed"
    openamber.advance_time(seconds=6, step_s=1)  # heat_cool_control_temperature follows Tc every 5s
    openamber.step(ms=100)

    # Step 3: Advance virtual time by 60 seconds
    t_start = openamber.get_time().get("offset_ms", 0)
    offset_1 = openamber.advance_time(seconds=60)
    assert offset_1 - t_start >= 60000, "Virtual time should advance by at least 60s"

    # Trigger PID update with virtual time dt
    assert openamber.set_sensor("heat_cool_temperature_tc", 30.1), "Setting sensor update must succeed"
    openamber.advance_time(seconds=6, step_s=1)  # heat_cool_control_temperature follows Tc every 5s
    openamber.step(ms=100)

    # Step 4: Advance another 120 seconds
    offset_2 = openamber.advance_time(seconds=120)
    assert offset_2 - t_start >= 180000, "Virtual time should advance by at least 180s total"

    assert openamber.set_sensor("heat_cool_temperature_tc", 30.2), "Setting sensor update must succeed"

    openamber.advance_time(seconds=6, step_s=1)  # heat_cool_control_temperature follows Tc every 5s
    openamber.step(ms=100)

    # Verify climate entity is still tracking and responding
    info = openamber.get_entity("pid_heat_temperature_control")
    assert info.get("status") == "ok", "Climate query status must be 'ok'"
    assert info.get("target_temperature") == pytest.approx(35.0, abs=0.1), (
        f"Expected target_temperature 35.0°C, got {info.get('target_temperature')}"
    )
    assert info.get("current_temperature") == pytest.approx(30.2, abs=0.1), (
        f"Expected current_temperature 30.2°C, got {info.get('current_temperature')}"
    )
    assert info.get("mode") == "HEAT", f"Expected mode 'HEAT', got '{info.get('mode')}'"
    # A sustained ~5°C deficit must drive the heating output; 'IDLE' here would mean the PID is not acting.
    assert info.get("action") == "HEATING", (
        f"PID action should be 'HEATING' with a 5°C deficit, got '{info.get('action')}'"
    )


def test_pid_deadband_stabilization_with_virtual_time(clean_system):
    """
    Deadband Test with Virtual Time (deadband_parameters threshold ±1°C):
    1. Target 35.0°C in HEAT mode, Tc held at the target for 180s.
    2. Small fluctuations inside the deadband (±0.5°C) barely move the PID output.
    3. A deviation outside the deadband (+2°C overshoot) changes the output strongly.

    Note: at the setpoint the output is NOT zero; the integral term holds it (steady state),
    so the climate action stays 'HEATING'. The old 'action == IDLE' expectation only passed
    because the injected process value was overwritten by the template sensor.
    """
    openamber = clean_system

    # Pin the target: in 'Stooklijn' mode current_setpoint overwrites the PID target every 5s.
    assert openamber.set_select("heat_mode_select", "Extern setpoint")
    assert openamber.set_number("manual_setpoint", 35.0)
    assert openamber.set_climate("pid_heat_temperature_control", target_temperature=35.0, mode="HEAT")

    def pid_output():
        # Sensor 'PID Controller Verwarmen' (platform pid, type HEAT) reports the output in %
        return float(openamber.get_entity("PID Controller Verwarmen"))

    assert openamber.set_sensor("heat_cool_temperature_tc", 35.0)
    openamber.advance_time(seconds=180, step_s=5)
    info = openamber.get_entity("pid_heat_temperature_control")
    assert info.get("target_temperature") == pytest.approx(35.0, abs=0.1)
    assert info.get("current_temperature") == pytest.approx(35.0, abs=0.1)
    settled = pid_output()

    for tc in (35.5, 34.5):
        assert openamber.set_sensor("heat_cool_temperature_tc", tc)
        openamber.advance_time(seconds=60, step_s=5)
        out = pid_output()
        assert abs(out - settled) <= 10.0, (
            f"Inside the deadband (Tc={tc}°C) the output should stay near {settled:.1f}%, got {out:.1f}%"
        )

    assert openamber.set_sensor("heat_cool_temperature_tc", 37.0)
    openamber.advance_time(seconds=60, step_s=5)
    out = pid_output()
    assert out < settled - 50.0, (
        f"Outside the deadband (+2°C overshoot) the output must drop strongly from {settled:.1f}%, got {out:.1f}%"
    )
