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

    # Establish a deterministic positive output baseline with an under-target period
    assert openamber.set_sensor("heat_cool_temperature_tc", 30.0)
    openamber.advance_time(seconds=60, step_s=5)
    assert pid_output() > 50.0, f"Expected output > 50% during under-target period, got {pid_output()}%"

    assert openamber.set_sensor("heat_cool_temperature_tc", 35.0)
    openamber.advance_time(seconds=120, step_s=5)
    info = openamber.get_entity("pid_heat_temperature_control")
    assert info.get("target_temperature") == pytest.approx(35.0, abs=0.1)
    assert info.get("current_temperature") == pytest.approx(35.0, abs=0.1)
    settled = pid_output()
    assert settled >= 50.0, f"Expected settled baseline >= 50%, got {settled}%"

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


@pytest.mark.parametrize("deadband,expect_mode_drop", [
    (1.0, False),  # +0.5°C overshoot is inside the PID deadband: integral frozen, compressor mode holds
    (0.2, True),   # +0.5°C overshoot is outside the PID deadband: PID reacts and the compressor modulates down
])
def test_pid_deadband_setting_governs_compressor_modulation(clean_system, deadband, expect_mode_drop):
    """
    The configured PID deadband (pid_heat_deadband) must be the only deadband between Tc and the
    setpoint. Previously a hardcoded 1.0°C deadband in the heat/cool controller froze the compressor
    mode regardless of this setting, so a 0.2°C deadband still only reacted at 1.0°C deviation.
    1. Run the compressor at full demand until it saturates at mode 10.
    2. Raise Tc to 0.5°C above the setpoint (stop delta is 5°C, so the compressor keeps running).
    3. With a 1.0°C deadband the mode holds; with a 0.2°C deadband the mode drops.
    """
    openamber = clean_system

    openamber.set_select("thermostat_mode_select", "Extern")
    openamber.set_select("heat_mode_select", "Extern setpoint")
    openamber.set_number("manual_setpoint", 40.0)
    openamber.set_climate("pid_heat_temperature_control", target_temperature=40.0)
    openamber.set_number("pid_heat_deadband", deadband)
    openamber.set_number("backup_heater_degmin_threshold", 999.0)
    openamber.set_select("heat_compressor_mode", "Maximaal")
    openamber.set_switch("heat_demand_switch", True)
    openamber.set_sensor("current_water_temperature_tc_sensor", 25.0)
    openamber.set_sensor("heat_cool_temperature_tc", 25.0)
    openamber.set_sensor("outlet_temperature_tuo", 25.0)
    openamber.set_sensor("inlet_temperature_tui", 22.0)
    openamber.step(ms=50)

    # Pump interval + settle + soft start, then ramp up to mode 10 (one step per 300s)
    openamber.advance_time(seconds=900, step_s=30)
    openamber.advance_time(seconds=140, step_s=10)
    openamber.advance_time(seconds=190, step_s=10)
    openamber.advance_time(seconds=1900, step_s=30)
    openamber.step(ms=50)

    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"
    saturated_mode = int(openamber.get_entity("compressor_control_select"))
    assert saturated_mode == 10, f"Compressor should saturate at mode 10 under full demand, got {saturated_mode}"

    # Slight overshoot: 0.5°C above setpoint
    openamber.set_sensor("current_water_temperature_tc_sensor", 40.5)
    openamber.set_sensor("heat_cool_temperature_tc", 40.5)
    openamber.set_sensor("outlet_temperature_tuo", 40.5)
    openamber.set_sensor("inlet_temperature_tui", 37.5)
    openamber.advance_time(seconds=120, step_s=5)
    openamber.step(ms=50)

    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"
    mode = int(openamber.get_entity("compressor_control_select"))
    if expect_mode_drop:
        assert mode < saturated_mode, (
            f"With a {deadband}°C deadband a 0.5°C overshoot must modulate the compressor down, mode stayed at {mode}"
        )
    else:
        assert mode == saturated_mode, (
            f"With a {deadband}°C deadband a 0.5°C overshoot is inside the deadband, mode should hold at {saturated_mode}, got {mode}"
        )
