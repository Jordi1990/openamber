"""Automated tests for heat/cool safety checks and emergency protection flows.

Covers:
1. Error active safety check: stops compressor immediately when system error is active.
2. Hydraulic excessive delta-T safety check: stops the compressor and raises TO08 when Tuo - Tui stays above
   8.0°C for 60 s while heating. A negative difference (Tuo below Tui, as at the end of a defrost) and a short
   spike do not stop the unit.
3. Flow switch missing pump safety check: stops compressor after flow_switch_safety_delay_minutes expires and
   raises TO09.
"""

import pytest


def test_error_active_safety_check_stops_compressor(clean_system):
    """
    Safety Flow:
    1. Space heating is actively running with compressor started.
    2. An active error condition occurs (error_active = True).
    3. PerformSafetyChecks detects active error.
    4. Controller immediately invokes StopAndSetIdleState to protect heat exchanger and system.
    """
    openamber = clean_system

    openamber.set_select("thermostat_mode_select", "Extern")
    openamber.set_select("heat_mode_select", "Extern setpoint")
    openamber.set_number("manual_setpoint", 35.0)
    openamber.set_climate("pid_heat_temperature_control", target_temperature=35.0)
    openamber.set_select("heat_compressor_mode", "Maximaal")
    openamber.set_switch("heat_demand_switch", True)
    openamber.set_sensor("current_water_temperature_tc_sensor", 28.0)
    openamber.set_sensor("heat_cool_temperature_tc", 28.0)
    openamber.step(ms=50)

    # Advance time to start Space Heating pump and compressor
    openamber.advance_time(seconds=900, step_s=30)
    openamber.advance_time(seconds=140, step_s=10)
    openamber.advance_time(seconds=190, step_s=10)
    openamber.step(ms=50)

    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running", "Compressor should be running"
    assert int(openamber.get_entity("compressor_control_select") or 0) > 0, "Compressor mode should be > 0"

    # Trigger error condition via error_pump_start_timeout
    assert openamber.set_binary_sensor("error_pump_start_timeout", True) is True
    # openamber_component has an update_interval of 5s; advance enough time for the cycle to run
    openamber.advance_time(seconds=10, step_s=1)
    openamber.step(ms=50)

    assert openamber.get_entity("state_machine_state_heat_cool") == "Idle", f"State should return to Idle but was {openamber.get_entity('state_machine_state_heat_cool')}"
    # Controller must immediately stop compressor and revert to Idle
    assert int(openamber.get_entity("compressor_control_select") or 0) == 0, "Compressor must be stopped immediately"

    # Cleanup
    openamber.set_switch("heat_demand_switch", False)
    openamber.step(ms=50)


def test_hydraulic_excessive_delta_t_safety_check_stops_compressor(clean_system):
    """
    Safety Flow:
    1. Space heating is actively running with compressor running.
    2. Temperature difference |Tuo - Tui| is initially safe (28.0 - 26.0 = 2.0°C <= 8.0°C).
    3. Outlet temperature surges so |Tuo - Tui| exceeds 8.0°C safety threshold (38.0 - 26.0 = 12.0°C).
    4. PerformSafetyChecks detects excessive delta-T across heat exchanger.
    5. Controller immediately stops compressor and returns state to Idle.
    """
    openamber = clean_system

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
    openamber.step(ms=50)

    # Start pump and compressor
    openamber.advance_time(seconds=900, step_s=30)
    openamber.advance_time(seconds=140, step_s=10)
    openamber.advance_time(seconds=190, step_s=10)
    openamber.step(ms=50)

    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"
    assert int(openamber.get_entity("compressor_control_select") or 0) > 0

    # Outlet temp Tuo jumps to 38.0°C while inlet Tui stays 26.0°C -> 38 - 26 = 12°C > 8°C limit
    openamber.set_sensor("outlet_temperature_tuo", 38.0)
    openamber.advance_time(seconds=30, step_s=1)
    openamber.step(ms=50)
    assert int(openamber.get_entity("compressor_control_select") or 0) > 0, "No stop within the 60 s delay"
    assert openamber.get_entity("error_water_delta_t_too_high") is False

    openamber.advance_time(seconds=40, step_s=1)
    openamber.step(ms=50)

    # After 60 s the compressor must trip, the state revert to Idle and TO08 be raised
    assert int(openamber.get_entity("compressor_control_select") or 0) == 0, (
        "Compressor must stop when Tuo - Tui stays above the 8.0°C safety limit for 60 s"
    )
    assert openamber.get_entity("state_machine_state_heat_cool") == "Idle", (
        f"State must return to Idle, got {openamber.get_entity('state_machine_state_heat_cool')}"
    )
    assert openamber.get_entity("error_water_delta_t_too_high") is True, "The safety stop must raise TO08"
    assert openamber.get_entity("error_active") is True

    # The error is cleared with the reset switch
    openamber.set_sensor("outlet_temperature_tuo", 28.0)
    openamber.set_switch("reset_start_timeout_errors_switch", True)
    openamber.step(ms=50)
    assert openamber.get_entity("error_water_delta_t_too_high") is False

    # Cleanup
    openamber.set_switch("heat_demand_switch", False)
    openamber.step(ms=50)


def _start_heating_with_compressor(openamber):
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
    openamber.step(ms=50)

    openamber.advance_time(seconds=900, step_s=30)
    openamber.advance_time(seconds=140, step_s=10)
    openamber.advance_time(seconds=190, step_s=10)
    openamber.step(ms=50)

    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"
    assert int(openamber.get_entity("compressor_control_select") or 0) > 0


def test_negative_delta_t_while_heating_does_not_stop(clean_system):
    """
    At the end of a defrost the plate heat exchanger has cooled the water: Tuo is well below Tui
    (field case: Tuo 24.5, Tui 32.6, -8.1°C). That is not a reason for a safety stop while heating.
    """
    openamber = clean_system
    _start_heating_with_compressor(openamber)

    openamber.set_sensor("outlet_temperature_tuo", 24.5)
    openamber.set_sensor("inlet_temperature_tui", 32.6)
    openamber.advance_time(seconds=120, step_s=5)
    openamber.step(ms=50)

    assert int(openamber.get_entity("compressor_control_select") or 0) > 0, "A negative Tuo - Tui must not stop heating"
    assert openamber.get_entity("state_machine_state_heat_cool") != "Idle"
    assert openamber.get_entity("error_water_delta_t_too_high") is False

    # Cleanup
    openamber.set_sensor("outlet_temperature_tuo", 28.0)
    openamber.set_sensor("inlet_temperature_tui", 26.0)
    openamber.set_switch("heat_demand_switch", False)
    openamber.step(ms=50)


def test_short_delta_t_spike_does_not_stop(clean_system):
    """A Tuo - Tui difference above 8°C that lasts less than 60 s does not stop the unit, and the delay restarts."""
    openamber = clean_system
    _start_heating_with_compressor(openamber)

    for _ in range(2):
        openamber.set_sensor("outlet_temperature_tuo", 38.0)
        openamber.advance_time(seconds=45, step_s=5)
        openamber.set_sensor("outlet_temperature_tuo", 28.0)
        openamber.advance_time(seconds=15, step_s=5)
    openamber.step(ms=50)

    assert int(openamber.get_entity("compressor_control_select") or 0) > 0, "Short spikes must not stop heating"
    assert openamber.get_entity("error_water_delta_t_too_high") is False

    # Cleanup
    openamber.set_switch("heat_demand_switch", False)
    openamber.step(ms=50)


def test_flow_switch_missing_pump_safety_delay_stops_compressor(clean_system):
    """
    Safety Flow:
    1. Space heating is actively running with compressor started.
    2. Pump flow signal drops (pump stops or flow missing while compressor runs).
    3. Controller enters safety grace period defined by flow_switch_safety_delay_minutes.
    4. While elapsed time < safety delay, compressor continues running.
    5. Once safety delay elapses, controller stops compressor to prevent heat exchanger damage.
    """
    openamber = clean_system

    openamber.set_number("flow_switch_safety_delay_minutes", 1.0)  # 60s delay
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
    openamber.step(ms=50)

    # Start pump and compressor
    openamber.advance_time(seconds=900, step_s=30)
    openamber.advance_time(seconds=140, step_s=10)
    openamber.advance_time(seconds=190, step_s=10)
    openamber.step(ms=50)

    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"
    assert int(openamber.get_entity("compressor_control_select") or 0) > 0

    # Simulate pump failure/flow lost: set pump_control_pwm_number to 1000.0 (0% PWM -> flow lost while relay stays ON)
    openamber.set_number("pump_control_pwm_number", 1000.0)
    openamber.step(ms=50)
    assert openamber.get_entity("internal_pump_active") is False, "internal_pump_active must be False"

    # Advance 30s (less than 60s delay): compressor should still be running
    openamber.advance_time(seconds=30, step_s=5)
    openamber.step(ms=50)
    assert int(openamber.get_entity("compressor_control_select") or 0) > 0, "Compressor should still run within grace period"

    # Advance past 60s delay (another 40s -> total 70s > 60s): compressor must trip
    openamber.advance_time(seconds=40, step_s=5)
    openamber.step(ms=50)

    assert int(openamber.get_entity("compressor_control_select") or 0) == 0, "Compressor must stop after safety delay expires"
    assert openamber.get_entity("state_machine_state_heat_cool") == "Idle", "State must return to Idle"
    assert openamber.get_entity("error_pump_flow_missing") is True, "The safety stop must raise TO09"
    assert openamber.get_entity("error_active") is True

    # Cleanup
    openamber.set_switch("reset_start_timeout_errors_switch", True)
    openamber.step(ms=50)
    assert openamber.get_entity("error_pump_flow_missing") is False
    openamber.set_switch("heat_demand_switch", False)
    openamber.set_number("flow_switch_safety_delay_minutes", 2.0)
    openamber.step(ms=50)


def test_flow_switch_safety_delay_setting_under_advanced_pump(clean_system):
    """The flow switch delay is an advanced setting: Instellingen > Geavanceerd > Pomp, not Pomp > Algemeen."""
    import os

    settings_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                "src", "openamber", "ui", "settings")
    with open(os.path.join(settings_dir, "page_settings_tab_pump_algemeen_content.yaml"), encoding="utf-8-sig") as f:
        assert "flow_switch_safety" not in f.read(), "The flow switch delay must not be on Pomp > Algemeen"

    openamber = clean_system
    openamber.set_number("flow_switch_safety_delay_minutes", 2.0)
    assert openamber.click("nav_settings")
    openamber.step(ms=100)
    openamber.set_switch("advanced_settings_enabled", True)
    openamber.step(ms=100)
    try:
        assert openamber.click("stab_advanced")
        openamber.step(ms=50)
        assert openamber.click("advanced_stab_pump")
        openamber.step(ms=100)
        assert openamber.is_hidden("panel_advanced_page_pump") is False
        assert openamber.is_hidden("set_pump_flow_switch_safety_row") is False
        assert openamber.get_label("set_pump_flow_switch_safety_delay_val").startswith("2")
    finally:
        openamber.set_switch("advanced_settings_enabled", False)
        openamber.step(ms=50)
