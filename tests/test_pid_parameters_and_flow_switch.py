"""Automated tests for heat/cool safety checks and emergency protection flows.

Covers:
1. Error active safety check: stops compressor immediately when system error is active.
2. Hydraulic excessive delta-T safety check: stops compressor when |Tuo - Tui| > 8.0°C.
3. Flow switch missing pump safety check: stops compressor after flow_switch_safety_delay_minutes expires.
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

    # Outlet temp Tuo jumps to 38.0°C while inlet Tui stays 26.0°C -> |38 - 26| = 12°C > 8°C limit
    openamber.set_sensor("outlet_temperature_tuo", 38.0)
    openamber.advance_time(seconds=10, step_s=1)
    openamber.step(ms=50)

    # Compressor must immediately trip and state must revert to Idle
    assert int(openamber.get_entity("compressor_control_select") or 0) == 0, (
        "Compressor must stop immediately when |Tuo - Tui| exceeds 8.0°C safety limit"
    )
    assert openamber.get_entity("state_machine_state_heat_cool") == "Idle", (
        f"State must return to Idle, got {openamber.get_entity('state_machine_state_heat_cool')}"
    )

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

    # Simulate pump failure/flow lost: turn off pump_p0_relay_switch
    openamber.set_switch("pump_p0_relay_switch", False)
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

    # Cleanup
    openamber.set_switch("heat_demand_switch", False)
    openamber.set_number("flow_switch_safety_delay_minutes", 2.0)
    openamber.step(ms=50)
