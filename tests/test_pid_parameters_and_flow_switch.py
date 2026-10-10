"""Automated tests for PID parameters and flow switch safety delay.

Covers settings:
1. Heating PID parameters:
   - pid_heat_kp, pid_heat_ki, pid_heat_kd, pid_heat_deadband
2. Cooling PID parameters:
   - pid_cool_kp, pid_cool_ki, pid_cool_kd, pid_cool_deadband
3. Pump P0 PID parameters:
   - pump_p0_pid_kp, pump_p0_pid_ki, pump_p0_pid_kd, pump_p0_pid_target_delta_t
4. Zone 1 Mixing Valve PID parameters:
   - pid_mixing_valve_zone1_kp, pid_mixing_valve_zone1_ki, pid_mixing_valve_zone1_kd, pid_mixing_valve_zone1_deadband
5. Zone 2 Mixing Valve PID parameters:
   - pid_mixing_valve_zone2_kp, pid_mixing_valve_zone2_ki, pid_mixing_valve_zone2_kd, pid_mixing_valve_zone2_deadband
6. Flow Switch Safety Delay:
   - flow_switch_safety_delay_minutes
"""

import pytest


def test_heating_and_cooling_pid_tuning_parameters(clean_system):
    """
    Verify configuration of Heating and Cooling PID tuning numbers:
    - Kp, Ki, Kd, and deadband thresholds.
    """
    openamber = clean_system

    # Heating PID parameters
    heat_params = [
        ("pid_heat_kp", 1.2),
        ("pid_heat_ki", 0.005),
        ("pid_heat_kd", 1.5),
        ("pid_heat_deadband", 0.8),
    ]
    for entity_id, val in heat_params:
        assert openamber.set_number(entity_id, val) is True
        openamber.step(ms=50)
        curr = float(openamber.get_entity(entity_id) or 0)
        assert curr == pytest.approx(val, 0.001), f"{entity_id} mismatch: {curr} != {val}"

    # Cooling PID parameters
    cool_params = [
        ("pid_cool_kp", 0.8),
        ("pid_cool_ki", 0.003),
        ("pid_cool_kd", 0.9),
        ("pid_cool_deadband", 0.6),
    ]
    for entity_id, val in cool_params:
        assert openamber.set_number(entity_id, val) is True
        openamber.step(ms=50)
        curr = float(openamber.get_entity(entity_id) or 0)
        assert curr == pytest.approx(val, 0.001), f"{entity_id} mismatch: {curr} != {val}"


def test_pump_and_mixing_valve_pid_parameters(clean_system):
    """
    Verify configuration of Pump P0 and Mixing Valve (Zone 1 & Zone 2) PID parameters.
    """
    openamber = clean_system

    # Pump P0 PID parameters
    pump_params = [
        ("pump_p0_pid_kp", 2.5),
        ("pump_p0_pid_ki", 0.08),
        ("pump_p0_pid_kd", 0.5),
        ("pump_p0_pid_target_delta_t", 4.0),
    ]
    for entity_id, val in pump_params:
        assert openamber.set_number(entity_id, val) is True
        openamber.step(ms=50)
        curr = float(openamber.get_entity(entity_id) or 0)
        assert curr == pytest.approx(val, 0.001), f"{entity_id} mismatch: {curr} != {val}"

    # Mixing Valve Zone 1 PID parameters
    mv1_params = [
        ("pid_mixing_valve_zone1_kp", 1.5),
        ("pid_mixing_valve_zone1_ki", 0.01),
        ("pid_mixing_valve_zone1_kd", 0.2),
        ("pid_mixing_valve_zone1_deadband", 1.2),
    ]
    for entity_id, val in mv1_params:
        assert openamber.set_number(entity_id, val) is True
        openamber.step(ms=50)
        curr = float(openamber.get_entity(entity_id) or 0)
        assert curr == pytest.approx(val, 0.001), f"{entity_id} mismatch: {curr} != {val}"

    # Mixing Valve Zone 2 PID parameters
    mv2_params = [
        ("pid_mixing_valve_zone2_kp", 1.8),
        ("pid_mixing_valve_zone2_ki", 0.012),
        ("pid_mixing_valve_zone2_kd", 0.3),
        ("pid_mixing_valve_zone2_deadband", 1.5),
    ]
    for entity_id, val in mv2_params:
        assert openamber.set_number(entity_id, val) is True
        openamber.step(ms=50)
        curr = float(openamber.get_entity(entity_id) or 0)
        assert curr == pytest.approx(val, 0.001), f"{entity_id} mismatch: {curr} != {val}"


def test_flow_switch_safety_delay_parameter(clean_system):
    """
    Verify flow_switch_safety_delay_minutes configuration setting.
    """
    openamber = clean_system

    openamber.set_number("flow_switch_safety_delay_minutes", 3.0)
    openamber.step(ms=50)
    assert float(openamber.get_entity("flow_switch_safety_delay_minutes") or 0) == 3.0, "Expected delay 3.0 min"

    openamber.set_number("flow_switch_safety_delay_minutes", 5.0)
    openamber.step(ms=50)
    assert float(openamber.get_entity("flow_switch_safety_delay_minutes") or 0) == 5.0, "Expected delay 5.0 min"

    # Reset
    openamber.set_number("flow_switch_safety_delay_minutes", 2.0)
    openamber.step(ms=50)


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
