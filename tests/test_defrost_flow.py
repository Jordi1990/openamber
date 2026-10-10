"""Automated tests for defrost handling in Heating and DHW modes.
Covers defrost entry, pump P1 stop/start, P0 defrost PWM duty cycle, post-defrost settle times (300s),
compressor recovery boost mode (+3 steps), and low ambient backup heater boost.
"""

import pytest


def test_heat_defrost_recovery_boost_and_settle_time(clean_system):
    """
    Defrost Flow 1: Defrost Recovery Boost (+3 steps) & 5-minute settle time
    1. System starts in space heating mode (target 35°C).
    2. Compressor softstart finishes and enters 'Compressor running'.
    3. Defrost cycle initiates (defrost_active_sensor = True).
    4. State machine transitions to 'Defrosting'.
    5. Tc drops to 28.0°C (< target 35°C - 3.0°C = 32°C).
    6. Defrost cycle ends (defrost_active_sensor = False).
    7. Compressor applies Defrost Recovery Boost (+3 steps mode increase).
    8. System enters 5-minute settle period (COMPRESSOR_SETTLE_TIME_AFTER_DEFROST_S = 300s).
    9. During settle period, state is 'Wait for state switch' and boosted mode is held.
    10. After 300s, system transitions back to 'Compressor running' and resumes normal PID modulation.
    """
    openamber = clean_system

    # Step 1: Start Space Heating
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
    openamber.set_sensor("temperature_outside_ta", 5.0)  # > -3°C boost threshold
    openamber.step(ms=50)

    # Advance time through IDLE pump interval (900s) + pump settle (140s) + softstart (190s)
    openamber.advance_time(seconds=900, step_s=30)
    openamber.advance_time(seconds=140, step_s=10)
    openamber.advance_time(seconds=190, step_s=10)
    openamber.step(ms=50)

    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running", (
        f"State should be 'Compressor running', got {openamber.get_entity('state_machine_state_heat_cool')}"
    )
    initial_mode = int(openamber.get_entity("compressor_control_select") or 0)
    assert initial_mode > 0, f"Expected initial compressor mode > 0, got {initial_mode}"

    # Step 2: Defrost cycle activates
    openamber.set_binary_sensor("defrost_active_sensor", True)
    openamber.advance_time(seconds=10, step_s=2)
    openamber.step(ms=50)

    assert openamber.get_entity("state_machine_state_heat_cool") == "Defrosting", "Expected 'Defrosting' state"

    # Step 3: Defrost cycle ends with Tc < target - 3°C (28°C < 32°C)
    openamber.set_sensor("current_water_temperature_tc_sensor", 28.0)
    openamber.set_sensor("heat_cool_temperature_tc", 28.0)
    openamber.set_binary_sensor("defrost_active_sensor", False)
    openamber.advance_time(seconds=10, step_s=2)
    openamber.step(ms=50)

    # Verify recovery boost mode is applied (+3 steps, capped by max mode 10)
    boosted_mode = int(openamber.get_entity("compressor_control_select") or 0)
    assert boosted_mode >= min(initial_mode + 3, 10), (
        f"Compressor mode should be boosted (+3 steps) from {initial_mode}, got {boosted_mode}"
    )

    # Verify settle state
    assert openamber.get_entity("state_machine_state_heat_cool") == "Wait for state switch", "State should be 'Wait for state switch'"

    # Step 4: Advance virtual time 270s (< 300s settle time, taking into account 10s already advanced)
    openamber.advance_time(seconds=270, step_s=20)
    openamber.step(ms=50)

    # Still waiting in settle period
    assert openamber.get_entity("state_machine_state_heat_cool") == "Wait for state switch", "State should hold in 'Wait for state switch'"
    assert int(openamber.get_entity("compressor_control_select") or 0) == boosted_mode, "Boosted mode should hold during settle"

    # Step 5: Advance remaining 30s (total > 300s settle time)
    openamber.advance_time(seconds=30, step_s=10)
    openamber.step(ms=50)

    # Transitioned back to normal compressor running
    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running", (
        f"State should return to 'Compressor running' after 300s settle, got {openamber.get_entity('state_machine_state_heat_cool')}"
    )


def test_heat_defrost_no_recovery_boost_when_near_target(clean_system):
    """
    Defrost Flow 2: No Recovery Boost when Tc is close to target setpoint (Tc >= target - 3°C)
    1. System is running in space heating mode (target 35°C).
    2. Defrost cycle initiates and ends.
    3. Tc is 34.0°C (>= target 35°C - 3.0°C = 32°C).
    4. Verify compressor mode is NOT boosted.
    5. Settle time of 300s is still observed before returning to Compressor running.
    """
    openamber = clean_system

    openamber.set_select("thermostat_mode_select", "Extern")
    openamber.set_select("heat_mode_select", "Extern setpoint")
    openamber.set_number("manual_setpoint", 35.0)
    openamber.set_climate("pid_heat_temperature_control", target_temperature=35.0)
    openamber.set_select("heat_compressor_mode", "Maximaal")
    openamber.set_switch("heat_demand_switch", True)
    openamber.set_sensor("current_water_temperature_tc_sensor", 30.0)
    openamber.set_sensor("heat_cool_temperature_tc", 30.0)
    openamber.set_sensor("outlet_temperature_tuo", 30.0)
    openamber.set_sensor("inlet_temperature_tui", 28.0)
    openamber.set_sensor("temperature_outside_ta", 5.0)
    openamber.step(ms=50)

    openamber.advance_time(seconds=900, step_s=30)
    openamber.advance_time(seconds=140, step_s=10)
    openamber.advance_time(seconds=190, step_s=10)
    openamber.step(ms=50)

    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running", "State should be 'Compressor running'"
    mode_before_defrost = int(openamber.get_entity("compressor_control_select") or 0)

    # Defrost activates
    openamber.set_binary_sensor("defrost_active_sensor", True)
    openamber.advance_time(seconds=10, step_s=2)
    openamber.step(ms=50)
    assert openamber.get_entity("state_machine_state_heat_cool") == "Defrosting", "State should be 'Defrosting'"

    # Tc is warm (34.0°C >= 32.0°C), defrost ends
    openamber.set_sensor("current_water_temperature_tc_sensor", 34.0)
    openamber.set_sensor("heat_cool_temperature_tc", 34.0)
    openamber.set_binary_sensor("defrost_active_sensor", False)
    openamber.advance_time(seconds=10, step_s=2)
    openamber.step(ms=50)

    # Mode should NOT have been boosted
    mode_after_defrost = int(openamber.get_entity("compressor_control_select") or 0)
    assert mode_after_defrost == mode_before_defrost, f"Mode should not boost when near target: expected {mode_before_defrost}, got {mode_after_defrost}"

    # Verify settle state
    assert openamber.get_entity("state_machine_state_heat_cool") == "Wait for state switch", "State should be 'Wait for state switch'"

    openamber.advance_time(seconds=300, step_s=20)
    openamber.step(ms=50)

    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running", "State should return to 'Compressor running'"


def test_heat_defrost_backup_heater_boost_low_outside_temp(clean_system):
    """
    Defrost Flow 3: Backup Heater Boost during severe cold (Ta <= threshold -3°C)
    1. Outside ambient temperature is very low: Ta = -5.0°C (<= -3.0°C threshold).
    2. Space heating is actively running.
    3. Defrost cycle initiates and completes.
    4. Because Ta <= -3.0°C, controller engages the backup heater automatically.
    5. State becomes 'Wait backup heater running' / 'Backup heater running'.
    6. Backup heater heats until water reaches setpoint + delta, then turns off.
    """
    openamber = clean_system

    openamber.set_select("thermostat_mode_select", "Extern")
    openamber.set_select("heat_mode_select", "Extern setpoint")
    openamber.set_number("manual_setpoint", 35.0)
    openamber.set_climate("pid_heat_temperature_control", target_temperature=35.0)
    openamber.set_number("compressor_stop_delta_heating", 5.0)
    openamber.set_number("defrost_backup_heater_boost_temperature_sensor", -3.0)
    openamber.set_sensor("temperature_outside_ta", -5.0)  # <= -3.0°C threshold
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

    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running", "State should be 'Compressor running'"
    assert openamber.get_entity("backup_heater_relay") is False, "Backup heater should initially be OFF"

    # Defrost initiates
    openamber.set_binary_sensor("defrost_active_sensor", True)
    openamber.advance_time(seconds=10, step_s=2)
    openamber.step(ms=50)
    assert openamber.get_entity("state_machine_state_heat_cool") == "Defrosting", "State should be 'Defrosting'"

    # Defrost completes
    openamber.set_binary_sensor("defrost_active_sensor", False)
    openamber.advance_time(seconds=10, step_s=2)
    openamber.step(ms=50)

    # Backup heater must activate because Ta (-5°C) <= threshold (-3°C)
    assert openamber.get_entity("backup_heater_relay") is True, "Backup heater must turn ON when Ta <= threshold after defrost"
    assert openamber.get_entity("state_machine_state_heat_cool") in (
        "Wait backup heater running", "Backup heater running"
    ), f"Expected backup heater state, got {openamber.get_entity('state_machine_state_heat_cool')}"


def test_heat_defrost_pump_p1_interruption_and_restart(clean_system):
    """
    Defrost Flow 4: Secondary Pump P1 shutoff during defrost and restart
    1. Secondary heating pump P1 is enabled (pump_p1_enabled = True).
    2. Space heating starts and activates pump P1 (pump_p1_relay_switch = True).
    3. Defrost cycle initiates (defrost_active_sensor = True).
    4. Controller turns OFF pump P1 during defrost (pump_p1_relay_switch = False).
    5. Defrost cycle ends (defrost_active_sensor = False).
    6. Controller turns ON pump P1 again (pump_p1_relay_switch = True).
    """
    openamber = clean_system

    openamber.set_select("thermostat_mode_select", "Extern")
    openamber.set_select("heat_mode_select", "Extern setpoint")
    openamber.set_number("manual_setpoint", 35.0)
    openamber.set_climate("pid_heat_temperature_control", target_temperature=35.0)
    openamber.set_select("heat_compressor_mode", "Maximaal")
    openamber.set_switch("pump_p1_enabled", True)
    openamber.set_switch("heat_demand_switch", True)
    openamber.set_sensor("current_water_temperature_tc_sensor", 28.0)
    openamber.set_sensor("heat_cool_temperature_tc", 28.0)
    openamber.set_sensor("outlet_temperature_tuo", 28.0)
    openamber.set_sensor("inlet_temperature_tui", 26.0)
    openamber.step(ms=50)

    # Advance time to start Space Heating pump and compressor
    openamber.advance_time(seconds=900, step_s=30)
    openamber.advance_time(seconds=140, step_s=10)
    openamber.advance_time(seconds=190, step_s=10)
    openamber.step(ms=50)

    # Pump P1 must be running
    assert openamber.get_entity("pump_p1_relay_switch") is True, "Pump P1 should be running during space heating"

    # Defrost initiates -> Pump P1 must stop
    openamber.set_binary_sensor("defrost_active_sensor", True)
    openamber.advance_time(seconds=10, step_s=2)
    openamber.step(ms=50)
    assert openamber.get_entity("state_machine_state_heat_cool") == "Defrosting", "State should be 'Defrosting'"
    assert openamber.get_entity("pump_p1_relay_switch") is False, "Pump P1 must stop during defrost"

    # Defrost ends -> Pump P1 must restart
    openamber.set_binary_sensor("defrost_active_sensor", False)
    openamber.advance_time(seconds=10, step_s=2)
    openamber.step(ms=50)
    assert openamber.get_entity("pump_p1_relay_switch") is True, "Pump P1 must restart after defrost completes"


def test_defrost_pump_p0_pid_defrost_pwm(clean_system):
    """
    Defrost Flow 5: Pump P0 Defrost PWM Duty Cycle
    1. Pump P0 PID speed control is enabled (pump_p0_pid_enabled = True).
    2. Defrost PWM duty cycle is set to 85.0% (pump_p0_pid_defrost_pwm = 85.0).
    3. Space heating is running.
    4. Defrost activates: controller applies 85% duty cycle (pump_control_pwm_number = 150.0).
    5. Defrost ends: pump returns to normal control.
    """
    openamber = clean_system

    openamber.set_select("thermostat_mode_select", "Extern")
    openamber.set_select("heat_mode_select", "Extern setpoint")
    openamber.set_number("manual_setpoint", 35.0)
    openamber.set_climate("pid_heat_temperature_control", target_temperature=35.0)
    openamber.set_select("heat_compressor_mode", "Maximaal")
    openamber.set_switch("pump_p0_pid_enabled", True)
    openamber.set_number("pump_p0_pid_defrost_pwm", 85.0)
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

    # Defrost initiates
    openamber.set_binary_sensor("defrost_active_sensor", True)
    openamber.advance_time(seconds=10, step_s=2)
    openamber.step(ms=50)

    # Duty cycle 85% corresponds to inverted speed value: ((85 * 10) * -1) + 1000 = 150.0
    expected_pwm_val = 150.0
    actual_pwm = float(openamber.get_entity("pump_control_pwm_number") or 0)
    assert actual_pwm == expected_pwm_val, (
        f"Pump PWM during defrost should be {expected_pwm_val} (85% duty cycle), got {actual_pwm}"
    )

    # Defrost ends
    openamber.set_binary_sensor("defrost_active_sensor", False)
    openamber.advance_time(seconds=10, step_s=2)
    openamber.step(ms=50)


def test_dhw_defrost_flow_and_settle_time(clean_system):
    """
    Defrost Flow 6: DHW Defrost & 5-minute settle time
    1. DHW tank calls for heating (Tw = 38°C < setpoint 50°C).
    2. DHW compressor and pump run in 'Compressor running'.
    3. Defrost activates (defrost_active_sensor = True).
    4. DHW state machine transitions to 'Defrosting'.
    5. Defrost ends (defrost_active_sensor = False).
    6. DHW state machine enters 5-minute settle period (COMPRESSOR_SETTLE_TIME_AFTER_DEFROST_S = 300s).
    7. After 300s, DHW state transitions back to 'Compressor running'.
    8. DHW heating completes when tank reaches 52°C.
    """
    openamber = clean_system

    # Step 1: Start DHW heating
    openamber.set_sensor("dhw_temperature_tw_sensor", 38.0)
    openamber.step(ms=50)
    assert openamber.get_entity("dhw_demand_active_sensor") is True, "DHW demand should be active"

    # Advance time for 3-way valve switch (60s) + pump wait/settle (130s) + compressor softstart (190s) = ~380-400s
    openamber.advance_time(seconds=400, step_s=20)
    openamber.step(ms=50)

    assert openamber.get_entity("state_machine_state_main") == "DHW", "Main state should be 'DHW'"
    assert openamber.get_entity("three_way_valve_dhw_switch") is True, "3-way valve should be on DHW"
    assert openamber.get_entity("state_machine_state_dhw") == "Compressor running", "DHW state should be 'Compressor running'"
    assert int(openamber.get_entity("compressor_control_select") or 0) > 0, "Compressor should run for DHW"

    # Step 2: Defrost activates during DHW heating
    openamber.set_binary_sensor("defrost_active_sensor", True)
    openamber.advance_time(seconds=10, step_s=2)
    openamber.step(ms=50)

    assert openamber.get_entity("state_machine_state_dhw") == "Defrosting", "DHW state should be 'Defrosting'"

    # Step 3: Defrost ends
    openamber.set_binary_sensor("defrost_active_sensor", False)
    openamber.advance_time(seconds=10, step_s=2)
    openamber.step(ms=50)

    # DHW enters settle state
    assert openamber.get_entity("state_machine_state_dhw") == "Wait for state switch", "DHW state should be 'Wait for state switch'"

    # Step 4: Advance virtual time 270s (< 300s settle time, 10s already elapsed)
    openamber.advance_time(seconds=270, step_s=20)
    openamber.step(ms=50)

    assert openamber.get_entity("state_machine_state_dhw") == "Wait for state switch", "DHW state should remain in 'Wait for state switch'"

    # Step 5: Advance remaining 30s (> 300s settle time)
    openamber.advance_time(seconds=30, step_s=10)
    openamber.step(ms=50)

    assert openamber.get_entity("state_machine_state_dhw") == "Compressor running", "DHW state should return to 'Compressor running'"

    # Step 6: DHW reaches target temperature
    openamber.set_sensor("dhw_temperature_tw_sensor", 52.0)
    openamber.step(ms=50)
    assert openamber.get_entity("dhw_demand_active_sensor") is False, "DHW demand should cease at target"

    # Advance past min-on time (600s) + valve switch (60s)
    openamber.advance_time(seconds=700, step_s=20)
    openamber.step(ms=50)

    assert openamber.get_entity("three_way_valve_dhw_switch") is False, "Valve should switch back to heating/cooling"
    assert openamber.get_entity("state_machine_state_main") == "Heat/Cool", "Main state should return to Heat/Cool"
