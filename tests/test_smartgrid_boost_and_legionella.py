"""Automated tests for SmartGrid (SG Ready) boost temperatures and Legionella prevention.

Covers settings:
1. sg_ready_heating_boost_temperature_number:
   Offsets the space heating setpoint when SG Ready Boost or Max Boost is active.
2. sg_ready_dhw_boost_temperature_number:
   Offsets the DHW setpoint when SG Ready Boost or Max Boost is active.
3. legio_enabled_switch:
   Enables or disables legionella prevention runs. Turning OFF immediately aborts an active legionella run.
4. legio_target_temperature_number:
   Overrides DHW target setpoint during a legionella run, and terminates the run once reached.
5. legio_repeat_days_number:
   Configures interval in days between legionella runs.
"""

import pytest


def test_sg_ready_heating_and_dhw_boost(clean_system):
    """
    Verify SG Ready Boost Temperature Numbers:
    1. In normal operation without SG boost, heating setpoint is baseline (e.g. 35°C)
       and DHW setpoint is baseline (e.g. 50°C).
    2. Activate SG Ready Boost (sg_ready_boost_mode_active_sensor = True).
    3. Heating setpoint increases by sg_ready_heating_boost_temperature_number (e.g. +4.0°C -> 39.0°C).
    4. DHW setpoint increases by sg_ready_dhw_boost_temperature_number (e.g. +6.0°C -> 56.0°C).
    5. Dynamically change boost numbers and verify immediate recalculation.
    6. Deactivate SG Ready Boost -> setpoints return to baseline.
    """
    openamber = clean_system

    openamber.set_select("heat_mode_select", "Extern setpoint")
    openamber.set_number("manual_setpoint", 35.0)
    openamber.set_number("dhw_setpoint_temperature", 50.0)
    openamber.set_number("sg_ready_heating_boost_temperature_number", 4.0)
    openamber.set_number("sg_ready_dhw_boost_temperature_number", 6.0)
    openamber.set_binary_sensor("sg_ready_boost_mode_active_sensor", False)
    openamber.set_binary_sensor("sg_ready_max_boost_mode_active_sensor", False)
    openamber.advance_time(seconds=6, step_s=1)
    openamber.step(ms=100)

    # Baseline setpoints
    heat_sp = float(openamber.get_entity("current_setpoint") or 0)
    dhw_sp = float(openamber.get_entity("current_dhw_setpoint_sensor") or 0)
    assert heat_sp == 35.0, f"Expected baseline heating setpoint 35.0°C, got {heat_sp}"
    assert dhw_sp == 50.0, f"Expected baseline DHW setpoint 50.0°C, got {dhw_sp}"

    # Activate SG Ready Boost via sg_ready_mode_select
    openamber.set_select("sg_ready_mode_select", "Boost")
    openamber.advance_time(seconds=6, step_s=1)
    openamber.step(ms=100)

    heat_sp = float(openamber.get_entity("current_setpoint") or 0)
    dhw_sp = float(openamber.get_entity("current_dhw_setpoint_sensor") or 0)
    assert heat_sp == 39.0, f"Expected boosted heating setpoint 39.0°C (35+4), got {heat_sp}"
    assert dhw_sp == 56.0, f"Expected boosted DHW setpoint 56.0°C (50+6), got {dhw_sp}"

    # Change boost parameters dynamically
    openamber.set_number("sg_ready_heating_boost_temperature_number", 5.0)
    openamber.set_number("sg_ready_dhw_boost_temperature_number", 8.0)
    openamber.advance_time(seconds=6, step_s=1)
    openamber.step(ms=100)

    heat_sp = float(openamber.get_entity("current_setpoint") or 0)
    dhw_sp = float(openamber.get_entity("current_dhw_setpoint_sensor") or 0)
    assert heat_sp == 40.0, f"Expected updated boosted heating setpoint 40.0°C (35+5), got {heat_sp}"
    assert dhw_sp == 58.0, f"Expected updated boosted DHW setpoint 58.0°C (50+8), got {dhw_sp}"

    # Deactivate boost
    openamber.set_select("sg_ready_mode_select", "Normaal")
    openamber.advance_time(seconds=6, step_s=1)
    openamber.step(ms=100)

    heat_sp = float(openamber.get_entity("current_setpoint") or 0)
    dhw_sp = float(openamber.get_entity("current_dhw_setpoint_sensor") or 0)
    assert heat_sp == 35.0, f"Expected return to baseline heating setpoint 35.0°C, got {heat_sp}"
    assert dhw_sp == 50.0, f"Expected return to baseline DHW setpoint 50.0°C, got {dhw_sp}"


def test_sg_ready_heating_and_dhw_max_boost(clean_system):
    """
    Verify SG Ready Max Boost:
    1. Baseline heating (35°C) and DHW (50°C) setpoints without boost.
    2. Activate Max Boost via sg_ready_mode_select ('Max boost').
    3. Verify sg_ready_max_boost_mode_active_sensor is True and setpoints boosted.
    4. Deactivate Max Boost -> returns to baseline.
    5. Activate Max Boost via wired contacts (sg_ready_a_active_sensor=True, sg_ready_b_active_sensor=True).
    6. Verify sg_ready_max_boost_mode_active_sensor is True and setpoints boosted.
    7. Reset wired contacts -> returns to baseline.
    """
    openamber = clean_system

    openamber.set_select("heat_mode_select", "Extern setpoint")
    openamber.set_number("manual_setpoint", 35.0)
    openamber.set_number("dhw_setpoint_temperature", 50.0)
    openamber.set_number("sg_ready_heating_boost_temperature_number", 4.0)
    openamber.set_number("sg_ready_dhw_boost_temperature_number", 6.0)
    openamber.set_select("sg_ready_mode_select", "Normaal")
    openamber.set_binary_sensor("sg_ready_a_active_sensor", False)
    openamber.set_binary_sensor("sg_ready_b_active_sensor", False)
    openamber.advance_time(seconds=6, step_s=1)
    openamber.step(ms=100)

    # Verify baseline
    assert openamber.get_entity("sg_ready_max_boost_mode_active_sensor") is False
    heat_sp = float(openamber.get_entity("current_setpoint") or 0)
    dhw_sp = float(openamber.get_entity("current_dhw_setpoint_sensor") or 0)
    assert heat_sp == 35.0
    assert dhw_sp == 50.0

    # 1. Activate Max Boost via select entity option "Max boost"
    openamber.set_select("sg_ready_mode_select", "Max boost")
    openamber.advance_time(seconds=6, step_s=1)
    openamber.step(ms=100)

    assert openamber.get_entity("sg_ready_max_boost_mode_active_sensor") is True, (
        "sg_ready_max_boost_mode_active_sensor should be True when mode is 'Max boost'"
    )
    heat_sp = float(openamber.get_entity("current_setpoint") or 0)
    dhw_sp = float(openamber.get_entity("current_dhw_setpoint_sensor") or 0)
    assert heat_sp == 39.0, f"Expected boosted heating setpoint 39.0°C (35+4), got {heat_sp}"
    assert dhw_sp == 56.0, f"Expected boosted DHW setpoint 56.0°C (50+6), got {dhw_sp}"

    # Deactivate Max Boost
    openamber.set_select("sg_ready_mode_select", "Normaal")
    openamber.advance_time(seconds=6, step_s=1)
    openamber.step(ms=100)

    assert openamber.get_entity("sg_ready_max_boost_mode_active_sensor") is False
    heat_sp = float(openamber.get_entity("current_setpoint") or 0)
    dhw_sp = float(openamber.get_entity("current_dhw_setpoint_sensor") or 0)
    assert heat_sp == 35.0
    assert dhw_sp == 50.0

    # 2. Activate Max Boost via wired contact inputs (A=True, B=True)
    openamber.set_binary_sensor("sg_ready_a_active_sensor", True)
    openamber.set_binary_sensor("sg_ready_b_active_sensor", True)
    openamber.advance_time(seconds=6, step_s=1)
    openamber.step(ms=100)

    assert openamber.get_entity("sg_ready_max_boost_mode_active_sensor") is True, (
        "sg_ready_max_boost_mode_active_sensor should be True when both A and B inputs are active"
    )
    heat_sp = float(openamber.get_entity("current_setpoint") or 0)
    dhw_sp = float(openamber.get_entity("current_dhw_setpoint_sensor") or 0)
    assert heat_sp == 39.0
    assert dhw_sp == 56.0

    # Reset wired contact inputs
    openamber.set_binary_sensor("sg_ready_a_active_sensor", False)
    openamber.set_binary_sensor("sg_ready_b_active_sensor", False)
    openamber.advance_time(seconds=6, step_s=1)
    openamber.step(ms=100)

    assert openamber.get_entity("sg_ready_max_boost_mode_active_sensor") is False


def test_sg_ready_max_boost_triggers_backup_heater_during_heating(clean_system):
    """
    Grid Flow:
    1. Space heating is actively running in COMPRESSOR_RUNNING with backup heater OFF.
    2. SG Ready Max Boost activates (sg_ready_mode_select = 'Max boost').
    3. Controller recognizes max boost condition and immediately activates backup heater (backup_heater_relay = True).
    4. State transitions to WAIT_BACKUP_HEATER_RUNNING / BACKUP_HEATER_RUNNING.
    5. Disabling max boost and satisfying setpoint turns off backup heater.
    """
    openamber = clean_system

    openamber.set_select("thermostat_mode_select", "Extern")
    openamber.set_select("heat_mode_select", "Extern setpoint")
    openamber.set_number("manual_setpoint", 35.0)
    openamber.set_climate("pid_heat_temperature_control", target_temperature=35.0)
    openamber.set_number("compressor_start_delta_heating", 2.0)
    openamber.set_number("compressor_stop_delta_heating", 2.0)
    openamber.set_select("heat_compressor_mode", "Maximaal")
    openamber.set_select("backup_heating_mode", "Intern verwarmingselement")
    openamber.set_select("sg_ready_mode_select", "Normaal")
    openamber.set_sensor("current_water_temperature_tc_sensor", 28.0)
    openamber.set_sensor("heat_cool_temperature_tc", 28.0)
    openamber.set_sensor("outlet_temperature_tuo", 28.0)
    openamber.set_sensor("inlet_temperature_tui", 26.0)
    openamber.step(ms=50)

    # Start heating demand
    openamber.set_binary_sensor("external_heat_demand_wired", True)
    openamber.step(ms=100)

    # Advance time through pump interval (900s) + pump settle (130s) + softstart (190s)
    openamber.advance_time(seconds=900, step_s=30)
    openamber.advance_time(seconds=130, step_s=10)
    openamber.advance_time(seconds=190, step_s=10)
    openamber.step(ms=100)

    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"
    assert int(openamber.get_entity("compressor_control_select") or 0) > 0
    assert openamber.get_entity("backup_heater_relay") is False, "Backup heater should initially be OFF"

    # Advance past compressor minimum on-time (600s)
    openamber.advance_time(seconds=610, step_s=20)
    openamber.step(ms=100)

    # Activate SG Ready Max Boost
    openamber.set_select("sg_ready_mode_select", "Max boost")
    openamber.advance_time(seconds=10, step_s=2)
    openamber.step(ms=100)

    assert openamber.get_entity("sg_ready_max_boost_mode_active_sensor") is True
    assert openamber.get_entity("backup_heater_relay") is True, (
        "SG Ready Max Boost must engage backup_heater_relay during active space heating"
    )
    assert openamber.get_entity("backup_heater_active_sensor") is True

    # Restore normal SG mode
    openamber.set_select("sg_ready_mode_select", "Normaal")
    openamber.advance_time(seconds=10, step_s=2)
    openamber.step(ms=100)

    # Water warms up to setpoint + stop delta (35 + 2 = 37°C) -> 38.0°C
    openamber.set_sensor("current_water_temperature_tc_sensor", 38.0)
    openamber.set_sensor("heat_cool_temperature_tc", 38.0)
    openamber.set_sensor("outlet_temperature_tuo", 38.0)
    openamber.advance_time(seconds=10, step_s=2)
    openamber.step(ms=100)

    assert openamber.get_entity("backup_heater_relay") is False, "Backup heater must turn OFF once satisfied"

    # Cleanup
    openamber.set_binary_sensor("external_heat_demand_wired", False)
    openamber.step(ms=50)


def test_legionella_settings_and_prevention_flow(clean_system):
    """
    Verify Legionella Settings:
    1. Verify legio_repeat_days_number configuration (e.g. 7 days, 14 days).
    2. Set legio_target_temperature_number = 65.0°C.
    3. Trigger legionella run (dhw_legionella_run_active_sensor = True).
    4. Verify DHW setpoint switches to legio_target_temperature_number (65.0°C).
    5. Verify updating legio_target_temperature_number updates the setpoint.
    6. Verify disabling and enabling legio_enabled_switch controls legionella configuration.
    """
    openamber = clean_system

    openamber.set_switch("legio_enabled_switch", True)
    openamber.set_number("legio_repeat_days_number", 7.0)
    openamber.set_number("legio_target_temperature_number", 65.0)
    openamber.set_number("dhw_setpoint_temperature", 50.0)
    openamber.set_sensor("dhw_temperature_tw_sensor", 45.0)
    openamber.advance_time(seconds=6, step_s=1)
    openamber.step(ms=100)

    # Verify repeat days number
    assert float(openamber.get_entity("legio_repeat_days_number") or 0) == 7.0
    openamber.set_number("legio_repeat_days_number", 14.0)
    openamber.step(ms=50)
    assert float(openamber.get_entity("legio_repeat_days_number") or 0) == 14.0

    # Normal DHW setpoint before legionella
    dhw_sp = float(openamber.get_entity("current_dhw_setpoint_sensor") or 0)
    assert dhw_sp == 50.0

    # Start legionella run
    openamber.set_binary_sensor("dhw_legionella_run_active_sensor", True)
    openamber.advance_time(seconds=6, step_s=1)
    openamber.step(ms=100)

    # DHW setpoint should now be 65.0°C
    dhw_sp = float(openamber.get_entity("current_dhw_setpoint_sensor") or 0)
    assert dhw_sp == 65.0, f"Expected legionella target 65.0°C, got {dhw_sp}"

    # Change legio_target_temperature_number to 62.0°C
    openamber.set_number("legio_target_temperature_number", 62.0)
    openamber.advance_time(seconds=6, step_s=1)
    openamber.step(ms=100)
    dhw_sp = float(openamber.get_entity("current_dhw_setpoint_sensor") or 0)
    assert dhw_sp == 62.0, f"Expected updated legionella target 62.0°C, got {dhw_sp}"

    # Disabling legionella must abort the active run. CheckLegionellaCycle runs in the 5s controller
    # update (only in Heat/Cool or DHW main state; a valve switch can take 60s), so advance time.
    openamber.set_switch("legio_enabled_switch", False)
    openamber.advance_time(seconds=70, step_s=5)
    openamber.step(ms=100)
    assert openamber.get_entity("dhw_legionella_run_active_sensor") is False, (
        "Turning legio_enabled_switch OFF must abort the active legionella run"
    )
    dhw_sp = float(openamber.get_entity("current_dhw_setpoint_sensor"))
    assert dhw_sp == 50.0, f"DHW setpoint must fall back to 50.0°C after abort, got {dhw_sp}"

    # Cleanup
    openamber.set_switch("legio_enabled_switch", True)
    openamber.set_binary_sensor("dhw_legionella_run_active_sensor", False)
    openamber.advance_time(seconds=6, step_s=1)
    openamber.step(ms=50)
    assert float(openamber.get_entity("current_dhw_setpoint_sensor") or 0) == 50.0
