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

    # Verify toggling legio_enabled_switch
    openamber.set_switch("legio_enabled_switch", False)
    openamber.step(ms=100)
    assert openamber.get_entity("legio_enabled_switch") is False

    openamber.set_switch("legio_enabled_switch", True)
    openamber.step(ms=100)
    assert openamber.get_entity("legio_enabled_switch") is True

    # Cleanup
    openamber.set_binary_sensor("dhw_legionella_run_active_sensor", False)
    openamber.advance_time(seconds=6, step_s=1)
    openamber.step(ms=50)
    assert float(openamber.get_entity("current_dhw_setpoint_sensor") or 0) == 50.0
