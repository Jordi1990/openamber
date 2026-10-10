"""Automated tests for Thermostat deadbands/defaults and DHW weekday schedules.

Covers settings:
1. Thermostat Deadbands and Setpoint Defaults:
   - thermostat_heat_deadband
   - thermostat_heat_overrun
   - thermostat_default_heat_setpoint
   - thermostat_cool_deadband
   - thermostat_cool_overrun
   - thermostat_default_cool_setpoint
2. DHW Schedule Weekday Switches:
   - dhw_schedule_monday_enabled_switch
   - dhw_schedule_tuesday_enabled_switch
   - dhw_schedule_wednesday_enabled_switch
   - dhw_schedule_thursday_enabled_switch
   - dhw_schedule_friday_enabled_switch
   - dhw_schedule_saturday_enabled_switch
   - dhw_schedule_sunday_enabled_switch
3. DHW Schedule Times:
   - dhw_start_time (via increment/decrement controls & label set_dhw_start_time_val)
   - dhw_end_time (via increment/decrement controls & label set_dhw_end_time_val)
"""

import pytest


def test_thermostat_deadband_and_default_setpoints(clean_system):
    """
    Verify Thermostat Deadband, Overrun, and Default setpoint numbers:
    - Setting numbers directly.
    - Verifying persistence and state queries.
    """
    openamber = clean_system

    # Heat deadband and overrun
    openamber.set_number("thermostat_heat_deadband", 0.4)
    openamber.set_number("thermostat_heat_overrun", 0.3)
    openamber.step(ms=50)
    assert float(openamber.get_entity("thermostat_heat_deadband") or 0) == pytest.approx(0.4, 0.05)
    assert float(openamber.get_entity("thermostat_heat_overrun") or 0) == pytest.approx(0.3, 0.05)

    # Cool deadband and overrun
    openamber.set_number("thermostat_cool_deadband", 0.6)
    openamber.set_number("thermostat_cool_overrun", 1.5)
    openamber.step(ms=50)
    assert float(openamber.get_entity("thermostat_cool_deadband") or 0) == pytest.approx(0.6, 0.05)
    assert float(openamber.get_entity("thermostat_cool_overrun") or 0) == pytest.approx(1.5, 0.05)

    # Default setpoints
    openamber.set_number("thermostat_default_heat_setpoint", 21.0)
    openamber.set_number("thermostat_default_cool_setpoint", 23.5)
    openamber.step(ms=50)
    assert float(openamber.get_entity("thermostat_default_heat_setpoint") or 0) == pytest.approx(21.0, 0.05)
    assert float(openamber.get_entity("thermostat_default_cool_setpoint") or 0) == pytest.approx(23.5, 0.05)

    # Reset
    openamber.set_number("thermostat_heat_deadband", 0.3)
    openamber.set_number("thermostat_heat_overrun", 0.2)
    openamber.set_number("thermostat_cool_deadband", 0.5)
    openamber.set_number("thermostat_cool_overrun", 3.0)
    openamber.set_number("thermostat_default_heat_setpoint", 20.5)
    openamber.set_number("thermostat_default_cool_setpoint", 24.0)
    openamber.step(ms=50)


def test_dhw_schedule_weekday_switches_and_time_controls(clean_system):
    """
    Verify DHW schedule weekday switches and time controls:
    1. Toggle individual weekday schedule switches Monday through Sunday.
    2. Verify DHW start time increment/decrement via UI widgets.
    3. Verify DHW end time increment/decrement via UI widgets.
    """
    openamber = clean_system

    weekdays = [
        "dhw_schedule_monday_enabled_switch",
        "dhw_schedule_tuesday_enabled_switch",
        "dhw_schedule_wednesday_enabled_switch",
        "dhw_schedule_thursday_enabled_switch",
        "dhw_schedule_friday_enabled_switch",
        "dhw_schedule_saturday_enabled_switch",
        "dhw_schedule_sunday_enabled_switch",
    ]

    # Test toggling each weekday switch
    for day_switch in weekdays:
        assert openamber.set_switch(day_switch, False) is True
        openamber.step(ms=50)
        assert openamber.get_entity(day_switch) is False, f"Failed turning off {day_switch}"

        assert openamber.set_switch(day_switch, True) is True
        openamber.step(ms=50)
        assert openamber.get_entity(day_switch) is True, f"Failed turning on {day_switch}"

    # Navigate to Settings -> Tapwater -> Schema to initialize UI labels
    assert openamber.click("nav_settings"), "Clicking nav_settings must succeed"
    openamber.step(ms=100)
    assert openamber.click("stab_dhw"), "Clicking stab_dhw must succeed"
    openamber.step(ms=100)
    assert openamber.click("dhw_stab_schema"), "Clicking dhw_stab_schema must succeed"
    openamber.step(ms=100)

    # Enable DHW schedule so time controls respond to clicks
    openamber.set_switch("dhw_schedule_enabled_switch", True)
    openamber.step(ms=50)

    # Initial start time label is "10:00"
    start_label = openamber.get_label("set_dhw_start_time_val")
    assert start_label is not None and "10" in start_label, f"Expected initial start time '10:00', got {start_label}"

    # Click increment start time -> +30 min (10:30)
    assert openamber.click("set_dhw_start_time_inc"), "Clicking set_dhw_start_time_inc must succeed"
    openamber.step(ms=100)
    assert openamber.get_label("set_dhw_start_time_val") == "10:30", "Start time label should update to 10:30 after increment"

    # Click decrement start time -> -30 min (back to 10:00)
    assert openamber.click("set_dhw_start_time_dec"), "Clicking set_dhw_start_time_dec must succeed"
    openamber.step(ms=100)
    assert openamber.get_label("set_dhw_start_time_val") == "10:00", "Start time label should return to 10:00 after decrement"

    # Initial end time label is "17:00"
    end_label = openamber.get_label("set_dhw_end_time_val")
    assert end_label is not None and "17" in end_label, f"Expected initial end time '17:00', got {end_label}"

    # Click increment end time -> +30 min (17:30)
    assert openamber.click("set_dhw_end_time_inc"), "Clicking set_dhw_end_time_inc must succeed"
    openamber.step(ms=100)
    assert openamber.get_label("set_dhw_end_time_val") == "17:30", "End time label should update to 17:30 after increment"

    # Click decrement end time -> -30 min (back to 17:00)
    assert openamber.click("set_dhw_end_time_dec"), "Clicking set_dhw_end_time_dec must succeed"
    openamber.step(ms=100)
    assert openamber.get_label("set_dhw_end_time_val") == "17:00", "End time label should return to 17:00 after decrement"
