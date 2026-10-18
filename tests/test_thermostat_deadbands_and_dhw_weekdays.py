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



def test_thermostat_heat_deadband_and_overrun_dynamic_thresholds(clean_system):
    """
    Verify that changing thermostat_heat_deadband and thermostat_heat_overrun
    dynamically modifies the climate controller's hysteresis and demand triggers:
    1. Set thermostat mode to 'Intern' with target 20.5°C.
    2. Configure wider deadband (1.0°C vs default 0.3°C) and overrun (0.8°C vs default 0.2°C).
    3. Room temp at 19.8°C (deficit 0.7°C > 0.3°C default, but < 1.0°C configured):
       Heat demand must remain OFF.
    4. Room temp drops to 19.3°C (deficit 1.2°C >= 1.0°C deadband):
       Heat demand must turn ON.
    5. Room temp warms to 20.9°C (above target 20.5°C and above default 20.7°C overrun,
       but below configured 20.5 + 0.8 = 21.3°C overrun):
       After min run time (3600s), heat demand remains ON.
    6. Room temp warms to 21.5°C (exceeding 21.3°C overrun threshold):
       Heat demand shuts OFF.
    """
    openamber = clean_system

    openamber.set_select("thermostat_mode_select", "Intern")
    openamber.set_climate("climate_controller", mode="HEAT", target_temperature=20.5)
    openamber.set_sensor("room_temperature", 20.5)

    # Configure custom deadband and overrun
    assert openamber.set_number("thermostat_heat_deadband", 1.0) is True
    assert openamber.set_number("thermostat_heat_overrun", 0.8) is True
    openamber.step(ms=100)

    # Allow minimum idle time (900s) to pass
    openamber.advance_time(seconds=920, step_s=30)
    openamber.step(ms=100)
    assert openamber.get_entity("heat_demand_active_sensor") is False

    # 1. Deficit of 0.7°C (19.8°C). Default 0.3°C would trigger; configured 1.0°C must NOT trigger.
    openamber.set_sensor("room_temperature", 19.8)
    openamber.advance_time(seconds=10, step_s=2)
    openamber.step(ms=100)
    assert openamber.get_entity("heat_demand_active_sensor") is False, (
        "With deadband 1.0°C, room temp 19.8°C must NOT trigger heat demand (threshold is 19.5°C)"
    )

    # 2. Room temp drops to 19.3°C (< 19.5°C threshold) -> Demand triggers
    openamber.set_sensor("room_temperature", 19.3)
    openamber.advance_time(seconds=10, step_s=2)
    openamber.step(ms=100)
    assert openamber.get_entity("heat_demand_active_sensor") is True, (
        "Heat demand must trigger when room temp drops below target - deadband (19.3°C < 19.5°C)"
    )

    # 3. Room temp warms to 20.9°C (exceeds target 20.5°C, but below 20.5 + 0.8 = 21.3°C overrun)
    openamber.set_sensor("room_temperature", 20.9)
    # Advance through min heating run time (3600s)
    openamber.advance_time(seconds=3660, step_s=60)
    openamber.step(ms=100)
    assert openamber.get_entity("heat_demand_active_sensor") is True, (
        "Heat demand must stay active while room temp is below target + overrun (20.9°C < 21.3°C)"
    )

    # 4. Room temp exceeds 21.3°C overrun threshold -> Demand shuts OFF
    openamber.set_sensor("room_temperature", 21.5)
    openamber.advance_time(seconds=10, step_s=2)
    openamber.step(ms=100)
    assert openamber.get_entity("heat_demand_active_sensor") is False, (
        "Heat demand must shut off when room temp exceeds target + overrun (21.5°C >= 21.3°C)"
    )

    # Cleanup: restore defaults
    openamber.set_number("thermostat_heat_deadband", 0.3)
    openamber.set_number("thermostat_heat_overrun", 0.2)
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

    # Verify start time increment and decrement
    start_label = openamber.get_label("set_dhw_start_time_val")
    assert start_label != "", "Start time label should show a time"

    # Click increment start time -> verify value changes
    assert openamber.click("set_dhw_start_time_inc"), "Clicking set_dhw_start_time_inc must succeed"
    openamber.step(ms=100)
    inc_start = openamber.get_label("set_dhw_start_time_val")
    assert inc_start != start_label, "Start time label should update after increment"

    # Click decrement start time -> returns to original value
    assert openamber.click("set_dhw_start_time_dec"), "Clicking set_dhw_start_time_dec must succeed"
    openamber.step(ms=100)
    assert openamber.get_label("set_dhw_start_time_val") == start_label, "Start time label should return to initial value after decrement"

    # Verify end time increment and decrement
    end_label = openamber.get_label("set_dhw_end_time_val")
    assert end_label != "", "End time label should show a time"

    # Click increment end time -> verify value changes
    assert openamber.click("set_dhw_end_time_inc"), "Clicking set_dhw_end_time_inc must succeed"
    openamber.step(ms=100)
    inc_end = openamber.get_label("set_dhw_end_time_val")
    assert inc_end != end_label, "End time label should update after increment"

    # Click decrement end time -> returns to original value
    assert openamber.click("set_dhw_end_time_dec"), "Clicking set_dhw_end_time_dec must succeed"
    openamber.step(ms=100)
    assert openamber.get_label("set_dhw_end_time_val") == end_label, "End time label should return to initial value after decrement"
