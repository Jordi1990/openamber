"""Automated tests for DHW enable/disable, restart hysteresis delta, and schedule settings.

Covers:
1. dhw_enabled_switch:
   - Suppresses DHW demand even when tank temperature is cold (Tw << setpoint).
   - Aborts active DHW heating when turned OFF mid-cycle.
   - Restores normal DHW demand when turned back ON.
2. dhw_restart_dhw_delta:
   - Threshold shifts accurately with different delta values (e.g. 4.0°C vs 8.0°C).
   - Hysteresis preserves demand during active heating above the restart threshold
     until tank reaches full setpoint temperature.
3. DHW schedule settings:
   - dhw_schedule_enabled_switch and weekday switches persist and toggle correctly.
"""

import pytest


def test_dhw_enabled_switch_suppresses_demand_and_shuts_down_running_dhw(clean_system):
    """
    Verify dhw_enabled_switch:
    1. With dhw_enabled_switch = False, cold tank does NOT trigger demand.
    2. Turning switch ON triggers demand immediately.
    3. System starts DHW cycle (valve switches, compressor runs).
    4. Turning switch OFF mid-cycle immediately cancels demand and shuts down DHW.
    """
    openamber = clean_system

    # Set cold tank with DHW disabled
    openamber.set_number("dhw_setpoint_temperature", 50.0)
    openamber.set_number("dhw_restart_dhw_delta", 5.0)
    openamber.set_sensor("dhw_temperature_tw_sensor", 38.0)
    openamber.set_switch("dhw_enabled_switch", False)
    openamber.step(ms=100)

    # Must NOT have demand when DHW switch is disabled
    assert openamber.get_entity("dhw_demand_active_sensor") is False, "DHW demand should be inactive when dhw_enabled_switch is OFF"
    assert openamber.get_entity("three_way_valve_dhw_switch") is False, "3-way valve should not switch to DHW when disabled"

    # Enable DHW switch -> demand must activate immediately
    openamber.set_switch("dhw_enabled_switch", True)
    openamber.step(ms=100)
    assert openamber.get_entity("dhw_demand_active_sensor") is True, "DHW demand should activate immediately when enabled"

    # Advance time to allow 3-way valve to switch (60s) and compressor to start
    openamber.advance_time(seconds=140, step_s=10)
    openamber.step(ms=100)
    assert openamber.get_entity("three_way_valve_dhw_switch") is True, "3-way valve should be in DHW position"
    assert int(openamber.get_entity("compressor_control_select") or 0) > 0, "Compressor should run for DHW"

    # Turn DHW switch OFF mid-cycle
    openamber.set_switch("dhw_enabled_switch", False)
    openamber.step(ms=100)

    # Demand drops immediately
    assert openamber.get_entity("dhw_demand_active_sensor") is False, "DHW demand should drop immediately when switched OFF"

    # Advance time past compressor minimum on time (600s) + valve switch
    openamber.advance_time(seconds=640, step_s=20)
    openamber.step(ms=100)

    # System must shut down DHW compressor
    assert int(openamber.get_entity("compressor_control_select") or 0) == 0, "Compressor should stop after min-on-time"

    # Cleanup: restore DHW enabled
    openamber.set_switch("dhw_enabled_switch", True)
    openamber.step(ms=100)


def test_dhw_restart_delta_hysteresis_behavior(clean_system):
    """
    Verify dhw_restart_dhw_delta:
    1. Setpoint 50°C, delta 4°C -> starts when Tw < 46°C.
    2. At 47°C -> no demand.
    3. At 45.5°C -> demand starts.
    4. System heats: Tw rises to 48°C (above 46°C threshold) -> demand remains active (hysteresis).
    5. Tw reaches setpoint 50.5°C -> demand shuts off.
    6. Change delta to 8°C -> starts when Tw < 42°C.
    7. At 45°C -> no demand (unlike 4°C delta).
    8. At 41.5°C -> demand starts.
    """
    openamber = clean_system

    openamber.set_switch("dhw_enabled_switch", True)
    openamber.set_number("dhw_setpoint_temperature", 50.0)
    openamber.set_number("dhw_restart_dhw_delta", 4.0)

    # 1. Tw = 47.0°C (above 50 - 4 = 46.0°C) -> No demand
    openamber.set_sensor("dhw_temperature_tw_sensor", 47.0)
    openamber.step(ms=100)
    assert openamber.get_entity("dhw_demand_active_sensor") is False, "47.0°C should not trigger demand for 4°C delta"

    # 2. Tw drops to 45.5°C (< 46.0°C) -> Demand activates
    openamber.set_sensor("dhw_temperature_tw_sensor", 45.5)
    openamber.step(ms=100)
    assert openamber.get_entity("dhw_demand_active_sensor") is True, "45.5°C should trigger demand for 4°C delta"

    # Start DHW heating
    openamber.advance_time(seconds=140, step_s=10)
    openamber.step(ms=100)
    assert openamber.get_entity("three_way_valve_dhw_switch") is True, "3-way valve should switch to DHW"
    assert openamber.get_entity("dhw_active") is True, "DHW should be active"

    # 3. Water warms up to 48.0°C (above 46°C threshold, but below setpoint 50°C)
    openamber.set_sensor("dhw_temperature_tw_sensor", 48.0)
    openamber.step(ms=100)
    assert openamber.get_entity("dhw_demand_active_sensor") is True, (
        "Hysteresis must keep DHW demand active until setpoint is satisfied"
    )

    # 4. Water reaches setpoint (50.5°C) -> Demand turns OFF
    openamber.set_sensor("dhw_temperature_tw_sensor", 50.5)
    openamber.step(ms=100)
    assert openamber.get_entity("dhw_demand_active_sensor") is False, "Demand should turn OFF at setpoint"

    # Settle shutdown
    openamber.advance_time(seconds=640, step_s=20)
    openamber.step(ms=100)

    # 5. Test larger delta: 8.0°C (threshold = 50.0 - 8.0 = 42.0°C)
    openamber.set_number("dhw_restart_dhw_delta", 8.0)
    openamber.set_sensor("dhw_temperature_tw_sensor", 45.0)
    openamber.step(ms=100)
    assert openamber.get_entity("dhw_demand_active_sensor") is False, (
        "With 8.0°C delta, 45.0°C must NOT trigger DHW demand (threshold is 42.0°C)"
    )

    # 6. Tw drops to 41.5°C (< 42.0°C) -> Demand activates
    openamber.set_sensor("dhw_temperature_tw_sensor", 41.5)
    openamber.step(ms=100)
    assert openamber.get_entity("dhw_demand_active_sensor") is True, "41.5°C should trigger demand with 8°C delta"


def test_dhw_schedule_settings_configuration(clean_system):
    """
    Verify DHW schedule configuration entities:
    - dhw_schedule_enabled_switch
    - Daily schedule switches for Monday through Sunday
    """
    openamber = clean_system

    days = ["monday", "tuesday", "wednesday", "thursday", "friday", "saturday", "sunday"]

    # Toggle master schedule switch
    openamber.set_switch("dhw_schedule_enabled_switch", True)
    openamber.step(ms=50)
    assert openamber.get_entity("dhw_schedule_enabled_switch") is True, "Schedule master switch should be True"

    # Turn all weekdays off, then specific ones on
    for day in days:
        openamber.set_switch(f"dhw_schedule_{day}_enabled_switch", False)
    openamber.step(ms=50)

    for day in days:
        assert openamber.get_entity(f"dhw_schedule_{day}_enabled_switch") is False, f"Expected {day} switch to be False"

    # Enable specific days: Monday, Wednesday, Friday
    active_days = ["monday", "wednesday", "friday"]
    for day in active_days:
        openamber.set_switch(f"dhw_schedule_{day}_enabled_switch", True)
    openamber.step(ms=50)

    for day in days:
        expected = day in active_days
        actual = openamber.get_entity(f"dhw_schedule_{day}_enabled_switch")
        assert actual is expected, f"Expected {day} to be {expected}, got {actual}"

    # Cleanup: restore schedule switch to disabled
    openamber.set_switch("dhw_schedule_enabled_switch", False)
    for day in days:
        openamber.set_switch(f"dhw_schedule_{day}_enabled_switch", True)
    openamber.step(ms=50)


def test_dhw_schedule_suppresses_demand_when_outside_active_days(clean_system):
    """
    Verify that when DHW schedule is enabled and all schedule days are disabled:
    1. A cold DHW tank (Tw = 38°C < setpoint 50°C - delta 5°C) does NOT generate DHW demand.
    2. Once schedule is turned OFF (or days enabled), DHW demand activates immediately.
    """
    openamber = clean_system

    days = ["monday", "tuesday", "wednesday", "thursday", "friday", "saturday", "sunday"]

    openamber.set_number("dhw_setpoint_temperature", 50.0)
    openamber.set_number("dhw_restart_dhw_delta", 5.0)
    openamber.set_sensor("dhw_temperature_tw_sensor", 38.0)

    # Enable schedule and disable all days
    openamber.set_switch("dhw_schedule_enabled_switch", True)
    for day in days:
        openamber.set_switch(f"dhw_schedule_{day}_enabled_switch", False)
    openamber.step(ms=100)

    # Schedule should be inactive and suppress DHW demand
    assert openamber.get_entity("dhw_schedule_active_sensor") is False, "Schedule sensor should be False when no days enabled"
    assert openamber.get_entity("dhw_demand_active_sensor") is False, (
        "DHW demand must be suppressed when outside schedule window even if water is cold"
    )

    # Disabling schedule master switch restores 24/7 DHW demand
    openamber.set_switch("dhw_schedule_enabled_switch", False)
    openamber.step(ms=100)

    assert openamber.get_entity("dhw_demand_active_sensor") is True, (
        "DHW demand must immediately activate once schedule is disabled"
    )

    # Cleanup
    for day in days:
        openamber.set_switch(f"dhw_schedule_{day}_enabled_switch", True)
    openamber.step(ms=50)
