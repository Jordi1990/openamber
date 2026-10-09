"""Tests for cooling demand flows and priority conflict resolution."""

import pytest


def test_external_cooling_demand_happy_flow(openamber):
    """
    Happy Flow:
    1. Thermostat mode is 'Extern'.
    2. External cooling thermostat calls for cooling (external_cool_demand_wired = True).
    3. Verify cool demand activates (cool_demand_active_sensor = True).
    4. Verify UI reflects cooling demand: navbar snowflake icon (nav_status_cool_icon) is VISIBLE.
    5. External cooling call ends -> snowflake icon is HIDDEN.
    """
    openamber.set_select("thermostat_mode_select", "Extern")
    openamber.set_binary_sensor("sg_ready_block_mode_active_sensor", False)
    openamber.set_binary_sensor("error_active", False)
    openamber.set_binary_sensor("external_heat_demand_wired", False)
    openamber.set_switch("heat_demand_switch", False)
    openamber.set_binary_sensor("external_cool_demand_wired", False)
    openamber.set_switch("cool_demand_switch", False)
    openamber.step(ms=150)

    assert openamber.get_entity("cool_demand_active_sensor") is False
    assert openamber.is_hidden("nav_status_cool_icon")

    # External cooling demand closes contact
    openamber.set_binary_sensor("external_cool_demand_wired", True)
    openamber.step(ms=150)

    # Cooling demand activates
    assert openamber.get_entity("cool_demand_active_sensor") is True
    assert openamber.is_visible("nav_status_cool_icon")

    # Cooling demand deactivates
    openamber.set_binary_sensor("external_cool_demand_wired", False)
    openamber.step(ms=150)

    assert openamber.get_entity("cool_demand_active_sensor") is False
    assert openamber.is_hidden("nav_status_cool_icon")


def test_cooling_suppressed_when_heating_active(openamber):
    """
    Priority Flow:
    1. Heating demand is actively running.
    2. Cooling thermostat simultaneously calls for cooling.
    3. Verify heating priority: cooling demand remains False.
    """
    openamber.set_select("thermostat_mode_select", "Extern")
    openamber.set_binary_sensor("external_heat_demand_wired", True)
    openamber.step(ms=150)

    assert openamber.get_entity("heat_demand_active_sensor") is True

    # Cooling call while heating is active
    openamber.set_binary_sensor("external_cool_demand_wired", True)
    openamber.step(ms=150)

    # Cooling must be suppressed
    assert openamber.get_entity("cool_demand_active_sensor") is False
    assert openamber.is_hidden("nav_status_cool_icon")

    # Cleanup
    openamber.set_binary_sensor("external_heat_demand_wired", False)
    openamber.set_binary_sensor("external_cool_demand_wired", False)
    openamber.step(ms=150)
