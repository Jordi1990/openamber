"""Tests for cooling demand flows and priority conflict resolution."""

import pytest


"""Tests for cooling demand flows and priority conflict resolution."""

import pytest


def test_external_cooling_demand_happy_flow(clean_system):
    """
    Happy Flow:
    1. Thermostat mode is 'Extern'.
    2. External cooling thermostat calls for cooling (external_cool_demand_wired = True).
    3. Verify cool demand activates (cool_demand_active_sensor = True).
    4. Verify UI reflects cooling demand: navbar snowflake icon (nav_status_cool_icon) is VISIBLE.
    5. External cooling call ends -> snowflake icon is HIDDEN.
    """
    openamber = clean_system

    assert openamber.set_select("thermostat_mode_select", "Extern")
    assert openamber.set_binary_sensor("sg_ready_block_mode_active_sensor", False)
    assert openamber.set_binary_sensor("error_active", False)
    assert openamber.set_binary_sensor("external_heat_demand_wired", False)
    assert openamber.set_switch("heat_demand_switch", False)
    assert openamber.set_binary_sensor("external_cool_demand_wired", False)
    assert openamber.set_switch("cool_demand_switch", False)
    openamber.step(ms=150)

    assert openamber.get_entity("cool_demand_active_sensor") is False, (
        "cool_demand_active_sensor should initially be False"
    )
    assert openamber.is_hidden("nav_status_cool_icon"), (
        "Navbar snowflake icon should initially be hidden"
    )

    # External cooling demand closes contact
    assert openamber.set_binary_sensor("external_cool_demand_wired", True)
    openamber.step(ms=150)

    # Cooling demand activates
    assert openamber.get_entity("cool_demand_active_sensor") is True, (
        "cool_demand_active_sensor should be True when wired cooling contact closes"
    )
    assert openamber.is_visible("nav_status_cool_icon"), (
        "Navbar snowflake icon should be visible when cooling demand is active"
    )

    # Cooling demand deactivates
    assert openamber.set_binary_sensor("external_cool_demand_wired", False)
    openamber.step(ms=150)

    assert openamber.get_entity("cool_demand_active_sensor") is False, (
        "cool_demand_active_sensor should deactivate when wired cooling contact opens"
    )
    assert openamber.is_hidden("nav_status_cool_icon"), (
        "Navbar snowflake icon should be hidden after cooling demand ends"
    )


def test_cooling_suppressed_when_heating_active(clean_system):
    """
    Priority Flow:
    1. Heating demand is actively running.
    2. Cooling thermostat simultaneously calls for cooling.
    3. Verify heating priority: cooling demand remains False.
    """
    openamber = clean_system

    assert openamber.set_select("thermostat_mode_select", "Extern")
    assert openamber.set_binary_sensor("external_heat_demand_wired", True)
    openamber.step(ms=150)

    assert openamber.get_entity("heat_demand_active_sensor") is True, (
        "Heat demand should be active when wired heat contact is closed"
    )

    # Cooling call while heating is active
    assert openamber.set_binary_sensor("external_cool_demand_wired", True)
    openamber.step(ms=150)

    # Cooling must be suppressed by heating priority
    assert openamber.get_entity("cool_demand_active_sensor") is False, (
        "Cooling demand must be suppressed while heating demand is active (heat priority)"
    )
    assert openamber.is_hidden("nav_status_cool_icon"), (
        "Navbar snowflake icon must be hidden while cooling is suppressed by heating"
    )

    # Heat demand ends -> cooling demand should now resume
    assert openamber.set_binary_sensor("external_heat_demand_wired", False)
    openamber.step(ms=150)

    assert openamber.get_entity("cool_demand_active_sensor") is True, (
        "Cooling demand should activate once higher-priority heating demand clears"
    )
    assert openamber.is_visible("nav_status_cool_icon"), (
        "Navbar snowflake icon should appear once cooling demand takes over"
    )

    # Cleanup
    assert openamber.set_binary_sensor("external_cool_demand_wired", False)
    openamber.step(ms=150)


def test_external_cooling_ignored_in_internal_thermostat_mode(clean_system):
    """
    Edge Case:
    When thermostat_mode_select is 'Intern', external wired cooling contact
    must NOT activate cooling demand.
    """
    openamber = clean_system

    assert openamber.set_select("thermostat_mode_select", "Intern")
    assert openamber.set_switch("cool_demand_switch", False)
    assert openamber.set_binary_sensor("external_cool_demand_wired", True)
    openamber.step(ms=150)

    assert openamber.get_entity("cool_demand_active_sensor") is False, (
        "Wired cooling contact must be ignored when thermostat mode is 'Intern'"
    )
    assert openamber.is_hidden("nav_status_cool_icon"), (
        "Snowflake icon must remain hidden when external contact is ignored in 'Intern' mode"
    )

    # Cleanup
    assert openamber.set_binary_sensor("external_cool_demand_wired", False)
    openamber.step(ms=150)
