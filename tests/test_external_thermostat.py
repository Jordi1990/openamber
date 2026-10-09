"""Tests for external thermostat happy flow, wired demand, and mode switching."""

import pytest


def test_external_thermostat_wired_demand_happy_flow(openamber):
    """
    Happy Flow:
    1. Thermostat mode is configured as 'Extern'.
    2. External wired contact closes (external_heat_demand_wired = True).
    3. Verify heat demand activates (heat_demand_active_sensor = True).
    4. Verify UI reflects heat demand: navbar heat icon (nav_status_heat_icon) is VISIBLE.
    5. External contact opens (external_heat_demand_wired = False).
    6. Verify heat demand deactivates and navbar heat icon is HIDDEN.
    """
    # 1. Set mode to Extern
    openamber.set_select("thermostat_mode_select", "Extern")
    openamber.set_binary_sensor("sg_ready_block_mode_active_sensor", False)
    openamber.set_binary_sensor("error_active", False)
    openamber.set_binary_sensor("external_heat_demand_wired", False)
    openamber.set_switch("heat_demand_switch", False)
    openamber.step(ms=150)

    assert openamber.get_entity("heat_demand_active_sensor") is False
    assert openamber.is_hidden("nav_status_heat_icon")

    # 2. External wired room thermostat closes contact (calling for heat)
    openamber.set_binary_sensor("external_heat_demand_wired", True)
    openamber.step(ms=150)

    # 3. System detects heat demand
    assert openamber.get_entity("heat_demand_active_sensor") is True, (
        "heat_demand_active_sensor should be True when external wired contact is closed"
    )

    # 4. LVGL UI updates navbar status icon
    assert openamber.is_visible("nav_status_heat_icon"), (
        "Navbar flame icon should be visible when heat demand is active"
    )

    # 5. External wired room thermostat reaches setpoint and opens contact
    openamber.set_binary_sensor("external_heat_demand_wired", False)
    openamber.step(ms=150)

    # 6. Demand ends and icon hides
    assert openamber.get_entity("heat_demand_active_sensor") is False
    assert openamber.is_hidden("nav_status_heat_icon")


def test_external_thermostat_manual_switch_flow(openamber):
    """
    Happy Flow:
    1. Thermostat mode is 'Extern'.
    2. User activates manual heat demand switch (heat_demand_switch).
    3. Verify heat_demand_active_sensor is True and nav_status_heat_icon is visible.
    4. User deactivates heat demand switch.
    5. Verify heat demand turns off.
    """
    openamber.set_select("thermostat_mode_select", "Extern")
    openamber.set_binary_sensor("external_heat_demand_wired", False)
    openamber.set_switch("heat_demand_switch", False)
    openamber.step(ms=150)

    # Activate manual heat demand
    openamber.set_switch("heat_demand_switch", True)
    openamber.step(ms=150)

    assert openamber.get_entity("heat_demand_active_sensor") is True
    assert openamber.is_visible("nav_status_heat_icon")

    # Deactivate manual heat demand
    openamber.set_switch("heat_demand_switch", False)
    openamber.step(ms=150)

    assert openamber.get_entity("heat_demand_active_sensor") is False
    assert openamber.is_hidden("nav_status_heat_icon")


def test_thermostat_mode_switching_intern_vs_extern(openamber):
    """
    Workflow Test:
    1. In 'Intern' mode, wired external contact does NOT trigger heat_demand_active_sensor.
    2. Switching mode to 'Extern' causes the existing wired contact to trigger heat demand immediately.
    """
    # Set to Intern mode
    openamber.set_select("thermostat_mode_select", "Intern")
    openamber.set_switch("heat_demand_switch", False)
    openamber.set_binary_sensor("external_heat_demand_wired", True)
    openamber.step(ms=150)

    # In Intern mode, external contact is ignored (controlled by climate entity action)
    assert openamber.get_entity("heat_demand_active_sensor") is False

    # Switch to Extern mode
    openamber.set_select("thermostat_mode_select", "Extern")
    openamber.step(ms=150)

    # In Extern mode, wired contact now activates heat demand
    assert openamber.get_entity("heat_demand_active_sensor") is True
    assert openamber.is_visible("nav_status_heat_icon")

    # Cleanup
    openamber.set_binary_sensor("external_heat_demand_wired", False)
    openamber.step(ms=150)
