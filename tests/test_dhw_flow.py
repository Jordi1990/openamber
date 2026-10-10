"""Tests for Domestic Hot Water (DHW) demand and pump animation flows."""

import pytest


"""Tests for Domestic Hot Water (DHW) demand and pump animation flows."""

import pytest


def test_dhw_demand_and_circulation_animation(clean_system):
    """
    Workflow Test:
    1. DHW tank calls for reheating (dhw_demand_active_sensor = True).
    2. Navbar shows tapwater icon (nav_status_dhw_icon).
    3. DHW circulation pump activates (dhw_pump_active_sensor = True).
    4. Home page shows circulation spinner (dhw_circulation_spinner) and pump badge 'AAN'.
    5. DHW reheating finishes, pump stops, animations return to idle.
    """
    openamber = clean_system

    # Navigate to Home page to verify widgets
    assert openamber.click("nav_home"), "Click on nav_home must succeed"
    openamber.step(ms=100)
    assert openamber.is_visible("page_home"), "page_home should be visible after clicking nav_home"

    # Initial state
    assert openamber.set_number("dhw_setpoint_temperature", 50.0)
    assert openamber.set_sensor("dhw_temperature_tw_sensor", 55.0)
    assert openamber.set_binary_sensor("dhw_pump_active_sensor", False)
    assert openamber.set_switch("dhw_pump_relay_switch", False)
    openamber.step(ms=150)

    assert openamber.is_hidden("nav_status_dhw_icon"), "DHW nav icon should initially be hidden"
    assert openamber.is_hidden("dhw_circulation_spinner"), "Circulation spinner should initially be hidden"
    assert openamber.get_label("dhw_pump_state_label") == "UIT", "Pump state label should initially display 'UIT'"

    # Step 1: DHW Demand activates
    assert openamber.set_sensor("dhw_temperature_tw_sensor", 40.0)
    openamber.step(ms=150)
    assert openamber.get_entity("dhw_demand_active_sensor") is True, (
        "DHW demand should activate when Tw (40°C) is below setpoint (50°C) - restart delta (5°C)"
    )
    assert openamber.is_visible("nav_status_dhw_icon"), (
        "Navbar tapwater icon should be visible when DHW demand is active"
    )

    # Step 2: DHW circulation pump turns on
    assert openamber.set_switch("dhw_pump_relay_switch", True)
    openamber.step(ms=150)

    assert openamber.is_visible("dhw_circulation_spinner"), (
        "DHW circulation spinner should animate when pump is running"
    )
    assert openamber.get_label("dhw_pump_state_label") == "AAN", (
        "DHW pump status label should display 'AAN'"
    )

    # Step 3: DHW cycle finishes
    assert openamber.set_switch("dhw_pump_relay_switch", False)
    assert openamber.set_sensor("dhw_temperature_tw_sensor", 52.0)
    openamber.step(ms=150)

    assert openamber.is_hidden("dhw_circulation_spinner"), (
        "DHW circulation spinner should be hidden after pump stops"
    )
    assert openamber.get_label("dhw_pump_state_label") == "UIT", (
        "DHW pump status label should return to 'UIT' after cycle finishes"
    )
    assert openamber.is_hidden("nav_status_dhw_icon"), (
        "Navbar tapwater icon should hide once DHW setpoint is reached"
    )


def test_dhw_demand_suppressed_when_dhw_disabled(clean_system):
    """
    Edge Case:
    When dhw_enabled_switch is OFF, DHW demand must NOT activate even if tank is cold.
    """
    openamber = clean_system

    assert openamber.set_switch("dhw_enabled_switch", False)
    assert openamber.set_number("dhw_setpoint_temperature", 50.0)
    assert openamber.set_sensor("dhw_temperature_tw_sensor", 35.0)
    openamber.step(ms=150)

    assert openamber.get_entity("dhw_demand_active_sensor") is False, (
        "DHW demand must remain False when dhw_enabled_switch is False"
    )
    assert openamber.is_hidden("nav_status_dhw_icon"), (
        "Navbar tapwater icon must remain hidden when DHW is disabled"
    )

    # Re-enabling DHW activates demand
    assert openamber.set_switch("dhw_enabled_switch", True)
    openamber.step(ms=150)

    assert openamber.get_entity("dhw_demand_active_sensor") is True, (
        "DHW demand should activate immediately once dhw_enabled_switch is turned back ON"
    )
    assert openamber.is_visible("nav_status_dhw_icon"), (
        "Navbar tapwater icon should appear when DHW is re-enabled with cold tank"
    )
