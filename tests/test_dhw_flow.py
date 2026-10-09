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
    openamber.click("nav_home")
    openamber.step(ms=100)
    assert openamber.is_visible("page_home")

    # Initial state
    openamber.set_number("dhw_setpoint_temperature", 50.0)
    openamber.set_sensor("dhw_temperature_tw_sensor", 55.0)
    openamber.set_binary_sensor("dhw_pump_active_sensor", False)
    openamber.step(ms=150)

    assert openamber.is_hidden("nav_status_dhw_icon")
    assert openamber.is_hidden("dhw_circulation_spinner")
    assert openamber.get_label("dhw_pump_state_label") == "UIT"

    # Step 1: DHW Demand activates
    openamber.set_sensor("dhw_temperature_tw_sensor", 40.0)
    openamber.step(ms=150)
    assert openamber.get_entity("dhw_demand_active_sensor") is True
    assert openamber.is_visible("nav_status_dhw_icon"), (
        "Navbar tapwater icon should be visible when DHW demand is active"
    )

    # Step 2: DHW circulation pump turns on
    openamber.set_switch("dhw_pump_relay_switch", True)
    openamber.step(ms=150)

    assert openamber.is_visible("dhw_circulation_spinner"), (
        "DHW circulation spinner should animate when pump is running"
    )
    assert openamber.get_label("dhw_pump_state_label") == "AAN", (
        "DHW pump status label should display 'AAN'"
    )

    # Step 3: DHW cycle finishes
    openamber.set_switch("dhw_pump_relay_switch", False)
    openamber.set_sensor("dhw_temperature_tw_sensor", 52.0)
    openamber.step(ms=150)

    assert openamber.is_hidden("dhw_circulation_spinner")
    assert openamber.get_label("dhw_pump_state_label") == "UIT"
    assert openamber.is_hidden("nav_status_dhw_icon")
