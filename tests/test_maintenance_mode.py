"""Automated tests for maintenance mode transitions and manual control.

Covers:
1. Enabling service_mode_enabled transitions the system into maintenance mode:
   - state_machine_state_main becomes "Maintenance".
   - working_mode_switch becomes "Onderhoud".
2. Exiting maintenance mode:
   - service_mode_enabled set to False.
   - System transitions back through initialization to normal operation.
3. UI switch set_service_mode_switch_ui toggles service_mode_enabled.
"""

import pytest


def test_maintenance_mode_transition(clean_system):
    """
    Verify entering and exiting maintenance mode:
    - Initially state_machine_state_main is Heat/Cool.
    - Enabling service_mode_enabled transitions state_machine_state_main to "Maintenance"
      and working_mode_switch to "Onderhoud".
    - Disabling service_mode_enabled transitions the system back to normal operation.
    """
    openamber = clean_system

    # Advance time to ensure initialization completes and system is in normal operation
    openamber.advance_time(seconds=20, step_s=5)
    openamber.step(ms=50)

    assert openamber.get_entity("state_machine_state_main") == "Heat/Cool", (
        f"Initial main state should be Heat/Cool, got {openamber.get_entity('state_machine_state_main')}"
    )

    # Enable maintenance mode
    assert openamber.set_switch("service_mode_enabled", True) is True
    openamber.step(ms=50)
    assert openamber.get_entity("service_mode_enabled") is True, "service_mode_enabled should be True"

    # Advance time for openamber_component (update interval 5s)
    openamber.advance_time(seconds=10, step_s=1)
    openamber.step(ms=50)

    assert openamber.get_entity("state_machine_state_main") == "Maintenance", (
        f"Main state should be Maintenance, got {openamber.get_entity('state_machine_state_main')}"
    )
    assert openamber.get_entity("working_mode_switch") == "Onderhoud", (
        f"working_mode_switch should be Onderhoud, got {openamber.get_entity('working_mode_switch')}"
    )

    # Disable maintenance mode
    assert openamber.set_switch("service_mode_enabled", False) is True
    openamber.step(ms=50)
    assert openamber.get_entity("service_mode_enabled") is False, "service_mode_enabled should be False"

    # Advance time for transition back through initialization delay (10s) and valve switch (60s)
    openamber.advance_time(seconds=25, step_s=5)
    openamber.advance_time(seconds=75, step_s=5)
    openamber.step(ms=50)

    assert openamber.get_entity("state_machine_state_main") == "Heat/Cool", (
        f"Main state should return to Heat/Cool, got {openamber.get_entity('state_machine_state_main')}"
    )


def test_maintenance_mode_ui_switch_toggle(clean_system):
    """
    Verify toggling maintenance mode via the UI switch set_service_mode_switch_ui.
    """
    openamber = clean_system

    # Advance to steady normal operation
    openamber.advance_time(seconds=20, step_s=5)
    openamber.step(ms=50)

    # Navigate to Service -> Maintenance tab (tab index 0)
    assert openamber.click("nav_service"), "Clicking nav_service must succeed"
    openamber.step(ms=100)
    assert openamber.click("service_tab_maintenance"), "Clicking service_tab_maintenance must succeed"
    openamber.step(ms=100)

    assert openamber.is_visible("set_service_mode_switch_ui"), "set_service_mode_switch_ui should be visible"
    assert openamber.get_entity("service_mode_enabled") is False, "service_mode_enabled should initially be False"

    # Click switch to turn ON maintenance mode
    assert openamber.click("set_service_mode_switch_ui"), "Clicking set_service_mode_switch_ui must succeed"
    openamber.step(ms=100)

    assert openamber.get_entity("service_mode_enabled") is True, (
        "service_mode_enabled should be True after clicking switch"
    )

    # Advance time to allow state machine transition
    openamber.advance_time(seconds=10, step_s=1)
    openamber.step(ms=50)
    assert openamber.get_entity("state_machine_state_main") == "Maintenance", (
        f"Main state should be Maintenance, got {openamber.get_entity('state_machine_state_main')}"
    )

    # Click switch again to turn OFF maintenance mode
    assert openamber.click("set_service_mode_switch_ui"), "Clicking set_service_mode_switch_ui again must succeed"
    openamber.step(ms=100)

    assert openamber.get_entity("service_mode_enabled") is False, (
        "service_mode_enabled should be False after toggling off"
    )


def test_maintenance_proxy_interlocks_prevent_hardware_actuation_when_disabled(clean_system):
    """
    Verify Maintenance Mode Hardware Safety Interlocks:
    1. In normal operation (service_mode_enabled = False), the maintenance proxy switches
       (pump P0, 3-way valve, bottom plate heater) must NOT actuate underlying hardware relays.
    2. Once maintenance mode is enabled (service_mode_enabled = True and state is Maintenance),
       toggling the proxies directly controls the hardware relays.
    3. Turning proxies OFF returns hardware relays to OFF.
    4. Exiting maintenance mode smoothly transitions system back to normal operation.
    """
    openamber = clean_system

    # Settle into steady normal operation
    openamber.advance_time(seconds=20, step_s=5)
    openamber.step(ms=50)

    assert openamber.get_entity("service_mode_enabled") is False
    assert openamber.get_entity("state_machine_state_main") == "Heat/Cool"

    # Initial hardware states should be OFF for auxiliary equipment
    assert openamber.get_entity("three_way_valve_dhw_switch") is False
    assert openamber.get_entity("bottom_plate_heater_manual_switch") is False
    assert openamber.get_entity("compressor_crankcase_heater_manual_switch") is False

    # 1. Attempt manual proxy actuation while maintenance mode is DISABLED
    openamber.set_switch("maintenance_three_way_valve_dhw_proxy", True)
    openamber.set_switch("maintenance_bottom_plate_heater_manual_proxy", True)
    openamber.set_switch("maintenance_compressor_crankcase_heater_manual_proxy", True)
    openamber.step(ms=100)

    # Interlocks MUST prevent relays from turning ON
    assert openamber.get_entity("three_way_valve_dhw_switch") is False, (
        "3-way valve relay must NOT turn ON when maintenance mode is disabled"
    )
    assert openamber.get_entity("bottom_plate_heater_manual_switch") is False, (
        "Bottom plate heater relay must NOT turn ON when maintenance mode is disabled"
    )
    assert openamber.get_entity("compressor_crankcase_heater_manual_switch") is False, (
        "Crankcase heater relay must NOT turn ON when maintenance mode is disabled"
    )

    # 2. Enter maintenance mode
    assert openamber.set_switch("service_mode_enabled", True) is True
    openamber.advance_time(seconds=10, step_s=1)
    openamber.step(ms=50)
    assert openamber.get_entity("state_machine_state_main") == "Maintenance"

    # In maintenance mode, pump P0 is explicitly stopped
    assert openamber.get_entity("pump_p0_relay_switch") is False

    # Now proxies MUST be permitted to actuate hardware
    openamber.set_switch("maintenance_pump_p0_relay_proxy", True)
    openamber.step(ms=50)
    assert openamber.get_entity("pump_p0_relay_switch") is True, (
        "Pump P0 relay must actuate when maintenance mode is active"
    )

    openamber.set_switch("maintenance_three_way_valve_dhw_proxy", True)
    openamber.step(ms=50)
    assert openamber.get_entity("three_way_valve_dhw_switch") is True, (
        "3-way valve relay must actuate when maintenance mode is active"
    )

    openamber.set_switch("maintenance_bottom_plate_heater_manual_proxy", True)
    openamber.step(ms=50)
    assert openamber.get_entity("bottom_plate_heater_manual_switch") is True, (
        "Bottom plate heater relay must actuate when maintenance mode is active"
    )

    # 3. Turn proxies OFF while still in maintenance mode
    openamber.set_switch("maintenance_pump_p0_relay_proxy", False)
    openamber.set_switch("maintenance_three_way_valve_dhw_proxy", False)
    openamber.set_switch("maintenance_bottom_plate_heater_manual_proxy", False)
    openamber.step(ms=50)

    assert openamber.get_entity("pump_p0_relay_switch") is False
    assert openamber.get_entity("three_way_valve_dhw_switch") is False
    assert openamber.get_entity("bottom_plate_heater_manual_switch") is False

    # 4. Exit maintenance mode
    openamber.set_switch("service_mode_enabled", False)
    openamber.advance_time(seconds=25, step_s=5)
    openamber.advance_time(seconds=75, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity("state_machine_state_main") == "Heat/Cool"
