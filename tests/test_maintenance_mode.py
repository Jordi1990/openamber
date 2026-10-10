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
