"""Automated tests for Modbus failsafe and reboot state synchronization.

Acceptance criteria:
After a failsafe or reboot of the inside unit, register 1099 (pump_p0_relay_switch)
resets to 0 (OFF) on the inside unit, heat pump regulation stops, and an error is active
(error_inside_unit_failsafe and error_active).
"""

import pytest


def test_failsafe_or_reboot_detection_when_register_1099_turns_off(clean_system):
    """
    Verify that when the inside unit enters failsafe or reboots (watchdog, power dip):
    - Register 1099 (pump_p0_relay_switch) drops to 0 (False) on readback.
    - OpenAmber expects 1099 to be ON (True) while active.
    - error_inside_unit_failsafe becomes True.
    - error_active becomes True.
    - Regulation stops (heat demand suppressed, standby mode).
    """
    openamber = clean_system

    # Set up normal heat demand operating conditions
    assert openamber.set_select("thermostat_mode_select", "Extern")
    assert openamber.set_sensor("temperature_outside_ta", 5.0)
    assert openamber.set_sensor("current_water_temperature_tc_sensor", 25.0)
    assert openamber.set_sensor("heat_cool_temperature_tc", 25.0)
    assert openamber.set_sensor("outlet_temperature_tuo", 25.0)
    assert openamber.set_sensor("inlet_temperature_tui", 23.0)
    assert openamber.set_switch("heat_demand_switch", True)
    openamber.step(ms=150)

    # Regulation should be active
    assert openamber.get_entity("heat_demand_active_sensor") is True, (
        "Heat demand should be active before failsafe/reboot"
    )
    assert openamber.get_entity("error_inside_unit_failsafe") is False, (
        "error_inside_unit_failsafe should initially be False"
    )
    assert openamber.get_entity("error_active") is False, (
        "error_active should initially be False"
    )

    # Failsafe / reboot event: inside unit resets register 1099 (pump_p0_relay_switch) to 0 (False)
    assert openamber.set_switch("pump_p0_relay_switch", False)
    openamber.advance_time(seconds=6, step_s=1)

    # Regulation must stop and error must be active
    assert openamber.get_entity("error_inside_unit_failsafe") is True, (
        "error_inside_unit_failsafe must be True after register 1099 drops to 0"
    )
    assert openamber.get_entity("error_active") is True, (
        "error_active must be True when failsafe error is active"
    )
    assert openamber.get_entity("heat_demand_active_sensor") is False, (
        "Heat demand must be stopped when failsafe error is active"
    )


def test_failsafe_error_reset_via_switch_reinitializes_register_1099(clean_system):
    """
    Verify that the failsafe error can be cleared via reset_start_timeout_errors_switch,
    which re-initializes the controller and turns register 1099 (pump_p0_relay_switch) back ON.
    """
    openamber = clean_system

    # Trigger failsafe / reboot (register 1099 drops to 0)
    assert openamber.set_switch("pump_p0_relay_switch", False)
    openamber.advance_time(seconds=6, step_s=1)
    assert openamber.get_entity("error_inside_unit_failsafe") is True
    assert openamber.get_entity("error_active") is True

    # Trigger reset switch -> controller re-runs INITIALIZING and turns pump_p0_relay_switch back ON
    assert openamber.set_switch("reset_start_timeout_errors_switch", True)
    openamber.advance_time(seconds=12, step_s=1)

    assert openamber.get_entity("error_inside_unit_failsafe") is False, (
        "error_inside_unit_failsafe should be cleared after reset"
    )
    assert openamber.get_entity("error_active") is False, (
        "error_active should be cleared after reset"
    )
    assert openamber.get_entity("pump_p0_relay_switch") is True, (
        "pump_p0_relay_switch (register 1099) must be turned back ON during re-initialization"
    )


def test_service_ui_displays_failsafe_error(clean_system):
    """
    Verify navigating to Service -> Storingen tab when failsafe error is active:
    - service_errors_panel is shown.
    - service_errors_reset_timeout_btn is visible.
    - Clicking reset button resets timeout error, re-initializes 1099, and hides the button.
    """
    openamber = clean_system

    # Trigger failsafe error
    assert openamber.set_switch("pump_p0_relay_switch", False)
    openamber.advance_time(seconds=6, step_s=1)
    assert openamber.get_entity("error_inside_unit_failsafe") is True

    # Navigate to Service -> Storingen
    assert openamber.click("nav_service")
    openamber.step(ms=100)
    assert openamber.click("service_tab_errors")
    openamber.step(ms=100)

    assert openamber.is_visible("service_errors_panel")
    assert openamber.is_visible("service_errors_reset_timeout_btn"), (
        "Reset button should be visible when failsafe timeout error is active"
    )

    # Click reset button
    assert openamber.click("service_errors_reset_timeout_btn")
    openamber.advance_time(seconds=12, step_s=1)

    assert openamber.get_entity("error_inside_unit_failsafe") is False
    assert openamber.get_entity("error_active") is False
    assert openamber.get_entity("pump_p0_relay_switch") is True

    # Re-click tab to refresh UI
    assert openamber.click("service_tab_errors")
    openamber.step(ms=100)
    assert openamber.is_hidden("service_errors_reset_timeout_btn")
