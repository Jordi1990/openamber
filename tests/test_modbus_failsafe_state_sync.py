"""Automated tests for Modbus failsafe and reboot state synchronization.

Acceptance criteria:
After a failsafe or reboot of the inside unit, heat pump regulation stops
and an error is active (error_inside_unit_failsafe and error_active).
"""

import pytest


def test_failsafe_detection_when_1111_turns_on(clean_system):
    """
    Verify that when the inside unit enters failsafe:
    - Register 1111 (relais PD6) turns ON (1).
    - OpenAmber detects the mismatch against expected (0).
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
        "Heat demand should be active before failsafe"
    )
    assert openamber.get_entity("error_inside_unit_failsafe") is False, (
        "error_inside_unit_failsafe should initially be False"
    )
    assert openamber.get_entity("error_active") is False, (
        "error_active should initially be False"
    )

    # Failsafe event: inside unit turns ON register 1111 (PD6)
    assert openamber.set_sensor("inside_unit_register_1111", 1.0)
    openamber.advance_time(seconds=6, step_s=1)

    # Regulation must stop and error must be active
    assert openamber.get_entity("error_inside_unit_failsafe") is True, (
        "error_inside_unit_failsafe must be True after failsafe register 1111 turns ON"
    )
    assert openamber.get_entity("error_active") is True, (
        "error_active must be True when failsafe error is active"
    )
    assert openamber.get_entity("heat_demand_active_sensor") is False, (
        "Heat demand must be stopped when failsafe error is active"
    )


def test_reboot_detection_when_outputs_reset_to_zero(clean_system):
    """
    Verify that when the inside unit reboots (watchdog, power dip):
    - Registers 1099-1113 reset to 0 (including P0 relay 1099 and HO relay 1104).
    - OpenAmber expects 1099 and 1104 to be ON (1).
    - Mismatch is detected.
    - error_inside_unit_failsafe becomes True.
    - error_active becomes True.
    - Regulation stops.
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

    assert openamber.get_entity("heat_demand_active_sensor") is True

    # Reboot event: inside unit resets registers to 0 (e.g. relay 1104 and pump 1099 drop to 0)
    assert openamber.set_sensor("inside_unit_register_1104", 0.0)
    assert openamber.set_sensor("inside_unit_register_1099", 0.0)
    openamber.advance_time(seconds=6, step_s=1)

    assert openamber.get_entity("error_inside_unit_failsafe") is True, (
        "error_inside_unit_failsafe must be True after inside unit registers reset to 0"
    )
    assert openamber.get_entity("error_active") is True, (
        "error_active must be True after inside unit reboot"
    )
    assert openamber.get_entity("heat_demand_active_sensor") is False, (
        "Heat demand must stop after inside unit reboot"
    )


def test_failsafe_error_reset_via_switch(clean_system):
    """
    Verify that the failsafe error can be cleared via reset_start_timeout_errors_switch
    once the register state is normalized.
    """
    openamber = clean_system

    # Trigger failsafe
    assert openamber.set_sensor("inside_unit_register_1111", 1.0)
    openamber.advance_time(seconds=6, step_s=1)
    assert openamber.get_entity("error_inside_unit_failsafe") is True
    assert openamber.get_entity("error_active") is True

    # Clear injected register condition and trigger reset switch
    assert openamber.set_sensor("inside_unit_register_1111", 0.0)
    assert openamber.set_switch("reset_start_timeout_errors_switch", True)
    openamber.advance_time(seconds=6, step_s=1)

    assert openamber.get_entity("error_inside_unit_failsafe") is False, (
        "error_inside_unit_failsafe should be cleared after reset"
    )
    assert openamber.get_entity("error_active") is False, (
        "error_active should be cleared after reset"
    )


def test_service_ui_displays_failsafe_error(clean_system):
    """
    Verify navigating to Service -> Storingen tab when failsafe error is active:
    - service_errors_panel is shown.
    - service_errors_reset_timeout_btn is visible.
    - Clicking reset button resets timeout error and hides the button.
    """
    openamber = clean_system

    # Trigger failsafe error
    assert openamber.set_sensor("inside_unit_register_1111", 1.0)
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

    # Normalize register and click reset button
    assert openamber.set_sensor("inside_unit_register_1111", 0.0)
    assert openamber.click("service_errors_reset_timeout_btn")
    openamber.advance_time(seconds=6, step_s=1)

    assert openamber.get_entity("error_inside_unit_failsafe") is False
    assert openamber.get_entity("error_active") is False

    # Re-click tab to refresh UI
    assert openamber.click("service_tab_errors")
    openamber.step(ms=100)
    assert openamber.is_hidden("service_errors_reset_timeout_btn")
