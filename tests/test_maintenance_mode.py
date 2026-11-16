"""Automated tests for maintenance mode transitions and manual control.

Covers:
1. Maintenance mode can only be turned on while the heat pump is stopped (controlled stop, Standby):
   - state_machine_state_main becomes "Maintenance".
   - working_mode_switch becomes "Onderhoud".
2. Exiting maintenance mode:
   - service_mode_enabled set to False.
   - The controlled stop is still active: the system re-initializes and returns to Standby.
   - Resuming is blocked while maintenance mode is on.
3. UI switch set_service_mode_switch_ui toggles service_mode_enabled, and is disabled while running.
4. Header badge shows that the heat pump is stopped.
"""

from helpers import advance_until_state, enter_maintenance, leave_maintenance, start_space_heating, stop_heat_pump

MAIN = "state_machine_state_main"


def test_maintenance_mode_transition(clean_system):
    """
    Verify entering and exiting maintenance mode:
    - Initially state_machine_state_main is Heat/Cool.
    - After a controlled stop, enabling service_mode_enabled transitions state_machine_state_main to "Maintenance"
      and working_mode_switch to "Onderhoud".
    - Disabling service_mode_enabled returns to Standby; resuming returns to normal operation.
    """
    openamber = clean_system

    # Advance time to ensure initialization completes and system is in normal operation
    openamber.advance_time(seconds=20, step_s=5)
    openamber.step(ms=50)

    assert openamber.get_entity(MAIN) == "Heat/Cool", (
        f"Initial main state should be Heat/Cool, got {openamber.get_entity(MAIN)}"
    )

    enter_maintenance(openamber)
    assert openamber.get_entity("working_mode_switch") == "Onderhoud", (
        f"working_mode_switch should be Onderhoud, got {openamber.get_entity('working_mode_switch')}"
    )

    leave_maintenance(openamber)


def test_maintenance_mode_rejected_while_running(clean_system):
    """Maintenance mode cannot be turned on while the heat pump is in operation."""
    openamber = clean_system
    start_space_heating(openamber)
    assert int(openamber.get_entity("compressor_control_select")) > 0

    openamber.set_switch("service_mode_enabled", True)
    openamber.step(ms=50)
    assert openamber.get_entity("service_mode_enabled") is False, "Maintenance mode must be refused while running"
    openamber.advance_time(seconds=30, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity(MAIN) == "Heat/Cool"
    assert int(openamber.get_entity("compressor_control_select")) > 0, "A refused request must not stop the compressor"

    openamber.set_switch("heat_demand_switch", False)
    openamber.advance_time(seconds=900, step_s=20)
    openamber.step(ms=50)


def test_resume_blocked_while_maintenance_mode_on(clean_system):
    """Resuming after a controlled stop is only possible once maintenance mode is off; no auto-resume either."""
    openamber = clean_system
    openamber.advance_time(seconds=20, step_s=5)
    enter_maintenance(openamber)

    openamber.press_button("controlled_stop_resume_button")
    openamber.advance_time(seconds=30, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity(MAIN) == "Maintenance", "Resume must be refused while maintenance mode is on"

    # The 30 minute auto-resume of the controlled stop does not end maintenance mode.
    openamber.advance_time(seconds=2000, step_s=50)
    openamber.step(ms=50)
    assert openamber.get_entity(MAIN) == "Maintenance"
    assert openamber.get_entity("service_mode_enabled") is True

    leave_maintenance(openamber)


def test_maintenance_mode_ui_switch_toggle(clean_system):
    """
    Verify toggling maintenance mode via the UI switch set_service_mode_switch_ui: disabled while running,
    enabled once stopped.
    """
    openamber = clean_system

    # Advance to steady normal operation
    openamber.advance_time(seconds=20, step_s=5)
    openamber.step(ms=50)

    # Navigate to Service -> Maintenance tab (tab index 0)
    assert openamber.click("nav_service"), "Clicking nav_service must succeed"
    openamber.step(ms=100)

    # When maintenance mode is OFF, dependent service categories must be disabled/greyed out
    assert openamber.get_widget("service_tab_compressor").get("disabled") is True, "Compressor card should be disabled when maintenance mode is OFF"
    assert openamber.get_widget("service_tab_deaeration").get("disabled") is True, "Deaeration card should be disabled when maintenance mode is OFF"
    assert openamber.get_widget("service_tab_pumps").get("disabled") is True, "Pumps card should be disabled when maintenance mode is OFF"
    assert openamber.get_widget("service_tab_valve").get("disabled") is True, "Valve card should be disabled when maintenance mode is OFF"
    assert openamber.get_widget("service_tab_backup").get("disabled") is True, "Backup card should be disabled when maintenance mode is OFF"

    assert openamber.click("service_tab_maintenance"), "Clicking service_tab_maintenance must succeed"
    openamber.step(ms=100)

    assert openamber.is_visible("set_service_mode_switch_ui"), "set_service_mode_switch_ui should be visible"
    assert openamber.get_entity("service_mode_enabled") is False, "service_mode_enabled should initially be False"
    assert openamber.is_disabled("set_service_mode_switch_ui"), "The switch is disabled while the heat pump runs"
    assert openamber.get_label("service_maintenance_lock_hint") == "Stop eerst de warmtepomp via Systeem > Beheer."

    stop_heat_pump(openamber)
    openamber.step(ms=100)
    assert not openamber.is_disabled("set_service_mode_switch_ui"), "The switch is enabled once stopped"

    # Click switch to turn ON maintenance mode
    assert openamber.click("set_service_mode_switch_ui"), "Clicking set_service_mode_switch_ui must succeed"
    openamber.step(ms=100)

    assert openamber.get_entity("service_mode_enabled") is True, (
        "service_mode_enabled should be True after clicking switch"
    )
    assert advance_until_state(openamber, MAIN, "Maintenance", max_s=300, step=2), (
        f"Main state should be Maintenance, got {openamber.get_entity(MAIN)}"
    )

    # In maintenance mode, dependent cards must be enabled
    assert openamber.get_widget("service_tab_compressor").get("disabled") is False, "Compressor card should be enabled in maintenance mode"
    assert openamber.get_widget("service_tab_deaeration").get("disabled") is False, "Deaeration card should be enabled in maintenance mode"

    # Click switch again to turn OFF maintenance mode
    assert openamber.click("set_service_mode_switch_ui"), "Clicking set_service_mode_switch_ui again must succeed"
    openamber.step(ms=100)

    assert openamber.get_entity("service_mode_enabled") is False, (
        "service_mode_enabled should be False after toggling off"
    )
    assert advance_until_state(openamber, MAIN, "Standby", max_s=300, step=5)
    assert openamber.click("nav_home")
    openamber.step(ms=100)


def test_header_badge_shows_heat_pump_stopped(clean_system):
    """The header badge shows that the heat pump is stopped, also in maintenance mode."""
    openamber = clean_system
    openamber.advance_time(seconds=20, step_s=5)
    openamber.step(ms=50)
    assert openamber.is_hidden("header_maintenance_badge")

    stop_heat_pump(openamber)
    openamber.advance_time(seconds=6, step_s=2)
    openamber.step(ms=50)
    assert openamber.is_visible("header_maintenance_badge")
    assert openamber.get_label("header_maintenance_badge_label") == "Warmtepomp gestopt"

    assert openamber.set_switch("service_mode_enabled", True) is True
    assert advance_until_state(openamber, MAIN, "Maintenance", max_s=300, step=2)
    openamber.step(ms=50)
    assert openamber.is_visible("header_maintenance_badge")
    assert openamber.get_label("header_maintenance_badge_label") == "Warmtepomp gestopt"

    leave_maintenance(openamber)
    openamber.advance_time(seconds=6, step_s=2)
    openamber.step(ms=50)
    assert openamber.is_hidden("header_maintenance_badge")

    # Dependent cards must be disabled again
    assert openamber.get_widget("service_tab_compressor").get("disabled") is True, "Compressor card should be disabled after exiting maintenance mode"
    assert openamber.get_widget("service_tab_deaeration").get("disabled") is True, "Deaeration card should be disabled after exiting maintenance mode"


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

    # 2. Enter maintenance mode (stop first)
    enter_maintenance(openamber)

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

    # 4. Exit maintenance mode: back to Standby, then resume
    leave_maintenance(openamber)
