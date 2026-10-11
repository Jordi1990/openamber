"""Automated tests for error and warning display, aggregation, and reset logic.

Covers:
1. Error aggregation in error_active binary sensor from error flags and timeouts.
2. Reset of timeout errors via reset_start_timeout_errors_switch.
3. UI navigation to Service -> Storingen (Errors) tab and reset timeout button behavior.
4. UI navigation to Service -> Waarschuwingen (Warnings) tab.
"""

import pytest


def test_error_active_aggregation_and_reset(clean_system):
    """
    Verify error_active aggregation:
    - Initially False.
    - Setting error_pump_start_timeout makes error_active True.
    - Turning on reset_start_timeout_errors_switch resets timeout errors and error_active back to False.
    - Setting raw register error_register_2121_raw to 1 (E01) triggers error_flag_e01 and error_active.
    - Clearing the register clears the error flag and error_active.
    """
    openamber = clean_system

    # Initial state
    assert openamber.get_entity("error_active") is False, "error_active should initially be False"

    # 1. Test timeout error flag
    assert openamber.set_binary_sensor("error_pump_start_timeout", True) is True
    openamber.step(ms=50)
    assert openamber.get_entity("error_pump_start_timeout") is True, "error_pump_start_timeout should be True"
    assert openamber.get_entity("error_active") is True, "error_active should be True when timeout error is active"

    # Reset via switch
    assert openamber.set_switch("reset_start_timeout_errors_switch", True) is True
    openamber.step(ms=50)
    assert openamber.get_entity("error_pump_start_timeout") is False, "error_pump_start_timeout should be reset to False"
    assert openamber.get_entity("error_active") is False, "error_active should be False after reset"

    # 2. Test hardware/communication error flag E01 via raw register
    # E01 bit is bit 0 of register 2121 (raw & 0x0001)
    assert openamber.set_sensor("error_register_2121_raw", 1.0) is True
    openamber.step(ms=50)
    assert openamber.get_entity("error_flag_e01") is True, "error_flag_e01 should be True when bit 0 of 2121 is set"
    assert openamber.get_entity("error_active") is True, "error_active should be True when error_flag_e01 is active"

    # Clear raw register
    assert openamber.set_sensor("error_register_2121_raw", 0.0) is True
    openamber.step(ms=50)
    assert openamber.get_entity("error_flag_e01") is False, "error_flag_e01 should be False when register is 0"
    assert openamber.get_entity("error_active") is False, "error_active should be False after clearing register"


def test_service_error_ui_navigation_and_timeout_reset(clean_system):
    """
    Verify navigating to Service -> Storingen tab:
    - service_errors_panel is shown.
    - service_errors_reset_timeout_btn is hidden when no timeout error is active.
    - When a timeout error occurs, reset button becomes visible.
    - Clicking reset button resets timeout error and hides the button.
    """
    openamber = clean_system

    # Navigate to Service page
    assert openamber.click("nav_service"), "Clicking nav_service must succeed"
    openamber.step(ms=100)
    assert openamber.is_visible("page_service"), "page_service should be visible"

    # Open Errors tab (tab index 7)
    assert openamber.click("service_tab_errors"), "Clicking service_tab_errors must succeed"
    openamber.step(ms=100)

    assert openamber.is_visible("service_errors_panel"), "service_errors_panel should be visible"
    assert openamber.is_hidden("service_errors_reset_timeout_btn"), "Reset button should be hidden initially"

    # Inject timeout error
    assert openamber.set_binary_sensor("error_compressor_start_timeout", True) is True
    openamber.step(ms=50)
    assert openamber.get_entity("error_compressor_start_timeout") is True

    # Re-click tab to trigger update_service_errors_ui
    assert openamber.click("service_tab_errors"), "Re-clicking service_tab_errors must succeed"
    openamber.step(ms=100)

    # Button should now be visible
    assert openamber.is_visible("service_errors_reset_timeout_btn"), "Reset button should be visible when timeout error is active"

    # Click reset button
    assert openamber.click("service_errors_reset_timeout_btn"), "Clicking reset button must succeed"
    openamber.step(ms=100)

    # Confirm error cleared
    assert openamber.get_entity("error_compressor_start_timeout") is False, "error_compressor_start_timeout should be cleared"

    # Update UI to verify button hidden
    assert openamber.click("service_tab_errors"), "Re-clicking service_tab_errors must succeed"
    openamber.step(ms=100)
    assert openamber.is_hidden("service_errors_reset_timeout_btn"), "Reset button should be hidden after error cleared"


def test_service_warning_ui_navigation(clean_system):
    """
    Verify navigating to Service -> Waarschuwingen tab:
    - service_warnings_panel is shown.
    - service_errors_panel is hidden.
    """
    openamber = clean_system

    # Navigate to Service page
    assert openamber.click("nav_service"), "Clicking nav_service must succeed"
    openamber.step(ms=100)

    # Open Warnings tab (tab index 8)
    assert openamber.click("service_tab_warnings"), "Clicking service_tab_warnings must succeed"
    openamber.step(ms=100)

    assert openamber.is_visible("service_warnings_panel"), "service_warnings_panel should be visible"
    assert openamber.is_hidden("service_errors_panel"), "service_errors_panel should be hidden"
