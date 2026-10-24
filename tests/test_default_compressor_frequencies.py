"""Automated tests for 'Gebruik standaard compressorfrequenties' setting.

Covers:
1. Entity existence & default state:
   - default_compressor_frequencies_enabled defaults to False (overwriting active).
   - Toggling the setting updates its state.
2. Preventing compressor frequency overwrite:
   - When default_compressor_frequencies_enabled is False, initialization writes OpenAmber's custom frequencies.
   - When default_compressor_frequencies_enabled is True, initialization skips writing frequency tables,
     preserving any existing frequencies on the unit.
3. LVGL UI integration & restart dialog:
   - Widget set_default_compressor_frequencies_enabled_switch_ui in Settings -> Geavanceerd -> Compressor.
   - Clicking the UI switch to enable toggles default_compressor_frequencies_enabled ON and shows
     default_compressor_frequencies_modal instructing the user to power-cycle the complete heat pump
     including outdoor unit.
   - Clicking default_compressor_frequencies_modal_ok_button dismisses the modal.
   - Clicking the UI switch to disable turns default_compressor_frequencies_enabled OFF without showing the modal.
"""

import pytest


def test_default_compressor_frequencies_entity_default_and_toggle(clean_system):
    """Verify default_compressor_frequencies_enabled defaults to False and can be toggled."""
    openamber = clean_system

    # Should be False by default
    assert openamber.get_entity("default_compressor_frequencies_enabled") is False, (
        "default_compressor_frequencies_enabled should be False by default"
    )

    # Enable
    assert openamber.set_switch("default_compressor_frequencies_enabled", True) is True
    openamber.step(ms=50)
    assert openamber.get_entity("default_compressor_frequencies_enabled") is True, (
        "default_compressor_frequencies_enabled should be True after enabling"
    )

    # Disable
    assert openamber.set_switch("default_compressor_frequencies_enabled", False) is True
    openamber.step(ms=50)
    assert openamber.get_entity("default_compressor_frequencies_enabled") is False, (
        "default_compressor_frequencies_enabled should be False after disabling"
    )


def test_default_compressor_frequencies_prevents_overwrite(clean_system):
    """
    Verify that when default_compressor_frequencies_enabled is True,
    OpenAmber does NOT overwrite heating or cooling frequency tables during initialization.
    When False, it DOES overwrite them.
    """
    openamber = clean_system

    # 1. Start in normal operation and verify frequencies are initialized to OpenAmber defaults
    openamber.advance_time(seconds=20, step_s=5)
    openamber.step(ms=50)

    # Set custom frequencies simulating factory defaults or user-chosen frequencies
    custom_heat_1 = 99.0
    custom_heat_2 = 98.0
    custom_cool_1 = 77.0
    custom_cool_2 = 76.0

    openamber.set_number("heating_frequency_index_1", custom_heat_1)
    openamber.set_number("heating_frequency_index_2", custom_heat_2)
    openamber.set_number("cooling_frequency_index_1", custom_cool_1)
    openamber.set_number("cooling_frequency_index_2", custom_cool_2)
    openamber.step(ms=50)

    assert float(openamber.get_entity("heating_frequency_index_1")) == custom_heat_1
    assert float(openamber.get_entity("heating_frequency_index_2")) == custom_heat_2
    assert float(openamber.get_entity("cooling_frequency_index_1")) == custom_cool_1
    assert float(openamber.get_entity("cooling_frequency_index_2")) == custom_cool_2

    # 2. Enable default_compressor_frequencies_enabled
    assert openamber.set_switch("default_compressor_frequencies_enabled", True) is True
    openamber.step(ms=50)

    # Trigger re-initialization via maintenance mode cycle
    assert openamber.set_switch("service_mode_enabled", True) is True
    openamber.advance_time(seconds=10, step_s=1)
    openamber.step(ms=50)
    assert openamber.get_entity("state_machine_state_main") == "Maintenance"

    # Exit maintenance mode -> triggers WAIT_MODBUS_CONNECTION -> INITIALIZING
    assert openamber.set_switch("service_mode_enabled", False) is True
    openamber.advance_time(seconds=25, step_s=5)
    openamber.advance_time(seconds=75, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity("state_machine_state_main") == "Heat/Cool"

    # Frequencies must NOT have been overwritten!
    assert float(openamber.get_entity("heating_frequency_index_1")) == custom_heat_1, (
        f"heating_frequency_index_1 should remain {custom_heat_1} when setting is enabled"
    )
    assert float(openamber.get_entity("heating_frequency_index_2")) == custom_heat_2, (
        f"heating_frequency_index_2 should remain {custom_heat_2} when setting is enabled"
    )
    assert float(openamber.get_entity("cooling_frequency_index_1")) == custom_cool_1, (
        f"cooling_frequency_index_1 should remain {custom_cool_1} when setting is enabled"
    )
    assert float(openamber.get_entity("cooling_frequency_index_2")) == custom_cool_2, (
        f"cooling_frequency_index_2 should remain {custom_cool_2} when setting is enabled"
    )

    # 3. Now disable default_compressor_frequencies_enabled -> frequencies SHOULD be overwritten on re-init
    assert openamber.set_switch("default_compressor_frequencies_enabled", False) is True
    openamber.step(ms=50)

    # Trigger re-initialization again
    assert openamber.set_switch("service_mode_enabled", True) is True
    openamber.advance_time(seconds=10, step_s=1)
    openamber.step(ms=50)
    assert openamber.get_entity("state_machine_state_main") == "Maintenance"

    assert openamber.set_switch("service_mode_enabled", False) is True
    openamber.advance_time(seconds=25, step_s=5)
    openamber.advance_time(seconds=75, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity("state_machine_state_main") == "Heat/Cool"

    # Frequencies should now be overwritten by OpenAmber table values (heating index 1 = 28, index 2 = 36, cooling index 1 = 20, index 2 = 26)
    assert float(openamber.get_entity("heating_frequency_index_1")) == 28.0, (
        "heating_frequency_index_1 should be overwritten to 28 when setting is disabled"
    )
    assert float(openamber.get_entity("heating_frequency_index_2")) == 36.0, (
        "heating_frequency_index_2 should be overwritten to 36 when setting is disabled"
    )
    assert float(openamber.get_entity("cooling_frequency_index_1")) == 20.0, (
        "cooling_frequency_index_1 should be overwritten to 20 when setting is disabled"
    )
    assert float(openamber.get_entity("cooling_frequency_index_2")) == 26.0, (
        "cooling_frequency_index_2 should be overwritten to 26 when setting is disabled"
    )


def test_default_compressor_frequencies_ui_switch_and_dialog(openamber):
    """
    Verify navigating to Settings -> Geavanceerd -> Compressor and toggling
    the 'Gebruik standaard compressorfrequenties' switch shows the restart dialog
    when enabled, and can be dismissed.
    """
    assert openamber.click("nav_settings"), "Clicking nav_settings must succeed"
    openamber.step(ms=100)
    assert openamber.is_visible("page_settings"), "page_settings must be visible"

    # Enable advanced settings so stab_advanced is visible
    openamber.set_switch("advanced_settings_enabled", True)
    openamber.step(ms=100)
    assert openamber.is_visible("stab_advanced"), "stab_advanced must be visible when advanced_settings_enabled is True"

    # Click Geavanceerd tab
    assert openamber.click("stab_advanced"), "Clicking stab_advanced must succeed"
    openamber.step(ms=50)

    # Click Compressor subtab (index 3)
    assert openamber.click("advanced_stab_compressor"), "Clicking advanced_stab_compressor must succeed"
    openamber.step(ms=50)

    # Widget should be visible and modal should be hidden
    assert openamber.is_visible("panel_advanced_page_compressor"), (
        "panel_advanced_page_compressor must be visible"
    )
    assert openamber.is_visible("set_default_compressor_frequencies_enabled_switch_ui"), (
        "set_default_compressor_frequencies_enabled_switch_ui must be visible"
    )
    assert not openamber.is_visible("default_compressor_frequencies_modal"), (
        "default_compressor_frequencies_modal must initially be hidden"
    )

    # Initially switch is unchecked (False)
    assert openamber.get_entity("default_compressor_frequencies_enabled") is False
    widget_info = openamber.get_widget("set_default_compressor_frequencies_enabled_switch_ui")
    assert widget_info.get("checked") is False, "Switch widget should initially be unchecked"

    # Click to toggle ON -> should enable setting AND show restart dialog
    assert openamber.click("set_default_compressor_frequencies_enabled_switch_ui")
    openamber.step(ms=50)

    assert openamber.get_entity("default_compressor_frequencies_enabled") is True, (
        "Entity state should be True after clicking switch widget"
    )
    widget_info = openamber.get_widget("set_default_compressor_frequencies_enabled_switch_ui")
    assert widget_info.get("checked") is True, "Switch widget should be checked after click"

    assert openamber.is_visible("default_compressor_frequencies_modal"), (
        "default_compressor_frequencies_modal must be shown when enabling the setting"
    )
    modal_text = openamber.get_label("default_compressor_frequencies_modal_text")
    assert "buitenunit" in modal_text and "stroom" in modal_text, (
        f"Modal text should instruct power-cycling the heat pump including outdoor unit, got: {modal_text}"
    )

    # Dismiss dialog by clicking 'Begrepen' button
    assert openamber.click("default_compressor_frequencies_modal_ok_button")
    openamber.step(ms=50)

    assert not openamber.is_visible("default_compressor_frequencies_modal"), (
        "default_compressor_frequencies_modal must be hidden after clicking OK button"
    )
    assert openamber.get_entity("default_compressor_frequencies_enabled") is True, (
        "Entity state should remain True after dismissing dialog"
    )

    # Click to toggle OFF -> should disable setting WITHOUT showing dialog
    assert openamber.click("set_default_compressor_frequencies_enabled_switch_ui")
    openamber.step(ms=50)

    assert openamber.get_entity("default_compressor_frequencies_enabled") is False, (
        "Entity state should be False after second click"
    )
    widget_info = openamber.get_widget("set_default_compressor_frequencies_enabled_switch_ui")
    assert widget_info.get("checked") is False, "Switch widget should be unchecked after second click"
    assert not openamber.is_visible("default_compressor_frequencies_modal"), (
        "default_compressor_frequencies_modal must NOT be shown when disabling the setting"
    )

    # Reset advanced_settings_enabled
    openamber.set_switch("advanced_settings_enabled", False)
    openamber.step(ms=50)
