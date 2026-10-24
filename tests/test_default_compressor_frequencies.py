"""Automated tests for 'Gebruik standaard compressorfrequenties' setting.

Covers:
1. Entity existence & default state:
   - default_compressor_frequencies_enabled defaults to False (overwriting active).
   - Toggling the setting updates its state.
2. Preventing compressor frequency overwrite:
   - When default_compressor_frequencies_enabled is False, initialization writes OpenAmber's custom frequencies.
   - When default_compressor_frequencies_enabled is True, initialization skips writing frequency tables,
     preserving any existing frequencies on the unit.
3. LVGL UI integration:
   - Widget set_default_compressor_frequencies_enabled_switch_ui in Settings -> Algemeen -> Opties.
   - Clicking the UI switch toggles default_compressor_frequencies_enabled.
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


def test_default_compressor_frequencies_ui_switch(openamber):
    """
    Verify navigating to Settings -> Algemeen -> Opties and toggling
    the 'Gebruik standaard compressorfrequenties' switch.
    """
    assert openamber.click("nav_settings"), "Clicking nav_settings must succeed"
    openamber.step(ms=100)
    assert openamber.is_visible("page_settings"), "page_settings must be visible"

    # Click Algemeen tab
    assert openamber.click("stab_general"), "Clicking stab_general must succeed"
    openamber.step(ms=50)

    # Click Opties subtab (index 3)
    assert openamber.click("general_stab_hardware"), "Clicking general_stab_hardware (Opties) must succeed"
    openamber.step(ms=50)

    # Widget should be visible
    assert openamber.is_visible("set_default_compressor_frequencies_enabled_switch_ui"), (
        "set_default_compressor_frequencies_enabled_switch_ui must be visible"
    )

    # Initially switch is unchecked (False)
    assert openamber.get_entity("default_compressor_frequencies_enabled") is False
    widget_info = openamber.get_widget("set_default_compressor_frequencies_enabled_switch_ui")
    assert widget_info.get("checked") is False, "Switch widget should initially be unchecked"

    # Click to toggle ON
    assert openamber.click("set_default_compressor_frequencies_enabled_switch_ui")
    openamber.step(ms=50)

    assert openamber.get_entity("default_compressor_frequencies_enabled") is True, (
        "Entity state should be True after clicking switch widget"
    )
    widget_info = openamber.get_widget("set_default_compressor_frequencies_enabled_switch_ui")
    assert widget_info.get("checked") is True, "Switch widget should be checked after click"

    # Click to toggle OFF
    assert openamber.click("set_default_compressor_frequencies_enabled_switch_ui")
    openamber.step(ms=50)

    assert openamber.get_entity("default_compressor_frequencies_enabled") is False, (
        "Entity state should be False after second click"
    )
    widget_info = openamber.get_widget("set_default_compressor_frequencies_enabled_switch_ui")
    assert widget_info.get("checked") is False, "Switch widget should be unchecked after second click"
