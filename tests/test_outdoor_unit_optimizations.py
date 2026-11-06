"""Automated tests for the 'Buitenunit optimalisaties' setting.

Covers:
1. Entity existence & default state:
   - outdoor_unit_optimizations_enabled defaults to True (optimizations active).
   - Toggling the setting updates its state.
2. Outdoor unit writes during initialization:
   - When outdoor_unit_optimizations_enabled is True, initialization writes OpenAmber's frequency tables
     and turns off the unit's own ambient temperature limit (3281 = 0).
   - When outdoor_unit_optimizations_enabled is False, initialization writes nothing to the outdoor unit,
     preserving the frequencies and the ambient limit setting on the unit.
3. LVGL UI integration & restart dialog:
   - Widget set_outdoor_unit_optimizations_enabled_switch_ui in Settings -> Geavanceerd -> Compressor.
   - Clicking the UI switch to disable turns outdoor_unit_optimizations_enabled OFF and shows
     outdoor_unit_optimizations_modal instructing the user to power-cycle the complete heat pump
     including outdoor unit.
   - Clicking outdoor_unit_optimizations_modal_ok_button dismisses the modal.
   - Clicking the UI switch to enable turns outdoor_unit_optimizations_enabled ON without showing the modal.
"""

import pytest


def _reinitialize(openamber):
    """Re-run initialization via a maintenance mode cycle."""
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


def test_outdoor_unit_optimizations_entity_default_and_toggle(clean_system):
    """Verify outdoor_unit_optimizations_enabled defaults to True and can be toggled."""
    openamber = clean_system

    assert openamber.get_entity("outdoor_unit_optimizations_enabled") is True, (
        "outdoor_unit_optimizations_enabled should be True by default"
    )

    assert openamber.set_switch("outdoor_unit_optimizations_enabled", False) is True
    openamber.step(ms=50)
    assert openamber.get_entity("outdoor_unit_optimizations_enabled") is False

    assert openamber.set_switch("outdoor_unit_optimizations_enabled", True) is True
    openamber.step(ms=50)
    assert openamber.get_entity("outdoor_unit_optimizations_enabled") is True


def test_outdoor_unit_optimizations_control_outdoor_unit_writes(clean_system):
    """
    With the optimizations disabled, initialization must not write the frequency tables nor the
    ambient limit control (3281). With them enabled, it writes both.
    """
    openamber = clean_system
    openamber.advance_time(seconds=20, step_s=5)
    openamber.step(ms=50)

    # Simulate the stock values in the unit
    custom_heat_1 = 99.0
    custom_heat_2 = 98.0
    custom_cool_1 = 77.0
    custom_cool_2 = 76.0
    openamber.set_number("heating_frequency_index_1", custom_heat_1)
    openamber.set_number("heating_frequency_index_2", custom_heat_2)
    openamber.set_number("cooling_frequency_index_1", custom_cool_1)
    openamber.set_number("cooling_frequency_index_2", custom_cool_2)
    openamber.set_number("outdoor_unit_ambient_limit_control", 1)
    openamber.step(ms=50)

    # 1. Disable optimizations -> nothing may be written on re-init
    assert openamber.set_switch("outdoor_unit_optimizations_enabled", False) is True
    openamber.step(ms=50)
    _reinitialize(openamber)

    assert float(openamber.get_entity("heating_frequency_index_1")) == custom_heat_1
    assert float(openamber.get_entity("heating_frequency_index_2")) == custom_heat_2
    assert float(openamber.get_entity("cooling_frequency_index_1")) == custom_cool_1
    assert float(openamber.get_entity("cooling_frequency_index_2")) == custom_cool_2
    assert int(openamber.get_entity("outdoor_unit_ambient_limit_control")) == 1, (
        "The unit's own ambient limit must stay on when the optimizations are disabled"
    )

    # 2. Enable optimizations -> OpenAmber's tables and 3281 = 0 are written on re-init
    assert openamber.set_switch("outdoor_unit_optimizations_enabled", True) is True
    openamber.step(ms=50)
    _reinitialize(openamber)

    # OpenAmber table values: heating index 1 = 20, index 2 = 28, cooling index 1 = 20, index 2 = 26
    assert float(openamber.get_entity("heating_frequency_index_1")) == 20.0
    assert float(openamber.get_entity("heating_frequency_index_2")) == 28.0
    assert float(openamber.get_entity("cooling_frequency_index_1")) == 20.0
    assert float(openamber.get_entity("cooling_frequency_index_2")) == 26.0
    assert int(openamber.get_entity("outdoor_unit_ambient_limit_control")) == 0, (
        "OpenAmber must turn off the unit's own ambient limit when the optimizations are enabled"
    )


def test_outdoor_unit_optimizations_ui_switch_and_dialog(openamber):
    """
    Verify navigating to Settings -> Geavanceerd -> Compressor and toggling the
    'Buitenunit optimalisaties' switch shows the restart dialog when disabled, and can be dismissed.
    """
    assert openamber.click("nav_settings"), "Clicking nav_settings must succeed"
    openamber.step(ms=100)
    assert openamber.is_visible("page_settings"), "page_settings must be visible"

    openamber.set_switch("outdoor_unit_optimizations_enabled", True)
    openamber.set_switch("advanced_settings_enabled", True)
    openamber.step(ms=100)
    assert openamber.is_visible("stab_advanced"), "stab_advanced must be visible when advanced_settings_enabled is True"

    assert openamber.click("stab_advanced"), "Clicking stab_advanced must succeed"
    openamber.step(ms=50)
    assert openamber.click("advanced_stab_compressor"), "Clicking advanced_stab_compressor must succeed"
    openamber.step(ms=50)

    assert openamber.is_visible("panel_advanced_page_compressor")
    assert openamber.is_visible("set_outdoor_unit_optimizations_enabled_switch_ui")
    assert not openamber.is_visible("outdoor_unit_optimizations_modal"), (
        "outdoor_unit_optimizations_modal must initially be hidden"
    )

    widget_info = openamber.get_widget("set_outdoor_unit_optimizations_enabled_switch_ui")
    assert widget_info.get("checked") is True, "Switch widget should initially be checked"

    # Click to toggle OFF -> should disable setting AND show restart dialog
    assert openamber.click("set_outdoor_unit_optimizations_enabled_switch_ui")
    openamber.step(ms=50)

    assert openamber.get_entity("outdoor_unit_optimizations_enabled") is False
    widget_info = openamber.get_widget("set_outdoor_unit_optimizations_enabled_switch_ui")
    assert widget_info.get("checked") is False, "Switch widget should be unchecked after click"

    assert openamber.is_visible("outdoor_unit_optimizations_modal"), (
        "outdoor_unit_optimizations_modal must be shown when disabling the setting"
    )
    modal_text = openamber.get_label("outdoor_unit_optimizations_modal_text")
    assert "buitenunit" in modal_text and "stroom" in modal_text, (
        f"Modal text should instruct power-cycling the heat pump including outdoor unit, got: {modal_text}"
    )

    # Dismiss dialog by clicking 'Begrepen' button
    assert openamber.click("outdoor_unit_optimizations_modal_ok_button")
    openamber.step(ms=50)
    assert not openamber.is_visible("outdoor_unit_optimizations_modal")
    assert openamber.get_entity("outdoor_unit_optimizations_enabled") is False

    # Click to toggle ON -> should enable setting WITHOUT showing dialog
    assert openamber.click("set_outdoor_unit_optimizations_enabled_switch_ui")
    openamber.step(ms=50)

    assert openamber.get_entity("outdoor_unit_optimizations_enabled") is True
    widget_info = openamber.get_widget("set_outdoor_unit_optimizations_enabled_switch_ui")
    assert widget_info.get("checked") is True, "Switch widget should be checked after second click"
    assert not openamber.is_visible("outdoor_unit_optimizations_modal"), (
        "outdoor_unit_optimizations_modal must NOT be shown when enabling the setting"
    )

    openamber.set_switch("advanced_settings_enabled", False)
    openamber.step(ms=50)
