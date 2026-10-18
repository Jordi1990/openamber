"""Automated tests for sensor source selection and UI settings configuration.

Covers:
1. heat_cool_control_temperature_source_select:
   Selects process control temperature between Tc (heating/cooling flow) and Tv1 (zone 1 flow).
2. advanced_settings_enabled:
   Toggles advanced settings tab visibility (stab_advanced) in the Settings page UI.
"""

import pytest


def test_heat_cool_control_temperature_source_selection(clean_system):
    """
    Verify heat_cool_control_temperature_source_select:
    - Option 'CV-aanvoer (Tc)': heat_cool_control_temperature reflects Tc.
    - Option 'CV-aanvoer zone 1 (Tv1)': heat_cool_control_temperature reflects Tv1.
    """
    openamber = clean_system

    # Set distinct temperatures for Tc and Tv1
    openamber.set_sensor("current_water_temperature_tc_sensor", 38.0)
    openamber.set_sensor("heat_cool_temperature_tc", 38.0)
    openamber.set_sensor("heat_cool_temperature_tv1", 30.0)
    openamber.step(ms=50)

    # 1. Source = Tc
    openamber.set_select("heat_cool_control_temperature_source_select", "CV-aanvoer (Tc)")
    openamber.advance_time(seconds=6, step_s=1)
    openamber.step(ms=100)

    ctrl_temp = float(openamber.get_entity("heat_cool_control_temperature") or 0)
    assert ctrl_temp == pytest.approx(38.0, abs=0.5), f"Expected control temp to track Tc (38°C), got {ctrl_temp}"

    # 2. Source = Tv1
    openamber.set_select("heat_cool_control_temperature_source_select", "CV-aanvoer zone 1 (Tv1)")
    openamber.advance_time(seconds=6, step_s=1)
    openamber.step(ms=100)

    ctrl_temp = float(openamber.get_entity("heat_cool_control_temperature") or 0)
    assert ctrl_temp == pytest.approx(30.0, abs=0.5), f"Expected control temp to track Tv1 (30°C), got {ctrl_temp}"

    # Reset back to Tc
    openamber.set_select("heat_cool_control_temperature_source_select", "CV-aanvoer (Tc)")
    openamber.advance_time(seconds=6, step_s=1)
    openamber.step(ms=50)


def test_advanced_settings_switch_controls_ui_tab_visibility(clean_system):
    """
    Verify that advanced_settings_enabled actively toggles advanced tab visibility:
    - In Settings page, stab_advanced is hidden when advanced_settings_enabled = False.
    - Turning advanced_settings_enabled = True makes stab_advanced visible.
    - Turning it back OFF hides stab_advanced again.
    """
    openamber = clean_system

    # Navigate to Settings page
    assert openamber.click("nav_settings"), "Clicking nav_settings must succeed"
    openamber.step(ms=100)
    assert openamber.is_visible("page_settings"), "page_settings should be visible"

    # Disable advanced settings -> tab must be hidden
    openamber.set_switch("advanced_settings_enabled", False)
    openamber.step(ms=100)
    assert openamber.is_hidden("stab_advanced"), "stab_advanced tab must be hidden when advanced_settings_enabled is False"

    # Enable advanced settings -> tab becomes visible
    openamber.set_switch("advanced_settings_enabled", True)
    openamber.step(ms=100)
    assert openamber.is_visible("stab_advanced"), "stab_advanced tab must be visible when advanced_settings_enabled is True"

    # Turn back off -> tab hidden again
    openamber.set_switch("advanced_settings_enabled", False)
    openamber.step(ms=100)
    assert openamber.is_hidden("stab_advanced"), "stab_advanced tab must return to hidden state"
