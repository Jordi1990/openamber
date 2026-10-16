import pytest


def test_settings_sidebar_tabs(openamber):
    """Verify clicking sidebar tabs in the Settings page switches both labels and panels."""
    # Ensure on Settings page
    assert openamber.click("nav_settings"), "Clicking nav_settings must succeed"
    openamber.step(ms=100)
    assert openamber.is_visible("page_settings"), "page_settings must be visible"

    # Click Algemeen tab
    assert openamber.click("stab_general"), "Clicking stab_general must succeed"
    openamber.step(ms=50)
    assert openamber.get_label("stab_general_lbl") == "Algemeen", "stab_general_lbl should be 'Algemeen'"
    assert openamber.is_visible("panel_general"), "panel_general must be visible when Algemeen tab is selected"
    assert openamber.is_hidden("panel_heating"), "panel_heating must be hidden when Algemeen tab is selected"

    # Click Verwarmen tab
    assert openamber.click("stab_heating"), "Clicking stab_heating must succeed"
    openamber.step(ms=50)
    assert openamber.get_label("stab_heating_lbl") == "Verwarmen", "stab_heating_lbl should be 'Verwarmen'"
    assert openamber.is_visible("panel_heating"), "panel_heating must be visible when Verwarmen tab is selected"
    assert openamber.is_hidden("panel_general"), "panel_general must be hidden when Verwarmen tab is selected"

    # Click Tapwater tab
    assert openamber.click("stab_dhw"), "Clicking stab_dhw must succeed"
    openamber.step(ms=50)
    assert openamber.get_label("stab_dhw_lbl") == "Tapwater", "stab_dhw_lbl should be 'Tapwater'"
    assert openamber.is_visible("panel_dhw"), "panel_dhw must be visible when Tapwater tab is selected"
    assert openamber.is_hidden("panel_heating"), "panel_heating must be hidden when Tapwater tab is selected"

    # Click Bijverwarmen tab
    assert openamber.click("stab_bijverwarmen"), "Clicking stab_bijverwarmen must succeed"
    openamber.step(ms=50)
    assert openamber.get_label("stab_bijverwarmen_lbl") == "Bijverwarmen", "stab_bijverwarmen_lbl should be 'Bijverwarmen'"
    assert openamber.is_visible("panel_bijverwarmen"), "panel_bijverwarmen must be visible when Bijverwarmen tab is selected"
    assert openamber.is_hidden("panel_dhw"), "panel_dhw must be hidden when Bijverwarmen tab is selected"


def test_sensor_calibration_pager_workflow(openamber):
    """Verify navigating to sensor calibration subpage and verifying sensor offset rows."""
    assert openamber.click("nav_settings"), "Clicking nav_settings must succeed"
    openamber.step(ms=100)
    assert openamber.is_visible("page_settings"), "page_settings must be visible"

    # Select Algemeen tab where sensor calibration drill-down resides
    assert openamber.click("stab_general"), "Clicking stab_general must succeed"
    openamber.step(ms=100)

    # Click drill-down button to Sensor Kalibratie subpage
    assert openamber.click("general_stab_sensor_kalibratie"), "Clicking general_stab_sensor_kalibratie must succeed"
    openamber.step(ms=100)
    assert openamber.is_visible("panel_general_page_sensor_kalibratie"), "panel_general_page_sensor_kalibratie must be visible"

    # Verify sensor offset rows are visible (unified without pagination)
    assert openamber.is_visible("sensor_calibration_page_1_row_tr"), "Row Tr must be visible"
    assert openamber.is_visible("sensor_calibration_page_1_row_tw"), "Row Tw must be visible"
    assert openamber.is_visible("sensor_calibration_page_1_row_tc"), "Row Tc must be visible"
    assert openamber.is_visible("sensor_calibration_page_1_row_tui"), "Row Tui must be visible"
    assert openamber.is_visible("sensor_calibration_page_2_row_tuo"), "Row Tuo must be visible"


def test_zone2_stooklijn_and_mixing_valves_subtabs(openamber):
    """Verify Zone 2 Stooklijn and Mengventielen UI subpages."""
    assert openamber.click("nav_settings"), "Clicking nav_settings must succeed"
    openamber.step(ms=100)
    assert openamber.is_visible("page_settings"), "page_settings must be visible"

    # 1. Under Verwarmen (stab_heating): Click Stooklijn Z2 subpage
    assert openamber.click("stab_heating"), "Clicking stab_heating must succeed"
    openamber.step(ms=50)
    assert openamber.click("heating_stab_stooklijn_z2"), "Clicking heating_stab_stooklijn_z2 must succeed"
    openamber.step(ms=50)
    assert openamber.is_visible("panel_heating_page_stooklijn_z2"), "panel_heating_page_stooklijn_z2 must be visible"

    # 2. Under Algemeen (stab_general): Click Mengventielen subpage
    assert openamber.click("stab_general"), "Clicking stab_general must succeed"
    openamber.step(ms=50)
    assert openamber.click("general_stab_mengventielen"), "Clicking general_stab_mengventielen must succeed"
    openamber.step(ms=50)
    assert openamber.is_visible("panel_general_page_mengventielen"), "panel_general_page_mengventielen must be visible"

    # Both Zone 1 and Zone 2 settings rows are directly present without pagination
    assert openamber.is_visible("set_general_mv_z1_enabled"), "Zone 1 settings row must be visible"
    assert openamber.is_visible("set_general_mv_z2_enabled"), "Zone 2 settings row must be visible"


def test_service_sidebar_tabs(openamber):
    """Verify clicking sidebar tabs in the Service page switches menu pages."""
    assert openamber.click("nav_service"), "Clicking nav_service must succeed"
    openamber.step(ms=100)
    assert openamber.is_visible("page_service"), "page_service must be visible"

    # Initially Algemeen (maintenance) is root page and visible
    assert openamber.is_visible("service_maintenance_panel"), "service_maintenance_panel must be visible initially"

    # Click Compressor (Buitenunit) tab
    assert openamber.click("service_tab_compressor"), "Clicking service_tab_compressor must succeed"
    openamber.step(ms=100)
    assert openamber.is_visible("service_mpanel_compressor"), "service_mpanel_compressor must be visible"
    assert openamber.is_hidden("service_maintenance_panel"), "service_maintenance_panel must be hidden"

    # Click back to Maintenance (Algemeen) tab
    assert openamber.click("service_tab_maintenance"), "Clicking service_tab_maintenance must succeed"
    openamber.step(ms=100)
    assert openamber.is_visible("service_maintenance_panel"), "service_maintenance_panel must be visible again"
    assert openamber.is_hidden("service_mpanel_compressor"), "service_mpanel_compressor must be hidden"



