import pytest


def test_settings_overview_navigation(openamber):
    """Verify clicking category rows in Settings overview navigates to subpages and back."""
    # Ensure on Settings page
    assert openamber.click("nav_settings"), "Clicking nav_settings must succeed"
    openamber.step(ms=100)
    assert openamber.is_visible("page_settings"), "page_settings must be visible"

    # Click Algemeen on overview
    assert openamber.click("stab_general"), "Clicking stab_general must succeed"
    openamber.step(ms=100)
    assert openamber.is_visible("panel_general"), "panel_general must be visible when Algemeen is selected"
    assert openamber.is_hidden("panel_heating"), "panel_heating must be hidden when Algemeen is selected"

    # Navigate back to overview via header back button
    assert openamber.click("settings_menu_back_btn"), "Clicking settings_menu_back_btn must succeed"
    openamber.step(ms=100)
    assert openamber.is_visible("settings_overview_page"), "settings_overview_page must be visible after back"

    # Click Verwarmen on overview
    assert openamber.click("stab_heating"), "Clicking stab_heating must succeed"
    openamber.step(ms=100)
    assert openamber.is_visible("panel_heating"), "panel_heating must be visible when Verwarmen is selected"
    assert openamber.is_hidden("panel_general"), "panel_general must be hidden when Verwarmen is selected"

    # Navigate back to overview
    assert openamber.click("settings_menu_back_btn"), "Clicking settings_menu_back_btn must succeed"
    openamber.step(ms=100)

    # Click Bijverwarmen on overview
    assert openamber.click("stab_bijverwarmen"), "Clicking stab_bijverwarmen must succeed"
    openamber.step(ms=100)
    assert openamber.is_visible("panel_bijverwarmen"), "panel_bijverwarmen must be visible when Bijverwarmen is selected"
    assert openamber.is_hidden("panel_heating"), "panel_heating must be hidden when Bijverwarmen is selected"


def test_sensor_calibration_pager_workflow(openamber):
    """Verify navigating to sensor calibration subpages (Ruimte & Tapwater, CV-Systeem)."""
    assert openamber.click("nav_settings"), "Clicking nav_settings must succeed"
    openamber.step(ms=100)
    assert openamber.is_visible("page_settings"), "page_settings must be visible"

    # Select Algemeen -> Sensor Kalibratie
    assert openamber.click("stab_general"), "Clicking stab_general must succeed"
    openamber.step(ms=100)
    assert openamber.click("general_stab_sensor_kalibratie"), "Clicking general_stab_sensor_kalibratie must succeed"
    openamber.step(ms=100)
    assert openamber.is_visible("panel_general_page_sensor_kalibratie"), "panel_general_page_sensor_kalibratie must be visible"

    # 1. Ruimte & Tapwater subcategory (<= 4 settings: Tr, Tw)
    assert openamber.click("sensor_kalibratie_tab_ruimte"), "Clicking sensor_kalibratie_tab_ruimte must succeed"
    openamber.step(ms=100)
    assert openamber.is_visible("panel_general_sensor_kalibratie_ruimte"), "Ruimte subpage must be visible"
    assert openamber.is_visible("sensor_calibration_page_1_row_tr"), "Row Tr must be visible"
    assert openamber.is_visible("sensor_calibration_page_1_row_tw"), "Row Tw must be visible"

    # Back to Sensor kalibratie choices
    assert openamber.click("settings_menu_back_btn"), "Clicking back button must succeed"
    openamber.step(ms=100)
    assert openamber.is_visible("panel_general_page_sensor_kalibratie"), "Choice page must be visible"

    # 2. CV-Systeem subcategory (<= 4 settings: Tc, Tui, Tuo)
    assert openamber.click("sensor_kalibratie_tab_cv"), "Clicking sensor_kalibratie_tab_cv must succeed"
    openamber.step(ms=100)
    assert openamber.is_visible("panel_general_sensor_kalibratie_cv"), "CV subpage must be visible"
    assert openamber.is_visible("sensor_calibration_page_1_row_tc"), "Row Tc must be visible"
    assert openamber.is_visible("sensor_calibration_page_1_row_tui"), "Row Tui must be visible"
    assert openamber.is_visible("sensor_calibration_page_2_row_tuo"), "Row Tuo must be visible"


def test_zone2_stooklijn_and_mixing_valves_subtabs(openamber):
    """Verify Zone 2 Stooklijn and Mengventielen Zone 1 & Zone 2 subpages."""
    assert openamber.click("nav_settings"), "Clicking nav_settings must succeed"
    openamber.step(ms=100)
    assert openamber.is_visible("page_settings"), "page_settings must be visible"

    # 1. Under Verwarmen (stab_heating): Click Stooklijn Z2 subpage
    assert openamber.click("stab_heating"), "Clicking stab_heating must succeed"
    openamber.step(ms=50)
    assert openamber.click("heating_stab_stooklijn_z2"), "Clicking heating_stab_stooklijn_z2 must succeed"
    openamber.step(ms=50)
    assert openamber.is_visible("panel_heating_page_stooklijn_z2"), "panel_heating_page_stooklijn_z2 must be visible"

    # 2. Under Algemeen -> Mengventielen
    assert openamber.click("nav_settings"), "Clicking nav_settings must succeed"
    openamber.step(ms=100)
    assert openamber.click("stab_general"), "Clicking stab_general must succeed"
    openamber.step(ms=50)
    assert openamber.click("general_stab_mengventielen"), "Clicking general_stab_mengventielen must succeed"
    openamber.step(ms=50)
    assert openamber.is_visible("panel_general_page_mengventielen"), "panel_general_page_mengventielen must be visible"

    # Click Mengventiel Zone 1 (<= 4 settings: enabled, min, max)
    assert openamber.click("mengventielen_tab_z1"), "Clicking mengventielen_tab_z1 must succeed"
    openamber.step(ms=100)
    assert openamber.is_visible("panel_general_mv_zone1"), "panel_general_mv_zone1 must be visible"
    assert openamber.is_visible("set_general_mv_z1_enabled"), "Zone 1 settings row must be visible"

    # Back to Mengventielen choices
    assert openamber.click("settings_menu_back_btn"), "Clicking back button must succeed"
    openamber.step(ms=100)

    # Click Mengventiel Zone 2 (<= 4 settings: enabled, min, max)
    assert openamber.click("mengventielen_tab_z2"), "Clicking mengventielen_tab_z2 must succeed"
    openamber.step(ms=100)
    assert openamber.is_visible("panel_general_mv_zone2"), "panel_general_mv_zone2 must be visible"
    assert openamber.is_visible("set_general_mv_z2_enabled"), "Zone 2 settings row must be visible"


def test_general_systeem_and_opties(openamber):
    """Verify Systeem and Opties subpages under Algemeen."""
    assert openamber.click("nav_settings"), "Clicking nav_settings must succeed"
    openamber.step(ms=100)
    assert openamber.click("stab_general"), "Clicking stab_general must succeed"
    openamber.step(ms=50)

    # Systeem subpage (<= 4 settings)
    assert openamber.click("general_stab_systeem"), "Clicking general_stab_systeem must succeed"
    openamber.step(ms=100)
    assert openamber.is_visible("panel_general_page_systeem"), "panel_general_page_systeem must be visible"
    assert openamber.is_visible("set_general_system_dhw"), "Tapwater row must be visible"
    assert openamber.is_visible("set_general_system_control_temp_source"), "Regeltemp row must be visible"
    assert openamber.is_visible("set_general_defrost_advanced_toggle_row"), "Geavanceerd row must be visible"
    assert openamber.is_visible("set_general_system_analytics"), "Analytics row must be visible"

    # Back to Algemeen choices
    assert openamber.click("settings_menu_back_btn"), "Clicking back button must succeed"
    openamber.step(ms=100)

    # Opties (Hardware) subpage (<= 4 settings)
    assert openamber.click("general_stab_hardware"), "Clicking general_stab_hardware must succeed"
    openamber.step(ms=100)
    assert openamber.is_visible("panel_general_page_hardware"), "panel_general_page_hardware must be visible"
    assert openamber.is_visible("set_general_hardware_flow_sensor"), "Flow sensor row must be visible"
    assert openamber.is_visible("set_general_hardware_flow_sensor_calibration"), "Kalibratie row must be visible"


def test_service_overview_navigation(openamber):
    """Verify clicking category rows in Service overview navigates to subpages and back."""
    assert openamber.click("nav_service"), "Clicking nav_service must succeed"
    openamber.step(ms=100)
    assert openamber.is_visible("page_service"), "page_service must be visible"

    # Initially Service Overview page is root page and visible
    assert openamber.is_visible("service_overview_page"), "service_overview_page must be visible initially"

    # Click Algemeen & Modus (service_tab_maintenance)
    assert openamber.click("service_tab_maintenance"), "Clicking service_tab_maintenance must succeed"
    openamber.step(ms=100)
    assert openamber.is_visible("service_maintenance_panel"), "service_maintenance_panel must be visible"
    assert openamber.is_hidden("service_overview_page"), "service_overview_page must be hidden"

    # Click back button to return to service overview
    assert openamber.click("service_menu_back_btn"), "Clicking service_menu_back_btn must succeed"
    openamber.step(ms=100)
    assert openamber.is_visible("service_overview_page"), "service_overview_page must be visible again"

    # Click Buitenunit (service_tab_compressor)
    assert openamber.click("service_tab_compressor"), "Clicking service_tab_compressor must succeed"
    openamber.step(ms=100)
    assert openamber.is_visible("service_mpanel_compressor"), "service_mpanel_compressor must be visible"
    assert openamber.is_hidden("service_overview_page"), "service_overview_page must be hidden"

    # Click back button to return to service overview
    assert openamber.click("service_menu_back_btn"), "Clicking service_menu_back_btn must succeed"
    openamber.step(ms=100)
    assert openamber.is_visible("service_overview_page"), "service_overview_page must be visible again"




