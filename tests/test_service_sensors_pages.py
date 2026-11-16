"""Automated tests for the Service Diagnose menu pages, demand and external signal rows,
DHW dropdown UI bindings, and Thermostat off-delay UI behavior.
"""

import pytest


def test_service_sensors_navigation(clean_system):
    """
    Verify:
    1. Navigation from navbar -> Service page -> Diagnose.
    2. Each of the five sensor cards opens its own menu page with its table.
    3. The menu back button returns to the Diagnose page.
    """
    openamber = clean_system

    # 1. Navigate to Service page
    assert openamber.click("nav_service"), "Clicking nav_service must succeed"
    openamber.step(ms=100)
    assert openamber.is_visible("page_service"), "page_service should be visible"

    # 2. Open Diagnose from the service overview
    assert openamber.click("service_tab_sensors"), "Clicking service_tab_sensors must succeed"
    openamber.step(ms=100)
    assert openamber.is_visible("service_sensors_panel"), "service_sensors_panel should be visible"

    pages = [
        ("service_sensors_stab_status", "service_sensors_status_page", "service_diag_status_table"),
        ("service_sensors_stab_temp", "service_sensors_temp_page", "service_diag_temp_table"),
        ("service_sensors_stab_control", "service_sensors_control_page", "service_diag_control_table"),
        ("service_sensors_stab_system", "service_sensors_system_page", "service_diag_system_table"),
        ("service_sensors_stab_demand", "service_sensors_demand_page", "service_diag_demand_table"),
    ]
    for card, page, table in pages:
        assert openamber.click(card), f"Clicking {card} must succeed"
        openamber.step(ms=100)
        assert openamber.is_visible(page), f"{page} should be visible after clicking {card}"
        assert openamber.is_visible(table), f"{table} should be visible on {page}"
        assert openamber.is_hidden("service_sensors_panel"), "The overview cards should be hidden on a sensor page"

        # 3. Back to the Diagnose page
        assert openamber.click("service_menu_back_btn"), "Clicking the menu back button must succeed"
        openamber.step(ms=100)
        assert openamber.is_visible("service_sensors_panel"), "Back must return to the Diagnose page"
        assert openamber.is_hidden(page), f"{page} should be hidden after going back"


def test_service_sensors_demand_and_external_signals_page_5(clean_system):
    """
    Verify the 'Warmte- en koudevraag' page shows both effective demand and raw external signals separately:
    1. Software/internal Zone 1 heat demand sets 'Warmtevraag Zone 1' to 'AAN' while
       'Warmtevraag Zone 1 (Extern signaal)' remains 'UIT'.
    2. Wired Zone 2 input when mixing valves are disabled sets 'Warmtevraag Zone 2 (Extern signaal)'
       to 'AAN' while effective 'Warmtevraag Zone 2' remains 'UIT'; enabling mixing valve sets both to 'AAN'.
    3. Wired cooling input while heating demand is active sets 'Koudevraag (Extern signaal)' to 'AAN'
       while effective 'Koudevraag' remains 'UIT'.
    """
    openamber = clean_system

    # Navigate to Service -> Diagnose -> Warmte- en koudevraag
    assert openamber.click("nav_service")
    openamber.step(ms=100)
    assert openamber.click("service_tab_sensors")
    openamber.step(ms=100)
    assert openamber.click("service_sensors_stab_demand")
    openamber.step(ms=100)
    assert openamber.is_visible("service_sensors_demand_page")

    tbl = openamber.get_widget("service_diag_demand_table")
    assert tbl.get("rows") == [
        ["Warmtevraag Zone 1", "UIT"],
        ["Warmtevraag Zone 2", "UIT"],
        ["Koudevraag", "UIT"],
        ["Warmtevraag Zone 1 (Extern signaal)", "UIT"],
        ["Warmtevraag Zone 2 (Extern signaal)", "UIT"],
        ["Koudevraag (Extern signaal)", "UIT"],
    ]

    # 1. Activate Zone 1 heat demand via software switch (without wired external signal)
    openamber.set_switch("heat_demand_switch", True)
    openamber.step(ms=150)
    rows = dict(openamber.get_widget("service_diag_demand_table").get("rows"))
    assert rows["Warmtevraag Zone 1"] == "AAN"
    assert rows["Warmtevraag Zone 1 (Extern signaal)"] == "UIT"

    # 2. Activate wired Zone 2 signal while mixing valves are disabled
    openamber.set_binary_sensor("external_heat_demand_wired_zone_2", True)
    openamber.step(ms=150)
    rows = dict(openamber.get_widget("service_diag_demand_table").get("rows"))
    assert rows["Warmtevraag Zone 2"] == "UIT"
    assert rows["Warmtevraag Zone 2 (Extern signaal)"] == "AAN"

    # Enable mixing valve -> effective Zone 2 heat demand now becomes AAN as well
    openamber.set_switch("mixing_valve_zone2_enabled", True)
    openamber.step(ms=150)
    rows = dict(openamber.get_widget("service_diag_demand_table").get("rows"))
    assert rows["Warmtevraag Zone 2"] == "AAN"
    assert rows["Warmtevraag Zone 2 (Extern signaal)"] == "AAN"

    # Deactivate wired Zone 2 signal while mixing valve remains enabled -> both return to UIT
    openamber.set_binary_sensor("external_heat_demand_wired_zone_2", False)
    openamber.step(ms=150)
    rows = dict(openamber.get_widget("service_diag_demand_table").get("rows"))
    assert rows["Warmtevraag Zone 2"] == "UIT"
    assert rows["Warmtevraag Zone 2 (Extern signaal)"] == "UIT"

    # 3. Activate wired cooling signal while heat demand is active -> effective Koudevraag stays UIT
    openamber.set_binary_sensor("external_cool_demand_wired", True)
    openamber.step(ms=150)
    rows = dict(openamber.get_widget("service_diag_demand_table").get("rows"))
    assert rows["Koudevraag"] == "UIT"
    assert rows["Koudevraag (Extern signaal)"] == "AAN"

    # Cleanup
    openamber.set_switch("heat_demand_switch", False)
    openamber.set_binary_sensor("external_heat_demand_wired_zone_2", False)
    openamber.set_binary_sensor("external_cool_demand_wired", False)
    openamber.set_switch("mixing_valve_zone2_enabled", False)
    openamber.step(ms=100)


def test_dhw_power_and_winter_dropdown_ui_bindings(clean_system):
    """
    Verify that changing DHW select entities (as from Home Assistant / Web UI / flash restore)
    updates the LVGL dropdown widgets on the display:
    - dhw_compressor_mode -> set_dhw_power_dd ('Basisvermogen')
    - dhw_compressor_mode_max -> set_dhw_winter_dd ('Wintervermogen')
    - dhw_pump_start_mode_select -> set_dhw_pump_dd ('Tapwaterpomp startmodus')
    """
    openamber = clean_system

    # 1. Basisvermogen (dhw_compressor_mode -> set_dhw_power_dd)
    openamber.set_select("dhw_compressor_mode", "Zeer laag")
    openamber.step(ms=100)
    dd_power = openamber.get_widget("set_dhw_power_dd")
    assert dd_power.get("text") == "Zeer laag", (
        f"Expected set_dhw_power_dd to show 'Zeer laag', got '{dd_power.get('text')}'"
    )
    assert dd_power.get("selected_index") == 1

    openamber.set_select("dhw_compressor_mode", "Beperkt")
    openamber.step(ms=100)
    assert openamber.get_widget("set_dhw_power_dd").get("text") == "Beperkt"

    # 2. Wintervermogen (dhw_compressor_mode_max -> set_dhw_winter_dd)
    openamber.set_select("dhw_compressor_mode_max", "Maximaal")
    openamber.step(ms=100)
    dd_winter = openamber.get_widget("set_dhw_winter_dd")
    assert dd_winter.get("text") == "Maximaal", (
        f"Expected set_dhw_winter_dd to show 'Maximaal', got '{dd_winter.get('text')}'"
    )
    assert dd_winter.get("selected_index") == 3

    openamber.set_select("dhw_compressor_mode_max", "Gemiddeld")
    openamber.step(ms=100)
    assert openamber.get_widget("set_dhw_winter_dd").get("text") == "Gemiddeld"

    # 3. Tapwaterpomp startmodus (dhw_pump_start_mode_select -> set_dhw_pump_dd)
    openamber.set_select("dhw_pump_start_mode_select", "Samen met compressor")
    openamber.step(ms=100)
    assert openamber.get_widget("set_dhw_pump_dd").get("selected_index") == 1

    openamber.set_select("dhw_pump_start_mode_select", "Aanvoer warmer dan vat (ΔT)")
    openamber.step(ms=100)
    assert openamber.get_widget("set_dhw_pump_dd").get("selected_index") == 0


def test_thermostat_off_delay_renamed_and_buttons_disabled_in_internal_mode(clean_system):
    """
    Verify:
    1. thermostat_row_off_delay label text is 'Uitschakelvertraging (Extern signaal)'.
    2. When thermostat_mode_select is 'Intern', thermostat_row_off_delay and its +/- buttons
       (set_therm_off_delay_dec_btn, set_therm_off_delay_inc_btn) are disabled and clicking
       them does NOT modify thermostat_off_delay.
    3. When thermostat_mode_select is 'Extern', the +/- buttons are enabled and increment/decrement
       thermostat_off_delay.
    """
    openamber = clean_system

    # Navigate to Settings -> Thermostaat -> Externe thermostaat
    assert openamber.click("nav_settings")
    openamber.step(ms=100)
    assert openamber.click("stab_thermostat")
    openamber.step(ms=100)
    assert openamber.click("thermostat_stab_extern")
    openamber.step(ms=100)
    assert openamber.is_visible("panel_thermostat_page_extern")

    row_widget = openamber.get_widget("thermostat_row_off_delay")
    assert row_widget.get("text") == "Uitschakelvertraging (Extern signaal)"

    openamber.set_number("thermostat_off_delay", 5.0)
    openamber.set_select("thermostat_mode_select", "Intern")
    openamber.step(ms=100)

    assert openamber.is_disabled("thermostat_row_off_delay")
    assert openamber.is_disabled("set_therm_off_delay_dec_btn")
    assert openamber.is_disabled("set_therm_off_delay_inc_btn")

    # Clicking + and - while disabled in Intern mode must NOT change thermostat_off_delay
    openamber.click("set_therm_off_delay_inc_btn")
    openamber.step(ms=100)
    assert openamber.get_entity("thermostat_off_delay") == pytest.approx(5.0)

    openamber.click("set_therm_off_delay_dec_btn")
    openamber.step(ms=100)
    assert openamber.get_entity("thermostat_off_delay") == pytest.approx(5.0)

    # Switch to Extern mode -> buttons become active and work
    openamber.set_select("thermostat_mode_select", "Extern")
    openamber.step(ms=100)

    assert not openamber.is_disabled("thermostat_row_off_delay")
    assert not openamber.is_disabled("set_therm_off_delay_dec_btn")
    assert not openamber.is_disabled("set_therm_off_delay_inc_btn")

    openamber.click("set_therm_off_delay_inc_btn")
    openamber.step(ms=100)
    assert openamber.get_entity("thermostat_off_delay") == pytest.approx(10.0)

    openamber.click("set_therm_off_delay_dec_btn")
    openamber.step(ms=100)
    assert openamber.get_entity("thermostat_off_delay") == pytest.approx(5.0)

    openamber.set_number("thermostat_off_delay", 0.0)
    openamber.step(ms=50)
