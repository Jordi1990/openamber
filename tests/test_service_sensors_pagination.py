"""Automated tests for Service Sensors pagination, Demand & External signal rows,
DHW dropdown UI bindings, and Thermostat off-delay UI behavior.
"""

import pytest


def test_service_sensors_navigation_and_pagination(clean_system):
    """
    Verify:
    1. Navigation from navbar -> Service page -> Sensoren tab.
    2. Sensors panel is visible with Page 1 displayed initially.
    3. Pager indicator shows 'Pagina 1 / 5'.
    4. Navigating forward through pages 2, 3, 4, 5 via next button updates visibility and indicator.
    5. Navigating backward through prev button updates visibility and indicator back to Page 1.
    """
    openamber = clean_system

    # 1. Navigate to Service page
    assert openamber.click("nav_service"), "Clicking nav_service must succeed"
    openamber.step(ms=100)
    assert openamber.is_visible("page_service"), "page_service should be visible"

    # 2. Click Sensoren tab in the sidebar
    assert openamber.click("service_tab_sensors"), "Clicking service_tab_sensors must succeed"
    openamber.step(ms=100)
    assert openamber.is_visible("service_sensors_panel"), "service_sensors_panel should be visible"

    # 3. Verify Page 1 is visible, Pages 2-5 are hidden, and indicator is Pagina 1 / 5
    assert openamber.is_visible("service_sensors_page_1"), "service_sensors_page_1 should be visible initially"
    assert openamber.is_hidden("service_sensors_page_2"), "service_sensors_page_2 should be hidden initially"
    assert openamber.is_hidden("service_sensors_page_3"), "service_sensors_page_3 should be hidden initially"
    assert openamber.is_hidden("service_sensors_page_4"), "service_sensors_page_4 should be hidden initially"
    assert openamber.is_hidden("service_sensors_page_5"), "service_sensors_page_5 should be hidden initially"
    indicator = openamber.get_widget("service_sensors_page_indicator")
    assert indicator.get("text") == "Pagina 1 / 5", (
        f"Expected pager indicator 'Pagina 1 / 5', got: {indicator.get('text')}"
    )

    # 4. Click Next -> Page 2 (Druk & Temperatuur)
    assert openamber.click("service_sensors_page_next_button"), "Clicking next button must succeed"
    openamber.step(ms=100)
    assert openamber.is_hidden("service_sensors_page_1")
    assert openamber.is_visible("service_sensors_page_2")
    assert openamber.is_hidden("service_sensors_page_3")
    assert openamber.is_hidden("service_sensors_page_4")
    assert openamber.is_hidden("service_sensors_page_5")
    assert openamber.get_widget("service_sensors_page_indicator").get("text") == "Pagina 2 / 5"

    # 5. Click Next -> Page 3 (Aansturing)
    assert openamber.click("service_sensors_page_next_button"), "Clicking next button must succeed"
    openamber.step(ms=100)
    assert openamber.is_hidden("service_sensors_page_1")
    assert openamber.is_hidden("service_sensors_page_2")
    assert openamber.is_visible("service_sensors_page_3")
    assert openamber.is_hidden("service_sensors_page_4")
    assert openamber.is_hidden("service_sensors_page_5")
    assert openamber.get_widget("service_sensors_page_indicator").get("text") == "Pagina 3 / 5"

    # 6. Click Next -> Page 4 (Systeem)
    assert openamber.click("service_sensors_page_next_button"), "Clicking next button must succeed"
    openamber.step(ms=100)
    assert openamber.is_hidden("service_sensors_page_1")
    assert openamber.is_hidden("service_sensors_page_2")
    assert openamber.is_hidden("service_sensors_page_3")
    assert openamber.is_visible("service_sensors_page_4")
    assert openamber.is_hidden("service_sensors_page_5")
    assert openamber.get_widget("service_sensors_page_indicator").get("text") == "Pagina 4 / 5"

    # 7. Click Next -> Page 5 (Vraag & Ingangen)
    assert openamber.click("service_sensors_page_next_button"), "Clicking next button must succeed"
    openamber.step(ms=100)
    assert openamber.is_hidden("service_sensors_page_1")
    assert openamber.is_hidden("service_sensors_page_2")
    assert openamber.is_hidden("service_sensors_page_3")
    assert openamber.is_hidden("service_sensors_page_4")
    assert openamber.is_visible("service_sensors_page_5")
    assert openamber.get_widget("service_sensors_page_indicator").get("text") == "Pagina 5 / 5"

    # 8. Click Prev -> Page 4
    assert openamber.click("service_sensors_page_prev_button"), "Clicking prev button must succeed"
    openamber.step(ms=100)
    assert openamber.is_visible("service_sensors_page_4")
    assert openamber.get_widget("service_sensors_page_indicator").get("text") == "Pagina 4 / 5"

    # 9. Click Prev -> Page 3
    assert openamber.click("service_sensors_page_prev_button"), "Clicking prev button must succeed"
    openamber.step(ms=100)
    assert openamber.is_visible("service_sensors_page_3")
    assert openamber.get_widget("service_sensors_page_indicator").get("text") == "Pagina 3 / 5"

    # 10. Click Prev -> Page 2
    assert openamber.click("service_sensors_page_prev_button")
    openamber.step(ms=100)
    assert openamber.is_visible("service_sensors_page_2")
    assert openamber.get_widget("service_sensors_page_indicator").get("text") == "Pagina 2 / 5"

    # 11. Click Prev -> Page 1
    assert openamber.click("service_sensors_page_prev_button")
    openamber.step(ms=100)
    assert openamber.is_visible("service_sensors_page_1")
    assert openamber.get_widget("service_sensors_page_indicator").get("text") == "Pagina 1 / 5"


def test_service_sensors_demand_and_external_signals_page_5(clean_system):
    """
    Verify Page 5 ('Vraag & Ingangen') shows both effective demand and raw external signals separately:
    1. Software/internal Zone 1 heat demand sets 'Warmtevraag Zone 1' to 'AAN' while
       'Warmtevraag Zone 1 (Extern signaal)' remains 'UIT'.
    2. Wired Zone 2 input when mixing valves are disabled sets 'Warmtevraag Zone 2 (Extern signaal)'
       to 'AAN' while effective 'Warmtevraag Zone 2' remains 'UIT'; enabling mixing valve sets both to 'AAN'.
    3. Wired cooling input while heating demand is active sets 'Koudevraag (Extern signaal)' to 'AAN'
       while effective 'Koudevraag' remains 'UIT'.
    """
    openamber = clean_system

    # Navigate to Service -> Sensoren -> Page 5
    assert openamber.click("nav_service")
    openamber.step(ms=100)
    assert openamber.click("service_tab_sensors")
    openamber.step(ms=100)
    for _ in range(4):
        assert openamber.click("service_sensors_page_next_button")
        openamber.step(ms=50)
    assert openamber.is_visible("service_sensors_page_5")

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

    # Navigate to Settings -> Thermostaat tab
    assert openamber.click("nav_settings")
    openamber.step(ms=100)
    assert openamber.click("stab_thermostat")
    openamber.step(ms=100)

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
