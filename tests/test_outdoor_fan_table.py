"""Automated tests for the outdoor unit fan table (3301-3306 heating, 3323-3328 cooling) on the
Ventilator tab under Geavanceerd. The values are modbus numbers written straight to the outdoor unit.
"""


def test_fan_tab_pages_and_values(clean_system):
    openamber = clean_system
    openamber.set_number("outdoor_unit_fan_param_3306", 720)
    openamber.step(ms=50)

    assert openamber.click("nav_settings")
    openamber.step(ms=100)
    openamber.set_switch("advanced_settings_enabled", True)
    openamber.step(ms=100)
    assert openamber.click("stab_advanced")
    openamber.step(ms=50)
    assert openamber.click("advanced_stab_fan")
    openamber.step(ms=100)

    assert openamber.is_hidden("panel_advanced_page_fan") is False
    assert openamber.get_label("advanced_fan_page_indicator") == "Pagina 1 / 4"
    assert openamber.is_hidden("set_fan_heat_max_row") is False
    assert openamber.is_hidden("set_fan_heat_mild_row") is False
    assert openamber.is_hidden("set_fan_heat_warm_row") is True

    openamber.set_number("outdoor_unit_fan_param_3306", 700)
    openamber.step(ms=50)
    assert openamber.get_label("set_fan_heat_max_val") == "700 rpm"

    openamber.click("advanced_fan_page_next_button")
    openamber.step(ms=100)
    assert openamber.get_label("advanced_fan_page_indicator") == "Pagina 2 / 4"
    assert openamber.is_hidden("set_fan_heat_max_row") is True
    assert openamber.is_hidden("set_fan_heat_warm_row") is False

    for _ in range(3):
        openamber.click("advanced_fan_page_next_button")
        openamber.step(ms=50)
    assert openamber.get_label("advanced_fan_page_indicator") == "Pagina 4 / 4"
    assert openamber.is_hidden("set_fan_cool_hz_mid_row") is False

    openamber.set_number("outdoor_unit_fan_param_3306", 720)
    openamber.set_switch("advanced_settings_enabled", False)
    openamber.step(ms=50)
