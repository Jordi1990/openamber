"""Automated tests for the own outdoor unit fan table (3301-3306 heating, 3323-3328 cooling).
Covers writing the chosen values to the outdoor unit, restoring the factory table when switched off,
and the Ventilator tab under Geavanceerd.
"""


def _reset(openamber):
    openamber.set_switch("outdoor_fan_table_enabled", False)
    openamber.set_number("outdoor_fan_heat_max_rpm", 720)
    openamber.set_number("outdoor_fan_heat_mild_rpm", 620)
    openamber.step(ms=50)


def test_fan_table_written_only_when_enabled(clean_system):
    openamber = clean_system
    _reset(openamber)

    openamber.set_number("outdoor_fan_heat_mild_rpm", 560)
    openamber.advance_time(seconds=65, step_s=5)
    assert openamber.get_entity("outdoor_unit_fan_param_3304") == 620, "Disabled table must not be written"

    openamber.set_switch("outdoor_fan_table_enabled", True)
    openamber.step(ms=50)
    assert openamber.get_entity("outdoor_unit_fan_param_3304") == 560

    # A restarted outdoor unit comes back with its factory value; the next minute writes it again.
    openamber.set_number("outdoor_unit_fan_param_3304", 620)
    openamber.advance_time(seconds=65, step_s=5)
    assert openamber.get_entity("outdoor_unit_fan_param_3304") == 560

    openamber.set_switch("outdoor_fan_table_enabled", False)
    openamber.step(ms=50)
    assert openamber.get_entity("outdoor_unit_fan_param_3304") == 620, "Switching off restores the factory table"
    _reset(openamber)


def test_fan_tab_pages(clean_system):
    openamber = clean_system
    _reset(openamber)
    assert openamber.click("nav_settings")
    openamber.step(ms=100)
    openamber.set_switch("advanced_settings_enabled", True)
    openamber.step(ms=100)
    assert openamber.click("stab_advanced")
    openamber.step(ms=50)
    assert openamber.click("advanced_stab_fan")
    openamber.step(ms=100)

    assert openamber.is_hidden("panel_advanced_page_fan") is False
    assert openamber.is_hidden("set_fan_enable_row") is False
    assert openamber.is_hidden("set_fan_heat_max_row") is True
    assert openamber.get_label("advanced_fan_page_indicator") == "Pagina 1 / 5"

    openamber.click("advanced_fan_page_next_button")
    openamber.step(ms=100)
    assert openamber.get_label("advanced_fan_page_indicator") == "Pagina 2 / 5"
    assert openamber.is_hidden("set_fan_heat_max_row") is False
    assert openamber.get_label("set_fan_heat_max_val") == "720 rpm"

    openamber.set_number("outdoor_fan_heat_max_rpm", 700)
    openamber.step(ms=50)
    assert openamber.get_label("set_fan_heat_max_val") == "700 rpm"

    openamber.set_switch("advanced_settings_enabled", False)
    openamber.step(ms=50)
    _reset(openamber)
