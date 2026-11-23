"""Runtime display language: the select "Display taal" switches the texts without a reboot.

Covers:
1. Static widget texts (nav bar labels) follow the selected language, in both directions.
2. The settings dropdown shows the selected language.
"""


def _set_language(openamber, option):
    assert openamber.set_select("display_language_select", option)
    openamber.advance_time(ms=1000)


def test_runtime_language_switch(clean_system):
    openamber = clean_system
    try:
        _set_language(openamber, "Deutsch")
        assert openamber.get_label("nav_home_label") == "Start"
        assert openamber.get_label("nav_graph_label") == "Grafik"

        _set_language(openamber, "English")
        assert openamber.get_label("nav_home_label") == "Home"
        assert openamber.get_label("nav_graph_label") == "Graph"
        assert openamber.get_widget("set_display_language_dd").get("selected_index") == 2

        _set_language(openamber, "Nederlands")
        assert openamber.get_label("nav_graph_label") == "Grafiek"
        assert openamber.get_label("nav_thermostat_label") == "Thermostaat"
    finally:
        _set_language(openamber, "Nederlands")


def test_breadcrumb_follows_language_switch(clean_system):
    """The menu breadcrumb keeps the parent title: it must switch language with the rest."""
    openamber = clean_system
    try:
        _set_language(openamber, "English")
        assert openamber.click("nav_settings")
        openamber.step(ms=100)
        assert openamber.click("stab_general")
        openamber.step(ms=100)
        assert openamber.click("general_stab_systeem")
        openamber.step(ms=100)
        assert openamber.get_label("settings_menu_title") == "System"
        assert openamber.get_label("settings_menu_crumb").startswith("General ")

        _set_language(openamber, "Nederlands")
        assert openamber.get_label("settings_menu_title") == "Systeem"
        assert openamber.get_label("settings_menu_crumb").startswith("Algemeen "), (
            "Breadcrumb must read Algemeen > Systeem after switching back to Nederlands"
        )

        # Navigating again keeps the Dutch parent title.
        assert openamber.click("settings_menu_back_btn")
        openamber.step(ms=100)
        assert openamber.click("general_stab_systeem")
        openamber.step(ms=100)
        assert openamber.get_label("settings_menu_crumb").startswith("Algemeen ")
    finally:
        _set_language(openamber, "Nederlands")
