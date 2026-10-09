import pytest


def test_settings_sidebar_tabs(openamber):
    """Verify clicking sidebar tabs in the Settings page."""
    # Ensure on Settings page
    openamber.click("nav_settings")
    openamber.step(ms=100)
    assert openamber.is_visible("page_settings")

    # Click Algemeen tab
    openamber.click("stab_general")
    openamber.step(ms=50)
    assert openamber.get_label("stab_general_lbl") == "Algemeen"

    # Click Verwarmen tab
    openamber.click("stab_heating")
    openamber.step(ms=50)
    assert openamber.get_label("stab_heating_lbl") == "Verwarmen"

    # Click Tapwater tab
    openamber.click("stab_dhw")
    openamber.step(ms=50)
    assert openamber.get_label("stab_dhw_lbl") == "Tapwater"

    # Click Bijverwarmen tab
    openamber.click("stab_bijverwarmen")
    openamber.step(ms=50)
    assert openamber.get_label("stab_bijverwarmen_lbl") == "Bijverwarmen"


def test_mengventielen_pager_workflow(openamber):
    """Verify mixing valves pager page navigation (Pagina 1 / 2 <-> Pagina 2 / 2)."""
    openamber.click("nav_settings")
    openamber.step(ms=100)

    # Click next page button
    openamber.click("mengventielen_page_2_button")
    openamber.step(ms=100)
    indicator = openamber.get_label("mengventielen_page_indicator")
    assert "2 / 2" in indicator

    # Click previous page button
    openamber.click("mengventielen_page_1_button")
    openamber.step(ms=100)
    indicator = openamber.get_label("mengventielen_page_indicator")
    assert "1 / 2" in indicator
