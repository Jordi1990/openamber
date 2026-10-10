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


def test_mengventielen_pager_workflow(openamber):
    """Verify mixing valves pager page navigation (Pagina 1 / 2 <-> Pagina 2 / 2)."""
    assert openamber.click("nav_settings"), "Clicking nav_settings must succeed"
    openamber.step(ms=100)
    assert openamber.is_visible("page_settings"), "page_settings must be visible"

    # Click next page button
    assert openamber.click("mengventielen_page_2_button"), "Clicking page 2 button must succeed"
    openamber.step(ms=100)
    indicator = openamber.get_label("mengventielen_page_indicator")
    assert "2 / 2" in indicator, f"Page indicator should show '2 / 2', got: '{indicator}'"

    # Click previous page button
    assert openamber.click("mengventielen_page_1_button"), "Clicking page 1 button must succeed"
    openamber.step(ms=100)
    indicator = openamber.get_label("mengventielen_page_indicator")
    assert "1 / 2" in indicator, f"Page indicator should show '1 / 2', got: '{indicator}'"

