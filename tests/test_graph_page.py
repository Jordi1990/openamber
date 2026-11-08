"""Automated tests for the history graph page in the navbar."""

OTHER_PAGES = ["page_home", "page_thermostat", "page_service", "page_system", "page_settings"]


def test_navigate_to_graph_page_and_back(openamber):
    """The Grafiek navbar button opens the graph page; another button hides it again."""
    assert openamber.click("nav_graph"), "Clicking nav_graph must succeed"
    openamber.step(ms=100)
    assert openamber.is_visible("page_graph"), "page_graph should be visible"
    for page in OTHER_PAGES:
        assert openamber.is_hidden(page), f"{page} should be hidden on the graph page"

    assert openamber.click("nav_home"), "Clicking nav_home must succeed"
    openamber.step(ms=100)
    assert openamber.is_visible("page_home"), "page_home should be visible"
    assert openamber.is_hidden("page_graph"), "page_graph should be hidden after leaving it"


def test_graph_page_shows_latest_sample(clean_system):
    """A sample is taken every minute; the page shows the newest Tc and hides the empty hint."""
    openamber = clean_system
    openamber.set_sensor("heat_cool_temperature_tc", 31.4)
    assert openamber.click("nav_graph"), "Clicking nav_graph must succeed"
    openamber.advance_time(seconds=65, step_s=5)

    assert openamber.is_hidden("graph_empty_label"), "The empty hint must hide once a sample exists"
    assert "31.4" in openamber.get_widget("graph_tc_value").get("text", "")

    # The Tuo and Tui values only show when the pipes toggle is on.
    assert openamber.is_hidden("graph_tuo_value")
    assert openamber.click("graph_btn_pipes"), "Clicking graph_btn_pipes must succeed"
    openamber.step(ms=100)
    assert openamber.is_visible("graph_tuo_value"), "Tuo must show after enabling the pipes toggle"
    assert openamber.click("graph_btn_pipes"), "Clicking graph_btn_pipes must succeed"
    openamber.step(ms=100)

    assert openamber.click("nav_home"), "Clicking nav_home must succeed"
    openamber.step(ms=100)
