"""Automated tests for the history graph page in the navbar."""

import pytest

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


def test_graph_redraw_with_full_day_of_compressor_history_is_fast(clean_system):
    """A redraw of 24 h with the compressor running all day stays within the redraw limit.

    The page logs its redraw time; in the simulator a mock sensor (graph_redraw_time) exposes it. The simulator is
    much faster than the ESP32, so this only catches large regressions; the real timing is checked on the device.
    """
    openamber = clean_system
    openamber.set_number("mock_compressor_frequency_override", 60)
    openamber.set_sensor("heat_cool_temperature_tc", 30.0)
    # One sample per minute: fill the full 24 h history.
    openamber.advance_time(seconds=24 * 3600 + 60, step_s=60)

    assert openamber.click("nav_graph"), "Clicking nav_graph must succeed"
    openamber.step(ms=100)
    assert openamber.click("graph_btn_24h"), "Clicking graph_btn_24h must succeed"
    openamber.step(ms=100)
    openamber.advance_time(seconds=2, step_s=1)  # the mock sensor publishes once per second

    redraw_ms = openamber.get_entity("graph_redraw_time")
    assert redraw_ms is not None, "The graph page must report its redraw time"
    assert float(redraw_ms) <= 100, f"Redraw of a full day took {redraw_ms} ms"

    openamber.set_number("mock_compressor_frequency_override", -1)
    assert openamber.click("nav_home"), "Clicking nav_home must succeed"
    openamber.step(ms=100)


@pytest.mark.parametrize("alert_button,tab_script_panel", [
    ("nav_status_error_btn", "page_service"),
    ("nav_status_warning_btn", "page_service"),
])
def test_header_alert_hides_graph_page(openamber, alert_button, tab_script_panel):
    """Opening the error or warning list from the header while the graph is open shows the service page on top:
    the graph page is hidden, so it cannot cover the list."""
    assert openamber.click("nav_graph"), "Clicking nav_graph must succeed"
    openamber.step(ms=100)
    assert openamber.is_visible("page_graph")

    assert openamber.click(alert_button), f"Clicking {alert_button} must succeed"
    openamber.step(ms=100)
    assert openamber.is_hidden("page_graph"), "The graph page must be hidden when an alert list opens"
    assert openamber.is_visible(tab_script_panel)

    assert openamber.click("nav_home"), "Clicking nav_home must succeed"
    openamber.step(ms=100)
