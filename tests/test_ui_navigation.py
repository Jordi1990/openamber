import pytest


def test_header_title_is_openamber(openamber):
    """Verify that the top header bar displays OpenAmber title."""
    header = openamber.get_widget("header_title")
    assert header.get("status") == "ok"
    assert "OpenAmber" in header.get("text", "")


def test_initial_page_home_is_visible(openamber):
    """Verify that on boot, the Home page is visible and Settings is hidden."""
    openamber.click("nav_home")
    openamber.step(ms=100)

    assert openamber.is_visible("page_home")
    assert openamber.is_hidden("page_settings")
    assert openamber.is_hidden("page_service")
    assert openamber.is_hidden("page_system")


def test_navigate_to_settings(openamber):
    """Verify clicking Settings in the navbar opens the Settings page."""
    openamber.click("nav_settings")
    openamber.step(ms=100)

    assert openamber.is_visible("page_settings")
    assert openamber.is_hidden("page_home")
    assert openamber.is_hidden("page_service")
    assert openamber.is_hidden("page_system")


def test_navigate_to_service(openamber):
    """Verify clicking Service in the navbar opens the Service page."""
    openamber.click("nav_service")
    openamber.step(ms=100)

    assert openamber.is_visible("page_service")
    assert openamber.is_hidden("page_settings")
    assert openamber.is_hidden("page_home")


def test_navigate_to_system(openamber):
    """Verify clicking System in the navbar opens the System page."""
    openamber.click("nav_system")
    openamber.step(ms=100)

    assert openamber.is_visible("page_system")
    assert openamber.is_hidden("page_service")
    assert openamber.is_hidden("page_settings")


def test_navigate_back_to_home(openamber):
    """Verify clicking Home in the navbar returns to the Home page."""
    openamber.click("nav_home")
    openamber.step(ms=100)

    assert openamber.is_visible("page_home")
    assert openamber.is_hidden("page_system")
