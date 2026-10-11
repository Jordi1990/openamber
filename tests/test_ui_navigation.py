import pytest


def test_header_title_is_openamber(openamber):
    """Verify that the top header bar displays OpenAmber title."""
    header = openamber.get_widget("header_title")
    assert header.get("status") == "ok", "header_title widget lookup should succeed"
    assert "OpenAmber" in header.get("text", ""), (
        f"header_title should contain 'OpenAmber', got: {header.get('text', '')}"
    )


def test_initial_page_home_is_visible(openamber):
    """Verify that on boot, the Home page is visible and Settings is hidden."""
    assert openamber.click("nav_home"), "Clicking nav_home must succeed"
    openamber.step(ms=100)

    assert openamber.is_visible("page_home"), "page_home should be visible"
    assert openamber.is_hidden("page_settings"), "page_settings should be hidden"
    assert openamber.is_hidden("page_service"), "page_service should be hidden"
    assert openamber.is_hidden("page_system"), "page_system should be hidden"


def test_navigate_to_settings(openamber):
    """Verify clicking Settings in the navbar opens the Settings page and hides others."""
    assert openamber.click("nav_settings"), "Clicking nav_settings must succeed"
    openamber.step(ms=100)

    assert openamber.is_visible("page_settings"), "page_settings should be visible"
    assert openamber.is_hidden("page_home"), "page_home should be hidden"
    assert openamber.is_hidden("page_service"), "page_service should be hidden"
    assert openamber.is_hidden("page_system"), "page_system should be hidden"


def test_navigate_to_service(openamber):
    """Verify clicking Service in the navbar opens the Service page and hides others."""
    assert openamber.click("nav_service"), "Clicking nav_service must succeed"
    openamber.step(ms=100)

    assert openamber.is_visible("page_service"), "page_service should be visible"
    assert openamber.is_hidden("page_settings"), "page_settings should be hidden"
    assert openamber.is_hidden("page_home"), "page_home should be hidden"
    assert openamber.is_hidden("page_system"), "page_system should be hidden"


def test_navigate_to_system(openamber):
    """Verify clicking System in the navbar opens the System page and hides others."""
    assert openamber.click("nav_system"), "Clicking nav_system must succeed"
    openamber.step(ms=100)

    assert openamber.is_visible("page_system"), "page_system should be visible"
    assert openamber.is_hidden("page_service"), "page_service should be hidden"
    assert openamber.is_hidden("page_settings"), "page_settings should be hidden"
    assert openamber.is_hidden("page_home"), "page_home should be hidden"


def test_navigate_back_to_home(openamber):
    """Verify clicking Home in the navbar returns to the Home page and hides system."""
    assert openamber.click("nav_home"), "Clicking nav_home must succeed"
    openamber.step(ms=100)

    assert openamber.is_visible("page_home"), "page_home should be visible"
    assert openamber.is_hidden("page_system"), "page_system should be hidden"
    assert openamber.is_hidden("page_service"), "page_service should be hidden"
    assert openamber.is_hidden("page_settings"), "page_settings should be hidden"
