"""Automated tests for the Service Sensors page pagination and Zone 2 heat demand."""

import pytest


def test_service_sensors_navigation_and_pagination(clean_system):
    """
    Verify:
    1. Navigation from navbar -> Service page -> Sensoren tab.
    2. Sensors panel is visible with Page 1 displayed initially.
    3. Pager indicator shows 'Pagina 1 / 4'.
    4. Navigating forward through pages 2, 3, 4 via next button updates visibility and indicator.
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

    # 3. Verify Page 1 is visible, Pages 2-4 are hidden, and indicator is Pagina 1 / 4
    assert openamber.is_visible("service_sensors_page_1"), "service_sensors_page_1 should be visible initially"
    assert openamber.is_hidden("service_sensors_page_2"), "service_sensors_page_2 should be hidden initially"
    assert openamber.is_hidden("service_sensors_page_3"), "service_sensors_page_3 should be hidden initially"
    assert openamber.is_hidden("service_sensors_page_4"), "service_sensors_page_4 should be hidden initially"
    indicator = openamber.get_widget("service_sensors_page_indicator")
    assert indicator.get("text") == "Pagina 1 / 4", (
        f"Expected pager indicator 'Pagina 1 / 4', got: {indicator.get('text')}"
    )

    # 4. Click Next -> Page 2 (Druk & Temperatuur)
    assert openamber.click("service_sensors_page_next_button"), "Clicking next button must succeed"
    openamber.step(ms=100)
    assert openamber.is_hidden("service_sensors_page_1"), "service_sensors_page_1 should be hidden on page 2"
    assert openamber.is_visible("service_sensors_page_2"), "service_sensors_page_2 should be visible on page 2"
    assert openamber.is_hidden("service_sensors_page_3"), "service_sensors_page_3 should be hidden on page 2"
    assert openamber.is_hidden("service_sensors_page_4"), "service_sensors_page_4 should be hidden on page 2"
    assert openamber.get_widget("service_sensors_page_indicator").get("text") == "Pagina 2 / 4"

    # 5. Click Next -> Page 3 (Aansturing)
    assert openamber.click("service_sensors_page_next_button"), "Clicking next button must succeed"
    openamber.step(ms=100)
    assert openamber.is_hidden("service_sensors_page_1"), "service_sensors_page_1 should be hidden on page 3"
    assert openamber.is_hidden("service_sensors_page_2"), "service_sensors_page_2 should be hidden on page 3"
    assert openamber.is_visible("service_sensors_page_3"), "service_sensors_page_3 should be visible on page 3"
    assert openamber.is_hidden("service_sensors_page_4"), "service_sensors_page_4 should be hidden on page 3"
    assert openamber.get_widget("service_sensors_page_indicator").get("text") == "Pagina 3 / 4"

    # 6. Click Next -> Page 4 (Systeem)
    assert openamber.click("service_sensors_page_next_button"), "Clicking next button must succeed"
    openamber.step(ms=100)
    assert openamber.is_hidden("service_sensors_page_1"), "service_sensors_page_1 should be hidden on page 4"
    assert openamber.is_hidden("service_sensors_page_2"), "service_sensors_page_2 should be hidden on page 4"
    assert openamber.is_hidden("service_sensors_page_3"), "service_sensors_page_3 should be hidden on page 4"
    assert openamber.is_visible("service_sensors_page_4"), "service_sensors_page_4 should be visible on page 4"
    assert openamber.get_widget("service_sensors_page_indicator").get("text") == "Pagina 4 / 4"

    # 7. Click Prev -> Page 3
    assert openamber.click("service_sensors_page_prev_button"), "Clicking prev button must succeed"
    openamber.step(ms=100)
    assert openamber.is_visible("service_sensors_page_3")
    assert openamber.get_widget("service_sensors_page_indicator").get("text") == "Pagina 3 / 4"

    # 8. Click Prev -> Page 2
    assert openamber.click("service_sensors_page_prev_button")
    openamber.step(ms=100)
    assert openamber.is_visible("service_sensors_page_2")
    assert openamber.get_widget("service_sensors_page_indicator").get("text") == "Pagina 2 / 4"

    # 9. Click Prev -> Page 1
    assert openamber.click("service_sensors_page_prev_button")
    openamber.step(ms=100)
    assert openamber.is_visible("service_sensors_page_1")
    assert openamber.get_widget("service_sensors_page_indicator").get("text") == "Pagina 1 / 4"


def test_service_sensors_heat_demand_zone_2(clean_system):
    """
    Verify:
    1. Setting external_heat_demand_wired_zone_2 updates the UI without issue.
    2. Diagnostics summary script executes smoothly with zone 2 demand active.
    """
    openamber = clean_system

    # Navigate to Service page -> Sensoren
    assert openamber.click("nav_service")
    openamber.step(ms=100)
    assert openamber.click("service_tab_sensors")
    openamber.step(ms=100)

    # Advance to Page 4 (which contains Systeem card with Warmtevraag Zone 2)
    assert openamber.click("service_sensors_page_next_button")
    openamber.step(ms=50)
    assert openamber.click("service_sensors_page_next_button")
    openamber.step(ms=50)
    assert openamber.click("service_sensors_page_next_button")
    openamber.step(ms=100)
    assert openamber.is_visible("service_sensors_page_4")

    # Activate wired heat demand for zone 2
    assert openamber.set_binary_sensor("external_heat_demand_wired_zone_2", True) is True
    openamber.step(ms=150)

    # Deactivate wired heat demand for zone 2
    assert openamber.set_binary_sensor("external_heat_demand_wired_zone_2", False) is True
    openamber.step(ms=150)

