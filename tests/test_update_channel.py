"""Update channels (Stabiel / Beta) and what the Updates card on the Systeem page offers.

The simulator's update entity serves one version per channel (mock_firmware_versions picks the installed version
and what the stable and beta manifests offer). Like ESPHome, any different version is an update: the beta
manifest always holds the newest release, beta or stable. On the Stabiel channel with a beta installed the card
offers the stable version instead ("Stabiel installeren").
"""

import pytest


@pytest.fixture
def channels(clean_system):
    openamber = clean_system
    yield openamber
    openamber.set_select("firmware_update_channel", "Stabiel")
    openamber.set_select("mock_firmware_versions", "Geen update")
    openamber.step(ms=100)


def show_updates(openamber, channel, scenario):
    openamber.set_select("firmware_update_channel", channel)
    openamber.set_select("mock_firmware_versions", scenario)
    openamber.step(ms=100)
    assert openamber.click("nav_system")
    openamber.step(ms=100)


def test_no_update_when_versions_match(channels):
    openamber = channels
    show_updates(openamber, "Stabiel", "Geen update")
    assert openamber.get_label("service_update_version").startswith("Geen nieuwe software beschikbaar")
    assert openamber.is_hidden("service_update_button")
    assert openamber.is_hidden("nav_system_badge")


def test_new_beta_only_on_beta_channel(channels):
    openamber = channels
    show_updates(openamber, "Stabiel", "Nieuwe beta")
    assert openamber.get_label("service_update_version").startswith("Geen nieuwe software beschikbaar")

    show_updates(openamber, "Beta", "Nieuwe beta")
    assert openamber.get_label("service_update_version") == "Nieuwe versie 1.1.0-beta.1 beschikbaar\nHuidige versie: 1.0.0"
    assert openamber.get_label("service_update_button_label") == "Update installeren"
    assert openamber.is_visible("nav_system_badge")


def test_final_release_is_an_update_for_beta(channels):
    openamber = channels
    show_updates(openamber, "Beta", "Beta, finale uit")
    assert openamber.get_label("service_update_version") == "Nieuwe versie 1.1.0 beschikbaar\nHuidige versie: 1.1.0-beta.1"
    assert openamber.get_label("service_update_button_label") == "Update installeren"
    assert openamber.is_visible("service_update_button")
    assert openamber.is_visible("nav_system_badge")

    # On the Stabiel channel the final release is offered as the stable version.
    show_updates(openamber, "Stabiel", "Beta, finale uit")
    assert openamber.get_label("service_update_version") == "Stabiele versie 1.1.0 beschikbaar\nHuidige versie: 1.1.0-beta.1 (beta)"
    assert openamber.get_label("service_update_button_label") == "Stabiel installeren"
    assert openamber.is_visible("nav_system_badge")


def test_older_stable_is_not_an_update_on_beta_channel(channels):
    openamber = channels
    show_updates(openamber, "Beta", "Beta, stabiel ouder")
    assert openamber.get_label("service_update_version").startswith("Geen nieuwe software beschikbaar")
    assert openamber.is_hidden("service_update_button")
    assert openamber.is_hidden("nav_system_badge")


def test_back_to_stable_on_stable_channel(channels):
    openamber = channels
    show_updates(openamber, "Stabiel", "Beta, stabiel ouder")
    assert openamber.get_label("service_update_version") == "Stabiele versie 1.0.0 beschikbaar\nHuidige versie: 1.1.0-beta.1 (beta)"
    assert openamber.get_label("service_update_button_label") == "Stabiel installeren"
    assert openamber.is_visible("service_update_button")

    assert openamber.click("service_update_button")
    openamber.step(ms=100)
    assert openamber.is_visible("service_confirm_modal")
    if openamber.get_label("service_confirm_title") != "Warmtepomp in bedrijf":
        assert openamber.get_label("service_confirm_title") == "Stabiele versie installeren"
        assert openamber.get_label("service_confirm_button_label") == "Installeren"
    assert openamber.click("service_confirm_cancel_button")
    openamber.step(ms=100)
    assert openamber.is_hidden("service_confirm_modal")


def test_channel_setting_on_display(channels):
    openamber = channels
    show_updates(openamber, "Stabiel", "Nieuwe beta")

    assert openamber.click("nav_system")
    openamber.step(ms=100)
    assert openamber.is_visible("service_update_channel_row")

    assert openamber.set_dropdown("service_update_channel_dd", 1)
    openamber.step(ms=100)
    assert openamber.get_entity("firmware_update_channel") == "Beta"

    # Switching the channel checks the beta manifest right away.
    assert openamber.get_label("service_update_version") == "Nieuwe versie 1.1.0-beta.1 beschikbaar\nHuidige versie: 1.0.0"

    # And the dropdown follows the entity (e.g. a change from Home Assistant).
    openamber.set_select("firmware_update_channel", "Stabiel")
    openamber.step(ms=100)
    assert openamber.get_widget("service_update_channel_dd").get("selected_index") == 0


@pytest.mark.parametrize("scenario", ["Geen update", "Beta, finale uit"])
def test_channel_row_fits_without_scrolling(channels, scenario):
    # With an update available the card also shows the release QR code and is taller; the channel row below the
    # tiles must still fit in the panel without scrolling.
    openamber = channels
    show_updates(openamber, "Stabiel", scenario)
    if scenario != "Geen update":
        assert openamber.is_visible("service_release_qr_hint")
    panel = openamber.get_widget("service_updates_panel")
    tiles = openamber.get_widget("service_action_row")
    row = openamber.get_widget("service_update_channel_row")
    assert tiles["y2"] < row["y1"]
    assert row["y2"] <= panel["y2"], f"channel row ends at {row['y2']}, panel at {panel['y2']}"
