"""Firmware update from the display while the heat pump runs.

The update dialog warns that the heat pump is active and offers to stop it in a controlled way first. With that
choice the update starts by itself once the system is in standby, or after 15 minutes anyway (the OTA hold keeps
an update during operation safe). When the heat pump is idle the dialog is the plain confirmation.
"""

from helpers import advance_until_state, start_space_heating

MAIN = "state_machine_state_main"


def open_update_dialog(openamber):
    assert openamber.click("service_update_button")
    openamber.step(ms=100)
    assert openamber.is_visible("service_confirm_modal")


def update_started(openamber):
    return openamber.is_visible("service_update_progress_overlay")


def finish(openamber):
    """Leave the system idle for the next test: wait out the update script, resume regulation, drop demand."""
    openamber.advance_time(seconds=610, step_s=10)
    openamber.press_button("controlled_stop_resume_button")
    openamber.set_switch("heat_demand_switch", False)
    openamber.advance_time(seconds=30, step_s=5)


def test_idle_update_dialog_is_plain_confirmation(clean_system):
    openamber = clean_system
    assert advance_until_state(openamber, "internal_pump_active", False, max_s=600, step=5)
    open_update_dialog(openamber)
    assert openamber.get_label("service_confirm_title") == "Update bevestigen"
    assert openamber.is_hidden("service_confirm_stop_first_button")
    assert openamber.click("service_confirm_cancel_button")
    openamber.step(ms=100)
    assert openamber.is_hidden("service_confirm_modal")


def test_update_waits_for_controlled_stop_then_starts(clean_system):
    openamber = clean_system
    start_space_heating(openamber)

    open_update_dialog(openamber)
    assert openamber.get_label("service_confirm_title") == "Warmtepomp actief"
    assert openamber.is_visible("service_confirm_stop_first_button")

    assert openamber.click("service_confirm_stop_first_button")
    openamber.step(ms=100)
    openamber.advance_time(seconds=10, step_s=2)
    openamber.step(ms=50)
    assert int(openamber.get_entity("compressor_control_select") or 0) == 0, "Choosing 'stop first' must start a controlled stop"
    assert openamber.is_visible("service_update_wait_overlay")
    assert not update_started(openamber), "The update must not start before the system is in standby"

    assert advance_until_state(openamber, MAIN, "Standby", max_s=600, step=5)
    openamber.advance_time(seconds=10, step_s=1)
    openamber.step(ms=100)
    assert update_started(openamber), "The update must start once the system is in standby"
    assert openamber.is_hidden("service_update_wait_overlay")

    # A later dialog, with the heat pump idle again, is back to the two-button layout.
    finish(openamber)
    assert advance_until_state(openamber, "internal_pump_active", False, max_s=600, step=5)
    open_update_dialog(openamber)
    assert openamber.is_hidden("service_confirm_stop_first_button")
    assert openamber.click("service_confirm_cancel_button")


def test_update_starts_anyway_when_stop_takes_too_long(clean_system):
    """An oil return cycle that does not end keeps the compressor running; after 15 min the update starts anyway."""
    openamber = clean_system
    start_space_heating(openamber)
    assert openamber.set_sensor("error_register_2119_raw", 8.0)
    openamber.step(ms=50)

    open_update_dialog(openamber)
    assert openamber.click("service_confirm_stop_first_button")
    openamber.advance_time(seconds=14 * 60, step_s=10)
    openamber.step(ms=100)
    assert openamber.get_entity(MAIN) != "Standby", "The stop still waits for the oil return cycle"
    assert not update_started(openamber), "The update must wait up to 15 minutes"

    openamber.advance_time(seconds=70, step_s=5)
    openamber.step(ms=100)
    assert update_started(openamber), "After 15 minutes the update must start anyway"

    assert openamber.set_sensor("error_register_2119_raw", 0.0)
    finish(openamber)


def test_cancel_while_waiting_resumes_without_update(clean_system):
    openamber = clean_system
    start_space_heating(openamber)
    assert openamber.set_sensor("error_register_2119_raw", 8.0)  # keep the stop pending
    openamber.step(ms=50)

    open_update_dialog(openamber)
    assert openamber.click("service_confirm_stop_first_button")
    openamber.advance_time(seconds=30, step_s=5)
    assert openamber.click("service_update_wait_cancel_button")
    openamber.step(ms=100)
    assert openamber.is_hidden("service_update_wait_overlay")

    openamber.advance_time(seconds=20 * 60, step_s=10)
    openamber.step(ms=100)
    assert not update_started(openamber), "A cancelled update must not start later"

    assert openamber.set_sensor("error_register_2119_raw", 0.0)
    finish(openamber)
