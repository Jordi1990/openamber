"""Firmware update from the display, depending on whether the heat pump is in standby.

Outside standby the heat pump is in operation, also while the compressor and pump happen to be idle. The update
dialog then offers to update right away (safe during operation) or to stop in a controlled way first. Stopping
first only starts the controlled stop: the user waits for standby and starts the update again. In standby the
dialog is the plain confirmation.
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


def test_dialog_offers_stop_first_outside_standby_even_when_idle(clean_system):
    openamber = clean_system
    assert advance_until_state(openamber, "internal_pump_active", False, max_s=600, step=5)
    assert openamber.get_entity(MAIN) != "Standby"
    open_update_dialog(openamber)
    assert openamber.get_label("service_confirm_title") == "Warmtepomp in bedrijf"
    assert openamber.is_visible("service_confirm_stop_first_button")
    assert openamber.click("service_confirm_cancel_button")
    openamber.step(ms=100)
    assert openamber.is_hidden("service_confirm_modal")
    assert openamber.is_hidden("service_confirm_stop_first_button")


def test_stop_first_only_stops_then_update_by_hand_in_standby(clean_system):
    openamber = clean_system
    start_space_heating(openamber)

    open_update_dialog(openamber)
    assert openamber.get_label("service_confirm_title") == "Warmtepomp in bedrijf"
    assert openamber.click("service_confirm_stop_first_button")
    openamber.step(ms=100)
    assert openamber.is_hidden("service_confirm_modal")
    openamber.advance_time(seconds=10, step_s=2)
    assert int(openamber.get_entity("compressor_control_select") or 0) == 0, "'Eerst stoppen' must start a controlled stop"

    assert advance_until_state(openamber, MAIN, "Standby", max_s=600, step=5)
    openamber.advance_time(seconds=60, step_s=5)
    openamber.step(ms=100)
    assert not update_started(openamber), "The update must not start by itself after the stop"

    # In standby the dialog is the plain confirmation, and confirming starts the update.
    open_update_dialog(openamber)
    assert openamber.get_label("service_confirm_title") == "Update bevestigen"
    assert openamber.is_hidden("service_confirm_stop_first_button")
    assert openamber.click("service_confirm_button")
    openamber.step(ms=100)
    assert update_started(openamber)
    finish(openamber)


def test_update_now_during_operation_starts_right_away(clean_system):
    openamber = clean_system
    start_space_heating(openamber)

    open_update_dialog(openamber)
    assert openamber.get_label("service_confirm_button_label") == "Nu updaten"
    assert openamber.click("service_confirm_button")
    openamber.step(ms=100)
    assert update_started(openamber)
    assert openamber.get_entity(MAIN) != "Standby", "A live update does not stop the heat pump"
    finish(openamber)
