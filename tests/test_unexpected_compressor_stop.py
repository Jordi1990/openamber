"""Automated tests for a compressor that the outdoor unit stops by itself.

The outdoor unit can stop the compressor on its own, for example after a protection trip, a power dip or a loss
of Modbus communication. A started compressor that is still requested but reports 0 Hz for
COMPRESSOR_UNEXPECTED_STOP_S (180 s), outside a defrost or oil return cycle, raises TO07. Like the inside unit
failsafe (TO06), OpenAmber then stops everything and re-initializes. The error blocks heat and DHW demand until
the user resets it.
"""

from helpers import advance_until_state, start_space_heating

UNEXPECTED_STOP_S = 180
FREQUENCY_OVERRIDE = "mock_compressor_frequency_override"
ERROR = "error_compressor_unexpected_stop"


def compressor_mode(openamber):
    """Compressor mode index the controller requests from the outdoor unit."""
    return int(openamber.get_entity("compressor_control_select") or 0)


def outdoor_unit_stops_compressor(openamber):
    """The outdoor unit stops the compressor by itself: the reported frequency drops to 0 Hz."""
    openamber.set_number(FREQUENCY_OVERRIDE, 0)
    openamber.step(ms=50)


def outdoor_unit_follows_requests_again(openamber):
    """The reported frequency follows the requested compressor mode again."""
    openamber.set_number(FREQUENCY_OVERRIDE, -1)
    openamber.step(ms=50)


def reset_error(openamber):
    openamber.set_switch("reset_start_timeout_errors_switch", True)
    openamber.step(ms=50)


def stop_heating(openamber):
    outdoor_unit_follows_requests_again(openamber)
    reset_error(openamber)
    openamber.set_switch("heat_demand_switch", False)
    openamber.advance_time(seconds=30, step_s=5)


def test_heating_unexpected_stop_raises_error_and_stops_everything(clean_system):
    """0 Hz for 180 s while heating raises TO07, stops the compressor and re-initializes; reset restarts heating."""
    openamber = clean_system
    start_space_heating(openamber)

    outdoor_unit_stops_compressor(openamber)
    openamber.advance_time(seconds=UNEXPECTED_STOP_S - 20, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity(ERROR) is False, "No error before 180 s"
    assert compressor_mode(openamber) > 0

    openamber.advance_time(seconds=30, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity(ERROR) is True, "TO07 after 180 s at 0 Hz"
    assert openamber.get_entity("error_active") is True
    assert compressor_mode(openamber) == 0, "The compressor must be stopped"
    assert openamber.get_entity("state_machine_state_heat_cool") == "Idle"
    assert openamber.get_entity("heat_demand_active_sensor") is False, "The error blocks heat demand"

    # The controller re-initializes and then stays idle while the error is active.
    outdoor_unit_follows_requests_again(openamber)
    openamber.advance_time(seconds=600, step_s=10)
    openamber.step(ms=50)
    assert compressor_mode(openamber) == 0, "No restart while the error is active"

    # After the user resets the error, heating starts again through the normal path.
    reset_error(openamber)
    assert openamber.get_entity(ERROR) is False
    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Compressor running", max_s=1800), (
        f"Heating never restarted after the reset (state: {openamber.get_entity('state_machine_state_heat_cool')})"
    )
    stop_heating(openamber)


def test_dhw_unexpected_stop_turns_off_backup_heater(clean_system):
    """During DHW with the backup heater on, an unexpected stop raises TO07 and turns the backup heater off."""
    openamber = clean_system
    openamber.set_number("dhw_setpoint_temperature", 50.0)
    openamber.set_number("dhw_restart_dhw_delta", 5.0)
    openamber.set_number("dhw_backup_min_avg_rate", 0.2)
    openamber.set_number("dhw_backup_min_avg_rate_delay_minutes", 5.0)
    # The tank temperature stays flat, so the backup heater turns on after the rate check.
    openamber.set_sensor("dhw_temperature_tw_sensor", 40.0)
    openamber.step(ms=100)
    assert advance_until_state(openamber, "state_machine_state_dhw", "Backup heater running", max_s=1500, step=20), (
        f"Backup heater never turned on (state: {openamber.get_entity('state_machine_state_dhw')})"
    )
    assert openamber.get_entity("backup_heater_relay") is True

    outdoor_unit_stops_compressor(openamber)
    openamber.advance_time(seconds=UNEXPECTED_STOP_S + 10, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity(ERROR) is True
    assert openamber.get_entity("backup_heater_relay") is False, "The backup heater must not keep heating alone"
    assert compressor_mode(openamber) == 0
    assert openamber.get_entity("state_machine_state_dhw") == "Idle"
    assert openamber.get_entity("dhw_demand_active_sensor") is False, "The error blocks DHW demand"

    outdoor_unit_follows_requests_again(openamber)
    openamber.set_sensor("dhw_temperature_tw_sensor", 55.0)
    reset_error(openamber)
    openamber.advance_time(seconds=60, step_s=5)


def test_no_error_while_compressor_runs(clean_system):
    """A running compressor never raises the error, however long it runs."""
    openamber = clean_system
    start_space_heating(openamber)

    openamber.advance_time(seconds=UNEXPECTED_STOP_S * 3, step_s=10)
    openamber.step(ms=50)
    assert openamber.get_entity(ERROR) is False
    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"
    stop_heating(openamber)


def test_no_error_during_defrost_or_oil_return(clean_system):
    """0 Hz during a defrost or an oil return cycle is expected; the 180 s only count outside them."""
    openamber = clean_system
    start_space_heating(openamber)

    outdoor_unit_stops_compressor(openamber)
    openamber.set_sensor("error_register_2119_raw", 8.0)
    openamber.advance_time(seconds=UNEXPECTED_STOP_S * 2, step_s=10)
    openamber.step(ms=50)
    assert openamber.get_entity(ERROR) is False, "0 Hz during an oil return cycle is no error"
    openamber.set_sensor("error_register_2119_raw", 0.0)

    openamber.set_binary_sensor("defrost_active_sensor", True)
    openamber.advance_time(seconds=UNEXPECTED_STOP_S * 2, step_s=10)
    openamber.step(ms=50)
    assert openamber.get_entity(ERROR) is False, "0 Hz during a defrost is no error"
    openamber.set_binary_sensor("defrost_active_sensor", False)

    # After the defrost a full 180 s at 0 Hz is needed again.
    openamber.advance_time(seconds=UNEXPECTED_STOP_S - 20, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity(ERROR) is False, "The 0 Hz period must start again after the defrost"
    openamber.advance_time(seconds=30, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity(ERROR) is True
    stop_heating(openamber)
