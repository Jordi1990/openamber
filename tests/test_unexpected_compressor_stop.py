"""Automated tests for a compressor that the outdoor unit stops by itself.

The outdoor unit can stop the compressor on its own, for example after a protection trip, a power dip or
a loss of Modbus communication. OpenAmber keeps requesting the same compressor mode, so without a check it
stays in a running state and, during DHW, lets the backup heater do all the work.

A started compressor that reports 0 Hz for COMPRESSOR_UNEXPECTED_STOP_S (180 s), outside a defrost or oil
return cycle, is stopped by OpenAmber so the normal start path can start it again. Three such stops within
an hour raise error TO07 instead.
"""

from helpers import advance_until_state, start_space_heating

UNEXPECTED_STOP_S = 180
FREQUENCY_OVERRIDE = "mock_compressor_frequency_override"


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


def stop_heating(openamber):
    outdoor_unit_follows_requests_again(openamber)
    openamber.set_switch("heat_demand_switch", False)
    openamber.advance_time(seconds=30, step_s=5)


def test_heating_restarts_compressor_after_unexpected_stop(clean_system):
    """Heating stops a compressor that reports 0 Hz for 180 s and starts it again through the normal path."""
    openamber = clean_system
    start_space_heating(openamber)

    outdoor_unit_stops_compressor(openamber)
    openamber.advance_time(seconds=UNEXPECTED_STOP_S - 20, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running", "No action before 180 s"
    assert compressor_mode(openamber) > 0

    openamber.advance_time(seconds=30, step_s=5)
    openamber.step(ms=50)
    assert compressor_mode(openamber) == 0, "The compressor must be stopped after 180 s at 0 Hz"
    assert openamber.get_entity("state_machine_state_heat_cool") != "Compressor running"
    assert openamber.get_entity("error_compressor_unexpected_stop") is False, "One stop is no error"

    outdoor_unit_follows_requests_again(openamber)
    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Compressor running", max_s=900), (
        f"Heating never restarted the compressor (state: {openamber.get_entity('state_machine_state_heat_cool')})"
    )
    stop_heating(openamber)


def test_no_action_while_compressor_still_runs(clean_system):
    """A running compressor is never stopped by the check, however long it runs."""
    openamber = clean_system
    start_space_heating(openamber)

    openamber.advance_time(seconds=UNEXPECTED_STOP_S * 2, step_s=10)
    openamber.step(ms=50)
    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"
    assert compressor_mode(openamber) > 0
    stop_heating(openamber)


def test_no_action_during_defrost_or_oil_return(clean_system):
    """0 Hz during a defrost (including its pre-phase) or an oil return cycle is expected and ignored."""
    openamber = clean_system
    start_space_heating(openamber)

    outdoor_unit_stops_compressor(openamber)
    openamber.set_sensor("error_register_2119_raw", 8.0)
    openamber.advance_time(seconds=UNEXPECTED_STOP_S * 2, step_s=10)
    openamber.step(ms=50)
    assert compressor_mode(openamber) > 0, "0 Hz during an oil return cycle must not stop the compressor"

    openamber.set_sensor("error_register_2119_raw", 0.0)
    openamber.set_binary_sensor("defrost_active_sensor", True)
    openamber.advance_time(seconds=UNEXPECTED_STOP_S * 2, step_s=10)
    openamber.step(ms=50)
    assert compressor_mode(openamber) > 0, "0 Hz during a defrost must not stop the compressor"

    openamber.set_binary_sensor("defrost_active_sensor", False)
    stop_heating(openamber)


def test_three_unexpected_stops_within_an_hour_raise_error(clean_system):
    """The third unexpected stop within an hour raises TO07 instead of another restart."""
    openamber = clean_system
    start_space_heating(openamber)

    for stop in range(1, 4):
        outdoor_unit_stops_compressor(openamber)
        openamber.advance_time(seconds=UNEXPECTED_STOP_S + 10, step_s=5)
        openamber.step(ms=50)
        assert compressor_mode(openamber) == 0, f"Unexpected stop {stop} must stop the compressor"
        if stop < 3:
            assert openamber.get_entity("error_compressor_unexpected_stop") is False
            outdoor_unit_follows_requests_again(openamber)
            assert advance_until_state(openamber, "state_machine_state_heat_cool", "Compressor running", max_s=900)

    assert openamber.get_entity("error_compressor_unexpected_stop") is True, "Third stop within an hour is an error"
    assert openamber.get_entity("error_active") is True

    openamber.set_switch("reset_start_timeout_errors_switch", True)
    openamber.step(ms=50)
    assert openamber.get_entity("error_compressor_unexpected_stop") is False, "The reset switch clears TO07"
    stop_heating(openamber)


def test_dhw_turns_off_backup_heater_and_restarts_after_unexpected_stop(clean_system):
    """During DHW with the backup heater on, a stopped compressor ends the backup heater run and DHW starts again."""
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
    assert openamber.get_entity("backup_heater_relay") is False, "The backup heater must not keep heating alone"
    assert compressor_mode(openamber) == 0

    outdoor_unit_follows_requests_again(openamber)
    assert advance_until_state(openamber, "state_machine_state_dhw", "Compressor running", max_s=900), (
        f"DHW never restarted the compressor (state: {openamber.get_entity('state_machine_state_dhw')})"
    )

    openamber.set_sensor("dhw_temperature_tw_sensor", 55.0)
    assert advance_until_state(openamber, "state_machine_state_dhw", "Idle", max_s=900)
