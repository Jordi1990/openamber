"""Automated tests for a compressor that the outdoor unit stops by itself.

The outdoor unit can stop the compressor on its own, for example after a protection trip, a power dip or
a loss of Modbus communication. OpenAmber keeps requesting the same compressor mode, so without a check it
stays in a running state and, during DHW, lets the backup heater do all the work.

A started compressor that reports 0 Hz for COMPRESSOR_UNEXPECTED_STOP_S (180 s), outside a defrost or oil
return cycle, is stopped by OpenAmber so the normal start path can start it again. Three such stops within
an hour raise error TO07 instead.
"""

from helpers import advance_until_state, offset_s, start_space_heating

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


def test_zero_hz_period_restarts_after_defrost_and_oil_return(clean_system):
    """0 Hz before a defrost or oil return cycle does not count afterwards: a fresh 180 s period starts."""
    openamber = clean_system
    # Unexpected stops of earlier tests fall outside the one hour TO07 window.
    openamber.advance_time(seconds=3700, step_s=60)
    start_space_heating(openamber)

    for exclusion in ("oil_return", "defrost"):
        # Seed the 0 Hz period, then let the exclusion run while the compressor stays at 0 Hz.
        outdoor_unit_stops_compressor(openamber)
        openamber.advance_time(seconds=150, step_s=5)
        if exclusion == "oil_return":
            openamber.set_sensor("error_register_2119_raw", 8.0)
            openamber.advance_time(seconds=120, step_s=5)
            openamber.set_sensor("error_register_2119_raw", 0.0)
        else:
            openamber.set_binary_sensor("defrost_active_sensor", True)
            openamber.advance_time(seconds=120, step_s=5)
            openamber.set_binary_sensor("defrost_active_sensor", False)
            # After a defrost heating first settles before it returns to "Compressor running".
            assert advance_until_state(openamber, "state_machine_state_heat_cool", "Compressor running", max_s=600, step=5)
        openamber.step(ms=50)

        openamber.advance_time(seconds=UNEXPECTED_STOP_S - 20, step_s=5)
        openamber.step(ms=50)
        assert compressor_mode(openamber) > 0, f"0 Hz from before the {exclusion} must not count afterwards"
        openamber.advance_time(seconds=30, step_s=5)
        openamber.step(ms=50)
        assert compressor_mode(openamber) == 0, f"A full 180 s at 0 Hz after the {exclusion} must stop the compressor"

        outdoor_unit_follows_requests_again(openamber)
        assert advance_until_state(openamber, "state_machine_state_heat_cool", "Compressor running", max_s=900), (
            f"No restart after the {exclusion} case (state: {openamber.get_entity('state_machine_state_heat_cool')}, "
            f"TO07: {openamber.get_entity('error_compressor_unexpected_stop')})"
        )

    openamber.set_switch("reset_start_timeout_errors_switch", True)
    stop_heating(openamber)


def test_dhw_keeps_demand_and_waits_min_off_time_after_unexpected_stop(clean_system):
    """With the tank between the restart threshold and the setpoint, DHW keeps its demand and the valve on DHW,
    and starts the compressor again only after the minimum off time."""
    openamber = clean_system
    # Unexpected stops of earlier tests fall outside the one hour TO07 window.
    openamber.advance_time(seconds=3700, step_s=60)
    openamber.set_number("dhw_setpoint_temperature", 50.0)
    openamber.set_number("dhw_restart_dhw_delta", 5.0)
    openamber.set_number("dhw_backup_min_avg_rate", 0.0)
    openamber.set_sensor("dhw_temperature_tw_sensor", 40.0)
    openamber.step(ms=100)
    assert advance_until_state(openamber, "state_machine_state_dhw", "Compressor running", max_s=600, step=5)

    # Tank between setpoint - delta (45) and setpoint (50): only the dhw_active hysteresis keeps the demand.
    openamber.set_sensor("dhw_temperature_tw_sensor", 47.0)
    outdoor_unit_stops_compressor(openamber)
    for _ in range(80):
        if compressor_mode(openamber) == 0:
            break
        openamber.advance_time(seconds=5, step_s=5)
        openamber.step(ms=20)
    assert compressor_mode(openamber) == 0, "The unexpected stop must stop the compressor"
    stopped_at = offset_s(openamber)
    outdoor_unit_follows_requests_again(openamber)

    restarted_at = None
    for _ in range(120):
        openamber.advance_time(seconds=5, step_s=5)
        openamber.step(ms=20)
        assert openamber.get_entity("state_machine_state_main") == "DHW", "DHW must keep its demand during recovery"
        assert openamber.get_entity("three_way_valve_dhw_switch") is True
        if compressor_mode(openamber) > 0:
            restarted_at = offset_s(openamber)
            break
        assert openamber.get_entity("pump_p0_relay_switch") is True, "P0 keeps running while DHW waits to restart"
    assert restarted_at is not None, "DHW never restarted the compressor"
    assert restarted_at - stopped_at >= 120, f"Restarted {restarted_at - stopped_at:.0f} s after the stop, before the minimum off time"
    assert openamber.get_entity("error_compressor_unexpected_stop") is False

    openamber.set_sensor("dhw_temperature_tw_sensor", 55.0)
    assert advance_until_state(openamber, "state_machine_state_dhw", "Idle", max_s=900)


def test_zero_hz_before_dhw_pump_settling_does_not_count_afterwards(clean_system):
    """DHW pump settling is a state without the check. 0 Hz that started before it must not count afterwards:
    once DHW is back in "Compressor running" a fresh 180 s period is needed."""
    openamber = clean_system
    # Unexpected stops of earlier tests fall outside the one hour TO07 window.
    openamber.advance_time(seconds=3700, step_s=60)
    openamber.set_number("dhw_setpoint_temperature", 50.0)
    openamber.set_number("dhw_restart_dhw_delta", 5.0)
    openamber.set_number("dhw_backup_min_avg_rate", 0.0)
    # Delta-T mode with supply colder than the tank: the tank pump waits, DHW stays in "Compressor running".
    openamber.set_select("dhw_pump_start_mode_select", "Aanvoer warmer dan vat (ΔT)")
    openamber.set_sensor("outlet_temperature_tuo", 30.0)
    openamber.set_sensor("inlet_temperature_tui", 28.0)
    openamber.set_sensor("dhw_temperature_tw_sensor", 40.0)
    openamber.step(ms=100)
    assert advance_until_state(openamber, "state_machine_state_dhw", "Compressor running", max_s=600, step=5)

    # 60 s at 0 Hz while the check runs, then the supply gets warmer than the tank and the tank pump starts.
    outdoor_unit_stops_compressor(openamber)
    openamber.advance_time(seconds=60, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity("state_machine_state_dhw") == "Compressor running"
    openamber.set_sensor("outlet_temperature_tuo", 45.0)
    openamber.set_sensor("inlet_temperature_tui", 40.0)
    assert advance_until_state(openamber, "state_machine_state_dhw", "DHW pump settled", max_s=300, step=5)
    assert advance_until_state(openamber, "state_machine_state_dhw", "Compressor running", max_s=60, step=1)
    back_at = offset_s(openamber)

    # Checked every 5 s: a stop right after settling would otherwise be hidden by the restart that follows it.
    for _ in range((UNEXPECTED_STOP_S - 20) // 5):
        openamber.advance_time(seconds=5, step_s=5)
        openamber.step(ms=20)
        assert compressor_mode(openamber) > 0, (
            f"Stopped {offset_s(openamber) - back_at:.0f} s after DHW pump settling: the 0 Hz from before it counted"
        )
    openamber.advance_time(seconds=30, step_s=5)
    openamber.step(ms=50)
    assert compressor_mode(openamber) == 0, "A full 180 s at 0 Hz after pump settling must stop the compressor"

    outdoor_unit_follows_requests_again(openamber)
    openamber.set_sensor("dhw_temperature_tw_sensor", 55.0)
    assert advance_until_state(openamber, "state_machine_state_dhw", "Idle", max_s=900)
