"""Automated tests for the minimum compressor off time before DHW starts the compressor.

The outdoor unit blocks a compressor restart for 180 s after a stop. When heating stops the compressor for a
DHW request, DHW starts its pump but requests the compressor only after COMPRESSOR_MIN_OFF_S (200 s), like
heating does.
"""

import pytest

from helpers import advance_until_state, offset_s, start_space_heating

MIN_OFF_S = 200


def compressor_mode(openamber):
    return int(openamber.get_entity("compressor_control_select") or 0)


def switch_heating_to_dhw(openamber):
    """Start heating, then request DHW. Returns the virtual time at which heating stopped the compressor."""
    openamber.set_switch("dhw_enabled_switch", False)
    openamber.set_sensor("dhw_temperature_tw_sensor", 38.0)
    start_space_heating(openamber)
    assert compressor_mode(openamber) > 0

    openamber.set_switch("dhw_enabled_switch", True)
    for _ in range(900):
        if compressor_mode(openamber) == 0:
            return offset_s(openamber)
        openamber.advance_time(seconds=1, step_s=1)
        openamber.step(ms=20)
    pytest.fail("Heating never stopped the compressor for the DHW request")


def test_dhw_requests_compressor_only_after_min_off_time(clean_system):
    """DHW starts its pump, but the compressor only after the minimum off time since heating stopped it."""
    openamber = clean_system
    stopped_at = switch_heating_to_dhw(openamber)

    assert advance_until_state(openamber, "state_machine_state_dhw", "Pump running", max_s=MIN_OFF_S, step=5), (
        f"DHW pump never started (dhw: {openamber.get_entity('state_machine_state_dhw')})"
    )
    requested_at = None
    for _ in range(int(MIN_OFF_S / 5) + 10):
        if compressor_mode(openamber) > 0:
            requested_at = offset_s(openamber)
            break
        assert openamber.get_entity("state_machine_state_main") == "DHW"
        openamber.advance_time(seconds=5, step_s=5)
        openamber.step(ms=20)

    assert requested_at is not None, "DHW never requested the compressor"
    waited = requested_at - stopped_at
    # The controller runs every 5 s and the stop is observed with 1 s steps, so allow one controller update.
    assert MIN_OFF_S - 5 <= waited <= MIN_OFF_S + 15, f"DHW requested the compressor {waited:.0f} s after the stop"
    assert advance_until_state(openamber, "state_machine_state_dhw", "Compressor running", max_s=180, step=5)


def test_dhw_stops_pump_when_demand_ends_while_waiting(clean_system):
    """When the DHW demand ends during the wait, DHW stops its pump instead of starting the compressor."""
    openamber = clean_system
    switch_heating_to_dhw(openamber)
    assert advance_until_state(openamber, "state_machine_state_dhw", "Pump running", max_s=MIN_OFF_S, step=5)

    openamber.set_switch("dhw_enabled_switch", False)
    openamber.step(ms=50)
    assert advance_until_state(openamber, "state_machine_state_dhw", "Idle", max_s=180, step=5), (
        f"DHW did not go idle (dhw: {openamber.get_entity('state_machine_state_dhw')})"
    )
    assert compressor_mode(openamber) == 0, "The compressor must not start without DHW demand"


def test_dhw_does_not_wait_in_emergency_mode(clean_system):
    """In emergency mode the compressor is not used, so DHW switches on the backup heater without waiting."""
    openamber = clean_system
    stopped_at = switch_heating_to_dhw(openamber)
    openamber.set_switch("emergency_mode_enabled", True)

    assert advance_until_state(openamber, "backup_heater_relay", True, max_s=MIN_OFF_S, step=5)
    assert offset_s(openamber) - stopped_at < MIN_OFF_S
    assert compressor_mode(openamber) == 0

    openamber.set_switch("emergency_mode_enabled", False)
    openamber.set_sensor("dhw_temperature_tw_sensor", 55.0)
    assert advance_until_state(openamber, "state_machine_state_dhw", "Idle", max_s=600)
