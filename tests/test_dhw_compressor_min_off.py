"""Automated tests for the minimum compressor off time before DHW starts.

The outdoor unit delays a restart after a stop. When heating stops the compressor for a DHW
request, DHW must wait until the compressor has been off for the configured time, otherwise the
start can run out the compressor start timeout.
"""

import pytest

from helpers import advance_until_state, offset_s, start_space_heating

MIN_OFF_SETTING = "dhw_compressor_min_off_minutes"


def compressor_stopped(openamber):
    """True when the controller has set the compressor control select to off."""
    return openamber.get_entity("compressor_control_select") == "0"


def switch_heating_to_dhw(openamber):
    """Start heating, then request DHW. Returns the virtual time at which the compressor was stopped."""
    openamber.set_switch("dhw_enabled_switch", False)
    openamber.set_sensor("dhw_temperature_tw_sensor", 38.0)
    start_space_heating(openamber)
    assert not compressor_stopped(openamber)

    openamber.set_switch("dhw_enabled_switch", True)
    for _ in range(600):
        if compressor_stopped(openamber):
            return offset_s(openamber)
        openamber.advance_time(seconds=1, step_s=1)
        openamber.step(ms=20)
    pytest.fail("Heating never stopped the compressor for the DHW request")


def time_dhw_leaves_idle(openamber, max_s=900):
    """Advance until the DHW controller leaves IDLE and return that virtual time."""
    assert advance_until_state(openamber, "state_machine_state_main", "DHW", max_s=max_s, step=5)
    for _ in range(max_s // 5):
        if openamber.get_entity("state_machine_state_dhw") != "Idle":
            return offset_s(openamber)
        openamber.advance_time(seconds=5, step_s=5)
        openamber.step(ms=20)
    pytest.fail(f"DHW never left Idle (dhw: {openamber.get_entity('state_machine_state_dhw')})")


@pytest.mark.parametrize("minutes", [5, 2])
def test_dhw_waits_for_compressor_min_off_time_after_heating(clean_system, minutes):
    """DHW starts its pump only after the compressor has been off for the configured time."""
    openamber = clean_system
    openamber.set_number(MIN_OFF_SETTING, minutes)

    stopped_at = switch_heating_to_dhw(openamber)
    started_at = time_dhw_leaves_idle(openamber)

    waited = started_at - stopped_at
    assert minutes * 60 <= waited <= minutes * 60 + 15, f"DHW started {waited:.0f} s after the compressor stop"


def test_dhw_starts_without_waiting_in_emergency_mode(clean_system):
    """In emergency mode the compressor is not used, so DHW does not wait for it."""
    openamber = clean_system
    openamber.set_number(MIN_OFF_SETTING, 10)

    stopped_at = switch_heating_to_dhw(openamber)
    openamber.set_switch("emergency_mode_enabled", True)
    started_at = time_dhw_leaves_idle(openamber)

    assert started_at - stopped_at < 10 * 60


def test_dhw_does_not_wait_when_setting_is_zero(clean_system):
    """With the setting at 0 minutes DHW starts right after heating stopped the compressor."""
    openamber = clean_system
    openamber.set_number(MIN_OFF_SETTING, 0)

    stopped_at = switch_heating_to_dhw(openamber)
    started_at = time_dhw_leaves_idle(openamber)

    assert started_at - stopped_at < 120
