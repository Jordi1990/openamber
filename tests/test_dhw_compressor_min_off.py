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


def time_dhw_leaves_idle(openamber, max_s=900, pump_must_stay_off=False):
    """Advance until the DHW controller leaves IDLE and return that virtual time.

    With pump_must_stay_off, P0 must stay off for as long as DHW is still idle.
    """
    for _ in range(max_s // 5):
        if openamber.get_entity("state_machine_state_dhw") != "Idle" and openamber.get_entity("state_machine_state_main") == "DHW":
            return offset_s(openamber)
        if pump_must_stay_off:
            assert openamber.get_entity("internal_pump_active") is False, "P0 must stay off while DHW waits"
        openamber.advance_time(seconds=5, step_s=5)
        openamber.step(ms=20)
    pytest.fail(f"DHW never left Idle (dhw: {openamber.get_entity('state_machine_state_dhw')})")


@pytest.mark.parametrize("minutes", [5, 2])
def test_dhw_waits_for_compressor_min_off_time_after_heating(clean_system, minutes):
    """DHW starts its pump only after the compressor has been off for the configured time."""
    openamber = clean_system
    openamber.set_number(MIN_OFF_SETTING, minutes)

    stopped_at = switch_heating_to_dhw(openamber)
    # P0 stays off while DHW waits; heating finishes its own pump stop first.
    assert advance_until_state(openamber, "internal_pump_active", False, max_s=180, step=5)
    started_at = time_dhw_leaves_idle(openamber, pump_must_stay_off=True)

    waited = started_at - stopped_at
    # The controller runs every 5 s and the stop is observed with 1 s steps, so allow one controller update.
    assert minutes * 60 - 5 <= waited <= minutes * 60 + 15, f"DHW started {waited:.0f} s after the compressor stop"
    assert advance_until_state(openamber, "internal_pump_active", True, max_s=120, step=5), (
        "The DHW pump must run after the wait"
    )


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


def test_dhw_does_not_wait_after_a_stop_without_compressor_run(clean_system):
    """Stop() calls while the compressor was not started (here: re-initialization) do not start a wait."""
    openamber = clean_system
    openamber.set_number(MIN_OFF_SETTING, 10)
    openamber.set_switch("dhw_enabled_switch", False)
    openamber.set_sensor("dhw_temperature_tw_sensor", 38.0)
    # Any compressor run of an earlier test is now longer ago than the 10 min setting.
    openamber.advance_time(seconds=11 * 60, step_s=30)

    # Leaving maintenance mode re-initializes the controllers, which calls Stop() without a compressor run.
    openamber.set_switch("service_mode_enabled", True)
    assert advance_until_state(openamber, "state_machine_state_main", "Maintenance", max_s=120, step=5)
    openamber.set_switch("service_mode_enabled", False)
    assert advance_until_state(openamber, "state_machine_state_main", "Heat/Cool", max_s=300, step=5)
    requested_at = offset_s(openamber)

    openamber.set_switch("dhw_enabled_switch", True)
    started_at = time_dhw_leaves_idle(openamber)

    # The 3-way valve takes 60 s; allow some polling on top of that.
    assert started_at - requested_at < 150, f"DHW waited {started_at - requested_at:.0f} s without a compressor run"


def test_dhw_waits_after_a_start_that_timed_out(clean_system):
    """A start that never got the compressor running (TO02) still counts: the outdoor unit may have reacted to
    it. After the error is reset, DHW waits the configured time from that stop."""
    openamber = clean_system
    openamber.set_number(MIN_OFF_SETTING, 5)
    openamber.set_sensor("dhw_temperature_tw_sensor", 38.0)
    # The outdoor unit never reports a frequency: the DHW start runs into the compressor start timeout.
    openamber.set_number("mock_compressor_frequency_override", 0)
    openamber.step(ms=50)
    assert advance_until_state(openamber, "error_compressor_start_timeout", True, max_s=900, step=5), (
        f"No start timeout (dhw: {openamber.get_entity('state_machine_state_dhw')})"
    )
    stopped_at = offset_s(openamber)
    assert advance_until_state(openamber, "state_machine_state_dhw", "Idle", max_s=300, step=5)

    openamber.set_number("mock_compressor_frequency_override", -1)
    openamber.set_switch("reset_start_timeout_errors_switch", True)
    openamber.step(ms=50)
    started_at = time_dhw_leaves_idle(openamber)

    assert started_at - stopped_at >= 5 * 60 - 5, f"DHW restarted {started_at - stopped_at:.0f} s after the timed-out start"
