"""Pump run and interval timers must survive the millisecond counter wrapping."""

import pytest

from helpers import advance_until_state, start_space_heating


MILLIS_PERIOD = 1 << 32


def advance_to_millis(openamber, target):
    """Move forward to a counter value without a single half-period time jump."""
    current = openamber.get_time()["millis"]
    remaining = (target - current) % MILLIS_PERIOD
    # ESPHome's scheduler also uses signed differences for deadlines. Give it
    # a chance to process due work between jumps shorter than half a period.
    while remaining:
        chunk = min(remaining, 1_000_000_000)
        openamber.advance_time(ms=chunk)
        remaining -= chunk


def test_pump_keeps_running_until_duration_elapses_across_rollover(clean_system):
    """A deadline beyond rollover must not stop the pump before two minutes."""
    openamber = clean_system
    openamber.set_switch("legio_enabled_switch", False)
    openamber.set_number("pump_duration", 2.0)
    openamber.set_select("heat_mode_select", "Extern setpoint")
    openamber.set_number("manual_setpoint", 25.0)

    advance_to_millis(openamber, MILLIS_PERIOD - 60_000)
    # Water is already above the heating target, so only the circulation pump
    # should run its interval cycle; the compressor must stay off.
    openamber.set_switch("heat_demand_switch", True)
    assert advance_until_state(
        openamber, "state_machine_state_heat_cool", "Pump running", max_s=30, step=5
    )
    assert openamber.get_time()["millis"] > MILLIS_PERIOD - 60_000

    openamber.advance_time(seconds=30, step_s=5)
    assert openamber.get_entity("internal_pump_active") is True
    openamber.advance_time(seconds=40, step_s=5)
    assert openamber.get_time()["millis"] < 60_000
    assert openamber.get_entity("internal_pump_active") is True
    assert float(openamber.get_entity("current_compressor_frequency")) == 0

    openamber.advance_time(seconds=60, step_s=5)
    assert openamber.get_entity("internal_pump_active") is False
    openamber.set_switch("heat_demand_switch", False)


def test_heat_to_dhw_after_pump_run_spans_rollover(clean_system):
    """An old pump run must finish so heating can hand over to hot water."""
    openamber = clean_system
    openamber.set_switch("legio_enabled_switch", False)
    openamber.set_number("pump_duration", 2.0)

    # Start the pump and compressor before rollover, with the two-minute pump
    # duration already expired before the counter wraps during a long heat run.
    advance_to_millis(openamber, MILLIS_PERIOD - 3_600_000)
    start_space_heating(openamber)
    assert openamber.get_time()["millis"] > MILLIS_PERIOD - 3_600_000
    assert openamber.get_entity("internal_pump_active") is True

    advance_to_millis(openamber, 20_000)
    assert openamber.get_time()["millis"] < 60_000
    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"

    # DHW requests a stop. Previously, now >= pump_start + duration became
    # false again after rollover and prevented the pump from stopping.
    openamber.set_sensor("dhw_temperature_tw_sensor", 38.0)
    assert advance_until_state(
        openamber, "state_machine_state_heat_cool", "Idle", max_s=90, step=5
    ), (
        "Heating did not stop its pump after rollover: "
        f"{openamber.get_entity('state_machine_state_heat_cool')}"
    )
    assert advance_until_state(
        openamber, "state_machine_state_main", "DHW", max_s=120
    ), (
        "Heating did not hand over to DHW after rollover: "
        f"{openamber.get_entity('state_machine_state_heat_cool')}"
    )
    assert openamber.get_entity("three_way_valve_dhw_switch") is True
    assert advance_until_state(
        openamber, "state_machine_state_dhw", "Compressor running", max_s=600
    ), "DHW compressor did not restart after the handover"
    assert float(openamber.get_entity("current_compressor_frequency")) > 0

    # Keep virtual time moving forward; resetting it would invalidate other
    # controllers' timestamps in the shared simulator session.
    openamber.set_switch("heat_demand_switch", False)
    openamber.set_sensor("dhw_temperature_tw_sensor", 55.0)
    assert advance_until_state(
        openamber, "state_machine_state_main", "Heat/Cool", max_s=900
    )


def run_pump_interval_before_rollover(openamber, start_millis):
    """Run a warm-water cycle to arm a fresh 15-minute interval before rollover."""
    openamber.set_switch("legio_enabled_switch", False)
    openamber.set_number("pump_duration", 2.0)
    openamber.set_number("pump_interval", 15.0)
    openamber.set_select("heat_mode_select", "Extern setpoint")
    openamber.set_number("manual_setpoint", 25.0)
    advance_to_millis(openamber, start_millis)
    openamber.set_switch("heat_demand_switch", True)
    assert advance_until_state(
        openamber, "state_machine_state_heat_cool", "Pump running", max_s=30, step=5
    )
    assert advance_until_state(
        openamber, "state_machine_state_heat_cool", "Idle", max_s=150, step=5
    )
    assert openamber.get_entity("internal_pump_active") is False


def test_pump_interval_waits_across_rollover(clean_system):
    """A wrapped deadline must not start the next cycle early."""
    openamber = clean_system
    run_pump_interval_before_rollover(openamber, MILLIS_PERIOD - 300_000)

    # Changing the configured interval must not change an already scheduled wait.
    openamber.set_number("pump_interval", 5.0)
    openamber.advance_time(seconds=60, step_s=5)
    assert openamber.get_time()["millis"] > MILLIS_PERIOD - 300_000
    assert openamber.get_entity("internal_pump_active") is False
    openamber.advance_time(seconds=780, step_s=30)
    assert openamber.get_time()["millis"] < 900_000
    assert openamber.get_entity("internal_pump_active") is False
    assert advance_until_state(
        openamber, "state_machine_state_heat_cool", "Pump running", max_s=90, step=5
    )

    openamber.set_number("pump_interval", 15.0)
    openamber.set_switch("heat_demand_switch", False)


@pytest.mark.parametrize("resume_millis", [20_000, 30 * 24 * 60 * 60 * 1000])
def test_overdue_pump_interval_starts_after_rollover(clean_system, resume_millis):
    """An expired interval stays due across rollover, including a 30-day idle."""
    openamber = clean_system
    run_pump_interval_before_rollover(openamber, MILLIS_PERIOD - 1_800_000)
    openamber.set_switch("heat_demand_switch", False)

    advance_to_millis(openamber, resume_millis)
    openamber.set_switch("heat_demand_switch", True)
    assert advance_until_state(
        openamber, "state_machine_state_heat_cool", "Pump running", max_s=30, step=5
    ), "An overdue pump interval must not delay new demand after rollover"
    openamber.set_switch("heat_demand_switch", False)


def test_frost_reset_bypasses_pump_interval_near_rollover(clean_system):
    """ResetInterval must allow an immediate start even late in the timer period."""
    openamber = clean_system
    run_pump_interval_before_rollover(openamber, MILLIS_PERIOD - 1_800_000)
    openamber.set_switch("heat_demand_switch", False)
    openamber.set_number("frost_protection_stage_1_temp_ta", 5.0)
    openamber.set_sensor("temperature_outside_ta", 2.0)

    # The frost filter takes five minutes; the pump interval still has ten left.
    openamber.advance_time(seconds=320, step_s=5)
    assert openamber.get_time()["millis"] > MILLIS_PERIOD - 1_800_000
    assert openamber.get_entity("frost_protection_stage1_active") is True
    assert openamber.get_entity("internal_pump_active") is True

    openamber.set_sensor("temperature_outside_ta", 7.0)
    openamber.advance_time(seconds=310, step_s=15)
