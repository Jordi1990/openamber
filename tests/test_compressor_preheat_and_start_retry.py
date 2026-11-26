"""Automated tests for the compressor preheat (P07) and the retry of a compressor start that times out.

While the outdoor unit preheats the compressor (P07, register 2119 bit 0x40, about 30 min) it does not start the
compressor. OpenAmber holds the start until P07 clears instead of letting the start time out after
COMPRESSOR_START_TIMEOUT_S (120 s).

A start that times out without P07 is retried after the minimum off time. Only COMPRESSOR_START_MAX_ATTEMPTS (3)
failed starts in a row raise TO02 (error_compressor_start_timeout), which blocks all demand until it is reset.
"""

from helpers import advance_until_state

P07 = 0x40
ERROR = "error_compressor_start_timeout"
FREQUENCY_OVERRIDE = "mock_compressor_frequency_override"
PREHEAT_S = 30 * 60
START_TIMEOUT_S = 120
MAX_ATTEMPTS = 3


def compressor_mode(openamber):
    """Compressor mode index the controller requests from the outdoor unit (1999)."""
    return int(openamber.get_entity("compressor_control_select") or 0)


def set_preheat(openamber, active):
    openamber.set_sensor("error_register_2119_raw", float(P07) if active else 0.0)
    openamber.step(ms=50)


def outdoor_unit_refuses_start(openamber, refuse):
    """refuse=True: the outdoor unit accepts the request but the compressor stays at 0 Hz."""
    openamber.set_number(FREQUENCY_OVERRIDE, 0 if refuse else -1)
    openamber.step(ms=50)


def request_heating(openamber, tc=28.0, target=35.0):
    """Heat demand with a start temperature that asks for the compressor (same setup as start_space_heating)."""
    openamber.set_select("thermostat_mode_select", "Extern")
    openamber.set_select("heat_mode_select", "Extern setpoint")
    openamber.set_number("manual_setpoint", target)
    openamber.set_climate("pid_heat_temperature_control", target_temperature=target)
    openamber.set_select("heat_compressor_mode", "Maximaal")
    openamber.set_number("compressor_start_delta_heating", 2.0)
    openamber.set_number("backup_heater_degmin_threshold", 9999.0)
    openamber.set_sensor("heat_cool_temperature_tc", tc)
    openamber.set_sensor("outlet_temperature_tuo", tc)
    openamber.set_sensor("inlet_temperature_tui", tc - 2.0)
    openamber.set_switch("heat_demand_switch", True)
    openamber.step(ms=50)


def request_dhw(openamber):
    """DHW demand: tank at 40 °C with setpoint 50 °C and restart delta 5 °C."""
    openamber.set_number("dhw_setpoint_temperature", 50.0)
    openamber.set_number("dhw_restart_dhw_delta", 5.0)
    openamber.set_sensor("dhw_temperature_tw_sensor", 40.0)
    openamber.step(ms=50)


def advance_without_compressor(openamber, seconds, state_entity, step=10):
    """Advance and check after every step that the compressor is not requested and no TO02 is raised."""
    waited = 0
    while waited < seconds:
        openamber.advance_time(seconds=step, step_s=step)
        openamber.step(ms=20)
        waited += step
        assert compressor_mode(openamber) == 0, (
            f"Compressor requested during the preheat after {waited} s (state: {openamber.get_entity(state_entity)})"
        )
        assert openamber.get_entity(ERROR) is False, f"TO02 raised during the preheat after {waited} s"


def count_start_attempts_until_error(openamber, max_s, step=5):
    """Advance until TO02 is raised. Returns the number of start commands (1999 going from 0 to > 0) seen."""
    attempts = 0
    commanded = compressor_mode(openamber) > 0
    if commanded:
        attempts = 1
    waited = 0
    while waited < max_s:
        if openamber.get_entity(ERROR) is True:
            return attempts
        openamber.advance_time(seconds=step, step_s=step)
        openamber.step(ms=20)
        waited += step
        now_commanded = compressor_mode(openamber) > 0
        if now_commanded and not commanded:
            attempts += 1
        commanded = now_commanded
    raise AssertionError(f"TO02 never raised within {max_s} s ({attempts} start attempts seen)")


def advance_until_compressor_runs(openamber, max_s, step=1):
    """Advance until the outdoor unit reports a compressor frequency above 0 Hz."""
    waited = 0
    while waited < max_s:
        frequency = openamber.get_entity("current_compressor_frequency")
        if frequency is not None and float(frequency) > 0:
            return True
        openamber.advance_time(seconds=step, step_s=step)
        openamber.step(ms=20)
        waited += step
    return False


def reset_errors(openamber):
    openamber.set_switch("reset_start_timeout_errors_switch", True)
    openamber.step(ms=50)


def cleanup(openamber):
    set_preheat(openamber, False)
    outdoor_unit_refuses_start(openamber, False)
    reset_errors(openamber)
    openamber.set_switch("heat_demand_switch", False)
    openamber.set_sensor("dhw_temperature_tw_sensor", 55.0)
    openamber.advance_time(seconds=30, step_s=5)
    openamber.step(ms=50)


def test_heating_waits_for_preheat_instead_of_start_timeout(clean_system):
    """P07 set for 30 min, heat demand 9 min in: no TO02, the compressor starts after P07 clears."""
    openamber = clean_system
    set_preheat(openamber, True)
    openamber.advance_time(seconds=9 * 60, step_s=30)
    openamber.step(ms=50)

    request_heating(openamber)
    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Pump running", max_s=1200, step=5), (
        f"Pump never started (state: {openamber.get_entity('state_machine_state_heat_cool')})"
    )
    # Pump settle time and minimum off time pass while the preheat goes on; the start stays held.
    advance_without_compressor(openamber, PREHEAT_S - 9 * 60 - 120, "state_machine_state_heat_cool")
    assert openamber.get_entity("state_machine_state_heat_cool") == "Pump running"

    set_preheat(openamber, False)
    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Compressor softstart", max_s=60, step=1), (
        f"Compressor did not start after the preheat (state: {openamber.get_entity('state_machine_state_heat_cool')})"
    )
    openamber.advance_time(seconds=10, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity(ERROR) is False
    cleanup(openamber)


def test_heating_genuine_no_start_retries_then_latches(clean_system):
    """Without P07 a start that times out is retried; TO02 is raised only after 3 failed starts in a row."""
    openamber = clean_system
    outdoor_unit_refuses_start(openamber, True)
    request_heating(openamber)
    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Wait compressor running", max_s=1800, step=5)

    # First timeout: no error, the start is retried.
    openamber.advance_time(seconds=START_TIMEOUT_S + 10, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity(ERROR) is False, "A single failed start must not raise TO02"
    assert compressor_mode(openamber) == 0
    assert openamber.get_entity("heat_demand_active_sensor") is True, "Heat demand must not be blocked after one failure"

    attempts = count_start_attempts_until_error(openamber, max_s=1800)
    assert attempts == MAX_ATTEMPTS - 1, f"Expected {MAX_ATTEMPTS - 1} more start attempts before TO02, saw {attempts}"
    assert openamber.get_entity("error_active") is True
    openamber.advance_time(seconds=30, step_s=5)
    openamber.step(ms=50)
    assert compressor_mode(openamber) == 0
    assert openamber.get_entity("heat_demand_active_sensor") is False, "TO02 blocks heat demand"
    cleanup(openamber)


def test_heating_successful_retry_clears_failure_count(clean_system):
    """Two failed starts followed by a successful one raise no error."""
    openamber = clean_system
    outdoor_unit_refuses_start(openamber, True)
    request_heating(openamber)
    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Wait compressor running", max_s=1800, step=5)
    openamber.advance_time(seconds=START_TIMEOUT_S + 10, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity(ERROR) is False

    # Two failures in a row, then the outdoor unit starts normally on the third attempt.
    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Wait compressor running", max_s=900, step=5)
    openamber.advance_time(seconds=START_TIMEOUT_S + 10, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity(ERROR) is False
    outdoor_unit_refuses_start(openamber, False)
    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Compressor softstart", max_s=900, step=5)
    assert openamber.get_entity(ERROR) is False
    cleanup(openamber)


def test_dhw_waits_for_preheat_instead_of_start_timeout(clean_system):
    """DHW: P07 set for 30 min, DHW demand 9 min in: no TO02, the compressor starts after P07 clears."""
    openamber = clean_system
    set_preheat(openamber, True)
    openamber.advance_time(seconds=9 * 60, step_s=30)
    openamber.step(ms=50)

    request_dhw(openamber)
    assert advance_until_state(openamber, "state_machine_state_dhw", "Pump running", max_s=600, step=5), (
        f"DHW pump never started (dhw: {openamber.get_entity('state_machine_state_dhw')})"
    )
    advance_without_compressor(openamber, PREHEAT_S - 9 * 60 - 120, "state_machine_state_dhw")
    assert openamber.get_entity("state_machine_state_dhw") == "Pump running"

    set_preheat(openamber, False)
    assert advance_until_compressor_runs(openamber, max_s=60), (
        f"DHW compressor did not start after the preheat (dhw: {openamber.get_entity('state_machine_state_dhw')})"
    )
    assert openamber.get_entity(ERROR) is False
    cleanup(openamber)


def test_dhw_genuine_no_start_retries_then_latches(clean_system):
    """DHW: without P07 a start that times out is retried; TO02 only after 3 failed starts in a row."""
    openamber = clean_system
    outdoor_unit_refuses_start(openamber, True)
    request_dhw(openamber)
    assert advance_until_state(openamber, "state_machine_state_dhw", "Wait compressor running", max_s=900, step=5)

    openamber.advance_time(seconds=START_TIMEOUT_S + 10, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity(ERROR) is False, "A single failed DHW start must not raise TO02"
    assert openamber.get_entity("dhw_demand_active_sensor") is True, "DHW demand must not be blocked after one failure"

    attempts = count_start_attempts_until_error(openamber, max_s=2400)
    assert attempts == MAX_ATTEMPTS - 1, f"Expected {MAX_ATTEMPTS - 1} more DHW start attempts before TO02, saw {attempts}"
    openamber.advance_time(seconds=30, step_s=5)
    openamber.step(ms=50)
    assert compressor_mode(openamber) == 0
    assert openamber.get_entity("dhw_demand_active_sensor") is False, "TO02 blocks DHW demand"
    cleanup(openamber)
