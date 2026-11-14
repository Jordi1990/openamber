"""Up-step guard: only step the compressor up when the current power will not reach the setpoint in time."""

import pytest

SETPOINT = 40.0
TC_START = 30.0


def _set_tc(openamber, tc):
    openamber.set_sensor("current_water_temperature_tc_sensor", tc)
    openamber.set_sensor("heat_cool_temperature_tc", tc)
    openamber.set_sensor("outlet_temperature_tuo", tc + 2.0)
    openamber.set_sensor("inlet_temperature_tui", tc - 1.0)


def _start_compressor(openamber, lookahead_min):
    openamber.set_select("thermostat_mode_select", "Extern")
    openamber.set_select("heat_mode_select", "Extern setpoint")
    openamber.set_number("manual_setpoint", SETPOINT)
    openamber.set_climate("pid_heat_temperature_control", target_temperature=SETPOINT)
    openamber.set_number("backup_heater_degmin_threshold", 999.0)
    openamber.set_number("compressor_up_step_lookahead_minutes", lookahead_min)
    openamber.set_select("heat_compressor_mode", "Maximaal")
    openamber.set_switch("heat_demand_switch", True)
    _set_tc(openamber, TC_START)
    openamber.step(ms=50)
    for _ in range(180):
        openamber.advance_time(seconds=10, step_s=5)
        if openamber.get_entity("state_machine_state_heat_cool") == "Compressor softstart":
            return
    pytest.fail("Compressor did not start")


def _ramp(openamber, tc, slope_c_per_min, seconds):
    for _ in range(int(seconds / 30)):
        tc += slope_c_per_min * 0.5
        _set_tc(openamber, tc)
        openamber.advance_time(seconds=30, step_s=5)
    return tc


@pytest.mark.parametrize("lookahead_min,slope,expect_step_up", [
    (0, 0.6, True),    # default 0 = off: PID demand steps the compressor up as before
    (20, 0.6, False),  # Tc rises fast enough to reach the setpoint within 20 min: hold the mode
    (20, 0.1, True),   # Tc rises too slowly: step up as before
])
def test_up_step_guard(clean_system, lookahead_min, slope, expect_step_up):
    openamber = clean_system
    _start_compressor(openamber, lookahead_min)

    tc = _ramp(openamber, TC_START, slope, 200)  # through the 3 min soft start
    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"
    start_mode = int(openamber.get_entity("compressor_control_select"))

    tc = _ramp(openamber, tc, slope, 10 * 60)
    assert tc < SETPOINT - 1.0, "Tc must stay below the deadband so the PID keeps asking for more power"
    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"
    pid = float(openamber.get_entity("PID Controller Verwarmen"))
    assert pid > 90.0, f"PID should demand (near) full power, got {pid}%"
    mode = int(openamber.get_entity("compressor_control_select"))

    if expect_step_up:
        assert mode > start_mode, f"Compressor should step up from mode {start_mode}, stayed at {mode}"
    else:
        assert mode == start_mode, f"Compressor should hold mode {start_mode} while Tc is on track, got {mode}"

    openamber.set_number("compressor_up_step_lookahead_minutes", 0)
    openamber.set_switch("heat_demand_switch", False)
    openamber.advance_time(seconds=60, step_s=5)


def test_up_step_guard_default_is_off(clean_system):
    assert float(clean_system.get_entity("compressor_up_step_lookahead_minutes")) == 0.0


def test_up_step_guard_restarts_trend_when_control_source_changes(clean_system):
    """Switching the control temperature from Tc to Tv1 during a run restarts the trend: a window that mixes the
    rising Tc with a higher, flat Tv1 would show a false rise and keep holding the step up."""
    openamber = clean_system
    _start_compressor(openamber, 20)
    tc = _ramp(openamber, TC_START, 0.6, 200)
    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"
    start_mode = int(openamber.get_entity("compressor_control_select"))
    tc = _ramp(openamber, tc, 0.6, 6 * 60)  # window full, guard holds the mode
    assert int(openamber.get_entity("compressor_control_select")) == start_mode

    openamber.set_sensor("heat_cool_temperature_tv1", SETPOINT - 1.5)  # flat, below the deadband
    openamber.set_select("heat_cool_control_temperature_source_select", "CV-aanvoer zone 1 (Tv1)")
    stepped_up = False
    for _ in range(8):  # 4 min, shorter than the 4.5 min trend window
        openamber.advance_time(seconds=30, step_s=5)
        openamber.set_sensor("heat_cool_temperature_tv1", SETPOINT - 1.5)
        if int(openamber.get_entity("compressor_control_select")) > start_mode:
            stepped_up = True
            break
    assert stepped_up, "After the source change the guard must not hold the step up on a mixed trend"

    openamber.set_select("heat_cool_control_temperature_source_select", "CV-aanvoer (Tc)")
    openamber.set_number("compressor_up_step_lookahead_minutes", 0)
    openamber.set_switch("heat_demand_switch", False)
    openamber.advance_time(seconds=60, step_s=5)
