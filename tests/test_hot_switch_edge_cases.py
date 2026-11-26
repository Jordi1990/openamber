"""Edge cases around the hot switch between heating and DHW.

The hot switch moves the 3-way valve while the compressor and pump P0 keep running. These flows cover what can
happen around it: demand that changes during or right after the switch, a defrost while the valve moves, a missing
relay feedback, an ESP restart within the valve travel time, and the cases that must fall back to the regular
switch (compressor off first). FlowMonitor (helpers.py) checks the hardware rules after every step.
"""

from helpers import FlowMonitor, elapsed_s, now_s

DEFROST_STANDBY_HZ = 36.0  # actual defrost: compressor at the defrost standby frequency
VALVE_TIME_S = 60          # THREE_WAY_VALVE_SWITCH_TIME_S
MIN_ON_S = 600             # COMPRESSOR_MIN_ON_S


def _level(openamber):
    return int(openamber.get_entity("compressor_control_select") or 0)


def _heating_conditions(openamber, tc=28.0, target=35.0):
    """Heat demand with Tc below the start temperature (target - start delta)."""
    openamber.set_select("heat_mode_select", "Extern setpoint")
    openamber.set_number("manual_setpoint", target)
    openamber.set_climate("pid_heat_temperature_control", target_temperature=target)
    openamber.set_number("compressor_start_delta_heating", 2.0)
    openamber.set_number("backup_heater_degmin_threshold", 9999.0)
    openamber.set_sensor("current_water_temperature_tc_sensor", tc)
    openamber.set_sensor("heat_cool_temperature_tc", tc)
    openamber.set_sensor("outlet_temperature_tuo", tc)
    openamber.set_sensor("inlet_temperature_tui", tc - 2.0)


def _start_heating(openamber, p1=False):
    """Start space heating under a FlowMonitor, so it sees the compressor start (minimum on time)."""
    openamber.set_switch("pump_p1_enabled", p1)
    _heating_conditions(openamber)
    monitor = FlowMonitor(openamber)
    monitor.check()
    openamber.set_switch("heat_demand_switch", True)
    assert monitor.advance_until("state_machine_state_heat_cool", "Compressor running", 1800), (
        f"Heating never started (heat/cool: {openamber.get_entity('state_machine_state_heat_cool')})"
    )
    assert _level(openamber) > 0
    return monitor


def _hot_switch_to_dhw(openamber, monitor):
    """DHW demand while heating runs: the valve moves to DHW with the compressor running."""
    openamber.set_sensor("dhw_temperature_tw_sensor", 38.0)
    assert monitor.advance_until("state_machine_state_main", "DHW", 60), (
        f"No hot switch to DHW (main: {openamber.get_entity('state_machine_state_main')})"
    )
    assert openamber.get_entity("three_way_valve_dhw_switch") is True
    assert _level(openamber) > 0


def _dhw_heating(openamber, tc=30.0):
    """Temperatures while the compressor heats the tank."""
    openamber.set_sensor("current_water_temperature_tc_sensor", tc)
    openamber.set_sensor("heat_cool_temperature_tc", tc)
    openamber.set_sensor("outlet_temperature_tuo", 45.0)
    openamber.set_sensor("inlet_temperature_tui", 40.0)


def _end_dhw_with_hot_water(openamber):
    """Tank at setpoint; the hot DHW water still passes the unit's sensors."""
    openamber.set_sensor("outlet_temperature_tuo", 52.0)
    openamber.set_sensor("inlet_temperature_tui", 30.0)
    openamber.set_sensor("dhw_temperature_tw_sensor", 55.0)


def _hot_switch_back_to_heating(openamber, monitor):
    _end_dhw_with_hot_water(openamber)
    assert monitor.advance_until("state_machine_state_main", "Heat/Cool", 200), (
        f"No switch back to heating (main: {openamber.get_entity('state_machine_state_main')}, "
        f"dhw: {openamber.get_entity('state_machine_state_dhw')})"
    )
    assert openamber.get_entity("three_way_valve_heat_cool_switch") is True


def _advance_until_compressor_off(monitor, max_s):
    waited = 0
    while waited <= max_s:
        if _level(monitor.openamber) == 0:
            return True
        monitor.advance(5)
        waited += 5
    return False


def _valve_moves_with_compressor(history):
    """Valve moves in `history` while the compressor was requested in the sample before or after."""
    return [
        (prev, now) for prev, now in zip(history, history[1:])
        if prev["valve_dhw"] != now["valve_dhw"] and (prev["compressor"] > 0 or now["compressor"] > 0)
    ]


def _restart_esp(openamber, seconds=60):
    """Simulated ESP restart (live update): controller state is lost, the units keep running.
    Returns the compressor levels sampled every 5 s."""
    assert openamber.set_switch("mock_simulate_esp_restart", True)
    openamber.step(ms=50)
    levels = []
    for _ in range(seconds // 5):
        openamber.advance_time(seconds=5, step_s=5)
        openamber.step(ms=20)
        levels.append(_level(openamber))
    return levels


def _start_defrost(openamber):
    openamber.set_number("mock_compressor_frequency_override", DEFROST_STANDBY_HZ)
    openamber.set_binary_sensor("defrost_active_sensor", True)


def _end_defrost(openamber):
    openamber.set_binary_sensor("defrost_active_sensor", False)
    openamber.set_number("mock_compressor_frequency_override", -1.0)


# ---------------------------------------------------------------------------
# Demand changes around the switch
# ---------------------------------------------------------------------------

def test_dhw_ending_without_heat_demand_keeps_minimum_on_time_from_heating_start(clean_system):
    """DHW takes over a compressor that heated for 5 minutes, and the tank is full right away without heat demand.
    The compressor stops once it ran its minimum on time counted from the start in heating (not from the switch),
    and only then the valve returns."""
    openamber = clean_system
    monitor = _start_heating(openamber)
    started = monitor.compressor_running_since
    monitor.advance(300)

    _hot_switch_to_dhw(openamber, monitor)
    history_start = len(monitor.history)
    monitor.advance(30)
    openamber.set_switch("heat_demand_switch", False)
    openamber.set_sensor("dhw_temperature_tw_sensor", 55.0)

    assert _advance_until_compressor_off(monitor, 600), "The compressor never stopped"
    ran = elapsed_s(started, now_s(openamber))
    assert MIN_ON_S <= ran <= MIN_ON_S + 60, f"Compressor stopped {ran:.0f} s after the start in heating"
    assert monitor.advance_until("state_machine_state_main", "Heat/Cool", 300)
    assert openamber.get_entity("three_way_valve_dhw_switch") is False
    assert not _valve_moves_with_compressor(monitor.history[history_start:]), "Without heat demand: no hot switch back"


def test_heat_demand_ending_during_dhw_gives_a_regular_stop(clean_system):
    """Heat demand ends during a long DHW run. When the tank is full the compressor stops and the valve returns
    with the compressor off (no hot switch)."""
    openamber = clean_system
    monitor = _start_heating(openamber)
    _hot_switch_to_dhw(openamber, monitor)
    _dhw_heating(openamber)
    history_start = len(monitor.history)
    monitor.advance(120)
    openamber.set_switch("heat_demand_switch", False)
    monitor.advance(600)
    assert _level(openamber) > 0, "DHW keeps running without heat demand"

    openamber.set_sensor("dhw_temperature_tw_sensor", 55.0)
    assert _advance_until_compressor_off(monitor, 120), "DHW did not stop the compressor after the minimum on time"
    assert monitor.advance_until("state_machine_state_main", "Heat/Cool", 300)
    assert openamber.get_entity("three_way_valve_dhw_switch") is False
    assert not _valve_moves_with_compressor(monitor.history[history_start:]), "Without heat demand: no hot switch back"
    monitor.advance(300)
    assert _level(openamber) == 0, "No heat demand: the compressor stays off"


def test_hot_switch_to_heating_above_setpoint_stops_after_the_grace_time(clean_system):
    """DHW ends with heat demand while Tc is already above the stop temperature. Heating takes over the compressor,
    keeps it running during the 2 minute grace time after the switch and then stops on the overshoot."""
    openamber = clean_system
    monitor = _start_heating(openamber)
    _hot_switch_to_dhw(openamber, monitor)
    # Tc 40 °C: above target 35 + stop delta 2, below the DHW valve protection (35 + 2 + 5).
    _dhw_heating(openamber, tc=40.0)
    monitor.advance(700)
    assert openamber.get_entity("error_three_way_valve_state_timeout") is False

    _hot_switch_back_to_heating(openamber, monitor)
    switched = now_s(openamber)
    assert _level(openamber) > 0, "Heating takes over the running compressor"

    monitor.advance(60)
    openamber.set_sensor("outlet_temperature_tuo", 41.0)
    openamber.set_sensor("inlet_temperature_tui", 38.0)
    assert _advance_until_compressor_off(monitor, 400), "Heating never stopped on the overshoot"
    after = elapsed_s(switched, now_s(openamber))
    assert 110 <= after <= 300, f"Overshoot stop {after:.0f} s after the switch (grace time 120 s)"
    assert openamber.get_entity("error_water_delta_t_too_high") is False
    assert openamber.get_entity("error_active") is False


def test_switching_back_and_forth_within_the_valve_travel_time(clean_system):
    """DHW demand ends 20 s after the hot switch and returns 20 s after the switch back. Each switch restarts the
    valve wait: no relay feedback error, P1 never starts in between, the compressor never stops."""
    openamber = clean_system
    # The DHW pump only starts once the supply is warmer than the tank, so DHW stays in 'Compressor running'.
    openamber.set_select("dhw_pump_start_mode_select", "Aanvoer warmer dan vat (ΔT)")
    monitor = _start_heating(openamber, p1=True)
    assert openamber.get_entity("pump_p1_relay_switch") is True
    history_start = len(monitor.history)

    _hot_switch_to_dhw(openamber, monitor)
    monitor.advance(20)
    assert openamber.get_entity("state_machine_state_dhw") == "Compressor running"

    openamber.set_sensor("dhw_temperature_tw_sensor", 55.0)
    assert monitor.advance_until("state_machine_state_main", "Heat/Cool", 30)
    monitor.advance(20)
    assert openamber.get_entity("pump_p1_relay_switch") is False, "P1 waits for the valve travel time"

    openamber.set_sensor("dhw_temperature_tw_sensor", 38.0)
    assert monitor.advance_until("state_machine_state_main", "DHW", 30)
    monitor.advance(VALVE_TIME_S + 60)

    history = monitor.history[history_start - 1:]
    assert all(s["compressor"] > 0 for s in history), "The compressor must keep running"
    assert sum(1 for a, b in zip(history, history[1:]) if a["valve_dhw"] != b["valve_dhw"]) == 3
    assert not any(s["p1"] for s in history[1:]), "P1 must not start: the valve was never 60 s on CV"
    assert openamber.get_entity("three_way_valve_dhw_switch") is True
    assert openamber.get_entity("error_three_way_valve_state_timeout") is False
    assert openamber.get_entity("error_active") is False


def test_hot_switch_to_heating_lowers_the_compressor_level(clean_system):
    """After DHW, heating takes the compressor at once back from the DHW level to its own start level."""
    openamber = clean_system
    monitor = _start_heating(openamber)
    _hot_switch_to_dhw(openamber, monitor)
    _dhw_heating(openamber)
    monitor.advance(700)
    dhw_level = _level(openamber)

    _hot_switch_back_to_heating(openamber, monitor)
    heating_level = _level(openamber)
    assert 0 < heating_level < dhw_level, f"Heating level {heating_level} should be below the DHW level {dhw_level}"


# ---------------------------------------------------------------------------
# Relay feedback
# ---------------------------------------------------------------------------

def test_hot_switch_to_heating_missing_valve_feedback_raises_error(clean_system):
    """The switch back to heating also checks the valve relay feedback after the valve travel time."""
    openamber = clean_system
    monitor = _start_heating(openamber)
    _hot_switch_to_dhw(openamber, monitor)
    _dhw_heating(openamber)
    monitor.advance(VALVE_TIME_S + 30)
    assert openamber.get_entity("error_three_way_valve_state_timeout") is False

    assert openamber.set_switch("mock_three_way_valve_feedback_missing", True)
    _hot_switch_back_to_heating(openamber, monitor)
    monitor.advance(30)
    assert openamber.get_entity("error_three_way_valve_state_timeout") is False, "No error before the travel time"

    monitor.allow_short_run = True  # the error stops the compressor at once
    monitor.advance(VALVE_TIME_S)
    assert openamber.get_entity("error_three_way_valve_state_timeout") is True
    assert openamber.get_entity("error_active") is True


# ---------------------------------------------------------------------------
# Defrost
# ---------------------------------------------------------------------------

def test_dhw_demand_during_defrost_waits_for_the_defrost(clean_system):
    """DHW demand while heating defrosts: the valve stays on CV until the defrost has ended."""
    openamber = clean_system
    try:
        monitor = _start_heating(openamber)
        _start_defrost(openamber)
        assert monitor.advance_until("state_machine_state_heat_cool", "Defrosting", 30)

        openamber.set_sensor("dhw_temperature_tw_sensor", 38.0)
        history_start = len(monitor.history)
        monitor.advance(180)
        assert all(not s["valve_dhw"] for s in monitor.history[history_start:]), "Valve moved during the defrost"
        assert _level(openamber) > 0, "A defrost is never stopped"

        _end_defrost(openamber)
        assert monitor.advance_until("state_machine_state_main", "DHW", 1200), (
            f"Never switched to DHW after the defrost (heat/cool: {openamber.get_entity('state_machine_state_heat_cool')})"
        )
        assert openamber.get_entity("error_active") is False
    finally:
        _end_defrost(openamber)


def test_defrost_during_the_valve_travel_to_dhw(clean_system):
    """A defrost starts 10 s after the hot switch to DHW: DHW handles it without errors and continues afterwards."""
    openamber = clean_system
    try:
        monitor = _start_heating(openamber)
        _hot_switch_to_dhw(openamber, monitor)
        monitor.advance(10)
        _start_defrost(openamber)
        assert monitor.advance_until("state_machine_state_dhw", "Defrosting", 200), (
            f"DHW did not go to Defrosting (dhw: {openamber.get_entity('state_machine_state_dhw')})"
        )
        monitor.advance(VALVE_TIME_S)
        assert _level(openamber) > 0
        assert openamber.get_entity("state_machine_state_main") == "DHW"
        assert openamber.get_entity("error_active") is False

        _end_defrost(openamber)
        _dhw_heating(openamber)
        monitor.advance(600)
        assert openamber.get_entity("state_machine_state_main") == "DHW"
        assert openamber.get_entity("state_machine_state_dhw") != "Defrosting"
        assert _level(openamber) > 0
        assert openamber.get_entity("error_active") is False
    finally:
        _end_defrost(openamber)


def test_defrost_during_the_valve_travel_to_heating(clean_system):
    """A defrost starts 10 s after the hot switch back to heating: P1 stays off during the defrost and starts once
    it has ended; no delta-T error from the hot DHW water."""
    openamber = clean_system
    try:
        monitor = _start_heating(openamber, p1=True)
        _hot_switch_to_dhw(openamber, monitor)
        _dhw_heating(openamber)
        monitor.advance(700)

        _hot_switch_back_to_heating(openamber, monitor)
        monitor.advance(10)
        _start_defrost(openamber)
        assert monitor.advance_until("state_machine_state_heat_cool", "Defrosting", 30)
        monitor.advance(90)
        openamber.set_sensor("outlet_temperature_tuo", 32.0)
        openamber.set_sensor("inlet_temperature_tui", 29.0)
        monitor.advance(60)
        assert openamber.get_entity("pump_p1_relay_switch") is False, "P1 must be off during the defrost"
        assert _level(openamber) > 0

        _end_defrost(openamber)
        assert monitor.advance_until("pump_p1_relay_switch", True, 60), "P1 did not start after the defrost"
        assert openamber.get_entity("error_water_delta_t_too_high") is False
        assert openamber.get_entity("error_active") is False
    finally:
        _end_defrost(openamber)


# ---------------------------------------------------------------------------
# Fall back to the regular switch
# ---------------------------------------------------------------------------

def test_dhw_ending_with_backup_heater_on_switches_it_off_before_the_valve_moves(clean_system):
    """The backup heater helps DHW when the tank heats too slowly. When DHW ends with heat demand, the backup heater
    is switched off first; the valve only moves with the backup heater off (FlowMonitor checks this at every step)."""
    openamber = clean_system
    try:
        openamber.set_number("dhw_backup_min_avg_rate", 0.5)
        openamber.set_number("dhw_backup_min_avg_rate_delay_minutes", 5.0)
        monitor = _start_heating(openamber)
        _hot_switch_to_dhw(openamber, monitor)
        _dhw_heating(openamber)
        assert monitor.advance_until("backup_heater_relay", True, 1800), "The DHW backup heater never turned on"
        assert openamber.get_entity("three_way_valve_dhw_switch") is True

        openamber.set_sensor("dhw_temperature_tw_sensor", 55.0)
        assert monitor.advance_until("state_machine_state_main", "Heat/Cool", 900)
        assert openamber.get_entity("three_way_valve_heat_cool_switch") is True
        assert openamber.get_entity("backup_heater_relay") is False
        assert openamber.get_entity("error_active") is False
    finally:
        openamber.set_number("dhw_backup_min_avg_rate", 0.12)
        openamber.set_number("dhw_backup_min_avg_rate_delay_minutes", 5.0)


def test_error_during_dhw_stops_instead_of_hot_switch(clean_system):
    """An error during DHW with heat demand: everything stops, there is no hot switch back to heating."""
    openamber = clean_system
    monitor = _start_heating(openamber)
    _hot_switch_to_dhw(openamber, monitor)
    _dhw_heating(openamber)
    monitor.advance(120)
    history_start = len(monitor.history)

    monitor.allow_short_run = True  # the error stops the compressor at once
    openamber.set_binary_sensor("error_compressor_start_timeout", True)
    openamber.set_sensor("dhw_temperature_tw_sensor", 55.0)
    assert _advance_until_compressor_off(monitor, 30), "The error must stop the compressor"
    assert not _valve_moves_with_compressor(monitor.history[history_start - 1:])
    # Not under FlowMonitor from here: the regular safety stop moves the valve back while the compressor still runs
    # down (frequency > 0), which is outside the scope of the hot switch.
    for _ in range(60):
        openamber.advance_time(seconds=5, step_s=5)
        openamber.step(ms=20)
        assert _level(openamber) == 0, "No restart and no hot switch after the error"
    assert openamber.get_entity("state_machine_state_heat_cool") != "Compressor running"


# ---------------------------------------------------------------------------
# ESP restart (live update) within the valve travel time
# ---------------------------------------------------------------------------

def test_restart_right_after_hot_switch_to_dhw_resumes_dhw(clean_system):
    """The ESP restarts 20 s after the hot switch to DHW: DHW is taken over with the compressor running."""
    openamber = clean_system
    monitor = _start_heating(openamber)
    _hot_switch_to_dhw(openamber, monitor)
    monitor.advance(20)

    levels = _restart_esp(openamber)
    assert all(level > 0 for level in levels), f"Compressor stopped during the restart: {levels}"
    assert openamber.get_entity("state_machine_state_main") == "DHW"
    assert openamber.get_entity("three_way_valve_dhw_switch") is True
    assert openamber.get_entity("error_active") is False


def test_restart_right_after_hot_switch_to_heating_resumes_heating(clean_system):
    """The ESP restarts 20 s after the hot switch back to heating: heating is taken over with the compressor
    running and P1 on."""
    openamber = clean_system
    monitor = _start_heating(openamber, p1=True)
    _hot_switch_to_dhw(openamber, monitor)
    _dhw_heating(openamber)
    monitor.advance(700)
    _hot_switch_back_to_heating(openamber, monitor)
    openamber.set_sensor("outlet_temperature_tuo", 32.0)
    openamber.set_sensor("inlet_temperature_tui", 29.0)
    monitor.advance(20)

    levels = _restart_esp(openamber, seconds=90)
    assert all(level > 0 for level in levels), f"Compressor stopped during the restart: {levels}"
    assert openamber.get_entity("state_machine_state_main") == "Heat/Cool"
    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"
    assert openamber.get_entity("pump_p1_relay_switch") is True
    assert openamber.get_entity("error_active") is False
