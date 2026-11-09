"""Live update: take over running units after a restart of the ESP instead of stopping them.

During an OTA update the ESP reboots while the inside and outside unit keep running on the last Modbus
commands. After the reboot the main state machine checks the readback registers (RESUME_CHECK) and takes
over a running pump/compressor, instead of stopping everything in INITIALIZING.

`mock_simulate_esp_restart` drops all controller state as if the ESP restarted; the simulated units keep
their state, just like the real units during an OTA update.
"""

import pytest

from helpers import advance_until_state, start_space_heating

RESUMED_MAIN_STATES = ("Heat/Cool", "DHW")
INIT_STATES = ("Initializing", "Waiting for state switch")
COMPRESSOR_MIN_ON_S = 10 * 60   # constants.h
COMPRESSOR_MIN_OFF_S = 2 * 60   # constants.h
OTA_HOLD_TIMEOUT_S = 2 * 60     # constants.h
MODBUS_CONNECTION_TIMEOUT_S = 2 * 60  # constants.h
DEFROST_STANDBY_HZ = 36.0       # actual defrost: compressor at the defrost standby frequency


def _restart_and_sample(openamber, seconds, step=5):
    """Simulate an ESP restart and sample the unit state every `step` seconds."""
    assert openamber.set_switch("mock_simulate_esp_restart", True)
    openamber.step(ms=50)
    assert openamber.get_entity("state_machine_state_main") == "Waiting for Modbus connection"

    samples = []
    waited = 0
    while waited < seconds:
        openamber.advance_time(seconds=step, step_s=step)
        openamber.step(ms=20)
        waited += step
        samples.append({
            "main": openamber.get_entity("state_machine_state_main"),
            "hc": openamber.get_entity("state_machine_state_heat_cool"),
            "dhw": openamber.get_entity("state_machine_state_dhw"),
            "level": int(openamber.get_entity("compressor_control_select") or 0),
            "mode": openamber.get_entity("working_mode_switch"),
            "pump": openamber.get_entity("internal_pump_active"),
        })
    return samples


def _resumed_sample(samples):
    """The sample of the update that took over the units: the first main controller state after 'Checking running units'.

    Does not require the two to be in adjacent samples, so it also holds when the resume check waits a few updates.
    """
    checked = False
    for sample in samples:
        if sample["main"] == "Checking running units":
            checked = True
        elif checked and sample["main"] in RESUMED_MAIN_STATES:
            return sample
        elif checked and sample["main"] in INIT_STATES:
            break
    raise AssertionError(f"Units were not taken over after the restart: {samples}")


def _assert_not_initialized(samples):
    assert all(s["main"] != "Initializing" for s in samples), f"Restart must not run the stopping initialization: {samples}"
    assert all(s["pump"] is True for s in samples), f"Pump stopped during the restart: {samples}"


def _assert_initialized(samples):
    assert any(s["main"] in INIT_STATES for s in samples), f"Expected a normal initialization: {samples}"
    assert samples[-1]["level"] == 0, f"Initialization must stop the compressor: {samples}"


def _start_dhw(openamber):
    openamber.set_number("dhw_setpoint_temperature", 50.0)
    openamber.set_number("dhw_restart_dhw_delta", 5.0)
    openamber.set_sensor("dhw_temperature_tw_sensor", 40.0)
    openamber.step(ms=50)
    assert advance_until_state(openamber, "state_machine_state_dhw", "Compressor running", max_s=600, step=5)
    assert openamber.get_entity("three_way_valve_dhw_switch") is True


def _start_cooling(openamber):
    openamber.set_switch("heat_demand_switch", False)
    openamber.set_switch("cool_demand_switch", True)
    openamber.set_sensor("current_water_temperature_tc_sensor", 25.0)
    openamber.set_sensor("heat_cool_temperature_tc", 25.0)  # > 18°C target + start delta
    openamber.set_sensor("outlet_temperature_tuo", 25.0)
    openamber.set_sensor("inlet_temperature_tui", 25.0)
    openamber.step(ms=50)
    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Compressor running", max_s=1800)
    assert openamber.get_entity("working_mode_switch") == "Koelen"


# ---------------------------------------------------------------------------
# Heat/cool: taken over
# ---------------------------------------------------------------------------

def test_running_heating_is_resumed_after_restart(clean_system):
    openamber = clean_system
    start_space_heating(openamber, tc=28.0, target=35.0)
    openamber.advance_time(seconds=30, step_s=5)
    assert int(openamber.get_entity("compressor_control_select") or 0) > 0

    samples = _restart_and_sample(openamber, seconds=40)

    _assert_not_initialized(samples)
    assert _resumed_sample(samples)["hc"] == "Compressor running"
    assert all(s["level"] > 0 for s in samples), f"Compressor was stopped during the restart: {samples}"
    assert all(s["mode"] == "Verwarmen" for s in samples), f"Working mode changed during the restart: {samples}"
    assert openamber.get_entity("state_machine_state_main") == "Heat/Cool"
    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"


def test_running_cooling_is_resumed_after_restart(clean_system):
    openamber = clean_system
    try:
        _start_cooling(openamber)

        samples = _restart_and_sample(openamber, seconds=40)

        _assert_not_initialized(samples)
        assert _resumed_sample(samples)["hc"] == "Compressor running"
        assert all(s["level"] > 0 for s in samples), f"Compressor was stopped during the restart: {samples}"
        assert all(s["mode"] == "Koelen" for s in samples), f"Working mode changed during the restart: {samples}"
        assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"
    finally:
        openamber.set_switch("cool_demand_switch", False)


def test_resumed_heating_keeps_minimum_on_time(clean_system):
    """The run time before the restart is unknown, so a resumed compressor runs at least the minimum on time."""
    openamber = clean_system
    start_space_heating(openamber, tc=28.0, target=35.0)
    # Run past the minimum on time first, so only the restart can explain a held compressor.
    openamber.advance_time(seconds=COMPRESSOR_MIN_ON_S + 30, step_s=10)
    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"

    samples = _restart_and_sample(openamber, seconds=30)
    resumed_index = samples.index(_resumed_sample(samples))
    # Elapsed time since the takeover at the end of the sampling (samples are 5 s apart, takeover at the end of its sample).
    elapsed = (len(samples) - resumed_index - 1) * 5
    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"

    # Overshoot: would stop at once without minimum on time.
    openamber.set_sensor("heat_cool_temperature_tc", 40.0)
    openamber.set_sensor("outlet_temperature_tuo", 40.0)
    openamber.set_sensor("inlet_temperature_tui", 38.0)

    # The compressor stays on for the whole minimum on time after the takeover. Margin: the simulator clock also
    # advances with the real time the test loop takes (a few seconds over ~120 steps).
    while elapsed < COMPRESSOR_MIN_ON_S - 30:
        openamber.advance_time(seconds=5, step_s=5)
        openamber.step(ms=20)
        elapsed += 5
        assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running", f"Stopped {elapsed} s after the takeover"
        assert int(openamber.get_entity("compressor_control_select") or 0) > 0, f"Compressor stopped {elapsed} s after the takeover"

    # After the minimum on time the overshoot stops the compressor.
    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Wait compressor stop", max_s=120, step=5) or \
        int(openamber.get_entity("compressor_control_select") or 0) == 0


def test_defrost_is_resumed_after_restart(clean_system):
    """Actual defrost (compressor at the standby frequency): taken over as DEFROSTING with P1 off."""
    openamber = clean_system
    try:
        openamber.set_switch("pump_p1_enabled", True)
        start_space_heating(openamber, tc=28.0, target=35.0)
        openamber.set_number("mock_compressor_frequency_override", DEFROST_STANDBY_HZ)
        openamber.set_binary_sensor("defrost_active_sensor", True)
        openamber.advance_time(seconds=10, step_s=2)
        # P1 left on, e.g. a defrost that started during the OTA hold.
        openamber.set_switch("pump_p1_relay_switch", True)

        samples = _restart_and_sample(openamber, seconds=40)

        _assert_not_initialized(samples)
        assert _resumed_sample(samples)["hc"] == "Defrosting"
        assert all(s["level"] > 0 for s in samples), f"Compressor was stopped during the defrost: {samples}"
        assert all(s["mode"] == "Verwarmen" for s in samples), f"Working mode changed during the defrost: {samples}"
        assert openamber.get_entity("state_machine_state_heat_cool") == "Defrosting"
        assert openamber.get_entity("pump_p1_relay_switch") is False, "P1 must be off during a resumed defrost"
    finally:
        openamber.set_binary_sensor("defrost_active_sensor", False)
        openamber.set_number("mock_compressor_frequency_override", -1.0)


def test_defrost_prephase_is_resumed_as_compressor_running(clean_system):
    """Restart in the defrost pre-phase: keep heating, stop P1 once the actual defrost starts."""
    openamber = clean_system
    try:
        openamber.set_switch("pump_p1_enabled", True)
        start_space_heating(openamber, tc=28.0, target=35.0)
        assert openamber.get_entity("pump_p1_relay_switch") is True
        # Pre-phase: 2118 set, compressor still at its normal frequency.
        openamber.set_number("mock_compressor_frequency_override", 70.0)
        openamber.set_binary_sensor("defrost_active_sensor", True)
        openamber.advance_time(seconds=10, step_s=2)
        assert openamber.get_entity("defrost_running_sensor") is False

        samples = _restart_and_sample(openamber, seconds=30)

        _assert_not_initialized(samples)
        assert _resumed_sample(samples)["hc"] == "Compressor running"
        assert openamber.get_entity("pump_p1_relay_switch") is True, "P1 keeps running during the pre-phase"

        # Actual defrost starts: the normal transition stops P1.
        openamber.set_number("mock_compressor_frequency_override", DEFROST_STANDBY_HZ)
        assert advance_until_state(openamber, "state_machine_state_heat_cool", "Defrosting", max_s=30, step=5)
        assert openamber.get_entity("pump_p1_relay_switch") is False, "P1 must stop when the actual defrost starts"
    finally:
        openamber.set_binary_sensor("defrost_active_sensor", False)
        openamber.set_number("mock_compressor_frequency_override", -1.0)


def test_p1_is_restored_when_resuming_heating(clean_system):
    """A defrost that ended during the OTA hold leaves P1 off: the takeover starts it again on demand."""
    openamber = clean_system
    openamber.set_switch("pump_p1_enabled", True)
    start_space_heating(openamber, tc=28.0, target=35.0)
    openamber.set_switch("pump_p1_relay_switch", False)

    samples = _restart_and_sample(openamber, seconds=30)

    _assert_not_initialized(samples)
    assert _resumed_sample(samples)["hc"] == "Compressor running"
    assert openamber.get_entity("pump_p1_relay_switch") is True, "P1 must run again after the takeover"


def test_cooling_resume_opens_mixing_valves(clean_system):
    """Cooling runs with the enabled mixing valves fully open; the takeover reapplies that."""
    openamber = clean_system
    try:
        openamber.set_switch("mixing_valve_zone1_enabled", True)
        openamber.set_number("mixing_valve_zone1_max_position", 80.0)
        _start_cooling(openamber)
        assert int(openamber.get_entity("mixing_valve_zone1_output")) == 80
        # Valve not at its cooling position when the ESP restarts.
        openamber.set_number("mixing_valve_zone1_output", 20.0)

        samples = _restart_and_sample(openamber, seconds=30)

        _assert_not_initialized(samples)
        assert _resumed_sample(samples)["hc"] == "Compressor running"
        assert int(openamber.get_entity("mixing_valve_zone1_output")) == 80, "Mixing valve must be opened on a resumed cooling cycle"
    finally:
        openamber.set_switch("cool_demand_switch", False)
        openamber.set_switch("mixing_valve_zone1_enabled", False)


def test_starting_compressor_is_cancelled_after_restart(clean_system):
    """Level set but compressor not running yet: the start is cancelled, the pump keeps running."""
    openamber = clean_system
    try:
        start_space_heating(openamber, tc=28.0, target=35.0)
        openamber.set_number("mock_compressor_frequency_override", 0.0)

        samples = _restart_and_sample(openamber, seconds=40)

        _assert_not_initialized(samples)
        assert _resumed_sample(samples)["hc"] == "Wait compressor stop"
        assert samples[-1]["level"] == 0, "The pending compressor start must be cancelled"
        assert openamber.get_entity("state_machine_state_main") == "Heat/Cool"
        assert openamber.get_entity("state_machine_state_heat_cool") == "Pump running"
    finally:
        openamber.set_number("mock_compressor_frequency_override", -1.0)


def test_stopping_compressor_is_finished_after_restart(clean_system):
    """Level 0 but compressor still running down: wait for the stop, the pump keeps running."""
    openamber = clean_system
    try:
        start_space_heating(openamber, tc=28.0, target=35.0)
        openamber.set_number("mock_compressor_frequency_override", 40.0)
        openamber.set_select("compressor_control_select", "0")

        samples = _restart_and_sample(openamber, seconds=30)

        _assert_not_initialized(samples)
        assert _resumed_sample(samples)["hc"] == "Wait compressor stop"
        assert openamber.get_entity("state_machine_state_heat_cool") == "Wait compressor stop"

        # Compressor has run down: continue with the pump cycle.
        openamber.set_number("mock_compressor_frequency_override", -1.0)
        assert advance_until_state(openamber, "state_machine_state_heat_cool", "Pump running", max_s=30, step=5)
        assert openamber.get_entity("internal_pump_active") is True
    finally:
        openamber.set_number("mock_compressor_frequency_override", -1.0)


def test_pump_run_on_after_compressor_stop_is_resumed_after_restart(clean_system):
    """Pump run-on after a compressor stop: working mode is still heating, the pump cycle continues."""
    openamber = clean_system
    start_space_heating(openamber, tc=28.0, target=35.0)
    # Compressor stopped and run down, pump still running, working mode not yet back to standby.
    openamber.set_select("compressor_control_select", "0")
    openamber.step(ms=50)
    assert openamber.get_entity("working_mode_switch") == "Verwarmen"

    samples = _restart_and_sample(openamber, seconds=30)

    _assert_not_initialized(samples)
    assert _resumed_sample(samples)["hc"] == "Pump running"
    assert all(s["level"] == 0 for s in samples), f"Compressor must not start right after the restart: {samples}"
    assert openamber.get_entity("state_machine_state_heat_cool") == "Pump running"


def test_pump_cycle_before_compressor_start_falls_back_to_initialization(clean_system):
    """Pump interval before a compressor start runs in standby: not taken over, the pump restarts after initialization."""
    openamber = clean_system
    openamber.set_climate("pid_heat_temperature_control", target_temperature=30.0)
    openamber.set_sensor("heat_cool_temperature_tc", 35.0)
    openamber.set_sensor("outlet_temperature_tuo", 35.0)
    openamber.set_sensor("inlet_temperature_tui", 33.0)
    openamber.set_switch("heat_demand_switch", True)
    openamber.step(ms=50)
    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Pump running", max_s=1800, step=10)
    assert openamber.get_entity("working_mode_switch") == "Standby"

    samples = _restart_and_sample(openamber, seconds=30)

    assert any(s["main"] == "Checking running units" for s in samples), samples
    _assert_initialized(samples)
    # Initialization resets the pump interval: the pump starts again right away on the pump demand.
    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Pump running", max_s=180, step=5)


def test_backup_heater_is_switched_off_on_resume(clean_system):
    """The backup heater timing is lost with the restart: switched off, the controller enables it again when needed."""
    openamber = clean_system
    start_space_heating(openamber, tc=28.0, target=35.0)
    openamber.set_switch("backup_heater_relay", True)

    samples = _restart_and_sample(openamber, seconds=30)

    _assert_not_initialized(samples)
    assert openamber.get_entity("backup_heater_relay") is False
    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"


def test_backup_heater_stages_are_switched_off_on_resume(clean_system):
    """After a reboot backup_heater_relay is already off and does not forward an off: the stages are switched off directly."""
    openamber = clean_system
    start_space_heating(openamber, tc=28.0, target=35.0)
    assert openamber.get_entity("backup_heater_relay") is False
    openamber.set_switch("backup_heater_stage_1", True)
    openamber.set_switch("backup_heater_stage_2", True)

    samples = _restart_and_sample(openamber, seconds=30)

    _assert_not_initialized(samples)
    assert openamber.get_entity("backup_heater_stage_1") is False
    assert openamber.get_entity("backup_heater_stage_2") is False


# ---------------------------------------------------------------------------
# DHW: taken over
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("dhw_pump_on, expected", [
    (True, "Wait DHW pump running"),
    (False, "Compressor running"),
])
def test_running_dhw_is_resumed_after_restart(clean_system, dhw_pump_on, expected):
    openamber = clean_system
    _start_dhw(openamber)
    openamber.set_switch("dhw_pump_relay_switch", dhw_pump_on)

    samples = _restart_and_sample(openamber, seconds=40)

    _assert_not_initialized(samples)
    resumed = _resumed_sample(samples)
    assert resumed["main"] == "DHW"
    assert resumed["dhw"] == expected
    assert all(s["level"] > 0 for s in samples), f"Compressor was stopped during the restart: {samples}"
    assert openamber.get_entity("dhw_active") is True
    assert openamber.get_entity("three_way_valve_dhw_switch") is True


def test_dhw_defrost_is_resumed_after_restart(clean_system):
    openamber = clean_system
    try:
        _start_dhw(openamber)
        openamber.set_number("mock_compressor_frequency_override", DEFROST_STANDBY_HZ)
        openamber.set_binary_sensor("defrost_active_sensor", True)
        openamber.advance_time(seconds=10, step_s=2)

        samples = _restart_and_sample(openamber, seconds=30)

        _assert_not_initialized(samples)
        assert _resumed_sample(samples)["dhw"] == "Defrosting"
        assert openamber.get_entity("state_machine_state_dhw") == "Defrosting"
    finally:
        openamber.set_binary_sensor("defrost_active_sensor", False)
        openamber.set_number("mock_compressor_frequency_override", -1.0)


def test_dhw_resumed_defrost_restarts_heating_rate_window(clean_system):
    """DHW pump on during a resumed defrost: after the defrost the pump settle time and rate window start again."""
    openamber = clean_system
    try:
        _start_dhw(openamber)
        openamber.set_switch("dhw_pump_relay_switch", True)
        openamber.set_number("mock_compressor_frequency_override", DEFROST_STANDBY_HZ)
        openamber.set_binary_sensor("defrost_active_sensor", True)
        openamber.advance_time(seconds=10, step_s=2)

        samples = _restart_and_sample(openamber, seconds=30)
        assert _resumed_sample(samples)["dhw"] == "Defrosting"

        openamber.set_binary_sensor("defrost_active_sensor", False)
        openamber.set_number("mock_compressor_frequency_override", -1.0)
        states = []
        for _ in range(0, 300 + 120 + 60, 5):  # settle after defrost + DHW pump settle + margin
            openamber.advance_time(seconds=5, step_s=5)
            openamber.step(ms=20)
            states.append(openamber.get_entity("state_machine_state_dhw"))
        assert "Wait DHW pump running" in states, f"Rate window not restarted after the resumed defrost: {states}"
        assert "DHW pump settled" in states, f"Rate window not restarted after the resumed defrost: {states}"
    finally:
        openamber.set_binary_sensor("defrost_active_sensor", False)
        openamber.set_number("mock_compressor_frequency_override", -1.0)


def test_dhw_starting_compressor_is_cancelled_after_restart(clean_system):
    """DHW, level set but compressor not running: the start is cancelled and the cycle stops via the normal path."""
    openamber = clean_system
    try:
        _start_dhw(openamber)
        openamber.set_number("mock_compressor_frequency_override", 0.0)

        samples = _restart_and_sample(openamber, seconds=30)

        assert all(s["main"] != "Initializing" for s in samples), samples
        resumed = _resumed_sample(samples)
        assert resumed["main"] == "DHW"
        assert resumed["dhw"] == "Wait compressor stop"
        assert samples[-1]["level"] == 0, "The pending compressor start must be cancelled"

        # The DHW demand continues: the compressor may only start again after the minimum off time.
        openamber.set_number("mock_compressor_frequency_override", -1.0)
        resumed_index = samples.index(resumed)
        elapsed = (len(samples) - resumed_index - 1) * 5
        while elapsed < COMPRESSOR_MIN_OFF_S - 20:
            openamber.advance_time(seconds=5, step_s=5)
            openamber.step(ms=20)
            elapsed += 5
            assert int(openamber.get_entity("compressor_control_select") or 0) == 0, \
                f"DHW compressor restarted {elapsed} s after the cancelled start (minimum off time)"
        assert advance_until_state(openamber, "state_machine_state_dhw", "Compressor running", max_s=600, step=5)
    finally:
        openamber.set_number("mock_compressor_frequency_override", -1.0)


# ---------------------------------------------------------------------------
# Not taken over: normal initialization
# ---------------------------------------------------------------------------

def test_inside_unit_failsafe_falls_back_to_initialization(clean_system):
    """Pump P0 relay read as off: the inside unit restarted too, nothing to take over."""
    openamber = clean_system
    start_space_heating(openamber, tc=28.0, target=35.0)

    assert openamber.set_switch("pump_p0_relay_switch", False)
    samples = _restart_and_sample(openamber, seconds=30)

    _assert_initialized(samples)
    assert all(s["main"] != "Checking running units" for s in samples), "P0 off must skip the resume check"
    assert openamber.get_entity("pump_p0_relay_switch") is True, "Initialization must switch P0 relay back on"
    assert openamber.get_entity("error_inside_unit_failsafe") is False


def test_idle_system_initializes_normally_after_restart(clean_system):
    openamber = clean_system
    openamber.advance_time(seconds=30, step_s=5)
    assert openamber.get_entity("internal_pump_active") is False

    samples = _restart_and_sample(openamber, seconds=60)

    assert any(s["main"] in INIT_STATES for s in samples), samples
    assert advance_until_state(openamber, "state_machine_state_main", "Heat/Cool", max_s=120, step=5)


def _set_working_mode_standby(openamber):
    openamber.set_select("working_mode_switch", "Standby")


def _set_both_valve_outputs(openamber):
    openamber.set_switch("three_way_valve_dhw_switch", True)
    openamber.set_switch("three_way_valve_heat_cool_switch", True)


def _set_valve_feedback_missing(openamber):
    openamber.set_switch("mock_three_way_valve_feedback_missing", True)


def _set_dhw_position_while_cooling(openamber):
    openamber.set_switch("three_way_valve_dhw_switch", True)
    openamber.set_switch("three_way_valve_heat_cool_switch", False)
    openamber.set_select("working_mode_switch", "Koelen")


def _set_error_active(openamber):
    openamber.set_binary_sensor("error_compressor_start_timeout", True)


@pytest.mark.parametrize("make_inconsistent", [
    _set_working_mode_standby,
    _set_both_valve_outputs,
    _set_valve_feedback_missing,
    _set_dhw_position_while_cooling,
    _set_error_active,
], ids=["working_mode_standby", "both_valve_outputs", "valve_feedback_missing", "dhw_position_while_cooling", "error_active"])
def test_inconsistent_state_falls_back_to_initialization(clean_system, make_inconsistent):
    openamber = clean_system
    try:
        start_space_heating(openamber, tc=28.0, target=35.0)
        make_inconsistent(openamber)
        openamber.step(ms=50)

        samples = _restart_and_sample(openamber, seconds=30)

        assert any(s["main"] == "Checking running units" for s in samples), samples
        _assert_initialized(samples)
    finally:
        openamber.set_switch("mock_three_way_valve_feedback_missing", False)
        openamber.set_switch("reset_start_timeout_errors_switch", True)


def test_maintenance_requested_falls_back_to_initialization(clean_system):
    openamber = clean_system
    try:
        start_space_heating(openamber, tc=28.0, target=35.0)
        openamber.set_switch("service_mode_enabled", True)

        samples = _restart_and_sample(openamber, seconds=30)

        assert any(s["main"] == "Checking running units" for s in samples), samples
        _assert_initialized(samples)
    finally:
        openamber.set_switch("service_mode_enabled", False)
        advance_until_state(openamber, "state_machine_state_main", "Heat/Cool", max_s=300, step=10)


# ---------------------------------------------------------------------------
# OTA hold: the state machine is frozen while an update is written
# ---------------------------------------------------------------------------
# The mock switches call the same controller hooks as the ota on_begin / on_progress / on_error callbacks
# (core/openamber.yaml). The ESPHome OTA platforms themselves are not part of the host build.

def _heating_past_min_on_time(openamber):
    """Heating with the compressor past its minimum on time, so an overshoot stops it at the next update."""
    start_space_heating(openamber, tc=28.0, target=35.0)
    openamber.advance_time(seconds=COMPRESSOR_MIN_ON_S + 30, step_s=10)
    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"


def _overshoot(openamber):
    openamber.set_sensor("heat_cool_temperature_tc", 40.0)
    openamber.set_sensor("outlet_temperature_tuo", 40.0)
    openamber.set_sensor("inlet_temperature_tui", 38.0)


def _level(openamber):
    return int(openamber.get_entity("compressor_control_select") or 0)


def test_ota_hold_freezes_regulation_until_error(clean_system):
    """OTA begin holds regulation (progress keeps it), an OTA error releases it before the hold timeout."""
    openamber = clean_system
    try:
        _heating_past_min_on_time(openamber)
        assert openamber.set_switch("mock_ota_begin", True)
        _overshoot(openamber)

        # Longer than the hold timeout, with progress reports: the hold stays.
        for _ in range(OTA_HOLD_TIMEOUT_S // 30 + 2):
            openamber.advance_time(seconds=30, step_s=5)
            assert openamber.set_switch("mock_ota_progress", True)
            assert _level(openamber) > 0, "Regulation must be held during the OTA update"
            assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"

        assert openamber.set_switch("mock_ota_error", True)
        openamber.advance_time(seconds=10, step_s=5)
        openamber.step(ms=50)
        assert _level(openamber) == 0, "After the OTA error regulation resumes and stops on the overshoot"
    finally:
        openamber.set_switch("mock_ota_error", True)


def test_ota_hold_times_out_without_progress(clean_system):
    """No progress reports (e.g. a web upload that disconnected): the hold is released after the timeout."""
    openamber = clean_system
    try:
        _heating_past_min_on_time(openamber)
        assert openamber.set_switch("mock_ota_begin", True)
        _overshoot(openamber)

        openamber.advance_time(seconds=OTA_HOLD_TIMEOUT_S - 20, step_s=5)
        openamber.step(ms=50)
        assert _level(openamber) > 0, "Regulation must be held before the timeout"

        openamber.advance_time(seconds=40, step_s=5)
        openamber.step(ms=50)
        assert _level(openamber) == 0, "Without OTA progress the hold must time out and regulation resume"
    finally:
        openamber.set_switch("mock_ota_error", True)


def test_ota_hold_does_not_block_safety_stop(clean_system):
    """A safety condition (Tuo-Tui difference) during the hold stops the compressor at once."""
    openamber = clean_system
    try:
        _heating_past_min_on_time(openamber)
        assert openamber.set_switch("mock_ota_begin", True)
        openamber.advance_time(seconds=10, step_s=5)
        assert _level(openamber) > 0

        openamber.set_sensor("outlet_temperature_tuo", 40.0)
        openamber.set_sensor("inlet_temperature_tui", 30.0)
        assert openamber.set_switch("mock_ota_progress", True)
        openamber.advance_time(seconds=10, step_s=5)
        openamber.step(ms=50)
        assert _level(openamber) == 0, "Safety stop must not be held by the OTA update"
    finally:
        openamber.set_switch("mock_ota_error", True)


def test_ota_hold_does_not_block_modbus_connection_timeout(clean_system):
    """Modbus lost during the hold: the connection timeout still raises its error and stops the units."""
    openamber = clean_system
    try:
        _heating_past_min_on_time(openamber)
        assert openamber.set_switch("mock_ota_begin", True)
        assert openamber.set_switch("mock_modbus_inside_offline", True)

        # Keep the hold alive with progress reports past the Modbus connection timeout.
        for _ in range(MODBUS_CONNECTION_TIMEOUT_S // 30 + 2):
            openamber.advance_time(seconds=30, step_s=5)
            assert openamber.set_switch("mock_ota_progress", True)
        openamber.step(ms=50)

        assert openamber.get_entity("error_modbus_connection_timeout") is True, "Modbus timeout must be raised during the hold"
        assert _level(openamber) == 0, "The Modbus timeout error must stop the compressor during the hold"
    finally:
        openamber.set_switch("mock_modbus_inside_offline", False)
        openamber.set_switch("mock_ota_error", True)
        openamber.set_switch("reset_start_timeout_errors_switch", True)
