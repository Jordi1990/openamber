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
    """The sample of the update that took over the units: right after 'Checking running units'."""
    for previous, current in zip(samples, samples[1:]):
        if previous["main"] == "Checking running units" and current["main"] in RESUMED_MAIN_STATES:
            return current
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
    _restart_and_sample(openamber, seconds=30)
    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"

    # Overshoot: would stop at once without minimum on time.
    openamber.set_sensor("heat_cool_temperature_tc", 40.0)
    openamber.set_sensor("outlet_temperature_tuo", 40.0)
    openamber.set_sensor("inlet_temperature_tui", 38.0)
    openamber.advance_time(seconds=120, step_s=5)
    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"

    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Wait compressor stop", max_s=900, step=10) or \
        int(openamber.get_entity("compressor_control_select") or 0) == 0


def test_defrost_is_resumed_after_restart(clean_system):
    openamber = clean_system
    try:
        start_space_heating(openamber, tc=28.0, target=35.0)
        openamber.set_binary_sensor("defrost_active_sensor", True)

        samples = _restart_and_sample(openamber, seconds=40)

        _assert_not_initialized(samples)
        assert _resumed_sample(samples)["hc"] == "Defrosting"
        assert all(s["level"] > 0 for s in samples), f"Compressor was stopped during the defrost: {samples}"
        assert all(s["mode"] == "Verwarmen" for s in samples), f"Working mode changed during the defrost: {samples}"
        assert openamber.get_entity("state_machine_state_heat_cool") == "Defrosting"
    finally:
        openamber.set_binary_sensor("defrost_active_sensor", False)


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
        openamber.set_binary_sensor("defrost_active_sensor", True)

        samples = _restart_and_sample(openamber, seconds=30)

        _assert_not_initialized(samples)
        assert _resumed_sample(samples)["dhw"] == "Defrosting"
        assert openamber.get_entity("state_machine_state_dhw") == "Defrosting"
    finally:
        openamber.set_binary_sensor("defrost_active_sensor", False)


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
