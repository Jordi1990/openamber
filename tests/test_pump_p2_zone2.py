"""Tests for "Pomp P2 aanwezig" (pump_p2_enabled): the circulation pump of zone 2 on the P2 contact.

Like the original Amber controller (installation manual M11.9 "Externe cv-pomp (P2) verwarmen" on, M11.10 "koelen"
off), P2 runs during space heating on zone 2 heat demand, and not during cooling. It is stopped at the same moments
as P1: defrost, DHW, maintenance and a controlled stop. The default is off.

Deaeration runs P0, P1 and P2 on the CV circuit only, like the original controller: for the DHW circuit P1 and P2 stop.
"""

import os

import yaml

from helpers import advance_until_state, enter_maintenance, leave_maintenance, set_outside_temperature
from test_entity_schema_sync import ESPHomeYamlLoader

SWITCHES_YAML = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                             "src", "openamber", "core", "switches.yaml")


def _setup_without_compressor_start(openamber):
    """Heat demand without a reason to start the compressor, so P0 only runs interval cycles."""
    openamber.set_select("thermostat_mode_select", "Extern")
    openamber.set_select("heat_mode_select", "Extern setpoint")
    openamber.set_number("manual_setpoint", 35.0)
    openamber.set_climate("pid_heat_temperature_control", target_temperature=35.0)
    openamber.set_climate("pid_cool_temperature_control", target_temperature=40.0)
    openamber.set_sensor("heat_cool_temperature_tc", 35.0)
    openamber.set_sensor("outlet_temperature_tuo", 35.0)
    openamber.set_sensor("inlet_temperature_tui", 33.0)
    # Zone 2 heat demand needs a mixing valve configured.
    openamber.set_switch("mixing_valve_zone2_enabled", True)
    openamber.set_switch("pump_p2_enabled", True)


def _settle(openamber, seconds=10):
    openamber.advance_time(seconds=seconds, step_s=5)
    openamber.step(ms=50)


def _end_demand(openamber):
    openamber.set_switch("heat_demand_switch", False)
    openamber.set_switch("heat_demand_zone_2_switch", False)
    openamber.set_switch("cool_demand_switch", False)
    advance_until_state(openamber, "state_machine_state_heat_cool", "Idle", max_s=900, step=10)
    _settle(openamber)


def test_pump_p2_defaults_off():
    with open(SWITCHES_YAML, encoding="utf-8") as f:
        switches = yaml.load(f, Loader=ESPHomeYamlLoader)["switch"]
    entry = next(s for s in switches if s.get("id") == "pump_p2_enabled")
    assert entry["restore_mode"] == "RESTORE_DEFAULT_OFF"


def test_pump_p2_runs_on_zone2_heat_demand(clean_system):
    openamber = clean_system
    _setup_without_compressor_start(openamber)

    openamber.set_switch("heat_demand_zone_2_switch", True)
    _settle(openamber)
    assert openamber.get_entity("heat_demand_active_sensor_zone_2") is True
    assert openamber.get_entity("pump_p2_relay_switch") is True, "P2 must run on zone 2 heat demand"

    # Through a P0 interval cycle and back to idle, P2 keeps running while zone 2 asks for heat.
    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Pump running", max_s=1200)
    assert openamber.get_entity("pump_p2_relay_switch") is True
    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Idle", max_s=900)
    _settle(openamber)
    assert openamber.get_entity("pump_p2_relay_switch") is True, "P2 must keep running in idle with zone 2 demand"

    _end_demand(openamber)
    assert openamber.get_entity("pump_p2_relay_switch") is False, "P2 must stop when the zone 2 demand ends"


def test_pump_p2_follows_zone2_demand_while_zone1_keeps_heating(clean_system):
    openamber = clean_system
    _setup_without_compressor_start(openamber)

    openamber.set_switch("heat_demand_switch", True)
    _settle(openamber)
    assert openamber.get_entity("pump_p2_relay_switch") is False, "P2 must not run on zone 1 heat demand only"

    openamber.set_switch("heat_demand_zone_2_switch", True)
    _settle(openamber)
    assert openamber.get_entity("pump_p2_relay_switch") is True, "P2 must start when zone 2 asks for heat"

    openamber.set_switch("heat_demand_zone_2_switch", False)
    _settle(openamber)
    assert openamber.get_entity("heat_demand_active_sensor") is True, "Zone 1 still asks for heat"
    assert openamber.get_entity("pump_p2_relay_switch") is False, "P2 must stop when only zone 1 asks for heat"

    _end_demand(openamber)


def test_pump_p2_not_used_when_not_present(clean_system):
    openamber = clean_system
    _setup_without_compressor_start(openamber)
    openamber.set_switch("pump_p2_enabled", False)

    openamber.set_switch("heat_demand_zone_2_switch", True)
    _settle(openamber)
    assert openamber.get_entity("heat_demand_active_sensor_zone_2") is True
    assert openamber.get_entity("pump_p2_relay_switch") is False

    _end_demand(openamber)


def test_pump_p2_off_during_cooling(clean_system):
    openamber = clean_system
    _setup_without_compressor_start(openamber)

    openamber.set_switch("cool_demand_switch", True)
    _settle(openamber)
    assert openamber.get_entity("cool_demand_active_sensor") is True
    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Pump running", max_s=1200)
    assert openamber.get_entity("pump_p2_relay_switch") is False, "P2 must not run during cooling"

    _end_demand(openamber)


def test_pump_p2_turning_off_present_switches_relay_off(clean_system):
    openamber = clean_system
    _setup_without_compressor_start(openamber)

    openamber.set_switch("heat_demand_zone_2_switch", True)
    _settle(openamber)
    assert openamber.get_entity("pump_p2_relay_switch") is True

    openamber.set_switch("pump_p2_enabled", False)
    _settle(openamber)
    assert openamber.get_entity("pump_p2_relay_switch") is False, "P2 must not stay on once it is not present"

    _end_demand(openamber)


def test_pump_p2_stops_when_dhw_requested(clean_system):
    openamber = clean_system
    _setup_without_compressor_start(openamber)
    openamber.set_switch("heat_demand_zone_2_switch", True)
    _settle(openamber)
    assert openamber.get_entity("pump_p2_relay_switch") is True

    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Pump running", max_s=1200)
    openamber.set_sensor("dhw_temperature_tw_sensor", 38.0)
    openamber.step(ms=50)

    assert advance_until_state(openamber, "state_machine_state_main", "DHW", max_s=2400)
    assert openamber.get_entity("pump_p2_relay_switch") is False, "P2 must not run while the valve is on DHW"

    openamber.set_sensor("dhw_temperature_tw_sensor", 55.0)
    _end_demand(openamber)


def test_pump_p2_runs_with_p0_cycles_on_frost_protection(clean_system):
    openamber = clean_system
    _setup_without_compressor_start(openamber)
    openamber.set_number("pump_interval", 15.0)
    set_outside_temperature(openamber, 2.0)
    openamber.set_number("frost_protection_stage_1_temp_ta", 5.0)
    try:
        assert advance_until_state(openamber, "frost_protection_stage1_active", True, max_s=400, step=10)
        assert advance_until_state(openamber, "state_machine_state_heat_cool", "Pump running", max_s=1200)
        _settle(openamber)
        assert openamber.get_entity("pump_p2_relay_switch") is True, "P2 must run with the P0 cycle on frost protection"

        assert advance_until_state(openamber, "state_machine_state_heat_cool", "Idle", max_s=900)
        _settle(openamber)
        assert openamber.get_entity("pump_p2_relay_switch") is False, "P2 must stop with the P0 cycle without demand"
    finally:
        set_outside_temperature(openamber, 7.0)
        openamber.advance_time(seconds=310, step_s=10)
        openamber.step(ms=50)


def test_pump_p2_ui_switch(clean_system):
    """The switch under Instellingen > Pomp > Algemeen."""
    openamber = clean_system
    assert openamber.get_widget("set_pump_p2_switch").get("checked") is False

    assert openamber.click("set_pump_p2_switch")
    openamber.step(ms=50)
    assert openamber.get_entity("pump_p2_enabled") is True
    assert openamber.get_widget("set_pump_p2_switch").get("checked") is True

    assert openamber.set_switch("pump_p2_enabled", False)
    openamber.step(ms=50)
    assert openamber.get_widget("set_pump_p2_switch").get("checked") is False


def test_deaeration_runs_p1_and_p2_on_the_cv_circuit_only(clean_system):
    """Like the original controller: P0, P1 and P2 run during CV deaeration; for the DHW circuit P1 and P2 stop,
    so no DHW water is pulled into the floor."""
    openamber = clean_system
    openamber.set_switch("pump_p1_enabled", True)
    openamber.set_switch("pump_p2_enabled", True)
    openamber.set_switch("dhw_enabled_switch", True)
    enter_maintenance(openamber)
    try:
        assert openamber.set_switch("deaeration_short_switch", True) is True
        _settle(openamber, seconds=5)
        assert openamber.get_entity("pump_p0_relay_switch") is True
        assert openamber.get_entity("pump_p1_relay_switch") is True, "P1 must run during CV deaeration"
        assert openamber.get_entity("pump_p2_relay_switch") is True, "P2 must run during CV deaeration"

        # The routine text only shows the phase for a moment ("Wachten..." in between), so wait for the valve.
        assert advance_until_state(openamber, "three_way_valve_dhw_switch", True, max_s=1200, step=5), (
            f"DHW circuit not reached (routine: {openamber.get_entity('state_machine_state_routine')})"
        )
        assert openamber.get_entity("pump_p1_relay_switch") is False, "P1 must stop before the valve switches to DHW"
        assert openamber.get_entity("pump_p2_relay_switch") is False, "P2 must stop before the valve switches to DHW"
        _settle(openamber, seconds=5)
        assert openamber.get_entity("pump_p0_relay_switch") is True
        assert openamber.get_entity("pump_p1_relay_switch") is False, "P1 must not run during DHW deaeration"
        assert openamber.get_entity("pump_p2_relay_switch") is False, "P2 must not run during DHW deaeration"
    finally:
        openamber.set_switch("deaeration_short_switch", False)
        openamber.step(ms=50)
        leave_maintenance(openamber)


def test_pump_p2_frost_protection_with_zone1_demand_only_with_p0(clean_system):
    """Zone 1 demand keeps the secondary pumps active in idle; frost protection alone must still only run P2
    together with P0, not continuously between the P0 cycles."""
    openamber = clean_system
    _setup_without_compressor_start(openamber)
    openamber.set_number("pump_interval", 15.0)
    set_outside_temperature(openamber, 2.0)
    openamber.set_number("frost_protection_stage_1_temp_ta", 5.0)
    try:
        assert advance_until_state(openamber, "frost_protection_stage1_active", True, max_s=400, step=10)
        openamber.set_switch("heat_demand_switch", True)
        _settle(openamber)

        assert advance_until_state(openamber, "state_machine_state_heat_cool", "Pump running", max_s=1200)
        _settle(openamber)
        assert openamber.get_entity("pump_p2_relay_switch") is True, "P2 must run with the P0 cycle on frost protection"

        assert advance_until_state(openamber, "state_machine_state_heat_cool", "Idle", max_s=900)
        _settle(openamber)
        assert openamber.get_entity("pump_p1_relay_switch") is False or openamber.get_entity("pump_p1_enabled") is False
        assert openamber.get_entity("heat_demand_active_sensor") is True, "Zone 1 still asks for heat"
        assert openamber.get_entity("pump_p2_relay_switch") is False, "P2 must not run between the P0 cycles"
    finally:
        _end_demand(openamber)
        set_outside_temperature(openamber, 7.0)
        openamber.advance_time(seconds=310, step_s=10)
        openamber.step(ms=50)


def test_pump_p2_stays_off_during_cooling_run_on(clean_system):
    """Zone 2 asks for heat while the compressor still runs on in cooling (minimum on time): P2 stays off as long
    as the working mode is cooling."""
    openamber = clean_system
    _setup_without_compressor_start(openamber)
    openamber.set_climate("pid_cool_temperature_control", target_temperature=18.0)
    for sensor in ("current_water_temperature_tc_sensor", "heat_cool_temperature_tc", "outlet_temperature_tuo",
                   "inlet_temperature_tui"):
        openamber.set_sensor(sensor, 25.0)
    openamber.set_switch("cool_demand_switch", True)
    openamber.step(ms=50)
    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Compressor running", max_s=1800)
    assert openamber.get_entity("working_mode_switch") == "Koelen"

    openamber.set_switch("cool_demand_switch", False)
    openamber.set_switch("heat_demand_zone_2_switch", True)
    openamber.step(ms=50)
    assert openamber.get_entity("heat_demand_active_sensor_zone_2") is True
    checked_while_cooling = 0
    for _ in range(24):
        if openamber.get_entity("working_mode_switch") != "Koelen":
            break
        assert openamber.get_entity("pump_p2_relay_switch") is False, "P2 must stay off while the mode is cooling"
        checked_while_cooling += 1
        openamber.advance_time(seconds=10, step_s=5)
        openamber.step(ms=20)
    assert checked_while_cooling >= 3, "The cooling run-on was not reached, the test checked nothing"

    _end_demand(openamber)


def test_pump_p2_restored_when_resuming_pump_cycle(clean_system):
    """After an ESP restart that takes over a running pump cycle, P2 runs again on zone 2 heat demand."""
    openamber = clean_system
    _setup_without_compressor_start(openamber)
    openamber.set_switch("heat_demand_zone_2_switch", True)
    _settle(openamber)
    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Pump running", max_s=1200)
    openamber.set_select("working_mode_switch", "Verwarmen")
    openamber.set_switch("pump_p2_relay_switch", False)
    openamber.step(ms=50)

    assert openamber.set_switch("mock_simulate_esp_restart", True)
    openamber.step(ms=50)
    assert advance_until_state(openamber, "state_machine_state_main", "Heat/Cool", max_s=120, step=5)
    _settle(openamber)
    assert openamber.get_entity("state_machine_state_heat_cool") == "Pump running", "The pump cycle must be taken over"
    assert openamber.get_entity("pump_p2_relay_switch") is True, "P2 must run again after the takeover"

    _end_demand(openamber)


def test_deaeration_starts_p2_after_the_valve_switched_from_dhw(clean_system):
    openamber = clean_system
    openamber.set_switch("pump_p2_enabled", True)
    enter_maintenance(openamber)
    openamber.set_switch("three_way_valve_heat_cool_switch", False)
    openamber.set_switch("three_way_valve_dhw_switch", True)
    openamber.set_switch("pump_p2_relay_switch", False)
    openamber.step(ms=50)
    try:
        assert openamber.set_switch("deaeration_short_switch", True) is True
        openamber.step(ms=50)
        assert openamber.get_entity("pump_p0_relay_switch") is True
        assert openamber.get_entity("pump_p2_relay_switch") is False, "P2 must wait for the valve"

        openamber.advance_time(seconds=65, step_s=5)
        openamber.step(ms=50)
        assert openamber.get_entity("pump_p2_relay_switch") is True, "P2 must start once the valve is on CV"
    finally:
        openamber.set_switch("deaeration_short_switch", False)
        openamber.step(ms=50)
        leave_maintenance(openamber)
