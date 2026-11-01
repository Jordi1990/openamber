"""DHW requested while space heating is active: 3-way valve, pump P1 and external DHW demand.

Covers the report where Home Assistant turns on "Tapwater aanwezig" during a heating run, after
which hot DHW water ended up in the floor (Tc rises) and OpenAmber raised the 3-way valve error.

1. Turning dhw_enabled_switch on while the compressor heats switches the valve to DHW.
2. Pump P1, left running while the heat/cool controller idles with heat demand, is stopped
   before the valve switches to DHW.
3. dhw_demand_switch ("Externe tapwatervraag") starts DHW without schedule and restart delta,
   and stops at the setpoint.
"""

from helpers import advance_until_state, start_space_heating

WEEKDAYS = ("monday", "tuesday", "wednesday", "thursday", "friday", "saturday", "sunday")


def _assert_valve_dhw(openamber):
    assert openamber.get_entity("three_way_valve_dhw_switch") is True, "3-way valve SWW output should be on"
    assert openamber.get_entity("three_way_valve_heat_cool_switch") is False, "3-way valve CV output should be off"


def test_enabling_dhw_switch_during_heating_run_switches_valve_to_dhw(clean_system):
    openamber = clean_system
    openamber.set_switch("dhw_enabled_switch", False)
    openamber.set_sensor("dhw_temperature_tw_sensor", 38.0)
    start_space_heating(openamber)
    assert openamber.get_entity("three_way_valve_dhw_switch") is False

    # Home Assistant turns on "Tapwater aanwezig" while the compressor heats the floor.
    openamber.set_switch("dhw_enabled_switch", True)
    openamber.step(ms=50)
    assert openamber.get_entity("dhw_demand_active_sensor") is True

    assert advance_until_state(openamber, "state_machine_state_main", "DHW", max_s=1500), (
        f"Never switched to DHW (main: {openamber.get_entity('state_machine_state_main')}, "
        f"heat/cool: {openamber.get_entity('state_machine_state_heat_cool')})"
    )
    _assert_valve_dhw(openamber)
    assert openamber.get_entity("pump_p1_relay_switch") is False

    assert advance_until_state(openamber, "state_machine_state_dhw", "Compressor running", max_s=300)
    openamber.set_sensor("heat_cool_temperature_tc", 30.0)
    openamber.set_sensor("outlet_temperature_tuo", 45.0)
    openamber.set_sensor("inlet_temperature_tui", 40.0)
    openamber.advance_time(seconds=300, step_s=10)
    openamber.step(ms=50)
    _assert_valve_dhw(openamber)
    assert openamber.get_entity("error_three_way_valve_state_timeout") is False

    # Home Assistant turns the switch off again after the run: back to heating with the valve on CV.
    openamber.set_sensor("dhw_temperature_tw_sensor", 51.0)
    openamber.set_switch("dhw_enabled_switch", False)
    openamber.step(ms=50)
    assert advance_until_state(openamber, "state_machine_state_main", "Heat/Cool", max_s=900)
    assert openamber.get_entity("three_way_valve_dhw_switch") is False
    assert openamber.get_entity("three_way_valve_heat_cool_switch") is True


def test_pump_p1_stops_when_dhw_starts_from_idle_heating(clean_system):
    openamber = clean_system
    openamber.set_switch("pump_p1_enabled", True)
    openamber.set_select("heat_mode_select", "Extern setpoint")
    openamber.set_number("manual_setpoint", 35.0)
    openamber.set_climate("pid_heat_temperature_control", target_temperature=35.0)
    # Supply temperature at setpoint: heat demand, but no reason to start the compressor.
    openamber.set_sensor("heat_cool_temperature_tc", 35.0)
    openamber.set_sensor("outlet_temperature_tuo", 35.0)
    openamber.set_sensor("inlet_temperature_tui", 33.0)
    openamber.set_switch("heat_demand_switch", True)
    openamber.advance_time(seconds=10, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity("pump_p1_relay_switch") is True, "P1 should run while heat demand is active"

    openamber.set_sensor("dhw_temperature_tw_sensor", 38.0)
    openamber.step(ms=50)
    assert advance_until_state(openamber, "state_machine_state_main", "DHW", max_s=600)
    _assert_valve_dhw(openamber)
    assert openamber.get_entity("pump_p1_relay_switch") is False, "P1 must not run while the valve is on DHW"

    openamber.advance_time(seconds=120, step_s=10)
    openamber.step(ms=50)
    assert openamber.get_entity("pump_p1_relay_switch") is False


def test_external_dhw_demand_ignores_schedule_and_restart_delta(clean_system):
    openamber = clean_system
    weekdays = {d: openamber.get_entity(f"dhw_schedule_{d}_enabled_switch") for d in WEEKDAYS}
    try:
        # Schedule without any day: internal DHW demand never starts.
        for d in WEEKDAYS:
            openamber.set_switch(f"dhw_schedule_{d}_enabled_switch", False)
        openamber.set_switch("dhw_schedule_enabled_switch", True)
        # Within the restart delta (50 - 5 = 45°C).
        openamber.set_sensor("dhw_temperature_tw_sensor", 47.0)
        openamber.step(ms=50)
        assert openamber.get_entity("dhw_demand_active_sensor") is False

        openamber.set_switch("dhw_demand_switch", True)
        openamber.step(ms=50)
        assert openamber.get_entity("dhw_demand_active_sensor") is True
        assert advance_until_state(openamber, "state_machine_state_main", "DHW", max_s=600)
        _assert_valve_dhw(openamber)

        # Setpoint reached: demand ends even though the external switch is still on.
        openamber.set_sensor("dhw_temperature_tw_sensor", 50.5)
        openamber.step(ms=50)
        assert openamber.get_entity("dhw_demand_active_sensor") is False

        # "Tapwater aanwezig" off still blocks external demand.
        openamber.set_sensor("dhw_temperature_tw_sensor", 40.0)
        openamber.set_switch("dhw_enabled_switch", False)
        openamber.step(ms=50)
        assert openamber.get_entity("dhw_demand_active_sensor") is False
    finally:
        openamber.set_switch("dhw_demand_switch", False)
        openamber.set_switch("dhw_schedule_enabled_switch", False)
        for d, state in weekdays.items():
            if state is not None:
                openamber.set_switch(f"dhw_schedule_{d}_enabled_switch", state)
