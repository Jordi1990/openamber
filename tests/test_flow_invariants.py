"""Flow tests that check the hardware rules during complete heating, cooling and DHW flows.

Most flow tests look at the state at a few points in time. These tests run the same kind of flows under
`FlowMonitor` (helpers.py), which checks after every 5 s of virtual time that:
- the 3-way valve only moves while the compressor, pump P0 and the backup heater are off;
- the compressor is only requested with flow and in a heating/cooling working mode;
- the compressor keeps its minimum on time (600 s) and minimum off time (200 s);
- the backup heater only runs with flow;
- pump P1 and the DHW pump match the 3-way valve position;
- only the controller that owns the main state is active.
"""

from helpers import FlowMonitor, elapsed_s, now_s


def _heating_conditions(openamber, tc=28.0, target=35.0):
    """Heat demand conditions with Tc below the start temperature (target - start delta)."""
    openamber.set_select("heat_mode_select", "Extern setpoint")
    openamber.set_number("manual_setpoint", target)
    openamber.set_climate("pid_heat_temperature_control", target_temperature=target)
    openamber.set_number("compressor_start_delta_heating", 2.0)
    openamber.set_number("backup_heater_degmin_threshold", 9999.0)
    openamber.set_sensor("heat_cool_temperature_tc", tc)
    openamber.set_sensor("outlet_temperature_tuo", tc)
    openamber.set_sensor("inlet_temperature_tui", tc - 2.0)


def _cooling_conditions(openamber, tc=25.0):
    """Cool demand conditions with Tc above the start temperature (18 °C + start delta)."""
    openamber.set_sensor("heat_cool_temperature_tc", tc)
    openamber.set_sensor("outlet_temperature_tuo", tc)
    openamber.set_sensor("inlet_temperature_tui", tc)


def _compressor(openamber):
    return int(openamber.get_entity("compressor_control_select") or 0)


def _advance_until_compressor(monitor, running, max_s):
    """Advance until the compressor request is on (running=True) or off. Returns the waited seconds, or None."""
    waited = 0
    while waited <= max_s:
        if (_compressor(monitor.openamber) > 0) == running:
            return waited
        monitor.advance(5)
        waited += 5
    return None


def test_heat_dhw_heat_cycle_keeps_hardware_rules(clean_system):
    """Heating -> DHW -> heating with pump P1 enabled: the rules hold throughout and heating resumes."""
    openamber = clean_system
    openamber.set_switch("pump_p1_enabled", True)
    _heating_conditions(openamber)
    openamber.set_switch("heat_demand_switch", True)
    monitor = FlowMonitor(openamber)

    assert monitor.advance_until("state_machine_state_heat_cool", "Compressor running", 1800)
    assert openamber.get_entity("pump_p1_relay_switch") is True, "P1 runs with heat demand"
    assert openamber.get_entity("three_way_valve_heat_cool_switch") is True

    # DHW request during heating: heating finishes its minimum on time, then the valve switches.
    openamber.set_sensor("dhw_temperature_tw_sensor", 38.0)
    assert monitor.advance_until("state_machine_state_main", "DHW", 900), (
        f"Never switched to DHW (main: {openamber.get_entity('state_machine_state_main')})"
    )
    assert openamber.get_entity("three_way_valve_dhw_switch") is True
    assert openamber.get_entity("three_way_valve_heat_cool_switch") is False
    assert openamber.get_entity("working_mode_switch") == "Standby", "Heating left the unit in standby for the valve switch"

    assert monitor.advance_until("state_machine_state_dhw", "Compressor running", 600)
    assert openamber.get_entity("working_mode_switch") == "Verwarmen"
    monitor.advance(700)
    assert openamber.get_entity("dhw_pump_relay_switch") is True

    # Tank at setpoint: DHW stops, the valve returns and heating resumes.
    openamber.set_sensor("dhw_temperature_tw_sensor", 52.0)
    assert _advance_until_compressor(monitor, running=False, max_s=30) is not None, "DHW must stop at the setpoint"
    assert monitor.advance_until("state_machine_state_main", "Heat/Cool", 200)
    assert openamber.get_entity("three_way_valve_dhw_switch") is False
    assert openamber.get_entity("three_way_valve_heat_cool_switch") is True
    assert openamber.get_entity("dhw_pump_relay_switch") is False

    assert _advance_until_compressor(monitor, running=True, max_s=1500) is not None, (
        f"Heating did not resume after DHW (state: {openamber.get_entity('state_machine_state_heat_cool')})"
    )
    assert openamber.get_entity("working_mode_switch") == "Verwarmen"
    assert openamber.get_entity("pump_p1_relay_switch") is True


def test_cool_dhw_cool_cycle_keeps_hardware_rules(clean_system):
    """Cooling -> DHW (heating mode) -> cooling: the working mode follows the circuit, the rules hold throughout."""
    openamber = clean_system
    _cooling_conditions(openamber)
    openamber.set_switch("cool_demand_switch", True)
    monitor = FlowMonitor(openamber)

    assert monitor.advance_until("state_machine_state_heat_cool", "Compressor running", 1800)
    assert openamber.get_entity("working_mode_switch") == "Koelen"

    openamber.set_sensor("dhw_temperature_tw_sensor", 38.0)
    assert monitor.advance_until("state_machine_state_dhw", "Compressor running", 1500)
    assert openamber.get_entity("working_mode_switch") == "Verwarmen"
    monitor.advance(700)

    openamber.set_sensor("dhw_temperature_tw_sensor", 52.0)
    assert monitor.advance_until("state_machine_state_main", "Heat/Cool", 300)
    assert openamber.get_entity("working_mode_switch") == "Standby"
    assert _advance_until_compressor(monitor, running=True, max_s=1500) is not None, (
        f"Cooling did not resume after DHW (state: {openamber.get_entity('state_machine_state_heat_cool')})"
    )
    assert openamber.get_entity("working_mode_switch") == "Koelen"


def test_dhw_demand_that_ends_during_compressor_start_keeps_minimum_on_time(clean_system):
    """The tank reaches its setpoint right after DHW requested the compressor: the compressor still runs its
    minimum on time before it stops, and the valve only returns afterwards."""
    openamber = clean_system
    openamber.set_sensor("dhw_temperature_tw_sensor", 38.0)
    monitor = FlowMonitor(openamber)
    assert _advance_until_compressor(monitor, running=True, max_s=600) is not None
    started_at = now_s(openamber)

    openamber.set_sensor("dhw_temperature_tw_sensor", 52.0)
    assert _advance_until_compressor(monitor, running=False, max_s=700) is not None
    assert elapsed_s(started_at, now_s(openamber)) >= 600
    assert monitor.advance_until("state_machine_state_main", "Heat/Cool", 200)


def test_pump_cycle_ends_when_the_supply_is_warm_enough(clean_system):
    """Heat demand with a supply above the start temperature: the compressor is not wanted, so the pump cycle
    ends after the pump duration and the compressor stays off."""
    openamber = clean_system
    _heating_conditions(openamber, tc=34.0)
    openamber.set_switch("heat_demand_switch", True)
    monitor = FlowMonitor(openamber)
    assert monitor.advance_until("state_machine_state_heat_cool", "Pump running", 1000)
    assert monitor.advance_until("state_machine_state_heat_cool", "Idle", 200), (
        f"Pump cycle did not end (state: {openamber.get_entity('state_machine_state_heat_cool')})"
    )
    assert _compressor(openamber) == 0
    assert openamber.get_entity("internal_pump_active") is False
