"""Hot switch from heating to DHW (hot_switch_heat_to_dhw_switch, default off).

With the switch on, DHW demand during a heating run moves the 3-way valve to DHW while the compressor and pump P0
keep running. The DHW controller takes over the running compressor; its minimum on time counts from the start
during heating. With the switch off the compressor stops before the valve moves, as before.
"""

from helpers import FlowMonitor, start_space_heating


def _request_dhw(openamber):
    openamber.set_sensor("dhw_temperature_tw_sensor", 38.0)
    openamber.step(ms=50)
    assert openamber.get_entity("dhw_demand_active_sensor") is True


def test_hot_switch_keeps_compressor_running(clean_system):
    openamber = clean_system
    openamber.set_switch("hot_switch_heat_to_dhw_switch", True)
    openamber.set_sensor("dhw_temperature_tw_sensor", 51.0)
    start_space_heating(openamber)

    monitor = FlowMonitor(openamber, allow_hot_switch=True)
    monitor.check()
    _request_dhw(openamber)

    assert monitor.advance_until("state_machine_state_main", "DHW", max_s=180), (
        f"Never switched to DHW (main: {openamber.get_entity('state_machine_state_main')})"
    )
    assert all(s["compressor"] > 0 for s in monitor.history), "Compressor must keep running during the hot switch"
    assert all(s["pump"] for s in monitor.history), "Pump P0 must keep running during the hot switch"
    assert openamber.get_entity("three_way_valve_dhw_switch") is True
    assert openamber.get_entity("three_way_valve_heat_cool_switch") is False
    assert openamber.get_entity("pump_p1_relay_switch") is False
    assert openamber.get_entity("working_mode_switch") == "Verwarmen"
    assert openamber.get_entity("state_machine_state_heat_cool") == "Idle"
    assert openamber.get_entity("state_machine_state_dhw") == "Compressor running"
    assert openamber.get_entity("error_three_way_valve_state_timeout") is False

    # DHW heats; the DHW pump starts once Tuo reaches the tank temperature.
    openamber.set_sensor("heat_cool_temperature_tc", 30.0)
    openamber.set_sensor("outlet_temperature_tuo", 45.0)
    openamber.set_sensor("inlet_temperature_tui", 40.0)
    monitor.advance(600, step=10)
    assert int(openamber.get_entity("compressor_control_select") or 0) > 0
    assert openamber.get_entity("three_way_valve_dhw_switch") is True

    # Demand ends: the DHW run stops as usual and heating takes over again with the valve on CV.
    openamber.set_sensor("dhw_temperature_tw_sensor", 55.0)
    openamber.step(ms=50)
    assert monitor.advance_until("state_machine_state_main", "Heat/Cool", max_s=900)
    assert openamber.get_entity("three_way_valve_heat_cool_switch") is True


def test_switch_off_stops_compressor_before_valve_moves(clean_system):
    openamber = clean_system
    openamber.set_sensor("dhw_temperature_tw_sensor", 51.0)
    start_space_heating(openamber)

    monitor = FlowMonitor(openamber)
    monitor.check()
    # The compressor started before the monitor did: its real run time is unknown here, the minimum on time is
    # covered by the other flow tests.
    monitor.compressor_running_since = None
    _request_dhw(openamber)

    assert monitor.advance_until("state_machine_state_main", "DHW", max_s=1500)
    assert any(s["compressor"] == 0 for s in monitor.history), "Without the switch the compressor stops first"

