"""Hot switch between heating and DHW.

DHW demand during a heating run moves the 3-way valve to DHW while the compressor and pump P0 keep running. The DHW
controller takes over the running compressor; its minimum on time counts from the start during heating. When the
DHW demand ends with heat demand, heating takes the compressor back the same way.
"""

from helpers import FlowMonitor, advance_until_state, start_space_heating


def test_hot_switch_keeps_compressor_running(clean_system):
    openamber = clean_system
    openamber.set_sensor("dhw_temperature_tw_sensor", 51.0)
    openamber.set_select("dhw_compressor_mode", "Maximaal")
    openamber.set_switch("pump_p1_enabled", True)
    start_space_heating(openamber)
    assert openamber.get_entity("pump_p1_relay_switch") is True
    heating_level = int(openamber.get_entity("compressor_control_select") or 0)

    monitor = FlowMonitor(openamber)
    monitor.check()
    openamber.set_sensor("dhw_temperature_tw_sensor", 38.0)
    openamber.step(ms=50)
    assert openamber.get_entity("dhw_demand_active_sensor") is True

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
    dhw_level = int(openamber.get_entity("compressor_control_select") or 0)
    assert dhw_level > heating_level + 1, (
        f"DHW should go straight to its own level (heating {heating_level}, DHW {dhw_level})"
    )

    # DHW heats; the DHW pump starts once Tuo reaches the tank temperature.
    openamber.set_sensor("heat_cool_temperature_tc", 30.0)
    openamber.set_sensor("outlet_temperature_tuo", 45.0)
    openamber.set_sensor("inlet_temperature_tui", 40.0)
    monitor.advance(600, step=10)
    assert int(openamber.get_entity("compressor_control_select") or 0) > 0
    assert openamber.get_entity("three_way_valve_dhw_switch") is True

    # DHW demand ends with heat demand: heating takes the running compressor back (hot switch), even with the hot
    # DHW water passing the sensors.
    openamber.set_sensor("dhw_temperature_tw_sensor", 55.0)
    openamber.set_sensor("outlet_temperature_tuo", 52.0)
    openamber.set_sensor("inlet_temperature_tui", 30.0)
    openamber.step(ms=50)
    history_start = len(monitor.history)
    assert monitor.advance_until("state_machine_state_main", "Heat/Cool", max_s=200)
    assert all(s["compressor"] > 0 for s in monitor.history[history_start:]), "Compressor must keep running"
    assert openamber.get_entity("three_way_valve_heat_cool_switch") is True
    assert openamber.get_entity("dhw_pump_relay_switch") is False
    assert openamber.get_entity("state_machine_state_dhw") == "Idle"
    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"
    # The zone pumps wait for the valve travel time to CV.
    assert openamber.get_entity("pump_p1_relay_switch") is False

    # The hot water is flushed out within the grace time; heating keeps running.
    monitor.advance(90, step=10)
    assert openamber.get_entity("pump_p1_relay_switch") is True
    openamber.set_sensor("outlet_temperature_tuo", 32.0)
    openamber.set_sensor("inlet_temperature_tui", 28.0)
    monitor.advance(120, step=10)
    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"
    assert openamber.get_entity("error_water_delta_t_too_high") is False


def test_hot_switch_missing_valve_feedback_raises_error(clean_system):
    """A hot switch checks the valve relay feedback after the valve travel time, like a regular switch."""
    openamber = clean_system
    openamber.set_sensor("dhw_temperature_tw_sensor", 51.0)
    start_space_heating(openamber)
    assert openamber.set_switch("mock_three_way_valve_feedback_missing", True)

    openamber.set_sensor("dhw_temperature_tw_sensor", 38.0)
    assert advance_until_state(openamber, "state_machine_state_main", "DHW", max_s=180)
    assert openamber.get_entity("error_three_way_valve_state_timeout") is False

    openamber.advance_time(seconds=70, step_s=5)
    assert openamber.get_entity("error_three_way_valve_state_timeout") is True
    assert openamber.get_entity("error_active") is True
