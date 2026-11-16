"""Automated tests for maintenance mode after stopping a running system.

Maintenance mode can only be turned on while the heat pump is stopped. The controlled stop ("Gecontroleerd
stoppen", see test_controlled_stop.py) stops the compressor, also during DHW and after a running oil return cycle;
maintenance mode is then entered from Standby. Turning it on while the compressor runs is refused.
"""

from helpers import advance_until_state, enter_maintenance


def _start_dhw_compressor_past_min_on_time(openamber):
    openamber.set_number("dhw_setpoint_temperature", 50.0)
    openamber.set_number("dhw_restart_dhw_delta", 5.0)
    openamber.set_number("dhw_backup_min_avg_rate", 0.0)
    openamber.set_sensor("dhw_temperature_tw_sensor", 40.0)
    openamber.step(ms=100)
    assert advance_until_state(openamber, "state_machine_state_dhw", "Compressor running", max_s=900, step=5)
    # Past the minimum compressor on time; the tank is still below the setpoint, so DHW demand stays.
    openamber.advance_time(seconds=620, step_s=20)
    openamber.step(ms=50)
    assert openamber.get_entity("dhw_demand_active_sensor") is True


def test_maintenance_mode_refused_during_dhw(clean_system):
    """Turning maintenance mode on while DHW runs is refused: the compressor keeps running."""
    openamber = clean_system
    _start_dhw_compressor_past_min_on_time(openamber)

    openamber.set_switch("service_mode_enabled", True)
    openamber.advance_time(seconds=10, step_s=2)
    openamber.step(ms=50)
    assert openamber.get_entity("service_mode_enabled") is False
    assert int(openamber.get_entity("compressor_control_select")) > 0
    assert openamber.get_entity("state_machine_state_main") == "DHW"

    openamber.set_sensor("dhw_temperature_tw_sensor", 55.0)
    openamber.advance_time(seconds=60, step_s=5)


def test_maintenance_mode_after_stopping_dhw_while_demand_remains(clean_system):
    """A controlled stop during DHW with remaining demand, then maintenance mode: everything is off."""
    openamber = clean_system
    _start_dhw_compressor_past_min_on_time(openamber)

    enter_maintenance(openamber)
    assert int(openamber.get_entity("compressor_control_select")) == 0
    # Same end state as maintenance mode entered from heating/cooling.
    assert openamber.get_entity("working_mode_switch") == "Onderhoud"
    assert openamber.get_entity("pump_p0_relay_switch") is False, "Pump P0 relay must be off in maintenance mode"
    assert openamber.get_entity("internal_pump_active") is False
    assert openamber.get_entity("dhw_pump_relay_switch") is False
    assert openamber.get_entity("backup_heater_relay") is False

    # Leaving maintenance mode does not restart DHW: the heat pump stays stopped until resumed.
    openamber.set_switch("service_mode_enabled", False)
    openamber.step(ms=50)
    assert advance_until_state(openamber, "state_machine_state_main", "Standby", max_s=300, step=5)
    assert int(openamber.get_entity("compressor_control_select")) == 0

    openamber.set_sensor("dhw_temperature_tw_sensor", 55.0)
    openamber.press_button("controlled_stop_resume_button")
    openamber.advance_time(seconds=60, step_s=5)
