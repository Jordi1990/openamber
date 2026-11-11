"""Automated tests for how maintenance mode stops a running system.

Maintenance mode stops the compressor on request, during heating and during DHW. A running oil return cycle
(bit 0x08 of register 2119) is finished first; after that the compressor stops and maintenance mode starts.
"""

from helpers import advance_until_state, start_space_heating


def test_maintenance_mode_waits_for_oil_return_cycle(clean_system):
    """Enabling maintenance mode during an oil return cycle keeps the compressor running until it ends."""
    openamber = clean_system
    start_space_heating(openamber)
    # Past the minimum compressor on time, so only the oil return cycle can hold the stop.
    openamber.advance_time(seconds=620, step_s=20)
    openamber.step(ms=50)
    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"

    assert openamber.set_sensor("error_register_2119_raw", 8.0)
    openamber.step(ms=50)
    assert openamber.get_entity("oil_return_cycle_active") is True

    openamber.set_switch("service_mode_enabled", True)
    openamber.advance_time(seconds=60, step_s=5)
    openamber.step(ms=50)
    assert int(openamber.get_entity("compressor_control_select")) > 0, "Compressor must run until the oil return cycle ends"
    assert openamber.get_entity("state_machine_state_main") != "Maintenance"

    # The cycle ends: the compressor stops right away, without the 300 s settle time of the overshoot stop.
    assert openamber.set_sensor("error_register_2119_raw", 0.0)
    openamber.advance_time(seconds=10, step_s=2)
    openamber.step(ms=50)
    assert int(openamber.get_entity("compressor_control_select")) == 0, "Compressor must stop once the oil return cycle ends"
    assert advance_until_state(openamber, "state_machine_state_main", "Maintenance", max_s=600)

    openamber.set_switch("service_mode_enabled", False)
    openamber.set_switch("heat_demand_switch", False)
    openamber.advance_time(seconds=30, step_s=5)


def test_maintenance_mode_stops_compressor_without_oil_return_cycle(clean_system):
    """Without an oil return cycle, maintenance mode stops the compressor as before."""
    openamber = clean_system
    start_space_heating(openamber)
    openamber.advance_time(seconds=620, step_s=20)
    openamber.step(ms=50)

    openamber.set_switch("service_mode_enabled", True)
    openamber.advance_time(seconds=10, step_s=2)
    openamber.step(ms=50)
    assert int(openamber.get_entity("compressor_control_select")) == 0
    assert advance_until_state(openamber, "state_machine_state_main", "Maintenance", max_s=600)

    openamber.set_switch("service_mode_enabled", False)
    openamber.set_switch("heat_demand_switch", False)
    openamber.advance_time(seconds=30, step_s=5)


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


def test_maintenance_mode_stops_dhw_while_demand_remains(clean_system):
    """Maintenance mode stops DHW even while the tank still asks for heat, instead of waiting until it is warm."""
    openamber = clean_system
    _start_dhw_compressor_past_min_on_time(openamber)

    openamber.set_switch("service_mode_enabled", True)
    openamber.advance_time(seconds=10, step_s=2)
    openamber.step(ms=50)
    assert int(openamber.get_entity("compressor_control_select")) == 0, "Maintenance mode must stop the DHW compressor"
    assert advance_until_state(openamber, "state_machine_state_main", "Maintenance", max_s=600)

    openamber.set_switch("service_mode_enabled", False)
    openamber.set_sensor("dhw_temperature_tw_sensor", 55.0)
    openamber.advance_time(seconds=60, step_s=5)


def test_maintenance_mode_waits_for_oil_return_cycle_during_dhw(clean_system):
    """During DHW a running oil return cycle is also finished before maintenance mode stops the compressor."""
    openamber = clean_system
    _start_dhw_compressor_past_min_on_time(openamber)

    assert openamber.set_sensor("error_register_2119_raw", 8.0)
    openamber.step(ms=50)
    openamber.set_switch("service_mode_enabled", True)
    openamber.advance_time(seconds=60, step_s=5)
    openamber.step(ms=50)
    assert int(openamber.get_entity("compressor_control_select")) > 0, "Compressor must run until the oil return cycle ends"

    assert openamber.set_sensor("error_register_2119_raw", 0.0)
    openamber.advance_time(seconds=10, step_s=2)
    openamber.step(ms=50)
    assert int(openamber.get_entity("compressor_control_select")) == 0
    assert advance_until_state(openamber, "state_machine_state_main", "Maintenance", max_s=600)

    openamber.set_switch("service_mode_enabled", False)
    openamber.set_sensor("dhw_temperature_tw_sensor", 55.0)
    openamber.advance_time(seconds=60, step_s=5)
