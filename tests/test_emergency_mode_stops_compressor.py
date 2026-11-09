"""Emergency mode enabled while the compressor runs.

Emergency mode is only checked when a controller would start the compressor. When it is enabled while the
compressor is requested, OpenAmber stops everything and re-initializes, so regulation starts again in emergency
mode with only the backup heater. No error is raised.
"""

from helpers import advance_until_state, start_space_heating


def compressor_mode(openamber):
    return int(openamber.get_entity("compressor_control_select") or 0)


def test_enabling_emergency_mode_during_heating_stops_compressor(clean_system):
    """Heating continues with the backup heater only; the compressor is stopped at once."""
    openamber = clean_system
    start_space_heating(openamber)
    openamber.set_select("backup_heating_mode", "Intern verwarmingselement")
    assert compressor_mode(openamber) > 0

    openamber.set_switch("emergency_mode_enabled", True)
    openamber.advance_time(seconds=10, step_s=1)
    openamber.step(ms=50)
    assert compressor_mode(openamber) == 0, "Enabling emergency mode must stop the compressor"
    assert openamber.get_entity("error_active") is False, "Emergency mode is no error"

    # After the re-initialization heating runs on the backup heater, without the compressor.
    assert advance_until_state(openamber, "backup_heater_relay", True, max_s=1800, step=10), (
        f"Heating never used the backup heater (heat/cool: {openamber.get_entity('state_machine_state_heat_cool')})"
    )
    assert compressor_mode(openamber) == 0
    assert openamber.get_entity("state_machine_state_main") == "Heat/Cool"

    openamber.set_switch("emergency_mode_enabled", False)
    openamber.set_switch("heat_demand_switch", False)
    openamber.advance_time(seconds=60, step_s=5)


def test_enabling_emergency_mode_during_dhw_stops_compressor(clean_system):
    """DHW continues with the backup heater only; the compressor is stopped at once."""
    openamber = clean_system
    openamber.set_number("dhw_setpoint_temperature", 50.0)
    openamber.set_number("dhw_restart_dhw_delta", 5.0)
    openamber.set_sensor("dhw_temperature_tw_sensor", 40.0)
    openamber.step(ms=100)
    assert advance_until_state(openamber, "state_machine_state_dhw", "Compressor running", max_s=900, step=5)

    openamber.set_switch("emergency_mode_enabled", True)
    openamber.advance_time(seconds=10, step_s=1)
    openamber.step(ms=50)
    assert compressor_mode(openamber) == 0, "Enabling emergency mode must stop the compressor"

    assert advance_until_state(openamber, "backup_heater_relay", True, max_s=900, step=10), (
        f"DHW never used the backup heater (dhw: {openamber.get_entity('state_machine_state_dhw')})"
    )
    assert compressor_mode(openamber) == 0
    assert openamber.get_entity("state_machine_state_main") == "DHW"

    openamber.set_switch("emergency_mode_enabled", False)
    openamber.set_sensor("dhw_temperature_tw_sensor", 55.0)
    assert advance_until_state(openamber, "state_machine_state_dhw", "Idle", max_s=900)


def test_emergency_mode_without_compressor_does_not_reinitialize(clean_system):
    """With emergency mode already on, the compressor is never requested, so the check never fires."""
    openamber = clean_system
    openamber.set_switch("emergency_mode_enabled", True)
    openamber.set_select("backup_heating_mode", "Intern verwarmingselement")
    openamber.set_select("thermostat_mode_select", "Extern")
    openamber.set_select("heat_mode_select", "Extern setpoint")
    openamber.set_number("manual_setpoint", 35.0)
    openamber.set_climate("pid_heat_temperature_control", target_temperature=35.0)
    openamber.set_sensor("heat_cool_temperature_tc", 28.0)
    openamber.set_switch("heat_demand_switch", True)
    assert advance_until_state(openamber, "backup_heater_relay", True, max_s=1800, step=10)

    # Stays in regulation: no re-initialization loop.
    for _ in range(30):
        openamber.advance_time(seconds=10, step_s=5)
        openamber.step(ms=20)
        assert openamber.get_entity("state_machine_state_main") == "Heat/Cool"
        assert compressor_mode(openamber) == 0

    openamber.set_switch("emergency_mode_enabled", False)
    openamber.set_switch("heat_demand_switch", False)
    openamber.advance_time(seconds=60, step_s=5)
