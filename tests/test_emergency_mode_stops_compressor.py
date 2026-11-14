"""Emergency mode enabled while the compressor runs.

Emergency mode is only checked when a controller would start the compressor. When it is enabled while the
compressor is requested, OpenAmber stops everything and re-initializes, so regulation starts again in emergency
mode with only the backup heater. No error is raised.
"""

import pytest

from helpers import advance_until_state, start_space_heating

INIT_STATES = ("Initializing", "Waiting for state switch")


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


def test_restart_in_emergency_mode_does_not_resume_running_compressor(clean_system):
    """After an ESP restart with emergency mode on, the running compressor is not taken over (RESUME_CHECK) but
    the controller initializes and continues on the backup heater. Stale readbacks after the emergency stop take
    the same path."""
    openamber = clean_system
    start_space_heating(openamber)
    openamber.set_select("backup_heating_mode", "Intern verwarmingselement")
    assert compressor_mode(openamber) > 0

    # Emergency mode is switched on and the ESP restarts before the next controller update.
    openamber.set_switch("emergency_mode_enabled", True)
    assert openamber.set_switch("mock_simulate_esp_restart", True)
    openamber.step(ms=50)

    mains = []
    for _ in range(60):
        openamber.advance_time(seconds=1, step_s=1)
        openamber.step(ms=20)
        main = openamber.get_entity("state_machine_state_main")
        if not mains or mains[-1] != main:
            mains.append(main)
    # Directly from the resume check into initialization, not first taken over as "Heat/Cool".
    first_init = next((i for i, m in enumerate(mains) if m in INIT_STATES), None)
    assert first_init is not None, f"Expected a normal initialization, got {mains}"
    assert "Heat/Cool" not in mains[:first_init], f"The running compressor was taken over first: {mains}"
    assert compressor_mode(openamber) == 0, "The compressor must be stopped in emergency mode"

    openamber.set_switch("emergency_mode_enabled", False)
    openamber.set_switch("heat_demand_switch", False)
    openamber.advance_time(seconds=60, step_s=5)


def test_disabling_emergency_mode_during_backup_heating_restarts_with_compressor(clean_system):
    """Emergency mode is switched off while heating regulates on the backup heater alone.

    The compressor is only started from the pump cycle. The backup heater stops, the controller returns to
    'Pump running' and starts the compressor the normal way (minimum off time, soft start), without
    re-initializing. It used to stay on the backup heater alone until the supply reached the stop temperature.
    """
    openamber = clean_system
    openamber.set_select("heat_mode_select", "Extern setpoint")
    openamber.set_number("manual_setpoint", 35.0)
    openamber.set_climate("pid_heat_temperature_control", target_temperature=35.0)
    openamber.set_number("compressor_start_delta_heating", 2.0)
    openamber.set_sensor("heat_cool_temperature_tc", 28.0)
    openamber.set_sensor("outlet_temperature_tuo", 28.0)
    openamber.set_sensor("inlet_temperature_tui", 26.0)
    openamber.set_switch("emergency_mode_enabled", True)
    openamber.set_switch("heat_demand_switch", True)
    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Backup heater running", max_s=1800, step=5)
    assert compressor_mode(openamber) == 0

    openamber.set_switch("emergency_mode_enabled", False)
    openamber.advance_time(seconds=10, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity("backup_heater_relay") is False, "The backup heater stops with emergency mode"
    assert compressor_mode(openamber) == 0
    assert openamber.get_entity("state_machine_state_main") == "Heat/Cool", "No re-initialization"
    assert openamber.get_entity("internal_pump_active") is True, "The pump keeps running for the compressor start"
    assert openamber.get_entity("error_active") is False

    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Pump running", max_s=150, step=5)
    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Compressor softstart", max_s=400, step=5), (
        f"Heating did not restart on the compressor (heat/cool: {openamber.get_entity('state_machine_state_heat_cool')})"
    )
    assert openamber.get_entity("state_machine_state_main") == "Heat/Cool"
    assert openamber.get_entity("backup_heater_relay") is False
    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Compressor running", max_s=300, step=5)


@pytest.mark.parametrize("seconds_in_backup_state", [60, 300], ids=["during_dhw_pump_settle_time", "backup_heater_running"])
def test_disabling_emergency_mode_during_dhw_backup_heating_restarts_with_compressor(clean_system, seconds_in_backup_state):
    """Same for DHW, 60 s (DHW pump settle time) and 300 s (backup heater state) after the backup heater started.

    The backup heater state used to request the compressor at once when emergency mode went off: without the
    minimum off time, without waiting for it to run and without a start time, so the minimum on time and the
    unexpected stop detection did not apply to that run.
    """
    openamber = clean_system
    openamber.set_number("dhw_backup_min_avg_rate", 0.0)
    openamber.set_switch("emergency_mode_enabled", True)
    openamber.set_sensor("dhw_temperature_tw_sensor", 38.0)
    assert advance_until_state(openamber, "state_machine_state_dhw", "Backup heater running", max_s=900, step=5)
    openamber.advance_time(seconds=seconds_in_backup_state, step_s=5)
    openamber.step(ms=50)
    assert compressor_mode(openamber) == 0
    assert openamber.get_entity("backup_heater_relay") is True

    openamber.set_switch("emergency_mode_enabled", False)
    for _ in range(30):
        openamber.advance_time(seconds=5, step_s=5)
        openamber.step(ms=20)
        if openamber.get_entity("backup_heater_relay") is False:
            break
        assert compressor_mode(openamber) == 0, "The compressor must not be requested straight from the backup heater state"
    assert openamber.get_entity("backup_heater_relay") is False, "The backup heater stops with emergency mode"
    assert compressor_mode(openamber) == 0
    assert openamber.get_entity("state_machine_state_main") == "DHW", "No re-initialization"
    assert openamber.get_entity("error_active") is False

    assert advance_until_state(openamber, "state_machine_state_dhw", "Wait compressor running", max_s=400, step=5), (
        f"DHW did not restart on the compressor (dhw: {openamber.get_entity('state_machine_state_dhw')})"
    )
    assert openamber.get_entity("backup_heater_relay") is False
    assert advance_until_state(openamber, "state_machine_state_dhw", "Compressor running", max_s=300, step=5)
    assert openamber.get_entity("state_machine_state_main") == "DHW"
