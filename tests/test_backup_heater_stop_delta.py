"""Tests for the configurable backup heater stop delta (backup_heater_stop_delta)."""


def test_backup_heater_stop_delta_honored_above_compressor_delta(clean_system):
    """
    The backup heater must keep running up to setpoint + backup_heater_stop_delta even
    when that is above compressor_stop_delta_heating (emergency overshoot cap fix),
    and must shut off once the stop delta is reached.
    """
    openamber = clean_system

    target_temperature = 35.0
    assert openamber.set_select("thermostat_mode_select", "Extern")
    assert openamber.set_select("heat_mode_select", "Extern setpoint")
    assert openamber.set_number("manual_setpoint", target_temperature)
    assert openamber.set_climate("pid_heat_temperature_control", target_temperature=target_temperature)
    assert openamber.set_number("compressor_start_delta_heating", 2.0)
    assert openamber.set_number("compressor_stop_delta_heating", 1.0)
    assert openamber.set_number("backup_heater_stop_delta", 5.0)
    assert openamber.set_select("heat_compressor_mode", "Beperkt")
    assert openamber.set_select("backup_heating_mode", "Intern verwarmingselement")
    assert openamber.set_number("backup_heater_degmin_threshold", 100.0)

    for t in (32.5,):
        assert openamber.set_sensor("current_water_temperature_tc_sensor", t)
        assert openamber.set_sensor("heat_cool_temperature_tc", t)
        assert openamber.set_sensor("outlet_temperature_tuo", t)
        assert openamber.set_sensor("inlet_temperature_tui", t - 2.0)
    openamber.step(ms=50)

    openamber.advance_time(seconds=130, step_s=10)
    openamber.step(ms=50)

    assert openamber.set_binary_sensor("external_heat_demand_wired", True)
    openamber.step(ms=100)

    # Advance through pump interval + settle + softstart to get the compressor running
    openamber.advance_time(seconds=900, step_s=30)
    openamber.advance_time(seconds=130, step_s=10)
    openamber.advance_time(seconds=190, step_s=10)
    openamber.step(ms=100)
    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"
    assert openamber.get_entity("backup_heater_relay") is False

    # Trigger the backup heater through degree-minute accumulation
    current_degmin = float(openamber.get_entity("backup_heater_degmin_current_sensor") or 0)
    assert openamber.set_number("backup_heater_degmin_threshold", current_degmin + 5.0)
    for t in (20.0,):
        assert openamber.set_sensor("current_water_temperature_tc_sensor", t)
        assert openamber.set_sensor("heat_cool_temperature_tc", t)
        assert openamber.set_sensor("outlet_temperature_tuo", t)
        assert openamber.set_sensor("inlet_temperature_tui", t - 2.0)
    openamber.step(ms=50)
    openamber.advance_time(seconds=60, step_s=10)
    openamber.step(ms=100)
    assert openamber.get_entity("backup_heater_relay") is True, "Backup heater should turn ON via degmin"

    # 36.0°C is above target + compressor_stop_delta (36.0) but below target + backup
    # stop delta (40.0): the backup heater must keep running (both checks use the backup delta).
    for t in (36.0,):
        assert openamber.set_sensor("current_water_temperature_tc_sensor", t)
        assert openamber.set_sensor("heat_cool_temperature_tc", t)
        assert openamber.set_sensor("outlet_temperature_tuo", t)
        assert openamber.set_sensor("inlet_temperature_tui", t - 2.0)
    openamber.advance_time(seconds=60, step_s=10)
    openamber.step(ms=100)
    assert openamber.get_entity("backup_heater_relay") is True, "Backup heater should stay ON below target + 5.0°C"

    # Above target + backup_heater_stop_delta (40.0°C) the measured shutoff must trigger.
    for t in (40.1,):
        assert openamber.set_sensor("current_water_temperature_tc_sensor", t)
        assert openamber.set_sensor("heat_cool_temperature_tc", t)
        assert openamber.set_sensor("outlet_temperature_tuo", t)
        assert openamber.set_sensor("inlet_temperature_tui", t - 2.0)
    openamber.advance_time(seconds=5, step_s=1)
    openamber.step(ms=100)
    assert openamber.get_entity("backup_heater_relay") is False, "Backup heater should shut OFF at target + 5.0°C"

    assert openamber.set_binary_sensor("external_heat_demand_wired", False)
    openamber.step(ms=100)


def test_external_backup_mode_uses_compressor_delta_while_element_off(clean_system):
    """
    With 'Externe backup verwarming' configured but the element not driven, compressor-only
    operation must stop on compressor_stop_delta_heating, not on the (larger) backup delta.
    """
    openamber = clean_system

    target_temperature = 35.0
    assert openamber.set_select("thermostat_mode_select", "Extern")
    assert openamber.set_select("heat_mode_select", "Extern setpoint")
    assert openamber.set_number("manual_setpoint", target_temperature)
    assert openamber.set_climate("pid_heat_temperature_control", target_temperature=target_temperature)
    assert openamber.set_number("compressor_start_delta_heating", 2.0)
    assert openamber.set_number("compressor_stop_delta_heating", 1.0)
    assert openamber.set_number("backup_heater_stop_delta", 10.0)
    assert openamber.set_select("heat_compressor_mode", "Beperkt")
    assert openamber.set_select("backup_heating_mode", "Externe backup verwarming")
    # High threshold so the degmin path never turns the element on during this test
    assert openamber.set_number("backup_heater_degmin_threshold", 600.0)

    assert openamber.set_sensor("current_water_temperature_tc_sensor", 32.5)
    assert openamber.set_sensor("heat_cool_temperature_tc", 32.5)
    assert openamber.set_sensor("outlet_temperature_tuo", 32.5)
    assert openamber.set_sensor("inlet_temperature_tui", 30.5)
    openamber.step(ms=50)
    openamber.advance_time(seconds=130, step_s=10)
    openamber.step(ms=50)

    assert openamber.set_binary_sensor("external_heat_demand_wired", True)
    openamber.step(ms=100)

    openamber.advance_time(seconds=900, step_s=30)
    openamber.advance_time(seconds=130, step_s=10)
    openamber.advance_time(seconds=190, step_s=10)
    openamber.step(ms=100)
    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"
    assert openamber.get_entity("external_backup_heating_relay") is False

    # 36.1°C is above target + compressor stop delta (36.0) but far below target + backup
    # delta (45.0): with the element off, the compressor must still stop on its own delta.
    assert openamber.set_sensor("current_water_temperature_tc_sensor", 36.1)
    assert openamber.set_sensor("heat_cool_temperature_tc", 36.1)
    assert openamber.set_sensor("outlet_temperature_tuo", 36.1)
    assert openamber.set_sensor("inlet_temperature_tui", 34.1)

    stopped = False
    for _ in range(60):  # up to 30 min virtual time (covers the compressor minimum on time)
        openamber.advance_time(seconds=30, step_s=10)
        openamber.step(ms=100)
        if openamber.get_entity("state_machine_state_heat_cool") != "Compressor running":
            stopped = True
            break

    assert stopped, "Compressor must stop at target + compressor delta while the external element is off"
    assert openamber.get_entity("external_backup_heating_relay") is False

    assert openamber.set_binary_sensor("external_heat_demand_wired", False)
    openamber.step(ms=100)
