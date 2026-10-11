"""Tests for virtual time advancement, timers, and settle time in OpenAmber."""

import pytest


def test_virtual_time_engine(clean_system):
    """Verify that virtual time can be queried and advanced monotonically."""
    openamber = clean_system
    info = openamber.get_time()
    assert info.get("status") == "ok", "Virtual time query should succeed"
    initial_millis = info.get("millis", 0)
    initial_offset = info.get("offset_ms", 0)

    # Advance time by 300 seconds (5 minutes)
    offset = openamber.advance_time(seconds=300)
    assert offset >= initial_offset + 300000, "Offset must increase by at least 300,000 ms"

    new_info = openamber.get_time()
    assert new_info.get("millis", 0) >= initial_millis + 300000, "Virtual millis must advance by >= 300,000 ms"
    assert new_info.get("offset_ms", 0) >= initial_offset + 300000, "Virtual offset must advance by >= 300,000 ms"


def test_backup_heater_prediction_settle_time_flow(clean_system):
    """
    Workflow Test with Virtual Time & Settle Time:
    1. Backup heater turns ON via heating degree-minute accumulation.
    2. Before BACKUP_HEATER_PREDICTION_SETTLE_TIME_S (300s), prediction shutoff is suppressed.
    3. Advance virtual time past settle time (total > 300s).
    4. Controller shutoff condition triggers and heater turns OFF automatically.
    5. UI badge updates to 'UIT' and switch widget unchecks.
    """
    openamber = clean_system

    target_temperature = 35.0
    assert openamber.set_select("thermostat_mode_select", "Extern")
    assert openamber.set_select("heat_mode_select", "Extern setpoint")
    assert openamber.set_number("manual_setpoint", target_temperature)
    assert openamber.set_climate("pid_heat_temperature_control", target_temperature=target_temperature)
    assert openamber.set_number("compressor_start_delta_heating", 2.0)
    assert openamber.set_number("compressor_stop_delta_heating", 5.0)
    assert openamber.set_select("heat_compressor_mode", "Beperkt")
    assert openamber.set_select("backup_heating_mode", "Intern verwarmingselement")
    # Keep threshold high during startup so backup heater does not turn on early
    assert openamber.set_number("backup_heater_degmin_threshold", 100.0)

    # Initial supply temperature: delta = 2.5°C below target (triggers compressor start delta 2.0°C)
    assert openamber.set_sensor("current_water_temperature_tc_sensor", 32.5)
    assert openamber.set_sensor("heat_cool_temperature_tc", 32.5)
    assert openamber.set_sensor("outlet_temperature_tuo", 32.5)
    assert openamber.set_sensor("inlet_temperature_tui", 30.5)
    openamber.step(ms=50)

    # Ensure minimum compressor off-time has elapsed
    openamber.advance_time(seconds=130, step_s=10)
    openamber.step(ms=50)

    # Trigger space heat demand
    assert openamber.set_binary_sensor("external_heat_demand_wired", True)
    openamber.step(ms=100)
    assert openamber.get_entity("heat_demand_active_sensor") is True, "Heat demand should become active"

    # Advance time through pump interval in IDLE (900s) + pump settle (130s) + softstart (190s)
    openamber.advance_time(seconds=900, step_s=30)
    openamber.advance_time(seconds=130, step_s=10)
    openamber.advance_time(seconds=190, step_s=10)
    openamber.step(ms=100)
    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running", "Compressor should be running"
    assert openamber.get_entity("backup_heater_relay") is False, "Backup heater should not be on yet"

    # Lower supply temperature to 20°C and set threshold to current + 5.0 to trigger backup heater
    current_degmin = float(openamber.get_entity("backup_heater_degmin_current_sensor") or 0)
    assert openamber.set_number("backup_heater_degmin_threshold", current_degmin + 5.0)
    assert openamber.set_sensor("current_water_temperature_tc_sensor", 20.0)
    assert openamber.set_sensor("heat_cool_temperature_tc", 20.0)
    assert openamber.set_sensor("outlet_temperature_tuo", 20.0)
    assert openamber.set_sensor("inlet_temperature_tui", 18.0)
    openamber.step(ms=50)

    # Advance time to accumulate degree-minutes and trigger backup heater (60s with delta 15°C -> 15 °C*min > 10 °C*min)
    openamber.advance_time(seconds=60, step_s=10)
    openamber.step(ms=100)

    # Backup heater turns ON via controller (marking start of 300s settle time)
    assert openamber.get_entity("backup_heater_relay") is True, "Backup heater relay should turn ON via degmin"
    assert openamber.get_widget("service_backup_heater_relay_switch_ui").get("checked") is True, "Switch should be checked"
    assert openamber.get_label("tile_backup_state") == "AAN", "Tile label should be 'AAN'"

    # Establish prediction-shutoff condition before the 30-second assertion:
    # Set supply temperature to 37.2°C (predicted >= target 35.0 + 2.0 = 37.0°C),
    # while keeping current temp below high-temperature shutoff threshold (35.0 + 5.0 = 40.0°C).
    assert openamber.set_sensor("current_water_temperature_tc_sensor", 37.2)
    assert openamber.set_sensor("heat_cool_temperature_tc", 37.2)
    assert openamber.set_sensor("outlet_temperature_tuo", 37.2)
    assert openamber.set_sensor("inlet_temperature_tui", 35.2)
    openamber.step(ms=50)

    # Advance virtual time by only 30 seconds (30s < BACKUP_HEATER_PREDICTION_SETTLE_TIME_S = 300s)
    openamber.advance_time(seconds=30, step_s=5)
    openamber.step(ms=50)

    # Settle time not elapsed yet (30s < 300s), prediction shutoff is suppressed: backup heater must remain ON
    assert openamber.get_entity("backup_heater_relay") is True, "Backup heater should stay ON before settle time (300s)"
    assert openamber.get_label("tile_backup_state") == "AAN", "Tile label should remain 'AAN'"

    # Advance virtual time by an additional 280 seconds (total 310s > 300s settle time)
    openamber.advance_time(seconds=280, step_s=10)
    openamber.step(ms=50)

    # Settle time elapsed (> 300s): prediction shutoff triggers and turns OFF backup heater
    assert openamber.get_entity("backup_heater_relay") is False, "Controller must shut OFF backup heater via prediction shutoff"
    assert openamber.get_widget("service_backup_heater_relay_switch_ui").get("checked") is False, "Switch should be unchecked"
    assert openamber.get_label("tile_backup_state") == "UIT", "Tile label should be 'UIT'"

    # Cleanup
    assert openamber.set_binary_sensor("external_heat_demand_wired", False)
    openamber.step(ms=100)

