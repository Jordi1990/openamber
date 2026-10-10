"""Automated tests for Emergency Mode (Noodbedrijf).

Covers:
1. Space Heating in Emergency Mode (emergency_mode_enabled = True):
   - Compressor remains completely OFF (compressor_control_select == 0).
   - Circulation pump runs to distribute heat.
   - Electric backup heater engages automatically (backup_heater_relay = True).
   - Once water temperature satisfies setpoint, backup heater shuts off.
2. DHW in Emergency Mode:
   - 3-way valve switches to DHW circuit.
   - Compressor remains OFF.
   - Backup heater engages to heat domestic hot water.
   - DHW pump circulates water once condition is met.
3. Cooling lockout in Emergency Mode:
   - Compressor remains locked at 0 during emergency mode even if cooling is called.
"""

import pytest


def test_space_heating_emergency_mode_runs_backup_heater_without_compressor(clean_system):
    """
    Verify Space Heating Emergency Mode:
    1. Enable emergency_mode_enabled = True.
    2. Activate heat demand (Tc = 28°C < setpoint 35°C).
    3. Verify pump starts, compressor stays at 0, and backup heater turns ON.
    4. Water warms to setpoint + stop delta -> backup heater turns OFF.
    """
    openamber = clean_system

    assert openamber.set_switch("emergency_mode_enabled", True)
    assert openamber.set_select("thermostat_mode_select", "Extern")
    assert openamber.set_select("heat_mode_select", "Extern setpoint")
    assert openamber.set_number("manual_setpoint", 35.0)
    assert openamber.set_number("compressor_start_delta_heating", 2.0)
    assert openamber.set_number("compressor_stop_delta_heating", 2.0)
    assert openamber.set_sensor("current_water_temperature_tc_sensor", 28.0)
    assert openamber.set_sensor("heat_cool_temperature_tc", 28.0)
    assert openamber.set_sensor("outlet_temperature_tuo", 28.0)
    assert openamber.set_sensor("inlet_temperature_tui", 26.0)
    openamber.step(ms=50)

    # Ensure minimum compressor off-time has elapsed before starting demand
    openamber.advance_time(seconds=130, step_s=10)
    openamber.step(ms=50)

    # Trigger heat demand
    assert openamber.set_binary_sensor("external_heat_demand_wired", True)
    openamber.step(ms=100)
    assert openamber.get_entity("heat_demand_active_sensor") is True, "Heat demand should become active"

    # Advance time to trigger pump interval cycle in IDLE (pump_interval = 15 min = 900s)
    openamber.advance_time(seconds=900, step_s=30)
    openamber.step(ms=100)

    # Advance time through pump settle (COMPRESSOR_MIN_TIME_PUMP_ON = 120s)
    openamber.advance_time(seconds=140, step_s=10)
    openamber.step(ms=100)

    # Compressor must NOT run in emergency mode
    comp_mode = int(openamber.get_entity("compressor_control_select") or 0)
    assert comp_mode == 0, f"Compressor must stay at 0 in emergency mode, got {comp_mode}"

    # Backup heater relay must turn ON to provide heat
    assert openamber.get_entity("backup_heater_relay") is True, "Backup heater relay must be ON in emergency heating"
    assert openamber.get_entity("internal_pump_active") is True, "Circulation pump must be active in emergency heating"

    # Temperature warms up past setpoint (35.0) + stop delta (2.0) = 37.0°C
    assert openamber.set_sensor("current_water_temperature_tc_sensor", 38.0)
    assert openamber.set_sensor("heat_cool_temperature_tc", 38.0)
    assert openamber.set_sensor("outlet_temperature_tuo", 38.0)
    openamber.advance_time(seconds=10, step_s=2)
    openamber.step(ms=100)

    # Backup heater must turn OFF once satisfied
    assert openamber.get_entity("backup_heater_relay") is False, "Backup heater must turn OFF when setpoint is satisfied"

    # Cleanup
    assert openamber.set_binary_sensor("external_heat_demand_wired", False)
    assert openamber.set_switch("emergency_mode_enabled", False)
    openamber.step(ms=100)


def test_dhw_emergency_mode_runs_backup_heater_without_compressor(clean_system):
    """
    Verify Domestic Hot Water (DHW) Emergency Mode:
    1. Enable emergency_mode_enabled = True.
    2. Tank temperature drops below setpoint -> DHW demand activates.
    3. 3-way valve aligns to DHW circuit.
    4. Compressor stays OFF (0), backup heater turns ON.
    5. Tank temperature reaches setpoint -> backup heater turns OFF.
    """
    openamber = clean_system

    assert openamber.set_switch("emergency_mode_enabled", True)
    assert openamber.set_number("dhw_setpoint_temperature", 50.0)
    assert openamber.set_number("dhw_restart_dhw_delta", 5.0)
    assert openamber.set_sensor("dhw_temperature_tw_sensor", 38.0)
    openamber.step(ms=50)

    assert openamber.get_entity("dhw_demand_active_sensor") is True, "DHW demand should be active when cold"

    # Advance virtual time through valve switch time (60s) + settle (60s)
    openamber.advance_time(seconds=130, step_s=10)
    openamber.step(ms=100)

    # 3-way valve aligned to DHW
    assert openamber.get_entity("three_way_valve_dhw_switch") is True, "3-way valve must switch to DHW in emergency mode"

    # Compressor must remain OFF (0)
    comp_mode = int(openamber.get_entity("compressor_control_select") or 0)
    assert comp_mode == 0, f"Compressor must stay at 0 in emergency mode during DHW, got {comp_mode}"

    # Backup heater must be active for DHW heating
    assert openamber.get_entity("backup_heater_relay") is True, "Backup heater relay must be ON for DHW emergency mode"

    # Tank temperature reaches setpoint
    assert openamber.set_sensor("dhw_temperature_tw_sensor", 51.0)
    # DHW pump temperature settle time is 120s (DHW_PUMP_TEMPERATURE_SETTLE_TIME_S)
    openamber.advance_time(seconds=140, step_s=10)
    openamber.step(ms=100)

    # Backup heater turns OFF
    assert openamber.get_entity("backup_heater_relay") is False, "Backup heater must turn OFF once DHW is satisfied"

    # Cleanup
    assert openamber.set_switch("emergency_mode_enabled", False)
    openamber.step(ms=100)


def test_cooling_compressor_lockout_in_emergency_mode(clean_system):
    """
    Edge Case:
    When emergency_mode_enabled is True, cooling cannot run the compressor.
    Verify compressor remains at 0 even under cooling demand.
    """
    openamber = clean_system

    assert openamber.set_switch("emergency_mode_enabled", True)
    assert openamber.set_select("thermostat_mode_select", "Extern")
    assert openamber.set_switch("cool_demand_switch", True)
    assert openamber.set_sensor("current_water_temperature_tc_sensor", 28.0)
    assert openamber.set_sensor("heat_cool_temperature_tc", 28.0)
    openamber.step(ms=50)

    openamber.advance_time(seconds=130, step_s=10)
    openamber.advance_time(seconds=900, step_s=30)
    openamber.advance_time(seconds=140, step_s=10)
    openamber.step(ms=100)

    comp_mode = int(openamber.get_entity("compressor_control_select") or 0)
    assert comp_mode == 0, f"Compressor must not run for cooling in emergency mode, got {comp_mode}"

    # Cleanup
    assert openamber.set_switch("cool_demand_switch", False)
    assert openamber.set_switch("emergency_mode_enabled", False)
    openamber.step(ms=50)

