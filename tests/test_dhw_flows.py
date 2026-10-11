"""Automated tests for Domestic Hot Water (DHW) flows.
Only tests real user inputs (temperatures, setpoints, user switches/selects).
"""

import pytest


def test_dhw_demand_while_idle_heats_dhw(clean_system):
    """
    User Flow:
    1. System is in idle state (Tw >= setpoint).
    2. DHW tank temperature drops below setpoint - delta_restart (e.g. 40°C with setpoint 50°C).
    3. DHW demand activates naturally (dhw_demand_active_sensor becomes True).
    4. Controller switches 3-way valve to DHW circuit (three_way_valve_dhw_switch = True).
    5. Compressor runs for DHW heating.
    """
    openamber = clean_system

    # Verify initially no demand
    assert openamber.get_entity("dhw_demand_active_sensor") is False, "DHW demand should initially be False"
    assert openamber.get_entity("three_way_valve_dhw_switch") is False, "3-way valve should initially be aligned to heating"

    # User tank temperature drops
    openamber.set_number("dhw_setpoint_temperature", 50.0)
    openamber.set_number("dhw_restart_dhw_delta", 5.0)
    openamber.set_sensor("dhw_temperature_tw_sensor", 40.0)
    openamber.step(ms=100)

    # Demand must be generated naturally
    assert openamber.get_entity("dhw_demand_active_sensor") is True, "DHW demand should activate when tank drops below setpoint - delta"

    # Advance time through 3-way valve switch time (60s) and state transition
    openamber.advance_time(seconds=80, step_s=10)
    openamber.step(ms=100)

    # Valve must switch to DHW
    assert openamber.get_entity("three_way_valve_dhw_switch") is True, "3-way valve should switch to DHW"
    assert openamber.get_entity("three_way_valve_active_sensor") is True, "3-way valve sensor should indicate DHW active"

    # Advance virtual time for DHW compressor start
    openamber.advance_time(seconds=80, step_s=10)
    openamber.step(ms=100)

    assert openamber.get_entity("compressor_control_select") is not None, "Compressor control select should not be None"
    assert int(openamber.get_entity("compressor_control_select")) > 0, "Compressor should run for DHW"


def test_dhw_demand_while_heating_switches_to_dhw(clean_system):
    """
    Priority Flow:
    1. Space heating is actively running (heat demand active).
    2. DHW demand occurs (tank temperature drops).
    3. Space heating respects compressor min-on-time (600s), then stops and switches 3-way valve to DHW.
    """
    openamber = clean_system

    # Start space heating demand
    openamber.set_switch("heat_demand_switch", True)
    openamber.set_sensor("heat_cool_temperature_tc", 28.0)
    openamber.step(ms=100)

    # Advance time to start space heating pump and compressor
    openamber.advance_time(seconds=120, step_s=10)
    openamber.step(ms=100)

    # Now DHW demand occurs (Tw = 38°C < setpoint 50°C - delta 5°C)
    openamber.set_sensor("dhw_temperature_tw_sensor", 38.0)
    openamber.step(ms=100)
    assert openamber.get_entity("dhw_demand_active_sensor") is True, "DHW demand should be active when tank drops"

    # Advance virtual time through space heating compressor min-on-time (600s) + valve switch (60s)
    openamber.advance_time(seconds=720, step_s=20)
    openamber.step(ms=100)

    # 3-way valve must now be aligned to DHW circuit
    assert openamber.get_entity("three_way_valve_dhw_switch") is True, "3-way valve should switch to DHW after heating min-on-time"


def test_dhw_demand_stops_when_temperature_reached(clean_system):
    """
    Shutoff Flow:
    1. System is heating DHW.
    2. DHW tank temperature reaches setpoint (e.g. 52°C >= 50°C).
    3. DHW demand turns OFF naturally.
    4. Once compressor passes min-on-time (600s), compressor stops and DHW pump stops.
    """
    openamber = clean_system

    # Start DHW heating
    openamber.set_number("dhw_setpoint_temperature", 50.0)
    openamber.set_sensor("dhw_temperature_tw_sensor", 40.0)
    openamber.step(ms=100)
    assert openamber.get_entity("dhw_demand_active_sensor") is True, "DHW demand should be active"

    openamber.advance_time(seconds=120, step_s=10)
    openamber.step(ms=100)
    assert openamber.get_entity("three_way_valve_dhw_switch") is True, "3-way valve should be on DHW"
    assert int(openamber.get_entity("compressor_control_select") or 0) > 0, "Compressor should be running"

    # Tank reaches setpoint
    openamber.set_sensor("dhw_temperature_tw_sensor", 52.0)
    openamber.step(ms=100)

    # Demand ceases naturally
    assert openamber.get_entity("dhw_demand_active_sensor") is False, "DHW demand should cease when tank reaches setpoint"

    # Advance time past compressor minimum on time (600s) + shutdown settle
    openamber.advance_time(seconds=620, step_s=20)
    openamber.step(ms=100)

    assert int(openamber.get_entity("compressor_control_select") or 0) == 0, "Compressor should stop after min-on-time"
    assert openamber.get_entity("dhw_pump_relay_switch") is False, "DHW pump should stop when demand ceases"


def test_dhw_backup_heater_turns_on_if_not_heating_properly(clean_system):
    """
    Fault/Backup Flow:
    1. User configures backup heating rate check: min rate 0.2 °C/min, delay 5 min.
    2. DHW heating starts, but water does not warm up properly (temperature stays flat).
    3. After pump settle (2 min) + grace period (10 min) + rate delay (5 min) = 17 min (~1020s),
       backup heater turns ON automatically.
    """
    openamber = clean_system

    openamber.set_number("dhw_setpoint_temperature", 50.0)
    openamber.set_number("dhw_restart_dhw_delta", 5.0)
    openamber.set_number("dhw_backup_min_avg_rate", 0.2)
    openamber.set_number("dhw_backup_min_avg_rate_delay_minutes", 5.0)
    openamber.set_sensor("dhw_temperature_tw_sensor", 40.0)
    openamber.step(ms=100)

    # Start DHW and start pump
    openamber.advance_time(seconds=100, step_s=10)
    openamber.step(ms=100)
    assert openamber.get_entity("three_way_valve_dhw_switch") is True, "3-way valve should be on DHW"

    # Backup heater should not be on immediately during grace period
    assert openamber.get_entity("backup_heater_relay") is False, "Backup heater should not turn on during grace period"

    # Advance past pump settle (120s) + grace period (600s) + delay (300s)
    openamber.advance_time(seconds=1150, step_s=20)
    openamber.step(ms=100)

    # Backup heater must have activated
    assert openamber.get_entity("backup_heater_relay") is True, "Backup heater should activate when heating rate is too low"
    assert openamber.get_entity("backup_heater_stage_1") is True, "Backup heater stage 1 should be active"


def test_dhw_pump_direct_mode(clean_system):
    """
    Direct Mode Flow:
    Starts pump immediately together with DHW compressor even if supply is colder than vat.
    """
    openamber = clean_system

    openamber.set_select("dhw_pump_start_mode_select", "Samen met compressor")
    openamber.set_sensor("dhw_temperature_tw_sensor", 40.0)
    openamber.set_sensor("outlet_temperature_tuo", 30.0)  # Supply colder than vat
    openamber.step(ms=100)

    openamber.advance_time(seconds=150, step_s=10)
    openamber.step(ms=100)

    # Pump must be active in direct mode
    assert openamber.get_entity("dhw_pump_relay_switch") is True, "DHW pump relay should be True in direct mode"
    assert openamber.get_entity("dhw_pump_active_sensor") is True, "DHW pump active sensor should be True in direct mode"


def test_dhw_pump_delta_t_mode(clean_system):
    """
    Delta-T Mode Flow:
    Keeps pump off while supply temperature (Tuo) < vat temperature (Tw).
    Starts pump once supply heats up to Tuo >= Tw.
    """
    openamber = clean_system

    openamber.set_select("dhw_pump_start_mode_select", "Aanvoer warmer dan vat (ΔT)")
    openamber.set_sensor("dhw_temperature_tw_sensor", 40.0)
    openamber.set_sensor("outlet_temperature_tuo", 30.0)  # Supply 30 < Vat 40
    openamber.step(ms=100)

    openamber.advance_time(seconds=150, step_s=10)
    openamber.step(ms=100)

    # In Delta-T mode, pump must wait because supply is colder than tank even though compressor runs
    assert int(openamber.get_entity("compressor_control_select") or 0) > 0, "Compressor should run"
    assert openamber.get_entity("dhw_pump_relay_switch") is False, "Pump should be OFF when supply is colder than tank"

    # Now supply heats up above vat temp
    openamber.set_sensor("outlet_temperature_tuo", 45.0)  # Supply 45 >= Vat 40
    openamber.set_sensor("inlet_temperature_tui", 35.0)  # Maintain Tuo - Tui <= 15°C safety limit
    openamber.advance_time(seconds=20, step_s=5)
    openamber.step(ms=100)

    # Pump must start once Tuo >= Tw
    assert openamber.get_entity("dhw_pump_relay_switch") is True, "Pump should start once supply exceeds tank temp"


@pytest.mark.parametrize("mode_name,expected_index", [
    ("Beperkt", 4),
    ("Laag", 6),
    ("Gemiddeld", 7),
    ("Maximaal", 10),
])
def test_dhw_compressor_limit_modes(clean_system, mode_name, expected_index):
    """
    Compressor Limit Modes Variations:
    Verify that selecting a compressor limit mode configures the compressor to the corresponding index.
    """
    openamber = clean_system

    openamber.set_select("dhw_compressor_mode", mode_name)
    openamber.set_sensor("temperature_outside_ta", 10.0)  # Ta 10 > threshold 5 -> normal base mode
    openamber.set_sensor("dhw_temperature_tw_sensor", 38.0)
    openamber.step(ms=50)

    # Advance time for 3-way valve switch (60s) + compressor start
    openamber.advance_time(seconds=140, step_s=10)
    openamber.step(ms=50)

    current_mode = int(openamber.get_entity("compressor_control_select") or 0)
    assert current_mode == expected_index, (
        f"Mode {mode_name} should set compressor to index {expected_index}, got {current_mode}"
    )


def test_dhw_compressor_limit_winter_mode(clean_system):
    """
    Winter Limit Mode Flow:
    When ambient temperature drops to or below threshold, controller overrides base mode with winter mode.
    """
    openamber = clean_system

    openamber.set_number("dhw_temperature_threshold_max_compressor_mode", 5.0)
    openamber.set_select("dhw_compressor_mode", "Beperkt")  # Base mode would be index 4
    openamber.set_select("dhw_compressor_mode_max", "Maximaal")  # Winter mode is index 10
    openamber.set_sensor("temperature_outside_ta", 0.0)  # Ta 0.0 <= threshold 5.0 -> winter mode
    openamber.set_sensor("dhw_temperature_tw_sensor", 38.0)
    openamber.step(ms=50)

    openamber.advance_time(seconds=140, step_s=10)
    openamber.step(ms=50)

    current_mode = int(openamber.get_entity("compressor_control_select") or 0)
    assert current_mode == 10, f"Winter mode should set compressor to index 10 (Maximaal), got {current_mode}"
