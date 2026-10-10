"""Tests for heat demand flow, expectation to heat, SG Ready blocking, and error suppression."""

import pytest


def test_heat_demand_expect_to_heat_happy_flow(clean_system):
    """
    Happy Flow:
    1. Outside temperature is low (5.0°C).
    2. Thermostat activates heat demand.
    3. Three-way valve is aligned to Heating circuit (CV).
    4. System registers demand and expects to heat.
    5. UI shows heating active in the status bar.
    """
    openamber = clean_system

    assert openamber.set_select("thermostat_mode_select", "Extern")
    assert openamber.set_sensor("temperature_outside_ta", 5.0)
    assert openamber.set_sensor("current_water_temperature_tc_sensor", 25.0)
    assert openamber.set_sensor("heat_cool_temperature_tc", 25.0)
    assert openamber.set_sensor("outlet_temperature_tuo", 25.0)
    assert openamber.set_sensor("inlet_temperature_tui", 23.0)
    assert openamber.set_switch("three_way_valve_heat_cool_switch", True)
    assert openamber.set_binary_sensor("sg_ready_block_mode_active_sensor", False)
    assert openamber.set_binary_sensor("error_active", False)

    # Thermostat calls for heat
    assert openamber.set_binary_sensor("external_heat_demand_wired", True)
    openamber.step(ms=150)

    assert openamber.get_entity("heat_demand_active_sensor") is True, (
        "heat_demand_active_sensor should be True when external heat demand contact is closed"
    )
    assert openamber.is_visible("nav_status_heat_icon"), (
        "nav_status_heat_icon should be visible in navbar when heating demand is active"
    )

    # Cleanup
    assert openamber.set_binary_sensor("external_heat_demand_wired", False)
    openamber.step(ms=150)


def test_heat_demand_sg_ready_block_suppression(clean_system):
    """
    Safety / Grid Flow:
    1. Active heat demand is running.
    2. Grid operator signals SG Ready Block mode (Lock/Curtailment).
    3. Verify heat demand is immediately suppressed, despite thermostat calling for heat.
    4. SG Ready block clears -> heat demand resumes.
    """
    openamber = clean_system

    assert openamber.set_select("thermostat_mode_select", "Extern")
    assert openamber.set_binary_sensor("error_pump_start_timeout", False)
    assert openamber.set_binary_sensor("external_heat_demand_wired", True)
    assert openamber.set_select("sg_ready_mode_select", "Normaal")
    openamber.step(ms=150)

    # Demand is active
    assert openamber.get_entity("heat_demand_active_sensor") is True, (
        "Heat demand should initially be active"
    )
    assert openamber.is_visible("nav_status_heat_icon"), (
        "Navbar flame icon should be visible"
    )

    # Grid block occurs
    assert openamber.set_select("sg_ready_mode_select", "Blokkeren")
    openamber.step(ms=150)

    # Demand must be blocked
    assert openamber.get_entity("heat_demand_active_sensor") is False, (
        "Heat demand must be blocked when SG Ready block is active"
    )
    assert openamber.is_hidden("nav_status_heat_icon"), (
        "Navbar flame icon must be hidden during SG Ready block"
    )

    # Grid block lifts
    assert openamber.set_select("sg_ready_mode_select", "Normaal")
    openamber.step(ms=150)

    # Demand resumes
    assert openamber.get_entity("heat_demand_active_sensor") is True, (
        "Heat demand must resume once SG Ready block lifts"
    )
    assert openamber.is_visible("nav_status_heat_icon"), (
        "Navbar flame icon must reappear once block lifts"
    )

    # Cleanup
    assert openamber.set_binary_sensor("external_heat_demand_wired", False)
    openamber.step(ms=150)


def test_heat_demand_error_active_suppression(clean_system):
    """
    Safety Flow:
    1. Active heat demand is running.
    2. A system fault occurs (error_active = True).
    3. Verify heat demand is suppressed to protect equipment.
    4. Fault is cleared -> heat demand recovers.
    """
    openamber = clean_system

    assert openamber.set_select("thermostat_mode_select", "Extern")
    assert openamber.set_select("sg_ready_mode_select", "Normaal")
    assert openamber.set_binary_sensor("external_heat_demand_wired", True)
    assert openamber.set_binary_sensor("error_pump_start_timeout", False)
    openamber.step(ms=150)

    assert openamber.get_entity("heat_demand_active_sensor") is True, (
        "Heat demand should initially be active"
    )

    # Error triggers
    assert openamber.set_binary_sensor("error_pump_start_timeout", True)
    openamber.step(ms=150)

    # Demand must shut down
    assert openamber.get_entity("heat_demand_active_sensor") is False, (
        "Heat demand must be suppressed when error is active"
    )
    assert openamber.is_hidden("nav_status_heat_icon"), (
        "Navbar flame icon must be hidden when error is active"
    )

    # Error clears
    assert openamber.set_binary_sensor("error_pump_start_timeout", False)
    openamber.step(ms=150)

    assert openamber.get_entity("heat_demand_active_sensor") is True, (
        "Heat demand should recover after error clears"
    )

    # Cleanup
    assert openamber.set_binary_sensor("external_heat_demand_wired", False)
    openamber.step(ms=150)


def test_heat_demand_compressor_continues_heating_below_stop_delta(clean_system):
    """
    Stop Delta Test 1:
    1. System is configured with:
       - Target temperature: 35.0°C (Extern setpoint)
       - Compressor start delta: 3.0°C
       - Compressor stop delta: 5.0°C
       - Initial supply Tc: 28.0°C (< target 35.0 - start_delta 3.0 = 32.0°C)
    2. Heat demand is activated (external_heat_demand_wired = True).
    3. Pump settles (120s) and compressor starts, passing softstart (180s) and min-on time (600s).
    4. Tc reaches 39.0°C (1 degree below target 35.0 + stop_delta 5.0 = 40.0°C).
    5. Verify the compressor continues to heat (compressor_control_select > 0 and state is Compressor running).
    """
    openamber = clean_system

    # Configure setpoint and deltas
    target_temperature = 35.0
    start_delta = 3.0
    stop_delta = 5.0

    openamber.set_select("thermostat_mode_select", "Extern")
    openamber.set_select("heat_mode_select", "Extern setpoint")
    openamber.set_number("manual_setpoint", target_temperature)
    openamber.set_climate("pid_heat_temperature_control", target_temperature=target_temperature)
    openamber.set_number("compressor_start_delta_heating", start_delta)
    openamber.set_number("compressor_stop_delta_heating", stop_delta)

    # Ensure minimum compressor off-time has elapsed before starting demand
    openamber.advance_time(seconds=130, step_s=10)
    openamber.step(ms=50)

    # Initial water temperatures: Tc < target - start_delta (28.0 < 32.0)
    openamber.set_sensor("current_water_temperature_tc_sensor", 28.0)
    openamber.set_sensor("heat_cool_temperature_tc", 28.0)
    openamber.set_sensor("outlet_temperature_tuo", 28.0)
    openamber.set_sensor("inlet_temperature_tui", 26.0)
    openamber.step(ms=100)

    # Thermostat calls for heat
    openamber.set_binary_sensor("external_heat_demand_wired", True)
    openamber.step(ms=100)
    assert openamber.get_entity("heat_demand_active_sensor") is True

    # Advance time to trigger pump interval cycle in IDLE (pump_interval = 15 min = 900s)
    openamber.advance_time(seconds=900, step_s=30)
    openamber.step(ms=100)

    # Advance time for pump temperature settle (COMPRESSOR_MIN_TIME_PUMP_ON = 120s)
    openamber.advance_time(seconds=130, step_s=10)
    openamber.step(ms=100)


    # Advance time through compressor soft start (COMPRESSOR_SOFT_START_DURATION_S = 180s)
    openamber.advance_time(seconds=190, step_s=10)
    openamber.step(ms=100)

    assert int(openamber.get_entity("compressor_control_select") or 0) > 0
    assert float(openamber.get_entity("current_compressor_frequency") or 0) > 0

    # Heat long enough to exceed minimum compressor on time (COMPRESSOR_MIN_ON_S = 600s)
    openamber.advance_time(seconds=450, step_s=20)
    openamber.step(ms=100)
    assert int(openamber.get_entity("compressor_control_select") or 0) > 0

    # Tc is 1 degree below stop delta: target (35) + stop_delta (5) - 1.0 = 39.0°C
    tc_below_stop_delta = target_temperature + stop_delta - 1.0
    openamber.set_sensor("current_water_temperature_tc_sensor", tc_below_stop_delta)
    openamber.set_sensor("heat_cool_temperature_tc", tc_below_stop_delta)
    openamber.set_sensor("outlet_temperature_tuo", tc_below_stop_delta)
    openamber.set_sensor("inlet_temperature_tui", tc_below_stop_delta - 2.0)
    openamber.step(ms=100)

    # Advance time so controller processes the temperature update
    openamber.advance_time(seconds=30, step_s=5)
    openamber.step(ms=100)

    # Confirm system continues to heat
    assert int(openamber.get_entity("compressor_control_select") or 0) > 0
    assert float(openamber.get_entity("current_compressor_frequency") or 0) > 0


def test_heat_demand_compressor_stops_above_stop_delta(clean_system):
    """
    Stop Delta Test 2:
    1. System is configured with:
       - Target temperature: 35.0°C (Extern setpoint)
       - Compressor start delta: 3.0°C
       - Compressor stop delta: 5.0°C
       - Initial supply Tc: 28.0°C (< target 35.0 - start_delta 3.0 = 32.0°C)
    2. Heat demand is activated (external_heat_demand_wired = True).
    3. Pump settles (120s) and compressor starts, passing softstart (180s) and min-on time (600s).
    4. Compressor is confirmed actively heating.
    5. Tc reaches 41.0°C (1 degree above target 35.0 + stop_delta 5.0 = 40.0°C).
    6. Verify the compressor stops heating (compressor_control_select == 0).
    """
    openamber = clean_system

    # Configure setpoint and deltas
    target_temperature = 35.0
    start_delta = 3.0
    stop_delta = 5.0

    openamber.set_select("thermostat_mode_select", "Extern")
    openamber.set_select("heat_mode_select", "Extern setpoint")
    openamber.set_number("manual_setpoint", target_temperature)
    openamber.set_climate("pid_heat_temperature_control", target_temperature=target_temperature)
    openamber.set_number("compressor_start_delta_heating", start_delta)
    openamber.set_number("compressor_stop_delta_heating", stop_delta)

    # Ensure minimum compressor off time has elapsed before starting demand
    openamber.advance_time(seconds=130, step_s=10)
    openamber.step(ms=50)

    # Initial water temperatures: Tc < target - start_delta (28.0 < 32.0)
    openamber.set_sensor("current_water_temperature_tc_sensor", 28.0)
    openamber.set_sensor("heat_cool_temperature_tc", 28.0)
    openamber.set_sensor("outlet_temperature_tuo", 28.0)
    openamber.set_sensor("inlet_temperature_tui", 26.0)
    openamber.step(ms=100)

    # Thermostat calls for heat
    openamber.set_binary_sensor("external_heat_demand_wired", True)
    openamber.step(ms=100)
    assert openamber.get_entity("heat_demand_active_sensor") is True

    # Advance time to trigger pump interval cycle in IDLE (pump_interval = 15 min = 900s)
    openamber.advance_time(seconds=900, step_s=30)
    openamber.step(ms=100)

    # Advance time for pump start and temperature settle (COMPRESSOR_MIN_TIME_PUMP_ON = 120s)
    openamber.advance_time(seconds=130, step_s=10)
    openamber.step(ms=100)

    # Advance time through compressor soft start (COMPRESSOR_SOFT_START_DURATION_S = 180s)
    openamber.advance_time(seconds=190, step_s=10)
    openamber.step(ms=100)

    assert int(openamber.get_entity("compressor_control_select") or 0) > 0
    assert float(openamber.get_entity("current_compressor_frequency") or 0) > 0

    # Heat long enough to exceed minimum compressor on time (COMPRESSOR_MIN_ON_S = 600s)
    openamber.advance_time(seconds=450, step_s=20)
    openamber.step(ms=100)
    assert int(openamber.get_entity("compressor_control_select") or 0) > 0

    # Tc is 1 degree above stop delta: target (35) + stop_delta (5) + 1.0 = 41.0°C
    tc_above_stop_delta = target_temperature + stop_delta + 1.0
    openamber.set_sensor("current_water_temperature_tc_sensor", tc_above_stop_delta)
    openamber.set_sensor("heat_cool_temperature_tc", tc_above_stop_delta)
    openamber.set_sensor("outlet_temperature_tuo", tc_above_stop_delta)
    openamber.set_sensor("inlet_temperature_tui", tc_above_stop_delta - 2.0)
    openamber.step(ms=100)

    # Advance time so controller processes the temperature overshoot and stops compressor
    openamber.advance_time(seconds=30, step_s=5)
    openamber.step(ms=100)

    # Confirm compressor has stopped
    assert int(openamber.get_entity("compressor_control_select") or 0) == 0
    assert float(openamber.get_entity("current_compressor_frequency") or 0) == 0.0


def test_heat_demand_pump_runs_full_interval_after_compressor_stop(clean_system):
    """
    Pump Interval Flow:
    1. System is configured with:
       - Target temperature: 35.0°C
       - Compressor start delta: 3.0°C
       - Compressor stop delta: 5.0°C
       - Pump interval: 15 minutes (900s)
       - Pump duration: 2 minutes (120s)
    2. Heating demand is running and compressor is actively heating.
    3. Temperature overshoots to stop threshold (41.0°C >= 35.0 + 5.0).
    4. Compressor shuts down and restarts pump run cycle.
    5. Verify pump P0 continues running immediately after compressor stop.
    6. Advance virtual time across the pump run duration (120s) and confirm the pump
       remains actively running while the compressor stays off.
    7. Once the pump duration completes, verify the pump shuts off (internal_pump_active is False).
    """
    openamber = clean_system

    target_temperature = 35.0
    start_delta = 3.0
    stop_delta = 5.0
    pump_interval_min = 15.0
    pump_duration_min = 2.0

    openamber.set_select("thermostat_mode_select", "Extern")
    openamber.set_select("heat_mode_select", "Extern setpoint")
    openamber.set_number("manual_setpoint", target_temperature)
    openamber.set_climate("pid_heat_temperature_control", target_temperature=target_temperature)
    openamber.set_number("compressor_start_delta_heating", start_delta)
    openamber.set_number("compressor_stop_delta_heating", stop_delta)
    openamber.set_number("pump_interval", pump_interval_min)
    openamber.set_number("pump_duration", pump_duration_min)

    # Ensure minimum compressor off-time has elapsed
    openamber.advance_time(seconds=130, step_s=10)
    openamber.step(ms=50)

    # Initial water temperatures: Tc < target - start_delta (28.0 < 32.0)
    openamber.set_sensor("current_water_temperature_tc_sensor", 28.0)
    openamber.set_sensor("heat_cool_temperature_tc", 28.0)
    openamber.set_sensor("outlet_temperature_tuo", 28.0)
    openamber.set_sensor("inlet_temperature_tui", 26.0)
    openamber.step(ms=100)

    # Thermostat calls for heat
    openamber.set_binary_sensor("external_heat_demand_wired", True)
    openamber.step(ms=100)
    assert openamber.get_entity("heat_demand_active_sensor") is True

    # Advance time to trigger pump interval cycle in IDLE (pump_interval = 15 min = 900s)
    openamber.advance_time(seconds=900, step_s=30)
    openamber.step(ms=100)

    # Advance time for pump start and temperature settle (120s)
    openamber.advance_time(seconds=130, step_s=10)
    openamber.step(ms=100)

    # Advance time through compressor soft start (180s)
    openamber.advance_time(seconds=190, step_s=10)
    openamber.step(ms=100)

    # Verify compressor and pump are running
    assert int(openamber.get_entity("compressor_control_select") or 0) > 0
    assert float(openamber.get_entity("current_compressor_frequency") or 0) > 0
    assert openamber.get_entity("pump_p0_relay_switch") is True
    assert openamber.get_entity("internal_pump_active") is True

    # Heat past compressor minimum on time (COMPRESSOR_MIN_ON_S = 600s)
    openamber.advance_time(seconds=450, step_s=20)
    openamber.step(ms=100)
    assert int(openamber.get_entity("compressor_control_select") or 0) > 0

    # Overshoot temperature above stop delta (41.0°C >= 35.0 + 5.0)
    tc_above_stop_delta = target_temperature + stop_delta + 1.0
    openamber.set_sensor("current_water_temperature_tc_sensor", tc_above_stop_delta)
    openamber.set_sensor("heat_cool_temperature_tc", tc_above_stop_delta)
    openamber.set_sensor("outlet_temperature_tuo", tc_above_stop_delta)
    openamber.set_sensor("inlet_temperature_tui", tc_above_stop_delta - 2.0)
    openamber.step(ms=100)

    # Advance time so controller processes stop condition
    openamber.advance_time(seconds=30, step_s=5)
    openamber.step(ms=100)

    # 1. Compressor must be stopped
    assert int(openamber.get_entity("compressor_control_select") or 0) == 0
    assert float(openamber.get_entity("current_compressor_frequency") or 0) == 0.0

    # 2. Pump must still be running immediately after compressor stop
    assert openamber.get_entity("internal_pump_active") is True

    # 3. Advance virtual time halfway through the pump duration cycle (60s)
    # Verify the pump continues to run and compressor stays stopped
    openamber.advance_time(seconds=60, step_s=10)
    openamber.step(ms=50)
    assert openamber.get_entity("internal_pump_active") is True
    assert int(openamber.get_entity("compressor_control_select") or 0) == 0

    # 4. Once pump duration (120s total from restart) completes, the pump stops
    openamber.advance_time(seconds=60, step_s=10)
    openamber.step(ms=50)
    assert openamber.get_entity("internal_pump_active") is False, (
        "Pump should stop once pump duration completes"
    )


def test_heat_demand_compressor_stops_at_exact_stop_delta_boundary(clean_system):
    """
    Boundary Condition Test:
    Verify that when Tc equals exactly target_temperature + stop_delta (inclusive >= condition),
    the compressor shuts down.
    """
    openamber = clean_system

    target_temperature = 35.0
    start_delta = 3.0
    stop_delta = 5.0

    assert openamber.set_select("thermostat_mode_select", "Extern")
    assert openamber.set_select("heat_mode_select", "Extern setpoint")
    assert openamber.set_number("manual_setpoint", target_temperature)
    assert openamber.set_climate("pid_heat_temperature_control", target_temperature=target_temperature)
    assert openamber.set_number("compressor_start_delta_heating", start_delta)
    assert openamber.set_number("compressor_stop_delta_heating", stop_delta)

    # Min compressor off time
    openamber.advance_time(seconds=130, step_s=10)
    openamber.step(ms=50)

    # Initial temp < target - start_delta
    assert openamber.set_sensor("current_water_temperature_tc_sensor", 28.0)
    assert openamber.set_sensor("heat_cool_temperature_tc", 28.0)
    assert openamber.set_sensor("outlet_temperature_tuo", 28.0)
    assert openamber.set_sensor("inlet_temperature_tui", 26.0)
    openamber.step(ms=50)

    assert openamber.set_binary_sensor("external_heat_demand_wired", True)
    openamber.step(ms=100)

    # Pump interval (900s) + pump settle (130s) + softstart (190s) + min-on time (450s)
    openamber.advance_time(seconds=900, step_s=30)
    openamber.advance_time(seconds=130, step_s=10)
    openamber.advance_time(seconds=190, step_s=10)
    openamber.step(ms=100)
    assert int(openamber.get_entity("compressor_control_select") or 0) > 0, "Compressor should be running"

    openamber.advance_time(seconds=450, step_s=20)
    openamber.step(ms=100)

    # Set Tc EXACTLY at target + stop_delta = 35.0 + 5.0 = 40.0°C
    exact_stop_temp = target_temperature + stop_delta
    assert openamber.set_sensor("current_water_temperature_tc_sensor", exact_stop_temp)
    assert openamber.set_sensor("heat_cool_temperature_tc", exact_stop_temp)
    assert openamber.set_sensor("outlet_temperature_tuo", exact_stop_temp)
    assert openamber.set_sensor("inlet_temperature_tui", exact_stop_temp - 2.0)
    openamber.step(ms=100)

    # Advance virtual time to process stop condition
    openamber.advance_time(seconds=30, step_s=5)
    openamber.step(ms=100)

    # Compressor must stop at exact boundary
    assert int(openamber.get_entity("compressor_control_select") or 0) == 0, (
        "Compressor must stop when Tc reaches exactly target + stop_delta (inclusive boundary)"
    )

    # Cleanup
    assert openamber.set_binary_sensor("external_heat_demand_wired", False)
    openamber.step(ms=100)








