"""Tests for frost protection detection and instant pump activation."""

import pytest


def test_frost_protection_stage1_instantly_starts_pump_in_idle(clean_system):
    """
    Frost Protection Stage 1 Flow:
    1. System is in IDLE with no heat demand and normal outside temperature (7.0°C).
    2. Pump is stopped (internal_pump_active is False) with full 15-min pump interval pending.
    3. Outside temperature drops below frost protection stage 1 threshold (Ta = 2.0°C < 5.0°C).
    4. Frost detection stage 1 activates (frost_protection_stage1_active = True).
    5. Verify pump P0 starts INSTANTLY without waiting for the 15-minute pump interval to elapse.
    """
    openamber = clean_system

    # Set normal baseline conditions (no demand, mild temperature)
    openamber.set_sensor("temperature_outside_ta", 7.0)
    openamber.set_number("frost_protection_stage_1_temp_ta", 5.0)
    openamber.set_number("pump_interval", 15.0)
    openamber.set_binary_sensor("external_heat_demand_wired", False)
    openamber.step(ms=100)

    # Verify initial state: system in idle, frost sensor inactive, pump is OFF
    assert openamber.get_entity("frost_protection_stage1_active") is False, "Frost protection stage 1 should initially be inactive"
    assert openamber.get_entity("internal_pump_active") is False, "Pump should initially be OFF in idle"

    # Outside temperature drops below stage 1 threshold
    openamber.set_sensor("temperature_outside_ta", 2.0)
    openamber.step(ms=100)

    # Allow 300s delayed_on filter to pass (well below the 15-min pump interval)
    openamber.advance_time(seconds=305, step_s=10)
    openamber.step(ms=100)

    # Frost protection stage 1 must be active
    assert openamber.get_entity("frost_protection_stage1_active") is True, "Frost protection stage 1 should activate when Ta < threshold"

    # Pump must start INSTANTLY (without waiting for the 15-minute pump interval to elapse)
    assert openamber.get_entity("internal_pump_active") is True, (
        "Pump should start instantly when frost protection stage 1 is detected"
    )

    # Cleanup frost protection conditions
    openamber.set_sensor("temperature_outside_ta", 7.0)
    openamber.advance_time(seconds=310, step_s=15)
    openamber.step(ms=100)


def test_frost_protection_stage2_instantly_starts_pump_in_idle(clean_system):
    """
    Frost Protection Stage 2 Flow:
    1. System is in IDLE with no heat demand, mild outside temp (7.0°C) and water temp (20.0°C).
    2. Pump is stopped (internal_pump_active is False) with full 15-min pump interval pending.
    3. Outside temp drops below Ta threshold (Ta = 2.0°C < 4.0°C) AND inlet water temp drops
       below Tui threshold (Tui = 5.0°C < 7.0°C).
    4. Frost detection stage 2 activates (frost_protection_stage2_active = True).
    5. Verify pump P0 starts INSTANTLY without waiting for the 15-minute pump interval to elapse.
    """
    openamber = clean_system

    # Set normal baseline conditions
    openamber.set_sensor("temperature_outside_ta", 7.0)
    openamber.set_sensor("inlet_temperature_tui", 20.0)
    openamber.set_number("frost_protection_stage_2_temp_ta", 4.0)
    openamber.set_number("frost_protection_stage_2_temp_tui", 7.0)
    openamber.set_number("pump_interval", 15.0)
    openamber.set_binary_sensor("external_heat_demand_wired", False)
    openamber.step(ms=100)

    # Verify initial state
    assert openamber.get_entity("frost_protection_stage2_active") is False, "Frost protection stage 2 should initially be inactive"
    assert openamber.get_entity("internal_pump_active") is False, "Pump should initially be OFF in idle"

    # Outside temp and inlet temp drop below stage 2 thresholds
    openamber.set_sensor("temperature_outside_ta", 2.0)
    openamber.set_sensor("inlet_temperature_tui", 5.0)
    openamber.step(ms=100)

    # Allow 300s delayed_on filter to pass (well below the 15-min pump interval)
    openamber.advance_time(seconds=305, step_s=10)
    openamber.step(ms=100)

    # Frost protection stage 2 must be active
    assert openamber.get_entity("frost_protection_stage2_active") is True, "Frost protection stage 2 should activate when Ta and Tui < thresholds"

    # Pump must start INSTANTLY (without waiting for the 15-minute pump interval to elapse)
    assert openamber.get_entity("internal_pump_active") is True, (
        "Pump should start instantly when frost protection stage 2 is detected"
    )

    # Cleanup frost protection conditions
    openamber.set_sensor("temperature_outside_ta", 7.0)
    openamber.set_sensor("inlet_temperature_tui", 20.0)
    openamber.advance_time(seconds=310, step_s=15)
    openamber.step(ms=100)


def test_frost_protection_stage2_triggers_compressor_demand_without_heat_demand(clean_system):
    """
    Verify Frost Protection Stage 2 initiates active heating demand:
    1. Thermostat heat demand is completely inactive (external_heat_demand_wired = False, heat_demand_switch = False).
    2. Outside temp (Ta = 2.0°C) and water temp (Tui = 5.0°C) trigger frost protection stage 2.
    3. Controller recognizes stage 2 as active compressor heating demand (HasCompressorDemand = True).
    4. After pump settle and softstart, compressor starts heating to protect system from freezing.
    """
    openamber = clean_system

    openamber.set_binary_sensor("external_heat_demand_wired", False)
    openamber.set_switch("heat_demand_switch", False)
    openamber.set_number("frost_protection_stage_2_temp_ta", 4.0)
    openamber.set_number("frost_protection_stage_2_temp_tui", 7.0)
    openamber.set_sensor("temperature_outside_ta", 2.0)
    openamber.set_sensor("inlet_temperature_tui", 5.0)
    openamber.set_sensor("current_water_temperature_tc_sensor", 12.0)
    openamber.set_sensor("heat_cool_temperature_tc", 12.0)
    openamber.set_sensor("outlet_temperature_tuo", 12.0)
    openamber.step(ms=100)

    # Allow 300s delayed_on filter for frost detection to engage
    openamber.advance_time(seconds=305, step_s=10)
    openamber.step(ms=100)

    assert openamber.get_entity("frost_protection_stage2_active") is True, "Stage 2 should be active"
    assert openamber.get_entity("internal_pump_active") is True, "Pump should be running"

    # Advance virtual time through pump settle (130s) + softstart (190s)
    openamber.advance_time(seconds=130, step_s=10)
    openamber.advance_time(seconds=190, step_s=10)
    openamber.step(ms=100)

    # Compressor must start heating despite no external heat demand
    comp_mode = int(openamber.get_entity("compressor_control_select") or 0)
    assert comp_mode > 0, f"Compressor should start heating in frost protection stage 2, got mode {comp_mode}"
    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running", (
        f"State should be 'Compressor running', got {openamber.get_entity('state_machine_state_heat_cool')}"
    )

    # Cleanup
    openamber.set_sensor("temperature_outside_ta", 7.0)
    openamber.set_sensor("inlet_temperature_tui", 20.0)
    openamber.advance_time(seconds=310, step_s=15)
    openamber.step(ms=100)
