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
    assert openamber.get_entity("frost_protection_stage1_active") is False
    assert openamber.get_entity("internal_pump_active") is False

    # Outside temperature drops below stage 1 threshold
    openamber.set_sensor("temperature_outside_ta", 2.0)
    openamber.step(ms=100)

    # Frost protection stage 1 must be active
    assert openamber.get_entity("frost_protection_stage1_active") is True

    # Advance virtual time by 10s (a few update cycles, well below 15-min pump interval)
    openamber.advance_time(seconds=10, step_s=1)
    openamber.step(ms=100)

    # Pump must start INSTANTLY (without waiting for the 15-minute pump interval to elapse)
    assert openamber.get_entity("internal_pump_active") is True, (
        "Pump should start instantly when frost protection stage 1 is detected"
    )


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
    assert openamber.get_entity("frost_protection_stage2_active") is False
    assert openamber.get_entity("internal_pump_active") is False

    # Outside temp and inlet temp drop below stage 2 thresholds
    openamber.set_sensor("temperature_outside_ta", 2.0)
    openamber.set_sensor("inlet_temperature_tui", 5.0)
    openamber.step(ms=100)

    # Frost protection stage 2 must be active
    assert openamber.get_entity("frost_protection_stage2_active") is True

    # Advance virtual time by 10s (a few update cycles, well below 15-min pump interval)
    openamber.advance_time(seconds=10, step_s=1)
    openamber.step(ms=100)

    # Pump must start INSTANTLY (without waiting for the 15-minute pump interval to elapse)
    assert openamber.get_entity("internal_pump_active") is True, (
        "Pump should start instantly when frost protection stage 2 is detected"
    )
