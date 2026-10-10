"""Automated tests for Internal Thermostat logic and configuration settings.

Covers:
1. Internal thermostat mode selection (thermostat_mode_select = 'Intern').
2. Internal heating demand triggering based on room_temperature falling below setpoint - deadband.
3. Internal heating shutoff upon exceeding setpoint + overrun.
4. Internal cooling demand triggering based on room_temperature exceeding setpoint + deadband.
5. Internal cooling shutoff upon falling below setpoint - overrun.
6. Verification that external wired contacts (external_heat_demand_wired) are ignored while in Intern mode.
"""

import pytest


def test_internal_thermostat_heat_demand_activation_and_overrun_shutoff(clean_system):
    """
    Verify internal thermostat heating flow:
    1. Set thermostat mode to 'Intern'.
    2. Set climate mode to HEAT.
    3. Allow min_idle_time (15 min = 900s) to pass.
    4. Room temp drops to 19.5°C (< 20.5°C - 0.3°C deadband) -> heat demand starts.
    5. Room temp warms to 20.4°C -> heat demand continues.
    6. Room temp exceeds 21.0°C (> 20.5°C + 0.2°C overrun) and min_run_time passes -> heat demand stops.
    """
    openamber = clean_system

    openamber.set_select("thermostat_mode_select", "Intern")
    openamber.set_climate("climate_controller", mode="HEAT")
    openamber.set_sensor("room_temperature", 20.5)
    openamber.step(ms=100)

    # Allow minimum idle time (min_idle_time = 15 min = 900s) to pass
    openamber.advance_time(seconds=920, step_s=30)
    openamber.step(ms=100)

    # Satisfied at 20.5°C
    assert openamber.get_entity("heat_demand_active_sensor") is False

    # Room temp drops below heat setpoint (20.5) - heat_deadband (0.3) = 20.2°C
    openamber.set_sensor("room_temperature", 19.5)
    openamber.advance_time(seconds=10, step_s=2)
    openamber.step(ms=100)

    # Heat demand must now activate
    assert openamber.get_entity("heat_demand_active_sensor") is True

    # Room warms to 20.4°C (still below setpoint + overrun = 20.7°C) -> heating continues
    openamber.set_sensor("room_temperature", 20.4)
    openamber.advance_time(seconds=10, step_s=2)
    openamber.step(ms=100)
    assert openamber.get_entity("heat_demand_active_sensor") is True

    # Room warms above setpoint (20.5) + overrun (0.2) = 20.7°C
    openamber.set_sensor("room_temperature", 21.2)
    # Advance time through min_heating_run_time (1h = 3600s)
    openamber.advance_time(seconds=3660, step_s=60)
    openamber.step(ms=100)

    # Heat demand must shut off
    assert openamber.get_entity("heat_demand_active_sensor") is False


def test_internal_thermostat_cool_demand_activation_and_overrun_shutoff(clean_system):
    """
    Verify internal thermostat cooling flow:
    1. Set thermostat mode to 'Intern'.
    2. Set climate mode to COOL.
    3. Allow min_idle_time (15 min) to pass.
    4. Room temp rises above cool setpoint (24.0) + deadband (0.5) = 24.5°C -> cooling demand starts.
    5. Room temp drops below cool setpoint (24.0) - overrun (3.0) = 21.0°C and min run time passes -> cooling stops.
    """
    openamber = clean_system

    openamber.set_select("thermostat_mode_select", "Intern")
    openamber.set_climate("climate_controller", mode="COOL")
    openamber.set_sensor("room_temperature", 23.0)
    openamber.step(ms=100)

    # Allow minimum cooling idle/off time (15 min = 900s) to pass
    openamber.advance_time(seconds=920, step_s=30)
    openamber.step(ms=100)

    assert openamber.get_entity("cool_demand_active_sensor") is False

    # Room warms above cooling threshold (24.0 + 0.5 = 24.5°C)
    openamber.set_sensor("room_temperature", 25.0)
    openamber.advance_time(seconds=10, step_s=2)
    openamber.step(ms=100)

    # Cooling demand must activate
    assert openamber.get_entity("cool_demand_active_sensor") is True

    # Room cools below cooling shutoff threshold (24.0 - 3.0 = 21.0°C)
    openamber.set_sensor("room_temperature", 20.5)
    # Advance virtual time through min_cooling_run_time (1h = 3600s)
    openamber.advance_time(seconds=3660, step_s=60)
    openamber.step(ms=100)

    # Cooling demand shuts off
    assert openamber.get_entity("cool_demand_active_sensor") is False


def test_internal_thermostat_ignores_wired_contact(clean_system):
    """
    Verify that when thermostat mode is 'Intern', the external contact
    (external_heat_demand_wired) does NOT trigger heat demand if room temperature is satisfied.
    """
    openamber = clean_system

    openamber.set_select("thermostat_mode_select", "Intern")
    openamber.set_climate("climate_controller", mode="HEAT")
    openamber.set_sensor("room_temperature", 22.0)  # Room is satisfied
    openamber.step(ms=100)

    assert openamber.get_entity("heat_demand_active_sensor") is False

    # External contact closes
    openamber.set_binary_sensor("external_heat_demand_wired", True)
    openamber.step(ms=100)

    # In Intern mode, external contact must be ignored
    assert openamber.get_entity("heat_demand_active_sensor") is False
