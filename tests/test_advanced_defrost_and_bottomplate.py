"""Automated tests for Advanced Defrost settings and Bottomplate Heater configurations.

Covers settings:
1. Bottomplate Heater Settings:
   - bottomplate_heater_mode (select: Onbekend, Buitentemperatuur, Tijdens defrost)
   - bottomplate_heater_ambient_temperature_start (number: start temp)
   - bottomplate_heater_ambient_hysteresis_stop (number: stop hysteresis)
2. Defrost Threshold Parameters:
   - enter_defrost_temperature (stage 1 start temp)
   - enter_defrost_temperature_2 (stage 2 start temp)
   - enter_defrost_temperature_3 (stage 3 start temp)
   - enter_defrost_temperature_4 (stage 4 start temp)
   - exit_defrost_temperature (exit temp)
   - max_defrost_time (maximum duration in minutes)
"""

import pytest


def test_bottomplate_heater_configuration(clean_system):
    """
    Verify Bottomplate Heater settings:
    - Mode switching between options:
      'Onbekend', 'Buitentemperatuur', 'Tijdens defrost'.
    - Ambient start temperature setting.
    - Ambient stop hysteresis setting.
    """
    openamber = clean_system

    # Test mode options
    modes = ["Onbekend", "Buitentemperatuur", "Tijdens defrost"]
    for mode in modes:
        assert openamber.set_select("bottomplate_heater_mode", mode) is True, f"Setting bottomplate_heater_mode to {mode} must succeed"
        openamber.step(ms=50)
        assert openamber.get_entity("bottomplate_heater_mode") == mode, f"Expected bottomplate_heater_mode {mode}"

    # Test start temperature configuration
    assert openamber.set_number("bottomplate_heater_ambient_temperature_start", 2.0), "Setting start temp must succeed"
    openamber.step(ms=50)
    assert float(openamber.get_entity("bottomplate_heater_ambient_temperature_start") or 0) == 2.0, "Expected start temp 2.0"

    assert openamber.set_number("bottomplate_heater_ambient_temperature_start", 4.0), "Setting start temp must succeed"
    openamber.step(ms=50)
    assert float(openamber.get_entity("bottomplate_heater_ambient_temperature_start") or 0) == 4.0, "Expected start temp 4.0"

    # Test stop hysteresis configuration
    assert openamber.set_number("bottomplate_heater_ambient_hysteresis_stop", 3.0), "Setting hysteresis must succeed"
    openamber.step(ms=50)
    assert float(openamber.get_entity("bottomplate_heater_ambient_hysteresis_stop") or 0) == 3.0, "Expected hysteresis 3.0"

    assert openamber.set_number("bottomplate_heater_ambient_hysteresis_stop", 1.5), "Setting hysteresis must succeed"
    openamber.step(ms=50)
    assert float(openamber.get_entity("bottomplate_heater_ambient_hysteresis_stop") or 0) == 1.5, "Expected hysteresis 1.5"


def test_advanced_defrost_threshold_parameters(clean_system):
    """
    Verify Advanced Defrost temperature thresholds and timers:
    - enter_defrost_temperature (-15 to +5°C)
    - enter_defrost_temperature_2 (-3 to +3°C)
    - enter_defrost_temperature_3 (-10 to -3°C)
    - enter_defrost_temperature_4 (-10 to -3°C)
    - exit_defrost_temperature (0 to 25°C)
    - max_defrost_time (1 to 30 min)
    """
    openamber = clean_system

    thresholds = [
        ("enter_defrost_temperature", -6.0),
        ("enter_defrost_temperature_2", 1.0),
        ("enter_defrost_temperature_3", -4.5),
        ("enter_defrost_temperature_4", -8.0),
        ("exit_defrost_temperature", 14.0),
        ("max_defrost_time", 15.0),
    ]

    for entity_id, val in thresholds:
        assert openamber.set_number(entity_id, val) is True
        openamber.step(ms=50)
        curr = float(openamber.get_entity(entity_id) or 0)
        assert curr == pytest.approx(val, 0.05), f"{entity_id} failed to set {val}, got {curr}"
