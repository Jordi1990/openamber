"""Tests for the "Backup element verwarmen toegestaan" switch (backup_heater_heating_allowed_switch).

When off, the backup heater is never switched on during space heating: not by the degree/minutes, not by SG-Ready
max boost and not by the boost after a defrost. A heater that is already on for heating is switched off.
Emergency mode ("Noodbedrijf") is an explicit request for the backup heater and keeps using it. The default is on,
so existing installs keep their backup heater.

Also covers a degree/minute threshold of 0: it used to switch the heater on at every compressor cycle because
0 >= 0 holds while nothing has been accumulated; now the heater needs an actual shortage.
"""

import os

import yaml

from helpers import advance_until_state, set_outside_temperature
from test_backup_heater_outside_block import (
    TARGET,
    _degmin,
    _finish_heating,
    _set_water_temperature,
    _start_heating_with_deficit,
)
from test_entity_schema_sync import ESPHomeYamlLoader

SWITCHES_YAML = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                             "src", "openamber", "core", "switches.yaml")


def test_backup_heater_heating_allowed_defaults_on():
    with open(SWITCHES_YAML, encoding="utf-8") as f:
        switches = yaml.load(f, Loader=ESPHomeYamlLoader)["switch"]
    entry = next(s for s in switches if s.get("id") == "backup_heater_heating_allowed_switch")
    assert entry["restore_mode"] == "RESTORE_DEFAULT_ON"


def test_backup_heater_stays_off_when_not_allowed(clean_system):
    """A large shortage at the maximum compressor mode does not switch the heater on, nor count degree/minutes."""
    openamber = clean_system
    set_outside_temperature(openamber, 2.0)
    assert openamber.set_switch("backup_heater_heating_allowed_switch", False)

    _start_heating_with_deficit(openamber)
    openamber.advance_time(seconds=180, step_s=5)  # 15 °C x 3 min = 45 °C·min, far above the threshold of 10
    openamber.step(ms=50)
    assert openamber.get_entity("backup_heater_relay") is False, "Backup heater must stay off when not allowed"
    assert _degmin(openamber) == 0.0, "Degree/minutes must not be counted when not allowed"
    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"

    # Allowed again: counting starts from zero, no immediate start for the time it was not allowed.
    assert openamber.set_switch("backup_heater_heating_allowed_switch", True)
    openamber.advance_time(seconds=10, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity("backup_heater_relay") is False
    assert _degmin(openamber) < 10.0

    openamber.advance_time(seconds=60, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity("backup_heater_relay") is True, "Backup heater must work again once allowed"

    _finish_heating(openamber)


def test_backup_heater_switches_off_when_disallowed_while_running(clean_system):
    openamber = clean_system
    set_outside_temperature(openamber, 2.0)

    _start_heating_with_deficit(openamber)
    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Backup heater running", max_s=180, step=5)
    assert openamber.get_entity("backup_heater_relay") is True

    assert openamber.set_switch("backup_heater_heating_allowed_switch", False)
    openamber.advance_time(seconds=10, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity("backup_heater_relay") is False, "Backup heater must go off when it is disallowed"

    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Compressor running", max_s=300, step=5)
    openamber.advance_time(seconds=120, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity("backup_heater_relay") is False

    _finish_heating(openamber)


def test_sg_ready_max_boost_does_not_switch_heater_on_when_not_allowed(clean_system):
    openamber = clean_system
    set_outside_temperature(openamber, 2.0)
    assert openamber.set_switch("backup_heater_heating_allowed_switch", False)

    _start_heating_with_deficit(openamber)
    openamber.set_select("sg_ready_mode_select", "Max boost")
    openamber.advance_time(seconds=30, step_s=2)
    openamber.step(ms=50)
    assert openamber.get_entity("sg_ready_max_boost_mode_active_sensor") is True
    assert openamber.get_entity("backup_heater_relay") is False

    openamber.set_select("sg_ready_mode_select", "Normaal")
    openamber.advance_time(seconds=10, step_s=2)
    _finish_heating(openamber)


def test_emergency_mode_still_uses_backup_heater_when_not_allowed(clean_system):
    openamber = clean_system
    assert openamber.set_switch("backup_heater_heating_allowed_switch", False)
    assert openamber.set_switch("emergency_mode_enabled", True)
    try:
        openamber.set_select("thermostat_mode_select", "Extern")
        openamber.set_select("heat_mode_select", "Extern setpoint")
        openamber.set_number("manual_setpoint", TARGET)
        openamber.set_climate("pid_heat_temperature_control", target_temperature=TARGET)
        openamber.set_number("compressor_start_delta_heating", 2.0)
        openamber.set_number("compressor_stop_delta_heating", 2.0)
        _set_water_temperature(openamber, TARGET - 7.0)
        openamber.set_binary_sensor("external_heat_demand_wired", True)
        openamber.step(ms=50)

        assert advance_until_state(openamber, "state_machine_state_heat_cool", "Backup heater running", max_s=1800, step=5), (
            f"Emergency mode did not run the backup heater (state: {openamber.get_entity('state_machine_state_heat_cool')})"
        )
        openamber.advance_time(seconds=60, step_s=5)
        openamber.step(ms=50)
        assert openamber.get_entity("backup_heater_relay") is True, "Emergency mode must keep using the backup heater"
        assert int(openamber.get_entity("compressor_control_select") or 0) == 0
    finally:
        _finish_heating(openamber)
        openamber.set_switch("emergency_mode_enabled", False)
        openamber.advance_time(seconds=300, step_s=10)
        openamber.step(ms=50)


def test_degree_minute_threshold_zero_needs_a_shortage(clean_system):
    """Threshold 0 below the maximum compressor mode: the degree/minutes stay 0 (not at the maximum mode), so the
    heater must stay off. It used to switch on directly because 0 >= 0."""
    openamber = clean_system
    set_outside_temperature(openamber, 2.0)

    openamber.set_select("thermostat_mode_select", "Extern")
    openamber.set_select("heat_mode_select", "Extern setpoint")
    openamber.set_number("manual_setpoint", TARGET)
    openamber.set_climate("pid_heat_temperature_control", target_temperature=TARGET)
    openamber.set_number("compressor_start_delta_heating", 2.0)
    openamber.set_number("compressor_stop_delta_heating", 2.0)
    openamber.set_select("heat_compressor_mode", "Maximaal")
    openamber.set_number("backup_heater_degmin_threshold", 0.0)
    _set_water_temperature(openamber, TARGET - 5.0)
    openamber.set_binary_sensor("external_heat_demand_wired", True)
    openamber.step(ms=50)
    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Compressor running", max_s=1800, step=5)

    openamber.advance_time(seconds=30, step_s=5)
    openamber.step(ms=50)
    assert int(openamber.get_entity("compressor_control_select") or 0) < 10, "Compressor must be below its maximum mode"
    assert _degmin(openamber) == 0.0
    assert openamber.get_entity("backup_heater_relay") is False, "Threshold 0 must not switch the heater on without a shortage"

    _finish_heating(openamber)


def test_backup_heater_heating_allowed_ui(clean_system):
    """The switch under Instellingen > Bijverwarmen > Verwarmen."""
    openamber = clean_system
    assert openamber.click("bijverwarmen_stab_verwarmen")
    openamber.step(ms=50)
    assert openamber.get_widget("set_backup_heating_allowed_switch_ui").get("checked") is True

    assert openamber.click("set_backup_heating_allowed_switch_ui")
    openamber.step(ms=50)
    assert openamber.get_entity("backup_heater_heating_allowed_switch") is False
    assert openamber.get_widget("set_backup_heating_allowed_switch_ui").get("checked") is False

    assert openamber.click("set_backup_heating_allowed_switch_ui")
    openamber.step(ms=50)
    assert openamber.get_entity("backup_heater_heating_allowed_switch") is True
    assert openamber.get_widget("set_backup_heating_allowed_switch_ui").get("checked") is True

    # A change from elsewhere (web server, Home Assistant) updates the control too.
    assert openamber.set_switch("backup_heater_heating_allowed_switch", False)
    openamber.step(ms=50)
    assert openamber.get_widget("set_backup_heating_allowed_switch_ui").get("checked") is False
