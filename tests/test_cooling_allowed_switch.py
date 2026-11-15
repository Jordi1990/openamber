"""Tests for the "Koelen toegestaan" switch (cooling_allowed_switch).

When the switch is off, cooling demand from every source (wired input, "Externe koudevraag",
internal thermostat) is ignored; heating is not affected. It defaults to on, so existing installs
keep cooling as before.
"""

import os

import yaml

from helpers import FlowMonitor
from test_entity_schema_sync import ESPHomeYamlLoader

SWITCHES_YAML = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                             "src", "openamber", "core", "switches.yaml")


def _set_cool_water_temperatures(openamber, tc=25.0):
    """Supply water above the 18 °C cooling setpoint + start delta, so cooling wants the compressor."""
    openamber.set_sensor("current_water_temperature_tc_sensor", tc)
    openamber.set_sensor("heat_cool_temperature_tc", tc)
    openamber.set_sensor("outlet_temperature_tuo", tc)
    openamber.set_sensor("inlet_temperature_tui", tc)


def test_cooling_allowed_defaults_on():
    """Existing installs keep cooling: the switch restores to on when nothing was saved yet."""
    with open(SWITCHES_YAML, encoding="utf-8") as f:
        switches = yaml.load(f, Loader=ESPHomeYamlLoader)["switch"]
    entry = next(s for s in switches if s.get("id") == "cooling_allowed_switch")
    assert entry["restore_mode"] == "RESTORE_DEFAULT_ON"
    assert entry["name"] == "Koelen toegestaan"


def test_external_cooling_demand_ignored_when_not_allowed(clean_system):
    """The wired cool input and the "Externe koudevraag" switch do not start cooling while cooling is not allowed."""
    openamber = clean_system
    assert openamber.get_entity("cooling_allowed_switch") is True

    assert openamber.set_switch("cooling_allowed_switch", False)
    assert openamber.set_binary_sensor("external_cool_demand_wired", True)
    openamber.step(ms=50)
    assert openamber.get_entity("cool_demand_active_sensor") is False, "Wired cool input must be ignored"
    assert openamber.is_hidden("nav_status_cool_icon")

    assert openamber.set_binary_sensor("external_cool_demand_wired", False)
    assert openamber.set_switch("cool_demand_switch", True)
    openamber.step(ms=50)
    assert openamber.get_entity("cool_demand_active_sensor") is False, "Externe koudevraag must be ignored"

    # Allowing cooling again lets the pending request through right away.
    assert openamber.set_switch("cooling_allowed_switch", True)
    openamber.step(ms=50)
    assert openamber.get_entity("cool_demand_active_sensor") is True
    assert openamber.is_visible("nav_status_cool_icon")

    assert openamber.set_switch("cool_demand_switch", False)
    openamber.step(ms=50)


def test_internal_thermostat_cooling_ignored_when_not_allowed(clean_system):
    """With the internal thermostat cooling, no cooling demand while cooling is not allowed."""
    openamber = clean_system
    assert openamber.set_switch("cooling_allowed_switch", False)
    assert openamber.set_select("thermostat_mode_select", "Intern")
    openamber.set_climate("climate_controller", mode="COOL", target_temperature=24.0)
    openamber.set_sensor("room_temperature", 23.0)
    openamber.step(ms=50)
    # Minimum cooling off time of the internal thermostat (15 min).
    openamber.advance_time(seconds=920, step_s=30)

    openamber.set_sensor("room_temperature", 25.0)  # above setpoint + deadband (24.5 °C)
    openamber.advance_time(seconds=10, step_s=2)
    openamber.step(ms=50)
    assert openamber.get_entity("cool_demand_active_sensor") is False

    assert openamber.set_switch("cooling_allowed_switch", True)
    openamber.step(ms=50)
    assert openamber.get_entity("cool_demand_active_sensor") is True

    # Room cools down again and the thermostat ends the request after its minimum run time (1 h).
    openamber.set_sensor("room_temperature", 20.5)
    openamber.advance_time(seconds=3660, step_s=60)
    openamber.step(ms=50)
    assert openamber.get_entity("cool_demand_active_sensor") is False


def test_running_cooling_stops_normally_when_disallowed(clean_system):
    """Switching cooling off during a cooling run ends the demand; the compressor stops through the normal
    stop path, so it keeps its minimum on time (checked by FlowMonitor)."""
    openamber = clean_system
    monitor = FlowMonitor(openamber)
    monitor.check()

    _set_cool_water_temperatures(openamber)
    assert openamber.set_switch("cool_demand_switch", True)
    openamber.step(ms=50)
    assert monitor.advance_until("state_machine_state_heat_cool", "Compressor running", max_s=1800)
    assert openamber.get_entity("working_mode_switch") == "Koelen"

    assert openamber.set_switch("cooling_allowed_switch", False)
    openamber.step(ms=50)
    assert openamber.get_entity("cool_demand_active_sensor") is False

    stopped = False
    for _ in range(180):  # up to 900 s: the rest of the minimum on time plus the stop
        monitor.advance(5)
        if int(openamber.get_entity("compressor_control_select") or 0) == 0:
            stopped = True
            break
    assert stopped, "Compressor must stop after cooling was switched off"
    assert openamber.get_entity("state_machine_state_heat_cool") != "Compressor running"

    # No restart while cooling stays disallowed, although the request is still there.
    monitor.advance(600)
    assert int(openamber.get_entity("compressor_control_select") or 0) == 0

    assert openamber.set_switch("cool_demand_switch", False)
    openamber.step(ms=50)


def test_heating_not_affected_when_cooling_not_allowed(clean_system):
    """Heat demand still works with cooling disallowed."""
    openamber = clean_system
    assert openamber.set_switch("cooling_allowed_switch", False)
    assert openamber.set_binary_sensor("external_heat_demand_wired", True)
    openamber.step(ms=50)
    assert openamber.get_entity("heat_demand_active_sensor") is True

    assert openamber.set_binary_sensor("external_heat_demand_wired", False)
    openamber.step(ms=50)


def test_cooling_allowed_ui_switch(clean_system):
    """The toggle under Instellingen > Koelen > Algemeen follows and changes the setting."""
    openamber = clean_system
    assert openamber.click("cooling_stab_algemeen")
    openamber.step(ms=50)
    assert openamber.get_widget("set_cooling_allowed_switch_ui").get("checked") is True

    assert openamber.click("set_cooling_allowed_switch_ui")
    openamber.step(ms=50)
    assert openamber.get_entity("cooling_allowed_switch") is False
    assert openamber.get_widget("set_cooling_allowed_switch_ui").get("checked") is False

    assert openamber.click("set_cooling_allowed_switch_ui")
    openamber.step(ms=50)
    assert openamber.get_entity("cooling_allowed_switch") is True
    assert openamber.get_widget("set_cooling_allowed_switch_ui").get("checked") is True

    # A change from elsewhere (web server, Home Assistant) updates the toggle too.
    assert openamber.set_switch("cooling_allowed_switch", False)
    openamber.step(ms=50)
    assert openamber.get_widget("set_cooling_allowed_switch_ui").get("checked") is False
