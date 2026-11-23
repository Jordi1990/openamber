"""Tests for "Ontdooien via tapwatervat" (defrost_from_dhw_tank_switch).

With the switch on, a defrost during heating takes its heat from the DHW tank: at the start of the actual defrost
the 3-way valve goes to DHW (compressor and pump P0 keep running, DHW pump relay on) and it goes back when the
defrost ends. Only with "Tapwater aanwezig" on and the tank above 30 °C and at most the configured maximum
(defrost_from_dhw_tank_max_temperature, default 60 °C). Defaults to off.
"""

import os

import pytest
import yaml

from test_entity_schema_sync import ESPHomeYamlLoader

CORE_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src", "openamber", "core")
SWITCHES_YAML = os.path.join(CORE_DIR, "switches.yaml")
NUMBERS_YAML = os.path.join(CORE_DIR, "numbers.yaml")


@pytest.fixture
def tank_defrost_system(clean_system):
    """clean_system does not know these settings: put them back for the tests that follow."""
    clean_system.set_number("defrost_from_dhw_tank_max_temperature", 60.0)
    yield clean_system
    clean_system.set_number("defrost_from_dhw_tank_max_temperature", 60.0)
    clean_system.set_sensor("dhw_temperature_tw_sensor", 50.0)
    clean_system.set_binary_sensor("defrost_active_sensor", False)
    clean_system.set_switch("defrost_from_dhw_tank_switch", False)
    clean_system.set_switch("heat_demand_switch", False)


def _start_heating(openamber, tank_temperature):
    """Space heating up to 'Compressor running', with the tank at tank_temperature and no DHW demand."""
    openamber.set_sensor("dhw_temperature_tw_sensor", tank_temperature)
    openamber.set_select("thermostat_mode_select", "Extern")
    openamber.set_select("heat_mode_select", "Extern setpoint")
    openamber.set_number("manual_setpoint", 35.0)
    openamber.set_climate("pid_heat_temperature_control", target_temperature=35.0)
    openamber.set_select("heat_compressor_mode", "Maximaal")
    openamber.set_switch("heat_demand_switch", True)
    openamber.set_sensor("current_water_temperature_tc_sensor", 28.0)
    openamber.set_sensor("heat_cool_temperature_tc", 28.0)
    openamber.set_sensor("outlet_temperature_tuo", 28.0)
    openamber.set_sensor("inlet_temperature_tui", 26.0)
    openamber.set_sensor("temperature_outside_ta", 5.0)
    openamber.step(ms=50)

    # IDLE pump interval (900s) + pump settle (140s) + softstart (190s)
    openamber.advance_time(seconds=900, step_s=30)
    openamber.advance_time(seconds=140, step_s=10)
    openamber.advance_time(seconds=190, step_s=10)
    openamber.step(ms=50)

    assert openamber.get_entity("dhw_demand_active_sensor") is False, "The tank must not ask for a DHW run"
    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"
    assert openamber.get_entity("three_way_valve_dhw_switch") is False


def _start_defrost(openamber):
    openamber.set_binary_sensor("defrost_active_sensor", True)
    openamber.advance_time(seconds=10, step_s=2)
    openamber.step(ms=50)
    assert openamber.get_entity("state_machine_state_heat_cool") == "Defrosting"


def test_defrost_from_dhw_tank_defaults_off():
    """Existing installs keep defrosting on the heating circuit."""
    with open(SWITCHES_YAML, encoding="utf-8") as f:
        switches = yaml.load(f, Loader=ESPHomeYamlLoader)["switch"]
    entry = next(s for s in switches if s.get("id") == "defrost_from_dhw_tank_switch")
    assert entry["restore_mode"] == "RESTORE_DEFAULT_OFF"
    assert entry["name"] == "Ontdooien via tapwatervat"

    with open(NUMBERS_YAML, encoding="utf-8") as f:
        numbers = yaml.load(f, Loader=ESPHomeYamlLoader)["number"]
    entry = next(n for n in numbers if n.get("id") == "defrost_from_dhw_tank_max_temperature")
    assert entry["initial_value"] == 60


def test_defrost_stays_on_heating_circuit_when_switched_off(tank_defrost_system):
    openamber = tank_defrost_system
    openamber.set_switch("defrost_from_dhw_tank_switch", False)
    _start_heating(openamber, tank_temperature=50.0)

    _start_defrost(openamber)

    assert openamber.get_entity("three_way_valve_dhw_switch") is False
    assert openamber.get_entity("three_way_valve_heat_cool_switch") is True
    assert openamber.get_entity("dhw_pump_relay_switch") is False


def test_defrost_takes_heat_from_dhw_tank(tank_defrost_system):
    """Valve to DHW at the start of the defrost, back to heating when it ends; the compressor and P0 keep running."""
    openamber = tank_defrost_system
    openamber.set_switch("defrost_from_dhw_tank_switch", True)
    _start_heating(openamber, tank_temperature=50.0)

    _start_defrost(openamber)

    assert openamber.get_entity("three_way_valve_dhw_switch") is True, "3-way valve should go to the tank"
    assert openamber.get_entity("three_way_valve_heat_cool_switch") is False
    assert openamber.get_entity("dhw_pump_relay_switch") is True, "DHW pump relay on, as in a DHW run"
    assert openamber.get_entity("state_machine_state_main") == "Heat/Cool", "Still a heating cycle, not a DHW run"
    assert openamber.get_entity("pump_p0_relay_switch") is True
    assert int(openamber.get_entity("compressor_control_select") or 0) > 0, "Compressor keeps running"

    # Hot tank water in the unit: Tui far above Tuo must not trip the Tuo/Tui safety stop of the heating circuit.
    openamber.set_sensor("inlet_temperature_tui", 48.0)
    openamber.set_sensor("outlet_temperature_tuo", 36.0)
    openamber.advance_time(seconds=120, step_s=10)
    openamber.step(ms=50)
    assert openamber.get_entity("state_machine_state_heat_cool") == "Defrosting"
    assert openamber.get_entity("three_way_valve_dhw_switch") is True

    openamber.set_binary_sensor("defrost_active_sensor", False)
    openamber.advance_time(seconds=10, step_s=2)
    openamber.step(ms=50)

    assert openamber.get_entity("three_way_valve_dhw_switch") is False, "3-way valve should go back to heating"
    assert openamber.get_entity("three_way_valve_heat_cool_switch") is True
    assert openamber.get_entity("dhw_pump_relay_switch") is False
    assert openamber.get_entity("state_machine_state_main") == "Heat/Cool"
    assert openamber.get_entity("state_machine_state_heat_cool") == "Wait for state switch"
    assert int(openamber.get_entity("compressor_control_select") or 0) > 0

    # Heating circuit water is back in the unit; the cycle continues after the settle time.
    openamber.set_sensor("inlet_temperature_tui", 26.0)
    openamber.set_sensor("outlet_temperature_tuo", 28.0)
    openamber.advance_time(seconds=310, step_s=10)
    openamber.step(ms=50)
    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"
    assert openamber.get_entity("three_way_valve_dhw_switch") is False


@pytest.mark.parametrize("tank_temperature, maximum, used", [
    (30.0, 60.0, False),  # at or below the fixed minimum
    (61.0, 60.0, False),  # above the default maximum
    (56.0, 60.0, True),   # a tank just above its 55 °C setpoint is used with the default
    (56.0, 55.0, False),  # ... and not with the maximum lowered
    (56.0, 56.0, True),   # the maximum itself is still used
])
def test_tank_temperature_limits(tank_defrost_system, tank_temperature, maximum, used):
    """The tank is used above 30 °C and up to the configured maximum."""
    openamber = tank_defrost_system
    openamber.set_switch("defrost_from_dhw_tank_switch", True)
    openamber.set_number("defrost_from_dhw_tank_max_temperature", maximum)
    _start_heating(openamber, tank_temperature=50.0)

    # The tank temperature counts at the start of the defrost. A cold tank also asks for a DHW run, which waits
    # for the defrost to finish.
    openamber.set_sensor("dhw_temperature_tw_sensor", tank_temperature)
    _start_defrost(openamber)

    assert openamber.get_entity("three_way_valve_dhw_switch") is used
    assert openamber.get_entity("dhw_pump_relay_switch") is used


def test_defrost_stays_on_heating_circuit_without_dhw(tank_defrost_system):
    """"Tapwater aanwezig" off: there is no tank to defrost from."""
    openamber = tank_defrost_system
    openamber.set_switch("defrost_from_dhw_tank_switch", True)
    openamber.set_switch("dhw_enabled_switch", False)
    _start_heating(openamber, tank_temperature=50.0)

    _start_defrost(openamber)

    assert openamber.get_entity("three_way_valve_dhw_switch") is False
    assert openamber.get_entity("dhw_pump_relay_switch") is False


def test_defrost_from_dhw_tank_ui_switch(tank_defrost_system):
    """The toggle under Instellingen > Tapwater > Algemeen follows and changes the setting."""
    openamber = tank_defrost_system
    openamber.set_switch("defrost_from_dhw_tank_switch", False)
    openamber.step(ms=50)
    assert openamber.get_widget("set_defrost_from_dhw_tank_switch_ui").get("checked") is False

    assert openamber.set_switch("defrost_from_dhw_tank_switch", True)
    openamber.step(ms=50)
    assert openamber.get_widget("set_defrost_from_dhw_tank_switch_ui").get("checked") is True

    assert openamber.set_switch("defrost_from_dhw_tank_switch", False)
    openamber.step(ms=50)
    assert openamber.get_widget("set_defrost_from_dhw_tank_switch_ui").get("checked") is False


def test_defrost_from_dhw_tank_max_temperature_ui(tank_defrost_system):
    """The maximum under Instellingen > Tapwater > Algemeen: -/+ without wrap-around."""
    openamber = tank_defrost_system
    openamber.step(ms=50)
    assert openamber.get_label("set_defrost_from_dhw_tank_max_val") == "60°C"

    assert openamber.click("set_defrost_from_dhw_tank_max_dec")
    openamber.step(ms=50)
    assert float(openamber.get_entity("defrost_from_dhw_tank_max_temperature")) == 59.0
    assert openamber.get_label("set_defrost_from_dhw_tank_max_val") == "59°C"

    assert openamber.click("set_defrost_from_dhw_tank_max_inc")
    assert openamber.click("set_defrost_from_dhw_tank_max_inc")
    openamber.step(ms=50)
    assert float(openamber.get_entity("defrost_from_dhw_tank_max_temperature")) == 61.0

    # No wrap-around at the ends, and a change from elsewhere updates the label.
    assert openamber.set_number("defrost_from_dhw_tank_max_temperature", 75.0)
    assert openamber.click("set_defrost_from_dhw_tank_max_inc")
    openamber.step(ms=50)
    assert float(openamber.get_entity("defrost_from_dhw_tank_max_temperature")) == 75.0
    assert openamber.get_label("set_defrost_from_dhw_tank_max_val") == "75°C"
