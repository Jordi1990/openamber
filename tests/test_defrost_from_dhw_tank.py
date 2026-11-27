"""Tests for "Ontdooien via tapwatervat" (defrost_from_dhw_tank_switch).

With the switch on, a defrost during heating takes its heat from the DHW tank when the heating circuit is too cold
to defrost on (Tuo at the start of the defrost at or below 22.5 °C), as the original controller does. The 3-way
valve then goes to DHW while the compressor and pump P0 keep running (DHW pump relay on) and goes back when the
defrost ends. Only with "Tapwater aanwezig" on and the tank above 30 °C and at most 60 °C. When the tank gives no
heat (Tui does not rise) the defrost goes back to the heating circuit and the tank is skipped until after a DHW
run. Defaults to off.
"""

import os

import pytest
import yaml

from helpers import advance_until_state
from test_entity_schema_sync import ESPHomeYamlLoader

CORE_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src", "openamber", "core")
SWITCHES_YAML = os.path.join(CORE_DIR, "switches.yaml")


@pytest.fixture
def tank_defrost_system(clean_system):
    """clean_system does not know this setting: put it back for the tests that follow."""
    yield clean_system
    clean_system.set_sensor("dhw_temperature_tw_sensor", 50.0)
    clean_system.set_binary_sensor("defrost_active_sensor", False)
    clean_system.set_switch("defrost_from_dhw_tank_switch", False)
    clean_system.set_switch("heat_demand_switch", False)


def _start_heating(openamber, tank_temperature=50.0):
    """Space heating up to 'Compressor running', with the tank at tank_temperature and no DHW demand."""
    # A previous test can leave the compressor running down: start from idle.
    openamber.set_sensor("dhw_temperature_tw_sensor", tank_temperature)
    openamber.step(ms=50)
    assert advance_until_state(
        openamber, "state_machine_state_heat_cool", "Idle", max_s=1500, step=10
    ), "Heating did not return to idle before the test"
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

    assert advance_until_state(
        openamber, "state_machine_state_heat_cool", "Compressor running", max_s=1500, step=10
    ), "Heating did not reach 'Compressor running'"
    assert openamber.get_entity("dhw_demand_active_sensor") is False, "The tank must not ask for a DHW run"
    assert openamber.get_entity("three_way_valve_dhw_switch") is False


def _start_defrost(openamber, tuo=21.0, tui=19.0):
    """Start a defrost with the heating circuit at tuo/tui (default: too cold to defrost on)."""
    openamber.set_sensor("outlet_temperature_tuo", tuo)
    openamber.set_sensor("inlet_temperature_tui", tui)
    # Tuo and Tui are read every 15 s: let the new values arrive before the defrost starts.
    openamber.advance_time(seconds=20, step_s=5)
    openamber.set_binary_sensor("defrost_active_sensor", True)
    # The state machine runs every 5 s: one pass to enter DEFROSTING, the next one moves the valve.
    openamber.advance_time(seconds=20, step_s=2)
    openamber.step(ms=50)
    assert openamber.get_entity("state_machine_state_heat_cool") == "Defrosting"


def _end_defrost(openamber):
    openamber.set_binary_sensor("defrost_active_sensor", False)
    openamber.advance_time(seconds=20, step_s=2)
    openamber.step(ms=50)


def _on_tank(openamber):
    return (
        openamber.get_entity("three_way_valve_dhw_switch") is True
        and openamber.get_entity("three_way_valve_heat_cool_switch") is False
        and openamber.get_entity("dhw_pump_relay_switch") is True
    )


def _on_heating_circuit(openamber):
    return (
        openamber.get_entity("three_way_valve_dhw_switch") is False
        and openamber.get_entity("three_way_valve_heat_cool_switch") is True
        and openamber.get_entity("dhw_pump_relay_switch") is False
    )


def test_defrost_from_dhw_tank_defaults_off():
    """Existing installs keep defrosting on the heating circuit."""
    with open(SWITCHES_YAML, encoding="utf-8") as f:
        switches = yaml.load(f, Loader=ESPHomeYamlLoader)["switch"]
    entry = next(s for s in switches if s.get("id") == "defrost_from_dhw_tank_switch")
    assert entry["restore_mode"] == "RESTORE_DEFAULT_OFF"
    assert entry["name"] == "Ontdooien via tapwatervat"


def test_defrost_stays_on_heating_circuit_when_switched_off(tank_defrost_system):
    openamber = tank_defrost_system
    openamber.set_switch("defrost_from_dhw_tank_switch", False)
    _start_heating(openamber)

    _start_defrost(openamber)

    assert _on_heating_circuit(openamber)


def test_defrost_stays_on_heating_circuit_when_it_is_warm_enough(tank_defrost_system):
    """Tuo above 22.5 °C at the start of the defrost: the heating circuit has enough heat, the tank is left alone."""
    openamber = tank_defrost_system
    openamber.set_switch("defrost_from_dhw_tank_switch", True)
    _start_heating(openamber)

    _start_defrost(openamber, tuo=28.0, tui=26.0)

    assert _on_heating_circuit(openamber)


def test_defrost_takes_heat_from_dhw_tank(tank_defrost_system):
    """Cold heating circuit: valve to DHW at the start of the defrost, back to heating when it ends; the compressor
    and P0 keep running and nothing raises an error."""
    openamber = tank_defrost_system
    openamber.set_switch("defrost_from_dhw_tank_switch", True)
    _start_heating(openamber)

    _start_defrost(openamber)

    assert _on_tank(openamber), "3-way valve and DHW pump relay should go to the tank"
    assert openamber.get_entity("state_machine_state_main") == "Heat/Cool", "Still a heating cycle, not a DHW run"
    assert openamber.get_entity("pump_p0_relay_switch") is True
    assert int(openamber.get_entity("compressor_control_select") or 0) > 0, "Compressor keeps running"

    # Tank water reaches the unit: Tui rises far above Tuo. The tank stays in use and the delta-T check, which only
    # looks at Tuo above Tui while heating, does not trip. The relay feedback is confirmed after the travel time.
    openamber.set_sensor("inlet_temperature_tui", 40.0)
    openamber.set_sensor("outlet_temperature_tuo", 30.0)
    openamber.advance_time(seconds=150, step_s=10)
    openamber.step(ms=50)
    assert openamber.get_entity("state_machine_state_heat_cool") == "Defrosting"
    assert _on_tank(openamber)
    assert openamber.get_entity("error_active") is False

    _end_defrost(openamber)

    assert _on_heating_circuit(openamber), "3-way valve and DHW pump relay should go back when the defrost ends"
    assert openamber.get_entity("state_machine_state_main") == "Heat/Cool"
    assert int(openamber.get_entity("compressor_control_select") or 0) > 0

    # Heating circuit water is back in the unit; the cycle continues after the settle time.
    openamber.set_sensor("inlet_temperature_tui", 26.0)
    openamber.set_sensor("outlet_temperature_tuo", 28.0)
    assert advance_until_state(
        openamber, "state_machine_state_heat_cool", "Compressor running", max_s=400, step=10
    ), "Heating did not continue after the defrost"
    assert _on_heating_circuit(openamber)
    assert openamber.get_entity("error_active") is False


def test_cold_tank_falls_back_to_heating_circuit_until_dhw_run(tank_defrost_system):
    """Tw says 50 °C but the tank gives no heat (Tui keeps falling): after the check time the defrost goes back to
    the heating circuit, and the next defrost does not try the tank again. After a DHW run it does."""
    openamber = tank_defrost_system
    openamber.set_switch("defrost_from_dhw_tank_switch", True)
    _start_heating(openamber)

    _start_defrost(openamber)
    assert _on_tank(openamber)

    # Still on the tank before the check time, back on the heating circuit after it.
    openamber.set_sensor("inlet_temperature_tui", 17.0)
    openamber.set_sensor("outlet_temperature_tuo", 13.0)
    openamber.advance_time(seconds=60, step_s=10)
    openamber.step(ms=50)
    assert _on_tank(openamber), "The tank gets the valve travel time before it is judged"
    openamber.advance_time(seconds=40, step_s=10)
    openamber.step(ms=50)
    assert _on_heating_circuit(openamber), "A tank that gives no heat is dropped during the defrost"
    assert openamber.get_entity("state_machine_state_heat_cool") == "Defrosting"

    _end_defrost(openamber)
    assert advance_until_state(
        openamber, "state_machine_state_heat_cool", "Compressor running", max_s=400, step=10
    )

    # Next defrost, heating circuit still cold: the tank is not tried again.
    _start_defrost(openamber)
    assert _on_heating_circuit(openamber), "The tank is skipped until after a DHW run"
    _end_defrost(openamber)
    assert advance_until_state(
        openamber, "state_machine_state_heat_cool", "Compressor running", max_s=400, step=10
    )

    # A DHW run heats the tank: after it the tank may be used again.
    openamber.set_sensor("inlet_temperature_tui", 26.0)
    openamber.set_sensor("outlet_temperature_tuo", 28.0)
    openamber.set_sensor("dhw_temperature_tw_sensor", 40.0)
    assert advance_until_state(openamber, "state_machine_state_main", "DHW", max_s=900, step=10), "DHW run did not start"
    openamber.set_sensor("dhw_temperature_tw_sensor", 56.0)
    assert advance_until_state(
        openamber, "state_machine_state_main", "Heat/Cool", max_s=1500, step=10
    ), "DHW run did not hand back to heating"
    assert advance_until_state(
        openamber, "state_machine_state_heat_cool", "Compressor running", max_s=1500, step=10
    )
    openamber.advance_time(seconds=180, step_s=10)

    _start_defrost(openamber)
    assert _on_tank(openamber), "After a DHW run the tank is used again"


@pytest.mark.parametrize("tank_temperature, used", [
    (30.0, False),  # at or below the minimum
    (61.0, False),  # above the maximum
    (56.0, True),   # a tank just above its 55 °C setpoint is still used
])
def test_tank_temperature_limits(tank_defrost_system, tank_temperature, used):
    """The tank is used above 30 °C and up to 60 °C."""
    openamber = tank_defrost_system
    openamber.set_switch("defrost_from_dhw_tank_switch", True)
    # A low DHW setpoint, so the 30 °C tank does not ask for a DHW run (which would take the compressor over).
    dhw_setpoint = openamber.get_entity("dhw_setpoint_temperature")
    openamber.set_number("dhw_setpoint_temperature", 35.0)
    try:
        _start_heating(openamber)

        # The tank temperature counts at the start of the defrost.
        openamber.set_sensor("dhw_temperature_tw_sensor", tank_temperature)
        _start_defrost(openamber)

        assert _on_tank(openamber) is used
        if not used:
            assert _on_heating_circuit(openamber)
    finally:
        openamber.set_number("dhw_setpoint_temperature", float(dhw_setpoint))


def test_defrost_stays_on_heating_circuit_without_dhw(tank_defrost_system):
    """"Tapwater aanwezig" off: there is no tank to defrost from."""
    openamber = tank_defrost_system
    openamber.set_switch("defrost_from_dhw_tank_switch", True)
    openamber.set_switch("dhw_enabled_switch", False)
    _start_heating(openamber)

    _start_defrost(openamber)

    assert _on_heating_circuit(openamber)


def test_missing_valve_feedback_during_tank_defrost_raises_error(tank_defrost_system):
    """The valve move for a tank defrost checks the relay feedback after the travel time, like a hot switch."""
    openamber = tank_defrost_system
    openamber.set_switch("defrost_from_dhw_tank_switch", True)
    _start_heating(openamber)
    assert openamber.set_switch("mock_three_way_valve_feedback_missing", True)

    _start_defrost(openamber)
    assert openamber.get_entity("three_way_valve_dhw_switch") is True
    openamber.set_sensor("inlet_temperature_tui", 40.0)
    openamber.set_sensor("outlet_temperature_tuo", 30.0)
    openamber.advance_time(seconds=80, step_s=10)
    openamber.step(ms=50)

    assert openamber.get_entity("error_three_way_valve_state_timeout") is True


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
