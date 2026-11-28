"""The backup heater boost after a defrost (Ta average <= "Defrost boost buitentemperatuur").

The boost only bridges the defrost dip: it ends once the control temperature is back at its value from the defrost
start, or after DEFROST_BACKUP_HEATER_BOOST_MAX_S (15 min). While the backup heater runs the compressor climbs to its
maximum allowed mode instead of staying at the mode it had before the defrost.
"""

from helpers import advance_until_state, set_outside_temperature

DEFROST_START_TC = 30.0
BOOST_MAX_S = 15 * 60


def _compressor_mode(openamber):
    return int(openamber.get_entity("compressor_control_select") or 0)


def _set_tc(openamber, tc):
    openamber.set_sensor("heat_cool_temperature_tc", tc)
    openamber.set_sensor("outlet_temperature_tuo", tc)
    openamber.set_sensor("inlet_temperature_tui", tc - 2.0)


def _heat_through_defrost_into_boost(openamber):
    """Space heating at Ta -5 °C, a defrost that drops Tc from 30 to 24 °C, then the boost backup heater."""
    openamber.set_select("thermostat_mode_select", "Extern")
    openamber.set_select("heat_mode_select", "Extern setpoint")
    openamber.set_number("manual_setpoint", 35.0)
    openamber.set_climate("pid_heat_temperature_control", target_temperature=35.0)
    openamber.set_number("compressor_stop_delta_heating", 5.0)
    openamber.set_number("backup_heater_degmin_threshold", 9999.0)
    openamber.set_number("defrost_backup_heater_boost_temperature_sensor", -3.0)
    set_outside_temperature(openamber, -5.0)
    _set_tc(openamber, 28.0)
    openamber.set_switch("heat_demand_switch", True)
    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Compressor running", max_s=1800, step=10)

    _set_tc(openamber, DEFROST_START_TC)
    openamber.step(ms=50)
    openamber.set_binary_sensor("defrost_active_sensor", True)
    openamber.advance_time(seconds=10, step_s=2)
    openamber.step(ms=50)
    assert openamber.get_entity("state_machine_state_heat_cool") == "Defrosting"

    _set_tc(openamber, 24.0)
    openamber.set_binary_sensor("defrost_active_sensor", False)
    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Backup heater running", max_s=60, step=2)
    assert openamber.get_entity("backup_heater_relay") is True


def test_defrost_boost_ends_when_temperature_is_back_at_defrost_start(clean_system):
    openamber = clean_system
    _heat_through_defrost_into_boost(openamber)

    # Still below the temperature from the defrost start: the boost goes on.
    _set_tc(openamber, 28.0)
    openamber.advance_time(seconds=120, step_s=10)
    openamber.step(ms=50)
    assert openamber.get_entity("backup_heater_relay") is True, "Boost must run until the defrost dip is recovered"

    # Back at the defrost start temperature, still 5 K below the setpoint: the boost ends.
    _set_tc(openamber, DEFROST_START_TC)
    openamber.advance_time(seconds=10, step_s=2)
    openamber.step(ms=50)
    assert openamber.get_entity("backup_heater_relay") is False, "Boost must end once Tc is back at the defrost start value"
    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Compressor running", max_s=300, step=10)
    assert _compressor_mode(openamber) > 0


def test_defrost_boost_ends_after_maximum_time(clean_system):
    openamber = clean_system
    _heat_through_defrost_into_boost(openamber)

    # Tc never gets back to the defrost start value.
    _set_tc(openamber, 26.0)
    openamber.advance_time(seconds=BOOST_MAX_S - 120, step_s=10)
    openamber.step(ms=50)
    assert openamber.get_entity("backup_heater_relay") is True, "Boost must still run before the maximum time"

    openamber.advance_time(seconds=180, step_s=10)
    openamber.step(ms=50)
    assert openamber.get_entity("backup_heater_relay") is False, "Boost must end after the maximum boost time"
    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Compressor running", max_s=300, step=10)


def test_compressor_climbs_to_max_mode_while_backup_heater_runs(clean_system):
    openamber = clean_system
    _heat_through_defrost_into_boost(openamber)

    # The compressor came out of the defrost at a low mode (36 Hz in the field case).
    openamber.set_select("compressor_control_select", "3")
    openamber.step(ms=50)
    _set_tc(openamber, 26.0)

    openamber.advance_time(seconds=11 * 60, step_s=10)
    openamber.step(ms=50)
    assert openamber.get_entity("state_machine_state_heat_cool") == "Backup heater running"
    assert _compressor_mode(openamber) >= 5, (
        f"The compressor must step up towards its maximum mode during the backup heater, got {_compressor_mode(openamber)}"
    )
