"""3-way valve position check from temperatures (TO04).

After the compressor has run for 5 minutes, the outlet water (Tuo - 5 °C) must stay within 12 °C of the circuit the
valve should serve: not far above Tc while heating, not far below Tw while heating DHW (DHW demand, DHW pump running).
A larger deviation that lasts 2 minutes raises TO04 and stops the system, like the existing check on the CV
temperature during DHW.
"""
from helpers import advance_until_state, start_space_heating

RUN_TIME_S = 5 * 60
HOLD_TIME_S = 2 * 60
SOFTSTART_S = 180  # 'Compressor running' follows the soft start; the run time counts from the soft start


def compressor_requested(openamber):
    return int(openamber.get_entity("compressor_control_select") or 0) > 0


def test_heating_with_water_from_dhw_side_raises_to04(clean_system):
    """Heating while the outlet water is far above Tc (valve stuck on DHW): TO04 5 + 2 minutes after the compressor
    started (the run time counts from the soft start), then a stop."""
    openamber = clean_system
    start_space_heating(openamber, tc=28.0, target=35.0)

    # Tuo far above Tc, Tuo - Tui small so the Tuo/Tui check does not stop first: (50 - 5) - 30 = 15 > 12.
    openamber.set_sensor("heat_cool_temperature_tc", 30.0)
    openamber.set_sensor("outlet_temperature_tuo", 50.0)
    openamber.set_sensor("inlet_temperature_tui", 45.0)

    # Within the run time plus the hold time, counted from the soft start, nothing happens yet.
    openamber.advance_time(seconds=RUN_TIME_S + HOLD_TIME_S - SOFTSTART_S - 40, step_s=5)
    assert openamber.get_entity("error_three_way_valve_state_timeout") is False
    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"

    # Had the run time started only in 'Compressor running', TO04 would come 3 minutes later.
    openamber.advance_time(seconds=80, step_s=5)
    assert openamber.get_entity("error_three_way_valve_state_timeout") is True
    assert openamber.get_entity("error_active") is True

    openamber.advance_time(seconds=120, step_s=5)
    assert not compressor_requested(openamber), "The compressor must be stopped after TO04"


def test_heating_with_backup_heater_and_compressor_is_checked(clean_system):
    """SG Ready max boost adds the backup heater while the compressor keeps running: the check stays active."""
    openamber = clean_system
    openamber.set_select("backup_heating_mode", "Intern verwarmingselement")
    start_space_heating(openamber, tc=28.0, target=35.0)

    openamber.set_select("sg_ready_mode_select", "Max boost")
    openamber.advance_time(seconds=10, step_s=2)
    assert openamber.get_entity("backup_heater_relay") is True
    assert openamber.get_entity("state_machine_state_heat_cool") in ("Wait backup heater running", "Backup heater running")
    assert compressor_requested(openamber)

    openamber.set_sensor("heat_cool_temperature_tc", 30.0)
    openamber.set_sensor("outlet_temperature_tuo", 50.0)
    openamber.set_sensor("inlet_temperature_tui", 45.0)
    # The run time counts from the soft start (10 s of max boost already passed).
    openamber.advance_time(seconds=RUN_TIME_S + HOLD_TIME_S - SOFTSTART_S - 50, step_s=5)
    assert openamber.get_entity("state_machine_state_heat_cool") == "Backup heater running"
    assert openamber.get_entity("error_three_way_valve_state_timeout") is False

    openamber.advance_time(seconds=80, step_s=5)
    assert openamber.get_entity("error_three_way_valve_state_timeout") is True
    openamber.set_select("sg_ready_mode_select", "Normaal")


def test_heating_with_normal_temperatures_keeps_running(clean_system):
    """Normal heating (Tuo a few degrees above Tc) never raises TO04."""
    openamber = clean_system
    start_space_heating(openamber, tc=28.0, target=35.0)

    openamber.set_sensor("heat_cool_temperature_tc", 30.0)
    openamber.set_sensor("outlet_temperature_tuo", 34.0)
    openamber.set_sensor("inlet_temperature_tui", 30.0)
    openamber.advance_time(seconds=20 * 60, step_s=10)

    assert openamber.get_entity("error_three_way_valve_state_timeout") is False
    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"


def test_heating_deviation_of_exactly_12_degrees_is_allowed(clean_system):
    """A deviation of exactly 12 °C above Tc is still within the limit: no TO04."""
    openamber = clean_system
    start_space_heating(openamber, tc=28.0, target=35.0)
    openamber.set_sensor("heat_cool_temperature_tc", 30.0)

    # (47 - 5) - 30 = +12
    openamber.set_sensor("outlet_temperature_tuo", 47.0)
    openamber.set_sensor("inlet_temperature_tui", 42.0)
    openamber.advance_time(seconds=RUN_TIME_S + HOLD_TIME_S + 60, step_s=10)
    assert openamber.get_entity("error_three_way_valve_state_timeout") is False


def test_heating_with_outlet_far_below_tc_is_not_checked(clean_system):
    """While heating only water far above Tc counts: with a stratified buffer Tc (top) can be far above the water the
    heat pump draws in, with the valve in the right position."""
    openamber = clean_system
    start_space_heating(openamber, tc=28.0, target=45.0)

    # (25 - 5) - 38 = -18
    openamber.set_sensor("heat_cool_temperature_tc", 38.0)
    openamber.set_sensor("outlet_temperature_tuo", 25.0)
    openamber.set_sensor("inlet_temperature_tui", 22.0)
    openamber.advance_time(seconds=RUN_TIME_S + HOLD_TIME_S + 120, step_s=10)
    assert openamber.get_entity("error_three_way_valve_state_timeout") is False


def test_heating_deviation_that_recovers_does_not_raise_to04(clean_system):
    """A deviation shorter than the hold time resets: no TO04."""
    openamber = clean_system
    start_space_heating(openamber, tc=28.0, target=35.0)
    openamber.set_sensor("heat_cool_temperature_tc", 30.0)
    openamber.set_sensor("inlet_temperature_tui", 30.0)
    openamber.set_sensor("outlet_temperature_tuo", 34.0)
    openamber.advance_time(seconds=RUN_TIME_S, step_s=10)

    openamber.set_sensor("inlet_temperature_tui", 45.0)
    openamber.set_sensor("outlet_temperature_tuo", 50.0)
    openamber.advance_time(seconds=HOLD_TIME_S - 30, step_s=5)
    openamber.set_sensor("inlet_temperature_tui", 30.0)
    openamber.set_sensor("outlet_temperature_tuo", 34.0)
    openamber.advance_time(seconds=HOLD_TIME_S, step_s=5)

    assert openamber.get_entity("error_three_way_valve_state_timeout") is False


def start_dhw_with_pump(openamber):
    """Start DHW heating and bring the DHW pump on (Tuo above Tw)."""
    openamber.set_select("dhw_pump_start_mode_select", "Aanvoer warmer dan vat (ΔT)")
    openamber.set_sensor("outlet_temperature_tuo", 45.0)
    openamber.set_sensor("inlet_temperature_tui", 40.0)
    openamber.set_sensor("dhw_temperature_tw_sensor", 40.0)
    openamber.step(ms=100)
    assert advance_until_state(openamber, "state_machine_state_dhw", "Compressor running", max_s=900, step=5)
    openamber.advance_time(seconds=30, step_s=5)
    assert openamber.get_entity("dhw_pump_relay_switch") is True


def test_dhw_with_water_from_cv_side_raises_to04(clean_system):
    """DHW with the DHW pump running while the outlet water stays far below Tw (valve stuck on CV): TO04."""
    openamber = clean_system
    start_dhw_with_pump(openamber)

    # (30 - 5) - 40 = -15 <= -12; Tuo - Tui small so the Tuo/Tui check does not stop first.
    openamber.set_sensor("outlet_temperature_tuo", 30.0)
    openamber.set_sensor("inlet_temperature_tui", 27.0)

    openamber.advance_time(seconds=RUN_TIME_S + HOLD_TIME_S - 60, step_s=5)
    assert openamber.get_entity("error_three_way_valve_state_timeout") is False

    openamber.advance_time(seconds=120, step_s=5)
    assert openamber.get_entity("error_three_way_valve_state_timeout") is True
    assert openamber.get_entity("error_active") is True


def test_dhw_without_running_dhw_pump_is_not_checked(clean_system):
    """With the ΔT start mode the DHW pump waits for Tuo above Tw; until then the cold outlet water is expected."""
    openamber = clean_system
    openamber.set_select("dhw_pump_start_mode_select", "Aanvoer warmer dan vat (ΔT)")
    openamber.set_sensor("outlet_temperature_tuo", 28.0)
    openamber.set_sensor("inlet_temperature_tui", 26.0)
    openamber.set_sensor("dhw_temperature_tw_sensor", 40.0)
    openamber.step(ms=100)
    assert advance_until_state(openamber, "state_machine_state_dhw", "Compressor running", max_s=900, step=5)

    openamber.advance_time(seconds=RUN_TIME_S + HOLD_TIME_S + 120, step_s=10)
    assert openamber.get_entity("dhw_pump_relay_switch") is False
    assert openamber.get_entity("error_three_way_valve_state_timeout") is False


def test_heating_after_hot_switch_from_dhw_starts_the_run_time_again(clean_system):
    """Heating -> DHW -> heating without stopping the compressor: back on heating, water still hot from the tank
    (Tuo far above Tc) must not raise TO04 within the run time. The time before the hand-over does not count."""
    openamber = clean_system
    start_space_heating(openamber, tc=28.0, target=35.0)
    openamber.set_sensor("heat_cool_temperature_tc", 30.0)
    openamber.set_sensor("outlet_temperature_tuo", 34.0)
    openamber.set_sensor("inlet_temperature_tui", 31.0)
    # Heating runs longer than the run time, so a stale run timer would let the check start at once after DHW.
    openamber.advance_time(seconds=RUN_TIME_S + 60, step_s=10)

    # DHW request during heating: hot switch to DHW (outlet above the tank).
    openamber.set_sensor("dhw_temperature_tw_sensor", 38.0)
    openamber.set_sensor("outlet_temperature_tuo", 42.0)
    openamber.set_sensor("inlet_temperature_tui", 39.0)
    assert advance_until_state(openamber, "state_machine_state_main", "DHW", max_s=900, step=5)
    assert advance_until_state(openamber, "state_machine_state_dhw", "Compressor running", max_s=600, step=5)
    openamber.advance_time(seconds=120, step_s=5)

    # Tank at setpoint with heat demand: hot switch back to heating with water still hot from the tank.
    openamber.set_sensor("dhw_temperature_tw_sensor", 52.0)
    openamber.set_sensor("outlet_temperature_tuo", 50.0)  # (50 - 5) - 30 = 15 > 12
    openamber.set_sensor("inlet_temperature_tui", 46.0)
    assert advance_until_state(openamber, "state_machine_state_main", "Heat/Cool", max_s=200, step=5)
    assert compressor_requested(openamber), "The hot switch keeps the compressor running"

    # Longer than the hold time, shorter than the run time: no TO04.
    openamber.advance_time(seconds=HOLD_TIME_S + 30, step_s=5)
    assert openamber.get_entity("error_three_way_valve_state_timeout") is False

    # The water clears: heating continues.
    openamber.set_sensor("outlet_temperature_tuo", 34.0)
    openamber.set_sensor("inlet_temperature_tui", 31.0)
    openamber.advance_time(seconds=RUN_TIME_S + HOLD_TIME_S, step_s=10)
    assert openamber.get_entity("error_three_way_valve_state_timeout") is False
    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"
