"""Emergency mode enabled while heating runs the compressor next to the backup heater.

When the supply reaches the stop temperature, heating turns the backup heater off and continues with
pump-only control. The compressor must be stopped as well: otherwise it keeps running while heating
later idles and the 3-way valve may switch to DHW.
"""

from helpers import advance_until_state, start_space_heating


def test_emergency_mode_backup_heater_exit_stops_compressor(clean_system):
    openamber = clean_system
    start_space_heating(openamber, tc=20.0, target=35.0)
    # As in test_backup_heater_degree_minutes: "Beperkt" puts the compressor at its maximum mode at once.
    openamber.set_select("heat_compressor_mode", "Beperkt")
    openamber.set_select("backup_heating_mode", "Intern verwarmingselement")
    openamber.set_number("backup_heater_degmin_threshold", 10.0)
    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Backup heater running", max_s=600, step=10), (
        f"Backup heater never turned on (state: {openamber.get_entity('state_machine_state_heat_cool')})"
    )
    assert int(openamber.get_entity("compressor_control_select") or 0) > 0, "The compressor runs next to the backup heater"

    openamber.set_switch("emergency_mode_enabled", True)
    openamber.step(ms=50)
    # The supply reaches setpoint + stop delta (35 + 2): the backup heater turns off.
    for sensor, value in (("current_water_temperature_tc_sensor", 38.0), ("heat_cool_temperature_tc", 38.0),
                          ("outlet_temperature_tuo", 38.0), ("inlet_temperature_tui", 36.0)):
        openamber.set_sensor(sensor, value)
    openamber.advance_time(seconds=10, step_s=2)
    openamber.step(ms=50)

    assert openamber.get_entity("backup_heater_relay") is False
    assert int(openamber.get_entity("compressor_control_select") or 0) == 0, (
        "Pump-only control in emergency mode must not leave the compressor running"
    )

    openamber.set_switch("emergency_mode_enabled", False)
    openamber.set_switch("heat_demand_switch", False)
    openamber.advance_time(seconds=60, step_s=5)
