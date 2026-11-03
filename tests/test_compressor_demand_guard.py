"""Regression test: the compressor must not start in PUMP_RUNNING without compressor demand.

Frost protection stage 1 is pump-only demand (HasPumpDemand) and no compressor demand
(HasCompressorDemand). When the pump interval cycle lasts longer than the pump settle time
(COMPRESSOR_MIN_TIME_PUMP_ON = 120s) and the control temperature is below the heating start
temperature, PUMP_RUNNING used to fall through to the compressor start because only
IsCompressorAllowedToStart() was checked once the interval cycle was still running.
"""

from helpers import advance_until_state


def test_frost_stage1_without_heat_demand_does_not_start_compressor(clean_system):
    openamber = clean_system

    original_pump_duration = openamber.get_entity("pump_duration")

    try:
        # Pump cycle longer than the 120s pump settle time, so the settle check passes while the
        # interval cycle is still running.
        openamber.set_number("pump_duration", 5.0)

        # No heat/cool demand at all.
        openamber.set_binary_sensor("external_heat_demand_wired", False)
        openamber.set_switch("heat_demand_switch", False)
        openamber.set_binary_sensor("external_cool_demand_wired", False)
        openamber.set_switch("cool_demand_switch", False)

        # Tc well below heating target - start delta: IsCompressorAllowedToStart() is true.
        openamber.set_climate("pid_heat_temperature_control", target_temperature=30.0)
        openamber.set_number("compressor_start_delta_heating", 2.0)
        openamber.set_sensor("current_water_temperature_tc_sensor", 20.0)
        openamber.set_sensor("heat_cool_temperature_tc", 20.0)
        openamber.set_sensor("outlet_temperature_tuo", 20.0)

        # Frost protection stage 1 only (Tui stays above the stage 2 threshold).
        openamber.set_number("frost_protection_stage_1_temp_ta", 5.0)
        openamber.set_number("frost_protection_stage_2_temp_ta", 4.0)
        openamber.set_number("frost_protection_stage_2_temp_tui", 7.0)
        openamber.set_sensor("inlet_temperature_tui", 20.0)
        openamber.set_sensor("temperature_outside_ta", 2.0)
        openamber.step(ms=100)

        # 300s delayed_on filter on the frost protection sensor.
        openamber.advance_time(seconds=305, step_s=10)
        openamber.step(ms=100)

        assert openamber.get_entity("frost_protection_stage1_active") is True
        assert openamber.get_entity("frost_protection_stage2_active") is False
        assert openamber.get_entity("heat_demand_active_sensor") is False
        assert advance_until_state(openamber, "state_machine_state_heat_cool", "Pump running", max_s=30), (
            f"Pump should run for frost protection (state: {openamber.get_entity('state_machine_state_heat_cool')})"
        )

        # Run through the pump settle time and most of the 5 min interval cycle.
        for _ in range(24):
            openamber.advance_time(seconds=10, step_s=10)
            openamber.step(ms=20)
            state = openamber.get_entity("state_machine_state_heat_cool")
            comp_mode = int(openamber.get_entity("compressor_control_select") or 0)
            assert state == "Pump running", f"Compressor must not start without compressor demand (state: {state})"
            assert comp_mode == 0, f"Compressor must stay off without compressor demand, got mode {comp_mode}"
    finally:
        openamber.set_sensor("temperature_outside_ta", 7.0)
        if original_pump_duration is not None:
            openamber.set_number("pump_duration", float(original_pump_duration))
        openamber.advance_time(seconds=640, step_s=20)
        openamber.step(ms=100)
