"""Automated tests for Cooling Deltas and Cooling Pump Speed.

Covers settings:
1. compressor_start_delta_cooling:
   Compressor is not allowed to start until supply temperature (Tc) exceeds cooling setpoint + start_delta.
2. compressor_stop_delta_cooling:
   Compressor runs until overshoot (setpoint - Tc) reaches or exceeds stop_delta.
3. pump_speed_cooling_number:
   During active cooling, pump PWM is modulated to (1000 - speed * 10).
"""

import pytest


def test_cooling_start_delta_hysteresis(clean_system):
    """
    Verify compressor start delta in Cooling mode:
    - Target cooling setpoint = 18.0°C.
    - compressor_start_delta_cooling = 2.5°C -> threshold = 20.5°C.
    - If Tc = 20.0°C (below threshold), compressor remains OFF (0).
    - When Tc rises to 21.0°C (> 20.5°C), compressor starts (mode > 0).
    """
    openamber = clean_system

    openamber.set_select("cool_mode_select", "Intern setpoint")
    openamber.set_number("cooling_setpoint_number", 18.0)
    openamber.set_number("compressor_start_delta_cooling", 2.5)
    openamber.set_number("compressor_stop_delta_cooling", 2.0)
    openamber.set_select("cool_compressor_mode", "Maximaal")

    # Set supply temp Tc to 20.0°C (below 18.0 + 2.5 = 20.5°C)
    openamber.set_sensor("current_water_temperature_tc_sensor", 20.0)
    openamber.set_sensor("heat_cool_temperature_tc", 20.0)
    openamber.set_sensor("outlet_temperature_tuo", 20.0)
    openamber.set_sensor("inlet_temperature_tui", 22.0)
    openamber.step(ms=50)

    # Allow minimum off time to pass
    openamber.advance_time(seconds=130, step_s=10)
    openamber.step(ms=50)

    # Request cooling
    openamber.set_binary_sensor("external_cool_demand_wired", True)
    openamber.step(ms=100)
    assert openamber.get_entity("cool_demand_active_sensor") is True

    # Advance through interval cycle while below threshold (pump runs 120s and stops)
    openamber.advance_time(seconds=920, step_s=30)
    openamber.advance_time(seconds=140, step_s=10)
    openamber.step(ms=100)

    # Compressor should NOT have started because Tc was 20.0°C <= 20.5°C threshold
    comp_mode = int(openamber.get_entity("compressor_control_select") or 0)
    assert comp_mode == 0, f"Compressor should not start below 20.5°C, got mode {comp_mode}"

    # Now water warms up to 21.5°C (> 20.5°C threshold)
    openamber.set_sensor("current_water_temperature_tc_sensor", 21.5)
    openamber.set_sensor("heat_cool_temperature_tc", 21.5)
    openamber.set_sensor("outlet_temperature_tuo", 21.5)
    openamber.set_climate("pid_cool_temperature_control", target_temperature=18.0)
    openamber.step(ms=50)

    # Advance through next pump interval + settle time (130s)
    openamber.advance_time(seconds=920, step_s=30)
    openamber.advance_time(seconds=140, step_s=10)
    openamber.step(ms=100)

    comp_mode = int(openamber.get_entity("compressor_control_select") or 0)
    assert comp_mode > 0, f"Compressor should start above 20.5°C threshold, got mode {comp_mode}"

    # Cleanup
    openamber.set_binary_sensor("external_cool_demand_wired", False)
    openamber.step(ms=100)


def test_cooling_stop_delta_and_pump_speed(clean_system):
    """
    Verify compressor stop delta and cooling pump speed:
    - Target cooling setpoint = 18.0°C.
    - compressor_stop_delta_cooling = 2.0°C -> stops when Tc <= 16.0°C.
    - pump_speed_cooling_number = 75% -> PWM = 250 (1000 - 75 * 10).
    - When Tc drops to 16.5°C (overshoot = 1.5°C < 2.0°C), compressor continues.
    - When Tc drops to 15.8°C (overshoot = 2.2°C >= 2.0°C), compressor stops.
    """
    openamber = clean_system

    openamber.set_select("cool_mode_select", "Intern setpoint")
    openamber.set_number("cooling_setpoint_number", 18.0)
    openamber.set_number("compressor_start_delta_cooling", 1.0)
    openamber.set_number("compressor_stop_delta_cooling", 2.0)
    openamber.set_number("pump_speed_cooling_number", 75.0)

    # Initial warm water (23°C) to start cooling easily
    openamber.set_sensor("current_water_temperature_tc_sensor", 23.0)
    openamber.set_sensor("heat_cool_temperature_tc", 23.0)
    openamber.set_sensor("outlet_temperature_tuo", 23.0)
    openamber.set_sensor("inlet_temperature_tui", 24.0)
    openamber.step(ms=50)

    # Advance past min off time
    openamber.advance_time(seconds=130, step_s=10)
    openamber.step(ms=50)

    # Trigger cool demand
    openamber.set_binary_sensor("external_cool_demand_wired", True)
    openamber.step(ms=100)

    # Advance pump interval (900s) + pump start (10s)
    openamber.advance_time(seconds=910, step_s=30)
    openamber.step(ms=100)
    assert openamber.get_entity("internal_pump_active") is True

    # Settle pump and start compressor
    openamber.advance_time(seconds=30, step_s=5)
    openamber.step(ms=100)
    comp_mode = int(openamber.get_entity("compressor_control_select") or 0)
    assert comp_mode > 0, "Compressor should be running in cooling"

    # Verify Cooling Pump Speed PWM at 75% -> 1000 - 750 = 250
    current_pwm = float(openamber.get_entity("pump_control_pwm_number") or openamber.get_entity("pump_p0_current_pwm_sensor") or 0)
    assert current_pwm == 250.0, f"Expected 250.0 PWM for 75% cooling pump speed, got {current_pwm}"

    # Water cools down to 16.5°C (target 18.0 - 16.5 = 1.5°C overshoot < stop delta 2.0°C)
    openamber.set_sensor("current_water_temperature_tc_sensor", 16.5)
    openamber.set_sensor("heat_cool_temperature_tc", 16.5)
    openamber.set_sensor("outlet_temperature_tuo", 16.5)
    openamber.step(ms=50)

    # Advance past min run time (600s)
    openamber.advance_time(seconds=610, step_s=20)
    openamber.step(ms=100)

    # Compressor should STILL be running because overshoot 1.5 < stop delta 2.0
    comp_mode = int(openamber.get_entity("compressor_control_select") or 0)
    assert comp_mode > 0, "Compressor should continue running when overshoot < stop delta"

    # Water cools further to 15.8°C (overshoot = 2.2°C >= 2.0°C stop delta)
    openamber.set_sensor("current_water_temperature_tc_sensor", 15.8)
    openamber.set_sensor("heat_cool_temperature_tc", 15.8)
    openamber.set_sensor("outlet_temperature_tuo", 15.8)
    openamber.step(ms=50)

    openamber.advance_time(seconds=20, step_s=5)
    openamber.step(ms=100)

    # Compressor must stop
    comp_mode = int(openamber.get_entity("compressor_control_select") or 0)
    assert comp_mode == 0, f"Compressor should stop when overshoot >= stop delta, got mode {comp_mode}"

    # Cleanup
    openamber.set_binary_sensor("external_cool_demand_wired", False)
    openamber.step(ms=100)
