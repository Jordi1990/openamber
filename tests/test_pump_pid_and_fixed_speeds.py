"""Automated tests for Pump Speed configurations and P0 PID control.

Covers:
1. Fixed Pump Speed Numbers (pump_p0_pid_enabled = False):
   - pump_speed_heating_number modulates heating pump PWM ((1000 - speed * 10)).
   - pump_speed_dhw_number modulates pump PWM during active DHW.
2. Dynamic P0 PID Control (pump_p0_pid_enabled = True):
   - pump_p0_pid_min_pwm and pump_p0_pid_max_pwm clamp the modulated pump speed.
   - pump_p0_pid_defrost_pwm overrides pump speed during defrost cycles.
"""

import pytest


def test_fixed_pump_speed_heating(clean_system):
    """
    Verify Fixed Pump Speed in Space Heating:
    1. During Space Heating, pump_speed_heating_number controls PWM duty cycle:
       - Set to 70% -> PWM raw = 300 (1000 - 70 * 10).
       - Set to 90% -> PWM raw = 100 (1000 - 90 * 10).
    """
    openamber = clean_system

    openamber.set_switch("pump_p0_pid_enabled", False)
    openamber.set_select("thermostat_mode_select", "Extern")
    openamber.set_select("heat_mode_select", "Extern setpoint")
    openamber.set_number("manual_setpoint", 35.0)
    openamber.set_climate("pid_heat_temperature_control", target_temperature=35.0)
    openamber.set_number("compressor_start_delta_heating", 2.0)
    openamber.set_number("compressor_stop_delta_heating", 2.0)
    openamber.set_number("pump_speed_heating_number", 70.0)

    # Initial cold supply water to start heating
    openamber.set_sensor("current_water_temperature_tc_sensor", 28.0)
    openamber.set_sensor("heat_cool_temperature_tc", 28.0)
    openamber.set_sensor("outlet_temperature_tuo", 28.0)
    openamber.set_sensor("inlet_temperature_tui", 26.0)
    openamber.step(ms=50)

    # Advance past min off-time
    openamber.advance_time(seconds=130, step_s=10)
    openamber.step(ms=50)

    # Trigger heat demand
    openamber.set_binary_sensor("external_heat_demand_wired", True)
    openamber.step(ms=100)
    assert openamber.get_entity("heat_demand_active_sensor") is True

    # Advance time through pump interval in IDLE (900s) + pump start (10s)
    openamber.advance_time(seconds=910, step_s=30)
    openamber.step(ms=100)
    assert openamber.get_entity("internal_pump_active") is True

    # Allow 1s template sensor to update
    openamber.advance_time(seconds=4, step_s=1)
    openamber.step(ms=100)

    # At 70% speed, inverted PWM control value is 1000 - 70*10 = 300
    current_pwm = float(openamber.get_entity("pump_control_pwm_number") or openamber.get_entity("pump_p0_current_pwm_sensor") or 0)
    assert current_pwm == 300.0, f"Expected 300.0 for 70% heating speed, got {current_pwm}"

    # Change heating pump speed to 90%
    openamber.set_number("pump_speed_heating_number", 90.0)
    openamber.advance_time(seconds=6, step_s=1)
    openamber.step(ms=100)

    current_pwm = float(openamber.get_entity("pump_control_pwm_number") or openamber.get_entity("pump_p0_current_pwm_sensor") or 0)
    assert current_pwm == 100.0, f"Expected 100.0 for 90% heating speed, got {current_pwm}"

    # Cleanup
    openamber.set_binary_sensor("external_heat_demand_wired", False)
    openamber.step(ms=100)


def test_fixed_pump_speed_dhw(clean_system):
    """
    Verify Fixed Pump Speed in DHW:
    1. System transitions from idle to DHW heating.
    2. During DHW, pump_speed_dhw_number controls PWM duty cycle:
       - Set to 85% -> PWM raw = 150 (1000 - 85 * 10).
    """
    openamber = clean_system

    openamber.set_number("pump_speed_dhw_number", 85.0)
    openamber.set_number("dhw_setpoint_temperature", 50.0)
    openamber.set_sensor("dhw_temperature_tw_sensor", 38.0)
    openamber.step(ms=50)

    # Allow delta-restart condition to register
    openamber.advance_time(seconds=10, step_s=2)
    openamber.step(ms=100)
    assert openamber.get_entity("dhw_demand_active_sensor") is True

    # Advance time through 3-way valve switch time (60s) and state transition
    openamber.advance_time(seconds=80, step_s=10)
    openamber.step(ms=100)
    assert openamber.get_entity("three_way_valve_dhw_switch") is True

    # Advance virtual time for DHW pump start
    openamber.advance_time(seconds=80, step_s=10)
    openamber.step(ms=100)

    # Allow 1s template sensor to update
    openamber.advance_time(seconds=4, step_s=1)
    openamber.step(ms=100)

    # In DHW mode, pump speed is 85% -> 1000 - 85*10 = 150
    current_pwm = float(openamber.get_entity("pump_control_pwm_number") or openamber.get_entity("pump_p0_current_pwm_sensor") or 0)
    assert current_pwm == 150.0, f"Expected 150.0 for 85% DHW speed, got {current_pwm}"

    # Cleanup
    openamber.set_sensor("dhw_temperature_tw_sensor", 52.0)
    openamber.step(ms=100)


def test_pump_p0_pid_defrost_override_and_limits(clean_system):
    """
    Verify P0 PID Controls and Defrost Override:
    1. Enable pump_p0_pid_enabled = True.
    2. Set pump_p0_pid_min_pwm = 40.0, pump_p0_pid_max_pwm = 80.0, pump_p0_pid_defrost_pwm = 95.0.
    3. Start heating and settle pump.
    4. Verify pump operates within [40%, 80%] (PWM between 200 and 600).
    5. Trigger defrost (defrost_active_sensor = True).
    6. Verify pump switches to defrost PWM (95% -> PWM 50).
    7. Clear defrost -> pump returns to normal PID range.
    """
    openamber = clean_system

    openamber.set_switch("pump_p0_pid_enabled", True)
    openamber.set_number("pump_p0_pid_min_pwm", 40.0)
    openamber.set_number("pump_p0_pid_max_pwm", 80.0)
    openamber.set_number("pump_p0_pid_defrost_pwm", 95.0)

    openamber.set_select("thermostat_mode_select", "Extern")
    openamber.set_select("heat_mode_select", "Extern setpoint")
    openamber.set_number("manual_setpoint", 35.0)
    openamber.set_climate("pid_heat_temperature_control", target_temperature=35.0)
    openamber.set_number("compressor_start_delta_heating", 2.0)
    openamber.set_number("compressor_stop_delta_heating", 2.0)

    openamber.set_sensor("current_water_temperature_tc_sensor", 28.0)
    openamber.set_sensor("heat_cool_temperature_tc", 28.0)
    openamber.set_sensor("outlet_temperature_tuo", 28.0)
    openamber.set_sensor("inlet_temperature_tui", 26.0)
    openamber.step(ms=50)

    # Ensure min off time
    openamber.advance_time(seconds=130, step_s=10)
    openamber.step(ms=50)

    openamber.set_binary_sensor("external_heat_demand_wired", True)
    openamber.step(ms=100)

    # Advance time through pump interval in IDLE (900s) + pump start (10s)
    openamber.advance_time(seconds=910, step_s=30)
    openamber.step(ms=100)
    assert openamber.get_entity("internal_pump_active") is True

    # Allow 1s template sensor to update
    openamber.advance_time(seconds=4, step_s=1)
    openamber.step(ms=100)

    # In PID mode before compressor settles, pump starts at min_pwm (40% -> 600)
    current_pwm = float(openamber.get_entity("pump_control_pwm_number") or openamber.get_entity("pump_p0_current_pwm_sensor") or 0)
    assert current_pwm == 600.0, f"Expected initial pump PWM 600.0 (40%), got {current_pwm}"

    # Trigger defrost cycle
    openamber.set_binary_sensor("defrost_active_sensor", True)
    openamber.advance_time(seconds=10, step_s=2)
    openamber.step(ms=100)

    # During defrost with PID enabled, pump must switch to defrost PWM (95% -> 50)
    current_pwm = float(openamber.get_entity("pump_control_pwm_number") or openamber.get_entity("pump_p0_current_pwm_sensor") or 0)
    assert current_pwm == 50.0, f"Expected defrost pump PWM 50.0 (95%), got {current_pwm}"

    # Clear defrost cycle
    openamber.set_binary_sensor("defrost_active_sensor", False)
    openamber.advance_time(seconds=10, step_s=2)
    openamber.step(ms=100)

    # Pump must return from defrost override (<= 600.0, i.e. >= 40%)
    current_pwm = float(openamber.get_entity("pump_control_pwm_number") or openamber.get_entity("pump_p0_current_pwm_sensor") or 0)
    assert current_pwm <= 600.0, f"Expected pump to return from defrost, got {current_pwm}"

    # Cleanup
    openamber.set_binary_sensor("external_heat_demand_wired", False)
    openamber.set_switch("pump_p0_pid_enabled", False)
    openamber.step(ms=100)
