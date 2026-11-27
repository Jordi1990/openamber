"""Frost protection keeps pump P0 circulating, except while stopped (standby) or in maintenance mode.

Also with an active error (which blocks heating, cooling and DHW) and when the flow switch reports no flow.
The compressor and the backup heater stay off while an error is active, also on frost protection stage 2.
"""

from helpers import advance_until_state, stop_heat_pump


def _pump_commanded(openamber):
    """Pump P0 commanded on: relay on and a PWM speed above 0 % (the control value 1000 is 0 %)."""
    pwm = openamber.get_entity("pump_control_pwm_number")
    return openamber.get_entity("pump_p0_relay_switch") is True and pwm is not None and float(pwm) < 995.0


def _enable_frost_protection_stage1(openamber):
    openamber.set_binary_sensor("external_heat_demand_wired", False)
    openamber.set_switch("heat_demand_switch", False)
    openamber.set_number("frost_protection_stage_1_temp_ta", 5.0)
    openamber.set_number("frost_protection_stage_2_temp_ta", -20.0)
    openamber.set_number("pump_interval", 15.0)
    openamber.set_number("pump_duration", 2.0)
    openamber.set_sensor("inlet_temperature_tui", 20.0)
    openamber.set_sensor("temperature_outside_ta", 2.0)
    # 300 s delayed_on filter of the frost protection sensors.
    openamber.advance_time(seconds=305, step_s=5)
    openamber.step(ms=100)
    assert openamber.get_entity("frost_protection_stage1_active") is True


def _cleanup(openamber):
    openamber.set_switch("mock_pump_flow_missing", False)
    openamber.set_switch("reset_start_timeout_errors_switch", True)
    openamber.set_sensor("temperature_outside_ta", 7.0)
    openamber.set_sensor("inlet_temperature_tui", 25.0)
    openamber.advance_time(seconds=310, step_s=15)
    openamber.step(ms=100)


def test_frost_protection_runs_pump_cycles_with_active_error(clean_system):
    """
    1. An error is active (TO01, pump start timeout): heat demand is suppressed.
    2. Frost protection stage 1 activates: pump P0 starts and runs the full pump duration, instead of being
       stopped by the error safety check right after its start.
    3. The pump stops after the pump duration and starts again on the next interval cycle.
    4. The compressor stays off throughout.
    """
    openamber = clean_system
    assert openamber.set_binary_sensor("error_pump_start_timeout", True) is True
    openamber.step(ms=100)
    assert openamber.get_entity("error_active") is True

    _enable_frost_protection_stage1(openamber)
    try:
        assert openamber.get_entity("internal_pump_active") is True, "P0 must start on frost protection with an error"

        # Well into the 2 min pump duration the pump still runs: the error must not stop it.
        openamber.advance_time(seconds=60, step_s=5)
        openamber.step(ms=100)
        assert openamber.get_entity("internal_pump_active") is True, "The error must not stop the frost protection cycle"
        assert openamber.get_entity("state_machine_state_heat_cool") == "Pump running"
        assert int(openamber.get_entity("compressor_control_select") or 0) == 0

        # Pump duration ends: P0 stops and waits for the next interval.
        openamber.advance_time(seconds=90, step_s=5)
        openamber.step(ms=100)
        assert openamber.get_entity("internal_pump_active") is False, "P0 must stop after the pump duration"
        assert openamber.get_entity("state_machine_state_heat_cool") == "Idle"

        # Next interval cycle (15 min after the stop) starts P0 again.
        assert advance_until_state(openamber, "internal_pump_active", True, max_s=960, step=5), (
            "P0 must start again on the next frost protection interval cycle"
        )
        assert int(openamber.get_entity("compressor_control_select") or 0) == 0
        assert openamber.get_entity("error_active") is True
    finally:
        _cleanup(openamber)


def test_frost_protection_stage2_with_error_runs_pump_only(clean_system):
    """
    Frost protection stage 2 normally starts the compressor. With an active error it only runs pump P0:
    the compressor and the backup heater stay off.
    """
    openamber = clean_system
    assert openamber.set_binary_sensor("error_pump_start_timeout", True) is True
    openamber.set_binary_sensor("external_heat_demand_wired", False)
    openamber.set_switch("heat_demand_switch", False)
    openamber.set_number("frost_protection_stage_2_temp_ta", 4.0)
    openamber.set_number("frost_protection_stage_2_temp_tui", 7.0)
    openamber.set_number("pump_duration", 10.0)
    openamber.set_sensor("temperature_outside_ta", 2.0)
    openamber.set_sensor("inlet_temperature_tui", 5.0)
    openamber.set_sensor("current_water_temperature_tc_sensor", 12.0)
    openamber.set_sensor("heat_cool_temperature_tc", 12.0)
    openamber.set_sensor("outlet_temperature_tuo", 12.0)
    openamber.advance_time(seconds=305, step_s=5)
    openamber.step(ms=100)
    try:
        assert openamber.get_entity("frost_protection_stage2_active") is True
        assert openamber.get_entity("internal_pump_active") is True, "P0 must start on frost protection with an error"

        # Past the pump settle time and soft start: without the error the compressor would run by now.
        openamber.advance_time(seconds=330, step_s=10)
        openamber.step(ms=100)
        assert openamber.get_entity("internal_pump_active") is True
        assert openamber.get_entity("state_machine_state_heat_cool") == "Pump running"
        assert int(openamber.get_entity("compressor_control_select") or 0) == 0, "No compressor with an active error"
        assert openamber.get_entity("backup_heater_relay") is not True, "No backup heater with an active error"
    finally:
        openamber.set_number("pump_duration", 2.0)
        openamber.set_sensor("current_water_temperature_tc_sensor", 28.0)
        openamber.set_sensor("heat_cool_temperature_tc", 28.0)
        openamber.set_sensor("outlet_temperature_tuo", 28.0)
        _cleanup(openamber)


def test_frost_protection_keeps_pump_on_without_flow(clean_system):
    """
    1. The flow switch reports no flow (1212 stays off) while frost protection starts pump P0.
    2. After the pump start timeout TO01 is raised, but the pump stays commanded on for the pump duration.
    3. The pump is commanded off after the duration and on again on the next interval cycle.
    4. The compressor stays off.
    """
    openamber = clean_system
    openamber.set_switch("mock_pump_flow_missing", True)
    openamber.step(ms=100)

    _enable_frost_protection_stage1(openamber)
    # The pump duration counts from the start, so it must be longer than the 2 min start timeout.
    openamber.set_number("pump_duration", 5.0)
    try:
        assert openamber.get_entity("internal_pump_active") is False, "The flow switch reports no flow"
        assert _pump_commanded(openamber), "P0 must be commanded on for frost protection"

        # Past the 2 min pump start timeout.
        openamber.advance_time(seconds=125, step_s=5)
        openamber.step(ms=100)
        assert openamber.get_entity("error_pump_start_timeout") is True, "Missing flow is still reported (TO01)"
        assert _pump_commanded(openamber), "P0 must stay on for frost protection without flow"
        assert openamber.get_entity("state_machine_state_heat_cool") == "Pump running"
        assert int(openamber.get_entity("compressor_control_select") or 0) == 0

        # The pump duration (5 min, counted from the start) ends: P0 commanded off.
        assert advance_until_state(openamber, "state_machine_state_heat_cool", "Idle", max_s=240, step=5)
        assert not _pump_commanded(openamber), "P0 must be commanded off after the pump duration"

        # Next interval cycle commands P0 on again, still without flow.
        for _ in range(200):
            if _pump_commanded(openamber):
                break
            openamber.advance_time(seconds=5, step_s=5)
            openamber.step(ms=20)
        assert _pump_commanded(openamber), "P0 must be commanded on again on the next interval cycle"
        assert int(openamber.get_entity("compressor_control_select") or 0) == 0
    finally:
        openamber.set_number("pump_duration", 2.0)
        _cleanup(openamber)


def test_no_heat_source_without_flow_after_error_reset(clean_system):
    """
    Frost protection stage 2 keeps the pump on without flow after the pump start timeout. When TO01 is reset
    during that cycle while the flow is still missing, the compressor must not start.
    """
    openamber = clean_system
    openamber.set_switch("mock_pump_flow_missing", True)
    openamber.set_binary_sensor("external_heat_demand_wired", False)
    openamber.set_switch("heat_demand_switch", False)
    openamber.set_number("frost_protection_stage_2_temp_ta", 4.0)
    openamber.set_number("frost_protection_stage_2_temp_tui", 7.0)
    openamber.set_number("pump_duration", 10.0)
    openamber.set_sensor("temperature_outside_ta", 2.0)
    openamber.set_sensor("inlet_temperature_tui", 5.0)
    openamber.set_sensor("current_water_temperature_tc_sensor", 12.0)
    openamber.set_sensor("heat_cool_temperature_tc", 12.0)
    openamber.set_sensor("outlet_temperature_tuo", 12.0)
    openamber.advance_time(seconds=305, step_s=5)
    openamber.step(ms=100)
    try:
        assert openamber.get_entity("frost_protection_stage2_active") is True
        assert _pump_commanded(openamber)

        # Past the pump start timeout: TO01, the pump stays on.
        openamber.advance_time(seconds=125, step_s=5)
        openamber.step(ms=100)
        assert openamber.get_entity("error_pump_start_timeout") is True
        assert openamber.get_entity("state_machine_state_heat_cool") == "Pump running"

        # Reset TO01 while the flow is still missing: no compressor and no backup heater without flow.
        openamber.set_switch("reset_start_timeout_errors_switch", True)
        openamber.step(ms=100)
        assert openamber.get_entity("error_active") is False
        openamber.advance_time(seconds=240, step_s=10)
        openamber.step(ms=100)
        assert int(openamber.get_entity("compressor_control_select") or 0) == 0, "No compressor start without flow"
        assert openamber.get_entity("backup_heater_relay") is not True, "No backup heater without flow"
        assert _pump_commanded(openamber), "The pump stays on for frost protection"
    finally:
        openamber.set_number("pump_duration", 2.0)
        openamber.set_sensor("current_water_temperature_tc_sensor", 28.0)
        openamber.set_sensor("heat_cool_temperature_tc", 28.0)
        openamber.set_sensor("outlet_temperature_tuo", 28.0)
        _cleanup(openamber)


def test_no_frost_protection_while_stopped(clean_system):
    """A controlled stop (standby) still ends frost protection: pump P0 does not run."""
    openamber = clean_system
    stop_heat_pump(openamber)
    try:
        _enable_frost_protection_stage1(openamber)
        openamber.advance_time(seconds=60, step_s=5)
        openamber.step(ms=100)
        assert openamber.get_entity("state_machine_state_main") == "Standby"
        assert openamber.get_entity("internal_pump_active") is False, "No frost protection while stopped"
    finally:
        openamber.press_button("controlled_stop_resume_button")
        openamber.step(ms=50)
        _cleanup(openamber)
