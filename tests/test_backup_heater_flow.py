import pytest

from helpers import advance_until_state

MIN_ON_S = 600


def test_backup_heater_ui_bindings_and_turn_off_flow(clean_system):
    """
    Workflow Test:
    1. Verify backup heater is initially off and UI displays 'UIT' / unchecked.
    2. Turn on backup heater via space heating emergency mode demand, verify UI updates (label 'AAN', switch checked).
    3. Simulate temperature reaching setpoint + shutoff condition.
    4. Verify controller shuts off backup heater and UI immediately reflects shutdown ('UIT' / unchecked).
    """
    openamber = clean_system

    # -------------------------------------------------------------------------
    # Step 1: Initial state verification
    # -------------------------------------------------------------------------
    assert openamber.get_entity("backup_heater_relay") is False, "Backup heater relay should initially be OFF"
    assert openamber.get_entity("backup_heater_active_sensor") is False, "Backup heater active sensor should initially be False"

    relay_switch_widget = openamber.get_widget("service_backup_heater_relay_switch_ui")
    assert relay_switch_widget.get("checked") is False, "LVGL switch widget should initially be unchecked"

    backup_label = openamber.get_label("tile_backup_state")
    assert backup_label == "UIT", "LVGL label tile_backup_state should initially display 'UIT'"

    # -------------------------------------------------------------------------
    # Step 2: Trigger heating demand requiring backup heater (emergency mode)
    # -------------------------------------------------------------------------
    assert openamber.set_switch("emergency_mode_enabled", True)
    assert openamber.set_select("thermostat_mode_select", "Extern")
    assert openamber.set_select("heat_mode_select", "Extern setpoint")
    target_temp = 35.0
    stop_delta = 2.0
    assert openamber.set_number("manual_setpoint", target_temp)
    assert openamber.set_number("compressor_start_delta_heating", 2.0)
    assert openamber.set_number("compressor_stop_delta_heating", stop_delta)
    assert openamber.set_number("pump_interval", 15.0)
    assert openamber.set_number("pump_duration", 2.0)
    assert openamber.set_sensor("current_water_temperature_tc_sensor", 25.0)
    assert openamber.set_sensor("heat_cool_temperature_tc", 25.0)
    assert openamber.set_sensor("outlet_temperature_tuo", 25.0)
    assert openamber.set_sensor("inlet_temperature_tui", 23.0)
    assert openamber.set_sensor("temperature_outside_ta", -5.0)
    openamber.step(ms=50)

    # Ensure minimum compressor off-time has elapsed
    openamber.advance_time(seconds=130, step_s=10)
    openamber.step(ms=50)

    # Activate heat demand
    assert openamber.set_binary_sensor("external_heat_demand_wired", True)
    openamber.step(ms=100)
    assert openamber.get_entity("heat_demand_active_sensor") is True, "Heat demand should become active"

    # Advance time through pump interval in IDLE (900s) + pump settle (140s)
    openamber.advance_time(seconds=900, step_s=30)
    openamber.step(ms=100)
    openamber.advance_time(seconds=140, step_s=10)
    openamber.step(ms=100)

    # Controller engages backup heater
    assert openamber.get_entity("backup_heater_relay") is True, "Backup heater relay should turn ON via controller"
    assert openamber.get_entity("backup_heater_active_sensor") is True, "Backup heater active sensor should become True"

    # Verify UI reflects active heater
    relay_switch_widget = openamber.get_widget("service_backup_heater_relay_switch_ui")
    assert relay_switch_widget.get("checked") is True, "LVGL switch widget should be checked when heater is ON"

    backup_label = openamber.get_label("tile_backup_state")
    assert backup_label == "AAN", "LVGL label should display 'AAN' when backup heater is active"

    # -------------------------------------------------------------------------
    # Step 3: Simulate temperature rising to setpoint + stop delta
    # -------------------------------------------------------------------------
    # Water temperature rises above target (35) + stop_delta (2) = 37 -> 38°C
    assert openamber.set_sensor("current_water_temperature_tc_sensor", target_temp + stop_delta + 1.0)
    assert openamber.set_sensor("heat_cool_temperature_tc", target_temp + stop_delta + 1.0)
    assert openamber.set_sensor("outlet_temperature_tuo", target_temp + stop_delta + 1.0)
    assert openamber.set_sensor("inlet_temperature_tui", target_temp + stop_delta - 1.0)
    openamber.advance_time(seconds=10, step_s=2)
    openamber.step(ms=100)

    # -------------------------------------------------------------------------
    # Step 4: Verify controller shut off the heater and UI reflects it
    # -------------------------------------------------------------------------
    assert openamber.get_entity("backup_heater_relay") is False, (
        "Backup heater relay should turn OFF automatically when temperature exceeds setpoint + stop delta"
    )
    assert openamber.get_entity("backup_heater_active_sensor") is False, (
        "Backup heater active sensor should return to False when heater turns off"
    )

    relay_switch_widget = openamber.get_widget("service_backup_heater_relay_switch_ui")
    assert relay_switch_widget.get("checked") is False, (
        "LVGL switch widget should be unchecked when heater is shut OFF by controller"
    )

    backup_label = openamber.get_label("tile_backup_state")
    assert backup_label == "UIT", (
        "LVGL label should display 'UIT' when backup heater turns off"
    )

    # Cleanup
    assert openamber.set_binary_sensor("external_heat_demand_wired", False)
    assert openamber.set_switch("emergency_mode_enabled", False)
    openamber.step(ms=100)


def _compressor_mode(openamber):
    return int(openamber.get_entity("compressor_control_select") or 0)


def _simulator_s(openamber):
    # millis() of the simulator: the virtual time offset plus the real time that passes during the test.
    return openamber.get_time()["millis"] / 1000.0


def _start_heating_with_sg_max_boost_backup(openamber):
    """Heating that switches on the backup heater right after the soft start (SG Ready max boost), so the
    compressor has run for about 3 minutes, well inside its minimum on time (600 s).

    Returns the simulator time at which the compressor was seen running.
    """
    openamber.set_select("thermostat_mode_select", "Extern")
    openamber.set_select("heat_mode_select", "Extern setpoint")
    openamber.set_number("manual_setpoint", 35.0)
    openamber.set_climate("pid_heat_temperature_control", target_temperature=35.0)
    openamber.set_number("compressor_start_delta_heating", 2.0)
    openamber.set_number("backup_heater_degmin_threshold", 9999.0)
    openamber.set_sensor("heat_cool_temperature_tc", 28.0)
    openamber.set_sensor("outlet_temperature_tuo", 28.0)
    openamber.set_sensor("inlet_temperature_tui", 26.0)
    openamber.set_switch("heat_demand_switch", True)
    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Compressor softstart", max_s=1800, step=5)
    started_at = _simulator_s(openamber)

    openamber.set_select("sg_ready_mode_select", "Max boost")
    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Backup heater running", max_s=400, step=5)
    assert openamber.get_entity("backup_heater_relay") is True
    assert _compressor_mode(openamber) > 0
    return started_at


def _run_until_compressor_stops(openamber, max_s=700):
    """Advance until the compressor request is off; the backup heater must stay off all the time."""
    waited = 0
    while _compressor_mode(openamber) > 0 and waited < max_s:
        openamber.advance_time(seconds=5, step_s=5)
        openamber.step(ms=20)
        waited += 5
        assert openamber.get_entity("backup_heater_relay") is False, (
            "The backup heater must stay off while the compressor only finishes its minimum on time"
        )
    assert _compressor_mode(openamber) == 0, "The compressor stops once the minimum on time has passed"


def test_backup_heater_state_keeps_minimum_on_time_when_heat_demand_ends(clean_system):
    """
    Heat demand ends while the backup heater helps the compressor, 3 minutes after the compressor start:
    1. The backup heater goes off at once.
    2. The compressor finishes its minimum on time (it used to stop at once).
    3. The backup heater stays off meanwhile, also with SG Ready max boost still active.
    """
    openamber = clean_system
    try:
        started_at = _start_heating_with_sg_max_boost_backup(openamber)

        openamber.set_switch("heat_demand_switch", False)
        openamber.advance_time(seconds=10, step_s=5)
        openamber.step(ms=50)
        assert openamber.get_entity("backup_heater_relay") is False, "No backup heater without heat demand"
        assert _compressor_mode(openamber) > 0, "The compressor must finish its minimum on time"

        _run_until_compressor_stops(openamber)
        ran = _simulator_s(openamber) - started_at
        assert ran >= MIN_ON_S, f"Compressor stopped after {ran:.0f} s"
        assert advance_until_state(openamber, "state_machine_state_heat_cool", "Idle", max_s=300)
    finally:
        openamber.set_select("sg_ready_mode_select", "Normaal")


def test_dhw_request_during_backup_heater_state_keeps_minimum_on_time(clean_system):
    """A DHW request while the backup heater helps the compressor: backup heater off, minimum on time kept, then DHW."""
    openamber = clean_system
    try:
        started_at = _start_heating_with_sg_max_boost_backup(openamber)

        openamber.set_sensor("dhw_temperature_tw_sensor", 38.0)
        openamber.advance_time(seconds=10, step_s=5)
        openamber.step(ms=50)
        assert openamber.get_entity("backup_heater_relay") is False
        assert _compressor_mode(openamber) > 0, "The compressor must finish its minimum on time"
        assert openamber.get_entity("state_machine_state_main") == "Heat/Cool"

        _run_until_compressor_stops(openamber)
        ran = _simulator_s(openamber) - started_at
        assert ran >= MIN_ON_S, f"Compressor stopped after {ran:.0f} s"
        assert advance_until_state(openamber, "state_machine_state_main", "DHW", max_s=300)
        assert openamber.get_entity("three_way_valve_dhw_switch") is True
    finally:
        openamber.set_select("sg_ready_mode_select", "Normaal")
