import pytest


import pytest


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
    openamber.advance_time(seconds=10, step_s=2)
    openamber.step(ms=100)

    # -------------------------------------------------------------------------
    # Step 4: Verify controller shut off the heater and UI reflects it
    # -------------------------------------------------------------------------
    assert openamber.get_entity("backup_heater_relay") is False, (
        "Backup heater relay should turn OFF automatically when temperature exceeds setpoint + stop delta"
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
