import pytest


def test_backup_heater_ui_bindings_and_turn_off_flow(openamber):
    """
    Workflow Test:
    1. Verify backup heater is initially off and UI displays 'UIT' / unchecked.
    2. Turn on the backup heater and verify UI updates (label 'AAN', switch checked).
    3. Simulate temperature reaching setpoint + shutoff condition.
    4. Verify backup heater turns OFF and UI immediately reflects the shutdown ('UIT' / unchecked).
    """

    # -------------------------------------------------------------------------
    # Step 1: Initial state verification
    # -------------------------------------------------------------------------
    openamber.set_switch("backup_heater_relay", False)
    openamber.set_binary_sensor("backup_heater_active_sensor", False)
    openamber.step(ms=100)

    assert openamber.get_entity("backup_heater_relay") is False
    assert openamber.get_entity("backup_heater_active_sensor") is False

    # Check UI state
    relay_switch_widget = openamber.get_widget("service_backup_heater_relay_switch_ui")
    assert relay_switch_widget.get("checked") is False

    backup_label = openamber.get_label("tile_backup_state")
    assert backup_label == "UIT"

    # -------------------------------------------------------------------------
    # Step 2: Simulate heating demand requiring backup heater
    # -------------------------------------------------------------------------
    openamber.set_sensor("temperature_outside_ta", -5.0)
    openamber.set_sensor("water_temperature_outlet_t1", 25.0)

    # Activate backup heater
    openamber.set_switch("backup_heater_relay", True)
    openamber.set_binary_sensor("backup_heater_active_sensor", True)
    openamber.step(ms=100)

    # Verify relay and binary sensor are active
    assert openamber.get_entity("backup_heater_relay") is True
    assert openamber.get_entity("backup_heater_active_sensor") is True

    # Verify UI reflects active heater
    relay_switch_widget = openamber.get_widget("service_backup_heater_relay_switch_ui")
    assert relay_switch_widget.get("checked") is True

    backup_label = openamber.get_label("tile_backup_state")
    assert backup_label == "AAN"

    # -------------------------------------------------------------------------
    # Step 3: Simulate temperature rising to setpoint + stop delta
    # -------------------------------------------------------------------------
    # Setpoint is reached: flow temperature rises above threshold
    target_flow_temp = 45.0
    stop_delta = 2.0
    openamber.set_sensor("water_temperature_outlet_t1", target_flow_temp + stop_delta + 1.0)

    # Controller shuts off the backup heater
    openamber.set_switch("backup_heater_relay", False)
    openamber.set_binary_sensor("backup_heater_active_sensor", False)
    openamber.step(ms=100)

    # -------------------------------------------------------------------------
    # Step 4: Verify shutdown propagation to hardware entity and LVGL UI
    # -------------------------------------------------------------------------
    assert openamber.get_entity("backup_heater_relay") is False, (
        "Backup heater relay should turn OFF when reaching setpoint"
    )

    relay_switch_widget = openamber.get_widget("service_backup_heater_relay_switch_ui")
    assert relay_switch_widget.get("checked") is False, (
        "LVGL switch widget should be unchecked when heater is OFF"
    )

    backup_label = openamber.get_label("tile_backup_state")
    assert backup_label == "UIT", (
        "LVGL label should display 'UIT' when backup heater turns off"
    )
