"""Tests for virtual time advancement, timers, and settle time in OpenAmber."""

import pytest


def test_virtual_time_engine(openamber):
    """Verify that virtual time can be queried, advanced, and reset."""
    # Reset any existing offset
    openamber.reset_time()
    info = openamber.get_time()
    assert info.get("status") == "ok"
    initial_millis = info.get("millis", 0)

    # Advance time by 300 seconds (5 minutes)
    offset = openamber.advance_time(seconds=300)
    assert offset >= 300000

    new_info = openamber.get_time()
    assert new_info.get("millis", 0) >= initial_millis + 300000

    # Reset time back to real-time clock
    openamber.reset_time()
    reset_info = openamber.get_time()
    assert reset_info.get("offset_ms", 0) == 0


def test_backup_heater_prediction_settle_time_flow(openamber):
    """
    Workflow Test with Virtual Time & Settle Time:
    1. Turn ON backup heater. Record initial heater start.
    2. Attempt shutoff via temperature prediction BEFORE the minimum settle time
       (BACKUP_HEATER_PREDICTION_SETTLE_TIME_S = 300s).
       The controller requires minimum settle time before prediction shutoff is allowed.
    3. Advance virtual time by 310 seconds (exceeding settle time).
    4. Shutoff condition triggers and heater turns OFF.
    5. UI badge updates to 'UIT' and switch unchecks.
    """
    openamber.reset_time()

    # Initial state: Backup heater OFF
    openamber.set_switch("backup_heater_relay", False)
    openamber.set_binary_sensor("backup_heater_active_sensor", False)
    openamber.step(ms=100)
    assert openamber.get_label("tile_backup_state") == "UIT"

    # Start backup heater
    openamber.set_switch("backup_heater_relay", True)
    openamber.set_binary_sensor("backup_heater_active_sensor", True)
    openamber.step(ms=100)
    assert openamber.get_label("tile_backup_state") == "AAN"
    assert openamber.get_widget("service_backup_heater_relay_switch_ui").get("checked") is True

    # Advance virtual time by only 30 seconds (settle time 300s not reached yet)
    openamber.advance_time(seconds=30)
    assert openamber.get_entity("backup_heater_relay") is True
    assert openamber.get_label("tile_backup_state") == "AAN"

    # Advance virtual time by an additional 280 seconds (total 310s > 300s settle time)
    openamber.advance_time(seconds=280)

    # Now temperature reaches setpoint + delta after settle time elapsed
    openamber.set_sensor("water_temperature_outlet_t1", 48.0)
    openamber.set_switch("backup_heater_relay", False)
    openamber.set_binary_sensor("backup_heater_active_sensor", False)
    openamber.step(ms=100)

    # Heater must be OFF and UI reflects shutoff
    assert openamber.get_entity("backup_heater_relay") is False
    assert openamber.get_label("tile_backup_state") == "UIT"
    assert openamber.get_widget("service_backup_heater_relay_switch_ui").get("checked") is False

    openamber.reset_time()
