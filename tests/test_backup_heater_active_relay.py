"""Automated tests for IsBackupHeaterActive() with external backup heating.

The function used to return true whenever external backup heating was selected, even with the
external relay off. It now reports the state of external_backup_heating_relay, which arms the
safety checks and the transition to BACKUP_HEATER_RUNNING in both controllers:
- heat/cool: Tuo-Tui difference above 15 K (heating) stops the running unit (PerformSafetyChecks),
- DHW: Tuo-Tui difference above 15 K stops the running unit,
- WAIT_BACKUP_HEATER_RUNNING only continues once the backup heater reports as active.
"""

from helpers import advance_until_state


def hold_without_element(openamber, state_entity, seconds=75):
    """Hold the current Tuo-Tui difference past the 60 s safety delay while the element stays off.

    Stops early when the element starts, because from then on a safety stop is legitimate. Until then the
    state may not drop to Idle and TO08 may not be raised.
    """
    for _ in range(seconds // 5):
        openamber.advance_time(seconds=5, step_s=5)
        openamber.step(ms=50)
        if openamber.get_entity("external_backup_heating_relay") is True:
            break
        assert openamber.get_entity(state_entity) != "Idle", (
            "With the external relay off, a Tuo-Tui difference must not stop the system"
        )
        assert openamber.get_entity("error_water_delta_t_too_high") is False


def test_dhw_emergency_external_backup_runs_on_external_relay(clean_system):
    """DHW emergency heating with 'Externe backup verwarming' runs on the external relay."""
    openamber = clean_system

    assert openamber.set_switch("emergency_mode_enabled", True)
    assert openamber.set_select("backup_heating_mode", "Externe backup verwarming")
    assert openamber.set_sensor("dhw_temperature_tw_sensor", 38.0)
    openamber.step(ms=50)

    # Valve switch time (60 s) + settle (60 s), then the element starts through WAIT_BACKUP_HEATER_RUNNING,
    # which now needs the external relay state to confirm.
    assert advance_until_state(openamber, "state_machine_state_dhw", "Backup heater running", max_s=300, step=5), (
        f"DHW never started the external backup heater (dhw: {openamber.get_entity('state_machine_state_dhw')})"
    )
    assert openamber.get_entity("external_backup_heating_relay") is True, "External relay must drive DHW backup heating"
    assert openamber.get_entity("backup_heater_relay") is False, "Internal element must stay off in external mode"

    # Tank reaches its setpoint: the element stops and the relay state follows. The emergency pump-start
    # branch runs first and adds a 120 s settle before the stop check evaluates, so wait for the cycle
    # to end instead of a fixed time.
    assert openamber.set_sensor("dhw_temperature_tw_sensor", 51.0)
    assert advance_until_state(openamber, "state_machine_state_dhw", "Idle", max_s=420, step=10), (
        f"DHW cycle never ended (dhw: {openamber.get_entity('state_machine_state_dhw')})"
    )
    assert openamber.get_entity("external_backup_heating_relay") is False, "External relay must turn off when DHW is satisfied"

    assert openamber.set_switch("emergency_mode_enabled", False)
    openamber.step(ms=100)


def test_heat_cool_delta_t_safety_follows_external_relay(clean_system):
    """The 15 K Tuo-Tui heating stop only applies in external mode while the external relay is on."""
    openamber = clean_system

    assert openamber.set_switch("emergency_mode_enabled", True)
    assert openamber.set_select("backup_heating_mode", "Externe backup verwarming")
    assert openamber.set_select("heat_mode_select", "Extern setpoint")
    assert openamber.set_number("manual_setpoint", 35.0)
    openamber.step(ms=50)
    openamber.advance_time(seconds=130, step_s=10)
    assert openamber.set_binary_sensor("external_heat_demand_wired", True)
    openamber.step(ms=50)

    # Emergency heating starts the external element (pump interval + settle).
    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Backup heater running", max_s=1300, step=10), (
        f"Emergency heating never started the external backup heater "
        f"(heat/cool: {openamber.get_entity('state_machine_state_heat_cool')})"
    )
    assert openamber.get_entity("external_backup_heating_relay") is True

    # Relay on: a 17 K difference points to missing flow and must trigger the safety stop.
    openamber.set_sensor("inlet_temperature_tui", 25.0)
    openamber.set_sensor("outlet_temperature_tuo", 42.0)  # 17 K above Tui
    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Idle", max_s=120, step=5), (
        "Safety stop must trigger while the external relay reports the element on"
    )
    assert openamber.get_entity("external_backup_heating_relay") is False, "Safety stop must switch the external relay off"
    assert openamber.get_entity("error_water_delta_t_too_high") is True, "The safety stop must raise TO08"
    openamber.set_switch("reset_start_timeout_errors_switch", True)
    openamber.step(ms=50)

    # The system restarts the cycle. During the pump phase the element is off, so the same 17 K
    # difference may not trigger the safety stop any more (it did when the function returned true
    # for external mode regardless of the relay).
    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Pump running", max_s=1300, step=10), (
        f"Heating never restarted after the safety stop "
        f"(heat/cool: {openamber.get_entity('state_machine_state_heat_cool')})"
    )
    hold_without_element(openamber, "state_machine_state_heat_cool")

    # The element takes over again and is safe to run with a small difference.
    openamber.set_sensor("outlet_temperature_tuo", 27.0)
    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Backup heater running", max_s=600, step=5), (
        f"Backup heater never restarted after the safety stop "
        f"(heat/cool: {openamber.get_entity('state_machine_state_heat_cool')})"
    )
    assert openamber.get_entity("external_backup_heating_relay") is True

    assert openamber.set_binary_sensor("external_heat_demand_wired", False)
    assert openamber.set_switch("emergency_mode_enabled", False)
    openamber.step(ms=100)


def test_dhw_delta_t_safety_follows_external_relay(clean_system):
    """The 15 K Tuo-Tui stop only applies in external mode while the external relay is on."""
    openamber = clean_system

    assert openamber.set_switch("emergency_mode_enabled", True)
    assert openamber.set_select("backup_heating_mode", "Externe backup verwarming")
    assert openamber.set_sensor("dhw_temperature_tw_sensor", 38.0)
    openamber.step(ms=50)

    assert advance_until_state(openamber, "state_machine_state_dhw", "Backup heater running", max_s=300, step=5), (
        f"DHW never started the external backup heater (dhw: {openamber.get_entity('state_machine_state_dhw')})"
    )
    assert openamber.get_entity("external_backup_heating_relay") is True

    # Relay on: a 16 K difference exceeds the DHW limit and must stop the unit.
    openamber.set_sensor("inlet_temperature_tui", 25.0)
    openamber.set_sensor("outlet_temperature_tuo", 46.0)  # 21 K above Tui
    assert advance_until_state(openamber, "state_machine_state_dhw", "Idle", max_s=120, step=5), (
        "Safety stop must trigger while the external relay reports the element on"
    )
    assert openamber.get_entity("external_backup_heating_relay") is False
    assert openamber.get_entity("error_water_delta_t_too_high") is True, "The safety stop must raise TO08"
    openamber.set_switch("reset_start_timeout_errors_switch", True)
    openamber.step(ms=50)

    # Restart: during the pump phase the element is off, so 16 K may not trigger the stop.
    assert advance_until_state(openamber, "state_machine_state_dhw", "Pump running", max_s=300, step=5), (
        f"DHW never restarted after the safety stop (dhw: {openamber.get_entity('state_machine_state_dhw')})"
    )
    hold_without_element(openamber, "state_machine_state_dhw")

    openamber.set_sensor("outlet_temperature_tuo", 31.0)
    assert advance_until_state(openamber, "state_machine_state_dhw", "Backup heater running", max_s=300, step=5), (
        f"Backup heater never restarted for DHW (dhw: {openamber.get_entity('state_machine_state_dhw')})"
    )
    assert openamber.get_entity("external_backup_heating_relay") is True

    # Finish the DHW cycle normally.
    assert openamber.set_sensor("dhw_temperature_tw_sensor", 51.0)
    assert advance_until_state(openamber, "state_machine_state_dhw", "Idle", max_s=420, step=10), (
        f"DHW cycle never ended (dhw: {openamber.get_entity('state_machine_state_dhw')})"
    )
    assert openamber.get_entity("external_backup_heating_relay") is False

    assert openamber.set_switch("emergency_mode_enabled", False)
    openamber.step(ms=100)
