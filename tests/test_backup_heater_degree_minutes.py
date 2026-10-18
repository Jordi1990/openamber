"""Automated tests for Backup Heater Degree-Minute Accumulation and Mode Selection.

Covers:
1. backup_heater_degmin_threshold & backup_heater_degmin_current_sensor:
   - When heating deficit exists and compressor operates at maximum capacity,
     degree-minutes accumulate over virtual time.
   - Once accumulated degree-minutes reach backup_heater_degmin_threshold,
     the backup heater turns ON automatically.
   - When supply temperature satisfies setpoint, backup heater turns OFF.
2. backup_heating_mode selection:
   - "Intern verwarmingselement": activates backup_heater_relay.
   - "Externe backup verwarming": activates external_backup_heating_relay.
"""

import pytest


def test_backup_heater_degree_minute_accumulation_triggers_internal_heater(clean_system):
    """
    Verify Degree-Minute Accumulation with Internal Backup Heater:
    1. Set backup_heating_mode to 'Intern verwarmingselement'.
    2. Configure threshold: backup_heater_degmin_threshold = 10.0 °C*min.
    3. Start heating demand with delta >= 12°C (target 35°C, Tc 20°C, diff = 15°C).
    4. With heat_compressor_mode 'Beperkt', softstart starts directly at mode 4 (max mode).
    5. Softstart completes (180s) -> enters COMPRESSOR_RUNNING at max mode.
    6. Advance virtual time by 60s (1 min * 15°C = 15 °C*min > threshold 10 °C*min).
    7. Verify backup_heater_relay turns ON automatically.
    8. Supply temperature warms to setpoint + stop delta -> backup heater turns OFF.
    """
    openamber = clean_system

    target_temperature = 35.0
    openamber.set_select("thermostat_mode_select", "Extern")
    openamber.set_select("heat_mode_select", "Extern setpoint")
    openamber.set_number("manual_setpoint", target_temperature)
    openamber.set_climate("pid_heat_temperature_control", target_temperature=target_temperature)
    openamber.set_number("compressor_start_delta_heating", 2.0)
    openamber.set_number("compressor_stop_delta_heating", 2.0)
    openamber.set_select("heat_compressor_mode", "Beperkt")
    openamber.set_select("backup_heating_mode", "Intern verwarmingselement")
    openamber.set_number("backup_heater_degmin_threshold", 10.0)

    # Initial supply temperature: delta = 15°C below target (triggers start mode 4 = max mode for 'Beperkt')
    openamber.set_sensor("current_water_temperature_tc_sensor", 20.0)
    openamber.set_sensor("heat_cool_temperature_tc", 20.0)
    openamber.set_sensor("outlet_temperature_tuo", 20.0)
    openamber.set_sensor("inlet_temperature_tui", 18.0)
    openamber.step(ms=50)

    # Ensure minimum compressor off-time has elapsed
    openamber.advance_time(seconds=130, step_s=10)
    openamber.step(ms=50)

    # Trigger space heat demand
    openamber.set_binary_sensor("external_heat_demand_wired", True)
    openamber.step(ms=100)
    assert openamber.get_entity("heat_demand_active_sensor") is True, "Heat demand should be active"

    # Advance time through pump interval in IDLE (900s)
    openamber.advance_time(seconds=900, step_s=30)
    openamber.step(ms=100)

    # Advance through pump settle (130s)
    openamber.advance_time(seconds=130, step_s=10)
    openamber.step(ms=100)

    # Advance through softstart (190s) into COMPRESSOR_RUNNING
    openamber.advance_time(seconds=190, step_s=10)
    openamber.step(ms=100)
    assert int(openamber.get_entity("compressor_control_select") or 0) == 4, "Compressor should be in capped mode 4 (Beperkt)"

    # Advance time to accumulate degree-minutes: ~60s with diff = 15°C -> 15 °C*min
    openamber.advance_time(seconds=60, step_s=10)
    openamber.step(ms=100)

    # Degree minutes reached threshold and triggered backup heater
    assert openamber.get_entity("backup_heater_relay") is True, (
        "Internal backup heater relay should turn ON when degree-minute threshold is exceeded"
    )
    assert openamber.get_entity("backup_heater_active_sensor") is True, (
        "Backup heater active sensor should be True when degree-minute threshold is exceeded"
    )

    # Water temperature warms to setpoint + stop delta (35.0 + 2.0 = 37.0°C) -> 38.0°C
    openamber.set_sensor("current_water_temperature_tc_sensor", 38.0)
    openamber.set_sensor("heat_cool_temperature_tc", 38.0)
    openamber.set_sensor("outlet_temperature_tuo", 38.0)
    openamber.set_sensor("inlet_temperature_tui", 36.0)
    openamber.advance_time(seconds=10, step_s=2)
    openamber.step(ms=100)

    # Backup heater must turn OFF
    assert openamber.get_entity("backup_heater_relay") is False, "Backup heater relay should turn OFF when water warms above setpoint + delta"
    assert openamber.get_entity("backup_heater_active_sensor") is False, "Backup heater active sensor should turn False when water warms above setpoint + delta"

    # Cleanup
    openamber.set_binary_sensor("external_heat_demand_wired", False)
    openamber.step(ms=100)


def test_backup_heater_mode_external_activates_external_relay(clean_system):
    """
    Verify Mode Selection:
    When backup_heating_mode is set to 'Externe backup verwarming',
    external_backup_heating_relay turns ON (instead of backup_heater_relay).
    """
    openamber = clean_system

    target_temperature = 35.0
    openamber.set_select("thermostat_mode_select", "Extern")
    openamber.set_select("heat_mode_select", "Extern setpoint")
    openamber.set_number("manual_setpoint", target_temperature)
    openamber.set_climate("pid_heat_temperature_control", target_temperature=target_temperature)
    openamber.set_number("compressor_start_delta_heating", 2.0)
    openamber.set_number("compressor_stop_delta_heating", 2.0)
    openamber.set_select("heat_compressor_mode", "Beperkt")
    openamber.set_select("backup_heating_mode", "Externe backup verwarming")
    openamber.set_number("backup_heater_degmin_threshold", 10.0)

    openamber.set_sensor("current_water_temperature_tc_sensor", 20.0)
    openamber.set_sensor("heat_cool_temperature_tc", 20.0)
    openamber.set_sensor("outlet_temperature_tuo", 20.0)
    openamber.set_sensor("inlet_temperature_tui", 18.0)
    openamber.step(ms=50)

    # Ensure min compressor off-time
    openamber.advance_time(seconds=130, step_s=10)
    openamber.step(ms=50)

    openamber.set_binary_sensor("external_heat_demand_wired", True)
    openamber.step(ms=100)

    # Pump interval (900s) + pump settle (130s) + softstart (190s)
    openamber.advance_time(seconds=900, step_s=30)
    openamber.step(ms=100)
    openamber.advance_time(seconds=130, step_s=10)
    openamber.step(ms=100)
    openamber.advance_time(seconds=190, step_s=10)
    openamber.step(ms=100)
    assert int(openamber.get_entity("compressor_control_select") or 0) == 4, "Compressor should be running in capped mode 4"

    # Advance time to accumulate degree-minutes
    openamber.advance_time(seconds=60, step_s=10)
    openamber.step(ms=100)

    # External relay must turn ON, internal relay must remain OFF
    assert openamber.get_entity("external_backup_heating_relay") is True, (
        "External backup heating relay should turn ON in 'Externe backup verwarming' mode"
    )
    assert openamber.get_entity("backup_heater_relay") is False, "Internal backup heater relay should remain OFF"

    # Water temperature satisfies setpoint
    openamber.set_sensor("current_water_temperature_tc_sensor", 38.0)
    openamber.set_sensor("heat_cool_temperature_tc", 38.0)
    openamber.set_sensor("outlet_temperature_tuo", 38.0)
    openamber.set_sensor("inlet_temperature_tui", 36.0)
    openamber.advance_time(seconds=10, step_s=2)
    openamber.step(ms=100)

    # External relay turns OFF
    assert openamber.get_entity("external_backup_heating_relay") is False, "External backup heater relay should turn OFF when satisfied"

    # Cleanup
    openamber.set_binary_sensor("external_heat_demand_wired", False)
    openamber.set_select("backup_heating_mode", "Intern verwarmingselement")
    openamber.step(ms=100)

def test_backup_heater_no_degree_minute_time_jump_after_defrost(clean_system):
    """
    Verify that `backup_degmin_last_ms_` is updated when recovering from defrost,
    preventing a massive degree-minute jump (defrost time + settle time) that would
    spontaneously trigger the backup heater:
    1. Start heating demand and advance until entering 'Compressor running' (mode 4).
    2. Accumulate a small baseline of degree-minutes (~2.5 - 5.0 °C*min).
    3. Trigger a 10-minute defrost (`defrost_active_sensor` = True for 600s) with water at 20°C (diff = 15°C).
    4. End defrost (`defrost_active_sensor` = False) with outside temp above boost threshold.
    5. Advance through COMPRESSOR_SETTLE_TIME_AFTER_DEFROST_S (300s) back into 'Compressor running'.
    6. Verify degree-minutes did NOT jump by (600s + 300s) * 15°C = +225 °C*min
       and `backup_heater_relay` remains OFF.
    """
    openamber = clean_system

    target_temperature = 35.0
    openamber.set_select("thermostat_mode_select", "Extern")
    openamber.set_select("heat_mode_select", "Extern setpoint")
    openamber.set_number("manual_setpoint", target_temperature)
    openamber.set_climate("pid_heat_temperature_control", target_temperature=target_temperature)
    openamber.set_number("compressor_start_delta_heating", 2.0)
    openamber.set_number("compressor_stop_delta_heating", 2.0)
    openamber.set_select("heat_compressor_mode", "Beperkt")
    openamber.set_select("backup_heating_mode", "Intern verwarmingselement")
    openamber.set_number("backup_heater_degmin_threshold", 30.0)

    # Initial supply temperature: 20°C (delta = 15°C -> starts in mode 4)
    openamber.set_sensor("current_water_temperature_tc_sensor", 20.0)
    openamber.set_sensor("heat_cool_temperature_tc", 20.0)
    openamber.set_sensor("outlet_temperature_tuo", 20.0)
    openamber.set_sensor("inlet_temperature_tui", 18.0)
    openamber.step(ms=50)

    # Ensure COMPRESSOR_MIN_OFF_S (120s) has elapsed
    openamber.advance_time(seconds=130, step_s=10)
    openamber.step(ms=50)

    # Trigger space heat demand
    openamber.set_binary_sensor("external_heat_demand_wired", True)
    openamber.step(ms=100)

    # Step in 10s increments until the state machine reaches "Compressor running"
    for _ in range(150):
        if openamber.get_entity("state_machine_state_heat_cool") == "Compressor running":
            break
        openamber.advance_time(seconds=10, step_s=10)
        openamber.step(ms=50)

    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"
    assert int(openamber.get_entity("compressor_control_select") or 0) == 4

    # Advance 10s in COMPRESSOR_RUNNING (accumulates ~2.5 - 5.0 °C*min)
    openamber.advance_time(seconds=10, step_s=10)
    openamber.step(ms=100)

    degmin_before = float(openamber.get_entity("backup_heater_degmin_current_sensor") or 0.0)
    assert degmin_before < 15.0, f"Pre-defrost degree-minutes should be low, got {degmin_before}"
    assert openamber.get_entity("backup_heater_relay") is False, "Backup heater must be OFF before defrost"

    # --- START DEFROST (10 minutes = 600s) ---
    openamber.set_binary_sensor("defrost_active_sensor", True)
    openamber.advance_time(seconds=10, step_s=5)
    openamber.step(ms=100)
    assert openamber.get_entity("state_machine_state_heat_cool") == "Defrosting"

    # Advance 10 minutes (600s) in DEFROSTING with cold water (20.0°C, diff = 15.0°C)
    openamber.advance_time(seconds=600, step_s=30)
    openamber.step(ms=100)
    assert openamber.get_entity("backup_heater_relay") is False, "Backup heater must remain OFF during defrost"

    # --- END DEFROST ---
    openamber.set_binary_sensor("defrost_active_sensor", False)
    for _ in range(30):
        openamber.set_sensor("temperature_outside_ta", 15.0)
        openamber.set_number("defrost_backup_heater_boost_temperature_sensor", -10.0)
        openamber.advance_time(seconds=1, step_s=1)
        openamber.step(ms=50)
        if openamber.get_entity("state_machine_state_heat_cool") != "Defrosting":
            break

    assert openamber.get_entity("state_machine_state_heat_cool") == "Wait for state switch"

    # Advance 310s to complete COMPRESSOR_SETTLE_TIME_AFTER_DEFROST_S (300s)
    # and transition through DEFROST_RECOVERY back into COMPRESSOR_RUNNING
    openamber.advance_time(seconds=310, step_s=10)
    openamber.step(ms=100)

    degmin_after = float(openamber.get_entity("backup_heater_degmin_current_sensor") or 0.0)
    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running", (
        f"Expected 'Compressor running' after settle time, got '{openamber.get_entity('state_machine_state_heat_cool')}' "
        f"(degmin={degmin_after})"
    )
    assert degmin_after < 20.0, (
        f"Expected degree-minutes < 20.0 after DEFROST_RECOVERY reset, but got {degmin_after}"
    )
    assert openamber.get_entity("backup_heater_relay") is False, (
        "Backup heater relay turned ON spontaneously after defrost!"
    )
    assert openamber.get_entity("backup_heater_active_sensor") is False

    # Cleanup
    openamber.set_binary_sensor("external_heat_demand_wired", False)
    openamber.step(ms=100)