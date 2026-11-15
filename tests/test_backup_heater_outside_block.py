"""Tests for blocking the space-heating backup heater above an outside temperature.

"Backup element blokkeren" (backup_heater_outside_block_enabled, default off) with
"Backup element blokkeren boven buitentemperatuur" (backup_heater_outside_block_temperature, default 7 °C):
above the threshold (averaged Ta, temperature_outside_ta_avg_1h) the degree/minutes do not switch the backup
heater on and are not counted, so the heater does not start at once when Ta drops below the threshold.
"""

import pytest

from helpers import advance_until_state, set_outside_temperature

TARGET = 35.0
RELAY = {
    "Intern verwarmingselement": "backup_heater_relay",
    "Externe backup verwarming": "external_backup_heating_relay",
}


def _set_water_temperature(openamber, tc):
    openamber.set_sensor("current_water_temperature_tc_sensor", tc)
    openamber.set_sensor("heat_cool_temperature_tc", tc)
    openamber.set_sensor("outlet_temperature_tuo", tc)
    openamber.set_sensor("inlet_temperature_tui", tc - 2.0)


def _start_heating_with_deficit(openamber, backup_mode="Intern verwarmingselement"):
    """Space heating 15 °C below the setpoint with 'Beperkt' (soft start directly at the maximum mode 4), so the
    degree/minutes count from the moment the compressor runs: 10 °C·min is reached after 40 s."""
    openamber.set_select("thermostat_mode_select", "Extern")
    openamber.set_select("heat_mode_select", "Extern setpoint")
    openamber.set_number("manual_setpoint", TARGET)
    openamber.set_climate("pid_heat_temperature_control", target_temperature=TARGET)
    openamber.set_number("compressor_start_delta_heating", 2.0)
    openamber.set_number("compressor_stop_delta_heating", 2.0)
    openamber.set_select("heat_compressor_mode", "Beperkt")
    openamber.set_select("backup_heating_mode", backup_mode)
    openamber.set_number("backup_heater_degmin_threshold", 10.0)
    _set_water_temperature(openamber, TARGET - 15.0)
    openamber.set_binary_sensor("external_heat_demand_wired", True)
    openamber.step(ms=50)
    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Compressor running", max_s=1800, step=5)
    assert int(openamber.get_entity("compressor_control_select") or 0) == 4


def _finish_heating(openamber):
    """Water above setpoint + stop delta: the backup heater goes off; then end the demand."""
    _set_water_temperature(openamber, TARGET + 3.0)
    openamber.advance_time(seconds=10, step_s=2)
    openamber.step(ms=50)
    openamber.set_binary_sensor("external_heat_demand_wired", False)
    openamber.set_select("backup_heating_mode", "Intern verwarmingselement")
    openamber.step(ms=50)


def _degmin(openamber):
    return float(openamber.get_entity("backup_heater_degmin_current_sensor") or 0.0)


@pytest.mark.parametrize("backup_mode", list(RELAY))
def test_backup_heater_blocked_above_outside_temperature(clean_system, backup_mode):
    """Above the threshold the heater stays off and the degree/minutes stay at 0; once Ta drops below it the
    degree/minutes count from 0 (no immediate start) and the heater comes on as before."""
    openamber = clean_system
    relay = RELAY[backup_mode]
    set_outside_temperature(openamber, 12.0)
    assert openamber.set_switch("backup_heater_outside_block_enabled", True)

    _start_heating_with_deficit(openamber, backup_mode)
    openamber.advance_time(seconds=120, step_s=5)  # 15 °C x 2 min = 30 °C·min without the block
    openamber.step(ms=50)
    assert openamber.get_entity(relay) is False, "Backup heater must stay off above the outside temperature"
    assert _degmin(openamber) == 0.0, "Degree/minutes must not be counted while blocked"
    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"

    # Ta drops below the threshold: counting starts from 0, so no immediate start.
    set_outside_temperature(openamber, 6.0)
    assert openamber.get_entity(relay) is False, "Backup heater must not start at once when the block ends"
    assert _degmin(openamber) < 10.0

    openamber.advance_time(seconds=60, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity(relay) is True, "Backup heater must work again below the outside temperature"

    _finish_heating(openamber)
    assert openamber.get_entity(relay) is False


def test_backup_heater_not_blocked_when_switch_off(clean_system):
    """Default (switch off): unchanged behaviour, the heater comes on at a high outside temperature too."""
    openamber = clean_system
    assert openamber.get_entity("backup_heater_outside_block_enabled") is False
    set_outside_temperature(openamber, 12.0)

    _start_heating_with_deficit(openamber)
    openamber.advance_time(seconds=60, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity("backup_heater_relay") is True

    _finish_heating(openamber)


def test_backup_heater_not_blocked_at_threshold(clean_system):
    """At the threshold itself (averaged Ta 7 °C = 7 °C) the heater is allowed."""
    openamber = clean_system
    assert abs(float(openamber.get_entity("temperature_outside_ta_avg_1h")) - 7.0) < 0.01
    assert openamber.set_switch("backup_heater_outside_block_enabled", True)

    _start_heating_with_deficit(openamber)
    openamber.advance_time(seconds=60, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity("backup_heater_relay") is True

    _finish_heating(openamber)


def test_sg_ready_max_boost_still_switches_heater_on_when_blocked(clean_system):
    """SG Ready max boost is an explicit request for extra consumption and still switches the heater on."""
    openamber = clean_system
    set_outside_temperature(openamber, 12.0)
    assert openamber.set_switch("backup_heater_outside_block_enabled", True)

    _start_heating_with_deficit(openamber)
    openamber.advance_time(seconds=120, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity("backup_heater_relay") is False

    openamber.set_select("sg_ready_mode_select", "Max boost")
    openamber.advance_time(seconds=10, step_s=2)
    openamber.step(ms=50)
    assert openamber.get_entity("sg_ready_max_boost_mode_active_sensor") is True
    assert openamber.get_entity("backup_heater_relay") is True

    openamber.set_select("sg_ready_mode_select", "Normaal")
    openamber.advance_time(seconds=10, step_s=2)
    _finish_heating(openamber)


def test_backup_heater_outside_block_ui(clean_system):
    """The toggle and the threshold under Instellingen > Bijverwarmen > Verwarmen."""
    openamber = clean_system
    assert openamber.click("bijverwarmen_stab_verwarmen")
    openamber.step(ms=50)
    assert openamber.get_widget("set_backup_block_switch_ui").get("checked") is False
    assert openamber.get_label("set_backup_block_temp_val") == "7°C"

    assert openamber.click("set_backup_block_switch_ui")
    openamber.step(ms=50)
    assert openamber.get_entity("backup_heater_outside_block_enabled") is True
    assert openamber.get_widget("set_backup_block_switch_ui").get("checked") is True

    assert openamber.click("set_backup_block_temp_inc")
    openamber.step(ms=50)
    assert float(openamber.get_entity("backup_heater_outside_block_temperature")) == 8.0
    assert openamber.get_label("set_backup_block_temp_val") == "8°C"

    assert openamber.click("set_backup_block_temp_dec")
    assert openamber.click("set_backup_block_temp_dec")
    openamber.step(ms=50)
    assert float(openamber.get_entity("backup_heater_outside_block_temperature")) == 6.0
    assert openamber.get_label("set_backup_block_temp_val") == "6°C"

    # Changes from elsewhere (web server, Home Assistant) update the controls too.
    assert openamber.set_switch("backup_heater_outside_block_enabled", False)
    assert openamber.set_number("backup_heater_outside_block_temperature", 3.0)
    openamber.step(ms=50)
    assert openamber.get_widget("set_backup_block_switch_ui").get("checked") is False
    assert openamber.get_label("set_backup_block_temp_val") == "3°C"
