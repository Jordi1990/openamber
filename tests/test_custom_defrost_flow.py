"""Automated tests for the custom (OpenAmber controlled) defrost algorithm.
Covers both defrost control modes (Buitenunit, OpenAmber) and switching between them, the Mitsubishi start line,
the interval route with the adaptive interval (whose 120 min maximum is the safety net), the ice counter, the
outdoor unit's 30 min lock, the optional humidity sensor, the return temperature block, the forced defrost request (working mode 4),
request timeout with retry, warning and a new attempt after 30 min, and the settings page in the UI.
"""

import pytest

from helpers import advance_until_state, start_space_heating

INTERVAL_S = 35 * 60


def _configure(openamber, mode):
    openamber.set_select("defrost_control_select", mode)
    openamber.set_number("custom_defrost_delta_at_zero", 7.0)
    openamber.set_number("custom_defrost_delta_at_cold", 4.0)
    openamber.set_number("custom_defrost_max_coil_temperature", -5.0)
    # Changing the minimum interval also resets the adaptive interval of a previous test
    openamber.set_number("custom_defrost_min_runtime", 40.0)
    openamber.advance_time(seconds=5, step_s=5)
    openamber.set_number("custom_defrost_min_runtime", 35.0)
    openamber.set_number("custom_defrost_frost_point_margin", 0.0)
    openamber.set_number("custom_defrost_ice_threshold", 70.0)
    openamber.set_binary_sensor("warning_custom_defrost_failed", False)
    openamber.set_sensor("outdoor_humidity", 0.0)
    openamber.set_sensor("temperature_outside_ta", 0.0)
    openamber.set_sensor("evaporation_temperature_tp", 5.0)
    openamber.step(ms=50)


def _run_unit_defrost(openamber):
    """Let the outdoor unit perform a defrost; this also resets the runtime counter."""
    openamber.set_binary_sensor("defrost_active_sensor", True)
    openamber.advance_time(seconds=10, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity("state_machine_state_heat_cool") == "Defrosting"
    openamber.set_binary_sensor("defrost_active_sensor", False)
    openamber.advance_time(seconds=10, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity("custom_defrost_runtime_sensor") == 0
    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Compressor running", max_s=400)


def _heat_and_reset_runtime(openamber):
    start_space_heating(openamber, tc=28.0, target=35.0)
    _run_unit_defrost(openamber)


def _end_defrost_by_unit(openamber):
    """Simulate the end of a forced defrost: outdoor unit drops 2118 and returns to heating."""
    openamber.set_select("working_mode_switch", "Verwarmen")
    openamber.set_binary_sensor("defrost_active_sensor", False)
    openamber.advance_time(seconds=10, step_s=5)
    openamber.step(ms=50)


def _wait_for_interval(openamber):
    """Let the compressor run the full interval with a clean coil."""
    openamber.advance_time(seconds=INTERVAL_S + 30, step_s=30)
    openamber.step(ms=50)


def test_outdoor_unit_mode_does_nothing(clean_system):
    """In mode 'Buitenunit' the algorithm stays off and 3274 stays on outdoor unit control."""
    openamber = clean_system
    _configure(openamber, "Buitenunit")
    _heat_and_reset_runtime(openamber)

    _wait_for_interval(openamber)
    openamber.set_sensor("evaporation_temperature_tp", -12.0)
    openamber.advance_time(seconds=240, step_s=10)
    openamber.step(ms=50)

    assert openamber.get_entity("custom_defrost_status") == "Uit"
    assert openamber.get_entity("working_mode_switch") == "Verwarmen"
    assert openamber.get_entity("defrost_control_mode_select") == "Buitenunit"
    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"


@pytest.mark.parametrize("ta, expected", [(7.0, -5.0), (5.0, -5.0), (0.0, -7.0), (-5.0, -11.0), (-10.0, -15.0), (-15.0, -19.0), (-20.0, -24.0)])
def test_start_line_follows_mitsubishi(clean_system, ta, expected):
    """Start line: Ta minus 7 K at 0 °C and warmer, 4 K at -15 °C and colder, linear in between, never above -5 °C."""
    openamber = clean_system
    _configure(openamber, "Buitenunit")
    openamber.set_sensor("temperature_outside_ta", ta)
    openamber.advance_time(seconds=10, step_s=5)
    assert openamber.get_entity("custom_defrost_start_temperature") == pytest.approx(expected, abs=0.05)


def test_openamber_waits_for_interval_and_start_line(clean_system):
    """OpenAmber waits for the interval, then requests a defrost once Tp stays 3 min below the start line."""
    openamber = clean_system
    _configure(openamber, "OpenAmber")
    _heat_and_reset_runtime(openamber)

    # Cold coil, but the interval since the last defrost has not passed yet
    openamber.set_sensor("evaporation_temperature_tp", -8.0)
    openamber.advance_time(seconds=5 * 60, step_s=10)
    openamber.step(ms=50)
    assert openamber.get_entity("custom_defrost_status").startswith("Bewaken: interval")

    # Interval passed, Tp above the start line (-7 °C at Ta 0 °C) and above -2 °C (interval route): no defrost
    openamber.set_sensor("evaporation_temperature_tp", -1.5)
    _wait_for_interval(openamber)
    openamber.advance_time(seconds=5 * 60, step_s=10)
    openamber.step(ms=50)
    assert openamber.get_entity("custom_defrost_status") == "Bewaken"

    # Tp below the start line: the 3 minute confirmation runs first
    openamber.set_sensor("evaporation_temperature_tp", -8.0)
    openamber.advance_time(seconds=120, step_s=10)
    openamber.step(ms=50)
    assert openamber.get_entity("custom_defrost_status") == "Bewaken"
    assert openamber.get_entity("working_mode_switch") == "Verwarmen"
    openamber.advance_time(seconds=70, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity("custom_defrost_status") == "Aangevraagd"
    assert openamber.get_entity("custom_defrost_last_trigger").startswith("Startlijn: Tp -8.0 <= -7.0")


def test_unit_defrost_measures_execution_without_prephase(clean_system):
    """In mode 'Buitenunit' the defrost is still measured: the execution time excludes the unit's pre-phase."""
    openamber = clean_system
    _configure(openamber, "Buitenunit")
    _heat_and_reset_runtime(openamber)
    _wait_for_interval(openamber)

    openamber.set_binary_sensor("defrost_active_sensor", True)
    openamber.advance_time(seconds=5, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity("custom_defrost_status") == "Ontdooien (buitenunit)"

    # 5 min pre-phase (2118 high, coil still cold), then hot gas: 90 s melting around 0 °C, 30 s up to the end temperature
    openamber.set_sensor("evaporation_temperature_tp", -8.0)
    openamber.advance_time(seconds=300, step_s=10)
    openamber.set_sensor("outlet_temperature_tuo", 25.0)
    openamber.set_sensor("inlet_temperature_tui", 30.0)
    openamber.set_sensor("evaporation_temperature_tp", 1.0)
    openamber.advance_time(seconds=90, step_s=10)
    openamber.set_sensor("evaporation_temperature_tp", 18.0)
    openamber.advance_time(seconds=30, step_s=10)
    openamber.set_binary_sensor("defrost_active_sensor", False)
    openamber.advance_time(seconds=10, step_s=5)
    openamber.step(ms=50)
    duration = openamber.get_entity("custom_defrost_last_duration")
    assert duration is not None and 115 <= duration <= 135, f"Execution must exclude the pre-phase, got {duration}"
    assert openamber.get_entity("custom_defrost_runtime_sensor") == 0
    # A defrost by the outdoor unit includes its pre-phase, so the interval is not adapted
    assert openamber.get_entity("custom_defrost_interval_sensor") == 35


def test_switch_to_outdoor_unit_during_defrost(clean_system):
    """Switching to 'Buitenunit' during a defrost lets it finish, restores heating and then hands 3274 back."""
    openamber = clean_system
    _configure(openamber, "OpenAmber")
    _heat_and_reset_runtime(openamber)
    _wait_for_interval(openamber)
    openamber.set_sensor("evaporation_temperature_tp", -8.0)
    openamber.advance_time(seconds=190, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity("working_mode_switch") == "Geforceerde defrost"

    openamber.set_binary_sensor("defrost_active_sensor", True)
    openamber.advance_time(seconds=10, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity("custom_defrost_status") == "Ontdooien"

    openamber.set_select("defrost_control_select", "Buitenunit")
    openamber.advance_time(seconds=70, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity("defrost_control_mode_select") == "Master", "3274 must not change during a defrost"
    assert openamber.get_entity("state_machine_state_heat_cool") == "Defrosting"

    openamber.set_sensor("evaporation_temperature_tp", 8.0)
    openamber.set_binary_sensor("defrost_active_sensor", False)
    openamber.advance_time(seconds=10, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity("working_mode_switch") == "Verwarmen"
    assert openamber.get_entity("defrost_control_mode_select") == "Buitenunit"
    assert openamber.get_entity("custom_defrost_status") == "Uit"
    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Compressor running", max_s=400)


def test_openamber_forced_defrost_flow(clean_system):
    """OpenAmber mode writes 3274 = master, requests working mode 4, restores heating and adapts the interval."""
    openamber = clean_system
    _configure(openamber, "OpenAmber")
    openamber.advance_time(seconds=10, step_s=5)
    assert openamber.get_entity("defrost_control_mode_select") == "Master"

    _heat_and_reset_runtime(openamber)
    _wait_for_interval(openamber)
    openamber.set_sensor("evaporation_temperature_tp", -8.0)
    openamber.advance_time(seconds=190, step_s=5)
    openamber.step(ms=50)

    assert openamber.get_entity("working_mode_switch") == "Geforceerde defrost"
    assert openamber.get_entity("state_machine_state_heat_cool") == "Defrost requested"
    assert openamber.get_entity("custom_defrost_status") == "Aangevraagd"
    assert openamber.get_entity("custom_defrost_last_trigger").startswith("Startlijn: Tp -8.0 <= -7.0")
    assert int(openamber.get_entity("compressor_control_select") or 0) > 0, "Compressor must keep running"

    # Outdoor unit confirms the defrost
    openamber.set_binary_sensor("defrost_active_sensor", True)
    openamber.advance_time(seconds=10, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity("state_machine_state_heat_cool") == "Defrosting"
    assert openamber.get_entity("custom_defrost_status") == "Ontdooien"
    assert openamber.get_entity("working_mode_switch") == "Geforceerde defrost", "Working mode must not be touched during defrost"

    # A short defrost (about 2 min): OpenAmber restores heating itself and waits 20 min longer next time
    openamber.set_sensor("evaporation_temperature_tp", 8.0)
    openamber.advance_time(seconds=110, step_s=10)
    openamber.set_binary_sensor("defrost_active_sensor", False)
    openamber.advance_time(seconds=10, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity("working_mode_switch") == "Verwarmen"
    assert openamber.get_entity("custom_defrost_runtime_sensor") == 0
    assert openamber.get_entity("custom_defrost_interval_sensor") == 55
    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Compressor running", max_s=400)
    assert openamber.get_entity("custom_defrost_status").startswith("Bewaken")

    # Back to outdoor unit control
    openamber.set_select("defrost_control_select", "Buitenunit")
    openamber.advance_time(seconds=10, step_s=5)
    assert openamber.get_entity("defrost_control_mode_select") == "Buitenunit"
    openamber.set_number("custom_defrost_min_runtime", 35.0)


def test_openamber_request_timeout_warns_and_retries(clean_system):
    """Without confirmation (2118) the request is retried once; after that a warning, and a new attempt after 30 min."""
    openamber = clean_system
    _configure(openamber, "OpenAmber")
    _heat_and_reset_runtime(openamber)

    _wait_for_interval(openamber)
    openamber.set_sensor("evaporation_temperature_tp", -8.0)
    openamber.advance_time(seconds=190, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity("working_mode_switch") == "Geforceerde defrost"

    # First timeout: working mode restored and a second request follows
    assert advance_until_state(openamber, "working_mode_switch", "Verwarmen", max_s=150, step=5)
    assert advance_until_state(openamber, "working_mode_switch", "Geforceerde defrost", max_s=30, step=5), "Second attempt expected"

    # Second timeout: warning, control stays with OpenAmber
    assert advance_until_state(openamber, "warning_custom_defrost_failed", True, max_s=150, step=5)
    assert openamber.get_entity("working_mode_switch") == "Verwarmen"
    # The trigger timers restart after the failure; the block shows once the start line holds for 3 min again
    openamber.advance_time(seconds=200, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity("defrost_control_select") == "OpenAmber"
    assert openamber.get_entity("defrost_control_mode_select") == "Master"
    assert openamber.get_entity("custom_defrost_status").startswith("Geblokkeerd: mislukt")
    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"

    # After 30 min a new request; a confirmed defrost clears the warning
    assert advance_until_state(openamber, "working_mode_switch", "Geforceerde defrost", max_s=35 * 60, step=30)
    openamber.set_binary_sensor("defrost_active_sensor", True)
    openamber.advance_time(seconds=10, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity("warning_custom_defrost_failed") is False
    _end_defrost_by_unit(openamber)


def _confirm_short_forced_defrost(openamber):
    """The outdoor unit confirms the forced defrost and finishes it in about 2 min (little ice)."""
    openamber.set_binary_sensor("defrost_active_sensor", True)
    openamber.advance_time(seconds=10, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity("custom_defrost_status") == "Ontdooien"
    openamber.set_sensor("evaporation_temperature_tp", 8.0)
    openamber.advance_time(seconds=110, step_s=10)
    _end_defrost_by_unit(openamber)
    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Compressor running", max_s=400)


def test_interval_route_follows_adaptive_interval(clean_system):
    """Without reaching the start line, the coil below -2 °C defrosts after the adaptive interval, which grows after
    a short defrost (instead of a fixed 120 min safety net)."""
    openamber = clean_system
    _configure(openamber, "OpenAmber")
    _heat_and_reset_runtime(openamber)

    # Tp -3 °C: well above the start line (-7 °C at Ta 0 °C), below -2 °C
    openamber.set_sensor("evaporation_temperature_tp", -3.0)
    openamber.advance_time(seconds=20 * 60, step_s=30)
    openamber.step(ms=50)
    assert openamber.get_entity("custom_defrost_status").startswith("Bewaken: interval")

    assert advance_until_state(openamber, "custom_defrost_status", "Aangevraagd", max_s=20 * 60, step=30)
    assert openamber.get_entity("custom_defrost_last_trigger").startswith("Interval")
    assert 35 <= openamber.get_entity("custom_defrost_runtime_sensor") <= 40

    # Short defrost: the interval grows to 55 min, so the same coil now waits longer
    _confirm_short_forced_defrost(openamber)
    assert openamber.get_entity("custom_defrost_interval_sensor") == 55
    openamber.set_sensor("evaporation_temperature_tp", -3.0)
    openamber.advance_time(seconds=45 * 60, step_s=30)
    openamber.step(ms=50)
    status = openamber.get_entity("custom_defrost_status")
    assert status.startswith("Bewaken: interval") and status.endswith("/ 55 min"), status
    assert openamber.get_entity("working_mode_switch") == "Verwarmen"

    assert advance_until_state(openamber, "custom_defrost_status", "Aangevraagd", max_s=20 * 60, step=30)
    assert openamber.get_entity("custom_defrost_last_trigger").startswith("Interval")
    assert openamber.get_entity("custom_defrost_runtime_sensor") >= 55
    _confirm_short_forced_defrost(openamber)


def test_ice_counter_triggers_with_humidity(clean_system):
    """With a humidity value the ice counter sums the icing rate and defrosts at the threshold, even when Tp stays
    above the start line and above -2 °C. The counter restarts after the defrost."""
    openamber = clean_system
    _configure(openamber, "OpenAmber")
    openamber.set_sensor("outdoor_humidity", 100.0)
    _heat_and_reset_runtime(openamber)

    # Ta 0 °C, RV 100 %, Tp -1.5 °C: about 71 Pa excess over ice, so 70 Pa·h after about 59 min
    openamber.set_sensor("evaporation_temperature_tp", -1.5)
    openamber.advance_time(seconds=50 * 60, step_s=30)
    openamber.step(ms=50)
    ice_load = openamber.get_entity("custom_defrost_ice_load_sensor")
    assert 55 <= ice_load <= 63, f"Ice counter after 50 min, got {ice_load}"
    assert openamber.get_entity("custom_defrost_status") == "Bewaken"
    assert openamber.get_label("set_custom_defrost_ice_load_val") == f"{ice_load:.0f} Pa·h"

    assert advance_until_state(openamber, "custom_defrost_status", "Aangevraagd", max_s=15 * 60, step=30)
    assert openamber.get_entity("custom_defrost_last_trigger").startswith("IJsteller")
    # The runtime also counts the minutes with the warm coil right after the reset defrost (no icing then)
    assert 56 <= openamber.get_entity("custom_defrost_runtime_sensor") <= 70
    assert openamber.get_entity("custom_defrost_ice_load_sensor") >= 70

    _confirm_short_forced_defrost(openamber)
    assert openamber.get_entity("custom_defrost_ice_load_sensor") == 0
    openamber.set_sensor("outdoor_humidity", 0.0)


def test_ice_counter_off_and_without_humidity(clean_system):
    """Threshold 0 switches the ice counter off; without a humidity value nothing is counted."""
    openamber = clean_system
    _configure(openamber, "OpenAmber")
    _heat_and_reset_runtime(openamber)

    openamber.set_sensor("evaporation_temperature_tp", -1.5)
    openamber.advance_time(seconds=40 * 60, step_s=30)
    openamber.step(ms=50)
    assert openamber.get_entity("custom_defrost_ice_load_sensor") == 0

    openamber.set_number("custom_defrost_ice_threshold", 0.0)
    openamber.set_sensor("outdoor_humidity", 100.0)
    openamber.advance_time(seconds=60 * 60, step_s=30)
    openamber.step(ms=50)
    assert openamber.get_entity("custom_defrost_ice_load_sensor") > 70
    assert openamber.get_entity("custom_defrost_status") == "Bewaken"
    assert openamber.get_entity("working_mode_switch") == "Verwarmen"
    openamber.set_sensor("outdoor_humidity", 0.0)
    openamber.set_number("custom_defrost_ice_threshold", 70.0)


def test_max_interval_is_safety_net(clean_system):
    """The interval is at most 120 min, so a coil just below -2 °C always defrosts after 120 min (former safety net)."""
    openamber = clean_system
    _configure(openamber, "OpenAmber")
    openamber.set_number("custom_defrost_min_runtime", 120.0)
    _heat_and_reset_runtime(openamber)

    openamber.set_sensor("evaporation_temperature_tp", -2.5)
    openamber.advance_time(seconds=114 * 60, step_s=60)
    openamber.step(ms=50)
    assert openamber.get_entity("custom_defrost_status").startswith("Bewaken: interval")

    assert advance_until_state(openamber, "custom_defrost_status", "Aangevraagd", max_s=12 * 60, step=30)
    assert openamber.get_entity("custom_defrost_last_trigger").startswith("Interval")
    # The 3 min Tp hold already passed during the interval, so it defrosts right at 120 min.
    assert openamber.get_entity("custom_defrost_runtime_sensor") >= 120
    openamber.set_number("custom_defrost_min_runtime", 35.0)

def test_humidity_sensor_blocks_defrost_in_dry_air(clean_system):
    """With a humidity value, a coil above the frost point of the air does not defrost."""
    openamber = clean_system
    _configure(openamber, "OpenAmber")
    openamber.set_sensor("outdoor_humidity", 40.0)
    _heat_and_reset_runtime(openamber)
    _wait_for_interval(openamber)

    # Ta 0 °C, RV 40 %: frost point about -10.7 °C. Tp -8 °C is below the start line but above the frost point.
    assert openamber.get_label("set_custom_defrost_humidity_value_val") == "40% / -10.7°C"
    openamber.set_sensor("evaporation_temperature_tp", -8.0)
    openamber.advance_time(seconds=5 * 60, step_s=10)
    openamber.step(ms=50)
    assert openamber.get_entity("custom_defrost_status").startswith("Bewaken: droge lucht")

    openamber.set_sensor("evaporation_temperature_tp", -11.0)
    openamber.advance_time(seconds=60, step_s=10)
    openamber.step(ms=50)
    assert openamber.get_entity("custom_defrost_status") == "Aangevraagd"

    # Without a valid humidity value the regulation works as without sensor
    openamber.set_sensor("outdoor_humidity", 0.0)
    openamber.advance_time(seconds=10, step_s=5)
    assert openamber.get_label("set_custom_defrost_humidity_value_val") == "0%"


def test_humidity_frost_point_margin_and_max_delay(clean_system):
    """The frost point margin moves the icing limit down; the maximum delay ends the wait for dry air."""
    openamber = clean_system
    _configure(openamber, "OpenAmber")
    openamber.set_sensor("outdoor_humidity", 40.0)
    openamber.set_number("custom_defrost_frost_point_margin", 2.0)
    _heat_and_reset_runtime(openamber)
    _wait_for_interval(openamber)

    # Frost point about -10.7 °C, with 2 K margin the limit is about -12.7 °C: Tp -11 °C still waits.
    openamber.set_sensor("evaporation_temperature_tp", -11.0)
    openamber.advance_time(seconds=4 * 60, step_s=10)
    openamber.step(ms=50)
    assert openamber.get_entity("custom_defrost_status").startswith("Bewaken: droge lucht")

    # At the maximum interval (120 min) the humidity no longer delays the defrost.
    assert advance_until_state(openamber, "custom_defrost_status", "Aangevraagd", max_s=90 * 60, step=60)
    assert openamber.get_entity("custom_defrost_runtime_sensor") >= 120

    openamber.set_sensor("outdoor_humidity", 0.0)


def test_low_return_temperature_blocks_until_max_wait(clean_system):
    """A low return temperature delays the defrost, but only up to the maximum wait time."""
    openamber = clean_system
    _configure(openamber, "OpenAmber")
    _heat_and_reset_runtime(openamber)
    _wait_for_interval(openamber)

    openamber.set_sensor("outlet_temperature_tuo", 22.0)
    openamber.set_sensor("inlet_temperature_tui", 16.0)
    openamber.set_sensor("evaporation_temperature_tp", -8.0)
    openamber.advance_time(seconds=190, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity("custom_defrost_status").startswith("Geblokkeerd: retour")

    # Maximum wait 15 min
    openamber.advance_time(seconds=15 * 60, step_s=10)
    openamber.step(ms=50)
    assert openamber.get_entity("custom_defrost_status") == "Aangevraagd"


def test_defrost_settings_menu(clean_system):
    """The 'Defrost' menu opens the sub-pages; the tiles follow the defrost control mode."""
    openamber = clean_system
    _configure(openamber, "OpenAmber")

    assert openamber.click("nav_settings")
    openamber.step(ms=100)
    openamber.set_switch("advanced_settings_enabled", True)
    openamber.step(ms=100)
    assert openamber.click("stab_advanced")
    openamber.step(ms=50)
    assert openamber.click("advanced_stab_defrost")
    openamber.step(ms=100)
    assert openamber.is_hidden("panel_advanced_page_defrost") is False
    # OpenAmber: start line and humidity, not the outdoor unit's start temperatures
    assert openamber.is_hidden("defrost_stab_startlijn") is False
    assert openamber.is_hidden("defrost_stab_vocht") is False
    assert openamber.is_hidden("defrost_stab_start_temps") is True
    assert openamber.is_hidden("defrost_stab_end") is True
    assert openamber.is_hidden("defrost_stab_status") is False

    # Algemeen
    assert openamber.click("defrost_stab_general")
    openamber.step(ms=100)
    assert openamber.is_hidden("panel_advanced_page_defrost_general") is False
    assert openamber.is_hidden("set_custom_defrost_mode_row") is False
    assert openamber.is_hidden("set_custom_defrost_frost_margin_row") is True
    assert openamber.is_hidden("set_custom_defrost_status_row") is True

    # Startlijn
    assert openamber.click("settings_menu_back_btn")
    openamber.step(ms=100)
    assert openamber.click("defrost_stab_startlijn")
    openamber.step(ms=100)
    assert openamber.is_hidden("set_custom_defrost_mode_row") is True
    assert openamber.is_hidden("set_custom_defrost_delta_zero_row") is False
    assert openamber.get_label("set_custom_defrost_delta_zero_val") == "7.0 K"

    # Luchtvochtigheid
    assert openamber.click("settings_menu_back_btn")
    openamber.step(ms=100)
    assert openamber.click("defrost_stab_vocht")
    openamber.step(ms=100)
    assert openamber.is_hidden("set_custom_defrost_frost_margin_row") is False
    openamber.set_sensor("outdoor_humidity", 85.0)
    openamber.step(ms=50)
    assert openamber.get_label("set_custom_defrost_humidity_value_val").startswith("85% / ")
    assert openamber.is_hidden("set_custom_defrost_ice_threshold_row") is False
    assert openamber.get_label("set_custom_defrost_ice_threshold_val") == "70 Pa·h"
    openamber.set_number("custom_defrost_ice_threshold", 0.0)
    openamber.step(ms=50)
    assert openamber.get_label("set_custom_defrost_ice_threshold_val") == "Uit"
    openamber.set_number("custom_defrost_ice_threshold", 70.0)
    openamber.set_sensor("outdoor_humidity", 0.0)

    # Status
    assert openamber.click("settings_menu_back_btn")
    openamber.step(ms=100)
    assert openamber.click("defrost_stab_status")
    openamber.step(ms=100)
    assert openamber.is_hidden("set_custom_defrost_status_row") is False
    assert openamber.is_hidden("set_custom_defrost_runtime_row") is False
    openamber.advance_time(seconds=10, step_s=5)
    assert openamber.get_label("set_custom_defrost_status_val").startswith("Bewaken")

    # Buitenunit: the outdoor unit's start temperatures instead of the start line and humidity (tiles on the menu page)
    assert openamber.click("settings_menu_back_btn")
    openamber.step(ms=100)
    openamber.set_select("defrost_control_select", "Buitenunit")
    openamber.step(ms=50)
    assert openamber.is_hidden("defrost_stab_start_temps") is False
    assert openamber.is_hidden("defrost_stab_startlijn") is True
    assert openamber.is_hidden("defrost_stab_vocht") is True
    assert openamber.is_hidden("defrost_stab_end") is False
    assert openamber.is_hidden("defrost_stab_status") is True
    openamber.set_switch("advanced_settings_enabled", False)
    openamber.step(ms=50)
