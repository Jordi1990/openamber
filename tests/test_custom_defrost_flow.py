"""Automated tests for the custom (OpenAmber controlled) defrost algorithm.
Covers both defrost control modes (Buitenunit, OpenAmber) and switching between them,
the interval route with the adaptive interval (whose 120 min maximum is the safety net), the outdoor unit's 30 min
lock, the return temperature block, the forced defrost request (working mode 4),
the request timeout (error TO10, no retry) and the settings page in the UI.
"""

import pytest

from helpers import advance_until_state, start_space_heating

INTERVAL_S = 35 * 60


def _configure(openamber, mode):
    openamber.set_select("defrost_control_select", mode)
    openamber.set_number("custom_defrost_min_runtime", 35.0)
    # Start from the minimum interval, not from what a previous test left
    openamber.set_switch("mock_reset_custom_defrost_interval", True)
    openamber.advance_time(seconds=5, step_s=5)
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


def test_openamber_waits_for_interval_and_cold_coil(clean_system):
    """OpenAmber waits for the interval, then requests a defrost once Tp stays 3 min below -2 °C."""
    openamber = clean_system
    _configure(openamber, "OpenAmber")
    _heat_and_reset_runtime(openamber)

    # Cold coil, but the interval since the last defrost has not passed yet
    openamber.set_sensor("evaporation_temperature_tp", -8.0)
    openamber.advance_time(seconds=5 * 60, step_s=10)
    openamber.step(ms=50)
    assert openamber.get_entity("custom_defrost_status").startswith("Bewaken: interval")

    # Interval passed, Tp above -2 °C: no defrost
    openamber.set_sensor("evaporation_temperature_tp", -1.5)
    _wait_for_interval(openamber)
    openamber.advance_time(seconds=5 * 60, step_s=10)
    openamber.step(ms=50)
    assert openamber.get_entity("custom_defrost_status") == "Bewaken"

    # Tp below -2 °C: the 3 minute confirmation runs first
    openamber.set_sensor("evaporation_temperature_tp", -8.0)
    openamber.advance_time(seconds=120, step_s=10)
    openamber.step(ms=50)
    assert openamber.get_entity("custom_defrost_status") == "Bewaken"
    assert openamber.get_entity("working_mode_switch") == "Verwarmen"
    openamber.advance_time(seconds=70, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity("custom_defrost_status") == "Aangevraagd"
    assert openamber.get_entity("custom_defrost_last_trigger").startswith("Interval: ")


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
    assert openamber.get_entity("custom_defrost_last_trigger").startswith("Interval: ")
    assert int(openamber.get_entity("compressor_control_select") or 0) > 0, "Compressor must keep running"

    # Outdoor unit confirms the defrost
    openamber.set_binary_sensor("defrost_active_sensor", True)
    openamber.advance_time(seconds=10, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity("state_machine_state_heat_cool") == "Defrosting"
    assert openamber.get_entity("custom_defrost_status") == "Ontdooien"
    assert openamber.get_entity("working_mode_switch") == "Geforceerde defrost", "Working mode must not be touched during defrost"

    # A short defrost (about 90 s): OpenAmber restores heating itself and waits 20 min longer next time
    openamber.set_sensor("evaporation_temperature_tp", 8.0)
    openamber.advance_time(seconds=80, step_s=10)
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


def test_openamber_request_timeout_raises_error(clean_system):
    """Without confirmation (2118) within 120 s the request is taken back and error TO10 stops the heat pump, without a
    new attempt. The timeout error reset clears it."""
    openamber = clean_system
    _configure(openamber, "OpenAmber")
    _heat_and_reset_runtime(openamber)

    _wait_for_interval(openamber)
    openamber.set_sensor("evaporation_temperature_tp", -8.0)
    openamber.advance_time(seconds=190, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity("working_mode_switch") == "Geforceerde defrost"

    # Timeout: working mode 4 taken back, TO10 latched
    assert advance_until_state(openamber, "error_defrost_start_timeout", True, max_s=150, step=5)
    openamber.step(ms=50)
    assert openamber.get_entity("error_active") is True
    assert openamber.get_entity("working_mode_switch") != "Geforceerde defrost"

    # No new attempt while TO10 is active, and the heat pump stops like with the other timeout errors
    for _ in range(20):
        openamber.advance_time(seconds=30, step_s=10)
        assert openamber.get_entity("working_mode_switch") != "Geforceerde defrost", "No retry expected"
    openamber.step(ms=50)
    assert openamber.get_entity("custom_defrost_status") == "Fout: defrost startte niet"
    assert openamber.get_entity("state_machine_state_heat_cool") != "Compressor running"
    assert openamber.get_entity("defrost_control_select") == "OpenAmber"

    # The timeout error reset clears it
    openamber.set_switch("reset_start_timeout_errors_switch", True)
    openamber.step(ms=50)
    assert openamber.get_entity("error_defrost_start_timeout") is False
    assert openamber.get_entity("error_active") is False


def _confirm_short_forced_defrost(openamber):
    """The outdoor unit confirms the forced defrost and finishes it in about 90 s (little ice)."""
    openamber.set_binary_sensor("defrost_active_sensor", True)
    openamber.advance_time(seconds=10, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity("custom_defrost_status") == "Ontdooien"
    openamber.set_sensor("evaporation_temperature_tp", 8.0)
    openamber.advance_time(seconds=80, step_s=10)
    _end_defrost_by_unit(openamber)
    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Compressor running", max_s=400)


def test_interval_route_follows_adaptive_interval(clean_system):
    """The coil below -2 °C defrosts after the adaptive interval, which grows after
    a short defrost (instead of a fixed 120 min safety net)."""
    openamber = clean_system
    _configure(openamber, "OpenAmber")
    _heat_and_reset_runtime(openamber)

    # Tp -3 °C: below -2 °C
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
    # OpenAmber: not the outdoor unit's start temperatures
    assert openamber.is_hidden("defrost_stab_start_temps") is True
    assert openamber.is_hidden("defrost_stab_end") is True
    assert openamber.is_hidden("defrost_stab_status") is False

    # Algemeen
    assert openamber.click("defrost_stab_general")
    openamber.step(ms=100)
    assert openamber.is_hidden("panel_advanced_page_defrost_general") is False
    assert openamber.is_hidden("set_custom_defrost_mode_row") is False
    assert openamber.is_hidden("set_custom_defrost_status_row") is True
    assert openamber.is_hidden("set_custom_defrost_min_runtime_row") is False
    assert openamber.get_label("set_custom_defrost_min_runtime_val") == "35 min"

    # Status
    assert openamber.click("settings_menu_back_btn")
    openamber.step(ms=100)
    assert openamber.click("defrost_stab_status")
    openamber.step(ms=100)
    assert openamber.is_hidden("set_custom_defrost_status_row") is False
    assert openamber.is_hidden("set_custom_defrost_runtime_row") is False
    openamber.advance_time(seconds=10, step_s=5)
    assert openamber.get_label("set_custom_defrost_status_val").startswith("Bewaken")

    # Buitenunit: the outdoor unit's start temperatures instead of the interval (tiles on the menu page)
    assert openamber.click("settings_menu_back_btn")
    openamber.step(ms=100)
    openamber.set_select("defrost_control_select", "Buitenunit")
    openamber.step(ms=50)
    assert openamber.is_hidden("defrost_stab_start_temps") is False
    assert openamber.is_hidden("set_custom_defrost_min_runtime_row") is True
    assert openamber.is_hidden("defrost_stab_end") is False
    assert openamber.is_hidden("defrost_stab_status") is True
    openamber.set_switch("advanced_settings_enabled", False)
    openamber.step(ms=50)


def test_control_mode_written_once_per_change_and_after_restart(clean_system):
    """3274 is set once when the mode changes and once after a (simulated) restart, not rewritten periodically."""
    openamber = clean_system
    _configure(openamber, "OpenAmber")
    openamber.advance_time(seconds=5, step_s=1)
    assert openamber.get_entity("defrost_control_mode_select") == "Master"

    # Something else puts 3274 back to the outdoor unit: OpenAmber does not keep rewriting it
    openamber.set_select("defrost_control_mode_select", "Buitenunit")
    openamber.advance_time(seconds=3 * 60, step_s=10)
    assert openamber.get_entity("defrost_control_mode_select") == "Buitenunit"

    # After a restart the mode is kept and 3274 is written once again
    assert openamber.set_switch("mock_simulate_esp_restart", True)
    openamber.advance_time(seconds=20, step_s=1)
    assert openamber.get_entity("defrost_control_select") == "OpenAmber"
    assert openamber.get_entity("defrost_control_mode_select") == "Master"

    # A mode change writes it once more
    openamber.set_select("defrost_control_select", "Buitenunit")
    openamber.advance_time(seconds=5, step_s=1)
    assert openamber.get_entity("defrost_control_mode_select") == "Buitenunit"


def test_defrost_over_two_minutes_keeps_interval(clean_system):
    """A defrost of about 150 s had ice (clearly white frost on the camera, 28 Nov 2026), so it is not 'short': the
    adaptive interval stays the same instead of growing 20 min."""
    openamber = clean_system
    _configure(openamber, "OpenAmber")
    _heat_and_reset_runtime(openamber)

    openamber.set_sensor("evaporation_temperature_tp", -3.0)
    assert advance_until_state(openamber, "custom_defrost_status", "Aangevraagd", max_s=45 * 60, step=30)
    openamber.set_binary_sensor("defrost_active_sensor", True)
    openamber.advance_time(seconds=10, step_s=5)
    openamber.set_sensor("evaporation_temperature_tp", 8.0)
    openamber.advance_time(seconds=150, step_s=10)
    _end_defrost_by_unit(openamber)
    duration = openamber.get_entity("custom_defrost_last_duration")
    assert 140 <= duration <= 170, duration
    assert openamber.get_entity("custom_defrost_interval_sensor") == 35
