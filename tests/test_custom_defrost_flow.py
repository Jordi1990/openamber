"""Automated tests for the custom (OpenAmber controlled) defrost algorithm.
Covers the three defrost control modes (Buitenunit, Dry-run, OpenAmber), the Mitsubishi start line,
the cycling compressor route, the safety net, the interval and the outdoor unit's 30 min lock,
the optional humidity sensor, the return temperature block, the forced defrost request (working mode 4),
request timeout with retry and fallback to the outdoor unit, and the settings page in the UI.
"""

import pytest

from helpers import advance_until_state, start_space_heating

INTERVAL_S = 35 * 60


def _configure(openamber, mode):
    openamber.set_select("custom_defrost_mode_select", mode)
    openamber.set_switch("custom_defrost_humidity_switch", False)
    openamber.set_number("custom_defrost_max_ta", 7.0)
    openamber.set_number("custom_defrost_delta_at_zero", 7.0)
    openamber.set_number("custom_defrost_delta_at_cold", 4.0)
    openamber.set_number("custom_defrost_max_coil_temperature", -5.0)
    openamber.set_number("custom_defrost_max_runtime", 120.0)
    openamber.set_number("custom_defrost_fallback_tp", -2.0)
    # Changing the minimum interval also resets the adaptive interval of a previous test
    openamber.set_number("custom_defrost_min_runtime", 40.0)
    openamber.advance_time(seconds=5, step_s=5)
    openamber.set_number("custom_defrost_min_runtime", 35.0)
    openamber.set_number("custom_defrost_min_return_temperature", 20.0)
    openamber.set_number("custom_defrost_max_return_wait", 15.0)
    openamber.set_number("custom_defrost_confirm_timeout", 120.0)
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
    _configure(openamber, "Dry-run")
    openamber.set_sensor("temperature_outside_ta", ta)
    openamber.advance_time(seconds=10, step_s=5)
    assert openamber.get_entity("custom_defrost_start_temperature") == pytest.approx(expected, abs=0.05)


def test_dry_run_waits_for_interval_and_start_line(clean_system):
    """Dry-run waits for the interval, then triggers once Tp stays 3 min below the start line."""
    openamber = clean_system
    _configure(openamber, "Dry-run")
    _heat_and_reset_runtime(openamber)

    # Cold coil, but the interval since the last defrost has not passed yet
    openamber.set_sensor("evaporation_temperature_tp", -8.0)
    openamber.advance_time(seconds=5 * 60, step_s=10)
    openamber.step(ms=50)
    assert openamber.get_entity("custom_defrost_status").startswith("Bewaken: interval")

    # Interval passed, Tp above the start line (-7 °C at Ta 0 °C): no defrost
    openamber.set_sensor("evaporation_temperature_tp", -6.0)
    _wait_for_interval(openamber)
    openamber.advance_time(seconds=5 * 60, step_s=10)
    openamber.step(ms=50)
    assert openamber.get_entity("custom_defrost_status") == "Bewaken"

    # Tp below the start line: the 3 minute confirmation runs first
    openamber.set_sensor("evaporation_temperature_tp", -8.0)
    openamber.advance_time(seconds=120, step_s=10)
    openamber.step(ms=50)
    assert openamber.get_entity("custom_defrost_status") == "Bewaken"
    openamber.advance_time(seconds=70, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity("custom_defrost_status") == "Dry-run: trigger"
    assert openamber.get_entity("custom_defrost_last_trigger").startswith("Dry-run: Startlijn: Tp -8.0 <= -7.0")
    # Dry-run never takes over control
    assert openamber.get_entity("working_mode_switch") == "Verwarmen"
    assert openamber.get_entity("defrost_control_mode_select") == "Buitenunit"
    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"

    # Outdoor unit starts its own defrost about 3 minutes later
    openamber.advance_time(seconds=180, step_s=10)
    openamber.set_binary_sensor("defrost_active_sensor", True)
    openamber.advance_time(seconds=5, step_s=5)
    openamber.step(ms=50)
    delta = openamber.get_entity("custom_defrost_dry_run_delta")
    assert delta is not None and 2.5 <= delta <= 4.0, f"Expected ~3 min dry-run delta, got {delta}"
    assert openamber.get_entity("custom_defrost_status") == "Ontdooien (buitenunit)"

    openamber.advance_time(seconds=120, step_s=10)
    openamber.set_binary_sensor("defrost_active_sensor", False)
    openamber.advance_time(seconds=10, step_s=5)
    openamber.step(ms=50)
    duration = openamber.get_entity("custom_defrost_last_duration")
    assert duration is not None and 120 <= duration <= 135, f"Unexpected defrost duration {duration}"
    assert openamber.get_entity("custom_defrost_runtime_sensor") == 0
    # A defrost by the outdoor unit includes its pre-phase, so the interval is not adapted
    assert openamber.get_entity("custom_defrost_interval_sensor") == 35


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
    openamber.set_select("custom_defrost_mode_select", "Buitenunit")
    openamber.advance_time(seconds=10, step_s=5)
    assert openamber.get_entity("defrost_control_mode_select") == "Buitenunit"
    openamber.set_number("custom_defrost_min_runtime", 35.0)


def test_openamber_request_timeout_retries_then_falls_back(clean_system):
    """Without confirmation (2118) the request is retried once, then control falls back to the outdoor unit."""
    openamber = clean_system
    _configure(openamber, "OpenAmber")
    openamber.set_number("custom_defrost_confirm_timeout", 30.0)
    _heat_and_reset_runtime(openamber)

    _wait_for_interval(openamber)
    openamber.set_sensor("evaporation_temperature_tp", -8.0)
    openamber.advance_time(seconds=190, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity("working_mode_switch") == "Geforceerde defrost"

    # First timeout: working mode restored and a second request follows
    assert advance_until_state(openamber, "working_mode_switch", "Verwarmen", max_s=60, step=5)
    assert openamber.get_entity("custom_defrost_mode_select") == "OpenAmber"
    assert advance_until_state(openamber, "working_mode_switch", "Geforceerde defrost", max_s=30, step=5), "Second attempt expected"

    # Second timeout: fallback to the outdoor unit
    assert advance_until_state(openamber, "custom_defrost_mode_select", "Buitenunit", max_s=60, step=5)
    assert openamber.get_entity("warning_custom_defrost_failed") is True
    assert openamber.get_entity("working_mode_switch") == "Verwarmen"
    openamber.advance_time(seconds=10, step_s=5)
    assert openamber.get_entity("defrost_control_mode_select") == "Buitenunit"
    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"


def test_cycling_compressor_route(clean_system):
    """With 10 or more compressor stops and Ta <= 3 °C, Tp below -5 °C is enough (no start line needed)."""
    openamber = clean_system
    _configure(openamber, "Dry-run")
    _heat_and_reset_runtime(openamber)
    _wait_for_interval(openamber)

    # Tp -6 °C is above the start line at Ta 0 °C (-7 °C), so only the cycling route can trigger
    openamber.set_sensor("evaporation_temperature_tp", -6.0)
    # Short compressor stops (the mock compressor follows compressor_control_select). Not every blip lands on a
    # controller update, so do more than 10.
    for _ in range(16):
        openamber.set_select("compressor_control_select", "0")
        openamber.advance_time(seconds=16, step_s=1)
    openamber.step(ms=50)
    assert openamber.get_entity("custom_defrost_status") == "Bewaken"

    openamber.advance_time(seconds=7 * 60, step_s=10)
    openamber.step(ms=50)
    assert openamber.get_entity("custom_defrost_status") == "Dry-run: trigger"
    assert openamber.get_entity("custom_defrost_last_trigger").startswith("Dry-run: Pendelen")


def test_safety_net_max_runtime(clean_system):
    """The safety net triggers after the maximum runtime even when Tp never reaches the start line."""
    openamber = clean_system
    _configure(openamber, "Dry-run")
    openamber.set_number("custom_defrost_max_runtime", 40.0)
    openamber.set_number("custom_defrost_min_runtime", 60.0)
    _heat_and_reset_runtime(openamber)

    openamber.set_sensor("evaporation_temperature_tp", -2.5)
    openamber.advance_time(seconds=34 * 60, step_s=30)
    openamber.step(ms=50)
    assert openamber.get_entity("custom_defrost_status").startswith("Bewaken")

    assert advance_until_state(openamber, "custom_defrost_status", "Dry-run: trigger", max_s=12 * 60, step=30)
    assert openamber.get_entity("custom_defrost_last_trigger").startswith("Dry-run: Vangnet")
    assert openamber.get_entity("custom_defrost_runtime_sensor") >= 43
    openamber.set_number("custom_defrost_min_runtime", 35.0)


def test_humidity_sensor_blocks_defrost_in_dry_air(clean_system):
    """With the humidity sensor enabled, a coil above the frost point of the air does not defrost."""
    openamber = clean_system
    _configure(openamber, "Dry-run")
    openamber.set_switch("custom_defrost_humidity_switch", True)
    openamber.set_sensor("outdoor_humidity", 40.0)
    _heat_and_reset_runtime(openamber)
    _wait_for_interval(openamber)

    # Ta 0 °C, RV 40 %: frost point about -10.7 °C. Tp -8 °C is below the start line but above the frost point.
    frost_point = openamber.get_entity("custom_defrost_frost_point")
    assert frost_point == pytest.approx(-10.7, abs=0.2)
    openamber.set_sensor("evaporation_temperature_tp", -8.0)
    openamber.advance_time(seconds=5 * 60, step_s=10)
    openamber.step(ms=50)
    assert openamber.get_entity("custom_defrost_status").startswith("Bewaken: droge lucht")

    openamber.set_sensor("evaporation_temperature_tp", -11.0)
    openamber.advance_time(seconds=60, step_s=10)
    openamber.step(ms=50)
    assert openamber.get_entity("custom_defrost_status") == "Dry-run: trigger"

    # Without a valid humidity value the regulation works as without sensor
    openamber.set_sensor("outdoor_humidity", 0.0)
    openamber.advance_time(seconds=10, step_s=5)
    assert openamber.get_entity("custom_defrost_frost_point") is None
    openamber.set_switch("custom_defrost_humidity_switch", False)


def test_low_return_temperature_blocks_until_max_wait(clean_system):
    """A low return temperature delays the defrost, but only up to the maximum wait time."""
    openamber = clean_system
    _configure(openamber, "Dry-run")
    openamber.set_number("custom_defrost_max_return_wait", 3.0)
    _heat_and_reset_runtime(openamber)
    _wait_for_interval(openamber)

    openamber.set_sensor("outlet_temperature_tuo", 22.0)
    openamber.set_sensor("inlet_temperature_tui", 16.0)
    openamber.set_sensor("evaporation_temperature_tp", -8.0)
    openamber.advance_time(seconds=190, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity("custom_defrost_status").startswith("Geblokkeerd: retour")

    openamber.advance_time(seconds=3 * 60, step_s=10)
    openamber.step(ms=50)
    assert openamber.get_entity("custom_defrost_status") == "Dry-run: trigger"


def test_custom_defrost_settings_page(clean_system):
    """The 'Eigen defrost' tab starts with the control mode, then the start line, and shows the details on page 4."""
    openamber = clean_system
    _configure(openamber, "Dry-run")

    assert openamber.click("nav_settings")
    openamber.step(ms=100)
    openamber.set_switch("advanced_settings_enabled", True)
    openamber.step(ms=100)
    assert openamber.click("stab_advanced")
    openamber.step(ms=50)
    assert openamber.click("advanced_stab_custom_defrost")
    openamber.step(ms=100)
    assert openamber.is_hidden("panel_advanced_page_custom_defrost") is False
    assert openamber.is_hidden("set_custom_defrost_mode_row") is False
    assert openamber.is_hidden("set_custom_defrost_humidity_row") is False
    assert openamber.is_hidden("set_custom_defrost_status_row") is True
    assert openamber.get_label("custom_defrost_page_indicator") == "Pagina 1 / 5"

    openamber.click("custom_defrost_page_next_button")
    openamber.step(ms=100)
    assert openamber.is_hidden("set_custom_defrost_mode_row") is True
    assert openamber.is_hidden("set_custom_defrost_delta_zero_row") is False
    assert openamber.get_label("set_custom_defrost_delta_zero_val") == "7.0 K"
    assert openamber.get_label("custom_defrost_page_indicator") == "Pagina 2 / 5"

    for _ in range(2):
        openamber.click("custom_defrost_page_next_button")
        openamber.step(ms=50)
    assert openamber.get_label("custom_defrost_page_indicator") == "Pagina 4 / 5"
    assert openamber.is_hidden("set_custom_defrost_status_row") is False
    assert openamber.is_hidden("set_custom_defrost_runtime_row") is False
    openamber.advance_time(seconds=10, step_s=5)
    assert openamber.get_label("set_custom_defrost_status_val").startswith("Bewaken")

    openamber.click("custom_defrost_page_prev_button")
    openamber.step(ms=100)
    assert openamber.get_label("custom_defrost_page_indicator") == "Pagina 3 / 5"

    openamber.set_select("custom_defrost_mode_select", "Buitenunit")
    openamber.set_switch("advanced_settings_enabled", False)
    openamber.step(ms=50)
