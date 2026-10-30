"""Automated tests for the custom (OpenAmber controlled) defrost algorithm.
Covers the three defrost control modes (Buitenunit, Dry-run, OpenAmber), the Tp trigger with debounce,
the max runtime safety net, the return temperature block, the forced defrost request (working mode 4),
request timeout with retry and fallback to the outdoor unit, and the settings page in the UI.
"""

from helpers import advance_until_state, start_space_heating


def _configure(openamber, mode):
    openamber.set_select("custom_defrost_mode_select", mode)
    openamber.set_switch("custom_defrost_tp_trigger_switch", True)
    openamber.set_switch("custom_defrost_dt_trigger_switch", False)
    openamber.set_number("custom_defrost_max_ta", 7.0)
    openamber.set_number("custom_defrost_tp_band1", -3.0)
    openamber.set_number("custom_defrost_tp_band2", -3.0)
    openamber.set_number("custom_defrost_tp_band3", -5.0)
    openamber.set_number("custom_defrost_tp_band4", -12.0)
    openamber.set_number("custom_defrost_tp_debounce", 60.0)
    openamber.set_number("custom_defrost_max_runtime", 120.0)
    openamber.set_number("custom_defrost_fallback_tp", -2.0)
    openamber.set_number("custom_defrost_min_runtime", 10.0)
    openamber.set_number("custom_defrost_min_compressor_on", 0.0)
    openamber.set_number("custom_defrost_min_return_temperature", 20.0)
    openamber.set_number("custom_defrost_max_return_wait", 15.0)
    openamber.set_number("custom_defrost_confirm_timeout", 120.0)
    openamber.set_binary_sensor("warning_custom_defrost_failed", False)
    openamber.set_sensor("temperature_outside_ta", 2.0)
    openamber.set_sensor("evaporation_temperature_tp", 5.0)
    openamber.set_sensor("evaporating_temperature_te", 0.0)
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


def test_outdoor_unit_mode_does_nothing(clean_system):
    """In mode 'Buitenunit' the algorithm stays off and 3274 stays on outdoor unit control."""
    openamber = clean_system
    _configure(openamber, "Buitenunit")
    _heat_and_reset_runtime(openamber)

    openamber.advance_time(seconds=11 * 60, step_s=30)
    openamber.set_sensor("evaporation_temperature_tp", -6.0)
    openamber.advance_time(seconds=120, step_s=10)
    openamber.step(ms=50)

    assert openamber.get_entity("custom_defrost_status") == "Uit"
    assert openamber.get_entity("working_mode_switch") == "Verwarmen"
    assert openamber.get_entity("defrost_control_mode_select") == "Buitenunit"
    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"


def test_dry_run_trigger_and_delta_with_unit_defrost(clean_system):
    """Dry-run logs the trigger moment and compares it with the real start of the outdoor unit."""
    openamber = clean_system
    _configure(openamber, "Dry-run")
    _heat_and_reset_runtime(openamber)

    # Below minimum runtime since last defrost: blocked
    openamber.set_sensor("evaporation_temperature_tp", -4.0)
    openamber.advance_time(seconds=90, step_s=10)
    openamber.step(ms=50)
    assert openamber.get_entity("custom_defrost_status").startswith("Geblokkeerd: draaitijd")

    openamber.set_sensor("evaporation_temperature_tp", 5.0)
    openamber.advance_time(seconds=10 * 60, step_s=30)
    openamber.set_sensor("evaporation_temperature_tp", -4.0)
    openamber.advance_time(seconds=70, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity("custom_defrost_status") == "Dry-run: trigger"
    assert openamber.get_entity("custom_defrost_last_trigger").startswith("Dry-run: Tp -4.0 < -3.0")
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


def test_openamber_forced_defrost_flow(clean_system):
    """OpenAmber mode writes 3274 = master, requests working mode 4 and follows the normal defrost handling."""
    openamber = clean_system
    _configure(openamber, "OpenAmber")
    openamber.advance_time(seconds=10, step_s=5)
    assert openamber.get_entity("defrost_control_mode_select") == "Master"

    _heat_and_reset_runtime(openamber)
    openamber.advance_time(seconds=11 * 60, step_s=30)
    openamber.set_sensor("evaporation_temperature_tp", -4.0)
    openamber.advance_time(seconds=70, step_s=5)
    openamber.step(ms=50)

    assert openamber.get_entity("working_mode_switch") == "Geforceerde defrost"
    assert openamber.get_entity("state_machine_state_heat_cool") == "Defrost requested"
    assert openamber.get_entity("custom_defrost_status") == "Aangevraagd"
    assert openamber.get_entity("custom_defrost_last_trigger").startswith("Tp -4.0 < -3.0")
    assert int(openamber.get_entity("compressor_control_select") or 0) > 0, "Compressor must keep running"

    # Outdoor unit confirms the defrost
    openamber.set_binary_sensor("defrost_active_sensor", True)
    openamber.advance_time(seconds=10, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity("state_machine_state_heat_cool") == "Defrosting"
    assert openamber.get_entity("custom_defrost_status") == "Ontdooien"
    assert openamber.get_entity("working_mode_switch") == "Geforceerde defrost", "Working mode must not be touched during defrost"

    openamber.set_sensor("evaporation_temperature_tp", 8.0)
    openamber.advance_time(seconds=180, step_s=10)
    _end_defrost_by_unit(openamber)
    assert openamber.get_entity("custom_defrost_runtime_sensor") == 0
    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Compressor running", max_s=400)
    assert openamber.get_entity("custom_defrost_status") == "Bewaken"

    # Back to outdoor unit control
    openamber.set_select("custom_defrost_mode_select", "Buitenunit")
    openamber.advance_time(seconds=10, step_s=5)
    assert openamber.get_entity("defrost_control_mode_select") == "Buitenunit"


def test_openamber_request_timeout_retries_then_falls_back(clean_system):
    """Without confirmation (2118) the request is retried once, then control falls back to the outdoor unit."""
    openamber = clean_system
    _configure(openamber, "OpenAmber")
    openamber.set_number("custom_defrost_confirm_timeout", 30.0)
    _heat_and_reset_runtime(openamber)

    openamber.advance_time(seconds=11 * 60, step_s=30)
    openamber.set_sensor("evaporation_temperature_tp", -4.0)
    openamber.advance_time(seconds=70, step_s=5)
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


def test_safety_net_max_runtime(clean_system):
    """The max runtime safety net triggers even when all triggers are disabled."""
    openamber = clean_system
    _configure(openamber, "Dry-run")
    openamber.set_switch("custom_defrost_tp_trigger_switch", False)
    openamber.set_number("custom_defrost_max_runtime", 30.0)
    _heat_and_reset_runtime(openamber)

    openamber.set_sensor("evaporation_temperature_tp", -2.5)
    openamber.advance_time(seconds=20 * 60, step_s=30)
    openamber.step(ms=50)
    assert openamber.get_entity("custom_defrost_status") == "Bewaken"

    openamber.advance_time(seconds=7 * 60, step_s=30)
    openamber.step(ms=50)
    assert openamber.get_entity("custom_defrost_status") == "Dry-run: trigger"
    assert openamber.get_entity("custom_defrost_last_trigger").startswith("Dry-run: Vangnet")


def test_low_return_temperature_blocks_until_max_wait(clean_system):
    """A low return temperature delays the defrost, but only up to the maximum wait time."""
    openamber = clean_system
    _configure(openamber, "Dry-run")
    openamber.set_number("custom_defrost_max_return_wait", 3.0)
    _heat_and_reset_runtime(openamber)
    openamber.advance_time(seconds=11 * 60, step_s=30)

    openamber.set_sensor("outlet_temperature_tuo", 22.0)
    openamber.set_sensor("inlet_temperature_tui", 16.0)
    openamber.set_sensor("evaporation_temperature_tp", -4.0)
    openamber.advance_time(seconds=70, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity("custom_defrost_status").startswith("Geblokkeerd: retour")

    openamber.advance_time(seconds=3 * 60, step_s=10)
    openamber.step(ms=50)
    assert openamber.get_entity("custom_defrost_status") == "Dry-run: trigger"


def test_custom_defrost_settings_page(clean_system):
    """The 'Eigen defrost' tab starts with the control mode and shows the details on the last page."""
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
    assert openamber.is_hidden("set_custom_defrost_status_row") is True
    assert openamber.get_label("custom_defrost_page_indicator") == "Pagina 1 / 7"

    openamber.click("custom_defrost_page_next_button")
    openamber.step(ms=100)
    assert openamber.is_hidden("set_custom_defrost_mode_row") is True
    assert openamber.is_hidden("set_custom_defrost_tp_band1_row") is False
    assert openamber.get_label("custom_defrost_page_indicator") == "Pagina 2 / 7"

    for _ in range(4):
        openamber.click("custom_defrost_page_next_button")
        openamber.step(ms=50)
    assert openamber.get_label("custom_defrost_page_indicator") == "Pagina 6 / 7"
    assert openamber.is_hidden("set_custom_defrost_status_row") is False
    assert openamber.is_hidden("set_custom_defrost_runtime_row") is False
    openamber.advance_time(seconds=10, step_s=5)
    assert openamber.get_label("set_custom_defrost_status_val") == "Bewaken"

    openamber.click("custom_defrost_page_prev_button")
    openamber.step(ms=100)
    assert openamber.get_label("custom_defrost_page_indicator") == "Pagina 5 / 7"

    openamber.set_select("custom_defrost_mode_select", "Buitenunit")
    openamber.set_switch("advanced_settings_enabled", False)
    openamber.step(ms=50)
