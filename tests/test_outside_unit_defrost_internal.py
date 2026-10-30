"""Automated tests for the read-only view of the outdoor unit's internal defrost variables
(registers 5176-5231, firmware 0x0110 only) on page 7 of the custom defrost settings.
"""


def _open_unit_algorithm_page(openamber):
    assert openamber.click("nav_settings")
    openamber.step(ms=100)
    openamber.set_switch("advanced_settings_enabled", True)
    openamber.step(ms=100)
    assert openamber.click("stab_advanced")
    openamber.step(ms=50)
    assert openamber.click("advanced_stab_custom_defrost")
    openamber.step(ms=100)
    for _ in range(6):
        openamber.click("custom_defrost_page_next_button")
        openamber.step(ms=50)
    assert openamber.get_label("custom_defrost_page_indicator") == "Pagina 7 / 7"


def test_unit_algorithm_page_shows_internal_state(clean_system):
    openamber = clean_system
    openamber.set_binary_sensor("defrost_active_sensor", False)
    openamber.advance_time(seconds=35, step_s=5)
    _open_unit_algorithm_page(openamber)

    assert openamber.get_entity("outside_unit_internal_registers_supported") is True
    assert openamber.is_hidden("set_custom_defrost_unit_state_row") is False
    assert openamber.is_hidden("set_custom_defrost_status_row") is True
    assert openamber.get_label("set_custom_defrost_unit_state_val") == "Detectie actief"
    assert openamber.get_label("set_custom_defrost_unit_runtime_val").endswith("/ 45 min")

    openamber.set_binary_sensor("defrost_active_sensor", True)
    openamber.advance_time(seconds=35, step_s=5)
    assert openamber.get_entity("outside_unit_defrost_internal_state_text") == "Ontdooien"
    assert openamber.get_label("set_custom_defrost_unit_state_val") == "Ontdooien"

    openamber.set_binary_sensor("defrost_active_sensor", False)
    openamber.set_switch("advanced_settings_enabled", False)
    openamber.step(ms=50)


def test_unit_algorithm_hidden_for_other_firmware(clean_system):
    openamber = clean_system
    openamber.set_sensor("outside_unit_program_version", 0x0111)
    openamber.advance_time(seconds=35, step_s=5)
    assert openamber.get_entity("outside_unit_internal_registers_supported") is False
    assert openamber.get_entity("outside_unit_defrost_internal_state_text") == "Niet beschikbaar voor deze firmware"
    assert openamber.get_label("set_custom_defrost_unit_state_val") == "Niet beschikbaar"
    openamber.set_sensor("outside_unit_program_version", 0x0110)
    openamber.advance_time(seconds=35, step_s=5)
