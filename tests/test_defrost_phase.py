"""Defrost phase detection: pre-phase, actual defrost and recovery.

When the outdoor unit decides itself, register 2118 rises up to 300 s before the actual defrost.
OpenAmber keeps heating during that pre-phase and only reacts once the compressor has dropped
to the defrost standby frequency (3279).
"""

from helpers import start_space_heating


def test_prephase_keeps_heating_until_frequency_drops(clean_system):
    openamber = clean_system
    start_space_heating(openamber, tc=28.0, target=35.0)

    # Pre-phase: 2118 set, compressor still at its normal frequency.
    openamber.set_number("mock_compressor_frequency_override", 70.0)
    openamber.set_binary_sensor("defrost_active_sensor", True)
    openamber.advance_time(seconds=60, step_s=2)
    openamber.step(ms=50)

    assert openamber.get_entity("defrost_phase_text") == "Voorfase"
    assert openamber.get_entity("defrost_running_sensor") is False
    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running", (
        "Heating must continue during the outdoor unit's pre-phase"
    )

    # Actual defrost: compressor drops to the standby frequency (36 Hz).
    openamber.set_number("mock_compressor_frequency_override", 36.0)
    openamber.advance_time(seconds=4, step_s=1)
    openamber.step(ms=50)

    assert openamber.get_entity("defrost_running_sensor") is True
    assert openamber.get_entity("defrost_phase_text") == "Ontdooien"
    assert openamber.get_entity("state_machine_state_heat_cool") == "Defrosting"

    # Defrost ends: recovery phase is reported for 30 s.
    openamber.set_binary_sensor("defrost_active_sensor", False)
    openamber.set_number("mock_compressor_frequency_override", -1.0)
    openamber.advance_time(seconds=4, step_s=1)
    openamber.step(ms=50)
    assert openamber.get_entity("defrost_phase_text") == "Herstel"

    openamber.advance_time(seconds=40, step_s=2)
    openamber.step(ms=50)
    assert openamber.get_entity("defrost_phase_text") == "Geen"


def test_prephase_times_out_without_frequency_drop(clean_system):
    openamber = clean_system
    start_space_heating(openamber, tc=28.0, target=35.0)

    openamber.set_number("mock_compressor_frequency_override", 70.0)
    openamber.set_binary_sensor("defrost_active_sensor", True)
    openamber.advance_time(seconds=340, step_s=10)
    openamber.step(ms=50)

    assert openamber.get_entity("defrost_running_sensor") is True, (
        "After DEFROST_PREPHASE_MAX_S the defrost is treated as running, even without a frequency drop"
    )
    assert openamber.get_entity("state_machine_state_heat_cool") == "Defrosting"

    openamber.set_binary_sensor("defrost_active_sensor", False)
    openamber.set_number("mock_compressor_frequency_override", -1.0)
    openamber.step(ms=50)
