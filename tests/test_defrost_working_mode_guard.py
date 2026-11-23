"""Standby is not written while the outdoor unit can still finish a defrost.

The outdoor unit aborts a running defrost without its recovery phase when 3999 leaves heating,
so the heat/cool controller waits in 'Wait pump stop' until defrost_active_sensor (2118) drops.

The unit only runs its pre-phase and defrost timers while the compressor runs. When 2118 rises just
before OpenAmber stops the compressor, 2118 stays set until standby is written. Waiting for it then
blocks the controller (and the pump cycles of frost protection) for good, so OpenAmber only waits while
the compressor still runs, and never longer than DEFROST_STANDBY_WAIT_MAX_S.
"""

from helpers import advance_until_state, start_space_heating

DEFROST_STANDBY_WAIT_MAX_S = 15 * 60


def _stop_compressor_then_defrost(openamber, compressor_frequency):
    """Stop the compressor through the stop delta, then report a defrost while the pump runs on.

    compressor_frequency is the frequency the outdoor unit reports once 2118 is set: 0 for a stopped
    compressor, above 0 when the unit still runs it.
    """
    openamber.set_sensor("heat_cool_temperature_tc", 38.0)
    openamber.set_sensor("current_water_temperature_tc_sensor", 38.0)
    waited = 0
    while openamber.get_entity("state_machine_state_heat_cool") != "Pump running":
        assert waited < 1800, (
            f"Compressor never stopped (state: {openamber.get_entity('state_machine_state_heat_cool')})"
        )
        openamber.advance_time(seconds=5, step_s=5)
        openamber.step(ms=20)
        waited += 5
    openamber.set_number("mock_compressor_frequency_override", compressor_frequency)
    openamber.set_binary_sensor("defrost_active_sensor", True)
    openamber.set_switch("heat_demand_switch", False)
    openamber.step(ms=50)


def test_standby_not_blocked_by_defrost_flag_with_compressor_off(clean_system):
    """2118 rose while the compressor was being stopped: the unit cannot finish it, standby clears it."""
    openamber = clean_system
    start_space_heating(openamber, tc=28.0, target=35.0)
    assert openamber.get_entity("working_mode_switch") == "Verwarmen"

    _stop_compressor_then_defrost(openamber, compressor_frequency=0.0)

    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Idle", max_s=600), (
        f"Heat/cool controller stayed in '{openamber.get_entity('state_machine_state_heat_cool')}' "
        "while 2118 was set with the compressor off"
    )
    assert openamber.get_entity("working_mode_switch") == "Standby"
    assert openamber.get_entity("defrost_running_sensor") is False, (
        "A stopped compressor is no running defrost"
    )


def test_standby_deferred_while_compressor_still_runs(clean_system):
    openamber = clean_system
    start_space_heating(openamber, tc=28.0, target=35.0)

    # The outdoor unit still runs the compressor at the defrost standby frequency.
    _stop_compressor_then_defrost(openamber, compressor_frequency=36.0)

    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Wait pump stop", max_s=1800)
    openamber.advance_time(seconds=120, step_s=5)
    assert openamber.get_entity("state_machine_state_heat_cool") == "Wait pump stop"
    assert openamber.get_entity("working_mode_switch") == "Verwarmen", (
        "Standby must not be written while the outdoor unit is defrosting"
    )

    openamber.set_binary_sensor("defrost_active_sensor", False)
    openamber.set_number("mock_compressor_frequency_override", 0.0)
    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Idle", max_s=60)
    assert openamber.get_entity("working_mode_switch") == "Standby", (
        "Standby must be written once the defrost has ended"
    )


def test_defrost_wait_before_standby_is_capped(clean_system):
    openamber = clean_system
    start_space_heating(openamber, tc=28.0, target=35.0)

    _stop_compressor_then_defrost(openamber, compressor_frequency=36.0)
    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Wait pump stop", max_s=1800)

    assert advance_until_state(
        openamber, "state_machine_state_heat_cool", "Idle", max_s=DEFROST_STANDBY_WAIT_MAX_S + 60
    ), "The wait for the defrost to end before standby must be capped"
    assert openamber.get_entity("working_mode_switch") == "Standby"
