"""Defrost (2118) and stopping the compressor.

The outdoor unit only runs its defrost pre-phase and the defrost itself while the compressor runs. A
compressor stop aborts it and leaves 2118 set until the working mode goes to standby. So OpenAmber does not
stop the compressor while 2118 is set, and does not wait for 2118 once the compressor has stopped anyway:
that wait never ended and blocked the pump cycles of frost protection.
"""

from helpers import advance_until_state, start_space_heating


def _stop_compressor_then_defrost(openamber, compressor_frequency):
    """Stop the compressor through the stop delta, then report a defrost while the pump runs on.

    compressor_frequency is the frequency the outdoor unit reports once 2118 is set.
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


def test_overshoot_does_not_stop_compressor_during_defrost_prephase(clean_system):
    """2118 rises while the compressor runs: an overshoot must not stop it before the defrost has finished."""
    openamber = clean_system
    start_space_heating(openamber, tc=28.0, target=35.0)
    # Past the minimum compressor on time, so the stop conditions are checked.
    openamber.advance_time(seconds=11 * 60, step_s=10)
    openamber.step(ms=50)
    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"

    try:
        # Pre-phase: 2118 set, compressor still at its normal frequency, and the temperature overshoots.
        openamber.set_number("mock_compressor_frequency_override", 70.0)
        openamber.set_binary_sensor("defrost_active_sensor", True)
        openamber.set_sensor("heat_cool_temperature_tc", 38.0)
        openamber.set_sensor("current_water_temperature_tc_sensor", 38.0)
        openamber.advance_time(seconds=120, step_s=5)
        openamber.step(ms=50)

        assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running", (
            "The compressor must keep running while 2118 is set"
        )
        assert int(openamber.get_entity("compressor_control_select") or 0) > 0

        # 2118 drops: the overshoot stops the compressor as usual.
        openamber.set_binary_sensor("defrost_active_sensor", False)
        openamber.set_number("mock_compressor_frequency_override", -1.0)
        assert advance_until_state(openamber, "state_machine_state_heat_cool", "Pump running", max_s=600), (
            f"Compressor did not stop after 2118 dropped (state: {openamber.get_entity('state_machine_state_heat_cool')})"
        )
    finally:
        openamber.set_number("mock_compressor_frequency_override", -1.0)
