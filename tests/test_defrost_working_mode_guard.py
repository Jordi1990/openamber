"""Standby is not written while the outdoor unit defrosts.

The outdoor unit aborts a running defrost without its recovery phase when 3999 leaves heating,
so the heat/cool controller waits in 'Wait pump stop' until defrost_active_sensor (2118) drops.
"""

from helpers import advance_until_state, start_space_heating


def _stop_compressor_then_defrost(openamber):
    """Stop the compressor through the stop delta, then report a defrost while the pump runs on.

    A defrost that starts while the compressor still runs is handled by the 'Defrosting' state,
    so 2118 is only raised once the stop sequence has begun (after the minimum compressor on time).
    """
    openamber.set_sensor("heat_cool_temperature_tc", 38.0)
    openamber.set_sensor("current_water_temperature_tc_sensor", 38.0)
    waited = 0
    while openamber.get_entity("state_machine_state_heat_cool") not in ("Wait compressor stop", "Pump running"):
        assert waited < 1800, (
            f"Compressor never stopped (state: {openamber.get_entity('state_machine_state_heat_cool')})"
        )
        openamber.advance_time(seconds=5, step_s=5)
        openamber.step(ms=20)
        waited += 5
    openamber.set_binary_sensor("defrost_active_sensor", True)
    openamber.set_switch("heat_demand_switch", False)
    openamber.step(ms=50)


def test_standby_deferred_until_defrost_ends(clean_system):
    openamber = clean_system
    start_space_heating(openamber, tc=28.0, target=35.0)
    assert openamber.get_entity("working_mode_switch") == "Verwarmen"

    _stop_compressor_then_defrost(openamber)

    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Wait pump stop", max_s=1800), (
        f"Heat/cool controller never reached 'Wait pump stop' (state: {openamber.get_entity('state_machine_state_heat_cool')})"
    )
    openamber.advance_time(seconds=120, step_s=5)
    assert openamber.get_entity("state_machine_state_heat_cool") == "Wait pump stop"
    assert openamber.get_entity("working_mode_switch") == "Verwarmen", (
        "Standby must not be written while the outdoor unit is defrosting"
    )

    openamber.set_binary_sensor("defrost_active_sensor", False)
    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Idle", max_s=60)
    assert openamber.get_entity("working_mode_switch") == "Standby", (
        "Standby must be written once the defrost has ended"
    )
