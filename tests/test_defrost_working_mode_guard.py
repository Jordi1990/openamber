"""Working mode writes are deferred while the outdoor unit defrosts.

The outdoor unit aborts a running defrost without its recovery phase when 3999 leaves heating,
so standby is only written once defrost_active_sensor (2118) drops.
"""

from helpers import advance_until_state, start_space_heating


def test_standby_deferred_until_defrost_ends(clean_system):
    openamber = clean_system
    start_space_heating(openamber, tc=28.0, target=35.0)
    assert openamber.get_entity("working_mode_switch") == "Verwarmen"

    # Stop the compressor through the stop delta, then report a defrost while the pump runs on.
    openamber.set_sensor("heat_cool_temperature_tc", 38.0)
    openamber.set_sensor("current_water_temperature_tc_sensor", 38.0)
    openamber.advance_time(seconds=10, step_s=1)
    openamber.step(ms=50)
    openamber.set_binary_sensor("defrost_active_sensor", True)
    openamber.set_switch("heat_demand_switch", False)
    openamber.step(ms=50)

    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Idle", max_s=1800), (
        f"Heat/cool controller never reached 'Idle' (state: {openamber.get_entity('state_machine_state_heat_cool')})"
    )
    assert openamber.get_entity("working_mode_switch") == "Verwarmen", (
        "Standby must not be written while the outdoor unit is defrosting"
    )

    openamber.set_binary_sensor("defrost_active_sensor", False)
    openamber.advance_time(seconds=10, step_s=2)
    openamber.step(ms=50)
    assert openamber.get_entity("working_mode_switch") == "Standby", (
        "Deferred standby must be written once the defrost has ended"
    )
