"""A new heat or cool demand starts pump P0 directly instead of waiting for the remaining pump interval.

Without demand the controller runs the pump only once per pump interval (e.g. 60 min). Before this fix a
heat or cool demand that arrived halfway such an interval also had to wait for it to elapse, so heating
could start up to a full pump interval late. Frost protection already reset the interval; heat and cool
demand now do the same.
"""

import pytest

from helpers import advance_until_state


def _run_one_pump_cycle_and_return_to_idle(openamber):
    """Run one pump cycle and stop it, so the pump interval is restarted and fully pending in Idle."""
    openamber.set_switch("heat_demand_switch", True)
    openamber.step(ms=50)
    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Pump running", max_s=3700, step=30), (
        f"Pump never started (state: {openamber.get_entity('state_machine_state_heat_cool')})"
    )

    openamber.set_switch("heat_demand_switch", False)
    openamber.step(ms=50)
    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Idle", max_s=900, step=10), (
        f"Did not return to Idle (state: {openamber.get_entity('state_machine_state_heat_cool')})"
    )

    # Stay in Idle a while: the pump must not restart on its own, the 60 min interval is pending.
    openamber.advance_time(seconds=120, step_s=10)
    openamber.step(ms=50)
    assert openamber.get_entity("state_machine_state_heat_cool") == "Idle"
    assert openamber.get_entity("internal_pump_active") is False


@pytest.mark.parametrize("demand_switch", ["heat_demand_switch", "cool_demand_switch"])
def test_new_demand_starts_pump_without_waiting_for_pump_interval(clean_system, demand_switch):
    openamber = clean_system

    original_interval = openamber.get_entity("pump_interval")
    try:
        openamber.set_select("thermostat_mode_select", "Extern")
        openamber.set_number("pump_interval", 60.0)
        openamber.set_number("pump_duration", 2.0)

        # Tc between the heating and cooling targets: pump demand, but no compressor start.
        openamber.set_climate("pid_heat_temperature_control", target_temperature=30.0)
        openamber.set_climate("pid_cool_temperature_control", target_temperature=40.0)
        openamber.set_sensor("heat_cool_temperature_tc", 35.0)
        openamber.set_sensor("outlet_temperature_tuo", 35.0)
        openamber.set_sensor("inlet_temperature_tui", 33.0)

        _run_one_pump_cycle_and_return_to_idle(openamber)

        # New demand while the 60 min pump interval is still pending.
        openamber.set_switch(demand_switch, True)
        openamber.step(ms=50)

        started = advance_until_state(openamber, "state_machine_state_heat_cool", "Pump running", max_s=60, step=5)
        assert started, (
            f"Pump did not start within 60s after a new demand via {demand_switch} "
            f"(state: {openamber.get_entity('state_machine_state_heat_cool')})"
        )
        assert openamber.get_entity("internal_pump_active") is True
    finally:
        openamber.set_switch(demand_switch, False)
        openamber.set_switch("heat_demand_switch", False)
        if original_interval is not None:
            openamber.set_number("pump_interval", float(original_interval))
        openamber.advance_time(seconds=300, step_s=10)
        openamber.step(ms=100)
