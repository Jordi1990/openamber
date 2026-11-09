"""Regression test: after a (re)start the pump starts directly when there is pump demand.

INITIALIZING stops the pump, and PumpController::Stop() restarts the pump interval. Without
resetting the interval afterwards the pump only started once the configured pump interval
(e.g. 60 min) had elapsed after a restart.

A real ESP boot runs OpenAmberComponent through WAIT_MODBUS_CONNECTION -> INITIALIZING.
Leaving service mode takes exactly the same path, so it is used here to re-run the boot
initialization inside the already running simulator.
"""

from helpers import advance_until_state, offset_s


def test_pump_starts_directly_after_restart_with_heat_demand(clean_system):
    openamber = clean_system

    original_interval = openamber.get_entity("pump_interval")
    try:
        openamber.set_number("pump_interval", 60.0)

        # Tc above the heating start temperature: pump demand, but no compressor start needed.
        openamber.set_climate("pid_heat_temperature_control", target_temperature=30.0)
        openamber.set_sensor("heat_cool_temperature_tc", 35.0)
        openamber.set_sensor("outlet_temperature_tuo", 35.0)
        openamber.set_sensor("inlet_temperature_tui", 33.0)

        # "Power off": go to maintenance so the main state machine has to re-initialize.
        openamber.set_switch("service_mode_enabled", True)
        assert advance_until_state(openamber, "state_machine_state_main", "Maintenance", max_s=900), (
            f"Did not reach Maintenance (state: {openamber.get_entity('state_machine_state_main')})"
        )

        # Heat demand is already present when the unit comes back (e.g. thermostat contact closed).
        openamber.set_switch("heat_demand_switch", True)
        openamber.step(ms=50)
        assert openamber.get_entity("heat_demand_active_sensor") is True

        # "Power on": boot path WAIT_MODBUS_CONNECTION -> INITIALIZING -> ... -> Heat/Cool.
        openamber.set_switch("service_mode_enabled", False)
        assert advance_until_state(openamber, "state_machine_state_main", "Heat/Cool", max_s=300, step=5), (
            f"Did not return to Heat/Cool (state: {openamber.get_entity('state_machine_state_main')})"
        )
        back_in_heat_cool = offset_s(openamber)

        # Expected: pump starts within a few controller cycles, not after the 60 min interval.
        started = advance_until_state(openamber, "state_machine_state_heat_cool", "Pump running", max_s=120, step=5)
        waited = offset_s(openamber) - back_in_heat_cool

        if not started:
            # Measure how long it actually takes, for the report.
            started_late = advance_until_state(openamber, "state_machine_state_heat_cool", "Pump running", max_s=3700, step=30)
            waited = offset_s(openamber) - back_in_heat_cool
            raise AssertionError(
                f"Pump did not start within 120s after restart despite heat demand. "
                f"Started late: {started_late}, after {waited:.0f}s (~{waited / 60:.0f} min), "
                f"state: {openamber.get_entity('state_machine_state_heat_cool')}"
            )

        assert openamber.get_entity("internal_pump_active") is True
        print(f"Pump started {waited:.0f}s after returning to Heat/Cool")
    finally:
        openamber.set_switch("service_mode_enabled", False)
        openamber.set_switch("heat_demand_switch", False)
        if original_interval is not None:
            openamber.set_number("pump_interval", float(original_interval))
        openamber.advance_time(seconds=300, step_s=10)
        openamber.step(ms=100)
