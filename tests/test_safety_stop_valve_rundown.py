"""The 3-way valve waits for the compressor to run down after a safety stop.

A safety stop (an active error) puts the DHW controller in idle at once, while the outdoor unit still runs the
compressor down for a while (frequency > 0). The 3-way valve must stay put until the compressor stands still, as it
does after a regular stop. FlowMonitor (helpers.py) fails the flow when the valve moves with the frequency above 0.
"""

from helpers import FlowMonitor

FREQUENCY_OVERRIDE = "mock_compressor_frequency_override"
RUN_DOWN_HZ = 25.0


def _level(openamber):
    return int(openamber.get_entity("compressor_control_select") or 0)


def test_valve_waits_for_compressor_run_down_after_error_during_dhw(clean_system):
    openamber = clean_system
    openamber.set_sensor("dhw_temperature_tw_sensor", 38.0)
    monitor = FlowMonitor(openamber, allow_short_run=True)  # the error stops the compressor at once
    assert monitor.advance_until("state_machine_state_dhw", "Compressor running", 900)
    monitor.advance(120)
    assert openamber.get_entity("three_way_valve_dhw_switch") is True

    # The outdoor unit keeps reporting a frequency while the compressor runs down after the stop.
    openamber.set_number(FREQUENCY_OVERRIDE, RUN_DOWN_HZ)
    openamber.set_binary_sensor("error_compressor_start_timeout", True)
    monitor.advance(60)
    assert _level(openamber) == 0, "The error must stop the compressor"
    assert openamber.get_entity("state_machine_state_dhw") == "Idle"
    assert openamber.get_entity("three_way_valve_dhw_switch") is True, "The valve must wait for the run-down"

    # Compressor stands still: now the valve returns to heating/cooling. The valve may move in the same 5 s step in
    # which the frequency reaches 0, which FlowMonitor cannot tell apart, so check this part in 1 s steps.
    openamber.set_number(FREQUENCY_OVERRIDE, -1)
    for _ in range(30):
        openamber.advance_time(seconds=1)
        openamber.step(ms=20)
        if openamber.get_entity("three_way_valve_heat_cool_switch") is True:
            break
        assert openamber.get_entity("three_way_valve_dhw_switch") is True
    assert openamber.get_entity("three_way_valve_heat_cool_switch") is True, "The valve must move after the run-down"
    assert openamber.get_entity("three_way_valve_dhw_switch") is False
    assert float(openamber.get_entity("current_compressor_frequency") or 0) == 0
    monitor.previous = monitor.snapshot()
    monitor.advance(120)
    assert _level(openamber) == 0, "No restart while the error is active"
