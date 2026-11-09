"""Shared helpers for flow tests.

They advance virtual time until the controller reaches a state, instead of assuming a fixed
timeline. The time at which the pump interval elapses depends on when the previous test last
stopped the pump, so hard-coded "900s + 130s + 190s" sequences land in different states
depending on test order.
"""


def offset_s(openamber):
    """Current virtual time offset in seconds."""
    return openamber.get_time().get("offset_ms", 0) / 1000.0


def advance_until_state(openamber, entity, wanted, max_s, step=10):
    """Advance virtual time in `step`-second increments until `entity` equals `wanted`.

    Returns True when the state was reached within `max_s` seconds.
    """
    waited = 0
    while True:
        if openamber.get_entity(entity) == wanted:
            return True
        if waited >= max_s:
            return False
        cur_step = min(step, max_s - waited)
        openamber.advance_time(seconds=cur_step, step_s=cur_step)
        openamber.step(ms=20)
        waited += cur_step


def set_outside_temperature(openamber, ta):
    """Set Ta and restart the averaged Ta used by the heating curves at the same value.

    The average is an exponential_moving_average sensor with a 60 s throttle on its output: after the reset
    the new value is published at the latest 60 s later, so this advances until it shows. Right after a
    defrost the Ta hold (5 min) drops values first, so it may take up to about 6 min.
    """
    openamber.set_switch("mock_reset_outside_temperature_average", True)
    openamber.step(ms=20)
    openamber.set_sensor("temperature_outside_ta", ta)
    openamber.step(ms=20)
    for _ in range(80):
        value = openamber.get_entity("temperature_outside_ta_avg_1h")
        if value is not None and abs(float(value) - ta) < 0.01:
            return
        openamber.advance_time(seconds=5, step_s=5)
        openamber.step(ms=20)
    raise AssertionError(f"Average Ta did not restart at {ta} (got {openamber.get_entity('temperature_outside_ta_avg_1h')})")


def start_space_heating(openamber, tc=28.0, target=35.0, use_curve=False):
    """Start space heating and advance until the heat/cool controller is in 'Compressor running'.

    use_curve=True keeps the 'Stooklijn' heat mode, so the setpoint comes from the heating curves.
    Otherwise 'Extern setpoint' with manual_setpoint = target is used.
    """
    openamber.set_select("thermostat_mode_select", "Extern")
    if not use_curve:
        openamber.set_select("heat_mode_select", "Extern setpoint")
        openamber.set_number("manual_setpoint", target)
        openamber.set_climate("pid_heat_temperature_control", target_temperature=target)
    openamber.set_select("heat_compressor_mode", "Maximaal")
    openamber.set_number("compressor_start_delta_heating", 2.0)
    openamber.set_number("backup_heater_degmin_threshold", 9999.0)
    openamber.set_sensor("heat_cool_temperature_tc", tc)
    openamber.set_sensor("outlet_temperature_tuo", tc)
    openamber.set_sensor("inlet_temperature_tui", tc - 2.0)
    openamber.set_switch("heat_demand_switch", True)
    openamber.step(ms=50)
    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Compressor running", max_s=1800), (
        f"Heating never reached 'Compressor running' (state: {openamber.get_entity('state_machine_state_heat_cool')})"
    )
