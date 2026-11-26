"""Replay of two defrosts measured on 2026-11-25 (Amber, Ta about 0 °C), see field_replay.py.

- 09:27: Tuo 7.3 K below Tui right after the defrost, then a full recovery. Heating continued in the field too.
- 10:22: Tuo 8.0 K below Tui five seconds after the defrost; in the field the |Tuo - Tui| > 8 K check stopped
  heating (state Idle at 10:26:21). With the direction-aware, delayed check heating must continue.
"""

from field_replay import replay_frames
from helpers import set_outside_temperature, start_space_heating
from temperature_profiles import no_safety_stop, play


def _start(openamber):
    # Ta above the defrost backup heater boost temperature; the setpoint stays above the recorded Tc, so the
    # replay is about the Tuo/Tui checks and not about overshoot stops.
    set_outside_temperature(openamber, 5.0)
    start_space_heating(openamber, tc=30.0, target=38.0)


def _cleanup(openamber):
    openamber.set_switch("heat_demand_switch", False)
    openamber.set_switch("reset_start_timeout_errors_switch", True)
    openamber.step(ms=50)


def _replay(openamber, name, hold_s):
    states = []

    def check(oa, frame):
        no_safety_stop()(oa, frame)
        states.append(oa.get_entity("state_machine_state_heat_cool"))

    play(openamber, replay_frames(name, hold_s=hold_s), check)
    return states


def test_replay_defrost_0927_heating_continues(clean_system):
    openamber = clean_system
    try:
        _start(openamber)
        states = _replay(openamber, "defrost_2026-11-25_0927.csv", hold_s=60)
        assert "Defrosting" in states, f"The replay never reached the defrost (states: {sorted(set(states))})"
        assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"
    finally:
        _cleanup(openamber)


def test_replay_defrost_1022_no_safety_stop(clean_system):
    """The field incident: the recorded series ends at the stop (10:26:21); hold those values for 2 more minutes."""
    openamber = clean_system
    try:
        _start(openamber)
        states = _replay(openamber, "defrost_2026-11-25_1022.csv", hold_s=120)
        assert "Defrosting" in states, f"The replay never reached the defrost (states: {sorted(set(states))})"
        assert "Wait for state switch" in states, "The recovery after the defrost was not reached"
    finally:
        _cleanup(openamber)
