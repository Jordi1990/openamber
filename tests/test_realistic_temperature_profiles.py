"""Flow tests with realistic Tuo/Tui/Tc profiles (see temperature_profiles.py).

The other tests keep Tuo and Tui at a fixed, small difference, so they never see what the plate heat exchanger
does around a defrost. These tests play the measured shape into the simulator every 15 s while the firmware runs:
- the defrost dip (Tuo about 8 K below Tui at the end of the defrost) and the cold return water after it must not
  cause a safety stop (field case 2026-11-25 10:26, before the check became direction-aware);
- a colder night with a deeper dip must not either;
- a real, sustained excess of Tuo over Tui during heating must still stop the unit with TO08.
"""

from helpers import set_outside_temperature, start_space_heating
from temperature_profiles import READ_INTERVAL_S, Frame, defrost_profile, no_safety_stop, play

TARGET = 38.0


def _start(openamber):
    # Above the defrost backup heater boost temperature, so the recovery runs on the compressor alone.
    set_outside_temperature(openamber, 5.0)
    start_space_heating(openamber, tc=30.0, target=TARGET)


def _cleanup(openamber):
    openamber.set_switch("heat_demand_switch", False)
    openamber.set_switch("reset_start_timeout_errors_switch", True)
    openamber.step(ms=50)


def test_defrost_dip_and_cold_return_do_not_stop_heating(clean_system):
    """Field shape of 2026-11-25: Tuo 24.8 / Tui 32.7 at the end of the defrost, Tui about 3 K lower after it."""
    openamber = clean_system
    try:
        _start(openamber)
        frames = defrost_profile(tui_start=33.0, end_delta_k=-8.5, cold_return_k=3.0)
        states = []

        def check(oa, frame):
            no_safety_stop()(oa, frame)
            states.append(oa.get_entity("state_machine_state_heat_cool"))

        play(openamber, frames, check)

        assert "Defrosting" in states, f"The profile never reached the defrost (states: {sorted(set(states))})"
        assert "Wait for state switch" in states, "The recovery after the defrost was not reached"
        assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running", (
            f"Heating must continue after the recovery (state: {openamber.get_entity('state_machine_state_heat_cool')})"
        )
    finally:
        _cleanup(openamber)


def test_deep_defrost_dip_on_a_cold_night_does_not_stop_heating(clean_system):
    """A longer defrost with a deeper dip (Tuo 12 K below Tui) and more cold return water."""
    openamber = clean_system
    try:
        _start(openamber)
        frames = defrost_profile(tui_start=30.0, defrost_s=360, end_delta_k=-12.0, cold_return_k=5.0)
        play(openamber, frames, no_safety_stop())
        assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"
    finally:
        _cleanup(openamber)


def test_sustained_excess_while_heating_still_stops_with_to08(clean_system):
    """Flow drops while heating: Tuo rises to 10 K above Tui and stays there. After 60 s the unit stops with TO08."""
    openamber = clean_system
    try:
        _start(openamber)
        tui = 30.0
        frames = []
        for i in range(10):
            # 4 K, 7 K, then 10 K from the third reading (t = 30 s) on.
            delta = min(10.0, 4.0 + 3.0 * i)
            frames.append(Frame(t_s=i * READ_INTERVAL_S, tuo=tui + delta, tui=tui, tc=min(tui + delta, 37.0),
                                label="flow restricted"))

        stopped_at = []

        def check(oa, frame):
            if oa.get_entity("error_water_delta_t_too_high") and not stopped_at:
                stopped_at.append(frame.t_s)

        play(openamber, frames, check)

        assert stopped_at, "A sustained 10 K excess must stop the unit"
        # Above 8 K from t = 30 s: the stop follows after the 60 s delay, seen at the reading at 90 or 105 s.
        assert 90 <= stopped_at[0] <= 105, f"Stop came at t={stopped_at[0]} s, expected 90-105 s"
        assert openamber.get_entity("state_machine_state_heat_cool") == "Idle"
    finally:
        _cleanup(openamber)
