"""Realistic water temperature profiles for flow tests.

The simulator has no thermal model: Tuo, Tui and Tc only change when a test sets them. Tests that keep them at a
fixed, comfortable difference never see what the plate heat exchanger really does, for example the dip in Tuo at
the end of a defrost. The helpers here build time series of frames with the shape measured on real units and
play them into the simulator at the rate the inside unit is read (every 15 s), while the firmware runs.

A frame holds the values that change at that moment; values left at None keep their previous state.
"""

from dataclasses import dataclass
from typing import Callable, Iterable, List, Optional

# The inside unit registers (Tuo 1202, Tui 1203) are read every 15 s.
READ_INTERVAL_S = 15
# Defrost standby frequency of the mock outdoor unit (3279); the mock reports it while 2118 is set.
DEFROST_STANDBY_HZ = 36.0


@dataclass
class Frame:
    """Inputs at time t_s (seconds from the start of the profile)."""

    t_s: float
    tuo: Optional[float] = None
    tui: Optional[float] = None
    tc: Optional[float] = None
    # Register 2118 (defrost flag of the outdoor unit).
    defrost: Optional[bool] = None
    # Reported compressor frequency; -1 lets the mock follow the requested level (or 36 Hz during a defrost).
    frequency_hz: Optional[float] = None
    label: str = ""


def _ease(fraction: float) -> float:
    """Smooth 0..1 transition (first-order-like start and end), like a sensor in a water flow."""
    fraction = max(0.0, min(1.0, fraction))
    return fraction * fraction * (3.0 - 2.0 * fraction)


def defrost_profile(
    tui_start: float = 33.0,
    heating_delta_k: float = 4.0,
    steady_s: int = 60,
    prephase_s: int = 30,
    prephase_hz: float = 43.0,
    defrost_s: int = 210,
    end_delta_k: float = -8.5,
    tui_drift_k: float = -0.3,
    transit_s: int = 75,
    cold_return_k: float = 3.0,
    cold_return_s: int = 90,
    recovery_delta_k: float = 3.0,
    recovery_s: int = 600,
    tc_follows_tuo: bool = True,
) -> List[Frame]:
    """Heating, a defrost and the recovery, every READ_INTERVAL_S.

    Defaults follow the defrosts of 2026-11-25 (Amber, Ta about 0 °C):
    - heating: Tuo about 4 K above Tui;
    - pre-phase: 2118 set, compressor still above the standby frequency;
    - defrost: the plate heat exchanger is the evaporator, Tuo falls to 8-8.5 K below Tui, Tui hardly moves;
    - after the defrost: Tuo recovers within about 2 min, then the water cooled during the defrost comes back:
      Tui drops about 3 K below its start value and recovers slowly while Tuo stays 3-5 K above it.
    """
    frames: List[Frame] = []
    t = 0

    def add(**kwargs):
        frames.append(Frame(t_s=t, **kwargs))

    def tc_for(tuo: float) -> Optional[float]:
        return tuo if tc_follows_tuo else None

    # Steady heating.
    tuo = tui_start + heating_delta_k
    while t < steady_s:
        add(tuo=tuo, tui=tui_start, tc=tc_for(tuo), label="heating")
        t += READ_INTERVAL_S

    # Pre-phase: 2118 rises, the compressor still runs above the standby frequency.
    first = True
    while t < steady_s + prephase_s:
        add(tuo=tuo, tui=tui_start, tc=tc_for(tuo), defrost=True if first else None,
            frequency_hz=prephase_hz if first else None, label="prephase")
        first = False
        t += READ_INTERVAL_S

    # Defrost: Tuo falls towards end_delta_k below Tui.
    defrost_start = t
    tuo_start = tuo
    first = True
    while t <= defrost_start + defrost_s:
        fraction = (t - defrost_start) / defrost_s
        tui = tui_start + tui_drift_k * fraction
        tuo = tuo_start + (tui + end_delta_k - tuo_start) * _ease(fraction)
        add(tuo=round(tuo, 2), tui=round(tui, 2), tc=tc_for(round(tuo, 2)),
            frequency_hz=-1.0 if first else None, label="defrost")
        first = False
        t += READ_INTERVAL_S

    # Defrost ends at the next reading: 2118 low, Tuo still at the bottom of the dip.
    end_t = t
    tui_end = tui_start + tui_drift_k
    tuo_end = tuo
    add(tuo=round(tuo_end, 2), tui=round(tui_end, 2), tc=tc_for(round(tuo_end, 2)), defrost=False, label="defrost end")
    t += READ_INTERVAL_S

    # Recovery: Tuo climbs back above Tui; after the transit time the cooled water returns at Tui.
    while t <= end_t + recovery_s:
        since_end = t - end_t
        if since_end < transit_s:
            tui = tui_end
        elif since_end < transit_s + cold_return_s:
            tui = tui_end - cold_return_k * _ease((since_end - transit_s) / cold_return_s)
        else:
            back = (since_end - transit_s - cold_return_s) / max(1, recovery_s - transit_s - cold_return_s)
            tui = tui_end - cold_return_k + (cold_return_k - 1.0) * _ease(back)
        target_tuo = tui + recovery_delta_k
        tuo = tuo_end + (target_tuo - tuo_end) * _ease(since_end / 120.0)
        add(tuo=round(tuo, 2), tui=round(tui, 2), tc=tc_for(round(tuo, 2)), label="recovery")
        t += READ_INTERVAL_S

    return frames


def apply_frame(openamber, frame: Frame, previous: dict) -> None:
    """Publish the values of a frame that differ from what was published before."""
    values = {
        "outlet_temperature_tuo": frame.tuo,
        "inlet_temperature_tui": frame.tui,
        "heat_cool_temperature_tc": frame.tc,
        "current_water_temperature_tc_sensor": frame.tc,
    }
    for entity, value in values.items():
        if value is not None and previous.get(entity) != value:
            openamber.set_sensor(entity, value)
            previous[entity] = value
    if frame.frequency_hz is not None and previous.get("frequency") != frame.frequency_hz:
        openamber.set_number("mock_compressor_frequency_override", frame.frequency_hz)
        previous["frequency"] = frame.frequency_hz
    if frame.defrost is not None and previous.get("defrost") != frame.defrost:
        openamber.set_binary_sensor("defrost_active_sensor", frame.defrost)
        previous["defrost"] = frame.defrost


def play(openamber, frames: Iterable[Frame], check: Optional[Callable[[object, Frame], None]] = None,
         step_s: float = 5) -> None:
    """Play frames into the simulator in time order and call check(openamber, frame) after each one.

    Time advances in step_s steps between frames, so the firmware sees the values the way it would between two
    Modbus readings.
    """
    previous: dict = {}
    current_t = None
    for frame in frames:
        if current_t is not None and frame.t_s > current_t:
            openamber.advance_time(seconds=frame.t_s - current_t, step_s=step_s)
        apply_frame(openamber, frame, previous)
        openamber.step(ms=20)
        current_t = frame.t_s
        if check is not None:
            check(openamber, frame)
    # Leave the frequency override and the defrost flag as the mock defaults.
    if previous.get("frequency", -1.0) != -1.0:
        openamber.set_number("mock_compressor_frequency_override", -1.0)
    if previous.get("defrost"):
        openamber.set_binary_sensor("defrost_active_sensor", False)
    openamber.step(ms=20)


def no_safety_stop(state_entity: str = "state_machine_state_heat_cool") -> Callable[[object, Frame], None]:
    """Check for play(): the controller stays out of Idle and no error is raised."""

    def check(openamber, frame: Frame) -> None:
        state = openamber.get_entity(state_entity)
        tuo, tui = openamber.get_entity("outlet_temperature_tuo"), openamber.get_entity("inlet_temperature_tui")
        context = f"at t={frame.t_s:.0f} s ({frame.label}), Tuo {tuo}, Tui {tui}, state {state}"
        assert openamber.get_entity("error_water_delta_t_too_high") is False, f"TO08 raised {context}"
        assert openamber.get_entity("error_active") is False, f"An error is active {context}"
        assert state != "Idle", f"Safety stop {context}"

    return check
