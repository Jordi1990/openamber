"""Automated tests for the averaged outside temperature used by the heating curves.

The average is an exponential_moving_average sensor (time constant 60 min, time_weighting: previous) on a copy
of Ta that drops values during a defrost and for 5 min after it. Its output is throttled to once a minute.
The outdoor unit blows warm air past the Ta sensor at the end of a defrost cycle. The average must ignore that
spike, while still following real outside temperature changes slowly.
"""

import math

import pytest

from helpers import set_outside_temperature

AVERAGE = "temperature_outside_ta_avg_1h"
TIME_CONSTANT_S = 3600.0
DEFROST_HOLD_S = 300
# The published average can be up to one throttle period old.
THROTTLE_S = 60


def average(openamber):
    """Return the averaged Ta used by the heating curves."""
    return float(openamber.get_entity(AVERAGE))


def test_average_follows_outside_temperature_with_one_hour_time_constant(clean_system):
    """After one time constant about 63% of a Ta step is reflected in the average."""
    openamber = clean_system
    set_outside_temperature(openamber, 5.0)

    openamber.set_sensor("temperature_outside_ta", 0.0)
    openamber.advance_time(seconds=TIME_CONSTANT_S, step_s=30)
    openamber.step(ms=50)

    # Between the value of one throttle period ago and the current one.
    newest = 5.0 * math.exp(-1.0)
    oldest = 5.0 * math.exp(-(TIME_CONSTANT_S - THROTTLE_S) / TIME_CONSTANT_S)
    assert newest - 0.05 <= average(openamber) <= oldest + 0.05


def test_heating_curve_uses_the_average_instead_of_raw_outside_temperature(clean_system):
    """The heating curve setpoint follows the average, so a short raw Ta jump barely moves it."""
    openamber = clean_system
    set_outside_temperature(openamber, 5.0)
    openamber.advance_time(seconds=10, step_s=5)
    assert float(openamber.get_entity("heat_curve_calculated_setpoint")) == pytest.approx(28.0, abs=0.05)

    # A short jump in the raw value barely moves the setpoint.
    openamber.set_sensor("temperature_outside_ta", 15.0)
    openamber.advance_time(seconds=10, step_s=5)
    assert float(openamber.get_entity("heat_curve_calculated_setpoint")) == pytest.approx(28.0, abs=0.05)


def test_average_ignores_warm_air_spike_during_and_after_defrost(clean_system):
    """The average holds during defrost and for the full hold time after it, then moves again."""
    openamber = clean_system
    set_outside_temperature(openamber, 1.0)

    # The spike starts near the end of the defrost cycle and continues after it.
    openamber.set_binary_sensor("defrost_active_sensor", True)
    openamber.advance_time(seconds=10, step_s=2)
    openamber.set_sensor("temperature_outside_ta", 21.0)
    openamber.advance_time(seconds=60, step_s=5)
    assert average(openamber) == pytest.approx(1.0, abs=0.01)

    openamber.set_binary_sensor("defrost_active_sensor", False)
    openamber.advance_time(seconds=DEFROST_HOLD_S - 10, step_s=5)
    assert average(openamber) == pytest.approx(1.0, abs=0.01), "Average must hold until the hold time ends"
    assert openamber.get_entity("temperature_outside_ta") == pytest.approx(21.0, abs=0.01)

    # The gap counts with the last Ta before the defrost (time_weighting: previous), so just after the hold
    # time the average is still 1 °C.
    openamber.advance_time(seconds=20, step_s=5)
    assert average(openamber) == pytest.approx(1.0, abs=0.01), "The hold gap must count with the Ta before the defrost"

    # After that it follows Ta again: the next published value reflects about one minute towards 21 °C.
    openamber.advance_time(seconds=THROTTLE_S + 10, step_s=5)
    moved = average(openamber)
    expected = 21.0 - 20.0 * math.exp(-THROTTLE_S / TIME_CONSTANT_S)
    assert expected - 0.15 < moved < expected + 0.15, f"Average must move again after the hold time, got {moved}"


def test_average_resumes_after_defrost_hold_time(clean_system):
    """After the hold time the average follows Ta with the normal time constant."""
    openamber = clean_system
    set_outside_temperature(openamber, 1.0)

    openamber.set_binary_sensor("defrost_active_sensor", True)
    openamber.advance_time(seconds=10, step_s=2)
    openamber.set_binary_sensor("defrost_active_sensor", False)
    openamber.set_sensor("temperature_outside_ta", 11.0)
    openamber.advance_time(seconds=DEFROST_HOLD_S + 10, step_s=5)
    openamber.advance_time(seconds=600, step_s=30)
    openamber.step(ms=50)

    # About 600 s of 11 °C after the hold time; the published value can be one throttle period old.
    newest = 11.0 - 10.0 * math.exp(-610.0 / TIME_CONSTANT_S)
    oldest = 11.0 - 10.0 * math.exp(-(600.0 - THROTTLE_S) / TIME_CONSTANT_S)
    assert oldest - 0.1 <= average(openamber) <= newest + 0.1


def test_average_is_published_at_most_once_a_minute(clean_system):
    """The output throttle limits heating curve updates (and Home Assistant traffic) to once a minute."""
    openamber = clean_system
    set_outside_temperature(openamber, 5.0)
    openamber.advance_time(seconds=THROTTLE_S + 5, step_s=5)

    openamber.set_sensor("temperature_outside_ta", 0.0)
    values = set()
    for _ in range(int(THROTTLE_S / 5) * 2):
        openamber.advance_time(seconds=5, step_s=5)
        openamber.step(ms=20)
        values.add(round(average(openamber), 4))
    # Two minutes of a falling average: at most three published values (start, +1 min, +2 min).
    assert 2 <= len(values) <= 3, f"Expected a new value at most once a minute, got {sorted(values)}"
