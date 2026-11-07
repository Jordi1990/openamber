"""Automated tests for the averaged outside temperature used by the heating curves.

The outdoor unit blows warm air past the Ta sensor at the end of a defrost cycle. The average must
ignore that spike, while still following real outside temperature changes slowly.
"""

import math

import pytest

from helpers import set_outside_temperature

AVERAGE = "temperature_outside_ta_avg_1h"
TIME_CONSTANT_S = 3600.0
DEFROST_HOLD_S = 300


def average(openamber):
    return float(openamber.get_entity(AVERAGE))


def test_average_follows_outside_temperature_with_one_hour_time_constant(clean_system):
    openamber = clean_system
    set_outside_temperature(openamber, 5.0)

    openamber.set_sensor("temperature_outside_ta", 0.0)
    openamber.advance_time(seconds=TIME_CONSTANT_S, step_s=30)
    openamber.step(ms=50)

    assert average(openamber) == pytest.approx(5.0 * math.exp(-1.0), abs=0.1)


def test_heating_curve_uses_the_average_instead_of_raw_outside_temperature(clean_system):
    openamber = clean_system
    set_outside_temperature(openamber, 5.0)
    openamber.advance_time(seconds=10, step_s=5)
    assert float(openamber.get_entity("heat_curve_calculated_setpoint")) == pytest.approx(28.0, abs=0.05)

    # A short jump in the raw value barely moves the setpoint.
    openamber.set_sensor("temperature_outside_ta", 15.0)
    openamber.advance_time(seconds=10, step_s=5)
    assert float(openamber.get_entity("heat_curve_calculated_setpoint")) == pytest.approx(28.0, abs=0.05)


def test_average_ignores_warm_air_spike_during_and_after_defrost(clean_system):
    openamber = clean_system
    set_outside_temperature(openamber, 1.0)

    # The spike starts near the end of the defrost cycle and continues after it.
    openamber.set_binary_sensor("defrost_active_sensor", True)
    openamber.advance_time(seconds=10, step_s=2)
    openamber.set_sensor("temperature_outside_ta", 21.0)
    openamber.advance_time(seconds=60, step_s=5)
    assert average(openamber) == pytest.approx(1.0, abs=0.01)

    openamber.set_binary_sensor("defrost_active_sensor", False)
    openamber.advance_time(seconds=DEFROST_HOLD_S - 30, step_s=5)
    assert average(openamber) == pytest.approx(1.0, abs=0.01)
    assert openamber.get_entity("temperature_outside_ta") == pytest.approx(21.0, abs=0.01)


def test_average_resumes_after_defrost_hold_time(clean_system):
    openamber = clean_system
    set_outside_temperature(openamber, 1.0)

    openamber.set_binary_sensor("defrost_active_sensor", True)
    openamber.advance_time(seconds=10, step_s=2)
    openamber.set_binary_sensor("defrost_active_sensor", False)
    openamber.set_sensor("temperature_outside_ta", 11.0)
    openamber.advance_time(seconds=DEFROST_HOLD_S + 10, step_s=5)
    openamber.advance_time(seconds=600, step_s=30)
    openamber.step(ms=50)

    expected = 11.0 - 10.0 * math.exp(-600.0 / TIME_CONSTANT_S)
    assert average(openamber) == pytest.approx(expected, abs=0.1)
