"""OpenAmber caps the compressor level per outdoor temperature band.

The Ta bands and the maximum level per band are fixed and match the outdoor unit (firmware v1.18,
registers 3282-3298). The unit caps by level in the frequency table OpenAmber writes, so OpenAmber uses
the same levels. A step to a band with a lower limit happens AMBIENT_LIMIT_TA_OFFSET_C (1 K) later than
in the unit, so the unit always caps first.

Heating: Ta 20-24°C = level 5, Ta 24-28°C = level 3. The unit enters the 24-28°C band at 25°C
(1 K hysteresis), OpenAmber at 26°C. Back down, both leave it below 23°C.
"""

from helpers import start_space_heating


def _level(openamber):
    return int(openamber.get_entity("compressor_control_select") or 0)


def _run_heating_at(openamber, ta):
    openamber.set_sensor("temperature_outside_ta", 5.0)
    start_space_heating(openamber, tc=25.0, target=40.0)
    openamber.set_sensor("temperature_outside_ta", ta)
    # Downward steps are limited to one per FREQUENCY_CHANGE_INTERVAL_DOWN_S (10 s).
    openamber.advance_time(seconds=180, step_s=5)
    openamber.step(ms=50)


def _set_ta(openamber, ta, seconds=600):
    openamber.set_sensor("temperature_outside_ta", ta)
    openamber.advance_time(seconds=seconds, step_s=10)
    openamber.step(ms=50)


def test_requested_level_capped_by_ambient_band(clean_system):
    openamber = clean_system
    _run_heating_at(openamber, 27.0)

    level = _level(openamber)
    assert 0 < level <= 3, f"Heating at Ta 27°C must be capped to level 3, got {level}"


def test_cap_kicks_in_after_the_outdoor_unit(clean_system):
    openamber = clean_system
    _run_heating_at(openamber, 22.0)
    assert _level(openamber) <= 5

    # 25.5°C: the unit is already in the 24-28°C band (level 3), OpenAmber still allows level 5.
    _set_ta(openamber, 25.5)
    level = _level(openamber)
    assert 3 < level <= 5, f"Within the Ta offset OpenAmber must keep the 20-24°C limit, got {level}"

    # 26°C: OpenAmber follows.
    _set_ta(openamber, 26.0, seconds=180)
    level = _level(openamber)
    assert 0 < level <= 3, f"From 26°C OpenAmber must cap to level 3, got {level}"


def test_band_hysteresis(clean_system):
    openamber = clean_system
    _run_heating_at(openamber, 27.0)
    assert _level(openamber) <= 3

    # 23.5°C is below the 24°C boundary but within the 1 K hysteresis: the band holds.
    _set_ta(openamber, 23.5)
    assert _level(openamber) <= 3, "Within the hysteresis the previous band must hold"

    # Below 23°C the unit allows level 5 again, and so does OpenAmber (no offset on the way back).
    _set_ta(openamber, 22.5, seconds=1800)
    level = _level(openamber)
    assert 3 < level <= 5, f"Below 23°C the 20-24°C limit (level 5) applies again, got {level}"
