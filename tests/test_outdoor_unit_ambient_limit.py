"""OpenAmber caps the compressor frequency per outdoor temperature band.

The Ta bands and the maximum frequency per band are fixed: the stock v1.18 level limits (3282-3298) looked
up in the stock v1.18 frequency table. OpenAmber requests the highest level whose frequency in its own
table stays within that limit, so the cap in Hz stays the same when the frequency table changes.
A step to a band with a lower limit happens AMBIENT_LIMIT_TA_OFFSET_C (1 K) later than in the unit.

Heating: Ta 20-24°C = 55 Hz (level 4 in OpenAmber's table 28, 36, 43, 55, ...), Ta 24-28°C = 43 Hz
(level 3). The unit enters the 24-28°C band at 25°C (1 K hysteresis), OpenAmber at 26°C. Back down,
both leave it below 23°C.
"""

import pytest

from helpers import start_space_heating

# OpenAmber's heating table (WriteHeatingFrequencyTable).
HEATING_TABLE_HZ = (28, 36, 43, 55, 61, 67, 72, 79, 85, 90)


@pytest.fixture(autouse=True)
def restore_table(clean_system):
    """The simulator is shared between tests: put the edited table back afterwards."""
    yield
    for index, hz in enumerate(HEATING_TABLE_HZ, start=1):
        clean_system.set_number(f"heating_frequency_index_{index}", hz)


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
    assert 0 < level <= 3, f"Heating at Ta 27°C must be capped to 43 Hz = level 3, got {level}"


def test_cap_follows_frequency_table_not_level(clean_system):
    openamber = clean_system
    # Level 4 = 40 Hz, levels 5-10 = 55-60 Hz. The cap is in Hz, so level 4 is now allowed and level 5 is not.
    openamber.set_number("heating_frequency_index_4", 40)
    for index in range(5, 11):
        openamber.set_number(f"heating_frequency_index_{index}", 50 + index)
    _run_heating_at(openamber, 27.0)

    level = _level(openamber)
    assert level == 4, f"Level 4 runs 40 Hz and level 5 55 Hz, so the 43 Hz limit allows level 4, got {level}"


def test_cap_kicks_in_after_the_outdoor_unit(clean_system):
    openamber = clean_system
    _run_heating_at(openamber, 22.0)
    assert _level(openamber) <= 4

    # 25.5°C: the unit is already in the 24-28°C band, OpenAmber still allows 55 Hz (level 4).
    _set_ta(openamber, 25.5)
    level = _level(openamber)
    assert level == 4, f"Within the Ta offset OpenAmber must keep the 20-24°C limit (level 4), got {level}"

    # 26°C: OpenAmber follows.
    _set_ta(openamber, 26.0, seconds=180)
    level = _level(openamber)
    assert 0 < level <= 3, f"From 26°C OpenAmber must cap to 43 Hz = level 3, got {level}"


def test_band_hysteresis(clean_system):
    openamber = clean_system
    _run_heating_at(openamber, 27.0)
    assert _level(openamber) <= 3

    # 23.5°C is below the 24°C boundary but within the 1 K hysteresis: the band holds.
    _set_ta(openamber, 23.5)
    assert _level(openamber) <= 3, "Within the hysteresis the previous band must hold"

    # Below 23°C the 20-24°C limit (55 Hz = level 4) applies again, without offset on the way back.
    _set_ta(openamber, 22.5, seconds=1800)
    level = _level(openamber)
    assert level == 4, f"Below 23°C the 20-24°C limit (level 4) applies again, got {level}"


def test_outdoor_unit_limit_turned_off(clean_system):
    """OpenAmber writes 3281 = 0 at start, so only its own limit in Hz applies."""
    assert int(clean_system.get_entity("outdoor_unit_ambient_limit_control")) == 0


def test_compressor_stops_outside_operating_range(clean_system):
    openamber = clean_system
    _run_heating_at(openamber, 5.0)
    assert _level(openamber) > 0

    # Below -25°C (1 K hysteresis) heating is outside the operating range: the compressor stops at once.
    _set_ta(openamber, -27.0, seconds=20)
    assert _level(openamber) == 0, "Below the low ambient limit the compressor must stay off"

    # It restarts only 4 K + 1 K above the limit, from -20°C.
    _set_ta(openamber, -21.0, seconds=60)
    assert _level(openamber) == 0, "Below -20°C the compressor must stay off"
    _set_ta(openamber, -19.5, seconds=60)
    assert _level(openamber) > 0, "From -20°C the compressor may run again"
