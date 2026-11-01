"""OpenAmber caps the compressor frequency per outdoor temperature band.

The Ta bands are the ones the outdoor unit uses (firmware v1.18). The limit per band is a maximum
frequency in Hz (ambient_limit_*_band_*_max_frequency, editable under Geavanceerd > Compressor). The
defaults are the stock v1.18 level limits looked up in the stock frequency table, so the cap keeps
matching the original unit when OpenAmber's own frequency table is changed.

Heating Ta 24-28°C = band 4, default 43 Hz. With OpenAmber's table (28, 36, 43, 55, ...) that is level 3.
"""

import pytest

from helpers import start_space_heating

# OpenAmber's heating table (WriteHeatingFrequencyTable) and the default band 4 limit.
HEATING_TABLE_HZ = (28, 36, 43, 55, 61, 67, 72, 79, 85, 90)
HEATING_BAND_4_DEFAULT_HZ = 43


@pytest.fixture(autouse=True)
def restore_tables(clean_system):
    """The simulator is shared between tests: put the edited table and limit back afterwards."""
    yield
    for index, hz in enumerate(HEATING_TABLE_HZ, start=1):
        clean_system.set_number(f"heating_frequency_index_{index}", hz)
    clean_system.set_number("ambient_limit_heating_band_4_max_frequency", HEATING_BAND_4_DEFAULT_HZ)


def _level(openamber):
    return int(openamber.get_entity("compressor_control_select") or 0)


def _run_heating_at(openamber, ta):
    openamber.set_sensor("temperature_outside_ta", 5.0)
    start_space_heating(openamber, tc=25.0, target=40.0)
    openamber.set_sensor("temperature_outside_ta", ta)
    # Downward steps are limited to one per FREQUENCY_CHANGE_INTERVAL_DOWN_S (10 s).
    openamber.advance_time(seconds=180, step_s=5)
    openamber.step(ms=50)


def test_requested_level_capped_by_ambient_band(clean_system):
    openamber = clean_system
    _run_heating_at(openamber, 26.0)

    level = _level(openamber)
    assert 0 < level <= 3, f"Heating at Ta 26°C must be capped to 43 Hz = level 3, got {level}"


def test_cap_follows_frequency_table_not_level(clean_system):
    openamber = clean_system
    # Level 4 = 40 Hz, levels 5-10 = 55-60 Hz. The cap is in Hz, so level 4 is now allowed and level 5 is not.
    openamber.set_number("heating_frequency_index_4", 40)
    for index in range(5, 11):
        openamber.set_number(f"heating_frequency_index_{index}", 50 + index)
    _run_heating_at(openamber, 26.0)

    level = _level(openamber)
    assert level == 4, f"Level 4 runs 40 Hz and level 5 55 Hz, so the 43 Hz limit allows level 4, got {level}"


def test_limit_is_configurable(clean_system):
    openamber = clean_system
    openamber.set_number("ambient_limit_heating_band_4_max_frequency", 61)
    _run_heating_at(openamber, 26.0)

    level = _level(openamber)
    assert 3 < level <= 5, f"A 61 Hz limit allows level 4-5 instead of the default level 3, got {level}"


def test_band_hysteresis(clean_system):
    openamber = clean_system
    _run_heating_at(openamber, 26.0)
    assert _level(openamber) <= 3

    # 23.5°C is below the 24°C boundary but within the 1 K hysteresis: the 43 Hz band holds.
    openamber.set_sensor("temperature_outside_ta", 23.5)
    openamber.advance_time(seconds=600, step_s=10)
    openamber.step(ms=50)
    assert _level(openamber) <= 3, "Within the hysteresis the previous band must hold"
