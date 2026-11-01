"""OpenAmber caps the compressor level per outdoor temperature band.

The Ta bands are fixed and match the outdoor unit (firmware v1.18). The maximum level per band
(ambient_limit_*_band_*_max_level) can be changed under Geavanceerd > Compressor. The defaults are the
stock v1.18 limits in Hz, converted to OpenAmber's own frequency table.

Heating Ta 24-28°C = band 4, stock 43 Hz = level 3 in OpenAmber's table (28, 36, 43, 55, ...).
"""

import pytest

from helpers import start_space_heating

HEATING_BAND_4_DEFAULT_LEVEL = 3


@pytest.fixture(autouse=True)
def restore_limit(clean_system):
    """The simulator is shared between tests: put the edited limit back afterwards."""
    yield
    clean_system.set_number("ambient_limit_heating_band_4_max_level", HEATING_BAND_4_DEFAULT_LEVEL)


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
    assert 0 < level <= 3, f"Heating at Ta 26°C must be capped to level 3, got {level}"


def test_limit_is_configurable(clean_system):
    openamber = clean_system
    openamber.set_number("ambient_limit_heating_band_4_max_level", 5)
    _run_heating_at(openamber, 26.0)

    level = _level(openamber)
    assert 3 < level <= 5, f"A level 5 limit allows level 4-5 instead of the default level 3, got {level}"


def test_band_hysteresis(clean_system):
    openamber = clean_system
    _run_heating_at(openamber, 26.0)
    assert _level(openamber) <= 3

    # 23.5°C is below the 24°C boundary but within the 1 K hysteresis: the band holds.
    openamber.set_sensor("temperature_outside_ta", 23.5)
    openamber.advance_time(seconds=600, step_s=10)
    openamber.step(ms=50)
    assert _level(openamber) <= 3, "Within the hysteresis the previous band must hold"
