"""Discharge side limiter: stay below the outdoor unit's own Pd/Td protection.

Hold the compressor level from Pd 24 bar / Td 90°C, step down one level per 60 s from
Pd 25 bar / Td 93°C, and release below Pd 23 bar and Td 87°C.
"""

from helpers import start_space_heating


def _level(openamber):
    return int(openamber.get_entity("compressor_control_select") or 0)


def test_step_down_on_high_discharge_pressure(clean_system):
    openamber = clean_system
    start_space_heating(openamber, tc=25.0, target=40.0)
    openamber.advance_time(seconds=600, step_s=10)
    openamber.step(ms=50)
    start_level = _level(openamber)
    assert start_level >= 3, f"Expected the compressor to have ramped up, got level {start_level}"

    openamber.set_sensor("discharge_pressure_pd", 25.5)
    openamber.advance_time(seconds=130, step_s=5)
    openamber.step(ms=50)

    assert openamber.get_entity("discharge_side_limit_active_sensor") is True
    assert _level(openamber) <= start_level - 2, (
        f"Expected two step-downs in 130 s from level {start_level}, got {_level(openamber)}"
    )


def test_hold_and_release(clean_system):
    openamber = clean_system
    start_space_heating(openamber, tc=25.0, target=40.0)

    openamber.set_sensor("discharge_temperature_td", 91.0)
    openamber.step(ms=50)
    held_level = _level(openamber)
    openamber.advance_time(seconds=900, step_s=10)
    openamber.step(ms=50)
    assert _level(openamber) <= held_level, "Close to the Td limit the level must not increase"
    assert openamber.get_entity("discharge_side_limit_active_sensor") is True

    # Between hold and release thresholds the limiter stays active.
    openamber.set_sensor("discharge_temperature_td", 88.0)
    openamber.advance_time(seconds=20, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity("discharge_side_limit_active_sensor") is True

    openamber.set_sensor("discharge_temperature_td", 80.0)
    openamber.advance_time(seconds=20, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity("discharge_side_limit_active_sensor") is False


def test_disabled_switch(clean_system):
    openamber = clean_system
    openamber.set_switch("discharge_side_limit_enabled", False)
    start_space_heating(openamber, tc=25.0, target=40.0)
    openamber.set_sensor("discharge_pressure_pd", 25.5)
    openamber.advance_time(seconds=130, step_s=5)
    openamber.step(ms=50)
    assert openamber.get_entity("discharge_side_limit_active_sensor") is not True
    openamber.set_switch("discharge_side_limit_enabled", True)
