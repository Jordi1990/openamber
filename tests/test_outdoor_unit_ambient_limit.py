"""OpenAmber mirrors the outdoor unit's ambient temperature limit (3281-3298).

The outdoor unit caps the requested compressor level per ambient band. OpenAmber applies the
same cap itself, so the requested level matches what the unit actually runs.
Simulator defaults are the firmware v1.10 defaults: heating Ta 24-28°C -> max level 3.
"""

from helpers import start_space_heating


def test_requested_level_capped_by_ambient_band(clean_system):
    openamber = clean_system
    openamber.set_sensor("temperature_outside_ta", 5.0)
    start_space_heating(openamber, tc=25.0, target=40.0)

    openamber.set_sensor("temperature_outside_ta", 26.0)
    # Downward steps are limited to one per FREQUENCY_CHANGE_INTERVAL_DOWN_S (10 s).
    openamber.advance_time(seconds=180, step_s=5)
    openamber.step(ms=50)

    level = int(openamber.get_entity("compressor_control_select") or 0)
    assert 0 < level <= 3, f"Heating at Ta 26°C must be capped to level 3, got {level}"


def test_band_hysteresis(clean_system):
    openamber = clean_system
    openamber.set_sensor("temperature_outside_ta", 26.0)
    start_space_heating(openamber, tc=25.0, target=40.0)
    openamber.advance_time(seconds=180, step_s=5)
    openamber.step(ms=50)
    assert int(openamber.get_entity("compressor_control_select") or 0) <= 3

    # 23.5°C is below the 24°C boundary but within the 1 K hysteresis: band stays at level 3.
    openamber.set_sensor("temperature_outside_ta", 23.5)
    openamber.advance_time(seconds=600, step_s=10)
    openamber.step(ms=50)
    assert int(openamber.get_entity("compressor_control_select") or 0) <= 3, (
        "Within the hysteresis the previous band must hold"
    )
