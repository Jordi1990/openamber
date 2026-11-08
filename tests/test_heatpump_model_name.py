"""Automated tests for the heat pump model name derived from the model code in register 3257."""

import pytest


@pytest.mark.parametrize(
    "code, name",
    [
        (140, "Amber 65"),
        (150, "Amber 95"),
        (15, "Amber 120"),
        (145, "HP-S 95"),
        (160, "HP-S 95"),
        (999, "Onbekend model"),
    ],
)
def test_model_code_maps_to_model_name(clean_system, code, name):
    """Each known model code shows its model name, other codes show an unknown model."""
    openamber = clean_system
    openamber.set_sensor("model_identification_1", code)
    # The model name text sensor updates once a minute.
    openamber.advance_time(seconds=65, step_s=5)

    assert openamber.get_entity("heatpump_model_name") == name
