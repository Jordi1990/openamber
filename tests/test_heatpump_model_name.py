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
    """Each known model code shows its model name in the text sensor and the header badge.

    Other codes show an unknown model. The header badge has its own mapping in the UI bindings.
    """
    openamber = clean_system
    assert openamber.get_entity("modbus_inside_online") is True
    assert openamber.get_entity("modbus_outside_online") is True
    openamber.set_sensor("model_identification_1", code)
    # The model name text sensor updates once a minute.
    openamber.advance_time(seconds=65, step_s=5)

    assert openamber.get_entity("heatpump_model_name") == name
    assert openamber.get_label("header_model_badge_label") == name
