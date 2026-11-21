"""Every number that Home Assistant and the web server show is an input box, not a slider.

A slider in Home Assistant is easily changed by accident while scrolling past it on a phone. Internal numbers
are not shown, so they are left out.
"""

import os

import pytest
import yaml

from test_entity_schema_sync import ESPHomeYamlLoader

SRC = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src", "openamber")
NUMBER_FILES = [
    os.path.join(SRC, "core", "numbers.yaml"),
    os.path.join(SRC, "modbus", "numbers.yaml"),
]


def _exposed_numbers(path):
    with open(path, encoding="utf-8") as f:
        numbers = yaml.load(f, Loader=ESPHomeYamlLoader)["number"]
    return [n for n in numbers if str(n.get("internal", False)).lower() != "true"]


@pytest.mark.parametrize("path", NUMBER_FILES, ids=lambda p: os.path.relpath(p, SRC))
def test_exposed_numbers_use_box_mode(path):
    numbers = _exposed_numbers(path)
    assert numbers, f"No exposed numbers found in {path}"
    not_box = [n.get("id") for n in numbers if n.get("mode") != "box"]
    assert not not_box, f"Numbers without 'mode: box' (shown as a slider in Home Assistant): {not_box}"
