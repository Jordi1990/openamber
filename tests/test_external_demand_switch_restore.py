"""The "Externe warmtevraag", "Externe warmtevraag Zone 2" and "Externe koudevraag" switches keep their
state across a reboot.

They are set from Home Assistant or another external controller. Without a restore mode ESPHome starts
them off after every reboot, so a running heat or cool demand is silently dropped until the controller
sets the switch again; Home Assistant does not re-send a state it already shows. Nothing saved yet
(a fresh install) still starts them off.
"""

import os

import pytest
import yaml

from test_entity_schema_sync import ESPHomeYamlLoader

SWITCHES_YAML = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                             "src", "openamber", "core", "switches.yaml")


@pytest.mark.parametrize("switch_id, name", [
    ("heat_demand_switch", "Externe warmtevraag"),
    ("heat_demand_zone_2_switch", "Externe warmtevraag Zone 2"),
    ("cool_demand_switch", "Externe koudevraag"),
])
def test_external_demand_switch_restores_state(switch_id, name):
    with open(SWITCHES_YAML, encoding="utf-8") as f:
        switches = yaml.load(f, Loader=ESPHomeYamlLoader)["switch"]
    entry = next(s for s in switches if s.get("id") == switch_id)
    assert entry["name"] == name
    assert entry["restore_mode"] == "RESTORE_DEFAULT_OFF"
