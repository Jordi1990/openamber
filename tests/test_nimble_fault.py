"""The actual F13 lambda must follow the selected room-temperature source."""
from pathlib import Path
import shutil
import subprocess

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]


def test_selected_room_sensor_fault(tmp_path):
    compiler = shutil.which("g++")
    if not compiler:
        pytest.skip("g++ is required")

    class Loader(yaml.SafeLoader):
        pass

    for tag in ("!extend", "!include", "!lambda"):
        Loader.add_constructor(tag, lambda loader, node: loader.construct_scalar(node))
    core = yaml.load((ROOT / "src/openamber/core/binary_sensors.yaml").read_text(), Loader=Loader)
    ble = yaml.load((ROOT / "src/openamber/ble/ble_package.yaml").read_text(), Loader=Loader)
    entities = {item.get("id"): item for item in core["binary_sensor"] + ble.get("binary_sensor", [])}
    fault = entities["error_flag_f13"]["lambda"]
    source = tmp_path / "fault.cpp"
    source.write_text(r'''
#include <cassert>
#include <cstdint>
#include <string>
#define id(x) x
struct {
    std::string mode = "Intern";
    std::string current_option() { return mode; }
} thermostat_mode_select;
struct {
    float state = 1;
    bool ready = true;
    bool has_state() { return ready; }
} inside_unit_status_1225_raw;
bool ble = true, fresh = true;
bool nimble_source_is_ble() { return ble; }
bool nimble_reading_fresh() { return fresh; }
bool fault() {
''' + fault + r'''
}
int main() {
    // Installed case: a wired Tr fault must not reject fresh BLE readings.
    assert(!fault());
    fresh = false;
    assert(fault());
    inside_unit_status_1225_raw.state = 0;
    assert(fault());  // Wired sensor cannot mask missing or expired BLE data.
    fresh = true;
    assert(!fault());
    ble = false;
    inside_unit_status_1225_raw.state = 1;
    assert(fault());  // Selecting Tr must reinstate its hardware fault.
    inside_unit_status_1225_raw.state = 0;
    assert(!fault());
    inside_unit_status_1225_raw.ready = false;
    assert(!fault());  // Preserve upstream behavior before the first Modbus poll.
    thermostat_mode_select.mode = "Extern";
    ble = true; fresh = false;
    assert(!fault());
    ble = false;
    inside_unit_status_1225_raw.ready = true;
    inside_unit_status_1225_raw.state = 1;
    assert(!fault());
}
''')
    binary = tmp_path / "fault-test"
    subprocess.run([compiler, "-std=c++17", str(source), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
