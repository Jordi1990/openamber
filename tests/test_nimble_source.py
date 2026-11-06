"""Exercise the actual YAML sensor lambdas, especially missing BLE readings."""
from pathlib import Path
import shutil
import subprocess

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]


def test_temperature_source(tmp_path):
    compiler = shutil.which("g++")
    if not compiler:
        pytest.skip("g++ is required")

    class Loader(yaml.SafeLoader):
        pass

    Loader.add_constructor("!extend", lambda loader, node: loader.construct_scalar(node))
    Loader.add_constructor("!include", lambda loader, node: loader.construct_scalar(node))
    config = yaml.load((ROOT / "src/openamber/ble/ble_package.yaml").read_text(), Loader=Loader)
    sensors = {item.get("id"): item for item in config["sensor"]}
    temperature = sensors["effective_room_temperature"]["lambda"]
    humidity = sensors["effective_room_humidity"]["lambda"]
    timeout = next(item for item in config['number'] if item['id'] == 'ble_reading_timeout')['on_value']['then'][0]['lambda']
    source = tmp_path / "source.cpp"
    source.write_text(r'''
#include <cassert>
#include <cmath>
#include <cstdint>
#define id(x) x
bool nimble_boot_ready = false;
uint32_t nimble_reading_timeout_ms = 300000;
unsigned updates = 0;
void nimble_update_source() { assert(nimble_boot_ready); }
void nimble_update_ui() { assert(nimble_boot_ready); }
struct { void update() { assert(nimble_boot_ready); updates++; } } effective_room_temperature, effective_room_humidity;
void restore_timeout(float x) {
''' + timeout + r'''
}
bool use_ble = false, fresh = false;
bool nimble_source_is_ble() { return use_ble; }
bool nimble_reading_fresh() { return fresh; }
float ble_temp_value = 19, ble_hum_value = 57;
struct { float state = 22; } room_temperature;
float temperature() {
''' + temperature + '\n}\nfloat humidity() {\n' + humidity + r'''
}
int main() {
    // Template numbers restore at priority 800; LVGL widgets exist only after 400.
    restore_timeout(7);
    assert(nimble_reading_timeout_ms == 420000 && updates == 0);
    nimble_boot_ready = true;
    restore_timeout(5);
    assert(nimble_reading_timeout_ms == 300000 && updates == 2);
    assert(temperature() == 22 && std::isnan(humidity()));
    use_ble = true;
    assert(std::isnan(temperature()) && std::isnan(humidity()));
    fresh = true;
    assert(temperature() == 19 && humidity() == 57);
    fresh = false; room_temperature.state = 0;
    assert(std::isnan(temperature()) && std::isnan(humidity()));
    use_ble = false;
    assert(temperature() == 0 && std::isnan(humidity()));
    room_temperature.state = NAN;
    assert(std::isnan(temperature()));
}
''')
    binary = tmp_path / "source-test"
    subprocess.run([compiler, "-std=c++17", str(source), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
