"""Exercise the real scanner with SDK stubs; no radio or simulator required."""
from pathlib import Path
import shutil
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[1]


def test_nimble_scanner(tmp_path):
    compiler = shutil.which("g++")
    if not compiler:
        pytest.skip("g++ is required for the standalone scanner test")
    for header in (
        "esp_heap_caps.h", "freertos/FreeRTOS.h", "freertos/task.h", "nimble/nimble_port.h",
        "nimble/nimble_port_freertos.h", "host/ble_hs.h", "host/ble_gap.h",
    ):
        path = tmp_path / header
        path.parent.mkdir(parents=True, exist_ok=True)
        path.touch()
    binary = tmp_path / "scanner-test"
    subprocess.run([
        compiler, "-std=c++17", "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
        "-I", str(tmp_path), "-I", str(ROOT / "src/openamber/controllers"),
        str(ROOT / "tests/nimble/scanner_test.cpp"), "-o", str(binary),
    ], check=True)
    subprocess.run([str(binary)], check=True)
