"""The PSRAM pass must never relocate hardware/network objects implicitly."""
import importlib.util
from pathlib import Path

import pytest
import esphome.codegen as cg
from esphome.core import CORE, ID

PATH = Path(__file__).resolve().parents[1] / "src/openamber/psram_static/__init__.py"
spec = importlib.util.spec_from_file_location("psram_static", PATH)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


@pytest.mark.parametrize("namespace,type_name,external", [
    ("lvgl", "lvgl::LVGLLabelUpdateAction<float>", True),
    ("template", "template_::TemplateSensor", True),
    ("esphome", "Automation<float>", True),
    ("esphome", "StatelessLambdaAction<>", True),
    ("esphome", "LvTableType", True),
    ("esphome", "esphome::ESPHomeOTAComponent", False),
    ("esphome", "UnknownFutureComponent", False),
    ("gpio", "gpio::GPIOBinarySensor", False),
    ("uart", "uart::IDFUARTComponent", False),
    ("i2c", "i2c::IDFI2CBus", False),
    ("wifi", "wifi::WiFiComponent", False),
    ("api", "api::APIServer", False),
    ("logger", "logger::Logger", False),
    ("mipi_rgb", "mipi_rgb::MipiRgb", False),
    ("gt911", "gt911::GT911Touchscreen", False),
    ("modbus_controller", "modbus_controller::ModbusController", False),
])
def test_storage_placement(namespace, type_name, external):
    original = f"alignas({type_name}) static unsigned char {namespace}__test__pstorage[sizeof({type_name})];"
    result = module.place_storage(original)
    assert ("EXT_RAM_BSS_ATTR" in result) == external
    assert result.replace("EXT_RAM_BSS_ATTR ", "") == original
    assert module.place_storage(result) == result


def test_other_statements_unchanged():
    for statement in ('#include "esp_attr.h"', "static unsigned char dma_buffer[1024];", "int x;"):
        assert module.place_storage(statement) == statement


@pytest.mark.parametrize("namespace,external", [("lvgl", True), ("uart", False)])
def test_pinned_generator_storage(namespace, external):
    """Check real placement-new output when upgrading the pinned generator."""
    CORE.reset()
    try:
        component = cg.esphome_ns.namespace(namespace).class_("TestComponent")
        cg.new_Pvariable(ID("test_component", type=component, is_declaration=True))
        storage = [str(s) for s in CORE.global_statements if "static unsigned char" in str(s)]
        assert len(storage) == 1
        relocated = module.place_storage(storage[0])
        assert ("EXT_RAM_BSS_ATTR" in relocated) == external
        assert relocated.replace("EXT_RAM_BSS_ATTR ", "") == storage[0]
    finally:
        CORE.reset()
