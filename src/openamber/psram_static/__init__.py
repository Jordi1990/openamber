"""Place selected main-loop objects in PSRAM on this pinned ESPHome release.

ESPHome 2026.4+ uses placement-new into static BSS, so malloc's PSRAM policy
no longer covers component/action storage. Keep the generated object lifetime
and alignment; change only the section of explicitly selected namespaces.
"""
import re

import esphome.codegen as cg
from esphome.components import esp32
import esphome.config_validation as cv
from esphome.const import __version__
from esphome.core import CORE
from esphome.coroutine import CoroPriority, coroutine_with_priority

DEPENDENCIES = ["esp32", "psram"]

# Hardware/ISR-facing namespaces (GPIO, UART, I2C, displays, touch, modbus),
# networking, logging, FreeRTOS stacks and our NimBLE state stay internal.
PSRAM_NAMESPACES = {
    "lvgl", "esphome", "template", "script", "number", "pid", "switch",
    "integration", "sensor", "font", "thermostat", "globals", "text_sensor",
    "image", "binary_sensor", "select", "climate", "output", "update",
}
MAIN_LOOP_TYPES = {
    "AndCondition", "Automation", "DelayAction", "IfAction", "NotCondition",
    "StartupTrigger", "StatelessLambdaAction", "StatelessLambdaCondition",
    "Trigger", "UpdateComponentAction", "LvDropdownType", "LvKeyboardType",
    "LvLineType", "LvTableType",
}
STORAGE = re.compile(r"^alignas\((.+)\) static unsigned char (\w+)__pstorage\[sizeof\(.+\)\];$")


def validate_version(config):
    if __version__ != "2026.10.1":
        raise cv.Invalid("psram_static requires ESPHome 2026.10.1; review code generation before upgrading")
    return config


CONFIG_SCHEMA = cv.All(cv.Schema({}), validate_version)


def place_storage(statement):
    match = STORAGE.fullmatch(statement)
    if not match:
        return statement
    namespace = match[2].split("__", 1)[0]
    if namespace not in PSRAM_NAMESPACES:
        return statement
    if namespace == "esphome" and match[1].split("<", 1)[0] not in MAIN_LOOP_TYPES:
        return statement
    return statement.replace("static unsigned char", "static EXT_RAM_BSS_ATTR unsigned char", 1)


@coroutine_with_priority(CoroPriority.FINAL - 10)
async def to_code(config):
    esp32.add_idf_sdkconfig_option("CONFIG_SPIRAM_ALLOW_BSS_SEG_EXTERNAL_MEMORY", True)
    count = 0
    for index, statement in enumerate(CORE.global_statements):
        if isinstance(statement, cg.RawStatement):
            original = str(statement)
            updated = place_storage(original)
            if updated != original:
                CORE.global_statements[index] = cg.RawStatement(updated)
                count += 1
    if count == 0:
        raise ValueError("No static component storage matched; refusing an unverified internal-RAM build")
    CORE.global_statements.insert(0, cg.RawStatement('#include "esp_attr.h"'))
