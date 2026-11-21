"""Place ESPHome's generated component objects in PSRAM instead of internal RAM.

ESPHome allocates every component, entity, trigger, automation, action and
LVGL widget pointer of the configuration as a static object in main.cpp. On
the Waveshare S3 that is ~84 kB of .bss in internal SRAM, which the heap then
cannot use. This component adds a small ESP-IDF component whose linker
fragment maps the .bss of main.cpp (in ESPHome's `src` archive) to external
RAM (.ext_ram.bss).

Only use this on a device that executes from PSRAM (XIP:
CONFIG_SPIRAM_FETCH_INSTRUCTIONS + CONFIG_SPIRAM_RODATA, or
CONFIG_SPIRAM_XIP_FROM_PSRAM). Then the cache stays enabled during flash
writes (NVS, OTA), so ISRs can still reach objects in PSRAM.
"""

from pathlib import Path

from esphome.components.esp32 import add_idf_component, add_idf_sdkconfig_option
import esphome.config_validation as cv

DEPENDENCIES = ["esp32", "psram"]

CONFIG_SCHEMA = cv.Schema({})


async def to_code(config):
    add_idf_sdkconfig_option("CONFIG_SPIRAM_ALLOW_BSS_SEG_EXTERNAL_MEMORY", True)
    add_idf_component(
        name="openamber_psram_bss",
        path=str(Path(__file__).parent.resolve() / "idf"),
    )
