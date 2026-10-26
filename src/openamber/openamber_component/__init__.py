import os
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID
from esphome.core import CORE

openamber_ns = cg.esphome_ns.namespace("openamber")
OpenAmberComponent = openamber_ns.class_("OpenAmberComponent", cg.PollingComponent)


def _check_host():
    if CORE.is_host:
        os.environ["ESPHOME_PCH_ENABLE"] = "0"


CONFIG_SCHEMA = cv.All(
    cv.Schema({
        cv.GenerateID(): cv.declare_id(OpenAmberComponent),
    }).extend(cv.polling_component_schema("5s")),
    lambda config: (_check_host(), config)[1],
)


async def to_code(config):
    if CORE.is_host:
        os.environ["ESPHOME_PCH_ENABLE"] = "0"
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
