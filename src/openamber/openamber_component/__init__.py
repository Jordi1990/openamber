import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

openamber_ns = cg.esphome_ns.namespace("openamber")
OpenAmberComponent = openamber_ns.class_("OpenAmberComponent", cg.Component)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(OpenAmberComponent),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
