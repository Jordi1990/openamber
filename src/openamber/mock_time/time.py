import esphome.codegen as cg
from esphome.components import time as time_
import esphome.config_validation as cv
from esphome.const import CONF_ID

mock_time_ns = cg.esphome_ns.namespace("mock_time")
MockTime = mock_time_ns.class_("MockTime", time_.RealTimeClock)

CONFIG_SCHEMA = time_.TIME_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(MockTime),
    }
).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await time_.register_time(var, config)
