import esphome.codegen as cg
from esphome.components import update
import esphome.config_validation as cv
from esphome.const import CONF_ID

mock_update_ns = cg.esphome_ns.namespace("mock_update")
MockUpdate = mock_update_ns.class_("MockUpdate", update.UpdateEntity, cg.Component)

CONFIG_SCHEMA = update.update_schema(MockUpdate).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = await update.new_update(config)
    await cg.register_component(var, config)
