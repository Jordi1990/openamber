import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import CONF_TYPE

from .. import CONF_OPENTHERM_SLAVE_ID, OpenthermSlaveHub

DEPENDENCIES = ["opentherm_slave"]

CONF_LINK_ONLINE = "link_online"

BINARY_SENSOR_TYPES = {
    CONF_LINK_ONLINE: "set_link_online_binary_sensor",
}

CONFIG_SCHEMA = binary_sensor.binary_sensor_schema().extend(
    {
        cv.GenerateID(CONF_OPENTHERM_SLAVE_ID): cv.use_id(OpenthermSlaveHub),
        cv.Required(CONF_TYPE): cv.one_of(*BINARY_SENSOR_TYPES, lower=True),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_OPENTHERM_SLAVE_ID])
    sens = await binary_sensor.new_binary_sensor(config)
    cg.add(getattr(hub, BINARY_SENSOR_TYPES[config[CONF_TYPE]])(sens))
