import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import CONF_TYPE

from .. import CONF_OPENTHERM_SLAVE_ID, OpenthermSlaveHub

DEPENDENCIES = ["opentherm_slave"]

CONF_STATUS = "status"
CONF_LAST_ERROR = "last_error"

TEXT_SENSOR_TYPES = {
    CONF_STATUS: "set_status_text_sensor",
    CONF_LAST_ERROR: "set_last_error_text_sensor",
}

CONFIG_SCHEMA = text_sensor.text_sensor_schema().extend(
    {
        cv.GenerateID(CONF_OPENTHERM_SLAVE_ID): cv.use_id(OpenthermSlaveHub),
        cv.Required(CONF_TYPE): cv.one_of(*TEXT_SENSOR_TYPES, lower=True),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_OPENTHERM_SLAVE_ID])
    sens = await text_sensor.new_text_sensor(config)
    cg.add(getattr(hub, TEXT_SENSOR_TYPES[config[CONF_TYPE]])(sens))