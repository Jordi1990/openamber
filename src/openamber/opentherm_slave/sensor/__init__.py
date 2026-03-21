import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_TYPE,
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_CELSIUS,
    UNIT_PERCENT,
)

from .. import CONF_OPENTHERM_SLAVE_ID, OpenthermSlaveHub

DEPENDENCIES = ["opentherm_slave"]

CONF_CH_SETPOINT = "ch_setpoint"
CONF_ROOM_SETPOINT = "room_setpoint"
CONF_ROOM_TEMP = "room_temp"
CONF_OUTSIDE_TEMP = "outside_temp"
CONF_DHW_SETPOINT = "dhw_setpoint"
CONF_MAX_REL_MOD_LEVEL = "max_rel_mod_level"
CONF_LINK_QUALITY = "link_quality"
CONF_FRAMES_RECEIVED = "frames_received"
CONF_FRAMES_SENT = "frames_sent"
CONF_ERRORS_TOTAL = "errors_total"
CONF_LAST_MESSAGE_AGE = "last_message_age"

_TEMPERATURE_DEFAULTS = dict(
    unit_of_measurement=UNIT_CELSIUS,
    accuracy_decimals=1,
    device_class=DEVICE_CLASS_TEMPERATURE,
    state_class=STATE_CLASS_MEASUREMENT,
)

SENSOR_TYPES = {
    CONF_CH_SETPOINT: ("set_ch_setpoint_sensor", _TEMPERATURE_DEFAULTS),
    CONF_ROOM_SETPOINT: ("set_room_setpoint_sensor", _TEMPERATURE_DEFAULTS),
    CONF_ROOM_TEMP: ("set_room_temp_sensor", _TEMPERATURE_DEFAULTS),
    CONF_OUTSIDE_TEMP: ("set_outside_temp_sensor", _TEMPERATURE_DEFAULTS),
    CONF_DHW_SETPOINT: ("set_dhw_setpoint_sensor", _TEMPERATURE_DEFAULTS),
    CONF_MAX_REL_MOD_LEVEL: (
        "set_max_rel_mod_level_sensor",
        dict(
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
    ),
    CONF_LINK_QUALITY: (
        "set_link_quality_sensor",
        dict(
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
    ),
    CONF_FRAMES_RECEIVED: (
        "set_frames_received_sensor",
        dict(
            accuracy_decimals=0,
            state_class=STATE_CLASS_TOTAL_INCREASING,
        ),
    ),
    CONF_FRAMES_SENT: (
        "set_frames_sent_sensor",
        dict(
            accuracy_decimals=0,
            state_class=STATE_CLASS_TOTAL_INCREASING,
        ),
    ),
    CONF_ERRORS_TOTAL: (
        "set_errors_total_sensor",
        dict(
            accuracy_decimals=0,
            state_class=STATE_CLASS_TOTAL_INCREASING,
        ),
    ),
    CONF_LAST_MESSAGE_AGE: (
        "set_last_message_age_sensor",
        dict(
            unit_of_measurement="s",
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
    ),
}

_EXTRA_KEYS = {
    cv.GenerateID(CONF_OPENTHERM_SLAVE_ID): cv.use_id(OpenthermSlaveHub),
    cv.Required(CONF_TYPE): cv.one_of(*SENSOR_TYPES, lower=True),
}


def _config_schema(config):
    type_val = cv.one_of(*SENSOR_TYPES, lower=True)(config.get(CONF_TYPE, ""))
    _, defaults = SENSOR_TYPES[type_val]
    return sensor.sensor_schema(**defaults).extend(_EXTRA_KEYS)(config)


CONFIG_SCHEMA = _config_schema


async def to_code(config):
    hub = await cg.get_variable(config[CONF_OPENTHERM_SLAVE_ID])
    sens = await sensor.new_sensor(config)
    setter = SENSOR_TYPES[config[CONF_TYPE]][0]
    cg.add(getattr(hub, setter)(sens))
