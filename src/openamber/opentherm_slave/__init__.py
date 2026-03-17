import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.const import CONF_ID
from esphome.components import sensor, binary_sensor

CODEOWNERS = ["@Jordi1990"]
AUTO_LOAD = ["sensor", "binary_sensor"]

CONF_IN_PIN = "in_pin"
CONF_OUT_PIN = "out_pin"
CONF_OPENTHERM_SLAVE_ID = "opentherm_slave_id"

CONF_FLOW_TEMP_INPUT = "flow_temp_input"
CONF_DHW_TEMP_INPUT = "dhw_temp_input"
CONF_RETURN_TEMP_INPUT = "return_temp_input"
CONF_OUTSIDE_TEMP_INPUT = "outside_temp_input"
CONF_REL_MOD_LEVEL_INPUT = "rel_mod_level_input"
CONF_CH_WATER_PRESSURE_INPUT = "ch_water_pressure_input"
CONF_MAX_CH_SETPOINT_INPUT = "max_ch_setpoint_input"

CONF_FLAME_ON_INPUT = "flame_on_input"
CONF_CH_ACTIVE_INPUT = "ch_active_input"
CONF_DHW_ACTIVE_INPUT = "dhw_active_input"
CONF_FAULT_INPUT = "fault_input"

CONF_SLAVE_MEMBER_ID = "slave_member_id"
CONF_SLAVE_FLAGS = "slave_flags"

opentherm_slave_ns = cg.esphome_ns.namespace("opentherm_slave")
OpenthermSlaveHub = opentherm_slave_ns.class_("OpenthermSlaveHub", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(OpenthermSlaveHub),
        cv.Required(CONF_IN_PIN): pins.internal_gpio_input_pin_schema,
        cv.Required(CONF_OUT_PIN): pins.internal_gpio_output_pin_schema,
        cv.Optional(CONF_FLOW_TEMP_INPUT): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_DHW_TEMP_INPUT): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_RETURN_TEMP_INPUT): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_OUTSIDE_TEMP_INPUT): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_REL_MOD_LEVEL_INPUT): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_CH_WATER_PRESSURE_INPUT): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_MAX_CH_SETPOINT_INPUT): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_FLAME_ON_INPUT): cv.use_id(binary_sensor.BinarySensor),
        cv.Optional(CONF_CH_ACTIVE_INPUT): cv.use_id(binary_sensor.BinarySensor),
        cv.Optional(CONF_DHW_ACTIVE_INPUT): cv.use_id(binary_sensor.BinarySensor),
        cv.Optional(CONF_FAULT_INPUT): cv.use_id(binary_sensor.BinarySensor),
        cv.Optional(CONF_SLAVE_MEMBER_ID, default=0): cv.uint8_t,
        cv.Optional(CONF_SLAVE_FLAGS, default=0x01): cv.uint8_t,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    in_pin = await cg.gpio_pin_expression(config[CONF_IN_PIN])
    cg.add(var.set_in_pin(in_pin))

    out_pin = await cg.gpio_pin_expression(config[CONF_OUT_PIN])
    cg.add(var.set_out_pin(out_pin))

    for key, setter in [
        (CONF_FLOW_TEMP_INPUT, "set_flow_temp_input"),
        (CONF_DHW_TEMP_INPUT, "set_dhw_temp_input"),
        (CONF_RETURN_TEMP_INPUT, "set_return_temp_input"),
        (CONF_OUTSIDE_TEMP_INPUT, "set_outside_temp_input"),
        (CONF_REL_MOD_LEVEL_INPUT, "set_rel_mod_level_input"),
        (CONF_CH_WATER_PRESSURE_INPUT, "set_ch_water_pressure_input"),
        (CONF_MAX_CH_SETPOINT_INPUT, "set_max_ch_setpoint_input"),
    ]:
        if key in config:
            sens = await cg.get_variable(config[key])
            cg.add(getattr(var, setter)(sens))

    for key, setter in [
        (CONF_FLAME_ON_INPUT, "set_flame_on_input"),
        (CONF_CH_ACTIVE_INPUT, "set_ch_active_input"),
        (CONF_DHW_ACTIVE_INPUT, "set_dhw_active_input"),
        (CONF_FAULT_INPUT, "set_fault_input"),
    ]:
        if key in config:
            sens = await cg.get_variable(config[key])
            cg.add(getattr(var, setter)(sens))

    cg.add(var.set_slave_member_id(config[CONF_SLAVE_MEMBER_ID]))
    cg.add(var.set_slave_flags(config[CONF_SLAVE_FLAGS]))
