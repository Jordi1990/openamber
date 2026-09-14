import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import esp32, binary_sensor, sensor, text_sensor
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_CONNECTIVITY,
    DEVICE_CLASS_POWER,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    UNIT_WATT,
)

CONF_DEVICE_SKU = "device_sku"
CONF_BRAND = "brand"
CONF_MODEL = "model"
CONF_FAILSAFE_DURATION = "failsafe_duration"
CONF_READ_POWER = "read_power"
CONF_READ_DHW_TEMP = "read_dhw_temp"
CONF_READ_POWER_ESTIMATE = "read_power_estimate"
CONF_IS_BOOST_ACTIVE = "is_boost_active"
CONF_APPLY_OPTIONAL = "apply_optional"
CONF_APPLY_DIM = "apply_dim"
CONF_APPLY_LIMIT = "apply_limit"

CONF_STATUS = "status"
CONF_LAST_ACTION = "last_action"
CONF_CONNECTED = "connected"
CONF_POWER_LIMIT = "power_limit"
CONF_SKI = "ski"

eebus_ns = cg.esphome_ns.namespace("openamber_eebus")
EEBusComponent = eebus_ns.class_("EEBusComponent", cg.PollingComponent)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(EEBusComponent),
            cv.Optional(CONF_DEVICE_SKU, default="OpenAmber-ESP32"): cv.string,
            cv.Optional(CONF_BRAND, default="OpenAmber"): cv.string,
            cv.Optional(CONF_MODEL, default="Amber"): cv.string,
            cv.Optional(CONF_FAILSAFE_DURATION, default="2h"): cv.positive_time_period,
            cv.Optional(CONF_READ_POWER): cv.returning_lambda,
            cv.Optional(CONF_READ_DHW_TEMP): cv.returning_lambda,
            cv.Optional(CONF_READ_POWER_ESTIMATE): cv.returning_lambda,
            cv.Optional(CONF_IS_BOOST_ACTIVE): cv.returning_lambda,
            cv.Optional(CONF_APPLY_OPTIONAL): cv.returning_lambda,
            cv.Optional(CONF_APPLY_DIM): cv.returning_lambda,
            cv.Optional(CONF_APPLY_LIMIT): cv.returning_lambda,
            cv.Optional(CONF_STATUS): text_sensor.text_sensor_schema(
                icon="mdi:ev-station",
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_LAST_ACTION): text_sensor.text_sensor_schema(
                icon="mdi:history",
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_CONNECTED): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_CONNECTIVITY,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_POWER_LIMIT): sensor.sensor_schema(
                unit_of_measurement=UNIT_WATT,
                device_class=DEVICE_CLASS_POWER,
                state_class=STATE_CLASS_MEASUREMENT,
                accuracy_decimals=0,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_SKI): text_sensor.text_sensor_schema(
                icon="mdi:certificate",
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
        }
    ).extend(cv.polling_component_schema("5s")),
)


async def to_code(config):
    # These must be set during code generation. ESPHome initializes its
    # sdkconfig map after importing external component modules, so module-level
    # calls are silently discarded.
    esp32.add_idf_sdkconfig_option("CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY", True)
    esp32.add_idf_sdkconfig_option("CONFIG_MBEDTLS_HARDWARE_AES", False)
    esp32.add_idf_sdkconfig_option("CONFIG_MBEDTLS_SSL_IN_CONTENT_LEN", 8192)
    esp32.add_idf_sdkconfig_option("CONFIG_MBEDTLS_SSL_OUT_CONTENT_LEN", 8192)
    esp32.add_idf_sdkconfig_option("CONFIG_MBEDTLS_PEM_WRITE_C", True)

    var = cg.new_Pvariable(config[CONF_ID])
    cg.add(var.set_device_sku(config[CONF_DEVICE_SKU]))
    cg.add(var.set_brand(config[CONF_BRAND]))
    cg.add(var.set_model(config[CONF_MODEL]))
    cg.add(var.set_failsafe_duration(config[CONF_FAILSAFE_DURATION].total_seconds))

    for key, args, return_type in (
        (CONF_READ_POWER, [], float),
        (CONF_READ_DHW_TEMP, [], float),
        (CONF_READ_POWER_ESTIMATE, [], float),
        (CONF_IS_BOOST_ACTIVE, [], bool),
        (CONF_APPLY_OPTIONAL, [(bool, "value")], bool),
        (CONF_APPLY_DIM, [(bool, "value")], bool),
        (CONF_APPLY_LIMIT, [(bool, "active"), (float, "limit_w")], bool),
    ):
        if key in config:
            lamb = await cg.process_lambda(config[key], args, return_type=return_type)
            setter = {
                CONF_READ_POWER: var.set_read_power,
                CONF_READ_DHW_TEMP: var.set_read_dhw_temp,
                CONF_READ_POWER_ESTIMATE: var.set_read_power_estimate,
                CONF_IS_BOOST_ACTIVE: var.set_is_boost_active,
                CONF_APPLY_OPTIONAL: var.set_apply_optional,
                CONF_APPLY_DIM: var.set_apply_dim,
                CONF_APPLY_LIMIT: var.set_apply_limit,
            }[key]
            cg.add(setter(lamb))

    if CONF_STATUS in config:
        sens = await text_sensor.new_text_sensor(config[CONF_STATUS])
        cg.add(var.set_status_sensor(sens))

    if CONF_LAST_ACTION in config:
        sens = await text_sensor.new_text_sensor(config[CONF_LAST_ACTION])
        cg.add(var.set_last_action_sensor(sens))

    if CONF_CONNECTED in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_CONNECTED])
        cg.add(var.set_connected_sensor(sens))

    if CONF_POWER_LIMIT in config:
        sens = await sensor.new_sensor(config[CONF_POWER_LIMIT])
        cg.add(var.set_power_limit_sensor(sens))

    if CONF_SKI in config:
        sens = await text_sensor.new_text_sensor(config[CONF_SKI])
        cg.add(var.set_ski_sensor(sens))

    await cg.register_component(var, config)
