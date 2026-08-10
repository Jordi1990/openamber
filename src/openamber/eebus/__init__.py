import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import esp32
from esphome.const import CONF_ID

CONF_DEVICE_SKU = "device_sku"
CONF_BRAND = "brand"
CONF_MODEL = "model"
CONF_FAILSAFE_DURATION = "failsafe_duration"
CONF_READ_POWER = "read_power"
CONF_READ_DHW_TEMP = "read_dhw_temp"
CONF_APPLY_OPTIONAL = "apply_optional"
CONF_APPLY_DIM = "apply_dim"

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
            cv.Optional(CONF_APPLY_OPTIONAL): cv.returning_lambda,
            cv.Optional(CONF_APPLY_DIM): cv.returning_lambda,
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

    var = cg.new_Pvariable(config[CONF_ID])
    cg.add(var.set_device_sku(config[CONF_DEVICE_SKU]))
    cg.add(var.set_brand(config[CONF_BRAND]))
    cg.add(var.set_model(config[CONF_MODEL]))
    cg.add(var.set_failsafe_duration(config[CONF_FAILSAFE_DURATION].total_seconds))

    for key, args, return_type in (
        (CONF_READ_POWER, [], float),
        (CONF_READ_DHW_TEMP, [], float),
        (CONF_APPLY_OPTIONAL, [(bool, "value")], bool),
        (CONF_APPLY_DIM, [(bool, "value")], bool),
    ):
        if key in config:
            lamb = await cg.process_lambda(config[key], args, return_type=return_type)
            setter = {
                CONF_READ_POWER: var.set_read_power,
                CONF_READ_DHW_TEMP: var.set_read_dhw_temp,
                CONF_APPLY_OPTIONAL: var.set_apply_optional,
                CONF_APPLY_DIM: var.set_apply_dim,
            }[key]
            cg.add(setter(lamb))

    await cg.register_component(var, config)
