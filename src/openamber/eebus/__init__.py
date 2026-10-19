import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.core import CORE
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
CONF_DEVICE_NAME = "device_name"
CONF_DEVICE_CATEGORY = "device_category"
CONF_DEVICE_TYPE = "device_type"
CONF_ENTITY_TYPE = "entity_type"
CONF_ENABLE_OHPCF = "enable_ohpcf"
CONF_ENABLE_MDT = "enable_mdt"
CONF_FAILSAFE_DURATION = "failsafe_duration"
CONF_READ_MODEL = "read_model"
CONF_READ_POWER = "read_power"
CONF_READ_DHW_TEMP = "read_dhw_temp"
CONF_READ_POWER_ESTIMATE = "read_power_estimate"
CONF_READ_VOLTAGE = "read_voltage"
CONF_READ_CURRENT = "read_current"
CONF_READ_ENERGY = "read_energy"
CONF_IS_BOOST_ACTIVE = "is_boost_active"
CONF_IS_ENABLED = "is_enabled"
CONF_APPLY_OPTIONAL = "apply_optional"
CONF_APPLY_DIM = "apply_dim"
CONF_APPLY_LIMIT = "apply_limit"

CONF_STATUS = "status"
CONF_LAST_ACTION = "last_action"
CONF_CONNECTED = "connected"
CONF_POWER_LIMIT = "power_limit"
CONF_SKI = "ski"
CONF_EMS = "ems"

# SPINE EntityTypeType values. They are case-sensitive on the wire: eebus-go
# only matches e.g. "HeatPumpAppliance", so "heatPumpAppliance" silently breaks
# MPC (power) and LPC (limits) discovery on the EMS side.
SPINE_ENTITY_TYPES = [
    "Inverter", "Battery",
    "Compressor", "DeviceInformation", "DHWCircuit", "DHWStorage",
    "Dishwasher", "Dryer", "ElectricalImmersionHeater", "Fan",
    "GasHeatingAppliance", "Generic", "HeatingBufferStorage", "HeatingCircuit",
    "HeatingObject", "HeatingZone", "HeatPumpAppliance", "HeatSinkCircuit",
    "HeatSourceCircuit", "HeatSourceUnit", "HVACController", "HVACRoom",
    "InstantDHWHeater", "OilHeatingAppliance", "Pump", "RefrigerantCircuit",
    "SmartEnergyAppliance", "SolarDHWStorage", "SolarThermalCircuit", "SubMeterElectricity",
    "TemperatureSensor", "Washer", "BatterySystem", "ElectricityGenerationSystem",
    "ElectricityStorageSystem", "GridConnectionPointOfPremises", "Household", "PVSystem",
    "EV", "EVSE", "ChargingOutlet", "CEM",
    "PV", "PVESHybrid", "ElectricalStorage", "PVString",
    "GridGuard", "ControllableSystem",
]
_SPINE_ENTITY_TYPES_LOWER = {t.lower(): t for t in SPINE_ENTITY_TYPES}


def spine_entity_type(value):
    value = cv.string(value)
    canonical = _SPINE_ENTITY_TYPES_LOWER.get(value.lower())
    if canonical is None:
        raise cv.Invalid(f"Unknown SPINE entity type '{value}'")
    return canonical

eebus_ns = cg.esphome_ns.namespace("eebus")
EEBusComponent = eebus_ns.class_("EEBusComponent", cg.PollingComponent)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(EEBusComponent),
            cv.Optional(CONF_DEVICE_SKU, default="EEBUS-Device"): cv.string,
            cv.Optional(CONF_BRAND, default="Generic"): cv.string,
            cv.Optional(CONF_MODEL, default="Device"): cv.string,
            cv.Optional(CONF_DEVICE_NAME): cv.string,
            cv.Optional(CONF_DEVICE_CATEGORY, default="heatPump"): cv.string,
            cv.Optional(CONF_DEVICE_TYPE, default="HeatGenerationSystem"): cv.string,
            cv.Optional(CONF_ENTITY_TYPE, default="HeatPumpAppliance"): spine_entity_type,
            cv.Optional(CONF_ENABLE_OHPCF): cv.boolean,
            cv.Optional(CONF_ENABLE_MDT): cv.boolean,
            cv.Optional(CONF_FAILSAFE_DURATION, default="2h"): cv.positive_time_period,
            cv.Optional(CONF_READ_MODEL): cv.returning_lambda,
            cv.Optional(CONF_READ_POWER): cv.returning_lambda,
            cv.Optional(CONF_READ_DHW_TEMP): cv.returning_lambda,
            cv.Optional(CONF_READ_POWER_ESTIMATE): cv.returning_lambda,
            cv.Optional(CONF_READ_VOLTAGE): cv.returning_lambda,
            cv.Optional(CONF_READ_CURRENT): cv.returning_lambda,
            cv.Optional(CONF_READ_ENERGY): cv.returning_lambda,
            cv.Optional(CONF_IS_BOOST_ACTIVE): cv.returning_lambda,
            cv.Optional(CONF_IS_ENABLED): cv.returning_lambda,
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
            cv.Optional(CONF_EMS): text_sensor.text_sensor_schema(
                icon="mdi:server-network",
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
        }
    ).extend(cv.polling_component_schema("5s")),
)


def FILTER_SOURCE_FILES() -> list[str]:
    if CORE.is_esp32:
        return ["eebus_mock.cpp", "eebus_mock.h"]
    return [
        "cert.cpp",
        "cert.h",
        "eebus_component.cpp",
        "eebus_component.h",
        "eebus_lpc.cpp",
        "eebus_lpc.h",
        "eebus_measurements.h",
        "eebus_node.cpp",
        "eebus_node.h",
        "eebus_ohpcf.cpp",
        "eebus_ohpcf.h",
        "eebus_ship.cpp",
        "eebus_ship.h",
        "eebus_spine.cpp",
        "eebus_spine.h",
        "eebus_websocket.cpp",
        "eebus_websocket.h",
        "mdns.cpp",
        "mdns.h",
        "nvs_store.cpp",
        "nvs_store.h",
        "ship_listener.cpp",
        "ship_listener.h",
    ]


async def to_code(config):
    cg.add_define("USE_EEBUS")
    if CORE.is_esp32:
        # These must be set during code generation. ESPHome initializes its
        # sdkconfig map after importing external component modules, so module-level
        # calls are silently discarded.
        esp32.add_idf_sdkconfig_option("CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY", True)
        esp32.add_idf_sdkconfig_option("CONFIG_MBEDTLS_HARDWARE_AES", False)
        esp32.add_idf_sdkconfig_option("CONFIG_MBEDTLS_SSL_IN_CONTENT_LEN", 8192)
        esp32.add_idf_sdkconfig_option("CONFIG_MBEDTLS_SSL_OUT_CONTENT_LEN", 8192)
        esp32.add_idf_sdkconfig_option("CONFIG_MBEDTLS_PEM_WRITE_C", True)
        for _req in (
            "require_mbedtls_tls",
            "require_mbedtls_tls_server",
            "require_mbedtls_ecp",
            "require_mbedtls_peer_cert",
        ):
            _fn = getattr(esp32, _req, None)
            if _fn is not None:
                _fn()
    else:
        cg.add_build_flag("-DUSE_EEBUS_MOCK")

    var = cg.new_Pvariable(config[CONF_ID])
    cg.add(var.set_device_sku(config[CONF_DEVICE_SKU]))
    cg.add(var.set_brand(config[CONF_BRAND]))
    cg.add(var.set_model(config[CONF_MODEL]))
    if CONF_DEVICE_NAME in config:
        cg.add(var.set_device_name(config[CONF_DEVICE_NAME]))
    cg.add(var.set_device_category(config[CONF_DEVICE_CATEGORY]))
    cg.add(var.set_device_type(config[CONF_DEVICE_TYPE]))
    cg.add(var.set_entity_type(config[CONF_ENTITY_TYPE]))

    enable_ohpcf = config.get(CONF_ENABLE_OHPCF)
    if enable_ohpcf is None:
        enable_ohpcf = (
            CONF_APPLY_OPTIONAL in config
            or CONF_READ_POWER_ESTIMATE in config
            or config.get(CONF_DEVICE_CATEGORY) == "heatPump"
        )
    cg.add(var.set_enable_ohpcf(enable_ohpcf))

    enable_mdt = config.get(CONF_ENABLE_MDT)
    if enable_mdt is None:
        enable_mdt = CONF_READ_DHW_TEMP in config
    cg.add(var.set_enable_mdt(enable_mdt))

    cg.add(var.set_failsafe_duration(config[CONF_FAILSAFE_DURATION].total_seconds))

    for key, args, return_type in (
        (CONF_READ_POWER, [], float),
        (CONF_READ_DHW_TEMP, [], float),
        (CONF_READ_POWER_ESTIMATE, [], float),
        (CONF_READ_VOLTAGE, [], float),
        (CONF_READ_CURRENT, [], float),
        (CONF_READ_ENERGY, [], float),
        (CONF_READ_MODEL, [], cg.std_string),
        (CONF_IS_BOOST_ACTIVE, [], bool),
        (CONF_IS_ENABLED, [], bool),
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
                CONF_READ_VOLTAGE: var.set_read_voltage,
                CONF_READ_CURRENT: var.set_read_current,
                CONF_READ_ENERGY: var.set_read_energy,
                CONF_READ_MODEL: var.set_read_model,
                CONF_IS_BOOST_ACTIVE: var.set_is_boost_active,
                CONF_IS_ENABLED: var.set_is_enabled,
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

    if CONF_EMS in config:
        sens = await text_sensor.new_text_sensor(config[CONF_EMS])
        cg.add(var.set_ems_sensor(sens))

    await cg.register_component(var, config)
