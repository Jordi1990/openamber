"""Minimal MQTT 3.1.1 publisher (QoS 0) that only opens a connection while it has data to send.

Replaces ESPHome's `mqtt:` component for fire-and-forget telemetry. Unlike `mqtt:` it does not
create an MQTT shadow object for every entity, runs no extra FreeRTOS task and keeps no socket or
buffers allocated between publishes.
"""

from esphome import automation
import esphome.codegen as cg
from esphome.components import socket
import esphome.config_validation as cv
from esphome.const import (
    CONF_BROKER,
    CONF_CLIENT_ID,
    CONF_ID,
    CONF_PASSWORD,
    CONF_PAYLOAD,
    CONF_PORT,
    CONF_RETAIN,
    CONF_TOPIC,
    CONF_USERNAME,
)

DEPENDENCIES = ["network"]
AUTO_LOAD = ["socket", "json"]

CONF_LINGER = "linger"
CONF_CONNECT_TIMEOUT = "connect_timeout"
CONF_MAX_QUEUE_SIZE = "max_queue_size"

tiny_mqtt_ns = cg.esphome_ns.namespace("tiny_mqtt")
TinyMqttClient = tiny_mqtt_ns.class_("TinyMqttClient", cg.Component)
PublishAction = tiny_mqtt_ns.class_("PublishAction", automation.Action)
PublishJsonAction = tiny_mqtt_ns.class_("PublishJsonAction", automation.Action)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(TinyMqttClient),
            cv.Required(CONF_BROKER): socket.ipv4_host,
            cv.Optional(CONF_PORT, default=1883): cv.port,
            cv.Optional(CONF_USERNAME, default=""): cv.string,
            cv.Optional(CONF_PASSWORD, default=""): cv.string,
            cv.Optional(CONF_CLIENT_ID): cv.All(cv.string, cv.Length(max=64)),
            # Keep the session open this long after the last publish so a burst of
            # messages (e.g. the analytics batch) shares one TCP connection.
            cv.Optional(
                CONF_LINGER, default="2s"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(
                CONF_CONNECT_TIMEOUT, default="10s"
            ): cv.positive_time_period_milliseconds,
            # Upper bound for bytes waiting to be sent; publishes beyond this are dropped.
            cv.Optional(CONF_MAX_QUEUE_SIZE, default=8192): cv.int_range(
                min=512, max=65535
            ),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    socket.consume_sockets(1, "tiny_mqtt"),
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    socket.require_ipv4_resolve()

    cg.add(var.set_broker(config[CONF_BROKER], config[CONF_PORT]))
    cg.add(var.set_credentials(config[CONF_USERNAME], config[CONF_PASSWORD]))
    if CONF_CLIENT_ID in config:
        cg.add(var.set_client_id(config[CONF_CLIENT_ID]))
    cg.add(var.set_linger(config[CONF_LINGER]))
    cg.add(var.set_connect_timeout(config[CONF_CONNECT_TIMEOUT]))
    cg.add(var.set_max_queue_size(config[CONF_MAX_QUEUE_SIZE]))


PUBLISH_BASE_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.use_id(TinyMqttClient),
        cv.Required(CONF_TOPIC): cv.templatable(cv.publish_topic),
        cv.Optional(CONF_RETAIN, default=False): cv.templatable(cv.boolean),
    }
)


@automation.register_action(
    "tiny_mqtt.publish",
    PublishAction,
    PUBLISH_BASE_SCHEMA.extend(
        {cv.Required(CONF_PAYLOAD): cv.templatable(cv.mqtt_payload)}
    ),
    synchronous=True,
)
async def publish_action_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)
    cg.add(var.set_topic(await cg.templatable(config[CONF_TOPIC], args, cg.std_string)))
    cg.add(
        var.set_payload(await cg.templatable(config[CONF_PAYLOAD], args, cg.std_string))
    )
    cg.add(var.set_retain(await cg.templatable(config[CONF_RETAIN], args, cg.bool_)))
    return var


@automation.register_action(
    "tiny_mqtt.publish_json",
    PublishJsonAction,
    PUBLISH_BASE_SCHEMA.extend({cv.Required(CONF_PAYLOAD): cv.lambda_}),
    synchronous=True,
)
async def publish_json_action_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)
    cg.add(var.set_topic(await cg.templatable(config[CONF_TOPIC], args, cg.std_string)))
    lambda_ = await cg.process_lambda(
        config[CONF_PAYLOAD], args + [(cg.JsonObject, "root")], return_type=cg.void
    )
    cg.add(var.set_payload(lambda_))
    cg.add(var.set_retain(await cg.templatable(config[CONF_RETAIN], args, cg.bool_)))
    return var
