import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.const import CONF_ID, CONF_PORT, CONF_VERSION

CONF_SORTING_GROUP_ID = "sorting_group_id"
CONF_SORTING_GROUPS = "sorting_groups"
CONF_SORTING_WEIGHT = "sorting_weight"
CONF_WEB_SERVER = "web_server"
CONF_WEB_SERVER_ID = "web_server_id"

web_server_ns = cg.esphome_ns.namespace("web_server")
WebServer = web_server_ns.class_("WebServer", cg.Component, cg.Controller)

sorting_group = {
    cv.Required(CONF_ID): cv.string,
    cv.Required("name"): cv.string,
    cv.Optional(CONF_SORTING_WEIGHT): cv.float_,
}

WEBSERVER_SORTING_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_WEB_SERVER): cv.Schema(
            {
                cv.Optional(CONF_WEB_SERVER_ID): cv.use_id(WebServer),
                cv.Optional(CONF_SORTING_WEIGHT): cv.float_,
                cv.Optional(CONF_SORTING_GROUP_ID): cv.string,
            }
        ),
    }
)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(WebServer),
    cv.Optional(CONF_PORT, default=80): cv.port,
    cv.Optional(CONF_VERSION, default=3): cv.positive_int,
    cv.Optional(CONF_SORTING_GROUPS): cv.ensure_list(sorting_group),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    pass

async def add_entity_config(entity, config):
    pass
