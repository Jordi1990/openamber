import esphome.codegen as cg
import esphome.config_validation as cv

CONF_CXX_FLAGS = "cxx_flags"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_CXX_FLAGS): cv.ensure_list(cv.string_strict),
    }
)


async def to_code(config):
    for flag in config[CONF_CXX_FLAGS]:
        cg.add_cxx_build_flag(flag)