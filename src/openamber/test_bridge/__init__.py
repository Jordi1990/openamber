import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_PORT
from esphome.core import CORE

DEPENDENCIES = ["lvgl"]
AUTO_LOAD = ["json"]

test_bridge_ns = cg.esphome_ns.namespace("test_bridge")
TestBridge = test_bridge_ns.class_("TestBridge", cg.Component)

CONF_WIDGETS = "widgets"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(TestBridge),
        cv.Optional(CONF_PORT, default=8888): cv.port,
        cv.Optional(CONF_WIDGETS): cv.ensure_list(cv.string),
    }
).extend(cv.COMPONENT_SCHEMA)

SUPPORTED_TYPES = {
    "obj", "label", "button", "arc", "bar", "slider",
    "checkbox", "switch", "roller", "textarea",
    "container", "spinner",
}


def _collect_widgets(obj, result, excluded_ids):
    if isinstance(obj, dict):
        for k, v in obj.items():
            if k in SUPPORTED_TYPES and isinstance(v, dict):
                if "id" in v and v["id"]:
                    val = str(getattr(v["id"], "id", v["id"]))
                    if val not in excluded_ids:
                        result.add(val)
                if "widgets" in v:
                    _collect_widgets(v["widgets"], result, excluded_ids)
            elif k == "widgets" and isinstance(v, list):
                _collect_widgets(v, result, excluded_ids)
            elif isinstance(v, (dict, list)):
                if k not in (
                    "on_click", "on_press", "on_release", "on_value",
                    "on_short_click", "on_long_press", "on_idle",
                ):
                    _collect_widgets(v, result, excluded_ids)
    elif isinstance(obj, list):
        for item in obj:
            _collect_widgets(item, result, excluded_ids)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_port(config[CONF_PORT]))

    # Register LVGL widgets
    widgets_to_register = set()
    if CONF_WIDGETS in config and config[CONF_WIDGETS]:
        widgets_to_register.update(config[CONF_WIDGETS])
    else:
        excluded_ids = set()
        for item in CORE.config.get("image", []):
            if isinstance(item, dict) and "id" in item:
                excluded_ids.add(str(getattr(item["id"], "id", item["id"])))
        for item in CORE.config.get("font", []):
            if isinstance(item, dict) and "id" in item:
                excluded_ids.add(str(getattr(item["id"], "id", item["id"])))

        lvgl_conf = CORE.config.get("lvgl", {})
        _collect_widgets(lvgl_conf, widgets_to_register, excluded_ids)

    for widget_id in sorted(widgets_to_register):
        cg.add(var.register_widget(widget_id, cg.RawExpression(f"&{widget_id}")))

    # Register sensors
    for s in CORE.config.get("sensor", []):
        if isinstance(s, dict) and "id" in s and s["id"]:
            s_id = str(getattr(s["id"], "id", s["id"]))
            cg.add(var.register_sensor(s_id, cg.RawExpression(s_id)))

    # Register numbers
    for n in CORE.config.get("number", []):
        if isinstance(n, dict) and "id" in n and n["id"]:
            n_id = str(getattr(n["id"], "id", n["id"]))
            cg.add(var.register_number(n_id, cg.RawExpression(n_id)))

    # Register switches
    for sw in CORE.config.get("switch", []):
        if isinstance(sw, dict) and "id" in sw and sw["id"]:
            sw_id = str(getattr(sw["id"], "id", sw["id"]))
            cg.add(var.register_switch(sw_id, cg.RawExpression(sw_id)))

    # Register binary sensors
    for bs in CORE.config.get("binary_sensor", []):
        if isinstance(bs, dict) and "id" in bs and bs["id"]:
            bs_id = str(getattr(bs["id"], "id", bs["id"]))
            cg.add(var.register_binary_sensor(bs_id, cg.RawExpression(bs_id)))

    # Register selects
    for sel in CORE.config.get("select", []):
        if isinstance(sel, dict) and "id" in sel and sel["id"]:
            sel_id = str(getattr(sel["id"], "id", sel["id"]))
            cg.add(var.register_select(sel_id, cg.RawExpression(sel_id)))

    # Register climates
    for c in CORE.config.get("climate", []):
        if isinstance(c, dict) and "id" in c and c["id"]:
            c_id = str(getattr(c["id"], "id", c["id"]))
            cg.add(var.register_climate(c_id, cg.RawExpression(c_id)))

    # Register text sensors
    for ts in CORE.config.get("text_sensor", []):
        if isinstance(ts, dict) and "id" in ts and ts["id"]:
            ts_id = str(getattr(ts["id"], "id", ts["id"]))
            cg.add(var.register_text_sensor(ts_id, cg.RawExpression(ts_id)))

    cg.add_define("TEST_BRIDGE_ENABLED")
