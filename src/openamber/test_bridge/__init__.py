import os

# ESPHome 2026.10 precompiled header pulls in namespace esphome::time (via datetime
# entities), which breaks the unqualified time() call in logger_host.cpp.
# Ensure virtual display / host builds always disable PCH.
os.environ["ESPHOME_PCH_ENABLE"] = "0"

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
DROPDOWN_TYPES = {"dropdown"}


def _collect_widgets(obj, result, dropdown_result, excluded_ids):
    if isinstance(obj, dict):
        for k, v in obj.items():
            if k in SUPPORTED_TYPES and isinstance(v, dict):
                if "id" in v and v["id"]:
                    val = str(getattr(v["id"], "id", v["id"]))
                    if val not in excluded_ids:
                        result.add(val)
                if "widgets" in v:
                    _collect_widgets(v["widgets"], result, dropdown_result, excluded_ids)
            elif k in DROPDOWN_TYPES and isinstance(v, dict):
                if "id" in v and v["id"]:
                    val = str(getattr(v["id"], "id", v["id"]))
                    if val not in excluded_ids:
                        dropdown_result.add(val)
                if "widgets" in v:
                    _collect_widgets(v["widgets"], result, dropdown_result, excluded_ids)
            elif k == "widgets" and isinstance(v, list):
                _collect_widgets(v, result, dropdown_result, excluded_ids)
            elif isinstance(v, (dict, list)):
                if k not in (
                    "on_click", "on_press", "on_release", "on_value",
                    "on_short_click", "on_long_press", "on_idle",
                ):
                    _collect_widgets(v, result, dropdown_result, excluded_ids)
    elif isinstance(obj, list):
        for item in obj:
            _collect_widgets(item, result, dropdown_result, excluded_ids)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_port(config[CONF_PORT]))

    # Register LVGL widgets
    widgets_to_register = set()
    dropdowns_to_register = set()
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
        _collect_widgets(lvgl_conf, widgets_to_register, dropdowns_to_register, excluded_ids)

    for widget_id in sorted(widgets_to_register):
        cg.add(var.register_widget(widget_id, cg.RawExpression(f"&{widget_id}")))

    for widget_id in sorted(dropdowns_to_register):
        cg.add(var.register_obj_getter(widget_id, cg.RawExpression(f"[=]() -> lv_obj_t* {{ return {widget_id} ? {widget_id}->obj : nullptr; }}")))

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

    # Register time entities
    for dt in CORE.config.get("datetime", []):
        if isinstance(dt, dict) and "id" in dt and dt["id"]:
            if str(dt.get("type", "")).upper() == "TIME":
                dt_id = str(getattr(dt["id"], "id", dt["id"]))
                cg.add(var.register_time_entity(dt_id, cg.RawExpression(dt_id)))

    # Register text sensors
    for ts in CORE.config.get("text_sensor", []):
        if isinstance(ts, dict) and "id" in ts and ts["id"]:
            ts_id = str(getattr(ts["id"], "id", ts["id"]))
            cg.add(var.register_text_sensor(ts_id, cg.RawExpression(ts_id)))

    # Register EEBUS child entities
    eebus_conf = CORE.config.get("eebus")
    if isinstance(eebus_conf, dict):
        if "status" in eebus_conf and isinstance(eebus_conf["status"], dict) and "id" in eebus_conf["status"]:
            sid = str(getattr(eebus_conf["status"]["id"], "id", eebus_conf["status"]["id"]))
            cg.add(var.register_text_sensor(sid, cg.RawExpression(sid)))
        if "last_action" in eebus_conf and isinstance(eebus_conf["last_action"], dict) and "id" in eebus_conf["last_action"]:
            sid = str(getattr(eebus_conf["last_action"]["id"], "id", eebus_conf["last_action"]["id"]))
            cg.add(var.register_text_sensor(sid, cg.RawExpression(sid)))
        if "connected" in eebus_conf and isinstance(eebus_conf["connected"], dict) and "id" in eebus_conf["connected"]:
            sid = str(getattr(eebus_conf["connected"]["id"], "id", eebus_conf["connected"]["id"]))
            cg.add(var.register_binary_sensor(sid, cg.RawExpression(sid)))
        if "power_limit" in eebus_conf and isinstance(eebus_conf["power_limit"], dict) and "id" in eebus_conf["power_limit"]:
            sid = str(getattr(eebus_conf["power_limit"]["id"], "id", eebus_conf["power_limit"]["id"]))
            cg.add(var.register_sensor(sid, cg.RawExpression(sid)))
        if "ski" in eebus_conf and isinstance(eebus_conf["ski"], dict) and "id" in eebus_conf["ski"]:
            sid = str(getattr(eebus_conf["ski"]["id"], "id", eebus_conf["ski"]["id"]))
            cg.add(var.register_text_sensor(sid, cg.RawExpression(sid)))
        if "ems" in eebus_conf and isinstance(eebus_conf["ems"], dict) and "id" in eebus_conf["ems"]:
            sid = str(getattr(eebus_conf["ems"]["id"], "id", eebus_conf["ems"]["id"]))
            cg.add(var.register_text_sensor(sid, cg.RawExpression(sid)))

    cg.add_define("TEST_BRIDGE_ENABLED")
