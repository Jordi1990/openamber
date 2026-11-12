"""UART wrapper that appends a padding byte after Modbus frames to selected addresses.

The outdoor unit (STM8, 19200 8E1) discards the whole frame when any byte has a parity error.
On RS485 adapters with automatic direction control the very last bits of a frame are the most
fragile: frames whose final CRC byte has bit 7 = 0 and an odd number of 1 bits (parity bit = 1)
go unanswered far more often. The outdoor unit reads the CRC at a fixed position (function 3/6)
or derives it from the byte count (function 16) and ignores trailing bytes, so sending one extra
byte after the CRC moves the fragile end-of-frame to a byte that is never checked.
See issue #322.
"""

import esphome.codegen as cg
from esphome.components import uart
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_UART_ID

DEPENDENCIES = ["uart"]

CONF_ADDRESSES = "addresses"
CONF_PAD_BYTE = "pad_byte"

modbus_frame_padding_ns = cg.esphome_ns.namespace("modbus_frame_padding")
ModbusFramePadding = modbus_frame_padding_ns.class_(
    "ModbusFramePadding", uart.UARTComponent, cg.Component
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(ModbusFramePadding),
        cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
        cv.Optional(CONF_ADDRESSES, default=[0x02]): cv.ensure_list(
            cv.int_range(min=1, max=247)
        ),
        cv.Optional(CONF_PAD_BYTE, default=0xFF): cv.hex_uint8_t,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_UART_ID])
    var = cg.new_Pvariable(config[CONF_ID], parent)
    await cg.register_component(var, config)
    for address in config[CONF_ADDRESSES]:
        cg.add(var.add_address(address))
    cg.add(var.set_pad_byte(config[CONF_PAD_BYTE]))
