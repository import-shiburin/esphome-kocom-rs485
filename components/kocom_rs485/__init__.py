import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, light
from esphome.const import CONF_ID, CONF_INDEX

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["light"]

kocom_rs485_ns = cg.esphome_ns.namespace("kocom_rs485")
KocomRS485Component = kocom_rs485_ns.class_(
    "KocomRS485", cg.Component, uart.UARTDevice
)

CONF_ROOMS = "rooms"
CONF_LIGHT_COUNT = "light_count"
CONF_LIGHT_IDS = "light_ids"
CONF_PLUG_COUNT = "plug_count"

ROOM_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_INDEX): cv.int_range(min=0, max=4),
        cv.Optional(CONF_LIGHT_COUNT, default=0): cv.int_range(min=0, max=8),
        cv.Optional(CONF_LIGHT_IDS, default=[]): cv.ensure_list(
            cv.use_id(light.LightState)
        ),
        cv.Optional(CONF_PLUG_COUNT, default=0): cv.int_range(min=0, max=8),
    }
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(KocomRS485Component),
            cv.Optional(CONF_ROOMS, default=[]): cv.ensure_list(ROOM_SCHEMA),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    for room in config.get(CONF_ROOMS, []):
        idx = room[CONF_INDEX]
        cg.add(var.set_light_count(idx, room[CONF_LIGHT_COUNT]))
        cg.add(var.set_plug_count(idx, room[CONF_PLUG_COUNT]))

        for sub, light_id in enumerate(room.get(CONF_LIGHT_IDS, [])):
            light_var = await cg.get_variable(light_id)
            cg.add(var.register_light(idx, sub, light_var))
