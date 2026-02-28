import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import fan
from esphome.const import CONF_ID

from . import kocom_rs485_ns, KocomRS485Component

DEPENDENCIES = ["kocom_rs485"]

CONF_KOCOM_RS485_ID = "kocom_rs485_id"

KocomFan = kocom_rs485_ns.class_(
    "KocomFan", fan.Fan, cg.Component
)

CONFIG_SCHEMA = fan.fan_schema(KocomFan).extend(
    {
        cv.GenerateID(CONF_KOCOM_RS485_ID): cv.use_id(KocomRS485Component),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await fan.register_fan(var, config)

    parent = await cg.get_variable(config[CONF_KOCOM_RS485_ID])
    cg.add(var.set_parent(parent))
