import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate
from esphome.const import CONF_ID

from . import kocom_rs485_ns, KocomRS485Component

DEPENDENCIES = ["kocom_rs485"]

CONF_KOCOM_RS485_ID = "kocom_rs485_id"
CONF_ROOM = "room"

KocomClimate = kocom_rs485_ns.class_(
    "KocomClimate", climate.Climate, cg.Component
)

CONFIG_SCHEMA = climate.CLIMATE_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(KocomClimate),
        cv.GenerateID(CONF_KOCOM_RS485_ID): cv.use_id(KocomRS485Component),
        cv.Required(CONF_ROOM): cv.int_range(min=0, max=3),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await climate.register_climate(var, config)

    parent = await cg.get_variable(config[CONF_KOCOM_RS485_ID])
    cg.add(var.set_parent(parent))
    cg.add(var.set_room(config[CONF_ROOM]))
