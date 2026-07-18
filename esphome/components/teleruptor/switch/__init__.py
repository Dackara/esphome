import esphome.config_validation as cv
from .. import (
    CONF_TELERUPTOR_ID,
    Teleruptor,
    TELERUPTOR_SWITCH_SCHEMA,
    register_teleruptor_switch,
)

CODEOWNERS = ["@Dackara"]
DEPENDENCIES = ["teleruptor"]

CONFIG_SCHEMA = TELERUPTOR_SWITCH_SCHEMA.extend(
    {
        cv.Required(CONF_TELERUPTOR_ID): cv.use_id(Teleruptor),
    }
)


async def to_code(config):
    import esphome.codegen as cg

    parent = await cg.get_variable(config[CONF_TELERUPTOR_ID])
    await register_teleruptor_switch(config, parent)
