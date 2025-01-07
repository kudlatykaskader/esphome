import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import button

from . import DieselHeaterBLE, CONF_HEATER_ID, diesel_heater_ble_ns

CONF_LEVEL_UP_ID = "level_up"
CONF_LEVEL_DOWN_ID = "level_down"
CONF_TEMP_UP_ID = "temp_up"
CONF_TEMP_DOWN_ID = "temp_down"


CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(CONF_HEATER_ID): cv.use_id(DieselHeaterBLE),
            cv.Optional(CONF_LEVEL_UP_ID): button.button_schema(
                diesel_heater_ble_ns.class_("LevelUpButton", button.Button),
                icon="mdi:arrow-up",
            ),
            cv.Optional(CONF_LEVEL_DOWN_ID): button.button_schema(
                diesel_heater_ble_ns.class_("LevelDownButton", button.Button),
                icon="mdi:arrow-down",
            ),
            cv.Optional(CONF_TEMP_UP_ID): button.button_schema(
                diesel_heater_ble_ns.class_("TempUpButton", button.Button),
                icon="mdi:arrow-up",
            ),
            cv.Optional(CONF_TEMP_DOWN_ID): button.button_schema(
                diesel_heater_ble_ns.class_("TempDownButton", button.Button),
                icon="mdi:arrow-down",
            ),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_HEATER_ID])

    for var in [CONF_LEVEL_UP_ID, CONF_LEVEL_DOWN_ID, CONF_TEMP_UP_ID, CONF_TEMP_DOWN_ID]:
        if conf := config.get(var):
            sw_var = await button.new_button(conf)
            await cg.register_parented(sw_var, parent)
            cg.add(getattr(parent, f"set_{var}_button")(sw_var))