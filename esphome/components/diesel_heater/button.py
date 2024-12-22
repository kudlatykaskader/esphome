# import esphome.codegen as cg
# from esphome.components import switch
# import esphome.config_validation as cv

# from . import DieselHeater, CONF_HEATER_ID

# DEPENDENCIES = []
# AUTO_LOAD = ["switch"]

# CONF_POWER = "power"
# CONF_POWER_UP = "power_up"
# CONF_POWER_DOWN = "power_down"

# heater_ns = cg.esphome_ns.namespace("diesel_heater")

# POWER_SWITCH = heater_ns.class_(CONF_POWER, switch.Switch)
# POWER_UP_SWITCH = heater_ns.class_(CONF_POWER, switch.Switch)
# POWER_DOWN_SWITCH = heater_ns.class_(CONF_POWER, switch.Switch)


# CONFIG_SCHEMA = (
#     cv.Schema(
#         {
#             cv.GenerateID(CONF_HEATER_ID): cv.use_id(DieselHeater),
#             cv.Optional(CONF_POWER): switch.switch_schema(POWER_SWITCH, icon="mdi:power"),
#             cv.Optional(CONF_POWER_UP): switch.switch_schema(POWER_UP_SWITCH, icon="mdi:power-up"),
#             cv.Optional(CONF_POWER_DOWN): switch.switch_schema(POWER_DOWN_SWITCH, icon="mdi:power-down")
#         }
#     )
#     .extend(cv.COMPONENT_SCHEMA)
# )


# async def to_code(config):
#     parent = await cg.get_variable(config[CONF_HEATER_ID])

#     for switch_type in [CONF_POWER, CONF_POWER_UP, CONF_POWER_DOWN]:
#         if conf := config.get(switch_type):
#             sw = await switch.new_switch(conf)
#             await cg.register_parented(sw, parent)
#             cg.add(getattr(parent, f"set_{switch_type}_switch")(sw))

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import button

from . import DieselHeater, CONF_HEATER_ID, heater_ns

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(CONF_HEATER_ID): cv.use_id(DieselHeater),
            cv.Optional("power_up"): button.button_schema(
                heater_ns.class_("PowerUpButton", button.Button)
            ),
            cv.Optional("power_down"): button.button_schema(
                heater_ns.class_("PowerDownButton", button.Button)
            ),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_HEATER_ID])

    for button_type in ["power_up", "power_down"]:
        if conf := config.get(button_type):
            sw_var = await button.new_button(conf)
            await cg.register_parented(sw_var, parent)
            cg.add(getattr(parent, f"set_{button_type}_button")(sw_var))
