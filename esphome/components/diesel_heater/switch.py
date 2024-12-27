import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch

from . import DieselHeater, CONF_HEATER_ID, heater_ns

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(CONF_HEATER_ID): cv.use_id(DieselHeater),
            cv.Optional("power"): switch.switch_schema(
                heater_ns.class_("PowerSwitch", switch.Switch),
                icon="mdi:power",
            ),
            cv.Optional("mode"): switch.switch_schema(
                heater_ns.class_("ModeSwitch", switch.Switch),
                icon="mdi:power",
            ),
            cv.Optional("alpine"): switch.switch_schema(
                heater_ns.class_("AlpineSwitch", switch.Switch),
                icon="mdi:power",
            ),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_HEATER_ID])

    for switch_type in ["power", "mode", "alpine"]:
        if conf := config.get(switch_type):
            sw_var = await switch.new_switch(conf)
            await cg.register_parented(sw_var, parent)
            cg.add(getattr(parent, f"set_{switch_type}_switch")(sw_var))
