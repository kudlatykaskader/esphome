from esphome import automation, pins
import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import CONF_ID

DEPENDENCIES = []


CONF_DATA_PIN = "data_pin"

heater_ns = cg.esphome_ns.namespace("diesel_heater")
DieselHeater = heater_ns.class_("DieselHeater", sensor.Sensor, cg.Component)

CONF_HEATER_ID = "diesel_heater"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(DieselHeater),
            cv.Optional(CONF_DATA_PIN): pins.gpio_output_pin_schema,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    if CONF_DATA_PIN in config:
        data_pin_ = await cg.gpio_pin_expression(config[CONF_DATA_PIN])
        cg.add(var.set_data_pin(data_pin_))

# DieselHeaterEnableAction = heater_ns.class_("DieselHeaterEnableAction", automation.Action)
DieselHeaterDisableAction = heater_ns.class_("DieselHeaterDisableAction", automation.Action)


@automation.register_action(
    "diesel_heater.disable",
    DieselHeaterDisableAction,
    automation.maybe_simple_id({}),
)
async def diese_heater_disable_to_code(config, action_id, template_arg, args):
    # print(config)
    # parent = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg)
    # await cg.register_parented(var, parent)

    return var


# @automation.register_action("diese_heater.enable", DieselHeaterEnableAction, cv.Schema({}))
# async def wifi_enable_to_code(config, action_id, template_arg, args):
#     return cg.new_Pvariable(action_id, template_arg)
