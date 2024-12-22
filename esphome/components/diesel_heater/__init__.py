from esphome import pins
import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import CONF_ID

DEPENDENCIES = []


CONF_DATA_PIN = "data_pin"
CONF_DEBUG_PIN_1 = "debug_pin_1"
CONF_DEBUG_PIN_2 = "debug_pin_2"

heater_ns = cg.esphome_ns.namespace("diesel_heater")
DieselHeater = heater_ns.class_("DieselHeater", sensor.Sensor, cg.Component)

CONF_HEATER_ID = "diesel_heater"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(DieselHeater),
            cv.Optional(CONF_DATA_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_DEBUG_PIN_1): pins.gpio_output_pin_schema,
            cv.Optional(CONF_DEBUG_PIN_2): pins.gpio_output_pin_schema,
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
    if CONF_DEBUG_PIN_1 in config:
        debug_pin_1_ = await cg.gpio_pin_expression(config[CONF_DEBUG_PIN_1])
        cg.add(var.set_debug_pin_1(debug_pin_1_))
    if CONF_DEBUG_PIN_2 in config:
        debug_pin_2_ = await cg.gpio_pin_expression(config[CONF_DEBUG_PIN_2])
        cg.add(var.set_debug_pin_2(debug_pin_2_))
