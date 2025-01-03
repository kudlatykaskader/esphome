import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import (
    CONF_POWER,
    ICON_POWER,
    DEVICE_CLASS_BUTTON
)

from . import DieselHeaterBLE, CONF_HEATER_ID, diesel_heater_ble_ns

CODEOWNERS = ["@warehog"]
DEPENDENCIES = ["diesel_heater_ble"]

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(CONF_HEATER_ID): cv.use_id(DieselHeaterBLE),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)

async def to_code(config):
    parent = await cg.get_variable(config[CONF_HEATER_ID])