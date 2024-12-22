import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (DEVICE_CLASS_POWER, ICON_COUNTER,
                           UNIT_CELSIUS,
                           DEVICE_CLASS_TEMPERATURE,
                           STATE_CLASS_MEASUREMENT,
                           DEVICE_CLASS_VOLTAGE)

from . import DieselHeater, CONF_HEATER_ID

DEPENDENCIES = []
AUTO_LOAD = ["sensor"]

CONF_HEATER_TEMPERATURE = "temperature"
CONF_VOLTAGE = "voltage"
CONF_POWER = "power"
CONF_MODE = "mode"
CONF_ALPINE = "alpine"
CONF_FAN = "fan"
CONF_PUMP = "pump"
CONF_SPARK_PLUG = "spark_plug"

heater_ns = cg.esphome_ns.namespace("diesel_heater")


CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(CONF_HEATER_ID): cv.use_id(DieselHeater),
            cv.Optional(CONF_HEATER_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_VOLTAGE): sensor.sensor_schema(
                unit_of_measurement="V",
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_VOLTAGE,
                state_class=STATE_CLASS_MEASUREMENT
            ),
            cv.Optional(CONF_POWER): sensor.sensor_schema(
                unit_of_measurement="W",
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_POWER,
                state_class=STATE_CLASS_MEASUREMENT
            ),
            cv.Optional(CONF_MODE): sensor.sensor_schema(
                state_class=STATE_CLASS_MEASUREMENT
            ),
            cv.Optional(CONF_ALPINE): sensor.sensor_schema(
                icon=ICON_COUNTER,
                state_class=STATE_CLASS_MEASUREMENT
            ),
            cv.Optional(CONF_FAN): sensor.sensor_schema(
                state_class=STATE_CLASS_MEASUREMENT
            ),
            cv.Optional(CONF_PUMP): sensor.sensor_schema(
                state_class=STATE_CLASS_MEASUREMENT
            ),
            cv.Optional(CONF_SPARK_PLUG): sensor.sensor_schema(
                state_class=STATE_CLASS_MEASUREMENT
            ),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def setup_conf(config, key, hub):
    if sensor_config := config.get(key):
        sens = await sensor.new_sensor(sensor_config)
        cg.add(getattr(hub, f"set_{key}_sensor")(sens))


async def to_code(config):
    var = await cg.get_variable(config[CONF_HEATER_ID])

    if CONF_HEATER_TEMPERATURE in config:
        await setup_conf(config, CONF_HEATER_TEMPERATURE, var)
    if CONF_VOLTAGE in config:
        await setup_conf(config, CONF_VOLTAGE, var)
    if CONF_POWER in config:
        await setup_conf(config, CONF_POWER, var)
    if CONF_MODE in config:
        await setup_conf(config, CONF_MODE, var)
    if CONF_ALPINE in config:
        await setup_conf(config, CONF_ALPINE, var)
    if CONF_FAN in config:
        await setup_conf(config, CONF_FAN, var)
    if CONF_PUMP in config:
        await setup_conf(config, CONF_PUMP, var)
    if CONF_SPARK_PLUG in config:
        await setup_conf(config, CONF_SPARK_PLUG, var)
