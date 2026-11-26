import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import i2c
from esphome.const import (
    CONF_ID,
    CONF_ADDRESS,
    CONF_SIZE,
)

CODEOWNERS = ["@huiflow"]
DEPENDENCIES = ["i2c"]
MULTI_CONF = True

CONF_ON_SETUP = "on_setup"

i2c_eeprom_ns = cg.esphome_ns.namespace("i2c_eeprom")
I2CEEPROMComponent = i2c_eeprom_ns.class_(
    "I2CEEPROMComponent", cg.Component, i2c.I2CDevice
)

# Map of human-readable sizes to byte counts
EEPROM_SIZES = {
    "1KB": 1024,
    "2KB": 2048,
    "4KB": 4096,
    "8KB": 8192,
    "16KB": 16384,
    "32KB": 32768,
    "64KB": 65536,
    "128KB": 131072,
}


def validate_eeprom_size(value):
    value = str(value).upper()
    if value not in EEPROM_SIZES:
        raise cv.Invalid(
            f"Unknown EEPROM size '{value}'. Supported sizes: {', '.join(EEPROM_SIZES.keys())}"
        )
    return value


CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(I2CEEPROMComponent),
            cv.Required(CONF_SIZE): validate_eeprom_size,
            cv.Optional(CONF_ON_SETUP): automation.validate_automation(single=True),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(i2c.i2c_device_schema(0x50))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    size_bytes = EEPROM_SIZES[config[CONF_SIZE]]
    cg.add(var.set_memory_size(size_bytes))

    if CONF_ON_SETUP in config:
        await automation.build_automation(
            var.get_on_setup_trigger(), [], config[CONF_ON_SETUP]
        )
