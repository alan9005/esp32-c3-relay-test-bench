import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

CODEOWNERS = ["@huiflow"]
MULTI_CONF = False

ws_client_ns = cg.esphome_ns.namespace("ws_client")
WSClientComponent = ws_client_ns.class_("WSClientComponent", cg.Component)

CONF_SERVER = "server"
CONF_API_KEY = "api_key"
CONF_DEVICE_ID = "device_id"
CONF_USE_SSL = "use_ssl"
CONF_PORT = "port"
CONF_SERVER2 = "server2"
CONF_PORT2 = "port2"
CONF_USE_SSL2 = "use_ssl2"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(WSClientComponent),
        cv.Required(CONF_SERVER): cv.string,
        cv.Required(CONF_API_KEY): cv.string,
        cv.Required(CONF_DEVICE_ID): cv.string,
        cv.Optional(CONF_USE_SSL, default=False): cv.boolean,
        cv.Optional(CONF_PORT, default=0): cv.int_range(min=0, max=65535),
        cv.Optional(CONF_SERVER2, default=""): cv.string,
        cv.Optional(CONF_PORT2, default=0): cv.int_range(min=0, max=65535),
        cv.Optional(CONF_USE_SSL2, default=False): cv.boolean,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_server(config[CONF_SERVER]))
    cg.add(var.set_api_key(config[CONF_API_KEY]))
    cg.add(var.set_device_id(config[CONF_DEVICE_ID]))
    cg.add(var.set_use_ssl(config[CONF_USE_SSL]))
    cg.add(var.set_port(config[CONF_PORT]))

    if config[CONF_SERVER2]:
        cg.add(var.set_server2(config[CONF_SERVER2]))
        cg.add(var.set_port2(config[CONF_PORT2]))
        cg.add(var.set_use_ssl2(config[CONF_USE_SSL2]))

    # No external libraries needed - uses ESP-IDF native transport layer
    # esp_transport, esp_transport_tcp, esp_transport_ws are built into ESP-IDF
