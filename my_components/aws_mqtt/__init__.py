import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

CODEOWNERS = ["@huiflow"]
MULTI_CONF = False

aws_mqtt_ns = cg.esphome_ns.namespace("aws_mqtt")
AWSMqttComponent = aws_mqtt_ns.class_("AWSMqttComponent", cg.Component)

CONF_ENDPOINT = "endpoint"
CONF_PORT = "port"
CONF_TENANT_ID = "tenant_id"
CONF_SECRET_KEY = "secret_key"
CONF_NONCE = "nonce"
CONF_AUTH_NAME = "auth_name"
CONF_CLIENT_ID = "client_id"
CONF_SUB_TOPIC = "sub_topic"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(AWSMqttComponent),
        cv.Required(CONF_ENDPOINT): cv.string,
        cv.Optional(CONF_PORT, default=443): cv.int_range(min=1, max=65535),
        cv.Required(CONF_TENANT_ID): cv.string,
        cv.Required(CONF_SECRET_KEY): cv.string,
        cv.Optional(CONF_NONCE, default="localtest"): cv.string,
        cv.Optional(CONF_AUTH_NAME, default="ygl-test-third"): cv.string,
        cv.Optional(CONF_CLIENT_ID, default="t-display-s3-001"): cv.string,
        cv.Optional(CONF_SUB_TOPIC, default=""): cv.string,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_endpoint(config[CONF_ENDPOINT]))
    cg.add(var.set_port(config[CONF_PORT]))
    cg.add(var.set_tenant_id(config[CONF_TENANT_ID]))
    cg.add(var.set_secret_key(config[CONF_SECRET_KEY]))
    cg.add(var.set_nonce(config[CONF_NONCE]))
    cg.add(var.set_auth_name(config[CONF_AUTH_NAME]))
    cg.add(var.set_client_id(config[CONF_CLIENT_ID]))
    cg.add(var.set_sub_topic(config[CONF_SUB_TOPIC]))
