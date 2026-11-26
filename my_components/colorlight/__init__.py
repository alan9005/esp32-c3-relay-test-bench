"""Colorlight HTTP client (fire-and-forget background task).

v3.16.57: 取代 ESPHome 內建 http_request.send（同步阻塞主 loop 最多 3 秒），
改用獨立 FreeRTOS task 送 HTTP PUT 到 Colorlight LED 播放盒。
- Colorlight 離線時，主 loop 完全不受影響（之前會卡 3 秒）
- 送完 request 立即 close socket，不讀 response（fire-and-forget）
- TCP connect timeout 500ms，send timeout 500ms
- 最多同時 2 個 task 並發（超過則丟棄）
"""
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

CODEOWNERS = ["@huiflow"]
MULTI_CONF = False

colorlight_ns = cg.esphome_ns.namespace("colorlight")
ColorlightClient = colorlight_ns.class_("ColorlightClient", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(ColorlightClient),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
