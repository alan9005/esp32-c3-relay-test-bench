"""BLE Advertising (Manufacturer Specific Data) — 主板廣播給 Tier 2 子板的單向通道。

v4.0.283: 純 ESP-IDF NimBLE host stack，**不用 ESPHome esp32_ble component**（保持自主控制）。

設計目的：
- 主板 → Tier 2 子板（LED 效果 / 販賣機 / 感測 box）的「單向、無 ACK、不延遲 main loop」通訊
- 取代過去 v4.0.128 UDP sendto 卡 main loop 的災難
- 取代過去 v4.0.238/278 REST device channel（v4.0.282 已拆乾淨）

API：
  id(ble_adv).publish_cmd(target_mask, cmd_id, args, args_len);  // < 100µs，純 syscall
  id(ble_adv).publish_idle();                                     // 1Hz heartbeat

Payload schema（Manufacturer Specific Data 內）:
  [2B company_id = 0xFF01]
  [4B device_sn (主板 SN 4 位 ASCII)]
  [1B seq (monotonic 0-255 wrap)]
  [1B target_mask (0xFF=全廣播; bit0=LED1, bit1=LED2, ...)]
  [1B cmd_id (0=IDLE_HEARTBEAT, 1=WASH_STAGE, 2=EMG, ...)]
  [N B args (≤22B)]
總長度 ≤31 byte（BLE 4.x 基本 adv payload）。
"""
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

CODEOWNERS = ["@huiflow"]
MULTI_CONF = False

ble_adv_ns = cg.esphome_ns.namespace("ble_adv")
BleAdv = ble_adv_ns.class_("BleAdv", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(BleAdv),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
