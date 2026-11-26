// ble_adv.h — BLE Extended Advertising + 自簽編號（HMAC/seq/boot_id）
//
// v4.0.287 — 主板 → Tier 2 子板單向廣播，升級自 v4.0.283 legacy adv
//
// 變更（vs v4.0.283/284）：
//   - legacy adv (31B) → BLE 5.0 Extended Adv（需 sdkconfig CONFIG_BT_NIMBLE_EXT_ADV=y）
//     payload 上限大幅提升，支援販賣機原子指令 (pre_eff + 出貨 + post_eff) 41-52B
//   - 加自簽編號安全層（12B overhead）：boot_id(2) + seq(2) + HMAC-SHA256-trunc(8)
//     PSK per store；子板記住 (boot_id, seq) 擋重放
//   - Static Random MAC（從 device_sn 推導，每台穩定）
//   - 保留 v4.0.284 kill switch / singleton / idle heartbeat
//
// Payload schema v2（Manufacturer Specific Data，company_id=0xFF01）：
//   off0  [2B] company_id (LE)
//   off2  [4B] device_sn (4 ASCII)
//   off6  [1B] schema_ver = 0x02
//   off7  [1B] target_mask  (0xFF=all; bit0=LED1 bit1=LED2 bit2=Sign bit3=Vend bit4=Sensor)
//   off8  [1B] cmd_id
//   off9  [2B] boot_id (LE)  — 主板 reboot_count & 0xFFFF（NVS 持久、跨重啟單調）
//   off11 [2B] seq (LE)      — per-boot 單調遞增
//   off13 [NB] args
//   末    [8B] HMAC-SHA256(PSK, bytes[0 .. 13+N)) 取前 8 byte
//   → 安全 overhead = 2+2+8 = 12B
//
// 子板驗證：company_id/device_sn/PSK HMAC 通過 + (boot_id,seq) 比上次新 → 接受
//
// rollback：移除 my_components/ble_adv/ + YAML ble_adv: 區塊 + sdkconfig CONFIG_BT_NIMBLE_EXT_ADV

#pragma once

#include "esphome/core/component.h"
#include <atomic>
#include <cstdint>
#include <cstddef>

namespace esphome {
namespace ble_adv {

// CMD ID（與子板 firmware 必須一致）
enum CmdId : uint8_t {
  CMD_IDLE_HEARTBEAT = 0,
  CMD_WASH_STAGE     = 1,
  CMD_EMG            = 2,
  CMD_WASH_INIT      = 3,
  CMD_WASH_COMPLETE  = 4,
  CMD_LED_SWITCH     = 5,
  CMD_VEND_DISPENSE  = 6,  // v4.0.287 販賣機原子指令（pre_eff + 出貨 + post_eff）
  CMD_PAIRING        = 7,  // v4.0.288 配對廣播：args = per-store PSK（子板 scan 學走 PSK+SN）
  CMD_LED_EFF_SCENE  = 8,  // v4.0.292 系統級 LED 場景特效廣播（lock/welcome/emg/min1）
                           //   args = [scene_id(1B), eff CSV ASCII(≤21B)]
                           //   scene_id: 0=lock, 1=welcome, 2=emg, 3=min1
                           //   eff: 例 "a1,b1,c1,d1" 子板自己解析逗號分隔 4 段
  CMD_UNPAIR         = 9,  // 廣播解綁：args = 目標(SN/model/all)；子板符合才自我解綁（離線遠端修復）
  CMD_FIND           = 10, // 廣播尋找：args = 目標(SN/model/編號/all)；子板符合則白色爆閃定位
  CMD_REBOOT         = 11, // 廣播重啟：args = 目標；子板符合則重啟
  CMD_OTA            = 12, // 廣播OTA：args = 目標；子板符合(在線)則拉 OTA
  CMD_DISPENSE       = 13, // 廣播出貨：args = 目標；子板符合(販賣機)則出貨一格
  CMD_DRV            = 14, // 通用driver動作：args = 目標\x1f動作\x1f數值(open/play/mic/dispense)
};

// Target mask bit
constexpr uint8_t TARGET_LED1   = 0x01;
constexpr uint8_t TARGET_LED2   = 0x02;
constexpr uint8_t TARGET_SIGN   = 0x04;
constexpr uint8_t TARGET_VEND   = 0x08;
constexpr uint8_t TARGET_SENSOR = 0x10;
constexpr uint8_t TARGET_ALL    = 0xFF;

// schema 版本（子板用來區分 v0.283 legacy vs v0.287 ext-adv）
constexpr uint8_t SCHEMA_VER = 0x02;

class BleAdv : public Component {
 public:
  void setup() override;
  void loop() override;
  float get_setup_priority() const override { return esphome::setup_priority::AFTER_WIFI; }

  // 廣播一筆 cmd，持續 CMD_BROADCAST_MS 毫秒後自動切回 idle。< 100µs 純 syscall。
  void publish_cmd(uint8_t target_mask, uint8_t cmd_id, const uint8_t *args, size_t args_len);

  // 切回 idle heartbeat（args = uptime 4 byte）
  void publish_idle();

  // v4.0.288 [BLE-PAIR]：廣播 cmd=PAIRING 含 per-store PSK，持續 window_ms（WPS 式配對）
  //   子板進配對模式 scan 到後學走 SN+PSK 寫 NVS；window 結束自動回 idle
  // v4.0.294：args = PSK [\x1f wifi_ssid [\x1f wifi_pass]]（夾帶店家 WiFi → 白板配對即拿網路）
  //   wifi_ssid 為 nullptr/空 → 只發 PSK（向後相容）
  void publish_pairing(uint32_t window_ms, const char *wifi_ssid = nullptr, const char *wifi_pass = nullptr, const char *role = nullptr, const char *store_sn = nullptr);

  // 主板 SN（4 位 ASCII，ws_client device_id_ 帶入）— 同時用來推導 Static Random MAC
  void set_device_serial(const char *sn_str);

  // v4.0.287 PSK（per-store HMAC key，YAML substitution ble_psk 注入）
  // 空字串 = 不簽章（向後相容 / dev；子板端對應放行）
  void set_psk(const char *psk_str);

  // v4.0.287 boot_id（主板 reboot_count & 0xFFFF，on_boot 注入；跨重啟單調，子板擋重放）
  void set_boot_id(uint16_t boot_id) { this->boot_id_ = boot_id; }

  // 觀測
  uint32_t get_seq() const { return this->seq_.load(); }
  bool is_advertising() const { return this->adv_active_.load(); }
  uint32_t get_publish_count() const { return this->publish_count_.load(); }
  bool is_ready() const { return this->ready_.load(); }

  static BleAdv *get_instance() { return BleAdv::instance_; }

  // === v4.0.284 [BLE-KILL-SWITCH] ===
  static void set_enabled(bool e);
  static bool is_enabled() { return BleAdv::enabled_.load(); }
  static std::atomic<bool> enabled_;

 protected:
  static void nimble_host_task(void *arg);
  static void on_sync_cb();
  static void on_reset_cb(int reason);

  // 設 Static Random MAC（從 device_sn 推導，每台穩定）— sync 後呼叫
  void set_static_random_addr_();

  // 設定 ext adv payload + 啟動 ext adv（若還沒啟動）
  void update_adv_data_();

  // 構建 payload（含 boot_id/seq/HMAC trailer），回傳總 byte 數
  size_t build_payload_(uint8_t cmd_id, uint8_t target_mask, const uint8_t *args, size_t args_len);

  static BleAdv *instance_;

  uint8_t device_sn_[4]{0, 0, 0, 0};

  // PSK（HMAC key，最多 32 byte raw）
  uint8_t psk_[32]{};
  size_t  psk_len_{0};

  uint16_t boot_id_{0};

  // Static Random MAC（6 byte，從 SN 推導）
  uint8_t rnd_addr_[6]{};
  bool    rnd_addr_set_{false};

  // ext adv instance 只能 configure 一次（再 configure while active 會錯）
  std::atomic<bool> configured_{false};

  // ext adv payload buffer（schema v2 可達數十 byte，給足空間）
  static constexpr size_t MFG_DATA_MAX = 200;
  uint8_t mfg_data_[MFG_DATA_MAX]{};
  size_t  mfg_data_len_{0};

  // ext adv instance handle（0 = primary）
  static constexpr uint8_t EXT_ADV_INSTANCE = 0;

  std::atomic<uint32_t> seq_{0};
  std::atomic<uint32_t> cmd_until_ms_{0};
  std::atomic<uint32_t> last_idle_ms_{0};
  std::atomic<bool> ready_{false};
  std::atomic<bool> adv_active_{false};
  std::atomic<uint32_t> publish_count_{0};

  static constexpr uint16_t COMPANY_ID = 0xFF01;
  static constexpr uint32_t CMD_BROADCAST_MS = 5000;
  static constexpr uint32_t IDLE_INTERVAL_MS = 30000;
};

}  // namespace ble_adv
}  // namespace esphome
