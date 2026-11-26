#pragma once

#include "esphome/core/component.h"

// Forward declaration for script type (避免循環 include)
namespace esphome {
namespace script {
template<typename... Ts> class Script;
}
}

#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/network/util.h"
#include "esphome/components/ethernet/ethernet_component.h"
#include "esphome/components/light/light_state.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/fan/fan.h"
#include "esphome/components/number/number.h"
#include "esphome/components/text/text.h"
#include "esphome/components/script/script.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <map>
#include <string>

extern "C" {
#include "mbedtls/ssl.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#include "cJSON.h"

// v4.0.16: 全域 reboot tag（所有重啟來源寫入，YAML 3s breadcrumb 讀取並寫 EEPROM）
//   任何組件呼叫 esp_restart() / safe_reboot() 前，先 set_reboot_tag_c("REBOOT_XXX")
//   下次開機 on_boot 讀 EEPROM 0x0200 → last_crash_context 顯示兇手
extern char g_reboot_tag[32];
void set_reboot_tag_c(const char *tag);
}

namespace esphome {
namespace ws_client {

enum ConnectState : uint8_t {
  CS_IDLE,
  CS_CONNECTING,    // Background task is running do_connect()
  CS_CONNECTED,     // Background task succeeded
  CS_FAILED,        // Background task failed
};

class WSClientComponent : public Component {
 public:
  void set_server(const std::string &server) { this->server_ = server; }
  void set_api_key(const std::string &key) { this->api_key_ = key; }
  void update_api_key(const std::string &key) { if (!key.empty()) this->api_key_ = key; }
  void set_device_id(const std::string &id) { this->device_id_ = id; }
  void set_firmware_version(const std::string &v) { this->firmware_version_ = v; }
  void set_use_ssl(bool use_ssl) { this->use_ssl_ = use_ssl; }
  void set_port(int port) { this->port_ = port; }
  // LAN WS (second connection)
  void set_server2(const std::string &s) { this->lan_server_ = s; }
  void set_port2(int p) { this->lan_port_ = p; }
  void set_use_ssl2(bool v) { this->lan_use_ssl_ = v; }
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  void setup() override;
  void loop() override;

  /// Manual connect/disconnect (called from Dashboard buttons)
  void manual_connect();
  void manual_disconnect();

  /// v4.0.91: WS1 LAN test switch — 切 server_ 到 192.168.1.99 (port/SSL/api_key 不變)
  ///   實作要點（v4.0.88 失敗版重做）：
  ///     (1) 每階段都 log_to_dashboard 確認觸發
  ///     (2) 改 server_ + reset_attempts + 強制 manual_disconnect
  ///     (3) 不依賴 loop reconnect，呼 manual_connect 立刻觸發 do_connect 用新 server_
  ///   非持久化，重開機從 YAML 還原。
  void switch_ws1_to_lan_test();

  /// Send arbitrary text data to server (for external callers)
  bool send_data(const std::string &data);

  /// Broadcast arbitrary text to BOTH WS1 (cloud) and WS2 (LAN) sockets.
  /// Used by STATUS / CMD_RESULT / PONG — any non-EVENT message that needs
  /// to land on both panels.
  void send_both(const std::string &data);

  /// 發送 EVENT 到 WS1+WS2（唯一對外接口，取代舊的 send_emg / send_event）
  /// 自動構建 {"type":"EVENT","event":"<name>","state":<bool>} JSON（無 device_id、無 arg）
  /// 同一個 event_name 在 1500ms 內只送一次，中間的忽略（per-event 節流）
  /// 由 YAML on_state lambda 直接呼叫，C++ 內部不再做 edge 偵測
  void send_event(const char *event_name, bool state);

  /// 發送 washer_status EVENT（特例：多一個 color 欄位）
  /// {"type":"EVENT","event":"washer_status","state":true,"color":"<red|blue|green>"}
  /// 不會發送 state=false（全 OFF 時完全不發事件）
  /// 1500ms 節流 key 為 "washer_status"（所有 color 共用一個 throttle 槽）
  void send_washer_status(const char *color);

  /// v3.15.26: GPIO39 啟停按鈕按下 → 根據 go_enable_arg 決定是否呼叫 gowasher，
  /// 然後送 EVENT {event:"go_enable","state":"off","go_enable_arg":"<arg>"}
  /// 由 YAML GPIO39 on_release lambda 呼叫
  void on_go_button_pressed();

  /// v3.16.0: GPIO38 緊急按鈕按下 → 自動循環 gowasher 直到 blue ON（最多 10 次）
  /// 同時禁用 GO 按鈕（go_enable_arg_=""），防止緊急停車後誤觸啟動
  /// 由 YAML GPIO38 on_release lambda 呼叫
  void on_emg_button_pressed();

  /// v3.16.0: Wash stage counter — YAML on_state 呼叫，取代直接 send_washer_status()
  /// 同時處理: (1) 既有 washer_status EVENT 發送  (2) stage counter debounce 追蹤
  void on_washer_led_change(const char *color, bool state);

  /// v3.16.0: HMI 密碼提交 — YAML uart_proc_hex 呼叫
  /// 離線 → 本地 debug 密碼比對；在線 → 送 hmi_password EVENT 到 WS1+WS2
  void on_hmi_password_submit(const char *pwd, int len);

  /// v4.0.85: 注入 current_total sensor 指標（YAML on_boot 呼叫）
  /// EMG 觸發時直接讀 current_total->state 確認機器是否在工作
  void set_current_total_sensor(sensor::Sensor *s) { this->current_total_sensor_ = s; }

  /// v4.0.117+: ping/pong log 開關（YAML switch 控制）
  ///   預設 false = silent（ping/pong 不寫 dashboard log，避免 30s/次 noise）
  ///   true = 寫 dashboard log（除錯用）
  static bool show_ping_log_;
  static void set_show_ping_log(bool v) { show_ping_log_ = v; }

  /// v4.0.85: HMI EMG alert 字串顯示（Y=480 wash flow status line 暫時佔用顯示）
  /// 由 hmi_refresh_bb4_line YAML script 讀取，有 alert 時顯示 alert 而非 wash flow
  /// EMG 觸發後 set，下次 wash flow 變化時自動 clear
  bool has_hmi_emg_alert() const { return !this->hmi_emg_alert_buf_.empty(); }
  const std::string &get_hmi_emg_alert() const { return this->hmi_emg_alert_buf_; }
  void set_hmi_emg_alert(const std::string &s) { this->hmi_emg_alert_buf_ = s; }

  // === v4.0.241 [WASHER-REDABORTED] BEGIN ===
  // 流程進行中（wash_active_=true）洗車機突然進紅燈 → 強制中斷
  // 必須在 destructive reset 清狀態前呼叫，才讀得到當前 stage / step / order / durations
  // 帶完整訂單資訊（給 server 退款）+ 中斷瞬間步驟 + 已完成的各 step 純 GREEN 秒數
  void send_washer_redaborted_event();
  // === v4.0.241 [WASHER-REDABORTED] END ===

  // === v4.0.238 [DEVICE-REGISTRY] BEGIN ===
  /// 外部裝置自報接收（YAML text entity device_announce_in on_value 呼叫）
  /// JSON 格式：{"id":"led-mod-1","ip":"192.168.1.105","ver":"1.1.0","paired_sn":"1140","bssid":"...","rssi":-65}
  /// SN 不匹配主板 device_id_ 自動拒絕（rogue 設備保護）
  /// 維護內部 device_registry_ map（id → DeviceInfo），STATUS 帶 paired_devices array
  void on_device_announce(const std::string &json);

  /// 外部裝置上報事件（YAML text entity device_event_in on_value 呼叫）
  /// JSON 格式：{"id":"button-1","action":"press","short":3,"paired_sn":"1140"}
  /// SN 匹配後透過 WS event 上報 server：{"type":"EVENT","event":"device_event",...}
  void on_device_event(const std::string &json);
  // === v4.0.238 [DEVICE-REGISTRY] END ===

  // === v4.0.278 [DEVICE-OUT-BUS] BEGIN ===
  // 主板 → 裝置 outbound 通道（plan 方案 A 的 REST polling 變體）
  //   為何不開 WS server：v3.14.2 已移除本地 WS server；ESPHome web_server 不支援 custom
  //   HTTP endpoint（Stage 21 已驗證）。改用既有 text_sensor entity REST API：
  //   裝置 GET /text_sensor/device_event_bus 拉 wash_stage / EMG 等廣播事件；
  //   裝置 GET /text_sensor/device_cmd_out 拉 per-target 指令；
  //   用 seq 編號做 dedup（裝置記住 last_seen_seq，只處理較新的）。
  //
  // 設計目的：plan Tier 2 裝置（LED 效果 / 販賣機 / 輔助感測 ESP32）可從主板接事件
  // 與指令，主板掛時裝置 fallback 預設行為（NVS 預存）。Tier 1 / 三件套不受影響。

  /// 廣播事件給所有訂閱裝置（無 target，全裝置可收）
  /// payload 完整 JSON，自動 wrap 為 {"seq":N,"event":payload}
  void publish_device_bus(const std::string &event_json);

  /// 發送指令給特定 target 裝置（target_id 對應裝置 announce 的 "id" 欄位）
  /// 自動 wrap 為 {"seq":N,"target":"...","cmd":...}
  void publish_device_command(const std::string &target_id, const std::string &cmd_json);

  /// 取得當前 seq 計數（除錯/STATUS 用）
  uint32_t get_device_bus_seq() const { return this->device_bus_seq_; }
  // === v4.0.278 [DEVICE-OUT-BUS] END ===

  void clear_hmi_emg_alert() { this->hmi_emg_alert_buf_.clear(); }

  /// v3.16.16: 工程頁面「重新開機」按鈕 — 走兩段式 reboot（500ms 延遲 → safe_reboot → 硬重啟）
  /// 等同 WS CMD "reboot"；YAML HMI 按鈕 0x2C 呼叫
  void request_reboot() {
    this->reboot_pending_ = true;
    this->reboot_requested_at_ = 0;
  }

  /// v3.16.16: 公開 send_event_raw 供 YAML lambda 呼叫（工程測試按鈕 Test EMG 直接送 EVENT）
  /// 底層雙發到 WS1+WS2，跳過節流
  void send_event_raw_public(const char *json, size_t len) {
    this->send_event_raw(json, len);
  }

  /// v4.0.22: HTTP API 追蹤事件 — 所有 Colorlight / 其他 HTTP 發送後呼叫
  ///   格式：{"type":"API","cmd":"<cmd>",<params_json...>}
  ///   params_json 是 JSON 物件的「內部欄位」（不含外層 {}），例如 `"name":"01.vsn","source":"lan"`
  ///   空字串則只有 type+cmd。跳過節流（每次 HTTP 都要追蹤）。
  ///   YAML LED scripts 在呼叫 id(cl).put(...) 之前呼叫本函式。
  void send_api_event(const char *cmd, const std::string &params_json);

  // === v4.0.217 [WASH-EVENTS] BEGIN ===
  // 為每個 wash CMD 觸發專屬 EVENT，讓 server / dashboard 不靠 STATUS pull 也能即時知道狀態。
  // 設計目的：取代 v4.0.13 dashboard 端「wash_active=true 自動勾 5s status checkbox」的耗能設計。
  // Rollback：刪除 BEGIN/END 區塊 + 各 CMD handler fire 處
  void send_wash_event_simple(const char *event_name, const char *source);
  // v4.0.220: 完整 wash_init EVENT — 帶全部 5 個 order 欄位 + steps array (含 led_eff)
  //   Server 可直接從 EVENT 拼出完整流程定義，不需要再從 wash_init CMD echo 推導
  void send_wash_init_event(const char *src_tag);
  // v4.0.224: wash_complete EVENT — DONE 時送一次，帶 order_sn / total / total_sec / durations[]
  //   取代原 wash_stage(phase=done) 空殼 EVENT，提供完整流程結算資訊
  void send_wash_complete_event();
  // === v4.0.217 [WASH-EVENTS] END ===

  // === v4.0.202 [RPI-EVENT-HELPER] BEGIN ===
  // RPi 通知事件 — 雙發 WS1+WS2 同步 + dashboard log [RPI] 前綴
  // 設計哲學：主板+洗車機+Colorlight (.14) 為「離線可運作三件套」
  //   — 即便 WS1/WS2/RPi 全掛，洗車流程必須照常跑
  //   — RPi event 是容錯通知，不替代既有 HTTP PUT 動作（執行原動作後再 fire）
  // 觸發點：LED 5 CMD 動作後 / GREEN→BLUE edge / BLUE 倒數剩 60s 等
  //
  // 格式：{"type":"EVENT","event":"rpi","action":"<action>"[,<params>]}
  // params_json: JSON 內部欄位（不含外層 {}），例 `"device_id":"1140","step":"Pre-Soak"`
  // log_kvs:     human-readable 字串，例 `device_id=1140 step=Pre-Soak`
  // Rollback：刪除 BEGIN/END 區塊 + 所有 send_rpi_event 呼叫點
  void send_rpi_event(const char *action, const std::string &params_json, const std::string &log_kvs);
  // === v4.0.202 [RPI-EVENT-HELPER] END ===

  // === v4.0.233 [RPI-CAPTURE-UNLOCK-COMPLETE] BEGIN ===
  // 包裝 send_rpi_event 用 cJSON 建 capture payload（保證 order 欄位含特殊字元也安全）
  // step_name = "unlock"   → 帶 order_sn / order_cost / order_product
  // step_name = "complete" → 只帶 order_sn
  // 其他 step_name → 通用 capture（device_id + order_sn + step_name + event_name=wash）
  // include_order_full=true 才帶 order_cost/order_product（unlock 用）
  void send_rpi_capture_event(const char *step_name, bool include_order_full);
  // === v4.0.233 [RPI-CAPTURE-UNLOCK-COMPLETE] END ===

  /// v3.16.16: 公開 hmi_show_page 供 YAML lambda 呼叫（HMI back 按鈕 0x29）
  /// 委派給 YAML hmi_show_page script（AA 52 clear + AA 70 switch + 完整重繪）
  void hmi_show_page_public(int page) {
    this->hmi_show_page(page);
  }

  /// v4.0.96: HMI 解鎖成功 — 4 個密碼框顯示「验证成功」(GBK 簡體)
  ///   AA 71 清密碼矩形（同 hmi_clear_pass_code）+ AA 98 大字型逐格寫
  ///   座標：X=110/225/340/455, Y=184（與 hmi_show_pass_code 一致）
  ///   字符：验=D1E9 / 证=D6A4 / 成=B3C9 / 功=B9A6
  ///   呼叫時機：do_wash_unlock(src="HMI") 解鎖成功
  void hmi_show_unlock_success();

  /// v4.0.97: HMI 密碼錯誤清空 — AA 71 矩形覆蓋密碼框（同 YAML hmi_clear_pass_code）
  ///   呼叫時機：on_hmi_password_submit 發現密碼不符任何（unlock_pwd / debug_pwd）→ 清空
  void hmi_clear_pass_code_c();

  // ═══════════════════════════════════════════════════════════════════════════
  // v4.0.99: VSN map (key → vsn_name 對應，全部 source="internet")
  //   設計目的：方便大量部署 — 所有 LAN 寫死的 VSN 改成 internet 來源 + 變數化
  //   持久化到 NVS namespace "vsn_map"，runtime 由 CMD vsn_update 修改
  //   key 列表：warn_emg / emg / welcome / lock / hmi_camtest / hmi_text / 01~09
  //   01~09 = 洗車流程 step LED（wash_init JSON 用 led_name="01" 這種 key 取代「01.vsn」）
  // ═══════════════════════════════════════════════════════════════════════════
  /// 取得 vsn key 對應的真實檔名；找不到回空字串
  /// v4.0.101: 自動 strip ".vsn" 後綴 → "01.vsn" / "01" 兩種 caller 格式都能 lookup
  /// v4.0.266: passthrough fallback 邏輯移到 caller 端（offline_led_trigger）。
  ///   vsn_lookup 回歸純粹 map 查找，不再做 .vsn 完整檔名 fallback。
  ///   caller 自行決定：「`.vsn` 結尾 → 直接用；其他 → 才 call vsn_lookup」。
  std::string vsn_lookup(const std::string &key) const {
    std::string k = key;
    if (k.size() > 4 && k.compare(k.size() - 4, 4, ".vsn") == 0) {
      k = k.substr(0, k.size() - 4);
    }
    auto it = this->vsn_map_.find(k);
    return (it != this->vsn_map_.end()) ? it->second : std::string();
  }
  /// 設定 vsn key 對應 + 寫 NVS（持久化）
  void vsn_set(const std::string &key, const std::string &name);
  /// 公開讀整個 map（CMD vsn_list 用）
  const std::map<std::string, std::string> &vsn_map_get() const { return this->vsn_map_; }

  // v3.16.1: HMI 當前頁面追蹤 — YAML lambda 讀取控制是否推送監控數據
  // Page 0=工程頁（正常推送）, Page 1=密碼頁（不推送，避免覆蓋）
  int hmi_current_page_{1};

  /// v3.16.1: 洗車是否進行中 — YAML lambda 讀取用於 Page 1 狀態行顯示
  bool is_wash_active() const { return this->wash_active_; }

  /// v4.0.20: 本次 wash flow 累積已走秒數（從 wash_stage_=1 進入那一刻起算）
  ///   wash_total_start_ms_ == 0 → 尚未進入第 1 步（still IDLE / stage 0 或未啟動）
  ///   !wash_active_ → 0（HMI 顯示用；已結束不顯示時間）
  uint32_t wash_elapsed_sec() const;

  /// v3.16.68: Breadcrumb accessors — YAML 3s interval 讀取寫入 EEPROM snapshot
  ///   crash 後開機讀出 → 可知當下 wash_phase/stage 卡在哪
  uint8_t get_wash_phase() const { return (uint8_t) this->wash_phase_; }
  uint8_t get_wash_stage() const { return this->wash_stage_; }
  uint8_t get_wash_total() const { return this->wash_total_; }

  /// Execute a CMD locally (from Dashboard simulation or external call)
  /// Returns result string (OK:... or ERR:...)
  /// v3.16.64: src_tag 預設 "DASH"；HMI 自動派送場景（工程頁 startwash5/6）傳 "HMI"
  std::string execute_cmd(const std::string &json_cmd, const char *src_tag = "DASH");

  bool is_connected() const { return this->connected_; }
  /// WS2 連線檢查：簡單回傳 lan_connected_ 旗標
  bool is_lan_connected() const { return this->lan_connected_; }

  /// 獲取洗車機指令 frame（供 YAML script 調用）
  /// cmd_num: 1-10 對應 startwasher1~6, gowasher, resetwasher, hardresetwasher, sparewasher
  /// 返回: 7-byte frame 指針，無效 cmd_num 返回 nullptr
  static const unsigned char* get_washer_frame(int cmd_num);

  /// 設置 washer script（由 YAML 在 on_boot 時調用）
  void set_washer_script(script::Script<int, std::string, std::string> *script) {
    this->washer_script_ = script;
  }

  /// v3.16.13: 設置 hmi_show_page YAML script（供 hmi_auth_result / on_hmi_password_submit 觸發完整重繪）
  /// YAML script 執行：AA 52 clear → AA 70 switch → 300ms delay → Page 0 完整重繪
  void set_hmi_show_page_script(script::Script<int> *script) {
    this->hmi_show_page_script_ = script;
  }

  /// v3.16.59: 設置 led_switch YAML script（供 offline_led_trigger 本地切換節目）
  ///   YAML 腳本參數：name (VSN 檔名), source (lan / internet)
  ///   注意 v3.16.59 簡化：每步驟只做 led_switch，不再做 led_update（文字/顏色）
  void set_led_switch_script(script::Script<std::string, std::string> *script) {
    this->led_switch_script_ = script;
  }

  /// v4.0.02: 直接注入 led_update/brightness/volume scripts，
  ///   WS CMD 路徑不再透過 publish_sensor("led_cmd") → text_sensor on_value 繞路。
  ///   ESP-IDF 下 template text_sensor on_value trigger 時序疑似異常（v4.0.00 WS CMD 沒反應）。
  void set_led_update_script(script::Script<std::string, std::string, std::string, int, int, std::string> *script) {
    this->led_update_script_ = script;
  }
  void set_led_brightness_script(script::Script<int> *script) {
    this->led_brightness_script_ = script;
  }
  void set_led_volume_script(script::Script<int> *script) {
    this->led_volume_script_ = script;
  }
  /// v4.0.111: led_dualcam YAML script — RPi 雙鏡頭 LED 模組啟動
  ///   參數：(order_sn, order_name, order_msg)
  ///   內部 cl.put PUT http://192.168.1.10:8096/module_dualcam_LED/start
  ///   body: top_text=order_name, bottom_text=order_msg, qr_url=https://odr.huiflow.com/<order_sn>
  void set_led_dualcam_script(script::Script<std::string, std::string, std::string> *script) {
    this->led_dualcam_script_ = script;
  }

  /// v4.0.116: lock 模式 LED 序列 script（解 v4.0.111 colorlight worker 1-slot queue race）
  ///   參數：(vsn, order_sn, order_name, order_msg)
  ///   內部 chain：led_switch → delay 400ms → led_text 上行 → delay 250ms → led_text 下行 → delay 500ms → led_dualcam
  ///   總 ~1.15s；mode: single 防重入
  void set_lock_led_sequence_script(script::Script<std::string, std::string, std::string, std::string> *script) {
    this->lock_led_sequence_script_ = script;
  }

  /// v3.16.35: 離線模式查詢（供 YAML lambda 用於 HMI 狀態列後綴 " OFFLINE"）
  ///   true = 超過 90s 無 online CMD → 進入本地流程控制
  ///   false = Server 心跳正常，所有流程由 Server 主導
  bool is_offline() const { return this->offline_mode_; }

  /// v3.16.67: Dashboard Reset 按鈕 — 清空所有洗車階段狀態
  ///   - 關閉所有泡泡 bb1~bb4
  ///   - 清空 wash_steps_ / wash_total_ / wash_stage_ / LED debounce
  ///   - 關閉 R03 軟水、設定 wash_active_=false / wash_done_flag_=false
  ///   - 送 wash_stage EVENT 通知 Dashboard 顯示 IDLE / 空白
  ///   - 不影響 server heartbeat（offline_mode_ 由心跳邏輯自然決定）
  ///   v3.16.73: 同時清掉 NVS 持久化狀態
  // v4.0.219: 加 source 參數，讓 wash_reset EVENT 帶 caller 來源（CMD/HMI/DASH/INTERNAL/RED-DEST 等)
  void wash_reset_public(const char *source = "INTERNAL");

  /// v4.0.252: 清 current_led_color_ + prev_led_color_for_wash_
  ///   wash_reset 內部呼叫（重置流程連同 LED 狀態都歸零）
  ///   Test LED 按鈕 OFF 時呼叫（本地模擬清乾淨，避免 stale red/blue 干擾下次 wash_unlock 派送）
  void clear_current_led_color();

  /// v4.0.28: 洗車流程暫停 / 恢復
  ///   paused=true：凍結 BLUE auto_sec 倒數剩餘秒數（存 pause_frozen_remain_ms_）
  ///   paused=false：平移 blue_enter_at_ 讓剩餘秒數延續（now - elapsed_at_pause）
  ///   送 wash_stage EVENT（帶 paused 欄位）+ send_status Dashboard 同步
  ///   source_tag：記 log 用（"EMG" / "CMD:wash_stop" / "CMD:wash_start" / "DASH"）
  void set_wash_paused(bool paused, const char *source_tag);
  bool is_wash_paused() const { return this->wash_paused_; }

  /// v3.16.73: 洗車流程狀態 NVS 持久化（ESP32 內建 flash）
  ///   寫：wash_init 完成 / 每次 phase 轉換 / wash_done
  ///   讀：loop() 開機後 2 秒（pending_state_load_done_ 旗標，避免太早碰 modbus switch）
  ///   清：wash_reset_public()
  ///   恢復範圍：steps + stage + phase + done_flag + bb1~bb4 + R03 軟水
  ///   不恢復：其他 switch（繼電器/SSR 等）、LED 螢幕（online 時靠 Server）
  ///   BLUE 倒數：重算（blue_enter_at_ = now）
  void wash_state_save_nvs();
  bool wash_state_load_nvs();   // true = 有效狀態已恢復；false = 無狀態/無效
  void wash_state_clear_nvs();

  /// v3.16.76: HMI 狀態列顯示當前洗車步驟
  ///   格式：`AIScanning (5/9)BB1/SW1/1.vsn/287`
  ///   ├─ 步驟名
  ///   ├─ (seq/total)
  ///   ├─ BB{bb}
  ///   ├─ SW{r03}   (r03=1→SW1, r03=0→SW0)
  ///   ├─ {led_name}
  ///   └─ {auto_remain_s}  — BLUE 階段顯示剩餘秒數；其他階段顯示 "-"
  ///   特殊回傳：
  ///     - wash_done_flag_ → "結束"
  ///     - !wash_active_ → ""（空字串讓 Y=480 bb4 列走 OFFLINE/ONLINE 格式）
  std::string wash_step_hmi_label() const {
    // v4.0.73: 結束改英文 "Done"；wash_paused_ 時前面加 (PAUSE)
    // v4.0.83: 鎖定狀態（unlock_pwd_ 非空）→ 前面加 "lock "
    // v4.0.95: 倒數格式改 (remain/auto_sec s)(total_sec s)，整合 last_step_finish_at_
    if (this->wash_done_flag_) return std::string("Done");
    if (!this->wash_active_ || this->wash_total_ == 0) return std::string();
    uint8_t idx = this->wash_stage_;
    if (this->wash_phase_ == WP_BLUE && idx > 0) idx = idx - 1;
    if (idx >= this->wash_total_ || idx >= WASH_MAX_STEPS) return std::string();
    unsigned seq = (unsigned) idx + 1;
    const WashStep &s = this->wash_steps_[idx];
    const char *nm = (s.name[0] != '\0') ? s.name : "?";
    const char *led = (s.led_name[0] != '\0') ? s.led_name : "-";

    // v4.0.95: 倒數格式 (remain/auto_sec s)(total_sec s)
    //   remain 計算：BLUE 階段用 blue_enter_at_；last_step_finish_at_ 啟動時覆蓋（last step GREEN）
    //   auto_sec=0 該步不倒數，顯示 (--)
    uint32_t now_ms = millis();
    uint32_t remain = 0;
    bool have_remain = false;
    if (this->wash_phase_ == WP_BLUE && this->blue_enter_at_ != 0 && s.auto_sec > 0) {
      uint32_t elapsed = (now_ms - this->blue_enter_at_) / 1000U;
      remain = (elapsed >= s.auto_sec) ? 0 : (s.auto_sec - elapsed);
      have_remain = true;
    }
    if (this->last_step_finish_at_ != 0 && s.auto_sec > 0) {
      remain = (this->last_step_finish_at_ > now_ms)
               ? (this->last_step_finish_at_ - now_ms) / 1000U : 0;
      have_remain = true;
    }
    // 總時長：流程已跑秒數（從 step[0]→step[1] 起算；done 後鎖定 wash_total_duration_s_）
    uint32_t total_s = this->wash_total_duration_s_;
    if (total_s == 0 && this->wash_total_start_ms_ != 0) {
      total_s = (now_ms - this->wash_total_start_ms_) / 1000U;
    }
    char countdown_buf[40];
    if (s.auto_sec == 0) {
      snprintf(countdown_buf, sizeof(countdown_buf), "(--)(%us)", total_s);
    } else if (have_remain) {
      snprintf(countdown_buf, sizeof(countdown_buf), "(%u/%us)(%us)",
               remain, (unsigned) s.auto_sec, total_s);
    } else {
      snprintf(countdown_buf, sizeof(countdown_buf), "(--/%us)(%us)",
               (unsigned) s.auto_sec, total_s);
    }

    char buf[200];
    // v4.0.83: 鎖定 (unlock_pwd_ 非空) → 加 "lock " 前綴；否則不顯示
    const char *lock_prefix = !this->unlock_pwd_.empty() ? "lock " : "";
    // v4.0.73: 暫停時前綴 (PAUSE)
    const char *pause_prefix = this->wash_paused_ ? "(PAUSE) " : "";
    snprintf(buf, sizeof(buf), "%s%s%s (%u/%u)BB%u/SW%u/%s%s",
             lock_prefix, pause_prefix, nm, seq, (unsigned) this->wash_total_,
             (unsigned) s.bb, (unsigned) s.r03, led, countdown_buf);
    return std::string(buf);
  }

 protected:
  bool do_connect();
  void do_disconnect();
  bool send_ws_frame(uint8_t opcode, const uint8_t *data, size_t len);
  bool send_text(const char *text, int len = -1);
  int read_one_frame();  // Returns: 1=frame processed, 0=no data, -1=error
  void process_message(const char *data, int len);
  // v4.0.42 refactor: process_message 拆分子函式（純重構，行為不變）
  //   handle_cmd_message: 解析 cmd/on/ms/arg → dispatch_cmd_doc → 組 CMD_RESULT/CMD_ERR 回覆
  //     tag="WS1" → 送 send_text；tag="LAN" → 送 lan_send_text + log [LAN] TX
  void handle_cmd_message(cJSON *doc, const char *tag);
  // v4.0.42 refactor: cJSON 欄位提取 helper — 減少 led_* 4 個 CMD 的重複模板
  static const char *json_str(cJSON *doc, const char *key, const char *def);
  static int json_int(cJSON *doc, const char *key, int def);
  // v4.0.03: send_status 支援 dashboard_only 模式
  //   dashboard_only=true：只 publish ws_last_status text_sensor（Dashboard SSE 拿）
  //                        不推 WS1 雲端、不推 WS2 LAN — 給 wash 相位變化等「UI 要新鮮狀態」的場景用
  //   dashboard_only=false（預設）：廣播 WS1 + WS2 + Dashboard — 給 CMD "status" 手動拉取用
  void send_status(bool dashboard_only = false);
  bool send_ping();
  void send_ack(const char *msg_id);
  // v3.16.31: handle_cmd 新增 source 參數 — 用於 UART2 TX log 區分來源（WS1/LAN/DASH/GO/EMG/HMI/CMD）
  std::string handle_cmd(const char *cmd, bool on, uint32_t ms, const char *arg = "", const char *source = "WS");
  // v3.16.31: 共用 CMD dispatch — 三個入口點（WS1/WS2/Dashboard）統一透過此函式執行 CMD
  //   source: "WS1" | "LAN" | "DASH" — 僅影響 log 標記與 UART2 washer TX 的來源欄位
  //   傳入已 parsed 的 cJSON doc（cmd/on/ms/arg 由此函式自行讀取）
  //   回傳：result 字串（"OK:..." 或 "ERR:..."）；doc 不在此處 delete
  std::string dispatch_cmd_doc(cJSON *doc, const char *source);
  // 底層 EVENT 發送：已組好的 JSON 字串雙發到 WS1+WS2（跳過節流）
  void send_event_raw(const char *json, size_t len);
  void check_timers(uint32_t now);
  bool press_button(const std::string &object_id);
  void publish_sensor(const std::string &object_id, const std::string &state);
  void log_to_dashboard(const std::string &msg);
  uint32_t get_backoff_with_jitter();

  // WebSocket upgrade via raw HTTP
  bool ws_handshake();

  // I/O helpers (abstract SSL vs plain TCP)
  ssize_t tls_write(const void *buf, size_t len);
  ssize_t tls_read(void *buf, size_t len);

  // FreeRTOS background connect task
  static void connect_task_fn(void *param);
  volatile ConnectState connect_state_{CS_IDLE};
  TaskHandle_t connect_task_handle_{nullptr};

  // Raw socket + mbedtls (replaces esp_tls)
  int sock_fd_{-1};
  bool ssl_active_{false};
  mbedtls_ssl_context ssl_;
  mbedtls_ssl_config ssl_conf_;
  mbedtls_entropy_context entropy_;
  mbedtls_ctr_drbg_context ctr_drbg_;
  bool mbedtls_initialized_{false};

  std::string server_;
  std::string api_key_;
  std::string device_id_;
  std::string firmware_version_;

  bool use_ssl_{false};
  int port_{0};
  bool connected_{false};
  bool enabled_{false};  // manual connect mode: starts disabled
  uint32_t connected_time_{0};
  uint32_t last_ping_time_{0};
  uint32_t last_pong_time_{0};    // Track last pong received
  bool got_server_data_{false};   // Flag: received any data from server this loop
  uint32_t last_status_time_{0};
  uint32_t last_connect_attempt_{0};
  uint32_t reconnect_interval_{5000};
  uint32_t reconnect_attempts_{0};
  uint32_t setup_time_{0};
  // v3.16.73: NVS wash state load — loop() 開機 2s 後 one-time 觸發
  //   延遲讓 modbus/switch 初始化，避免太早 wash_set_bb 打不到
  bool pending_state_load_done_{false};
  bool last_link_up_{false};      // Track Ethernet link state changes
  // v3.16.32: 記錄最近一次斷線原因（為了下次 "Connected" 印出對照，幫忙除錯）
  //   例如 "close 4001 Superseded by newer device connection (up 12s)"
  //   或 "pong timeout 30s (up 48s)" / "ping send fail" / "read error errno=104"
  std::string last_disconnect_reason_;
  char read_buf_[2048];

  // Leftover data from WS handshake (server may send frames with 101 response)
  char handshake_leftover_[512];
  int handshake_leftover_len_{0};

  // WebSocket frame reassembly
  uint8_t frame_buf_[2048];

  // Auto-off timers for CMD control
  struct CmdTimer {
    std::string target;  // object_id of the switch/fan/light
    uint8_t type;        // 0=switch, 1=fan, 2=light
    uint32_t off_at;     // millis() timestamp to turn off
    bool active;
  };
  static const int MAX_TIMERS = 8;
  CmdTimer timers_[MAX_TIMERS] = {};

  // Washer script pointer (set from YAML on_boot, 用於非阻塞 UART2 發送)
  script::Script<int, std::string, std::string> *washer_script_{nullptr};
  script::Script<int> *hmi_show_page_script_{nullptr};  // v3.16.13: YAML hmi_show_page(int page)
  // v3.16.59: YAML led_switch(name, source) — 切 Colorlight VSN 節目
  //   每個 WashStep 只做 led_switch（不再做 led_update 字幕 / scene light）
  script::Script<std::string, std::string> *led_switch_script_{nullptr};
  // v4.0.02: 直接注入其餘 3 個 LED script，dispatch_cmd_doc 略過 text_sensor 中間層
  script::Script<std::string, std::string, std::string, int, int, std::string> *led_update_script_{nullptr};
  script::Script<int> *led_brightness_script_{nullptr};
  script::Script<int> *led_volume_script_{nullptr};
  // v4.0.111: led_dualcam (order_sn, order_name, order_msg) — RPi 雙鏡頭 LED API
  script::Script<std::string, std::string, std::string> *led_dualcam_script_{nullptr};

  // v4.0.116: lock_led_sequence (vsn, order_sn, order_name, order_msg) — chain delay 避 race
  script::Script<std::string, std::string, std::string, std::string> *lock_led_sequence_script_{nullptr};

  // v4.0.99: VSN map 內部存儲 + NVS helper
  std::map<std::string, std::string> vsn_map_;
  void vsn_init_defaults();        // setup 時填入預設 keys
  void vsn_load_all_from_nvs();    // 從 NVS namespace "vsn_map" 載入覆蓋 defaults
  void vsn_save_to_nvs(const std::string &key, const std::string &name);

  // EVENT per-event 節流：同一事件 1500ms 內只送一次
  // key = event name hash (djb2), value = last send millis
  // 所有 send_event / send_washer_status 共用這張表
  // v4.0.276: 16→24 — IO event 改 name+state hash (7×2=14 slots) + washer_status，留 headroom
  static const int EMG_THROTTLE_SLOTS = 24;
  struct EmgThrottle { uint32_t hash; uint32_t last_ms; } emg_throttle_[EMG_THROTTLE_SLOTS]{};
  int emg_throttle_count_{0};
  // 節流內部 helper：回傳 true 代表應發送、false 代表應丟棄
  // v4.0.276: 加 window_ms 參數（預設 1500，IO event 傳 5000）
  bool throttle_check(uint32_t name_hash, uint32_t window_ms = 1500);

  // v3.15.26: go_enable_arg 狀態（TTL 過期自動回 "gowasher"）
  // CMD 設定：{"cmd":"go_enable_arg","arg":"lotto","ms":5000}
  // ms>0 → 計時結束後 go_enable_arg_ 回復預設 "gowasher"
  // ms=0 → 永久設定（無 TTL）
  std::string go_enable_arg_{"gowasher"};
  uint32_t go_enable_arg_expires_at_{0};  // 0 = no TTL

  // v3.15.26: emergency_enable 狀態（TTL 過期自動回 true）
  // CMD 設定：{"cmd":"emergency_enable","on":0,"ms":10000}
  // emergency_enable_=false 時按緊急按鈕完全忽略（抽獎模式）
  // ms>0 → 計時結束後 emergency_enable_ 恢復為 true（預設狀態）
  // ms=0 → 永久設定（無 TTL）
  bool emergency_enable_{true};
  uint32_t emergency_enable_expires_at_{0};  // 0 = no TTL

  // v3.16.0: EMG 自動循環 gowasher（取代 v3.15.28 的 emergency_count_ + emg_reset）
  // 按一次 EMG → 若 blue=OFF → gowasher + 1500ms 後再檢查 → 重複直到 blue=ON 或 10 次
  uint32_t emg_gowasher_next_at_{0};  // 0=未啟用；非0=下次 gowasher 的 millis deadline
  uint8_t emg_gowasher_count_{0};     // 已循環次數（上限 10）

  // v3.16.11: 按鈕軟體 debounce（500ms 內重複觸發忽略）
  //   機械按鈕彈跳可能產生 3~5 次 on_release，YAML delayed_on filter 無法過濾 OFF 方向
  //   → 在 C++ 層用時戳 gating 去重
  uint32_t go_btn_last_ms_{0};   // 上次成功處理 GO 按鈕的 millis
  uint32_t emg_btn_last_ms_{0};  // 上次成功處理 EMG 按鈕的 millis

  // v4.0.88: EMG 連按計數視窗 — 3s 內連按 3 次 → bypass 電流判斷直接觸發
  //   保護電流 sensor 失效 / Modbus box 失聯時 EMG 永遠卡在 IDLE(lowA) 的失能情境
  //   第 1 次按下（不管 gate 是否擋下）→ 強制切 warn_emg.vsn (lan) 視覺警示
  uint32_t emg_press_window_start_ms_{0};  // 計數視窗起始 ms（0=尚未開始 / 已過期）
  uint8_t emg_press_count_in_window_{0};   // 視窗內累積按下次數

  // v3.16.0: 每 loop 呼叫 — 處理 TTL 過期 + EMG 自動循環
  void check_emergency_state(uint32_t now);

  // ===== Wash Stage Counter (v3.16.0) =====
  // 追蹤洗車機 LED 相位變化，計算已完成的洗車階段數
  // wash_init CMD 初始化（帶 step ID + 泡泡編號）；Green→Blue debounced 轉換 → stage++
  // 400ms 軟體防抖過濾 ~500ms 週期的 LED 閃爍
  // v4.0.67: 新增 WP_NOPOWER — 洗車機無任何 LED 亮（可能斷電/剛開機）
  //          wash_init 收到 ""（current_led_color_ empty）時進入此相位並 REJECT
  enum WashPhase : uint8_t { WP_IDLE = 0, WP_GREEN = 1, WP_BLUE = 2, WP_RED = 3, WP_DONE = 4, WP_NOPOWER = 5 };
  struct WashStep {
    char id[8];              // 步驟 ID（如 "1423"），空項 ID 為 ""
    uint8_t bb;              // 泡泡編號 0=無, 1=bb1, 2=bb2, 3=bb3, 4=bb4
    uint8_t r03;             // v3.16.48: 軟水切換（R3 / soft_water switch）0=off, 1=on；進步驟=套用此值、WP_DONE=關閉
    // v3.16.35: Offline 模式所需的步驟 metadata（wash_init steps[] 陣列格式才會填入）
    uint32_t auto_sec;       // 進入 BLUE 後幾秒自動 gowasher（0=不自動，需手動按 GO 或 Server 送 gowasher）
    char name[16];           // 步驟名稱（如 "泡沫"），會寫進 wash_stage EVENT
    // v3.16.59: 新增 led_name — 每步驟對應的 Colorlight VSN 檔名（供 offline_led_trigger 切換節目）
    //           移除 led_text / led_color / light_color（統一只用 led_switch 切節目，source 固定 "lan"）
    char led_name[80];       // Colorlight 節目檔名（含 .vsn），空字串 → 該步驟不切節目
    // === v4.0.203 [RPI-EVENT-LED-EFF] BEGIN ===
    // led_eff CSV 字串（例 "a1,b1,c1,d1"），GREEN→BLUE edge 送 RPi event led_step 的 eff 欄位
    // 暫不執行 UDP（待整體穩定後同位置插 UDP send）
    char led_eff[40];        // 4 模組 LED 效果 CSV，空字串 → eff 欄位送空
    // === v4.0.203 [RPI-EVENT-LED-EFF] END ===
  };
  // v3.16.72: 步驟上限 12 → 18（用戶預留需求）
  static const int WASH_MAX_STEPS = 18;
  WashStep wash_steps_[WASH_MAX_STEPS]{};
  uint8_t wash_total_{0};          // wash_init 傳入的總段數
  bool wash_active_{false};        // wash_init 後 =true，重啟 =false
  uint8_t wash_stage_{0};          // 已完成階段數（GREEN→BLUE 遞增）
  WashPhase wash_phase_{WP_IDLE};  // 當前確認相位
  // v3.16.55: 流程結束旗標 — 最後一步倒數完成後 true，下次 wash_init 或重啟時 false
  //   用途：HMI 狀態列顯示 "結束"（此時 wash_active_ 已 false、stage/total 已歸零）
  bool wash_done_flag_{false};

  // v4.0.28: 洗車流程暫停機制 — EMG 安全保護
  //   背景：EMG 按下時洗車機會從綠燈變藍燈，若此步驟 auto_sec>0，會自動送 gowasher
  //         解除 EMG 鎖定 → 流程繼續跑 → 危險
  //   解法：wash_paused_ 旗標；暫停時跳過 auto_sec 倒數與 auto-continue gowasher
  //   恢復策略（用戶選 B）：EMG 解除後 **不自動恢復**，需 Dashboard 按鈕或 Server 送 wash_start
  //   倒數處理（用戶選 A）：凍結剩餘秒數；wash_start 時平移 blue_enter_at_ 讓剩餘秒數延續
  //   wash_goto 特例：跳步時自動清暫停 flag（wash_paused_=false）
  //   wash_init / wash_reset：reset flag（新流程預設繼續狀態）
  bool wash_paused_{false};
  uint32_t pause_frozen_remain_ms_{0};  // 暫停時凍結的剩餘毫秒（僅 BLUE+auto_sec>0 時有效）
  uint32_t pause_started_at_{0};        // 暫停起點 millis()（暫停時長統計用）

  // v4.0.100/v4.0.111: 訂單資訊（wash_init 帶入）
  //   order_sn   → wash_stage EVENT 一起送出（Server 對訂單關聯用）+ qr_url 拼接
  //   order_name → lock LED top text (x=320, y=0, 藍字 v4.0.111 改色)
  //   order_msg  → lock LED bottom text (x=320, y=32, 紅字) + dualcam bottom_text
  //   wash_reset / DONE 時清空
  std::string order_sn_;
  std::string order_name_;
  std::string order_msg_;
  // v4.0.215: 訂單金額 + 訂單明細 (Server 端對帳 + RPi 顯示用)
  //   order_cost    → 訂單金額字串 (例 "99" / "168")，cap 16 chars
  //   order_product → 訂單方案 / 商品名稱，cap 100 chars
  //   只進 wash_stage EVENT + STATUS，不進 LED 螢幕（避免 lock 流程改動）
  std::string order_cost_;
  std::string order_product_;

  // v4.0.12: 洗車機「當前燈色」狀態機 —
  //   current_led_color_ ∈ {"", "green", "blue", "red"}
  //   只在 OFF→ON 邊緣且 color ≠ current_led_color_ 時才更新 + 送 washer_status EVENT
  //   LED 下降沿 (OFF) 完全忽略（閃爍中依舊視為同一色）
  //   相位機 (check_wash_stage) 也改由 current_led_color_ 變化驅動，與 EVENT 同源
  std::string current_led_color_{""};
  uint32_t current_led_color_since_ms_{0};
  std::string prev_led_color_for_wash_{""};  // 相位機上一輪看到的色；用來偵測變化

  // v4.0.10: 洗車機 offline 偵測 — LED 以 500ms 週期閃爍時視為洗車機離線
  //   在 on_washer_led_change() 記錄最近一次 OFF 時間；若 OFF→ON 邊緣距上次 OFF ≤ 1200ms
  //   連續出現 ≥2 次，標記該 color flashing=true；超過 2 秒無新邊緣自動清除。
  //   washer_offline_ = (任一 color flashing) 寫進 STATUS.washer_offline
  uint32_t led_green_last_off_at_{0};
  uint32_t led_blue_last_off_at_{0};
  uint32_t led_red_last_off_at_{0};
  uint32_t led_green_last_on_at_{0};
  uint32_t led_blue_last_on_at_{0};
  uint32_t led_red_last_on_at_{0};
  uint8_t led_green_flash_hits_{0};
  uint8_t led_blue_flash_hits_{0};
  uint8_t led_red_flash_hits_{0};
  bool led_green_flashing_{false};
  bool led_blue_flashing_{false};
  bool led_red_flashing_{false};
  bool washer_offline_{false};

  // v3.16.9: 泡泡 auto-off 安全保護
  //   開啟泡泡時設定 deadline（預設 180 秒），時間到自動關閉
  //   避免程序卡住導致泡泡一直開著
  //   wash_init 可帶 "bb_timeout" (秒) 覆寫預設值
  uint32_t wash_bb_expire_at_{0};      // millis() deadline；0=未啟用
  uint32_t wash_bb_timeout_ms_{180000}; // 預設 180000 ms = 180 秒

  void check_wash_stage(uint32_t now);   // per-loop tick: debounce + phase transition
  void send_wash_stage_event();          // 送 wash_stage EVENT 到 WS1+WS2
  void send_wash_stage_restore_event();  // v4.0.55: NVS restore 專用純通知 EVENT
  void wash_all_bb_off();               // 關閉所有泡泡（bb1~bb4）
  void wash_set_bb(uint8_t bb);         // 開指定泡泡（1~4），0=不開
  // === v4.0.209 [INDOOR-LIGHT-SCENE] BEGIN ===
  // 室內燈 R15 (indoor_light2) / R16 (indoor_light3) 場景控制
  // wash 流程設計（用戶要求）：
  //   wash_init (cur 不是 red 時) → R15 ON
  //   解鎖 / 進入 step[0] GREEN → R16 ON
  //   DONE 切 welcome 之前：R15 立即 OFF + R16 排程 2 分鐘後 OFF
  //   wash_reset / cur=red destructive：R15+R16 立即 OFF + 取消排程
  // v4.0.209 改進（vs v4.0.206）：
  //   wash_init 等 cur_color 判斷後才開 R15（避免 red REJECT 時 R15 ON→OFF 閃爍）
  // Rollback：刪除 BEGIN/END 區塊 + 所有 wash_set_indoor_light 呼叫
  void wash_set_indoor_light(int n, bool on);  // n=2 or 3
  void wash_all_indoor_light_off();             // 立即關 R15+R16 + 取消排程
  uint32_t indoor_light3_off_at_{0};            // millis 截止點（0=無排程），main loop tick 關 R16
  static const uint32_t INDOOR_LIGHT3_DELAY_MS = 120000;  // 2 分鐘
  // === v4.0.209 [INDOOR-LIGHT-SCENE] END ===
  // v3.16.59: 總洗車時間統計 — 從 step[0]→step[1] 轉換開始，到 wash_done_flag_ 結束
  //   每秒在 check_wash_stage 內更新，reset 時機：wash_init（歸零）、第二步開始（設起始點）
  uint32_t wash_total_start_ms_{0};    // 計時起點（0=尚未開始）
  uint32_t wash_total_duration_s_{0};  // 最終總秒數（完成後鎖定，寫入 EVENT/STATUS）
  // v3.16.48: R03 軟水（soft_water switch）— 步驟進入時套用，WP_DONE 關閉
  void wash_apply_r03(uint8_t r03);     // r03=1→turn_on soft_water, r03=0→turn_off
  /// v3.16.8: 共用 wash_init 處理（三處入口共用）— 解析 + 初始化 + Green LED 立即偵測
  /// 回傳 result string（"OK:wash_init" 或 "ERR:..."）
  std::string handle_wash_init_doc(cJSON *doc, const char *src_tag);

  // v3.16.35: ===== Offline Mode + Auto-Continue =====
  // Server 每分鐘送 {"type":"CMD","cmd":"online"} 心跳；超過 90s 無心跳 → 進入 offline 模式
  // Offline：啟用本地流程控制（BLUE 倒數 + 自動 gowasher + 本地 LED 顯示）
  // Online：Server 完全主導，本地只維持 HMI/基本 IO，不觸發 LED / 不自動倒數
  uint32_t last_online_cmd_ms_{0};   // 最後收到 online CMD 的 millis()（0=從未收到）
  // v3.16.76: 預設 offline（開機無法保證網路已就緒）— 收到第一包 {"cmd":"online"} 才翻 false
  bool offline_mode_{true};           // 當前離線模式狀態；true → 本地接管流程
  uint32_t blue_enter_at_{0};         // 進入 WP_BLUE 的 millis()（0=未在 BLUE），用於 auto_sec 倒數
  // v4.0.93: last-step 獨立倒數結束時間（millis 截止點）— 進入 last step GREEN 時設定
  //   值=0：無 last-step 倒數中
  //   值>0：到達時觸發 DONE + welcome.vsn，與 BLUE auto-continue 倒數獨立運作
  //   設計目的：避免依賴 GREEN→BLUE edge 才啟動倒數（用戶反映機器在 last step 卡 BLUE 不切 GREEN）
  uint32_t last_step_finish_at_{0};
  // v4.0.224: 每 step 純 GREEN 期間時長追蹤
  //   step_green_enter_ms_ = 進入該 step GREEN 的 millis() (0=未進入)
  //   step_durations_s_[i] = step i 完成 GREEN 的純秒數 (含 idle GREEN 等候 GO + auto_sec 倒數)
  //   開始計時：BLUE→GREEN edge / IDLE→GREEN / RED→GREEN / wash_init non-lock GREEN / wash_unlock GREEN
  //   停止計時：GREEN→BLUE edge (該 step 結束 GREEN)
  //   DONE last step：last_step_finish_at_ trigger / auto-continue last
  //   wash_complete EVENT 把 array 全送出 (用戶 v4.0.224 設計)
  uint32_t step_green_enter_ms_{0};
  uint16_t step_durations_s_[WASH_MAX_STEPS]{0};
  // === v4.0.202 [1MIN-VSN] BEGIN ===
  // BLUE 倒數剩 60s 時 led_switch("1min") 一次性 fire flag
  // GREEN→BLUE edge 重置為 false；fire 後設 true 防止重複觸發
  // wash_reset / cur=red 清空狀態時也重置
  // Rollback：刪除本欄位 + 所有引用點
  bool one_min_fired_{false};
  // === v4.0.202 [1MIN-VSN] END ===
  void check_offline_state(uint32_t now);   // 1s 節流：檢測 heartbeat 超時 + 狀態轉換
  // v3.16.59: 離線時觸發 Colorlight 節目切換（僅呼叫 led_switch 腳本）
  //   移除 scene light（status_light_1/2 已拔）與 led_update（字幕）— 只切 VSN 節目
  void offline_led_trigger(uint8_t step_idx);

  // v3.16.59: 追蹤最近一次切換的 Colorlight 節目檔名（STATUS 顯示用）
  //   兩個來源：(1) online led_switch CMD  (2) offline_led_trigger
  std::string last_led_name_;
  std::string last_led_source_;

  // v4.0.23: 上次 send_wash_stage_event 觸發 offline_led_trigger 的 step_idx
  //   同一 step_idx 內 GREEN→BLUE 相位轉換時，VSN 檔名不變，不需要再送 HTTP PUT
  //   -1 = 尚未觸發過（wash_init/wash_reset 後必定會送一次）
  int8_t last_led_step_idx_{-1};

  // ===== HMI Password System (v3.16.30 簡化) =====
  // 工程模式密碼：預設為裝置序號末四碼（lazy init in on_hmi_password_submit）
  //   Server 可主動送 set_debug_pwd CMD 覆蓋（{"cmd":"set_debug_pwd","pwd":"5678"}）
  //   不再主動送 get_debug_pwd EVENT — 避免 WS1 斷線重連時重送
  //   僅在 HMI 按密碼確認鍵 (0x28) 時才送 hmi_password EVENT 到 WS1+WS2
  std::string debug_pwd_;              // 預設序號末四碼；set_debug_pwd CMD 可覆蓋

  // v4.0.67: 店鋪解鎖密碼 — wash_init 帶 unlock_pwd 字串即進入鎖定模式
  //   非空 → 鎖定中：wash_active_=true 但 phase=WP_IDLE、go_enable_arg_="lock"、播 lock.vsn
  //   空   → 正常流程（由 current_led_color_ 驅動 state machine）
  //   解鎖兩路：(1) CMD wash_unlock  (2) HMI 輸入密碼 == unlock_pwd_
  //   NVS 一併持久化（重啟後鎖定狀態會恢復）
  std::string unlock_pwd_;
  void do_wash_unlock(const char *src);
 public:
  void wash_unlock_public(const char *src) { this->do_wash_unlock(src); }
 protected:

  // v4.0.85: 電流確認 (EMG GREEN 分支用) + HMI alert 字串
  sensor::Sensor *current_total_sensor_{nullptr};
  std::string hmi_emg_alert_buf_;
  // 電流閾值（A）— 低於此值視為機器待命，EMG 不觸發推進
  static constexpr float EMG_CURRENT_IDLE_THRESHOLD_A = 0.6f;

  // helper — 查 binary_sensor 的當前狀態（object_id 對應）
  bool bs_state(const char *oid);

  // helper — HMI 顯示文字/頁面（直接寫 UART0 DWIN 協議）
  // hmi_show_page 同步更新 hmi_current_page_，YAML 讀此變數控制是否推送監控數據
  void hmi_show_page(int page);
  void hmi_show_text(int x, int y, const char *text);

  // v3.16.27: hmi_auth_result idempotent — 防止 Server 斷線重連後重發造成 HMI 重繪風暴
  //   每次 Server 重連會重送 hmi_auth_result → hmi_show_page(0) 會做完整重繪（20+ icon）
  //   重繪過程 100+ UART0 byte 可能干擾 HMI 觸摸狀態，導致 HMI 回送殘留按鍵事件
  //   解法：短時間內（<3s）已處理過相同 page 的 auth → skip 完整重繪
  uint32_t last_hmi_auth_at_{0};      // 上次處理 hmi_auth_result 的 millis()
  int last_hmi_auth_page_{-1};        // 上次處理的 page 編號

  // UART2 monitor (washer TX/RX hex log)
  void uart2_log_tx(const unsigned char *data, int len);
  void uart2_check_rx();

  // Deferred reboot (allows CMD reply to be sent first)
  // v3.15.22: 兩段式重啟
  //   Phase 1 (+500ms): App.safe_reboot() — 乾淨關閉元件後重啟
  //   Phase 2 (+5500ms): 若 safe_reboot 返回或卡住後 main loop 仍活著 → esp_restart() 強制
  bool reboot_pending_{false};
  bool reboot_safe_triggered_{false};
  uint32_t reboot_requested_at_{0};

  // ===== LAN WebSocket Client (second connection, plain TCP) =====
  bool lan_do_connect();
  void lan_do_disconnect();
  bool lan_ws_handshake();
  bool lan_send_text(const char *text, int len = -1);
  bool lan_send_ws_frame(uint8_t opcode, const uint8_t *data, size_t len);
  int  lan_read_one_frame();
  void lan_process_message(const char *data, int len);
  void lan_send_status();
  static void lan_connect_task_fn(void *param);

  std::string lan_server_;
  int lan_port_{0};
  bool lan_use_ssl_{false};
  int lan_sock_fd_{-1};
  bool lan_connected_{false};
  bool lan_enabled_{true};  // auto-connect by default
  uint32_t lan_connected_time_{0};
  uint32_t lan_last_ping_time_{0};
  uint32_t lan_last_pong_time_{0};
  bool lan_got_server_data_{false};
  uint32_t lan_last_status_time_{0};
  uint32_t lan_last_connect_attempt_{0};
  uint32_t lan_reconnect_interval_{5000};
  uint32_t lan_reconnect_attempts_{0};
  bool lan_last_link_up_{false};
  volatile ConnectState lan_connect_state_{CS_IDLE};
  TaskHandle_t lan_connect_task_handle_{nullptr};
  // CS_CONNECTING watchdog：記錄進入 CONNECTING 的時間，超過 20s 強制 CS_FAILED
  // 防止背景 task 卡在 ::connect() 的 lwIP SYN timeout（預設 63s）導致 UI 黃燈回不來
  uint32_t lan_connecting_since_{0};
  char lan_read_buf_[2048];
  uint8_t lan_frame_buf_[2048];
  char lan_handshake_leftover_[512];
  int lan_handshake_leftover_len_{0};

  // === v4.0.238 [DEVICE-REGISTRY] BEGIN ===
  // 外部裝置 registry — 以 device_id 為 key，DHCP 友善（不寫死 IP）
  // 設備透過 POST /text/device_announce_in/set?value=<json> 自報
  // 60s 沒 announce 視為 offline (STATUS 仍列出但 age_s 顯示)
  // 5min stale → STATUS 略過顯示
  struct DeviceInfo {
    std::string id;          // "led-mod-1"
    std::string ip;          // "192.168.1.105" (DHCP 動態)
    std::string ver;         // "1.1.0"
    std::string paired_sn;   // "1140" (= 主板 device_id_) 或 "" 未綁
    std::string bssid;       // "AA:BB:CC:DD:EE:FF"（連的 AP）
    int rssi{0};             // 訊號強度 dBm
    uint32_t last_seen_ms{0};
  };
  std::map<std::string, DeviceInfo> device_registry_;
  // === v4.0.238 [DEVICE-REGISTRY] END ===

  // === v4.0.278 [DEVICE-OUT-BUS] BEGIN ===
  // 全域 monotonic 序號，每次 publish_device_bus / publish_device_command 加 1
  // 重啟後從 0 開始（裝置 last_seen_seq 大於當前 → 視為 mainboard reboot，重新 sync）
  uint32_t device_bus_seq_{0};
  // === v4.0.278 [DEVICE-OUT-BUS] END ===
};

}  // namespace ws_client
}  // namespace esphome
