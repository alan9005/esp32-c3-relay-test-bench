#include "ws_client.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include "../colorlight/colorlight.h"  // v4.0.279: cl_reset CMD calls ColorlightClient::soft_reset

#include <cstring>
#include <cstdlib>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <netinet/tcp.h>
#include "driver/uart.h"
#include "esp_system.h"  // v3.15.21: for esp_restart() in deferred reboot
#include "esp_random.h"  // v4.0.00: esp_random() replaces Arduino random(min, max)
#include "esp_heap_caps.h"  // v4.0.00: esp_get_free_heap_size() replaces esp_get_free_heap_size()
#include "nvs.h"         // v3.16.73: wash state persistence via ESP32 NVS
#include "nvs_flash.h"

// For base64 encoding of WebSocket key
extern "C" {
#include "mbedtls/base64.h"
#include "mbedtls/sha1.h"
#include "mbedtls/error.h"
#include "mbedtls/net_sockets.h"
}

// v4.0.16: 全域 reboot tag 定義（宣告在 ws_client.h，所有組件共用）
//   任何重啟路徑呼叫前填入，YAML 3s breadcrumb interval 讀取並寫 EEPROM
// v4.0.60: 加 RTC_NOINIT breadcrumb — panic/crash/esp_restart 後保留，不依賴 EEPROM I2C
//   reboot_count=0xFFFFFFFF 代表 EEPROM 讀失敗，crumb 就完全丟失。RTC_NOINIT 繞過 I2C
//   用來抓 cl.put() crash 的確切步驟（CL_PARSE_URL / CL_DNS / CL_SOCKET / CL_CONNECT / CL_SEND）
#include "esp_attr.h"
RTC_NOINIT_ATTR uint32_t g_rtc_crumb_magic;
RTC_NOINIT_ATTR char     g_rtc_crumb_tag[28];
static constexpr uint32_t RTC_CRUMB_MAGIC = 0x48424352;  // 'HBCR' HuiFlow Breadcrumb

extern "C" {
  char g_reboot_tag[32] = {0};
  // v4.0.71: 一旦寫入 "REBOOT_*" tag（caller 明確打算觸發 reboot）就 latch，
  //   後續非 REBOOT_* 的寫入全部忽略。避免 check_wash_stage / HMI_RX 等 loop-tick
  //   tag 在 reboot 延遲窗口（500ms+）內覆蓋掉真正的 reboot 兇手 tag。
  //   開機 on_boot 讀取後會 clear_rtc_crumb_c() 解除 latch。
  static bool g_reboot_tag_latched = false;
  void set_reboot_tag_c(const char *tag) {
    if (tag == nullptr) return;
    bool is_reboot = (tag[0] == 'R' && tag[1] == 'E' && tag[2] == 'B' &&
                      tag[3] == 'O' && tag[4] == 'O' && tag[5] == 'T' &&
                      tag[6] == '_');
    // v4.0.71: 已 latch 且新 tag 不是 REBOOT_* → 忽略
    if (g_reboot_tag_latched && !is_reboot) return;
    size_t i = 0;
    for (; i < 31 && tag[i]; i++) g_reboot_tag[i] = tag[i];
    g_reboot_tag[i] = '\0';
    // v4.0.60: 同步寫 RTC（27 char + null，夠放 CL_CONNECT_START 這類長 tag）
    size_t j = 0;
    for (; j < 27 && tag[j]; j++) g_rtc_crumb_tag[j] = tag[j];
    g_rtc_crumb_tag[j] = '\0';
    g_rtc_crumb_magic = RTC_CRUMB_MAGIC;
    // v4.0.71: 若本次寫的是 REBOOT_*，latch 起來阻擋後續非 REBOOT tag 寫入
    if (is_reboot) g_reboot_tag_latched = true;
  }
  // v4.0.60: 開機讀取最後 tag；magic 不符回 nullptr。讀完呼叫 clear_rtc_crumb_c() 清掉
  const char *get_last_rtc_crumb_c() {
    if (g_rtc_crumb_magic != RTC_CRUMB_MAGIC) return nullptr;
    g_rtc_crumb_tag[27] = '\0';
    return g_rtc_crumb_tag;
  }
  void clear_rtc_crumb_c() {
    g_rtc_crumb_magic = 0;
    g_rtc_crumb_tag[0] = '\0';
    // v4.0.71: 開機後解除 latch，下次 runtime 可以再寫任何 tag
    g_reboot_tag_latched = false;
  }
}

namespace esphome {
namespace ws_client {

static const char *TAG = "ws_client";
// LAN_TAG moved up from line ~2288 so loop() 可以用它印 lan_connected_ 狀態轉換
static const char *LAN_TAG = "ws_lan";

// Max consecutive failures before switching to slow retry (5 min interval)
static const uint32_t MAX_FAST_RETRIES = 10;
// Slow retry interval after MAX_FAST_RETRIES exhausted
static const uint32_t SLOW_RETRY_INTERVAL_MS = 60000;  // 1 minute

// =========================================================================
// UART2 洗車機指令表（靜態，供 get_washer_frame() 和 YAML washer_send_cmd 使用）
// 7-byte frame: FD 03 C7 50 {CMD} {CHK} DF
// =========================================================================
static const unsigned char WASHER_TABLE[10][7] = {
  {0xfd, 0x03, 0xc7, 0x50, 0x82, 0x4a, 0xdf},  // 1: startwasher1
  {0xfd, 0x03, 0xc7, 0x50, 0x88, 0x4b, 0xdf},  // 2: startwasher2
  {0xfd, 0x03, 0xc7, 0x50, 0x81, 0x4a, 0xdf},  // 3: startwasher3
  {0xfd, 0x03, 0xc7, 0x50, 0x84, 0x4b, 0xdf},  // 4: startwasher4
  {0xfd, 0x03, 0xc7, 0x50, 0x8c, 0x4b, 0xdf},  // 5: startwasher5
  {0xfd, 0x03, 0xc7, 0x50, 0x83, 0x4b, 0xdf},  // 6: startwasher6
  {0xfd, 0x03, 0xc7, 0x50, 0x85, 0x4b, 0xdf},  // 7: gowasher
  {0xfd, 0x03, 0xc7, 0x50, 0x8a, 0x4b, 0xdf},  // 8: resetwasher
  {0xfd, 0x03, 0xc7, 0x50, 0x89, 0x43, 0xdf},  // 9: hardresetwasher
  {0xfd, 0x03, 0xc7, 0x50, 0x90, 0x3c, 0xdf}   // 10: sparewasher
};

/// 取得 UART2 洗車機指令 frame（供 YAML washer_send_cmd script 調用）
/// @param cmd_num 1~10 對應 startwasher1~6, gowasher, resetwasher, hardresetwasher, sparewasher
/// @return 7-byte frame 指針，無效 cmd_num 返回 nullptr
const unsigned char* WSClientComponent::get_washer_frame(int cmd_num) {
  if (cmd_num >= 1 && cmd_num <= 10) return WASHER_TABLE[cmd_num - 1];
  return nullptr;
}

// =========================================================================
// mbedtls BIO callbacks: send/recv on raw socket fd
// =========================================================================

static int mbedtls_net_send_cb(void *ctx, const unsigned char *buf, size_t len) {
  int fd = *(int *)ctx;
  ssize_t ret = ::send(fd, buf, len, 0);
  if (ret < 0) {
    int err = errno;
    if (err == EAGAIN || err == EWOULDBLOCK) return MBEDTLS_ERR_SSL_WANT_WRITE;
    return MBEDTLS_ERR_NET_SEND_FAILED;
  }
  return (int)ret;
}

static int mbedtls_net_recv_cb(void *ctx, unsigned char *buf, size_t len) {
  int fd = *(int *)ctx;
  ssize_t ret = ::recv(fd, buf, len, 0);
  if (ret < 0) {
    int err = errno;
    if (err == EAGAIN || err == EWOULDBLOCK) return MBEDTLS_ERR_SSL_WANT_READ;
    return MBEDTLS_ERR_NET_RECV_FAILED;
  }
  // ret == 0 means TCP FIN (peer closed); return 0 to signal EOF to mbedtls
  return (int)ret;
}

// =========================================================================
// FreeRTOS background connect task
// =========================================================================

void WSClientComponent::connect_task_fn(void *param) {
  WSClientComponent *self = static_cast<WSClientComponent *>(param);
  bool ok = self->do_connect();
  self->connect_state_ = ok ? CS_CONNECTED : CS_FAILED;
  self->connect_task_handle_ = nullptr;
  vTaskDelete(nullptr);  // delete this task
}

// =========================================================================
// Setup / Manual Connect / Disconnect
// =========================================================================

void WSClientComponent::setup() {
  this->setup_time_ = millis();
  // v4.0.99: VSN map 初始化（先填預設值，再從 NVS 覆蓋）
  this->vsn_init_defaults();
  this->vsn_load_all_from_nvs();
  // v4.0.60: 讀 RTC breadcrumb — 若上次是 panic/crash，這裡會顯示最後步驟
  {
    const char *last = ::get_last_rtc_crumb_c();
    if (last != nullptr && last[0] != '\0') {
      ESP_LOGW(TAG, "=== LAST RTC CRUMB: %s ===", last);
      char buf[64];
      snprintf(buf, sizeof(buf), "[CRUMB] last=%s", last);
      this->publish_sensor("ws_log", buf);
      this->publish_sensor("last_rtc_crumb", last);
      ::clear_rtc_crumb_c();
    } else {
      this->publish_sensor("last_rtc_crumb", "(none)");
    }
  }
  ESP_LOGI(TAG, "WebSocket client ready, server: %s:%d ssl=%d (auto-connect in 10s)",
           this->server_.c_str(), this->port_ ? this->port_ : (this->use_ssl_ ? 443 : 80), this->use_ssl_);
  this->publish_sensor("ws_connection", "Waiting (10s)...");
  // Auto-connect on boot after 10s delay
  this->enabled_ = true;
  this->last_connect_attempt_ = this->setup_time_;  // will connect at setup_time + reconnect_interval
  this->reconnect_interval_ = 10000;  // 10s initial delay
  // LAN WS client (second connection to RPi/local server)
  if (!this->lan_server_.empty()) {
    ESP_LOGI(TAG, "LAN WS client ready, server2: %s:%d ssl=%d (auto-connect in 10s)",
             this->lan_server_.c_str(), this->lan_port_ ? this->lan_port_ : 80, this->lan_use_ssl_);
    this->lan_enabled_ = true;
    this->lan_last_connect_attempt_ = this->setup_time_;
    this->lan_reconnect_interval_ = 10000;
    this->publish_sensor("ws_lan_connection", "Waiting (10s)...");
    // Publish server address for Dashboard display
    char srv_buf[80];
    snprintf(srv_buf, sizeof(srv_buf), "%s:%d", this->lan_server_.c_str(),
             this->lan_port_ ? this->lan_port_ : 80);
    this->publish_sensor("ws_lan_server_info", srv_buf);
  }
}

void WSClientComponent::manual_connect() {
  if (this->connected_) {
    ESP_LOGW(TAG, "Already connected");
    return;
  }
  if (this->connect_state_ == CS_CONNECTING) {
    ESP_LOGW(TAG, "Connection already in progress");
    return;
  }
  ESP_LOGI(TAG, "Manual connect requested");
  this->enabled_ = true;
  this->reconnect_attempts_ = 0;
  this->reconnect_interval_ = 1000;  // connect quickly
  this->last_connect_attempt_ = 0;   // connect on next loop
  this->publish_sensor("ws_connection", "Connecting...");
  this->log_to_dashboard("Manual connect requested");
}

void WSClientComponent::manual_disconnect() {
  ESP_LOGI(TAG, "Manual disconnect requested");
  this->enabled_ = false;
  if (this->connected_) {
    this->connected_ = false;
    this->do_disconnect();
    this->publish_sensor("ws_connection", "Disconnected");
    this->log_to_dashboard("Disconnected (manual)");
  } else {
    this->publish_sensor("ws_connection", "Idle");
  }
}

// v4.0.91: WS1 LAN test switch — 重做（v4.0.88 失敗版撤回後重新設計）
//   v4.0.88 失敗原因（推測）：用 manual_disconnect + 等 loop reconnect，但
//     - 截圖顯示 server_ 改了之後，後續 DNS 仍解析 wss.huiflow.com
//     - 可能 connect_task 內部 capture 了舊 server_ 或時序問題
//   v4.0.91 改進：
//     (1) 每階段都明確 log_to_dashboard，方便 user 從 Dashboard log 判斷是否觸發
//     (2) 強制 disconnect 後直接設 enabled_=true + 清 attempt 狀態
//     (3) 呼叫 manual_connect()（不只設旗標），由 loop 用最新 server_ 觸發 connect_task
void WSClientComponent::switch_ws1_to_lan_test() {
  this->log_to_dashboard("[WS1-SWITCH] button pressed — server -> 192.168.1.99");
  ESP_LOGW(TAG, "[WS1-SWITCH] === SWITCH TO LAN TEST ===");
  ESP_LOGW(TAG, "[WS1-SWITCH] OLD server='%s' port=%d ssl=%d",
           this->server_.c_str(), this->port_, this->use_ssl_ ? 1 : 0);

  // (1) 改 server_（in-memory，重開機從 YAML 還原）
  this->server_ = "192.168.1.99";
  ESP_LOGW(TAG, "[WS1-SWITCH] NEW server='%s' (port/ssl unchanged)", this->server_.c_str());
  this->log_to_dashboard("[WS1-SWITCH] server_ now='192.168.1.99'");

  // (2) 順便重置洗車流程
  // v4.0.219: 帶 source — wash_reset EVENT 會標 "WS1-SWITCH"
  this->wash_reset_public("WS1-SWITCH");
  this->log_to_dashboard("[WS1-SWITCH] wash flow reset");

  // (3) 強制斷開既有連線（如果有）
  if (this->connected_) {
    this->connected_ = false;
    this->do_disconnect();
    this->log_to_dashboard("[WS1-SWITCH] disconnected old TLS session");
  }
  // 強制清掉所有 reconnect cooldown
  this->enabled_ = true;
  this->reconnect_attempts_ = 0;
  this->reconnect_interval_ = 1000;
  this->last_connect_attempt_ = 0;
  this->connect_state_ = CS_IDLE;  // 清掉可能殘留的 CS_FAILED/CS_CONNECTING

  // (4) 呼叫 manual_connect 立刻觸發（內部設定下次 loop 嘗試 connect）
  //     loop 會 spawn connect_task → do_connect 讀當前 server_ → DNS 解析 192.168.1.99
  this->manual_connect();
  this->log_to_dashboard("[WS1-SWITCH] manual_connect() called — loop will spawn task");
  ESP_LOGW(TAG, "[WS1-SWITCH] === END (waiting for connect_task) ===");
}

// === v4.0.241 [WASHER-REDABORTED] BEGIN ===
// 流程進行中（wash_active_=true）洗車機突然進紅燈 → 送 washer_redaborted EVENT
// 必須在 check_wash_stage *→RED 的 destructive reset 前呼叫（要讀當前狀態）
// 帶完整訂單資訊 + 中斷瞬間 step 快照 + durations[]（已完成各 step 純 GREEN 秒數）
void WSClientComponent::send_washer_redaborted_event() {
  // 計算 step_idx (同 send_wash_stage_event 邏輯)
  int step_idx = this->wash_stage_;
  if (this->wash_phase_ == WP_BLUE && step_idx > 0) step_idx -= 1;
  const char *step_name = "";
  const char *step_id = "";
  if (step_idx >= 0 && step_idx < (int)this->wash_total_) {
    step_name = this->wash_steps_[step_idx].name;
    step_id = this->wash_steps_[step_idx].id;
  }

  // phase string
  static const char *phase_names[] = {"idle", "green", "blue", "red", "done", "nopower"};
  const char *ps = (this->wash_phase_ <= WP_NOPOWER) ? phase_names[this->wash_phase_] : "unknown";
  // 鎖定中覆蓋為 "lock"
  if (!this->unlock_pwd_.empty()) ps = "lock";

  // elapsed_sec
  uint32_t elapsed_sec = 0;
  if (this->wash_total_start_ms_ != 0) {
    elapsed_sec = (millis() - this->wash_total_start_ms_) / 1000U;
  }

  // 用 cJSON 安全組裝（order 欄位含中文 / 特殊字元 escape 安全）
  cJSON *ev = cJSON_CreateObject();
  cJSON_AddStringToObject(ev, "type", "EVENT");
  cJSON_AddStringToObject(ev, "event", "washer_redaborted");
  // 訂單資訊（給 server 退款用）
  cJSON_AddStringToObject(ev, "order_sn", this->order_sn_.c_str());
  cJSON_AddStringToObject(ev, "order_name", this->order_name_.c_str());
  cJSON_AddStringToObject(ev, "order_msg", this->order_msg_.c_str());
  cJSON_AddStringToObject(ev, "order_cost", this->order_cost_.c_str());
  cJSON_AddStringToObject(ev, "order_product", this->order_product_.c_str());
  // 中斷瞬間 step 快照
  cJSON_AddNumberToObject(ev, "aborted_stage", step_idx + 1);  // 1-based
  cJSON_AddNumberToObject(ev, "aborted_total", (int)this->wash_total_);
  cJSON_AddStringToObject(ev, "aborted_phase", ps);
  cJSON_AddNumberToObject(ev, "aborted_step_seq", step_idx + 1);
  cJSON_AddStringToObject(ev, "aborted_step_id", step_id);
  cJSON_AddStringToObject(ev, "aborted_step_name", step_name);
  cJSON_AddNumberToObject(ev, "elapsed_sec", (int)elapsed_sec);
  // durations[] — 已完成各 step 純 GREEN 秒數（同 wash_complete 風格）
  cJSON *dur_arr = cJSON_CreateArray();
  for (uint8_t i = 0; i < this->wash_total_ && i < WASH_MAX_STEPS; i++) {
    cJSON_AddItemToArray(dur_arr, cJSON_CreateNumber(this->step_durations_s_[i]));
  }
  cJSON_AddItemToObject(ev, "durations", dur_arr);

  char *out = cJSON_PrintUnformatted(ev);
  if (out) {
    this->send_event_raw(out, strlen(out));
    char log[200];
    snprintf(log, sizeof(log),
             "[WASHER-REDABORTED] order=%s stage=%d/%u step=%s elapsed=%us → server",
             this->order_sn_.c_str(), step_idx + 1, (unsigned)this->wash_total_,
             step_name, (unsigned)elapsed_sec);
    ESP_LOGW(TAG, "%s", log);
    this->log_to_dashboard(log);
    free(out);
  }
  cJSON_Delete(ev);
}
// === v4.0.241 [WASHER-REDABORTED] END ===

// === v4.0.238 [DEVICE-REGISTRY] BEGIN ===
// 外部裝置自報接收 — YAML text entity device_announce_in on_value 呼叫
// JSON: {"id":"led-mod-1","ip":"192.168.1.105","ver":"1.1.0","paired_sn":"1140","bssid":"...","rssi":-65}
// SN 不匹配主板 device_id_ → 拒絕（rogue 設備保護，譬如鄰店誤連）
// 維護內部 registry，STATUS 帶 paired_devices array
void WSClientComponent::on_device_announce(const std::string &json) {
  if (json.empty()) return;
  cJSON *doc = cJSON_Parse(json.c_str());
  if (!doc) {
    ESP_LOGW(TAG, "[DEV-ANNOUNCE] invalid JSON: %.80s", json.c_str());
    return;
  }
  const char *id = json_str(doc, "id", "");
  const char *ip = json_str(doc, "ip", "");
  const char *ver = json_str(doc, "ver", "");
  const char *paired_sn = json_str(doc, "paired_sn", "");
  const char *bssid = json_str(doc, "bssid", "");
  int rssi = json_int(doc, "rssi", 0);

  if (!*id) {
    ESP_LOGW(TAG, "[DEV-ANNOUNCE] missing id");
    cJSON_Delete(doc);
    return;
  }
  // SN 校驗：paired_sn 非空時必須等於主板 device_id_，否則拒絕（rogue 防護）
  // paired_sn 為空 = 未綁設備，允許進 registry（給配對用）
  if (*paired_sn && this->device_id_ != paired_sn) {
    char log[160];
    snprintf(log, sizeof(log),
             "[DEV-ANNOUNCE] SN MISMATCH id=%s reported='%s' main='%s' (REJECT)",
             id, paired_sn, this->device_id_.c_str());
    ESP_LOGW(TAG, "%s", log);
    this->log_to_dashboard(log);
    cJSON_Delete(doc);
    return;
  }

  // 寫入/更新 registry（first-seen 時 log 出來，後續 silent 避免 spam）
  bool first_seen = (this->device_registry_.find(id) == this->device_registry_.end());
  DeviceInfo &info = this->device_registry_[id];
  info.id = id;
  info.ip = ip;
  info.ver = ver;
  info.paired_sn = paired_sn;
  info.bssid = bssid;
  info.rssi = rssi;
  info.last_seen_ms = millis();

  if (first_seen) {
    char log[180];
    snprintf(log, sizeof(log),
             "[DEV] FIRST-SEEN id=%s ip=%s ver=%s paired_sn=%s rssi=%d",
             id, ip, ver, *paired_sn ? paired_sn : "(unbound)", rssi);
    ESP_LOGI(TAG, "%s", log);
    this->log_to_dashboard(log);
  } else {
    ESP_LOGD(TAG, "[DEV] announce id=%s ip=%s rssi=%d age=0s", id, ip, rssi);
  }
  cJSON_Delete(doc);
}

// 外部裝置事件上報 — YAML text entity device_event_in on_value 呼叫
// JSON: {"id":"button-1","action":"press","short":3,"paired_sn":"1140"}
// SN 匹配後透過 WS event 上報 server (WS1+WS2)
void WSClientComponent::on_device_event(const std::string &json) {
  if (json.empty()) return;
  cJSON *doc = cJSON_Parse(json.c_str());
  if (!doc) {
    ESP_LOGW(TAG, "[DEV-EVENT] invalid JSON: %.80s", json.c_str());
    return;
  }
  const char *id = json_str(doc, "id", "");
  const char *action = json_str(doc, "action", "");
  const char *paired_sn = json_str(doc, "paired_sn", "");

  if (!*id || !*action) {
    ESP_LOGW(TAG, "[DEV-EVENT] missing id or action");
    cJSON_Delete(doc);
    return;
  }
  // SN 校驗：必須等於主板 device_id_，否則拒絕
  if (this->device_id_ != paired_sn) {
    char log[160];
    snprintf(log, sizeof(log),
             "[DEV-EVENT] SN MISMATCH id=%s reported='%s' main='%s' (REJECT)",
             id, paired_sn, this->device_id_.c_str());
    ESP_LOGW(TAG, "%s", log);
    this->log_to_dashboard(log);
    cJSON_Delete(doc);
    return;
  }

  // 透過 WS event 上報 (WS1+WS2)
  // {"type":"EVENT","event":"device_event","device_id":"button-1","action":"press","short":N,...}
  // 把 doc 內所有非 paired_sn 欄位帶過去（簡單做法：直接序列化 doc）
  // 移除 paired_sn 欄位後輸出
  cJSON_DeleteItemFromObject(doc, "paired_sn");
  // 加 type / event 欄位
  cJSON_AddStringToObject(doc, "type", "EVENT");
  cJSON_AddStringToObject(doc, "event", "device_event");
  // rename "id" → "device_id" 對齊 v4.0.x EVENT 風格
  cJSON *id_item = cJSON_DetachItemFromObject(doc, "id");
  if (id_item) {
    cJSON_AddItemToObject(doc, "device_id", id_item);
  }
  // v4.0.243: 加 order_sn（流程進行中關聯訂單）
  if (!this->order_sn_.empty()) {
    cJSON_AddStringToObject(doc, "order_sn", this->order_sn_.c_str());
  }
  char *out = cJSON_PrintUnformatted(doc);
  if (out) {
    this->send_event_raw(out, strlen(out));
    char log[200];
    snprintf(log, sizeof(log), "[DEV-EVENT] id=%s action=%s → WS forwarded", id, action);
    ESP_LOGI(TAG, "%s", log);
    this->log_to_dashboard(log);
    free(out);
  }
  cJSON_Delete(doc);
}
// === v4.0.238 [DEVICE-REGISTRY] END ===


// === v4.0.278 [DEVICE-OUT-BUS] BEGIN ===
// 主板 → Tier 2 裝置 outbound 廣播（plan 方案 A 變體 — REST polling）
//
// 為何不用 WS server：
//   1. v3.14.2 已移除本地 WS server (port 81)，從頭實作要 800+ 行
//   2. ESPHome web_server 不支援 custom HTTP endpoint (Stage 21 已驗證)
//   3. 改走 ESPHome 內建 text_sensor entity REST API：
//      device_event_bus  ← 廣播事件（wash_stage / EMG / device 事件等）
//      device_cmd_out    ← per-target 指令
//   裝置 GET /text_sensor/<id> 拉資料，用 seq 號 dedup（裝置記住 last_seen_seq）
//
// 設計目的：plan Tier 2 裝置（LED 效果 / 販賣機 / 輔助感測 ESP32）可從主板接事件
// 與指令，主板掛時裝置 fallback 預設行為（NVS 預存）。Tier 1 / 三件套不受影響。

void WSClientComponent::publish_device_bus(const std::string &event_json) {
  this->device_bus_seq_++;
  // 用 cJSON 建外層 wrapper {"seq":N,"event":<inner>}
  cJSON *doc = cJSON_CreateObject();
  if (!doc) return;
  cJSON_AddNumberToObject(doc, "seq", (double) this->device_bus_seq_);
  cJSON *inner = cJSON_Parse(event_json.c_str());
  if (inner) {
    cJSON_AddItemToObject(doc, "event", inner);
  } else {
    // 不是合法 JSON 就當字串塞進去
    cJSON_AddStringToObject(doc, "event", event_json.c_str());
  }
  char *out = cJSON_PrintUnformatted(doc);
  if (out) {
    this->publish_sensor("device_event_bus", std::string(out));
    free(out);
  }
  cJSON_Delete(doc);
}

void WSClientComponent::publish_device_command(const std::string &target_id,
                                                const std::string &cmd_json) {
  this->device_bus_seq_++;
  cJSON *doc = cJSON_CreateObject();
  if (!doc) return;
  cJSON_AddNumberToObject(doc, "seq", (double) this->device_bus_seq_);
  cJSON_AddStringToObject(doc, "target", target_id.c_str());
  cJSON *cmd = cJSON_Parse(cmd_json.c_str());
  if (cmd) {
    cJSON_AddItemToObject(doc, "cmd", cmd);
  } else {
    cJSON_AddStringToObject(doc, "cmd", cmd_json.c_str());
  }
  char *out = cJSON_PrintUnformatted(doc);
  if (out) {
    this->publish_sensor("device_cmd_out", std::string(out));
    char log[160];
    snprintf(log, sizeof(log), "[DEV-CMD] target=%s seq=%u", target_id.c_str(),
             (unsigned) this->device_bus_seq_);
    ESP_LOGI(TAG, "%s", log);
    this->log_to_dashboard(log);
    free(out);
  }
  cJSON_Delete(doc);
}
// === v4.0.278 [DEVICE-OUT-BUS] END ===


// =========================================================================
// Main Loop (non-blocking — connection runs in background task)
// =========================================================================

void WSClientComponent::loop() {
  uint32_t now = millis();

  // v4.0.71: reboot_pending_ 處理移到 loop() 最頂 — 避免 check_wash_stage 等
  //   下方 function 覆蓋掉 caller 寫的 REBOOT_HMI / REBOOT_WS_CMD 等 tag。
  //   v4.0.70 新增的 CWS_ACTIVE tag 在 wash_active=true 時每個 tick 都會寫，
  //   若 reboot_pending_ 處理在下方 return 之前 check_wash_stage 已經跑過，
  //   RTC crumb 會從 REBOOT_HMI 被改寫成 CWS_ACTIVE，造成誤判為 crash。
  //   把 reboot_pending_ 提到最前面，一旦觸發就跳過所有 check_*/* tag 寫入。
  if (this->reboot_pending_) {
    if (this->reboot_requested_at_ == 0) {
      this->reboot_requested_at_ = now;
    }
    uint32_t elapsed = now - this->reboot_requested_at_;
    // Phase 1: 500ms 後觸發 safe_reboot（給 CMD_RESULT 送出的時間）
    if (!this->reboot_safe_triggered_ && elapsed >= 500) {
      ESP_LOGW(TAG, "Reboot phase 1: App.safe_reboot()");
      this->reboot_safe_triggered_ = true;
      App.safe_reboot();
      ESP_LOGW(TAG, "safe_reboot() returned unexpectedly — waiting 5s for phase 2");
    }
    // Phase 2: safe_reboot 觸發後再等 5 秒，若我們還能跑到這裡就強制硬重啟
    if (this->reboot_safe_triggered_ && elapsed >= 5500) {
      ESP_LOGW(TAG, "Reboot phase 2: force esp_restart() watchdog fallback");
      esp_restart();
    }
    return;
  }

  // v3.16.73: one-time NVS wash state load
  //   開機後延遲 2s 再 load，讓 modbus/switch 初始化完成
  //   避免太早 wash_set_bb 打不到繼電器
  if (!this->pending_state_load_done_ && this->setup_time_ != 0 &&
      (int32_t)(now - this->setup_time_) > 2000) {
    this->pending_state_load_done_ = true;
    this->wash_state_load_nvs();
  }

  // Always check auto-off timers (even when not connected/enabled)
  this->check_timers(now);

  // v3.15.26: 處理 go_enable_arg / emergency_enable TTL 過期 + emg 自動循環 gowasher
  this->check_emergency_state(now);

  // v3.16.35: Offline heartbeat 檢查（Server 每分鐘送 online CMD，超時 90s 進入離線）
  this->check_offline_state(now);

  // v3.16.0: 洗車階段計數器 — LED debounce 確認 + 相位轉換
  this->check_wash_stage(now);

  // === v4.0.211 [INDOOR-LIGHT-SCENE] BEGIN ===
  // R15 (室內燈 2) 延遲關閉排程 tick — DONE 後 2 分鐘到 → 關 R15
  // (順序與 v4.0.209 反：R16 先關，R15 後關 — 用戶要求)
  // 變數名 indoor_light3_off_at_ 保留（避免 NVS schema 改）但語義變成「R15 延遲關截止點」
  if (this->indoor_light3_off_at_ != 0 &&
      (int32_t)(now - this->indoor_light3_off_at_) >= 0) {
    this->indoor_light3_off_at_ = 0;  // 防重複觸發
    this->wash_set_indoor_light(2, false);
    ESP_LOGI(TAG, "[WASH] indoor_light2 (R15) scheduled OFF fired (2min after DONE)");
    this->log_to_dashboard("[WASH] R15 scheduled OFF fired (2min after DONE)");
  }
  // === v4.0.211 [INDOOR-LIGHT-SCENE] END ===

  // v4.0.10: 洗車機 LED 閃爍偵測衰減 — 超過 2 秒無新 OFF/ON 邊緣 → 視為穩定，清 flashing
  //   washer_offline_ = (任一 color flashing)。寫進 STATUS.washer_offline 欄位
  auto decay_flash = [&](uint32_t last_off, uint32_t last_on, uint8_t &hits, bool &flashing) {
    uint32_t ref = last_off > last_on ? last_off : last_on;
    if (ref != 0 && (uint32_t)(now - ref) > 2000) {
      hits = 0;
      flashing = false;
    }
  };
  decay_flash(this->led_green_last_off_at_, this->led_green_last_on_at_,
              this->led_green_flash_hits_, this->led_green_flashing_);
  decay_flash(this->led_blue_last_off_at_, this->led_blue_last_on_at_,
              this->led_blue_flash_hits_, this->led_blue_flashing_);
  decay_flash(this->led_red_last_off_at_, this->led_red_last_on_at_,
              this->led_red_flash_hits_, this->led_red_flashing_);
  this->washer_offline_ = this->led_green_flashing_ ||
                          this->led_blue_flashing_ ||
                          this->led_red_flashing_;

  // v3.16.79: status_pending_ drain 已移除 — send_status 現在只由 CMD "status" 手動觸發

  // v4.0.71: breadcrumb — 進 UART2 RX 檢查區段
  set_reboot_tag_c("LOOP_UART2");
  // Always check UART2 RX (washer response monitor, works via local web too)
  this->uart2_check_rx();

  // v3.15.25: check_auto_triggers() 已移除。所有 EVENT 現在由 YAML on_state
  //           lambda 直接呼叫 id(ws).send_event(name, state) 觸發。

  // ===== LAN WebSocket Client (second connection) =====
  // v4.0.71: breadcrumb — 進 LAN WS 區段（link detection / reconnect / read frame / send ping）
  set_reboot_tag_c("LOOP_LAN_WS");
  if (this->lan_enabled_ && !this->lan_server_.empty()) {
    // Link detection (shared with cloud WS)
    bool lan_link_up = false;
    auto *lan_eth = ethernet::global_eth_component;
    if (lan_eth) lan_link_up = lan_eth->is_connected();

    if (this->lan_last_link_up_ && !lan_link_up) {
      if (this->lan_connected_) {
        ESP_LOGW(LAN_TAG, "lan_connected_ true→false: Link DOWN");
        this->lan_connected_ = false;
      }
      // v3.15.20: 無條件關 socket（即使 CS_CONNECTING 中）
      // 讓背景 task 卡在 ::connect() 的 SYN timeout 被 EBADF 喚醒 → 立即返回 CS_FAILED
      // 否則 task 最多要等 lwIP 預設 TCP connect timeout（~63s）才能收尾
      // 造成 Dashboard 黃燈卡死回不來
      if (this->lan_sock_fd_ >= 0) {
        this->lan_do_disconnect();
      }
      this->publish_sensor("ws_lan_connection", "Link DOWN");
      this->log_to_dashboard("[LAN] Link DOWN — cable disconnected");
      this->lan_reconnect_attempts_ = 0;
      this->lan_reconnect_interval_ = 3000;
    }
    if (!this->lan_last_link_up_ && lan_link_up) {
      // v3.15.20: 放寬條件，Link UP 就無條件重置重試計時
      // 即使 state 卡在 CS_CONNECTING/CS_FAILED 也要重置，
      // state 切換由下方 CS_FAILED handler + CS_CONNECTING watchdog 處理
      this->lan_last_connect_attempt_ = 0;
      this->lan_reconnect_interval_ = 3000;
      this->lan_reconnect_attempts_ = 0;
      if (!this->lan_connected_) {
        this->publish_sensor("ws_lan_connection", "Link UP...");
        this->log_to_dashboard("[LAN] Link UP — will connect in 3s");
      }
    }
    this->lan_last_link_up_ = lan_link_up;

    // v3.15.21: CS_CONNECTING watchdog（無論 link up/down 都檢查）
    // 背景 task 若卡在 ::connect() 的 lwIP TCP SYN timeout（預設 ~63s），
    // 這裡 30s 主動關 socket，讓 task 收到 EBADF → 返回失敗 → 設 CS_FAILED
    // 二重保險：即使 Link DOWN handler 因某種原因沒關到 socket，這裡也會收尾
    // v3.15.21: 由 20s 改為 30s，給慢速網路/DNS 更充裕的正常連線時間
    if (this->lan_connect_state_ == CS_CONNECTING &&
        this->lan_connecting_since_ != 0 &&
        now - this->lan_connecting_since_ > 30000) {
      ESP_LOGW(LAN_TAG, "CS_CONNECTING watchdog: >30s, force close socket");
      this->log_to_dashboard("[LAN] Connect watchdog — force timeout after 30s");
      if (this->lan_sock_fd_ >= 0) {
        this->lan_do_disconnect();  // task 會收到 EBADF 自行返回 CS_FAILED
      }
      // 不直接寫 CS_FAILED（避免與 task 競爭），讓 task 自己設。
      // 若 task 已失敗但卡在 CS_CONNECTING（理論不該發生），下次 loop 仍會命中此 watchdog
      this->lan_connecting_since_ = 0;  // 避免重複觸發，直到下次進入 CS_CONNECTING
    }

    if (lan_link_up) {
      // Background connect task state machine
      if (this->lan_connect_state_ == CS_CONNECTED) {
        this->lan_connect_state_ = CS_IDLE;
        ESP_LOGW(LAN_TAG, "lan_connected_ false→true: CS_CONNECTED");
        this->lan_connected_ = true;
        this->lan_connected_time_ = now;
        this->lan_reconnect_attempts_ = 0;
        this->lan_reconnect_interval_ = 5000;
        if (this->lan_sock_fd_ >= 0) {
          int fl = fcntl(this->lan_sock_fd_, F_GETFL, 0);
          fcntl(this->lan_sock_fd_, F_SETFL, fl | O_NONBLOCK);

          // TCP keepalive：OS 層探測 dead socket，比 ping/pong 更快發現對端消失
          // 折衷值：總偵測時間 = KEEPIDLE + (KEEPINTVL × KEEPCNT) = 30 + (10×3) = 60 秒
          // 對穩定 LAN 提供充足容錯，對真正斷線能在 60s 內偵測，不會因微抖動誤斷
          int ka = 1;
          int ka_idle = 30;
          int ka_intvl = 10;
          int ka_cnt = 3;
          setsockopt(this->lan_sock_fd_, SOL_SOCKET, SO_KEEPALIVE, &ka, sizeof(ka));
          setsockopt(this->lan_sock_fd_, IPPROTO_TCP, TCP_KEEPIDLE, &ka_idle, sizeof(ka_idle));
          setsockopt(this->lan_sock_fd_, IPPROTO_TCP, TCP_KEEPINTVL, &ka_intvl, sizeof(ka_intvl));
          setsockopt(this->lan_sock_fd_, IPPROTO_TCP, TCP_KEEPCNT, &ka_cnt, sizeof(ka_cnt));
        }
        this->lan_last_pong_time_ = now;
        this->lan_last_ping_time_ = now;
        this->publish_sensor("ws_lan_connection", "Connected");
        this->log_to_dashboard("[LAN] Connected to " + this->lan_server_);
      }

      if (this->lan_connect_state_ == CS_FAILED) {
        this->lan_connect_state_ = CS_IDLE;
        // 防禦性：CS_FAILED 必然代表 WS2 未連線，無論前值如何都強制設 false
        // 防止 race condition 下 lan_connected_ 卡在 true 但實際在重連中
        if (this->lan_connected_) {
          ESP_LOGW(LAN_TAG, "lan_connected_ true→false: CS_FAILED defensive reset");
          this->lan_connected_ = false;
          this->lan_do_disconnect();
        }
        this->lan_last_connect_attempt_ = now;
        if (this->lan_reconnect_attempts_ < 10) {
          this->lan_reconnect_attempts_++;
          uint32_t base = 5000 * (1 << this->lan_reconnect_attempts_);
          if (base > 60000) base = 60000;
          this->lan_reconnect_interval_ = base;
        } else {
          this->lan_reconnect_interval_ = 60000;
        }
        char buf[64];
        snprintf(buf, sizeof(buf), "Retry %us (#%u)", this->lan_reconnect_interval_ / 1000, this->lan_reconnect_attempts_);
        this->publish_sensor("ws_lan_connection", buf);
        char log_buf[96];
        snprintf(log_buf, sizeof(log_buf), "[LAN] Connect failed → retry #%u in %us", this->lan_reconnect_attempts_, this->lan_reconnect_interval_ / 1000);
        this->log_to_dashboard(log_buf);
      }

      // Reconnect logic
      if (!this->lan_connected_ && this->lan_connect_state_ == CS_IDLE) {
        if (now - this->lan_last_connect_attempt_ >= this->lan_reconnect_interval_) {
          // Check IP
          auto ips = network::get_ip_addresses();
          if (ips.empty() || !ips[0].is_set()) {
            this->lan_last_connect_attempt_ = now;
            this->lan_reconnect_interval_ = 10000;
          } else {
            this->lan_connect_state_ = CS_CONNECTING;
            // v3.15.20: 記錄進入 CS_CONNECTING 的時間供 watchdog 使用
            this->lan_connecting_since_ = now;
            this->publish_sensor("ws_lan_connection", "Connecting...");
            {
              char cb[80];
              snprintf(cb, sizeof(cb), "[LAN] Connecting to %s:%d...", this->lan_server_.c_str(), this->lan_port_ ? this->lan_port_ : 80);
              this->log_to_dashboard(cb);
            }
            BaseType_t ret = xTaskCreate(lan_connect_task_fn, "lan_ws", 4096, this, 3, &this->lan_connect_task_handle_);
            if (ret != pdPASS) {
              this->lan_connect_state_ = CS_FAILED;
              ESP_LOGE(TAG, "LAN WS task create failed");
            }
          }
        }
      }

      // Connected: read frames + periodic messages
      if (this->lan_connected_) {
        // v4.0.71: breadcrumb — 進 LAN WS read frames 區段
        set_reboot_tag_c("LOOP_LAN_READ");
        this->lan_got_server_data_ = false;
        int read_count = 0;
        while (read_count < 5) {
          int r = this->lan_read_one_frame();
          if (r <= 0) {
            if (r < 0) {
              ESP_LOGW(LAN_TAG, "lan_connected_ true→false: read error (r=%d)", r);
              this->lan_connected_ = false;
              this->lan_do_disconnect();
              this->lan_last_connect_attempt_ = now;
              this->lan_reconnect_interval_ = 5000;
              this->lan_reconnect_attempts_ = 0;
              this->publish_sensor("ws_lan_connection", "Disconnected");
              this->log_to_dashboard("[LAN] Disconnected (read error) — retry in 5s");
            }
            break;
          }
          read_count++;
        }

        if (this->lan_got_server_data_) this->lan_last_pong_time_ = now;

        // 已取消「連線後首次自動 STATUS」。WS2 完全被動：只在收到 GET_STATUS / CMD status 才回 STATUS。

        // WS ping every 10s — 發送 WS 協議層 ping 給 RPi
        // 折衷值：比原版 15s 略積極，但保留網路抖動容錯空間
        // 檢查 send 返回值：若 TCP socket 已損壞（對端 FIN/RST），立即判定斷線
        // 配合 TCP keepalive（60s OS 層偵測）+ 20s pong timeout，三重保險
        if (this->lan_connected_ && (now - this->lan_last_ping_time_ > 10000)) {
          this->lan_last_ping_time_ = now;
          bool ping_ok = this->lan_send_ws_frame(0x09, nullptr, 0);  // WS ping
          if (!ping_ok) {
            ESP_LOGW(LAN_TAG, "lan_connected_ true→false: ping send failed, socket broken");
            this->lan_connected_ = false;
            this->lan_do_disconnect();
            this->lan_last_connect_attempt_ = now;
            this->lan_reconnect_interval_ = 5000;
            this->lan_reconnect_attempts_ = 0;
            this->publish_sensor("ws_lan_connection", "Send failed");
            this->log_to_dashboard("[LAN] Ping send failed — socket broken, retry in 5s");
          } else {
            // v4.0.117+: ping/pong silent 預設，YAML switch ping_log_sw 開啟才寫
            if (show_ping_log_) this->log_to_dashboard("[LAN] TX: WS Ping");
          }
        }

        // Pong timeout 20s（折衷值：比原版 30s 略積極，但容忍 RPi 短暫忙碌）
        // 10s 送 ping，20s 內沒回 pong（相當於漏 2 次 pong）→ 判定斷線
        if (this->lan_connected_ && (now - this->lan_last_pong_time_ > 20000)) {
          ESP_LOGW(LAN_TAG, "lan_connected_ true→false: pong timeout 20s");
          this->lan_connected_ = false;
          this->lan_do_disconnect();
          this->lan_last_connect_attempt_ = now;
          this->lan_reconnect_interval_ = 5000;
          this->lan_reconnect_attempts_ = 0;
          this->publish_sensor("ws_lan_connection", "Pong timeout");
          this->log_to_dashboard("[LAN] Pong timeout 20s — disconnected, retry in 5s");
        }
      }
    }
  }

  // v3.15.25: go_enable / emergency_enable 佈防/retry/BS 追蹤邏輯已全部移除。
  //           按鈕事件現在由 YAML 直接呼叫 id(ws).send_event(...) 發 EVENT，
  //           C++ 不再維護 armed state、timeout、emergency retry、bs_blue_/green_ 指標。

  // Deferred reboot (allows CMD reply to be sent first)
  // v4.0.71: reboot_pending_ 處理已移到 loop() 頂端，此處保留 enabled_ 檢查

  if (!this->enabled_) return;

  // === Ethernet link detection (W5500 hardware-level, instant) ===
  bool link_up = false;
  auto *eth = ethernet::global_eth_component;
  if (eth != nullptr) {
    link_up = eth->is_connected();
  }

  // Link went DOWN — immediately disconnect WebSocket
  if (this->last_link_up_ && !link_up) {
    ESP_LOGW(TAG, "[WS1] Ethernet link DOWN — cable unplugged");
    this->log_to_dashboard("[WS1] Ethernet link DOWN");
    if (this->connected_) {
      this->connected_ = false;
      this->last_disconnect_reason_ = "ethernet link DOWN";
      this->do_disconnect();
    }
    this->publish_sensor("ws_connection", "Link DOWN");
    this->reconnect_attempts_ = 0;
    this->reconnect_interval_ = 3000;  // Try quickly when link restores
  }

  // Link came UP — reconnect quickly
  if (!this->last_link_up_ && link_up) {
    ESP_LOGI(TAG, "Ethernet link UP — cable connected");
    this->log_to_dashboard("Ethernet link UP");
    if (!this->connected_ && this->connect_state_ == CS_IDLE) {
      this->last_connect_attempt_ = 0;  // Trigger immediate connect
      this->reconnect_interval_ = 3000; // Small delay for DHCP/IP
      this->reconnect_attempts_ = 0;
      this->publish_sensor("ws_connection", "Link UP, connecting...");
    }
  }

  this->last_link_up_ = link_up;

  // If link is down, don't try anything
  if (!link_up) {
    if (this->connected_) {
      this->connected_ = false;
      this->do_disconnect();
      this->publish_sensor("ws_connection", "Link DOWN");
    }
    return;
  }

  // === State: background connect task is running ===
  if (this->connect_state_ == CS_CONNECTING) {
    // Just wait — task is running in background, main loop is NOT blocked
    return;
  }

  // === State: background task finished with success ===
  if (this->connect_state_ == CS_CONNECTED) {
    this->connect_state_ = CS_IDLE;
    this->connected_ = true;
    this->connected_time_ = now;
    this->reconnect_attempts_ = 0;
    this->reconnect_interval_ = 5000;

    // Set socket to non-blocking for subsequent reads
    if (this->sock_fd_ >= 0) {
      int flags = fcntl(this->sock_fd_, F_GETFL, 0);
      fcntl(this->sock_fd_, F_SETFL, flags | O_NONBLOCK);
    }

    this->last_pong_time_ = now;  // Reset pong timer on fresh connect
    this->last_ping_time_ = now;  // Reset ping timer (prevent immediate ping)
    this->publish_sensor("ws_connection", "Connected");
    // v3.16.32: 若此次為重連，印出上次斷線原因幫忙對照（第一次開機 reason 為空）
    if (!this->last_disconnect_reason_.empty()) {
      std::string reconnect_log = "[WS1] Connected to " + this->server_ +
                                   " (reconnect after: " + this->last_disconnect_reason_ + ")";
      this->log_to_dashboard(reconnect_log);
      ESP_LOGI(TAG, "[WS1] Connected to %s (reconnect after: %s)",
               this->server_.c_str(), this->last_disconnect_reason_.c_str());
    } else {
      this->log_to_dashboard("[WS1] Connected to " + this->server_);
      ESP_LOGI(TAG, "[WS1] Connected to %s (first)", this->server_.c_str());
    }

    // v3.16.30: 移除 on_connect 主動送 get_debug_pwd EVENT
    //   用戶反饋「WS1 一直斷線（4003 Device blocked）每次重連都重送 get_debug_pwd」
    //   改為：debug_pwd 預設用裝置序號末四碼（於 on_hmi_password_submit lazy init）
    //   Server 仍可主動送 set_debug_pwd CMD 覆蓋（維持工程模式密碼可設定）
  }

  // === State: background task finished with failure ===
  if (this->connect_state_ == CS_FAILED) {
    this->connect_state_ = CS_IDLE;
    this->last_connect_attempt_ = now;

    if (this->reconnect_attempts_ < MAX_FAST_RETRIES) {
      this->reconnect_attempts_++;
      this->reconnect_interval_ = this->get_backoff_with_jitter();
    } else {
      // After MAX_FAST_RETRIES, switch to slow retry (5 min)
      this->reconnect_interval_ = SLOW_RETRY_INTERVAL_MS;
    }

    char buf[80];
    snprintf(buf, sizeof(buf), "Retry in %us (#%u)",
             this->reconnect_interval_ / 1000, this->reconnect_attempts_);
    this->publish_sensor("ws_connection", buf);
    ESP_LOGW(TAG, "Connect failed, retry in %u ms (attempt %u)",
             this->reconnect_interval_, this->reconnect_attempts_);
  }

  // v4.0.71: breadcrumb — 進 WS1 connect attempt 區段
  set_reboot_tag_c("LOOP_WS1_RECONN");
  // === Not connected: check if it's time to start a new connection attempt ===
  if (!this->connected_ && this->connect_state_ == CS_IDLE) {
    if (now - this->last_connect_attempt_ >= this->reconnect_interval_) {
      // Check network availability first (must have an IP address)
      auto ip_addrs = network::get_ip_addresses();
      if (ip_addrs.empty() || !ip_addrs[0].is_set()) {
        // No network — don't even try, retry in 10s
        this->last_connect_attempt_ = now;
        this->reconnect_interval_ = 10000;
        ESP_LOGW(TAG, "No network IP, skip connect attempt");
        this->publish_sensor("ws_connection", "No network, retry 10s");
        return;
      }

      // Spawn background task for connection (does NOT block main loop)
      this->connect_state_ = CS_CONNECTING;
      this->publish_sensor("ws_connection", "Connecting...");
      ESP_LOGI(TAG, "Starting connect task (attempt %u)...", this->reconnect_attempts_ + 1);

      BaseType_t ret = xTaskCreate(
        connect_task_fn,
        "ws_conn",
        8192,          // 8KB stack for DNS + TLS
        this,
        5,             // priority (lower than main loop)
        &this->connect_task_handle_
      );
      if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create connect task!");
        this->connect_state_ = CS_FAILED;
      }
    }
    return;
  }

  // v4.0.71: breadcrumb — 進 WS1 read incoming frames 區段（TLS read / frame parse）
  set_reboot_tag_c("LOOP_WS1_READ");
  // === Connected: read incoming data ===
  this->got_server_data_ = false;  // Reset flag before reading
  int frames_read = 0;
  while (true) {
    int read_result = this->read_one_frame();
    if (read_result < 0) {
      // Fatal error or server closed
      // v3.16.32: 注意：若是 close frame，last_disconnect_reason_ 已在 read_one_frame 內設定
      //           若是 TCP EOF 或讀取錯誤，read_one_frame 內已 log 原因；這裡補一個通用原因
      uint32_t uptime_s = (now - this->connected_time_) / 1000;
      this->connected_ = false;
      if (this->last_disconnect_reason_.empty() ||
          this->last_disconnect_reason_.find("close") == std::string::npos) {
        char rb[96];
        snprintf(rb, sizeof(rb), "read returned -1 (up %us)", uptime_s);
        this->last_disconnect_reason_ = rb;
      }
      this->last_connect_attempt_ = now;
      this->reconnect_interval_ = this->get_backoff_with_jitter();
      if (this->reconnect_attempts_ < MAX_FAST_RETRIES) this->reconnect_attempts_++;
      char disc_msg[120];
      snprintf(disc_msg, sizeof(disc_msg), "[WS1] Disconnected after %us (%s)",
               uptime_s, this->last_disconnect_reason_.c_str());
      this->publish_sensor("ws_connection", disc_msg);
      this->log_to_dashboard(disc_msg);
      ESP_LOGI(TAG, "[WS1] Disconnected after %u s (%s), reconnect in %u ms",
               uptime_s, this->last_disconnect_reason_.c_str(), this->reconnect_interval_);
      this->do_disconnect();
      return;
    }
    if (read_result == 0) break;  // No more data
    frames_read++;
    if (frames_read > 10) break;  // Safety limit
  }

  // Update pong timer using the SAME `now` captured at top of loop()
  // (avoids uint32_t underflow when millis() inside read handler > now)
  if (this->got_server_data_) {
    this->last_pong_time_ = now;
  }

  // 已取消「連線後首次自動 STATUS」。WS1 完全被動：只在收到 GET_STATUS / CMD status 才回 STATUS。

  // Keep-alive ping every 15s (WebSocket-level)
  // 檢查 send 返回值：若 TLS socket 已損壞（對端 FIN/RST 或 TLS 失效），立即判定斷線
  // 否則要等 30s Pong timeout 才會發現，期間 Dashboard 顯示為「Connected」但實際已斷
  if (now - this->last_ping_time_ > 15000) {
    this->last_ping_time_ = now;
    bool ping_ok = this->send_ping();
    if (!ping_ok) {
      uint32_t uptime_s = (now - this->connected_time_) / 1000;
      ESP_LOGW(TAG, "[WS1] ping send failed, socket broken (up %us) → disconnecting", uptime_s);
      this->publish_sensor("ws_connection", "Send failed");
      char rb[80];
      snprintf(rb, sizeof(rb), "ping send fail (up %us)", uptime_s);
      this->last_disconnect_reason_ = rb;
      this->log_to_dashboard(std::string("[WS1] Ping send failed — ") + rb);
      this->connected_ = false;
      this->last_connect_attempt_ = now;
      // v3.16.32: 不再強制 reset reconnect_attempts_=0 + interval=5s
      //           改用 backoff，避免 server 持續 kick 導致 5s 重連風暴
      this->reconnect_interval_ = this->get_backoff_with_jitter();
      if (this->reconnect_attempts_ < MAX_FAST_RETRIES) this->reconnect_attempts_++;
      this->do_disconnect();
      return;
    }
    if (show_ping_log_) this->log_to_dashboard("[WS1] TX: WS Ping");  // v4.0.117+
  }

  // Pong timeout: if no pong received for 30s, connection is dead
  if (now - this->last_pong_time_ > 30000) {
    uint32_t uptime_s = (now - this->connected_time_) / 1000;
    ESP_LOGW(TAG, "[WS1] No pong for 30s, connection dead (was up %us)", uptime_s);
    char rb[80];
    snprintf(rb, sizeof(rb), "pong timeout 30s (up %us)", uptime_s);
    this->last_disconnect_reason_ = rb;
    char disc_msg[120];
    snprintf(disc_msg, sizeof(disc_msg), "[WS1] No pong 30s (up %us), reconnecting", uptime_s);
    this->publish_sensor("ws_connection", disc_msg);
    this->log_to_dashboard(disc_msg);
    this->connected_ = false;
    this->last_connect_attempt_ = now;
    // v3.16.32: 不再強制 reset reconnect_attempts_=0 + interval=5s
    //           改用 backoff，避免 server 持續 kick 導致 5s 重連風暴
    this->reconnect_interval_ = this->get_backoff_with_jitter();
    if (this->reconnect_attempts_ < MAX_FAST_RETRIES) this->reconnect_attempts_++;
    this->do_disconnect();
    return;
  }
}

uint32_t WSClientComponent::get_backoff_with_jitter() {
  uint32_t base = 5000 * (1 << this->reconnect_attempts_);
  if (base > 60000) base = 60000;
  uint32_t jitter_range = base * 4 / 10;
  uint32_t jitter = (esp_random() % jitter_range);
  return base - (jitter_range / 2) + jitter;
}

// =========================================================================
// TLS I/O helpers (abstract SSL vs plain TCP)
// =========================================================================

ssize_t WSClientComponent::tls_write(const void *buf, size_t len) {
  if (this->ssl_active_) {
    int ret;
    // === v4.0.230 [WANT-WRITE-FIX] BEGIN ===
    // Bug: internet 斷但 LAN 通時 (TCP 半開、send buffer 累滿)，::send 回 EAGAIN
    //      → mbedtls 回 SSL_WANT_WRITE → 此處原本 do-while 無限 busy-loop
    //      → main loop 卡死、web_server SSE 不回、LED script 跑不到 → 觀察到的「LED 卡」
    // 場景：v4.0.212 [WDT-FIX] 只補 SNDTIMEO（blocking 才有用），但 sock_fd_ 已切 non-blocking
    //       SNDTIMEO 失效，busy-loop 沒救
    // 修法：max 100 retries x vTaskDelay(10ms) = 1000ms 上限（< Task WDT 5s 安全）
    //       超時 return -1 → send_text 上層處理 connected_=false 觸發 reconnect
    //       vTaskDelay 讓出 CPU 給 IDLE task 餵 Task WDT + 給其他 task 跑
    // Rollback: 移除整個 BEGIN/END 區塊改回原 do-while
    const int MAX_WANT_WRITE_RETRIES = 100;
    int retries = 0;
    do {
      ret = mbedtls_ssl_write(&this->ssl_, (const unsigned char *)buf, len);
      if (ret == MBEDTLS_ERR_SSL_WANT_WRITE) {
        if (++retries >= MAX_WANT_WRITE_RETRIES) {
          ESP_LOGE(TAG, "[WANT-WRITE-FIX] tls_write SSL_WANT_WRITE x%d (1000ms timeout) — net stuck, abort",
                   retries);
          return -1;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
      }
    } while (ret == MBEDTLS_ERR_SSL_WANT_WRITE);
    // === v4.0.230 [WANT-WRITE-FIX] END ===
    if (ret < 0) {
      char err_buf[100];
      mbedtls_strerror(ret, err_buf, sizeof(err_buf));
      ESP_LOGE(TAG, "ssl_write error: -0x%04x %s", -ret, err_buf);
      return -1;
    }
    return ret;
  } else {
    return ::send(this->sock_fd_, buf, len, 0);
  }
}

ssize_t WSClientComponent::tls_read(void *buf, size_t len) {
  // First, consume any leftover data from WS handshake
  if (this->handshake_leftover_len_ > 0) {
    size_t copy = len < (size_t)this->handshake_leftover_len_ ? len : (size_t)this->handshake_leftover_len_;
    memcpy(buf, this->handshake_leftover_, copy);
    // Shift remaining leftover
    this->handshake_leftover_len_ -= copy;
    if (this->handshake_leftover_len_ > 0) {
      memmove(this->handshake_leftover_, this->handshake_leftover_ + copy, this->handshake_leftover_len_);
    }
    return (ssize_t)copy;
  }

  if (this->ssl_active_) {
    int ret = mbedtls_ssl_read(&this->ssl_, (unsigned char *)buf, len);
    if (ret == MBEDTLS_ERR_SSL_WANT_READ || ret == MBEDTLS_ERR_SSL_WANT_WRITE) {
      errno = EAGAIN;
      return -1;
    }
    if (ret == MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY || ret == 0) {
      ESP_LOGW(TAG, "tls_read: peer close notify or EOF");
      return 0;  // Connection closed
    }
    if (ret < 0) {
      char err_buf[100];
      mbedtls_strerror(ret, err_buf, sizeof(err_buf));
      ESP_LOGE(TAG, "tls_read error: -0x%04x %s", -ret, err_buf);
      errno = EIO;
      return -1;
    }
    return ret;
  } else {
    return ::recv(this->sock_fd_, buf, len, 0);
  }
}

// =========================================================================
// Connection: raw socket + mbedtls TLS (runs in FreeRTOS background task)
// =========================================================================

bool WSClientComponent::do_connect() {
  int port = this->port_;
  if (port == 0) {
    port = this->use_ssl_ ? 443 : 80;
  }

  uint32_t t0 = millis();

  // v4.0.273 [DNS-CACHE-FIX]: 解 getaddrinfo() lwIP 同步 API 卡 main loop 4-7s 問題
  //   - v4.0.270 只 wrap ::connect()，沒覆蓋 getaddrinfo()
  //   - 用 cached IP 跳過 DNS；連續 10 次 TCP fail 才強制 re-resolve
  //   - 連到 wss.huiflow.com (Google CDN) 環境不穩時 main loop 卡頓會大幅改善
  static char cached_ip_[16] = "";       // 上次解析成功 IP，空字串 = 無 cache
  static int tcp_fail_count_ = 0;         // 連續 TCP fail 計數
  static std::string cached_host_;        // cached IP 是哪個 host 解析來的（防 set_server() 換 host）

  // Step 1: DNS resolve (skip if cached IP valid)
  // v4.0.273 [BREADCRUMB-WS1-DETAIL]: 細化 LOOP_WS1_* breadcrumb 利於 crash 定位
  ::set_reboot_tag_c("LOOP_WS1_RESOLV");
  struct sockaddr_in dest;
  memset(&dest, 0, sizeof(dest));
  dest.sin_family = AF_INET;
  dest.sin_port = htons(port);
  char ip_str[16];

  bool cache_valid = (cached_ip_[0] != '\0' && cached_host_ == this->server_ && tcp_fail_count_ < 10);
  if (cache_valid) {
    // 跳過 getaddrinfo 直接用 cached IP
    strncpy(ip_str, cached_ip_, sizeof(ip_str));
    ip_str[sizeof(ip_str) - 1] = '\0';
    if (inet_aton(ip_str, &dest.sin_addr) == 0) {
      // cached IP 解析失敗（不該發生），清掉 cache 走重新 resolve
      cached_ip_[0] = '\0';
      cache_valid = false;
    } else {
      ESP_LOGI(TAG, "[1/4] DNS CACHED: %s -> %s (fails=%d/10)",
               this->server_.c_str(), ip_str, tcp_fail_count_);
      char log_buf[64];
      snprintf(log_buf, sizeof(log_buf), "DNS cached: %s (fail %d/10)", ip_str, tcp_fail_count_);
      this->log_to_dashboard(log_buf);
    }
  }

  if (!cache_valid) {
    ESP_LOGI(TAG, "[1/4] DNS resolving %s...", this->server_.c_str());
    struct addrinfo hints = {};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    struct addrinfo *res = nullptr;
    int dns_err = getaddrinfo(this->server_.c_str(), nullptr, &hints, &res);
    if (dns_err != 0 || !res) {
      ESP_LOGE(TAG, "DNS failed: %d", dns_err);
      this->log_to_dashboard("DNS failed for " + this->server_);
      if (res) freeaddrinfo(res);
      return false;
    }
    struct sockaddr_in *addr = (struct sockaddr_in *)res->ai_addr;
    inet_ntoa_r(addr->sin_addr, ip_str, sizeof(ip_str));
    dest.sin_addr = addr->sin_addr;
    freeaddrinfo(res);
    ESP_LOGI(TAG, "[1/4] DNS OK: %s -> %s (%ums)", this->server_.c_str(), ip_str, millis() - t0);
    this->log_to_dashboard(std::string("DNS: ") + this->server_ + " -> " + ip_str);
    // 寫入 cache
    strncpy(cached_ip_, ip_str, sizeof(cached_ip_));
    cached_ip_[sizeof(cached_ip_) - 1] = '\0';
    cached_host_ = this->server_;
    tcp_fail_count_ = 0;
  }

  // Step 2: TCP connect using raw socket
  ESP_LOGI(TAG, "[2/4] TCP connecting to %s:%d...", ip_str, port);
  ::set_reboot_tag_c("LOOP_WS1_TCPCONN");

  this->sock_fd_ = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (this->sock_fd_ < 0) {
    ESP_LOGE(TAG, "[2/4] socket() failed: errno=%d", errno);
    this->log_to_dashboard("TCP socket create FAIL");
    return false;
  }

  // v4.0.270 [CONNECT-TIMEOUT-FIX]: Non-blocking connect + 500ms select timeout
  //   Root cause: ::connect() default blocking ~75s (lwIP TCP SYN retransmit) when remote unreachable
  //   SO_SNDTIMEO/SO_RCVTIMEO 不管 connect (只管 read/write)
  //   修法: 切 non-blocking → connect 返 EINPROGRESS → select() 500ms timeout
  //   遠端不通的話 main loop 卡 75s → 500ms，恢復速度 150x
  int orig_flags = fcntl(this->sock_fd_, F_GETFL, 0);
  fcntl(this->sock_fd_, F_SETFL, orig_flags | O_NONBLOCK);

  uint32_t tcp_t0 = millis();
  int conn_ret = ::connect(this->sock_fd_, (struct sockaddr *)&dest, sizeof(dest));
  // Non-blocking connect 預期回 -1 + errno=EINPROGRESS (115)
  if (conn_ret != 0 && errno != EINPROGRESS) {
    uint32_t tcp_dt = millis() - tcp_t0;
    ESP_LOGE(TAG, "[2/4] TCP connect immediate fail in %ums: errno=%d", tcp_dt, errno);
    char log_buf[128];
    snprintf(log_buf, sizeof(log_buf), "TCP FAIL %ums: errno=%d (immediate)", tcp_dt, errno);
    this->log_to_dashboard(log_buf);
    close(this->sock_fd_);
    this->sock_fd_ = -1;
    tcp_fail_count_++;  // v4.0.273 [DNS-CACHE-FIX]
    return false;
  }

  // 等 writable (= connect 完成 or error)，500ms timeout
  fd_set wfds;
  FD_ZERO(&wfds);
  FD_SET(this->sock_fd_, &wfds);
  struct timeval tv_conn = { .tv_sec = 0, .tv_usec = 500000 };  // 500ms
  int sel_ret = select(this->sock_fd_ + 1, NULL, &wfds, NULL, &tv_conn);
  uint32_t tcp_dt = millis() - tcp_t0;

  if (sel_ret <= 0) {
    ESP_LOGE(TAG, "[2/4] TCP connect TIMEOUT in %ums (select=%d errno=%d)", tcp_dt, sel_ret, errno);
    char log_buf[128];
    snprintf(log_buf, sizeof(log_buf), "TCP TIMEOUT %ums (500ms cap)", tcp_dt);
    this->log_to_dashboard(log_buf);
    close(this->sock_fd_);
    this->sock_fd_ = -1;
    tcp_fail_count_++;  // v4.0.273 [DNS-CACHE-FIX]
    return false;
  }

  // 確認 connect 真的成功（不是 ECONNREFUSED / EHOSTUNREACH 等）
  int so_err = 0;
  socklen_t so_len = sizeof(so_err);
  getsockopt(this->sock_fd_, SOL_SOCKET, SO_ERROR, &so_err, &so_len);
  if (so_err != 0) {
    ESP_LOGE(TAG, "[2/4] TCP connect SO_ERROR=%d in %ums", so_err, tcp_dt);
    char log_buf[128];
    snprintf(log_buf, sizeof(log_buf), "TCP FAIL %ums: SO_ERROR=%d", tcp_dt, so_err);
    this->log_to_dashboard(log_buf);
    close(this->sock_fd_);
    this->sock_fd_ = -1;
    tcp_fail_count_++;  // v4.0.273 [DNS-CACHE-FIX]
    return false;
  }

  // 還原 blocking mode (post-connect SNDTIMEO/RCVTIMEO 才能正常作用)
  fcntl(this->sock_fd_, F_SETFL, orig_flags);

  // 設 15s read/write timeout (v4.0.212 [WDT-FIX] 後面會改成 3s)
  struct timeval tv;
  tv.tv_sec = 15;
  tv.tv_usec = 0;
  setsockopt(this->sock_fd_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
  setsockopt(this->sock_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  ESP_LOGI(TAG, "[2/4] TCP OK in %ums to %s:%d", tcp_dt, ip_str, port);
  tcp_fail_count_ = 0;  // v4.0.273 [DNS-CACHE-FIX] TCP 成功 reset fail counter
  {
    char log_buf[80];
    snprintf(log_buf, sizeof(log_buf), "TCP OK %ums to %s:%d", tcp_dt, ip_str, port);
    this->log_to_dashboard(log_buf);
  }

  // === v4.0.212 [WDT-FIX] BEGIN ===
  // 設計原則：主板↔洗車機=485 (UART)，網路斷不可重啟主板
  // Bug: TCP connect 用 SNDTIMEO=15s，但 ESP32 Task WDT=5s
  //      拔網線後 main loop send_text 等 15s socket timeout → WDT 5s 觸發 reboot
  // 修法: TCP connect OK 後重設 SNDTIMEO=3s（< WDT 5s 安全 buffer）
  //      send_text 失敗最多卡 3 秒，main loop 讓 WDT feed 機會
  // Rollback: 刪除整個 BEGIN/END 區塊
  {
    struct timeval tv_send;
    tv_send.tv_sec = 3;
    tv_send.tv_usec = 0;
    setsockopt(this->sock_fd_, SOL_SOCKET, SO_SNDTIMEO, &tv_send, sizeof(tv_send));
    ESP_LOGI(TAG, "[WDT-FIX] WS1 SNDTIMEO 15s → 3s (post-connect, prevent main-loop block > WDT 5s)");
  }
  // === v4.0.212 [WDT-FIX] END ===

  // Step 3: TLS handshake (if SSL enabled)
  if (this->use_ssl_) {
    ::set_reboot_tag_c("LOOP_WS1_TLS");  // v4.0.273 [BREADCRUMB-WS1-DETAIL]
    ESP_LOGI(TAG, "[3/4] TLS handshake...");
    uint32_t tls_t0 = millis();

    // Initialize mbedtls
    mbedtls_ssl_init(&this->ssl_);
    mbedtls_ssl_config_init(&this->ssl_conf_);
    mbedtls_entropy_init(&this->entropy_);
    mbedtls_ctr_drbg_init(&this->ctr_drbg_);
    this->mbedtls_initialized_ = true;

    int ret = mbedtls_ctr_drbg_seed(&this->ctr_drbg_, mbedtls_entropy_func,
                                     &this->entropy_, nullptr, 0);
    if (ret != 0) {
      char err_buf[100];
      mbedtls_strerror(ret, err_buf, sizeof(err_buf));
      ESP_LOGE(TAG, "ctr_drbg_seed failed: -0x%04x %s", -ret, err_buf);
      this->log_to_dashboard("TLS init failed: ctr_drbg_seed");
      this->do_disconnect();
      return false;
    }

    ret = mbedtls_ssl_config_defaults(&this->ssl_conf_,
                                       MBEDTLS_SSL_IS_CLIENT,
                                       MBEDTLS_SSL_TRANSPORT_STREAM,
                                       MBEDTLS_SSL_PRESET_DEFAULT);
    if (ret != 0) {
      char err_buf[100];
      mbedtls_strerror(ret, err_buf, sizeof(err_buf));
      ESP_LOGE(TAG, "ssl_config_defaults failed: -0x%04x %s", -ret, err_buf);
      this->log_to_dashboard("TLS init failed: config_defaults");
      this->do_disconnect();
      return false;
    }

    // Skip certificate verification (same as VERIFY_NONE before)
    mbedtls_ssl_conf_authmode(&this->ssl_conf_, MBEDTLS_SSL_VERIFY_NONE);
    mbedtls_ssl_conf_rng(&this->ssl_conf_, mbedtls_ctr_drbg_random, &this->ctr_drbg_);

    // ALPN - required by Google Cloud Load Balancer
    static const char *alpn_protocols[] = {"http/1.1", nullptr};
    ret = mbedtls_ssl_conf_alpn_protocols(&this->ssl_conf_, alpn_protocols);
    if (ret != 0) {
      ESP_LOGW(TAG, "ALPN setup failed: -0x%04x (continuing without ALPN)", -ret);
    } else {
      ESP_LOGI(TAG, "ALPN set: http/1.1");
    }

    // Force TLS 1.2 minimum (Google Cloud requirement)
    mbedtls_ssl_conf_min_tls_version(&this->ssl_conf_, MBEDTLS_SSL_VERSION_TLS1_2);

    ret = mbedtls_ssl_setup(&this->ssl_, &this->ssl_conf_);
    if (ret != 0) {
      char err_buf[100];
      mbedtls_strerror(ret, err_buf, sizeof(err_buf));
      ESP_LOGE(TAG, "ssl_setup failed: -0x%04x %s", -ret, err_buf);
      this->log_to_dashboard("TLS init failed: ssl_setup");
      this->do_disconnect();
      return false;
    }

    // Set hostname for SNI (Server Name Indication)
    mbedtls_ssl_set_hostname(&this->ssl_, this->server_.c_str());

    // Set I/O callbacks using our raw socket
    mbedtls_ssl_set_bio(&this->ssl_, &this->sock_fd_,
                         mbedtls_net_send_cb, mbedtls_net_recv_cb, nullptr);

    // Perform TLS handshake
    ESP_LOGI(TAG, "Starting TLS handshake to %s...", this->server_.c_str());
    while ((ret = mbedtls_ssl_handshake(&this->ssl_)) != 0) {
      if (ret != MBEDTLS_ERR_SSL_WANT_READ && ret != MBEDTLS_ERR_SSL_WANT_WRITE) {
        char err_buf[100];
        mbedtls_strerror(ret, err_buf, sizeof(err_buf));
        uint32_t tls_dt = millis() - tls_t0;
        ESP_LOGE(TAG, "[3/4] TLS handshake FAILED in %ums: -0x%04x %s", tls_dt, -ret, err_buf);
        char log_buf[160];
        snprintf(log_buf, sizeof(log_buf), "TLS FAIL %ums: -0x%04x %s", tls_dt, -ret, err_buf);
        this->log_to_dashboard(log_buf);
        this->do_disconnect();
        return false;
      }
      // Check timeout (30s)
      if (millis() - tls_t0 > 30000) {
        ESP_LOGE(TAG, "[3/4] TLS handshake timeout (30s)");
        this->log_to_dashboard("TLS FAIL: timeout 30s");
        this->do_disconnect();
        return false;
      }
    }

    uint32_t tls_dt = millis() - tls_t0;
    this->ssl_active_ = true;
    ESP_LOGI(TAG, "[3/4] TLS handshake OK in %ums (cipher: %s)",
             tls_dt, mbedtls_ssl_get_ciphersuite(&this->ssl_));
    char tls_log[120];
    snprintf(tls_log, sizeof(tls_log), "TLS OK %ums (%s)",
             tls_dt, mbedtls_ssl_get_ciphersuite(&this->ssl_));
    this->log_to_dashboard(tls_log);
  } else {
    ESP_LOGI(TAG, "[3/4] Plain TCP (no SSL)");
  }

  // Step 4: WebSocket handshake
  ESP_LOGI(TAG, "[4/4] WebSocket handshake...");
  if (!this->ws_handshake()) {
    ESP_LOGE(TAG, "[4/4] WebSocket handshake failed");
    this->log_to_dashboard("WS handshake FAILED");
    this->do_disconnect();
    return false;
  }

  // NOTE: socket is set to non-blocking AFTER this function returns,
  // in the main loop's CS_CONNECTED handler (must be done from main task)

  uint32_t total = millis() - t0;
  ESP_LOGI(TAG, "WebSocket connected in %ums total", total);
  char log_buf[64];
  snprintf(log_buf, sizeof(log_buf), "WS connected OK (%ums total)", total);
  this->log_to_dashboard(log_buf);
  return true;
}

bool WSClientComponent::ws_handshake() {
  // Generate random 16-byte key, base64 encode it
  uint8_t raw_key[16];
  for (int i = 0; i < 16; i++) raw_key[i] = (esp_random() & 0xFF);

  unsigned char b64_key[32];
  size_t b64_len = 0;
  mbedtls_base64_encode(b64_key, sizeof(b64_key), &b64_len, raw_key, 16);
  b64_key[b64_len] = '\0';

  // Build HTTP upgrade request
  std::string path = "/ws?device_id=" + this->device_id_;
  std::string request =
    "GET " + path + " HTTP/1.1\r\n"
    "Host: " + this->server_ + "\r\n"
    "Upgrade: websocket\r\n"
    "Connection: Upgrade\r\n"
    "Sec-WebSocket-Key: " + std::string((char *)b64_key) + "\r\n"
    "Sec-WebSocket-Version: 13\r\n"
    "x-api-key: " + this->api_key_ + "\r\n"
    "Origin: https://" + this->server_ + "\r\n"
    "\r\n";

  ESP_LOGD(TAG, "WS request:\n%s", request.c_str());

  // Send request
  size_t written = 0;
  while (written < request.length()) {
    ssize_t ret = this->tls_write(request.c_str() + written,
                                   request.length() - written);
    if (ret < 0) {
      ESP_LOGE(TAG, "WS handshake write failed: %d", (int)ret);
      return false;
    }
    written += ret;
  }

  // Read response (wait up to 10s)
  char resp[1024];
  int resp_len = 0;
  uint32_t start = millis();
  while (millis() - start < 10000 && resp_len < (int)sizeof(resp) - 1) {
    ssize_t ret = this->tls_read(resp + resp_len, sizeof(resp) - 1 - resp_len);
    if (ret > 0) {
      resp_len += ret;
      resp[resp_len] = '\0';
      // Check if we have complete headers (ends with \r\n\r\n)
      if (strstr(resp, "\r\n\r\n")) break;
    } else if (ret == 0) {
      // Connection closed
      ESP_LOGE(TAG, "WS handshake: connection closed");
      return false;
    } else {
      // EAGAIN or error
      int err = errno;
      if (err == EAGAIN || err == EWOULDBLOCK) {
        delay(10);
        continue;
      }
      ESP_LOGE(TAG, "WS handshake read error: %d errno=%d", (int)ret, err);
      return false;
    }
  }

  resp[resp_len] = '\0';
  ESP_LOGD(TAG, "WS response (%d bytes): %s", resp_len, resp);

  // Parse HTTP status line
  if (strncmp(resp, "HTTP/1.1 101", 12) != 0) {
    // Extract status code for logging
    char status_line[128] = {0};
    const char *eol = strstr(resp, "\r\n");
    if (eol) {
      size_t len = eol - resp;
      if (len >= sizeof(status_line)) len = sizeof(status_line) - 1;
      memcpy(status_line, resp, len);
      status_line[len] = '\0';
    }
    ESP_LOGE(TAG, "WS upgrade failed: %s", status_line);
    this->log_to_dashboard(std::string("WS upgrade: ") + status_line);
    return false;
  }

  // Save any leftover data after \r\n\r\n (could be the first WebSocket frame)
  this->handshake_leftover_len_ = 0;
  const char *header_end = strstr(resp, "\r\n\r\n");
  if (header_end) {
    const char *leftover_start = header_end + 4;
    int leftover_len = resp_len - (leftover_start - resp);
    if (leftover_len > 0) {
      if (leftover_len > (int)sizeof(this->handshake_leftover_))
        leftover_len = sizeof(this->handshake_leftover_);
      memcpy(this->handshake_leftover_, leftover_start, leftover_len);
      this->handshake_leftover_len_ = leftover_len;
      ESP_LOGI(TAG, "WS handshake: %d leftover bytes after headers", leftover_len);
      this->log_to_dashboard("WS handshake: " + std::to_string(leftover_len) + " leftover bytes");
    }
  }

  ESP_LOGI(TAG, "WS upgrade 101 OK");
  return true;
}

void WSClientComponent::do_disconnect() {
  if (this->ssl_active_) {
    mbedtls_ssl_close_notify(&this->ssl_);
    this->ssl_active_ = false;
  }
  if (this->mbedtls_initialized_) {
    mbedtls_ssl_free(&this->ssl_);
    mbedtls_ssl_config_free(&this->ssl_conf_);
    mbedtls_ctr_drbg_free(&this->ctr_drbg_);
    mbedtls_entropy_free(&this->entropy_);
    this->mbedtls_initialized_ = false;
  }
  if (this->sock_fd_ >= 0) {
    close(this->sock_fd_);
    this->sock_fd_ = -1;
  }
}

// =========================================================================
// WebSocket Frame Send (RFC 6455 - client must mask frames)
// =========================================================================

bool WSClientComponent::send_ws_frame(uint8_t opcode, const uint8_t *data, size_t len) {
  if (this->sock_fd_ < 0) return false;

  // Use member frame_buf_ to avoid stack overflow (2KB+ on stack = crash)
  // frame_buf_ is 2048 bytes; max usable payload = 2048 - 14 (header) = 2034
  uint8_t *frame_out = this->frame_buf_;
  size_t frame_len = 0;

  // FIN + opcode
  frame_out[0] = 0x80 | (opcode & 0x0F);
  frame_len = 1;

  // Payload length + mask bit (client frames must be masked)
  if (len < 126) {
    frame_out[1] = 0x80 | (uint8_t)len;
    frame_len = 2;
  } else if (len < 65536) {
    frame_out[1] = 0x80 | 126;
    frame_out[2] = (len >> 8) & 0xFF;
    frame_out[3] = len & 0xFF;
    frame_len = 4;
  } else {
    frame_out[1] = 0x80 | 127;
    memset(&frame_out[2], 0, 4);
    frame_out[6] = (len >> 24) & 0xFF;
    frame_out[7] = (len >> 16) & 0xFF;
    frame_out[8] = (len >> 8) & 0xFF;
    frame_out[9] = len & 0xFF;
    frame_len = 10;
  }

  // Generate 4-byte mask key
  uint8_t mask[4];
  for (int i = 0; i < 4; i++) mask[i] = (esp_random() & 0xFF);
  memcpy(frame_out + frame_len, mask, 4);
  frame_len += 4;

  // Apply mask and append payload
  if (len > 0 && data) {
    size_t payload_space = sizeof(this->frame_buf_) - frame_len;
    size_t copy_len = len < payload_space ? len : payload_space;
    for (size_t i = 0; i < copy_len; i++) {
      frame_out[frame_len + i] = data[i] ^ mask[i % 4];
    }
    frame_len += copy_len;
  }

  // Send entire frame as single tls_write (single TLS record)
  size_t written = 0;
  while (written < frame_len) {
    ssize_t ret = this->tls_write(frame_out + written, frame_len - written);
    if (ret <= 0) {
      ESP_LOGE(TAG, "Frame write failed at %u/%u: %d", (unsigned)written, (unsigned)frame_len, (int)ret);
      return false;
    }
    written += ret;
  }

  return true;
}

bool WSClientComponent::send_text(const char *text, int len) {
  if (!this->connected_ || this->sock_fd_ < 0) return false;
  if (len < 0) len = strlen(text);
  ESP_LOGI(TAG, "TX(%d): %.*s", len, len > 200 ? 200 : len, text);
  bool ok = this->send_ws_frame(0x01, (const uint8_t *)text, len);
  if (!ok) {
    ESP_LOGE(TAG, "send_text FAILED");
    this->log_to_dashboard("TX FAIL: frame write error");
    // === v4.0.212 [WDT-FIX] / v4.0.230 [WANT-WRITE-FIX] BEGIN ===
    // send 失敗（SNDTIMEO 3s 或 SSL_WANT_WRITE 1000ms timeout）→ 標記 disconnected 觸發 reconnect
    // 否則下次 send_text 又會卡（每次 STATUS / EVENT 累積 main loop 阻塞）
    this->connected_ = false;
    this->last_disconnect_reason_ = "TX FAIL (SNDTIMEO/WANT_WRITE timeout)";
    // === v4.0.212 [WDT-FIX] / v4.0.230 [WANT-WRITE-FIX] END ===
  } else {
    // Publish every outgoing message to dashboard (除 STATUS 外，STATUS 有獨立 ws_last_status channel)
    // 避免 STATUS 與 CMD_RESULT 連續送時，後者透過 SSE 蓋掉前者的 ws_last_sent 更新
    bool is_status = (len >= 17 && strncmp(text, "{\"type\":\"STATUS\"", 16) == 0);
    if (!is_status) {
      this->publish_sensor("ws_last_sent", std::string(text, len));
    }
  }
  return ok;
}

bool WSClientComponent::send_data(const std::string &data) {
  return this->send_text(data.c_str(), data.length());
}

// send_both: 廣播同一則訊息到 WS1（雲端）+ WS2（LAN）兩個 socket
// 用於 YAML EMG lambda（AC/緊急按鈕/車高/門磁），確保兩個面板都看得到
// - WS1: send_text 已自動 publish 到 ws_last_sent（Dashboard WS1 面板）
// - WS2: lan_send_text 不會 publish，所以額外呼叫 log_to_dashboard("[LAN] TX: ...")
//        以便 JS 透過 ws_log [LAN] 前綴路由到 WS2 面板
void WSClientComponent::send_both(const std::string &data) {
  if (this->connected_) {
    this->send_text(data.c_str(), data.length());
  }
  // WS2 使用嚴格版檢查，避免在重連中或狀態過渡期誤發
  if (this->is_lan_connected()) {
    this->lan_send_text(data.c_str(), data.length());
    this->log_to_dashboard(std::string("[LAN] TX: ") + data);
  }
}

// =========================================================================
// v3.15.25: 統一 EVENT 系統
// =========================================================================
// throttle_check: per-event 節流共用 helper（window_ms 預設 1500，IO event 傳 5000）
//   回傳 true → 應發送（已更新 slot 時間戳）
//   回傳 false → 節流中，丟棄本次
// v4.0.276: window_ms 參數化 — washer_status 沿用 1500，IO 感測器 event 改 5000
bool WSClientComponent::throttle_check(uint32_t h, uint32_t window_ms) {
  uint32_t now = millis();
  for (int i = 0; i < this->emg_throttle_count_; i++) {
    if (this->emg_throttle_[i].hash == h) {
      if ((now - this->emg_throttle_[i].last_ms) < window_ms) return false;
      this->emg_throttle_[i].last_ms = now;
      return true;
    }
  }
  // 新 event name → 新增 slot
  if (this->emg_throttle_count_ < EMG_THROTTLE_SLOTS) {
    this->emg_throttle_[this->emg_throttle_count_++] = {h, now};
  } else {
    int oldest = 0;
    for (int i = 1; i < EMG_THROTTLE_SLOTS; i++) {
      if ((int32_t)(this->emg_throttle_[i].last_ms - this->emg_throttle_[oldest].last_ms) < 0) oldest = i;
    }
    this->emg_throttle_[oldest] = {h, now};
  }
  return true;
}

// send_event_raw: 底層雙發 helper（跳過節流）
// WS1: send_text 會自動 publish ws_last_sent → Dashboard WS1 面板
// WS2: lan_send_text 不會 publish，需額外 log_to_dashboard("[LAN] TX:...") 路由到 WS2 面板
void WSClientComponent::send_event_raw(const char *json, size_t len) {
  if (this->connected_) {
    this->send_text(json, (int)len);
  }
  if (this->is_lan_connected()) {
    this->lan_send_text(json, (int)len);
    this->log_to_dashboard(std::string("[LAN] TX: ") + json);
  }
  // === v4.0.278 [DEVICE-OUT-BUS] BEGIN ===
  // 自動把每個 EVENT 推上 device_event_bus 給 Tier 2 裝置接收
  //   裝置 GET /text_sensor/device_event_bus 拉，用 seq 號 dedup
  //   不論 WS1/WS2 連線狀態，bus 總是更新（讓裝置在主板網路全斷時也能拿到本地事件）
  this->publish_device_bus(std::string(json, len));
  // === v4.0.278 [DEVICE-OUT-BUS] END ===
}

// v4.0.22: HTTP API 追蹤事件
//   外層：{"type":"API","cmd":"<cmd>"[,<params>]}
//   params_json 帶入時是欄位列表（不含外層 {}），例如 `"name":"01.vsn","source":"lan"`
//   雙發 WS1+WS2，跳過節流。Colorlight 4 個 LED script 在呼叫 put() 前呼叫本函式。
void WSClientComponent::send_api_event(const char *cmd, const std::string &params_json) {
  std::string msg;
  msg.reserve(48 + params_json.size());
  msg.append("{\"type\":\"API\",\"cmd\":\"");
  msg.append(cmd);
  msg.append("\"");
  if (!params_json.empty()) {
    msg.append(",");
    msg.append(params_json);
  }
  msg.append("}");
  this->send_event_raw(msg.c_str(), msg.size());
}

// === v4.0.217 [WASH-EVENTS] BEGIN ===
// 簡單版 wash CMD EVENT helper — 只帶 source 欄位
//   格式：{"type":"EVENT","event":"<name>","source":"<src>"[,"order_sn":"..."]}
//   用於 wash_reset / wash_start / wash_stop / test_flow1 / test_flow2
// wash_init / wash_goto 帶額外欄位，inline 在各 fire 點處理
// v4.0.243: order_sn_ 非空時自動帶 order_sn 欄位（流程進行中 EVENT 都關聯訂單）
void WSClientComponent::send_wash_event_simple(const char *event_name, const char *source) {
  char buf[160];
  int n;
  if (!this->order_sn_.empty()) {
    n = snprintf(buf, sizeof(buf),
      "{\"type\":\"EVENT\",\"event\":\"%s\",\"source\":\"%s\",\"order_sn\":\"%s\"}",
      event_name, source ? source : "?", this->order_sn_.c_str());
  } else {
    n = snprintf(buf, sizeof(buf),
      "{\"type\":\"EVENT\",\"event\":\"%s\",\"source\":\"%s\"}",
      event_name, source ? source : "?");
  }
  if (n > 0 && (size_t)n < sizeof(buf)) {
    this->send_event_raw(buf, n);
  }
}

// v4.0.224: wash_complete EVENT — DONE 時送一次，帶完整流程結算
//   {type, event:'wash_complete', order_sn, total, total_sec, complete_auto_sec, durations:[d0,d1,...]}
//   取代原 wash_stage(phase=done) 空殼通知，提供更有用的 per-step duration array
// v4.0.267: 加 complete_auto_sec — 最後一步 (Complete) 的 auto_sec 設定值
//   用途：server 可知道結束 step 設定的倒數時長（用於日誌 / 計費 / 流程驗證）
void WSClientComponent::send_wash_complete_event() {
  cJSON *ev = cJSON_CreateObject();
  if (!ev) return;
  cJSON_AddStringToObject(ev, "type", "EVENT");
  cJSON_AddStringToObject(ev, "event", "wash_complete");
  cJSON_AddStringToObject(ev, "order_sn", this->order_sn_.c_str());
  cJSON_AddNumberToObject(ev, "total", this->wash_total_);
  cJSON_AddNumberToObject(ev, "total_sec", this->wash_total_duration_s_);
  // v4.0.267: Complete step (last step) auto_sec 設定值
  if (this->wash_total_ > 0) {
    uint8_t last_idx = this->wash_total_ - 1;
    cJSON_AddNumberToObject(ev, "complete_auto_sec", this->wash_steps_[last_idx].auto_sec);
  }
  cJSON *durs = cJSON_AddArrayToObject(ev, "durations");
  if (durs) {
    for (uint8_t i = 0; i < this->wash_total_; ++i) {
      cJSON_AddItemToArray(durs, cJSON_CreateNumber(this->step_durations_s_[i]));
    }
  }
  char *out = cJSON_PrintUnformatted(ev);
  if (out) {
    this->send_event_raw(out, strlen(out));
    cJSON_free(out);
  }
  cJSON_Delete(ev);
  // === v4.0.233 [RPI-CAPTURE-UNLOCK-COMPLETE] BEGIN ===
  // 流程結束後通知 RPi 拍照（complete 場景只帶 order_sn，不帶 cost/product）
  // 兩個 DONE 路徑（主迴圈 last_step_finish_at_ 倒數 + BLUE auto-continue is_last_step）
  // 都呼叫 send_wash_complete_event() → 共用同一個 capture 觸發
  // v4.0.236: wash_step_name 改 "finish"（區分既有 GREEN→BLUE edge capture 用的 step.name="Complete"）
  //   #963 capture wash_step_name="Complete" — last step GREEN 完成照（v4.0.202 設計）
  //   #966 capture wash_step_name="finish"   — 整個流程結束結算照（v4.0.233 設計）
  this->send_rpi_capture_event("finish", /*include_order_full=*/false);
  // === v4.0.233 [RPI-CAPTURE-UNLOCK-COMPLETE] END ===
}

// v4.0.220: 完整 wash_init EVENT — 帶 5 個 order 欄位 + steps array (含 led_eff)
//   用 cJSON 建 JSON 自動處理 UTF-8 escape，避免中文 / 引號 / 反斜線炸 server parse
//   約 500-900 byte (取決於 step 數量 + UTF-8 字元數)
void WSClientComponent::send_wash_init_event(const char *src_tag) {
  cJSON *ev = cJSON_CreateObject();
  if (!ev) return;
  cJSON_AddStringToObject(ev, "type", "EVENT");
  cJSON_AddStringToObject(ev, "event", "wash_init");
  cJSON_AddStringToObject(ev, "order_sn",      this->order_sn_.c_str());
  cJSON_AddStringToObject(ev, "order_cost",    this->order_cost_.c_str());
  cJSON_AddStringToObject(ev, "order_product", this->order_product_.c_str());
  cJSON_AddStringToObject(ev, "order_name",    this->order_name_.c_str());
  cJSON_AddStringToObject(ev, "order_msg",     this->order_msg_.c_str());
  cJSON_AddNumberToObject(ev, "total", this->wash_total_);
  cJSON_AddBoolToObject(ev, "unlock_pwd_set", !this->unlock_pwd_.empty());
  cJSON *steps = cJSON_AddArrayToObject(ev, "steps");
  if (steps) {
    for (uint8_t i = 0; i < this->wash_total_; ++i) {
      cJSON *step = cJSON_CreateObject();
      if (!step) continue;
      cJSON_AddStringToObject(step, "id",       this->wash_steps_[i].id);
      cJSON_AddNumberToObject(step, "bb",       this->wash_steps_[i].bb);
      cJSON_AddNumberToObject(step, "r03",      this->wash_steps_[i].r03);
      cJSON_AddNumberToObject(step, "auto_sec", this->wash_steps_[i].auto_sec);
      cJSON_AddStringToObject(step, "name",     this->wash_steps_[i].name);
      cJSON_AddStringToObject(step, "led_name", this->wash_steps_[i].led_name);
      cJSON_AddStringToObject(step, "led_eff",  this->wash_steps_[i].led_eff);
      cJSON_AddItemToArray(steps, step);
    }
  }
  cJSON_AddStringToObject(ev, "source", src_tag ? src_tag : "?");
  char *out = cJSON_PrintUnformatted(ev);
  if (out) {
    this->send_event_raw(out, strlen(out));
    cJSON_free(out);
  }
  cJSON_Delete(ev);
}
// === v4.0.217 [WASH-EVENTS] END ===

// === v4.0.233 [RPI-CAPTURE-UNLOCK-COMPLETE] BEGIN ===
// send_rpi_capture_event — 用 cJSON 建 capture payload，安全處理中文 / 特殊字元
// step_name="unlock"    → 帶 device_id, order_sn, order_cost, order_product, wash_step_name, event_name
// step_name="complete"  → 帶 device_id, order_sn, wash_step_name, event_name
// 其他                  → 通用 capture（不帶 order_cost/product）
// 觸發點：do_wash_unlock 解鎖完成後 + send_wash_complete_event 結算後
// 風險已評估：throttle 200ms / JSON 安全（cJSON 自動 escape）/ 兩 DONE 路徑共享
void WSClientComponent::send_rpi_capture_event(const char *step_name, bool include_order_full) {
  cJSON *params = cJSON_CreateObject();
  if (!params) return;
  cJSON_AddStringToObject(params, "device_id",  this->device_id_.c_str());
  cJSON_AddStringToObject(params, "order_sn",   this->order_sn_.c_str());
  if (include_order_full) {
    cJSON_AddStringToObject(params, "order_cost",    this->order_cost_.c_str());
    cJSON_AddStringToObject(params, "order_product", this->order_product_.c_str());
  }
  cJSON_AddStringToObject(params, "wash_step_name", step_name ? step_name : "");
  cJSON_AddStringToObject(params, "event_name",     "wash");
  char *out = cJSON_PrintUnformatted(params);
  cJSON_Delete(params);
  if (!out) return;
  // 剝外層 { } 配合 send_rpi_event 的 params_json 介面（內部欄位字串）
  size_t len = strlen(out);
  std::string inner;
  if (len >= 2 && out[0] == '{' && out[len - 1] == '}') {
    inner.assign(out + 1, len - 2);
  } else {
    inner.assign(out);
  }
  cJSON_free(out);
  // dashboard log（人讀格式）
  char lbuf[256];
  if (include_order_full) {
    snprintf(lbuf, sizeof(lbuf), "device_id=%s order_sn=%s cost=%s step=%s event=wash",
             this->device_id_.c_str(), this->order_sn_.c_str(),
             this->order_cost_.c_str(), step_name ? step_name : "");
  } else {
    snprintf(lbuf, sizeof(lbuf), "device_id=%s order_sn=%s step=%s event=wash",
             this->device_id_.c_str(), this->order_sn_.c_str(),
             step_name ? step_name : "");
  }
  this->send_rpi_event("capture", inner, std::string(lbuf));
}
// === v4.0.233 [RPI-CAPTURE-UNLOCK-COMPLETE] END ===

// === v4.0.202 [RPI-EVENT-HELPER] BEGIN ===
// 發送 RPi 通知事件 — 雙發 WS1+WS2 + dashboard log [RPI] 前綴
// 容錯設計：fire-and-forget，不等 RPi 回應，不影響洗車流程
void WSClientComponent::send_rpi_event(const char *action,
                                       const std::string &params_json,
                                       const std::string &log_kvs) {
  // === v4.0.213 [RPI-EVENT-THROTTLE] BEGIN ===
  // 防 spam — 同一 action 200ms 內最多 fire 1 次。
  // 用戶 spam LED Text + Test Wash5/6 反覆切 LED 觸發 Task WDT (last_rtc_crumb=STATUS_DASH)。
  // 真兇：每個 LED CMD 同步走 send_text WS1 + lan_send_text WS2 + log_to_dashboard SSE push，
  //       高頻累積 main loop 阻塞 > 5s。
  // 修法：per-action LRU 16 slots，djb2 hash + 200ms gap，超頻直接 return。
  // Rollback：刪除整個 BEGIN/END 區塊。
  static struct { uint32_t hash; uint32_t last_ms; } slots[16] = {};
  uint32_t h = 5381;
  for (const char *p = action; *p; ++p) h = h * 33 + (uint8_t)*p;
  if (h == 0) h = 1;  // reserve 0 for empty slot
  uint32_t now = millis();
  int hit = -1, empty_idx = -1, oldest_idx = 0;
  for (int i = 0; i < 16; ++i) {
    if (slots[i].hash == h) { hit = i; break; }
    if (empty_idx < 0 && slots[i].hash == 0) empty_idx = i;
    if (slots[i].last_ms < slots[oldest_idx].last_ms) oldest_idx = i;
  }
  if (hit >= 0) {
    if ((uint32_t)(now - slots[hit].last_ms) < 200) return;  // throttled
    slots[hit].last_ms = now;
  } else {
    int idx = empty_idx >= 0 ? empty_idx : oldest_idx;
    slots[idx].hash = h;
    slots[idx].last_ms = now;
  }
  // === v4.0.213 [RPI-EVENT-THROTTLE] END ===

  // 1. JSON 雙發到 WS1+WS2
  //    格式：{"type":"EVENT","event":"rpi","action":"<action>"[,<params>][,"order_sn":"..."]}
  // v4.0.243: order_sn_ 非空時自動帶 order_sn 欄位（流程進行中 RPi 通知關聯訂單）
  //   特例：capture event 已在 send_rpi_capture_event 內透過 params_json 帶入 order_sn，這裡若 order_sn_ 仍非空會
  //         duplicate 一個 order_sn 欄位 — 但 capture 路徑 params_json 內的 order_sn 是 cJSON 序列化結果（最後欄位），
  //         整體 JSON 仍為合法（duplicate key 後者覆蓋前者，server 解析無歧義）
  std::string msg;
  msg.reserve(64 + params_json.size() + 32);
  msg.append("{\"type\":\"EVENT\",\"event\":\"rpi\",\"action\":\"");
  msg.append(action);
  msg.append("\"");
  if (!params_json.empty()) {
    msg.append(",");
    msg.append(params_json);
  }
  // v4.0.243: 加 order_sn（避開 capture：capture 的 params_json 已含 order_sn，避免 duplicate key）
  if (!this->order_sn_.empty() && strcmp(action, "capture") != 0) {
    msg.append(",\"order_sn\":\"");
    msg.append(this->order_sn_);
    msg.append("\"");
  }
  msg.append("}");
  this->send_event_raw(msg.c_str(), msg.size());

  // 2. dashboard log: [RPI] action key=val ...
  std::string logline;
  logline.reserve(16 + strlen(action) + log_kvs.size());
  logline.append("[RPI] ");
  logline.append(action);
  if (!log_kvs.empty()) {
    logline.append(" ");
    logline.append(log_kvs);
  }
  this->log_to_dashboard(logline.c_str());
}
// === v4.0.202 [RPI-EVENT-HELPER] END ===

// send_event: 發送標準 IO 事件到 WS1+WS2
// 格式：{"type":"EVENT","event":"<name>","state":<bool>[,"order_sn":"..."]}
// per-event 1500ms 節流（同名事件 1500ms 內只送一次）
// 由 YAML on_state lambda 直接呼叫
// v4.0.243: order_sn_ 非空時自動帶 order_sn 欄位（流程進行中 IO 事件關聯訂單）
void WSClientComponent::send_event(const char *event_name, bool state) {
  // v4.0.276: IO 感測器 event 改 5 秒節流 + name+state hash
  //   同名+同狀態 5s 內擋（防 sensor 抖動 spam）；相反狀態立刻放行（不漏狀態變化）
  //   用戶要求「發生事件後等 5 秒才可以重新發送，不要連著發送」
  uint32_t h = 5381;
  for (const char *p = event_name; *p; p++) h = ((h << 5) + h) + (uint8_t)*p;
  h = ((h << 5) + h) + (state ? 0x01u : 0x02u);  // fold state → true/false 各佔一個 slot
  if (!this->throttle_check(h, 5000)) {
    ESP_LOGD(TAG, "EVENT throttled (5s): %s state=%s", event_name, state ? "true" : "false");
    return;
  }
  char msg[160];
  int n;
  if (!this->order_sn_.empty()) {
    n = snprintf(msg, sizeof(msg),
                 "{\"type\":\"EVENT\",\"event\":\"%s\",\"state\":%s,\"order_sn\":\"%s\"}",
                 event_name, state ? "true" : "false", this->order_sn_.c_str());
  } else {
    n = snprintf(msg, sizeof(msg), "{\"type\":\"EVENT\",\"event\":\"%s\",\"state\":%s}",
                 event_name, state ? "true" : "false");
  }
  ESP_LOGI(TAG, "EVENT: %s state=%s", event_name, state ? "true" : "false");
  this->send_event_raw(msg, n);
}

// send_washer_status: 洗車機狀態事件（特例，多一個 color 欄位）
// 格式：{"type":"EVENT","event":"washer_status","state":true,"color":"<red|blue|green>"}
// 只在有燈 ON 時發送（由 YAML lambda 判斷後才呼叫），不會發 state=false
// 節流 key 固定為 "washer_status"（djb2 預計算為 0x9f91ecec）
//   → 所有 color 共用一個節流槽，1500ms 內只保留第一次
void WSClientComponent::send_washer_status(const char *color) {
  // djb2 hash of "washer_status"（直接用 send_event 的 helper 比較安全）
  const char *key = "washer_status";
  uint32_t h = 5381;
  for (const char *p = key; *p; p++) h = ((h << 5) + h) + (uint8_t)*p;
  if (!this->throttle_check(h)) {
    ESP_LOGD(TAG, "washer_status throttled (color=%s)", color);
    return;
  }
  char msg[160];
  int n;
  if (!this->order_sn_.empty()) {
    n = snprintf(msg, sizeof(msg),
                 "{\"type\":\"EVENT\",\"event\":\"washer_status\",\"state\":true,\"color\":\"%s\",\"order_sn\":\"%s\"}",
                 color, this->order_sn_.c_str());
  } else {
    n = snprintf(msg, sizeof(msg),
                 "{\"type\":\"EVENT\",\"event\":\"washer_status\",\"state\":true,\"color\":\"%s\"}",
                 color);
  }
  ESP_LOGI(TAG, "EVENT: washer_status color=%s", color);
  this->send_event_raw(msg, n);
}

// =========================================================================
// v3.15.26: go_enable / emergency_enable 按鈕處理 + 狀態機
// =========================================================================

// bs_state: 查 binary_sensor 當前狀態（object_id 對應）
// 找不到時回傳 false，避免 crash（失效的感測器視為未觸發）
bool WSClientComponent::bs_state(const char *oid) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  for (auto *bs : App.get_binary_sensors()) {
    if (bs->get_object_id() == oid) return bs->state;
  }
#pragma GCC diagnostic pop
  return false;
}

// on_go_button_pressed: GPIO39 啟停按鈕 on_release 時呼叫
// 流程：
//   1. 讀取當前 go_enable_arg_（可能為 "gowasher" 或 "lotto"）
//   2. 若 arg=="gowasher" 且 washer_led_blue=ON → 透過 washer_script_ 呼叫 gowasher（UART2 x5）
//      若 arg=="lotto" → 只送 EVENT，不呼叫 washer
//   3. 發送 EVENT：{"event":"go_enable","state":"off","go_enable_arg":"<arg>"}
//   注意：state 是字串 "off"（user spec），代表 NC 按鈕 ON→OFF 轉換
//   v3.15.27: 按鈕事件一律跳過 1500ms 節流（按鈕每次都要送達伺服器）
void WSClientComponent::on_go_button_pressed() {
  // v3.16.11: 500ms 軟體 debounce — 過濾機械彈跳重複觸發
  uint32_t now = millis();

  // v3.16.33: 診斷誤觸 — 印出 uptime + 距上次 WS1/WS2 連線時間 + 當下 start_stop_input state
  uint32_t uptime_s = now / 1000;
  uint32_t since_ws1_s = this->connected_time_ != 0 ? (now - this->connected_time_) / 1000 : 999;
  uint32_t since_ws2_s = this->lan_connected_time_ != 0 ? (now - this->lan_connected_time_) / 1000 : 999;
  bool ss_state = this->bs_state("start_stop_input");
  char diag[160];
  snprintf(diag, sizeof(diag),
           "[GO-DIAG] on_release fired uptime=%us ws1=%us ws2=%us ss=%s arg='%s'",
           uptime_s, since_ws1_s, since_ws2_s, ss_state ? "ON" : "OFF",
           this->go_enable_arg_.c_str());
  ESP_LOGW(TAG, "%s", diag);
  this->log_to_dashboard(diag);

  // v3.16.34: 重連後 3 秒內忽略 on_release — WS1/WS2 重連時的 SPI/TLS 負載會造成 GPIO 噪訊假觸發
  //   若是真的按下，延後 3 秒再按即可；寧可漏一次，也不要誤觸
  if ((this->connected_ && (now - this->connected_time_) < 3000) ||
      (this->lan_connected_ && (now - this->lan_connected_time_) < 3000)) {
    char guard[128];
    snprintf(guard, sizeof(guard),
             "[GO-DIAG] within 3s of WS reconnect (ws1=%us ws2=%us) → ignore as noise",
             since_ws1_s, since_ws2_s);
    ESP_LOGW(TAG, "%s", guard);
    this->log_to_dashboard(guard);
    return;
  }

  // v4.0.22: 500 → 200ms（用戶回報觸發時間過長）
  if (this->go_btn_last_ms_ != 0 && (now - this->go_btn_last_ms_) < 200) {
    ESP_LOGW(TAG, "go button debounced (%lu ms since last)",
             (unsigned long)(now - this->go_btn_last_ms_));
    return;
  }
  this->go_btn_last_ms_ = now;

  std::string arg = this->go_enable_arg_;
  if (arg.empty()) {
    ESP_LOGW(TAG, "go button pressed but arg is empty (disabled by EMG?) — still send EVENT");
  }
  ESP_LOGI(TAG, "go button pressed (arg=%s)", arg.c_str());

  // 條件觸發 UART2 gowasher（只在 arg=="gowasher" 且 blue 燈亮時）
  // v4.0.13: 用 current_led_color_ state machine 判斷，避免 raw GPIO 閃爍間隙
  if (arg == "gowasher" && this->current_led_color_ == "blue") {
    ESP_LOGI(TAG, "go button → gowasher (current_led_color=blue)");
    if (this->washer_script_) {
      std::string source = "GO";
      std::string cmd_name = "gowasher";
      // v3.16.50: gowasher 發送 log — 方便除錯誰觸發的
      ESP_LOGI(TAG, "[WASH] gowasher send: source='%s' cmd='%s' (go button)",
               source.c_str(), cmd_name.c_str());
      this->washer_script_->execute(7, source, cmd_name);  // idx 7 = gowasher
    } else {
      ESP_LOGW(TAG, "go button: washer_script_ not set, skipping gowasher call");
    }
  }

  // v3.15.27: 按鈕事件跳過節流 → 每次按下都送 EVENT
  // state 為字串 "off"（代表 NC 按下 ON→OFF），附帶當前 go_enable_arg
  // v4.0.243: 加 order_sn（流程進行中關聯訂單）
  char msg[200];
  int n;
  if (!this->order_sn_.empty()) {
    n = snprintf(msg, sizeof(msg),
      "{\"type\":\"EVENT\",\"event\":\"go_enable\",\"state\":\"off\",\"go_enable_arg\":\"%s\",\"order_sn\":\"%s\"}",
      arg.c_str(), this->order_sn_.c_str());
  } else {
    n = snprintf(msg, sizeof(msg),
      "{\"type\":\"EVENT\",\"event\":\"go_enable\",\"state\":\"off\",\"go_enable_arg\":\"%s\"}",
      arg.c_str());
  }
  this->send_event_raw(msg, n);
}

// on_emg_button_pressed: GPIO38 緊急按鈕 on_release 時呼叫
// v3.16.0 重寫流程：
//   1. 送 base EVENT {"event":"emergency_enable","state":false}（跳過節流，每次都送）
//   2. 若 emergency_enable_=false（抽獎模式）→ return
//   3. 若 washer_led_blue=ON → 送 emergency_stop EVENT → return
//   4. 若 blue=OFF → 禁用 GO 按鈕 + 呼叫 gowasher + 啟動 1500ms 自動循環
//      （check_emergency_state 中每 1500ms 重複 gowasher 直到 blue=ON 或 10 次）
void WSClientComponent::on_emg_button_pressed() {
  // v3.16.11: 500ms 軟體 debounce — 過濾機械彈跳重複觸發
  uint32_t now = millis();

  // [Step 0a] WS 重連 3 秒內忽略 — 防 WS reconnect 假觸發
  if ((this->connected_ && (now - this->connected_time_) < 3000) ||
      (this->lan_connected_ && (now - this->lan_connected_time_) < 3000)) {
    uint32_t since_ws1_s = this->connected_time_ != 0 ? (now - this->connected_time_) / 1000 : 999;
    uint32_t since_ws2_s = this->lan_connected_time_ != 0 ? (now - this->lan_connected_time_) / 1000 : 999;
    char guard[128];
    snprintf(guard, sizeof(guard),
             "[EMG-DIAG] within 3s of WS reconnect (ws1=%us ws2=%us) → ignore as noise",
             since_ws1_s, since_ws2_s);
    ESP_LOGW(TAG, "%s", guard);
    this->log_to_dashboard(guard);
    return;
  }

  // [Step 0b] 200ms 軟體 debounce
  if (this->emg_btn_last_ms_ != 0 && (now - this->emg_btn_last_ms_) < 200) {
    ESP_LOGW(TAG, "emg button debounced (%lu ms since last)",
             (unsigned long)(now - this->emg_btn_last_ms_));
    return;
  }
  this->emg_btn_last_ms_ = now;

  // [Step 0bb] v4.0.95: 洗車機 BLUE 完全靜默（用戶要求 C1）
  //   機器已停（BLUE）→ 急停沒事可做 → 不切 warn_emg / 不算連按計數 / 不送 EVENT / 不記 alert
  //   等於這次按按鈕當作沒發生（連 dashboard log 都不刷）
  if (this->current_led_color_ == "blue") {
    ESP_LOGI(TAG, "[EMG] BLUE state — silent return (C1: no warn_emg/count/EVENT/alert)");
    return;
  }

  // [Step 0c] v4.0.88: EMG 連按計數視窗 — 3s 內連按 3 次 → bypass 電流判斷
  //   保護電流 sensor 失效 / Modbus box 失聯時 cur=NaN/0 永遠卡 IDLE(lowA) 的失能情境
  //   視窗：第 1 次按下啟動 3s 計時；之後每次按 ≤3s 累計，>3s 重置為 1
  //   force_bypass_current=true 時 [Gate 5] 跳過電流檢查，GREEN 直接 fired
  //   第 1 次按下（不管後續 gate 是否擋下）→ 切 warn_emg.vsn (lan) 視覺警示
  bool first_press_in_window = false;
  if (this->emg_press_window_start_ms_ == 0 ||
      (now - this->emg_press_window_start_ms_) > 3000) {
    // 開新 window 或舊 window 已過期
    this->emg_press_window_start_ms_ = now;
    this->emg_press_count_in_window_ = 1;
    first_press_in_window = true;
  } else {
    this->emg_press_count_in_window_++;
  }
  bool force_bypass_current = (this->emg_press_count_in_window_ >= 3);
  ESP_LOGI(TAG, "[EMG] press count=%u/3 window_age=%ums force_bypass=%d",
           this->emg_press_count_in_window_,
           (unsigned)(now - this->emg_press_window_start_ms_),
           force_bypass_current ? 1 : 0);

  // 第 1 次按下無條件切 warn_emg vsn — Online/Offline 都送，不過濾任何 gate
  //   即使後續 [Gate 1~5] 擋下、抽獎模式 [Step 1] return，warn_emg 已切出
  //   後續若成功觸發，emg vsn 會在 [Step 2] 覆蓋 warn_emg（既有邏輯）
  // v4.0.99: VSN 從 vsn_map 取（key="warn_emg"），source 統一 "internet"
  if (first_press_in_window && this->led_switch_script_) {
    std::string vsn = this->vsn_lookup("warn_emg");
    std::string src = "internet";
    if (!vsn.empty()) {
      this->led_switch_script_->execute(vsn, src);
      ESP_LOGI(TAG, "[EMG] 1st press → led_switch '%s'/internet (key=warn_emg)", vsn.c_str());
    } else {
      ESP_LOGW(TAG, "[EMG] 1st press → vsn_map[warn_emg] empty, skip");
    }
  }

  // [Step 1] 抽獎模式 → 完全忽略（連 EVENT 都不送）
  if (!this->emergency_enable_) {
    ESP_LOGI(TAG, "emg button pressed but emergency_enable=OFF — ignored (lottery mode)");
    return;
  }

  // v4.0.85: 重新設計 — sequential gate flow + HMI alert 顯示判斷過程
  //   核心哲學（用戶確立）：
  //     - 不是我啟動的腳本（!wash_active）→ 不干涉
  //     - lock 中（unlock_pwd_ 非空）→ 不推進
  //     - 已暫停（wash_paused_）→ 不重複處理
  //     - LED=BLUE → 已停下，不再觸發、不送 emergency_stop EVENT
  //     - LED=RED/empty → 不推流程
  //     - LED=GREEN → 確認電流（>0.6A 視為工作中才推進，待命=lowA 忽略）
  //   每個分流組 alert 字串顯示在 HMI Y=480；EVENT 含 reason 給 Server
  // v4.0.88: alert 字串緊湊化 — " > " → ">" 去空格，整體變短（HMI / EVENT reason 都受惠）
  std::string alert = "EMG";
  bool fired = false;

  // [Gate 1] 不是我啟動的腳本 → 不觸發
  if (!this->wash_active_) {
    alert += ">!active";
    goto emg_exit;
  }
  alert += ">active";

  // [Gate 2] lock 中（unlock_pwd_ 非空）→ 不觸發
  if (!this->unlock_pwd_.empty()) {
    alert += ">lock";
    goto emg_exit;
  }
  alert += ">!lock";

  // [Gate 3] 已暫停 → 不重複處理
  if (this->wash_paused_) {
    alert += ">paused";
    goto emg_exit;
  }
  alert += ">!paused";

  // [Gate 4] LED 色分流
  if (this->current_led_color_ == "blue") {
    alert += ">BLUE";  // 已停下，不發 emergency_stop EVENT（用戶要求）
    goto emg_exit;
  }
  if (this->current_led_color_ == "red") {
    alert += ">RED";
    goto emg_exit;
  }
  if (this->current_led_color_.empty()) {
    alert += ">noPower";
    goto emg_exit;
  }
  // green
  alert += ">GREEN";

  // [Gate 5] 電流確認 — 機器是不是真的在工作（>0.6A）vs 待命（接近 0A）
  //   用 cached current_total->state；YAML 端按下時已 trigger modbus update
  //   modbus update_interval=5s 確保 cache 永遠 ≤5s 新鮮
  // v4.0.88: 連按 3 次 (force_bypass_current=true) → 跳過電流檢查直接 fired
  //   保護 sensor 失效 / Modbus box 失聯時 EMG 永遠失能的情境
  {
    float cur = NAN;
    if (this->current_total_sensor_ != nullptr) {
      cur = this->current_total_sensor_->state;  // centi-A，÷100 = A
    }
    char curbuf[16];
    if (std::isnan(cur)) {
      snprintf(curbuf, sizeof(curbuf), "NaN");
    } else {
      snprintf(curbuf, sizeof(curbuf), "%.2fA", cur / 100.0f);
    }
    if (force_bypass_current) {
      // 強制 bypass：alert 標註原因，不論電流值如何都繼續 fired
      alert += ">FORCE3x(";
      alert += curbuf;
      alert += ")";
    } else {
      if (std::isnan(cur) || (cur / 100.0f) < EMG_CURRENT_IDLE_THRESHOLD_A) {
        alert += ">IDLE(";
        alert += curbuf;
        char nbuf[8];
        snprintf(nbuf, sizeof(nbuf), " %u/3", this->emg_press_count_in_window_);
        alert += nbuf;
        alert += ")";
        goto emg_exit;
      }
      alert += ">work(";
      alert += curbuf;
      alert += ")";
    }
  }

  // [Step 2] 真正觸發 EMG — 必做動作 + 啟 gowasher loop
  fired = true;
  alert += ">Event";

  // 切 emg LED（強制，不過濾 offline）
  // v4.0.99: VSN 從 vsn_map 取（key="emg"），source 統一 "internet"
  if (this->led_switch_script_) {
    std::string vsn = this->vsn_lookup("emg");
    std::string src = "internet";
    if (!vsn.empty()) {
      this->led_switch_script_->execute(vsn, src);
      ESP_LOGI(TAG, "[EMG] force led_switch '%s'/internet (key=emg)", vsn.c_str());
    } else {
      ESP_LOGW(TAG, "[EMG] vsn_map[emg] empty, skip LED switch");
    }
  }

  // 自動暫停洗車流程（防綠→藍跳步推進）
  this->set_wash_paused(true, "EMG");
  // 禁 GO 按鈕（go_enable_arg="emg_stop"）
  this->go_enable_arg_ = "emg_stop";
  this->go_enable_arg_expires_at_ = 0;

  // 第一次 gowasher
  if (this->washer_script_) {
    std::string source = "EMG";
    std::string cmd_name = "gowasher";
    ESP_LOGI(TAG, "[WASH] gowasher send: source='%s' cmd='%s' (EMG first)",
             source.c_str(), cmd_name.c_str());
    this->washer_script_->execute(7, source, cmd_name);
  }
  // v4.0.85: 1500ms → 2000ms（用戶要求 2 秒間隔）；上限 10 次（不變）
  this->emg_gowasher_next_at_ = millis() + 2000;
  this->emg_gowasher_count_ = 1;

emg_exit:
  // [Step 3] 任何分流都送 EVENT 給 Server，含 reason（HMI alert 字串）
  // v4.0.243: 加 order_sn（流程進行中關聯訂單）
  {
    char ev[320];
    int n;
    if (!this->order_sn_.empty()) {
      n = snprintf(ev, sizeof(ev),
        "{\"type\":\"EVENT\",\"event\":\"emergency_enable\","
        "\"state\":false,\"fired\":%s,\"reason\":\"%s\",\"order_sn\":\"%s\"}",
        fired ? "true" : "false", alert.c_str(), this->order_sn_.c_str());
    } else {
      n = snprintf(ev, sizeof(ev),
        "{\"type\":\"EVENT\",\"event\":\"emergency_enable\","
        "\"state\":false,\"fired\":%s,\"reason\":\"%s\"}",
        fired ? "true" : "false", alert.c_str());
    }
    this->send_event_raw(ev, n);
  }
  ESP_LOGW(TAG, "[EMG] %s (fired=%d)", alert.c_str(), fired ? 1 : 0);

  // [Step 4] HMI Y=480 顯示 alert 字串（hmi_refresh_bb4_line 會優先顯示 alert）
  this->hmi_emg_alert_buf_ = alert;
  // 觸發 hmi_refresh_bb4_line — YAML script，需要從這邊間接觸發。Dashboard 也讀 ws_log
  this->log_to_dashboard(alert);
}

// check_emergency_state: 每 loop 呼叫，處理三件事
//   (1) go_enable_arg TTL 過期 → 回復預設 "gowasher"
//   (2) emergency_enable TTL 過期 → 回復預設 true
//   (3) v3.16.0: EMG gowasher 自動循環（每 1500ms 呼叫 gowasher，直到 blue=ON 或 10 次）
void WSClientComponent::check_emergency_state(uint32_t now) {
  // (1) go_enable_arg TTL
  if (this->go_enable_arg_expires_at_ != 0 &&
      (int32_t)(now - this->go_enable_arg_expires_at_) >= 0) {
    ESP_LOGI(TAG, "go_enable_arg TTL expired → reset to 'gowasher'");
    this->go_enable_arg_ = "gowasher";
    this->go_enable_arg_expires_at_ = 0;
  }

  // (2) emergency_enable TTL
  if (this->emergency_enable_expires_at_ != 0 &&
      (int32_t)(now - this->emergency_enable_expires_at_) >= 0) {
    ESP_LOGI(TAG, "emergency_enable TTL expired → restore to ON");
    this->emergency_enable_ = true;
    this->emergency_enable_expires_at_ = 0;
  }

  // (3) EMG gowasher 自動循環
  if (this->emg_gowasher_next_at_ != 0 &&
      (int32_t)(now - this->emg_gowasher_next_at_) >= 0) {
    // v4.0.13: 用 current_led_color_ state machine 判斷
    if (this->current_led_color_ == "blue") {
      // blue 亮了 → 停止循環，送 emergency_stop EVENT
      ESP_LOGI(TAG, "emg auto-loop: current_led_color=blue → send emergency_stop, stop loop (count=%d)",
               this->emg_gowasher_count_);
      // v4.0.243: 加 order_sn（流程進行中關聯訂單）
      if (!this->order_sn_.empty()) {
        char stop_msg[160];
        int n = snprintf(stop_msg, sizeof(stop_msg),
          "{\"type\":\"EVENT\",\"event\":\"emergency_stop\",\"state\":true,\"order_sn\":\"%s\"}",
          this->order_sn_.c_str());
        this->send_event_raw(stop_msg, n);
      } else {
        static const char stop_msg[] = "{\"type\":\"EVENT\",\"event\":\"emergency_stop\",\"state\":true}";
        this->send_event_raw(stop_msg, sizeof(stop_msg) - 1);
      }
      this->emg_gowasher_next_at_ = 0;
      this->emg_gowasher_count_ = 0;
    } else if (this->emg_gowasher_count_ >= 10) {
      // 安全上限 10 次 → 強制停止
      ESP_LOGW(TAG, "emg auto-loop: max 10 reached, stopping (blue never came ON)");
      this->emg_gowasher_next_at_ = 0;
      this->emg_gowasher_count_ = 0;
    } else if (this->current_led_color_ != "green") {
      // v4.0.30: 非綠燈（red/off）→ 停止循環（用戶規則：只在綠燈時送 gowasher）
      ESP_LOGI(TAG, "emg auto-loop: current_led_color='%s' (not green) → stop loop (count=%d)",
               this->current_led_color_.c_str(), this->emg_gowasher_count_);
      this->emg_gowasher_next_at_ = 0;
      this->emg_gowasher_count_ = 0;
    } else {
      // green 仍亮、blue 未亮 → 再呼叫 gowasher，排程下次檢查
      ESP_LOGI(TAG, "emg auto-loop: green=ON blue=OFF → gowasher again (attempt %d/10)",
               this->emg_gowasher_count_ + 1);
      if (this->washer_script_) {
        std::string source = "EMG";
        std::string cmd_name = "gowasher";
        // v3.16.50: gowasher 發送 log — 方便除錯誰觸發的
        ESP_LOGI(TAG, "[WASH] gowasher send: source='%s' cmd='%s' (EMG loop %d/10)",
                 source.c_str(), cmd_name.c_str(), this->emg_gowasher_count_ + 1);
        this->washer_script_->execute(7, source, cmd_name);
      }
      // v4.0.85: 1500ms → 2000ms（同 on_emg_button_pressed）
      this->emg_gowasher_next_at_ = now + 2000;
      this->emg_gowasher_count_++;
    }
  }

  // v3.16.30: 移除 5s fallback 邏輯
  //   debug_pwd 改為 on_hmi_password_submit 首次呼叫時 lazy init 為序號末四碼
  //   Server 主動送 set_debug_pwd CMD 可覆蓋此預設值
}

// =========================================================================
// v3.16.35: Offline Mode heartbeat watchdog
// =========================================================================
// 每 loop 呼叫（內部 1s 節流），判斷是否收到 Server "online" CMD 心跳。
//   - 首次啟動：from_ref = setup_time_（開機後 90s 才進 offline，留給 Server 連線時間）
//   - 已收過：from_ref = last_online_cmd_ms_
//   - now - from_ref > 90s  →  進入 offline_mode_（若之前是 online）
//   - online CMD handler 收到訊息時立即恢復 online（不在此處）
void WSClientComponent::check_offline_state(uint32_t now) {
  // 1s 節流（避免每 loop 重複計算 + 寫 log）
  static uint32_t last_check_ms = 0;
  if ((int32_t)(now - last_check_ms) < 1000) return;
  last_check_ms = now;

  // 參考基準：已收過 heartbeat → 以最後一次為準；否則以開機時間為準
  uint32_t ref = (this->last_online_cmd_ms_ != 0) ? this->last_online_cmd_ms_ : this->setup_time_;
  // setup_time_ 在 setup() 內寫入 millis()，開機即有值
  bool should_offline = ((int32_t)(now - ref) > 90000);

  if (should_offline && !this->offline_mode_) {
    this->offline_mode_ = true;
    ESP_LOGW(TAG, "[OFFLINE] enter → offline (no online CMD > 90s, ref=%ums ago)",
             (uint32_t)(now - ref));
    this->log_to_dashboard("[OFFLINE] enter → local flow control active");
    // v3.16.74: 用戶規則 — 開機後若還是 online 就不管 LED，錯過就錯過。
    //   原 v3.16.73 在此加了 wash_active_ → offline_led_trigger 重送 hook，已移除。
  }
}

// offline_led_trigger: 離線模式下，依當前步驟的 led_name 切換 Colorlight 節目
//   呼叫時機：GREEN→BLUE transition / IDLE→GREEN / wash_goto 後 / wash_init 後
//   只在 offline_mode_==true 時觸發；online 時 LED 由 Server 主導
//   step_idx: 當前要顯示的 step index（0-based）
// v3.16.59 簡化：
//   (a) 移除 led_update（字幕）— 所有步驟固定只做 led_switch 切 VSN
//   (b) 移除 scene light（status_light_1/2）— LED 燈帶已從硬體/YAML 拔除
//   (c) source 固定 "lan"（本地 Colorlight 直連，不透過雲端轉送）
// v4.0.99: step.led_name 改用作 vsn_map key（"01" / "02" ... 取代 "01.vsn"）
//   走 vsn_lookup 取得真實 vsn_name，source 改 "internet"（統一部署）
//   Server 送 wash_init 帶 led_name="01" 時，韌體 lookup vsn_map["01"] → 真實 vsn
//   找不到 / 空字串 → skip（不送 HTTP，保留既有 LED 顯示）
void WSClientComponent::offline_led_trigger(uint8_t step_idx) {
  // v4.0.265: 拿掉 offline_mode_ gate — 用戶規則：不論 online/offline 都本地切 LED
  //   舊版 (v3.16.74) 規則「online 時 LED 由 server 主導」，本地不切。
  //   實務上 Dashboard 小測試直接送 wash_init，server 不知道流程不會回 led_switch CMD →
  //   online 模式下 LED 完全不動。v4.0.265 改為 ESP 永遠本地觸發。
  //   若 server 同時送 led_switch CMD，Colorlight worker 1-slot latest-wins 自動處理重複。
  // if (!this->offline_mode_) return;  // ← v4.0.265 移除
  if (step_idx >= this->wash_total_) return;
  const WashStep &s = this->wash_steps_[step_idx];
  if (s.led_name[0] == '\0') return;          // 空 led_name → 不切節目（保留既有顯示）
  if (!this->led_switch_script_) return;       // YAML 腳本尚未設定 → 忽略

  // v4.0.70: breadcrumb — 進 offline_led_trigger（會 execute YAML led_switch script → cl.put）
  //   注意 execute() 裡面會同步跑 YAML lambda、最後 enqueue 到 Colorlight queue。CL_* tag
  //   會在 worker task 覆蓋此 tag。若 execute() 路徑本身 crash，tag 停在 OLED_SWITCH
  set_reboot_tag_c("OLED_SWITCH");

  // v4.0.266: caller 端 explicit 分流 — 用戶規則：
  //   `.vsn` 結尾 → led_name 已是完整 VSN 檔名，直接用
  //   其他 → 當成 vsn_map key 查表 (v4.0.99)
  std::string key(s.led_name);
  std::string name;
  if (key.size() > 4 && key.compare(key.size() - 4, 4, ".vsn") == 0) {
    name = key;  // 完整 VSN 檔名 passthrough
  } else {
    name = this->vsn_lookup(key);  // 走 vsn_map key 查表
  }
  if (name.empty()) {
    ESP_LOGW(TAG, "[OFFLINE] led_switch skip: led_name='%s' resolved empty (step_idx=%u)",
             key.c_str(), (unsigned) step_idx);
    return;
  }
  std::string src("internet");
  this->last_led_name_ = name;
  this->last_led_source_ = src;
  this->led_switch_script_->execute(name, src);
  ESP_LOGI(TAG, "[OFFLINE] led_switch → key='%s' name='%s' source='%s'",
           key.c_str(), name.c_str(), src.c_str());
}

// =========================================================================
// v3.16.0: Wash Stage Counter — LED debounce + phase transitions
// =========================================================================

// on_washer_led_change: YAML on_state lambda 呼叫
// v4.0.12 全面重寫 — 由「邊緣偵測」改為「當前燈色狀態機」
//   (1) OFF 邊緣：只更新 flash 偵測 last_off 時戳（給 washer_offline），不送 EVENT、不改 current_led_color_
//   (2) ON 邊緣：
//       - 若 color == current_led_color_ → 同色再閃 → 什麼都不做
//       - 若 color ≠ current_led_color_ → 顏色變了 → 更新 current_led_color_、送一次 washer_status EVENT
//   (3) 相位切換完全交給 check_wash_stage 用 current_led_color_ 與 prev 比對
//   效果：綠燈閃 1000 次 = 0 次誤送、0 次誤切相位
void WSClientComponent::on_washer_led_change(const char *color, bool state) {
  uint32_t now = millis();
  // v4.0.70: breadcrumb — LED 邊緣事件（R/G/B × ON/OFF）。狀態機的觸發源頭
  {
    char crumb[28];
    snprintf(crumb, sizeof(crumb), "LED:%s:%c", color, state ? '1' : '0');
    set_reboot_tag_c(crumb);
  }

  // ── 閃爍偵測（washer_offline）— 不論顏色變化與否，邊緣都要記錄 ──
  //   OFF→ON 邊緣距 last_off ≤1200ms 累計 ≥2 次 → flashing=true
  //   loop() 內的 decay 每 >2 秒無新邊緣會自動清除
  auto update_flash = [&](bool st, uint32_t &last_off, uint32_t &last_on,
                          uint8_t &hits, bool &flashing) {
    if (st) {
      if (last_off != 0 && (uint32_t)(now - last_off) <= 1200) {
        if (hits < 255) hits++;
        if (hits >= 2) flashing = true;
      }
      last_on = now;
    } else {
      last_off = now;
    }
  };
  if (strcmp(color, "green") == 0) {
    update_flash(state, this->led_green_last_off_at_, this->led_green_last_on_at_,
                 this->led_green_flash_hits_, this->led_green_flashing_);
  } else if (strcmp(color, "blue") == 0) {
    update_flash(state, this->led_blue_last_off_at_, this->led_blue_last_on_at_,
                 this->led_blue_flash_hits_, this->led_blue_flashing_);
  } else if (strcmp(color, "red") == 0) {
    update_flash(state, this->led_red_last_off_at_, this->led_red_last_on_at_,
                 this->led_red_flash_hits_, this->led_red_flashing_);
  }

  // ── OFF 邊緣：完全忽略 ──
  if (!state) return;

  // ── ON 邊緣：同色 → 忽略（閃爍中） ──
  if (this->current_led_color_ == color) return;

  // ── 顏色變了：更新狀態 + 送一次 EVENT ──
  this->current_led_color_ = color;
  this->current_led_color_since_ms_ = now;
  // v4.0.225: 取消 washer_status EVENT — wash_stage EVENT 已含 wash_phase 欄位 (green/blue/red/...)
  //   server 從 wash_stage 即可知道洗車機 LED 狀態，washer_status 變冗餘
  // this->send_washer_status(color);  // ← removed
  ESP_LOGI(TAG, "[LED] color changed → %s", color);
}

// v4.0.252: 清 current_led_color_ — wash_reset_public 跟 Test LED OFF 共用
//   on_washer_led_change 設計上 sticky（OFF 邊緣不清色），需要外部主動清
//   清掉後實機 next ON edge 會重新設定，不影響真實機台行為
void WSClientComponent::clear_current_led_color() {
  if (!this->current_led_color_.empty()) {
    ESP_LOGI(TAG, "[LED] current_led_color cleared: '%s' → ''", this->current_led_color_.c_str());
  }
  this->current_led_color_.clear();
  this->prev_led_color_for_wash_.clear();
}

// v3.16.8: handle_wash_init_doc — 三處入口共用的 wash_init 處理邏輯
// 功能：
//   (1) 解析 total + ids (格式: "id1:bb1,id2:bb2,...")
//   (2) 重設狀態 + 關閉所有泡泡
//   (3) 詳細 log 所有 step 資訊
//   (4) Bug 修復：若 Green LED 當前已亮，立即進入 GREEN 相位並開第一段泡泡
//       （不等 400ms debounce，避免 wash_init 來時 Green 已亮但狀態機卡在 IDLE）
//   (5) 送 wash_stage EVENT
std::string WSClientComponent::handle_wash_init_doc(cJSON *doc, const char *src_tag) {
  cJSON *total_it = cJSON_GetObjectItem(doc, "total");
  cJSON *ids_it = cJSON_GetObjectItem(doc, "ids");
  cJSON *steps_it = cJSON_GetObjectItem(doc, "steps");  // v3.16.35: 新格式
  cJSON *timeout_it = cJSON_GetObjectItem(doc, "bb_timeout");  // v3.16.9: 秒
  // v4.0.67: 可選 unlock_pwd — 非空字串 → 進入鎖定模式
  cJSON *unlock_it = cJSON_GetObjectItem(doc, "unlock_pwd");
  const char *unlock_pwd_in = (cJSON_IsString(unlock_it) && unlock_it->valuestring) ? unlock_it->valuestring : "";
  // v4.0.100/v4.0.111: 可選 Order 物件 — 帶 order_sn / order_name / order_msg
  //   order_sn → wash_stage EVENT + qr_url 拼接
  //   order_name → lock LED top text (x=320, y=0, 藍字) + dualcam top_text
  //   order_msg → lock LED bottom text (x=320, y=32, 紅字) + dualcam bottom_text
  this->order_sn_.clear();
  this->order_name_.clear();
  this->order_msg_.clear();
  this->order_cost_.clear();   // v4.0.215
  this->order_product_.clear();   // v4.0.215
  cJSON *order_obj = cJSON_GetObjectItem(doc, "Order");
  if (cJSON_IsObject(order_obj)) {
    cJSON *sn_it = cJSON_GetObjectItem(order_obj, "order_sn");
    cJSON *name_it = cJSON_GetObjectItem(order_obj, "order_name");
    cJSON *msg_it = cJSON_GetObjectItem(order_obj, "order_msg");
    // v4.0.215: order_cost + order_product
    cJSON *amt_it = cJSON_GetObjectItem(order_obj, "order_cost");
    cJSON *det_it = cJSON_GetObjectItem(order_obj, "order_product");
    if (cJSON_IsString(sn_it) && sn_it->valuestring) {
      this->order_sn_ = sn_it->valuestring;
    }
    if (cJSON_IsString(name_it) && name_it->valuestring) {
      this->order_name_ = name_it->valuestring;
    }
    if (cJSON_IsString(msg_it) && msg_it->valuestring) {
      this->order_msg_ = msg_it->valuestring;
    }
    // v4.0.215: order_cost cap 16 chars, order_product cap 100 chars
    if (cJSON_IsString(amt_it) && amt_it->valuestring) {
      this->order_cost_ = amt_it->valuestring;
      if (this->order_cost_.size() > 16) this->order_cost_.resize(16);
    }
    if (cJSON_IsString(det_it) && det_it->valuestring) {
      this->order_product_ = det_it->valuestring;
      if (this->order_product_.size() > 100) this->order_product_.resize(100);
    }
    ESP_LOGI(src_tag, "  Order: sn='%s' name='%s' msg='%s' amount='%s' detail='%s'",
             this->order_sn_.c_str(), this->order_name_.c_str(), this->order_msg_.c_str(),
             this->order_cost_.c_str(), this->order_product_.c_str());
  }
  int total = cJSON_IsNumber(total_it) ? total_it->valueint : 0;
  const char *ids = (cJSON_IsString(ids_it) && ids_it->valuestring) ? ids_it->valuestring : "";
  bool has_steps = cJSON_IsArray(steps_it);

  // v3.16.35: 優先接受 steps[] 陣列；若未帶則退回 ids 字串（向後相容）
  //   total 若未帶，steps 陣列長度當作 total
  if (has_steps && total <= 0) total = cJSON_GetArraySize(steps_it);
  if (total <= 0 || total > WASH_MAX_STEPS) {
    ESP_LOGW(src_tag, "wash_init REJECTED: invalid total=%d (max=%d)", total, WASH_MAX_STEPS);
    return "ERR:wash_init invalid total";
  }
  // v3.16.9: 可選 bb_timeout 參數（秒）— 覆寫泡泡 auto-off 時限
  //          範圍 10~1800 秒；無帶 → 保留當前值（預設 180s）
  if (cJSON_IsNumber(timeout_it)) {
    int t = timeout_it->valueint;
    if (t >= 10 && t <= 1800) {
      this->wash_bb_timeout_ms_ = (uint32_t)t * 1000U;
    } else {
      ESP_LOGW(src_tag, "wash_init: bb_timeout=%d out of range (10~1800), ignored", t);
    }
  }

  // v3.16.76: 用戶規則 — WS1/WS2 收到 wash_init 一律 reset + 重新載入，回 OK；
  //   JSON malformed 才回 ERR。（v3.16.74 的 no-op check 已移除）

  // (1) 重設狀態
  this->wash_all_bb_off();
  memset(this->wash_steps_, 0, sizeof(this->wash_steps_));
  this->wash_total_ = total;
  this->wash_stage_ = 0;
  this->wash_phase_ = WP_IDLE;
  this->wash_active_ = true;
  // === v4.0.255 [INDOOR-LIGHT-SCENE] BEGIN ===
  // 簡化規則（v4.0.255 重構）：
  //   wash_init 不帶 lock → 直接開 R15+R16（無條件）
  //   wash_init 帶 lock   → 只開 R15，等 wash_unlock 才開 R16（do_wash_unlock 內處理）
  //   原 v4.0.211 用 send_wash_stage_event 條件 `stage==0 && phase==GREEN && !lock` 判斷 R16
  //   太複雜且漏 corner case（NOPOWER → ... 卡住時 R16 永遠不開）→ 簡化
  // cur=red REJECT 會在 destructive reset 階段呼叫 wash_all_indoor_light_off 立即關回 OFF
  this->wash_set_indoor_light(2, true);
  if (unlock_pwd_in[0] == '\0') {
    // 不帶 lock → R16 直接開
    this->wash_set_indoor_light(3, true);
  }
  // === v4.0.255 [INDOOR-LIGHT-SCENE] END ===
  // v4.0.23: 重置 LED step dedup，確保 wash_init 後第一次 send_wash_stage_event 會送 LED
  this->last_led_step_idx_ = -1;
  // v3.16.55: 清除前次完成旗標（HMI 從「結束」切回「IDLE」等待）
  this->wash_done_flag_ = false;
  // v3.16.59: 總洗車時間歸零（step[0]→step[1] 才開始計時）
  this->wash_total_start_ms_ = 0;
  this->wash_total_duration_s_ = 0;
  // v4.0.224: 重置 step duration 追蹤陣列
  this->step_green_enter_ms_ = 0;
  memset(this->step_durations_s_, 0, sizeof(this->step_durations_s_));
  // v4.0.28: 新流程預設繼續狀態（清 pause 旗標）
  this->wash_paused_ = false;
  this->pause_frozen_remain_ms_ = 0;
  this->pause_started_at_ = 0;
  // v4.0.12: current_led_color_ 是硬體真實狀態 — 不要重置
  //   prev_led_color_for_wash_ 會在下方依 current_led_color_ 同步（避免多送一次相位轉換）

  // (2) 解析步驟：優先 steps[] 陣列（v3.16.35），退回 ids 字串（向後相容）
  int idx = 0;
  if (has_steps) {
    // v3.16.59: 新格式：steps: [{"id":"1423","bb":1,"R03":1,"auto_sec":30,
    //                           "name":"預洗","led_name":"1.vsn"}, ...]
    //   已移除 led_text / led_color / light_color（只用 led_switch 切 VSN）
    int array_len = cJSON_GetArraySize(steps_it);
    if (array_len > total) array_len = total;
    if (array_len > WASH_MAX_STEPS) array_len = WASH_MAX_STEPS;
    for (int i = 0; i < array_len; i++) {
      cJSON *s = cJSON_GetArrayItem(steps_it, i);
      if (!cJSON_IsObject(s)) continue;
      WashStep &w = this->wash_steps_[i];

      cJSON *it;
      it = cJSON_GetObjectItem(s, "id");
      if (cJSON_IsString(it) && it->valuestring)
        strncpy(w.id, it->valuestring, sizeof(w.id) - 1);
      it = cJSON_GetObjectItem(s, "bb");
      if (cJSON_IsNumber(it)) w.bb = (uint8_t)it->valueint;
      // v3.16.48: R03 軟水切換（soft_water switch）— 支援大寫 "R03" 或小寫 "r03"
      //   0=off, 1=on；步驟進入時套用，洗車完成時關閉
      it = cJSON_GetObjectItem(s, "R03");
      if (!cJSON_IsNumber(it)) it = cJSON_GetObjectItem(s, "r03");
      if (cJSON_IsNumber(it)) w.r03 = it->valueint ? 1 : 0;
      it = cJSON_GetObjectItem(s, "auto_sec");
      if (cJSON_IsNumber(it) && it->valueint >= 0) w.auto_sec = (uint32_t)it->valueint;
      it = cJSON_GetObjectItem(s, "name");
      if (cJSON_IsString(it) && it->valuestring)
        strncpy(w.name, it->valuestring, sizeof(w.name) - 1);
      // v3.16.59: led_name — Colorlight VSN 檔名（offline_led_trigger 會透過 led_switch 腳本切換）
      //   空字串或未帶欄位 → 不切節目，保留既有顯示
      it = cJSON_GetObjectItem(s, "led_name");
      if (cJSON_IsString(it) && it->valuestring)
        strncpy(w.led_name, it->valuestring, sizeof(w.led_name) - 1);
      // === v4.0.203 [RPI-EVENT-LED-EFF] BEGIN ===
      // led_eff CSV 字串（例 "a1,b1,c1,d1"）→ GREEN→BLUE edge 送 RPi event led_step.eff
      it = cJSON_GetObjectItem(s, "led_eff");
      if (cJSON_IsString(it) && it->valuestring)
        strncpy(w.led_eff, it->valuestring, sizeof(w.led_eff) - 1);
      // === v4.0.203 [RPI-EVENT-LED-EFF] END ===
      idx++;
    }
  } else {
    // 舊格式：ids="id1:bb1,id2:bb2,..."
    const char *p = ids;
    while (*p && idx < total && idx < WASH_MAX_STEPS) {
      const char *colon = strchr(p, ':');
      const char *comma = strchr(p, ',');
      if (!comma) comma = p + strlen(p);
      if (colon && colon < comma) {
        int id_len = colon - p;
        if (id_len > 7) id_len = 7;
        memcpy(this->wash_steps_[idx].id, p, id_len);
        this->wash_steps_[idx].id[id_len] = '\0';
        this->wash_steps_[idx].bb = (uint8_t)atoi(colon + 1);
      } else {
        this->wash_steps_[idx].id[0] = '\0';
        this->wash_steps_[idx].bb = 0;
      }
      // 舊格式無 auto_sec/name/LED 資訊 → 全為預設值（memset 已清零）
      idx++;
      if (*comma == '\0') break;
      p = comma + 1;
    }
  }

  // (3) 詳細 log
  // v4.0.12: current_led_color_ 為狀態機真值來源；若尚未初始化（開機後還沒看過任何 LED ON）
  //   則 fallback 讀一次 GPIO 當前值
  if (this->current_led_color_.empty()) {
    bool g = this->bs_state("washer_led_green");
    bool b = this->bs_state("washer_led_blue");
    bool r = this->bs_state("washer_led_red");
    if (r) this->current_led_color_ = "red";
    else if (b) this->current_led_color_ = "blue";
    else if (g) this->current_led_color_ = "green";
    // else leave "" → IDLE path
  }
  const std::string &cur_color = this->current_led_color_;
  ESP_LOGI(src_tag, "═══ wash_init ═══");
  ESP_LOGI(src_tag, "  total=%d  parsed=%d  fmt=%s  bb_timeout=%us",
           total, idx, has_steps ? "steps[]" : "ids",
           this->wash_bb_timeout_ms_ / 1000);
  for (int i = 0; i < idx; i++) {
    const WashStep &w = this->wash_steps_[i];
    ESP_LOGI(src_tag, "  step[%d] id='%s' bb=%d r03=%d auto=%us name='%s' led_name='%s'",
             i, w.id, w.bb, w.r03, w.auto_sec, w.name, w.led_name);
  }
  ESP_LOGI(src_tag, "  current_led_color='%s'",
           cur_color.empty() ? "(none)" : cur_color.c_str());

  // (4) v4.0.12: 依 current_led_color_ 決定初始相位（完全取代舊 GPIO/debounce 判斷）
  //     Case A/B: "" → WP_IDLE，等 on_washer_led_change 帶進 green
  //     Case C  : "green" → WP_GREEN，開 step[0] 泡泡 + R03 + LED trigger
  //     Case D  : "blue"  → WP_BLUE，wash_stage=1，啟動 auto_sec 倒數
  //     Case E  : "red"   → 拒絕 wash_init
  uint32_t now_ms_init = millis();
  // v4.0.67: 鎖定模式分支 — unlock_pwd 非空 → 只進入 IDLE + 播 3cam.vsn，不套 LED 狀態機
  // v4.0.96: lock 期間固定播 huiflow_3cam_xxx (internet)，不論 online/offline 都播
  //   用戶要求：直到解鎖才能進入 step[0].led_name 第一個影片
  if (unlock_pwd_in[0] != '\0') {
    // === v4.0.203 [UNLOCK-PWD-LOG] BEGIN ===
    // 用戶提問：第 2 次 wash_init 改 unlock_pwd 期間 HMI 輸入 race，第一次按可能不生效
    // 修法：unlock_pwd 變動時 log 提示（純除錯，行為不變）
    // 觀察 dashboard log 即可看到 race 是否發生
    if (!this->unlock_pwd_.empty() && this->unlock_pwd_ != unlock_pwd_in) {
      ESP_LOGW(src_tag, "  → unlock_pwd CHANGED: '%s' → '%s' (HMI input race risk!)",
               this->unlock_pwd_.c_str(), unlock_pwd_in);
      char log_buf[128];
      snprintf(log_buf, sizeof(log_buf),
               "[WASH] unlock_pwd CHANGED: %s → %s (HMI race risk)",
               this->unlock_pwd_.c_str(), unlock_pwd_in);
      this->log_to_dashboard(log_buf);
    }
    // === v4.0.203 [UNLOCK-PWD-LOG] END ===
    this->unlock_pwd_ = unlock_pwd_in;
    this->wash_phase_ = WP_IDLE;
    this->go_enable_arg_ = "lock";
    this->go_enable_arg_expires_at_ = 0;
    ESP_LOGI(src_tag, "  → LOCKED mode (unlock_pwd set, len=%u), phase=IDLE go_enable_arg='lock'",
             (unsigned)this->unlock_pwd_.size());
    // v4.0.220: 重排順序 — wash_init EVENT → wash_stage EVENT → NVS save → LED switch (LAST)
    //   設計理念：LED HTTP PUT 會阻塞較久（colorlight queue + chain delay），放最後最不會卡 EVENT 廣播
    //   先送 EVENT 給 server / dashboard 即時知道流程開始；LED 是後續視覺反饋
    this->prev_led_color_for_wash_ = cur_color;
    // (1) wash_init EVENT (帶完整 order + steps + led_eff)
    this->send_wash_init_event(src_tag);
    // (2) wash_stage EVENT (lock 模式下不會內部 trigger LED — unlock_pwd_ 非空 → skip offline_led_trigger)
    this->send_wash_stage_event();
    this->send_status(true);
    // (3) NVS save (write 完才能容忍掉電)
    this->wash_state_save_nvs();
    // (4) LED switch 序列 — 放最後（lock_led_sequence_script chain delay 約 1.15s）
    // v4.0.96: 不論 online/offline 都播（移除 offline_mode_ 條件）
    // v4.0.116: lock_led_sequence YAML script 內部 chain delay 避 colorlight worker queue race
    //   led_switch → 400ms → led_text 上行 → 250ms → led_text 下行 → 500ms → led_dualcam
    {
      std::string vsn = this->vsn_lookup("lock");
      if (vsn.empty()) {
        ESP_LOGW(src_tag, "  → LOCK: vsn_map[lock] empty, skip LED sequence");
      } else if (this->lock_led_sequence_script_) {
        this->lock_led_sequence_script_->execute(
          vsn, this->order_sn_, this->order_name_, this->order_msg_);
        this->last_led_name_ = vsn;
        this->last_led_source_ = "internet";
        ESP_LOGI(src_tag, "  → LOCK: lock_led_sequence vsn='%s' sn='%s' name='%s' msg='%s'",
                 vsn.c_str(), this->order_sn_.c_str(),
                 this->order_name_.c_str(), this->order_msg_.c_str());
      } else {
        ESP_LOGW(src_tag, "  → LOCK: lock_led_sequence_script_ not injected, skip");
      }
    }
    return "OK:wash_init locked";
  }
  // v4.0.67: 無 unlock_pwd → 依既有 state machine 分發（清 unlock_pwd_ 確保乾淨）
  if (!this->unlock_pwd_.empty()) {
    this->unlock_pwd_.clear();
  }
  if (cur_color == "red") {
    ESP_LOGW(src_tag, "  → current_led_color='red', wash_init REJECTED (washer in RED state)");
    this->wash_active_ = false;
    this->wash_total_ = 0;
    this->wash_stage_ = 0;
    this->wash_phase_ = WP_RED;  // v4.0.67: 明確記錄 RED phase
    this->blue_enter_at_ = 0;
    this->last_step_finish_at_ = 0;  // v4.0.93: 清 last-step finish 倒數
    // === v4.0.209 [INDOOR-LIGHT-SCENE] BEGIN ===
    // wash_init cur=red REJECT → R15/R16 全關 + 取消排程
    // (上方避免閃爍邏輯確保 R15 沒在這 path 開過，但保險起見呼叫 wash_all_indoor_light_off 清殘留)
    this->wash_all_indoor_light_off();
    // === v4.0.209 [INDOOR-LIGHT-SCENE] END ===

    this->one_min_fired_ = false;  // v4.0.202 [1MIN-VSN] reset
    memset(this->wash_steps_, 0, sizeof(this->wash_steps_));
    this->prev_led_color_for_wash_ = cur_color;
    this->wash_state_clear_nvs();
    return "ERR:wash_init washer in RED state";
  }
  // v4.0.81: 無電（LED 全暗）→ 不再 REJECT，仍載入 steps 進 WP_IDLE 等綠燈
  //   先前 v4.0.67 在 cur_color 為空時直接清空 wash_state 並 REJECT，造成 HMI 0x17/0x18
  //   按下後 WS log 看不到 wash_stage EVENT（用戶反映：「洗車機還沒亮過任何燈號時送流程
  //   一/二，WS log 只有 gowasher5/6 沒有 wash_init 資料」）。改為：保留 steps，標 WP_IDLE
  //   等綠燈邊緣推進，並送一個 wash_nopower 通知 EVENT 讓 Server 知道目前洗車機沒電。
  //   wash_stage EVENT（含完整 steps）會在下方 fall-through 到 send_wash_stage_event() 送出。
  if (cur_color.empty()) {
    ESP_LOGI(src_tag, "  → current_led_color='' (no power), loaded but waiting for green LED");
    // v4.0.243: 加 order_sn（流程進行中關聯訂單）
    if (!this->order_sn_.empty()) {
      char np[128];
      int n = snprintf(np, sizeof(np),
        "{\"type\":\"EVENT\",\"event\":\"wash_nopower\",\"order_sn\":\"%s\"}",
        this->order_sn_.c_str());
      this->send_event_raw(np, n);
    } else {
      const char *np = "{\"type\":\"EVENT\",\"event\":\"wash_nopower\"}";
      this->send_event_raw(np, strlen(np));
    }
    // 不 REJECT、不清空 — 繼續走下方 else 分支（WP_IDLE 等待綠燈）
  }
  if (cur_color == "green" && this->wash_total_ > 0) {
    this->wash_phase_ = WP_GREEN;
    uint8_t bb = this->wash_steps_[0].bb;
    this->wash_set_bb(bb);
    this->wash_apply_r03(this->wash_steps_[0].r03);
    // v4.0.224: wash_init 直接進 GREEN — 記錄 step[0] GREEN 開始時間
    this->step_green_enter_ms_ = now_ms_init;
    ESP_LOGI(src_tag, "  → current='green', entering GREEN immediately, bb=%d r03=%d",
             bb, this->wash_steps_[0].r03);
    // v4.0.21: LED 交由 send_wash_stage_event 末端統一觸發
    // v4.0.211 [INDOOR-LIGHT-SCENE] R16 turn_on 移到 send_wash_stage_event 開頭統一處理
  } else if (cur_color == "blue" && this->wash_total_ > 0) {
    // 藍燈已亮 — 視 step[0] 完成，進入 step[0]→[1] 的 BLUE 等待相位
    //   wash_stage_ = 1 代表「已完成 step[0]，正在等 GO 進 step[1] GREEN」
    //   send_wash_stage_event 會把 step_idx 回退 1 → 報 step 0
    this->wash_stage_ = 1;
    this->wash_phase_ = WP_BLUE;
    this->blue_enter_at_ = now_ms_init;
    this->wash_total_start_ms_ = now_ms_init;
    ESP_LOGW(src_tag, "  → current='blue' (washer mid-cycle), resume as step[0]→[1] BLUE");
    ESP_LOGW(src_tag, "    wash_stage_=1 phase=BLUE blue_enter_at=%u auto_sec=%us",
             now_ms_init, this->wash_steps_[0].auto_sec);
    // v4.0.21: LED 交由 send_wash_stage_event 末端統一觸發（BLUE phase step_idx=0）
  } else {
    ESP_LOGI(src_tag, "  → phase=IDLE, waiting for green LED");
  }
  // v4.0.12: 同步 prev_led_color_for_wash_ 到 current_led_color_
  //   避免 check_wash_stage 第一輪又送一次已處理的轉換
  this->prev_led_color_for_wash_ = cur_color;

  // (5) v3.16.12: wash_init 後恢復 go_enable_arg 預設值 "gowasher"
  //     （EMG 觸發後會設為 "emg_stop"，新洗車開始時應恢復正常）
  this->go_enable_arg_ = "gowasher";
  this->go_enable_arg_expires_at_ = 0;
  ESP_LOGI(src_tag, "  → go_enable_arg reset to 'gowasher'");

  // (6) 送 EVENT + STATUS — v4.0.220 重排順序
  //   設計理念：先送 wash_init EVENT 通知 server (帶完整 order + steps + led_eff)，
  //             再送 wash_stage EVENT (內部 offline_led_trigger fire LED 是最後)
  //             最後 NVS save
  // (1) wash_init EVENT (FIRST — 完整流程定義)
  this->send_wash_init_event(src_tag);
  // (2) wash_stage EVENT — 末端可能透過 offline_led_trigger 切 LED (LED 自然在最後)
  this->send_wash_stage_event();
  // v4.0.03: 恢復 send_status() — Dashboard 卡片有時靠 ws_last_status text_sensor
  this->send_status(true);  // dashboard_only — 不推 WS1/WS2
  // (3) NVS save 最後
  this->wash_state_save_nvs();
  return "OK:wash_init";
}

// check_wash_stage: 每 loop 呼叫 — 確認 LED 持續 400ms 後觸發相位轉換
void WSClientComponent::check_wash_stage(uint32_t now) {
  if (!this->wash_active_) return;

  // v4.0.67: 鎖定模式 → 跳過 state machine（所有燈色變化忽略，等 wash_unlock）
  //   吸收當前色避免 unlock 後被殘留邊緣推進流程
  // v4.0.269: REVERT v4.0.268 auto-unlock — 實機條件不符合（機器多半不會經過明確 BLUE→GREEN）
  //   未來若要重做，需考量 wash_init 後 lock 期間 LED 多半 ""→"green" 或 "green" 不動，
  //   非預設「BLUE→GREEN」邊緣
  if (!this->unlock_pwd_.empty()) {
    this->prev_led_color_for_wash_ = this->current_led_color_;
    return;
  }

  // v4.0.31: 流程暫停 → 完全跳過 state machine（用戶規則：wash_stop=true 忽略洗車機燈號變化）
  //   不推進相位、不跳步、不跑 bb 安全 auto-off、不跑 auto-continue 倒數。
  //   只同步 prev_led_color_for_wash_ 吸收期間色變邊緣，resume 時從「現在的顏色」重新開始
  //   （避免 EMG gowasher 把燈打藍的邊緣在 resume 瞬間爆發推進流程）。
  if (this->wash_paused_) {
    this->prev_led_color_for_wash_ = this->current_led_color_;
    return;
  }

  // v4.0.70: breadcrumb — 進 state machine active 分支（非 paused / non-locked）
  //   active 分支有 bb auto-off / phase transition / send_wash_stage_event / offline_led_trigger
  //   crash 若來自此路徑會被此 tag 覆蓋 CL_DONE
  set_reboot_tag_c("CWS_ACTIVE");

  // v3.16.9: 泡泡 auto-off 安全保護（每 loop 檢查）
  //   時間到 → 關閉所有泡泡，log warning 提示可能卡住
  if (this->wash_bb_expire_at_ != 0 && (int32_t)(now - this->wash_bb_expire_at_) >= 0) {
    ESP_LOGW(TAG, "wash: bubble auto-off timeout (%u ms elapsed, safety)",
             this->wash_bb_timeout_ms_);
    this->wash_all_bb_off();  // 內部會清 wash_bb_expire_at_=0
  }

  // --- v4.0.12: 相位切換由 current_led_color_ 變化驅動 ---
  //   只在 current_led_color_ != prev_led_color_for_wash_ 時檢查一次
  //   閃爍期間 current_led_color_ 不會變 → 相位穩定、無誤切
  WashPhase prev = this->wash_phase_;

  if (this->current_led_color_ != this->prev_led_color_for_wash_) {
    const std::string &cur = this->current_led_color_;
    const char *cur_c = cur.c_str();
    const char *prev_c = this->prev_led_color_for_wash_.c_str();

    if (cur == "green") {
      // * → GREEN
      // v4.0.254: WP_NOPOWER 也視為 IDLE 同等處理（修 wash_unlock cur="" 後 LED 變綠卡住 bug）
      //   情境：wash_unlock 時洗車機沒燈號 → phase=WP_NOPOWER（不是 WP_IDLE）
      //         等實機 LED 點亮綠燈 → 進來這 branch，原本三個 case 都不匹配 → 卡住
      if (this->wash_phase_ == WP_IDLE || this->wash_phase_ == WP_NOPOWER) {
        // IDLE/NOPOWER → GREEN：第 [wash_stage_] 步開工
        bool from_nopower = (this->wash_phase_ == WP_NOPOWER);
        this->wash_phase_ = WP_GREEN;
        this->wash_all_bb_off();
        if (this->wash_stage_ < this->wash_total_) {
          this->wash_set_bb(this->wash_steps_[this->wash_stage_].bb);
          this->wash_apply_r03(this->wash_steps_[this->wash_stage_].r03);
        }
        // v4.0.224: 記錄此 step 的 GREEN 開始時間（純 GREEN 期間統計用）
        this->step_green_enter_ms_ = now;
        ESP_LOGI(TAG, "wash: %s→GREEN (stage=%d, step_id=%s)",
                 from_nopower ? "NOPOWER" : "IDLE",
                 this->wash_stage_, this->wash_stage_ < this->wash_total_ ?
                 this->wash_steps_[this->wash_stage_].id : "?");
        // v4.0.211 [INDOOR-LIGHT-SCENE] R16 turn_on 移到 send_wash_stage_event 開頭統一處理
        // v4.0.21: LED 交由 send_wash_stage_event 末端統一觸發
      } else if (this->wash_phase_ == WP_BLUE) {
        // BLUE → GREEN：下一步開工
        this->wash_phase_ = WP_GREEN;
        this->blue_enter_at_ = 0;
        this->wash_all_bb_off();
        if (this->wash_stage_ < this->wash_total_) {
          this->wash_set_bb(this->wash_steps_[this->wash_stage_].bb);
          this->wash_apply_r03(this->wash_steps_[this->wash_stage_].r03);
        }
        // v4.0.224: 新 step GREEN 開始 — 記錄時間戳（用戶設計：BLUE→GREEN 開始計算）
        this->step_green_enter_ms_ = now;
        bool is_last_entering = (this->wash_stage_ == this->wash_total_ - 1);
        ESP_LOGI(TAG, "wash: BLUE→GREEN (next stage=%d, step_id=%s%s)",
                 this->wash_stage_, this->wash_stage_ < this->wash_total_ ?
                 this->wash_steps_[this->wash_stage_].id : "?",
                 is_last_entering ? " [LAST]" : "");
        // v4.0.21: LED 交由 send_wash_stage_event 末端統一觸發（修 GO/auto 路徑不一致）
        // v4.0.93: 進入 last step GREEN 時：立即送 gowasher 給機器走結束 sequence
        //   並啟動獨立的 last_step_finish_at_ 倒數（用 step[last].auto_sec）
        //   倒數結束 → DONE + welcome.vsn（不依賴後續 LED 邊緣）
        //   不論機器後續 LED 行為（停 BLUE / 切 GREEN→BLUE→GREEN→停 / 任何模式），
        //   倒數時間到就強制收尾 — 解決洗車機 last-step LED 行為不可預測的問題
        if (is_last_entering && this->washer_script_) {
          std::string source = this->offline_mode_ ? "AUTO_OFFLINE" : "AUTO_ONLINE";
          std::string cmd_name = "gowasher";
          ESP_LOGI(TAG, "[WASH] gowasher send (LAST step entry, source='%s')",
                   source.c_str());
          this->washer_script_->execute(7, source, cmd_name);
          uint32_t auto_sec = this->wash_steps_[this->wash_stage_].auto_sec;
          this->last_step_finish_at_ = now + (uint32_t)auto_sec * 1000U;
          ESP_LOGI(TAG, "[WASH] LAST step finish countdown: %us (deadline=%u)",
                   auto_sec, this->last_step_finish_at_);
        }
      } else if (this->wash_phase_ == WP_RED) {
        // RED → GREEN：恢復（不開泡泡，等下個 GREEN 邊緣）
        this->wash_phase_ = WP_GREEN;
        // v4.0.224: 記錄 GREEN 開始時間（recovery 路徑也算進該 step duration）
        this->step_green_enter_ms_ = now;
        ESP_LOGI(TAG, "wash: RED→GREEN (recovery, stage=%d, no bubble)", this->wash_stage_);
      }
    } else if (cur == "blue") {
      // v4.0.254: WP_NOPOWER → BLUE — 修實機 startwasher 後 10-15s 才亮藍燈卡住的 bug
      //   情境：wash_unlock cur="" → phase=NOPOWER，洗車機收到 startwasher 之後物理 LED 全暗
      //         10-15s 後直接亮藍燈（跳過綠燈，因為 startwasher 機器內部進入工作）
      //         原 cur="blue" branch 只處理 WP_GREEN→BLUE，NOPOWER→BLUE 落空 → phase 卡 NOPOWER
      //   修法：視為「從 stage[0] GREEN 已經完成、進入 BLUE 倒數」處理（stage++）
      if (this->wash_phase_ == WP_NOPOWER) {
        // NOPOWER → BLUE：機器啟動延遲後直接進 BLUE，視為 step[0] GREEN 完成
        this->wash_stage_++;
        this->wash_all_bb_off();
        if (this->wash_stage_ == 1 && this->wash_total_start_ms_ == 0) {
          this->wash_total_start_ms_ = now;
          ESP_LOGI(TAG, "[WASH] total-duration TIMER START (NOPOWER→BLUE shortcut) at %ums", now);
        }
        this->wash_phase_ = WP_BLUE;
        this->blue_enter_at_ = now;
        this->one_min_fired_ = false;  // v4.0.202 [1MIN-VSN] reset
        ESP_LOGI(TAG, "wash: NOPOWER→BLUE (stage=%d/%d, blue_enter_at=%u)",
                 this->wash_stage_, this->wash_total_, now);
        // 不 fire capture EVENT — NOPOWER→BLUE 沒有真正的 GREEN 期可拍照
      } else if (this->wash_phase_ == WP_GREEN) {
        // GREEN → BLUE：階段完成
        // v4.0.91: 移除「stage>=total 直接 DONE」捷徑，所有步驟統一走 BLUE 倒數
        // v4.0.93: last-step gowasher 改在 BLUE→GREEN edge 進 GREEN 時送（不在這裡）
        //   邏輯改為：last_step_finish_at_ 獨立倒數，主迴圈處理結束（不依賴 GREEN→BLUE edge）
        // v4.0.224: 該 step 的純 GREEN 期結束 — 記錄 duration (用戶設計：GREEN→BLUE 停止計算)
        //   step_idx = wash_stage_（此時還沒 ++，所以仍是該 step 的 index）
        if (this->wash_stage_ < WASH_MAX_STEPS && this->step_green_enter_ms_ != 0) {
          uint32_t dur_s = (now - this->step_green_enter_ms_) / 1000;
          if (dur_s > 65535) dur_s = 65535;  // uint16_t cap
          this->step_durations_s_[this->wash_stage_] = (uint16_t)dur_s;
          ESP_LOGI(TAG, "[WASH] step[%u] GREEN duration=%us recorded",
                   this->wash_stage_, (unsigned)dur_s);
        }
        this->wash_stage_++;
        this->wash_all_bb_off();
        if (this->wash_stage_ == 1 && this->wash_total_start_ms_ == 0) {
          this->wash_total_start_ms_ = now;
          ESP_LOGI(TAG, "[WASH] total-duration TIMER START (step[0]→step[1]) at %ums", now);
        }
        this->wash_phase_ = WP_BLUE;
        this->blue_enter_at_ = now;
        // === v4.0.202 [1MIN-VSN] BEGIN ===
        // 進入新 BLUE 倒數 → 重置 1min fire flag（每個 step 獨立 fire 一次）
        this->one_min_fired_ = false;
        // === v4.0.202 [1MIN-VSN] END ===
        ESP_LOGI(TAG, "wash: GREEN→BLUE (stage=%d/%d, blue_enter_at=%u)",
                 this->wash_stage_, this->wash_total_, now);

        // === v4.0.202 [RPI-EVENT-CAPTURE] BEGIN ===
        // 每次 GREEN→BLUE edge fire capture event 給 RPi（含 last step）
        // 容錯：純通知，不影響洗車流程（即便 RPi 掛掉主板照樣跑）
        // step_idx = wash_stage_ - 1（剛完成 GREEN 進入 BLUE 倒數的 step）
        {
          uint8_t step_idx = this->wash_stage_ - 1;
          const char *step_name = (step_idx < this->wash_total_) ? this->wash_steps_[step_idx].name : "";
          char pbuf[256];
          snprintf(pbuf, sizeof(pbuf),
                   "\"device_id\":\"%s\",\"order_sn\":\"%s\",\"wash_step_name\":\"%s\",\"event_name\":\"wash\"",
                   this->device_id_.c_str(), this->order_sn_.c_str(), step_name);
          char lbuf[200];
          snprintf(lbuf, sizeof(lbuf), "device_id=%s order_sn=%s step=%s event=wash",
                   this->device_id_.c_str(), this->order_sn_.c_str(), step_name);
          this->send_rpi_event("capture", std::string(pbuf), std::string(lbuf));
        }
        // === v4.0.202 [RPI-EVENT-CAPTURE] END ===

        // v4.0.224: led_step EVENT 取消 — wash_stage EVENT 已加 led_eff 欄位，led_step 變冗餘
        //   原 v4.0.205 設計：因為當時 wash_stage 沒帶 led_eff 才另外送 led_step 給 RPi
        //   現在 wash_stage 含 step / led_name / led_eff / auto_sec 全部資訊，led_step 多餘
        //   capture EVENT (上方 RPI-EVENT-CAPTURE 區塊) 保留 — 不同用途（觸發 RPi 拍照）
      }
    } else if (cur == "red") {
      // v4.0.67: * → RED 改為「全部清除重置」(用戶決定 option b：洗車機進紅燈無法恢復，流程必須從頭)
      ESP_LOGW(TAG, "wash: %s→RED (destructive reset, stage=%d)",
               prev_c[0] ? prev_c : "?", this->wash_stage_);
      // === v4.0.241 [WASHER-REDABORTED] BEGIN ===
      // 必須在 destructive reset 前送 EVENT — 之後 wash_total_/order_sn_/durations 都被清空
      // wash_active_=true 才送（流程進行中才算 abort）
      if (this->wash_active_) {
        this->send_washer_redaborted_event();
      }
      // === v4.0.241 [WASHER-REDABORTED] END ===
      this->wash_all_bb_off();
      this->wash_apply_r03(0);
      // === v4.0.209 [INDOOR-LIGHT-SCENE] BEGIN ===
      // *→RED destructive → R15+R16 立即 OFF + 取消排程
      this->wash_all_indoor_light_off();
      // === v4.0.209 [INDOOR-LIGHT-SCENE] END ===
      this->wash_active_ = false;
      this->wash_total_ = 0;
      this->wash_stage_ = 0;
      this->wash_phase_ = WP_RED;
      memset(this->wash_steps_, 0, sizeof(this->wash_steps_));
      this->wash_total_start_ms_ = 0;
      this->wash_total_duration_s_ = 0;
      this->blue_enter_at_ = 0;
      this->last_step_finish_at_ = 0;  // v4.0.93: 清 last-step finish 倒數

      this->one_min_fired_ = false;  // v4.0.202 [1MIN-VSN] reset
      this->last_led_step_idx_ = -1;
      this->wash_state_clear_nvs();
      // v4.0.243: 補 send_status(true) 雙保險 — 同步 ws_last_status text_sensor 為 RED 狀態
      //   防瀏覽器 SSE 重連時讀到 stale active STATUS 跳幽靈
      //   dashboard_only=true 不推 WS1/WS2（server 已收 washer_redaborted EVENT）
      this->send_status(true);
    } else if (cur.empty()) {
      // v4.0.67: * → "" (LED 全暗) — 通常不會發生（current_led_color_ sticky），但閃爍斷網仍可能
      //   送 wash_nopower EVENT 告知 Server；不破壞流程狀態（只是通知）
      ESP_LOGW(TAG, "wash: %s→'' (no power, nopower EVENT)",
               prev_c[0] ? prev_c : "?");
      // v4.0.243: 加 order_sn（流程進行中關聯訂單）
      if (!this->order_sn_.empty()) {
        char np[128];
        int n = snprintf(np, sizeof(np),
          "{\"type\":\"EVENT\",\"event\":\"wash_nopower\",\"order_sn\":\"%s\"}",
          this->order_sn_.c_str());
        this->send_event_raw(np, n);
      } else {
        const char *np = "{\"type\":\"EVENT\",\"event\":\"wash_nopower\"}";
        this->send_event_raw(np, strlen(np));
      }
    }

    this->prev_led_color_for_wash_ = cur;
    (void)cur_c;  // 未來 log 備用
  }

  // 相位變化 → 送 wash_stage EVENT（WS1/WS2）+ send_status（Dashboard 面板）
  // v3.16.45: Dashboard 「洗車階段狀態」卡片只讀 ws_last_status text_sensor，
  //           必須 send_status() 才會更新。EVENT 是給 Server 用的。
  // v3.16.73: 相位變化也同時存 NVS（下次重啟可從同一相位繼續）
  if (this->wash_phase_ != prev) {
    this->send_wash_stage_event();
    // v4.0.09: 移除 send_status — Dashboard 改從 EVENT 更新卡片（updateWashStatusFromEvent）
    this->wash_state_save_nvs();
  }

  // v4.0.93: last-step 獨立倒數（不依賴 phase / LED 邊緣）
  //   進入 last step GREEN 時設定 last_step_finish_at_ = now + step[last].auto_sec * 1000
  //   到達截止點 → 切 welcome.vsn + 清空流程狀態 + DONE
  //   不論機器在 last step 期間 LED 怎麼跳（停 BLUE / GREEN→BLUE→GREEN→停 / 任何模式），
  //   倒數時間到就強制收尾 — 解決洗車機 last-step LED 行為不可預測的問題
  // 暫停期間（wash_paused_=true）凍結倒數：當切回 unpause 時 deadline 會被平移
  //   （簡化版本：暫停就 skip，不平移；若需精確平移可加 last_step_pause_remain_ms_）
  if (this->last_step_finish_at_ != 0 && this->wash_active_ && !this->wash_paused_ &&
      (int32_t)(now - this->last_step_finish_at_) >= 0) {
    ESP_LOGI(TAG, "[WASH] LAST step finish countdown done (now=%u deadline=%u) → DONE + welcome.vsn",
             now, this->last_step_finish_at_);
    this->last_step_finish_at_ = 0;

    this->one_min_fired_ = false;  // v4.0.202 [1MIN-VSN] reset

    // 清空流程狀態（對齊 BLUE auto-continue is_last_step 分支的 cleanup）
    this->wash_phase_ = WP_DONE;
    this->blue_enter_at_ = 0;
    this->wash_apply_r03(0);     // 安全保護：關閉軟水
    this->wash_all_bb_off();     // 關閉所有泡泡
    if (this->wash_total_start_ms_ != 0) {
      this->wash_total_duration_s_ = (now - this->wash_total_start_ms_) / 1000U;
      ESP_LOGI(TAG, "[WASH] total-duration TIMER STOP = %us (last-step finish)",
               this->wash_total_duration_s_);
    }
    // v4.0.224: 記錄 last step 的純 GREEN duration
    if (this->wash_stage_ < WASH_MAX_STEPS && this->step_green_enter_ms_ != 0) {
      uint32_t dur_s = (now - this->step_green_enter_ms_) / 1000;
      if (dur_s > 65535) dur_s = 65535;
      this->step_durations_s_[this->wash_stage_] = (uint16_t)dur_s;
      ESP_LOGI(TAG, "[WASH] step[%u] LAST GREEN duration=%us recorded", this->wash_stage_, (unsigned)dur_s);
    }
    // v4.0.224: 改用 wash_complete EVENT 取代 wash_stage(phase=done) 空殼
    //   send_wash_complete_event 必須在 wash_total_ / order_sn_ 清零之前送（讀這些欄位）
    this->send_wash_complete_event();
    // 清狀態歸零
    this->wash_active_ = false;
    this->wash_total_ = 0;
    this->wash_stage_ = 0;
    this->wash_phase_ = WP_IDLE;
    memset(this->wash_steps_, 0, sizeof(this->wash_steps_));
    this->wash_done_flag_ = true;
    // v4.0.100: 清訂單資訊
    this->order_sn_.clear();
    this->order_name_.clear();
    this->order_msg_.clear();
    this->order_cost_.clear();   // v4.0.215
    this->order_product_.clear();   // v4.0.215
    this->wash_state_save_nvs();
    // === v4.0.234 [DONE-STATUS-PUSH] BEGIN ===
    // DONE 補一次 send_status 確保 dashboard ws_last_status 同步到「結束」狀態
    // 之前 v3.16.79 移除 → reload dashboard 看到 cache 過期顯示「流程還在跑」
    // dashboard_only=true 不推 WS1/WS2 (server 已收到 wash_complete EVENT 不需要重複)
    this->send_status(true);
    // === v4.0.234 [DONE-STATUS-PUSH] END ===
    // === v4.0.211 [INDOOR-LIGHT-SCENE] BEGIN ===
    // 用戶設計：DONE 切 welcome 之前 R16 立即 OFF + R15 排程 2 分鐘後 OFF
    // (R16 先關於 R15 — 順序與 v4.0.206/210 反過來)
    this->wash_set_indoor_light(3, false);
    this->indoor_light3_off_at_ = millis() + INDOOR_LIGHT3_DELAY_MS;  // 變數名保留，語義變成「R15 延遲關截止點」
    ESP_LOGI(TAG, "[WASH] DONE (last-step finish): R16 OFF, R15 scheduled OFF in 2min");
    this->log_to_dashboard("[WASH] DONE: R16 OFF, R15 scheduled OFF in 2min");
    // === v4.0.211 [INDOOR-LIGHT-SCENE] END ===
    // v4.0.99: welcome 從 vsn_map 取，source 統一 "internet"
    if (this->led_switch_script_) {
      std::string vsn = this->vsn_lookup("welcome");
      std::string src = "internet";
      if (!vsn.empty()) {
        this->led_switch_script_->execute(vsn, src);
        ESP_LOGI(TAG, "[WASH] DONE (last-step finish) -> led_switch '%s'/internet (key=welcome)", vsn.c_str());
      } else {
        ESP_LOGW(TAG, "[WASH] DONE (last-step finish): vsn_map[welcome] empty, skip");
      }
    }
    return;  // 已完成，不再進 BLUE auto-continue
  }

  // v3.16.36: Auto-continue 倒數（online / offline 都執行）
  //   BLUE 相位下，距 blue_enter_at_ 達 auto_sec 秒 → 本地自動送 gowasher
  //   auto_sec=0：該步驟不自動繼續，需手動按 GO 或 Server 送 gowasher
  //   設計變更：Server 端即使 Online，也希望由設備本地倒數控制流程節拍，避免網路延遲
  // v3.16.50: 加強 log — 明確列出 elapsed_ms / target_ms / blue_enter_at / now，方便除錯延遲
  // v3.16.52: 修復 step_idx 計算 — 進 BLUE 時 wash_stage_ 已遞增（指向下一步），
  //   當前 BLUE 這一步的 index 是 wash_stage_-1。每個 step 的 auto_sec 控制「它自己的
  //   BLUE 停留秒數」，與用戶直覺一致。原先用 wash_stage_ 會讀到下一步的 auto_sec，
  //   導致 step 0 auto_sec=0（等待開始）時仍被下一步 auto_sec 觸發 gowasher。
  // v4.0.28: 流程暫停時跳過 auto-continue 倒數（EMG 安全保護）
  //   wash_paused_=true 期間：倒數凍結，不觸發 auto gowasher。wash_start 時 blue_enter_at_
  //   會被平移回補凍結的剩餘秒數，流程自然接續。
  if (this->wash_phase_ == WP_BLUE &&
      this->blue_enter_at_ != 0 && this->wash_active_ &&
      !this->wash_paused_ &&
      this->wash_stage_ > 0) {
    uint8_t step_idx = this->wash_stage_ - 1;  // 當前 BLUE 這一步（已完成 GREEN）
    if (step_idx < this->wash_total_) {
      uint32_t auto_sec = this->wash_steps_[step_idx].auto_sec;
      uint32_t target_ms = auto_sec * 1000U;
      int32_t elapsed_ms = (int32_t)(now - this->blue_enter_at_);

      // === v4.0.202 [1MIN-VSN] BEGIN ===
      // BLUE 倒數中：當 auto_sec > 65 且剩餘 <= 60s → led_switch("1min") 一次 fire
      // 條件 auto_sec > 65：留 5s 餘裕，避免邊界 race（auto_sec=60 時剩 60s = 一啟動就 fire）
      // one_min_fired_ flag 防重複（每次 GREEN→BLUE edge 重置）
      // 設計：先切 vsn 後 fire RPi event（用戶要求順序）
      if (auto_sec > 65 && !this->one_min_fired_ && !this->wash_paused_) {
        int32_t fire_threshold_ms = (int32_t)((auto_sec - 60) * 1000U);
        if (elapsed_ms >= fire_threshold_ms) {
          this->one_min_fired_ = true;
          // 1. 先切 LED 螢幕到 1min vsn @internet
          if (this->led_switch_script_) {
            std::string vsn = this->vsn_lookup("1min");
            std::string src = "internet";
            if (!vsn.empty()) {
              this->led_switch_script_->execute(vsn, src);
              this->last_led_name_ = vsn;
              this->last_led_source_ = src;
              ESP_LOGI(TAG, "[WASH] 1min countdown remain → led_switch '%s'/internet (key=1min)", vsn.c_str());
            } else {
              ESP_LOGW(TAG, "[WASH] 1min: vsn_map[1min] empty, skip");
            }
          }
          // 2. 後 fire RPi event
          uint8_t step_idx_local = this->wash_stage_ - 1;
          const char *step_name = (step_idx_local < this->wash_total_) ? this->wash_steps_[step_idx_local].name : "";
          char pbuf[256];
          snprintf(pbuf, sizeof(pbuf),
                   "\"step\":\"%s\",\"remain_sec\":60,\"auto_sec\":%u",
                   step_name, auto_sec);
          char lbuf[200];
          snprintf(lbuf, sizeof(lbuf), "step=%s remain_sec=60 auto_sec=%u", step_name, auto_sec);
          this->send_rpi_event("one_min_remain", std::string(pbuf), std::string(lbuf));
        }
      }
      // === v4.0.202 [1MIN-VSN] END ===

      if (auto_sec > 0 && elapsed_ms >= (int32_t)target_ms) {
        // v3.16.53: 最後一步 BLUE 倒數結束 → 直接清空流程，不送 gowasher
        //   原本最後一步會送 gowasher，等洗車機再繞一圈才進 DONE；改為倒數到即完成
        bool is_last_step = (step_idx == this->wash_total_ - 1);
        ESP_LOGI(TAG,
                 "[WASH] auto-continue FIRE: step=%u auto_sec=%us elapsed=%dms target=%ums "
                 "blue_enter_at=%u now=%u offline=%d last=%d",
                 step_idx, auto_sec, elapsed_ms, target_ms,
                 this->blue_enter_at_, now, this->offline_mode_ ? 1 : 0, is_last_step ? 1 : 0);
        this->blue_enter_at_ = 0;  // 防重複觸發（待 BLUE→GREEN 後下次 BLUE 才重新設定）

        if (is_last_step) {
          // v4.0.93: last-step 改由 last_step_finish_at_ 處理（BLUE→GREEN edge 進入時設定）
          //   這個分支只作為 fallback（理論上 last_step_finish_at_ 會先 fire）
          //   若 last_step_finish_at_ 還在跑 → skip（讓主迴圈倒數處理）
          if (this->last_step_finish_at_ != 0) {
            ESP_LOGI(TAG, "[WASH] auto-continue: LAST step deferred to last_step_finish_at_ (skip)");
            return;
          }
          ESP_LOGI(TAG, "[WASH] auto-continue: LAST step %u countdown done (fallback) → DONE + welcome.vsn",
                   step_idx);
          this->wash_phase_ = WP_DONE;
          this->wash_apply_r03(0);         // 安全保護：關閉軟水
          this->wash_all_bb_off();          // 關閉所有泡泡
          // v3.16.59: 鎖定總洗車時間
          if (this->wash_total_start_ms_ != 0) {
            this->wash_total_duration_s_ = (now - this->wash_total_start_ms_) / 1000U;
            ESP_LOGI(TAG, "[WASH] total-duration TIMER STOP = %us (auto-continue last step)",
                     this->wash_total_duration_s_);
          }
          // v4.0.224: 記錄 last step 的純 GREEN duration
          if (step_idx < WASH_MAX_STEPS && this->step_green_enter_ms_ != 0) {
            uint32_t dur_s = (now - this->step_green_enter_ms_) / 1000;
            if (dur_s > 65535) dur_s = 65535;
            this->step_durations_s_[step_idx] = (uint16_t)dur_s;
            ESP_LOGI(TAG, "[WASH] step[%u] LAST GREEN (auto-continue) duration=%us recorded",
                     step_idx, (unsigned)dur_s);
          }
          // v4.0.224: 改用 wash_complete EVENT 取代 wash_stage(phase=done)
          this->send_wash_complete_event();
          // 清狀態歸零
          this->wash_active_ = false;
          this->wash_total_ = 0;
          this->wash_stage_ = 0;
          this->wash_phase_ = WP_IDLE;
          memset(this->wash_steps_, 0, sizeof(this->wash_steps_));
          // v3.16.55: 設置結束旗標 — HMI 狀態列顯示「結束」直到下次 wash_init
          this->wash_done_flag_ = true;
          // v4.0.100: 清訂單資訊
          this->order_sn_.clear();
          this->order_name_.clear();
          this->order_msg_.clear();
          this->order_cost_.clear();   // v4.0.215
          this->order_product_.clear();   // v4.0.215
          // v4.0.12: LED debounce 欄位已移除；prev_led_color_for_wash_ 留原值（硬體狀態未變）
          // v3.16.79: 移除 STATUS 自動觸發（只保留 CMD "status" 手動）
          // v3.16.73: 流程結束 → NVS 標記 active=false（下次開機就 IDLE，不會重啟後又跑一次）
          this->wash_state_save_nvs();
          // === v4.0.234 [DONE-STATUS-PUSH] BEGIN ===
          // DONE 補一次 send_status 確保 dashboard ws_last_status 同步到「結束」狀態
          this->send_status(true);
          // === v4.0.234 [DONE-STATUS-PUSH] END ===
          // === v4.0.211 [INDOOR-LIGHT-SCENE] BEGIN ===
          // 用戶設計：DONE 切 welcome 之前 R16 立即 OFF + R15 排程 2 分鐘
          this->wash_set_indoor_light(3, false);
          this->indoor_light3_off_at_ = millis() + INDOOR_LIGHT3_DELAY_MS;
          ESP_LOGI(TAG, "[WASH] DONE (auto-continue last): R16 OFF, R15 scheduled OFF in 2min");
          this->log_to_dashboard("[WASH] DONE: R16 OFF, R15 scheduled OFF in 2min");
          // === v4.0.211 [INDOOR-LIGHT-SCENE] END ===
          // v4.0.99: welcome 從 vsn_map 取，source 統一 "internet"
          if (this->led_switch_script_) {
            std::string vsn = this->vsn_lookup("welcome");
            std::string src = "internet";
            if (!vsn.empty()) {
              this->led_switch_script_->execute(vsn, src);
              ESP_LOGI(TAG, "[WASH] DONE (auto-continue last) -> led_switch '%s'/internet (key=welcome)", vsn.c_str());
            } else {
              ESP_LOGW(TAG, "[WASH] DONE (auto-continue last): vsn_map[welcome] empty, skip");
            }
          }
        } else if (this->washer_script_) {
          // === v4.0.203 [GO-ARG-AUTO-RESET] BEGIN ===
          // 用戶情境：雲端主持樂透時 GO 按鈕只抽獎（go_enable_arg!="gowasher"），
          //   斷線後 arg 永遠回不到 "gowasher" 用戶體驗差。
          //   修法：BLUE auto-continue 自動倒數結束進下一個 step 時 reset arg=gowasher
          //   後續流程恢復正常加速體驗
          // 安全分析：EMG 觸發時 wash_paused_=true，BLUE auto-continue 整個 block 不進入
          //   （line ~2486 if 守衛），故 EMG 後 arg=emg_stop 不會誤被 reset
          //
          // === v4.0.205 [GO-ARG-RESET-STEP0-ONLY] BEGIN ===
          // 用戶優化：只在 step[0] → step[1] 第一次轉換時 reset
          //   設計意圖：樂透只在第一個 step（step[0]），第一次 BLUE auto-continue 結束
          //   即進入 step[1]，此時 reset arg；step[1] 之後不再 check（精簡邏輯）
          //   wash_stage_ 在 GREEN→BLUE edge 已 ++，BLUE auto-continue 觸發時
          //   step_idx=wash_stage_-1=0 表示「step[0] 倒數結束將進 step[1]」
          if (step_idx == 0 && this->go_enable_arg_ != "gowasher") {
            ESP_LOGI(TAG, "[WASH] auto-continue (step[0]→step[1]): go_enable_arg auto-reset '%s' → 'gowasher'",
                     this->go_enable_arg_.c_str());
            this->log_to_dashboard(std::string("[WASH] go_enable_arg auto-reset (step0→1): ") +
                                   this->go_enable_arg_ + " → gowasher");
            this->go_enable_arg_ = "gowasher";
            this->go_enable_arg_expires_at_ = 0;
          }
          // === v4.0.205 [GO-ARG-RESET-STEP0-ONLY] END ===
          // === v4.0.203 [GO-ARG-AUTO-RESET] END ===
          // ── 中間步驟：非阻塞 UART2 gowasher x5（共用 EMG 使用的 script 路徑） ──
          std::string source = this->offline_mode_ ? "AUTO_OFFLINE" : "AUTO_ONLINE";
          std::string cmd_name = "gowasher";
          ESP_LOGI(TAG, "[WASH] gowasher send: source='%s' cmd='%s' (auto-continue)",
                   source.c_str(), cmd_name.c_str());
          this->washer_script_->execute(7, source, cmd_name);
        } else {
          ESP_LOGW(TAG, "[WASH] auto-continue: washer_script_ not set, skipping gowasher");
        }
      }
    }
  }
}

// send_wash_stage_event: 送 wash_stage EVENT 到 WS1+WS2
// v3.16.35: 新增 name / auto_sec / auto_remain_s / offline 欄位
//   讓 Server 能看到當前步驟的顯示名稱、預設倒數秒數、剩餘秒數、當下是否離線
void WSClientComponent::send_wash_stage_event() {
  // v4.0.70: breadcrumb — 集中式 wash EVENT 發送 + LED 切換（送出後觸發 offline_led_trigger）
  set_reboot_tag_c("WSTAGE_EVT");
  // v4.0.85: wash 狀態變化時清 EMG alert（Y=480 自動恢復顯示 wash flow）
  this->hmi_emg_alert_buf_.clear();
  // === v4.0.255 [INDOOR-LIGHT-SCENE] 簡化 ===
  // R16 開啟邏輯移到：(1) wash_init 不帶 lock 時直接開（line 2521）
  //                   (2) do_wash_unlock 解鎖成功時直接開（看下方 do_wash_unlock）
  // 不再依賴 send_wash_stage_event 條件判斷 → 簡化 + 修「卡 NOPOWER 時 R16 永不開」bug
  static const char *phase_names[] = {"idle", "green", "blue", "red", "done", "nopower"};
  const char *ps = (this->wash_phase_ <= WP_NOPOWER) ? phase_names[this->wash_phase_] : "unknown";
  // v4.0.67: 鎖定模式覆蓋顯示 phase="lock"（真實 wash_phase_=WP_IDLE，但對外語義為鎖定）
  if (!this->unlock_pwd_.empty()) ps = "lock";
  // v3.16.76: step_idx = 當前「正在執行或即將執行」的步驟 0-based index
  //   GREEN：step_idx = wash_stage_（正在跑這一步的 GREEN）
  //   BLUE ：step_idx = wash_stage_ - 1（剛完成 GREEN，BLUE 停留）
  //   對外欄位 wash_stage / step_seq 一律 = step_idx + 1（1-based），兩者語義相同不混淆
  int step_idx = this->wash_stage_;
  if (this->wash_phase_ == WP_BLUE && step_idx > 0) step_idx = step_idx - 1;
  int step_seq = (step_idx < this->wash_total_) ? step_idx + 1 : 0;
  const char *step_id = (step_idx < this->wash_total_) ? this->wash_steps_[step_idx].id : "";
  const char *step_name = (step_idx < this->wash_total_) ? this->wash_steps_[step_idx].name : "";
  uint32_t auto_sec = (step_idx < this->wash_total_) ? this->wash_steps_[step_idx].auto_sec : 0;
  uint8_t r03 = (step_idx < this->wash_total_) ? this->wash_steps_[step_idx].r03 : 0;
  // v3.16.76: 新增 bb / led_name 欄位
  uint8_t bb   = (step_idx < this->wash_total_) ? this->wash_steps_[step_idx].bb : 0;
  const char *led_name = (step_idx < this->wash_total_) ? this->wash_steps_[step_idx].led_name : "";
  // v4.0.224: led_eff (該 step 的 LED 特效 CSV) — 取代之前 RPI:led_step EVENT 的功能
  const char *led_eff = (step_idx < this->wash_total_) ? this->wash_steps_[step_idx].led_eff : "";
  // v4.0.224: step_duration_s (該 step 純 GREEN 期間秒數)
  //   GREEN→BLUE edge fired 時 step_durations_s_[step_idx] 已被記錄
  //   只有 BLUE 相位的 wash_stage EVENT 帶非零值（GREEN 還在跑時為 0）
  uint16_t step_duration_s = 0;
  if (this->wash_phase_ == WP_BLUE && step_idx >= 0 && step_idx < (int)this->wash_total_
      && step_idx < WASH_MAX_STEPS) {
    step_duration_s = this->step_durations_s_[step_idx];
  }

  // auto_remain_s：BLUE 相位 OR last step GREEN 獨立倒數時才有意義
  // v4.0.28: 暫停時回報凍結的剩餘秒數（由 pause_frozen_remain_ms_ 換算）
  // v4.0.251: 加 last_step_finish_at_ 分支 — last step GREEN 期間用獨立倒數算剩餘秒
  //   修 dashboard 「最後一段倒數不顯示」bug（之前只看 BLUE phase）
  //   STATUS send_status() 早就有這分支（v4.0.94），現在 EVENT 也補齊
  uint32_t auto_remain_s = 0;
  uint32_t now_ms_dbg = millis();
  if (this->wash_paused_ && this->pause_frozen_remain_ms_ > 0) {
    auto_remain_s = (this->pause_frozen_remain_ms_ + 999) / 1000U;  // 無條件進位
  } else if (this->wash_phase_ == WP_BLUE && this->blue_enter_at_ != 0 && auto_sec > 0) {
    uint32_t elapsed = (now_ms_dbg - this->blue_enter_at_) / 1000U;
    auto_remain_s = (elapsed >= auto_sec) ? 0 : (auto_sec - elapsed);
  } else if (this->last_step_finish_at_ != 0 && auto_sec > 0) {
    // v4.0.251: last step GREEN 獨立倒數（last_step_finish_at_ = now + auto_sec*1000）
    int32_t remain_ms = (int32_t)(this->last_step_finish_at_ - now_ms_dbg);
    auto_remain_s = remain_ms > 0 ? (uint32_t)(remain_ms / 1000) : 0;
  }
  // v3.16.76: [WASH] 除錯 log — Auto-continue 秒數亂跳時可看出 blue_enter_at_ 是否被重置
  ESP_LOGD(TAG, "[WASH] auto_remain calc: phase=%s blue_enter=%u now=%u auto_sec=%u remain=%u",
           ps, this->blue_enter_at_, now_ms_dbg, auto_sec, auto_remain_s);

  // JSON escape step_name + led_name（簡易：引號與反斜線）
  auto escape = [](const char *src, char *dst, size_t cap) {
    size_t j = 0;
    for (size_t i = 0; src[i] && j + 2 < cap; i++) {
      char c = src[i];
      if (c == '"' || c == '\\') {
        if (j + 3 >= cap) break;
        dst[j++] = '\\';
      }
      dst[j++] = c;
    }
    dst[j] = '\0';
  };
  char name_esc[48] = {0};
  char led_esc[80] = {0};
  escape(step_name, name_esc, sizeof(name_esc));
  escape(led_name, led_esc, sizeof(led_esc));

  // === v4.0.235 [WASH-STAGE-LAST-LED-NAME] BEGIN ===
  // 解析 led_name (vsn_map key) → 完整 vsn 檔名給 dashboard 顯示「LED 節目 Name」
  // 用戶觀察「節目名稱不會跳」：dashboard 優先 wash_last_led_name（完整檔名），
  // 但 wash_last_led_name 只在 STATUS 推送時更新；wash_stage EVENT 流程中沒帶 → 卡住
  // 此處 inline vsn_lookup 取完整檔名（如 "01" → "01_47ddcb...vsn"）放進 EVENT
  std::string last_led_full;
  if (led_name[0]) {
    last_led_full = this->vsn_lookup(led_name);
    if (last_led_full.empty()) last_led_full = led_name;  // fallback: 若 lookup 空就用 key 原值
  }
  char last_led_esc[96] = {0};
  escape(last_led_full.c_str(), last_led_esc, sizeof(last_led_esc));
  // === v4.0.235 [WASH-STAGE-LAST-LED-NAME] END ===

  // 計算總洗車時間（運行中動態遞增，done 後鎖定為 wash_total_duration_s_）
  uint32_t total_sec = this->wash_total_duration_s_;
  if (total_sec == 0 && this->wash_total_start_ms_ != 0) {
    total_sec = (now_ms_dbg - this->wash_total_start_ms_) / 1000U;
  }

  // v3.16.76: 新 EVENT 格式
  //   - wash_stage / wash_total 雙欄純整數（取代原 wash_stage_ 單數字，語義統一 = step_seq）
  //   - 新增 bb (小寫) / led_name
  //   - 移除 status 欄位（保留 offline 即可）
  // v4.0.83: 鎖定中（unlock_pwd_ 非空）→ EVENT 帶 unlock_pwd 欄位（明文）
  char msg[640];
  int n;
  // v4.0.112: wash_stage EVENT 只保留 order_sn（用戶要求 — order_name/msg 在 wash_init 已送過 Server 端 cache 即可）
  // STATUS 仍保留全部 3 欄位（給 Dashboard 顯示用）
  // v4.0.243: 加 wash_active 欄位 — Dashboard 純 EVENT 驅動，靠這個欄位判斷流程是否真的結束
  //   解決 *→RED destructive reset 後 wash_stage EVENT 帶 total=0 但 dashboard cache 仍 active 的幽靈
  if (!this->unlock_pwd_.empty()) {
    n = snprintf(msg, sizeof(msg),
      "{\"type\":\"EVENT\",\"event\":\"wash_stage\","
      "\"wash_active\":%s,"                          // v4.0.243 新增
      "\"wash_stage\":%d,\"wash_total\":%u,\"wash_phase\":\"%s\","
      "\"step_seq\":%d,\"step_id\":\"%s\",\"name\":\"%s\","
      "\"bb\":%u,\"r03\":%u,\"led_name\":\"%s\",\"led_eff\":\"%s\","
      "\"last_led_name\":\"%s\","                    // v4.0.235 完整 vsn 檔名
      "\"auto_sec\":%u,\"auto_remain_s\":%u,\"step_duration_s\":%u,"
      "\"total_sec\":%u,\"offline\":%s,\"paused\":%s,"
      "\"unlock_pwd\":\"%s\",\"order_sn\":\"%s\"}",
      this->wash_active_ ? "true" : "false",         // v4.0.243
      step_seq, (unsigned)this->wash_total_, ps,
      step_seq, step_id, name_esc,
      (unsigned)bb, (unsigned)r03, led_esc, led_eff,
      last_led_esc,                                  // v4.0.235
      auto_sec, auto_remain_s, (unsigned)step_duration_s, total_sec,
      this->offline_mode_ ? "true" : "false",
      this->wash_paused_ ? "true" : "false",
      this->unlock_pwd_.c_str(),
      this->order_sn_.c_str());
  } else {
    n = snprintf(msg, sizeof(msg),
      "{\"type\":\"EVENT\",\"event\":\"wash_stage\","
      "\"wash_active\":%s,"                          // v4.0.243 新增
      "\"wash_stage\":%d,\"wash_total\":%u,\"wash_phase\":\"%s\","
      "\"step_seq\":%d,\"step_id\":\"%s\",\"name\":\"%s\","
      "\"bb\":%u,\"r03\":%u,\"led_name\":\"%s\",\"led_eff\":\"%s\","
      "\"last_led_name\":\"%s\","                    // v4.0.235 完整 vsn 檔名
      "\"auto_sec\":%u,\"auto_remain_s\":%u,\"step_duration_s\":%u,"
      "\"total_sec\":%u,\"offline\":%s,\"paused\":%s,"
      "\"order_sn\":\"%s\"}",
      this->wash_active_ ? "true" : "false",         // v4.0.243
      step_seq, (unsigned)this->wash_total_, ps,
      step_seq, step_id, name_esc,
      (unsigned)bb, (unsigned)r03, led_esc, led_eff,
      last_led_esc,                                  // v4.0.235
      auto_sec, auto_remain_s, (unsigned)step_duration_s, total_sec,
      this->offline_mode_ ? "true" : "false",
      this->wash_paused_ ? "true" : "false",
      this->order_sn_.c_str());
  }
  ESP_LOGI(TAG, "EVENT: wash_stage=%d/%u phase=%s id=%s name='%s' bb=%u led='%s' auto=%u/%us offline=%d",
           step_seq, (unsigned)this->wash_total_, ps, step_id, step_name,
           (unsigned)bb, led_name, auto_remain_s, auto_sec,
           this->offline_mode_ ? 1 : 0);
  this->send_event_raw(msg, n);

  // v4.0.21: LED 切換時機統一放在 EVENT 送出「之後」— 集中化避免多路徑不一致
  //   所有會改相位/跳步的路徑（check_wash_stage / wash_init / wash_goto / NVS restore）
  //   最終都會呼叫本函數。把 LED 觸發集中在此，GO 路徑和 auto-continue 路徑走同一點。
  //   step_idx 已在上方依 phase 計算（GREEN=wash_stage_、BLUE=wash_stage_-1）。
  //   offline_led_trigger 內部自己檢查 offline_mode_ + led_name 非空 + led_switch_script_ 存在。
  //   Colorlight v4.0.20 rate-limit 200ms 會擋掉 BLUE→GREEN 瞬間可能的連發。
  // v4.0.23: 同 step_idx 內 GREEN→BLUE 相位轉換不再重發 LED（VSN 檔名相同 = 空切）
  //   只有 step_idx 真的前進/跳步時才送 HTTP PUT；wash_reset / wash_init 都會把 last_led_step_idx_ 歸 -1
  // v4.0.98: lock 中（unlock_pwd_ 非空）skip step LED 切換 — 避免覆蓋 3cam.vsn (handle_wash_init_doc lock 分支)
  //   解鎖時 do_wash_unlock 會 clear unlock_pwd_，後續 send_wash_stage_event 正常切 step[0] LED
  if (this->wash_active_ && this->unlock_pwd_.empty() &&
      step_idx >= 0 && step_idx < (int)this->wash_total_) {
    if ((int8_t)step_idx != this->last_led_step_idx_) {
      this->offline_led_trigger((uint8_t)step_idx);
      this->last_led_step_idx_ = (int8_t)step_idx;
    } else {
      ESP_LOGD(TAG, "led_switch skipped: same step_idx=%d (GREEN↔BLUE phase swap)", step_idx);
    }
  } else if (!this->unlock_pwd_.empty()) {
    ESP_LOGD(TAG, "led_switch skipped: lock mode (unlock_pwd active, 3cam.vsn keep playing)");
  }
}

// v4.0.20: wash_elapsed_sec — HMI Page 1 狀態列顯示 T{m}m{s}s 用
uint32_t WSClientComponent::wash_elapsed_sec() const {
  if (!this->wash_active_ || this->wash_total_start_ms_ == 0) return 0;
  return (uint32_t)(millis() - this->wash_total_start_ms_) / 1000U;
}

// v3.16.67: wash_reset_public — Dashboard Reset 按鈕呼叫
//   清空所有洗車階段狀態（等同 wash_init 的 "(1) 重設狀態" 區塊，
//   但不重新載入任何步驟、wash_active_ 歸 false）
//   不觸碰 offline_mode_（由心跳邏輯管理）、不改 go_enable_arg_
void WSClientComponent::wash_reset_public(const char *source) {
  // v4.0.85: 清 EMG alert（Y=480 恢復顯示）
  this->hmi_emg_alert_buf_.clear();
  // === v4.0.209 [INDOOR-LIGHT-SCENE] BEGIN ===
  // wash_reset → R15+R16 立即 OFF + 取消排程
  this->wash_all_indoor_light_off();
  // === v4.0.209 [INDOOR-LIGHT-SCENE] END ===
  // v4.0.222: LED 切 welcome.vsn 從這裡 移到函式末尾 (放在 wash_reset EVENT 之後)
  //   舊版順序: led_switch → state cleanup → EVENT (LED publish 在 EVENT 前 → 違反 LED 最後原則)
  //   新版順序: state cleanup → EVENT → led_switch (LED HTTP enqueue 最後)
  this->wash_all_bb_off();
  memset(this->wash_steps_, 0, sizeof(this->wash_steps_));
  this->wash_total_ = 0;
  this->wash_stage_ = 0;
  this->wash_phase_ = WP_IDLE;
  this->wash_active_ = false;
  this->wash_done_flag_ = false;
  this->wash_total_start_ms_ = 0;
  this->wash_total_duration_s_ = 0;
  this->blue_enter_at_ = 0;
  this->last_step_finish_at_ = 0;  // v4.0.93: 清 last-step finish 倒數
  // v4.0.224: 清 step duration 追蹤
  this->step_green_enter_ms_ = 0;
  memset(this->step_durations_s_, 0, sizeof(this->step_durations_s_));

  this->one_min_fired_ = false;  // v4.0.202 [1MIN-VSN] reset
  // v4.0.67: 清鎖定狀態
  this->unlock_pwd_.clear();
  // v4.0.243: 重排 — wash_reset EVENT 在 order_sn 清空之前送，讓 EVENT 帶 order_sn 關聯被 reset 的訂單
  //   舊版（v4.0.219）順序：清 order → fire EVENT (沒帶 order_sn)
  //   新版順序：fire EVENT (帶 order_sn) → 清 order → send_status (idle 狀態同步)
  // v4.0.219: 改用 wash_reset EVENT 取代空 wash_stage(idle) 廣播 — 一個事件就夠表達「重置」
  this->send_wash_event_simple("wash_reset", source);

  // v4.0.100/v4.0.111: 清訂單資訊（移到 EVENT 之後）
  this->order_sn_.clear();
  this->order_name_.clear();
  this->order_msg_.clear();
  this->order_cost_.clear();   // v4.0.215
  this->order_product_.clear();   // v4.0.215
  // v4.0.252: 清 current_led_color_ + prev_led_color_for_wash_（兩個都歸零）
  //   原 v4.0.12: prev_led_color_for_wash_ = current_led_color_（保留 cur 避免誤切相位）
  //   問題：current_led_color_ 是 sticky，OFF 邊緣不會清掉，造成 stale red/blue 殘留
  //         user 期望 wash_reset 後是「乾淨無 LED 狀態」結果還記得上次的色
  //   修法：reset 等於完整重置，LED 狀態歸 ""。後續真實 LED ON 邊緣自然會更新
  this->clear_current_led_color();
  // v4.0.23: 清除 LED step dedup，下次 send_wash_stage_event 會強制觸發 offline_led_trigger
  this->last_led_step_idx_ = -1;
  // R03 軟水切回 off（避免殘留上一腳本的設定）
  this->wash_apply_r03(0);

  // v4.0.222: LED switch (welcome.vsn) 放最後 — wash_reset EVENT 已先送出，LED 動作不阻塞通訊
  //   ESP-IDF colorlight worker queue 異步處理，led_switch_script execute 內部會 publish API:led_switch
  //   到 WS1+WS2 (即時 send_text)，所以「API:led_switch」frame 順序在「wash_reset EVENT」之後
  if (this->led_switch_script_) {
    std::string vsn = this->vsn_lookup("welcome");
    if (vsn.empty()) {
      vsn = "welcome_afa81f0c7bbe5bd9b920aecf0dc48adf_3136.vsn";  // fallback
    }
    this->led_switch_script_->execute(vsn, std::string("internet"));
    this->last_led_name_ = vsn;
    this->last_led_source_ = "internet";
  }
  // v3.16.73: 清掉 NVS 持久化，下次重啟不 auto-resume
  this->wash_state_clear_nvs();
  // v4.0.28: 清暫停旗標（新流程預設繼續）
  this->wash_paused_ = false;
  this->pause_frozen_remain_ms_ = 0;
  this->pause_started_at_ = 0;
  // v4.0.243: 補 send_status(true) 雙保險 — 同步 ws_last_status text_sensor 為 idle 狀態
  //   防瀏覽器 SSE 重連時讀到 stale STATUS 跳幽靈（之前 wash_reset 後 ws_last_status 卡在 active 狀態）
  this->send_status(true);
  ESP_LOGI(TAG, "wash_reset: all cleared");
}

// ═══════════════════════════════════════════════════════════════════════════
// v4.0.67: do_wash_unlock — 清鎖定旗標 + 視當前燈色套用 A 表邏輯
//   觸發路徑：(1) CMD wash_unlock  (2) HMI 密碼 == unlock_pwd_
//   必做事項：
//     - 清 unlock_pwd_ + NVS 持久化（重啟後不再鎖）
//     - go_enable_arg_ 回 "gowasher"
//     - 送 wash_unlock EVENT（Server 可據此更新店鋪狀態）
//   A 表分發（依 current_led_color_）：
//     red   → 全部清除重置（等同 check_wash_stage any→red）+ WP_RED
//     green → WP_GREEN + 開 step[0] bb/R03
//     blue  → WP_BLUE + wash_stage=1 + blue_enter_at=now + 啟動 total 計時
//     ""    → WP_NOPOWER + 送 wash_nopower EVENT
// ═══════════════════════════════════════════════════════════════════════════
void WSClientComponent::do_wash_unlock(const char *src) {
  if (this->unlock_pwd_.empty()) {
    ESP_LOGI(TAG, "[%s] do_wash_unlock: not locked, skip", src ? src : "?");
    return;
  }
  this->unlock_pwd_.clear();
  this->go_enable_arg_ = "gowasher";
  this->go_enable_arg_expires_at_ = 0;
  // v4.0.228: 解鎖也順便解除 pause — 用戶要求 stage1 入口統一處理 go_enable + pause
  //   wash_init 已在 line ~2196 清 wash_paused_=false (新流程)，這裡也清 (lock→unlock 路徑)
  //   直接清旗標 + 凍結資料，不走 set_wash_paused (避免重複 fire wash_start EVENT，
  //   因為下方 send_wash_stage_event 已會 carry paused:false 給 server)
  this->wash_paused_ = false;
  this->pause_frozen_remain_ms_ = 0;
  this->pause_started_at_ = 0;
  // === v4.0.255 [INDOOR-LIGHT-SCENE] BEGIN ===
  // 簡化規則：unlock 成功 → R16 直接 ON（不論 cur LED 色）
  //   原 v4.0.211 用 send_wash_stage_event 條件判斷，只在 phase=GREEN 才開 → 太複雜
  //   現改成：解鎖那一刻就開，後續任何 LED 變化都不影響
  this->wash_set_indoor_light(3, true);
  // === v4.0.255 [INDOOR-LIGHT-SCENE] END ===
  // 送 EVENT
  // v4.0.243: 加 order_sn（解鎖時 order_sn_ 必非空，但保險判斷）
  char ev[160];
  int n;
  if (!this->order_sn_.empty()) {
    n = snprintf(ev, sizeof(ev),
      "{\"type\":\"EVENT\",\"event\":\"wash_unlock\",\"source\":\"%s\",\"order_sn\":\"%s\"}",
      src ? src : "?", this->order_sn_.c_str());
  } else {
    n = snprintf(ev, sizeof(ev),
      "{\"type\":\"EVENT\",\"event\":\"wash_unlock\",\"source\":\"%s\"}",
      src ? src : "?");
  }
  this->send_event_raw(ev, n);
  // === v4.0.233 [RPI-CAPTURE-UNLOCK-COMPLETE] BEGIN ===
  // 解鎖後通知 RPi 拍照（unlock 場景帶完整 order 三欄位）
  this->send_rpi_capture_event("unlock", /*include_order_full=*/true);
  // === v4.0.233 [RPI-CAPTURE-UNLOCK-COMPLETE] END ===
  ESP_LOGI(TAG, "[%s] wash_unlock: cleared, dispatching via current_led_color='%s'",
           src ? src : "?", this->current_led_color_.c_str());

  const std::string &cur = this->current_led_color_;
  uint32_t now = millis();

  if (cur == "red") {
    // 全部清除重置（對齊 check_wash_stage any→red 分支）
    ESP_LOGW(TAG, "[%s] unlock→RED: destructive reset", src ? src : "?");
    this->wash_all_bb_off();
    this->wash_apply_r03(0);
    // === v4.0.209 [INDOOR-LIGHT-SCENE] BEGIN ===
    // unlock→RED destructive → R15+R16 立即 OFF + 取消排程
    this->wash_all_indoor_light_off();
    // === v4.0.209 [INDOOR-LIGHT-SCENE] END ===
    this->wash_active_ = false;
    this->wash_total_ = 0;
    this->wash_stage_ = 0;
    this->wash_phase_ = WP_RED;
    memset(this->wash_steps_, 0, sizeof(this->wash_steps_));
    this->wash_total_start_ms_ = 0;
    this->wash_total_duration_s_ = 0;
    this->blue_enter_at_ = 0;
    this->last_step_finish_at_ = 0;  // v4.0.93: 清 last-step finish 倒數

    this->one_min_fired_ = false;  // v4.0.202 [1MIN-VSN] reset
    this->last_led_step_idx_ = -1;
    this->wash_state_clear_nvs();
  } else if (cur == "green" && this->wash_total_ > 0) {
    this->wash_phase_ = WP_GREEN;
    this->wash_stage_ = 0;
    this->wash_all_bb_off();
    this->wash_set_bb(this->wash_steps_[0].bb);
    this->wash_apply_r03(this->wash_steps_[0].r03);
    // v4.0.224: 解鎖進入 step[0] GREEN — 記錄開始時間
    this->step_green_enter_ms_ = now;
    ESP_LOGI(TAG, "[%s] unlock→GREEN: open step[0] bb=%d r03=%d",
             src ? src : "?", this->wash_steps_[0].bb, this->wash_steps_[0].r03);
    // v4.0.211 [INDOOR-LIGHT-SCENE] R16 turn_on 移到 send_wash_stage_event 開頭統一處理
  } else if (cur == "blue" && this->wash_total_ > 0) {
    this->wash_phase_ = WP_BLUE;
    this->wash_stage_ = 1;
    this->blue_enter_at_ = now;
    this->wash_total_start_ms_ = now;
    ESP_LOGI(TAG, "[%s] unlock→BLUE: resume as step[0]→[1]", src ? src : "?");
  } else if (cur.empty()) {
    this->wash_phase_ = WP_NOPOWER;
    // v4.0.243: 加 order_sn（解鎖時 order_sn_ 必非空，但保險判斷）
    if (!this->order_sn_.empty()) {
      char np[128];
      int n = snprintf(np, sizeof(np),
        "{\"type\":\"EVENT\",\"event\":\"wash_nopower\",\"order_sn\":\"%s\"}",
        this->order_sn_.c_str());
      this->send_event_raw(np, n);
    } else {
      const char *np = "{\"type\":\"EVENT\",\"event\":\"wash_nopower\"}";
      this->send_event_raw(np, strlen(np));
    }
    ESP_LOGW(TAG, "[%s] unlock→NOPOWER: no LED on", src ? src : "?");
  }
  this->prev_led_color_for_wash_ = cur;
  this->send_wash_stage_event();
  this->send_status(true);
  this->wash_state_save_nvs();
  // v4.0.96: HMI 解鎖路徑 → 4 格顯示「验证成功」覆蓋原密碼數字
  if (src && std::string(src) == "HMI") {
    this->hmi_show_unlock_success();
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// v4.0.28: set_wash_paused — 洗車流程暫停 / 恢復 helper
//   paused=true：
//     - BLUE phase + auto_sec>0：凍結剩餘 ms（pause_frozen_remain_ms_）
//     - 其他 phase：只記旗標，暫停無實際倒數可凍
//     - 設 pause_started_at_ = now
//   paused=false：
//     - 若 pause_frozen_remain_ms_>0 且仍在 BLUE：平移 blue_enter_at_ 讓剩餘秒數延續
//       blue_enter_at_ = now - (target_ms - remain_ms) = now - elapsed_at_pause
//     - 清 pause_frozen_remain_ms_ / pause_started_at_
//   副作用：send_wash_stage_event() + send_status(true) 同步 Dashboard
// ═══════════════════════════════════════════════════════════════════════════
void WSClientComponent::set_wash_paused(bool paused, const char *source_tag) {
  if (this->wash_paused_ == paused) {
    ESP_LOGD(TAG, "[%s] set_wash_paused(%d) — already in that state, skip",
             source_tag ? source_tag : "?", paused ? 1 : 0);
    return;
  }
  uint32_t now = millis();
  if (paused) {
    this->wash_paused_ = true;
    this->pause_started_at_ = now;
    this->pause_frozen_remain_ms_ = 0;
    if (this->wash_phase_ == WP_BLUE && this->blue_enter_at_ != 0 &&
        this->wash_stage_ > 0 && this->wash_stage_ <= this->wash_total_) {
      uint8_t step_idx = this->wash_stage_ - 1;
      uint32_t auto_sec = this->wash_steps_[step_idx].auto_sec;
      if (auto_sec > 0) {
        uint32_t target_ms = auto_sec * 1000U;
        uint32_t elapsed_ms = now - this->blue_enter_at_;
        uint32_t remain_ms = (elapsed_ms < target_ms) ? (target_ms - elapsed_ms) : 0;
        this->pause_frozen_remain_ms_ = remain_ms;
        ESP_LOGI(TAG, "[%s] wash PAUSED: step=%u auto_sec=%us elapsed=%ums remain=%ums",
                 source_tag ? source_tag : "?", step_idx, auto_sec, elapsed_ms, remain_ms);
      } else {
        ESP_LOGI(TAG, "[%s] wash PAUSED: BLUE but auto_sec=0 (no countdown to freeze)",
                 source_tag ? source_tag : "?");
      }
    } else {
      ESP_LOGI(TAG, "[%s] wash PAUSED: phase=%d (no active countdown)",
               source_tag ? source_tag : "?", (int)this->wash_phase_);
    }
  } else {
    this->wash_paused_ = false;
    if (this->pause_frozen_remain_ms_ > 0 && this->wash_phase_ == WP_BLUE &&
        this->wash_stage_ > 0 && this->wash_stage_ <= this->wash_total_) {
      uint8_t step_idx = this->wash_stage_ - 1;
      uint32_t auto_sec = this->wash_steps_[step_idx].auto_sec;
      if (auto_sec > 0) {
        uint32_t target_ms = auto_sec * 1000U;
        uint32_t remain_ms = this->pause_frozen_remain_ms_;
        uint32_t elapsed_at_pause = (remain_ms < target_ms) ? (target_ms - remain_ms) : 0;
        this->blue_enter_at_ = now - elapsed_at_pause;
        ESP_LOGI(TAG, "[%s] wash RESUMED: step=%u auto_sec=%us remain=%ums blue_enter_at shifted",
                 source_tag ? source_tag : "?", step_idx, auto_sec, remain_ms);
      }
    } else {
      ESP_LOGI(TAG, "[%s] wash RESUMED: phase=%d (no countdown to restore)",
               source_tag ? source_tag : "?", (int)this->wash_phase_);
    }
    this->pause_frozen_remain_ms_ = 0;
    this->pause_started_at_ = 0;
  }
  // v4.0.227: set_wash_paused 內部自動 fire wash_stop / wash_start EVENT
  //   讓所有觸發路徑（CMD wash_stop / EMG / 其他）一致 — 只要 paused 狀態改變就有對應 EVENT
  //   省掉 CMD handler 重複 fire；EMG 觸發 pause 也會自動帶 wash_stop 通知 server
  this->send_wash_event_simple(paused ? "wash_stop" : "wash_start",
                               source_tag ? source_tag : "INTERNAL");
  // 同步 Dashboard + Server
  this->send_wash_stage_event();
  this->send_status(true);
}

// ═══════════════════════════════════════════════════════════════════════════
// v3.16.73: Wash flow state persistence via NVS (ESP32 internal flash)
// ═══════════════════════════════════════════════════════════════════════════
// 寫入時機：
//   (1) handle_wash_init_doc 完成後 — full blob
//   (2) check_wash_stage 轉換相位後（GREEN→BLUE / BLUE→GREEN / →DONE）
// 讀取時機：
//   loop() 開機 2s 後 one-time 觸發（pending_state_load_done_ 旗標）
// 清除時機：
//   wash_reset_public()
// 重啟恢復規則：
//   - wash_init 中斷 → NVS 無 active flag → 重啟 IDLE
//   - BLUE 倒數重算（blue_enter_at_ = millis()）
//   - 只恢復 bb1~bb4 + R03 軟水；其他 switch 不動
//   - online → 不動 LED（Server 主導）
//   - offline_mode_ 變 true 時 + wash_active_ → offline_led_trigger 重送 LED
// Blob 結構：header(32B) + 18 step × 112B = 2048B（NVS blob 4KB 限內）
// ═══════════════════════════════════════════════════════════════════════════
namespace {
static const char *NVS_WASH_NS = "hfwash";
static const char *NVS_WASH_KEY = "state";
static const uint32_t WASH_NVS_MAGIC = 0xC0FFEE42;
// v4.0.67: 版本 2 — 新增 unlock_pwd (32 byte cstring at tail of header)
// v4.0.100: 版本 3 — 新增 order_sn / order_name，header 64→128
// v4.0.111: 版本 4 — 新增 order_msg[64]，header 128→192
// v4.0.203: 版本 5 — WashStep 結構新增 led_eff[40]，sizeof(WashStep) 110→150
// v4.0.215: 版本 6 — 新增 order_cost[16] + order_product[100]，header 192→308
//   舊 v5 blob 開機 load 時 version mismatch 直接拒絕 → IDLE（流程中重啟需 server 重派 wash_init）
static const uint8_t WASH_NVS_VERSION = 6;

struct __attribute__((packed)) WashStateBlob {
  uint32_t magic;                // 0  : WASH_NVS_MAGIC
  uint8_t  version;              // 4  : WASH_NVS_VERSION
  uint8_t  flags;                // 5  : bit0=active, bit1=done_flag
  uint8_t  wash_stage;           // 6
  uint8_t  wash_total;           // 7
  uint8_t  wash_phase;           // 8
  uint8_t  reserved1[3];         // 9-11
  uint32_t auto_sec_remain;      // 12 : BLUE 剩餘秒（log 用，恢復重算）
  uint32_t total_duration_s;     // 16
  uint32_t wash_bb_timeout_ms;   // 20
  uint8_t  reserved2[8];         // 24-31
  // v4.0.67: unlock_pwd (null-terminated, max 31 chars + NUL)
  char     unlock_pwd[32];       // 32-63
  // v4.0.100: 訂單資訊
  char     order_sn[24];         // 64-87  (null-terminated)
  char     order_name[32];       // 88-119 (null-terminated, UTF-8 中文約可放 8~10 字)
  uint8_t  reserved3[8];         // 120-127
  // v4.0.111: 訂單訊息（lock LED 紅字 + dualcam bottom_text）
  char     order_msg[64];        // 128-191 (UTF-8 約 20 字)
  // v4.0.215: 訂單金額 + 訂單明細
  char     order_cost[16];     // 192-207  (例 "$1,200")
  char     order_product[100];    // 208-307  (商品/套餐摘要，UTF-8 約 33 字)
  // steps[] 跟在後面
};
static const size_t WASH_BLOB_HEADER_SIZE = 308;
}  // namespace

// ═══════════════════════════════════════════════════════════════════════════
// v4.0.99: VSN map (key → vsn_name) 持久化 NVS namespace "vsn_map"
//   每個 key 一個 nvs_set_str（簡單，避免設計 blob schema）
//   key 列表：warn_emg / emg / welcome / lock / hmi_camtest / hmi_text / 01~09
// ═══════════════════════════════════════════════════════════════════════════
namespace {
constexpr const char *NVS_VSN_NS = "vsn_map";
}  // namespace

// v4.0.117+: ping/pong log flag (default false = silent — 30s/次 noise 太大)
//   YAML switch "ping_log_sw" 開關，預設 OFF
bool WSClientComponent::show_ping_log_ = false;

void WSClientComponent::vsn_init_defaults() {
  // v4.0.99: 預設 keys 寫死於程式碼，setup 後 vsn_load_all_from_nvs 用 NVS 值覆蓋
  //   實際值由 CMD vsn_update 部署時設定；NVS 內已有值會覆蓋預設（持久化優先）
  // v4.0.103: 全部填上用戶 Colorlight 清單真實 VSN（hmi_camtest 與 lock 同 vsn）
  // v4.0.114: 對齊用戶實際 Colorlight 播放盒清單 — 5 個 key 更新（warn_emg / welcome / lock / hmi_camtest / 05）
  // v4.0.115: 用戶 2026-04-30 第二次同步 — Colorlight 換了 welcome / lock 兩個節目
  //   - welcome: 6187_1329 → 3999_3136
  //   - lock:    5a59_10616 → c2215_10625
  //   - hmi_camtest 跟著 lock 換
  // v4.0.116: welcome 又換 hash（3999 → afa81f）
  this->vsn_map_["warn_emg"]    = "warn_emg_0864216cbbb9c9bdb18da5afc13a7bbf_1329.vsn";
  this->vsn_map_["emg"]         = "emg_4a0bc81a84dd2508cf40f375456d965b_1324.vsn";
  this->vsn_map_["welcome"]     = "welcome_afa81f0c7bbe5bd9b920aecf0dc48adf_3136.vsn";
  this->vsn_map_["lock"]        = "lock_c2215b6c142325160126e0db0c4c3eef_10625.vsn";
  // === v4.0.202 [HMI-CAMTEST-REMOVED] BEGIN ===
  // 用戶要求移除 hmi_camtest key（HMI 0x30 / Dashboard Test LED Camera 改直接用 lock）
  // Rollback：取消註解下一行恢復
  // this->vsn_map_["hmi_camtest"] = "lock_c2215b6c142325160126e0db0c4c3eef_10625.vsn";
  // === v4.0.202 [HMI-CAMTEST-REMOVED] END ===
  this->vsn_map_["hmi_text"]    = "huiflow_text_e41dd09cdea4b9c80ed6260c08eca422_3639.vsn";
  // v4.0.271: 用戶 2026-05-19 同步 — 01~09 全部換新 hash + 命名加 "_c_" infix（size 從 1323/1324 → 2394/2395）
  // v4.0.272: 改成硬寫進韌體 — vsn_load_all_from_nvs 內 skip 01~09，且開機時 erase NVS 舊值
  //   → 既有部署機 OTA 後預設即正確，不需要 vsn_bulk_update CMD
  //   → vsn_update CMD 對 01~09 寫入只 in-memory 生效，重啟後韌體 default 復原
  this->vsn_map_["01"] = "01_c_7fde8e415c4d6b531ece65c00ed474c3_2394.vsn";
  this->vsn_map_["02"] = "02_c_224e2b338c5115a7d7967523a836c0ee_2395.vsn";
  this->vsn_map_["03"] = "03_c_445654031ce5e8a875b201ce741584b9_2395.vsn";
  this->vsn_map_["04"] = "04_c_de6455945cc7dcac2ce10173d1a0dcdb_2395.vsn";
  this->vsn_map_["05"] = "05_c_0ee2ea22a9749f00383dc5218abbc793_2395.vsn";
  this->vsn_map_["06"] = "06_c_4658bc82b2d7f75b04996a8bb7407aa5_2395.vsn";
  this->vsn_map_["07"] = "07_c_55ad53546f4e0da824fd9927edad4a13_2395.vsn";
  this->vsn_map_["08"] = "08_c_e9899987eeab969d267a5cc12efaec4f_2395.vsn";
  this->vsn_map_["09"] = "09_c_364fa3b195f8756d8db920d492c49c9b_2395.vsn";
  // v4.0.117+: 加 1min 倒數提示 vsn (CD 倒數動畫節目)
  this->vsn_map_["1min"] = "1min_7cdb915d8c9b96085184731484a0a5cb_1324.vsn";
  ESP_LOGI(TAG, "[VSN] defaults init: %u keys", (unsigned) this->vsn_map_.size());
}

// v4.0.272 [VSN-FIRMWARE-HARDCODE]：01~09 強制走韌體預設，不從 NVS 載入
//   理由：v4.0.271 換新 hash 後，舊機 NVS 內舊值會優先覆蓋預設 → 需主動推 vsn_bulk_update
//   現改成「skip load + erase NVS」一次升級即可，未來再換 vsn 只要改韌體 OTA
//   其他 keys（warn_emg/emg/welcome/lock/hmi_text/1min）行為不變，仍可透過 vsn_update 動態
static bool vsn_key_is_firmware_hardcoded(const std::string &key) {
  return (key.size() == 2 &&
          key[0] >= '0' && key[0] <= '9' &&
          key[1] >= '0' && key[1] <= '9');  // "00"~"99" 涵蓋 01~09
}

void WSClientComponent::vsn_load_all_from_nvs() {
  nvs_handle_t h;
  // v4.0.272: 先以 READWRITE 開啟，方便清除 01~09 的舊 NVS 值
  if (nvs_open(NVS_VSN_NS, NVS_READWRITE, &h) != ESP_OK) {
    ESP_LOGI(TAG, "[VSN] no NVS data, use defaults");
    return;
  }
  int loaded = 0, skipped = 0, erased = 0;
  bool need_commit = false;
  for (auto &kv : this->vsn_map_) {
    // v4.0.272: 01~09 韌體 hardcoded → skip load + erase NVS（一次升級即清乾淨）
    if (vsn_key_is_firmware_hardcoded(kv.first)) {
      skipped++;
      esp_err_t er = nvs_erase_key(h, kv.first.c_str());
      if (er == ESP_OK) {
        erased++;
        need_commit = true;
        ESP_LOGI(TAG, "[VSN] erased NVS key '%s' (firmware-hardcoded)", kv.first.c_str());
      }
      continue;
    }
    size_t len = 0;
    if (nvs_get_str(h, kv.first.c_str(), nullptr, &len) != ESP_OK || len == 0) continue;
    char *buf = (char *) malloc(len);
    if (!buf) continue;
    if (nvs_get_str(h, kv.first.c_str(), buf, &len) == ESP_OK) {
      kv.second = std::string(buf);
      ESP_LOGI(TAG, "[VSN] loaded '%s' = '%s'", kv.first.c_str(), kv.second.c_str());
      loaded++;
    }
    free(buf);
  }
  if (need_commit) nvs_commit(h);
  nvs_close(h);
  ESP_LOGI(TAG, "[VSN] NVS load done: %d loaded, %d skipped (firmware), %d erased",
           loaded, skipped, erased);
}

void WSClientComponent::vsn_save_to_nvs(const std::string &key, const std::string &name) {
  nvs_handle_t h;
  if (nvs_open(NVS_VSN_NS, NVS_READWRITE, &h) != ESP_OK) {
    ESP_LOGW(TAG, "[VSN] save '%s': nvs_open failed", key.c_str());
    return;
  }
  esp_err_t err = nvs_set_str(h, key.c_str(), name.c_str());
  if (err == ESP_OK) err = nvs_commit(h);
  nvs_close(h);
  ESP_LOGI(TAG, "[VSN] saved '%s' = '%s' (err=%d)", key.c_str(), name.c_str(), (int) err);
}

void WSClientComponent::vsn_set(const std::string &key, const std::string &name) {
  this->vsn_map_[key] = name;
  this->vsn_save_to_nvs(key, name);
}

void WSClientComponent::wash_state_save_nvs() {
  nvs_handle_t h;
  esp_err_t err = nvs_open(NVS_WASH_NS, NVS_READWRITE, &h);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "wash_state_save_nvs: nvs_open err=%d", (int) err);
    return;
  }

  size_t total_size = WASH_BLOB_HEADER_SIZE + sizeof(this->wash_steps_);
  uint8_t *buf = (uint8_t *) malloc(total_size);
  if (!buf) { nvs_close(h); return; }
  memset(buf, 0, total_size);

  WashStateBlob *hdr = (WashStateBlob *) buf;
  hdr->magic = WASH_NVS_MAGIC;
  hdr->version = WASH_NVS_VERSION;
  hdr->flags = (this->wash_active_ ? 0x01 : 0) | (this->wash_done_flag_ ? 0x02 : 0);
  hdr->wash_stage = this->wash_stage_;
  hdr->wash_total = this->wash_total_;
  hdr->wash_phase = (uint8_t) this->wash_phase_;
  if (this->wash_phase_ == WP_BLUE && this->blue_enter_at_ != 0 &&
      this->wash_stage_ > 0 && this->wash_stage_ - 1 < WASH_MAX_STEPS) {
    uint32_t elapsed_s = (millis() - this->blue_enter_at_) / 1000U;
    uint32_t au = this->wash_steps_[this->wash_stage_ - 1].auto_sec;
    hdr->auto_sec_remain = (au > elapsed_s) ? (au - elapsed_s) : 0;
  }
  hdr->total_duration_s = this->wash_total_duration_s_;
  hdr->wash_bb_timeout_ms = this->wash_bb_timeout_ms_;
  // v4.0.67: 寫 unlock_pwd（最長 31 char，尾端 NUL）
  memset(hdr->unlock_pwd, 0, sizeof(hdr->unlock_pwd));
  if (!this->unlock_pwd_.empty()) {
    size_t cp = this->unlock_pwd_.size();
    if (cp > sizeof(hdr->unlock_pwd) - 1) cp = sizeof(hdr->unlock_pwd) - 1;
    memcpy(hdr->unlock_pwd, this->unlock_pwd_.data(), cp);
  }
  // v4.0.100: 寫 order_sn / order_name（null-terminated）
  memset(hdr->order_sn, 0, sizeof(hdr->order_sn));
  if (!this->order_sn_.empty()) {
    size_t cp = this->order_sn_.size();
    if (cp > sizeof(hdr->order_sn) - 1) cp = sizeof(hdr->order_sn) - 1;
    memcpy(hdr->order_sn, this->order_sn_.data(), cp);
  }
  memset(hdr->order_name, 0, sizeof(hdr->order_name));
  if (!this->order_name_.empty()) {
    size_t cp = this->order_name_.size();
    if (cp > sizeof(hdr->order_name) - 1) cp = sizeof(hdr->order_name) - 1;
    memcpy(hdr->order_name, this->order_name_.data(), cp);
  }
  // v4.0.111: 寫 order_msg
  memset(hdr->order_msg, 0, sizeof(hdr->order_msg));
  if (!this->order_msg_.empty()) {
    size_t cp = this->order_msg_.size();
    if (cp > sizeof(hdr->order_msg) - 1) cp = sizeof(hdr->order_msg) - 1;
    memcpy(hdr->order_msg, this->order_msg_.data(), cp);
  }
  // v4.0.215: 寫 order_cost + order_product
  memset(hdr->order_cost, 0, sizeof(hdr->order_cost));
  if (!this->order_cost_.empty()) {
    size_t cp = this->order_cost_.size();
    if (cp > sizeof(hdr->order_cost) - 1) cp = sizeof(hdr->order_cost) - 1;
    memcpy(hdr->order_cost, this->order_cost_.data(), cp);
  }
  memset(hdr->order_product, 0, sizeof(hdr->order_product));
  if (!this->order_product_.empty()) {
    size_t cp = this->order_product_.size();
    if (cp > sizeof(hdr->order_product) - 1) cp = sizeof(hdr->order_product) - 1;
    memcpy(hdr->order_product, this->order_product_.data(), cp);
  }
  memcpy(buf + WASH_BLOB_HEADER_SIZE, this->wash_steps_, sizeof(this->wash_steps_));

  err = nvs_set_blob(h, NVS_WASH_KEY, buf, total_size);
  if (err == ESP_OK) err = nvs_commit(h);
  nvs_close(h);
  free(buf);

  ESP_LOGI(TAG, "wash_state_save_nvs: err=%d stage=%u/%u phase=%u flags=0x%02X",
           (int) err, this->wash_stage_, this->wash_total_,
           (unsigned) this->wash_phase_, hdr->flags);
}

bool WSClientComponent::wash_state_load_nvs() {
  nvs_handle_t h;
  esp_err_t err = nvs_open(NVS_WASH_NS, NVS_READONLY, &h);
  if (err != ESP_OK) return false;

  size_t expected = WASH_BLOB_HEADER_SIZE + sizeof(this->wash_steps_);
  size_t actual = 0;
  err = nvs_get_blob(h, NVS_WASH_KEY, nullptr, &actual);
  if (err != ESP_OK || actual == 0) { nvs_close(h); return false; }
  if (actual != expected) {
    ESP_LOGW(TAG, "wash_state_load_nvs: blob size mismatch %u vs %u",
             (unsigned) actual, (unsigned) expected);
    nvs_close(h);
    return false;
  }

  uint8_t *buf = (uint8_t *) malloc(actual);
  if (!buf) { nvs_close(h); return false; }
  err = nvs_get_blob(h, NVS_WASH_KEY, buf, &actual);
  nvs_close(h);
  if (err != ESP_OK) { free(buf); return false; }

  WashStateBlob *hdr = (WashStateBlob *) buf;
  if (hdr->magic != WASH_NVS_MAGIC || hdr->version != WASH_NVS_VERSION) {
    // v4.0.67: v1 blob（32B header）遇到 v2（64B header）直接放棄恢復，避免 offset 錯誤
    ESP_LOGW(TAG, "wash_state_load_nvs: version mismatch (have=%u expect=%u) → discard",
             (unsigned) hdr->version, (unsigned) WASH_NVS_VERSION);
    free(buf);
    return false;
  }
  if ((hdr->flags & 0x01) == 0) {
    // 非 active → 不恢復
    free(buf);
    return false;
  }
  if (hdr->wash_total == 0 || hdr->wash_total > WASH_MAX_STEPS) {
    ESP_LOGW(TAG, "wash_state_load_nvs: invalid total=%u", (unsigned) hdr->wash_total);
    free(buf);
    return false;
  }

  // 恢復 state
  this->wash_active_ = true;
  this->wash_stage_ = hdr->wash_stage;
  this->wash_total_ = hdr->wash_total;
  this->wash_phase_ = (WashPhase) hdr->wash_phase;
  this->wash_done_flag_ = (hdr->flags & 0x02) != 0;
  this->wash_total_duration_s_ = hdr->total_duration_s;
  if (hdr->wash_bb_timeout_ms >= 10000U && hdr->wash_bb_timeout_ms <= 1800000U) {
    this->wash_bb_timeout_ms_ = hdr->wash_bb_timeout_ms;
  }
  // v4.0.67: 恢復 unlock_pwd（若曾處於鎖定狀態）
  hdr->unlock_pwd[sizeof(hdr->unlock_pwd) - 1] = '\0';  // 防呆
  if (hdr->unlock_pwd[0] != '\0') {
    this->unlock_pwd_ = hdr->unlock_pwd;
    this->go_enable_arg_ = "lock";
    ESP_LOGI(TAG, "wash_state_load_nvs: restored unlock_pwd (locked mode)");
  } else {
    this->unlock_pwd_.clear();
  }
  // v4.0.100/v4.0.111: 恢復 order_sn / order_name / order_msg
  hdr->order_sn[sizeof(hdr->order_sn) - 1] = '\0';
  hdr->order_name[sizeof(hdr->order_name) - 1] = '\0';
  hdr->order_msg[sizeof(hdr->order_msg) - 1] = '\0';
  // v4.0.215: 恢復 order_cost + order_product
  hdr->order_cost[sizeof(hdr->order_cost) - 1] = '\0';
  hdr->order_product[sizeof(hdr->order_product) - 1] = '\0';
  this->order_sn_     = hdr->order_sn;
  this->order_name_   = hdr->order_name;
  this->order_msg_    = hdr->order_msg;
  this->order_cost_ = hdr->order_cost;
  this->order_product_ = hdr->order_product;
  if (!this->order_sn_.empty() || !this->order_name_.empty() || !this->order_msg_.empty()
      || !this->order_cost_.empty() || !this->order_product_.empty()) {
    ESP_LOGI(TAG, "wash_state_load_nvs: restored order sn='%s' name='%s' amount='%s'",
             this->order_sn_.c_str(), this->order_name_.c_str(), this->order_cost_.c_str());
  }
  memcpy(this->wash_steps_, buf + WASH_BLOB_HEADER_SIZE, sizeof(this->wash_steps_));
  free(buf);

  // v4.0.12: NVS restore — 依 wash_phase_ 回推 current_led_color_ + 同步 prev
  //   避免 restore 後第一輪 check_wash_stage 又誤判為「顏色變化」觸發重複轉換
  if (this->wash_phase_ == WP_GREEN) this->current_led_color_ = "green";
  else if (this->wash_phase_ == WP_BLUE) this->current_led_color_ = "blue";
  else if (this->wash_phase_ == WP_RED) this->current_led_color_ = "red";
  this->prev_led_color_for_wash_ = this->current_led_color_;

  // v4.0.55: BLUE 倒數延續 — 用 auto_sec_remain 重算 blue_enter_at_
  //   save 時記的是剩餘秒；restore 時倒推 blue_enter_at_ = now - (auto_sec - remain)*1000
  //   remain=0 → blue_enter_at 已在「target 秒之前」→ check_wash_stage 下一輪直接 fire gowasher
  //   remain>0 → 正常倒數繼續
  if (this->wash_phase_ == WP_BLUE && this->wash_stage_ > 0 &&
      this->wash_stage_ - 1 < WASH_MAX_STEPS) {
    uint32_t au = this->wash_steps_[this->wash_stage_ - 1].auto_sec;
    uint32_t remain = hdr->auto_sec_remain;
    if (remain > au) remain = au;
    uint32_t elapsed = au - remain;
    this->blue_enter_at_ = millis() - elapsed * 1000U;
  } else {
    this->blue_enter_at_ = 0;
  }

  // 決定當前活躍 step index
  //   GREEN: step_idx = wash_stage_
  //   BLUE:  step_idx = wash_stage_ - 1
  uint8_t active_idx = 0;
  if (this->wash_phase_ == WP_GREEN) {
    active_idx = this->wash_stage_;
  } else if (this->wash_phase_ == WP_BLUE && this->wash_stage_ > 0) {
    active_idx = this->wash_stage_ - 1;
  }

  // v4.0.55: NVS restore 改為「純資料載入」— 不做任何硬體動作
  //   不呼叫 wash_set_bb / wash_apply_r03 / wash_all_bb_off
  //   不呼叫 offline_led_trigger（避免開機 2s 時 HTTP PUT 可能觸發重啟）
  //   正常相位轉換由 check_wash_stage 接管；LED 交給 server / 使用者下一個 GO 觸發
  //   設 last_led_step_idx_ = active_idx → send_wash_stage_event 的 LED dedup 會跳過
  if (active_idx < WASH_MAX_STEPS && active_idx < this->wash_total_) {
    this->last_led_step_idx_ = (int8_t) active_idx;
  }

  ESP_LOGW(TAG, "wash_state_load_nvs: restored stage=%u/%u phase=%u active_idx=%u done=%u (pure-data, no HW actions)",
           this->wash_stage_, this->wash_total_, (unsigned) this->wash_phase_,
           active_idx, this->wash_done_flag_ ? 1 : 0);
  this->log_to_dashboard(std::string("[RESTORE] wash state loaded from NVS (pure-data)"));

  // v4.0.55: 送 wash_stage_restore EVENT（與一般 wash_stage 區隔，方便 server 辨識是 restore）
  //   改走 inline send — 用最小化 payload，不觸發 offline_led_trigger
  this->send_wash_stage_restore_event();
  // Dashboard 卡片同步（STATUS 本來就不帶 LED HTTP）
  this->send_status(true);

  return true;
}

// v4.0.55: send_wash_stage_restore_event — NVS restore 專用 EVENT
//   語義：告訴 server「ESP32 剛開機，從 NVS 把洗車狀態復活」
//   與 wash_stage 的差異：event name 不同、純通知性質、不帶 LED 觸發
void WSClientComponent::send_wash_stage_restore_event() {
  int step_seq = 0;
  const char *ps = "idle";
  if (this->wash_phase_ == WP_GREEN) { step_seq = this->wash_stage_ + 1; ps = "green"; }
  else if (this->wash_phase_ == WP_BLUE) { step_seq = this->wash_stage_; ps = "blue"; }
  else if (this->wash_phase_ == WP_RED) { ps = "red"; }

  uint32_t auto_sec = 0, auto_remain = 0;
  if (this->wash_phase_ == WP_BLUE && this->wash_stage_ > 0 &&
      this->wash_stage_ - 1 < WASH_MAX_STEPS) {
    auto_sec = this->wash_steps_[this->wash_stage_ - 1].auto_sec;
    if (this->blue_enter_at_ != 0) {
      uint32_t elapsed = (millis() - this->blue_enter_at_) / 1000U;
      auto_remain = (auto_sec > elapsed) ? (auto_sec - elapsed) : 0;
    }
  }

  char msg[320];
  int n = snprintf(msg, sizeof(msg),
    "{\"type\":\"EVENT\",\"event\":\"wash_stage_restore\","
    "\"wash_stage\":%d,\"wash_total\":%u,\"wash_phase\":\"%s\","
    "\"step_seq\":%d,\"auto_sec\":%u,\"auto_remain_s\":%u}",
    step_seq, (unsigned) this->wash_total_, ps,
    step_seq, auto_sec, auto_remain);
  ESP_LOGI(TAG, "EVENT: wash_stage_restore stage=%d/%u phase=%s remain=%u/%us",
           step_seq, (unsigned) this->wash_total_, ps, auto_remain, auto_sec);
  this->send_event_raw(msg, n);
}

void WSClientComponent::wash_state_clear_nvs() {
  nvs_handle_t h;
  esp_err_t err = nvs_open(NVS_WASH_NS, NVS_READWRITE, &h);
  if (err != ESP_OK) return;
  nvs_erase_key(h, NVS_WASH_KEY);
  nvs_commit(h);
  nvs_close(h);
  ESP_LOGI(TAG, "wash_state_clear_nvs: erased");
}

// wash_all_bb_off: 關閉所有泡泡 bb1~bb4 並清除 auto-off timer
void WSClientComponent::wash_all_bb_off() {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  for (auto *sw : App.get_switches()) {
    const std::string &oid = sw->get_object_id();
    if (oid == "bb1" || oid == "bb2" || oid == "bb3" || oid == "bb4") {
      if (sw->state) sw->turn_off();
    }
  }
#pragma GCC diagnostic pop
  this->wash_bb_expire_at_ = 0;  // v3.16.9: 清除 auto-off timer
}

// === v4.0.209 [INDOOR-LIGHT-SCENE] BEGIN ===
// wash_set_indoor_light: 開/關 R15 (indoor_light2) 或 R16 (indoor_light3)
// v4.0.210 簡化：直接 turn_on/turn_off 不判斷 state（用戶要求「觸發就開不判斷」）
//   modbus_controller_switch 對重複 turn_on/off idempotent，不會浪費 bus 資源
void WSClientComponent::wash_set_indoor_light(int n, bool on) {
  if (n != 2 && n != 3) return;
  char oid[20];
  snprintf(oid, sizeof(oid), "indoor_light%d", n);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  for (auto *sw : App.get_switches()) {
    if (sw->get_object_id() == oid) {
      if (on) sw->turn_on();
      else    sw->turn_off();
      ESP_LOGI(TAG, "[WASH] indoor_light%d %s", n, on ? "ON" : "OFF");
      char log_buf[48];
      snprintf(log_buf, sizeof(log_buf), "[WASH] indoor_light%d %s", n, on ? "ON" : "OFF");
      this->log_to_dashboard(log_buf);
      break;
    }
  }
#pragma GCC diagnostic pop
}

// wash_all_indoor_light_off: 立即關 R15 + R16 + 取消 R16 延遲排程
void WSClientComponent::wash_all_indoor_light_off() {
  this->wash_set_indoor_light(2, false);
  this->wash_set_indoor_light(3, false);
  this->indoor_light3_off_at_ = 0;  // 取消排程
}
// === v4.0.209 [INDOOR-LIGHT-SCENE] END ===

// wash_set_bb: 開啟指定泡泡（1=bb1, 2=bb2, 3=bb3, 4=bb4），0=不開
// v3.16.9: 開啟後啟動 auto-off timer (wash_bb_timeout_ms_，預設 180 秒)
void WSClientComponent::wash_set_bb(uint8_t bb) {
  if (bb == 0 || bb > 4) return;
  char oid[4];
  snprintf(oid, sizeof(oid), "bb%d", bb);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  for (auto *sw : App.get_switches()) {
    if (sw->get_object_id() == oid) {
      if (!sw->state) sw->turn_on();
      // 啟動 auto-off timer（安全保護，避免卡住導致一直開著）
      this->wash_bb_expire_at_ = millis() + this->wash_bb_timeout_ms_;
      ESP_LOGI(TAG, "wash: open bubble %s (auto-off in %u ms)",
               oid, this->wash_bb_timeout_ms_);
      break;
    }
  }
#pragma GCC diagnostic pop
}

// v3.16.48: wash_apply_r03 — 步驟進入時套用 R03 軟水切換（soft_water switch / Modbus R3）
//   r03=1 → turn_on soft_water；r03=0 → turn_off
//   每次切步驟都強制寫入（確保狀態與當前步驟 metadata 一致，即使中途手動改過）
void WSClientComponent::wash_apply_r03(uint8_t r03) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  for (auto *sw : App.get_switches()) {
    if (sw->get_object_id() == "soft_water") {
      if (r03) {
        if (!sw->state) sw->turn_on();
      } else {
        if (sw->state) sw->turn_off();
      }
      ESP_LOGI(TAG, "wash: R03 soft_water → %s", r03 ? "ON" : "off");
      break;
    }
  }
#pragma GCC diagnostic pop
}

// =========================================================================
// v3.16.0: HMI Password System
// =========================================================================

// v4.0.97: hmi_clear_pass_code_c — AA 71 矩形覆蓋密碼框
//   邏輯與 YAML hmi_clear_pass_code 完全一致（X1=68 Y1=169 X2=518 Y2=252 藍底）
//   呼叫時機：密碼錯誤（不符 unlock_pwd / debug_pwd）→ 即時清空 HMI 4 格密碼數字
void WSClientComponent::hmi_clear_pass_code_c() {
  static const unsigned char clear_rect[] = {
    0xAA, 0x71, 0x01, 0x00, 0x44, 0x00, 0xA9,
    0x02, 0x06, 0x00, 0xFC,
    0x00, 0x44, 0x00, 0xA9,
    0xCC, 0x33, 0xC3, 0x3C
  };
  uart_write_bytes((uart_port_t)0, clear_rect, sizeof(clear_rect));
  ESP_LOGI(TAG, "[HMI] password mismatch → clear pass code");
}

// v4.0.96/v4.0.97: hmi_show_unlock_success — HMI 解鎖成功時顯示「GO」於後 2 格
//   呼叫時機：do_wash_unlock(src="HMI") 內，解鎖路徑成功完成
//   邏輯：(1) AA 71 清密碼矩形（藍底，同 hmi_clear_pass_code）
//        (2) AA 98 大字型 ASCII 'G' / 'O' 各寫一格
//   座標：格 3 X=340, 格 4 X=455, Y=184（前 2 格留藍底空白）
//   v4.0.97 改：v4.0.96 用 GBK 中文「验证成功」DWIN 字庫無對應 → 亂碼，改 ASCII "GO"
void WSClientComponent::hmi_show_unlock_success() {
  // (1) AA 71 清密碼矩形（同 YAML hmi_clear_pass_code helper）
  static const unsigned char clear_rect[] = {
    0xAA, 0x71, 0x01, 0x00, 0x44, 0x00, 0xA9,
    0x02, 0x06, 0x00, 0xFC,
    0x00, 0x44, 0x00, 0xA9,
    0xCC, 0x33, 0xC3, 0x3C
  };
  uart_write_bytes((uart_port_t)0, clear_rect, sizeof(clear_rect));
  delay(2);
  // (2) AA 98 大字型 ASCII 'G' / 'O' 寫後 2 格（前 2 格留藍底空白）
  static const int positions[2] = {340, 455};
  static const unsigned char chars[2] = {'G', 'O'};
  for (int i = 0; i < 2; i++) {
    unsigned char buf[18] = {
      0xAA, 0x98,
      0x00, 0x00,        // X high/low (填下)
      0x00, 0xB8,        // Y=184
      0x00, 0x03,        // font 3
      0x07, 0xF8,        // size 大字型 64dots
      0x00,              // bg transparent
      0xFF, 0xFF,        // color white
      chars[i],          // ASCII char (1 byte)
      0xCC, 0x33, 0xC3, 0x3C     // tail
    };
    buf[2] = (positions[i] >> 8) & 0xFF;
    buf[3] = positions[i] & 0xFF;
    uart_write_bytes((uart_port_t)0, buf, sizeof(buf));
    delay(1);
  }
  ESP_LOGI(TAG, "[HMI] unlock success → display GO (last 2 cells)");
}

// hmi_show_page: 送 DWIN 切頁指令
// v3.16.13: 優先委派給 YAML hmi_show_page script（AA 52 clear + AA 70 switch + 300ms delay + Page 0 完整重繪）
//           fallback 為直接 UART 寫入（僅切頁，不重繪）
// v3.16.27: 加入 idempotent 去重 — 已經在目標 page 且 3 秒內剛處理過 → skip 重繪
//           防止 Server 斷線重連後重發 hmi_auth_result 造成的重繪風暴（100+ UART0 byte）
//           干擾 HMI 觸摸狀態導致殘留按鍵事件（如 0x2D startwasher1）被重複觸發
void WSClientComponent::hmi_show_page(int page) {
  uint32_t now = millis();
  // v3.16.27: Idempotent 檢查 — 同一 page 在 3 秒內重複呼叫 → skip 完整重繪
  //   場景：Server 每 123s 斷線重連，每次都重送 hmi_auth_result
  //   保護：避免連續的 AA 52 + AA 70 + 20+ icon 重繪序列干擾 DWIN HMI
  //   注意：hmi_show_text() 不受影響（走另一函數），文字仍會刷新
  if (page == this->hmi_current_page_ &&
      this->last_hmi_auth_page_ == page &&
      this->last_hmi_auth_at_ != 0 &&
      (now - this->last_hmi_auth_at_) < 3000) {
    ESP_LOGI(TAG, "HMI page switch → %d: skip (already on page, last %ums ago)",
             page, now - this->last_hmi_auth_at_);
    this->last_hmi_auth_at_ = now;  // 刷新時間戳，持續去重
    return;
  }

  this->last_hmi_auth_at_ = now;
  this->last_hmi_auth_page_ = page;

  if (this->hmi_show_page_script_) {
    // 委派給 YAML script — 會觸發完整 AA 52 + AA 70 + 延遲 + Page 0 圖示/文字/感測器重繪
    this->hmi_show_page_script_->execute(page);
    this->hmi_current_page_ = page;
    ESP_LOGI(TAG, "HMI page switch → %d (via YAML script, full redraw)", page);
    return;
  }
  // Fallback: YAML script 尚未綁定，只送原始切頁指令
  static const unsigned char clear_cmd[6] = {0xaa, 0x52, 0xcc, 0x33, 0xc3, 0x3c};
  uart_write_bytes((uart_port_t)0, clear_cmd, 6);
  delay(5);
  unsigned char buf[7] = {0xaa, 0x70, 0x00, 0xcc, 0x33, 0xc3, 0x3c};
  buf[2] = page & 0xff;
  uart_write_bytes((uart_port_t)0, buf, 7);
  this->hmi_current_page_ = page;
  ESP_LOGW(TAG, "HMI page switch → %d (fallback: direct UART, no redraw)", page);
}

// hmi_show_text: 送 DWIN 文字顯示指令 AA 55 <x_h> <x_l> <y_h> <y_l> <text> CC 33 C3 3C
void WSClientComponent::hmi_show_text(int x, int y, const char *text) {
  unsigned char hdr[6] = {0xaa, 0x55, 0x00, 0x00, 0x00, 0x00};
  hdr[2] = (x >> 8) & 0xff;
  hdr[3] = x & 0xff;
  hdr[4] = (y >> 8) & 0xff;
  hdr[5] = y & 0xff;
  uart_write_bytes((uart_port_t)0, hdr, 6);
  uart_write_bytes((uart_port_t)0, text, strlen(text));
  unsigned char tail[4] = {0xcc, 0x33, 0xc3, 0x3c};
  uart_write_bytes((uart_port_t)0, tail, 4);
}

// on_hmi_password_submit: HMI 數字鍵盤按 0x28 確認後呼叫
// v3.16.30: 極簡化邏輯 — 送完就結束，不等待、不 retry
//   1. 本地預設密碼比對（序號末四碼，Server 可用 set_debug_pwd CMD 覆蓋）→ 符合直接切工程頁
//   2. 不符 → 送 hmi_password EVENT 到 WS1+WS2，Server 驗證成功會回 hmi_auth_result CMD 切頁
void WSClientComponent::on_hmi_password_submit(const char *pwd, int len) {
  std::string code(pwd, len);
  ESP_LOGI(TAG, "HMI password submit: '%s'", code.c_str());

  // Lazy init 預設工程密碼 = 裝置序號末四碼
  if (this->debug_pwd_.empty()) {
    this->debug_pwd_ = (this->device_id_.size() >= 4)
                         ? this->device_id_.substr(this->device_id_.size() - 4)
                         : this->device_id_;
  }

  // v4.0.67: 優先比對 unlock_pwd_（避免巧合與工程密碼相同時走錯路）
  //   解鎖成功 → 送 wash_unlock EVENT + 走 A 表，不切頁（用戶停留在洗車頁）
  if (!this->unlock_pwd_.empty() && code == this->unlock_pwd_) {
    ESP_LOGI(TAG, "HMI unlock_pwd match → do_wash_unlock");
    this->do_wash_unlock("HMI");
    return;
  }

  // 本地工程密碼符合 → 直接切工程頁
  if (code == this->debug_pwd_) {
    ESP_LOGI(TAG, "HMI local debug auth OK → page 0");
    this->hmi_show_page(0);
    return;
  }

  // 送 Server 判斷（Server 會送 hmi_auth_result CMD 切頁，ESP32 不等待）
  // v4.0.243: 加 order_sn（流程進行中關聯訂單）
  char msg[200];
  int n;
  if (!this->order_sn_.empty()) {
    n = snprintf(msg, sizeof(msg),
      "{\"type\":\"EVENT\",\"event\":\"hmi_password\",\"code\":\"%s\",\"order_sn\":\"%s\"}",
      code.c_str(), this->order_sn_.c_str());
  } else {
    n = snprintf(msg, sizeof(msg),
      "{\"type\":\"EVENT\",\"event\":\"hmi_password\",\"code\":\"%s\"}", code.c_str());
  }
  this->send_event_raw(msg, n);
  // v4.0.97: 本地端不符任何密碼（unlock_pwd / debug_pwd）→ 立即清空 HMI 4 格
  //   不等 Server 回覆（hmi_auth_result CMD 失敗時 Server 回 success=false 也只是 alert，沒清螢幕）
  //   用戶要求：密碼輸錯就清空 HMI 4 格密碼數字
  this->hmi_clear_pass_code_c();
}

// execute_cmd: Dashboard / YAML text entity 入口（text entity `ws_cmd_input` 觸發）
// v3.16.31: 統一走 dispatch_cmd_doc(doc, "DASH") 確保與 WS1/WS2 完全一致
std::string WSClientComponent::execute_cmd(const std::string &json_cmd, const char *src_tag) {
  cJSON *doc = cJSON_Parse(json_cmd.c_str());
  if (!doc) return "ERR:invalid JSON";
  std::string result = this->dispatch_cmd_doc(doc, src_tag);
  cJSON_Delete(doc);
  return result;
}

bool WSClientComponent::send_ping() {
  if (!this->connected_ || this->sock_fd_ < 0) return false;
  // Match reference: send ping with 1-byte payload (like arduinoWebSockets sendPing)
  uint8_t ping_data = 0;
  return this->send_ws_frame(0x09, &ping_data, 1);  // opcode 0x9 = ping
}

// =========================================================================
// WebSocket Frame Read (non-blocking)
// =========================================================================

// Returns: 1=frame processed, 0=no data, -1=error/disconnect
int WSClientComponent::read_one_frame() {
  if (this->sock_fd_ < 0) return -1;

  // Read frame header (at least 2 bytes)
  uint8_t hdr[2];
  ssize_t ret = this->tls_read(hdr, 2);
  if (ret == 0) {
    // v3.16.32: TCP EOF / TLS close_notify — 對端乾淨關閉但沒送 WS close frame
    uint32_t uptime_s = (millis() - this->connected_time_) / 1000;
    ESP_LOGW(TAG, "[WS1] read_one_frame: TCP EOF / close_notify (up %us)", uptime_s);
    char rb[96];
    snprintf(rb, sizeof(rb), "TCP EOF (up %us)", uptime_s);
    this->last_disconnect_reason_ = rb;
    this->log_to_dashboard(std::string("[WS1] Server closed connection — ") + rb);
    return -1;
  }
  if (ret < 0) {
    int err = errno;
    if (err == EAGAIN || err == EWOULDBLOCK) return 0;  // No data
    uint32_t uptime_s = (millis() - this->connected_time_) / 1000;
    ESP_LOGE(TAG, "[WS1] read_one_frame: read error ret=%d errno=%d (up %us)",
             (int)ret, err, uptime_s);
    char rb[96];
    snprintf(rb, sizeof(rb), "read errno=%d (up %us)", err, uptime_s);
    this->last_disconnect_reason_ = rb;
    char log_buf[120];
    snprintf(log_buf, sizeof(log_buf), "[WS1] Read error: %s", rb);
    this->log_to_dashboard(log_buf);
    return -1;
  }
  if (ret < 2) {
    return 0;
  }

  uint8_t opcode = hdr[0] & 0x0F;
  bool masked = (hdr[1] & 0x80) != 0;
  uint64_t payload_len = hdr[1] & 0x7F;

  // Extended payload length
  if (payload_len == 126) {
    uint8_t ext[2];
    ret = this->tls_read(ext, 2);
    if (ret < 2) return -1;
    payload_len = (ext[0] << 8) | ext[1];
  } else if (payload_len == 127) {
    uint8_t ext[8];
    ret = this->tls_read(ext, 8);
    if (ret < 8) return -1;
    payload_len = 0;
    for (int i = 0; i < 8; i++) payload_len = (payload_len << 8) | ext[i];
  }

  // Read mask key if present
  uint8_t mask[4] = {0};
  if (masked) {
    ret = this->tls_read(mask, 4);
    if (ret < 4) return -1;
  }

  // Read payload
  if (payload_len > sizeof(this->read_buf_) - 1) {
    ESP_LOGW(TAG, "Frame too large: %u bytes, skipping", (unsigned)payload_len);
    size_t remaining = payload_len;
    while (remaining > 0) {
      size_t chunk = remaining > sizeof(this->read_buf_) ? sizeof(this->read_buf_) : remaining;
      ret = this->tls_read(this->read_buf_, chunk);
      if (ret <= 0) return -1;
      remaining -= ret;
    }
    return 1;
  }

  size_t total_read = 0;
  while (total_read < payload_len) {
    ret = this->tls_read(this->read_buf_ + total_read, payload_len - total_read);
    if (ret <= 0) {
      int err = errno;
      if (err == EAGAIN || err == EWOULDBLOCK) {
        delay(1);
        continue;
      }
      return -1;
    }
    total_read += ret;
  }

  // Unmask if needed
  if (masked) {
    for (size_t i = 0; i < total_read; i++) {
      this->read_buf_[i] ^= mask[i % 4];
    }
  }

  // Process by opcode
  switch (opcode) {
    case 0x01:  // Text frame
      this->read_buf_[total_read] = '\0';
      this->process_message(this->read_buf_, total_read);
      break;
    case 0x08: {  // Close frame
      // v3.16.32: 伺服端主動斷線 — 提高診斷可見性
      //   code=4001 "Superseded by newer device connection" → 同 device_id 重覆連線
      //   code=4002 "PONG timeout"                           → 伺服端心跳逾時
      //   code=4003 "Device blocked"                         → 裝置被封鎖
      uint16_t close_code = 0;
      if (total_read >= 2) {
        close_code = (this->read_buf_[0] << 8) | this->read_buf_[1];
      }
      char close_reason[128] = "";
      if (total_read > 2) {
        size_t reason_len = total_read - 2 < 120 ? total_read - 2 : 120;
        memcpy(close_reason, this->read_buf_ + 2, reason_len);
        close_reason[reason_len] = '\0';
      }
      uint32_t uptime_s = (millis() - this->connected_time_) / 1000;
      ESP_LOGW(TAG, "[WS1] *** Server CLOSE frame: code=%u reason='%s' (was up %us) ***",
               close_code, close_reason, uptime_s);
      // 儲存最後斷線原因，下次 "Connected" 時印出對照
      char reason_buf[160];
      snprintf(reason_buf, sizeof(reason_buf), "close %u %s (up %us)",
               close_code, close_reason, uptime_s);
      this->last_disconnect_reason_ = reason_buf;
      char log_buf[200];
      snprintf(log_buf, sizeof(log_buf), "[WS1] *** Server CLOSE: %u %s (up %us) ***",
               close_code, close_reason, uptime_s);
      this->log_to_dashboard(log_buf);
      this->send_ws_frame(0x08, nullptr, 0);
      return -1;
    }
    case 0x09:  // Ping from server
      this->got_server_data_ = true;  // Server is alive
      ESP_LOGI(TAG, "RX: WS Ping, sending Pong");
      if (show_ping_log_) this->log_to_dashboard("RX: WS Ping -> TX: Pong");  // v4.0.117+
      this->send_ws_frame(0x0A, (const uint8_t *)this->read_buf_, total_read);
      break;
    case 0x0A:  // Pong from server
      this->got_server_data_ = true;
      ESP_LOGD(TAG, "RX: WS Pong");
      break;
    default:
      ESP_LOGD(TAG, "Unknown opcode: 0x%02x", opcode);
      break;
  }

  return 1;  // Frame processed
}

// =========================================================================
// Sensor Publishing
// =========================================================================

bool WSClientComponent::press_button(const std::string &object_id) {
  for (auto *btn : App.get_buttons()) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    if (btn->get_object_id() == object_id) {
      btn->press();
      return true;
    }
#pragma GCC diagnostic pop
  }
  return false;
}

void WSClientComponent::publish_sensor(const std::string &object_id, const std::string &state) {
  // v4.0.257: dedup for connection status sensors (ws_connection / ws_lan_connection)
  //   原 BUG: 多處 publish_sensor("ws_connection", "Connected") 連續被呼叫 → SSE 推同值多次
  //         dashboard panel 連續顯示「Connected」造成「斷線重連」誤解（其實沒斷）
  //   雙保險：v4.0.256 dashboard JS 已加 dedup；這邊 ESP32 端再從源頭擋
  //   只對 connection-style sensor dedup，其他 sensor（譬如 ws_log 帶 timestamp）不影響
  static std::string last_ws_conn, last_ws_lan_conn;
  if (object_id == "ws_connection") {
    if (last_ws_conn == state) return;
    last_ws_conn = state;
  } else if (object_id == "ws_lan_connection") {
    if (last_ws_lan_conn == state) return;
    last_ws_lan_conn = state;
  }

  for (auto *sensor : App.get_text_sensors()) {
    if (sensor->get_name() == object_id) {
      sensor->publish_state(state);
      return;
    }
  }
  for (auto *sensor : App.get_text_sensors()) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    if (sensor->get_object_id() == object_id) {
      sensor->publish_state(state);
      return;
    }
#pragma GCC diagnostic pop
  }
}

void WSClientComponent::log_to_dashboard(const std::string &msg) {
  this->publish_sensor("ws_log", msg);
}

// =========================================================================
// UART2 Monitor — log TX/RX hex data to uart2_monitor text_sensor
// =========================================================================

void WSClientComponent::uart2_log_tx(const unsigned char *data, int len) {
  char buf[128];
  int pos = snprintf(buf, sizeof(buf), "TX>");
  for (int i = 0; i < len && pos < (int)sizeof(buf) - 4; i++) {
    pos += snprintf(buf + pos, sizeof(buf) - pos, " %02X", data[i]);
  }
  this->publish_sensor("uart2_monitor", std::string(buf));
}

void WSClientComponent::uart2_check_rx() {
  unsigned char rxbuf[64];
  int len = uart_read_bytes((uart_port_t)2, rxbuf, sizeof(rxbuf), 0);
  if (len > 0) {
    char buf[256];
    int pos = snprintf(buf, sizeof(buf), "RX<");
    for (int i = 0; i < len && pos < (int)sizeof(buf) - 4; i++) {
      pos += snprintf(buf + pos, sizeof(buf) - pos, " %02X", rxbuf[i]);
    }
    ESP_LOGI(TAG, "UART2 %s", buf);
    this->publish_sensor("uart2_monitor", std::string(buf));
  }
}

// =========================================================================
// Message Processing (same protocol as reference web_socket/esphome_client.h)
// =========================================================================

// v4.0.42: cJSON 欄位提取 helper — 減少 led_* 等多欄位 CMD 的 GetObjectItem+IsString+valuestring 模板重複
const char *WSClientComponent::json_str(cJSON *doc, const char *key, const char *def) {
  cJSON *it = cJSON_GetObjectItem(doc, key);
  return (cJSON_IsString(it) && it->valuestring) ? it->valuestring : def;
}
int WSClientComponent::json_int(cJSON *doc, const char *key, int def) {
  cJSON *it = cJSON_GetObjectItem(doc, key);
  return cJSON_IsNumber(it) ? it->valueint : def;
}

// v4.0.42 refactor: CMD 處理拆出獨立函式（dispatch + 回覆序列化），給 process_message / lan_process_message 共用
//   tag="WS1" → send_text（雲端）；tag="LAN" → lan_send_text（RPi）+ [LAN] TX log
void WSClientComponent::handle_cmd_message(cJSON *doc, const char *tag) {
  cJSON *cmd_item = cJSON_GetObjectItem(doc, "cmd");
  if (!cJSON_IsString(cmd_item)) return;
  const char *cmd_str = cmd_item->valuestring;

  // v4.0.70: RTC breadcrumb — 寫 CMD 名稱到 RTC_NOINIT，panic 時保留（tag[27] null-term）
  //   格式 "CMD:<name>" 前 27 字，超過截斷
  {
    char crumb[28];
    snprintf(crumb, sizeof(crumb), "CMD:%s", cmd_str);
    set_reboot_tag_c(crumb);
  }

  // on/ms/arg — reply echo 需要
  cJSON *on_item = cJSON_GetObjectItem(doc, "on");
  bool on = cJSON_IsNumber(on_item) ? (on_item->valueint != 0) : true;
  uint32_t ms = 0;
  cJSON *ms_item = cJSON_GetObjectItem(doc, "ms");
  if (cJSON_IsNumber(ms_item)) ms = (uint32_t)ms_item->valuedouble;
  const char *arg = json_str(doc, "arg", "");

  // 執行 CMD（三路入口統一 dispatch）
  std::string result = this->dispatch_cmd_doc(doc, tag);

  bool ok = (result.rfind("OK", 0) == 0);
  std::string result_field;
  if (ok)                                 result_field = "OK";
  else if (result.rfind("ERR:", 0) == 0)  result_field = result.substr(4);
  else                                    result_field = result;

  // getsn / vsn_list 特例：result payload 放 arg 欄位回傳
  //   getsn   → result="OK:HW12345"          → arg="HW12345"          (substr(3) 去掉 "OK:")
  //   vsn_list → result="OK {\"warn_emg\":...}" → arg="{\"warn_emg\":...}" (substr(3) 去掉 "OK ")
  // 兩者前 3 byte 都是 prefix，後面才是真正資料。
  // 不加這個特例時 result_field 只剩字面 "OK"、payload 整個丟掉（v4.0.99~112 vsn_list 隱藏 bug）。
  std::string reply_arg = arg;
  if (ok && result.size() > 3 &&
      (strcmp(cmd_str, "getsn") == 0 || strcmp(cmd_str, "vsn_list") == 0)) {
    reply_arg = result.substr(3);
  }

  cJSON *reply = cJSON_CreateObject();
  cJSON_AddStringToObject(reply, "type", ok ? "CMD_RESULT" : "CMD_ERR");
  cJSON_AddStringToObject(reply, "result", result_field.c_str());
  cJSON_AddStringToObject(reply, "cmd", cmd_str);
  cJSON_AddNumberToObject(reply, "on", on ? 1 : 0);
  cJSON_AddNumberToObject(reply, "ms", ms);
  cJSON_AddStringToObject(reply, "arg", reply_arg.c_str());
  char *out = cJSON_PrintUnformatted(reply);
  if (out) {
    if (strcmp(tag, "LAN") == 0) {
      this->lan_send_text(out);
      std::string log_tx = std::string("[LAN] TX: ") + out;
      if (log_tx.size() > 220) log_tx = log_tx.substr(0, 220) + "...";
      this->log_to_dashboard(log_tx);
    } else {
      this->send_text(out);  // send_text 會 publish 到 ws_last_sent
    }
    cJSON_free(out);
  }
  cJSON_Delete(reply);
}

void WSClientComponent::process_message(const char *data, int len) {
  ESP_LOGI(TAG, "Received: %.*s", len, data);
  // v4.0.70: breadcrumb — 進 process_message（WS1 雲端收到訊息）
  set_reboot_tag_c("WS1_MSG");

  std::string msg(data, len);
  this->publish_sensor("ws_last_recv", msg);

  cJSON *doc = cJSON_Parse(data);
  if (!doc) {
    ESP_LOGE(TAG, "JSON parse failed: %s", cJSON_GetErrorPtr());
    return;
  }

  const char *msg_type = json_str(doc, "type", "DATA");

  // Any message from server = connection alive
  this->got_server_data_ = true;

  // 1. PING -> PONG (application-level)
  if (strcmp(msg_type, "PING") == 0) {
    cJSON *pong = cJSON_CreateObject();
    cJSON_AddStringToObject(pong, "type", "PONG");
    char *out = cJSON_PrintUnformatted(pong);
    this->send_text(out);
    this->log_to_dashboard("TX: PONG");
    cJSON_free(out);
    cJSON_Delete(pong);
    cJSON_Delete(doc);
    return;
  }

  // 2. TIME_LIMITED expiration check
  cJSON *expires_item = cJSON_GetObjectItem(doc, "expires_at");
  if (cJSON_IsNumber(expires_item)) {
    time_t now_ts;
    time(&now_ts);
    if (now_ts > 1600000000 && (uint32_t)now_ts > (uint32_t)expires_item->valuedouble) {
      ESP_LOGW(TAG, "Message expired, dropped.");
      cJSON_Delete(doc);
      return;
    }
  }

  // 3. ACK for GUARANTEED messages（有 msg_id 就回 ACK）
  const char *msg_id = json_str(doc, "msg_id", nullptr);
  if (msg_id) this->send_ack(msg_id);

  // 4. CMD handling（三路入口統一走 handle_cmd_message + dispatch_cmd_doc）
  if (strcmp(msg_type, "CMD") == 0) {
    this->handle_cmd_message(doc, "WS1");
  }

  cJSON_Delete(doc);
}

// =========================================================================
// CMD Control — object_id-based switch/fan/light/button routing + auto-off timer
// Format: {"type":"CMD","cmd":"<object_id>","on":1,"ms":5000,"arg":""}
//   cmd: ESPHome object_id（e.g. spare_io, cooling_fan, web_server, check_ota）
//        或特殊字串：startwasher1~6 / gowasher / resetwasher / hardresetwasher / sparewasher
//                    go_enable / emergency_enable / reboot / getsn / all_off / status
//   on:  1=on, 0=off
//   ms:  auto-off timer in milliseconds (0=no timer)
//   arg: optional argument (used by go_enable, light effects)
// =========================================================================

// ─────────────────────────────────────────────────────────────────────
// v3.16.31: dispatch_cmd_doc — 三個 CMD 入口點（WS1/WS2/Dashboard）共用的路由函式
// 保證三個來源的 CMD 行為、訊息格式、回覆內容完全一致（只差 log tag + 送回通道）
//
// 支援的 CMD（完整列表 v3.16.60）：
//   (1) led_switch / led_update / led_brightness / led_volume — Colorlight LED 螢幕 4 APIs
//   (2) wash_init         — 洗車階段初始化（委派 handle_wash_init_doc）
//   (3) hmi_auth_result   — Server 密碼驗證結果（切頁 + 顯示訊息）
//   (4) set_debug_pwd     — 覆蓋預設工程密碼
//   (5) 其他（entity/系統/washer）— 委派 handle_cmd(cmd, on, ms, arg, source)
//
// v3.16.59 移除：climate_*（風扇改為自動控制）、status_light_* entity（燈帶已拔）
// v3.16.60: 恢復 led_update/brightness/volume CMD dispatch —
//   wash_step 不再引用這些欄位，但 CMD API 本身仍可被 Server/Dashboard 直接呼叫
//
// source: "WS1" | "LAN" | "DASH" — 透傳到 handle_cmd → UART2 TX log 標記
// 回傳：result 字串（"OK:..." 或 "ERR:..."）；doc 由 caller delete
// ─────────────────────────────────────────────────────────────────────
std::string WSClientComponent::dispatch_cmd_doc(cJSON *doc, const char *source) {
  cJSON *cmd_item = cJSON_GetObjectItem(doc, "cmd");
  if (!cJSON_IsString(cmd_item) || !cmd_item->valuestring) {
    return "ERR:missing cmd";
  }
  const char *cmd_str = cmd_item->valuestring;

  // ── (1) LED 螢幕 4 APIs — led_switch / led_update / led_brightness / led_volume ──
  //   統一編成 pipe-delimited payload → publish 到 led_cmd text_sensor，
  //   YAML led_cmd on_value 解析後呼叫對應 script
  //   v3.16.60: wash_step 不再自動觸發 led_update/brightness/volume（簡化為 led_switch only），
  //            但這些 CMD 本身仍可被手動呼叫
  // v4.0.42 refactor: 欄位提取走 json_str/json_int helper，消除 15 處 GetObjectItem+IsString+valuestring 模板
  if (strcmp(cmd_str, "led_switch") == 0 ||
      strcmp(cmd_str, "led_update") == 0 ||
      strcmp(cmd_str, "led_brightness") == 0 ||
      strcmp(cmd_str, "led_volume") == 0) {
    std::string payload;
    std::string result;
    if (strcmp(cmd_str, "led_switch") == 0) {
      // {"cmd":"led_switch","name":"foo.vsn","source":"lan"}
      const char *nm  = json_str(doc, "name", "");
      const char *src = json_str(doc, "source", "lan");
      if (!*nm) {
        result = "ERR:missing name";
      } else {
        payload = std::string("switch|") + src + "|" + nm;
        // v3.16.59: 紀錄最近一次切換的節目（STATUS 顯示用）
        this->last_led_name_ = nm;
        this->last_led_source_ = src;
        ESP_LOGI(TAG, "[%s] led_switch → name='%s' source='%s'", source, nm, src);
        result = std::string("OK:led_switch ") + nm;
        if (this->led_switch_script_) this->led_switch_script_->execute(std::string(nm), std::string(src));
        // === v4.0.202 [RPI-EVENT-LED] BEGIN ===
        // 動作後通知 RPi（容錯設計，不替代執行）
        {
          std::string p = std::string("\"name\":\"") + nm + "\",\"source\":\"" + src + "\"";
          std::string l = std::string("name=") + nm + " source=" + src;
          this->send_rpi_event("led_switch", p, l);
        }
        // === v4.0.202 [RPI-EVENT-LED] END ===
      }
    } else if (strcmp(cmd_str, "led_update") == 0) {
      // {"cmd":"led_update","name":"foo.vsn","text":"...","color":"#FFFFFF","x":0,"y":0,"source":"internet"}
      const char *nm  = json_str(doc, "name", "");
      const char *txt = json_str(doc, "text", "");
      const char *col = json_str(doc, "color", "#FFFFFF");
      const char *src = json_str(doc, "source", "internet");
      int xv = json_int(doc, "x", 0);
      int yv = json_int(doc, "y", 0);
      if (!*nm) {
        result = "ERR:missing name";
      } else {
        // payload 格式：update|{src}|{x}|{y}|{color}|{name}|{text}
        // 注意：text 放最後，允許內含 | 字元
        char hdr[100];
        snprintf(hdr, sizeof(hdr), "update|%s|%d|%d|%s|", src, xv, yv, col);
        payload = std::string(hdr) + nm + "|" + txt;
        this->last_led_name_ = nm;
        this->last_led_source_ = src;
        ESP_LOGI(TAG, "[%s] led_update → name='%s' text='%s' color='%s' x=%d y=%d source='%s'",
                 source, nm, txt, col, xv, yv, src);
        result = std::string("OK:led_update ") + nm;
        if (this->led_update_script_)
          this->led_update_script_->execute(std::string(nm), std::string(txt), std::string(col), xv, yv, std::string(src));
        // === v4.0.202 [RPI-EVENT-LED] BEGIN ===
        {
          char pbuf[384];
          snprintf(pbuf, sizeof(pbuf),
                   "\"name\":\"%s\",\"text\":\"%s\",\"color\":\"%s\",\"x\":%d,\"y\":%d,\"source\":\"%s\"",
                   nm, txt, col, xv, yv, src);
          char lbuf[256];
          snprintf(lbuf, sizeof(lbuf), "name=%s text=%s color=%s x=%d y=%d source=%s",
                   nm, txt, col, xv, yv, src);
          this->send_rpi_event("led_update", std::string(pbuf), std::string(lbuf));
        }
        // === v4.0.202 [RPI-EVENT-LED] END ===
      }
    } else if (strcmp(cmd_str, "led_brightness") == 0) {
      int v = json_int(doc, "value", 255);
      if (v < 0) v = 0; if (v > 255) v = 255;
      payload = std::string("brightness|") + std::to_string(v);
      ESP_LOGI(TAG, "[%s] led_brightness=%d", source, v);
      result = std::string("OK:led_brightness ") + std::to_string(v);
      if (this->led_brightness_script_) this->led_brightness_script_->execute(v);
      // === v4.0.202 [RPI-EVENT-LED] BEGIN ===
      {
        std::string p = std::string("\"value\":") + std::to_string(v);
        std::string l = std::string("value=") + std::to_string(v);
        this->send_rpi_event("led_brightness", p, l);
      }
      // === v4.0.202 [RPI-EVENT-LED] END ===
    } else if (strcmp(cmd_str, "led_volume") == 0) {
      int v = json_int(doc, "value", 10);
      if (v < 0) v = 0; if (v > 15) v = 15;
      payload = std::string("volume|") + std::to_string(v);
      ESP_LOGI(TAG, "[%s] led_volume=%d", source, v);
      result = std::string("OK:led_volume ") + std::to_string(v);
      if (this->led_volume_script_) this->led_volume_script_->execute(v);
      // === v4.0.202 [RPI-EVENT-LED] BEGIN ===
      {
        std::string p = std::string("\"value\":") + std::to_string(v);
        std::string l = std::string("value=") + std::to_string(v);
        this->send_rpi_event("led_volume", p, l);
      }
      // === v4.0.202 [RPI-EVENT-LED] END ===
    }
    // === v4.0.279 [CL-AUTO-RECOVERY] BEGIN ===
    // CMD cl_reset — 遠端手動觸發 Colorlight soft recovery
    //   清 cooldown / 清 fail_count / bump last_done_ms_ → 下一筆 cl.put 有乾淨環境
    //   不殺 worker（避免 lwIP mutex deadlock）；若 worker 真卡死，自動 30s watchdog 也會 fire
    //   payload: {"type":"CMD","cmd":"cl_reset"}
    //   reply:   {"type":"CMD_RESULT","result":"OK","cmd":"cl_reset",...}
    else if (strcmp(cmd_str, "cl_reset") == 0) {
      esphome::colorlight::ColorlightClient::soft_reset("cmd");
      ESP_LOGI(TAG, "[%s] cl_reset triggered", source);
      result = "OK:cl_reset";
    }
    // === v4.0.279 [CL-AUTO-RECOVERY] END ===
    // === v4.0.203 [LED-CMD-DEDUP] BEGIN ===
    // 用戶截圖反映：LED CMD 在 ws_log 重複出現 2 次（API event + [CL] PUT + LED ... → async）
    // 根因：cpp 已 execute script (line 3929 等) + publish_sensor("led_cmd") → yaml on_value 又 execute (line 2020)
    // 修法：mark-out cpp publish_sensor，保留 yaml led_cmd on_value 不動
    // Rollback：取消下方 #if 0 區塊註解
    // v4.0.02 原註解保留（avoiding publish_sensor → text_sensor on_value 中間層時序異常）
    //   但本版發現該路徑會「同時觸發兩條 LED 動作」，故停用 publish_sensor
#if 0
    if (!payload.empty()) {
      this->publish_sensor("led_cmd", std::to_string(millis()) + "|" + payload);
    }
#else
    (void)payload;  // suppress unused-variable warning
#endif
    // === v4.0.203 [LED-CMD-DEDUP] END ===
    return result;
  }

  // ── (2) wash_init — 委派共用 helper ──
  if (strcmp(cmd_str, "wash_init") == 0) {
    return this->handle_wash_init_doc(doc, source);
  }

  // ── (2.5) v4.0.22: wash_reset — Dashboard S4 測試按鈕 / 手動清空洗車狀態 ──
  //   {"cmd":"wash_reset"}
  //   等同於 wash_reset_public()（關 bb1~bb4、清 steps、wash_active_=false、發 wash_stage EVENT、清 NVS）
  if (strcmp(cmd_str, "wash_reset") == 0) {
    ESP_LOGI(TAG, "[%s] wash_reset", source);
    // v4.0.219: wash_reset_public 內部已 fire wash_reset EVENT（取代舊空 wash_stage(idle)）
    this->wash_reset_public(source);
    return "OK:wash_reset";
  }

  // ── (2.6) v4.0.27: test_flow1 / test_flow2 — 遠端觸發等同 HMI 0x17/0x18 的完整流程 ──
  //   {"cmd":"test_flow1"}  → wash_reset + WASH5_JSON wash_init + UART2 startwasher5
  //   {"cmd":"test_flow2"}  → wash_reset + WASH6_JSON wash_init + UART2 startwasher6
  //   預設腳本與 iot-washer-base.yaml 的 HMI 按鈕 0x17/0x18 分支保持一致（改動時兩處都要改）
  if (strcmp(cmd_str, "test_flow1") == 0 || strcmp(cmd_str, "test_flow2") == 0) {
    bool is_flow2 = (strcmp(cmd_str, "test_flow2") == 0);
    ESP_LOGI(TAG, "[%s] %s → wash_reset + wash_init + startwasher%d",
             source, cmd_str, is_flow2 ? 6 : 5);
    // (1) 清空舊狀態（bb1~bb4 off、steps 清零、NVS 清）
    // v4.0.219: 帶 source — wash_reset EVENT 會帶這個 caller tag
    this->wash_reset_public(source);
    // (2) 派送預設 wash_init 腳本（與 YAML WASH5_JSON/WASH6_JSON 同步）
    // v4.0.116: 同步 YAML 當前版本（含 Order + unlock_pwd + 對齊 vsn_map key 格式）
    //   v4.0.27 以來 cpp 端版本沒跟 YAML 同步 → test_flow1/2 不會進 lock 模式（缺 unlock_pwd）
    // v4.0.221: 跟 YAML iot-washer-base.yaml WASH5/6_JSON 同步 (Order 5 欄位 + step.led_eff)
    // v4.0.275: 同步 YAML — Order 完整 5 欄位 + 新 auto_sec (Intro 90/99 / AIScanning 300)
    //   cpp 端 test_flow1/2 CMD 用固定 order_sn (yaml HMI/Dashboard 動態 SNTP yymmddhhiiss_T1/T2)
    static const char *WASH5_JSON = R"JSON({"type":"CMD","cmd":"wash_init","Order":{"order_sn":"TEST_T1","order_cost":"168","order_product":"168洗車方案|軟水:10,擦乾10分:30","order_name":"卡最多","order_msg":"請掃碼啟動洗車機或於控制螢幕上輸入啟動碼!! 下一個可預約的時間在3:30 約20分鐘後"},"unlock_pwd":"1234","total":6,"steps":[{"id":"501","bb":0,"R03":0,"auto_sec":90,"name":"Intro","led_name":"01","led_eff":"a1,b1,c1,"},{"id":"502","bb":0,"R03":0,"auto_sec":3,"name":"Pre-Wash","led_name":"03","led_eff":"a3,,c3,d3"},{"id":"503","bb":1,"R03":0,"auto_sec":3,"name":"Foam","led_name":"04","led_eff":",b4,c4,d4"},{"id":"504","bb":0,"R03":0,"auto_sec":300,"name":"AIScanning","led_name":"05","led_eff":"a5,b5,c5,d5"},{"id":"505","bb":0,"R03":1,"auto_sec":3,"name":"Final_Rinse","led_name":"06","led_eff":"a6,,c6,d6"},{"id":"506","bb":0,"R03":0,"auto_sec":70,"name":"Complete","led_name":"09","led_eff":"a9,b9,c9,d9"}]})JSON";
    static const char *WASH6_JSON = R"JSON({"type":"CMD","cmd":"wash_init","Order":{"order_sn":"TEST_T2","order_cost":"268","order_product":"268進階洗車|預洗:20,軟水:10,擦乾10分:30","order_name":"卡有錢","order_msg":"請掃碼啟動洗車機或於控制螢幕上輸入啟動碼!! 下一個可預約的時間在6:30 約30分鐘後"},"unlock_pwd":"8888","total":9,"steps":[{"id":"601","bb":0,"R03":0,"auto_sec":99,"name":"Intro","led_name":"01","led_eff":"a1,b1,c1,"},{"id":"602","bb":2,"R03":0,"auto_sec":5,"name":"Pre-Soak","led_name":"02","led_eff":"a2,b2,,d2"},{"id":"603","bb":0,"R03":0,"auto_sec":3,"name":"Pre-Wash","led_name":"03","led_eff":"a3,,c3,d3"},{"id":"604","bb":1,"R03":0,"auto_sec":3,"name":"Foam","led_name":"04","led_eff":",b4,c4,d4"},{"id":"605","bb":0,"R03":0,"auto_sec":300,"name":"AIScanning","led_name":"05","led_eff":"a5,b5,c5,d5"},{"id":"606","bb":0,"R03":0,"auto_sec":3,"name":"Rinse","led_name":"06","led_eff":"a6,,c6,d6"},{"id":"607","bb":3,"R03":0,"auto_sec":3,"name":"Coating","led_name":"07","led_eff":"a7,b7,,d7"},{"id":"608","bb":0,"R03":1,"auto_sec":3,"name":"Final_Rinse","led_name":"08","led_eff":"a8,b8,c8,"},{"id":"609","bb":0,"R03":0,"auto_sec":10,"name":"Complete","led_name":"09","led_eff":"a9,b9,c9,d9"}]})JSON";
    this->execute_cmd(std::string(is_flow2 ? WASH6_JSON : WASH5_JSON), source);
    // (3) 送 UART2 startwasher5/6（非阻塞 script，5 次 x 100ms）
    if (this->washer_script_) {
      int idx = is_flow2 ? 6 : 5;
      std::string src_str = source;
      std::string cmd_name = is_flow2 ? "startwasher6" : "startwasher5";
      this->washer_script_->execute(idx, src_str, cmd_name);
    }
    // v4.0.217 [WASH-EVENTS] 送 test_flow1/test_flow2 EVENT（讓 Server 知道是測試觸發）
    this->send_wash_event_simple(is_flow2 ? "test_flow2" : "test_flow1", source);
    return is_flow2 ? "OK:test_flow2" : "OK:test_flow1";
  }

  // ── (2.7) v4.0.28: wash_start / wash_stop — 洗車流程暫停 / 恢復（純狀態切換）──
  //   {"cmd":"wash_stop"}  → wash_paused_=true （EMG 安全：凍結 auto_sec 倒數）
  //   {"cmd":"wash_start"} → wash_paused_=false（平移 blue_enter_at_ 讓倒數延續）
  //   不送 UART2 gowasher；不改 steps；只切狀態 + 送 wash_stage EVENT + STATUS
  //   EMG 按下會自動呼叫 set_wash_paused(true, "EMG")，用戶再手動 wash_start 才恢復
  if (strcmp(cmd_str, "wash_stop") == 0) {
    // v4.0.227: wash_stop EVENT 已從 set_wash_paused 內部自動 fire (source 帶 "CMD:wash_stop")
    this->set_wash_paused(true, "CMD:wash_stop");
    return "OK:wash_stop";
  }
  if (strcmp(cmd_str, "wash_start") == 0) {
    // v4.0.227: wash_start EVENT 同上 (set_wash_paused 內部自動 fire)
    this->set_wash_paused(false, "CMD:wash_start");
    return "OK:wash_start";
  }

  // ── v4.0.67: wash_unlock — 解鎖店鋪（配對 wash_init unlock_pwd）──
  //   {"cmd":"wash_unlock"} → 清 unlock_pwd_ + NVS；依當前燈色套 A 表進流程
  //   若非鎖定狀態：回 OK:wash_unlock not_locked（non-op，不報錯）
  if (strcmp(cmd_str, "wash_unlock") == 0) {
    if (this->unlock_pwd_.empty()) {
      return "OK:wash_unlock not_locked";
    }
    this->do_wash_unlock(source);
    return "OK:wash_unlock";
  }

  // ── (3) hmi_auth_result — Server 密碼驗證結果 ──
  //   {"cmd":"hmi_auth_result","success":true,"page":0,"msg":"工程模式"}
  if (strcmp(cmd_str, "hmi_auth_result") == 0) {
    cJSON *success_it = cJSON_GetObjectItem(doc, "success");
    cJSON *page_it = cJSON_GetObjectItem(doc, "page");
    cJSON *msg_it = cJSON_GetObjectItem(doc, "msg");
    bool success = cJSON_IsTrue(success_it);
    const char *msg_text = (cJSON_IsString(msg_it) && msg_it->valuestring) ? msg_it->valuestring : "";
    if (cJSON_IsNumber(page_it)) {
      this->hmi_show_page(page_it->valueint);
      ESP_LOGI(TAG, "[%s] hmi_auth_result: page=%d msg='%s'", source, page_it->valueint, msg_text);
    }
    if (msg_text[0]) {
      this->hmi_show_text(224, 176, msg_text);
    }
    if (success) {
      ESP_LOGI(TAG, "[%s] hmi_auth_result: success msg='%s'", source, msg_text);
    } else {
      ESP_LOGW(TAG, "[%s] hmi_auth_result: failed msg='%s'", source, msg_text);
    }
    return "OK:hmi_auth_result";
  }

  // ── (4) set_debug_pwd — 覆蓋預設工程密碼 ──
  //   {"cmd":"set_debug_pwd","pwd":"5678"}
  if (strcmp(cmd_str, "set_debug_pwd") == 0) {
    cJSON *pwd_it = cJSON_GetObjectItem(doc, "pwd");
    const char *pwd = (cJSON_IsString(pwd_it) && pwd_it->valuestring) ? pwd_it->valuestring : "";
    if (pwd[0]) {
      this->debug_pwd_ = pwd;
      ESP_LOGI(TAG, "[%s] set_debug_pwd: '%s' (override default)", source, pwd);
      return "OK:set_debug_pwd";
    }
    return "ERR:set_debug_pwd empty";
  }

  // ── (5) vsn_list — 查詢當前 VSN map（key → vsn_name 對應）──
  //   {"cmd":"vsn_list"} → CMD_RESULT result 內含完整 map JSON
  // v4.0.99: 部署檢查用（看哪些 key 已設定/還沒設定）
  if (strcmp(cmd_str, "vsn_list") == 0) {
    std::string j = "{";
    bool first = true;
    for (const auto &kv : this->vsn_map_get()) {
      if (!first) j += ",";
      first = false;
      j += "\"";
      j += kv.first;
      j += "\":\"";
      j += kv.second;
      j += "\"";
    }
    j += "}";
    ESP_LOGI(TAG, "[%s] vsn_list: %u keys", source, (unsigned) this->vsn_map_get().size());
    return std::string("OK ") + j;
  }

  // ── (5b) vsn_bulk_update — 一次更新整個 map + NVS 持久化 ──
  //   {"cmd":"vsn_bulk_update","map":{"warn_emg":"...","emg":"...","01":"...",...}}
  // v4.0.104: 部署用 — 一個 CMD 推全部 keys（取代 14 次 vsn_update round-trip）
  if (strcmp(cmd_str, "vsn_bulk_update") == 0) {
    cJSON *map_it = cJSON_GetObjectItem(doc, "map");
    if (!cJSON_IsObject(map_it)) {
      return "ERR:vsn_bulk_update missing map object";
    }
    int updated = 0;
    int skipped = 0;
    cJSON *kv = nullptr;
    cJSON_ArrayForEach(kv, map_it) {
      if (!kv->string || !cJSON_IsString(kv)) { skipped++; continue; }
      std::string key(kv->string);
      std::string vsn(kv->valuestring ? kv->valuestring : "");
      this->vsn_set(key, vsn);
      updated++;
    }
    ESP_LOGI(TAG, "[%s] vsn_bulk_update: %d updated, %d skipped", source, updated, skipped);
    char buf[80];
    snprintf(buf, sizeof(buf), "OK:vsn_bulk_update %d/%d", updated, updated + skipped);
    return std::string(buf);
  }

  // ── (6) vsn_update — 更新單一 VSN key + NVS 持久化 ──
  //   {"cmd":"vsn_update","key":"emg","vsn":"huiflow_xxx.vsn"}
  // v4.0.99: 部署用 — Server 推一次 vsn_update 後永久生效（重開機從 NVS 載入）
  if (strcmp(cmd_str, "vsn_update") == 0) {
    cJSON *key_it = cJSON_GetObjectItem(doc, "key");
    cJSON *vsn_it = cJSON_GetObjectItem(doc, "vsn");
    if (!cJSON_IsString(key_it) || !key_it->valuestring) {
      return "ERR:vsn_update missing key";
    }
    if (!cJSON_IsString(vsn_it)) {
      return "ERR:vsn_update missing vsn";
    }
    std::string key(key_it->valuestring);
    std::string vsn(vsn_it->valuestring ? vsn_it->valuestring : "");
    this->vsn_set(key, vsn);
    ESP_LOGI(TAG, "[%s] vsn_update: key='%s' vsn='%s'", source, key.c_str(), vsn.c_str());
    return std::string("OK:vsn_update ") + key + "=" + vsn;
  }

  // ── (6b) led_dualcam — RPi 雙鏡頭 LED API 觸發（v4.0.111）──
  //   {"cmd":"led_dualcam","order_sn":"...","order_name":"...","order_msg":"..."}
  //   無 sn/name/msg 時用當前 wash 流程的訂單值（lock 模式有效）
  if (strcmp(cmd_str, "led_dualcam") == 0) {
    cJSON *sn_it   = cJSON_GetObjectItem(doc, "order_sn");
    cJSON *name_it = cJSON_GetObjectItem(doc, "order_name");
    cJSON *msg_it  = cJSON_GetObjectItem(doc, "order_msg");
    std::string sn   = (cJSON_IsString(sn_it)   && sn_it->valuestring)   ? sn_it->valuestring   : this->order_sn_;
    std::string name = (cJSON_IsString(name_it) && name_it->valuestring) ? name_it->valuestring : this->order_name_;
    std::string msg  = (cJSON_IsString(msg_it)  && msg_it->valuestring)  ? msg_it->valuestring  : this->order_msg_;
    if (this->led_dualcam_script_) {
      this->led_dualcam_script_->execute(sn, name, msg);
      ESP_LOGI(TAG, "[%s] led_dualcam: sn='%s' name='%s'", source, sn.c_str(), name.c_str());
      // === v4.0.202 [RPI-EVENT-LED] BEGIN ===
      // led_dualcam HTTP PUT 保留（不砍）；動作後 fire RPi event
      {
        char pbuf[384];
        snprintf(pbuf, sizeof(pbuf),
                 "\"sn\":\"%s\",\"top\":\"%s\",\"bottom\":\"%s\"",
                 sn.c_str(), name.c_str(), msg.c_str());
        char lbuf[256];
        snprintf(lbuf, sizeof(lbuf), "sn=%s top=%s bottom=%s",
                 sn.c_str(), name.c_str(), msg.c_str());
        this->send_rpi_event("led_dualcam", std::string(pbuf), std::string(lbuf));
      }
      // === v4.0.202 [RPI-EVENT-LED] END ===
      return "OK:led_dualcam";
    }
    return "ERR:led_dualcam script not injected";
  }

  // === v4.0.214 [WS1-REDIRECT] BEGIN ===
  // CMD ws1_server — 動態切 WS1 server (host/port/ssl)，用 WS2 console 即可重導
  // 不持久化（重開機從 YAML 還原）— 純 dev/test 用
  // 格式：{"type":"CMD","cmd":"ws1_server","host":"192.168.1.99","port":8443,"ssl":true}
  // Rollback：刪除整個 BEGIN/END 區塊
  if (strcmp(cmd_str, "ws1_server") == 0) {
    const char *new_host = json_str(doc, "host", "");
    if (!*new_host) {
      return "ERR:ws1_server missing host (string)";
    }
    int new_port = json_int(doc, "port", this->use_ssl_ ? 443 : 80);
    cJSON *ssl_item = cJSON_GetObjectItemCaseSensitive(doc, "ssl");
    bool new_ssl = cJSON_IsBool(ssl_item) ? cJSON_IsTrue(ssl_item) : this->use_ssl_;
    if (new_port <= 0 || new_port > 65535) {
      return "ERR:ws1_server invalid port";
    }
    char log[180];
    snprintf(log, sizeof(log), "[WS1-REDIRECT] %s:%d ssl=%d -> %s:%d ssl=%d",
             this->server_.c_str(), this->port_, this->use_ssl_ ? 1 : 0,
             new_host, new_port, new_ssl ? 1 : 0);
    this->log_to_dashboard(log);
    ESP_LOGW(TAG, "%s", log);
    this->server_ = new_host;
    this->port_ = new_port;
    this->use_ssl_ = new_ssl;
    if (this->connected_) {
      this->connected_ = false;
      this->do_disconnect();
    }
    this->enabled_ = true;
    this->reconnect_attempts_ = 0;
    this->reconnect_interval_ = 1000;
    this->last_connect_attempt_ = 0;
    this->connect_state_ = CS_IDLE;
    this->manual_connect();
    return std::string("OK:ws1_server -> ") + new_host + ":" + std::to_string(new_port);
  }
  // === v4.0.214 [WS1-REDIRECT] END ===

  // === v4.0.237 [WS2-REDIRECT] BEGIN ===
  // CMD ws2_server — 動態切 WS2 LAN server (host/port/ssl)
  // 不持久化（重開機從 YAML 還原，預設 = 192.168.1.10）— 純 dev/test 用
  // 格式：{"type":"CMD","cmd":"ws2_server","host":"192.168.1.99","port":8090,"ssl":false}
  // Rollback：刪除整個 BEGIN/END 區塊
  if (strcmp(cmd_str, "ws2_server") == 0) {
    const char *new_host = json_str(doc, "host", "");
    if (!*new_host) {
      return "ERR:ws2_server missing host (string)";
    }
    int new_port = json_int(doc, "port", this->lan_use_ssl_ ? 443 : 80);
    cJSON *ssl_item = cJSON_GetObjectItemCaseSensitive(doc, "ssl");
    bool new_ssl = cJSON_IsBool(ssl_item) ? cJSON_IsTrue(ssl_item) : this->lan_use_ssl_;
    if (new_port <= 0 || new_port > 65535) {
      return "ERR:ws2_server invalid port";
    }
    char log[180];
    snprintf(log, sizeof(log), "[WS2-REDIRECT] %s:%d ssl=%d -> %s:%d ssl=%d",
             this->lan_server_.c_str(), this->lan_port_, this->lan_use_ssl_ ? 1 : 0,
             new_host, new_port, new_ssl ? 1 : 0);
    this->log_to_dashboard(log);
    ESP_LOGW(LAN_TAG, "%s", log);
    this->lan_server_ = new_host;
    this->lan_port_ = new_port;
    this->lan_use_ssl_ = new_ssl;
    // 更新 dashboard text_sensor 顯示
    char srv_buf[64];
    snprintf(srv_buf, sizeof(srv_buf), "%s:%d", new_host, new_port);
    this->publish_sensor("ws_lan_server_info", srv_buf);
    // 斷既有連線（若有）→ 主迴圈自動重連到新 host
    if (this->lan_connected_) {
      this->lan_connected_ = false;
      this->lan_do_disconnect();
    }
    this->lan_enabled_ = true;
    this->lan_reconnect_attempts_ = 0;
    this->lan_reconnect_interval_ = 1000;
    this->lan_last_connect_attempt_ = 0;
    this->lan_connect_state_ = CS_IDLE;
    return std::string("OK:ws2_server -> ") + new_host + ":" + std::to_string(new_port);
  }
  // === v4.0.237 [WS2-REDIRECT] END ===

  // ── (7) Fallback — entity / 系統 / washer CMD → handle_cmd ──
  cJSON *on_item = cJSON_GetObjectItem(doc, "on");
  bool on = cJSON_IsNumber(on_item) ? (on_item->valueint != 0) : true;
  uint32_t ms = 0;
  cJSON *ms_item = cJSON_GetObjectItem(doc, "ms");
  if (cJSON_IsNumber(ms_item)) ms = (uint32_t)ms_item->valuedouble;
  cJSON *arg_item = cJSON_GetObjectItem(doc, "arg");
  const char *arg = (cJSON_IsString(arg_item) && arg_item->valuestring) ? arg_item->valuestring : "";
  return this->handle_cmd(cmd_str, on, ms, arg, source);
}

// ─────────────────────────────────────────────────────────────────────
// handle_cmd: Central command router for all device control
// Called by: cloud WS CMD (source="WS1"), LAN WS CMD (source="LAN"),
//            Dashboard CMD (source="DASH"), go_enable / emergency_enable triggers
// v3.16.31: source 參數用於 UART2 washer TX log 分流（"WS1"/"LAN"/"DASH"/"GO"/"EMG"/"HMI"/"CMD"）
// Returns: "OK:..." on success, "ERR:..." on failure
// ─────────────────────────────────────────────────────────────────────
std::string WSClientComponent::handle_cmd(const char *cmd, bool on, uint32_t ms, const char *arg, const char *source) {
  // ═══════════════════════════════════════════════════════════════
  // (1) 特殊字串指令（非 ESPHome entity）優先處理
  // ═══════════════════════════════════════════════════════════════

  // ── UART2 洗車機指令（非阻塞：透過 YAML washer_send_cmd script 發送 5 次，間隔 100ms）──
  // 改用 washer_script_->execute() 委託 YAML script，避免阻塞 loop() 500ms
  if (strncmp(cmd, "startwasher", 11) == 0 || strcmp(cmd, "gowasher") == 0 ||
      strcmp(cmd, "resetwasher") == 0 || strcmp(cmd, "hardresetwasher") == 0 ||
      strcmp(cmd, "sparewasher") == 0) {
    static const struct { const char *name; int idx; } wcmds[] = {
      {"startwasher1", 1}, {"startwasher2", 2}, {"startwasher3", 3},
      {"startwasher4", 4}, {"startwasher5", 5}, {"startwasher6", 6},
      {"gowasher", 7}, {"resetwasher", 8}, {"hardresetwasher", 9}, {"sparewasher", 10}
    };
    int idx = -1;
    for (int i = 0; i < 10; i++) {
      if (strcmp(cmd, wcmds[i].name) == 0) { idx = wcmds[i].idx; break; }
    }
    if (idx < 1 || idx > 10) return std::string("ERR:invalid washer cmd ") + cmd;
    // 透過 YAML washer_send_cmd script 非阻塞發送（5 次 x 100ms delay）
    // v3.16.31: source 來自 handle_cmd 參數（WS1/LAN/DASH/GO/EMG/HMI/CMD）
    if (this->washer_script_) {
      std::string src_str = source;  // "WS1"/"LAN"/"DASH" 等，由 dispatch_cmd_doc 傳入
      std::string cmd_name = cmd;
      this->washer_script_->execute(idx, src_str, cmd_name);
      ESP_LOGI(TAG, "[%s] CMD washer: %s idx=%d (via script, non-blocking)", source, cmd, idx);
    } else {
      // Fallback: script 尚未設定，用阻塞方式直接發送
      ESP_LOGW(TAG, "CMD washer: %s idx=%d (blocking fallback, no script)", cmd, idx);
      const unsigned char *frame = get_washer_frame(idx);
      if (frame) {
        for (int tx = 0; tx < 5; tx++) {
          if (tx > 0) vTaskDelay(pdMS_TO_TICKS(100));
          uart_write_bytes((uart_port_t)2, frame, 7);
          this->uart2_log_tx(frame, 7);
        }
      }
    }
    return std::string("OK:") + cmd;
  }

  // ── v3.16.35: online heartbeat — Server 每分鐘送一次，超過 90s 未收到 → 進入 offline 模式 ──
  //   {"type":"CMD","cmd":"online"}
  //   收到此 CMD：更新 last_online_cmd_ms_，若當下為 offline 則立即恢復 online
  //   （不送 EVENT，不觸發任何實體動作 — 純狀態維護）
  if (strcmp(cmd, "online") == 0) {
    this->last_online_cmd_ms_ = millis();
    if (this->offline_mode_) {
      this->offline_mode_ = false;
      ESP_LOGI(TAG, "[OFFLINE] exit → online (received online CMD)");
      this->log_to_dashboard("[OFFLINE] exit → Server heartbeat resumed");
    }
    return "OK:online";
  }

  // ── 系統指令 ──
  if (strcmp(cmd, "reboot") == 0) {
    ESP_LOGW(TAG, "CMD reboot requested (source=%s)", source ? source : "?");
    // v4.0.80: source-aware reboot 兇手標記 — 先前寫死 REBOOT_WS_CMD，
    //   HMI 0x2F 改走此路徑後 tag 會被誤標為 WS_CMD。改為依 source 區分：
    //   "HMI" → REBOOT_HMI / "DASH" → REBOOT_DASHBOARD / 其他 → REBOOT_WS_CMD
    if (source && strcmp(source, "HMI") == 0) {
      set_reboot_tag_c("REBOOT_HMI");
    } else if (source && strcmp(source, "DASH") == 0) {
      set_reboot_tag_c("REBOOT_DASHBOARD");
    } else {
      set_reboot_tag_c("REBOOT_WS_CMD");
    }
    this->reboot_pending_ = true;
    // v3.15.21: 設為 0，讓主 loop 的 deferred reboot init path 重新捕獲時間戳
    // 原本設 millis() 會略過 init 路徑，雖然仍可 reboot 但時序不一致
    this->reboot_requested_at_ = 0;
    return std::string("OK:rebooting");
  }
  if (strcmp(cmd, "status") == 0) {
    ESP_LOGI(TAG, "CMD status: sending STATUS report");
    this->send_status();
    return "OK:status sent";
  }
  if (strcmp(cmd, "getsn") == 0) {
    ESP_LOGI(TAG, "CMD getsn: returning device serial");
    return std::string("OK:") + this->device_id_;
  }

  // v3.16.28: go_enable CMD — 純粹設定 go_enable_arg_ 狀態變數（不觸發任何動作）
  //
  // 設計意圖：Server WS 斷線重連時會主動重送 go_enable CMD 來「重設狀態」。
  //   v3.16.26 曾把此 CMD 合併為「設定 + 模擬按鈕」雙功能，但導致 Server 每次重連
  //   都觸發一次 UART2 gowasher x5（當 blue=ON 時），造成洗車機誤動作。
  //   v3.16.28 改回純設定：CMD 只寫狀態變數，不送 EVENT，不觸發 UART2。
  //
  // 用法：
  //   {"cmd":"go_enable","arg":"gowasher"}            // 寫入 arg，無 TTL（永久）
  //   {"cmd":"go_enable","arg":"gowasher","ms":5000}  // 寫入 arg + 5 秒 TTL（倒數完回 gowasher）
  //   {"cmd":"go_enable","arg":"emg_stop"}            // Server 可手動設定任意 arg
  //
  // arg 值（物理 GO 按鈕按下時的行為對照）：
  //   "gowasher"  — 預設，blue=ON 時觸發 UART2 gowasher x5 + 送 EVENT
  //   "emg_stop"  — EMG 觸發後狀態，只送 EVENT，不觸發 UART2
  //   其他字串    — 只送 EVENT，不觸發 UART2（自訂模式）
  //
  // 本 CMD 不做的事：
  //   • 不送 EVENT（只有物理 GO 按鈕按下才送 go_enable EVENT）
  //   • 不觸發 UART2（只有物理 GO 按鈕按下才條件觸發）
  //   • 不模擬按鈕（按鈕永遠啟用，不可由 CMD 關閉）
  //
  // 副作用：Server 重送此 CMD 完全安全（只是反覆寫同樣狀態，無實體動作）
  //
  // 恢復 emg_stop 的方式：
  //   (1) Server 送 {"cmd":"go_enable","arg":"gowasher"}（純寫狀態，推薦）
  //   (2) wash_init CMD 自動重設（見 ws_client.cpp:1512）
  //   (3) 設定時 ms>0 的 TTL 過期（但 EMG 觸發時 expires_at_=0 無 TTL）
  if (strcmp(cmd, "go_enable") == 0) {
    bool has_arg = (arg && *arg);
    if (!has_arg) {
      ESP_LOGW(TAG, "CMD go_enable: arg missing — ignore (no-op)");
      return "OK:go_enable no-op (arg missing)";
    }
    std::string new_arg(arg);
    this->go_enable_arg_ = new_arg;
    if (ms > 0) {
      this->go_enable_arg_expires_at_ = millis() + ms;
    } else {
      this->go_enable_arg_expires_at_ = 0;
    }
    ESP_LOGI(TAG, "CMD go_enable: go_enable_arg='%s' ttl=%ums (pure setter, no EVENT, no UART2)",
             new_arg.c_str(), ms);
    return std::string("OK:go_enable arg=") + new_arg;
  }

  // v3.15.26: emergency_enable CMD — 啟用/停用緊急按鈕
  // {"cmd":"emergency_enable","on":0,"ms":10000}
  //   → 緊急按鈕 10 秒內停用（抽獎模式），10000ms 後自動回復 true
  // {"cmd":"emergency_enable","on":1,"ms":0}
  //   → 手動恢復啟用，無 TTL
  // ms=0 代表永久設定；收到新 CMD 會覆蓋舊 TTL
  if (strcmp(cmd, "emergency_enable") == 0) {
    this->emergency_enable_ = on;
    if (ms > 0) {
      this->emergency_enable_expires_at_ = millis() + ms;
    } else {
      this->emergency_enable_expires_at_ = 0;
    }
    ESP_LOGI(TAG, "emergency_enable set to %s ttl=%ums", on ? "ON" : "OFF", ms);
    return std::string("OK:emergency_enable=") + (on ? "ON" : "OFF");
  }

  // ── v3.16.0: wash_goto — Server 同步跳到指定步驟 ──
  // {"cmd":"wash_goto","id":"2311"} → arg 欄位帶 id
  if (strcmp(cmd, "wash_goto") == 0) {
    if (!this->wash_active_) return "ERR:wash not active";
    if (!arg || arg[0] == '\0') return "ERR:wash_goto missing id (use arg field)";
    int found_idx = -1;
    for (int i = 0; i < this->wash_total_; i++) {
      if (strcmp(this->wash_steps_[i].id, arg) == 0) {
        found_idx = i;
        break;
      }
    }
    if (found_idx < 0) return "ERR:wash_goto id not found";
    this->wash_stage_ = found_idx;
    this->wash_phase_ = WP_GREEN;
    // v4.0.12: 強制切到 GREEN 相位 — 同步 prev 到 "green" 避免 check_wash_stage 立即覆蓋
    this->prev_led_color_for_wash_ = "green";
    this->wash_all_bb_off();
    this->wash_set_bb(this->wash_steps_[found_idx].bb);
    // v3.16.48: R03 軟水套用目標步驟設定
    this->wash_apply_r03(this->wash_steps_[found_idx].r03);
    this->blue_enter_at_ = 0;  // v3.16.35: 跳轉到 GREEN → 離開 BLUE 倒數
    // v4.0.28: 用戶規則 — wash_goto 清暫停 flag（跳步視為明確繼續指令）
    this->wash_paused_ = false;
    this->pause_frozen_remain_ms_ = 0;
    this->pause_started_at_ = 0;
    ESP_LOGI(TAG, "wash_goto: jump to step %d (id='%s', bb=%d r03=%d)",
             found_idx + 1, arg, this->wash_steps_[found_idx].bb,
             this->wash_steps_[found_idx].r03);
    // v4.0.21: LED 交由 send_wash_stage_event 末端統一觸發
    this->send_wash_stage_event();
    // v4.0.09: 移除 send_status — Dashboard 改從 EVENT 更新卡片
    // v4.0.217 [WASH-EVENTS] 送 wash_goto EVENT — 帶目標 step id + 索引（1-based）
    // v4.0.243: 加 order_sn（流程進行中關聯訂單）
    {
      char ev[240];
      int n;
      if (!this->order_sn_.empty()) {
        n = snprintf(ev, sizeof(ev),
          "{\"type\":\"EVENT\",\"event\":\"wash_goto\","
          "\"target_id\":\"%s\",\"step_seq\":%d,\"source\":\"%s\",\"order_sn\":\"%s\"}",
          arg, found_idx + 1, source ? source : "?", this->order_sn_.c_str());
      } else {
        n = snprintf(ev, sizeof(ev),
          "{\"type\":\"EVENT\",\"event\":\"wash_goto\","
          "\"target_id\":\"%s\",\"step_seq\":%d,\"source\":\"%s\"}",
          arg, found_idx + 1, source ? source : "?");
      }
      if (n > 0 && (size_t)n < sizeof(ev)) this->send_event_raw(ev, n);
    }
    return "OK:wash_goto";
  }

  // ── all_off：關閉所有 switch（除 web_server）/ fan / light + 清除 timer ──
  if (strcmp(cmd, "all_off") == 0) {
    ESP_LOGW(TAG, "CMD all_off: shutting down all outputs");
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    for (auto *sw : App.get_switches()) {
      if (sw->get_object_id() != "web_server") sw->turn_off();
    }
    for (auto *f : App.get_fans()) {
      auto c = f->turn_off(); c.perform();
    }
    for (auto *lt : App.get_lights()) {
      auto c = lt->turn_off(); c.perform();
    }
#pragma GCC diagnostic pop
    for (int i = 0; i < MAX_TIMERS; i++) this->timers_[i].active = false;
    this->log_to_dashboard("ALL OFF executed");
    return "OK:all outputs off";
  }

  // ═══════════════════════════════════════════════════════════════
  // (2) Entity 查找（cmd 直接當 object_id）
  //     順序：switch → fan → light → button
  // ═══════════════════════════════════════════════════════════════
  const std::string target_oid = cmd;
  uint8_t dev_type = 0;  // 0=switch, 1=fan, 2=light
  bool found = false;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  // switch
  for (auto *sw : App.get_switches()) {
    if (sw->get_object_id() == target_oid) {
      if (on) sw->turn_on(); else sw->turn_off();
      found = true;
      dev_type = 0;
      break;
    }
  }
  // fan
  // arg 可指定速度 1-100（例：{"cmd":"cooling_fan","on":1,"arg":"50"} = 50%）
  // arg 省略或 <=0 → 預設最大速度
  if (!found) {
    for (auto *f : App.get_fans()) {
      if (f->get_object_id() == target_oid) {
        if (on) {
          auto c = f->turn_on();
          if (arg && *arg) {
            int speed = atoi(arg);
            if (speed > 0 && speed <= 100) c.set_speed(speed);
          }
          c.perform();
        } else {
          auto c = f->turn_off();
          c.perform();
        }
        found = true;
        dev_type = 1;
        break;
      }
    }
  }
  // light (WS2812 LED strips)
  // arg query-string format: "brightness=0-255&r=0-255&g=0-255&b=0-255&effect=NAME"
  // v3.16.58: 移除 ledcount 參數（led_count number entity 已拔除）
  if (!found) {
    for (auto *lt : App.get_lights()) {
      if (lt->get_object_id() == target_oid) {
        if (on) {
          auto c = lt->turn_on();
          // Parse arg query string for brightness / r / g / b / effect
          if (arg && *arg) {
            const char *p = arg;
            bool has_rgb = false;
            float rf = 0.f, gf = 0.f, bf = 0.f;
            while (*p) {
              const char *eq = strchr(p, '=');
              if (!eq) break;
              std::string key(p, eq - p);
              const char *val_start = eq + 1;
              const char *amp = strchr(val_start, '&');
              std::string val = amp ? std::string(val_start, amp - val_start) : std::string(val_start);
              if (key == "brightness") {
                int v = atoi(val.c_str());
                if (v < 0) v = 0; if (v > 255) v = 255;
                c.set_brightness((float)v / 255.0f);
              } else if (key == "r") {
                int v = atoi(val.c_str()); if (v < 0) v = 0; if (v > 255) v = 255;
                rf = (float)v / 255.0f; has_rgb = true;
              } else if (key == "g") {
                int v = atoi(val.c_str()); if (v < 0) v = 0; if (v > 255) v = 255;
                gf = (float)v / 255.0f; has_rgb = true;
              } else if (key == "b") {
                int v = atoi(val.c_str()); if (v < 0) v = 0; if (v > 255) v = 255;
                bf = (float)v / 255.0f; has_rgb = true;
              } else if (key == "effect") {
                c.set_effect(val);
              }
              // v3.16.58: ledcount 參數不再支援（靜默忽略）
              if (!amp) break;
              p = amp + 1;
            }
            if (has_rgb) c.set_rgb(rf, gf, bf);
          }
          c.perform();
        } else {
          auto c = lt->turn_off(); c.perform();
        }
        found = true;
        dev_type = 2;
        break;
      }
    }
  }
#pragma GCC diagnostic pop

  // button — 只在 on=true 時觸發 press（button 無 off 狀態，也無 auto-off timer）
  if (!found && on) {
    if (this->press_button(target_oid)) {
      ESP_LOGI(TAG, "CMD button pressed: %s", cmd);
      return std::string("OK:") + cmd + " pressed";
    }
  }

  if (!found) {
    ESP_LOGW(TAG, "CMD unknown/not found: %s", cmd);
    return std::string("ERR:not found ") + cmd;
  }

  // ── Auto-off timer (ms > 0 且 on=true 時啟動) ──
  if (ms > 0 && on) {
    uint32_t off_at = millis() + ms;
    int slot = -1;
    for (int i = 0; i < MAX_TIMERS; i++) {
      if (this->timers_[i].active && this->timers_[i].target == target_oid) {
        slot = i;
        break;
      }
    }
    if (slot < 0) {
      for (int i = 0; i < MAX_TIMERS; i++) {
        if (!this->timers_[i].active) { slot = i; break; }
      }
    }
    if (slot >= 0) {
      this->timers_[slot] = {target_oid, dev_type, off_at, true};
      ESP_LOGI(TAG, "CMD timer: %s off in %ums (slot %d)", target_oid.c_str(), ms, slot);
    } else {
      ESP_LOGW(TAG, "CMD timer: no free slot for %s", target_oid.c_str());
    }
  }

  char buf[96];
  snprintf(buf, sizeof(buf), "OK:%s %s%s%u%s",
           cmd, on ? "ON" : "OFF",
           ms > 0 ? " " : "",
           ms > 0 ? ms : 0,
           ms > 0 ? "ms" : "");
  ESP_LOGI(TAG, "CMD: %s", buf);
  return std::string(buf);
}

void WSClientComponent::check_timers(uint32_t now) {
  for (int i = 0; i < MAX_TIMERS; i++) {
    if (!this->timers_[i].active) continue;
    if ((int32_t)(now - this->timers_[i].off_at) < 0) continue;  // Not yet

    // Timer expired — turn off the device
    auto &t = this->timers_[i];
    ESP_LOGI(TAG, "Timer expired: %s (slot %d)", t.target.c_str(), i);

    if (t.type == 0) {
      for (auto *sw : App.get_switches()) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
        if (sw->get_object_id() == t.target) { sw->turn_off(); break; }
#pragma GCC diagnostic pop
      }
    } else if (t.type == 1) {
      for (auto *f : App.get_fans()) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
        if (f->get_object_id() == t.target) { auto c = f->turn_off(); c.perform(); break; }
#pragma GCC diagnostic pop
      }
    } else if (t.type == 2) {
      for (auto *lt : App.get_lights()) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
        if (lt->get_object_id() == t.target) { auto c = lt->turn_off(); c.perform(); break; }
#pragma GCC diagnostic pop
      }
    }

    this->log_to_dashboard("Timer OFF: " + t.target);
    t.active = false;
  }
}

void WSClientComponent::send_ack(const char *msg_id) {
  cJSON *doc = cJSON_CreateObject();
  cJSON_AddStringToObject(doc, "type", "ACK");
  cJSON_AddStringToObject(doc, "msg_id", msg_id);
  char *out = cJSON_PrintUnformatted(doc);
  this->send_text(out);
  cJSON_free(out);
  cJSON_Delete(doc);
}

// v3.15.25: 舊的 send_event(event_name, detail) 和 check_auto_triggers() 已移除。
//           新版 send_event(event_name, bool state) 定義於本檔案較上方（~L1150）。
//           所有 edge 偵測現由 YAML on_state lambda 負責。

// ─────────────────────────────────────────────────────────────────────
// send_status: 廣播完整 STATUS 訊息到 WS1（雲端）+ WS2（LAN）+ 本地 WS 伺服器
// 格式嚴格遵照 docs/hardware-api-reference.md §4.3.1（nested esp32/modbus/io）
// 發送時機：
//   (1) 連線成功 1 秒後一次
//   (2) 每 60 秒週期
//   (3) 收到 GET_STATUS / CMD "status"
// IO 狀態改變不會觸發此函數（per 使用者指示：只有 EVENT 主動送）
// ─────────────────────────────────────────────────────────────────────
void WSClientComponent::send_status(bool dashboard_only) {
  // v4.0.70: breadcrumb — 進 send_status（大量 cJSON/snprintf + 兩路 WS send，heap 密集）
  set_reboot_tag_c(dashboard_only ? "STATUS_DASH" : "STATUS_BOTH");
  // 即使 cloud 尚未連線也要建立 JSON 以推送到 LAN / 本地 WS
  // ── 1. 收集數值 sensor ──
  float last_uptime = 0, loop_max_us = 0, loop_time_us = 0;
  float cpu_usage = 0, reboot_count = 0;
  float min_free_heap_bytes = NAN;
  float temp = NAN, humidity = NAN, fan_speed_rpm = NAN;
  // v4.0.85: 電流變數對應新 sensor IDs (washer/pump/allExt)
  float total_current_a = NAN, washer_current_a = NAN, pump_current_a = NAN, allExt_current_a = NAN;
  float total_flow_l = NAN, water_flow_in = NAN, water_flow_out = NAN;
  for (auto *s : App.get_sensors()) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    const std::string &oid = s->get_object_id();
    if (oid == "last_boot_uptime") last_uptime = s->state;
    else if (oid == "loop_time_max") loop_max_us = s->state;
    else if (oid == "loop_time") loop_time_us = s->state;
    else if (oid == "cpu_usage") cpu_usage = s->state;
    else if (oid == "reboot_count") reboot_count = s->state;
    else if (oid == "min_free_heap") min_free_heap_bytes = s->state;
    else if (oid == "temperature") temp = s->state;
    else if (oid == "humidity") humidity = s->state;
    else if (oid == "fan_tachometer") fan_speed_rpm = s->state;
    else if (oid == "current_total") total_current_a = s->state;
    // v4.0.85: 電流 sensor 重命名 — current_pump→current_washer / current_air→current_pump / current_led→current_allExt
    else if (oid == "current_washer") washer_current_a = s->state;
    else if (oid == "current_pump") pump_current_a = s->state;
    else if (oid == "current_allExt") allExt_current_a = s->state;
    else if (oid == "total_flow") total_flow_l = s->state;
    else if (oid == "water_flow_1") water_flow_in = s->state;
    else if (oid == "water_flow_2") water_flow_out = s->state;
#pragma GCC diagnostic pop
  }

  // ── 2. 收集 text_sensor（reset_reason / crash_pc）──
  std::string reset_reason_str, crash_pc_str;
  for (auto *ts : App.get_text_sensors()) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    const std::string &oid = ts->get_object_id();
    if (oid == "reset_reason") reset_reason_str = ts->state;
    else if (oid == "crash_pc") crash_pc_str = ts->state;
#pragma GCC diagnostic pop
  }

  // ── 3. 收集 binary_sensor → 16-bit bitmask (固定字串長度 16，未使用位 '0' 補齊) ──
  // v4.0.14: 順序對齊 Dashboard BINARY_SENSOR_DEFS 顯示編號 01~12，方便人眼比對
  //   char[0]=bit1=01-外接電源, char[1]=bit2=02-漏水檢測, char[2]=bit3=03-軟水箱水位,
  //   char[3]=bit4=04-水箱水位, char[4]=bit5=05-洗車機藍燈, char[5]=bit6=06-洗車機綠燈,
  //   char[6]=bit7=07-洗車機紅燈, char[7]=bit8=08-緊急按鈕, char[8]=bit9=09-啟動停止,
  //   char[9]=bit10=10-車輛超高, char[10]=bit11=21-缺泡泡液, char[11]=bit12=25-門磁,
  //   char[12..15]=reserved '0'
  // v4.0.18 修正：bs->get_object_id() 回傳的是從 YAML `name:` 衍生的 object_id，
  //   不是 `id:`（C++ 變數名）。舊列表 12 個裡有 9 個拼錯，位元永遠鎖死 '0'。
  //   已對齊：
  //     "AC Power"             → ac_power            (原 ac_power_lose ✗)
  //     "Leak Detection"       → leak_detection      (原 leak_det ✗)
  //     "Soft Water Level"     → soft_water_level    (原 water_level_1 ✗)
  //     "Water Tank Level"     → water_tank_level    (原 water_level_2 ✗)
  //     "Washer LED Blue"      → washer_led_blue     (原樣 ✓)
  //     "Washer LED Green"     → washer_led_green    (原樣 ✓)
  //     "Washer LED Red"       → washer_led_red      (原樣 ✓)
  //     "Emergency Button"     → emergency_button    (原 emgr_input ✗)
  //     "Start Stop Button"    → start_stop_button   (原 start_stop_input ✗)
  //     "Car Height Detection" → car_height_detection(原 car_height_input ✗)
  //     "Bubble Liquid 1"      → bubble_liquid_1     (原 check_bubble_input_1 ✗)
  //     "Door Sensor"          → door_sensor         (原 check_door_input ✗)
  static const char *bs_order[] = {
    "ac_power",             // bit1  01 外接電源
    "leak_detection",       // bit2  02 漏水檢測
    "soft_water_level",     // bit3  03 軟水箱水位
    "water_tank_level",     // bit4  04 水箱水位
    "washer_led_blue",      // bit5  05 洗車機藍燈
    "washer_led_green",     // bit6  06 洗車機綠燈
    "washer_led_red",       // bit7  07 洗車機紅燈
    "emergency_button",     // bit8  08 緊急按鈕
    "start_stop_button",    // bit9  09 啟停按鈕
    "car_height_detection", // bit10 10 車輛超高
    "bubble_liquid_1",      // bit11 21 缺泡泡液
    "door_sensor"           // bit12 25 門磁
  };
  const int bs_count = sizeof(bs_order) / sizeof(bs_order[0]);
  uint32_t bs_bits = 0;
  for (auto *bs : App.get_binary_sensors()) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    const std::string &oid = bs->get_object_id();
    for (int i = 0; i < bs_count; i++) {
      if (oid == bs_order[i] && bs->state) { bs_bits |= (1u << i); break; }
    }
#pragma GCC diagnostic pop
  }
  // 轉成 16-char 字串（每 bit 一字元，無分隔；bit 1 在左，bit 16 預留=0）
  // 範例："0100100000000000" = ac_power_lose=0, emgr_input=1, ..., bit16=0
  char bs_str[17];
  for (int i = 0; i < 16; i++) {
    if (i < bs_count) {
      bs_str[i] = (bs_bits & (1u << i)) ? '1' : '0';
    } else {
      bs_str[i] = '0';  // reserved bit 16
    }
  }
  bs_str[16] = '\0';

  // ── 4. Modbus Relay (16-bit) ──
  // v3.16.77: SSR 1-4 mask 已完全移除（V1/V2 硬體層統一無 SSR 輸出），STATUS.data.ssr 欄位同步拔除
  uint16_t relay_mask = 0;
  for (auto *sw : App.get_switches()) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    const std::string &oid = sw->get_object_id();
    if (sw->state) {
      static const struct { const char *oid; int bit; } rmap[] = {
        {"spare_io",1}, {"pressure",2}, {"soft_water",3}, {"filter_clean",4},
        {"pwr_led",5}, {"pwr_other",6}, {"pwr_pump",7}, {"pwr_washer",8},
        {"bb1",9}, {"bb2",10}, {"bb3",11}, {"bb4",12},
        {"gate",13}, {"indoor_light1",14}, {"indoor_light2",15}, {"indoor_light3",16}
      };
      for (int r = 0; r < 16; r++) {
        if (oid == rmap[r].oid) { relay_mask |= (1 << (rmap[r].bit - 1)); break; }
      }
    }
#pragma GCC diagnostic pop
  }
  // v4.0.14: 順序修正 — char[0]=R01(spare_io), char[15]=R16(indoor_light3)，與 Dashboard SWITCH_DEFS 順序一致
  // 舊版（~v4.0.13）為 R16→R01 反向，造成位元和 Dashboard R01~R16 編號對不上
  char rly_bin[17];
  for (int i = 0; i < 16; i++) rly_bin[i] = (relay_mask & (1 << i)) ? '1' : '0';
  rly_bin[16] = '\0';

  // v3.16.59: LED 燈條（status_light_1/2）硬體與 YAML entity 已完全移除，
  //   Dashboard Light/Fan 控制卡也已拔掉。STATUS 不再輸出 led1/led2 物件。
  //   保留的 light entity 只剩 light_rgb1（ID LED, internal）與 light_net（網路 LED, internal）。

  // ═══════════════════════════════════════════════════════════════
  // Build flat JSON: {type, data:{...all fields flat...}}
  // ═══════════════════════════════════════════════════════════════
  cJSON *doc = cJSON_CreateObject();
  cJSON_AddStringToObject(doc, "type", "STATUS");
  cJSON *data = cJSON_AddObjectToObject(doc, "data");

  // ── 身分 + WS 連線狀態 ──
  cJSON_AddStringToObject(data, "serial", this->device_id_.c_str());
  cJSON_AddStringToObject(data, "firmware", this->firmware_version_.c_str());
  // === v4.0.241 [STATUS-LOCAL-IP] BEGIN ===
  // 主板本機 IP（W5500 / Wi-Fi 都適用，用 ESPHome network 統一抽象）
  // 開發 / 維運 / 連線除錯參考用
  {
    auto ips = network::get_ip_addresses();
    const char *ip_str = "";
    std::string ip_buf;
    if (!ips.empty() && ips[0].is_set()) {
      ip_buf = ips[0].str();
      ip_str = ip_buf.c_str();
    }
    cJSON_AddStringToObject(data, "local_ip", ip_str);
  }
  // === v4.0.241 [STATUS-LOCAL-IP] END ===
  cJSON_AddStringToObject(data, "WS1", this->connected_ ? "connected" : "disconnected");
  // WS2 使用嚴格版 is_lan_connected() 檢查（lan_connected_ + CS_IDLE + retry=0）
  // 防止 race condition 下 lan_connected_ 卡 true 但實際在重連時，
  // STATUS 錯誤回報 "connected" 誤導 RPi/雲端
  cJSON_AddStringToObject(data, "WS2", this->is_lan_connected() ? "connected" : "disconnected");

  // ── 系統資訊 ──
  // v4.0.69: NaN-guard 所有 sensor 讀值。web_server_switch 關閉時，被 gate 的 template
  //          sensor (reboot_count/last_boot_uptime/loop_time/loop_time_max/cpu_usage) 永不
  //          publish，s->state 為 NaN；(int)NaN 在 Xtensa/GCC 回傳 INT32_MAX=2147483647，
  //          STATUS 會噴 2147483647 讓 Server 解讀為「離譜大數」。改為 NaN 時省略欄位。
  cJSON_AddNumberToObject(data, "uptime_s", millis() / 1000);
  if (!std::isnan(reboot_count)) cJSON_AddNumberToObject(data, "reboot_count", (int)reboot_count);
  if (!std::isnan(last_uptime)) cJSON_AddNumberToObject(data, "last_uptime_s", (int)last_uptime);
  if (!reset_reason_str.empty()) cJSON_AddStringToObject(data, "reset_reason", reset_reason_str.c_str());
  if (!crash_pc_str.empty()) cJSON_AddStringToObject(data, "crash_pc", crash_pc_str.c_str());
  char val_buf[32];
  if (!std::isnan(cpu_usage)) {
    snprintf(val_buf, sizeof(val_buf), "%.1f%%", cpu_usage);
    cJSON_AddStringToObject(data, "cpu_usage", val_buf);
  }
  snprintf(val_buf, sizeof(val_buf), "%.1f KB", esp_get_free_heap_size() / 1024.0);
  cJSON_AddStringToObject(data, "free_heap", val_buf);
  if (!std::isnan(min_free_heap_bytes)) {
    snprintf(val_buf, sizeof(val_buf), "%.1f KB", min_free_heap_bytes / 1024.0);
    cJSON_AddStringToObject(data, "min_free_heap", val_buf);
  }
  if (!std::isnan(loop_max_us)) cJSON_AddNumberToObject(data, "loop_time_max_ms", (int)(loop_max_us / 1000.0 + 0.5));
  if (!std::isnan(loop_time_us)) cJSON_AddNumberToObject(data, "loop_time_ms", (int)(loop_time_us / 1000.0 + 0.5));
  // === v4.0.202 [RECONNECT-COUNT] BEGIN ===
  // WS1/WS2 當前 retry 連續次數（連上後歸 0）— 區分「真重連 vs log noise」
  // ws1_retry > 0 = 雲端正在 retry；ws2_retry > 0 = LAN 正在 retry
  cJSON_AddNumberToObject(data, "ws1_retry", (int)this->reconnect_attempts_);
  cJSON_AddNumberToObject(data, "ws2_retry", (int)this->lan_reconnect_attempts_);
  // === v4.0.202 [RECONNECT-COUNT] END ===
  if (!std::isnan(fan_speed_rpm)) cJSON_AddNumberToObject(data, "fan_speed_rpm", (int)fan_speed_rpm);
  if (!std::isnan(temp)) { snprintf(val_buf, sizeof(val_buf), "%.1f\u00B0C", temp); cJSON_AddStringToObject(data, "temperature", val_buf); }
  if (!std::isnan(humidity)) { snprintf(val_buf, sizeof(val_buf), "%.1f%%", humidity); cJSON_AddStringToObject(data, "humidity", val_buf); }

  // ── Modbus 數值（流量 + 電流）──
  if (!std::isnan(total_flow_l)) cJSON_AddNumberToObject(data, "total_flow_l", total_flow_l);
  // 電流 sensor 原始值為 centi-ampere（raw integer，例如 321 = 3.21 A），輸出時 ÷100 轉換為 A
  // v4.0.85: STATUS 電流欄位對應新命名（current_pump_a 重複用名 = 加壓馬達 addr 10，舊版是水泵 addr 9）
  if (!std::isnan(total_current_a)) cJSON_AddNumberToObject(data, "current_total_a", total_current_a / 100.0);
  if (!std::isnan(washer_current_a)) cJSON_AddNumberToObject(data, "current_washer_a", washer_current_a / 100.0);
  if (!std::isnan(pump_current_a)) cJSON_AddNumberToObject(data, "current_pump_a", pump_current_a / 100.0);
  if (!std::isnan(allExt_current_a)) cJSON_AddNumberToObject(data, "current_allExt_a", allExt_current_a / 100.0);
  if (!std::isnan(water_flow_in)) cJSON_AddNumberToObject(data, "water_flow_in_lpm", water_flow_in);
  if (!std::isnan(water_flow_out)) cJSON_AddNumberToObject(data, "water_flow_out_lpm", water_flow_out);

  // ── IO 狀態 ──
  cJSON_AddStringToObject(data, "binary_sensors", bs_str);
  // v4.0.10: 洗車機離線偵測 — 三個 LED 任一以 500ms 週期閃爍 → true
  //   判定邏輯：OFF→ON 邊緣距上次 OFF ≤1200ms 累計 ≥2 次 → flashing=true
  //            超過 2 秒無新邊緣 → 自動清除（loop() decay）
  cJSON_AddBoolToObject(data, "washer_offline", this->washer_offline_);
  // v3.16.77: "ssr" 欄位已移除（SSR 1-4 硬體層拔除）
  cJSON_AddStringToObject(data, "relay", rly_bin);
  // v3.16.59: led1 / led2 欄位移除（status_light_1/2 燈帶已拔）
  // v3.15.25: go_enable / emergency_enable 已移除佈防機制，STATUS 不再輸出這兩個物件

  // ── v3.16.0: 洗車階段計數器 ──
  // v3.16.8: 擴充 — 加入 wash_total / wash_current_bb / wash_steps_str
  //                   方便 Dashboard / Server 顯示完整運行狀況
  // v4.0.94: 基本欄位永遠輸出（即使 wash_active=false），讓 status CMD 永遠能查到流程狀態
  //   永遠輸出：wash_active / wash_paused / wash_phase / wash_total / wash_stage
  //   仍只 active 時輸出：詳細 step metadata / steps_str / bb / led_name 等
  {
    static const char *phase_names[] = {"idle", "green", "blue", "red", "done", "nopower"};
    const char *ps = (this->wash_phase_ <= WP_NOPOWER) ? phase_names[this->wash_phase_] : "unknown";
    if (!this->unlock_pwd_.empty()) ps = "lock";
    cJSON_AddBoolToObject(data, "wash_active", this->wash_active_);
    cJSON_AddBoolToObject(data, "wash_paused", this->wash_paused_);
    cJSON_AddNumberToObject(data, "wash_total", this->wash_total_);
    int step_idx = this->wash_stage_;
    if (this->wash_phase_ == WP_BLUE && step_idx > 0) step_idx = step_idx - 1;
    int step_seq = (step_idx < this->wash_total_) ? step_idx + 1 : 0;
    cJSON_AddNumberToObject(data, "wash_stage", step_seq);
    cJSON_AddStringToObject(data, "wash_phase", ps);
  }
  if (this->wash_active_) {
    static const char *phase_names[] = {"idle", "green", "blue", "red", "done", "nopower"};
    const char *ps = (this->wash_phase_ <= WP_NOPOWER) ? phase_names[this->wash_phase_] : "unknown";
    if (!this->unlock_pwd_.empty()) ps = "lock";
    (void)ps;  // 上方 block 已輸出 wash_phase
    // v4.0.83: 鎖定密碼 — 非空時放入 STATUS（Dashboard 顯示用，明文）
    if (!this->unlock_pwd_.empty()) {
      cJSON_AddStringToObject(data, "unlock_pwd", this->unlock_pwd_.c_str());
    }
    int step_idx = this->wash_stage_;
    if (this->wash_phase_ == WP_BLUE && step_idx > 0) step_idx = step_idx - 1;
    int step_seq = (step_idx < this->wash_total_) ? step_idx + 1 : 0;
    const char *step_id = (step_idx < this->wash_total_) ? this->wash_steps_[step_idx].id : "";
    uint8_t step_bb = (step_idx < this->wash_total_) ? this->wash_steps_[step_idx].bb : 0;
    cJSON_AddNumberToObject(data, "wash_step_seq", step_seq);
    cJSON_AddStringToObject(data, "wash_step_id", step_id);
    // 當前正在開的泡泡（GREEN 時有值，其他 phase 通常 0）
    cJSON_AddNumberToObject(data, "wash_current_bb", (this->wash_phase_ == WP_GREEN) ? step_bb : 0);
    // v3.16.9: 泡泡 auto-off 設定 + 剩餘秒數
    cJSON_AddNumberToObject(data, "wash_bb_timeout_s", this->wash_bb_timeout_ms_ / 1000);
    if (this->wash_bb_expire_at_ != 0) {
      int32_t remain_ms = (int32_t)(this->wash_bb_expire_at_ - millis());
      cJSON_AddNumberToObject(data, "wash_bb_remain_s", remain_ms > 0 ? remain_ms / 1000 : 0);
    } else {
      cJSON_AddNumberToObject(data, "wash_bb_remain_s", 0);
    }
    // 所有 step 的 id:bb:R03 字串（方便 Dashboard 一次顯示）
    // v3.16.50: 擴充為 id:bb:R03 格式（原 id:bb），讓 Dashboard 能直接顯示完整狀態
    char steps_buf[256];
    int off = 0;
    for (int i = 0; i < this->wash_total_ && off < (int)sizeof(steps_buf) - 20; i++) {
      off += snprintf(steps_buf + off, sizeof(steps_buf) - off, "%s%s:%d:%d",
                      i == 0 ? "" : ",",
                      this->wash_steps_[i].id, this->wash_steps_[i].bb, this->wash_steps_[i].r03);
    }
    steps_buf[sizeof(steps_buf) - 1] = '\0';
    cJSON_AddStringToObject(data, "wash_steps_str", steps_buf);

    // v3.16.47: 當前步驟 metadata（name/auto_sec/LED 資訊），供 Dashboard 顯示
    //   auto_remain_s：僅 BLUE 相位且 auto_sec>0 時才回報剩餘秒數；Dashboard 收到
    //   後可用 setInterval 本地每秒 tick 避免靠 STATUS push 刷新
    if (step_idx < this->wash_total_) {
      const WashStep &s = this->wash_steps_[step_idx];
      cJSON_AddStringToObject(data, "wash_step_name", s.name);
      cJSON_AddNumberToObject(data, "wash_step_r03", s.r03);  // v3.16.48: 軟水切換狀態
      cJSON_AddNumberToObject(data, "wash_step_auto_sec", s.auto_sec);
      int auto_remain = 0;
      if (this->wash_phase_ == WP_BLUE && this->blue_enter_at_ != 0 && s.auto_sec > 0) {
        uint32_t elapsed_s = (millis() - this->blue_enter_at_) / 1000;
        int32_t remain = (int32_t)s.auto_sec - (int32_t)elapsed_s;
        auto_remain = remain > 0 ? remain : 0;
      }
      // v4.0.94: last_step_finish_at_ 倒數中（last step 獨立倒數）也回報剩餘秒
      if (this->last_step_finish_at_ != 0 && s.auto_sec > 0) {
        int32_t remain_ms = (int32_t)(this->last_step_finish_at_ - millis());
        auto_remain = remain_ms > 0 ? remain_ms / 1000 : 0;
      }
      cJSON_AddNumberToObject(data, "wash_step_auto_remain_s", auto_remain);
      // v3.16.59: 只保留 led_name（VSN 檔名）— led_text / led_color / light_color 已移除
      cJSON_AddStringToObject(data, "wash_step_led_name", s.led_name);
    }
    // v3.16.59: 最近一次 LED 節目切換（led_switch CMD + offline_led_trigger 共用）
    cJSON_AddStringToObject(data, "wash_last_led_name", this->last_led_name_.c_str());
    cJSON_AddStringToObject(data, "wash_last_led_source", this->last_led_source_.c_str());

    // v3.16.59: 總洗車時間（秒）— step[0]→step[1] 轉換起算，done_flag 鎖定
    //   運行中動態遞增，完成後鎖定為 wash_total_duration_s_
    uint32_t total_sec = this->wash_total_duration_s_;
    if (total_sec == 0 && this->wash_total_start_ms_ != 0) {
      total_sec = (millis() - this->wash_total_start_ms_) / 1000U;
    }
    cJSON_AddNumberToObject(data, "wash_total_sec", total_sec);
    // v4.0.111: 訂單三欄位 (Dashboard wash card 顯示用)
    cJSON_AddStringToObject(data, "order_sn",     this->order_sn_.c_str());
    cJSON_AddStringToObject(data, "order_name",   this->order_name_.c_str());
    cJSON_AddStringToObject(data, "order_msg",    this->order_msg_.c_str());
    cJSON_AddStringToObject(data, "order_cost", this->order_cost_.c_str());  // v4.0.215
    cJSON_AddStringToObject(data, "order_product", this->order_product_.c_str());  // v4.0.215
  }

  // v3.16.59: done_flag=true 但 wash_active=false 時，仍回報最終總秒數（HMI/Dashboard 可顯示「結束 Xs」）
  if (!this->wash_active_ && this->wash_done_flag_ && this->wash_total_duration_s_ > 0) {
    cJSON_AddNumberToObject(data, "wash_total_sec", this->wash_total_duration_s_);
    cJSON_AddBoolToObject(data, "wash_done", true);
  }

  // === v4.0.238 [DEVICE-REGISTRY] BEGIN ===
  // 加 paired_devices array — 列出所有 5 分鐘內 announce 過的外部裝置
  // 過期 (>5min 沒 announce) 的條目 STATUS 略過顯示 (但仍留在 registry，下次 announce 自動 refresh)
  if (!this->device_registry_.empty()) {
    cJSON *devs = cJSON_CreateArray();
    uint32_t now = millis();
    for (auto &p : this->device_registry_) {
      const DeviceInfo &d = p.second;
      uint32_t age_ms = now - d.last_seen_ms;
      if (age_ms > 300000U) continue;  // >5min stale, 略過
      cJSON *o = cJSON_CreateObject();
      cJSON_AddStringToObject(o, "id", d.id.c_str());
      cJSON_AddStringToObject(o, "ip", d.ip.c_str());
      cJSON_AddBoolToObject(o, "paired", (!d.paired_sn.empty() && d.paired_sn == this->device_id_));
      cJSON_AddNumberToObject(o, "age_s", (int)(age_ms / 1000U));
      cJSON_AddStringToObject(o, "ver", d.ver.c_str());
      cJSON_AddNumberToObject(o, "rssi", d.rssi);
      if (!d.bssid.empty()) cJSON_AddStringToObject(o, "bssid", d.bssid.c_str());
      cJSON_AddItemToArray(devs, o);
    }
    cJSON_AddItemToObject(data, "paired_devices", devs);
  }
  // === v4.0.238 [DEVICE-REGISTRY] END ===

  // ═══════════════════════════════════════════════════════════════
  // Serialize + 廣播（Cloud WS1 + LAN WS2 uplink）
  // ═══════════════════════════════════════════════════════════════
  char *out = cJSON_PrintUnformatted(doc);
  if (out) {
    // v4.0.03: dashboard_only=true 時跳過 WS1/WS2 推送 — 給 wash UI 更新等高頻場景用
    //   避免對 Server 灌垃圾（Server 拿 EVENT 就夠），只讓 Dashboard SSE 取得新鮮狀態
    if (!dashboard_only) {
      if (this->connected_) this->send_text(out);           // WS1 雲端
      // v3.15.21: 回到 v3.15.19 的簡單邏輯 — is_lan_connected() 現在回到簡單版
      if (this->is_lan_connected()) {
        this->lan_send_text(out);                            // WS2 LAN uplink
        this->log_to_dashboard(std::string("[LAN] TX STATUS: ") + out);
      }
    }
    this->publish_sensor("ws_last_status", std::string(out));
    cJSON_free(out);
  }
  cJSON_Delete(doc);
}

// =========================================================================
// LAN WebSocket Client (second connection — plain TCP, auto-connect)
// =========================================================================
// LAN_TAG 已移至檔案頂部（line 28）讓 loop() 可以使用

void WSClientComponent::lan_connect_task_fn(void *param) {
  auto *self = static_cast<WSClientComponent *>(param);
  bool ok = self->lan_do_connect();
  self->lan_connect_state_ = ok ? CS_CONNECTED : CS_FAILED;
  self->lan_connect_task_handle_ = nullptr;
  vTaskDelete(nullptr);
}

bool WSClientComponent::lan_do_connect() {
  int port = this->lan_port_ ? this->lan_port_ : 80;

  uint32_t t0 = millis();

  // v4.0.273 [DNS-CACHE-FIX]: 同 WS1，cache 解析結果跳過後續 getaddrinfo
  //   WS2 通常是 IP literal (192.168.1.10)，getaddrinfo 應立即返回；但若用 hostname 也保護
  static char lan_cached_ip_[16] = "";
  static int lan_tcp_fail_count_ = 0;
  static std::string lan_cached_host_;

  // Step 1: DNS resolve (skip if cached)
  ::set_reboot_tag_c("LOOP_LAN_WS_RESOLV");  // v4.0.273 [BREADCRUMB-WS1-DETAIL]
  struct sockaddr_in dest;
  memset(&dest, 0, sizeof(dest));
  dest.sin_family = AF_INET;
  dest.sin_port = htons(port);

  bool lan_cache_valid = (lan_cached_ip_[0] != '\0' && lan_cached_host_ == this->lan_server_ && lan_tcp_fail_count_ < 10);
  if (lan_cache_valid) {
    if (inet_aton(lan_cached_ip_, &dest.sin_addr) == 0) {
      lan_cached_ip_[0] = '\0';
      lan_cache_valid = false;
    } else {
      ESP_LOGI(LAN_TAG, "[1/3] DNS CACHED: %s -> %s (fails=%d/10)",
               this->lan_server_.c_str(), lan_cached_ip_, lan_tcp_fail_count_);
    }
  }
  if (!lan_cache_valid) {
    ESP_LOGI(LAN_TAG, "[1/3] DNS resolving %s...", this->lan_server_.c_str());
    struct addrinfo hints = {};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    struct addrinfo *res = nullptr;
    int dns_err = getaddrinfo(this->lan_server_.c_str(), nullptr, &hints, &res);
    if (dns_err != 0 || !res) {
      ESP_LOGE(LAN_TAG, "DNS failed: %d", dns_err);
      if (res) freeaddrinfo(res);
      return false;
    }
    struct sockaddr_in *addr = (struct sockaddr_in *)res->ai_addr;
    dest.sin_addr = addr->sin_addr;
    char ip_str[16];
    inet_ntoa_r(addr->sin_addr, ip_str, sizeof(ip_str));
    freeaddrinfo(res);
    strncpy(lan_cached_ip_, ip_str, sizeof(lan_cached_ip_));
    lan_cached_ip_[sizeof(lan_cached_ip_) - 1] = '\0';
    lan_cached_host_ = this->lan_server_;
    lan_tcp_fail_count_ = 0;
  }

  // Step 2: TCP connect
  ESP_LOGI(LAN_TAG, "[2/3] TCP connecting to %s:%d...", this->lan_server_.c_str(), port);
  ::set_reboot_tag_c("LOOP_LAN_WS_TCPCONN");  // v4.0.273 [BREADCRUMB-WS1-DETAIL]

  this->lan_sock_fd_ = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (this->lan_sock_fd_ < 0) {
    ESP_LOGE(LAN_TAG, "socket() failed: errno=%d", errno);
    return false;
  }

  // v4.0.270 [CONNECT-TIMEOUT-FIX]: Non-blocking connect + 500ms select timeout
  //   Root cause: ::connect() default blocking ~75s when remote (RPi) unreachable
  //   SO_SNDTIMEO/SO_RCVTIMEO 不管 connect (只管 read/write)
  //   修法: 切 non-blocking → connect 返 EINPROGRESS → select() 500ms timeout
  //   RPi 離線時 main loop 卡 75s → 500ms (恢復速度 150x)
  int lan_orig_flags = fcntl(this->lan_sock_fd_, F_GETFL, 0);
  fcntl(this->lan_sock_fd_, F_SETFL, lan_orig_flags | O_NONBLOCK);

  uint32_t lan_tcp_t0 = millis();
  int lan_conn_ret = ::connect(this->lan_sock_fd_, (struct sockaddr *)&dest, sizeof(dest));
  if (lan_conn_ret != 0 && errno != EINPROGRESS) {
    uint32_t dt = millis() - lan_tcp_t0;
    ESP_LOGE(LAN_TAG, "TCP connect immediate fail in %ums: errno=%d", dt, errno);
    close(this->lan_sock_fd_);
    this->lan_sock_fd_ = -1;
    lan_tcp_fail_count_++;  // v4.0.273 [DNS-CACHE-FIX]
    return false;
  }

  fd_set lan_wfds;
  FD_ZERO(&lan_wfds);
  FD_SET(this->lan_sock_fd_, &lan_wfds);
  struct timeval lan_tv_conn = { .tv_sec = 0, .tv_usec = 500000 };  // 500ms
  int lan_sel_ret = select(this->lan_sock_fd_ + 1, NULL, &lan_wfds, NULL, &lan_tv_conn);
  uint32_t lan_tcp_dt = millis() - lan_tcp_t0;

  if (lan_sel_ret <= 0) {
    ESP_LOGE(LAN_TAG, "TCP connect TIMEOUT in %ums (select=%d errno=%d)", lan_tcp_dt, lan_sel_ret, errno);
    close(this->lan_sock_fd_);
    this->lan_sock_fd_ = -1;
    lan_tcp_fail_count_++;  // v4.0.273 [DNS-CACHE-FIX]
    return false;
  }

  int lan_so_err = 0;
  socklen_t lan_so_len = sizeof(lan_so_err);
  getsockopt(this->lan_sock_fd_, SOL_SOCKET, SO_ERROR, &lan_so_err, &lan_so_len);
  if (lan_so_err != 0) {
    ESP_LOGE(LAN_TAG, "TCP connect SO_ERROR=%d in %ums", lan_so_err, lan_tcp_dt);
    close(this->lan_sock_fd_);
    this->lan_sock_fd_ = -1;
    lan_tcp_fail_count_++;  // v4.0.273 [DNS-CACHE-FIX]
    return false;
  }
  lan_tcp_fail_count_ = 0;  // v4.0.273 [DNS-CACHE-FIX] TCP 成功 reset fail counter

  // 還原 blocking
  fcntl(this->lan_sock_fd_, F_SETFL, lan_orig_flags);

  struct timeval tv;
  tv.tv_sec = 10;
  tv.tv_usec = 0;
  setsockopt(this->lan_sock_fd_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
  setsockopt(this->lan_sock_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  // === v4.0.212 [WDT-FIX] BEGIN ===
  // 同 WS1：connect OK 後 SNDTIMEO 10s → 3s 防 main loop 卡 > WDT 5s
  {
    struct timeval tv_send;
    tv_send.tv_sec = 3;
    tv_send.tv_usec = 0;
    setsockopt(this->lan_sock_fd_, SOL_SOCKET, SO_SNDTIMEO, &tv_send, sizeof(tv_send));
    ESP_LOGI(LAN_TAG, "[WDT-FIX] WS2 SNDTIMEO 10s → 3s (post-connect)");
  }
  // === v4.0.212 [WDT-FIX] END ===

  // Step 3: WebSocket handshake (plain TCP, no TLS)
  ESP_LOGI(LAN_TAG, "[3/3] WebSocket handshake...");
  if (!this->lan_ws_handshake()) {
    ESP_LOGE(LAN_TAG, "WS handshake failed");
    close(this->lan_sock_fd_);
    this->lan_sock_fd_ = -1;
    return false;
  }

  uint32_t total = millis() - t0;
  ESP_LOGI(LAN_TAG, "LAN WS connected in %ums", total);

  // 連線成功後切換為 non-blocking mode
  // 重要：recv() 在主 loop 中調用，如果用 blocking mode（SO_RCVTIMEO=10s）
  // 會阻塞整個 ESPHome 主 loop（包括 web_server、SSE、HMI 等），導致網頁凍住
  int flags = fcntl(this->lan_sock_fd_, F_GETFL, 0);
  fcntl(this->lan_sock_fd_, F_SETFL, flags | O_NONBLOCK);

  return true;
}

bool WSClientComponent::lan_ws_handshake() {
  // Generate random 16-byte key
  uint8_t raw_key[16];
  for (int i = 0; i < 16; i++) raw_key[i] = (esp_random() & 0xFF);
  unsigned char b64_key[32];
  size_t b64_len = 0;
  mbedtls_base64_encode(b64_key, sizeof(b64_key), &b64_len, raw_key, 16);
  b64_key[b64_len] = '\0';

  std::string path = "/ws?device_id=" + this->device_id_;
  std::string request =
    "GET " + path + " HTTP/1.1\r\n"
    "Host: " + this->lan_server_ + "\r\n"
    "Upgrade: websocket\r\n"
    "Connection: Upgrade\r\n"
    "Sec-WebSocket-Key: " + std::string((char *)b64_key) + "\r\n"
    "Sec-WebSocket-Version: 13\r\n"
    "\r\n";

  size_t written = 0;
  while (written < request.length()) {
    ssize_t ret = ::send(this->lan_sock_fd_, request.c_str() + written, request.length() - written, 0);
    if (ret < 0) return false;
    written += ret;
  }

  char resp[1024];
  int resp_len = 0;
  uint32_t start = millis();
  while (millis() - start < 10000 && resp_len < (int)sizeof(resp) - 1) {
    ssize_t ret = ::recv(this->lan_sock_fd_, resp + resp_len, sizeof(resp) - 1 - resp_len, 0);
    if (ret > 0) {
      resp_len += ret;
      resp[resp_len] = '\0';
      if (strstr(resp, "\r\n\r\n")) break;
    } else if (ret == 0) {
      return false;
    } else {
      if (errno == EAGAIN || errno == EWOULDBLOCK) { delay(10); continue; }
      return false;
    }
  }

  if (strncmp(resp, "HTTP/1.1 101", 12) != 0) {
    ESP_LOGE(LAN_TAG, "WS upgrade failed: %.64s", resp);
    return false;
  }

  // Save leftover data
  this->lan_handshake_leftover_len_ = 0;
  const char *header_end = strstr(resp, "\r\n\r\n");
  if (header_end) {
    const char *lo_start = header_end + 4;
    int lo_len = resp_len - (lo_start - resp);
    if (lo_len > 0) {
      if (lo_len > (int)sizeof(this->lan_handshake_leftover_)) lo_len = sizeof(this->lan_handshake_leftover_);
      memcpy(this->lan_handshake_leftover_, lo_start, lo_len);
      this->lan_handshake_leftover_len_ = lo_len;
    }
  }

  return true;
}

void WSClientComponent::lan_do_disconnect() {
  if (this->lan_sock_fd_ >= 0) {
    close(this->lan_sock_fd_);
    this->lan_sock_fd_ = -1;
  }
}

bool WSClientComponent::lan_send_ws_frame(uint8_t opcode, const uint8_t *data, size_t len) {
  if (this->lan_sock_fd_ < 0) return false;
  uint8_t *frame = this->lan_frame_buf_;
  size_t frame_len = 0;

  frame[0] = 0x80 | (opcode & 0x0F);
  frame_len = 1;
  if (len < 126) {
    frame[1] = 0x80 | (uint8_t)len;
    frame_len = 2;
  } else {
    frame[1] = 0x80 | 126;
    frame[2] = (len >> 8) & 0xFF;
    frame[3] = len & 0xFF;
    frame_len = 4;
  }

  // Mask key
  uint8_t mask[4];
  for (int i = 0; i < 4; i++) mask[i] = (esp_random() & 0xFF);
  memcpy(frame + frame_len, mask, 4);
  frame_len += 4;

  if (frame_len + len > sizeof(this->lan_frame_buf_)) return false;
  for (size_t i = 0; i < len; i++) {
    frame[frame_len + i] = data[i] ^ mask[i % 4];
  }
  frame_len += len;

  size_t sent = 0;
  while (sent < frame_len) {
    ssize_t ret = ::send(this->lan_sock_fd_, frame + sent, frame_len - sent, 0);
    if (ret <= 0) return false;
    sent += ret;
  }
  return true;
}

bool WSClientComponent::lan_send_text(const char *text, int len) {
  if (len < 0) len = strlen(text);
  // === v4.0.212 [WDT-FIX] BEGIN ===
  // LAN socket 斷線/不通的守衛 — 不可在 main loop 內等 SNDTIMEO 3s 反覆消耗
  if (!this->is_lan_connected() || this->lan_sock_fd_ < 0) return false;
  bool ok = this->lan_send_ws_frame(0x01, (const uint8_t *)text, len);
  if (!ok) {
    // send 失敗 → 標記 disconnected 觸發 reconnect
    this->lan_connected_ = false;
    ESP_LOGE(LAN_TAG, "lan_send_text FAILED → mark disconnected (likely SNDTIMEO)");
  } else {
    // === v4.0.234 [WS2-FALLBACK-PUBLISH] BEGIN ===
    // WS1 斷線時 send_text 不會走 → ws_last_sent 不更新 → dashboard JS 失憶
    // 解法：WS2 lan_send_text 成功時也 publish ws_last_sent 作 fallback
    // 排除 STATUS（同 send_text 邏輯，line 1310；STATUS 走 ws_last_status 通道）
    // 風險：WS1+WS2 都 connected 時會雙寫（WS1 先寫 → WS2 後寫蓋掉），但內容相同無害
    bool is_status = (len >= 17 && strncmp(text, "{\"type\":\"STATUS\"", 16) == 0);
    if (!is_status) {
      this->publish_sensor("ws_last_sent", std::string(text, len));
    }
    // === v4.0.234 [WS2-FALLBACK-PUBLISH] END ===
  }
  return ok;
  // === v4.0.212 [WDT-FIX] END ===
}

int WSClientComponent::lan_read_one_frame() {
  // Read from leftover first, then socket
  uint8_t hdr[2];
  int got = 0;

  auto do_read = [&](void *buf, size_t want) -> ssize_t {
    if (this->lan_handshake_leftover_len_ > 0) {
      size_t copy = want < (size_t)this->lan_handshake_leftover_len_ ? want : (size_t)this->lan_handshake_leftover_len_;
      memcpy(buf, this->lan_handshake_leftover_, copy);
      this->lan_handshake_leftover_len_ -= copy;
      if (this->lan_handshake_leftover_len_ > 0)
        memmove(this->lan_handshake_leftover_, this->lan_handshake_leftover_ + copy, this->lan_handshake_leftover_len_);
      return (ssize_t)copy;
    }
    return ::recv(this->lan_sock_fd_, buf, want, 0);
  };

  ssize_t ret = do_read(hdr, 2);
  if (ret <= 0) {
    if (ret == 0) return -1;  // closed
    if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
    return -1;
  }
  if (ret < 2) return 0;  // incomplete

  uint8_t opcode = hdr[0] & 0x0F;
  bool masked = (hdr[1] & 0x80) != 0;
  size_t payload_len = hdr[1] & 0x7F;

  if (payload_len == 126) {
    uint8_t ext[2];
    if (do_read(ext, 2) < 2) return -1;
    payload_len = ((size_t)ext[0] << 8) | ext[1];
  }

  uint8_t mask_key[4] = {};
  if (masked) {
    if (do_read(mask_key, 4) < 4) return -1;
  }

  if (payload_len >= sizeof(this->lan_read_buf_)) return -1;  // too big

  size_t total_read = 0;
  while (total_read < payload_len) {
    ret = do_read(this->lan_read_buf_ + total_read, payload_len - total_read);
    if (ret <= 0) return -1;
    total_read += ret;
  }

  if (masked) {
    for (size_t i = 0; i < payload_len; i++)
      this->lan_read_buf_[i] ^= mask_key[i % 4];
  }
  this->lan_read_buf_[payload_len] = '\0';

  // Handle opcodes
  if (opcode == 0x08) return -1;  // close
  if (opcode == 0x09) {  // ping from RPi → reply pong
    this->lan_send_ws_frame(0x0A, (const uint8_t *)this->lan_read_buf_, payload_len);
    if (show_ping_log_) this->log_to_dashboard("[LAN] RX: WS Ping → TX: Pong");  // v4.0.117+
    return 1;
  }
  if (opcode == 0x0A) {  // pong from RPi（回應我方 ping）
    this->lan_got_server_data_ = true;
    if (show_ping_log_) this->log_to_dashboard("[LAN] RX: WS Pong");  // v4.0.117+
    return 1;
  }
  if (opcode == 0x01 || opcode == 0x02) {  // text or binary
    this->lan_got_server_data_ = true;
    this->lan_process_message(this->lan_read_buf_, payload_len);
    return 1;
  }
  return 1;
}

void WSClientComponent::lan_process_message(const char *data, int len) {
  ESP_LOGI(LAN_TAG, "RX: %.*s", len > 200 ? 200 : len, data);
  // v4.0.70: breadcrumb — 進 lan_process_message（WS2 LAN 收到訊息）
  set_reboot_tag_c("LAN_MSG");

  std::string msg(data, len);

  cJSON *doc = cJSON_Parse(data);
  if (!doc) return;

  const char *msg_type = "DATA";
  cJSON *type_item = cJSON_GetObjectItem(doc, "type");
  if (cJSON_IsString(type_item)) msg_type = type_item->valuestring;

  this->lan_got_server_data_ = true;

  // 記錄完整收到訊息到 Dashboard（[LAN] 前綴讓 JS 分流到 WS2 面板）
  {
    std::string log_rx = std::string("[LAN] RX: ") + msg;
    if (log_rx.size() > 220) log_rx = log_rx.substr(0, 220) + "...";
    this->log_to_dashboard(log_rx);
  }

  // PING → PONG (application-level，RPi 層級的 ping)
  if (strcmp(msg_type, "PING") == 0) {
    const char *pong = "{\"type\":\"PONG\"}";
    this->lan_send_text(pong);
    this->log_to_dashboard(std::string("[LAN] TX: ") + pong);
    cJSON_Delete(doc);
    return;
  }

  // CMD handling — v4.0.42: 統一走 handle_cmd_message（WS1/WS2 共用）
  if (strcmp(msg_type, "CMD") == 0) {
    this->handle_cmd_message(doc, "LAN");
    cJSON_Delete(doc);
    return;
  }

  cJSON_Delete(doc);
}

// lan_send_status：保留名稱供既有呼叫點使用，直接委派給 send_status
// 由 send_status 內部統一廣播到 WS1 + WS2 + 本地 WS，確保三者格式一致。
void WSClientComponent::lan_send_status() {
  this->send_status();
}

// v3.15.25: lan_send_event() 已移除。統一透過 send_event() / send_washer_status() 雙發 WS1+WS2。

}  // namespace ws_client
}  // namespace esphome
