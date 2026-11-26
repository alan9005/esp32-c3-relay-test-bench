// colorlight.h — HTTP PUT client for Colorlight LED 播放盒
// v4.0.64 — 持久 worker task + queue（根治 lwIP close race）
//
// 架構：setup() 啟動**一支常駐 worker task**，for(;;) 從 queue 取請求做 PUT。
// task 永不退出 → lwIP tcpip_task 的 async callback（SYN timer 取消等）
// 永遠有有效 task context 可引用，不會再踩到已釋放的 socket PCB。
//
// v4.0.18~63 歷史：fire-and-forget xTaskCreate 每次 put 開新 task，
//   雖然加了 RAII guard + shutdown+SO_LINGER+close + 200ms post-delay，
//   但 close 返回後 task 退出，lwIP callback 仍可能在 task 死後觸發 → PC=0 panic。
//
// 呼叫方式（從 YAML lambda）：
//   id(cl).put(
//     "http://192.168.1.14/api/brightness",          // url
//     "Basic YWRtaW46Q29uc29sZUAxMjM=",              // auth header
//     "application/json; charset=utf-8",             // content-type
//     "{\"brightness\":128}"                        // body
//   );
//
// 限制：
//   - 只支援 http:// (無 TLS)
//   - 僅限 ESP32 LAN 內可達的裝置（Colorlight 192.168.1.14）
//   - Queue 1-slot latest-wins：連發時只保留最新請求
//   - 不回報成功/失敗（fire-and-forget），debug 靠 ESP_LOG / ws_log callback

#pragma once

#include "esphome/core/component.h"
#include <string>
#include <atomic>
#include <mutex>
#include <vector>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

namespace esphome {
namespace colorlight {

// ────────────────────────────────────────────────────────────────
// Queue 內傳遞的請求（heap allocated，worker 處理完 delete）
// ────────────────────────────────────────────────────────────────
struct ColorlightRequest {
  std::string url;
  std::string auth;
  std::string content_type;
  std::string body;
  std::string method;  // v4.0.105: HTTP method (空字串 = "PUT" 向後相容)
};

class ColorlightClient : public Component {
 public:
  void setup() override;
  void loop() override;
  float get_setup_priority() const override { return esphome::setup_priority::AFTER_WIFI; }

  // 發送 HTTP PUT（非阻塞）。立刻返回；實際 I/O 在常駐 worker task。
  // v4.0.105: ColorlightRequest 內加 method 欄位（預設 "PUT"），do_put 用 req.method 動態
  //   未來若有 GET 需求可擴展（不在 v4.0.105 範圍）
  void put(const std::string &url,
           const std::string &auth,
           const std::string &content_type,
           const std::string &body);

  // ── Dashboard log callback（task-safe 2-stage：task 推 queue → loop drain）
  using LogCallback = void(*)(const char *msg);
  static void set_log_callback(LogCallback cb) { log_cb_ = cb; }
  static LogCallback log_cb_;
  static void enqueue_log(const char *msg);
  static std::mutex log_queue_mutex_;
  static std::vector<std::string> log_queue_;

  // v4.0.229: WS EVENT callback — 邊緣觸發 (worker fail / 卡住 / kill switch 變化)
  //   同 log 2-stage 機制：worker task 推 queue → main loop drain → 呼 callback fire EVENT
  //   callback 由 ws_client.cpp 註冊，呼叫 send_event_raw_public 安全發送 (main thread)
  //   Rollback：拿掉 callback 註冊 + enqueue_event 呼叫即可
  using EventCallback = void(*)(const char *json, size_t len);
  static void set_event_callback(EventCallback cb) { event_cb_ = cb; }
  static EventCallback event_cb_;
  static void enqueue_event(const std::string &json);
  static std::mutex event_queue_mutex_;
  static std::vector<std::string> event_queue_;

  // v4.0.229: worker watchdog timestamps (atomic 跨 thread)
  //   worker_entry 取出 request 時寫 last_dispatch_ms_，do_put 結束時寫 last_done_ms_
  //   loop() 每 5s 檢查：若 last_dispatch_ms_ > last_done_ms_ + WORKER_STUCK_MS → 自救 + fire event
  //   stuck_fired_ flag 防 spam (worker 解開後重置)
  // v4.0.279: 60s → 30s + 偵測到自動 soft_reset
  static constexpr uint32_t WORKER_STUCK_MS = 30000;  // v4.0.279: was 60000
  static std::atomic<uint32_t> last_dispatch_ms_;
  static std::atomic<uint32_t> last_done_ms_;
  static std::atomic<bool> stuck_fired_;

  // === v4.0.279 [CL-AUTO-RECOVERY] BEGIN ===
  // Layer 2: soft recovery — 不殺 worker（避免 lwIP mutex deadlock），但清掉狀態
  //   讓下一筆 cl.put 有乾淨環境：cooldown=0、fail_count=0、bump last_done_ms_ 跳過卡點
  //   recovery_attempts_：累計做過幾次（diagnostics 用，並無 cap，純監控）
  static void soft_reset(const char *trigger);
  static std::atomic<uint32_t> recovery_attempts_;
  // Layer 3: 獨立 probe task — 每 3 分鐘 TCP connect 192.168.1.14:80 測通路
  //   3 次連續失敗 → fire cl_probe_unreachable EVENT；成功 → 順手 soft_reset 清歷史狀態
  //   獨立於 worker task → 即使 worker 卡死 probe 仍能跑
  static constexpr uint32_t PROBE_INTERVAL_MS = 180000;  // 3 min
  static constexpr uint8_t PROBE_FAIL_THRESHOLD = 3;
  static TaskHandle_t probe_handle_;
  static void probe_entry(void *arg);
  static std::atomic<uint32_t> probe_last_success_ms_;
  static std::atomic<uint8_t> probe_consecutive_fails_;
  // === v4.0.279 [CL-AUTO-RECOVERY] END ===

  // ── Unreachable cache（target 連續 fail 後冷卻）
  static constexpr uint8_t UNREACH_FAIL_THRESHOLD = 2;
  static constexpr uint32_t UNREACH_COOLDOWN_MS = 15000;  // v4.0.231: 60s → 15s（用戶要求 LED 短暫失聯後快速恢復）
  static constexpr int CONNECT_TIMEOUT_MS = 200;
  static std::atomic<uint8_t> unreach_fail_count_;
  static std::atomic<uint32_t> unreach_until_ms_;

  // ── Kill-switch（YAML switch 可整支關閉）
  static std::atomic<bool> enabled_;
  static void set_enabled(bool e) { enabled_.store(e); }
  static bool is_enabled() { return enabled_.load(); }

  // v4.0.83: 對外查詢冷卻是否仍在進行（Page 1 SN 行 coldDown 後綴用）
  //   true = 目前在 cooldown 中（cl.put 會被 skip），false = 已過冷卻，正常工作
  static bool is_in_cooldown();

  // v4.0.86: 回傳 cooldown 剩餘毫秒（不在 cooldown 時回 0）
  //   HMI / Web 顯示 "CD Ns" 用
  static uint32_t cooldown_remain_ms();

 protected:
  // 常駐 worker task entry（for(;;) xQueueReceive → do_put）
  static void worker_entry(void *arg);
  // 單筆請求處理（connect+send+close），worker 內呼叫
  static void do_put(const ColorlightRequest &req);

  // URL parse → host / port / path
  static bool parse_url(const std::string &url, std::string &host, uint16_t &port, std::string &path);
  // 非阻塞 connect + select(timeout_ms)
  static bool connect_with_timeout(int sock, const void *addr, int addr_len, int timeout_ms);

  // Queue / worker handles
  static QueueHandle_t queue_;
  static TaskHandle_t worker_handle_;
  // v4.0.64: queue 長度 = 1，xQueueOverwrite 實作 latest-wins
  static constexpr UBaseType_t QUEUE_LEN = 1;
};

}  // namespace colorlight
}  // namespace esphome
