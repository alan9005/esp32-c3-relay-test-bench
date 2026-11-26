// colorlight.cpp — HTTP PUT client for Colorlight LED 播放盒
// v4.0.64 — 持久 worker task + queue（根治 lwIP close race）

#include "colorlight.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <cstring>
#include <cstdio>

#include "esp_heap_caps.h"

// ws_client 的全域 breadcrumb（寫 RTC_NOINIT + EEPROM）
extern "C" void set_reboot_tag_c(const char *tag);

namespace esphome {
namespace colorlight {

static const char *TAG = "colorlight";

// ── 靜態成員定義
std::atomic<uint8_t> ColorlightClient::unreach_fail_count_{0};
std::atomic<uint32_t> ColorlightClient::unreach_until_ms_{0};
std::atomic<bool> ColorlightClient::enabled_{true};
ColorlightClient::LogCallback ColorlightClient::log_cb_{nullptr};
std::mutex ColorlightClient::log_queue_mutex_;
std::vector<std::string> ColorlightClient::log_queue_;

// v4.0.229: EVENT callback (邊緣觸發 - cl_put_result fail / cl_worker_stuck / cl_kill_switch_change)
ColorlightClient::EventCallback ColorlightClient::event_cb_{nullptr};
std::mutex ColorlightClient::event_queue_mutex_;
std::vector<std::string> ColorlightClient::event_queue_;
// v4.0.229: worker watchdog
std::atomic<uint32_t> ColorlightClient::last_dispatch_ms_{0};
std::atomic<uint32_t> ColorlightClient::last_done_ms_{0};
std::atomic<bool> ColorlightClient::stuck_fired_{false};
QueueHandle_t ColorlightClient::queue_{nullptr};
TaskHandle_t ColorlightClient::worker_handle_{nullptr};

// === v4.0.279 [CL-AUTO-RECOVERY] BEGIN ===
std::atomic<uint32_t> ColorlightClient::recovery_attempts_{0};
TaskHandle_t ColorlightClient::probe_handle_{nullptr};
std::atomic<uint32_t> ColorlightClient::probe_last_success_ms_{0};
std::atomic<uint8_t> ColorlightClient::probe_consecutive_fails_{0};

// Layer 2: soft_reset — 清狀態讓下一筆 cl.put 走乾淨路徑
//   不殺 worker（避免 lwIP mutex deadlock）；若 worker 真卡死，要等 lwIP socket 內部 timeout
//   但這個 reset 解了「cooldown 鎖住」「fail count 累積」這些「軟性卡住」
//   trigger 標記呼叫來源（watchdog / probe_unreachable / cmd_reset）給 log 追溯
void ColorlightClient::soft_reset(const char *trigger) {
  uint32_t prev_cooldown = unreach_until_ms_.exchange(0);
  uint8_t prev_fails = unreach_fail_count_.exchange(0);
  uint32_t now = millis();
  // bump last_done_ms_ to current — 重置 watchdog 計算基準
  // 若 worker 真卡死，30s 後仍會再次 fire（每 30s 一次自救嘗試）
  last_done_ms_.store(now);
  stuck_fired_.store(false);
  uint32_t attempts = ++recovery_attempts_;

  char log[160];
  snprintf(log, sizeof(log),
    "soft_reset (%s) cd_was=%ums fails_was=%u attempts=%u",
    trigger ? trigger : "?",
    prev_cooldown > now ? (unsigned)(prev_cooldown - now) : 0u,
    (unsigned)prev_fails, (unsigned)attempts);
  enqueue_log(log);

  char ev[200];
  snprintf(ev, sizeof(ev),
    "{\"type\":\"EVENT\",\"event\":\"cl_soft_reset\","
    "\"trigger\":\"%s\",\"attempts\":%u,\"cooldown_cleared_ms\":%u}",
    trigger ? trigger : "?", (unsigned)attempts,
    prev_cooldown > now ? (unsigned)(prev_cooldown - now) : 0u);
  enqueue_event(std::string(ev));
}

// Layer 3: probe_entry — 獨立 task，3 分鐘 TCP connect 測 192.168.1.14:80
//   只測連線 + 立即關閉，不打 HTTP（不需要 auth、不影響 LED 狀態）
//   獨立於 worker → 即使 worker 卡死 probe 仍跑
void ColorlightClient::probe_entry(void * /*arg*/) {
  // 等開機 cooldown 過去
  vTaskDelay(pdMS_TO_TICKS(20000));
  for (;;) {
    if (!enabled_.load()) {
      // kill switch off — 跳過 probe
      vTaskDelay(pdMS_TO_TICKS(PROBE_INTERVAL_MS));
      continue;
    }
    uint32_t t0 = millis();
    int sock = ::socket(AF_INET, SOCK_STREAM, 0);
    bool ok = false;
    int e = 0;
    if (sock >= 0) {
      struct sockaddr_in addr{};
      addr.sin_family = AF_INET;
      addr.sin_port = htons(80);
      addr.sin_addr.s_addr = inet_addr("192.168.1.14");
      ok = connect_with_timeout(sock, &addr, sizeof(addr), 500);  // 500ms timeout
      if (!ok) e = errno;
      ::shutdown(sock, SHUT_RDWR);
      ::close(sock);
    } else {
      e = errno;
      // socket() fail 通常是 EMFILE = socket pool 耗盡 → 嚴重訊號
    }
    uint32_t dt = millis() - t0;
    if (ok) {
      probe_last_success_ms_.store(millis());
      uint8_t prev_fails = probe_consecutive_fails_.exchange(0);
      if (prev_fails > 0) {
        // 從失敗回到成功 → 順手 soft_reset 清歷史 cooldown
        soft_reset("probe_recover");
      }
      char buf[64];
      snprintf(buf, sizeof(buf), "probe OK (%ums)", (unsigned)dt);
      enqueue_log(buf);
    } else {
      uint8_t fails = ++probe_consecutive_fails_;
      char buf[96];
      snprintf(buf, sizeof(buf), "probe FAIL errno=%d (%ums, %u/%u)",
               e, (unsigned)dt, (unsigned)fails, (unsigned)PROBE_FAIL_THRESHOLD);
      enqueue_log(buf);
      if (fails == PROBE_FAIL_THRESHOLD) {
        // 第 N 次失敗 → fire 一次 cl_probe_unreachable EVENT 通知 server
        char ev[200];
        snprintf(ev, sizeof(ev),
          "{\"type\":\"EVENT\",\"event\":\"cl_probe_unreachable\","
          "\"fails\":%u,\"errno\":%d,\"dt_ms\":%u}",
          (unsigned)fails, e, (unsigned)dt);
        enqueue_event(std::string(ev));
      }
    }
    vTaskDelay(pdMS_TO_TICKS(PROBE_INTERVAL_MS));
  }
}
// === v4.0.279 [CL-AUTO-RECOVERY] END ===

void ColorlightClient::enqueue_log(const char *msg) {
  std::lock_guard<std::mutex> l(log_queue_mutex_);
  if (log_queue_.size() < 20) log_queue_.emplace_back(msg);
}

// v4.0.229: thread-safe enqueue from worker task → main loop drain
void ColorlightClient::enqueue_event(const std::string &json) {
  std::lock_guard<std::mutex> l(event_queue_mutex_);
  if (event_queue_.size() < 10) event_queue_.emplace_back(json);
}

// ────────────────────────────────────────────────────────────────
// URL parser
// ────────────────────────────────────────────────────────────────
bool ColorlightClient::parse_url(const std::string &url, std::string &host,
                                  uint16_t &port, std::string &path) {
  const char *prefix = "http://";
  const size_t prefix_len = 7;
  if (url.size() <= prefix_len || url.compare(0, prefix_len, prefix) != 0) return false;
  size_t slash = url.find('/', prefix_len);
  std::string host_port;
  if (slash == std::string::npos) {
    host_port = url.substr(prefix_len);
    path = "/";
  } else {
    host_port = url.substr(prefix_len, slash - prefix_len);
    path = url.substr(slash);
  }
  size_t colon = host_port.find(':');
  if (colon != std::string::npos) {
    host = host_port.substr(0, colon);
    int p = atoi(host_port.substr(colon + 1).c_str());
    if (p <= 0 || p > 65535) return false;
    port = (uint16_t) p;
  } else {
    host = host_port;
    port = 80;
  }
  return !host.empty();
}

bool ColorlightClient::connect_with_timeout(int sock, const void *addr,
                                             int addr_len, int timeout_ms) {
  int flags = fcntl(sock, F_GETFL, 0);
  if (flags < 0) return false;
  if (fcntl(sock, F_SETFL, flags | O_NONBLOCK) < 0) return false;

  int rc = ::connect(sock, (const struct sockaddr *) addr, addr_len);
  if (rc == 0) {
    fcntl(sock, F_SETFL, flags);
    return true;
  }
  if (errno != EINPROGRESS) return false;

  fd_set wfds;
  FD_ZERO(&wfds);
  FD_SET(sock, &wfds);
  struct timeval tv;
  tv.tv_sec = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000;
  int sel = ::select(sock + 1, nullptr, &wfds, nullptr, &tv);
  if (sel <= 0) return false;

  int so_err = 0;
  socklen_t so_err_len = sizeof(so_err);
  if (getsockopt(sock, SOL_SOCKET, SO_ERROR, &so_err, &so_err_len) < 0) return false;
  if (so_err != 0) { errno = so_err; return false; }

  fcntl(sock, F_SETFL, flags);
  return true;
}

// ────────────────────────────────────────────────────────────────
// setup() — 建 queue + 啟常駐 worker task
// v4.0.64: 開機 60s 初始 cooldown（防 .14 offline 下 reboot loop 時 crash）
// ────────────────────────────────────────────────────────────────
// v4.0.83: 對外提供 cooldown 狀態查詢（Page 1 SN 行 coldDown 後綴用）
bool ColorlightClient::is_in_cooldown() {
  uint32_t until = unreach_until_ms_.load();
  if (until == 0) return false;
  return (int32_t)(millis() - until) < 0;
}

// v4.0.86: cooldown 剩餘毫秒（不在 cooldown 時回 0）
uint32_t ColorlightClient::cooldown_remain_ms() {
  uint32_t until = unreach_until_ms_.load();
  if (until == 0) return 0;
  uint32_t now = millis();
  int32_t diff = (int32_t)(until - now);
  return (diff > 0) ? (uint32_t)diff : 0;
}

void ColorlightClient::setup() {
  // v4.0.83: 開機冷卻 60s → 10s（用戶要求降到 10s 觀察穩定性）
  unreach_until_ms_.store(millis() + 10000);

  // 1-slot queue（latest-wins via xQueueOverwrite）
  //   item = ColorlightRequest *（heap-allocated pointer），8KB 負擔最小
  queue_ = xQueueCreate(QUEUE_LEN, sizeof(ColorlightRequest *));
  if (!queue_) {
    ESP_LOGE(TAG, "xQueueCreate failed — cl.put() disabled");
    enabled_.store(false);
    return;
  }

  // 常駐 worker：stack 12288 (排除 stack 嫌疑)、priority 1（低於主 loop/WS）
  BaseType_t ok = xTaskCreate(&ColorlightClient::worker_entry, "cl_worker",
                               12288, nullptr, 1, &worker_handle_);
  if (ok != pdPASS) {
    ESP_LOGE(TAG, "worker xTaskCreate failed — cl.put() disabled");
    enabled_.store(false);
    return;
  }
  ESP_LOGI(TAG, "v4.0.64 worker ready (queue=%u, unreach cooldown=%us)",
           (unsigned)QUEUE_LEN, (unsigned)(UNREACH_COOLDOWN_MS / 1000));

  // === v4.0.279 [CL-AUTO-RECOVERY] BEGIN ===
  // Layer 3: 啟動獨立 probe task — 與 worker 隔離，能在 worker 卡死時繼續監測連通性
  BaseType_t probe_ok = xTaskCreate(&ColorlightClient::probe_entry, "cl_probe",
                                     4096, nullptr, 1, &probe_handle_);
  if (probe_ok != pdPASS) {
    ESP_LOGW(TAG, "probe xTaskCreate failed — auto-recovery layer 3 disabled");
  } else {
    ESP_LOGI(TAG, "v4.0.279 probe task ready (interval=%us, fail_threshold=%u)",
             (unsigned)(PROBE_INTERVAL_MS / 1000), (unsigned)PROBE_FAIL_THRESHOLD);
  }
  // === v4.0.279 [CL-AUTO-RECOVERY] END ===
}

// loop() — drain log queue（task-safe 2-stage log）+ event queue (v4.0.229)
//          + worker watchdog 5s tick (cl_worker_stuck event 一次)
void ColorlightClient::loop() {
  // log
  if (log_cb_) {
    std::vector<std::string> drain;
    {
      std::lock_guard<std::mutex> l(log_queue_mutex_);
      drain.swap(log_queue_);
    }
    for (auto &s : drain) log_cb_(s.c_str());
  }
  // v4.0.229: event drain
  if (event_cb_) {
    std::vector<std::string> drain;
    {
      std::lock_guard<std::mutex> l(event_queue_mutex_);
      drain.swap(event_queue_);
    }
    for (auto &s : drain) event_cb_(s.c_str(), s.size());
  }
  // v4.0.229: worker watchdog (每 5s 檢查一次節流)
  // v4.0.279: 60s → WORKER_STUCK_MS(30s) + 偵測到自動 soft_reset
  static uint32_t last_wd_check_ms = 0;
  uint32_t now = millis();
  if ((uint32_t)(now - last_wd_check_ms) >= 5000) {
    last_wd_check_ms = now;
    uint32_t disp = last_dispatch_ms_.load();
    uint32_t done = last_done_ms_.load();
    // dispatch 有值 + done < dispatch (還沒結束) + 卡 > WORKER_STUCK_MS
    if (disp != 0 && done < disp && (uint32_t)(now - disp) > WORKER_STUCK_MS && !stuck_fired_.load()) {
      stuck_fired_.store(true);
      char ev[160];
      snprintf(ev, sizeof(ev),
        "{\"type\":\"EVENT\",\"event\":\"cl_worker_stuck\","
        "\"stuck_ms\":%u,\"dispatch_ms\":%u}",
        (unsigned)(now - disp), disp);
      enqueue_event(std::string(ev));
      char lbuf[120];
      snprintf(lbuf, sizeof(lbuf), "worker STUCK > %us — auto soft_reset",
               (unsigned)(WORKER_STUCK_MS / 1000));
      enqueue_log(lbuf);
      // v4.0.279 Layer 2: 自動 soft recovery — 清 cooldown / fail_count，
      //   讓下一筆 cl.put 有乾淨環境（不殺 worker，避免 lwIP mutex 風險）
      //   若 worker 仍卡死，下一個 30s 又會 fire（stuck_fired_ 在 last_done_ms_ 變動才會重置）
      //   為了讓下一次 watchdog 能再觸發，soft_reset 內 bump last_done_ms_ 重置基準
      soft_reset("watchdog");
    }
  }
}

// ────────────────────────────────────────────────────────────────
// put() — 主 loop 呼叫，alloc req 後 xQueueOverwrite（非阻塞）
// ────────────────────────────────────────────────────────────────
void ColorlightClient::put(const std::string &url, const std::string &auth,
                           const std::string &content_type, const std::string &body) {
  // v4.0.229: skip 路徑 fire cl_put_result EVENT 通知 server (server 才知道為什麼 LED 沒切)
  auto fire_skip_event = [&](const char *reason) {
    char ev[256];
    snprintf(ev, sizeof(ev),
      "{\"type\":\"EVENT\",\"event\":\"cl_put_result\",\"ok\":false,"
      "\"reason\":\"%s\",\"url\":\"%s\"}",
      reason, url.c_str());
    enqueue_event(std::string(ev));
  };
  if (!enabled_.load()) {
    ESP_LOGD(TAG, "disabled — skip put %s", url.c_str());
    fire_skip_event("kill_switch_off");
    return;
  }
  if (!queue_) return;  // setup 失敗

  uint32_t now = millis();

  // Unreachable cooldown
  uint32_t unreach_until = unreach_until_ms_.load();
  if (unreach_until != 0 && (int32_t)(now - unreach_until) < 0) {
    ESP_LOGD(TAG, "unreachable cooldown: skip %s", url.c_str());
    fire_skip_event("cooldown");
    return;
  }

  // Heap gate 40KB
  size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (free_heap < 40000) {
    ESP_LOGW(TAG, "heap low (%u) — skip put %s", (unsigned)free_heap, url.c_str());
    char lbuf[96];
    snprintf(lbuf, sizeof(lbuf), "heap LOW %u → skip %s", (unsigned)free_heap, url.c_str());
    enqueue_log(lbuf);
    fire_skip_event("heap_low");
    return;
  }

  // v4.0.105: method 預設 "PUT"（向後相容既有 caller）
  auto *req = new (std::nothrow) ColorlightRequest{url, auth, content_type, body, "PUT"};
  if (!req) {
    ESP_LOGW(TAG, "alloc failed (heap low): %s", url.c_str());
    return;
  }

  // v4.0.64: xQueueOverwrite → 1-slot latest-wins
  //   queue 空 → 投入；queue 滿 → 覆蓋舊 req（worker 拿到新值，舊值我們接回來 delete）
  ColorlightRequest *old = nullptr;
  if (xQueuePeek(queue_, &old, 0) == pdTRUE) {
    // 有舊請求 → receive 拿回來 delete 後再投新
    ColorlightRequest *evicted = nullptr;
    if (xQueueReceive(queue_, &evicted, 0) == pdTRUE && evicted) {
      char lbuf[128];
      snprintf(lbuf, sizeof(lbuf), "queue OVERWRITE old=%s new=%s",
               evicted->url.c_str(), req->url.c_str());
      enqueue_log(lbuf);
      delete evicted;
    }
  }
  if (xQueueSend(queue_, &req, 0) != pdTRUE) {
    ESP_LOGW(TAG, "xQueueSend fail: %s", url.c_str());
    delete req;
  }
}

// ────────────────────────────────────────────────────────────────
// worker_entry — 常駐 task，永不退出
// ────────────────────────────────────────────────────────────────
void ColorlightClient::worker_entry(void * /*arg*/) {
  ESP_LOGI(TAG, "worker started");
  for (;;) {
    ColorlightRequest *req = nullptr;
    // 阻塞等請求（無請求時 task 休眠、CPU 0%）
    if (xQueueReceive(queue_, &req, portMAX_DELAY) != pdTRUE) continue;
    if (!req) continue;

    // v4.0.229: watchdog timestamp 取出時寫 dispatch
    last_dispatch_ms_.store(millis());
    do_put(*req);
    last_done_ms_.store(millis());  // do_put 完成
    stuck_fired_.store(false);       // 解開重置 (允許下次卡住再 fire)
    delete req;
    // → loop 回到 xQueueReceive，task 永不死
  }
}

// ────────────────────────────────────────────────────────────────
// do_put — 單筆 connect → send → close
//   所有退出路徑都**不退 task**，只 return 回 worker loop。
//   lwIP async callback（SYN timer 取消等）在我們下一輪 xQueueReceive
//   阻塞期間觸發 → worker task 仍活著，PCB 引用安全。
// ────────────────────────────────────────────────────────────────
void ColorlightClient::do_put(const ColorlightRequest &req) {
  uint32_t task_start_ms = millis();
  set_reboot_tag_c("CL_DO_PUT");

  set_reboot_tag_c("CL_PARSE_URL");
  std::string host, path;
  uint16_t port = 80;
  if (!parse_url(req.url, host, port, path)) {
    ESP_LOGE(TAG, "Bad URL: %s", req.url.c_str());
    return;
  }

  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);

  set_reboot_tag_c("CL_RESOLVE");
  in_addr_t ip = inet_addr(host.c_str());
  if (ip == INADDR_NONE) {
    set_reboot_tag_c("CL_DNS");
    struct hostent *he = gethostbyname(host.c_str());
    if (!he || he->h_length != 4 || he->h_addr_list[0] == nullptr) {
      ESP_LOGW(TAG, "DNS fail: %s", host.c_str());
      return;
    }
    memcpy(&addr.sin_addr, he->h_addr_list[0], 4);
  } else {
    addr.sin_addr.s_addr = ip;
  }

  set_reboot_tag_c("CL_SOCKET");
  int sock = ::socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0) {
    ESP_LOGE(TAG, "socket() fail: errno=%d", errno);
    return;
  }

  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 200000;
  setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  set_reboot_tag_c("CL_CONNECT");
  if (!connect_with_timeout(sock, &addr, sizeof(addr), CONNECT_TIMEOUT_MS)) {
    int e = errno;
    uint8_t fails = ++unreach_fail_count_;
    if (fails >= UNREACH_FAIL_THRESHOLD) {
      unreach_until_ms_.store(millis() + UNREACH_COOLDOWN_MS);
      ESP_LOGI(TAG, "unreachable × %u → cooldown %us",
               (unsigned)fails, (unsigned)(UNREACH_COOLDOWN_MS / 1000));
    } else {
      ESP_LOGI(TAG, "connect %s:%u fail errno=%d (fails=%u)",
               host.c_str(), port, e, (unsigned)fails);
    }
    char buf[128];
    snprintf(buf, sizeof(buf), "connect FAIL %s:%u errno=%d (%ums, fails=%u)",
             host.c_str(), port, e, (unsigned)(millis() - task_start_ms),
             (unsigned)fails);
    enqueue_log(buf);

    // v4.0.229: fire cl_put_result EVENT (失敗時才送 — 成功完全靜默省頻寬)
    {
      char ev[256];
      bool in_cd = (fails >= UNREACH_FAIL_THRESHOLD);
      snprintf(ev, sizeof(ev),
        "{\"type\":\"EVENT\",\"event\":\"cl_put_result\",\"ok\":false,"
        "\"reason\":\"%s\",\"host\":\"%s\",\"port\":%u,\"errno\":%d,\"fails\":%u}",
        in_cd ? "connect_timeout_cooldown" : "connect_timeout",
        host.c_str(), port, e, (unsigned)fails);
      enqueue_event(std::string(ev));
    }

    set_reboot_tag_c("CL_RESTORE_BLOCK");
    int cur_flags = fcntl(sock, F_GETFL, 0);
    if (cur_flags >= 0) fcntl(sock, F_SETFL, cur_flags & ~O_NONBLOCK);
    set_reboot_tag_c("CL_SHUTDOWN");
    ::shutdown(sock, SHUT_RDWR);
    struct linger lg;
    lg.l_onoff = 1;
    lg.l_linger = 0;
    setsockopt(sock, SOL_SOCKET, SO_LINGER, &lg, sizeof(lg));
    set_reboot_tag_c("CL_CLOSE");
    ::close(sock);
    set_reboot_tag_c("CL_CLOSED_OK");
    return;  // 回 worker loop → xQueueReceive 阻塞等待（task 活著）
  }
  unreach_fail_count_.store(0);
  unreach_until_ms_.store(0);

  set_reboot_tag_c("CL_BUILD_FRAME");
  // v4.0.105: 動態 HTTP method (空字串 = PUT 向後相容)
  const std::string &method = req.method.empty() ? std::string("PUT") : req.method;
  std::string frame;
  frame.reserve(256 + req.body.size());
  frame.append(method).append(" ").append(path).append(" HTTP/1.1\r\n");
  frame.append("Host: ").append(host);
  if (port != 80) {
    char pbuf[8];
    snprintf(pbuf, sizeof(pbuf), ":%u", port);
    frame.append(pbuf);
  }
  frame.append("\r\n");
  frame.append("User-Agent: HuiFlow/1.0\r\n");
  if (!req.auth.empty())
    frame.append("Authorization: ").append(req.auth).append("\r\n");
  if (!req.content_type.empty())
    frame.append("Content-Type: ").append(req.content_type).append("\r\n");
  {
    char lbuf[40];
    snprintf(lbuf, sizeof(lbuf), "Content-Length: %u\r\n", (unsigned) req.body.size());
    frame.append(lbuf);
  }
  frame.append("Connection: close\r\n\r\n");
  if (!req.body.empty()) frame.append(req.body);

  set_reboot_tag_c("CL_SEND");
  const char *p = frame.data();
  size_t left = frame.size();
  bool send_ok = true;
  int send_errno = 0;
  while (left > 0) {
    ssize_t n = ::send(sock, p, left, 0);
    if (n <= 0) {
      send_errno = errno;
      ESP_LOGW(TAG, "send fail (left=%u, errno=%d): %s",
               (unsigned) left, send_errno, req.url.c_str());
      send_ok = false;
      break;
    }
    p += n;
    left -= (size_t) n;
  }

  set_reboot_tag_c("CL_CLOSE_OK_PATH");
  ::close(sock);
  set_reboot_tag_c("CL_DONE");
  uint32_t elapsed = millis() - task_start_ms;
  ESP_LOGD(TAG, "%s %s done (%u bytes, %ums)",
           method.c_str(), req.url.c_str(), (unsigned) frame.size(), (unsigned)elapsed);

  char buf[128];
  if (send_ok) {
    snprintf(buf, sizeof(buf), "%s OK (%ums,%ub) %s",
             method.c_str(), (unsigned)elapsed, (unsigned)frame.size(), path.c_str());
  } else {
    snprintf(buf, sizeof(buf), "send FAIL left=%u errno=%d (%ums) url=%s",
             (unsigned)left, send_errno, (unsigned)elapsed, path.c_str());
    // v4.0.229: send fail 也 fire cl_put_result event (TCP write 卡 / kernel buffer 滿)
    char ev[256];
    snprintf(ev, sizeof(ev),
      "{\"type\":\"EVENT\",\"event\":\"cl_put_result\",\"ok\":false,"
      "\"reason\":\"send_fail\",\"left\":%u,\"errno\":%d,\"path\":\"%s\"}",
      (unsigned)left, send_errno, path.c_str());
    enqueue_event(std::string(ev));
  }
  enqueue_log(buf);
  // return → worker loop 回 xQueueReceive
}

}  // namespace colorlight
}  // namespace esphome
