// ble_adv.cpp — BLE Extended Advertising + 自簽編號（NimBLE）
//
// v4.0.287 — 升級自 v4.0.283 legacy adv：
//   - BLE 5.0 Extended Adv（ble_gap_ext_adv_*），payload 上限大幅提升
//   - 自簽編號安全層：boot_id(2) + seq(2) + HMAC-SHA256-trunc(8) = 12B overhead
//   - Static Random MAC（從 device_sn 推導，每台穩定）
//   - 保留 v4.0.284 kill switch / singleton / idle heartbeat
//
// 編譯需求（iot-washer-base.yaml esp32: sdkconfig_options）：
//   CONFIG_BT_ENABLED: y
//   CONFIG_BT_NIMBLE_ENABLED: y
//   CONFIG_BT_NIMBLE_EXT_ADV: y          ← v4.0.287 新增（啟用 ext adv API）
//   CONFIG_BT_NIMBLE_ROLE_BROADCASTER: y
//
// rollback：移除 my_components/ble_adv/ + YAML ble_adv: 區塊 + 上述 sdkconfig

#include "ble_adv.h"

#ifdef USE_ESP32

#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <cstring>

// NimBLE host headers
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "host/ble_gap.h"
#include "os/os_mbuf.h"

// mbedtls — HMAC-SHA256（自簽編號）+ SHA256（推導 Static Random MAC）
#include "mbedtls/md.h"

namespace esphome {
namespace ble_adv {

static const char *const TAG = "ble_adv";

BleAdv *BleAdv::instance_ = nullptr;

// === v4.0.284 [BLE-KILL-SWITCH] ===
std::atomic<bool> BleAdv::enabled_{true};

// ext adv 非連線/非掃描 instance 的 gap event callback（事件極少，直接回 0）
static int ble_adv_gap_event(struct ble_gap_event *event, void *arg) {
  (void) event;
  (void) arg;
  return 0;
}

void BleAdv::set_enabled(bool e) {
  bool prev = BleAdv::enabled_.exchange(e);
  if (prev == e) return;
  ESP_LOGI(TAG, "set_enabled %s → %s", prev ? "ON" : "OFF", e ? "ON" : "OFF");
  if (BleAdv::instance_ == nullptr || !BleAdv::instance_->ready_.load()) return;
  if (e) {
    BleAdv::instance_->publish_idle();  // re-enable：重啟 adv
  } else {
    // disable：立刻停止 ext adv，sticky 直到 set_enabled(true)
    if (ble_gap_ext_adv_active(BleAdv::EXT_ADV_INSTANCE)) {
      int rc = ble_gap_ext_adv_stop(BleAdv::EXT_ADV_INSTANCE);
      if (rc == 0) {
        BleAdv::instance_->adv_active_.store(false);
        ESP_LOGI(TAG, "BLE ext adv stopped (kill switch)");
      } else {
        ESP_LOGW(TAG, "ble_gap_ext_adv_stop rc=%d", rc);
      }
    }
  }
}

// ─────────────────────────────────────────────────────────────
// setup()
// ─────────────────────────────────────────────────────────────
void BleAdv::setup() {
  ESP_LOGI(TAG, "BleAdv setup() — NimBLE ext-adv host stack");
  BleAdv::instance_ = this;

  esp_err_t ret = nimble_port_init();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "nimble_port_init fail: %d", ret);
    return;
  }

  ble_hs_cfg.sync_cb = BleAdv::on_sync_cb;
  ble_hs_cfg.reset_cb = BleAdv::on_reset_cb;

  nimble_port_freertos_init(BleAdv::nimble_host_task);

  // 先備好 idle payload（sync_cb 後才真正 configure + start）
  this->publish_idle();

  ESP_LOGI(TAG, "BleAdv setup() done, waiting for NimBLE sync...");
}

void BleAdv::nimble_host_task(void *arg) {
  ESP_LOGI(TAG, "NimBLE host task started");
  nimble_port_run();
  nimble_port_freertos_deinit();
}

void BleAdv::on_sync_cb() {
  ESP_LOGI(TAG, "NimBLE sync — ext-adv broadcaster ready");
  if (BleAdv::instance_ == nullptr) return;
  // 設 Static Random MAC（從 SN 推導，每台穩定）
  BleAdv::instance_->set_static_random_addr_();
  BleAdv::instance_->ready_.store(true);
  BleAdv::instance_->update_adv_data_();
}

void BleAdv::on_reset_cb(int reason) {
  ESP_LOGW(TAG, "NimBLE reset, reason=%d", reason);
  if (BleAdv::instance_ != nullptr) {
    BleAdv::instance_->ready_.store(false);
    BleAdv::instance_->adv_active_.store(false);
    BleAdv::instance_->configured_.store(false);  // reset 後需重新 configure
  }
}

// ─────────────────────────────────────────────────────────────
// Static Random MAC — sha256("HFBLE-MAC" + sn) 取 6 byte，頂 2 bit = 0b11
// ─────────────────────────────────────────────────────────────
void BleAdv::set_static_random_addr_() {
  uint8_t seed[16];
  size_t n = 0;
  const char *salt = "HFBLE-MAC";
  for (const char *p = salt; *p && n < sizeof(seed); ++p) seed[n++] = (uint8_t) *p;
  for (int i = 0; i < 4 && n < sizeof(seed); i++) seed[n++] = this->device_sn_[i];

  uint8_t h[32];
  const mbedtls_md_info_t *info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
  if (info != nullptr && mbedtls_md(info, seed, n, h) == 0) {
    memcpy(this->rnd_addr_, h, 6);
  } else {
    // fallback（SHA256 不可用時，理論上不會發生）：直接用 sn 填
    memcpy(this->rnd_addr_, this->device_sn_, 4);
    this->rnd_addr_[4] = 0xA5;
    this->rnd_addr_[5] = 0x5A;
  }
  // Static Random Address：最高 byte 的最高 2 bit 必須是 0b11
  this->rnd_addr_[5] |= 0xC0;

  int rc = ble_hs_id_set_rnd(this->rnd_addr_);
  if (rc != 0) {
    ESP_LOGW(TAG, "ble_hs_id_set_rnd rc=%d", rc);
    return;
  }
  this->rnd_addr_set_ = true;
  ESP_LOGI(TAG, "Static Random MAC set %02X:%02X:%02X:%02X:%02X:%02X",
           this->rnd_addr_[5], this->rnd_addr_[4], this->rnd_addr_[3],
           this->rnd_addr_[2], this->rnd_addr_[1], this->rnd_addr_[0]);
}

// ─────────────────────────────────────────────────────────────
// payload 構建（schema v2 + 自簽編號 trailer）
// ─────────────────────────────────────────────────────────────
size_t BleAdv::build_payload_(uint8_t cmd_id, uint8_t target_mask,
                              const uint8_t *args, size_t args_len) {
  size_t off = 0;
  // [2B company_id LE]
  this->mfg_data_[off++] = (uint8_t)(COMPANY_ID & 0xFF);
  this->mfg_data_[off++] = (uint8_t)(COMPANY_ID >> 8);
  // [4B sn]
  for (int i = 0; i < 4; i++) this->mfg_data_[off++] = this->device_sn_[i];
  // [1B schema_ver]
  this->mfg_data_[off++] = SCHEMA_VER;
  // [1B target_mask][1B cmd_id]
  this->mfg_data_[off++] = target_mask;
  this->mfg_data_[off++] = cmd_id;
  // [2B boot_id LE]
  this->mfg_data_[off++] = (uint8_t)(this->boot_id_ & 0xFF);
  this->mfg_data_[off++] = (uint8_t)(this->boot_id_ >> 8);
  // [2B seq LE]
  uint16_t seq = (uint16_t)(this->seq_.fetch_add(1) & 0xFFFF);
  this->mfg_data_[off++] = (uint8_t)(seq & 0xFF);
  this->mfg_data_[off++] = (uint8_t)(seq >> 8);

  // [N B args]（保留末 8 byte 給 HMAC）
  size_t remain = MFG_DATA_MAX - off - 8;
  size_t copy_len = (args_len > remain) ? remain : args_len;
  if (copy_len > 0 && args != nullptr) {
    memcpy(&this->mfg_data_[off], args, copy_len);
    off += copy_len;
  }

  // [8B HMAC-SHA256(PSK, bytes[0..off)) 取前 8]（psk 空 → 全 0，子板放行）
  uint8_t mac[32] = {0};
  if (this->psk_len_ > 0) {
    const mbedtls_md_info_t *info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
    if (info != nullptr) {
      mbedtls_md_hmac(info, this->psk_, this->psk_len_, this->mfg_data_, off, mac);
    }
  }
  memcpy(&this->mfg_data_[off], mac, 8);
  off += 8;

  this->mfg_data_len_ = off;
  return off;
}

// ─────────────────────────────────────────────────────────────
// publish_cmd / publish_idle
// ─────────────────────────────────────────────────────────────
void BleAdv::publish_cmd(uint8_t target_mask, uint8_t cmd_id,
                         const uint8_t *args, size_t args_len) {
  if (!BleAdv::enabled_.load()) return;
  this->build_payload_(cmd_id, target_mask, args, args_len);
  this->cmd_until_ms_.store(millis() + CMD_BROADCAST_MS);
  this->publish_count_.fetch_add(1);
  if (this->ready_.load()) this->update_adv_data_();
}

void BleAdv::publish_idle() {
  if (!BleAdv::enabled_.load()) return;
  uint32_t up_s = millis() / 1000;
  uint8_t args[4] = {
      (uint8_t)(up_s & 0xFF), (uint8_t)((up_s >> 8) & 0xFF),
      (uint8_t)((up_s >> 16) & 0xFF), (uint8_t)((up_s >> 24) & 0xFF),
  };
  this->build_payload_(CMD_IDLE_HEARTBEAT, TARGET_ALL, args, sizeof(args));
  this->last_idle_ms_.store(millis());
  if (this->ready_.load()) this->update_adv_data_();
}

// ─────────────────────────────────────────────────────────────
// v4.0.288 [BLE-PAIR] publish_pairing — 廣播 cmd=PAIRING 含 PSK，持續 window_ms
// ─────────────────────────────────────────────────────────────
void BleAdv::publish_pairing(uint32_t window_ms, const char *wifi_ssid, const char *wifi_pass, const char *role, const char *store_sn) {
  if (!BleAdv::enabled_.load()) return;
  // v4.0.301: args = PSK [\x1f ssid [\x1f pass [\x1f role]]]。子板驗 HMAC,過則學 SN+PSK+WiFi(+role 配對即開通)。
  uint8_t buf[160];
  size_t off = 0;
  size_t pl = this->psk_len_ > sizeof(buf) ? sizeof(buf) : this->psk_len_;
  memcpy(buf, this->psk_, pl); off = pl;
  bool has_wifi = (wifi_ssid != nullptr && wifi_ssid[0] != '\0');
  if (has_wifi) {
    if (off < sizeof(buf)) buf[off++] = 0x1F;
    for (const char *p = wifi_ssid; *p && off < sizeof(buf); ++p) buf[off++] = (uint8_t) *p;
    if (off < sizeof(buf)) buf[off++] = 0x1F;
    if (wifi_pass) for (const char *p = wifi_pass; *p && off < sizeof(buf); ++p) buf[off++] = (uint8_t) *p;
    bool has_role = (role != nullptr && role[0] != '\0');
    bool has_store = (store_sn != nullptr && store_sn[0] != '\0');
    if (has_role || has_store) {                 // 第4段:角色(配對即開通)
      if (off < sizeof(buf)) buf[off++] = 0x1F;
      if (has_role) for (const char *p = role; *p && off < sizeof(buf); ++p) buf[off++] = (uint8_t) *p;
      if (has_store) {                           // 第5段:主板全 SN(店家識別碼,回報雲端)
        if (off < sizeof(buf)) buf[off++] = 0x1F;
        for (const char *p = store_sn; *p && off < sizeof(buf); ++p) buf[off++] = (uint8_t) *p;
      }
    }
  }
  this->build_payload_(CMD_PAIRING, TARGET_ALL, buf, off);
  this->cmd_until_ms_.store(millis() + window_ms);
  this->publish_count_.fetch_add(1);
  if (this->ready_.load()) this->update_adv_data_();
  ESP_LOGI(TAG, "PAIRING broadcast %u ms (psk %u + wifi=%d, total %u bytes)",
           (unsigned) window_ms, (unsigned) this->psk_len_, has_wifi ? 1 : 0, (unsigned) off);
}

// ─────────────────────────────────────────────────────────────
// update_adv_data_() — configure(一次) + set_data + start（ext adv）
// ─────────────────────────────────────────────────────────────
void BleAdv::update_adv_data_() {
  // 1. configure ext adv instance（只做一次）
  if (!this->configured_.load()) {
    struct ble_gap_ext_adv_params params;
    memset(&params, 0, sizeof(params));
    params.connectable = 0;
    params.scannable = 0;
    params.legacy_pdu = 0;                 // 用 extended PDU
    params.own_addr_type = BLE_OWN_ADDR_RANDOM;
    params.primary_phy = 1;                // 1M PHY
    params.secondary_phy = 1;              // 1M PHY
    params.itvl_min = 48;                  // 48 × 0.625ms = 30ms
    params.itvl_max = 96;                  // 96 × 0.625ms = 60ms
    params.sid = 0;
    params.tx_power = 127;                 // no preference

    int rc = ble_gap_ext_adv_configure(EXT_ADV_INSTANCE, &params, NULL,
                                       ble_adv_gap_event, NULL);
    if (rc != 0) {
      ESP_LOGW(TAG, "ext_adv_configure rc=%d", rc);
      return;
    }
    // 設 instance 的 random address
    if (this->rnd_addr_set_) {
      ble_addr_t addr;
      addr.type = BLE_ADDR_RANDOM;
      memcpy(addr.val, this->rnd_addr_, 6);
      rc = ble_gap_ext_adv_set_addr(EXT_ADV_INSTANCE, &addr);
      if (rc != 0) ESP_LOGW(TAG, "ext_adv_set_addr rc=%d", rc);
    }
    this->configured_.store(true);
  }

  // 2. 組 adv data bytes：[flags AD] + [mfg AD]
  uint8_t adv[3 + 2 + MFG_DATA_MAX];
  size_t a = 0;
  // Flags AD：len=2, type=0x01, value=GEN_DISC|BREDR_UNSUP
  adv[a++] = 0x02;
  adv[a++] = 0x01;
  adv[a++] = 0x06;
  // Manufacturer Specific Data AD：len=1+mfg, type=0xFF
  adv[a++] = (uint8_t)(1 + this->mfg_data_len_);
  adv[a++] = 0xFF;
  memcpy(&adv[a], this->mfg_data_, this->mfg_data_len_);
  a += this->mfg_data_len_;

  // 3. set_data（用 os_mbuf 包）。更新時若 active 先 stop 避免 EBUSY
  bool was_active = ble_gap_ext_adv_active(EXT_ADV_INSTANCE);
  if (was_active) {
    ble_gap_ext_adv_stop(EXT_ADV_INSTANCE);
  }

  struct os_mbuf *om = os_msys_get_pkthdr(a, 0);
  if (om == nullptr) {
    ESP_LOGW(TAG, "os_msys_get_pkthdr null (a=%u)", (unsigned) a);
    return;
  }
  int rc = os_mbuf_append(om, adv, a);
  if (rc != 0) {
    ESP_LOGW(TAG, "os_mbuf_append rc=%d", rc);
    os_mbuf_free_chain(om);
    return;
  }
  rc = ble_gap_ext_adv_set_data(EXT_ADV_INSTANCE, om);  // 成功會消化 om
  if (rc != 0) {
    ESP_LOGW(TAG, "ext_adv_set_data rc=%d (a=%u)", rc, (unsigned) a);
    return;
  }

  // 4. start（duration=0 forever, max_events=0 unlimited）
  rc = ble_gap_ext_adv_start(EXT_ADV_INSTANCE, 0, 0);
  if (rc != 0) {
    ESP_LOGW(TAG, "ext_adv_start rc=%d", rc);
    return;
  }
  this->adv_active_.store(true);
}

// ─────────────────────────────────────────────────────────────
// loop() — cmd adv 過期 → idle；idle 每 30s 刷新
// ─────────────────────────────────────────────────────────────
void BleAdv::loop() {
  uint32_t now = millis();
  uint32_t cmd_until = this->cmd_until_ms_.load();
  if (cmd_until != 0 && now >= cmd_until) {
    this->cmd_until_ms_.store(0);
    this->publish_idle();
    return;
  }
  if (cmd_until == 0) {
    uint32_t last = this->last_idle_ms_.load();
    if (now - last >= IDLE_INTERVAL_MS) this->publish_idle();
  }
}

// ─────────────────────────────────────────────────────────────
// set_device_serial — 4 位 ASCII → 4 bytes
// ─────────────────────────────────────────────────────────────
void BleAdv::set_device_serial(const char *sn_str) {
  if (sn_str == nullptr) {
    memset(this->device_sn_, 0, 4);
    return;
  }
  for (int i = 0; i < 4; i++) {
    char c = sn_str[i];
    if (c == '\0') {
      for (int j = i; j < 4; j++) this->device_sn_[j] = 0;
      break;
    }
    this->device_sn_[i] = (uint8_t) c;
  }
}

// ─────────────────────────────────────────────────────────────
// set_psk — per-store HMAC key（raw 字串，最多 32 byte）
// ─────────────────────────────────────────────────────────────
void BleAdv::set_psk(const char *psk_str) {
  if (psk_str == nullptr) {
    this->psk_len_ = 0;
    return;
  }
  size_t n = strlen(psk_str);
  if (n > sizeof(this->psk_)) n = sizeof(this->psk_);
  memcpy(this->psk_, psk_str, n);
  this->psk_len_ = n;
  ESP_LOGI(TAG, "PSK set (%u bytes)", (unsigned) n);
}

}  // namespace ble_adv
}  // namespace esphome

#endif  // USE_ESP32
