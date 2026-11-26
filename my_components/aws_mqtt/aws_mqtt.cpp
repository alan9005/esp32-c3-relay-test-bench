#include "aws_mqtt.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

#include <cstring>
#include <cstdlib>
#include <ctime>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

extern "C" {
#include "mbedtls/error.h"
}

namespace esphome {
namespace aws_mqtt {

static const char *TAG = "aws_mqtt";

static const uint32_t MAX_FAST_RETRIES = 10;
static const uint32_t SLOW_RETRY_INTERVAL_MS = 60000;

// MQTT packet types
static const uint8_t MQTT_CONNECT     = 0x10;
static const uint8_t MQTT_CONNACK     = 0x20;
static const uint8_t MQTT_PUBLISH     = 0x30;
static const uint8_t MQTT_SUBSCRIBE   = 0x82;
static const uint8_t MQTT_SUBACK      = 0x90;
static const uint8_t MQTT_PINGREQ     = 0xC0;
static const uint8_t MQTT_PINGRESP    = 0xD0;
static const uint8_t MQTT_DISCONNECT  = 0xE0;

// =========================================================================
// mbedtls BIO callbacks
// =========================================================================

static int mbed_send_cb(void *ctx, const unsigned char *buf, size_t len) {
  int fd = *(int *)ctx;
  ssize_t ret = ::send(fd, buf, len, 0);
  if (ret < 0) {
    int err = errno;
    if (err == EAGAIN || err == EWOULDBLOCK) return MBEDTLS_ERR_SSL_WANT_WRITE;
    return -1;
  }
  return (int)ret;
}

static int mbed_recv_cb(void *ctx, unsigned char *buf, size_t len) {
  int fd = *(int *)ctx;
  ssize_t ret = ::recv(fd, buf, len, 0);
  if (ret < 0) {
    int err = errno;
    if (err == EAGAIN || err == EWOULDBLOCK) return MBEDTLS_ERR_SSL_WANT_READ;
    return -1;
  }
  return (int)ret;
}

// =========================================================================
// FreeRTOS background connect task
// =========================================================================

void AWSMqttComponent::connect_task_fn(void *param) {
  AWSMqttComponent *self = static_cast<AWSMqttComponent *>(param);
  bool ok = self->do_connect();
  self->connect_state_ = ok ? MCS_CONNECTED : MCS_FAILED;
  self->connect_task_handle_ = nullptr;
  vTaskDelete(nullptr);
}

// =========================================================================
// HMAC-SHA256 credential generation
// =========================================================================

void AWSMqttComponent::generate_credentials(std::string &username, std::string &password) {
  time_t now_ts;
  time(&now_ts);
  uint32_t timestamp = (uint32_t)now_ts;

  // If NTP not synced, use millis-based pseudo timestamp
  if (timestamp < 1600000000) {
    timestamp = 1700000000 + (millis() / 1000);
  }

  // username_base = "{tenant_id}-{timestamp}-{nonce}"
  char username_base[256];
  snprintf(username_base, sizeof(username_base), "%s-%u-%s",
           this->tenant_id_.c_str(), timestamp, this->nonce_.c_str());

  // HMAC-SHA256
  unsigned char hmac_result[32];
  mbedtls_md_context_t ctx;
  mbedtls_md_init(&ctx);
  mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), 1);
  mbedtls_md_hmac_starts(&ctx,
                          (const unsigned char *)this->secret_key_.c_str(),
                          this->secret_key_.length());
  mbedtls_md_hmac_update(&ctx,
                          (const unsigned char *)username_base,
                          strlen(username_base));
  mbedtls_md_hmac_finish(&ctx, hmac_result);
  mbedtls_md_free(&ctx);

  // Base64 encode
  unsigned char b64[64];
  size_t b64_len = 0;
  mbedtls_base64_encode(b64, sizeof(b64), &b64_len, hmac_result, 32);

  // username = "{username_base}?x-amz-customauthorizer-name={auth_name}"
  username = std::string(username_base) + "?x-amz-customauthorizer-name=" + this->auth_name_;
  password = std::string((char *)b64, b64_len);

  ESP_LOGI(TAG, "Credentials generated: ts=%u user_base=%s", timestamp, username_base);
}

// =========================================================================
// Setup / Manual Connect / Disconnect
// =========================================================================

void AWSMqttComponent::setup() {
  this->setup_time_ = millis();
  ESP_LOGI(TAG, "AWS MQTT ready, host: %s:%d (manual connect)", this->endpoint_.c_str(), this->port_);
  ESP_LOGI(TAG, "  client_id: %s", this->client_id_.c_str());
  ESP_LOGI(TAG, "  sub_topic: %s", this->sub_topic_.c_str());
  this->publish_sensor("MQTT Status", "Idle");
  this->publish_sensor("MQTT Endpoint", this->endpoint_);
  this->publish_sensor("MQTT Topic", this->sub_topic_);
}

void AWSMqttComponent::manual_connect() {
  if (this->connected_) {
    ESP_LOGW(TAG, "Already connected");
    return;
  }
  if (this->connect_state_ == MCS_CONNECTING) {
    ESP_LOGW(TAG, "Connection already in progress");
    return;
  }
  ESP_LOGI(TAG, "Manual connect requested");
  this->enabled_ = true;
  this->reconnect_attempts_ = 0;
  this->reconnect_interval_ = 1000;
  this->last_connect_attempt_ = 0;
  this->publish_sensor("MQTT Status", "Connecting...");
  this->log_to_dashboard("MQTT connect: " + this->endpoint_ + ":" + std::to_string(this->port_));
}

void AWSMqttComponent::manual_disconnect() {
  ESP_LOGI(TAG, "Manual disconnect requested");
  this->enabled_ = false;
  if (this->connected_) {
    this->mqtt_send_disconnect();
    this->connected_ = false;
    this->subscribed_ = false;
    this->do_disconnect();
    this->publish_sensor("MQTT Status", "Disconnected");
    this->log_to_dashboard("MQTT disconnected (manual)");
  } else {
    if (this->connect_state_ == MCS_CONNECTING) {
      ESP_LOGW(TAG, "Connection in progress, will stop after it finishes");
    }
    this->publish_sensor("MQTT Status", "Idle");
  }
}

// Runtime config update methods
void AWSMqttComponent::update_endpoint(const std::string &v) {
  if (v.empty()) return;
  this->endpoint_ = v;
  this->publish_sensor("MQTT Endpoint", v);
  ESP_LOGI(TAG, "Endpoint updated: %s", v.c_str());
  this->log_to_dashboard("Endpoint: " + v);
}

void AWSMqttComponent::update_port(int v) {
  if (v <= 0 || v > 65535) return;
  this->port_ = v;
  ESP_LOGI(TAG, "Port updated: %d", v);
  this->log_to_dashboard("Port: " + std::to_string(v));
}

void AWSMqttComponent::update_tenant_id(const std::string &v) {
  if (v.empty()) return;
  this->tenant_id_ = v;
  ESP_LOGI(TAG, "Tenant ID updated");
  this->log_to_dashboard("Tenant ID set");
}

void AWSMqttComponent::update_secret_key(const std::string &v) {
  if (v.empty()) return;
  this->secret_key_ = v;
  ESP_LOGI(TAG, "Secret key updated");
  this->log_to_dashboard("Secret key set");
}

void AWSMqttComponent::update_nonce(const std::string &v) {
  if (v.empty()) return;
  this->nonce_ = v;
  ESP_LOGI(TAG, "Nonce updated: %s", v.c_str());
  this->log_to_dashboard("Nonce: " + v);
}

void AWSMqttComponent::update_auth_name(const std::string &v) {
  if (v.empty()) return;
  this->auth_name_ = v;
  ESP_LOGI(TAG, "Auth name updated: %s", v.c_str());
  this->log_to_dashboard("Auth: " + v);
}

void AWSMqttComponent::update_client_id(const std::string &v) {
  if (v.empty()) return;
  this->client_id_ = v;
  ESP_LOGI(TAG, "Client ID updated: %s", v.c_str());
  this->log_to_dashboard("ClientID: " + v);
}

void AWSMqttComponent::update_sub_topic(const std::string &v) {
  if (v.empty()) return;
  this->sub_topic_ = v;
  this->publish_sensor("MQTT Topic", v);
  ESP_LOGI(TAG, "Sub topic updated: %s", v.c_str());
  this->log_to_dashboard("Topic: " + v);
}

// =========================================================================
// Main Loop
// =========================================================================

void AWSMqttComponent::loop() {
  if (!this->enabled_) return;

  uint32_t now = millis();

  // === Ethernet link detection ===
  bool link_up = false;
  auto *eth = ethernet::global_eth_component;
  if (eth != nullptr) {
    link_up = eth->is_connected();
  }

  if (this->last_link_up_ && !link_up) {
    ESP_LOGW(TAG, "Ethernet link DOWN");
    this->log_to_dashboard("MQTT: Link DOWN");
    if (this->connected_) {
      this->connected_ = false;
      this->subscribed_ = false;
      this->do_disconnect();
    }
    this->publish_sensor("MQTT Status", "Link DOWN");
    this->reconnect_attempts_ = 0;
    this->reconnect_interval_ = 3000;
  }

  if (!this->last_link_up_ && link_up) {
    ESP_LOGI(TAG, "Ethernet link UP");
    this->log_to_dashboard("MQTT: Link UP");
    if (!this->connected_ && this->connect_state_ == MCS_IDLE) {
      this->last_connect_attempt_ = 0;
      this->reconnect_interval_ = 3000;
      this->reconnect_attempts_ = 0;
      this->publish_sensor("MQTT Status", "Link UP, connecting...");
    }
  }

  this->last_link_up_ = link_up;

  if (!link_up) {
    if (this->connected_) {
      this->connected_ = false;
      this->subscribed_ = false;
      this->do_disconnect();
      this->publish_sensor("MQTT Status", "Link DOWN");
    }
    return;
  }

  // === Background connect task states ===
  if (this->connect_state_ == MCS_CONNECTING) {
    return;
  }

  if (this->connect_state_ == MCS_CONNECTED) {
    this->connect_state_ = MCS_IDLE;
    this->connected_ = true;
    this->connected_time_ = now;
    this->subscribed_ = false;
    this->reconnect_attempts_ = 0;
    this->reconnect_interval_ = 5000;

    // Set socket to non-blocking
    if (this->sock_fd_ >= 0) {
      int flags = fcntl(this->sock_fd_, F_GETFL, 0);
      fcntl(this->sock_fd_, F_SETFL, flags | O_NONBLOCK);
    }

    this->last_data_time_ = now;
    this->last_ping_time_ = now;
    this->publish_sensor("MQTT Status", "Connected");
    this->log_to_dashboard("MQTT connected to " + this->endpoint_);
    ESP_LOGI(TAG, "Connected to %s:%d", this->endpoint_.c_str(), this->port_);

    // Auto-subscribe
    if (!this->sub_topic_.empty()) {
      if (this->mqtt_subscribe(this->sub_topic_)) {
        this->subscribed_ = true;
        this->log_to_dashboard("Subscribed: " + this->sub_topic_);
      } else {
        this->log_to_dashboard("Subscribe FAILED: " + this->sub_topic_);
      }
    }
  }

  if (this->connect_state_ == MCS_FAILED) {
    this->connect_state_ = MCS_IDLE;
    this->last_connect_attempt_ = now;

    if (this->reconnect_attempts_ < MAX_FAST_RETRIES) {
      this->reconnect_attempts_++;
      this->reconnect_interval_ = this->get_backoff_with_jitter();
    } else {
      this->reconnect_interval_ = SLOW_RETRY_INTERVAL_MS;
    }

    char buf[80];
    snprintf(buf, sizeof(buf), "Retry %us (#%u)",
             this->reconnect_interval_ / 1000, (unsigned)this->reconnect_attempts_);
    this->publish_sensor("MQTT Status", buf);
    ESP_LOGW(TAG, "Connect failed, retry in %u ms (#%u)",
             (unsigned)this->reconnect_interval_, (unsigned)this->reconnect_attempts_);
  }

  // === Not connected: start new attempt ===
  if (!this->connected_ && this->connect_state_ == MCS_IDLE) {
    if (now - this->last_connect_attempt_ >= this->reconnect_interval_) {
      auto ip_addrs = network::get_ip_addresses();
      if (ip_addrs.empty() || !ip_addrs[0].is_set()) {
        this->last_connect_attempt_ = now;
        this->reconnect_interval_ = 10000;
        this->publish_sensor("MQTT Status", "No IP, retry 10s");
        return;
      }

      this->connect_state_ = MCS_CONNECTING;
      this->publish_sensor("MQTT Status", "Connecting...");
      ESP_LOGI(TAG, "Starting connect task (#%u)...", (unsigned)this->reconnect_attempts_ + 1);

      BaseType_t ret = xTaskCreate(
        connect_task_fn, "mqtt_conn", 8192, this, 5, &this->connect_task_handle_
      );
      if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create connect task!");
        this->connect_state_ = MCS_FAILED;
      }
    }
    return;
  }

  // === Connected: read incoming MQTT packets ===
  int packets_read = 0;
  while (packets_read < 10) {
    int pkt_type = this->mqtt_read_packet();
    if (pkt_type < 0) {
      uint32_t uptime_s = (now - this->connected_time_) / 1000;
      this->connected_ = false;
      this->subscribed_ = false;
      this->last_connect_attempt_ = now;
      this->reconnect_interval_ = this->get_backoff_with_jitter();
      if (this->reconnect_attempts_ < MAX_FAST_RETRIES) this->reconnect_attempts_++;
      char msg[80];
      snprintf(msg, sizeof(msg), "Disconnected after %us", (unsigned)uptime_s);
      this->publish_sensor("MQTT Status", msg);
      this->log_to_dashboard(msg);
      this->do_disconnect();
      return;
    }
    if (pkt_type == 0) break;
    packets_read++;
    this->last_data_time_ = now;
  }

  // Keep-alive PINGREQ every 10s
  if (now - this->last_ping_time_ > 10000) {
    this->last_ping_time_ = now;
    this->mqtt_send_pingreq();
  }

  // Pong timeout: no data for 15s = dead
  if (now - this->last_data_time_ > 15000) {
    uint32_t uptime_s = (now - this->connected_time_) / 1000;
    ESP_LOGW(TAG, "No data 15s, dead (up %us)", (unsigned)uptime_s);
    char msg[80];
    snprintf(msg, sizeof(msg), "No response 15s (up %us)", (unsigned)uptime_s);
    this->publish_sensor("MQTT Status", msg);
    this->log_to_dashboard(msg);
    this->connected_ = false;
    this->subscribed_ = false;
    this->last_connect_attempt_ = now;
    this->reconnect_interval_ = 5000;
    this->reconnect_attempts_ = 0;
    this->do_disconnect();
  }
}

uint32_t AWSMqttComponent::get_backoff_with_jitter() {
  uint32_t base = 5000 * (1 << this->reconnect_attempts_);
  if (base > 60000) base = 60000;
  uint32_t jitter_range = base * 4 / 10;
  uint32_t jitter = random(0, jitter_range);
  return base - (jitter_range / 2) + jitter;
}

// =========================================================================
// TLS I/O helpers
// =========================================================================

ssize_t AWSMqttComponent::tls_write(const void *buf, size_t len) {
  int ret;
  do {
    ret = mbedtls_ssl_write(&this->ssl_, (const unsigned char *)buf, len);
  } while (ret == MBEDTLS_ERR_SSL_WANT_WRITE);
  if (ret < 0) {
    char err_buf[100];
    mbedtls_strerror(ret, err_buf, sizeof(err_buf));
    ESP_LOGE(TAG, "ssl_write error: -0x%04x %s", -ret, err_buf);
    return -1;
  }
  return ret;
}

ssize_t AWSMqttComponent::tls_read(void *buf, size_t len) {
  int ret = mbedtls_ssl_read(&this->ssl_, (unsigned char *)buf, len);
  if (ret == MBEDTLS_ERR_SSL_WANT_READ || ret == MBEDTLS_ERR_SSL_WANT_WRITE) {
    errno = EAGAIN;
    return -1;
  }
  if (ret == MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY || ret == 0) {
    return 0;
  }
  if (ret < 0) {
    char err_buf[100];
    mbedtls_strerror(ret, err_buf, sizeof(err_buf));
    ESP_LOGE(TAG, "ssl_read error: -0x%04x %s", -ret, err_buf);
    errno = EIO;
    return -1;
  }
  return ret;
}

// =========================================================================
// Connection (runs in FreeRTOS background task)
// =========================================================================

bool AWSMqttComponent::do_connect() {
  uint32_t t0 = millis();

  // Step 1: DNS resolve
  ESP_LOGI(TAG, "[1/4] DNS %s...", this->endpoint_.c_str());
  struct addrinfo hints = {};
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;
  struct addrinfo *res = nullptr;
  int dns_err = getaddrinfo(this->endpoint_.c_str(), nullptr, &hints, &res);
  if (dns_err != 0 || !res) {
    ESP_LOGE(TAG, "DNS failed: %d", dns_err);
    this->log_to_dashboard("DNS fail: " + this->endpoint_);
    if (res) freeaddrinfo(res);
    return false;
  }
  char ip_str[16];
  struct sockaddr_in *addr = (struct sockaddr_in *)res->ai_addr;
  inet_ntoa_r(addr->sin_addr, ip_str, sizeof(ip_str));
  ESP_LOGI(TAG, "[1/4] DNS OK: %s -> %s (%ums)", this->endpoint_.c_str(), ip_str, (unsigned)(millis() - t0));

  // Step 2: TCP connect
  ESP_LOGI(TAG, "[2/4] TCP %s:%d...", ip_str, this->port_);
  struct sockaddr_in dest;
  memset(&dest, 0, sizeof(dest));
  dest.sin_family = AF_INET;
  dest.sin_port = htons(this->port_);
  dest.sin_addr = addr->sin_addr;
  freeaddrinfo(res);

  this->sock_fd_ = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (this->sock_fd_ < 0) {
    ESP_LOGE(TAG, "socket() failed: %d", errno);
    return false;
  }

  struct timeval tv;
  tv.tv_sec = 15;
  tv.tv_usec = 0;
  setsockopt(this->sock_fd_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
  setsockopt(this->sock_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  uint32_t tcp_t0 = millis();
  int conn_ret = ::connect(this->sock_fd_, (struct sockaddr *)&dest, sizeof(dest));
  uint32_t tcp_dt = millis() - tcp_t0;

  if (conn_ret != 0) {
    ESP_LOGE(TAG, "[2/4] TCP FAIL %ums errno=%d", (unsigned)tcp_dt, errno);
    this->log_to_dashboard("TCP FAIL errno=" + std::to_string(errno));
    close(this->sock_fd_);
    this->sock_fd_ = -1;
    return false;
  }
  ESP_LOGI(TAG, "[2/4] TCP OK %ums", (unsigned)tcp_dt);

  // Step 3: TLS handshake with ALPN "mqtt"
  ESP_LOGI(TAG, "[3/4] TLS (ALPN: mqtt)...");
  uint32_t tls_t0 = millis();

  mbedtls_ssl_init(&this->ssl_);
  mbedtls_ssl_config_init(&this->ssl_conf_);
  mbedtls_entropy_init(&this->entropy_);
  mbedtls_ctr_drbg_init(&this->ctr_drbg_);
  this->mbedtls_initialized_ = true;

  int ret = mbedtls_ctr_drbg_seed(&this->ctr_drbg_, mbedtls_entropy_func,
                                   &this->entropy_, nullptr, 0);
  if (ret != 0) {
    ESP_LOGE(TAG, "ctr_drbg_seed failed: -0x%04x", -ret);
    this->do_disconnect();
    return false;
  }

  ret = mbedtls_ssl_config_defaults(&this->ssl_conf_,
                                     MBEDTLS_SSL_IS_CLIENT,
                                     MBEDTLS_SSL_TRANSPORT_STREAM,
                                     MBEDTLS_SSL_PRESET_DEFAULT);
  if (ret != 0) {
    ESP_LOGE(TAG, "ssl_config failed: -0x%04x", -ret);
    this->do_disconnect();
    return false;
  }

  mbedtls_ssl_conf_authmode(&this->ssl_conf_, MBEDTLS_SSL_VERIFY_NONE);
  mbedtls_ssl_conf_rng(&this->ssl_conf_, mbedtls_ctr_drbg_random, &this->ctr_drbg_);

  // ALPN: "mqtt" — REQUIRED for AWS IoT port 443
  static const char *alpn_protocols[] = {"mqtt", nullptr};
  mbedtls_ssl_conf_alpn_protocols(&this->ssl_conf_, alpn_protocols);

  // TLS 1.2 minimum
  mbedtls_ssl_conf_min_tls_version(&this->ssl_conf_, MBEDTLS_SSL_VERSION_TLS1_2);

  ret = mbedtls_ssl_setup(&this->ssl_, &this->ssl_conf_);
  if (ret != 0) {
    ESP_LOGE(TAG, "ssl_setup failed: -0x%04x", -ret);
    this->do_disconnect();
    return false;
  }

  // SNI
  mbedtls_ssl_set_hostname(&this->ssl_, this->endpoint_.c_str());

  // I/O callbacks
  mbedtls_ssl_set_bio(&this->ssl_, &this->sock_fd_, mbed_send_cb, mbed_recv_cb, nullptr);

  // TLS handshake
  while ((ret = mbedtls_ssl_handshake(&this->ssl_)) != 0) {
    if (ret != MBEDTLS_ERR_SSL_WANT_READ && ret != MBEDTLS_ERR_SSL_WANT_WRITE) {
      char err_buf[100];
      mbedtls_strerror(ret, err_buf, sizeof(err_buf));
      ESP_LOGE(TAG, "[3/4] TLS FAIL %ums: %s", (unsigned)(millis() - tls_t0), err_buf);
      this->log_to_dashboard(std::string("TLS FAIL: ") + err_buf);
      this->do_disconnect();
      return false;
    }
    if (millis() - tls_t0 > 30000) {
      ESP_LOGE(TAG, "[3/4] TLS timeout 30s");
      this->log_to_dashboard("TLS timeout 30s");
      this->do_disconnect();
      return false;
    }
  }

  uint32_t tls_dt = millis() - tls_t0;
  ESP_LOGI(TAG, "[3/4] TLS OK %ums (cipher: %s)", (unsigned)tls_dt,
           mbedtls_ssl_get_ciphersuite(&this->ssl_));

  // Step 4: MQTT CONNECT packet
  ESP_LOGI(TAG, "[4/4] MQTT CONNECT (client_id=%s)...", this->client_id_.c_str());
  if (!this->mqtt_connect()) {
    ESP_LOGE(TAG, "[4/4] MQTT CONNECT failed");
    this->log_to_dashboard("MQTT CONNECT failed");
    this->do_disconnect();
    return false;
  }

  uint32_t total = millis() - t0;
  ESP_LOGI(TAG, "Connected in %ums", (unsigned)total);
  this->log_to_dashboard("Connected OK (" + std::to_string(total) + "ms)");
  return true;
}

void AWSMqttComponent::do_disconnect() {
  if (this->mbedtls_initialized_) {
    mbedtls_ssl_close_notify(&this->ssl_);
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
// MQTT Protocol Implementation
// =========================================================================

static int encode_remaining_length(uint8_t *buf, uint32_t len) {
  int i = 0;
  do {
    uint8_t byte = len % 128;
    len /= 128;
    if (len > 0) byte |= 0x80;
    buf[i++] = byte;
  } while (len > 0);
  return i;
}

bool AWSMqttComponent::mqtt_connect() {
  std::string username, password;
  this->generate_credentials(username, password);

  // Use configured client_id
  const std::string &cid = this->client_id_;
  ESP_LOGI(TAG, "MQTT client_id=%s", cid.c_str());

  uint16_t client_id_len = cid.length();
  uint16_t username_len = username.length();
  uint16_t password_len = password.length();

  uint32_t remaining = 10 + (2 + client_id_len) + (2 + username_len) + (2 + password_len);

  uint8_t pkt[512];
  int pos = 0;

  pkt[pos++] = MQTT_CONNECT;
  pos += encode_remaining_length(pkt + pos, remaining);

  // Protocol Name "MQTT"
  pkt[pos++] = 0x00; pkt[pos++] = 0x04;
  pkt[pos++] = 'M'; pkt[pos++] = 'Q'; pkt[pos++] = 'T'; pkt[pos++] = 'T';
  // Protocol Level 4 (MQTT 3.1.1)
  pkt[pos++] = 0x04;
  // Connect Flags: Clean Session=1, Username=1, Password=1
  pkt[pos++] = 0xC2;
  // Keep Alive = 10 seconds
  pkt[pos++] = 0x00; pkt[pos++] = 10;

  // Client ID
  pkt[pos++] = (client_id_len >> 8) & 0xFF;
  pkt[pos++] = client_id_len & 0xFF;
  memcpy(pkt + pos, cid.c_str(), client_id_len);
  pos += client_id_len;

  // Username
  pkt[pos++] = (username_len >> 8) & 0xFF;
  pkt[pos++] = username_len & 0xFF;
  memcpy(pkt + pos, username.c_str(), username_len);
  pos += username_len;

  // Password
  pkt[pos++] = (password_len >> 8) & 0xFF;
  pkt[pos++] = password_len & 0xFF;
  memcpy(pkt + pos, password.c_str(), password_len);
  pos += password_len;

  ssize_t written = this->tls_write(pkt, pos);
  if (written != pos) {
    ESP_LOGE(TAG, "CONNECT send failed: %d/%d", (int)written, pos);
    return false;
  }

  // Wait for CONNACK (up to 10s)
  uint32_t start = millis();
  while (millis() - start < 10000) {
    uint8_t hdr[4];
    ssize_t ret = this->tls_read(hdr, 4);
    if (ret == 4) {
      if ((hdr[0] & 0xF0) == MQTT_CONNACK) {
        uint8_t return_code = hdr[3];
        if (return_code == 0) {
          ESP_LOGI(TAG, "CONNACK OK");
          return true;
        } else {
          const char *reasons[] = {
            "Accepted", "Bad protocol", "ID rejected",
            "Server unavail", "Bad user/pass", "Not authorized"
          };
          const char *reason = return_code <= 5 ? reasons[return_code] : "Unknown";
          ESP_LOGE(TAG, "CONNACK rejected: %u (%s)", return_code, reason);
          this->log_to_dashboard(std::string("Rejected: ") + reason);
          return false;
        }
      }
    } else if (ret == 0) {
      return false;
    } else if (ret < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        delay(50);
        continue;
      }
      return false;
    }
  }

  ESP_LOGE(TAG, "CONNACK timeout");
  return false;
}

bool AWSMqttComponent::mqtt_subscribe(const std::string &topic) {
  if (this->sock_fd_ < 0) return false;

  this->packet_id_++;
  uint16_t pid = this->packet_id_;
  uint16_t topic_len = topic.length();

  uint32_t remaining = 2 + 2 + topic_len + 1;

  uint8_t pkt[512];
  int pos = 0;

  pkt[pos++] = MQTT_SUBSCRIBE;
  pos += encode_remaining_length(pkt + pos, remaining);

  pkt[pos++] = (pid >> 8) & 0xFF;
  pkt[pos++] = pid & 0xFF;

  pkt[pos++] = (topic_len >> 8) & 0xFF;
  pkt[pos++] = topic_len & 0xFF;
  memcpy(pkt + pos, topic.c_str(), topic_len);
  pos += topic_len;

  pkt[pos++] = 0x01;  // QoS 1

  // Blocking mode for subscribe
  if (this->sock_fd_ >= 0) {
    int flags = fcntl(this->sock_fd_, F_GETFL, 0);
    fcntl(this->sock_fd_, F_SETFL, flags & ~O_NONBLOCK);
  }

  ssize_t written = this->tls_write(pkt, pos);

  if (this->sock_fd_ >= 0) {
    int flags = fcntl(this->sock_fd_, F_GETFL, 0);
    fcntl(this->sock_fd_, F_SETFL, flags | O_NONBLOCK);
  }

  if (written != pos) {
    ESP_LOGE(TAG, "SUBSCRIBE send failed");
    return false;
  }

  ESP_LOGI(TAG, "SUBSCRIBE: %s qos=1 pid=%u", topic.c_str(), pid);

  // Wait for SUBACK (5s blocking)
  if (this->sock_fd_ >= 0) {
    int flags = fcntl(this->sock_fd_, F_GETFL, 0);
    fcntl(this->sock_fd_, F_SETFL, flags & ~O_NONBLOCK);
    struct timeval tv;
    tv.tv_sec = 5;
    tv.tv_usec = 0;
    setsockopt(this->sock_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  }

  uint8_t suback[5];
  ssize_t ret = this->tls_read(suback, 5);

  if (this->sock_fd_ >= 0) {
    int flags = fcntl(this->sock_fd_, F_GETFL, 0);
    fcntl(this->sock_fd_, F_SETFL, flags | O_NONBLOCK);
  }

  if (ret >= 5 && (suback[0] & 0xF0) == MQTT_SUBACK) {
    uint8_t qos = suback[4];
    if (qos <= 2) {
      ESP_LOGI(TAG, "SUBACK OK qos=%u", qos);
      return true;
    } else {
      ESP_LOGE(TAG, "SUBACK fail: 0x%02x", qos);
      return false;
    }
  }

  ESP_LOGW(TAG, "No SUBACK (ret=%d)", (int)ret);
  return true;
}

int AWSMqttComponent::mqtt_read_packet() {
  if (this->sock_fd_ < 0) return -1;

  uint8_t hdr_byte;
  ssize_t ret = this->tls_read(&hdr_byte, 1);
  if (ret == 0) return -1;
  if (ret < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
    return -1;
  }

  uint8_t pkt_type = hdr_byte & 0xF0;

  // Read remaining length
  uint32_t remaining = 0;
  uint32_t multiplier = 1;
  uint8_t len_byte;
  int len_bytes_read = 0;
  do {
    ret = this->tls_read(&len_byte, 1);
    if (ret <= 0) return -1;
    remaining += (len_byte & 0x7F) * multiplier;
    multiplier *= 128;
    len_bytes_read++;
    if (len_bytes_read > 4) return -1;
  } while (len_byte & 0x80);

  // Read payload
  if (remaining > sizeof(this->read_buf_) - 1) {
    size_t left = remaining;
    while (left > 0) {
      size_t chunk = left > sizeof(this->read_buf_) ? sizeof(this->read_buf_) : left;
      ret = this->tls_read(this->read_buf_, chunk);
      if (ret <= 0) return -1;
      left -= ret;
    }
    return pkt_type >> 4;
  }

  size_t total_read = 0;
  while (total_read < remaining) {
    ret = this->tls_read(this->read_buf_ + total_read, remaining - total_read);
    if (ret <= 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        delay(1);
        continue;
      }
      return -1;
    }
    total_read += ret;
  }

  switch (pkt_type) {
    case MQTT_PINGRESP:
      ESP_LOGD(TAG, "RX PINGRESP");
      break;

    case MQTT_PUBLISH: {
      uint8_t qos = (hdr_byte >> 1) & 0x03;
      if (total_read < 2) break;
      uint16_t topic_len = (this->read_buf_[0] << 8) | this->read_buf_[1];
      if (topic_len + 2 > total_read) break;

      std::string topic((char *)this->read_buf_ + 2, topic_len);
      size_t payload_offset = 2 + topic_len;
      if (qos > 0 && payload_offset + 2 <= total_read) {
        payload_offset += 2;
      }

      size_t payload_len = total_read - payload_offset;
      std::string payload((char *)this->read_buf_ + payload_offset, payload_len);

      this->message_count_++;
      ESP_LOGI(TAG, "RX #%u %s (%u bytes)", (unsigned)this->message_count_,
               topic.c_str(), (unsigned)payload_len);

      this->publish_sensor("MQTT Last Topic", topic);

      cJSON *json = cJSON_Parse(payload.c_str());
      if (json) {
        char *pretty = cJSON_PrintUnformatted(json);
        if (pretty) {
          this->publish_sensor("MQTT Last Message", std::string(pretty));
          this->log_to_dashboard("RX: " + topic.substr(topic.length() > 30 ? topic.length() - 30 : 0) + " -> " + std::string(pretty).substr(0, 100));
          cJSON_free(pretty);
        }
        // Extract deviceCode, workStatus, stepStatus
        cJSON *dc = cJSON_GetObjectItem(json, "deviceCode");
        if (dc && cJSON_IsString(dc)) this->publish_sensor("MQTT Device Code", dc->valuestring);
        cJSON *ws = cJSON_GetObjectItem(json, "workStatus");
        if (ws) this->publish_sensor("MQTT Work Status", cJSON_IsString(ws) ? ws->valuestring : std::to_string(ws->valueint));
        cJSON *ss = cJSON_GetObjectItem(json, "stepStatus");
        if (ss) this->publish_sensor("MQTT Step Status", cJSON_IsString(ss) ? ss->valuestring : std::to_string(ss->valueint));
        cJSON_Delete(json);
      } else {
        this->publish_sensor("MQTT Last Message", payload);
        this->log_to_dashboard("RX: " + payload.substr(0, 100));
      }
      break;
    }

    case MQTT_SUBACK:
      ESP_LOGI(TAG, "RX SUBACK");
      break;

    default:
      ESP_LOGD(TAG, "RX 0x%02x len=%u", pkt_type, (unsigned)remaining);
      break;
  }

  return pkt_type >> 4;
}

void AWSMqttComponent::mqtt_send_pingreq() {
  if (this->sock_fd_ < 0 || !this->connected_) return;
  uint8_t pkt[2] = {MQTT_PINGREQ, 0x00};
  this->tls_write(pkt, 2);
}

void AWSMqttComponent::mqtt_send_disconnect() {
  if (this->sock_fd_ < 0) return;
  uint8_t pkt[2] = {MQTT_DISCONNECT, 0x00};
  this->tls_write(pkt, 2);
}

// =========================================================================
// Sensor Publishing
// =========================================================================

void AWSMqttComponent::publish_sensor(const std::string &name, const std::string &state) {
  for (auto *sensor : App.get_text_sensors()) {
    if (sensor->get_name() == name) {
      sensor->publish_state(state);
      return;
    }
  }
  for (auto *sensor : App.get_text_sensors()) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    if (sensor->get_object_id() == name) {
      sensor->publish_state(state);
      return;
    }
#pragma GCC diagnostic pop
  }
}

void AWSMqttComponent::log_to_dashboard(const std::string &msg) {
  this->publish_sensor("MQTT Log", msg);
}

}  // namespace aws_mqtt
}  // namespace esphome
