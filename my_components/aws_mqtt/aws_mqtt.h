#pragma once

#include "esphome/core/component.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/network/util.h"
#include "esphome/components/ethernet/ethernet_component.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern "C" {
#include "mbedtls/ssl.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/md.h"
#include "mbedtls/base64.h"
#include "cJSON.h"
}

namespace esphome {
namespace aws_mqtt {

enum MqttConnectState : uint8_t {
  MCS_IDLE,
  MCS_CONNECTING,
  MCS_CONNECTED,
  MCS_FAILED,
};

class AWSMqttComponent : public Component {
 public:
  void set_endpoint(const std::string &v) { this->endpoint_ = v; }
  void set_port(int v) { this->port_ = v; }
  void set_tenant_id(const std::string &v) { this->tenant_id_ = v; }
  void set_secret_key(const std::string &v) { this->secret_key_ = v; }
  void set_nonce(const std::string &v) { this->nonce_ = v; }
  void set_auth_name(const std::string &v) { this->auth_name_ = v; }
  void set_client_id(const std::string &v) { this->client_id_ = v; }
  void set_sub_topic(const std::string &v) { this->sub_topic_ = v; }

  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  void setup() override;
  void loop() override;

  // Manual connect/disconnect (called from Dashboard buttons)
  void manual_connect();
  void manual_disconnect();

  // Runtime config updates (called from Dashboard text inputs)
  void update_endpoint(const std::string &v);
  void update_port(int v);
  void update_tenant_id(const std::string &v);
  void update_secret_key(const std::string &v);
  void update_nonce(const std::string &v);
  void update_auth_name(const std::string &v);
  void update_client_id(const std::string &v);
  void update_sub_topic(const std::string &v);

  bool is_connected() const { return this->connected_; }
  std::string get_endpoint() const { return this->endpoint_; }
  std::string get_sub_topic() const { return this->sub_topic_; }

 protected:
  bool do_connect();
  void do_disconnect();

  // MQTT protocol
  bool mqtt_connect();
  bool mqtt_subscribe(const std::string &topic);
  int mqtt_read_packet();
  void mqtt_send_pingreq();
  void mqtt_send_disconnect();

  // HMAC-SHA256 credential generation
  void generate_credentials(std::string &username, std::string &password);

  // TLS I/O helpers
  ssize_t tls_write(const void *buf, size_t len);
  ssize_t tls_read(void *buf, size_t len);

  // Sensor publishing
  void publish_sensor(const std::string &name, const std::string &state);
  void log_to_dashboard(const std::string &msg);

  // Backoff
  uint32_t get_backoff_with_jitter();

  // FreeRTOS background connect task
  static void connect_task_fn(void *param);
  volatile MqttConnectState connect_state_{MCS_IDLE};
  TaskHandle_t connect_task_handle_{nullptr};

  // Raw socket + mbedtls
  int sock_fd_{-1};
  mbedtls_ssl_context ssl_;
  mbedtls_ssl_config ssl_conf_;
  mbedtls_entropy_context entropy_;
  mbedtls_ctr_drbg_context ctr_drbg_;
  bool mbedtls_initialized_{false};

  // Config
  std::string endpoint_;
  int port_{443};
  std::string tenant_id_;
  std::string secret_key_;
  std::string nonce_;
  std::string auth_name_;
  std::string client_id_;
  std::string sub_topic_;

  // State
  bool connected_{false};
  bool enabled_{false};
  bool subscribed_{false};
  uint32_t connected_time_{0};
  uint32_t last_ping_time_{0};
  uint32_t last_data_time_{0};
  uint32_t last_connect_attempt_{0};
  uint32_t reconnect_interval_{5000};
  uint32_t reconnect_attempts_{0};
  uint32_t setup_time_{0};
  bool last_link_up_{false};
  uint32_t message_count_{0};

  // MQTT packet ID counter
  uint16_t packet_id_{0};

  // Buffers
  uint8_t read_buf_[2048];
};

}  // namespace aws_mqtt
}  // namespace esphome
