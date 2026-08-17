# 修改记录（CHANGELOG）

## 2026-08-15 ESPHome 2026.x 升级兼容性修复

项目原在 ESPHome 2024.12.4 可编译，升级到 2026.6.4 后编译报错，原因及修复如下：

### 1. 灯效颜色类型 `ESPColor` 已迁移为 `Color`
ESPHome 2025.2 起删除旧类型 `ESPColor` / `light::ESPColor`，统一为 `esphome::Color`。
- 受影响文件（`effects/` 下）：`bluez.yaml`、`christmas.yaml`、`fire.yaml`、
  `flames.yaml`、`vesuvius.yaml`、`wheel_of_color.yaml`、`wipe_in.yaml`、`wipe_out.yaml`
- 修复：`ESPColor(r,g,b)` / `light::ESPColor(r,g,b)` → `Color(r,g,b)`

### 2. `effects/fireplace.yaml` 使用了 FastLED 专属函数
`random8` / `scale8_video` / `qadd8` 不是 ESPHome API，需替换：
- `random8()` → `random_uint32() & 0xFF`
- `scale8_video(h, n)` → `esp_scale8(h, n)`
- `qadd8(a, b)` → `(a + b) > 255 ? 255 : (a + b)`

### 3. `column.yaml` 中全局 `WiFi` 对象不再可用
新版 esp32 框架（arduino + espidf，底层 ESP-IDF 5.5）不再暴露 Arduino 全局 `WiFi`。
- `WiFi.macAddress()` → 使用已通过 `get_mac_address_raw(id(my_local_mac))` 获取的
  MAC 字节，用 `%02X` 格式化
- `WiFi.localIP().toString()` → 使用
  `esphome::wifi::global_wifi_component->get_ip_addresses()` + `ip.is_ip4()` +
  `ip.str_to(buf)` 获取 IPv4 地址

### 4. `column.yaml` 中 `uart_*_bytes` 参数类型
新版解析为 IDF 原生函数，第一个参数必须是 `uart_port_t`，裸 `int` 会报错。
- `uart_write_bytes(0/1, ...)` / `uart_read_bytes(1, ...)` → 加 `(uart_port_t)` 强制转换
- 共 5 处

### 5. `effects/flames.yaml` 中 `min()` 类型不匹配
`std::min` 模板要求参数类型一致（`it.size()` 为 int32_t，`7` 为 int）。
- `min(it.size() / 10, 7)` → `min((int) (it.size() / 10), 7)`

## 2026-08-17 NTC 温度测量修复

### 电路接法
```
3.3V ── 5.6kΩ ──●── NTC(10kΩ@25℃, B=3950) ── GND
                 │
              GPIO2 (ADC1_CH1)
```
采样点电压：V = 3.3V × R_ntc / (R_ntc + 5.6kΩ)

### 问题
页面温度值一直不变，原因有两个：
1. ADC 未设置 `attenuation`，默认 `0db`（量程 0~0.95V），电压超量程后读数饱和不变；
2. `resistance` 传感器的 `configuration` 配成 `UPSTREAM`，与实际电路不符
   （实际 NTC 在采样点与 GND 之间，应为 `DOWNSTREAM`）。

### 修复
- `adc` 传感器：增加 `attenuation: auto`（自动量程），并加 `on_value` 调试日志
- `resistance` 传感器：`configuration: UPSTREAM` → `DOWNSTREAM`

### NTC 相关最终配置
```yaml
sensor:
  - platform: ntc
    sensor: resistance_sensor
    calibration:
      b_constant: 3950
      reference_temperature: 25°C
      reference_resistance: 10kOhm
    name: NTC Temperature

  - platform: resistance
    id: resistance_sensor
    sensor: source_sensor
    configuration: DOWNSTREAM
    resistor: 5.6kOhm
    name: Resistance Sensor

  - platform: adc
    id: source_sensor
    pin: 2
    attenuation: auto
    update_interval: 6s
```
