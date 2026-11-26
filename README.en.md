# ESP32-C3 16-Channel Relay Control Board Test Bench

An [ESPHome](https://esphome.io/)-based test and integration environment for a **16-channel relay control board** (Modbus RTU slave).

The **ESP32-C3** acts as the Modbus master, controlling 16 relay outputs over an **RS485 bus**, with a built-in **web control panel** for:

- Functional testing of the relay module (single-channel switching, delayed action, self-test)
- Performance testing (cyclic switching with configurable cycles / duration / interval)
- Integrating the relay module into real applications (use the register map with any host / PLC)

> The web panel supports **Chinese / English** switching.

---

## 1. Hardware & Wiring

| Part | Description |
|------|-------------|
| Master | ESP32-C3 dev board (Generic ESP32-C3, esp-idf framework) |
| Device | 16-channel relay control board (Modbus RTU **slave**, default address **100**) |
| Bus | RS485, default **9600 8N1** (19200 / 38400 / 57600 / 115200 selectable) |

**Wiring (ESP32-C3 ↔ relay board):**

| ESP32-C3 | Relay board |
|----------|-------------|
| GPIO21 | RS485 TX |
| GPIO20 | RS485 RX |
| GND | GND |

> If your RS485 transceiver needs direction control (DE/RE), add a `flow_control_pin` accordingly.

---

## 2. Features

- 16 independent relay switches + one-click All On / All Off
- Delayed relay action (auto-release after a delay, registers 400~415)
- Pickup delay setting (register 202, μs, saved to EEPROM)
- Power-on state mode (register 204: all off / all on / restore last state)
- Relay delay self-test (register 999, pickup/release timing auto-learn)
- Performance test: cyclic switching of selected relays with configurable cycles, duration and interval
- Live status: firmware version, AC period, pickup delay, DC current, power-on mode, config index
- Network status LED (solid = connected / slow blink = connecting / fast blink = AP mode)
- Web control panel with Chinese / English switching

---

## 3. Directory Layout

```
.
├── my-iot-relay-c3-demo.yaml      # ESP32-C3 relay test bench config (recommended)
├── my-iot-relay-demo.yaml         # ESP32-S3 variant
├── web/
│   ├── relay_dashboard.js         # Web control panel (bilingual)
│   └── relay_dashboard.css        # Panel styles
├── 继电器控制板-485接口说明.md     # Full Modbus register reference (Chinese)
├── secrets.example.yaml           # WiFi config template
├── README.md                      # Chinese README
└── LICENSE                        # MIT License
```

---

## 4. Quick Start

### 1. Install ESPHome

```bash
pip install esphome
# or use the official Docker image / ESPHome Dashboard
```

### 2. Configure WiFi

```bash
cp secrets.example.yaml secrets.yaml
```

Edit `secrets.yaml`:

```yaml
wifi_ssid: "your-ssid"
wifi_password: "your-password"
```

> `secrets.yaml` is git-ignored and never committed.

### 3. Validate

```bash
esphome config my-iot-relay-c3-demo.yaml
```

### 4. Compile & Flash

```bash
esphome run my-iot-relay-c3-demo.yaml
```

Flash over USB the first time; afterwards use OTA or the in-page firmware upload.

### 5. Open the Web Panel

Browse to `http://<device-ip>/` once the device joins your WiFi.

If it cannot connect, it enters AP mode (hotspot: `relay-c3-demo Fallback Hotspot`).

---

## 5. Web Panel

| Section | Function |
|---------|----------|
| Network Info | IP / MAC / uptime |
| Relay Control | 16 switches, all on/off, delayed action |
| Performance Test | relay selection, cyclic test, cycles / duration / interval |
| Settings | module baud, ESP 485 baud, power-on mode, pickup delay |
| Status Read | read-only status + manual read |
| Relay Self-test | self-test trigger + wiring hint |
| Firmware Update (OTA) | upload .bin |

> Switch language in the top-right corner; the choice is remembered in your browser.

---

## 6. Register Map Quick Reference

Full details: [继电器控制板-485接口说明.md](继电器控制板-485接口说明.md) (Chinese).

**Write (function code 0x06):**

| Address | Function |
|---------|----------|
| 0 ~ 15 | Single relay on/off (0=off / 1=on) |
| 400 ~ 415 | Delayed relay action (value × 0.1 s) |
| 51 | Set baud rate (1/3/4/6/7) |
| 52 | All relays (0=all off / 1=all on) |
| 202 | Pickup delay (μs, ≤21666, saved) |
| 204 | Power-on mode (0/1/2) |
| 999 | Relay delay self-test |

**Read (function code 0x03 / 0x04):**

| Address | Content |
|---------|---------|
| 0 ~ 15 | Relay states |
| 200 | Firmware version |
| 201 | AC period (μs) |
| 202 | Pickup delay (μs) |
| 203 | DC current (mA) |
| 204 | Power-on mode |
| 205 | Config storage index |

---

## 7. Real-World Integration

The relay board is a standard **Modbus RTU slave** and can be used independently:

1. **PLC / industrial systems**: write relays with FC 0x06 and read status with FC 0x03.
2. **Home Assistant**: import the entities (switch / sensor / select / button) from `my-iot-relay-c3-demo.yaml`.
3. **Custom IoT gateway**: read/write relays directly via the register map.
4. **Custom UI**: `web/relay_dashboard.js` uses the ESPHome REST API + SSE; adapt it to your own frontend.

---

## 8. Troubleshooting

- **Lost connection after changing module baud**: switch the ESP 485 baud to match via Settings, then set both sides consistently.
- **All Off button does nothing**: it now directly sends the all-off command (write register 52 = 0), independent of switch state.
- **Status not updating**: the module does not support multi-register reads; all reads are single-register.

---

## 9. License

Code is released under the [MIT License](LICENSE). The relay board hardware and Modbus protocol belong to their respective owners.
