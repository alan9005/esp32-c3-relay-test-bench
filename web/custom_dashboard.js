// =============================================================================
// HuiFlow Mainboard ESP32-S3 -- Custom Web Dashboard JavaScript
// Real-time SCADA/HMI interface via ESPHome web_server v2
// =============================================================================
// State updates: EventSource (SSE) from /events (ESPHome built-in)
// Control actions: REST API (/switch/xxx/toggle, /button/xxx/press, etc.)
// =============================================================================

(function () {
  'use strict';

  // ===========================================================================
  // Configuration Constants
  // ===========================================================================
  var FIRMWARE_VERSION = 'v3.15.8';
  var RECONNECT_DELAY_MS = 3000;
  var RECONNECT_MAX_DELAY_MS = 30000;
  var LOADING_HIDE_DELAY_MS = 800;
  var ESP_BASE_URL = '';  // empty = same host, or 'http://192.168.1.10'

  // ===========================================================================
  // Entity ID Mappings
  // These must match the ESPHome YAML configuration entity IDs exactly.
  // The event data arrives as: { id: "sensor-system_uptime", value: "12345" }
  // ===========================================================================

  // --- Text Sensors ---
  var TEXT_SENSOR_IDS = {
    serial: 'text_sensor-device_serial',
    wsLastJob: 'text_sensor-ws_last_job',
    wsConnection: 'text_sensor-ws_connection',
    wsLanConnection: 'text_sensor-ws_lan_connection',
    wsLastRecv: 'text_sensor-ws_last_recv',
    wsLastStatus: 'text_sensor-ws_last_status',
    wsLastSent: 'text_sensor-ws_last_sent',
    wsLog: 'text_sensor-ws_log',
    resetReason: 'text_sensor-reset_reason',
    otaStatus: 'text_sensor-ota_status',
    apiResult: 'text_sensor-api_response_result',
    eepromResult: 'text_sensor-eeprom_result',
    climateStatus: 'text_sensor-climate_status',
    uart2Monitor: 'text_sensor-uart2_monitor',
    wsLanServerInfo: 'text_sensor-ws_lan_server_info'  // LAN WS 伺服器地址
  };

  // UART2 log ring buffer (max 30 entries)
  var UART2_LOG_MAX = 30;
  var uart2LogEntries = [];

  // WS1 (雲端) / WS2 (LAN) 收發日誌 ring buffer
  var WS_PANEL_LOG_MAX = 80;
  var ws1LogEntries = [];
  var ws2LogEntries = [];

  // Preset LED color buttons for quick color selection
  // Values are RGB 0-255, sent to ESPHome REST API with effect:"Solid"
  var LED_COLORS = [
    { name: '\u7D05', r: 255, g: 0,   b: 0   },
    { name: '\u7DA0', r: 0,   g: 255, b: 0   },
    { name: '\u85CD', r: 0,   g: 0,   b: 255 },
    { name: '\u767D', r: 255, g: 255, b: 255 },
    { name: '\u9EC3', r: 255, g: 255, b: 0   },
    { name: '\u6A58', r: 255, g: 165, b: 0   },
    { name: '\u7D2B', r: 128, g: 0,   b: 255 },
    { name: '\u9752', r: 0,   g: 255, b: 255 },
    { name: '\u7C89', r: 255, g: 105, b: 180 },
    { name: '\u6696\u767D', r: 255, g: 200, b: 120 }
  ];

  // --- Numeric Sensors ---
  // 感測器數據按類別分組排列（相關的靠近）
  var SENSOR_DEFS = [
    // ── 環境感測 ──
    { key: 'temperature',    id: 'sensor-temperature',                  label: '温度',           unit: '\u00B0C', icon: 'TMP', format: 'f1' },
    { key: 'humidity',       id: 'sensor-humidity',                     label: '湿度',           unit: '%',     icon: 'HUM', format: 'f1' },
    // ── 水系統 ──
    { key: 'water_pressure', id: 'sensor-water_pressure',               label: '水压',           unit: 'mPa',   icon: 'WPR', format: 'f1' },
    { key: 'water_flow_in',  id: 'sensor-water_flow_in',                label: '入水流量',       unit: 'L/min', icon: 'IN',  format: 'div100' },
    { key: 'water_flow_out', id: 'sensor-water_flow_out',               label: '出水流量',       unit: 'L/min', icon: 'OUT', format: 'div100' },
    { key: 'total_flow',     id: 'sensor-total_water_flow',             label: '入水累计总流量', unit: 'L',     icon: 'FLW', format: 'f2' },
    // ── 電力 ──
    { key: 'current_total',  id: 'sensor-total_current',                label: '总电流',         unit: 'A',     icon: 'AMP', format: 'div100' },
    { key: 'current_pump',   id: 'sensor-pump_current',                 label: '水泵电流',       unit: 'A',     icon: 'PMP', format: 'div100' },
    { key: 'current_air',    id: 'sensor-air_pump_current',             label: '气泵电流',       unit: 'A',     icon: 'AIR', format: 'div100' },
    // ── 風扇 ──
    { key: 'fan_speed',      id: 'sensor-fan_speed',                    label: '风扇速度',       unit: 'RPM',   icon: 'FAN', format: 'f0' },
    // ── 系統運行 ──
    { key: 'uptime',         id: 'sensor-system_uptime',                label: '运行时间',       unit: '',      icon: 'CLK', format: 'uptime' },
    { key: 'reboot_count',  id: 'sensor-reboot_count',                 label: '重启次数',       unit: '次',    icon: 'RST', format: 'f0' },
    { key: 'reset_reason',  id: 'text_sensor-reset_reason',            label: '重启原因',       unit: '',      icon: 'RSN', format: 'text' },
    { key: 'last_uptime',   id: 'sensor-last_boot_uptime',             label: '上次运行时长',   unit: '',      icon: 'LST', format: 'uptime' },
    // ── 記憶體 / 效能 ──
    { key: 'free_heap',     id: 'sensor-free_heap',                    label: '可用内存',       unit: '',      icon: 'MEM', format: 'heap' },
    { key: 'min_free_heap', id: 'sensor-min_free_heap',               label: '最低内存',       unit: '',      icon: 'MIN', format: 'heap' },
    { key: 'cpu_freq',     id: 'sensor-cpu_freq',                     label: 'CPU频率',        unit: 'MHz',   icon: 'CPU', format: 'f0' },
    { key: 'cpu_usage',    id: 'sensor-cpu_usage',                    label: 'CPU使用率',      unit: '%',     icon: 'USE', format: 'f1' },
    { key: 'loop_time',    id: 'sensor-loop_time',                    label: '循环耗时',       unit: '',      icon: 'LOP', format: 'us' },
    { key: 'loop_max',     id: 'sensor-loop_time_max',                label: '循环峰值',       unit: '',      icon: 'LMX', format: 'us' },
    { key: 'crash_pc',      id: 'text_sensor-crash_pc',                label: 'Crash PC',      unit: '',      icon: 'BUG', format: 'text' }
  ];

  // --- Binary Sensors ---
  var BINARY_SENSOR_DEFS = [
    { key: 'ac_power',       id: 'binary_sensor-ac_power',              label: '01-外接电源',       icon: 'PWR' },
    { key: 'leak_detect',    id: 'binary_sensor-leak_detection',        label: '02-漏水检测',       icon: 'LKD' },
    { key: 'soft_water',     id: 'binary_sensor-soft_water_level',      label: '03-软水箱水位',     icon: 'SWL' },
    { key: 'water_tank',     id: 'binary_sensor-water_tank_level',      label: '04-水箱水位',       icon: 'WTL' },
    { key: 'led_blue',       id: 'binary_sensor-washer_led_blue',       label: '05-洗车机蓝色灯',   icon: 'BLU' },
    { key: 'led_green',      id: 'binary_sensor-washer_led_green',      label: '06-洗车机绿色灯',   icon: 'GRN' },
    { key: 'led_red',        id: 'binary_sensor-washer_led_red',        label: '07-洗车机红色灯',   icon: 'RED' },
    { key: 'emergency',      id: 'binary_sensor-emergency_button',      label: '08-紧急按钮',       icon: 'EMG' },
    { key: 'start_stop',     id: 'binary_sensor-start_stop_button',     label: '09-启动停止',       icon: 'RUN' },
    { key: 'car_height',     id: 'binary_sensor-car_height_detection',  label: '10-车辆超高',       icon: 'CAR' },
    { key: 'bubble_1',       id: 'binary_sensor-bubble_liquid_1',       label: '21-泡泡液1',        icon: 'BB1' },
    { key: 'bubble_2',       id: 'binary_sensor-bubble_liquid_2',       label: '22-泡泡液2',        icon: 'BB2' },
    { key: 'bubble_3',       id: 'binary_sensor-bubble_liquid_3',       label: '23-泡泡液3',        icon: 'BB3' },
    { key: 'bubble_4',       id: 'binary_sensor-bubble_liquid_4',       label: '24-泡泡液4',        icon: 'BB4' },
    { key: 'door_sensor',    id: 'binary_sensor-door_sensor',           label: '25-门磁',           icon: 'DOR' }
  ];

  // --- Switches (Relays + SSR) ---
  var SWITCH_DEFS = [
    { id: 'switch-spare_io',      name: 'R01',  label: '備用',       type: 'relay' },
    { id: 'switch-pressure',      name: 'R02',  label: '加壓回洗',   type: 'relay' },
    { id: 'switch-soft_water',    name: 'R03',  label: '軟水切換',   type: 'relay' },
    { id: 'switch-filter_clean',  name: 'R04',  label: '濾芯回洗',   type: 'relay' },
    { id: 'switch-pwr_led',       name: 'R05',  label: 'LED螢幕電源', type: 'relay' },
    { id: 'switch-pwr_other',     name: 'R06',  label: '燈條/泡沫/販賣機', type: 'relay' },
    { id: 'switch-pwr_pump',      name: 'R07',  label: '加壓馬達',   type: 'relay' },
    { id: 'switch-pwr_washer',    name: 'R08',  label: '洗車機電源', type: 'relay' },
    { id: 'switch-bb1',           name: 'R09',  label: '泡沫選擇1', type: 'relay' },
    { id: 'switch-bb2',           name: 'R10',  label: '泡沫選擇2', type: 'relay' },
    { id: 'switch-bb3',           name: 'R11',  label: '泡沫選擇3', type: 'relay' },
    { id: 'switch-bb4',           name: 'R12',  label: '泡沫選擇4', type: 'relay' },
    { id: 'switch-gate',          name: 'R13',  label: '閘門機',     type: 'relay' },
    { id: 'switch-indoor_light1',  name: 'R14',  label: '室內燈1',   type: 'relay' },
    { id: 'switch-indoor_light2',  name: 'R15',  label: '室內燈2',   type: 'relay' },
    { id: 'switch-indoor_light3',  name: 'R16',  label: '室內燈3',   type: 'relay' },
    { id: 'switch-ssr_1', name: 'SSR1', label: 'SSR 1',   type: 'ssr'   },
    { id: 'switch-ssr_2', name: 'SSR2', label: 'SSR 2',   type: 'ssr'   },
    { id: 'switch-ssr_3', name: 'SSR3', label: 'SSR 3',   type: 'ssr'   },
    { id: 'switch-ssr_4', name: 'SSR4', label: 'SSR 4',   type: 'ssr'   }
  ];

  // --- Light Entities (WS2812 LED strips) ---
  // key: internal identifier for DOM element IDs (lc-light1, lc-state-light1, etc.)
  // id:  ESPHome SSE entity ID (light-{object_id}), used to match incoming state events
  // name: ESPHome object_id, used in REST API URLs (/light/{name}/turn_on)
  var LIGHT_DEFS = [
    { key: 'light1', id: 'light-status_light_1', name: 'status_light_1', label: 'LED1 狀態燈 (GPIO5, 30顆)' },
    { key: 'light2', id: 'light-status_light_2', name: 'status_light_2', label: 'LED2 狀態燈 (GPIO6, 30顆)' }
  ];
  // Available addressable_lambda effects (must match YAML effect names exactly)
  var LIGHT_EFFECTS = ['None', 'Solid', 'Rainbow', 'Color Wipe', 'Scanner', 'Twinkle', 'Fire', 'Breathing', 'Chase', 'Strobe', 'Running'];

  // --- Fan Entity (PWM cooling fan on GPIO37, LEDC 25kHz) ---
  // id:   SSE entity ID for state matching
  // name: object_id for REST API URL (/fan/{name}/toggle)
  var FAN_DEF = { id: 'fan-cooling_fan', name: 'cooling_fan', label: 'Cooling Fan' };

  // ===========================================================================
  // Runtime State
  // ===========================================================================
  var stateCache = {};       // id -> { value, state, ... }
  var evtSource = null;      // EventSource (SSE) from /events
  var isConnected = false;
  var reconnectDelay = RECONNECT_DELAY_MS;
  var wsFailCount = 0;
  var reconnectTimer = null;
  var lastEventTime = null;
  var dashboardBuilt = false;

  // Build lookup maps for fast access
  var sensorById = {};
  var binaryById = {};
  var switchById = {};
  var switchByIndex = {};

  (function buildLookups() {
    var i;
    for (i = 0; i < SENSOR_DEFS.length; i++) {
      sensorById[SENSOR_DEFS[i].id] = SENSOR_DEFS[i];
    }
    for (i = 0; i < BINARY_SENSOR_DEFS.length; i++) {
      binaryById[BINARY_SENSOR_DEFS[i].id] = BINARY_SENSOR_DEFS[i];
    }
    for (i = 0; i < SWITCH_DEFS.length; i++) {
      switchById[SWITCH_DEFS[i].id] = SWITCH_DEFS[i];
      switchByIndex[i] = SWITCH_DEFS[i];
    }
  })();

  // ===========================================================================
  // Utility Functions
  // ===========================================================================

  /**
   * Format seconds into human-readable "Xd Xh Xm Xs".
   */
  function formatUptime(totalSeconds) {
    var secs = Math.floor(parseFloat(totalSeconds) || 0);
    var d = Math.floor(secs / 86400);
    var h = Math.floor((secs % 86400) / 3600);
    var m = Math.floor((secs % 3600) / 60);
    var s = secs % 60;
    var parts = [];
    if (d > 0) parts.push(d + 'd');
    if (h > 0 || d > 0) parts.push(h + 'h');
    if (m > 0 || h > 0 || d > 0) parts.push(m + 'm');
    parts.push(s + 's');
    return parts.join(' ');
  }

  /**
   * Format a sensor value based on its format type.
   */
  function formatSensorValue(rawValue, formatType) {
    if (formatType === 'text') return rawValue || '--';
    var num = parseFloat(rawValue);
    if (isNaN(num)) return rawValue || '--';
    switch (formatType) {
      case 'uptime':  return formatUptime(num);
      case 'div100':  return (num / 100).toFixed(2);
      case 'f0':      return Math.round(num).toString();
      case 'f1':      return num.toFixed(1);
      case 'f2':      return num.toFixed(2);
      case 'heap':    return num >= 1048576 ? (num/1048576).toFixed(1) + ' MB' : num >= 1024 ? (num/1024).toFixed(0) + ' KB' : num + ' B';
      case 'us':      return num >= 1000 ? (num/1000).toFixed(1) + ' ms' : Math.round(num) + ' us';
      default:        return num.toString();
    }
  }

  /**
   * Determine if a value represents the ON state.
   */
  function isOn(val) {
    if (val === undefined || val === null) return null;
    var s = String(val).toUpperCase();
    return s === 'ON' || s === 'TRUE' || s === '1';
  }

  /**
   * Get current time as HH:MM:SS string.
   */
  function timeString() {
    var now = new Date();
    var hh = String(now.getHours()).padStart(2, '0');
    var mm = String(now.getMinutes()).padStart(2, '0');
    var ss = String(now.getSeconds()).padStart(2, '0');
    return hh + ':' + mm + ':' + ss;
  }

  /**
   * Safely set text content of an element by ID.
   */
  function setText(id, text) {
    var el = document.getElementById(id);
    if (el) el.textContent = text;
  }

  /**
   * Add and auto-remove a CSS class for flash animation.
   */
  function flashElement(el) {
    if (!el) return;
    el.classList.remove('value-flash');
    // Force reflow so the animation restarts
    void el.offsetWidth;
    el.classList.add('value-flash');
  }

  // ===========================================================================
  // WebSocket Log Helper — WS1(雲端) / WS2(LAN) 統一收發日誌
  // ===========================================================================

  function escapeHtml(str) {
    var div = document.createElement('div');
    div.textContent = str;
    return div.innerHTML;
  }

  /**
   * 根據 WS 連線狀態字串回傳 CSS class
   * Connected → 藍色(WS1)/綠色(WS2) | Connecting/Link UP/Waiting → 黃色 | 其他 → 紅色
   */
  function getConnStatusClass(value, isWs1) {
    if (value === 'Connected') return isWs1 ? 'status-connected-ws1' : 'status-connected';
    if (value && (value.indexOf('onnecting') !== -1 || value.indexOf('Link UP') !== -1 || value.indexOf('Waiting') !== -1)) return 'status-connecting';
    return 'status-disconnected';
  }

  /**
   * 追加 WS 面板日誌（統一函數）
   * @param {number} panel - 1=WS1, 2=WS2
   * @param {string} text - 顯示文字
   * @param {string} dir - 'sent'=綠色 | 'recv'=藍色 | 'cmd_ok'=黃色 | 'info'=灰色 | 'error'=紅色（EMG 送出自動標紅）
   */
  function appendWsPanelLog(panel, text, dir) {
    if (!text) return;
    var ts = timeString();
    var entries = panel === 1 ? ws1LogEntries : ws2LogEntries;
    entries.push({ time: ts, text: text, dir: dir || 'info' });
    if (entries.length > WS_PANEL_LOG_MAX) entries.shift();
    renderWsPanelLog(panel);
  }

  function renderWsPanelLog(panel) {
    var containerId = panel === 1 ? 'ws1-log' : 'ws2-log';
    var entries = panel === 1 ? ws1LogEntries : ws2LogEntries;
    var container = document.getElementById(containerId);
    if (!container) return;
    // sent=綠色, recv=藍色（用戶要求），cmd_ok=黃, info=灰, error=紅
    var colorMap = { sent: '#22c55e', recv: '#3b82f6', cmd_ok: '#facc15', info: '#9ca3af', error: '#ef4444' };
    var prefixMap = { sent: '\u2191 ', recv: '\u2193 ', cmd_ok: '\u2713 ', info: '', error: '\u2716 ' };
    var html = '';
    for (var i = entries.length - 1; i >= 0; i--) {
      var e = entries[i];
      var c = colorMap[e.dir] || '#9ca3af';
      var p = prefixMap[e.dir] || '';
      // 送出 EVENT 訊息 → 紅色高亮（重要事件：IO 變化、go/emg trigger）
      if (e.dir === 'sent' && e.text && e.text.indexOf('"type":"EVENT"') !== -1) {
        c = '#ef4444';
      }
      html += '<div style="margin-bottom:1px"><span class="ws-log-time">' + e.time + '</span> <span style="color:' + c + '">' + p + escapeHtml(e.text) + '</span></div>';
    }
    if (!html) html = '<div class="ws-log-empty">\u5C1A\u7121\u65E5\u8A8C</div>';
    container.innerHTML = html;
  }

  function appendUart2Log(rawValue) {
    if (!rawValue) return;
    var ts = timeString();
    var isTx = rawValue.indexOf('TX>') === 0;
    uart2LogEntries.push({ time: ts, text: rawValue, tx: isTx });
    if (uart2LogEntries.length > UART2_LOG_MAX) uart2LogEntries.shift();
    renderUart2Log();
  }

  function renderUart2Log() {
    var container = document.getElementById('uart2-log');
    if (!container) return;
    var html = '';
    for (var i = uart2LogEntries.length - 1; i >= 0; i--) {
      var e = uart2LogEntries[i];
      var color = e.tx ? '#3b82f6' : '#22c55e';
      html += '<div style="margin-bottom:2px"><span class="ws-log-time">' + e.time + '</span> <span style="color:' + color + '">' + escapeHtml(e.text) + '</span></div>';
    }
    if (!html) html = '<div class="ws-log-empty">\u5C1A\u7121 UART2 \u8CC7\u6599</div>';
    container.innerHTML = html;
  }

  window.__hfUart2Clear = function() {
    uart2LogEntries = [];
    renderUart2Log();
  };


  // ===========================================================================
  // DOM Builder - Loading Overlay
  // ===========================================================================
  function createLoadingOverlay() {
    var overlay = document.createElement('div');
    overlay.id = 'hmi-loading';
    overlay.innerHTML =
      '<div class="spinner"></div>' +
      '<div class="loading-text">Connecting to HuiFlow Controller...</div>';
    return overlay;
  }

  // ===========================================================================
  // DOM Builder - Main Dashboard
  // ===========================================================================
  function buildDashboard() {
    // Remove all existing body children (hides default ESPHome UI)
    var body = document.body;
    while (body.firstChild) {
      body.removeChild(body.firstChild);
    }

    // Add loading overlay
    body.appendChild(createLoadingOverlay());

    // Create main dashboard container
    var dashboard = document.createElement('div');
    dashboard.id = 'hmi-dashboard';

    // --- Header ---
    dashboard.innerHTML = buildHeaderHTML() +
      '<div class="hmi-grid">' +
        buildDeviceInfoCard() +
        buildBinarySensorCard() +
        buildSensorDataCard() +
        buildRelayControlCard() +
        buildLightControlCard() +
        buildFanControlCard() +
        buildTriggerCard() +
        buildUart2Card() +
        buildWsConnectionCard() +
        buildSerialModCard() +
      '</div>' +
      buildFooterHTML();

    body.appendChild(dashboard);
    dashboardBuilt = true;
  }

  // --- Header ---
  function buildHeaderHTML() {
    return '' +
      '<div class="hmi-header">' +
        '<div class="title-group">' +
          '<div class="logo-icon">HF</div>' +
          '<h1>HuiFlow IoT Washer' +
            '<span class="sub">ESP32-S3 Smart Car Wash Controller</span>' +
          '</h1>' +
        '</div>' +
        '<div class="header-right">' +
          '<span id="conn-badge" class="connection-badge disconnected">' +
            '<span class="connection-dot"></span> OFFLINE' +
          '</span>' +
          '<div style="margin-left:auto;display:flex;align-items:center;gap:4px">' +
            '<label style="font-size:0.65rem;color:var(--text-muted)">ESP IP:</label>' +
            '<input type="text" id="sse-host-input" style="width:120px;font-size:0.7rem;padding:2px 6px;background:var(--bg-dark);color:var(--text-primary);border:1px solid var(--border-color);border-radius:4px" ' +
              'value="" placeholder="192.168.1.10" />' +
            '<button class="action-btn action-btn-blue" onclick="window.__hfSetHost()" style="font-size:0.65rem;padding:2px 6px">Set</button>' +
          '</div>' +
        '</div>' +
      '</div>';
  }

  // --- Section 1: Device Info ---
  function buildDeviceInfoCard() {
    return '' +
      '<div class="hmi-card card-full-width">' +
        '<div class="hmi-card-header">' +
          '<div class="card-icon blue">i</div>' +
          '<div class="card-title">设备信息 <small>Device Information</small></div>' +
        '</div>' +
        '<div class="hmi-card-body">' +
          '<div class="device-info-grid">' +
            infoItem('device-serial', '\u5E8F\u5217\u865F / Serial', '--') +
            infoItem('device-firmware', '\u56FA\u4EF6\u7248\u672C / Firmware', FIRMWARE_VERSION) +
            infoItem('device-uptime', '\u904B\u884C\u6642\u9593 / Uptime', '--') +
            infoItem('device-last-update', '\u6700\u5F8C\u66F4\u65B0 / Last Update', '--') +
            infoItem('device-ip', 'IP Address', window.location.hostname || '--') +
          '</div>' +
          // OTA / Web Server / Reboot 控制列
          '<div style="display:flex;gap:6px;flex-wrap:wrap;margin-top:10px;padding-top:10px;border-top:1px solid var(--border-color);align-items:center">' +
            '<button class="action-btn action-btn-blue" onclick="window.__hfOtaCheck()" style="font-size:0.7rem">\u6AA2\u67E5 OTA</button>' +
            '<div id="ota-status-display" style="flex:1;min-width:100px;font-size:0.7rem;color:var(--text-muted);font-family:monospace"></div>' +
            '<div id="ota-progress-bar-wrap" style="display:none;width:120px;height:6px;background:#333;border-radius:4px;overflow:hidden">' +
              '<div id="ota-progress-bar" style="width:0%;height:100%;background:linear-gradient(90deg,#0066ff,#00cc66);border-radius:4px;transition:width 0.3s"></div>' +
            '</div>' +
            '<div style="border-left:1px solid var(--border-color);height:24px;margin:0 4px"></div>' +
            '<button class="action-btn action-btn-green" onclick="window.__hfWebServerOn()" style="font-size:0.65rem">Web ON</button>' +
            '<button class="action-btn action-btn-red" onclick="window.__hfWebServerOff()" style="font-size:0.65rem">Web OFF</button>' +
            '<div style="border-left:1px solid var(--border-color);height:24px;margin:0 4px"></div>' +
            '<button class="action-btn" onclick="window.__hfReboot()" style="font-size:0.65rem;background:#d33">\u91CD\u555F</button>' +
          '</div>' +
        '</div>' +
      '</div>';
  }

  function infoItem(id, label, value) {
    return '' +
      '<div class="info-item">' +
        '<div class="info-label">' + label + '</div>' +
        '<div class="info-value mono" id="' + id + '">' + value + '</div>' +
      '</div>';
  }

  // --- Section 2: Binary Sensors ---
  function buildBinarySensorCard() {
    var items = '';
    for (var i = 0; i < BINARY_SENSOR_DEFS.length; i++) {
      var def = BINARY_SENSOR_DEFS[i];
      items += '' +
        '<div class="binary-item" id="bs-' + def.key + '">' +
          '<div class="binary-dot unknown"></div>' +
          '<div class="binary-name">' + def.label + '</div>' +
          '<div class="binary-state unknown">--</div>' +
        '</div>';
    }
    return '' +
      '<div class="hmi-card card-full-width">' +
        '<div class="hmi-card-header">' +
          '<div class="card-icon green">S</div>' +
          '<div class="card-title">状态输入 <small>Binary Sensors / Status Inputs</small></div>' +
        '</div>' +
        '<div class="hmi-card-body">' +
          '<div class="binary-sensors-grid">' + items + '</div>' +
        '</div>' +
      '</div>';
  }

  // --- Section 3: Sensor Data ---
  function buildSensorDataCard() {
    var items = '';
    for (var i = 0; i < SENSOR_DEFS.length; i++) {
      var def = SENSOR_DEFS[i];
      items += '' +
        '<div class="sensor-item" id="sd-' + def.key + '-wrap">' +
          '<div class="sensor-label">' + def.label + '</div>' +
          '<div>' +
            '<span class="sensor-value" id="sd-' + def.key + '">--</span>' +
            '<span class="sensor-unit">' + def.unit + '</span>' +
          '</div>' +
        '</div>';
    }
    return '' +
      '<div class="hmi-card card-full-width">' +
        '<div class="hmi-card-header">' +
          '<div class="card-icon cyan">D</div>' +
          '<div class="card-title">传感器数据 <small>Sensor Data</small></div>' +
        '</div>' +
        '<div class="hmi-card-body">' +
          '<div class="sensor-data-grid">' + items + '</div>' +
        '</div>' +
      '</div>';
  }

  // --- Section 4: Relay Control ---
  function buildRelayControlCard() {
    var relayItems = '';
    var ssrItems = '';
    for (var i = 0; i < SWITCH_DEFS.length; i++) {
      var sw = SWITCH_DEFS[i];
      var itemClass = 'relay-item' + (sw.type === 'ssr' ? ' ssr-type' : '');
      var html = '' +
        '<div class="' + itemClass + '" id="sw-' + i + '">' +
          '<div class="relay-name">' + sw.name + ' ' + sw.label + '</div>' +
          '<button class="relay-toggle" id="sw-btn-' + i + '" ' +
            'onclick="window.__hfToggle(' + i + ')" ' +
            'title="Toggle ' + sw.name + '"></button>' +
          '<div class="relay-state-label unknown" id="sw-state-' + i + '">--</div>' +
        '</div>';
      if (sw.type === 'ssr') {
        ssrItems += html;
      } else {
        relayItems += html;
      }
    }

    return '' +
      '<div class="hmi-card card-full-width">' +
        '<div class="hmi-card-header">' +
          '<div class="card-icon amber">R</div>' +
          '<div class="card-title">继电器控制 <small>Relay & SSR Control</small></div>' +
        '</div>' +
        '<div class="hmi-card-body">' +
          '<div class="relay-grid">' +
            relayItems +
            '<div class="section-divider"><span>SSR Outputs</span></div>' +
            ssrItems +
          '</div>' +
        '</div>' +
      '</div>';
  }

  // --- Section 5: LED Strip Control ---
  // Builds the LED1/LED2 control panel with:
  //   - ON/OFF toggle button (REST API: /light/{name}/toggle)
  //   - LED count slider (number entity: led_count_1/led_count_2)
  //   - Brightness slider (REST API: /light/{name}/turn_on {brightness:N})
  //   - Effect dropdown (REST API: /light/{name}/turn_on {effect:"name"})
  //   - Color preset buttons (REST API: /light/{name}/turn_on {r,g,b,effect:"Solid"})
  function buildLightControlCard() {
    var items = '';
    for (var i = 0; i < LIGHT_DEFS.length; i++) {
      var ld = LIGHT_DEFS[i];
      // Build effect dropdown options (must match YAML effect names)
      var effectOptions = '';
      for (var j = 0; j < LIGHT_EFFECTS.length; j++) {
        effectOptions += '<option value="' + LIGHT_EFFECTS[j] + '">' + LIGHT_EFFECTS[j] + '</option>';
      }
      // Map light index to the corresponding ESPHome number entity
      var countId = (i === 0) ? 'led_count_1' : 'led_count_2';
      // Build color preset buttons
      var colorBtns = '';
      for (var c = 0; c < LED_COLORS.length; c++) {
        var clr = LED_COLORS[c];
        var bgColor = 'rgb(' + clr.r + ',' + clr.g + ',' + clr.b + ')';
        var textColor = (clr.r + clr.g + clr.b > 400) ? '#000' : '#fff';
        colorBtns += '<button class="led-color-btn" style="background:' + bgColor + ';color:' + textColor + '" ' +
          'onclick="window.__hfLightColor(\'' + ld.name + '\',' + clr.r + ',' + clr.g + ',' + clr.b + ')" ' +
          'title="' + clr.name + '">' + clr.name + '</button>';
      }
      items +=
        '<div class="light-control-item" id="lc-' + ld.key + '">' +
          '<div class="light-header">' +
            '<span class="light-label">' + ld.label + '</span>' +
            '<span class="light-state" id="lc-state-' + ld.key + '">--</span>' +
            // Toggle button: text/class updated by updateLightDisplay()
            '<button class="relay-toggle off" id="lc-btn-' + ld.key + '" ' +
              'onclick="window.__hfLightToggle(\'' + ld.name + '\')">OFF</button>' +
          '</div>' +
          '<div class="light-controls">' +
            // 燈珠數: active LED count (1~30, matches physical hardware)
            '<label>\u71C8\u73E0\u6578: <input type="number" min="1" max="30" value="30" ' +
              'id="lc-count-' + ld.key + '" style="width:60px" ' +
              'oninput="window.__hfLedCount(\'' + countId + '\', this.value)">' +
              '</label>' +
            // 亮度: brightness slider 0~255
            '<label>\u4EAE\u5EA6: <input type="range" min="0" max="255" value="128" ' +
              'id="lc-bri-' + ld.key + '" ' +
              'onchange="window.__hfLightBri(\'' + ld.name + '\', this.value)">' +
              '<span id="lc-bri-val-' + ld.key + '">128</span></label>' +
            // 效果: effect selector
            '<label>\u6548\u679C: <select id="lc-eff-' + ld.key + '" ' +
              'onchange="window.__hfLightEff(\'' + ld.name + '\', this.value)">' +
              effectOptions + '</select></label>' +
          '</div>' +
          '<div class="light-colors">' +
            '<span class="color-label">\u984F\u8272:</span>' +
            colorBtns +
          '</div>' +
        '</div>';
    }
    return '' +
      '<div class="hmi-card card-full-width">' +
        '<div class="hmi-card-header">' +
          '<div class="card-icon magenta">L</div>' +
          '<div class="card-title">LED\u706F\u5E26\u63A7\u5236 <small>LED Strip Control</small></div>' +
        '</div>' +
        '<div class="hmi-card-body">' + items + '</div>' +
      '</div>';
  }

  // --- Section 5b: Fan Control ---
  // PWM cooling fan (GPIO37, LEDC 25kHz, speed_count=100)
  // Toggle: POST /fan/cooling_fan/toggle
  // Speed:  POST /fan/cooling_fan/turn_on { speed_level: 0-100 }
  // State updates via SSE entity: fan-cooling_fan
  function buildFanControlCard() {
    return '' +
      '<div class="hmi-card card-full-width">' +
        '<div class="hmi-card-header">' +
          '<div class="card-icon teal">F</div>' +
          '<div class="card-title">\u98CE\u6247\u63A7\u5236 <small>Fan & Climate</small></div>' +
        '</div>' +
        '<div class="hmi-card-body">' +
          // 閉環溫控大按鈕（與上方 relay toggle 同風格）
          '<div style="display:flex;align-items:center;gap:10px;margin-bottom:10px;padding:8px;background:var(--bg-dark);border-radius:6px">' +
            '<span style="font-size:0.78rem;font-weight:600">\u9589\u74B0\u6EAB\u63A7 (35\u00B0C / 20min / 3min)</span>' +
            '<div style="flex:1"></div>' +
            '<span id="climate-fan-state" style="font-size:0.72rem;font-weight:700;color:var(--text-muted)">-- \u5F85\u6A5F</span>' +
            '<span id="climate-fan-timer" style="font-size:0.65rem;color:var(--text-muted);min-width:60px"></span>' +
            '<button class="relay-toggle off" id="climate-btn" ' +
              'onclick="window.__hfClimateToggle()" ' +
              'style="min-width:60px">OFF</button>' +
          '</div>' +
          // 風扇手動控制
          '<div class="fan-control-item" id="fan-ctrl">' +
            '<div class="fan-header">' +
              '<span class="light-label">' + FAN_DEF.label + '</span>' +
              '<span class="light-state" id="fan-state">--</span>' +
              '<button class="relay-toggle off" id="fan-btn" ' +
                'onclick="window.__hfFanToggle()">OFF</button>' +
            '</div>' +
            '<div class="light-controls">' +
              // PID 當前速度指示 + 手動速度滑桿
              '<label style="display:flex;align-items:center;gap:6px">' +
                '<span>\u901F\u5EA6:</span>' +
                '<input type="range" min="0" max="100" value="50" id="fan-speed" ' +
                  'onchange="window.__hfFanSpeed(this.value)" style="flex:1">' +
                '<span id="fan-speed-val">50</span>%' +
                '<span id="climate-pid-speed" style="margin-left:8px;font-size:0.65rem;color:#8b5cf6;min-width:70px">(PID: --%)</span>' +
              '</label>' +
            '</div>' +
          '</div>' +
        '</div>' +
      '</div>';
  }

  // --- Combined WS Connection Card (WS1 left, WS2 right) ---
  // WS1 = 雲端 TLS (wss.huiflow.com:443), WS2 = LAN TCP (192.168.1.10:8090)
  // 各自獨立顯示：連線狀態 + 收發日誌
  function buildWsConnectionCard() {
    // 每側結構：標題 → 狀態 → 日誌
    function wsPanel(num, title, server) {
      var pre = num === 1 ? 'ws1' : 'ws2';
      return '' +
        '<div style="flex:1;min-width:280px">' +
          '<div style="font-size:0.78rem;font-weight:700;margin-bottom:6px;color:' + (num === 1 ? '#4fc3f7' : '#4ade80') + '">' + title + '</div>' +
          '<div style="font-size:0.65rem;color:var(--text-muted);margin-bottom:6px">' + server + '</div>' +
          '<div style="display:flex;align-items:center;gap:6px;margin-bottom:8px">' +
            '<span style="font-size:0.7rem;color:var(--text-muted)">\u72C0\u614B:</span>' +
            '<span class="aws-value status-disconnected" id="' + pre + '-conn" style="font-size:0.75rem">\u7B49\u5F85\u9023\u7DDA...</span>' +
          '</div>' +
          '<div style="font-size:0.65rem;color:var(--text-muted);margin-bottom:4px">\u6536\u767C\u65E5\u8A8C / Log</div>' +
          '<div class="ws-log-container" id="' + pre + '-log" style="max-height:220px;font-size:0.68rem">' +
            '<div class="ws-log-empty">\u5C1A\u7121\u65E5\u8A8C</div>' +
          '</div>' +
        '</div>';
    }

    return '' +
      '<div class="hmi-card card-full-width">' +
        '<div class="hmi-card-header">' +
          '<div class="card-icon orange">W</div>' +
          '<div class="card-title">WebSocket \u9023\u7DDA <small>WS1 Cloud + WS2 LAN</small></div>' +
        '</div>' +
        '<div class="hmi-card-body">' +
          '<div style="display:flex;gap:16px;flex-wrap:wrap">' +
            wsPanel(1, 'WS1 \u96F2\u7AEF', 'wss://wss.huiflow.com:443 (TLS)') +
            '<div style="width:1px;background:var(--border-color);align-self:stretch;min-height:100px"></div>' +
            wsPanel(2, 'WS2 \u5167\u7DB2', 'ws://192.168.1.10:8090 (TCP)') +
          '</div>' +
        '</div>' +
      '</div>';
  }


  // --- Trigger & Washer Test Card ---
  function buildTriggerCard() {
    return '' +
      '<div class="hmi-card card-full-width">' +
        '<div class="hmi-card-header">' +
          '<div class="card-icon" style="background:#f59e0b">T</div>' +
          '<div class="card-title">\u89F8\u767C\u6E2C\u8A66 <small>Trigger & Washer Test</small></div>' +
        '</div>' +
        '<div class="hmi-card-body">' +
          // go_enable / emergency_enable status
          '<div style="display:flex;gap:8px;margin-bottom:10px;flex-wrap:wrap">' +
            '<div style="flex:1;min-width:140px;padding:8px;background:var(--bg-dark);border-radius:6px">' +
              '<div style="font-size:0.7rem;color:var(--text-muted);margin-bottom:4px">go_enable \u72C0\u614B</div>' +
              '<div id="trigger-go-status" style="font-size:0.85rem;font-weight:700;color:#888">IDLE</div>' +
              '<div id="trigger-go-arg" style="font-size:0.65rem;color:var(--text-muted);margin-top:2px">--</div>' +
            '</div>' +
            '<div style="flex:1;min-width:140px;padding:8px;background:var(--bg-dark);border-radius:6px">' +
              '<div style="font-size:0.7rem;color:var(--text-muted);margin-bottom:4px">emergency_enable \u72C0\u614B</div>' +
              '<div id="trigger-emg-status" style="font-size:0.85rem;font-weight:700;color:#888">IDLE</div>' +
              '<div id="trigger-emg-arg" style="font-size:0.65rem;color:var(--text-muted);margin-top:2px">--</div>' +
            '</div>' +
          '</div>' +
          // Quick trigger buttons
          '<div style="display:flex;gap:4px;flex-wrap:wrap;margin-bottom:8px">' +
            '<button class="action-btn action-btn-green" onclick="window.__hfTriggerGo(\'gowasher\')" style="font-size:0.7rem;padding:4px 8px">Go: gowasher</button>' +
            '<button class="action-btn action-btn-green" onclick="window.__hfTriggerGo(\'lottoTrigger\')" style="font-size:0.7rem;padding:4px 8px">Go: lottoTrigger</button>' +
            '<button class="action-btn action-btn-green" onclick="window.__hfTriggerGo(\'gobtn\')" style="font-size:0.7rem;padding:4px 8px">Go: gobtn</button>' +
            '<button class="action-btn" onclick="window.__hfTriggerEmg(\'gowasher\')" style="font-size:0.7rem;padding:4px 8px;background:#ef4444">Emg: gowasher</button>' +
            '<button class="action-btn" onclick="window.__hfTriggerEmg(\'lottoTrigger\')" style="font-size:0.7rem;padding:4px 8px;background:#ef4444">Emg: lottoTrigger</button>' +
          '</div>' +
          // Washer quick buttons
          '<div style="font-size:0.7rem;color:var(--text-muted);margin-bottom:4px">\u6D17\u8ECA\u6A5F UART2 \u5FEB\u6377\u6309\u9215</div>' +
          '<div style="display:flex;gap:4px;flex-wrap:wrap;margin-bottom:8px">' +
            '<button class="action-btn action-btn-blue" onclick="window.__hfWasherCmd(\'startwasher1\')" style="font-size:0.65rem;padding:3px 6px">SW1</button>' +
            '<button class="action-btn action-btn-blue" onclick="window.__hfWasherCmd(\'startwasher2\')" style="font-size:0.65rem;padding:3px 6px">SW2</button>' +
            '<button class="action-btn action-btn-blue" onclick="window.__hfWasherCmd(\'startwasher3\')" style="font-size:0.65rem;padding:3px 6px">SW3</button>' +
            '<button class="action-btn action-btn-blue" onclick="window.__hfWasherCmd(\'startwasher4\')" style="font-size:0.65rem;padding:3px 6px">SW4</button>' +
            '<button class="action-btn action-btn-blue" onclick="window.__hfWasherCmd(\'startwasher5\')" style="font-size:0.65rem;padding:3px 6px">SW5</button>' +
            '<button class="action-btn action-btn-blue" onclick="window.__hfWasherCmd(\'startwasher6\')" style="font-size:0.65rem;padding:3px 6px">SW6</button>' +
            '<button class="action-btn action-btn-green" onclick="window.__hfWasherCmd(\'gowasher\')" style="font-size:0.65rem;padding:3px 6px">GO</button>' +
            '<button class="action-btn" onclick="window.__hfWasherCmd(\'resetwasher\')" style="font-size:0.65rem;padding:3px 6px;background:#f97316">Reset</button>' +
            '<button class="action-btn" onclick="window.__hfWasherCmd(\'hardresetwasher\')" style="font-size:0.65rem;padding:3px 6px;background:#ef4444">HardReset</button>' +
            '<button class="action-btn" onclick="window.__hfWasherCmd(\'sparewasher\')" style="font-size:0.65rem;padding:3px 6px;background:#6b7280">Spare</button>' +
          '</div>' +
          // System quick buttons
          '<div style="display:flex;gap:4px;flex-wrap:wrap">' +
            '<button class="action-btn" onclick="window.__hfQuickCmd(\'status\')" style="font-size:0.65rem;padding:3px 6px;background:#8b5cf6">Status</button>' +
            '<button class="action-btn" onclick="window.__hfQuickCmd(\'getsn\')" style="font-size:0.65rem;padding:3px 6px;background:#8b5cf6">GetSN</button>' +
            '<button class="action-btn" onclick="window.__hfQuickCmd(\'all_off\')" style="font-size:0.65rem;padding:3px 6px;background:#dc2626">ALL OFF</button>' +
          '</div>' +
          '<div id="trigger-result" style="margin-top:6px;font-size:0.7rem;color:var(--text-muted);font-family:monospace">--</div>' +
        '</div>' +
      '</div>';
  }

  // --- UART2 Monitor Card ---
  function buildUart2Card() {
    return '' +
      '<div class="hmi-card card-full-width">' +
        '<div class="hmi-card-header">' +
          '<div class="card-icon" style="background:#0ea5e9">U</div>' +
          '<div class="card-title">UART2 \u76E3\u63A7 <small>Washer TX/RX</small></div>' +
          '<button class="action-btn" onclick="window.__hfUart2Clear()" style="font-size:0.6rem;padding:2px 8px;margin-left:auto;background:#6b7280">Clear</button>' +
        '</div>' +
        '<div class="hmi-card-body">' +
          '<div id="uart2-log" style="height:160px;overflow-y:auto;background:var(--bg-dark);border-radius:6px;padding:6px;font-family:monospace;font-size:0.7rem;color:var(--text-muted)">' +
            '<div class="ws-log-empty">\u5C1A\u7121 UART2 \u8CC7\u6599</div>' +
          '</div>' +
        '</div>' +
      '</div>';
  }

  // --- Climate Closed-Loop Control Card ---
  // 固定參數：目標溫度 35°C / PID 風速 / 運行 20 分鐘 / 休息 3 分鐘
  // Dashboard 只顯示開關 + 狀態 + 即時數據，不可調整參數

  // --- Section 8: System Log ---

  // --- Section: Serial Number Modification ---
  function buildSerialModCard() {
    return '' +
      '<div class="hmi-card card-full-width">' +
        '<div class="hmi-card-header">' +
          '<div class="card-icon" style="background:var(--accent-red,#ef4444)">#</div>' +
          '<div class="card-title">\u5E8F\u5217\u865F\u4FEE\u6539 <small>Serial Number Modification</small></div>' +
        '</div>' +
        '<div class="hmi-card-body">' +
          '<div style="display:flex;gap:8px;flex-wrap:wrap;align-items:flex-end">' +
            '<div style="flex:1;min-width:120px">' +
              '<label style="font-size:0.72rem;color:var(--text-muted)">\u65B0\u5E8F\u5217\u865F / New Serial (1-10 digits)</label>' +
              '<input type="text" id="serial-input" class="ws-send-input" style="width:100%;margin-top:2px;font-size:0.8rem" ' +
                'maxlength="10" placeholder="\u8F38\u5165\u65B0\u5E8F\u5217\u865F\u2026" />' +
            '</div>' +
            '<div style="flex:1;min-width:120px">' +
              '<label style="font-size:0.72rem;color:var(--text-muted)">\u5BC6\u78BC / Password</label>' +
              '<input type="password" id="serial-password" class="ws-send-input" style="width:100%;margin-top:2px;font-size:0.8rem" ' +
                'maxlength="10" placeholder="\u8F38\u5165\u5BC6\u78BC\u2026" />' +
            '</div>' +
            '<div>' +
              '<button class="action-btn action-btn-red" onclick="window.__hfWriteSerial()" style="font-size:0.78rem;white-space:nowrap">' +
                '\u5BEB\u5165 / Write Serial</button>' +
            '</div>' +
          '</div>' +
          '<div id="serial-result" style="margin-top:8px;font-size:0.75rem;color:var(--text-muted);font-family:monospace">--</div>' +
        '</div>' +
      '</div>';
  }

  // --- Footer ---
  function buildFooterHTML() {
    return '' +
      '<div class="hmi-footer">' +
        'HuiFlow IoT Platform &copy; 2025-2026 &nbsp;|&nbsp; ' +
        'ESP32-S3 + W5500 + AWS IoT Core' +
      '</div>';
  }

  // ===========================================================================
  // UI Update Functions
  // ===========================================================================

  /**
   * Update the connection badge in the header.
   */
  function setConnectionStatus(connected) {
    var badge = document.getElementById('conn-badge');
    if (!badge) return;
    if (connected) {
      badge.className = 'connection-badge connected';
      badge.innerHTML = '<span class="connection-dot"></span> ONLINE';
    } else {
      badge.className = 'connection-badge disconnected';
      badge.innerHTML = '<span class="connection-dot"></span> OFFLINE';
    }
  }

  /**
   * Hide the loading overlay (with fade animation).
   */
  function hideLoading() {
    var overlay = document.getElementById('hmi-loading');
    if (overlay && !overlay.classList.contains('fade-out')) {
      overlay.classList.add('fade-out');
      setTimeout(function () {
        if (overlay.parentNode) {
          overlay.parentNode.removeChild(overlay);
        }
      }, 500);
    }
  }

  /**
   * Update a binary sensor item in the DOM.
   */
  function updateBinarySensor(def, value) {
    var container = document.getElementById('bs-' + def.key);
    if (!container) return;

    var on = isOn(value);
    var dot = container.querySelector('.binary-dot');
    var stateLabel = container.querySelector('.binary-state');

    if (on === true) {
      container.className = 'binary-item state-on';
      if (dot) dot.className = 'binary-dot on';
      if (stateLabel) { stateLabel.textContent = 'ON'; stateLabel.className = 'binary-state on'; }
    } else if (on === false) {
      container.className = 'binary-item state-off';
      if (dot) dot.className = 'binary-dot off';
      if (stateLabel) { stateLabel.textContent = 'OFF'; stateLabel.className = 'binary-state off'; }
    }
    // else leave as unknown

    flashElement(container);
  }

  /**
   * Update a numeric sensor display.
   */
  function updateSensorDisplay(def, rawValue) {
    var el = document.getElementById('sd-' + def.key);
    if (!el) return;

    var formatted = formatSensorValue(rawValue, def.format);
    el.textContent = formatted;

    var wrap = document.getElementById('sd-' + def.key + '-wrap');
    flashElement(wrap);
  }

  /**
   * Update a switch/relay display and toggle button.
   */
  function updateSwitchDisplay(index, value) {
    var container = document.getElementById('sw-' + index);
    var btn = document.getElementById('sw-btn-' + index);
    var stateLabel = document.getElementById('sw-state-' + index);
    if (!container) return;

    var on = isOn(value);

    if (on === true) {
      container.classList.add('state-on');
      container.classList.remove('state-off');
      if (btn) { btn.classList.add('on'); btn.classList.remove('off'); }
      if (stateLabel) { stateLabel.textContent = 'ON'; stateLabel.className = 'relay-state-label on'; }
    } else if (on === false) {
      container.classList.remove('state-on');
      container.classList.add('state-off');
      if (btn) { btn.classList.remove('on'); btn.classList.add('off'); }
      if (stateLabel) { stateLabel.textContent = 'OFF'; stateLabel.className = 'relay-state-label off'; }
    }
  }

  /**
   * Update a light entity display from SSE state event.
   * Called when ESPHome pushes a state update for a light entity.
   * @param {Object} def - LIGHT_DEFS entry (key, id, name, label)
   * @param {Object} data - SSE event data (state, brightness, effect, color, etc.)
   */
  function updateLightDisplay(def, data) {
    var stateEl = document.getElementById('lc-state-' + def.key);   // "ON" / "OFF" text
    var btn = document.getElementById('lc-btn-' + def.key);          // toggle button
    var briSlider = document.getElementById('lc-bri-' + def.key);   // brightness range input
    var briVal = document.getElementById('lc-bri-val-' + def.key);  // brightness value text
    var effSelect = document.getElementById('lc-eff-' + def.key);   // effect dropdown
    var on = isOn(data.state);

    // Update state indicator text
    if (stateEl) stateEl.textContent = on ? 'ON' : 'OFF';
    // Update toggle button: text + CSS class for visual feedback
    if (btn) {
      btn.textContent = on ? 'ON' : 'OFF';
      btn.classList.toggle('on', !!on);
      btn.classList.toggle('off', !on);
    }
    // Sync brightness slider and label with actual device state
    if (data.brightness !== undefined && briSlider) {
      briSlider.value = data.brightness;
      if (briVal) briVal.textContent = data.brightness;
    }
    // Sync effect dropdown with current running effect
    if (data.effect && effSelect) {
      for (var i = 0; i < effSelect.options.length; i++) {
        if (effSelect.options[i].value === data.effect) {
          effSelect.selectedIndex = i;
          break;
        }
      }
    }
  }

  /**
   * Update the fan entity display from SSE state event.
   * Called when ESPHome pushes a state update for the cooling_fan entity.
   * @param {Object} data - SSE event data (state, speed_level)
   */
  function updateFanDisplay(data) {
    var stateEl = document.getElementById('fan-state');    // "ON" / "OFF" text
    var btn = document.getElementById('fan-btn');           // toggle button
    var speedSlider = document.getElementById('fan-speed'); // speed range input (0-100)
    var speedVal = document.getElementById('fan-speed-val'); // speed value text
    var on = isOn(data.state);

    // Update state indicator
    if (stateEl) stateEl.textContent = on ? 'ON' : 'OFF';
    // Update toggle button: text + CSS class for visual feedback
    if (btn) {
      btn.textContent = on ? 'ON' : 'OFF';
      btn.classList.toggle('on', !!on);
      btn.classList.toggle('off', !on);
    }
    // Sync speed slider and label with actual device state
    if (data.speed_level !== undefined && speedSlider) {
      speedSlider.value = data.speed_level;
      if (speedVal) speedVal.textContent = data.speed_level;
    }
  }

  /**
   * Global timestamp of last received event.
   */
  function updateTimestamp() {
    lastEventTime = new Date();
    setText('device-last-update', timeString());
  }

  // ===========================================================================
  // Event Processing
  // ===========================================================================

  /**
   * Process a single state event from WebSocket or SSE.
   */
  function processStateEvent(data) {
    var id = data.id;
    var value = data.value !== undefined ? data.value : data.state;

    // Store in cache
    stateCache[id] = data;
    updateTimestamp();

    // --- Text Sensor: Serial Number ---
    if (id === TEXT_SENSOR_IDS.serial) {
      setText('device-serial', value || '--');
      return;
    }

    // --- Text Sensor: WS1 雲端 WebSocket 連線狀態 ---
    if (id === TEXT_SENSOR_IDS.wsConnection) {
      var wsEl = document.getElementById('ws1-conn');
      if (wsEl) {
        wsEl.textContent = value || '--';
        wsEl.className = 'aws-value ' + getConnStatusClass(value, true);
      }
      appendWsPanelLog(1, value || '--', 'info');
      return;
    }

    // --- Text Sensor: WS2 LAN WebSocket 連線狀態 ---
    if (id === TEXT_SENSOR_IDS.wsLanConnection) {
      var lanEl = document.getElementById('ws2-conn');
      if (lanEl) {
        lanEl.textContent = value || '--';
        lanEl.className = 'aws-value ' + getConnStatusClass(value, false);
      }
      appendWsPanelLog(2, value || '--', 'info');
      return;
    }

    // --- Text Sensor: LAN WebSocket Server Address ---
    if (id === TEXT_SENSOR_IDS.wsLanServerInfo) {
      return; // 伺服器地址已顯示在面板標題
    }

    // --- Text Sensor: WebSocket Last Job (雲端收到的任務) ---
    if (id === TEXT_SENSOR_IDS.wsLastJob) {
      appendWsPanelLog(1, value, 'recv');
      return;
    }

    // --- Text Sensor: WebSocket All Received Messages (雲端收到的訊息) ---
    if (id === TEXT_SENSOR_IDS.wsLastRecv) {
      appendWsPanelLog(1, value, 'recv');
      return;
    }

    // --- Text Sensor: WebSocket Last Sent Packet (雲端發送的封包) ---
    if (id === TEXT_SENSOR_IDS.wsLastSent) {
      appendWsPanelLog(1, value, 'sent');
      return;
    }

    // --- Text Sensor: WebSocket Sent Status (STATUS JSON 完整內容) ---
    // 獨立通道避免被 ws_last_sent 的 CMD_RESULT 更新蓋掉
    // STATUS 同時廣播到 WS1（雲端）+ WS2（LAN），兩個面板都要顯示
    if (id === TEXT_SENSOR_IDS.wsLastStatus) {
      appendWsPanelLog(1, 'TX STATUS: ' + (value || ''), 'sent');
      appendWsPanelLog(2, '[LAN] TX STATUS: ' + (value || ''), 'sent');
      return;
    }

    // --- Text Sensor: WebSocket Connection/Error Log ---
    // ws_log 包含 WS1（雲端）和 WS2（LAN）的所有日誌
    // [LAN] 前綴 → WS2 面板，其他 → WS1 面板
    // CMD: 前綴 → WS1 面板（cmd_ok/error 顏色）+ 更新 trigger-result
    if (id === TEXT_SENSOR_IDS.wsLog) {
      var isLan = value && value.indexOf('[LAN]') !== -1;
      // CMD 結果特殊處理：只寫一次，用 cmd_ok/error 顏色
      if (value && value.indexOf('CMD:') === 0) {
        var isOk = value.indexOf('OK') >= 0;
        appendWsPanelLog(1, value, isOk ? 'cmd_ok' : 'error');
        // 也更新觸發測試卡片的結果欄
        var trigResult = document.getElementById('trigger-result');
        if (trigResult) {
          trigResult.textContent = timeString() + ' ' + value;
          trigResult.style.color = isOk ? '#4ade80' : '#ef4444';
        }
      } else {
        // 一般連線日誌
        var logDir = 'info';
        if (value && (value.indexOf('ERROR') !== -1 || value.indexOf('failed') !== -1)) {
          logDir = 'error';
        } else if (isLan && value) {
          // WS2 面板：TX/RX 顏色對齊 WS1（sent=綠, recv=藍, EVENT=紅）
          if (value.indexOf('TX:') !== -1 || value.indexOf('TX ') !== -1) logDir = 'sent';
          if (value.indexOf('RX:') !== -1 || value.indexOf('RX ') !== -1) logDir = 'recv';
        }
        appendWsPanelLog(isLan ? 2 : 1, value, logDir);
      }
      // Update go_enable / emergency_enable status from ws_log
      if (value) {
        if (value.indexOf('go_enable ARMED') >= 0) {
          var goEl = document.getElementById('trigger-go-status');
          if (goEl) { goEl.textContent = 'ARMED'; goEl.style.color = '#4ade80'; }
          var goArg = document.getElementById('trigger-go-arg');
          var m = value.match(/arg=(.+)/);
          if (goArg && m) goArg.textContent = 'arg: ' + m[1];
        } else if (value.indexOf('go_enable TRIGGERED') >= 0) {
          var goEl2 = document.getElementById('trigger-go-status');
          if (goEl2) { goEl2.textContent = 'TRIGGERED'; goEl2.style.color = '#facc15'; }
          setTimeout(function () {
            var g = document.getElementById('trigger-go-status');
            if (g) { g.textContent = 'IDLE'; g.style.color = '#888'; }
            var ga = document.getElementById('trigger-go-arg');
            if (ga) ga.textContent = '--';
          }, 3000);
        } else if (value.indexOf('go_enable DISARMED') >= 0) {
          var goEl3 = document.getElementById('trigger-go-status');
          if (goEl3) { goEl3.textContent = 'IDLE'; goEl3.style.color = '#888'; }
          var goArg3 = document.getElementById('trigger-go-arg');
          if (goArg3) goArg3.textContent = '--';
        } else if (value.indexOf('emergency_enable ARMED') >= 0) {
          var emgEl = document.getElementById('trigger-emg-status');
          if (emgEl) { emgEl.textContent = 'ARMED'; emgEl.style.color = '#f87171'; }
          var emgArg = document.getElementById('trigger-emg-arg');
          var m2 = value.match(/arg=(.+)/);
          if (emgArg && m2) emgArg.textContent = 'arg: ' + m2[1];
        } else if (value.indexOf('emergency_enable TRIGGERED') >= 0) {
          var emgEl2 = document.getElementById('trigger-emg-status');
          if (emgEl2) { emgEl2.textContent = 'TRIGGERED'; emgEl2.style.color = '#facc15'; }
          setTimeout(function () {
            var e2 = document.getElementById('trigger-emg-status');
            if (e2) { e2.textContent = 'IDLE'; e2.style.color = '#888'; }
            var ea = document.getElementById('trigger-emg-arg');
            if (ea) ea.textContent = '--';
          }, 3000);
        } else if (value.indexOf('emergency_enable DISARMED') >= 0) {
          var emgEl3 = document.getElementById('trigger-emg-status');
          if (emgEl3) { emgEl3.textContent = 'IDLE'; emgEl3.style.color = '#888'; }
          var emgArg3 = document.getElementById('trigger-emg-arg');
          if (emgArg3) emgArg3.textContent = '--';
        }
      }
      return;
    }

    // --- Text Sensor: OTA Status (log-style list) ---
    if (id === TEXT_SENSOR_IDS.otaStatus) {
      var otaEl = document.getElementById('ota-status-display');
      var barWrap = document.getElementById('ota-progress-bar-wrap');
      var bar = document.getElementById('ota-progress-bar');
      if (otaEl && value) {
        // Determine color
        var color = '#fa0';
        if (value.indexOf('Blocked') >= 0 || value.indexOf('Failed') >= 0 || value.indexOf('MISMATCH') >= 0) {
          color = '#f44';
        } else if (value.indexOf('Updated') >= 0 || value.indexOf('Up to date') >= 0 || value.indexOf('MATCH') >= 0) {
          color = '#4f4';
        } else if (value.indexOf('Downloading') >= 0 || value.indexOf('[4/6]') >= 0 || value.indexOf('[5/6]') >= 0) {
          color = '#4af';
        }
        // For Checking/Idle, clear previous log to start fresh
        if (value === 'Checking...' || value === 'Idle') {
          otaEl.innerHTML = '';
        }
        // Append new line
        var line = document.createElement('div');
        line.style.color = color;
        line.style.padding = '1px 0';
        var ts = new Date();
        var tsStr = ('0'+ts.getHours()).slice(-2)+':'+('0'+ts.getMinutes()).slice(-2)+':'+('0'+ts.getSeconds()).slice(-2);
        line.textContent = tsStr + ' ' + value;
        otaEl.appendChild(line);
        otaEl.scrollTop = otaEl.scrollHeight;
        // Auto reload for reboot
        if (value.indexOf('Updated') >= 0 && value.indexOf('reboot') >= 0 && !window.__otaReloadTimer) {
          var countdown = 15;
          window.__otaReloadTimer = setInterval(function() {
            countdown--;
            var rl = document.createElement('div');
            rl.style.color = '#4f4';
            rl.textContent = 'Reloading in ' + countdown + 's...';
            otaEl.appendChild(rl);
            otaEl.scrollTop = otaEl.scrollHeight;
            if (countdown <= 0) {
              clearInterval(window.__otaReloadTimer);
              window.__otaReloadTimer = null;
              location.reload();
            }
          }, 1000);
        }
      }
      // Progress bar for download
      if (barWrap && bar) {
        var pctMatch = value ? value.match(/(\d+)%/) : null;
        if (pctMatch) {
          barWrap.style.display = 'block';
          bar.style.width = pctMatch[1] + '%';
        } else if (value && (value.indexOf('Up to date') >= 0 || value.indexOf('Idle') >= 0 || value.indexOf('Blocked') >= 0)) {
          barWrap.style.display = 'none';
          bar.style.width = '0%';
        }
      }
      return;
    }


    // --- Text Sensor: EEPROM Result (Serial Modification feedback) ---
    if (id === TEXT_SENSOR_IDS.eepromResult) {
      var serialResultEl = document.getElementById('serial-result');
      if (serialResultEl) {
        serialResultEl.textContent = value || '--';
        if (value && value.indexOf('OK') >= 0) {
          serialResultEl.style.color = '#4f4';
        } else if (value && value.indexOf('FAILED') >= 0) {
          serialResultEl.style.color = '#f44';
        } else {
          serialResultEl.style.color = 'var(--text-muted)';
        }
      }
      return;
    }

    // --- Switch: Climate Enable (同步 #climate-btn 的 ON/OFF) ---
    // YAML template switch (id: climate_enable_sw) 包裝 id(climate_enabled) 全域
    if (id === 'switch-climate_enable_sw') {
      var cbtn = document.getElementById('climate-btn');
      if (cbtn) {
        var cOn = isOn(value);
        cbtn.textContent = cOn ? 'ON' : 'OFF';
        cbtn.classList.toggle('on', !!cOn);
        cbtn.classList.toggle('off', !cOn);
      }
      return;
    }

    // --- Text Sensor: Climate Status (fan run/rest timer) ---
    // 格式：RUN|elapsed|max|speed  或  REST|elapsed|max  或  WASH  或  DISABLED  或  OFF
    if (id === TEXT_SENSOR_IDS.climateStatus) {
      var stEl = document.getElementById('climate-fan-state');
      var tmEl = document.getElementById('climate-fan-timer');
      var pidEl = document.getElementById('climate-pid-speed');
      var climBtn = document.getElementById('climate-btn');
      if (stEl && value) {
        var parts = value.split('|');
        var mode = parts[0];
        if (mode === 'RUN') {
          stEl.textContent = '\u25B6 \u904B\u8F49\u4E2D';
          stEl.style.color = '#4ade80';
          if (tmEl) tmEl.textContent = parts[1] + ' / ' + parts[2] + ' min';
          if (pidEl && parts[3]) pidEl.textContent = '(PID: ' + parts[3] + '%)';
          if (climBtn) { climBtn.textContent = 'ON'; climBtn.classList.add('on'); climBtn.classList.remove('off'); }
        } else if (mode === 'REST') {
          stEl.textContent = '\u23F8 \u4F11\u606F\u4E2D';
          stEl.style.color = '#f59e0b';
          if (tmEl) tmEl.textContent = parts[1] + ' / ' + parts[2] + ' min';
          if (pidEl) pidEl.textContent = '(PID: 0%)';
          if (climBtn) { climBtn.textContent = 'ON'; climBtn.classList.add('on'); climBtn.classList.remove('off'); }
        } else if (mode === 'WASH') {
          stEl.textContent = '\u{1F6BF} \u6D17\u8ECA\u4E2D 1000RPM';
          stEl.style.color = '#38bdf8';
          if (tmEl) tmEl.textContent = '';
          if (pidEl) pidEl.textContent = '(PID: 100%)';
          if (climBtn) { climBtn.textContent = 'ON'; climBtn.classList.add('on'); climBtn.classList.remove('off'); }
        } else if (mode === 'DISABLED') {
          stEl.textContent = '\u23F9 \u5DF2\u505C\u7528';
          stEl.style.color = 'var(--text-muted)';
          if (tmEl) tmEl.textContent = '';
          // 更新風扇卡片上的閉環按鈕為 OFF
          var btn = document.getElementById('climate-btn');
          if (btn) { btn.textContent = 'OFF'; btn.classList.remove('on'); btn.classList.add('off'); }
        } else {
          stEl.textContent = '\u2B55 \u5F85\u6A5F';
          stEl.style.color = 'var(--text-muted)';
          if (tmEl) tmEl.textContent = '';
        }
      }
      return;
    }

    // --- Text Sensor: UART2 Monitor (TX/RX hex log) ---
    if (id === TEXT_SENSOR_IDS.uart2Monitor) {
      appendUart2Log(value);
      return;
    }


    // --- Numeric Sensors ---
    var sensorDef = sensorById[id];
    if (sensorDef) {
      updateSensorDisplay(sensorDef, value);
      if (sensorDef.key === 'uptime') {
        setText('device-uptime', formatUptime(value));
      }
      // Climate card live readings
      return;
    }

    // --- Binary Sensors ---
    var binaryDef = binaryById[id];
    if (binaryDef) {
      updateBinarySensor(binaryDef, value);
      return;
    }

    // --- Switches ---
    var switchDef = switchById[id];
    if (switchDef) {
      // Find the index
      for (var i = 0; i < SWITCH_DEFS.length; i++) {
        if (SWITCH_DEFS[i].id === id) {
          updateSwitchDisplay(i, value);
          break;
        }
      }
      return;
    }

    // --- Light entities ---
    for (var li = 0; li < LIGHT_DEFS.length; li++) {
      if (id === LIGHT_DEFS[li].id) {
        updateLightDisplay(LIGHT_DEFS[li], data);
        return;
      }
    }

    // --- Fan entity ---
    if (id === FAN_DEF.id) {
      updateFanDisplay(data);
      return;
    }

    // --- LED Count number entities ---
    if (id === 'number-led_count_1') {
      var el = document.getElementById('lc-count-light1');
      if (el) el.value = Math.round(parseFloat(value));
      return;
    }
    if (id === 'number-led_count_2') {
      var el2 = document.getElementById('lc-count-light2');
      if (el2) el2.value = Math.round(parseFloat(value));
      return;
    }

  }

  /**
   * Process log events for system log viewer.
   */


  // ===========================================================================
  // EventSource (SSE) Connection — ESPHome web_server v2 built-in
  // ===========================================================================

  function getBaseUrl() {
    return ESP_BASE_URL || '';
  }

  function connectSSE() {
    if (reconnectTimer) {
      clearTimeout(reconnectTimer);
      reconnectTimer = null;
    }
    if (evtSource) {
      evtSource.close();
      evtSource = null;
    }

    var sseUrl = getBaseUrl() + '/events';
    console.log('[HF] Connecting SSE to:', sseUrl);
    var badge = document.getElementById('conn-badge');
    if (badge) badge.innerHTML = '<span class="connection-dot"></span> Connecting: ' + sseUrl;

    try {
      evtSource = new EventSource(sseUrl);
    } catch (err) {
      console.error('[HF] SSE create error:', err);
      if (badge) badge.innerHTML = '<span class="connection-dot"></span> SSE Error: ' + err.message;
      scheduleReconnect();
      return;
    }

    evtSource.addEventListener('state', function (e) {
      try {
        var data = JSON.parse(e.data);
        // ESPHome SSE format: { id: "sensor-system_uptime", value: "123", state: "123" }
        processStateEvent(data);
        if (!isConnected) {
          isConnected = true;
          wsFailCount = 0;
          reconnectDelay = RECONNECT_DELAY_MS;
          setConnectionStatus(true);
          var ov = document.getElementById('web-server-off-overlay');
          if (ov && ov.parentNode) ov.parentNode.removeChild(ov);
          setTimeout(hideLoading, LOADING_HIDE_DELAY_MS);
        }
        updateTimestamp();
      } catch (err) {
        // Silently ignore parse errors
      }
    });

    evtSource.addEventListener('ping', function () {
      updateTimestamp();
    });

    // SSE 斷線處理：最多重連 3 次後停止
    // 避免無限重連造成記憶體洩漏 → ESP32 主板重啟
    evtSource.onerror = function () {
      isConnected = false;
      setConnectionStatus(false);
      evtSource.close();
      evtSource = null;
      wsFailCount = (wsFailCount || 0) + 1;
      var badge2 = document.getElementById('conn-badge');
      if (wsFailCount >= 3) {
        // 3 次失敗後停止重連，顯示手動重新整理提示
        if (badge2) badge2.innerHTML = '<span class="connection-dot"></span> OFFLINE (\u5DF2\u505C\u6B62\u91CD\u9023\uFF0C\u8ACB\u624B\u52D5\u91CD\u65B0\u6574\u7406)';
        showWebServerOffOverlay();
        return;  // 不再 scheduleReconnect
      }
      if (badge2) badge2.innerHTML = '<span class="connection-dot"></span> OFFLINE (retry #' + wsFailCount + '/3)';
      scheduleReconnect();
    };

    evtSource.onopen = function () {
      console.log('[HF] SSE connected');
      var b = document.getElementById('conn-badge');
      if (b) b.innerHTML = '<span class="connection-dot connected"></span> CONNECTED (SSE)';
    };
  }

  function scheduleReconnect() {
    if (reconnectTimer) return;
    reconnectTimer = setTimeout(function () {
      reconnectTimer = null;
      connectSSE();
    }, reconnectDelay);
    reconnectDelay = Math.min(reconnectDelay * 1.5, RECONNECT_MAX_DELAY_MS);
  }

  // --- Send CMD via REST API (text entity + button) ---
  function wsSendCmd(cmd, on, ms, arg) {
    var json = { type: 'CMD', cmd: cmd };
    if (on !== undefined) json.on = on ? 1 : 0;
    if (ms) json.ms = ms;
    if (arg) json.arg = arg;
    var base = getBaseUrl();
    // Write CMD JSON to ws_cmd_input text entity, then press execute button
    fetch(base + '/text/ws_cmd_input/set?value=' + encodeURIComponent(JSON.stringify(json)), { method: 'POST' })
      .then(function () { return fetch(base + '/button/ws_cmd_exec/press', { method: 'POST' }); })
      .catch(function (err) { console.error('[HF] CMD error:', err); });
  }

  // --- Send control commands via ESPHome web_server v2 REST API ---
  // 重要：ESPHome REST API 使用 URL query parameters（不是 JSON body）
  // 格式範例：
  //   POST /light/status_light_1/turn_on?brightness=128&r=255&g=0&b=0&effect=Solid
  //   POST /fan/cooling_fan/turn_on?speed_level=50
  //   POST /switch/ssr_1/toggle
  //   POST /number/led_count_1/set?value=20
  // @param {string} domain  - Entity domain: switch, button, light, fan, number, text
  // @param {string} action  - Action: toggle, turn_on, turn_off, set, press
  // @param {string} id      - Entity object_id (e.g. 'status_light_1', 'ssr_1')
  // @param {Object} extra   - Optional params object → 轉為 URL query string
  function wsSendApi(domain, action, id, extra) {
    var base = getBaseUrl();
    var url;
    if (domain === 'switch') {
      url = base + '/switch/' + id + '/' + action;
    } else if (domain === 'button') {
      url = base + '/button/' + id + '/press';
    } else if (domain === 'light') {
      // ESPHome light REST: /light/{id}/{action}?brightness=N&r=N&g=N&b=N&effect=X
      if (action === 'toggle') {
        url = base + '/light/' + id + '/toggle';
      } else if (action === 'turn_on') {
        url = base + '/light/' + id + '/turn_on';
      } else if (action === 'turn_off') {
        url = base + '/light/' + id + '/turn_off';
      } else {
        url = base + '/light/' + id + '/' + action;
      }
    } else if (domain === 'fan') {
      // ESPHome fan REST: /fan/{id}/{action}?speed_level=N
      if (action === 'toggle') {
        url = base + '/fan/' + id + '/toggle';
      } else if (action === 'turn_on') {
        url = base + '/fan/' + id + '/turn_on';
      } else if (action === 'turn_off') {
        url = base + '/fan/' + id + '/turn_off';
      } else {
        url = base + '/fan/' + id + '/' + action;
      }
    } else if (domain === 'number') {
      url = base + '/number/' + id + '/set?value=' + (extra && extra.value !== undefined ? extra.value : 0);
    } else if (domain === 'text') {
      url = base + '/text/' + id + '/set?value=' + encodeURIComponent(extra && extra.value ? extra.value : '');
    } else {
      url = base + '/' + domain + '/' + id + '/' + action;
    }
    // ESPHome web_server v2: extra 參數透過 URL query string 傳遞（不支援 JSON body）
    if (extra && (domain === 'light' || domain === 'fan') && action === 'turn_on') {
      var params = [];
      for (var key in extra) {
        if (extra.hasOwnProperty(key)) {
          params.push(encodeURIComponent(key) + '=' + encodeURIComponent(extra[key]));
        }
      }
      if (params.length > 0) {
        url += (url.indexOf('?') >= 0 ? '&' : '?') + params.join('&');
      }
    }
    fetch(url, { method: 'POST' }).catch(function (err) { console.error('[HF] API error:', err); });
  }

  // ===========================================================================
  // Switch Toggle Handler (exposed globally for onclick)
  // ===========================================================================

  window.__hfToggle = function (index) {
    var sw = SWITCH_DEFS[index];
    if (!sw) return;
    var entityName = sw.id.replace('switch-', '');
    wsSendApi('switch', 'toggle', entityName);
  };

  // ===========================================================================
  // Light Control Handlers (WS2812 LED strips)
  // All use ESPHome web_server v2 REST API: POST /light/{name}/{action}
  // JSON body: { brightness: 0-255, r/g/b: 0-255, effect: "name" }
  // ===========================================================================

  // Toggle light ON/OFF — POST /light/{name}/toggle
  window.__hfLightToggle = function (name) {
    wsSendApi('light', 'toggle', name);
  };

  // Set brightness (0-255) — automatically turns on the light
  // POST /light/{name}/turn_on { brightness: N }
  window.__hfLightBri = function (name, value) {
    wsSendApi('light', 'turn_on', name, { brightness: parseInt(value) });
  };

  // Switch effect — POST /light/{name}/turn_on { effect: "Rainbow" }
  // "None" disables all effects and shows static color
  window.__hfLightEff = function (name, effect) {
    wsSendApi('light', 'turn_on', name, { effect: effect });
  };

  // Set color from preset button — also switches to "Solid" effect
  // Without effect:"Solid", animated effects (Rainbow, Fire, etc.) would
  // continuously overwrite the pixel buffer, making color change invisible
  // POST /light/{name}/turn_on { r, g, b, effect: "Solid" }
  window.__hfLightColor = function (name, r, g, b) {
    wsSendApi('light', 'turn_on', name, { r: r, g: g, b: b, effect: 'Solid' });
  };

  // Set active LED count — controls how many LEDs animate (1~30)
  // POST /number/{numberId}/set?value=N
  window.__hfLedCount = function (numberId, value) {
    wsSendApi('number', 'set', numberId, { value: parseFloat(value) });
  };

  // ===========================================================================
  // Fan Control Handlers (PWM cooling fan on GPIO37)
  // Uses ESPHome web_server v2 REST API: POST /fan/{name}/{action}
  // ===========================================================================

  // Toggle fan ON/OFF — ESPHome speed fan 的 /toggle 端點在 2026.x 行為不一致，
  // 改用讀取目前狀態 → 送 turn_on / turn_off 確保可靠切換。
  // 當前狀態由 SSE fan-cooling_fan 同步到 #fan-btn.textContent (updateFanDisplay)。
  window.__hfFanToggle = function () {
    var btn = document.getElementById('fan-btn');
    var isOn = btn && btn.textContent === 'ON';
    wsSendApi('fan', isOn ? 'turn_off' : 'turn_on', FAN_DEF.name);
  };

  // Set fan speed (0-100%) — POST /fan/cooling_fan/turn_on { speed_level: N }
  // Also updates the displayed value immediately for responsive UX
  window.__hfFanSpeed = function (value) {
    wsSendApi('fan', 'turn_on', FAN_DEF.name, { speed_level: parseInt(value) });
    var valEl = document.getElementById('fan-speed-val');
    if (valEl) valEl.textContent = value;
  };

  // ===========================================================================
  // WebSocket Control Handlers
  // ===========================================================================


  // WS1 雲端 Connect/Disconnect（按鈕 entity: ws_connect / ws_disconnect）
  window.__hfWsConnect = function () { wsSendApi('button', 'press', 'ws_connect'); };
  window.__hfWsConnect1 = window.__hfWsConnect;
  window.__hfWsDisconnect = function () { wsSendApi('button', 'press', 'ws_disconnect'); };
  window.__hfWsDisconnect1 = window.__hfWsDisconnect;


  window.__hfWebServerOn = function () {
    wsSendApi('switch', 'turn_on', 'web_server');
  };

  window.__hfWebServerOff = function () {
    if (!confirm('\u78BA\u5B9A\u8981\u95DC\u9589 Web Server\uFF1F\n\u95DC\u9589\u5F8C\u7DB2\u9801\u5C07\u7121\u6CD5\u5B58\u53D6\uFF0C\u9700\u900F\u904E BOOT \u6309\u9215\u6216 WS CMD \u91CD\u65B0\u958B\u555F\u3002')) return;
    wsSendApi('switch', 'turn_off', 'web_server');
    showWebServerOffOverlay();
  };

  function showWebServerOffOverlay() {
    if (document.getElementById('web-server-off-overlay')) return;
    var ov = document.createElement('div');
    ov.id = 'web-server-off-overlay';
    ov.style.cssText = 'position:fixed;top:0;left:0;width:100%;height:100%;background:rgba(0,0,0,0.92);z-index:99999;display:flex;flex-direction:column;align-items:center;justify-content:center;color:#fff;font-family:monospace;';
    ov.innerHTML =
      '<div style="font-size:2rem;margin-bottom:16px">\u26A0\uFE0F Web Server \u5DF2\u95DC\u9589</div>' +
      '<div style="font-size:1rem;color:#aaa;text-align:center;line-height:1.8">' +
        '\u7DB2\u9801\u5DF2\u7121\u6CD5\u6B63\u5E38\u904B\u4F5C\uFF0C\u8ACB\u900F\u904E\u4EE5\u4E0B\u65B9\u5F0F\u91CD\u65B0\u958B\u555F\uFF1A<br>' +
        '<span style="color:#4fc3f7">\u2022 \u6309\u4E00\u4E0B\u4E3B\u677F\u4E0A\u7684 BOOT \u6309\u9215</span><br>' +
        '<span style="color:#4fc3f7">\u2022 \u900F\u904E WS CMD: {"cmd":"web_server","on":1}</span><br>' +
        '<span style="color:#4fc3f7">\u2022 \u91CD\u65B0\u4E0A\u96FB\uFF08\u958B\u6A5F\u9810\u8A2D\u6703\u555F\u52D5 Web Server\uFF09</span>' +
      '</div>';
    document.body.appendChild(ov);
  }



  // CMD 執行（由觸發測試卡片使用）
  function execCmd(cmdJson) {
    try {
      var parsed = JSON.parse(cmdJson);
      if (!parsed.type) parsed.type = 'CMD';
      wsSendCmd(parsed.cmd, parsed.on, parsed.ms, parsed.arg);
    } catch (err) {
      // ignore invalid JSON
    }
  }

  // (LED Screen Control functions removed — LED 螢幕控制已移除)


  // --- Trigger & Washer Test handlers ---
  function triggerExec(jsonStr) {
    execCmd(jsonStr);
  }
  function triggerResult(msg) {
    var el = document.getElementById('trigger-result');
    if (el) el.textContent = timeString() + ' ' + msg;
  }
  window.__hfTriggerGo = function (arg) {
    var cmd = '{"cmd":"go_enable","arg":"' + arg + '"}';
    triggerExec(cmd);
    var el = document.getElementById('trigger-go-status');
    if (el) { el.textContent = 'ARMED'; el.style.color = '#4ade80'; }
    var el2 = document.getElementById('trigger-go-arg');
    if (el2) el2.textContent = 'arg: ' + arg;
    triggerResult('go_enable armed: ' + arg);
  };
  window.__hfTriggerEmg = function (arg) {
    var cmd = '{"cmd":"emergency_enable","arg":"' + arg + '"}';
    triggerExec(cmd);
    var el = document.getElementById('trigger-emg-status');
    if (el) { el.textContent = 'ARMED'; el.style.color = '#f87171'; }
    var el2 = document.getElementById('trigger-emg-arg');
    if (el2) el2.textContent = 'arg: ' + arg;
    triggerResult('emergency_enable armed: ' + arg);
  };
  window.__hfWasherCmd = function (name) {
    var cmd = '{"cmd":"' + name + '","ms":200,"arg":"1"}';
    triggerExec(cmd);
    triggerResult('washer: ' + name);
  };
  window.__hfQuickCmd = function (name) {
    var cmd = '{"cmd":"' + name + '"}';
    triggerExec(cmd);
    triggerResult('exec: ' + name);
  };

  // --- Climate Control (閉環自動控制) ---
  // 固定參數：35°C / 運行 20 分鐘 / 休息 3 分鐘（寫死在 YAML，不可遠端配置）
  // 閉環溫控開關：走 ESPHome web_server REST /switch/climate_enable_sw/toggle
  // 底層包裝 YAML template switch（見 iot-washer-base.yaml switch: climate_enable_sw）
  // 按鈕 text + class 由 SSE switch-climate_enable_sw 自動同步（在 STATE handler 處理）
  // 等同 WS CMD {"cmd":"climate_enable","on":1}（兩條路徑更新同一個全域）
  window.__hfClimateToggle = function () {
    wsSendApi('switch', 'toggle', 'climate_enable_sw');
  };

  // --- Job Script Engine ---

  window.__hfOtaCheck = function () {
    var statusEl = document.getElementById('ota-status-display');
    if (statusEl) { statusEl.innerHTML = '<div style="color:var(--text-muted)">Requesting OTA check...</div>'; }
    wsSendApi('button', 'press', 'check_ota');
  };

  window.__hfReboot = function () {
    if (!confirm('\u78BA\u5B9A\u8981\u91CD\u555F\u88DD\u7F6E\uFF1F / Reboot device?')) return;
    wsSendApi('button', 'press', 'restart_device');
    var statusEl = document.getElementById('ota-status-display');
    if (statusEl) { statusEl.textContent = 'Rebooting...'; statusEl.style.color = '#fa0'; }
    if (!window.__otaReloadTimer) {
      var countdown = 15;
      window.__otaReloadTimer = setInterval(function() {
        countdown--;
        if (statusEl) statusEl.textContent = 'Rebooting... (' + countdown + 's reload)';
        if (countdown <= 0) {
          clearInterval(window.__otaReloadTimer);
          window.__otaReloadTimer = null;
          location.reload();
        }
      }, 1000);
    }
  };

  // ===========================================================================
  // Serial Number Modification Handler
  // ===========================================================================

  window.__hfWriteSerial = function () {
    var serialEl = document.getElementById('serial-input');
    var pwdEl = document.getElementById('serial-password');
    var resultEl = document.getElementById('serial-result');
    if (!serialEl || !pwdEl) return;
    var serial = serialEl.value.trim();
    var pwd = pwdEl.value.trim();
    if (!serial) { if (resultEl) resultEl.textContent = 'ERROR: enter serial number'; return; }
    if (!pwd) { if (resultEl) resultEl.textContent = 'ERROR: enter password'; return; }
    if (!/^\d{1,10}$/.test(serial)) { if (resultEl) resultEl.textContent = 'ERROR: digits only, 1-10 chars'; return; }
    if (resultEl) { resultEl.textContent = 'Writing...'; resultEl.style.color = 'var(--text-muted)'; }
    wsSendApi('text', 'set', 'serial_input', { value: serial });
    setTimeout(function () {
      wsSendApi('text', 'set', 'serial_password', { value: pwd });
      setTimeout(function () {
        wsSendApi('button', 'press', 'write_serial');
      }, 100);
    }, 100);
  };




  // ===========================================================================
  // Initialization
  // ===========================================================================

  // --- Camera snapshot refresh (every 30s) ---
  function refreshCameras() {
    var t = Date.now();
    for (var i = 1; i <= 3; i++) {
      var img = document.getElementById('cam-img-' + i);
      if (img) {
        img.style.opacity = '1';
        img.src = 'http://192.168.1.10:8080/thumbs/latest_cam' + i + '.jpg?t=' + t;
      }
    }
    var ts = document.getElementById('cam-refresh-ts');
    if (ts) ts.textContent = timeString() + ' refreshed';
  }

  // --- ESP IP host config ---
  window.__hfSetHost = function () {
    var input = document.getElementById('sse-host-input');
    if (!input) return;
    var host = input.value.trim();
    if (host && !/^http/.test(host)) host = 'http://' + host;
    ESP_BASE_URL = host;
    // Reconnect WS with new host
    if (evtSource) { evtSource.close(); evtSource = null; }
    connectSSE();
    var badge = document.getElementById('conn-badge');
    if (badge) badge.innerHTML = '<span class="connection-dot"></span> Reconnecting to ' + (host || 'local') + '...';
  };

  function init() {
    // 單連線強制鎖 — 先檢查，被阻擋就不建立 Dashboard / SSE
    if (!setupTabGuard()) return;
    buildDashboard();
    connectSSE();
    setInterval(refreshCameras, 30000);
  }

  /**
   * 單連線強制鎖（localStorage + heartbeat）
   * ESPHome web_server 只允許一個 Dashboard 同時連線。
   * - 偵測到其他分頁 heartbeat 在 6 秒內 → 顯示全屏阻擋層，拒絕啟動 Dashboard（不建立 SSE）
   * - heartbeat 過期（>6 秒）視為上一個分頁已關閉 → 接管
   * 回傳 true 代表允許啟動；false 代表被阻擋（呼叫端應終止 init）
   */
  function setupTabGuard() {
    var TAB_KEY = 'hf_dashboard_tab';
    var TAB_TS_KEY = 'hf_dashboard_tab_ts';
    var HEARTBEAT_MS = 3000;
    var STALE_MS = 6000;
    var myTabId = Date.now().toString(36) + Math.random().toString(36).substring(2, 6);
    try {
      var existing = localStorage.getItem(TAB_KEY);
      var existingTs = parseInt(localStorage.getItem(TAB_TS_KEY) || '0');
      var isStale = (Date.now() - existingTs) > STALE_MS;
      if (existing && !isStale && existing !== myTabId) {
        // 已有其他分頁活躍 → 全屏阻擋，拒絕啟動
        var block = document.createElement('div');
        block.id = 'tab-guard-block';
        block.style.cssText = 'position:fixed;inset:0;background:#1a1a1a;color:#fff;display:flex;align-items:center;justify-content:center;z-index:999999;font-family:monospace;';
        block.innerHTML =
          '<div style="max-width:480px;text-align:center;padding:32px;background:#2a2a2a;border-radius:8px;border:2px solid #ff5252">' +
            '<div style="font-size:3rem;margin-bottom:16px">\u26D4</div>' +
            '<div style="font-size:1.2rem;font-weight:bold;margin-bottom:12px;color:#ff5252">\u5DF2\u6709\u4E00\u500B Dashboard \u9023\u7DDA</div>' +
            '<div style="font-size:0.9rem;line-height:1.6;color:#ccc;margin-bottom:20px">ESPHome web \u53EA\u5141\u8A31\u4E00\u500B\u540C\u6642\u9023\u7DDA\u3002<br>\u8ACB\u5148\u95DC\u9589\u5176\u4ED6\u5206\u9801\u6216\u8A2D\u5099\u7684 Dashboard\uFF0C\u518D\u91CD\u65B0\u6574\u7406\u6B64\u9801\u3002</div>' +
            '<button onclick="location.reload()" style="background:#ff5252;color:#fff;border:none;padding:10px 24px;border-radius:4px;cursor:pointer;font-size:0.95rem">\u91CD\u8A66</button>' +
          '</div>';
        // 清空 body 其它元素（CSS 會隱藏非 #hmi-dashboard，但這裡還沒建立 dashboard）
        document.body.appendChild(block);
        return false;
      }
      // 註冊此分頁 + heartbeat
      localStorage.setItem(TAB_KEY, myTabId);
      localStorage.setItem(TAB_TS_KEY, Date.now().toString());
      var hbTimer = setInterval(function () {
        if (localStorage.getItem(TAB_KEY) === myTabId) {
          localStorage.setItem(TAB_TS_KEY, Date.now().toString());
        } else {
          clearInterval(hbTimer); // 其他分頁覆蓋了我 → 停止心跳
        }
      }, HEARTBEAT_MS);
      // 頁面關閉時清除（只清自己的）
      window.addEventListener('beforeunload', function () {
        clearInterval(hbTimer);
        if (localStorage.getItem(TAB_KEY) === myTabId) {
          localStorage.removeItem(TAB_KEY);
          localStorage.removeItem(TAB_TS_KEY);
        }
      });
      return true;
    } catch (e) {
      // localStorage 不可用（隱私模式等）→ 無法強制，允許啟動
      return true;
    }
  }

  // Start when DOM is ready
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }

})();
