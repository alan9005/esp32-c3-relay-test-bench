// =============================================================================
// 继电器控制板 — 简洁 Web 控制页
// ESPHome web_server v2 自定义仪表盘
// =============================================================================
// 状态更新: EventSource (SSE) /events
// 控制指令: REST API
//   POST /switch/{id}/toggle         开关继电器
//   POST /switch/all_relays/turn_on  全部打开
//   POST /switch/all_relays/turn_off 全部关闭
//   POST /select/{id}/set?option=..  选择
//   POST /number/{id}/set?value=..   数值
//   POST /button/{id}/press          按钮
// =============================================================================

(function () {
  'use strict';

  var BASE = '';

  // ---- 语言 ----
  var STORAGE_KEY = 'relay_lang';
  function detectLang() {
    var n = (navigator.language || navigator.userLanguage || 'zh').toLowerCase();
    return n.indexOf('zh') === 0 ? 'zh' : 'en';
  }
  var lang;
  try {
    lang = localStorage.getItem(STORAGE_KEY) || detectLang();
  } catch (err) {
    lang = detectLang();
  }
  var T = {
    zh: {
      relay: '继电器',
      relayTitle: '继电器控制板',
      relaySub: 'ESP32-C3 · Modbus RTU · 16 路继电器',
      connecting: '连接中...',
      network: '网络信息',
      relayCtrl: '继电器控制',
      allOn: '全部打开',
      allOff: '全部关闭',
      delayRelay: '延时动作 · 继电器（1~16）',
      delayDuration: '延时动作 · 时长（0.1s，1~60000）',
      delayRun: '执行延时动作',
      perfTest: '性能测试',
      testStart: '开始测试',
      testStop: '停止测试',
      testCount: '测试次数（0=不限）',
      testDuration: '持续时间（秒，0=不限）',
      testInterval: '开关间隔（ms）',
      selectRelay: '选择继电器',
      selectAll: '全选',
      clearAll: '清空',
      status: '状态：',
      cycles: '已完成：',
      elapsed: '已运行：',
      times: ' 次',
      seconds: ' 秒',
      settings: '参数设置',
      moduleBaud: '模块波特率（掉电保存）',
      espBaud: 'ESP 485 波特率（运行时）',
      powerMode: '上电状态模式',
      pickupDelay: '吸合延时（μs，≤21666）',
      apply: '应用',
      statusRead: '状态读取',
      readStatus: '手动读取状态',
      selftestTitle: '继电器延时自检',
      selftestBtn: '继电器延时自检',
      selftestHint: '请将任意输出的 NO 接到 CHK 连接器上',
      otaTitle: '固件升级 (OTA)',
      upload: '上传并升级',
      otaHint: '选择 .bin 固件，上传完成后设备会自动重启。',
      otaUploading: '正在上传固件，请勿断电...',
      footer: '数据来自 ESPHome 设备，实时更新',
      on: '开',
      off: '关',
      connFail: '连接失败',
      connOk: '已连接',
      connLost: '连接断开，请刷新页面',
      connRetry: '连接断开，重连中',
      alertSelectRelay: '请选择至少一个继电器',
      alertTestParam: '测试次数和持续时间至少设置一个',
      day: ' 天',
      hour: ' 时',
      minute: ' 分',
      sec: ' 秒',
      powerAllOff: '全关',
      powerAllOn: '全开',
      powerRestore: '恢复断电前状态',
      sensorFirmware: '固件版本',
      sensorAcPeriod: '交流电源周期',
      sensorPickup: '吸合延时',
      sensorDcCurrent: '直流电源电流',
      sensorPowerMode: '上电状态模式',
      sensorConfigIndex: '配置存储区索引',
      infoIp: 'IP 地址',
      infoMac: 'MAC 地址',
      infoUptime: '运行时间'
    },
    en: {
      relay: 'Relay',
      relayTitle: 'Relay Control Board',
      relaySub: 'ESP32-C3 · Modbus RTU · 16 Relays',
      connecting: 'Connecting...',
      network: 'Network Info',
      relayCtrl: 'Relay Control',
      allOn: 'All On',
      allOff: 'All Off',
      delayRelay: 'Delay action · Relay (1~16)',
      delayDuration: 'Delay action · Duration (0.1s, 1~60000)',
      delayRun: 'Run Delay',
      perfTest: 'Performance Test',
      testStart: 'Start Test',
      testStop: 'Stop Test',
      testCount: 'Cycles (0=unlimited)',
      testDuration: 'Duration (s, 0=unlimited)',
      testInterval: 'Interval (ms)',
      selectRelay: 'Select relays',
      selectAll: 'All',
      clearAll: 'Clear',
      status: 'Status: ',
      cycles: 'Cycles: ',
      elapsed: 'Elapsed: ',
      times: '',
      seconds: ' s',
      settings: 'Settings',
      moduleBaud: 'Module baud (saved)',
      espBaud: 'ESP 485 baud (runtime)',
      powerMode: 'Power-on mode',
      pickupDelay: 'Pickup delay (μs, ≤21666)',
      apply: 'Apply',
      statusRead: 'Status Read',
      readStatus: 'Read Status',
      selftestTitle: 'Relay Delay Self-test',
      selftestBtn: 'Relay Delay Self-test',
      selftestHint: 'Connect NO of any output to the CHK connector',
      otaTitle: 'Firmware Update (OTA)',
      upload: 'Upload & Update',
      otaHint: 'Select a .bin firmware; the device reboots after upload.',
      otaUploading: 'Uploading firmware, do not power off...',
      footer: 'Live data from ESPHome device',
      on: 'ON',
      off: 'OFF',
      connFail: 'Connection failed',
      connOk: 'Connected',
      connLost: 'Disconnected, please refresh',
      connRetry: 'Disconnected, reconnecting',
      alertSelectRelay: 'Please select at least one relay',
      alertTestParam: 'Set cycles or duration at least one',
      day: ' d',
      hour: ' h',
      minute: ' m',
      sec: ' s',
      powerAllOff: 'All off',
      powerAllOn: 'All on',
      powerRestore: 'Restore last state',
      sensorFirmware: 'Firmware version',
      sensorAcPeriod: 'AC period',
      sensorPickup: 'Pickup delay',
      sensorDcCurrent: 'DC current',
      sensorPowerMode: 'Power-on mode',
      sensorConfigIndex: 'Config index',
      infoIp: 'IP address',
      infoMac: 'MAC address',
      infoUptime: 'Uptime'
    }
  };
  function t(key) {
    return (T[lang] && T[lang][key] !== undefined) ? T[lang][key] : key;
  }

  // ---- 16 路继电器 ----
  var RELAYS = [];
  (function () {
    for (var n = 1; n <= 16; n++) {
      var s = n < 10 ? '0' + n : '' + n;
      RELAYS.push({
        n: n,
        id: 'switch-relay_' + s,
        name: 'relay_' + s
      });
    }
  })();

  // ---- 只读状态 ----
  var SENSORS = [
    { key: 'firmware',     id: 'sensor-firmware_version',     labelKey: 'sensorFirmware',     unit: '' },
    { key: 'ac_period',    id: 'sensor-ac_period',            labelKey: 'sensorAcPeriod',     unit: 'μs' },
    { key: 'pickup_delay', id: 'sensor-pickup_delay_sensor',  labelKey: 'sensorPickup',       unit: 'μs' },
    { key: 'dc_current',   id: 'sensor-dc_current',           labelKey: 'sensorDcCurrent',    unit: 'mA' },
    { key: 'power_mode',   id: 'sensor-power_on_mode_sensor', labelKey: 'sensorPowerMode',    unit: '' },
    { key: 'config_index', id: 'sensor-config_index',         labelKey: 'sensorConfigIndex',  unit: '' }
  ];

  // ---- 网络信息 ----
  var INFO = [
    { key: 'ip',     id: 'text_sensor-wifi_ip',  labelKey: 'infoIp' },
    { key: 'mac',    id: 'text_sensor-wifi_mac', labelKey: 'infoMac' },
    { key: 'uptime', id: 'sensor-system_uptime', labelKey: 'infoUptime' }
  ];

  var BAUD_OPTIONS = ['9600', '19200', '38400', '57600', '115200'];
  // value 必须与 YAML 中 power_on_mode_select 的 options 一致（中文），显示文本按语言翻译
  var POWER_OPTIONS = [
    { value: '全关', key: 'powerAllOff' },
    { value: '全开', key: 'powerAllOn' },
    { value: '恢复断电前状态', key: 'powerRestore' }
  ];

  var evtSource = null;
  var retry = 0;
  var reconnectTimer = null;
  var MAX_RETRY = 5;

  // ===========================================================================
  // 工具函数
  // ===========================================================================
  function post(path) {
    return fetch(BASE + path, { method: 'POST' }).catch(function (err) {
      console.error('[relay] API error:', err);
    });
  }

  function isOn(v) {
    if (v === undefined || v === null) return false;
    var s = String(v).toUpperCase();
    return s === 'ON' || s === 'TRUE' || s === '1';
  }

  function setText(id, text) {
    var el = document.getElementById(id);
    if (el) el.textContent = (text === undefined || text === null || text === '') ? '--' : text;
  }

  function formatUptime(sec) {
    var total = Math.floor(parseFloat(sec) || 0);
    var d = Math.floor(total / 86400);
    var h = Math.floor((total % 86400) / 3600);
    var m = Math.floor((total % 3600) / 60);
    var s = total % 60;
    var p = [];
    if (d) p.push(d + t('day'));
    if (h || d) p.push(h + t('hour'));
    if (m || h || d) p.push(m + t('minute'));
    p.push(s + t('sec'));
    return p.join(' ');
  }

  function powerModeText(v) {
    var n = parseInt(v, 10);
    if (n === 0) return t('powerAllOff');
    if (n === 1) return t('powerAllOn');
    if (n === 2) return t('powerRestore');
    return v;
  }

  // ===========================================================================
  // DOM 构建
  // ===========================================================================
  function build() {
    var body = document.body;
    while (body.firstChild) body.removeChild(body.firstChild);

    var relayItems = '';
    for (var i = 0; i < RELAYS.length; i++) {
      var r = RELAYS[i];
      var rl = t('relay') + ' ' + r.n;
      relayItems +=
        '<div class="relay" id="relay-' + r.n + '">' +
          '<button class="relay-btn off" id="relay-btn-' + r.n + '" ' +
            'onclick="window.__relayToggle(' + r.n + ')" aria-label="' + rl + '">' +
            '<span class="dot"></span>' +
          '</button>' +
          '<div class="relay-label">' + rl + '</div>' +
          '<div class="relay-state" id="relay-state-' + r.n + '">--</div>' +
        '</div>';
    }

    var sensorItems = '';
    for (var j = 0; j < SENSORS.length; j++) {
      var sn = SENSORS[j];
      sensorItems +=
        '<div class="kv">' +
          '<span class="kv-label">' + t(sn.labelKey) + '</span>' +
          '<span class="kv-value" id="sensor-' + sn.key + '">--</span>' +
          '<span class="kv-unit">' + sn.unit + '</span>' +
        '</div>';
    }

    var infoItems = '';
    for (var k = 0; k < INFO.length; k++) {
      var inf = INFO[k];
      infoItems +=
        '<div class="kv">' +
          '<span class="kv-label">' + t(inf.labelKey) + '</span>' +
          '<span class="kv-value mono" id="info-' + inf.key + '">--</span>' +
        '</div>';
    }

    var baudOptions = '';
    for (var b = 0; b < BAUD_OPTIONS.length; b++) {
      baudOptions += '<option value="' + BAUD_OPTIONS[b] + '">' + BAUD_OPTIONS[b] + '</option>';
    }
    var powerOptions = '';
    for (var p = 0; p < POWER_OPTIONS.length; p++) {
      powerOptions += '<option value="' + POWER_OPTIONS[p].value + '">' + t(POWER_OPTIONS[p].key) + '</option>';
    }
    var relayOptions = '';
    for (var ro = 1; ro <= 16; ro++) {
      relayOptions += '<option value="' + ro + '">' + t('relay') + ' ' + ro + '</option>';
    }

    var testCheckboxes = '';
    for (var tc = 0; tc < RELAYS.length; tc++) {
      var tn = RELAYS[tc].n;
      var checked = tn === 1 ? ' checked' : '';
      testCheckboxes +=
        '<label class="chk"><input type="checkbox" class="test-relay-chk" value="' + tn + '"' + checked + '>' +
        '<span>' + t('relay') + ' ' + tn + '</span></label>';
    }

    var root = document.createElement('main');
    root.id = 'dashboard';
    root.innerHTML =
      '<header class="topbar">' +
        '<div class="brand">' +
          '<span class="brand-icon">R</span>' +
          '<div>' +
            '<h1>' + t('relayTitle') + '</h1>' +
            '<p>' + t('relaySub') + '</p>' +
          '</div>' +
        '</div>' +
        '<div class="topbar-right">' +
          '<button class="lang-btn" onclick="window.__toggleLang()">' + (lang === 'zh' ? 'EN' : '中文') + '</button>' +
          '<div class="conn" id="conn-badge">' +
            '<span class="conn-dot"></span><span id="conn-text">' + t('connecting') + '</span>' +
          '</div>' +
        '</div>' +
      '</header>' +

      '<section class="card">' +
        '<h2>' + t('network') + '</h2>' +
        '<div class="kv-grid">' + infoItems + '</div>' +
      '</section>' +

      '<section class="card">' +
        '<div class="card-head">' +
          '<h2>' + t('relayCtrl') + '</h2>' +
          '<div class="card-actions">' +
            '<button class="btn btn-green" onclick="window.__relayAll(true)">' + t('allOn') + '</button>' +
            '<button class="btn btn-red" onclick="window.__relayAll(false)">' + t('allOff') + '</button>' +
          '</div>' +
        '</div>' +
        '<div class="relay-grid">' + relayItems + '</div>' +
        '<div class="settings-grid">' +
          '<label class="field"><span>' + t('delayRelay') + '</span>' +
            '<select id="delay-relay-input">' + relayOptions + '</select></label>' +
          '<label class="field"><span>' + t('delayDuration') + '</span>' +
            '<input type="number" id="delay-duration-input" min="1" max="60000" step="1" value="10"></label>' +
          '<div class="field field-btn">' +
            '<button class="btn btn-blue" onclick="window.__setDelay()">' + t('delayRun') + '</button></div>' +
        '</div>' +
      '</section>' +

      '<section class="card">' +
        '<div class="card-head">' +
          '<h2>' + t('perfTest') + '</h2>' +
          '<div class="card-actions">' +
            '<button class="btn btn-blue" onclick="window.__testStart()">' + t('testStart') + '</button>' +
            '<button class="btn btn-red" onclick="window.__testStop()">' + t('testStop') + '</button>' +
          '</div>' +
        '</div>' +
        '<div class="settings-grid">' +
          '<label class="field"><span>' + t('testCount') + '</span>' +
            '<input type="number" id="test-count-input" min="0" max="100000" step="1" value="100"></label>' +
          '<label class="field"><span>' + t('testDuration') + '</span>' +
            '<input type="number" id="test-duration-input" min="0" max="86400" step="1" value="120"></label>' +
          '<label class="field"><span>' + t('testInterval') + '</span>' +
            '<input type="number" id="test-interval-input" min="100" max="60000" step="100" value="1500"></label>' +
        '</div>' +
        '<div class="test-relays">' +
          '<div class="test-relays-head">' +
            '<span>' + t('selectRelay') + '</span>' +
            '<button class="btn btn-small" onclick="window.__testSelectAll()">' + t('selectAll') + '</button>' +
            '<button class="btn btn-small" onclick="window.__testClear()">' + t('clearAll') + '</button>' +
          '</div>' +
          '<div class="chk-grid">' + testCheckboxes + '</div>' +
        '</div>' +
        '<div class="test-status">' +
          '<span>' + t('status') + '<b id="test-status">--</b></span>' +
          '<span>' + t('cycles') + '<b id="test-cycles">--</b>' + t('times') + '</span>' +
          '<span>' + t('elapsed') + '<b id="test-elapsed">--</b>' + t('seconds') + '</span>' +
        '</div>' +
      '</section>' +

      '<section class="card">' +
        '<h2>' + t('settings') + '</h2>' +
        '<div class="settings-grid">' +
          '<div class="field">' +
            '<span>' + t('moduleBaud') + '</span>' +
            '<select id="baud-select" onchange="window.__setBaud(this.value)">' + baudOptions + '</select>' +
            '<button class="btn btn-blue" onclick="window.__applyBaud()">' + t('apply') + '</button>' +
          '</div>' +
          '<div class="field">' +
            '<span>' + t('espBaud') + '</span>' +
            '<select id="esp-baud-select" onchange="window.__setEspBaud(this.value)">' + baudOptions + '</select>' +
            '<button class="btn btn-blue" onclick="window.__applyEspBaud()">' + t('apply') + '</button>' +
          '</div>' +
          '<div class="field">' +
            '<span>' + t('powerMode') + '</span>' +
            '<select id="power-select" onchange="window.__setPower(this.value)">' + powerOptions + '</select>' +
            '<button class="btn btn-blue" onclick="window.__applyPower()">' + t('apply') + '</button>' +
          '</div>' +
          '<div class="field">' +
            '<span>' + t('pickupDelay') + '</span>' +
            '<input type="number" id="pickup-input" min="0" max="21666" step="1" value="0">' +
            '<button class="btn btn-blue" onclick="window.__setPickup()">' + t('apply') + '</button>' +
          '</div>' +
        '</div>' +
      '</section>' +

      '<section class="card">' +
        '<div class="card-head">' +
          '<h2>' + t('statusRead') + '</h2>' +
          '<div class="card-actions">' +
            '<button class="btn btn-blue" onclick="window.__readStatus()">' + t('readStatus') + '</button>' +
          '</div>' +
        '</div>' +
        '<div class="kv-grid">' + sensorItems + '</div>' +
      '</section>' +

      '<section class="card">' +
        '<div class="card-head">' +
          '<h2>' + t('selftestTitle') + '</h2>' +
          '<div class="card-actions">' +
            '<button class="btn btn-amber" onclick="window.__selfTest()">' + t('selftestBtn') + '</button>' +
          '</div>' +
        '</div>' +
        '<p class="hint">' + t('selftestHint') + '</p>' +
      '</section>' +

      '<section class="card">' +
        '<h2>' + t('otaTitle') + '</h2>' +
        '<div class="ota-row">' +
          '<form id="ota-form" method="POST" action="/update" enctype="multipart/form-data" target="ota-iframe">' +
            '<input type="file" id="ota-file" name="firmware" accept=".bin" required>' +
            '<button class="btn btn-blue" type="submit">' + t('upload') + '</button>' +
          '</form>' +
          '<iframe name="ota-iframe" style="display:none"></iframe>' +
          '<p class="ota-hint" id="ota-hint">' + t('otaHint') + '</p>' +
        '</div>' +
      '</section>' +

      '<footer>' + t('footer') + '</footer>';

    body.appendChild(root);
  }

  // ===========================================================================
  // 状态更新
  // ===========================================================================
  function setRelay(n, on) {
    var btn = document.getElementById('relay-btn-' + n);
    var state = document.getElementById('relay-state-' + n);
    var box = document.getElementById('relay-' + n);
    if (btn) {
      btn.classList.toggle('on', on);
      btn.classList.toggle('off', !on);
    }
    if (state) state.textContent = on ? t('on') : t('off');
    if (box) box.classList.toggle('active', on);
  }

  function applyState(d) {
    var id = d.id;
    var v = d.value;
    var i;

    for (i = 0; i < RELAYS.length; i++) {
      if (id === RELAYS[i].id) {
        setRelay(RELAYS[i].n, isOn(v));
        return;
      }
    }
    for (i = 0; i < SENSORS.length; i++) {
      if (id === SENSORS[i].id) {
        setText('sensor-' + SENSORS[i].key,
          SENSORS[i].key === 'power_mode' ? powerModeText(v) : v);
        return;
      }
    }
    for (i = 0; i < INFO.length; i++) {
      if (id === INFO[i].id) {
        setText('info-' + INFO[i].key,
          INFO[i].key === 'uptime' ? formatUptime(v) : v);
        return;
      }
    }

    // 同步设置控件（用户正在编辑时不覆盖）
    if (id === 'select-baud_rate_select' &&
        document.activeElement !== document.getElementById('baud-select')) {
      document.getElementById('baud-select').value = v;
    }
    if (id === 'select-esp_baud_select' &&
        document.activeElement !== document.getElementById('esp-baud-select')) {
      document.getElementById('esp-baud-select').value = v;
    }
    if (id === 'select-power_on_mode_select' &&
        document.activeElement !== document.getElementById('power-select')) {
      document.getElementById('power-select').value = v;
    }
    if (id === 'number-pickup_delay_number' &&
        document.activeElement !== document.getElementById('pickup-input')) {
      document.getElementById('pickup-input').value = v;
    }
    if (id === 'number-delay_relay_number' &&
        document.activeElement !== document.getElementById('delay-relay-input')) {
      document.getElementById('delay-relay-input').value = v;
    }
    if (id === 'number-delay_duration_number' &&
        document.activeElement !== document.getElementById('delay-duration-input')) {
      document.getElementById('delay-duration-input').value = v;
    }

    // 性能测试状态
    if (id === 'text_sensor-test_status_text') {
      document.getElementById('test-status').textContent = v || '--';
      return;
    }
    if (id === 'sensor-test_cycle_sensor') {
      document.getElementById('test-cycles').textContent = v;
      return;
    }
    if (id === 'sensor-test_elapsed_sensor') {
      document.getElementById('test-elapsed').textContent = v;
      return;
    }
    if (id === 'number-test_count_target' &&
        document.activeElement !== document.getElementById('test-count-input')) {
      document.getElementById('test-count-input').value = v;
    }
    if (id === 'number-test_duration_s' &&
        document.activeElement !== document.getElementById('test-duration-input')) {
      document.getElementById('test-duration-input').value = v;
    }
    if (id === 'number-test_interval_ms' &&
        document.activeElement !== document.getElementById('test-interval-input')) {
      document.getElementById('test-interval-input').value = v;
    }
  }

  function setConn(ok, text) {
    var badge = document.getElementById('conn-badge');
    var el = document.getElementById('conn-text');
    if (badge) {
      badge.classList.toggle('online', ok);
      badge.classList.toggle('offline', !ok);
    }
    if (el) el.textContent = text;
  }

  // ===========================================================================
  // SSE 连接
  // ===========================================================================
  function connect() {
    if (reconnectTimer) {
      clearTimeout(reconnectTimer);
      reconnectTimer = null;
    }
    if (evtSource) {
      evtSource.close();
      evtSource = null;
    }

    try {
      evtSource = new EventSource(BASE + '/events');
    } catch (err) {
      setConn(false, t('connFail'));
      scheduleReconnect();
      return;
    }

    evtSource.addEventListener('state', function (e) {
      try {
        applyState(JSON.parse(e.data));
        retry = 0;
        setConn(true, t('connOk'));
      } catch (err) {
        /* ignore parse errors */
      }
    });

    evtSource.onopen = function () {
      setConn(true, t('connOk'));
    };

    evtSource.onerror = function () {
      evtSource.close();
      evtSource = null;
      retry++;
      if (retry >= MAX_RETRY) {
        setConn(false, t('connLost'));
        return;
      }
      setConn(false, t('connRetry') + ' (' + retry + '/' + MAX_RETRY + ')');
      scheduleReconnect();
    };
  }

  function scheduleReconnect() {
    if (reconnectTimer) return;
    reconnectTimer = setTimeout(function () {
      reconnectTimer = null;
      connect();
    }, 3000);
  }

  // ===========================================================================
  // 控制动作（暴露到全局，供 onclick 调用）
  // ===========================================================================
  window.__relayToggle = function (n) {
    var r = RELAYS[n - 1];
    if (r) post('/switch/' + r.name + '/toggle');
  };

  window.__relayAll = function (on) {
    post('/button/' + (on ? 'all_relays_on' : 'all_relays_off') + '/press');
  };

  window.__setBaud = function (v) {
    post('/select/baud_rate_select/set?option=' + encodeURIComponent(v));
  };

  window.__setEspBaud = function (v) {
    post('/select/esp_baud_select/set?option=' + encodeURIComponent(v));
  };

  window.__setPower = function (v) {
    post('/select/power_on_mode_select/set?option=' + encodeURIComponent(v));
  };

  window.__applyBaud = function () {
    post('/button/baud_apply/press');
  };

  window.__applyEspBaud = function () {
    post('/button/esp_baud_apply/press');
  };

  window.__applyPower = function () {
    post('/button/power_apply/press');
  };

  window.__setPickup = function () {
    var v = document.getElementById('pickup-input').value;
    post('/number/pickup_delay_number/set?value=' + encodeURIComponent(v))
      .then(function () { return post('/button/pickup_delay_apply/press'); });
  };

  window.__setDelay = function () {
    var r = document.getElementById('delay-relay-input').value;
    var dur = document.getElementById('delay-duration-input').value;
    post('/number/delay_relay_number/set?value=' + encodeURIComponent(r))
      .then(function () { return post('/number/delay_duration_number/set?value=' + encodeURIComponent(dur)); })
      .then(function () { return post('/button/delay_execute/press'); });
  };

  window.__selfTest = function () {
    post('/button/self_test/press');
  };

  window.__readStatus = function () {
    post('/button/read_status/press');
  };

  window.__testStart = function () {
    var nums = [];
    var boxes = document.querySelectorAll('.test-relay-chk:checked');
    for (var i = 0; i < boxes.length; i++) nums.push(boxes[i].value);
    if (!nums.length) {
      alert(t('alertSelectRelay'));
      return;
    }
    var count = document.getElementById('test-count-input').value;
    var duration = document.getElementById('test-duration-input').value;
    var interval = document.getElementById('test-interval-input').value;
    if ((parseInt(count, 10) || 0) <= 0 && (parseInt(duration, 10) || 0) <= 0) {
      alert(t('alertTestParam'));
      return;
    }
    post('/text/test_relay_selection/set?value=' + encodeURIComponent(nums.join(',')))
      .then(function () { return post('/number/test_count_target/set?value=' + encodeURIComponent(count)); })
      .then(function () { return post('/number/test_duration_s/set?value=' + encodeURIComponent(duration)); })
      .then(function () { return post('/number/test_interval_ms/set?value=' + encodeURIComponent(interval)); })
      .then(function () { return post('/button/test_start/press'); });
  };

  window.__testStop = function () {
    post('/button/test_stop_btn/press');
  };

  window.__testSelectAll = function () {
    var boxes = document.querySelectorAll('.test-relay-chk');
    for (var i = 0; i < boxes.length; i++) boxes[i].checked = true;
  };

  window.__testClear = function () {
    var boxes = document.querySelectorAll('.test-relay-chk');
    for (var i = 0; i < boxes.length; i++) boxes[i].checked = false;
  };

  window.__toggleLang = function () {
    lang = (lang === 'zh') ? 'en' : 'zh';
    try { localStorage.setItem(STORAGE_KEY, lang); } catch (err) { /* ignore */ }
    build();
    connect();
  };

  // ===========================================================================
  // 启动
  // ===========================================================================
  function init() {
    build();
    connect();
    var form = document.getElementById('ota-form');
    if (form) {
      form.addEventListener('submit', function () {
        var hint = document.getElementById('ota-hint');
        if (hint) hint.textContent = t('otaUploading');
      });
    }
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }
})();
