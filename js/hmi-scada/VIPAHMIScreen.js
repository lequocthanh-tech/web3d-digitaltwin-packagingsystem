/**
 * VIPAHMIScreen.js
 * 
 * Engineering Workstation 4: VIPA Touch Panel HMI & Industrial SCADA Screen
 * Simulates a high-end industrial operator touch panel (VIPA / Yaskawa SmartPanel)
 * Features:
 * - Brushed aluminum bezel with industrial status header
 * - Tab 1: [MAIN OVERVIEW] Synoptic interactive line layout, J1000 inverter Hz, production counters
 * - Tab 2: [RECIPE & SETTINGS] Batch recipe configuration, timing delays, servo speed limits
 * - Tab 3: [ALARM & DIAGNOSTICS] Event log, active alarm banner, and DO_19 alarm beacon flasher
 */

export class VIPAHMIScreen {
  constructor(app, options = {}) {
    this.app = app;
    this.hardwareBus = app.hardwareBus;
    this.factory = app.factory;
    this.physicsEngine = app.physicsEngine;
    this.robotArm = app.arm6DOF;
    this.deltaRobot = app.deltaRobot;

    this.activeHMITab = 'overview'; // 'overview', 'recipe', 'alarm'

    // Production OEE Counters
    this.totalBoxesShipped = 0;
    this.totalBarsPacked = 0;
    this.inverterFrequencyHz = 50.0;
    this.barsPerGroupRecipe = 4;
    this.groupsPerBoxRecipe = 2;
    this.pickDwellSec = 0.20;

    // Alarms System
    this.activeAlarms = [];
    this.alarmHistory = [
      { id: 'INF-001', time: '08:00:00', type: 'INFO', msg: 'Hệ thống MP3300 CPU-301 khởi động thành công.' },
      { id: 'INF-002', time: '08:00:02', type: 'INFO', msg: 'Kết nối mạng UDP 10040 máy chủ YRC1000micro sẵn sàng.' }
    ];

    // DO_19 Alarm Flasher State
    this.alarmBlinkTimer = null;
    this.alarmBlinkState = false;
  }

  initDOM(containerEl) {
    this.container = containerEl;
    this.renderHMIUI();
    this.bindEvents();
    this.updateScreenData();
  }

  renderHMIUI() {
    this.container.innerHTML = `
      <div class="vipa-hmi-wrapper">
        
        <!-- Aluminum Industrial Touch Panel Bezel -->
        <div class="vipa-bezel">
          
          <!-- HMI Top Title & Status Bar -->
          <div class="vipa-top-header">
            <div class="vipa-brand">
              <span class="vipa-logo">VIPA</span>
              <span class="vipa-sub">SMART PANEL 2100 // INDUSTRIAL SCADA</span>
            </div>
            
            <div class="vipa-clock-bar">
              <span class="vipa-status-tag" id="hmi-comm-tag">MP3300: ONLINE</span>
              <span class="vipa-clock" id="hmi-clock">12:00:00</span>
            </div>
          </div>

          <!-- Main Touch Screen Area -->
          <div class="vipa-touch-screen">
            
            <!-- Navigation Tabs Strip -->
            <div class="vipa-tab-nav">
              <button class="vipa-tab-btn active" id="vipa-tab-btn-overview">
                📊 [1. MAIN OVERVIEW]
              </button>
              <button class="vipa-tab-btn" id="vipa-tab-btn-recipe">
                ⚙ [2. RECIPE & SETTINGS]
              </button>
              <button class="vipa-tab-btn" id="vipa-tab-btn-alarm">
                🚨 [3. ALARM & DIAGNOSTICS] <span class="badge" id="vipa-alarm-counter-badge" style="background:#ef4444; color:#fff; display:none;">0</span>
              </button>
            </div>

            <!-- Active Alarm Ticker Banner (Flashing when alarm active) -->
            <div class="vipa-alarm-ticker" id="vipa-alarm-ticker" style="display: none;">
              <span class="vipa-alarm-icon">⚠</span>
              <span id="vipa-alarm-ticker-text">CẢNH BÁO: HỆ THỐNG ĐANG CÓ SỰ CỐ!</span>
            </div>

            <!-- ===============================================================
                 TAB 1: MAIN OVERVIEW (SYNOPTIC LINE DIAGRAM & OEE)
                 =============================================================== -->
            <div class="vipa-page active" id="vipa-page-overview">
              
              <!-- Synoptic Line View (3 Belts & 2 Robots Animation Diagram) -->
              <div class="vipa-synoptic-card">
                <div class="vipa-card-header">SƠ ĐỒ TRỰC QUAN DÂY CHUYỀN (SYNOPTIC MONITOR)</div>
                <div class="synoptic-diagram">
                  
                  <!-- Belt 1 (Infeed) -->
                  <div class="synoptic-belt-node">
                    <div class="belt-label-box">
                      <span>BĂNG TẢI 1 (INFEED)</span>
                      <span class="scada-led led-green" id="hmi-led-belt1"></span>
                    </div>
                    <div class="belt-anim-track" id="hmi-track-belt1">
                      <span class="belt-dot"></span>
                      <span class="belt-dot"></span>
                    </div>
                    <div class="belt-info-line">Biến tần J1000: <b id="hmi-val-hz1">50.0 Hz</b></div>
                  </div>

                  <!-- Delta Robot Station -->
                  <div class="synoptic-robot-node">
                    <span class="robot-icon-tag">🤖 DELTA 4-DOF</span>
                    <span class="robot-state-tag" id="hmi-state-delta">STANDBY</span>
                    <span class="robot-sub-info" id="hmi-delta-vacuum">Vac: 0.0 kPa</span>
                  </div>

                  <!-- Belt 2 (Collation) -->
                  <div class="synoptic-belt-node">
                    <div class="belt-label-box">
                      <span>BĂNG TẢI 2 (GOM CỤM)</span>
                      <span class="scada-led led-green" id="hmi-led-belt2"></span>
                    </div>
                    <div class="belt-anim-track" id="hmi-track-belt2">
                      <span class="belt-dot"></span>
                      <span class="belt-dot"></span>
                    </div>
                    <div class="belt-info-line">Trạm DI_11: <b id="hmi-val-di11" style="color:#4ade80;">WAITING</b></div>
                  </div>

                  <!-- 6-DOF Robot Arm Station -->
                  <div class="synoptic-robot-node" style="border-color: #38bdf8;">
                    <span class="robot-icon-tag" style="color: #38bdf8;">🦾 6-DOF ARM</span>
                    <span class="robot-state-tag" id="hmi-state-arm">IDLE</span>
                    <span class="robot-sub-info" id="hmi-arm-vacuum">Multi-Vac: OFF</span>
                  </div>

                  <!-- Belt 3 (Carton Box) -->
                  <div class="synoptic-belt-node">
                    <div class="belt-label-box">
                      <span>BĂNG TẢI 3 (THÙNG CARTON)</span>
                      <span class="scada-led led-yellow" id="hmi-led-belt3"></span>
                    </div>
                    <div class="belt-anim-track" id="hmi-track-belt3">
                      <span class="belt-dot"></span>
                    </div>
                    <div class="belt-info-line">Stopper DI_13: <b id="hmi-val-di13">BOX IN POS</b></div>
                  </div>

                </div>
              </div>

              <!-- Production OEE Metrics Dashboard -->
              <div class="vipa-metrics-row">
                <div class="vipa-kpi-card">
                  <span class="kpi-title">SẢN PHẨM TRONG CỤM</span>
                  <div class="kpi-val"><span id="hmi-kpi-bars-in-cluster">0</span> <small>/ 4 thanh</small></div>
                  <div class="kpi-progress-bar"><div class="kpi-progress-fill" id="hmi-prog-cluster" style="width: 0%;"></div></div>
                </div>

                <div class="vipa-kpi-card">
                  <span class="kpi-title">CỤM TRONG THÙNG</span>
                  <div class="kpi-val"><span id="hmi-kpi-clusters-in-box">0</span> <small>/ 2 cụm</small></div>
                  <div class="kpi-progress-bar"><div class="kpi-progress-fill" id="hmi-prog-box" style="width: 0%; background: #0284c7;"></div></div>
                </div>

                <div class="vipa-kpi-card">
                  <span class="kpi-title">TỔNG THÙNG XUẤT XƯỞNG</span>
                  <div class="kpi-val" style="color: #4ade80;"><span id="hmi-kpi-total-boxes">0</span> <small>thùng</small></div>
                  <span class="kpi-sub">Quy đổi: <b id="hmi-kpi-total-bars">0</b> thanh kẹo</span>
                </div>

                <div class="vipa-kpi-card">
                  <span class="kpi-title">TỐC ĐỘ DÂY CHUYỀN & OEE</span>
                  <div class="kpi-val" id="hmi-kpi-line-speed" style="color: #facc15;">0.0 <small>thanh/phút</small></div>
                  <span class="kpi-sub">Hiệu suất OEE: <b id="hmi-kpi-oee-val" style="color: #22c55e;">100.0%</b> (<span id="hmi-oee-avail">100%</span>A · <span id="hmi-oee-perf">100%</span>P · <span id="hmi-oee-qual">100%</span>Q)</span>
                </div>
              </div>

            </div>

            <!-- ===============================================================
                 TAB 2: RECIPE & SETTINGS
                 =============================================================== -->
            <div class="vipa-page" id="vipa-page-recipe">
              <div class="vipa-synoptic-card">
                <div class="vipa-card-header">CẤU HÌNH THÔNG SỐ ĐÓNG GÓI (RECIPE PARAMETERS)</div>
                <div class="vipa-form-grid">
                  
                  <div class="vipa-form-group">
                    <label>Số lượng thanh sản phẩm trong 1 cụm gom (Làn Băng tải 2):</label>
                    <select class="vipa-select" id="recipe-bars-per-cluster">
                      <option value="3">3 Thanh / Cụm</option>
                      <option value="4" selected>4 Thanh / Cụm (Tiêu chuẩn)</option>
                      <option value="5">5 Thanh / Cụm</option>
                    </select>
                  </div>

                  <div class="vipa-form-group">
                    <label>Số lượng cụm xếp trong 1 Thùng Carton (Băng tải 3):</label>
                    <select class="vipa-select" id="recipe-clusters-per-box">
                      <option value="1">1 Cụm (1 Lớp = 4 thanh)</option>
                      <option value="2" selected>2 Cụm (2 Lớp = 8 thanh - Chuẩn Đồ Án)</option>
                      <option value="3">3 Cụm (3 Lớp = 12 thanh)</option>
                    </select>
                  </div>

                  <div class="vipa-form-group">
                    <label>Tần số định mức Biến tần J1000 Băng tải (Hz):</label>
                    <div style="display: flex; gap: 8px; align-items: center;">
                      <input type="range" min="20" max="60" value="50" class="vipa-range" id="recipe-inverter-hz">
                      <span id="recipe-hz-val" style="font-family: var(--font-mono); font-weight: 700; color:#38bdf8; min-width: 50px;">50.0 Hz</span>
                    </div>
                  </div>

                  <div class="vipa-form-group">
                    <label>Thời gian trễ hút chân không (Dwell Time Pick/Place - giây):</label>
                    <input type="number" step="0.05" min="0.05" max="1.0" value="0.20" class="vipa-input" id="recipe-dwell-time">
                  </div>

                  <div class="vipa-form-group">
                    <label>Độ cao đặt hàng Thùng Carton Lớp 1 (Z mm):</label>
                    <input type="number" value="625" class="vipa-input" id="recipe-z-layer1">
                  </div>

                  <div class="vipa-form-group">
                    <label>Độ cao đặt hàng Thùng Carton Lớp 2 (Z mm):</label>
                    <input type="number" value="665" class="vipa-input" id="recipe-z-layer2">
                  </div>

                </div>

                <div style="margin-top: 12px; display: flex; justify-content: flex-end; gap: 8px;">
                  <button class="scada-btn btn-secondary" id="btn-recipe-default">LOAD DEFAULT</button>
                  <button class="scada-btn btn-primary" id="btn-recipe-save" style="background:#0284c7;">
                    💾 LƯU & NẠP VÀO THANH GHI MP3300
                  </button>
                </div>
              </div>
            </div>

            <!-- ===============================================================
                 TAB 3: ALARM & DIAGNOSTICS
                 =============================================================== -->
            <div class="vipa-page" id="vipa-page-alarm">
              <div class="vipa-synoptic-card">
                <div class="vipa-card-header" style="display:flex; justify-content:space-between; align-items:center;">
                  <span>NHẬT KÝ SỰ CỐ & CẢNH BÁO HỆ THỐNG (ALARM & EVENT LOG)</span>
                  <button class="scada-btn btn-primary" id="btn-hmi-alarm-reset" style="background:#15803d; border-color:#22c55e; padding:3px 10px; font-size:9.5px;">
                    ✅ ALARM RESET (XÓA LỖI & TẮT ĐÈN DO_19)
                  </button>
                </div>

                <!-- Active Alarm Banner -->
                <div id="hmi-alarm-active-container">
                  <!-- Dynamically populated -->
                </div>

                <!-- Event History Table -->
                <div class="vipa-alarm-table-wrap">
                  <table class="vipa-table" id="hmi-alarm-table">
                    <thead>
                      <tr>
                        <th style="width: 80px;">MÃ LỖI</th>
                        <th style="width: 85px;">THỜI GIAN</th>
                        <th style="width: 70px;">MỨC ĐỘ</th>
                        <th>NỘI DUNG SỰ CỐ / THAO TÁC HỆ THỐNG</th>
                        <th style="width: 90px;">TRẠNG THÁI</th>
                      </tr>
                    </thead>
                    <tbody id="hmi-alarm-tbody">
                      <!-- Rendered by JS -->
                    </tbody>
                  </table>
                </div>

                <div style="font-family: var(--font-mono); font-size: 8px; color: var(--text-dim); margin-top: 6px;">
                  * Theo tiêu chuẩn Yaskawa: Khi có sự cố, đèn chỉ báo DO_19 trên mặt tủ sẽ nhấp nháy liên tục cho đến khi người vận hành xác nhận ALARM RESET.
                </div>
              </div>
            </div>

          </div>

        </div>

      </div>
    `;

    this.renderAlarmTable();
  }

  bindEvents() {
    // 1. Tab switching
    const tabBtns = [
      { id: 'vipa-tab-btn-overview', page: 'overview' },
      { id: 'vipa-tab-btn-recipe', page: 'recipe' },
      { id: 'vipa-tab-btn-alarm', page: 'alarm' }
    ];

    tabBtns.forEach(item => {
      const btn = document.getElementById(item.id);
      if (btn) {
        btn.addEventListener('click', () => {
          this.switchTab(item.page);
        });
      }
    });

    // 2. Recipe Inverter Hz slider
    const sHz = document.getElementById('recipe-inverter-hz');
    const lblHz = document.getElementById('recipe-hz-val');
    if (sHz) {
      sHz.addEventListener('input', (e) => {
        const val = parseFloat(e.target.value);
        this.inverterFrequencyHz = val;
        if (lblHz) lblHz.textContent = `${val.toFixed(1)} Hz`;
        const hzOverview = document.getElementById('hmi-val-hz1');
        if (hzOverview) hzOverview.textContent = `${val.toFixed(1)} Hz`;
      });
    }

    // 3. Save Recipe Button
    const btnSaveRecipe = document.getElementById('btn-recipe-save');
    if (btnSaveRecipe) {
      btnSaveRecipe.addEventListener('click', () => {
        this.barsPerGroupRecipe = parseInt(document.getElementById('recipe-bars-per-cluster').value, 10);
        this.groupsPerBoxRecipe = parseInt(document.getElementById('recipe-clusters-per-box').value, 10);
        this.pickDwellSec = parseFloat(document.getElementById('recipe-dwell-time').value);

        if (this.hardwareBus) {
          // Write recipes to MP3300 M-registers
          this.hardwareBus.writeMW(20, this.barsPerGroupRecipe);
          this.hardwareBus.writeMW(21, this.groupsPerBoxRecipe);
        }

        btnSaveRecipe.textContent = 'ĐÃ LƯU THÀNH CÔNG!';
        setTimeout(() => {
          btnSaveRecipe.textContent = '💾 LƯU & NẠP VÀO THANH GHI MP3300';
        }, 1200);

        this.addAlarmEvent('INF-010', 'RECIPE', `Đã cập nhật công thức sản xuất: ${this.barsPerGroupRecipe} thanh/cụm, ${this.groupsPerBoxRecipe} cụm/thùng.`);
      });
    }

    // 4. Alarm Reset Button
    const btnAlarmReset = document.getElementById('btn-hmi-alarm-reset');
    if (btnAlarmReset) {
      btnAlarmReset.addEventListener('click', () => {
        this.clearAllAlarms();
      });
    }
  }

  switchTab(pageName) {
    this.activeHMITab = pageName;
    document.querySelectorAll('.vipa-tab-btn').forEach(b => b.classList.remove('active'));
    document.querySelectorAll('.vipa-page').forEach(p => p.classList.remove('active'));

    const activeBtn = document.getElementById(`vipa-tab-btn-${pageName}`);
    const activePage = document.getElementById(`vipa-page-${pageName}`);
    if (activeBtn) activeBtn.classList.add('active');
    if (activePage) activePage.classList.add('active');
  }

  // ===========================================================================
  // ALARM MANAGEMENT & DO_19 BEACON FLASHER
  // ===========================================================================

  addAlarm(alarmId, severity = 'EMERGENCY', alarmText = '') {
    this.triggerAlarm(alarmId, alarmText, severity);
  }

  triggerAlarm(alarmId, alarmText, severity = 'CRITICAL') {
    // Check if already active
    if (this.activeAlarms.some(a => a.id === alarmId)) return;

    const alarmObj = {
      id: alarmId,
      time: new Date().toLocaleTimeString(),
      severity,
      text: alarmText
    };

    this.activeAlarms.push(alarmObj);
    this.addAlarmEvent(alarmId, severity, alarmText);

    // Start blinking DO_19 beacon lamp
    this.startAlarmBlinking();

    this.updateAlarmVisuals();
  }

  clearAllAlarms() {
    this.activeAlarms = [];
    this.stopAlarmBlinking();
    this.addAlarmEvent('RST-000', 'RESET', 'Người vận hành đã thực hiện ALARM RESET. Hệ thống trở lại bình thường.');
    this.updateAlarmVisuals();
  }

  clearAlarm(alarmId) {
    this.activeAlarms = this.activeAlarms.filter(a => a.id !== alarmId);
    if (this.activeAlarms.length === 0) {
      this.stopAlarmBlinking();
    }
    this.updateAlarmVisuals();
  }

  startAlarmBlinking() {
    if (this.alarmBlinkTimer) return;
    this.alarmBlinkTimer = setInterval(() => {
      this.alarmBlinkState = !this.alarmBlinkState;
      if (this.hardwareBus) {
        // DO_19 is mapped to Alarm Beacon
        this.hardwareBus.setDO(19, this.alarmBlinkState ? 1 : 0);
      }
    }, 250); // 2 Hz blink rate
  }

  stopAlarmBlinking() {
    if (this.alarmBlinkTimer) {
      clearInterval(this.alarmBlinkTimer);
      this.alarmBlinkTimer = null;
    }
    if (this.hardwareBus) {
      this.hardwareBus.setDO(19, 0);
    }
  }

  addAlarmEvent(id, type, msg) {
    this.alarmHistory.unshift({
      id,
      time: new Date().toLocaleTimeString(),
      type,
      msg
    });
    if (this.alarmHistory.length > 50) this.alarmHistory.pop();
    this.renderAlarmTable();
  }

  renderAlarmTable() {
    const tbody = document.getElementById('hmi-alarm-tbody');
    if (!tbody) return;

    tbody.innerHTML = '';
    this.alarmHistory.forEach(item => {
      const tr = document.createElement('tr');
      const isErr = item.type === 'CRITICAL' || item.type === 'ERROR';
      tr.innerHTML = `
        <td style="color:${isErr ? '#ef4444' : '#38bdf8'}; font-weight:700;">${item.id}</td>
        <td>${item.time}</td>
        <td><span class="badge ${isErr ? 'badge-danger' : 'badge-primary'}">${item.type}</span></td>
        <td style="color:${isErr ? '#f87171' : '#cbd5e1'};">${item.msg}</td>
        <td><span style="color:${isErr ? '#facc15' : '#4ade80'};">${isErr ? 'ACTIVE' : 'LOGGED'}</span></td>
      `;
      tbody.appendChild(tr);
    });
  }

  updateAlarmVisuals() {
    const ticker = document.getElementById('vipa-alarm-ticker');
    const tickerText = document.getElementById('vipa-alarm-ticker-text');
    const badge = document.getElementById('vipa-alarm-counter-badge');
    const activeBox = document.getElementById('hmi-alarm-active-container');

    const hasAlarms = this.activeAlarms.length > 0;

    if (ticker) {
      ticker.style.display = hasAlarms ? 'flex' : 'none';
      if (hasAlarms && tickerText) {
        tickerText.textContent = `🚨 [SỰ CỐ ĐANG KÍCH HOẠT]: ${this.activeAlarms[0].text}`;
      }
    }

    if (badge) {
      badge.style.display = hasAlarms ? 'inline-block' : 'none';
      badge.textContent = `${this.activeAlarms.length}`;
    }

    if (activeBox) {
      if (hasAlarms) {
        activeBox.innerHTML = this.activeAlarms.map(a => `
          <div style="background: rgba(239, 68, 68, 0.15); border: 1px solid #ef4444; border-radius: 4px; padding: 6px 10px; margin-bottom: 6px; display:flex; justify-content:space-between; align-items:center;">
            <div>
              <b style="color: #ef4444;">${a.id}:</b> <span style="color: #f87171;">${a.text}</span>
            </div>
            <span style="font-size: 8px; color: var(--text-dim);">${a.time}</span>
          </div>
        `).join('');
      } else {
        activeBox.innerHTML = `
          <div style="background: rgba(34, 197, 94, 0.15); border: 1px solid #22c55e; border-radius: 4px; padding: 6px 10px; margin-bottom: 6px; color: #4ade80; font-family: var(--font-mono); font-size: 9px;">
            ✔ TẤT CẢ TRẠM HOẠT ĐỘNG BÌNH THƯỜNG - KHÔNG CÓ SỰ CỐ
          </div>
        `;
      }
    }
  }

  // ===========================================================================
  // REAL-TIME SCREEN DATA UPDATE (CALLED EVERY FRAME / SCAN)
  // ===========================================================================

  updateScreenData() {
    // 1. Clock
    const clockEl = document.getElementById('hmi-clock');
    if (clockEl) {
      clockEl.textContent = new Date().toLocaleTimeString();
    }

    if (!this.hardwareBus) return;

    // 2. Belt & Inverter Status
    const belt1Active = this.hardwareBus.getDO(0) === 1;
    const belt3Active = this.hardwareBus.getDO(3) === 1;

    const ledB1 = document.getElementById('hmi-led-belt1');
    const ledB2 = document.getElementById('hmi-led-belt2');
    const ledB3 = document.getElementById('hmi-led-belt3');

    if (ledB1) ledB1.classList.toggle('active', belt1Active);
    if (ledB2) ledB2.classList.toggle('active', belt1Active);
    if (ledB3) ledB3.classList.toggle('active', belt3Active);

    // 3. DI_11 Sensor & DI_13 Stopper
    const di11Val = document.getElementById('hmi-val-di11');
    if (di11Val) {
      const isDi11 = this.hardwareBus.getDI(11) === 1;
      di11Val.textContent = isDi11 ? 'PACK READY (ON)' : 'WAITING';
      di11Val.style.color = isDi11 ? '#22c55e' : '#94a3b8';
    }

    const di13Val = document.getElementById('hmi-val-di13');
    if (di13Val) {
      const isDi13 = this.hardwareBus.getDI(13) === 1;
      di13Val.textContent = isDi13 ? 'BOX IN POS' : 'DISCHARGING';
      di13Val.style.color = isDi13 ? '#4ade80' : '#facc15';
    }

    // 4. Robot Vacuum & Work States
    const stateArm = document.getElementById('hmi-state-arm');
    const vacArm = document.getElementById('hmi-arm-vacuum');
    if (stateArm && this.app) {
      const busy = this.app.isSingleCycleRunning;
      stateArm.textContent = busy ? 'PACKING' : 'IDLE';
      stateArm.style.color = busy ? '#facc15' : '#4ade80';
    }
    if (vacArm && this.physicsEngine) {
      const isArmVac = this.physicsEngine.armVacuum.active;
      vacArm.textContent = isArmVac ? 'Multi-Vac: ON (-85 kPa)' : 'Multi-Vac: OFF';
      vacArm.style.color = isArmVac ? '#22c55e' : '#94a3b8';
    }

    const stateDelta = document.getElementById('hmi-state-delta');
    const vacDelta = document.getElementById('hmi-delta-vacuum');
    if (stateDelta && this.physicsEngine) {
      const isDeltaVac = this.physicsEngine.deltaVacuum.active;
      stateDelta.textContent = isDeltaVac ? 'TRANSFERRING' : (belt1Active ? 'SCANNING' : 'STANDBY');
      if (vacDelta) {
        vacDelta.textContent = isDeltaVac ? 'Vac: -85.0 kPa' : 'Vac: 0.0 kPa';
        vacDelta.style.color = isDeltaVac ? '#22c55e' : '#94a3b8';
      }
    }

    // 5. OEE Counters
    const kpiCluster = document.getElementById('hmi-kpi-bars-in-cluster');
    const progCluster = document.getElementById('hmi-prog-cluster');
    if (kpiCluster && this.factory) {
      const collatedCount = this.factory.collatedPacks.length > 0 ? 4 : (this.physicsEngine && this.physicsEngine.deltaVacuum.active ? 1 : 0);
      kpiCluster.textContent = `${collatedCount}`;
      if (progCluster) progCluster.style.width = `${(collatedCount / 4) * 100}%`;
    }

    const kpiBox = document.getElementById('hmi-kpi-clusters-in-box');
    const progBox = document.getElementById('hmi-prog-box');
    if (kpiBox) {
      const clusterCount = this.hardwareBus.plcBoxClusterCount || 0;
      kpiBox.textContent = `${clusterCount}`;
      if (progBox) progBox.style.width = `${(clusterCount / 2) * 100}%`;
    }

    const kpiTotalBoxes = document.getElementById('hmi-kpi-total-boxes');
    const kpiTotalBars = document.getElementById('hmi-kpi-total-bars');
    if (kpiTotalBoxes) kpiTotalBoxes.textContent = `${this.totalBoxesShipped}`;
    if (kpiTotalBars) kpiTotalBars.textContent = `${this.totalBoxesShipped * 8 + (this.hardwareBus.plcBoxClusterCount * 4)}`;
  }
}
