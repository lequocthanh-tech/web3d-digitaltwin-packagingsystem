/**
 * FaultInjectionPanel.js
 * 
 * Factory Fault Injection Lab (Chaos Engineering & Diagnostics)
 * Allows instructors and students to simulate real-world factory failures:
 * 1. Belt Jam (Kẹt phôi trên Băng tải)
 * 2. Safety Fence Gate Open (Mở cửa rào bảo vệ -> E-Stop ngắt nguồn)
 * 3. Network Packet Drop (Mất gói tin truyền thông UDP 10040)
 * 4. Photoelectric Sensor DI_11 Failure (Hỏng cảm biến quang)
 * 5. System Master Reset (Khôi phục toàn bộ hệ thống về trạng thái ban đầu)
 */

export class FaultInjectionPanel {
  constructor(app, options = {}) {
    this.app = app;
    this.hardwareBus = app.hardwareBus;
    this.ethernetServer = app.ethernetServer;
    this.factory = app.factory;
    this.physicsEngine = app.physicsEngine;
    this.hmiScreen = app.vipaHMI;

    this.faultStates = {
      beltJam: false,
      safetyGateOpen: false,
      packetDrop: false,
      sensorFail: false
    };

    this.isOpen = false;
  }

  initDOM(containerEl) {
    this.container = containerEl;
    this.renderPanelUI();
    this.bindEvents();
  }

  renderPanelUI() {
    this.container.innerHTML = `
      <div class="fault-panel-drawer" id="fault-panel-drawer">
        <div class="fault-panel-header">
          <div style="display:flex; align-items:center; gap:6px;">
            <span style="font-size:16px;">⚡</span>
            <div>
              <div style="font-weight:900; color:#ef4444; font-family:var(--font-mono); font-size:11px;">
                FACTORY FAULT INJECTION LAB // CHAOS SIMULATION
              </div>
              <div style="font-size:8px; color:var(--text-dim); font-family:var(--font-mono);">
                GIẢ LẬP SỰ CỐ CÔNG NGHIỆP CHO ĐỒ ÁN & ĐÀO TẠO
              </div>
            </div>
          </div>
          <button class="scada-btn btn-secondary" id="btn-close-fault-panel" style="padding:2px 8px; font-size:10px;">✕ ĐÓNG</button>
        </div>

        <div class="fault-cards-grid">
          
          <!-- Fault 1: Belt Jam -->
          <div class="fault-card" id="card-fault-jam">
            <div class="fault-card-top">
              <span class="fault-badge">FAULT 01</span>
              <span class="fault-status-tag" id="status-fault-jam">NORMAL</span>
            </div>
            <div class="fault-title">Kẹt sản phẩm trên Băng tải 1 (Belt Jam)</div>
            <div class="fault-desc">
              Giả lập thanh kẹo bị kẹt góc thành băng tải, gây quá tải motor biến tần J1000. Dừng băng tải, bật cờ báo lỗi và nhấp nháy đèn DO_19.
            </div>
            <div class="fault-action-row">
              <button class="scada-btn btn-danger" id="btn-inject-jam">⚡ INJECT JAM</button>
            </div>
          </div>

          <!-- Fault 2: Safety Gate Open -->
          <div class="fault-card" id="card-fault-gate">
            <div class="fault-card-top">
              <span class="fault-badge">FAULT 02</span>
              <span class="fault-status-tag" id="status-fault-gate">CLOSED</span>
            </div>
            <div class="fault-title">Mở cửa hàng rào an toàn (Safety Fence)</div>
            <div class="fault-desc">
              Kích hoạt công tắc an toàn rào chắn (Interlock Switch). Ngắt tức thì nguồn Servo toàn hệ thống, khóa cơ cấu robot và xả áp chân không.
            </div>
            <div class="fault-action-row">
              <button class="scada-btn btn-danger" id="btn-inject-gate">🚪 OPEN GATE</button>
            </div>
          </div>

          <!-- Fault 3: Network Packet Drop -->
          <div class="fault-card" id="card-fault-packet">
            <div class="fault-card-top">
              <span class="fault-badge">FAULT 03</span>
              <span class="fault-status-tag" id="status-fault-packet">ONLINE</span>
            </div>
            <div class="fault-title">Mất gói tin mạng UDP (Packet Loss 10040)</div>
            <div class="fault-desc">
              Cắt đứt luồng truyền thông nhị phân YERC giữa module 218IFD và YRC1000micro. Kiểm tra cơ chế xử lý lỗi timeout và cờ Error GB000104 của PLC.
            </div>
            <div class="fault-action-row">
              <button class="scada-btn btn-danger" id="btn-inject-packet">📡 DROP PACKETS</button>
            </div>
          </div>

          <!-- Fault 4: Sensor DI_11 Failure -->
          <div class="fault-card" id="card-fault-sensor">
            <div class="fault-card-top">
              <span class="fault-badge">FAULT 04</span>
              <span class="fault-status-tag" id="status-fault-sensor">OK</span>
            </div>
            <div class="fault-title">Hỏng cảm biến quang DI_11 (Sensor Fail)</div>
            <div class="fault-desc">
              Làm cảm biến DI_11 bị kẹt trạng thái OFF vĩnh viễn dù sản phẩm đã vào vị trí. Dây chuyền dừng chờ vô hạn, kích hoạt cảnh báo trạm gom hàng.
            </div>
            <div class="fault-action-row">
              <button class="scada-btn btn-danger" id="btn-inject-sensor">❌ FAIL SENSOR</button>
            </div>
          </div>

        </div>

        <!-- Master Recovery & Reset Row -->
        <div class="fault-master-actions">
          <button class="scada-btn btn-secondary" id="btn-fault-clear-all" style="border-color:#22c55e; color:#4ade80; padding:6px 14px; font-weight:800;">
            ✔ CLEAR ALL FAULTS (KHÔI PHỤC BÌNH THƯỜNG)
          </button>
          <button class="scada-btn btn-primary" id="btn-fault-master-reset" style="background:#ca8a04; border-color:#facc15; padding:6px 16px; font-weight:800; color:#000;">
            🔄 SYSTEM MASTER RESET (ĐẶT LẠI BAN ĐẦU)
          </button>
        </div>
      </div>
    `;
  }

  bindEvents() {
    const drawer = document.getElementById('fault-panel-drawer');
    const btnClose = document.getElementById('btn-close-fault-panel');

    if (btnClose) {
      btnClose.addEventListener('click', () => {
        this.toggle(false);
      });
    }

    // 1. Fault 1: Belt Jam
    const btnJam = document.getElementById('btn-inject-jam');
    if (btnJam) {
      btnJam.addEventListener('click', () => {
        this.toggleBeltJam();
      });
    }

    // 2. Fault 2: Safety Gate Open
    const btnGate = document.getElementById('btn-inject-gate');
    if (btnGate) {
      btnGate.addEventListener('click', () => {
        this.toggleSafetyGate();
      });
    }

    // 3. Fault 3: Network Packet Drop
    const btnPacket = document.getElementById('btn-inject-packet');
    if (btnPacket) {
      btnPacket.addEventListener('click', () => {
        this.togglePacketDrop();
      });
    }

    // 4. Fault 4: Sensor DI_11 Failure
    const btnSensor = document.getElementById('btn-inject-sensor');
    if (btnSensor) {
      btnSensor.addEventListener('click', () => {
        this.toggleSensorFailure();
      });
    }

    // 5. Clear All Faults
    const btnClearAll = document.getElementById('btn-fault-clear-all');
    if (btnClearAll) {
      btnClearAll.addEventListener('click', () => {
        this.clearAllFaults();
      });
    }

    // 6. Master Reset
    const btnMasterReset = document.getElementById('btn-fault-master-reset');
    if (btnMasterReset) {
      btnMasterReset.addEventListener('click', () => {
        this.executeMasterReset();
      });
    }
  }

  toggle(show = null) {
    this.isOpen = (show !== null) ? show : !this.isOpen;
    const drawer = document.getElementById('fault-panel-drawer');
    if (drawer) {
      drawer.classList.toggle('open', this.isOpen);
    }
  }

  // ===========================================================================
  // FAULT HANDLERS
  // ===========================================================================

  toggleBeltJam() {
    this.faultStates.beltJam = !this.faultStates.beltJam;
    const isAct = this.faultStates.beltJam;

    const btn = document.getElementById('btn-inject-jam');
    const status = document.getElementById('status-fault-jam');
    const card = document.getElementById('card-fault-jam');

    if (isAct) {
      if (btn) btn.textContent = 'RESTORE BELT';
      if (status) { status.textContent = 'JAMMED!'; status.className = 'fault-status-tag error'; }
      if (card) card.classList.add('active-fault');

      // Stop belt output and trigger alarm
      if (this.hardwareBus) {
        this.hardwareBus.setDO(0, 0);
      }
      if (this.app.vipaHMI) {
        this.app.vipaHMI.triggerAlarm('ALM-01', 'KẸT PHÔI BĂNG TẢI 1 - QUÁ TẢI BIẾN TẦN J1000!');
      }
    } else {
      if (btn) btn.textContent = '⚡ INJECT JAM';
      if (status) { status.textContent = 'NORMAL'; status.className = 'fault-status-tag'; }
      if (card) card.classList.remove('active-fault');
    }
  }

  toggleSafetyGate() {
    if (this.app && typeof this.app.toggleSafetyDoor === 'function') {
      this.app.toggleSafetyDoor();
      return;
    }

    this.faultStates.safetyGateOpen = !this.faultStates.safetyGateOpen;
    const isAct = this.faultStates.safetyGateOpen;

    const btn = document.getElementById('btn-inject-gate');
    const status = document.getElementById('status-fault-gate');
    const card = document.getElementById('card-fault-gate');

    if (isAct) {
      if (btn) btn.textContent = 'CLOSE GATE';
      if (status) { status.textContent = 'OPEN (E-STOP)'; status.className = 'fault-status-tag error'; }
      if (card) card.classList.add('active-fault');

      // Trip E-Stop
      if (this.hardwareBus) {
        this.hardwareBus.setEmergencyStop(true);
      }
      const estopBtn = document.getElementById('ctrl-estop');
      if (estopBtn) estopBtn.classList.add('locked');

      if (this.app.vipaHMI) {
        this.app.vipaHMI.triggerAlarm('ALM-02', 'CỬA HÀNG RÀO AN TOÀN MỞ - KHÓA DỪNG KHẨN CẤP (E-STOP)!');
      }
    } else {
      if (btn) btn.textContent = '🚪 OPEN GATE';
      if (status) { status.textContent = 'CLOSED'; status.className = 'fault-status-tag'; }
      if (card) card.classList.remove('active-fault');

      if (this.hardwareBus) {
        this.hardwareBus.setEmergencyStop(false);
      }
      const estopBtn = document.getElementById('ctrl-estop');
      if (estopBtn) estopBtn.classList.remove('locked');
    }
  }

  togglePacketDrop() {
    this.faultStates.packetDrop = !this.faultStates.packetDrop;
    const isAct = this.faultStates.packetDrop;

    const btn = document.getElementById('btn-inject-packet');
    const status = document.getElementById('status-fault-packet');
    const card = document.getElementById('card-fault-packet');

    if (isAct) {
      if (btn) btn.textContent = 'RESTORE NETWORK';
      if (status) { status.textContent = 'DROPPING (ERR 0x1F)'; status.className = 'fault-status-tag error'; }
      if (card) card.classList.add('active-fault');

      if (this.ethernetServer) {
        this.ethernetServer.simulatePacketDrop = true;
      }
      if (this.app.vipaHMI) {
        this.app.vipaHMI.triggerAlarm('ALM-03', 'MẤT GÓI TIN UDP 10040 - 218IFD TIMEOUT (MẤT LIÊN LẠC ROBOT)!');
      }
    } else {
      if (btn) btn.textContent = '📡 DROP PACKETS';
      if (status) { status.textContent = 'ONLINE'; status.className = 'fault-status-tag'; }
      if (card) card.classList.remove('active-fault');

      if (this.ethernetServer) {
        this.ethernetServer.simulatePacketDrop = false;
      }
    }
  }

  toggleSensorFailure() {
    this.faultStates.sensorFail = !this.faultStates.sensorFail;
    const isAct = this.faultStates.sensorFail;

    const btn = document.getElementById('btn-inject-sensor');
    const status = document.getElementById('status-fault-sensor');
    const card = document.getElementById('card-fault-sensor');

    if (isAct) {
      if (btn) btn.textContent = 'RESTORE SENSOR';
      if (status) { status.textContent = 'DEAD (STUCK OFF)'; status.className = 'fault-status-tag error'; }
      if (card) card.classList.add('active-fault');

      if (this.factory) {
        this.factory.sensorDI11Failure = true;
      }
      if (this.app.vipaHMI) {
        this.app.vipaHMI.triggerAlarm('ALM-04', 'HỎNG CẢM BIẾN QUANG DI_11 - MẤT TÍN HIỆU GOM CỤM!');
      }
    } else {
      if (btn) btn.textContent = '❌ FAIL SENSOR';
      if (status) { status.textContent = 'OK'; status.className = 'fault-status-tag'; }
      if (card) card.classList.remove('active-fault');

      if (this.factory) {
        this.factory.sensorDI11Failure = false;
      }
    }
  }

  clearAllFaults() {
    if (this.faultStates.beltJam) this.toggleBeltJam();
    if (this.faultStates.safetyGateOpen) this.toggleSafetyGate();
    if (this.faultStates.packetDrop) this.togglePacketDrop();
    if (this.faultStates.sensorFail) this.toggleSensorFailure();

    if (this.app.vipaHMI) {
      this.app.vipaHMI.clearAllAlarms();
    }
  }

  // ===========================================================================
  // MASTER SYSTEM RESET
  // ===========================================================================

  async executeMasterReset() {
    this.clearAllFaults();

    // 1. Stop all trajectories & clear busy/auto flags
    if (this.app.trajectoryPlanner) {
      this.app.trajectoryPlanner.stopAll();
    }
    this.app.isDeltaBusy = false;
    this.app.isArmBusy = false;
    this.app.isSingleCycleRunning = false;
    this.app.autoDemoMode = false;
    this.app.isCollisionActive = false;

    const btnAutoDemo = document.getElementById('btn-auto-demo-cycle');
    const ledAutoDemo = document.getElementById('led-auto-demo');
    const labelAutoDemo = document.getElementById('label-auto-demo');
    if (btnAutoDemo) btnAutoDemo.classList.remove('running');
    if (ledAutoDemo) ledAutoDemo.classList.remove('active');
    if (labelAutoDemo) labelAutoDemo.textContent = "AUTO DEMO CYCLE";

    // 2. Release grippers & clear physics queues
    if (this.physicsEngine) {
      this.physicsEngine.releaseDeltaGripper();
      this.physicsEngine.releaseArmGripper();
      this.physicsEngine.clearFallingObjects();
      this.physicsEngine.armVacuum.currentBoxLayers = 0;
      if (this.physicsEngine.scannedProductIds) {
        this.physicsEngine.scannedProductIds.clear();
      }
      this.physicsEngine.visionQueue = [];
    }

    // 3. Clear and respawn factory items, close safety gate & restore interlock
    if (this.factory) {
      this.factory.clearAllItems();
      this.factory.spawnInitialItems();
      this.factory.conveyor1Active = false;
      this.factory.conveyor2Active = false;
      this.factory.conveyor3Active = false;
      this.factory.toggleSafetyDoor(true);
      this.factory.setAndonState('STANDBY');
    }

    if (this.app.isSafetyInterlockTripped) {
      this.app.resetSafetyInterlock();
    } else {
      const modal = document.getElementById('safety-interlock-modal');
      if (modal) modal.style.display = 'none';
      const btnQuick = document.getElementById('btn-quick-safety-door');
      const ledQuick = document.getElementById('led-safety-door-top');
      const lblQuick = document.getElementById('label-safety-door-top');
      if (btnQuick) btnQuick.classList.remove('door-open');
      if (ledQuick) ledQuick.className = 'scada-led led-green active';
      if (lblQuick) lblQuick.textContent = 'CỬA RÀO: ĐÓNG';
    }

    ['1', '2', '3'].forEach(idx => {
      const t = document.getElementById(`toggle-conv${idx}`);
      const l = document.getElementById(`led-conv${idx}`);
      if (t) { t.checked = false; t.disabled = false; }
      if (l) l.classList.remove('active');
    });

    // 4. Return robots to Home
    if (this.app.arm6DOF) {
      this.app.arm6DOF.setJointAngles(-75.0, -18.0, -10.0, 0.0, -62.0, 0.0);
      this.app.updateArmUIFromModel();
    }
    if (this.app.deltaRobot) {
      this.app.deltaRobot.setJointAngles(20.0, 20.0, 20.0, 0.0);
      this.app.updateDeltaUIFromModel();
    }

    // 5. Reset AGV Logistics & OEE Shift Counters
    if (this.app.agvController) {
      this.app.agvController.reset();
    }
    if (this.app.oeeCalculator) {
      this.app.oeeCalculator.resetShift();
    }

    // 6. Reset Hardware Bus
    if (this.hardwareBus) {
      for (let i = 0; i < 32; i++) {
        this.hardwareBus.setDI(i, 0);
        this.hardwareBus.setDO(i, 0);
      }
      this.hardwareBus.setDI(1, 1); // Gate Closed normal contact
      this.hardwareBus.setEmergencyStop(false);
      this.hardwareBus.plcState = 'IDLE';
      this.hardwareBus.plcBoxClusterCount = 0;
      this.hardwareBus.setRobotMode(2); // REMOTE
    }

    // 7. Reset UI indicators
    const lblBoxLayers = document.getElementById('label-box-layers');
    if (lblBoxLayers) lblBoxLayers.textContent = 'Lớp: 0 / 2';

    const estopBtn = document.getElementById('ctrl-estop');
    if (estopBtn) estopBtn.classList.remove('locked');
    const statusBadge = document.getElementById('ctrl-panel-status');
    if (statusBadge) {
      statusBadge.textContent = 'READY';
      statusBadge.className = 'badge badge-success';
    }

    // 8. Update HMI Counters
    if (this.app.vipaHMI) {
      this.app.vipaHMI.totalBoxesShipped = 0;
      this.app.vipaHMI.totalBarsPacked = 0;
      this.app.vipaHMI.clearAllAlarms();
      this.app.vipaHMI.addAlarmEvent('RST-SYS', 'RESET', 'SYSTEM MASTER RESET: Đã khôi phục toàn bộ dây chuyền về trạng thái ban đầu.');
    }

    this.toggle(false);
  }
}
