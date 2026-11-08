/**
 * TeachPendantStation.js - Version 2.1 (Industrial Authentic Edition)
 * 
 * Virtual Teach Pendant: Yaskawa YRC1000micro Programming Pendant (INFORM III)
 * Fully compliant with Bài thí nghiệm 1 (Kỹ thuật Robot ĐHBK TPHCM).
 * 
 * Key Features:
 * 1. Authentic Industrial Handheld Pendant Ergonomics & Form Factor:
 *    - Deep dark casing (#2b2d30), high-visibility Yaskawa yellow rubber bumpers (#f59e0b).
 *    - High-readability industrial typography (12-14px legible fonts, no microscopic text).
 *    - Large, tactile 3D physical controls: 3-position Rotary Mode Switch [TEACH/PLAY/REMOTE],
 *      Mushroom Emergency Stop, illuminated START and HOLD dome buttons, SERVO ON READY with LED,
 *      and 3-state Deadman Switch (Release, Light Squeeze, Tight Squeeze).
 * 2. Complete Project / Job Management (Chuẩn Trang 12 & 14 Bài 1):
 *    - Select Project: choose which job is loaded for PLAY auto mode.
 *    - Create New Job: create new projects, learn new motion profiles (MOVJ, MOVL, DOUT, TIMER, WAIT, JUMP).
 *    - Save Project & Reload presets.
 * 3. Comprehensive Position Variable Setup (Chuẩn Trang 11 Bài 1):
 *    - P000..P014 database with full coordinate and pulse displays.
 *    - Method 1: Teach Current Pose (Bắt tọa độ 3D hiện tại bằng [MODIFY] + [ENTER]).
 *    - Method 2: Manual Numeric Input (Nhập trực tiếp X, Y, Z, Rx, Ry, Rz hoặc S, L, U, R, B, T).
 *    - Method 3: Test Motion ([FWD TỚI ĐIỂM NÀY] để robot 3D di chuyển kiểm chứng).
 * 4. Safety Interlock, Coordinate Jogging, Step FWD/BWD, and Continuous PLAY Auto Interpreter.
 */

import * as THREE from 'three';
import { LETTER_T_PROJECT, LETTER_T_POS_VARIABLES, runAutoLetterTJob } from './JobLetterT.js';

export class TeachPendantStation {
  constructor(app, options = {}) {
    this.app = app;
    this.robotArm = app.arm6DOF;
    this.kinematics = app.armKinematics;
    this.planner = app.trajectoryPlanner;
    this.hardwareBus = app.hardwareBus;
    this.physicsEngine = app.physicsEngine;

    // Window Display & Docking State: 'docked' or 'floating'
    this.dockMode = options.dockMode || 'docked';
    this.isMinimized = false;
    this.windowPos = { x: null, y: null };
    this.isDragging = false;
    this.dragOffset = { x: 0, y: 0 };

    // Hardware States
    this.robotMode = 0; // 0 = TEACH, 1 = PLAY, 2 = REMOTE
    this.deadmanState = 1; // 0 = RELEASED (OFF), 1 = LIGHT SQUEEZE (ON - SAFE), 2 = TIGHT SQUEEZE (OFF - PANIC)
    this.servoReady = true;
    this.servoOn = true;
    this.eStop = false;
    this.startLit = false;
    this.holdLit = false;

    // Coordinate System: 'JOINT', 'CARTESIAN', 'TOOL'
    this.currentCoord = 'JOINT';

    // Jog Speeds (4 levels: INCH, LOW, MED, HIGH)
    this.jogSpeedMode = 'MED';
    this.jogSpeeds = {
      INCH: { deg: 0.3, mm: 1.5, label: 'INCH (1%)', duration: 0.8 },
      LOW: { deg: 1.2, mm: 6.0, label: 'LOW (5%)', duration: 0.5 },
      MED: { deg: 6.0, mm: 30.0, label: 'MED (25%)', duration: 0.35 },
      HIGH: { deg: 18.0, mm: 90.0, label: 'HIGH (100%)', duration: 0.22 }
    };
    this.highSpeedActive = false;

    // Active Screen & Subtabs in Zone 4
    this.currentScreen = 'JOB'; // 'JOB', 'VARIABLE', 'INOUT', 'ROBOT', 'SYSTEM'
    this.jobSubtab = 'CONTENT'; // 'CONTENT', 'SELECT', 'CREATE'
    this.posVarFormat = 'ROBOT'; // 'ROBOT' vs 'PULSE'

    // Multi-Project / Job Database (Chuẩn Bài 1 ĐHBK TPHCM)
    this.currentJobName = 'PACKAGING_CELL_DEMO';
    this.projects = {
      'PACKAGING_CELL_DEMO': {
        name: 'PACKAGING_CELL_DEMO',
        desc: 'Chu trình đóng gói tự động chuẩn: Gắp cụm 4 thanh từ Băng tải 2 xếp 2 lớp vào thùng carton Băng tải 3',
        instructions: [
          'NOP',
          'MOVJ P000 VJ=50.00',
          'WAIT IN#(1)=ON',
          'MOVJ P001 VJ=80.00',
          'MOVL P002 V=250.0 PL=0',
          'DOUT OT#(1) ON',
          'TIMER T=0.15',
          'MOVL P001 V=300.0',
          'MOVJ P003 VJ=80.00',
          'MOVL P004 V=250.0 PL=0',
          'DOUT OT#(1) OFF',
          'TIMER T=0.15',
          'PULSE OT#(3) T=0.10',
          'MOVL P003 V=300.0',
          'MOVJ P000 VJ=80.00',
          'END'
        ]
      },
      'LAB1_PICK_A1_PLACE_B': {
        name: 'LAB1_PICK_A1_PLACE_B',
        desc: 'Bài thí nghiệm 1 (Yêu cầu 2.1) - Gắp vật tại vị trí A1 đưa sang thả vào ô B',
        instructions: [
          'NOP',
          'MOVJ P000 VJ=20.00',
          'MOVJ P001 VJ=20.00',
          'MOVL P002 V=50.00 PL=0',
          'DOUT OT#(1) ON',
          'TIMER T=0.500',
          'MOVL P001 V=50.00',
          'MOVJ P003 VJ=20.00',
          'MOVL P004 V=50.00 PL=0',
          'DOUT OT#(1) OFF',
          'TIMER T=0.500',
          'MOVL P003 V=50.00',
          'MOVJ P000 VJ=20.00',
          'END'
        ]
      },
      'LAB1_PICK_A2_PLACE_C': {
        name: 'LAB1_PICK_A2_PLACE_C',
        desc: 'Bài thí nghiệm 1 (Yêu cầu 1.2a & 2.1) - Gắp vật tại vị trí A2 đưa sang thả vào ô C',
        instructions: [
          'NOP',
          'MOVJ P000 VJ=20.00',
          'MOVJ P006 VJ=20.00',
          'MOVL P007 V=50.00 PL=0',
          'DOUT OT#(1) ON',
          'TIMER T=0.500',
          'MOVL P006 V=50.00',
          'MOVJ P008 VJ=20.00',
          'MOVL P009 V=50.00 PL=0',
          'DOUT OT#(1) OFF',
          'TIMER T=0.500',
          'MOVL P008 V=50.00',
          'MOVJ P000 VJ=20.00',
          'END'
        ]
      },
      'LAB1_STACK_3_OBJECTS': {
        name: 'LAB1_STACK_3_OBJECTS',
        desc: 'Bài thí nghiệm 1 (Yêu cầu 1.2b) - Gắp 3 vật lần lượt từ A1, A2, A3 thả chồng lên nhau tại B',
        instructions: [
          'NOP',
          'MOVJ P001 VJ=30.00',
          'MOVL P002 V=60.00 PL=0',
          'DOUT OT#(1) ON',
          'TIMER T=0.300',
          'MOVL P001 V=60.00',
          'MOVJ P003 VJ=30.00',
          'MOVL P004 V=60.00 PL=0',
          'DOUT OT#(1) OFF',
          'TIMER T=0.300',
          'MOVL P003 V=60.00',
          'MOVJ P006 VJ=30.00',
          'MOVL P007 V=60.00 PL=0',
          'DOUT OT#(1) ON',
          'TIMER T=0.300',
          'MOVL P006 V=60.00',
          'MOVJ P003 VJ=30.00',
          'MOVL P005 V=60.00 PL=0',
          'DOUT OT#(1) OFF',
          'TIMER T=0.300',
          'MOVL P003 V=60.00',
          'MOVJ P000 VJ=30.00',
          'END'
        ]
      },
      'LAB1_IO_CONTROL': {
        name: 'LAB1_IO_CONTROL',
        desc: 'Bài thí nghiệm 1 (Yêu cầu 2.2) - Chờ công tắc I4..I7 ON, gắp phôi và xuất xung 1s ra đèn Q4',
        instructions: [
          'NOP',
          'WAIT IN#(5)=ON',
          'MOVJ P001 VJ=30.00',
          'MOVL P002 V=60.00 PL=0',
          'DOUT OT#(1) ON',
          'TIMER T=0.300',
          'MOVL P001 V=60.00',
          'MOVJ P003 VJ=30.00',
          'MOVL P004 V=60.00 PL=0',
          'DOUT OT#(1) OFF',
          'PULSE OT#(5) T=1.000',
          'MOVL P003 V=60.00',
          'MOVJ P000 VJ=30.00',
          'END'
        ]
      },
      [LETTER_T_PROJECT.name]: LETTER_T_PROJECT
    };

    // Position Variables P000..P014 and P020..P025 (Letter T Trajectory Points)
    this.selectedVarIndex = 0;
    this.posVariables = [
      { id: 'P000', name: 'HOME_POS', x: -0.98, y: 1.32, z: 0.24, rx: 0, ry: 0, rz: 0, s: -75.0, l: -18.0, u: -10.0, r: 0.0, b: -62.0, t: 0.0, type: 'FRONT UP FLIP' },
      { id: 'P001', name: 'A1_APPROACH', x: -1.25, y: 1.18, z: 0.18, rx: 0, ry: 0, rz: 0, s: -90.0, l: -8.0, u: 12.0, r: 0.0, b: -94.0, t: 0.0, type: 'FRONT UP FLIP' },
      { id: 'P002', name: 'A1_PICK_TOUCH', x: -1.25, y: 0.91, z: 0.18, rx: 0, ry: 0, rz: 0, s: -90.0, l: 12.0, u: 18.0, r: 0.0, b: -120.0, t: 0.0, type: 'FRONT UP FLIP' },
      { id: 'P003', name: 'B_APPROACH', x: -1.25, y: 1.18, z: -1.45, rx: 0, ry: 0, rz: 0, s: 90.0, l: -8.0, u: 12.0, r: 0.0, b: -94.0, t: 0.0, type: 'REAR UP FLIP' },
      { id: 'P004', name: 'B_PLACE_LAYER1', x: -1.25, y: 0.79, z: -1.45, rx: 0, ry: 0, rz: 0, s: 90.0, l: -22.0, u: -18.0, r: 0.0, b: -50.0, t: 0.0, type: 'REAR UP FLIP' },
      { id: 'P005', name: 'B_PLACE_LAYER2', x: -1.25, y: 0.83, z: -1.45, rx: 0, ry: 0, rz: 0, s: 90.0, l: -20.0, u: -16.0, r: 0.0, b: -54.0, t: 0.0, type: 'REAR UP FLIP' },
      { id: 'P006', name: 'A2_APPROACH', x: -1.05, y: 1.18, z: 0.18, rx: 0, ry: 0, rz: 0, s: -80.0, l: -8.0, u: 12.0, r: 0.0, b: -94.0, t: 0.0, type: 'FRONT UP FLIP' },
      { id: 'P007', name: 'A2_PICK_TOUCH', x: -1.05, y: 0.91, z: 0.18, rx: 0, ry: 0, rz: 0, s: -80.0, l: 12.0, u: 18.0, r: 0.0, b: -120.0, t: 0.0, type: 'FRONT UP FLIP' },
      { id: 'P008', name: 'C_APPROACH', x: -1.25, y: 1.18, z: -1.80, rx: 0, ry: 0, rz: 0, s: 95.0, l: -8.0, u: 12.0, r: 0.0, b: -94.0, t: 0.0, type: 'REAR UP FLIP' },
      { id: 'P009', name: 'C_PLACE_LAYER1', x: -1.25, y: 0.79, z: -1.80, rx: 0, ry: 0, rz: 0, s: 95.0, l: -22.0, u: -18.0, r: 0.0, b: -50.0, t: 0.0, type: 'REAR UP FLIP' },
      { id: 'P010', name: 'B_PLACE_LAYER3', x: -1.25, y: 0.87, z: -1.45, rx: 0, ry: 0, rz: 0, s: 90.0, l: -18.0, u: -14.0, r: 0.0, b: -58.0, t: 0.0, type: 'REAR UP FLIP' },
      { id: 'P011', name: 'C_PLACE_LAYER2', x: -1.25, y: 0.83, z: -1.80, rx: 0, ry: 0, rz: 0, s: 95.0, l: -20.0, u: -16.0, r: 0.0, b: -54.0, t: 0.0, type: 'REAR UP FLIP' },
      ...LETTER_T_POS_VARIABLES
    ];

    // Execution & Cursor
    this.currentLineIndex = 0;
    this.isRunningJob = false;
    this.isPaused = false;
    this.jobAborted = false;

    // Buffer Line & Pending Action
    this.bufferLineText = 'MOVJ P001 VJ=50.00';
    this.pendingAction = null;
    this.interlockActive = false;

    // Jog interval
    this.activeJogInterval = null;

    // System Message
    this.systemMessage = "Sẵn sàng vận hành. Gạt Mode Switch sang [TEACH] và bóp nhẹ Deadman để bật Servo.";
    this.systemMessageType = "info";

    this.container = null;
    this.floatingHost = null;
  }

  getCurrentInstructions() {
    const proj = this.projects[this.currentJobName];
    return proj ? proj.instructions : [];
  }

  /**
   * Initializes DOM elements and mounts UI
   */
  initDOM(containerEl = null) {
    if (containerEl) {
      this.container = containerEl;
      this.dockMode = 'docked';
    }

    // Ensure floating host exists
    let host = document.getElementById('yaskawa-pendant-floating');
    if (!host) {
      host = document.createElement('div');
      host.id = 'yaskawa-pendant-floating';
      host.className = 'yaskawa-pendant-floating';
      host.style.display = 'none';
      document.body.appendChild(host);
    }
    this.floatingHost = host;

    this.renderPendantHTML();
    this.bindEvents();
    this.updatePendantDisplay();
    this.updateScreenZone4();
    this.updateLiveReadouts();
    this.applyDockMode(false);
  }

  setDockMode(mode, autoShow = false) {
    this.dockMode = mode;
    this.applyDockMode(autoShow);
  }

  applyDockMode(autoShow = false) {
    const pendantEl = document.getElementById('yaskawa-teach-pendant');
    if (!pendantEl) return;

    if (this.dockMode === 'docked' && this.container) {
      this.floatingHost.style.display = 'none';
      pendantEl.classList.add('is-docked');
      pendantEl.classList.remove('is-floating');
      this.container.appendChild(pendantEl);
    } else {
      pendantEl.classList.remove('is-docked');
      pendantEl.classList.add('is-floating');
      this.floatingHost.appendChild(pendantEl);
      if (autoShow) {
        this.floatingHost.style.display = 'block';
      }

      if (this.windowPos.x === null || this.windowPos.y === null) {
        this.floatingHost.style.right = '24px';
        this.floatingHost.style.top = '72px';
        this.floatingHost.style.left = 'auto';
      } else {
        this.floatingHost.style.left = `${this.windowPos.x}px`;
        this.floatingHost.style.top = `${this.windowPos.y}px`;
        this.floatingHost.style.right = 'auto';
      }
    }

    const btnDock = document.getElementById('yp-btn-toggle-dock');
    if (btnDock) {
      btnDock.innerHTML = (this.dockMode === 'docked') 
        ? '<span>🪟 Thả nổi (Float)</span>' 
        : '<span>📌 Ghim phải (Dock)</span>';
    }
  }

  toggleFloating() {
    if (this.dockMode === 'floating') {
      const isVisible = (this.floatingHost && this.floatingHost.style.display !== 'none');
      if (isVisible) {
        this.floatingHost.style.display = 'none';
      } else {
        this.show();
      }
    } else {
      if (this.app && this.app.activeWorkstation === 'pendant') {
        this.app.switchWorkstation('default');
      }
      this.setDockMode('floating', true);
      this.show();
    }
  }

  show() {
    if (this.dockMode === 'floating' && this.floatingHost) {
      // Chỉ hiển thị 1 tay cầm nổi tại 1 thời điểm: ẩn tay cầm Delta để không chồng lấp
      const other = document.getElementById('delta-pendant-floating');
      if (other) other.style.display = 'none';
      this.floatingHost.style.display = 'block';
      this.updateLiveReadouts();
    }
  }

  hide() {
    if (this.floatingHost) {
      this.floatingHost.style.display = 'none';
    }
    if (this.dockMode === 'docked') {
      const mount = document.getElementById('dock-teach-pendant-mount');
      if (mount && this.container === mount) {
        mount.style.display = 'none';
      }
      this.setDockMode('floating', false);
      if (this.app && this.app.activeWorkstation === 'pendant') {
        this.app.switchWorkstation('default');
      }
    }
  }

  /**
   * Master HTML Render
   */
  renderPendantHTML() {
    const html = `
      <div id="yaskawa-teach-pendant" class="yaskawa-teach-pendant ${this.dockMode === 'docked' ? 'is-docked' : 'is-floating'}">
        
        <!-- ===================================================================
             TOP HEADER: CARRY HANDLE & CONTROLS
             =================================================================== -->
        <div class="yp-chassis-top-handle" id="yp-drag-handle">
          <div class="yp-handle-grip">
            <span class="yp-brand-emboss">YASKAWA</span>
            <span class="yp-model-badge">YRC1000micro // TEACH PENDANT</span>
            <span class="yp-active-proj-pill" id="yp-header-active-proj">PROJECT: ${this.currentJobName}</span>
          </div>

          <div class="yp-window-controls">
            <button class="yp-win-btn" id="yp-btn-toggle-dock" title="Ghim sang phải / Thả nổi">
              ${this.dockMode === 'docked' ? '🪟 Thả nổi' : '📌 Ghim'}
            </button>
            <button class="yp-win-btn" id="yp-btn-minimize" title="Thu nhỏ / Mở rộng">
              ➖
            </button>
            <button class="yp-win-btn yp-win-close" id="yp-btn-close" title="Ẩn tay cầm">
              ✕
            </button>
          </div>
        </div>

        <div class="yp-pendant-body" id="yp-pendant-body">
          
          <!-- ===================================================================
               UPPER PHYSICAL CONTROLS DECK
               =================================================================== -->
          <div class="yp-upper-deck">
            
            <!-- Rotary Mode Switch: [TEACH] - [PLAY] - [REMOTE] -->
            <div class="yp-mode-switch-group">
              <span class="yp-deck-label">MODE SWITCH</span>
              <div class="yp-rotary-dial-wrap" id="yp-mode-rotary" title="Click để xoay chìa khóa: TEACH - PLAY - REMOTE">
                <div class="yp-rotary-labels">
                  <span class="yp-mode-lbl mode-teach" data-mode="0">TEACH</span>
                  <span class="yp-mode-lbl mode-play" data-mode="1">PLAY</span>
                  <span class="yp-mode-lbl mode-remote" data-mode="2">REMOTE</span>
                </div>
                <div class="yp-rotary-knob pos-teach" id="yp-knob-indicator">
                  <div class="yp-knob-notch"></div>
                  <div class="yp-knob-keyhole"></div>
                </div>
              </div>
            </div>

            <!-- Center Industrial Buttons: [START], [HOLD], [SERVO ON READY] -->
            <div class="yp-transport-buttons-group">
              <div class="yp-btn-col">
                <button class="yp-dome-btn btn-start" id="yp-btn-start" title="[START] Bắt đầu chạy tự động Job đang chọn trong chế độ PLAY">
                  <span class="yp-btn-glow"></span>
                  <span class="yp-btn-text">START</span>
                </button>
                <span class="yp-btn-sublabel">CHẠY TỰ ĐỘNG</span>
              </div>

              <div class="yp-btn-col">
                <button class="yp-dome-btn btn-hold" id="yp-btn-hold" title="[HOLD] Tạm dừng chuyển động robot">
                  <span class="yp-btn-glow"></span>
                  <span class="yp-btn-text">HOLD</span>
                </button>
                <span class="yp-btn-sublabel">TẠM DỪNG</span>
              </div>

              <div class="yp-btn-col">
                <button class="yp-servo-btn" id="yp-btn-servo-ready" title="[SERVO ON READY] Sẵn sàng cấp nguồn động lực / Bật Servo">
                  <div class="yp-servo-led-ring">
                    <span class="yp-servo-led" id="yp-led-servo"></span>
                  </div>
                  <span class="yp-servo-btn-text">SERVO ON<br>READY</span>
                </button>
                <span class="yp-btn-sublabel" id="yp-lbl-servo-status">SERVO: ON</span>
              </div>
            </div>

            <!-- Emergency Stop Mushroom Button -->
            <div class="yp-estop-group">
              <span class="yp-deck-label">DỪNG KHẨN CẤP</span>
              <button class="yp-estop-mushroom" id="yp-btn-estop" title="Nhấn [EMERGENCY STOP] để ngắt ngay lập tức nguồn servo. Xoay phải để reset.">
                <div class="yp-estop-head">
                  <span class="yp-estop-text">EMERGENCY<br>STOP</span>
                  <span class="yp-estop-arrow">◄ RESET ►</span>
                </div>
              </button>
            </div>

          </div>

          <!-- Deadman Switch (Enable Switch) Bar -->
          <div class="yp-deadman-bar">
            <div class="yp-deadman-info">
              <span class="yp-deadman-title">DEADMAN SWITCH (ENABLE SWITCH - MẶT SAU):</span>
              <span class="yp-deadman-state-badge badge-safe" id="yp-deadman-state-badge">LIGHT SQUEEZE (ON - SAFE)</span>
            </div>

            <div class="yp-deadman-selector">
              <button class="yp-dm-btn" data-dm="0" id="yp-dm-0" title="Nhả tay (OFF - Ngắt nguồn)">0: RELEASE (OFF)</button>
              <button class="yp-dm-btn active" data-dm="1" id="yp-dm-1" title="Bóp nhẹ nấc giữa (ON - SAFE)">1: LIGHT SQUEEZE (ON)</button>
              <button class="yp-dm-btn" data-dm="2" id="yp-dm-2" title="Bóp quá chặt (OFF - PANIC)">2: TIGHT SQUEEZE (OFF)</button>
            </div>
          </div>

          <!-- ===================================================================
               5-ZONE YASKAWA LCD TOUCH SCREEN
               =================================================================== -->
          <div class="yp-lcd-screen-frame">
            
            <!-- ZONE 1: MENU AREA (TOP) -->
            <div class="yp-screen-zone-1 yp-menu-bar">
              <div class="yp-menu-item" id="yp-menu-data">
                <span>DATA ▼</span>
                <div class="yp-dropdown-menu" id="yp-dd-data">
                  <div class="yp-dd-item" id="yp-mi-save-job">💾 Lưu Project hiện tại</div>
                  <div class="yp-dd-item" id="yp-mi-clear-data">🗑 Xóa dữ liệu điểm đang chọn</div>
                  <div class="yp-dd-item" id="yp-mi-reload-job">🔄 Nạp lại mẫu Bài 1 (Default)</div>
                </div>
              </div>

              <div class="yp-menu-item" id="yp-menu-edit">
                <span>EDIT ▼</span>
                <div class="yp-dropdown-menu" id="yp-dd-edit">
                  <div class="yp-dd-item" id="yp-mi-insert">➕ Chèn dòng lệnh (INSERT)</div>
                  <div class="yp-dd-item" id="yp-mi-modify">✏️ Sửa dòng lệnh (MODIFY)</div>
                  <div class="yp-dd-item" id="yp-mi-delete">❌ Xóa dòng lệnh (DELETE)</div>
                </div>
              </div>

              <div class="yp-menu-item" id="yp-menu-display">
                <span>DISPLAY ▼</span>
                <div class="yp-dropdown-menu" id="yp-dd-display">
                  <div class="yp-dd-item" id="yp-mi-toggle-format">Đổi ROBOT / PULSE</div>
                  <div class="yp-dd-item" id="yp-mi-goto-top">Về đầu chương trình (0000)</div>
                </div>
              </div>

              <div class="yp-menu-item" id="yp-menu-utility">
                <span>UTILITY ▼</span>
                <div class="yp-dropdown-menu" id="yp-dd-utility">
                  <div class="yp-dd-item" id="yp-mi-help-lab1">📖 Xem Hướng Dẫn Bài Thí Nghiệm 1</div>
                  <div class="yp-dd-item" id="yp-mi-io-monitor">⚡ Bảng I/O Monitor Panel</div>
                </div>
              </div>
            </div>

            <!-- ZONE 2: STATUS DISPLAY AREA (TOP RIGHT) -->
            <div class="yp-screen-zone-2 yp-status-bar">
              <span class="yp-status-icon icon-mode" id="yp-ico-mode" title="Chế độ chìa khóa">
                <span class="ico-symbol" id="yp-sym-mode">☝T</span>
                <span class="ico-text" id="yp-txt-mode">TEACH</span>
              </span>

              <span class="yp-status-icon icon-coord" id="yp-ico-coord" title="Hệ tọa độ Jog">
                <span class="ico-symbol" id="yp-sym-coord">☵</span>
                <span class="ico-text" id="yp-txt-coord">JOINT</span>
              </span>

              <span class="yp-status-icon icon-speed" id="yp-ico-speed" title="Cấp tốc độ manual">
                <span class="ico-symbol" id="yp-sym-speed">𝑴</span>
                <span class="ico-text" id="yp-txt-speed">MED</span>
              </span>

              <span class="yp-status-icon icon-state" id="yp-ico-state" title="Trạng thái hoạt động">
                <span class="ico-symbol" id="yp-sym-state">■</span>
                <span class="ico-text" id="yp-txt-state">STOP</span>
              </span>

              <span class="yp-status-icon icon-sec" title="Cấp quyền: Chỉnh sửa">
                <span class="ico-symbol">🔑🔑</span>
              </span>

              <span class="yp-status-icon icon-tool" title="Tool Index">
                <span class="ico-text">TOOL:00</span>
              </span>

              <span class="yp-status-icon icon-servo" id="yp-ico-servo" title="Nguồn động cơ Servo">
                <span class="ico-led" id="yp-sym-servo-led"></span>
                <span class="ico-text" id="yp-txt-servo">SERVO ON</span>
              </span>
            </div>

            <!-- ZONE 3 (MAIN MENU COLUMN) + ZONE 4 (GENERAL PURPOSE DISPLAY) -->
            <div class="yp-screen-middle-row">
              
              <!-- ZONE 3: MAIN MENU (LEFT COLUMN) -->
              <div class="yp-screen-zone-3 yp-main-menu-col">
                <button class="yp-menu-btn active" data-screen="JOB" id="yp-mbtn-job" title="Quản lý Project / Job">
                  <div class="yp-mbtn-ico">📁</div>
                  <div class="yp-mbtn-txt">JOB / PROJ</div>
                </button>
                <button class="yp-menu-btn" data-screen="VARIABLE" id="yp-mbtn-var" title="Quản lý & Cài đặt biến điểm P000..P014">
                  <div class="yp-mbtn-ico">🔢</div>
                  <div class="yp-mbtn-txt">VARIABLE</div>
                </button>
                <button class="yp-menu-btn" data-screen="INOUT" id="yp-mbtn-io" title="Giám sát & Bật tắt I/O">
                  <div class="yp-mbtn-ico">🔄</div>
                  <div class="yp-mbtn-txt">IN/OUT</div>
                </button>
                <button class="yp-menu-btn" data-screen="ROBOT" id="yp-mbtn-robot" title="Work Home Pos">
                  <div class="yp-mbtn-ico">🤖</div>
                  <div class="yp-mbtn-txt">ROBOT</div>
                </button>
                <button class="yp-menu-btn" data-screen="SYSTEM" id="yp-mbtn-sys" title="Thông tin YRC1000micro">
                  <div class="yp-mbtn-ico">ℹ️</div>
                  <div class="yp-mbtn-txt">SYS INFO</div>
                </button>
              </div>

              <!-- ZONE 4: GENERAL PURPOSE DISPLAY AREA (MAIN VIEWPORT) -->
              <div class="yp-screen-zone-4 yp-general-display" id="yp-general-display">
                <!-- Dynamically rendered by JS -->
              </div>

            </div>

            <!-- ZONE 5: HUMAN INTERFACE / BUFFER LINE (BOTTOM) -->
            <div class="yp-screen-zone-5 yp-human-interface">
              <div class="yp-buffer-line-box">
                <span class="yp-buffer-label">BUFFER:</span>
                <input type="text" class="yp-buffer-input" id="yp-buffer-input" value="${this.bufferLineText}" spellcheck="false">
                <button class="yp-buffer-btn" id="yp-btn-exec-buffer" title="Nạp vào dòng đệm">APPLY BUFFER</button>
              </div>

              <div class="yp-prompt-line" id="yp-prompt-line">
                <span class="yp-prompt-prefix" id="yp-prompt-prefix">[INFO]</span>
                <span class="yp-prompt-text" id="yp-prompt-text">${this.systemMessage}</span>
              </div>
            </div>

          </div>

          <!-- ===================================================================
               LOWER PHYSICAL BUTTON DECK (JOGGING, EDITING & TESTING)
               =================================================================== -->
          <div class="yp-lower-deck">
            
            <!-- Left Deck: 6-Axis Jog Keys (12 Buttons) with Dynamic [COORD] Labels -->
            <div class="yp-jog-panel">
              <div class="yp-jog-header">
                <span class="yp-sub-title">CỤM PHÍM JOG 6 TRỤC (NHẤN & GIỮ ĐỂ DI CHUYỂN)</span>
                <div class="yp-coord-cycle-wrap">
                  <button class="yp-hardware-key btn-coord" id="yp-key-coord" title="Chuyển hệ tọa độ: JOINT -> CARTESIAN -> TOOL">
                    [COORD]
                  </button>
                  <span class="yp-coord-badge" id="yp-jog-coord-badge">JOINT</span>
                </div>
              </div>

              <div class="yp-axis-keys-grid" id="yp-axis-keys-grid">
                <!-- Pair 1: S / X -->
                <div class="yp-jog-pair">
                  <button class="yp-jog-key" data-axis="0" data-dir="-1" id="yp-jkey-0-neg">
                    <span class="yp-jkey-sym">S -</span>
                  </button>
                  <button class="yp-jog-key" data-axis="0" data-dir="1" id="yp-jkey-0-pos">
                    <span class="yp-jkey-sym">S +</span>
                  </button>
                </div>

                <!-- Pair 2: L / Y -->
                <div class="yp-jog-pair">
                  <button class="yp-jog-key" data-axis="1" data-dir="-1" id="yp-jkey-1-neg">
                    <span class="yp-jkey-sym">L -</span>
                  </button>
                  <button class="yp-jog-key" data-axis="1" data-dir="1" id="yp-jkey-1-pos">
                    <span class="yp-jkey-sym">L +</span>
                  </button>
                </div>

                <!-- Pair 3: U / Z -->
                <div class="yp-jog-pair">
                  <button class="yp-jog-key" data-axis="2" data-dir="-1" id="yp-jkey-2-neg">
                    <span class="yp-jkey-sym">U -</span>
                  </button>
                  <button class="yp-jog-key" data-axis="2" data-dir="1" id="yp-jkey-2-pos">
                    <span class="yp-jkey-sym">U +</span>
                  </button>
                </div>

                <!-- Pair 4: R / Rx -->
                <div class="yp-jog-pair">
                  <button class="yp-jog-key" data-axis="3" data-dir="-1" id="yp-jkey-3-neg">
                    <span class="yp-jkey-sym">R -</span>
                  </button>
                  <button class="yp-jog-key" data-axis="3" data-dir="1" id="yp-jkey-3-pos">
                    <span class="yp-jkey-sym">R +</span>
                  </button>
                </div>

                <!-- Pair 5: B / Ry -->
                <div class="yp-jog-pair">
                  <button class="yp-jog-key" data-axis="4" data-dir="-1" id="yp-jkey-4-neg">
                    <span class="yp-jkey-sym">B -</span>
                  </button>
                  <button class="yp-jog-key" data-axis="4" data-dir="1" id="yp-jkey-4-pos">
                    <span class="yp-jkey-sym">B +</span>
                  </button>
                </div>

                <!-- Pair 6: T / Rz -->
                <div class="yp-jog-pair">
                  <button class="yp-jog-key" data-axis="5" data-dir="-1" id="yp-jkey-5-neg">
                    <span class="yp-jkey-sym">T -</span>
                  </button>
                  <button class="yp-jog-key" data-axis="5" data-dir="1" id="yp-jkey-5-pos">
                    <span class="yp-jkey-sym">T +</span>
                  </button>
                </div>
              </div>

              <!-- Speed Controls: FAST, SLOW, HIGH SPEED -->
              <div class="yp-speed-row">
                <button class="yp-hardware-key btn-speed" id="yp-key-fast" title="Tăng cấp tốc độ manual">
                  ▲ FAST
                </button>
                <button class="yp-hardware-key btn-speed" id="yp-key-slow" title="Giảm cấp tốc độ manual">
                  ▼ SLOW
                </button>
                <button class="yp-hardware-key btn-speed-high" id="yp-key-high-speed" title="Nhấn giữ để tăng tốc tối đa tức thời">
                  ⚡ HIGH SPEED
                </button>
                <div class="yp-cur-speed-pill" id="yp-cur-speed-pill">MED (25%)</div>
              </div>
            </div>

            <!-- Right Deck: INFORM Programming & Testing Operation Keys -->
            <div class="yp-command-panel">
              <div class="yp-cmd-header">
                <span class="yp-sub-title">PHÍM LẬP TRÌNH & THAO TÁC LỆNH</span>
              </div>

              <div class="yp-cmd-grid">
                <button class="yp-hardware-key btn-cmd-primary" id="yp-key-inform-list" title="Mở danh sách tập lệnh INFORM">
                  INFORM LIST
                </button>
                <button class="yp-hardware-key btn-cmd-action" id="yp-key-modify" title="[MODIFY] Chỉnh sửa câu lệnh / Dạy điểm">
                  MODIFY
                </button>
                <button class="yp-hardware-key btn-cmd-action" id="yp-key-insert" title="[INSERT] Chèn câu lệnh từ BUFFER">
                  INSERT
                </button>
                <button class="yp-hardware-key btn-cmd-action" id="yp-key-delete" title="[DELETE] Xóa câu lệnh tại con trỏ">
                  DELETE
                </button>

                <button class="yp-hardware-key btn-cmd-enter" id="yp-key-enter" title="[ENTER] Xác nhận ghi nội dung / Lưu điểm dạy">
                  ENTER ↵
                </button>
                <button class="yp-hardware-key btn-cmd-sec" id="yp-key-select" title="[SELECT] Chọn một nội dung hoặc thông số">
                  SELECT
                </button>
                <button class="yp-hardware-key btn-cmd-sec" id="yp-key-cancel" title="[CANCEL] Hủy bỏ thao tác">
                  CANCEL
                </button>
                <button class="yp-hardware-key btn-cmd-sec" id="yp-key-area" title="[AREA] Chuyển đổi vùng làm việc">
                  AREA
                </button>

                <button class="yp-hardware-key btn-interlock" id="yp-key-interlock" title="[INTERLOCK] Khóa kết hợp">
                  INTERLOCK
                </button>
                <button class="yp-hardware-key btn-test-start" id="yp-key-test-start" title="[TEST START] Giữ cùng INTERLOCK để chạy thử trong TEACH">
                  TEST START
                </button>
                <button class="yp-hardware-key btn-motion-fwd" id="yp-key-fwd" title="[FWD] Robot di chuyển từng bước tới điểm kế tiếp">
                  FWD ▶
                </button>
                <button class="yp-hardware-key btn-motion-bwd" id="yp-key-bwd" title="[BWD] Robot di chuyển về điểm trước đó">
                  ◀ BWD
                </button>
              </div>

              <!-- D-Pad & Numeric Row -->
              <div class="yp-dpad-numeric-row">
                <div class="yp-dpad">
                  <button class="yp-dpad-btn up" id="yp-dpad-up">▲</button>
                  <button class="yp-dpad-btn left" id="yp-dpad-left">◀</button>
                  <button class="yp-dpad-btn down" id="yp-dpad-down">▼</button>
                  <button class="yp-dpad-btn right" id="yp-dpad-right">▶</button>
                </div>
                <div class="yp-numeric-quick">
                  <button class="yp-num-key" data-val="0">0</button>
                  <button class="yp-num-key" data-val="1">1</button>
                  <button class="yp-num-key" data-val="2">2</button>
                  <button class="yp-num-key" data-val="3">3</button>
                  <button class="yp-num-key" data-val="4">4</button>
                  <button class="yp-num-key" data-val="5">5</button>
                  <button class="yp-num-key" data-val=".">.</button>
                  <button class="yp-num-key" data-val="-">-</button>
                </div>
              </div>

            </div>

          </div>

        </div>

        <!-- INFORM COMMAND SELECTION MODAL -->
        <div class="yp-inform-modal" id="yp-inform-modal" style="display: none;">
          <div class="yp-inform-modal-content">
            <div class="yp-inform-modal-header">
              <span>TẬP LỆNH INFORM III // ROBOT MOTOMINI</span>
              <button class="yp-modal-close" id="yp-btn-close-inform-modal">✕</button>
            </div>
            
            <div class="yp-inform-tabs">
              <button class="yp-itab-btn active" data-cat="MOTION">MOTION (Di chuyển)</button>
              <button class="yp-itab-btn" data-cat="INOUT">IN/OUT (Đọc ghi I/O)</button>
              <button class="yp-itab-btn" data-cat="CONTROL">CONTROL (Cấu trúc lệnh)</button>
            </div>

            <div class="yp-inform-list-body" id="yp-inform-list-body">
              <!-- Rendered by JS -->
            </div>
          </div>
        </div>

      </div>
    `;

    if (this.dockMode === 'docked' && this.container) {
      this.container.innerHTML = html;
    } else {
      this.floatingHost.innerHTML = html;
    }
  }

  /**
   * Binds all hardware buttons and interactions
   */
  bindEvents() {
    // 1. Draggable Window Handle
    const handle = document.getElementById('yp-drag-handle');
    const floatingHost = this.floatingHost;

    if (handle && floatingHost) {
      const onMouseDown = (e) => {
        if (e.target.closest('.yp-window-controls')) return;
        if (this.dockMode === 'docked') return;

        this.isDragging = true;
        const rect = floatingHost.getBoundingClientRect();
        this.dragOffset.x = e.clientX - rect.left;
        this.dragOffset.y = e.clientY - rect.top;

        document.addEventListener('mousemove', onMouseMove);
        document.addEventListener('mouseup', onMouseUp);
      };

      const onMouseMove = (e) => {
        if (!this.isDragging) return;
        const x = Math.max(10, Math.min(window.innerWidth - 320, e.clientX - this.dragOffset.x));
        const y = Math.max(10, Math.min(window.innerHeight - 150, e.clientY - this.dragOffset.y));
        floatingHost.style.left = `${x}px`;
        floatingHost.style.top = `${y}px`;
        floatingHost.style.right = 'auto';
        this.windowPos.x = x;
        this.windowPos.y = y;
      };

      const onMouseUp = () => {
        this.isDragging = false;
        document.removeEventListener('mousemove', onMouseMove);
        document.removeEventListener('mouseup', onMouseUp);
      };

      handle.addEventListener('mousedown', onMouseDown);
    }

    // 2. Window Controls
    const btnDock = document.getElementById('yp-btn-toggle-dock');
    if (btnDock) {
      btnDock.addEventListener('click', () => {
        if (this.dockMode === 'docked') {
          const mount = document.getElementById('dock-teach-pendant-mount');
          if (mount && this.container === mount) {
            mount.style.display = 'none';
          }
          if (this.app && this.app.activeWorkstation === 'pendant') {
            this.app.switchWorkstation('default');
          }
          this.setDockMode('floating', true);
          this.show();
        } else {
          if (this.app) {
            this.app.switchWorkstation('pendant');
          } else {
            this.setDockMode('docked');
          }
        }
      });
    }

    const btnMin = document.getElementById('yp-btn-minimize');
    const bodyEl = document.getElementById('yp-pendant-body');
    if (btnMin && bodyEl) {
      btnMin.addEventListener('click', () => {
        this.isMinimized = !this.isMinimized;
        bodyEl.style.display = this.isMinimized ? 'none' : 'flex';
        btnMin.textContent = this.isMinimized ? '🗖' : '➖';
      });
    }

    const btnClose = document.getElementById('yp-btn-close');
    if (btnClose) {
      btnClose.addEventListener('click', () => {
        this.hide();
      });
    }

    // 3. Rotary Mode Switch
    const rotaryEl = document.getElementById('yp-mode-rotary');
    if (rotaryEl) {
      rotaryEl.addEventListener('click', (e) => {
        const targetMode = e.target.closest('.yp-mode-lbl');
        if (targetMode) {
          this.setRobotMode(parseInt(targetMode.dataset.mode, 10));
        } else {
          this.setRobotMode((this.robotMode + 1) % 3);
        }
      });
    }

    // 4. Deadman Switch
    document.querySelectorAll('.yp-dm-btn').forEach(btn => {
      btn.addEventListener('click', () => {
        const state = parseInt(btn.dataset.dm, 10);
        this.setDeadmanState(state);
      });
    });

    // 5. Emergency Stop
    const btnEstop = document.getElementById('yp-btn-estop');
    if (btnEstop) {
      btnEstop.addEventListener('click', () => {
        this.toggleEmergencyStop();
      });
    }

    // 6. START & HOLD
    const btnStart = document.getElementById('yp-btn-start');
    if (btnStart) {
      btnStart.addEventListener('click', () => {
        this.onStartPressed();
      });
    }

    const btnHold = document.getElementById('yp-btn-hold');
    if (btnHold) {
      btnHold.addEventListener('click', () => {
        this.onHoldPressed();
      });
    }

    // 7. SERVO ON READY
    const btnServoReady = document.getElementById('yp-btn-servo-ready');
    if (btnServoReady) {
      btnServoReady.addEventListener('click', () => {
        this.onServoReadyPressed();
      });
    }

    // 8. Main Menu Screen Switch Buttons (JOB, VARIABLE, INOUT, ROBOT, SYSTEM)
    document.querySelectorAll('.yp-menu-btn').forEach(btn => {
      btn.addEventListener('click', () => {
        document.querySelectorAll('.yp-menu-btn').forEach(b => b.classList.remove('active'));
        btn.classList.add('active');
        this.currentScreen = btn.dataset.screen;
        this.updateScreenZone4();
      });
    });

    // 9. [COORD] Key
    const keyCoord = document.getElementById('yp-key-coord');
    if (keyCoord) {
      keyCoord.addEventListener('click', () => {
        const coords = ['JOINT', 'CARTESIAN', 'TOOL'];
        const idx = coords.indexOf(this.currentCoord);
        this.currentCoord = coords[(idx + 1) % coords.length];
        this.updateJogKeyLabels();
        this.updatePendantDisplay();
        this.setHumanMessage(`Hệ tọa độ Jog đã chuyển sang: [${this.currentCoord}]`, 'info');
      });
    }

    // 10. Speed Keys
    const keyFast = document.getElementById('yp-key-fast');
    if (keyFast) {
      keyFast.addEventListener('click', () => {
        const list = ['INCH', 'LOW', 'MED', 'HIGH'];
        const idx = list.indexOf(this.jogSpeedMode);
        if (idx < list.length - 1) {
          this.jogSpeedMode = list[idx + 1];
          this.updatePendantDisplay();
          this.setHumanMessage(`Tốc độ di chuyển Jog tăng lên: ${this.jogSpeeds[this.jogSpeedMode].label}`, 'info');
        }
      });
    }

    const keySlow = document.getElementById('yp-key-slow');
    if (keySlow) {
      keySlow.addEventListener('click', () => {
        const list = ['INCH', 'LOW', 'MED', 'HIGH'];
        const idx = list.indexOf(this.jogSpeedMode);
        if (idx > 0) {
          this.jogSpeedMode = list[idx - 1];
          this.updatePendantDisplay();
          this.setHumanMessage(`Tốc độ di chuyển Jog giảm xuống: ${this.jogSpeeds[this.jogSpeedMode].label}`, 'info');
        }
      });
    }

    const keyHighSpeed = document.getElementById('yp-key-high-speed');
    if (keyHighSpeed) {
      const onHighSpeedDown = (e) => {
        e.preventDefault();
        this.highSpeedActive = true;
        keyHighSpeed.classList.add('active');
      };
      const onHighSpeedUp = (e) => {
        e.preventDefault();
        this.highSpeedActive = false;
        keyHighSpeed.classList.remove('active');
      };
      keyHighSpeed.addEventListener('mousedown', onHighSpeedDown);
      keyHighSpeed.addEventListener('mouseup', onHighSpeedUp);
      keyHighSpeed.addEventListener('mouseleave', onHighSpeedUp);
      keyHighSpeed.addEventListener('touchstart', onHighSpeedDown, { passive: false });
      keyHighSpeed.addEventListener('touchend', onHighSpeedUp, { passive: false });
    }

    // 11. 12 Jog Keys (Continuous Jogging at 20Hz)
    document.querySelectorAll('.yp-jog-key').forEach(btn => {
      const axis = parseInt(btn.dataset.axis, 10);
      const dir = parseFloat(btn.dataset.dir);

      const startJog = (e) => {
        e.preventDefault();
        if (!this.canJog()) return;

        btn.classList.add('active');
        this.performJogStep(axis, dir);

        this.activeJogInterval = setInterval(() => {
          if (!this.canJog()) {
            this.stopJog(btn);
            return;
          }
          this.performJogStep(axis, dir);
        }, 50);
      };

      const stopJogHandler = (e) => {
        e.preventDefault();
        this.stopJog(btn);
      };

      btn.addEventListener('mousedown', startJog);
      btn.addEventListener('mouseup', stopJogHandler);
      btn.addEventListener('mouseleave', stopJogHandler);
      btn.addEventListener('touchstart', startJog, { passive: false });
      btn.addEventListener('touchend', stopJogHandler, { passive: false });
    });

    // 12. Programming Keys
    const keyInformList = document.getElementById('yp-key-inform-list');
    if (keyInformList) {
      keyInformList.addEventListener('click', () => {
        this.openInformListModal();
      });
    }

    const keyModify = document.getElementById('yp-key-modify');
    if (keyModify) {
      keyModify.addEventListener('click', () => {
        this.onModifyPressed();
      });
    }

    const keyInsert = document.getElementById('yp-key-insert');
    if (keyInsert) {
      keyInsert.addEventListener('click', () => {
        this.onInsertPressed();
      });
    }

    const keyDelete = document.getElementById('yp-key-delete');
    if (keyDelete) {
      keyDelete.addEventListener('click', () => {
        this.onDeletePressed();
      });
    }

    const keyEnter = document.getElementById('yp-key-enter');
    if (keyEnter) {
      keyEnter.addEventListener('click', () => {
        this.onEnterPressed();
      });
    }

    const keySelect = document.getElementById('yp-key-select');
    if (keySelect) {
      keySelect.addEventListener('click', () => {
        this.onSelectPressed();
      });
    }

    const keyCancel = document.getElementById('yp-key-cancel');
    if (keyCancel) {
      keyCancel.addEventListener('click', () => {
        this.onCancelPressed();
      });
    }

    // 13. [INTERLOCK] & [TEST START]
    const keyInterlock = document.getElementById('yp-key-interlock');
    if (keyInterlock) {
      keyInterlock.addEventListener('click', () => {
        this.interlockActive = !this.interlockActive;
        keyInterlock.classList.toggle('active', this.interlockActive);
        this.setHumanMessage(`[INTERLOCK] ${this.interlockActive ? 'ĐÃ BẬT' : 'ĐÃ TẮT'}`, 'info');
      });
    }

    const keyTestStart = document.getElementById('yp-key-test-start');
    if (keyTestStart) {
      const onTestDown = (e) => {
        e.preventDefault();
        if (!this.interlockActive) {
          this.setHumanMessage("KHÓA LIÊN ĐỘNG: Cần nhấn giữ [INTERLOCK] + [TEST START] để chạy thử trong TEACH!", "warning");
          return;
        }
        if (this.robotMode !== 0 || !this.servoOn || this.deadmanState !== 1) {
          this.setHumanMessage("KHÓA LIÊN ĐỘNG: Cần Mode=TEACH, Deadman=Light Squeeze và SERVO ON!", "warning");
          return;
        }
        keyTestStart.classList.add('active');
        this.runTestStartINFORM();
      };

      const onTestUp = (e) => {
        e.preventDefault();
        keyTestStart.classList.remove('active');
        this.stopJobExecution();
      };

      keyTestStart.addEventListener('mousedown', onTestDown);
      keyTestStart.addEventListener('mouseup', onTestUp);
      keyTestStart.addEventListener('mouseleave', onTestUp);
      keyTestStart.addEventListener('touchstart', onTestDown, { passive: false });
      keyTestStart.addEventListener('touchend', onTestUp, { passive: false });
    }

    // 14. Motion Step Keys [FWD] & [BWD]
    const keyFwd = document.getElementById('yp-key-fwd');
    if (keyFwd) {
      keyFwd.addEventListener('click', () => {
        this.stepForwardINFORM();
      });
    }

    const keyBwd = document.getElementById('yp-key-bwd');
    if (keyBwd) {
      keyBwd.addEventListener('click', () => {
        this.stepBackwardINFORM();
      });
    }

    // 15. D-Pad & Numeric Keys
    const dpadUp = document.getElementById('yp-dpad-up');
    if (dpadUp) dpadUp.addEventListener('click', () => this.navigateCursor(-1));
    const dpadDown = document.getElementById('yp-dpad-down');
    if (dpadDown) dpadDown.addEventListener('click', () => this.navigateCursor(1));

    document.querySelectorAll('.yp-num-key').forEach(btn => {
      btn.addEventListener('click', () => {
        const val = btn.dataset.val;
        const input = document.getElementById('yp-buffer-input');
        if (input) {
          input.value += val;
          this.bufferLineText = input.value;
        }
      });
    });

    // 16. Buffer line apply button
    const btnBuffer = document.getElementById('yp-btn-exec-buffer');
    const inputBuffer = document.getElementById('yp-buffer-input');
    if (btnBuffer && inputBuffer) {
      btnBuffer.addEventListener('click', () => {
        this.bufferLineText = inputBuffer.value.trim();
        this.setHumanMessage(`Đã cập nhật dòng đệm: ${this.bufferLineText}`, 'info');
      });
      inputBuffer.addEventListener('change', () => {
        this.bufferLineText = inputBuffer.value.trim();
      });
    }

    // 17. Close INFORM Modal
    const btnCloseModal = document.getElementById('yp-btn-close-inform-modal');
    if (btnCloseModal) {
      btnCloseModal.addEventListener('click', () => {
        const modal = document.getElementById('yp-inform-modal');
        if (modal) modal.style.display = 'none';
      });
    }

    // 18. Dropdown menus
    const menuClear = document.getElementById('yp-mi-clear-data');
    if (menuClear) {
      menuClear.addEventListener('click', () => {
        const v = this.posVariables[this.selectedVarIndex];
        if (v) {
          v.x = 0; v.y = 0; v.z = 0; v.rx = 0; v.ry = 0; v.rz = 0;
          v.s = 0; v.l = 0; v.u = 0; v.r = 0; v.b = 0; v.t = 0;
          this.updateScreenZone4();
          this.setHumanMessage(`Đã xóa dữ liệu biến ${v.id}.`, 'info');
        }
      });
    }

    const menuSave = document.getElementById('yp-mi-save-job');
    if (menuSave) {
      menuSave.addEventListener('click', () => {
        this.setHumanMessage(`Đã lưu thành công Project [${this.currentJobName}] vào bộ nhớ.`, 'success');
      });
    }

    const menuReload = document.getElementById('yp-mi-reload-job');
    if (menuReload) {
      menuReload.addEventListener('click', () => {
        this.currentJobName = 'LAB1_PICK_A1_PLACE_B';
        this.updateScreenZone4();
        this.setHumanMessage("Đã nạp lại chương trình mẫu Bài 1 (LAB1_PICK_A1_PLACE_B).", 'info');
      });
    }

    const menuHelp = document.getElementById('yp-mi-help-lab1');
    if (menuHelp) {
      menuHelp.addEventListener('click', () => {
        alert("BÀI THÍ NGHIỆM 1 - ĐHBK TPHCM:\n1. Chọn Mode = TEACH, bóp Deadman Switch ở nấc giữa (Light Squeeze).\n2. Nhấn [SERVO ON READY] để bật Servo (đèn xanh sáng).\n3. Nhấn [COORD] để chuyển sang JOINT hoặc CARTESIAN.\n4. Giữ phím Jog để đưa robot tới điểm cần dạy.\n5. Vào tab VARIABLE, chọn P00x, bấm [TEACH TỌA ĐỘ HIỆN TẠI] hoặc nhập số thủ công.\n6. Vào tab JOB, chọn Project cần chạy hoặc tạo mới.\n7. Gạt sang PLAY, nhấn [START] để chạy tự động.");
      });
    }
  }

  stopJog(btn) {
    if (btn) btn.classList.remove('active');
    if (this.activeJogInterval) {
      clearInterval(this.activeJogInterval);
      this.activeJogInterval = null;
    }
  }

  // ===========================================================================
  // INTERLOCK & SAFETY VALIDATION
  // ===========================================================================

  canJog() {
    if (this.eStop) {
      this.setHumanMessage("CẢNH BÁO: ĐANG DỪNG KHẨN CẤP (EMERGENCY STOP)! Xoay nút nấm sang phải để thoát.", "danger");
      return false;
    }
    if (this.robotMode !== 0) { // 0 = TEACH
      this.setHumanMessage("KHÓA LIÊN ĐỘNG: Cần gạt Mode Switch sang [TEACH] để điều khiển bằng tay!", "warning");
      return false;
    }
    if (this.deadmanState !== 1) { // 1 = Light Squeeze
      this.setHumanMessage("KHÓA LIÊN ĐỘNG: Cần bóp Enable/Deadman Switch ở nấc giữa [LIGHT SQUEEZE (ON)]!", "warning");
      return false;
    }
    if (!this.servoOn) {
      this.setHumanMessage("KHÓA LIÊN ĐỘNG: Nguồn động cơ Servo chưa bật! Nhấn [SERVO ON READY].", "warning");
      return false;
    }
    return true;
  }

  setRobotMode(mode) {
    this.robotMode = mode;
    if (this.hardwareBus) {
      this.hardwareBus.setRobotMode(mode);
    }

    if (this.robotMode === 1 && this.servoReady) {
      this.servoOn = true;
    }

    if (this.robotMode !== 0 && this.activeJogInterval) {
      clearInterval(this.activeJogInterval);
      this.activeJogInterval = null;
    }

    this.updatePendantDisplay();
    const modeNames = ['TEACH (Dạy học bằng tay)', 'PLAY (Chạy tự động)', 'REMOTE (Vận hành từ xa)'];
    this.setHumanMessage(`Chế độ vận hành: ${modeNames[mode]}`, 'info');
  }

  setDeadmanState(state) {
    this.deadmanState = state;

    if (this.robotMode === 0) {
      if (this.deadmanState !== 1) {
        this.servoOn = false;
        if (this.activeJogInterval) {
          clearInterval(this.activeJogInterval);
          this.activeJogInterval = null;
        }
      } else if (this.servoReady) {
        this.servoOn = true;
      }
    }

    this.updatePendantDisplay();
    const dmNames = ['RELEASED (OFF)', 'LIGHT SQUEEZE (ON - SAFE)', 'TIGHT SQUEEZE (OFF - PANIC)'];
    const msgType = (state === 1) ? 'success' : 'warning';
    this.setHumanMessage(`Deadman Switch: ${dmNames[state]}`, msgType);
  }

  toggleEmergencyStop() {
    this.eStop = !this.eStop;
    if (this.eStop) {
      this.servoOn = false;
      this.servoReady = false;
      this.stopJobExecution();
      this.setHumanMessage("DỪNG KHẨN CẤP (EMERGENCY STOP)! Đã ngắt toàn bộ nguồn Servo.", "danger");
    } else {
      this.setHumanMessage("Đã reset nút dừng khẩn cấp. Nhấn [SERVO ON READY] để tái lập nguồn.", "info");
    }
    this.updatePendantDisplay();
  }

  onServoReadyPressed() {
    if (this.eStop) {
      this.setHumanMessage("Không thể bật Servo: Nút dừng khẩn cấp đang bị nhấn!", "danger");
      return;
    }

    if (this.robotMode === 0) {
      if (this.deadmanState !== 1) {
        this.servoReady = true;
        this.servoOn = false;
        this.setHumanMessage("SẴN SÀNG BẬT SERVO: Hãy bóp nhẹ Enable Switch vào nấc giữa để đóng nguồn động lực!", "warning");
      } else {
        this.servoReady = true;
        this.servoOn = true;
        this.setHumanMessage("SERVO ĐÃ ĐƯỢC BẬT (SERVO ON). Sẵn sàng điều khiển tay.", "success");
      }
    } else {
      this.servoReady = true;
      this.servoOn = !this.servoOn;
      this.setHumanMessage(`SERVO: ${this.servoOn ? 'ON' : 'OFF'}`, this.servoOn ? 'success' : 'info');
    }

    this.updatePendantDisplay();
  }

  onStartPressed() {
    if (this.eStop) {
      this.setHumanMessage("Không thể chạy: Nút Dừng khẩn cấp đang kích hoạt!", "danger");
      return;
    }
    if (this.robotMode !== 1) {
      this.setHumanMessage("LƯU Ý: Phím [START] chỉ chạy tự động trong chế độ PLAY! Hãy gạt chìa khóa sang PLAY.", "warning");
      return;
    }
    if (!this.servoOn) {
      this.setHumanMessage("Không thể chạy: Nguồn Servo đang ngắt. Nhấn [SERVO ON READY].", "warning");
      return;
    }

    this.startLit = true;
    this.holdLit = false;
    this.runPlayStartINFORM();
  }

  onHoldPressed() {
    this.holdLit = true;
    this.startLit = false;
    this.stopJobExecution();
    this.setHumanMessage("[HOLD] Robot đã tạm dừng tại vị trí hiện tại.", "warning");
    this.updatePendantDisplay();
  }

  // ===========================================================================
  // JOGGING LOGIC (JOINT & CARTESIAN 6-DOF)
  // ===========================================================================

  performJogStep(axis, dir) {
    if (!this.robotArm || !this.kinematics) return;

    const baseSpd = this.jogSpeeds[this.jogSpeedMode];
    const multiplier = this.highSpeedActive ? 2.5 : 1.0;

    if (this.currentCoord === 'JOINT') {
      const curQ = this.robotArm.getJointAngles();
      const keys = ['s', 'l', 'u', 'r', 'b', 't'];
      const targetQ = { ...curQ };
      const stepDeg = (baseSpd.deg * 0.4) * multiplier;

      targetQ[keys[axis]] += dir * stepDeg;

      const lim = this.robotArm.jointLimits[keys[axis]];
      if (lim) {
        targetQ[keys[axis]] = Math.max(lim.min, Math.min(lim.max, targetQ[keys[axis]]));
      }

      this.robotArm.setJointAngles(targetQ.s, targetQ.l, targetQ.u, targetQ.r, targetQ.b, targetQ.t);
      this.app.updateArmUIFromModel();
    } else {
      const curPose = this.kinematics.forwardKinematics(this.robotArm.getJointAngles());
      const targetPose = {
        x: curPose.tcp_m.x,
        y: curPose.tcp_m.y,
        z: curPose.tcp_m.z,
        rx: curPose.orientation_deg.rx,
        ry: curPose.orientation_deg.ry,
        rz: curPose.orientation_deg.rz
      };

      const stepM = ((baseSpd.mm * 0.001) * 0.3) * multiplier;
      const stepDeg = (baseSpd.deg * 0.3) * multiplier;

      if (axis === 0) targetPose.x += dir * stepM;
      if (axis === 1) targetPose.y += dir * stepM;
      if (axis === 2) targetPose.z += dir * stepM;
      if (axis === 3) targetPose.rx += dir * stepDeg;
      if (axis === 4) targetPose.ry += dir * stepDeg;
      if (axis === 5) targetPose.rz += dir * stepDeg;

      const ik = this.kinematics.inverseKinematics(targetPose, this.robotArm.getJointAngles(), true);
      if (ik.success) {
        const q = ik.jointsDeg;
        this.robotArm.setJointAngles(q.s, q.l, q.u, q.r, q.b, q.t);
        this.app.updateArmUIFromModel();
      }
    }

    this.updateLiveReadouts();
  }

  // ===========================================================================
  // TEACHING & VARIABLE MANAGEMENT (P000..P014)
  // ===========================================================================

  onModifyPressed() {
    if (this.currentScreen === 'VARIABLE') {
      this.pendingAction = 'TEACH_VAR';
      const v = this.posVariables[this.selectedVarIndex];
      this.setHumanMessage(`[MODIFY] Chuẩn bị bắt tọa độ 3D hiện tại vào biến ${v.id}. Nhấn [ENTER] để xác nhận.`, 'warning');
    } else if (this.currentScreen === 'JOB') {
      this.pendingAction = 'MODIFY_LINE';
      this.setHumanMessage(`[MODIFY] Chuẩn bị thay dòng ${this.currentLineIndex} bằng câu lệnh BUFFER. Nhấn [ENTER] để xác nhận.`, 'warning');
    }
  }

  onInsertPressed() {
    if (this.currentScreen === 'JOB') {
      this.pendingAction = 'INSERT_LINE';
      this.setHumanMessage(`[INSERT] Chuẩn bị chèn câu lệnh BUFFER vào sau dòng ${this.currentLineIndex}. Nhấn [ENTER] để chèn.`, 'info');
    }
  }

  onDeletePressed() {
    if (this.currentScreen === 'JOB') {
      this.pendingAction = 'DELETE_LINE';
      this.setHumanMessage(`[DELETE] Chuẩn bị xóa dòng ${this.currentLineIndex}. Nhấn [ENTER] để xóa.`, 'danger');
    }
  }

  onEnterPressed() {
    const instructions = this.getCurrentInstructions();

    if (this.pendingAction === 'TEACH_VAR') {
      this.teachCurrentPointToVariable();
      this.pendingAction = null;
    } else if (this.pendingAction === 'MODIFY_LINE') {
      instructions[this.currentLineIndex] = this.bufferLineText;
      this.pendingAction = null;
      this.updateScreenZone4();
      this.setHumanMessage(`Đã sửa câu lệnh dòng ${this.currentLineIndex}: ${this.bufferLineText}`, 'success');
    } else if (this.pendingAction === 'INSERT_LINE') {
      instructions.splice(this.currentLineIndex + 1, 0, this.bufferLineText);
      this.currentLineIndex++;
      this.pendingAction = null;
      this.updateScreenZone4();
      this.setHumanMessage(`Đã chèn câu lệnh mới vào dòng ${this.currentLineIndex}: ${this.bufferLineText}`, 'success');
    } else if (this.pendingAction === 'DELETE_LINE') {
      if (instructions.length > 1) {
        instructions.splice(this.currentLineIndex, 1);
        if (this.currentLineIndex >= instructions.length) {
          this.currentLineIndex = instructions.length - 1;
        }
        this.pendingAction = null;
        this.updateScreenZone4();
        this.setHumanMessage(`Đã xóa câu lệnh. Con trỏ hiện tại: ${this.currentLineIndex}`, 'info');
      }
    } else {
      if (this.currentScreen === 'VARIABLE') {
        this.teachCurrentPointToVariable();
      }
    }
  }

  onSelectPressed() {
    const instructions = this.getCurrentInstructions();
    if (this.currentScreen === 'JOB') {
      const line = instructions[this.currentLineIndex];
      if (line) {
        this.bufferLineText = line;
        const input = document.getElementById('yp-buffer-input');
        if (input) input.value = line;
        this.setHumanMessage(`Đã chọn lệnh vào BUFFER: ${line}`, 'info');
      }
    } else if (this.currentScreen === 'VARIABLE') {
      this.posVarFormat = (this.posVarFormat === 'ROBOT') ? 'PULSE' : 'ROBOT';
      this.updateScreenZone4();
      this.setHumanMessage(`Định dạng hiển thị: [${this.posVarFormat}]`, 'info');
    }
  }

  onCancelPressed() {
    this.pendingAction = null;
    const modal = document.getElementById('yp-inform-modal');
    if (modal) modal.style.display = 'none';
    this.setHumanMessage("Đã hủy thao tác.", 'info');
  }

  teachCurrentPointToVariable() {
    if (!this.robotArm || !this.kinematics) return;

    const curQ = this.robotArm.getJointAngles();
    const fk = this.kinematics.forwardKinematics(curQ);

    const v = this.posVariables[this.selectedVarIndex];
    v.x = fk.tcp_m.x;
    v.y = fk.tcp_m.y;
    v.z = fk.tcp_m.z;
    v.rx = fk.orientation_deg.rx;
    v.ry = fk.orientation_deg.ry;
    v.rz = fk.orientation_deg.rz;
    v.s = curQ.s;
    v.l = curQ.l;
    v.u = curQ.u;
    v.r = curQ.r;
    v.b = curQ.b;
    v.t = curQ.t;

    this.updateScreenZone4();
    this.setHumanMessage(`[DẠY ĐIỂM THÀNH CÔNG] Đã lưu tọa độ thực tế vào biến ${v.id} (${v.name}): X=${(v.x * 1000).toFixed(1)}, Y=${(v.y * 1000).toFixed(1)}, Z=${(v.z * 1000).toFixed(1)} mm`, 'success');
  }

  saveManualCoordinates(idx, newCoords) {
    const v = this.posVariables[idx];
    if (!v) return;

    if (newCoords.x !== undefined) v.x = newCoords.x / 1000.0;
    if (newCoords.y !== undefined) v.y = newCoords.y / 1000.0;
    if (newCoords.z !== undefined) v.z = newCoords.z / 1000.0;
    if (newCoords.rx !== undefined) v.rx = newCoords.rx;
    if (newCoords.ry !== undefined) v.ry = newCoords.ry;
    if (newCoords.rz !== undefined) v.rz = newCoords.rz;

    // Calculate IK to sync joint angles
    const targetPose = { x: v.x, y: v.y, z: v.z, rx: v.rx, ry: v.ry, rz: v.rz };
    const ik = this.kinematics.inverseKinematics(targetPose, this.robotArm.getJointAngles(), true);
    if (ik.success) {
      v.s = ik.jointsDeg.s;
      v.l = ik.jointsDeg.l;
      v.u = ik.jointsDeg.u;
      v.r = ik.jointsDeg.r;
      v.b = ik.jointsDeg.b;
      v.t = ik.jointsDeg.t;
    }

    this.updateScreenZone4();
    this.setHumanMessage(`[CẬP NHẬT TỌA ĐỘ THỦ CÔNG] Đã cập nhật tọa độ mới cho biến ${v.id} (${v.name}).`, 'success');
  }

  // ===========================================================================
  // SCREEN ZONE 4 RENDERER (JOB / VARIABLE / INOUT / ROBOT / SYSTEM)
  // ===========================================================================

  updateScreenZone4() {
    const container = document.getElementById('yp-general-display');
    if (!container) return;

    if (this.currentScreen === 'JOB') {
      this.renderJobScreen(container);
    } else if (this.currentScreen === 'VARIABLE') {
      this.renderVariableScreen(container);
    } else if (this.currentScreen === 'INOUT') {
      this.renderInOutScreen(container);
    } else if (this.currentScreen === 'ROBOT') {
      this.renderRobotScreen(container);
    } else if (this.currentScreen === 'SYSTEM') {
      this.renderSystemScreen(container);
    }
  }

  renderJobScreen(container) {
    const instructions = this.getCurrentInstructions();
    const proj = this.projects[this.currentJobName];

    let html = `
      <div class="yp-job-wrapper">
        <!-- Job Subtabs Bar -->
        <div class="yp-job-subtabs-nav">
          <button class="yp-subtab-btn ${this.jobSubtab === 'CONTENT' ? 'active' : ''}" id="yp-subtab-content">
            📝 1. SOẠN THẢO (JOB CONTENT)
          </button>
          <button class="yp-subtab-btn ${this.jobSubtab === 'SELECT' ? 'active' : ''}" id="yp-subtab-select">
            📁 2. CHỌN PROJECT ĐỂ CHẠY (${Object.keys(this.projects).length})
          </button>
          <button class="yp-subtab-btn ${this.jobSubtab === 'CREATE' ? 'active' : ''}" id="yp-subtab-create">
            ➕ 3. TẠO PROJECT MỚI
          </button>
        </div>
    `;

    if (this.jobSubtab === 'CONTENT') {
      let rowsHtml = '';
      instructions.forEach((code, idx) => {
        const isSelected = (idx === this.currentLineIndex);
        const isRunning = (this.isRunningJob && idx === this.currentLineIndex);
        const addr = String(idx).padStart(4, '0');

        rowsHtml += `
          <tr class="yp-job-row ${isSelected ? 'selected' : ''} ${isRunning ? 'running' : ''}" data-line="${idx}">
            <td class="col-addr">${addr}</td>
            <td class="col-code">${code}</td>
          </tr>
        `;
      });

      html += `
        <div class="yp-job-editor-view">
          <div class="yp-view-header">
            <div class="yp-view-title">
              <span style="color: #38bdf8; font-weight: 800; font-size: 13px;">PROJECT: ${this.currentJobName}</span>
              <span class="yp-group-badge">R1 // TOOL: 00 // ${instructions.length} DÒNG LỆNH</span>
            </div>
            <div class="yp-job-actions">
              <button class="yp-action-chip btn-primary" id="yp-btn-open-inform-list">+ THÊM LỆNH</button>
              <button class="yp-action-chip" id="yp-btn-quick-run-t" style="background: linear-gradient(135deg, #0d9488, #059669); border-color: #2dd4bf; color: #fff; font-weight: 700;" title="Chạy tự động vẽ chữ T (Hiển thị đồ thị chữ T)">✍️ VẼ CHỮ T</button>
              <button class="yp-action-chip" id="yp-btn-fwd-step">FWD ▶</button>
              <button class="yp-action-chip" id="yp-btn-bwd-step">◀ BWD</button>
            </div>
          </div>

          <div class="yp-job-table-scroll">
            <table class="yp-job-table">
              <thead>
                <tr>
                  <th style="width: 75px;">ĐỊA CHỈ</th>
                  <th>CÂU LỆNH INFORM III (CLICK ĐỂ CHỌN CON TRỎ)</th>
                </tr>
              </thead>
              <tbody>
                ${rowsHtml}
              </tbody>
            </table>
          </div>

          <div class="yp-job-desc-footer">
            <span>ℹ️ <b>Mô tả:</b> ${proj ? proj.desc : ''}</span>
          </div>
        </div>
      `;
    } else if (this.jobSubtab === 'SELECT') {
      // List of Projects Screen
      let projRows = '';
      Object.keys(this.projects).forEach(pKey => {
        const p = this.projects[pKey];
        const isActive = (pKey === this.currentJobName);
        const isLetterT = (pKey === 'AUTO_DRAW_LETTER_T');
        projRows += `
          <div class="yp-proj-card ${isActive ? 'active-proj' : ''} ${isLetterT ? 'letter-t-proj' : ''}" data-proj="${pKey}" style="${isLetterT ? 'border: 1px solid #2dd4bf; background: rgba(13, 148, 136, 0.12);' : ''}">
            <div class="yp-proj-card-header">
              <div class="yp-proj-card-name">
                <span class="yp-folder-icon">${isLetterT ? '✍️' : '📁'}</span>
                <b>${p.name}</b>
                ${isLetterT ? '<span class="yp-badge-active" style="background:rgba(45,212,191,0.25); color:#2dd4bf; border:1px solid #2dd4bf; margin-left:6px;">★ ĐỒ THỊ CHỮ T</span>' : ''}
                ${isActive ? '<span class="yp-badge-active">ĐANG CHỌN ĐỂ CHẠY PLAY</span>' : ''}
              </div>
              <div style="display:flex; gap:6px;">
                ${isLetterT ? `
                  <button class="yp-action-chip yp-btn-auto-run-t" data-proj="${pKey}" style="background: linear-gradient(135deg, #0d9488, #059669); border-color: #2dd4bf; color: #fff; font-weight: 800;" title="Chạy tự động vẽ chữ T ngay lập tức">
                    ⚡ CHẠY TỰ ĐỘNG CHỮ T
                  </button>
                ` : ''}
                <button class="yp-action-chip ${isActive ? 'btn-success' : 'btn-primary'} yp-btn-pick-proj" data-proj="${pKey}">
                  ${isActive ? '✓ ĐANG KÍCH HOẠT' : '▶ CHỌN PROJECT NÀY'}
                </button>
              </div>
            </div>
            <div class="yp-proj-card-desc">${p.desc}</div>
            <div class="yp-proj-card-meta">
              <span>Số câu lệnh: <b>${p.instructions.length} dòng</b></span>
              <span>Robot: <b>Yaskawa MotoMINI (6-DOF)</b></span>
            </div>
          </div>
        `;
      });

      html += `
        <div class="yp-proj-select-view">
          <div class="yp-view-header">
            <span style="font-size: 13px; font-weight: 800; color: #4ade80;">CHỌN PROJECT ĐỂ THỰC THI (PLAY MODE)</span>
            <span class="yp-group-badge">TỔNG CỘNG: ${Object.keys(this.projects).length} CHƯƠNG TRÌNH</span>
          </div>

          <div class="yp-proj-list-scroll">
            ${projRows}
          </div>
        </div>
      `;
    } else if (this.jobSubtab === 'CREATE') {
      // Create New Project Screen
      html += `
        <div class="yp-proj-create-view">
          <div class="yp-view-header">
            <span style="font-size: 13px; font-weight: 800; color: #facc15;">TẠO PROJECT CHƯƠNG TRÌNH MỚI (NEW JOB CREATE)</span>
          </div>

          <div class="yp-create-form">
            <div class="yp-form-row">
              <label>TÊN PROJECT (JOB NAME):</label>
              <input type="text" id="yp-input-new-job-name" value="MY_ROBOT_MOTION_01" class="yp-input-text">
            </div>

            <div class="yp-form-row">
              <label>MÔ TẢ MỤC ĐÍCH:</label>
              <input type="text" id="yp-input-new-job-desc" value="Thử nghiệm nội suy chuyển động đường thẳng MOVL và gắp nhả" class="yp-input-text">
            </div>

            <div class="yp-form-row">
              <label>MẪU KHỞI TẠO (TEMPLATE):</label>
              <select id="yp-select-new-job-template" class="yp-select-box">
                <option value="BLANK">Chương trình rỗng (Chỉ gồm NOP và END)</option>
                <option value="LAB1_PICK_PLACE">Mẫu Bài 1: Gắp A1 thả vào B</option>
                <option value="LAB1_STACK">Mẫu Bài 1: Gắp xếp chồng 3 vật</option>
              </select>
            </div>

            <div style="margin-top: 15px; display: flex; gap: 10px;">
              <button class="yp-large-btn btn-success" id="yp-btn-submit-create-job">
                ➕ TẠO VÀ CHUYỂN SANG SOẠN THẢO NGAY
              </button>
            </div>
          </div>
        </div>
      `;
    }

    html += `</div>`;
    container.innerHTML = html;

    // Subtab navigation bindings
    const subContent = document.getElementById('yp-subtab-content');
    if (subContent) subContent.addEventListener('click', () => { this.jobSubtab = 'CONTENT'; this.updateScreenZone4(); });
    const subSelect = document.getElementById('yp-subtab-select');
    if (subSelect) subSelect.addEventListener('click', () => { this.jobSubtab = 'SELECT'; this.updateScreenZone4(); });
    const subCreate = document.getElementById('yp-subtab-create');
    if (subCreate) subCreate.addEventListener('click', () => { this.jobSubtab = 'CREATE'; this.updateScreenZone4(); });

    // Job line selection click
    container.querySelectorAll('.yp-job-row').forEach(row => {
      row.addEventListener('click', () => {
        this.currentLineIndex = parseInt(row.dataset.line, 10);
        this.updateScreenZone4();
        const line = this.getCurrentInstructions()[this.currentLineIndex];
        if (line) {
          this.bufferLineText = line;
          const input = document.getElementById('yp-buffer-input');
          if (input) input.value = line;
        }
      });
    });

    // Pick Project Buttons
    container.querySelectorAll('.yp-btn-pick-proj').forEach(btn => {
      btn.addEventListener('click', (e) => {
        e.stopPropagation();
        const pKey = btn.dataset.proj;
        this.currentJobName = pKey;
        this.currentLineIndex = 0;
        this.jobSubtab = 'CONTENT';
        const hdr = document.getElementById('yp-header-active-proj');
        if (hdr) hdr.textContent = `PROJECT: ${this.currentJobName}`;
        this.updateScreenZone4();
        this.setHumanMessage(`Đã kích hoạt Project: [${this.currentJobName}] để chạy trong chế độ PLAY!`, 'success');
      });
    });

    // Create Job Submit
    const btnCreate = document.getElementById('yp-btn-submit-create-job');
    if (btnCreate) {
      btnCreate.addEventListener('click', () => {
        const nameInput = document.getElementById('yp-input-new-job-name');
        const descInput = document.getElementById('yp-input-new-job-desc');
        const tmplSelect = document.getElementById('yp-select-new-job-template');

        const newName = nameInput ? nameInput.value.trim().toUpperCase().replace(/\s+/g, '_') : 'NEW_JOB';
        const newDesc = descInput ? descInput.value.trim() : 'Project người dùng tạo';
        const tmpl = tmplSelect ? tmplSelect.value : 'BLANK';

        let initLines = ['NOP', 'MOVJ P000 VJ=50.00', 'END'];
        if (tmpl === 'LAB1_PICK_PLACE') {
          initLines = [...this.projects['LAB1_PICK_A1_PLACE_B'].instructions];
        } else if (tmpl === 'LAB1_STACK') {
          initLines = [...this.projects['LAB1_STACK_3_OBJECTS'].instructions];
        }

        this.projects[newName] = {
          name: newName,
          desc: newDesc,
          instructions: initLines
        };

        this.currentJobName = newName;
        this.currentLineIndex = 0;
        this.jobSubtab = 'CONTENT';
        const hdr = document.getElementById('yp-header-active-proj');
        if (hdr) hdr.textContent = `PROJECT: ${this.currentJobName}`;
        this.updateScreenZone4();
        this.setHumanMessage(`Tạo thành công Project mới: [${newName}]!`, 'success');
      });
    }

    // Auto Run Letter T Buttons from Select Projects List
    container.querySelectorAll('.yp-btn-auto-run-t').forEach(btn => {
      btn.addEventListener('click', (e) => {
        e.stopPropagation();
        runAutoLetterTJob(this.app);
      });
    });

    const btnQuickRunT = document.getElementById('yp-btn-quick-run-t');
    if (btnQuickRunT) {
      btnQuickRunT.addEventListener('click', () => {
        runAutoLetterTJob(this.app);
      });
    }

    const btnOpenList = document.getElementById('yp-btn-open-inform-list');
    if (btnOpenList) btnOpenList.addEventListener('click', () => this.openInformListModal());
    const btnFwd = document.getElementById('yp-btn-fwd-step');
    if (btnFwd) btnFwd.addEventListener('click', () => this.stepForwardINFORM());
    const btnBwd = document.getElementById('yp-btn-bwd-step');
    if (btnBwd) btnBwd.addEventListener('click', () => this.stepBackwardINFORM());
  }

  renderVariableScreen(container) {
    const isRobot = (this.posVarFormat === 'ROBOT');
    let rowsHtml = '';

    this.posVariables.forEach((item, idx) => {
      const isSel = (idx === this.selectedVarIndex);
      const val1 = isRobot ? (item.x * 1000).toFixed(0) : item.s.toFixed(0);
      const val2 = isRobot ? (item.y * 1000).toFixed(0) : item.l.toFixed(0);
      const val3 = isRobot ? (item.z * 1000).toFixed(0) : item.u.toFixed(0);

      rowsHtml += `
        <tr class="yp-var-row ${isSel ? 'selected' : ''}" data-idx="${idx}">
          <td style="color: #38bdf8; font-weight: 800; font-size: 13px;">${item.id}</td>
          <td style="font-weight: 700;">${item.name}</td>
          <td>${val1}</td>
          <td>${val2}</td>
          <td>${val3}</td>
        </tr>
      `;
    });

    const v = this.posVariables[this.selectedVarIndex];

    container.innerHTML = `
      <div class="yp-var-editor-view">
        <div class="yp-view-header">
          <div class="yp-view-title">
            <span style="color: #facc15; font-weight: 800; font-size: 13px;">QUẢN LÝ TỌA ĐỘ CÁC ĐIỂM (POSITION VARIABLES P000..P014)</span>
            <span class="yp-group-badge">ĐỊNH DẠNG: ${this.posVarFormat}</span>
          </div>
          <div class="yp-job-actions">
            <button class="yp-action-chip" id="yp-btn-var-toggle-fmt">CHUYỂN: [${isRobot ? 'PULSE' : 'ROBOT'}]</button>
          </div>
        </div>

        <div class="yp-var-split">
          <!-- Variables Table -->
          <div class="yp-var-table-scroll">
            <table class="yp-var-table">
              <thead>
                <tr>
                  <th style="width: 60px;">BIẾN</th>
                  <th>TÊN VỊ TRÍ</th>
                  <th>${isRobot ? 'X (mm)' : 'S (pls)'}</th>
                  <th>${isRobot ? 'Y (mm)' : 'L (pls)'}</th>
                  <th>${isRobot ? 'Z (mm)' : 'U (pls)'}</th>
                </tr>
              </thead>
              <tbody>
                ${rowsHtml}
              </tbody>
            </table>
          </div>

          <!-- Variable Full Setup & Edit Panel -->
          <div class="yp-var-detail-card">
            <div class="yp-card-title">
              <span>ĐANG CHỌN: <b style="color:#38bdf8; font-size:14px;">${v.id}</b> (${v.name})</span>
              <span style="color:#94a3b8; font-size:11px;">TOOL: 00 // ${v.type}</span>
            </div>

            <!-- Teaching Options: Method 1 (Direct Teach) & Method 2 (Manual Input) -->
            <div class="yp-teach-methods-box">
              <div class="yp-method-section">
                <span class="yp-method-label">CÁCH 1: DẠY ĐIỂM TRỰC TIẾP TỪ ROBOT 3D</span>
                <button class="yp-large-btn btn-primary" id="yp-btn-teach-direct" style="width:100%; margin-top:4px;">
                  📸 BẮT TỌA ĐỘ HIỆN TẠI VÀO BIẾN ${v.id} (TEACH)
                </button>
              </div>

              <div class="yp-method-section" style="margin-top: 8px;">
                <span class="yp-method-label">CÁCH 2: NHẬP SỐ TỌA ĐỘ THỦ CÔNG</span>
                <div class="yp-manual-inputs-grid">
                  <div class="yp-input-item">
                    <span>X (mm):</span>
                    <input type="number" step="1" id="yp-inp-x" value="${(v.x * 1000).toFixed(0)}">
                  </div>
                  <div class="yp-input-item">
                    <span>Y (mm):</span>
                    <input type="number" step="1" id="yp-inp-y" value="${(v.y * 1000).toFixed(0)}">
                  </div>
                  <div class="yp-input-item">
                    <span>Z (mm):</span>
                    <input type="number" step="1" id="yp-inp-z" value="${(v.z * 1000).toFixed(0)}">
                  </div>
                  <div class="yp-input-item">
                    <span>Rx (°):</span>
                    <input type="number" step="1" id="yp-inp-rx" value="${v.rx.toFixed(0)}">
                  </div>
                  <div class="yp-input-item">
                    <span>Ry (°):</span>
                    <input type="number" step="1" id="yp-inp-ry" value="${v.ry.toFixed(0)}">
                  </div>
                  <div class="yp-input-item">
                    <span>Rz (°):</span>
                    <input type="number" step="1" id="yp-inp-rz" value="${v.rz.toFixed(0)}">
                  </div>
                </div>
                <button class="yp-action-chip btn-primary" id="yp-btn-save-manual-coords" style="width:100%; margin-top:6px; height:32px;">
                  💾 LƯU TỌA ĐỘ NHẬP TAY
                </button>
              </div>

              <!-- Test Move Button -->
              <div class="yp-method-section" style="margin-top: 8px; border-top:1px dashed #334155; padding-top:6px;">
                <span class="yp-method-label">KIỂM CHỨNG VỊ TRÍ TRÊN 3D:</span>
                <button class="yp-large-btn btn-success" id="yp-btn-goto-var-fwd" style="width:100%; margin-top:4px;">
                  ▶ FWD DI CHUYỂN ROBOT TỚI ĐIỂM NÀY
                </button>
              </div>

            </div>
          </div>
        </div>
      </div>
    `;

    // Bind row selection
    container.querySelectorAll('.yp-var-row').forEach(row => {
      row.addEventListener('click', () => {
        this.selectedVarIndex = parseInt(row.dataset.idx, 10);
        this.updateScreenZone4();
      });
    });

    const btnToggleFmt = document.getElementById('yp-btn-var-toggle-fmt');
    if (btnToggleFmt) {
      btnToggleFmt.addEventListener('click', () => {
        this.posVarFormat = (this.posVarFormat === 'ROBOT') ? 'PULSE' : 'ROBOT';
        this.updateScreenZone4();
      });
    }

    const btnTeachDirect = document.getElementById('yp-btn-teach-direct');
    if (btnTeachDirect) {
      btnTeachDirect.addEventListener('click', () => {
        this.teachCurrentPointToVariable();
      });
    }

    const btnSaveManual = document.getElementById('yp-btn-save-manual-coords');
    if (btnSaveManual) {
      btnSaveManual.addEventListener('click', () => {
        const x = parseFloat(document.getElementById('yp-inp-x').value);
        const y = parseFloat(document.getElementById('yp-inp-y').value);
        const z = parseFloat(document.getElementById('yp-inp-z').value);
        const rx = parseFloat(document.getElementById('yp-inp-rx').value);
        const ry = parseFloat(document.getElementById('yp-inp-ry').value);
        const rz = parseFloat(document.getElementById('yp-inp-rz').value);

        this.saveManualCoordinates(this.selectedVarIndex, { x, y, z, rx, ry, rz });
      });
    }

    const btnGoto = document.getElementById('yp-btn-goto-var-fwd');
    if (btnGoto) {
      btnGoto.addEventListener('click', () => {
        this.moveArmToVariable(this.posVariables[this.selectedVarIndex]);
      });
    }
  }

  renderInOutScreen(container) {
    const ot1 = (this.hardwareBus && this.hardwareBus.robotOT[1] === 1);

    container.innerHTML = `
      <div class="yp-io-view">
        <div class="yp-view-header">
          <span style="color: #4ade80; font-weight: 800; font-size: 13px;">BẢNG ĐIỀU KHIỂN & GIÁM SÁT DIGITAL I/O (BÀI 1 & BÀI 2)</span>
          <span class="yp-group-badge">YASKAWA GENERAL PURPOSE I/O</span>
        </div>

        <div class="yp-io-columns">
          <div class="yp-io-col">
            <span class="yp-io-title">OUTPUTS (OUT#0001..OUT#0008)</span>
            <div class="yp-io-item ${ot1 ? 'active' : ''}">
              <div class="yp-io-item-label">
                <b>OUT#0001</b>
                <span>Giác hút chân không / Nam châm điện</span>
              </div>
              <button class="yp-io-btn ${ot1 ? 'btn-active' : ''}" id="yp-btn-toggle-ot1">${ot1 ? 'ON (ĐANG HÚT)' : 'OFF (NHẢ)'}</button>
            </div>
            <div class="yp-io-item">
              <div class="yp-io-item-label"><b>OUT#0002</b><span>Băng tải 1 Infeed Inverter Run</span></div>
              <span class="yp-io-val">OFF</span>
            </div>
            <div class="yp-io-item">
              <div class="yp-io-item-label"><b>OUT#0003</b><span>Xung 100ms thả cụm xong (DI_12)</span></div>
              <span class="yp-io-val">READY</span>
            </div>
            <div class="yp-io-item">
              <div class="yp-io-item-label"><b>OUT#0005</b><span>Đèn chỉ thị Q4 trên Control Panel</span></div>
              <span class="yp-io-val">OFF</span>
            </div>
          </div>

          <div class="yp-io-col">
            <span class="yp-io-title">INPUTS (IN#0001..IN#0008)</span>
            <div class="yp-io-item">
              <div class="yp-io-item-label"><b>IN#0001</b><span>Cảm biến quang DI_11 (Cụm 4 thanh sẵn sàng)</span></div>
              <span class="yp-io-val">${(this.app.factory && this.app.factory.sensorDI11) ? 'ON (DETECTED)' : 'OFF'}</span>
            </div>
            <div class="yp-io-item">
              <div class="yp-io-item-label"><b>IN#0002</b><span>Cảm biến thùng carton DI_12 đỗ tại trạm</span></div>
              <span class="yp-io-val">${(this.app.factory && this.app.factory.sensorDI12) ? 'ON' : 'OFF'}</span>
            </div>
            <div class="yp-io-item">
              <div class="yp-io-item-label"><b>IN#0005</b><span>Công tắc gạt Panel I4 (Gắp A1)</span></div>
              <span class="yp-io-val">OFF</span>
            </div>
            <div class="yp-io-item">
              <div class="yp-io-item-label"><b>IN#0006</b><span>Công tắc gạt Panel I5 (Gắp A2)</span></div>
              <span class="yp-io-val">OFF</span>
            </div>
          </div>
        </div>
      </div>
    `;

    const btnOt1 = document.getElementById('yp-btn-toggle-ot1');
    if (btnOt1) {
      btnOt1.addEventListener('click', () => {
        const newState = !ot1;
        if (this.hardwareBus) this.hardwareBus.setRobotOT(1, newState ? 1 : 0);
        if (this.physicsEngine) {
          if (newState) {
            const pack = this.physicsEngine.findReachableCollatedPack(-1.25, 0.45);
            if (pack) this.physicsEngine.attachArmGripper(pack);
          } else {
            this.physicsEngine.releaseArmGripper();
          }
        }
        this.updateScreenZone4();
        this.setHumanMessage(`OUT#0001 (Giác hút) đã chuyển sang: ${newState ? 'ON' : 'OFF'}`, 'info');
      });
    }
  }

  renderRobotScreen(container) {
    const q = this.robotArm ? this.robotArm.getJointAngles() : { s: 0, l: 0, u: 0, r: 0, b: 0, t: 0 };

    container.innerHTML = `
      <div class="yp-robot-view">
        <div class="yp-view-header">
          <span style="color: #38bdf8; font-weight: 800; font-size: 13px;">ROBOT // WORK HOME POS (HÌNH 11 BÀI 1)</span>
          <button class="yp-large-btn btn-success" id="yp-btn-fwd-home">[FWD] VỀ VỊ TRÍ GỐC HOME (0° TẤT CẢ KHỚP)</button>
        </div>

        <p style="font-size: 12px; color: #cbd5e1; margin: 8px 0 12px 0; line-height: 1.5;">
          Home là gốc 0° của tất cả các góc khớp robot. Khi bắt đầu hoặc kết thúc một thao tác, robot cần được đưa về vị trí Home. Nhấn [FWD] để điều khiển tất cả các trục về 0°.
        </p>

        <div class="yp-coord-grid">
          <div class="yp-cbox"><span>Khớp 1: S (Base)</span><b>${q.s.toFixed(1)}°</b> (Gốc 0°)</div>
          <div class="yp-cbox"><span>Khớp 2: L (Lower Arm)</span><b>${q.l.toFixed(1)}°</b> (Gốc 0°)</div>
          <div class="yp-cbox"><span>Khớp 3: U (Upper Arm)</span><b>${q.u.toFixed(1)}°</b> (Gốc 0°)</div>
          <div class="yp-cbox"><span>Khớp 4: R (Roll)</span><b>${q.r.toFixed(1)}°</b> (Gốc 0°)</div>
          <div class="yp-cbox"><span>Khớp 5: B (Bend)</span><b>${q.b.toFixed(1)}°</b> (Gốc 0°)</div>
          <div class="yp-cbox"><span>Khớp 6: T (Twist)</span><b>${q.t.toFixed(1)}°</b> (Gốc 0°)</div>
        </div>
      </div>
    `;

    const btnHome = document.getElementById('yp-btn-fwd-home');
    if (btnHome) {
      btnHome.addEventListener('click', () => {
        this.moveArmToHomeZero();
      });
    }
  }

  renderSystemScreen(container) {
    container.innerHTML = `
      <div class="yp-sys-view">
        <div class="yp-view-header">
          <span style="color: #e2e8f0; font-weight: 800; font-size: 13px;">SYSTEM INFO // YASKAWA YRC1000micro</span>
          <span class="yp-group-badge">YASKAWA ELECTRIC CORP</span>
        </div>

        <div class="yp-sys-list">
          <div><span>BỘ ĐIỀU KHIỂN:</span> <b>Yaskawa YRC1000micro (Small Manipulator Controller)</b></div>
          <div><span>CÁNH TAY ROBOT:</span> <b>MotoMINI (6 Bậc tự do / 6-DOF)</b></div>
          <div><span>NGÔN NGỮ LẬP TRÌNH:</span> <b>INFORM III Industrial Robot Language</b></div>
          <div><span>DỰ ÁN ĐANG CHỌN (ACTIVE JOB):</span> <b style="color:#4ade80;">${this.currentJobName}</b></div>
          <div><span>KHÔNG GIAN LÀM VIỆC:</span> <b>Hình cầu bán kính R = 350 mm</b></div>
          <div><span>TẢI TRỌNG / ĐỘ LẶP LẠI:</span> <b>Payload: 0.5 kg // Repeatability: ±0.02 mm</b></div>
        </div>
      </div>
    `;
  }

  // ===========================================================================
  // INFORM COMMAND MODAL (INFORM LIST)
  // ===========================================================================

  openInformListModal() {
    const modal = document.getElementById('yp-inform-modal');
    if (!modal) return;

    modal.style.display = 'flex';
    this.renderInformModalCategory('MOTION');

    modal.querySelectorAll('.yp-itab-btn').forEach(tab => {
      tab.addEventListener('click', () => {
        modal.querySelectorAll('.yp-itab-btn').forEach(t => t.classList.remove('active'));
        tab.classList.add('active');
        this.renderInformModalCategory(tab.dataset.cat);
      });
    });
  }

  renderInformModalCategory(category) {
    const body = document.getElementById('yp-inform-list-body');
    if (!body) return;

    const commands = {
      MOTION: [
        { code: 'MOVJ P000 VJ=50.00', desc: 'Di chuyển nội suy góc khớp nhanh về điểm Home (P000)' },
        { code: 'MOVJ P001 VJ=80.00', desc: 'Di chuyển nội suy góc khớp tới điểm tiếp cận P001' },
        { code: 'MOVL P002 V=250.0 PL=0', desc: 'Di chuyển nội suy đường thẳng Cartesian tới điểm gắp P002' },
        { code: 'MOVL P004 V=250.0 PL=0', desc: 'Di chuyển nội suy đường thẳng hạ xếp vào thùng P004' },
        { code: 'SPEED VJ=50.00', desc: 'Cài đặt tốc độ di chuyển mặc định' }
      ],
      INOUT: [
        { code: 'DOUT OT#(1) ON', desc: 'Bật giác hút chân không / nam châm điện' },
        { code: 'DOUT OT#(1) OFF', desc: 'Ngắt giác hút chân không / nhả phôi' },
        { code: 'PULSE OT#(3) T=0.10', desc: 'Xuất xung 100ms báo PLC hoàn tất thả cụm (DI_12)' },
        { code: 'PULSE OT#(5) T=1.00', desc: 'Xuất xung 1s báo đèn Q4 hoàn thành chu trình' },
        { code: 'WAIT IN#(1)=ON', desc: 'Chờ cảm biến cụm phôi DI_11 sẵn sàng' },
        { code: 'WAIT IN#(5)=ON', desc: 'Chờ công tắc gạt I4 trên Panel ON' }
      ],
      CONTROL: [
        { code: 'TIMER T=0.300', desc: 'Trì hoãn chương trình 300 miligiây (Dwell time)' },
        { code: 'JUMP *LABEL', desc: 'Nhảy đến nhãn đánh dấu trong chương trình' },
        { code: 'CALL JOB:PICK_A1', desc: 'Gọi chương trình con gắp tại A1' },
        { code: 'NOP', desc: 'Lệnh rỗng (Đầu chương trình)' },
        { code: 'END', desc: 'Kết thúc chương trình JOB' }
      ]
    };

    const list = commands[category] || [];
    let html = '';
    list.forEach(item => {
      html += `
        <div class="yp-cmd-choice-item" data-code="${item.code}">
          <div class="cmd-code">${item.code}</div>
          <div class="cmd-desc">${item.desc}</div>
        </div>
      `;
    });

    body.innerHTML = html;

    body.querySelectorAll('.yp-cmd-choice-item').forEach(item => {
      item.addEventListener('click', () => {
        const code = item.dataset.code;
        this.bufferLineText = code;
        const input = document.getElementById('yp-buffer-input');
        if (input) input.value = code;
        document.getElementById('yp-inform-modal').style.display = 'none';
        this.setHumanMessage(`Đã chọn lệnh vào BUFFER: ${code}`, 'info');
      });
    });
  }

  // ===========================================================================
  // EXECUTION & STEPPING ([FWD], [BWD], [TEST START], [PLAY START])
  // ===========================================================================

  async stepForwardINFORM() {
    if (!this.canJog() && this.robotMode === 0) return;

    if (this.currentScreen === 'JOB') {
      const instructions = this.getCurrentInstructions();
      if (this.currentLineIndex >= instructions.length) {
        this.currentLineIndex = 0;
      }

      const line = instructions[this.currentLineIndex];
      this.updateScreenZone4();
      this.setHumanMessage(`[FWD DÒNG ${this.currentLineIndex}]: ${line}`, 'info');

      await this.executeInformLine(line);
      this.currentLineIndex++;
      if (this.currentLineIndex >= instructions.length) {
        this.currentLineIndex = 0;
      }
      this.updateScreenZone4();
    } else if (this.currentScreen === 'VARIABLE') {
      const v = this.posVariables[this.selectedVarIndex];
      await this.moveArmToVariable(v);
    } else if (this.currentScreen === 'ROBOT') {
      await this.moveArmToHomeZero();
    }
  }

  stepBackwardINFORM() {
    if (this.currentLineIndex > 0) {
      this.currentLineIndex--;
      this.updateScreenZone4();
      const instructions = this.getCurrentInstructions();
      this.setHumanMessage(`[BWD DÒNG ${this.currentLineIndex}]: ${instructions[this.currentLineIndex]}`, 'info');
    }
  }

  async runTestStartINFORM() {
    if (this.isRunningJob) return;
    this.isRunningJob = true;
    this.jobAborted = false;
    this.setHumanMessage(`[TEST START] Đang chạy thử Project [${this.currentJobName}] trong TEACH mode...`, "warning");

    const instructions = this.getCurrentInstructions();
    try {
      while (this.currentLineIndex < instructions.length && !this.jobAborted && this.interlockActive) {
        const line = instructions[this.currentLineIndex];
        this.updateScreenZone4();
        await this.executeInformLine(line);
        this.currentLineIndex++;
        await new Promise(r => setTimeout(r, 60));
      }
    } catch (err) {
      console.error("Test start error:", err);
    } finally {
      this.isRunningJob = false;
      this.updateScreenZone4();
    }
  }

  async runPlayStartINFORM() {
    if (this.isRunningJob) return;
    this.isRunningJob = true;
    this.isPaused = false;
    this.jobAborted = false;
    this.startLit = true;
    this.holdLit = false;
    this.updatePendantDisplay();
    this.setHumanMessage(`[PLAY START] Bắt đầu chạy tự động Project [${this.currentJobName}]...`, "success");

    const instructions = this.getCurrentInstructions();
    try {
      while (this.currentLineIndex < instructions.length && !this.jobAborted) {
        if (this.isPaused) {
          await new Promise(r => setTimeout(r, 100));
          continue;
        }

        const line = instructions[this.currentLineIndex];
        this.updateScreenZone4();

        const res = await this.executeInformLine(line);
        if (res && res.jumpTo) {
          const target = `*${res.jumpTo}`;
          const idx = instructions.findIndex(l => l.startsWith(target));
          if (idx !== -1) {
            this.currentLineIndex = idx;
            continue;
          }
        }

        this.currentLineIndex++;
        await new Promise(r => setTimeout(r, 50));
      }

      this.setHumanMessage(`[HOÀN TẤT PROJECT ${this.currentJobName}] Thực thi kết thúc thành công!`, "success");
    } catch (err) {
      console.error("Play job error:", err);
      this.setHumanMessage(`[LỖI CHƯƠNG TRÌNH]: ${err.message}`, "danger");
    } finally {
      this.isRunningJob = false;
      this.startLit = false;
      this.currentLineIndex = 0;
      this.updatePendantDisplay();
      this.updateScreenZone4();
    }
  }

  stopJobExecution() {
    this.jobAborted = true;
    this.isRunningJob = false;
    this.startLit = false;
    if (this.planner) {
      this.planner.stopAll();
    }
    this.updatePendantDisplay();
  }

  async executeInformLine(line) {
    if (!line || line.startsWith('//') || line.startsWith("'") || line === 'NOP' || line === 'END') {
      return { success: true };
    }

    const sleep = (ms) => new Promise(res => setTimeout(res, ms));

    // 1. MOVJ P00x VJ=...
    if (line.startsWith('MOVJ')) {
      const match = line.match(/MOVJ\s+(P\d+)(?:\s+VJ=([\d.]+))?/i);
      if (match) {
        const pId = match[1].toUpperCase();
        const vj = match[2] ? parseFloat(match[2]) : 50.0;
        const v = this.posVariables.find(item => item.id === pId);

        if (v) {
          const duration = Math.max(0.22, (100.0 / vj) * 0.35);
          const startQ = this.robotArm.getJointAngles();
          const targetPose = { x: v.x, y: v.y, z: v.z, rx: v.rx, ry: v.ry, rz: v.rz };
          const ik = this.kinematics.inverseKinematics(targetPose, startQ, true);

          if (ik.success) {
            const traj = this.planner.createMOVJ(startQ, ik.jointsDeg, { duration, degree: 5 });
            await new Promise(resolve => {
              this.planner.startTrajectory('pendant-movj', traj, {
                onUpdate: (data) => {
                  const q = data.joints;
                  this.robotArm.setJointAngles(q.s, q.l, q.u, q.r, q.b, q.t);
                  this.app.updateArmUIFromModel();
                  this.updateLiveReadouts();
                },
                onComplete: resolve
              });
            });
          }
        }
      }
      return { success: true };
    }

    // 2. MOVL P00x V=...
    if (line.startsWith('MOVL')) {
      const match = line.match(/MOVL\s+(P\d+)(?:\s+V=([\d.]+))?/i);
      if (match) {
        const pId = match[1].toUpperCase();
        const vMm = match[2] ? parseFloat(match[2]) : 200.0;
        const v = this.posVariables.find(item => item.id === pId);

        if (v) {
          const startPose = this.kinematics.forwardKinematics(this.robotArm.getJointAngles());
          const targetPose = { x: v.x, y: v.y, z: v.z, rx: v.rx, ry: v.ry, rz: v.rz };

          const dist = Math.sqrt(
            Math.pow(targetPose.x - startPose.tcp_m.x, 2) +
            Math.pow(targetPose.y - startPose.tcp_m.y, 2) +
            Math.pow(targetPose.z - startPose.tcp_m.z, 2)
          );

          const duration = Math.max(0.20, (dist * 1000.0) / vMm);
          const traj = this.planner.createMOVL(
            { x: startPose.tcp_m.x, y: startPose.tcp_m.y, z: startPose.tcp_m.z, rx: startPose.orientation_deg.rx, ry: startPose.orientation_deg.ry, rz: startPose.orientation_deg.rz },
            targetPose,
            this.kinematics,
            { duration, degree: 5, isWorld: true, initialJoints: this.robotArm.getJointAngles() }
          );

          await new Promise(resolve => {
            this.planner.startTrajectory('pendant-movl', traj, {
              onUpdate: (data) => {
                if (data.ikResult && data.ikResult.success) {
                  const q = data.ikResult.jointsDeg;
                  this.robotArm.setJointAngles(q.s, q.l, q.u, q.r, q.b, q.t);
                  this.app.updateArmUIFromModel();
                  this.updateLiveReadouts();
                }
              },
              onComplete: resolve
            });
          });
        }
      }
      return { success: true };
    }

    // 3. DOUT OT#(x) ON / OFF
    if (line.startsWith('DOUT')) {
      const match = line.match(/DOUT\s+OT#\((\d+)\)\s+(ON|OFF)/i);
      if (match) {
        const otNum = parseInt(match[1], 10);
        const state = (match[2].toUpperCase() === 'ON');

        if (this.hardwareBus) {
          this.hardwareBus.setRobotOT(otNum, state ? 1 : 0);
        }

        if (otNum === 1 && this.physicsEngine) {
          if (state) {
            const pack = this.physicsEngine.findReachableCollatedPack(-1.25, 0.45);
            if (pack) this.physicsEngine.attachArmGripper(pack);
          } else {
            this.physicsEngine.releaseArmGripper();
          }
        }
      }
      return { success: true };
    }

    // 4. PULSE OT#(x) T=...
    if (line.startsWith('PULSE')) {
      const match = line.match(/PULSE\s+OT#\((\d+)\)/i);
      if (match) {
        const otNum = parseInt(match[1], 10);
        if (this.hardwareBus) {
          this.hardwareBus.setRobotOT(otNum, 1);
          setTimeout(() => {
            if (this.hardwareBus) this.hardwareBus.setRobotOT(otNum, 0);
          }, 100);
        }
      }
      return { success: true };
    }

    // 5. TIMER T=...
    if (line.startsWith('TIMER')) {
      const match = line.match(/TIMER\s+T=([\d.]+)/i);
      if (match) {
        const tSec = parseFloat(match[1]);
        await sleep(tSec * 1000);
      }
      return { success: true };
    }

    // 6. WAIT IN#(x)=ON / OFF
    if (line.startsWith('WAIT')) {
      const match = line.match(/WAIT\s+IN#\((\d+)\)=(ON|OFF)/i);
      if (match) {
        const inNum = parseInt(match[1], 10);
        const expected = (match[2].toUpperCase() === 'ON') ? 1 : 0;
        let count = 0;
        while (this.hardwareBus && !this.jobAborted) {
          if (this.hardwareBus.robotIN[inNum] === expected) break;
          if (inNum === 1 && this.app.factory && this.app.factory.sensorDI11) {
            this.hardwareBus.robotIN[1] = 1;
            break;
          }
          await sleep(100);
          count++;
          if (count > 40) break;
        }
      }
      return { success: true };
    }

    // 7. JUMP *LABEL
    if (line.startsWith('JUMP')) {
      const match = line.match(/JUMP\s+\*?(\w+)/i);
      if (match) return { success: true, jumpTo: match[1] };
    }

    return { success: true };
  }

  async moveArmToVariable(v) {
    if (!v || !this.robotArm || !this.planner) return;
    const curQ = this.robotArm.getJointAngles();
    const duration = this.jogSpeeds[this.jogSpeedMode].duration;

    const traj = this.planner.createMOVJ(
      curQ,
      { s: v.s, l: v.l, u: v.u, r: v.r, b: v.b, t: v.t },
      { duration, degree: 5 }
    );

    await new Promise(resolve => {
      this.planner.startTrajectory('pendant-step-var', traj, {
        onUpdate: (data) => {
          const q = data.joints;
          this.robotArm.setJointAngles(q.s, q.l, q.u, q.r, q.b, q.t);
          this.app.updateArmUIFromModel();
          this.updateLiveReadouts();
        },
        onComplete: resolve
      });
    });
  }

  /**
   * Chạy tự động chu trình vẽ quỹ đạo chữ T
   */
  async runAutoLetterT() {
    return runAutoLetterTJob(this.app);
  }

  async moveArmToHomeZero() {
    if (!this.robotArm || !this.planner) return;
    const curQ = this.robotArm.getJointAngles();
    const traj = this.planner.createMOVJ(
      curQ,
      { s: 0, l: 0, u: 0, r: 0, b: 0, t: 0 },
      { duration: 0.5, degree: 5 }
    );

    await new Promise(resolve => {
      this.planner.startTrajectory('pendant-zero-home', traj, {
        onUpdate: (data) => {
          const q = data.joints;
          this.robotArm.setJointAngles(q.s, q.l, q.u, q.r, q.b, q.t);
          this.app.updateArmUIFromModel();
          this.updateLiveReadouts();
        },
        onComplete: resolve
      });
    });
    this.setHumanMessage("Robot đã hoàn tất trở về vị trí Home (Tất cả góc khớp = 0°).", 'success');
  }

  // ===========================================================================
  // UI DISPLAY & LABELS SYNC
  // ===========================================================================

  updateJogKeyLabels() {
    const isJoint = (this.currentCoord === 'JOINT');
    const labels = isJoint
      ? [['S -', 'S +'], ['L -', 'L +'], ['U -', 'U +'], ['R -', 'R +'], ['B -', 'B +'], ['T -', 'T +']]
      : [['X -', 'X +'], ['Y -', 'Y +'], ['Z -', 'Z +'], ['Rx -', 'Rx +'], ['Ry -', 'Ry +'], ['Rz -', 'Rz +']];

    for (let i = 0; i < 6; i++) {
      const bNeg = document.getElementById(`yp-jkey-${i}-neg`);
      const bPos = document.getElementById(`yp-jkey-${i}-pos`);
      if (bNeg) bNeg.querySelector('.yp-jkey-sym').textContent = labels[i][0];
      if (bPos) bPos.querySelector('.yp-jkey-sym').textContent = labels[i][1];
    }

    const badge = document.getElementById('yp-jog-coord-badge');
    if (badge) badge.textContent = this.currentCoord;
  }

  updatePendantDisplay() {
    const knob = document.getElementById('yp-knob-indicator');
    if (knob) {
      knob.className = `yp-rotary-knob pos-${['teach', 'play', 'remote'][this.robotMode]}`;
    }

    document.querySelectorAll('.yp-dm-btn').forEach(btn => {
      btn.classList.toggle('active', parseInt(btn.dataset.dm, 10) === this.deadmanState);
    });
    const dmBadge = document.getElementById('yp-deadman-state-badge');
    if (dmBadge) {
      const names = ['RELEASED (OFF)', 'LIGHT SQUEEZE (ON - SAFE)', 'TIGHT SQUEEZE (OFF - PANIC)'];
      dmBadge.textContent = names[this.deadmanState];
      dmBadge.className = `yp-deadman-state-badge ${this.deadmanState === 1 ? 'badge-safe' : 'badge-danger'}`;
    }

    const ledServo = document.getElementById('yp-led-servo');
    const lblServo = document.getElementById('yp-lbl-servo-status');
    const icoServo = document.getElementById('yp-ico-servo');
    const symServoLed = document.getElementById('yp-sym-servo-led');
    const txtServo = document.getElementById('yp-txt-servo');

    if (ledServo) {
      ledServo.className = `yp-servo-led ${this.servoOn ? 'active' : (this.servoReady ? 'blinking' : '')}`;
    }
    if (lblServo) {
      lblServo.textContent = `SERVO: ${this.servoOn ? 'ON' : 'OFF'}`;
      lblServo.style.color = this.servoOn ? '#4ade80' : '#94a3b8';
    }
    if (symServoLed) {
      symServoLed.className = `ico-led ${this.servoOn ? 'on' : 'off'}`;
    }
    if (txtServo) {
      txtServo.textContent = this.servoOn ? 'SERVO ON' : 'SERVO OFF';
    }
    if (icoServo) {
      icoServo.classList.toggle('active', this.servoOn);
    }

    const btnStart = document.getElementById('yp-btn-start');
    if (btnStart) btnStart.classList.toggle('lit', this.startLit);

    const btnHold = document.getElementById('yp-btn-hold');
    if (btnHold) btnHold.classList.toggle('lit', this.holdLit);

    const btnEstop = document.getElementById('yp-btn-estop');
    if (btnEstop) btnEstop.classList.toggle('depressed', this.eStop);

    const symMode = document.getElementById('yp-sym-mode');
    const txtMode = document.getElementById('yp-txt-mode');
    if (symMode && txtMode) {
      const syms = ['☝T', '↻P', '📡R'];
      const txts = ['TEACH', 'PLAY', 'REMOTE'];
      symMode.textContent = syms[this.robotMode];
      txtMode.textContent = txts[this.robotMode];
    }

    const symCoord = document.getElementById('yp-sym-coord');
    const txtCoord = document.getElementById('yp-txt-coord');
    if (symCoord && txtCoord) {
      symCoord.textContent = (this.currentCoord === 'JOINT') ? '☵' : (this.currentCoord === 'CARTESIAN' ? '⊞' : '⛯');
      txtCoord.textContent = this.currentCoord;
    }

    const symSpeed = document.getElementById('yp-sym-speed');
    const txtSpeed = document.getElementById('yp-txt-speed');
    const pillSpeed = document.getElementById('yp-cur-speed-pill');
    if (symSpeed && txtSpeed) {
      const symbols = { INCH: '𝚰', LOW: '𝑳', MED: '𝑴', HIGH: '𝑯' };
      symSpeed.textContent = symbols[this.jogSpeedMode];
      txtSpeed.textContent = this.jogSpeedMode;
    }
    if (pillSpeed) {
      pillSpeed.textContent = this.jogSpeeds[this.jogSpeedMode].label;
    }

    const symState = document.getElementById('yp-sym-state');
    const txtState = document.getElementById('yp-txt-state');
    if (symState && txtState) {
      if (this.eStop) {
        symState.textContent = '⚠';
        txtState.textContent = 'ESTOP';
      } else if (this.isRunningJob) {
        symState.textContent = '▶';
        txtState.textContent = 'RUN';
      } else if (this.holdLit) {
        symState.textContent = '❚❚';
        txtState.textContent = 'HOLD';
      } else {
        symState.textContent = '■';
        txtState.textContent = 'STOP';
      }
    }
  }

  setHumanMessage(msg, type = 'info') {
    this.systemMessage = msg;
    this.systemMessageType = type;

    const elText = document.getElementById('yp-prompt-text');
    const elPfx = document.getElementById('yp-prompt-prefix');
    const elLine = document.getElementById('yp-prompt-line');

    if (elText) elText.textContent = msg;
    if (elPfx) {
      const pfxMap = { info: '[INFO]', warning: '[CẢNH BÁO]', danger: '[NGUY CẤP]', success: '[THÀNH CÔNG]' };
      elPfx.textContent = pfxMap[type] || '[INFO]';
    }
    if (elLine) {
      elLine.className = `yp-prompt-line type-${type}`;
    }
  }

  navigateCursor(dir) {
    const instructions = this.getCurrentInstructions();
    if (this.currentScreen === 'JOB' && this.jobSubtab === 'CONTENT') {
      const next = this.currentLineIndex + dir;
      if (next >= 0 && next < instructions.length) {
        this.currentLineIndex = next;
        this.updateScreenZone4();
        const line = instructions[this.currentLineIndex];
        if (line) {
          this.bufferLineText = line;
          const input = document.getElementById('yp-buffer-input');
          if (input) input.value = line;
        }
      }
    } else if (this.currentScreen === 'VARIABLE') {
      const next = this.selectedVarIndex + dir;
      if (next >= 0 && next < this.posVariables.length) {
        this.selectedVarIndex = next;
        this.updateScreenZone4();
      }
    }
  }

  updateLiveReadouts() {
    // Keeps live feedback in sync
  }
}
