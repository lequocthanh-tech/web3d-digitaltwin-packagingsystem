import { Delta3DOFKinematics } from '../kinematics/Delta3DOFKinematics.js';

/**
 * DeltaTeachPendantStation.js
 * 
 * Industrial Virtual Teach Pendant & Auto Cycle Controller for 3-DOF Parallel Delta Robot
 * Benchmark Specifications:
 * - Upper Fixed Base Radius: R_base = 180 mm
 * - Upper Arm Length: L1 = 200 mm
 * - Lower Arm Length: L2 = 500 mm
 * - Mobile Platform Radius: r_platform = 50 mm
 * - Workspace: Z in [-600, -350] mm, R_max = 250 mm
 * - Cartesian Jog: [X+][X-], [Y+][Y-], [Z+][Z-] with Step (1mm, 10mm, 50mm)
 * - Vacuum Gripper Control: [VACUUM ON/OFF]
 * - Point Memory: Pick (Conveyor), Place (Tray), Safe/Home (High clearance)
 * - Auto Cycle: Gate trajectory (Inverted-U / Pi-curve) with CPM Speed Control
 */
export class DeltaTeachPendantStation {
  constructor(app, options = {}) {
    this.app = app;
    this.deltaRobot = app.deltaRobot;
    this.kinematics = new Delta3DOFKinematics({
      R_base: 180.0,
      r_platform: 50.0,
      L1: 200.0,
      L2: 500.0,
      zMin: -445.0, // Chặn mềm: không để đầu hút chạm sâu cơ khí vào mặt băng tải (-445.5 mm)
      zMax: -340.0,
      rMax: 250.0
    });

    // Window Display & Docking State: 'docked' or 'floating'
    this.dockMode = options.dockMode || 'docked';
    this.isMinimized = false;
    this.windowPos = { x: null, y: null };
    this.isDragging = false;
    this.dragOffset = { x: 0, y: 0 };

    // Cartesian Coordinates (in millimeters, local Delta frame)
    // Z is negative downward from base plate
    this.currentPos = {
      x: 0.0,
      y: 0.0,
      z: -380.0
    };

    // Motor joint angles in degrees
    this.currentAngles = {
      theta1: 1.24,
      theta2: 1.24,
      theta3: 1.24
    };

    // Jog Step Size: 1, 10, or 50 mm
    this.jogStep = 10.0;

    // Vacuum Pneumatic Gripper State
    this.vacuumActive = false;
    this.vacuumPressureKPa = 0.0; // 0 (vent) to -85 (suction)

    // Point Memory Management (Pick, Place, Safe)
    this.selectedPointKey = 'PICK';
    this.pointMemory = {
      PICK: {
        id: 'PICK',
        name: 'Vị trí hút phôi trên băng tải (Pick Belt)',
        x: 0.0,
        y: -120.0,
        z: -408.0,
        desc: 'Hạ sát bề mặt phôi sô-cô-la trên băng tải (cách mặt băng 38mm, không va chạm)'
      },
      PLACE: {
        id: 'PLACE',
        name: 'Vị trí thả phôi vào khay chứa (Place Tray)',
        x: 120.0,
        y: 100.0,
        z: -408.0,
        desc: 'Hạ thả êm ái vào ô khay thành phẩm bên cạnh'
      },
      SAFE: {
        id: 'SAFE',
        name: 'Vị trí chờ trên cao an toàn (Safe / Home)',
        x: 0.0,
        y: 0.0,
        z: -380.0,
        desc: 'Độ cao an toàn 65.5mm phía trên mặt băng tải'
      }
    };

    // Auto Cycle & Trajectory Execution State
    this.isAutoRunning = false;
    this.isSingleCycle = false;
    this.cycleAborted = false;
    this.cpm = 60; // Cycles Per Minute (20 to 120 CPM)
    this.cycleCount = 0;
    this.activeStage = 'IDLE'; // '1_LIFT', '2_FLY_PICK', '3_LOWER_PICK', '4_GRIP', '5_LIFT_CARRY', '6_FLY_PLACE', '7_LOWER_PLACE', '8_RELEASE', '9_RETURN'
    this.cycleTimeMs = 0;

    // Prompt message
    this.systemMessage = "Sẵn sàng vận hành Robot Delta 3-DOF. Sử dụng cụm phím Jog để điều khiển hoặc nhấn [AUTO RUN] để chạy chu trình gắp thả.";
    this.systemMessageType = "info";

    this.container = null;
    this.floatingHost = null;

    // Initialize initial kinematics pose
    this.updateKinematicsPose(this.currentPos.x, this.currentPos.y, this.currentPos.z);
  }

  /**
   * Updates coordinates, solves IK, and commands the 3D model
   */
  updateKinematicsPose(x, y, z) {
    this.currentPos.x = Number(x.toFixed(2));
    this.currentPos.y = Number(y.toFixed(2));
    this.currentPos.z = Number(z.toFixed(2));

    const ik = this.kinematics.inverseKinematics(this.currentPos.x, this.currentPos.y, this.currentPos.z);
    if (ik.success) {
      this.currentAngles = ik.thetaDeg;
      if (this.deltaRobot && this.deltaRobot.setDeltaCartesian) {
        this.deltaRobot.setDeltaCartesian(
          this.currentPos.x,
          this.currentPos.y,
          this.currentPos.z,
          0
        );
      } else if (this.deltaRobot && this.deltaRobot.setJointAngles) {
        this.deltaRobot.setJointAngles(
          this.currentAngles.theta1,
          this.currentAngles.theta2,
          this.currentAngles.theta3,
          0
        );
      }
    }

    return ik;
  }

  /**
   * Initializes DOM and mounts UI
   */
  initDOM(containerEl = null) {
    if (containerEl) {
      this.container = containerEl;
      this.dockMode = 'docked';
    }

    // Ensure floating host container exists in DOM
    let host = document.getElementById('delta-pendant-floating');
    if (!host) {
      host = document.createElement('div');
      host.id = 'delta-pendant-floating';
      host.className = 'delta-pendant-floating';
      host.style.display = 'none';
      document.body.appendChild(host);
    }
    this.floatingHost = host;

    this.renderPendantHTML();
    this.bindEvents();
    this.updateLiveDisplay();
    this.applyDockMode(false);
  }

  setDockMode(mode, autoShow = false) {
    this.dockMode = mode;
    this.applyDockMode(autoShow);
  }

  applyDockMode(autoShow = false) {
    const pendantEl = document.getElementById('delta-teach-pendant');
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

    const btnDock = document.getElementById('dp-btn-toggle-dock');
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
      if (this.app && this.app.activeWorkstation === 'delta') {
        this.app.switchWorkstation('default');
      }
      this.setDockMode('floating', true);
      this.show();
    }
  }

  show() {
    if (this.dockMode === 'floating' && this.floatingHost) {
      // Chỉ hiển thị 1 tay cầm nổi tại 1 thời điểm: ẩn tay cầm Yaskawa để không chồng lấp
      const other = document.getElementById('yaskawa-pendant-floating');
      if (other) other.style.display = 'none';
      this.floatingHost.style.display = 'block';
      this.updateLiveDisplay();
    }
  }

  showFloating() {
    if (this.dockMode !== 'floating') {
      if (this.app && this.app.activeWorkstation === 'delta') {
        this.app.switchWorkstation('default');
      }
      this.setDockMode('floating', true);
    }
    this.show();
  }

  toggleFloatWindow() {
    this.toggleFloating();
  }

  hide() {
    if (this.floatingHost) {
      this.floatingHost.style.display = 'none';
    }
    if (this.dockMode === 'docked') {
      this.setDockMode('floating', false);
      if (this.app && this.app.activeWorkstation === 'delta') {
        this.app.switchWorkstation('default');
      }
    }
  }

  /**
   * Renders the master HTML structure
   */
  renderPendantHTML() {
    const html = `
      <div id="delta-teach-pendant" class="delta-teach-pendant ${this.dockMode === 'docked' ? 'is-docked' : 'is-floating'}">
        
        <!-- Header Bar -->
        <div class="dp-header-handle" id="dp-drag-handle">
          <div class="dp-header-title">
            <span class="dp-brand-logo">⚡ DELTA 3-DOF</span>
            <span class="dp-model-badge">PARALLEL ROBOT // TEACH PENDANT</span>
            <span class="dp-status-pill" id="dp-header-status-pill">STANDBY</span>
          </div>
          <div class="dp-header-actions">
            <button class="dp-win-btn" id="dp-btn-toggle-dock" title="Ghim sang phải / Thả nổi">
              ${this.dockMode === 'docked' ? '🪟 Thả nổi' : '📌 Ghim'}
            </button>
            <button class="dp-win-btn" id="dp-btn-minimize" title="Thu nhỏ / Mở rộng">➖</button>
            <button class="dp-win-btn dp-win-close" id="dp-btn-close" title="Ẩn tay cầm">✕</button>
          </div>
        </div>

        <!-- Scrollable Pendant Body -->
        <div class="dp-pendant-body" id="dp-pendant-body">

          <!-- 1. Geometric Specs Banner -->
          <div class="dp-specs-banner">
            <div class="dp-spec-chip"><span>R_base:</span> <b>180 mm</b></div>
            <div class="dp-spec-chip"><span>L1 (Biceps):</span> <b>200 mm</b></div>
            <div class="dp-spec-chip"><span>L2 (Rods):</span> <b>500 mm</b></div>
            <div class="dp-spec-chip"><span>r_plat:</span> <b>50 mm</b></div>
            <div class="dp-spec-chip highlight"><span>Z Work:</span> <b>[-445, -340] mm</b></div>
            <div class="dp-spec-chip highlight"><span>R_max:</span> <b>250 mm</b></div>
          </div>

          <!-- 2. Cartesian Jog Control & Live Readouts -->
          <div class="dp-card">
            <div class="dp-card-header">
              <span class="dp-card-title">🕹️ CỤM PHÍM JOG CARTESIAN (ĐỀ-CÁC 3 CHIỀU)</span>
              <div class="dp-step-selector">
                <span class="dp-step-label">BƯỚC (STEP):</span>
                <button class="dp-step-btn ${this.jogStep === 1 ? 'active' : ''}" data-step="1">1 mm</button>
                <button class="dp-step-btn ${this.jogStep === 10 ? 'active' : ''}" data-step="10">10 mm</button>
                <button class="dp-step-btn ${this.jogStep === 50 ? 'active' : ''}" data-step="50">50 mm</button>
              </div>
            </div>

            <!-- 3-Axis Jog Keys Row -->
            <div class="dp-jog-axes-grid">
              
              <!-- Axis X -->
              <div class="dp-jog-axis-box">
                <div class="dp-axis-name">TRỤC X (NGANG)</div>
                <div class="dp-jog-controls-row">
                  <button class="dp-jog-btn btn-neg" data-axis="x" data-dir="-1" id="dp-jog-x-neg">[X -]</button>
                  <div class="dp-axis-val-display" id="dp-disp-x">${this.currentPos.x.toFixed(1)} <small>mm</small></div>
                  <button class="dp-jog-btn btn-pos" data-axis="x" data-dir="1" id="dp-jog-x-pos">[X +]</button>
                </div>
              </div>

              <!-- Axis Y -->
              <div class="dp-jog-axis-box">
                <div class="dp-axis-name">TRỤC Y (DỌC)</div>
                <div class="dp-jog-controls-row">
                  <button class="dp-jog-btn btn-neg" data-axis="y" data-dir="-1" id="dp-jog-y-neg">[Y -]</button>
                  <div class="dp-axis-val-display" id="dp-disp-y">${this.currentPos.y.toFixed(1)} <small>mm</small></div>
                  <button class="dp-jog-btn btn-pos" data-axis="y" data-dir="1" id="dp-jog-y-pos">[Y +]</button>
                </div>
              </div>

              <!-- Axis Z -->
              <div class="dp-jog-axis-box">
                <div class="dp-axis-name">TRỤC Z (ĐỘ CAO)</div>
                <div class="dp-jog-controls-row">
                  <button class="dp-jog-btn btn-neg" data-axis="z" data-dir="-1" id="dp-jog-z-neg" title="Hạ thấp xuống">[Z -]</button>
                  <div class="dp-axis-val-display" id="dp-disp-z">${this.currentPos.z.toFixed(1)} <small>mm</small></div>
                  <button class="dp-jog-btn btn-pos" data-axis="z" data-dir="1" id="dp-jog-z-pos" title="Nâng lên cao">[Z +]</button>
                </div>
              </div>

            </div>

            <!-- Kinematics Readout & Workspace Status -->
            <div class="dp-kinematics-readout-row">
              <div class="dp-k-box">
                <span class="dp-k-label">BÁN KÍNH R:</span>
                <b class="dp-k-val" id="dp-disp-r">0.0 mm</b>
                <span class="dp-k-sub">(Tối đa 250 mm)</span>
              </div>
              <div class="dp-k-box">
                <span class="dp-k-label">GÓC MOTOR THETA 1:</span>
                <b class="dp-k-val" id="dp-disp-th1">${this.currentAngles.theta1.toFixed(1)}°</b>
              </div>
              <div class="dp-k-box">
                <span class="dp-k-label">GÓC MOTOR THETA 2:</span>
                <b class="dp-k-val" id="dp-disp-th2">${this.currentAngles.theta2.toFixed(1)}°</b>
              </div>
              <div class="dp-k-box">
                <span class="dp-k-label">GÓC MOTOR THETA 3:</span>
                <b class="dp-k-val" id="dp-disp-th3">${this.currentAngles.theta3.toFixed(1)}°</b>
              </div>
            </div>

            <div class="dp-workspace-status-badge badge-ok" id="dp-ws-badge">
              <span class="dp-status-dot"></span>
              <span id="dp-ws-badge-text">TRONG VÙNG LÀM VIỆC AN TOÀN (WORKSPACE OK)</span>
            </div>
          </div>

          <!-- 3. Vacuum Gripper Control Card -->
          <div class="dp-card dp-card-vacuum">
            <div class="dp-card-header">
              <span class="dp-card-title">💨 ĐIỀU KHIỂN GIÁC HÚT CHÂN KHÔNG (VACUUM GRIPPER)</span>
              <span class="dp-vacuum-gauge-val" id="dp-disp-vacuum-kpa">0.0 kPa (VENT)</span>
            </div>

            <div class="dp-vacuum-action-row">
              <button class="dp-vacuum-toggle-btn" id="dp-btn-toggle-vacuum">
                <span class="dp-vacuum-led" id="dp-led-vacuum"></span>
                <span class="dp-vacuum-btn-text" id="dp-text-vacuum">BẬT GIÁC HÚT (VACUUM ON)</span>
              </button>
              <div class="dp-vacuum-status-desc">
                <span>Áp suất: <b>-85 kPa</b> khi hút phôi, <b>0 kPa</b> khi nhả khí.</span>
                <span style="color: #94a3b8; font-size: 11px;">Tự động khóa phôi kẹo khi hạ sát điểm Pick.</span>
              </div>
            </div>
          </div>

          <!-- 4. Point Memory Management (Dạy điểm Gắp - Thả - Safe) -->
          <div class="dp-card">
            <div class="dp-card-header">
              <span class="dp-card-title">📍 QUẢN LÝ BỘ NHỚ ĐIỂM (POINT MEMORY TEACHING)</span>
              <div class="dp-point-actions">
                <button class="dp-action-btn btn-teach" id="dp-btn-teach-current">
                  📸 GÁN TỌA ĐỘ HIỆN TẠI VÀO ĐIỂM (TEACH)
                </button>
                <button class="dp-action-btn btn-goto" id="dp-btn-goto-point">
                  ▶ DI CHUYỂN TỚI ĐIỂM NÀY
                </button>
              </div>
            </div>

            <!-- Point Cards Grid -->
            <div class="dp-point-cards-grid">

              <!-- Pick Point -->
              <div class="dp-point-item ${this.selectedPointKey === 'PICK' ? 'active' : ''}" data-key="PICK" id="dp-pt-card-PICK">
                <div class="dp-pt-header">
                  <div class="dp-pt-radio ${this.selectedPointKey === 'PICK' ? 'checked' : ''}"></div>
                  <b class="dp-pt-name">1. ĐIỂM PICK (BĂNG TẢI)</b>
                  <span class="dp-pt-chip chip-pick">HÚT PHÔI</span>
                </div>
                <div class="dp-pt-coords">
                  <span>X: <b id="dp-val-pick-x">${this.pointMemory.PICK.x}</b></span>
                  <span>Y: <b id="dp-val-pick-y">${this.pointMemory.PICK.y}</b></span>
                  <span>Z: <b id="dp-val-pick-z">${this.pointMemory.PICK.z}</b> mm</span>
                </div>
                <div class="dp-pt-desc">${this.pointMemory.PICK.desc}</div>
              </div>

              <!-- Place Point -->
              <div class="dp-point-item ${this.selectedPointKey === 'PLACE' ? 'active' : ''}" data-key="PLACE" id="dp-pt-card-PLACE">
                <div class="dp-pt-header">
                  <div class="dp-pt-radio ${this.selectedPointKey === 'PLACE' ? 'checked' : ''}"></div>
                  <b class="dp-pt-name">2. ĐIỂM PLACE (KHAY CHỨA)</b>
                  <span class="dp-pt-chip chip-place">THẢ VÀO KHAY</span>
                </div>
                <div class="dp-pt-coords">
                  <span>X: <b id="dp-val-place-x">${this.pointMemory.PLACE.x}</b></span>
                  <span>Y: <b id="dp-val-place-y">${this.pointMemory.PLACE.y}</b></span>
                  <span>Z: <b id="dp-val-place-z">${this.pointMemory.PLACE.z}</b> mm</span>
                </div>
                <div class="dp-pt-desc">${this.pointMemory.PLACE.desc}</div>
              </div>

              <!-- Safe / Home Point -->
              <div class="dp-point-item ${this.selectedPointKey === 'SAFE' ? 'active' : ''}" data-key="SAFE" id="dp-pt-card-SAFE">
                <div class="dp-pt-header">
                  <div class="dp-pt-radio ${this.selectedPointKey === 'SAFE' ? 'checked' : ''}"></div>
                  <b class="dp-pt-name">3. ĐIỂM SAFE / HOME</b>
                  <span class="dp-pt-chip chip-safe">CHỜ TRÊN CAO</span>
                </div>
                <div class="dp-pt-coords">
                  <span>X: <b id="dp-val-safe-x">${this.pointMemory.SAFE.x}</b></span>
                  <span>Y: <b id="dp-val-safe-y">${this.pointMemory.SAFE.y}</b></span>
                  <span>Z: <b id="dp-val-safe-z">${this.pointMemory.SAFE.z}</b> mm</span>
                </div>
                <div class="dp-pt-desc">${this.pointMemory.SAFE.desc}</div>
              </div>

            </div>

            <!-- Manual Numeric Entry Modal Trigger / Inline Editor -->
            <div class="dp-manual-edit-row">
              <span style="font-size: 11.5px; color: #94a3b8; font-weight: 700;">Nhập số thủ công cho điểm đang chọn:</span>
              <div class="dp-inputs-wrap">
                <label>X: <input type="number" id="dp-inp-pt-x" value="${this.pointMemory[this.selectedPointKey].x}"></label>
                <label>Y: <input type="number" id="dp-inp-pt-y" value="${this.pointMemory[this.selectedPointKey].y}"></label>
                <label>Z: <input type="number" id="dp-inp-pt-z" value="${this.pointMemory[this.selectedPointKey].z}"></label>
                <button class="dp-action-btn btn-save-inputs" id="dp-btn-save-inputs">💾 LƯU SỐ</button>
              </div>
            </div>

          </div>

          <!-- 5. Auto Cycle & Inverted-U Gate Trajectory (Pi-Curve) -->
          <div class="dp-card dp-card-autocycle">
            <div class="dp-card-header">
              <span class="dp-card-title">⚡ QUỸ ĐẠO CHUYỂN ĐỘNG TỰ ĐỘNG (GATE TRAJECTORY / Π-CURVE)</span>
              <span class="dp-cycle-counter-badge">ĐÃ GẮP THẢ: <b id="dp-disp-cycle-count">0</b> CHU KỲ</span>
            </div>

            <!-- Stage Visualizer Strip -->
            <div class="dp-trajectory-stages-strip">
              <div class="dp-stage-step" id="dp-stg-1">1. Nhấc Z_safe</div>
              <div class="dp-stage-arrow">➔</div>
              <div class="dp-stage-step" id="dp-stg-2">2. Lao tới Pick</div>
              <div class="dp-stage-arrow">➔</div>
              <div class="dp-stage-step" id="dp-stg-3">3. Hạ & Hút phôi</div>
              <div class="dp-stage-arrow">➔</div>
              <div class="dp-stage-step" id="dp-stg-4">4. Nhấc Z_safe</div>
              <div class="dp-stage-arrow">➔</div>
              <div class="dp-stage-step" id="dp-stg-5">5. Lao tới Khay</div>
              <div class="dp-stage-arrow">➔</div>
              <div class="dp-stage-step" id="dp-stg-6">6. Hạ & Nhả khay</div>
              <div class="dp-stage-arrow">➔</div>
              <div class="dp-stage-step" id="dp-stg-7">7. Về Safe</div>
            </div>

            <!-- Speed Control (CPM Slider) -->
            <div class="dp-cpm-control-row">
              <div class="dp-cpm-label-wrap">
                <span class="dp-cpm-title">TỐC ĐỘ CHU KỲ (CPM - CYCLES PER MINUTE):</span>
                <b class="dp-cpm-val" id="dp-disp-cpm">${this.cpm} CPM</b>
                <span class="dp-cpm-period" id="dp-disp-period">(${(60 / this.cpm).toFixed(2)} giây/chu kỳ)</span>
              </div>
              <div class="dp-slider-wrap">
                <span style="font-size: 11px; color: #94a3b8;">20 CPM</span>
                <input type="range" min="20" max="120" step="5" value="${this.cpm}" class="dp-cpm-slider" id="dp-slider-cpm">
                <span style="font-size: 11px; color: #38bdf8; font-weight: 800;">120 CPM</span>
              </div>
            </div>

            <!-- Auto Cycle Command Buttons -->
            <div class="dp-cycle-buttons-row">
              <button class="dp-master-btn btn-auto-run" id="dp-btn-auto-run">
                <span class="dp-btn-icon">▶</span>
                <span>AUTO RUN (CHẠY LIÊN TỤC)</span>
              </button>
              <button class="dp-master-btn btn-single-cycle" id="dp-btn-single-cycle">
                <span class="dp-btn-icon">⚡</span>
                <span>CHẠY 1 CHU KỲ</span>
              </button>
              <button class="dp-master-btn btn-stop" id="dp-btn-stop-cycle">
                <span class="dp-btn-icon">⏹</span>
                <span>DỪNG (STOP)</span>
              </button>
              <button class="dp-master-btn btn-reset" id="dp-btn-reset-tray">
                <span class="dp-btn-icon">🔄</span>
                <span>RESET KHAY CHỨA</span>
              </button>
              <button class="dp-master-btn" id="dp-btn-matlab-plot" style="background:#1e3a8a; border-color:#60a5fa; color:#93c5fd;" title="Xem Đồ Thị Động Học MATLAB (Theta, Vận Tốc, Gia Tốc)">
                <span class="dp-btn-icon">📈</span>
                <span>ĐỒ THỊ MATLAB</span>
              </button>
            </div>

          </div>

          <!-- 6. Prompt & Interlock Message Bar -->
          <div class="dp-prompt-line" id="dp-prompt-line">
            <span class="dp-prompt-prefix" id="dp-prompt-prefix">[INFO]</span>
            <span class="dp-prompt-text" id="dp-prompt-text">${this.systemMessage}</span>
          </div>

        </div>

      </div>
    `;

    if (this.dockMode === 'docked' && this.container) {
      this.container.innerHTML = html;
    } else if (this.floatingHost) {
      this.floatingHost.innerHTML = html;
    }
  }

  /**
   * Binds UI events and user interactions
   */
  bindEvents() {
    // 1. Window controls
    const btnDock = document.getElementById('dp-btn-toggle-dock');
    if (btnDock) {
      btnDock.addEventListener('click', () => {
        if (this.dockMode === 'docked') {
          if (this.app && this.app.activeWorkstation === 'delta') {
            this.app.switchWorkstation('default');
          }
          this.setDockMode('floating', true);
          this.show();
        } else {
          if (this.app) {
            this.app.switchWorkstation('delta');
          } else {
            this.setDockMode('docked');
          }
        }
      });
    }

    const btnMin = document.getElementById('dp-btn-minimize');
    if (btnMin) {
      btnMin.addEventListener('click', () => {
        this.isMinimized = !this.isMinimized;
        const body = document.getElementById('dp-pendant-body');
        if (body) body.style.display = this.isMinimized ? 'none' : 'flex';
      });
    }

    const btnClose = document.getElementById('dp-btn-close');
    if (btnClose) {
      btnClose.addEventListener('click', () => this.hide());
    }

    // Dragging support for floating window
    const handle = document.getElementById('dp-drag-handle');
    if (handle) {
      handle.addEventListener('mousedown', (e) => {
        if (this.dockMode !== 'floating') return;
        this.isDragging = true;
        const rect = this.floatingHost.getBoundingClientRect();
        this.dragOffset.x = e.clientX - rect.left;
        this.dragOffset.y = e.clientY - rect.top;
      });

      window.addEventListener('mousemove', (e) => {
        if (!this.isDragging || this.dockMode !== 'floating') return;
        let nx = e.clientX - this.dragOffset.x;
        let ny = e.clientY - this.dragOffset.y;
        nx = Math.max(10, Math.min(window.innerWidth - 700, nx));
        ny = Math.max(10, Math.min(window.innerHeight - 200, ny));
        this.windowPos.x = nx;
        this.windowPos.y = ny;
        this.floatingHost.style.left = `${nx}px`;
        this.floatingHost.style.top = `${ny}px`;
        this.floatingHost.style.right = 'auto';
      });

      window.addEventListener('mouseup', () => {
        this.isDragging = false;
      });
    }

    // 2. Step Size Buttons
    document.querySelectorAll('.dp-step-btn').forEach(btn => {
      btn.addEventListener('click', () => {
        document.querySelectorAll('.dp-step-btn').forEach(b => b.classList.remove('active'));
        btn.classList.add('active');
        this.jogStep = parseFloat(btn.dataset.step) || 10.0;
        this.setPrompt(`Bước dịch chuyển Jog đã đặt: ${this.jogStep} mm`, 'info');
      });
    });

    // 3. Cartesian Jog Keys
    const axes = ['x', 'y', 'z'];
    const dirs = [-1, 1];
    axes.forEach(axis => {
      dirs.forEach(dir => {
        const id = `dp-jog-${axis}-${dir === -1 ? 'neg' : 'pos'}`;
        const btn = document.getElementById(id);
        if (btn) {
          btn.addEventListener('click', () => {
            this.performJogStep(axis, dir);
          });
        }
      });
    });

    // 4. Vacuum Gripper Toggle
    const btnVac = document.getElementById('dp-btn-toggle-vacuum');
    if (btnVac) {
      btnVac.addEventListener('click', () => {
        this.toggleVacuum();
      });
    }

    // 5. Point Memory Selection
    ['PICK', 'PLACE', 'SAFE'].forEach(key => {
      const card = document.getElementById(`dp-pt-card-${key}`);
      if (card) {
        card.addEventListener('click', () => {
          this.selectPointMemory(key);
        });
      }
    });

    // 6. Teach Current Pos Button
    const btnTeach = document.getElementById('dp-btn-teach-current');
    if (btnTeach) {
      btnTeach.addEventListener('click', () => {
        this.teachCurrentPositionToMemory();
      });
    }

    // 7. Goto Point Button
    const btnGoto = document.getElementById('dp-btn-goto-point');
    if (btnGoto) {
      btnGoto.addEventListener('click', () => {
        this.moveDirectlyToPoint(this.pointMemory[this.selectedPointKey]);
      });
    }

    // 8. Manual Inputs Save
    const btnSaveInputs = document.getElementById('dp-btn-save-inputs');
    if (btnSaveInputs) {
      btnSaveInputs.addEventListener('click', () => {
        this.saveManualPointInputs();
      });
    }

    // 9. CPM Slider
    const sliderCpm = document.getElementById('dp-slider-cpm');
    if (sliderCpm) {
      sliderCpm.addEventListener('input', () => {
        this.cpm = parseInt(sliderCpm.value, 10) || 60;
        const dispCpm = document.getElementById('dp-disp-cpm');
        const dispPeriod = document.getElementById('dp-disp-period');
        if (dispCpm) dispCpm.textContent = `${this.cpm} CPM`;
        if (dispPeriod) dispPeriod.textContent = `(${(60 / this.cpm).toFixed(2)} giây/chu kỳ)`;
      });
    }

    // 10. Auto Run Buttons
    const btnAuto = document.getElementById('dp-btn-auto-run');
    if (btnAuto) {
      btnAuto.addEventListener('click', () => {
        this.startAutoRun();
      });
    }

    const btnSingle = document.getElementById('dp-btn-single-cycle');
    if (btnSingle) {
      btnSingle.addEventListener('click', () => {
        this.startSingleCycle();
      });
    }

    const btnStop = document.getElementById('dp-btn-stop-cycle');
    if (btnStop) {
      btnStop.addEventListener('click', () => {
        this.stopCycle();
      });
    }

    const btnResetTray = document.getElementById('dp-btn-reset-tray');
    if (btnResetTray) {
      btnResetTray.addEventListener('click', () => {
        this.resetTrayWorkpieces();
      });
    }

    const btnMatlab = document.getElementById('dp-btn-matlab-plot');
    if (btnMatlab) {
      btnMatlab.addEventListener('click', () => {
        if (this.app.matlabPlotter) {
          this.app.matlabPlotter.open();
        }
      });
    }
  }

  /**
   * Performs a single Cartesian Jog step along X, Y, or Z
   */
  performJogStep(axis, dir) {
    if (this.isAutoRunning) {
      this.setPrompt("Không thể Jog khi đang chạy chu trình tự động (AUTO RUN)! Nhấn [STOP] trước.", "warning");
      return;
    }

    const delta = this.jogStep * dir;
    let newX = this.currentPos.x;
    let newY = this.currentPos.y;
    let newZ = this.currentPos.z;

    if (axis === 'x') newX += delta;
    else if (axis === 'y') newY += delta;
    else if (axis === 'z') newZ += delta;

    const ws = this.kinematics.checkWorkspace(newX, newY, newZ);
    if (!ws.valid) {
      this.setPrompt(`[CẢNH BÁO BIÊN AN TOÀN]: ${ws.reason}`, "danger");
      return;
    }

    const ik = this.updateKinematicsPose(newX, newY, newZ);
    if (!ik.success) {
      this.setPrompt(`[LỖI ĐỘNG HỌC]: ${ik.message}`, "danger");
      return;
    }

    this.updateLiveDisplay();
    this.setPrompt(`Jog [${axis.toUpperCase()}${dir > 0 ? '+' : '-'}] thành công: X=${newX.toFixed(1)}, Y=${newY.toFixed(1)}, Z=${newZ.toFixed(1)} mm`, "info");
  }

  /**
   * Toggles the Vacuum Gripper
   */
  toggleVacuum(forceState = null) {
    this.vacuumActive = (forceState !== null) ? forceState : !this.vacuumActive;
    this.vacuumPressureKPa = this.vacuumActive ? -85.0 : 0.0;

    if (this.deltaRobot && this.deltaRobot.setVacuumState) {
      this.deltaRobot.setVacuumState(this.vacuumActive);
    }

    // Attach / release physical workpiece in 3D scene
    if (this.app && this.app.physicsEngine) {
      if (this.vacuumActive) {
        // Attempt to find and grip workpiece near Delta TCP
        const pos = this.deltaRobot.getEndEffectorWorldPosition();
        const prod = this.app.physicsEngine.findNearestInfeedProduct(pos.x, pos.z, 0.15);
        if (prod) {
          this.app.physicsEngine.attachDeltaGripper(prod);
        }
      } else {
        this.app.physicsEngine.releaseDeltaGripper();
      }
    }

    this.updateLiveDisplay();
    this.setPrompt(`Giác hút chân không: ${this.vacuumActive ? 'ĐÃ BẬT (ON, -85 kPa)' : 'ĐÃ TẮT (OFF, 0 kPa)'}`, this.vacuumActive ? 'success' : 'info');
  }

  /**
   * Selects an active point memory slot
   */
  selectPointMemory(key) {
    this.selectedPointKey = key;
    ['PICK', 'PLACE', 'SAFE'].forEach(k => {
      const card = document.getElementById(`dp-pt-card-${k}`);
      if (card) {
        card.classList.toggle('active', k === key);
        const radio = card.querySelector('.dp-pt-radio');
        if (radio) radio.classList.toggle('checked', k === key);
      }
    });

    const pt = this.pointMemory[key];
    const inX = document.getElementById('dp-inp-pt-x');
    const inY = document.getElementById('dp-inp-pt-y');
    const inZ = document.getElementById('dp-inp-pt-z');
    if (inX) inX.value = pt.x;
    if (inY) inY.value = pt.y;
    if (inZ) inZ.value = pt.z;

    this.setPrompt(`Đang chọn: ${pt.name}`, 'info');
  }

  /**
   * Teaches the current (X, Y, Z) coordinates to the selected memory slot
   */
  teachCurrentPositionToMemory() {
    const pt = this.pointMemory[this.selectedPointKey];
    pt.x = Number(this.currentPos.x.toFixed(1));
    pt.y = Number(this.currentPos.y.toFixed(1));
    pt.z = Number(this.currentPos.z.toFixed(1));

    // Update displays
    const elX = document.getElementById(`dp-val-${this.selectedPointKey.toLowerCase()}-x`);
    const elY = document.getElementById(`dp-val-${this.selectedPointKey.toLowerCase()}-y`);
    const elZ = document.getElementById(`dp-val-${this.selectedPointKey.toLowerCase()}-z`);
    if (elX) elX.textContent = pt.x;
    if (elY) elY.textContent = pt.y;
    if (elZ) elZ.textContent = pt.z;

    this.selectPointMemory(this.selectedPointKey);
    this.setPrompt(`[DẠY ĐIỂM THÀNH CÔNG] Đã lưu tọa độ hiện tại vào ${pt.name}: X=${pt.x}, Y=${pt.y}, Z=${pt.z} mm`, 'success');
  }

  /**
   * Saves manually entered coordinates from text inputs
   */
  saveManualPointInputs() {
    const inX = parseFloat(document.getElementById('dp-inp-pt-x').value) || 0;
    const inY = parseFloat(document.getElementById('dp-inp-pt-y').value) || 0;
    const inZ = parseFloat(document.getElementById('dp-inp-pt-z').value) || -450;

    const ws = this.kinematics.checkWorkspace(inX, inY, inZ);
    if (!ws.valid) {
      this.setPrompt(`[TỌA ĐỘ NGOÀI VÙNG]: ${ws.reason}`, 'danger');
      return;
    }

    const pt = this.pointMemory[this.selectedPointKey];
    pt.x = inX;
    pt.y = inY;
    pt.z = inZ;

    const elX = document.getElementById(`dp-val-${this.selectedPointKey.toLowerCase()}-x`);
    const elY = document.getElementById(`dp-val-${this.selectedPointKey.toLowerCase()}-y`);
    const elZ = document.getElementById(`dp-val-${this.selectedPointKey.toLowerCase()}-z`);
    if (elX) elX.textContent = pt.x;
    if (elY) elY.textContent = pt.y;
    if (elZ) elZ.textContent = pt.z;

    this.setPrompt(`Đã cập nhật tọa độ thủ công cho ${pt.name}: X=${pt.x}, Y=${pt.y}, Z=${pt.z} mm`, 'success');
  }

  /**
   * Moves Delta robot smoothly to a target memory point
   */
  async moveDirectlyToPoint(targetPoint, durationSec = 0.5) {
    if (this.isAutoRunning) return;
    const startX = this.currentPos.x;
    const startY = this.currentPos.y;
    const startZ = this.currentPos.z;

    const steps = 30;
    const dt = (durationSec * 1000) / steps;

    for (let i = 1; i <= steps; i++) {
      const u = i / steps;
      // Quintic smoothstep
      const ease = u * u * u * (10 - 15 * u + 6 * u * u);
      const nx = startX + (targetPoint.x - startX) * ease;
      const ny = startY + (targetPoint.y - startY) * ease;
      const nz = startZ + (targetPoint.z - startZ) * ease;

      this.updateKinematicsPose(nx, ny, nz);
      this.updateLiveDisplay();
      await new Promise(r => setTimeout(r, dt));
    }

    this.setPrompt(`Đã di chuyển tới điểm: ${targetPoint.name}`, 'success');
  }

  /**
   * Starts High-Speed Inverted-U Gate Trajectory Continuous Auto Run
   */
  async startAutoRun() {
    if (this.isAutoRunning) return;
    this.isAutoRunning = true;
    this.cycleAborted = false;

    const btnAuto = document.getElementById('dp-btn-auto-run');
    if (btnAuto) btnAuto.classList.add('running');
    const pill = document.getElementById('dp-header-status-pill');
    if (pill) {
      pill.textContent = 'RUNNING (AUTO)';
      pill.className = 'dp-status-pill running';
    }

    this.setPrompt(`[AUTO RUN BẮT ĐẦU] Tốc độ chu kỳ: ${this.cpm} CPM. Đang thực hiện quỹ đạo chữ U ngược...`, 'success');

    while (this.isAutoRunning && !this.cycleAborted) {
      const startTime = performance.now();
      await this.executeGatePickAndPlaceCycle();
      this.cycleCount++;
      const elCount = document.getElementById('dp-disp-cycle-count');
      if (elCount) elCount.textContent = this.cycleCount;

      this.cycleTimeMs = Math.round(performance.now() - startTime);

      // Brief pause matching CPM target
      const targetPeriodMs = (60.0 / this.cpm) * 1000.0;
      const remainMs = Math.max(10, targetPeriodMs - this.cycleTimeMs);
      await new Promise(r => setTimeout(r, remainMs));
    }

    if (btnAuto) btnAuto.classList.remove('running');
    if (pill) {
      pill.textContent = 'STANDBY';
      pill.className = 'dp-status-pill';
    }
    this.highlightStage('');
    this.setPrompt("Đã dừng chu trình tự động (AUTO RUN STOPPED).", 'info');
  }

  /**
   * Runs exactly 1 Single Pick & Place Cycle
   */
  async startSingleCycle() {
    if (this.isAutoRunning) return;
    this.isSingleCycle = true;
    this.cycleAborted = false;

    const pill = document.getElementById('dp-header-status-pill');
    if (pill) {
      pill.textContent = 'RUNNING (SINGLE)';
      pill.className = 'dp-status-pill running';
    }

    this.setPrompt("[SINGLE CYCLE] Đang thực thi 1 chu trình gắp thả mẫu...", 'info');
    await this.executeGatePickAndPlaceCycle();
    this.cycleCount++;
    const elCount = document.getElementById('dp-disp-cycle-count');
    if (elCount) elCount.textContent = this.cycleCount;

    this.isSingleCycle = false;
    if (pill) {
      pill.textContent = 'STANDBY';
      pill.className = 'dp-status-pill';
    }
    this.highlightStage('');
    this.setPrompt("[SINGLE CYCLE HOÀN TẤT] Gắp thả thành công 1 phôi vào khay chứa!", 'success');
  }

  /**
   * Stops active cycle immediately
   */
  stopCycle() {
    this.isAutoRunning = false;
    this.isSingleCycle = false;
    this.cycleAborted = true;
    this.toggleVacuum(false);
    this.highlightStage('');
    this.setPrompt("Đã phát lệnh DỪNG (STOP) khẩn cấp!", 'warning');
  }

  /**
   * Resets workpieces in the receiving tray
   */
  resetTrayWorkpieces() {
    this.cycleCount = 0;
    const elCount = document.getElementById('dp-disp-cycle-count');
    if (elCount) elCount.textContent = '0';
    if (this.app && this.app.factory && this.app.factory.resetConveyorItems) {
      this.app.factory.resetConveyorItems();
    }
    this.setPrompt("Đã làm sạch khay chứa và reset bộ đếm chu kỳ về 0.", 'info');
  }

  /**
   * Core High-Speed Inverted-U Gate Trajectory Execution (7 Steps)
   */
  async executeGatePickAndPlaceCycle() {
    const pick = this.pointMemory.PICK;
    const place = this.pointMemory.PLACE;
    const safe = this.pointMemory.SAFE;
    const zSafe = safe.z;

    // Total nominal duration based on CPM
    const cycleDurationSec = 60.0 / this.cpm;
    // Step segment durations (seconds)
    const tLift = cycleDurationSec * 0.12;
    const tFly = cycleDurationSec * 0.28;
    const tLower = cycleDurationSec * 0.12;
    const tGrip = cycleDurationSec * 0.08;

    const sleep = (ms) => new Promise(r => setTimeout(r, ms));

    // -------------------------------------------------------------
    // 1. Nhấc lên độ cao an toàn (Z_safe) từ vị trí hiện tại
    // -------------------------------------------------------------
    this.highlightStage('dp-stg-1');
    await this.interpolateMove(this.currentPos.x, this.currentPos.y, zSafe, tLift);
    if (this.cycleAborted) return;

    // -------------------------------------------------------------
    // 2. Lao nhanh theo phương ngang tới phía trên điểm Pick
    // -------------------------------------------------------------
    this.highlightStage('dp-stg-2');
    await this.interpolateMove(pick.x, pick.y, zSafe, tFly);
    if (this.cycleAborted) return;

    // -------------------------------------------------------------
    // 3. Hạ xuống điểm Pick, kích hoạt Vacuum
    // -------------------------------------------------------------
    this.highlightStage('dp-stg-3');
    await this.interpolateMove(pick.x, pick.y, pick.z, tLower);
    this.toggleVacuum(true);
    await sleep(tGrip * 1000);
    if (this.cycleAborted) return;

    // -------------------------------------------------------------
    // 4. Nhấc phôi lên độ cao Z_safe
    // -------------------------------------------------------------
    this.highlightStage('dp-stg-4');
    await this.interpolateMove(pick.x, pick.y, zSafe, tLift);
    if (this.cycleAborted) return;

    // -------------------------------------------------------------
    // 5. Di chuyển ngang đến khay Place (với vòm cong nhẹ apex)
    // -------------------------------------------------------------
    this.highlightStage('dp-stg-5');
    await this.interpolateMove(place.x, place.y, zSafe, tFly);
    if (this.cycleAborted) return;

    // -------------------------------------------------------------
    // 6. Hạ xuống điểm Place, ngắt Vacuum nhả phôi vào khay
    // -------------------------------------------------------------
    this.highlightStage('dp-stg-6');
    await this.interpolateMove(place.x, place.y, place.z, tLower);
    this.toggleVacuum(false);
    await sleep(tGrip * 1000);
    if (this.cycleAborted) return;

    // -------------------------------------------------------------
    // 7. Trở về vị trí Safe trên cao
    // -------------------------------------------------------------
    this.highlightStage('dp-stg-7');
    await this.interpolateMove(safe.x, safe.y, safe.z, tLift);
  }

  /**
   * Smooth quintic polynomial interpolation between current and target
   */
  async interpolateMove(tx, ty, tz, durationSec) {
    const sx = this.currentPos.x;
    const sy = this.currentPos.y;
    const sz = this.currentPos.z;

    const steps = Math.max(12, Math.round(durationSec * 60));
    const dt = (durationSec * 1000) / steps;

    for (let i = 1; i <= steps; i++) {
      if (this.cycleAborted) return;
      const u = i / steps;
      // Quintic polynomial s^3 * (10 - 15s + 6s^2) ensuring 0 velocity & acceleration at ends
      const ease = u * u * u * (10.0 - 15.0 * u + 6.0 * u * u);
      const nx = sx + (tx - sx) * ease;
      const ny = sy + (ty - sy) * ease;
      const nz = sz + (tz - sz) * ease;

      this.updateKinematicsPose(nx, ny, nz);
      this.updateLiveDisplay();
      await new Promise(r => setTimeout(r, dt));
    }
  }

  /**
   * Highlights active stage in the UI flow diagram
   */
  highlightStage(id) {
    for (let s = 1; s <= 7; s++) {
      const el = document.getElementById(`dp-stg-${s}`);
      if (el) el.classList.toggle('active', `dp-stg-${s}` === id);
    }
  }

  /**
   * Updates all numbers, badges, and readouts on the pendant
   */
  updateLiveDisplay() {
    const dispX = document.getElementById('dp-disp-x');
    const dispY = document.getElementById('dp-disp-y');
    const dispZ = document.getElementById('dp-disp-z');
    const dispR = document.getElementById('dp-disp-r');
    const dispTh1 = document.getElementById('dp-disp-th1');
    const dispTh2 = document.getElementById('dp-disp-th2');
    const dispTh3 = document.getElementById('dp-disp-th3');

    if (dispX) dispX.innerHTML = `${this.currentPos.x.toFixed(1)} <small>mm</small>`;
    if (dispY) dispY.innerHTML = `${this.currentPos.y.toFixed(1)} <small>mm</small>`;
    if (dispZ) dispZ.innerHTML = `${this.currentPos.z.toFixed(1)} <small>mm</small>`;

    const r = Math.sqrt(this.currentPos.x * this.currentPos.x + this.currentPos.y * this.currentPos.y);
    if (dispR) dispR.textContent = `${r.toFixed(1)} mm`;

    if (dispTh1) dispTh1.textContent = `${this.currentAngles.theta1.toFixed(1)}°`;
    if (dispTh2) dispTh2.textContent = `${this.currentAngles.theta2.toFixed(1)}°`;
    if (dispTh3) dispTh3.textContent = `${this.currentAngles.theta3.toFixed(1)}°`;

    // Workspace check badge
    const ws = this.kinematics.checkWorkspace(this.currentPos.x, this.currentPos.y, this.currentPos.z);
    const badge = document.getElementById('dp-ws-badge');
    const badgeText = document.getElementById('dp-ws-badge-text');
    if (badge && badgeText) {
      if (ws.valid) {
        badge.className = 'dp-workspace-status-badge badge-ok';
        badgeText.textContent = 'TRONG VÙNG LÀM VIỆC AN TOÀN (WORKSPACE OK)';
      } else {
        badge.className = 'dp-workspace-status-badge badge-danger';
        badgeText.textContent = `NGOÀI VÙNG LÀM VIỆC: ${ws.reason}`;
      }
    }

    // Vacuum indicator
    const ledVac = document.getElementById('dp-led-vacuum');
    const txtVac = document.getElementById('dp-text-vacuum');
    const dispKpa = document.getElementById('dp-disp-vacuum-kpa');
    const btnVac = document.getElementById('dp-btn-toggle-vacuum');
    if (ledVac) ledVac.classList.toggle('active', this.vacuumActive);
    if (btnVac) btnVac.classList.toggle('active', this.vacuumActive);
    if (txtVac) txtVac.textContent = this.vacuumActive ? 'TẮT GIÁC HÚT (RELEASE)' : 'BẬT GIÁC HÚT (VACUUM ON)';
    if (dispKpa) {
      dispKpa.textContent = this.vacuumActive ? '-85.0 kPa (SUCTION)' : '0.0 kPa (VENT)';
      dispKpa.style.color = this.vacuumActive ? '#4ade80' : '#94a3b8';
    }
  }

  /**
   * Sets human prompt message line
   */
  setPrompt(msg, type = 'info') {
    this.systemMessage = msg;
    this.systemMessageType = type;
    const txt = document.getElementById('dp-prompt-text');
    const pfx = document.getElementById('dp-prompt-prefix');
    const line = document.getElementById('dp-prompt-line');
    if (txt) txt.textContent = msg;
    if (pfx) {
      const pfxs = { info: '[INFO]', success: '[SUCCESS]', warning: '[WARNING]', danger: '[ERROR]' };
      pfx.textContent = pfxs[type] || '[INFO]';
    }
    if (line) {
      line.className = `dp-prompt-line type-${type}`;
    }
  }
}
