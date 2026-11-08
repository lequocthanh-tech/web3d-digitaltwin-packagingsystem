/**
 * MatlabKinematicsPlotter.js
 * 
 * High-Fidelity MATLAB-Style Kinematics Figure Plotter for 6-DOF Industrial Robot Arm & Delta Robot
 * 
 * Specifically designed for 6-DOF Articulated Robot Arm (Yaskawa GP8 / 6 bậc tự do):
 * - Joint 1 (Trục S - Base / Đế xoay): theta_1
 * - Joint 2 (Trục L - Lower Arm / Cánh tay dưới): theta_2
 * - Joint 3 (Trục U - Upper Arm / Cánh tay trên): theta_3
 * - Joint 4 (Trục R - Wrist Roll / Xoay cổ tay): theta_4
 * - Joint 5 (Trục B - Wrist Pitch / Gập cổ tay): theta_5
 * - Joint 6 (Trục T - Wrist Twist / Xoay mặt bích kẹp): theta_6
 * 
 * 3 Presentation Modes:
 * 1. "grid3x3": 3x3 Subplots identical to MATLAB template image (Tabs: J1..J3 or J4..J6)
 * 2. "overview3": 3 Large Synoptic Subplots showing all 6 joints simultaneously with color legend
 * 3. "grid6x3": Full 6-column x 3-row Matrix showing all 18 state graphs simultaneously
 */

export class MatlabKinematicsPlotter {
  constructor(app) {
    this.app = app;
    this.arm6DOF = app.arm6DOF;
    this.deltaRobot = app.deltaRobot;
    this.armKinematics = app.armKinematics;
    this.deltaKinematics = app.deltaKinematics;

    this.isOpen = false;
    this.isMaximized = false;
    this.isRecording = false;
    this.recordStartTime = 0;
    this.recordedSamples = [];

    // Default settings focused on 6-DOF Robot Arm
    this.selectedRobot = 'arm6dof';      // 'arm6dof', 'delta'
    this.selectedLayout = 'grid3x3';     // 'grid3x3', 'overview3', 'grid6x3'
    this.armJointGroup = 'j123';         // 'j123' (Khớp cơ sở J1..J3), 'j456' (Khớp cổ tay J4..J6)
    this.selectedProfile = 'live_scope'; // 'live_scope' (mặc định ghi theo robot), 'arm_cycle', 'letter_t', 'matlab_s_curve'
    this.selectedUnit = 'rad';           // 'rad', 'deg'

    // Live Scope DAQ state (0% CPU khi đóng, tự động theo robot khi mở)
    this.liveScopeSamples = [];
    this.scopeElapsedTime = 0;
    this.lastSampleTime = 0;
    this.lastRedrawTime = 0;
    this.isRobotCurrentlyMoving = false;
    this.motionStoppedTimer = 0;
    this.lastArmAngles = null;
    this.lastDeltaTheta = null;

    // Plot data cache
    this.plotData = null;

    // Draggable window state
    this.isDragging = false;
    this.dragOffset = { x: 0, y: 0 };

    this.container = null;
    this.canvas = null;
    this.ctx = null;
  }

  initDOM(rootContainer = document.body) {
    if (this.container) return;

    this.container = document.createElement('div');
    this.container.id = 'matlab-figure-window';
    this.container.className = 'matlab-window-wrapper';
    this.container.style.display = 'none';

    this.container.innerHTML = `
      <div class="matlab-dialog">
        <!-- MATLAB Figure Window Title Bar -->
        <div class="matlab-titlebar" id="matlab-titlebar-handle">
          <div class="matlab-title-content">
            <span class="matlab-icon">📈</span>
            <span class="matlab-title-text" id="matlab-win-title">
              Figure 1: 6-DOF Robot Arm Kinematics [Position, Velocity, Acceleration] - MATLAB R2024b
            </span>
          </div>
          <div class="matlab-window-controls">
            <button class="matlab-win-btn" id="matlab-btn-toggle-max" title="Phóng to / Thu nhỏ">⛶</button>
            <button class="matlab-win-btn close" id="matlab-btn-close" title="Đóng">✕</button>
          </div>
        </div>

        <!-- MATLAB Menu Bar & Toolbar -->
        <div class="matlab-toolbar">
          <div class="matlab-tool-group">
            <label>Đối tượng:</label>
            <select id="matlab-sel-robot" class="matlab-select">
              <option value="arm6dof" selected>🦾 Robot Arm 6 Bậc (Yaskawa GP8 - 6-DOF)</option>
              <option value="delta">⚡ Robot Delta 3-DOF (Gắp phôi song song)</option>
            </select>
          </div>

          <div class="matlab-tool-group" id="matlab-group-arm-subset">
            <label>Khớp xem:</label>
            <div class="matlab-btn-toggle-group">
              <button class="matlab-toggle-btn active" id="btn-sub-j123" title="Xem Khớp Cơ Sở J1, J2, J3">
                J1 - J3 (Đế, Tay dưới, Tay trên)
              </button>
              <button class="matlab-toggle-btn" id="btn-sub-j456" title="Xem Khớp Cổ Tay J4, J5, J6">
                J4 - J6 (Cổ tay R, B, T)
              </button>
            </div>
          </div>

          <div class="matlab-tool-group">
            <label>Kiểu đồ thị:</label>
            <select id="matlab-sel-layout" class="matlab-select">
              <option value="grid3x3" selected>Lưới 3x3 (Chuẩn mẫu MATLAB - 3 Trục)</option>
              <option value="overview3">Đồ thị Tổng hợp 6 Khớp (3 Khung lớn lồng nhau)</option>
              <option value="grid6x3">Toàn cảnh 6 Khớp (Lưới 6 Cột x 3 Hàng)</option>
            </select>
          </div>

          <div class="matlab-tool-group">
            <label>Quỹ đạo:</label>
            <select id="matlab-sel-profile" class="matlab-select">
              <option value="live_scope" selected>📈 Đồ Thị Động Theo Robot (Live Scope - Chạy cùng Robot)</option>
              <option value="letter_t">✍️ Quỹ Đạo Chữ T (Yaskawa Teach Pendant Job)</option>
              <option value="arm_cycle">Chu trình Đóng Thùng Mẫu (Pick & Place 6 Bậc)</option>
              <option value="matlab_s_curve">Quỹ đạo Đa thức Bậc 5 Mẫu (Quintic S-Curve T=2.2s)</option>
            </select>
          </div>

          <div class="matlab-tool-group">
            <label>Đơn vị:</label>
            <select id="matlab-sel-unit" class="matlab-select">
              <option value="rad" selected>Radian (rad, rad/s, rad/s²)</option>
              <option value="deg">Độ (deg, deg/s, deg/s²)</option>
            </select>
          </div>

          <div class="matlab-tool-actions">
            <button class="matlab-action-btn" id="matlab-btn-clear-daq" style="background: #fef2f2; border-color: #fca5a5; color: #991b1b; font-weight: 600;" title="Xóa dữ liệu ghi nhận và bắt đầu lại từ 0s">
              🗑 Xóa / Ghi Mới
            </button>
            <button class="matlab-action-btn" id="matlab-btn-refresh" title="Vẽ lại đồ thị">
              🔄 Cập Nhật
            </button>
            <button class="matlab-action-btn" id="matlab-btn-export-png" title="Tải ảnh đồ thị PNG độ nét cao">
              📷 Xuất PNG
            </button>
            <button class="matlab-action-btn" id="matlab-btn-copy-script" title="Sao chép toàn bộ mã lệnh MATLAB (.m)">
              📜 Copy Code MATLAB
            </button>
            <button class="matlab-action-btn" id="matlab-btn-export-csv" title="Tải bảng dữ liệu CSV">
              📥 Xuất CSV
            </button>
          </div>
        </div>

        <!-- MATLAB Canvas Surface -->
        <div class="matlab-canvas-area" id="matlab-canvas-container">
          <canvas id="canvas-matlab-figure" width="1040" height="670"></canvas>
        </div>

        <!-- MATLAB Status Bar -->
        <div class="matlab-statusbar">
          <span id="matlab-status-info">Ready. 6-DOF Robot Arm Kinematics Analysis [Yaskawa GP8].</span>
          <span id="matlab-status-coords" style="font-family: monospace;">Time: 0..2.80s | Sampling: 120 points</span>
        </div>
      </div>
    `;

    rootContainer.appendChild(this.container);

    this.canvas = document.getElementById('canvas-matlab-figure');
    if (this.canvas) {
      this.ctx = this.canvas.getContext('2d');
    }

    this.bindEvents();
    this.injectStyles();
    // On-Demand: Chỉ tính toán và vẽ đồ thị 18-subplot khi người dùng mở cửa sổ MATLAB (`open()`),
    // tránh chiếm CPU lúc khởi động trang.
  }

  injectStyles() {
    if (document.getElementById('matlab-plotter-styles')) return;

    const style = document.createElement('style');
    style.id = 'matlab-plotter-styles';
    style.textContent = `
      .matlab-window-wrapper {
        position: fixed;
        top: 50px;
        left: 50%;
        transform: translateX(-50%);
        z-index: 99999;
        box-shadow: 0 20px 50px rgba(0, 0, 0, 0.7), 0 0 0 1px rgba(255, 255, 255, 0.2);
        font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
        border-radius: 6px;
        overflow: hidden;
      }

      .matlab-dialog {
        background: #e6e6e6; /* Classic MATLAB Figure Gray */
        color: #111;
        display: flex;
        flex-direction: column;
        width: 1060px;
        max-width: 96vw;
        border: 1px solid #7a7a7a;
      }

      .matlab-window-wrapper.maximized {
        top: 8px !important;
        left: 8px !important;
        right: 8px !important;
        bottom: 8px !important;
        transform: none !important;
        width: calc(100vw - 16px) !important;
        height: calc(100vh - 16px) !important;
      }

      .matlab-window-wrapper.maximized .matlab-dialog {
        width: 100% !important;
        height: 100% !important;
      }

      .matlab-window-wrapper.maximized .matlab-canvas-area {
        flex: 1 1 auto;
      }

      .matlab-titlebar {
        background: linear-gradient(180deg, #f2f2f2 0%, #d8d8d8 100%);
        border-bottom: 1px solid #b2b2b2;
        padding: 6px 12px;
        display: flex;
        align-items: center;
        justify-content: space-between;
        cursor: grab;
        user-select: none;
      }

      .matlab-titlebar:active {
        cursor: grabbing;
      }

      .matlab-title-content {
        display: flex;
        align-items: center;
        gap: 8px;
        font-size: 13px;
        font-weight: 600;
        color: #222;
      }

      .matlab-icon {
        font-size: 15px;
      }

      .matlab-window-controls {
        display: flex;
        gap: 4px;
      }

      .matlab-win-btn {
        background: #e0e0e0;
        border: 1px solid #999;
        border-radius: 3px;
        width: 24px;
        height: 20px;
        display: flex;
        align-items: center;
        justify-content: center;
        font-size: 11px;
        cursor: pointer;
        color: #333;
      }

      .matlab-win-btn:hover {
        background: #cfcfcf;
      }

      .matlab-win-btn.close:hover {
        background: #ef4444;
        color: #fff;
        border-color: #dc2626;
      }

      .matlab-toolbar {
        background: #f7f7f7;
        border-bottom: 1px solid #c8c8c8;
        padding: 6px 12px;
        display: flex;
        align-items: center;
        flex-wrap: wrap;
        gap: 10px;
        font-size: 12px;
      }

      .matlab-tool-group {
        display: flex;
        align-items: center;
        gap: 5px;
      }

      .matlab-tool-group label {
        font-weight: 600;
        color: #333;
        font-size: 11.5px;
      }

      .matlab-select {
        background: #fff;
        border: 1px solid #aaa;
        border-radius: 3px;
        padding: 3px 6px;
        font-size: 11.5px;
        color: #111;
        outline: none;
      }

      .matlab-btn-toggle-group {
        display: flex;
        border: 1px solid #aaa;
        border-radius: 3px;
        overflow: hidden;
      }

      .matlab-toggle-btn {
        background: #f0f0f0;
        border: none;
        border-right: 1px solid #ccc;
        padding: 3px 8px;
        font-size: 11px;
        font-weight: 600;
        color: #555;
        cursor: pointer;
      }

      .matlab-toggle-btn:last-child {
        border-right: none;
      }

      .matlab-toggle-btn.active {
        background: #2563eb;
        color: #fff;
      }

      .matlab-tool-actions {
        display: flex;
        align-items: center;
        gap: 6px;
        margin-left: auto;
      }

      .matlab-action-btn {
        background: linear-gradient(180deg, #fbfbfb 0%, #e2e2e2 100%);
        border: 1px solid #999;
        border-radius: 3px;
        padding: 3px 9px;
        font-size: 11.5px;
        font-weight: 500;
        color: #222;
        cursor: pointer;
        display: flex;
        align-items: center;
        gap: 4px;
        white-space: nowrap;
      }

      .matlab-action-btn:hover {
        background: #d4d4d4;
      }

      .matlab-action-btn.record {
        background: #fef2f2;
        border-color: #f87171;
        color: #b91c1c;
        font-weight: 600;
      }

      .matlab-action-btn.recording {
        background: #ef4444 !important;
        color: #fff !important;
        animation: pulseRecord 1s infinite alternate;
      }

      @keyframes pulseRecord {
        from { opacity: 1; }
        to { opacity: 0.7; }
      }

      .matlab-canvas-area {
        padding: 10px;
        background: #e6e6e6;
        display: flex;
        justify-content: center;
        align-items: center;
        overflow: auto;
      }

      #canvas-matlab-figure {
        background: #e6e6e6;
        box-shadow: 0 1px 3px rgba(0,0,0,0.2);
        max-width: 100%;
        height: auto;
      }

      .matlab-statusbar {
        background: #dfdfdf;
        border-top: 1px solid #c0c0c0;
        padding: 4px 12px;
        display: flex;
        justify-content: space-between;
        align-items: center;
        font-size: 11px;
        color: #555;
      }
    `;
    document.head.appendChild(style);
  }

  bindEvents() {
    // 1. Draggable Window Handle
    const handle = document.getElementById('matlab-titlebar-handle');
    if (handle) {
      handle.addEventListener('mousedown', (e) => {
        if (this.isMaximized) return;
        if (e.target.closest('.matlab-win-btn')) return;
        this.isDragging = true;
        const rect = this.container.getBoundingClientRect();
        this.dragOffset.x = e.clientX - rect.left;
        this.dragOffset.y = e.clientY - rect.top;
      });

      window.addEventListener('mousemove', (e) => {
        if (!this.isDragging) return;
        const x = Math.max(10, Math.min(window.innerWidth - 300, e.clientX - this.dragOffset.x));
        const y = Math.max(10, Math.min(window.innerHeight - 100, e.clientY - this.dragOffset.y));
        this.container.style.left = `${x}px`;
        this.container.style.top = `${y}px`;
        this.container.style.transform = 'none';
      });

      window.addEventListener('mouseup', () => {
        this.isDragging = false;
      });
    }

    // 2. Window control buttons
    document.getElementById('matlab-btn-close')?.addEventListener('click', () => {
      this.close();
    });

    document.getElementById('matlab-btn-toggle-max')?.addEventListener('click', () => {
      this.toggleMaximize();
    });

    // 3. Dropdown changes
    document.getElementById('matlab-sel-robot')?.addEventListener('change', (e) => {
      this.selectedRobot = e.target.value;
      const armGroup = document.getElementById('matlab-group-arm-subset');
      if (armGroup) {
        armGroup.style.display = (this.selectedRobot === 'arm6dof') ? 'flex' : 'none';
      }
      if (this.selectedProfile === 'live_scope') {
        this.clearLiveScopeData();
      } else {
        this.generateDataAndRender();
      }
    });

    document.getElementById('matlab-sel-layout')?.addEventListener('change', (e) => {
      this.selectedLayout = e.target.value;
      this.resizeCanvas();
      this.renderCanvas();
    });

    document.getElementById('matlab-sel-profile')?.addEventListener('change', (e) => {
      this.selectedProfile = e.target.value;
      if (this.selectedProfile === 'live_scope') {
        if (this.liveScopeSamples && this.liveScopeSamples.length >= 2) {
          this.buildLivePlotData();
          this.renderCanvas();
        } else {
          this.renderEmptyGrid();
        }
      } else {
        this.generateDataAndRender();
      }
    });

    document.getElementById('matlab-sel-unit')?.addEventListener('change', (e) => {
      this.selectedUnit = e.target.value;
      this.renderCanvas();
    });

    // 4. Arm Subset Toggles (J1..J3 vs J4..J6)
    const btnJ123 = document.getElementById('btn-sub-j123');
    const btnJ456 = document.getElementById('btn-sub-j456');

    btnJ123?.addEventListener('click', () => {
      this.armJointGroup = 'j123';
      btnJ123.classList.add('active');
      btnJ456.classList.remove('active');
      this.renderCanvas();
    });

    btnJ456?.addEventListener('click', () => {
      this.armJointGroup = 'j456';
      btnJ456.classList.add('active');
      btnJ123.classList.remove('active');
      this.renderCanvas();
    });

    // 5. Action buttons
    document.getElementById('matlab-btn-clear-daq')?.addEventListener('click', () => {
      this.clearLiveScopeData();
    });

    document.getElementById('matlab-btn-refresh')?.addEventListener('click', () => {
      this.generateDataAndRender();
    });

    document.getElementById('matlab-btn-record')?.addEventListener('click', () => {
      this.toggleRecording();
    });

    document.getElementById('matlab-btn-export-png')?.addEventListener('click', () => {
      this.exportPNG();
    });

    document.getElementById('matlab-btn-copy-script')?.addEventListener('click', () => {
      this.copyMatlabScript();
    });

    document.getElementById('matlab-btn-export-csv')?.addEventListener('click', () => {
      this.exportCSV();
    });
  }

  open() {
    if (!this.container) this.initDOM();
    this.container.style.display = 'block';
    this.isOpen = true;
    this.resizeCanvas();

    if (this.selectedProfile === 'live_scope') {
      if (this.liveScopeSamples && this.liveScopeSamples.length >= 2) {
        this.buildLivePlotData();
        this.renderCanvas();
      } else {
        this.renderEmptyGrid();
        const statusInfo = document.getElementById('matlab-status-info');
        const statusCoords = document.getElementById('matlab-status-coords');
        if (statusInfo) {
          statusInfo.innerHTML = '<span style="color:#0284c7; font-weight:600;">📈 Sẵn sàng ghi nhận Live Scope. Hãy cho robot hoạt động để bắt đầu vẽ đồ thị.</span>';
        }
        if (statusCoords) {
          statusCoords.textContent = 'Time: 0.00s | 0 mẫu | Chờ chuyển động robot';
        }
      }
    } else {
      this.generateDataAndRender();
    }
  }

  close() {
    if (!this.container) return;
    this.container.style.display = 'none';
    this.isOpen = false; // Khi đóng: update() thoát ngay lập tức -> 0% CPU!
    this.isRobotCurrentlyMoving = false;
    if (this.isRecording) {
      this.toggleRecording();
    }
  }

  toggleMaximize() {
    this.isMaximized = !this.isMaximized;
    if (this.isMaximized) {
      this.container.classList.add('maximized');
    } else {
      this.container.classList.remove('maximized');
      this.container.style.top = '50px';
      this.container.style.left = '50%';
      this.container.style.transform = 'translateX(-50%)';
    }
    this.resizeCanvas();
    this.renderCanvas();
  }

  resizeCanvas() {
    if (!this.canvas) return;
    if (this.isMaximized) {
      const container = document.getElementById('matlab-canvas-container');
      if (container) {
        this.canvas.width = Math.max(1040, container.clientWidth - 20);
        this.canvas.height = Math.max(670, container.clientHeight - 20);
      }
    } else {
      if (this.selectedLayout === 'grid6x3') {
        this.canvas.width = 1320;
        this.canvas.height = 680;
      } else {
        this.canvas.width = 1040;
        this.canvas.height = 670;
      }
    }
  }

  toggleRecording() {
    const btn = document.getElementById('matlab-btn-record');
    const status = document.getElementById('matlab-status-info');

    if (!this.isRecording) {
      this.isRecording = true;
      this.recordStartTime = performance.now();
      this.recordedSamples = [];
      if (btn) {
        btn.innerHTML = '⏹ Dừng & Vẽ Đồ Thị';
        btn.classList.add('recording');
      }
      if (status) {
        status.innerHTML = '🔴 Đang ghi nhận chuyển động thực tế 6 khớp... Hãy điều khiển robot hoặc chạy chu trình!';
      }
    } else {
      this.isRecording = false;
      if (btn) {
        btn.innerHTML = '🔴 Ghi Live DAQ';
        btn.classList.remove('recording');
      }
      if (status) {
        status.innerHTML = `Đã ghi nhận ${this.recordedSamples.length} mẫu. Đang dựng đồ thị 6 bậc...`;
      }
      this.selectedProfile = 'live_daq';
      const sel = document.getElementById('matlab-sel-profile');
      if (sel) sel.value = 'live_daq';
      this.generateDataAndRender();
    }
  }

  /**
   * Phát hiện robot đang chuyển động hay đứng yên
   */
  checkRobotMovement() {
    if (this.selectedRobot === 'arm6dof') {
      if (this.app.isArmBusy) return true;
      if (this.app.isSingleCycleRunning) return true;
      if (this.app.autoDemoMode && this.app.systemPower) return true;
      if (this.arm6DOF) {
        const q = this.arm6DOF.getJointAngles();
        if (!this.lastArmAngles) {
          this.lastArmAngles = { ...q };
          return false;
        }
        const diff = Math.abs(q.s - this.lastArmAngles.s) +
                     Math.abs(q.l - this.lastArmAngles.l) +
                     Math.abs(q.u - this.lastArmAngles.u) +
                     Math.abs(q.r - this.lastArmAngles.r) +
                     Math.abs(q.b - this.lastArmAngles.b) +
                     Math.abs(q.t - this.lastArmAngles.t);
        this.lastArmAngles = { ...q };
        return diff > 0.04; // > 0.04 độ chuyển động trong 1 frame
      }
    } else if (this.selectedRobot === 'delta') {
      if (this.app.isDeltaBusy) return true;
      if (this.app.isSingleCycleRunning) return true;
      if (this.app.autoDemoMode && this.app.systemPower) return true;
      if (this.deltaRobot && this.deltaRobot.theta) {
        const th = this.deltaRobot.theta;
        if (!this.lastDeltaTheta) {
          this.lastDeltaTheta = [th[0] || 0, th[1] || 0, th[2] || 0];
          return false;
        }
        const diff = Math.abs((th[0] || 0) - this.lastDeltaTheta[0]) +
                     Math.abs((th[1] || 0) - this.lastDeltaTheta[1]) +
                     Math.abs((th[2] || 0) - this.lastDeltaTheta[2]);
        this.lastDeltaTheta = [th[0] || 0, th[1] || 0, th[2] || 0];
        return diff > 0.001; // rad
      }
    }
    return false;
  }

  /**
   * Ghi nhận mẫu góc khớp tại thời điểm t
   */
  recordCurrentJointSample(t) {
    if (this.selectedRobot === 'arm6dof' && this.arm6DOF) {
      const q = this.arm6DOF.getJointAngles();
      this.liveScopeSamples.push({
        t,
        q1: THREE.MathUtils.degToRad(q.s),
        q2: THREE.MathUtils.degToRad(q.l),
        q3: THREE.MathUtils.degToRad(q.u),
        q4: THREE.MathUtils.degToRad(q.r),
        q5: THREE.MathUtils.degToRad(q.b),
        q6: THREE.MathUtils.degToRad(q.t)
      });
    } else if (this.deltaRobot) {
      const th = this.deltaRobot.theta || [0, 0, 0];
      this.liveScopeSamples.push({
        t,
        q1: th[0] || 0,
        q2: th[1] || 0,
        q3: th[2] || 0,
        q4: 0, q5: 0, q6: 0
      });
    }

    // Giới hạn bộ đệm mẫu để không tràn RAM (tối đa 3000 điểm ~ 1.5 phút ghi liên tục)
    if (this.liveScopeSamples.length > 3000) {
      this.liveScopeSamples.shift();
    }
  }

  /**
   * Tạo cấu trúc dữ liệu đồ thị từ mảng mẫu sống
   */
  buildLivePlotData() {
    if (!this.liveScopeSamples || this.liveScopeSamples.length < 2) return;

    const time = this.liveScopeSamples.map(s => s.t);
    const q1 = this.liveScopeSamples.map(s => s.q1);
    const q2 = this.liveScopeSamples.map(s => s.q2);
    const q3 = this.liveScopeSamples.map(s => s.q3);
    const q4 = this.liveScopeSamples.map(s => s.q4 || 0);
    const q5 = this.liveScopeSamples.map(s => s.q5 || 0);
    const q6 = this.liveScopeSamples.map(s => s.q6 || 0);

    const q = [q1, q2, q3, q4, q5, q6];
    const v = q.map(series => this.numericalDerivative(time, series));
    const a = v.map(series => this.numericalDerivative(time, series));
    const T = Math.max(0.1, time[time.length - 1]);

    const isArm = (this.selectedRobot === 'arm6dof');
    this.plotData = {
      robotName: isArm ? 'Live Scope - 6-DOF Robot Arm' : 'Live Scope - Delta 3-DOF Robot',
      jointLabels: isArm ? [
        'Theta_1 (J1 - Base S)',
        'Theta_2 (J2 - Lower Arm L)',
        'Theta_3 (J3 - Upper Arm U)',
        'Theta_4 (J4 - Wrist Roll R)',
        'Theta_5 (J5 - Wrist Pitch B)',
        'Theta_6 (J6 - Wrist Twist T)'
      ] : [
        'Theta_1 (q7) - Active Arm 1',
        'Theta_2 (q10) - Active Arm 2',
        'Theta_3 (q11) - Active Arm 3',
        '-', '-', '-'
      ],
      shortLabels: isArm ? ['Theta_1', 'Theta_2', 'Theta_3', 'Theta_4', 'Theta_5', 'Theta_6']
                         : ['Theta_1', 'Theta_2', 'Theta_3', '-', '-', '-'],
      time,
      q,
      v,
      a,
      duration: T
    };
  }

  /**
   * Xóa toàn bộ dữ liệu ghi nhận và làm mới Live Scope
   */
  clearLiveScopeData() {
    this.liveScopeSamples = [];
    this.scopeElapsedTime = 0;
    this.lastSampleTime = 0;
    this.lastRedrawTime = 0;
    this.isRobotCurrentlyMoving = false;
    this.plotData = null;
    this.renderEmptyGrid("Đã làm mới dữ liệu. Hãy điều khiển hoặc chạy robot để bắt đầu ghi nhận.");
    const statusInfo = document.getElementById('matlab-status-info');
    const statusCoords = document.getElementById('matlab-status-coords');
    if (statusInfo) {
      statusInfo.innerHTML = '<span>Đã làm mới đồ thị Live Scope (t = 0.00s). Đang chờ chuyển động robot...</span>';
    }
    if (statusCoords) {
      statusCoords.textContent = 'Time: 0.00s | 0 mẫu | Chờ chuyển động';
    }
  }

  /**
   * Vẽ khung lưới chờ khi chưa có chuyển động (0% CPU)
   */
  renderEmptyGrid(infoMsg = '') {
    if (!this.ctx || !this.canvas) return;
    const ctx = this.ctx;
    const W = this.canvas.width;
    const H = this.canvas.height;

    ctx.fillStyle = '#e8e8e8';
    ctx.fillRect(0, 0, W, H);

    const marginLeft = 65;
    const marginRight = 30;
    const marginTop = 35;
    const marginBottom = 45;
    const gapX = 50;
    const gapY = 48;
    const subW = (W - marginLeft - marginRight - 2 * gapX) / 3;
    const subH = (H - marginTop - marginBottom - 2 * gapY) / 3;

    for (let c = 0; c < 3; c++) {
      for (let r = 0; r < 3; r++) {
        const x0 = marginLeft + c * (subW + gapX);
        const y0 = marginTop + r * (subH + gapY);

        ctx.fillStyle = '#ffffff';
        ctx.fillRect(x0, y0, subW, subH);
        ctx.strokeStyle = '#cccccc';
        ctx.lineWidth = 1;
        ctx.strokeRect(x0, y0, subW, subH);

        ctx.save();
        ctx.strokeStyle = '#f0f0f0';
        ctx.setLineDash([2, 2]);
        for (let i = 1; i < 4; i++) {
          ctx.beginPath();
          ctx.moveTo(x0 + (i / 4) * subW, y0);
          ctx.lineTo(x0 + (i / 4) * subW, y0 + subH);
          ctx.stroke();
          ctx.beginPath();
          ctx.moveTo(x0, y0 + (i / 4) * subH);
          ctx.lineTo(x0 + subW, y0 + (i / 4) * subH);
          ctx.stroke();
        }
        ctx.restore();
      }
    }

    const cardW = 580;
    const cardH = 110;
    const cardX = (W - cardW) / 2;
    const cardY = (H - cardH) / 2;

    ctx.save();
    ctx.fillStyle = 'rgba(255, 255, 255, 0.95)';
    ctx.strokeStyle = '#94a3b8';
    ctx.lineWidth = 1.5;
    ctx.shadowColor = 'rgba(0, 0, 0, 0.15)';
    ctx.shadowBlur = 12;
    if (ctx.roundRect) {
      ctx.beginPath();
      ctx.roundRect(cardX, cardY, cardW, cardH, 8);
      ctx.fill();
      ctx.stroke();
    } else {
      ctx.fillRect(cardX, cardY, cardW, cardH);
      ctx.strokeRect(cardX, cardY, cardW, cardH);
    }
    ctx.restore();

    ctx.fillStyle = '#0f172a';
    ctx.font = 'bold 14px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText('📈 ĐỒ THỊ ĐỘNG THỜI GIAN THỰC (LIVE SCOPE - 0% CPU)', W / 2, cardY + 34);

    ctx.fillStyle = '#475569';
    ctx.font = '12px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif';
    ctx.fillText('Robot hiện đang dừng. Đồ thị sẽ tự động bắt đầu ghi và vẽ ngay khi Robot chuyển động.', W / 2, cardY + 58);
    ctx.fillText('(Tự động ghi khi chạy Auto Demo, Pick & Place 1 chu trình, Chạy Job chữ T hoặc Jogging)', W / 2, cardY + 78);
    if (infoMsg) {
      ctx.fillStyle = '#0284c7';
      ctx.font = 'italic 11px sans-serif';
      ctx.fillText(infoMsg, W / 2, cardY + 98);
    }
  }

  /**
   * Continuous telemetry recording & Dynamic Live Scope in animation loop
   */
  update(delta) {
    // 1. Nếu cửa sổ chưa mở: THOÁT NGAY LẬP TỨC -> 0% CPU!
    if (!this.isOpen) return;

    // 2. Chế độ thủ công qua nút "🔴 Ghi Live DAQ"
    if (this.isRecording) {
      const t = (performance.now() - this.recordStartTime) / 1000.0;
      if (this.selectedRobot === 'arm6dof' && this.arm6DOF) {
        const q = this.arm6DOF.getJointAngles();
        this.recordedSamples.push({
          t,
          q1: THREE.MathUtils.degToRad(q.s),
          q2: THREE.MathUtils.degToRad(q.l),
          q3: THREE.MathUtils.degToRad(q.u),
          q4: THREE.MathUtils.degToRad(q.r),
          q5: THREE.MathUtils.degToRad(q.b),
          q6: THREE.MathUtils.degToRad(q.t)
        });
      } else if (this.deltaRobot) {
        this.recordedSamples.push({
          t,
          q1: this.deltaRobot.theta[0],
          q2: this.deltaRobot.theta[1],
          q3: this.deltaRobot.theta[2],
          q4: 0, q5: 0, q6: 0
        });
      }
      return;
    }

    // 3. Chế độ mặc định Live Scope: Tự động ghi khi robot chạy, tự động dừng khi robot dừng
    if (this.selectedProfile !== 'live_scope') return;

    const isMoving = this.checkRobotMovement();
    const now = performance.now();

    if (isMoving) {
      this.motionStoppedTimer = 0;
      this.isRobotCurrentlyMoving = true;

      // Tích lũy thời gian chuyển động thực tế
      this.scopeElapsedTime += Math.min(delta, 0.1);

      // Lấy mẫu khớp ở tần số ~30Hz (mỗi 33ms)
      if (now - this.lastSampleTime >= 33) {
        this.lastSampleTime = now;
        this.recordCurrentJointSample(this.scopeElapsedTime);
      }

      // Vẽ đồ thị mượt ở tần số ~15 FPS (mỗi 66ms) tránh tốn GPU
      if (now - this.lastRedrawTime >= 66 && this.liveScopeSamples.length >= 2) {
        this.lastRedrawTime = now;
        this.buildLivePlotData();
        this.renderCanvas();

        const statusInfo = document.getElementById('matlab-status-info');
        const statusCoords = document.getElementById('matlab-status-coords');
        if (statusInfo) {
          statusInfo.innerHTML = '<span style="color:#15803d; font-weight:700;">● ĐANG GHI ĐỒ THỊ THỜI GIAN THỰC (LIVE SCOPE): Robot đang chuyển động...</span>';
        }
        if (statusCoords) {
          statusCoords.textContent = `T = ${this.scopeElapsedTime.toFixed(2)}s | ${this.liveScopeSamples.length} mẫu | Trạng thái: RUNNING`;
        }
      }
    } else {
      // Robot đang dừng
      if (this.isRobotCurrentlyMoving) {
        this.motionStoppedTimer += delta;
        // Trễ 150ms để chắc chắn đã dừng hoàn toàn trước khi đóng băng
        if (this.motionStoppedTimer > 0.15) {
          this.isRobotCurrentlyMoving = false;
          // Vẽ lại 1 lần cuối chốt chính xác vị trí dừng
          if (this.liveScopeSamples.length >= 2) {
            this.buildLivePlotData();
            this.renderCanvas();
          }
          const statusInfo = document.getElementById('matlab-status-info');
          const statusCoords = document.getElementById('matlab-status-coords');
          if (statusInfo) {
            statusInfo.innerHTML = `<span style="color:#b45309; font-weight:700;">⏸ ROBOT DỪNG: Đồ thị đóng băng tại ${this.liveScopeSamples.length} mẫu (${this.scopeElapsedTime.toFixed(2)}s). 0% CPU. Sẵn sàng tiếp tục khi robot chuyển động lại.</span>`;
          }
          if (statusCoords) {
            statusCoords.textContent = `T = ${this.scopeElapsedTime.toFixed(2)}s | Đã dừng (Freeze) | 0% CPU`;
          }
        }
      }
    }
  }

  generateDataAndRender() {
    if (this.selectedProfile === 'live_scope') {
      if (this.liveScopeSamples && this.liveScopeSamples.length >= 2) {
        this.buildLivePlotData();
        this.renderCanvas();
      } else {
        this.renderEmptyGrid();
      }
      return;
    }

    if (this.selectedRobot === 'arm6dof') {
      if (this.selectedProfile === 'arm_cycle') {
        this.generate6DOFPickAndPlaceProfile();
      } else if (this.selectedProfile === 'letter_t') {
        this.generateLetterTProfile();
      } else if (this.selectedProfile === 'matlab_s_curve') {
        this.generate6DOFQuinticSCurveProfile();
      } else if (this.selectedProfile === 'live_daq') {
        this.generateLiveDaqProfile();
      }
    } else {
      // Delta robot profile
      this.generateDeltaProfile();
    }

    this.renderCanvas();
  }

  /**
   * Profile 1: Full 6-DOF Pick & Place Box Packing Trajectory (6 Joints moving concurrently)
   * Stages:
   * 0.0s - 0.5s: Descend to Pick (J2: 25->42 deg, J3: -15->18 deg, J5: -10->-60 deg)
   * 0.5s - 1.0s: Lift to Safe (J2: 42->18 deg, J3: 18->-22 deg, J5: -60->4 deg)
   * 1.0s - 2.0s: 180-deg Base Swing to Rear Box Conveyor (J1: 0->175 deg, J2: 18->22 deg, J3: -22->-18 deg)
   * 2.0s - 2.5s: Descend into Carton Box (J2: 22->46 deg, J3: -18->20 deg, J5: 4->-66 deg)
   * 2.5s - 2.8s: Retract to Standby (All joints return smoothly)
   */
  generate6DOFPickAndPlaceProfile() {
    const N = 120;
    const T = 2.8;
    const time = [];
    const q1 = [], q2 = [], q3 = [], q4 = [], q5 = [], q6 = [];

    // Joint keyframes in degrees { s, l, u, r, b, t }
    const keyframes = [
      { t: 0.0,  s: 0.0,   l: 25.0,  u: -15.0, r: 0.0,  b: -10.0, tAxis: 0.0 },  // Standby
      { t: 0.5,  s: 0.0,   l: 42.0,  u: 18.0,  r: 0.0,  b: -60.0, tAxis: 0.0 },  // Pick Grip
      { t: 1.0,  s: 0.0,   l: 18.0,  u: -22.0, r: 0.0,  b: 4.0,   tAxis: 0.0 },  // Lift Safe
      { t: 2.0,  s: 175.0, l: 22.0,  u: -18.0, r: 5.0,  b: -4.0,  tAxis: 0.0 },  // Swing 180°
      { t: 2.5,  s: 175.0, l: 46.0,  u: 20.0,  r: 0.0,  b: -66.0, tAxis: 0.0 },  // Place in Box
      { t: 2.8,  s: 0.0,   l: 25.0,  u: -15.0, r: 0.0,  b: -10.0, tAxis: 0.0 }   // Return
    ];

    for (let i = 0; i <= N; i++) {
      const curT = (i / N) * T;
      time.push(curT);

      // Find active segment
      let seg = 0;
      while (seg < keyframes.length - 2 && keyframes[seg + 1].t < curT) {
        seg++;
      }

      const k0 = keyframes[seg];
      const k1 = keyframes[seg + 1];
      const segDur = Math.max(0.01, k1.t - k0.t);
      const tau = Math.max(0, Math.min(1, (curT - k0.t) / segDur));

      // Quintic polynomial smoothing s(tau) = 10*tau^3 - 15*tau^4 + 6*tau^5
      const s = tau * tau * tau * (10 - 15 * tau + 6 * tau * tau);

      const d2r = Math.PI / 180.0;
      q1.push((k0.s + (k1.s - k0.s) * s) * d2r);
      q2.push((k0.l + (k1.l - k0.l) * s) * d2r);
      q3.push((k0.u + (k1.u - k0.u) * s) * d2r);
      q4.push((k0.r + (k1.r - k0.r) * s) * d2r);
      q5.push((k0.b + (k1.b - k0.b) * s) * d2r);
      q6.push((k0.tAxis + (k1.tAxis - k0.tAxis) * s) * d2r);
    }

    // Compute numerical derivatives for all 6 joints
    const v = [
      this.numericalDerivative(time, q1),
      this.numericalDerivative(time, q2),
      this.numericalDerivative(time, q3),
      this.numericalDerivative(time, q4),
      this.numericalDerivative(time, q5),
      this.numericalDerivative(time, q6)
    ];

    const a = [
      this.numericalDerivative(time, v[0]),
      this.numericalDerivative(time, v[1]),
      this.numericalDerivative(time, v[2]),
      this.numericalDerivative(time, v[3]),
      this.numericalDerivative(time, v[4]),
      this.numericalDerivative(time, v[5])
    ];

    this.plotData = {
      robotName: 'Yaskawa GP8 6-DOF Robot Arm',
      jointLabels: [
        'Theta_1 (J1 - Base S)',
        'Theta_2 (J2 - Lower Arm L)',
        'Theta_3 (J3 - Upper Arm U)',
        'Theta_4 (J4 - Wrist Roll R)',
        'Theta_5 (J5 - Wrist Pitch B)',
        'Theta_6 (J6 - Wrist Twist T)'
      ],
      shortLabels: ['Theta_1', 'Theta_2', 'Theta_3', 'Theta_4', 'Theta_5', 'Theta_6'],
      time,
      q: [q1, q2, q3, q4, q5, q6],
      v,
      a,
      duration: T
    };

    const statusCoord = document.getElementById('matlab-status-coords');
    if (statusCoord) {
      statusCoord.textContent = `Robot: 6-DOF Arm | Profile: Full Box Packing Cycle | T = ${T}s | 120 points`;
    }
  }

  /**
   * Profile: Quỹ đạo Vẽ Chữ T (Letter T Trajectory - Đồ thị Chữ T)
   * Keyframes tương ứng với các nét vẽ chữ T:
   * 0.0s: P020 - Tiếp cận trên cao (-1.25, 1.30, 0.18)
   * 0.8s: P021 - Đầu nét ngang trái (-1.45, 1.20, 0.18)
   * 1.8s: P022 - Kéo nét ngang sang phải (-1.05, 1.20, 0.18) [Nét ngang 400mm]
   * 2.4s: P023 - Trượt về tâm nét ngang (-1.25, 1.20, 0.18)
   * 3.2s: P024 - Kéo nét đứng thẳng xuống (-1.25, 0.88, 0.18) [Nét đứng 320mm]
   * 3.6s: Dwell chiêm ngưỡng chữ T
   * 4.2s: P020 - Rút về vị trí an toàn hoàn tất
   */
  generateLetterTProfile() {
    const N = 120;
    const T = 4.2;
    const time = [];
    const q1 = [], q2 = [], q3 = [], q4 = [], q5 = [], q6 = [];

    const keyframes = [
      { t: 0.0, s: -90.0,  l: -7.7,  u: -0.5,  r: 0.0, b: -81.9, tAxis: 0.0 }, // Safe P020
      { t: 0.8, s: -102.2, l: -10.8, u: -3.8,  r: 0.0, b: -75.4, tAxis: 0.0 }, // Bar Left P021
      { t: 1.8, s: -77.9,  l: -10.8, u: -3.8,  r: 0.0, b: -75.4, tAxis: 0.0 }, // Bar Right P022
      { t: 2.4, s: -90.0,  l: -9.2,  u: -5.7,  r: 0.0, b: -75.1, tAxis: 0.0 }, // Bar Center P023
      { t: 3.2, s: -90.0,  l: -24.7, u: -16.4, r: 0.0, b: -48.9, tAxis: 0.0 }, // Stem Bottom P024
      { t: 3.6, s: -90.0,  l: -24.7, u: -16.4, r: 0.0, b: -48.9, tAxis: 0.0 }, // Dwell
      { t: 4.2, s: -90.0,  l: -7.7,  u: -0.5,  r: 0.0, b: -81.9, tAxis: 0.0 }  // Safe P020
    ];

    for (let i = 0; i <= N; i++) {
      const curT = (i / N) * T;
      time.push(curT);

      let seg = 0;
      while (seg < keyframes.length - 2 && keyframes[seg + 1].t < curT) {
        seg++;
      }

      const k0 = keyframes[seg];
      const k1 = keyframes[seg + 1];
      const segDur = Math.max(0.01, k1.t - k0.t);
      const tau = Math.max(0, Math.min(1, (curT - k0.t) / segDur));

      // Quintic polynomial smoothing
      const s = tau * tau * tau * (10 - 15 * tau + 6 * tau * tau);
      const d2r = Math.PI / 180.0;

      q1.push((k0.s + (k1.s - k0.s) * s) * d2r);
      q2.push((k0.l + (k1.l - k0.l) * s) * d2r);
      q3.push((k0.u + (k1.u - k0.u) * s) * d2r);
      q4.push((k0.r + (k1.r - k0.r) * s) * d2r);
      q5.push((k0.b + (k1.b - k0.b) * s) * d2r);
      q6.push((k0.tAxis + (k1.tAxis - k0.tAxis) * s) * d2r);
    }

    const v = [
      this.numericalDerivative(time, q1),
      this.numericalDerivative(time, q2),
      this.numericalDerivative(time, q3),
      this.numericalDerivative(time, q4),
      this.numericalDerivative(time, q5),
      this.numericalDerivative(time, q6)
    ];

    const a = [
      this.numericalDerivative(time, v[0]),
      this.numericalDerivative(time, v[1]),
      this.numericalDerivative(time, v[2]),
      this.numericalDerivative(time, v[3]),
      this.numericalDerivative(time, v[4]),
      this.numericalDerivative(time, v[5])
    ];

    this.plotData = {
      robotName: 'Yaskawa GP8 6-DOF Robot Arm (Quỹ đạo Chữ T)',
      jointLabels: [
        'Theta_1 (J1 - Base S)',
        'Theta_2 (J2 - Lower Arm L)',
        'Theta_3 (J3 - Upper Arm U)',
        'Theta_4 (J4 - Wrist Roll R)',
        'Theta_5 (J5 - Wrist Pitch B)',
        'Theta_6 (J6 - Wrist Twist T)'
      ],
      shortLabels: ['Theta_1', 'Theta_2', 'Theta_3', 'Theta_4', 'Theta_5', 'Theta_6'],
      time,
      q: [q1, q2, q3, q4, q5, q6],
      v,
      a,
      duration: T
    };

    const statusCoord = document.getElementById('matlab-status-coords');
    if (statusCoord) {
      statusCoord.textContent = `Profile: Quỹ đạo Vẽ Chữ T (Letter T) | T = ${T}s | Nét ngang: 400mm, Nét đứng: 320mm | 120 pts`;
    }
  }

  /**
   * Profile 2: Analytical 5th-Order S-Curve for 6-DOF Arm (2.2s Segment matching template)
   */
  generate6DOFQuinticSCurveProfile() {
    const N = 100;
    const T = 2.2;
    const time = [];
    const q = [[], [], [], [], [], []];
    const v = [[], [], [], [], [], []];
    const a = [[], [], [], [], [], []];

    // Initial and final joint coordinates in radians for 6 joints
    const q0 = [-0.174,  0.350, -0.260,  0.000, -0.520,  0.000];
    const qf = [ 0.785, -0.280,  0.350,  0.520,  0.260, -0.785];

    for (let i = 0; i <= N; i++) {
      const t = (i / N) * T;
      const tau = t / T;

      const s = tau * tau * tau * (10 - 15 * tau + 6 * tau * tau);
      const ds = (1.0 / T) * (30 * tau * tau - 60 * tau * tau * tau + 30 * tau * tau * tau * tau);
      const d2s = (1.0 / (T * T)) * (60 * tau - 180 * tau * tau + 120 * tau * tau * tau);

      time.push(t);

      for (let j = 0; j < 6; j++) {
        const deltaQ = qf[j] - q0[j];
        q[j].push(q0[j] + deltaQ * s);
        v[j].push(deltaQ * ds);
        a[j].push(deltaQ * d2s);
      }
    }

    this.plotData = {
      robotName: 'Yaskawa GP8 6-DOF Robot Arm',
      jointLabels: [
        'Theta_1 (J1 - Base S)',
        'Theta_2 (J2 - Lower Arm L)',
        'Theta_3 (J3 - Upper Arm U)',
        'Theta_4 (J4 - Wrist Roll R)',
        'Theta_5 (J5 - Wrist Pitch B)',
        'Theta_6 (J6 - Wrist Twist T)'
      ],
      shortLabels: ['Theta_1', 'Theta_2', 'Theta_3', 'Theta_4', 'Theta_5', 'Theta_6'],
      time,
      q,
      v,
      a,
      duration: T
    };

    const statusCoord = document.getElementById('matlab-status-coords');
    if (statusCoord) {
      statusCoord.textContent = `Robot: 6-DOF Arm | Profile: Analytical Quintic S-Curve | T = ${T}s | 100 points`;
    }
  }

  /**
   * Profile 3: Live DAQ Recorded Data from Robot Execution
   */
  generateLiveDaqProfile() {
    if (!this.recordedSamples || this.recordedSamples.length < 5) {
      alert('Chưa có đủ mẫu ghi nhận! Hãy bấm [🔴 Ghi Live DAQ], điều khiển robot rồi bấm Dừng.');
      this.generate6DOFPickAndPlaceProfile();
      return;
    }

    const time = this.recordedSamples.map(s => s.t);
    const q1 = this.recordedSamples.map(s => s.q1);
    const q2 = this.recordedSamples.map(s => s.q2);
    const q3 = this.recordedSamples.map(s => s.q3);
    const q4 = this.recordedSamples.map(s => s.q4 || 0);
    const q5 = this.recordedSamples.map(s => s.q5 || 0);
    const q6 = this.recordedSamples.map(s => s.q6 || 0);

    const q = [q1, q2, q3, q4, q5, q6];
    const v = q.map(series => this.numericalDerivative(time, series));
    const a = v.map(series => this.numericalDerivative(time, series));

    const T = time[time.length - 1];

    this.plotData = {
      robotName: 'Live DAQ - 6-DOF Robot Arm',
      jointLabels: [
        'Theta_1 (J1 - Base S)',
        'Theta_2 (J2 - Lower Arm L)',
        'Theta_3 (J3 - Upper Arm U)',
        'Theta_4 (J4 - Wrist Roll R)',
        'Theta_5 (J5 - Wrist Pitch B)',
        'Theta_6 (J6 - Wrist Twist T)'
      ],
      shortLabels: ['Theta_1', 'Theta_2', 'Theta_3', 'Theta_4', 'Theta_5', 'Theta_6'],
      time,
      q,
      v,
      a,
      duration: T
    };

    const statusCoord = document.getElementById('matlab-status-coords');
    if (statusCoord) {
      statusCoord.textContent = `Profile: Live DAQ Recording | T = ${T.toFixed(2)}s | ${time.length} samples`;
    }
  }

  /**
   * Delta Robot Profile fallback
   */
  generateDeltaProfile() {
    const N = 100;
    const T = 2.2;
    const time = [];
    const q1 = [], q2 = [], q3 = [];
    const v1 = [], v2 = [], v3 = [];
    const a1 = [], a2 = [], a3 = [];

    const q1_0 = -0.174, q1_f = -0.1395;
    const q2_0 = 0.350,  q2_f = 0.2795;
    const q3_0 = -0.174, q3_f = -0.1395;

    for (let i = 0; i <= N; i++) {
      const t = (i / N) * T;
      const tau = t / T;
      const s = tau * tau * tau * (10 - 15 * tau + 6 * tau * tau);
      const ds = (1.0 / T) * (30 * tau * tau - 60 * tau * tau * tau + 30 * tau * tau * tau * tau);
      const d2s = (1.0 / (T * T)) * (60 * tau - 180 * tau * tau + 120 * tau * tau * tau);

      time.push(t);
      q1.push(q1_0 + (q1_f - q1_0) * s);
      v1.push((q1_f - q1_0) * ds);
      a1.push((q1_f - q1_0) * d2s);

      q2.push(q2_0 + (q2_f - q2_0) * s);
      v2.push((q2_f - q2_0) * ds);
      a2.push((q2_f - q2_0) * d2s);

      q3.push(q3_0 + (q3_f - q3_0) * s);
      v3.push((q3_f - q3_0) * ds);
      a3.push((q3_f - q3_0) * d2s);
    }

    const zeros = new Array(time.length).fill(0);
    this.plotData = {
      robotName: 'Delta 3-DOF Robot',
      jointLabels: [
        'Theta_1 (q7) - Active Arm 1',
        'Theta_2 (q10) - Active Arm 2',
        'Theta_3 (q11) - Active Arm 3'
      ],
      shortLabels: ['Theta_1', 'Theta_2', 'Theta_3'],
      time,
      q: [q1, q2, q3, zeros, zeros, zeros],
      v: [v1, v2, v3, zeros, zeros, zeros],
      a: [a1, a2, a3, zeros, zeros, zeros],
      duration: T
    };
  }

  /**
   * Numerical derivative with 3-point smoothing
   */
  numericalDerivative(tArr, xArr) {
    const n = tArr.length;
    const deriv = new Array(n).fill(0);

    for (let i = 0; i < n; i++) {
      if (i === 0) {
        deriv[i] = (xArr[1] - xArr[0]) / Math.max(0.001, tArr[1] - tArr[0]);
      } else if (i === n - 1) {
        deriv[i] = (xArr[n - 1] - xArr[n - 2]) / Math.max(0.001, tArr[n - 1] - tArr[n - 2]);
      } else {
        const dt = Math.max(0.002, tArr[i + 1] - tArr[i - 1]);
        deriv[i] = (xArr[i + 1] - xArr[i - 1]) / dt;
      }
    }

    const smoothed = new Array(n);
    for (let i = 0; i < n; i++) {
      if (i === 0 || i === n - 1) {
        smoothed[i] = deriv[i];
      } else {
        smoothed[i] = (deriv[i - 1] + 2 * deriv[i] + deriv[i + 1]) / 4.0;
      }
    }
    return smoothed;
  }

  /**
   * Main Render Dispatcher
   */
  renderCanvas() {
    if (!this.ctx || !this.canvas || !this.plotData) return;

    if (this.selectedLayout === 'overview3') {
      this.renderOverviewLayout();
    } else if (this.selectedLayout === 'grid6x3') {
      this.renderFull6x3Layout();
    } else {
      this.renderGrid3x3Layout();
    }
  }

  /**
   * Mode 1: Classic 3x3 Subplots (matching user's uploaded MATLAB template image)
   * Shows either J1..J3 or J4..J6
   */
  renderGrid3x3Layout() {
    const ctx = this.ctx;
    const W = this.canvas.width;
    const H = this.canvas.height;

    ctx.fillStyle = '#e8e8e8';
    ctx.fillRect(0, 0, W, H);

    const isDeg = (this.selectedUnit === 'deg');
    const rad2deg = 180.0 / Math.PI;

    const marginTop = 35;
    const marginBottom = 45;
    const marginLeft = 65;
    const marginRight = 30;
    const gapX = 50;
    const gapY = 48;

    const subW = (W - marginLeft - marginRight - 2 * gapX) / 3;
    const subH = (H - marginTop - marginBottom - 2 * gapY) / 3;

    const data = this.plotData;
    const time = data.time;

    // Joint indices offset: 0..2 for J1..J3 or 3..5 for J4..J6
    const offset = (this.selectedRobot === 'arm6dof' && this.armJointGroup === 'j456') ? 3 : 0;

    const rowConfigs = [
      {
        rowIdx: 0,
        dataMatrix: data.q,
        unitLabel: isDeg ? '(deg)' : '(rad)',
        color: '#0020aa', // MATLAB Blue
        multiplier: isDeg ? rad2deg : 1.0,
        titlePrefix: '',
        yPrefix: 'q_'
      },
      {
        rowIdx: 1,
        dataMatrix: data.v,
        unitLabel: isDeg ? '(deg/s)' : '(rad/s)',
        color: '#cc0000', // MATLAB Red
        multiplier: isDeg ? rad2deg : 1.0,
        titlePrefix: 'Velocity of ',
        yPrefix: 'v_'
      },
      {
        rowIdx: 2,
        dataMatrix: data.a,
        unitLabel: isDeg ? '(deg/s²)' : '(rad/s²)',
        color: '#008020', // MATLAB Green
        multiplier: isDeg ? rad2deg : 1.0,
        titlePrefix: 'Acceleration of ',
        yPrefix: 'a_'
      }
    ];

    for (let c = 0; c < 3; c++) {
      const jointIdx = offset + c;
      const jointTitle = data.jointLabels[jointIdx] || `Joint ${jointIdx + 1}`;
      const thetaKey = `theta_${jointIdx + 1}`;

      for (let r = 0; r < 3; r++) {
        const rowCfg = rowConfigs[r];
        const rawSeries = rowCfg.dataMatrix[jointIdx] || [];
        const scaledSeries = rawSeries.map(val => val * rowCfg.multiplier);

        const x0 = marginLeft + c * (subW + gapX);
        const y0 = marginTop + r * (subH + gapY);

        let subplotTitle = '';
        if (r === 0) subplotTitle = jointTitle;
        else if (r === 1) subplotTitle = `Velocity of ${thetaKey}`;
        else subplotTitle = `Acceleration of ${thetaKey}`;

        const yLabel = `${rowCfg.yPrefix}${jointIdx + 1} ${rowCfg.unitLabel}`;

        this.drawSubplot(ctx, {
          x: x0,
          y: y0,
          w: subW,
          h: subH,
          time,
          values: scaledSeries,
          title: subplotTitle,
          yLabel,
          xLabel: 'Time (s)',
          color: rowCfg.color
        });
      }
    }
  }

  /**
   * Mode 2: Synoptic Overview (3 large plots, all 6 joints overlaid with colors & legend)
   */
  renderOverviewLayout() {
    const ctx = this.ctx;
    const W = this.canvas.width;
    const H = this.canvas.height;

    ctx.fillStyle = '#e8e8e8';
    ctx.fillRect(0, 0, W, H);

    const isDeg = (this.selectedUnit === 'deg');
    const rad2deg = 180.0 / Math.PI;

    const marginTop = 35;
    const marginBottom = 45;
    const marginLeft = 75;
    const marginRight = 150; // Space for Legend on the right
    const gapY = 40;

    const plotW = W - marginLeft - marginRight;
    const plotH = (H - marginTop - marginBottom - 2 * gapY) / 3;

    const data = this.plotData;
    const time = data.time;

    const jointColors = [
      '#0284c7', // J1 Cyan/Blue
      '#16a34a', // J2 Green
      '#d97706', // J3 Amber
      '#9333ea', // J4 Purple
      '#e11d48', // J5 Rose
      '#0d9488'  // J6 Teal
    ];

    const rows = [
      {
        matrix: data.q,
        title: '6-DOF JOINT ANGLES // VỊ TRÍ GÓC (THETA_1 .. THETA_6)',
        yLabel: isDeg ? 'Position (deg)' : 'Position (rad)',
        multiplier: isDeg ? rad2deg : 1.0
      },
      {
        matrix: data.v,
        title: '6-DOF JOINT VELOCITIES // VẬN TỐC GÓC (V_1 .. V_6)',
        yLabel: isDeg ? 'Velocity (deg/s)' : 'Velocity (rad/s)',
        multiplier: isDeg ? rad2deg : 1.0
      },
      {
        matrix: data.a,
        title: '6-DOF JOINT ACCELERATIONS // GIA TỐC GÓC (A_1 .. A_6)',
        yLabel: isDeg ? 'Acceleration (deg/s²)' : 'Acceleration (rad/s²)',
        multiplier: isDeg ? rad2deg : 1.0
      }
    ];

    rows.forEach((row, rIdx) => {
      const y0 = marginTop + rIdx * (plotH + gapY);
      this.drawMultiLinePlot(ctx, {
        x: marginLeft,
        y: y0,
        w: plotW,
        h: plotH,
        time,
        seriesList: row.matrix.slice(0, 6).map(s => s.map(v => v * row.multiplier)),
        colors: jointColors,
        labels: data.shortLabels.slice(0, 6),
        title: row.title,
        yLabel: row.yLabel,
        xLabel: (rIdx === 2) ? 'Time (s)' : ''
      });
    });

    // Draw Master Legend on Right Side
    this.drawLegend(ctx, W - marginRight + 15, marginTop + 20, jointColors, data.jointLabels.slice(0, 6));
  }

  /**
   * Mode 3: Full 6x3 Matrix (6 columns for all 6 joints, 3 rows for Position, Vel, Acc)
   */
  renderFull6x3Layout() {
    const ctx = this.ctx;
    const W = this.canvas.width;
    const H = this.canvas.height;

    ctx.fillStyle = '#e8e8e8';
    ctx.fillRect(0, 0, W, H);

    const isDeg = (this.selectedUnit === 'deg');
    const rad2deg = 180.0 / Math.PI;

    const marginTop = 35;
    const marginBottom = 45;
    const marginLeft = 60;
    const marginRight = 25;
    const gapX = 28;
    const gapY = 45;

    const subW = (W - marginLeft - marginRight - 5 * gapX) / 6;
    const subH = (H - marginTop - marginBottom - 2 * gapY) / 3;

    const data = this.plotData;
    const time = data.time;

    const rowConfigs = [
      { matrix: data.q, color: '#0020aa', yPrefix: 'q_', unit: isDeg ? '(deg)' : '(rad)' },
      { matrix: data.v, color: '#cc0000', yPrefix: 'v_', unit: isDeg ? '(deg/s)' : '(rad/s)' },
      { matrix: data.a, color: '#008020', yPrefix: 'a_', unit: isDeg ? '(deg/s²)' : '(rad/s²)' }
    ];

    for (let c = 0; c < 6; c++) {
      const jointTitle = data.shortLabels[c] || `J${c + 1}`;

      for (let r = 0; r < 3; r++) {
        const rowCfg = rowConfigs[r];
        const rawSeries = rowCfg.matrix[c] || [];
        const mult = isDeg ? rad2deg : 1.0;
        const scaledSeries = rawSeries.map(val => val * mult);

        const x0 = marginLeft + c * (subW + gapX);
        const y0 = marginTop + r * (subH + gapY);

        let title = '';
        if (r === 0) title = jointTitle;
        else if (r === 1) title = `Vel ${jointTitle}`;
        else title = `Acc ${jointTitle}`;

        this.drawSubplot(ctx, {
          x: x0,
          y: y0,
          w: subW,
          h: subH,
          time,
          values: scaledSeries,
          title,
          yLabel: `${rowCfg.yPrefix}${c + 1}`,
          xLabel: 'Time (s)',
          color: rowCfg.color
        });
      }
    }
  }

  /**
   * Draw a single MATLAB-style Subplot
   */
  drawSubplot(ctx, opt) {
    const { x, y, w, h, time, values, title, yLabel, xLabel, color } = opt;

    ctx.fillStyle = '#ffffff';
    ctx.fillRect(x, y, w, h);

    ctx.strokeStyle = '#8c8c8c';
    ctx.lineWidth = 1;
    ctx.strokeRect(x, y, w, h);

    let minVal = Math.min(...values);
    let maxVal = Math.max(...values);
    let span = maxVal - minVal;

    if (span < 1e-4) {
      span = 0.1;
      minVal -= 0.05;
      maxVal += 0.05;
    } else {
      minVal -= span * 0.08;
      maxVal += span * 0.08;
      span = maxVal - minVal;
    }

    const tMin = 0;
    const tMax = time[time.length - 1] || 2.2;
    const tSpan = Math.max(0.1, tMax - tMin);

    // Grid Lines
    ctx.save();
    ctx.strokeStyle = '#e2e2e2';
    ctx.lineWidth = 1;
    ctx.setLineDash([2, 2]);

    const tDivs = 4;
    for (let i = 1; i < tDivs; i++) {
      const gx = x + (i / tDivs) * w;
      ctx.beginPath();
      ctx.moveTo(gx, y);
      ctx.lineTo(gx, y + h);
      ctx.stroke();
    }

    const yDivs = 4;
    for (let i = 1; i < yDivs; i++) {
      const gy = y + (i / yDivs) * h;
      ctx.beginPath();
      ctx.moveTo(x, gy);
      ctx.lineTo(x + w, gy);
      ctx.stroke();
    }
    ctx.restore();

    // Zero baseline
    if (minVal < 0 && maxVal > 0) {
      const zeroY = y + h - ((0 - minVal) / span) * h;
      ctx.strokeStyle = '#cccccc';
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(x, zeroY);
      ctx.lineTo(x + w, zeroY);
      ctx.stroke();
    }

    // Trajectory curve (Đậm nét chuẩn MATLAB publication)
    ctx.strokeStyle = color;
    ctx.lineWidth = 2.8;
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';
    ctx.beginPath();

    const n = time.length;
    for (let i = 0; i < n; i++) {
      const px = x + ((time[i] - tMin) / tSpan) * w;
      const py = y + h - ((values[i] - minVal) / span) * h;
      if (i === 0) ctx.moveTo(px, py);
      else ctx.lineTo(px, py);
    }
    ctx.stroke();

    // Subplot title
    ctx.fillStyle = '#000000';
    ctx.font = 'bold 11px Arial, Helvetica, sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText(title, x + w / 2, y - 6);

    // Y-Axis Ticks
    ctx.fillStyle = '#111111';
    ctx.font = '9.5px Arial, Helvetica, sans-serif';
    ctx.textAlign = 'right';

    const nYTicks = 4;
    for (let k = 0; k < nYTicks; k++) {
      const val = minVal + (k / (nYTicks - 1)) * span;
      const ty = y + h - ((val - minVal) / span) * h;
      ctx.strokeStyle = '#666';
      ctx.beginPath();
      ctx.moveTo(x, ty);
      ctx.lineTo(x - 3, ty);
      ctx.stroke();

      let str = Math.abs(val) < 1e-4 ? '0' : val.toFixed(2);
      if (span < 0.12 && Math.abs(val) >= 1e-4) str = val.toFixed(3);
      ctx.fillText(str, x - 5, ty + 3);
    }

    // Y-Axis title
    ctx.save();
    ctx.translate(x - 36, y + h / 2);
    ctx.rotate(-Math.PI / 2);
    ctx.textAlign = 'center';
    ctx.font = '10.5px Arial, Helvetica, sans-serif';
    ctx.fillText(yLabel, 0, 0);
    ctx.restore();

    // X-Axis Ticks
    ctx.textAlign = 'center';
    ctx.font = '9px Arial, Helvetica, sans-serif';

    const step = tMax <= 3.0 ? 0.5 : 1.0;
    for (let tv = 0; tv <= tMax + 0.05; tv += step) {
      const tx = x + ((tv - tMin) / tSpan) * w;
      ctx.strokeStyle = '#666';
      ctx.beginPath();
      ctx.moveTo(tx, y + h);
      ctx.lineTo(tx, y + h + 3);
      ctx.stroke();
      ctx.fillText(tv.toFixed(1), tx, y + h + 13);
    }
  }

  /**
   * Draw Multi-Line Synoptic Plot
   */
  drawMultiLinePlot(ctx, opt) {
    const { x, y, w, h, time, seriesList, colors, title, yLabel, xLabel } = opt;

    ctx.fillStyle = '#ffffff';
    ctx.fillRect(x, y, w, h);
    ctx.strokeStyle = '#8c8c8c';
    ctx.lineWidth = 1;
    ctx.strokeRect(x, y, w, h);

    // Global min/max across all 6 series
    let minVal = Infinity;
    let maxVal = -Infinity;
    seriesList.forEach(s => {
      s.forEach(v => {
        if (v < minVal) minVal = v;
        if (v > maxVal) maxVal = v;
      });
    });

    let span = maxVal - minVal;
    if (span < 1e-4) {
      span = 0.2;
      minVal -= 0.1;
      maxVal += 0.1;
    } else {
      minVal -= span * 0.08;
      maxVal += span * 0.08;
      span = maxVal - minVal;
    }

    const tMin = 0;
    const tMax = time[time.length - 1] || 2.8;
    const tSpan = Math.max(0.1, tMax - tMin);

    // Grid Lines
    ctx.save();
    ctx.strokeStyle = '#e2e2e2';
    ctx.lineWidth = 1;
    ctx.setLineDash([2, 2]);
    for (let i = 1; i < 6; i++) {
      const gx = x + (i / 6) * w;
      ctx.beginPath();
      ctx.moveTo(gx, y);
      ctx.lineTo(gx, y + h);
      ctx.stroke();
    }
    for (let i = 1; i < 4; i++) {
      const gy = y + (i / 4) * h;
      ctx.beginPath();
      ctx.moveTo(x, gy);
      ctx.lineTo(x + w, gy);
      ctx.stroke();
    }
    ctx.restore();

    // Zero line
    if (minVal < 0 && maxVal > 0) {
      const zeroY = y + h - ((0 - minVal) / span) * h;
      ctx.strokeStyle = '#cbd5e1';
      ctx.beginPath();
      ctx.moveTo(x, zeroY);
      ctx.lineTo(x + w, zeroY);
      ctx.stroke();
    }

    // Draw all 6 curves (Đậm nét chuẩn MATLAB publication)
    seriesList.forEach((series, sIdx) => {
      ctx.strokeStyle = colors[sIdx];
      ctx.lineWidth = 2.8;
      ctx.lineCap = 'round';
      ctx.lineJoin = 'round';
      ctx.beginPath();

      for (let i = 0; i < time.length; i++) {
        const px = x + ((time[i] - tMin) / tSpan) * w;
        const py = y + h - ((series[i] - minVal) / span) * h;
        if (i === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      }
      ctx.stroke();
    });

    // Subplot title
    ctx.fillStyle = '#000000';
    ctx.font = 'bold 11px Arial, Helvetica, sans-serif';
    ctx.textAlign = 'left';
    ctx.fillText(title, x + 6, y - 6);

    // Y ticks
    ctx.font = '9.5px Arial, Helvetica, sans-serif';
    ctx.textAlign = 'right';
    for (let k = 0; k < 5; k++) {
      const val = minVal + (k / 4) * span;
      const ty = y + h - ((val - minVal) / span) * h;
      ctx.fillText(val.toFixed(2), x - 6, ty + 3);
    }

    // Y Axis label
    ctx.save();
    ctx.translate(x - 42, y + h / 2);
    ctx.rotate(-Math.PI / 2);
    ctx.textAlign = 'center';
    ctx.font = '10.5px Arial, Helvetica, sans-serif';
    ctx.fillText(yLabel, 0, 0);
    ctx.restore();

    // X Axis ticks
    if (xLabel) {
      ctx.textAlign = 'center';
      ctx.font = '10px Arial, Helvetica, sans-serif';
      const step = 0.5;
      for (let tv = 0; tv <= tMax + 0.05; tv += step) {
        const tx = x + ((tv - tMin) / tSpan) * w;
        ctx.fillText(tv.toFixed(1), tx, y + h + 14);
      }
      ctx.font = '11px Arial, Helvetica, sans-serif';
      ctx.fillText(xLabel, x + w / 2, y + h + 28);
    }
  }

  /**
   * Draw Legend for 6 Joints
   */
  drawLegend(ctx, x, y, colors, labels) {
    ctx.fillStyle = '#f8fafc';
    ctx.fillRect(x, y, 130, labels.length * 20 + 16);
    ctx.strokeStyle = '#cbd5e1';
    ctx.strokeRect(x, y, 130, labels.length * 20 + 16);

    ctx.font = 'bold 10px Arial, Helvetica, sans-serif';
    ctx.fillStyle = '#1e293b';
    ctx.textAlign = 'left';
    ctx.fillText('CHÚ THÍCH (LEGEND)', x + 8, y + 14);

    labels.forEach((lbl, idx) => {
      const itemY = y + 30 + idx * 20;
      ctx.fillStyle = colors[idx];
      ctx.fillRect(x + 8, itemY - 7, 14, 8);

      ctx.fillStyle = '#334155';
      ctx.font = '9.5px Arial, Helvetica, sans-serif';
      ctx.fillText(`J${idx + 1}: ${lbl.split('(')[0].trim()}`, x + 28, itemY);
    });
  }

  /**
   * Export High-Res PNG
   */
  exportPNG() {
    if (!this.canvas) return;
    const link = document.createElement('a');
    link.download = `MATLAB_6DOF_Robot_Kinematics_${Date.now()}.png`;
    link.href = this.canvas.toDataURL('image/png', 1.0);
    link.click();
  }

  /**
   * Export CSV with all 6 joints
   */
  exportCSV() {
    if (!this.plotData) return;
    const d = this.plotData;
    let csv = 'Time(s),q1,q2,q3,q4,q5,q6,v1,v2,v3,v4,v5,v6,a1,a2,a3,a4,a5,a6\n';

    for (let i = 0; i < d.time.length; i++) {
      const row = [
        d.time[i].toFixed(4),
        d.q[0][i].toFixed(6), d.q[1][i].toFixed(6), d.q[2][i].toFixed(6),
        d.q[3][i].toFixed(6), d.q[4][i].toFixed(6), d.q[5][i].toFixed(6),
        d.v[0][i].toFixed(6), d.v[1][i].toFixed(6), d.v[2][i].toFixed(6),
        d.v[3][i].toFixed(6), d.v[4][i].toFixed(6), d.v[5][i].toFixed(6),
        d.a[0][i].toFixed(6), d.a[1][i].toFixed(6), d.a[2][i].toFixed(6),
        d.a[3][i].toFixed(6), d.a[4][i].toFixed(6), d.a[5][i].toFixed(6)
      ];
      csv += row.join(',') + '\n';
    }

    const blob = new Blob([csv], { type: 'text/csv;charset=utf-8;' });
    const url = URL.createObjectURL(blob);
    const link = document.createElement('a');
    link.setAttribute('href', url);
    link.setAttribute('download', 'kinematics_data_6dof_robot_arm.csv');
    link.click();
  }

  /**
   * Generate Full 6-DOF MATLAB (.m) Script
   */
  copyMatlabScript() {
    if (!this.plotData) return;
    const d = this.plotData;

    const tStr = d.time.map(v => v.toFixed(3)).join(', ');
    const qStrs = d.q.map(arr => arr.map(v => v.toFixed(4)).join(', '));
    const vStrs = d.v.map(arr => arr.map(v => v.toFixed(4)).join(', '));
    const aStrs = d.a.map(arr => arr.map(v => v.toFixed(4)).join(', '));

    const script = `% =========================================================================
% MATLAB 6-DOF Robot Arm Kinematics Analysis Script (Yaskawa GP8)
% Generated from Industrial Robotics Simulation Web App
% =========================================================================

clear; clc; close all;

% 1. Time Vector (seconds)
t = [${tStr}];

% 2. 6-DOF Joint Angles q_1 .. q_6 (radians)
q1 = [${qStrs[0]}];
q2 = [${qStrs[1]}];
q3 = [${qStrs[2]}];
q4 = [${qStrs[3]}];
q5 = [${qStrs[4]}];
q6 = [${qStrs[5]}];

% 3. 6-DOF Joint Velocities v_1 .. v_6 (rad/s)
v1 = [${vStrs[0]}];
v2 = [${vStrs[1]}];
v3 = [${vStrs[2]}];
v4 = [${vStrs[3]}];
v5 = [${vStrs[4]}];
v6 = [${vStrs[5]}];

% 4. 6-DOF Joint Accelerations a_1 .. a_6 (rad/s^2)
a1 = [${aStrs[0]}];
a2 = [${aStrs[1]}];
a3 = [${aStrs[2]}];
a4 = [${aStrs[3]}];
a5 = [${aStrs[4]}];
a6 = [${aStrs[5]}];

% 5. FIGURE 1: Khớp Cơ Sở (Joints 1..3 - Base, Lower Arm, Upper Arm)
figure('Name', '6-DOF Robot Arm - Base Joints J1..J3', 'Color', [0.9 0.9 0.9], 'Position', [80, 80, 1050, 700]);

subplot(3,3,1); plot(t, q1, 'b', 'LineWidth', 2); grid on; title('Theta_1 (J1 - Base S)'); ylabel('q_1 (rad)'); xlabel('Time (s)');
subplot(3,3,2); plot(t, q2, 'b', 'LineWidth', 2); grid on; title('Theta_2 (J2 - Lower Arm L)'); ylabel('q_2 (rad)'); xlabel('Time (s)');
subplot(3,3,3); plot(t, q3, 'b', 'LineWidth', 2); grid on; title('Theta_3 (J3 - Upper Arm U)'); ylabel('q_3 (rad)'); xlabel('Time (s)');

subplot(3,3,4); plot(t, v1, 'r', 'LineWidth', 2); grid on; title('Velocity of theta_1'); ylabel('v_1 (rad/s)'); xlabel('Time (s)');
subplot(3,3,5); plot(t, v2, 'r', 'LineWidth', 2); grid on; title('Velocity of theta_2'); ylabel('v_2 (rad/s)'); xlabel('Time (s)');
subplot(3,3,6); plot(t, v3, 'r', 'LineWidth', 2); grid on; title('Velocity of theta_3'); ylabel('v_3 (rad/s)'); xlabel('Time (s)');

subplot(3,3,7); plot(t, a1, 'Color', [0 0.5 0], 'LineWidth', 2); grid on; title('Acceleration of theta_1'); ylabel('a_1 (rad/s^2)'); xlabel('Time (s)');
subplot(3,3,8); plot(t, a2, 'Color', [0 0.5 0], 'LineWidth', 2); grid on; title('Acceleration of theta_2'); ylabel('a_2 (rad/s^2)'); xlabel('Time (s)');
subplot(3,3,9); plot(t, a3, 'Color', [0 0.5 0], 'LineWidth', 2); grid on; title('Acceleration of theta_3'); ylabel('a_3 (rad/s^2)'); xlabel('Time (s)');

% 6. FIGURE 2: Khớp Cổ Tay (Joints 4..6 - Wrist Roll, Pitch, Twist)
figure('Name', '6-DOF Robot Arm - Wrist Joints J4..J6', 'Color', [0.9 0.9 0.9], 'Position', [120, 120, 1050, 700]);

subplot(3,3,1); plot(t, q4, 'b', 'LineWidth', 2); grid on; title('Theta_4 (J4 - Wrist Roll R)'); ylabel('q_4 (rad)'); xlabel('Time (s)');
subplot(3,3,2); plot(t, q5, 'b', 'LineWidth', 2); grid on; title('Theta_5 (J5 - Wrist Pitch B)'); ylabel('q_5 (rad)'); xlabel('Time (s)');
subplot(3,3,3); plot(t, q6, 'b', 'LineWidth', 2); grid on; title('Theta_6 (J6 - Wrist Twist T)'); ylabel('q_6 (rad)'); xlabel('Time (s)');

subplot(3,3,4); plot(t, v4, 'r', 'LineWidth', 2); grid on; title('Velocity of theta_4'); ylabel('v_4 (rad/s)'); xlabel('Time (s)');
subplot(3,3,5); plot(t, v5, 'r', 'LineWidth', 2); grid on; title('Velocity of theta_5'); ylabel('v_5 (rad/s)'); xlabel('Time (s)');
subplot(3,3,6); plot(t, v6, 'r', 'LineWidth', 2); grid on; title('Velocity of theta_6'); ylabel('v_6 (rad/s)'); xlabel('Time (s)');

subplot(3,3,7); plot(t, a4, 'Color', [0 0.5 0], 'LineWidth', 2); grid on; title('Acceleration of theta_4'); ylabel('a_4 (rad/s^2)'); xlabel('Time (s)');
subplot(3,3,8); plot(t, a5, 'Color', [0 0.5 0], 'LineWidth', 2); grid on; title('Acceleration of theta_5'); ylabel('a_5 (rad/s^2)'); xlabel('Time (s)');
subplot(3,3,9); plot(t, a6, 'Color', [0 0.5 0], 'LineWidth', 2); grid on; title('Acceleration of theta_6'); ylabel('a_6 (rad/s^2)'); xlabel('Time (s)');
`;

    navigator.clipboard.writeText(script).then(() => {
      const status = document.getElementById('matlab-status-info');
      if (status) {
        status.innerHTML = '✔ Đã sao chép toàn bộ mã lệnh MATLAB 6 Bậc (.m) vào Clipboard! Dán vào MATLAB và bấm F5 để chạy.';
      }
    }).catch(err => {
      alert('Không thể sao chép tự động: ' + err);
    });
  }
}
