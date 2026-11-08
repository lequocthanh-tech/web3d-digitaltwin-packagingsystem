/**
 * MPE720StudioStation.js
 * 
 * Engineering Workstation 2: Yaskawa MPE720 Ver 7 Engineering Studio (Bài 2 & 3)
 * Features:
 * 1. Project Navigation Tree (CPU-301 MP3300 controller structure)
 * 2. Tab 1: Interactive Ladder Logic Editor & Live Rung Monitor with glowing power flow (Rungs 1..5)
 * 3. Tab 2: Motion Program Editor (MPM001) for Delta Robot & Servo Axes
 * 4. Compiler & Controller Download engine
 */

export class MPE720StudioStation {
  constructor(app, options = {}) {
    this.app = app;
    this.hardwareBus = app.hardwareBus;
    this.deltaRobot = app.deltaRobot;
    this.deltaKinematics = app.deltaKinematics;
    this.factory = app.factory;
    this.physicsEngine = app.physicsEngine;

    this.activeStudioTab = 'ladder'; // 'ladder' or 'motion'

    // Motion Program Variables GL100..GL202
    this.motionVariables = [
      { id: 'GL100', name: 'Infeed Pick X,Y,Z', val: '1200, -320, 890 mm', desc: 'Tọa độ gắp sản phẩm lẻ từ Camera Cognex' },
      { id: 'GL102', name: 'Infeed Overhead', val: '1200, -320, 1250 mm', desc: 'Vị trí hạ tiếp cận trên Băng tải 1' },
      { id: 'GL104', name: 'Collation Drop', val: '1200, 180, 890 mm', desc: 'Vị trí thả ghép cụm 4 thanh trên Băng tải 2' },
      { id: 'GL200', name: 'Target Yaw Angle', val: '0.0 deg', desc: 'Góc xoay nắn thẳng sản phẩm' },
      { id: 'GL202', name: 'Tracking Velocity', val: '450.0 mm/s', desc: 'Tốc độ bám đồng bộ băng tải' }
    ];

    // Default Motion Program Code (MPM001)
    this.defaultMotionCode = [
      "// ========================================================",
      "// MOTION PROGRAM: MPM001 (Delta Robot 4-DOF Pick & Pack)",
      "// Controller: MP3300 [CPU-301(16axes)] | Module: 218IFD",
      "// ========================================================",
      "VAR",
      "  Speed_Fast : REAL := 800.0; // mm/s",
      "  Speed_Pick : REAL := 250.0; // mm/s",
      "  Acc_Rate   : REAL := 3000.0; // mm/s^2",
      "  Pack_Count : INT  := 0;",
      "END_VAR;",
      "",
      "// Main Pick & Place loop for Delta Robot",
      "MAIN_LOOP:",
      "  // 1. Chờ biến tần Băng tải 1 chạy và có sản phẩm đến vùng Vision",
      "  WAIT(IW00C80.0 = 1); // START Signal",
      "",
      "  // 2. Di chuyển nhanh đến vị trí chờ tiếp cận",
      "  VEL(Speed_Fast);",
      "  ACC(Acc_Rate);",
      "  MOV(GL102); // Overhead Infeed",
      "",
      "  // 3. Hạ độ cao gắp thanh kẹo lẻ theo tọa độ Vision (GL100)",
      "  VEL(Speed_Pick);",
      "  MOV(GL100);",
      "  SET_BIT(MB000050, 0); // Bật đầu hút chân không Delta (MAG2)",
      "  DWELL(0.12);",
      "",
      "  // 4. Nhấc lên và nắn thẳng góc Yaw = 0 độ",
      "  VEL(Speed_Fast);",
      "  MOV(GL102);",
      "",
      "  // 5. Chuyển sang Băng tải 2 thả ghép cụm 4 thanh",
      "  MOV(GL104); // Collate Target",
      "  RESET_BIT(MB000050, 0); // Tắt giác hút chân không",
      "  DWELL(0.12);",
      "",
      "  // 6. Lặp lại chu trình",
      "  GOTO MAIN_LOOP;"
    ].join('\n');
  }

  initDOM(containerEl) {
    this.container = containerEl;
    this.renderStudioUI();
    this.bindEvents();
    this.updateLiveRungs();
  }

  renderStudioUI() {
    this.container.innerHTML = `
      <div class="mpe720-studio-wrapper">
        
        <!-- MPE720 Top Window Chrome (Classic Industrial Engineering Tool) -->
        <div class="mpe-window-titlebar">
          <div class="mpe-title-text">
            <span class="mpe-icon">📁</span>
            <span>MPE720 Ver.7 - [BaiTN_DigitalTwin.mpe : MP3300 CPU-301(16axes)]</span>
          </div>
          <div class="mpe-title-tags">
            <span class="badge badge-success">ONLINE</span>
            <span class="badge" style="background:#0284c7; color:#fff;">SCAN: 2.0 ms (H-SCAN)</span>
          </div>
        </div>

        <!-- MPE720 Main Split Layout: Left Project Tree, Right Workspace -->
        <div class="mpe-body-split">
          
          <!-- Left: Engineering Project Explorer Tree -->
          <div class="mpe-tree-sidebar">
            <div class="mpe-tree-header">PROJECT EXPLORER</div>
            <div class="mpe-tree-content">
              <div class="tree-node active">
                <span class="node-arrow">▼</span>
                <span class="node-icon">💾</span>
                <span class="node-label">BaiTN_DigitalTwin</span>
              </div>
              <div class="tree-children">
                <div class="tree-node">
                  <span class="node-arrow">▼</span>
                  <span class="node-icon">⚙</span>
                  <span class="node-label">Controller [CPU-301]</span>
                </div>
                <div class="tree-children">
                  <div class="tree-node active-item" id="tree-item-ladder">
                    <span class="node-icon">⚡</span>
                    <span class="node-label">Ladder (H-Scan Rungs)</span>
                  </div>
                  <div class="tree-node" id="tree-item-motion">
                    <span class="node-icon">🎯</span>
                    <span class="node-label">Motion (MPM001 Delta)</span>
                  </div>
                  <div class="tree-node">
                    <span class="node-icon">🌐</span>
                    <span class="node-label">218IFD (192.168.1.1)</span>
                  </div>
                  <div class="tree-node">
                    <span class="node-icon">📊</span>
                    <span class="node-label">Data Registers (MW00010)</span>
                  </div>
                </div>
              </div>
            </div>

            <!-- Controller Hardware Status Mini Box -->
            <div class="mpe-hw-status-box">
              <div style="font-weight: 700; color: #e2e8f0; margin-bottom: 4px;">MP3300 CONTROLLER</div>
              <div style="display:flex; justify-content:space-between;"><span>RUN:</span><span style="color:#4ade80;">OK</span></div>
              <div style="display:flex; justify-content:space-between;"><span>ERR:</span><span style="color:#94a3b8;">NONE</span></div>
              <div style="display:flex; justify-content:space-between;"><span>PACK CNT:</span><b id="mpe-stat-pack-cnt" style="color:#facc15;">0 / 2</b></div>
            </div>
          </div>

          <!-- Right: Main Editor Workspace (Tabs for Ladder & Motion) -->
          <div class="mpe-workspace">
            
            <!-- Tab Navigation Bar -->
            <div class="mpe-tab-strip">
              <button class="mpe-tab-btn active" id="mpe-tab-ladder-btn">
                ⚡ LADDER PROGRAM // MAIN SCAN (H-SCAN)
              </button>
              <button class="mpe-tab-btn" id="mpe-tab-motion-btn">
                🎯 MOTION PROGRAM // MPM001 (DELTA ROBOT)
              </button>
            </div>

            <!-- Tab 1: Interactive Ladder Logic Diagram with Live Power Flow -->
            <div class="mpe-tab-content active" id="mpe-pane-ladder">
              <div class="mpe-toolbar">
                <span style="font-size: 9px; color: var(--text-muted);">
                  * Các tiếp điểm và cuộn dây phát sáng <b style="color:#4ade80;">XANH LÁ</b> thời gian thực khi có dòng điện Logic đi qua.
                </span>
                <div>
                  <button class="scada-btn btn-secondary" id="mpe-btn-force-start" style="padding: 2px 6px; font-size: 8.5px;">
                    FORCE START
                  </button>
                  <button class="scada-btn btn-secondary" id="mpe-btn-reset-counter" style="padding: 2px 6px; font-size: 8.5px;">
                    RESET MW00010
                  </button>
                </div>
              </div>

              <!-- Ladder Diagram Rungs View -->
              <div class="mpe-ladder-diagram" id="mpe-ladder-canvas">
                
                <!-- RUNG 1: Start/Stop Seal-in & Inverter DO_00 + Delta Start -->
                <div class="ladder-rung" id="rung-1">
                  <div class="rung-number">0001</div>
                  <div class="rung-circuit">
                    <div class="rail left-rail"></div>
                    
                    <!-- Parallel Branch: DI_00 (START) OR DO_00 (Seal-in) -->
                    <div class="branch-container">
                      <div class="branch-top">
                        <div class="ladder-element contact-no" id="contact-di0" title="START Push Button (DI_00)">
                          <span class="symbol-elem">-[ ]-</span>
                          <span class="elem-tag">DI_00</span>
                          <span class="elem-desc">START_BTN</span>
                        </div>
                      </div>
                      <div class="branch-bottom">
                        <div class="ladder-element contact-no" id="contact-do0-seal" title="Seal-in Coil (DO_00)">
                          <span class="symbol-elem">-[ ]-</span>
                          <span class="elem-tag">DO_00</span>
                          <span class="elem-desc">SEAL_IN</span>
                        </div>
                      </div>
                    </div>

                    <!-- Series: NOT DI_01 (STOP) -->
                    <div class="ladder-element contact-nc" id="contact-di1" title="STOP Push Button (DI_01)">
                      <span class="symbol-elem">-[/] -</span>
                      <span class="elem-tag">DI_01</span>
                      <span class="elem-desc">STOP_NC</span>
                    </div>

                    <!-- Series: NOT ESTOP -->
                    <div class="ladder-element contact-nc" id="contact-estop" title="Emergency Stop">
                      <span class="symbol-elem">-[/] -</span>
                      <span class="elem-tag">E_STOP</span>
                      <span class="elem-desc">SAFETY</span>
                    </div>

                    <!-- Wire Extension -->
                    <div class="wire-horizontal"></div>

                    <!-- Output Coil: DO_00 (J1000 AC Drive / Conveyors 1 & 2) -->
                    <div class="ladder-element coil" id="coil-do0" title="DO_00: Conveyor 1 & 2 Run">
                      <span class="symbol-elem">-( )-</span>
                      <span class="elem-tag">DO_00</span>
                      <span class="elem-desc">J1000_RUN</span>
                    </div>

                    <!-- Inline Call: MSEE MPM001 (Delta Motion) -->
                    <div class="ladder-element fb-box" id="fb-msee" title="Call Motion Program MPM001">
                      <span class="fb-title">MSEE</span>
                      <span class="fb-param">MPM001</span>
                    </div>

                    <div class="rail right-rail"></div>
                  </div>
                </div>

                <!-- RUNG 2: Sensor DI_11 (4-Pack Ready) -> DO_01 (Robot IN#1 Request) -->
                <div class="ladder-rung" id="rung-2">
                  <div class="rung-number">0002</div>
                  <div class="rung-circuit">
                    <div class="rail left-rail"></div>

                    <!-- Contact: DI_11 (Sensor Collation Station) -->
                    <div class="ladder-element contact-no" id="contact-di11" title="Sensor DI_11: 4-Pack Ready">
                      <span class="symbol-elem">-[ ]-</span>
                      <span class="elem-tag">DI_11</span>
                      <span class="elem-desc">PACK_SEN</span>
                    </div>

                    <div class="wire-horizontal"></div>

                    <!-- Coil: DO_01 (Arm IN#1 Pick Request) -->
                    <div class="ladder-element coil" id="coil-do1" title="DO_01: Robot Arm IN#1 Pick Request">
                      <span class="symbol-elem">-(S)-</span>
                      <span class="elem-tag">DO_01</span>
                      <span class="elem-desc">ARM_IN#1</span>
                    </div>

                    <div class="rail right-rail"></div>
                  </div>
                </div>

                <!-- RUNG 3: Rising Edge DI_12 (Robot Arm OT#3 Pulse) -> Counter MW00010 + DO_24 (Q0) -->
                <div class="ladder-rung" id="rung-3">
                  <div class="rung-number">0003</div>
                  <div class="rung-circuit">
                    <div class="rail left-rail"></div>

                    <!-- Contact: Rising Edge DI_12 (Arm Done Drop) -->
                    <div class="ladder-element contact-no" id="contact-di12" title="DI_12: Robot Arm OT#3 100ms Pulse">
                      <span class="symbol-elem">-[P]-</span>
                      <span class="elem-tag">DI_12</span>
                      <span class="elem-desc">ARM_DONE</span>
                    </div>

                    <div class="wire-horizontal"></div>

                    <!-- Function Block: ADD MW00010 = MW00010 + 1 -->
                    <div class="ladder-element fb-box" id="fb-add-counter" title="Counter MW00010 Increment">
                      <span class="fb-title">ADD_INC</span>
                      <span class="fb-param">MW00010 + 1</span>
                    </div>

                    <!-- Coil: DO_24 (I/O Box Q0 Lamp) -->
                    <div class="ladder-element coil" id="coil-do24" title="DO_24: Q0 Indicator Lamp">
                      <span class="symbol-elem">-( )-</span>
                      <span class="elem-tag">DO_24</span>
                      <span class="elem-desc">Q0_DONE</span>
                    </div>

                    <div class="rail right-rail"></div>
                  </div>
                </div>

                <!-- RUNG 4: Comparator MW00010 >= 2 (Full Box) -> DO_03 (Discharge Box Conveyor 3) -->
                <div class="ladder-rung" id="rung-4">
                  <div class="rung-number">0004</div>
                  <div class="rung-circuit">
                    <div class="rail left-rail"></div>

                    <!-- Comparator Contact: MW00010 >= 2 -->
                    <div class="ladder-element contact-no" id="contact-cmp-box" title="MW00010 >= 2 (2 Layers / 8 Bars)">
                      <span class="symbol-elem">-[>=]-</span>
                      <span class="elem-tag">MW00010</span>
                      <span class="elem-desc">K2_FULL</span>
                    </div>

                    <!-- Series: NOT DI_13 (Carton Moving) -->
                    <div class="ladder-element contact-nc" id="contact-di13" title="DI_13: Box in Position">
                      <span class="symbol-elem">-[/] -</span>
                      <span class="elem-tag">DI_13</span>
                      <span class="elem-desc">BOX_POS</span>
                    </div>

                    <div class="wire-horizontal"></div>

                    <!-- Output Coil: DO_03 (Box Conveyor 3 Run) -->
                    <div class="ladder-element coil" id="coil-do3" title="DO_03: Box Conveyor 3 Run">
                      <span class="symbol-elem">-( )-</span>
                      <span class="elem-tag">DO_03</span>
                      <span class="elem-desc">BOX_RUN</span>
                    </div>

                    <div class="rail right-rail"></div>
                  </div>
                </div>

                <!-- RUNG 5: Ethernet Send MSG-SNDE Block (DI_31 Trigger) -->
                <div class="ladder-rung" id="rung-5">
                  <div class="rung-number">0005</div>
                  <div class="rung-circuit">
                    <div class="rail left-rail"></div>

                    <!-- Contact: DI_31 (I/O Box Switch I7) -->
                    <div class="ladder-element contact-no" id="contact-di31" title="DI_31: Ethernet Send Request">
                      <span class="symbol-elem">-[ ]-</span>
                      <span class="elem-tag">DI_31</span>
                      <span class="elem-desc">SND_REQ</span>
                    </div>

                    <div class="wire-horizontal"></div>

                    <!-- MPE720 Standard Function Block: MSG-SNDE -->
                    <div class="ladder-element fb-box" id="fb-msgsnde" title="MSG-SNDE Ethernet Function Block" style="min-width: 140px;">
                      <span class="fb-title">MSG-SNDE</span>
                      <span class="fb-param" style="font-size:7.5px;">Dev:16 | Cir:1 | Ch:1</span>
                      <span class="fb-param" style="font-size:7.5px;">Param: MA00400</span>
                    </div>

                    <div class="rail right-rail"></div>
                  </div>
                </div>

              </div>
            </div>

            <!-- Tab 2: Motion Program Editor (MPM001 for Delta Robot) -->
            <div class="mpe-tab-content" id="mpe-pane-motion">
              <div class="mpe-toolbar">
                <span style="font-size: 9px; color: var(--text-muted);">
                  MPM001: Điều khiển Robot Delta 4 bậc đọc camera Cognex, gắp lẻ Băng tải 1 và ghép cụm 4 thanh sang Băng tải 2.
                </span>
                <div>
                  <button class="scada-btn btn-primary" id="mpe-btn-compile-motion" style="padding: 3px 8px; font-size: 9px;">
                    ⚡ COMPILE & DOWNLOAD TO MP3300
                  </button>
                </div>
              </div>

              <!-- Motion Variables Grid -->
              <div class="mpe-motion-vars-grid">
                <div class="pendant-panel-title" style="margin-bottom: 4px;">GLOBAL POSITION REGISTERS (GL100..GL202)</div>
                <div class="mpe-vars-chips">
                  ${this.motionVariables.map(v => `
                    <div class="mpe-chip">
                      <b style="color: #38bdf8;">${v.id}</b>
                      <span>${v.val}</span>
                      <span style="font-size:7.5px; color:var(--text-dim);">${v.desc}</span>
                    </div>
                  `).join('')}
                </div>
              </div>

              <!-- Motion Code Text Editor -->
              <div class="mpe-code-box">
                <textarea class="mpe-motion-textarea" id="mpe-motion-editor" spellcheck="false">${this.defaultMotionCode}</textarea>
              </div>

              <div class="mpe-compile-log" id="mpe-compile-log">
                [COMPILER]: MPE720 Motion Compiler sẵn sàng. Bấm [COMPILE & DOWNLOAD TO MP3300] để nạp xuống CPU.
              </div>
            </div>

          </div>

        </div>

      </div>
    `;
  }

  bindEvents() {
    // 1. Tab switching between Ladder & Motion
    const tabLadderBtn = document.getElementById('mpe-tab-ladder-btn');
    const tabMotionBtn = document.getElementById('mpe-tab-motion-btn');
    const paneLadder = document.getElementById('mpe-pane-ladder');
    const paneMotion = document.getElementById('mpe-pane-motion');

    const treeItemLadder = document.getElementById('tree-item-ladder');
    const treeItemMotion = document.getElementById('tree-item-motion');

    const switchTab = (tabName) => {
      this.activeStudioTab = tabName;
      if (tabName === 'ladder') {
        tabLadderBtn.classList.add('active');
        tabMotionBtn.classList.remove('active');
        paneLadder.classList.add('active');
        paneMotion.classList.remove('active');
        if (treeItemLadder) treeItemLadder.classList.add('active-item');
        if (treeItemMotion) treeItemMotion.classList.remove('active-item');
      } else {
        tabMotionBtn.classList.add('active');
        tabLadderBtn.classList.remove('active');
        paneMotion.classList.add('active');
        paneLadder.classList.remove('active');
        if (treeItemMotion) treeItemMotion.classList.add('active-item');
        if (treeItemLadder) treeItemLadder.classList.remove('active-item');
      }
    };

    if (tabLadderBtn) tabLadderBtn.addEventListener('click', () => switchTab('ladder'));
    if (tabMotionBtn) tabMotionBtn.addEventListener('click', () => switchTab('motion'));
    if (treeItemLadder) treeItemLadder.addEventListener('click', () => switchTab('ladder'));
    if (treeItemMotion) treeItemMotion.addEventListener('click', () => switchTab('motion'));

    // 2. Force Start Button
    const btnForceStart = document.getElementById('mpe-btn-force-start');
    if (btnForceStart) {
      btnForceStart.addEventListener('click', () => {
        if (this.hardwareBus) {
          this.hardwareBus.setDI(0, 1);
          setTimeout(() => this.hardwareBus.setDI(0, 0), 200);
        }
      });
    }

    // 3. Reset Counter Button
    const btnResetCounter = document.getElementById('mpe-btn-reset-counter');
    if (btnResetCounter) {
      btnResetCounter.addEventListener('click', () => {
        if (this.hardwareBus) {
          this.hardwareBus.writeMW(10, 0);
          this.hardwareBus.plcBoxClusterCount = 0;
        }
        if (this.physicsEngine) {
          this.physicsEngine.armVacuum.currentBoxLayers = 0;
        }
        const lbl = document.getElementById('label-box-layers');
        if (lbl) lbl.textContent = 'Lớp: 0 / 2';
      });
    }

    // 4. Compile Motion Program Button
    const btnCompile = document.getElementById('mpe-btn-compile-motion');
    if (btnCompile) {
      btnCompile.addEventListener('click', async () => {
        const logEl = document.getElementById('mpe-compile-log');
        btnCompile.disabled = true;
        btnCompile.textContent = 'COMPILING...';

        logEl.textContent = '[COMPILER] Đang phân tích cú pháp chương trình MPM001...';
        await new Promise(r => setTimeout(r, 300));
        logEl.textContent += '\n[COMPILER] Cú pháp hợp lệ 100%. Đang kết nối MP3300 CPU-301 qua 218IFD (192.168.1.1)...';
        await new Promise(r => setTimeout(r, 350));
        logEl.textContent += '\n[COMPILER] DOWNLOAD SUCCESSFUL! Chương trình MPM001 đã nạp vào bộ nhớ Motion CPU.';

        btnCompile.disabled = false;
        btnCompile.textContent = '⚡ COMPILE & DOWNLOAD TO MP3300';
      });
    }
  }

  /**
   * Updates real-time green glowing power flow through ladder rungs
   */
  updateLiveRungs() {
    if (!this.hardwareBus) return;

    const setGlowing = (elemId, active) => {
      const el = document.getElementById(elemId);
      if (el) el.classList.toggle('power-flow-active', !!active);
    };

    // RUNG 1
    const di0 = this.hardwareBus.getDI(0) === 1;
    const do0 = this.hardwareBus.getDO(0) === 1;
    const di1 = this.hardwareBus.getDI(1) === 1;
    const estop = this.hardwareBus.emergencyStop;

    setGlowing('contact-di0', di0);
    setGlowing('contact-do0-seal', do0);
    setGlowing('contact-di1', !di1); // Normally closed: active when NOT pressed
    setGlowing('contact-estop', !estop);
    setGlowing('coil-do0', do0);
    setGlowing('fb-msee', do0);

    // RUNG 2
    const di11 = this.hardwareBus.getDI(11) === 1;
    const do1 = this.hardwareBus.getDO(1) === 1;
    setGlowing('contact-di11', di11);
    setGlowing('coil-do1', do1);

    // RUNG 3
    const di12 = this.hardwareBus.getDI(12) === 1;
    const do24 = this.hardwareBus.getDO(24) === 1;
    setGlowing('contact-di12', di12);
    setGlowing('fb-add-counter', di12);
    setGlowing('coil-do24', do24);

    // RUNG 4
    const boxFull = this.hardwareBus.plcBoxClusterCount >= 2;
    const di13 = this.hardwareBus.getDI(13) === 1;
    const do3 = this.hardwareBus.getDO(3) === 1;
    setGlowing('contact-cmp-box', boxFull);
    setGlowing('contact-di13', !di13);
    setGlowing('coil-do3', do3);

    // RUNG 5
    const di31 = this.hardwareBus.getDI(31) === 1;
    setGlowing('contact-di31', di31);
    setGlowing('fb-msgsnde', di31);

    // Mini box update
    const packCntEl = document.getElementById('mpe-stat-pack-cnt');
    if (packCntEl) {
      packCntEl.textContent = `${this.hardwareBus.plcBoxClusterCount} / 2`;
    }
  }
}
