/**
 * CSharpPCStudioStation.js
 * 
 * Engineering Workstation 3: C# / Qt PC Control & Vision Studio (Bài 4 & 5)
 * Split-view layout:
 * - Left: Genuine Windows Form / Qt Robot Controller GUI matching Bài 4
 * - Right: C# Syntax Code Editor & Cognex Vision Script (YRCCommunication IDE)
 */

import { runAutoLetterTJob } from './JobLetterT.js';

export class CSharpPCStudioStation {
  constructor(app, options = {}) {
    this.app = app;
    this.ethernetServer = app.ethernetServer;
    this.hardwareBus = app.hardwareBus;
    this.robotArm = app.arm6DOF;
    this.kinematics = app.armKinematics;
    this.planner = app.trajectoryPlanner;
    this.physicsEngine = app.physicsEngine;

    // PC Connection & Control States
    this.isConnected = false;
    this.isServoOn = false;
    this.isToolOn = false;
    this.robotStatus = 'Stop';

    this.nudgeDist = 10.0; // mm or deg
    this.nudgeSpeed = 100.0; // mm/s or deg/s

    // Default C# Application Template Code (Bài 4 & Bài 5)
    this.defaultCSharpCode = [
      "// ===========================================================================",
      "// C# / WPF YASKAWA ROBOT CONTROLLER & COGNEX VISION PACKAGING APPLICATION",
      "// BÀI 4: LẬP TRÌNH ĐIỀU KHIỂN ROBOT QUA MẠNG ETHERNET (UDP 10040)",
      "// BÀI 5: THU THẬP TỌA ĐỘ VISION & ĐÓNG GÓI TỰ ĐỘNG",
      "// ===========================================================================",
      "using System;",
      "using System.Net;",
      "using System.Net.Sockets;",
      "using System.Threading;",
      "using YRCCommunication;",
      "",
      "namespace DigitalTwinPackagingStudio",
      "{",
      "    public class MainWindow : IDisposable",
      "    {",
      "        private YRCServerClient yrcClient;",
      "        private CognexVisionCamera visionSensor;",
      "        private bool isRunningAutoCycle = false;",
      "",
      "        // Kết nối Server YRC1000micro (192.168.1.11:10040)",
      "        public void btn_Connect_Click()",
      "        {",
      "            yrcClient = new YRCServerClient(\"192.168.1.11\", 10040);",
      "            yrcClient.Connect();",
      "            Console.WriteLine(\"[YRC COMM] Kết nối UDP 10040 thành công.\");",
      "        }",
      "",
      "        // Bật / Tắt Servo (Command 0x0083)",
      "        public void btn_OnOff_Servo_Click(bool enable)",
      "        {",
      "            if (enable) yrcClient.ServoOn();",
      "            else yrcClient.ServoOff();",
      "        }",
      "",
      "        // Điều khiển dịch chuyển tương đối trục X (Command 0x008A)",
      "        public void btn_xP_Click(double stepMm, double speed)",
      "        {",
      "            double[] posInc = new double[] { stepMm, 0, 0, 0, 0, 0 };",
      "            yrcClient.YRCMoveCartesian(",
      "                Constant.CMD_DATA_MOVE_COORDINATE_ROBOT,",
      "                Constant.CMD_HEADER_MOVE_INSTANCE_STRAIGHT_INCREMENT,",
      "                Constant.CMD_DATA_MOVE_SPEED_TYPE_V_SPEED,",
      "                speed,",
      "                posInc",
      "            );",
      "        }",
      "",
      "        // BÀI 5: Chu trình tự động gắp thanh kẹo từ Vision Camera sang thùng",
      "        public void RunAutoVisionPackagingLoop()",
      "        {",
      "            Console.WriteLine(\"[VISION LOOP] Khởi động vòng lặp đóng gói tự động...\");",
      "            isRunningAutoCycle = true;",
      "",
      "            // 1. Về vị trí Home P000",
      "            yrcClient.YRCMoveCartesian(0, 1, 1, 100.0, new double[] { -1200, 1400, -100, 0, 0, 0 });",
      "",
      "            // 2. Tiếp cận cụm 4 thanh tại trạm DI_11",
      "            yrcClient.YRCMoveCartesian(0, 2, 0, 180.0, new double[] { -1250, 910, 180, 0, 0, 0 });",
      "            yrcClient.SetVacuumTool(true); // Bật đầu hút chân không",
      "            Thread.Sleep(200);",
      "",
      "            // 3. Đưa vào thùng carton phía sau Robot Arm (X=-1250, Y=790, Z=-1450)",
      "            yrcClient.YRCMoveCartesian(0, 2, 0, 500.0, new double[] { -1250, 790, -1450, 0, 0, 0 });",
      "            yrcClient.SetVacuumTool(false); // Thả hàng",
      "            yrcClient.PulseRobotOutput(3, 100); // Xung 100ms báo PLC",
      "",
      "            Console.WriteLine(\"[VISION LOOP] Hoàn tất 1 chu trình đóng gói!\");",
      "        }",
      "",
      "        // BÀI THỰC HÀNH: Tự động điều khiển Robot Arm vẽ chữ T và hiển thị đồ thị",
      "        public void RunAutoDrawLetterT()",
      "        {",
      "            Console.WriteLine(\"[TRAJECTORY] Khởi động chu trình tự động vẽ chữ T...\");",
      "            yrcClient.ServoOn();",
      "",
      "            // 1. Điểm an toàn tiếp cận P020 (-1250, 1300, 180)",
      "            yrcClient.YRCMoveCartesian(0, 1, 1, 100.0, new double[] { -1250, 1300, 180, 0, 0, 0 });",
      "",
      "            // 2. Đầu nét ngang bên trái P021 (-1450, 1200, 180)",
      "            yrcClient.YRCMoveCartesian(0, 2, 0, 150.0, new double[] { -1450, 1200, 180, 0, 0, 0 });",
      "",
      "            // 3. Kéo nét ngang sang phải P022 (-1050, 1200, 180)",
      "            yrcClient.YRCMoveCartesian(0, 2, 0, 80.0, new double[] { -1050, 1200, 180, 0, 0, 0 });",
      "",
      "            // 4. Trượt về tâm nét ngang P023 (-1250, 1200, 180)",
      "            yrcClient.YRCMoveCartesian(0, 2, 0, 120.0, new double[] { -1250, 1200, 180, 0, 0, 0 });",
      "",
      "            // 5. Kéo nét đứng thẳng xuống P024 (-1250, 880, 180) -> Hoàn tất chữ T",
      "            yrcClient.YRCMoveCartesian(0, 2, 0, 80.0, new double[] { -1250, 880, 180, 0, 0, 0 });",
      "            Thread.Sleep(500);",
      "",
      "            // 6. Rút về an toàn",
      "            yrcClient.YRCMoveCartesian(0, 1, 1, 100.0, new double[] { -1250, 1300, 180, 0, 0, 0 });",
      "            Console.WriteLine(\"[TRAJECTORY] Hoàn tất vẽ chữ T! Đồ thị đã hiển thị chữ T.\");",
      "        }",
      "",
      "        public void Dispose()",
      "        {",
      "            yrcClient?.Disconnect();",
      "        }",
      "    }",
      "}"
    ].join('\n');
  }

  initDOM(containerEl) {
    this.container = containerEl;
    this.renderPCStudioUI();
    this.bindEvents();
    this.updateCurrentPositionUI();
  }

  renderPCStudioUI() {
    this.container.innerHTML = `
      <div class="csharp-studio-wrapper">
        
        <!-- Window Title Bar (Windows 11 / Visual Studio Dark Style) -->
        <div class="pc-window-titlebar">
          <div class="pc-title-left">
            <span class="pc-icon">💻</span>
            <span>Yaskawa Robot Controller GUI & Vision IDE - [Visual Studio C# 2026]</span>
          </div>
          <div class="pc-title-right">
            <span class="badge" id="csharp-conn-badge">DISCONNECTED</span>
            <span class="badge badge-success" id="csharp-udp-badge">UDP: 192.168.1.11:10040</span>
          </div>
        </div>

        <!-- Split Body: Left Windows Form GUI, Right Code Editor -->
        <div class="csharp-body-split">
          
          <!-- ===============================================================
               LEFT: WINDOWS FORM / QT GUI (Bài 4 Standard Form)
               =============================================================== -->
          <div class="csharp-gui-pane">
            <div class="gui-form-scrollable">
              
              <!-- 1. Group: Connection -->
              <div class="gui-group-box">
                <div class="gui-group-title">Connection (UDP High-Speed)</div>
                <div class="gui-grid-row">
                  <div class="gui-field">
                    <label>IP address:</label>
                    <input type="text" class="gui-input" id="gui-ip" value="192.168.1.11">
                  </div>
                  <div class="gui-field">
                    <label>Port:</label>
                    <input type="text" class="gui-input" id="gui-port" value="10040">
                  </div>
                  <div class="gui-field">
                    <label>File Port:</label>
                    <input type="text" class="gui-input" id="gui-fileport" value="10041">
                  </div>
                  <div class="gui-field" style="align-self: flex-end;">
                    <button class="gui-btn btn-connect" id="gui-btn-connect">Connect</button>
                  </div>
                </div>
              </div>

              <!-- 2. Group: Basic Control -->
              <div class="gui-group-box">
                <div class="gui-group-title">Basic Control</div>
                <div class="gui-buttons-row">
                  <button class="gui-btn" id="gui-btn-home">Go Home</button>
                  <button class="gui-btn" id="gui-btn-tool">Tool On</button>
                  <button class="gui-btn" id="gui-btn-servo">ON Servo</button>
                  <div class="gui-status-tag">
                    <span>Robot Status:</span>
                    <b id="gui-robot-status" style="color: #4ade80;">Stop</b>
                  </div>
                </div>
              </div>

              <!-- 3. Group: Current Position -->
              <div class="gui-group-box">
                <div class="gui-group-title">
                  <span>Current Position (mm / deg)</span>
                  <button class="gui-btn-mini" id="gui-btn-getpos">Get Position</button>
                </div>
                <div class="gui-grid-6">
                  <div class="pos-cell"><span>X:</span><b id="cur-x">0.0</b></div>
                  <div class="pos-cell"><span>Y:</span><b id="cur-y">0.0</b></div>
                  <div class="pos-cell"><span>Z:</span><b id="cur-z">0.0</b></div>
                  <div class="pos-cell"><span>Roll:</span><b id="cur-rx">0.0°</b></div>
                  <div class="pos-cell"><span>Pitch:</span><b id="cur-ry">0.0°</b></div>
                  <div class="pos-cell"><span>Yaw:</span><b id="cur-rz">0.0°</b></div>
                </div>
              </div>

              <!-- 4. Group: Set Position -->
              <div class="gui-group-box">
                <div class="gui-group-title">Set Position & Trajectory Move</div>
                <div class="gui-grid-6">
                  <div class="gui-input-col"><label>X:</label><input type="number" step="10" id="set-x" value="-1250" class="gui-input"></div>
                  <div class="gui-input-col"><label>Y:</label><input type="number" step="10" id="set-y" value="910" class="gui-input"></div>
                  <div class="gui-input-col"><label>Z:</label><input type="number" step="10" id="set-z" value="180" class="gui-input"></div>
                  <div class="gui-input-col"><label>Rx:</label><input type="number" step="5" id="set-rx" value="0" class="gui-input"></div>
                  <div class="gui-input-col"><label>Ry:</label><input type="number" step="5" id="set-ry" value="0" class="gui-input"></div>
                  <div class="gui-input-col"><label>Rz:</label><input type="number" step="5" id="set-rz" value="0" class="gui-input"></div>
                </div>

                <div class="gui-options-row" style="margin-top: 6px;">
                  <div class="gui-field" style="width: 100px;">
                    <label>Speed (mm/s):</label>
                    <input type="number" id="set-speed" value="150" class="gui-input">
                  </div>
                  <div class="gui-radios-box">
                    <label><input type="radio" name="coord-type" value="xyz" checked> XYZ</label>
                    <label><input type="radio" name="coord-type" value="rpy"> RPY</label>
                  </div>
                  <div class="gui-radios-box">
                    <label><input type="radio" name="move-type" value="link_abs"> Link Abs</label>
                    <label><input type="radio" name="move-type" value="line_abs" checked> Line Abs</label>
                    <label><input type="radio" name="move-type" value="line_inc"> Line Inc</label>
                  </div>
                  <button class="gui-btn btn-primary" id="gui-btn-setpos" style="align-self: flex-end;">Set Position</button>
                </div>
              </div>

              <!-- 5. Group: Incremental Position (Nudge Buttons) -->
              <div class="gui-group-box">
                <div class="gui-group-title">Incremental Position (Nudge Control)</div>
                <div class="gui-nudge-grid">
                  <div class="nudge-pair"><span>X:</span><button class="gui-btn-step" id="nudge-xm">-X</button><button class="gui-btn-step" id="nudge-xp">+X</button></div>
                  <div class="nudge-pair"><span>Y:</span><button class="gui-btn-step" id="nudge-ym">-Y</button><button class="gui-btn-step" id="nudge-yp">+Y</button></div>
                  <div class="nudge-pair"><span>Z:</span><button class="gui-btn-step" id="nudge-zm">-Z</button><button class="gui-btn-step" id="nudge-zp">+Z</button></div>
                  <div class="nudge-pair"><span>Rx:</span><button class="gui-btn-step" id="nudge-rxm">-Rx</button><button class="gui-btn-step" id="nudge-rxp">+Rx</button></div>
                  <div class="nudge-pair"><span>Ry:</span><button class="gui-btn-step" id="nudge-rym">-Ry</button><button class="gui-btn-step" id="nudge-ryp">+Ry</button></div>
                  <div class="nudge-pair"><span>Rz:</span><button class="gui-btn-step" id="nudge-rzm">-Rz</button><button class="gui-btn-step" id="nudge-rzp">+Rz</button></div>
                </div>

                <div class="gui-sliders-row" style="margin-top: 6px;">
                  <div class="gui-slider-col">
                    <div style="display:flex; justify-content:space-between; font-size:8px;"><span>Distance:</span><b id="lbl-dist-val">10.0 mm/deg</b></div>
                    <input type="range" min="1" max="50" value="10" id="slider-dist">
                  </div>
                  <div class="gui-slider-col">
                    <div style="display:flex; justify-content:space-between; font-size:8px;"><span>Speed:</span><b id="lbl-speed-val">100.0 mm/s</b></div>
                    <input type="range" min="10" max="300" value="100" id="slider-speed">
                  </div>
                </div>
              </div>

              <!-- 6. Group: Forward Kinematics DH Validation -->
              <div class="gui-group-box">
                <div class="gui-group-title">
                  <span>Get Joint & Calculate Forward Kinematics (DH Matrix)</span>
                  <div>
                    <button class="gui-btn-mini" id="gui-btn-getjoints">Get Joints</button>
                    <button class="gui-btn-mini" id="gui-btn-calcfk" style="color:#4ade80;">Calculate FK</button>
                  </div>
                </div>
                <div class="gui-grid-6">
                  <div class="gui-input-col"><label>J1:</label><input type="text" id="fk-j1" value="0.0" class="gui-input" readonly></div>
                  <div class="gui-input-col"><label>J2:</label><input type="text" id="fk-j2" value="0.0" class="gui-input" readonly></div>
                  <div class="gui-input-col"><label>J3:</label><input type="text" id="fk-j3" value="0.0" class="gui-input" readonly></div>
                  <div class="gui-input-col"><label>J4:</label><input type="text" id="fk-j4" value="0.0" class="gui-input" readonly></div>
                  <div class="gui-input-col"><label>J5:</label><input type="text" id="fk-j5" value="0.0" class="gui-input" readonly></div>
                  <div class="gui-input-col"><label>J6:</label><input type="text" id="fk-j6" value="0.0" class="gui-input" readonly></div>
                </div>
                <div class="fk-results-line" id="fk-calc-result" style="margin-top: 4px; font-family: var(--font-mono); font-size: 8px; color: #38bdf8;">
                  FK Output: X=0.0, Y=0.0, Z=0.0 mm | Rx=0.0°, Ry=0.0°, Rz=0.0°
                </div>
              </div>

            </div>
          </div>

          <!-- ===============================================================
               RIGHT: C# CODE EDITOR & COGNEX VISION IDE
               =============================================================== -->
          <div class="csharp-code-pane">
            <div class="code-pane-header">
              <div style="display:flex; align-items:center; gap: 6px;">
                <span class="file-icon">📄</span>
                <span style="font-weight: 700; color: #e2e8f0;">MainWindow.xaml.cs</span>
                <span style="color: var(--text-dim); font-size: 8px;">(YRCCommunication.dll)</span>
              </div>
              <div style="display:flex; gap:6px;">
                <button class="scada-btn" id="csharp-btn-run-t" style="padding: 3px 10px; font-size: 9px; background: #0d9488; border-color: #2dd4bf; color: #fff; font-weight: 700;">
                  ✍️ RUN LETTER T (C#)
                </button>
                <button class="scada-btn btn-primary" id="csharp-btn-execute" style="padding: 3px 10px; font-size: 9px; background: #2563eb;">
                  ▶ COMPILE & EXECUTE C# CODE
                </button>
              </div>
            </div>

            <div class="code-pane-body">
              <textarea class="csharp-code-textarea" id="csharp-code-editor" spellcheck="false">${this.defaultCSharpCode}</textarea>
            </div>

            <!-- C# Runtime & Output Terminal -->
            <div class="csharp-terminal" id="csharp-terminal">
              [C# COMPILER]: Roslyn .NET 8.0 Compiler sẵn sàng. Bấm [COMPILE & EXECUTE C# CODE] để chạy logic đóng gói tự động.
            </div>
          </div>

        </div>

      </div>
    `;
  }

  bindEvents() {
    // 1. Connect / Disconnect Button
    const btnConnect = document.getElementById('gui-btn-connect');
    const connBadge = document.getElementById('csharp-conn-badge');
    if (btnConnect) {
      btnConnect.addEventListener('click', () => {
        this.isConnected = !this.isConnected;
        if (this.isConnected) {
          btnConnect.textContent = 'Disconnect';
          btnConnect.classList.add('connected');
          if (connBadge) {
            connBadge.textContent = 'CONNECTED (UDP)';
            connBadge.className = 'badge badge-success';
          }
          this.logTerminal('[C# NETWORK] Đã kết nối UDP tới YRC1000micro (192.168.1.11:10040). Server Ready.');
        } else {
          btnConnect.textContent = 'Connect';
          btnConnect.classList.remove('connected');
          if (connBadge) {
            connBadge.textContent = 'DISCONNECTED';
            connBadge.className = 'badge';
          }
          this.logTerminal('[C# NETWORK] Đã ngắt kết nối UDP Socket.');
        }
      });
    }

    // 2. Servo On / Off Button
    const btnServo = document.getElementById('gui-btn-servo');
    if (btnServo) {
      btnServo.addEventListener('click', async () => {
        this.isServoOn = !this.isServoOn;
        if (this.isServoOn) {
          btnServo.textContent = 'OFF Servo';
          btnServo.classList.add('active');
          if (this.ethernetServer) {
            await this.ethernetServer.sendMSGSNDE('SERVO_ON');
          }
          this.logTerminal('[C# SERVO] Gửi mã lệnh 0x0083 -> SERVO ON hoàn tất.');
        } else {
          btnServo.textContent = 'ON Servo';
          btnServo.classList.remove('active');
          if (this.ethernetServer) {
            await this.ethernetServer.sendMSGSNDE('SERVO_OFF');
          }
          this.logTerminal('[C# SERVO] Gửi mã lệnh 0x0083 -> SERVO OFF.');
        }
      });
    }

    // 3. Tool On / Off Button
    const btnTool = document.getElementById('gui-btn-tool');
    if (btnTool) {
      btnTool.addEventListener('click', () => {
        this.isToolOn = !this.isToolOn;
        btnTool.textContent = this.isToolOn ? 'Tool Off' : 'Tool On';
        btnTool.classList.toggle('active', this.isToolOn);

        if (this.physicsEngine) {
          if (this.isToolOn) {
            const pack = this.physicsEngine.findReachableCollatedPack(-1.25, 0.45);
            if (pack) this.physicsEngine.attachArmGripper(pack);
          } else {
            this.physicsEngine.releaseArmGripper();
          }
        }
        this.logTerminal(`[C# TOOL] Giác hút chân không Robot Arm: ${this.isToolOn ? 'ON (MAG2)' : 'OFF'}`);
      });
    }

    // 4. Go Home Button
    const btnHome = document.getElementById('gui-btn-home');
    if (btnHome) {
      btnHome.addEventListener('click', () => {
        this.moveArmToPose({ x: -0.98, y: 1.32, z: 0.24, rx: 0, ry: 0, rz: 0 }, 150.0);
        this.logTerminal('[C# MOTION] Di chuyển Robot Arm về vị trí HOME (-980, 1320, 240).');
      });
    }

    // 5. Get Position Button
    const btnGetPos = document.getElementById('gui-btn-getpos');
    if (btnGetPos) {
      btnGetPos.addEventListener('click', async () => {
        if (this.ethernetServer) {
          await this.ethernetServer.sendMSGSNDE('READ_POS');
        }
        this.updateCurrentPositionUI();
        this.logTerminal('[C# CMD 0x0075] Đã đọc tọa độ thực từ YRC1000 Server.');
      });
    }

    // 6. Set Position Button
    const btnSetPos = document.getElementById('gui-btn-setpos');
    if (btnSetPos) {
      btnSetPos.addEventListener('click', () => {
        const x = parseFloat(document.getElementById('set-x').value) * 0.001;
        const y = parseFloat(document.getElementById('set-y').value) * 0.001;
        const z = parseFloat(document.getElementById('set-z').value) * 0.001;
        const rx = parseFloat(document.getElementById('set-rx').value);
        const ry = parseFloat(document.getElementById('set-ry').value);
        const rz = parseFloat(document.getElementById('set-rz').value);
        const spd = parseFloat(document.getElementById('set-speed').value);

        this.moveArmToPose({ x, y, z, rx, ry, rz }, spd);
        this.logTerminal(`[C# YRCMoveCartesian] Di chuyển tới X=${x * 1000}, Y=${y * 1000}, Z=${z * 1000} mm, Speed=${spd} mm/s.`);
      });
    }

    // 7. Distance & Speed Sliders
    const sDist = document.getElementById('slider-dist');
    const sSpeed = document.getElementById('slider-speed');
    const lblDist = document.getElementById('lbl-dist-val');
    const lblSpeed = document.getElementById('lbl-speed-val');

    if (sDist) {
      sDist.addEventListener('input', (e) => {
        this.nudgeDist = parseFloat(e.target.value);
        if (lblDist) lblDist.textContent = `${this.nudgeDist.toFixed(1)} mm/deg`;
      });
    }
    if (sSpeed) {
      sSpeed.addEventListener('input', (e) => {
        this.nudgeSpeed = parseFloat(e.target.value);
        if (lblSpeed) lblSpeed.textContent = `${this.nudgeSpeed.toFixed(1)} mm/s`;
      });
    }

    // 8. 12 Nudge Buttons (X, Y, Z, Rx, Ry, Rz +/-)
    const attachNudge = (btnId, axisIdx, dir) => {
      const btn = document.getElementById(btnId);
      if (!btn) return;
      btn.addEventListener('click', () => {
        this.nudgeAxis(axisIdx, dir);
      });
    };

    attachNudge('nudge-xm', 0, -1);
    attachNudge('nudge-xp', 0, 1);
    attachNudge('nudge-ym', 1, -1);
    attachNudge('nudge-yp', 1, 1);
    attachNudge('nudge-zm', 2, -1);
    attachNudge('nudge-zp', 2, 1);
    attachNudge('nudge-rxm', 3, -1);
    attachNudge('nudge-rxp', 3, 1);
    attachNudge('nudge-rym', 4, -1);
    attachNudge('nudge-ryp', 4, 1);
    attachNudge('nudge-rzm', 5, -1);
    attachNudge('nudge-rzp', 5, 1);

    // 9. Get Joints & Calculate FK
    const btnGetJoints = document.getElementById('gui-btn-getjoints');
    if (btnGetJoints) {
      btnGetJoints.addEventListener('click', () => {
        const q = this.robotArm.getJointAngles();
        document.getElementById('fk-j1').value = q.s.toFixed(2);
        document.getElementById('fk-j2').value = q.l.toFixed(2);
        document.getElementById('fk-j3').value = q.u.toFixed(2);
        document.getElementById('fk-j4').value = q.r.toFixed(2);
        document.getElementById('fk-j5').value = q.b.toFixed(2);
        document.getElementById('fk-j6').value = q.t.toFixed(2);
        this.logTerminal('[C# GET JOINTS] Đã đọc 6 góc khớp J1..J6 từ bộ mã hóa xung Encoder.');
      });
    }

    const btnCalcFK = document.getElementById('gui-btn-calcfk');
    if (btnCalcFK) {
      btnCalcFK.addEventListener('click', () => {
        const j1 = parseFloat(document.getElementById('fk-j1').value);
        const j2 = parseFloat(document.getElementById('fk-j2').value);
        const j3 = parseFloat(document.getElementById('fk-j3').value);
        const j4 = parseFloat(document.getElementById('fk-j4').value);
        const j5 = parseFloat(document.getElementById('fk-j5').value);
        const j6 = parseFloat(document.getElementById('fk-j6').value);

        const fk = this.kinematics.forwardKinematics({ s: j1, l: j2, u: j3, r: j4, b: j5, t: j6 });
        const resEl = document.getElementById('fk-calc-result');
        if (resEl) {
          resEl.textContent = `FK Output: X=${(fk.tcp_m.x * 1000).toFixed(1)}, Y=${(fk.tcp_m.y * 1000).toFixed(1)}, Z=${(fk.tcp_m.z * 1000).toFixed(1)} mm | Rx=${fk.orientation_deg.rx.toFixed(1)}°, Ry=${fk.orientation_deg.ry.toFixed(1)}°, Rz=${fk.orientation_deg.rz.toFixed(1)}°`;
        }
        this.logTerminal('[C# CALC FK] Đã tính toán ma trận chuyển vị T0_6 Denavit-Hartenberg thành công.');
      });
    }

    // 9b. Run Letter T from C# Studio
    const btnRunTCSharp = document.getElementById('csharp-btn-run-t');
    if (btnRunTCSharp) {
      btnRunTCSharp.addEventListener('click', async () => {
        btnRunTCSharp.disabled = true;
        btnRunTCSharp.textContent = 'RUNNING T...';
        this.logTerminal('[ROS-LYN C#] Đang biên dịch MainWindow.RunAutoDrawLetterT()...');
        this.logTerminal('[YRC COMM 10040]: Gửi chuỗi lệnh YRCMoveCartesian vẽ quỹ đạo chữ T...');
        try {
          await runAutoLetterTJob(this.app);
          this.logTerminal('[C# APP] Chu trình RunAutoDrawLetterT() hoàn tất! Đồ thị đã hiển thị chữ T.');
        } catch (err) {
          this.logTerminal(`[C# RUNTIME EXCEPTION]: ${err.message}`);
        } finally {
          btnRunTCSharp.disabled = false;
          btnRunTCSharp.textContent = '✍️ RUN LETTER T (C#)';
        }
      });
    }

    // 10. Compile & Execute C# Code Button
    const btnExecCSharp = document.getElementById('csharp-btn-execute');
    if (btnExecCSharp) {
      btnExecCSharp.addEventListener('click', async () => {
        btnExecCSharp.disabled = true;
        btnExecCSharp.textContent = 'EXECUTING C#...';

        this.logTerminal('[ROS-LYN C#] Đang biên dịch MainWindow.xaml.cs & nạp assembly YRCCommunication.dll...');
        await new Promise(r => setTimeout(r, 350));
        this.logTerminal('[ROS-LYN C#] Biên dịch thành công 0 Warning, 0 Error. Bắt đầu luồng RunAutoVisionPackagingLoop()...');

        // Execute the automated packaging cycle via simulated C# loop
        try {
          await this.app.runSinglePickAndPlaceCycle();
          this.logTerminal('[C# APP] Chu trình RunAutoVisionPackagingLoop() đã hoàn tất thành công!');
        } catch (err) {
          this.logTerminal(`[C# RUNTIME EXCEPTION]: ${err.message}`);
        } finally {
          btnExecCSharp.disabled = false;
          btnExecCSharp.textContent = '▶ COMPILE & EXECUTE C# CODE';
        }
      });
    }
  }

  logTerminal(msg) {
    const term = document.getElementById('csharp-terminal');
    if (!term) return;
    const timeStr = new Date().toLocaleTimeString();
    term.textContent = `[${timeStr}] ${msg}`;
  }

  updateCurrentPositionUI() {
    if (!this.robotArm || !this.kinematics) return;
    const q = this.robotArm.getJointAngles();
    const fk = this.kinematics.forwardKinematics(q);

    const setT = (id, val) => {
      const el = document.getElementById(id);
      if (el) el.textContent = val;
    };

    setT('cur-x', (fk.tcp_m.x * 1000).toFixed(1));
    setT('cur-y', (fk.tcp_m.y * 1000).toFixed(1));
    setT('cur-z', (fk.tcp_m.z * 1000).toFixed(1));
    setT('cur-rx', `${fk.orientation_deg.rx.toFixed(1)}°`);
    setT('cur-ry', `${fk.orientation_deg.ry.toFixed(1)}°`);
    setT('cur-rz', `${fk.orientation_deg.rz.toFixed(1)}°`);

    // Robot running badge
    const statusEl = document.getElementById('gui-robot-status');
    if (statusEl) {
      const isBusy = this.app.isSingleCycleRunning || (this.planner && this.planner.activeTrajectories.size > 0);
      statusEl.textContent = isBusy ? 'Running' : 'Stop';
      statusEl.style.color = isBusy ? '#facc15' : '#4ade80';
    }
  }

  moveArmToPose(targetPose, speedMmSec = 150.0) {
    if (!this.robotArm || !this.kinematics || !this.planner) return;

    const startPose = this.kinematics.forwardKinematics(this.robotArm.getJointAngles());
    const dist = Math.sqrt(
      Math.pow(targetPose.x - startPose.tcp_m.x, 2) +
      Math.pow(targetPose.y - startPose.tcp_m.y, 2) +
      Math.pow(targetPose.z - startPose.tcp_m.z, 2)
    );

    const duration = Math.max(0.22, (dist * 1000.0) / Math.max(speedMmSec, 350.0));
    const traj = this.planner.createMOVL(
      { x: startPose.tcp_m.x, y: startPose.tcp_m.y, z: startPose.tcp_m.z, rx: startPose.orientation_deg.rx, ry: startPose.orientation_deg.ry, rz: startPose.orientation_deg.rz },
      targetPose,
      this.kinematics,
      { duration, degree: 5, isWorld: true, initialJoints: this.robotArm.getJointAngles() }
    );

    this.planner.startTrajectory('csharp-gui-move', traj, {
      onUpdate: (data) => {
        if (data.ikResult && data.ikResult.success) {
          const q = data.ikResult.jointsDeg;
          this.robotArm.setJointAngles(q.s, q.l, q.u, q.r, q.b, q.t);
          this.app.updateArmUIFromModel();
          this.updateCurrentPositionUI();
        }
      },
      onComplete: () => {
        this.updateCurrentPositionUI();
      }
    });
  }

  nudgeAxis(axisIdx, dir) {
    if (!this.robotArm || !this.kinematics) return;

    const curPose = this.kinematics.forwardKinematics(this.robotArm.getJointAngles());
    const target = {
      x: curPose.tcp_m.x,
      y: curPose.tcp_m.y,
      z: curPose.tcp_m.z,
      rx: curPose.orientation_deg.rx,
      ry: curPose.orientation_deg.ry,
      rz: curPose.orientation_deg.rz
    };

    const deltaM = (this.nudgeDist * 0.001) * dir;
    const deltaDeg = this.nudgeDist * dir;

    if (axisIdx === 0) target.x += deltaM;
    if (axisIdx === 1) target.y += deltaM;
    if (axisIdx === 2) target.z += deltaM;
    if (axisIdx === 3) target.rx += deltaDeg;
    if (axisIdx === 4) target.ry += deltaDeg;
    if (axisIdx === 5) target.rz += deltaDeg;

    this.moveArmToPose(target, this.nudgeSpeed);
  }
}
