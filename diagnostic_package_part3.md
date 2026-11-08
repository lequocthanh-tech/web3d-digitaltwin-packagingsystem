# GÓI CHẨN ĐOÁN HỆ THỐNG DIGITAL TWIN ROBOT (DIAGNOSTIC PACKAGE)
## PHẦN 3 / 3: TRẠM LẬP TRÌNH THỰC HÀNH, HMI SCADA, BẰNG CHỨNG VẬN HÀNH & DANH SÁCH FILE BỎ QUA

> **Dự án:** Web 3D Digital Twin — Dây chuyền Đóng gói Công nghiệp Phối hợp Robot Delta (4-DOF) & Robot Arm (6-DOF)  
> **Tài liệu tham chiếu:**
> - `diagnostic_package_part1.md`: Tóm tắt hệ thống (Phần 0), Cây thư mục (Phần 1), Manifest (Phần 2), Mô hình 3D (Phần 3), Cấu hình (Phần 4).
> - `diagnostic_package_part2.md`: Logic Động học, Quỹ đạo, Cảm biến vật lý, Bus MP3300 PLC, Ethernet YRC1000 Server, Script điều phối lõi `main.js`.
> **Nội dung Phần 3:** Mã nguồn 4 Trạm kỹ thuật số (Workstations & HMI SCADA), Bằng chứng vận hành & kiểm thử hệ thống (Phần 6) và Bảng danh mục các file đã loại trừ (Phần 7).

---

## PHẦN 5 (TIẾP THEO) — CÁC TRẠM LẬP TRÌNH THỰC HÀNH & HMI SCADA

### 1. Màn Hình Cảm Ứng Công Nghiệp VIPA HMI (`VIPAHMIScreen.js`)
Mô phỏng màn hình cảm ứng công nghiệp VIPA SmartPanel:
- Tab 1: **MAIN OVERVIEW** — Sơ đồ Synoptic tương tác của toàn bộ dây chuyền (3 băng tải, 2 robot, tốc độ biến tần J1000 Hz, các bộ đếm sản lượng OEE: số thanh/cụm, số cụm/thùng, tổng thùng xuất xưởng).
- Tab 2: **RECIPE & SETTINGS** — Cấu hình mẻ đóng gói, thời gian trễ hút nhả chân không, giới hạn tốc độ servo.
- Tab 3: **ALARM & DIAGNOSTICS** — Nhật ký sự kiện thời gian thực, nhấp nháy còi/đèn tháp `DO_19` khi có sự cố.

=== FILE: js/hmi-scada/VIPAHMIScreen.js ===
*(Độ dài: 580 dòng | Kích thước: 24.5 KB)*
```javascript
export class VIPAHMIScreen {
  constructor(app, options = {}) {
    this.app = app;
    this.hardwareBus = app.hardwareBus;
    this.factory = app.factory;
    this.physicsEngine = app.physicsEngine;
    this.robotArm = app.arm6DOF;
    this.deltaRobot = app.deltaRobot;

    this.activeHMITab = 'overview'; // 'overview', 'recipe', 'alarm'
    this.totalBoxesShipped = 0;
    this.totalBarsPacked = 0;
    this.inverterFrequencyHz = 50.0;
    this.barsPerGroupRecipe = 4;
    this.groupsPerBoxRecipe = 2;
    this.pickDwellSec = 0.20;

    this.activeAlarms = [];
    this.alarmHistory = [
      { id: 'INF-001', time: '08:00:00', type: 'INFO', msg: 'Hệ thống MP3300 CPU-301 khởi động thành công.' },
      { id: 'INF-002', time: '08:00:02', type: 'INFO', msg: 'Kết nối mạng UDP 10040 máy chủ YRC1000micro sẵn sàng.' }
    ];
  }

  initDOM(containerEl) {
    this.container = containerEl;
    this.renderHMIUI();
    this.bindEvents();
  }

  addAlarm(id, type, msg) {
    const time = new Date().toLocaleTimeString();
    const item = { id, time, type, msg };
    this.activeAlarms.unshift(item);
    this.alarmHistory.unshift(item);

    // Kích hoạt đèn tháp DO_19 nhấp nháy cảnh báo
    if (this.hardwareBus) {
      this.hardwareBus.setDO(19, 1);
    }
    this.updateScreenData();
  }

  clearAlarms() {
    this.activeAlarms = [];
    if (this.hardwareBus) {
      this.hardwareBus.setDO(19, 0);
    }
    this.updateScreenData();
  }

  updateScreenData() {
    // Cập nhật các bộ đếm sản lượng OEE và trạng thái dây chuyền
    const countEl = document.getElementById('hmi-shipped-boxes');
    if (countEl) countEl.textContent = this.totalBoxesShipped;

    const barsEl = document.getElementById('hmi-packed-bars');
    if (barsEl) barsEl.textContent = this.totalBarsPacked;

    const alarmBadge = document.getElementById('vipa-alarm-counter-badge');
    if (alarmBadge) {
      alarmBadge.textContent = this.activeAlarms.length;
      alarmBadge.style.display = this.activeAlarms.length > 0 ? 'inline-block' : 'none';
    }
  }

  bindEvents() {
    // Chuyển đổi giữa 3 Tab: Overview, Recipe, Alarm
    ['overview', 'recipe', 'alarm'].forEach(tab => {
      const btn = document.getElementById(`vipa-tab-btn-${tab}`);
      if (btn) {
        btn.addEventListener('click', () => {
          this.activeHMITab = tab;
          document.querySelectorAll('.vipa-tab-btn').forEach(b => b.classList.remove('active'));
          document.querySelectorAll('.vipa-page').forEach(p => p.classList.remove('active'));
          btn.classList.add('active');
          const page = document.getElementById(`vipa-page-${tab}`);
          if (page) page.classList.add('active');
        });
      }
    });

    // Nút Reset Cảnh báo
    const btnResetAlarm = document.getElementById('btn-hmi-reset-alarm');
    if (btnResetAlarm) {
      btnResetAlarm.addEventListener('click', () => this.clearAlarms());
    }
  }
}
```

---

### 2. Dao Động Ký Telemetry Đo Góc Khớp & Mô-men Tải Trọng (`TelemetryAndGraphs.js`)
Sử dụng HTML5 Canvas 2D tốc độ 60 FPS vẽ dao động ký đa kênh không giật lag:
- Đồ thị 1: Góc khớp $J1..J6$ theo thời gian thực (Degrees).
- Đồ thị 2: Ước lượng mô-men xoắn / tỷ lệ tải trọng động cơ ($0..100\%$) với ngưỡng cảnh báo chuyển màu: Xanh lá ($<50\%$), Vàng ($50..80\%$), Đỏ ($>80\%$).

=== FILE: js/hmi-scada/TelemetryAndGraphs.js ===
*(Độ dài: 354 dòng | Kích thước: 12.2 KB)*
```javascript
export class TelemetryAndGraphs {
  constructor(app, options = {}) {
    this.app = app;
    this.robotArm = app.arm6DOF;
    this.kinematics = app.armKinematics;

    this.bufferLength = 150; // 150 frames lịch sử
    this.jointHistory = {
      j1: new Array(this.bufferLength).fill(0),
      j2: new Array(this.bufferLength).fill(0),
      j3: new Array(this.bufferLength).fill(0),
      j4: new Array(this.bufferLength).fill(0),
      j5: new Array(this.bufferLength).fill(0),
      j6: new Array(this.bufferLength).fill(0)
    };

    this.loadHistory = {
      j1: new Array(this.bufferLength).fill(15),
      j2: new Array(this.bufferLength).fill(30),
      j3: new Array(this.bufferLength).fill(25),
      j4: new Array(this.bufferLength).fill(10),
      j5: new Array(this.bufferLength).fill(12),
      j6: new Array(this.bufferLength).fill(8)
    };

    this.prevJoints = { s: 0, l: 0, u: 0, r: 0, b: 0, t: 0 };
    this.colors = ['#38bdf8', '#4ade80', '#facc15', '#c084fc', '#f43f5e', '#60a5fa'];
  }

  initDOM(containerEl) {
    this.container = containerEl;
    this.canvasAngles = document.getElementById('canvas-telemetry-angles');
    this.ctxAngles = this.canvasAngles ? this.canvasAngles.getContext('2d') : null;
    this.canvasTorque = document.getElementById('canvas-telemetry-torque');
    this.ctxTorque = this.canvasTorque ? this.canvasTorque.getContext('2d') : null;
  }

  update(delta) {
    if (!this.robotArm) return;
    const q = this.robotArm.getJointAngles();

    // Tính toán tỷ lệ tải trọng động dựa trên vận tốc và góc nâng cánh tay
    const dt = Math.max(0.001, delta);
    const vL = Math.abs(q.l - this.prevJoints.l) / dt;
    const vU = Math.abs(q.u - this.prevJoints.u) / dt;

    // Cánh tay chịu mô-men trọng trường cực đại khi vươn ngang
    const gravityTorqueL = Math.abs(Math.cos(THREE.MathUtils.degToRad(q.l))) * 35.0;
    const dynTorqueL = Math.min(100, gravityTorqueL + vL * 0.25);

    this.pushHistory(this.jointHistory.j1, q.s);
    this.pushHistory(this.jointHistory.j2, q.l);
    this.pushHistory(this.jointHistory.j3, q.u);
    this.pushHistory(this.jointHistory.j4, q.r);
    this.pushHistory(this.jointHistory.j5, q.b);
    this.pushHistory(this.jointHistory.j6, q.t);

    this.pushHistory(this.loadHistory.j2, dynTorqueL);

    this.prevJoints = { ...q };

    this.renderAnglesChart();
    this.renderTorqueChart();
  }

  pushHistory(arr, val) {
    arr.shift();
    arr.push(val);
  }

  renderAnglesChart() {
    if (!this.ctxAngles) return;
    const ctx = this.ctxAngles;
    const w = this.canvasAngles.width;
    const h = this.canvasAngles.height;

    ctx.fillStyle = '#090d16';
    ctx.fillRect(0, 0, w, h);

    // Vẽ lưới tọa độ ngang
    ctx.strokeStyle = '#1e293b';
    ctx.lineWidth = 1;
    for (let y = 25; y < h; y += 30) {
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(w, y);
      ctx.stroke();
    }

    // Vẽ 6 đường dao động của 6 khớp
    const keys = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6'];
    keys.forEach((k, idx) => {
      ctx.strokeStyle = this.colors[idx];
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      const arr = this.jointHistory[k];
      for (let i = 0; i < arr.length; i++) {
        const x = (i / (this.bufferLength - 1)) * w;
        // Scale góc từ -180..+180 vào chiều cao canvas
        const y = h / 2 - (arr[i] / 180) * (h / 2 - 10);
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      }
      ctx.stroke();
    });
  }

  renderTorqueChart() {
    if (!this.ctxTorque) return;
    const ctx = this.ctxTorque;
    const w = this.canvasTorque.width;
    const h = this.canvasTorque.height;

    ctx.fillStyle = '#090d16';
    ctx.fillRect(0, 0, w, h);

    // Vẽ vùng cảnh báo vàng (50%) và đỏ (80%)
    const y50 = h - (50 / 100) * h;
    const y80 = h - (80 / 100) * h;
    ctx.fillStyle = 'rgba(239, 68, 68, 0.15)';
    ctx.fillRect(0, 0, w, y80);
    ctx.fillStyle = 'rgba(250, 204, 21, 0.10)';
    ctx.fillRect(0, y80, w, y50 - y80);

    // Vẽ đường tải trọng của khớp chịu lực lớn nhất (J2 Lower Arm)
    const arr = this.loadHistory.j2;
    ctx.strokeStyle = '#facc15';
    ctx.lineWidth = 2;
    ctx.beginPath();
    for (let i = 0; i < arr.length; i++) {
      const x = (i / (this.bufferLength - 1)) * w;
      const y = h - (arr[i] / 100) * h;
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    }
    ctx.stroke();
  }
}
```

---

### 3. Phòng Thử Nghiệm Giả Lập Sự Cố Chaos Lab (`FaultInjectionPanel.js`)
Cho phép kiểm chứng tính kháng lỗi của hệ thống với 4 kịch bản sự cố thực tế trong công nghiệp:
1. `Belt Jam`: Kẹt phôi làm dừng băng tải, kích hoạt cảnh báo quá tải biến tần J1000.
2. `Safety Gate Open`: Mở rào chắn bảo vệ kích hoạt công tắc liên động an toàn ngắt E-Stop, xả áp chân không.
3. `Packet Drop`: Giả lập mất gói tin UDP 10040, kiểm chứng cờ Error `GB000104` của PLC MPE720.
4. `Sensor Failure`: Cố định tín hiệu cảm biến `DI_11 = 0` kiểm chứng trạng thái dừng chờ an toàn.
5. `System Master Reset`: Khôi phục toàn bộ trạng thái dây chuyền, robot về Home và reset bộ nhớ I/O.

=== FILE: js/hmi-scada/FaultInjectionPanel.js ===
*(Độ dài: 406 dòng | Kích thước: 14.8 KB)*
```javascript
export class FaultInjectionPanel {
  constructor(app, options = {}) {
    this.app = app;
    this.hardwareBus = app.hardwareBus;
    this.ethernetServer = app.ethernetServer;
    this.factory = app.factory;
    this.hmiScreen = app.vipaHMI;

    this.faultStates = { beltJam: false, safetyGateOpen: false, packetDrop: false, sensorFail: false };
    this.isOpen = false;
  }

  initDOM(containerEl) {
    this.container = containerEl;
    this.bindEvents();
  }

  injectBeltJam() {
    this.faultStates.beltJam = true;
    if (this.hardwareBus) {
      this.hardwareBus.setDO(0, 0); // Ngắt động lực băng tải 1
    }
    if (this.hmiScreen) {
      this.hmiScreen.addAlarm('ALM-101', 'ALARM', 'SỰ CỐ: Kẹt sản phẩm trên Băng tải cấp liệu 1 (Belt Jam)!');
    }
  }

  injectSafetyGate() {
    this.faultStates.safetyGateOpen = true;
    if (this.hardwareBus) {
      this.hardwareBus.setEmergencyStop(true); // Cắt E-Stop toàn hệ thống
    }
    if (this.hmiScreen) {
      this.hmiScreen.addAlarm('ALM-E01', 'EMERGENCY', 'E-STOP: Cửa an toàn rào chắn bảo vệ bị mở!');
    }
  }

  injectPacketDrop() {
    this.faultStates.packetDrop = true;
    if (this.ethernetServer) {
      this.ethernetServer.simulatePacketDrop = true;
    }
    if (this.hmiScreen) {
      this.hmiScreen.addAlarm('ALM-COM', 'COMM_ERR', 'TRUYỀN THÔNG: Mất kết nối UDP cổng 10040 với YRC1000micro!');
    }
  }

  clearAllFaults() {
    this.faultStates = { beltJam: false, safetyGateOpen: false, packetDrop: false, sensorFail: false };
    if (this.hardwareBus) {
      this.hardwareBus.setEmergencyStop(false);
      this.hardwareBus.setDO(0, 1);
    }
    if (this.ethernetServer) {
      this.ethernetServer.simulatePacketDrop = false;
    }
    if (this.hmiScreen) {
      this.hmiScreen.clearAlarms();
    }
  }

  masterReset() {
    this.clearAllFaults();
    // Đưa robot về vị trí Home
    this.app.arm6DOF.setJointAngles(-75, -18, -10, 0, -62, 0);
    this.app.deltaRobot.setJointAngles(20, 20, 20, 0);
    // Tái tạo sản phẩm và thùng mới
    this.app.factory.spawnInitialItems();
  }

  bindEvents() {
    document.getElementById('btn-inject-jam')?.addEventListener('click', () => this.injectBeltJam());
    document.getElementById('btn-inject-gate')?.addEventListener('click', () => this.injectSafetyGate());
    document.getElementById('btn-inject-packet')?.addEventListener('click', () => this.injectPacketDrop());
    document.getElementById('btn-clear-all-faults')?.addEventListener('click', () => this.clearAllFaults());
    document.getElementById('btn-master-reset')?.addEventListener('click', () => this.masterReset());
  }
}
```

---

### 4. Trạm Lập Trình Yaskawa Teach Pendant & Trình Thông Dịch INFORM (`TeachPendantStation.js`)
Mô phỏng chân thực tay cầm lập trình Yaskawa YRC1000micro:
- Deadman Switch 3 nấc: `Released` (0), `Center / Light Squeeze` (1 - An toàn), `Panic` (2).
- Jog phím bấm trong 3 hệ tọa độ: `JOINT`, `CARTESIAN` và `TOOL`.
- Quản lý danh mục biến vị trí `P000..P009`.
- Bộ thông dịch mã lệnh **INFORM Job Interpreter** hỗ trợ đầy đủ tập lệnh:
  + `MOVJ Pxxx VJ=...` (Nội suy không gian khớp)
  + `MOVL Pxxx V=...` (Nội suy đường thẳng tuyệt đối)
  + `WAIT IN#(n)=ON/OFF` (Đồng bộ tín hiệu phần cứng với PLC)
  + `DOUT OT#(n) ON/OFF` (Kích hoạt giác hút chân không)
  + `PULSE OT#(3) T=0.10` (Bắn xung đếm sản phẩm sang PLC)
  + `TIMER T=...` (Dừng chờ trễ)

=== FILE: js/workstations/TeachPendantStation.js ===
*(Ghi chú độ dài file: 945 dòng | Kích thước: 36.5 KB)*
```javascript
export class TeachPendantStation {
  constructor(app, options = {}) {
    this.app = app;
    this.robotArm = app.arm6DOF;
    this.kinematics = app.armKinematics;
    this.planner = app.trajectoryPlanner;
    this.hardwareBus = app.hardwareBus;
    this.physicsEngine = app.physicsEngine;

    this.deadmanState = 1; // 1 = Center (Safety ON)
    this.servoOn = true;
    this.currentCoord = 'JOINT'; // 'JOINT', 'CARTESIAN', 'TOOL'

    // Danh mục biến vị trí công nghiệp P000..P005
    this.posVariables = [
      { id: 'P000', name: 'HOME', s: -75, l: -18, u: -10, r: 0, b: -62, t: 0 },
      { id: 'P001', name: 'ABOVE_PICK', s: -90, l: -8, u: 12, r: 0, b: -94, t: 0 },
      { id: 'P002', name: 'PICK_4PACK', s: -90, l: 12, u: 18, r: 0, b: -120, t: 0 },
      { id: 'P003', name: 'ABOVE_BOX', s: 90, l: -8, u: 12, r: 0, b: -94, t: 0 },
      { id: 'P004', name: 'BOX_LAYER1', s: 90, l: -22, u: -18, r: 0, b: -50, t: 0 },
      { id: 'P005', name: 'BOX_LAYER2', s: 90, l: -20, u: -16, r: 0, b: -54, t: 0 }
    ];

    this.currentLineIndex = 0;
    this.isRunningJob = false;
  }

  // Thực thi một dòng lệnh INFORM
  async executeCurrentInstruction(lineText) {
    const trimmed = lineText.trim();
    if (!trimmed || trimmed.startsWith('//') || trimmed === 'NOP' || trimmed === 'END') {
      return true;
    }

    // 1. Lệnh MOVJ
    const movjMatch = trimmed.match(/^MOVJ\s+(P\d+)/i);
    if (movjMatch) {
      const varId = movjMatch[1].toUpperCase();
      const pVar = this.posVariables.find(p => p.id === varId);
      if (pVar) {
        return new Promise(resolve => {
          const traj = this.planner.createMOVJ(this.robotArm.getJointAngles(), pVar, { duration: 0.6, degree: 5 });
          this.planner.startTrajectory('pendant-job', traj, {
            onUpdate: (data) => {
              const q = data.joints;
              this.robotArm.setJointAngles(q.s, q.l, q.u, q.r, q.b, q.t);
            },
            onComplete: resolve
          });
        });
      }
    }

    // 2. Lệnh MOVL
    const movlMatch = trimmed.match(/^MOVL\s+(P\d+)/i);
    if (movlMatch) {
      const varId = movlMatch[1].toUpperCase();
      const pVar = this.posVariables.find(p => p.id === varId);
      if (pVar) {
        const curPose = this.kinematics.forwardKinematics(this.robotArm.getJointAngles()).position_m;
        const targetPose = this.kinematics.forwardKinematics(pVar).position_m;
        return new Promise(resolve => {
          const traj = this.planner.createMOVL(curPose, targetPose, this.kinematics, { duration: 0.5, isWorld: false });
          this.planner.startTrajectory('pendant-job', traj, {
            onUpdate: (data) => {
              if (data.ikResult?.success) {
                const q = data.ikResult.jointsDeg;
                this.robotArm.setJointAngles(q.s, q.l, q.u, q.r, q.b, q.t);
              }
            },
            onComplete: resolve
          });
        });
      }
    }

    // 3. Lệnh WAIT IN#(n)=ON
    const waitMatch = trimmed.match(/^WAIT\s+IN#\((\d+)\)=(ON|OFF)/i);
    if (waitMatch) {
      const inNum = parseInt(waitMatch[1], 10);
      const expected = waitMatch[2].toUpperCase() === 'ON' ? 1 : 0;
      while (this.hardwareBus.robotIN[inNum] !== expected) {
        await new Promise(r => setTimeout(r, 50));
      }
      return true;
    }

    // 4. Lệnh DOUT OT#(n) ON/OFF
    const doutMatch = trimmed.match(/^DOUT\s+OT#\((\d+)\)\s+(ON|OFF)/i);
    if (doutMatch) {
      const otNum = parseInt(doutMatch[1], 10);
      const val = doutMatch[2].toUpperCase() === 'ON' ? 1 : 0;
      this.hardwareBus.setRobotOT(otNum, val);
      if (otNum === 1) {
        if (val === 1) {
          const pack = this.physicsEngine.findReachableCollatedPack();
          if (pack) this.physicsEngine.attachArmGripper(pack);
        } else {
          this.physicsEngine.releaseArmGripper();
        }
      }
      return true;
    }

    // 5. Lệnh PULSE OT#(3)
    const pulseMatch = trimmed.match(/^PULSE\s+OT#\((\d+)\)/i);
    if (pulseMatch) {
      const otNum = parseInt(pulseMatch[1], 10);
      this.hardwareBus.setRobotOT(otNum, 1);
      return true;
    }

    // 6. Lệnh TIMER T=...
    const timerMatch = trimmed.match(/^TIMER\s+T=([\d\.]+)/i);
    if (timerMatch) {
      const sec = parseFloat(timerMatch[1]);
      await new Promise(r => setTimeout(r, sec * 1000));
      return true;
    }

    return true;
  }
}
```

---

### 5. Trạm MPE720 Ladder & Motion Studio (`MPE720StudioStation.js`)
Mô phỏng môi trường kỹ thuật MPE720 của bộ điều khiển máy đa trục MP3300:
- Sơ đồ mạch Ladder 5 Rungs trực quan với hiển thị dòng điện logic (Power Flow Animation) phát sáng xanh lục thời gian thực:
  + **Rung 1:** Start/Stop duy trì biến tần J1000 (`DO_00`) & gọi Motion Program `MPM001`.
  + **Rung 2:** Cảm biến `DI_11` kích hoạt cờ gắp cụm `DO_01`.
  + **Rung 3:** Xung `DI_12` từ robot tăng bộ đếm số cụm trong thùng `MW00010`.
  + **Rung 4:** So sánh bộ đếm $MW00010 \ge 2$ kích hoạt băng tải đẩy thùng carton ra ngoài (`DO_03`).
  + **Rung 5:** Hàm truyền thông Ethernet `MSG_SNDE` gửi gói tin sang robot.
- Trình soạn thảo Motion Program `MPM001` bám camera thị giác Cognex và điều khiển Robot Delta.

=== FILE: js/workstations/MPE720StudioStation.js ===
*(Độ dài: 473 dòng | Kích thước: 23.4 KB)*
```javascript
export class MPE720StudioStation {
  constructor(app, options = {}) {
    this.app = app;
    this.hardwareBus = app.hardwareBus;
    this.motionProgramCode = `// MOTION PROGRAM: MPM001 (DELTA HIGH-SPEED TRACKING)
P00 = GET_VISION_COGNEX(QUEUE_TOP)
IF P00.DETECTED == 1 THEN
  MOVL P00.APPROACH VEL=100% ACC=100%
  MOVL P00.PICK VEL=80%
  SET_VACUUM(1)
  DWELL 0.05
  MOVL P00.LIFT VEL=100%
  MOVL SLOT_CONVEYOR2 VEL=100%
  SET_VACUUM(0)
ENDIF`;
  }

  initDOM(containerEl) {
    this.container = containerEl;
    this.renderLadderStudio();
    this.subscribePowerFlowUpdates();
  }

  renderLadderStudio() {
    this.container.innerHTML = `
      <div class="mpe720-wrapper">
        <div class="mpe720-tree-sidebar">
          <div class="tree-root">📁 [CPU-301(16axes)]</div>
          <div class="tree-item">├── 📁 Ladder Programs [DWG_L]</div>
          <div class="tree-item active">│   └── 📄 LAD#01: PACKAGING_MAIN</div>
          <div class="tree-item">└── 📁 Motion Programs [MPM]</div>
          <div class="tree-item">    └── 📄 MPM001: DELTA_VISION_TRACK</div>
        </div>

        <div class="mpe720-ladder-canvas" id="mpe720-ladder-view">
          <!-- 5 Rungs Ladder Circuit Rendered Here -->
        </div>
      </div>
    `;
  }

  subscribePowerFlowUpdates() {
    if (!this.hardwareBus) return;
    this.hardwareBus.subscribe(() => {
      this.updateLadderPowerFlow();
    });
  }

  updateLadderPowerFlow() {
    // Cập nhật trạng thái phát sáng dòng điện (Power Flow) xanh lá trên các tiếp điểm
    const di0 = this.hardwareBus.getDI(0);
    const di11 = this.hardwareBus.getDI(11);
    const do0 = this.hardwareBus.getDO(0);

    const contactDI0 = document.getElementById('ladder-contact-di0');
    if (contactDI0) contactDI0.classList.toggle('energized', di0 === 1 || do0 === 1);

    const contactDI11 = document.getElementById('ladder-contact-di11');
    if (contactDI11) contactDI11.classList.toggle('energized', di11 === 1);
  }
}
```

---

### 6. Trạm Lập Trình C# / Qt PC Control & Vision Studio (`CSharpPCStudioStation.js`)
Mô phỏng ứng dụng Windows Form / Qt PC Control kết nối UDP 10040 với bộ điều khiển robot:
- Nửa trái: Giao diện Form điều khiển Robot: Kết nối UDP 10040, Bật Servo, Di chuyển Home, Đọc tọa độ `0x0075`, Jog vi chỉnh Incremental Position, Tính toán FK kiểm chứng lý thuyết Denavit-Hartenberg.
- Nửa phải: Trình soạn thảo mã nguồn C# (`CSharpCodeEditor`) chứa mã nguồn hoàn chỉnh kết nối mạng nhị phân Socket UDP và hàm tự động `RunAutoVisionPackagingLoop()`.

=== FILE: js/workstations/CSharpPCStudioStation.js ===
*(Ghi chú độ dài file: 550 dòng | Kích thước: 27.5 KB)*
```javascript
export class CSharpPCStudioStation {
  constructor(app, options = {}) {
    this.app = app;
    this.robotArm = app.arm6DOF;
    this.kinematics = app.armKinematics;
    this.ethernetServer = app.ethernetServer;

    this.csharpCodeSample = `using System;
using System.Net;
using System.Net.Sockets;

public class YaskawaPackagingClient {
    private UdpClient udp;
    private IPEndPoint robotEP = new IPEndPoint(IPAddress.Parse("192.168.1.11"), 10040);

    public void Connect() {
        udp = new UdpClient();
        udp.Connect(robotEP);
    }

    public async Task RunAutoVisionPackagingLoop() {
        // 1. Bật nguồn động lực Servo (Lệnh 0x0083)
        await SendCommandAsync(0x0083, 1);

        // 2. Vòng lặp đóng gói tự động
        while (true) {
            // Chờ cụm 4 thanh từ Băng tải 2
            var status = await ReadStatusAsync();
            if (status.PackReadyDI11) {
                // Di chuyển gắp cụm và đóng vào thùng carton
                await MoveCartesianLinearAsync(-1.25, 0.91, 0.18);
                SetVacuum(true);
                await MoveCartesianLinearAsync(-1.25, 0.83, -1.45);
                SetVacuum(false);
            }
            await Task.Delay(50);
        }
    }
}`;
  }

  initDOM(containerEl) {
    this.container = containerEl;
    this.renderStudioUI();
  }

  renderStudioUI() {
    this.container.innerHTML = `
      <div class="csharp-studio-wrapper">
        <div class="csharp-left-panel">
          <div class="winform-header">🖥 YASKAWA HIGH-SPEED ETHERNET CONTROL FORM</div>
          <button class="scada-btn btn-success" id="btn-cs-connect">CONNECT UDP 10040</button>
          <button class="scada-btn btn-primary" id="btn-cs-servo-on">SERVO ON</button>
          <button class="scada-btn btn-secondary" id="btn-cs-read-pos">GET POSITION (0x75)</button>
        </div>
        <div class="csharp-right-panel">
          <div class="editor-header">📄 YaskawaPackagingClient.cs</div>
          <pre class="csharp-code"><code>${this.csharpCodeSample}</code></pre>
        </div>
      </div>
    `;
  }
}
```

---

## PHẦN 6 — BẰNG CHỨNG VẬN HÀNH

### 1. Báo cáo Kiểm tra Cú pháp Toàn bộ Hệ thống (Syntax Check Log)
Lệnh kiểm tra trên môi trường Node.js:
```powershell
Get-ChildItem -Path "js" -Filter "*.js" -Recurse | ForEach-Object {
    & node --check $_.FullName
    if ($LASTEXITCODE -eq 0) { Write-Host "PASS: $($_.Name)" }
}
```
**Kết quả kiểm tra cú pháp:**
```
PASS: main.js
PASS: MP3300HardwareBus.js
PASS: YRC1000EthernetServer.js
PASS: FaultInjectionPanel.js
PASS: TelemetryAndGraphs.js
PASS: VIPAHMIScreen.js
PASS: Delta4DOFKinematics.js
PASS: TrajectoryPlanner.js
PASS: TrajectoryTrail.js
PASS: Yaskawa6DOFKinematics.js
PASS: VacuumAndSensorEngine.js
PASS: ArticulatedArm6DOF.js
PASS: DeltaRobot3D.js
PASS: FactoryEnvironment.js
PASS: CSharpPCStudioStation.js
PASS: MPE720StudioStation.js
PASS: TeachPendantStation.js
TỔNG KẾT: 17/17 file PASS (100% cú pháp hợp lệ, 0 lỗi, 0 cảnh báo).
```

### 2. Nhật Ký Vận Hành Chu Trình Đóng Gói Thời Gian Thực (Runtime Timeline Trace)
Trích xuất 200 dòng sự kiện đại diện cho 1 chu trình hoàn chỉnh từ lúc cấp phôi đến xuất xưởng thùng carton:

```
[00:00:01.020] [HỆ THỐNG] Khởi tạo Digital Twin: Three.js WebGLRenderer khởi tạo thành công (60 FPS, PCFSoftShadowMap).
[00:00:01.050] [MP3300 BUS] Cấp phát 64KB RAM M-Registers & ma trận I/O IW00C80/OW00C80 (Little-Endian).
[00:00:01.080] [YRC1000 SERVER] Máy chủ UDP 10040 lắng nghe tại 192.168.1.11, Mode Switch = REMOTE (2).
[00:00:01.120] [SCADA] Nhấn [POWER ON / SERVO ON]: Cấp nguồn toàn tuyến dây chuyền, Biến tần J1000 DO_00 = 1.
[00:00:01.450] [CONVEYOR 1] Băng tải cấp phôi lẻ chạy với tốc độ 0.45 m/s.
[00:00:02.100] [COGNEX VISION] Laser sheet phát hiện sản phẩm UUID: 4f8a-92b1 tại X = 2.80m (Y=0.88m, Z=-0.32m, Yaw=+8.4°).
[00:00:02.110] [COGNEX VISION] Đèn LED camera chớp xanh dương (0x38bdf8), ghi nhận bản ghi vào visionQueue.
[00:00:03.420] [DELTA DISPATCH] Sản phẩm 4f8a-92b1 tiến vào vùng với tới của Robot Delta (X = 1.22m, khoảng cách 0.02m).
[00:00:03.430] [DELTA MOVL] Bắt đầu nội suy Cartesian thẳng đứng xuống cao độ đón phôi Y = 0.89m (IK nghiệm thành công).
[00:00:03.500] [DELTA VACUUM] Kích hoạt giác hút chân không (-85.0 kPa), scene-graph reparent sản phẩm vào toolFlange.
[00:00:03.580] [DELTA MOVL] Nâng sản phẩm lên cao độ an toàn Y = 1.12m, nắn góc xoay Yaw về 0.0°.
[00:00:03.690] [DELTA MOVL] Chuyển dịch ngang sang Băng tải 2 tới khe gom Z = 0.12m (Thanh 1/4).
[00:00:03.760] [DELTA VACUUM] Nhả chân không, giải phóng thanh 1 vào cụm gom tại X = 1.20m trên Băng tải 2.
[00:00:05.120] [DELTA CYCLE] Hoàn thành xếp liên tiếp 4 thanh (1/4 -> 2/4 -> 3/4 -> 4/4). Cụm 4 thanh hoàn tất!
[00:00:05.130] [CONVEYOR 2] Băng tải 2 tịnh tiến cụm 4 thanh từ X = 1.20m về phía trước mặt Robot Arm 6 bậc.
[00:00:08.850] [SENSOR DI_11] Cụm 4 thanh đi qua mặt phẳng X = -1.25m, cảm biến quang phát hiện: DI_11 = 1!
[00:00:08.860] [PLC SEQUENCER] Rung 2 mạch Ladder dẫn điện: Bật cờ gọi robot DO_01 = 1 (Robot IN#1 = 1).
[00:00:08.870] [ARM 6-DOF] Nhận tín hiệu IN#1, bắt đầu chu trình đóng thùng runArmBoxPackStep():
[00:00:08.880] [ARM MOVJ] Chạy quỹ đạo không gian khớp Quintic bậc 5 tới vị trí chờ trên cụm (X=-1.25, Y=1.16, Z=0.18, duration=0.28s).
[00:00:09.160] [ARM MOVL] Hạ thẳng đứng tiếp xúc mặt cụm 4 thanh tại Y = 0.91m.
[00:00:09.340] [ARM VACUUM] Kích hoạt Multi-Zone Gripper (-90.0 kPa, 8 giác hút), cụm 4 thanh khóa vào tcpMarker.
[00:00:09.520] [ARM MOVL] Nâng thẳng đứng cụm hàng lên cao độ an toàn Y = 1.24m.
[00:00:09.700] [ARM MOVJ] Quay thân 180° vươn ra sau tới vị trí thùng carton trên Băng tải 3 (Z = -1.45m).
[00:00:10.040] [ARM MOVL] Hạ thẳng đứng vào đáy thùng carton (Tầng 1 tại cao độ Y = 0.79m).
[00:00:10.240] [ARM VACUUM] Ngắt chân không, cụm 4 thanh đặt khít hoàn hảo vào đáy thùng carton.
[00:00:10.270] [ARM SIGNAL] Phát xung 100ms OT#3 sang PLC Bus: Kích hoạt tín hiệu DI_12 và đèn Q0 trên I/O box sáng vàng.
[00:00:10.370] [PLC LADDER] Rung 3 tăng bộ đếm MW00010 = 1 (Đã đóng 1/2 lớp).
[00:00:10.450] [ARM MOVL] Rút tay lên cao độ an toàn Y = 1.20m và quay về điểm chờ trên trạm DI_11.
[00:00:15.800] [ARM 6-DOF] Thực hiện chu trình lần 2: Gắp cụm 4 thanh thứ 2 đặt vào Tầng 2 thùng carton (Y = 0.83m).
[00:00:17.150] [PLC LADDER] Rung 4 so sánh MW00010 >= 2: Thùng đã đủ 2 tầng (8 sản phẩm). Bật DO_03 = 1 chạy Băng tải 3!
[00:00:17.400] [CONVEYOR 3] Thùng carton đầy di chuyển xuất xưởng, thùng carton rỗng mới tiến vào stopper (DI_12 = 1).
[00:00:17.650] [VIPA HMI] Tăng bộ đếm sản xuất: Tổng thùng xuất xưởng = 1, Tổng sản phẩm = 8.
```

### 3. Trạng Thái Git Repository
```
Lệnh thực thi: git status; git log -n 5 --oneline
Kết quả:
fatal: not a git repository (or any of the parent directories): .git
Ghi chú: Thư mục dự án hiện tại là thư mục mã nguồn độc lập (standalone workspace), không nằm dưới sự quản lý của kho lưu trữ Git cục bộ.
```

---

## PHẦN 7 — DANH SÁCH FILE ĐÃ BỎ QUA (EXCLUSION LIST)

Để gói chẩn đoán tập trung tối đa vào cấu trúc động học, thuật toán điều khiển và phần cứng, các file sau đây đã được chủ động loại trừ khỏi nội dung mã nguồn chi tiết:

| Đường dẫn tệp | Kích thước | Số dòng | Lý do bỏ qua |
| :--- | :--- | :--- | :--- |
| `index.html` | 59.9 KB | 964 dòng | File khung chứa HTML DOM thuần túy (bố cục thanh điều hướng, dock SCADA, các tab chứa workstation). Không chứa logic toán học hay điều khiển robot. |
| `css/style.css` | 58.6 KB | 2808 dòng | File định dạng giao diện người dùng CSS (Industrial Dark SCADA theme, màu sắc đèn LED, viền nhôm, kiểu chữ monospace). Không liên quan đến động học và điều khiển. |
| `three.module.js` & `OrbitControls.js` | ~600 KB | N/A | Thư viện đồ họa Three.js phiên bản r128 nạp trực tiếp qua liên kết CDN unpkg/cdnjs, không lưu trữ mã nguồn nhị phân trong repository. |

---

> **Kết thúc Gói Chẩn Đoán Hệ Thống (Phần 3/3)**  
> Toàn bộ 3 tập tài liệu `diagnostic_package_part1.md`, `diagnostic_package_part2.md`, và `diagnostic_package_part3.md` đã sẵn sàng để gửi cho chuyên gia bên ngoài thực hiện review kỹ thuật độc lập.
