/**
 * TelemetryAndGraphs.js
 * 
 * Real-time Industrial SCADA Telemetry & High-Performance Canvas 2D Oscilloscope
 * Tracks:
 * 1. Joint Angle Tracking (J1..J6 Degrees vs. Time)
 * 2. Joint Load & Torque Percentage (0..100% with Green -> Yellow -> Red warning threshold)
 * 3. 2D Cartesian End-Effector Trajectory Plot (TCP X-Y) - Displays Letter T Trajectory Graph
 */

import { runAutoLetterTJob } from '../workstations/JobLetterT.js';

export class TelemetryAndGraphs {
  constructor(app, options = {}) {
    this.app = app;
    this.robotArm = app.arm6DOF;
    this.kinematics = app.armKinematics;
    this.physicsEngine = app.physicsEngine;

    // Buffer length for scrolling graphs (150 frames = ~5s at 30 fps graph update)
    this.bufferLength = 150;
    
    // Joint angles history buffer
    this.jointHistory = {
      j1: new Array(this.bufferLength).fill(0),
      j2: new Array(this.bufferLength).fill(0),
      j3: new Array(this.bufferLength).fill(0),
      j4: new Array(this.bufferLength).fill(0),
      j5: new Array(this.bufferLength).fill(0),
      j6: new Array(this.bufferLength).fill(0)
    };

    // Estimated load & torque percentage (0..100)
    this.loadHistory = {
      j1: new Array(this.bufferLength).fill(15),
      j2: new Array(this.bufferLength).fill(30),
      j3: new Array(this.bufferLength).fill(25),
      j4: new Array(this.bufferLength).fill(10),
      j5: new Array(this.bufferLength).fill(12),
      j6: new Array(this.bufferLength).fill(8)
    };

    // 2D Cartesian Trajectory Buffer (X-Y coordinates in meters)
    this.trajectoryPoints = [];
    this.maxTrajectoryPoints = 600;
    this.lastRecordedTCP = { x: 0, y: 0, z: 0 };
    this.isLetterTMode = false;

    this.prevJoints = { s: 0, l: 0, u: 0, r: 0, b: 0, t: 0 };
    this.prevVelocity = { s: 0, l: 0, u: 0, r: 0, b: 0, t: 0 };

    this.canvasAngles = null;
    this.ctxAngles = null;
    this.canvasTorque = null;
    this.ctxTorque = null;
    this.canvasTraj = null;
    this.ctxTraj = null;

    this.updateThrottle = 0;
  }

  resetTrajectory() {
    this.trajectoryPoints = [];
    this.lastRecordedTCP = { x: 0, y: 0, z: 0 };
  }

  setLetterTMode(active) {
    this.isLetterTMode = !!active;
    const tag = document.getElementById('tag-letter-t-status');
    if (tag) {
      if (active) {
        tag.textContent = 'ĐANG VẼ CHỮ T...';
        tag.style.background = 'rgba(234, 179, 8, 0.2)';
        tag.style.color = '#facc15';
      } else {
        tag.textContent = 'CHỮ T SẴN SÀNG';
        tag.style.background = 'rgba(45, 212, 191, 0.2)';
        tag.style.color = '#2dd4bf';
      }
    }
  }

  initDOM(containerEl) {
    this.container = containerEl;
    this.container.innerHTML = `
      <div class="telemetry-graphs-wrapper">
        <div class="telemetry-graphs-header">
          <div style="display:flex; align-items:center; gap:6px;">
            <span style="font-size:14px;">📈</span>
            <span style="font-weight:800; color:#e2e8f0;">SCADA REAL-TIME TELEMETRY // 6-DOF MOTOR DYNAMICS & TRAJECTORY</span>
          </div>
          <div style="font-family:var(--font-mono); font-size:8.5px; color:#4ade80;">
            SAMPLING: 30 Hz // HARDWARE SENSORS & ENCODER FEEDBACK
          </div>
        </div>

        <div class="telemetry-charts-grid">
          
          <!-- Chart 1: Joint Angle Trend (deg) -->
          <div class="telemetry-chart-card">
            <div class="chart-card-title">
              <span>JOINT ANGLES OSCILLOSCOPE (J1..J6 DEGREES)</span>
              <div class="chart-legend">
                <span style="color:#38bdf8;">■ J1</span>
                <span style="color:#4ade80;">■ J2</span>
                <span style="color:#facc15;">■ J3</span>
                <span style="color:#c084fc;">■ J4</span>
                <span style="color:#f43f5e;">■ J5</span>
                <span style="color:#60a5fa;">■ J6</span>
              </div>
            </div>
            <div class="canvas-container">
              <canvas id="canvas-telemetry-angles" width="460" height="140"></canvas>
            </div>
            <div class="chart-readouts-bar" id="readouts-angles">
              <!-- Rendered by JS -->
            </div>
          </div>

          <!-- Chart 2: Dynamic Joint Load & Torque Percentage (%) -->
          <div class="telemetry-chart-card">
            <div class="chart-card-title">
              <span>ESTIMATED MOTOR TORQUE / LOAD (% NOMINAL)</span>
              <div class="chart-legend">
                <span style="color:#22c55e;">● &lt;50% Normal</span>
                <span style="color:#facc15;">● 50-80% High</span>
                <span style="color:#ef4444;">● &gt;80% Overload</span>
              </div>
            </div>
            <div class="canvas-container">
              <canvas id="canvas-telemetry-torque" width="460" height="140"></canvas>
            </div>
            <div class="chart-readouts-bar" id="readouts-torque">
              <!-- Rendered by JS -->
            </div>
          </div>

          <!-- Chart 3: 2D Cartesian Trajectory Plot (TCP X-Y // ĐỒ THỊ CHỮ T) -->
          <div class="telemetry-chart-card" id="card-telemetry-trajectory" style="grid-column: span 2;">
            <div class="chart-card-title">
              <div style="display:flex; align-items:center; gap:8px;">
                <span style="color:#2dd4bf; font-weight:800;">CARTESIAN 2D TRAJECTORY // ĐỒ THỊ QUỸ ĐẠO TCP (X-Y) // HÌNH CHỮ T</span>
                <span class="card-tag" id="tag-letter-t-status" style="background: rgba(45, 212, 191, 0.2); color: #2dd4bf; font-size: 9px; padding: 1px 6px; border-radius: 3px; font-weight:700;">CHỮ T SẴN SÀNG</span>
              </div>
              <div class="chart-legend" style="gap:8px;">
                <button class="action-btn" id="btn-telemetry-run-t" style="padding: 3px 10px; font-size: 10px; background: linear-gradient(135deg, #0d9488, #059669); border-color: #2dd4bf; color: #fff; font-weight: 700; border-radius: 3px; cursor: pointer;">✍️ CHẠY TỰ ĐỘNG VẼ CHỮ T</button>
                <button class="action-btn" id="btn-telemetry-clear-traj" style="padding: 3px 8px; font-size: 10px; background: #1e293b; border-color: #475569; color: #cbd5e1; border-radius: 3px; cursor: pointer;">Xóa Đồ Thị</button>
              </div>
            </div>
            <div class="canvas-container" style="display:flex; justify-content:center;">
              <canvas id="canvas-telemetry-trajectory" width="930" height="150" style="max-width:100%;"></canvas>
            </div>
            <div class="chart-readouts-bar" id="readouts-trajectory">
              <!-- Rendered by JS -->
            </div>
          </div>

        </div>
      </div>
    `;

    this.canvasAngles = document.getElementById('canvas-telemetry-angles');
    if (this.canvasAngles) this.ctxAngles = this.canvasAngles.getContext('2d');

    this.canvasTorque = document.getElementById('canvas-telemetry-torque');
    if (this.canvasTorque) this.ctxTorque = this.canvasTorque.getContext('2d');

    this.canvasTraj = document.getElementById('canvas-telemetry-trajectory');
    if (this.canvasTraj) this.ctxTraj = this.canvasTraj.getContext('2d');

    document.getElementById('btn-telemetry-run-t')?.addEventListener('click', () => {
      runAutoLetterTJob(this.app);
    });

    document.getElementById('btn-telemetry-clear-traj')?.addEventListener('click', () => {
      this.resetTrajectory();
      if (this.app.trajectoryTrail) {
        this.app.trajectoryTrail.clear();
      }
    });
  }

  /**
   * Sample current robot dynamics and draw charts (called in animation loop)
   */
  update(delta) {
    if (!this.robotArm) return;

    // Tối ưu CPU triệt để: Nếu container Telemetry đang ẩn (không mở Station 4), dừng hoàn toàn vẽ Canvas
    if (!this.container || this.container.offsetParent === null) {
      return;
    }

    this.updateThrottle += delta;
    if (this.updateThrottle < 0.05) return; // ~20 fps graph refresh tối ưu CPU mà vẫn siêu mượt
    this.updateThrottle = 0;

    const q = this.robotArm.getJointAngles();

    // 1. Calculate velocities and accelerations for dynamic torque estimation
    const dt = Math.max(0.016, delta);
    const vel = {
      s: (q.s - this.prevJoints.s) / dt,
      l: (q.l - this.prevJoints.l) / dt,
      u: (q.u - this.prevJoints.u) / dt,
      r: (q.r - this.prevJoints.r) / dt,
      b: (q.b - this.prevJoints.b) / dt,
      t: (q.t - this.prevJoints.t) / dt
    };

    const acc = {
      s: Math.abs((vel.s - this.prevVelocity.s) / dt),
      l: Math.abs((vel.l - this.prevVelocity.l) / dt),
      u: Math.abs((vel.u - this.prevVelocity.u) / dt),
      r: Math.abs((vel.r - this.prevVelocity.r) / dt),
      b: Math.abs((vel.b - this.prevVelocity.b) / dt),
      t: Math.abs((vel.t - this.prevVelocity.t) / dt)
    };

    this.prevJoints = { ...q };
    this.prevVelocity = { ...vel };

    // Payload multiplier: higher when holding 4-pack chocolate bars
    const hasPayload = (this.physicsEngine && this.physicsEngine.armVacuum.active);
    const payloadFactor = hasPayload ? 1.4 : 1.0;

    // Gravity component approximation based on angle
    const gravL = Math.abs(Math.cos((q.l * Math.PI) / 180));
    const gravU = Math.abs(Math.sin(((q.l + q.u) * Math.PI) / 180));

    // Dynamic load percentages
    const loadS = Math.min(100, Math.max(5, (12 + acc.s * 0.12) * payloadFactor));
    const loadL = Math.min(100, Math.max(8, (20 + gravL * 35 + acc.l * 0.18) * payloadFactor));
    const loadU = Math.min(100, Math.max(8, (18 + gravU * 30 + acc.u * 0.15) * payloadFactor));
    const loadR = Math.min(100, Math.max(4, (10 + acc.r * 0.08) * payloadFactor));
    const loadB = Math.min(100, Math.max(4, (12 + acc.b * 0.08) * payloadFactor));
    const loadT = Math.min(100, Math.max(3, (8 + acc.t * 0.06) * payloadFactor));

    // Push into history ring buffers
    const pushVal = (arr, val) => {
      arr.shift();
      arr.push(val);
    };

    pushVal(this.jointHistory.j1, q.s);
    pushVal(this.jointHistory.j2, q.l);
    pushVal(this.jointHistory.j3, q.u);
    pushVal(this.jointHistory.j4, q.r);
    pushVal(this.jointHistory.j5, q.b);
    pushVal(this.jointHistory.j6, q.t);

    pushVal(this.loadHistory.j1, loadS);
    pushVal(this.loadHistory.j2, loadL);
    pushVal(this.loadHistory.j3, loadU);
    pushVal(this.loadHistory.j4, loadR);
    pushVal(this.loadHistory.j5, loadB);
    pushVal(this.loadHistory.j6, loadT);

    // 2. Sample 2D Cartesian TCP Position (X-Y)
    const curTCP = this.robotArm.getTCPWorldPosition ? this.robotArm.getTCPWorldPosition() : null;
    if (curTCP) {
      const dist = Math.hypot(curTCP.x - this.lastRecordedTCP.x, curTCP.y - this.lastRecordedTCP.y);
      if (dist > 0.003) { // 3mm resolution
        this.trajectoryPoints.push({ x: curTCP.x, y: curTCP.y, z: curTCP.z, t: performance.now() });
        if (this.trajectoryPoints.length > this.maxTrajectoryPoints) {
          this.trajectoryPoints.shift();
        }
        this.lastRecordedTCP = { x: curTCP.x, y: curTCP.y, z: curTCP.z };
      }
    }

    // Draw all 3 canvases
    this.drawAnglesChart(q);
    this.drawTorqueChart({ loadS, loadL, loadU, loadR, loadB, loadT });
    if (curTCP) {
      this.drawTrajectoryChart(curTCP);
    }
  }

  drawTrajectoryChart(curTCP) {
    if (!this.ctxTraj || !this.canvasTraj) return;
    const ctx = this.ctxTraj;
    const w = this.canvasTraj.width;
    const h = this.canvasTraj.height;

    // Background
    ctx.fillStyle = '#02040a';
    ctx.fillRect(0, 0, w, h);

    // Coordinate mapping bounds
    // X range: -1.60m to -0.90m (width 0.70m)
    // Y range: 0.75m to 1.35m (height 0.60m)
    const xMin = -1.60;
    const xMax = -0.90;
    const yMin = 0.75;
    const yMax = 1.35;

    const padL = 60;
    const padR = 40;
    const padT = 20;
    const padB = 25;
    const plotW = w - padL - padR;
    const plotH = h - padT - padB;

    const mapX = (x) => padL + ((x - xMin) / (xMax - xMin)) * plotW;
    const mapY = (y) => padT + ((yMax - y) / (yMax - yMin)) * plotH;

    // 1. Grid lines
    ctx.lineWidth = 1;
    ctx.strokeStyle = '#1e293b';

    // X Grid (every 100mm = 0.10m)
    for (let xVal = -1.50; xVal <= -1.00; xVal += 0.10) {
      const cx = mapX(xVal);
      ctx.beginPath();
      ctx.moveTo(cx, padT);
      ctx.lineTo(cx, padT + plotH);
      ctx.stroke();

      ctx.fillStyle = '#64748b';
      ctx.font = '8.5px monospace';
      ctx.fillText(`${(xVal * 1000).toFixed(0)}`, cx - 14, padT + plotH + 14);
    }

    // Y Grid (every 100mm = 0.10m)
    for (let yVal = 0.80; yVal <= 1.30; yVal += 0.10) {
      const cy = mapY(yVal);
      ctx.beginPath();
      ctx.moveTo(padL, cy);
      ctx.lineTo(padL + plotW, cy);
      ctx.stroke();

      ctx.fillStyle = '#64748b';
      ctx.font = '8.5px monospace';
      ctx.fillText(`${(yVal * 1000).toFixed(0)}`, 14, cy + 3);
    }

    // Highlight Centerlines for Letter T Target Zones
    // Target T Horizontal Bar: Y = 1.20m, X from -1.45 to -1.05
    // Target T Vertical Stem: X = -1.25m, Y from 1.20 to 0.88
    const barY = mapY(1.20);
    const stemX = mapX(-1.25);
    const barLeftX = mapX(-1.45);
    const barRightX = mapX(-1.05);
    const stemBottomY = mapY(0.88);

    // Ghost Template Guide for Letter T (Làm đậm nét để người dùng dễ quan sát mục tiêu)
    ctx.strokeStyle = 'rgba(45, 212, 191, 0.30)';
    ctx.lineWidth = 12;
    ctx.lineCap = 'round';
    ctx.beginPath();
    ctx.moveTo(barLeftX, barY);
    ctx.lineTo(barRightX, barY);
    ctx.moveTo(stemX, barY);
    ctx.lineTo(stemX, stemBottomY);
    ctx.stroke();

    // 2. Draw Recorded TCP Trajectory Trail (Nét vẽ đậm đà, phát sáng Neon Cyan rực rỡ)
    if (this.trajectoryPoints.length > 1) {
      ctx.shadowColor = '#00ffff';
      ctx.shadowBlur = 14;
      ctx.strokeStyle = '#00ffff';
      ctx.lineWidth = 5.0; // Tăng từ 3 lên 5px siêu rõ nét
      ctx.lineCap = 'round';
      ctx.lineJoin = 'round';
      ctx.beginPath();

      for (let i = 0; i < this.trajectoryPoints.length; i++) {
        const pt = this.trajectoryPoints[i];
        const px = mapX(pt.x);
        const py = mapY(pt.y);
        if (i === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      }
      ctx.stroke();
      ctx.shadowBlur = 0; // Reset shadow
    }

    // 3. Draw Active TCP Cursor
    const curX = mapX(curTCP.x);
    const curY = mapY(curTCP.y);

    ctx.fillStyle = '#facc15';
    ctx.beginPath();
    ctx.arc(curX, curY, 4.5, 0, Math.PI * 2);
    ctx.fill();

    ctx.strokeStyle = '#fff';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(curX, curY, 7.0, 0, Math.PI * 2);
    ctx.stroke();

    // 4. Shape Verification: Check if trajectory has formed letter T
    let hasLeft = false;
    let hasRight = false;
    let hasBottom = false;
    let hasMidTop = false;

    for (const pt of this.trajectoryPoints) {
      if (pt.x <= -1.40 && Math.abs(pt.y - 1.20) < 0.08) hasLeft = true;
      if (pt.x >= -1.10 && Math.abs(pt.y - 1.20) < 0.08) hasRight = true;
      if (Math.abs(pt.x - (-1.25)) < 0.08 && pt.y <= 0.92) hasBottom = true;
      if (Math.abs(pt.x - (-1.25)) < 0.08 && Math.abs(pt.y - 1.20) < 0.06) hasMidTop = true;
    }

    const isLetterTComplete = (hasLeft && hasRight && hasBottom && hasMidTop);
    const statusTag = document.getElementById('tag-letter-t-status');
    if (statusTag) {
      if (isLetterTComplete) {
        statusTag.textContent = '★ HOÀN TẤT ĐỒ THỊ CHỮ T (VERIFIED)';
        statusTag.style.background = 'rgba(34, 197, 94, 0.25)';
        statusTag.style.color = '#4ade80';
      }
    }

    // Watermark Label on Canvas
    if (isLetterTComplete) {
      ctx.fillStyle = 'rgba(74, 222, 128, 0.9)';
      ctx.font = 'bold 12px monospace';
      ctx.fillText('QUỸ ĐẠO: CHỮ "T" [ĐÃ XÁC THỰC HOÀN THÀNH]', padL + 12, padT + 22);
    } else {
      ctx.fillStyle = 'rgba(148, 163, 184, 0.6)';
      ctx.font = '10px monospace';
      ctx.fillText('ĐỒ THỊ QUỸ ĐẠO TỌA ĐỘ TCP (X vs Y)', padL + 12, padT + 18);
    }

    // Update readouts bar
    const bar = document.getElementById('readouts-trajectory');
    if (bar) {
      bar.innerHTML = `
        <span>TCP X: <b style="color:#38bdf8;">${(curTCP.x * 1000).toFixed(0)} mm</b></span>
        <span>TCP Y: <b style="color:#4ade80;">${(curTCP.y * 1000).toFixed(0)} mm</b></span>
        <span>TCP Z: <b style="color:#facc15;">${(curTCP.z * 1000).toFixed(0)} mm</b></span>
        <span>MẪU VẼ: <b>${this.trajectoryPoints.length} pts</b></span>
        <span>HÌNH DẠNG: <b style="color:${isLetterTComplete ? '#4ade80' : '#facc15'};">${isLetterTComplete ? 'CHỮ "T" HOÀN HẢO' : (this.trajectoryPoints.length > 5 ? 'ĐANG VẼ...' : 'CHỜ CHẠY')}</b></span>
      `;
    }
  }

  drawAnglesChart(curQ) {
    if (!this.ctxAngles || !this.canvasAngles) return;
    const ctx = this.ctxAngles;
    const w = this.canvasAngles.width;
    const h = this.canvasAngles.height;

    // Background
    ctx.fillStyle = '#02040a';
    ctx.fillRect(0, 0, w, h);

    // Grid lines
    ctx.strokeStyle = '#1e293b';
    ctx.lineWidth = 1;
    for (let y = 25; y < h; y += 30) {
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(w, y);
      ctx.stroke();
    }
    // Zero center line
    const yZero = h / 2;
    ctx.strokeStyle = '#334155';
    ctx.beginPath();
    ctx.moveTo(0, yZero);
    ctx.lineTo(w, yZero);
    ctx.stroke();

    // Trace settings: min -180, max +180
    const traces = [
      { data: this.jointHistory.j1, color: '#38bdf8' },
      { data: this.jointHistory.j2, color: '#4ade80' },
      { data: this.jointHistory.j3, color: '#facc15' },
      { data: this.jointHistory.j4, color: '#c084fc' },
      { data: this.jointHistory.j5, color: '#f43f5e' },
      { data: this.jointHistory.j6, color: '#60a5fa' }
    ];

    traces.forEach(t => {
      ctx.strokeStyle = t.color;
      ctx.lineWidth = 2.8; // Đường nét đậm, rõ ràng khi robot chuyển động
      ctx.beginPath();
      const stepX = w / (this.bufferLength - 1);
      for (let i = 0; i < this.bufferLength; i++) {
        const val = t.data[i];
        // Normalize -180..+180 to height
        const normY = yZero - (val / 160.0) * (h * 0.45);
        if (i === 0) ctx.moveTo(0, normY);
        else ctx.lineTo(i * stepX, normY);
      }
      ctx.stroke();
    });

    // Update readout text
    const bar = document.getElementById('readouts-angles');
    if (bar) {
      bar.innerHTML = `
        <span>J1: <b style="color:#38bdf8;">${curQ.s.toFixed(1)}°</b></span>
        <span>J2: <b style="color:#4ade80;">${curQ.l.toFixed(1)}°</b></span>
        <span>J3: <b style="color:#facc15;">${curQ.u.toFixed(1)}°</b></span>
        <span>J4: <b style="color:#c084fc;">${curQ.r.toFixed(1)}°</b></span>
        <span>J5: <b style="color:#f43f5e;">${curQ.b.toFixed(1)}°</b></span>
        <span>J6: <b style="color:#60a5fa;">${curQ.t.toFixed(1)}°</b></span>
      `;
    }
  }

  drawTorqueChart(currentLoads) {
    if (!this.ctxTorque || !this.canvasTorque) return;
    const ctx = this.ctxTorque;
    const w = this.canvasTorque.width;
    const h = this.canvasTorque.height;

    // Background
    ctx.fillStyle = '#02040a';
    ctx.fillRect(0, 0, w, h);

    // Grid lines (0%, 50%, 80%, 100%)
    ctx.strokeStyle = '#1e293b';
    ctx.lineWidth = 1;
    [0.2, 0.5, 0.8].forEach(pct => {
      const y = h - (pct * h);
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(w, y);
      ctx.stroke();
    });

    // Draw Overload zone shading (>80%)
    ctx.fillStyle = 'rgba(239, 68, 68, 0.1)';
    ctx.fillRect(0, 0, w, h * 0.2);

    // Draw rolling wave for Max Active Load
    ctx.strokeStyle = '#38bdf8';
    ctx.lineWidth = 3.0; // Nét đậm rõ nét
    ctx.beginPath();
    const stepX = w / (this.bufferLength - 1);
    for (let i = 0; i < this.bufferLength; i++) {
      // Find max load among 6 joints at frame i
      const maxL = Math.max(
        this.loadHistory.j1[i],
        this.loadHistory.j2[i],
        this.loadHistory.j3[i],
        this.loadHistory.j4[i],
        this.loadHistory.j5[i],
        this.loadHistory.j6[i]
      );
      const y = h - (maxL / 100.0) * h;
      if (i === 0) ctx.moveTo(0, y);
      else ctx.lineTo(i * stepX, y);
    }
    ctx.stroke();

    // 6 Vertical Real-time Bar Gauges on the right side
    const barWidth = 14;
    const barGap = 6;
    const startX = w - (6 * (barWidth + barGap)) - 10;
    const loadsArr = [
      currentLoads.loadS,
      currentLoads.loadL,
      currentLoads.loadU,
      currentLoads.loadR,
      currentLoads.loadB,
      currentLoads.loadT
    ];

    loadsArr.forEach((ld, idx) => {
      const bx = startX + idx * (barWidth + barGap);
      const barH = (ld / 100.0) * (h - 10);
      const by = h - barH - 4;

      // Color selection
      let color = '#22c55e';
      if (ld >= 80) color = '#ef4444';
      else if (ld >= 50) color = '#facc15';

      ctx.fillStyle = 'rgba(255, 255, 255, 0.05)';
      ctx.fillRect(bx, 4, barWidth, h - 8);

      ctx.fillStyle = color;
      ctx.fillRect(bx, by, barWidth, barH);

      // Label below bar
      ctx.fillStyle = '#94a3b8';
      ctx.font = '8px monospace';
      ctx.fillText(`J${idx + 1}`, bx + 2, h - 2);
    });

    // Update readout text
    const bar = document.getElementById('readouts-torque');
    if (bar) {
      const getC = (v) => v >= 80 ? '#ef4444' : (v >= 50 ? '#facc15' : '#22c55e');
      bar.innerHTML = `
        <span>J1: <b style="color:${getC(currentLoads.loadS)};">${currentLoads.loadS.toFixed(0)}%</b></span>
        <span>J2: <b style="color:${getC(currentLoads.loadL)};">${currentLoads.loadL.toFixed(0)}%</b></span>
        <span>J3: <b style="color:${getC(currentLoads.loadU)};">${currentLoads.loadU.toFixed(0)}%</b></span>
        <span>J4: <b style="color:${getC(currentLoads.loadR)};">${currentLoads.loadR.toFixed(0)}%</b></span>
        <span>J5: <b style="color:${getC(currentLoads.loadB)};">${currentLoads.loadB.toFixed(0)}%</b></span>
        <span>J6: <b style="color:${getC(currentLoads.loadT)};">${currentLoads.loadT.toFixed(0)}%</b></span>
      `;
    }
  }
}
