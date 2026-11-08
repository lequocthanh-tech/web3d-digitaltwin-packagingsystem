/**
 * OEECalculator.js
 * 
 * GIAI ĐOẠN 3 - PHẦN 1: Hệ Thống Giám Sát Chỉ Số OEE Thời Gian Thực & Quản Trị Chất Lượng Thông Minh
 * (Real-Time Overall Equipment Effectiveness & Smart Quality Management Engine)
 * 
 * Công thức chuẩn công nghiệp (Industry 4.0 / ISO 22400):
 * 1. Availability (A) = (Operating Time / Planned Production Time) * 100%
 * 2. Performance  (P) = (Actual Production Rate / Target Ideal Rate) * 100% (trần <= 100%)
 * 3. Quality      (Q) = (Total OK Products / Total Inspected Products) * 100%
 * 4. Overall OEE      = (A * P * Q) / 10000 (%)
 * 
 * Bảng xếp hạng (Benchmarking):
 * - OEE >= 85%          : "WORLD CLASS" (#22c55e - Xanh lá)
 * - 65% <= OEE < 85%    : "TYPICAL / GOOD" (#f59e0b - Vàng cam)
 * - OEE < 65%           : "UNACCEPTABLE / ATTENTION" (#ef4444 - Đỏ)
 */

export class OEECalculator {
  constructor(app) {
    this.app = app;

    // 1. Time Accumulators (Seconds)
    this.shiftStarted = false;
    this.plannedTimeSec = 0;
    this.operatingTimeSec = 0;
    this.downtimeSec = 0;

    // 2. Performance Rate Parameters
    // Target Ideal Rate = 60 sản phẩm/phút (15 cụm 4-pack/phút)
    this.targetIdealRatePerMin = 60;
    this.pickTimestamps = []; // Cửa sổ trượt 60 giây gần nhất (ms timestamps)
    this.totalPickedOkBars = 0;
    this.actualRatePcsPerMin = 0;

    // 3. Quality Counters
    this.totalInspectedCount = 0;
    this.totalOkCount = 0;
    this.totalNgCount = 0;
    this.totalCompletedBoxes = 0;

    // 4. Computed OEE Metrics (%)
    this.availability = 100.0;
    this.performance = 100.0;
    this.quality = 100.0;
    this.oee = 100.0;

    // 5. State Tracking for Live Event Detection
    this._lastRunState = false;
    this._lastAutoDemo = false;
    this._lastGateClosed = true;
    this._lastEstop = false;
    this._lastNgCount = 0;
    this._lastSealedBoxes = 0;
    this._lastVerifiedBoxes = 0;

    // 6. Batch & Event Log (Nhật ký Lô hàng & Sự kiện)
    this.eventLogs = [];
    this.maxEventLogs = 25;

    // Khởi tạo dòng sự kiện đầu tiên
    this.logEvent('INFO', 'OEE MONITOR READY: Industry 4.0 Analytics initialized.');

    // Chạy chu kỳ tính toán OEE mỗi 1 giây (1000ms) theo đúng đặc tả
    this.calcInterval = setInterval(() => {
      this.calculateStep(1.0);
    }, 1000);
  }

  /**
   * Định dạng thời gian hiện tại [hh:mm:ss]
   */
  getCurrentTimeStr() {
    const now = new Date();
    const hh = String(now.getHours()).padStart(2, '0');
    const mm = String(now.getMinutes()).padStart(2, '0');
    const ss = String(now.getSeconds()).padStart(2, '0');
    return `${hh}:${mm}:${ss}`;
  }

  /**
   * Định dạng số giây sang hh:mm:ss
   */
  formatHMS(totalSeconds) {
    const sec = Math.max(0, Math.floor(totalSeconds));
    const hh = String(Math.floor(sec / 3600)).padStart(2, '0');
    const mm = String(Math.floor((sec % 3600) / 60)).padStart(2, '0');
    const ss = String(sec % 60).padStart(2, '0');
    return `${hh}:${mm}:${ss}`;
  }

  /**
   * Ghi nhận 1 sự kiện vào Nhật ký Lô hàng & Sự kiện (Batch & Event Log)
   * @param {'RUN'|'DEFECT'|'BOX'|'STOP'|'INFO'} type
   * @param {string} message
   */
  logEvent(type, message) {
    const timeStr = this.getCurrentTimeStr();
    const entry = {
      time: timeStr,
      type,
      text: `[${timeStr}] ${message}`
    };

    this.eventLogs.unshift(entry);
    if (this.eventLogs.length > this.maxEventLogs) {
      this.eventLogs.pop();
    }

    this.renderEventLogDOM();
  }

  /**
   * Ghi nhận khi Robot Delta gắp thành công 1 sản phẩm OK sang Băng tải 2
   */
  recordProductPicked(count = 1) {
    const now = performance.now();
    for (let i = 0; i < count; i++) {
      this.pickTimestamps.push(now);
      this.totalPickedOkBars++;
    }
    this.prunePickWindow(now);
    this.updateMetricsNow();
    this.renderDashboardDOM();
  }

  /**
   * Loại bỏ các mẫu gắp quá 60 giây khỏi cửa sổ trượt (Sliding Window 60s)
   */
  prunePickWindow(now = performance.now()) {
    const cutoff = now - 60000;
    while (this.pickTimestamps.length > 0 && this.pickTimestamps[0] < cutoff) {
      this.pickTimestamps.shift();
    }
  }

  /**
   * Chu kỳ tính toán OEE mỗi 1 giây (1000ms)
   */
  calculateStep(dtSec = 1.0) {
    if (!this.app || !this.app.factory) return;

    const factory = this.app.factory;
    const hw = this.app.hardwareBus;

    const isPowerOn = Boolean(this.app.systemPower);
    const isGateClosed = factory.safetyGateClosed !== false;
    const isEstop = Boolean(hw && hw.emergencyStop);
    const isInterlockTripped = Boolean(this.app.isSafetyInterlockTripped);
    const isCollision = Boolean(this.app.isCollisionActive);

    // Kiểm tra có lỗi dừng máy khẩn cấp hoặc mở cửa rào không
    const hasSafetyOrFault = !isGateClosed || isEstop || isInterlockTripped || isCollision;

    // Kiểm tra hệ thống đang ở trạng thái RUN tự động (băng tải & robot hoạt động bình thường, không lỗi)
    const isRunningAuto = isPowerOn && !hasSafetyOrFault && Boolean(
      this.app.autoDemoMode ||
      this.app.isSingleCycleRunning ||
      this.app.isDeltaBusy ||
      this.app.isArmBusy ||
      factory.conveyor1Active ||
      factory.conveyor2Active
    );

    // Bắt đầu đếm Planned Production Time từ khi bật nguồn / khởi động hệ thống lần đầu trong ca
    if (isPowerOn || isRunningAuto || hasSafetyOrFault) {
      this.shiftStarted = true;
    }

    // On-Demand Idle Sleep: Nếu hệ thống chưa bắt đầu ca và đang tắt nguồn hoàn toàn, không cần tính toán/vẽ lại DOM mỗi giây
    if (!this.shiftStarted && !isPowerOn) {
      return;
    }

    // -------------------------------------------------------------------------
    // A. PHÁT HIỆN SỰ KIỆN TỰ ĐỘNG (LIVE EVENT TRIGGERS)
    // -------------------------------------------------------------------------

    // 1. Sự kiện SYSTEM RUN
    if (isRunningAuto && !this._lastRunState) {
      this.logEvent('RUN', 'SYSTEM RUN: Auto packaging started.');
    }
    this._lastRunState = isRunningAuto;

    // 2. Sự kiện SAFETY STOP (Mở cửa bảo trì)
    if (!isGateClosed && this._lastGateClosed) {
      this.logEvent('STOP', 'SAFETY STOP: Door opened - Interlock tripped!');
    } else if (isGateClosed && !this._lastGateClosed) {
      this.logEvent('INFO', 'SAFETY GATE: Maintenance door closed.');
    }
    this._lastGateClosed = isGateClosed;

    // 3. Sự kiện E-STOP
    if (isEstop && !this._lastEstop && isGateClosed) {
      this.logEvent('STOP', 'SAFETY STOP: Emergency Stop button pressed!');
    }
    this._lastEstop = isEstop;

    // 4. Sự kiện DEFECT DETECTED (Xilanh khí nén gạt phế phẩm NG)
    const currentNg = factory.ngDefectCount || 0;
    if (currentNg > this._lastNgCount) {
      this.logEvent('DEFECT', `DEFECT DETECTED: Reject cylinder actuated (NG #${currentNg}).`);
      this._lastNgCount = currentNg;
    }

    // 5. Sự kiện BOX COMPLETED (Thùng hoàn thiện qua máy dán / quét Barcode)
    const currentVerified = factory.totalVerifiedBoxes || 0;
    const currentSealed = factory.sealedCartonCount || 0;
    if (currentVerified > this._lastVerifiedBoxes) {
      const boxId = (factory.lastPrintedBoxInfo && factory.lastPrintedBoxInfo.boxId)
        ? factory.lastPrintedBoxInfo.boxId
        : `BOX-2026-${String(40 + currentVerified).padStart(4, '0')}`;
      this.logEvent('BOX', `BOX COMPLETED: ID #${boxId}.`);
      this._lastVerifiedBoxes = currentVerified;
      this._lastSealedBoxes = Math.max(this._lastSealedBoxes, currentSealed);
    } else if (currentSealed > this._lastSealedBoxes) {
      const boxId = `BOX-2026-${String(40 + currentSealed).padStart(4, '0')}`;
      this.logEvent('BOX', `BOX COMPLETED: ID #${boxId} (Top Sealer).`);
      this._lastSealedBoxes = currentSealed;
    }

    // -------------------------------------------------------------------------
    // B. TÍCH LŨY THỜI GIAN (PLANNED TIME, OPERATING TIME, DOWNTIME)
    // -------------------------------------------------------------------------
    if (this.shiftStarted) {
      this.plannedTimeSec += dtSec;

      if (isRunningAuto) {
        this.operatingTimeSec += dtSec;
      } else {
        // Đếm tăng Downtime khi E-Stop, mở cửa an toàn, hoặc PAUSE / STANDBY
        this.downtimeSec += dtSec;
      }
    }

    // Cập nhật các chỉ số A, P, Q, OEE
    this.updateMetricsNow();

    // Render lên giao diện SCADA & VIPA HMI
    this.renderDashboardDOM();
  }

  /**
   * Tính toán tức thời các chỉ số Availability (A), Performance (P), Quality (Q) và OEE tổng
   */
  updateMetricsNow() {
    if (!this.app || !this.app.factory) return;
    const factory = this.app.factory;

    // -------------------------------------------------------------------------
    // 1. AVAILABILITY (A) = (Operating Time / Planned Production Time) * 100%
    // -------------------------------------------------------------------------
    if (this.plannedTimeSec > 0) {
      this.availability = Math.min(100.0, Math.max(0.0, (this.operatingTimeSec / this.plannedTimeSec) * 100.0));
    } else {
      this.availability = 100.0;
    }

    // -------------------------------------------------------------------------
    // 2. PERFORMANCE (P) = (Actual Production Rate / Target Ideal Rate) * 100%
    // -------------------------------------------------------------------------
    const now = performance.now();
    this.prunePickWindow(now);

    const picksInLast60s = this.pickTimestamps.length;
    const isRunning = this._lastRunState;

    if (this.operatingTimeSec <= 0 && picksInLast60s === 0) {
      // Khi vừa khởi động chưa chạy
      this.actualRatePcsPerMin = 0;
      this.performance = 100.0;
    } else {
      // Tính tốc độ thực tế trong cửa sổ trượt 60s (quy đổi nếu thời gian chạy < 60s để phản hồi mượt ngay từ đầu ca)
      const effectiveWindowSec = Math.min(60.0, Math.max(4.0, this.operatingTimeSec));
      let slidingRate = (picksInLast60s / effectiveWindowSec) * 60.0;

      // Kết hợp với tốc độ cấp phôi thực tế của Băng tải 1 khi dây chuyền đang chạy ổn định nhịp nhàng
      if (isRunning && factory.conveyor1Active) {
        const feedMultiplier = factory.infeedRateMultiplier || 1.0;
        // Ở tốc độ chuẩn 1.0x, nhịp thiết kế đạt ~56-58 pcs/min (~93-97% Performance)
        const nominalBeltRate = Math.min(60.0, 56.5 * feedMultiplier);
        if (this.operatingTimeSec < 15 && picksInLast60s > 0) {
          slidingRate = Math.max(slidingRate, nominalBeltRate);
        } else if (picksInLast60s > 0) {
          slidingRate = slidingRate * 0.65 + nominalBeltRate * 0.35;
        } else {
          // Băng tải đang chạy nhưng đang chờ phôi đầu tiên tới vùng gắp
          slidingRate = nominalBeltRate * 0.82;
        }
      } else if (!isRunning) {
        // Khi dừng chờ / Pause thì tốc độ thực tế giảm dần theo cửa sổ trượt
        slidingRate = picksInLast60s;
      }

      this.actualRatePcsPerMin = Math.round(slidingRate);
      const rawP = (slidingRate / this.targetIdealRatePerMin) * 100.0;
      this.performance = Math.min(100.0, Math.max(0.0, rawP));
    }

    // -------------------------------------------------------------------------
    // 3. QUALITY (Q) = (Total OK Products / Total Inspected Products) * 100%
    // -------------------------------------------------------------------------
    const rawInspected = factory.totalInspectedCount || 0;
    const ngCount = factory.ngDefectCount || 0;
    // Đảm bảo Total Inspected >= OK + NG
    const okFromPicks = Math.max(this.totalPickedOkBars, Math.max(0, rawInspected - ngCount));
    const totalInspected = Math.max(rawInspected, okFromPicks + ngCount);

    this.totalNgCount = ngCount;
    this.totalOkCount = Math.max(0, totalInspected - ngCount);
    this.totalInspectedCount = totalInspected;
    this.totalCompletedBoxes = Math.max(factory.totalVerifiedBoxes || 0, factory.sealedCartonCount || 0);

    if (totalInspected > 0) {
      this.quality = Math.min(100.0, Math.max(0.0, (this.totalOkCount / totalInspected) * 100.0));
    } else {
      this.quality = 100.0;
    }

    // -------------------------------------------------------------------------
    // 4. CHỈ SỐ TỔNG HỢP OEE = (A * P * Q) / 10000 (%)
    // -------------------------------------------------------------------------
    this.oee = (this.availability * this.performance * this.quality) / 10000.0;
    this.oee = Math.min(100.0, Math.max(0.0, this.oee));
  }

  /**
   * Trả về phân hạng màu sắc & nhãn đánh giá chuẩn công nghiệp (Benchmarking)
   */
  getOEEBenchmark(oeeVal = this.oee) {
    if (oeeVal >= 85.0) {
      return {
        tier: 'WORLD CLASS',
        badge: 'World Class',
        color: '#22c55e',
        bgRgba: 'rgba(34, 197, 94, 0.16)',
        borderRgba: 'rgba(34, 197, 94, 0.45)'
      };
    } else if (oeeVal >= 65.0) {
      return {
        tier: 'TYPICAL / GOOD',
        badge: 'Operating Good',
        color: '#f59e0b',
        bgRgba: 'rgba(245, 158, 11, 0.16)',
        borderRgba: 'rgba(245, 158, 11, 0.45)'
      };
    } else {
      return {
        tier: 'UNACCEPTABLE / ATTENTION',
        badge: 'Line Lagging',
        color: '#ef4444',
        bgRgba: 'rgba(239, 68, 68, 0.18)',
        borderRgba: 'rgba(239, 68, 68, 0.5)'
      };
    }
  }

  /**
   * Trả về màu sắc cho từng thanh Progress Bar (A, P, Q)
   */
  getMetricColor(val) {
    if (val >= 85.0) return '#22c55e';
    if (val >= 65.0) return '#f59e0b';
    return '#ef4444';
  }

  /**
   * Đặt lại ca làm việc (Reset Shift / Clear OEE Data)
   * Xóa toàn bộ bộ đếm và thời gian về 0 để bắt đầu ca sản xuất mới
   */
  resetShift() {
    this.shiftStarted = Boolean(this.app && this.app.systemPower);
    this.plannedTimeSec = 0;
    this.operatingTimeSec = 0;
    this.downtimeSec = 0;

    this.pickTimestamps = [];
    this.totalPickedOkBars = 0;
    this.actualRatePcsPerMin = 0;

    this.totalInspectedCount = 0;
    this.totalOkCount = 0;
    this.totalNgCount = 0;
    this.totalCompletedBoxes = 0;

    this.availability = 100.0;
    this.performance = 100.0;
    this.quality = 100.0;
    this.oee = 100.0;

    // Đồng bộ xóa bộ đếm trên FactoryEnvironment & VIPA HMI
    if (this.app && this.app.factory) {
      this.app.factory.totalInspectedCount = 0;
      this.app.factory.ngDefectCount = 0;
      this.app.factory.sealedCartonCount = 0;
      this.app.factory.totalVerifiedBoxes = 0;
    }
    if (this.app && this.app.vipaHMI) {
      this.app.vipaHMI.totalBoxesShipped = 0;
      this.app.vipaHMI.totalBarsPacked = 0;
    }

    this._lastNgCount = 0;
    this._lastSealedBoxes = 0;
    this._lastVerifiedBoxes = 0;

    this.eventLogs = [];
    this.logEvent('INFO', 'SHIFT RESET: All OEE counters & timers cleared to 0.');
    if (this._lastRunState) {
      this.logEvent('RUN', 'SYSTEM RUN: Auto packaging started.');
    }

    this.renderDashboardDOM();
  }

  /**
   * Cập nhật toàn bộ giao diện OEE Dashboard (Tab OEE, Widget SCADA, Topbar Chip & VIPA HMI)
   */
  renderDashboardDOM() {
    const bm = this.getOEEBenchmark(this.oee);
    const oeeStr = `${this.oee.toFixed(1)}%`;
    const aStr = `${this.availability.toFixed(1)}%`;
    const pStr = `${this.performance.toFixed(1)}%`;
    const qStr = `${this.quality.toFixed(1)}%`;

    const runTimeStr = this.formatHMS(this.operatingTimeSec);
    const downTimeStr = this.formatHMS(this.downtimeSec);
    const plannedTimeStr = this.formatHMS(this.plannedTimeSec);

    // 1. Topbar OEE Chip
    const chipOeeVal = document.getElementById('chip-oee-val');
    const ledOeeChip = document.getElementById('led-oee-chip');
    if (chipOeeVal) {
      chipOeeVal.textContent = oeeStr;
      chipOeeVal.style.color = bm.color;
    }
    if (ledOeeChip) {
      ledOeeChip.className = `scada-led active ${
        this.oee >= 85 ? 'led-green' : (this.oee >= 65 ? 'led-yellow' : 'led-red')
      }`;
    }

    // 2. On-Demand Tab-Aware DOM Update: Chỉ cập nhật phần tử OEE của Tab đang thực sự hiển thị trước mắt người dùng
    const isDockVisible = !this.app || this.app.activeWorkstation === 'default';
    const isOeeTabActive = isDockVisible && document.getElementById('tab-pane-oee')?.classList.contains('active');
    const isJoggerTabActive = isDockVisible && document.getElementById('tab-pane-jogger')?.classList.contains('active');

    const prefixes = [];
    if (isOeeTabActive) prefixes.push('oee-main');
    if (isJoggerTabActive) prefixes.push('oee-widget');
    const circumference = 2 * Math.PI * 52; // r = 52 -> C ≈ 326.7256

    prefixes.forEach(prefix => {
      // Vòng tròn Donut SVG
      const ringEl = document.getElementById(`${prefix}-donut-ring`);
      const valEl = document.getElementById(`${prefix}-donut-val`);
      const badgeEl = document.getElementById(`${prefix}-benchmark-badge`);
      const tierEl = document.getElementById(`${prefix}-tier-text`);

      if (ringEl) {
        const offset = circumference - (this.oee / 100.0) * circumference;
        ringEl.style.strokeDasharray = `${circumference.toFixed(2)}`;
        ringEl.style.strokeDashoffset = `${offset.toFixed(2)}`;
        ringEl.style.stroke = bm.color;
      }
      if (valEl) {
        valEl.textContent = oeeStr;
        valEl.style.color = bm.color;
      }
      if (badgeEl) {
        badgeEl.textContent = bm.badge;
        badgeEl.style.color = bm.color;
        badgeEl.style.background = bm.bgRgba;
        badgeEl.style.borderColor = bm.borderRgba;
      }
      if (tierEl) {
        tierEl.textContent = bm.tier;
        tierEl.style.color = bm.color;
      }

      // Thanh 1: Availability
      const aValEl = document.getElementById(`${prefix}-avail-val`);
      const aBarEl = document.getElementById(`${prefix}-avail-bar`);
      const aSubEl = document.getElementById(`${prefix}-avail-sub`);
      const aCol = this.getMetricColor(this.availability);
      if (aValEl) {
        aValEl.textContent = aStr;
        aValEl.style.color = aCol;
      }
      if (aBarEl) {
        aBarEl.style.width = `${this.availability.toFixed(1)}%`;
        aBarEl.style.background = aCol;
      }
      if (aSubEl) {
        aSubEl.innerHTML = `Run Time: <b style="color:#4ade80;">${runTimeStr}</b> | Downtime: <b style="color:${this.downtimeSec > 0 ? '#f87171' : '#94a3b8'};">${downTimeStr}</b>`;
      }

      // Thanh 2: Performance
      const pValEl = document.getElementById(`${prefix}-perf-val`);
      const pBarEl = document.getElementById(`${prefix}-perf-bar`);
      const pSubEl = document.getElementById(`${prefix}-perf-sub`);
      const pCol = this.getMetricColor(this.performance);
      if (pValEl) {
        pValEl.textContent = pStr;
        pValEl.style.color = pCol;
      }
      if (pBarEl) {
        pBarEl.style.width = `${this.performance.toFixed(1)}%`;
        pBarEl.style.background = pCol;
      }
      if (pSubEl) {
        pSubEl.innerHTML = `Actual Speed: <b style="color:#38bdf8;">${this.actualRatePcsPerMin} pcs/min</b> <span style="color:#64748b;">(Target: ${this.targetIdealRatePerMin} pcs/min)</span>`;
      }

      // Thanh 3: Quality
      const qValEl = document.getElementById(`${prefix}-qual-val`);
      const qBarEl = document.getElementById(`${prefix}-qual-bar`);
      const qSubEl = document.getElementById(`${prefix}-qual-sub`);
      const qCol = this.getMetricColor(this.quality);
      if (qValEl) {
        qValEl.textContent = qStr;
        qValEl.style.color = qCol;
      }
      if (qBarEl) {
        qBarEl.style.width = `${this.quality.toFixed(1)}%`;
        qBarEl.style.background = qCol;
      }
      if (qSubEl) {
        qSubEl.innerHTML = `OK: <b style="color:#4ade80;">${this.totalOkCount}</b> / NG: <b style="color:#ef4444;">${this.totalNgCount}</b> <span style="color:#64748b;">(Inspected: ${this.totalInspectedCount})</span>`;
      }

      // Planned Time Summary
      const plannedEl = document.getElementById(`${prefix}-planned-time`);
      if (plannedEl) {
        plannedEl.textContent = plannedTimeStr;
      }
    });

    // 3. Đồng bộ lên màn hình VIPA HMI (Station 4) chỉ khi người dùng đang mở Station 4
    if (this.app && this.app.activeWorkstation === 'vipa') {
      const hmiSpeedVal = document.getElementById('hmi-kpi-line-speed');
      const hmiOeeVal = document.getElementById('hmi-kpi-oee-val');
      const hmiAvailVal = document.getElementById('hmi-oee-avail');
      const hmiPerfVal = document.getElementById('hmi-oee-perf');
      const hmiQualVal = document.getElementById('hmi-oee-qual');
      if (hmiSpeedVal) {
        hmiSpeedVal.innerHTML = `${this.actualRatePcsPerMin.toFixed(1)} <small>thanh/phút</small>`;
      }
      if (hmiOeeVal) {
        hmiOeeVal.textContent = oeeStr;
        hmiOeeVal.style.color = bm.color;
      }
      if (hmiAvailVal) hmiAvailVal.textContent = aStr;
      if (hmiPerfVal) hmiPerfVal.textContent = pStr;
      if (hmiQualVal) hmiQualVal.textContent = qStr;
    }
  }

  /**
   * Render danh sách Nhật ký Lô hàng & Sự kiện (Batch & Event Log)
   */
  renderEventLogDOM() {
    const containers = [
      document.getElementById('oee-main-event-log'),
      document.getElementById('oee-widget-event-log')
    ];

    const visibleLogs = this.eventLogs.slice(0, 8);

    const html = visibleLogs.map(item => {
      let badgeClass = 'oee-log-info';
      if (item.type === 'RUN') badgeClass = 'oee-log-run';
      else if (item.type === 'DEFECT') badgeClass = 'oee-log-defect';
      else if (item.type === 'BOX') badgeClass = 'oee-log-box';
      else if (item.type === 'STOP') badgeClass = 'oee-log-stop';

      return `<div class="oee-log-item ${badgeClass}">${item.text}</div>`;
    }).join('');

    containers.forEach(el => {
      if (el) {
        el.innerHTML = html;
      }
    });
  }
}
