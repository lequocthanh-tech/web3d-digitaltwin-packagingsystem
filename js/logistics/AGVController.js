import * as THREE from 'three';

/**
 * AGVController.js
 * 
 * GIAI ĐOẠN 3 - PHẦN 2: Hệ Thống Xe Tự Hành AGV/AMR Xếp Pallet & Vận Chuyển Vào Kho Ảo
 * (Autonomous Guided Vehicle / Mobile Robot Palletizing & Virtual Warehouse Logistics Engine)
 * 
 * Tính năng:
 * 1. Quản lý tải trọng lô hàng (10 - 20 thùng, mặc định AGV_CAPACITY = 12 thùng xếp 2 tầng 3x2 trên Euro Pallet).
 * 2. Tạo ma trận tọa độ xếp thùng (Palletizing Matrix) trên mặt pallet xe AGV.
 * 3. Quy trình đón hàng (Transfer & Stacking Sequence) từ cuối Băng tải 3 thông qua cảm biến quang cuối chuyền.
 * 4. Quỹ đạo chuyển động nội suy Waypoints + Slerp góc lái + quay bánh xe theo lộ trình Magnetic Guide Line tới Kho ảo.
 * 5. Quy trình hạ tải tại Kho ảo (Unloading Sequence 1.8s), chuyển lô pallet vào ô trống trên kệ Selective Rack.
 * 6. Chế độ Điều khiển thủ công [FORCE DISPATCH] và [LOAD TEST BOX] để nghiệm thu nhanh.
 */

export const AGV_CAPACITY_DEFAULT = 12;

export class AGVController {
  constructor(app, factory) {
    this.app = app;
    this.factory = factory;

    // 1. Cấu hình thông số lô hàng (10 - 20 thùng, mặc định 12 thùng)
    this.agvCapacity = AGV_CAPACITY_DEFAULT;
    this.currentPayloadCount = 0;
    this.tripsCompleted = 0;
    this.totalWarehouseStoredBoxes = 0;
    this.batteryLevel = 98.0; // %

    // 2. Trạng thái vận hành của AGV:
    // 'READY_FOR_LOADING' | 'TRANSITING' | 'UNLOADING' | 'RETURNING'
    this.state = 'READY_FOR_LOADING';

    // 3. Điểm đỗ đón hàng (Docking Station) tại đuôi Băng tải 3 và Điểm dỡ tải (Drop-off Bay) tại Kho ảo
    this.dockPosition = new THREE.Vector3(-4.58, 0, -1.45);
    this.dockRotationY = 0; // Hướng dọc trục X

    // Lộ trình Waypoints từ đuôi Băng tải 3 đến Phân khu Kho ảo (cách ~6.5m)
    this.deliveryWaypoints = [
      { pos: new THREE.Vector3(-4.58, 0, -1.45), yaw: 0 },
      { pos: new THREE.Vector3(-6.10, 0, -1.45), yaw: 0 },
      { pos: new THREE.Vector3(-7.40, 0, -1.05), yaw: -Math.PI * 0.22 },
      { pos: new THREE.Vector3(-8.45, 0, -0.25), yaw: -Math.PI * 0.12 },
      { pos: new THREE.Vector3(-9.35, 0,  0.15), yaw: 0 }
    ];

    // Nội suy hành trình
    this.routeProgress = 0;      // 0.0 -> 1.0
    this.transitSpeed = 0.36;    // Tốc độ hoàn thành quãng đường (1/s -> ~2.8s toàn tuyến)
    this.unloadTimer = 0;
    this.unloadDuration = 1.8;   // 1.5s - 2.0s theo đặc tả
    this.departureDelayTimer = 0;

    // Danh sách các thùng đang chuyển tiếp (Transferring) và đã xếp trên Pallet
    this.activeTransfers = [];
    this.loadedBoxMeshes = [];

    // Ma trận tọa độ xếp thùng trên mặt Pallet
    this.palletSlots = [];
    this.computePalletizingMatrix(this.agvCapacity);
  }

  /**
   * Tính toán lưới tọa độ ma trận xếp thùng (Palletizing Matrix) trên mặt Euro Pallet (1.2m x 0.8m)
   * Với AGV_CAPACITY = 12: Tầng 1 (6 thùng: 3x2), Tầng 2 (6 thùng: 3x2 chồng khít lên tầng 1)
   * Hỗ trợ linh hoạt từ 10 đến 20 thùng.
   */
  computePalletizingMatrix(capacity = 12) {
    this.agvCapacity = Math.max(1, Math.min(24, parseInt(capacity, 10) || AGV_CAPACITY_DEFAULT));
    this.palletSlots = [];

    // Với capacity <= 12: mỗi tầng 6 thùng (3 cột dọc X x 2 hàng ngang Z)
    // Với capacity > 12 (ví dụ 16, 20): mỗi tầng 8 thùng (4x2) hoặc 3 tầng (3x2)
    let colsX = 3;
    let rowsZ = 2;
    if (this.agvCapacity === 16) {
      colsX = 4;
      rowsZ = 2;
    } else if (this.agvCapacity > 16) {
      colsX = 4;
      rowsZ = 2;
    }

    const boxesPerLayer = colsX * rowsZ;
    const pitchX = colsX === 3 ? 0.34 : 0.27;
    const pitchZ = 0.28;
    const boxHeightY = 0.165;
    const palletTopLocalY = 0.56; // Độ cao mặt trên Euro Pallet trong hệ tọa độ local của AGV

    for (let i = 0; i < this.agvCapacity; i++) {
      const layerIdx = Math.floor(i / boxesPerLayer);
      const idxInLayer = i % boxesPerLayer;
      const col = idxInLayer % colsX;
      const row = Math.floor(idxInLayer / colsX);

      // Xếp từ phía mũi xe (-X) về đuôi xe (+X) để thùng vào trước nằm gọn phía trước
      const localX = ((col - (colsX - 1) / 2) * pitchX);
      const localZ = ((row - (rowsZ - 1) / 2) * pitchZ);
      const localY = palletTopLocalY + layerIdx * boxHeightY;

      this.palletSlots.push(new THREE.Vector3(localX, localY, localZ));
    }
  }

  /**
   * Thay đổi sức chứa lô hàng (10 - 20 thùng) từ giao diện SCADA
   */
  setCapacity(newCap) {
    this.computePalletizingMatrix(newCap);
    // Nếu số thùng hiện tại đã đạt hoặc vượt ngưỡng mới khi đang chờ hàng -> xuất phát ngay
    if (this.state === 'READY_FOR_LOADING' && this.currentPayloadCount >= this.agvCapacity && this.activeTransfers.length === 0) {
      this.startDeliveryCycle();
    }
    this.updateMonitorUI();
  }

  /**
   * Kiểm tra xem AGV có đang đỗ tại trạm Docking và sẵn sàng nhận thêm thùng từ Băng tải 3 không
   */
  canAcceptBoxFromConveyor() {
    return (
      this.state === 'READY_FOR_LOADING' &&
      (this.currentPayloadCount + this.activeTransfers.length) < this.agvCapacity
    );
  }

  /**
   * Bắt đầu chuyển tiếp 1 thùng carton đã dán nắp & quét barcode từ cuối Băng tải 3 sang vị trí (x_i, y_i, z_i) trên Pallet AGV
   */
  receiveBoxFromConveyor(boxRecord) {
    if (!this.factory || !this.factory.agvGroup || !boxRecord || !boxRecord.mesh) return false;

    const slotIndex = this.currentPayloadCount + this.activeTransfers.length;
    if (slotIndex >= this.agvCapacity) return false;

    const targetLocalPos = this.palletSlots[slotIndex]
      ? this.palletSlots[slotIndex].clone()
      : new THREE.Vector3(0, 0.56, 0);

    const mesh = boxRecord.mesh;

    // Chuyển đổi thùng hàng thành mesh con (child) của cụm AGV giữ nguyên vị trí thế giới ban đầu
    const worldPos = new THREE.Vector3();
    const worldQuat = new THREE.Quaternion();
    mesh.getWorldPosition(worldPos);
    mesh.getWorldQuaternion(worldQuat);

    this.factory.agvPayloadGroup.add(mesh);
    const startLocalPos = this.factory.agvPayloadGroup.worldToLocal(worldPos.clone());
    mesh.position.copy(startLocalPos);

    // Kích hoạt đèn cảm biến quang cuối chuyền trên đuôi Băng tải 3 / AGV
    if (this.factory.agvTransferSensorLed) {
      this.factory.agvTransferSensorLed.material.color.setHex(0xef4444);
    }

    this.activeTransfers.push({
      mesh,
      boxId: boxRecord.boxId || `BOX-2026-00${40 + slotIndex}`,
      startLocalPos,
      targetLocalPos,
      progress: 0,
      duration: 0.42 // Trượt êm từ con lăn băng tải sang vị trí (x_i, y_i, z_i) trong 0.42s
    });

    return true;
  }

  /**
   * Tạo nhanh 1 thùng thành phẩm xếp lên AGV (dùng khi test nhanh hoặc cưỡng bức nạp hàng)
   */
  spawnDirectBoxOntoAGV() {
    if (!this.factory || !this.canAcceptBoxFromConveyor()) return false;
    const boxMesh = this.factory.createSealedBoxMeshForAGV();
    const slotIndex = this.currentPayloadCount + this.activeTransfers.length;
    const targetLocalPos = this.palletSlots[slotIndex]
      ? this.palletSlots[slotIndex].clone()
      : new THREE.Vector3(0, 0.56, 0);

    this.factory.agvPayloadGroup.add(boxMesh);
    const startLocalPos = new THREE.Vector3(0.75, 0.72, 0);
    boxMesh.position.copy(startLocalPos);

    this.activeTransfers.push({
      mesh: boxMesh,
      boxId: `BOX-2026-${String(this.factory.nextBoxSerial++).padStart(4, '0')}`,
      startLocalPos,
      targetLocalPos,
      progress: 0,
      duration: 0.35
    });
    return true;
  }

  /**
   * Nút điều khiển thủ công: [CHO XE ĐI NGAY (FORCE DISPATCH)]
   * Cho phép cưỡng bức xe chở hàng đi ngay lập tức dù chưa nhận đủ 10-20 thùng
   */
  forceDispatch() {
    if (this.state !== 'READY_FOR_LOADING') return false;

    // Nếu trên xe chưa có thùng nào, tự động đặt 4 thùng mẫu lên pallet để hiệu ứng chở hàng vào kho trực quan
    if (this.currentPayloadCount === 0 && this.activeTransfers.length === 0 && this.factory) {
      const demoCount = Math.min(6, this.agvCapacity);
      for (let i = 0; i < demoCount; i++) {
        const boxMesh = this.factory.createSealedBoxMeshForAGV();
        const slot = this.palletSlots[i] || new THREE.Vector3(0, 0.56, 0);
        boxMesh.position.copy(slot);
        this.factory.agvPayloadGroup.add(boxMesh);
        this.loadedBoxMeshes.push(boxMesh);
        this.currentPayloadCount++;
      }
    } else if (this.activeTransfers.length > 0) {
      // Hoàn tất ngay các thùng đang trượt dở lên xe
      for (const tr of this.activeTransfers) {
        tr.mesh.position.copy(tr.targetLocalPos);
        tr.mesh.rotation.set(0, 0, 0);
        this.loadedBoxMeshes.push(tr.mesh);
        this.currentPayloadCount++;
      }
      this.activeTransfers = [];
    }

    this.startDeliveryCycle(true);
    return true;
  }

  /**
   * Bắt đầu hành trình chở hàng vào Phân khu Kho ảo (Delivery Leg)
   */
  startDeliveryCycle(isForced = false) {
    if (this.state !== 'READY_FOR_LOADING') return;

    this.state = 'TRANSITING';
    this.routeProgress = 0;
    this.playAGVWarningHorn();

    if (this.app && this.app.oeeCalculator) {
      const tag = isForced ? 'FORCE DISPATCH' : 'BATCH FULL';
      this.app.oeeCalculator.logEvent(
        'BOX',
        `AGV DISPATCH (${tag}): Transporting ${this.currentPayloadCount} boxes to Warehouse.`
      );
    }

    this.updateMonitorUI();
  }

  /**
   * Phát âm thanh còi cảnh báo AGV xuất phát & đèn chớp vàng
   */
  playAGVWarningHorn() {
    try {
      const AudioCtx = window.AudioContext || window.webkitAudioContext;
      if (!AudioCtx) return;
      if (!this._audioCtx) this._audioCtx = new AudioCtx();
      const ctx = this._audioCtx;
      if (ctx.state === 'suspended') ctx.resume();

      const now = ctx.currentTime;
      [0, 0.18].forEach(offset => {
        const osc = ctx.createOscillator();
        const gain = ctx.createGain();
        osc.type = 'triangle';
        osc.frequency.setValueAtTime(580, now + offset);
        osc.frequency.exponentialRampToValueAtTime(740, now + offset + 0.12);
        gain.gain.setValueAtTime(0.05, now + offset);
        gain.gain.exponentialRampToValueAtTime(0.001, now + offset + 0.14);
        osc.connect(gain);
        gain.connect(ctx.destination);
        osc.start(now + offset);
        osc.stop(now + offset + 0.15);
      });
    } catch (_) {
      // Ignore audio block
    }
  }

  /**
   * Nội suy vị trí và góc hướng (Slerp / Lerp) dọc theo chuỗi Waypoints
   * @param {number} t - Tiến độ chuẩn hóa [0..1] (0 = Docking Station, 1 = Warehouse Drop-off Bay)
   * @param {boolean} isReverse - Đang quay về hay đi giao
   */
  evaluateRoutePose(t, isReverse = false) {
    const pts = this.deliveryWaypoints;
    const n = pts.length - 1;
    const clampedT = THREE.MathUtils.clamp(t, 0, 1);

    // Làm mượt gia tốc đầu & cuối hành trình (Smoothstep easing)
    const smoothT = clampedT * clampedT * (3 - 2 * clampedT);
    const scaled = smoothT * n;
    const idx = Math.min(n - 1, Math.floor(scaled));
    const localAlpha = scaled - idx;

    const p0 = pts[idx];
    const p1 = pts[idx + 1];

    const pos = new THREE.Vector3().lerpVectors(p0.pos, p1.pos, localAlpha);
    const yaw = THREE.MathUtils.lerp(p0.yaw, p1.yaw, localAlpha);

    return { pos, yaw, smoothT };
  }

  /**
   * Vòng lặp cập nhật thời gian thực mỗi frame (60 FPS)
   */
  update(delta) {
    if (!this.factory || !this.factory.agvGroup) return;

    // 1. Cập nhật các thùng đang trượt từ cuối Băng tải 3 lên mặt Pallet AGV
    if (this.activeTransfers.length > 0) {
      for (let i = this.activeTransfers.length - 1; i >= 0; i--) {
        const tr = this.activeTransfers[i];
        tr.progress += delta / tr.duration;

        if (tr.progress >= 1.0) {
          tr.mesh.position.copy(tr.targetLocalPos);
          tr.mesh.rotation.set(0, 0, 0);
          this.loadedBoxMeshes.push(tr.mesh);
          this.currentPayloadCount++;
          this.activeTransfers.splice(i, 1);

          if (this.factory.agvTransferSensorLed && this.activeTransfers.length === 0) {
            this.factory.agvTransferSensorLed.material.color.setHex(0x22c55e);
          }

          // Kiểm tra điều kiện xuất phát tự động khi đủ tải trọng AGV_CAPACITY
          if (this.currentPayloadCount >= this.agvCapacity && this.state === 'READY_FOR_LOADING') {
            this.departureDelayTimer = 0.25; // Chờ 0.25s sau khi thùng cuối cùng yên vị rồi xuất phát
          }
          this.updateMonitorUI();
        } else {
          // Nội suy quỹ đạo trượt mượt mà kèm đường cong hạ độ cao từ mặt băng tải (0.75m) xuống tầng Pallet
          const s = tr.progress * tr.progress * (3 - 2 * tr.progress);
          tr.mesh.position.lerpVectors(tr.startLocalPos, tr.targetLocalPos, s);
          // Tạo độ vồng nhẹ ở giữa lúc chuyển tiếp
          tr.mesh.position.y += Math.sin(tr.progress * Math.PI) * 0.04;
        }
      }
    }

    // 2. Xử lý trễ xuất phát ngắn khi vừa xếp đủ thùng thứ AGV_CAPACITY
    if (this.departureDelayTimer > 0 && this.state === 'READY_FOR_LOADING') {
      this.departureDelayTimer -= delta;
      if (this.departureDelayTimer <= 0) {
        this.departureDelayTimer = 0;
        this.startDeliveryCycle(false);
      }
    }

    // 3. Máy trạng thái Chu trình Vận chuyển Tự hành (Autonomous Logistics Cycle)
    const agvGroup = this.factory.agvGroup;

    if (this.state === 'READY_FOR_LOADING') {
      // On-Demand Sleep: Nếu xe AGV đang đỗ chờ tại trạm và không có kiện hàng nào đang bay sang,
      // bỏ qua việc cập nhật tọa độ/hiệu ứng 3D và DOM mỗi frame để giải phóng CPU.
      if (this.activeTransfers.length === 0 && this.batteryLevel >= 100.0) {
        return;
      }
      // Đỗ chuẩn xác tại Docking Station sát đuôi Băng tải 3
      agvGroup.position.copy(this.dockPosition);
      agvGroup.rotation.y = this.dockRotationY;
      this.factory.updateAGVVisualEffects(delta, 'LOADING');

      // Sạc duy trì pin nhẹ khi đỗ tại trạm Docking
      if (this.batteryLevel < 100.0) {
        const prevRounded = Math.round(this.batteryLevel);
        this.batteryLevel = Math.min(100.0, this.batteryLevel + delta * 0.5);
        if (Math.round(this.batteryLevel) !== prevRounded) {
          const batEl = document.getElementById('val-agv-battery');
          if (batEl) {
            batEl.textContent = `${this.batteryLevel.toFixed(0)}%`;
            batEl.style.color = this.batteryLevel > 35 ? '#4ade80' : '#f59e0b';
          }
        }
      }
      return;
    } else if (this.state === 'TRANSITING') {
      // Hành trình chở hàng vào Kho ảo (Delivery Leg)
      this.routeProgress += this.transitSpeed * delta;
      this.batteryLevel = Math.max(15.0, this.batteryLevel - delta * 0.12);

      if (this.routeProgress >= 1.0) {
        this.routeProgress = 1.0;
        const pose = this.evaluateRoutePose(1.0);
        agvGroup.position.copy(pose.pos);
        agvGroup.rotation.y = pose.yaw;

        // Chuyển sang pha Hạ tải tại Kho ảo
        this.state = 'UNLOADING';
        this.unloadTimer = 0;
        this.updateMonitorUI();
      } else {
        const pose = this.evaluateRoutePose(this.routeProgress);
        agvGroup.position.copy(pose.pos);
        agvGroup.rotation.y = pose.yaw;
      }

      this.factory.updateAGVVisualEffects(delta, 'TRANSITING');

    } else if (this.state === 'UNLOADING') {
      // Hạ tải tại Kho ảo (Unloading Sequence: 1.5s - 2.0s)
      this.unloadTimer += delta;
      const progress = THREE.MathUtils.clamp(this.unloadTimer / this.unloadDuration, 0, 1);

      // Hiệu ứng trượt toàn bộ lô thùng carton trên xe vào ô tiếp nhận (Drop-off Bay) của kệ kho (dịch theo trục -Z local)
      if (this.factory.agvPayloadGroup) {
        const slideZ = -THREE.MathUtils.smoothstep(progress, 0.0, 0.85) * 1.15;
        this.factory.agvPayloadGroup.position.z = slideZ;
      }

      this.factory.updateAGVVisualEffects(delta, 'UNLOADING');

      if (this.unloadTimer >= this.unloadDuration) {
        // Hoàn tất dỡ tải vào Kho ảo!
        const deliveredCount = this.currentPayloadCount;
        this.totalWarehouseStoredBoxes += deliveredCount;
        this.tripsCompleted++;

        // Xóa các thùng trên lưng AGV và hiển thị lô hàng mới lưu trữ trong kệ Kho ảo
        this.loadedBoxMeshes.forEach(m => {
          if (this.factory.agvPayloadGroup) {
            this.factory.agvPayloadGroup.remove(m);
          }
        });
        this.loadedBoxMeshes = [];
        if (this.factory.agvPayloadGroup) {
          this.factory.agvPayloadGroup.position.set(0, 0, 0);
        }

        this.currentPayloadCount = 0;

        // Cập nhật lô pallet hiển thị trong ô kệ Kho ảo
        if (this.factory.placeDeliveredBatchInWarehouseRack) {
          this.factory.placeDeliveredBatchInWarehouseRack(this.tripsCompleted, deliveredCount);
        }

        if (this.app && this.app.oeeCalculator) {
          this.app.oeeCalculator.logEvent(
            'BOX',
            `WAREHOUSE STORED: +${deliveredCount} boxes (Total Stored: ${this.totalWarehouseStoredBoxes}).`
          );
        }

        // Bắt đầu hành trình quay về (Return Leg)
        this.state = 'RETURNING';
        this.routeProgress = 1.0;
        this.updateMonitorUI();
      }

    } else if (this.state === 'RETURNING') {
      // Hành trình quay về trạm đón hàng (Return Leg)
      this.routeProgress -= (this.transitSpeed * 1.15) * delta;
      this.batteryLevel = Math.max(15.0, this.batteryLevel - delta * 0.08);

      if (this.routeProgress <= 0.0) {
        this.routeProgress = 0.0;
        agvGroup.position.copy(this.dockPosition);
        agvGroup.rotation.y = this.dockRotationY;
        this.state = 'READY_FOR_LOADING';
        this.playAGVWarningHorn();
        this.updateMonitorUI();
      } else {
        const pose = this.evaluateRoutePose(this.routeProgress, true);
        agvGroup.position.copy(pose.pos);
        agvGroup.rotation.y = pose.yaw;
      }

      this.factory.updateAGVVisualEffects(delta, 'RETURNING');
    }

    // Cập nhật chỉ số pin định kỳ
    const batEl = document.getElementById('val-agv-battery');
    if (batEl) {
      batEl.textContent = `${this.batteryLevel.toFixed(0)}%`;
      batEl.style.color = this.batteryLevel > 35 ? '#4ade80' : '#f59e0b';
    }
  }

  /**
   * Cập nhật dữ liệu lên Thẻ AGV Fleet Monitor trên giao diện HMI / SCADA
   */
  updateMonitorUI() {
    const stateMap = {
      'READY_FOR_LOADING': { text: 'LOADING [Chờ hàng]', color: '#22c55e', bg: 'rgba(34, 197, 94, 0.18)', led: 'led-green active' },
      'TRANSITING':        { text: 'TRANSITING [Đang chạy]', color: '#f59e0b', bg: 'rgba(245, 158, 11, 0.2)', led: 'led-yellow active' },
      'UNLOADING':         { text: 'UNLOADING [Đang dỡ]', color: '#38bdf8', bg: 'rgba(56, 189, 248, 0.2)', led: 'led-cyan active' },
      'RETURNING':         { text: 'RETURNING [Quay về]', color: '#a855f7', bg: 'rgba(168, 85, 247, 0.2)', led: 'led-yellow active' }
    };

    const st = stateMap[this.state] || stateMap['READY_FOR_LOADING'];

    // 1. Trạng thái hiện tại
    const statusEl = document.getElementById('val-agv-status');
    const ledEl = document.getElementById('led-agv-status');
    if (statusEl) {
      statusEl.textContent = st.text;
      statusEl.style.color = st.color;
      statusEl.style.background = st.bg;
      statusEl.style.borderColor = st.color;
    }
    if (ledEl) {
      ledEl.className = `scada-led ${st.led}`;
    }

    // 2. Tải trọng thùng & Thanh tiến trình động
    const payloadTextEl = document.getElementById('val-agv-payload');
    const payloadBarEl = document.getElementById('bar-agv-payload');
    const pct = Math.min(100, Math.round((this.currentPayloadCount / this.agvCapacity) * 100));
    if (payloadTextEl) {
      payloadTextEl.textContent = `${this.currentPayloadCount} / ${this.agvCapacity} thùng (${pct}%)`;
    }
    if (payloadBarEl) {
      payloadBarEl.style.width = `${pct}%`;
      payloadBarEl.style.background = pct >= 100 ? '#f59e0b' : '#22c55e';
    }

    // 3. Bộ đếm chuyến & Tổng lưu kho
    const tripsEl = document.getElementById('val-agv-trips');
    const storedEl = document.getElementById('val-warehouse-stored');
    const chipStoredEl = document.getElementById('chip-warehouse-boxes');
    if (tripsEl) tripsEl.textContent = `${this.tripsCompleted} chuyến`;
    if (storedEl) storedEl.textContent = `${this.totalWarehouseStoredBoxes} thùng`;
    if (chipStoredEl) chipStoredEl.textContent = `${this.totalWarehouseStoredBoxes}`;

    // 4. Tình trạng Pin
    const batEl = document.getElementById('val-agv-battery');
    if (batEl) {
      batEl.textContent = `${this.batteryLevel.toFixed(0)}%`;
    }
  }

  /**
   * Reset dữ liệu đội xe AGV & Kho ảo về ban đầu
   */
  reset() {
    this.loadedBoxMeshes.forEach(m => {
      if (this.factory && this.factory.agvPayloadGroup) {
        this.factory.agvPayloadGroup.remove(m);
      }
    });
    this.activeTransfers.forEach(tr => {
      if (this.factory && this.factory.agvPayloadGroup) {
        this.factory.agvPayloadGroup.remove(tr.mesh);
      }
    });
    this.loadedBoxMeshes = [];
    this.activeTransfers = [];
    this.currentPayloadCount = 0;
    this.tripsCompleted = 0;
    this.totalWarehouseStoredBoxes = 0;
    this.batteryLevel = 98.0;
    this.state = 'READY_FOR_LOADING';
    this.routeProgress = 0;

    if (this.factory && this.factory.agvGroup) {
      this.factory.agvGroup.position.copy(this.dockPosition);
      this.factory.agvGroup.rotation.y = this.dockRotationY;
    }
    if (this.factory && this.factory.agvPayloadGroup) {
      this.factory.agvPayloadGroup.position.set(0, 0, 0);
    }

    this.updateMonitorUI();
  }
}
