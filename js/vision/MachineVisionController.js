import * as THREE from 'three';

/**
 * MachineVisionController.js
 * 
 * GIAI ĐOẠN 4 - PHẦN 1: Trạm Xử Lý Thị Giác Máy Tính Công Nghiệp & Luồng Camera Trực Tiếp
 * (Industrial Machine Vision Studio - Cognex In-Sight / Keyence CV-X Simulation)
 * 
 * Tính năng:
 * 1. Top-Down Orthographic Vision Camera (`THREE.OrthographicCamera`) đặt vuông góc 90° ngay trên cổng vòm
 *    Camera Vision của Băng tải 1 (X = +2.80m, Y = 1.42m, Z = -0.32m), bao phủ trọn vùng kiểm tra 600mm.
 * 2. Cơ chế Render-to-Texture (`THREE.WebGLRenderTarget`) kết hợp bộ xử lý ảnh DSP 2D lên `<canvas id="vision-canvas">`
 *    tối ưu hiệu năng ở tần số 20 FPS hoặc khi có sự kiện `DI_VISION_TRIGGER == true` / `[Single Trigger]`.
 * 3. Mô phỏng 3 chế độ lọc ống kính (Image Processing Pipeline):
 *    - RAW COLOR: Ảnh màu thực tế từ cảm biến CMOS.
 *    - GRAYSCALE: Chuyển đổi mức xám 8-bit (Luminance: 0.299R + 0.587G + 0.114B).
 *    - BINARY THRESHOLD: Tách ngưỡng nhị phân đen/trắng (`Threshold = 128`) tách phôi khỏi nền băng tải.
 * 4. Nhận diện Bounding Box (Oriented / Axis-Aligned Box), trọng tâm Centroid (`+`), mũi tên Vector góc xoay `Angle: θ°`,
 *    diện tích Pixel Area và tỷ lệ khung bao Aspect Ratio.
 * 5. Thuật toán Phân loại Ngoại quan (PASS [OK] vs FAIL [NG - DEFECT]):
 *    - PASS [OK]: Khung xanh lá, đẩy tọa độ offset (X, Y, θ) vào hàng đợi FIFO của Robot Delta để bù góc xoay trục thứ 4 (Yaw).
 *    - FAIL [NG]: Khung đỏ nhấp nháy cảnh báo, đánh dấu `isRejected = true` để kích hoạt xilanh gạt phế phẩm (Reject Pusher).
 */

export class MachineVisionController {
  constructor(app) {
    this.app = app;
    this.scene = app.scene;
    this.renderer = app.renderer;
    this.factory = app.factory;
    this.physicsEngine = app.physicsEngine;

    // 1. Thiết lập Camera Thị giác Phụ (Top-down Orthographic Camera)
    // Bao phủ vùng kiểm tra quang học: Chiều dài 600mm dọc Băng tải 1 (X = 2.80m ± 0.30m) x Rộng 400mm (Z = -0.32m ± 0.20m)
    this.fovWidthM = 0.60;  // 600mm along X
    this.fovHeightM = 0.40; // 400mm along Z
    this.camCenter = new THREE.Vector3(2.80, 1.38, -0.32);

    this.visionCamera = new THREE.OrthographicCamera(
      -this.fovWidthM / 2,
       this.fovWidthM / 2,
       this.fovHeightM / 2,
      -this.fovHeightM / 2,
      0.05,
      1.20
    );
    // Đặt vuông góc 90 độ hướng thẳng ống kính xuống mặt Băng tải 1 (Y = 0.865m)
    this.visionCamera.position.copy(this.camCenter);
    // Với up = (0, 0, -1), trục ngang (+X màn hình) ứng với +X băng tải, trục dọc (+Y màn hình) ứng với +Z băng tải
    this.visionCamera.up.set(0, 0, -1);
    this.visionCamera.lookAt(this.camCenter.x, 0.865, this.camCenter.z);
    this.visionCamera.updateProjectionMatrix();

    // 2. Thiết lập kích thước màn hình soi (PIP Offscreen Buffer 330 x 220)
    this.rtWidth = 330;
    this.rtHeight = 220;

    // 3. Cấu hình Pipeline Xử lý Ảnh & Trạng thái Giao diện
    this.filterMode = 'COLOR'; // 'COLOR' | 'GRAY' | 'BINARY'
    this.thresholdValue = 128;
    this.isLiveVideo = true;
    this.isMinimized = true; // Mặc định thu nhỏ gọn khi khởi động để không che góc nhìn 3D và không tốn CPU
    this.DI_VISION_TRIGGER = false;

    // Tối ưu hiệu năng: render ở tần số ~15-20 FPS (mỗi 60ms) và chỉ khi mở cửa sổ
    this.fpsIntervalSec = 0.06;
    this.accumTimeSec = 0;

    // Tiêu chuẩn kiểm định ngoại quan (Pass/Fail Criteria)
    this.criteria = {
      areaMinPx: 2600,
      areaMaxPx: 4200,
      aspectMin: 2.4,
      aspectMax: 3.6
    };

    // Dữ liệu Telemetry lần kiểm tra gần nhất
    this.lastInspection = {
      hasTarget: false,
      status: 'PASS [OK]',
      passed: true,
      offsetX_mm: 0.0,
      offsetY_mm: 0.0,
      angleDeg: 0.0,
      lengthMm: 180.0,
      widthMm: 60.0,
      areaPx: 3320,
      aspectRatio: 3.0,
      processingTimeMs: 11,
      boxId: 'OBJ-0001'
    };
    this.inspectedObjectCounter = 1;

    this.initDOMBindings();
  }

  /**
   * Liên kết các sự kiện nút bấm trên cửa sổ COGNEX VISION PRO - SENSOR #CAM01
   */
  initDOMBindings() {
    this.canvasEl = document.getElementById('vision-canvas');
    if (this.canvasEl) {
      this.ctx2d = this.canvasEl.getContext('2d');
    }

    // 1. Bộ 3 nút bấm chuyển Filter: [Color] | [Gray] | [Binary]
    const filterBtns = [
      { id: 'btn-vision-filter-color', mode: 'COLOR' },
      { id: 'btn-vision-filter-gray', mode: 'GRAY' },
      { id: 'btn-vision-filter-binary', mode: 'BINARY' }
    ];
    filterBtns.forEach(item => {
      const btn = document.getElementById(item.id);
      if (btn) {
        btn.addEventListener('click', () => {
          this.setFilterMode(item.mode);
        });
      }
    });

    // 2. Nút [Chụp đơn (Single Trigger)] & [Live Video]
    const btnSingleTrigger = document.getElementById('btn-vision-single-trigger');
    const btnLiveVideo = document.getElementById('btn-vision-live-toggle');

    if (btnSingleTrigger) {
      btnSingleTrigger.addEventListener('click', () => {
        this.isLiveVideo = false;
        if (btnLiveVideo) btnLiveVideo.classList.remove('active');
        btnSingleTrigger.classList.add('active');
        setTimeout(() => btnSingleTrigger.classList.remove('active'), 220);
        this.captureAndProcessFrame(true);
      });
    }

    if (btnLiveVideo) {
      btnLiveVideo.addEventListener('click', () => {
        this.isLiveVideo = !this.isLiveVideo;
        btnLiveVideo.classList.toggle('active', this.isLiveVideo);
        btnLiveVideo.textContent = this.isLiveVideo ? '🎥 Live Video: ON' : '⏸ Live Video: PAUSED';
        if (this.isLiveVideo) {
          this.captureAndProcessFrame(true);
        }
      });
    }

    // 3. Nút thu nhỏ / phóng to cửa sổ Vision Studio
    const btnMinimize = document.getElementById('btn-vision-minimize');
    const winBody = document.getElementById('vision-studio-body');
    const winEl = document.getElementById('cognex-vision-floating-window');
    if (btnMinimize && winBody) {
      // Áp dụng trạng thái thu nhỏ mặc định ngay khi khởi tạo
      winBody.style.display = this.isMinimized ? 'none' : 'block';
      btnMinimize.textContent = this.isMinimized ? '➕' : '➖';
      btnMinimize.title = this.isMinimized ? 'Phóng to cửa sổ Vision Studio' : 'Thu nhỏ cửa sổ Vision Studio';
      if (winEl) {
        winEl.classList.toggle('minimized', this.isMinimized);
      }

      btnMinimize.addEventListener('click', () => {
        this.isMinimized = !this.isMinimized;
        winBody.style.display = this.isMinimized ? 'none' : 'block';
        btnMinimize.textContent = this.isMinimized ? '➕' : '➖';
        btnMinimize.title = this.isMinimized ? 'Phóng to cửa sổ Vision Studio' : 'Thu nhỏ cửa sổ Vision Studio';
        if (winEl) {
          winEl.classList.toggle('minimized', this.isMinimized);
        }
        if (!this.isMinimized) {
          this.captureAndProcessFrame(true);
        }
      });
    }

    // 4. Hỗ trợ kéo thả cửa sổ Vision Studio trên màn hình 3D
    const headerEl = document.getElementById('vision-studio-drag-header');
    if (headerEl && winEl) {
      let dragging = false;
      let startX = 0, startY = 0, initLeft = 0, initTop = 0;

      headerEl.addEventListener('pointerdown', (e) => {
        if (e.target.tagName === 'BUTTON') return;
        dragging = true;
        const rect = winEl.getBoundingClientRect();
        startX = e.clientX;
        startY = e.clientY;
        initLeft = rect.left;
        initTop = rect.top;
        winEl.style.right = 'auto';
        winEl.style.bottom = 'auto';
        winEl.style.left = `${initLeft}px`;
        winEl.style.top = `${initTop}px`;
        headerEl.setPointerCapture(e.pointerId);
      });

      headerEl.addEventListener('pointermove', (e) => {
        if (!dragging) return;
        const dx = e.clientX - startX;
        const dy = e.clientY - startY;
        winEl.style.left = `${Math.max(8, initLeft + dx)}px`;
        winEl.style.top = `${Math.max(8, initTop + dy)}px`;
      });

      headerEl.addEventListener('pointerup', (e) => {
        if (dragging) {
          dragging = false;
          headerEl.releasePointerCapture(e.pointerId);
        }
      });
    }

    // Render frame khởi tạo đầu tiên
    setTimeout(() => {
      this.captureAndProcessFrame(true);
    }, 150);
  }

  /**
   * Chuyển đổi chế độ lọc ảnh: COLOR | GRAY | BINARY
   */
  setFilterMode(mode) {
    this.filterMode = mode;
    const map = {
      'COLOR': 'btn-vision-filter-color',
      'GRAY': 'btn-vision-filter-gray',
      'BINARY': 'btn-vision-filter-binary'
    };
    Object.entries(map).forEach(([k, id]) => {
      const el = document.getElementById(id);
      if (el) el.classList.toggle('active', k === mode);
    });

    // Cập nhật ngay hình ảnh theo bộ lọc mới
    this.captureAndProcessFrame(true);
  }

  /**
   * Được gọi trong vòng lặp animate() chính; tự động giới hạn ở ~15-20 FPS để bảo toàn 60 FPS cho cảnh 3D chính
   */
  update(delta) {
    this.accumTimeSec += delta;
    if (this.accumTimeSec < this.fpsIntervalSec) return;
    this.accumTimeSec = 0;

    // Tối ưu CPU: nếu cửa sổ đang ẩn hoặc thu nhỏ, không chạy xử lý đồ họa
    const winEl = document.getElementById('cognex-vision-floating-window');
    if (this.isMinimized || (winEl && winEl.style.display === 'none')) {
      return;
    }

    // Kiểm tra xem có phôi đang nằm trong vùng kiểm tra quang học 600mm không
    const visibleProducts = this.getProductsInVisionFOV();
    this.DI_VISION_TRIGGER = visibleProducts.length > 0;

    if (this.isLiveVideo) {
      this.captureAndProcessFrame(false, visibleProducts);
    }
  }

  /**
   * Tìm danh sách các phôi sô-cô-la trên Băng tải 1 đang đi qua vùng nhìn 600mm (X = 2.50m .. 3.10m)
   */
  getProductsInVisionFOV() {
    if (!this.factory || !this.factory.infeedProducts) return [];
    const minX = this.camCenter.x - this.fovWidthM / 2;
    const maxX = this.camCenter.x + this.fovWidthM / 2;

    return this.factory.infeedProducts.filter(p => {
      if (p.picked || p.falling || !p.mesh) return false;
      const px = p.mesh.position.x;
      return px >= minX && px <= maxX;
    });
  }

  /**
   * Render mô phỏng trực tiếp trên 2D Canvas siêu tốc (0 GPU stall, 0 blocking readPixels)
   * Áp dụng 3 chế độ lọc ống kính DSP: RAW COLOR / GRAYSCALE / BINARY THRESHOLD
   * và vẽ lớp phủ HUD Cognex Machine Vision (Crosshair Grid, Bounding Box, Centroid +, Vector θ, Pass/Fail)
   */
  captureAndProcessFrame(forceRender = false, prefilteredProducts = null) {
    if (!this.ctx2d) return;

    const winEl = document.getElementById('cognex-vision-floating-window');
    if (!forceRender && winEl && (winEl.style.display === 'none' || this.isMinimized)) {
      return;
    }

    const tStart = performance.now();
    const ctx = this.ctx2d;
    const w = this.rtWidth;
    const h = this.rtHeight;
    const mode = this.filterMode;

    // 1. Render Mặt Băng Tải theo chế độ lọc
    if (mode === 'BINARY') {
      ctx.fillStyle = '#090d16'; // Nền đen chuẩn xử lý nhị phân Blob Analysis
      ctx.fillRect(0, 0, w, h);
    } else if (mode === 'GRAY') {
      ctx.fillStyle = '#1e2430'; // Mặt băng tải mức xám
      ctx.fillRect(0, 0, w, h);
      ctx.fillStyle = '#334155';
      ctx.fillRect(0, 0, w, 14);
      ctx.fillRect(0, h - 14, w, 14);
    } else {
      // COLOR MODE: Băng tải Polyurethane công nghiệp
      ctx.fillStyle = '#0b0f19';
      ctx.fillRect(0, 0, w, h);

      // Mặt belt
      ctx.fillStyle = '#1e293b';
      ctx.fillRect(0, 14, w, h - 28);

      // Vân gân băng tải chuyển động nhẹ
      ctx.strokeStyle = 'rgba(51, 65, 85, 0.6)';
      ctx.lineWidth = 1;
      const seamX = (performance.now() * 0.04) % 40;
      for (let sx = seamX; sx < w; sx += 40) {
        ctx.beginPath();
        ctx.moveTo(sx, 14);
        ctx.lineTo(sx, h - 14);
        ctx.stroke();
      }

      // Thanh nẹp dẫn hướng nhôm anodized
      ctx.fillStyle = '#475569';
      ctx.fillRect(0, 0, w, 14);
      ctx.fillRect(0, h - 14, w, 14);
      ctx.fillStyle = '#64748b';
      ctx.fillRect(0, 12, w, 2);
      ctx.fillRect(0, h - 14, w, 2);
    }

    // 2. Render các phôi sô-cô-la đang di chuyển trên băng tải
    const productsInFOV = prefilteredProducts || this.getProductsInVisionFOV();
    for (let i = 0; i < productsInFOV.length; i++) {
      const item = productsInFOV[i];
      const mesh = item.mesh;
      const relX = mesh.position.x - this.camCenter.x;
      const relZ = mesh.position.z - this.camCenter.z;
      const u = (0.5 + relX / this.fovWidthM) * w;
      const v = (0.5 + relZ / this.fovHeightM) * h;
      const yawRad = mesh.rotation.y || 0;
      const isNG = (item.status === 'NG' || item.isDefective === true);

      const boxW_px = (0.18 / this.fovWidthM) * w; // ~99 px
      const boxH_px = (0.06 / this.fovHeightM) * h; // ~33 px

      ctx.save();
      ctx.translate(u, v);
      ctx.rotate(-yawRad);

      if (mode === 'BINARY') {
        // Cognex Binary: Khối phôi ngưỡng trắng rực (255) nổi bật trên nền đen
        ctx.fillStyle = '#f8fafc';
        ctx.fillRect(-boxW_px / 2, -boxH_px / 2, boxW_px, boxH_px);
      } else if (mode === 'GRAY') {
        // Mức xám 8-bit
        ctx.fillStyle = isNG ? '#475569' : '#94a3b8';
        ctx.fillRect(-boxW_px / 2, -boxH_px / 2, boxW_px, boxH_px);
        ctx.strokeStyle = '#cbd5e1';
        ctx.lineWidth = 1.2;
        ctx.strokeRect(-boxW_px / 2, -boxH_px / 2, boxW_px, boxH_px);
      } else {
        // MÀU SẮC THỰC TẾ (COLOR):
        if (isNG) {
          // Phôi phế phẩm NG: Vỏ giấy bạc đỏ sẫm cháy xém có vết nứt vỡ
          ctx.fillStyle = '#7f1d1d';
          ctx.fillRect(-boxW_px / 2, -boxH_px / 2, boxW_px, boxH_px);
          ctx.strokeStyle = '#ef4444';
          ctx.lineWidth = 1.8;
          ctx.strokeRect(-boxW_px / 2, -boxH_px / 2, boxW_px, boxH_px);

          // Vết nứt vỡ
          ctx.strokeStyle = '#fca5a5';
          ctx.lineWidth = 1.5;
          ctx.beginPath();
          ctx.moveTo(-18, -boxH_px / 2 + 2);
          ctx.lineTo(-2, 0);
          ctx.lineTo(14, boxH_px / 2 - 2);
          ctx.stroke();
        } else {
          // Phôi đạt chuẩn OK: Lớp vỏ giấy bạc ánh kim vàng sang trọng
          const grad = ctx.createLinearGradient(0, -boxH_px / 2, 0, boxH_px / 2);
          grad.addColorStop(0, '#f59e0b');
          grad.addColorStop(0.5, '#fde68a');
          grad.addColorStop(1, '#d97706');
          ctx.fillStyle = grad;
          ctx.fillRect(-boxW_px / 2, -boxH_px / 2, boxW_px, boxH_px);

          // Rãnh dập nổi sô-cô-la
          ctx.strokeStyle = 'rgba(255, 255, 255, 0.5)';
          ctx.lineWidth = 1.2;
          for (let rx = -boxW_px / 2 + 16; rx < boxW_px / 2 - 8; rx += 16) {
            ctx.beginPath();
            ctx.moveTo(rx, -boxH_px / 2 + 3);
            ctx.lineTo(rx, boxH_px / 2 - 3);
            ctx.stroke();
          }
          ctx.strokeStyle = '#b45309';
          ctx.lineWidth = 1;
          ctx.strokeRect(-boxW_px / 2, -boxH_px / 2, boxW_px, boxH_px);
        }
      }

      ctx.restore();
    }

    // 3. Vẽ Thước đo tọa độ Pixel & Lưới tâm quang học (Crosshair Calibration Grid)
    this.drawCalibrationGridOverlay();

    // 4. Trích xuất Đặc trưng (Feature Extraction) & Vẽ Bounding Box / Centroid / Vector góc xoay θ
    this.extractFeaturesAndRenderHUD(productsInFOV, tStart);
  }

  /**
   * Vẽ thước đo tọa độ pixel (Crosshair Grid) và đường laser chuẩn X = 2.80m trên màn hình soi
   */
  drawCalibrationGridOverlay() {
    const ctx = this.ctx2d;
    const w = this.rtWidth;
    const h = this.rtHeight;
    const cx = w / 2;
    const cy = h / 2;

    ctx.save();

    // Lưới chia độ mờ (Sub-grid mỗi 50mm ~ 27.5px)
    ctx.strokeStyle = 'rgba(56, 189, 248, 0.14)';
    ctx.lineWidth = 1;
    const stepPx = (0.05 / this.fovWidthM) * w;
    for (let x = cx % stepPx; x < w; x += stepPx) {
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x, h);
      ctx.stroke();
    }
    for (let y = cy % stepPx; y < h; y += stepPx) {
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(w, y);
      ctx.stroke();
    }

    // Trục tọa độ tâm quang học (Optical Center Crosshair X=0, Y=0)
    ctx.strokeStyle = 'rgba(6, 182, 212, 0.42)';
    ctx.setLineDash([4, 4]);
    ctx.beginPath();
    ctx.moveTo(cx, 0);
    ctx.lineTo(cx, h);
    ctx.moveTo(0, cy);
    ctx.lineTo(w, cy);
    ctx.stroke();
    ctx.setLineDash([]);

    // Thước vạch chia mm dọc cạnh trên và cạnh trái
    ctx.fillStyle = 'rgba(148, 163, 184, 0.75)';
    ctx.font = '9px Consolas, monospace';
    ctx.fillText('-200mm', cx - (0.20 / this.fovWidthM) * w - 14, 10);
    ctx.fillText('0mm', cx + 3, 10);
    ctx.fillText('+200mm', cx + (0.20 / this.fovWidthM) * w - 18, 10);

    // Khung ROI (Region of Interest) chuẩn Cognex In-Sight
    ctx.strokeStyle = 'rgba(56, 189, 248, 0.55)';
    ctx.lineWidth = 1;
    ctx.strokeRect(18, 16, w - 36, h - 32);
    ctx.fillStyle = '#38bdf8';
    ctx.font = 'bold 9px Consolas, monospace';
    ctx.fillText(`ROI [600x400mm] // MODE: ${this.filterMode}`, 22, 27);

    ctx.restore();
  }

  /**
   * Nhận diện Bounding Box, tính trọng tâm Centroid (+), góc xoay θ, diện tích Pixel Area
   * và phân loại PASS [OK] vs FAIL [NG - DEFECT]
   */
  extractFeaturesAndRenderHUD(productsInFOV, tStart) {
    const ctx = this.ctx2d;
    const w = this.rtWidth;
    const h = this.rtHeight;

    // Sắp xếp để lấy phôi gần tâm vùng quét X = 2.80m nhất làm đối tượng phân tích chính
    const sorted = [...productsInFOV].sort(
      (a, b) => Math.abs(a.mesh.position.x - this.camCenter.x) - Math.abs(b.mesh.position.x - this.camCenter.x)
    );

    let primaryTarget = sorted[0] || null;

    ctx.save();

    for (let i = 0; i < sorted.length; i++) {
      const item = sorted[i];
      const mesh = item.mesh;

      // Chuyển đổi tọa độ thế giới (x, z) sang tọa độ pixel (px, py) trên vision-canvas
      // Lưu ý: Trên canvas, trục ngang u đi từ X_min (2.50m) -> X_max (3.10m)
      // Trục dọc v đi từ Z_min (-0.52m) -> Z_max (-0.12m)
      const relX = mesh.position.x - this.camCenter.x; // [-0.30 .. +0.30] m
      const relZ = mesh.position.z - this.camCenter.z; // [-0.20 .. +0.20] m

      const u = (0.5 + relX / this.fovWidthM) * w;
      const v = (0.5 + relZ / this.fovHeightM) * h;

      // Góc xoay thực tế của phôi trên mặt băng tải (rad & deg)
      const yawRad = mesh.rotation.y || 0;
      const yawDeg = Number(THREE.MathUtils.radToDeg(yawRad).toFixed(1));

      // Kích thước vật lý của thanh sô-cô-la: 180mm x 60mm
      const isNG = (item.status === 'NG' || item.isDefective === true);
      // Nếu là phôi lỗi NG: giả lập biến dạng hình học / sứt mẻ diện tích ngoài dung sai
      const lengthMm = isNG ? 142.5 : 180.0;
      const widthMm = isNG ? 74.0 : 60.0;
      const boxW_px = (0.18 / this.fovWidthM) * w; // ~99 px
      const boxH_px = (0.06 / this.fovHeightM) * h; // ~33 px

      const areaPx = isNG ? 2140 : Math.round(boxW_px * boxH_px);
      const aspectRatio = Number((lengthMm / widthMm).toFixed(2));

      // Kiểm tra điều kiện PASS / FAIL
      const areaOk = areaPx >= this.criteria.areaMinPx && areaPx <= this.criteria.areaMaxPx;
      const aspectOk = aspectRatio >= this.criteria.aspectMin && aspectRatio <= this.criteria.aspectMax;
      const passed = !isNG && areaOk && aspectOk;

      // Nếu phát hiện FAIL (NG), đảm bảo đánh dấu cờ để kích hoạt xilanh gạt phế phẩm (Reject Pusher)
      if (!passed) {
        item.isDefective = true;
        item.status = 'NG';
        item.isRejected = true;
      } else {
        item.inspectedYawDeg = yawDeg;
      }

      // Vẽ Oriented Bounding Box bám sát mép phôi
      ctx.save();
      ctx.translate(u, v);
      ctx.rotate(-yawRad);

      const strokeCol = passed ? '#22c55e' : '#ef4444';
      const fillCol = passed ? 'rgba(34, 197, 94, 0.14)' : 'rgba(239, 68, 68, 0.24)';

      ctx.fillStyle = fillCol;
      ctx.fillRect(-boxW_px / 2, -boxH_px / 2, boxW_px, boxH_px);

      ctx.strokeStyle = strokeCol;
      ctx.lineWidth = passed ? 2.0 : 2.5;
      ctx.strokeRect(-boxW_px / 2, -boxH_px / 2, boxW_px, boxH_px);

      // 4 góc ngắm (Corner Fiducials) kiểu Cognex PatMax
      const cLen = 7;
      ctx.lineWidth = 2.5;
      [
        [-boxW_px / 2, -boxH_px / 2, 1, 1],
        [ boxW_px / 2, -boxH_px / 2, -1, 1],
        [-boxW_px / 2,  boxH_px / 2, 1, -1],
        [ boxW_px / 2,  boxH_px / 2, -1, -1]
      ].forEach(([cx0, cy0, sx, sy]) => {
        ctx.beginPath();
        ctx.moveTo(cx0, cy0 + sy * cLen);
        ctx.lineTo(cx0, cy0);
        ctx.lineTo(cx0 + sx * cLen, cy0);
        ctx.stroke();
      });

      // Vẽ tâm phôi (Crosshair '+') tại trọng tâm hình học (Centroid)
      ctx.strokeStyle = '#facc15';
      ctx.lineWidth = 1.6;
      ctx.beginPath();
      ctx.moveTo(-7, 0);
      ctx.lineTo(7, 0);
      ctx.moveTo(0, -7);
      ctx.lineTo(0, 7);
      ctx.stroke();

      // Vẽ mũi tên Vector định hướng chỉ góc lệch xoay hiện tại của phôi (Angle: θ°)
      const arrowLen = boxW_px * 0.42;
      ctx.strokeStyle = '#38bdf8';
      ctx.fillStyle = '#38bdf8';
      ctx.lineWidth = 2.0;
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.lineTo(arrowLen, 0);
      ctx.stroke();
      // Đầu mũi tên
      ctx.beginPath();
      ctx.moveTo(arrowLen + 5, 0);
      ctx.lineTo(arrowLen - 3, -4);
      ctx.lineTo(arrowLen - 3, 4);
      ctx.closePath();
      ctx.fill();

      ctx.restore();

      // Nhãn thông số nổi ngay trên phôi (HUD Tag)
      const offsetX_mm = Number((relX * 1000).toFixed(1));
      const offsetY_mm = Number((relZ * 1000).toFixed(1));
      const labelY = Math.max(38, v - boxH_px / 2 - 10);
      const labelX = Math.min(w - 135, Math.max(8, u - 55));

      ctx.fillStyle = 'rgba(9, 13, 22, 0.85)';
      ctx.fillRect(labelX, labelY - 11, 132, 15);
      ctx.strokeStyle = strokeCol;
      ctx.lineWidth = 1;
      ctx.strokeRect(labelX, labelY - 11, 132, 15);

      ctx.fillStyle = strokeCol;
      ctx.font = 'bold 9.5px Consolas, monospace';
      const statusShort = passed ? 'PASS [OK]' : 'FAIL [NG-DEFECT]';
      ctx.fillText(`${statusShort} θ:${yawDeg >= 0 ? '+' : ''}${yawDeg}°`, labelX + 4, labelY);

      // Nếu đây là phôi chính gần tâm quét nhất, lưu vào telemetry
      if (item === primaryTarget) {
        const procMs = Math.max(8, Math.min(16, Math.round(performance.now() - tStart + 9)));
        this.lastInspection = {
          hasTarget: true,
          status: passed ? 'INSPECTION: PASS [OK]' : 'INSPECTION: FAIL [NG - DEFECT]',
          passed,
          offsetX_mm,
          offsetY_mm,
          angleDeg: yawDeg,
          lengthMm,
          widthMm,
          areaPx,
          aspectRatio,
          processingTimeMs: procMs
        };
      }
    }

    // Banner HUD trạng thái tổng quát góc dưới màn hình Vision
    const bannerH = 22;
    ctx.fillStyle = 'rgba(9, 13, 22, 0.88)';
    ctx.fillRect(0, h - bannerH, w, bannerH);

    const info = this.lastInspection;
    ctx.fillStyle = info.passed ? '#4ade80' : '#f87171';
    ctx.font = 'bold 10px Consolas, monospace';
    const signX = info.offsetX_mm >= 0 ? '+' : '';
    const signY = info.offsetY_mm >= 0 ? '+' : '';
    const signA = info.angleDeg >= 0 ? '+' : '';
    ctx.fillText(
      `${info.status} | X:${signX}${info.offsetX_mm} Y:${signY}${info.offsetY_mm}mm θ:${signA}${info.angleDeg}°`,
      8,
      h - 7
    );

    ctx.restore();

    // Cập nhật các phần tử DOM trên cửa sổ COGNEX VISION PRO
    this.updateVisionStudioDOM();
  }

  /**
   * Cập nhật thanh trạng thái bên dưới (Inspection Telemetry) của cửa sổ Vision Studio
   */
  updateVisionStudioDOM() {
    const info = this.lastInspection;

    const statusBadge = document.getElementById('vision-ui-status-badge');
    const centroidEl = document.getElementById('vision-ui-centroid');
    const areaEl = document.getElementById('vision-ui-area');
    const procTimeEl = document.getElementById('vision-ui-proctime');
    const winEl = document.getElementById('cognex-vision-floating-window');

    if (statusBadge) {
      statusBadge.textContent = info.passed ? 'PASS [OK]' : 'FAIL [NG - DEFECT]';
      statusBadge.className = info.passed
        ? 'vision-status-badge pass'
        : 'vision-status-badge fail';
    }

    if (winEl) {
      winEl.classList.toggle('vision-alert-ng', !info.passed);
    }

    if (centroidEl) {
      const sx = info.offsetX_mm >= 0 ? '+' : '';
      const sy = info.offsetY_mm >= 0 ? '+' : '';
      const sa = info.angleDeg >= 0 ? '+' : '';
      centroidEl.innerHTML = `X=<b>${sx}${info.offsetX_mm.toFixed(1)}</b>mm, Y=<b>${sy}${info.offsetY_mm.toFixed(1)}</b>mm, Angle=<b>${sa}${info.angleDeg.toFixed(1)}°</b>`;
    }

    if (areaEl) {
      areaEl.innerHTML = `Area: <b>${info.areaPx}</b> px² | Size: <b>${info.lengthMm}×${info.widthMm}</b> mm`;
    }

    if (procTimeEl) {
      procTimeEl.textContent = `${info.processingTimeMs} ms`;
    }
  }
}
