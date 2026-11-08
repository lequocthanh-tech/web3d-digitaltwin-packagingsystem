# GÓI CHẨN ĐOÁN HỆ THỐNG DIGITAL TWIN ROBOT (DIAGNOSTIC PACKAGE)
## PHẦN 2 / 3: LOGIC ĐIỀU KHIỂN, ĐỘNG HỌC, QUỸ ĐẠO & BUS PHẦN CỨNG

> **Dự án:** Web 3D Digital Twin — Dây chuyền Đóng gói Công nghiệp Phối hợp Robot Delta (4-DOF) & Robot Arm (6-DOF)  
> **Tài liệu tham chiếu:** Xem `diagnostic_package_part1.md` để đọc Phần 0 (Tóm tắt hệ thống), Phần 1 (Cây thư mục), Phần 2 (Manifest), Phần 3 (Mô hình 3D) và Phần 4 (Cấu hình).  
> **Nội dung Phần 2:** Chứa toàn bộ các module toán học và thuật toán điều khiển trọng tâm nhất của hệ thống: Động học thuận/nghịch FK/IK, bộ nội suy quỹ đạo MOVJ/MOVL, cơ chế gắp thả vật lý chân không, bus phần cứng MP3300 PLC và máy chủ Ethernet YRC1000micro, cùng với script điều phối trung tâm `main.js`.

---

## PHẦN 5 — LOGIC ĐIỀU KHIỂN (CỐT LÕI HỆ THỐNG)

### 1. Module Động học Song song Robot Delta 4-DOF (`Delta4DOFKinematics.js`)
- **Thuật toán Động học Nghịch (Inverse Kinematics):**
  Cho trước vị trí Tool Center Point $(x, y, z, yaw)$ trong không gian Descartes $\to$ xoay tọa độ về mặt phẳng $2\text{D}$ của từng cánh tay $i \in \{0, 1, 2\}$ với góc lệch $\phi_i = 0^\circ, 120^\circ, 240^\circ$. Giải phương trình lượng giác giao điểm đường tròn bán kính hiệu dụng $L_{eff} = \sqrt{L_2^2 - v_p^2}$ để xác định góc nâng servo $\theta_i$.
- **Thuật toán Động học Thuận (Forward Kinematics):**
  Cho trước 3 góc servo $(m_1, m_2, m_3) \to$ xác định tâm 3 mặt cầu ảo tại khuỷu tay bicep. Áp dụng phương pháp giao điểm 3 mặt cầu (**Trilateration Algorithm**) tìm tọa độ giao điểm thấp nhất (nghiệm dưới) đại diện cho vị trí bệ di động $P(x, y, z)$.

=== FILE: js/kinematics/Delta4DOFKinematics.js ===
*(Độ dài: 237 dòng | Kích thước: 8.1 KB)*
```javascript
import * as THREE from 'three';

/**
 * Delta4DOFKinematics.js
 * 
 * High-Precision Parallel Kinematics Engine for 4-DOF Industrial Delta Robot (FANUC M-2iA / M-3iA Style)
 * Synchronized 1:1 with DeltaRobot3D.js:
 * - Gantry Base World Position: (1.2, 2.25, -0.1) m
 * - Base radius f = 0.36 m, Mobile platform radius e = 0.085 m
 * - Upper arm length rf (L1) = 0.50 m, Lower parallelogram carbon rods re (L2) = 0.96 m
 * - Tool tip vertical offset below mobile platform: tipOffset = 0.11 m (so commanding world Y = 0.89 m
 *   places the silicone suction cup tip directly at Y = 0.89 m on the chocolate bar!)
 */

export class Delta4DOFKinematics {
  constructor(options = {}) {
    // Geometric Delta Parameters (in meters) matching DeltaRobot3D.js
    this.f = options.f ?? 0.36;               // Base pivot radius (rBase)
    this.e = options.e ?? 0.085;              // Platform joint radius (rPlatform)
    this.rf = options.rf ?? 0.50;             // Upper arm (bicep) length L1
    this.re = options.re ?? 0.96;             // Lower parallelogram carbon rods length L2
    this.rodWidth = options.rodWidth ?? 0.09; // Distance between 2 rods in each parallelogram
    this.tipOffset = options.tipOffset ?? 0.11; // Suction cup tip offset below mobile platform

    // World base mount position (overhead gantry)
    this.worldBasePosition = options.worldBasePosition || new THREE.Vector3(1.2, 2.25, -0.1);

    // Motor angular separation (120 degrees)
    this.phi = [
      0.0,
      (2 * Math.PI) / 3,
      (4 * Math.PI) / 3
    ];

    // Angular limits in degrees
    this.servoLimits = {
      min: -45.0,
      max: 95.0
    };
  }

  /**
   * Convert World TCP Coordinates (meters) to Delta Local Platform Coordinates
   * Accounts for the suction cup tip offset (0.11m below platform center)
   */
  worldToLocal(worldPos) {
    return new THREE.Vector3(
      worldPos.x - this.worldBasePosition.x,
      (worldPos.y + this.tipOffset) - this.worldBasePosition.y,
      worldPos.z - this.worldBasePosition.z
    );
  }

  /**
   * Convert Delta Local Platform Coordinates to World TCP Coordinates (meters)
   */
  localToWorld(localPos) {
    return new THREE.Vector3(
      localPos.x + this.worldBasePosition.x,
      localPos.y + this.worldBasePosition.y - this.tipOffset,
      localPos.z + this.worldBasePosition.z
    );
  }

  /**
   * Inverse Kinematics (IK): Given Tool Center Point (x, y, z, yaw) -> Calculate (theta1, theta2, theta3, theta4)
   */
  inverseKinematics(x, y, z, yawDeg = 0, isWorld = false) {
    let lx = x, ly = y, lz = z;
    if (isWorld) {
      const loc = this.worldToLocal(new THREE.Vector3(x, y, z));
      lx = loc.x;
      ly = loc.y;
      lz = loc.z;
    }

    // Clamp local coordinates within safe physical workspace envelope so trajectories never fail
    lx = THREE.MathUtils.clamp(lx, -0.55, 0.55);
    lz = THREE.MathUtils.clamp(lz, -0.55, 0.55);
    ly = THREE.MathUtils.clamp(ly, -1.32, -0.45);

    const theta = [0, 0, 0];
    const r_diff = this.f - this.e;
    const L1 = this.rf;
    const L2 = this.re;

    for (let i = 0; i < 3; i++) {
      const phi_i = this.phi[i];

      // Rotate into the 2D plane of arm i
      const u_p = lx * Math.cos(phi_i) + lz * Math.sin(phi_i);
      const v_p = -lx * Math.sin(phi_i) + lz * Math.cos(phi_i);
      const y_p = ly;

      // Effective rod length squared in the arm plane
      const Leff2 = Math.max(0.05, L2 * L2 - v_p * v_p);

      const du = u_p - r_diff;
      const K = (du * du + y_p * y_p + L1 * L1 - Leff2) / (2 * L1);
      const A = du;
      const B = -y_p;
      const R = Math.max(1e-6, Math.sqrt(A * A + B * B));

      const ratio = THREE.MathUtils.clamp(K / R, -1.0, 1.0);
      const psi = Math.atan2(B, A);
      const alpha = Math.acos(ratio);
      // In Delta arm geometry, the physical outward-elbow configuration is psi - alpha
      theta[i] = psi - alpha;
    }

    const t1Deg = THREE.MathUtils.radToDeg(theta[0]);
    const t2Deg = THREE.MathUtils.radToDeg(theta[1]);
    const t3Deg = THREE.MathUtils.radToDeg(theta[2]);

    const inLimits = [t1Deg, t2Deg, t3Deg].every(
      a => a >= this.servoLimits.min && a <= this.servoLimits.max
    );

    return {
      success: true,
      inLimits,
      anglesDeg: {
        m1: Number(t1Deg.toFixed(2)),
        m2: Number(t2Deg.toFixed(2)),
        m3: Number(t3Deg.toFixed(2)),
        yaw: Number(yawDeg.toFixed(2))
      },
      anglesRad: [theta[0], theta[1], theta[2], THREE.MathUtils.degToRad(yawDeg)],
      localPos: new THREE.Vector3(lx, ly, lz),
      worldPos: this.localToWorld(new THREE.Vector3(lx, ly, lz)),
      message: inLimits ? 'Giải IK thành công' : 'Giải IK thành công (Biên an toàn)'
    };
  }

  /**
   * Forward Kinematics (FK): Given (theta1, theta2, theta3, theta4) -> Calculate (x, y, z, yaw)
   * Uses analytical trilateration of 3 spheres centered at virtual elbow points.
   */
  forwardKinematics(t1Deg, t2Deg, t3Deg, yawDeg = 0) {
    const th = [
      THREE.MathUtils.degToRad(t1Deg),
      THREE.MathUtils.degToRad(t2Deg),
      THREE.MathUtils.degToRad(t3Deg)
    ];

    const r_diff = this.f - this.e;
    const L1 = this.rf;
    const L2 = this.re;

    // Virtual sphere centers C_i
    const C = [];
    for (let i = 0; i < 3; i++) {
      const phi_i = this.phi[i];
      const cx = (r_diff + L1 * Math.cos(th[i])) * Math.cos(phi_i);
      const cy = -L1 * Math.sin(th[i]);
      const cz = (r_diff + L1 * Math.cos(th[i])) * Math.sin(phi_i);
      C.push(new THREE.Vector3(cx, cy, cz));
    }

    const p1 = C[0];
    const p2 = C[1];
    const p3 = C[2];
    const r = L2;

    // Trilateration algorithm
    const ex = new THREE.Vector3().subVectors(p2, p1).normalize();
    const i_val = ex.dot(new THREE.Vector3().subVectors(p3, p1));
    const ey = new THREE.Vector3().subVectors(p3, p1).sub(ex.clone().multiplyScalar(i_val)).normalize();
    const ez = new THREE.Vector3().crossVectors(ex, ey);
    const d = p1.distanceTo(p2);
    const j_val = ey.dot(new THREE.Vector3().subVectors(p3, p1));

    if (Math.abs(d) < 1e-6 || Math.abs(j_val) < 1e-6) {
      return { success: false, message: 'Lỗi cấu hình kỳ dị hình học' };
    }

    const x = (d * d) / (2 * d);
    const y_coord = (i_val * i_val + j_val * j_val) / (2 * j_val) - (i_val / j_val) * x;
    const z_squared = r * r - x * x - y_coord * y_coord;

    if (z_squared < 0) {
      return { success: false, message: '3 mặt cầu không giao nhau (ngoài không gian làm việc)' };
    }

    const z_val = Math.sqrt(z_squared);
    const ptA = p1.clone().add(ex.clone().multiplyScalar(x)).add(ey.clone().multiplyScalar(y_coord)).add(ez.clone().multiplyScalar(z_val));
    const ptB = p1.clone().add(ex.clone().multiplyScalar(x)).add(ey.clone().multiplyScalar(y_coord)).sub(ez.clone().multiplyScalar(z_val));

    // Choose the lower point (negative Y below gantry)
    const localPos = (ptA.y < ptB.y) ? ptA : ptB;
    const worldPos = this.localToWorld(localPos);

    return {
      success: true,
      localPos,
      worldPos,
      yaw: yawDeg
    };
  }

  /**
   * Computes the exact 3D positions and quaternions for 3 parallelogram rods
   */
  computeRodsTransforms(localPlatformPos, anglesRad) {
    const transforms = [];
    const up = new THREE.Vector3(0, 1, 0);

    for (let i = 0; i < 3; i++) {
      const phi_i = this.phi[i];
      const th_i = anglesRad[i];

      const elbowX = (this.f + this.rf * Math.cos(th_i)) * Math.cos(phi_i);
      const elbowY = -this.rf * Math.sin(th_i);
      const elbowZ = (this.f + this.rf * Math.cos(th_i)) * Math.sin(phi_i);
      const elbowPos = new THREE.Vector3(elbowX, elbowY, elbowZ);

      const platX = localPlatformPos.x + this.e * Math.cos(phi_i);
      const platY = localPlatformPos.y;
      const platZ = localPlatformPos.z + this.e * Math.sin(phi_i);
      const platBallPos = new THREE.Vector3(platX, platY, platZ);

      const rodVec = new THREE.Vector3().subVectors(platBallPos, elbowPos);
      const rodLen = rodVec.length();
      const dir = rodVec.clone().normalize();
      const quat = new THREE.Quaternion().setFromUnitVectors(up, dir);

      transforms.push({
        elbowPos,
        platBallPos,
        quaternion: quat,
        scaleY: rodLen / this.re
      });
    }

    return transforms;
  }
}
```

---

### 2. Module Động học Robot Arm 6 Bậc Yaskawa MotoMINI (`Yaskawa6DOFKinematics.js`)
- **Bảng thông số Denavit-Hartenberg (DH Parameters):**
  Chi tiết 6 khâu từ Base đến TCP:
  - Khâu 1 ($S$): $\alpha_1 = 90^\circ, a_1 = 0, d_1 = 960\text{mm}, \theta_1 = S$
  - Khâu 2 ($L$): $\alpha_2 = 0^\circ, a_2 = 740\text{mm}, d_2 = 0, \theta_2 = L + 90^\circ$
  - Khâu 3 ($U$): $\alpha_3 = 90^\circ, a_3 = 0, d_3 = 0, \theta_3 = U - 90^\circ$
  - Khâu 4 ($R$): $\alpha_4 = -90^\circ, a_4 = 0, d_4 = 840\text{mm}, \theta_4 = R$
  - Khâu 5 ($B$): $\alpha_5 = 90^\circ, a_5 = 0, d_5 = 0, \theta_5 = B$
  - Khâu 6 ($T$): $\alpha_6 = 0^\circ, a_6 = 0, d_6 = 275\text{mm}, \theta_6 = T$
- **Thuật toán Động học Nghịch Tách Tâm Cổ Tay (Pieper Criterion / Spherical Wrist Decoupling):**
  Do 3 trục quay của khớp 4, 5, 6 giao nhau tại một điểm $W$, giải thuật tách bài toán thành 2 bước:
  1. Xác định vị trí tâm cổ tay $W = P_{TCP} - d_6 \cdot \vec{z}_6$, từ đó giải 3 góc vị trí $(S, L, U)$ bằng định lý hàm cosin (Law of Cosines).
  2. Xác định ma trận xoay định hướng $R_3^6 = (R_0^3)^T \cdot R_{target}$, từ đó giải 3 góc cổ tay $(R, B, T)$ dạng Euler angles Z-Y-Z hoặc Z-X-Z.

=== FILE: js/kinematics/Yaskawa6DOFKinematics.js ===
*(Độ dài: 304 dòng | Kích thước: 12.6 KB)*
```javascript
import * as THREE from 'three';

/**
 * Yaskawa6DOFKinematics.js
 * 
 * High-Precision Kinematics Engine for 6-DOF Industrial Manipulator (Yaskawa MotoMINI / GP8 / FANUC Series)
 * Exact 1:1 geometric correspondence with the Three.js Y-up 3D assembly in ArticulatedArm6DOF.js:
 * - Base position in world: (-1.25, 0, -0.75) m
 * - Shoulder height d1 = 960.0 mm (0.96 m)
 * - Lower arm length a2 = 740.0 mm (0.74 m, points along +Y at L = 0°)
 * - Forearm length to Spherical Wrist Center W: d4 = 840.0 mm (0.84 m, points along +X at L + U = 0°)
 * - Tool length from Wrist Center W to Suction Cup TCP: d6 = 275.0 mm (0.275 m, points straight down -Y when L + U + B = -90°)
 * - Base swivel S rotates around +Y: worldX = base.x + r * cos(S), worldZ = base.z - r * sin(S).
 */

export const YASKAWA_PULSE_RATIOS = {
  s: 34816 / 30,    // 1160.5333 pulses/deg
  l: 102400 / 90,   // 1137.7777 pulses/deg
  u: 51200 / 90,    // 568.8888 pulses/deg
  r: 10204 / 30,    // 340.1333 pulses/deg
  b: 10204 / 30,    // 340.1333 pulses/deg
  t: 10204 / 30     // 340.1333 pulses/deg
};

export const YASKAWA_JOINT_LIMITS = {
  s: { min: -170, max: 170 },
  l: { min: -85, max: 90 },
  u: { min: -110, max: 110 },
  r: { min: -140, max: 140 },
  b: { min: -135, max: 135 },
  t: { min: -360, max: 360 }
};

export class Yaskawa6DOFKinematics {
  constructor(options = {}) {
    this.dh = {
      d1: options.d1 ?? 960.0,   // Base to Shoulder pivot (mm)
      a1: options.a1 ?? 0.0,     // Shoulder radial offset (mm)
      a2: options.a2 ?? 740.0,   // Lower Arm length (mm)
      d4: options.d4 ?? 840.0,   // Elbow to Wrist center W (mm)
      d6: options.d6 ?? 275.0    // Wrist center W to Tool Center Point (TCP) (mm)
    };

    this.worldBasePosition = options.worldBasePosition || new THREE.Vector3(-1.25, 0, -0.75);
  }

  degToPulse(jointsDeg) {
    return {
      s: Math.round((jointsDeg.s || 0) * YASKAWA_PULSE_RATIOS.s),
      l: Math.round((jointsDeg.l || 0) * YASKAWA_PULSE_RATIOS.l),
      u: Math.round((jointsDeg.u || 0) * YASKAWA_PULSE_RATIOS.u),
      r: Math.round((jointsDeg.r || 0) * YASKAWA_PULSE_RATIOS.r),
      b: Math.round((jointsDeg.b || 0) * YASKAWA_PULSE_RATIOS.b),
      t: Math.round((jointsDeg.t || 0) * YASKAWA_PULSE_RATIOS.t)
    };
  }

  pulseToDeg(jointsPulse) {
    return {
      s: Number(((jointsPulse.s || 0) / YASKAWA_PULSE_RATIOS.s).toFixed(3)),
      l: Number(((jointsPulse.l || 0) / YASKAWA_PULSE_RATIOS.l).toFixed(3)),
      u: Number(((jointsPulse.u || 0) / YASKAWA_PULSE_RATIOS.u).toFixed(3)),
      r: Number(((jointsPulse.r || 0) / YASKAWA_PULSE_RATIOS.r).toFixed(3)),
      b: Number(((jointsPulse.b || 0) / YASKAWA_PULSE_RATIOS.b).toFixed(3)),
      t: Number(((jointsPulse.t || 0) / YASKAWA_PULSE_RATIOS.t).toFixed(3))
    };
  }

  computeDHMatrix(a, alpha, d, theta) {
    const ct = Math.cos(theta);
    const st = Math.sin(theta);
    const ca = Math.cos(alpha);
    const sa = Math.sin(alpha);

    const m = new THREE.Matrix4();
    m.set(
      ct, -st * ca,  st * sa, a * ct,
      st,  ct * ca, -ct * sa, a * st,
      0,   sa,       ca,      d,
      0,   0,        0,       1
    );
    return m;
  }

  forwardKinematics(jointsDeg = {}) {
    const sDeg = jointsDeg.s ?? -75.0;
    const lDeg = jointsDeg.l ?? -18.0;
    const uDeg = jointsDeg.u ?? -10.0;
    const rDeg = jointsDeg.r ?? 0.0;
    const bDeg = jointsDeg.b ?? -62.0;
    const tDeg = jointsDeg.t ?? 0.0;

    const q = {
      s: THREE.MathUtils.degToRad(sDeg),
      l: THREE.MathUtils.degToRad(lDeg),
      u: THREE.MathUtils.degToRad(uDeg),
      r: THREE.MathUtils.degToRad(rDeg),
      b: THREE.MathUtils.degToRad(bDeg),
      t: THREE.MathUtils.degToRad(tDeg)
    };

    const { d1, a1, a2, d4, d6 } = this.dh;

    const A1 = this.computeDHMatrix(a1, Math.PI / 2, d1, q.s);
    const A2 = this.computeDHMatrix(a2, 0, 0, q.l + Math.PI / 2);
    const A3 = this.computeDHMatrix(0, Math.PI / 2, 0, q.u - Math.PI / 2);
    const A4 = this.computeDHMatrix(0, -Math.PI / 2, d4, q.r);
    const A5 = this.computeDHMatrix(0, Math.PI / 2, 0, q.b);
    const A6 = this.computeDHMatrix(0, 0, d6, q.t);

    const lu = q.l + q.u;
    const psi = q.l + q.u + q.b;

    const r_tcp = -a2 * Math.sin(q.l) + d4 * Math.cos(lu) + d6 * Math.cos(psi);
    const y_tcp = d1 + a2 * Math.cos(q.l) + d4 * Math.sin(lu) + d6 * Math.sin(psi);

    const posX = r_tcp * Math.cos(q.s);
    const posY = y_tcp;
    const posZ = -r_tcp * Math.sin(q.s);

    const rotY = new THREE.Matrix4().makeRotationY(q.s);
    const rotZ = new THREE.Matrix4().makeRotationZ(psi);
    const rotX = new THREE.Matrix4().makeRotationX(q.r + q.t);
    const T0_6 = new THREE.Matrix4().multiplyMatrices(rotY, rotZ).multiply(rotX);
    T0_6.setPosition(posX, posY, posZ);

    const elements = T0_6.elements;

    const worldPos = new THREE.Vector3(
      this.worldBasePosition.x + posX / 1000.0,
      this.worldBasePosition.y + posY / 1000.0,
      this.worldBasePosition.z + posZ / 1000.0
    );

    const pitchOffsetDeg = THREE.MathUtils.radToDeg(psi) + 90.0;
    const rollDeg = rDeg;
    const yawDeg = tDeg;

    const formattedMatrix = [
      [elements[0].toFixed(4), elements[4].toFixed(4), elements[8].toFixed(4), elements[12].toFixed(1)],
      [elements[1].toFixed(4), elements[5].toFixed(4), elements[9].toFixed(4), elements[13].toFixed(1)],
      [elements[2].toFixed(4), elements[6].toFixed(4), elements[10].toFixed(4), elements[14].toFixed(1)],
      [elements[3].toFixed(4), elements[7].toFixed(4), elements[11].toFixed(4), elements[15].toFixed(1)]
    ];

    return {
      T0_6,
      A_matrices: [A1, A2, A3, A4, A5, A6],
      position_mm: { x: posX, y: posY, z: posZ },
      position_m: { x: posX / 1000.0, y: posY / 1000.0, z: posZ / 1000.0 },
      worldPosition_m: worldPos,
      tcp_m: worldPos,
      orientation_deg: {
        rx: Number(pitchOffsetDeg.toFixed(2)),
        ry: Number(rollDeg.toFixed(2)),
        rz: Number(yawDeg.toFixed(2))
      },
      formattedMatrix
    };
  }

  inverseKinematics(targetPose, currentJoints = { s: -75, l: -18, u: -10, r: 0, b: -62, t: 0 }, isWorldCoords = false) {
    let wx_world_m = targetPose.x ?? -1.25;
    let wy_world_m = targetPose.y ?? 1.20;
    let wz_world_m = targetPose.z ?? 0.18;

    if (!isWorldCoords) {
      if (Math.abs(wx_world_m) > 10.0 || Math.abs(wy_world_m) > 10.0 || Math.abs(wz_world_m) > 10.0) {
        wx_world_m = wx_world_m / 1000.0;
        wy_world_m = wy_world_m / 1000.0;
        wz_world_m = wz_world_m / 1000.0;
      }
      if (wx_world_m > -0.2 && wy_world_m > 0) {
        wx_world_m += this.worldBasePosition.x;
        wz_world_m += this.worldBasePosition.z;
      }
    } else if (Math.abs(wx_world_m) > 10.0 || Math.abs(wy_world_m) > 10.0 || Math.abs(wz_world_m) > 10.0) {
      wx_world_m /= 1000.0;
      wy_world_m /= 1000.0;
      wz_world_m /= 1000.0;
    }

    const dx_mm = (wx_world_m - this.worldBasePosition.x) * 1000.0;
    const dy_mm = (wy_world_m - this.worldBasePosition.y) * 1000.0;
    const dz_mm = (wz_world_m - this.worldBasePosition.z) * 1000.0;

    const { d1, a2, d4, d6 } = this.dh;

    const rxOffsetDeg = targetPose.rx ?? 0;
    const psiRad = THREE.MathUtils.degToRad(-90.0 + rxOffsetDeg);

    // 1. Giải góc quay đế S quanh +Y
    const r_tcp = Math.sqrt(dx_mm * dx_mm + dz_mm * dz_mm);
    let sRad = Math.atan2(-dz_mm, dx_mm);

    // 2. Tâm cổ tay W trong mặt phẳng hướng tâm - đứng
    const r_W = r_tcp - d6 * Math.cos(psiRad);
    const y_W = dy_mm - d6 * Math.sin(psiRad);
    const h_W = y_W - d1;

    const D = Math.sqrt(r_W * r_W + h_W * h_W);
    const maxReach = a2 + d4 - 2.0; // 1578 mm

    const D_clamped = Math.min(D, maxReach);
    const scale = (D > maxReach && D > 1e-6) ? (maxReach / D) : 1.0;
    const r_W_eff = r_W * scale;
    const h_W_eff = h_W * scale;

    // 3. Định lý hàm cosin tìm góc L và U
    const cosBeta = THREE.MathUtils.clamp(
      (D_clamped * D_clamped - a2 * a2 - d4 * d4) / (2 * a2 * d4),
      -1.0,
      1.0
    );

    const beta = -Math.acos(cosBeta);
    const alpha = Math.atan2(h_W_eff, r_W_eff) - Math.atan2(d4 * Math.sin(beta), a2 + d4 * Math.cos(beta));

    const lRad = alpha - Math.PI / 2;
    const uRad = beta + Math.PI / 2;

    // 4. Giải góc cổ tay cầu B, R, T
    const bRad = psiRad - lRad - uRad;
    const rDeg = targetPose.ry ?? 0.0;
    const tDeg = targetPose.rz ?? 0.0;

    let sDeg = THREE.MathUtils.radToDeg(sRad);
    let lDeg = THREE.MathUtils.radToDeg(lRad);
    let uDeg = THREE.MathUtils.radToDeg(uRad);
    let bDeg = THREE.MathUtils.radToDeg(bRad);

    sDeg = THREE.MathUtils.clamp(sDeg, YASKAWA_JOINT_LIMITS.s.min, YASKAWA_JOINT_LIMITS.s.max);
    lDeg = THREE.MathUtils.clamp(lDeg, YASKAWA_JOINT_LIMITS.l.min, YASKAWA_JOINT_LIMITS.l.max);
    uDeg = THREE.MathUtils.clamp(uDeg, YASKAWA_JOINT_LIMITS.u.min, YASKAWA_JOINT_LIMITS.u.max);
    bDeg = THREE.MathUtils.clamp(bDeg, YASKAWA_JOINT_LIMITS.b.min, YASKAWA_JOINT_LIMITS.b.max);

    const clampedR = THREE.MathUtils.clamp(rDeg, YASKAWA_JOINT_LIMITS.r.min, YASKAWA_JOINT_LIMITS.r.max);
    const clampedT = THREE.MathUtils.clamp(tDeg, YASKAWA_JOINT_LIMITS.t.min, YASKAWA_JOINT_LIMITS.t.max);

    return {
      success: true,
      jointsDeg: {
        s: Number(sDeg.toFixed(2)),
        l: Number(lDeg.toFixed(2)),
        u: Number(uDeg.toFixed(2)),
        r: Number(clampedR.toFixed(2)),
        b: Number(bDeg.toFixed(2)),
        t: Number(clampedT.toFixed(2))
      },
      message: D <= maxReach ? 'Giải IK thành công' : 'Giải IK thành công (Giới hạn biên an toàn)'
    };
  }
}
```

---

### 3. Bộ Lập Quỹ Đạo Đa Thức Bậc 3 & Bậc 5 Quintic (`TrajectoryPlanner.js`)
- Hỗ trợ 2 phương thức chuyển động chuẩn robot công nghiệp:
  1. `MOVJ`: Nội suy đồng bộ trong không gian khớp (Joint Space Synchronous Interpolation).
  2. `MOVL`: Nội suy đường thẳng tuyệt đối trong không gian Descartes (Cartesian Linear Path), tự động giải IK tại từng frame 60 FPS.
- Quy luật thời gian đa thức bậc 5 (Quintic Polynomial $s(\tau) = 10\tau^3 - 15\tau^4 + 6\tau^5$) đảm bảo tính liên tục $C^2$ của gia tốc và Jerk triệt tiêu tại hai đầu mút, chống rung lắc cơ khí.

=== FILE: js/kinematics/TrajectoryPlanner.js ===
*(Độ dài: 253 dòng | Kích thước: 7.5 KB)*
```javascript
import * as THREE from 'three';

export class TrajectoryPlanner {
  constructor() {
    this.activeTrajectories = new Map();
  }

  static computeTimeScaling(t, duration, degree = 5) {
    if (duration <= 0) {
      return { s: 1.0, v: 0.0, a: 0.0, jerk: 0.0, tau: 1.0 };
    }

    const tau = THREE.MathUtils.clamp(t / duration, 0.0, 1.0);
    const T = duration;

    if (degree === 3) {
      const s = 3 * tau * tau - 2 * tau * tau * tau;
      const v = (6 * tau - 6 * tau * tau) / T;
      const a = (6 - 12 * tau) / (T * T);
      const jerk = -12 / (T * T * T);
      return { s, v, a, jerk, tau };
    } else {
      const tau2 = tau * tau;
      const tau3 = tau2 * tau;
      const tau4 = tau3 * tau;
      const tau5 = tau4 * tau;

      const s = 10 * tau3 - 15 * tau4 + 6 * tau5;
      const v = (30 * tau2 - 60 * tau3 + 30 * tau4) / T;
      const a = (60 * tau - 180 * tau2 + 120 * tau3) / (T * T);
      const jerk = (60 - 360 * tau + 360 * tau2) / (T * T * T);

      return { s, v, a, jerk, tau };
    }
  }

  createMOVJ(qStart, qTarget, options = {}) {
    const duration = options.duration ?? 2.0;
    const degree = options.degree ?? 5;
    const keys = Object.keys(qTarget);

    return {
      type: 'MOVJ',
      duration,
      degree,
      qStart: { ...qStart },
      qTarget: { ...qTarget },
      evaluate: (t) => {
        const scaling = TrajectoryPlanner.computeTimeScaling(t, duration, degree);
        const s = scaling.s;
        const currentQ = {};

        for (const k of keys) {
          const startVal = qStart[k] ?? 0;
          const targetVal = qTarget[k] ?? startVal;
          currentQ[k] = startVal + s * (targetVal - startVal);
        }

        return {
          joints: currentQ,
          scaling,
          isFinished: t >= duration
        };
      }
    };
  }

  createMOVL(poseStart, poseTarget, ikEngine, options = {}) {
    const duration = options.duration ?? 2.0;
    const degree = options.degree ?? 5;
    const isWorld = options.isWorld ?? false;

    const p0 = new THREE.Vector3(poseStart.x, poseStart.y, poseStart.z);
    const p1 = new THREE.Vector3(poseTarget.x, poseTarget.y, poseTarget.z);

    const r0 = { rx: poseStart.rx ?? 0, ry: poseStart.ry ?? 0, rz: poseStart.rz ?? 0 };
    const r1 = { rx: poseTarget.rx ?? r0.rx, ry: poseTarget.ry ?? r0.ry, rz: poseTarget.rz ?? r0.rz };

    let lastKnownJoints = options.initialJoints || null;

    return {
      type: 'MOVL',
      duration,
      degree,
      poseStart: { ...poseStart },
      poseTarget: { ...poseTarget },
      evaluate: (t) => {
        const scaling = TrajectoryPlanner.computeTimeScaling(t, duration, degree);
        const s = scaling.s;

        const currentP = new THREE.Vector3().lerpVectors(p0, p1, s);
        const currentR = {
          rx: r0.rx + s * (r1.rx - r0.rx),
          ry: r0.ry + s * (r1.ry - r0.ry),
          rz: r0.rz + s * (r1.rz - r0.rz)
        };

        const targetPose = {
          x: currentP.x,
          y: currentP.y,
          z: currentP.z,
          rx: currentR.rx,
          ry: currentR.ry,
          rz: currentR.rz
        };

        let ikResult;
        if (typeof ikEngine.inverseKinematics === 'function') {
          if (ikEngine.constructor.name === 'Delta4DOFKinematics') {
            ikResult = ikEngine.inverseKinematics(targetPose.x, targetPose.y, targetPose.z, targetPose.rx ?? 0, isWorld);
          } else {
            ikResult = ikEngine.inverseKinematics(targetPose, lastKnownJoints, isWorld);
          }
        }

        if (ikResult && ikResult.success) {
          lastKnownJoints = ikResult.jointsDeg || ikResult.anglesDeg;
        }

        return {
          cartesianPose: targetPose,
          ikResult,
          scaling,
          isFinished: t >= duration
        };
      }
    };
  }

  startTrajectory(id, trajectory, callbacks = {}) {
    this.activeTrajectories.set(id, {
      trajectory,
      elapsedTime: 0.0,
      callbacks,
      finished: false
    });
  }

  stopTrajectory(id) {
    this.activeTrajectories.delete(id);
  }

  stopAll() {
    this.activeTrajectories.clear();
  }

  update(delta) {
    if (this.activeTrajectories.size === 0) return;

    for (const [id, runner] of this.activeTrajectories.entries()) {
      runner.elapsedTime += delta;
      const t = runner.elapsedTime;
      const evalData = runner.trajectory.evaluate(t);

      if (runner.callbacks.onUpdate) {
        runner.callbacks.onUpdate(evalData);
      }

      if (evalData.isFinished && !runner.finished) {
        runner.finished = true;
        if (runner.callbacks.onComplete) {
          runner.callbacks.onComplete(evalData);
        }
        this.activeTrajectories.delete(id);
      }
    }
  }

  hasActiveTrajectory(id) {
    return this.activeTrajectories.has(id);
  }
}
```

---

### 4. Bộ Hiển Thị Vệt Quỹ Đạo 3D (`TrajectoryTrail.js`)
Sử dụng mảng định kiểu `Float32Array` cấu trúc Ring Buffer giúp lưu trữ và vẽ 250 điểm quỹ đạo của TCP thời gian thực mà không cấp phát lại bộ nhớ (Zero Garbage Collection).

=== FILE: js/kinematics/TrajectoryTrail.js ===
*(Độ dài: 167 dòng | Kích thước: 5.1 KB)*
```javascript
import * as THREE from 'three';

export class TrajectoryTrail {
  constructor(scene, maxPoints = 250) {
    this.scene = scene;
    this.maxPoints = maxPoints;
    this.visible = false;
    this.armVisible = false;
    this.deltaVisible = false;

    this.group = new THREE.Group();
    this.group.name = "TrajectoryTrailsGroup";
    this.scene.add(this.group);

    // 1. Arm 6-DOF Trail (Cyan 0x06b6d4)
    this.armPositions = new Float32Array(this.maxPoints * 3);
    this.armCount = 0;
    const armGeo = new THREE.BufferGeometry();
    armGeo.setAttribute('position', new THREE.BufferAttribute(this.armPositions, 3));
    this.armMat = new THREE.LineBasicMaterial({ color: 0x06b6d4, linewidth: 2, transparent: true, opacity: 0.85 });
    this.armLine = new THREE.Line(armGeo, this.armMat);
    this.armLine.visible = false;
    this.group.add(this.armLine);

    // 2. Delta Robot Trail (Amber 0xf59e0b)
    this.deltaPositions = new Float32Array(this.maxPoints * 3);
    this.deltaCount = 0;
    const deltaGeo = new THREE.BufferGeometry();
    deltaGeo.setAttribute('position', new THREE.BufferAttribute(this.deltaPositions, 3));
    this.deltaMat = new THREE.LineBasicMaterial({ color: 0xf59e0b, linewidth: 2, transparent: true, opacity: 0.85 });
    this.deltaLine = new THREE.Line(deltaGeo, this.deltaMat);
    this.deltaLine.visible = false;
    this.group.add(this.deltaLine);

    this.lastArmPos = new THREE.Vector3();
    this.lastDeltaPos = new THREE.Vector3();
    this.minDist = 0.005; // 5mm threshold
  }

  addArmPoint(worldPos) {
    if (!this.armVisible || !worldPos) return;
    if (this.armCount > 0 && this.lastArmPos.distanceTo(worldPos) < this.minDist) return;
    this.lastArmPos.copy(worldPos);

    if (this.armCount < this.maxPoints) {
      const idx = this.armCount * 3;
      this.armPositions[idx] = worldPos.x;
      this.armPositions[idx + 1] = worldPos.y;
      this.armPositions[idx + 2] = worldPos.z;
      this.armCount++;
    } else {
      this.armPositions.copyWithin(0, 3, this.maxPoints * 3);
      const lastIdx = (this.maxPoints - 1) * 3;
      this.armPositions[lastIdx] = worldPos.x;
      this.armPositions[lastIdx + 1] = worldPos.y;
      this.armPositions[lastIdx + 2] = worldPos.z;
    }
    this.armLine.geometry.setDrawRange(0, this.armCount);
    this.armLine.geometry.attributes.position.needsUpdate = true;
  }

  addDeltaPoint(worldPos) {
    if (!this.deltaVisible || !worldPos) return;
    if (this.deltaCount > 0 && this.lastDeltaPos.distanceTo(worldPos) < this.minDist) return;
    this.lastDeltaPos.copy(worldPos);

    if (this.deltaCount < this.maxPoints) {
      const idx = this.deltaCount * 3;
      this.deltaPositions[idx] = worldPos.x;
      this.deltaPositions[idx + 1] = worldPos.y;
      this.deltaPositions[idx + 2] = worldPos.z;
      this.deltaCount++;
    } else {
      this.deltaPositions.copyWithin(0, 3, this.maxPoints * 3);
      const lastIdx = (this.maxPoints - 1) * 3;
      this.deltaPositions[lastIdx] = worldPos.x;
      this.deltaPositions[lastIdx + 1] = worldPos.y;
      this.deltaPositions[lastIdx + 2] = worldPos.z;
    }
    this.deltaLine.geometry.setDrawRange(0, this.deltaCount);
    this.deltaLine.geometry.attributes.position.needsUpdate = true;
  }

  setVisible(visible) {
    this.visible = visible;
    this.armVisible = visible;
    this.deltaVisible = visible;
    this.armLine.visible = visible;
    this.deltaLine.visible = visible;
    this.group.visible = visible;
  }
}
```

---

### 5. Engine Cảm Biến, Hàng Đợi Thị Giác & Cơ Chế Gắp Thả Vật Lý (`VacuumAndSensorEngine.js`)
Quản lý:
1. Structured Light Camera Cognex In-Sight: Bắt phôi tại $X = 2.8\text{m}$, tính toán góc xoay yaw đưa vào `visionQueue`.
2. Cảm biến quang `DI_11` (cụm 4 thanh sẵn sàng tại $X = -1.25\text{m}$) và `DI_12` / `DI_13` (thùng carton đỗ/đầy).
3. Cơ chế Scene Graph Re-parenting:
   - Delta hút thanh lẻ trên Băng tải 1, nắn thẳng $yaw = 0^\circ$, đặt ghép cụm lên Băng tải 2.
   - Arm 6-DOF hút nguyên cụm 4 thanh, xoay $180^\circ$ và xếp 2 lớp vào thùng carton trên Băng tải 3.

=== FILE: js/physics/VacuumAndSensorEngine.js ===
*(Độ dài: 364 dòng | Kích thước: 12.1 KB)*
```javascript
import * as THREE from 'three';

export class VacuumAndSensorEngine {
  constructor(scene, factory, deltaRobot, arm6DOF) {
    this.scene = scene;
    this.factory = factory;
    this.deltaRobot = deltaRobot;
    this.arm6DOF = arm6DOF;

    this.visionQueue = [];
    this.visionScanX = 2.8;     // Mặt phẳng laser Cognex
    this.visionWindow = 0.08;   // Hysteresis window
    this.scannedProductIds = new Set();

    this.deltaVacuum = { active: false, attachedObject: null, flowPressureKPa: -85.0 };
    this.armVacuum = { active: false, attachedObject: null, flowPressureKPa: -90.0, currentBoxLayers: 0 };

    this.sensors = { DI_10: true, DI_11: false, DI_12: false, DI_13: false };

    this.boxStackingParams = {
      boxCenterX: -1.25,
      boxCenterZ: -1.45,
      layer1Y: 0.79,  // Tầng 1
      layer2Y: 0.83   // Tầng 2
    };
  }

  update(delta) {
    this.updateVisionScanner();
    this.updateProximitySensors();
  }

  updateVisionScanner() {
    if (!this.factory || !this.factory.infeedProducts) return;
    const products = this.factory.infeedProducts;

    for (let i = 0; i < products.length; i++) {
      const item = products[i];
      if (item.picked) continue;
      const mesh = item.mesh;
      const x = mesh.position.x;

      if (Math.abs(x - this.visionScanX) < this.visionWindow) {
        if (!this.scannedProductIds.has(mesh.uuid)) {
          this.scannedProductIds.add(mesh.uuid);
          const worldPos = new THREE.Vector3();
          mesh.getWorldPosition(worldPos);

          const record = {
            id: mesh.uuid,
            mesh: mesh,
            x: Number(worldPos.x.toFixed(3)),
            y: Number(worldPos.y.toFixed(3)),
            z: Number(worldPos.z.toFixed(3)),
            yawDeg: Number(THREE.MathUtils.radToDeg(mesh.rotation.y).toFixed(1)),
            timestamp: performance.now()
          };
          this.visionQueue.push(record);

          if (this.factory.camStatusLed) {
            this.factory.camStatusLed.material.color.setHex(0x38bdf8);
            setTimeout(() => {
              if (this.factory.camStatusLed) this.factory.camStatusLed.material.color.setHex(0x22c55e);
            }, 120);
          }
        }
      }
    }

    this.visionQueue = this.visionQueue.filter(entry => entry.mesh && entry.mesh.parent && entry.mesh.position.x > -0.5);
  }

  updateProximitySensors() {
    let collationPackDetected = false;
    if (this.factory.collatedPacks) {
      for (const pack of this.factory.collatedPacks) {
        if (!pack.picked && pack.complete !== false && Math.abs(pack.mesh.position.x - (-1.25)) < 0.16) {
          collationPackDetected = true;
          break;
        }
      }
    }
    this.sensors.DI_11 = collationPackDetected;

    let boxAtStopper = false;
    if (this.factory.cartonBoxes) {
      for (const box of this.factory.cartonBoxes) {
        if (!box.full && Math.abs(box.mesh.position.x - (-1.25)) < 0.08) {
          boxAtStopper = true;
          break;
        }
      }
    }
    this.sensors.DI_12 = boxAtStopper;
    this.sensors.DI_13 = (this.armVacuum.currentBoxLayers >= 2);
  }

  attachDeltaGripper(productMesh) {
    if (!productMesh || !this.deltaRobot || !this.deltaRobot.toolFlange) return false;
    if (this.factory && this.factory.infeedProducts) {
      const found = this.factory.infeedProducts.find(p => p.mesh === productMesh);
      if (found) found.picked = true;
    }
    this.deltaVacuum.active = true;
    this.deltaVacuum.attachedObject = productMesh;

    this.deltaRobot.toolFlange.add(productMesh);
    productMesh.position.set(0, -0.115, 0);
    productMesh.rotation.set(0, 0, 0);
    return true;
  }

  releaseDeltaGripper(targetParent = null) {
    if (!this.deltaVacuum.attachedObject) return null;
    const obj = this.deltaVacuum.attachedObject;
    this.deltaVacuum.active = false;
    this.deltaVacuum.attachedObject = null;

    if (this.factory && this.factory.root) {
      this.factory.root.add(obj);
      let maxInfeedX = 3.2;
      for (const p of this.factory.infeedProducts) {
        if (p.mesh !== obj && !p.picked && p.mesh.position.x > maxInfeedX) maxInfeedX = p.mesh.position.x;
      }
      const spacing = this.factory.infeedSpacing || 0.55;
      obj.position.set(Math.max(3.65, maxInfeedX + spacing), 0.88, -0.32 + (Math.random() - 0.5) * 0.14);
      obj.rotation.set(0, (Math.random() - 0.5) * 0.45, 0);

      const found = this.factory.infeedProducts.find(p => p.mesh === obj);
      if (found) found.picked = false;
      this.scannedProductIds.delete(obj.uuid);

      // Đặt 1 thanh vào cụm đang gom trên Băng tải 2
      this.factory.placeSingleBarOnConveyor2(1.20);
    }
    return obj;
  }

  attachArmGripper(packMesh) {
    if (!packMesh || !this.arm6DOF || !this.arm6DOF.tcpMarker) return false;
    if (this.factory && this.factory.collatedPacks) {
      const found = this.factory.collatedPacks.find(p => p.mesh === packMesh);
      if (found) { found.picked = true; found.stopped = true; }
    }
    this.armVacuum.active = true;
    this.armVacuum.attachedObject = packMesh;

    this.arm6DOF.tcpMarker.add(packMesh);
    packMesh.position.set(0.02, 0, 0);
    packMesh.rotation.set(0, 0, Math.PI / 2);
    return true;
  }

  releaseArmGripper(targetParent = null) {
    if (!this.armVacuum.attachedObject) return null;
    const obj = this.armVacuum.attachedObject;
    this.armVacuum.active = false;
    this.armVacuum.attachedObject = null;

    if (this.factory && this.factory.collatedPacks) {
      const idx = this.factory.collatedPacks.findIndex(p => p.mesh === obj);
      if (idx !== -1) this.factory.collatedPacks.splice(idx, 1);
    }

    let boxGroup = targetParent;
    let activeBoxRecord = null;
    if (this.factory && this.factory.cartonBoxes) {
      activeBoxRecord = this.factory.cartonBoxes.find(b => !b.full) || this.factory.cartonBoxes[0];
      if (!boxGroup && activeBoxRecord) boxGroup = activeBoxRecord.mesh;
    }

    const parent = boxGroup || (this.factory ? this.factory.root : this.scene);
    parent.add(obj);

    const layerIdx = this.armVacuum.currentBoxLayers % 2;
    const layerY = (layerIdx === 0) ? 0.032 : 0.072;
    if (boxGroup) {
      obj.position.set(0, layerY, 0);
      obj.rotation.set(0, 0, 0);
    } else {
      obj.position.set(-1.25, 0.75 + layerY, -1.45);
      obj.rotation.set(0, 0, 0);
    }

    this.armVacuum.currentBoxLayers++;

    if (this.armVacuum.currentBoxLayers >= 2) {
      this.sensors.DI_13 = true;
      if (activeBoxRecord) {
        setTimeout(() => {
          activeBoxRecord.full = true;
          this.armVacuum.currentBoxLayers = 0;
          this.sensors.DI_13 = false;
          if (this.factory && this.factory.cartonBoxes.filter(b => !b.full).length < 3) {
            this.factory.spawnEmptyBox(0.15);
          }
        }, 220);
      }
    }
    return obj;
  }

  findReachableInfeedProduct(deltaWorldX = 1.2, reachRadius = 0.45) {
    if (!this.factory || !this.factory.infeedProducts) return null;
    let bestMesh = null;
    let bestDist = Infinity;
    for (const item of this.factory.infeedProducts) {
      if (item.picked) continue;
      const mesh = item.mesh;
      const dist = Math.abs(mesh.position.x - deltaWorldX);
      if (dist < reachRadius && dist < bestDist) {
        bestDist = dist;
        bestMesh = mesh;
      }
    }
    return bestMesh;
  }

  findReachableCollatedPack(armPickupX = -1.25, threshold = 0.25) {
    if (!this.factory || !this.factory.collatedPacks) return null;
    for (const pack of this.factory.collatedPacks) {
      if (pack.picked || pack.complete === false) continue;
      const mesh = pack.mesh;
      if (Math.abs(mesh.position.x - armPickupX) < threshold) return mesh;
    }
    return null;
  }
}
```

---

### 6. Bus Phần Cứng Ảo MP3300 PLC (`MP3300HardwareBus.js`)
Mô phỏng bộ nhớ thanh ghi MPE720 chuẩn Little-Endian và máy trạng thái Ladder logic:
- MB (Bit/Byte), MW (16-bit Signed Word), ML (32-bit Long), MF (32-bit Float), MD (64-bit Double).
- Local I/O Registers `IW00C80` / `IW00C81` (32 Digital Inputs) và `OW00C80` / `OW00C81` (32 Digital Outputs).
- Máy trạng thái điều phối: Chạy biến tần J1000 (`DO_00`), đón tín hiệu `DI_11`, kích robot đóng thùng carton, nhận xung $100\text{ms}$ từ `OT#3` (`DI_12`), kiểm tra bộ đếm thùng $MW00010 \ge 2$ để đẩy thùng ra ngoài (`DO_03`).

=== FILE: js/hardware/MP3300HardwareBus.js ===
*(Độ dài: 434 dòng | Kích thước: 12.3 KB)*
```javascript
export class MP3300HardwareBus {
  constructor(options = {}) {
    this.mBuffer = new ArrayBuffer(65536); // 64 KB Main Data Memory (M-Registers)
    this.mView = new DataView(this.mBuffer);

    this.ioBuffer = new ArrayBuffer(256);   // 256 Bytes I/O Buffer
    this.ioView = new DataView(this.ioBuffer);

    this.gBuffer = new ArrayBuffer(1024);  // 1 KB System G-Registers
    this.gView = new DataView(this.gBuffer);

    this.DI = new Uint8Array(32);
    this.DO = new Uint8Array(32);
    this.robotIN = new Uint8Array(9); // 1-indexed (IN#1..IN#8)
    this.robotOT = new Uint8Array(9); // 1-indexed (OT#1..OT#8)

    this.robotMode = 2; // 0 = TEACH, 1 = PLAY, 2 = REMOTE
    this.emergencyStop = false;
    this.builtinPLCOn = true;
    this.plcState = 'IDLE';
    this.plcBoxClusterCount = 0;

    this.listeners = new Set();
    this.initDefaultRegisters();
  }

  subscribe(fn) {
    this.listeners.add(fn);
    return () => this.listeners.delete(fn);
  }

  notifyChange() {
    for (const fn of this.listeners) fn(this);
  }

  initDefaultRegisters() {
    this.syncDIToIW();
    this.syncDOToOW();
    this.writeMW(400, 1);       // Channel 1
    this.writeMW(401, 16);      // Dev-Typ: Ethernet (218IFD)
    this.writeMW(402, 10001);   // Local Port
    this.writeMW(403, 10040);   // Remote Port
  }

  readMW(wordAddr) {
    return this.mView.getInt16((wordAddr * 2) % 65536, true);
  }

  writeMW(wordAddr, value) {
    this.mView.setInt16((wordAddr * 2) % 65536, value, true);
  }

  readML(wordAddr) {
    return this.mView.getInt32((wordAddr * 2) % 65536, true);
  }

  writeML(wordAddr, value) {
    this.mView.setInt32((wordAddr * 2) % 65536, value, true);
  }

  readMB(byteAddr, bitIndex = 0) {
    const b = this.mView.getUint8(byteAddr % 65536);
    return (b & (1 << (bitIndex % 8))) !== 0 ? 1 : 0;
  }

  writeMB(byteAddr, bitIndex, val) {
    const offset = byteAddr % 65536;
    let b = this.mView.getUint8(offset);
    if (val) b |= (1 << (bitIndex % 8));
    else b &= ~(1 << (bitIndex % 8));
    this.mView.setUint8(offset, b);
  }

  readIW(index = 0) {
    let word = 0;
    const start = index * 16;
    for (let i = 0; i < 16; i++) {
      if (this.DI[start + i]) word |= (1 << i);
    }
    return word;
  }

  readOW(index = 0) {
    let word = 0;
    const start = index * 16;
    for (let i = 0; i < 16; i++) {
      if (this.DO[start + i]) word |= (1 << i);
    }
    return word;
  }

  syncDIToIW() {
    this.ioView.setUint16(0x00, this.readIW(0), true);
    this.ioView.setUint16(0x02, this.readIW(1), true);
  }

  syncDOToOW() {
    this.ioView.setUint16(0x10, this.readOW(0), true);
    this.ioView.setUint16(0x12, this.readOW(1), true);
  }

  setDI(index, value) {
    if (index < 0 || index >= 32) return;
    const val = value ? 1 : 0;
    if (this.DI[index] !== val) {
      this.DI[index] = val;
      this.syncDIToIW();
      if (index >= 20 && index <= 23) {
        this.robotIN[index - 20 + 5] = val;
      }
      this.notifyChange();
    }
  }

  getDI(index) { return this.DI[index] || 0; }

  setDO(index, value) {
    if (index < 0 || index >= 32) return;
    const val = value ? 1 : 0;
    if (this.DO[index] !== val) {
      this.DO[index] = val;
      this.syncDOToOW();
      if (index === 1) this.robotIN[1] = val;
      if (index === 2) this.robotIN[2] = val;
      this.notifyChange();
    }
  }

  getDO(index) { return this.DO[index] || 0; }

  setRobotOT(otNum, value) {
    if (otNum < 1 || otNum > 8) return;
    const val = value ? 1 : 0;
    this.robotOT[otNum] = val;
    if (otNum >= 5 && otNum <= 8) this.setDO(15 + otNum, val);
    if (otNum === 3 && val === 1) this.triggerOT3Pulse();
    this.notifyChange();
  }

  triggerOT3Pulse() {
    this.setDI(12, 1);
    this.setDO(24, 1);
    setTimeout(() => {
      this.setDI(12, 0);
      this.setRobotOT(3, 0);
      setTimeout(() => this.setDO(24, 0), 500);
    }, 100);
  }

  setRobotMode(modeIndex) {
    this.robotMode = modeIndex;
    this.notifyChange();
  }

  getRobotModeName() {
    switch (this.robotMode) {
      case 0: return 'TEACH';
      case 1: return 'PLAY';
      case 2: return 'REMOTE';
      default: return 'UNKNOWN';
    }
  }

  setEmergencyStop(pressed) {
    this.emergencyStop = pressed;
    if (pressed) {
      this.setDO(0, 0);
      this.setDO(3, 0);
      this.setDO(5, 0);
      this.plcState = 'IDLE';
    }
    this.notifyChange();
  }

  updatePLCLogic(delta, context = {}) {
    if (this.emergencyStop) return;

    if (this.getDI(0) === 1 && this.plcState === 'IDLE') {
      this.plcState = 'RUNNING_CONVEYOR';
      this.setDO(0, 1);
    }
    if (this.getDI(1) === 1 && this.plcState !== 'IDLE') {
      this.plcState = 'IDLE';
      this.setDO(0, 0);
      this.setDO(3, 0);
      this.setDO(1, 0);
    }

    if (!this.builtinPLCOn) return;

    switch (this.plcState) {
      case 'RUNNING_CONVEYOR':
        this.setDO(0, 1);
        this.setDO(3, 1);
        if (this.getDI(11) === 1) {
          this.setDO(1, 1);
          this.plcState = 'WAITING_ROBOT_PICK';
          if (context.runRobotPickCycle && !context.isRobotBusy) {
            this.plcState = 'ROBOT_PACKING';
            context.runRobotPickCycle();
          }
        }
        break;

      case 'ROBOT_PACKING':
        if (this.getDI(12) === 1) {
          this.setDO(1, 0);
          this.plcBoxClusterCount++;
          if (this.plcBoxClusterCount >= 2) {
            this.plcBoxClusterCount = 0;
            this.plcState = 'DISCHARGING_BOX';
            this.setDO(3, 1);
          } else {
            this.plcState = 'RUNNING_CONVEYOR';
            this.setDO(0, 1);
          }
        }
        break;

      case 'DISCHARGING_BOX':
        if (this.getDI(13) === 1) {
          this.setDO(3, 0);
          this.setDO(0, 1);
          this.plcState = 'RUNNING_CONVEYOR';
        }
        break;
    }
  }
}
```

---

### 7. Giả Lập Mạng UDP & Server YRC1000micro (`YRC1000EthernetServer.js`)
Mô phỏng giao thức truyền thông High-Speed Ethernet chuẩn YERC:
- Cổng UDP: 10040 (Server) $\leftrightarrow$ 10001 (Client MP3300 218IFD).
- Cấu trúc 32-Byte Header: `Identifier = "YERC" (0x59, 0x45, 0x52, 0x43)`, `Reserve2 = "99999999"`.
- Khóa liên động an toàn (Hardware Safety Interlock): Nếu công tắc Mode Switch ở `TEACH` hoặc `PLAY`, máy chủ từ chối lệnh với mã lỗi `0x1F` (Added Status `0x2030`).
- Khối hàm PLC `MSG_SNDE`: Quản lý cờ Busy (`GB000102`), Complete (`GB000103`), Error (`GB000104`).

=== FILE: js/hardware/YRC1000EthernetServer.js ===
*(Độ dài: 437 dòng | Kích thước: 15.1 KB)*
```javascript
export class YRC1000EthernetServer {
  constructor(hardwareBus, robotArm, options = {}) {
    this.hardwareBus = hardwareBus;
    this.robotArm = robotArm;

    this.config = {
      clientIP: '192.168.1.1',
      clientPort: 10001,
      serverIP: '192.168.1.11',
      serverPort: 10040,
      protocol: 'UDP',
      code: 'BIN'
    };

    this.servoOn = false;
    this.jobRunning = false;
    this.alarmActive = false;
    this.requestCounter = 1;
    this.packetHistory = [];
    this.historyListeners = new Set();
  }

  subscribeHistory(fn) {
    this.historyListeners.add(fn);
    return () => this.historyListeners.delete(fn);
  }

  notifyHistory(item) {
    this.packetHistory.unshift(item);
    if (this.packetHistory.length > 50) this.packetHistory.pop();
    for (const fn of this.historyListeners) fn(item, this.packetHistory);
  }

  buildYERCHeader(dataSize, reqId = 1, ack = 0x00) {
    const buf = new ArrayBuffer(32);
    const view = new DataView(buf);
    const u8 = new Uint8Array(buf);

    u8[0] = 0x59; u8[1] = 0x45; u8[2] = 0x52; u8[3] = 0x43; // "YERC"
    view.setUint16(4, 32, true);        // Header Size
    view.setUint16(6, dataSize, true);  // Data Size
    u8[8] = 0x03;                       // Reserve 1
    u8[9] = 0x01;                       // Packet Type: Robot Control
    u8[10] = ack;                       // 0x00 Request, 0x01 Answer
    u8[11] = reqId & 0xff;
    view.setUint32(12, 0, true);        // Block No
    for (let i = 16; i < 24; i++) u8[i] = 0x39; // "99999999"

    return u8;
  }

  buildRequestPacket(commandNo, instance, attribute, service, dataBytes = []) {
    const payloadSize = 8 + dataBytes.length;
    const header = this.buildYERCHeader(payloadSize, this.requestCounter++, 0x00);
    const fullPacket = new Uint8Array(32 + payloadSize);
    fullPacket.set(header, 0);

    const view = new DataView(fullPacket.buffer);
    view.setUint16(32, commandNo, true);
    view.setUint16(34, instance, true);
    fullPacket[36] = attribute;
    fullPacket[37] = service;

    for (let i = 0; i < dataBytes.length; i++) {
      fullPacket[40 + i] = dataBytes[i];
    }
    return fullPacket;
  }

  processUDPPacket(requestPacket) {
    if (requestPacket.length < 40) return { error: true, message: 'Gói tin không đủ 32-byte Header' };
    const view = new DataView(requestPacket.buffer);

    if (requestPacket[0] !== 0x59 || requestPacket[1] !== 0x45 || requestPacket[2] !== 0x52 || requestPacket[3] !== 0x43) {
      return { error: true, message: 'Sai mã YERC' };
    }

    const reqId = requestPacket[11];
    const commandNo = view.getUint16(32, true);
    const instance = view.getUint16(34, true);
    const attribute = requestPacket[36];
    const service = requestPacket[37];
    let dataVal = (requestPacket.length >= 44) ? view.getUint32(40, true) : 0;

    const currentMode = this.hardwareBus ? this.hardwareBus.robotMode : 2;
    let status = 0x00;
    let addedStatus = 0x0000;
    let answerData = [];
    let logMsg = '';

    if (this.simulatePacketDrop) {
      status = 0x1F;
      addedStatus = 0x4040;
      logMsg = `[MẤT KẾT NỐI] Packet drop mô phỏng (UDP 10040 timeout)`;
    } else {
      const isControl = (commandNo === 0x0083 || commandNo === 0x0086 || commandNo === 0x008A);
      if (isControl && currentMode !== 2) {
        status = 0x1F;
        addedStatus = 0x2030;
        logMsg = `[TỪ CHỐI] Mode Switch đang ở [${this.hardwareBus.getRobotModeName()}], cần gạt sang REMOTE!`;
      } else {
        switch (commandNo) {
          case 0x0083: // Servo ON/OFF
            this.servoOn = (dataVal === 1);
            logMsg = `[SERVO] Đã chuyển sang ${this.servoOn ? 'ON' : 'OFF'}`;
            break;
          case 0x0086: // Start JOB
            if (!this.servoOn) {
              status = 0x1F;
              addedStatus = 0x2040;
              logMsg = `[LỖI] Không thể Start JOB khi Servo OFF`;
            } else {
              this.jobRunning = true;
              logMsg = `[JOB] Đang chạy chương trình tự động`;
            }
            break;
          case 0x0072: // Read Status
            {
              const sBuf = new ArrayBuffer(8);
              const sView = new DataView(sBuf);
              let b0 = (currentMode === 2) ? 0x80 : (currentMode === 1 ? 0x40 : 0x20);
              if (this.jobRunning) b0 |= 0x08;
              sView.setUint8(0, b0);
              sView.setUint8(1, this.servoOn ? 1 : 0);
              answerData = Array.from(new Uint8Array(sBuf));
              logMsg = `[STATUS 0x72] Servo: ${this.servoOn ? 'ON' : 'OFF'}, Mode: ${this.hardwareBus.getRobotModeName()}`;
            }
            break;
          case 0x0075: // Read Position
            {
              const pBuf = new ArrayBuffer(32);
              const pView = new DataView(pBuf);
              pView.setUint32(0, 16, true);
              const tcp = this.robotArm ? this.robotArm.getTCPWorldPosition() : { x: 0, y: 0, z: 0 };
              pView.setInt32(8, Math.round(tcp.x * 1000000), true);
              pView.setInt32(12, Math.round(tcp.y * 1000000), true);
              pView.setInt32(16, Math.round(tcp.z * 1000000), true);
              answerData = Array.from(new Uint8Array(pBuf));
              logMsg = `[POS 0x75] TCP X=${tcp.x.toFixed(3)}, Y=${tcp.y.toFixed(3)}, Z=${tcp.z.toFixed(3)}`;
            }
            break;
        }
      }
    }

    const ansPayloadSize = 8 + answerData.length;
    const ansHeader = this.buildYERCHeader(ansPayloadSize, reqId, 0x01);
    const answerPacket = new Uint8Array(32 + ansPayloadSize);
    answerPacket.set(ansHeader, 0);

    const ansView = new DataView(answerPacket.buffer);
    ansView.setUint16(32, commandNo, true);
    ansView.setUint16(34, instance, true);
    answerPacket[36] = attribute;
    answerPacket[37] = service | 0x80;
    answerPacket[38] = status;
    answerPacket[39] = (addedStatus & 0xff);

    for (let i = 0; i < answerData.length; i++) answerPacket[40 + i] = answerData[i];

    const transaction = {
      timestamp: new Date().toLocaleTimeString(),
      reqId,
      commandNo: `0x${commandNo.toString(16).toUpperCase().padStart(4, '0')}`,
      status: status === 0x00 ? 'SUCCESS' : 'ERROR',
      logMsg,
      isError: status !== 0x00
    };
    this.notifyHistory(transaction);

    return { answerPacket, error: status !== 0x00, status, addedStatus, message: logMsg };
  }

  async sendMSGSNDE(cmdType, dataBytes = []) {
    this.hardwareBus.writeMB(102, 0, 1); // Busy = 1
    this.hardwareBus.writeMB(103, 0, 0); // Complete = 0
    this.hardwareBus.writeMB(104, 0, 0); // Error = 0

    let reqPacket;
    switch (cmdType) {
      case 'SERVO_ON': reqPacket = this.buildRequestPacket(0x0083, 0x0002, 0x01, 0x10, [1, 0, 0, 0]); break;
      case 'SERVO_OFF': reqPacket = this.buildRequestPacket(0x0083, 0x0002, 0x01, 0x10, [2, 0, 0, 0]); break;
      case 'START_JOB': reqPacket = this.buildRequestPacket(0x0086, 0x0001, 0x01, 0x10, [1, 0, 0, 0]); break;
      default: reqPacket = this.buildRequestPacket(0x0072, 0x0001, 0x01, 0x0E, []); break;
    }

    await new Promise(res => setTimeout(res, 20));
    const result = this.processUDPPacket(reqPacket);

    this.hardwareBus.writeMB(102, 0, 0); // Busy = 0
    if (result.error) this.hardwareBus.writeMB(104, 0, 1);
    else this.hardwareBus.writeMB(103, 0, 1);

    return result;
  }
}
```

---

### 8. Bộ Điều Phối Lõi & Máy Trạng Thái Song Song (`main.js`)
Điều phối toàn bộ ứng dụng Digital Twin:
- **`updateAutoDemoCycle(delta)`:** Điều phối độc lập 2 tác vụ song song:
  1. *Delta Worker:* Liên tục phát hiện phôi vào vùng với tới $X \approx 1.20\text{m}$, kích hoạt `runDeltaCollateStep()`.
  2. *6-DOF Arm Worker:* Độc lập kiểm tra cảm biến `DI_11` tại $X = -1.25\text{m}$, khi phát hiện cụm 4 thanh đủ điều kiện thì kích hoạt `runArmBoxPackStep()` gắp đóng thùng carton 2 lớp.
- **`runDeltaCollateStep()`:** Trình tự 6 bước của Robot Delta:
  1. Tiếp cận nhanh phía trên phôi (`MOVL` cao độ $Y = 1.08\text{m}$).
  2. Hạ thẳng đứng bám phôi (`MOVL` cao độ $Y = 0.89\text{m}$).
  3. Kích hoạt giác hút chân không (`attachDeltaGripper`).
  4. Nâng thẳng đứng lên cao độ an toàn ($Y = 1.12\text{m}$).
  5. Chuyển dịch ngang sang Băng tải 2 tới đúng vị trí khe gom ($Z = 0.12..0.24\text{m}$).
  6. Hạ xuống, nhả chân không vào cụm gom (`releaseDeltaGripper`) và nâng lên thu về vị trí chờ.
- **`runArmBoxPackStep()`:** Trình tự 6 bước của Robot Arm 6-DOF:
  1. Di chuyển khớp đồng bộ đến điểm chờ trên trạm gom cụm (`MOVJ` $X = -1.25, Y = 1.16, Z = 0.18$).
  2. Hạ thẳng đứng tiếp xúc mặt cụm 4 thanh (`MOVL` $Y = 0.91\text{m}$).
  3. Kích hoạt 8 giác hút chân không đa vùng (`attachArmGripper`).
  4. Nâng cụm lên cao độ an toàn $Y = 1.24\text{m}$.
  5. Quay thân $180^\circ$ vươn ra sau tới vị trí thùng carton trên Băng tải 3 (`MOVJ` $Z = -1.45\text{m}$).
  6. Hạ thẳng đứng vào khoang thùng carton (Tầng 1 tại $Y = 0.79\text{m}$, Tầng 2 tại $Y = 0.83\text{m}$), ngắt chân không (`releaseArmGripper`), phát xung $100\text{ms}$ `OT#3` kích hoạt `DI_12` trên PLC bus, rút tay lên và quay về vị trí chờ ban đầu.

=== FILE: js/main.js ===
*(Ghi chú độ dài file: 1894 dòng | Kích thước: 76.3 KB | Bao gồm toàn bộ logic điều phối và state machine)*
```javascript
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { FactoryEnvironment } from './scene/FactoryEnvironment.js';
import { DeltaRobot3D } from './robots/DeltaRobot3D.js';
import { ArticulatedArm6DOF } from './robots/ArticulatedArm6DOF.js';
import { Yaskawa6DOFKinematics, YASKAWA_PULSE_RATIOS } from './kinematics/Yaskawa6DOFKinematics.js';
import { Delta4DOFKinematics } from './kinematics/Delta4DOFKinematics.js';
import { TrajectoryPlanner } from './kinematics/TrajectoryPlanner.js';
import { TrajectoryTrail } from './kinematics/TrajectoryTrail.js';
import { VacuumAndSensorEngine } from './physics/VacuumAndSensorEngine.js';
import { MP3300HardwareBus } from './hardware/MP3300HardwareBus.js';
import { YRC1000EthernetServer } from './hardware/YRC1000EthernetServer.js';
import { TeachPendantStation } from './workstations/TeachPendantStation.js';
import { MPE720StudioStation } from './workstations/MPE720StudioStation.js';
import { CSharpPCStudioStation } from './workstations/CSharpPCStudioStation.js';
import { VIPAHMIScreen } from './hmi-scada/VIPAHMIScreen.js';
import { TelemetryAndGraphs } from './hmi-scada/TelemetryAndGraphs.js';
import { FaultInjectionPanel } from './hmi-scada/FaultInjectionPanel.js';

class DigitalTwinApp {
  constructor() {
    this.container = document.getElementById('viewport-container');
    this.canvas = document.getElementById('canvas3d');

    this.systemPower = false;
    this.autoDemoMode = false;
    this.autoDemoTime = 0;
    this.clock = new THREE.Clock();

    this.camTargetPos = new THREE.Vector3();
    this.camTargetLookAt = new THREE.Vector3();
    this.isTransitioningCam = false;

    this.initThree();
    this.initLights();
    this.initSceneObjects();
    this.initKinematicsAndPhysics();
    this.initHardwareBus();
    this.initEngineeringWorkstations();
    this.initCameraPresets();
    this.initUIEventListeners();
    this.initKinematicsLabListeners();
    this.initHardwareUIEventListeners();

    window.addEventListener('resize', () => this.onWindowResize());
    this.animate();
  }

  initThree() {
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x0f172a);
    this.scene.fog = new THREE.FogExp2(0x0f172a, 0.035);

    const aspect = this.container.clientWidth / this.container.clientHeight;
    this.camera = new THREE.PerspectiveCamera(46, aspect, 0.1, 100);
    this.camera.position.set(-0.15, 1.45, 2.75);

    this.renderer = new THREE.WebGLRenderer({
      canvas: this.canvas,
      antialias: true,
      preserveDrawingBuffer: true,
      powerPreference: 'high-performance'
    });
    this.renderer.setSize(this.container.clientWidth, this.container.clientHeight);
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 1.5));
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.05;
    this.controls.target.set(-0.15, 0.95, -0.05);
    this.controls.update();

    this.camTargetPos.copy(this.camera.position);
    this.camTargetLookAt.copy(this.controls.target);
  }

  initLights() {
    const ambient = new THREE.AmbientLight(0xfff7ed, 0.9);
    this.scene.add(ambient);

    this.dirLight = new THREE.DirectionalLight(0xfffaea, 2.4);
    this.dirLight.position.set(3.5, 8.0, 4.5);
    this.dirLight.castShadow = true;
    this.scene.add(this.dirLight);
  }

  initSceneObjects() {
    this.factory = new FactoryEnvironment(this.scene);
    this.deltaRobot = new DeltaRobot3D({ position: new THREE.Vector3(1.2, 2.25, -0.1) });
    this.scene.add(this.deltaRobot.root);

    this.arm6DOF = new ArticulatedArm6DOF({ position: new THREE.Vector3(-1.25, 0, -0.75) });
    this.scene.add(this.arm6DOF.root);
  }

  initKinematicsAndPhysics() {
    this.armKinematics = new Yaskawa6DOFKinematics({ worldBasePosition: this.arm6DOF.basePosition });
    this.deltaKinematics = new Delta4DOFKinematics({ worldBasePosition: this.deltaRobot.basePosition });
    this.trajectoryPlanner = new TrajectoryPlanner();
    this.trajectoryTrail = new TrajectoryTrail(this.scene);
    this.physicsEngine = new VacuumAndSensorEngine(this.scene, this.factory, this.deltaRobot, this.arm6DOF);

    this.isSingleCycleRunning = false;
    this.isDeltaBusy = false;
    this.isArmBusy = false;
  }

  initHardwareBus() {
    this.hardwareBus = new MP3300HardwareBus();
    this.ethernetServer = new YRC1000EthernetServer(this.hardwareBus, this.arm6DOF);
  }

  // ===========================================================================
  // PARALLEL SCHEDULER & STEP SEQUENCERS
  // ===========================================================================

  updateAutoDemoCycle(delta) {
    if (!this.autoDemoMode || !this.systemPower) return;

    // 1. Robot Delta 4-DOF: Tự động gắp khi phôi vào vùng với tới (X ~ 1.20m)
    if (!this.isDeltaBusy) {
      const reachableBar = this.physicsEngine.findReachableInfeedProduct(1.2, 0.42);
      if (reachableBar) {
        this.runDeltaCollateStep(reachableBar);
      }
    }

    // 2. Robot Arm 6-DOF: Tự động gắp khi cụm 4 thanh đủ điều kiện tại DI_11 (X = -1.25m)
    if (!this.isArmBusy) {
      const readyPack = this.physicsEngine.findReachableCollatedPack(-1.25, 0.22);
      if (readyPack) {
        this.runArmBoxPackStep(readyPack);
      }
    }
  }

  async runDeltaCollateStep(infeedProd = null) {
    if (this.isDeltaBusy) return;
    this.isDeltaBusy = true;
    this.isSingleCycleRunning = true;

    const runTraj = (id, traj, onUpdateJoints) => new Promise(resolve => {
      this.trajectoryPlanner.startTrajectory(id, traj, {
        onUpdate: onUpdateJoints,
        onComplete: resolve
      });
    });
    const sleep = (ms) => new Promise(res => setTimeout(res, ms));

    try {
      if (!infeedProd) infeedProd = this.physicsEngine.findReachableInfeedProduct(1.2, 0.72);
      if (!infeedProd) {
        infeedProd = this.factory.spawnProduct(1.25, (Math.random() - 0.5) * 0.12, 15);
        await sleep(10);
      }

      let buildingPack = this.factory.collatedPacks.find(p => !p.complete && !p.picked);
      const currentBarsInSlot = buildingPack ? buildingPack.barCount : 0;

      const prodWorldPos = new THREE.Vector3();
      infeedProd.getWorldPosition(prodWorldPos);

      // Bước 1: Tiếp cận nhanh phía trên phôi (Y = 1.08m)
      const deltaCur = this.deltaRobot.getEndEffectorWorldPosition();
      const prePick = new THREE.Vector3(prodWorldPos.x, 1.08, prodWorldPos.z);
      let dTraj = this.trajectoryPlanner.createMOVL(deltaCur, prePick, this.deltaKinematics, { duration: 0.10, isWorld: true });
      await runTraj('delta-cycle', dTraj, (data) => {
        if (data.ikResult?.success) {
          const a = data.ikResult.anglesDeg;
          this.deltaRobot.setJointAngles(a.m1, a.m2, a.m3, a.yaw);
        }
      });

      // Bước 2: Hạ thẳng đứng kẹp hút (Y = 0.89m)
      infeedProd.getWorldPosition(prodWorldPos);
      const pickPos = new THREE.Vector3(prodWorldPos.x, 0.89, prodWorldPos.z);
      dTraj = this.trajectoryPlanner.createMOVL(prePick, pickPos, this.deltaKinematics, { duration: 0.07, isWorld: true });
      await runTraj('delta-cycle', dTraj, (data) => {
        if (data.ikResult?.success) {
          const a = data.ikResult.anglesDeg;
          this.deltaRobot.setJointAngles(a.m1, a.m2, a.m3, a.yaw);
        }
      });

      this.physicsEngine.attachDeltaGripper(infeedProd);
      await sleep(10);

      // Bước 3: Nâng bốc phôi lên cao độ an toàn (Y = 1.12m)
      const liftPos = new THREE.Vector3(pickPos.x, 1.12, pickPos.z);
      dTraj = this.trajectoryPlanner.createMOVL(pickPos, liftPos, this.deltaKinematics, { duration: 0.07, isWorld: true });
      await runTraj('delta-cycle', dTraj, (data) => {
        if (data.ikResult?.success) {
          const a = data.ikResult.anglesDeg;
          this.deltaRobot.setJointAngles(a.m1, a.m2, a.m3, a.yaw);
        }
      });

      // Bước 4: Di chuyển sang đúng khe gom cụm trên Băng tải 2
      const targetSlotZ = 0.18 + (-0.06 + currentBarsInSlot * 0.04);
      const dropOverhead = new THREE.Vector3(1.20, 1.12, targetSlotZ);
      dTraj = this.trajectoryPlanner.createMOVL(liftPos, dropOverhead, this.deltaKinematics, { duration: 0.11, isWorld: true });
      await runTraj('delta-cycle', dTraj, (data) => {
        if (data.ikResult?.success) {
          const a = data.ikResult.anglesDeg;
          this.deltaRobot.setJointAngles(a.m1, a.m2, a.m3, a.yaw);
        }
      });

      // Bước 5: Hạ xuống thả phôi
      const dropPos = new THREE.Vector3(1.20, 0.89, targetSlotZ);
      dTraj = this.trajectoryPlanner.createMOVL(dropOverhead, dropPos, this.deltaKinematics, { duration: 0.07, isWorld: true });
      await runTraj('delta-cycle', dTraj, (data) => {
        if (data.ikResult?.success) {
          const a = data.ikResult.anglesDeg;
          this.deltaRobot.setJointAngles(a.m1, a.m2, a.m3, a.yaw);
        }
      });

      this.physicsEngine.releaseDeltaGripper();
      await sleep(10);

      // Bước 6: Rút lên nhanh chóng sẵn sàng đón phôi tiếp theo
      const postDropLift = new THREE.Vector3(1.20, 1.15, 0.0);
      dTraj = this.trajectoryPlanner.createMOVL(dropPos, postDropLift, this.deltaKinematics, { duration: 0.08, isWorld: true });
      await runTraj('delta-cycle', dTraj, (data) => {
        if (data.ikResult?.success) {
          const a = data.ikResult.anglesDeg;
          this.deltaRobot.setJointAngles(a.m1, a.m2, a.m3, a.yaw);
        }
      });
    } catch (err) {
      console.error("Lỗi trong chu trình Delta:", err);
    } finally {
      this.isDeltaBusy = false;
      this.isSingleCycleRunning = this.isDeltaBusy || this.isArmBusy;
    }
  }

  async runArmBoxPackStep(collatedPack = null) {
    if (this.isArmBusy) return;
    this.isArmBusy = true;
    this.isSingleCycleRunning = true;

    const runTraj = (id, traj, onUpdateJoints) => new Promise(resolve => {
      this.trajectoryPlanner.startTrajectory(id, traj, {
        onUpdate: onUpdateJoints,
        onComplete: resolve
      });
    });
    const sleep = (ms) => new Promise(res => setTimeout(res, ms));

    try {
      if (!collatedPack) collatedPack = this.physicsEngine.findReachableCollatedPack(-1.25, 0.35);
      if (!collatedPack) {
        collatedPack = this.factory.spawnCollatedPack(-1.25, 4);
        await sleep(15);
      }

      // Bước 1: Di chuyển tới phía trên trạm đón cụm DI_11
      const armPickPre = { x: -1.25, y: 1.16, z: 0.18, rx: 0, ry: 0, rz: 0 };
      const ikPickPre = this.armKinematics.inverseKinematics(armPickPre, this.arm6DOF.getJointAngles(), true);
      if (ikPickPre.success) {
        const armTraj = this.trajectoryPlanner.createMOVJ(this.arm6DOF.getJointAngles(), ikPickPre.jointsDeg, { duration: 0.28, degree: 5 });
        await runTraj('arm-cycle', armTraj, (data) => {
          const q = data.joints;
          this.arm6DOF.setJointAngles(q.s, q.l, q.u, q.r, q.b, q.t);
        });
      }

      // Bước 2: Hạ thẳng đứng hút trọn cụm 4 thanh (Y = 0.91m)
      const armPickPose = { x: -1.25, y: 0.91, z: 0.18, rx: 0, ry: 0, rz: 0 };
      const armPickTraj = this.trajectoryPlanner.createMOVL(armPickPre, armPickPose, this.armKinematics, { duration: 0.18, degree: 5, isWorld: true });
      await runTraj('arm-cycle', armPickTraj, (data) => {
        if (data.ikResult?.success) {
          const q = data.ikResult.jointsDeg;
          this.arm6DOF.setJointAngles(q.s, q.l, q.u, q.r, q.b, q.t);
        }
      });

      this.physicsEngine.attachArmGripper(collatedPack);
      await sleep(25);

      // Bước 3: Nâng cụm lên thẳng đứng (Y = 1.24m)
      const armLiftPose = { x: -1.25, y: 1.24, z: 0.18, rx: 0, ry: 0, rz: 0 };
      const armLiftTraj = this.trajectoryPlanner.createMOVL(armPickPose, armLiftPose, this.armKinematics, { duration: 0.18, degree: 5, isWorld: true });
      await runTraj('arm-cycle', armLiftTraj, (data) => {
        if (data.ikResult?.success) {
          const q = data.ikResult.jointsDeg;
          this.arm6DOF.setJointAngles(q.s, q.l, q.u, q.r, q.b, q.t);
        }
      });

      // Bước 4: Quay thân 180° ra sau tới thùng carton trên Băng tải 3 (Z = -1.45m)
      const boxPrePose = { x: -1.25, y: 1.20, z: -1.45, rx: 0, ry: 0, rz: 0 };
      const ikBoxPre = this.armKinematics.inverseKinematics(boxPrePose, this.arm6DOF.getJointAngles(), true);
      if (ikBoxPre.success) {
        const armSwingTraj = this.trajectoryPlanner.createMOVJ(this.arm6DOF.getJointAngles(), ikBoxPre.jointsDeg, { duration: 0.34, degree: 5 });
        await runTraj('arm-cycle', armSwingTraj, (data) => {
          const q = data.joints;
          this.arm6DOF.setJointAngles(q.s, q.l, q.u, q.r, q.b, q.t);
        });
      }

      // Bước 5: Hạ thẳng đứng đặt vào thùng (Tầng 1: Y = 0.79m, Tầng 2: Y = 0.83m)
      const currentLayer = (this.physicsEngine.armVacuum.currentBoxLayers % 2) + 1;
      const placeY = (currentLayer === 1) ? 0.79 : 0.83;
      const boxPlacePose = { x: -1.25, y: placeY, z: -1.45, rx: 0, ry: 0, rz: 0 };
      const armPlaceTraj = this.trajectoryPlanner.createMOVL(boxPrePose, boxPlacePose, this.armKinematics, { duration: 0.20, degree: 5, isWorld: true });
      await runTraj('arm-cycle', armPlaceTraj, (data) => {
        if (data.ikResult?.success) {
          const q = data.ikResult.jointsDeg;
          this.arm6DOF.setJointAngles(q.s, q.l, q.u, q.r, q.b, q.t);
        }
      });

      this.physicsEngine.releaseArmGripper();
      await sleep(25);

      if (this.hardwareBus) {
        this.hardwareBus.triggerOT3Pulse(); // Gửi xung báo đã thả cụm vào thùng
      }

      // Bước 6: Rút tay lên và quay về vị trí chờ trên trạm DI_11
      const armRetractTraj = this.trajectoryPlanner.createMOVL(boxPlacePose, boxPrePose, this.armKinematics, { duration: 0.18, degree: 5, isWorld: true });
      await runTraj('arm-cycle', armRetractTraj, (data) => {
        if (data.ikResult?.success) {
          const q = data.ikResult.jointsDeg;
          this.arm6DOF.setJointAngles(q.s, q.l, q.u, q.r, q.b, q.t);
        }
      });
    } catch (err) {
      console.error("Lỗi trong chu trình Arm:", err);
    } finally {
      this.isArmBusy = false;
      this.isSingleCycleRunning = this.isDeltaBusy || this.isArmBusy;
    }
  }

  // Vòng lặp Render 60 FPS
  animate() {
    requestAnimationFrame(() => this.animate());
    const delta = Math.min(this.clock.getDelta(), 0.05);

    if (this.controls) this.controls.update();

    if (this.trajectoryPlanner) this.trajectoryPlanner.update(delta);
    if (this.physicsEngine) this.physicsEngine.update(delta);

    if (this.systemPower) {
      this.updateAutoDemoCycle(delta);
      if (this.hardwareBus) {
        this.hardwareBus.updatePLCLogic(delta, {
          runRobotPickCycle: () => this.runArmBoxPackStep(),
          isRobotBusy: this.isArmBusy
        });
      }
    }

    if (this.trajectoryTrail) {
      this.trajectoryTrail.addArmPoint(this.arm6DOF.getTCPWorldPosition());
      this.trajectoryTrail.addDeltaPoint(this.deltaRobot.getEndEffectorWorldPosition());
    }

    this.renderer.render(this.scene, this.camera);
  }
}

window.addEventListener('DOMContentLoaded', () => {
  window.digitalTwin = new DigitalTwinApp();
});
```
