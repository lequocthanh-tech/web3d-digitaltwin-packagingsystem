/**
 * Delta3DOFKinematics.js
 * 
 * High-Precision Analytical Kinematics Engine for 3-DOF Parallel Delta Robot
 * Benchmark Mechanical Specs:
 * - Upper Fixed Base Radius: R_base = 180 mm (3 motors at 0°, 120°, 240°)
 * - Upper Arm (Biceps / Active Arm): L1 = 200 mm
 * - Lower Arm (Forearms / 3 Parallelograms): L2 = 500 mm
 * - Mobile End-Effector Platform Radius: r_platform = 50 mm
 * - Workspace Limits: Z in [-600 mm, -350 mm], R_max = 250 mm (sqrt(X^2 + Y^2) <= 250 mm)
 */

export class Delta3DOFKinematics {
  constructor(options = {}) {
    // Mechanical dimensions in millimeters (mm)
    this.R_base = options.R_base ?? 180.0;         // Radius of upper base (mm)
    this.r_platform = options.r_platform ?? 50.0;   // Radius of mobile platform (mm)
    this.L1 = options.L1 ?? 200.0;                 // Length of active upper arm (mm)
    this.L2 = options.L2 ?? 500.0;                 // Length of passive lower parallelogram rods (mm)
    this.r_diff = this.R_base - this.r_platform;   // Effective offset rd = 130 mm

    // Servo angle physical limits in degrees
    this.thetaMin = options.thetaMin ?? -20.0;
    this.thetaMax = options.thetaMax ?? 90.0;

    // Workspace envelope in mm (zMin = -445.0 mm prevents collision with conveyor belt bed at -445.5 mm)
    this.zMin = options.zMin ?? -445.0; // Lowest Z reach
    this.zMax = options.zMax ?? -340.0; // Highest Z reach
    this.rMax = options.rMax ?? 250.0;  // Maximum radial displacement

    // 3 Motor angular positions in base plane
    this.phi = [
      0.0,
      (2.0 * Math.PI) / 3.0, // 120 deg
      (4.0 * Math.PI) / 3.0  // 240 deg
    ];

    // Conversion factor to meters for Three.js
    this.toMeters = 0.001;
    this.toMillimeters = 1000.0;
  }

  /**
   * Validates if target (X, Y, Z) is strictly inside the specified workspace
   * @param {number} x in mm
   * @param {number} y in mm
   * @param {number} z in mm
   * @returns {{ valid: boolean, reason: string }}
   */
  checkWorkspace(x, y, z) {
    const r = Math.sqrt(x * x + y * y);
    if (z < this.zMin || z > this.zMax) {
      return {
        valid: false,
        reason: `Z = ${z.toFixed(1)} mm nằm ngoài giới hạn [${this.zMin}, ${this.zMax}] mm`
      };
    }
    if (r > this.rMax) {
      return {
        valid: false,
        reason: `Bán kính R = ${r.toFixed(1)} mm vượt quá bán kính tối đa R_max = ${this.rMax} mm`
      };
    }
    return { valid: true, reason: 'Trong vùng làm việc an toàn (Workspace OK)' };
  }

  /**
   * Inverse Kinematics (IK) for 3-DOF Delta Robot
   * Uses analytical circle-plane intersection (Sphere of forearm with upper arm rotation plane)
   * 
   * @param {number} x - Target X coordinate of end-effector (mm)
   * @param {number} y - Target Y coordinate of end-effector (mm)
   * @param {number} z - Target Z coordinate of end-effector (mm, negative below base)
   * @returns {{ success: boolean, inLimits: boolean, thetaDeg: { theta1: number, theta2: number, theta3: number }, thetaRad: number[], message: string }}
   */
  inverseKinematics(x, y, z) {
    const ws = this.checkWorkspace(x, y, z);
    const theta = [0, 0, 0];
    const L1 = this.L1;
    const L2 = this.L2;
    const rd = this.r_diff;

    for (let i = 0; i < 3; i++) {
      const phi_i = this.phi[i];

      // Rotate coordinates into the vertical 2D plane of arm i
      const xp = x * Math.cos(phi_i) + y * Math.sin(phi_i);
      const yp = -x * Math.sin(phi_i) + y * Math.cos(phi_i);
      const zp = z;

      // Delta X in arm plane relative to rd
      const deltaX = xp - rd;

      // Equation: a * cos(theta) + b * sin(theta) = c
      const a = 2.0 * L1 * deltaX;
      const b = -2.0 * L1 * zp;
      const c = deltaX * deltaX + yp * yp + zp * zp + L1 * L1 - L2 * L2;

      // Half-angle substitution t = tan(theta / 2):
      // (c + a)*t^2 - 2*b*t + (c - a) = 0
      const A_quad = c + a;
      const B_quad = -2.0 * b;
      const C_quad = c - a;

      const D = a * a + b * b - c * c;

      if (D < 0) {
        return {
          success: false,
          inLimits: false,
          thetaDeg: { theta1: 0, theta2: 0, theta3: 0 },
          thetaRad: [0, 0, 0],
          workspaceCheck: ws,
          message: `Vị trí (${x.toFixed(1)}, ${y.toFixed(1)}, ${z.toFixed(1)}) ngoài tầm với của trục ${i + 1} (Không có nghiệm hình học D < 0)`
        };
      }

      // Physical outward elbow configuration: t = (b - sqrt(D)) / (c + a)
      const sqrtD = Math.sqrt(D);
      let t;
      if (Math.abs(A_quad) < 1e-6) {
        t = -C_quad / B_quad;
      } else {
        t = (b - sqrtD) / A_quad;
      }

      theta[i] = 2.0 * Math.atan(t);
    }

    const t1Deg = (theta[0] * 180.0) / Math.PI;
    const t2Deg = (theta[1] * 180.0) / Math.PI;
    const t3Deg = (theta[2] * 180.0) / Math.PI;

    const inLimits = [t1Deg, t2Deg, t3Deg].every(
      th => th >= this.thetaMin && th <= this.thetaMax
    ) && ws.valid;

    return {
      success: true,
      inLimits,
      thetaDeg: {
        theta1: Number(t1Deg.toFixed(2)),
        theta2: Number(t2Deg.toFixed(2)),
        theta3: Number(t3Deg.toFixed(2))
      },
      thetaRad: [theta[0], theta[1], theta[2]],
      workspaceCheck: ws,
      message: inLimits ? 'Giải IK thành công (Hợp lệ)' : (ws.valid ? 'Góc quay vượt giới hạn động cơ' : ws.reason)
    };
  }

  /**
   * Forward Kinematics (FK) for 3-DOF Delta Robot
   * Trilateration of 3 spheres centered at virtual elbow points
   * 
   * @param {number} t1Deg - Angle of motor 1 (degrees)
   * @param {number} t2Deg - Angle of motor 2 (degrees)
   * @param {number} t3Deg - Angle of motor 3 (degrees)
   * @returns {{ success: boolean, x: number, y: number, z: number, message: string }}
   */
  forwardKinematics(t1Deg, t2Deg, t3Deg) {
    const th = [
      (t1Deg * Math.PI) / 180.0,
      (t2Deg * Math.PI) / 180.0,
      (t3Deg * Math.PI) / 180.0
    ];

    const rd = this.r_diff;
    const L1 = this.L1;
    const L2 = this.L2;

    // Virtual sphere centers C_i (elbow points offset by r_platform)
    const C = [];
    for (let i = 0; i < 3; i++) {
      const phi_i = this.phi[i];
      const cx = (rd + L1 * Math.cos(th[i])) * Math.cos(phi_i);
      const cy = (rd + L1 * Math.cos(th[i])) * Math.sin(phi_i);
      const cz = -L1 * Math.sin(th[i]);
      C.push({ x: cx, y: cy, z: cz });
    }

    const p1 = C[0];
    const p2 = C[1];
    const p3 = C[2];

    // Trilateration algorithm
    // Vector p2 - p1
    const d12 = { x: p2.x - p1.x, y: p2.y - p1.y, z: p2.z - p1.z };
    const dist12 = Math.hypot(d12.x, d12.y, d12.z);
    if (dist12 < 1e-6) return { success: false, x: 0, y: 0, z: 0, message: 'Lỗi cấu hình kỳ dị' };

    const ex = { x: d12.x / dist12, y: d12.y / dist12, z: d12.z / dist12 };

    // Vector p3 - p1
    const d13 = { x: p3.x - p1.x, y: p3.y - p1.y, z: p3.z - p1.z };
    const i_val = ex.x * d13.x + ex.y * d13.y + ex.z * d13.z;

    const ey_unnorm = {
      x: d13.x - i_val * ex.x,
      y: d13.y - i_val * ex.y,
      z: d13.z - i_val * ex.z
    };
    const dist_ey = Math.hypot(ey_unnorm.x, ey_unnorm.y, ey_unnorm.z);
    if (dist_ey < 1e-6) return { success: false, x: 0, y: 0, z: 0, message: '3 tâm khớp thẳng hàng' };

    const ey = { x: ey_unnorm.x / dist_ey, y: ey_unnorm.y / dist_ey, z: ey_unnorm.z / dist_ey };
    const ez = {
      x: ex.y * ey.z - ex.z * ey.y,
      y: ex.z * ey.x - ex.x * ey.z,
      z: ex.x * ey.y - ex.y * ey.x
    };

    const d = dist12;
    const j_val = ey.x * d13.x + ey.y * d13.y + ey.z * d13.z;

    const x_coord = (d * d) / (2.0 * d);
    const y_coord = (i_val * i_val + j_val * j_val) / (2.0 * j_val) - (i_val / j_val) * x_coord;
    const z_sq = L2 * L2 - x_coord * x_coord - y_coord * y_coord;

    if (z_sq < 0) {
      return { success: false, x: 0, y: 0, z: 0, message: '3 mặt cầu không giao nhau (ngoài không gian làm việc)' };
    }

    const z_val = Math.sqrt(z_sq);

    // Pick lower branch (Z is negative downward)
    const solA = {
      x: p1.x + ex.x * x_coord + ey.x * y_coord + ez.x * z_val,
      y: p1.y + ex.y * x_coord + ey.y * y_coord + ez.y * z_val,
      z: p1.z + ex.z * x_coord + ey.z * y_coord + ez.z * z_val
    };
    const solB = {
      x: p1.x + ex.x * x_coord + ey.x * y_coord - ez.x * z_val,
      y: p1.y + ex.y * x_coord + ey.y * y_coord - ez.y * z_val,
      z: p1.z + ex.z * x_coord + ey.z * y_coord - ez.z * z_val
    };

    const chosen = (solA.z < solB.z) ? solA : solB;

    return {
      success: true,
      x: Number(chosen.x.toFixed(2)),
      y: Number(chosen.y.toFixed(2)),
      z: Number(chosen.z.toFixed(2)),
      message: 'Giải FK thành công'
    };
  }

  /**
   * Generates a 3D Gate Trajectory (Inverted-U / Pi-Curve) for High-Speed Pick & Place
   * 
   * @param {Object} pick - { x, y, z } in mm
   * @param {Object} place - { x, y, z } in mm
   * @param {number} zSafe - Safe height for horizontal transit (mm, e.g. -380 mm)
   * @param {number} totalSteps - Number of discrete trajectory steps (e.g. 60)
   * @returns {Array<Object>} List of trajectory points with stage annotations
   */
  generateGateTrajectory(pick, place, zSafe = -380.0, totalSteps = 60) {
    const path = [];
    const stepCount = Math.max(20, totalSteps);

    // Timing distribution:
    // 0.0 - 0.20: Lift from Pick to zSafe
    // 0.20 - 0.70: Horizontal transit from Pick (X,Y) to Place (X,Y) at zSafe
    // 0.70 - 0.90: Lower from zSafe to Place
    // 0.90 - 1.00: Dwell at Place

    for (let i = 0; i <= stepCount; i++) {
      const u = i / stepCount;
      let px, py, pz;
      let stage = '';
      let vacuumCmd = false;

      if (u <= 0.20) {
        // Stage 1: Vertical Lift from Pick to zSafe
        const s = u / 0.20;
        // Quintic smoothstep s^3 * (10 - 15s + 6s^2)
        const ease = s * s * s * (10 - 15 * s + 6 * s * s);
        px = pick.x;
        py = pick.y;
        pz = pick.z + (zSafe - pick.z) * ease;
        stage = '1_LIFT_PICK';
        vacuumCmd = true;
      } else if (u <= 0.70) {
        // Stage 2: Fast Horizontal Transit above workpiece
        const s = (u - 0.20) / 0.50;
        const ease = s * s * s * (10 - 15 * s + 6 * s * s);
        px = pick.x + (place.x - pick.x) * ease;
        py = pick.y + (place.y - pick.y) * ease;
        // Parabolic arch apex in the middle of flight for aesthetic high-speed clearing
        const arch = 15.0 * Math.sin(Math.PI * s);
        pz = zSafe + arch;
        stage = '2_FLY_HORIZONTAL';
        vacuumCmd = true;
      } else if (u <= 0.90) {
        // Stage 3: Vertical Lower to Place
        const s = (u - 0.70) / 0.20;
        const ease = s * s * s * (10 - 15 * s + 6 * s * s);
        px = place.x;
        py = place.y;
        pz = zSafe + (place.z - zSafe) * ease;
        stage = '3_LOWER_PLACE';
        vacuumCmd = (u < 0.88); // Turn off just as touching place
      } else {
        // Stage 4: Dwell at Place point
        px = place.x;
        py = place.y;
        pz = place.z;
        stage = '4_DWELL_PLACE';
        vacuumCmd = false;
      }

      const ik = this.inverseKinematics(px, py, pz);
      path.push({
        progress: u,
        x: px,
        y: py,
        z: pz,
        stage,
        vacuum: vacuumCmd,
        ik
      });
    }

    return path;
  }
}
