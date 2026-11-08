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
 *   Thus negative S (-90° to -135°) rotates the arm forward (+Z) toward Conveyor 2 (Z = +0.18m) and Carton Box (Z = +0.25m).
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
    // Dimensions in millimeters (mm) matching ArticulatedArm6DOF.js
    this.dh = {
      d1: options.d1 ?? 960.0,   // Base to Shoulder pivot (mm)
      a1: options.a1 ?? 0.0,     // Shoulder radial offset (mm)
      a2: options.a2 ?? 740.0,   // Lower Arm length (mm)
      d4: options.d4 ?? 840.0,   // Elbow to Wrist center W (mm)
      d6: options.d6 ?? 275.0    // Wrist center W to Tool Center Point (TCP) (mm)
    };

    // Base position in 3D Scene world coordinates (meters)
    this.worldBasePosition = options.worldBasePosition || new THREE.Vector3(-1.25, 0, -0.75);
  }

  /**
   * Convert Joint Angles (Degrees) to Yaskawa Encoder Pulses
   */
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

  /**
   * Convert Yaskawa Encoder Pulses to Joint Angles (Degrees)
   */
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

  /**
   * Compute standard Denavit-Hartenberg transformation matrix A_i
   */
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

  /**
   * Forward Kinematics (FK) matching exact Three.js 3D hierarchy of ArticulatedArm6DOF
   * @param {Object} jointsDeg - { s, l, u, r, b, t } in degrees
   * @returns {Object} FK Solution containing T0_6, A1..A6, position_mm, position_m, worldPosition_m, tcp_m, orientation_deg, formattedMatrix
   */
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

    // Individual DH link matrices for educational table & inspection
    const A1 = this.computeDHMatrix(a1, Math.PI / 2, d1, q.s);
    const A2 = this.computeDHMatrix(a2, 0, 0, q.l + Math.PI / 2);
    const A3 = this.computeDHMatrix(0, Math.PI / 2, 0, q.u - Math.PI / 2);
    const A4 = this.computeDHMatrix(0, -Math.PI / 2, d4, q.r);
    const A5 = this.computeDHMatrix(0, Math.PI / 2, 0, q.b);
    const A6 = this.computeDHMatrix(0, 0, d6, q.t);

    // Exact 3D Planar reach (r_tcp) and vertical height (y_tcp) in Robot Base Frame (mm)
    // Lower arm (a2) points straight UP (+Y) when l = 0:
    //   elbow_r = -a2 * sin(l), elbow_y = d1 + a2 * cos(l)
    // Forearm (d4) points along +X when l + u = 0:
    //   wrist_r = elbow_r + d4 * cos(l + u), wrist_y = elbow_y + d4 * sin(l + u)
    // Tool (d6) points along +X when l + u + b = 0, and straight DOWN (-Y) when l + u + b = -90 deg:
    const lu = q.l + q.u;
    const psi = q.l + q.u + q.b;

    const r_tcp = -a2 * Math.sin(q.l) + d4 * Math.cos(lu) + d6 * Math.cos(psi);
    const y_tcp = d1 + a2 * Math.cos(q.l) + d4 * Math.sin(lu) + d6 * Math.sin(psi);

    // Rotate around Three.js +Y axis by angle q.s:
    // local X = r_tcp * cos(s), local Z = -r_tcp * sin(s)
    const posX = r_tcp * Math.cos(q.s);
    const posY = y_tcp;
    const posZ = -r_tcp * Math.sin(q.s);

    // Build exact homogeneous transformation matrix T0_6 in Three.js Y-up frame
    const rotY = new THREE.Matrix4().makeRotationY(q.s);
    const rotZ = new THREE.Matrix4().makeRotationZ(psi);
    const rotX = new THREE.Matrix4().makeRotationX(q.r + q.t);
    const T0_6 = new THREE.Matrix4().multiplyMatrices(rotY, rotZ).multiply(rotX);
    T0_6.setPosition(posX, posY, posZ);

    const elements = T0_6.elements;

    // World TCP position in meters
    const worldPos = new THREE.Vector3(
      this.worldBasePosition.x + posX / 1000.0,
      this.worldBasePosition.y + posY / 1000.0,
      this.worldBasePosition.z + posZ / 1000.0
    );

    // Orientation angles relative to downward-facing tool (rx = pitch offset from -90 deg, ry = roll, rz = yaw)
    const pitchOffsetDeg = THREE.MathUtils.radToDeg(psi) + 90.0;
    const rollDeg = rDeg;
    const yawDeg = tDeg;

    // Formatted 4x4 array for UI table display (row-major)
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
      tcp_m: worldPos, // Alias used by TeachPendantStation & CSharpPCStudioStation
      orientation_deg: {
        rx: Number(pitchOffsetDeg.toFixed(2)),
        ry: Number(rollDeg.toFixed(2)),
        rz: Number(yawDeg.toFixed(2))
      },
      formattedMatrix
    };
  }

  /**
   * Analytical Inverse Kinematics (IK) for 6-DOF Spherical Wrist in Three.js Y-up Coordinates
   * Guarantees the arm reaches forward over the conveyor table (Elbow-Up industrial posture)
   * and keeps the multi-vacuum gripper level with the conveyor/carton box.
   * 
   * @param {Object} targetPose - { x, y, z, rx, ry, rz }
   * @param {Object} [currentJoints] - Reference joint angles
   * @param {boolean} [isWorldCoords=false] - Whether x, y, z are world coordinates in meters
   * @returns {Object} { success: boolean, jointsDeg: { s, l, u, r, b, t }, message: string }
   */
  inverseKinematics(targetPose, currentJoints = { s: -75, l: -18, u: -10, r: 0, b: -62, t: 0 }, isWorldCoords = false) {
    let wx_world_m = targetPose.x ?? -1.25;
    let wy_world_m = targetPose.y ?? 1.20;
    let wz_world_m = targetPose.z ?? 0.18;

    if (!isWorldCoords) {
      // Check if passed in millimeters or meters
      if (Math.abs(wx_world_m) > 10.0 || Math.abs(wy_world_m) > 10.0 || Math.abs(wz_world_m) > 10.0) {
        wx_world_m = wx_world_m / 1000.0;
        wy_world_m = wy_world_m / 1000.0;
        wz_world_m = wz_world_m / 1000.0;
      }
      // If passed in robot-base relative coordinates, convert to world meters
      if (wx_world_m > -0.2 && wy_world_m > 0) {
        wx_world_m += this.worldBasePosition.x;
        wz_world_m += this.worldBasePosition.z;
      }
    } else if (Math.abs(wx_world_m) > 10.0 || Math.abs(wy_world_m) > 10.0 || Math.abs(wz_world_m) > 10.0) {
      // Handle case where mm were passed with isWorldCoords = true
      wx_world_m /= 1000.0;
      wy_world_m /= 1000.0;
      wz_world_m /= 1000.0;
    }

    // Offset from Robot Base in millimeters (Three.js Y-up: X = right, Y = up, Z = forward)
    const dx_mm = (wx_world_m - this.worldBasePosition.x) * 1000.0;
    const dy_mm = (wy_world_m - this.worldBasePosition.y) * 1000.0;
    const dz_mm = (wz_world_m - this.worldBasePosition.z) * 1000.0;

    const { d1, a2, d4, d6 } = this.dh;

    // Desired tool pitch angle psi (default -90 deg = pointing straight down -Y)
    const rxOffsetDeg = targetPose.rx ?? 0;
    const psiRad = THREE.MathUtils.degToRad(-90.0 + rxOffsetDeg);

    // 1. STEP 1: Solve Base Swivel Angle S (rotation around +Y)
    // Since worldX = base.x + r * cos(s) and worldZ = base.z - r * sin(s):
    const r_tcp = Math.sqrt(dx_mm * dx_mm + dz_mm * dz_mm);
    let sRad = Math.atan2(-dz_mm, dx_mm);

    // 2. STEP 2: Wrist Center W in the radial-vertical plane (r_W, h_W relative to shoulder pivot d1)
    const r_W = r_tcp - d6 * Math.cos(psiRad);
    const y_W = dy_mm - d6 * Math.sin(psiRad);
    const h_W = y_W - d1;

    const D = Math.sqrt(r_W * r_W + h_W * h_W);
    const maxReach = a2 + d4 - 2.0; // 1578 mm

    // Clamp softly if slightly outside boundary so trajectories never abort mid-path
    const D_clamped = Math.min(D, maxReach);
    const scale = (D > maxReach && D > 1e-6) ? (maxReach / D) : 1.0;
    const r_W_eff = r_W * scale;
    const h_W_eff = h_W * scale;

    // 3. STEP 3: Law of Cosines in the 2-link plane (L2 = a2 along +Y at l=0, L3 = d4 along +X at l+u=0)
    // Let theta_a = l + pi/2 (angle of L2 from +r axis), theta_b = u - pi/2 (relative angle of L3 from L2)
    const cosBeta = THREE.MathUtils.clamp(
      (D_clamped * D_clamped - a2 * a2 - d4 * d4) / (2 * a2 * d4),
      -1.0,
      1.0
    );

    // Elbow-up configuration corresponds to negative beta in standard plane
    const beta = -Math.acos(cosBeta);
    const alpha = Math.atan2(h_W_eff, r_W_eff) - Math.atan2(d4 * Math.sin(beta), a2 + d4 * Math.cos(beta));

    const lRad = alpha - Math.PI / 2;
    const uRad = beta + Math.PI / 2;

    // 4. STEP 4: Spherical Wrist angles (R, B, T)
    // Keep tool pitch equal to psiRad (-90 deg = straight down): l + u + b = psiRad
    const bRad = psiRad - lRad - uRad;
    const rDeg = targetPose.ry ?? 0.0;
    const tDeg = targetPose.rz ?? 0.0;

    let sDeg = THREE.MathUtils.radToDeg(sRad);
    let lDeg = THREE.MathUtils.radToDeg(lRad);
    let uDeg = THREE.MathUtils.radToDeg(uRad);
    let bDeg = THREE.MathUtils.radToDeg(bRad);

    // Clamp within safety joint limits
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
