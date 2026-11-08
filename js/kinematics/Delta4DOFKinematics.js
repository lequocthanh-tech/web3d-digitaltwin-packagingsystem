import * as THREE from 'three';

/**
 * Delta4DOFKinematics.js
 * 
 * High-Precision Parallel Kinematics Engine for 4-DOF Industrial Delta Robot (FANUC M-2iA / M-3iA Style)
 * Synchronized 1:1 with DeltaRobot3D.js:
 * - Gantry Base World Position: (1.2, 1.42, -0.1) m
 * - Base radius f = 0.180 m, Mobile platform radius e = 0.050 m
 * - Upper arm length rf (L1) = 0.200 m, Lower parallelogram carbon rods re (L2) = 0.500 m
 * - Tool tip vertical offset below mobile platform: tipOffset = 0.109 m (suction cup lip bottom)
 * - Safe vertical workspace: ly in [-0.445m, -0.20m], guaranteeing 0mm mechanical penetration into conveyor belt!
 */

export class Delta4DOFKinematics {
  constructor(options = {}) {
    // Geometric Delta Parameters (in meters) matching DeltaRobot3D.js
    this.f = options.f ?? 0.180;              // Base pivot radius (rBase = 180 mm)
    this.e = options.e ?? 0.050;              // Platform joint radius (rPlatform = 50 mm)
    this.rf = options.rf ?? 0.200;            // Upper arm (bicep) length L1 = 200 mm
    this.re = options.re ?? 0.500;            // Lower parallelogram carbon rods length L2 = 500 mm
    this.rodWidth = options.rodWidth ?? 0.055; // Distance between 2 rods in each parallelogram
    this.tipOffset = options.tipOffset ?? 0.109; // Suction cup tip offset below mobile platform (0.012 flange + 0.097 tool)

    // World base mount position (overhead gantry)
    this.worldBasePosition = options.worldBasePosition || new THREE.Vector3(1.2, 1.42, -0.1);

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
   * Accounts for the suction cup tip offset (0.109m below platform center)
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

    // Clamp local coordinates within safe physical workspace envelope:
    // ly in [-0.445, -0.20] prevents suction cup from EVER penetrating the conveyor belt (table surface Y = 0.8655m)
    lx = THREE.MathUtils.clamp(lx, -0.45, 0.45);
    lz = THREE.MathUtils.clamp(lz, -0.45, 0.45);
    ly = THREE.MathUtils.clamp(ly, -0.445, -0.20);

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
