import * as THREE from 'three';

/**
 * TrajectoryPlanner.js
 * 
 * Industrial Trajectory Interpolator & Motion Planner for Multi-Axis Robots
 * 
 * Features:
 * 1. Cubic (Bậc 3) & Quintic (Bậc 5) Polynomial Time Scaling:
 *    Calculates Position s(t), Velocity v(t), Acceleration a(t), and Jerk(t).
 * 2. MOVJ (Joint Interpolation):
 *    Synchronous multi-axis motion in joint space from q_start to q_target.
 * 3. MOVL (Linear Cartesian Interpolation):
 *    Strict linear motion in Cartesian space from Pose_A to Pose_B.
 *    Interpolates position and orientation, invoking inverse kinematics at each time step.
 * 4. Active Trajectory Runner / Execution Engine with callback events (onUpdate, onComplete).
 */

export class TrajectoryPlanner {
  constructor() {
    this.activeTrajectories = new Map();
  }

  /**
   * Polynomial Time Scaling s(t)
   * @param {number} t - Current elapsed time (seconds)
   * @param {number} duration - Total motion duration (seconds)
   * @param {number} [degree=5] - 3 for Cubic, 5 for Quintic (default)
   * @returns {Object} { s, v, a, jerk, tau }
   */
  static computeTimeScaling(t, duration, degree = 5) {
    if (duration <= 0) {
      return { s: 1.0, v: 0.0, a: 0.0, jerk: 0.0, tau: 1.0 };
    }

    const tau = THREE.MathUtils.clamp(t / duration, 0.0, 1.0);
    const T = duration;

    if (degree === 3) {
      // Cubic Polynomial: s(tau) = 3*tau^2 - 2*tau^3
      const s = 3 * tau * tau - 2 * tau * tau * tau;
      const v = (6 * tau - 6 * tau * tau) / T;
      const a = (6 - 12 * tau) / (T * T);
      const jerk = -12 / (T * T * T);
      return { s, v, a, jerk, tau };
    } else {
      // Quintic Polynomial (C2 Continuous, Zero Jerk at endpoints):
      // s(tau) = 10*tau^3 - 15*tau^4 + 6*tau^5
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

  /**
   * Create MOVJ (Joint Interpolation) Trajectory
   * Synchronously interpolates all joints from qStart to qTarget.
   * 
   * @param {Object} qStart - Initial joint angles { s, l, u, r, b, t } or { m1, m2, m3, yaw }
   * @param {Object} qTarget - Target joint angles
   * @param {Object} options - { duration: number, degree: number }
   * @returns {Object} Trajectory descriptor
   */
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

  /**
   * Create MOVL (Cartesian Linear Interpolation) Trajectory
   * Interpolates strictly along a 3D straight line in space, calling IK at each step.
   * 
   * @param {Object} poseStart - { x, y, z, rx, ry, rz }
   * @param {Object} poseTarget - { x, y, z, rx, ry, rz }
   * @param {Object} ikEngine - Instance providing inverseKinematics(pose, currentJoints)
   * @param {Object} options - { duration: number, degree: number, isWorld: boolean }
   * @returns {Object} Trajectory descriptor
   */
  createMOVL(poseStart, poseTarget, ikEngine, options = {}) {
    const duration = options.duration ?? 2.0;
    const degree = options.degree ?? 5;
    const isWorld = options.isWorld ?? false;

    // Vectors for linear Cartesian path
    const p0 = new THREE.Vector3(poseStart.x, poseStart.y, poseStart.z);
    const p1 = new THREE.Vector3(poseTarget.x, poseTarget.y, poseTarget.z);

    // Orientation angles in degrees (supports yaw for Delta 4th axis)
    const r0 = {
      rx: poseStart.rx ?? poseStart.yaw ?? 0,
      ry: poseStart.ry ?? 0,
      rz: poseStart.rz ?? 0
    };
    const r1 = {
      rx: poseTarget.rx ?? poseTarget.yaw ?? r0.rx,
      ry: poseTarget.ry ?? 0,
      rz: poseTarget.rz ?? 0
    };

    // Orientation interpolation via THREE.Quaternion.slerp() to eliminate Gimbal Lock
    const euler0 = new THREE.Euler(
      THREE.MathUtils.degToRad(r0.rx),
      THREE.MathUtils.degToRad(r0.ry),
      THREE.MathUtils.degToRad(r0.rz),
      'XYZ'
    );
    const q0 = poseStart.quaternion
      ? poseStart.quaternion.clone()
      : new THREE.Quaternion().setFromEuler(euler0);

    const euler1 = new THREE.Euler(
      THREE.MathUtils.degToRad(r1.rx),
      THREE.MathUtils.degToRad(r1.ry),
      THREE.MathUtils.degToRad(r1.rz),
      'XYZ'
    );
    const q1 = poseTarget.quaternion
      ? poseTarget.quaternion.clone()
      : new THREE.Quaternion().setFromEuler(euler1);

    // Enforce shortest geodesic arc on unit sphere S^3
    if (q0.dot(q1) < 0) {
      q1.set(-q1.x, -q1.y, -q1.z, -q1.w);
    }

    let lastKnownJoints = options.initialJoints || null;

    return {
      type: 'MOVL',
      duration,
      degree,
      poseStart: { ...poseStart, quaternion: q0.clone() },
      poseTarget: { ...poseTarget, quaternion: q1.clone() },
      evaluate: (t) => {
        const scaling = TrajectoryPlanner.computeTimeScaling(t, duration, degree);
        const s = scaling.s;

        // Current Cartesian position
        const currentP = new THREE.Vector3().lerpVectors(p0, p1, s);

        // Spherical Linear Interpolation (SLERP) for smooth wrist orientation without Gimbal Lock
        const currentQ = new THREE.Quaternion();
        currentQ.slerpQuaternions(q0, q1, s);

        // Convert interpolated quaternion back to Euler angles (degrees) for IK engine compatibility
        const currentEuler = new THREE.Euler().setFromQuaternion(currentQ, 'XYZ');
        const currentR = {
          rx: THREE.MathUtils.radToDeg(currentEuler.x),
          ry: THREE.MathUtils.radToDeg(currentEuler.y),
          rz: THREE.MathUtils.radToDeg(currentEuler.z)
        };

        const targetPose = {
          x: currentP.x,
          y: currentP.y,
          z: currentP.z,
          rx: currentR.rx,
          ry: currentR.ry,
          rz: currentR.rz,
          quaternion: currentQ
        };

        // Solve IK for this waypoint
        let ikResult;
        if (typeof ikEngine.inverseKinematics === 'function') {
          if (ikEngine.constructor.name === 'Delta4DOFKinematics') {
            ikResult = ikEngine.inverseKinematics(
              targetPose.x,
              targetPose.y,
              targetPose.z,
              targetPose.rx ?? 0,
              isWorld
            );
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

  /**
   * Register and start executing a trajectory on a target robot
   * @param {string} id - Unique ID (e.g. 'arm-move' or 'delta-move')
   * @param {Object} trajectory - Trajectory generated by createMOVJ or createMOVL
   * @param {Object} callbacks - { onUpdate(evalData), onComplete() }
   */
  startTrajectory(id, trajectory, callbacks = {}) {
    this.activeTrajectories.set(id, {
      trajectory,
      elapsedTime: 0.0,
      callbacks,
      finished: false
    });
  }

  /**
   * Stop an active trajectory
   */
  stopTrajectory(id) {
    const runner = this.activeTrajectories.get(id);
    if (runner) {
      if (typeof runner.callbacks.onCancel === 'function') {
        runner.callbacks.onCancel();
      } else if (typeof runner.callbacks.onComplete === 'function') {
        runner.callbacks.onComplete({ isFinished: true, cancelled: true });
      }
      this.activeTrajectories.delete(id);
    }
  }

  /**
   * Stop all active trajectories
   */
  stopAll() {
    for (const [id, runner] of this.activeTrajectories.entries()) {
      if (typeof runner.callbacks.onCancel === 'function') {
        runner.callbacks.onCancel();
      } else if (typeof runner.callbacks.onComplete === 'function') {
        runner.callbacks.onComplete({ isFinished: true, cancelled: true });
      }
    }
    this.activeTrajectories.clear();
  }

  /**
   * Update all active trajectories in the main animation loop
   * @param {number} delta - Elapsed frame time in seconds
   */
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
