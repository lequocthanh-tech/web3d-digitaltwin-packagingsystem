import * as THREE from 'three';

/**
 * TrajectoryTrail.js
 * 
 * 3D Trajectory Visualizer for End-Effector Paths in Three.js space
 * - Arm 6-DOF Trail: Vibrant Cyan (0x00ffff) with Glowing Neon Laser Core
 * - Delta Robot Trail: Industrial Amber/Yellow (0xffb700) with Radiant Bead Trail
 * Uses pre-allocated Float32Array ring buffers to ensure 60 FPS without GC frame drops.
 */

function createGlowDotTexture() {
  const canvas = document.createElement('canvas');
  canvas.width = 64;
  canvas.height = 64;
  const ctx = canvas.getContext('2d');
  const grad = ctx.createRadialGradient(32, 32, 0, 32, 32, 32);
  grad.addColorStop(0, 'rgba(255, 255, 255, 1)');
  grad.addColorStop(0.25, 'rgba(255, 255, 255, 0.95)');
  grad.addColorStop(0.55, 'rgba(255, 255, 255, 0.55)');
  grad.addColorStop(0.85, 'rgba(255, 255, 255, 0.15)');
  grad.addColorStop(1, 'rgba(255, 255, 255, 0)');
  ctx.fillStyle = grad;
  ctx.fillRect(0, 0, 64, 64);
  const tex = new THREE.CanvasTexture(canvas);
  return tex;
}

const sharedGlowTexture = typeof document !== 'undefined' ? createGlowDotTexture() : null;

export class TrajectoryTrail {
  constructor(scene, maxPoints = 400) {
    this.scene = scene;
    this.maxPoints = maxPoints;
    this.visible = false;       // Mặc định TẮT - chỉ hiển thị khi người dùng yêu cầu bật đồ thị / quỹ đạo
    this.armVisible = false;
    this.deltaVisible = false;

    this.group = new THREE.Group();
    this.group.name = "TrajectoryTrailsGroup";
    this.group.visible = false;
    this.scene.add(this.group);

    // 1. Arm 6-DOF Trail (High-Intensity Electric Cyan - Neon 3D Glow)
    this.armPositions = new Float32Array(this.maxPoints * 3);
    this.armCount = 0;
    this.armHead = 0;

    const armGeo = new THREE.BufferGeometry();
    armGeo.setAttribute('position', new THREE.BufferAttribute(this.armPositions, 3));
    
    // Core vibrant line (depthTest: false guarantees it is NEVER obscured or faint behind objects)
    this.armMat = new THREE.LineBasicMaterial({
      color: 0x00ffff, // Electric Cyan
      linewidth: 3,
      transparent: true,
      opacity: 1.0,
      depthTest: false,
      depthWrite: false
    });
    this.armLine = new THREE.Line(armGeo, this.armMat);
    this.armLine.renderOrder = 9999;
    this.armLine.frustumCulled = false;
    this.armLine.visible = false;

    // Glowing Laser Beads along trajectory to give the line thick, bold body in WebGL!
    this.armPointsMat = new THREE.PointsMaterial({
      color: 0x00e5ff,
      size: 9, // 9px screen-space thickness
      map: sharedGlowTexture,
      blending: THREE.AdditiveBlending,
      sizeAttenuation: false,
      depthTest: false,
      depthWrite: false,
      transparent: true,
      opacity: 0.95
    });
    this.armPoints = new THREE.Points(armGeo, this.armPointsMat);
    this.armPoints.renderOrder = 9999;
    this.armPoints.frustumCulled = false;
    this.armPoints.visible = false;

    this.group.add(this.armLine, this.armPoints);

    // 2. Delta Robot Trail (High-Intensity Neon Amber/Gold)
    this.deltaPositions = new Float32Array(this.maxPoints * 3);
    this.deltaCount = 0;
    this.deltaHead = 0;

    const deltaGeo = new THREE.BufferGeometry();
    deltaGeo.setAttribute('position', new THREE.BufferAttribute(this.deltaPositions, 3));
    this.deltaMat = new THREE.LineBasicMaterial({
      color: 0xffb700, // Vibrant Gold/Amber
      linewidth: 3,
      transparent: true,
      opacity: 1.0,
      depthTest: false,
      depthWrite: false
    });
    this.deltaLine = new THREE.Line(deltaGeo, this.deltaMat);
    this.deltaLine.renderOrder = 9999;
    this.deltaLine.frustumCulled = false;
    this.deltaLine.visible = false;

    this.deltaPointsMat = new THREE.PointsMaterial({
      color: 0xffc107,
      size: 9,
      map: sharedGlowTexture,
      blending: THREE.AdditiveBlending,
      sizeAttenuation: false,
      depthTest: false,
      depthWrite: false,
      transparent: true,
      opacity: 0.95
    });
    this.deltaPoints = new THREE.Points(deltaGeo, this.deltaPointsMat);
    this.deltaPoints.renderOrder = 9999;
    this.deltaPoints.frustumCulled = false;
    this.deltaPoints.visible = false;

    this.group.add(this.deltaLine, this.deltaPoints);

    this.lastArmPos = new THREE.Vector3();
    this.lastDeltaPos = new THREE.Vector3();
    this.minDist = 0.003; // 3mm threshold for continuous smooth density
  }

  /**
   * Add a point to Arm 6-DOF trail
   */
  addArmPoint(worldPos) {
    if (!this.armVisible || !worldPos) return;

    if (this.armCount > 0 && this.lastArmPos.distanceTo(worldPos) < this.minDist) {
      return;
    }
    this.lastArmPos.copy(worldPos);

    if (this.armCount < this.maxPoints) {
      const idx = this.armCount * 3;
      this.armPositions[idx] = worldPos.x;
      this.armPositions[idx + 1] = worldPos.y;
      this.armPositions[idx + 2] = worldPos.z;
      this.armCount++;
    } else {
      // Shift buffer to slide trail
      this.armPositions.copyWithin(0, 3, this.maxPoints * 3);
      const lastIdx = (this.maxPoints - 1) * 3;
      this.armPositions[lastIdx] = worldPos.x;
      this.armPositions[lastIdx + 1] = worldPos.y;
      this.armPositions[lastIdx + 2] = worldPos.z;
    }

    this.armLine.geometry.setDrawRange(0, this.armCount);
    this.armLine.geometry.attributes.position.needsUpdate = true;
  }

  /**
   * Add a point to Delta Robot trail
   */
  addDeltaPoint(worldPos) {
    if (!this.deltaVisible || !worldPos) return;

    if (this.deltaCount > 0 && this.lastDeltaPos.distanceTo(worldPos) < this.minDist) {
      return;
    }
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
    this.armPoints.visible = visible;
    this.deltaLine.visible = visible;
    this.deltaPoints.visible = visible;
    this.group.visible = visible;
  }

  setArmVisible(visible) {
    this.armVisible = visible;
    this.armLine.visible = visible;
    this.armPoints.visible = visible;
    this.visible = this.armVisible || this.deltaVisible;
    this.group.visible = this.visible;
    if (!visible) {
      this.armCount = 0;
      this.armLine.geometry.setDrawRange(0, 0);
      this.armLine.geometry.attributes.position.needsUpdate = true;
    }
  }

  setDeltaVisible(visible) {
    this.deltaVisible = visible;
    this.deltaLine.visible = visible;
    this.deltaPoints.visible = visible;
    this.visible = this.armVisible || this.deltaVisible;
    this.group.visible = this.visible;
    if (!visible) {
      this.deltaCount = 0;
      this.deltaLine.geometry.setDrawRange(0, 0);
      this.deltaLine.geometry.attributes.position.needsUpdate = true;
    }
  }

  clear() {
    this.armCount = 0;
    this.deltaCount = 0;
    this.armLine.geometry.setDrawRange(0, 0);
    this.deltaLine.geometry.setDrawRange(0, 0);
    this.armLine.geometry.attributes.position.needsUpdate = true;
    this.deltaLine.geometry.attributes.position.needsUpdate = true;
  }
}
