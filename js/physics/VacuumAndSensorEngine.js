import * as THREE from 'three';

/**
 * VacuumAndSensorEngine.js
 * 
 * Physics, Vision Scanner, Optical Proximity Sensors & Vacuum Gripper Engine
 * 
 * Features:
 * 1. Cognex Structured Light Vision Scanner (Conveyor 1):
 *    - Real-time laser triangulation trigger when products cross plane X = 2.8m.
 *    - Logs real coordinates (x, y, z) and yaw angle into visionQueue.
 * 2. Optical Sensors:
 *    - DI_11: Collation accumulation sensor at X = -1.25m on Conveyor 2.
 *    - DI_12 / DI_13: Carton box stopper parking sensor on Conveyor 3.
 * 3. Vacuum Grippers via Scene Graph Re-parenting:
 *    - Delta single suction cup: picks bar from Belt 1, aligns yaw to 0, collates on Belt 2.
 *    - 6-DOF Multi-Gripper: picks full 4-pack, lifts via trajectory, stacks flat into Carton Box (Layer 1 & Layer 2).
 */

export class VacuumAndSensorEngine {
  constructor(scene, factory, deltaRobot, arm6DOF) {
    this.scene = scene;
    this.factory = factory;
    this.deltaRobot = deltaRobot;
    this.arm6DOF = arm6DOF;

    // Vision Scanner Queue (Detected products waiting for pick)
    this.visionQueue = [];
    this.visionScanX = 2.8;     // X plane of Cognex laser
    this.visionWindow = 0.08;   // Trigger hysteresis window
    this.scannedProductIds = new Set();

    // Vacuum Grippers State
    this.deltaVacuum = {
      active: false,
      attachedObject: null,
      flowPressureKPa: -85.0 // Vacuum level (kPa)
    };

    this.armVacuum = {
      active: false,
      attachedObject: null,
      flowPressureKPa: -90.0,
      currentBoxLayers: 0    // Track stacked layers inside carton box
    };

    // Active free-fall objects under gravity (g = -9.8 m/s^2)
    this.fallingObjects = [];

    // Sensor State Cache
    this.sensors = {
      DI_10: true,   // Cognex Camera Ready
      DI_11: false,  // Collation 4-Pack Ready
      DI_12: false,  // Box at Stopper
      DI_13: false   // Box Full / Stacking Complete
    };

    // Stacking height parameters inside Carton Box (on Rear Conveyor 3 behind 6-DOF Arm at X = -1.25, Z = -1.45, Y = 0.75)
    this.boxStackingParams = {
      boxCenterX: -1.25,
      boxCenterZ: -1.45,
      layer1Y: 0.79,  // First layer bottom
      layer2Y: 0.83   // Second layer top
    };
  }

  /**
   * Main physics & sensor update loop called each frame
   * @param {number} delta - Frame delta time in seconds
   */
  update(delta = 0.016) {
    this.updateVisionScanner();
    this.updateProximitySensors();
    this.updateGravityDrop(delta);
  }

  /**
   * Clear active falling objects (e.g. on fault reset or scene reinit)
   */
  clearFallingObjects() {
    this.fallingObjects = [];
  }

  /**
   * 1. Structured Light Vision Scanner on Conveyor 1
   * Detects products crossing Cognex laser sheet at X = 2.8m
   */
  updateVisionScanner() {
    if (!this.factory || !this.factory.infeedProducts) return;

    const products = this.factory.infeedProducts;
    for (let i = 0; i < products.length; i++) {
      const item = products[i];
      if (item.picked) continue;
      const mesh = item.mesh;
      const x = mesh.position.x;

      // Check if crossing Cognex scanner plane
      if (Math.abs(x - this.visionScanX) < this.visionWindow) {
        if (!this.scannedProductIds.has(mesh.uuid)) {
          this.scannedProductIds.add(mesh.uuid);

          // Increment Total Inspected Metric in Factory Environment
          if (this.factory) {
            this.factory.totalInspectedCount++;
          }

          const isNG = (item.status === "NG" || item.isDefective === true);

          // Latch Rotary Encoder Base Pulse (Base_Encoder_Pulse) at t_0 for Dynamic Conveyor Tracking & Reject Pusher Shift Tracking
          const worldPos = new THREE.Vector3();
          mesh.getWorldPosition(worldPos);
          const basePulse = this.factory ? (this.factory.encoderPulseCounter || 0) : 0;
          item.scanX0 = worldPos.x;
          item.scanZ0 = worldPos.z;
          item.baseEncoderPulse = basePulse;
          item.scanned = true;
          if (this.factory) {
            this.factory.baseEncoderPulse = Math.floor(basePulse);
            this.factory.Base_Encoder_Pulse = Math.floor(basePulse);
          }

          // Only queue OK products for Delta Robot pick & place! (NG pieces are tracked to X = 2.45m and ejected by Reject Pusher)
          if (!isNG) {
            const record = {
              id: mesh.uuid,
              mesh: mesh,
              x: Number(worldPos.x.toFixed(3)),
              y: Number(worldPos.y.toFixed(3)),
              z: Number(worldPos.z.toFixed(3)),
              yawDeg: Number(THREE.MathUtils.radToDeg(mesh.rotation.y).toFixed(1)),
              baseEncoderPulse: Math.floor(basePulse),
              timestamp: performance.now()
            };

            this.visionQueue.push(record);
          }

          // Trigger Cognex Camera visual flash effect (Red for NG defect, Cyan for OK)
          if (this.factory.camStatusLed) {
            this.factory.camStatusLed.material.color.setHex(isNG ? 0xef4444 : 0x38bdf8);
            setTimeout(() => {
              if (this.factory.camStatusLed) {
                this.factory.camStatusLed.material.color.setHex(0x22c55e);
              }
            }, 140);
          }
        }
      }
    }

    // Clean up items that moved far past the Delta pick area (X < -0.5)
    this.visionQueue = this.visionQueue.filter(entry => {
      if (!entry.mesh || !entry.mesh.parent) return false;
      return entry.mesh.position.x > -0.5;
    });
  }

  /**
   * 2. Optical Proximity Sensors (DI_11, DI_12, DI_13)
   */
  updateProximitySensors() {
    // Sensor DI_11 (Pickup station for 6-DOF Arm - only triggers for COMPLETE 4-packs!)
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

    // Sensor DI_12 / DI_13 (Rear Box Conveyor 3 at X = -1.25, Z = -1.45)
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

  // ===========================================================================
  // VACUUM GRIPPER ATTACH / DETACH (Clean Alignment & Physics Lock)
  // ===========================================================================

  /**
   * Attach product to Robot Delta single suction cup
   * @param {THREE.Object3D} productMesh - Product to attach
   */
  attachDeltaGripper(productMesh) {
    if (!productMesh || !this.deltaRobot || !this.deltaRobot.toolFlange) return false;

    // Mark item as picked so Conveyor 1 stops translating it
    if (this.factory && this.factory.infeedProducts) {
      const found = this.factory.infeedProducts.find(p => p.mesh === productMesh);
      if (found) found.picked = true;
    }

    this.deltaVacuum.active = true;
    this.deltaVacuum.attachedObject = productMesh;
    if (this.deltaRobot && typeof this.deltaRobot.setVacuumActive === 'function') {
      this.deltaRobot.setVacuumActive(true);
    }

    // Parent directly to toolFlange and snap cleanly to the silicone suction cup tip (-0.116m)
    this.deltaRobot.toolFlange.add(productMesh);
    productMesh.position.set(0, -0.116, 0);
    productMesh.rotation.set(0, 0, 0);

    return true;
  }

  /**
   * Release product from Robot Delta with Free-Fall Gravity Drop (g = -9.8 m/s^2)
   * The chocolate bar drops freely until contact with Conveyor 2 (beltCollate),
   * verified via 3D bounding box (THREE.Box3) collision detection.
   */
  releaseDeltaGripper(targetParent = null) {
    if (!this.deltaVacuum.attachedObject) return null;

    const obj = this.deltaVacuum.attachedObject;
    this.deltaVacuum.active = false;
    this.deltaVacuum.attachedObject = null;
    if (this.deltaRobot && typeof this.deltaRobot.setVacuumActive === 'function') {
      this.deltaRobot.setVacuumActive(false);
    }

    if (this.factory && this.factory.root) {
      // 1. Capture current world position & orientation from Delta suction cup
      const worldPos = new THREE.Vector3();
      const worldQuat = new THREE.Quaternion();
      obj.getWorldPosition(worldPos);
      obj.getWorldQuaternion(worldQuat);

      // 2. Detach from toolFlange and re-parent to FactoryEnvironment root
      this.factory.root.add(obj);
      obj.position.copy(worldPos);
      obj.quaternion.copy(worldQuat);

      // 3. Register in fallingObjects queue for physics gravity drop
      const targetBelt = this.factory.beltCollate;
      let targetTopY = 0.865;
      if (targetBelt) {
        const beltBox = new THREE.Box3().setFromObject(targetBelt);
        targetTopY = beltBox.max.y;
      }

      this.fallingObjects.push({
        type: 'delta_bar',
        mesh: obj,
        vy: 0.0,
        targetBelt: targetBelt,
        targetTopY: targetTopY,
        onLanded: () => {
          // Place 1 bar into the progressive 4-bar collation slot at X = +1.20m on Conveyor 2!
          this.factory.placeSingleBarOnConveyor2(1.20);

          // Notify OEE Calculator for real-time Performance (P) 60s sliding window
          if (window.app && window.app.oeeCalculator) {
            window.app.oeeCalculator.recordProductPicked(1);
          }

          // Recycle the individual bar back to the upstream infeed of Conveyor 1 cleanly spaced
          let maxInfeedX = 3.2;
          for (const p of this.factory.infeedProducts) {
            if (p.mesh !== obj && !p.picked && p.mesh.position.x > maxInfeedX) {
              maxInfeedX = p.mesh.position.x;
            }
          }
          const spacing = this.factory.infeedSpacing || 0.55;
          obj.position.set(Math.max(3.65, maxInfeedX + spacing), 0.885, -0.32 + (Math.random() - 0.5) * 0.14);
          obj.rotation.set(0, (Math.random() - 0.5) * 0.45, 0);

          const found = this.factory.infeedProducts.find(p => p.mesh === obj);
          if (found) {
            found.picked = false;
            found.baseEncoderPulse = undefined;
            found.scanX0 = undefined;
            found.scanZ0 = undefined;
            found.scanned = false;
            found.rejected = false;
            found.falling = false;
            if (this.factory.nextProductIsNG) {
              found.isDefective = true;
              found.status = "NG";
              if (found.wrapMesh) found.wrapMesh.material = this.factory.matProductDefectNG;
              if (found.goldMesh) found.goldMesh.material = this.factory.matSafetyBlack;
              if (found.crackGroup) found.crackGroup.visible = true;
              this.factory.nextProductIsNG = false;
            } else {
              found.isDefective = false;
              found.status = "OK";
              if (found.wrapMesh) found.wrapMesh.material = this.factory.matFoilRed;
              if (found.goldMesh) found.goldMesh.material = this.factory.matFoilGold;
              if (found.crackGroup) found.crackGroup.visible = false;
            }
            if (found.markerMesh) found.markerMesh.visible = (found.status === "NG");
          }
          this.scannedProductIds.delete(obj.uuid);
        }
      });
    }

    return obj;
  }

  /**
   * Attach collated 4-Pack to 6-DOF Robot Arm Multi-Gripper
   * @param {THREE.Object3D} packMesh - 4-Pack cluster to attach
   */
  attachArmGripper(packMesh) {
    if (!packMesh || !this.arm6DOF || !this.arm6DOF.tcpMarker) return false;

    // Mark pack as picked so Conveyor 2 stops translating it
    if (this.factory && this.factory.collatedPacks) {
      const found = this.factory.collatedPacks.find(p => p.mesh === packMesh);
      if (found) {
        found.picked = true;
        found.stopped = true;
      }
    }

    this.armVacuum.active = true;
    this.armVacuum.attachedObject = packMesh;

    // Parent directly to tcpMarker and snap flat against the 8 downward-facing suction cups
    // Since tcpMarker local +X points straight down (-Y in world) when L+U+B = -90°,
    // rotating packMesh by +90° around Z makes the 4-pack completely horizontal!
    this.arm6DOF.tcpMarker.add(packMesh);
    packMesh.position.set(0.02, 0, 0);
    packMesh.rotation.set(0, 0, Math.PI / 2);

    return true;
  }

  /**
   * Release 4-Pack from 6-DOF Arm with Free-Fall Gravity Drop (g = -9.8 m/s^2)
   * Stacks neatly into Carton Box on Conveyor 3 until bounding box contact with box floor or layer 1.
   * @param {THREE.Object3D} [targetParent] - Active Carton Box group
   */
  releaseArmGripper(targetParent = null) {
    if (!this.armVacuum.attachedObject) return null;

    const obj = this.armVacuum.attachedObject;
    this.armVacuum.active = false;
    this.armVacuum.attachedObject = null;

    // Remove this pack from Conveyor 2's collatedPacks array since it is now packed inside the box
    if (this.factory && this.factory.collatedPacks) {
      const idx = this.factory.collatedPacks.findIndex(p => p.mesh === obj);
      if (idx !== -1) {
        this.factory.collatedPacks.splice(idx, 1);
      }
    }

    // Find active carton box if not explicitly passed
    let boxGroup = targetParent;
    let activeBoxRecord = null;
    if (this.factory && this.factory.cartonBoxes) {
      activeBoxRecord = this.factory.cartonBoxes.find(b => !b.full) || this.factory.cartonBoxes[0];
      if (!boxGroup && activeBoxRecord) {
        boxGroup = activeBoxRecord.mesh;
      }
    }

    const parent = boxGroup || (this.factory ? this.factory.root : this.scene);

    // Capture world position & re-parent
    const worldPos = new THREE.Vector3();
    obj.getWorldPosition(worldPos);

    parent.add(obj);

    if (boxGroup) {
      parent.worldToLocal(worldPos);
      obj.position.copy(worldPos);
      obj.rotation.set(0, 0, 0);
    } else {
      obj.position.copy(worldPos);
      obj.rotation.set(0, 0, 0);
    }

    const layerIdx = this.armVacuum.currentBoxLayers % 2;

    this.fallingObjects.push({
      type: 'arm_pack',
      mesh: obj,
      boxGroup: boxGroup,
      activeBoxRecord: activeBoxRecord,
      layerIdx: layerIdx,
      vy: 0.0,
      onLanded: () => {
        this.armVacuum.currentBoxLayers++;

        // If 2 layers (8 chocolate bars) are stacked, mark the box as full so it discharges and a fresh box rolls in
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
      }
    });

    return obj;
  }

  /**
   * Free-fall gravity drop physics simulation (g = -9.8 m/s^2)
   * Calculates continuous acceleration, updates Y coordinate over time delta,
   * and verifies contact with conveyor/carton floor via 3D Bounding Box (THREE.Box3).
   * @param {number} delta - Frame delta time in seconds
   */
  updateGravityDrop(delta) {
    if (!this.fallingObjects || this.fallingObjects.length === 0) return;

    const g = -9.8; // Gravity acceleration m/s^2

    for (let i = this.fallingObjects.length - 1; i >= 0; i--) {
      const item = this.fallingObjects[i];
      if (!item.mesh || !item.mesh.parent) {
        this.fallingObjects.splice(i, 1);
        continue;
      }

      // Update vertical velocity and apply gravity
      item.vy += g * delta;
      item.mesh.position.y += item.vy * delta;

      if (item.type === 'delta_bar') {
        // Delta bar falling onto Collation Belt (Conveyor 2)
        let targetTopY = item.targetTopY;
        if (item.targetBelt) {
          const beltBox = new THREE.Box3().setFromObject(item.targetBelt);
          targetTopY = beltBox.max.y;
        }

        // Bounding box of the falling bar in world space
        const objBox = new THREE.Box3().setFromObject(item.mesh);

        // Check contact: bottom of bar meets or passes belt top surface
        if (objBox.min.y <= targetTopY || item.mesh.position.y <= 0.88) {
          if (objBox.min.y <= targetTopY) {
            item.mesh.position.y += (targetTopY - objBox.min.y);
          }
          item.vy = 0.0;
          this.fallingObjects.splice(i, 1);
          if (typeof item.onLanded === 'function') {
            item.onLanded();
          }
        }
      } else if (item.type === 'arm_pack') {
        // 4-Pack falling inside Carton Box
        const targetFloorY = (item.layerIdx === 0) ? 0.032 : 0.072;

        if (item.boxGroup) {
          let landed = false;

          if (item.layerIdx === 0) {
            // Collision check against box bottom plate (first child)
            const btm = item.boxGroup.children[0];
            if (btm) {
              const btmBox = new THREE.Box3().setFromObject(btm);
              const objBox = new THREE.Box3().setFromObject(item.mesh);
              if (objBox.min.y <= btmBox.max.y) {
                landed = true;
              }
            }
          } else {
            // Collision check against Layer 1 pack
            const layer1Mesh = item.boxGroup.children.find(c => c !== item.boxGroup.children[0] && c !== item.mesh);
            if (layer1Mesh) {
              const layer1Box = new THREE.Box3().setFromObject(layer1Mesh);
              const objBox = new THREE.Box3().setFromObject(item.mesh);
              if (objBox.min.y <= layer1Box.max.y) {
                landed = true;
              }
            }
          }

          // Also check local Y threshold to ensure stability
          if (landed || item.mesh.position.y <= targetFloorY) {
            item.mesh.position.y = targetFloorY;
            item.mesh.position.x = 0;
            item.mesh.position.z = 0;
            item.vy = 0.0;
            this.fallingObjects.splice(i, 1);
            if (typeof item.onLanded === 'function') {
              item.onLanded();
            }
          }
        } else {
          // Fallback if not parented to boxGroup (world coordinates)
          const worldTargetY = 0.75 + targetFloorY;
          if (item.mesh.position.y <= worldTargetY) {
            item.mesh.position.y = worldTargetY;
            item.vy = 0.0;
            this.fallingObjects.splice(i, 1);
            if (typeof item.onLanded === 'function') {
              item.onLanded();
            }
          }
        }
      }
    }
  }

  /**
   * Compute stacking target position inside Carton Box
   * Layer 1 (bottom) or Layer 2 (stacked on top)
   */
  getBoxPlacementPose(layer = 1) {
    const yPos = (layer === 1) ? this.boxStackingParams.layer1Y : this.boxStackingParams.layer2Y;
    return {
      x: this.boxStackingParams.boxCenterX,
      y: yPos,
      z: this.boxStackingParams.boxCenterZ,
      rx: 0,
      ry: 0,
      rz: 0
    };
  }

  /**
   * Find the closest product on Conveyor 1 within reach of Robot Delta
   */
  findReachableInfeedProduct(deltaWorldX = 1.2, reachRadius = 0.45) {
    if (!this.factory || !this.factory.infeedProducts) return null;

    let bestMesh = null;
    let bestDist = Infinity;
    for (const item of this.factory.infeedProducts) {
      // Delta ignores picked items, NG defect products, items being rejected, and falling items!
      if (item.picked || item.status === 'NG' || item.isDefective || item.rejected || item.falling) continue;
      const mesh = item.mesh;
      const dist = Math.abs(mesh.position.x - deltaWorldX);
      if (dist < reachRadius && dist < bestDist) {
        bestDist = dist;
        bestMesh = mesh;
      }
    }
    return bestMesh;
  }

  /**
   * Find collated 4-pack at Conveyor 2 ready for 6-DOF Arm pickup
   */
  findReachableCollatedPack(armPickupX = -1.25, threshold = 0.25) {
    if (!this.factory || !this.factory.collatedPacks) return null;

    for (const pack of this.factory.collatedPacks) {
      if (pack.picked || pack.complete === false) continue;
      const mesh = pack.mesh;
      if (Math.abs(mesh.position.x - armPickupX) < threshold) {
        return mesh;
      }
    }
    return null;
  }
}
