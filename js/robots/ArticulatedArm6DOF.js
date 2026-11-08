import * as THREE from 'three';

/**
 * ArticulatedArm6DOF - Heavy/Medium Industrial 6-DOF Robotic Arm (FANUC M-10 / M-20 Series)
 * 
 * Upgraded Features for Realism & Scale:
 * - Full industrial proportion matching real packaging line cells (reaches across conveyor table).
 * - Sculpted FANUC organic casting geometry (tapered lower arm, hollow ribbing, contoured elbow, rounded spherical wrist).
 * - Realistic black corrugated cable conduits & pneumatic harness routed along joints.
 * - Heavy-duty industrial riser pedestal with triangular stiffener gussets & anchor bolts.
 * - Multi-Zone vacuum gripper manifold with spring-loaded silicone suction cups & digital vacuum display.
 * - Denavit-Hartenberg (DH) Frames {0}..{6} with billboarded axis labels & floor coordinate drop line.
 */
export class ArticulatedArm6DOF {
  constructor(options = {}) {
    // Positioned alongside conveyor table
    this.basePosition = options.position || new THREE.Vector3(-1.25, 0, -0.75);
    
    // Joint angles in degrees (S, L, U, R, B, T)
    // Default Home Pose facing forward (+Z) over the packaging conveyor table
    this.angles = {
      s: -75.0,  // J1 (Base Swivel - faces forward toward conveyor table)
      l: -18.0,  // J2 (Lower Arm Pitch)
      u: -10.0,  // J3 (Upper Arm Pitch)
      r: 0.0,    // J4 (Forearm Roll)
      b: -62.0,  // J5 (Wrist Bend - keeps suction cups horizontal: l+u+b = -90°)
      t: 0.0     // J6 (Tool Twist)
    };

    this.jointLimits = {
      s: { min: -170, max: 170 },
      l: { min: -85, max: 90 },
      u: { min: -110, max: 110 },
      r: { min: -140, max: 140 },
      b: { min: -135, max: 135 },
      t: { min: -360, max: 360 }
    };

    // Encoder Pulses per degree ratio (Yaskawa MotoMINI standard)
    this.pulseRatios = {
      s: 34816 / 30,    // 1160.5333 pls/deg
      l: 102400 / 90,   // 1137.7777 pls/deg
      u: 51200 / 90,    // 568.8888 pls/deg
      r: 10204 / 30,    // 340.1333 pls/deg
      b: 10204 / 30,    // 340.1333 pls/deg
      t: 10204 / 30     // 340.1333 pls/deg
    };

    this.dhVisible = false;
    this.root = new THREE.Group();
    this.root.name = "FANUC_ArticulatedArm6DOF";
    this.root.position.copy(this.basePosition);

    this.dhHelpers = [];

    this.initMaterials();
    this.buildRobotAssembly();
    this.buildDHFrames();
    this.buildFloorDropLine();
    this.setDHVisible(false);

    // Initial joint state update
    this.updateJoints();
  }

  initMaterials() {
    // Signature FANUC Industrial Gloss Yellow
    this.matFanucYellow = new THREE.MeshStandardMaterial({
      color: 0xfed100, // Vibrant FANUC Yellow
      roughness: 0.26,
      metalness: 0.16
    });

    // Dark Cast Iron / Motor Housing
    this.matCastDark = new THREE.MeshStandardMaterial({
      color: 0x1f2937,
      roughness: 0.5,
      metalness: 0.65
    });

    // Machined Steel & Joint Bearings
    this.matJointSteel = new THREE.MeshStandardMaterial({
      color: 0x94a3b8,
      roughness: 0.25,
      metalness: 0.85
    });

    // Polished Chrome
    this.matChrome = new THREE.MeshStandardMaterial({
      color: 0xf1f5f9,
      roughness: 0.15,
      metalness: 0.95
    });

    // Industrial Pedestal (Coated Steel)
    this.matPedestal = new THREE.MeshStandardMaterial({
      color: 0x1e293b,
      roughness: 0.55,
      metalness: 0.5
    });

    // Flexible Black Corrugated Cable Conduit
    this.matConduit = new THREE.MeshStandardMaterial({
      color: 0x111827,
      roughness: 0.65,
      metalness: 0.3
    });

    // Gripper Aluminum Plenum Block
    this.matGripperWhite = new THREE.MeshStandardMaterial({
      color: 0xf8fafc,
      roughness: 0.25,
      metalness: 0.4
    });

    // Suction Cups (Silicone Blue)
    this.matSuctionCups = new THREE.MeshStandardMaterial({
      color: 0x0284c7,
      roughness: 0.4,
      metalness: 0.1
    });
  }

  createLabelSprite(text) {
    const canvas = document.createElement('canvas');
    canvas.width = 128;
    canvas.height = 64;
    const ctx = canvas.getContext('2d');

    // SCADA style framed badge
    ctx.fillStyle = 'rgba(15, 23, 42, 0.9)';
    ctx.strokeStyle = '#06b6d4';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.roundRect(8, 8, 112, 48, 10);
    ctx.fill();
    ctx.stroke();

    ctx.font = 'bold 26px "JetBrains Mono", Consolas, monospace';
    ctx.fillStyle = '#fed100';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(text, 64, 32);

    const texture = new THREE.CanvasTexture(canvas);
    const spriteMat = new THREE.SpriteMaterial({
      map: texture,
      depthTest: false,
      transparent: true
    });
    const sprite = new THREE.Sprite(spriteMat);
    sprite.scale.set(0.24, 0.12, 1);
    return sprite;
  }

  buildRobotAssembly() {
    // =========================================================================
    // 1. HEAVY-DUTY INDUSTRIAL RISER PEDESTAL
    // Elevated so the robot arm naturally overlooks and reaches the conveyor ($0.85m)
    // =========================================================================
    const pedHeight = 0.52;
    this.pedestalGroup = new THREE.Group();
    this.pedestalGroup.name = "RobotPedestal";

    // Bottom Mounting Flange (Diameter 0.52m)
    const btmFlangeGeo = new THREE.CylinderGeometry(0.26, 0.28, 0.035, 32);
    const btmFlange = new THREE.Mesh(btmFlangeGeo, this.matPedestal);
    btmFlange.position.y = 0.0175;
    btmFlange.castShadow = true;
    btmFlange.receiveShadow = true;
    this.pedestalGroup.add(btmFlange);

    // Stiffener Triangular Gussets (8 gussets around base)
    const gussetShape = new THREE.Shape();
    gussetShape.moveTo(0, 0);
    gussetShape.lineTo(0.12, 0);
    gussetShape.lineTo(0, 0.22);
    gussetShape.closePath();
    const gussetExtrude = { depth: 0.02, bevelEnabled: false };
    const gussetGeo = new THREE.ExtrudeGeometry(gussetShape, gussetExtrude);
    gussetGeo.center();

    for (let i = 0; i < 8; i++) {
      const ang = (i * Math.PI) / 4;
      const gusset = new THREE.Mesh(gussetGeo, this.matPedestal);
      gusset.position.set(0.19 * Math.cos(ang), 0.13, 0.19 * Math.sin(ang));
      gusset.rotation.y = -ang + Math.PI / 2;
      gusset.castShadow = true;
      this.pedestalGroup.add(gusset);
    }

    // Main Steel Column
    const colGeo = new THREE.CylinderGeometry(0.18, 0.20, pedHeight - 0.04, 32);
    const column = new THREE.Mesh(colGeo, this.matPedestal);
    column.position.y = (pedHeight - 0.04) / 2 + 0.02;
    column.castShadow = true;
    column.receiveShadow = true;
    this.pedestalGroup.add(column);

    // Top Adapter Flange
    const topFlangeGeo = new THREE.CylinderGeometry(0.24, 0.24, 0.04, 32);
    const topFlange = new THREE.Mesh(topFlangeGeo, this.matCastDark);
    topFlange.position.y = pedHeight;
    topFlange.castShadow = true;
    this.pedestalGroup.add(topFlange);

    this.root.add(this.pedestalGroup);

    // =========================================================================
    // FRAME {0}: ROBOT FIXED BASE CASTING
    // =========================================================================
    this.frame0 = new THREE.Group();
    this.frame0.position.y = pedHeight + 0.02;
    this.frame0.name = "Frame_0_Base";

    // Sculpted Base Ring
    const baseCastGeo = new THREE.CylinderGeometry(0.22, 0.24, 0.12, 32);
    const baseCast = new THREE.Mesh(baseCastGeo, this.matFanucYellow);
    baseCast.position.y = 0.06;
    baseCast.castShadow = true;
    baseCast.receiveShadow = true;
    this.frame0.add(baseCast);

    // Electrical connector junction box on base side
    const junctionGeo = new THREE.BoxGeometry(0.10, 0.09, 0.08);
    const junction = new THREE.Mesh(junctionGeo, this.matCastDark);
    junction.position.set(-0.21, 0.06, 0);
    this.frame0.add(junction);

    // =========================================================================
    // FRAME {1}: JOINT 1 (S - Base Swivel / Turret around Y)
    // =========================================================================
    this.j1Group = new THREE.Group();
    this.j1Group.position.y = 0.12;
    this.j1Group.name = "Joint1_S_Turret";

    // Turret Lower Rotating Hub
    const turretHubGeo = new THREE.CylinderGeometry(0.20, 0.21, 0.08, 32);
    const turretHub = new THREE.Mesh(turretHubGeo, this.matCastDark);
    turretHub.position.y = 0.04;
    turretHub.castShadow = true;
    this.j1Group.add(turretHub);

    // Main Sculpted Shoulder Casting (Smooth curved industrial contours)
    const shoulderGroup = new THREE.Group();
    shoulderGroup.position.set(0, 0.18, 0);

    const shoulderCenterGeo = new THREE.BoxGeometry(0.28, 0.30, 0.26);
    const shoulderCenter = new THREE.Mesh(shoulderCenterGeo, this.matFanucYellow);
    shoulderCenter.castShadow = true;
    shoulderGroup.add(shoulderCenter);

    // Left & Right Chamfered Cheeks
    for (let sign of [-1, 1]) {
      const cheekGeo = new THREE.CylinderGeometry(0.14, 0.14, 0.28, 24);
      cheekGeo.rotateZ(Math.PI / 2);
      const cheek = new THREE.Mesh(cheekGeo, this.matFanucYellow);
      cheek.position.set(0, 0.12, sign * 0.13);
      cheek.castShadow = true;
      shoulderGroup.add(cheek);
    }

    // J2 Drive Motor Housing (Black cylinder on shoulder side)
    const j2MotorGeo = new THREE.CylinderGeometry(0.10, 0.10, 0.18, 24);
    j2MotorGeo.rotateX(Math.PI / 2);
    const j2Motor = new THREE.Mesh(j2MotorGeo, this.matCastDark);
    j2Motor.position.set(-0.10, 0.12, 0.18);
    j2Motor.castShadow = true;
    shoulderGroup.add(j2Motor);

    // FANUC Brand Plate on Shoulder
    const brandPlateGeo = new THREE.BoxGeometry(0.16, 0.05, 0.01);
    const brandMat = new THREE.MeshBasicMaterial({ color: 0x111827 });
    const brandPlate = new THREE.Mesh(brandPlateGeo, brandMat);
    brandPlate.position.set(0.141, 0.05, 0);
    shoulderGroup.add(brandPlate);

    this.j1Group.add(shoulderGroup);
    this.frame0.add(this.j1Group);

    // =========================================================================
    // FRAME {2}: JOINT 2 (L - Lower Arm Pitch around Z)
    // Link Length a2 = 0.74m (Large robust industrial arm)
    // =========================================================================
    this.link2Length = 0.74;
    this.j2Group = new THREE.Group();
    this.j2Group.position.set(0, 0.30, 0); // Shoulder pivot axis
    this.j2Group.name = "Joint2_L_LowerArm";

    // Shoulder Pivot Bearing Caps
    const shoulderBearingGeo = new THREE.CylinderGeometry(0.12, 0.12, 0.32, 24);
    shoulderBearingGeo.rotateX(Math.PI / 2);
    const shoulderBearing = new THREE.Mesh(shoulderBearingGeo, this.matJointSteel);
    shoulderBearing.castShadow = true;
    this.j2Group.add(shoulderBearing);

    // Lower Arm Casting (Tapered, sculpted structure)
    const lArmGroup = new THREE.Group();

    // Main Tapered Bicep Casting
    const bicepGeo = new THREE.BoxGeometry(0.18, this.link2Length, 0.18);
    bicepGeo.translate(0, this.link2Length / 2, 0);
    const bicepMesh = new THREE.Mesh(bicepGeo, this.matFanucYellow);
    bicepMesh.castShadow = true;
    lArmGroup.add(bicepMesh);

    // Outer reinforcement ribs along the arm
    for (let sign of [-1, 1]) {
      const ribGeo = new THREE.BoxGeometry(0.04, this.link2Length * 0.7, 0.03);
      ribGeo.translate(0, this.link2Length * 0.45, sign * 0.095);
      const ribMesh = new THREE.Mesh(ribGeo, this.matFanucYellow);
      ribMesh.castShadow = true;
      lArmGroup.add(ribMesh);
    }

    // Counterbalance / Spring cylinder accent (Characteristic on FANUC packaging arms)
    const cbCylGeo = new THREE.CylinderGeometry(0.045, 0.045, this.link2Length * 0.55, 16);
    const cbCyl = new THREE.Mesh(cbCylGeo, this.matCastDark);
    cbCyl.position.set(-0.11, this.link2Length * 0.45, 0);
    cbCyl.castShadow = true;
    lArmGroup.add(cbCyl);

    const cbRodGeo = new THREE.CylinderGeometry(0.025, 0.025, this.link2Length * 0.45, 16);
    const cbRod = new THREE.Mesh(cbRodGeo, this.matChrome);
    cbRod.position.set(-0.11, this.link2Length * 0.25, 0);
    lArmGroup.add(cbRod);

    // Cable bracket clamps along arm
    for (let yPos of [this.link2Length * 0.3, this.link2Length * 0.7]) {
      const bracket = new THREE.Mesh(new THREE.BoxGeometry(0.06, 0.03, 0.22), this.matCastDark);
      bracket.position.set(0.09, yPos, 0);
      lArmGroup.add(bracket);
    }

    this.j2Group.add(lArmGroup);
    this.j1Group.add(this.j2Group);

    // =========================================================================
    // FRAME {3}: JOINT 3 (U - Upper Arm Pitch around Z)
    // Link Length a3 = 0.70m
    // =========================================================================
    this.link3Length = 0.70;
    this.j3Group = new THREE.Group();
    this.j3Group.position.set(0, this.link2Length, 0); // Elbow pivot point
    this.j3Group.name = "Joint3_U_UpperArm";

    // Elbow Cast Housing
    const elbowHubGeo = new THREE.CylinderGeometry(0.12, 0.12, 0.28, 24);
    elbowHubGeo.rotateX(Math.PI / 2);
    const elbowHub = new THREE.Mesh(elbowHubGeo, this.matJointSteel);
    elbowHub.castShadow = true;
    this.j3Group.add(elbowHub);

    // J3 Servo Motor mounted on elbow rear
    const j3MotorGeo = new THREE.CylinderGeometry(0.09, 0.09, 0.16, 20);
    j3MotorGeo.rotateZ(Math.PI / 2);
    const j3Motor = new THREE.Mesh(j3MotorGeo, this.matCastDark);
    j3Motor.position.set(-0.14, 0, 0);
    j3Motor.castShadow = true;
    this.j3Group.add(j3Motor);

    // Upper Arm Forearm Boom (Tapered casting extending along X)
    const uArmGroup = new THREE.Group();

    // Contoured Boom
    const boomGeo = new THREE.BoxGeometry(this.link3Length, 0.16, 0.16);
    boomGeo.translate(this.link3Length / 2, 0, 0);
    const boomMesh = new THREE.Mesh(boomGeo, this.matFanucYellow);
    boomMesh.castShadow = true;
    uArmGroup.add(boomMesh);

    // Tapered end towards wrist
    const taperGeo = new THREE.ConeGeometry(0.11, 0.22, 20);
    taperGeo.rotateZ(-Math.PI / 2);
    taperGeo.translate(this.link3Length - 0.05, 0, 0);
    const taperMesh = new THREE.Mesh(taperGeo, this.matFanucYellow);
    uArmGroup.add(taperMesh);

    this.j3Group.add(uArmGroup);
    this.j2Group.add(this.j3Group);

    // =========================================================================
    // FRAME {4}: JOINT 4 (R - Forearm Roll around X)
    // =========================================================================
    this.j4Group = new THREE.Group();
    this.j4Group.position.set(this.link3Length, 0, 0);
    this.j4Group.name = "Joint4_R_ForearmRoll";

    const j4BarrelGeo = new THREE.CylinderGeometry(0.09, 0.09, 0.14, 24);
    j4BarrelGeo.rotateZ(Math.PI / 2);
    const j4Barrel = new THREE.Mesh(j4BarrelGeo, this.matCastDark);
    j4Barrel.position.x = 0.07;
    j4Barrel.castShadow = true;
    this.j4Group.add(j4Barrel);

    this.j3Group.add(this.j4Group);

    // =========================================================================
    // FRAME {5}: JOINT 5 (B - Spherical Wrist Bend / Pitch around Z)
    // Center of Spherical Wrist (Axes 4, 5, 6 intersect here!)
    // =========================================================================
    this.j5Group = new THREE.Group();
    this.j5Group.position.set(0.14, 0, 0); // Wrist center
    this.j5Group.name = "Joint5_B_SphericalWrist";

    // Sculpted Spherical Wrist Yoke
    const wristSphGeo = new THREE.SphereGeometry(0.095, 24, 24);
    const wristSph = new THREE.Mesh(wristSphGeo, this.matJointSteel);
    wristSph.castShadow = true;
    this.j5Group.add(wristSph);

    // Cast Yoke Fork (Yellow)
    const yokeGeo = new THREE.CylinderGeometry(0.08, 0.09, 0.12, 20);
    yokeGeo.rotateZ(Math.PI / 2);
    const yoke = new THREE.Mesh(yokeGeo, this.matFanucYellow);
    yoke.position.x = 0.05;
    yoke.castShadow = true;
    this.j5Group.add(yoke);

    this.j4Group.add(this.j5Group);

    // =========================================================================
    // FRAME {6}: JOINT 6 (T - Flange Twist around X) & Large Vacuum Gripper
    // =========================================================================
    this.j6Group = new THREE.Group();
    this.j6Group.position.set(0.11, 0, 0); // Tool mounting flange
    this.j6Group.name = "Joint6_T_ToolFlange";

    const flangeGeo = new THREE.CylinderGeometry(0.08, 0.08, 0.03, 32);
    flangeGeo.rotateZ(Math.PI / 2);
    const flange = new THREE.Mesh(flangeGeo, this.matChrome);
    flange.castShadow = true;
    this.j6Group.add(flange);

    // Build the authentic packaging vacuum gripper
    this.buildVacuumGripper();

    this.j5Group.add(this.j6Group);
    this.root.add(this.frame0);

    // Build flexible black corrugated cables along robot body
    this.buildCableDresspack();
  }

  buildVacuumGripper() {
    this.gripperGroup = new THREE.Group();
    this.gripperGroup.name = "PackagingVacuumGripper_Large";
    this.gripperGroup.position.set(0.02, 0, 0);

    // Main Anodized Aluminum Plenum Chamber (Large box matching video)
    // Sized to grip a full collated pack (0.24m x 0.20m x 0.09m)
    const boxGeo = new THREE.BoxGeometry(0.10, 0.24, 0.20);
    boxGeo.translate(0.05, 0, 0);
    const boxMesh = new THREE.Mesh(boxGeo, this.matGripperWhite);
    boxMesh.castShadow = true;
    boxMesh.receiveShadow = true;
    this.gripperGroup.add(boxMesh);

    // Pneumatic solenoid valve block on top
    const valveGeo = new THREE.BoxGeometry(0.06, 0.08, 0.08);
    valveGeo.translate(0.04, 0.14, 0);
    const valveMesh = new THREE.Mesh(valveGeo, this.matCastDark);
    this.gripperGroup.add(valveMesh);

    // Illuminated digital vacuum display
    const ledDispGeo = new THREE.PlaneGeometry(0.035, 0.018);
    const ledDispMat = new THREE.MeshBasicMaterial({ color: 0x22c55e });
    const ledDisp = new THREE.Mesh(ledDispGeo, ledDispMat);
    ledDisp.position.set(0.04, 0.14, 0.041);
    this.gripperGroup.add(ledDisp);

    // Coiled blue pneumatic air line
    const coilGeo = new THREE.TorusGeometry(0.025, 0.006, 8, 20);
    const coilMat = new THREE.MeshStandardMaterial({ color: 0x0284c7 });
    const coil = new THREE.Mesh(coilGeo, coilMat);
    coil.position.set(0.02, 0.16, -0.05);
    coil.rotation.y = Math.PI / 2;
    this.gripperGroup.add(coil);

    // 8 Spring-loaded Silicone Suction Cups arranged in a 4x2 matrix
    this.suctionCups = [];
    const cupRows = [-0.075, -0.025, 0.025, 0.075];
    const cupCols = [-0.05, 0.05];

    cupRows.forEach(ry => {
      cupCols.forEach(rz => {
        const cupUnit = new THREE.Group();
        cupUnit.position.set(0.10, ry, rz);

        // Spring compensator shaft
        const springGeo = new THREE.CylinderGeometry(0.008, 0.008, 0.035, 12);
        springGeo.rotateZ(Math.PI / 2);
        const spring = new THREE.Mesh(springGeo, this.matChrome);
        cupUnit.add(spring);

        // Silicone bellows suction cup
        const cupGeo = new THREE.CylinderGeometry(0.024, 0.016, 0.028, 16);
        cupGeo.rotateZ(Math.PI / 2);
        const cup = new THREE.Mesh(cupGeo, this.matSuctionCups);
        cup.position.x = 0.025;
        cupUnit.add(cup);

        this.gripperGroup.add(cupUnit);
        this.suctionCups.push(cupUnit);
      });
    });

    // Tool Center Point (TCP) at vacuum suction cup face
    this.tcpMarker = new THREE.Group();
    this.tcpMarker.position.set(0.145, 0, 0);
    this.gripperGroup.add(this.tcpMarker);

    this.j6Group.add(this.gripperGroup);
  }

  buildCableDresspack() {
    // Black corrugated flexible conduit running along lower arm
    const conduitGeo1 = new THREE.CylinderGeometry(0.022, 0.022, this.link2Length * 0.75, 12);
    conduitGeo1.translate(0.11, this.link2Length * 0.5, 0.10);
    const conduit1 = new THREE.Mesh(conduitGeo1, this.matConduit);
    this.j2Group.add(conduit1);

    // Black conduit along upper arm
    const conduitGeo2 = new THREE.CylinderGeometry(0.020, 0.020, this.link3Length * 0.75, 12);
    conduitGeo2.rotateZ(Math.PI / 2);
    conduitGeo2.translate(this.link3Length * 0.5, 0.10, 0.09);
    const conduit2 = new THREE.Mesh(conduitGeo2, this.matConduit);
    this.j3Group.add(conduit2);
  }

  buildDHFrames() {
    this.dhFramesGroup = new THREE.Group();
    this.dhFramesGroup.name = "DHFramesGroup";

    const frameDefs = [
      { parent: this.frame0, name: "{0}", offset: new THREE.Vector3(0, 0.06, 0) },
      { parent: this.j1Group, name: "{1}", offset: new THREE.Vector3(0, 0.12, 0) },
      { parent: this.j2Group, name: "{2}", offset: new THREE.Vector3(0, 0, 0.18) },
      { parent: this.j3Group, name: "{3}", offset: new THREE.Vector3(0, 0, 0.18) },
      { parent: this.j4Group, name: "{4}", offset: new THREE.Vector3(0.08, 0.12, 0) },
      { parent: this.j5Group, name: "{5}", offset: new THREE.Vector3(0, 0, 0.12) },
      { parent: this.j6Group, name: "{6}", offset: new THREE.Vector3(0.15, 0, 0) }
    ];

    frameDefs.forEach(def => {
      const frameObj = new THREE.Group();
      frameObj.position.copy(def.offset);

      const axes = new THREE.AxesHelper(0.22);
      axes.renderOrder = 999;
      axes.material.depthTest = false;
      axes.material.linewidth = 2;
      frameObj.add(axes);

      const label = this.createLabelSprite(def.name);
      label.position.set(0.10, 0.10, 0.10);
      label.renderOrder = 1000;
      frameObj.add(label);

      def.parent.add(frameObj);
      this.dhHelpers.push(frameObj);
    });

    this.root.add(this.dhFramesGroup);
  }

  buildFloorDropLine() {
    this.dropLineGroup = new THREE.Group();
    this.dropLineGroup.name = "FloorProjectionDropLine";

    // Pre-allocated static buffer to avoid GC frame drops
    this.dropLinePositions = new Float32Array(6);
    const lineGeo = new THREE.BufferGeometry();
    lineGeo.setAttribute('position', new THREE.BufferAttribute(this.dropLinePositions, 3));

    this.dropLineMat = new THREE.LineDashedMaterial({
      color: 0x06b6d4,
      dashSize: 0.05,
      gapSize: 0.025,
      linewidth: 2,
      depthTest: false,
      transparent: true,
      opacity: 0.85
    });

    this.dropLine = new THREE.Line(lineGeo, this.dropLineMat);
    this.dropLine.renderOrder = 998;
    this.dropLineGroup.add(this.dropLine);

    // Floor Target Crosshair Ring
    const ringGeo = new THREE.RingGeometry(0.08, 0.11, 32);
    ringGeo.rotateX(-Math.PI / 2);
    const ringMat = new THREE.MeshBasicMaterial({
      color: 0x06b6d4,
      side: THREE.DoubleSide,
      transparent: true,
      opacity: 0.8,
      depthTest: false
    });
    this.targetRing = new THREE.Mesh(ringGeo, ringMat);
    this.targetRing.renderOrder = 998;
    this.dropLineGroup.add(this.targetRing);

    // Floor coordinate label sprite
    this.floorCoordSprite = this.createLabelSprite("TCP: [0, 0, 0]");
    this.floorCoordSprite.scale.set(0.45, 0.22, 1);
    this.floorCoordSprite.renderOrder = 1000;
    this.dropLineGroup.add(this.floorCoordSprite);

    this.root.add(this.dropLineGroup);
  }

  setJointAngles(sDeg, lDeg, uDeg, rDeg, bDeg, tDeg) {
    if (sDeg !== undefined) this.angles.s = sDeg;
    if (lDeg !== undefined) this.angles.l = lDeg;
    if (uDeg !== undefined) this.angles.u = uDeg;
    if (rDeg !== undefined) this.angles.r = rDeg;
    if (bDeg !== undefined) this.angles.b = bDeg;
    if (tDeg !== undefined) this.angles.t = tDeg;

    this.updateJoints();
  }

  updateJoints() {
    this.j1Group.rotation.y = THREE.MathUtils.degToRad(this.angles.s);
    this.j2Group.rotation.z = THREE.MathUtils.degToRad(this.angles.l);
    this.j3Group.rotation.z = THREE.MathUtils.degToRad(this.angles.u);
    this.j4Group.rotation.x = THREE.MathUtils.degToRad(this.angles.r);
    this.j5Group.rotation.z = THREE.MathUtils.degToRad(this.angles.b);
    this.j6Group.rotation.x = THREE.MathUtils.degToRad(this.angles.t);

    this.updateDropLine();
  }

  updateDropLine() {
    if (!this.dhVisible || !this.tcpMarker) return;

    this.root.updateMatrixWorld(true);
    const tcpWorld = new THREE.Vector3();
    this.tcpMarker.getWorldPosition(tcpWorld);

    const localTop = this.dropLineGroup.worldToLocal(tcpWorld.clone());
    const floorWorld = new THREE.Vector3(tcpWorld.x, 0.01, tcpWorld.z);
    const localBottom = this.dropLineGroup.worldToLocal(floorWorld);

    const pos = this.dropLine.geometry.attributes.position.array;
    pos[0] = localTop.x;
    pos[1] = localTop.y;
    pos[2] = localTop.z;
    pos[3] = localBottom.x;
    pos[4] = localBottom.y;
    pos[5] = localBottom.z;
    this.dropLine.geometry.attributes.position.needsUpdate = true;
    this.dropLine.computeLineDistances();

    this.targetRing.position.set(localBottom.x, localBottom.y, localBottom.z);
    this.floorCoordSprite.position.set(localBottom.x + 0.18, localBottom.y + 0.12, localBottom.z);
  }

  setDHVisible(visible) {
    this.dhVisible = visible;
    this.dhHelpers.forEach(helper => {
      helper.visible = visible;
    });
    this.dropLineGroup.visible = visible;
  }

  getJointAngles() {
    return { ...this.angles };
  }

  getJointPulses() {
    return {
      s: Math.round(this.angles.s * this.pulseRatios.s),
      l: Math.round(this.angles.l * this.pulseRatios.l),
      u: Math.round(this.angles.u * this.pulseRatios.u),
      r: Math.round(this.angles.r * this.pulseRatios.r),
      b: Math.round(this.angles.b * this.pulseRatios.b),
      t: Math.round(this.angles.t * this.pulseRatios.t)
    };
  }

  getTCPWorldPosition() {
    const tcp = new THREE.Vector3();
    if (this.tcpMarker) {
      this.tcpMarker.getWorldPosition(tcp);
    }
    return tcp;
  }
}
