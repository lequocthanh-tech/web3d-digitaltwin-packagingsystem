import * as THREE from 'three';
import { Delta3DOFKinematics } from '../kinematics/Delta3DOFKinematics.js';

/**
 * DeltaRobot3D - Industrial 3-DOF Parallel Delta Robot for High-Speed Pick & Place
 * 
 * Benchmark Specifications (Technical Assignment):
 * - Fixed Upper Base: Radius R_base = 180 mm (0.180 m), 3 servo motors at 0°, 120°, 240°
 * - Active Upper Arm (Biceps): Length L1 = 200 mm (0.200 m)
 * - Passive Lower Forearm (Parallelograms): 3 pairs of parallel carbon fiber rods L2 = 500 mm (0.500 m)
 * - Mobile End-Effector Platform: Radius r_platform = 50 mm (0.050 m) with Vacuum Gripper
 * - Kinematics Engine: Analytical Inverse Kinematics via sphere-plane intersection
 * - Workspace: Z in [-600 mm, -350 mm], R_max = 250 mm
 * - Visual 3D Structure:
 *   * Sturdy 3-legged suspension tripod frame (Khung treo 3 chân vững chắc)
 *   * Stainless steel receiving tray (Khay chứa phôi) at place location
 *   * Smooth articulated arms, pneumatic suction cup with dynamic state glow
 */
export class DeltaRobot3D {
  constructor(options = {}) {
    // Delta base mount position above conveyor table (Y = 1.42 m places workspace at table surface Y = 0.85 m)
    this.basePosition = options.position || new THREE.Vector3(1.2, 1.42, -0.1);
    
    // Benchmark 3-DOF Parallel Delta Kinematic Parameters (in meters)
    this.rBase = options.rBase ?? 0.180;          // Upper Base Radius R_base = 180 mm (0.180 m)
    this.rPlatform = options.rPlatform ?? 0.050;  // Mobile Platform Radius r_platform = 50 mm (0.050 m)
    this.L1 = options.L1 ?? 0.200;                // Active Upper Arm (Biceps) L1 = 200 mm (0.200 m)
    this.L2 = options.L2 ?? 0.500;                // Passive Forearm Parallelogram Rods L2 = 500 mm (0.500 m)
    this.rodWidth = 0.055;                       // Parallelogram rod spacing = 55 mm
    this.tipOffset = 0.109;                      // Suction cup offset below mobile platform = 109 mm (flange 12mm + tool 97mm)
    
    // Kinematics engine instance (Workspace limit: zMin = -445mm prevents collision with conveyor belt at -445.5mm)
    this.kinEngine = new Delta3DOFKinematics({
      R_base: this.rBase * 1000.0,
      r_platform: this.rPlatform * 1000.0,
      L1: this.L1 * 1000.0,
      L2: this.L2 * 1000.0,
      zMin: -445.0,
      zMax: -340.0,
      rMax: 250.0
    });

    // Servo angles (in radians): 1.24° = 0.0216 rad at nominal home (0, 0, -380 mm)
    this.theta = [0.0216, 0.0216, 0.0216]; // M1 (0°), M2 (120°), M3 (240°)
    this.theta4 = 0.0;                     // End-effector Yaw (radians)
    
    // Current Mobile Platform relative position (relative to base center in meters)
    // Note: In Three.js local space, X = kin X, Y = kin Z (vertical negative), Z = kin Y (depth)
    this.platformPos = new THREE.Vector3(0, -0.38, 0);
    this.vacuumActive = false;
    
    // Root group
    this.root = new THREE.Group();
    this.root.name = "FANUC_DeltaRobot3DOF";
    this.root.position.copy(this.basePosition);
    
    this.initMaterials();
    this.coordsVisible = false;

    // 3D Visual assemblies
    this.buildTripodFrame();
    this.buildReceivingTray();
    this.buildBaseAssembly();
    this.buildArmsAndPlatform();
    this.buildCoordHelpers();
    this.setCoordsVisible(false);
    
    // Initial kinematics update
    this.updateKinematicsFromAngles();
  }

  initMaterials() {
    // Signature FANUC Industrial Gloss Yellow
    this.matFanucYellow = new THREE.MeshStandardMaterial({
      color: 0xfed100,
      roughness: 0.26,
      metalness: 0.16
    });

    this.matDarkMetal = new THREE.MeshStandardMaterial({
      color: 0x1e293b,
      roughness: 0.5,
      metalness: 0.7
    });

    this.matSteelGantry = new THREE.MeshStandardMaterial({
      color: 0x0f172a,
      roughness: 0.55,
      metalness: 0.85
    });

    this.matChrome = new THREE.MeshStandardMaterial({
      color: 0xf1f5f9,
      roughness: 0.15,
      metalness: 0.95
    });

    // White/Light Carbon Fiber Rods (matching FANUC white arms in video)
    this.matCarbonRods = new THREE.MeshStandardMaterial({
      color: 0xf8fafc,
      roughness: 0.3,
      metalness: 0.4
    });

    this.matSuctionCup = new THREE.MeshStandardMaterial({
      color: 0x0284c7, // silicone blue
      roughness: 0.5,
      metalness: 0.1
    });

    this.matRubberBoot = new THREE.MeshStandardMaterial({
      color: 0x111827,
      roughness: 0.8,
      metalness: 0.1
    });

    this.matVacuumGlow = new THREE.MeshBasicMaterial({
      color: 0x334155
    });

    this.matInoxTray = new THREE.MeshStandardMaterial({
      color: 0xe2e8f0,
      roughness: 0.25,
      metalness: 0.90
    });

    this.matChocolate = new THREE.MeshStandardMaterial({
      color: 0x3a1d13,
      roughness: 0.45,
      metalness: 0.15
    });

    this.matGoldFoil = new THREE.MeshStandardMaterial({
      color: 0xd97706,
      roughness: 0.3,
      metalness: 0.8
    });
  }

  /**
   * Sturdy 3-Legged Suspension Tripod Frame (Khung treo 3 chân vững chắc)
   * Interleaved at 60°, 180°, 300° to guarantee total clearance for the 3 active arms at 0°, 120°, 240°!
   */
  buildTripodFrame() {
    this.tripodGroup = new THREE.Group();
    this.tripodGroup.name = "DeltaTripodFrame";

    // 1. Heavy Overhead Mounting Crown / Triangular Plate
    const crownTopY = 0.08;
    const crownShape = new THREE.Shape();
    const R_crown = 0.32;
    for (let i = 0; i < 3; i++) {
      const a = (Math.PI / 3) + (i * 2 * Math.PI) / 3;
      const x = R_crown * Math.cos(a);
      const z = R_crown * Math.sin(a);
      if (i === 0) crownShape.moveTo(x, z);
      else crownShape.lineTo(x, z);
    }
    crownShape.closePath();

    const crownExtrude = { depth: 0.035, bevelEnabled: true, bevelSegments: 3, bevelSize: 0.01, bevelThickness: 0.01 };
    const crownGeo = new THREE.ExtrudeGeometry(crownShape, crownExtrude);
    crownGeo.rotateX(Math.PI / 2);
    const crownMesh = new THREE.Mesh(crownGeo, this.matSteelGantry);
    crownMesh.position.y = crownTopY;
    crownMesh.castShadow = true;
    this.tripodGroup.add(crownMesh);

    // Center circular bezel
    const crownBezel = new THREE.Mesh(new THREE.CylinderGeometry(0.25, 0.26, 0.04, 24), this.matDarkMetal);
    crownBezel.position.y = crownTopY + 0.02;
    this.tripodGroup.add(crownBezel);

    // 2. 3 Heavy-Duty Angled Tubular Structural Legs (Interleaved at 60°, 180°, 300°)
    const legAngles = [
      Math.PI / 3,           // 60 deg
      Math.PI,               // 180 deg
      (5 * Math.PI) / 3      // 300 deg
    ];

    const R_top = 0.28;      // Attachment radius at crown
    const R_btm = 0.48;      // Foot radius on table surface
    const Y_top = crownTopY; // Top connection height
    const Y_btm = -0.57;     // Table top surface in local coordinates (world 0.85 m)

    const legFootPositions = [];
    const legMidPositions = [];

    legAngles.forEach((psi, idx) => {
      const topP = new THREE.Vector3(R_top * Math.cos(psi), Y_top, R_top * Math.sin(psi));
      const btmP = new THREE.Vector3(R_btm * Math.cos(psi), Y_btm, R_btm * Math.sin(psi));
      legFootPositions.push(btmP);

      const legVec = new THREE.Vector3().subVectors(btmP, topP);
      const legLen = legVec.length();
      const legMid = new THREE.Vector3().addVectors(topP, btmP).multiplyScalar(0.5);
      legMidPositions.push(new THREE.Vector3(
        R_top * 0.4 + R_btm * 0.6 * Math.cos(psi),
        Y_btm * 0.45,
        R_top * 0.4 + R_btm * 0.6 * Math.sin(psi)
      ));

      // Main structural tubular column (55mm diameter)
      const legGeo = new THREE.CylinderGeometry(0.028, 0.028, legLen, 16);
      const legMesh = new THREE.Mesh(legGeo, this.matSteelGantry);
      legMesh.position.copy(legMid);

      const up = new THREE.Vector3(0, 1, 0);
      const legDir = legVec.clone().normalize();
      legMesh.quaternion.copy(new THREE.Quaternion().setFromUnitVectors(up, legDir));
      legMesh.castShadow = true;
      legMesh.receiveShadow = true;
      this.tripodGroup.add(legMesh);

      // Upper welded flange bracket
      const topFlange = new THREE.Mesh(new THREE.BoxGeometry(0.08, 0.04, 0.08), this.matDarkMetal);
      topFlange.position.copy(topP);
      topFlange.position.y -= 0.015;
      this.tripodGroup.add(topFlange);

      // Lower heavy mounting foot plate bolted to table
      const footPlate = new THREE.Mesh(new THREE.BoxGeometry(0.13, 0.02, 0.13), this.matDarkMetal);
      footPlate.position.set(btmP.x, Y_btm + 0.01, btmP.z);
      this.tripodGroup.add(footPlate);

      // 4 Heavy-duty anchor hex bolts per leg
      for (let bx of [-0.045, 0.045]) {
        for (let bz of [-0.045, 0.045]) {
          const bolt = new THREE.Mesh(new THREE.CylinderGeometry(0.009, 0.009, 0.025, 8), this.matChrome);
          bolt.position.set(btmP.x + bx, Y_btm + 0.025, btmP.z + bz);
          this.tripodGroup.add(bolt);
        }
      }
    });

    // 3. Stiffening Horizontal Struts Linking the 3 Legs (at Y = -0.22 m)
    const strutY = -0.22;
    const R_strut = R_top + (R_btm - R_top) * ((Y_top - strutY) / (Y_top - Y_btm));
    for (let i = 0; i < 3; i++) {
      const a1 = legAngles[i];
      const a2 = legAngles[(i + 1) % 3];
      const p1 = new THREE.Vector3(R_strut * Math.cos(a1), strutY, R_strut * Math.sin(a1));
      const p2 = new THREE.Vector3(R_strut * Math.cos(a2), strutY, R_strut * Math.sin(a2));
      const sVec = new THREE.Vector3().subVectors(p2, p1);
      const sLen = sVec.length();
      const sMid = new THREE.Vector3().addVectors(p1, p2).multiplyScalar(0.5);

      const strutGeo = new THREE.CylinderGeometry(0.014, 0.014, sLen, 12);
      const strutMesh = new THREE.Mesh(strutGeo, this.matDarkMetal);
      strutMesh.position.copy(sMid);
      strutMesh.quaternion.copy(new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), sVec.normalize()));
      this.tripodGroup.add(strutMesh);
    }

    // 4. 3-Color Industrial Andon Stack Light (Green / Amber / Red tower)
    const stackPole = new THREE.Mesh(new THREE.CylinderGeometry(0.012, 0.012, 0.28, 12), this.matDarkMetal);
    stackPole.position.set(0.24, crownTopY + 0.16, 0.24);
    this.tripodGroup.add(stackPole);

    const lightColors = [0x22c55e, 0xfacc15, 0xef4444];
    lightColors.forEach((col, idx) => {
      const seg = new THREE.Mesh(
        new THREE.CylinderGeometry(0.022, 0.022, 0.045, 16),
        new THREE.MeshBasicMaterial({ color: col })
      );
      seg.position.set(0.24, crownTopY + 0.24 + idx * 0.048, 0.24);
      this.tripodGroup.add(seg);
    });

    // 5. Pneumatic Filter-Regulator-Gauge Unit (FRL) with blue air tubing
    const frlBlock = new THREE.Mesh(new THREE.BoxGeometry(0.06, 0.09, 0.05), this.matDarkMetal);
    frlBlock.position.set(-0.25, -0.15, 0);
    const gaugeFace = new THREE.Mesh(new THREE.CylinderGeometry(0.025, 0.025, 0.015, 16), this.matChrome);
    gaugeFace.rotateZ(Math.PI / 2);
    gaugeFace.position.set(-0.285, -0.15, 0);
    this.tripodGroup.add(frlBlock, gaugeFace);

    this.root.add(this.tripodGroup);
  }

  // Backwards compatibility alias
  buildGantry() {
    this.buildTripodFrame();
  }

  /**
   * Stainless Steel Receiving Tray (Khay chứa phôi để robot gắp thả)
   * Placed on the table at Place position: X = +120 mm (+0.12 m), Y = +100 mm (+0.10 m), Z_table = -0.57 m
   */
  buildReceivingTray() {
    this.receivingTrayGroup = new THREE.Group();
    this.receivingTrayGroup.name = "DeltaReceivingTray";

    const trayX = 0.12;   // Matches PLACE X coordinate (+120 mm)
    const trayZ = 0.10;   // Matches PLACE Y coordinate in Three.js Z (+100 mm)
    const trayY = -0.565; // Directly on conveyor table top

    this.receivingTrayGroup.position.set(trayX, trayY, trayZ);

    const trayW = 0.24;   // Width along X (240 mm)
    const trayD = 0.17;   // Depth along Z (170 mm)
    const trayH = 0.035;  // Height (35 mm)
    const wallThick = 0.008;

    // 1. Tray Bottom Base Plate
    const basePlate = new THREE.Mesh(
      new THREE.BoxGeometry(trayW, 0.01, trayD),
      this.matInoxTray
    );
    basePlate.position.y = 0.005;
    basePlate.castShadow = true;
    basePlate.receiveShadow = true;
    this.receivingTrayGroup.add(basePlate);

    // 2. 4 Perimeter Rims
    const rimFront = new THREE.Mesh(new THREE.BoxGeometry(trayW, trayH, wallThick), this.matInoxTray);
    rimFront.position.set(0, trayH / 2, trayD / 2 - wallThick / 2);
    const rimBack = new THREE.Mesh(new THREE.BoxGeometry(trayW, trayH, wallThick), this.matInoxTray);
    rimBack.position.set(0, trayH / 2, -trayD / 2 + wallThick / 2);
    const rimLeft = new THREE.Mesh(new THREE.BoxGeometry(wallThick, trayH, trayD), this.matInoxTray);
    rimLeft.position.set(-trayW / 2 + wallThick / 2, trayH / 2, 0);
    const rimRight = new THREE.Mesh(new THREE.BoxGeometry(wallThick, trayH, trayD), this.matInoxTray);
    rimRight.position.set(trayW / 2 - wallThick / 2, trayH / 2, 0);

    this.receivingTrayGroup.add(rimFront, rimBack, rimLeft, rimRight);

    // 3. Internal Dividers creating a 2x2 matrix of pocket cavities
    const divLong = new THREE.Mesh(new THREE.BoxGeometry(trayW - wallThick * 2, trayH * 0.7, 0.004), this.matDarkMetal);
    divLong.position.set(0, (trayH * 0.7) / 2 + 0.005, 0);
    const divCross = new THREE.Mesh(new THREE.BoxGeometry(0.004, trayH * 0.7, trayD - wallThick * 2), this.matDarkMetal);
    divCross.position.set(0, (trayH * 0.7) / 2 + 0.005, 0);
    this.receivingTrayGroup.add(divLong, divCross);

    // 4. Sample Chocolate Bars already placed inside the tray (indicating active production)
    const barGeo = new THREE.BoxGeometry(0.095, 0.018, 0.055);
    const foilGeo = new THREE.BoxGeometry(0.097, 0.019, 0.022);

    // Pre-placed bar in back-left pocket
    const bar1Group = new THREE.Group();
    const bar1Mesh = new THREE.Mesh(barGeo, this.matChocolate);
    const foil1Mesh = new THREE.Mesh(foilGeo, this.matGoldFoil);
    bar1Group.add(bar1Mesh, foil1Mesh);
    bar1Group.position.set(-0.055, 0.015, -0.040);
    bar1Group.castShadow = true;

    // Pre-placed bar in back-right pocket
    const bar2Group = new THREE.Group();
    const bar2Mesh = new THREE.Mesh(barGeo, this.matChocolate);
    const foil2Mesh = new THREE.Mesh(foilGeo, this.matGoldFoil);
    bar2Group.add(bar2Mesh, foil2Mesh);
    bar2Group.position.set(0.055, 0.015, -0.040);
    bar2Group.castShadow = true;

    this.receivingTrayGroup.add(bar1Group, bar2Group);

    // 5. Yellow/Black Safety Striping along bottom rim
    const stripGeo = new THREE.BoxGeometry(trayW + 0.01, 0.004, trayD + 0.01);
    const stripMat = new THREE.MeshBasicMaterial({ color: 0xf59e0b });
    const safetyStripe = new THREE.Mesh(stripGeo, stripMat);
    safetyStripe.position.y = 0.002;
    this.receivingTrayGroup.add(safetyStripe);

    this.root.add(this.receivingTrayGroup);
  }

  buildBaseAssembly() {
    this.baseAssembly = new THREE.Group();
    this.baseAssembly.name = "FANUC_DeltaBaseDome";

    // Sculpted FANUC Dome Base (Convex rounded industrial yellow housing)
    const domeShape = new THREE.CylinderGeometry(this.rBase + 0.06, this.rBase + 0.12, 0.12, 32);
    const dome = new THREE.Mesh(domeShape, this.matFanucYellow);
    dome.position.y = 0.02;
    dome.castShadow = true;
    dome.receiveShadow = true;
    this.baseAssembly.add(dome);

    // Center circular top cover
    const topCap = new THREE.Mesh(new THREE.CylinderGeometry(0.18, 0.20, 0.035, 24), this.matDarkMetal);
    topCap.position.y = 0.09;
    this.baseAssembly.add(topCap);

    // 4th Axis Rotary Spline Drive Motor
    const motor4 = new THREE.Mesh(new THREE.CylinderGeometry(0.045, 0.045, 0.13, 16), this.matDarkMetal);
    motor4.position.y = 0.16;
    this.baseAssembly.add(motor4);

    // 3 Cycloidal Servo Motor Pods (spaced 120 deg apart at 0°, 120°, 240°)
    this.motorHousings = [];
    for (let i = 0; i < 3; i++) {
      const phi = (i * 2 * Math.PI) / 3;
      const motorGroup = new THREE.Group();
      motorGroup.position.set(
        (this.rBase + 0.02) * Math.cos(phi),
        0.015,
        (this.rBase + 0.02) * Math.sin(phi)
      );
      motorGroup.rotation.y = -phi + Math.PI / 2;

      // Servo cylinder body
      const mBodyGeo = new THREE.CylinderGeometry(0.055, 0.055, 0.15, 20);
      mBodyGeo.rotateZ(Math.PI / 2);
      const mBody = new THREE.Mesh(mBodyGeo, this.matDarkMetal);
      mBody.castShadow = true;
      motorGroup.add(mBody);

      // Gearbox machined flange
      const mFlangeGeo = new THREE.CylinderGeometry(0.065, 0.065, 0.035, 20);
      mFlangeGeo.rotateZ(Math.PI / 2);
      const mFlange = new THREE.Mesh(mFlangeGeo, this.matChrome);
      mFlange.position.x = 0.085;
      motorGroup.add(mFlange);

      this.baseAssembly.add(motorGroup);
      this.motorHousings.push(motorGroup);
    }

    this.root.add(this.baseAssembly);
  }

  buildArmsAndPlatform() {
    this.arms = [];

    for (let i = 0; i < 3; i++) {
      const phi = (i * 2 * Math.PI) / 3;
      const armData = {
        phi: phi,
        upperPivotGroup: new THREE.Group(),
        upperArmMesh: null,
        parallelogramGroup: new THREE.Group()
      };

      const bx = this.rBase * Math.cos(phi);
      const bz = this.rBase * Math.sin(phi);
      armData.upperPivotGroup.position.set(bx, 0, bz);
      armData.upperPivotGroup.rotation.y = -phi; // Points along local X

      // ========================================
      // Sculpted Upper Arm (Bicep) L1 = 200 mm
      // ========================================
      const bicepGroup = new THREE.Group();
      
      // Aerodynamic tapered casting in FANUC Yellow
      const bicepGeo = new THREE.BoxGeometry(this.L1, 0.040, 0.065);
      bicepGeo.translate(this.L1 / 2, 0, 0);
      const bicepMesh = new THREE.Mesh(bicepGeo, this.matFanucYellow);
      bicepMesh.castShadow = true;
      bicepGroup.add(bicepMesh);

      // Chamfered arm boss
      const bicepBoss = new THREE.Mesh(new THREE.CylinderGeometry(0.045, 0.045, 0.07, 16), this.matFanucYellow);
      bicepBoss.rotateX(Math.PI / 2);
      bicepGroup.add(bicepBoss);

      // Elbow cross shaft
      const elbowShaftGeo = new THREE.CylinderGeometry(0.016, 0.016, this.rodWidth + 0.04, 16);
      elbowShaftGeo.rotateX(Math.PI / 2);
      const elbowShaft = new THREE.Mesh(elbowShaftGeo, this.matChrome);
      elbowShaft.position.x = this.L1;
      bicepGroup.add(elbowShaft);

      // Precision dual ball joint balls
      for (let sign of [-1, 1]) {
        const ball = new THREE.Mesh(new THREE.SphereGeometry(0.020, 16, 16), this.matChrome);
        ball.position.set(this.L1, 0, (sign * this.rodWidth) / 2);
        bicepGroup.add(ball);
      }

      armData.upperArmMesh = bicepGroup;
      armData.upperPivotGroup.add(bicepGroup);
      this.root.add(armData.upperPivotGroup);

      // ========================================
      // Parallelogram Lower Arm: L2 = 500 mm (White Carbon Fiber Rods)
      // ========================================
      const paraGroup = new THREE.Group();
      paraGroup.name = `Parallelogram_Arm_${i}`;

      const rodGeo = new THREE.CylinderGeometry(0.008, 0.008, this.L2, 12);
      rodGeo.translate(0, this.L2 / 2, 0);

      for (let sign of [-1, 1]) {
        const rodSub = new THREE.Group();
        rodSub.position.z = (sign * this.rodWidth) / 2;

        // White/Silver rod tube
        const rodMesh = new THREE.Mesh(rodGeo, this.matCarbonRods);
        rodMesh.castShadow = true;
        rodSub.add(rodMesh);

        // Metallic spherical socket ends
        const topEnd = new THREE.Mesh(new THREE.SphereGeometry(0.013, 12, 12), this.matChrome);
        topEnd.position.y = 0.01;
        const btmEnd = new THREE.Mesh(new THREE.SphereGeometry(0.013, 12, 12), this.matChrome);
        btmEnd.position.y = this.L2 - 0.01;
        rodSub.add(topEnd, btmEnd);

        paraGroup.add(rodSub);
      }

      // Hallmark FANUC Retention Springs between the 2 parallel rods
      for (let yFrac of [0.35, 0.65]) {
        const springGeo = new THREE.CylinderGeometry(0.005, 0.005, this.rodWidth, 8);
        springGeo.rotateX(Math.PI / 2);
        const springMesh = new THREE.Mesh(springGeo, this.matChrome);
        springMesh.position.y = this.L2 * yFrac;
        paraGroup.add(springMesh);
      }

      armData.parallelogramGroup = paraGroup;
      this.root.add(paraGroup);
      this.arms.push(armData);
    }

    // ==========================================
    // Mobile Platform (End-Effector Platform) r_platform = 50 mm
    // ==========================================
    this.platformGroup = new THREE.Group();
    this.platformGroup.name = "FANUC_DeltaMobilePlatform";

    const platShape = new THREE.Shape();
    const Rp = this.rPlatform + 0.028;
    for (let i = 0; i < 3; i++) {
      const angle = (i * 2 * Math.PI) / 3;
      const x = Rp * Math.cos(angle);
      const z = Rp * Math.sin(angle);
      if (i === 0) platShape.moveTo(x, z);
      else platShape.lineTo(x, z);
    }
    platShape.closePath();

    const platExtrude = { depth: 0.022, bevelEnabled: true, bevelSize: 0.008, bevelThickness: 0.008 };
    const platGeo = new THREE.ExtrudeGeometry(platShape, platExtrude);
    platGeo.rotateX(Math.PI / 2);
    const platMesh = new THREE.Mesh(platGeo, this.matFanucYellow);
    platMesh.castShadow = true;
    this.platformGroup.add(platMesh);

    // Platform ball joints
    for (let i = 0; i < 3; i++) {
      const phi = (i * 2 * Math.PI) / 3;
      const px = this.rPlatform * Math.cos(phi);
      const pz = this.rPlatform * Math.sin(phi);

      const crossShaft = new THREE.Mesh(
        new THREE.CylinderGeometry(0.013, 0.013, this.rodWidth, 12),
        this.matChrome
      );
      crossShaft.position.set(px, 0.016, pz);
      crossShaft.rotation.y = -phi;
      crossShaft.rotateX(Math.PI / 2);
      this.platformGroup.add(crossShaft);
    }

    // Tool Flange & High-Speed Single Suction Tool
    this.toolFlange = new THREE.Group();
    this.toolFlange.position.y = -0.012;

    const spindle = new THREE.Mesh(new THREE.CylinderGeometry(0.022, 0.022, 0.065, 20), this.matChrome);
    spindle.position.y = -0.032;
    this.toolFlange.add(spindle);

    // Rubber Accordion Bellows
    const bootGeo = new THREE.CylinderGeometry(0.030, 0.026, 0.030, 16);
    const boot = new THREE.Mesh(bootGeo, this.matRubberBoot);
    boot.position.y = -0.068;

    // Silicone Suction Cup Lip
    const cupLip = new THREE.Mesh(new THREE.CylinderGeometry(0.044, 0.034, 0.018, 20), this.matSuctionCup);
    cupLip.position.y = -0.088;

    // Active Vacuum Glow Ring Indicator
    const vacRingGeo = new THREE.TorusGeometry(0.040, 0.004, 8, 24);
    vacRingGeo.rotateX(Math.PI / 2);
    this.vacRing = new THREE.Mesh(vacRingGeo, this.matVacuumGlow);
    this.vacRing.position.y = -0.088;

    this.toolFlange.add(boot, cupLip, this.vacRing);
    this.platformGroup.add(this.toolFlange);
    this.root.add(this.platformGroup);

    // Central Telescopic 4th Axis Spline Shaft (connects base dome motor to mobile platform)
    this.telescopicGroup = new THREE.Group();
    this.telescopicGroup.name = "DeltaCentralSplineShaft";

    // Standardized unit-length geometries (length = 1.0, translated from 0 down to -1.0)
    // This allows scale.y to directly define exact length in meters without protruding below platform
    const upperShaftGeo = new THREE.CylinderGeometry(0.016, 0.016, 1.0, 16);
    upperShaftGeo.translate(0, -0.5, 0);
    this.telescopicUpper = new THREE.Mesh(upperShaftGeo, this.matDarkMetal);

    const lowerShaftGeo = new THREE.CylinderGeometry(0.012, 0.012, 1.0, 16);
    lowerShaftGeo.translate(0, -0.5, 0);
    this.telescopicLower = new THREE.Mesh(lowerShaftGeo, this.matChrome);

    // Ball/U-joint coupler collar sitting right at the mobile platform interface
    const platCouplerGeo = new THREE.CylinderGeometry(0.018, 0.018, 0.02, 16);
    this.telescopicCoupler = new THREE.Mesh(platCouplerGeo, this.matDarkMetal);

    this.telescopicGroup.add(this.telescopicUpper, this.telescopicLower, this.telescopicCoupler);
    this.root.add(this.telescopicGroup);
  }

  /**
   * Directly sets the 3 servo joint angles in degrees
   */
  setJointAngles(t1Deg, t2Deg, t3Deg, yawDeg = null) {
    this.theta[0] = THREE.MathUtils.degToRad(t1Deg);
    this.theta[1] = THREE.MathUtils.degToRad(t2Deg);
    this.theta[2] = THREE.MathUtils.degToRad(t3Deg);

    if (yawDeg !== null) {
      this.theta4 = THREE.MathUtils.degToRad(yawDeg);
    }

    this.solveForwardKinematics();
    this.updateMeshTransforms();
  }

  /**
   * High-Level Cartesian Positioning for Teaching Pendant & Auto Cycle
   * @param {number} x_mm - Cartesian X in mm
   * @param {number} y_mm - Cartesian Y in mm
   * @param {number} z_mm - Cartesian Z in mm (negative downward, e.g. -450 mm)
   * @param {number} yawDeg - End-effector yaw angle in degrees
   */
  setDeltaCartesian(x_mm, y_mm, z_mm, yawDeg = 0) {
    // In Three.js: X is horizontal, Y is vertical (downward negative), Z is depth
    this.platformPos.set(x_mm * 0.001, z_mm * 0.001, y_mm * 0.001);
    this.theta4 = THREE.MathUtils.degToRad(yawDeg);

    if (this.kinEngine) {
      const ik = this.kinEngine.inverseKinematics(x_mm, y_mm, z_mm);
      if (ik.success) {
        this.theta[0] = THREE.MathUtils.degToRad(ik.thetaDeg.theta1);
        this.theta[1] = THREE.MathUtils.degToRad(ik.thetaDeg.theta2);
        this.theta[2] = THREE.MathUtils.degToRad(ik.thetaDeg.theta3);
      }
    }
    this.updateMeshTransforms();
  }

  /**
   * Returns current Cartesian coordinates in millimeters (Delta frame)
   */
  getDeltaCartesian() {
    return {
      x: Number((this.platformPos.x * 1000.0).toFixed(2)),
      y: Number((this.platformPos.z * 1000.0).toFixed(2)),
      z: Number((this.platformPos.y * 1000.0).toFixed(2))
    };
  }

  /**
   * Visual toggle for vacuum gripper suction
   * @param {boolean} active 
   */
  setVacuumState(active) {
    this.vacuumActive = Boolean(active);
    if (this.vacRing) {
      this.vacRing.material.color.setHex(this.vacuumActive ? 0x22c55e : 0x334155);
    }
    if (this.matSuctionCup) {
      this.matSuctionCup.color.setHex(this.vacuumActive ? 0x06b6d4 : 0x0284c7);
    }
  }

  updateKinematicsFromAngles() {
    this.solveForwardKinematics();
    this.updateMeshTransforms();
  }

  /**
   * Solves Forward Kinematics (FK) using the analytical engine or fallback trilateration
   */
  solveForwardKinematics() {
    if (this.kinEngine) {
      const t1 = THREE.MathUtils.radToDeg(this.theta[0]);
      const t2 = THREE.MathUtils.radToDeg(this.theta[1]);
      const t3 = THREE.MathUtils.radToDeg(this.theta[2]);
      const fk = this.kinEngine.forwardKinematics(t1, t2, t3);
      if (fk.success) {
        this.platformPos.set(fk.x * 0.001, fk.z * 0.001, fk.y * 0.001);
        return;
      }
    }

    // Fallback closed-form trilateration
    const C = [];
    const r_diff = this.rBase - this.rPlatform;

    for (let i = 0; i < 3; i++) {
      const phi = (i * 2 * Math.PI) / 3;
      const th = this.theta[i];

      const cx = (r_diff + this.L1 * Math.cos(th)) * Math.cos(phi);
      const cy = -this.L1 * Math.sin(th);
      const cz = (r_diff + this.L1 * Math.cos(th)) * Math.sin(phi);

      C.push(new THREE.Vector3(cx, cy, cz));
    }

    const p1 = C[0];
    const p2 = C[1];
    const p3 = C[2];
    const r = this.L2;

    const ex = new THREE.Vector3().subVectors(p2, p1).normalize();
    const i_val = ex.dot(new THREE.Vector3().subVectors(p3, p1));
    const ey = new THREE.Vector3().subVectors(p3, p1).sub(ex.clone().multiplyScalar(i_val)).normalize();
    const ez = new THREE.Vector3().crossVectors(ex, ey);
    const d = p1.distanceTo(p2);
    const j_val = ey.dot(new THREE.Vector3().subVectors(p3, p1));

    const x = (r * r - r * r + d * d) / (2 * d);
    const y_coord = (r * r - r * r + i_val * i_val + j_val * j_val) / (2 * j_val) - (i_val / j_val) * x;
    const z_squared = r * r - x * x - y_coord * y_coord;

    if (z_squared >= 0) {
      const z_val = Math.sqrt(z_squared);
      const ptA = p1.clone().add(ex.clone().multiplyScalar(x)).add(ey.clone().multiplyScalar(y_coord)).add(ez.clone().multiplyScalar(z_val));
      const ptB = p1.clone().add(ex.clone().multiplyScalar(x)).add(ey.clone().multiplyScalar(y_coord)).sub(ez.clone().multiplyScalar(z_val));

      const chosen = (ptA.y < ptB.y) ? ptA : ptB;
      this.platformPos.copy(chosen);
    }
  }

  setEndEffectorPose(x, y, z, yawDeg = 0) {
    this.platformPos.set(x, y, z);
    this.theta4 = THREE.MathUtils.degToRad(yawDeg);

    // Convert from Three.js local meters to Delta millimeters:
    // kin X = Three X * 1000, kin Y = Three Z * 1000, kin Z = Three Y * 1000
    const kinX = x * 1000.0;
    const kinY = z * 1000.0;
    const kinZ = y * 1000.0;

    if (this.kinEngine) {
      const ik = this.kinEngine.inverseKinematics(kinX, kinY, kinZ);
      if (ik.success) {
        this.theta[0] = THREE.MathUtils.degToRad(ik.thetaDeg.theta1);
        this.theta[1] = THREE.MathUtils.degToRad(ik.thetaDeg.theta2);
        this.theta[2] = THREE.MathUtils.degToRad(ik.thetaDeg.theta3);
      }
    }
    this.updateMeshTransforms();
  }

  updateMeshTransforms() {
    for (let i = 0; i < 3; i++) {
      const arm = this.arms[i];
      const th = this.theta[i];
      arm.upperArmMesh.rotation.z = -th;
    }

    this.platformGroup.position.copy(this.platformPos);
    this.toolFlange.rotation.y = this.theta4;

    for (let i = 0; i < 3; i++) {
      const arm = this.arms[i];
      const phi = arm.phi;
      const th = this.theta[i];

      const elbowX = (this.rBase + this.L1 * Math.cos(th)) * Math.cos(phi);
      const elbowY = -this.L1 * Math.sin(th);
      const elbowZ = (this.rBase + this.L1 * Math.cos(th)) * Math.sin(phi);
      const elbowPos = new THREE.Vector3(elbowX, elbowY, elbowZ);

      const platX = this.platformPos.x + this.rPlatform * Math.cos(phi);
      const platY = this.platformPos.y;
      const platZ = this.platformPos.z + this.rPlatform * Math.sin(phi);
      const platBallPos = new THREE.Vector3(platX, platY, platZ);

      const rodVec = new THREE.Vector3().subVectors(platBallPos, elbowPos);
      const currentLen = rodVec.length();

      arm.parallelogramGroup.position.copy(elbowPos);

      const up = new THREE.Vector3(0, 1, 0);
      const dir = rodVec.clone().normalize();
      const quat = new THREE.Quaternion().setFromUnitVectors(up, dir);
      arm.parallelogramGroup.quaternion.copy(quat);
      arm.parallelogramGroup.scale.set(1, currentLen / this.L2, 1);
    }

    const shaftVec = this.platformPos.clone();
    const shaftLen = shaftVec.length();
    const shaftDir = shaftVec.clone().normalize();
    const shaftQuat = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, -1, 0), shaftDir);

    this.telescopicGroup.position.set(0, 0, 0);
    this.telescopicGroup.quaternion.copy(shaftQuat);

    // Upper sleeve extends from base motor (y = 0) down to 55% of shaft length
    const upperLen = shaftLen * 0.55;
    this.telescopicUpper.position.set(0, 0, 0);
    this.telescopicUpper.scale.set(1, upperLen, 1);

    // Lower spline shaft starts inside upper sleeve (at y = -0.45 * shaftLen)
    // and terminates EXACTLY at the mobile platform top surface (y = -shaftLen)
    // Length: shaftLen - 0.45 * shaftLen = 0.55 * shaftLen
    const lowerLen = shaftLen * 0.55;
    this.telescopicLower.position.set(0, -shaftLen * 0.45, 0);
    this.telescopicLower.scale.set(1, lowerLen, 1);

    // Coupler collar positioned precisely at the mobile platform connection point (y = -shaftLen)
    this.telescopicCoupler.position.set(0, -shaftLen, 0);

    this.updateCoordDropLine();
  }

  buildCoordHelpers() {
    this.coordHelpersGroup = new THREE.Group();
    this.coordHelpersGroup.name = "DeltaCoordHelpers";

    // Base Frame {B} at center of Delta dome
    const baseAxes = new THREE.AxesHelper(0.20);
    baseAxes.renderOrder = 999;
    baseAxes.material.depthTest = false;
    this.coordHelpersGroup.add(baseAxes);

    // End-Effector Frame {EE} attached to toolFlange
    this.eeAxes = new THREE.AxesHelper(0.14);
    this.eeAxes.position.set(0, -0.09, 0);
    this.eeAxes.renderOrder = 999;
    this.eeAxes.material.depthTest = false;
    this.toolFlange.add(this.eeAxes);

    // Drop line to conveyor/floor
    this.dropLinePositions = new Float32Array(6);
    const lineGeo = new THREE.BufferGeometry();
    lineGeo.setAttribute('position', new THREE.BufferAttribute(this.dropLinePositions, 3));
    const lineMat = new THREE.LineDashedMaterial({
      color: 0xf59e0b,
      dashSize: 0.04,
      gapSize: 0.02,
      linewidth: 2,
      depthTest: false,
      transparent: true,
      opacity: 0.85
    });
    this.dropLine = new THREE.Line(lineGeo, lineMat);
    this.dropLine.renderOrder = 998;
    this.coordHelpersGroup.add(this.dropLine);

    this.root.add(this.coordHelpersGroup);
  }

  updateCoordDropLine() {
    if (!this.coordsVisible || !this.dropLine) return;
    const eePos = this.platformPos.clone();
    eePos.y -= 0.09;
    const pos = this.dropLine.geometry.attributes.position.array;
    pos[0] = eePos.x;
    pos[1] = eePos.y;
    pos[2] = eePos.z;
    pos[3] = eePos.x;
    pos[4] = -0.57; // Table surface
    pos[5] = eePos.z;
    this.dropLine.geometry.attributes.position.needsUpdate = true;
    this.dropLine.computeLineDistances();
  }

  setCoordsVisible(visible) {
    this.coordsVisible = visible;
    if (this.coordHelpersGroup) this.coordHelpersGroup.visible = visible;
    if (this.eeAxes) this.eeAxes.visible = visible;
    if (visible) this.updateCoordDropLine();
  }

  getJointAngles() {
    return {
      m1: Number(THREE.MathUtils.radToDeg(this.theta[0]).toFixed(2)),
      m2: Number(THREE.MathUtils.radToDeg(this.theta[1]).toFixed(2)),
      m3: Number(THREE.MathUtils.radToDeg(this.theta[2]).toFixed(2)),
      yaw: Number(THREE.MathUtils.radToDeg(this.theta4).toFixed(2))
    };
  }

  getEndEffectorWorldPosition() {
    const tipLocal = new THREE.Vector3(0, -0.097, 0);
    return this.toolFlange.localToWorld(tipLocal);
  }
}
