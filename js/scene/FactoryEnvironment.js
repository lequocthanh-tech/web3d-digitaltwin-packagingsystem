import * as THREE from 'three';

/**
 * FactoryEnvironment - High-Fidelity Packaging Cell Environment
 * Inspired by real FANUC / Yaskawa automated packaging lines (matching video prototype).
 * 
 * Features:
 * 1. Industrial Yellow Cell Enclosure Wall in background with clear safety inspection windows.
 * 2. Continuous Brushed Stainless Steel Packaging Conveyor Table with Infeed & Collation lanes.
 * 3. Pre-collated groups of 4 chocolate bars lined up in a neat row along the front lane (matching video).
 * 4. Infeed lane with randomly positioned & oriented chocolate bars.
 * 5. Cognex In-Sight Smart Camera with volumetric green laser line scanning plane.
 * 6. Photoelectric Sensor DI_11 with active red laser beam at the 6-DOF robot pickup station.
 * 7. Box Conveyor with realistic open-top corrugated carton boxes.
 */
export class FactoryEnvironment {
  constructor(scene) {
    this.scene = scene;
    this.root = new THREE.Group();
    this.root.name = "FactoryEnvironment";
    this.scene.add(this.root);

    // Collections of dynamic products
    this.infeedProducts = [];
    this.collatedPacks = [];
    this.cartonBoxes = [];

    // Conveyor speeds (m/s) - Synchronized so Delta Robot catches every single chocolate bar!
    this.infeedRateMultiplier = 1.0;
    this.infeedSpacing = 0.55;
    this.speedConveyor1 = 0.28;
    this.speedConveyor2 = 0.48;
    this.speedConveyor3 = 0.55;

    // States
    this.conveyor1Active = false;
    this.conveyor2Active = false;
    this.conveyor3Active = false;

    this.sensorDI11 = false;
    this.sensorDI12 = false;

    // Safety Enclosure, Door & Interlock states
    this.safetyGateClosed = true;
    this.SafetyGate_Closed = true; // Backward compatibility alias
    this.doorAngle = 0;
    this.targetDoorAngle = 0;

    // Andon Tower Light state ('RUN', 'STANDBY', 'ALARM')
    this.andonState = 'STANDBY';
    this.andonBlinkTimer = 0;
    this.andonBlinkVisible = false;

    // Pneumatic Stopper & Clamps state on Conveyor 3
    this.stopperState = "EXTENDED"; // "EXTENDED" (holding) | "RETRACTED" (passing)
    this.clampState = "CLAMPED";     // "CLAMPED" | "RELEASED"
    this.stopperCurrentY = 0.785;    // Animated Y position
    this.frontClampCurrentZ = -1.325; // Animated front clamp Z
    this.rearClampCurrentZ = -1.575;  // Animated rear clamp Z

    // QC & Reject Pusher System on Conveyor 1
    this.totalInspectedCount = 0;
    this.ngDefectCount = 0;
    this.NG_Defect_Count = 0;        // SCADA alias
    this.nextProductIsNG = false;
    this.pusherState = "RETRACTED";  // "RETRACTED" | "EXTENDING" | "RETRACTING"
    this.pusherCurrentZ = -0.13;     // Retracted at -0.13m, Extended at -0.46m
    this.rejectCylinderDO = false;
    this.DO_REJECT_CYLINDER = false; // SCADA alias
    this.activeRejectItem = null;
    this.ngScrapBinItems = [];       // Rejected NG pieces resting inside the NG Scrap Bin (fade out after 5s)

    // Automatic Top Carton Sealer on Conveyor 3
    this.sealedCartonCount = 0;
    this.sealerSensorActive = false;
    this.DI_SEALER_SENSOR = false;   // SCADA alias
    this.sealerActive = false;
    this.DO_SEALER_ACTIVE = false;   // SCADA alias

    // Conveyor 1 Rotary Encoder & Dynamic Tracking (HSC)
    this.encoderPulseCounter = 0;
    this.Encoder_Pulse_Counter = 0;  // SCADA alias
    this.encoderScaleK = 0.0001;     // K_scale = 0.1 mm/pulse (0.0001 m/pulse)
    this.encoderFreqHz = 0;
    this.encoderChannelA = false;
    this.encoderChannelB = false;
    this.baseEncoderPulse = 0;
    this.Base_Encoder_Pulse = 0;     // SCADA alias
    this.lastTrackedX = 0;
    this.lastTrackedDeltaPulse = 0;
    this.encoderRotAngle = 0;

    // Industrial Inkjet Printer & Barcode Scanner on Conveyor 3
    this.printTriggerDI = false;
    this.DI_PRINT_TRIGGER = false;   // SCADA alias
    this.inkjetActiveDO = false;
    this.DO_INKJET_ACTIVE = false;   // SCADA alias
    this.DO_INKJET_FIRE = false;     // SCADA alias
    this.barcodeScanValid = false;
    this.Barcode_Scan_Valid = false; // SCADA alias
    this.DI_BARCODE_OK = false;      // SCADA alias
    this.barcodeReadError = false;
    this.Barcode_Read_Error = false;
    this.injectBarcodeError = false; // Optional fault flag
    this.totalVerifiedBoxes = 0;
    this.Total_Verified_Boxes = 0;   // SCADA alias
    this.nextBoxSerial = 42;
    this.barcodeScanFlashTimer = 0;

    const now = new Date();
    const yyyy = now.getFullYear();
    const mm = String(now.getMonth() + 1).padStart(2, '0');
    const dd = String(now.getDate()).padStart(2, '0');
    this.currentMfgDate = `${yyyy}-${mm}-${dd}`;
    this.currentExpDate = `${yyyy + 1}-${mm}-${dd}`;
    this.currentLotCode = `#LOT-${yyyy}A`;

    this.lastPrintedBoxInfo = {
      boxId: `BOX-${yyyy}-0041`,
      mfg: this.currentMfgDate,
      exp: this.currentExpDate,
      lot: this.currentLotCode,
      status: 'READY / STANDBY',
      valid: true
    };

    // Industrial Control Cabinet & Cable Routing Infrastructure States
    this.cabinetPowerOn = false;
    this.cabinetBooting = false;
    this.cabinetBootTimer = 0;
    this.cabinetDoorOpen = false;
    this.cabinetCutawayMode = false;
    this.cabinetDoorAngle = 0;
    this.targetCabinetDoorAngle = 0;
    this.targetIsolatorAngle = 0;
    this.currentIsolatorAngle = 0;
    this.lastJ1000FreqText = '';

    this.initTexturesAndMaterials();
    this.buildIntegratedSafetyEnclosure();
    this.buildFloor();
    this.buildIntegratedConveyorTable();
    this.buildConveyor3();
    this.buildVisionStation();
    this.buildPneumaticRejectPusherAndNGBin();
    this.buildTopCartonSealer();
    this.buildConveyor1RotaryEncoder();
    this.buildInkjetAndBarcodeStation();
    this.buildAGVAndVirtualWarehouse();
    this.buildIndustrialControlCabinetAndCableInfrastructure();
    this.buildSensors();

    // Spawn demo items matching the video scene
    this.spawnInitialItems();

    // Đóng băng ma trận biến đổi (matrixAutoUpdate = false) cho các nhóm mô hình 3D tĩnh
    // Giúp Three.js không phải tính toán lại hàng trăm ma trận 4x4 trên CPU mỗi khung hình!
    this.freezeStaticMatrices();
  }

  freezeStaticMatrices() {
    this.root.updateMatrixWorld(true);
    const staticGroups = [
      this.conveyorTable,
      this.visionGroup,
      this.cartonSealerGroup,
      this.agvGuidePathGroup,
      this.warehouseGroup,
      this.cableInfrastructureGroup
    ];
    for (const grp of staticGroups) {
      if (!grp) continue;
      grp.updateMatrixWorld(true);
      grp.traverse(obj => {
        if (obj !== this.warehouseDropBayGroup && !this.warehouseDropBayGroup?.getObjectById(obj.id)) {
          obj.matrixAutoUpdate = false;
        }
      });
    }
    if (this.enclosureGroup) {
      this.enclosureGroup.updateMatrixWorld(true);
      this.enclosureGroup.traverse(obj => {
        if (obj !== this.doorGroup && !this.doorGroup?.getObjectById(obj.id)) {
          obj.matrixAutoUpdate = false;
        }
      });
    }
    if (this.conveyor3Group) {
      this.conveyor3Group.updateMatrixWorld(true);
      const dynamicConv3 = new Set([
        this.stopperPistonGroup,
        this.frontClampGroup,
        this.rearClampGroup,
        ...(this.rollers3 || [])
      ]);
      this.conveyor3Group.traverse(obj => {
        let isDyn = false;
        for (const d of dynamicConv3) {
          if (d && (obj === d || d.getObjectById(obj.id))) {
            isDyn = true;
            break;
          }
        }
        if (!isDyn) obj.matrixAutoUpdate = false;
      });
    }
  }

  initTexturesAndMaterials() {
    // 1. Procedural Grid Floor Texture
    const floorCanvas = document.createElement('canvas');
    floorCanvas.width = 512;
    floorCanvas.height = 512;
    const fCtx = floorCanvas.getContext('2d');
    fCtx.fillStyle = '#111827';
    fCtx.fillRect(0, 0, 512, 512);

    fCtx.strokeStyle = 'rgba(75, 85, 99, 0.4)';
    fCtx.lineWidth = 2;
    for (let x = 0; x <= 512; x += 64) {
      fCtx.beginPath();
      fCtx.moveTo(x, 0);
      fCtx.lineTo(x, 512);
      fCtx.stroke();
    }
    for (let y = 0; y <= 512; y += 64) {
      fCtx.beginPath();
      fCtx.moveTo(0, y);
      fCtx.lineTo(512, y);
      fCtx.stroke();
    }

    const floorTexture = new THREE.CanvasTexture(floorCanvas);
    floorTexture.wrapS = THREE.RepeatWrapping;
    floorTexture.wrapT = THREE.RepeatWrapping;
    floorTexture.repeat.set(14, 10);

    this.matFloor = new THREE.MeshStandardMaterial({
      map: floorTexture,
      roughness: 0.3,
      metalness: 0.25
    });

    // 2. Yellow & Black Hazard Perimeter Stripes
    const hazardCanvas = document.createElement('canvas');
    hazardCanvas.width = 256;
    hazardCanvas.height = 64;
    const hCtx = hazardCanvas.getContext('2d');
    hCtx.fillStyle = '#facc15';
    hCtx.fillRect(0, 0, 256, 64);
    hCtx.fillStyle = '#111827';
    for (let i = -64; i < 320; i += 40) {
      hCtx.beginPath();
      hCtx.moveTo(i, 0);
      hCtx.lineTo(i + 28, 0);
      hCtx.lineTo(i - 12, 64);
      hCtx.lineTo(i - 40, 64);
      hCtx.closePath();
      hCtx.fill();
    }
    const hazardTexture = new THREE.CanvasTexture(hazardCanvas);
    hazardTexture.wrapS = THREE.RepeatWrapping;
    hazardTexture.repeat.set(12, 1);

    this.matHazardStripe = new THREE.MeshBasicMaterial({ map: hazardTexture });

    // 3. Brushed Stainless Steel (Inox 304 - High Realism)
    this.matInox = new THREE.MeshStandardMaterial({
      color: 0xe2e8f0,
      roughness: 0.22,
      metalness: 0.88
    });

    // 4. Food-Grade Conveyor Belts (High-contrast flight cleats & index markings for visible 1:1 motion)
    // Belt 1: Conveyor 1 Infeed Belt (White hygienic polyurethane with transverse flight slats)
    const belt1Canvas = document.createElement('canvas');
    belt1Canvas.width = 512;
    belt1Canvas.height = 256;
    const b1Ctx = belt1Canvas.getContext('2d');
    b1Ctx.fillStyle = '#f8fafc'; // Clean white
    b1Ctx.fillRect(0, 0, 512, 256);

    // Subtle edge tracking lines
    b1Ctx.fillStyle = '#cbd5e1';
    b1Ctx.fillRect(0, 0, 512, 10);
    b1Ctx.fillRect(0, 246, 512, 10);

    // Transverse flight slats / cleats across X
    for (let x = 0; x < 512; x += 32) {
      b1Ctx.fillStyle = '#94a3b8';
      b1Ctx.fillRect(x, 10, 6, 236);
      b1Ctx.fillStyle = '#e2e8f0';
      b1Ctx.fillRect(x + 6, 10, 2, 236);
    }

    this.belt1Texture = new THREE.CanvasTexture(belt1Canvas);
    this.belt1Texture.wrapS = THREE.RepeatWrapping;
    this.belt1Texture.wrapT = THREE.RepeatWrapping;
    // Length 5.80m / 0.20m per tile = 29 repeats
    this.belt1Texture.repeat.set(29, 1);

    this.matBelt1 = new THREE.MeshStandardMaterial({
      map: this.belt1Texture,
      roughness: 0.42,
      metalness: 0.08
    });

    // Belt 2: Conveyor 2 Collation Belt (High-contrast slate rubber with cyan/amber flight guide stripes)
    const belt2Canvas = document.createElement('canvas');
    belt2Canvas.width = 512;
    belt2Canvas.height = 256;
    const b2Ctx = belt2Canvas.getContext('2d');
    b2Ctx.fillStyle = '#1e293b'; // Slate dark rubber
    b2Ctx.fillRect(0, 0, 512, 256);

    // Longitudinal boundary guides
    b2Ctx.fillStyle = '#334155';
    b2Ctx.fillRect(0, 0, 512, 12);
    b2Ctx.fillRect(0, 244, 512, 12);

    // Index flight cleats (high visibility amber and cyan stripes)
    for (let x = 0; x < 512; x += 64) {
      b2Ctx.fillStyle = '#06b6d4'; // Cyan index bar
      b2Ctx.fillRect(x, 12, 8, 232);
      b2Ctx.fillStyle = '#f59e0b'; // Amber secondary marker
      b2Ctx.fillRect(x + 32, 24, 4, 208);
      // Textured grip grooves
      b2Ctx.fillStyle = '#0f172a';
      b2Ctx.fillRect(x + 16, 12, 2, 232);
      b2Ctx.fillRect(x + 48, 12, 2, 232);
    }

    this.belt2Texture = new THREE.CanvasTexture(belt2Canvas);
    this.belt2Texture.wrapS = THREE.RepeatWrapping;
    this.belt2Texture.wrapT = THREE.RepeatWrapping;
    this.belt2Texture.repeat.set(29, 1);

    this.matBelt2 = new THREE.MeshStandardMaterial({
      map: this.belt2Texture,
      roughness: 0.38,
      metalness: 0.12
    });

    // Backward compatibility aliases
    this.beltTexture = this.belt1Texture;
    this.matBelt = this.matBelt1;

    // 5. Factory Enclosure Materials & Industrial Safety Styling
    this.matWallYellow = new THREE.MeshStandardMaterial({
      color: 0xfacc15,
      roughness: 0.4,
      metalness: 0.15
    });

    this.matWallFrame = new THREE.MeshStandardMaterial({
      color: 0x1e293b,
      roughness: 0.6,
      metalness: 0.7
    });

    this.matWindowGlass = new THREE.MeshStandardMaterial({
      color: 0x93c5fd,
      transparent: true,
      opacity: 0.28,
      roughness: 0.1,
      metalness: 0.1
    });

    // Integrated Safety Enclosure Materials
    this.matSafetyYellow = new THREE.MeshStandardMaterial({
      color: 0xf59e0b, // Industrial safety amber/yellow (#f59e0b)
      roughness: 0.35,
      metalness: 0.25
    });

    this.matSafetyBlack = new THREE.MeshStandardMaterial({
      color: 0x111827, // Dark industrial structural steel
      roughness: 0.55,
      metalness: 0.65
    });

    // Procedural High-Fidelity Anti-Glare Wire Mesh Texture (Troax / Axelent style)
    const meshCanvas = document.createElement('canvas');
    meshCanvas.width = 128;
    meshCanvas.height = 128;
    const mCtx = meshCanvas.getContext('2d');
    mCtx.clearRect(0, 0, 128, 128);
    // Sợi lưới mảnh, ô lưới thoáng rộng (>96% diện tích rỗng trong suốt) để nhìn xuyên rõ vào trong buồng máy
    mCtx.strokeStyle = 'rgba(148, 163, 184, 0.85)';
    mCtx.lineWidth = 2;
    mCtx.strokeRect(1, 1, 126, 126);

    const meshTexture = new THREE.CanvasTexture(meshCanvas);
    meshTexture.wrapS = THREE.RepeatWrapping;
    meshTexture.wrapT = THREE.RepeatWrapping;
    meshTexture.repeat.set(14, 14);

    this.matWireMesh = new THREE.MeshBasicMaterial({
      map: meshTexture,
      transparent: false,
      alphaTest: 0.4,
      side: THREE.DoubleSide
    });

    // Các ô quan sát mặt trước & cửa bảo trì: dùng chung lưới thép thưa mỏng (nhìn xuyên rõ, không còn tấm chắn đặc)
    this.matPolycarbWindow = this.matWireMesh;

    // Safety Interlock Switch Materials
    this.matInterlockRed = new THREE.MeshStandardMaterial({
      color: 0xdc2626,
      roughness: 0.35,
      metalness: 0.2
    });
    this.matInterlockYellow = new THREE.MeshStandardMaterial({
      color: 0xfacc15,
      roughness: 0.35,
      metalness: 0.2
    });
    this.matInterlockLed = new THREE.MeshBasicMaterial({
      color: 0x22c55e // Green when door is closed
    });

    // Andon 3-Tier Tower Light Materials with Emissive Properties
    this.matAndonRed = new THREE.MeshStandardMaterial({
      color: 0xef4444,
      emissive: 0xff1111,
      emissiveIntensity: 0.05,
      roughness: 0.2,
      metalness: 0.1,
      transparent: true,
      opacity: 0.92
    });
    this.matAndonYellow = new THREE.MeshStandardMaterial({
      color: 0xf59e0b,
      emissive: 0xff9900,
      emissiveIntensity: 2.2, // Default: Standby lit
      roughness: 0.2,
      metalness: 0.1,
      transparent: true,
      opacity: 0.92
    });
    this.matAndonGreen = new THREE.MeshStandardMaterial({
      color: 0x22c55e,
      emissive: 0x00ff44,
      emissiveIntensity: 0.05,
      roughness: 0.2,
      metalness: 0.1,
      transparent: true,
      opacity: 0.92
    });
    this.matAndonPole = new THREE.MeshStandardMaterial({
      color: 0x334155,
      roughness: 0.4,
      metalness: 0.8
    });

    // Pneumatic Cylinder & Clamp Materials
    this.matPneumaticBody = new THREE.MeshStandardMaterial({
      color: 0xd1d5db,
      roughness: 0.28,
      metalness: 0.85
    });
    this.matPistonRod = new THREE.MeshStandardMaterial({
      color: 0xffffff,
      roughness: 0.05,
      metalness: 0.98
    });
    this.matAirTubeBlue = new THREE.MeshStandardMaterial({
      color: 0x0284c7,
      roughness: 0.45,
      metalness: 0.1
    });
    this.matClampUrethane = new THREE.MeshStandardMaterial({
      color: 0x0284c7,
      roughness: 0.35,
      metalness: 0.25
    });

    // 6. Realistic Chocolate Bars (Rich glossy chocolate + metallic foil)
    this.matChocolate = new THREE.MeshStandardMaterial({
      color: 0x3d1c06,
      roughness: 0.3,
      metalness: 0.1
    });

    this.matFoilRed = new THREE.MeshStandardMaterial({
      color: 0x991b1b, // Premium maroon red foil
      roughness: 0.18,
      metalness: 0.75
    });

    this.matFoilGold = new THREE.MeshStandardMaterial({
      color: 0xf59e0b, // Golden brand crest
      roughness: 0.15,
      metalness: 0.95
    });

    // 7. Cardboard Box
    this.matCardboard = new THREE.MeshStandardMaterial({
      color: 0xc29b68,
      roughness: 0.85,
      metalness: 0.05
    });

    // 8. Quality Control / NG Defective Products (Bright Red #ef4444 with Fracture Cracks & Hazard Bands)
    const ngCanvas = document.createElement('canvas');
    ngCanvas.width = 256;
    ngCanvas.height = 128;
    const ngCtx = ngCanvas.getContext('2d');
    ngCtx.fillStyle = '#ef4444'; // Bright defective red (#ef4444)
    ngCtx.fillRect(0, 0, 256, 128);
    // Dark jagged fracture/crack veins across the defective piece
    ngCtx.strokeStyle = '#450a0a';
    ngCtx.lineWidth = 5;
    ngCtx.beginPath();
    ngCtx.moveTo(15, 18); ngCtx.lineTo(55, 58); ngCtx.lineTo(88, 26); ngCtx.lineTo(142, 92);
    ngCtx.moveTo(165, 108); ngCtx.lineTo(205, 52); ngCtx.lineTo(245, 96);
    ngCtx.stroke();
    // Secondary white/yellow stress cracks
    ngCtx.strokeStyle = '#fde047';
    ngCtx.lineWidth = 2;
    ngCtx.beginPath();
    ngCtx.moveTo(18, 22); ngCtx.lineTo(55, 58); ngCtx.lineTo(88, 26);
    ngCtx.stroke();
    // Yellow/Black hazard warning band & "NG DEFECT" stamp
    ngCtx.fillStyle = '#facc15';
    ngCtx.fillRect(102, 0, 52, 128);
    ngCtx.fillStyle = '#111827';
    for (let sy = -20; sy < 140; sy += 24) {
      ngCtx.beginPath();
      ngCtx.moveTo(102, sy);
      ngCtx.lineTo(154, sy + 18);
      ngCtx.lineTo(154, sy + 10);
      ngCtx.lineTo(102, sy - 8);
      ngCtx.fill();
    }
    const ngTexture = new THREE.CanvasTexture(ngCanvas);
    this.matProductDefectNG = new THREE.MeshStandardMaterial({
      color: 0xef4444,
      map: ngTexture,
      roughness: 0.32,
      metalness: 0.25,
      emissive: 0x7f1d1d,
      emissiveIntensity: 0.35
    });

    // 9. Top Carton Sealer Materials
    this.matSealerFrame = new THREE.MeshStandardMaterial({
      color: 0x0284c7, // Industrial Sky Blue RAL 5015
      roughness: 0.38,
      metalness: 0.35
    });

    this.matSealerAccents = new THREE.MeshStandardMaterial({
      color: 0x1e293b, // Industrial Slate Grey
      roughness: 0.55,
      metalness: 0.65
    });

    this.matRollerOrange = new THREE.MeshStandardMaterial({
      color: 0xea580c, // High-visibility polyurethane rubber roller
      roughness: 0.40,
      metalness: 0.10
    });

    this.matTapeAmber = new THREE.MeshStandardMaterial({
      color: 0xf59e0b, // Amber packaging adhesive tape
      roughness: 0.15,
      metalness: 0.05,
      transparent: true,
      opacity: 0.85,
      side: THREE.DoubleSide
    });

    this.matNGBinYellow = new THREE.MeshStandardMaterial({
      color: 0xeab308, // Industrial yellow container
      roughness: 0.45,
      metalness: 0.15
    });
  }

  buildFactoryEnclosure() {
    this.buildIntegratedSafetyEnclosure();
  }

  buildIntegratedSafetyEnclosure() {
    // =========================================================================
    // INTEGRATED INDUSTRIAL ROBOT SAFETY CELL ENCLOSURE
    // Encloses entire cell: Delta Robot, 6-DOF Arm, and all 3 Conveyors
    // Bounds: X [-3.50m, +4.10m] (7.60m length), Z [-1.85m, +0.70m] (2.55m depth), Height 2.20m
    // =========================================================================
    this.enclosureGroup = new THREE.Group();
    this.enclosureGroup.name = "IntegratedSafetyEnclosure";

    const xMin = -3.50;
    const xMax = 4.10;
    const zMin = -1.85;
    const zMax = 0.70;
    const fenceH = 2.20;
    const postW = 0.05;

    // Helper: Create Upright Safety Post with heavy-duty foot plate & 4 anchor bolts
    const createPost = (x, z, h = fenceH) => {
      const postGrp = new THREE.Group();
      postGrp.position.set(x, 0, z);

      // Yellow Extrusion Post Body
      const postMesh = new THREE.Mesh(new THREE.BoxGeometry(postW, h, postW), this.matSafetyYellow);
      postMesh.position.y = h / 2;
      postMesh.castShadow = true;
      postGrp.add(postMesh);

      // Black Steel Floor Base Plate (lược bỏ 4 bu-lông 8mm siêu nhỏ để giảm 72 draw calls trên máy văn phòng)
      const basePlate = new THREE.Mesh(new THREE.BoxGeometry(0.12, 0.015, 0.12), this.matSafetyBlack);
      basePlate.position.y = 0.0075;
      postGrp.add(basePlate);

      this.enclosureGroup.add(postGrp);
      return postGrp;
    };

    // 1. Upright Posts Layout
    // 4 Main Corners
    createPost(xMin, zMax);
    createPost(xMax, zMax);
    createPost(xMax, zMin);
    createPost(xMin, zMin);

    // Front Wall Posts (Z = +0.70m)
    createPost(-2.10, zMax);
    createPost(-0.70, zMax); // Left Door Jamb & Andon Light Mast
    createPost(0.30, zMax);  // Right Door Jamb & Interlock Switch
    createPost(1.70, zMax);
    createPost(2.90, zMax);

    // Rear Wall Posts (Z = -1.85m)
    createPost(-2.25, zMin);
    createPost(-1.00, zMin);
    createPost(0.25, zMin);
    createPost(1.50, zMin);
    createPost(2.80, zMin);

    // Left Wall Posts (X = -3.50m)
    createPost(xMin, -0.60);
    createPost(xMin, -1.15); // Outfeed tunnel upper edge
    createPost(xMin, -1.75); // Outfeed tunnel lower edge

    // Right Wall Posts (X = +4.10m)
    createPost(xMax, 0.05);
    createPost(xMax, -0.60); // Infeed tunnel lower edge (infeed centered at Z = -0.32m)
    createPost(xMax, -1.25);

    // 2. Upper Perimeter Header Rails (Y = 2.18m)
    const headerThick = 0.04;
    const addHeader = (x, y, z, wX, wZ) => {
      const beam = new THREE.Mesh(new THREE.BoxGeometry(wX, headerThick, wZ), this.matSafetyYellow);
      beam.position.set(x, y, z);
      beam.castShadow = true;
      this.enclosureGroup.add(beam);
    };

    // 4 Top Perimeter Beams
    const totalX = xMax - xMin;
    const totalZ = zMax - zMin;
    addHeader((xMin + xMax) / 2, fenceH - headerThick / 2, zMax, totalX, headerThick);
    addHeader((xMin + xMax) / 2, fenceH - headerThick / 2, zMin, totalX, headerThick);
    addHeader(xMin, fenceH - headerThick / 2, (zMin + zMax) / 2, headerThick, totalZ);
    addHeader(xMax, fenceH - headerThick / 2, (zMin + zMax) / 2, headerThick, totalZ);

    // 3. Mid-Rails (Y = 1.05m) & Kick Plates (Toe Boards Y = 0.075m)
    const addMidAndKick = (x1, z1, x2, z2) => {
      const cx = (x1 + x2) / 2;
      const cz = (z1 + z2) / 2;
      const len = Math.hypot(x2 - x1, z2 - z1);
      const isAlongX = Math.abs(z2 - z1) < 0.001;

      const wX = isAlongX ? len : headerThick;
      const wZ = isAlongX ? headerThick : len;

      // Mid rail
      const mid = new THREE.Mesh(new THREE.BoxGeometry(wX, headerThick, wZ), this.matSafetyYellow);
      mid.position.set(cx, 1.05, cz);
      this.enclosureGroup.add(mid);

      // Kick plate (Sheet metal toe board, height 0.14m)
      const kick = new THREE.Mesh(new THREE.BoxGeometry(isAlongX ? len : 0.015, 0.14, isAlongX ? 0.015 : len), this.matSafetyBlack);
      kick.position.set(cx, 0.075, cz);
      this.enclosureGroup.add(kick);
    };

    // Rear wall rails (continuous)
    addMidAndKick(xMin, zMin, xMax, zMin);

    // Front wall rails (skipping door opening between X = -0.70 and +0.30)
    addMidAndKick(xMin, zMax, -0.70, zMax);
    addMidAndKick(0.30, zMax, xMax, zMax);

    // Left wall rails (skipping outfeed tunnel between Z = -1.15 and -1.75)
    addMidAndKick(xMin, zMax, xMin, -1.15);
    addMidAndKick(xMin, -1.75, xMin, zMin);

    // Right wall rails (skipping infeed tunnel between Z = -0.05 and -0.60)
    addMidAndKick(xMax, zMax, xMax, 0.05);
    addMidAndKick(xMax, -0.60, xMax, zMin);

    // 4. Wall Infill Panels (Tấm lưới đơn 1 lớp PlaneGeometry mảnh nhẹ, không tạo hiệu ứng chồng lưới 2 mặt)
    // Helper: Wire Mesh Panel
    const addMeshPanel = (cx, cy, cz, wX, hY, wZ) => {
      const isAlongX = wX >= wZ;
      const pGeo = new THREE.PlaneGeometry(isAlongX ? wX : wZ, hY);
      const mesh = new THREE.Mesh(pGeo, this.matWireMesh);
      if (!isAlongX) mesh.rotation.y = Math.PI / 2;
      mesh.position.set(cx, cy, cz);
      this.enclosureGroup.add(mesh);
    };

    // Helper: Wire Mesh Inspection Bay (Tấm lưới đơn nhìn xuyên thấu hoàn toàn)
    const addPolycarbWindow = (x1, x2, z, hY = 1.95, yCenter = 1.12) => {
      const w = Math.abs(x2 - x1);
      const cx = (x1 + x2) / 2;

      const glassGeo = new THREE.PlaneGeometry(w - 0.06, hY - 0.06);
      const glass = new THREE.Mesh(glassGeo, this.matWireMesh);
      glass.position.set(cx, yCenter, z);

      this.enclosureGroup.add(glass);
    };

    // Front Wall Viewing Panels (Clear Polycarbonate Windows for unobstructed observation!)
    // Left of door (2 wide observation bays)
    addPolycarbWindow(xMin, -2.10, zMax);
    addPolycarbWindow(-2.10, -0.70, zMax);
    // Right of door (3 observation bays for Delta & Infeed)
    addPolycarbWindow(0.30, 1.70, zMax);
    addPolycarbWindow(1.70, 2.90, zMax);
    addPolycarbWindow(2.90, xMax, zMax);

    // Rear Wall Infill (High-Fidelity Anti-Glare Wire Mesh)
    const rearSegments = [-3.50, -2.25, -1.00, 0.25, 1.50, 2.80, 4.10];
    for (let i = 0; i < rearSegments.length - 1; i++) {
      const rx1 = rearSegments[i];
      const rx2 = rearSegments[i + 1];
      addMeshPanel((rx1 + rx2) / 2, 1.13, zMin, Math.abs(rx2 - rx1) - 0.04, 1.95, 0.008);
    }

    // Left Wall Infill Panels
    addMeshPanel(xMin, 1.13, (zMax + (-1.15)) / 2, 0.008, 1.95, Math.abs(-1.15 - zMax) - 0.04);
    addMeshPanel(xMin, 1.13, (-1.75 + zMin) / 2, 0.008, 1.95, Math.abs(zMin - (-1.75)) - 0.04);
    // Header mesh above Conveyor 3 outfeed tunnel
    addMeshPanel(xMin, 1.68, -1.45, 0.008, 0.94, 0.58);

    // Right Wall Infill Panels
    addMeshPanel(xMax, 1.13, (zMax + 0.05) / 2, 0.008, 1.95, Math.abs(0.05 - zMax) - 0.04);
    addMeshPanel(xMax, 1.13, (-0.60 + zMin) / 2, 0.008, 1.95, Math.abs(zMin - (-0.60)) - 0.04);
    // Header mesh above Conveyor 1 infeed tunnel
    addMeshPanel(xMax, 1.72, -0.32, 0.008, 0.88, 0.52);

    // =========================================================================
    // 5. CONVEYOR SAFETY TUNNELS (Cửa ngõ Băng tải & Hộp hầm bảo vệ nhô ra 400mm)
    // =========================================================================

    // A. Conveyor 1 Infeed Tunnel (Right side: X = +4.10m to +4.50m, protruding +400mm)
    // Centered at Z = -0.32m, Conveyor height Y = 0.85m
    this.infeedTunnelGroup = new THREE.Group();
    this.infeedTunnelGroup.name = "SafetyTunnel_Infeed_Conveyor1";
    {
      const tunLen = 0.40; // 400mm protrusion
      const tunWidthZ = 0.54;
      const tunHeightY = 0.44;
      const tunCenterX = 4.10 + tunLen / 2;
      const tunCenterY = 0.85 + tunHeightY / 2;
      const tunCenterZ = -0.32;

      // 4 Yellow Corner Longitudinal Beams
      const tPostGeo = new THREE.BoxGeometry(tunLen, 0.03, 0.03);
      for (const dy of [-tunHeightY / 2, tunHeightY / 2]) {
        for (const dz of [-tunWidthZ / 2, tunWidthZ / 2]) {
          const tEdge = new THREE.Mesh(tPostGeo, this.matSafetyYellow);
          tEdge.position.set(tunCenterX, tunCenterY + dy, tunCenterZ + dz);
          tEdge.castShadow = true;
          this.infeedTunnelGroup.add(tEdge);
        }
      }

      // Outer Entrance Rim with Hazard Perimeter Stripes at X = +4.50m
      const rimGeoH = new THREE.BoxGeometry(0.015, 0.035, tunWidthZ);
      const rimTop = new THREE.Mesh(rimGeoH, this.matHazardStripe);
      rimTop.position.set(4.10 + tunLen, tunCenterY + tunHeightY / 2, tunCenterZ);
      const rimBottom = new THREE.Mesh(rimGeoH, this.matHazardStripe);
      rimBottom.position.set(4.10 + tunLen, tunCenterY - tunHeightY / 2, tunCenterZ);
      this.infeedTunnelGroup.add(rimTop, rimBottom);

      const rimGeoV = new THREE.BoxGeometry(0.015, tunHeightY, 0.035);
      const rimLeft = new THREE.Mesh(rimGeoV, this.matHazardStripe);
      rimLeft.position.set(4.10 + tunLen, tunCenterY, tunCenterZ + tunWidthZ / 2);
      const rimRight = new THREE.Mesh(rimGeoV, this.matHazardStripe);
      rimRight.position.set(4.10 + tunLen, tunCenterY, tunCenterZ - tunWidthZ / 2);
      this.infeedTunnelGroup.add(rimLeft, rimRight);

      // Clear Polycarbonate Roof & Sides
      const roofMesh = new THREE.Mesh(new THREE.BoxGeometry(tunLen - 0.02, 0.008, tunWidthZ - 0.04), this.matPolycarbWindow);
      roofMesh.position.set(tunCenterX, tunCenterY + tunHeightY / 2, tunCenterZ);
      const sideMeshF = new THREE.Mesh(new THREE.BoxGeometry(tunLen - 0.02, tunHeightY - 0.04, 0.008), this.matPolycarbWindow);
      sideMeshF.position.set(tunCenterX, tunCenterY, tunCenterZ + tunWidthZ / 2);
      const sideMeshB = new THREE.Mesh(new THREE.BoxGeometry(tunLen - 0.02, tunHeightY - 0.04, 0.008), this.matPolycarbWindow);
      sideMeshB.position.set(tunCenterX, tunCenterY, tunCenterZ - tunWidthZ / 2);
      this.infeedTunnelGroup.add(roofMesh, sideMeshF, sideMeshB);

      // Warning Signboard
      const signMesh = new THREE.Mesh(new THREE.BoxGeometry(0.01, 0.06, 0.32), this.matSafetyYellow);
      signMesh.position.set(4.10 + tunLen + 0.002, tunCenterY + tunHeightY / 2 + 0.035, tunCenterZ);
      this.infeedTunnelGroup.add(signMesh);

      this.enclosureGroup.add(this.infeedTunnelGroup);
    }

    // B. Conveyor 3 Outfeed Tunnel (Left side: X = -3.50m to -3.90m, protruding -400mm)
    // Centered at Z = -1.45m, Conveyor height Y = 0.72m
    this.outfeedTunnelGroup = new THREE.Group();
    this.outfeedTunnelGroup.name = "SafetyTunnel_Outfeed_Conveyor3";
    {
      const tunLen = 0.40; // 400mm protrusion
      const tunWidthZ = 0.64;
      const tunHeightY = 0.50;
      const tunCenterX = -3.50 - tunLen / 2;
      const tunCenterY = 0.72 + tunHeightY / 2;
      const tunCenterZ = -1.45;

      // 4 Yellow Corner Longitudinal Beams
      const tPostGeo = new THREE.BoxGeometry(tunLen, 0.03, 0.03);
      for (const dy of [-tunHeightY / 2, tunHeightY / 2]) {
        for (const dz of [-tunWidthZ / 2, tunWidthZ / 2]) {
          const tEdge = new THREE.Mesh(tPostGeo, this.matSafetyYellow);
          tEdge.position.set(tunCenterX, tunCenterY + dy, tunCenterZ + dz);
          tEdge.castShadow = true;
          this.outfeedTunnelGroup.add(tEdge);
        }
      }

      // Outer Exit Rim with Hazard Perimeter Stripes at X = -3.90m
      const rimGeoH = new THREE.BoxGeometry(0.015, 0.035, tunWidthZ);
      const rimTop = new THREE.Mesh(rimGeoH, this.matHazardStripe);
      rimTop.position.set(-3.50 - tunLen, tunCenterY + tunHeightY / 2, tunCenterZ);
      const rimBottom = new THREE.Mesh(rimGeoH, this.matHazardStripe);
      rimBottom.position.set(-3.50 - tunLen, tunCenterY - tunHeightY / 2, tunCenterZ);
      this.outfeedTunnelGroup.add(rimTop, rimBottom);

      const rimGeoV = new THREE.BoxGeometry(0.015, tunHeightY, 0.035);
      const rimLeft = new THREE.Mesh(rimGeoV, this.matHazardStripe);
      rimLeft.position.set(-3.50 - tunLen, tunCenterY, tunCenterZ + tunWidthZ / 2);
      const rimRight = new THREE.Mesh(rimGeoV, this.matHazardStripe);
      rimRight.position.set(-3.50 - tunLen, tunCenterY, tunCenterZ - tunWidthZ / 2);
      this.outfeedTunnelGroup.add(rimLeft, rimRight);

      // Polycarbonate Roof & Sides
      const roofMesh = new THREE.Mesh(new THREE.BoxGeometry(tunLen - 0.02, 0.008, tunWidthZ - 0.04), this.matPolycarbWindow);
      roofMesh.position.set(tunCenterX, tunCenterY + tunHeightY / 2, tunCenterZ);
      const sideMeshF = new THREE.Mesh(new THREE.BoxGeometry(tunLen - 0.02, tunHeightY - 0.04, 0.008), this.matPolycarbWindow);
      sideMeshF.position.set(tunCenterX, tunCenterY, tunCenterZ + tunWidthZ / 2);
      const sideMeshB = new THREE.Mesh(new THREE.BoxGeometry(tunLen - 0.02, tunHeightY - 0.04, 0.008), this.matPolycarbWindow);
      sideMeshB.position.set(tunCenterX, tunCenterY, tunCenterZ - tunWidthZ / 2);
      this.outfeedTunnelGroup.add(roofMesh, sideMeshF, sideMeshB);

      // Warning Signboard
      const signMesh = new THREE.Mesh(new THREE.BoxGeometry(0.01, 0.06, 0.36), this.matSafetyYellow);
      signMesh.position.set(-3.50 - tunLen - 0.002, tunCenterY + tunHeightY / 2 + 0.035, tunCenterZ);
      this.outfeedTunnelGroup.add(signMesh);

      this.enclosureGroup.add(this.outfeedTunnelGroup);
    }

    // =========================================================================
    // 6. MAINTENANCE SAFETY DOOR WITH INTERLOCK SWITCH (Cửa bảo trì có Khóa An toàn)
    // =========================================================================
    // Hinged at left door post X = -0.70m, Z = +0.70m
    // Door opening spans from X = -0.70m to X = +0.30m (width 1.00m, height 2.05m)
    this.doorGroup = new THREE.Group();
    this.doorGroup.name = "MaintenanceSafetyDoor";
    this.doorGroup.position.set(-0.70, 0, zMax);

    const doorW = 0.98;
    const doorH = 2.02;
    const doorFrameThick = 0.04;

    // Door Frame Outer Box (Local origin at hinge X = 0)
    // Top & Bottom Rails
    const dRailTop = new THREE.Mesh(new THREE.BoxGeometry(doorW, doorFrameThick, doorFrameThick), this.matSafetyYellow);
    dRailTop.position.set(doorW / 2, doorH - doorFrameThick / 2 + 0.06, 0);
    const dRailBtm = new THREE.Mesh(new THREE.BoxGeometry(doorW, doorFrameThick, doorFrameThick), this.matSafetyYellow);
    dRailBtm.position.set(doorW / 2, doorFrameThick / 2 + 0.06, 0);
    // Mid horizontal brace
    const dRailMid = new THREE.Mesh(new THREE.BoxGeometry(doorW, doorFrameThick, doorFrameThick), this.matSafetyYellow);
    dRailMid.position.set(doorW / 2, 1.05, 0);

    // Left stile (hinge side) & Right stile (latch side)
    const dStileL = new THREE.Mesh(new THREE.BoxGeometry(doorFrameThick, doorH, doorFrameThick), this.matSafetyYellow);
    dStileL.position.set(doorFrameThick / 2, doorH / 2 + 0.06, 0);
    const dStileR = new THREE.Mesh(new THREE.BoxGeometry(doorFrameThick, doorH, doorFrameThick), this.matSafetyYellow);
    dStileR.position.set(doorW - doorFrameThick / 2, doorH / 2 + 0.06, 0);

    this.doorGroup.add(dRailTop, dRailBtm, dRailMid, dStileL, dStileR);

    // Heavy-duty Stainless Steel Hinges (Top & Bottom)
    const hingeGeo = new THREE.CylinderGeometry(0.015, 0.015, 0.07, 12);
    const hingeTop = new THREE.Mesh(hingeGeo, this.matInox);
    hingeTop.position.set(0, 1.85, 0.025);
    const hingeBtm = new THREE.Mesh(hingeGeo, this.matInox);
    hingeBtm.position.set(0, 0.35, 0.025);
    this.doorGroup.add(hingeTop, hingeBtm);

    // Wire Mesh Door Window Panes (Top & Bottom halves - single-layer PlaneGeometry)
    const dGlassTop = new THREE.Mesh(new THREE.PlaneGeometry(doorW - 0.08, 0.88), this.matWireMesh);
    dGlassTop.position.set(doorW / 2, 1.54, 0);
    const dGlassBtm = new THREE.Mesh(new THREE.PlaneGeometry(doorW - 0.08, 0.84), this.matWireMesh);
    dGlassBtm.position.set(doorW / 2, 0.54, 0);
    this.doorGroup.add(dGlassTop, dGlassBtm);

    // Industrial Ergonomic Door Handle (Black steel pull handle at local X = 0.90m, Y = 1.05m)
    const handleGrp = new THREE.Group();
    handleGrp.position.set(doorW - 0.08, 1.05, 0.035);
    const handleBar = new THREE.Mesh(new THREE.CylinderGeometry(0.01, 0.01, 0.22, 12), this.matSafetyBlack);
    const hStanchion1 = new THREE.Mesh(new THREE.CylinderGeometry(0.008, 0.008, 0.035, 10), this.matInox);
    hStanchion1.rotateX(Math.PI / 2);
    hStanchion1.position.set(0, 0.09, -0.018);
    const hStanchion2 = new THREE.Mesh(new THREE.CylinderGeometry(0.008, 0.008, 0.035, 10), this.matInox);
    hStanchion2.rotateX(Math.PI / 2);
    hStanchion2.position.set(0, -0.09, -0.018);
    handleGrp.add(handleBar, hStanchion1, hStanchion2);
    this.doorHandle = handleGrp;
    this.doorGroup.add(handleGrp);

    // Stainless Steel Interlock Actuator Key Tongue (protruding from door edge to slide into switch)
    this.interlockTongue = new THREE.Mesh(new THREE.BoxGeometry(0.045, 0.02, 0.006), this.matInox);
    this.interlockTongue.position.set(doorW + 0.012, 1.12, 0);
    this.doorGroup.add(this.interlockTongue);

    this.enclosureGroup.add(this.doorGroup);

    // Safety Door Interlock Switch (Mounted on stationary right door post at X = +0.30m, Z = +0.70m, Y = 1.12m)
    this.interlockSwitchGroup = new THREE.Group();
    this.interlockSwitchGroup.position.set(0.30, 1.12, zMax);

    // Switch Body (Schmersal / Telemecanique safety switch: Red body, yellow head)
    const swBody = new THREE.Mesh(new THREE.BoxGeometry(0.05, 0.12, 0.06), this.matInterlockRed);
    const swHead = new THREE.Mesh(new THREE.BoxGeometry(0.052, 0.04, 0.062), this.matInterlockYellow);
    swHead.position.y = 0.05;
    // Slot entry
    const swSlot = new THREE.Mesh(new THREE.BoxGeometry(0.015, 0.025, 0.008), this.matSafetyBlack);
    swSlot.position.set(-0.026, 0.05, 0);

    // Bi-Color Status LED on Interlock Switch (Green = Safe/Closed, Red = Tripped/Open)
    this.interlockSwitchLed = new THREE.Mesh(new THREE.SphereGeometry(0.007, 12, 12), this.matInterlockLed);
    this.interlockSwitchLed.position.set(0, -0.02, 0.032);

    this.interlockSwitchGroup.add(swBody, swHead, swSlot, this.interlockSwitchLed);
    this.enclosureGroup.add(this.interlockSwitchGroup);

    // =========================================================================
    // 7. ANDON SIGNAL TOWER LIGHT (Cột đèn tháp Andon 3 tầng trên đỉnh rào)
    // =========================================================================
    // Mounted at top of left door post at X = -0.70m, Z = +0.70m, Y = 2.20m
    this.andonTowerGroup = new THREE.Group();
    this.andonTowerGroup.name = "AndonSignalTowerLight";
    this.andonTowerGroup.position.set(-0.70, fenceH, zMax);

    // Base Mounting Flange Collar (Black anodized metal)
    const aFlange = new THREE.Mesh(new THREE.CylinderGeometry(0.04, 0.045, 0.02, 16), this.matSafetyBlack);
    aFlange.position.y = 0.01;
    this.andonTowerGroup.add(aFlange);

    // Metallic Mast Pole (0.40m tall)
    const aPole = new THREE.Mesh(new THREE.CylinderGeometry(0.013, 0.013, 0.40, 16), this.matAndonPole);
    aPole.position.y = 0.02 + 0.20;
    this.andonTowerGroup.add(aPole);

    // Tower Body Base
    const aBase = new THREE.Mesh(new THREE.CylinderGeometry(0.036, 0.036, 0.035, 20), this.matSafetyBlack);
    aBase.position.y = 0.42 + 0.0175;
    this.andonTowerGroup.add(aBase);

    // 3 Light Tiers (Cylindrical lenses with chrome divider rings)
    const tierH = 0.065;
    const tierR = 0.034;
    const tierGeo = new THREE.CylinderGeometry(tierR, tierR, tierH, 24);
    const ringGeo = new THREE.CylinderGeometry(tierR + 0.002, tierR + 0.002, 0.008, 24);

    // Tier 1: GREEN (Bottom, RUN)
    const yGreen = 0.44 + tierH / 2;
    this.andonLampGreen = new THREE.Mesh(tierGeo, this.matAndonGreen);
    this.andonLampGreen.position.y = yGreen;
    const ring1 = new THREE.Mesh(ringGeo, this.matSafetyBlack);
    ring1.position.y = yGreen + tierH / 2 + 0.004;

    // Tier 2: YELLOW (Middle, STANDBY/PAUSE/JOG)
    const yYellow = yGreen + tierH + 0.008;
    this.andonLampYellow = new THREE.Mesh(tierGeo, this.matAndonYellow);
    this.andonLampYellow.position.y = yYellow;
    const ring2 = new THREE.Mesh(ringGeo, this.matSafetyBlack);
    ring2.position.y = yYellow + tierH / 2 + 0.004;

    // Tier 3: RED (Top, ALARM/E-STOP/DOOR OPEN)
    const yRed = yYellow + tierH + 0.008;
    this.andonLampRed = new THREE.Mesh(tierGeo, this.matAndonRed);
    this.andonLampRed.position.y = yRed;

    // Top Cap with Sounder / Buzzer Slits
    const aCap = new THREE.Mesh(new THREE.CylinderGeometry(0.028, tierR + 0.002, 0.03, 20), this.matSafetyBlack);
    aCap.position.y = yRed + tierH / 2 + 0.015;

    this.andonTowerGroup.add(
      this.andonLampGreen, ring1,
      this.andonLampYellow, ring2,
      this.andonLampRed, aCap
    );

    // Dynamic Andon Point Light for realistic factory floor illumination
    this.andonPointLight = new THREE.PointLight(0xf59e0b, 1.2, 5.0);
    this.andonPointLight.position.set(0, yYellow, 0.05);
    this.andonTowerGroup.add(this.andonPointLight);

    this.enclosureGroup.add(this.andonTowerGroup);

    this.root.add(this.enclosureGroup);
  }

  buildFloor() {
    const floorGeo = new THREE.PlaneGeometry(28, 16);
    floorGeo.rotateX(-Math.PI / 2);
    this.floorMesh = new THREE.Mesh(floorGeo, this.matFloor);
    this.floorMesh.position.set(-2.5, 0, 0);
    this.floorMesh.receiveShadow = true;
    this.root.add(this.floorMesh);

    // Hazard Safety Perimeter Border
    const stripeWidth = 0.14;
    const borderOffsets = [
      { x: 0, z: -1.55, w: 9.6, d: stripeWidth },
      { x: 0, z:  1.65, w: 9.6, d: stripeWidth },
      { x: -4.8, z: 0.05, w: stripeWidth, d: 3.2 },
      { x:  4.8, z: 0.05, w: stripeWidth, d: 3.2 }
    ];

    borderOffsets.forEach(b => {
      const bGeo = new THREE.PlaneGeometry(b.w, b.d);
      bGeo.rotateX(-Math.PI / 2);
      const bMesh = new THREE.Mesh(bGeo, this.matHazardStripe);
      bMesh.position.set(b.x, 0.003, b.z);
      this.root.add(bMesh);
    });
  }

  buildIntegratedConveyorTable() {
    // =========================================================================
    // Right-Side Dual Packaging Conveyor Table (Conveyor 1 Infeed & Conveyor 2 Collation)
    // Spans X = -1.65m (right at DI_11 pickup station) to X = +4.15m (length 5.80m)
    // Leaves the Left side (X < -1.65m) completely open for Conveyor 3 (Carton Box Conveyor)!
    // =========================================================================
    this.conveyorTable = new THREE.Group();
    this.conveyorTable.name = "IntegratedPackagingConveyorTable";

    const lengthX = 5.80;
    const centerX = 1.25; // From X = -1.65 to X = +4.15
    const totalWidthZ = 0.92;
    const heightY = 0.85;
    const apronHeight = 0.24;
    const centerZ = -0.07;

    // 1. Heavy Brushed Stainless Steel Side Skirts / Apron
    const frontSkirtGeo = new THREE.BoxGeometry(lengthX, apronHeight, 0.03);
    const frontSkirt = new THREE.Mesh(frontSkirtGeo, this.matInox);
    frontSkirt.position.set(centerX, heightY - apronHeight / 2, centerZ + totalWidthZ / 2);
    frontSkirt.castShadow = true;
    this.conveyorTable.add(frontSkirt);

    const rearSkirt = new THREE.Mesh(frontSkirtGeo, this.matInox);
    rearSkirt.position.set(centerX, heightY - apronHeight / 2, centerZ - totalWidthZ / 2);
    rearSkirt.castShadow = true;
    this.conveyorTable.add(rearSkirt);

    // End cap skirt at X = -1.65m
    const endCapGeo = new THREE.BoxGeometry(0.03, apronHeight, totalWidthZ);
    const endCapLeft = new THREE.Mesh(endCapGeo, this.matInox);
    endCapLeft.position.set(centerX - lengthX / 2, heightY - apronHeight / 2, centerZ);
    this.conveyorTable.add(endCapLeft);

    // 2. High-Fidelity Conveyor Belts (Infeed matBelt1 & Collation matBelt2)
    // Rear Belt: Infeed Belt (Z = -0.32)
    const infeedBeltGeo = new THREE.BoxGeometry(lengthX - 0.04, 0.015, 0.40);
    this.beltInfeed = new THREE.Mesh(infeedBeltGeo, this.matBelt1);
    this.beltInfeed.position.set(centerX, heightY + 0.008, -0.32);
    this.beltInfeed.receiveShadow = true;
    this.conveyorTable.add(this.beltInfeed);

    // Front Belt: Collation Belt (Z = +0.18)
    const collateBeltGeo = new THREE.BoxGeometry(lengthX - 0.04, 0.015, 0.42);
    this.beltCollate = new THREE.Mesh(collateBeltGeo, this.matBelt2);
    this.beltCollate.position.set(centerX, heightY + 0.008, 0.18);
    this.beltCollate.receiveShadow = true;
    this.conveyorTable.add(this.beltCollate);

    // Inox Center Divider Rail between the 2 lanes
    const dividerGeo = new THREE.BoxGeometry(lengthX, 0.04, 0.02);
    const divider = new THREE.Mesh(dividerGeo, this.matInox);
    divider.position.set(centerX, heightY + 0.02, -0.07);
    this.conveyorTable.add(divider);

    // Stopper end-rail at the end of Collation Belt (X = -1.63m)
    const endStopGeo = new THREE.BoxGeometry(0.025, 0.045, 0.40);
    const endStop = new THREE.Mesh(endStopGeo, this.matWallYellow);
    endStop.position.set(-1.62, heightY + 0.025, 0.18);
    this.conveyorTable.add(endStop);

    // Front & Back Safety Inox Lips
    const lipGeo = new THREE.BoxGeometry(lengthX, 0.035, 0.015);
    const lipFront = new THREE.Mesh(lipGeo, this.matInox);
    lipFront.position.set(centerX, heightY + 0.02, centerZ + totalWidthZ / 2 - 0.008);
    const lipBack = new THREE.Mesh(lipGeo, this.matInox);
    lipBack.position.set(centerX, heightY + 0.02, centerZ - totalWidthZ / 2 + 0.008);
    this.conveyorTable.add(lipFront, lipBack);

    // 3. Sturdy Stainless Steel Structural Legs (3 pairs along X = -1.45, +1.20, +3.85)
    const legXPositions = [-1.45, 1.20, 3.85];
    legXPositions.forEach(lx => {
      const legHeight = heightY - apronHeight;
      const legGeo = new THREE.CylinderGeometry(0.035, 0.035, legHeight, 16);

      for (let zOffset of [-totalWidthZ / 2 + 0.06, totalWidthZ / 2 - 0.06]) {
        const leg = new THREE.Mesh(legGeo, this.matInox);
        leg.position.set(lx, legHeight / 2, centerZ + zOffset);
        leg.castShadow = true;
        this.conveyorTable.add(leg);

        // Leveling foot
        const foot = new THREE.Mesh(new THREE.CylinderGeometry(0.05, 0.05, 0.025, 16), this.matInox);
        foot.position.set(lx, 0.0125, centerZ + zOffset);
        this.conveyorTable.add(foot);
      }

      // Cross gusset between legs
      const crossGusset = new THREE.Mesh(new THREE.BoxGeometry(0.04, 0.04, totalWidthZ - 0.12), this.matInox);
      crossGusset.position.set(lx, legHeight * 0.45, centerZ);
      this.conveyorTable.add(crossGusset);
    });

    this.root.add(this.conveyorTable);
  }

  buildConveyor3() {
    // =========================================================================
    // Conveyor 3 (Dedicated Rear Carton Box Roller Conveyor BEHIND 6-DOF Robot Arm)
    // Placed BEHIND the 6-DOF Robot Arm base (base at X = -1.25m, Z = -0.75m):
    //   - Center line along Z = -1.45m, spanning X = -3.60m to X = +0.50m (length 4.10m)
    //   - Height Y = 0.72m (Box bottom at Y = 0.75m, Box top rim at Y = 0.91m)
    //   - Empty carton boxes feed in from X = +0.10m -> roll left (-X) -> stop at packing station (X = -1.25m, Z = -1.45m)
    //   - 6-DOF Robot Arm picks 4-pack in front of it (X = -1.25m, Z = +0.18m) and rotates 180° behind it
    //     to stack into the box at (X = -1.25m, Z = -1.45m).
    //   - Once full (2 layers = 8 chocolate bars), the full box discharges out to the left (X -> -3.50m)
    //     while the next empty box in the queue immediately advances into X = -1.25m!
    // =========================================================================
    this.conveyor3Group = new THREE.Group();
    this.conveyor3Group.name = "Conveyor3_CartonBox";

    const centerX = -1.55;
    const lengthX = 4.10; // Spans X = -3.60m to X = +0.50m behind the 6-DOF Robot Arm
    const widthZ = 0.52;
    const height = 0.72;
    const centerZ = -1.45;

    // Stainless Steel Side Frame Bed
    const bedGeo = new THREE.BoxGeometry(lengthX, 0.12, widthZ);
    const bedMesh = new THREE.Mesh(bedGeo, this.matInox);
    bedMesh.position.set(centerX, height - 0.06, centerZ);
    bedMesh.castShadow = true;
    bedMesh.receiveShadow = true;
    this.conveyor3Group.add(bedMesh);

    // 14 Chrome Rollers along the X axis (giảm số con lăn chi tiết để nhẹ CPU trên máy văn phòng)
    this.rollers3 = [];
    const rollerGeo = new THREE.CylinderGeometry(0.020, 0.020, widthZ - 0.05, 8);
    rollerGeo.rotateX(Math.PI / 2);
    for (let i = 0; i < 14; i++) {
      const rx = centerX - lengthX / 2 + (i + 0.5) * (lengthX / 14);
      const roller = new THREE.Mesh(rollerGeo, this.matInox);
      roller.position.set(rx, height + 0.015, centerZ);
      this.conveyor3Group.add(roller);
      this.rollers3.push(roller);
    }

    // Front & Back Guide Rails along X
    const railGeo = new THREE.BoxGeometry(lengthX, 0.07, 0.02);
    const railF = new THREE.Mesh(railGeo, this.matInox);
    railF.position.set(centerX, height + 0.035, centerZ + widthZ / 2 - 0.01);
    const railB = new THREE.Mesh(railGeo, this.matInox);
    railB.position.set(centerX, height + 0.035, centerZ - widthZ / 2 + 0.01);
    this.conveyor3Group.add(railF, railB);

    // Support Legs for Rear Carton Conveyor
    [-3.30, -2.15, -0.95, 0.25].forEach(lx => {
      const legHeight = height - 0.12;
      for (let oz of [-widthZ / 2 + 0.05, widthZ / 2 - 0.05]) {
        const leg = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.03, legHeight, 16), this.matInox);
        leg.position.set(lx, legHeight / 2, centerZ + oz);
        leg.castShadow = true;
        this.conveyor3Group.add(leg);
      }
    });

    // =========================================================================
    // PNEUMATIC STOPPER CYLINDER & SIDE GUIDE CLAMP ASSEMBLY (Trạm đón Robot Arm)
    // Precise carton box locating station at X = -1.25m, Z = -1.45m
    // =========================================================================
    this.stopperStationGroup = new THREE.Group();
    this.stopperStationGroup.name = "PneumaticBoxStopperStation";

    // 1. Heavy-duty Pneumatic Stopper Cylinder at X = -1.40m (Stops front rim of box)
    const stopperBaseX = -1.40;
    const stopperBaseZ = centerZ;

    // Heavy Mounting Bracket bolted between conveyor rails
    const mountPlate = new THREE.Mesh(new THREE.BoxGeometry(0.12, 0.02, 0.26), this.matSafetyBlack);
    mountPlate.position.set(stopperBaseX, height - 0.14, stopperBaseZ);
    this.stopperStationGroup.add(mountPlate);

    // Aluminum Cylinder Body (ISO standard profile with 4 tie rod bolts)
    const cylBody = new THREE.Mesh(new THREE.BoxGeometry(0.065, 0.12, 0.065), this.matPneumaticBody);
    cylBody.position.set(stopperBaseX, height - 0.07, stopperBaseZ);
    this.stopperStationGroup.add(cylBody);

    // Blue Polyurethane Air Tube Elbow Fittings
    const fitting1 = new THREE.Mesh(new THREE.CylinderGeometry(0.007, 0.007, 0.015, 10), this.matAirTubeBlue);
    fitting1.rotateZ(Math.PI / 2);
    fitting1.position.set(stopperBaseX + 0.038, height - 0.03, stopperBaseZ);
    const fitting2 = new THREE.Mesh(new THREE.CylinderGeometry(0.007, 0.007, 0.015, 10), this.matAirTubeBlue);
    fitting2.rotateZ(Math.PI / 2);
    fitting2.position.set(stopperBaseX + 0.038, height - 0.11, stopperBaseZ);
    this.stopperStationGroup.add(fitting1, fitting2);

    // Moving Piston Rod & Stopper Head Assembly (translates vertically)
    this.stopperPistonGroup = new THREE.Group();
    this.stopperPistonGroup.position.set(stopperBaseX, this.stopperCurrentY, stopperBaseZ);

    // Mirror-polished Chrome Piston Rod
    const pistonRod = new THREE.Mesh(new THREE.CylinderGeometry(0.014, 0.014, 0.11, 16), this.matPistonRod);
    pistonRod.position.y = -0.045;
    this.stopperPistonGroup.add(pistonRod);

    // Heavy Hardened Stopper Head Bracket
    const stopHeadBlock = new THREE.Mesh(new THREE.BoxGeometry(0.036, 0.045, 0.24), this.matSafetyYellow);
    stopHeadBlock.position.y = 0.005;
    this.stopperPistonGroup.add(stopHeadBlock);

    // Free-wheeling Contact Roller Pin along Z (stops carton box without wear)
    const rollerPinGeo = new THREE.CylinderGeometry(0.014, 0.014, 0.22, 16);
    rollerPinGeo.rotateX(Math.PI / 2);
    const rollerPin = new THREE.Mesh(rollerPinGeo, this.matChromeRod || this.matPistonRod);
    rollerPin.position.set(0.012, 0.012, 0);
    this.stopperPistonGroup.add(rollerPin);

    this.stopperStationGroup.add(this.stopperPistonGroup);
    // Backward compatibility alias
    this.boxStopper = this.stopperPistonGroup;

    // 2. Dual Side Guide Clamps (Hai má kẹp dẫn hướng khí nén at X = -1.25m)
    const clampStationX = -1.25;

    // A. Front Guide Clamp (Z = -1.18m, moves inward towards -Z)
    this.frontClampMount = new THREE.Group();
    this.frontClampMount.position.set(clampStationX, height + 0.04, centerZ + widthZ / 2 + 0.015);
    const frontCyl = new THREE.Mesh(new THREE.BoxGeometry(0.12, 0.05, 0.06), this.matPneumaticBody);
    this.frontClampMount.add(frontCyl);
    this.stopperStationGroup.add(this.frontClampMount);

    this.frontClampGroup = new THREE.Group();
    this.frontClampGroup.position.set(clampStationX, height + 0.04, this.frontClampCurrentZ);
    // 2 Chrome Guide Rods
    for (const dx of [-0.04, 0.04]) {
      const gRod = new THREE.Mesh(new THREE.CylinderGeometry(0.006, 0.006, 0.08, 12), this.matPistonRod);
      gRod.rotateX(Math.PI / 2);
      gRod.position.set(dx, 0, 0.03);
      this.frontClampGroup.add(gRod);
    }
    // High-friction Polyurethane Clamp Jaw Pad with lead-in chamfer
    const fJawGeo = new THREE.BoxGeometry(0.24, 0.042, 0.015);
    const fJaw = new THREE.Mesh(fJawGeo, this.matClampUrethane);
    this.frontClampGroup.add(fJaw);
    this.stopperStationGroup.add(this.frontClampGroup);

    // B. Rear Guide Clamp (Z = -1.72m, moves inward towards +Z)
    this.rearClampMount = new THREE.Group();
    this.rearClampMount.position.set(clampStationX, height + 0.04, centerZ - widthZ / 2 - 0.015);
    const rearCyl = new THREE.Mesh(new THREE.BoxGeometry(0.12, 0.05, 0.06), this.matPneumaticBody);
    this.rearClampMount.add(rearCyl);
    this.stopperStationGroup.add(this.rearClampMount);

    this.rearClampGroup = new THREE.Group();
    this.rearClampGroup.position.set(clampStationX, height + 0.04, this.rearClampCurrentZ);
    // 2 Chrome Guide Rods
    for (const dx of [-0.04, 0.04]) {
      const gRod = new THREE.Mesh(new THREE.CylinderGeometry(0.006, 0.006, 0.08, 12), this.matPistonRod);
      gRod.rotateX(Math.PI / 2);
      gRod.position.set(dx, 0, -0.03);
      this.rearClampGroup.add(gRod);
    }
    // High-friction Polyurethane Clamp Jaw Pad
    const rJawGeo = new THREE.BoxGeometry(0.24, 0.042, 0.015);
    const rJaw = new THREE.Mesh(rJawGeo, this.matClampUrethane);
    this.rearClampGroup.add(rJaw);
    this.stopperStationGroup.add(this.rearClampGroup);

    this.conveyor3Group.add(this.stopperStationGroup);
    this.root.add(this.conveyor3Group);
  }

  buildVisionStation() {
    // =========================================================================
    // Cognex In-Sight Smart Camera with Volumetric Green Laser Scanner
    // Mounted over Infeed Lane at X = +2.8
    // =========================================================================
    this.visionGroup = new THREE.Group();
    this.visionGroup.position.set(2.8, 0, -0.32);
    this.visionGroup.name = "VisionStation_Cognex";

    const archHeight = 1.48;
    const archWidth = 0.52;

    // Upright posts
    const postGeo = new THREE.BoxGeometry(0.04, archHeight - 0.85, 0.04);
    const p1 = new THREE.Mesh(postGeo, this.matWallFrame);
    p1.position.set(0, 0.85 + (archHeight - 0.85) / 2, archWidth / 2);
    const p2 = new THREE.Mesh(postGeo, this.matWallFrame);
    p2.position.set(0, 0.85 + (archHeight - 0.85) / 2, -archWidth / 2);
    this.visionGroup.add(p1, p2);

    // Cross beam
    const beam = new THREE.Mesh(new THREE.BoxGeometry(0.06, 0.04, archWidth + 0.04), this.matWallFrame);
    beam.position.set(0, archHeight, 0);
    this.visionGroup.add(beam);

    // Cognex Camera Housing
    const camHousing = new THREE.Group();
    camHousing.position.set(0, archHeight - 0.08, 0);

    const camBody = new THREE.Mesh(new THREE.BoxGeometry(0.08, 0.12, 0.08), this.matWallFrame);
    const blueBand = new THREE.Mesh(new THREE.BoxGeometry(0.082, 0.02, 0.082), new THREE.MeshBasicMaterial({ color: 0x0284c7 }));
    blueBand.position.y = 0.02;
    const ringLed = new THREE.Mesh(new THREE.CylinderGeometry(0.035, 0.035, 0.02, 20), new THREE.MeshBasicMaterial({ color: 0xffffff }));
    ringLed.position.y = -0.065;

    this.camStatusLed = new THREE.Mesh(new THREE.SphereGeometry(0.008, 12, 12), new THREE.MeshBasicMaterial({ color: 0x22c55e }));
    this.camStatusLed.position.set(0.042, 0.04, 0);

    camHousing.add(camBody, blueBand, ringLed, this.camStatusLed);
    this.visionGroup.add(camHousing);

    // Green Laser Triangulation Sheet
    const laserPlaneGeo = new THREE.BufferGeometry();
    const topY = archHeight - 0.145;
    const btmY = 0.865;
    const spanZ = 0.38;

    const vertices = new Float32Array([
      0, topY, -0.02,
      0, btmY, -spanZ / 2,
      0, btmY,  spanZ / 2,

      0, topY,  0.02,
      0, btmY,  spanZ / 2,
      0, topY, -0.02
    ]);
    laserPlaneGeo.setAttribute('position', new THREE.BufferAttribute(vertices, 3));

    const laserMat = new THREE.MeshBasicMaterial({
      color: 0x00ff88,
      transparent: true,
      opacity: 0.35,
      side: THREE.DoubleSide,
      blending: THREE.AdditiveBlending,
      depthWrite: false
    });

    this.laserSheet = new THREE.Mesh(laserPlaneGeo, laserMat);
    this.visionGroup.add(this.laserSheet);

    // Green Laser line on belt
    const laserLine = new THREE.Mesh(
      new THREE.PlaneGeometry(0.008, spanZ),
      new THREE.MeshBasicMaterial({ color: 0x22c55e, blending: THREE.AdditiveBlending })
    );
    laserLine.rotateX(-Math.PI / 2);
    laserLine.position.set(0, 0.868, 0);
    this.visionGroup.add(laserLine);

    this.root.add(this.visionGroup);
  }

  // ===========================================================================
  // QUALITY CONTROL REJECT PUSHER & NG COLLECTION BIN (CONVEYOR 1)
  // Positioned at X = +2.45m (~350mm downstream of Cognex at X = 2.80m)
  // ===========================================================================
  buildPneumaticRejectPusherAndNGBin() {
    this.qcStationGroup = new THREE.Group();
    this.qcStationGroup.name = "QCStation_RejectPusher_NGBin";
    const posX = 2.45;
    const beltTopY = 0.865;

    // -------------------------------------------------------------------------
    // 1. Pneumatic Pusher Cylinder Assembly (Mounted on divider frame at Z = -0.05)
    // -------------------------------------------------------------------------
    this.rejectPusherGroup = new THREE.Group();
    this.rejectPusherGroup.position.set(posX, beltTopY + 0.02, 0);

    // Aluminum mounting bracket fixed to central divider rail
    const bracket = new THREE.Mesh(new THREE.BoxGeometry(0.12, 0.07, 0.04), this.matSafetyBlack);
    bracket.position.set(0, 0, -0.05);
    bracket.castShadow = true;
    this.rejectPusherGroup.add(bracket);

    // Pneumatic ISO compact cylinder body (oriented along Z axis)
    const cylLength = 0.14;
    const cylBody = new THREE.Mesh(new THREE.BoxGeometry(0.06, 0.06, cylLength), this.matPneumaticBody);
    cylBody.position.set(0, 0, -0.05 + cylLength / 2);
    cylBody.castShadow = true;
    this.rejectPusherGroup.add(cylBody);

    // 2 Quick-connect air fittings (Blue PU tubing connections)
    const fitGeo = new THREE.CylinderGeometry(0.007, 0.007, 0.016, 10);
    fitGeo.rotateZ(Math.PI / 2);
    const fitFront = new THREE.Mesh(fitGeo, this.matAirTubeBlue);
    fitFront.position.set(0.035, 0.018, -0.03);
    const fitRear = new THREE.Mesh(fitGeo, this.matAirTubeBlue);
    fitRear.position.set(0.035, 0.018, 0.07);
    this.rejectPusherGroup.add(fitFront, fitRear);

    // Magnetic position reed sensor with bi-color LED
    const sensorBody = new THREE.Mesh(new THREE.BoxGeometry(0.015, 0.012, 0.03), this.matSafetyBlack);
    sensorBody.position.set(-0.032, 0.02, 0.02);
    this.rejectCylinderLed = new THREE.Mesh(
      new THREE.SphereGeometry(0.005, 8, 8),
      new THREE.MeshBasicMaterial({ color: 0x22c55e })
    );
    this.rejectCylinderLed.position.set(-0.04, 0.02, 0.02);
    this.rejectPusherGroup.add(sensorBody, this.rejectCylinderLed);

    // Moving Piston Rod & Flat Pusher Plate Assembly
    this.pusherRodGroup = new THREE.Group();
    this.pusherRodGroup.position.set(0, 0, this.pusherCurrentZ); // starts at -0.13m

    // Chrome-plated piston rod extending into -Z
    const rodGeo = new THREE.CylinderGeometry(0.009, 0.009, 0.32, 16);
    rodGeo.rotateX(Math.PI / 2);
    const rod = new THREE.Mesh(rodGeo, this.matPistonRod);
    rod.position.set(0, 0, 0.16);
    this.pusherRodGroup.add(rod);

    // Horizontal Flat Pusher Plate (Bàn gạt nhôm phẳng sát mặt băng tải)
    const plateW = 0.16; // along X
    const plateH = 0.042; // along Y
    const plateT = 0.012; // along Z
    const plate = new THREE.Mesh(new THREE.BoxGeometry(plateW, plateH, plateT), this.matInox);
    plate.position.set(0, 0, 0);
    plate.castShadow = true;
    this.pusherRodGroup.add(plate);

    // Rubber bumper face on pusher plate
    const bumper = new THREE.Mesh(new THREE.BoxGeometry(plateW - 0.004, plateH - 0.004, 0.003), this.matSafetyBlack);
    bumper.position.set(0, 0, -plateT / 2 - 0.0015);
    this.pusherRodGroup.add(bumper);

    this.rejectPusherGroup.add(this.pusherRodGroup);
    this.qcStationGroup.add(this.rejectPusherGroup);

    // -------------------------------------------------------------------------
    // 2. Galvanized 35° Guide Chute & Industrial Wheeled NG Scrap Bin (Z = -0.52m to -0.82m)
    // -------------------------------------------------------------------------
    // Angled Galvanized Sheet-Metal Guide Chute (Nghiêng chéo 35 độ nối từ mép đai Băng tải 1 vào miệng thùng)
    const chuteGroup = new THREE.Group();
    chuteGroup.name = "RejectGuideChute_35Deg";
    chuteGroup.position.set(posX, beltTopY, -0.52);

    const chuteSlopeRad = THREE.MathUtils.degToRad(35); // 35° downward slope
    const chuteLen = 0.26;
    const chuteGeo = new THREE.BoxGeometry(0.24, 0.008, chuteLen);
    chuteGeo.rotateX(-chuteSlopeRad);
    const chuteBed = new THREE.Mesh(chuteGeo, this.matInox);
    chuteBed.position.set(0, -Math.sin(chuteSlopeRad) * (chuteLen / 2), -Math.cos(chuteSlopeRad) * (chuteLen / 2));
    chuteBed.castShadow = true;
    chuteBed.receiveShadow = true;
    chuteGroup.add(chuteBed);

    // Chute Side Guard Skirts (35° angled side walls to guide rejected pieces into bin)
    const sideSkGeo = new THREE.BoxGeometry(0.008, 0.055, chuteLen);
    sideSkGeo.rotateX(-chuteSlopeRad);
    const sideL = new THREE.Mesh(sideSkGeo, this.matInox);
    sideL.position.set(-0.12, -Math.sin(chuteSlopeRad) * (chuteLen / 2) + 0.02, -Math.cos(chuteSlopeRad) * (chuteLen / 2));
    const sideR = new THREE.Mesh(sideSkGeo, this.matInox);
    sideR.position.set(0.12, -Math.sin(chuteSlopeRad) * (chuteLen / 2) + 0.02, -Math.cos(chuteSlopeRad) * (chuteLen / 2));
    chuteGroup.add(sideL, sideR);
    this.qcStationGroup.add(chuteGroup);

    // Heavy-duty Industrial NG Scrap Bin on 4 Swivel Caster Wheels with "NG / SCRAP ONLY" Red Cross Decal
    const binGroup = new THREE.Group();
    binGroup.name = "NGScrapBin_Wheeled";
    binGroup.position.set(posX, 0.42, -0.76);

    const binW = 0.34;
    const binH = 0.34;
    const binD = 0.34;
    const bWall = 0.014;

    // Bin Bottom (Interior floor at world Y = 0.42 - 0.17 + 0.014 = 0.264m)
    const bBtm = new THREE.Mesh(new THREE.BoxGeometry(binW, bWall, binD), this.matNGBinYellow);
    bBtm.position.y = -binH / 2 + bWall / 2;
    bBtm.castShadow = true;
    bBtm.receiveShadow = true;
    binGroup.add(bBtm);

    // Inner dark rubber cushion mat at bottom of NG bin
    const innerMat = new THREE.Mesh(new THREE.BoxGeometry(binW - 0.03, 0.006, binD - 0.03), this.matSafetyBlack);
    innerMat.position.y = -binH / 2 + bWall + 0.003;
    binGroup.add(innerMat);

    // 4 Walls (#eab308 Industrial Warning Yellow with Dark Slate Reinforced Rim)
    const wFront = new THREE.Mesh(new THREE.BoxGeometry(binW, binH, bWall), this.matNGBinYellow);
    wFront.position.set(0, 0, binD / 2 - bWall / 2);
    wFront.castShadow = true;
    const wBack = new THREE.Mesh(new THREE.BoxGeometry(binW, binH, bWall), this.matNGBinYellow);
    wBack.position.set(0, 0, -binD / 2 + bWall / 2);
    wBack.castShadow = true;
    const wLeft = new THREE.Mesh(new THREE.BoxGeometry(bWall, binH, binD - 2 * bWall), this.matNGBinYellow);
    wLeft.position.set(-binW / 2 + bWall / 2, 0, 0);
    wLeft.castShadow = true;
    const wRight = new THREE.Mesh(new THREE.BoxGeometry(bWall, binH, binD - 2 * bWall), this.matNGBinYellow);
    wRight.position.set(binW / 2 - bWall / 2, 0, 0);
    wRight.castShadow = true;
    binGroup.add(wFront, wBack, wLeft, wRight);

    // Reinforced Top Lip Collar (#334155 Dark Slate)
    const lipF = new THREE.Mesh(new THREE.BoxGeometry(binW + 0.016, 0.022, 0.02), this.matSealerAccents);
    lipF.position.set(0, binH / 2, binD / 2 - 0.005);
    const lipB = new THREE.Mesh(new THREE.BoxGeometry(binW + 0.016, 0.022, 0.02), this.matSealerAccents);
    lipB.position.set(0, binH / 2, -binD / 2 + 0.005);
    binGroup.add(lipF, lipB);

    // Black Industrial Push Handles on Left & Right Sides
    const hGeo = new THREE.BoxGeometry(0.024, 0.045, 0.08);
    const hL = new THREE.Mesh(hGeo, this.matSafetyBlack);
    hL.position.set(-binW / 2 - 0.012, binH / 4, 0);
    const hR = new THREE.Mesh(hGeo, this.matSafetyBlack);
    hR.position.set(binW / 2 + 0.012, binH / 4, 0);
    binGroup.add(hL, hR);

    // Red Cross & "NG / SCRAP ONLY" Decal Placards on Rear, Left & Right Walls of Bin
    const ngSignTex = this.createNGScrapBinDecalTexture();
    const ngSignMat = new THREE.MeshBasicMaterial({ map: ngSignTex });
    const signBack = new THREE.Mesh(new THREE.PlaneGeometry(0.26, 0.16), ngSignMat);
    signBack.rotation.y = Math.PI;
    signBack.position.set(0, 0.01, -binD / 2 - 0.003);
    const signFront = new THREE.Mesh(new THREE.PlaneGeometry(0.26, 0.16), ngSignMat);
    signFront.position.set(0, 0.01, binD / 2 + 0.003);
    const signRight = new THREE.Mesh(new THREE.PlaneGeometry(0.24, 0.15), ngSignMat);
    signRight.rotation.y = Math.PI / 2;
    signRight.position.set(binW / 2 + 0.003, 0.01, 0);
    binGroup.add(signBack, signFront, signRight);

    // Mobile Caster Frame Base & 4 Industrial Swivel Caster Wheels under the NG Scrap Bin
    const baseFrame = new THREE.Mesh(new THREE.BoxGeometry(binW + 0.02, 0.025, binD + 0.02), this.matSealerAccents);
    baseFrame.position.y = -binH / 2 - 0.012;
    binGroup.add(baseFrame);

    const legH = 0.16;
    const standLegGeo = new THREE.BoxGeometry(0.025, legH, 0.025);
    const wheelGeo = new THREE.CylinderGeometry(0.032, 0.032, 0.022, 16);
    wheelGeo.rotateZ(Math.PI / 2);
    const hubGeo = new THREE.CylinderGeometry(0.016, 0.016, 0.025, 12);
    hubGeo.rotateZ(Math.PI / 2);

    const legPositions = [
      [-binW / 2 + 0.035, -binD / 2 + 0.035],
      [ binW / 2 - 0.035, -binD / 2 + 0.035],
      [-binW / 2 + 0.035,  binD / 2 - 0.035],
      [ binW / 2 - 0.035,  binD / 2 - 0.035]
    ];
    legPositions.forEach(([lx, lz]) => {
      const strut = new THREE.Mesh(standLegGeo, this.matSealerAccents);
      strut.position.set(lx, -binH / 2 - 0.025 - legH / 2, lz);
      strut.castShadow = true;
      binGroup.add(strut);

      // Swivel caster fork + wheel
      const wheel = new THREE.Mesh(wheelGeo, this.matSafetyBlack);
      wheel.position.set(lx, -binH / 2 - 0.025 - legH - 0.018, lz);
      const hub = new THREE.Mesh(hubGeo, this.matInox);
      hub.position.copy(wheel.position);
      binGroup.add(wheel, hub);
    });

    this.qcStationGroup.add(binGroup);
    this.root.add(this.qcStationGroup);
  }

  /**
   * Creates the "NG / SCRAP ONLY" Red Cross Sign Texture for the NG Scrap Bin
   */
  createNGScrapBinDecalTexture() {
    const canvas = document.createElement('canvas');
    canvas.width = 256;
    canvas.height = 160;
    const ctx = canvas.getContext('2d');

    // Dark industrial plate background with yellow/red border
    ctx.fillStyle = '#0f172a';
    ctx.fillRect(0, 0, 256, 160);
    ctx.strokeStyle = '#ef4444';
    ctx.lineWidth = 8;
    ctx.strokeRect(4, 4, 248, 152);

    // Red Cross / X Badge on the left
    ctx.fillStyle = '#ef4444';
    ctx.beginPath();
    ctx.arc(54, 80, 34, 0, Math.PI * 2);
    ctx.fill();

    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 9;
    ctx.beginPath();
    ctx.moveTo(38, 64); ctx.lineTo(70, 96);
    ctx.moveTo(70, 64); ctx.lineTo(38, 96);
    ctx.stroke();

    // Bold Text: "NG / SCRAP ONLY"
    ctx.fillStyle = '#ef4444';
    ctx.font = '900 24px Arial, sans-serif';
    ctx.fillText('NG / SCRAP', 100, 68);
    ctx.fillStyle = '#facc15';
    ctx.font = '900 22px Arial, sans-serif';
    ctx.fillText('ONLY ZONE', 100, 98);
    ctx.fillStyle = '#cbd5e1';
    ctx.font = 'bold 13px Consolas, monospace';
    ctx.fillText('AUTO PNEUMATIC REJECT', 98, 128);

    return new THREE.CanvasTexture(canvas);
  }

  // ===========================================================================
  // AUTOMATIC TOP CARTON SEALER (MÁY DÁN BĂNG KEO NẮP THÙNG) (CONVEYOR 3)
  // Positioned on Conveyor 3 at X = -2.40m, Z = -1.45m
  // ===========================================================================
  buildTopCartonSealer() {
    this.cartonSealerGroup = new THREE.Group();
    this.cartonSealerGroup.name = "AutomaticTopCartonSealer";
    const posX = -2.40;
    const centerZ = -1.45;
    const rollerY = 0.74;
    const gantryH = 1.34;
    const spanZ = 0.54;

    this.cartonSealerGroup.position.set(posX, 0, centerZ);

    // 1. Heavy Industrial Portal Gantry Frame (Sky Blue RAL 5015)
    const colGeo = new THREE.BoxGeometry(0.06, gantryH, 0.06);
    const colFront = new THREE.Mesh(colGeo, this.matSealerFrame);
    colFront.position.set(0, gantryH / 2, spanZ / 2);
    colFront.castShadow = true;

    const colRear = new THREE.Mesh(colGeo, this.matSealerFrame);
    colRear.position.set(0, gantryH / 2, -spanZ / 2);
    colRear.castShadow = true;

    // Base Floor Mounting Plates
    const baseGeo = new THREE.BoxGeometry(0.12, 0.015, 0.12);
    const bPlateF = new THREE.Mesh(baseGeo, this.matSafetyBlack);
    bPlateF.position.set(0, 0.0075, spanZ / 2);
    const bPlateR = new THREE.Mesh(baseGeo, this.matSafetyBlack);
    bPlateR.position.set(0, 0.0075, -spanZ / 2);

    // Top Cross Beam
    const crossBeam = new THREE.Mesh(new THREE.BoxGeometry(0.08, 0.07, spanZ + 0.08), this.matSealerFrame);
    crossBeam.position.set(0, gantryH - 0.035, 0);
    crossBeam.castShadow = true;

    this.cartonSealerGroup.add(colFront, colRear, bPlateF, bPlateR, crossBeam);

    // Height Adjustment Threaded Spindle & Handwheel
    const spindle = new THREE.Mesh(new THREE.CylinderGeometry(0.012, 0.012, 0.32, 14), this.matInox);
    spindle.position.set(0, gantryH - 0.12, 0);
    const handwheel = new THREE.Mesh(new THREE.TorusGeometry(0.055, 0.012, 8, 20), this.matRollerOrange);
    handwheel.rotateX(Math.PI / 2);
    handwheel.position.set(0, gantryH + 0.05, 0);
    this.cartonSealerGroup.add(spindle, handwheel);

    // 2. Flap Folding Plows (Thanh nẹp dẫn hướng gập cánh nắp thùng)
    // Symmetrical stainless steel plows extending upstream (+X direction)
    const plowGeo = new THREE.BoxGeometry(0.24, 0.015, 0.02);
    plowGeo.rotateZ(Math.PI * 0.09); // Angle down toward sealer center

    const plowFront = new THREE.Mesh(plowGeo, this.matInox);
    plowFront.position.set(0.12, rollerY + 0.20, 0.12);
    plowFront.castShadow = true;

    const plowRear = new THREE.Mesh(plowGeo, this.matInox);
    plowRear.position.set(0.12, rollerY + 0.20, -0.12);
    plowRear.castShadow = true;
    this.cartonSealerGroup.add(plowFront, plowRear);

    // 3. Taping Head Unit (Cụm đầu dán băng keo nắp thùng)
    const headGroup = new THREE.Group();
    headGroup.position.set(0, rollerY + 0.19, 0);

    // Side mounting steel chassis plates
    const plateGeo = new THREE.BoxGeometry(0.24, 0.14, 0.008);
    const plLeft = new THREE.Mesh(plateGeo, this.matSealerAccents);
    plLeft.position.set(0, 0.07, 0.045);
    const plRight = new THREE.Mesh(plateGeo, this.matSealerAccents);
    plRight.position.set(0, 0.07, -0.045);
    headGroup.add(plLeft, plRight);

    // Translucent Amber Packaging Tape Roll
    const tapeRollGeo = new THREE.CylinderGeometry(0.055, 0.055, 0.048, 24);
    tapeRollGeo.rotateX(Math.PI / 2);
    const tapeRoll = new THREE.Mesh(tapeRollGeo, this.matTapeAmber);
    tapeRoll.position.set(0, 0.13, 0);
    const tapeCore = new THREE.Mesh(new THREE.CylinderGeometry(0.025, 0.025, 0.052, 16), this.matSafetyBlack);
    tapeCore.rotateX(Math.PI / 2);
    tapeCore.position.set(0, 0.13, 0);
    headGroup.add(tapeRoll, tapeCore);

    // Orange Rubber Front Application Roller
    const rGeo = new THREE.CylinderGeometry(0.022, 0.022, 0.065, 18);
    rGeo.rotateX(Math.PI / 2);
    const rFront = new THREE.Mesh(rGeo, this.matRollerOrange);
    rFront.position.set(0.08, -0.005, 0);
    rFront.castShadow = true;

    // Orange Rubber Rear Wipe-Down Roller
    const rRear = new THREE.Mesh(rGeo, this.matRollerOrange);
    rRear.position.set(-0.08, -0.005, 0);
    rRear.castShadow = true;

    headGroup.add(rFront, rRear);
    this.cartonSealerGroup.add(headGroup);

    // 4. Photo Sensor Pair at Sealer Entrance (DI_SEALER_SENSOR at X = -2.15m -> relative +0.25m)
    const sensorX = 0.25;
    const sensorY = rollerY + 0.10;

    const sensBodyGeo = new THREE.BoxGeometry(0.025, 0.035, 0.035);
    const sensEmitter = new THREE.Mesh(sensBodyGeo, this.matSafetyBlack);
    sensEmitter.position.set(sensorX, sensorY, spanZ / 2 - 0.01);

    const sensReceiver = new THREE.Mesh(sensBodyGeo, this.matSafetyBlack);
    sensReceiver.position.set(sensorX, sensorY, -spanZ / 2 + 0.01);

    this.sealerSensorLed = new THREE.Mesh(
      new THREE.SphereGeometry(0.006, 10, 10),
      new THREE.MeshBasicMaterial({ color: 0x22c55e })
    );
    this.sealerSensorLed.position.set(sensorX - 0.015, sensorY + 0.015, spanZ / 2 - 0.01);

    // Red Optical Sensing Beam across Conveyor 3
    const beamGeo = new THREE.CylinderGeometry(0.0018, 0.0018, spanZ - 0.04, 8);
    beamGeo.rotateX(Math.PI / 2);
    this.sealerBeamMat = new THREE.MeshBasicMaterial({
      color: 0xff0033,
      transparent: true,
      opacity: 0.45,
      blending: THREE.AdditiveBlending
    });
    this.sealerBeamMesh = new THREE.Mesh(beamGeo, this.sealerBeamMat);
    this.sealerBeamMesh.position.set(sensorX, sensorY, 0);

    this.cartonSealerGroup.add(sensEmitter, sensReceiver, this.sealerSensorLed, this.sealerBeamMesh);

    this.root.add(this.cartonSealerGroup);
  }

  // ===========================================================================
  // CONVEYOR 1 ROTARY ENCODER (BỘ MÃ HÓA XUNG BÁM BĂNG TẢI CHO ROBOT DELTA)
  // Coaxially mounted to Conveyor 1 Drive Pulley with Flexible Jaw Coupling
  // ===========================================================================
  buildConveyor1RotaryEncoder() {
    this.encoderGroup = new THREE.Group();
    this.encoderGroup.name = "Conveyor1_RotaryEncoder";

    // Placed at X = +3.12m, Y = 0.815m, Z = -0.32m (Conveyor 1 Drive Pulley axis)
    const pulleyX = 3.12;
    const pulleyY = 0.815;
    const beltCenterZ = -0.32;
    const outerSkirtZ = -0.53;

    this.encoderGroup.position.set(pulleyX, pulleyY, 0);

    // 1. Rotating Assembly: Drive Pulley Drum + Extended Shaft + Flexible Coupling
    this.encoderRotatingShaftGroup = new THREE.Group();
    this.encoderRotatingShaftGroup.position.set(0, 0, 0);

    // Drive Pulley Roller under Belt 1
    const drumGeo = new THREE.CylinderGeometry(0.042, 0.042, 0.39, 20);
    drumGeo.rotateX(Math.PI / 2);
    const drumMesh = new THREE.Mesh(drumGeo, this.matInox);
    drumMesh.position.set(0, 0, beltCenterZ);
    this.encoderRotatingShaftGroup.add(drumMesh);

    // Coaxial Steel Drive Shaft protruding through outer side apron (Z = -0.50m to -0.68m)
    const shaftGeo = new THREE.CylinderGeometry(0.014, 0.014, 0.24, 16);
    shaftGeo.rotateX(Math.PI / 2);
    const shaftMesh = new THREE.Mesh(shaftGeo, this.matPistonRod);
    shaftMesh.position.set(0, 0, outerSkirtZ - 0.06);
    this.encoderRotatingShaftGroup.add(shaftMesh);

    // Flexible Jaw Coupling (Khớp nối mềm nhôm xẻ rãnh + đệm giảm chấn đỏ)
    const coupHubGeo = new THREE.CylinderGeometry(0.024, 0.024, 0.018, 16);
    coupHubGeo.rotateX(Math.PI / 2);
    const coupHub1 = new THREE.Mesh(coupHubGeo, this.matPneumaticBody);
    coupHub1.position.set(0, 0, outerSkirtZ - 0.055);
    const coupHub2 = new THREE.Mesh(coupHubGeo, this.matPneumaticBody);
    coupHub2.position.set(0, 0, outerSkirtZ - 0.085);

    // Red Elastomer Spider Insert in middle of coupling
    const spiderGeo = new THREE.CylinderGeometry(0.025, 0.025, 0.012, 12);
    spiderGeo.rotateX(Math.PI / 2);
    const spiderMesh = new THREE.Mesh(spiderGeo, this.matInterlockRed);
    spiderMesh.position.set(0, 0, outerSkirtZ - 0.070);

    // High-contrast rotation index fins on coupling so rotation is clearly visible
    for (let k = 0; k < 4; k++) {
      const fin = new THREE.Mesh(new THREE.BoxGeometry(0.052, 0.006, 0.038), this.matSafetyBlack);
      fin.rotation.z = (k * Math.PI) / 2;
      fin.position.set(0, 0, outerSkirtZ - 0.070);
      this.encoderRotatingShaftGroup.add(fin);
    }

    this.encoderRotatingShaftGroup.add(coupHub1, coupHub2, spiderMesh);
    this.encoderGroup.add(this.encoderRotatingShaftGroup);

    // 2. Pillow-Block Bearing Flange on Conveyor Skirt (Z = -0.53m)
    const bearingFlange = new THREE.Mesh(new THREE.BoxGeometry(0.09, 0.09, 0.02), this.matSafetyBlack);
    bearingFlange.position.set(0, 0, outerSkirtZ - 0.01);
    this.encoderGroup.add(bearingFlange);

    // Anti-rotation Spring Tether Bracket mounting encoder to conveyor frame
    const tetherBracket = new THREE.Mesh(new THREE.BoxGeometry(0.06, 0.04, 0.14), this.matInox);
    tetherBracket.position.set(0.045, 0.01, outerSkirtZ - 0.08);
    this.encoderGroup.add(tetherBracket);

    // 3. Industrial Optical Rotary Encoder Housing (Vỏ nhôm sáng + mặt bích tròn)
    const encZ = outerSkirtZ - 0.135;
    const encFlangeGeo = new THREE.CylinderGeometry(0.038, 0.038, 0.010, 24);
    encFlangeGeo.rotateX(Math.PI / 2);
    const encFlange = new THREE.Mesh(encFlangeGeo, this.matSafetyBlack);
    encFlange.position.set(0, 0, encZ + 0.032);

    const encBodyGeo = new THREE.CylinderGeometry(0.034, 0.034, 0.060, 24);
    encBodyGeo.rotateX(Math.PI / 2);
    const encBody = new THREE.Mesh(encBodyGeo, this.matInox);
    encBody.position.set(0, 0, encZ);
    encBody.castShadow = true;

    const encRearCapGeo = new THREE.CylinderGeometry(0.032, 0.034, 0.012, 24);
    encRearCapGeo.rotateX(Math.PI / 2);
    const encRearCap = new THREE.Mesh(encRearCapGeo, this.matSafetyBlack);
    encRearCap.position.set(0, 0, encZ - 0.034);

    // Quadrature Pulse Indicator LEDs (Channel A & Channel B)
    this.encoderLedA = new THREE.Mesh(
      new THREE.SphereGeometry(0.0055, 10, 10),
      new THREE.MeshBasicMaterial({ color: 0x22c55e })
    );
    this.encoderLedA.position.set(-0.012, 0.036, encZ - 0.015);

    this.encoderLedB = new THREE.Mesh(
      new THREE.SphereGeometry(0.0055, 10, 10),
      new THREE.MeshBasicMaterial({ color: 0x38bdf8 })
    );
    this.encoderLedB.position.set(0.012, 0.036, encZ - 0.015);

    // 4. Black Rubber Shielded Signal Cable & Gland leading down to PLC conduit
    const gland = new THREE.Mesh(new THREE.CylinderGeometry(0.009, 0.009, 0.025, 12), this.matSafetyBlack);
    gland.position.set(0, -0.038, encZ - 0.02);
    const cable = new THREE.Mesh(new THREE.CylinderGeometry(0.006, 0.006, 0.55, 10), this.matSafetyBlack);
    cable.position.set(0, -0.31, encZ - 0.02);

    this.encoderGroup.add(encFlange, encBody, encRearCap, this.encoderLedA, this.encoderLedB, gland, cable);
    this.root.add(this.encoderGroup);
  }

  // ===========================================================================
  // INDUSTRIAL INKJET PRINTER & BARCODE SCANNER (CONVEYOR 3 OUT-FEED TRACEABILITY)
  // Positioned at X = -3.05m, Z = -1.45m (Immediately downstream of Top Sealer)
  // ===========================================================================
  buildInkjetAndBarcodeStation() {
    this.traceabilityGroup = new THREE.Group();
    this.traceabilityGroup.name = "InkjetPrinter_BarcodeScanner_Station";

    const stationX = -3.05;
    const centerZ = -1.45;
    const rollerY = 0.74;
    const spanZ = 0.52;

    this.traceabilityGroup.position.set(stationX, 0, centerZ);

    // 1. Trigger Photo Sensor (DI_PRINT_TRIGGER at X = -2.84m -> relative +0.21m)
    const trigX = 0.21;
    const trigY = rollerY + 0.09;
    const sensGeo = new THREE.BoxGeometry(0.024, 0.034, 0.032);
    const trigEmitter = new THREE.Mesh(sensGeo, this.matSafetyBlack);
    trigEmitter.position.set(trigX, trigY, spanZ / 2 - 0.01);
    const trigReflector = new THREE.Mesh(sensGeo, this.matSafetyBlack);
    trigReflector.position.set(trigX, trigY, -spanZ / 2 + 0.01);

    this.printTriggerLed = new THREE.Mesh(
      new THREE.SphereGeometry(0.006, 10, 10),
      new THREE.MeshBasicMaterial({ color: 0x22c55e })
    );
    this.printTriggerLed.position.set(trigX, trigY + 0.020, spanZ / 2 - 0.01);

    const trigBeamGeo = new THREE.CylinderGeometry(0.0018, 0.0018, spanZ - 0.04, 8);
    trigBeamGeo.rotateX(Math.PI / 2);
    this.printTriggerBeamMat = new THREE.MeshBasicMaterial({
      color: 0xf59e0b,
      transparent: true,
      opacity: 0.45,
      blending: THREE.AdditiveBlending
    });
    this.printTriggerBeamMesh = new THREE.Mesh(trigBeamGeo, this.printTriggerBeamMat);
    this.printTriggerBeamMesh.position.set(trigX, trigY, 0);

    this.traceabilityGroup.add(trigEmitter, trigReflector, this.printTriggerLed, this.printTriggerBeamMesh);

    // 2. Adjustable Stainless Steel Tubular Mounting Stand (Gắn bên hông trước Băng tải 3)
    const standZ = spanZ / 2 + 0.09; // Z = -1.10m in world
    const standBase = new THREE.Mesh(new THREE.BoxGeometry(0.36, 0.016, 0.14), this.matSafetyBlack);
    standBase.position.set(-0.02, 0.008, standZ);

    // Dual Vertical Inox Columns for Printhead (X = +0.08m) & Barcode Scanner (X = -0.11m)
    const colGeo = new THREE.CylinderGeometry(0.016, 0.016, 0.98, 16);
    const colPrint = new THREE.Mesh(colGeo, this.matInox);
    colPrint.position.set(0.08, 0.49, standZ);
    const colScan = new THREE.Mesh(colGeo, this.matInox);
    colScan.position.set(-0.11, 0.49, standZ);

    // Cross-Clamps & Adjustment Knobs
    const clampGeo = new THREE.BoxGeometry(0.048, 0.048, 0.055);
    const clampPrint = new THREE.Mesh(clampGeo, this.matSafetyBlack);
    clampPrint.position.set(0.08, rollerY + 0.09, standZ);
    const clampScan = new THREE.Mesh(clampGeo, this.matSafetyBlack);
    clampScan.position.set(-0.11, rollerY + 0.11, standZ);

    this.traceabilityGroup.add(standBase, colPrint, colScan, clampPrint, clampScan);

    // 3. Industrial CIJ Inkjet Printhead (Đầu in phun công nghiệp chĩa ngang vào hông thùng)
    const printheadGroup = new THREE.Group();
    printheadGroup.position.set(0.08, rollerY + 0.09, standZ - 0.07);

    // Stainless steel & black anodized body
    const phBody = new THREE.Mesh(new THREE.BoxGeometry(0.045, 0.055, 0.12), this.matInox);
    phBody.castShadow = true;
    const phFrontCap = new THREE.Mesh(new THREE.BoxGeometry(0.047, 0.057, 0.035), this.matSafetyBlack);
    phFrontCap.position.set(0, 0, -0.055);
    const phNozzle = new THREE.Mesh(
      new THREE.CylinderGeometry(0.010, 0.014, 0.025, 14).rotateX(Math.PI / 2),
      this.matRollerOrange
    );
    phNozzle.position.set(0, 0, -0.082);

    this.inkjetFireLed = new THREE.Mesh(
      new THREE.SphereGeometry(0.006, 10, 10),
      new THREE.MeshBasicMaterial({ color: 0x475569 })
    );
    this.inkjetFireLed.position.set(0, 0.032, -0.04);

    // Inkjet High-Speed Micro-Mist Flash Effect (visible when DO_INKJET_FIRE is true)
    const sprayGeo = new THREE.ConeGeometry(0.025, 0.06, 12);
    sprayGeo.rotateX(Math.PI / 2);
    this.inkjetSprayMat = new THREE.MeshBasicMaterial({
      color: 0x38bdf8,
      transparent: true,
      opacity: 0.0,
      blending: THREE.AdditiveBlending
    });
    this.inkjetSprayMesh = new THREE.Mesh(sprayGeo, this.inkjetSprayMat);
    this.inkjetSprayMesh.position.set(0, 0, -0.115);

    printheadGroup.add(phBody, phFrontCap, phNozzle, this.inkjetFireLed, this.inkjetSprayMesh);
    this.traceabilityGroup.add(printheadGroup);

    // 4. Industrial Fixed-Mount Barcode Scanner (Đầu đọc mã vạch tia Laser Đỏ)
    const scannerGroup = new THREE.Group();
    scannerGroup.position.set(-0.11, rollerY + 0.11, standZ - 0.07);
    scannerGroup.rotation.y = -0.18; // Angled slightly toward printed label zone
    scannerGroup.rotation.x = -0.08;

    const scBody = new THREE.Mesh(new THREE.BoxGeometry(0.065, 0.048, 0.085), this.matSafetyBlack);
    scBody.castShadow = true;
    const scAccent = new THREE.Mesh(new THREE.BoxGeometry(0.067, 0.012, 0.087), this.matSealerFrame);
    scAccent.position.set(0, 0.012, 0);
    const scLens = new THREE.Mesh(
      new THREE.BoxGeometry(0.048, 0.028, 0.006),
      new THREE.MeshBasicMaterial({ color: 0x991b1b })
    );
    scLens.position.set(0, -0.002, -0.044);

    // Top confirmation LED bar on Barcode Scanner
    this.barcodeScannerLed = new THREE.Mesh(
      new THREE.BoxGeometry(0.045, 0.008, 0.014),
      new THREE.MeshBasicMaterial({ color: 0x22c55e })
    );
    this.barcodeScannerLed.position.set(0, 0.027, -0.02);

    scannerGroup.add(scBody, scAccent, scLens, this.barcodeScannerLed);
    this.traceabilityGroup.add(scannerGroup);

    // 5. Red Laser Fan Sheet & Scan Line on Carton Side Wall
    const fanGeo = new THREE.BufferGeometry();
    const fanVerts = new Float32Array([
      -0.11, rollerY + 0.108, standZ - 0.11,
      -0.21, rollerY + 0.095, 0.122,
      -0.01, rollerY + 0.095, 0.122
    ]);
    fanGeo.setAttribute('position', new THREE.BufferAttribute(fanVerts, 3));
    this.barcodeLaserFanMat = new THREE.MeshBasicMaterial({
      color: 0xff0022,
      transparent: true,
      opacity: 0.0,
      side: THREE.DoubleSide,
      blending: THREE.AdditiveBlending,
      depthWrite: false
    });
    this.barcodeLaserFan = new THREE.Mesh(fanGeo, this.barcodeLaserFanMat);

    // Bright Red Laser Scan Line projected onto box side wall
    this.barcodeLaserLineMat = new THREE.MeshBasicMaterial({
      color: 0xff1133,
      transparent: true,
      opacity: 0.0,
      blending: THREE.AdditiveBlending
    });
    this.barcodeLaserLine = new THREE.Mesh(
      new THREE.BoxGeometry(0.20, 0.005, 0.004),
      this.barcodeLaserLineMat
    );
    this.barcodeLaserLine.position.set(-0.11, rollerY + 0.095, 0.123);

    this.traceabilityGroup.add(this.barcodeLaserFan, this.barcodeLaserLine);
    this.root.add(this.traceabilityGroup);
  }

  /**
   * Generates a crisp industrial CIJ dot-matrix & barcode texture for the carton side wall
   */
  createBoxPrintTexture(boxId, mfgDate, expDate, lotCode) {
    const canvas = document.createElement('canvas');
    canvas.width = 512;
    canvas.height = 256;
    const ctx = canvas.getContext('2d');

    // Light kraft label backing with subtle industrial border
    ctx.fillStyle = '#e5ccb0';
    ctx.fillRect(0, 0, 512, 256);
    ctx.strokeStyle = '#1e293b';
    ctx.lineWidth = 6;
    ctx.strokeRect(6, 6, 500, 244);

    // Header badge
    ctx.fillStyle = '#0f172a';
    ctx.fillRect(14, 14, 484, 38);
    ctx.fillStyle = '#facc15';
    ctx.font = 'bold 22px Consolas, monospace';
    ctx.fillText(`ID: ${boxId}`, 24, 40);
    ctx.fillStyle = '#38bdf8';
    ctx.font = 'bold 18px Consolas, monospace';
    ctx.fillText('QC VERIFIED', 355, 39);

    // 3 Required CIJ Dot-Matrix Lines (MFG / EXP / LOT)
    ctx.fillStyle = '#090d16';
    ctx.font = 'bold 23px Consolas, monospace';
    ctx.fillText(`MFG: ${mfgDate}`, 22, 88);
    ctx.fillText(`EXP: ${expDate}`, 22, 124);
    ctx.fillText(`LOT: ${lotCode}`, 22, 160);

    // 1D Industrial Barcode along bottom left
    const barPattern = [4,2,2,6,2,4,2,2,6,4,2,2,4,6,2,2,4,2,6,2,4,2,2,6,2,4,2,6,2,2,4];
    let bx = 24;
    for (let i = 0; i < barPattern.length; i++) {
      const w = barPattern[i] * 2.5;
      if (i % 2 === 0) {
        ctx.fillStyle = '#000000';
        ctx.fillRect(bx, 178, w, 50);
      }
      bx += w;
    }
    ctx.font = 'bold 14px Consolas, monospace';
    ctx.fillText(`*${boxId}-${lotCode.replace('#', '')}*`, 32, 243);

    // 2D DataMatrix Code on the right side
    const dmX = 365;
    const dmY = 72;
    const cell = 11;
    const cells = 11;
    ctx.fillStyle = '#000000';
    for (let r = 0; r < cells; r++) {
      for (let c = 0; c < cells; c++) {
        // L-finder border + alternating timing border + deterministic interior bits
        const isFinder = (c === 0) || (r === cells - 1);
        const isTiming = (r === 0 && c % 2 === 0) || (c === cells - 1 && r % 2 === 1);
        const isData = ((r * 7 + c * 13 + boxId.charCodeAt(boxId.length - 1)) % 3 === 0);
        if (isFinder || isTiming || (r > 0 && r < cells - 1 && c > 0 && c < cells - 1 && isData)) {
          ctx.fillRect(dmX + c * cell, dmY + r * cell, cell, cell);
        }
      }
    }

    const tex = new THREE.CanvasTexture(canvas);
    tex.anisotropy = 4;
    tex.needsUpdate = true;
    return tex;
  }

  // ===========================================================================
  // GIAI ĐOẠN 3 - PHẦN 2: HỆ THỐNG XE TỰ HÀNH AGV/AMR & PHÂN KHU KHO ẢO
  // (Autonomous Guided Vehicle / Mobile Robot & Virtual Warehouse Rack Zone)
  // ===========================================================================
  buildAGVAndVirtualWarehouse() {
    // -------------------------------------------------------------------------
    // 1. Vật liệu chuyên dụng cho AGV, Pallet gỗ và Kệ kho Selective Rack
    // -------------------------------------------------------------------------
    this.matAGVYellow = new THREE.MeshStandardMaterial({
      color: 0xeab308,
      roughness: 0.32,
      metalness: 0.35
    });
    this.matAGVBumperBlack = new THREE.MeshStandardMaterial({
      color: 0x111827,
      roughness: 0.75,
      metalness: 0.2
    });
    this.matWoodPallet = new THREE.MeshStandardMaterial({
      color: 0xc29b61,
      roughness: 0.82,
      metalness: 0.05
    });
    this.matRackBlue = new THREE.MeshStandardMaterial({
      color: 0x1d4ed8,
      roughness: 0.42,
      metalness: 0.55
    });
    this.matRackOrange = new THREE.MeshStandardMaterial({
      color: 0xf97316,
      roughness: 0.38,
      metalness: 0.45
    });
    this.matLidarScan = new THREE.MeshBasicMaterial({
      color: 0x06b6d4,
      transparent: true,
      opacity: 0.32,
      side: THREE.DoubleSide,
      blending: THREE.AdditiveBlending,
      depthWrite: false
    });
    this.matAGVBeacon = new THREE.MeshStandardMaterial({
      color: 0xf59e0b,
      emissive: 0xf59e0b,
      emissiveIntensity: 1.5,
      roughness: 0.2
    });

    // -------------------------------------------------------------------------
    // 2. Dải vạch kẻ đường dẫn hướng từ tính (Magnetic Guide Line / Path Marker)
    // Nối từ đuôi Băng tải 3 (X = -4.58, Z = -1.45) đến Cổng Kho Ảo (X = -9.35, Z = +0.15)
    // -------------------------------------------------------------------------
    this.agvGuidePathGroup = new THREE.Group();
    this.agvGuidePathGroup.name = "AGV_MagneticGuidePath";

    const pathSegments = [
      { p1: new THREE.Vector3(-3.90, 0.004, -1.45), p2: new THREE.Vector3(-6.10, 0.004, -1.45) },
      { p1: new THREE.Vector3(-6.10, 0.004, -1.45), p2: new THREE.Vector3(-7.40, 0.004, -1.05) },
      { p1: new THREE.Vector3(-7.40, 0.004, -1.05), p2: new THREE.Vector3(-8.45, 0.004, -0.25) },
      { p1: new THREE.Vector3(-8.45, 0.004, -0.25), p2: new THREE.Vector3(-9.75, 0.004,  0.15) }
    ];

    pathSegments.forEach(seg => {
      const dir = new THREE.Vector3().subVectors(seg.p2, seg.p1);
      const len = dir.length();
      const mid = new THREE.Vector3().addVectors(seg.p1, seg.p2).multiplyScalar(0.5);
      const angleY = Math.atan2(dir.x, dir.z);

      // Dải băng từ màu đen ở tâm (rộng 90mm) kèm 2 viền vàng phản quang hai bên (rộng 180mm)
      const borderStrip = new THREE.Mesh(
        new THREE.PlaneGeometry(0.18, len + 0.06),
        new THREE.MeshBasicMaterial({ color: 0xfacc15 })
      );
      borderStrip.rotateX(-Math.PI / 2);
      borderStrip.rotation.z = angleY;
      borderStrip.position.copy(mid);
      this.agvGuidePathGroup.add(borderStrip);

      const coreStrip = new THREE.Mesh(
        new THREE.PlaneGeometry(0.09, len + 0.04),
        new THREE.MeshBasicMaterial({ color: 0x0f172a })
      );
      coreStrip.rotateX(-Math.PI / 2);
      coreStrip.rotation.z = angleY;
      coreStrip.position.set(mid.x, 0.005, mid.z);
      this.agvGuidePathGroup.add(coreStrip);
    });

    // Ô kẻ vạch vị trí đỗ Docking Station tại đuôi Băng tải 3 (X = -4.58m, Z = -1.45m)
    const dockPad = new THREE.Mesh(
      new THREE.PlaneGeometry(1.65, 1.22),
      this.matHazardStripe
    );
    dockPad.rotateX(-Math.PI / 2);
    dockPad.position.set(-4.58, 0.003, -1.45);
    this.agvGuidePathGroup.add(dockPad);

    const dockInner = new THREE.Mesh(
      new THREE.PlaneGeometry(1.48, 1.05),
      new THREE.MeshBasicMaterial({ color: 0x0f172a })
    );
    dockInner.rotateX(-Math.PI / 2);
    dockInner.position.set(-4.58, 0.0045, -1.45);
    this.agvGuidePathGroup.add(dockInner);

    // Cảm biến quang cuối chuyền kích hoạt chuyển thùng sang AGV (End-of-Line Transfer Sensor tại X = -3.86m)
    this.agvTransferSensorGroup = new THREE.Group();
    this.agvTransferSensorGroup.position.set(-3.86, 0.75, -1.16);
    const tsBody = new THREE.Mesh(new THREE.BoxGeometry(0.03, 0.04, 0.03), this.matSafetyBlack);
    this.agvTransferSensorLed = new THREE.Mesh(
      new THREE.SphereGeometry(0.007, 10, 10),
      new THREE.MeshBasicMaterial({ color: 0x22c55e })
    );
    this.agvTransferSensorLed.position.set(0, 0.022, 0);
    this.agvTransferSensorGroup.add(tsBody, this.agvTransferSensorLed);
    this.agvGuidePathGroup.add(this.agvTransferSensorGroup);

    this.root.add(this.agvGuidePathGroup);

    // -------------------------------------------------------------------------
    // 3. Dựng Mô hình 3D Xe Tự Hành AGV/AMR Công Nghiệp (~1.4m x 1.0m x 0.45m)
    // -------------------------------------------------------------------------
    this.agvGroup = new THREE.Group();
    this.agvGroup.name = "Industrial_AGV_AMR";
    this.agvGroup.position.set(-4.58, 0, -1.45);

    const chassisL = 1.40; // Dọc theo X
    const chassisW = 1.00; // Ngang theo Z
    const chassisH = 0.34;

    // A. Dải ốp viền đen chống va đập (Safety Bumper Skirt) sát sàn
    const bumperMesh = new THREE.Mesh(
      new THREE.BoxGeometry(chassisL + 0.06, 0.10, chassisW + 0.06),
      this.matAGVBumperBlack
    );
    bumperMesh.position.set(0, 0.09, 0);
    bumperMesh.castShadow = true;
    this.agvGroup.add(bumperMesh);

    // Dải sọc phản quang vàng/đen quanh hông xe
    const sideStripeGeo = new THREE.BoxGeometry(chassisL - 0.10, 0.035, chassisW + 0.065);
    const sideStripe = new THREE.Mesh(sideStripeGeo, this.matHazardStripe);
    sideStripe.position.set(0, 0.11, 0);
    this.agvGroup.add(sideStripe);

    // B. Thân xe chính (Low-profile Yellow Industrial Chassis)
    const bodyMesh = new THREE.Mesh(
      new THREE.BoxGeometry(chassisL, chassisH, chassisW),
      this.matAGVYellow
    );
    bodyMesh.position.set(0, 0.25, 0);
    bodyMesh.castShadow = true;
    bodyMesh.receiveShadow = true;
    this.agvGroup.add(bodyMesh);

    // Nắp capo kỹ thuật màu xám đen phía trên thân xe + màn hình LED trạng thái pin
    const topDeck = new THREE.Mesh(
      new THREE.BoxGeometry(chassisL - 0.12, 0.03, chassisW - 0.12),
      this.matWallFrame
    );
    topDeck.position.set(0, 0.43, 0);
    this.agvGroup.add(topDeck);

    // Dải đèn LED viền trạng thái xanh cyan hiện đại hai bên thân AMR
    const ledStripMat = new THREE.MeshBasicMaterial({ color: 0x06b6d4 });
    for (const sz of [-chassisW / 2 - 0.004, chassisW / 2 + 0.004]) {
      const ledStrip = new THREE.Mesh(new THREE.BoxGeometry(chassisL - 0.24, 0.014, 0.006), ledStripMat);
      ledStrip.position.set(0, 0.34, sz);
      this.agvGroup.add(ledStrip);
    }

    // C. 4 Bánh xe công nghiệp chịu tải nặng (Heavy-duty Polyurethane Drive Wheels)
    this.agvWheels = [];
    const wheelGeo = new THREE.CylinderGeometry(0.085, 0.085, 0.06, 18);
    wheelGeo.rotateX(Math.PI / 2);
    const wheelHubGeo = new THREE.CylinderGeometry(0.04, 0.04, 0.065, 12);
    wheelHubGeo.rotateX(Math.PI / 2);

    const wheelCoords = [
      [-0.46,  0.46],
      [ 0.46,  0.46],
      [-0.46, -0.46],
      [ 0.46, -0.46]
    ];
    wheelCoords.forEach(([wx, wz]) => {
      const wGrp = new THREE.Group();
      wGrp.position.set(wx, 0.085, wz);
      const tire = new THREE.Mesh(wheelGeo, this.matAGVBumperBlack);
      const hub = new THREE.Mesh(wheelHubGeo, this.matAGVYellow);
      wGrp.add(tire, hub);
      this.agvGroup.add(wGrp);
      this.agvWheels.push(wGrp);
    });

    // D. 2 Cụm Cảm biến LiDAR / Laser Scanner ảo ở hai góc đối xứng của thân xe
    const lidarCorners = [
      { x: -chassisL / 2 + 0.04, z:  chassisW / 2 - 0.04, rotY:  Math.PI * 0.75 },
      { x:  chassisL / 2 - 0.04, z: -chassisW / 2 + 0.04, rotY: -Math.PI * 0.25 }
    ];
    this.agvLidarFans = [];

    lidarCorners.forEach(c => {
      const lidarGrp = new THREE.Group();
      lidarGrp.position.set(c.x, 0.14, c.z);

      // Thân trụ LiDAR SICK màu vàng/đen
      const lBase = new THREE.Mesh(new THREE.CylinderGeometry(0.038, 0.042, 0.06, 14), this.matAGVYellow);
      const lWindow = new THREE.Mesh(new THREE.CylinderGeometry(0.035, 0.035, 0.035, 14), this.matSafetyBlack);
      lWindow.position.y = 0.035;
      lidarGrp.add(lBase, lWindow);

      // Vệt sáng quạt Laser 270° quét mặt sàn để dò đường & chống va chạm
      const fanGeo = new THREE.CircleGeometry(0.75, 24, -Math.PI * 0.65, Math.PI * 1.3);
      fanGeo.rotateX(-Math.PI / 2);
      const fanMesh = new THREE.Mesh(fanGeo, this.matLidarScan);
      fanMesh.position.y = -0.11;
      fanMesh.rotation.y = c.rotY;
      lidarGrp.add(fanMesh);
      this.agvLidarFans.push(fanMesh);

      this.agvGroup.add(lidarGrp);
    });

    // E. Cột đèn tháp cảnh báo nhỏ & Đèn LED chớp nháy màu cam/xanh trên đuôi xe
    const beaconPole = new THREE.Mesh(new THREE.CylinderGeometry(0.012, 0.012, 0.24, 10), this.matInox);
    beaconPole.position.set(chassisL / 2 - 0.08, 0.54, chassisW / 2 - 0.08);
    this.agvBeaconDome = new THREE.Mesh(new THREE.CylinderGeometry(0.026, 0.028, 0.055, 14), this.matAGVBeacon);
    this.agvBeaconDome.position.set(chassisL / 2 - 0.08, 0.68, chassisW / 2 - 0.08);
    this.agvGroup.add(beaconPole, this.agvBeaconDome);

    // F. Mặt sàn chở hàng: Tấm Pallet gỗ công nghiệp chuẩn Euro Pallet (1.2m x 0.8m) trên lưng xe
    this.agvPalletGroup = new THREE.Group();
    this.agvPalletGroup.name = "AGV_EuroPalletBase";
    this.agvPalletGroup.position.set(0, 0.445, 0);

    const palL = 1.20;
    const palW = 0.80;

    // 3 thanh đố dọc chịu lực dưới đáy pallet
    for (const pz of [-palW / 2 + 0.06, 0, palW / 2 - 0.06]) {
      const stringer = new THREE.Mesh(new THREE.BoxGeometry(palL, 0.028, 0.09), this.matWoodPallet);
      stringer.position.set(0, 0.014, pz);
      stringer.castShadow = true;
      this.agvPalletGroup.add(stringer);
    }

    // 7 thanh ván mặt trên của Euro Pallet
    for (let i = 0; i < 7; i++) {
      const px = -palL / 2 + 0.06 + i * ((palL - 0.12) / 6);
      const slat = new THREE.Mesh(new THREE.BoxGeometry(0.11, 0.018, palW), this.matWoodPallet);
      slat.position.set(px, 0.036, 0);
      slat.castShadow = true;
      slat.receiveShadow = true;
      this.agvPalletGroup.add(slat);
    }

    // 4 vấu kim loại chặn góc giữ pallet cố định trên lưng AGV
    for (const cx of [-palL / 2 - 0.015, palL / 2 + 0.015]) {
      for (const cz of [-palW / 2 - 0.015, palW / 2 + 0.015]) {
        const stopClip = new THREE.Mesh(new THREE.BoxGeometry(0.03, 0.055, 0.03), this.matSafetyBlack);
        stopClip.position.set(cx, 0.025, cz);
        this.agvPalletGroup.add(stopClip);
      }
    }

    this.agvGroup.add(this.agvPalletGroup);

    // G. Group con chứa toàn bộ các thùng carton đã xếp lên mặt Pallet (Child của AGV)
    this.agvPayloadGroup = new THREE.Group();
    this.agvPayloadGroup.name = "AGV_StackedCartonPayload";
    this.agvGroup.add(this.agvPayloadGroup);

    this.root.add(this.agvGroup);

    // -------------------------------------------------------------------------
    // 4. Dựng Phân Khu Kho Lưu Trữ Ảo (Virtual Warehouse Selective Pallet Rack Zone)
    // Đặt tại X = -9.35m, Z = -1.65m (cách đuôi Băng tải 3 khoảng ~6m)
    // -------------------------------------------------------------------------
    this.warehouseGroup = new THREE.Group();
    this.warehouseGroup.name = "VirtualWarehouse_SelectiveRackZone";
    this.warehouseGroup.position.set(-9.35, 0, -1.45);

    // Nền khu vực kho ảo có viền vàng/đen cảnh báo
    const whBorder = new THREE.Mesh(new THREE.PlaneGeometry(5.2, 3.6), this.matHazardStripe);
    whBorder.rotateX(-Math.PI / 2);
    whBorder.position.set(0, 0.002, 0.25);
    const whFloor = new THREE.Mesh(
      new THREE.PlaneGeometry(4.92, 3.32),
      new THREE.MeshStandardMaterial({ color: 0x0f172a, roughness: 0.45, metalness: 0.2 })
    );
    whFloor.rotateX(-Math.PI / 2);
    whFloor.position.set(0, 0.0035, 0.25);
    this.warehouseGroup.add(whBorder, whFloor);

    // Xây dựng 2 Khoang Kệ Chứa Hàng Công Nghiệp 3 Tầng (2-Bay x 3-Level Selective Pallet Racks)
    const rackTotalW = 4.40; // 2 khoang, mỗi khoang rộng 2.15m
    const rackDepthZ = 1.05;
    const rackHeightY = 3.20;
    const bayXPositions = [-2.20, 0.0, 2.20];
    const levelHeights = [0.16, 1.25, 2.30];

    // Cột đứng khung thép màu xanh công nghiệp (Blue Upright Frames)
    const uprightGeo = new THREE.BoxGeometry(0.08, rackHeightY, 0.08);
    bayXPositions.forEach(ux => {
      for (const uz of [-rackDepthZ / 2, rackDepthZ / 2]) {
        const col = new THREE.Mesh(uprightGeo, this.matRackBlue);
        col.position.set(ux, rackHeightY / 2, uz);
        col.castShadow = true;
        this.warehouseGroup.add(col);
      }

      // Thanh giằng chéo chữ Z bên hông cột kệ
      for (let lv = 0; lv < 3; lv++) {
        const brace = new THREE.Mesh(new THREE.BoxGeometry(0.04, 0.04, rackDepthZ - 0.08), this.matRackBlue);
        brace.position.set(ux, 0.45 + lv * 0.95, 0);
        this.warehouseGroup.add(brace);
      }
    });

    // Thanh dầm ngang chịu lực màu cam (Orange Load Beams) cho 3 tầng
    levelHeights.forEach(ly => {
      for (const bz of [-rackDepthZ / 2 + 0.04, rackDepthZ / 2 - 0.04]) {
        const beam = new THREE.Mesh(new THREE.BoxGeometry(rackTotalW + 0.08, 0.09, 0.05), this.matRackOrange);
        beam.position.set(0, ly, bz);
        beam.castShadow = true;
        this.warehouseGroup.add(beam);
      }
    });

    // Biển hiệu trên đỉnh kệ kho: "SMART VIRTUAL WAREHOUSE // AGV DROP-OFF ZONE"
    const headerSign = new THREE.Mesh(new THREE.BoxGeometry(3.6, 0.28, 0.04), this.matSafetyBlack);
    headerSign.position.set(0, rackHeightY + 0.16, rackDepthZ / 2);
    const headerBorder = new THREE.Mesh(new THREE.BoxGeometry(3.66, 0.32, 0.02), this.matRackOrange);
    headerBorder.position.set(0, rackHeightY + 0.16, rackDepthZ / 2 - 0.015);
    this.warehouseGroup.add(headerSign, headerBorder);

    // Đặt sẵn một số kiện Pallet thùng hàng mẫu trên Tầng 2, Tầng 3 và Khoang phụ để tạo bối cảnh kho thực tế
    // Chừa trống hoàn toàn Khoang Trung Tâm Tầng 1 (Bay 1: X = -1.10m và X = +1.10m) làm Điểm đỗ dỡ tải (Drop-off Bay)!
    const sampleLocations = [
      { x: -1.10, y: 1.25 },
      { x:  1.10, y: 1.25 },
      { x: -1.10, y: 2.30 },
      { x:  1.10, y: 2.30 },
      { x: -1.35, y: 0.16 }
    ];
    sampleLocations.forEach(loc => {
      const samplePallet = this.createSampleWarehousePalletUnit(6);
      samplePallet.position.set(loc.x, loc.y + 0.05, 0);
      this.warehouseGroup.add(samplePallet);
    });

    // Khung đánh dấu vị trí Ô Tiếp Nhận Hàng ở Tầng 1 (Drop-off Bay tại X = +0.85m, Y = 0.16m)
    this.warehouseDropBayGroup = new THREE.Group();
    this.warehouseDropBayGroup.position.set(0.85, 0.21, 0);
    this.warehouseGroup.add(this.warehouseDropBayGroup);

    this.root.add(this.warehouseGroup);
  }

  /**
   * Tạo một cụm Pallet + Thùng carton hoàn chỉnh để trưng bày trên kệ kho hoặc khi AGV vừa dỡ hàng xong
   */
  createSampleWarehousePalletUnit(boxCount = 12) {
    const unit = new THREE.Group();

    // Đế gỗ pallet
    const palBase = new THREE.Mesh(new THREE.BoxGeometry(1.15, 0.06, 0.78), this.matWoodPallet);
    palBase.position.y = 0.03;
    palBase.castShadow = true;
    unit.add(palBase);

    const cols = 2;
    const rows = 2;
    const perLayer = cols * rows;
    const actualCount = Math.min(boxCount, 4);
    const boxGeo = new THREE.BoxGeometry(0.32, 0.16, 0.28);
    for (let i = 0; i < actualCount; i++) {
      const layer = Math.floor(i / perLayer);
      const idx = i % perLayer;
      const c = idx % cols;
      const r = Math.floor(idx / cols);

      const bx = (c - 0.5) * 0.42;
      const bz = (r - 0.5) * 0.30;
      const by = 0.06 + 0.08 + layer * 0.165;

      const box = new THREE.Mesh(boxGeo, this.matCardboard);
      box.position.set(bx, by, bz);

      unit.add(box);
    }
    return unit;
  }

  /**
   * Tạo một thùng carton đã dán kín nắp kèm tem Barcode (dùng khi xếp trực tiếp lên AGV)
   */
  createSealedBoxMeshForAGV() {
    const boxGroup = new THREE.Group();
    boxGroup.name = "SealedCartonBox_AGV";

    const wX = 0.28;
    const hY = 0.16;
    const dZ = 0.24;

    const body = new THREE.Mesh(new THREE.BoxGeometry(wX, hY, dZ), this.matCardboard);
    body.position.y = hY / 2;
    body.castShadow = true;
    body.receiveShadow = true;
    boxGroup.add(body);

    const tape = new THREE.Mesh(new THREE.BoxGeometry(wX + 0.02, 0.004, 0.048), this.matTapeAmber);
    tape.position.set(0, hY + 0.002, 0);
    boxGroup.add(tape);

    const yyyy = new Date().getFullYear();
    const boxId = `BOX-${yyyy}-${String(this.nextBoxSerial++).padStart(4, '0')}`;
    const tex = this.createBoxPrintTexture(boxId, this.currentMfgDate, this.currentExpDate, this.currentLotCode);
    const labelMesh = new THREE.Mesh(
      new THREE.PlaneGeometry(0.18, 0.095),
      new THREE.MeshBasicMaterial({ map: tex, side: THREE.DoubleSide })
    );
    labelMesh.position.set(0, hY * 0.52, dZ / 2 + 0.002);
    boxGroup.add(labelMesh);

    return boxGroup;
  }

  /**
   * Đặt kiện hàng AGV vừa giao vào ô trống Tầng 1 của Kệ Kho Ảo (Drop-off Bay)
   */
  placeDeliveredBatchInWarehouseRack(tripIndex, boxCount = 12) {
    if (!this.warehouseDropBayGroup) return;
    // Xóa kiện cũ ở Drop-off Bay nếu đầy quá 2 vị trí để đặt kiện mới vừa giao
    while (this.warehouseDropBayGroup.children.length >= 2) {
      this.warehouseDropBayGroup.remove(this.warehouseDropBayGroup.children[0]);
    }
    const palletUnit = this.createSampleWarehousePalletUnit(Math.max(4, boxCount));
    const offsetSlot = ((tripIndex - 1) % 2) * 1.15;
    palletUnit.position.set(-offsetSlot, 0, 0);
    this.warehouseDropBayGroup.add(palletUnit);
  }

  /**
   * Cập nhật hiệu ứng quay bánh xe, quét tia LiDAR và đèn chớp cảnh báo của AGV
   */
  updateAGVVisualEffects(delta, mode = 'LOADING') {
    const isMoving = (mode === 'TRANSITING' || mode === 'RETURNING');
    const dirSign = (mode === 'RETURNING') ? 1 : -1;

    // 1. Quay 4 bánh xe khi AGV di chuyển
    if (isMoving && this.agvWheels) {
      for (const w of this.agvWheels) {
        w.rotation.z += dirSign * delta * 8.5;
      }
    }

    // 2. Nhấp nháy đèn tháp cảnh báo trên đuôi xe & quét quạt LiDAR
    if (this.matAGVBeacon) {
      if (isMoving || mode === 'UNLOADING') {
        const blink = Math.sin(performance.now() * 0.015) > 0;
        this.matAGVBeacon.color.setHex(mode === 'UNLOADING' ? 0x38bdf8 : 0xf59e0b);
        this.matAGVBeacon.emissive.setHex(mode === 'UNLOADING' ? 0x38bdf8 : 0xf59e0b);
        this.matAGVBeacon.emissiveIntensity = blink ? 2.6 : 0.25;
      } else {
        // Đỗ chờ nhận hàng: Sáng xanh lá êm dịu
        this.matAGVBeacon.color.setHex(0x22c55e);
        this.matAGVBeacon.emissive.setHex(0x22c55e);
        this.matAGVBeacon.emissiveIntensity = 1.1;
      }
    }

    if (this.matLidarScan) {
      this.matLidarScan.opacity = isMoving
        ? (0.28 + 0.18 * Math.sin(performance.now() * 0.012))
        : 0.22;
    }
  }

  /**
   * Emits a subtle industrial barcode scanner confirmation beep via Web Audio API
   */
  playScannerBeep(isSuccess = true) {
    try {
      const AudioCtx = window.AudioContext || window.webkitAudioContext;
      if (!AudioCtx) return;
      if (!this._audioCtx) {
        this._audioCtx = new AudioCtx();
      }
      if (this._audioCtx.state === 'suspended') {
        this._audioCtx.resume().catch(() => {});
      }
      const osc = this._audioCtx.createOscillator();
      const gain = this._audioCtx.createGain();
      osc.type = isSuccess ? 'sine' : 'sawtooth';
      osc.frequency.setValueAtTime(isSuccess ? 2650 : 420, this._audioCtx.currentTime);
      gain.gain.setValueAtTime(0.035, this._audioCtx.currentTime);
      gain.gain.exponentialRampToValueAtTime(0.001, this._audioCtx.currentTime + (isSuccess ? 0.085 : 0.22));
      osc.connect(gain);
      gain.connect(this._audioCtx.destination);
      osc.start();
      osc.stop(this._audioCtx.currentTime + (isSuccess ? 0.09 : 0.23));
    } catch (_) {
      // Ignore if browser audio autoplay policy blocks sound before user interaction
    }
  }

  // ===========================================================================
  // GIAI ĐOẠN 4 - PHẦN 2: MAIN INDUSTRIAL ELECTRICAL CABINET & CABLE ROUTING
  // (YRC1000micro, MP3300 + 218IFD, J1000 AC Drive, Servo Drives A/B/C,
  //  Floor Cable Trays, Delta Energy Drag Chain & Flexible Corrugated Conduits)
  // ===========================================================================

  buildIndustrialControlCabinetAndCableInfrastructure() {
    this.controlCabinetGroup = new THREE.Group();
    this.controlCabinetGroup.name = "MainIndustrialElectricalCabinet";
    // Placed right behind the safety enclosure at X = +0.55m, Z = -2.28m, facing +Z toward the cell & front camera
    const cabX = 0.55;
    const cabZ = -2.28;
    this.controlCabinetGroup.position.set(cabX, 0, cabZ);

    // 1. Cabinet Materials (RAL 7035 Light Grey Powder-Coated Sheet Steel + Cutaway Support)
    this.matCabinetRAL7035 = new THREE.MeshStandardMaterial({
      color: 0xd1d5db, // RAL 7035 Industrial Light Grey
      roughness: 0.36,
      metalness: 0.42,
      transparent: false,
      opacity: 1.0
    });

    this.matCabinetDoorShell = new THREE.MeshStandardMaterial({
      color: 0xcbd5e1, // Slightly contrasted door panel RAL 7035
      roughness: 0.34,
      metalness: 0.45,
      transparent: false,
      opacity: 1.0
    });

    this.matGalvPlate = new THREE.MeshStandardMaterial({
      color: 0x94a3b8, // Galvanized inner sub-panel & perforated cable tray
      roughness: 0.32,
      metalness: 0.78
    });

    this.matYaskawaBlue = new THREE.MeshStandardMaterial({
      color: 0x1d4ed8, // Yaskawa signature royal blue
      roughness: 0.32,
      metalness: 0.35
    });

    this.matDinTrunkGrey = new THREE.MeshStandardMaterial({
      color: 0x64748b, // Slotted PVC wiring duct
      roughness: 0.55,
      metalness: 0.15
    });

    this.matCableRubberBlack = new THREE.MeshStandardMaterial({
      color: 0x0f172a,
      roughness: 0.65,
      metalness: 0.15
    });

    this.matEthernetGreen = new THREE.MeshStandardMaterial({
      color: 0x16a34a, // Industrial Profinet / Ethernet Cat6 shielded green cable
      roughness: 0.4,
      metalness: 0.15
    });

    this.matMechatrolinkBlue = new THREE.MeshStandardMaterial({
      color: 0x2563eb, // Yaskawa Mechatrolink-III bus cable
      roughness: 0.38,
      metalness: 0.2
    });

    const cabW = 1.24; // Width along X
    const cabH = 1.92; // Enclosure height above plinth
    const cabD = 0.56; // Depth along Z
    const plinthH = 0.12; // Black base plinth height

    // 2. Heavy-Duty Black Steel Plinth Base fixed to Factory Floor
    const plinthMesh = new THREE.Mesh(
      new THREE.BoxGeometry(cabW + 0.02, plinthH, cabD + 0.02),
      this.matSafetyBlack
    );
    plinthMesh.position.set(0, plinthH / 2, 0);
    plinthMesh.castShadow = true;
    plinthMesh.receiveShadow = true;
    this.controlCabinetGroup.add(plinthMesh);

    // 3. Outer Sheet-Steel Enclosure Shell (Rear, Left, Right, Top, Bottom + Interior Backplate)
    this.cabinetShellMeshes = [];
    const wallT = 0.022;

    const rearWall = new THREE.Mesh(new THREE.BoxGeometry(cabW, cabH, wallT), this.matCabinetRAL7035);
    rearWall.position.set(0, plinthH + cabH / 2, -cabD / 2 + wallT / 2);
    rearWall.castShadow = true;

    const leftWall = new THREE.Mesh(new THREE.BoxGeometry(wallT, cabH, cabD), this.matCabinetRAL7035);
    leftWall.position.set(-cabW / 2 + wallT / 2, plinthH + cabH / 2, 0);
    leftWall.castShadow = true;

    const rightWall = new THREE.Mesh(new THREE.BoxGeometry(wallT, cabH, cabD), this.matCabinetRAL7035);
    rightWall.position.set(cabW / 2 - wallT / 2, plinthH + cabH / 2, 0);
    rightWall.castShadow = true;

    const topRoof = new THREE.Mesh(new THREE.BoxGeometry(cabW + 0.02, 0.03, cabD + 0.03), this.matCabinetRAL7035);
    topRoof.position.set(0, plinthH + cabH + 0.015, 0);
    topRoof.castShadow = true;

    const btmFloor = new THREE.Mesh(new THREE.BoxGeometry(cabW - 0.02, wallT, cabD - 0.02), this.matCabinetRAL7035);
    btmFloor.position.set(0, plinthH + wallT / 2, 0);

    // Interior dark border gasket rim
    const gasketTop = new THREE.Mesh(new THREE.BoxGeometry(cabW - 0.02, 0.025, 0.02), this.matSafetyBlack);
    gasketTop.position.set(0, plinthH + cabH - 0.012, cabD / 2 - 0.015);
    const centerPillar = new THREE.Mesh(new THREE.BoxGeometry(0.03, cabH - 0.04, 0.02), this.matCabinetRAL7035);
    centerPillar.position.set(0, plinthH + cabH / 2, cabD / 2 - 0.02);

    this.controlCabinetGroup.add(rearWall, leftWall, rightWall, topRoof, btmFloor, gasketTop, centerPillar);
    this.cabinetShellMeshes.push(leftWall, rightWall, topRoof, centerPillar);

    // Interior LED Inspection Strip Light at top of cabinet
    this.cabinetInteriorLight = new THREE.PointLight(0xe0f2fe, 0.0, 2.2);
    this.cabinetInteriorLight.position.set(0, plinthH + cabH - 0.12, 0.08);
    this.controlCabinetGroup.add(this.cabinetInteriorLight);

    // Galvanized Steel Mounting Sub-Plate inside Cabinet
    const backPlate = new THREE.Mesh(new THREE.BoxGeometry(cabW - 0.10, cabH - 0.12, 0.012), this.matGalvPlate);
    backPlate.position.set(0, plinthH + cabH / 2, -cabD / 2 + 0.045);
    this.controlCabinetGroup.add(backPlate);

    // =========================================================================
    // 4. INTERIOR YASKAWA CONTROL HARDWARE (Based on Yaskawa Lab Kit)
    // =========================================================================
    this.cabinetInteriorGroup = new THREE.Group();
    this.cabinetInteriorGroup.name = "YaskawaControlHardwareInterior";
    this.cabinetInteriorGroup.position.set(0, plinthH, -cabD / 2 + 0.055);
    // On-Demand 3D Culling: Mặc định ẩn toàn bộ ruột tủ điện (MP3300, J1000, 3x Sigma-7, YRC1000micro, 18 đèn LED)
    // khi cánh tủ đang đóng kín để tiết kiệm hơn 45 draw calls mỗi frame.
    this.cabinetInteriorGroup.visible = false;
    this.controlCabinetGroup.add(this.cabinetInteriorGroup);

    this.cabinetStatusLeds = []; // Stores { mesh, type: 'PWR'|'RUN'|'ALARM'|'LINK' }

    const makeStatusLed = (x, y, z, type = 'PWR', labelColor = 0x1e293b) => {
      const ledMat = new THREE.MeshBasicMaterial({ color: 0x1e293b });
      const ledMesh = new THREE.Mesh(new THREE.SphereGeometry(0.0065, 10, 10), ledMat);
      ledMesh.position.set(x, y, z);
      this.cabinetStatusLeds.push({ mesh: ledMesh, mat: ledMat, type, baseColor: labelColor });
      return ledMesh;
    };

    // Slotted Grey PVC Wiring Trunking Channels (Horizontal & Vertical Cable Ducts)
    const hDuctPositions = [1.72, 1.30, 0.86, 0.36];
    hDuctPositions.forEach(dy => {
      const duct = new THREE.Mesh(new THREE.BoxGeometry(cabW - 0.16, 0.036, 0.045), this.matDinTrunkGrey);
      duct.position.set(0, dy, 0.022);
      this.cabinetInteriorGroup.add(duct);
    });
    [-0.50, 0.02, 0.50].forEach(dx => {
      const vDuct = new THREE.Mesh(new THREE.BoxGeometry(0.036, cabH - 0.24, 0.045), this.matDinTrunkGrey);
      vDuct.position.set(dx, cabH / 2, 0.022);
      this.cabinetInteriorGroup.add(vDuct);
    });

    // Chrome DIN Rails across the mounting plate
    [1.52, 1.08, 0.62].forEach(ry => {
      const dinRail = new THREE.Mesh(new THREE.BoxGeometry(cabW - 0.20, 0.022, 0.008), this.matInox);
      dinRail.position.set(0, ry, 0.006);
      this.cabinetInteriorGroup.add(dinRail);
    });

    // -------------------------------------------------------------------------
    // A. YASKAWA MP3300 MACHINE CONTROLLER + 218IFD ETHERNET + MECHATROLINK-III
    //    Mounted on Top-Left DIN Rail (X = -0.26m, Y = 1.52m)
    // -------------------------------------------------------------------------
    const mp3300Group = new THREE.Group();
    mp3300Group.name = "Yaskawa_MP3300_Controller";
    mp3300Group.position.set(-0.26, 1.52, 0.01);

    // Power Supply Module (MBU-01) + CPU-301 + 218IFD Ethernet + SVC-01 Mechatrolink
    const mpChassis = new THREE.Mesh(new THREE.BoxGeometry(0.34, 0.22, 0.13), this.matPneumaticBody);
    mpChassis.position.set(0, 0, 0.065);
    mp3300Group.add(mpChassis);

    // Yaskawa Blue Header Accent Strip
    const mpHeader = new THREE.Mesh(new THREE.BoxGeometry(0.338, 0.032, 0.006), this.matYaskawaBlue);
    mpHeader.position.set(0, 0.088, 0.132);
    mp3300Group.add(mpHeader);

    // Module Slot Dividers & Faceplate Texture
    const mpFaceTex = this.createDeviceLabelTexture(
      'YASKAWA MP3300',
      'CPU-301 | 218IFD | ML-III',
      '#1e293b',
      '#38bdf8'
    );
    const mpFace = new THREE.Mesh(
      new THREE.PlaneGeometry(0.32, 0.065),
      new THREE.MeshBasicMaterial({ map: mpFaceTex })
    );
    mpFace.position.set(0, 0.035, 0.133);
    mp3300Group.add(mpFace);

    // 218IFD Ethernet RJ45 Port (with Green Cat6 Ethernet Cable connected to Industrial Switch)
    const rj45Housing = new THREE.Mesh(new THREE.BoxGeometry(0.032, 0.042, 0.025), this.matSafetyBlack);
    rj45Housing.position.set(-0.04, -0.04, 0.135);
    const rj45Boot = new THREE.Mesh(new THREE.CylinderGeometry(0.007, 0.007, 0.04, 10), this.matEthernetGreen);
    rj45Boot.rotateX(Math.PI / 2);
    rj45Boot.position.set(-0.04, -0.04, 0.155);
    mp3300Group.add(rj45Housing, rj45Boot);

    // 3 Mechatrolink-III Blue Bus Connectors (Linking MP3300 to Servo Drives A, B, C)
    [-0.11, 0.04, 0.09].forEach(mx => {
      const mlPort = new THREE.Mesh(new THREE.BoxGeometry(0.026, 0.048, 0.026), this.matMechatrolinkBlue);
      mlPort.position.set(mx, -0.04, 0.135);
      mp3300Group.add(mlPort);
    });

    // MP3300 Status LEDs: PWR (Green), RUN (Flashing Green), ALM/ERR (Red), TX/RX (Green)
    mp3300Group.add(
      makeStatusLed(-0.13, 0.088, 0.136, 'PWR'),
      makeStatusLed(-0.10, 0.088, 0.136, 'RUN'),
      makeStatusLed(-0.07, 0.088, 0.136, 'ALARM'),
      makeStatusLed(-0.04, -0.01, 0.145, 'LINK')
    );

    this.cabinetInteriorGroup.add(mp3300Group);

    // -------------------------------------------------------------------------
    // B. YASKAWA J1000 AC INVERTER DRIVE (Conveyor Motor Speed Controller)
    //    Mounted Top-Right (X = +0.26m, Y = 1.52m) with Red 7-Segment "50.0 Hz" LED
    // -------------------------------------------------------------------------
    const j1000Group = new THREE.Group();
    j1000Group.name = "Yaskawa_J1000_Inverter";
    j1000Group.position.set(0.26, 1.52, 0.01);

    // Finned Aluminum Heatsink Rear + Dark Industrial Body
    const jHeatsink = new THREE.Mesh(new THREE.BoxGeometry(0.19, 0.23, 0.06), this.matGalvPlate);
    jHeatsink.position.set(0, 0, 0.03);
    const jBody = new THREE.Mesh(new THREE.BoxGeometry(0.18, 0.22, 0.09), this.matSealerAccents);
    jBody.position.set(0, 0, 0.105);
    const jTopBlue = new THREE.Mesh(new THREE.BoxGeometry(0.182, 0.03, 0.092), this.matYaskawaBlue);
    jTopBlue.position.set(0, 0.095, 0.105);
    j1000Group.add(jHeatsink, jBody, jTopBlue);

    // J1000 7-Segment LED Display Canvas ("50.0 Hz" when powered, dark when OFF)
    this.j1000Canvas = document.createElement('canvas');
    this.j1000Canvas.width = 256;
    this.j1000Canvas.height = 128;
    this.j1000Texture = new THREE.CanvasTexture(this.j1000Canvas);
    this.j1000ScreenMat = new THREE.MeshBasicMaterial({ map: this.j1000Texture });
    this.updateJ1000DisplayTexture('OFF');

    const jScreenMesh = new THREE.Mesh(new THREE.PlaneGeometry(0.135, 0.068), this.j1000ScreenMat);
    jScreenMesh.position.set(0, 0.035, 0.152);
    j1000Group.add(jScreenMesh);

    // Digital Keypad Buttons (RUN / STOP / UP / DOWN / LO-RE)
    const btnRun = new THREE.Mesh(new THREE.BoxGeometry(0.035, 0.02, 0.008), new THREE.MeshBasicMaterial({ color: 0x22c55e }));
    btnRun.position.set(-0.035, -0.03, 0.152);
    const btnStop = new THREE.Mesh(new THREE.BoxGeometry(0.035, 0.02, 0.008), new THREE.MeshBasicMaterial({ color: 0xef4444 }));
    btnStop.position.set(0.035, -0.03, 0.152);
    const potKnob = new THREE.Mesh(new THREE.CylinderGeometry(0.012, 0.012, 0.015, 14), this.matSafetyBlack);
    potKnob.rotateX(Math.PI / 2);
    potKnob.position.set(0, -0.03, 0.154);
    j1000Group.add(btnRun, btnStop, potKnob);

    // J1000 Status LEDs
    j1000Group.add(
      makeStatusLed(-0.065, 0.095, 0.153, 'PWR'),
      makeStatusLed(-0.040, 0.095, 0.153, 'RUN'),
      makeStatusLed(-0.015, 0.095, 0.153, 'ALARM')
    );

    this.cabinetInteriorGroup.add(j1000Group);

    // -------------------------------------------------------------------------
    // C. SERVOPACK DRIVES (SIGMA-7 SERVO DRIVE A, B, C)
    //    Mounted on Middle DIN Rail (Y = 1.08m), linked to MP3300 via Mechatrolink
    // -------------------------------------------------------------------------
    const servoConfigs = [
      { name: 'SERVO DRIVE A (SGD7S-X)', x: -0.34 },
      { name: 'SERVO DRIVE B (SGD7S-Y)', x: -0.06 },
      { name: 'SERVO DRIVE C (SGD7S-Z)', x:  0.22 }
    ];

    servoConfigs.forEach((sc, idx) => {
      const sGrp = new THREE.Group();
      sGrp.name = sc.name;
      sGrp.position.set(sc.x, 1.08, 0.01);

      const sBody = new THREE.Mesh(new THREE.BoxGeometry(0.21, 0.24, 0.14), this.matPneumaticBody);
      sBody.position.set(0, 0, 0.07);
      const sFrontStripe = new THREE.Mesh(new THREE.BoxGeometry(0.212, 0.028, 0.006), this.matYaskawaBlue);
      sFrontStripe.position.set(0, 0.098, 0.141);

      const sLabelTex = this.createDeviceLabelTexture(
        `SERVO DRIVE ${String.fromCharCode(65 + idx)}`,
        'YASKAWA SIGMA-7 400W',
        '#0f172a',
        '#facc15'
      );
      const sLabel = new THREE.Mesh(
        new THREE.PlaneGeometry(0.19, 0.055),
        new THREE.MeshBasicMaterial({ map: sLabelTex })
      );
      sLabel.position.set(0, 0.045, 0.142);

      // Mechatrolink-III Top & Bottom Blue Ports + Orange Motor Power Terminal Block
      const mlPortA = new THREE.Mesh(new THREE.BoxGeometry(0.025, 0.042, 0.022), this.matMechatrolinkBlue);
      mlPortA.position.set(-0.05, -0.03, 0.145);
      const mlPortB = new THREE.Mesh(new THREE.BoxGeometry(0.025, 0.042, 0.022), this.matMechatrolinkBlue);
      mlPortB.position.set(-0.01, -0.03, 0.145);
      const pwrTerm = new THREE.Mesh(new THREE.BoxGeometry(0.045, 0.075, 0.02), this.matRollerOrange);
      pwrTerm.position.set(0.055, -0.04, 0.145);

      sGrp.add(
        sBody, sFrontStripe, sLabel, mlPortA, mlPortB, pwrTerm,
        makeStatusLed(-0.075, 0.098, 0.145, 'PWR'),
        makeStatusLed(-0.048, 0.098, 0.145, 'RUN'),
        makeStatusLed(-0.022, 0.098, 0.145, 'ALARM')
      );

      this.cabinetInteriorGroup.add(sGrp);

      // Mechatrolink-III Blue Patch Cable looping between MP3300 and Servo Drives A, B, C
      const mlCurve = new THREE.CatmullRomCurve3([
        new THREE.Vector3(-0.26 + (idx - 1) * 0.06, 1.46, 0.16),
        new THREE.Vector3((sc.x - 0.26) * 0.5, 1.32, 0.22),
        new THREE.Vector3(sc.x - 0.03, 1.05, 0.16)
      ]);
      const mlTube = new THREE.Mesh(new THREE.TubeGeometry(mlCurve, 12, 0.0055, 8, false), this.matMechatrolinkBlue);
      this.cabinetInteriorGroup.add(mlTube);
    });

    // Green Ethernet Cat6 Cable from MP3300 218IFD port to Din-Rail Network Switch
    const swBox = new THREE.Mesh(new THREE.BoxGeometry(0.11, 0.14, 0.10), this.matSafetyBlack);
    swBox.position.set(0.43, 1.08, 0.06);
    this.cabinetInteriorGroup.add(swBox);
    this.cabinetInteriorGroup.add(makeStatusLed(0.41, 1.12, 0.115, 'LINK'));

    const ethCurve = new THREE.CatmullRomCurve3([
      new THREE.Vector3(-0.30, 1.48, 0.165),
      new THREE.Vector3(0.05, 1.34, 0.21),
      new THREE.Vector3(0.43, 1.12, 0.12)
    ]);
    const ethCable = new THREE.Mesh(new THREE.TubeGeometry(ethCurve, 14, 0.006, 8, false), this.matEthernetGreen);
    this.cabinetInteriorGroup.add(ethCable);

    // -------------------------------------------------------------------------
    // D. YASKAWA YRC1000micro ROBOT CONTROLLER RACK-MOUNT CHASSIS
    //    Mounted in Lower Bay (Y = 0.54m), with Ventilation Slots, LAN & Heavy Umbilical Plug
    // -------------------------------------------------------------------------
    const yrcGroup = new THREE.Group();
    yrcGroup.name = "Yaskawa_YRC1000micro_Controller";
    yrcGroup.position.set(-0.04, 0.54, 0.01);

    // Rack-mount metallic blue/grey chassis (0.86m wide x 0.28m tall x 0.22m deep)
    const yrcChassis = new THREE.Mesh(new THREE.BoxGeometry(0.86, 0.28, 0.20), this.matYaskawaBlue);
    yrcChassis.position.set(0, 0, 0.10);
    const yrcFaceBezel = new THREE.Mesh(new THREE.BoxGeometry(0.82, 0.24, 0.015), this.matPneumaticBody);
    yrcFaceBezel.position.set(0, 0, 0.205);
    yrcGroup.add(yrcChassis, yrcFaceBezel);

    // YRC1000micro Nameplate Banner
    const yrcTex = this.createDeviceLabelTexture(
      'YASKAWA YRC1000micro',
      '6-DOF & DELTA HIGH-SPEED ROBOT CONTROLLER // UDP 10040',
      '#1e3a8a',
      '#ffffff'
    );
    const yrcPlate = new THREE.Mesh(
      new THREE.PlaneGeometry(0.48, 0.075),
      new THREE.MeshBasicMaterial({ map: yrcTex })
    );
    yrcPlate.position.set(-0.14, 0.065, 0.214);
    yrcGroup.add(yrcPlate);

    // Cooling Louver Vents on YRC1000micro Faceplate
    for (let v = 0; v < 5; v++) {
      const slot = new THREE.Mesh(new THREE.BoxGeometry(0.22, 0.012, 0.006), this.matSafetyBlack);
      slot.position.set(-0.26, -0.02 - v * 0.02, 0.214);
      yrcGroup.add(slot);
    }

    // Heavy Multi-Pin Metallic Umbilical Robot Connector Hoods (CN1 / CN2 Robot Harness)
    const cn1Hood = new THREE.Mesh(new THREE.BoxGeometry(0.085, 0.11, 0.055), this.matInox);
    cn1Hood.position.set(0.18, -0.02, 0.23);
    const cn2Hood = new THREE.Mesh(new THREE.BoxGeometry(0.085, 0.11, 0.055), this.matInox);
    cn2Hood.position.set(0.30, -0.02, 0.23);
    // LAN RJ45 Port on YRC1000micro
    const yrcLan = new THREE.Mesh(new THREE.BoxGeometry(0.032, 0.038, 0.025), this.matEthernetGreen);
    yrcLan.position.set(0.06, -0.02, 0.22);
    yrcGroup.add(cn1Hood, cn2Hood, yrcLan);

    // Thick Black Shielded Umbilical Cables dropping from CN1/CN2 into floor cable tray
    [0.18, 0.30].forEach(cx => {
      const umbCurve = new THREE.CatmullRomCurve3([
        new THREE.Vector3(cx - 0.04, 0.49, 0.25),
        new THREE.Vector3(cx - 0.04, 0.32, 0.27),
        new THREE.Vector3(cx - 0.04, 0.10, 0.18)
      ]);
      const umbCable = new THREE.Mesh(new THREE.TubeGeometry(umbCurve, 10, 0.016, 10, false), this.matCableRubberBlack);
      this.cabinetInteriorGroup.add(umbCable);
    });

    // YRC1000micro Status LEDs (PWR, RUN, ALARM, LAN)
    yrcGroup.add(
      makeStatusLed(0.18, 0.085, 0.216, 'PWR'),
      makeStatusLed(0.22, 0.085, 0.216, 'RUN'),
      makeStatusLed(0.26, 0.085, 0.216, 'ALARM'),
      makeStatusLed(0.30, 0.085, 0.216, 'LINK')
    );

    this.cabinetInteriorGroup.add(yrcGroup);

    // =========================================================================
    // 5. HINGED DOUBLE DOORS WITH MAIN ISOLATOR SWITCH, 380V SIGN & LOUVER FANS
    // =========================================================================
    const doorW = cabW / 2 - 0.02;
    const doorH = cabH - 0.03;
    const doorZ = cabD / 2;

    // Left Hinged Door (Pivot at X = -cabW/2 + 0.015, Z = +cabD/2)
    this.cabinetLeftDoorPivot = new THREE.Group();
    this.cabinetLeftDoorPivot.name = "CabinetDoor_Left";
    this.cabinetLeftDoorPivot.position.set(-cabW / 2 + 0.015, plinthH + cabH / 2, doorZ);

    const leftDoorPanel = new THREE.Mesh(new THREE.BoxGeometry(doorW, doorH, 0.018), this.matCabinetDoorShell);
    leftDoorPanel.position.set(doorW / 2, 0, 0);
    leftDoorPanel.castShadow = true;
    this.cabinetLeftDoorPivot.add(leftDoorPanel);
    this.cabinetShellMeshes.push(leftDoorPanel);

    // High-Voltage Triangle Warning Sign ("DANGER 380V / 220V") on Left Door
    const warnSignTex = this.createHighVoltageWarningTexture();
    const warnSignMesh = new THREE.Mesh(
      new THREE.PlaneGeometry(0.28, 0.24),
      new THREE.MeshBasicMaterial({ map: warnSignTex, transparent: true })
    );
    warnSignMesh.position.set(doorW / 2, 0.42, 0.011);
    this.cabinetLeftDoorPivot.add(warnSignMesh);
    this.cabinetShellMeshes.push(warnSignMesh);

    // Filter Fan Louver #1 on Left Door
    const louver1 = this.createCabinetFilterLouver();
    louver1.position.set(doorW / 2, -0.42, 0.012);
    this.cabinetLeftDoorPivot.add(louver1);
    this.cabinetShellMeshes.push(louver1);

    // Left Door Swing Handle
    const handleL = new THREE.Mesh(new THREE.BoxGeometry(0.025, 0.16, 0.028), this.matSafetyBlack);
    handleL.position.set(doorW - 0.045, 0.02, 0.018);
    this.cabinetLeftDoorPivot.add(handleL);
    this.cabinetShellMeshes.push(handleL);

    this.controlCabinetGroup.add(this.cabinetLeftDoorPivot);

    // Right Hinged Door (Pivot at X = +cabW/2 - 0.015, Z = +cabD/2)
    // Houses the Interactive Main Rotary Isolator Switch (Yellow Bezel / Red Handle) & Power Pilot Lamps
    this.cabinetRightDoorPivot = new THREE.Group();
    this.cabinetRightDoorPivot.name = "CabinetDoor_Right";
    this.cabinetRightDoorPivot.position.set(cabW / 2 - 0.015, plinthH + cabH / 2, doorZ);

    const rightDoorPanel = new THREE.Mesh(new THREE.BoxGeometry(doorW, doorH, 0.018), this.matCabinetDoorShell);
    rightDoorPanel.position.set(-doorW / 2, 0, 0);
    rightDoorPanel.castShadow = true;
    this.cabinetRightDoorPivot.add(rightDoorPanel);
    this.cabinetShellMeshes.push(rightDoorPanel);

    // Filter Fan Louver #2 on Right Door
    const louver2 = this.createCabinetFilterLouver();
    louver2.position.set(-doorW / 2, -0.42, 0.012);
    this.cabinetRightDoorPivot.add(louver2);
    this.cabinetShellMeshes.push(louver2);

    // Right Door Swing Handle
    const handleR = new THREE.Mesh(new THREE.BoxGeometry(0.025, 0.16, 0.028), this.matSafetyBlack);
    handleR.position.set(-doorW + 0.045, 0.02, 0.018);
    this.cabinetRightDoorPivot.add(handleR);
    this.cabinetShellMeshes.push(handleR);

    // Interactive Main Rotary Isolator Switch (Yellow Square Bezel + Red Rotary Knob)
    // Placed on outer upper-left of Right Door so it remains clickable even in cutaway mode!
    this.mainIsolatorGroup = new THREE.Group();
    this.mainIsolatorGroup.name = "MainPowerIsolatorSwitch";
    this.mainIsolatorGroup.position.set(-doorW / 2, 0.42, 0.015);

    const isoYellowPlate = new THREE.Mesh(
      new THREE.BoxGeometry(0.11, 0.11, 0.012),
      new THREE.MeshStandardMaterial({ color: 0xfacc15, roughness: 0.3, metalness: 0.15 })
    );
    const isoCollar = new THREE.Mesh(
      new THREE.CylinderGeometry(0.032, 0.036, 0.018, 20),
      this.matInterlockRed
    );
    isoCollar.rotateX(Math.PI / 2);
    isoCollar.position.z = 0.012;

    this.isolatorHandlePivot = new THREE.Group();
    this.isolatorHandlePivot.position.z = 0.024;
    const isoRedKnob = new THREE.Mesh(
      new THREE.BoxGeometry(0.022, 0.082, 0.022),
      new THREE.MeshStandardMaterial({ color: 0xdc2626, roughness: 0.25, metalness: 0.2 })
    );
    this.isolatorHandlePivot.add(isoRedKnob);

    // Main Cabinet Door Pilot Lamp ("MAIN POWER 380V ON") above Isolator Switch
    this.cabinetMainPowerLampMat = new THREE.MeshBasicMaterial({ color: 0x334155 });
    const mainLamp = new THREE.Mesh(new THREE.CylinderGeometry(0.018, 0.018, 0.016, 16), this.cabinetMainPowerLampMat);
    mainLamp.rotateX(Math.PI / 2);
    mainLamp.position.set(0, 0.095, 0.008);

    this.mainIsolatorGroup.add(isoYellowPlate, isoCollar, this.isolatorHandlePivot, mainLamp);
    this.cabinetRightDoorPivot.add(this.mainIsolatorGroup);

    this.controlCabinetGroup.add(this.cabinetRightDoorPivot);
    this.root.add(this.controlCabinetGroup);

    // =========================================================================
    // 6. INDUSTRIAL FLOOR CABLE TRAYS, ENERGY DRAG CHAINS & CORRUGATED CONDUITS
    // =========================================================================
    this.buildCableRoutingAndDragChains(cabX, cabZ);
  }

  /**
   * Builds perforated galvanized floor cable trays, Delta robot U-shaped energy drag chain,
   * and flexible black corrugated conduits connecting the Main Control Cabinet to both robots and 3 conveyors.
   */
  buildCableRoutingAndDragChains(cabX = 0.55, cabZ = -2.28) {
    this.cableInfrastructureGroup = new THREE.Group();
    this.cableInfrastructureGroup.name = "IndustrialCableRoutingInfrastructure";

    const trayY = 0.024;
    const trayH = 0.044;
    const trayW = 0.15;

    // Helper: Build a Perforated Galvanized U-Channel Floor Cable Tray Segment with Exposed Bundled Cables & Partial Protective Covers
    const addFloorCableTraySegment = (x1, z1, x2, z2, hasCover = true) => {
      const segGrp = new THREE.Group();
      const cx = (x1 + x2) / 2;
      const cz = (z1 + z2) / 2;
      const len = Math.hypot(x2 - x1, z2 - z1);
      const angleY = Math.atan2(x2 - x1, z2 - z1);

      segGrp.position.set(cx, 0, cz);
      segGrp.rotation.y = angleY;

      // Galvanized U-channel base & side flanges
      const btm = new THREE.Mesh(new THREE.BoxGeometry(trayW, 0.006, len), this.matGalvPlate);
      btm.position.y = 0.005;
      const sideL = new THREE.Mesh(new THREE.BoxGeometry(0.008, trayH, len), this.matGalvPlate);
      sideL.position.set(-trayW / 2 + 0.004, trayY, 0);
      const sideR = new THREE.Mesh(new THREE.BoxGeometry(0.008, trayH, len), this.matGalvPlate);
      sideR.position.set(trayW / 2 - 0.004, trayY, 0);
      segGrp.add(btm, sideL, sideR);

      // Internal bundled cables (Black power umbilical, Blue Mechatrolink, Green Ethernet, Air tube)
      const cPower1 = new THREE.Mesh(new THREE.CylinderGeometry(0.013, 0.013, len - 0.02, 8), this.matCableRubberBlack);
      cPower1.rotateX(Math.PI / 2);
      cPower1.position.set(-0.036, 0.018, 0);

      const cPower2 = new THREE.Mesh(new THREE.CylinderGeometry(0.011, 0.011, len - 0.02, 8), this.matCableRubberBlack);
      cPower2.rotateX(Math.PI / 2);
      cPower2.position.set(-0.008, 0.016, 0);

      const cBus = new THREE.Mesh(new THREE.CylinderGeometry(0.007, 0.007, len - 0.02, 8), this.matMechatrolinkBlue);
      cBus.rotateX(Math.PI / 2);
      cBus.position.set(0.020, 0.015, 0);

      const cEth = new THREE.Mesh(new THREE.CylinderGeometry(0.006, 0.006, len - 0.02, 8), this.matEthernetGreen);
      cEth.rotateX(Math.PI / 2);
      cEth.position.set(0.042, 0.015, 0);

      segGrp.add(cPower1, cPower2, cBus, cEth);

      // Intermittent Dark Slate Protective Top Cover Plates (leaving gaps to expose internal cables)
      if (hasCover && len > 0.5) {
        const coverCount = Math.max(1, Math.floor(len / 0.75));
        const coverLen = (len / coverCount) * 0.56;
        for (let i = 0; i < coverCount; i++) {
          const zPos = -len / 2 + (i + 0.5) * (len / coverCount);
          const cover = new THREE.Mesh(new THREE.BoxGeometry(trayW + 0.006, 0.006, coverLen), this.matSafetyBlack);
          cover.position.set(0, trayH + 0.004, zPos);
          // Yellow/black hazard stripe on cover edge
          const stripe = new THREE.Mesh(new THREE.BoxGeometry(0.024, 0.002, coverLen * 0.85), this.matHazardStripe);
          stripe.position.set(0, trayH + 0.008, zPos);
          segGrp.add(cover, stripe);
        }
      }

      this.cableInfrastructureGroup.add(segGrp);
    };

    // -------------------------------------------------------------------------
    // ROUTE 1: Main Cabinet (X = +0.55, Z = -2.00) -> Rear Trunk (Z = -1.94)
    //          -> 6-DOF Robot Arm Pedestal (X = -1.25, Z = -0.75) & Conveyor 3 Motor (X = -2.15, Z = -1.45)
    // -------------------------------------------------------------------------
    addFloorCableTraySegment(cabX, cabZ + 0.25, cabX, -1.94, false);
    addFloorCableTraySegment(cabX, -1.94, -2.35, -1.94, true);
    // Branch into 6-DOF Robot Arm Base Pedestal (X = -1.25, Z = -1.94 -> Z = -0.82)
    addFloorCableTraySegment(-1.25, -1.94, -1.25, -0.82, true);
    // Branch into Conveyor 3 Drive Motor & Carton Sealer (X = -2.25, Z = -1.94 -> Z = -1.52)
    addFloorCableTraySegment(-2.25, -1.94, -2.25, -1.52, false);

    // -------------------------------------------------------------------------
    // ROUTE 2: Main Cabinet (X = +0.55, Z = -1.94) -> Along Safety Fence Rear (X = +0.55 -> +3.85)
    //          -> Delta Robot Gantry Column (X = +1.20, Z = -0.45) & Conveyor 1 Drive + Vision Gantry (X = +2.80, +3.85)
    // -------------------------------------------------------------------------
    addFloorCableTraySegment(cabX, -1.94, 3.85, -1.94, true);
    // Branch into Delta Robot Gantry Frame Base (X = +1.20, Z = -1.94 -> Z = -0.45)
    addFloorCableTraySegment(1.20, -1.94, 1.20, -0.45, true);
    // Branch into Cognex Vision Gantry & Reject Pusher Manifold (X = +2.80, Z = -1.94 -> Z = -0.55)
    addFloorCableTraySegment(2.80, -1.94, 2.80, -0.55, true);
    // Branch into Conveyor 1 Infeed Drive Motor & Rotary Encoder (X = +3.85, Z = -1.94 -> Z = -0.55)
    addFloorCableTraySegment(3.85, -1.94, 3.85, -0.55, true);

    // -------------------------------------------------------------------------
    // 7. ARTICULATED U-SHAPED ENERGY DRAG CHAIN (Xích dẫn cáp nhựa đen trên Robot Delta & 6-DOF Arm)
    // -------------------------------------------------------------------------
    const addArticulatedDragChain = (points, linkCount = 10, width = 0.065) => {
      const curve = new THREE.CatmullRomCurve3(points);
      const chainGrp = new THREE.Group();
      const count = Math.min(linkCount, 10);
      const linkGeo = new THREE.BoxGeometry(width, 0.022, 0.042);

      for (let i = 0; i < count; i++) {
        const t = i / (count - 1);
        const pt = curve.getPointAt(t);
        const tangent = curve.getTangentAt(t);

        const link = new THREE.Mesh(linkGeo, this.matSafetyBlack);
        link.position.copy(pt);
        link.lookAt(pt.clone().add(tangent));

        chainGrp.add(link);
      }

      // Internal servo power & blue pneumatic air hose running inside the drag chain
      const innerHose = new THREE.Mesh(new THREE.TubeGeometry(curve, 12, 0.009, 6, false), this.matAirTubeBlue);
      chainGrp.add(innerHose);

      this.cableInfrastructureGroup.add(chainGrp);
    };

    // A. Delta Robot Gantry Vertical Column & Overhead U-Loop Energy Drag Chain (X = 1.20m)
    addArticulatedDragChain([
      new THREE.Vector3(1.52, 0.85, -0.48),
      new THREE.Vector3(1.52, 1.35, -0.48),
      new THREE.Vector3(1.52, 1.78, -0.42),
      new THREE.Vector3(1.44, 1.92, -0.25),
      new THREE.Vector3(1.30, 1.82, -0.14),
      new THREE.Vector3(1.22, 1.56, -0.10)
    ], 10, 0.068);

    // B. 6-DOF Robot Arm Base-to-Shoulder Dress Pack Drag Chain (X = -1.25m, Z = -0.75m)
    addArticulatedDragChain([
      new THREE.Vector3(-1.25, 0.06, -0.92),
      new THREE.Vector3(-1.38, 0.28, -0.95),
      new THREE.Vector3(-1.42, 0.56, -0.88),
      new THREE.Vector3(-1.32, 0.72, -0.78)
    ], 8, 0.058);

    // -------------------------------------------------------------------------
    // 8. FLEXIBLE CORRUGATED CONDUIT TUBING (Ống ruột gà nối vào Hộp đấu dây Động cơ & Cảm biến)
    // -------------------------------------------------------------------------
    const addCorrugatedConduit = (pts, radius = 0.014, isBlueAir = false) => {
      const curve = new THREE.CatmullRomCurve3(pts);
      const tube = new THREE.Mesh(
        new THREE.TubeGeometry(curve, 10, radius, 6, false),
        isBlueAir ? this.matAirTubeBlue : this.matCableRubberBlack
      );
      this.cableInfrastructureGroup.add(tube);
    };

    // Conduit 1: Floor Tray (X = +3.85, Z = -0.55) -> Conveyor 1 Drive Motor Terminal Box & Rotary Encoder
    addCorrugatedConduit([
      new THREE.Vector3(3.85, 0.03, -0.55),
      new THREE.Vector3(3.92, 0.35, -0.62),
      new THREE.Vector3(3.88, 0.68, -0.56),
      new THREE.Vector3(3.85, 0.80, -0.52)
    ], 0.014);

    // Conduit 2: Floor Tray (X = +2.80, Z = -0.55) -> Cognex Vision Gantry & Pneumatic Reject Pusher Solenoid
    addCorrugatedConduit([
      new THREE.Vector3(2.80, 0.03, -0.55),
      new THREE.Vector3(2.84, 0.45, -0.60),
      new THREE.Vector3(2.80, 0.88, -0.56),
      new THREE.Vector3(2.80, 1.32, -0.52)
    ], 0.013);

    // Blue Pneumatic Air Line from Floor Tray to Reject Pusher Cylinder (X = +2.45m)
    addCorrugatedConduit([
      new THREE.Vector3(2.65, 0.04, -0.55),
      new THREE.Vector3(2.55, 0.45, -0.25),
      new THREE.Vector3(2.45, 0.86, -0.05)
    ], 0.008, true);

    // Conduit 3: Floor Tray (X = -1.25, Z = -0.82) -> Photoelectric Sensor DI_11 & Conveyor 2 Junction Box
    addCorrugatedConduit([
      new THREE.Vector3(-1.25, 0.03, -0.82),
      new THREE.Vector3(-1.35, 0.36, -0.35),
      new THREE.Vector3(-1.25, 0.72, 0.36),
      new THREE.Vector3(-1.25, 0.86, 0.40)
    ], 0.012);

    // Conduit 4: Floor Tray (X = -2.25, Z = -1.52) -> Conveyor 3 Drive Motor, Stopper Cylinder & Top Carton Sealer
    addCorrugatedConduit([
      new THREE.Vector3(-2.25, 0.03, -1.52),
      new THREE.Vector3(-2.32, 0.38, -1.72),
      new THREE.Vector3(-2.38, 0.72, -1.68)
    ], 0.014);

    this.root.add(this.cableInfrastructureGroup);
  }

  /**
   * Helper: Creates a procedural nameplate texture for Yaskawa PLC / Servo / Robot Controller modules
   */
  createDeviceLabelTexture(title, subtitle, bgHex = '#1e293b', accentHex = '#38bdf8') {
    const canvas = document.createElement('canvas');
    canvas.width = 256;
    canvas.height = 64;
    const ctx = canvas.getContext('2d');

    ctx.fillStyle = bgHex;
    ctx.fillRect(0, 0, 256, 64);
    ctx.strokeStyle = accentHex;
    ctx.lineWidth = 3;
    ctx.strokeRect(2, 2, 252, 60);

    ctx.fillStyle = '#ffffff';
    ctx.font = 'bold 20px Consolas, monospace';
    ctx.fillText(title, 12, 28);

    ctx.fillStyle = accentHex;
    ctx.font = 'bold 13px Consolas, monospace';
    ctx.fillText(subtitle, 12, 50);

    return new THREE.CanvasTexture(canvas);
  }

  /**
   * Helper: Creates the yellow triangle "DANGER 380V / 220V" high-voltage warning decal
   */
  createHighVoltageWarningTexture() {
    const canvas = document.createElement('canvas');
    canvas.width = 256;
    canvas.height = 220;
    const ctx = canvas.getContext('2d');

    // Yellow rounded placard background
    ctx.fillStyle = '#facc15';
    ctx.strokeStyle = '#111827';
    ctx.lineWidth = 8;
    ctx.fillRect(8, 8, 240, 204);
    ctx.strokeRect(8, 8, 240, 204);

    // Black Hazard Triangle
    ctx.beginPath();
    ctx.moveTo(128, 26);
    ctx.lineTo(200, 134);
    ctx.lineTo(56, 134);
    ctx.closePath();
    ctx.fillStyle = '#facc15';
    ctx.fill();
    ctx.lineWidth = 10;
    ctx.strokeStyle = '#111827';
    ctx.stroke();

    // Red/Black Lightning Bolt inside Triangle
    ctx.fillStyle = '#dc2626';
    ctx.beginPath();
    ctx.moveTo(132, 48);
    ctx.lineTo(108, 90);
    ctx.lineTo(128, 90);
    ctx.lineTo(116, 124);
    ctx.lineTo(152, 80);
    ctx.lineTo(130, 80);
    ctx.closePath();
    ctx.fill();

    // Bold Warning Text
    ctx.fillStyle = '#111827';
    ctx.font = '900 22px Arial, sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText('DANGER 380V / 220V', 128, 168);
    ctx.font = 'bold 15px Consolas, monospace';
    ctx.fillText('HIGH VOLTAGE CABINET', 128, 194);

    return new THREE.CanvasTexture(canvas);
  }

  /**
   * Helper: Creates a square ventilation louver grille with dust filter mesh & cooling fan
   */
  createCabinetFilterLouver() {
    const grp = new THREE.Group();
    const frame = new THREE.Mesh(new THREE.BoxGeometry(0.24, 0.24, 0.016), this.matCabinetRAL7035);
    const grille = new THREE.Mesh(new THREE.BoxGeometry(0.20, 0.20, 0.018), this.matSafetyBlack);
    grp.add(frame, grille);

    for (let i = -3; i <= 3; i++) {
      const slat = new THREE.Mesh(new THREE.BoxGeometry(0.19, 0.012, 0.022), this.matCabinetRAL7035);
      slat.position.set(0, i * 0.025, 0.004);
      slat.rotation.x = 0.35;
      grp.add(slat);
    }
    return grp;
  }

  /**
   * Updates the Yaskawa J1000 AC Drive 7-segment LED display canvas ("50.0 Hz", "BOOT", or "OFF")
   */
  updateJ1000DisplayTexture(modeText = 'OFF') {
    if (!this.j1000Canvas || this.lastJ1000FreqText === modeText) return;
    this.lastJ1000FreqText = modeText;

    const ctx = this.j1000Canvas.getContext('2d');
    ctx.fillStyle = '#090d16';
    ctx.fillRect(0, 0, 256, 128);
    ctx.strokeStyle = '#334155';
    ctx.lineWidth = 4;
    ctx.strokeRect(4, 4, 248, 120);

    ctx.fillStyle = '#94a3b8';
    ctx.font = 'bold 16px Consolas, monospace';
    ctx.fillText('YASKAWA J1000 DRIVE', 16, 26);

    if (modeText === 'OFF') {
      ctx.fillStyle = '#1e293b';
      ctx.font = 'bold 48px Consolas, monospace';
      ctx.fillText('88.8 Hz', 34, 88);
    } else if (modeText === 'BOOT') {
      ctx.fillStyle = '#f59e0b';
      ctx.font = 'bold 42px Consolas, monospace';
      ctx.fillText('INIT...', 42, 88);
    } else {
      ctx.fillStyle = '#ef4444';
      ctx.shadowColor = '#ef4444';
      ctx.shadowBlur = 12;
      ctx.font = 'bold 48px Consolas, monospace';
      ctx.fillText(modeText, 34, 88);
      ctx.shadowBlur = 0;

      ctx.fillStyle = '#22c55e';
      ctx.font = 'bold 14px Consolas, monospace';
      ctx.fillText('● DRV RUN   FWD', 16, 114);
    }

    if (this.j1000Texture) {
      this.j1000Texture.needsUpdate = true;
    }
  }

  /**
   * Toggles the Cabinet Shell Cutaway / Open Door Inspection Mode so users can clearly inspect
   * the internal YRC1000micro, MP3300, J1000 and Sigma-7 Servo Drives.
   */
  toggleCabinetEnclosureVisibility(forceCutaway = null) {
    if (forceCutaway !== null) {
      this.cabinetCutawayMode = Boolean(forceCutaway);
    } else {
      this.cabinetCutawayMode = !this.cabinetCutawayMode;
    }

    this.cabinetDoorOpen = this.cabinetCutawayMode;
    this.targetCabinetDoorAngle = this.cabinetDoorOpen ? (Math.PI * 0.62) : 0;
    if (this.cabinetInteriorGroup) {
      this.cabinetInteriorGroup.visible = this.cabinetCutawayMode;
    }

    // Make outer cabinet walls & door panels semi-transparent (cutaway) when Inspect Mode is ON
    if (this.matCabinetDoorShell) {
      this.matCabinetDoorShell.transparent = this.cabinetCutawayMode;
      this.matCabinetDoorShell.opacity = this.cabinetCutawayMode ? 0.18 : 1.0;
      this.matCabinetDoorShell.needsUpdate = true;
    }
    if (this.matCabinetRAL7035) {
      this.matCabinetRAL7035.transparent = this.cabinetCutawayMode;
      this.matCabinetRAL7035.opacity = this.cabinetCutawayMode ? 0.35 : 1.0;
      this.matCabinetRAL7035.needsUpdate = true;
    }

    return this.cabinetCutawayMode;
  }

  /**
   * Sets the Main Industrial Cabinet Power State with realistic 1.5s controller boot sequence
   */
  setCabinetMainPower(powerOn, isBooting = false) {
    this.cabinetPowerOn = Boolean(powerOn);
    this.cabinetBooting = Boolean(isBooting);
    this.targetIsolatorAngle = (this.cabinetPowerOn || this.cabinetBooting) ? (-Math.PI / 2) : 0;
  }

  /**
   * Updates Cabinet Doors, Rotary Isolator Knob, J1000 7-Segment Display & Module Status LEDs
   */
  updateControlCabinet(delta, hasSystemAlarm = false) {
    // 1. Smoothly animate cabinet double doors & rotary isolator handle only when moving
    if (this.cabinetLeftDoorPivot && this.cabinetRightDoorPivot && Math.abs(this.cabinetDoorAngle - this.targetCabinetDoorAngle) > 0.001) {
      this.cabinetDoorAngle = THREE.MathUtils.lerp(
        this.cabinetDoorAngle,
        this.targetCabinetDoorAngle,
        Math.min(1.0, delta * 7.5)
      );
      this.cabinetLeftDoorPivot.rotation.y = -this.cabinetDoorAngle;
      this.cabinetRightDoorPivot.rotation.y = this.cabinetDoorAngle;
    }

    if (this.isolatorHandlePivot && Math.abs(this.currentIsolatorAngle - this.targetIsolatorAngle) > 0.001) {
      this.currentIsolatorAngle = THREE.MathUtils.lerp(
        this.currentIsolatorAngle,
        this.targetIsolatorAngle,
        Math.min(1.0, delta * 12.0)
      );
      this.isolatorHandlePivot.rotation.z = this.currentIsolatorAngle;
    }

    const powered = this.cabinetPowerOn || this.cabinetBooting;
    if (this.cabinetMainPowerLampMat) {
      this.cabinetMainPowerLampMat.color.setHex(
        this.cabinetBooting ? 0xf59e0b : (this.cabinetPowerOn ? 0x22c55e : 0x334155)
      );
    }

    // On-Demand 3D Culling: Chỉ hiển thị và cập nhật 18 đèn LED + màn hình biến tần J1000 bên trong tủ điện
    // khi người dùng đang mở cánh tủ hoặc bật chế độ xuyên thấu kiểm tra tủ điện (`cabinetCutawayMode`)!
    const interiorVisible = Boolean(this.cabinetCutawayMode || this.cabinetDoorAngle > 0.02);
    if (this.cabinetInteriorGroup) {
      this.cabinetInteriorGroup.visible = interiorVisible;
    }
    if (!interiorVisible) {
      if (this.cabinetInteriorLight && this.cabinetInteriorLight.intensity !== 0) {
        this.cabinetInteriorLight.intensity = 0;
      }
      return;
    }

    if (this.cabinetInteriorLight) {
      this.cabinetInteriorLight.intensity = powered ? 1.35 : 0.0;
    }

    // 2. Update J1000 AC Inverter 7-Segment LED Display ("50.0 Hz")
    if (!powered) {
      this.updateJ1000DisplayTexture('OFF');
    } else if (this.cabinetBooting) {
      this.updateJ1000DisplayTexture('BOOT');
    } else {
      const freqVal = (50.0 * (this.infeedRateMultiplier || 1.0)).toFixed(1);
      this.updateJ1000DisplayTexture(`${freqVal} Hz`);
    }

    // 3. Update MP3300, YRC1000micro, J1000 & Servo Drive A/B/C LEDs
    if (this.cabinetStatusLeds) {
      const now = performance.now();
      const blinkRun = Math.sin(now * 0.012) > -0.2;
      const blinkLink = Math.sin(now * 0.035) > 0;
      const blinkBoot = Math.sin(now * 0.025) > 0;

      for (const led of this.cabinetStatusLeds) {
        if (!powered) {
          led.mat.color.setHex(0x1e293b);
        } else if (this.cabinetBooting) {
          led.mat.color.setHex(blinkBoot ? 0xf59e0b : 0x22c55e);
        } else if (led.type === 'PWR') {
          led.mat.color.setHex(0x22c55e); // Solid Green PWR
        } else if (led.type === 'RUN') {
          led.mat.color.setHex(blinkRun ? 0x22c55e : 0x14532d); // Flashing Green RUN
        } else if (led.type === 'LINK') {
          led.mat.color.setHex(blinkLink ? 0x4ade80 : 0x15803d); // High-speed Ethernet/Mechatrolink activity
        } else if (led.type === 'ALARM') {
          led.mat.color.setHex(hasSystemAlarm ? 0xef4444 : 0x1e293b); // Red when faulted
        }
      }
    }
  }

  buildSensors() {
    // Photoelectric Sensor DI_11 at Pickup Station (X = -1.25, Z = 0.18)
    this.sensorGroupDI11 = new THREE.Group();
    this.sensorGroupDI11.position.set(-1.25, 0.88, 0.18);
    this.sensorGroupDI11.name = "Sensor_DI_11";

    const sBody = new THREE.Mesh(new THREE.BoxGeometry(0.025, 0.035, 0.05), this.matWallFrame);
    sBody.position.z = 0.22;
    this.sensorGroupDI11.add(sBody);

    this.sensorDI11Led = new THREE.Mesh(
      new THREE.SphereGeometry(0.006, 10, 10),
      new THREE.MeshBasicMaterial({ color: 0x22c55e })
    );
    this.sensorDI11Led.position.set(0, 0.015, 0.246);
    this.sensorGroupDI11.add(this.sensorDI11Led);

    // Active Red Laser Beam
    const beamGeo = new THREE.CylinderGeometry(0.002, 0.002, 0.44, 8);
    beamGeo.rotateX(Math.PI / 2);
    this.beamDI11Mat = new THREE.MeshBasicMaterial({
      color: 0xff0033,
      transparent: true,
      opacity: 0.75,
      blending: THREE.AdditiveBlending
    });
    this.laserBeamDI11 = new THREE.Mesh(beamGeo, this.beamDI11Mat);
    this.sensorGroupDI11.add(this.laserBeamDI11);

    this.root.add(this.sensorGroupDI11);

    // Photoelectric Sensor DI_12 on Rear Box Conveyor (at packing station X = -1.25, Z = -1.45)
    this.sensorGroupDI12 = new THREE.Group();
    this.sensorGroupDI12.position.set(-1.25, 0.77, -1.45);
    this.sensorGroupDI12.name = "Sensor_DI_12";

    const s12Body = new THREE.Mesh(new THREE.BoxGeometry(0.035, 0.035, 0.025), this.matWallFrame);
    s12Body.position.z = 0.26;
    this.sensorGroupDI12.add(s12Body);

    this.sensorDI12Led = new THREE.Mesh(
      new THREE.SphereGeometry(0.006, 10, 10),
      new THREE.MeshBasicMaterial({ color: 0x22c55e })
    );
    this.sensorDI12Led.position.set(0, 0.015, 0.275);
    this.sensorGroupDI12.add(this.sensorDI12Led);

    this.root.add(this.sensorGroupDI12);
  }

  // ===========================================================================
  // REALISTIC PRODUCT GENERATION (Matching Video Screenshot)
  // ===========================================================================

  /**
   * Spawns an individual chocolate bar on the infeed belt
   * Supports normal OK bars and NG defective bars (with visual defect indicator)
   */
  spawnProduct(x = 3.6, zOffset = null, yawDeg = null, isNG = false) {
    const prodGroup = new THREE.Group();
    prodGroup.name = "InfeedProductBar";

    const isDefect = Boolean(isNG || this.nextProductIsNG);
    if (this.nextProductIsNG) {
      this.nextProductIsNG = false;
    }

    const finalZ = (zOffset !== null) ? (-0.32 + zOffset) : (-0.32 + (Math.random() - 0.5) * 0.18);
    const finalYaw = (yawDeg !== null) ? THREE.MathUtils.degToRad(yawDeg) : ((Math.random() - 0.5) * 0.7);

    prodGroup.position.set(x, 0.885, finalZ);
    prodGroup.rotation.y = finalYaw;

    // Dark Chocolate Bar Body
    const barGeo = new THREE.BoxGeometry(0.18, 0.038, 0.06);
    const barMesh = new THREE.Mesh(barGeo, this.matChocolate);
    barMesh.castShadow = true;
    prodGroup.add(barMesh);

    // Wrapper: Normal (Glossy Red Foil) OR Defect (Scorched Maroon with Warning Band)
    const wrapGeo = new THREE.BoxGeometry(0.13, 0.039, 0.061);
    const wrapMesh = new THREE.Mesh(wrapGeo, isDefect ? this.matProductDefectNG : this.matFoilRed);
    wrapMesh.castShadow = true;
    prodGroup.add(wrapMesh);

    // Golden Brand Band (or dark black on defect)
    const goldGeo = new THREE.BoxGeometry(0.025, 0.04, 0.062);
    const goldMesh = new THREE.Mesh(goldGeo, isDefect ? this.matSafetyBlack : this.matFoilGold);
    prodGroup.add(goldMesh);

    // Visual defect marker pip
    let markerMesh = null;
    if (isDefect) {
      const markerGeo = new THREE.CylinderGeometry(0.012, 0.012, 0.006, 12);
      const markerMat = new THREE.MeshBasicMaterial({ color: 0xef4444 });
      markerMesh = new THREE.Mesh(markerGeo, markerMat);
      markerMesh.position.y = 0.022;
      prodGroup.add(markerMesh);
    }

    this.root.add(prodGroup);
    const record = {
      mesh: prodGroup,
      wrapMesh: wrapMesh,
      goldMesh: goldMesh,
      markerMesh: markerMesh,
      initialYaw: finalYaw,
      isDefective: isDefect,
      status: isDefect ? "NG" : "OK",
      rejected: false,
      falling: false,
      chuteProgress: 0,
      fallVy: 0,
      scanned: false
    };
    this.infeedProducts.push(record);
    return prodGroup;
  }

  /**
   * Immediately spawns a defective NG workpiece (`isDefective = true`, `status = "NG"`, bright red `#ef4444` with cracks)
   * right at the start of Conveyor 1 (`X = 3.85m`, upstream of the Cognex Vision Camera at `X = 2.80m`)
   * so it travels along the belt, gets flagged NG by Vision, and gets ejected into the NG Scrap Bin!
   */
  spawnDefectPieceAtStart() {
    // Place at head of Conveyor 1 (or slightly upstream of any existing bar at the start)
    let spawnX = 3.65;
    for (const p of this.infeedProducts) {
      if (!p.picked && !p.rejected && Math.abs(p.mesh.position.x - spawnX) < 0.24) {
        spawnX = Math.min(4.02, p.mesh.position.x + 0.28);
      }
    }
    return this.spawnProduct(spawnX, (Math.random() - 0.5) * 0.10, (Math.random() - 0.5) * 24, true);
  }

  /**
   * Fault Injection Helper: Marks the upcoming chocolate bar as an NG defect
   */
  injectDefectIntoNextProduct() {
    this.nextProductIsNG = true;
    // Also try to find the immediate next product upstream of Vision Camera (X > 2.80m) to defect immediately!
    const candidate = this.infeedProducts
      .filter(p => !p.picked && !p.rejected && p.mesh.position.x > 2.85)
      .sort((a, b) => b.mesh.position.x - a.mesh.position.x)[0];

    if (candidate) {
      candidate.isDefective = true;
      candidate.status = "NG";
      if (candidate.wrapMesh) candidate.wrapMesh.material = this.matProductDefectNG;
      if (candidate.goldMesh) candidate.goldMesh.material = this.matSafetyBlack;
      if (!candidate.markerMesh) {
        const markerGeo = new THREE.CylinderGeometry(0.012, 0.012, 0.006, 12);
        const markerMat = new THREE.MeshBasicMaterial({ color: 0xef4444 });
        const markerMesh = new THREE.Mesh(markerGeo, markerMat);
        markerMesh.position.y = 0.022;
        candidate.mesh.add(markerMesh);
        candidate.markerMesh = markerMesh;
      }
      this.nextProductIsNG = false;
      return candidate;
    }
    return null;
  }

  /**
   * Spawns a collated group of 1..4 bars grouped snugly together in a row on Conveyor 2
   * Supports progressive bar-by-bar collation by the Delta Robot!
   */
  spawnCollatedPack(x = 1.6, initialBarCount = 4) {
    const packGroup = new THREE.Group();
    packGroup.name = "CollatedPack_4Bars";
    packGroup.position.set(x, 0.88, 0.18);

    const barSlots = [];
    // 4 bar slots grouped side-by-side along Z axis (-0.06, -0.02, +0.02, +0.06)
    for (let i = 0; i < 4; i++) {
      const barZ = -0.06 + i * 0.04;
      const barSub = new THREE.Group();
      barSub.position.set(0, 0, barZ);
      barSub.visible = (i < initialBarCount);

      const barMesh = new THREE.Mesh(new THREE.BoxGeometry(0.18, 0.038, 0.038), this.matChocolate);
      barMesh.castShadow = true;

      const wrapMesh = new THREE.Mesh(new THREE.BoxGeometry(0.13, 0.039, 0.039), this.matFoilRed);
      const goldMesh = new THREE.Mesh(new THREE.BoxGeometry(0.025, 0.04, 0.040), this.matFoilGold);

      barSub.add(barMesh, wrapMesh, goldMesh);
      packGroup.add(barSub);
      barSlots.push(barSub);
    }

    this.root.add(packGroup);
    const packRecord = {
      mesh: packGroup,
      stopped: false,
      barCount: initialBarCount,
      barSlots: barSlots,
      complete: (initialBarCount >= 4)
    };
    this.collatedPacks.push(packRecord);
    return packGroup;
  }

  /**
   * Adds 1 chocolate bar into an in-progress collation pack at the Delta drop station (X = +1.20m).
   * Returns the slot index (0..3) where the bar was placed, and releases the 4-pack to roll down Conveyor 2 once all 4 bars are placed!
   */
  placeSingleBarOnConveyor2(dropX = 1.20) {
    let buildingPack = this.collatedPacks.find(p => !p.complete && !p.picked);
    if (!buildingPack) {
      this.spawnCollatedPack(dropX, 0);
      buildingPack = this.collatedPacks[this.collatedPacks.length - 1];
    }

    const slotIdx = Math.min(3, buildingPack.barCount);
    if (buildingPack.barSlots && buildingPack.barSlots[slotIdx]) {
      buildingPack.barSlots[slotIdx].visible = true;
    }
    buildingPack.barCount++;

    if (buildingPack.barCount >= 4) {
      buildingPack.complete = true;
      buildingPack.stopped = false;
    }

    return {
      slotIdx,
      barCount: buildingPack.barCount,
      complete: buildingPack.complete,
      slotWorldZ: 0.18 + (-0.06 + slotIdx * 0.04)
    };
  }

  spawnEmptyBox(x = 0.05) {
    const boxGroup = new THREE.Group();
    boxGroup.name = "OpenCartonBox";
    // Default infeed on rear box conveyor (Z = -1.45m) is at X = +0.05m
    const startX = (x > 0.6) ? 0.05 : x;
    boxGroup.position.set(startX, 0.75, -1.45);

    const thickness = 0.01;
    const wX = 0.28;
    const hY = 0.16;
    const dZ = 0.24;

    const btm = new THREE.Mesh(new THREE.BoxGeometry(wX, thickness, dZ), this.matCardboard);
    btm.position.y = thickness / 2;
    btm.castShadow = true;
    btm.receiveShadow = true;
    boxGroup.add(btm);

    // 4 Walls
    const wallF = new THREE.Mesh(new THREE.BoxGeometry(wX, hY, thickness), this.matCardboard);
    wallF.position.set(0, hY / 2, dZ / 2 - thickness / 2);
    wallF.castShadow = true;

    const wallB = new THREE.Mesh(new THREE.BoxGeometry(wX, hY, thickness), this.matCardboard);
    wallB.position.set(0, hY / 2, -dZ / 2 + thickness / 2);
    wallB.castShadow = true;

    const wallL = new THREE.Mesh(new THREE.BoxGeometry(thickness, hY, dZ - 2 * thickness), this.matCardboard);
    wallL.position.set(-wX / 2 + thickness / 2, hY / 2, 0);
    wallL.castShadow = true;

    const wallR = new THREE.Mesh(new THREE.BoxGeometry(thickness, hY, dZ - 2 * thickness), this.matCardboard);
    wallR.position.set(wX / 2 - thickness / 2, hY / 2, 0);
    wallR.castShadow = true;

    // Outward flaring flaps (4 top flaps: 2 major length flaps + 2 minor side flaps)
    const majorFlapDepth = dZ / 2 + 0.001; // exactly 0.121m so front & back flaps meet tightly at center seam Z = 0!
    const minorFlapWidth = 0.08;

    // Major Front Flap (hinge at +dZ/2, folds inward along -Z to Z = 0)
    const geoFlapFront = new THREE.BoxGeometry(wX, thickness, majorFlapDepth);
    geoFlapFront.translate(0, 0, -majorFlapDepth / 2);
    const flapFront = new THREE.Mesh(geoFlapFront, this.matCardboard);
    flapFront.position.set(0, hY, dZ / 2);
    flapFront.rotation.x = -Math.PI * 0.42; // flared open upwards
    flapFront.castShadow = true;

    // Major Back Flap (hinge at -dZ/2, folds inward along +Z to Z = 0)
    const geoFlapBack = new THREE.BoxGeometry(wX, thickness, majorFlapDepth);
    geoFlapBack.translate(0, 0, majorFlapDepth / 2);
    const flapBack = new THREE.Mesh(geoFlapBack, this.matCardboard);
    flapBack.position.set(0, hY, -dZ / 2);
    flapBack.rotation.x = Math.PI * 0.42; // flared open upwards
    flapBack.castShadow = true;

    // Minor Left Flap (hinge at -wX/2, folds inward along +X)
    const geoFlapLeft = new THREE.BoxGeometry(minorFlapWidth, thickness, dZ - 0.01);
    geoFlapLeft.translate(minorFlapWidth / 2, 0, 0);
    const flapLeft = new THREE.Mesh(geoFlapLeft, this.matCardboard);
    flapLeft.position.set(-wX / 2, hY, 0);
    flapLeft.rotation.z = Math.PI * 0.40; // flared open upwards
    flapLeft.castShadow = true;

    // Minor Right Flap (hinge at +wX/2, folds inward along -X)
    const geoFlapRight = new THREE.BoxGeometry(minorFlapWidth, thickness, dZ - 0.01);
    geoFlapRight.translate(-minorFlapWidth / 2, 0, 0);
    const flapRight = new THREE.Mesh(geoFlapRight, this.matCardboard);
    flapRight.position.set(wX / 2, hY, 0);
    flapRight.rotation.z = -Math.PI * 0.40; // flared open upwards
    flapRight.castShadow = true;

    // Adhesive Packaging Tape Mesh (amber packaging tape along center seam Z = 0)
    const tapeGeo = new THREE.BoxGeometry(wX + 0.04, 0.003, 0.048);
    const tapeMesh = new THREE.Mesh(tapeGeo, this.matTapeAmber);
    tapeMesh.position.set(0, hY + thickness / 2 + 0.002, 0);
    tapeMesh.visible = false;
    tapeMesh.castShadow = true;

    // Industrial Inkjet Dot-Matrix & Barcode Side Label (revealed at CIJ Printer station)
    const labelGeo = new THREE.PlaneGeometry(0.18, 0.095);
    const labelMat = new THREE.MeshBasicMaterial({
      color: 0xffffff,
      side: THREE.DoubleSide
    });
    const printLabelMesh = new THREE.Mesh(labelGeo, labelMat);
    printLabelMesh.position.set(0, hY * 0.52, dZ / 2 + 0.002);
    printLabelMesh.visible = false;

    boxGroup.add(wallF, wallB, wallL, wallR, flapFront, flapBack, flapLeft, flapRight, tapeMesh, printLabelMesh);
    this.root.add(boxGroup);

    const boxRecord = {
      mesh: boxGroup,
      stopped: false,
      full: false,
      flapFront: flapFront,
      flapBack: flapBack,
      flapLeft: flapLeft,
      flapRight: flapRight,
      tapeMesh: tapeMesh,
      printLabelMesh: printLabelMesh,
      sealed: false,
      printed: false,
      barcodeScanned: false,
      boxId: null
    };
    this.cartonBoxes.push(boxRecord);
    return boxGroup;
  }

  spawnInitialItems() {
    // 8 evenly spaced chocolate bars on Conveyor 1 (spacing ~0.55m) so Delta catches every single bar cleanly!
    const infeedInitX = [3.70, 3.15, 2.60, 2.05, 1.50, 0.95, 0.40, -0.15];
    infeedInitX.forEach(x => {
      this.spawnProduct(x, (Math.random() - 0.5) * 0.14, (Math.random() - 0.5) * 35);
    });

    // 3 finished collated 4-packs on Conveyor 2 (at X = -1.25, -0.60, +0.05), leaving X = +1.20m wide open for Delta to build 4-packs bar-by-bar!
    const collateInitX = [-1.25, -0.60, 0.05];
    collateInitX.forEach(x => {
      this.spawnCollatedPack(x, 4);
    });

    // Rear Carton Box Conveyor (Z = -1.45m):
    // 1 open carton box parked at packing station (X = -1.25, Z = -1.45) and 2 empty boxes queued on the right infeed (X = -0.55, X = +0.10)
    this.spawnEmptyBox(-1.25);
    this.spawnEmptyBox(-0.55);
    this.spawnEmptyBox(0.10);
  }

  /**
   * Adjusts chocolate infeed rate & belt speeds dynamically so the user can observe:
   * - Slow rate (0.3x): Clear step-by-step sequence and robots waiting for incoming bars/packs.
   * - Fast rate (2.2x): Continuous high-density stream where both Delta and 6-DOF Arm work at 100% capacity!
   */
  setInfeedRate(multiplier = 1.0) {
    this.infeedRateMultiplier = THREE.MathUtils.clamp(multiplier, 0.2, 2.5);
    this.speedConveyor1 = Number((0.28 * this.infeedRateMultiplier).toFixed(3));
    this.speedConveyor2 = Number((0.48 * Math.max(0.65, Math.min(1.65, this.infeedRateMultiplier))).toFixed(3));
    this.speedConveyor3 = Number((0.55 * Math.max(0.75, Math.min(1.5, this.infeedRateMultiplier))).toFixed(3));

    // Adjust spacing between chocolate bars on Conveyor 1 (wider when slow so robot waits visibly, tighter when fast!)
    if (this.infeedRateMultiplier <= 0.5) {
      this.infeedSpacing = 1.15;
    } else if (this.infeedRateMultiplier >= 1.6) {
      this.infeedSpacing = 0.40;
    } else {
      this.infeedSpacing = 0.55;
    }

    // Re-space upstream bars (X > 1.75m) immediately so the user sees the density change right away
    const upstreamBars = this.infeedProducts
      .filter(p => !p.picked && p.mesh.position.x > 1.75)
      .sort((a, b) => a.mesh.position.x - b.mesh.position.x);

    let cursorX = 1.85;
    for (const item of upstreamBars) {
      item.mesh.position.x = cursorX;
      if (cursorX > 2.80) {
        item.baseEncoderPulse = undefined;
        item.scanX0 = undefined;
        item.scanned = false;
      }
      cursorX += this.infeedSpacing;
    }
  }

  clearAllItems() {
    this.infeedProducts.forEach(p => this.root.remove(p.mesh));
    this.collatedPacks.forEach(p => this.root.remove(p.mesh));
    this.cartonBoxes.forEach(b => this.root.remove(b.mesh));
    if (this.ngScrapBinItems) {
      this.ngScrapBinItems.forEach(s => this.root.remove(s.mesh));
      this.ngScrapBinItems = [];
    }

    this.infeedProducts = [];
    this.collatedPacks = [];
    this.cartonBoxes = [];

    this.sensorDI11 = false;
    this.sensorDI12 = false;
    this.updateSensorVisuals();
  }

  update(delta) {
    // 1. Conveyor Texture Scrolling (Physically matched to object linear velocities)
    // Belt repeat = 29 along 5.80m length -> 0.20m per texture tile period
    const tilePitch = 0.20;

    if (this.conveyor1Active && this.belt1Texture) {
      this.belt1Texture.offset.x = (this.belt1Texture.offset.x + (this.speedConveyor1 * delta) / tilePitch) % 1.0;
      if (this.beltTexture) this.beltTexture.offset.x = this.belt1Texture.offset.x;

      // -----------------------------------------------------------------------
      // 1a. Conveyor 1 Rotary Encoder High-Speed Counter (HSC) & Shaft Rotation
      // -----------------------------------------------------------------------
      const pulleyRadius = 0.042;
      const dThetaEnc = (this.speedConveyor1 * delta) / pulleyRadius;
      this.encoderRotAngle += dThetaEnc;
      if (this.encoderRotatingShaftGroup) {
        this.encoderRotatingShaftGroup.rotation.z = this.encoderRotAngle;
      }

      // Accumulate encoder pulses: deltaDist / K_scale (K_scale = 0.0001 m/pulse = 0.1 mm/pulse)
      const deltaDist = this.speedConveyor1 * delta;
      const deltaPulses = deltaDist / this.encoderScaleK;
      this.encoderPulseCounter += deltaPulses;
      this.Encoder_Pulse_Counter = Math.floor(this.encoderPulseCounter);
      this.encoderFreqHz = Math.round(this.speedConveyor1 / this.encoderScaleK);

      // Quadrature A/B 90-deg phase shift simulation
      this.encoderChannelA = (Math.floor(this.encoderPulseCounter / 18) % 2 === 0);
      this.encoderChannelB = (Math.floor((this.encoderPulseCounter + 9) / 18) % 2 === 0);

      if (this.encoderLedA) {
        this.encoderLedA.material.color.setHex(this.encoderChannelA ? 0x22c55e : 0x14532d);
      }
      if (this.encoderLedB) {
        this.encoderLedB.material.color.setHex(this.encoderChannelB ? 0x38bdf8 : 0x0c4a6e);
      }
    } else {
      this.encoderFreqHz = 0;
    }

    if (this.conveyor2Active && this.belt2Texture) {
      this.belt2Texture.offset.x = (this.belt2Texture.offset.x + (this.speedConveyor2 * delta) / tilePitch) % 1.0;
    }

    if (this.conveyor3Active && this.rollers3) {
      // Rotate 32 chrome rollers around Z axis (r = 0.020m -> dTheta = (v * delta) / r)
      const dTheta = (this.speedConveyor3 * delta) / 0.020;
      for (let r = 0; r < this.rollers3.length; r++) {
        this.rollers3[r].rotation.z += dTheta;
      }
    }

    // -------------------------------------------------------------------------
    // 1b. Quality Control Reject Pusher Pneumatic Cylinder & NG Bin Physics
    // -------------------------------------------------------------------------
    // Pusher stroke mechanical animation (Fast 0.15s extension & 0.15s retraction: 0.33m / 0.15s = 2.20 m/s)
    if (this.pusherState === "EXTENDING") {
      this.pusherCurrentZ -= 2.20 * delta;
      if (this.activeRejectItem && !this.activeRejectItem.falling) {
        this.activeRejectItem.mesh.position.z = Math.min(this.activeRejectItem.mesh.position.z, this.pusherCurrentZ - 0.035);
        if (this.activeRejectItem.mesh.position.z <= -0.52) {
          // Pushed past edge of belt onto the 35° galvanized guide chute!
          this.activeRejectItem.falling = true;
          this.activeRejectItem.chuteProgress = 0;
          this.activeRejectItem.fallVy = 0.25;
        }
      }
      if (this.pusherCurrentZ <= -0.46) {
        this.pusherCurrentZ = -0.46;
        this.pusherState = "RETRACTING";
      }
    } else if (this.pusherState === "RETRACTING") {
      this.pusherCurrentZ += 2.20 * delta; // 0.15s retraction back to -0.13m
      if (this.pusherCurrentZ >= -0.13) {
        this.pusherCurrentZ = -0.13;
        this.pusherState = "RETRACTED";
        this.rejectCylinderDO = false;
        this.DO_REJECT_CYLINDER = false;
        this.activeRejectItem = null;
      }
    } else {
      this.pusherCurrentZ = -0.13;
      this.rejectCylinderDO = false;
      this.DO_REJECT_CYLINDER = false;
    }

    if (this.pusherRodGroup) {
      this.pusherRodGroup.position.z = this.pusherCurrentZ;
    }
    if (this.rejectCylinderLed) {
      this.rejectCylinderLed.material.color.setHex(this.rejectCylinderDO ? 0xef4444 : 0x22c55e);
    }

    // 1c. Animate rejected NG pieces sliding down the 35° guide chute & dropping into the NG Scrap Bin
    //     (Runs even if Conveyor 1 is temporarily paused after the piston strike)
    for (let i = this.infeedProducts.length - 1; i >= 0; i--) {
      const item = this.infeedProducts[i];
      if (!item.falling) continue;

      // Phase 1: Slide along the 35° galvanized guide chute (Z: -0.52m -> -0.73m, Y: 0.87m -> 0.72m)
      if (item.mesh.position.z > -0.72) {
        const slideSpeed = 1.15; // m/s along 35° slope
        const slopeRad = THREE.MathUtils.degToRad(35);
        item.mesh.position.z -= Math.cos(slopeRad) * slideSpeed * delta;
        item.mesh.position.y -= Math.sin(slopeRad) * slideSpeed * delta;
        item.mesh.rotation.x = THREE.MathUtils.lerp(item.mesh.rotation.x, -slopeRad, Math.min(1.0, delta * 12.0));
      } else {
        // Phase 2: Gravity free-fall from chute lip into the bottom of the NG Scrap Bin (Y -> 0.29m)
        item.fallVy += 9.8 * delta;
        item.mesh.position.y -= item.fallVy * delta;
        item.mesh.position.z = THREE.MathUtils.lerp(item.mesh.position.z, -0.76, Math.min(1.0, delta * 5.0));
        item.mesh.rotation.x -= 4.2 * delta;
        item.mesh.rotation.z += 2.0 * delta;
      }

      const binRestingY = 0.285 + Math.min(0.08, (this.ngScrapBinItems ? this.ngScrapBinItems.length : 0) * 0.015);
      if (item.mesh.position.y <= binRestingY) {
        // Landed inside bottom of NG Scrap Bin! Keep visible inside bin and fade out after 5 seconds.
        item.mesh.position.y = binRestingY;
        item.mesh.position.x = 2.45 + (Math.random() - 0.5) * 0.10;
        item.mesh.position.z = -0.76 + (Math.random() - 0.5) * 0.10;
        item.mesh.rotation.set((Math.random() - 0.5) * 0.25, item.mesh.rotation.y, (Math.random() - 0.5) * 0.25);

        this.infeedProducts.splice(i, 1);
        this.ngDefectCount++;
        this.NG_Defect_Count = this.ngDefectCount;
        if (this.activeRejectItem === item) this.activeRejectItem = null;

        this.ngScrapBinItems.push({
          mesh: item.mesh,
          age: 0,
          maxLife: 5.5 // Remains in bin for 5s, then fades out over 0.5s
        });
      }
    }

    // 1d. Manage NG Scrap Bin resting items (retain for 5s, then fade out & dispose to prevent memory buildup)
    if (this.ngScrapBinItems && this.ngScrapBinItems.length > 0) {
      for (let s = this.ngScrapBinItems.length - 1; s >= 0; s--) {
        const scrap = this.ngScrapBinItems[s];
        scrap.age += delta;
        if (scrap.age >= 5.0) {
          const fadeAlpha = Math.max(0, 1.0 - (scrap.age - 5.0) / 0.5);
          scrap.mesh.scale.setScalar(Math.max(0.01, fadeAlpha));
          if (scrap.age >= scrap.maxLife) {
            this.root.remove(scrap.mesh);
            this.ngScrapBinItems.splice(s, 1);
          }
        }
      }
    }

    if (this.conveyor1Active) {
      for (let i = this.infeedProducts.length - 1; i >= 0; i--) {
        const item = this.infeedProducts[i];
        if (item.picked || item.falling) continue;

        // Advance product along Conveyor 1 belt (Synchronized with Rotary Encoder pulses)
        if (item.baseEncoderPulse !== undefined && item.scanX0 !== undefined && !item.rejected) {
          // Conveyor Tracking Formula: X(t) = X_0 + Delta_Pulse * K_scale (directional along -X)
          const deltaPulse = this.encoderPulseCounter - item.baseEncoderPulse;
          item.mesh.position.x = item.scanX0 - (deltaPulse * this.encoderScaleK);
          this.lastTrackedX = item.mesh.position.x;
          this.lastTrackedDeltaPulse = Math.floor(deltaPulse);
        } else {
          item.mesh.position.x -= this.speedConveyor1 * delta;
        }
        if (!item.rejected) {
          item.mesh.position.y = 0.88;
          item.mesh.rotation.x = 0;
          item.mesh.rotation.z = 0;
        }

        // Shift Tracking: Trigger Reject Pusher when NG defect product (`status === "NG"` or `isDefective === true`) centers in front of pusher (X = 2.45m)
        if ((item.status === "NG" || item.isDefective) && !item.rejected && this.pusherState === "RETRACTED") {
          if (item.mesh.position.x <= 2.52 && item.mesh.position.x >= 2.38) {
            this.pusherState = "EXTENDING";
            this.rejectCylinderDO = true;
            this.DO_REJECT_CYLINDER = true;
            item.rejected = true;
            this.activeRejectItem = item;
          }
        }

        // Recycle normal items if they reach far end of Conveyor 1
        if (item.mesh.position.x < -1.55) {
          let maxInfeedX = 3.3;
          for (const p of this.infeedProducts) {
            if (p !== item && !p.picked && !p.rejected && p.mesh.position.x > maxInfeedX) {
              maxInfeedX = p.mesh.position.x;
            }
          }
          item.mesh.position.x = Math.max(3.65, maxInfeedX + (this.infeedSpacing || 0.55));
          item.mesh.position.z = -0.32 + (Math.random() - 0.5) * 0.16;
          item.mesh.rotation.y = (Math.random() - 0.5) * 0.7;
          item.baseEncoderPulse = undefined;
          item.scanX0 = undefined;
          item.scanned = false;
          // Re-evaluate defect queue flag
          if (this.nextProductIsNG) {
            item.isDefective = true;
            item.status = "NG";
            if (item.wrapMesh) item.wrapMesh.material = this.matProductDefectNG;
            if (item.goldMesh) item.goldMesh.material = this.matSafetyBlack;
            if (item.crackGroup) item.crackGroup.visible = true;
            this.nextProductIsNG = false;
          } else {
            item.isDefective = false;
            item.status = "OK";
            if (item.wrapMesh) item.wrapMesh.material = this.matFoilRed;
            if (item.goldMesh) item.goldMesh.material = this.matFoilGold;
            if (item.crackGroup) item.crackGroup.visible = false;
          }
          if (item.markerMesh) item.markerMesh.visible = (item.status === "NG");
        }
      }
    }

    // 2. Conveyor 2 (Collation) - Moves completed 4-packs from Delta drop (X = +1.20) to Arm pickup (X = -1.25)
    let packDetected = false;
    for (let i = 0; i < this.collatedPacks.length; i++) {
      const pack = this.collatedPacks[i];
      if (pack.picked) continue;

      // If this pack is still being assembled bar-by-bar (1/4, 2/4, 3/4) by Delta at X = 1.20m, hold it stationary at X = 1.20m!
      if (pack.complete === false) {
        pack.mesh.position.set(1.20, 0.88, 0.18);
        pack.mesh.rotation.set(0, 0, 0);
        pack.stopped = true;
        continue;
      }

      // Stop automatically at Sensor DI_11 pickup station (X = -1.25) so 6-DOF Arm can grip it cleanly
      const distToStopper = pack.mesh.position.x - (-1.25);
      if (distToStopper <= 0.05 && distToStopper >= -0.15) {
        pack.mesh.position.x = -1.25;
        pack.stopped = true;
        packDetected = true;
      } else {
        pack.stopped = false;
      }

      if (this.conveyor2Active && !pack.stopped) {
        // Prevent colliding into the pack ahead of it (smaller 0.36m spacing so the queue advances visibly!)
        let blockedByAhead = false;
        for (let j = 0; j < this.collatedPacks.length; j++) {
          if (i === j || this.collatedPacks[j].picked) continue;
          const dx = pack.mesh.position.x - this.collatedPacks[j].mesh.position.x;
          if (dx > 0 && dx < 0.36) {
            blockedByAhead = true;
            break;
          }
        }
        if (!blockedByAhead) {
          pack.mesh.position.x -= this.speedConveyor2 * delta;
          if (pack.mesh.position.x <= -1.25) {
            pack.mesh.position.x = -1.25;
            pack.stopped = true;
            packDetected = true;
          }
        }
      }

      pack.mesh.position.y = 0.88;
      pack.mesh.position.z = 0.18;
      pack.mesh.rotation.set(0, 0, 0);

      if (pack.mesh.position.x < -3.6) {
        pack.mesh.position.x = 2.2;
        pack.stopped = false;
      }
    }
    this.sensorDI11 = this.sensorDI11Failure ? false : packDetected;

    // 2b. Maintenance Door, Andon Tower Light & Main Control Cabinet Real-time Updates
    if (this.doorGroup) {
      this.doorGroup.rotation.y = THREE.MathUtils.lerp(
        this.doorGroup.rotation.y,
        this.targetDoorAngle,
        Math.min(1.0, delta * 8.0)
      );
    }
    this.updateAndonLight(delta);
    this.updateControlCabinet(delta, this.andonState === 'ALARM');

    // 3. Conveyor 3 (Rear Carton Box Conveyor behind 6-DOF Arm at Z = -1.45m, X = +0.50m to -3.60m)
    // Empty boxes feed in from the right (X = +0.15m) and roll left (-X) up to the stopper at X = -1.25m.
    // When box is full (2 layers = 8 bars), the stopper cylinder retracts below rollers and clamps release,
    // allowing the full box to roll out along the conveyor through the 400mm outfeed safety tunnel (X -> -3.90m)!
    let boxDetected = false;
    const anyFullDischarging = this.cartonBoxes.some(b => b.full && b.mesh.position.x > -2.2);

    if (anyFullDischarging) {
      this.stopperState = "RETRACTED";
      this.clampState = "RELEASED";
    } else {
      const boxAtStation = this.cartonBoxes.some(b => !b.full && Math.abs(b.mesh.position.x - (-1.25)) < 0.06);
      if (boxAtStation) {
        this.stopperState = "EXTENDED";
        this.clampState = "CLAMPED";
      } else {
        this.stopperState = "EXTENDED"; // Standing ready to catch incoming box
        this.clampState = "RELEASED";   // Open to allow box entry
      }
    }

    // Smooth Pneumatic Cylinder & Clamp Mechanical Animation
    const targetStopperY = (this.stopperState === "EXTENDED") ? 0.785 : 0.705;
    const targetFrontZ = (this.clampState === "CLAMPED") ? -1.325 : -1.240;
    const targetRearZ = (this.clampState === "CLAMPED") ? -1.575 : -1.660;

    this.stopperCurrentY = THREE.MathUtils.lerp(this.stopperCurrentY, targetStopperY, Math.min(1.0, delta * 14.0));
    this.frontClampCurrentZ = THREE.MathUtils.lerp(this.frontClampCurrentZ, targetFrontZ, Math.min(1.0, delta * 12.0));
    this.rearClampCurrentZ = THREE.MathUtils.lerp(this.rearClampCurrentZ, targetRearZ, Math.min(1.0, delta * 12.0));

    if (this.stopperPistonGroup) {
      this.stopperPistonGroup.position.y = this.stopperCurrentY;
    }
    if (this.frontClampGroup) {
      this.frontClampGroup.position.z = this.frontClampCurrentZ;
    }
    if (this.rearClampGroup) {
      this.rearClampGroup.position.z = this.rearClampCurrentZ;
    }

    // -------------------------------------------------------------------------
    // 3b. Automatic Top Carton Sealer on Conveyor 3 (X = -2.40m, Sensor at X = -2.15m)
    // -------------------------------------------------------------------------
    const boxInSealerZone = this.cartonBoxes.some(b => b.full && b.mesh.position.x <= -2.05 && b.mesh.position.x >= -2.55);
    this.sealerSensorActive = boxInSealerZone;
    this.DI_SEALER_SENSOR = boxInSealerZone;
    this.sealerActive = boxInSealerZone;
    this.DO_SEALER_ACTIVE = boxInSealerZone;

    if (this.sealerSensorLed) {
      this.sealerSensorLed.material.color.setHex(boxInSealerZone ? 0xef4444 : 0x22c55e);
    }
    if (this.sealerBeamMesh) {
      this.sealerBeamMesh.material.opacity = boxInSealerZone ? 0.95 : 0.35;
    }

    // -------------------------------------------------------------------------
    // 3c. Industrial Inkjet Printer & Barcode Scanner on Conveyor 3 (X = -3.05m)
    // -------------------------------------------------------------------------
    const boxInPrintTrigger = this.cartonBoxes.some(b => b.full && b.mesh.position.x <= -2.76 && b.mesh.position.x >= -3.06);
    const boxInInkjetFire = this.cartonBoxes.some(b => b.full && b.mesh.position.x <= -2.86 && b.mesh.position.x >= -3.08);
    const boxInBarcodeZone = this.cartonBoxes.some(b => b.full && b.mesh.position.x <= -3.06 && b.mesh.position.x >= -3.32);

    this.printTriggerDI = boxInPrintTrigger;
    this.DI_PRINT_TRIGGER = boxInPrintTrigger;
    this.inkjetActiveDO = boxInInkjetFire;
    this.DO_INKJET_ACTIVE = boxInInkjetFire;
    this.DO_INKJET_FIRE = boxInInkjetFire;

    if (this.printTriggerLed) {
      this.printTriggerLed.material.color.setHex(boxInPrintTrigger ? 0xef4444 : 0x22c55e);
    }
    if (this.printTriggerBeamMesh) {
      this.printTriggerBeamMesh.material.opacity = boxInPrintTrigger ? 0.95 : 0.35;
    }
    if (this.inkjetFireLed) {
      this.inkjetFireLed.material.color.setHex(boxInInkjetFire ? 0x38bdf8 : 0x475569);
    }
    if (this.inkjetSprayMat) {
      this.inkjetSprayMat.opacity = boxInInkjetFire ? (0.45 + 0.45 * Math.sin(performance.now() * 0.08)) : 0.0;
    }

    // Barcode Red Laser Fan & Scan Timer (~0.20s active laser sweep)
    if (this.barcodeScanFlashTimer > 0) {
      this.barcodeScanFlashTimer = Math.max(0, this.barcodeScanFlashTimer - delta);
    }
    const laserActive = boxInBarcodeZone || (this.barcodeScanFlashTimer > 0);
    if (this.barcodeLaserFanMat) {
      const flicker = 0.45 + 0.45 * Math.abs(Math.sin(performance.now() * 0.06));
      this.barcodeLaserFanMat.opacity = laserActive ? flicker : 0.0;
    }
    if (this.barcodeLaserLineMat) {
      this.barcodeLaserLineMat.opacity = laserActive ? 0.95 : 0.0;
    }
    if (this.barcodeScannerLed) {
      if (this.barcodeReadError) {
        this.barcodeScannerLed.material.color.setHex(0xef4444);
      } else if (laserActive || this.barcodeScanValid) {
        this.barcodeScannerLed.material.color.setHex(0x22c55e);
      } else {
        this.barcodeScannerLed.material.color.setHex(0x166534);
      }
    }
    if (!boxInBarcodeZone && this.barcodeScanFlashTimer <= 0) {
      this.DI_BARCODE_OK = false;
    }

    for (let i = this.cartonBoxes.length - 1; i >= 0; i--) {
      const box = this.cartonBoxes[i];
      if (box.full) {
        // Full box rolls out to the left (-X) along the rear box conveyor only when conveyor 3 is running!
        if (this.conveyor3Active) {
          box.mesh.position.x -= this.speedConveyor3 * 1.65 * delta;

          // Process top flap folding and amber packaging tape application under sealer (Case Sealer at X = -2.40m)
          if (box.mesh.position.x <= -1.98) {
            if (box.mesh.position.x >= -2.48 && !box.sealed) {
              // Normalize progress t in [0..1] across the sealer plows (X: -1.98m -> -2.48m)
              const t = THREE.MathUtils.clamp((-1.98 - box.mesh.position.x) / 0.45, 0.0, 1.0);

              // 1. Fold side minor flaps down first under side plows (t: 0.0 -> 0.35)
              const tSide = THREE.MathUtils.clamp(t / 0.35, 0.0, 1.0);
              if (box.flapLeft) box.flapLeft.rotation.z = THREE.MathUtils.lerp(Math.PI * 0.40, 0, tSide);
              if (box.flapRight) box.flapRight.rotation.z = THREE.MathUtils.lerp(-Math.PI * 0.40, 0, tSide);

              // 2. Fold front & rear major flaps down to flat under top rollers (t: 0.15 -> 0.75)
              const tMajor = THREE.MathUtils.clamp((t - 0.15) / 0.60, 0.0, 1.0);
              if (box.flapFront) box.flapFront.rotation.x = THREE.MathUtils.lerp(-Math.PI * 0.42, 0, tMajor);
              if (box.flapBack) box.flapBack.rotation.x = THREE.MathUtils.lerp(Math.PI * 0.42, 0, tMajor);

              // 3. Reveal and stretch top packaging adhesive tape line along center seam (t: 0.35 -> 1.0)
              if (box.tapeMesh && t > 0.35) {
                box.tapeMesh.visible = true;
                const tTape = THREE.MathUtils.clamp((t - 0.35) / 0.60, 0.0, 1.0);
                box.tapeMesh.scale.x = tTape;
              }
            } else {
              // Once at or past the sealer head (X <= -2.48m): GUARANTEE 100% COMPLETELY CLOSED & SEALED!
              if (!box.sealed) {
                box.sealed = true;
                box.mesh.name = "SealedCartonBox";
                this.sealedCartonCount++;
              }
              if (box.flapLeft) box.flapLeft.rotation.z = 0;
              if (box.flapRight) box.flapRight.rotation.z = 0;
              if (box.flapFront) box.flapFront.rotation.x = 0;
              if (box.flapBack) box.flapBack.rotation.x = 0;
              if (box.tapeMesh) {
                box.tapeMesh.visible = true;
                box.tapeMesh.scale.x = 1.0;
              }
            }
          }

          // Process CIJ Inkjet Dot-Matrix Printing on Carton Side Wall (X = -2.86m to -3.08m)
          if (box.mesh.position.x <= -2.86 && box.mesh.position.x >= -3.12) {
            if (!box.printed) {
              box.printed = true;
              const yyyy = new Date().getFullYear();
              box.boxId = `BOX-${yyyy}-${String(this.nextBoxSerial++).padStart(4, '0')}`;
              if (box.printLabelMesh) {
                const tex = this.createBoxPrintTexture(
                  box.boxId,
                  this.currentMfgDate,
                  this.currentExpDate,
                  this.currentLotCode
                );
                box.printLabelMesh.material.map = tex;
                box.printLabelMesh.material.needsUpdate = true;
                box.printLabelMesh.visible = true;
              }
            }
            if (box.printLabelMesh) {
              const pScale = THREE.MathUtils.clamp((-2.86 - box.mesh.position.x) / 0.16, 0.15, 1.0);
              box.printLabelMesh.scale.x = pScale;
            }
          }

          // Process Barcode Scanner Verification (X <= -3.12m)
          if (box.mesh.position.x <= -3.12 && !box.barcodeScanned) {
            box.barcodeScanned = true;
            this.barcodeScanFlashTimer = 0.22; // 0.2s laser scan burst
            if (box.printLabelMesh) {
              box.printLabelMesh.scale.x = 1.0;
            }
            if (!this.injectBarcodeError) {
              this.barcodeScanValid = true;
              this.Barcode_Scan_Valid = true;
              this.DI_BARCODE_OK = true;
              this.barcodeReadError = false;
              this.Barcode_Read_Error = false;
              this.totalVerifiedBoxes++;
              this.Total_Verified_Boxes = this.totalVerifiedBoxes;
              this.playScannerBeep(true);
              this.lastPrintedBoxInfo = {
                boxId: box.boxId || `BOX-2026-0042`,
                mfg: this.currentMfgDate,
                exp: this.currentExpDate,
                lot: this.currentLotCode,
                status: 'VERIFIED [OK]',
                valid: true
              };
            } else {
              this.barcodeScanValid = false;
              this.Barcode_Scan_Valid = false;
              this.DI_BARCODE_OK = false;
              this.barcodeReadError = true;
              this.Barcode_Read_Error = true;
              this.playScannerBeep(false);
              this.lastPrintedBoxInfo = {
                boxId: box.boxId || `BOX-2026-ERR`,
                mfg: this.currentMfgDate,
                exp: this.currentExpDate,
                lot: this.currentLotCode,
                status: 'READ ERROR [NG]',
                valid: false
              };
            }
          }

          // End-of-Line Transfer through the 400mm Outfeed Safety Tunnel (X = -3.85m) onto AGV Euro Pallet!
          if (box.mesh.position.x <= -3.82) {
            if (this.agvController) {
              if (this.agvController.canAcceptBoxFromConveyor()) {
                // Transfer box smoothly onto AGV Pallet slot (x_i, y_i, z_i)
                this.cartonBoxes.splice(i, 1);
                this.agvController.receiveBoxFromConveyor(box);
                if (this.cartonBoxes.filter(b => !b.full).length < 3) {
                  this.spawnEmptyBox(0.15);
                }
              } else {
                // AGV is away delivering to Virtual Warehouse: hold box buffered at end-of-line sensor (X = -3.82m)
                box.mesh.position.x = -3.82;
                if (this.agvTransferSensorLed) {
                  this.agvTransferSensorLed.material.color.setHex(0xf59e0b);
                }
              }
            } else {
              this.root.remove(box.mesh);
              this.cartonBoxes.splice(i, 1);
              // Ensure empty box queue on the right is replenished
              if (this.cartonBoxes.filter(b => !b.full).length < 3) {
                this.spawnEmptyBox(0.15);
              }
            }
          }
        }
        continue;
      }

      // Advance empty box from right (+X) toward the packing station stopper at X = -1.25m
      if (box.mesh.position.x > -1.25) {
        // Check if blocked by another non-full box ahead of it (to its left, lower X)
        let blocked = false;
        for (let j = 0; j < this.cartonBoxes.length; j++) {
          if (i === j || this.cartonBoxes[j].full) continue;
          const dx = box.mesh.position.x - this.cartonBoxes[j].mesh.position.x;
          if (dx > 0 && dx < 0.52) {
            blocked = true;
            break;
          }
        }

        if (!blocked && this.conveyor3Active) {
          box.mesh.position.x -= this.speedConveyor3 * delta;
          if (box.mesh.position.x <= -1.25 && this.stopperState === "EXTENDED") {
            box.mesh.position.x = -1.25;
            box.stopped = true;
          }
        }
      }

      if (Math.abs(box.mesh.position.x - (-1.25)) < 0.06) {
        boxDetected = true;
      }
    }
    this.sensorDI12 = boxDetected;

    this.updateSensorVisuals();
  }

  /**
   * Toggles the Maintenance Safety Door (Open / Closed)
   * @param {boolean|null} forceClosed - Optional explicit state
   * @returns {boolean} Current door closed state (true = closed, false = open)
   */
  toggleSafetyDoor(forceClosed = null) {
    if (forceClosed !== null) {
      this.safetyGateClosed = Boolean(forceClosed);
    } else {
      this.safetyGateClosed = !this.safetyGateClosed;
    }
    this.SafetyGate_Closed = this.safetyGateClosed;
    this.targetDoorAngle = this.safetyGateClosed ? 0 : -Math.PI * 0.45;

    if (this.interlockSwitchLed) {
      this.interlockSwitchLed.material.color.setHex(this.safetyGateClosed ? 0x22c55e : 0xef4444);
    }

    return this.safetyGateClosed;
  }

  /**
   * Sets the Andon Signal Tower Light operational mode
   * @param {'RUN'|'STANDBY'|'ALARM'} state
   */
  setAndonState(state) {
    this.andonState = state;
  }

  /**
   * Updates Andon Signal Tower Light lamp intensities and flash timings
   * @param {number} delta
   */
  updateAndonLight(delta) {
    if (!this.matAndonRed || !this.matAndonYellow || !this.matAndonGreen) return;

    if (this.andonState === 'ALARM') {
      this.andonBlinkTimer += delta;
      if (this.andonBlinkTimer >= 0.20) { // 2.5 Hz flash
        this.andonBlinkTimer = 0;
        this.andonBlinkVisible = !this.andonBlinkVisible;
      }
      const intensity = this.andonBlinkVisible ? 2.8 : 0.05;
      this.matAndonRed.emissiveIntensity = intensity;
      this.matAndonYellow.emissiveIntensity = 0.05;
      this.matAndonGreen.emissiveIntensity = 0.05;
      if (this.andonPointLight) {
        this.andonPointLight.color.setHex(0xef4444);
        this.andonPointLight.intensity = this.andonBlinkVisible ? 1.6 : 0.0;
      }
    } else if (this.andonState === 'RUN') {
      this.matAndonRed.emissiveIntensity = 0.05;
      this.matAndonYellow.emissiveIntensity = 0.05;
      this.matAndonGreen.emissiveIntensity = 2.4;
      if (this.andonPointLight) {
        this.andonPointLight.color.setHex(0x22c55e);
        this.andonPointLight.intensity = 1.2;
      }
    } else { // 'STANDBY' / 'PAUSE' / 'JOG'
      this.matAndonRed.emissiveIntensity = 0.05;
      this.matAndonYellow.emissiveIntensity = 2.4;
      this.matAndonGreen.emissiveIntensity = 0.05;
      if (this.andonPointLight) {
        this.andonPointLight.color.setHex(0xf59e0b);
        this.andonPointLight.intensity = 1.0;
      }
    }
  }

  updateSensorVisuals() {
    if (this.sensorDI11) {
      this.sensorDI11Led.material.color.setHex(0xef4444);
      this.beamDI11Mat.opacity = 0.2;
    } else {
      this.sensorDI11Led.material.color.setHex(0x22c55e);
      this.beamDI11Mat.opacity = 0.85;
    }

    if (this.sensorDI12) {
      this.sensorDI12Led.material.color.setHex(0xef4444);
    } else {
      this.sensorDI12Led.material.color.setHex(0x22c55e);
    }
  }
}
