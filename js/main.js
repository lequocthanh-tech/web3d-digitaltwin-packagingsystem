import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { FactoryEnvironment } from './scene/FactoryEnvironment.js?v=3.5';
import { DeltaRobot3D } from './robots/DeltaRobot3D.js?v=3.5';
import { ArticulatedArm6DOF } from './robots/ArticulatedArm6DOF.js?v=3.3';
import { Yaskawa6DOFKinematics, YASKAWA_PULSE_RATIOS } from './kinematics/Yaskawa6DOFKinematics.js?v=3.3';
import { Delta4DOFKinematics } from './kinematics/Delta4DOFKinematics.js?v=3.3';
import { TrajectoryPlanner } from './kinematics/TrajectoryPlanner.js?v=3.3';
import { TrajectoryTrail } from './kinematics/TrajectoryTrail.js?v=3.5';
import { VacuumAndSensorEngine } from './physics/VacuumAndSensorEngine.js?v=3.5';
import { MP3300HardwareBus } from './hardware/MP3300HardwareBus.js?v=3.3';
import { YRC1000EthernetServer } from './hardware/YRC1000EthernetServer.js?v=3.3';
import { TeachPendantStation } from './workstations/TeachPendantStation.js?v=3.3';
import { DeltaTeachPendantStation } from './workstations/DeltaTeachPendantStation.js?v=3.3';
import { MPE720StudioStation } from './workstations/MPE720StudioStation.js?v=3.3';
import { CSharpPCStudioStation } from './workstations/CSharpPCStudioStation.js?v=3.0';
import { VIPAHMIScreen } from './hmi-scada/VIPAHMIScreen.js?v=3.0';
import { TelemetryAndGraphs } from './hmi-scada/TelemetryAndGraphs.js?v=3.4';
import { FaultInjectionPanel } from './hmi-scada/FaultInjectionPanel.js?v=3.0';
import { MatlabKinematicsPlotter } from './kinematics/MatlabKinematicsPlotter.js?v=3.4';
import { OEECalculator } from './analytics/OEECalculator.js?v=3.0';
import { AGVController } from './logistics/AGVController.js?v=3.0';
import { MachineVisionController } from './vision/MachineVisionController.js?v=3.0';
import { runAutoLetterTJob } from './workstations/JobLetterT.js?v=3.0';

// Expose THREE on window for console & devtools interaction
if (typeof window !== 'undefined') {
  window.THREE = THREE;
}

/**
 * Main Controller - Digital Twin Packaging Line
 * Coordinates 3D Scene, Animation Loop, Anti-Lag Rendering & Interactive Camera Navigation.
 */
class DigitalTwinApp {
  constructor() {
    this.container = document.getElementById('viewport-container');
    this.canvas = document.getElementById('canvas3d');

    // System States
    this.systemPower = false;
    this.autoDemoMode = false;
    this.autoDemoTime = 0;
    this.isSafetyInterlockTripped = false; // Safety Gate Interlock Trip Flag
    this.clock = new THREE.Clock();
    
    // FPS & Performance tracking
    this.frameCount = 0;
    this.lastFpsTime = performance.now();
    this.fpsValueEl = document.getElementById('telemetry-fps');

    // Camera Target Interpolation
    this.camTargetPos = new THREE.Vector3();
    this.camTargetLookAt = new THREE.Vector3();
    this.isTransitioningCam = false;

    // Factory I/O Free / Fly Camera navigation state
    this.isFlyCamActive = false;
    this.camVelocity = new THREE.Vector3(0, 0, 0);
    this.camKeys = {
      ArrowUp: false,
      ArrowDown: false,
      ArrowLeft: false,
      ArrowRight: false,
      KeyW: false,
      KeyS: false,
      KeyA: false,
      KeyD: false,
      Space: false,
      KeyC: false,
      PageUp: false,
      PageDown: false,
      ShiftLeft: false,
      ShiftRight: false
    };
    this.isRightMouseDown = false;
    this.lastRightMousePos = { x: 0, y: 0 };

    // Raycaster for double-click focus
    this.raycaster = new THREE.Raycaster();
    this.mouse = new THREE.Vector2();

    try {
      this.initThree();
      this.initLights();
      this.initSceneObjects();
      this.initKinematicsAndPhysics();
      this.initHardwareBus();
      this.initEngineeringWorkstations();
      this.initCameraPresets();
      this.initFlyCameraListeners();
      this.initUIEventListeners();
      this.initKinematicsLabListeners();
      this.initHardwareUIEventListeners();
    } catch (err) {
      console.error('[DigitalTwinApp] Lỗi khởi tạo hệ thống:', err);
    }

    // Dedicated Industrial PLC Scan Cycle (50 Hz / 20ms interval) decoupled from 60 FPS rendering
    this.plcScanTimer = setInterval(() => {
      if (!this.systemPower || document.hidden) return;
      if (this.hardwareBus) {
        this.hardwareBus.updatePLCLogic(0.02, {
          runRobotPickCycle: () => this.runSinglePickAndPlaceCycle(),
          isRobotBusy: this.isSingleCycleRunning
        });
      }
    }, 20);

    window.addEventListener('resize', () => this.onWindowResize());
    // Tự cập nhật kích thước canvas khi bố cục Split-Screen thay đổi (tránh méo hình / render thừa)
    if (window.ResizeObserver) {
      new ResizeObserver(() => this.onWindowResize()).observe(this.container);
    }
    this.animate();
  }

  initThree() {
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x0f172a);
    this.scene.fog = new THREE.FogExp2(0x0f172a, 0.035);

    const aspect = this.container.clientWidth / this.container.clientHeight;
    this.camera = new THREE.PerspectiveCamera(46, aspect, 0.1, 100);
    
    // Default View: Front perspective matching TikTok packaging video frame!
    this.camera.position.set(-0.15, 1.45, 2.75);

    this.renderer = new THREE.WebGLRenderer({
      canvas: this.canvas,
      antialias: false, // Máy văn phòng (GPU tích hợp): tắt MSAA để giảm tải mạnh
      preserveDrawingBuffer: false,
      powerPreference: 'high-performance'
    });
    this.renderer.setSize(this.container.clientWidth, this.container.clientHeight);
    // Chế độ mặc định "Nhanh": pixelRatio 1.0, không bóng đổ (có thể bật lại bằng nút chất lượng HIGH)
    this.renderer.setPixelRatio(1.0);
    this.renderer.shadowMap.enabled = false;
    this.renderer.shadowMap.type = THREE.PCFShadowMap;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.15;

    // Full 360 Freedom OrbitControls
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.05;
    this.controls.enablePan = true;
    this.controls.screenSpacePanning = true;
    this.controls.panSpeed = 1.0;
    this.controls.zoomSpeed = 1.2;
    this.controls.rotateSpeed = 0.9;
    this.controls.maxPolarAngle = Math.PI / 2 - 0.01; // Avoid underground clipping
    this.controls.minDistance = 0.2; // Ultra close zoom allowed
    this.controls.maxDistance = 40.0; // Wide overview zoom
    this.controls.mouseButtons = {
      LEFT: THREE.MOUSE.ROTATE,
      MIDDLE: THREE.MOUSE.PAN,
      RIGHT: THREE.MOUSE.PAN
    };
    // Người dùng chạm chuột/cuộn -> hủy ngay hiệu ứng bay camera để điều khiển tự do
    this.controls.addEventListener('start', () => { this.isTransitioningCam = false; });
    this.controls.target.set(-0.15, 0.95, -0.05);
    this.controls.update();

    this.camTargetPos.copy(this.camera.position);
    this.camTargetLookAt.copy(this.controls.target);
    this.createPivotReticle();
  }

  /**
   * Tạo con trỏ mục tiêu 3D (Pivot Reticle) dạng vòng tròn phát sáng hiệu ứng radar
   * hiển thị vị trí tâm xoay mới mỗi khi người dùng đổi tâm ngắm
   */
  createPivotReticle() {
    const group = new THREE.Group();
    group.name = "PivotReticle";

    const ringGeo = new THREE.RingGeometry(0.045, 0.065, 32);
    const ringMat = new THREE.MeshBasicMaterial({
      color: 0x38bdf8,
      side: THREE.DoubleSide,
      transparent: true,
      opacity: 0.9,
      depthTest: false
    });
    const ringMesh = new THREE.Mesh(ringGeo, ringMat);
    ringMesh.rotation.x = -Math.PI / 2;
    group.add(ringMesh);

    const outerGeo = new THREE.RingGeometry(0.09, 0.105, 32);
    const outerMat = new THREE.MeshBasicMaterial({
      color: 0x22c55e,
      side: THREE.DoubleSide,
      transparent: true,
      opacity: 0.8,
      depthTest: false
    });
    const outerMesh = new THREE.Mesh(outerGeo, outerMat);
    outerMesh.rotation.x = -Math.PI / 2;
    group.add(outerMesh);

    const dotGeo = new THREE.CircleGeometry(0.016, 16);
    const dotMat = new THREE.MeshBasicMaterial({
      color: 0xffffff,
      side: THREE.DoubleSide,
      transparent: true,
      opacity: 0.95,
      depthTest: false
    });
    const dotMesh = new THREE.Mesh(dotGeo, dotMat);
    dotMesh.rotation.x = -Math.PI / 2;
    group.add(dotMesh);

    group.visible = false;
    this.scene.add(group);
    this.pivotReticle = group;
  }

  showPivotReticle(point) {
    if (!this.pivotReticle) return;
    this.pivotReticle.position.copy(point);
    this.pivotReticle.position.y += 0.012; // Cao hơn mặt sàn một chút để không bị z-fighting
    this.pivotReticle.scale.set(0.6, 0.6, 0.6);
    this.pivotReticle.visible = true;
    this.pivotReticleTime = 0;
  }

  /**
   * Tính toán điểm 3D chính xác dưới vị trí con trỏ chuột trên màn hình
   */
  getPointUnderCursor(clientX, clientY) {
    if (!this.canvas) return null;
    const rect = this.canvas.getBoundingClientRect();
    const ndcX = ((clientX - rect.left) / rect.width) * 2 - 1;
    const ndcY = -((clientY - rect.top) / rect.height) * 2 + 1;
    this.raycaster.setFromCamera(new THREE.Vector2(ndcX, ndcY), this.camera);

    const candidates = [];
    if (this.factory?.root) candidates.push(this.factory.root);
    if (this.arm6DOF?.root) candidates.push(this.arm6DOF.root);
    if (this.deltaRobot?.root) candidates.push(this.deltaRobot.root);

    const hits = this.raycaster.intersectObjects(candidates, true).filter(h => {
      if (!h.object) return false;
      // Bỏ qua tấm lưới rào bảo vệ mỏng để cho phép bắt thẳng vào các robot bên trong
      if (h.object.material === this.factory?.matWireMesh) return false;
      if (h.object.name && h.object.name.includes('WireMesh')) return false;
      return true;
    });

    if (hits.length > 0) {
      return hits[0].point;
    }

    // Nếu không trúng vật thể nào, chiếu vào mặt phẳng ảo tại tâm controls.target
    const camDir = new THREE.Vector3();
    this.camera.getWorldDirection(camDir);
    const plane = new THREE.Plane().setFromNormalAndCoplanarPoint(camDir, this.controls.target);
    const hitPoint = new THREE.Vector3();
    return this.raycaster.ray.intersectPlane(plane, hitPoint);
  }

  /**
   * Đặt tâm xoay mới cho OrbitControls kèm hiệu ứng sóng tâm ngắm
   */
  setNewOrbitCenter(point, zoomCloser = false) {
    if (!point) return;
    this.camTargetLookAt.copy(point);
    if (zoomCloser) {
      const viewDir = new THREE.Vector3().subVectors(this.camera.position, this.controls.target).normalize();
      const currentDist = this.camera.position.distanceTo(this.controls.target);
      const targetDist = Math.max(0.75, Math.min(currentDist * 0.55, 2.0));
      this.camTargetPos.copy(point).addScaledVector(viewDir, targetDist);
    } else {
      const offset = new THREE.Vector3().subVectors(this.camera.position, this.controls.target);
      this.camTargetPos.copy(point).add(offset);
    }
    this.isTransitioningCam = true;
    this.camTransitionTime = 0;
    this.showPivotReticle(point);
  }

  /**
   * Khởi tạo các sự kiện lắng nghe bàn phím và chuột cho cơ chế Free / Fly Camera kiểu Factory I/O
   */
  initFlyCameraListeners() {
    // Ngăn chặn context menu trình duyệt khi click chuột phải trên canvas 3D
    this.canvas.addEventListener('contextmenu', (e) => {
      e.preventDefault();
    });

    // Lắng nghe bàn phím điều hướng (Arrow keys, Space, C, PageUp/Down, Shift, V)
    window.addEventListener('keydown', (e) => {
      const activeTag = document.activeElement ? document.activeElement.tagName : '';
      if (activeTag === 'INPUT' || activeTag === 'TEXTAREA' || activeTag === 'SELECT') {
        return;
      }

      // Phím 'V': Bật / Tắt nhanh chế độ Factory I/O Free Cam
      if ((e.key === 'v' || e.key === 'V') && !e.ctrlKey && !e.altKey) {
        this.toggleFlyCameraMode();
        return;
      }

      // Cập nhật trạng thái phím điều hướng
      if (e.code in this.camKeys) {
        this.camKeys[e.code] = true;
        // Ngăn cuộn trang web ngoài ý muốn khi nhấn các phím mũi tên hoặc Space
        if (e.code.startsWith('Arrow') || e.code === 'Space' || e.code === 'PageUp' || e.code === 'PageDown') {
          e.preventDefault();
        }
      }
    });

    window.addEventListener('keyup', (e) => {
      if (e.code in this.camKeys) {
        this.camKeys[e.code] = false;
      }
    });

    // Reset trạng thái phím khi cửa sổ mất tiêu điểm
    window.addEventListener('blur', () => {
      for (const k in this.camKeys) {
        this.camKeys[k] = false;
      }
      this.isRightMouseDown = false;
    });

    // Lắng nghe chuột phải để xoay góc nhìn tự do tại chỗ (Factory I/O Free Look)
    this.canvas.addEventListener('pointerdown', (e) => {
      this.isTransitioningCam = false;
      if (e.button === 2) {
        this.isRightMouseDown = true;
        this.lastRightMousePos.x = e.clientX;
        this.lastRightMousePos.y = e.clientY;
      }
    });

    window.addEventListener('pointermove', (e) => {
      if (this.isRightMouseDown && this.isFlyCamActive) {
        const dx = e.clientX - this.lastRightMousePos.x;
        const dy = e.clientY - this.lastRightMousePos.y;
        this.lastRightMousePos.x = e.clientX;
        this.lastRightMousePos.y = e.clientY;
        if (dx !== 0 || dy !== 0) {
          this.rotateCameraFreeLook(dx, dy);
        }
      }
    });

    window.addEventListener('pointerup', (e) => {
      if (e.button === 2) {
        this.isRightMouseDown = false;
      }
    });
  }

  /**
   * Bật/Tắt chế độ điều khiển góc nhìn tự do kiểu Factory I/O (Free / Fly Camera)
   */
  toggleFlyCameraMode(forceState = null) {
    this.isFlyCamActive = (forceState !== null) ? forceState : !this.isFlyCamActive;
    this.isTransitioningCam = false;

    const flyBadge = document.getElementById('fly-cam-badge');
    if (flyBadge) {
      flyBadge.style.display = this.isFlyCamActive ? 'flex' : 'none';
    }

    const btnToolbarFly = document.getElementById('btn-toggle-fly-cam');
    if (btnToolbarFly) {
      btnToolbarFly.classList.toggle('active', this.isFlyCamActive);
      if (this.isFlyCamActive) {
        btnToolbarFly.style.background = 'var(--scada-yellow)';
        btnToolbarFly.style.color = '#0b0f19';
      } else {
        btnToolbarFly.style.background = '';
        btnToolbarFly.style.color = '#fef08a';
      }
    }

    const btnToolFly = document.getElementById('btn-tool-fly-cam');
    if (btnToolFly) {
      btnToolFly.classList.toggle('active', this.isFlyCamActive);
    }

    if (this.isFlyCamActive) {
      // Cho phép ngước lên nhìn toàn cảnh trần nhà máy & khung robot
      this.controls.maxPolarAngle = Math.PI - 0.05;
      // Dành chuột phải cho chế độ Free Look tự do
      this.controls.mouseButtons.RIGHT = -1;
      // Bỏ kích hoạt các nút preset góc nhìn cố định
      document.querySelectorAll('.camera-toolbar .cam-btn').forEach(btn => {
        if (btn !== btnToolbarFly) btn.classList.remove('active');
      });
    } else {
      // Trả lại OrbitControls chuẩn
      this.controls.maxPolarAngle = Math.PI / 2 - 0.01;
      this.controls.mouseButtons.RIGHT = THREE.MOUSE.PAN;
      this.camVelocity.set(0, 0, 0);
    }
  }

  /**
   * Xoay hướng nhìn tự do của camera tại vị trí hiện tại (Factory I/O Free Look)
   */
  rotateCameraFreeLook(deltaX, deltaY) {
    const rotSpeed = 0.0032;
    const offset = new THREE.Vector3().subVectors(this.controls.target, this.camera.position);

    // 1. Xoay Yaw theo phương ngang quanh trục thế giới Y
    const yawAngle = -deltaX * rotSpeed;
    offset.applyAxisAngle(new THREE.Vector3(0, 1, 0), yawAngle);

    // 2. Xoay Pitch theo phương đứng quanh trục ngang của camera (Right vector)
    const right = new THREE.Vector3().crossVectors(offset, new THREE.Vector3(0, 1, 0)).normalize();
    if (right.lengthSq() > 0.001) {
      const pitchAngle = -deltaY * rotSpeed;
      const testOffset = offset.clone().applyAxisAngle(right, pitchAngle);
      // Giới hạn góc ngước lên/cúi xuống để tránh lật ngược camera
      const upDot = testOffset.clone().normalize().y;
      if (Math.abs(upDot) < 0.985) {
        offset.copy(testOffset);
      }
    }

    this.controls.target.copy(this.camera.position).add(offset);
  }

  /**
   * Di chuyển và Pan camera mượt mà theo cụm 4 phím mũi tên / WASD với quán tính vật lý và chống va sàn
   */
  updateCameraMovement(delta) {
    const keys = this.camKeys;
    let moveForward = 0;
    let moveRight = 0;
    let moveUp = 0;

    if (keys.ArrowUp || keys.KeyW) moveForward += 1;
    if (keys.ArrowDown || keys.KeyS) moveForward -= 1;
    if (keys.ArrowRight || keys.KeyD) moveRight += 1;
    if (keys.ArrowLeft || keys.KeyA) moveRight -= 1;
    if (keys.Space || keys.PageUp) moveUp += 1;
    if (keys.KeyC || keys.PageDown) moveUp -= 1;

    const isMoving = (moveForward !== 0 || moveRight !== 0 || moveUp !== 0);

    // Nếu không bấm phím và quán tính đã tắt hẳn, thoát để tiết kiệm CPU
    if (!isMoving && this.camVelocity.lengthSq() < 0.0001) {
      this.camVelocity.set(0, 0, 0);
      return;
    }

    if (isMoving) {
      this.isTransitioningCam = false;
    }

    // Tốc độ cơ bản: 3.8 m/s; Nhấn Shift tăng tốc gấp 2.4 lần: 9.2 m/s
    const isBoost = keys.ShiftLeft || keys.ShiftRight;
    const baseSpeed = isBoost ? 9.2 : 3.8;

    // Vector hướng nhìn
    const camDir = new THREE.Vector3();
    this.camera.getWorldDirection(camDir);

    let forwardVec;
    if (this.isFlyCamActive) {
      forwardVec = camDir.clone().normalize();
    } else {
      forwardVec = new THREE.Vector3(camDir.x, 0, camDir.z);
      if (forwardVec.lengthSq() < 0.001) {
        forwardVec.set(0, 0, -1);
      } else {
        forwardVec.normalize();
      }
    }

    // Vector sang phải (vuông góc với hướng nhìn ngang và trục Y thế giới)
    const rightVec = new THREE.Vector3().crossVectors(forwardVec, new THREE.Vector3(0, 1, 0)).normalize();
    const upVec = new THREE.Vector3(0, 1, 0);

    const targetVel = new THREE.Vector3();
    if (moveForward !== 0) targetVel.addScaledVector(forwardVec, moveForward);
    if (moveRight !== 0) targetVel.addScaledVector(rightVec, moveRight);
    if (moveUp !== 0) targetVel.addScaledVector(upVec, moveUp);

    if (targetVel.lengthSq() > 0) {
      targetVel.normalize().multiplyScalar(baseSpeed);
    }

    // Gia tốc & giảm tốc mượt mà (Inertia Damping)
    const lerpRate = isMoving ? 14.0 : 10.0;
    this.camVelocity.lerp(targetVel, Math.min(1.0, lerpRate * delta));

    // Tính độ dịch chuyển trong khung hình hiện tại
    const displacement = this.camVelocity.clone().multiplyScalar(delta);

    // Đồng bộ cả tọa độ camera VÀ tâm ngắm controls.target để giữ góc nhìn ổn định tuyệt đối
    this.camera.position.add(displacement);
    this.controls.target.add(displacement);

    // Giới hạn cao độ sàn tối thiểu: không cho camera chìm xuống dưới sàn epoxy (Y >= 0.15m)
    const minFloorY = 0.15;
    if (this.camera.position.y < minFloorY) {
      const diff = minFloorY - this.camera.position.y;
      this.camera.position.y = minFloorY;
      this.controls.target.y += diff;
      this.camVelocity.y = 0;
    }
  }

  initLights() {
    const ambient = new THREE.AmbientLight(0xfff7ed, 0.9);
    this.scene.add(ambient);

    this.dirLight = new THREE.DirectionalLight(0xfffaea, 2.4);
    this.dirLight.position.set(3.5, 8.0, 4.5);
    this.dirLight.castShadow = false;
    this.dirLight.shadow.mapSize.width = 1024;
    this.dirLight.shadow.mapSize.height = 1024;
    this.dirLight.shadow.camera.near = 1.0;
    this.dirLight.shadow.camera.far = 18;
    this.dirLight.shadow.bias = -0.0003;

    const shadowDist = 5.8;
    this.dirLight.shadow.camera.left = -shadowDist;
    this.dirLight.shadow.camera.right = shadowDist;
    this.dirLight.shadow.camera.top = shadowDist;
    this.dirLight.shadow.camera.bottom = -shadowDist;
    this.scene.add(this.dirLight);

    const fillLight = new THREE.DirectionalLight(0x38bdf8, 0.75);
    fillLight.position.set(-5.0, 5.0, -3.0);
    this.scene.add(fillLight);

    const armSpot = new THREE.SpotLight(0xfffbeb, 3.8, 9.0, Math.PI / 3.5, 0.35, 1.0);
    armSpot.position.set(-1.25, 4.0, -0.2);
    armSpot.target.position.set(-1.25, 0.9, 0.1);
    this.scene.add(armSpot);
    this.scene.add(armSpot.target);

    const deltaSpot = new THREE.SpotLight(0xfffbeb, 3.8, 9.0, Math.PI / 3.5, 0.35, 1.0);
    deltaSpot.position.set(1.2, 4.0, -0.1);
    deltaSpot.target.position.set(1.2, 0.9, -0.1);
    this.scene.add(deltaSpot);
    this.scene.add(deltaSpot.target);
  }

  initSceneObjects() {
    this.factory = new FactoryEnvironment(this.scene);

    // FANUC Delta Robot on Gantry
    this.deltaRobot = new DeltaRobot3D({
      position: new THREE.Vector3(1.2, 1.42, -0.1)
    });
    this.scene.add(this.deltaRobot.root);

    // FANUC Heavy Industrial 6-DOF Arm
    this.arm6DOF = new ArticulatedArm6DOF({
      position: new THREE.Vector3(-1.25, 0, -0.75)
    });
    this.scene.add(this.arm6DOF.root);

    this.updateDeltaUIFromModel();
    this.updateArmUIFromModel();
  }

  initCameraPresets() {
    this.presets = {
      overview: {
        pos: new THREE.Vector3(0, 3.6, 4.8),
        lookAt: new THREE.Vector3(0, 0.9, 0)
      },
      front: {
        pos: new THREE.Vector3(-0.15, 1.62, 2.65),
        lookAt: new THREE.Vector3(-0.15, 0.92, -0.10)
      },
      delta: {
        pos: new THREE.Vector3(1.45, 1.68, 1.35),
        lookAt: new THREE.Vector3(1.20, 0.96, -0.08)
      },
      arm: {
        pos: new THREE.Vector3(-1.30, 1.72, 1.55),
        lookAt: new THREE.Vector3(-1.25, 0.85, -0.25)
      },
      topdown: {
        pos: new THREE.Vector3(0, 7.8, 0.01),
        lookAt: new THREE.Vector3(0, 0, 0)
      },
      cell: {
        pos: new THREE.Vector3(-1.0, 2.05, 2.25),
        lookAt: new THREE.Vector3(-1.25, 0.88, -0.15)
      },
      agv: {
        pos: new THREE.Vector3(-6.4, 3.2, 3.8),
        lookAt: new THREE.Vector3(-6.8, 0.85, -1.0)
      },
      cabinet: {
        pos: new THREE.Vector3(0.55, 1.38, -0.32),
        lookAt: new THREE.Vector3(0.55, 1.12, -2.28)
      }
    };
  }

  /**
   * Synchronizes the Main Industrial Control Cabinet UI & 3D Rotary Isolator / Controller LEDs
   */
  syncControlCabinetUI(isBooting = false) {
    if (!this.factory) return;
    this.factory.setCabinetMainPower(this.systemPower, isBooting);

    const btnTop = document.getElementById('btn-quick-cabinet-power');
    const ledTop = document.getElementById('led-cabinet-power-top');
    const lblTop = document.getElementById('label-cabinet-power-top');
    const statusBadge = document.getElementById('status-cabinet-power');
    const j1000FreqEl = document.getElementById('ui-cabinet-j1000-freq');
    const plcLedEl = document.getElementById('ui-cabinet-plc-led');

    if (isBooting) {
      if (btnTop) btnTop.style.borderColor = '#f59e0b';
      if (ledTop) { ledTop.className = 'scada-led led-yellow active'; }
      if (lblTop) lblTop.textContent = 'NGUỒN TỦ: BOOTING...';
      if (statusBadge) {
        statusBadge.textContent = 'BOOTING (1.5s)';
        statusBadge.style.background = 'rgba(245, 158, 11, 0.25)';
        statusBadge.style.color = '#facc15';
      }
      if (j1000FreqEl) { j1000FreqEl.textContent = 'INIT...'; j1000FreqEl.style.color = '#f59e0b'; }
      if (plcLedEl) { plcLedEl.textContent = 'BOOTING...'; plcLedEl.style.color = '#f59e0b'; }
    } else if (this.systemPower) {
      if (btnTop) btnTop.style.borderColor = '#22c55e';
      if (ledTop) { ledTop.className = 'scada-led led-green active'; }
      if (lblTop) lblTop.textContent = 'NGUỒN TỦ: 380V ON';
      if (statusBadge) {
        statusBadge.textContent = '380V ONLINE (RUN)';
        statusBadge.style.background = 'rgba(34, 197, 94, 0.22)';
        statusBadge.style.color = '#4ade80';
      }
      const freqVal = (50.0 * (this.factory.infeedRateMultiplier || 1.0)).toFixed(1);
      if (j1000FreqEl) { j1000FreqEl.textContent = `${freqVal} Hz`; j1000FreqEl.style.color = '#ef4444'; }
      if (plcLedEl) { plcLedEl.textContent = 'PWR/RUN [OK]'; plcLedEl.style.color = '#22c55e'; }
    } else {
      if (btnTop) btnTop.style.borderColor = '#ef4444';
      if (ledTop) { ledTop.className = 'scada-led led-red'; }
      if (lblTop) lblTop.textContent = 'NGUỒN TỦ: OFF';
      if (statusBadge) {
        statusBadge.textContent = 'ISOLATED (OFF)';
        statusBadge.style.background = 'rgba(239, 68, 68, 0.2)';
        statusBadge.style.color = '#f87171';
      }
      if (j1000FreqEl) { j1000FreqEl.textContent = '0.0 Hz (OFF)'; j1000FreqEl.style.color = '#64748b'; }
      if (plcLedEl) { plcLedEl.textContent = 'OFFLINE'; plcLedEl.style.color = '#94a3b8'; }
    }
  }

  /**
   * Toggles the Cabinet Shell Cutaway / Open Door Inspection Mode
   */
  toggleCabinetInspectionShell(forceState = null) {
    if (!this.factory) return;
    const isCutaway = this.factory.toggleCabinetEnclosureVisibility(forceState);
    const btnDock = document.getElementById('btn-toggle-cabinet-shell');
    const btnBar = document.getElementById('btn-toggle-cabinet-shell-bar');
    if (btnDock) {
      btnDock.textContent = isCutaway ? '🔒 ĐÓNG VỎ TỦ ĐIỆN' : '🔍 ẨN/HIỆN VỎ TỦ';
      btnDock.style.background = isCutaway ? 'rgba(245, 158, 11, 0.25)' : '#1e293b';
    }
    if (btnBar) {
      btnBar.textContent = isCutaway ? '🔒 Đóng Vỏ Tủ' : '🔍 Ẩn/Hiện Vỏ Tủ';
      btnBar.classList.toggle('active', isCutaway);
    }
  }

  switchCamera(presetKey) {
    const p = this.presets[presetKey];
    if (!p) return;

    this.toggleFlyCameraMode(false);
    this.camTargetPos.copy(p.pos);
    this.camTargetLookAt.copy(p.lookAt);
    this.isTransitioningCam = true;
    this.camTransitionTime = 0;

    // On-Demand 3D Detail Activation:
    // - Khi người dùng bấm sang góc nhìn Tủ Điện ('cabinet'), tự động mở cửa tủ & bật hiển thị chi tiết bên trong (MP3300, J1000, Sigma-7, 18 đèn LED).
    // - Khi người dùng chuyển sang các góc nhìn tổng thể khác ('overview', 'front', 'delta', 'arm', 'top', 'agv'), tự động đóng kín vỏ tủ để ngừng vẽ 45+ chi tiết bên trong, giúp giải phóng CPU/GPU.
    if (this.factory) {
      if (presetKey === 'cabinet' && !this.factory.cabinetCutawayMode) {
        this.toggleCabinetInspectionShell(true);
      } else if (presetKey !== 'cabinet' && this.factory.cabinetCutawayMode) {
        this.toggleCabinetInspectionShell(false);
      }
    }

    document.querySelectorAll('.cam-btn').forEach(btn => {
      btn.classList.remove('active');
      if (btn.dataset.cam === presetKey) {
        btn.classList.add('active');
      }
    });
  }

  initUIEventListeners() {
    // 1. Camera Toolbar buttons
    document.querySelectorAll('.cam-btn').forEach(btn => {
      btn.addEventListener('click', () => {
        if (btn.dataset.cam) {
          this.switchCamera(btn.dataset.cam);
        }
      });
    });

    // Factory I/O Fly Camera toggle buttons
    const btnToggleFly = document.getElementById('btn-toggle-fly-cam');
    if (btnToggleFly) {
      btnToggleFly.addEventListener('click', () => {
        this.toggleFlyCameraMode();
      });
    }

    const btnToolFly = document.getElementById('btn-tool-fly-cam');
    if (btnToolFly) {
      btnToolFly.addEventListener('click', () => {
        this.toggleFlyCameraMode();
      });
    }

    // Nút ẩn/hiện nhanh lưới hàng rào bảo vệ để soi rõ bên trong cell
    const btnToggleFence = document.getElementById('btn-toggle-fence-mesh');
    if (btnToggleFence) {
      btnToggleFence.addEventListener('click', () => {
        if (this.factory && this.factory.enclosureGroup) {
          this.factory.enclosureGroup.visible = !this.factory.enclosureGroup.visible;
          const vis = this.factory.enclosureGroup.visible;
          btnToggleFence.textContent = vis ? '🕸 Ẩn/Hiện Lưới Rào' : '🕸 Hiện Lại Lưới Rào';
          btnToggleFence.classList.toggle('active', !vis);
        }
      });
    }

    // 2. Interactive Zoom & Camera Gizmo buttons
    const btnZoomIn = document.getElementById('btn-zoom-in');
    if (btnZoomIn) {
      btnZoomIn.addEventListener('click', () => {
        this.isTransitioningCam = false;
        const dir = new THREE.Vector3().subVectors(this.controls.target, this.camera.position).normalize();
        this.camera.position.addScaledVector(dir, 0.5);
        this.controls.update();
      });
    }

    const btnZoomOut = document.getElementById('btn-zoom-out');
    if (btnZoomOut) {
      btnZoomOut.addEventListener('click', () => {
        this.isTransitioningCam = false;
        const dir = new THREE.Vector3().subVectors(this.camera.position, this.controls.target).normalize();
        this.camera.position.addScaledVector(dir, 0.5);
        this.controls.update();
      });
    }

    const btnResetCam = document.getElementById('btn-reset-cam');
    if (btnResetCam) {
      btnResetCam.addEventListener('click', () => {
        this.switchCamera('front');
      });
    }

    const btnAutoRotate = document.getElementById('btn-auto-rotate');
    if (btnAutoRotate) {
      btnAutoRotate.addEventListener('click', () => {
        this.controls.autoRotate = !this.controls.autoRotate;
        this.controls.autoRotateSpeed = 1.2;
        btnAutoRotate.classList.toggle('active', this.controls.autoRotate);
      });
    }

    // Thu phóng tự do theo con trỏ chuột (CAD / Blender Style Zoom-to-Cursor)
    this.canvas.addEventListener('wheel', (e) => {
      this.isTransitioningCam = false;
      if (e.deltaY < 0) {
        // Cuộn tiến (Zoom in): Lia tâm xoay controls.target dịch dần về phía điểm 3D dưới chuột
        const point = this.getPointUnderCursor(e.clientX, e.clientY);
        if (point) {
          const factor = Math.min(0.22, Math.max(0.06, Math.abs(e.deltaY) * 0.0012));
          this.controls.target.lerp(point, factor);
          this.camera.position.addScaledVector(point.clone().sub(this.camera.position), factor * 0.4);
        }
      }
    }, { passive: true });
    this.canvas.addEventListener('pointerdown', () => { this.isTransitioningCam = false; }, { passive: true });

    // 3. Double-Click vào bất kỳ điểm nào trên 3D để đổi tâm xoay & phóng lại gần điểm đó
    this.canvas.addEventListener('dblclick', (e) => {
      const point = this.getPointUnderCursor(e.clientX, e.clientY);
      if (point) {
        this.setNewOrbitCenter(point, true);
      }
    });

    // Nút [🎯 Đặt Tâm Xoay] trên thanh công cụ camera dưới cùng
    const btnPickPivot = document.getElementById('btn-pick-pivot');
    let isPickingPivot = false;
    if (btnPickPivot) {
      btnPickPivot.addEventListener('click', () => {
        isPickingPivot = !isPickingPivot;
        btnPickPivot.classList.toggle('active', isPickingPivot);
        btnPickPivot.style.background = isPickingPivot ? 'rgba(56, 189, 248, 0.3)' : 'transparent';
        btnPickPivot.style.color = isPickingPivot ? '#fff' : '#7dd3fc';
        this.canvas.style.cursor = isPickingPivot ? 'crosshair' : 'default';
      });
    }

    // Theo dõi tọa độ chuột trên canvas để phục vụ phím tắt F
    this.lastMousePos = { x: window.innerWidth / 2, y: window.innerHeight / 2 };
    this.canvas.addEventListener('pointermove', (e) => {
      this.lastMousePos.x = e.clientX;
      this.lastMousePos.y = e.clientY;
    });

    // Phím tắt F (Focus): Đổi tâm xoay vào điểm đang chỉ chuột
    window.addEventListener('keydown', (e) => {
      if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA') return;
      if (e.key === 'f' || e.key === 'F') {
        const point = this.getPointUnderCursor(this.lastMousePos.x, this.lastMousePos.y);
        if (point) {
          this.setNewOrbitCenter(point, true);
        }
      }
    });

    // 3b. Interactive 3D Safety Door Click Interaction
    let pointerDownPos = new THREE.Vector2();
    this.canvas.addEventListener('pointerdown', (e) => {
      pointerDownPos.set(e.clientX, e.clientY);
    });
    this.canvas.addEventListener('pointerup', (e) => {
      if (isPickingPivot) {
        const point = this.getPointUnderCursor(e.clientX, e.clientY);
        if (point) {
          this.setNewOrbitCenter(point, false);
          isPickingPivot = false;
          if (btnPickPivot) {
            btnPickPivot.classList.remove('active');
            btnPickPivot.style.background = 'transparent';
            btnPickPivot.style.color = '#7dd3fc';
          }
          this.canvas.style.cursor = 'default';
          return;
        }
      }

      const dist = pointerDownPos.distanceTo(new THREE.Vector2(e.clientX, e.clientY));
      if (dist > 6) return; // Drag / orbit navigation, ignore

      const rect = this.canvas.getBoundingClientRect();
      this.mouse.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
      this.mouse.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;

      this.raycaster.setFromCamera(this.mouse, this.camera);
      if (this.factory && (this.factory.doorGroup || this.factory.interlockSwitchGroup)) {
        const doorTargets = [
          this.factory.doorHandle,
          this.factory.interlockTongue,
          this.factory.interlockSwitchGroup
        ].filter(Boolean);
        const hits = this.raycaster.intersectObjects(
          doorTargets.length > 0 ? doorTargets : [this.factory.doorGroup],
          true
        );
        if (hits.length > 0) {
          this.toggleSafetyDoor();
          return;
        }
      }
      // 3b-2. Click on 3D Main Rotary Isolator Switch on the Control Cabinet to toggle Main Power!
      if (this.factory && this.factory.mainIsolatorGroup) {
        const isoHits = this.raycaster.intersectObjects([this.factory.mainIsolatorGroup], true);
        if (isoHits.length > 0) {
          const btnPwr = document.getElementById('btn-system-power');
          if (btnPwr) btnPwr.click();
          return;
        }
      }
      // 3b-3. Click on Control Cabinet Doors to toggle Open/Cutaway Inspection View
      if (this.factory && this.factory.controlCabinetGroup) {
        const cabHits = this.raycaster.intersectObjects([
          this.factory.cabinetLeftDoorPivot,
          this.factory.cabinetRightDoorPivot
        ].filter(Boolean), true);
        if (cabHits.length > 0) {
          this.toggleCabinetInspectionShell();
        }
      }
    });

    // 3c. Maintenance Safety Door Controls & Interlock Reset Buttons
    const btnQuickDoor = document.getElementById('btn-quick-safety-door');
    if (btnQuickDoor) {
      btnQuickDoor.addEventListener('click', () => this.toggleSafetyDoor());
    }

    // 3d. Main Industrial Control Cabinet Power & Cutaway Inspection Controls
    const btnQuickCabPower = document.getElementById('btn-quick-cabinet-power');
    if (btnQuickCabPower) {
      btnQuickCabPower.addEventListener('click', () => {
        const btnPwr = document.getElementById('btn-system-power');
        if (btnPwr) btnPwr.click();
      });
    }

    const btnViewCabDock = document.getElementById('btn-view-cabinet-dock');
    if (btnViewCabDock) {
      btnViewCabDock.addEventListener('click', () => {
        this.switchCamera('cabinet');
      });
    }

    const btnToggleCabShell = document.getElementById('btn-toggle-cabinet-shell');
    if (btnToggleCabShell) {
      btnToggleCabShell.addEventListener('click', () => {
        this.toggleCabinetInspectionShell();
      });
    }

    const btnToggleCabShellBar = document.getElementById('btn-toggle-cabinet-shell-bar');
    if (btnToggleCabShellBar) {
      btnToggleCabShellBar.addEventListener('click', () => {
        this.toggleCabinetInspectionShell();
      });
    }

    const btnToggleDoor = document.getElementById('btn-toggle-safety-door');
    if (btnToggleDoor) {
      btnToggleDoor.addEventListener('click', () => this.toggleSafetyDoor());
    }

    const btnResetInterlock = document.getElementById('btn-safety-reset-interlock');
    if (btnResetInterlock) {
      btnResetInterlock.addEventListener('click', () => this.resetSafetyInterlock());
    }

    const btnModalCloseDoor = document.getElementById('btn-modal-close-door');
    if (btnModalCloseDoor) {
      btnModalCloseDoor.addEventListener('click', () => this.toggleSafetyDoor(true));
    }

    const btnModalResetInterlock = document.getElementById('btn-modal-reset-interlock');
    if (btnModalResetInterlock) {
      btnModalResetInterlock.addEventListener('click', () => this.resetSafetyInterlock());
    }

    // 4. Anti-Lag Graphics Performance Selector
    document.querySelectorAll('.q-btn').forEach(btn => {
      btn.addEventListener('click', () => {
        document.querySelectorAll('.q-btn').forEach(b => b.classList.remove('active'));
        btn.classList.add('active');

        const quality = btn.dataset.q;
        if (quality === 'high') {
          this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 1.5));
          this.renderer.shadowMap.enabled = true;
          this.dirLight.castShadow = true;
        } else if (quality === 'balanced') {
          this.renderer.setPixelRatio(1.0);
          this.renderer.shadowMap.enabled = true;
          this.dirLight.castShadow = true;
        } else if (quality === 'fast') {
          this.renderer.setPixelRatio(1.0);
          this.renderer.shadowMap.enabled = false;
          this.dirLight.castShadow = false;
    this.dirLight.shadow.mapSize.width = 1024;
        }
      });
    });

    // 5. Auto Demo Cycle Button
    const btnAutoDemo = document.getElementById('btn-auto-demo-cycle');
    const ledAutoDemo = document.getElementById('led-auto-demo');
    const labelAutoDemo = document.getElementById('label-auto-demo');

    if (btnAutoDemo) {
      btnAutoDemo.addEventListener('click', () => {
        if (!this.autoDemoMode && this.isSafetyInterlockTripped) {
          alert("⚠ CẢNH BÁO AN TOÀN LAO ĐỘNG (SAFETY INTERLOCK):\nCửa bảo trì đang mở hoặc mạch an toàn chưa được Reset!\nHãy đóng cửa rào và nhấn nút [RESET FAULT / ROBOT ON] để tiếp tục.");
          return;
        }

        this.autoDemoMode = !this.autoDemoMode;
        
        if (this.autoDemoMode) {
          // Auto turn on system power & conveyors
          if (!this.systemPower) {
            document.getElementById('btn-system-power').click();
          }
          this.factory.conveyor1Active = true;
          this.factory.conveyor2Active = true;
          this.factory.conveyor3Active = true;
          document.getElementById('toggle-conv1').checked = true;
          document.getElementById('toggle-conv2').checked = true;
          document.getElementById('toggle-conv3').checked = true;
          document.getElementById('led-conv1').classList.add('active');
          document.getElementById('led-conv2').classList.add('active');
          document.getElementById('led-conv3').classList.add('active');

          btnAutoDemo.classList.add('running');
          ledAutoDemo.classList.add('active');
          labelAutoDemo.textContent = "AUTO RUNNING (60 FPS)";
          if (this.oeeCalculator) {
            this.oeeCalculator.logEvent('RUN', 'SYSTEM RUN: Auto packaging started.');
          }
        } else {
          btnAutoDemo.classList.remove('running');
          ledAutoDemo.classList.remove('active');
          labelAutoDemo.textContent = "AUTO DEMO CYCLE";
          if (this.oeeCalculator) {
            this.oeeCalculator.logEvent('INFO', 'SYSTEM STANDBY: Auto packaging paused.');
          }
        }
      });
    }

    // 6. Master System Power Button
    const btnPower = document.getElementById('btn-system-power');
    const ledPower = document.getElementById('led-power-status');
    const powerLabel = document.getElementById('power-status-label');
    const dockStatus = document.getElementById('dock-system-tag');
    const lineStateEl = document.getElementById('telemetry-line-state');

    btnPower.addEventListener('click', () => {
      if (!this.systemPower && this.isSafetyInterlockTripped) {
        if (!this.factory.safetyGateClosed) {
          alert("⚠ SAFETY INTERLOCK TRIPPED:\nCửa bảo trì hàng rào an toàn đang mở!\nVui lòng đóng kín cửa và nhấn nút [RESET FAULT / ROBOT ON] để cấp nguồn.");
          return;
        } else {
          this.resetSafetyInterlock();
          return;
        }
      }

      this.systemPower = !this.systemPower;

      if (this.systemPower) {
        btnPower.classList.add('active');
        ledPower.classList.add('active');
        powerLabel.textContent = "BOOTING CABINET (1.5s)...";
        dockStatus.textContent = "BOOTING";
        dockStatus.classList.add('online');
        lineStateEl.textContent = "CABINET BOOT (1.5s)";
        lineStateEl.style.color = "#facc15";

        // Start 1.5s Industrial Control Cabinet Boot Sequence (MP3300, YRC1000micro, J1000, Servo Drives)
        this.syncControlCabinetUI(true);
        if (this._cabinetBootTimeout) clearTimeout(this._cabinetBootTimeout);
        this._cabinetBootTimeout = setTimeout(() => {
          if (!this.systemPower) return;
          powerLabel.textContent = "SERVO ON // READY";
          dockStatus.textContent = "ONLINE";
          lineStateEl.textContent = "READY / RUN";
          lineStateEl.style.color = "#22c55e";
          this.syncControlCabinetUI(false);
        }, 1500);

        document.querySelectorAll('.conveyor-toggle-input').forEach(input => {
          input.disabled = false;
        });
      } else {
        if (this._cabinetBootTimeout) clearTimeout(this._cabinetBootTimeout);
        this.syncControlCabinetUI(false);
        btnPower.classList.remove('active');
        ledPower.classList.remove('active');
        powerLabel.textContent = "POWER OFF";
        dockStatus.textContent = "OFFLINE";
        dockStatus.classList.remove('online');
        lineStateEl.textContent = "OFFLINE";
        lineStateEl.style.color = "#94a3b8";

        this.autoDemoMode = false;
        if (btnAutoDemo) {
          btnAutoDemo.classList.remove('running');
          ledAutoDemo.classList.remove('active');
          labelAutoDemo.textContent = "AUTO DEMO CYCLE";
        }

        this.factory.conveyor1Active = false;
        this.factory.conveyor2Active = false;
        this.factory.conveyor3Active = false;

        if (this.hardwareBus) {
          this.hardwareBus.setDO(0, 0);
          this.hardwareBus.setDO(1, 0);
          this.hardwareBus.setDO(3, 0);
        }

        if (this.trajectoryPlanner) {
          this.trajectoryPlanner.stopAll();
        }
        this.isDeltaBusy = false;
        this.isArmBusy = false;
        this.isSingleCycleRunning = false;

        document.getElementById('toggle-conv1').checked = false;
        document.getElementById('toggle-conv2').checked = false;
        document.getElementById('toggle-conv3').checked = false;
        document.getElementById('led-conv1').classList.remove('active');
        document.getElementById('led-conv2').classList.remove('active');
        document.getElementById('led-conv3').classList.remove('active');

        document.querySelectorAll('.conveyor-toggle-input').forEach(input => {
          input.disabled = true;
        });
      }
    });

    // 7. Conveyor Toggles
    const setupConveyorToggle = (toggleId, ledId, onToggle) => {
      const toggle = document.getElementById(toggleId);
      const led = document.getElementById(ledId);
      toggle.addEventListener('change', () => {
        if (!this.systemPower) {
          toggle.checked = false;
          return;
        }
        if (toggle.checked) {
          led.classList.add('active');
          onToggle(true);
        } else {
          led.classList.remove('active');
          onToggle(false);
        }
      });
    };

    setupConveyorToggle('toggle-conv1', 'led-conv1', (active) => {
      this.factory.conveyor1Active = active;
      if (this.hardwareBus) {
        this.hardwareBus.setDO(0, active ? 1 : 0);
      }
    });

    setupConveyorToggle('toggle-conv2', 'led-conv2', (active) => {
      this.factory.conveyor2Active = active;
      if (this.hardwareBus) {
        this.hardwareBus.setDO(0, active ? 1 : 0);
      }
    });

    setupConveyorToggle('toggle-conv3', 'led-conv3', (active) => {
      this.factory.conveyor3Active = active;
      if (this.hardwareBus) {
        this.hardwareBus.setDO(3, active ? 1 : 0);
      }
    });

    // Chocolate Infeed Speed / Rate Slider & Preset Buttons
    const sliderChocoRate = document.getElementById('slider-choco-feed-speed');
    const valChocoRate = document.getElementById('val-choco-feed-speed');
    const rateBtns = document.querySelectorAll('.choco-rate-btn');

    const applyChocoRate = (rateVal) => {
      const rate = parseFloat(rateVal) || 1.0;
      this.factory.setInfeedRate(rate);
      if (sliderChocoRate) sliderChocoRate.value = rate.toFixed(1);
      if (valChocoRate) {
        valChocoRate.textContent = `${rate.toFixed(1)}x (${this.factory.speedConveyor1.toFixed(2)} m/s)`;
      }
      rateBtns.forEach(btn => {
        const isMatch = Math.abs(parseFloat(btn.dataset.rate) - rate) < 0.08;
        btn.classList.toggle('active', isMatch);
        btn.style.borderColor = isMatch ? 'var(--scada-cyan)' : '';
        btn.style.color = isMatch ? 'var(--scada-cyan)' : '';
      });
      this.syncControlCabinetUI(false);
    };

    if (sliderChocoRate) {
      sliderChocoRate.addEventListener('input', (e) => {
        applyChocoRate(e.target.value);
      });
    }

    rateBtns.forEach(btn => {
      btn.addEventListener('click', () => {
        applyChocoRate(btn.dataset.rate);
      });
    });

    // Spawner buttons
    document.getElementById('btn-spawn-product').addEventListener('click', () => {
      this.factory.spawnProduct(3.8);
    });

    document.getElementById('btn-spawn-box').addEventListener('click', () => {
      this.factory.spawnEmptyBox(1.9);
    });

    document.getElementById('btn-spawn-pack').addEventListener('click', () => {
      this.factory.spawnCollatedPack(1.6);
    });

    document.getElementById('btn-clear-objects').addEventListener('click', () => {
      if (this.physicsEngine) {
        this.physicsEngine.releaseDeltaGripper();
        this.physicsEngine.releaseArmGripper();
        this.physicsEngine.clearFallingObjects();
        if (this.physicsEngine.scannedProductIds) {
          this.physicsEngine.scannedProductIds.clear();
        }
        this.physicsEngine.visionQueue = [];
      }
      this.factory.clearAllItems();
    });

    // Quick Test Pick & Place Cycle Button in Jogger Tab
    const btnQuickCycle = document.getElementById('btn-quick-run-cycle');
    if (btnQuickCycle) {
      btnQuickCycle.addEventListener('click', () => {
        if (!this.systemPower) {
          document.getElementById('btn-system-power').click();
        }
        this.runSinglePickAndPlaceCycle();
      });
    }

    // Defect Workpiece Spawner Button & Keyboard Shortcut [D] (Spawns a bright red NG defect piece at the start of Conveyor 1)
    const triggerSpawnDefectPiece = () => {
      if (!this.factory) return;
      this.factory.spawnDefectPieceAtStart();

      const btnInjectDefect = document.getElementById('btn-inject-defect');
      const btnQuickSpawnDefect = document.getElementById('btn-quick-spawn-defect');
      if (btnInjectDefect) btnInjectDefect.classList.add('active-queued');
      if (btnQuickSpawnDefect) btnQuickSpawnDefect.style.transform = 'scale(0.95)';

      // Ensure Conveyor 1 is running so the spawned defect piece immediately travels through the Cognex Vision Camera & Reject Pusher!
      if (!this.factory.conveyor1Active && !this.isSafetyInterlockTripped) {
        if (!this.systemPower) {
          const btnPwr = document.getElementById('btn-system-power');
          if (btnPwr) btnPwr.click();
        }
        this.factory.conveyor1Active = true;
        const tog1 = document.getElementById('toggle-conv1');
        const led1 = document.getElementById('led-conv1');
        if (tog1) tog1.checked = true;
        if (led1) led1.classList.add('active');
        if (this.hardwareBus) this.hardwareBus.setDO(0, 1);
      }

      setTimeout(() => {
        if (btnInjectDefect) btnInjectDefect.classList.remove('active-queued');
        if (btnQuickSpawnDefect) btnQuickSpawnDefect.style.transform = '';
      }, 600);
    };

    const btnInjectDefect = document.getElementById('btn-inject-defect');
    if (btnInjectDefect) {
      btnInjectDefect.addEventListener('click', triggerSpawnDefectPiece);
    }

    const btnQuickSpawnDefect = document.getElementById('btn-quick-spawn-defect');
    if (btnQuickSpawnDefect) {
      btnQuickSpawnDefect.addEventListener('click', triggerSpawnDefectPiece);
    }

    window.addEventListener('keydown', (e) => {
      if (e.target && (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA' || e.target.tagName === 'SELECT' || e.target.isContentEditable)) {
        return;
      }
      if (e.key === 'd' || e.key === 'D') {
        e.preventDefault();
        triggerSpawnDefectPiece();
      }
    });

    // 8. 6-DOF Robot Arm Sliders
    const joints = ['s', 'l', 'u', 'r', 'b', 't'];
    joints.forEach(j => {
      const slider = document.getElementById(`slider-arm-${j}`);
      if (slider) {
        slider.addEventListener('input', () => {
          this.autoDemoMode = false; // Disable auto demo when manually jogging
          if (btnAutoDemo) {
            btnAutoDemo.classList.remove('running');
            ledAutoDemo.classList.remove('active');
            labelAutoDemo.textContent = "AUTO DEMO CYCLE";
          }
          const val = parseFloat(slider.value);
          this.arm6DOF.angles[j] = val;
          this.arm6DOF.updateJoints();
          this.updateArmUIFromModel();
        });
      }
    });

    const chkDH = document.getElementById('chk-show-dh');
    if (chkDH) {
      chkDH.addEventListener('change', () => {
        this.arm6DOF.setDHVisible(chkDH.checked);
      });
    }

    const chkArmTrail = document.getElementById('chk-show-arm-trail');
    if (chkArmTrail) {
      chkArmTrail.addEventListener('change', () => {
        if (this.trajectoryTrail) {
          this.trajectoryTrail.setArmVisible(chkArmTrail.checked);
          this.syncTrailToolbarState();
        }
      });
    }

    const chkDeltaCoords = document.getElementById('chk-show-delta-coords');
    if (chkDeltaCoords) {
      chkDeltaCoords.addEventListener('change', () => {
        if (this.deltaRobot && this.deltaRobot.setCoordsVisible) {
          this.deltaRobot.setCoordsVisible(chkDeltaCoords.checked);
        }
      });
    }

    const chkDeltaTrail = document.getElementById('chk-show-delta-trail');
    if (chkDeltaTrail) {
      chkDeltaTrail.addEventListener('change', () => {
        if (this.trajectoryTrail) {
          this.trajectoryTrail.setDeltaVisible(chkDeltaTrail.checked);
          this.syncTrailToolbarState();
        }
      });
    }

    document.getElementById('btn-arm-home').addEventListener('click', () => {
      this.arm6DOF.setJointAngles(-75.0, -18.0, -10.0, 0.0, -62.0, 0.0);
      this.updateArmUIFromModel();
    });

    document.getElementById('btn-arm-pick').addEventListener('click', () => {
      const ik = this.armKinematics.inverseKinematics({ x: -1.25, y: 0.91, z: 0.18, rx: 0, ry: 0, rz: 0 }, this.arm6DOF.getJointAngles(), true);
      if (ik.success) {
        const q = ik.jointsDeg;
        this.arm6DOF.setJointAngles(q.s, q.l, q.u, q.r, q.b, q.t);
      }
      this.updateArmUIFromModel();
    });

    document.getElementById('btn-arm-place').addEventListener('click', () => {
      const ik = this.armKinematics.inverseKinematics({ x: -1.25, y: 0.80, z: -1.45, rx: 0, ry: 0, rz: 0 }, this.arm6DOF.getJointAngles(), true);
      if (ik.success) {
        const q = ik.jointsDeg;
        this.arm6DOF.setJointAngles(q.s, q.l, q.u, q.r, q.b, q.t);
      }
      this.updateArmUIFromModel();
    });

    // 8a-2. Quick Run Letter T Buttons
    // 8b. Yaskawa Virtual Teach Pendant Launch & Dock Controls
    const btnQuickFloat = document.getElementById('btn-quick-float-pendant');
    if (btnQuickFloat) {
      btnQuickFloat.addEventListener('click', () => {
        if (this.teachPendant) {
          this.teachPendant.toggleFloating();
        }
      });
    }

    const btnOpenFloat = document.getElementById('btn-open-float-pendant');
    if (btnOpenFloat) {
      btnOpenFloat.addEventListener('click', () => {
        if (this.teachPendant) {
          this.teachPendant.setDockMode('floating');
          this.teachPendant.show();
        }
      });
    }

    const btnDockPendant = document.getElementById('btn-dock-pendant-here');
    if (btnDockPendant) {
      btnDockPendant.addEventListener('click', () => {
        if (this.teachPendant) {
          const mount = document.getElementById('dock-teach-pendant-mount');
          if (mount) {
            if (this.teachPendant.dockMode === 'docked' && this.teachPendant.container === mount && mount.style.display !== 'none') {
              this.teachPendant.hide();
            } else {
              mount.style.display = 'block';
              this.teachPendant.initDOM(mount);
              this.teachPendant.setDockMode('docked');
            }
          }
        }
      });
    }

    // 9. Delta Robot Sliders
    ['m1', 'm2', 'm3', 'yaw'].forEach(m => {
      const slider = document.getElementById(`slider-delta-${m}`);
      if (slider) {
        slider.addEventListener('input', () => {
          this.autoDemoMode = false;
          if (btnAutoDemo) {
            btnAutoDemo.classList.remove('running');
            ledAutoDemo.classList.remove('active');
            labelAutoDemo.textContent = "AUTO DEMO CYCLE";
          }
          const m1 = parseFloat(document.getElementById('slider-delta-m1').value);
          const m2 = parseFloat(document.getElementById('slider-delta-m2').value);
          const m3 = parseFloat(document.getElementById('slider-delta-m3').value);
          const yaw = parseFloat(document.getElementById('slider-delta-yaw').value);

          this.deltaRobot.setJointAngles(m1, m2, m3, yaw);
          this.updateDeltaUIFromModel();
        });
      }
    });

    document.getElementById('btn-delta-home').addEventListener('click', () => {
      this.deltaRobot.setDeltaCartesian(0, 0, -380, 0);
      this.updateDeltaUIFromModel();
    });

    document.getElementById('btn-delta-pick1').addEventListener('click', () => {
      this.deltaRobot.setDeltaCartesian(0, -120, -408, 15);
      this.updateDeltaUIFromModel();
    });

    document.getElementById('btn-delta-collate').addEventListener('click', () => {
      this.deltaRobot.setDeltaCartesian(120, 100, -408, 0);
      this.updateDeltaUIFromModel();
    });
  }

  updateAutoDemoCycle(delta) {
    if (!this.autoDemoMode || !this.systemPower) return;

    const statusEl = document.getElementById('cycle-status-text');
    const lineStateEl = document.getElementById('telemetry-line-state');

    // Independent Parallel Multi-Robot Task Dispatch:
    // 1. Robot Delta 4-DOF works continuously whenever a chocolate bar enters its reach zone (X around 1.20m) on Conveyor 1!
    if (!this.isDeltaBusy) {
      const reachableBar = this.physicsEngine.findReachableInfeedProduct(1.2, 0.42);
      if (reachableBar) {
        this.runDeltaCollateStep(reachableBar);
      }
    }

    // 2. Robot Arm 6-DOF works simultaneously whenever a completed 4-pack arrives at Sensor DI_11 (X = -1.25m)!
    if (!this.isArmBusy) {
      const readyPack = this.physicsEngine.findReachableCollatedPack(-1.25, 0.22);
      if (readyPack) {
        this.runArmBoxPackStep(readyPack);
      }
    }

    // 3. Real-time waiting telemetry when incoming product rate is slow
    if (!this.isDeltaBusy && !this.isArmBusy) {
      let buildingPack = this.factory.collatedPacks.find(p => !p.complete && !p.picked);
      const curBars = buildingPack ? buildingPack.barCount : 0;
      if (statusEl) {
        statusEl.textContent = `[CHỜ HÀNG] DELTA ĐANG ĐỢI SÔ-CÔ-LA (${curBars}/4) | ARM ĐỢI CỤM TẠI DI_11`;
      }
      if (lineStateEl) {
        lineStateEl.textContent = `WAITING PRODUCT (${curBars}/4)`;
        lineStateEl.style.color = "#38bdf8";
      }
    }
  }

  updateArmUIFromModel() {
    const angles = this.arm6DOF.getJointAngles();
    const pulses = this.arm6DOF.getJointPulses();

    ['s', 'l', 'u', 'r', 'b', 't'].forEach(j => {
      const slider = document.getElementById(`slider-arm-${j}`);
      const valDeg = document.getElementById(`val-arm-${j}-deg`);
      const valPulse = document.getElementById(`val-arm-${j}-pulse`);

      if (slider) slider.value = angles[j];
      if (valDeg) valDeg.textContent = `${angles[j].toFixed(1)}°`;
      if (valPulse) valPulse.textContent = `${pulses[j]} pls`;
    });

    const tcp = this.arm6DOF.getTCPWorldPosition();
    const tcpX = document.getElementById('arm-tcp-x');
    const tcpY = document.getElementById('arm-tcp-y');
    const tcpZ = document.getElementById('arm-tcp-z');
    if (tcpX) tcpX.textContent = tcp.x.toFixed(3);
    if (tcpY) tcpY.textContent = tcp.y.toFixed(3);
    if (tcpZ) tcpZ.textContent = tcp.z.toFixed(3);

    // Synchronize Yaskawa Teach Pendant Status in Dock
    if (this.teachPendant) {
      const elMode = document.getElementById('dock-status-mode');
      const elServo = document.getElementById('dock-status-servo');
      const elDm = document.getElementById('dock-status-deadman');
      const elCoord = document.getElementById('dock-status-coord');

      if (elMode) elMode.textContent = ['TEACH', 'PLAY', 'REMOTE'][this.teachPendant.robotMode] || 'TEACH';
      if (elServo) {
        elServo.textContent = this.teachPendant.servoOn ? 'ON' : 'OFF';
        elServo.style.color = this.teachPendant.servoOn ? '#4ade80' : '#94a3b8';
      }
      if (elDm) {
        const dmNames = ['RELEASED', 'LIGHT (SAFE)', 'TIGHT (PANIC)'];
        elDm.textContent = dmNames[this.teachPendant.deadmanState] || 'OFF';
        elDm.style.color = (this.teachPendant.deadmanState === 1) ? '#4ade80' : '#ef4444';
      }
      if (elCoord) elCoord.textContent = this.teachPendant.currentCoord;

      this.teachPendant.updateLiveReadouts();
    }
  }

  updateDeltaUIFromModel() {
    const angles = this.deltaRobot.getJointAngles();

    ['m1', 'm2', 'm3'].forEach(m => {
      const slider = document.getElementById(`slider-delta-${m}`);
      const valEl = document.getElementById(`val-delta-${m}-deg`);
      if (slider) slider.value = angles[m];
      if (valEl) valEl.textContent = `${angles[m].toFixed(1)}°`;
    });

    const sliderYaw = document.getElementById('slider-delta-yaw');
    const valYaw = document.getElementById('val-delta-yaw-deg');
    if (sliderYaw) sliderYaw.value = angles.yaw;
    if (valYaw) valYaw.textContent = `${angles.yaw.toFixed(1)}°`;

    const pPos = this.deltaRobot.getEndEffectorWorldPosition();
    document.getElementById('delta-tcp-x').textContent = pPos.x.toFixed(3);
    document.getElementById('delta-tcp-y').textContent = pPos.y.toFixed(3);
    document.getElementById('delta-tcp-z').textContent = pPos.z.toFixed(3);
  }

  syncTrailToolbarState() {
    const btn = document.getElementById('btn-toggle-trajectory-trails');
    if (btn && this.trajectoryTrail) {
      const isAnyActive = Boolean(this.trajectoryTrail.visible && (this.trajectoryTrail.armVisible || this.trajectoryTrail.deltaVisible));
      btn.classList.toggle('active', isAnyActive);
      if (isAnyActive) {
        btn.style.background = '#854d0e';
        btn.style.borderColor = '#facc15';
        btn.style.color = '#fef08a';
      } else {
        btn.style.background = '';
        btn.style.borderColor = '#f59e0b';
        btn.style.color = '#fbbf24';
      }
    }
  }

  updateIOTelemetry(forceDock = false) {
    // 1. Topbar Lightweight Status Chips (Chỉ cập nhật những chỉ số hiển thị trên thanh trạng thái trên cùng)
    const chipDI11 = document.getElementById('chip-di11');
    if (chipDI11) {
      if (this.factory.sensorDI11) {
        chipDI11.textContent = "DETECTED";
        chipDI11.style.color = "#ef4444";
      } else {
        chipDI11.textContent = "CLEAR";
        chipDI11.style.color = "#22c55e";
      }
    }

    const chipDI12 = document.getElementById('chip-di12');
    if (chipDI12) {
      if (this.factory.sensorDI12) {
        chipDI12.textContent = "BOX READY";
        chipDI12.style.color = "#22c55e";
      } else {
        chipDI12.textContent = "WAITING";
        chipDI12.style.color = "#94a3b8";
      }
    }

    const totalInspected = this.factory.totalInspectedCount || 0;
    const ngDefects = this.factory.ngDefectCount || 0;
    const sealedBoxes = this.factory.sealedCartonCount || 0;
    const verifiedBoxes = this.factory.totalVerifiedBoxes || 0;
    const encPulses = this.factory.Encoder_Pulse_Counter || 0;

    const elChipInspected = document.getElementById('chip-total-inspected');
    const elChipNG = document.getElementById('chip-ng-defect');
    const elChipSealed = document.getElementById('chip-sealed-boxes');
    const elChipVerified = document.getElementById('chip-verified-boxes');
    const elChipEnc = document.getElementById('chip-encoder-pulses');

    if (elChipInspected) elChipInspected.textContent = totalInspected;
    if (elChipNG) elChipNG.textContent = ngDefects;
    if (elChipSealed) elChipSealed.textContent = sealedBoxes;
    if (elChipVerified) elChipVerified.textContent = verifiedBoxes;
    if (elChipEnc) elChipEnc.textContent = encPulses;

    // Dynamic Andon Tower Light Logic
    if (this.factory) {
      if (this.isSafetyInterlockTripped || (this.hardwareBus && this.hardwareBus.emergencyStop)) {
        this.factory.setAndonState('ALARM');
      } else if (this.autoDemoMode || this.isSingleCycleRunning || (this.systemPower && (this.factory.conveyor1Active || this.isDeltaBusy || this.isArmBusy))) {
        this.factory.setAndonState('RUN');
      } else {
        this.factory.setAndonState('STANDBY');
      }
    }

    // 2. On-Demand Detail DOM Updates: Chỉ truy vấn và cập nhật ~35 thẻ DOM chi tiết bên trong Inspector Dock
    // khi người dùng đang ở màn hình 3D chính (`activeWorkstation === 'default'`) và đang mở tab Jogger hoặc Kinematics!
    const isDockVisible = (this.activeWorkstation === 'default');
    const isJoggerActive = forceDock || (isDockVisible && document.getElementById('tab-pane-jogger')?.classList.contains('active'));
    const isKinActive = forceDock || (isDockVisible && document.getElementById('tab-pane-kinematics')?.classList.contains('active'));

    if (!isJoggerActive && !isKinActive) return;

    const isStopperRetracted = (this.factory.stopperState === "RETRACTED");
    const isPusherRejecting = Boolean(this.factory.rejectCylinderDO || this.factory.DO_REJECT_CYLINDER);
    const isSealerRunning = Boolean(this.factory.sealerActive || this.factory.DO_SEALER_ACTIVE);
    const isSealerSensTripped = Boolean(this.factory.sealerSensorActive || this.factory.DI_SEALER_SENSOR);

    if (isKinActive) {
      const ledKinStopper = document.getElementById('led-kin-stopper');
      const lblKinStopper = document.getElementById('label-kin-stopper');
      const ledKinReject = document.getElementById('led-kin-reject');
      const lblKinReject = document.getElementById('label-kin-reject');
      const ledKinSealer = document.getElementById('led-kin-sealer');
      const lblKinSealer = document.getElementById('label-kin-sealer');

      if (ledKinStopper) ledKinStopper.className = isStopperRetracted ? 'scada-led led-green active' : 'scada-led led-gray';
      if (lblKinStopper) lblKinStopper.textContent = isStopperRetracted ? 'MỞ CHỐT (PASS)' : 'ĐÓNG CHỐT (BLOCK)';
      if (ledKinReject) ledKinReject.className = isPusherRejecting ? 'scada-led led-red active' : 'scada-led led-gray';
      if (lblKinReject) lblKinReject.textContent = isPusherRejecting ? 'BẮN PHẾ PHẨM' : 'CHỜ PHẾ PHẨM';
      if (ledKinSealer) ledKinSealer.className = isSealerRunning ? 'scada-led led-green active' : 'scada-led led-gray';
      if (lblKinSealer) lblKinSealer.textContent = isSealerRunning ? 'ĐANG DÁN NẮP' : 'CHỜ DÁN NẮP';
    }

    if (!isJoggerActive) return;

    const ledDI11 = document.getElementById('led-di11');
    if (ledDI11) ledDI11.classList.toggle('active', Boolean(this.factory.sensorDI11));

    const ledDI12 = document.getElementById('led-di12');
    if (ledDI12) ledDI12.classList.toggle('active', Boolean(this.factory.sensorDI12));

    const ledStopper = document.getElementById('led-stopper-cylinder');
    const lblStopper = document.getElementById('label-stopper-cylinder');
    if (ledStopper) ledStopper.className = isStopperRetracted ? 'scada-led led-green active' : 'scada-led led-gray';
    if (lblStopper) lblStopper.textContent = isStopperRetracted ? 'MỞ CHỐT (PASS)' : 'ĐÓNG CHỐT (BLOCK)';

    const ledReject = document.getElementById('led-do-reject');
    const lblReject = document.getElementById('label-do-reject');
    if (ledReject) ledReject.className = isPusherRejecting ? 'scada-led led-red active' : 'scada-led led-gray';
    if (lblReject) lblReject.textContent = isPusherRejecting ? 'BẮN PHẾ PHẨM (REJECT)' : 'CHỜ PHẾ PHẨM';

    const ledSealer = document.getElementById('led-do-sealer');
    const lblSealer = document.getElementById('label-do-sealer');
    const ledDiSealer = document.getElementById('led-di-sealer');
    const lblDiSealer = document.getElementById('label-di-sealer');

    if (ledSealer) ledSealer.className = isSealerRunning ? 'scada-led led-green active' : 'scada-led led-gray';
    if (lblSealer) lblSealer.textContent = isSealerRunning ? 'ĐANG DÁN BĂNG KEO' : 'CHỜ THÙNG';
    if (ledDiSealer) ledDiSealer.className = isSealerSensTripped ? 'scada-led led-red active' : 'scada-led led-green';
    if (lblDiSealer) lblDiSealer.textContent = isSealerSensTripped ? 'CÓ THÙNG (DETECT)' : 'THÔNG THOÁNG';

    const elCardInspected = document.getElementById('val-inspected-card');
    const elCardNG = document.getElementById('val-ng-card');
    const elCardSealed = document.getElementById('val-sealed-card');
    const elCardVerified = document.getElementById('val-verified-boxes');
    if (elCardInspected) elCardInspected.textContent = totalInspected;
    if (elCardNG) elCardNG.textContent = ngDefects;
    if (elCardSealed) elCardSealed.textContent = sealedBoxes;
    if (elCardVerified) elCardVerified.textContent = verifiedBoxes;

    const lineSpeedMms = this.factory.conveyor1Active ? ((this.factory.speedConveyor1 || 0) * 1000).toFixed(0) : '0';
    const basePulse = this.factory.Base_Encoder_Pulse || 0;
    const deltaPulse = this.factory.lastTrackedDeltaPulse || 0;
    const encFreq = this.factory.encoderFreqHz || 0;

    const elEncPulses = document.getElementById('val-encoder-pulses');
    const elLineSpeed = document.getElementById('val-line-speed-mms');
    const elEncBaseDelta = document.getElementById('val-encoder-base-delta');
    const elEncFreq = document.getElementById('val-encoder-freq');
    const ledEncA = document.getElementById('led-enc-ch-a');
    const ledEncB = document.getElementById('led-enc-ch-b');

    if (elEncPulses) elEncPulses.textContent = `${encPulses.toLocaleString()} pls`;
    if (elLineSpeed) elLineSpeed.textContent = `${lineSpeedMms} mm/s`;
    if (elEncBaseDelta) elEncBaseDelta.textContent = `${basePulse} / +${deltaPulse} pls`;
    if (elEncFreq) elEncFreq.textContent = `${encFreq} Hz`;
    if (ledEncA) ledEncA.classList.toggle('active', Boolean(this.factory.conveyor1Active && this.factory.encoderChannelA));
    if (ledEncB) ledEncB.classList.toggle('active', Boolean(this.factory.conveyor1Active && this.factory.encoderChannelB));

    // Virtual I/O LEDs: DI_PRINT_TRIGGER, DO_INKJET_FIRE, DI_BARCODE_OK
    const isPrintTrig = Boolean(this.factory.DI_PRINT_TRIGGER);
    const isInkjetFire = Boolean(this.factory.DO_INKJET_FIRE);
    const isBarcodeOk = Boolean(this.factory.DI_BARCODE_OK || (this.factory.barcodeScanFlashTimer > 0 && this.factory.Barcode_Scan_Valid));

    const ledPrintTrig = document.getElementById('led-di-print-trigger');
    const lblPrintTrig = document.getElementById('label-di-print-trigger');
    const ledInkjet = document.getElementById('led-do-inkjet-fire');
    const lblInkjet = document.getElementById('label-do-inkjet-fire');
    const ledBarcodeOk = document.getElementById('led-di-barcode-ok');
    const lblBarcodeOk = document.getElementById('label-di-barcode-ok');

    if (ledPrintTrig) ledPrintTrig.className = isPrintTrig ? 'scada-led led-red active' : 'scada-led led-green';
    if (lblPrintTrig) lblPrintTrig.textContent = isPrintTrig ? 'PHÁT HIỆN THÙNG' : 'Cảm biến Trạm In';
    if (ledInkjet) ledInkjet.className = isInkjetFire ? 'scada-led led-cyan active' : 'scada-led led-gray';
    if (lblInkjet) lblInkjet.textContent = isInkjetFire ? 'ĐANG BẮN MỰC CIJ' : 'CHỜ BẮN MỰC';
    if (ledBarcodeOk) {
      if (this.factory.Barcode_Read_Error) {
        ledBarcodeOk.className = 'scada-led led-red active';
      } else if (isBarcodeOk) {
        ledBarcodeOk.className = 'scada-led led-green active';
      } else {
        ledBarcodeOk.className = 'scada-led led-gray';
      }
    }
    if (lblBarcodeOk) {
      lblBarcodeOk.textContent = this.factory.Barcode_Read_Error ? 'BARCODE ERROR!' : (isBarcodeOk ? 'VERIFIED [OK]' : 'CHỜ QUÉT MÃ');
    }

    // Traceability Panel (Last Printed Box Info)
    const info = this.factory.lastPrintedBoxInfo;
    if (info) {
      const elBoxId = document.getElementById('val-trace-box-id');
      const elMfg = document.getElementById('val-trace-mfg');
      const elExp = document.getElementById('val-trace-exp');
      const elLot = document.getElementById('val-trace-lot');
      const elStatus = document.getElementById('val-trace-barcode-status');

      if (elBoxId) elBoxId.textContent = info.boxId;
      if (elMfg) elMfg.textContent = info.mfg;
      if (elExp) elExp.textContent = info.exp;
      if (elLot) elLot.textContent = info.lot;
      if (elStatus) {
        elStatus.textContent = info.status;
        if (info.valid) {
          elStatus.style.color = '#4ade80';
          elStatus.style.borderColor = '#22c55e';
          elStatus.style.background = 'rgba(34, 197, 94, 0.18)';
        } else {
          elStatus.style.color = '#f87171';
          elStatus.style.borderColor = '#ef4444';
          elStatus.style.background = 'rgba(239, 68, 68, 0.22)';
        }
      }
    }
  }

  /**
   * Toggles the Maintenance Safety Door & Safety Interlock Circuit
   * @param {boolean|null} forceClosed
   */
  toggleSafetyDoor(forceClosed = null) {
    if (!this.factory) return;
    const isClosed = this.factory.toggleSafetyDoor(forceClosed);

    // 1. Update Topbar Button & LED
    const btnQuick = document.getElementById('btn-quick-safety-door');
    const ledQuick = document.getElementById('led-safety-door-top');
    const lblQuick = document.getElementById('label-safety-door-top');
    if (btnQuick) btnQuick.classList.toggle('door-open', !isClosed);
    if (ledQuick) {
      ledQuick.className = isClosed ? 'scada-led led-green active' : 'scada-led led-red active';
    }
    if (lblQuick) {
      lblQuick.textContent = isClosed ? 'CỬA RÀO: ĐÓNG' : 'CỬA RÀO: MỞ (ALARM)';
    }

    // 2. Update Dock SCADA card
    const btnDock = document.getElementById('btn-toggle-safety-door');
    const statusDock = document.getElementById('status-safety-door');
    if (btnDock) {
      btnDock.textContent = isClosed ? '🚪 MỞ CỬA BẢO TRÌ' : '🚪 ĐÓNG CỬA BẢO TRÌ';
      btnDock.style.borderColor = isClosed ? '#f59e0b' : '#ef4444';
      btnDock.style.color = isClosed ? '#facc15' : '#fca5a5';
    }
    if (statusDock) {
      statusDock.textContent = isClosed ? 'SAFE / CLOSED' : 'TRIPPED / OPEN';
      statusDock.className = isClosed ? 'dock-status-tag online' : 'dock-status-tag error';
    }

    // 3. Update DI_01 in telemetry
    const ledDi01 = document.getElementById('led-di01');
    const lblDi01 = document.getElementById('label-di01');
    if (ledDi01) ledDi01.className = isClosed ? 'scada-led led-green active' : 'scada-led led-red active';
    if (lblDi01) lblDi01.textContent = isClosed ? 'Gate Closed' : 'GATE OPEN';
    if (this.hardwareBus) {
      this.hardwareBus.setDI(1, isClosed ? 1 : 0);
    }

    // 4. Synchronize Chaos Lab Fault card
    if (this.faultPanel) {
      this.faultPanel.faultStates.safetyGateOpen = !isClosed;
      const btnGate = document.getElementById('btn-inject-gate');
      const statusGate = document.getElementById('status-fault-gate');
      const cardGate = document.getElementById('card-fault-gate');
      if (btnGate) btnGate.textContent = !isClosed ? 'CLOSE GATE' : '🚪 OPEN GATE';
      if (statusGate) {
        statusGate.textContent = !isClosed ? 'OPEN (E-STOP)' : 'CLOSED';
        statusGate.className = !isClosed ? 'fault-status-tag error' : 'fault-status-tag';
      }
      if (cardGate) cardGate.classList.toggle('active-fault', !isClosed);
    }

    // 5. SAFETY INTERLOCK CIRCUIT TRIP LOGIC
    if (!isClosed) {
      // DOOR OPENED -> IMMEDIATE HARDWARE INTERLOCK TRIP!
      this.isSafetyInterlockTripped = true;

      // 1. Ngắt ngay lập tức tín hiệu động lực Servo của cả 2 robot (Servo OFF / STO)
      this.systemPower = false;
      const btnPower = document.getElementById('btn-system-power');
      const ledPower = document.getElementById('led-power-status');
      const powerLabel = document.getElementById('power-status-label');
      const dockStatus = document.getElementById('dock-system-tag');
      const lineStateEl = document.getElementById('telemetry-line-state');

      if (btnPower) btnPower.classList.remove('active');
      if (ledPower) ledPower.classList.remove('active');
      if (powerLabel) powerLabel.textContent = "STO ACTIVATED // SERVO OFF";
      if (dockStatus) {
        dockStatus.textContent = "INTERLOCK TRIPPED";
        dockStatus.className = "dock-status-tag error";
      }
      if (lineStateEl) {
        lineStateEl.textContent = "GATE OPEN // STO";
        lineStateEl.style.color = "#ef4444";
      }

      // Halt motion trajectories & release grippers
      if (this.trajectoryPlanner) {
        this.trajectoryPlanner.stopAll();
      }
      this.isDeltaBusy = false;
      this.isArmBusy = false;
      this.isSingleCycleRunning = false;
      this.autoDemoMode = false;
      const btnAutoDemo = document.getElementById('btn-auto-demo-cycle');
      const ledAutoDemo = document.getElementById('led-auto-demo');
      const labelAutoDemo = document.getElementById('label-auto-demo');
      if (btnAutoDemo) btnAutoDemo.classList.remove('running');
      if (ledAutoDemo) ledAutoDemo.classList.remove('active');
      if (labelAutoDemo) labelAutoDemo.textContent = "AUTO DEMO CYCLE";

      if (this.physicsEngine) {
        this.physicsEngine.releaseDeltaGripper();
        this.physicsEngine.releaseArmGripper();
        this.physicsEngine.clearFallingObjects();
      }

      // 2. Dừng khẩn cấp cả 3 động cơ băng tải
      this.factory.conveyor1Active = false;
      this.factory.conveyor2Active = false;
      this.factory.conveyor3Active = false;
      ['1', '2', '3'].forEach(idx => {
        const t = document.getElementById(`toggle-conv${idx}`);
        const l = document.getElementById(`led-conv${idx}`);
        if (t) { t.checked = false; t.disabled = true; }
        if (l) l.classList.remove('active');
      });

      if (this.hardwareBus) {
        this.hardwareBus.setEmergencyStop(true);
        this.hardwareBus.setDO(0, 0);
        this.hardwareBus.setDO(1, 0);
        this.hardwareBus.setDO(3, 0);
      }

      // 3. Đèn Andon chuyển sang ĐỎ nhấp nháy
      this.factory.setAndonState("ALARM");

      // 4. Hiển thị popup cảnh báo viền đỏ trên màn hình
      const modal = document.getElementById('safety-interlock-modal');
      if (modal) modal.style.display = 'flex';
      const modalStatus = document.getElementById('modal-interlock-status');
      if (modalStatus) {
        modalStatus.textContent = 'CONTACT OPEN // TIẾP ĐIỂM HỞ';
        modalStatus.className = 'safety-status-val danger';
        modalStatus.style.color = '#ef4444';
      }

      if (this.vipaHMI) {
        this.vipaHMI.triggerAlarm('ALM-02', 'CỬA BẢO TRÌ HÀNG RÀO MỞ - SAFETY INTERLOCK KÍCH HOẠT (STO / E-STOP)!');
      }
      if (this.oeeCalculator) {
        this.oeeCalculator.logEvent('STOP', 'SAFETY STOP: Door opened - Interlock tripped!');
        this.oeeCalculator._lastGateClosed = false;
      }
    } else {
      // Door was closed back: update modal guide, system stays safe until Reset Fault / Robot On is pressed
      const modalStatus = document.getElementById('modal-interlock-status');
      if (modalStatus) {
        modalStatus.textContent = 'CONTACT CLOSED (CỬA ĐÃ ĐÓNG - CHỜ RESET)';
        modalStatus.className = 'safety-status-val';
        modalStatus.style.color = '#4ade80';
      }
    }

    return isClosed;
  }

  /**
   * Resets the Safety Interlock circuit and restores normal readiness
   */
  resetSafetyInterlock() {
    if (!this.factory.safetyGateClosed) {
      alert("⚠ CẢNH BÁO AN TOÀN:\nCửa bảo trì rào chắn vẫn đang mở!\nHãy đóng kín cánh cửa rào trước khi thực hiện Reset Fault / Robot On.");
      return false;
    }

    this.isSafetyInterlockTripped = false;

    // Release Hardware Bus E-Stop
    if (this.hardwareBus) {
      this.hardwareBus.setEmergencyStop(false);
    }
    const estopBtn = document.getElementById('ctrl-estop');
    if (estopBtn) estopBtn.classList.remove('locked');

    // Hide warning popup
    const modal = document.getElementById('safety-interlock-modal');
    if (modal) modal.style.display = 'none';

    // Set Andon Light to STANDBY (Yellow ready)
    this.factory.setAndonState("STANDBY");

    // Enable conveyor switches
    document.querySelectorAll('.conveyor-toggle-input').forEach(input => {
      input.disabled = false;
    });

    // Turn on system power & servo (Robot On)
    this.systemPower = true;
    const btnPower = document.getElementById('btn-system-power');
    const ledPower = document.getElementById('led-power-status');
    const powerLabel = document.getElementById('power-status-label');
    const dockStatus = document.getElementById('dock-system-tag');
    const lineStateEl = document.getElementById('telemetry-line-state');

    if (btnPower) btnPower.classList.add('active');
    if (ledPower) ledPower.classList.add('active');
    if (powerLabel) powerLabel.textContent = "SERVO ON // READY";
    if (dockStatus) {
      dockStatus.textContent = "ONLINE";
      dockStatus.className = "dock-status-tag online";
    }
    if (lineStateEl) {
      lineStateEl.textContent = "STANDBY / READY";
      lineStateEl.style.color = "#22c55e";
    }

    // Pulse Robot On (DI_03)
    if (this.hardwareBus) {
      this.hardwareBus.setDI(3, 1);
      setTimeout(() => {
        if (this.hardwareBus) this.hardwareBus.setDI(3, 0);
      }, 200);
    }

    if (this.vipaHMI) {
      this.vipaHMI.clearAlarm('ALM-02');
    }
    if (this.oeeCalculator) {
      this.oeeCalculator.logEvent('INFO', 'SAFETY RESET: Interlock cleared, Servo ON ready.');
      this.oeeCalculator._lastGateClosed = true;
    }

    return true;
  }

  initKinematicsAndPhysics() {
    this.armKinematics = new Yaskawa6DOFKinematics({
      worldBasePosition: this.arm6DOF.basePosition
    });
    this.deltaKinematics = new Delta4DOFKinematics({
      worldBasePosition: this.deltaRobot.basePosition
    });
    this.trajectoryPlanner = new TrajectoryPlanner();
    this.trajectoryTrail = new TrajectoryTrail(this.scene);
    this.trajectoryTrail.setVisible(false);
    this.trajectoryTrail.setArmVisible(false);
    this.trajectoryTrail.setDeltaVisible(false);
    this.syncTrailToolbarState();
    this.physicsEngine = new VacuumAndSensorEngine(
      this.scene,
      this.factory,
      this.deltaRobot,
      this.arm6DOF
    );
    this.isSingleCycleRunning = false;
    this.isDeltaBusy = false;
    this.isArmBusy = false;
  }

  initKinematicsLabListeners() {
    // 1. Tab Switching between Jogger, Kinematics Lab, Control Panel, UDP Sniffer, and OEE
    const tabBtns = document.querySelectorAll('.dock-tab-btn');
    tabBtns.forEach(btn => {
      btn.addEventListener('click', () => {
        tabBtns.forEach(b => b.classList.remove('active'));
        document.querySelectorAll('.tab-pane').forEach(p => p.classList.remove('active'));
        btn.classList.add('active');
        const targetId = btn.dataset.tab;
        const targetPane = document.getElementById(targetId);
        if (targetPane) targetPane.classList.add('active');

        // On-Demand Synchronization: Đồng bộ dữ liệu ngay lập tức cho tab vừa được người dùng bấm mở
        if (targetId === 'tab-pane-jogger') {
          this.updateIOTelemetry(true);
          if (this.oeeCalculator) this.oeeCalculator.renderDashboardDOM();
        } else if (targetId === 'tab-pane-kinematics') {
          this.updateKinematicsLabUI();
          this.updateIOTelemetry(true);
        } else if (targetId === 'tab-pane-control-panel') {
          this.updateHardwareUI();
        } else if (targetId === 'tab-pane-oee') {
          if (this.oeeCalculator) this.oeeCalculator.renderDashboardDOM();
        }
      });
    });

    // 2. Single Pick & Place Cycle Button
    const btnSingleCycle = document.getElementById('btn-test-single-cycle');
    if (btnSingleCycle) {
      btnSingleCycle.addEventListener('click', () => {
        this.runSinglePickAndPlaceCycle();
      });
    }

    // 3. Trajectory Trails Checkbox & Clear Button
    const chkTrails = document.getElementById('chk-show-trails');
    if (chkTrails) {
      chkTrails.addEventListener('change', (e) => {
        this.trajectoryTrail.setVisible(e.target.checked);
        const chkArmTrail = document.getElementById('chk-show-arm-trail');
        const chkDeltaTrail = document.getElementById('chk-show-delta-trail');
        if (chkArmTrail) chkArmTrail.checked = e.target.checked;
        if (chkDeltaTrail) chkDeltaTrail.checked = e.target.checked;
        this.syncTrailToolbarState();
      });
    }

    const btnClearTrails = document.getElementById('btn-clear-trails');
    if (btnClearTrails) {
      btnClearTrails.addEventListener('click', () => {
        this.trajectoryTrail.clear();
      });
    }

    // Quick camera toolbar toggle for 3D trajectory path
    const btnToggleTrails = document.getElementById('btn-toggle-trajectory-trails');
    if (btnToggleTrails) {
      btnToggleTrails.addEventListener('click', () => {
        const nextState = !(this.trajectoryTrail && this.trajectoryTrail.visible);
        if (this.trajectoryTrail) {
          this.trajectoryTrail.setVisible(nextState);
        }
        const chkArmTrail = document.getElementById('chk-show-arm-trail');
        const chkDeltaTrail = document.getElementById('chk-show-delta-trail');
        const chkTrails = document.getElementById('chk-show-trails');
        if (chkArmTrail) chkArmTrail.checked = nextState;
        if (chkDeltaTrail) chkDeltaTrail.checked = nextState;
        if (chkTrails) chkTrails.checked = nextState;
        this.syncTrailToolbarState();
      });
    }

    // 4. Robot Selector in Kinematics Lab
    const robotSelect = document.getElementById('lab-robot-select');
    const updatePresetsForRobot = () => {
      const isArm = robotSelect.value === 'arm';
      const btnPick = document.getElementById('btn-lab-pos-pick');
      const btnBox = document.getElementById('btn-lab-pos-box');
      const btnOver = document.getElementById('btn-lab-pos-overhead');
      const btnHome = document.getElementById('btn-lab-pos-home');

      if (isArm) {
        if (btnPick) btnPick.textContent = 'Pick Belt 2';
        if (btnBox) btnBox.textContent = 'Box Place';
        if (btnOver) btnOver.textContent = 'Overhead';
        if (btnHome) btnHome.textContent = 'Home';
      } else {
        if (btnPick) btnPick.textContent = 'Pick Belt 1';
        if (btnBox) btnBox.textContent = 'Collate Belt 2';
        if (btnOver) btnOver.textContent = 'Overhead';
        if (btnHome) btnHome.textContent = 'Delta Ready';
      }
    };

    if (robotSelect) {
      robotSelect.addEventListener('change', () => {
        updatePresetsForRobot();
      });
    }

    // Quick Presets
    const setLabInputs = (x, y, z, rx = 0, ry = 0, rz = 0) => {
      document.getElementById('lab-target-x').value = x.toFixed(2);
      document.getElementById('lab-target-y').value = y.toFixed(2);
      document.getElementById('lab-target-z').value = z.toFixed(2);
      document.getElementById('lab-target-rx').value = rx.toFixed(1);
      document.getElementById('lab-target-ry').value = ry.toFixed(1);
      document.getElementById('lab-target-rz').value = rz.toFixed(1);
    };

    const btnPick = document.getElementById('btn-lab-pos-pick');
    if (btnPick) {
      btnPick.addEventListener('click', () => {
        if (robotSelect.value === 'arm') {
          setLabInputs(-1.25, 0.91, 0.18, 0, 0, 0);
        } else {
          setLabInputs(1.20, 0.90, -0.32, 0, 0, 0);
        }
      });
    }

    const btnBox = document.getElementById('btn-lab-pos-box');
    if (btnBox) {
      btnBox.addEventListener('click', () => {
        if (robotSelect.value === 'arm') {
          setLabInputs(-1.25, 0.80, -1.45, 0, 0, 0);
        } else {
          setLabInputs(1.20, 0.90, 0.18, 0, 0, 0);
        }
      });
    }

    const btnOver = document.getElementById('btn-lab-pos-overhead');
    if (btnOver) {
      btnOver.addEventListener('click', () => {
        if (robotSelect.value === 'arm') {
          setLabInputs(-1.25, 1.30, 0.18, 0, 0, 0);
        } else {
          setLabInputs(1.20, 1.35, -0.10, 0, 0, 0);
        }
      });
    }

    const btnHome = document.getElementById('btn-lab-pos-home');
    if (btnHome) {
      btnHome.addEventListener('click', () => {
        if (robotSelect.value === 'arm') {
          setLabInputs(-1.20, 1.40, -0.10, 0, 0, 0);
        } else {
          setLabInputs(1.20, 1.45, -0.10, 0, 0, 0);
        }
      });
    }

    // 5. MOVJ and MOVL Execution Handlers
    const btnExecMOVJ = document.getElementById('btn-exec-movj');
    if (btnExecMOVJ) {
      btnExecMOVJ.addEventListener('click', () => {
        this.executeLabMOVJ();
      });
    }

    const btnExecMOVL = document.getElementById('btn-exec-movl');
    if (btnExecMOVL) {
      btnExecMOVL.addEventListener('click', () => {
        this.executeLabMOVL();
      });
    }

    const btnStopMotion = document.getElementById('btn-stop-traj');
    if (btnStopMotion) {
      btnStopMotion.addEventListener('click', () => {
        this.trajectoryPlanner.stopTrajectory('lab-motion');
        this.trajectoryPlanner.stopTrajectory('arm-cycle');
        this.trajectoryPlanner.stopTrajectory('delta-cycle');
        this.isSingleCycleRunning = false;
        this.isDeltaBusy = false;
        this.isArmBusy = false;
        const cycleBtn = document.getElementById('btn-test-single-cycle');
        if (cycleBtn) cycleBtn.classList.remove('running');
        const fb = document.getElementById('lab-ik-feedback');
        if (fb) fb.textContent = "ĐÃ DỪNG (STOPPED)";
      });
    }
  }

  getLabTargetPose() {
    return {
      x: parseFloat(document.getElementById('lab-target-x').value) || 0,
      y: parseFloat(document.getElementById('lab-target-y').value) || 0,
      z: parseFloat(document.getElementById('lab-target-z').value) || 0,
      rx: parseFloat(document.getElementById('lab-target-rx').value) || 0,
      ry: parseFloat(document.getElementById('lab-target-ry').value) || 0,
      rz: parseFloat(document.getElementById('lab-target-rz').value) || 0
    };
  }

  executeLabMOVJ() {
    const targetPose = this.getLabTargetPose();
    const isArm = document.getElementById('lab-robot-select').value === 'arm';
    const duration = parseFloat(document.getElementById('lab-motion-duration').value) || 2.0;
    const degree = parseInt(document.getElementById('lab-poly-degree').value) || 5;
    const feedback = document.getElementById('lab-ik-feedback');

    if (isArm) {
      const curJoints = this.arm6DOF.getJointAngles();
      const ikResult = this.armKinematics.inverseKinematics(targetPose, curJoints, true);

      if (!ikResult.success) {
        if (feedback) {
          feedback.textContent = ikResult.message;
          feedback.style.color = "#ef4444";
        }
        return;
      }

      if (feedback) {
        feedback.textContent = "MOVJ ĐANG NỘI SUY KHỚP...";
        feedback.style.color = "#38bdf8";
      }

      const traj = this.trajectoryPlanner.createMOVJ(curJoints, ikResult.jointsDeg, {
        duration,
        degree
      });

      this.trajectoryPlanner.startTrajectory('lab-motion', traj, {
        onUpdate: (data) => {
          const q = data.joints;
          this.arm6DOF.setJointAngles(q.s, q.l, q.u, q.r, q.b, q.t);
          this.updateArmUIFromModel();
        },
        onComplete: () => {
          if (feedback) {
            feedback.textContent = "MOVJ HOÀN TẤT";
            feedback.style.color = "#22c55e";
          }
        }
      });
    } else {
      const curAngles = this.deltaRobot.getJointAngles();
      const ikResult = this.deltaKinematics.inverseKinematics(
        targetPose.x,
        targetPose.y,
        targetPose.z,
        targetPose.rx,
        true
      );

      if (!ikResult.success) {
        if (feedback) {
          feedback.textContent = ikResult.message;
          feedback.style.color = "#ef4444";
        }
        return;
      }

      if (feedback) {
        feedback.textContent = "MOVJ DELTA ĐANG NỘI SUY...";
        feedback.style.color = "#38bdf8";
      }

      const traj = this.trajectoryPlanner.createMOVJ(
        { m1: curAngles.m1, m2: curAngles.m2, m3: curAngles.m3, yaw: curAngles.yaw },
        ikResult.anglesDeg,
        { duration, degree }
      );

      this.trajectoryPlanner.startTrajectory('lab-motion', traj, {
        onUpdate: (data) => {
          const q = data.joints;
          this.deltaRobot.setJointAngles(q.m1, q.m2, q.m3, q.yaw);
          this.updateDeltaUIFromModel();
        },
        onComplete: () => {
          if (feedback) {
            feedback.textContent = "MOVJ DELTA HOÀN TẤT";
            feedback.style.color = "#22c55e";
          }
        }
      });
    }
  }

  executeLabMOVL() {
    const targetPose = this.getLabTargetPose();
    const isArm = document.getElementById('lab-robot-select').value === 'arm';
    const duration = parseFloat(document.getElementById('lab-motion-duration').value) || 2.0;
    const degree = parseInt(document.getElementById('lab-poly-degree').value) || 5;
    const feedback = document.getElementById('lab-ik-feedback');

    if (isArm) {
      const curPos = this.arm6DOF.getTCPWorldPosition();
      const startPose = {
        x: curPos.x,
        y: curPos.y,
        z: curPos.z,
        rx: targetPose.rx,
        ry: targetPose.ry,
        rz: targetPose.rz
      };

      const ikCheck = this.armKinematics.inverseKinematics(targetPose, this.arm6DOF.getJointAngles(), true);
      if (!ikCheck.success) {
        if (feedback) {
          feedback.textContent = ikCheck.message;
          feedback.style.color = "#ef4444";
        }
        return;
      }

      if (feedback) {
        feedback.textContent = "MOVL ĐANG NỘI SUY THẲNG 3D...";
        feedback.style.color = "#38bdf8";
      }

      const traj = this.trajectoryPlanner.createMOVL(
        startPose,
        targetPose,
        this.armKinematics,
        {
          duration,
          degree,
          isWorld: true,
          initialJoints: this.arm6DOF.getJointAngles()
        }
      );

      this.trajectoryPlanner.startTrajectory('lab-motion', traj, {
        onUpdate: (data) => {
          if (data.ikResult && data.ikResult.success) {
            const q = data.ikResult.jointsDeg;
            this.arm6DOF.setJointAngles(q.s, q.l, q.u, q.r, q.b, q.t);
            this.updateArmUIFromModel();
          }
        },
        onComplete: () => {
          if (feedback) {
            feedback.textContent = "MOVL HOÀN TẤT";
            feedback.style.color = "#22c55e";
          }
        }
      });
    } else {
      const curPos = this.deltaRobot.getEndEffectorWorldPosition();
      const curAngles = this.deltaRobot.getJointAngles();
      const startPose = {
        x: curPos.x,
        y: curPos.y,
        z: curPos.z,
        rx: curAngles.yaw
      };

      const ikCheck = this.deltaKinematics.inverseKinematics(
        targetPose.x,
        targetPose.y,
        targetPose.z,
        targetPose.rx,
        true
      );

      if (!ikCheck.success) {
        if (feedback) {
          feedback.textContent = ikCheck.message;
          feedback.style.color = "#ef4444";
        }
        return;
      }

      if (feedback) {
        feedback.textContent = "MOVL DELTA ĐANG NỘI SUY THẲNG...";
        feedback.style.color = "#38bdf8";
      }

      const traj = this.trajectoryPlanner.createMOVL(
        startPose,
        targetPose,
        this.deltaKinematics,
        {
          duration,
          degree,
          isWorld: true
        }
      );

      this.trajectoryPlanner.startTrajectory('lab-motion', traj, {
        onUpdate: (data) => {
          if (data.ikResult && data.ikResult.success) {
            const a = data.ikResult.anglesDeg;
            this.deltaRobot.setJointAngles(a.m1, a.m2, a.m3, a.yaw);
            this.updateDeltaUIFromModel();
          }
        },
        onComplete: () => {
          if (feedback) {
            feedback.textContent = "MOVL DELTA HOÀN TẤT";
            feedback.style.color = "#22c55e";
          }
        }
      });
    }
  }

  /**
   * Independent Worker 1: High-Speed Robot Delta 4-DOF
   * Picks 1 chocolate bar from Conveyor 1 and places it into the current 4-pack slot on Conveyor 2.
   * Runs continuously whenever a bar is within reach, without waiting for the 6-DOF Arm!
   */
  async runDeltaCollateStep(infeedProd = null) {
    if (this.isDeltaBusy) return;
    this.isDeltaBusy = true;
    this.isSingleCycleRunning = true;

    const statusEl = document.getElementById('cycle-status-text');
    const lineStateEl = document.getElementById('telemetry-line-state');
    const chipVisionEl = document.getElementById('chip-vision');

    const isAborted = () => (
      !this.systemPower ||
      this.isSafetyInterlockTripped ||
      this.isCollisionActive ||
      (this.hardwareBus && this.hardwareBus.emergencyStop)
    );

    const runTraj = (id, traj, onUpdateJoints) => {
      return new Promise((resolve) => {
        if (isAborted()) {
          resolve({ cancelled: true });
          return;
        }
        this.trajectoryPlanner.startTrajectory(id, traj, {
          onUpdate: (evalData) => {
            if (isAborted()) return;
            if (onUpdateJoints) onUpdateJoints(evalData);
          },
          onComplete: (evalData) => resolve(evalData || {})
        });
      });
    };
    const sleep = (ms) => new Promise(res => setTimeout(res, ms));
    // Dynamic trajectory speed scaling based on chocolate infeed rate so at 2.2x Max rate Delta works at blazing capacity!
    const rateBoost = Math.max(0.75, Math.min(1.85, this.factory.infeedRateMultiplier || 1.0));
    const dScale = 1.0 / rateBoost;

    try {
      if (isAborted()) return;
      if (!infeedProd) {
        infeedProd = this.physicsEngine.findReachableInfeedProduct(1.2, 0.72);
      }
      if (!infeedProd) {
        infeedProd = this.factory.spawnProduct(1.25, (Math.random() - 0.5) * 0.12, 15);
        await sleep(10);
      }
      if (isAborted()) return;

      let buildingPack = this.factory.collatedPacks.find(p => !p.complete && !p.picked);
      const currentBarsInSlot = buildingPack ? buildingPack.barCount : 0;
      const barNum = Math.min(4, currentBarsInSlot + 1);

      if (statusEl) statusEl.textContent = `[SONG SONG] DELTA XẾP THANH ${barNum}/4 | ARM ĐÓNG THÙNG`;
      if (lineStateEl) { lineStateEl.textContent = `PARALLEL: DELTA ${barNum}/4`; lineStateEl.style.color = "#38bdf8"; }
      if (chipVisionEl) chipVisionEl.textContent = `BAR ${barNum}/4`;

      const prodWorldPos = new THREE.Vector3();
      infeedProd.getWorldPosition(prodWorldPos);
      // Extract rotation angle θ (deg) detected by Cognex Vision Camera (or product yaw) for 4th-axis alignment
      const prodYawDeg = THREE.MathUtils.radToDeg(infeedProd.rotation ? infeedProd.rotation.y : 0);

      // 1. High-speed Pre-pick approach (4th-axis yaw rotates to align with bar angle θ)
      const deltaCur = this.deltaRobot.getEndEffectorWorldPosition();
      const prePick = new THREE.Vector3(prodWorldPos.x, 1.08, prodWorldPos.z);
      let dTraj = this.trajectoryPlanner.createMOVL(
        { x: deltaCur.x, y: deltaCur.y, z: deltaCur.z, yaw: 0 },
        { x: prePick.x, y: prePick.y, z: prePick.z, yaw: prodYawDeg },
        this.deltaKinematics,
        { duration: 0.10 * dScale, isWorld: true }
      );
      let res = await runTraj('delta-cycle', dTraj, (data) => {
        if (data.ikResult && data.ikResult.success) {
          const a = data.ikResult.anglesDeg;
          this.deltaRobot.setJointAngles(a.m1, a.m2, a.m3, a.yaw);
          this.updateDeltaUIFromModel();
        }
      });
      if ((res && res.cancelled) || isAborted()) return;

      // 2. Dynamic Conveyor Tracking Descend & Pick (Synchronized with Rotary Encoder Pulses + Vision Angle θ)
      // Real-time tracking: X(t) = X_0 - (Encoder_Pulse_Counter - Base_Encoder_Pulse) * K_scale
      // pickPos Y = 0.902m lands silicone suction cup lip flush against top face of chocolate bar (0.904m) with 2mm sealing compression
      infeedProd.getWorldPosition(prodWorldPos);
      const pickPos = new THREE.Vector3(prodWorldPos.x, 0.902, prodWorldPos.z);
      dTraj = this.trajectoryPlanner.createMOVL(
        { x: prePick.x, y: prePick.y, z: prePick.z, yaw: prodYawDeg },
        { x: pickPos.x, y: pickPos.y, z: pickPos.z, yaw: prodYawDeg },
        this.deltaKinematics,
        { duration: 0.08 * dScale, isWorld: true }
      );
      res = await runTraj('delta-cycle', dTraj, (data) => {
        infeedProd.getWorldPosition(prodWorldPos);
        pickPos.x = prodWorldPos.x;
        pickPos.z = prodWorldPos.z;
        const curY = (data.pose && data.pose.y !== undefined) ? data.pose.y : 0.902;
        const dynIk = this.deltaKinematics.inverseKinematics(prodWorldPos.x, curY, prodWorldPos.z, prodYawDeg, true);
        if (dynIk && dynIk.success) {
          const a = dynIk.anglesDeg;
          this.deltaRobot.setJointAngles(a.m1, a.m2, a.m3, a.yaw);
          this.updateDeltaUIFromModel();
        } else if (data.ikResult && data.ikResult.success) {
          const a = data.ikResult.anglesDeg;
          this.deltaRobot.setJointAngles(a.m1, a.m2, a.m3, a.yaw);
          this.updateDeltaUIFromModel();
        }
      });
      if ((res && res.cancelled) || isAborted()) return;

      this.physicsEngine.attachDeltaGripper(infeedProd);
      await sleep(10);
      if (isAborted()) {
        this.physicsEngine.releaseDeltaGripper();
        return;
      }

      // 3. Lift up & rotate 4th-axis yaw back to 0° so chocolate bar is square for 4-pack collation
      const liftPos = new THREE.Vector3(pickPos.x, 1.12, pickPos.z);
      dTraj = this.trajectoryPlanner.createMOVL(
        { x: pickPos.x, y: pickPos.y, z: pickPos.z, yaw: prodYawDeg },
        { x: liftPos.x, y: liftPos.y, z: liftPos.z, yaw: 0 },
        this.deltaKinematics,
        { duration: 0.07 * dScale, isWorld: true }
      );
      res = await runTraj('delta-cycle', dTraj, (data) => {
        if (data.ikResult && data.ikResult.success) {
          const a = data.ikResult.anglesDeg;
          this.deltaRobot.setJointAngles(a.m1, a.m2, a.m3, a.yaw);
          this.updateDeltaUIFromModel();
        }
      });
      if ((res && res.cancelled) || isAborted()) return;

      // 4. Transfer to exact Z slot in the 4-pack on Conveyor 2 (slot 0..3: Z = 0.12, 0.16, 0.20, 0.24)
      const targetSlotZ = 0.18 + (-0.06 + currentBarsInSlot * 0.04);
      const dropOverhead = new THREE.Vector3(1.20, 1.12, targetSlotZ);
      dTraj = this.trajectoryPlanner.createMOVL(
        { x: liftPos.x, y: liftPos.y, z: liftPos.z },
        { x: dropOverhead.x, y: dropOverhead.y, z: dropOverhead.z },
        this.deltaKinematics,
        { duration: 0.11 * dScale, isWorld: true }
      );
      res = await runTraj('delta-cycle', dTraj, (data) => {
        if (data.ikResult && data.ikResult.success) {
          const a = data.ikResult.anglesDeg;
          this.deltaRobot.setJointAngles(a.m1, a.m2, a.m3, a.yaw);
          this.updateDeltaUIFromModel();
        }
      });
      if ((res && res.cancelled) || isAborted()) return;

      // 5. Descend to drop into slot (dropPos Y = 0.906m allows smooth bar contact with collation belt without penetration)
      const dropPos = new THREE.Vector3(1.20, 0.906, targetSlotZ);
      dTraj = this.trajectoryPlanner.createMOVL(
        { x: dropOverhead.x, y: dropOverhead.y, z: dropOverhead.z },
        { x: dropPos.x, y: dropPos.y, z: dropPos.z },
        this.deltaKinematics,
        { duration: 0.07 * dScale, isWorld: true }
      );
      res = await runTraj('delta-cycle', dTraj, (data) => {
        if (data.ikResult && data.ikResult.success) {
          const a = data.ikResult.anglesDeg;
          this.deltaRobot.setJointAngles(a.m1, a.m2, a.m3, a.yaw);
          this.updateDeltaUIFromModel();
        }
      });
      if ((res && res.cancelled) || isAborted()) return;

      // Release 1 bar into slot (barNum / 4)
      this.physicsEngine.releaseDeltaGripper();
      await sleep(10);
      if (isAborted()) return;

      // 6. Quick lift after drop ready for the next bar immediately
      const postDropLift = new THREE.Vector3(1.20, 1.15, 0.0);
      dTraj = this.trajectoryPlanner.createMOVL(
        { x: dropPos.x, y: dropPos.y, z: dropPos.z },
        { x: postDropLift.x, y: postDropLift.y, z: postDropLift.z },
        this.deltaKinematics,
        { duration: 0.08 * dScale, isWorld: true }
      );
      await runTraj('delta-cycle', dTraj, (data) => {
        if (data.ikResult && data.ikResult.success) {
          const a = data.ikResult.anglesDeg;
          this.deltaRobot.setJointAngles(a.m1, a.m2, a.m3, a.yaw);
          this.updateDeltaUIFromModel();
        }
      });
    } catch (err) {
      console.error("Lỗi trong chu trình Delta:", err);
    } finally {
      this.isDeltaBusy = false;
      this.isSingleCycleRunning = this.isDeltaBusy || this.isArmBusy;
    }
  }

  /**
   * Independent Worker 2: High-Speed 6-DOF Robot Arm
   * Picks a completed 4-pack at Sensor DI_11 (X = -1.25m, Z = +0.18m) and stacks it into the Rear Carton Box (X = -1.25m, Z = -1.45m).
   * Runs simultaneously whenever a 4-pack arrives at DI_11, without waiting for the Delta Robot!
   */
  async runArmBoxPackStep(collatedPack = null) {
    if (this.isArmBusy) return;
    this.isArmBusy = true;
    this.isSingleCycleRunning = true;

    const statusEl = document.getElementById('cycle-status-text');
    const lineStateEl = document.getElementById('telemetry-line-state');

    const isAborted = () => (
      !this.systemPower ||
      this.isSafetyInterlockTripped ||
      this.isCollisionActive ||
      (this.hardwareBus && this.hardwareBus.emergencyStop)
    );

    const runTraj = (id, traj, onUpdateJoints) => {
      return new Promise((resolve) => {
        if (isAborted()) {
          resolve({ cancelled: true });
          return;
        }
        this.trajectoryPlanner.startTrajectory(id, traj, {
          onUpdate: (evalData) => {
            if (isAborted()) return;
            if (onUpdateJoints) onUpdateJoints(evalData);
          },
          onComplete: (evalData) => resolve(evalData || {})
        });
      });
    };
    const sleep = (ms) => new Promise(res => setTimeout(res, ms));
    const rateBoost = Math.max(0.75, Math.min(1.65, this.factory.infeedRateMultiplier || 1.0));
    const aScale = 1.0 / rateBoost;

    try {
      if (isAborted()) return;
      if (!collatedPack) {
        collatedPack = this.physicsEngine.findReachableCollatedPack(-1.25, 0.35);
      }
      if (!collatedPack) {
        collatedPack = this.factory.spawnCollatedPack(-1.25, 4);
        await sleep(15);
      }
      if (isAborted()) return;

      if (statusEl) statusEl.textContent = "[SONG SONG] 6-DOF ARM HÚT CỤM 4 THANH -> ĐÓNG THÙNG";
      if (lineStateEl) { lineStateEl.textContent = "PARALLEL: ARM PACKING"; lineStateEl.style.color = "#facc15"; }

      // 1. Move above pickup station
      const armPickPre = { x: -1.25, y: 1.16, z: 0.18, rx: 0, ry: 0, rz: 0 };
      const ikPickPre = this.armKinematics.inverseKinematics(armPickPre, this.arm6DOF.getJointAngles(), true);
      if (ikPickPre.success) {
        const armTraj = this.trajectoryPlanner.createMOVJ(
          this.arm6DOF.getJointAngles(),
          ikPickPre.jointsDeg,
          { duration: 0.28 * aScale, degree: 5 }
        );
        const res1 = await runTraj('arm-cycle', armTraj, (data) => {
          const q = data.joints;
          this.arm6DOF.setJointAngles(q.s, q.l, q.u, q.r, q.b, q.t);
          this.updateArmUIFromModel();
        });
        if ((res1 && res1.cancelled) || isAborted()) return;
      }

      // 2. Descend linearly to grip
      const armPickPose = { x: -1.25, y: 0.91, z: 0.18, rx: 0, ry: 0, rz: 0 };
      const armPickTraj = this.trajectoryPlanner.createMOVL(
        armPickPre,
        armPickPose,
        this.armKinematics,
        { duration: 0.18 * aScale, degree: 5, isWorld: true, initialJoints: this.arm6DOF.getJointAngles() }
      );
      let res = await runTraj('arm-cycle', armPickTraj, (data) => {
        if (data.ikResult && data.ikResult.success) {
          const q = data.ikResult.jointsDeg;
          this.arm6DOF.setJointAngles(q.s, q.l, q.u, q.r, q.b, q.t);
          this.updateArmUIFromModel();
        }
      });
      if ((res && res.cancelled) || isAborted()) return;

      this.physicsEngine.attachArmGripper(collatedPack);
      await sleep(25);
      if (isAborted()) {
        this.physicsEngine.releaseArmGripper();
        return;
      }

      // 3. Lift vertically
      const armLiftPose = { x: -1.25, y: 1.24, z: 0.18, rx: 0, ry: 0, rz: 0 };
      const armLiftTraj = this.trajectoryPlanner.createMOVL(
        armPickPose,
        armLiftPose,
        this.armKinematics,
        { duration: 0.18 * aScale, degree: 5, isWorld: true, initialJoints: this.arm6DOF.getJointAngles() }
      );
      res = await runTraj('arm-cycle', armLiftTraj, (data) => {
        if (data.ikResult && data.ikResult.success) {
          const q = data.ikResult.jointsDeg;
          this.arm6DOF.setJointAngles(q.s, q.l, q.u, q.r, q.b, q.t);
          this.updateArmUIFromModel();
        }
      });
      if ((res && res.cancelled) || isAborted()) return;

      // 4. Swing 180° behind the robot arm to Carton Box on Rear Conveyor 3 (at X = -1.25m, Z = -1.45m)
      if (this.factory.cartonBoxes.filter(b => !b.full).length === 0) {
        this.factory.spawnEmptyBox(-1.25);
      }

      const boxPrePose = { x: -1.25, y: 1.20, z: -1.45, rx: 0, ry: 0, rz: 0 };
      const ikBoxPre = this.armKinematics.inverseKinematics(boxPrePose, this.arm6DOF.getJointAngles(), true);
      if (ikBoxPre.success) {
        const armSwingTraj = this.trajectoryPlanner.createMOVJ(
          this.arm6DOF.getJointAngles(),
          ikBoxPre.jointsDeg,
          { duration: 0.34 * aScale, degree: 5 }
        );
        res = await runTraj('arm-cycle', armSwingTraj, (data) => {
          const q = data.joints;
          this.arm6DOF.setJointAngles(q.s, q.l, q.u, q.r, q.b, q.t);
          this.updateArmUIFromModel();
        });
        if ((res && res.cancelled) || isAborted()) return;
      }

      // 5. Descend to safe release height above carton box opening for Gravity Drop
      const currentLayer = (this.physicsEngine.armVacuum.currentBoxLayers % 2) + 1;
      const placeY = 0.98; // Release cleanly above box rim (0.91m) with Gravity Drop
      const boxPlacePose = { x: -1.25, y: placeY, z: -1.45, rx: 0, ry: 0, rz: 0 };
      const armPlaceTraj = this.trajectoryPlanner.createMOVL(
        boxPrePose,
        boxPlacePose,
        this.armKinematics,
        { duration: 0.20 * aScale, degree: 5, isWorld: true, initialJoints: this.arm6DOF.getJointAngles() }
      );
      res = await runTraj('arm-cycle', armPlaceTraj, (data) => {
        if (data.ikResult && data.ikResult.success) {
          const q = data.ikResult.jointsDeg;
          this.arm6DOF.setJointAngles(q.s, q.l, q.u, q.r, q.b, q.t);
          this.updateArmUIFromModel();
        }
      });
      if ((res && res.cancelled) || isAborted()) return;

      // Release gripper into active carton box
      const activeBox = this.factory.cartonBoxes.find(b => !b.full) || this.factory.cartonBoxes[0];
      this.physicsEngine.releaseArmGripper(activeBox ? activeBox.mesh : null);
      await sleep(25);
      if (isAborted()) return;

      if (this.hardwareBus) {
        this.hardwareBus.triggerOT3Pulse();
      }

      // 6. Retract & swing straight back to above DI_11 pickup station so it is immediately ready for the next 4-pack!
      const armRetractTraj = this.trajectoryPlanner.createMOVL(
        boxPlacePose,
        boxPrePose,
        this.armKinematics,
        { duration: 0.18 * aScale, degree: 5, isWorld: true, initialJoints: this.arm6DOF.getJointAngles() }
      );
      res = await runTraj('arm-cycle', armRetractTraj, (data) => {
        if (data.ikResult && data.ikResult.success) {
          const q = data.ikResult.jointsDeg;
          this.arm6DOF.setJointAngles(q.s, q.l, q.u, q.r, q.b, q.t);
          this.updateArmUIFromModel();
        }
      });
      if ((res && res.cancelled) || isAborted()) return;

      if (ikPickPre.success) {
        const armReturnTraj = this.trajectoryPlanner.createMOVJ(
          this.arm6DOF.getJointAngles(),
          ikPickPre.jointsDeg,
          { duration: 0.30 * aScale, degree: 5 }
        );
        await runTraj('arm-cycle', armReturnTraj, (data) => {
          const q = data.joints;
          this.arm6DOF.setJointAngles(q.s, q.l, q.u, q.r, q.b, q.t);
          this.updateArmUIFromModel();
        });
      }

      if (currentLayer === 2 && this.vipaHMI) {
        this.vipaHMI.totalBoxesShipped++;
        this.vipaHMI.totalBarsPacked += 8;
      }

      if (statusEl) statusEl.textContent = `HOÀN TẤT XẾP LỚP ${currentLayer}/2 VÀO THÙNG CARTON!`;
    } catch (err) {
      console.error("Lỗi trong chu trình 6-DOF Arm:", err);
    } finally {
      this.isArmBusy = false;
      this.isSingleCycleRunning = this.isDeltaBusy || this.isArmBusy;
    }
  }

  /**
   * Kích hoạt chu trình tự động vẽ chữ T cho Robot Arm 6-DOF
   * Hiển thị quỹ đạo vết di chuyển 3D (Cyan) và vẽ đồ thị chữ T trên Telemetry / MATLAB
   */
  async runAutoLetterT() {
    return runAutoLetterTJob(this);
  }

  /**
   * Launches BOTH robots concurrently when triggered manually or by PLC!
   */
  async runSinglePickAndPlaceCycle() {
    if (this.isSafetyInterlockTripped) {
      alert("⚠ CẢNH BÁO AN TOÀN LAO ĐỘNG (SAFETY INTERLOCK):\nCửa bảo trì đang mở hoặc mạch an toàn chưa được Reset!\nHãy đóng kín cánh cửa rào và nhấn nút [RESET FAULT / ROBOT ON].");
      return;
    }

    if (!this.systemPower) {
      const btnPower = document.getElementById('btn-system-power');
      if (btnPower) btnPower.click();
    }

    // Ensure all 3 conveyors are running continuously and synchronously!
    this.factory.conveyor1Active = true;
    this.factory.conveyor2Active = true;
    this.factory.conveyor3Active = true;
    ['1', '2', '3'].forEach(idx => {
      const t = document.getElementById(`toggle-conv${idx}`);
      const l = document.getElementById(`led-conv${idx}`);
      if (t) t.checked = true;
      if (l) l.classList.add('active');
    });
    if (this.hardwareBus) {
      this.hardwareBus.setDO(0, 1);
      this.hardwareBus.setDO(3, 1);
    }

    // Launch both robots simultaneously in parallel (Promise.all) without waiting for each other!
    const tasks = [];
    if (!this.isDeltaBusy) {
      tasks.push((async () => {
        for (let i = 0; i < 4; i++) {
          await this.runDeltaCollateStep();
        }
      })());
    }
    if (!this.isArmBusy) {
      tasks.push(this.runArmBoxPackStep());
    }
    await Promise.all(tasks);
  }

  updateKinematicsLabUI() {
    if (!this.armKinematics || !this.arm6DOF) return;
    const pane = document.getElementById('tab-pane-kinematics');
    if (pane && !pane.classList.contains('active')) return;

    // 1. Forward Kinematics 4x4 Matrix
    const fk = this.armKinematics.forwardKinematics(this.arm6DOF.getJointAngles());
    const m = fk.formattedMatrix;
    for (let r = 0; r < 3; r++) {
      for (let c = 0; c < 4; c++) {
        const cell = document.getElementById(`mat-${r}-${c}`);
        if (cell) cell.textContent = m[r][c];
      }
    }

    const rxEl = document.getElementById('lab-euler-rx');
    const ryEl = document.getElementById('lab-euler-ry');
    const rzEl = document.getElementById('lab-euler-rz');
    if (rxEl) rxEl.textContent = `${fk.orientation_deg.rx.toFixed(1)}°`;
    if (ryEl) ryEl.textContent = `${fk.orientation_deg.ry.toFixed(1)}°`;
    if (rzEl) rzEl.textContent = `${fk.orientation_deg.rz.toFixed(1)}°`;

    // 2. Physics & Sensor Status
    if (this.physicsEngine) {
      const ledDeltaVac = document.getElementById('led-delta-vacuum');
      const lblDeltaVac = document.getElementById('label-delta-vac-kpa');
      if (ledDeltaVac) {
        if (this.physicsEngine.deltaVacuum.active) {
          ledDeltaVac.classList.add('active');
          if (lblDeltaVac) lblDeltaVac.textContent = "-85.0 kPa (ON)";
        } else {
          ledDeltaVac.classList.remove('active');
          if (lblDeltaVac) lblDeltaVac.textContent = "0.0 kPa (OFF)";
        }
      }

      const ledArmVac = document.getElementById('led-arm-vacuum');
      const lblArmVac = document.getElementById('label-arm-vac-kpa');
      if (ledArmVac) {
        if (this.physicsEngine.armVacuum.active) {
          ledArmVac.classList.add('active');
          if (lblArmVac) lblArmVac.textContent = "-90.0 kPa (ON)";
        } else {
          ledArmVac.classList.remove('active');
          if (lblArmVac) lblArmVac.textContent = "0.0 kPa (OFF)";
        }
      }

      const lblBoxLayers = document.getElementById('label-box-layers');
      const ledBoxLayers = document.getElementById('led-box-layers');
      if (lblBoxLayers) {
        const layers = this.physicsEngine.armVacuum.currentBoxLayers;
        lblBoxLayers.textContent = `Lớp: ${layers} / 2`;
        if (ledBoxLayers) {
          if (layers >= 2) ledBoxLayers.classList.add('active');
          else ledBoxLayers.classList.remove('active');
        }
      }

      const lblVisionCount = document.getElementById('label-vision-queue-count');
      if (lblVisionCount) {
        lblVisionCount.textContent = `${this.physicsEngine.visionQueue.length} Vật thể`;
      }
    }
  }

  // ===========================================================================
  // PHASE 3: HARDWARE BUS & ETHERNET SERVER INITIALIZATION & UI BINDINGS
  // ===========================================================================

  initHardwareBus() {
    this.hardwareBus = new MP3300HardwareBus();
    this.ethernetServer = new YRC1000EthernetServer(this.hardwareBus, this.arm6DOF);
  }

  initHardwareUIEventListeners() {
    // 1. Build 32-Bit DI & DO Matrix Bit Cells (MPE720 RAM)
    const di0_15 = document.getElementById('matrix-di-0-15');
    const di16_31 = document.getElementById('matrix-di-16-31');
    const do0_15 = document.getElementById('matrix-do-0-15');
    const do16_31 = document.getElementById('matrix-do-16-31');

    if (di0_15 && di0_15.children.length === 0) {
      for (let i = 0; i < 16; i++) {
        const cell = document.createElement('div');
        cell.className = 'bit-cell';
        cell.id = `bit-di-${i}`;
        cell.textContent = i < 10 ? `0${i}` : `${i}`;
        cell.title = `DI_${i < 10 ? '0' + i : i} (Bit ${i})`;
        di0_15.appendChild(cell);
      }
    }

    if (di16_31 && di16_31.children.length === 0) {
      for (let i = 16; i < 32; i++) {
        const cell = document.createElement('div');
        cell.className = 'bit-cell';
        cell.id = `bit-di-${i}`;
        cell.textContent = `${i}`;
        cell.title = `DI_${i} (Bit ${i})`;
        di16_31.appendChild(cell);
      }
    }

    if (do0_15 && do0_15.children.length === 0) {
      for (let i = 0; i < 16; i++) {
        const cell = document.createElement('div');
        cell.className = 'bit-cell';
        cell.id = `bit-do-${i}`;
        cell.textContent = i < 10 ? `0${i}` : `${i}`;
        cell.title = `DO_${i < 10 ? '0' + i : i} (Bit ${i})`;
        do0_15.appendChild(cell);
      }
    }

    if (do16_31 && do16_31.children.length === 0) {
      for (let i = 16; i < 32; i++) {
        const cell = document.createElement('div');
        cell.className = 'bit-cell';
        cell.id = `bit-do-${i}`;
        cell.textContent = `${i}`;
        cell.title = `DO_${i} (Bit ${i})`;
        do16_31.appendChild(cell);
      }
    }

    // 2. Emergency Stop Button (Twist to reset)
    const estop = document.getElementById('ctrl-estop');
    const statusBadge = document.getElementById('ctrl-panel-status');
    if (estop) {
      estop.addEventListener('click', () => {
        const newStop = !this.hardwareBus.emergencyStop;
        this.hardwareBus.setEmergencyStop(newStop);
        estop.classList.toggle('locked', newStop);

        if (newStop) {
          if (this.trajectoryPlanner) {
            this.trajectoryPlanner.stopAll();
          }
          this.isDeltaBusy = false;
          this.isArmBusy = false;
          this.isSingleCycleRunning = false;
          this.autoDemoMode = false;
          const btnAutoDemo = document.getElementById('btn-auto-demo-cycle');
          const ledAutoDemo = document.getElementById('led-auto-demo');
          const labelAutoDemo = document.getElementById('label-auto-demo');
          if (btnAutoDemo) btnAutoDemo.classList.remove('running');
          if (ledAutoDemo) ledAutoDemo.classList.remove('active');
          if (labelAutoDemo) labelAutoDemo.textContent = "AUTO DEMO CYCLE";

          if (statusBadge) {
            statusBadge.textContent = 'E-STOPPED';
            statusBadge.className = 'badge badge-danger';
          }
          if (this.physicsEngine) {
            this.physicsEngine.releaseDeltaGripper();
            this.physicsEngine.releaseArmGripper();
            this.physicsEngine.clearFallingObjects();
          }
        } else {
          if (statusBadge) {
            statusBadge.textContent = 'READY';
            statusBadge.className = 'badge badge-success';
          }
        }
      });
    }

    // 3. Industrial Round Push Buttons (Momentary: DI_00..DI_03)
    const attachMomentary = (btnId, diIndex) => {
      const el = document.getElementById(btnId);
      if (!el) return;
      const press = (e) => {
        e.preventDefault();
        el.classList.add('active');
        this.hardwareBus.setDI(diIndex, 1);
      };
      const release = (e) => {
        e.preventDefault();
        el.classList.remove('active');
        this.hardwareBus.setDI(diIndex, 0);
      };
      el.addEventListener('mousedown', press);
      el.addEventListener('mouseup', release);
      el.addEventListener('mouseleave', release);
      el.addEventListener('touchstart', press, { passive: false });
      el.addEventListener('touchend', release, { passive: false });
    };

    attachMomentary('btn-ctrl-start', 0);
    attachMomentary('btn-ctrl-stop', 1);
    attachMomentary('btn-ctrl-servoon', 2);
    attachMomentary('btn-ctrl-roboton', 3);

    const elRobotOn = document.getElementById('btn-ctrl-roboton');
    if (elRobotOn) {
      elRobotOn.addEventListener('click', () => {
        if (this.isSafetyInterlockTripped && this.factory && this.factory.safetyGateClosed) {
          this.resetSafetyInterlock();
        }
      });
    }

    // 4. Rotary Mode Key Switch (TEACH = 0, PLAY = 1, REMOTE = 2)
    const modeConfigs = [
      { id: 'btn-mode-teach', mode: 0, lbl: 'lbl-mode-teach' },
      { id: 'btn-mode-play', mode: 1, lbl: 'lbl-mode-play' },
      { id: 'btn-mode-remote', mode: 2, lbl: 'lbl-mode-remote' }
    ];

    const applyRobotMode = (targetMode) => {
      this.hardwareBus.setRobotMode(targetMode);

      modeConfigs.forEach(item => {
        const btn = document.getElementById(item.id);
        const lbl = document.getElementById(item.lbl);
        const isAct = (item.mode === targetMode);
        if (btn) btn.classList.toggle('active', isAct);
        if (lbl) lbl.classList.toggle('active', isAct);
      });

      // Update Sniffer safety lockout banner
      const banner = document.getElementById('sniffer-mode-banner');
      const bannerText = document.getElementById('sniffer-mode-banner-text');
      if (banner && bannerText) {
        if (targetMode === 2) {
          banner.style.background = 'rgba(34, 197, 94, 0.15)';
          banner.style.borderColor = '#22c55e';
          banner.style.color = '#4ade80';
          bannerText.textContent = 'SAFETY INTERLOCK: MODE REMOTE (2) - ETHERNET READY';
        } else {
          banner.style.background = 'rgba(239, 68, 68, 0.15)';
          banner.style.borderColor = '#ef4444';
          banner.style.color = '#f87171';
          bannerText.textContent = `⚠ SAFETY LOCKOUT: MODE ${this.hardwareBus.getRobotModeName()} (${targetMode}) - UDP COMMANDS REJECTED (ERROR 0x1F)!`;
        }
      }
    };

    modeConfigs.forEach(item => {
      const btn = document.getElementById(item.id);
      if (btn) {
        btn.addEventListener('click', () => applyRobotMode(item.mode));
      }
    });

    // 5. Control Panel 8 Metal Switches (DI_16..DI_23)
    for (let i = 0; i < 8; i++) {
      const sw = document.getElementById(`sw-cp-${i}`);
      if (sw) {
        sw.addEventListener('click', () => {
          const isAct = sw.classList.toggle('active');
          this.hardwareBus.setDI(16 + i, isAct ? 1 : 0);
        });
      }
    }

    // 6. I/O Box 8 Metal Switches (DI_24..DI_31)
    for (let i = 0; i < 8; i++) {
      const sw = document.getElementById(`sw-box-${i}`);
      if (sw) {
        sw.addEventListener('click', () => {
          const isAct = sw.classList.toggle('active');
          this.hardwareBus.setDI(24 + i, isAct ? 1 : 0);

          // I0 (DI_24): Reset Box Layers & Cycle Count
          if (i === 0 && isAct) {
            this.hardwareBus.plcBoxClusterCount = 0;
            if (this.physicsEngine) {
              this.physicsEngine.armVacuum.currentBoxLayers = 0;
            }
            const lbl = document.getElementById('label-box-layers');
            if (lbl) lbl.textContent = 'Lớp: 0 / 2';
          }

          // I7 (DI_31): Trigger MSG-SNDE Ethernet Command
          if (i === 7 && isAct) {
            this.ethernetServer.sendMSGSNDE('READ_STATUS');
          }
        });
      }
    }

    // 7. Built-in PLC Logic Switch
    const togglePLC = document.getElementById('toggle-builtin-plc');
    if (togglePLC) {
      togglePLC.addEventListener('change', (e) => {
        this.hardwareBus.builtinPLCOn = e.target.checked;
        const badge = document.getElementById('plc-state-badge');
        if (badge) {
          if (!e.target.checked) {
            badge.textContent = 'MANUAL';
            badge.className = 'badge badge-secondary';
          } else {
            badge.textContent = this.hardwareBus.plcState;
            badge.className = 'badge badge-success';
          }
        }
      });
    }

    // 8. UDP Sniffer Command Buttons & Console Output
    const bindUDPBtn = (btnId, cmdType) => {
      const btn = document.getElementById(btnId);
      if (btn) {
        btn.addEventListener('click', async () => {
          btn.style.opacity = '0.6';
          try {
            await this.ethernetServer.sendMSGSNDE(cmdType);
          } finally {
            setTimeout(() => { btn.style.opacity = '1.0'; }, 100);
          }
        });
      }
    };

    bindUDPBtn('btn-udp-servoon', 'SERVO_ON');
    bindUDPBtn('btn-udp-servooff', 'SERVO_OFF');
    bindUDPBtn('btn-udp-startjob', 'START_JOB');
    bindUDPBtn('btn-udp-status', 'READ_STATUS');
    bindUDPBtn('btn-udp-pos', 'READ_POS');
    bindUDPBtn('btn-udp-msgsnde', 'READ_STATUS');

    const btnClearConsole = document.getElementById('btn-udp-clear');
    if (btnClearConsole) {
      btnClearConsole.addEventListener('click', () => {
        const consoleEl = document.getElementById('udp-hex-console');
        if (consoleEl) consoleEl.innerHTML = '';
      });
    }

    // Packet sniffer real-time trace subscriber
    this.ethernetServer.subscribeHistory((item) => {
      this.renderHexPacketLog(item);
    });

    // Send an initial handshake packet to populate the sniffer
    setTimeout(() => {
      this.ethernetServer.sendMSGSNDE('READ_STATUS');
    }, 600);
  }

  renderHexPacketLog(item) {
    const consoleEl = document.getElementById('udp-hex-console');
    if (!consoleEl) return;

    const entry = document.createElement('div');
    entry.className = 'hex-monitor-entry';

    const header = document.createElement('div');
    header.style.display = 'flex';
    header.style.justifyContent = 'space-between';
    header.style.marginBottom = '4px';

    const tagSpan = document.createElement('span');
    if (item.isError) {
      tagSpan.className = 'hex-tag-err';
      tagSpan.textContent = `[YERC REQ #${item.reqId}] ${item.commandNo} -> ${item.status}`;
    } else {
      tagSpan.className = 'hex-tag-ans';
      tagSpan.textContent = `[YERC REQ #${item.reqId}] ${item.commandNo} -> ${item.status}`;
    }

    const timeSpan = document.createElement('span');
    timeSpan.style.color = 'var(--text-dim)';
    timeSpan.style.fontSize = '8.5px';
    timeSpan.textContent = item.timestamp;

    header.appendChild(tagSpan);
    header.appendChild(timeSpan);
    entry.appendChild(header);

    const msgDiv = document.createElement('div');
    msgDiv.style.color = item.isError ? '#f87171' : '#94a3b8';
    msgDiv.style.marginBottom = '4px';
    msgDiv.textContent = item.logMsg;
    entry.appendChild(msgDiv);

    const hexDump = document.createElement('pre');
    hexDump.style.margin = '0';
    hexDump.style.padding = '4px';
    hexDump.style.background = 'rgba(0,0,0,0.45)';
    hexDump.style.borderRadius = '3px';
    hexDump.style.fontSize = '8.5px';
    hexDump.style.overflowX = 'auto';
    hexDump.textContent = `REQUEST:\n${item.requestHex}\n\nANSWER:\n${item.answerHex}`;
    entry.appendChild(hexDump);

    if (consoleEl.firstChild) {
      consoleEl.insertBefore(entry, consoleEl.firstChild);
    } else {
      consoleEl.appendChild(entry);
    }

    while (consoleEl.children.length > 25) {
      consoleEl.removeChild(consoleEl.lastChild);
    }
  }

  updateHardwareUI() {
    if (!this.hardwareBus) return;
    const pane = document.getElementById('tab-pane-control-panel');
    if (pane && !pane.classList.contains('active')) return;

    // 1. Bit matrix & Hex strings
    const iw0 = this.hardwareBus.readIW(0);
    const iw1 = this.hardwareBus.readIW(1);
    const diHex = ((iw1 << 16) | iw0) >>> 0;
    const diHexStr = '0x' + diHex.toString(16).toUpperCase().padStart(8, '0');
    const lblDiHex = document.getElementById('label-di-hex');
    if (lblDiHex) lblDiHex.textContent = diHexStr;

    const ow0 = this.hardwareBus.readOW(0);
    const ow1 = this.hardwareBus.readOW(1);
    const doHex = ((ow1 << 16) | ow0) >>> 0;
    const doHexStr = '0x' + doHex.toString(16).toUpperCase().padStart(8, '0');
    const lblDoHex = document.getElementById('label-do-hex');
    if (lblDoHex) lblDoHex.textContent = doHexStr;

    for (let i = 0; i < 32; i++) {
      const bitDi = document.getElementById(`bit-di-${i}`);
      if (bitDi) {
        bitDi.classList.toggle('active-di', this.hardwareBus.getDI(i) === 1);
      }
      const bitDo = document.getElementById(`bit-do-${i}`);
      if (bitDo) {
        bitDo.classList.toggle('active-do', this.hardwareBus.getDO(i) === 1);
      }
    }

    // 2. Control panel lamps Q0..Q7 (DO_16..DO_23)
    for (let i = 0; i < 8; i++) {
      const lamp = document.getElementById(`lamp-cp-${i}`);
      if (lamp) {
        lamp.classList.toggle('active', this.hardwareBus.getDO(16 + i) === 1);
      }
    }

    // 3. I/O Box lamps Q0..Q7 (DO_24..DO_31)
    for (let i = 0; i < 8; i++) {
      const lamp = document.getElementById(`lamp-box-${i}`);
      if (lamp) {
        lamp.classList.toggle('active', this.hardwareBus.getDO(24 + i) === 1);
      }
    }

    // 4. PLC State Badge
    const plcBadge = document.getElementById('plc-state-badge');
    if (plcBadge && this.hardwareBus.builtinPLCOn) {
      plcBadge.textContent = this.hardwareBus.plcState;
      if (this.hardwareBus.plcState === 'IDLE') {
        plcBadge.className = 'badge';
      } else if (this.hardwareBus.plcState.includes('RUNNING')) {
        plcBadge.className = 'badge badge-success';
      } else if (this.hardwareBus.plcState.includes('PACKING') || this.hardwareBus.plcState.includes('PICK')) {
        plcBadge.className = 'badge badge-primary';
      } else {
        plcBadge.className = 'badge badge-warning';
      }
    }
  }

  // ===========================================================================
  // PHASE 4 & 5: ENGINEERING WORKSTATIONS & VIPA HMI / TELEMETRY
  // ===========================================================================

  initEngineeringWorkstations() {
    this.teachPendant = new TeachPendantStation(this);
    // Initialize Teach Pendant floating DOM immediately on startup
    this.teachPendant.initDOM();
    this.teachPendant.hide(); // Ẩn mặc định, chỉ hiện khi người dùng bấm mở

    this.deltaPendant = new DeltaTeachPendantStation(this);
    // Initialize Delta Teach Pendant floating DOM immediately on startup
    this.deltaPendant.initDOM();
    this.deltaPendant.hide();

    this.mpeStudio = new MPE720StudioStation(this);
    this.csharpStudio = new CSharpPCStudioStation(this);
    this.vipaHMI = new VIPAHMIScreen(this);
    this.telemetryGraphs = new TelemetryAndGraphs(this);
    this.faultPanel = new FaultInjectionPanel(this);
    this.matlabPlotter = new MatlabKinematicsPlotter(this);
    this.matlabPlotter.initDOM();
    this.oeeCalculator = new OEECalculator(this);
    this.oeeCalculator.renderDashboardDOM();

    // Initialize AGV / AMR Autonomous Logistics & Virtual Warehouse Controller
    this.agvController = new AGVController(this, this.factory);
    if (this.factory) {
      this.factory.agvController = this.agvController;
    }
    this.agvController.updateMonitorUI();

    // Initialize Industrial Machine Vision Studio & Top-Down PIP Inspection Controller (Cognex #CAM01)
    this.visionController = new MachineVisionController(this);

    const selAgvCap = document.getElementById('select-agv-capacity');
    if (selAgvCap) {
      selAgvCap.addEventListener('change', (e) => {
        if (this.agvController) {
          this.agvController.setCapacity(parseInt(e.target.value, 10));
        }
      });
    }

    const btnAgvLoadBox = document.getElementById('btn-agv-load-box');
    if (btnAgvLoadBox) {
      btnAgvLoadBox.addEventListener('click', () => {
        if (this.agvController) {
          this.agvController.spawnDirectBoxOntoAGV();
        }
      });
    }

    const btnAgvForceDispatch = document.getElementById('btn-agv-force-dispatch');
    if (btnAgvForceDispatch) {
      btnAgvForceDispatch.addEventListener('click', () => {
        if (this.agvController) {
          this.agvController.forceDispatch();
        }
      });
    }

    // Shift Reset / Clear OEE Data Buttons
    ['btn-reset-oee-shift', 'btn-reset-oee-widget'].forEach(btnId => {
      const btn = document.getElementById(btnId);
      if (btn) {
        btn.addEventListener('click', () => {
          if (this.oeeCalculator) {
            this.oeeCalculator.resetShift();
          }
        });
      }
    });
    this.activeWorkstation = 'default';

    // Mount Chaos Engineering Fault Injection Panel
    const faultContainer = document.getElementById('fault-injection-container');
    if (faultContainer) {
      this.faultPanel.initDOM(faultContainer);
    }

    const wsButtons = [
      { id: 'ws-btn-default', mode: 'default' },
      { id: 'ws-btn-pendant', mode: 'pendant' },
      { id: 'ws-btn-delta', mode: 'delta' },
      { id: 'ws-btn-mpe720', mode: 'mpe720' },
      { id: 'ws-btn-csharp', mode: 'csharp' },
      { id: 'ws-btn-vipa', mode: 'vipa' }
    ];

    wsButtons.forEach(btnInfo => {
      const btn = document.getElementById(btnInfo.id);
      if (btn) {
        btn.addEventListener('click', () => {
          this.switchWorkstation(btnInfo.mode);
        });
      }
    });

    // Quick Floating Toggle for Yaskawa Robot Arm Pendant
    const btnQuickFloatPendant = document.getElementById('btn-quick-float-pendant');
    if (btnQuickFloatPendant) {
      btnQuickFloatPendant.addEventListener('click', () => {
        if (this.teachPendant) {
          this.teachPendant.toggleFloatWindow();
        }
      });
    }

    // Quick Floating Toggle for Delta Pendant
    const btnQuickFloatDelta = document.getElementById('btn-quick-float-delta');
    if (btnQuickFloatDelta) {
      btnQuickFloatDelta.addEventListener('click', () => {
        if (this.deltaPendant) {
          this.deltaPendant.toggleFloatWindow();
        }
      });
    }

    // Workstation Switcher Dropdown Menu Trigger & Auto-Close
    const wsDropdownTrigger = document.getElementById('ws-dropdown-trigger');
    const wsDropdownContainer = document.getElementById('ws-dropdown-container');
    const wsDropdownMenu = document.getElementById('ws-dropdown-menu');

    if (wsDropdownTrigger && wsDropdownContainer) {
      wsDropdownTrigger.addEventListener('click', (e) => {
        e.stopPropagation();
        wsDropdownContainer.classList.toggle('open');
        const extraMetricsContainer = document.getElementById('extra-metrics-container');
        if (extraMetricsContainer) extraMetricsContainer.classList.remove('open');
      });
    }

    if (wsDropdownMenu && wsDropdownContainer) {
      wsDropdownMenu.addEventListener('click', () => {
        wsDropdownContainer.classList.remove('open');
      });
    }

    // Extra Metrics Popup Toggle
    const extraMetricsTrigger = document.getElementById('btn-extra-metrics-trigger');
    const extraMetricsContainer = document.getElementById('extra-metrics-container');

    if (extraMetricsTrigger && extraMetricsContainer) {
      extraMetricsTrigger.addEventListener('click', (e) => {
        e.stopPropagation();
        extraMetricsContainer.classList.toggle('open');
        if (wsDropdownContainer) wsDropdownContainer.classList.remove('open');
      });
    }

    // Global Click outside to close open dropdowns
    document.addEventListener('click', () => {
      if (wsDropdownContainer) wsDropdownContainer.classList.remove('open');
      if (extraMetricsContainer) extraMetricsContainer.classList.remove('open');
    });

    // Topbar Collapse / Re-Open Toggle (Hide to view full 3D)
    const btnTopbarCollapse = document.getElementById('btn-topbar-collapse');
    const topbarUnified = document.getElementById('scada-topbar-unified');
    const topbarReopenTab = document.getElementById('topbar-reopen-tab');
    const mouseNavHint = document.querySelector('.mouse-nav-hint');

    if (btnTopbarCollapse && topbarUnified && topbarReopenTab) {
      btnTopbarCollapse.addEventListener('click', () => {
        topbarUnified.classList.add('collapsed');
        topbarReopenTab.classList.add('visible');
        if (mouseNavHint) mouseNavHint.classList.add('topbar-collapsed');
      });

      topbarReopenTab.addEventListener('click', () => {
        topbarUnified.classList.remove('collapsed');
        topbarReopenTab.classList.remove('visible');
        if (mouseNavHint) mouseNavHint.classList.remove('topbar-collapsed');
      });
    }

    // Inspector Dock Card 3 Quick Controls
    const btnOpenDeltaPendant = document.getElementById('btn-open-delta-pendant');
    if (btnOpenDeltaPendant) {
      btnOpenDeltaPendant.addEventListener('click', () => {
        if (this.deltaPendant) {
          this.deltaPendant.showFloating();
          this.switchCamera('delta');
        }
      });
    }

    const btnRunDeltaAuto = document.getElementById('btn-run-delta-auto');
    if (btnRunDeltaAuto) {
      btnRunDeltaAuto.addEventListener('click', () => {
        if (this.deltaPendant) {
          this.deltaPendant.toggleAutoRun();
          this.switchCamera('delta');
        }
      });
    }

    // Fault Injection Toggle Button
    const btnFaults = document.getElementById('ws-btn-faults');
    if (btnFaults) {
      btnFaults.addEventListener('click', () => {
        this.faultPanel.toggle();
      });
    }

    // MATLAB Kinematics Plotter Toggles
    const btnMatlabTop = document.getElementById('ws-btn-matlab');
    if (btnMatlabTop) {
      btnMatlabTop.addEventListener('click', () => {
        if (this.matlabPlotter) this.matlabPlotter.open();
      });
    }

    const btnMatlabKin = document.getElementById('btn-open-matlab-plotter');
    if (btnMatlabKin) {
      btnMatlabKin.addEventListener('click', () => {
        if (this.matlabPlotter) this.matlabPlotter.open();
      });
    }

    const btnMatlabArm = document.getElementById('btn-open-matlab-from-arm');
    if (btnMatlabArm) {
      btnMatlabArm.addEventListener('click', () => {
        if (this.matlabPlotter) {
          this.matlabPlotter.selectedRobot = 'arm6dof';
          const sel = document.getElementById('matlab-robot-select');
          if (sel) sel.value = 'arm6dof';
          this.matlabPlotter.open();
        }
      });
    }

    const btnMatlabDelta = document.getElementById('btn-open-matlab-from-delta');
    if (btnMatlabDelta) {
      btnMatlabDelta.addEventListener('click', () => {
        if (this.matlabPlotter) {
          this.matlabPlotter.selectedRobot = 'delta';
          const sel = document.getElementById('matlab-robot-select');
          if (sel) sel.value = 'delta';
          this.matlabPlotter.open();
        }
      });
    }

    // Master Reset Global Button
    const btnMasterReset = document.getElementById('ws-btn-master-reset');
    if (btnMasterReset) {
      btnMasterReset.addEventListener('click', () => {
        this.faultPanel.executeMasterReset();
      });
    }
  }

  switchWorkstation(mode) {
    this.activeWorkstation = mode;

    document.querySelectorAll('.ws-btn').forEach(b => {
      b.classList.toggle('active', b.dataset.ws === mode);
    });

    // Cập nhật biểu tượng và nhãn hiển thị trên nút Dropdown trạm làm việc
    const wsMeta = {
      default: { icon: '🏭', label: '3D FACTORY & SCADA' },
      pendant: { icon: '📱', label: 'STATION 1: TEACH PENDANT' },
      delta: { icon: '⚡', label: 'DELTA 3-DOF STATION' },
      mpe720: { icon: '💻', label: 'STATION 2: MPE720' },
      csharp: { icon: '⚙️', label: 'STATION 3: C# PC & VISION' },
      vipa: { icon: '📊', label: 'STATION 4: VIPA HMI' }
    };
    const currentMeta = wsMeta[mode] || wsMeta.default;
    const activeIconEl = document.getElementById('ws-active-icon');
    const activeLabelEl = document.getElementById('ws-active-label');
    if (activeIconEl) activeIconEl.textContent = currentMeta.icon;
    if (activeLabelEl) activeLabelEl.textContent = currentMeta.label;

    const wsDropdownContainer = document.getElementById('ws-dropdown-container');
    if (wsDropdownContainer) wsDropdownContainer.classList.remove('open');

    const vpContainer = document.getElementById('viewport-container');
    const wsContainer = document.getElementById('workstation-container');
    const dock = document.getElementById('inspector-dock');

    if (mode === 'default') {
      if (vpContainer) vpContainer.style.flex = '0 0 75%';
      if (wsContainer) {
        wsContainer.style.display = 'none';
        wsContainer.innerHTML = '';
      }
      if (dock) dock.style.display = 'flex';
      if (this.teachPendant && this.teachPendant.container === wsContainer) {
        this.teachPendant.setDockMode('floating', false);
      }
      if (this.deltaPendant && this.deltaPendant.container === wsContainer) {
        this.deltaPendant.setDockMode('floating', false);
      }
      if (this.teachPendant && this.teachPendant.floatingHost) {
        this.teachPendant.floatingHost.style.display = 'none';
      }
      if (this.deltaPendant && this.deltaPendant.floatingHost) {
        this.deltaPendant.floatingHost.style.display = 'none';
      }
      this.switchCamera('front');
    } else {
      if (this.teachPendant && this.teachPendant.floatingHost) {
        this.teachPendant.floatingHost.style.display = 'none';
      }
      if (this.deltaPendant && this.deltaPendant.floatingHost) {
        this.deltaPendant.floatingHost.style.display = 'none';
      }
      if (vpContainer) vpContainer.style.flex = '0 0 48%';
      if (dock) dock.style.display = 'none';
      if (wsContainer) {
        wsContainer.style.display = 'flex';
        wsContainer.style.flexDirection = 'column';
        wsContainer.innerHTML = '';

        if (mode === 'pendant') {
          this.teachPendant.initDOM(wsContainer);
          this.teachPendant.setDockMode('docked');
          this.switchCamera('arm');
        } else if (mode === 'delta') {
          this.deltaPendant.initDOM(wsContainer);
          this.deltaPendant.setDockMode('docked');
          this.switchCamera('delta');
        } else if (mode === 'mpe720') {
          this.mpeStudio.initDOM(wsContainer);
          this.switchCamera('overview');
        } else if (mode === 'csharp') {
          this.csharpStudio.initDOM(wsContainer);
          this.switchCamera('cell');
        } else if (mode === 'vipa') {
          // Dual Layout: VIPA HMI Screen (Top) + Real-time Telemetry Oscilloscope (Bottom)
          const hmiContainer = document.createElement('div');
          hmiContainer.style.cssText = 'flex: 1 1 54%; min-height: 400px; display: flex; flex-direction: column; overflow: hidden; border-bottom: 2px solid #334155;';
          
          const telemetryContainer = document.createElement('div');
          telemetryContainer.style.cssText = 'flex: 1 1 46%; min-height: 290px; display: flex; flex-direction: column; overflow: hidden;';

          wsContainer.appendChild(hmiContainer);
          wsContainer.appendChild(telemetryContainer);

          this.vipaHMI.initDOM(hmiContainer);
          this.telemetryGraphs.initDOM(telemetryContainer);
          this.switchCamera('front');
        }
      }
    }

    setTimeout(() => {
      this.onWindowResize();
    }, 60);
  }

  onWindowResize() {
    const w = this.container.clientWidth;
    const h = this.container.clientHeight;
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(w, h);
  }

  /**
   * Collision Detection & Safety Protection Engine
   * Uses THREE.Box3 (0.1m x 0.1m x 0.1m) enclosing both robot TCPs (end-effectors).
   * Verifies whether the TCP intersects the Floor (Y <= 0) or any Carton Boxes on the conveyors.
   * If collision is detected: halts all motions via trajectoryPlanner.stopAll()
   * and reports emergency alarm via vipaHMI.addAlarm('ALM-COL', 'EMERGENCY', 'VA CHẠM: Đầu hút đã đâm vào vật cản!').
   */
  checkCollisions() {
    if (!this.deltaRobot || !this.arm6DOF) return;

    // On-Demand Collision Check: Chỉ tính toán hộp giới hạn THREE.Box3 khi có ít nhất 1 robot đang chuyển động!
    const hasActiveTraj = Boolean(
      this.trajectoryPlanner &&
      this.trajectoryPlanner.activeTrajectories &&
      this.trajectoryPlanner.activeTrajectories.size > 0
    );
    if (!this.isDeltaBusy && !this.isArmBusy && !this.autoDemoMode && !this.isSingleCycleRunning && !hasActiveTraj) {
      return;
    }

    if (!this._colBoxSize) {
      this._colBoxSize = new THREE.Vector3(0.08, 0.08, 0.08);
      this._armTcpBox = new THREE.Box3();
      this._deltaTcpBox = new THREE.Box3();
    }

    // 1. Construct Bounding Box around both robot TCPs using pre-allocated Box3
    const armTcpPos = this.arm6DOF.getTCPWorldPosition();
    const deltaTcpPos = this.deltaRobot.getEndEffectorWorldPosition();

    this._armTcpBox.setFromCenterAndSize(armTcpPos, this._colBoxSize);
    this._deltaTcpBox.setFromCenterAndSize(deltaTcpPos, this._colBoxSize);

    let isColliding = false;

    // A. Mutual collision between the two robots (Delta TCP vs Arm TCP)
    if (this._armTcpBox.intersectsBox(this._deltaTcpBox)) {
      isColliding = true;
    }

    // B. Floor collision check (Y <= 0.05m)
    if (!isColliding) {
      if (armTcpPos.y <= 0.05 || deltaTcpPos.y <= 0.05) {
        isColliding = true;
      }
    }

    // 4. Collision Event Actions
    if (isColliding) {
      if (!this.isCollisionActive) {
        this.isCollisionActive = true;

        if (this.trajectoryPlanner) {
          this.trajectoryPlanner.stopAll();
        }

        this.isArmBusy = false;
        this.isDeltaBusy = false;
        this.isSingleCycleRunning = false;

        if (this.vipaHMI) {
          this.vipaHMI.addAlarm('ALM-COL', 'EMERGENCY', 'VA CHẠM: Phát hiện nguy cơ đâm va giữa 2 robot hoặc va chạm sàn!');
        }
      }
    } else {
      this.isCollisionActive = false;
    }
  }

  animate() {
    requestAnimationFrame(() => this.animate());

    // Giới hạn 30 FPS: giảm ~50% tải CPU/GPU trên máy văn phòng mà chuyển động vẫn mượt
    const frameNow = performance.now();
    if (frameNow - (this._lastFrameTime || 0) < 1000 / 31) return;
    this._lastFrameTime = frameNow;
    if (document.hidden) return;

    const delta = Math.min(this.clock.getDelta(), 0.1);

    // 1. Factory I/O Fly & Pan Camera Movement (Arrow keys / WASD / Space / C / Shift)
    this.updateCameraMovement(delta);

    // 1b. Camera Transition or Auto Rotation
    if (this.isTransitioningCam) {
      this.camTransitionTime = (this.camTransitionTime || 0) + delta;
      this.camera.position.lerp(this.camTargetPos, 0.14);
      this.controls.target.lerp(this.camTargetLookAt, 0.14);

      if (
        this.camTransitionTime > 0.65 ||
        (this.camera.position.distanceTo(this.camTargetPos) < 0.03 &&
          this.controls.target.distanceTo(this.camTargetLookAt) < 0.03)
      ) {
        this.isTransitioningCam = false;
      }
    }

    this.controls.update();

    // 1b. Animate Pivot Reticle Ripple Effect
    if (this.pivotReticle && this.pivotReticle.visible) {
      this.pivotReticleTime = (this.pivotReticleTime || 0) + delta;
      const progress = this.pivotReticleTime / 1.0;
      if (progress >= 1.0) {
        this.pivotReticle.visible = false;
      } else {
        const s = 0.5 + progress * 1.6;
        this.pivotReticle.scale.set(s, s, s);
        const alpha = Math.max(0, 1.0 - progress);
        this.pivotReticle.children.forEach(c => {
          if (c.material) c.material.opacity = alpha;
        });
      }
    }

    // 2. Trajectory & Motion Engine Update
    if (this.trajectoryPlanner) {
      this.trajectoryPlanner.update(delta);
    }

    // 3. Physics, Vision & Sensor Engine Update
    if (this.physicsEngine) {
      this.physicsEngine.update(delta);
    }

    // 4. End-Effector 3D Trajectory Trails
    if (this.trajectoryTrail && this.trajectoryTrail.visible) {
      this.trajectoryTrail.addArmPoint(this.arm6DOF.getTCPWorldPosition());
      this.trajectoryTrail.addDeltaPoint(this.deltaRobot.getEndEffectorWorldPosition());
    }

    // 4b. Real-Time Collision Detection Check
    this.checkCollisions();

    // 5. Throttled UI Telemetry & Kinematics updates (~12.5 Hz / 80ms) to free CPU and prevent DOM thrashing
    this.uiUpdateTimer = (this.uiUpdateTimer || 0) + delta;
    const shouldUpdateUI = this.uiUpdateTimer >= 0.08;
    if (shouldUpdateUI) {
      this.uiUpdateTimer = 0;
      this.updateKinematicsLabUI();
    }

    // 6. Auto Demo Synchronization
    if (this.autoDemoMode) {
      this.updateAutoDemoCycle(delta);
    }

    // 7. Hardware Bus Synchronization & PLC Execution
    if (this.hardwareBus) {
      if (!this.systemPower || this.hardwareBus.emergencyStop) {
        // System is Powered Off or E-Stop pressed: immediately halt all 3 conveyors!
        this.factory.conveyor1Active = false;
        this.factory.conveyor2Active = false;
        this.factory.conveyor3Active = false;
        this.hardwareBus.setDO(0, 0);
        this.hardwareBus.setDO(3, 0);
      } else if (this.hardwareBus.getDO(0) === 1) {
        // DO_00 is ON: Infeed (Belt 1) & Collation (Belt 2) run with synchronized texture animation
        this.factory.conveyor1Active = true;
        this.factory.conveyor2Active = true;
        if (this.hardwareBus.getDO(3) === 1 || this.autoDemoMode) {
          this.factory.conveyor3Active = true;
        }
      } else if (this.autoDemoMode || this.isDeltaBusy || this.isArmBusy) {
        // In Auto Demo or Parallel Cycle mode, keep all 3 conveyors running continuously!
        this.factory.conveyor1Active = true;
        this.factory.conveyor2Active = true;
        this.factory.conveyor3Active = true;
        this.hardwareBus.setDO(0, 1);
      } else if (this.hardwareBus.builtinPLCOn) {
        // DO_00 drives Infeed & Collation belts (J1000 AC Inverter S1)
        this.factory.conveyor1Active = (this.hardwareBus.getDO(0) === 1);
        this.factory.conveyor2Active = (this.hardwareBus.getDO(0) === 1);
        // DO_03 drives Carton Box conveyor
        this.factory.conveyor3Active = (this.hardwareBus.getDO(3) === 1);
      }

      // Synchronize 3D Sensor states into Hardware Bus Inputs
      this.hardwareBus.setDI(11, this.factory.sensorDI11 ? 1 : 0);
      this.hardwareBus.setDI(13, this.factory.sensorDI12 ? 1 : 0);

      // Update Hardware UI (Matrix, LEDs, Hex) at 12.5 Hz
      if (shouldUpdateUI) {
        this.updateHardwareUI();
      }
    }

    // 8. Factory update (belts, products, sensors)
    this.factory.update(delta);

    // 8b. AGV / AMR Autonomous Logistics & Palletizing Update
    if (this.agvController) {
      this.agvController.update(delta);
    }

    // 8c. Industrial Machine Vision Studio PIP Camera & Image Processing Update
    if (this.visionController) {
      this.visionController.update(delta);
    }

    // 9. Update IO Telemetry Panels at 12.5 Hz
    if (shouldUpdateUI) {
      this.updateIOTelemetry();
    }

    // 10. Active Engineering Workstation Real-time Update (Throttled to 12.5 Hz & only when visible)
    if (shouldUpdateUI) {
      if (this.teachPendant) {
        const ypVis = (this.teachPendant.dockMode === 'docked') ||
          (this.teachPendant.floatingHost && this.teachPendant.floatingHost.style.display !== 'none');
        if (ypVis) this.teachPendant.updateLiveReadouts();
      }
      if (this.activeWorkstation === 'mpe720' && this.mpeStudio) {
        this.mpeStudio.updateLiveRungs();
      } else if (this.activeWorkstation === 'csharp' && this.csharpStudio) {
        this.csharpStudio.updateCurrentPositionUI();
      } else if (this.activeWorkstation === 'vipa' && this.vipaHMI) {
        this.vipaHMI.updateScreenData();
      }
    }

    // Telemetry Oscilloscope continuous sampling
    if (this.telemetryGraphs) {
      this.telemetryGraphs.update(delta);
    }

    // MATLAB Kinematics Live Telemetry Sampling
    if (this.matlabPlotter) {
      this.matlabPlotter.update(delta);
    }

    // 11. FPS Tracking
    this.frameCount++;
    const now = performance.now();
    if (now - this.lastFpsTime >= 500) {
      const fps = Math.round((this.frameCount * 1000) / (now - this.lastFpsTime));
      this.fpsValueEl.textContent = `${fps} FPS`;
      this.frameCount = 0;
      this.lastFpsTime = now;
    }

    // 10. High-Performance Render
    this.renderer.render(this.scene, this.camera);
  }
}

const bootstrapApp = () => {
  if (!window.app) {
    window.app = new DigitalTwinApp();
  }
};
if (document.readyState === 'loading') {
  window.addEventListener('DOMContentLoaded', bootstrapApp);
} else {
  bootstrapApp();
}
