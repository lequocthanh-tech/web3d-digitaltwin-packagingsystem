/**
 * JobLetterT.js
 * 
 * Module Lập trình Quỹ đạo Tự động Vẽ Chữ T cho Robot Arm 6-DOF (Yaskawa MotoMINI / GP8)
 * Thuộc phân hệ Lập trình Robot Arm (Teach Pendant INFORM III & Workstation Controller)
 * 
 * Bao gồm:
 * 1. Định nghĩa Project INFORM III: "AUTO_DRAW_LETTER_T"
 * 2. Cơ sở dữ liệu biến vị trí P020..P025 (Cartesian ROBOT & Xung Encoder PULSE)
 * 3. Trình thực thi tự động (Auto Runner) liên kết TrajectoryTrail 3D và Đồ thị Telemetry/MATLAB
 *    giúp người dùng quan sát cánh tay robot hoạt động và đồ thị vẽ hiển thị hình chữ "T" hoàn hảo.
 */

import * as THREE from 'three';

// 1. Định nghĩa INFORM III Job cho Teach Pendant Yaskawa
export const LETTER_T_PROJECT = {
  name: 'AUTO_DRAW_LETTER_T',
  desc: 'Chương trình tự động điều khiển Robot Arm 6-DOF vẽ quỹ đạo chữ T trong không gian Cartesian, hiển thị đồ thị chữ T',
  instructions: [
    'NOP',
    '// ===============================================',
    '// CHƯƠNG TRÌNH TỰ ĐỘNG VẼ QUỸ ĐẠO CHỮ T (6-DOF ARM)',
    '// BÀI THỰC HÀNH NỘI SUY QUỸ ĐẠO CARTESIAN & ĐỒ THỊ',
    '// ===============================================',
    'MOVJ P020 VJ=50.00',        // 1. Bay nhanh về điểm an toàn tiếp cận (Safe Approach)
    'MOVL P021 V=150.0 PL=0',    // 2. Tiếp cận đầu nét ngang chữ T bên trái (X=-1.45, Y=1.20)
    'MOVL P022 V=80.0 PL=0',     // 3. Kéo nét ngang sang bên phải (X=-1.05, Y=1.20)
    'MOVL P023 V=120.0 PL=0',    // 4. Trượt mượt về tâm nét ngang (X=-1.25, Y=1.20)
    'MOVL P024 V=80.0 PL=0',     // 5. Kéo nét đứng chữ T thẳng đứng xuống (X=-1.25, Y=0.88)
    'TIMER T=0.500',             // 6. Dừng 0.5s hiển thị rõ nét chữ T
    'MOVL P023 V=120.0',         // 7. Nhấc lên tâm nét ngang
    'MOVJ P020 VJ=50.00',        // 8. Rút về điểm an toàn hoàn tất
    'END'
  ]
};

// 2. Danh sách biến vị trí P020..P025 tương ứng với các nét vẽ chữ T
export const LETTER_T_POS_VARIABLES = [
  {
    id: 'P020',
    name: 'T_SAFE_APPROACH',
    x: -1.250, y: 1.300, z: 0.180,
    rx: 0.0, ry: 0.0, rz: 0.0,
    s: -90.0, l: -7.7, u: -0.5, r: 0.0, b: -81.9, t: 0.0,
    type: 'FRONT UP FLIP'
  },
  {
    id: 'P021',
    name: 'T_BAR_LEFT',
    x: -1.450, y: 1.200, z: 0.180,
    rx: 0.0, ry: 0.0, rz: 0.0,
    s: -102.2, l: -10.8, u: -3.8, r: 0.0, b: -75.4, t: 0.0,
    type: 'FRONT UP FLIP'
  },
  {
    id: 'P022',
    name: 'T_BAR_RIGHT',
    x: -1.050, y: 1.200, z: 0.180,
    rx: 0.0, ry: 0.0, rz: 0.0,
    s: -77.9, l: -10.8, u: -3.8, r: 0.0, b: -75.4, t: 0.0,
    type: 'FRONT UP FLIP'
  },
  {
    id: 'P023',
    name: 'T_BAR_CENTER',
    x: -1.250, y: 1.200, z: 0.180,
    rx: 0.0, ry: 0.0, rz: 0.0,
    s: -90.0, l: -9.2, u: -5.7, r: 0.0, b: -75.1, t: 0.0,
    type: 'FRONT UP FLIP'
  },
  {
    id: 'P024',
    name: 'T_STEM_BOTTOM',
    x: -1.250, y: 0.880, z: 0.180,
    rx: 0.0, ry: 0.0, rz: 0.0,
    s: -90.0, l: -24.7, u: -16.4, r: 0.0, b: -48.9, t: 0.0,
    type: 'FRONT UP FLIP'
  },
  {
    id: 'P025',
    name: 'T_SAFE_RETRACT',
    x: -1.250, y: 1.300, z: 0.180,
    rx: 0.0, ry: 0.0, rz: 0.0,
    s: -90.0, l: -7.7, u: -0.5, r: 0.0, b: -81.9, t: 0.0,
    type: 'FRONT UP FLIP'
  }
];

/**
 * Hàm thực thi chu trình tự động vẽ chữ T
 * @param {Object} app - Thể hiện chính DigitalTwinApp
 * @param {Object} options - Tùy chọn cấu hình tốc độ và callback
 */
export async function runAutoLetterTJob(app, options = {}) {
  if (!app) {
    console.error('[JobLetterT] Không tìm thấy DigitalTwinApp');
    return false;
  }

  // Khóa chống kích hoạt đồng thời
  if (app.isLetterTRunning) {
    console.warn('[JobLetterT] Chu trình vẽ chữ T đang chạy...');
    return false;
  }
  app.isLetterTRunning = true;

  const sleep = (ms) => new Promise(res => setTimeout(res, ms));

  try {
    // 1. Đảm bảo nguồn và servo sẵn sàng
    if (!app.systemPower) {
      if (typeof app.setSystemPower === 'function') {
        app.setSystemPower(true);
      } else {
        app.systemPower = true;
      }
    }

    // 2. Kích hoạt và dọn sạch vết quỹ đạo 3D (TrajectoryTrail)
    if (app.trajectoryTrail) {
      app.trajectoryTrail.clear();
      app.trajectoryTrail.setArmVisible(true);
      app.trajectoryTrail.visible = true;

      const chkArmTrail = document.getElementById('chk-show-arm-trail');
      if (chkArmTrail) chkArmTrail.checked = true;
      const chkTrails = document.getElementById('chk-show-trails');
      if (chkTrails) chkTrails.checked = true;
    }

    // 3. Khởi tạo lại bộ vẽ đồ thị quỹ đạo 2D Telemetry
    if (app.telemetryGraphs && typeof app.telemetryGraphs.resetTrajectory === 'function') {
      app.telemetryGraphs.resetTrajectory();
      app.telemetryGraphs.setLetterTMode(true);
    }

    // 4. Đồng bộ Teach Pendant sang Project AUTO_DRAW_LETTER_T và PLAY MODE
    if (app.teachPendant) {
      app.teachPendant.currentJobName = LETTER_T_PROJECT.name;
      app.teachPendant.robotMode = 1; // PLAY mode
      app.teachPendant.servoOn = true;
      app.teachPendant.servoReady = true;
      app.teachPendant.startLit = true;
      app.teachPendant.holdLit = false;
      app.teachPendant.currentLineIndex = 0;
      app.teachPendant.updatePendantDisplay();
      app.teachPendant.updateScreenZone4();
      app.teachPendant.setHumanMessage('[AUTO DRAW T] Đang tự động chạy vẽ quỹ đạo chữ T...', 'info');
    }

    // Cập nhật trạng thái Lab UI nếu có
    const cycleStatus = document.getElementById('cycle-status-text');
    if (cycleStatus) {
      cycleStatus.textContent = 'ĐANG VẼ QUỸ ĐẠO CHỮ T (RUNNING)';
      cycleStatus.style.color = '#38bdf8';
    }

    // 5. Chuỗi tọa độ các điểm vẽ nét chữ T
    // Định dạng: { id, target, speed, label, lineIdx }
    const steps = [
      {
        id: 'P020',
        target: { x: -1.250, y: 1.300, z: 0.180, rx: 0, ry: 0, rz: 0 },
        duration: 1.2,
        label: 'Bước 1: Tiếp cận vị trí an toàn trên cao (P020)',
        lineIdx: 4
      },
      {
        id: 'P021',
        target: { x: -1.450, y: 1.200, z: 0.180, rx: 0, ry: 0, rz: 0 },
        duration: 0.9,
        label: 'Bước 2: Di chuyển tới đầu nét ngang bên trái (P021)',
        lineIdx: 5
      },
      {
        id: 'P022',
        target: { x: -1.050, y: 1.200, z: 0.180, rx: 0, ry: 0, rz: 0 },
        duration: 1.5,
        label: 'Bước 3: Kéo nét ngang chữ T sang bên phải (P022) [VẼ NÉT NGANG]',
        lineIdx: 6
      },
      {
        id: 'P023',
        target: { x: -1.250, y: 1.200, z: 0.180, rx: 0, ry: 0, rz: 0 },
        duration: 0.8,
        label: 'Bước 4: Trượt về tâm nét ngang chữ T (P023)',
        lineIdx: 7
      },
      {
        id: 'P024',
        target: { x: -1.250, y: 0.880, z: 0.180, rx: 0, ry: 0, rz: 0 },
        duration: 1.4,
        label: 'Bước 5: Kéo nét đứng chữ T thẳng xuống dưới (P024) [VẼ THÂN CHỮ T]',
        lineIdx: 8
      }
    ];

    // Chạy tuần tự từng bước nội suy đường thẳng mượt mà MOVL
    for (const step of steps) {
      if (app.isSafetyInterlockTripped) {
        console.warn('[JobLetterT] Cửa rào mở hoặc E-Stop kích hoạt, dừng chu trình.');
        break;
      }

      if (app.teachPendant) {
        app.teachPendant.currentLineIndex = step.lineIdx;
        app.teachPendant.updateScreenZone4();
        app.teachPendant.setHumanMessage(`[INFORM] ${step.label}`, 'info');
      }

      const startPose = app.armKinematics.forwardKinematics(app.arm6DOF.getJointAngles());
      const traj = app.trajectoryPlanner.createMOVL(
        {
          x: startPose.tcp_m.x,
          y: startPose.tcp_m.y,
          z: startPose.tcp_m.z,
          rx: startPose.orientation_deg.rx,
          ry: startPose.orientation_deg.ry,
          rz: startPose.orientation_deg.rz
        },
        step.target,
        app.armKinematics,
        {
          duration: step.duration,
          degree: 5,
          isWorld: true,
          initialJoints: app.arm6DOF.getJointAngles()
        }
      );

      await new Promise(resolve => {
        app.trajectoryPlanner.startTrajectory('letter-t-step', traj, {
          onUpdate: (data) => {
            if (data.ikResult && data.ikResult.success) {
              const q = data.ikResult.jointsDeg;
              app.arm6DOF.setJointAngles(q.s, q.l, q.u, q.r, q.b, q.t);
              app.updateArmUIFromModel();
              if (app.teachPendant) {
                app.teachPendant.updateLiveReadouts();
              }
            }
          },
          onComplete: resolve
        });
      });

      await sleep(40);
    }

    // Dừng 0.6s tại đáy chữ T để người dùng chiêm ngưỡng trọn vẹn nét vẽ chữ T
    if (app.teachPendant) {
      app.teachPendant.currentLineIndex = 9; // TIMER T=0.500
      app.teachPendant.updateScreenZone4();
      app.teachPendant.setHumanMessage('Đã vẽ xong chữ T! Đồ thị và quỹ đạo 3D hiển thị chữ T hoàn hảo.', 'success');
    }
    await sleep(600);

    // Bước 6 & 7: Nhấc lên và rút về vị trí an toàn hoàn tất
    const returnSteps = [
      {
        target: { x: -1.250, y: 1.200, z: 0.180, rx: 0, ry: 0, rz: 0 },
        duration: 0.8,
        lineIdx: 10
      },
      {
        target: { x: -1.250, y: 1.300, z: 0.180, rx: 0, ry: 0, rz: 0 },
        duration: 1.0,
        lineIdx: 11
      }
    ];

    for (const rStep of returnSteps) {
      if (app.teachPendant) {
        app.teachPendant.currentLineIndex = rStep.lineIdx;
        app.teachPendant.updateScreenZone4();
      }

      const curPose = app.armKinematics.forwardKinematics(app.arm6DOF.getJointAngles());
      const retTraj = app.trajectoryPlanner.createMOVL(
        {
          x: curPose.tcp_m.x,
          y: curPose.tcp_m.y,
          z: curPose.tcp_m.z,
          rx: curPose.orientation_deg.rx,
          ry: curPose.orientation_deg.ry,
          rz: curPose.orientation_deg.rz
        },
        rStep.target,
        app.armKinematics,
        {
          duration: rStep.duration,
          degree: 5,
          isWorld: true,
          initialJoints: app.arm6DOF.getJointAngles()
        }
      );

      await new Promise(resolve => {
        app.trajectoryPlanner.startTrajectory('letter-t-ret', retTraj, {
          onUpdate: (data) => {
            if (data.ikResult && data.ikResult.success) {
              const q = data.ikResult.jointsDeg;
              app.arm6DOF.setJointAngles(q.s, q.l, q.u, q.r, q.b, q.t);
              app.updateArmUIFromModel();
              if (app.teachPendant) {
                app.teachPendant.updateLiveReadouts();
              }
            }
          },
          onComplete: resolve
        });
      });
      await sleep(30);
    }

    if (cycleStatus) {
      cycleStatus.textContent = 'HOÀN THÀNH VẼ CHỮ T (LETTER T OK)';
      cycleStatus.style.color = '#4ade80';
    }

    if (app.teachPendant) {
      app.teachPendant.currentLineIndex = 12; // END
      app.teachPendant.startLit = false;
      app.teachPendant.updatePendantDisplay();
      app.teachPendant.updateScreenZone4();
      app.teachPendant.setHumanMessage('🎉 HOÀN TẤT VẼ QUỸ ĐẠO CHỮ T! ĐỒ THỊ ĐÃ HIỂN THỊ CHỮ T.', 'success');
    }

    console.log('[JobLetterT] Chu trình tự động vẽ chữ T hoàn thành thành công.');
    return true;
  } catch (err) {
    console.error('[JobLetterT] Lỗi khi thực thi vẽ chữ T:', err);
    return false;
  } finally {
    app.isLetterTRunning = false;
  }
}
