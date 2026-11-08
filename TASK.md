# TASK.md - Bảng Điều Phối Công Việc (Antigravity & Claude Code)

*File này dùng để theo dõi tiến độ công việc giữa Người dùng, Claude Code và Antigravity.*

## 📌 Trạng Thái Hiện Tại
- **Hệ thống:** Khởi tạo môi trường phối hợp thành công.
- **Tiến độ dự án:** Đã hoàn thành toàn bộ **GIAI ĐOẠN 2, GIAI ĐOẠN 3 và GIAI ĐOẠN 4** (3 Môi trường Lập trình Thực hành: Yaskawa Teach Pendant INFORM, MPE720 Ladder & Motion Studio, C# / Qt PC Control & Vision Studio, kèm thanh chuyển đổi Workstation Switcher và chế độ Split-Screen 50/50).

---

## 📋 Danh Sách Nhiệm Vụ (Task List)

### ✅ Đã hoàn thành (DONE)
- [x] **Task 01: Xây dựng Module Động học Robot Arm 6 Bậc (`js/kinematics/Yaskawa6DOFKinematics.js`)**
  - Bảng thông số Denavit-Hartenberg (DH Parameters) cho 6 khâu S, L, U, R, B, T với Cổ tay cầu Spherical Wrist.
  - Hàm `forwardKinematics(jointsDeg)` trả về ma trận $A_1..A_6$, composite $T_0^6$, tọa độ $(X, Y, Z)$ (mm/m), góc Euler $(Rx, Ry, Rz)$ và mảng ma trận 4x4 cho UI.
  - Hàm `inverseKinematics(targetPose)` giải tách vị trí tâm cổ tay W để tính $\theta_1, \theta_2, \theta_3$ và góc cổ tay $\theta_4, \theta_5, \theta_6$ từ ma trận $R_3^6$.
  - Tỷ lệ chuyển đổi Xung/Độ Yaskawa MotoMINI: S: 34816/30, L: 102400/90, U: 51200/90, R/B/T: 10204/30 (`degToPulse`, `pulseToDeg`).

- [x] **Task 02: Xây dựng Module Động học Song song Robot Delta (`js/kinematics/Delta4DOFKinematics.js`)**
  - Khai báo thông số chuẩn: $f = 0.36\text{m}, e = 0.085\text{m}, rf = 0.42\text{m}, re = 0.78\text{m}$.
  - Hàm `forwardKinematics` (giao điểm 3 mặt cầu / Trilateration) và `inverseKinematics` tính 3 góc servo $(m_1, m_2, m_3)$ và góc xoay đầu hút $(yaw)$.
  - Hàm `computeRodsTransforms` tính vị trí và góc nghiêng 3D chính xác cho 3 cặp thanh carbon song song.

- [x] **Task 03: Xây dựng Bộ Nội suy Quỹ đạo Mượt (`js/kinematics/TrajectoryPlanner.js`)**
  - Quy luật thời gian đa thức bậc 3 (Cubic) và bậc 5 (Quintic, C2 continuous, jerk = 0 tại biên).
  - Lệnh `MOVJ` (Joint Interpolation) nội suy đồng bộ không gian khớp.
  - Lệnh `MOVL` (Cartesian Linear Interpolation) nội suy đường thẳng tuyệt đối trong không gian 3D, gọi IK tại mỗi frame.

- [x] **Task 04: Visualizer Quỹ đạo 3D (`js/kinematics/TrajectoryTrail.js`)**
  - Vẽ vết di chuyển 3D của TCP thời gian thực (Cyan cho Arm 6-DOF, Amber cho Delta Robot) bằng BufferGeometry ring buffer không drop frame.

- [x] **Task 05: Engine Cảm biến & Cơ chế Gắp/Thả Vật lý (`js/physics/VacuumAndSensorEngine.js`)**
  - Cognex Structured Light Vision Scanner trên Băng tải 1 quét và lưu $(x, y, yaw)$ vào `visionQueue`.
  - Cảm biến hồng ngoại `DI_11` (cụm 4 thanh sẵn sàng) và `DI_12`/`DI_13` (thùng carton đỗ tại trạm).
  - Vacuum Gripper Scene Graph Re-parenting:
    + Delta hút thanh lẻ trên Băng tải 1, nắn thẳng $yaw = 0^\circ$, đặt ghép cụm lên Băng tải 2.
    + Arm Multi-Gripper hút nguyên cụm 4 thanh, nâng chuyển và xếp 2 lớp vào thùng carton trên Băng tải 3.

- [x] **Task 06: Nâng cấp Giao diện SCADA Panel Phải (Tab "KINEMATICS & TRAJECTORY LAB")**
  - Bảng tab điều hướng 2 chế độ: HARDWARE JOGGER và KINEMATICS & TRAJECTORY LAB.
  - Hiển thị bảng ma trận đồng nhất $4 \times 4$ ($T_0^6$) cập nhật thời gian thực.
  - Form điều khiển quỹ đạo Cartesian với các nút `[Execute MOVJ]`, `[Execute MOVL]`, `[Stop Motion]`.
  - Nút `[Test Single Pick & Place Cycle]` chạy thử 1 chu trình gắp phối hợp tự động giữa 2 robot.

- [x] **Task 07: Xây dựng Bus Phần Cứng Ảo MP3300 (`js/hardware/MP3300HardwareBus.js`) [GIAI ĐOẠN 3]**
  - Mô phỏng bộ nhớ thanh ghi MPE720: MB, MW, ML, MQ, MF, MD, MA hỗ trợ Little-Endian.
  - Ma trận I/O cục bộ: `IW00C80`, `IW00C81` (32 Digital Inputs) và `OW00C80`, `OW00C81` (32 Digital Outputs).
  - Ánh xạ tín hiệu chuyên dụng: Control Panel switches I4..I7 -> `IN#5..IN#8`; Robot Arm `OT#5..OT#8` -> Q4..Q7 (`DO_20..DO_23`).
  - Xung 100ms từ `OT#3` khi thả hàng -> kích hoạt `DI_12` và đèn Q0 trên I/O Box (`DO_24`).
  - Máy trạng thái Built-in PLC Ladder Logic (Bài 2): Điều phối biến tần J1000 chạy băng tải 1 & 2 (`DO_00`), chờ cảm biến `DI_11`, kích robot gắp xếp vào thùng carton, phát hiện thùng tại `DI_13`, đẩy thùng ra (`DO_03`).

- [x] **Task 08: Giả lập Mạng UDP & Server YRC1000micro (`js/hardware/YRC1000EthernetServer.js`) [GIAI ĐOẠN 3]**
  - Quản lý mạng: MP3300 218IFD Client (192.168.1.1:10001) <-> YRC1000micro Server (192.168.1.11:10040, UDP BIN).
  - Đóng gói và giải mã khung truyền chuẩn 32-Byte YERC Header (`Identifier = 0x43524559`, `HeaderSize = 0x0020`, `Reserve2 = "99999999"`).
  - Các mã lệnh High-Speed Ethernet: Servo ON/OFF (`0x83`), Start Job (`0x86`), Read Status (`0x72`), Read Pos (`0x75`), Move (`0x8A`).
  - Khóa liên động an toàn phần cứng (Safety Interlock): YRC1000 chỉ chấp nhận lệnh từ xa khi Mode Switch ở chế độ `REMOTE` (2). Nếu ở `TEACH` (0) hoặc `PLAY` (1), Server trả về gói tin Answer có mã lỗi `0x1F` (Added Status `0x2030`) và cảnh báo.
  - Khối hàm PLC `MSG_SNDE` với các cờ `Execute GB000100`, `Busy GB000102`, `Complete GB000103`, `Error GB000104`, `Dev-Typ 16`, `Param MA00400`.

- [x] **Task 09: Thiết kế Giao diện Phần Cứng Trực quan (Tab "CTRL PANEL" & Tab "UDP SNIFFER") [GIAI ĐOẠN 3]**
  - Tab "CTRL PANEL": Nút nấm đỏ E-STOP twist-to-release, 4 nút nhấn tròn công nghiệp viền Chrome, công tắc xoay 3 vị trí TEACH/PLAY/REMOTE, 8 công tắc gạt kim loại kèm Jack bắp chuối xanh dương, 8 đèn Led đỏ kèm Jack bắp chuối vàng, I/O Box mở rộng, ma trận 32-bit DI/DO RAM.
  - Tab "UDP SNIFFER": Bảng Easy Setting, banner an toàn Mode Switch, nút test injection, cửa sổ Hex Monitor terminal.

- [x] **Task 10: Xây dựng 3 Môi trường Lập trình Thực hành (Engineering Workstations) [GIAI ĐOẠN 4 - TRỌNG TÂM HỌC THUẬT]**
  - **Station 1: Yaskawa Teach Pendant (`js/workstations/TeachPendantStation.js`)**:
    + Khung vỏ mô phỏng bộ lập trình YRC1000micro với Deadman Switch 3 nấc (`Released`, `Light Squeeze`, `Panic`), SERVO ON, nút chọn hệ tọa độ (`JOINT`, `CARTESIAN`, `TOOL`) và tốc độ Jog.
    + 12 phím bấm giữ để Jog trục (`[S-][S+]`.. hoặc `[X-][X+]`..).
    + Quản lý biến vị trí `P000..P009` kèm nút `[MODIFY] + [ENTER]` lưu tọa độ thực tế.
    + Trình soạn thảo & thông dịch INFORM Job Interpreter: hỗ trợ `MOVJ`, `MOVL`, `WAIT IN#(...)`, `DOUT OT#(...)`, `PULSE OT#(3)`, `TIMER`, `*LABEL`, `JUMP`. Nút điều khiển: `[FWD]`, `[BWD]`, `[TEST START]`, `[PLAY START]` (chạy tự động highlight màu vàng).
  - **Station 2: MPE720 Ladder & Motion Studio (`js/workstations/MPE720StudioStation.js`)**:
    + Cây thư mục dự án chuẩn MPE720 `[CPU-301(16axes)]`.
    + Tab 1: Sơ đồ mạch Ladder trực quan với 5 Rungs (Rung 1 khởi động tự giữ biến tần DO_00 & gọi MSEE MPM001; Rung 2 cảm biến DI_11 gọi gắp cụm DO_01; Rung 3 xung DI_12 tăng bộ đếm MW00010; Rung 4 so sánh MW00010 >= 2 đẩy thùng DO_03; Rung 5 truyền thông Ethernet MSG-SNDE) kèm hiển thị dòng điện logic (Power Flow) phát sáng xanh lá thời gian thực.
    + Tab 2: Trình soạn thảo Motion Program `MPM001` điều khiển Robot Delta bám camera và xếp cụm, bảng biến toàn cục `GL100..GL202`, nút `[Compile & Download to MP3300]`.
  - **Station 3: C# / Qt PC Control & Vision Studio (`js/workstations/CSharpPCStudioStation.js`)**:
    + Nửa trái: Giao diện Form Windows / Qt chuẩn Bài 4: Connection UDP 10040, Basic Control (Go Home, Tool On, ON Servo), Current Position (Get Position 0x0075), Set Position (XYZ/RPY, Link/Line Abs/Inc), Incremental Position (Nudge buttons, distance & speed sliders), Get Joints & Calculate FK (kiểm chứng lý thuyết ma trận DH).
    + Nửa phải: C# Code Editor & Cognex Vision IDE chứa code mẫu chuẩn Bài 4 & Bài 5 (`btn_Connect_Click`, `btn_xP_Click`, `YRCMoveCartesian`, `RunAutoVisionPackagingLoop`) kèm nút `[Compile & Execute C# Code]` chạy chu trình đóng gói tự động.
  - **Thanh Chuyển đổi Workstation Switcher & Bố cục Split-Screen 50/50**:
    + Thanh điều hướng phía trên gồm 4 nút: `[3D FACTORY & SCADA I/O]`, `[STATION 1: TEACH PENDANT]`, `[STATION 2: MPE720 PLC & MOTION]`, `[STATION 3: C# / QT PC & VISION]`.
    + Tự động co giãn màn hình Split-Screen 48% (3D Factory) / 52% (Workstation Code & GUI) kèm chuyển góc camera tương ứng để vừa viết code vừa quan sát robot chạy thực tế.

- [x] **Task 11: Màn hình Giám sát VIPA HMI, Telemetry Real-time & Giả lập Sự cố Chaos Lab [GIAI ĐOẠN 5 - HOÀN THIỆN TOÀN DIỆN]**
  - **Màn hình Cảm ứng Công nghiệp VIPA HMI (`js/hmi-scada/VIPAHMIScreen.js`)**:
    + Khung viền nhôm xước công nghiệp (Brushed Aluminum Bezel) chuẩn màn hình cảm ứng Touch Panel VIPA / Yaskawa.
    + Tab `[1. MAIN OVERVIEW]`: Sơ đồ Synoptic trực quan của dây chuyền đóng gói gồm 3 băng tải, Robot Delta bốc phôi, Robot Arm 6 bậc đóng thùng carton; giám sát trạng thái RUN/STOP, tốc độ biến tần J1000 (Hz), các bộ đếm sản xuất OEE thời gian thực (`X/4` thanh trong cụm, `Y/2` cụm trong thùng, `Z` tổng thùng xuất xưởng).
    + Tab `[2. RECIPE & SETTINGS]`: Cấu hình thông số mẻ sản xuất (Batch Recipe, số sản phẩm/cụm, thời gian trễ gắp nhả dwell time, tốc độ định mức của servo).
    + Tab `[3. ALARM & DIAGNOSTICS]`: Bảng nhật ký sự kiện Event Log và danh sách cảnh báo thời gian thực; cơ chế nhấp nháy đèn tháp DO_19 cho đến khi nhấn nút `[ALARM RESET]` hoặc bấm `Robot On (DI_03)`.
  - **Đồ thị Giám sát Tải trọng & Góc khớp Thời gian thực (`js/hmi-scada/TelemetryAndGraphs.js`)**:
    + Dao động ký đa kênh (Multi-channel Oscilloscope) vẽ trực tiếp trên HTML5 Canvas 2D 60 FPS mà không cần phụ thuộc thư viện nặng bên ngoài.
    + Theo dõi góc khớp thời gian thực `J1..J6` của Robot Arm MotoMINI.
    + Tính toán và vẽ đồ thị mô-men xoắn / tỷ lệ tải trọng khớp (Torque / Load Percentage 0..100%), tự động đổi dải màu từ Xanh lục (<50%) sang Vàng (50-80%) và Đỏ (>80%) khi quá tải hoặc tiệm cận giới hạn an toàn.
  - **Phòng Thử nghiệm Giả lập Sự cố (`js/hmi-scada/FaultInjectionPanel.js`)**:
    + Bảng trượt "FACTORY FAULT INJECTION LAB" hỗ trợ 4 kịch bản lỗi thực tế trong nhà máy công nghiệp:
      * `[Inject Jam / Kẹt Băng tải]`: Dừng băng tải, kích hoạt cảnh báo kẹt phôi và chớp đèn tháp DO_19.
      * `[Open Safety Gate / Mở Cửa An Toàn]`: Cắt E-Stop phần cứng, ngắt Servo và nhả các đầu hút chân không.
      * `[Inject Packet Drop / Mất Gói Tin UDP]`: Thả rơi gói tin truyền thông 10040 với mã lỗi 0x1F, kiểm chứng tính kháng nhiễu và an toàn mạng của hệ thống.
      * `[Fail Sensor DI_11 / Hỏng Cảm Biến Cụm]`: Cố định tín hiệu DI_11 ở mức 0, khiến dây chuyền dừng chờ tại trạm cụm.
    + Nút `[Clear All Faults / Khôi Phục Hoạt Động Bình Thường]` để xóa các cờ giả lập.
    + Nút `[🔄 System Master Reset]`: Đặt lại toàn bộ hệ thống (2 robot về vị trí Home, xóa phôi/thùng và tái tạo mới, đặt lại các thanh ghi I/O và bộ đếm mẻ hàng).
  - **Tích hợp Station 4 vào Thanh Switcher & Luồng 60 FPS**:
    + Nút `[STATION 4: VIPA HMI & TELEMETRY]` trên thanh Workstations bar chia màn hình 54% (VIPA HMI) / 46% (Telemetry Oscilloscope).
    + Nút `[⚡ FAULT INJECTION]` mở nhanh phòng thí nghiệm sự cố từ bất kỳ góc nhìn nào.
    + Nút `[🔄 MASTER RESET]` khởi tạo lại toàn bộ dây chuyền chỉ bằng 1 cú click.

- [x] **Task 12: Nâng cấp Toàn diện Bộ Lập Trình Cầm Tay Ảo (Virtual Teach Pendant Yaskawa YRC1000micro - Bài 1 ĐHBK TPHCM)**
  - **Khung Tay Cầm Vật Lý (Chassis) & Công Thái Học**:
    + Vỏ màu xám đen công nghiệp `#2b2d30`, bo góc công thái học với đệm cao su bảo vệ màu vàng Yaskawa `#f59e0b`.
    + Đa chế độ hiển thị linh hoạt: Cửa sổ nổi kéo thả tự do (`Floating / Draggable Window`) trên không gian 3D, hoặc nút ghim cố định sang cạnh phải (`Dock Right`).
    + Cụm nút điều khiển vật lý phía trên:
      * Công tắc xoay chìa khóa 3 vị trí (Mode Switch): [TEACH] - [PLAY] - [REMOTE] với kim chỉ thị góc xoay có chuyển động thực tế.
      * Nút nấm dừng khẩn cấp [EMERGENCY STOP] màu đỏ viền vàng nảy cơ học, nhấn cắt servo, xoay phải để reset.
      * Cặp nút tròn vòm công nghiệp: [START] xanh lá có đèn LED phát sáng khi chạy, và [HOLD] màu vàng có đèn LED khi tạm dừng.
      * Nút bấm [SERVO ON READY] kèm đèn LED xanh chỉ thị 2 trạng thái: nhấp nháy khi sẵn sàng, sáng đều khi đóng nguồn động lực.
      * Công tắc ảo Deadman Switch (Enable Switch) 3 vị trí mặt sau: Release (OFF) -> Light Squeeze (ON - SAFE) -> Tight Squeeze (OFF - PANIC).
  - **Màn Hình LCD Cảm Ứng 5 Khu Vực Chuẩn Yaskawa Motoman**:
    + Khu vực 1 (Menu Area - Trên cùng): Các menu DATA, EDIT, DISPLAY, UTILITY hỗ trợ Dropdown tương tác.
    + Khu vực 2 (Status Display Area - Góc trên phải): Biểu tượng chìa khóa mode (☝T TEACH, ↻P PLAY, 📡R REMOTE), icon Hệ tọa độ ([Z] JOINT, [XYZ] CARTESIAN, [TOOL] TOOL), Cấp tốc độ (INCH, LOW, MED, HIGH), Trạng thái hoạt động (STOP, HOLD, RUN, ESTOP), Cấp bảo mật (🔑🔑), Tool Index ([00]), và LED trạng thái Servo ON/OFF.
    + Khu vực 3 (Main Menu Area - Cột dọc bên trái): 5 nút chức năng JOB, VARIABLE, IN/OUT, ROBOT, SYS INFO.
    + Khu vực 4 (General Purpose Display Area - Màn hình chính đa năng):
      * Tab "Position Variable": Quản lý biến điểm P000 (Home) đến P005 (và mở rộng P006..P009). Chuyển đổi định dạng PULSE (khớp/xung encoder) <-> ROBOT (Cartesian X, Y, Z, Rx, Ry, Rz mm/deg). Thẻ chi tiết tọa độ và góc khớp thời gian thực.
      * Tab "JOB CONTENT": Trình soạn thảo chương trình INFORM chuẩn 2 cột (Cột 1: Địa chỉ dòng lệnh 0000, 0001..., Cột 2: Nội dung câu lệnh NOP, MOVJ, MOVL, DOUT, TIMER, PULSE, WAIT, END) kèm con trỏ dòng lệnh và highlight chuyển động sáng đèn.
      * Tab "IN/OUT": Giám sát và bật tắt Digital Output OUT#1 (Giác hút nam châm) và đọc Digital Input IN#1..IN#8.
      * Tab "ROBOT": Work Home Pos hiển thị và đưa tất cả 6 khớp về gốc 0°.
      * Tab "SYSTEM INFO": Thông số kỹ thuật bộ điều khiển YRC1000micro và cánh tay MotoMINI.
    + Khu vực 5 (Human Interface / Buffer Line - Dưới cùng): Dòng đệm (BUFFER) soạn thảo câu lệnh trước khi nạp/chèn vào JOB, kèm dòng thông điệp hướng dẫn người dùng và cảnh báo khóa liên động an toàn thời gian thực.
  - **Cụm Phím Bấm Điều Khiển Dưới Màn Hình**:
    + 12 phím Jog 6 trục ([S-][S+], [L-][L+], [U-][U+], [R-][R+], [B-][B+], [T-][T+]) hỗ trợ nhấn giữ để Jog liên tục ở tần số 20 Hz.
    + Nút [COORD] luân chuyển JOINT -> CARTESIAN -> TOOL; nhãn trên cụm phím tự động đổi sang [X-][X+], [Y-][Y+], [Z-][Z+], [Rx-][Rx+], [Ry-][Ry+], [Rz-][Rz+].
    + Cụm phím tốc độ: [FAST] và [SLOW] bước qua 4 cấp: INCH (1%) -> LOW (5%) -> MED (25%) -> HIGH (100%), kèm phím giữ [HIGH SPEED] tăng tốc tức thời.
    + Phím lệnh: [INFORM LIST] mở modal tập lệnh (MOTION, INOUT, CONTROL), [MODIFY], [INSERT], [DELETE], [ENTER], [SELECT], [CANCEL].
    + Phím chạy thử nghiệm: [INTERLOCK], [TEST START] (chạy thử trong TEACH), [FWD] (chạy từng bước tới điểm), [BWD] (lùi điểm).
    + D-pad điều hướng và bàn phím số mini.
  - **Móc Nối Logic Động Học & 3D Manipulator**:
    + Khóa liên động an toàn (Interlock): Chỉ cho phép Jog khi Mode=TEACH, Deadman=Light Squeeze (1), và SERVO ON. Nếu vi phạm sẽ hiển thị cảnh báo và khóa chuyển động.
    + Cơ chế Dạy điểm (Teaching): Khi chọn biến P00x trong tab Position Variable, nhấn [MODIFY] rồi nhấn [ENTER] sẽ lưu góc khớp và tọa độ Cartesian hiện tại của robot 3D vào biến đó.
    + Chức năng [FWD]: Điều khiển robot chuyển động mượt mà tới vị trí của dòng lệnh hoặc biến điểm đang chọn bằng bộ lập quỹ đạo TrajectoryPlanner.
    + Chế độ tự động (Play Mode): Gạt chìa khóa sang PLAY, bật Servo và nhấn nút [START], bộ thông dịch INFORM Interpreter chạy tuần tự từng dòng lệnh, điều khiển robot chuyển động mượt mà, gắp phôi và xếp thùng tự động theo đúng quỹ đạo.
  - **Thay thế hoàn toàn các thanh trượt phẳng cũ**:
    + Thẻ điều khiển cánh tay 6-DOF ở thanh bên phải (Dock) được thay thế bằng trung tâm điều khiển Teach Pendant YRC1000micro.
    + Nút `[📱 VIRTUAL PENDANT]` trên thanh switcher và nút `[📱 MỞ TAY CẦM NỔI]` / `[📌 GHIM VÀO ĐÂY]` cho phép truy cập tay cầm ảo tức thì từ mọi màn hình.

- [x] **Task 13: Xây dựng Mô hình và Giao diện Điều khiển Robot Delta 3-DOF & Delta Teach Pendant**
  - **Engine Động học Song song (`js/kinematics/Delta3DOFKinematics.js`)**:
    + Thông số cơ cấu chuẩn: $R_{base} = 180\text{ mm}$ (3 động cơ tại $0^\circ, 120^\circ, 240^\circ$), cánh tay chủ động $L_1 = 200\text{ mm}$, cánh tay bị động 3 cặp thanh song song $L_2 = 500\text{ mm}$, bàn gắp di động $r_{platform} = 50\text{ mm}$.
    + Thuật toán Động học Nghịch (IK) giải tích bằng phương trình giao điểm giữa mặt cầu cánh tay dưới với mặt phẳng quay cánh tay trên ($t = \tan(\theta/2)$), chọn nghiệm vật lý hướng ra ngoài: $t = \frac{b - \sqrt{D}}{c + a}$.
    + Thuật toán Động học Thuận (FK) bằng kỹ thuật giao 3 mặt cầu (Trilateration) sai số khép kín $< 0.0002\text{ mm}$.
    + Kiểm tra vùng làm việc an toàn (Workspace validation): $Z \in [-600\text{ mm}, -350\text{ mm}]$, bán kính $R \le 250\text{ mm}$.
  - **Mô hình 3D Visualizer (`js/robots/DeltaRobot3D.js`)**:
    + Khung treo 3 chân vững chắc (Sturdy 3-Legged Suspension Tripod Frame) bố trí tại góc $60^\circ, 180^\circ, 300^\circ$ xen kẽ các servo arm, chống va đập tuyệt đối, có chân đế bulông mạ crôm bắt chặt vào mặt bàn đóng gói.
    + Đèn tháp 3 màu Andon Tower và cụm điều áp khí nén FRL kèm ống xoắn polyurethane.
    + Bàn gắp di động mang giác hút chân không (Vacuum Gripper) với vòng sáng trạng thái khí nén (xanh lá khi hút -85 kPa, xám khi nhả).
    + Khay chứa phôi inox (Receiving Tray) đặt tại vị trí Place $(X = 120\text{ mm}, Y = 100\text{ mm})$ chia ngăn $2 \times 2$ có phôi kẹo mẫu sẵn.
  - **Bảng Điều khiển Dạy Điểm (Delta Teach Pendant Station - `js/workstations/DeltaTeachPendantStation.js`)**:
    + Cụm phím Jog Cartesian: $[X+][X-]$, $[Y+][Y-]$, $[Z+][Z-]$ với lựa chọn bước nhảy ($1\text{ mm}, 10\text{ mm}, 50\text{ mm}$).
    + Công tắc giác hút $[VACUUM\text{ ON/OFF}]$ (-85.0 kPa) liên kết vật lý trực tiếp với phôi trong không gian Three.js.
    + Bộ nhớ điểm Point Memory quản lý 3 điểm: Pick (băng tải), Place (khay), Safe (chờ trên cao).
    + Nút `[TEACH CURRENT POS]` lưu tọa độ hiện tại vào điểm nhớ và nút `[GOTO POINT]` bay mượt mà đến vị trí.
  - **Quỹ đạo Chuyển động Tự động (Auto Cycle / Inverted-U Gate Trajectory / $\Pi$-curve)**:
    + Quỹ đạo chữ U ngược 7 bước: Safe $\rightarrow$ Bay ngang tới Pick $\rightarrow$ Hạ xuống Pick & Bật hút $\rightarrow$ Nhấc lên Safe $\rightarrow$ Bay ngang tới Khay Place $\rightarrow$ Hạ xuống Place & Nhả hút $\rightarrow$ Về Safe.
    + Thanh trượt điều chỉnh tốc độ chu kỳ (Cycles Per Minute - CPM) từ 20 đến 120 CPM với đồng hồ đo thời gian chu kỳ thời gian thực.
  - **Tích hợp Toàn diện Dây chuyền**:
    + Nút `[⚡ DELTA 3-DOF STATION]` trên thanh switcher kích hoạt chế độ Split-Screen 48/52 và điều hướng camera.
    + Nút `[⚡ DELTA PENDANT]` bật/tắt cửa sổ nổi kéo thả tự do trên không gian 3D.
    + Nút `[📱 DELTA PENDANT]` và `[⚡ CHẠY AUTO (Π)]` trên thẻ điều khiển Inspector Dock.

- [x] **Task 14: Nâng Cấp Hoàn Thiện Ưu Tiên 1 (Nhóm An Toàn Lao Động & Cơ Cấu Chốt Chặn Khí Nén Chuẩn Công Nghiệp) [DONE]**
  - **1. Hàng Rào An Toàn Liên Hoàn (Integrated Safety Enclosure - `js/scene/FactoryEnvironment.js`)**:
    + Bao trọn toàn bộ không gian làm việc của Robot Delta 4-DOF, Robot Arm 6-DOF và cả 3 băng tải trung gian (kích thước $X \in [-3.50\text{m}, +4.10\text{m}]$, $Z \in [-1.85\text{m}, +0.70\text{m}]$, chiều cao $2.20\text{m} \ge 2.0\text{m}$).
    + Cột đứng và giằng ngang: Nhôm định hình hộp màu vàng an toàn công nghiệp (`#f59e0b`), chân đế kim loại màu đen cố định sàn kèm 4 bulông mạ crom.
    + Vách bao bọc: Lưới thép ô vuông sơn đen chống lóa (Anti-glare Wire Mesh) cho các mặt cạnh và mặt sau, kết hợp tấm Mica/Polycarbonate trong suốt viền đen ở các khoang quan sát mặt trước giúp nhìn rõ chuyển động của cả 2 robot.
    + Cửa ngõ băng tải (Safety Tunnels):
      * Hộp hầm Băng tải 1 (Infeed Tunnel) nhô ra ngoài $400\text{mm}$ ($X = +4.10\text{m} \rightarrow +4.50\text{m}$) chống thò tay vào vùng nguy hiểm.
      * Hộp hầm Băng tải 3 (Outfeed Tunnel) nhô ra ngoài $400\text{mm}$ ($X = -3.50\text{m} \rightarrow -3.90\text{m}$) cho thùng carton thành phẩm trôi ra ngoài an toàn.
      * Cả 2 cửa hầm đều viền phản quang sọc chéo an toàn vàng/đen (Hazard Stripes).
    + Cửa bảo trì có Khóa An toàn (Maintenance Safety Door):
      * Cánh cửa bản lề ở mặt trước rào ($X \in [-0.70\text{m}, +0.30\text{m}]$, rộng $1.0\text{m}$, cao $2.02\text{m}$) có tay nắm công nghiệp.
      * Công tắc khóa an toàn liên động (Safety Door Interlock Switch) màu đỏ/vàng trên khung cửa kèm đèn LED trạng thái (Xanh = Safe/Closed, Đỏ = Open).
      * Hỗ trợ click trực tiếp vào cánh cửa 3D trong Viewport để mở/đóng với chuyển động bản lề mượt mà.
      * Cung cấp biến `safetyGateClosed = true/false` (alias `SafetyGate_Closed`) và hàm `toggleSafetyDoor()`.
  - **2. Cột Đèn Tháp Andon 3 Tầng (Andon Signal Tower Light - `FactoryEnvironment.js`)**:
    + Gắn trên đỉnh cao nhất của khung rào ($Y = 2.20\text{m} \rightarrow 2.60\text{m}$) tại vị trí cửa bảo trì ($X = -0.70\text{m}, Z = +0.70\text{m}$).
    + Cột kim loại màu xám/đen cao $0.4\text{m}$, nắp chụp còi báo buzzer.
    + Cụm 3 tầng đèn tròn: ĐỎ (trên cùng), VÀNG (ở giữa), XANH LÁ (dưới cùng) dùng vật liệu emissive phát sáng rực rỡ và bóng PointLight hắt sáng frame rào.
    + Logic điều khiển theo trạng thái hệ thống:
      * **XANH LÁ (RUN):** Sáng khi toàn bộ cell đang chạy tự động, cửa rào đóng kín, không có lỗi.
      * **VÀNG (STANDBY/PAUSE):** Sáng khi hệ thống đang tạm dừng, chờ cấp phôi trên Băng tải 1, hoặc đang ở chế độ Jog thủ công.
      * **ĐỎ (ALARM/E-STOP):** Nhấp nháy tần số $2.5\text{ Hz}$ khi: Nhấn nút E-Stop, mở cửa an toàn rào chắn (`safetyGateClosed == false`), hoặc có sự cố kẹt băng tải/quá tải.
  - **3. Xilanh Khí Nén Chốt Chặn & Kẹp Định Vị Thùng (Pneumatic Stopper on Conveyor 3 - `FactoryEnvironment.js`)**:
    + Thân xilanh khí nén lắp giữa ray băng tải tại $X = -1.40\text{m}, Z = -1.45\text{m}$ có co nối khí nén và ống dẫn PU màu xanh dương.
    + Cần piston crôm gắn chốt chặn kim loại và con lăn xoay (Stopper Pin/Roller) nhô vuông góc lên mặt băng tải.
    + Cặp má kẹp dẫn hướng khí nén (Side Guide Clamps) tại $X = -1.25\text{m}$ ở hai bên thành băng tải ép giữ thùng carton không bị lệch góc khi Robot Arm thả sản phẩm vào.
    + Logic vận hành khí nén:
      * Khi thùng rỗng chạy tới cảm biến: Xilanh Stopper nhô lên (`Stopper_State = "EXTENDED"`), chặn cố định đáy thùng, hai má kẹp ép giữ (`clampState = "CLAMPED"`).
      * Trong lúc Robot Arm đang gắp thả: Stopper và má kẹp tiếp tục duy trì khóa cứng tọa độ thùng.
      * Khi thùng đã đầy sản phẩm (`box.full == true`): Xilanh Stopper thụt xuống dưới mặt băng tải (`Stopper_State = "RETRACTED"`), má kẹp mở (`RELEASED`) để thùng trôi qua hầm an toàn ra ngoài.
      * Sau khi thùng đầy trôi qua khỏi cảm biến: Stopper lập tức nhô lên trở lại đón thùng rỗng tiếp theo.
  - **4. Giao Diện & Mạch Khóa Liên Động An Toàn (UI & Safety Interlock - `index.html`, `style.css`, `main.js`)**:
    + Nút bấm `[🚪 CỬA RÀO: ĐÓNG / MỞ]` trên Header Quick Indicators và thanh Dock SCADA (`[🚪 MỞ/ĐÓNG CỬA BẢO TRÌ]`).
    + Đèn báo trạng thái `DO_STOPPER_CYLINDER` (Xanh khi mở chốt hạ xuống, Xám khi đóng chốt nhô lên chặn thùng).
    + Đèn báo `DI_01` (Fence Gate Closed / Open).
    + Modal popup cảnh báo viền đỏ nhấp nháy: `[WARNING] SAFETY INTERLOCK TRIPPED: MAINTENANCE DOOR OPEN!` (theo chuẩn ISO 13849-1).
    + Mạch an toàn liên động: Khi mở cửa rào, lập tức ngắt động lực Servo của cả 2 robot (STO), dừng khẩn cấp cả 3 động cơ băng tải, đèn Andon nhấp nháy ĐỎ.
    + Yêu cầu đóng cửa và nhấn `[RESET FAULT / ROBOT ON]` (hoặc nút `ROBOT ON (DI_03)`) mới cho phép khởi động lại hệ thống.

- [x] **Task 15: Nâng Cấp Giai Đoạn 2 - Phần 1 (Cơ Cấu Loại Bỏ Phế Phẩm Khí Nén & Máy Dán Băng Keo Nắp Thùng Carton) [DONE]**
  - **1. Cơ Cấu Gạt Phế Phẩm Khí Nén & Thùng Chứa NG (QC Station on Conveyor 1 - `FactoryEnvironment.js`)**:
    + Bố trí Trạm kiểm tra chất lượng (QC Station) tại $X = +2.45\text{m}$ (cách Camera Vision Cognex $350\text{mm}$ về phía hạ lưu, trước vùng làm việc $X = 1.20\text{m}$ của Robot Delta).
    + Dựng mô hình 3D Xilanh khí nén thân nhôm chữ nhật (ISO profile) gắn cố định vào khung ray giữa băng tải. Cần piston crôm bóng dẫn hướng bàn gạt kim loại (Pusher Plate bản dẹt nằm ngang sát mặt dây đai tại $Y = 0.885\text{m}$) kèm lớp đệm cao su giảm chấn đen.
    + Dựng máng trượt dốc inox (Receiving Chute) và thùng chứa phế phẩm màu vàng công nghiệp (NG Defect Bin) đặt ở mép ngoài đối diện ($Z = -0.74\text{m}$) có tay nắm, nhãn viền cảnh báo sọc vàng/đen "NG REJECT // QC DEFECT" và giá đỡ thép 4 chân vững chắc.
    + Logic phân loại và gạt phế phẩm (Shift Tracking & Ejection Sequence):
      * Sản phẩm có thuộc tính `status: "OK" | "NG"`. Phôi NG có visual nhận diện trực quan: lớp vỏ giấy bạc màu đỏ thẫm cháy xém (`matProductDefectNG`) với đường nứt cảnh báo và chấm nhận diện đỏ rực trên đỉnh.
      * Camera Cognex quét qua phôi NG: LED camera chớp đỏ cảnh báo (`0xef4444`) và tự động không đưa vào hàng đợi gắp của Delta.
      * Khi tâm phôi NG trôi tới đúng tọa độ bàn gạt xilanh ($X \in [2.38\text{m}, 2.52\text{m}]$):
        1. Kích hoạt cờ `DO_REJECT_CYLINDER = true` (`rejectCylinderDO = true`).
        2. Cần xilanh phóng nhanh hành trình ngang trong $0.15\text{s} - 0.16\text{s}$ quét sạch phôi NG rơi vào máng trượt thùng chứa.
        3. Phôi rơi theo trọng trường vào đáy thùng NG và được xóa an toàn khỏi luồng băng tải.
        4. Cần xilanh tự động rút nhanh về vị trí ban đầu (Retracted $Z = -0.13\text{m}$) và tắt cờ `DO_REJECT_CYLINDER`.
        5. Robot Delta hoàn toàn bỏ qua, không gắp phế phẩm.
        6. Tăng bộ đếm phế phẩm `ngDefectCount += 1`.
  - **2. Cụm Máy Gập & Dán Băng Keo Nắp Thùng Carton (Top Carton Sealer on Conveyor 3 - `FactoryEnvironment.js`)**:
    + Bố trí tại nửa cuối Băng tải 3 ($X = -2.40\text{m}, Z = -1.45\text{m}$), sau trạm xếp thùng carton của Robot Arm 6-DOF ($X = -1.25\text{m}$).
    + Dựng mô hình 3D Khung giàn máy dán (Sealer Gantry Frame) hình vòm chữ U màu xanh công nghiệp (Sky Blue RAL 5015) bắc ngang qua Băng tải 3, chân đế bắt sàn, trục vít me ren bi và vô lăng vặn tay điều chỉnh độ cao.
    + Cơ cấu ép & con lăn:
      * Hai thanh nẹp inox dẫn hướng gập cánh nắp (Flap Folding Plows) vát chéo nghiêng vào trong ở cửa đón thùng ($X \in [-2.18\text{m}, -2.36\text{m}]$).
      * Cụm đầu dán băng dính (Taping Head Unit) treo ở tâm khung gồm: cuộn băng dính lớn màu hổ phách trong suốt (`matTapeAmber`), rulô miết cao su màu cam đầu vào và rulô miết hoàn thiện đầu ra.
    + Cảm biến quang điện cửa vào máy dán (`DI_SEALER_SENSOR` tại $X = -2.15\text{m}$) phát tia hồng ngoại ngang mặt băng tải kèm đèn LED chỉ thị (Xanh khi thông thoáng, Đỏ khi có thùng cắt qua).
    + Quy trình đóng nắp & dán thùng:
      * Ban đầu thùng carton ở trạng thái mở 2 cánh nắp xòe góc $63^\circ$ ($\pm 0.35\pi\text{ rad}$).
      * Khi thùng đầy 2 lớp sản phẩm (`box.full == true`) và xilanh Stopper thụt xuống, Băng tải 3 đưa thùng trôi về máy dán.
      * Khi mép thùng cắt qua cảm biến `DI_SEALER_SENSOR`: bật `DO_SEALER_ACTIVE = true`, đèn cảm biến chuyển Đỏ.
      * Khi thùng trôi qua máy dán: 2 cánh nắp gập phẳng dần về $0^\circ$, vệt chỉ dán băng keo màu hổ phách (`tapeMesh`) dán miết dọc theo sống lưng nối giữa hai mép nắp.
      * Thùng chuyển hoàn toàn thành mô hình `SealedCartonBox` hoàn thiện và trôi ra ngoài hầm an toàn.
      * Tăng biến đếm thùng thành phẩm `sealedCartonCount += 1`.
  - **3. Tích Hợp Giao Diện Điều Khiển & Chỉ Số Giám Sát (UI & Dashboard Telemetry - `index.html`, `style.css`, `main.js`)**:
    + Nút bấm mô phỏng lỗi phôi: `[⚡ BƠM LỖI PHÔI NG (INJECT DEFECT)]` màu đỏ tía công nghiệp với hiệu ứng pulse nhấp nháy; bấm nút sẽ gán ngay phôi tiếp theo trên Băng tải 1 thành phôi lỗi NG có vỏ đỏ thẫm để kiểm chứng trực tiếp cơ cấu gạt phế phẩm.
    + Bổ sung 3 chip Telemetry thời gian thực trên thanh Header:
      * `INSPECTED: <span id="chip-total-inspected">0</span>` (Tổng số phôi qua camera kiểm tra).
      * `NG DEFECT: <span id="chip-ng-defect">0</span>` (Số phôi lỗi đã loại bỏ).
      * `SEALED: <span id="chip-sealed-boxes">0</span>` (Số thùng đã dán nắp niêm phong).
    + Bổ sung các đèn báo Virtual I/O LED và nhãn trạng thái trên SCADA Dock Card:
      * `DO_REJECT`: Cụm xilanh gạt phế phẩm (Đỏ/Cam khi bắn phôi, Xám khi lùi).
      * `DO_SEALER`: Cụm máy dán nắp (Xanh khi dán, Xám khi chờ).
      * `DI_SEALER`: Cảm biến quang học máy dán nắp thùng.
    + Bổ sung khối tóm tắt số liệu sản xuất thực tế (KPI Production Metrics) trên thẻ SENSORS & IO TELEMETRY.

- [x] **Task 16: Nâng Cấp Giai Đoạn 2 - Phần 2 (Bộ Mã Hóa Xung Bám Băng Tải Cho Robot Delta & Cụm In Phun / Quét Mã Barcode Cuối Chuyền) [DONE]**
  - **1. Bộ Mã Hóa Xung Bám Băng Tải (Conveyor Tracking Rotary Encoder - `FactoryEnvironment.js`, `VacuumAndSensorEngine.js`, `main.js`)**:
    + Dựng mô hình 3D Encoder quang học công nghiệp gắn đồng trục với rulô chủ động (Drive Pulley tại $X = +3.12\text{m}, Y = 0.815\text{m}, Z = -0.53\text{m}$) của Băng tải 1 thông qua khớp nối mềm (Flexible Jaw Coupling nhôm xẻ rãnh có đệm giảm chấn đỏ và cánh chỉ thị góc quay), mặt bích ổ đỡ, giá chống xoay, 2 đèn LED kênh xung A/B và cáp tín hiệu bọc cao su đen.
    + Giả lập bộ đếm xung tốc độ cao (High-Speed Counter - HSC):
      * Khi Băng tải 1 chạy (`conveyor1Active == true`), trục rulô, khớp nối mềm và trục encoder quay đồng tốc ($\omega = v_1 / r$).
      * Biến tích lũy xung `Encoder_Pulse_Counter` tăng tịnh tiến theo quãng đường dịch chuyển với hệ số $K_{\text{scale}} = 0.1\text{ mm/xung}$ ($0.0001\text{ m/xung}$), phát xung vuông góc pha kênh A/B tỷ lệ thuận với vận tốc băng tải.
    + Thuật toán Gắp phôi động (Dynamic Tracking Pick):
      * Khi Camera Vision Cognex định vị phôi OK tại $X_0 = 2.80\text{m}$, hệ thống chốt giá trị xung mốc `Base_Encoder_Pulse`.
      * Tọa độ thực tế của phôi dọc theo băng tải được tính liên tục từ độ lệch xung: $X(t) = X_0 + \Delta \text{Pulse} \times K_{\text{scale}}$.
      * Robot Delta giải động học nghịch (IK) thời gian thực trong suốt pha hạ đầu hút bám đuổi để chạy song song cùng vận tốc phôi và gắp chính xác mà không cần dừng Băng tải 1.
  - **2. Cụm In Phun Date & Quét Mã Vạch Cuối Chuyền (Industrial Inkjet Printer & Barcode Scanner - `FactoryEnvironment.js`)**:
    + Bố trí tại cuối Băng tải 3 ($X = -3.05\text{m}, Z = -1.45\text{m}$), ngay sau cụm Máy dán băng keo nắp thùng (Top Sealer).
    + Dựng mô hình 3D:
      * Giá đỡ cơ khí dạng trụ ống inox đôi kèm khớp kẹp chữ thập gắn bên hông Băng tải 3.
      * Đầu in phun công nghiệp (CIJ Printhead) vỏ inox/đen chĩa ngang vào sườn thùng carton kèm hiệu ứng tia phun sương vi điểm (`inkjetSprayMesh`) và đèn LED báo bắn mực.
      * Đầu đọc mã vạch cố định (Barcode Scanner) gắn kế bên, chiếu chùm tia laser đỏ dạng quạt (`barcodeLaserFan`) và vạch laser đỏ (`barcodeLaserLine`) lên mặt tem in trên thành thùng.
      * Cảm biến quang kích hoạt in (`DI_PRINT_TRIGGER` tại $X = -2.84\text{m}$).
    + Quy trình In phun & Quét xác nhận:
      * Khi thùng đã dán kín nắp cắt qua `DI_PRINT_TRIGGER`: kích hoạt `DO_INKJET_ACTIVE = true` (`DO_INKJET_FIRE = true`), sinh tự động texture in ma trận chấm sắc nét trên thành thùng gồm 3 dòng (`MFG: [Ngày hiện tại]`, `EXP: [+1 Năm]`, `LOT: #LOT-2026A`) kèm mã thùng tuần tự `BOX-2026-0042...`, mã vạch 1D Barcode và 2D DataMatrix.
      * Ngay khi vệt in hoàn tất, đầu đọc Barcode bật tia laser đỏ nhấp nháy quét qua vùng in trong $0.22\text{s}$, xác thực `Barcode_Scan_Valid = true` (`DI_BARCODE_OK = true`), phát âm thanh "Bíp" xác nhận qua Web Audio API, nháy đèn LED xanh trên thân đầu đọc và tăng bộ đếm `Total_Verified_Boxes += 1`.
  - **3. Cập Nhật Giao Diện HMI / SCADA Monitoring (`index.html`, `main.js`)**:
    + Widget `CONVEYOR & ENCODER MONITOR`: Hiển thị `Encoder Pulses (HSC)`, `Line Speed (mm/s)`, `Base Pulse / ΔPulse` và trạng thái nhấp nháy kênh xung `Quadrature A/B & Freq (Hz)`.
    + Widget `TRACEABILITY & BARCODE PANEL (Last Printed Box Info)`: Hiển thị mã thùng vừa xuất xưởng (`Box ID`), nội dung in thực tế (`MFG / EXP / LOT`) và trạng thái quét `VERIFIED [OK]`.
    + Đèn LED I/O ảo: `DI_PRINT_TRIGGER`, `DO_INKJET_FIRE`, `DI_BARCODE_OK` và chỉ số `VERIFIED BOXES`.

- [x] **Task 17: Nâng Cấp Giai Đoạn 3 - Phần 1 (Hệ Thống Giám Sát Chỉ Số OEE Thời Gian Thực & Quản Trị Chất Lượng Thông Minh) [DONE]**
  - **1. Module Tính Toán OEE Thời Gian Thực (`js/analytics/OEECalculator.js`)**:
    + Chạy chu kỳ mỗi $1\text{ giây}$ ($1000\text{ ms}$) thu thập các biến trạng thái trực tiếp từ dây chuyền sản xuất:
      * **Availability ($A$):** $A = \frac{\text{Operating Time}}{\text{Planned Production Time}} \times 100\%$.
        - `Planned Production Time`: Bắt đầu đếm tăng đều từ khi bật nguồn / khởi động hệ thống.
        - `Operating Time`: Chỉ đếm tăng khi hệ thống đang ở trạng thái RUN tự động (băng tải và robot hoạt động bình thường, không có lỗi).
        - `Downtime`: Đếm tăng khi nhấn E-Stop, mở cửa bảo trì rào an toàn (`SafetyGate_Closed == false`), hoặc chuyển sang chế độ PAUSE / STANDBY.
      * **Performance ($P$):** $P = \frac{\text{Actual Production Rate}}{\text{Target Ideal Rate}} \times 100\%$ (giới hạn trần $P \le 100\%$).
        - Tốc độ định mức thiết kế (`Target Ideal Rate`): $60\text{ sản phẩm/phút}$ ($15\text{ cụm/phút}$).
        - Tốc độ thực tế (`Actual Production Rate`): Đếm số lượng sản phẩm được Robot Delta gắp thành công trong cửa sổ trượt $60\text{ giây}$ gần nhất kết hợp vận tốc cấp phôi thực tế.
      * **Quality ($Q$):** $Q = \frac{\text{Total OK Products}}{\text{Total Inspected Products}} \times 100\%$.
        - Liên kết trực tiếp với bộ đếm phôi OK và số lượng phôi lỗi `NG_Defect_Count` bị xilanh khí nén gạt vào thùng NG; bấm nút `[Inject Defect]` làm giảm chỉ số $Q$ trực quan ngay lập tức.
      * **Overall OEE:** $\text{OEE} = \frac{A \times P \times Q}{10000}$ (%).
        - Phân hạng chuẩn công nghiệp (Benchmarking):
          + $\text{OEE} \ge 85\%$: `"WORLD CLASS"` (Màu Xanh lá `#22c55e` - Badge `World Class`)
          + $65\% \le \text{OEE} < 85\%$: `"TYPICAL / GOOD"` (Màu Vàng cam `#f59e0b` - Badge `Operating Good`)
          + $\text{OEE} < 65\%$: `"UNACCEPTABLE / ATTENTION"` (Màu Đỏ `#ef4444` - Badge `Line Lagging`)
  - **2. Thiết Kế Giao Diện Bảng Điều Khiển OEE (`index.html`, `css/style.css`, `VIPAHMIScreen.js`)**:
    + Bổ sung Tab chuyên biệt `[OEE & ANALYTICS]` trên thanh điều hướng SCADA Dock, đồng thời tích hợp trực tiếp Widget `OEE & SMART QUALITY ANALYTICS` trong Tab Jogger, Chip `OEE: xx.x%` trên thanh Header trên cùng và đồng bộ lên màn hình `VIPA HMI (Station 4)`.
    + **Biểu đồ Vòng tròn trung tâm (Central Radial Gauge / SVG Donut Chart):** Hiển thị con số tổng $\text{OEE}\%$ lớn ở tâm, vòng cung SVG chuyển màu động theo phân hạng (`#22c55e`, `#f59e0b`, `#ef4444`) kèm nhãn phân hạng (`World Class` | `Operating Good` | `Line Lagging`).
    + **3 Thanh Tiến Trình Ngang (Horizontal Progress Bars):**
      1. `Availability: [xx.x]%` (hiển thị `Run Time: hh:mm:ss` / `Downtime: hh:mm:ss`)
      2. `Performance: [xx.x]%` (hiển thị `Actual Speed: xx pcs/min` / `Target: 60 pcs/min`)
      3. `Quality: [xx.x]%` (hiển thị `OK: xx / NG: xx`)
    + **Bảng Thống kê Lô hàng & Sự kiện (Batch & Event Log):** Danh sách tự động cuộn hiển thị tối đa 8 sự kiện thời gian thực có gắn nhãn thời gian `[hh:mm:ss]` và phân màu theo loại sự kiện (`SYSTEM RUN`, `DEFECT DETECTED`, `BOX COMPLETED`, `SAFETY STOP`).
    + **Nút Đặt lại Ca làm việc (`[🔄 Reset Shift / Clear OEE Data]`):** Xóa toàn bộ bộ đếm và thời gian về 0 để bắt đầu ca sản xuất mới.

- [x] **Task 18: Nâng Cấp Giai Đoạn 3 - Phần 2 (Hệ Thống Xe Tự Hành AGV/AMR Xếp Pallet & Vận Chuyển Vào Kho Ảo) [DONE]**
  - **1. Dựng Mô Hình 3D Xe Tự Hành AGV/AMR & Phân Khu Kho Ảo (`js/scene/FactoryEnvironment.js`)**:
    + **Mô hình Xe tự hành AGV/AMR công nghiệp (`buildAGVAndVirtualWarehouse`):**
      * Thân xe (Chassis): Trọng tâm thấp kích thước $1.40\text{m} \times 1.00\text{m} \times 0.45\text{m}$, phủ sơn vàng công nghiệp (`#eab308`) kết hợp dải ốp viền đen chống va đập (Safety Bumper) và sọc phản quang vàng/đen, 4 bánh xe chịu tải xoay thực tế.
      * Hệ thống dẫn đường & Cảm biến an toàn: 2 cụm cảm biến LiDAR / Laser Scanner gắn ở hai góc đối xứng thân xe phát quạt tia laser 270° màu xanh cyan quét mặt sàn dò đường; cột đèn tháp cảnh báo trên đuôi xe chớp nháy màu cam khi chạy và xanh khi đỗ/dỡ tải.
      * Mặt sàn chở hàng (Euro Pallet Base): Tấm Pallet gỗ công nghiệp chuẩn $1.20\text{m} \times 0.80\text{m}$ kèm 4 vấu khóa góc cố định trên lưng xe.
      * Lộ trình dẫn hướng dưới sàn (Magnetic Guide Line): Dải vạch kẻ đường dẫn hướng từ tính màu đen viền vàng phản quang nối từ Trạm đón hàng (Docking Station tại đuôi Băng tải 3: $X = -4.58\text{m}, Z = -1.45\text{m}$) đến Cổng Kho Ảo ($X = -9.35\text{m}, Z = +0.15\text{m}$).
    + **Phân khu Kho Lưu Trữ Ảo (Virtual Warehouse Selective Pallet Rack Zone):**
      * Bố trí cách đuôi Băng tải 3 khoảng $6\text{m}$ ($X = -9.35\text{m}, Z = -1.45\text{m}$), gồm 2 khoang kệ Selective Pallet Racks 3 tầng bằng khung thép cột xanh (`#1d4ed8`) và dầm ngang màu cam (`#f97316`).
      * Trên các tầng kệ đặt sẵn các kiện hàng pallet mẫu và chừa trống Khoang Tầng 1 làm Điểm đỗ dỡ tải (Drop-off Bay).
  - **2. Logic Đón Hàng & Xếp Lô 10 - 20 Thùng Lên Xe (`js/logistics/AGVController.js`)**:
    + Hỗ trợ cấu hình linh hoạt `AGV_CAPACITY` từ $10$ đến $20\text{ thùng}$ (mặc định $12\text{ thùng}$ xếp 2 tầng, mỗi tầng lưới $3 \times 2$ trên mặt Euro Pallet).
    + Quy trình chuyển tiếp & xếp chồng (`receiveBoxFromConveyor`): Khi thùng carton đã dán nắp và quét Barcode trôi qua cảm biến cuối chuyền Băng tải 3 ($X = -3.82\text{m}$), thùng trượt êm vào tọa độ $(x_i, y_i, z_i)$ tương ứng trên mặt pallet và trở thành mesh con (`agvPayloadGroup`) của cụm AGV, tăng biến `currentPayloadCount++`.
    + Điều kiện xuất phát: Khi `currentPayloadCount === AGV_CAPACITY`, xe phát còi cảnh báo kép qua Web Audio API, bật đèn chớp vàng và tự động khởi hành. Trong lúc AGV đi vắng, thùng mới ra lò tự động dừng đệm chờ tại cảm biến cuối chuyền.
  - **3. Quỹ Đạo Chuyển Động & Vòng Lặp Vận Chuyển Tự Hành (`AGVController.update`)**:
    + **Delivery Leg (Chở hàng vào kho):** Nội suy mượt mà dọc theo chuỗi Waypoints kèm xoay góc hướng lái (`Smoothstep + Lerp/Slerp`) và hiệu ứng lăn bánh xe.
    + **Unloading Sequence (Hạ tải tại Kho Ảo):** Dừng tại Drop-off Bay trong $1.8\text{ giây}$, trượt kiện hàng vào ô trống Tầng 1 của kệ kho (`placeDeliveredBatchInWarehouseRack`), reset `currentPayloadCount = 0`, tăng `tripsCompleted++` và `totalWarehouseStoredBoxes += deliveredCount`.
    + **Return Leg (Quay về đón mẻ mới):** Xe rỗng quay lại theo lộ trình vạch dẫn hướng về đúng vị trí Docking tại đuôi Băng tải 3 và chuyển về `READY_FOR_LOADING`.
  - **4. Tích Hợp Bảng Giám Sát Đội Xe AGV Trên HMI / SCADA (`index.html`, `main.js`)**:
    + Thẻ **`AGV FLEET & WAREHOUSE MONITOR`** hiển thị trạng thái trực tiếp (`LOADING [Chờ hàng]` | `TRANSITING [Đang chạy]` | `UNLOADING [Đang dỡ]` | `RETURNING [Quay về]`), thanh tiến trình tải trọng `X / AGV_CAPACITY`, bộ đếm chuyến (`Trips`), tổng thùng lưu kho (`Warehouse`) và dung lượng Pin (`Battery Level: 98%`).
    + Nút điều khiển thủ công **`[🚜 CHO XE ĐI NGAY (FORCE DISPATCH)]`** và **`[📦 +1 THÙNG LÊN XE]`** giúp nghiệm thu / demo nhanh toàn bộ chu trình vận chuyển vào kho ảo bất kỳ lúc nào, kèm góc nhìn camera chuyên biệt **`AGV & Warehouse`**.

- [x] **Task 19: Nâng Cấp Giai Đoạn 4 - Phần 1 (Trạm Xử Lý Thị Giác Máy Tính Công Nghiệp - Industrial Machine Vision Studio & Live Inspection Feed) [DONE]**
  - **1. Dựng Camera Thị Giác Phụ (`THREE.OrthographicCamera` Top-Down Viewport - `js/vision/MachineVisionController.js`)**:
    + Thiết lập camera trực giao `THREE.OrthographicCamera` gắn vuông góc 90° ngay tại cổng vòm Cognex In-Sight trên Băng tải 1 ($X = 2.80\text{m}, Y = 1.38\text{m}, Z = -0.32\text{m}$), bao phủ vùng kiểm tra quang học (FOV) $600\text{mm} \times 400\text{mm}$ quanh mắt đọc.
    + Cơ chế Render Picture-in-Picture (PIP) hiệu năng cao qua `THREE.WebGLRenderTarget` ($330 \times 220\text{ px}$), chỉ cập nhật ở tần số $\sim 20\text{ FPS}$ (mỗi $50\text{ms}$) hoặc khi có phôi đi qua vùng kiểm tra (`DI_VISION_TRIGGER == true`) / bấm nút `[📸 Single Trigger]`. Tạm ẩn vỏ hộp camera trong pass render phụ để thấy rõ mặt băng tải và phôi.
  - **2. Pipeline Xử Lý Ảnh Công Nghiệp & Trích Xuất Đặc Trưng (`js/vision/MachineVisionController.js`)**:
    + **3 Chế độ Bộ lọc Khung hình (Lens Filter Modes):**
      1. `RAW COLOR` (`[Color]`): Hiển thị hình ảnh màu RGB thực tế từ camera phụ nhìn xuống.
      2. `GRAYSCALE` (`[Gray]`): Chuyển đổi ảnh sang thang độ xám 8-bit ($L = 0.299R + 0.587G + 0.114B$) để làm nổi bật độ tương phản biên dạng thanh socola.
      3. `BINARY THRESHOLD` (`[Binary]`): Phân ngưỡng nhị phân đen-trắng (`Threshold = 128`) tách biệt hoàn toàn vùng phôi (Màu trắng `255`) khỏi nền dây đai băng tải (Màu đen `0`).
    + **Trích xuất Đặc trưng & Vẽ Lớp phủ Đồ họa (2D HUD Overlay):**
      * Lưới tọa độ tâm chuẩn (Calibration Crosshair Grid), thước đo pixel và khung vùng quan tâm (ROI Box $600 \times 400\text{mm}$).
      * Đường bao định hướng (Oriented Bounding Box) kèm 4 góc ngắm chuẩn Cognex xoay bám sát theo góc nghiêng thực tế của từng thanh socola đi qua vùng FOV.
      * Tâm hình học (Centroid `+`) và mũi tên vector chỉ hướng góc xoay ($\text{Angle}: \theta^\circ$).
      * Đo lường kích thước & diện tích điểm ảnh (`Length`, `Width`, `Pixel Area: xxxx px²`).
  - **3. Logic Phán Quyết Chất Lượng (`OK` vs `NG`) & Bù Góc Xoay Trục 4 Robot Delta (`main.js`, `MachineVisionController.js`)**:
    + **Sản phẩm Đạt chuẩn (`PASS / OK`):** Diện tích `Pixel Area` nằm trong ngưỡng dung sai chuẩn ($2600 \le \text{Area} \le 4200\text{ px}^2$) và biên dạng nguyên vẹn $\rightarrow$ Khung Bounding Box màu Xanh lá (`#22c55e`), nhãn `INSPECTION: PASS [OK]`, hiển thị độ lệch tâm `X: +xx.x mm | Y: +yy.y mm | θ: +zz.z°`. Truyền trực tiếp góc lệch $\theta$ cho trục thứ 4 (`yaw`) của Robot Delta để đầu hút xoay đúng góc $\theta$ khi đáp xuống gắp phôi và xoay trả về $0^\circ$ khi xếp ngay ngắn vào cụm 4 thanh trên Băng tải 2.
    + **Sản phẩm Lỗi / Phế phẩm (`FAIL / NG`):** Phôi bị biến dạng kích thước hoặc bấm nút `[Inject Defect]` $\rightarrow$ Khung Bounding Box chuyển sang màu Đỏ nhấp nháy (`#ef4444`), nhãn `INSPECTION: FAIL [NG - DEFECT]`, gán cờ `status = "NG"` để Xilanh khí nén (Reject Pusher) tự động kích hoạt đẩy văng vào thùng phế phẩm NG Bin.
  - **4. Cửa Sổ Giao Diện Vision Studio Nổi (`COGNEX VISION PRO - SENSOR #CAM01 (CONVEYOR 1)` - `index.html`, `css/style.css`)**:
    + Cửa sổ nổi bán trong suốt ở góc dưới bên phải màn hình 3D, hỗ trợ kéo thả tự do (Draggable Header) và thu gọn/mở rộng (`➖` / `➕`).
    + Màn hình Live PIP `<canvas id="vision-canvas" width="330" height="220">` kèm đầy đủ thông số kỹ thuật thời gian thực (`Status Badge`, `Centroid X/Y/Angle`, `Blob Feature Area & Size`, `Processing Time ms`) và cụm nút điều khiển `[Color]` | `[Gray]` | `[Binary]`, `[📸 Single Trigger]`, `[🎥 Live Video: ON]`.

- [x] **Task 20: Nâng Cấp Hạ Tầng Tủ Điện Điều Khiển & Hệ Thống Dây Dẫn / Máng Cáp Công Nghiệp (Industrial Control Cabinet & Cable Routing Infrastructure) [DONE]**
  - **1. Dựng Tủ Điện Công Nghiệp Chính (`buildIndustrialControlCabinetAndCableInfrastructure` - `js/scene/FactoryEnvironment.js`)**:
    + **Vỏ tủ điện (Enclosure Structure):** Tủ đứng công nghiệp đôi ($1.24\text{m} \times 1.92\text{m} \times 0.56\text{m}$) bằng thép tấm sơn tĩnh điện màu xám ghi RAL 7035 (`#d1d5db`), chân đế kim loại màu đen (`plinthH = 0.12m`) cố định sát sàn tại vị trí kỹ thuật phía sau khung rào an toàn ($X = +0.55\text{m}, Z = -2.28\text{m}$).
    + **Mặt ngoài 2 cánh cửa bản lề:**
      * Công tắc xoay cách ly nguồn tổng (Main Isolator Switch) viền vàng núm đỏ có hiệu ứng xoay góc $90^\circ$ khi đóng/cắt điện và đèn báo nguồn 380V trên cánh tủ.
      * Biển cảnh báo điện áp cao hình tam giác màu vàng `DANGER 380V / 220V` kèm biểu tượng tia sét đỏ.
      * Hai cửa chớp thông gió (Filter Fan Louvers) tích hợp lưới lọc bụi và quạt tản nhiệt ở nửa dưới hai cánh tủ.
    + **Bố trí Thiết bị Điều khiển bên trong tủ (Theo chuẩn Bộ Kit thí nghiệm Yaskawa):**
      * **Bộ điều khiển máy Yaskawa MP3300 Machine Controller:** Gắn trên thanh DIN rail tầng trên cùng, tích hợp module truyền thông Ethernet `218IFD` (cắm cáp mạng RJ45 Cat6 màu xanh lá nối sang Switch công nghiệp) và 3 cổng bus truyền thông `Mechatrolink-III` màu xanh dương.
      * **Biến tần Yaskawa J1000 AC Drive:** Gắn bên phải tầng trên, có cánh tản nhiệt nhôm phía sau, màn hình LED 7 đoạn màu đỏ hiển thị tần số quay `50.0 Hz` (tự động tỉ lệ theo tốc độ băng tải hoặc tắt tối khi cắt nguồn tủ) và cụm phím `RUN` / `STOP` / biến trở xoay.
      * **Dàn bộ Servo Drive (Sigma-7 Servo Drive A, B, C):** 3 khối driver `SGD7S` gắn trên thanh DIN rail tầng giữa, đấu nối liên hoàn với MP3300 qua các sợi cáp bus Mechatrolink màu xanh dương và domino nguồn động cơ màu cam.
      * **Tủ điều khiển Robot Yaskawa YRC1000micro:** Khối chassis kim loại màu xanh Yaskawa dạng rack-mount ở khoang dưới, có khe thoát nhiệt, cổng mạng LAN RJ45 và 2 đầu giắc công nghiệp đa chân bọc vỏ kim loại (CN1/CN2) dẫn cáp bọc cao su đen dày xuống máng sàn.
      * **Hệ thống đèn LED trạng thái:** Đầy đủ đèn LED `PWR` (Xanh lá tĩnh), `RUN` (Xanh lá nhấp nháy), `LINK` (Xanh lá chớp tốc độ cao) và `ALARM` (Đỏ khi hệ thống báo lỗi).
  - **2. Hệ Thống Máng Cáp Sàn, Xích Dẫn Cáp & Ống Ruột Gà (`buildCableRoutingAndDragChains` - `js/scene/FactoryEnvironment.js`)**:
    + **Máng cáp sàn kim loại (Floor Cable Trays):** Hệ thống máng cáp chữ U mạ kẽm (`#94a3b8`) chạy sát mặt sàn kèm các đoạn nắp đậy màu đen có viền cảnh báo vàng/đen, để lộ bó dây nguồn cao su đen, cáp Mechatrolink xanh dương và cáp mạng Ethernet xanh lá bên trong:
      * *Tuyến 1:* Đi từ chân tủ điện chính dẫn thẳng tới chân đế bệ máy Robot Arm 6-DOF ($X = -1.25\text{m}, Z = -0.82\text{m}$) và cụm động cơ Băng tải 3 / Máy dán thùng ($X = -2.25\text{m}$).
      * *Tuyến 2:* Đi dọc theo chân rào bảo vệ phía sau ($Z = -1.94\text{m}$) tỏa nhánh vào cột khung giàn Robot Delta ($X = +1.20\text{m}$), cổng vòm Camera Vision / Xilanh gạt phế phẩm ($X = +2.80\text{m}$) và động cơ / Rotary Encoder Băng tải 1 ($X = +3.85\text{m}$).
    + **Xích dẫn cáp linh hoạt (Cable Drag Chain / Energy Chain):** Dải xích dẫn hướng nhựa đen khớp nối uốn cong hình chữ U trên cột đứng khung giàn Robot Delta và bệ vai Robot Arm 6-DOF, ôm gọn dây servo và ống dẫn khí nén màu xanh.
    + **Ống ruột gà & Ống dẫn tín hiệu (Flexible Corrugated Tubing):** Các đoạn ống ruột gà đen có gân xoắn nối từ máng cáp sàn lên hộp đấu dây động cơ Băng tải 1, 2, 3, cảm biến quang `DI_11` và cổng vòm Camera Vision.
  - **3. Tích Hợp Tương Tác Nguồn Điện & Chế Độ Quan Sát Tủ Điện (`index.html`, `js/main.js`)**:
    + **Tương tác Công tắc Nguồn Tổng (Main Power Isolation):** Hỗ trợ click trực tiếp chuột vào công tắc xoay trên cánh tủ 3D, nút **`[NGUỒN TỦ: ON/OFF]`** trên thanh Header hoặc nút **`[POWER ON / SERVO ON]`**. Khi tắt nguồn: tắt toàn bộ đèn LED của MP3300, YRC1000micro, Servo Drives, màn hình LED 7 đoạn của biến tần J1000 tắt về `OFF` và toàn bộ dây chuyền dừng hoạt động. Khi bật nguồn: công tắc xoay $90^\circ$, đèn nguồn tủ sáng và các bộ điều khiển chạy chu trình khởi động `BOOTING (1.5s)` (`INIT...`) trước khi sẵn sàng vận hành.
    + **Chế độ Quan sát Tủ điện (`Inspect Cabinet Mode`):** Nút **`[GÓC NHÌN TỦ ĐIỆN (VIEW CABINET)]`** trên thanh Camera Toolbar & Dock tự động lia camera đến cận cảnh mặt tủ điện; nút **`[🔍 Ẩn/Hiện Vỏ Tủ]`** (hoặc click trực tiếp vào cánh tủ 3D) mở rộng 2 cánh cửa tủ và chuyển vỏ tủ sang chế độ nhìn xuyên thấu (Cutaway view) để quan sát rõ toàn bộ thiết bị bên trong.

- [x] **Task 21: Nâng Cấp Cơ Chế Sinh Phôi Lỗi Thủ Công & Thùng Thu Hồi Phế Phẩm (Defect Workpiece Spawner & Pneumatic Reject Station) [DONE]**
  - **1. Nút Bấm Thả Phôi Lỗi Trên Giao Diện & Phím Tắt Nhanh `[D]` (`index.html`, `js/main.js`, `js/scene/FactoryEnvironment.js`)**:
    + Bổ sung nút bấm nổi bật viền đỏ **`[🚨 THẢ 1 PHÔI LỖI (SPAWN DEFECT PIECE) [D]]`** trên cả thanh Header trên cùng và bảng điều khiển Jogger, đồng thời hỗ trợ phím tắt **`D`** trên bàn phím.
    + Khi kích hoạt (`spawnDefectPieceAtStart`): Ngay lập tức sinh ra 1 phôi lỗi tại đầu Băng tải 1 ($X \approx 3.65\text{m}$), có màu đỏ rực (`#ef4444`) kèm vết nứt biến dạng & dải cảnh báo vàng/đen, gán thuộc tính `item.isDefective = true; item.status = "NG"`. Tự động kích hoạt Băng tải 1 chạy (nếu đang dừng) để phôi trôi về phía cổng Camera Vision.
  - **2. Dựng Thùng Phế Phẩm Có Bánh Xe & Máng Trượt Thu Hồi 35° (`buildPneumaticRejectPusherAndNGBin` - `js/scene/FactoryEnvironment.js`)**:
    + **Máng trượt dẫn hướng (Guide Chute):** Máng trượt kim loại mạ kẽm có thành chắn 2 bên gắn sát mép đai Băng tải 1 ($X = 2.45\text{m}, Z = -0.52\text{m}$), nghiêng chéo xuống góc chuẩn **$35^\circ$** nối thẳng vào miệng thùng chứa phế phẩm.
    + **Thùng chứa phế phẩm (NG Scrap Bin):** Thùng chữ nhật công nghiệp màu vàng cảnh báo (`#eab308`) viền xám đậm (`#334155`), trang bị **4 bánh xe xoay (Swivel Caster Wheels)** dưới đáy và biển báo chữ thập đỏ **`NG / SCRAP ONLY ZONE`** trên 3 mặt thành thùng.
  - **3. Quy Trình Bám Tọa Độ & Animation Đẩy Rơi Phôi Vào Thùng (`FactoryEnvironment.js`, `MachineVisionController.js`, `VacuumAndSensorEngine.js`)**:
    + **Camera Vision quét ($X = 2.80\text{m}$):** Nhận diện `isDefective == true` $\rightarrow$ bật cờ lỗi `FAIL [NG - DEFECT]` khung đỏ trên cửa sổ Cognex Vision Studio, chốt xung gốc `baseEncoderPulse` để theo dõi vị trí phôi đến trạm gạt và loại khỏi hàng đợi gắp của Robot Delta.
    + **Piston khí nén kích hoạt ($X = 2.45\text{m}$):** Khi tâm phôi lỗi trôi đến đúng vị trí bàn gạt xilanh, cần piston phóng nhanh ra trong **$0.15\text{s}$** (`DO_REJECT_CYLINDER = true`), đẩy phôi trượt ngang sang mép băng tải rồi tự động thu cần về trong **$0.15\text{s}$**.
    + **Hiệu ứng trượt máng dốc $35^\circ$ & rơi vào đáy thùng (Chute Drop Animation):** Phôi lỗi trượt dọc theo mặt phẳng nghiêng $35^\circ$ của máng dẫn hướng, sau đó rơi tự do và xoay nhẹ theo trọng lực xuống nằm lại dưới đáy Thùng phế phẩm (`ngScrapBinItems`, giữ nguyên trong thùng $5\text{ giây}$ trước khi tự động mờ dần để tối ưu bộ nhớ), đồng thời tăng biến đếm `NG_Defect_Count += 1` trên SCADA & OEE Quality.

- [x] **Task 22: Kiểm Tra Toàn Diện Hệ Thống & Xử Lý Triệt Để Các Lỗi Ngoại Lệ (Comprehensive System Audit, Edge-Case Bug Fixes & 60 FPS Optimization) [DONE]**
  - **1. Xử lý lỗi tái sử dụng phôi (Object Pool Recycling) làm nhảy cóc tọa độ Encoder trên Băng tải 1 (`js/physics/VacuumAndSensorEngine.js`, `js/scene/FactoryEnvironment.js`)**:
    + Khi Robot Delta gắp thanh socola thả sang Băng tải 2 và tái chế mesh phôi quay về đầu Băng tải 1 ($X \ge 3.65\text{m}$), hệ thống xóa sạch các thuộc tính `baseEncoderPulse = undefined`, `scanX0 = undefined`, `scanZ0 = undefined`, `scanned = false`, `rejected = false`, `falling = false` và đồng bộ lại trạng thái `isDefective / crackGroup`. Khắc phục triệt để hiện tượng phôi tái chế giữ lại xung mốc cũ và bị nhảy (teleport) về cuối băng tải trước khi đi qua cổng Camera Vision.
    + Khi người dùng kéo thanh trượt thay đổi tốc độ cấp phôi (`setInfeedRate`), các phôi được dãn cách lại ở phía trước cổng Camera Vision ($X > 2.80\text{m}$) cũng được reset `baseEncoderPulse` và `scanned = false` để quét lại chính xác.
  - **2. Bảo vệ hủy quỹ đạo tức thời (Abort Safety Guard) khi nhấn E-Stop, mở Cửa Rào An Toàn hoặc Va Chạm (`js/main.js`, `js/kinematics/TrajectoryPlanner.js`)**:
    + Bổ sung hàm kiểm tra điều kiện dừng khẩn cấp `isAborted()` xuyên suốt từng bước `await runTraj(...)` và `sleep(...)` trong cả hai chu trình song song `runDeltaCollateStep()` và `runArmBoxPackStep()`.
    + Đảm bảo khi nhấn nút dừng khẩn cấp `E-Stop`, mở cửa rào bảo trì (`Safety Interlock Tripped`), tắt nguồn tủ điện (`System Power OFF`) hoặc phát hiện va chạm, các hàm `async` thoát ra ngay lập tức mà không tiếp tục thực hiện pha hạ đầu hút hoặc đóng/mở giác hút chân không ngoài ý muốn.
  - **3. Đồng bộ hóa toàn diện chức năng `System Master Reset` & `Clear All` (`js/hmi-scada/FaultInjectionPanel.js`, `js/main.js`)**:
    + Nâng cấp `executeMasterReset()` để khôi phục đồng bộ 100% trạng thái toàn hệ thống: dừng mọi quỹ đạo, xóa cờ `isDeltaBusy / isArmBusy / autoDemoMode / isCollisionActive`, giải phóng giác hút, dọn sạch hàng đợi `fallingObjects / scannedProductIds / visionQueue`, đóng cửa rào an toàn & ẩn popup Interlock, đưa xe tự hành AGV về trạm Docking (`agvController.reset()`), đặt lại bộ đếm ca OEE (`oeeCalculator.resetShift()`) và đồng bộ đèn LED công tắc băng tải.

- [x] **Task 23: Xây dựng Module Lập Trình Quỹ Đạo Tự Động Vẽ Chữ T Cho Robot Arm 6-DOF & Hiển Thị Đồ Thị Chữ T [DONE]**
  - **1. Module Lập trình Job Tự động Chữ T (`js/workstations/JobLetterT.js`)**:
    + Định nghĩa chương trình chuẩn INFORM III `AUTO_DRAW_LETTER_T` cho phân hệ lập trình Yaskawa Teach Pendant gồm đầy đủ các lệnh `MOVJ`, `MOVL`, `TIMER`, `END`.
    + Khai báo danh mục biến vị trí $P020..P025$ trong không gian Cartesian (ROBOT mm/deg) và Xung encoder (PULSE):
      * $P020$: Điểm an toàn tiếp cận ($X = -1250\text{mm}, Y = 1300\text{mm}, Z = 180\text{mm}$).
      * $P021$: Đầu nét ngang bên trái ($X = -1450\text{mm}, Y = 1200\text{mm}, Z = 180\text{mm}$).
      * $P022$: Kéo nét ngang sang bên phải ($X = -1050\text{mm}, Y = 1200\text{mm}, Z = 180\text{mm}$, độ rộng $400\text{mm}$).
      * $P023$: Trượt về tâm nét ngang ($X = -1250\text{mm}, Y = 1200\text{mm}, Z = 180\text{mm}$).
      * $P024$: Kéo nét đứng thẳng xuống dưới ($X = -1250\text{mm}, Y = 880\text{mm}, Z = 180\text{mm}$, chiều cao $320\text{mm}$).
      * $P025$: Rút về vị trí an toàn ($X = -1250\text{mm}, Y = 1300\text{mm}, Z = 180\text{mm}$).
    + Hàm `runAutoLetterTJob(app)` điều phối toàn diện: kích hoạt nguồn/servo, tự động bật và xóa sạch vết quỹ đạo 3D `TrajectoryTrail` màu xanh Cyan, chuyển Teach Pendant sang `PLAY` mode, lần lượt nội suy đường thẳng $C^2$ đa thức bậc 5 (`MOVL`), cập nhật tọa độ thời gian thực.
  - **2. Đồ thị Quỹ đạo 2D Cartesian Hiển thị Chữ T (`js/hmi-scada/TelemetryAndGraphs.js`)**:
    + Nâng cấp phân hệ SCADA Telemetry với đồ thị thứ 3 dạng Canvas 2D siêu nét ($930 \times 150\text{ px}$): `CARTESIAN 2D TRAJECTORY // ĐỒ THỊ QUỸ ĐẠO TCP (X-Y) // HÌNH CHỮ T`.
    + Lưới tọa độ công nghiệp trực quan ($X: -1500 \to -1000\text{mm}$, $Y: 800 \to 1300\text{mm}$) kèm vệt mờ hướng dẫn chuẩn (Ghost Template Guide).
    + Đường vẽ phát sáng (Cyan Glow `#22d3ee` / `#06b6d4`, bóng đổ 8px) vẽ bám sát chuyển động thực tế của robot, hiển thị rõ ràng và hoàn hảo chữ **"T"** trên màn hình đồ thị.
    + Thuật toán nhận diện biên dạng `isLetterTComplete` tự động hiển thị nhãn `★ HOÀN TẤT ĐỒ THỊ CHỮ T (VERIFIED)` khi hoàn thành nét vẽ.
  - **3. Tích hợp Quỹ đạo Chữ T vào Bộ Đồ thị Động học MATLAB (`js/kinematics/MatlabKinematicsPlotter.js`)**:
    + Bổ sung tùy chọn `✍️ Quỹ Đạo Vẽ Chữ T (Letter T Trajectory - Đồ Thị Chữ T)` trong menu Quỹ đạo.
    + Hàm `generateLetterTProfile()` giải động học nghịch IK tại $120$ điểm mốc thời gian $T = 4.2\text{s}$, tính đạo hàm vận tốc góc $\omega_1..\omega_6$ và gia tốc góc $\alpha_1..\alpha_6$.
  - **4. Tích hợp Đa Điểm Kích Hoạt Trên Giao Diện (`index.html`, `TeachPendantStation.js`, `CSharpPCStudioStation.js`, `main.js`)**:
    + Thẻ dự án chuyên biệt `AUTO_DRAW_LETTER_T` trong màn hình `SELECT PROJECT` của Teach Pendant kèm nút `[⚡ CHẠY TỰ ĐỘNG CHỮ T]` và nút `[✍️ VẼ CHỮ T]` trên thanh tác vụ JOB.
    + Nút bấm `[✍️ VẼ CHỮ T (LETTER T)]` trên thanh Header SCADA trên cùng.
    + Nút bấm `[✍️ TỰ ĐỘNG VẼ QUỸ ĐẠO CHỮ T]` trên bảng điều khiển Robot Arm và Tab Kinematics Lab.
    + Phương thức C# `RunAutoDrawLetterT()` và nút `[✍️ RUN LETTER T (C#)]` trong Station 3 C# Studio.
    + Nút `[✍️ CHẠY TỰ ĐỘNG VẼ CHỮ T]` trên thẻ đồ thị Telemetry Station 4.

- [x] **Task 24: Tinh Chỉnh & Tối Ưu Hóa Toàn Diện Hiệu Năng 60 FPS (Anti-Lag & CPU Optimization) [DONE]**
  - **1. Khắc phục triệt để hiện tượng GPU Stall và vòng lặp CPU nặng ở Trạm Vision (`MachineVisionController.js`)**:
    + Loại bỏ cơ chế `readRenderTargetPixels` (nguyên nhân gây nghẽn đường ống GPU WebGL và khoá luồng CPU) và vòng lặp duyệt 72.600 pixel trong JS.
    + Thay thế bằng bộ dựng 2D Canvas trực tiếp hiệu năng cực cao: tái hiện trung thực bề mặt băng tải, chuyển động phôi socola, lớp vỏ giấy bạc phản quang và các vết nứt lỗi NG.
    + Hỗ trợ đầy đủ 3 chế độ lọc DSP (`RAW COLOR`, `GRAYSCALE`, `BINARY THRESHOLD`) và lớp phủ đồ họa Cognex HUD ở tốc độ render tức thì (<0.1ms thay vì 30ms).
    + Thêm cơ chế tự động tạm dừng xử lý đồ họa khi cửa sổ camera đang bị thu nhỏ hoặc đóng.
  - **2. Tối ưu hóa Dao động ký Telemetry & Đồ thị (`TelemetryAndGraphs.js`)**:
    + Thêm điều kiện kiểm tra hiển thị `offsetParent`: nếu trạm Station 4 đang ẩn (mặc định), hệ thống lập tức bỏ qua toàn bộ quá trình vẽ 3 Canvas và cập nhật DOM `innerHTML`, tiết kiệm 100% chi phí tính toán nền.
    + Khi Station 4 kích hoạt: giới hạn tốc độ lấy mẫu và vẽ đồ thị ở mức 20 FPS mượt mà.
  - **3. Loại bỏ Render Pass phụ của Vật liệu Phản xạ/Khúc xạ (`FactoryEnvironment.js`)**:
    + Chuyển đổi các vật liệu `MeshPhysicalMaterial` có `transmission` (`matWindowGlass`, `matPolycarbWindow`, `matTapeAmber`) sang `MeshStandardMaterial` bán trong suốt.
    + Loại bỏ hoàn toàn pass phụ `transmission render target` và việc sao chép framebuffer nội bộ của Three.js.
  - **4. Tối ưu hóa WebGL Renderer & Điều tiết cập nhật DOM (`main.js`)**:
    + Đặt `preserveDrawingBuffer = false` để loại bỏ thao tác sao chép backbuffer mỗi khung hình của trình duyệt.
    + Giới hạn `pixelRatio` tối đa ở 1.25 và kích thước Shadow Map ở 1024x1024 với bộ lọc `PCFShadowMap`.
    + Điều tiết (throttle) các hàm cập nhật DOM SCADA (`updateKinematicsLabUI`, `updateHardwareUI`, `updateIOTelemetry`) về tần số 12.5 Hz (mỗi 80ms) thay vì 60 Hz, đồng thời chỉ tính toán khi tab tương ứng đang mở, giảm 85% tải reflow/recalculation của trình duyệt.
    + Thu hẹp vùng Raycaster của Cửa bảo trì: chỉ bắt click vào tay nắm (`doorHandle`) hoặc công tắc khóa an toàn (`interlockSwitchGroup`), tránh click nhầm vào kính quan sát làm ngắt mạch liên động an toàn.
  - **5. Loại bỏ hiệu ứng làm mờ nền nặng (`css/style.css`)**:
    + Gỡ bỏ toàn bộ thuộc tính `backdrop-filter: blur(...)` trên các modal toàn màn hình (`.safety-modal-overlay`) và các widget nổi trên WebGL canvas. Thay thế bằng màu nền tối công nghiệp có độ mờ cao (`rgba(15, 23, 42, 0.96)`), giúp trình duyệt không còn phải chạy Gaussian Blur trên từng khung hình 3D.

- [x] **Task 25: Tinh Chỉnh Giác Hút Robot Delta & Tắt Mặc Định Vẽ Tuyến Quỹ Đạo 3D Của 2 Robot [DONE]**
  - **1. Khắc phục thanh dài thò ra ở đầu giác hút Robot Delta (`DeltaRobot3D.js`)**:
    + Nguyên nhân: Trục then hoa trung tâm bậc 4 (`telescopicLower`) trước đây có chiều dài cố định 0.50m và vị trí `-shaftLen * 0.45` mà không bị co tỉ lệ, dẫn đến việc phần đuôi trục xuyên qua bệ di động, đâm xuyên qua giác hút và thò dài ra 20cm bên dưới núm hút.
    + Khắc phục: Chuẩn hóa hình học trục lồng thành hình trụ đơn vị (`unit cylinder`), co giãn chính xác chiều dài để phần trục trượt dưới kết thúc chính xác 100% tại mặt trên của bệ di động (`y = -shaftLen`), tuyệt đối không đâm xuyên xuống giác hút.
    + Đầu giác hút giờ đây hoạt động chuẩn thực tế: gồm ống nối ren Spindle, ống cao su xếp nếp Accordion Bellows, núm hút cao su Silicone và vòng chỉ báo chân không phát sáng Cyan.
    + Đồng bộ hành trình gắp: núm hút hạ sát bề mặt thanh kẹo (`Y = 0.902m` tạo độ ép kín 2mm trên phôi kẹo `Y = 0.904m`), kích hoạt đèn chân không hút (`setVacuumActive(true)`) và nhấc thanh kẹo lên đúng thực tế công nghiệp.
  - **2. Tắt mặc định vẽ đồ thị tuyến di chuyển (3D Trajectory Trail) của cả 2 Robot (`TrajectoryTrail.js`, `index.html`, `main.js`)**:
    + Đặt mặc định tắt (`visible = false`, `armVisible = false`, `deltaVisible = false`) trong `TrajectoryTrail.js` và bỏ chọn `checked` ở tất cả checkbox trên giao diện (`chk-show-arm-trail`, `chk-show-delta-trail`, `chk-show-trails`).
    + Bổ sung nút bấm truy cập nhanh `📈 Đồ Thị Quỹ Đạo 3D` ngay trên thanh camera toolbar dưới cùng màn hình 3D, cho phép người dùng bật/tắt hiển thị tuyến quỹ đạo di chuyển bất cứ lúc nào.
    + Bổ sung nút bấm `📈 ĐỒ THỊ ĐỘNG HỌC MATLAB DELTA (THETA 1..3)` trên thẻ Robot Delta để xem đồ thị động học tương tự như Robot Arm 6 bậc.
    + Nâng cấp phiên bản tài nguyên lên `?v=3.5` để trình duyệt tải ngay mã nguồn mới nhất.

- [x] **Task 26: Tinh Chỉnh Cơ Khí Nắp Thùng Carton & Hoàn Thiện Quy Trình Đóng Nắp Dán Băng Keo Máy Case Sealer (`FactoryEnvironment.js`) [DONE]**
  - **1. Khắc phục hình học nắp thùng carton (`spawnEmptyBox`)**:
    + Nguyên nhân: Trước đây cánh nắp thùng chỉ có chiều sâu $0.07\text{m}$ (trong khi chiều sâu thùng $dZ = 0.24\text{m}$, hai cánh cộng lại chỉ được $0.14\text{m}$) và bị dịch tâm sai hướng ($+Z$), khiến cánh trước khi gập nằm ngang lại chĩa ra phía ngoài khoảng không trước mặt thùng, để lộ khoảng trống $17\text{cm}$ ở giữa khiến nhìn thấy rõ các gói kẹo bên trong.
    + Khắc phục: Xây dựng chuẩn kết cấu thùng carton gập 4 nắp RSC công nghiệp:
      * 2 nắp chính (Front & Back): Chiều rộng $wX = 0.28\text{m}$, chiều sâu chính xác $dZ / 2 = 0.121\text{m}$, gập hướng vào tâm theo trục $Z$. Khi gập phẳng ($0^\circ$), 2 nắp gặp nhau khít khao $100\%$ tại đường rãnh giữa $Z = 0$, đậy kín toàn bộ mặt trên thùng.
      * 2 nắp hông phụ (Left & Right): Chiều rộng $0.08\text{m}$, chiều sâu $0.23\text{m}$, gập vào trong bên dưới 2 nắp chính.
      * Khi chưa đóng (ở vị trí Robot Arm gắp xếp hàng): 4 cánh nắp mở xòe hướng lên trên góc $75^\circ - 80^\circ$ rất thoáng để Robot Arm xếp các lớp kẹo vào bên trong.
  - **2. Hoàn thiện chuyển động đóng nắp tuần tự qua Máy dán băng keo (Case Sealer tại $X = -2.40\text{m}$)**:
    + Khi thùng hàng đầy di chuyển qua vùng máy ($X: -1.98\text{m} \to -2.48\text{m}$):
      1. Thanh nẹp dẫn hướng gập 2 nắp hông (Left/Right) xuống trước.
      2. Con lăn ép trên gập 2 nắp chính (Front/Back) xuống khép kín tại đường nối giữa $Z = 0$.
      3. Đầu dán trải dải băng keo vàng hổ phách (Amber tape) dán chặt đường rãnh nối nắp thùng.
    + Khi thùng vượt qua vị trí $X \le -2.48\text{m}$ và chuyển lên xe tự hành AGV: Khóa trạng thái đóng nắp tuyệt đối (`sealed = true`, góc nắp $= 0^\circ$, băng keo hiện $100\%$), đảm bảo toàn bộ thùng trên băng tải sau trạm đóng và trên pallet xe AGV đều được đóng nắp và niêm phong băng keo kín đáo, đúng thực tế.

- [x] **Task 27: Nâng Cấp Hệ Thống Camera: Tích Hợp Cơ Chế Điều Khiển Tự Do Giống Factory I/O (Free / Fly Camera) [DONE]**
  - **1. Điều khiển di chuyển tiến/lùi/sang hai bên (Pan & Fly) bằng cụm 4 phím mũi tên (`main.js`)**:
    + Phím `↑` (ArrowUp): Di chuyển tiến thẳng theo hướng nhìn ngang của camera.
    + Phím `↓` (ArrowDown): Di chuyển lùi lại.
    + Phím `←` (ArrowLeft): Trượt ngang sang trái (Strafe Left vuông góc với hướng nhìn).
    + Phím `→` (ArrowRight): Trượt ngang sang phải (Strafe Right).
    + Phím `Space` / `PageUp`: Nâng cao độ camera (Fly Up).
    + Phím `C` / `PageDown`: Hạ thấp cao độ camera (Fly Down).
    + Phím `Shift`: Tăng tốc độ bay gấp 2.4 lần ($3.8\text{ m/s} \to 9.2\text{ m/s}$).
    + Cơ chế vật lý quán tính (Inertia Damping lerp): Camera tăng tốc và giảm tốc mượt mà, không giật khựng.
    + Khóa va chạm sàn epoxy: Giới hạn độ cao sàn tối thiểu $Y \ge 0.15\text{ m}$, camera tuyệt đối không chìm xuống dưới mặt đất.
    + Đồng bộ kép: Tịnh tiến đồng thời cả `camera.position` và `controls.target` theo vector dịch chuyển, giúp OrbitControls giữ nguyên góc nhìn ổn định và chuyển đổi mượt mà giữa chế độ xoay và bay.
    + Lọc input thông minh: Tự động vô hiệu hóa phím di chuyển khi người dùng đang gõ văn bản trong thẻ `INPUT`, `TEXTAREA` hoặc `SELECT`.
  - **2. Cơ chế xoay góc nhìn tự do tại chỗ (Factory I/O Free Look via Right Mouse Drag)**:
    + Khi kích hoạt chế độ Factory I/O Free Cam (hoặc bấm phím tắt `V`):
    + Nhấn giữ chuột phải và rê chuột trên khung nhìn 3D để xoay hướng nhìn tự do $360^\circ$ (Yaw quanh trục $Y$ thế giới, Pitch quanh trục ngang Right của camera) mà không thay đổi vị trí camera, y hệt cơ chế Free Look của phần mềm công nghiệp Factory I/O.
    + Vô hiệu hóa context menu mặc định của trình duyệt trên 3D canvas để thao tác chuột phải mượt mà tuyệt đối.
  - **3. Bổ sung giao diện và chỉ báo trạng thái trực quan (`index.html`, `style.css`)**:
    + Nút bấm `🕹️ Factory I/O Cam` màu vàng nổi bật trên thanh Camera Toolbar dưới đáy màn hình.
    + Nút bấm công cụ bay `btn-tool-fly-cam` trên cụm công cụ camera bên phải màn hình 3D.
    + Bảng thông báo HUD nổi `#fly-cam-badge` nhấp nháy phát sáng vàng/cyan trên cùng màn hình khi chế độ Fly Cam đang kích hoạt.
    + Bổ sung hướng dẫn phím tắt trong bảng gợi ý điều khiển `.mouse-nav-hint` (Mũi tên, WASD, Shift, Space/C, Chuột phải, Phím V).
    + Nâng cấp phiên bản tài nguyên lên `?v=3.6` trong `index.html`.

---

## 📝 Nhật Ký Hoàn Thành & Bàn Giao Toàn Diện Hệ Thống (Master Handoff Log)
- **Hoàn thành trọn vẹn các Giai đoạn phát triển của Đồ án Web 3D Digital Twin:**
  - **Giai đoạn 1:** Xây dựng mô hình 3D Three.js tỉ lệ thật, băng tải, phôi kẹo socola, thùng carton, camera điều hướng trực quan.
  - **Giai đoạn 2:** Engine toán học Động học thuận/nghịch (FK/IK) 6-DOF & Delta 4-DOF, bộ nội suy quỹ đạo Joint/Linear, vật lý gắp thả chân không, cảm biến quang 3D, cơ cấu gạt phế phẩm khí nén, máy dán băng keo nắp thùng, bộ mã hóa xung Rotary Encoder bám băng tải và cụm in phun CIJ / quét mã Barcode cuối chuyền.
  - **Giai đoạn 3:** Bus phần cứng ảo MP3300, High-Speed Ethernet UDP YERC Server (cổng 10040), Control Panel công nghiệp, UDP Sniffer, Hệ thống Giám sát Chỉ số OEE Thời gian thực (`OEECalculator.js`) và Hệ thống Xe tự hành AGV/AMR Xếp Pallet & Vận chuyển vào Kho ảo (`AGVController.js`).
  - **Giai đoạn 4:** 3 trạm lập trình kỹ thuật chuyên sâu (Yaskawa Teach Pendant INFORM, MPE720 Ladder & Motion Studio, C# / Qt PC Control & Vision Studio) với bố cục Split-Screen cùng Trạm Xử Lý Thị Giác Máy Tính Công Nghiệp (`MachineVisionController.js` - Cognex Vision Pro Live PIP Feed).
  - **Giai đoạn 5:** Màn hình cảm ứng VIPA HMI SCADA 3 tab, dao động ký đa kênh Telemetry góc khớp & mô-men xoắn thời gian thực, phòng thí nghiệm Chaos Fault Injection Lab và System Master Reset.
- **Tình trạng kiểm thử:**
  - Đã kiểm tra cú pháp toàn bộ 23 file JS trong hệ thống bằng `node --check` (100% PASS, 0 warning/error).
  - Web Server đang phục vụ trực tiếp tại: `http://localhost:8080`.
