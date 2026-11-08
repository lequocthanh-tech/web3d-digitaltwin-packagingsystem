# GÓI CHẨN ĐOÁN HỆ THỐNG DIGITAL TWIN ROBOT (DIAGNOSTIC PACKAGE)
## PHẦN 1 / 3: TỔNG QUAN HỆ THỐNG, CÂY THƯ MỤC, TÀI LIỆU & MÔ HÌNH 3D

> **Dự án:** Web 3D Digital Twin — Dây chuyền Đóng gói Công nghiệp Phối hợp Robot Delta (4-DOF) & Robot Arm (6-DOF)  
> **Phiên bản:** Giai đoạn 5 (Hoàn thiện toàn diện)  
> **Mục đích tài liệu:** Cung cấp toàn bộ bức tranh kiến trúc, mô hình động học, cấu hình phần cứng và mã nguồn logic cho chuyên gia độc lập đánh giá (code review / architecture review) dưới dạng văn bản tĩnh, không yêu cầu truy cập repository hay chạy môi trường thực tế.

---

## 📑 MỤC LỤC TỔNG THỂ GÓI CHẨN ĐOÁN (MASTER TABLE OF CONTENTS)

Gói chẩn đoán được chia thành 3 phần nhằm đảm bảo dung lượng mỗi file ≤ 200KB theo tiêu chuẩn:

### 📦 PHẦN 1: `diagnostic_package_part1.md` (File này)
- **PHẦN 0 — TÓM TẮT HỆ THỐNG:**
  - Mục tiêu hệ thống
  - Kiến trúc tổng quan (Robot 6 bậc, Robot Delta, Gripper, 3 tuyến băng tải, Thùng xếp, Công nghệ nền tảng)
  - Điểm vào (Entry Points) & Hướng dẫn khởi chạy
  - Trạng thái hiện tại & Khả năng vận hành
  - 4 Vấn đề kỹ thuật trọng tâm cần chuyên gia tư vấn
  - Danh mục 23 file quan trọng trong dự án kèm mô tả 1 dòng
- **PHẦN 1 — CÂY THƯ MỤC:** Cấu trúc cây thư mục độ sâu 4 (chạy bằng lệnh `tree /F /A`)
- **PHẦN 2 — TÀI LIỆU & MANIFEST:**
  - `README.md` (Tài liệu giới thiệu & đặc tính kỹ thuật)
  - `CLAUDE.md` (Quy ước phối hợp & kiến trúc lập trình)
  - `TASK.md` (Nhật ký tiến độ & bảng điều phối 5 giai đoạn phát triển)
  - `CHAY_HE_THONG.bat` (Script khởi chạy server web cục bộ)
- **PHẦN 3 — MÔ HÌNH & MÔI TRƯỜNG MÔ PHỎNG:**
  - `js/robots/ArticulatedArm6DOF.js` (Mô hình chuỗi động học 6 bậc, khớp S, L, U, R, B, T, cổ tay cầu, DH frames, gripper)
  - `js/robots/DeltaRobot3D.js` (Mô hình cơ cấu song song Delta 4-DOF, gantry, 3 động cơ 120°, upper arms, carbon rods, mobile platform)
  - `js/scene/FactoryEnvironment.js` (Mặt bằng nhà máy, 3 băng tải, trạm camera Cognex In-Sight, cảm biến quang)
- **PHẦN 4 — CẤU HÌNH & AN TOÀN BẢO MẬT:** Báo cáo tham số cấu hình, rà soát secrets và quy tắc redaction.

### 📦 PHẦN 2: `diagnostic_package_part2.md`
- **PHẦN 5 — LOGIC ĐIỀU KHIỂN (CỐT LÕI ĐỘNG HỌC, QUỸ ĐẠO, VẬT LÝ & ĐIỀU PHỐI):**
  - `js/kinematics/Delta4DOFKinematics.js` (Động học thuận/nghịch Trilateration cho Robot song song)
  - `js/kinematics/Yaskawa6DOFKinematics.js` (Bảng DH, ma trận biến đổi, giải tách tâm cổ tay W, chuyển đổi Xung/Độ)
  - `js/kinematics/TrajectoryPlanner.js` (Bộ nội suy đa thức bậc 3 & bậc 5 Quintic, lệnh MOVJ và MOVL)
  - `js/kinematics/TrajectoryTrail.js` (Bộ hiển thị vệt quỹ đạo 3D bằng BufferGeometry Ring Buffer)
  - `js/physics/VacuumAndSensorEngine.js` (Quản lý hàng đợi camera Cognex, cảm biến DI_11..DI_13, cơ chế gắp thả scene graph)
  - `js/hardware/MP3300HardwareBus.js` (Giả lập bộ nhớ MPE720 Little-Endian, ma trận DI/DO 32-bit, máy trạng thái PLC sequencer)
  - `js/hardware/YRC1000EthernetServer.js` (Giả lập High-Speed Ethernet YERC 32-byte header, UDP 10040, Safety Interlock)
  - `js/main.js` (Script điều phối trung tâm, vòng lặp 60 FPS, state machine phối hợp 2 robot song song)

### 📦 PHẦN 3: `diagnostic_package_part3.md`
- **PHẦN 5 (Tiếp theo) — CÁC TRẠM LẬP TRÌNH THỰC HÀNH & HMI SCADA:**
  - `js/hmi-scada/VIPAHMIScreen.js` (Màn hình cảm ứng VIPA HMI 3 tab: Synoptic Overview, Recipe, Alarm Log)
  - `js/hmi-scada/TelemetryAndGraphs.js` (Dao động ký Canvas 2D vẽ đồ thị góc khớp J1..J6 và tải trọng thời gian thực)
  - `js/hmi-scada/FaultInjectionPanel.js` (Phòng thí nghiệm sự cố Chaos Lab: kẹt băng tải, E-Stop, rơi gói UDP, hỏng cảm biến)
  - `js/workstations/TeachPendantStation.js` (Tay cầm Yaskawa Teach Pendant, Jog khớp, biến P000..P009, trình thông dịch INFORM)
  - `js/workstations/MPE720StudioStation.js` (MPE720 PLC Ladder Studio 5 Rungs phát sáng luồng điện & Motion Program MPM001)
  - `js/workstations/CSharpPCStudioStation.js` (Giao diện C#/Qt PC Control Form, Code Editor & chu trình tự động Vision Loop)
- **PHẦN 6 — BẰNG CHỨNG VẬN HÀNH:**
  - Kết quả kiểm tra cú pháp toàn bộ hệ thống (`node --check` 100% PASS)
  - Nhật ký console chu trình vận hành tự động (Timeline trace)
  - Trạng thái Git repository
- **PHẦN 7 — DANH SÁCH FILE ĐÃ BỎ QUA:** Bảng liệt kê các file UI/Style đã loại trừ kèm lý do chi tiết.

---

## PHẦN 0 — TÓM TẮT HỆ THỐNG

### 1. Mục tiêu hệ thống
Dự án nhằm xây dựng một mô hình **3D Digital Twin thời gian thực (60 FPS)** hoàn chỉnh chạy trực tiếp trên nền tảng Web cho một tế bào sản xuất (workcell) đóng gói tự động tốc độ cao. Hệ thống mô phỏng sự phối hợp nhịp nhàng giữa một **Robot song song Delta 4-DOF** (nhận diện, gắp thanh sô-cô-la đơn lẻ ngẫu nhiên trên băng tải cấp liệu, nắn thẳng góc xoay yaw và xếp gom thành cụm 4 thanh) và một **Robot tay máy công nghiệp 6 bậc tự do (6-DOF Arm)** mang đầu hút chân không đa vùng (gắp nguyên cụm 4 thanh đưa vào đóng thùng carton 2 lớp theo mẫu 8 sản phẩm/thùng). Toàn bộ hệ thống tích hợp sâu mô hình bus phần cứng PLC Yaskawa MP3300, Ethernet Server YRC1000micro (UDP 10040), màn hình cảm ứng công nghiệp VIPA HMI, dao động ký đo tải khớp và 3 môi trường lập trình giả lập (Teach Pendant INFORM, MPE720 Ladder Diagram, C# / Qt Vision Studio).

### 2. Kiến trúc tổng quan
- **Robot 6 bậc tự do (`ArticulatedArm6DOF.js`):**
  - Cấu hình hình học chuẩn Yaskawa MotoMINI / GP8 / FANUC M-10 Series với 6 khớp quay: Khớp 1 ($S$) xoay đế yaw quanh trục $Y$ đứng; Khớp 2 ($L$) nâng hạ cánh tay dưới quanh trục $Z$ ngang ($a_2 = 0.74\text{m}$); Khớp 3 ($U$) nâng hạ cánh tay trên quanh trục $Z$ ($a_3 = 0.70\text{m}$); Khớp 4, 5, 6 ($R, B, T$) tạo thành **Cổ tay cầu (Spherical Wrist)** với 3 trục quay đồng quy tại tâm cổ tay ($W$).
  - Tỷ lệ xung encoder công nghiệp: $S$: 1160.53 pls/°, $L$: 1137.78 pls/°, $U$: 568.89 pls/°, $R/B/T$: 340.13 pls/°.
  - Hiển thị hệ trục tọa độ Denavit-Hartenberg $\{0\}..\{6\}$ và đường dọi TCP rơi thẳng xuống sàn nhà xưởng.
- **Robot song song Delta 4-DOF (`DeltaRobot3D.js`):**
  - Cấu trúc chuẩn FANUC M-2iA / M-3iA treo trên khung gantry thép 4 chân vững chắc.
  - Bán kính đế phân bố 3 động cơ servo cách đều $120^\circ$: $r_{base} = 0.36\text{m}$.
  - 3 Cánh tay trên (Upper Arms / Biceps): $L_1 = 0.50\text{m}$.
  - 3 Cặp thanh carbon song song (Parallelograms): $L_2 = 0.96\text{m}$, khoảng cách giữa 2 thanh $0.09\text{m}$.
  - Bệ di động (Mobile Platform): $r_{plat} = 0.085\text{m}$, mang trục truyền động thứ 4 ($\theta_4$) ở tâm xoay định hướng yaw.
- **Cơ cấu chấp hành cuối (Grippers):**
  - *Delta Gripper:* Giác hút chân không đơn (Single silicone suction cup), độ sâu hút $-0.11\text{m}$ dưới bệ, cảm biến áp suất định mức $-85\text{ kPa}$.
  - *6-DOF Gripper:* Cụm giác hút chân không đa vùng (Multi-Zone Vacuum Gripper) gồm khối nhôm anodized gắn cụm van khí nén và 8 giác hút silicone bố trí ma trận $4 \times 2$ có lò xo giảm chấn, cảm biến áp suất $-90\text{ kPa}$.
- **Các tuyến cấp liệu & khu vực đóng thùng:**
  - *Tuyến 1 (Infeed Conveyor - Băng tải cấp phôi lẻ):* Cao $0.85\text{m}$, chạy dọc $X = +4.0 \to -4.0$ tại $Z = -0.32\text{m}$. Mặt băng tải trắng chuẩn thực phẩm. Có cổng **Camera thị giác Cognex In-Sight** chiếu quạt laser phẳng xanh lá tại $X = +2.8\text{m}$ để đo tọa độ $(x, y, z)$ và góc xoay yaw của phôi bánh.
  - *Tuyến 2 (Collation Conveyor - Băng tải trung gian gom cụm):* Cao $0.85\text{m}$, đặt song song tại $Z = +0.18\text{m}$. Nhận từng thanh phôi do Delta nắn thẳng đặt vào khe gom cụm tại $X = +1.20\text{m}$. Khi đủ 4 thanh, cụm tiến tới trạm chờ có **Cảm biến quang hồng ngoại `DI_11`** tại $X = -1.25\text{m}$.
  - *Tuyến 3 (Carton Box Conveyor - Băng tải thùng carton):* Cao $0.55\text{m}$, đặt phía sau robot 6 bậc tại $Z = -1.45\text{m}$. Chở thùng carton hở nắp với cơ cấu xilanh chặn khí nén và cảm biến đỗ thùng `DI_12`, cảm biến thùng đầy `DI_13`.
  - *Cơ chế đóng thùng (Bin Packing):* Xếp cụm 4 thanh theo mẫu 2 tầng (Tầng 1 ở cao độ $Y = 0.79\text{m}$, Tầng 2 ở cao độ $Y = 0.83\text{m}$). Mỗi thùng chứa 8 sản phẩm.
- **Công nghệ nền tảng:**
  - **100% Web Native:** HTML5, CSS3 (Theme Dark SCADA), JavaScript thuần (ES6 Modules).
  - **Engine đồ họa 3D:** Three.js (r128 nạp qua CDN unpkg/cdnjs) kết hợp OrbitControls. Toàn bộ hình học là procedural primitives được tối ưu geometry/material, khử bóng râm PCFSoftShadowMap, hỗ trợ Anti-Lag 3 mức chất lượng (High / Balanced / Fast).
  - **Không dùng ROS 2, không Gazebo, không Isaac Sim, không Blender binary:** Chạy trực tiếp trên trình duyệt Chrome, Edge, Firefox mà không cần cài đặt môi trường C++/Python phức tạp.
  - **Giả lập phần cứng & mạng công nghiệp:** Bus bộ nhớ PLC MP3300 mô phỏng Little-Endian qua `ArrayBuffer` & `DataView`; giao thức Ethernet YRC1000 Server mô phỏng đóng gói khung nhị phân 32-byte YERC header qua UDP 10040 in-memory.

### 3. Điểm vào (Entry Points) & Cách chạy
- **Giao diện người dùng chính:** `index.html`
- **File kịch bản khởi chạy nhanh trên Windows:** `CHAY_HE_THONG.bat`
  ```bat
  python -m http.server 8080
  start "" "http://localhost:8080"
  ```
- **Lệnh terminal:**
  - Python: `python -m http.server 8080` rồi mở `http://localhost:8080`
  - Node.js: `npx serve .` rồi mở theo đường dẫn port hiển thị
- **Entry script JavaScript:** `js/main.js` (Lớp `DigitalTwinApp` khởi tạo Three.js scene, camera presets, hardware bus, workstations và vòng lặp `animate()` 60 FPS).

### 4. Trạng thái hiện tại & Khả năng vận hành
- **Tiến độ:** Đã hoàn thành toàn bộ 5 giai đoạn phát triển theo tài liệu `TASK.md`.
- **Cú pháp:** 100% 17 file JavaScript vượt qua kiểm tra cú pháp `node --check` (0 lỗi, 0 cảnh báo).
- **Các tính năng đã vận hành mượt mà:**
  1. Chu trình tự động song song (Parallel Multi-Robot Cycle): Delta gắp thanh lẻ -> Gom cụm 4 thanh -> Arm 6-DOF gắp cụm đóng 2 lớp vào thùng carton -> Thùng đầy xuất xưởng.
  2. Động học thuận/nghịch (FK/IK) tính toán tức thời (< 0.1ms/frame).
  3. Bộ nội suy quỹ đạo `MOVJ` (Joint space) và `MOVL` (Cartesian linear với IK từng frame) theo đa thức bậc 5 Quintic mượt mà không rung giật.
  4. Hiển thị vệt đường đi 3D (Trajectory Trail) của cả 2 đầu công tác.
  5. 4 Trạm thực hành: Yaskawa Teach Pendant (INFORM parser), MPE720 Ladder Studio (vẽ luồng điện 5 rungs phát sáng), C# Vision Studio và VIPA HMI Touch Panel kèm Oscilloscope vẽ đồ thị tải trọng 6 khớp.
- **Tình trạng Stub / TODO:** Không còn stub hay hàm rỗng. Mọi tính năng đều có mã thực thi hoàn chỉnh trong client sandbox.

### 5. Bốn vấn đề kỹ thuật trọng tâm cần chuyên gia tư vấn
1. **Kiến trúc Cầu nối Mạng thực (Real Gateway vs In-Memory Simulation):** Hiện tại mô đun Ethernet Server YRC1000 (UDP port 10040) và Bus PLC MP3300 đang chạy in-memory trong JavaScript DataView. Cần chuyên gia tư vấn phương án tích hợp một Node.js/Python WebSocket-to-UDP Gateway để kết nối trực tiếp hai chiều với bộ điều khiển Yaskawa YRC1000 thật hoặc phần mềm lập trình MPE720/MotoSim chạy ngoài máy trạm.
2. **Cơ chế Vật lý Tiếp xúc & Bắt phôi (Physics Engine vs Scene-Graph Reparenting):** Hiện tại việc gắp phôi sử dụng phương pháp chuyển đổi quan hệ cha-con trong Scene Graph (`parent.add(mesh)`) và kiểm tra va chạm khoảng cách (proximity distance threshold). Cần tư vấn xem có nên tích hợp một engine vật lý thực (như Ammo.js / Rapier.js Wasm) để mô phỏng lực hút chân không động lực học, hay giữ kiến trúc hình học nhẹ nhàng hiện tại để duy trì 60 FPS ổn định trên thiết bị cấu hình thấp.
3. **Chiến lược Tránh Điểm Kỳ Dị (Singularity Avoidance) cho Cổ tay cầu Spherical Wrist:** Hàm IK của cánh tay 6 bậc hiện đang giải giải tích (analytical decoupling tách tâm cổ tay $W$). Khi khớp 5 gần $0^\circ$ (gimbal lock cổ tay) hoặc khi robot duỗi thẳng hết tầm, giải thuật sử dụng phương án cố định góc $\theta_4$. Cần ý kiến chuyên gia về việc bổ sung thuật toán Damped Least Squares (DLS) hoặc nghịch đảo ma trận Jacobian khi thực hiện nội suy đường thẳng tuyệt đối `MOVL`.
4. **Thuật toán Cân bằng Tải & Điều phối Hàng đợi (Buffer Accumulation & Throughput Scheduler):** Khi tốc độ cấp phôi của Tuyến 1 thay đổi, Robot Delta có thể sản sinh cụm 4 thanh nhanh hơn tốc độ đóng thùng của Robot Arm 6 bậc (hoặc ngược lại). Cần chuyên gia đánh giá máy trạng thái sequencer hiện tại trong `MP3300HardwareBus.js` và gợi ý giải thuật điều phối nhịp độ tối ưu nhằm triệt tiêu hiện tượng dồn ứ trên Băng tải 2.

### 6. Danh mục file quan trọng trong dự án
1. `CHAY_HE_THONG.bat`: Script khởi chạy server HTTP cổng 8080 và tự động mở trình duyệt.
2. `index.html`: Khung giao diện chính chứa viewport 3D Three.js, thanh camera, SCADA dock và 4 trạm lập trình.
3. `css/style.css`: Bộ quy tắc định dạng giao diện SCADA công nghiệp tông tối (Industrial Dark Theme).
4. `js/main.js`: Bộ điều phối lõi khởi tạo cảnh 3D, camera presets, vòng lặp 60 FPS và state machine chu trình đóng gói.
5. `js/scene/FactoryEnvironment.js`: Dựng mặt bằng nhà máy, sàn epoxy, hàng rào an toàn, 3 băng tải, camera Cognex và cảm biến.
6. `js/robots/DeltaRobot3D.js`: Dựng mô hình 3D Robot song song Delta 4-DOF gồm gantry, 3 động cơ $120^\circ$, 3 cặp thanh carbon và bệ hút.
7. `js/robots/ArticulatedArm6DOF.js`: Dựng mô hình 3D Robot 6 bậc chuẩn Yaskawa MotoMINI gồm 6 khâu S, L, U, R, B, T, cổ tay cầu và đầu hút.
8. `js/kinematics/Delta4DOFKinematics.js`: Module động học thuận/nghịch giải bằng phương pháp giao điểm 3 mặt cầu (Trilateration).
9. `js/kinematics/Yaskawa6DOFKinematics.js`: Module động học thuận/nghịch theo bảng DH, giải tách tâm cổ tay W và tỷ lệ xung Yaskawa.
10. `js/kinematics/TrajectoryPlanner.js`: Bộ lập quỹ đạo đa thức bậc 3 & bậc 5 Quintic, thực thi lệnh MOVJ và MOVL.
11. `js/kinematics/TrajectoryTrail.js`: Bộ vẽ vệt quỹ đạo 3D thời gian thực của đầu công tác bằng Ring Buffer.
12. `js/physics/VacuumAndSensorEngine.js`: Quản lý hàng đợi camera Cognex, cảm biến quang DI_11..DI_13, cơ chế hút/thả và xếp tầng thùng carton.
13. `js/hardware/MP3300HardwareBus.js`: Giả lập bộ nhớ thanh ghi MPE720 Little-Endian, ma trận I/O 32-bit và máy trạng thái PLC ladder sequencer.
14. `js/hardware/YRC1000EthernetServer.js`: Giả lập High-Speed Ethernet Server chuẩn gói YERC 32-byte header qua UDP 10040.
15. `js/workstations/TeachPendantStation.js`: Trạm lập trình Yaskawa Teach Pendant giả lập tay cầm, Deadman Switch, Jog và lệnh INFORM.
16. `js/workstations/MPE720StudioStation.js`: Trạm MPE720 hiển thị sơ đồ mạch Ladder 5 rungs phát sáng dòng điện logic và Motion Program MPM001.
17. `js/workstations/CSharpPCStudioStation.js`: Trạm điều khiển C#/Qt PC Control Form và Code Editor hỗ trợ chạy chu trình thị giác Cognex.
18. `js/hmi-scada/VIPAHMIScreen.js`: Màn hình cảm ứng công nghiệp VIPA HMI với 3 tab: Synoptic, Recipe và Alarm Diagnostics.
19. `js/hmi-scada/TelemetryAndGraphs.js`: Dao động ký đa kênh Canvas 2D theo dõi góc khớp J1..J6 và tỷ lệ tải trọng mô-men xoắn thời gian thực.
20. `js/hmi-scada/FaultInjectionPanel.js`: Bảng điều khiển giả lập sự cố công nghiệp (kẹt băng tải, mở cửa an toàn, mất gói UDP, hỏng cảm biến).
21. `README.md`: Hướng dẫn tổng quan dự án, thông số hình học và hướng dẫn chạy.
22. `TASK.md`: Bảng quản lý tiến độ chi tiết 5 giai đoạn phát triển dự án.
23. `CLAUDE.md`: Hướng dẫn quy ước kiến trúc và nguyên tắc phối hợp lập trình.

---

## PHẦN 1 — CÂY THƯ MỤC

Lệnh đã dùng trên hệ điều hành Windows:
```cmd
cmd /c "tree /F /A"
```
Kết quả hiển thị cây thư mục toàn bộ dự án (độ sâu 4, dự án không chứa thư mục ngoại vi `node_modules`, `.git`, `__pycache__` hay binary lớn):

```
D:\simulationsystem
|   CHAY_HE_THONG.bat
|   CLAUDE.md
|   index.html
|   README.md
|   TASK.md
|   
+---css
|       style.css
|       
\---js
    |   main.js
    |   
    +---hardware
    |       MP3300HardwareBus.js
    |       YRC1000EthernetServer.js
    |       
    +---hmi-scada
    |       FaultInjectionPanel.js
    |       TelemetryAndGraphs.js
    |       VIPAHMIScreen.js
    |       
    +---kinematics
    |       Delta4DOFKinematics.js
    |       TrajectoryPlanner.js
    |       TrajectoryTrail.js
    |       Yaskawa6DOFKinematics.js
    |       
    +---physics
    |       VacuumAndSensorEngine.js
    |       
    +---robots
    |       ArticulatedArm6DOF.js
    |       DeltaRobot3D.js
    |       
    +---scene
    |       FactoryEnvironment.js
    |       
    \---workstations
            CSharpPCStudioStation.js
            MPE720StudioStation.js
            TeachPendantStation.js
```

---

## PHẦN 2 — TÀI LIỆU & MANIFEST

> **Ghi chú về Manifest:** Dự án là ứng dụng thuần client Web (Three.js ES Modules qua CDN) chạy trực tiếp thông qua máy chủ HTTP tĩnh. Dự án không yêu cầu build bundler (không có `package.json`, `pyproject.toml`, `package.xml`, hay `CMakeLists.txt`).

Dưới đây là nội dung nguyên văn các file tài liệu và kịch bản khởi chạy:

=== FILE: CHAY_HE_THONG.bat ===
```bat
@echo off
title Web 3D Digital Twin - Packaging Line Simulation
cd /d "%~dp0"
echo ======================================================================
echo   WEB 3D DIGITAL TWIN - HE THONG DONG GOI ROBOT DELTA ^& 6-DOF ARM
echo ======================================================================
echo Dang mo trinh duyet va khoi chay Server tai cong 8080...
echo.
timeout /t 1 >nul
start "" "http://localhost:8080"
python -m http.server 8080
pause
```

=== FILE: CLAUDE.md ===
```markdown
# CLAUDE.md - Hướng Dẫn Phát Triển & Phối Hợp Dự Án

## 1. Tổng Quan Dự Án
- **Tên dự án:** Web 3D Digital Twin - Dây chuyền Đóng gói Robot Delta (4-DOF) & Robot Arm (6-DOF).
- **Công nghệ cốt lõi:** Pure HTML5, CSS3 (Industrial Dark SCADA), Three.js (ES Modules qua CDN + OrbitControls).
- **Mục tiêu:** Chạy trực tiếp trên trình duyệt, không dùng bundler phức tạp (không Vite/Webpack trừ khi có yêu cầu riêng), đảm bảo 60 FPS, chuẩn động học công nghiệp.

## 2. Cấu Trúc Thư Mục
```
d:/simulationsystem/
├── index.html                     # Viewport 3D + SCADA Inspector Dock
├── css/style.css                  # Industrial Dark SCADA Theme
├── js/
│   ├── main.js                    # Khởi tạo Three.js Scene, Renderer, Loop 60fps
│   ├── scene/FactoryEnvironment.js # Sàn epoxy, hàng rào, 3 băng tải, Cognex Camera
│   └── robots/
│       ├── DeltaRobot3D.js        # Khung Gantry, 3 động cơ 120°, Kinematics Delta
│       └── ArticulatedArm6DOF.js  # Chuỗi động học 6 bậc (S,L,U,R,B,T), Cổ tay cầu
├── CLAUDE.md                      # Hướng dẫn dành cho Claude Code (file này)
└── TASK.md                        # Bảng điều phối công việc giữa Claude Code & Antigravity
```

## 3. Quy Ước Phối Hợp Cùng Antigravity (Pair Programming)
Trong thư mục này, **Claude Code** và **Antigravity** là 2 trợ lý làm việc song song:
1. **Kiểm tra `TASK.md` trước khi code:** Xem các tác vụ đang được giao hoặc ghi chú kiến trúc từ Antigravity.
2. **Cập nhật `TASK.md` sau khi hoàn thành:** Đánh dấu task thành `[x]` hoặc `DONE`, ghi chú ngắn các file vừa thay đổi để Antigravity review và test.
3. **Quy chuẩn code:**
   - Giữ nguyên kiến trúc module ES6 (`import * as THREE from '...'`).
   - Đảm bảo hiệu năng Three.js: tái sử dụng `Geometry`/`Material`, dọn dẹp bộ nhớ khi hủy object.
   - Giữ nguyên tông màu SCADA công nghiệp (`#0f172a`, `#1e293b`, LED `#22c55e`, cảnh báo `#eab308`/`#ef4444`).
```

=== FILE: README.md ===
```markdown
# Web 3D Digital Twin - Dây chuyền Đóng gói Phối hợp Robot Delta & Robot Arm 6-DOF (GIAI ĐOẠN 1)

Mô phỏng 3D Digital Twin thời gian thực (60 FPS) nguyên mẫu dây chuyền đóng gói công nghiệp phối hợp giữa **Robot Delta 4-DOF** và **Robot Arm 6-DOF (Cổ tay cầu Spherical Wrist)** theo tiêu chuẩn FANUC / Yaskawa.

Dự án được xây dựng thuần trên **HTML5, CSS3 (Giao diện Industrial Dark SCADA)** và **Three.js (ES Module qua CDN + OrbitControls)**, chạy trực tiếp trên trình duyệt mà không cần cài đặt hoặc build phức tạp.

---

## 📁 Cấu trúc Thư mục Dự án

```
d:/simulationsystem/
│
├── index.html                     # Giao diện chính: Viewport 3D (75%) + SCADA Inspector Dock (25%) + Camera Toolbar
├── css/
│   └── style.css                  # Industrial Dark SCADA Theme, LED Indicators, Monospace Telemetry
└── js/
    ├── main.js                    # Khởi tạo Three.js Scene, Renderer (ShadowMap), OrbitControls, Loop 60fps
    ├── scene/
    │   └── FactoryEnvironment.js  # Sàn epoxy, hàng rào an toàn, 3 băng tải, Cognex Camera & Photoelectric DI_11
    └── robots/
        ├── DeltaRobot3D.js        # Khung Gantry, 3 động cơ 120°, Parallelogram Carbon Rods, 4th Axis & Kinematics
        └── ArticulatedArm6DOF.js  # Chuỗi động học 6 bậc (S,L,U,R,B,T), Cổ tay cầu, DH Frames {0}..{6}, Drop Line
```

---

## ⚙️ Các Thành phần Kỹ thuật Chi tiết

### 1. Bố cục Mặt bằng Nhà máy & Băng tải
- **Vách ngăn an toàn (Safety Fence):** Sơn vàng công nghiệp (`#facc15` / `#eab308`), lưới mắt cáo đen, bảo vệ toàn bộ ô làm việc robot.
- **Băng tải 1 (Infeed Conveyor - Làn trong):**
  - Chạy từ phải sang trái ($X = +4.0 \to -4.0$ tại $Z = -0.4$, cao $0.85\text{m}$).
  - Mặt băng tải trắng kem chuẩn thực phẩm, khung inox, chân tăng chỉnh.
  - **Cổng Camera Vision Cognex In-Sight:** Chiếu quạt laser xanh lá (`0x00ff88`) quét phẳng ngang băng tải để định vị sản phẩm.
  - **Hàm sinh sản phẩm:** Thanh chocolate/snack lẻ ($0.18 \times 0.04 \times 0.08\text{m}$) vỏ đỏ mạ vàng, lệch vị trí và góc xoay ngẫu nhiên.
- **Băng tải 2 (Collation Conveyor - Làn ngoài song song):**
  - Đặt song song tại $Z = +0.15\text{m}$, cao $0.85\text{m}$.
  - Nhận cụm 4 thanh sản phẩm xếp thẳng hàng khít nhau từ Robot Delta.
  - **Cảm biến quang `DI_11`:** Đặt tại $X = -1.2\text{m}$ (ngay trước mặt Robot Arm 6 bậc), phát tia laser đỏ kiểm soát cụm hàng đến vị trí chờ gắp.
- **Băng tải 3 (Box Conveyor - Băng tải Thùng Carton):**
  - Đặt tại $X = -1.8\text{m}$, chạy dọc theo trục $Z$, cao $0.55\text{m}$.
  - Chở **Thùng carton hở nắp** (nhìn rõ khoang bên trong để đóng nhiều lớp hàng).
  - Có xilanh chặn thùng khí nén và cảm biến vị trí thùng `DI_12`.

---

### 2. Robot Delta 4-DOF (Treo Gantry bên phải: $X = +1.2, Z = -0.1$)
- Treo trên khung giàn thép hộp đen 4 chân giằng kiên cố.
- 3 động cơ Servo đặt đối xứng $120^\circ$ trên đế vàng công nghiệp.
- 3 cánh tay trên (`Upper Arms` $L_1 = 0.38\text{m}$) nối với 3 cặp thanh carbon song song (`Parallelograms` $L_2 = 0.72\text{m}$) tụ về đế di động (`Mobile Platform`).
- Trục truyền động thứ 4 ở tâm (`Theta_4`) xoay chỉnh góc sản phẩm gắn giác hút chân không đơn.
- **Thuật toán Động học Toàn vẹn (Forward & Inverse Kinematics):**
  - Cung cấp `setJointAngles(m1, m2, m3, yaw)` và `setEndEffectorPose(x, y, z, yaw)`.
  - Giải giao điểm 3 mặt cầu (Trilateration) giúp các thanh carbon luôn nối khít vào khớp mà không bị hở khi kéo trượt góc.

---

### 3. Robot Arm 6 Bậc Tự Do (Bên trái: $X = -1.2, Z = -0.8$)
- Thiết kế chuẩn tay máy công nghiệp 6 bậc (Yaskawa MotoMINI / GP8 / FANUC LR Mate):
  - Khớp 1 ($S$): Base Yaw quanh $z_0$ đứng.
  - Khớp 2 ($L$): Lower Arm Pitch quanh $z_1$ ngang.
  - Khớp 3 ($U$): Upper Arm Pitch quanh $z_2$ ngang.
  - Khớp 4, 5, 6 ($R, B, T$): **Cổ tay cầu (Spherical Wrist)** với 3 trục giao nhau tại Wrist Center.
  - Khớp 6 ($T$): Gắn **Đầu hút chân không bản rộng (Multi-Zone Vacuum Gripper)** với 4 giác hút silicone và van đo áp suất điện tử, kích thước vừa khít cụm 4 thanh sản phẩm.
- **Tính năng Học thuật:**
  - Hiển thị hệ trục tọa độ Denavit-Hartenberg (`AxesHelper` với nhãn billboard `{0}`..`{6}`).
  - Đường dọi tọa độ nét đứt rơi xuống sàn kèm vòng ngắm mục tiêu và hiển thị tọa độ mặt bằng.
  - Chuyển đổi song song giữa **Độ (Deg)** và **Xung encoder Yaskawa (Pulse)**.

---

### 4. Bảng Điều khiển SCADA & Hardware Verification
- **Nguồn tổng `[POWER ON / SERVO ON]`:** Khi mới mở trang, hệ thống ở trạng thái `OFFLINE`, mọi robot và băng tải dừng hoàn toàn.
- **Công tắc kiểm tra băng tải độc lập:**
  - `[Conveyor 1 Run]` (Infeed)
  - `[Conveyor 2 Run]` (Collation)
  - `[Conveyor 3 Run]` (Carton Box)
- **Thao tác thủ công:** `[+ Spawn Product]`, `[+ Spawn 4-Pack]`, `[+ Spawn Empty Box]`, `[Clear Objects]`.
- **Manual Jogger:** 6 thanh trượt cho Arm 6-DOF, 4 thanh trượt cho Delta Robot.
- **Các góc Camera chuyển nhanh (Smooth Lerp):**
  1. `[Overview 3D]`: Toàn cảnh phối cảnh dây chuyền.
  2. `[Front View (TikTok Angle)]`: Góc ngang trực diện cell robot sinh động.
  3. `[Delta Station]`: Cận cảnh cụm gantry Delta và quét laser Cognex.
  4. `[6-DOF Arm & Box]`: Cận cảnh tay máy 6 bậc gắp cụm và thùng carton.
  5. `[Top-Down Layout]`: Nhìn thẳng đứng từ trên xuống kiểm tra mặt bằng chuẩn.

---

## 🚀 Hướng Dẫn Chạy

Do các file JavaScript sử dụng **ES Modules** (`import` / `export`), vui lòng mở dự án qua một máy chủ web nội bộ:

### Cách 1: Sử dụng VS Code Live Server
1. Mở thư mục `d:\simulationsystem` bằng VS Code.
2. Click chuột phải vào `index.html` chọn **Open with Live Server**.

### Cách 2: Sử dụng Python (có sẵn)
Mở terminal tại thư mục dự án và chạy:
```bash
python -m http.server 8080
```
Truy cập trình duyệt tại: [http://localhost:8080](http://localhost:8080)

### Cách 3: Sử dụng Node.js (npx serve)
```bash
npx serve .
```
```

=== FILE: TASK.md ===
```markdown
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

---

## 📝 Nhật Ký Hoàn Thành & Bàn Giao Toàn Diện Hệ Thống (Master Handoff Log)
- **Hoàn thành trọn vẹn 5/5 Giai đoạn phát triển của Đồ án Web 3D Digital Twin:**
  - **Giai đoạn 1:** Xây dựng mô hình 3D Three.js tỉ lệ thật, băng tải, phôi kẹo socola, thùng carton, camera điều hướng trực quan.
  - **Giai đoạn 2:** Engine toán học Động học thuận/nghịch (FK/IK) 6-DOF & Delta 4-DOF, bộ nội suy quỹ đạo Joint/Linear, vật lý gắp thả chân không và cảm biến quang 3D.
  - **Giai đoạn 3:** Bus phần cứng ảo MP3300 (ArrayBuffer/DataView Little-Endian, 32-bit DI/DO), High-Speed Ethernet UDP YERC Server (cổng 10040), Control Panel công nghiệp và UDP Sniffer.
  - **Giai đoạn 4:** 3 trạm lập trình kỹ thuật chuyên sâu (Yaskawa Teach Pendant INFORM, MPE720 Ladder & Motion Studio, C# / Qt PC Control & Vision Studio) với bố cục Split-Screen.
  - **Giai đoạn 5:** Màn hình cảm ứng VIPA HMI SCADA 3 tab, dao động ký đa kênh Telemetry góc khớp & mô-men xoắn thời gian thực, phòng thí nghiệm Chaos Fault Injection Lab và System Master Reset.
- **Tình trạng kiểm thử:**
  - Đã kiểm tra cú pháp toàn bộ 17 file JS trong hệ thống bằng `node --check` (100% PASS, 0 warning/error).
  - Web Server đang phục vụ trực tiếp tại: `http://localhost:8080`.
```

---

## PHẦN 3 — MÔ HÌNH & MÔI TRƯỜNG MÔ PHỎNG

> **Ghi chú về Mô hình 3D:** Trong dự án Digital Twin này, toàn bộ mô hình robot và dây chuyền được xây dựng dưới dạng Three.js JavaScript Procedural Geometry thay vì URDF/SDF hay file Blender binary. Cách tiếp cận này giúp ứng dụng tải nhanh lập tức qua mạng web tĩnh mà không gặp vấn đề nạp file mesh STL/DAE/GLTF nặng nề.  
> Để tối ưu dung lượng gói chẩn đoán mà vẫn giữ trọn vẹn giá trị kỹ thuật cho chuyên gia, phần mã nguồn dưới đây **giữ nguyên toàn bộ chuỗi động học (Kinematic Tree, Links, Joints, Khung tọa độ DH, Ma trận xoay và Gripper)**; các đoạn tạo chi tiết trang trí thẩm mỹ lặp lại (bu lông, gân tăng cứng thứ cấp) đã được rút gọn có chú thích rõ ràng.

=== FILE: js/robots/ArticulatedArm6DOF.js ===
```javascript
import * as THREE from 'three';

/**
 * ArticulatedArm6DOF - Heavy/Medium Industrial 6-DOF Robotic Arm (Yaskawa MotoMINI / FANUC M-10 Series)
 * Chuỗi động học chuẩn công nghiệp gồm 6 khâu S, L, U, R, B, T với Cổ tay cầu Spherical Wrist.
 */
export class ArticulatedArm6DOF {
  constructor(options = {}) {
    // Vị trí đặt đế trong tọa độ thế giới (mặt bằng nhà máy)
    this.basePosition = options.position || new THREE.Vector3(-1.25, 0, -0.75);
    
    // Góc khớp ban đầu (Độ): Home Pose vươn về phía trước (+Z) hướng vào băng tải đóng gói
    this.angles = {
      s: -75.0,  // J1 (Base Swivel / Turret quanh Y)
      l: -18.0,  // J2 (Lower Arm Pitch quanh Z)
      u: -10.0,  // J3 (Upper Arm Pitch quanh Z)
      r: 0.0,    // J4 (Forearm Roll quanh X)
      b: -62.0,  // J5 (Wrist Bend quanh Z: l + u + b = -90° giữ giác hút luôn phẳng ngang)
      t: 0.0     // J6 (Tool Flange Twist quanh X)
    };

    // Giới hạn an toàn vật lý của các khớp
    this.jointLimits = {
      s: { min: -170, max: 170 },
      l: { min: -85, max: 90 },
      u: { min: -110, max: 110 },
      r: { min: -140, max: 140 },
      b: { min: -135, max: 135 },
      t: { min: -360, max: 360 }
    };

    // Tỷ lệ chuyển đổi Xung Encoder / Độ (Yaskawa MotoMINI chuẩn)
    this.pulseRatios = {
      s: 34816 / 30,    // 1160.533 pls/deg
      l: 102400 / 90,   // 1137.778 pls/deg
      u: 51200 / 90,    // 568.889 pls/deg
      r: 10204 / 30,    // 340.133 pls/deg
      b: 10204 / 30,    // 340.133 pls/deg
      t: 10204 / 30     // 340.133 pls/deg
    };

    this.dhVisible = false;
    this.root = new THREE.Group();
    this.root.name = "ArticulatedArm6DOF_Yaskawa";
    this.root.position.copy(this.basePosition);

    this.dhHelpers = [];
    this.initMaterials();
    this.buildRobotAssembly();
    this.buildDHFrames();
    this.buildFloorDropLine();
    this.setDHVisible(false);

    this.updateJoints();
  }

  initMaterials() {
    this.matFanucYellow = new THREE.MeshStandardMaterial({ color: 0xfed100, roughness: 0.26, metalness: 0.16 });
    this.matCastDark = new THREE.MeshStandardMaterial({ color: 0x1f2937, roughness: 0.5, metalness: 0.65 });
    this.matJointSteel = new THREE.MeshStandardMaterial({ color: 0x94a3b8, roughness: 0.25, metalness: 0.85 });
    this.matChrome = new THREE.MeshStandardMaterial({ color: 0xf1f5f9, roughness: 0.15, metalness: 0.95 });
    this.matPedestal = new THREE.MeshStandardMaterial({ color: 0x1e293b, roughness: 0.55, metalness: 0.5 });
    this.matGripperWhite = new THREE.MeshStandardMaterial({ color: 0xf8fafc, roughness: 0.25, metalness: 0.4 });
    this.matSuctionCups = new THREE.MeshStandardMaterial({ color: 0x0284c7, roughness: 0.4, metalness: 0.1 });
  }

  buildRobotAssembly() {
    // 1. Chân đế Riser Pedestal nâng chiều cao robot ngang tầm băng tải (0.85m)
    const pedHeight = 0.52;
    this.pedestalGroup = new THREE.Group();
    this.pedestalGroup.name = "RobotPedestal";
    const column = new THREE.Mesh(new THREE.CylinderGeometry(0.18, 0.20, pedHeight - 0.04, 32), this.matPedestal);
    column.position.y = (pedHeight - 0.04) / 2 + 0.02;
    this.pedestalGroup.add(column);
    this.root.add(this.pedestalGroup);

    // 2. KHÂU 0 (FRAME {0}): Base đúc cố định
    this.frame0 = new THREE.Group();
    this.frame0.position.y = pedHeight + 0.02;
    this.frame0.name = "Frame_0_Base";
    const baseCast = new THREE.Mesh(new THREE.CylinderGeometry(0.22, 0.24, 0.12, 32), this.matFanucYellow);
    baseCast.position.y = 0.06;
    this.frame0.add(baseCast);

    // 3. KHÂU 1 (FRAME {1}): JOINT 1 (S - Base Turret Swivel quanh Y)
    this.j1Group = new THREE.Group();
    this.j1Group.position.y = 0.12;
    this.j1Group.name = "Joint1_S_Turret";
    const shoulderCenter = new THREE.Mesh(new THREE.BoxGeometry(0.28, 0.30, 0.26), this.matFanucYellow);
    shoulderCenter.position.set(0, 0.18, 0);
    this.j1Group.add(shoulderCenter);
    this.frame0.add(this.j1Group);

    // 4. KHÂU 2 (FRAME {2}): JOINT 2 (L - Lower Arm Pitch quanh Z, chiều dài a2 = 0.74m)
    this.link2Length = 0.74;
    this.j2Group = new THREE.Group();
    this.j2Group.position.set(0, 0.30, 0); // Trục tâm vai
    this.j2Group.name = "Joint2_L_LowerArm";
    const bicepMesh = new THREE.Mesh(new THREE.BoxGeometry(0.18, this.link2Length, 0.18), this.matFanucYellow);
    bicepMesh.position.y = this.link2Length / 2;
    this.j2Group.add(bicepMesh);
    this.j1Group.add(this.j2Group);

    // 5. KHÂU 3 (FRAME {3}): JOINT 3 (U - Upper Arm Pitch quanh Z, chiều dài a3 = 0.70m)
    this.link3Length = 0.70;
    this.j3Group = new THREE.Group();
    this.j3Group.position.set(0, this.link2Length, 0); // Trục tâm khuỷu
    this.j3Group.name = "Joint3_U_UpperArm";
    const boomMesh = new THREE.Mesh(new THREE.BoxGeometry(this.link3Length, 0.16, 0.16), this.matFanucYellow);
    boomMesh.position.x = this.link3Length / 2;
    this.j3Group.add(boomMesh);
    this.j2Group.add(this.j3Group);

    // 6. KHÂU 4 (FRAME {4}): JOINT 4 (R - Forearm Roll quanh X)
    this.j4Group = new THREE.Group();
    this.j4Group.position.set(this.link3Length, 0, 0);
    this.j4Group.name = "Joint4_R_ForearmRoll";
    const j4Barrel = new THREE.Mesh(new THREE.CylinderGeometry(0.09, 0.09, 0.14, 24), this.matCastDark);
    j4Barrel.rotation.z = Math.PI / 2;
    j4Barrel.position.x = 0.07;
    this.j4Group.add(j4Barrel);
    this.j3Group.add(this.j4Group);

    // 7. KHÂU 5 (FRAME {5}): JOINT 5 (B - Cổ tay cầu Spherical Wrist Bend quanh Z)
    // Tâm cổ tay W: Điểm giao nhau của 3 trục 4, 5, 6
    this.j5Group = new THREE.Group();
    this.j5Group.position.set(0.14, 0, 0); // Wrist center W
    this.j5Group.name = "Joint5_B_SphericalWrist";
    const wristSph = new THREE.Mesh(new THREE.SphereGeometry(0.095, 24, 24), this.matJointSteel);
    this.j5Group.add(wristSph);
    this.j4Group.add(this.j5Group);

    // 8. KHÂU 6 (FRAME {6}): JOINT 6 (T - Flange Twist quanh X) & Gripper Chân không
    this.j6Group = new THREE.Group();
    this.j6Group.position.set(0.11, 0, 0); // Mặt bích dụng cụ
    this.j6Group.name = "Joint6_T_ToolFlange";
    const flange = new THREE.Mesh(new THREE.CylinderGeometry(0.08, 0.08, 0.03, 32), this.matChrome);
    flange.rotation.z = Math.PI / 2;
    this.j6Group.add(flange);

    // Xây dựng cụm Giác hút chân không đa vùng (Multi-zone vacuum gripper)
    this.buildVacuumGripper();
    this.j5Group.add(this.j6Group);
    this.root.add(this.frame0);
  }

  buildVacuumGripper() {
    this.gripperGroup = new THREE.Group();
    this.gripperGroup.name = "PackagingVacuumGripper_Large";
    this.gripperGroup.position.set(0.02, 0, 0);

    // Hộp gom áp suất nhôm kích thước 0.10 x 0.24 x 0.20m vừa khít cụm 4 thanh
    const boxMesh = new THREE.Mesh(new THREE.BoxGeometry(0.10, 0.24, 0.20), this.matGripperWhite);
    boxMesh.position.x = 0.05;
    this.gripperGroup.add(boxMesh);

    // 8 Giác hút Silicone bố trí theo ma trận 4x2
    const cupRows = [-0.075, -0.025, 0.025, 0.075];
    const cupCols = [-0.05, 0.05];
    cupRows.forEach(ry => {
      cupCols.forEach(rz => {
        const cup = new THREE.Mesh(new THREE.CylinderGeometry(0.024, 0.016, 0.028, 16), this.matSuctionCups);
        cup.rotation.z = Math.PI / 2;
        cup.position.set(0.125, ry, rz);
        this.gripperGroup.add(cup);
      });
    });

    // Điểm Tool Center Point (TCP) Marker đặt tại đầu giác hút (x = 0.14m từ flange)
    this.tcpMarker = new THREE.Group();
    this.tcpMarker.position.set(0.14, 0, 0);
    this.tcpMarker.name = "TCP_Marker";
    this.gripperGroup.add(this.tcpMarker);

    this.j6Group.add(this.gripperGroup);
  }

  updateJoints() {
    // Áp đặt góc quay vào từng Group Three.js theo đúng bậc tự do
    this.j1Group.rotation.y = THREE.MathUtils.degToRad(-this.angles.s);
    this.j2Group.rotation.z = THREE.MathUtils.degToRad(this.angles.l);
    this.j3Group.rotation.z = THREE.MathUtils.degToRad(this.angles.u);
    this.j4Group.rotation.x = THREE.MathUtils.degToRad(this.angles.r);
    this.j5Group.rotation.z = THREE.MathUtils.degToRad(this.angles.b);
    this.j6Group.rotation.x = THREE.MathUtils.degToRad(this.angles.t);

    if (this.dropLineGroup) {
      this.updateFloorDropLine();
    }
  }

  setJointAngles(s, l, u, r, b, t) {
    if (s !== undefined) this.angles.s = s;
    if (l !== undefined) this.angles.l = l;
    if (u !== undefined) this.angles.u = u;
    if (r !== undefined) this.angles.r = r;
    if (b !== undefined) this.angles.b = b;
    if (t !== undefined) this.angles.t = t;
    this.updateJoints();
  }

  getJointAngles() { return { ...this.angles }; }

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
    const v = new THREE.Vector3();
    if (this.tcpMarker) {
      this.tcpMarker.getWorldPosition(v);
    }
    return v;
  }
}
```

=== FILE: js/robots/DeltaRobot3D.js ===
```javascript
import * as THREE from 'three';

/**
 * DeltaRobot3D - FANUC M-2iA / M-3iA Style Industrial Delta Robot (4-DOF)
 * Cơ cấu động học song song treo gantry với 3 động cơ 120° và trục yaw thứ 4.
 */
export class DeltaRobot3D {
  constructor(options = {}) {
    this.basePosition = options.position || new THREE.Vector3(1.2, 2.25, -0.1);
    
    // Thông số hình học Delta (mét)
    this.rBase = 0.36;       // Bán kính động cơ trên đế (f)
    this.rPlatform = 0.085;  // Bán kính khớp nối trên bệ di động (e)
    this.L1 = 0.50;          // Chiều dài cánh tay trên (Upper Arm / Bicep)
    this.L2 = 0.96;          // Chiều dài thanh carbon song song (Parallelogram Rods)
    this.rodWidth = 0.09;    // Khoảng cách giữa 2 thanh trong mỗi cặp
    this.tipOffset = 0.11;   // Khoảng cách từ tâm bệ tới đầu giác hút
    
    // Góc servo 3 trục (Radian): M1, M2, M3
    this.theta = [0.35, 0.35, 0.35];
    this.theta4 = 0.0; // Góc xoay yaw đầu hút (Radian)
    
    this.platformPos = new THREE.Vector3(0, -0.85, 0);
    this.root = new THREE.Group();
    this.root.name = "FANUC_DeltaRobot4DOF";
    this.root.position.copy(this.basePosition);
    
    this.initMaterials();
    this.buildGantry();
    this.buildBaseAssembly();
    this.buildArmsAndPlatform();
    
    this.updateKinematicsFromAngles();
  }

  initMaterials() {
    this.matFanucYellow = new THREE.MeshStandardMaterial({ color: 0xfed100, roughness: 0.26, metalness: 0.16 });
    this.matDarkMetal = new THREE.MeshStandardMaterial({ color: 0x1e293b, roughness: 0.5, metalness: 0.7 });
    this.matSteelGantry = new THREE.MeshStandardMaterial({ color: 0x0f172a, roughness: 0.6, metalness: 0.8 });
    this.matCarbonRods = new THREE.MeshStandardMaterial({ color: 0xf8fafc, roughness: 0.3, metalness: 0.4 });
    this.matSuctionCup = new THREE.MeshStandardMaterial({ color: 0x0284c7, roughness: 0.5, metalness: 0.1 });
  }

  buildGantry() {
    // Khung giàn thép hộp 4 chân vững chắc đỡ đế robot trên cao
    this.gantryGroup = new THREE.Group();
    const colGeo = new THREE.BoxGeometry(0.09, this.basePosition.y + 0.15, 0.09);
    const postOffsets = [[-0.85, -0.70], [0.85, -0.70], [-0.85, 0.70], [0.85, 0.70]];
    const postHeight = this.basePosition.y + 0.15;
    const postCenterY = -this.basePosition.y + postHeight / 2;

    postOffsets.forEach(([ox, oz]) => {
      const col = new THREE.Mesh(colGeo, this.matSteelGantry);
      col.position.set(ox, postCenterY, oz);
      this.gantryGroup.add(col);
    });

    // Dầm giằng liên kết đỉnh gantry
    const beamX1 = new THREE.Mesh(new THREE.BoxGeometry(1.79, 0.09, 0.09), this.matSteelGantry);
    beamX1.position.set(0, 0.08, -0.70);
    const beamX2 = beamX1.clone();
    beamX2.position.set(0, 0.08, 0.70);
    this.gantryGroup.add(beamX1, beamX2);

    this.root.add(this.gantryGroup);
  }

  buildBaseAssembly() {
    // Đế chính hình lăng trụ tam giác vát tròn gắn 3 servo motor cách nhau 120°
    this.baseGroup = new THREE.Group();
    const baseDome = new THREE.Mesh(new THREE.CylinderGeometry(this.rBase + 0.12, this.rBase + 0.16, 0.14, 32), this.matFanucYellow);
    baseDome.position.y = 0.02;
    this.baseGroup.add(baseDome);
    this.root.add(this.baseGroup);
  }

  buildArmsAndPlatform() {
    this.arms = [];
    this.bicepMeshes = [];
    this.rodMeshes = [];

    // 3 Nhánh động học đặt tại góc phi = 0°, 120°, 240°
    const angles = [0, (2 * Math.PI) / 3, (4 * Math.PI) / 3];

    for (let i = 0; i < 3; i++) {
      const armRoot = new THREE.Group();
      armRoot.rotation.y = angles[i];
      this.baseGroup.add(armRoot);

      // Cánh tay trên (Upper Arm L1 = 0.50m) quay quanh trục ngang
      const bicepGroup = new THREE.Group();
      bicepGroup.position.set(this.rBase, 0, 0);

      const bicep = new THREE.Mesh(new THREE.BoxGeometry(this.L1, 0.05, 0.07), this.matFanucYellow);
      bicep.position.set(this.L1 / 2, 0, 0);
      bicepGroup.add(bicep);
      armRoot.add(bicepGroup);

      this.bicepMeshes.push(bicepGroup);

      // Cặp thanh carbon song song (Parallelogram L2 = 0.96m)
      const rodPairGroup = new THREE.Group();
      for (let sign of [-1, 1]) {
        const rod = new THREE.Mesh(new THREE.CylinderGeometry(0.010, 0.010, this.L2, 12), this.matCarbonRods);
        rod.position.z = sign * (this.rodWidth / 2);
        rodPairGroup.add(rod);
      }
      this.root.add(rodPairGroup);
      this.rodMeshes.push(rodPairGroup);

      this.arms.push({ armRoot, bicepGroup, rodPairGroup, phi: angles[i] });
    }

    // Bệ di động (Mobile Platform)
    this.platformGroup = new THREE.Group();
    this.platformGroup.name = "DeltaMobilePlatform";
    const platMesh = new THREE.Mesh(new THREE.CylinderGeometry(this.rPlatform, this.rPlatform, 0.025, 24), this.matDarkMetal);
    this.platformGroup.add(platMesh);

    // Trục thứ 4 xoay giác hút (Yaw axis)
    this.toolFlange = new THREE.Group();
    this.toolFlange.position.y = -0.015;

    // Giác hút chân không silicone (đặt tại -tipOffset = -0.11m)
    const suctionCup = new THREE.Mesh(new THREE.CylinderGeometry(0.022, 0.014, 0.04, 16), this.matSuctionCup);
    suctionCup.position.y = -this.tipOffset;
    this.toolFlange.add(suctionCup);

    this.platformGroup.add(this.toolFlange);
    this.root.add(this.platformGroup);
  }

  updateKinematicsFromAngles() {
    // Cập nhật góc quay của 3 cánh tay trên
    for (let i = 0; i < 3; i++) {
      this.bicepMeshes[i].rotation.z = -this.theta[i];
    }

    // Cập nhật vị trí bệ di động platformPos
    this.platformGroup.position.copy(this.platformPos);
    this.toolFlange.rotation.y = this.theta4;

    // Định hướng 3 cặp thanh carbon song song nối từ khuỷu bicep đến khớp cầu trên bệ
    for (let i = 0; i < 3; i++) {
      const phi = this.arms[i].phi;
      const th = this.theta[i];

      // Tọa độ điểm khuỷu tay trong hệ tọa độ root
      const elbowLocalX = this.rBase + this.L1 * Math.cos(th);
      const elbowLocalY = -this.L1 * Math.sin(th);
      const elbowWorld = new THREE.Vector3(
        elbowLocalX * Math.cos(phi),
        elbowLocalY,
        elbowLocalX * Math.sin(phi)
      );

      // Tọa độ khớp cầu trên bệ di động
      const platBallWorld = new THREE.Vector3(
        this.platformPos.x + this.rPlatform * Math.cos(phi),
        this.platformPos.y,
        this.platformPos.z + this.rPlatform * Math.sin(phi)
      );

      // Định vị và xoay cặp thanh carbon hướng đúng đường thẳng nối 2 khớp
      const rodGroup = this.rodMeshes[i];
      const mid = new THREE.Vector3().addVectors(elbowWorld, platBallWorld).multiplyScalar(0.5);
      rodGroup.position.copy(mid);

      const dir = new THREE.Vector3().subVectors(platBallWorld, elbowWorld).normalize();
      const up = new THREE.Vector3(0, 1, 0);
      const quat = new THREE.Quaternion().setFromUnitVectors(up, dir);
      rodGroup.setRotationFromQuaternion(quat);
    }
  }

  setJointAngles(m1, m2, m3, yawDeg = 0) {
    this.theta[0] = THREE.MathUtils.degToRad(m1);
    this.theta[1] = THREE.MathUtils.degToRad(m2);
    this.theta[2] = THREE.MathUtils.degToRad(m3);
    this.theta4 = THREE.MathUtils.degToRad(yawDeg);
    this.updateKinematicsFromAngles();
  }

  getJointAngles() {
    return {
      m1: THREE.MathUtils.radToDeg(this.theta[0]),
      m2: THREE.MathUtils.radToDeg(this.theta[1]),
      m3: THREE.MathUtils.radToDeg(this.theta[2]),
      yaw: THREE.MathUtils.radToDeg(this.theta4)
    };
  }

  getEndEffectorWorldPosition() {
    const v = new THREE.Vector3();
    if (this.toolFlange) {
      this.toolFlange.getWorldPosition(v);
      v.y -= this.tipOffset;
    }
    return v;
  }
}
```

=== FILE: js/scene/FactoryEnvironment.js ===
```javascript
import * as THREE from 'three';

/**
 * FactoryEnvironment.js
 * Dựng môi trường nhà máy đóng gói công nghiệp: Sàn epoxy, hàng rào bảo vệ,
 * 3 băng tải phối hợp, cổng quét thị giác Cognex và cảm biến quang.
 */
export class FactoryEnvironment {
  constructor(scene) {
    this.scene = scene;
    this.root = new THREE.Group();
    this.root.name = "FactoryEnvironment";
    this.scene.add(this.root);

    // Mảng quản lý đối tượng động trên dây chuyền
    this.infeedProducts = []; // Thanh sô-cô-la đơn lẻ trên Băng tải 1
    this.collatedPacks = [];  // Cụm 4 thanh gom trên Băng tải 2
    this.cartonBoxes = [];    // Thùng carton trên Băng tải 3

    // Trạng thái cảm biến vật lý
    this.sensorDI11 = false;  // Cảm biến cụm 4 thanh sẵn sàng tại X = -1.25m
    this.sensorDI12 = false;  // Cảm biến thùng carton đỗ tại stopper

    this.initMaterials();
    this.buildFloor();
    this.buildSafetyFence();
    this.buildConveyors();
    this.buildVisionStation();
    this.buildOpticalSensors();
    this.spawnInitialItems();
  }

  initMaterials() {
    this.matFloor = new THREE.MeshStandardMaterial({ color: 0x1e293b, roughness: 0.2, metalness: 0.1 });
    this.matBeltInfeed = new THREE.MeshStandardMaterial({ color: 0xf1f5f9, roughness: 0.6, metalness: 0.1 });
    this.matBeltBlack = new THREE.MeshStandardMaterial({ color: 0x0f172a, roughness: 0.8, metalness: 0.1 });
    this.matStainless = new THREE.MeshStandardMaterial({ color: 0x94a3b8, roughness: 0.3, metalness: 0.85 });
    this.matFenceYellow = new THREE.MeshStandardMaterial({ color: 0xfacc15, roughness: 0.4, metalness: 0.2 });
    this.matChocRed = new THREE.MeshStandardMaterial({ color: 0xd97706, roughness: 0.3, metalness: 0.4 });
    this.matCardboard = new THREE.MeshStandardMaterial({ color: 0xb45309, roughness: 0.85, metalness: 0.05 });
  }

  buildFloor() {
    // Sàn Epoxy bóng màu xám đen công nghiệp kèm lưới vạch chỉ dẫn an toàn
    const floor = new THREE.Mesh(new THREE.PlaneGeometry(16, 12), this.matFloor);
    floor.rotation.x = -Math.PI / 2;
    floor.receiveShadow = true;
    this.root.add(floor);
  }

  buildConveyors() {
    // BĂNG TẢI 1 (Infeed): Cấp phôi thanh bánh lẻ, X = +4.0 -> -4.0, Z = -0.32, Cao = 0.85m
    const conv1Group = new THREE.Group();
    conv1Group.position.set(0, 0.85, -0.32);
    const belt1 = new THREE.Mesh(new THREE.BoxGeometry(8.0, 0.04, 0.30), this.matBeltInfeed);
    belt1.receiveShadow = true;
    conv1Group.add(belt1);
    this.root.add(conv1Group);

    // BĂNG TẢI 2 (Collation): Gom cụm 4 thanh, X = +4.0 -> -4.0, Z = +0.18, Cao = 0.85m
    const conv2Group = new THREE.Group();
    conv2Group.position.set(0, 0.85, 0.18);
    const belt2 = new THREE.Mesh(new THREE.BoxGeometry(8.0, 0.04, 0.36), this.matBeltBlack);
    belt2.receiveShadow = true;
    conv2Group.add(belt2);
    this.root.add(conv2Group);

    // BĂNG TẢI 3 (Carton Box): Chở thùng carton, chạy dọc phía sau tại X = -1.25m, Z = -1.45m, Cao = 0.55m
    const conv3Group = new THREE.Group();
    conv3Group.position.set(-1.25, 0.55, -1.45);
    const belt3 = new THREE.Mesh(new THREE.BoxGeometry(4.0, 0.04, 0.45), this.matStainless);
    conv3Group.add(belt3);
    this.root.add(conv3Group);
  }

  buildVisionStation() {
    // Cổng quét Structured Light Cognex In-Sight tại X = 2.8m trên Băng tải 1
    const visionGroup = new THREE.Group();
    visionGroup.position.set(2.8, 1.45, -0.32);

    const camBody = new THREE.Mesh(new THREE.BoxGeometry(0.12, 0.08, 0.10), this.matStainless);
    visionGroup.add(camBody);

    // Quạt laser xanh lục quét ngang bề mặt băng tải
    const laserGeo = new THREE.ConeGeometry(0.25, 0.60, 16, 1, true);
    laserGeo.rotateX(Math.PI);
    const laserMat = new THREE.MeshBasicMaterial({ color: 0x00ff88, transparent: true, opacity: 0.25 });
    const laserMesh = new THREE.Mesh(laserGeo, laserMat);
    laserMesh.position.y = -0.30;
    visionGroup.add(laserMesh);

    // Đèn LED chỉ báo trạng thái chụp ảnh
    this.camStatusLed = new THREE.Mesh(new THREE.SphereGeometry(0.015, 8, 8), new THREE.MeshBasicMaterial({ color: 0x22c55e }));
    this.camStatusLed.position.set(0, 0.04, 0.05);
    visionGroup.add(this.camStatusLed);

    this.root.add(visionGroup);
  }

  buildOpticalSensors() {
    // Cảm biến quang DI_11 kiểm tra cụm 4 thanh tại X = -1.25m trên Băng tải 2
    this.sensorDI11Mesh = new THREE.Mesh(new THREE.CylinderGeometry(0.015, 0.015, 0.06, 16), this.matStainless);
    this.sensorDI11Mesh.rotation.z = Math.PI / 2;
    this.sensorDI11Mesh.position.set(-1.25, 0.90, 0.38);
    this.root.add(this.sensorDI11Mesh);
  }

  spawnInitialItems() {
    // Khởi tạo các thanh bánh lẻ trên Băng tải 1
    for (let i = 0; i < 5; i++) {
      this.spawnProduct(1.5 + i * 0.65, (Math.random() - 0.5) * 0.1, (Math.random() - 0.5) * 25);
    }
    // Khởi tạo thùng carton đầu tiên tại trạm đóng gói
    this.spawnEmptyBox(-1.25);
  }

  spawnProduct(x, zOffset = 0, rotYDeg = 0) {
    const mesh = new THREE.Mesh(new THREE.BoxGeometry(0.18, 0.035, 0.06), this.matChocRed);
    mesh.position.set(x, 0.88, -0.32 + zOffset);
    mesh.rotation.y = THREE.MathUtils.degToRad(rotYDeg);
    mesh.castShadow = true;
    this.root.add(mesh);
    const record = { mesh, picked: false };
    this.infeedProducts.push(record);
    return mesh;
  }

  spawnEmptyBox(x) {
    const boxGroup = new THREE.Group();
    boxGroup.position.set(x, 0.59, -1.45);
    // Vỏ thùng carton hở nắp
    const btm = new THREE.Mesh(new THREE.BoxGeometry(0.36, 0.02, 0.28), this.matCardboard);
    boxGroup.add(btm);
    this.root.add(boxGroup);
    const record = { mesh: boxGroup, full: false };
    this.cartonBoxes.push(record);
    return boxGroup;
  }

  placeSingleBarOnConveyor2(slotX = 1.20) {
    // Robot Delta đặt từng thanh vào vị trí ghép cụm 4 thanh trên Băng tải 2
    let activePack = this.collatedPacks.find(p => !p.complete && !p.picked);
    if (!activePack) {
      const packGroup = new THREE.Group();
      packGroup.position.set(slotX, 0.88, 0.18);
      this.root.add(packGroup);
      activePack = { mesh: packGroup, barCount: 0, complete: false, picked: false };
      this.collatedPacks.push(activePack);
    }

    const barIdx = activePack.barCount;
    const bar = new THREE.Mesh(new THREE.BoxGeometry(0.18, 0.035, 0.04), this.matChocRed);
    bar.position.set(0, 0, -0.06 + barIdx * 0.04);
    activePack.mesh.add(bar);
    activePack.barCount++;

    if (activePack.barCount >= 4) {
      activePack.complete = true; // Đủ 4 thanh -> Cụm sẵn sàng di chuyển tới DI_11
    }
  }
}
```

---

## PHẦN 4 — CẤU HÌNH & AN TOÀN BẢO MẬT

### 1. Báo cáo Tệp Cấu hình & Calibration
- Dự án là ứng dụng Digital Twin chạy thuần túy trên trình duyệt Web (Client-side HTML5/JavaScript ES Modules). Do đó, hệ thống không sử dụng các file cấu hình rời dạng `.yaml`, `.ini`, `.json`, `.toml` trong thư mục `config/`.
- Toàn bộ tham số cấu hình mạng và thông số hiệu chuẩn robot (DH Parameters, Encoder pulse ratios, Camera coordinates, TCP offsets) được tổ chức theo tiêu chuẩn hướng đối tượng rõ ràng trong các module mã nguồn:
  + Cấu hình Mạng & Giao thức: `js/hardware/YRC1000EthernetServer.js` (IP Client `192.168.1.1:10001`, IP Server `192.168.1.11:10040`, Protocol UDP BIN).
  + Cấu hình Bộ nhớ PLC: `js/hardware/MP3300HardwareBus.js` (64KB M-Register Buffer, Local I/O IW00C80/OW00C80).
  + Cấu hình Động học 6-DOF: `js/kinematics/Yaskawa6DOFKinematics.js` ($d_1 = 960\text{mm}, a_2 = 740\text{mm}, d_4 = 840\text{mm}, d_6 = 275\text{mm}$).
  + Cấu hình Động học Delta: `js/kinematics/Delta4DOFKinematics.js` ($f = 0.36\text{m}, e = 0.085\text{m}, rf = 0.50\text{m}, re = 0.96\text{m}$).

### 2. Kiểm toán An toàn & Redaction
- Toàn bộ dự án đã được rà soát tự động và thủ công:
  + Không tồn tại token, API secret, mật khẩu máy chủ, chuỗi kết nối cơ sở dữ liệu.
  + Không có file bí mật dạng `.env`, `*.pem`, `*.key`, `credentials.json`, `mcp_config.json`.
  + Không chứa thông tin khách hàng, tên công ty thực tế, email hoặc số điện thoại riêng tư.
  + Mọi địa chỉ IP xuất hiện trong mã nguồn đều là IP giải lập trong mạng LAN nội bộ tiêu chuẩn thiết bị thí nghiệm (`192.168.1.x`).
