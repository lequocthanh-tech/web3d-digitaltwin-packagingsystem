# Web 3D Digital Twin - Dây Chuyền Đóng Gói Phối Hợp Robot Delta & Robot Arm 6-DOF

Mô phỏng 3D Digital Twin thời gian thực (60 FPS) dây chuyền đóng gói công nghiệp phối hợp giữa **Robot Delta 4-DOF** và **Robot Arm 6-DOF (Cổ tay cầu Spherical Wrist)** theo tiêu chuẩn công nghiệp FANUC / Yaskawa.

Dự án được xây dựng thuần trên **HTML5, CSS3 (Giao diện Industrial Dark SCADA)** và **Three.js (ES Module qua CDN + OrbitControls + Factory I/O Free Look)**, chạy trực tiếp trên trình duyệt web mà không cần cài đặt hoặc build phức tạp.

---

## 🌟 Tính Năng Nổi Bật

1. **Robot Song Song Delta 4-DOF & Robot Tay Máy 6-DOF**:
   - Thuật toán Động học Thuận/Nghịch (FK / IK) chính xác 100%.
   - Chuyển động nội suy trơn mượt `MOVJ` và `MOVL` bậc 5 (Quintic polynomial).
   - Hệ trục tọa độ Denavit-Hartenberg (DH Frames `{0}..{6}`) trực quan.
   - Đầu hút chân không nắn phôi kẹo, xếp cụm 4 thanh và đóng thùng carton 2 lớp.

2. **Dây Chuyền Đóng Gói Hoàn Chỉnh**:
   - 3 băng tải phối hợp (Infeed, Collation, Box Conveyor).
   - Cổng kiểm tra quang học Cognex Vision với cửa sổ quét PIP thời gian thực.
   - Máy dán thùng carton (Case Sealer) tự động gập 4 nắp và dán băng keo niêm phong.
   - Xe tự hành AGV/AMR bốc dỡ pallet thùng carton vào kho bãi.
   - Tủ điện điều khiển công nghiệp tích hợp PLC Yaskawa MP3300, YRC1000micro, biến tần J1000 và Servo Sigma-7 với chế độ soi mở cửa tủ.

3. **Hệ Thống Camera Đa Chế Độ & Factory I/O Free / Fly Cam**:
   - Chuyển nhanh 7 góc nhìn preset (Overview, Front, Delta, Arm, Top-Down, AGV, Cabinet).
   - Chế độ điều khiển tự do **Factory I/O Free / Fly Camera**: dùng 4 phím mũi tên `↑↓←→` để di chuyển, `Shift` tăng tốc, `Space/C` chỉnh cao độ, chuột phải xoay nhìn tự do $360^\circ$.
   - Tính năng **Đặt Tâm Xoay (Pivot Point)** với phím tắt `F` hoặc Double-click.

4. **4 Trạm Kỹ Thuật Chuyên Sâu (Engineering Workstations)**:
   - **Station 1**: Yaskawa Teach Pendant với ngôn ngữ lập trình robot INFORM.
   - **Station 2**: MPE720 Studio với sơ đồ mạch Ladder Logic và Motion Program.
   - **Station 3**: C# / Qt PC Control & Vision Studio điều khiển robot qua Ethernet UDP (Port 10040).
   - **Station 4**: Màn hình cảm ứng công nghiệp VIPA HMI SCADA & Dao động ký Telemetry giám sát góc khớp và mô-men xoắn thời gian thực.
   - **Đồ thị động học kiểu MATLAB**: Phân tích góc xoay $\theta$, vận tốc góc $\omega$ và gia tốc góc $\alpha$.

---

## 📁 Cấu trúc Thư mục Dự án

```
simulationsystem/
├── index.html                     # Giao diện chính: 3D Viewport + SCADA Panels + Workstations
├── css/
│   └── style.css                  # Industrial Dark SCADA Theme, Responsive Layout & Animations
├── js/
│   ├── main.js                    # Entry point: Three.js Loop, Factory I/O Camera, State Manager
│   ├── scene/
│   │   └── FactoryEnvironment.js  # Sàn epoxy, băng tải, máy dán thùng, tủ điện, rào an toàn
│   ├── robots/
│   │   ├── DeltaRobot3D.js        # Mô hình & động học cơ cấu Delta 4-DOF
│   │   └── ArticulatedArm6DOF.js  # Mô hình & động học Robot Arm 6 bậc cổ tay cầu
│   ├── kinematics/
│   │   ├── Yaskawa6DOFKinematics.js # Động học thuận/nghịch DH & xung encoder
│   │   ├── Delta4DOFKinematics.js   # Động học song song trilateration
│   │   ├── TrajectoryPlanner.js     # Bộ nội suy đa thức MOVJ / MOVL bậc 5
│   │   ├── TrajectoryTrail.js       # Vẽ tuyến quỹ đạo 3D thời gian thực
│   │   └── MatlabKinematicsPlotter.js # Đồ thị động học góc/vận tốc/gia tốc
│   ├── physics/
│   │   └── VacuumAndSensorEngine.js # Cảm biến quang, giác hút chân không & gạt lỗi
│   ├── hardware/
│   │   ├── MP3300HardwareBus.js    # PLC ảo MP3300 chu kỳ quét 50 Hz & bộ nhớ thanh ghi
│   │   └── YRC1000EthernetServer.js # Server UDP giao thức YERC High-Speed Ethernet
│   ├── workstations/              # 4 trạm lập trình kỹ thuật (Teach Pendant, MPE720, C# Studio)
│   ├── hmi-scada/                 # VIPA HMI, dao động ký Telemetry, giả lập sự cố Chaos Lab
│   ├── vision/                    # Bộ điều khiển thị giác máy tính Cognex In-Sight
│   └── logistics/                 # Xe tự hành AGV/AMR điều phối pallet
├── CHAY_HE_THONG.bat              # Script 1-click khởi chạy Server và mở trình duyệt
├── TASK.md                        # Nhật ký chi tiết 27 nhiệm vụ phát triển
└── README.md                      # Tài liệu hướng dẫn dự án
```

---

## 🚀 Hướng Dẫn Chạy

Do các file JavaScript sử dụng **ES Modules** (`import` / `export`), vui lòng chạy thông qua một web server nội bộ (không mở trực tiếp `file://`):

### Cách 1: Chạy file Batch (Khuyên dùng trên Windows)
Nhấp đúp chuột vào file:
```
CHAY_HE_THONG.bat
```
Script sẽ tự động khởi động server và mở trình duyệt tại `http://localhost:8080`.

### Cách 2: Sử dụng Python (Có sẵn)
Mở terminal tại thư mục dự án và chạy:
```bash
python -m http.server 8080
```
Truy cập trình duyệt: **[http://localhost:8080](http://localhost:8080)**

### Cách 3: Sử dụng VS Code Live Server
1. Mở thư mục dự án bằng VS Code.
2. Click chuột phải vào `index.html` chọn **Open with Live Server**.

### Cách 4: Sử dụng Node.js
```bash
npx serve . -p 8080
```
