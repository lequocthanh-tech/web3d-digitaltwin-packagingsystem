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
