# VÍ DỤ XUYÊN SUỐT — Từ Grid Map đến MPC Metrics

> **Mục đích:** Tài liệu này đi kèm `SELF_STUDY.md`. Dùng MỘT ví dụ duy nhất
> xuyên suốt 8 phần, với TẤT CẢ số liệu tính sẵn để sinh viên năm 2 có thể
> đối chiếu khi đọc code.
>
> **Bối cảnh:** Giả sử bạn là sinh viên được giao bài: "Cho robot đi từ góc
> trái dưới đến góc phải trên của bản đồ 6×6, có 3 vật cản ở giữa."

---

## Tình huống bài toán

```
Bạn có:
  - Bản đồ 6×6, mỗi ô 1 mét
  - 3 vật cản hình chữ L
  - Robot hình tròn, đường kính 0.3m
  - Cần đi từ (0,0) đến (5,5)

Câu hỏi:
  1. Đường đi ngắn nhất là gì?          → Part 3 (A*)
  2. Robot cần lái như thế nào?         → Part 4 (Trajectory) + Part 5 (Vehicle)
  3. MPC tính toán ra sao?              → Part 6 (MPC)
  4. Kết quả tốt hay xấu?              → Part 7 (Evaluation)
```

---

# Bước 1: Tạo bản đồ (Part 2)

## 1.1 Vẽ bản đồ ra giấy

```
Grid 6×6, resolution = 1.0 m/cell, origin = (0, 0)

     col0  col1  col2  col3  col4  col5
    ┌─────┬─────┬─────┬─────┬─────┬─────┐
row5│     │     │     │     │     │ GOAL│   y=5.0
    ├─────┼─────┼─────┼─────┼─────┼─────┤
row4│     │     │     │     │     │     │   y=4.0
    ├─────┼─────┼─────┼─────┼─────┼─────┤
row3│     │     │ ███ │     │     │     │   y=3.0
    ├─────┼─────┼─────┼─────┼─────┼─────┤
row2│     │     │ ███ │ ███ │     │     │   y=2.0
    ├─────┼─────┼─────┼─────┼─────┼─────┤
row1│     │     │     │     │     │     │   y=1.0
    ├─────┼─────┼─────┼─────┼─────┼─────┤
row0│START│     │     │     │     │     │   y=0.0
    └─────┴─────┴─────┴─────┴─────┴─────┘
    x=0   x=1   x=2   x=3   x=4   x=5

Vật cản: (row=2,col=2), (row=2,col=3), (row=3,col=2)
```

## 1.2 File YAML

```yaml
# maps/example_6x6.yaml
name: "example_6x6"
width: 6
height: 6
resolution: 1.0
origin: [0, 0]
obstacles:
  - [2, 2]   # row=2, col=2
  - [2, 3]   # row=2, col=3
  - [3, 2]   # row=3, col=2
```

## 1.3 Ma trận grid

```
grid = [
    [0, 0, 0, 0, 0, 0],   # row 0
    [0, 0, 0, 0, 0, 0],   # row 1
    [0, 0, 1, 1, 0, 0],   # row 2  ← vật cản tại col 2,3
    [0, 0, 1, 0, 0, 0],   # row 3  ← vật cản tại col 2
    [0, 0, 0, 0, 0, 0],   # row 4
    [0, 0, 0, 0, 0, 0],   # row 5
]

grid[2][2] = 1  →  "ô (row=2, col=2) là vật cản"
grid[0][0] = 0  →  "ô (row=0, col=0) là ô trống"
```

## 1.4 Chuyển đổi tọa độ

```
Cell (row, col) → World (x, y):
  x = origin_x + col × resolution = 0 + col × 1.0 = col
  y = origin_y + row × resolution = 0 + row × 1.0 = row

Ví dụ:
  Cell (0, 0) → x=0, y=0    (START)
  Cell (5, 5) → x=5, y=5    (GOAL)
  Cell (2, 2) → x=2, y=2    (vật cản)
  Cell (3, 2) → x=2, y=3    (vật cản)
```

---

# Bước 2: Tìm đường A* (Part 3)

## 2.1 Thiết lập

```
Start: cell (0, 0)  → world (0, 0)
Goal:  cell (5, 5)  → world (5, 5)
Heuristic: Euclidean distance
Movement: 8-connected (cho phép đi chéo)
```

## 2.2 Chạy A* từng bước

```
Bước 0: Khởi tạo
  open_set = {(0,0): f=0+h(0,0→5,5)=7.07}
  closed_set = {}

  h(0,0 → 5,5) = sqrt((5-0)² + (5-0)²) = sqrt(50) = 7.07

─────────────────────────────────────────────────────────────────

Bước 1: Lấy (0,0) ra, khám phá neighbors
  (0,0): g=0, h=7.07, f=7.07

  Neighbors (8-connected):
  ┌─────────┬─────────┬─────────┐
  │ (-1,-1) │ (-1, 0) │ (-1, 1) │  ← out of bounds
  ├─────────┼─────────┼─────────┤
  │ ( 0,-1) │  (0,0)  │ ( 0, 1) │  ← (0,-1) out of bounds
  ├─────────┼─────────┼─────────┤
  │ ( 1,-1) │ ( 1, 0) │ ( 1, 1) │  ← (1,-1) out of bounds
  └─────────┴─────────┴─────────┘

  Hợp lệ: (0,1), (1,0), (1,1)

  Tính f:
  ┌──────────┬───────┬───────┬───────┬─────────────────────────┐
  │ Neighbor │ g     │ h     │ f     │ Chi tiết                │
  ├──────────┼───────┼───────┼───────┼─────────────────────────┤
  │ (0, 1)   │ 1.00  │ 6.40  │ 7.40  │ g=0+1, h=sqrt(25+16)   │
  │ (1, 0)   │ 1.00  │ 6.40  │ 7.40  │ g=0+1, h=sqrt(16+25)   │
  │ (1, 1)   │ 1.41  │ 5.66  │ 7.07  │ g=0+1.41, h=sqrt(16+16)│
  └──────────┴───────┴───────┴───────┴─────────────────────────┘

  open_set = {(0,1):7.40, (1,0):7.40, (1,1):7.07}
  closed_set = {(0,0)}
  came_from = {(0,1):(0,0), (1,0):(0,0), (1,1):(0,0)}

  Chọn (1,1) vì f thấp nhất (7.07)

─────────────────────────────────────────────────────────────────

Bước 2: Lấy (1,1) ra, khám phá neighbors
  (1,1): g=1.41, h=5.66, f=7.07

  Hợp lệ (không out of bounds, không vật cản, không trong closed):
  (0,1) đã trong open → kiểm tra g mới
  (1,0) đã trong open → kiểm tra g mới
  (0,2), (1,2), (2,0), (2,1), (2,2)=VẬT CẢN!

  ┌──────────┬───────┬───────┬───────┬─────────────────────────┐
  │ Neighbor │ g     │ h     │ f     │ Ghi chú                │
  ├──────────┼───────┼───────┼───────┼─────────────────────────┤
  │ (0, 2)   │ 2.41  │ 5.00  │ 7.41  │ g=1.41+1               │
  │ (1, 2)   │ 2.41  │ 4.24  │ 6.65  │ g=1.41+1               │
  │ (2, 0)   │ 2.41  │ 5.00  │ 7.41  │ g=1.41+1               │
  │ (2, 1)   │ 2.41  │ 4.24  │ 6.65  │ g=1.41+1               │
  │ (2, 2)   │  —    │  —    │  —    │ VẬT CẢN → bỏ qua       │
  └──────────┴───────┴───────┴───────┴─────────────────────────┘

  open_set = {(0,1):7.40, (1,0):7.40, (0,2):7.41, (1,2):6.65,
              (2,0):7.41, (2,1):6.65}
  closed_set = {(0,0), (1,1)}

  Chọn (1,2) vì f thấp nhất (6.65)

─────────────────────────────────────────────────────────────────

Bước 3–N: Tiếp tục... (sinh viên tự tính)

Đường đi cuối cùng (kết quả):
  (0,0) → (1,1) → (1,2) → (1,3) → (2,4) → (3,4) → (4,5) → (5,5)

  Hoặc:
  (0,0) → (1,1) → (1,2) → (1,3) → (1,4) → (2,4) → (3,4) → (4,4) → (5,5)
```

## 2.3 Kết quả A*

```python
path_cells = [(0,0), (1,1), (1,2), (1,3), (2,4), (3,4), (4,5), (5,5)]
#             start   đi chéo  →      →     đi chéo  →     đi chéo  goal
```

```
Vẽ trên bản đồ:

     col0  col1  col2  col3  col4  col5
    ┌─────┬─────┬─────┬─────┬─────┬─────┐
row5│     │     │     │     │     │ ● G │
    ├─────┼─────┼─────┼─────┼─────┼─────┤
row4│     │     │     │     │     │ ●   │  ← (4,5)
    ├─────┼─────┼─────┼─────┼─────┼─────┤
row3│     │     │ ███ │     │ ●   │     │  ← (3,4)
    ├─────┼─────┼─────┼─────┼─────┼─────┤
row2│     │     │ ███ │ ███ │ ●   │     │  ← (2,4)
    ├─────┼─────┼─────┼─────┼─────┼─────┤
row1│     │ ●   │ ●   │ ●   │     │     │  ← (1,1),(1,2),(1,3)
    ├─────┼─────┼─────┼─────┼─────┼─────┤
row0│ S   │     │     │     │     │     │  ← (0,0) start
    └─────┴─────┴─────┴─────┴─────┴─────┘

Path length = 7 segments
  - 3 diagonal (×1.41) = 4.24
  - 4 straight (×1.0)  = 4.00
  Total ≈ 8.24 mét
```

## 2.4 Chuyển sang world coordinates

```python
path_xy = [(0,0), (1,1), (1,2), (1,3), (2,4), (3,4), (4,5), (5,5)]
# Vì resolution=1.0 và origin=(0,0), cell coords = world coords
```

---

# Bước 3: Sinh Trajectory (Part 4)

## 3.1 Tính arc length

```
Path: [(0,0), (1,1), (1,2), (1,3), (2,4), (3,4), (4,5), (5,5)]

Segment lengths:
  (0,0)→(1,1): sqrt((1-0)²+(1-0)²) = sqrt(2) = 1.414
  (1,1)→(1,2): sqrt((1-1)²+(2-1)²) = 1.000
  (1,2)→(1,3): sqrt((1-1)²+(3-2)²) = 1.000
  (1,3)→(2,4): sqrt((2-1)²+(4-3)²) = 1.414
  (2,4)→(3,4): sqrt((3-2)²+(4-4)²) = 1.000
  (3,4)→(4,5): sqrt((4-3)²+(5-4)²) = 1.414
  (4,5)→(5,5): sqrt((5-4)²+(5-5)²) = 1.000

Cumulative arc length:
  s = [0.000, 1.414, 2.414, 3.414, 4.828, 5.828, 7.243, 8.243]
```

## 3.2 Tính thời gian và số điểm

```
target_speed = 1.0 m/s
total_length = 8.243 m
total_time = 8.243 / 1.0 = 8.243 s
dt = 0.1 s
n_steps = 8.243 / 0.1 ≈ 83 steps

→ Trajectory có 83 điểm, mỗi điểm cách nhau 0.1s
```

## 3.3 Nội suy (interpolation)

```
Tại t=0.0s: s=0.000 → x=0.000, y=0.000
Tại t=0.1s: s=0.100 → x≈0.071, y≈0.071  (trên đoạn (0,0)→(1,1))
Tại t=0.2s: s=0.200 → x≈0.141, y≈0.141
...
Tại t=1.4s: s=1.400 → x≈0.990, y≈0.990  (gần cuối đoạn 1)
Tại t=1.5s: s=1.500 → x=1.000, y=1.071  (bắt đầu đoạn 2)
...
Tại t=8.2s: s=8.200 → x≈4.970, y≈4.970  (gần goal)
```

## 3.4 Tính yaw

```
yaw = atan2(dy, dx)

Đoạn (0,0)→(1,1):
  dx=1, dy=1
  yaw = atan2(1, 1) = π/4 = 0.785 rad = 45°

Đoạn (1,1)→(1,2):
  dx=0, dy=1
  yaw = atan2(1, 0) = π/2 = 1.571 rad = 90°

Đoạn (1,2)→(1,3):
  dx=0, dy=1
  yaw = π/2 = 1.571 rad

Đoạn (1,3)→(2,4):
  dx=1, dy=1
  yaw = π/4 = 0.785 rad

Đoạn (2,4)→(3,4):
  dx=1, dy=0
  yaw = atan2(0, 1) = 0 rad = 0°

Đoạn (3,4)→(4,5):
  dx=1, dy=1
  yaw = π/4 = 0.785 rad

Đoạn (4,5)→(5,5):
  dx=1, dy=0
  yaw = 0 rad
```

## 3.5 Bảng trajectory (10 điểm đầu)

```
┌───────┬───────┬───────┬────────┬─────┐
│ t (s) │ x (m) │ y (m) │ yaw(r) │ v   │
├───────┼───────┼───────┼────────┼─────┤
│ 0.0   │ 0.000 │ 0.000 │ 0.785  │ 1.0 │
│ 0.1   │ 0.071 │ 0.071 │ 0.785  │ 1.0 │
│ 0.2   │ 0.141 │ 0.141 │ 0.785  │ 1.0 │
│ 0.3   │ 0.212 │ 0.212 │ 0.785  │ 1.0 │
│ 0.4   │ 0.283 │ 0.283 │ 0.785  │ 1.0 │
│ 0.5   │ 0.354 │ 0.354 │ 0.785  │ 1.0 │
│ 0.6   │ 0.424 │ 0.424 │ 0.785  │ 1.0 │
│ 0.7   │ 0.495 │ 0.495 │ 0.785  │ 1.0 │
│ 0.8   │ 0.566 │ 0.566 │ 0.785  │ 1.0 │
│ 0.9   │ 0.636 │ 0.636 │ 0.785  │ 1.0 │
│ ...   │ ...   │ ...   │ ...    │ ... │
│ 1.4   │ 0.990 │ 0.990 │ 0.785  │ 1.0 │
│ 1.5   │ 1.000 │ 1.071 │ 1.571  │ 1.0 │  ← yaw thay đổi!
│ ...   │ ...   │ ...   │ ...    │ ... │
└───────┴───────┴───────┴────────┴─────┘
```

---

# Bước 4: Mô hình xe (Part 5)

## 4.1 Xe tại đâu?

```
State ban đầu: [x=0, y=0, yaw=0.785, v=1.0]
  - Vị trí: (0, 0)
  - Hướng: 45° (đi chéo lên phải)
  - Tốc độ: 1.0 m/s

Control: [steer=0, accel=0]  (giữ nguyên hướng, giữ nguyên tốc)
  - steer=0 → không quay bánh
  - accel=0 → không tăng/giảm tốc
```

## 4.2 Tính bước tiếp theo (dt=0.1s, L=0.3m)

```
x[t+1] = x[t] + v[t] × cos(yaw[t]) × dt
       = 0 + 1.0 × cos(0.785) × 0.1
       = 1.0 × 0.707 × 0.1
       = 0.0707

y[t+1] = y[t] + v[t] × sin(yaw[t]) × dt
       = 0 + 1.0 × sin(0.785) × 0.1
       = 1.0 × 0.707 × 0.1
       = 0.0707

yaw[t+1] = yaw[t] + (v[t]/L) × tan(steer[t]) × dt
         = 0.785 + (1.0/0.3) × tan(0) × 0.1
         = 0.785 + 3.333 × 0 × 0.1
         = 0.785  (không đổi vì steer=0)

v[t+1] = v[t] + accel[t] × dt
       = 1.0 + 0 × 0.1
       = 1.0  (không đổi vì accel=0)

State mới: [0.0707, 0.0707, 0.785, 1.0]
```

## 4.3 Tính 5 bước liên tiếp (steer=0, accel=0)

```
┌───────┬────────┬────────┬────────┬─────┐
│ Step  │ x      │ y      │ yaw    │ v   │
├───────┼────────┼────────┼────────┼─────┤
│ 0     │ 0.0000 │ 0.0000 │ 0.785  │ 1.0 │
│ 1     │ 0.0707 │ 0.0707 │ 0.785  │ 1.0 │
│ 2     │ 0.1414 │ 0.1414 │ 0.785  │ 1.0 │
│ 3     │ 0.2121 │ 0.2121 │ 0.785  │ 1.0 │
│ 4     │ 0.2828 │ 0.2828 │ 0.785  │ 1.0 │
│ 5     │ 0.3536 │ 0.3536 │ 0.785  │ 1.0 │
└───────┴────────┴────────┴────────┴─────┘

→ Xe đi thẳng theo hướng 45°, mỗi bước 0.1m
```

## 4.4 Cho steer=0.3 rad (rẽ phải)

```
yaw[t+1] = 0.785 + (1.0/0.3) × tan(0.3) × 0.1
         = 0.785 + 3.333 × 0.309 × 0.1
         = 0.785 + 0.103
         = 0.888 rad ≈ 50.9°

→ Yaw tăng → xe rẽ trái (theo convention counter-clockwise)

Nếu steer=-0.3:
  yaw[t+1] = 0.785 + 3.333 × (-0.309) × 0.1
           = 0.785 - 0.103
           = 0.682 rad ≈ 39.1°
  → Yaw giảm → xe rẽ phải
```

---

# Bước 5: MPC (Part 6)

## 5.1 Thiết lập MPC

```
Config:
  dt = 0.1s
  horizon N = 5       (dự đoán 5 bước)
  Q = [1.0, 1.0, 0.5, 0.1]   (trọng số: x, y, yaw, v)
  R = [0.1, 0.1]              (trọng số: steer, accel)
  max_steer = 0.5 rad
  max_accel = 2.0 m/s²
  L = 0.3m
```

## 5.2 MPC nhìn thấy gì?

```
Tại step=0:
  Current state: [0, 0, 0.785, 1.0]

  Reference window (5 bước tiếp theo từ trajectory):
  ┌───────┬───────┬───────┬────────┬─────┐
  │ k     │ x_ref │ y_ref │ yaw_ref│ v_ref│
  ├───────┼───────┼───────┼────────┼─────┤
  │ 0     │ 0.000 │ 0.000 │ 0.785  │ 1.0 │
  │ 1     │ 0.071 │ 0.071 │ 0.785  │ 1.0 │
  │ 2     │ 0.141 │ 0.141 │ 0.785  │ 1.0 │
  │ 3     │ 0.212 │ 0.212 │ 0.785  │ 1.0 │
  │ 4     │ 0.283 │ 0.283 │ 0.785  │ 1.0 │
  │ 5     │ 0.354 │ 0.354 │ 0.785  │ 1.0 │
  └───────┴───────┴───────┴────────┴─────┘
```

## 5.3 MPC giải bài toán gì?

```
MPC cần tìm: u[0], u[1], u[2], u[3], u[4]  (5 cặp steer, accel)

Để minimize:
  J = Σ[k=0→4] (x[k]-x_ref[k])²×1.0    ← sai số x
    + Σ[k=0→4] (y[k]-y_ref[k])²×1.0    ← sai số y
    + Σ[k=0→4] (yaw[k]-yaw_ref[k])²×0.5 ← sai số yaw
    + Σ[k=0→4] (v[k]-v_ref[k])²×0.1    ← sai số v
    + Σ[k=0→4] steer[k]²×0.1            ← nỗ lực lái
    + Σ[k=0→4] accel[k]²×0.1            ← nỗ lực tăng tốc
    + terminal cost (×2)

Subject to:
  x[k+1] = x[k] + v[k]×cos(yaw[k])×dt
  y[k+1] = y[k] + v[k]×sin(yaw[k])×dt
  yaw[k+1] = yaw[k] + (v[k]/L)×tan(steer[k])×dt
  v[k+1] = v[k] + accel[k]×dt

  |steer[k]| ≤ 0.5
  |accel[k]| ≤ 2.0
  -0.5 ≤ v[k] ≤ 3.0
```

## 5.4 Kết quả MPC (giả sử solver tìm được)

```
MPC trả về control đầu tiên: steer=0.02, accel=0.05
  (gần 0 vì xe đang đi đúng hướng, chỉ cần điều chỉnh nhỏ)

Áp dụng cho robot:
  state mới = vehicle.step([0, 0, 0.785, 1.0], [0.02, 0.05])

  x = 0 + 1.0 × cos(0.785) × 0.1 = 0.0707
  y = 0 + 1.0 × sin(0.785) × 0.1 = 0.0707
  yaw = 0.785 + (1.0/0.3) × tan(0.02) × 0.1
      = 0.785 + 3.333 × 0.020 × 0.1
      = 0.785 + 0.0067
      = 0.7917
  v = 1.0 + 0.05 × 0.1 = 1.005

State mới: [0.0707, 0.0707, 0.7917, 1.005]
```

## 5.5 Lặp lại cho step tiếp theo

```
Step 1:
  Current state: [0.0707, 0.0707, 0.7917, 1.005]
  Reference window: trajectory[1:7]  (dịch 1 bước)

  MPC giải lại → control mới: steer=-0.01, accel=0.02
  State mới: [0.141, 0.142, 0.790, 1.007]

Step 2:
  Current state: [0.141, 0.142, 0.790, 1.007]
  ...

Cứ lặp cho đến khi:
  - Đến goal (khoảng cách < 0.3m), HOẶC
  - Hết max_steps
```

---

# Bước 6: Simulation Log (Part 7)

## 6.1 Log entries (5 bước đầu)

```
┌──────┬────────┬────────┬────────┬───────┬────────┬───────┬───────────┬──────────┐
│ t    │ x      │ y      │ yaw    │ v     │ steer  │ accel │ collision │ min_dist │
├──────┼────────┼────────┼────────┼───────┼────────┼───────┼───────────┼──────────┤
│ 0.0  │ 0.000  │ 0.000  │ 0.785  │ 1.000 │ 0.020  │ 0.050 │ 0         │ 2.83     │
│ 0.1  │ 0.071  │ 0.071  │ 0.792  │ 1.005 │ -0.010 │ 0.020 │ 0         │ 2.69     │
│ 0.2  │ 0.141  │ 0.142  │ 0.790  │ 1.007 │ 0.015  │ 0.030 │ 0         │ 2.55     │
│ 0.3  │ 0.212  │ 0.213  │ 0.793  │ 1.010 │ -0.005 │ 0.010 │ 0         │ 2.41     │
│ 0.4  │ 0.283  │ 0.284  │ 0.791  │ 1.011 │ 0.008  │ 0.025 │ 0         │ 2.27     │
└──────┴────────┴────────┴────────┴───────┴────────┴───────┴───────────┴──────────┘

Giải thích:
  - collision=0: không va chạm (vì đi đường vòng vật cản)
  - min_dist: khoảng cách đến vật cản gần nhất
    Tại (0,0): dist đến (2,2) = sqrt(4+4) = 2.83m
  - steer gần 0: xe đang đi đúng hướng
  - accel gần 0: tốc độ gần đúng
```

## 6.2 Tính RMSE (5 bước đầu)

```
RMSE = sqrt(1/N × Σ((x[i]-x_ref[i])² + (y[i]-y_ref[i])²))

┌──────┬────────┬────────┬──────────┬──────────┬──────────────────┐
│ Step │ x      │ x_ref  │ x_err    │ y_err    │ (x_err²+y_err²) │
├──────┼────────┼────────┼──────────┼──────────┼──────────────────┤
│ 0    │ 0.000  │ 0.000  │ 0.000    │ 0.000    │ 0.000000         │
│ 1    │ 0.071  │ 0.071  │ 0.000    │ 0.000    │ 0.000000         │
│ 2    │ 0.141  │ 0.141  │ 0.000    │ 0.001    │ 0.000001         │
│ 3    │ 0.212  │ 0.212  │ 0.000    │ 0.001    │ 0.000001         │
│ 4    │ 0.283  │ 0.283  │ 0.000    │ 0.001    │ 0.000001         │
└──────┴────────┴────────┴──────────┴──────────┴──────────────────┘

Sum = 0.000003
Mean = 0.000003 / 5 = 0.0000006
RMSE = sqrt(0.0000006) ≈ 0.0008 m = 0.8 mm

→ Rất tốt! (< 0.1m)
```

## 6.3 Tính Control Smoothness

```
Steer smoothness = mean(|steer[i+1] - steer[i]|)

|steer[1]-steer[0]| = |-0.010 - 0.020| = 0.030
|steer[2]-steer[1]| = |0.015 - (-0.010)| = 0.025
|steer[3]-steer[2]| = |-0.005 - 0.015| = 0.020
|steer[4]-steer[3]| = |0.008 - (-0.005)| = 0.013

Mean = (0.030 + 0.025 + 0.020 + 0.013) / 4 = 0.022

→ Smoothness = 0.022 (< 0.05 → tốt)
```

---

# Bước 7: So sánh cấu hình (Part 7)

## 7.1 Chạy với 3 cấu hình khác nhau

```
┌─────────────┬──────────┬──────────┬──────────┬───────────┬───────────────────┐
│ Config      │ horizon  │ Q        │ R        │ RMSE (m)  │ Smoothness        │
├─────────────┼──────────┼──────────┼──────────┼───────────┼───────────────────┤
│ Baseline    │ 10       │ [1,1,    │ [0.1,    │ 0.0234    │ 0.0156            │
│             │          │  0.5,0.1]│  0.1]    │           │                   │
├─────────────┼──────────┼──────────┼──────────┼───────────┼───────────────────┤
│ Conservative│ 8        │ [2,2,    │ [0.5,    │ 0.0891    │ 0.0089 ← mượt nhất│
│             │          │  1,0.2]  │  0.5]    │           │                   │
├─────────────┼──────────┼──────────┼──────────┼───────────┼───────────────────┤
│ Aggressive  │ 15       │ [0.5,0.5,│ [0.05,   │ 0.0112    │ 0.0312 ← RMSE thấp│
│             │          │  0.2,0.05│  0.05]   │           │                   │
└─────────────┴──────────┴──────────┴──────────┴───────────┴───────────────────┘

Nhận xét:
  Conservative: R lớn → ít thay đổi steer → mượt nhưng track kém
  Aggressive:   Q lớn, R nhỏ → track tốt nhưng steer "gắt"
  Baseline:     Cân bằng
```

## 7.2 Tại sao Conservative RMSE cao hơn?

```
Conservative: R=[0.5, 0.5]
  → MPC "sợ" thay đổi steer/accel
  → Chỉ dám điều khiển nhẹ
  → Xe đi lệch reference một chút nhưng không dám sửa
  → RMSE cao hơn

Aggressive: R=[0.05, 0.05]
  → MPC "thoải mái" thay đổi steer/accel
  → Sửa sai nhanh, bám sát reference
  → RMSE thấp hơn nhưng steer thay đổi nhiều → không mượt
```

---

# Bước 8: AI Agent (Part 8)

## 8.1 Agent nhận mục tiêu

```
Goal: {
  "rmse_target": 0.05,      # RMSE phải ≤ 5cm
  "collision_target": 0,    # Không được va chạm
  "max_trials": 5           # Thử tối đa 5 cấu hình
}
```

## 8.2 Agent chạy từng trial

```
Trial 1: dùng baseline
  config = {horizon:10, Q:[1,1,0.5,0.1], R:[0.1,0.1], ...}
  → RMSE = 0.0234, collision = 0
  → Đạt goal! (0.0234 ≤ 0.05 và 0 ≤ 0)
  → Lưu là best

Trial 2: thử aggressive (vì muốn RMSE thấp hơn nữa)
  config = {horizon:15, Q:[0.5,0.5,0.2,0.05], R:[0.05,0.05], ...}
  → RMSE = 0.0112, collision = 0
  → Tốt hơn! Cập nhật best

Trial 3: thử tăng horizon thêm
  config = {horizon:20, Q:[0.5,0.5,0.2,0.05], R:[0.05,0.05], ...}
  → RMSE = 0.0098, collision = 0
  → Tốt hơn nữa!

Trial 4: thử R nhỏ hơn
  config = {horizon:20, Q:[0.5,0.5,0.2,0.05], R:[0.01,0.01], ...}
  → RMSE = 0.0085, collision = 0
  → Nhưng steer_smoothness = 0.089 (quá cao!)

Trial 5: cân bằng
  config = {horizon:15, Q:[1,1,0.5,0.1], R:[0.05,0.05], ...}
  → RMSE = 0.0156, collision = 0
  → Smoothness = 0.0198 (tốt)

Kết quả: Agent chọn Trial 4 (RMSE thấp nhất) hoặc Trial 5 (cân bằng)
```

## 8.3 Agent output

```json
{
  "best_config": {
    "horizon": 20,
    "Q": [0.5, 0.5, 0.2, 0.05],
    "R": [0.01, 0.01],
    "max_steer": 0.5,
    "max_accel": 2.0,
    "max_speed": 3.0,
    "vehicle_length": 0.3,
    "num_iterations": 3
  },
  "best_metrics": {
    "position_rmse": 0.0085,
    "collision_count": 0,
    "steering_smoothness": 0.089
  },
  "trials": 5
}
```

---

# Tóm tắt: Dữ liệu chạy suốt ví dụ

```
┌─────────────────────────────────────────────────────────────────────────┐
│  INPUT                                                                  │
│  ─────                                                                  │
│  Map: 6×6, resolution=1.0m, obstacles=[(2,2),(2,3),(3,2)]             │
│  Start: (0,0), Goal: (5,5)                                             │
│  Vehicle: L=0.3m, dt=0.1s                                              │
│  MPC: horizon=5, Q=[1,1,0.5,0.1], R=[0.1,0.1]                         │
│                                                                         │
│  PROCESS                                                                │
│  ───────                                                                │
│  A*: path = [(0,0),(1,1),(1,2),(1,3),(2,4),(3,4),(4,5),(5,5)]         │
│  Trajectory: 83 points, duration=8.24s                                 │
│  Simulation: 83 steps, 0 collisions                                    │
│                                                                         │
│  OUTPUT                                                                 │
│  ──────                                                                 │
│  RMSE = 0.0234 m                                                        │
│  Collision = 0                                                          │
│  Smoothness = 0.0156                                                    │
│  Goal reached = True                                                    │
└─────────────────────────────────────────────────────────────────────────┘
```

---

# Bài tập tổng hợp

Sau khi đọc xong ví dụ này, sinh viên nên:

- [ ] Vẽ lại bản đồ 6×6 ra giấy, đánh dấu vật cản
- [ ] Tính tay heuristic h(start, goal) = ?
- [ ] Chạy A* trên giấy, ghi lại g, h, f cho mỗi node
- [ ] Tính arc length cho path kết quả
- [ ] Tính yaw cho 3 đoạn đầu tiên
- [ ] Tính vehicle.step() với steer=0.2, accel=0.5
- [ ] Tính cost J cho 1 bước MPC (dùng Q, R cho sẵn)
- [ ] Chạy `python run_pipeline.py --scenario basic_circle` và so sánh kết quả
