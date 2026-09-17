# Concepts — Kiến thức nền tảng

Tài liệu này giải thích chi tiết các khái niệm cốt lõi mà sinh viên cần hiểu trước khi đọc code.

---

## 1. Tối ưu lồi (Convex Optimization)

### 1.1 Bài toán tối ưu tổng quát

```
minimize    f(x)
subject to  g_i(x) ≤ 0,  i = 1,...,m
            h_j(x) = 0,  j = 1,...,p

Trong đó:
  f(x)     : hàm mục tiêu (objective function)
  g_i(x)   : ràng buộc bất đẳng thức
  h_j(x)   : ràng buộc đẳng thức
  x        : vector biến quyết định (decision variables)
```

### 1.2 Convex vs Non-convex

```
Convex (lồi):                    Non-convex (không lồi):

    f(x)                             f(x)
     │  ╲    ╱                        │    ╱╲
     │   ╲  ╱                         │   ╱  ╲    ╱╲
     │    ╲╱                          │  ╱    ╲  ╱  ╲
     │    ●                           │ ╱      ╲╱    ╲
     │  minimum                       │╱    ●         ╲
     └──────────── x                  └──────────────── x
                                      local    local   global
                                      min      min     min

Convex: Mọi điểm trên đường thẳng nối 2 điểm trong feasible region
        đều nằm trong feasible region.
        → Chỉ có 1 minimum global.
        → Giải được bằng CVXPY.

Non-convex: Có thể có nhiều local minimum.
            → Cần thuật toán phức tạp hơn.
```

### 1.3 Quadratic Programming (QP)

QP là bài toán tối ưu lồi đặc biệt:

```
minimize    (1/2) xᵀPx + qᵀx
subject to  Gx ≤ h
            Ax = b

Trong đó:
  P là ma trận dương bán định (positive semidefinite)
  → Hàm mục tiêu là lồi
  → Giải được bằng OSQP, SCS, ECOS

Trong MPC:
  x = [x[0], x[1], ..., x[N], u[0], ..., u[N-1]]
  P = ma trận trọng số Q, R
  q = vector reference trajectory
```

---

## 2. Ma trận và Vector

### 2.1 Ký hiệu

```
Vector cột (state):          Vector hàng:
  x = [x₁]                    xᵀ = [x₁, x₂, x₃, x₄]
      [x₂]
      [x₃]
      [x₄]

Ma trận (4×4):
  A = [a₁₁  a₁₂  a₁₃  a₁₄]
      [a₂₁  a₂₂  a₂₃  a₂₄]
      [a₃₁  a₃₂  a₃₃  a₃₄]
      [a₄₁  a₄₂  a₄₃  a₄₄]

Ma trận đường chéo:
  Q = diag(q₁, q₂, q₃, q₄) = [q₁  0   0   0 ]
                                [0   q₂  0   0 ]
                                [0   0   q₃  0 ]
                                [0   0   0   q₄]
```

### 2.2 Phép toán ma trận

```
Nhân ma trận-vectors:
  y = A·x
  y₁ = a₁₁·x₁ + a₁₂·x₂ + a₁₃·x₃ + a₁₄·x₄
  y₂ = a₂₁·x₁ + a₂₂·x₂ + a₂₃·x₃ + a₂₄·x₄
  ...

Quadratic form:
  xᵀQx = q₁·x₁² + q₂·x₂² + q₃·x₃² + q₄·x₄²

  Ý nghĩa: Tổng trọng số bình phương sai số
  Nếu Q = diag(1, 1, 0.5, 0.1):
    xᵀQx = 1·x₁² + 1·x₂² + 0.5·x₃² + 0.1·x₄²
    → x₁, x₂ (vị trí) quan trọng hơn x₃ (yaw), x₄ (vận tốc)
```

---

## 3. Jacobian và Linearization

### 3.1 Jacobian là gì?

Jacobian là ma trận đạo hàm riêng, mô tả hàm phi tuyến thay đổi như thế nào quanh một điểm:

```
f: ℝⁿ → ℝᵐ

Jacobian J = [∂f₁/∂x₁  ∂f₁/∂x₂  ...  ∂f₁/∂xₙ]
             [∂f₂/∂x₁  ∂f₂/∂x₂  ...  ∂f₂/∂xₙ]
             [  ...       ...     ...    ...    ]
             [∂fₘ/∂x₁  ∂fₘ/∂x₂  ...  ∂fₘ/∂xₙ]

Ví dụ: f(x,y) = [x² + y]
                 [x·y  ]

J = [2x  1]    Tại (1,2): J = [2  1]
    [y   x]                   [2  1]
```

### 3.2 Linearization trong MPC

```
Mô hình phi tuyến: x[t+1] = f(x[t], u[t])

Linearize quanh (x₀, u₀):
  x[t+1] ≈ f(x₀, u₀) + A·(x[t] - x₀) + B·(u[t] - u₀)

Trong đó:
  A = ∂f/∂x |_(x₀,u₀)    (Jacobian theo state)
  B = ∂f/∂u |_(x₀,u₀)    (Jacobian theo control)
  c = f(x₀, u₀) - A·x₀ - B·u₀   (hằng số)

Viết gọn:
  x[t+1] = A·x[t] + B·u[t] + c

→ Bài toán tuyến tính, giải bằng QP!
```

---

## 4. Trigonometry cơ bản

### 4.1 Radians vs Degrees

```
0°     = 0 rad
45°    = π/4 ≈ 0.785 rad
90°    = π/2 ≈ 1.571 rad
180°   = π   ≈ 3.142 rad
360°   = 2π  ≈ 6.283 rad

Chuyển đổi:
  rad = deg × π / 180
  deg = rad × 180 / π
```

### 4.2 sin, cos, tan, atan2

```
Trong unit circle:
                y
                ▲
                │    ● (cos θ, sin θ)
                │   /|
                │  / |
                │ /  | sin θ
                │/ θ |
    ────────────●────┼──────────▶ x
                │    cos θ

cos(0) = 1, sin(0) = 0      → hướng phải
cos(π/2) = 0, sin(π/2) = 1  → hướng lên
cos(π) = -1, sin(π) = 0     → hướng trái

atan2(y, x): tính góc từ tọa độ
  atan2(0, 1) = 0           → phải
  atan2(1, 0) = π/2         → lên
  atan2(0, -1) = π          → trái
  atan2(-1, 0) = -π/2       → xuống

  Ưu điểm hơn atan(y/x): xử lý được x=0 và xác định đúng góc phần tư
```

---

## 5. Coordinate Systems

### 5.1 World Frame vs Robot Frame

```
World Frame (tọa độ thế giới):
  y
  ▲
  │
  │    ● robot
  │   /│
  │  / │ yaw = π/4
  │ /  │
  │/θ  │
  ●────┼──────────▶ x
  origin

Robot Frame (tọa độ robot):
  y_r
  ▲
  │
  │    → forward (x_r)
  │
  ●──────────▶ x_r
  robot center

Chuyển đổi:
  x_world = x_robot·cos(yaw) - y_robot·sin(yaw) + x_robot_center
  y_world = x_robot·sin(yaw) + y_robot·cos(yaw) + y_robot_center
```

---

## 6. Time Series và Interpolation

### 6.1 Discrete Time

```
Continuous:                    Discrete (sampled):
  x(t)                           x[0], x[1], x[2], ...
  │  ╱╲                          │  ●     ●     ●
  │ ╱  ╲                         │
  │╱    ╲                        │
  ●      ╲                       ●
  └──────── t                    └──────── t
  t ∈ ℝ                          t = 0, dt, 2·dt, ...

dt = 0.1s → 10 samples/second
```

### 6.2 Interpolation (Nội suy)

```
Cho: points = [(0,0), (1,1), (2,0)]

Linear interpolation tại t=0.5:
  P(0.5) = P(0)·(1-0.5) + P(1)·0.5 = 0·0.5 + 1·0.5 = 0.5

Spline interpolation (mượt hơn):
  Dùng hàm bậc 3 nối các điểm
  → Đường cong mượt, không có góc nhọn
```

---

## 7. References

1. **Boyd, S., Vandenberghe, L.** — *Convex Optimization*. Cambridge University Press, 2004.
   https://web.stanford.edu/~boyd/cvxbook/

2. **Strang, G.** — *Introduction to Linear Algebra*. Wellesley-Cambridge Press.
   https://math.mit.edu/~gs/ila/

3. **Nocedal, J., Wright, S.J.** — *Numerical Optimization*. Springer, 2006.
   https://www.springer.com/gp/book/9780387303031

4. **CVXPY Tutorial** — https://www.cvxpy.org/tutorial/

5. **SciPy Interpolation** — https://docs.scipy.org/doc/scipy/reference/interpolate.html
