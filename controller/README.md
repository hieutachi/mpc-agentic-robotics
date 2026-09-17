# controller — Vehicle Model & MPC Module

Module mô hình động học xe đạp (kinematic bicycle model) và bộ điều khiển Iterative MPC sử dụng CVXPY.

## Files

- `vehicle_model.py` — Mô hình động học xe, hàm `step()` và `linearize()`.
- `iterative_mpc.py` — Iterative MPC: tuyến tính hóa lặp, giải QP bằng CVXPY.

## Key Concepts

### Kinematic Bicycle Model
```
State:  [x, y, yaw, v]  — vị trí, hướng, vận tốc
Control: [steer, accel]  — góc lái, gia tốc

x_{t+1}   = x_t + v_t * cos(yaw_t) * dt
y_{t+1}   = y_t + v_t * sin(yaw_t) * dt
yaw_{t+1} = yaw_t + (v_t / L) * tan(steer_t) * dt
v_{t+1}   = v_t + accel_t * dt
```

### Iterative MPC
1. Khởi tạo quỹ đạo danh nghĩa từ reference.
2. Tuyến tính hóa động học quanh quỹ đạo danh nghĩa.
3. Giải QP (convex) bằng CVXPY.
4. Cập nhật quỹ đạo danh nghĩa, lặp lại.

## Usage

```python
from controller import IterativeMPC
import numpy as np

config = {"dt": 0.1, "horizon": 10, "Q": [1,1,0.5,0.1], "R": [0.1,0.1]}
mpc = IterativeMPC(config)

state = np.array([0.0, 0.0, 0.0, 1.0])
ref = np.tile([0.5, 0.0, 0.0, 1.0], (11, 1))
control, status = mpc.solve(state, ref)
```

## Run Tests

```bash
pytest controller/tests/ -v
```
