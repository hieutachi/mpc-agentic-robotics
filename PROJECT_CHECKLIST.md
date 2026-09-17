# PROJECT CHECKLIST — MPC + A* + MCP Agent Repository

## 1. Cấu trúc repository

```
mpc-agentic-robotics/
├── README.md                      # Giới thiệu, cài đặt, demo nhanh
├── LEARNING_PATH.md               # Đường dẫn tài liệu học theo tuần
├── ARCHITECTURE.md                # Kiến trúc hệ thống + sơ đồ Mermaid
├── PROJECT_CHECKLIST.md           # File này
├── requirements.txt               # Dependencies Python
├── run_pipeline.py                # Script chạy end-to-end
├── configs/
│   ├── mpc_baseline.yaml          # Config MPC mặc định
│   ├── mpc_tuning.yaml            # Các biến thể MPC để thử nghiệm
│   └── scenarios.yaml             # Định nghĩa scenario (map, start, goal)
├── maps/
│   ├── empty_20x20.yaml           # Map trống 20x20
│   ├── corridor_30x10.yaml        # Map hành lang
│   └── obstacles_20x20.yaml       # Map có vật cản
├── planner/
│   ├── __init__.py
│   ├── astar.py                   # Thuật toán A*
│   ├── trajectory.py              # Sinh trajectory từ path + smoothing
│   ├── tests/
│   │   ├── test_astar.py
│   │   └── test_trajectory.py
│   └── README.md
├── controller/
│   ├── __init__.py
│   ├── vehicle_model.py           # Kinematic bicycle model
│   ├── iterative_mpc.py           # Iterative MPC (CVXPY)
│   ├── tests/
│   │   ├── test_vehicle_model.py
│   │   └── test_mpc.py
│   └── README.md
├── simulator/
│   ├── __init__.py
│   ├── environment.py             # Môi trường 2D, load map, vật cản
│   ├── scenarios.py               # Quản lý scenario
│   ├── run_simulation.py          # Vòng lặp simulation chính
│   ├── tests/
│   │   └── test_simulation.py
│   └── README.md
├── evaluation/
│   ├── __init__.py
│   ├── metrics.py                 # Tính metric (RMSE, collision, ...)
│   ├── batch_runner.py            # Chạy batch nhiều config
│   ├── plots.py                   # Vẽ biểu đồ kết quả
│   ├── tests/
│   │   └── test_metrics.py
│   └── README.md
├── mcp_server/
│   ├── __init__.py
│   ├── server.py                  # MCP tool server
│   ├── tools.py                   # Định nghĩa tools
│   ├── safety.py                  # Validation config an toàn
│   ├── tests/
│   │   └── test_tools.py
│   └── README.md
├── agent/
│   ├── __init__.py
│   ├── agent_loop.py              # Agent loop chính
│   ├── prompts.py                 # Prompt mẫu cho LLM
│   ├── run_agent.py               # Script chạy agent CLI
│   └── README.md
├── notebooks/
│   ├── 01_path_planning.ipynb     # A* và trajectory
│   ├── 02_mpc_basics.ipynb        # Mô hình xe + MPC
│   ├── 03_simulation.ipynb        # Simulation loop
│   ├── 04_evaluation.ipynb        # Metrics và so sánh
│   └── 05_agent_demo.ipynb        # Demo AI Agent
├── docs/
│   ├── architecture.md            # Chi tiết kiến trúc
│   ├── experiment_protocol.md     # Quy trình thí nghiệm
│   └── tool_schema.md             # Schema MCP tools
└── data/
    ├── logs/                      # Simulation logs
    └── results/                   # Kết quả batch experiments
```

**Trách nhiệm từng folder:**

| Folder | Trách nhiệm chính |
|---|---|
| `planner/` | Tìm đường A* trên grid map, chuyển path thành trajectory có (x, y, yaw, v) |
| `controller/` | Mô hình động học xe đạp, bộ điều khiển MPC dùng CVXPY |
| `simulator/` | Mô phỏng 2D, chạy vòng lặp control → state → log |
| `evaluation/` | Tính metric, chạy batch, vẽ biểu đồ so sánh |
| `mcp_server/` | Tool server tương thích MCP, validation an toàn |
| `agent/` | Agent loop gọi tool server, đề xuất config MPC |
| `configs/` | File YAML cho MPC config, scenario, map |
| `maps/` | Grid map mẫu (YAML) |
| `notebooks/` | Notebook hướng dẫn học tương tác |
| `docs/` | Tài liệu kiến trúc, quy trình thí nghiệm, schema tools |

---

## 2. Danh sách module cần xây dựng

### 2.1. planner/

| File | Hàm/class chính | Input | Output | Test |
|---|---|---|---|---|
| `astar.py` | `astar(grid, start, goal)` | grid 2D (0=free, 1=wall), start (r,c), goal (r,c) | `list[tuple]` path cells | Map nhỏ 5x5, kiểm tra path đúng |
| `astar.py` | `load_map(path)` | file YAML | grid ndarray | Load map mẫu, kiểm tra shape |
| `trajectory.py` | `path_to_trajectory(path, resolution, dt)` | list cells, resolution (m/cell), dt | DataFrame (t, x, y, yaw, v) | Kiểm tra số điểm, liên tục |
| `trajectory.py` | `smooth_trajectory(traj, window)` | DataFrame, window size | DataFrame đã smooth | Kiểm tra không thay đổi đầu/cuối |

### 2.2. controller/

| File | Hàm/class chính | Input | Output | Test |
|---|---|---|---|---|
| `vehicle_model.py` | `VehicleModel(dt, L)` | dt, wheelbase L | object | Kiểm tra step() với input cố định |
| `vehicle_model.py` | `VehicleModel.step(state, control)` | state (x,y,yaw,v), control (steer, accel) | new state | Steer=0 → đi thẳng |
| `iterative_mpc.py` | `IterativeMPC(config)` | dict config (horizon, Q, R, constraints) | object | Kiểm tra init với config mặc định |
| `iterative_mpc.py` | `solve(state, ref_trajectory)` | state hiện tại, ref trajectory | control (steer, accel), status | Control nằm trong bounds |

### 2.3. simulator/

| File | Hàm/class chính | Input | Output | Test |
|---|---|---|---|---|
| `environment.py` | `Environment(map_config)` | dict từ YAML | object với grid, obstacles | Load map, kiểm tra shape |
| `environment.py` | `check_collision(x, y)` | tọa độ xe | bool | Điểm trong obstacle → True |
| `scenarios.py` | `load_scenario(path)` | file YAML | dict scenario | Load scenario mẫu |
| `run_simulation.py` | `run_sim(env, controller, ref_traj, dt, max_steps)` | môi trường, controller, trajectory | log DataFrame, metrics dict | Chạy 10 bước, kiểm tra log có đủ cột |

### 2.4. evaluation/

| File | Hàm/class chính | Input | Output | Test |
|---|---|---|---|---|
| `metrics.py` | `compute_metrics(log, ref_traj)` | log DataFrame, ref trajectory | dict metrics | RMSE > 0, collision_count >= 0 |
| `metrics.py` | `position_rmse(log, ref)` | 2 DataFrames | float | Hai trajectory giống nhau → RMSE = 0 |
| `batch_runner.py` | `run_batch(scenarios, configs)` | list scenarios, list configs | DataFrame kết quả | Chạy 2 config, trả 2 dòng |
| `plots.py` | `plot_trajectory(log, ref, save_path)` | log, ref, path | file PNG | Tạo file PNG |

### 2.5. mcp_server/

| File | Hàm/class chính | Input | Output | Test |
|---|---|---|---|---|
| `tools.py` | `list_scenarios()` | none | list[str] | Trả list không rỗng |
| `tools.py` | `run_mpc_sim(config)` | dict config | dict {log_path, metrics} | Chạy 1 scenario, trả log |
| `tools.py` | `evaluate_experiment(log_path)` | path | dict metrics | Tính metric từ log |
| `tools.py` | `get_safe_config_range()` | none | dict ranges | Trả dict có horizon, Q, R |
| `safety.py` | `validate_config(config)` | dict | (bool, str) | Config âm → (False, reason) |
| `server.py` | `MCPServer()` | none | server object | Register tools, gọi được |

### 2.6. agent/

| File | Hàm/class chính | Input | Output | Test |
|---|---|---|---|---|
| `agent_loop.py` | `AgentLoop(server, max_trials)` | MCP server, budget | object | Init không lỗi |
| `agent_loop.py` | `run(goal)` | dict goal (rmse_target, collision_target) | dict best_config, best_metrics | Chạy 2 trials, trả kết quả |
| `prompts.py` | `build_prompt(goal, history)` | goal, lịch sử thử | str prompt | Trả string không rỗng |

---

## 3. Checklist công việc chi tiết

### 3.1. Infrastructure (ưu tiên cao)

- [ ] Tạo cây thư mục đầy đủ
- [ ] Tạo `requirements.txt` với: numpy, scipy, cvxpy, pyyaml, matplotlib, pandas, pytest, jupyter
- [ ] Tạo `configs/mpc_baseline.yaml` (horizon=10, Q=[1,1,0.5,0.1], R=[0.1, 0.1], max_steer=0.5, max_accel=2.0)
- [ ] Tạo `configs/scenarios.yaml` (3 scenario: basic_circle, corridor, obstacles)
- [ ] Tạo 3 file map YAML mẫu
- [ ] Tạo `__init__.py` cho tất cả package
- [ ] Tạo `.gitignore` (Python, __pycache__, .ipynb_checkpoints, data/logs/*, *.png)

### 3.2. README.md chính

- [ ] Giới thiệu đề tài 1 đoạn
- [ ] Yêu cầu hệ thống (Python 3.9+, pip)
- [ ] Cài đặt step-by-step (clone, pip install, verify)
- [ ] Chạy demo nhanh (`python run_pipeline.py --scenario basic_circle`)
- [ ] Link tới LEARNING_PATH.md và ARCHITECTURE.md
- [ ] Phân công nhóm 5 người (tham khảo kế hoạch)

### 3.3. LEARNING_PATH.md

- [ ] Giai đoạn Tuần 1–2: Python, CVXPY cơ bản, mô hình xe
  - Link: CVXPY docs, mpc_python repo
  - Notebook: 02_mpc_basics.ipynb
  - Bài tập: thay đổi horizon, quan sát RMSE
- [ ] Giai đoạn Tuần 3–4: A*, trajectory, MPC bám quỹ đạo
  - Link: PythonRobotics A*, LaValle Planning Algorithms
  - Notebook: 01_path_planning.ipynb
  - Bài tập: tạo map mới, chạy A*, sinh trajectory
- [ ] Giai đoạn Tuần 5–6: Simulation, metrics, batch experiments
  - Link: PyBullet docs
  - Notebook: 03_simulation.ipynb, 04_evaluation.ipynb
  - Bài tập: so sánh 3 cấu hình MPC
- [ ] Giai đoạn Tuần 7–8: MCP, AI Agent
  - Link: MCP docs, HuggingFace Agents Course, Coursera MCP
  - Notebook: 05_agent_demo.ipynb
  - Bài tập: thêm 1 tool mới cho agent
- [ ] Danh sách tài liệu tham khảo đầy đủ (từ Section 13 kế hoạch gốc)

### 3.4. ARCHITECTURE.md

- [ ] Mô tả bằng lời: 6 module + luồng dữ liệu
- [ ] Sơ đồ Mermaid flowchart (map → A* → trajectory → MPC → sim → log → metrics)
- [ ] Sơ đồ Mermaid: Agent ↔ MCP Server ↔ Tools
- [ ] Bảng interface contracts (Map, Trajectory, MPC Config, Log, Metrics)

---

## 4. Thứ tự thực thi

```
Bước 1: Infrastructure + configs + maps
    ↓
Bước 2: planner/ (A* + trajectory) + test + notebook 01
    ↓
Bước 3: controller/ (vehicle model + MPC) + test + notebook 02
    ↓
Bước 4: simulator/ (environment + run_simulation) + test + notebook 03
    ↓
Bước 5: evaluation/ (metrics + batch + plots) + test + notebook 04
    ↓
Bước 6: run_pipeline.py (tích hợp bước 1–5)
    ↓
Bước 7: mcp_server/ (server + tools + safety) + test
    ↓
Bước 8: agent/ (agent_loop + prompts) + notebook 05
    ↓
Bước 9: Tài liệu (LEARNING_PATH, ARCHITECTURE, README hoàn chỉnh)
    ↓
Bước 10: Kiểm tra end-to-end + polish
```

**Điểm tích hợp sớm:** Sau Bước 4, có thể chạy `map → A* → trajectory → MPC → sim → log` để xác nhận pipeline cơ bản hoạt động.

---

## 5. Tiêu chí hoàn thành repo

### Mức đạt yêu cầu (bắt buộc)

- [ ] `python run_pipeline.py --scenario basic_circle` chạy end-to-end, in ra metrics
- [ ] Có ≥ 3 cấu hình MPC trong `configs/` và bảng metric so sánh
- [ ] Agent gọi tool server, chạy ≥ 1 mục tiêu, trả kết quả
- [ ] `README.md` đủ để sinh viên cài đặt trong ≤ 30 phút
- [ ] `LEARNING_PATH.md` liệt kê tài liệu theo tuần, có link
- [ ] Không có lỗi import, tất cả test tối thiểu pass

### Mức tốt (nên có)

- [ ] Có ≥ 3 notebook chạy được end-to-end
- [ ] Agent có validation config và giới hạn ngân sách thử nghiệm
- [ ] Batch runner chạy được nhiều scenario × nhiều config
- [ ] Biểu đồ trajectory và error theo thời gian
- [ ] Demo script chạy ổn định, tái lập được

### Mức xuất sắc (bonus)

- [ ] Có baseline PID hoặc pure-pursuit để đối chiếu MPC
- [ ] Agent so sánh với random search có giới hạn
- [ ] Phân tích thống kê trên nhiều seed
- [ ] MCP server có schema validation đầy đủ

---

## 6. Interface Contracts (định nghĩa trước khi code)

### Map YAML
```yaml
name: "empty_20x20"
width: 20
height: 20
resolution: 0.5  # meter/cell
origin: [0, 0]
obstacles: [[5,5],[5,6],[6,5],[6,6]]  # list [row, col]
```

### Reference Trajectory CSV
```
t, x_ref, y_ref, yaw_ref, v_ref
0.0, 0.0, 0.0, 0.0, 1.0
0.1, 0.1, 0.0, 0.0, 1.0
...
```

### MPC Config YAML
```yaml
dt: 0.1
horizon: 10
Q: [1.0, 1.0, 0.5, 0.1]      # x, y, yaw, v
R: [0.1, 0.1]                  # steer, accel
max_steer: 0.5                 # rad
max_accel: 2.0                 # m/s^2
max_speed: 3.0                 # m/s
vehicle_length: 0.3            # m (wheelbase)
```

### Simulation Log CSV
```
t, x, y, yaw, v, steer, accel, collision, min_obstacle_distance
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 5.0
...
```

### Metrics JSON
```json
{
  "position_rmse": 0.05,
  "yaw_rmse": 0.03,
  "completion_time": 12.5,
  "collision_count": 0,
  "min_obstacle_distance": 0.8,
  "steering_smoothness": 0.02,
  "acceleration_smoothness": 0.15
}
```
