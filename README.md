# MPC Agentic Robotics

**Khung học Model Predictive Control cho robot di động trên Python, kết hợp AI Agent và MCP để tự động cấu hình, mô phỏng và đánh giá bài toán bám quỹ đạo, tìm đường và tránh vật cản.**

## Kết quả học tập

Sau khi hoàn thành repo này, sinh viên sẽ:

- Hiểu nguyên lý MPC và cách triển khai bằng CVXPY
- Biết cách tìm đường bằng A* và sinh trajectory tham chiếu
- Xây dựng pipeline mô phỏng robot di động end-to-end
- Đánh giá chất lượng điều khiển bằng metric có cấu trúc
- Sử dụng AI Agent kết hợp MCP để tự động tối ưu cấu hình

## Cài đặt nhanh

```bash
# 1. Clone repo
git clone <repo-url>
cd mpc-agentic-robotics

# 2. Tạo virtual environment
python -m venv venv
source venv/bin/activate  # Linux/Mac
# venv\Scripts\activate   # Windows

# 3. Cài dependencies
pip install -r requirements.txt

# 4. Chạy demo
python run_pipeline.py --scenario basic_circle --save-plot
```

## Chạy demo

```bash
# Single scenario
python run_pipeline.py --scenario basic_circle

# All scenarios × all configs
python run_pipeline.py --batch --save-results

# AI Agent
python agent/run_agent.py --scenario basic_circle --max-trials 5
```

## Cấu trúc repo

```
mpc-agentic-robotics/
├── run_pipeline.py          # Script chạy end-to-end
├── configs/                 # MPC config, scenarios
├── maps/                    # Grid map mẫu
├── planner/                 # A* + trajectory generation
├── controller/              # Vehicle model + Iterative MPC (CVXPY)
├── simulator/               # Environment + simulation loop
├── evaluation/              # Metrics + batch runner + plots
├── mcp_server/              # MCP tool server + safety validation
├── agent/                   # AI Agent loop
├── notebooks/               # 5 notebook hướng dẫn
├── docs/                    # Tài liệu chi tiết
└── data/                    # Logs, results
```

## Tài liệu

- **[LEARNING_PATH.md](LEARNING_PATH.md)** — Đường dẫn tài liệu học theo tuần (bắt đầu từ đây)
- **[ARCHITECTURE.md](ARCHITECTURE.md)** — Kiến trúc hệ thống và sơ đồ
- **[PROJECT_CHECKLIST.md](PROJECT_CHECKLIST.md)** — Checklist công việc cho nhóm

## Phân công nhóm 5 người

| Thành viên | Vai trò | Module chính |
|---|---|---|
| Member 1 | Nhóm trưởng / Tích hợp | `run_pipeline.py`, configs, docs |
| Member 2 | Path Planning | `planner/` |
| Member 3 | MPC & Control | `controller/` |
| Member 4 | Simulator & Evaluation | `simulator/`, `evaluation/` |
| Member 5 | AI Agent & MCP | `mcp_server/`, `agent/` |

## Chạy tests

```bash
pytest planner/ controller/ simulator/ evaluation/ mcp_server/ -v
```

## Yêu cầu hệ thống

- Python 3.9+
- OS: Linux, macOS, Windows
- Không cần GPU, không cần MuJoCo/PyBullet (mô phỏng 2D grid-based)
