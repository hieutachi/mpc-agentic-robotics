# LEARNING_PATH — Đường dẫn tài liệu học tập

Hướng dẫn học theo tuần cho nhóm sinh viên 5 người trong 3 tháng.

---

## Tuần 1–2: Python, CVXPY, mô hình xe

**Mục tiêu:** Hiểu nền tảng Python tối ưu, CVXPY, và mô hình động học xe.

### Tài liệu bắt buộc

1. **CVXPY Documentation** — đọc phần "Introduction" và "Tutorial"
   - https://www.cvxpy.org/
2. **CVXPY Examples** — xem ví dụ MPC
   - https://www.cvxpy.org/examples/
3. **mpc_python Repository** — đọc source code `iterative_mpc.py`
   - https://github.com/mcarfagno/mpc_python
4. **Rawlings et al.** — Chapter 1 (Overview of MPC)
   - https://sites.engineering.ucsb.edu/~jbraw/mpc/

### Notebook tương ứng

- `notebooks/02_mpc_basics.ipynb` — Mô hình xe + MPC cơ bản

### Bài tập

- [ ] Cài CVXPY, chạy ví dụ "Least squares" trong docs
- [ ] Chạy notebook 02, thay đổi horizon (5, 10, 20), quan sát RMSE
- [ ] Thay đổi Q = [2, 2, 1, 0.2] vs Q = [0.5, 0.5, 0.2, 0.05], so sánh kết quả
- [ ] Viết function `step()` cho vehicle model không dùng class

---

## Tuần 3–4: A*, Trajectory, MPC bám quỹ đạo

**Mục tiêu:** Kết nối path planning với MPC control.

### Tài liệu bắt buộc

1. **A* Algorithm** — Hart, Nilsson, Raphael (1968)
   - https://doi.org/10.1109/TSSC.1968.300136
2. **PythonRobotics** — phần Path Planning
   - https://atsushisakai.github.io/PythonRobotics/modules/5_path_planning/grid_base_search/grid_base_search.html
3. **LaValle, Planning Algorithms** — Chapter 2 (Basic Concepts)
   - https://lavalle.pl/planning/

### Notebook tương ứng

- `notebooks/01_path_planning.ipynb` — A* và trajectory
- `notebooks/03_simulation.ipynb` — Simulation end-to-end

### Bài tập

- [ ] Tạo grid map mới (ví dụ: hình chữ L), chạy A*
- [ ] Thử `allow_diagonal=False`, so sánh path dài hơn bao nhiêu
- [ ] Thay đổi `target_speed` trong trajectory (0.5, 1.0, 2.0), chạy MPC
- [ ] Thêm obstacle mới vào map, kiểm tra A* tìm đường khác

---

## Tuần 5–6: Simulation, Metrics, Batch Experiments

**Mục tiêu:** Đánh giá có hệ thống nhiều cấu hình MPC.

### Tài liệu bắt buộc

1. **PyBullet Quickstart** (tham khảo, không bắt buộc cài)
   - https://pybullet.org/wordpress/

### Notebook tương ứng

- `notebooks/03_simulation.ipynb` — Chạy simulation
- `notebooks/04_evaluation.ipynb` — Metrics và batch comparison

### Bài tập

- [ ] Chạy `python run_pipeline.py --batch --save-results`
- [ ] Phân tích bảng kết quả: config nào tốt nhất? Tại sao?
- [ ] Thêm 1 config variant mới vào `configs/mpc_tuning.yaml`
- [ ] Vẽ biểu đồ so sánh RMSE giữa các config

---

## Tuần 7–8: MCP, AI Agent

**Mục tiêu:** Hiểu cách AI Agent tự động tối ưu cấu hình.

### Tài liệu bắt buộc

1. **Model Context Protocol** — Introduction
   - https://modelcontextprotocol.io/
2. **Anthropic: Introduction to MCP**
   - https://www.anthropic.com/news/model-context-protocol
3. **Hugging Face Agents Course**
   - https://huggingface.co/learn/agents-course
4. **Coursera: AI Agents with MCP Specialization**
   - https://www.coursera.org/specializations/ai-agents-model-context-protocol

### Notebook tương ứng

- `notebooks/05_agent_demo.ipynb` — Demo AI Agent

### Bài tập

- [ ] Chạy notebook 05, hiểu flow: goal → agent → tool → metrics
- [ ] Thêm 1 tool mới vào MCP server (ví dụ: `get_scenario_info`)
- [ ] Thử goal khác: ưu tiên `collision_target = 0` thay vì RMSE thấp
- [ ] Viết strategy mới trong `agent/prompts.py` (ví dụ: random search có giới hạn)

---

## Tài liệu tham khảo đầy đủ

### MPC và Control
- mpc_python Repository: https://github.com/mcarfagno/mpc_python
- CVXPY Documentation: https://www.cvxpy.org/
- CVXPY Examples: https://www.cvxpy.org/examples/
- pyMPC: http://www.marcoforgione.it/pyMPC/
- Rawlings, Mayne, Diehl — MPC: Theory, Computation, and Design: https://sites.engineering.ucsb.edu/~jbraw/mpc/

### Robot mô phỏng
- MuJoCo Documentation: https://mujoco.readthedocs.io/
- MuJoCo GitHub: https://github.com/google-deepmind/mujoco
- MuSHR: https://mushr.io/
- PyBullet: https://pybullet.org/wordpress/

### Path Planning
- A* (Hart, Nilsson, Raphael, 1968): https://doi.org/10.1109/TSSC.1968.300136
- PythonRobotics: https://github.com/AtsushiSakai/PythonRobotics
- PythonRobotics A*: https://atsushisakai.github.io/PythonRobotics/modules/5_path_planning/grid_base_search/grid_base_search.html
- LaValle, Planning Algorithms: https://lavalle.pl/planning/

### AI Agents và MCP
- Model Context Protocol: https://modelcontextprotocol.io/
- MCP GitHub: https://github.com/modelcontextprotocol
- Anthropic MCP: https://www.anthropic.com/news/model-context-protocol
- Hugging Face Agents Course: https://huggingface.co/learn/agents-course
- Coursera AI Agents with MCP: https://www.coursera.org/specializations/ai-agents-model-context-protocol
- IBM Build AI Agents using MCP: https://www.coursera.org/learn/build-ai-agents-using-mcp
- Kaggle 5-Day AI Agents: https://www.kaggle.com/learn-guide/5-day-agents
