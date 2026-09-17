# mcp_server — MCP Tool Server Module

Module server tương thích tinh thần Model Context Protocol (MCP), cung cấp tools cho AI Agent gọi.

## Files

- `server.py` — MCPServer: register tools, dispatch calls.
- `tools.py` — Định nghĩa các tool: `list_scenarios`, `run_mpc_sim`, `evaluate_experiment`.
- `safety.py` — Validation config MPC (giới hạn an toàn).

## Tools Available

| Tool | Input | Output | Mô tả |
|---|---|---|---|
| `list_scenarios` | — | `list[str]` | Liệt kê scenario có sẵn |
| `run_mpc_sim` | config, scenario_name | dict (status, metrics) | Chạy simulation với config MPC |
| `evaluate_experiment` | log_path | dict (metrics) | Đánh giá từ log CSV |
| `get_safe_config_range` | — | dict (ranges) | Khoảng tham số an toàn |
| `validate_config` | config | (bool, str) | Kiểm tra config hợp lệ |

## Usage

```python
from mcp_server import MCPServer

server = MCPServer()

# List tools
for tool in server.list_tools():
    print(f"  {tool['name']}: {tool['description']}")

# Run simulation via tool
result = server.call("run_mpc_sim", config={
    "dt": 0.1, "horizon": 10,
    "Q": [1, 1, 0.5, 0.1], "R": [0.1, 0.1],
    "max_steer": 0.5, "max_accel": 2.0,
    "max_speed": 3.0, "vehicle_length": 0.3,
    "num_iterations": 3,
})
print(result["metrics"])
```

## Safety Validation

Mọi config MPC phải vượt qua `validate_config()` trước khi chạy:
- horizon ∈ [1, 30]
- Q, R ≥ 0
- max_steer ∈ (0, 1.57]
- max_accel ∈ (0, 10.0]
- max_speed ∈ (0, 10.0]

## Run Tests

```bash
pytest mcp_server/tests/ -v
```
