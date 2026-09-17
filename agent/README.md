# agent — AI Agent Module

Module AI Agent tự động tối ưu cấu hình MPC bằng cách gọi MCP tools.

## Files

- `agent_loop.py` — AgentLoop: iterative config search với goal-driven strategy.
- `prompts.py` — Prompt templates và rule-based suggestion (không cần LLM).
- `run_agent.py` — CLI script chạy agent.

## How It Works

1. Agent nhận mục tiêu (RMSE target, collision target, max trials).
2. Mỗi trial: suggest config → validate → gọi `run_mpc_sim` qua MCP → đọc metrics.
3. Cập nhật best result (ưu tiên ít collisions, sau đó RMSE thấp).
4. Dừng khi đạt goal hoặc hết budget.

## Usage

```python
from agent import AgentLoop
from mcp_server import MCPServer

server = MCPServer()
agent = AgentLoop(server=server, max_trials=10)

results = agent.run(
    goal={"rmse_target": 0.5, "collision_target": 0, "max_trials": 5},
    scenario_name="basic_circle",
)
print(results["best_metrics"])
```

## CLI

```bash
python agent/run_agent.py --scenario basic_circle --max-trials 10
python agent/run_agent.py --scenario obstacles --rmse-target 0.3 --save-results data/results/agent_run.json
```

## Strategy (Rule-based)

- **Trial 1:** Baseline config.
- **If collisions:** Tăng R, giảm max_speed.
- **If RMSE cao:** Tăng horizon, tăng Q.
- **Exploration:** Random perturbation nhỏ.
