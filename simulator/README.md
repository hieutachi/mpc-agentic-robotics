# simulator — Simulation Module

Module mô phỏng 2D: môi trường grid map, quản lý scenario, và vòng lặp simulation.

## Files

- `environment.py` — Grid map với collision checking và distance computation.
- `scenarios.py` — Load scenario từ YAML.
- `run_simulation.py` — Vòng lặp chính: MPC → step → log.

## Usage

```python
from simulator import Environment, load_scenario, run_sim
from controller import IterativeMPC
from planner import astar, load_map, path_to_cells, path_to_trajectory
import yaml

# Load scenario
scenario = load_scenario("configs/scenarios.yaml", "basic_circle")
env = Environment(scenario["map"])

# Plan path
m = load_map(scenario["map"])
path_cells = astar(m["grid"], scenario["start"], scenario["goal"])
path_xy = path_to_cells(path_cells, m["resolution"])
traj = path_to_trajectory(path_xy, dt=0.1, target_speed=1.0)

# Load MPC config
with open(scenario["mpc_config"]) as f:
    mpc_config = yaml.safe_load(f)
mpc = IterativeMPC(mpc_config)

# Run simulation
log, summary = run_sim(env, mpc, traj)
print(summary)
```

## Run Tests

```bash
pytest simulator/tests/ -v
```
