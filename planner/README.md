# planner — Path Planning Module

Module tìm đường A* trên grid map 2D và sinh trajectory tham chiếu.

## Files

- `astar.py` — Thuật toán A* (4/8-connected), load map từ YAML.
- `trajectory.py` — Chuyển discrete path thành trajectory (t, x, y, yaw, v) với smoothing.

## Usage

```python
from planner.astar import astar, load_map
from planner.trajectory import path_to_trajectory, smooth_trajectory
from planner.astar import path_to_cells

# 1. Load map
m = load_map("maps/empty_20x20.yaml")
grid = m["grid"]

# 2. Find path
path_cells = astar(grid, start=(2, 2), goal=(17, 17))

# 3. Convert to world coordinates
path_xy = path_to_cells(path_cells, resolution=m["resolution"])

# 4. Generate trajectory
traj = path_to_trajectory(path_xy, dt=0.1, target_speed=1.0)

# 5. Smooth
traj_smooth = smooth_trajectory(traj, window=5)
```

## Run Tests

```bash
pytest planner/tests/ -v
```
