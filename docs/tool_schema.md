# MCP Tool Schema

Mô tả chi tiết input/output của mỗi tool trong MCP Server.

## list_scenarios()

**Input:** Không có

**Output:**
```json
["basic_circle", "corridor", "obstacles"]
```

---

## run_mpc_sim(config, scenario_name, save_log)

**Input:**
```json
{
  "config": {
    "dt": 0.1,
    "horizon": 10,
    "Q": [1.0, 1.0, 0.5, 0.1],
    "R": [0.1, 0.1],
    "max_steer": 0.5,
    "max_accel": 2.0,
    "max_speed": 3.0,
    "vehicle_length": 0.3,
    "num_iterations": 3
  },
  "scenario_name": "basic_circle",
  "save_log": true
}
```

**Output (success):**
```json
{
  "status": "success",
  "log_path": "data/logs/basic_circle_agent_log.csv",
  "metrics": {
    "position_rmse": 0.05,
    "yaw_rmse": 0.03,
    "completion_time": 12.5,
    "collision_count": 0,
    "min_obstacle_distance": 5.0,
    "steering_smoothness": 0.02,
    "acceleration_smoothness": 0.15
  },
  "summary": {
    "total_steps": 125,
    "total_time": 12.5,
    "collision_count": 0,
    "goal_reached": true
  }
}
```

**Output (rejected):**
```json
{
  "status": "rejected",
  "error": "horizon=-5 must be positive integer; Q[1] = -1.0 must be non-negative"
}
```

---

## evaluate_experiment(log_path)

**Input:**
```json
{"log_path": "data/logs/basic_circle_log.csv"}
```

**Output:**
```json
{
  "status": "success",
  "collision_count": 0,
  "min_obstacle_distance": 5.0,
  "total_steps": 125,
  "final_x": 8.5,
  "final_y": 8.5
}
```

---

## get_safe_config_range()

**Input:** Không có

**Output:**
```json
{
  "horizon": [1, 30],
  "dt": [0.01, 1.0],
  "Q": [0.0, 100.0],
  "R": [0.0, 100.0],
  "max_steer": [0.01, 1.57],
  "max_accel": [0.1, 10.0],
  "max_speed": [0.1, 10.0],
  "vehicle_length": [0.05, 5.0],
  "num_iterations": [1, 10]
}
```

---

## validate_config(config)

**Input:**
```json
{"config": {"horizon": 10, "Q": [1,1,0.5,0.1], "R": [0.1,0.1]}}
```

**Output (valid):**
```json
[true, ""]
```

**Output (invalid):**
```json
[false, "horizon=-5 must be positive integer"]
```
