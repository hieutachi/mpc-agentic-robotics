"""MCP Tool definitions — functions that agents can call."""

import json
from pathlib import Path
from typing import Any

import pandas as pd
import yaml

from .safety import validate_config, get_safe_config_range
from simulator.environment import Environment
from simulator.scenarios import load_scenario, list_scenarios as _list_scenarios
from simulator.run_simulation import run_sim
from controller.iterative_mpc import IterativeMPC
from planner.astar import astar, load_map, path_to_cells
from planner.trajectory import path_to_trajectory, smooth_trajectory
from evaluation.metrics import compute_metrics


def list_scenarios(scenarios_path: str = "configs/scenarios.yaml") -> list[str]:
    """List all available scenario names.

    Returns:
        List of scenario name strings.
    """
    return _list_scenarios(scenarios_path)


def run_mpc_sim(
    config: dict,
    scenario_name: str = "basic_circle",
    scenarios_path: str = "configs/scenarios.yaml",
    save_log: bool = True,
) -> dict[str, Any]:
    """Run a simulation with the given MPC configuration.

    Args:
        config: MPC configuration dict (must pass safety validation).
        scenario_name: Which scenario to run.
        scenarios_path: Path to scenarios YAML.
        save_log: Whether to save the simulation log.

    Returns:
        dict with keys: status, log_path, metrics, error.
    """
    # Validate config
    is_valid, reason = validate_config(config)
    if not is_valid:
        return {"status": "rejected", "error": reason}

    try:
        scenario = load_scenario(scenarios_path, scenario_name)
        env = Environment(scenario["map"])
        m = load_map(scenario["map"])

        path_cells = astar(m["grid"], scenario["start"], scenario["goal"])
        if path_cells is None:
            return {"status": "error", "error": "No path found"}

        path_xy = path_to_cells(path_cells, m["resolution"])
        traj = path_to_trajectory(path_xy, dt=config.get("dt", 0.1), target_speed=1.0)
        traj = smooth_trajectory(traj, window=5)

        mpc = IterativeMPC(config)
        log, summary = run_sim(env, mpc, traj, dt=config.get("dt", 0.1))
        metrics = compute_metrics(log, traj, dt=config.get("dt", 0.1))

        log_path = ""
        if save_log:
            log_path = f"data/logs/{scenario_name}_agent_log.csv"
            Path(log_path).parent.mkdir(parents=True, exist_ok=True)
            log.to_csv(log_path, index=False)

        return {
            "status": "success",
            "log_path": log_path,
            "metrics": metrics,
            "summary": summary,
        }

    except Exception as e:
        return {"status": "error", "error": str(e)}


def evaluate_experiment(log_path: str) -> dict[str, Any]:
    """Evaluate an existing simulation log.

    Args:
        log_path: Path to simulation log CSV.

    Returns:
        dict of metrics.
    """
    if not Path(log_path).exists():
        return {"status": "error", "error": f"Log file not found: {log_path}"}

    try:
        log = pd.read_csv(log_path)
        # Basic metrics from log only (no reference comparison)
        return {
            "status": "success",
            "collision_count": int(log["collision"].sum()),
            "min_obstacle_distance": float(log["min_obstacle_distance"].min()),
            "total_steps": len(log),
            "final_x": float(log["x"].iloc[-1]),
            "final_y": float(log["y"].iloc[-1]),
        }
    except Exception as e:
        return {"status": "error", "error": str(e)}
