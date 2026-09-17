"""Batch experiment runner: run multiple scenarios × configs."""

import json
from pathlib import Path

import pandas as pd
import yaml

from simulator.environment import Environment
from simulator.scenarios import load_scenario
from simulator.run_simulation import run_sim
from controller.iterative_mpc import IterativeMPC
from planner.astar import astar, load_map, path_to_cells
from planner.trajectory import path_to_trajectory, smooth_trajectory
from .metrics import compute_metrics


def _run_single(
    scenario_path: str,
    scenario_name: str,
    mpc_config: dict,
    config_label: str = "default",
) -> dict:
    """Run a single scenario + config combination."""
    scenario = load_scenario(scenario_path, scenario_name)
    env = Environment(scenario["map"])

    # Plan path
    m = load_map(scenario["map"])
    path_cells = astar(m["grid"], scenario["start"], scenario["goal"])
    if path_cells is None:
        return {
            "scenario": scenario_name,
            "config": config_label,
            "error": "no_path_found",
        }

    path_xy = path_to_cells(path_cells, m["resolution"])
    traj = path_to_trajectory(path_xy, dt=mpc_config.get("dt", 0.1), target_speed=1.0)
    traj = smooth_trajectory(traj, window=5)

    mpc = IterativeMPC(mpc_config)
    log, summary = run_sim(env, mpc, traj, dt=mpc_config.get("dt", 0.1))

    metrics = compute_metrics(log, traj, dt=mpc_config.get("dt", 0.1))
    metrics["scenario"] = scenario_name
    metrics["config"] = config_label
    metrics["goal_reached"] = summary["goal_reached"]
    metrics["total_steps"] = summary["total_steps"]

    return metrics


def run_batch(
    scenarios_path: str = "configs/scenarios.yaml",
    configs_path: str = "configs/mpc_tuning.yaml",
    save_path: str | None = None,
) -> pd.DataFrame:
    """Run batch experiments across all scenarios and config variants.

    Args:
        scenarios_path: Path to scenarios YAML.
        configs_path: Path to MPC tuning YAML.
        save_path: Optional path to save results CSV.

    Returns:
        DataFrame with one row per (scenario, config) combination.
    """
    with open(scenarios_path, "r") as f:
        scenarios = yaml.safe_load(f)
    with open(configs_path, "r") as f:
        configs = yaml.safe_load(f)

    results = []
    for scenario_name in scenarios:
        # Baseline config
        with open(scenarios[scenario_name]["mpc_config"], "r") as f:
            baseline_cfg = yaml.safe_load(f)
        result = _run_single(scenarios_path, scenario_name, baseline_cfg, "baseline")
        results.append(result)

        # Variant configs
        for variant_name, variant_cfg in configs.get("variants", {}).items():
            result = _run_single(scenarios_path, scenario_name, variant_cfg, variant_name)
            results.append(result)

    df = pd.DataFrame(results)

    if save_path:
        Path(save_path).parent.mkdir(parents=True, exist_ok=True)
        df.to_csv(save_path, index=False)

    return df
