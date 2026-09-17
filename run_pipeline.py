#!/usr/bin/env python3
"""
run_pipeline.py — End-to-end MPC trajectory tracking pipeline.

Usage:
    python run_pipeline.py --scenario basic_circle
    python run_pipeline.py --scenario obstacles --save-log
    python run_pipeline.py --batch                     # Run all scenarios × all configs
    python run_pipeline.py --batch --save-results      # Save batch results to CSV

Pipeline: map → A* → trajectory → MPC → simulation → metrics
"""

import argparse
import json
import sys
import time
from pathlib import Path

import yaml
import numpy as np
import pandas as pd

from planner.astar import astar, load_map, path_to_cells
from planner.trajectory import path_to_trajectory, smooth_trajectory
from controller.iterative_mpc import IterativeMPC
from simulator.environment import Environment
from simulator.scenarios import load_scenario
from simulator.run_simulation import run_sim
from evaluation.metrics import compute_metrics
from evaluation.plots import plot_trajectory
from evaluation.batch_runner import run_batch


def run_single_scenario(scenario_name: str, save_log: bool = False, save_plot: bool = False) -> dict:
    """Run a single scenario end-to-end."""
    print(f"\n{'='*60}")
    print(f"  MPC Pipeline — Scenario: {scenario_name}")
    print(f"{'='*60}")

    # 1. Load scenario
    scenario = load_scenario("configs/scenarios.yaml", scenario_name)
    print(f"  Map: {scenario['map']}")
    print(f"  Start: {scenario['start']} → Goal: {scenario['goal']}")

    # 2. Load MPC config
    with open(scenario["mpc_config"], "r") as f:
        mpc_config = yaml.safe_load(f)

    # 3. Load map
    m = load_map(scenario["map"])
    env = Environment(scenario["map"])
    print(f"  Grid: {m['height']}x{m['width']}, obstacles: {int(np.sum(m['grid']))}")

    # 4. Path planning (A*)
    t0 = time.time()
    path_cells = astar(m["grid"], scenario["start"], scenario["goal"])
    t_astar = time.time() - t0

    if path_cells is None:
        print("  ERROR: No path found!")
        return {"scenario": scenario_name, "error": "no_path_found"}

    print(f"  A* path: {len(path_cells)} cells ({t_astar:.3f}s)")

    # 5. Trajectory generation
    path_xy = path_to_cells(path_cells, m["resolution"])
    traj = path_to_trajectory(path_xy, dt=mpc_config["dt"], target_speed=1.0)
    traj = smooth_trajectory(traj, window=5)
    print(f"  Trajectory: {len(traj)} points, duration: {traj['t'].iloc[-1]:.1f}s")

    # 6. MPC simulation
    mpc = IterativeMPC(mpc_config)
    t0 = time.time()
    log, summary = run_sim(env, mpc, traj, dt=mpc_config["dt"])
    t_sim = time.time() - t0
    print(f"  Simulation: {summary['total_steps']} steps ({t_sim:.2f}s)")

    # 7. Metrics
    metrics = compute_metrics(log, traj, dt=mpc_config["dt"])
    print(f"\n  {'Metric':<30} {'Value':>10}")
    print(f"  {'-'*40}")
    for k, v in metrics.items():
        print(f"  {k:<30} {v:>10}")

    print(f"\n  Goal reached: {summary['goal_reached']}")
    print(f"  Collisions:   {summary['collision_count']}")

    # 8. Save outputs
    if save_log:
        log_path = f"data/logs/{scenario_name}_log.csv"
        Path(log_path).parent.mkdir(parents=True, exist_ok=True)
        log.to_csv(log_path, index=False)
        print(f"  Log saved: {log_path}")

    if save_plot:
        plot_path = f"data/results/{scenario_name}_trajectory.png"
        plot_trajectory(log, traj, save_path=plot_path,
                       title=f"{scenario_name} — MPC Tracking")
        print(f"  Plot saved: {plot_path}")

    metrics["scenario"] = scenario_name
    metrics["goal_reached"] = summary["goal_reached"]
    return metrics


def run_batch_mode(save_results: bool = False) -> pd.DataFrame:
    """Run all scenarios × all config variants."""
    print(f"\n{'='*60}")
    print(f"  MPC Pipeline — Batch Mode")
    print(f"{'='*60}")

    results = run_batch(
        scenarios_path="configs/scenarios.yaml",
        configs_path="configs/mpc_tuning.yaml",
    )

    print(f"\n  Results ({len(results)} runs):\n")
    display_cols = ["scenario", "config", "position_rmse", "yaw_rmse",
                    "collision_count", "min_obstacle_distance", "goal_reached"]
    available = [c for c in display_cols if c in results.columns]
    print(results[available].to_string(index=False))

    if save_results:
        out_path = "data/results/batch_results.csv"
        Path(out_path).parent.mkdir(parents=True, exist_ok=True)
        results.to_csv(out_path, index=False)
        print(f"\n  Results saved: {out_path}")

    return results


def main():
    parser = argparse.ArgumentParser(description="MPC Trajectory Tracking Pipeline")
    parser.add_argument("--scenario", type=str, default="basic_circle",
                        help="Scenario name (default: basic_circle)")
    parser.add_argument("--save-log", action="store_true",
                        help="Save simulation log to CSV")
    parser.add_argument("--save-plot", action="store_true",
                        help="Save trajectory plot")
    parser.add_argument("--batch", action="store_true",
                        help="Run all scenarios × all configs")
    parser.add_argument("--save-results", action="store_true",
                        help="Save batch results to CSV")
    args = parser.parse_args()

    if args.batch:
        run_batch_mode(save_results=args.save_results)
    else:
        metrics = run_single_scenario(
            args.scenario,
            save_log=args.save_log,
            save_plot=args.save_plot,
        )
        if "error" in metrics:
            sys.exit(1)

    print(f"\n{'='*60}")
    print("  Pipeline complete.")
    print(f"{'='*60}\n")


if __name__ == "__main__":
    main()
