"""Evaluation metrics for trajectory tracking and safety."""

import numpy as np
import pandas as pd


def position_rmse(log: pd.DataFrame, ref: pd.DataFrame) -> float:
    """Compute position RMSE between simulation log and reference trajectory.

    Aligns by index (shortest length).

    Args:
        log: Simulation log with 'x', 'y' columns.
        ref: Reference trajectory with 'x_ref', 'y_ref' columns.

    Returns:
        RMSE in meters.
    """
    n = min(len(log), len(ref))
    dx = log["x"].values[:n] - ref["x_ref"].values[:n]
    dy = log["y"].values[:n] - ref["y_ref"].values[:n]
    return float(np.sqrt(np.mean(dx ** 2 + dy ** 2)))


def yaw_rmse(log: pd.DataFrame, ref: pd.DataFrame) -> float:
    """Compute yaw (heading) RMSE."""
    n = min(len(log), len(ref))
    dyaw = log["yaw"].values[:n] - ref["yaw_ref"].values[:n]
    # Normalize to [-pi, pi]
    dyaw = (dyaw + np.pi) % (2 * np.pi) - np.pi
    return float(np.sqrt(np.mean(dyaw ** 2)))


def completion_time(log: pd.DataFrame, dt: float = 0.1) -> float:
    """Return total simulation time."""
    return float(log["t"].iloc[-1]) if len(log) > 0 else 0.0


def collision_count(log: pd.DataFrame) -> int:
    """Count total collision steps."""
    return int(log["collision"].sum())


def min_obstacle_distance(log: pd.DataFrame) -> float:
    """Minimum obstacle distance across entire log."""
    if len(log) == 0 or "min_obstacle_distance" not in log.columns:
        return float("inf")
    return float(log["min_obstacle_distance"].min())


def control_smoothness(log: pd.DataFrame) -> dict[str, float]:
    """Compute control signal smoothness (total variation).

    Returns:
        dict with 'steering_variation' and 'acceleration_variation'.
    """
    if len(log) < 2:
        return {"steering_variation": 0.0, "acceleration_variation": 0.0}

    steer_diff = np.abs(np.diff(log["steer"].values))
    accel_diff = np.abs(np.diff(log["accel"].values))

    return {
        "steering_variation": round(float(np.mean(steer_diff)), 6),
        "acceleration_variation": round(float(np.mean(accel_diff)), 6),
    }


def compute_metrics(log: pd.DataFrame, ref: pd.DataFrame, dt: float = 0.1) -> dict:
    """Compute all evaluation metrics.

    Args:
        log: Simulation log DataFrame.
        ref: Reference trajectory DataFrame.
        dt: Time step.

    Returns:
        dict of metric name → value.
    """
    pos_rmse = position_rmse(log, ref)
    yaw_rmse_val = yaw_rmse(log, ref)
    comp_time = completion_time(log, dt)
    coll = collision_count(log)
    min_dist = min_obstacle_distance(log)
    smooth = control_smoothness(log)

    return {
        "position_rmse": round(pos_rmse, 4),
        "yaw_rmse": round(yaw_rmse_val, 4),
        "completion_time": round(comp_time, 2),
        "collision_count": coll,
        "min_obstacle_distance": round(min_dist, 4),
        "steering_smoothness": smooth["steering_variation"],
        "acceleration_smoothness": smooth["acceleration_variation"],
    }
