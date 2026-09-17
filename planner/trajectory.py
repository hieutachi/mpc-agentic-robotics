"""Trajectory generation and smoothing from a discrete path."""

import numpy as np
import pandas as pd
from scipy.interpolate import interp1d


def path_to_trajectory(
    path: list[tuple[float, float]],
    dt: float = 0.1,
    target_speed: float = 1.0,
) -> pd.DataFrame:
    """Convert a discrete path to a time-parameterized trajectory.

    Computes heading (yaw) from consecutive points and assigns constant speed.

    Args:
        path: List of (x, y) waypoints.
        dt: Time step in seconds.
        target_speed: Desired speed along the path (m/s).

    Returns:
        DataFrame with columns: t, x_ref, y_ref, yaw_ref, v_ref.
    """
    pts = np.array(path, dtype=float)
    if len(pts) < 2:
        raise ValueError("Path must have at least 2 points")

    # Compute cumulative arc length
    diffs = np.diff(pts, axis=0)
    seg_lengths = np.linalg.norm(diffs, axis=1)
    cum_length = np.concatenate([[0], np.cumsum(seg_lengths)])
    total_length = cum_length[-1]

    if total_length < 1e-6:
        raise ValueError("Path has zero length")

    # Interpolate at uniform time steps
    total_time = total_length / target_speed
    n_steps = max(int(total_time / dt), 2)
    t_uniform = np.linspace(0, total_time, n_steps)
    s_uniform = t_uniform * target_speed  # arc length at each time

    # Clamp to valid range
    s_uniform = np.clip(s_uniform, 0, total_length - 1e-10)

    interp_x = interp1d(cum_length, pts[:, 0], kind="linear")
    interp_y = interp1d(cum_length, pts[:, 1], kind="linear")

    x = interp_x(s_uniform)
    y = interp_y(s_uniform)

    # Compute yaw from finite differences
    yaw = np.zeros(n_steps)
    for i in range(n_steps - 1):
        dx = x[i + 1] - x[i]
        dy = y[i + 1] - y[i]
        yaw[i] = np.arctan2(dy, dx)
    yaw[-1] = yaw[-2] if n_steps > 1 else 0.0

    return pd.DataFrame({
        "t": np.round(t_uniform, 4),
        "x_ref": np.round(x, 4),
        "y_ref": np.round(y, 4),
        "yaw_ref": np.round(yaw, 4),
        "v_ref": target_speed,
    })


def smooth_trajectory(
    traj: pd.DataFrame,
    window: int = 5,
    columns: tuple[str, ...] = ("x_ref", "y_ref"),
) -> pd.DataFrame:
    """Smooth trajectory coordinates using a moving average.

    Preserves the first and last points exactly.

    Args:
        traj: Input trajectory DataFrame.
        window: Moving average window size (must be odd).
        columns: Which columns to smooth.

    Returns:
        Smoothed DataFrame (copy).
    """
    result = traj.copy()
    half = window // 2

    for col in columns:
        vals = result[col].values.copy()
        smoothed = vals.copy()
        for i in range(half, len(vals) - half):
            smoothed[i] = np.mean(vals[i - half:i + half + 1])
        result[col] = np.round(smoothed, 4)

    # Recompute yaw after smoothing
    if "x_ref" in columns or "y_ref" in columns:
        x = result["x_ref"].values
        y = result["y_ref"].values
        for i in range(len(x) - 1):
            dx = x[i + 1] - x[i]
            dy = y[i + 1] - y[i]
            if abs(dx) > 1e-10 or abs(dy) > 1e-10:
                result.loc[result.index[i], "yaw_ref"] = round(np.arctan2(dy, dx), 4)
        result.loc[result.index[-1], "yaw_ref"] = result.loc[result.index[-2], "yaw_ref"]

    return result
