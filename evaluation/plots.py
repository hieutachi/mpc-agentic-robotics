"""Plotting utilities for simulation results."""

from pathlib import Path

import matplotlib
matplotlib.use("Agg")  # Non-interactive backend
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def plot_trajectory(
    log: pd.DataFrame,
    ref: pd.DataFrame,
    save_path: str | Path | None = None,
    title: str = "Trajectory Tracking",
) -> None:
    """Plot actual vs reference trajectory.

    Args:
        log: Simulation log with x, y columns.
        ref: Reference trajectory with x_ref, y_ref columns.
        save_path: If provided, save figure to this path.
        title: Plot title.
    """
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    # Trajectory plot
    ax = axes[0]
    ax.plot(ref["x_ref"], ref["y_ref"], "b--", label="Reference", linewidth=2)
    ax.plot(log["x"], log["y"], "r-", label="Actual", linewidth=1.5)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title(title)
    ax.legend()
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)

    # Error over time
    ax = axes[1]
    n = min(len(log), len(ref))
    t = log["t"].values[:n]
    err = np.sqrt(
        (log["x"].values[:n] - ref["x_ref"].values[:n]) ** 2
        + (log["y"].values[:n] - ref["y_ref"].values[:n]) ** 2
    )
    ax.plot(t, err, "g-", linewidth=1.5)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Position Error (m)")
    ax.set_title("Tracking Error over Time")
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_path:
        Path(save_path).parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(save_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_comparison(
    results_df: pd.DataFrame,
    metric: str = "position_rmse",
    save_path: str | Path | None = None,
) -> None:
    """Bar chart comparing a metric across scenario × config combinations.

    Args:
        results_df: DataFrame from batch_runner.
        metric: Column name to plot.
        save_path: If provided, save figure.
    """
    if metric not in results_df.columns:
        raise ValueError(f"Metric '{metric}' not in results columns: {results_df.columns.tolist()}")

    fig, ax = plt.subplots(figsize=(10, 5))

    labels = [f"{row['scenario']}\n({row['config']})" for _, row in results_df.iterrows()]
    values = results_df[metric].values

    colors = plt.cm.Set2(np.linspace(0, 1, len(values)))
    bars = ax.bar(range(len(values)), values, color=colors)

    ax.set_xticks(range(len(labels)))
    ax.set_xticklabels(labels, fontsize=8, rotation=15, ha="right")
    ax.set_ylabel(metric)
    ax.set_title(f"Comparison: {metric}")
    ax.grid(True, alpha=0.3, axis="y")

    # Add value labels
    for bar, val in zip(bars, values):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.001,
                f"{val:.4f}", ha="center", va="bottom", fontsize=7)

    plt.tight_layout()
    if save_path:
        Path(save_path).parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(save_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
