"""Prompt templates for the MPC AI Agent."""

from typing import Any


def build_prompt(
    goal: dict[str, Any],
    history: list[dict[str, Any]],
    safe_ranges: dict,
) -> str:
    """Build a prompt for the agent to decide next MPC configuration.

    Args:
        goal: Target objectives, e.g. {"rmse_target": 0.5, "collision_target": 0, "max_trials": 10}.
        history: List of past trial results [{config, metrics, status}, ...].
        safe_ranges: Safe parameter ranges from get_safe_config_range().

    Returns:
        Formatted prompt string.
    """
    lines = [
        "You are an MPC configuration optimizer.",
        "",
        f"GOAL: {goal}",
        "",
        "SAFE PARAMETER RANGES:",
    ]
    for param, (lo, hi) in safe_ranges.items():
        lines.append(f"  {param}: [{lo}, {hi}]")

    if history:
        lines.append("")
        lines.append(f"HISTORY ({len(history)} trials):")
        for i, trial in enumerate(history):
            cfg = trial.get("config", {})
            metrics = trial.get("metrics", {})
            status = trial.get("status", "unknown")
            lines.append(
                f"  Trial {i+1}: status={status}, "
                f"rmse={metrics.get('position_rmse', 'N/A')}, "
                f"collisions={metrics.get('collision_count', 'N/A')}, "
                f"config={cfg}"
            )
    else:
        lines.append("")
        lines.append("HISTORY: No trials yet. Suggest an initial configuration.")

    lines.append("")
    lines.append(
        "Suggest the next MPC config as a Python dict with keys: "
        "dt, horizon, Q (list of 4), R (list of 2), "
        "max_steer, max_accel, max_speed, vehicle_length, num_iterations."
    )

    return "\n".join(lines)


def build_simple_suggestion(
    goal: dict[str, Any],
    history: list[dict[str, Any]],
    safe_ranges: dict,
) -> dict:
    """Rule-based config suggestion (no LLM needed).

    Strategy:
    - First trial: use baseline config.
    - If RMSE too high: increase horizon or Q weights.
    - If collisions: increase R (control penalty) or reduce max_speed.
    - Random perturbation for exploration.

    Returns:
        Suggested MPC config dict.
    """
    import numpy as np

    baseline = {
        "dt": 0.1,
        "horizon": 10,
        "Q": [1.0, 1.0, 0.5, 0.1],
        "R": [0.1, 0.1],
        "max_steer": 0.5,
        "max_accel": 2.0,
        "max_speed": 3.0,
        "vehicle_length": 0.3,
        "num_iterations": 3,
    }

    if not history:
        return baseline

    # Analyze last trial
    last = history[-1]
    last_metrics = last.get("metrics", {})
    last_config = last.get("config", {})

    new_config = dict(last_config)

    rmse = last_metrics.get("position_rmse", 999)
    collisions = last_metrics.get("collision_count", 0)
    rmse_target = goal.get("rmse_target", 0.5)
    collision_target = goal.get("collision_target", 0)

    # Adjust based on results
    if collisions > collision_target:
        # More conservative: increase R, reduce speed
        new_config["R"] = [r * 1.5 for r in new_config.get("R", [0.1, 0.1])]
        new_config["max_speed"] = max(0.5, new_config.get("max_speed", 3.0) * 0.8)

    if rmse > rmse_target * 1.5:
        # Increase tracking aggressiveness
        new_config["horizon"] = min(30, new_config.get("horizon", 10) + 2)
        new_config["Q"] = [q * 1.2 for q in new_config.get("Q", [1, 1, 0.5, 0.1])]

    # Add small random perturbation for exploration
    rng = np.random.default_rng(len(history))
    new_config["horizon"] = int(np.clip(
        new_config.get("horizon", 10) + rng.integers(-2, 3), 1, 30
    ))
    new_config["max_speed"] = float(np.clip(
        new_config.get("max_speed", 3.0) + rng.uniform(-0.3, 0.3), 0.5, 10.0
    ))

    return new_config
