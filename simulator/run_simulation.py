"""Main simulation loop: connect planner → controller → environment."""

import numpy as np
import pandas as pd

from .environment import Environment
from controller.iterative_mpc import IterativeMPC


def run_sim(
    env: Environment,
    controller: IterativeMPC,
    ref_trajectory: pd.DataFrame,
    start_state: np.ndarray | None = None,
    dt: float = 0.1,
    max_steps: int = 1000,
    collision_radius: float = 0.15,
) -> tuple[pd.DataFrame, dict]:
    """Run a full simulation.

    Args:
        env: Simulation environment with collision checking.
        controller: MPC controller.
        ref_trajectory: DataFrame with columns [t, x_ref, y_ref, yaw_ref, v_ref].
        start_state: Initial [x, y, yaw, v]. If None, derived from ref_trajectory.
        dt: Time step (should match MPC dt).
        max_steps: Maximum simulation steps.
        collision_radius: Radius for collision detection.

    Returns:
        (log_df, summary) where log_df is a DataFrame of simulation states
        and summary contains key metrics.
    """
    if start_state is None:
        start_state = np.array([
            ref_trajectory["x_ref"].iloc[0],
            ref_trajectory["y_ref"].iloc[0],
            ref_trajectory["yaw_ref"].iloc[0],
            ref_trajectory["v_ref"].iloc[0],
        ])

    state = start_state.copy()
    ref_np = ref_trajectory[["x_ref", "y_ref", "yaw_ref", "v_ref"]].values

    log_entries = []
    collision_count = 0
    goal_reached = False
    goal = ref_np[-1, :2]

    for step in range(max_steps):
        # Get reference window for MPC
        ref_window = ref_np[step:step + controller.N + 1]
        if len(ref_window) == 0:
            ref_window = ref_np[-(controller.N + 1):]

        # Solve MPC
        control, status = controller.solve(state, ref_window)

        # Clip control to bounds
        steer = float(np.clip(control[0], -controller.max_steer, controller.max_steer))
        accel = float(np.clip(control[1], -controller.max_accel, controller.max_accel))

        # Step environment
        collision = env.check_collision(state[0], state[1], collision_radius)
        min_dist = env.min_obstacle_distance(state[0], state[1])

        # Log
        log_entries.append({
            "t": round(step * dt, 4),
            "x": round(float(state[0]), 4),
            "y": round(float(state[1]), 4),
            "yaw": round(float(state[2]), 4),
            "v": round(float(state[3]), 4),
            "steer": round(steer, 4),
            "accel": round(accel, 4),
            "collision": int(collision),
            "min_obstacle_distance": round(min_dist, 4),
            "mpc_status": status,
        })

        if collision:
            collision_count += 1

        # Advance state
        control_vec = np.array([steer, accel])
        state = controller.vehicle.step(state, control_vec)

        # Check goal reached
        dist_to_goal = np.sqrt((state[0] - goal[0]) ** 2 + (state[1] - goal[1]) ** 2)
        if dist_to_goal < 0.3:
            goal_reached = True
            break

    log_df = pd.DataFrame(log_entries)

    summary = {
        "total_steps": len(log_df),
        "total_time": round(len(log_df) * dt, 2),
        "collision_count": collision_count,
        "goal_reached": goal_reached,
        "final_x": round(float(state[0]), 4),
        "final_y": round(float(state[1]), 4),
        "min_obstacle_distance_all": round(float(log_df["min_obstacle_distance"].min()), 4)
        if len(log_df) > 0 else float("inf"),
    }

    return log_df, summary
