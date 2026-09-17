"""Tests for trajectory generation and smoothing."""

import numpy as np
import pytest

from planner.trajectory import path_to_trajectory, smooth_trajectory


class TestPathToTrajectory:
    def test_straight_line(self):
        path = [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0)]
        traj = path_to_trajectory(path, dt=0.1, target_speed=1.0)
        assert "t" in traj.columns
        assert "x_ref" in traj.columns
        assert "y_ref" in traj.columns
        assert "yaw_ref" in traj.columns
        assert "v_ref" in traj.columns
        assert len(traj) > 0

    def test_yaw_direction(self):
        # Horizontal path → yaw ≈ 0
        path = [(0, 0), (1, 0), (2, 0)]
        traj = path_to_trajectory(path, dt=0.1, target_speed=1.0)
        # Most yaw values should be near 0
        assert abs(traj["yaw_ref"].iloc[0]) < 0.1

    def test_constant_speed(self):
        path = [(0, 0), (1, 0), (2, 0), (3, 0)]
        traj = path_to_trajectory(path, dt=0.1, target_speed=2.0)
        assert all(traj["v_ref"] == 2.0)

    def test_too_short_path(self):
        with pytest.raises(ValueError):
            path_to_trajectory([(0, 0)], dt=0.1)


class TestSmoothTrajectory:
    def test_preserves_length(self):
        path = [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0), (5, 0)]
        traj = path_to_trajectory(path, dt=0.1, target_speed=1.0)
        smoothed = smooth_trajectory(traj, window=5)
        assert len(smoothed) == len(traj)

    def test_preserves_endpoints(self):
        path = [(0, 0), (1, 0.1), (2, -0.1), (3, 0.05), (4, 0), (5, 0)]
        traj = path_to_trajectory(path, dt=0.1, target_speed=1.0)
        smoothed = smooth_trajectory(traj, window=5)
        assert smoothed["x_ref"].iloc[0] == traj["x_ref"].iloc[0]
        assert smoothed["x_ref"].iloc[-1] == traj["x_ref"].iloc[-1]
