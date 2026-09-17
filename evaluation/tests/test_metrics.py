"""Tests for evaluation metrics."""

import numpy as np
import pandas as pd
import pytest

from evaluation.metrics import (
    position_rmse,
    yaw_rmse,
    collision_count,
    control_smoothness,
    compute_metrics,
)


class TestMetrics:
    def _make_log(self, n=100):
        return pd.DataFrame({
            "t": np.arange(n) * 0.1,
            "x": np.linspace(0, 1, n),
            "y": np.zeros(n),
            "yaw": np.zeros(n),
            "v": np.ones(n),
            "steer": np.zeros(n),
            "accel": np.zeros(n),
            "collision": np.zeros(n, dtype=int),
            "min_obstacle_distance": np.ones(n) * 5.0,
        })

    def _make_ref(self, n=100):
        return pd.DataFrame({
            "t": np.arange(n) * 0.1,
            "x_ref": np.linspace(0, 1, n),
            "y_ref": np.zeros(n),
            "yaw_ref": np.zeros(n),
            "v_ref": np.ones(n),
        })

    def test_position_rmse_perfect(self):
        log = self._make_log()
        ref = self._make_ref()
        assert position_rmse(log, ref) < 1e-6

    def test_position_rmse_nonzero(self):
        log = self._make_log()
        ref = self._make_ref()
        log["x"] += 0.5  # Offset
        rmse = position_rmse(log, ref)
        assert rmse > 0.4

    def test_collision_count_zero(self):
        log = self._make_log()
        assert collision_count(log) == 0

    def test_collision_count_nonzero(self):
        log = self._make_log()
        log.loc[10, "collision"] = 1
        log.loc[20, "collision"] = 1
        assert collision_count(log) == 2

    def test_smoothness_zero(self):
        log = self._make_log()
        s = control_smoothness(log)
        assert s["steering_variation"] < 1e-6

    def test_compute_metrics_returns_dict(self):
        log = self._make_log()
        ref = self._make_ref()
        m = compute_metrics(log, ref)
        assert "position_rmse" in m
        assert "yaw_rmse" in m
        assert "collision_count" in m
