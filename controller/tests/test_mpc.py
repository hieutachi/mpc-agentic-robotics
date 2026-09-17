"""Tests for Iterative MPC."""

import numpy as np
import pytest

from controller.iterative_mpc import IterativeMPC


class TestIterativeMPC:
    def setup_method(self):
        self.config = {
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
        self.mpc = IterativeMPC(self.config)

    def test_init(self):
        assert self.mpc.N == 10
        assert self.mpc.dt == 0.1

    def test_solve_returns_control(self):
        state = np.array([0.0, 0.0, 0.0, 1.0])
        # Reference: straight line
        ref = np.zeros((11, 4))
        for i in range(11):
            ref[i] = [i * 0.1, 0.0, 0.0, 1.0]
        control, status = self.mpc.solve(state, ref)
        assert control.shape == (2,)
        assert abs(control[0]) <= self.config["max_steer"] + 0.01
        assert abs(control[1]) <= self.config["max_accel"] + 0.01

    def test_solve_short_reference(self):
        state = np.array([0.0, 0.0, 0.0, 1.0])
        ref = np.array([[0.0, 0.0, 0.0, 1.0], [0.1, 0.0, 0.0, 1.0]])
        control, status = self.mpc.solve(state, ref)
        assert control.shape == (2,)
