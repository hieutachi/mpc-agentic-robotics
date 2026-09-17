"""Tests for vehicle model."""

import numpy as np
import pytest

from controller.vehicle_model import VehicleModel


class TestVehicleModel:
    def test_straight_line(self):
        vm = VehicleModel(dt=0.1, wheelbase=0.3)
        state = np.array([0.0, 0.0, 0.0, 1.0])  # x, y, yaw=0, v=1
        control = np.array([0.0, 0.0])            # no steer, no accel
        new_state = vm.step(state, control)
        assert abs(new_state[0] - 0.1) < 1e-6  # x increased by v*dt
        assert abs(new_state[1]) < 1e-6         # y unchanged
        assert abs(new_state[2]) < 1e-6         # yaw unchanged
        assert abs(new_state[3] - 1.0) < 1e-6  # speed unchanged

    def test_acceleration(self):
        vm = VehicleModel(dt=0.1, wheelbase=0.3)
        state = np.array([0.0, 0.0, 0.0, 0.0])
        control = np.array([0.0, 1.0])  # accel = 1
        new_state = vm.step(state, control)
        assert abs(new_state[3] - 0.1) < 1e-6

    def test_steering(self):
        vm = VehicleModel(dt=0.1, wheelbase=0.3)
        state = np.array([0.0, 0.0, 0.0, 1.0])
        control = np.array([0.3, 0.0])  # steer right
        new_state = vm.step(state, control)
        assert new_state[2] > 0  # yaw should increase

    def test_linearize_shape(self):
        vm = VehicleModel(dt=0.1, wheelbase=0.3)
        state = np.array([0.0, 0.0, 0.0, 1.0])
        control = np.array([0.1, 0.5])
        A, B, c = vm.linearize(state, control)
        assert A.shape == (4, 4)
        assert B.shape == (4, 2)
        assert c.shape == (4,)
