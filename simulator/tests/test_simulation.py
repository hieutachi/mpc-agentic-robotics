"""Tests for simulation environment and run."""

import numpy as np
import pytest

from simulator.environment import Environment


class TestEnvironment:
    def test_load_empty_map(self):
        env = Environment("maps/empty_20x20.yaml")
        assert env.name == "empty_20x20"
        assert env.height == 20
        assert env.width == 20

    def test_no_collision_empty_map(self):
        env = Environment("maps/empty_20x20.yaml")
        assert env.check_collision(1.0, 1.0) is False

    def test_collision_on_obstacle(self):
        env = Environment("maps/obstacles_20x20.yaml")
        # Obstacle at [5,5] → world coords (2.5, 2.5)
        assert env.check_collision(2.5, 2.5) is True

    def test_out_of_bounds_is_collision(self):
        env = Environment("maps/empty_20x20.yaml")
        assert env.check_collision(-1.0, -1.0) is True
        assert env.check_collision(100.0, 100.0) is True

    def test_min_distance(self):
        env = Environment("maps/obstacles_20x20.yaml")
        d = env.min_obstacle_distance(2.5, 3.0)
        assert d >= 0
        assert d < 2.0  # Should be close to obstacle

    def test_is_inside(self):
        env = Environment("maps/empty_20x20.yaml")
        assert env.is_inside(5.0, 5.0) is True
        assert env.is_inside(-1.0, 5.0) is False
