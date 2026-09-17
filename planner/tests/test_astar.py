"""Tests for A* path planning."""

import numpy as np
import pytest

from planner.astar import astar, load_map, path_to_cells


class TestAstar:
    def test_simple_path(self):
        grid = np.zeros((5, 5), dtype=np.int8)
        path = astar(grid, (0, 0), (4, 4))
        assert path is not None
        assert path[0] == (0, 0)
        assert path[-1] == (4, 4)

    def test_no_path_blocked(self):
        grid = np.zeros((5, 5), dtype=np.int8)
        grid[2, :] = 1  # Wall across row 2
        path = astar(grid, (0, 0), (4, 4))
        assert path is None

    def test_start_is_obstacle(self):
        grid = np.zeros((5, 5), dtype=np.int8)
        grid[0, 0] = 1
        path = astar(grid, (0, 0), (4, 4))
        assert path is None

    def test_goal_equals_start(self):
        grid = np.zeros((5, 5), dtype=np.int8)
        path = astar(grid, (2, 2), (2, 2))
        assert path is not None
        assert len(path) == 1
        assert path[0] == (2, 2)

    def test_path_around_obstacle(self):
        grid = np.zeros((5, 5), dtype=np.int8)
        grid[1:4, 2] = 1  # Vertical wall
        path = astar(grid, (2, 0), (2, 4))
        assert path is not None
        assert path[0] == (2, 0)
        assert path[-1] == (2, 4)
        # Path should not go through wall
        for r, c in path:
            assert grid[r, c] == 0

    def test_no_diagonal(self):
        grid = np.zeros((5, 5), dtype=np.int8)
        path = astar(grid, (0, 0), (2, 2), allow_diagonal=False)
        assert path is not None
        assert path[0] == (0, 0)
        assert path[-1] == (2, 2)
        # Without diagonal, path should be longer
        assert len(path) >= 5  # Manhattan distance = 4, so >= 5 nodes


class TestLoadMap:
    def test_load_empty_map(self):
        m = load_map("maps/empty_20x20.yaml")
        assert m["name"] == "empty_20x20"
        assert m["grid"].shape == (20, 20)
        assert np.sum(m["grid"]) == 0

    def test_load_obstacles_map(self):
        m = load_map("maps/obstacles_20x20.yaml")
        assert m["name"] == "obstacles_20x20"
        assert np.sum(m["grid"]) > 0


class TestPathToCells:
    def test_conversion(self):
        path = [(0, 0), (1, 1), (2, 2)]
        cells = path_to_cells(path, resolution=0.5, origin=(0, 0))
        assert len(cells) == 3
        assert cells[0] == (0.0, 0.0)
        assert cells[1] == (0.5, 0.5)
