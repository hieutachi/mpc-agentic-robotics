"""2D simulation environment with grid map and obstacle detection."""

import numpy as np
from pathlib import Path

import yaml

from planner.astar import load_map


class Environment:
    """2D grid environment for robot simulation.

    Wraps a grid map with collision checking and distance computation.
    """

    def __init__(self, map_path: str | Path):
        map_data = load_map(map_path)
        self.name = map_data["name"]
        self.grid = map_data["grid"]
        self.resolution = map_data["resolution"]
        self.origin = map_data["origin"]
        self.height, self.width = self.grid.shape

        # Precompute obstacle positions for fast distance queries
        self._obstacle_cells = np.argwhere(self.grid == 1)
        self._obstacle_xy = None
        if len(self._obstacle_cells) > 0:
            self._obstacle_xy = (
                self._obstacle_cells[:, [1, 0]].astype(float) * self.resolution
                + np.array([self.origin[1], self.origin[0]])
            )

    def world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        """Convert world coordinates to grid cell (row, col)."""
        c = int((x - self.origin[1]) / self.resolution)
        r = int((y - self.origin[0]) / self.resolution)
        return r, c

    def grid_to_world(self, r: int, c: int) -> tuple[float, float]:
        """Convert grid cell to world coordinates (x, y)."""
        x = self.origin[1] + c * self.resolution
        y = self.origin[0] + r * self.resolution
        return x, y

    def check_collision(self, x: float, y: float, radius: float = 0.0) -> bool:
        """Check if a point (or circle) collides with any obstacle.

        Args:
            x, y: World coordinates.
            radius: Collision radius around the point.

        Returns:
            True if collision detected.
        """
        r, c = self.world_to_grid(x, y)

        # Out of bounds
        if r < 0 or r >= self.height or c < 0 or c >= self.width:
            return True

        # Direct cell check
        if self.grid[r, c] == 1:
            return True

        # Radius check: look at neighboring cells
        if radius > 0:
            cell_radius = int(np.ceil(radius / self.resolution))
            for dr in range(-cell_radius, cell_radius + 1):
                for dc in range(-cell_radius, cell_radius + 1):
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < self.height and 0 <= nc < self.width:
                        if self.grid[nr, nc] == 1:
                            # Check actual distance
                            ox, oy = self.grid_to_world(nr, nc)
                            dist = ((x - ox) ** 2 + (y - oy) ** 2) ** 0.5
                            if dist < radius + self.resolution * 0.5:
                                return True
        return False

    def min_obstacle_distance(self, x: float, y: float) -> float:
        """Compute minimum distance from (x, y) to any obstacle.

        Returns:
            Distance in meters. Returns 0 if on an obstacle.
        """
        if self._obstacle_xy is None or len(self._obstacle_xy) == 0:
            return float("inf")

        dists = np.sqrt(
            (self._obstacle_xy[:, 0] - x) ** 2 + (self._obstacle_xy[:, 1] - y) ** 2
        )
        return float(np.min(dists))

    def is_inside(self, x: float, y: float) -> bool:
        """Check if (x, y) is inside the map boundaries."""
        r, c = self.world_to_grid(x, y)
        return 0 <= r < self.height and 0 <= c < self.width
