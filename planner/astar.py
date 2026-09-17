"""A* path planning on a 2D grid map."""

import heapq
from pathlib import Path
from typing import Optional

import numpy as np
import yaml


def load_map(path: str | Path) -> dict:
    """Load a grid map from YAML file.

    Returns:
        dict with keys: name, width, height, resolution, origin, grid (ndarray).
    """
    with open(path, "r", encoding="utf-8") as f:
        raw = yaml.safe_load(f)

    w, h = raw["width"], raw["height"]
    grid = np.zeros((h, w), dtype=np.int8)

    for obs in raw.get("obstacles", []):
        if isinstance(obs, list) and len(obs) == 2 and isinstance(obs[0], int):
            # Single cell: [row, col]
            r, c = obs
            if 0 <= r < h and 0 <= c < w:
                grid[r, c] = 1
        elif isinstance(obs, list) and len(obs) == 2 and isinstance(obs[0], list):
            # Range: [[r1,c1], [r2,c2]]
            (r1, c1), (r2, c2) = obs
            for r in range(min(r1, r2), max(r1, r2) + 1):
                for c in range(min(c1, c2), max(c1, c2) + 1):
                    if 0 <= r < h and 0 <= c < w:
                        grid[r, c] = 1

    return {
        "name": raw["name"],
        "width": w,
        "height": h,
        "resolution": raw.get("resolution", 0.5),
        "origin": tuple(raw.get("origin", [0, 0])),
        "grid": grid,
    }


def _heuristic(a: tuple[int, int], b: tuple[int, int]) -> float:
    """Euclidean distance heuristic."""
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5


def astar(
    grid: np.ndarray,
    start: tuple[int, int],
    goal: tuple[int, int],
    allow_diagonal: bool = True,
) -> Optional[list[tuple[int, int]]]:
    """A* search on a 2D grid.

    Args:
        grid: 2D array where 0 = free, 1 = obstacle.
        start: (row, col) of start cell.
        goal: (row, col) of goal cell.
        allow_diagonal: whether to allow 8-connected movement.

    Returns:
        List of (row, col) from start to goal, or None if no path found.
    """
    h, w = grid.shape

    if grid[start[0], start[1]] == 1 or grid[goal[0], goal[1]] == 1:
        return None

    # 4-connected or 8-connected neighbors
    if allow_diagonal:
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1),
                      (-1, -1), (-1, 1), (1, -1), (1, 1)]
    else:
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    open_set: list[tuple[float, int, tuple[int, int]]] = []
    heapq.heappush(open_set, (0, 0, start))

    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    g_score = {start: 0.0}
    counter = 1  # tie-breaker for heapq

    while open_set:
        _, _, current = heapq.heappop(open_set)

        if current == goal:
            # Reconstruct path
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1]

        for dr, dc in directions:
            nr, nc = current[0] + dr, current[1] + dc
            if not (0 <= nr < h and 0 <= nc < w):
                continue
            if grid[nr, nc] == 1:
                continue

            # For diagonal moves, check that we don't cut corners
            if abs(dr) + abs(dc) == 2:
                if grid[current[0] + dr, current[1]] == 1 and \
                   grid[current[0], current[1] + dc] == 1:
                    continue

            move_cost = (2 ** 0.5 if abs(dr) + abs(dc) == 2 else 1.0)
            tentative_g = g_score[current] + move_cost

            neighbor = (nr, nc)
            if tentative_g < g_score.get(neighbor, float("inf")):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f = tentative_g + _heuristic(neighbor, goal)
                heapq.heappush(open_set, (f, counter, neighbor))
                counter += 1

    return None  # No path found


def path_to_cells(path: list[tuple[int, int]], resolution: float = 0.5,
                  origin: tuple[float, float] = (0, 0)) -> list[tuple[float, float]]:
    """Convert grid cell path to world coordinates.

    Args:
        path: List of (row, col) cell indices.
        resolution: meters per cell.
        origin: world coordinate of cell (0, 0).

    Returns:
        List of (x, y) in world coordinates.
    """
    return [(origin[1] + c * resolution, origin[0] + r * resolution)
            for r, c in path]
