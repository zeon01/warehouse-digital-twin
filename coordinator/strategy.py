"""Pluggable strategy interface for multi-agent path planning.

`PathPlanner` is an abstract base; concrete planners (greedy, CBS, etc.)
register themselves in the `_REGISTRY` dict and are resolved by name via
`get_planner()`. Coordinator nodes ask for a planner by name so we can
swap strategies without changing call sites.

Phase 2 adds the ``cbs`` registration wrapping ``coordinator.cbs.GridCBS``
so the planner ablation can switch between ``greedy`` (straight-line) and
``cbs`` (conflict-free) via a single ROS2 parameter.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Sequence
from dataclasses import dataclass

from coordinator.cbs import GridCBS


@dataclass(frozen=True)
class Goal:
    robot_id: str
    x: float
    y: float


@dataclass(frozen=True)
class Path:
    robot_id: str
    waypoints: tuple[tuple[float, float], ...]


class PathPlanner(ABC):
    name: str = "abstract"

    @abstractmethod
    def plan(
        self,
        robot_poses: dict[str, tuple[float, float]],
        goals: Sequence[Goal],
    ) -> list[Path]: ...


class GreedyPlanner(PathPlanner):
    """Trivial straight-line planner. Each goal gets a 2-waypoint path
    (current pose → goal); no collision check, no obstacle avoidance.
    Useful as a baseline and as a sanity-check for the coordinator wiring.
    """

    name = "greedy"

    def plan(self, robot_poses, goals):
        return [Path(g.robot_id, (robot_poses[g.robot_id], (g.x, g.y))) for g in goals]


class CBSPlanner(PathPlanner):
    """Conflict-Based Search wrapper for the Phase 2 ablation.

    Converts continuous (x, y) world coordinates into ``GridCBS`` cells
    at 5cm resolution (matching the Nav2 occupancy grid in
    ``warehouse/maps/small.{pgm,yaml}``), runs CBS, then converts the
    resulting cell paths back to world-coord waypoints.

    Defaults match the ``small`` layout (20 m × 30 m). For larger
    warehouses pass ``grid_w`` / ``grid_h`` / ``blocked`` overrides.
    """

    name = "cbs"

    def __init__(
        self,
        grid_w: int = 400,
        grid_h: int = 600,
        resolution_m_per_px: float = 0.05,
        blocked: set[tuple[int, int]] | None = None,
    ) -> None:
        self._resolution = resolution_m_per_px
        self._impl = GridCBS(grid_w=grid_w, grid_h=grid_h, blocked=blocked or set())

    def plan(
        self,
        robot_poses: dict[str, tuple[float, float]],
        goals: Sequence[Goal],
    ) -> list[Path]:
        agents: dict[str, tuple[tuple[int, int], tuple[int, int]]] = {}
        for g in goals:
            start_xy = robot_poses[g.robot_id]
            start_cell = (
                int(round(start_xy[0] / self._resolution)),
                int(round(start_xy[1] / self._resolution)),
            )
            goal_cell = (
                int(round(g.x / self._resolution)),
                int(round(g.y / self._resolution)),
            )
            agents[g.robot_id] = (start_cell, goal_cell)
        solution = self._impl.plan(agents)
        paths: list[Path] = []
        for rid, cells in solution.items():
            waypoints = tuple((c[0] * self._resolution, c[1] * self._resolution) for c in cells)
            paths.append(Path(rid, waypoints))
        return paths


_REGISTRY: dict[str, type[PathPlanner]] = {
    "greedy": GreedyPlanner,
    "cbs": CBSPlanner,
}


def get_planner(name: str) -> PathPlanner:
    if name not in _REGISTRY:
        raise KeyError(f"unknown planner: {name}; available={list(_REGISTRY)}")
    return _REGISTRY[name]()
