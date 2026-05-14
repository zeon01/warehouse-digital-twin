"""Pluggable strategy interface for multi-agent path planning.

`PathPlanner` is an abstract base; concrete planners (greedy, Hungarian +
CBS, etc.) register themselves in the `_REGISTRY` dict and are resolved
by name via `get_planner()`. Coordinator nodes ask for a planner by name
so we can swap strategies without changing call sites.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Sequence
from dataclasses import dataclass


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


_REGISTRY: dict[str, type[PathPlanner]] = {
    "greedy": GreedyPlanner,
}


def get_planner(name: str) -> PathPlanner:
    if name not in _REGISTRY:
        raise KeyError(f"unknown planner: {name}; available={list(_REGISTRY)}")
    return _REGISTRY[name]()
