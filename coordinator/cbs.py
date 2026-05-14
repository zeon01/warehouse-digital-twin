"""Conflict-Based Search for multi-agent grid path planning.

Reference: Sharon et al., 2015. Implements vertex conflicts only (no edge
swaps). Sufficient for the warehouse use case at Phase 1; ECBS / edge
conflicts deferred.
"""

from __future__ import annotations

import heapq
from dataclasses import dataclass, field

Cell = tuple[int, int]
Path = list[Cell]


def _a_star(
    grid_w: int,
    grid_h: int,
    blocked: set[Cell],
    start: Cell,
    goal: Cell,
    constraints: set[tuple[int, Cell]],
) -> Path:
    """A* with timestep-cell constraints. Permits staying in place (dx=dy=0)."""

    def h(c: Cell) -> int:
        return abs(c[0] - goal[0]) + abs(c[1] - goal[1])

    open_heap: list[tuple[int, int, int, Cell, list[Cell]]] = []
    counter = 0
    heapq.heappush(open_heap, (h(start), 0, counter, start, [start]))
    seen: set[tuple[int, Cell]] = set()

    while open_heap:
        _, t, _, cur, path = heapq.heappop(open_heap)
        if cur == goal:
            return path
        if (t, cur) in seen:
            continue
        seen.add((t, cur))
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (0, 0)):
            nx, ny = cur[0] + dx, cur[1] + dy
            if not (0 <= nx < grid_w and 0 <= ny < grid_h):
                continue
            if (nx, ny) in blocked:
                continue
            nt = t + 1
            if (nt, (nx, ny)) in constraints:
                continue
            counter += 1
            heapq.heappush(
                open_heap,
                (nt + h((nx, ny)), nt, counter, (nx, ny), path + [(nx, ny)]),
            )
    return []


@dataclass(order=True)
class CTNode:
    cost: int
    paths: dict[str, Path] = field(compare=False)
    constraints: dict[str, set[tuple[int, Cell]]] = field(compare=False)


@dataclass
class GridCBS:
    grid_w: int
    grid_h: int
    blocked: set[Cell] = field(default_factory=set)

    def plan(self, agents: dict[str, tuple[Cell, Cell]]) -> dict[str, Path]:
        """Plan collision-free paths for all agents. Returns {id: path}."""
        constraints: dict[str, set[tuple[int, Cell]]] = {a: set() for a in agents}
        initial_paths: dict[str, Path] = {}
        for aid, (s, g) in agents.items():
            initial_paths[aid] = _a_star(
                self.grid_w, self.grid_h, self.blocked, s, g, constraints[aid]
            )
        cost0 = sum(len(p) for p in initial_paths.values())

        open_list: list[CTNode] = [CTNode(cost=cost0, paths=initial_paths, constraints=constraints)]
        while open_list:
            node = heapq.heappop(open_list)
            conflict = self._first_conflict(node.paths)
            if conflict is None:
                return node.paths
            (a, b, t, cell) = conflict
            for who in (a, b):
                new_constraints = {k: set(v) for k, v in node.constraints.items()}
                new_constraints[who].add((t, cell))
                start, goal = agents[who]
                new_path = _a_star(
                    self.grid_w,
                    self.grid_h,
                    self.blocked,
                    start,
                    goal,
                    new_constraints[who],
                )
                if not new_path:
                    continue
                new_paths = dict(node.paths)
                new_paths[who] = new_path
                heapq.heappush(
                    open_list,
                    CTNode(
                        cost=sum(len(p) for p in new_paths.values()),
                        paths=new_paths,
                        constraints=new_constraints,
                    ),
                )
        return {}

    @staticmethod
    def _first_conflict(paths: dict[str, Path]):
        max_t = max(len(p) for p in paths.values()) if paths else 0
        for t in range(max_t):
            positions: dict[Cell, str] = {}
            for aid, p in paths.items():
                cell = p[t] if t < len(p) else p[-1]
                if cell in positions:
                    return (positions[cell], aid, t, cell)
                positions[cell] = aid
        return None
