"""Pairwise deadlock detection — robots idle close together for >T seconds.

Simple distance-and-timeout heuristic: any pair of robots within
`idle_radius_m` of each other for at least `idle_secs` continuous seconds
is considered deadlocked. A robot can be in the deadlocked set if it's
deadlocked with at least one other robot; recovery (re-planning, backoff)
is up to the caller — this module only flags.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass
class DeadlockMonitor:
    idle_radius_m: float
    idle_secs: float
    _last_poses: dict[str, tuple[float, float]] = field(default_factory=dict)
    _stuck_since: dict[tuple[str, str], float] = field(default_factory=dict)
    _deadlocked: set[str] = field(default_factory=set)

    def tick(self, t: float, poses: dict[str, tuple[float, float]]) -> None:
        """Record a snapshot at time t and update the deadlocked set."""
        ids = sorted(poses)
        for i, a in enumerate(ids):
            for b in ids[i + 1 :]:
                dx = poses[a][0] - poses[b][0]
                dy = poses[a][1] - poses[b][1]
                dist = math.hypot(dx, dy)
                key = (a, b)
                if dist <= self.idle_radius_m:
                    self._stuck_since.setdefault(key, t)
                    if t - self._stuck_since[key] >= self.idle_secs:
                        self._deadlocked.update(key)
                else:
                    self._stuck_since.pop(key, None)
                    self._deadlocked.discard(a)
                    self._deadlocked.discard(b)
        self._last_poses = dict(poses)

    def deadlocked(self) -> set[str]:
        """Return the set of robot IDs currently considered deadlocked."""
        return set(self._deadlocked)
