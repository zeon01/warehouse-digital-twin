"""Task allocation for the AMR fleet.

Two allocators are exported:

- :func:`hungarian_assign` — optimal (in total-distance sense) via
  ``scipy.optimize.linear_sum_assignment`` over an Euclidean cost
  matrix. Handles unbalanced inputs without padding.
- :func:`nearest_assign` — greedy nearest-AMR baseline used as the
  Phase 2 ablation control. Iterates robots in sorted-id order so
  results are deterministic.
"""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np
from scipy.optimize import linear_sum_assignment


def hungarian_assign(
    robots: dict[str, tuple[float, float]],
    orders: Sequence[tuple[str, tuple[float, float]]],
) -> dict[str, str]:
    """Return {robot_id: order_id} that minimizes total Euclidean distance.

    Unbalanced inputs (more robots than orders, or vice versa) yield a
    partial assignment of size min(len(robots), len(orders)).
    """
    if not robots or not orders:
        return {}

    robot_ids = list(robots)
    order_ids = [o[0] for o in orders]

    rxy = np.array([robots[r] for r in robot_ids])
    oxy = np.array([o[1] for o in orders])

    diff = rxy[:, None, :] - oxy[None, :, :]
    cost = np.linalg.norm(diff, axis=-1)

    row_ind, col_ind = linear_sum_assignment(cost)
    return {robot_ids[r]: order_ids[c] for r, c in zip(row_ind, col_ind, strict=False)}


def nearest_assign(
    robots: dict[str, tuple[float, float]],
    orders: Sequence[tuple[str, tuple[float, float]]],
) -> dict[str, str]:
    """Greedy nearest-order assignment, one pass over sorted robots.

    Each robot (in sorted-id order) grabs the closest still-unassigned
    order. Not optimal — used as the Phase 2 ablation baseline against
    :func:`hungarian_assign`. Deterministic given the same input.
    """
    if not robots or not orders:
        return {}

    assignment: dict[str, str] = {}
    available = list(orders)
    for r_id, r_xy in sorted(robots.items()):
        if not available:
            break
        idx = min(
            range(len(available)),
            key=lambda i: (available[i][1][0] - r_xy[0]) ** 2 + (available[i][1][1] - r_xy[1]) ** 2,
        )
        assignment[r_id] = available[idx][0]
        del available[idx]
    return assignment
