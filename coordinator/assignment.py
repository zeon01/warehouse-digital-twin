"""Hungarian-algorithm task allocation for the AMR fleet.

Wraps scipy.optimize.linear_sum_assignment over an Euclidean-distance
cost matrix. Handles unbalanced cases (more robots than orders or vice
versa) by returning a partial assignment — linear_sum_assignment can
take rectangular matrices directly so we don't need to pad.
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
