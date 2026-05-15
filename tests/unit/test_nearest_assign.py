"""Unit tests for coordinator.assignment.nearest_assign — the greedy baseline."""

from __future__ import annotations


def test_nearest_assign_basic():
    from coordinator.assignment import nearest_assign

    robots = {"r0": (0.0, 0.0), "r1": (10.0, 10.0)}
    orders = [("o0", (1.0, 1.0)), ("o1", (11.0, 11.0))]
    result = nearest_assign(robots, orders)
    assert result == {"r0": "o0", "r1": "o1"}


def test_nearest_assign_more_orders_than_robots():
    from coordinator.assignment import nearest_assign

    robots = {"r0": (0.0, 0.0)}
    orders = [("o0", (1.0, 1.0)), ("o1", (10.0, 10.0))]
    result = nearest_assign(robots, orders)
    assert result == {"r0": "o0"}


def test_nearest_assign_more_robots_than_orders():
    from coordinator.assignment import nearest_assign

    robots = {"r0": (0.0, 0.0), "r1": (1.0, 1.0), "r2": (10.0, 10.0)}
    orders = [("o0", (0.5, 0.5))]
    result = nearest_assign(robots, orders)
    # Only one order; goes to closest robot (r0 — assignment iterates by
    # sorted robot id, so r0 gets first dibs).
    assert result == {"r0": "o0"}


def test_nearest_assign_empty():
    from coordinator.assignment import nearest_assign

    assert nearest_assign({}, []) == {}
    assert nearest_assign({"r0": (0.0, 0.0)}, []) == {}
    assert nearest_assign({}, [("o0", (0.0, 0.0))]) == {}
