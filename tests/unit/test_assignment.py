from coordinator.assignment import hungarian_assign


def test_hungarian_optimal_assignment_two_robots():
    robots = {"a": (0.0, 0.0), "b": (10.0, 0.0)}
    orders = [("o1", (1.0, 0.0)), ("o2", (9.0, 0.0))]
    assignment = hungarian_assign(robots, orders)
    assert assignment == {"a": "o1", "b": "o2"}


def test_hungarian_more_robots_than_orders():
    robots = {"a": (0.0, 0.0), "b": (10.0, 0.0), "c": (5.0, 5.0)}
    orders = [("o1", (1.0, 0.0))]
    assignment = hungarian_assign(robots, orders)
    assert set(assignment.values()) == {"o1"}
    assert len(assignment) == 1


def test_hungarian_more_orders_than_robots():
    robots = {"a": (0.0, 0.0)}
    orders = [("o1", (1.0, 0.0)), ("o2", (2.0, 0.0))]
    assignment = hungarian_assign(robots, orders)
    assert len(assignment) == 1
