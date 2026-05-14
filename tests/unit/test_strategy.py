import pytest

from coordinator.strategy import PathPlanner, get_planner


def test_get_planner_unknown():
    with pytest.raises(KeyError):
        get_planner("doesnotexist")


def test_get_planner_known_returns_planner():
    planner = get_planner("greedy")
    assert isinstance(planner, PathPlanner)
