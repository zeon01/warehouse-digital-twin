import pytest

from coordinator.strategy import Goal, PathPlanner, get_planner


def test_get_planner_unknown():
    with pytest.raises(KeyError):
        get_planner("doesnotexist")


def test_get_planner_known_returns_planner():
    planner = get_planner("greedy")
    assert isinstance(planner, PathPlanner)


def test_cbs_planner_registered():
    planner = get_planner("cbs")
    assert isinstance(planner, PathPlanner)
    assert planner.name == "cbs"


def test_cbs_planner_produces_non_empty_paths():
    planner = get_planner("cbs")
    poses = {"r0": (0.0, 0.0), "r1": (2.0, 0.0)}
    goals = [Goal("r0", 1.0, 0.0), Goal("r1", 3.0, 0.0)]
    paths = planner.plan(poses, goals)
    assert len(paths) == 2
    for p in paths:
        assert len(p.waypoints) >= 2  # at least start + goal
