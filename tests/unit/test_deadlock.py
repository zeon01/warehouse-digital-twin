from coordinator.deadlock import DeadlockMonitor


def test_no_deadlock_when_robots_apart():
    mon = DeadlockMonitor(idle_radius_m=1.0, idle_secs=5.0)
    mon.tick(t=0.0, poses={"a": (0.0, 0.0), "b": (5.0, 0.0)})
    mon.tick(t=10.0, poses={"a": (0.0, 0.0), "b": (5.0, 0.0)})
    assert mon.deadlocked() == set()


def test_deadlock_when_two_robots_idle_close_for_threshold():
    mon = DeadlockMonitor(idle_radius_m=1.0, idle_secs=5.0)
    mon.tick(t=0.0, poses={"a": (0.0, 0.0), "b": (0.5, 0.0)})
    mon.tick(t=4.0, poses={"a": (0.0, 0.0), "b": (0.5, 0.0)})
    assert mon.deadlocked() == set()
    mon.tick(t=6.0, poses={"a": (0.0, 0.0), "b": (0.5, 0.0)})
    assert {"a", "b"}.issubset(mon.deadlocked())
