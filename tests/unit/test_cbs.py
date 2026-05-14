from coordinator.cbs import GridCBS


def test_cbs_resolves_corridor_conflict():
    """Two robots approach in a 1-wide corridor; CBS must reroute one."""
    cbs = GridCBS(grid_w=5, grid_h=3, blocked=set())
    # Robot A: (0,1) → (4,1); Robot B: (4,1) → (0,1) — head-on in middle row.
    paths = cbs.plan({"a": ((0, 1), (4, 1)), "b": ((4, 1), (0, 1))})
    # Both paths exist; they never occupy the same cell at the same timestep.
    assert "a" in paths and "b" in paths
    seen: set[tuple[int, tuple[int, int]]] = set()
    for _rid, p in paths.items():
        for t, cell in enumerate(p):
            key = (t, cell)
            assert key not in seen, f"collision at t={t} cell={cell}"
            seen.add(key)


def test_cbs_handles_no_conflict():
    cbs = GridCBS(grid_w=5, grid_h=5, blocked=set())
    paths = cbs.plan({"a": ((0, 0), (4, 0)), "b": ((0, 4), (4, 4))})
    assert paths["a"][0] == (0, 0) and paths["a"][-1] == (4, 0)
    assert paths["b"][0] == (0, 4) and paths["b"][-1] == (4, 4)
