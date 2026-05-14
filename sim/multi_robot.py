"""Bring up N namespaced Nova Carter AMRs in one Isaac Sim world.

Each spawned Carter's ROS2 publisher OG nodes get their `inputs:nodeNamespace`
attribute set to `amr_{i}` so each robot's topics are namespaced (e.g.
`/amr_0/cmd_vel`, `/amr_3/chassis/odom`) instead of all six writing to the
same `/cmd_vel` channel.
"""

from __future__ import annotations

from collections.abc import Sequence

from sim.spawn import spawn_nova_carter


def _namespace_subtree(prim_path: str, namespace: str) -> int:
    """Set inputs:nodeNamespace on every OG node under prim_path. Returns count set.

    The Nova_Carter_ROS USD wires up ~30 ROS2 publisher / subscriber OG nodes
    (one each for tf, odom, imu, lidar, several stereo cameras, etc.). Each
    of these nodes has an `inputs:nodeNamespace` token attribute that, if
    set, gets prepended to the topic name. Setting them in bulk via stage
    traversal handles all publishers and subscribers without us needing to
    know each node's path explicitly.
    """
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    count = 0
    prefix = prim_path + "/"
    for prim in stage.Traverse():
        path = prim.GetPath().pathString
        if not (path == prim_path or path.startswith(prefix)):
            continue
        attr = prim.GetAttribute("inputs:nodeNamespace")
        if attr.IsValid():
            attr.Set(namespace)
            count += 1
    return count


def spawn_amr_fleet(world, spawn_poses: Sequence[tuple[float, float]]):
    """Spawn one Nova Carter per pose, each in its own namespace. Returns the robot list."""
    robots = []
    for i, pose in enumerate(spawn_poses):
        ns = f"amr_{i}"
        prim_path = f"/World/{ns}"
        r = spawn_nova_carter(world, prim_path, ns, position_xy=pose)
        n_set = _namespace_subtree(prim_path, ns)
        # Note: we don't fail if n_set is 0 — the USD might change versions
        # and we'd rather see empty topics than crash. The fleet smoke test
        # is what verifies namespacing actually worked.
        r._namespace_nodes_set = n_set  # type: ignore[attr-defined]
        robots.append(r)
    return robots
