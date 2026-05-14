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
    """Namespace all ROS2 publishers/subscribers under prim_path. Returns count set.

    Nova_Carter_ROS uses TWO namespacing patterns and we need to patch both:

    1. Direct `inputs:nodeNamespace` on publisher OG nodes (e.g. the per-IMU
       ROS2PublishImu nodes). Setting this directly prepends to the topic.

    2. Indirect via a "namespace" or "node_namespace" constant OG node whose
       `inputs:value` is connected as an input to other publishers (drives
       /cmd_vel, /chassis/odom, /front_3d_lidar/lidar_points, stereo camera
       publishers, etc.). The constant's pre-baked value is either '' / 'None'
       (no namespace) or a partial like 'front_fisheye_camera/left'; we
       prepend our namespace so multi-instance topics don't collide.
    """
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    count = 0
    prefix = prim_path + "/"
    for prim in stage.Traverse():
        path = prim.GetPath().pathString
        if not (path == prim_path or path.startswith(prefix)):
            continue

        # Pattern 1: direct nodeNamespace on a publisher node
        ns_attr = prim.GetAttribute("inputs:nodeNamespace")
        if ns_attr.IsValid():
            ns_attr.Set(namespace)
            count += 1

        # Pattern 2: namespace / node_namespace constant OG node feeding others
        if path.endswith("/namespace") or path.endswith("/node_namespace"):
            val_attr = prim.GetAttribute("inputs:value")
            if val_attr.IsValid():
                current = str(val_attr.Get() or "").strip()
                if current in ("", "None"):
                    new = namespace
                else:
                    new = f"{namespace}/{current}"
                val_attr.Set(new)
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
