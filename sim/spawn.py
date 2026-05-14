"""Spawn helpers for Nova Carter (AMR) and Franka (manipulator) in Isaac Sim 5.0.

Module is only callable inside Isaac Sim's Python runtime
(/isaac-sim/kit/python/bin/python3 via /isaac-sim/python.sh).

Default asset paths point at NVIDIA's S3-hosted Isaac Sim 5.0 catalog so the
helpers work on a fresh vast.ai container without us pre-pulling anything.
Isaac Sim's USD loader resolves HTTPS URLs natively. Override `asset_usd`
to point at a local file or Modal Volume path if you've pre-pulled.
"""

from __future__ import annotations

from collections.abc import Sequence

# S3-hosted Isaac Sim 5.0 vendor-namespaced asset paths (per Task 8 discovery —
# the plan's `Robots/NovaCarter/...` and `Robots/Franka/...` paths 404 on the
# 5.0 catalog because NVIDIA reorganized under vendor folders).
_ISAAC_S3 = "https://omniverse-content-production.s3.us-west-2.amazonaws.com/Assets/Isaac/5.0/Isaac"

# Nova_Carter_ROS.usd wraps the bare nova_carter.usd and ADDS the OmniGraph
# action graphs that publish /tf, /odom, /clock and subscribe to /cmd_vel.
# The bare `Robots/NVIDIA/NovaCarter/nova_carter.usd` has NO ROS wiring — the
# bridge can be loaded but topics stay empty. Verified via the NVIDIA
# benchmark example at /isaac-sim/standalone_examples/benchmarks/
# benchmark_robots_nova_carter_ros2.py which references this exact path.
NOVA_CARTER_USD_S3 = f"{_ISAAC_S3}/Samples/ROS2/Robots/Nova_Carter_ROS.usd"
FRANKA_USD_S3 = f"{_ISAAC_S3}/Robots/FrankaRobotics/FrankaPanda/franka.usd"


def spawn_nova_carter(
    world,
    prim_path: str,
    name: str,
    position_xy: Sequence[float],
    asset_usd: str | None = None,
):
    """Spawn a Nova Carter AMR at (x, y, 0) under prim_path. Returns the robot."""
    import numpy as np
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.robot.wheeled_robots.robots import WheeledRobot

    asset_usd = asset_usd or NOVA_CARTER_USD_S3
    add_reference_to_stage(usd_path=asset_usd, prim_path=prim_path)
    # Actual joint names in the Isaac Sim 5.0 Nova Carter USD are
    # joint_wheel_left / joint_wheel_right — the plan listed
    # left_wheel_joint / right_wheel_joint which 404s with KeyError on
    # world.reset() (verified by traversing the USD and listing
    # RevoluteJoint prims).
    robot = WheeledRobot(
        prim_path=prim_path,
        name=name,
        wheel_dof_names=["joint_wheel_left", "joint_wheel_right"],
        create_robot=False,
        position=np.array([position_xy[0], position_xy[1], 0.0]),
    )
    world.scene.add(robot)
    return robot


def spawn_franka(
    world,
    prim_path: str,
    name: str,
    position_xyz: Sequence[float],
    asset_usd: str | None = None,
):
    """Spawn a Franka Panda arm at position_xyz under prim_path. Returns the arm."""
    import numpy as np
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.robot.manipulators.examples.franka import Franka

    asset_usd = asset_usd or FRANKA_USD_S3
    add_reference_to_stage(usd_path=asset_usd, prim_path=prim_path)
    arm = Franka(
        prim_path=prim_path,
        name=name,
        position=np.array(position_xyz),
    )
    world.scene.add(arm)
    return arm
