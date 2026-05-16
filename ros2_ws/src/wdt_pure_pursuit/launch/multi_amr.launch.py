"""Multi-AMR pure-pursuit driver — Nav2-bypass for the fleet.

Iterates N AMRs (default 6, matching warehouse/layouts/small.yaml's
3x2 grid at origin (2.0, 2.0) with 1.5m spacing). Each AMR gets:

    * its own ``map → odom`` static transform locked to its spawn pose,
    * its own ``pure_pursuit_driver`` action server on
      ``/amr_{i}/navigate_to_pose``,
    * the same tf/tf_static remappings as single_amr.launch.py (per
      the tf2_ros hardcoded-/tf gotcha — see launch comments).

The spawn-pose grid mirrors what ``sim/multi_robot.py::spawn_amr_fleet``
+ ``warehouse/layouts/small.yaml`` produce. If the layout grid changes,
update DEFAULT_SPAWN_POSES below or pass a JSON-encoded override via
the ``spawn_poses`` launch arg.

Use this OR ``wdt_nav2_bringup multi_amr.launch.py``, not both —
they share the per-AMR ``/<ns>/navigate_to_pose`` action name.
fleet_coordinator works unchanged either way (it just binds to
``/amr_i/navigate_to_pose`` for each ``i``).
"""

from __future__ import annotations

import json

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

# Mirrors `warehouse/layouts/small.yaml`:
#   amrs.spawn.grid = [3, 2]
#   amrs.spawn.origin_xy = [2.0, 2.0]
#   amrs.spawn.spacing_m = 1.5
# Order matches `sim/multi_robot.py::spawn_amr_fleet`'s row-major
# iteration (col-then-row): amr_0..amr_5.
DEFAULT_SPAWN_POSES: list[tuple[float, float]] = [
    (2.0, 2.0),
    (3.5, 2.0),
    (5.0, 2.0),
    (2.0, 3.5),
    (3.5, 3.5),
    (5.0, 3.5),
]


def _per_amr_group(i: int, xy: tuple[float, float], context_args) -> GroupAction:
    """Build the GroupAction for one AMR's stack: namespace + static TF + driver."""
    ns = f"amr_{i}"
    return GroupAction(
        [
            PushRosNamespace(ns),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom_static",
                arguments=[
                    "--x",
                    str(xy[0]),
                    "--y",
                    str(xy[1]),
                    "--z",
                    "0.0",
                    "--roll",
                    "0.0",
                    "--pitch",
                    "0.0",
                    "--yaw",
                    "0.0",
                    "--frame-id",
                    "map",
                    "--child-frame-id",
                    "odom",
                ],
                # See single_amr.launch.py — tf2_ros C++ broadcasters
                # hardcode absolute /tf{,_static}, so we must remap to
                # relative for PushRosNamespace to route them per-AMR.
                remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
                output="screen",
            ),
            Node(
                package="wdt_pure_pursuit",
                executable="pure_pursuit_driver",
                name="pure_pursuit_driver",
                parameters=[
                    {
                        "map_frame": "map",
                        "base_frame": "base_link",
                        "cmd_vel_topic": "cmd_vel",
                        "action_name": "navigate_to_pose",
                        "control_rate_hz": 20.0,
                        "goal_timeout_s": context_args["goal_timeout_s"],
                        "max_linear": context_args["max_linear"],
                        "max_angular": context_args["max_angular"],
                        "goal_tolerance_m": context_args["goal_tolerance_m"],
                    }
                ],
                remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
                output="screen",
            ),
        ]
    )


def _spawn_groups(context):
    raw = LaunchConfiguration("spawn_poses").perform(context).strip()
    if raw:
        try:
            poses = [tuple(pair) for pair in json.loads(raw)]
        except (json.JSONDecodeError, TypeError, ValueError) as e:
            raise RuntimeError(f"spawn_poses must be JSON list[[x,y]]; got {raw!r}") from e
    else:
        poses = DEFAULT_SPAWN_POSES

    args = {
        "goal_timeout_s": float(LaunchConfiguration("goal_timeout_s").perform(context)),
        "max_linear": float(LaunchConfiguration("max_linear").perform(context)),
        "max_angular": float(LaunchConfiguration("max_angular").perform(context)),
        "goal_tolerance_m": float(LaunchConfiguration("goal_tolerance_m").perform(context)),
    }

    return [_per_amr_group(i, xy, args) for i, xy in enumerate(poses)]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "spawn_poses",
                default_value="",
                description=(
                    "JSON list of [x, y] spawn poses per AMR. Empty → use "
                    "DEFAULT_SPAWN_POSES (warehouse/layouts/small.yaml grid)."
                ),
            ),
            DeclareLaunchArgument("max_linear", default_value="0.5"),
            DeclareLaunchArgument("max_angular", default_value="1.0"),
            DeclareLaunchArgument("goal_tolerance_m", default_value="0.25"),
            DeclareLaunchArgument("goal_timeout_s", default_value="60.0"),
            OpaqueFunction(function=_spawn_groups),
        ]
    )
