"""Single-AMR pure-pursuit driver — Nav2 controller fallback.

Brings up everything needed to drive one Carter to a goal *without* the
Nav2 stack:

    * static ``map → odom`` transform (Carter spawn-pose locked, same
      ground-truth pose Nav2's single_amr_no_map uses).
    * ``pure_pursuit_driver`` action server on
      ``/<robot_namespace>/navigate_to_pose``.

Use this OR ``wdt_nav2_bringup single_amr.launch.py``, not both — they
share the action name. fleet_coordinator binds to the action name and
works unchanged either way.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    ns = LaunchConfiguration("robot_namespace")

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace", default_value="amr_0"),
            DeclareLaunchArgument("max_linear", default_value="0.5"),
            DeclareLaunchArgument("max_angular", default_value="1.0"),
            DeclareLaunchArgument("goal_tolerance_m", default_value="0.25"),
            DeclareLaunchArgument("goal_timeout_s", default_value="60.0"),
            GroupAction(
                [
                    PushRosNamespace(ns),
                    Node(
                        package="tf2_ros",
                        executable="static_transform_publisher",
                        name="map_to_odom_static",
                        arguments=[
                            "--x",
                            "1.0",
                            "--y",
                            "1.0",
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
                        # static_transform_publisher's C++ broadcaster
                        # HARDCODES /tf_static (absolute), so PushRosNamespace
                        # does NOT route it under the namespace. We must
                        # remap explicitly. Same for /tf (in case any
                        # publisher elsewhere uses TransformBroadcaster).
                        # Verified 2026-05-16: without these remappings the
                        # pp_driver's tf2 buffer sees /amr_0/tf (Carter) but
                        # not /amr_0/tf_static (this node) — "two
                        # unconnected trees" error.
                        remappings=[
                            ("/tf", "tf"),
                            ("/tf_static", "tf_static"),
                        ],
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
                                "goal_timeout_s": LaunchConfiguration("goal_timeout_s"),
                                "max_linear": LaunchConfiguration("max_linear"),
                                "max_angular": LaunchConfiguration("max_angular"),
                                "goal_tolerance_m": LaunchConfiguration("goal_tolerance_m"),
                            }
                        ],
                        # tf2_ros::TransformListener ALSO hardcodes /tf and
                        # /tf_static absolute paths. Remap them back to
                        # relative so PushRosNamespace routes us to
                        # /amr_0/tf and /amr_0/tf_static where Carter +
                        # the static publisher actually live.
                        remappings=[
                            ("/tf", "tf"),
                            ("/tf_static", "tf_static"),
                        ],
                        output="screen",
                    ),
                ]
            ),
        ]
    )
