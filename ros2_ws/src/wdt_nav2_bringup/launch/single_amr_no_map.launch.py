"""Per-AMR Nav2 bringup MINUS the map_server.

Use this when the caller (e.g., multi_amr.launch.py) provides a shared
map_server at the top level. The lifecycle_manager here only manages
the per-namespace nodes (amcl, planner_server, etc.) — NOT map_server,
which is managed at the parent scope.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("wdt_nav2_bringup")
    params_file = PathJoinSubstitution([pkg, "config", "nav2_params.yaml"])
    ns = LaunchConfiguration("robot_namespace")

    lifecycle_nodes = [
        "amcl",
        "planner_server",
        "controller_server",
        "bt_navigator",
        "behavior_server",
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace", default_value="robot_0"),
            GroupAction(
                [
                    PushRosNamespace(ns),
                    Node(
                        package="nav2_amcl",
                        executable="amcl",
                        name="amcl",
                        parameters=[params_file],
                        output="screen",
                    ),
                    Node(
                        package="nav2_planner",
                        executable="planner_server",
                        name="planner_server",
                        parameters=[params_file],
                        output="screen",
                    ),
                    Node(
                        package="nav2_controller",
                        executable="controller_server",
                        name="controller_server",
                        parameters=[params_file],
                        output="screen",
                    ),
                    Node(
                        package="nav2_bt_navigator",
                        executable="bt_navigator",
                        name="bt_navigator",
                        parameters=[params_file],
                        output="screen",
                    ),
                    Node(
                        package="nav2_behaviors",
                        executable="behavior_server",
                        name="behavior_server",
                        parameters=[params_file],
                        output="screen",
                    ),
                    Node(
                        package="nav2_lifecycle_manager",
                        executable="lifecycle_manager",
                        name="lifecycle_manager_navigation",
                        parameters=[
                            {
                                "use_sim_time": True,
                                "autostart": True,
                                "node_names": lifecycle_nodes,
                            }
                        ],
                        output="screen",
                    ),
                ]
            ),
        ]
    )
