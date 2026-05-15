"""Per-AMR Nav2 bringup MINUS the map_server.

Use this when the caller (e.g., multi_amr.launch.py) provides a shared
map_server at the top level. The lifecycle_manager here only manages
the per-namespace nodes (amcl, planner_server, etc.) — NOT map_server,
which is managed at the parent scope.

Also bridges Nova Carter's PointCloud2 LIDAR (which Isaac Sim's
``Nova_Carter_ROS.usd`` publishes on ``<ns>/front_3d_lidar/lidar_points``)
to the LaserScan topic Nav2's AMCL + costmap expect on ``<ns>/scan``,
via the ``pointcloud_to_laserscan`` ROS2 package.

Params are loaded via ``RewrittenYaml(root_key=namespace, ...)`` —
without that the nested keys (e.g. ``FollowPath.critics``) silently
don't reach the namespaced node because ROS2's param-loader can't
match ``controller_server:`` against a node at ``/<ns>/controller_server``
when both PushRosNamespace and nested-yaml keys are in play.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg = FindPackageShare("wdt_nav2_bringup")
    params_file = PathJoinSubstitution([pkg, "config", "nav2_params.yaml"])
    ns = LaunchConfiguration("robot_namespace")

    # Substitute the namespace into the params YAML at launch time so
    # the YAML's top-level keys (`amcl:`, `controller_server:`, …) get
    # prefixed with `<ns>/`. Mirrors nav2_bringup's nav2_multirobot_launch
    # pattern.
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=ns,
        param_rewrites={},
        convert_types=True,
    )

    lifecycle_nodes = [
        "amcl",
        "planner_server",
        "controller_server",
        "bt_navigator",
        "behavior_server",
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace", default_value="amr_0"),
            GroupAction(
                [
                    PushRosNamespace(ns),
                    # PointCloud2 -> LaserScan bridge for AMCL + costmap.
                    Node(
                        package="pointcloud_to_laserscan",
                        executable="pointcloud_to_laserscan_node",
                        name="pointcloud_to_laserscan",
                        remappings=[
                            ("cloud_in", "front_3d_lidar/lidar_points"),
                            ("scan", "scan"),
                        ],
                        parameters=[
                            {
                                "use_sim_time": True,
                                "target_frame": "base_link",
                                "min_height": 0.1,
                                "max_height": 1.5,
                                "angle_min": -3.14159,
                                "angle_max": 3.14159,
                                "angle_increment": 0.0087,  # ~0.5°
                                "scan_time": 0.1,
                                "range_min": 0.2,
                                "range_max": 30.0,
                                "use_inf": True,
                            }
                        ],
                        output="screen",
                    ),
                    Node(
                        package="nav2_amcl",
                        executable="amcl",
                        name="amcl",
                        parameters=[configured_params],
                        output="screen",
                    ),
                    Node(
                        package="nav2_planner",
                        executable="planner_server",
                        name="planner_server",
                        parameters=[configured_params],
                        output="screen",
                    ),
                    Node(
                        package="nav2_controller",
                        executable="controller_server",
                        name="controller_server",
                        parameters=[configured_params],
                        output="screen",
                    ),
                    Node(
                        package="nav2_bt_navigator",
                        executable="bt_navigator",
                        name="bt_navigator",
                        parameters=[configured_params],
                        output="screen",
                    ),
                    Node(
                        package="nav2_behaviors",
                        executable="behavior_server",
                        name="behavior_server",
                        parameters=[configured_params],
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
