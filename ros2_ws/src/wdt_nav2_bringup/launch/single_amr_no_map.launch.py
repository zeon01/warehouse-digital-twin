"""Per-AMR Nav2 bringup MINUS the map_server, with GROUND-TRUTH pose.

Phase 2 design §8 risk-table option: instead of AMCL, publish a static
``map → odom`` transform locked to the Carter's known spawn pose. The
planner_server, controller_server, bt_navigator, behavior_server,
and both costmaps all run *as real Nav2 nodes* — only localization is
shortcut. This unblocks M2-M9 without depending on the Nova Carter
LIDAR publisher which doesn't fire under standalone-python Isaac Sim
(verified during M1 smoke: topic listed but `ros2 topic hz` returned
zero messages even with render=True and world.play()).

Carter spawns at world (1, 1, 0). odom origin = spawn pose, so the
map→odom static transform is (1, 1, 0) — that puts the AMR at (1, 1)
in the map frame at startup, consistent with the AMCL `initial_pose`
the canonical config would have set.

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

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=ns,
        param_rewrites={},
        convert_types=True,
    )

    lifecycle_nodes = [
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
                    # Ground-truth map -> odom static transform. Carter
                    # spawned at world (1, 1, 0); odom origin = spawn,
                    # so map → odom = (1, 1, 0) keeps map and world
                    # aligned. Replace with a dynamic publisher if the
                    # spawn location ever needs to change at runtime.
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
                        output="screen",
                    ),
                    # PointCloud2 -> LaserScan bridge — kept active so
                    # costmaps can still get obstacle observations once
                    # the Carter LIDAR publisher is fixed (Phase 3).
                    # Costmap obstacle_layer also configured to tolerate
                    # the topic being silent for now (no clearing).
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
                                "angle_increment": 0.0087,
                                "scan_time": 0.1,
                                "range_min": 0.2,
                                "range_max": 30.0,
                                "use_inf": True,
                            }
                        ],
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
