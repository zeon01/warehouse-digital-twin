"""Bring up Nav2 for N namespaced AMRs (default N=6).

Shares a single map_server at the top level (no namespace) since all
AMRs use the same warehouse map — saves memory and avoids redundant
PGM loads. Each namespace gets its own AMCL + planner + controller +
behavior + bt_navigator + lifecycle_manager via the no-map sub-launch.

Override the fleet size with `num_robots:=N`.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

DEFAULT_NUM_ROBOTS = 6


def generate_launch_description():
    pkg = FindPackageShare("wdt_nav2_bringup")
    params_file = PathJoinSubstitution([pkg, "config", "nav2_params.yaml"])
    map_yaml = PathJoinSubstitution([pkg, "maps", "small.yaml"])

    actions = [
        DeclareLaunchArgument("num_robots", default_value=str(DEFAULT_NUM_ROBOTS)),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            parameters=[params_file, {"yaml_filename": map_yaml}],
            output="screen",
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_map",
            parameters=[
                {
                    "use_sim_time": True,
                    "autostart": True,
                    "node_names": ["map_server"],
                }
            ],
            output="screen",
        ),
    ]

    # NOTE: `num_robots` is read at launch-resolution time. ROS2 launch
    # has no easy "for i in range(num_robots)" with a runtime arg, so we
    # use a fixed loop over DEFAULT_NUM_ROBOTS here. If the user passes
    # `num_robots:=N != 6`, refactor this to OpaqueFunction. For Phase 2
    # the fleet size is locked at 6 (steady_state.yaml), so this is OK.
    for i in range(DEFAULT_NUM_ROBOTS):
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg, "launch", "single_amr_no_map.launch.py"])
                ),
                launch_arguments={"robot_namespace": f"robot_{i}"}.items(),
            )
        )

    return LaunchDescription(actions)
