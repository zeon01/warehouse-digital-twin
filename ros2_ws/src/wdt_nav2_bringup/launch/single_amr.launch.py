"""Bring up the full Nav2 stack for one namespaced AMR.

Composes a shared map_server (no namespace) + the per-namespace Nav2
nodes from single_amr_no_map.launch.py. This is the M1 entrypoint;
for fleets of 2+ AMRs use multi_amr.launch.py which shares the
map_server across namespaces.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("wdt_nav2_bringup")
    params_file = PathJoinSubstitution([pkg, "config", "nav2_params.yaml"])
    map_yaml = PathJoinSubstitution([pkg, "maps", "small.yaml"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace", default_value="amr_0"),
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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg, "launch", "single_amr_no_map.launch.py"])
                ),
                launch_arguments={
                    "robot_namespace": LaunchConfiguration("robot_namespace"),
                }.items(),
            ),
        ]
    )
