"""Launch Nav2 for a single namespaced AMR.

Usage:
    ros2 launch warehouse_bringup amr.launch.py ns:=amr_0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    ns = LaunchConfiguration("ns")
    params_file = PathJoinSubstitution(
        [FindPackageShare("warehouse_bringup"), "config", "nav2_amr.yaml"]
    )
    nav2_launch = PathJoinSubstitution(
        [FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("ns", default_value="amr_0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch),
                launch_arguments={
                    "namespace": ns,
                    "use_sim_time": "True",
                    "params_file": params_file,
                    "use_composition": "False",
                }.items(),
            ),
        ]
    )
