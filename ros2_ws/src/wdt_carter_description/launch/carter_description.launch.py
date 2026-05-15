"""Launch robot_state_publisher for one Carter namespace.

Usage:
    ros2 launch wdt_carter_description carter_description.launch.py \\
        robot_namespace:=robot_0

Spawns one robot_state_publisher node inside the given namespace,
publishing TF for base_footprint → base_link → laser_frame from the
xacro-expanded URDF in this package.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ns = LaunchConfiguration("robot_namespace")
    xacro_file = PathJoinSubstitution(
        [FindPackageShare("wdt_carter_description"), "urdf", "carter.urdf.xacro"]
    )
    robot_description = Command(
        [FindExecutable(name="xacro"), " ", xacro_file, " robot_namespace:=", ns]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace", default_value="robot_0"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                namespace=ns,
                parameters=[{"robot_description": robot_description}],
                output="screen",
            ),
        ]
    )
