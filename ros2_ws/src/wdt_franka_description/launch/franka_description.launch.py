"""Publish Franka Panda robot_description + TF for the warehouse pick cell.

Uses the apt-installed franka_description package's URDF (via
moveit_resources_panda_moveit_config's xacro entrypoint, which sets
the standard Panda parameters) and spins up a robot_state_publisher.

The Franka in the warehouse digital twin is stationary at the pick
cell, so we don't namespace this — there's only one. Phase 2's
wdt_manipulation_bringup/move_group.launch.py includes this.
"""

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # moveit_resources_panda_moveit_config ships panda.urdf.xacro in its
    # config/ dir, paired with the SRDF + MoveIt2 configs we reuse in
    # wdt_manipulation_bringup. Stay consistent so robot_description in
    # robot_state_publisher matches what move_group sees.
    urdf_xacro = PathJoinSubstitution(
        [
            FindPackageShare("moveit_resources_panda_moveit_config"),
            "config",
            "panda.urdf.xacro",
        ]
    )
    robot_description = Command([FindExecutable(name="xacro"), " ", urdf_xacro])

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="franka_robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                output="screen",
            ),
        ]
    )
