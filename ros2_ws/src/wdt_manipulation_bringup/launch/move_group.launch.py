"""Bring up MoveIt2 move_group for the Franka Panda pick cell.

Reuses the apt-installed moveit_resources_panda_moveit_config for the
URDF, SRDF, kinematics, joint limits, and OMPL planner config — these
are the canonical Panda MoveIt2 configs and include the full
self-collision matrix that's tedious to hand-write.

This launch starts:
  - robot_state_publisher (via wdt_franka_description)
  - move_group (panda_arm + panda_hand groups, OMPL RRTConnect default)

The pick_cell_orchestrator node is launched separately in Task 27
(once wdt_manipulation_bringup converts to ament_python). For Task 17
+ Task 18 (MoveIt2 smoke), only move_group is needed.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    panda_cfg = FindPackageShare("moveit_resources_panda_moveit_config")

    urdf_xacro = PathJoinSubstitution([panda_cfg, "config", "panda.urdf.xacro"])
    srdf_path = PathJoinSubstitution([panda_cfg, "config", "panda.srdf"])
    kinematics = PathJoinSubstitution([panda_cfg, "config", "kinematics.yaml"])
    joint_limits = PathJoinSubstitution([panda_cfg, "config", "joint_limits.yaml"])
    ompl = PathJoinSubstitution([panda_cfg, "config", "ompl_planning.yaml"])

    robot_description = Command([FindExecutable(name="xacro"), " ", urdf_xacro])
    robot_description_semantic = Command([FindExecutable(name="cat"), " ", srdf_path])

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("wdt_franka_description"),
                            "launch",
                            "franka_description.launch.py",
                        ]
                    )
                ),
            ),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[
                    {"robot_description": robot_description},
                    {"robot_description_semantic": robot_description_semantic},
                    kinematics,
                    joint_limits,
                    ompl,
                    {"use_sim_time": True},
                ],
            ),
        ]
    )
