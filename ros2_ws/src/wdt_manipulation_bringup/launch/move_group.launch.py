"""Bring up MoveIt2 move_group for the Franka Panda pick cell.

Uses ``moveit_configs_utils.MoveItConfigsBuilder`` to load the
apt-installed ``moveit_resources_panda_moveit_config`` — that's the
canonical Humble path. Direct ``--params-file`` injection of the
kinematics/joint_limits/ompl YAMLs fails because those files are in
MoveIt's own config schema (top-level keys per group/joint), NOT in
ROS2's ``<node>/ros__parameters:`` schema, so rcl rejects them with
"Cannot have a value before ros__parameters" (SIGABRT during move_group
init). Verified on the 2026-05-16 Quebec instance.

This launch starts:
  - robot_state_publisher (via wdt_franka_description)
  - move_group (panda_arm + panda_hand, OMPL RRTConnect default)

The pick_cell_orchestrator node is launched separately via Task 27
(once wdt_manipulation_bringup is the ament_python package + entry
point). For Task 18 (M3 plan-to-pose smoke) only move_group is needed.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "moveit_resources_panda",
        package_name="moveit_resources_panda_moveit_config",
    ).to_moveit_configs()

    move_group_params = [
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
        moveit_config.planning_pipelines,
        moveit_config.trajectory_execution,
        moveit_config.planning_scene_monitor,
        {"use_sim_time": True},
    ]

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
            # NOTE: no joint_state_publisher launched here on purpose —
            # the Panda's zero-position joint state (which JSP emits by
            # default) puts panda_link5 and panda_link7 in self-collision,
            # so move_group rejects every plan with "Skipping invalid
            # start state". The smoke script (moveit_plan_smoke.py)
            # publishes /joint_states with the Panda's canonical "ready"
            # pose itself before sending the action goal. In production
            # the real robot driver / Isaac Sim OG publishes /joint_states.
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=move_group_params,
            ),
        ]
    )
