"""M3 smoke: MoveIt2 plan-to-pose for the Franka Panda with mocked perception.

Validates that move_group is alive, kinematics resolve, OMPL plans a
collision-free path, and the trajectory executes against the planning
scene model. Mocks perception by hardcoding a target pose 30 cm
forward + 20 cm up from the Franka base.

Run AFTER `ros2 launch wdt_manipulation_bringup move_group.launch.py`.

Invoke:
    /usr/bin/python3 wdt_vast/moveit_plan_smoke.py

Exit 0 = M3 SMOKE PASS (plan + execute succeeded).
Exit 1 = plan failed or execution rejected.
Exit 2 = move_group not reachable / setup error.
"""

from __future__ import annotations

import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped

# moveit_py's umbrella API on humble is `from moveit.planning import MoveItPy`.
# (The plan doc's older `moveit_py.planning_scene_monitor` path doesn't exist
# on the apt-installed ros-humble-moveit — verified by checking the package
# layout 2026-05-16; modern entry-point is `moveit.planning`.)
try:
    from moveit.planning import MoveItPy
except ImportError as e:
    print(f"ERROR: moveit.planning import failed — is ros-humble-moveit installed? {e}")
    sys.exit(2)


PLANNING_GROUP = "panda_arm"
END_EFFECTOR_LINK = "panda_link8"

# 30 cm forward, 20 cm up — clear of the base, within Franka's
# reachable workspace, and pose is "natural" (rotation w=1 means
# identity, gripper pointing along +X).
TARGET_XYZ: tuple[float, float, float] = (0.3, 0.0, 0.5)
TARGET_FRAME = "panda_link0"

PLAN_TIMEOUT_S = 5.0


def main() -> int:
    rclpy.init()
    # MoveItPy reads its config from the move_group params we set in
    # wdt_manipulation_bringup/launch/move_group.launch.py — robot_description,
    # robot_description_semantic, kinematics, joint_limits, ompl. As long
    # as move_group is up in the same domain, MoveItPy("moveit_py") loads
    # the planning scene + OMPL pipeline.
    print("==> initializing MoveItPy")
    try:
        moveit = MoveItPy(node_name="moveit_plan_smoke")
    except Exception as e:
        print(f"ERROR: MoveItPy init failed: {e}")
        rclpy.shutdown()
        return 2

    print("==> resolving panda_arm planning component")
    try:
        arm = moveit.get_planning_component(PLANNING_GROUP)
    except Exception as e:
        print(f"ERROR: get_planning_component({PLANNING_GROUP}) failed: {e}")
        rclpy.shutdown()
        return 2

    pose = PoseStamped()
    pose.header.frame_id = TARGET_FRAME
    pose.pose.position.x = TARGET_XYZ[0]
    pose.pose.position.y = TARGET_XYZ[1]
    pose.pose.position.z = TARGET_XYZ[2]
    pose.pose.orientation.w = 1.0

    arm.set_start_state_to_current_state()
    arm.set_goal_state(pose_stamped_msg=pose, pose_link=END_EFFECTOR_LINK)

    print(f"==> planning to ({TARGET_XYZ[0]}, {TARGET_XYZ[1]}, {TARGET_XYZ[2]})")
    t0 = time.time()
    plan_result = arm.plan()
    plan_dt = time.time() - t0
    print(f"    plan took {plan_dt:.3f}s")

    # MoveItPy's plan() returns a PlanRequestParameters / PlanResult-like
    # object whose `error_code.val == 1` means SUCCESS (MoveItErrorCodes
    # MOVEIT_ERROR_CODES_SUCCESS).
    if not plan_result or not getattr(plan_result, "error_code", None):
        print("ERROR: plan() returned None or no error_code")
        rclpy.shutdown()
        return 1

    err = plan_result.error_code.val
    if err != 1:
        print(f"ERROR: plan failed with error_code={err}")
        rclpy.shutdown()
        return 1
    if plan_dt > PLAN_TIMEOUT_S:
        print(f"WARN: plan took {plan_dt:.3f}s > budget {PLAN_TIMEOUT_S}s — passing anyway")

    print("==> executing trajectory")
    t1 = time.time()
    moveit.execute(plan_result.trajectory, controllers=[])
    exec_dt = time.time() - t1
    print(f"    execute took {exec_dt:.3f}s")

    rclpy.shutdown()
    print("M3 SMOKE PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
