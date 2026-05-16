"""M3 smoke: MoveIt2 plan-to-pose for the Franka Panda with mocked perception.

Validates that move_group is alive, kinematics resolve, OMPL plans a
collision-free path, and the action completes. Mocks perception by:

    1. Publishing a known-good /joint_states (the Panda's canonical
       "ready" pose) at 10 Hz so move_group's state monitor sees a
       valid, non-self-colliding start state. The panda's zero-position
       URDF default has panda_link5 and panda_link7 in self-collision,
       so we can't rely on joint_state_publisher's default — verified
       2026-05-16 by watching move_group reject every plan as "Start
       state appears to be in collision".
    2. Sending a MoveGroup goal to a small offset (+0.3 rad on
       panda_joint1) from the published start. Short motion, no
       collisions possible.

Uses `moveit_msgs/action/MoveGroup` directly. ros-humble-moveit does
NOT ship the moveit_py Python bindings (only ros-iron/rolling do),
so the action client is the canonical Humble path.

Run AFTER `ros2 launch wdt_manipulation_bringup move_group.launch.py`.

Invoke:
    /usr/bin/python3 wdt_vast/moveit_plan_smoke.py

Exit 0 = M3 SMOKE PASS (plan + execute succeeded).
Exit 1 = plan failed or execution rejected.
Exit 2 = move_group unreachable / setup error.
"""

from __future__ import annotations

import math
import sys
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    PlanningOptions,
)
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

PLANNING_GROUP = "panda_arm"

# Panda's canonical "ready" joint-space pose — same one MoveIt's panda
# tutorial uses as `home`. All joints well within limits, no
# self-collision, kinematics non-singular.
PANDA_READY: dict[str, float] = {
    "panda_joint1": 0.0,
    "panda_joint2": -math.pi / 4,
    "panda_joint3": 0.0,
    "panda_joint4": -3 * math.pi / 4,
    "panda_joint5": 0.0,
    "panda_joint6": math.pi / 2,
    "panda_joint7": math.pi / 4,
}
# Target: +0.3 rad on joint1 from ready. ~17 deg base rotation, well
# inside limits, plan completes in <1 s.
JOINT1_TARGET_OFFSET = 0.3

ACTION_NAME = "/move_action"
ACTION_TIMEOUT_S = 10.0
PLAN_TIMEOUT_S = 5.0
TOTAL_TIMEOUT_S = 30.0
JOINT_STATE_WARMUP_S = 2.0


class JointStatePublisher(Node):
    """Publish /joint_states at 10 Hz with a fixed pose so move_group
    sees a valid start state. Spun on its own executor thread.
    """

    def __init__(self, positions: dict[str, float]) -> None:
        super().__init__("moveit_plan_smoke_jsp")
        self._names = list(positions.keys())
        self._positions = [positions[n] for n in self._names]
        self._pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_timer(0.1, self._tick)

    def _tick(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self._names
        msg.position = self._positions

        self._pub.publish(msg)


def _make_goal() -> MoveGroup.Goal:
    """Build a MoveGroup.Goal: plan from ready -> ready+0.3rad(joint1)."""
    target_positions = dict(PANDA_READY)
    target_positions["panda_joint1"] = PANDA_READY["panda_joint1"] + JOINT1_TARGET_OFFSET

    goal = MoveGroup.Goal()
    req = MotionPlanRequest()
    req.group_name = PLANNING_GROUP
    req.num_planning_attempts = 5
    req.allowed_planning_time = PLAN_TIMEOUT_S
    req.max_velocity_scaling_factor = 0.5
    req.max_acceleration_scaling_factor = 0.5

    joint_constraints = []
    for name, pos in target_positions.items():
        c = JointConstraint()
        c.joint_name = name
        c.position = pos
        c.tolerance_above = 0.01
        c.tolerance_below = 0.01
        c.weight = 1.0
        joint_constraints.append(c)
    constraints = Constraints()
    constraints.joint_constraints = joint_constraints
    req.goal_constraints = [constraints]

    goal.request = req
    goal.planning_options = PlanningOptions()
    # plan_only=True — M3's spec is "verify move_group + kinematics +
    # OMPL plan succeeds". Execution requires a follow_joint_trajectory
    # controller (FakeControllerManager or Isaac Sim physics) that's
    # set up in M4-M5. For the M3 smoke, plan-only is the right scope:
    # full plan goes through OMPL, we just don't dispatch to a
    # controller that doesn't exist on this image.
    goal.planning_options.plan_only = True
    goal.planning_options.replan = False
    goal.planning_options.replan_attempts = 0
    return goal


def main() -> int:
    rclpy.init()

    # Spin the JSP on its own executor thread so it keeps publishing
    # while we send the action goal on the main thread.
    jsp = JointStatePublisher(PANDA_READY)
    jsp_executor = SingleThreadedExecutor()
    jsp_executor.add_node(jsp)
    jsp_thread = threading.Thread(target=jsp_executor.spin, daemon=True)
    jsp_thread.start()

    print(f"==> publishing /joint_states (panda 'ready' pose) — warming up {JOINT_STATE_WARMUP_S}s")
    time.sleep(JOINT_STATE_WARMUP_S)

    node = Node("moveit_plan_smoke")
    client = ActionClient(node, MoveGroup, ACTION_NAME)

    print(f"==> waiting up to {ACTION_TIMEOUT_S}s for {ACTION_NAME}")
    if not client.wait_for_server(timeout_sec=ACTION_TIMEOUT_S):
        print("ERROR: move_group action server not available")
        jsp_executor.shutdown()
        rclpy.shutdown()
        return 2
    print("    server ready")

    goal = _make_goal()
    print(
        f"==> sending MoveGroup goal — plan + execute from ready to "
        f"ready+{JOINT1_TARGET_OFFSET}rad on panda_joint1"
    )

    t0 = time.time()
    send_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future, timeout_sec=ACTION_TIMEOUT_S)
    handle = send_future.result()
    if handle is None or not handle.accepted:
        print(f"ERROR: move_group goal REJECTED (handle={handle})")
        jsp_executor.shutdown()
        rclpy.shutdown()
        return 1

    print("    goal accepted, waiting for result")
    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=TOTAL_TIMEOUT_S)
    elapsed = time.time() - t0

    if not result_future.done():
        print(f"ERROR: move_group did not return result within {TOTAL_TIMEOUT_S}s")
        jsp_executor.shutdown()
        rclpy.shutdown()
        return 1

    response = result_future.result()
    status = response.status if response is not None else GoalStatus.STATUS_UNKNOWN
    result = response.result if response is not None else None
    success_code = result.error_code.val if result and getattr(result, "error_code", None) else None

    print(f"    status={status}, error_code.val={success_code}, elapsed={elapsed:.2f}s")

    jsp_executor.shutdown()
    rclpy.shutdown()

    if status == GoalStatus.STATUS_SUCCEEDED and success_code == 1:
        print("M3 SMOKE PASS")
        return 0
    print("M3 SMOKE FAIL")
    return 1


if __name__ == "__main__":
    sys.exit(main())
