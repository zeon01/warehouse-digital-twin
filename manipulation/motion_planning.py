"""MoveIt2 plan + (optionally execute) via the `moveit_msgs/action/MoveGroup` action client.

ros-humble-moveit does NOT ship the `moveit_py` Python bindings (verified
2026-05-16, see gotcha #2 in `feedback-foundationpose-install-gotchas`);
they're only in ros-iron / ros-rolling. The canonical Humble path is the
raw action client, so this wrapper builds a `MotionPlanRequest` with a
pose constraint and sends it to `/move_action`.

`ArmPlanner` reuses a parent rclpy Node (typically the orchestrator's)
for the action client — that way the orchestrator's executor spins both
its own callbacks and the action client's, and we avoid the
double-`rclpy.init` trap. If no parent is passed, a local Node is
created (useful for one-shot scripts + the unit tests' MagicMock path).

For the M5 smoke we use `plan_only=True` — execution requires a
`follow_joint_trajectory` controller that's not part of the apt-only
MoveIt install. The smoke validates: orchestrator → move_group →
OMPL plans a path; not actual joint motion. M5b adds real execution
once a controller manager is wired in (FakeControllerManager or Isaac
Sim physics).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass
class ArmExecutionResult:
    success: bool
    message: str


PLANNING_FRAME = "panda_link0"
END_EFFECTOR_LINK = "panda_link8"
ACTION_NAME = "/move_action"
ACTION_TIMEOUT_S = 5.0
GOAL_TIMEOUT_S = 15.0


def _rot_to_quat(R: np.ndarray) -> tuple[float, float, float, float]:
    """3x3 rotation matrix → (x, y, z, w) quaternion (Shepperd-style branchy)."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = (tr + 1.0) ** 0.5 * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    else:
        if (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            s = (1.0 + R[0, 0] - R[1, 1] - R[2, 2]) ** 0.5 * 2
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = (1.0 + R[1, 1] - R[0, 0] - R[2, 2]) ** 0.5 * 2
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = (1.0 + R[2, 2] - R[0, 0] - R[1, 1]) ** 0.5 * 2
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
    return (float(x), float(y), float(z), float(w))


class ArmPlanner:
    def __init__(
        self,
        parent_node: Any = None,
        planning_group: str = "panda_arm",
        plan_only: bool = True,
    ) -> None:
        self._planning_group = planning_group
        self._parent_node = parent_node
        self._plan_only = plan_only
        self._client: Any = None
        self._owns_node = parent_node is None
        self._node: Any = None

    def _lazy_load(self) -> None:
        if self._client is not None:
            return
        import rclpy
        from moveit_msgs.action import MoveGroup
        from rclpy.action import ActionClient
        from rclpy.node import Node

        if self._parent_node is not None:
            self._node = self._parent_node
        else:
            if not rclpy.ok():
                rclpy.init()
            self._node = Node("arm_planner_client")
        self._client = ActionClient(self._node, MoveGroup, ACTION_NAME)

    def _build_goal(self, translation: np.ndarray, rotation: np.ndarray):
        from geometry_msgs.msg import PoseStamped
        from moveit_msgs.action import MoveGroup
        from moveit_msgs.msg import (
            Constraints,
            MotionPlanRequest,
            OrientationConstraint,
            PlanningOptions,
            PositionConstraint,
        )
        from shape_msgs.msg import SolidPrimitive

        target_pose = PoseStamped()
        target_pose.header.frame_id = PLANNING_FRAME
        target_pose.pose.position.x = float(translation[0])
        target_pose.pose.position.y = float(translation[1])
        target_pose.pose.position.z = float(translation[2])
        qx, qy, qz, qw = _rot_to_quat(rotation)
        target_pose.pose.orientation.x = qx
        target_pose.pose.orientation.y = qy
        target_pose.pose.orientation.z = qz
        target_pose.pose.orientation.w = qw

        # Position constraint: a tiny sphere around target translation.
        pos_c = PositionConstraint()
        pos_c.header.frame_id = PLANNING_FRAME
        pos_c.link_name = END_EFFECTOR_LINK
        pos_c.target_point_offset.x = 0.0
        pos_c.target_point_offset.y = 0.0
        pos_c.target_point_offset.z = 0.0
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]  # 1 cm tolerance
        pos_c.constraint_region.primitives = [sphere]
        pos_c.constraint_region.primitive_poses = [target_pose.pose]
        pos_c.weight = 1.0

        # Orientation constraint.
        ori_c = OrientationConstraint()
        ori_c.header.frame_id = PLANNING_FRAME
        ori_c.link_name = END_EFFECTOR_LINK
        ori_c.orientation = target_pose.pose.orientation
        ori_c.absolute_x_axis_tolerance = 0.1
        ori_c.absolute_y_axis_tolerance = 0.1
        ori_c.absolute_z_axis_tolerance = 0.1
        ori_c.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints = [pos_c]
        constraints.orientation_constraints = [ori_c]

        req = MotionPlanRequest()
        req.group_name = self._planning_group
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.5
        req.max_acceleration_scaling_factor = 0.5
        req.goal_constraints = [constraints]

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = self._plan_only
        goal.planning_options.replan = False
        goal.planning_options.replan_attempts = 0
        return goal

    def plan_to_pose(self, translation: np.ndarray, rotation: np.ndarray) -> ArmExecutionResult:
        """Plan (and optionally execute) to a 6D goal (R, t) on panda_link8."""
        import rclpy
        from action_msgs.msg import GoalStatus

        self._lazy_load()

        if not self._client.wait_for_server(timeout_sec=ACTION_TIMEOUT_S):
            return ArmExecutionResult(False, "move_group action server not ready")

        goal = self._build_goal(translation, rotation)
        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._node, send_future, timeout_sec=ACTION_TIMEOUT_S)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return ArmExecutionResult(False, f"goal_rejected handle={handle!r}")

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future, timeout_sec=GOAL_TIMEOUT_S)
        if not result_future.done():
            return ArmExecutionResult(False, f"timeout_{GOAL_TIMEOUT_S}s")

        response = result_future.result()
        status = response.status if response is not None else GoalStatus.STATUS_UNKNOWN
        result = response.result if response is not None else None
        # MoveItErrorCodes.SUCCESS = 1
        success_code = (
            result.error_code.val if result and getattr(result, "error_code", None) else None
        )

        if status == GoalStatus.STATUS_SUCCEEDED and success_code == 1:
            return ArmExecutionResult(True, "ok")
        return ArmExecutionResult(False, f"status={status} error_code={success_code}")
