"""MoveIt2 plan + execute via the moveit_py Python binding.

Wraps `moveit.planning.MoveItPy` with a single `plan_to_pose()` entry that
takes a 6-DoF goal (rotation matrix + translation) and runs plan-and-execute
on the named planning group (default `panda_arm`). The moveit_py dependency
is loaded lazily so the wrapper imports on systems without MoveIt2 installed.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class ArmExecutionResult:
    success: bool
    message: str


class ArmPlanner:
    def __init__(self, planning_group: str = "panda_arm"):
        self._planning_group = planning_group
        self._mp = None

    def _lazy_load(self):
        if self._mp is not None:
            return
        from moveit.planning import MoveItPy  # type: ignore[import]

        self._mp = MoveItPy(node_name="moveit_py_arm")

    def plan_to_pose(self, translation: np.ndarray, rotation: np.ndarray) -> ArmExecutionResult:
        """Plan to a 6D goal expressed as (R, t) and execute."""
        self._lazy_load()
        arm = self._mp.get_planning_component(self._planning_group)

        from geometry_msgs.msg import PoseStamped

        target = PoseStamped()
        target.header.frame_id = "panda_link0"
        target.pose.position.x = float(translation[0])
        target.pose.position.y = float(translation[1])
        target.pose.position.z = float(translation[2])
        q = _rot_to_quat(rotation)
        (
            target.pose.orientation.x,
            target.pose.orientation.y,
            target.pose.orientation.z,
            target.pose.orientation.w,
        ) = q

        arm.set_goal_state(pose_stamped_msg=target, pose_link="panda_link8")
        plan = arm.plan()
        if not plan:
            return ArmExecutionResult(False, "plan failed")
        ok = self._mp.execute(plan.trajectory, controllers=[])
        return ArmExecutionResult(bool(ok), "ok" if ok else "execution failed")


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
