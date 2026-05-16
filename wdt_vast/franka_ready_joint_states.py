"""Continuously publish the Franka Panda 'ready' joint pose on /joint_states.

run_scenario.py launches this as a subprocess so MoveIt's planning_scene
monitor has a valid (collision-free, non-singular) start state. The Panda's
URDF zero-position has panda_link5 and panda_link7 in self-collision, so
without this MoveIt rejects every plan with "Skipping invalid start state"
and the orchestrator returns exhausted_candidates. Hit in M5 v13 — same
gotcha as moveit_plan_smoke.py (M3 smoke).

Once Isaac Sim's Franka articulation publishes its own live /joint_states
(M5b / Phase 3), this static publisher should be removed.

Invocation (matches the wdt_vast/synthetic_* pattern):
    /usr/bin/python3 wdt_vast/franka_ready_joint_states.py
"""

from __future__ import annotations

import math
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

PANDA_READY = {
    "panda_joint1": 0.0,
    "panda_joint2": -math.pi / 4,
    "panda_joint3": 0.0,
    "panda_joint4": -3 * math.pi / 4,
    "panda_joint5": 0.0,
    "panda_joint6": math.pi / 2,
    "panda_joint7": math.pi / 4,
    "panda_finger_joint1": 0.04,
    "panda_finger_joint2": 0.04,
}
PUBLISH_RATE_HZ = 10.0


class FrankaReadyJointStates(Node):
    def __init__(self) -> None:
        super().__init__("franka_ready_joint_states")
        self._names = list(PANDA_READY.keys())
        self._positions = [PANDA_READY[n] for n in self._names]
        self._pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_timer(1.0 / PUBLISH_RATE_HZ, self._tick)
        self.get_logger().info(
            f"franka_ready_joint_states publishing {len(self._names)} joints at "
            f"{PUBLISH_RATE_HZ:.1f} Hz on /joint_states"
        )

    def _tick(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self._names
        msg.position = self._positions
        self._pub.publish(msg)


def main() -> int:
    rclpy.init()
    node = FrankaReadyJointStates()
    try:
        rclpy.spin(node)
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
