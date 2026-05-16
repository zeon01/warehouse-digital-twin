"""Publish a static /world/cube_pose at a configurable rate.

Originally designed to run inside Isaac Sim's kit python (3.11) and read
the cube's USD prim transform live. That design hit gotcha #18 (kit
python lacks Humble's cpython-310 rclpy). Refactored to a system-python
(3.10) static publisher: run_scenario.py passes the cube's spawn coords
as CLI args. M5 smoke uses ``plan_only=True`` so MoveIt never executes
the grasp — slight physics drift on a resting DynamicCuboid is harmless.

Invocation (from run_scenario.py):
    /usr/bin/python3 wdt_vast/sim_world_pose_publisher.py \\
        --x 16.40 --y 15.0 --z 0.75 --frame-id world

GroundTruthPoseSource on the orchestrator subscribes to /world/cube_pose
and feeds the latest position into the PickWorker without any
FoundationPose call.
"""

from __future__ import annotations

import argparse
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class StaticWorldCubePosePublisher(Node):
    def __init__(  # noqa: PLR0913
        self,
        x: float,
        y: float,
        z: float,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
        frame_id: str,
        rate_hz: float,
    ) -> None:
        super().__init__("sim_world_cube_pose")
        self._x = x
        self._y = y
        self._z = z
        self._qx = qx
        self._qy = qy
        self._qz = qz
        self._qw = qw
        self._frame_id = frame_id
        self._pub = self.create_publisher(PoseStamped, "/world/cube_pose", 10)
        self._timer = self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(
            f"sim_world_cube_pose publishing /world/cube_pose at "
            f"({x:.3f}, {y:.3f}, {z:.3f}) frame={frame_id} rate={rate_hz:.1f} Hz"
        )

    def _tick(self) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.pose.position.x = self._x
        msg.pose.position.y = self._y
        msg.pose.position.z = self._z
        msg.pose.orientation.x = self._qx
        msg.pose.orientation.y = self._qy
        msg.pose.orientation.z = self._qz
        msg.pose.orientation.w = self._qw
        self._pub.publish(msg)


def main() -> int:
    parser = argparse.ArgumentParser(prog="sim_world_pose_publisher")
    parser.add_argument("--x", type=float, default=16.40)
    parser.add_argument("--y", type=float, default=15.00)
    parser.add_argument("--z", type=float, default=0.75)
    parser.add_argument("--qx", type=float, default=0.0)
    parser.add_argument("--qy", type=float, default=0.0)
    parser.add_argument("--qz", type=float, default=0.0)
    parser.add_argument("--qw", type=float, default=1.0)
    parser.add_argument("--frame-id", default="world")
    parser.add_argument("--rate", type=float, default=10.0)
    args = parser.parse_args()

    rclpy.init()
    node = StaticWorldCubePosePublisher(
        x=args.x,
        y=args.y,
        z=args.z,
        qx=args.qx,
        qy=args.qy,
        qz=args.qz,
        qw=args.qw,
        frame_id=args.frame_id,
        rate_hz=args.rate,
    )
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
