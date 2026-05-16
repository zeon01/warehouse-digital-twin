"""Publish the pick cube's worldspace pose on /world/cube_pose at 10 Hz.

Runs inside the Isaac Sim kit process (python 3.11) since it needs USD
stage access. The orchestrator subscribes to this topic when running in
``pose_source=gt`` mode, completely bypassing FoundationPose for the M5
acceptance loop.

Invocation (from run_scenario.py):
    /isaac-sim/python.sh wdt_vast/sim_world_pose_publisher.py \\
        --cube-prim-path /World/pick_cube
"""

from __future__ import annotations

import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

PUBLISH_RATE_HZ = 10.0


def _quaternion_from_matrix(m) -> tuple[float, float, float, float]:
    """3x3 rotation matrix (USD Gf.Matrix3d or numpy) → (qx, qy, qz, qw)."""

    # Tolerate both pxr.Gf.Matrix3d and numpy.ndarray
    def _r(i, j):
        try:
            return float(m[i][j])
        except TypeError:
            return float(m[i, j])

    tr = _r(0, 0) + _r(1, 1) + _r(2, 2)
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        qw = 0.25 * s
        qx = (_r(2, 1) - _r(1, 2)) / s
        qy = (_r(0, 2) - _r(2, 0)) / s
        qz = (_r(1, 0) - _r(0, 1)) / s
    else:
        # Branchy fallback — pick the largest diagonal.
        if _r(0, 0) > _r(1, 1) and _r(0, 0) > _r(2, 2):
            s = math.sqrt(1.0 + _r(0, 0) - _r(1, 1) - _r(2, 2)) * 2
            qw = (_r(2, 1) - _r(1, 2)) / s
            qx = 0.25 * s
            qy = (_r(0, 1) + _r(1, 0)) / s
            qz = (_r(0, 2) + _r(2, 0)) / s
        elif _r(1, 1) > _r(2, 2):
            s = math.sqrt(1.0 + _r(1, 1) - _r(0, 0) - _r(2, 2)) * 2
            qw = (_r(0, 2) - _r(2, 0)) / s
            qx = (_r(0, 1) + _r(1, 0)) / s
            qy = 0.25 * s
            qz = (_r(1, 2) + _r(2, 1)) / s
        else:
            s = math.sqrt(1.0 + _r(2, 2) - _r(0, 0) - _r(1, 1)) * 2
            qw = (_r(1, 0) - _r(0, 1)) / s
            qx = (_r(0, 2) + _r(2, 0)) / s
            qy = (_r(1, 2) + _r(2, 1)) / s
            qz = 0.25 * s
    return (qx, qy, qz, qw)


class WorldCubePosePublisher(Node):
    def __init__(self, cube_prim_path: str) -> None:
        super().__init__("sim_world_cube_pose")
        self._cube_prim_path = cube_prim_path
        self._pub = self.create_publisher(PoseStamped, "/world/cube_pose", 10)
        self._timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self._tick)
        self.get_logger().info(
            f"sim_world_cube_pose publishing /world/cube_pose from {cube_prim_path} "
            f"at {PUBLISH_RATE_HZ:.1f} Hz"
        )

    def _tick(self) -> None:
        import omni.usd
        from pxr import UsdGeom

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return
        prim = stage.GetPrimAtPath(self._cube_prim_path)
        if not prim or not prim.IsValid():
            return
        xform = UsdGeom.Xformable(prim)
        # ComputeLocalToWorldTransform returns the prim's world transform
        # as a Gf.Matrix4d. Default time = 0 is fine; the cube isn't
        # animated in the smoke (FixedCuboid; DynamicCuboid may drift
        # slightly under physics but world pose is still current).
        world_xform = xform.ComputeLocalToWorldTransform(0.0)
        translation = world_xform.ExtractTranslation()
        rotation = world_xform.ExtractRotationMatrix()

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x = float(translation[0])
        msg.pose.position.y = float(translation[1])
        msg.pose.position.z = float(translation[2])
        qx, qy, qz, qw = _quaternion_from_matrix(rotation)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self._pub.publish(msg)


def main() -> int:
    parser = argparse.ArgumentParser(prog="sim_world_pose_publisher")
    parser.add_argument(
        "--cube-prim-path",
        default="/World/pick_cube",
        help="USD prim path of the cube whose world pose is published.",
    )
    args = parser.parse_args()

    rclpy.init()
    node = WorldCubePosePublisher(cube_prim_path=args.cube_prim_path)
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
