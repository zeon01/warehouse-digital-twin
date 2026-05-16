"""Mock cell-cam publisher for the M5 end-to-end pick smoke.

Publishes synthetic RGB / depth / CameraInfo at 10 Hz on:
    /cell/cam/rgb    sensor_msgs/Image  (480x640 rgb8)
    /cell/cam/depth  sensor_msgs/Image  (480x640 32FC1, meters)
    /cell/cam/info   sensor_msgs/CameraInfo

The depth image has an 8 cm cube centered at ~50 cm from the camera —
matches `wdt_vast/m5_smoke_box.obj` (a trimesh.creation.box exported by
run_scenario.py at startup). FoundationPose can register against this
pair and the orchestrator's pipeline.pick() returns success on plan.

Use until Isaac Sim Camera + ROS2CameraHelper plumbing is wired in
(M5b / Phase 3). The pose-estimator + grasp + MoveIt chain is the same
in either case; the only thing that changes is the source of RGB-D.

Invoke standalone (mostly for debug):
    /usr/bin/python3 wdt_vast/synthetic_cell_camera.py
Or via run_scenario.py which launches it as a subprocess.
"""

from __future__ import annotations

import sys

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header

# Camera intrinsics — typical realsense-style depth cam.
H, W = 480, 640
FX = FY = 600.0
CX, CY = 320.0, 240.0
# Cube extents (m) — matches the trimesh.creation.box(extents=(0.08, 0.08, 0.08))
# that run_scenario.py exports.
CUBE_EDGE_M = 0.08
CUBE_Z_M = 0.5  # depth to cube center
FLOOR_Z_M = 1.0  # background depth (planar floor)
FRAME_ID = "panda_link0"
PUBLISH_RATE_HZ = 10.0


def _build_synthetic_frame() -> tuple[np.ndarray, np.ndarray]:
    """Return (rgb, depth) — 480x640 rgb8 + 480x640 float32 depth in meters.

    Constant per frame so subscribers see a stable scene. The cube
    projects to ~96 px wide at z=0.5 m with fx=600 (size_px = fx * size_m / z).
    """
    depth = np.full((H, W), FLOOR_Z_M, dtype=np.float32)
    rgb = np.full((H, W, 3), 100, dtype=np.uint8)
    cube_px = int(FX * CUBE_EDGE_M / CUBE_Z_M)
    half = cube_px // 2
    cy_px, cx_px = H // 2, W // 2
    depth[cy_px - half : cy_px + half, cx_px - half : cx_px + half] = CUBE_Z_M
    rgb[cy_px - half : cy_px + half, cx_px - half : cx_px + half] = (200, 80, 50)
    return rgb, depth


class SyntheticCellCamera(Node):
    def __init__(self) -> None:
        super().__init__("synthetic_cell_camera")
        self._rgb_pub = self.create_publisher(Image, "/cell/cam/rgb", 1)
        self._depth_pub = self.create_publisher(Image, "/cell/cam/depth", 1)
        self._info_pub = self.create_publisher(CameraInfo, "/cell/cam/info", 1)

        rgb, depth = _build_synthetic_frame()
        self._rgb_bytes = rgb.tobytes()
        self._depth_bytes = depth.tobytes()

        self.create_timer(1.0 / PUBLISH_RATE_HZ, self._tick)
        self.get_logger().info(
            f"synthetic_cell_camera publishing {H}x{W} rgb+depth at "
            f"{PUBLISH_RATE_HZ:.1f} Hz on /cell/cam/{{rgb,depth,info}}"
        )

    def _header(self) -> Header:
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = FRAME_ID
        return h

    def _tick(self) -> None:
        rgb_msg = Image()
        rgb_msg.header = self._header()
        rgb_msg.height = H
        rgb_msg.width = W
        rgb_msg.encoding = "rgb8"
        rgb_msg.is_bigendian = 0
        rgb_msg.step = W * 3
        rgb_msg.data = self._rgb_bytes
        self._rgb_pub.publish(rgb_msg)

        depth_msg = Image()
        depth_msg.header = self._header()
        depth_msg.height = H
        depth_msg.width = W
        depth_msg.encoding = "32FC1"
        depth_msg.is_bigendian = 0
        depth_msg.step = W * 4  # float32
        depth_msg.data = self._depth_bytes
        self._depth_pub.publish(depth_msg)

        info = CameraInfo()
        info.header = self._header()
        info.height = H
        info.width = W
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [FX, 0.0, CX, 0.0, 0.0, FY, CY, 0.0, 0.0, 0.0, 1.0, 0.0]
        self._info_pub.publish(info)


def main() -> int:
    rclpy.init()
    node = SyntheticCellCamera()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
