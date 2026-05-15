"""ROS2 node that runs the manipulation pipeline at the pick cell.

Subscribes:
    /cell/start_pick (std_msgs/String) — payload = order_id
    /cell/cam/rgb    (sensor_msgs/Image)
    /cell/cam/depth  (sensor_msgs/Image)
    /cell/cam/info   (sensor_msgs/CameraInfo)

Publishes:
    /cell/pick_result (std_msgs/String) — JSON payload:
        {"order_id": "...", "success": bool, "attempts": int,
         "cycle_time_s": float, "failure_reason": "..."}

On every /cell/start_pick:
    1. Snapshot the latest RGB-D + intrinsics.
    2. Run FoundationPose → estimate object pose.
    3. Bind a TopDownGrasp via TopDownGraspFromPose.
    4. Run ManipulationPipeline.pick() — which calls the bound grasp
       generator, then plans + executes with MoveIt2's ArmPlanner.
    5. Publish /cell/pick_result with the PickResult fields.

The CAD path defaults to FoundationPose's bundled mustard demo asset
(installed at /opt/foundationpose/src/demo_data/mustard0/) so the
M5 smoke can run end-to-end without bespoke warehouse SKU meshes.
Override per-instance via the `cad_path` ROS2 parameter.
"""

from __future__ import annotations

import json
import threading
from time import perf_counter

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from manipulation.grasping import TopDownGrasp, TopDownGraspFromPose
from manipulation.motion_planning import ArmPlanner
from manipulation.pipeline import ManipulationPipeline
from manipulation.pose_estimation import PoseEstimator


class _PrecomputedPose:
    """ManipulationPipeline expects ``pose_estimator.estimate(...)`` to
    return poses; we've already run estimation here so wrap the result
    in something that satisfies the duck-typed interface without
    re-running the (expensive) FoundationPose inference.
    """

    def __init__(self, poses):
        self._poses = poses

    def estimate(self, **kwargs):  # noqa: ARG002 — kwargs ignored on purpose
        return self._poses


class PickCellOrchestrator(Node):
    def __init__(self) -> None:
        super().__init__("pick_cell_orchestrator")
        self.declare_parameter(
            "cad_path",
            "/opt/foundationpose/src/demo_data/mustard0/mesh/textured_simple.obj",
        )
        self._cad_path = self.get_parameter("cad_path").get_parameter_value().string_value

        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._latest_rgb: np.ndarray | None = None
        self._latest_depth: np.ndarray | None = None
        self._latest_K: np.ndarray | None = None

        self.create_subscription(Image, "/cell/cam/rgb", self._on_rgb, 1)
        self.create_subscription(Image, "/cell/cam/depth", self._on_depth, 1)
        self.create_subscription(CameraInfo, "/cell/cam/info", self._on_info, 1)
        self.create_subscription(String, "/cell/start_pick", self._on_start, 1)
        self._pub = self.create_publisher(String, "/cell/pick_result", 10)

        self._pose_estimator = PoseEstimator()
        self._top_down = TopDownGrasp(standoff_m=0.05)
        self._arm = ArmPlanner(planning_group="panda_arm")
        self.get_logger().info("pick_cell_orchestrator ready")

    def _on_rgb(self, msg: Image) -> None:
        with self._lock:
            self._latest_rgb = self._bridge.imgmsg_to_cv2(msg, "rgb8")

    def _on_depth(self, msg: Image) -> None:
        with self._lock:
            self._latest_depth = self._bridge.imgmsg_to_cv2(msg, "32FC1")

    def _on_info(self, msg: CameraInfo) -> None:
        with self._lock:
            self._latest_K = np.array(msg.k).reshape(3, 3)

    def _on_start(self, msg: String) -> None:
        order_id = msg.data
        self.get_logger().info(f"start_pick received: {order_id}")
        with self._lock:
            rgb = self._latest_rgb
            depth = self._latest_depth
            K = self._latest_K
        if rgb is None or depth is None or K is None:
            self._publish_result(order_id, False, 0, 0.0, "no_cam_data")
            return

        t0 = perf_counter()
        poses = self._pose_estimator.estimate(
            rgb=rgb, depth=depth, cad_path=self._cad_path, camera_K=K
        )
        if not poses:
            self._publish_result(order_id, False, 0, perf_counter() - t0, "no_pose")
            return

        grasp_gen = TopDownGraspFromPose(inner=self._top_down, pose=poses[0])
        pipeline = ManipulationPipeline(
            pose_estimator=_PrecomputedPose(poses),
            grasp_generator=grasp_gen,
            arm=self._arm,
        )
        result = pipeline.pick(rgb=rgb, depth=depth, cad_path=self._cad_path, camera_K=K)
        self._publish_result(
            order_id,
            result.success,
            result.attempts,
            result.cycle_time_s,
            result.failure_reason,
        )

    def _publish_result(
        self,
        order_id: str,
        success: bool,
        attempts: int,
        cycle_time_s: float,
        reason: str,
    ) -> None:
        msg = String()
        msg.data = json.dumps(
            {
                "order_id": order_id,
                "success": success,
                "attempts": attempts,
                "cycle_time_s": cycle_time_s,
                "failure_reason": reason,
            }
        )
        self._pub.publish(msg)
        self.get_logger().info(f"pick_result: {msg.data}")


def main() -> None:
    rclpy.init()
    node = PickCellOrchestrator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
