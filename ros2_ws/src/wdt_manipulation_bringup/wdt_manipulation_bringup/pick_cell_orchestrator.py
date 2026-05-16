"""Thin rclpy node that hosts the M5 pick chain via a PickWorker thread.

The chronic v11–v21 bug was running synchronous MoveIt action calls
inside subscription callbacks → rclpy executor deadlock. This rewrite
keeps callbacks tiny (cache state, enqueue a request) and runs all
heavy lifting on a worker thread that owns its own rclpy Node +
Executor for the MoveGroup action client. See
``docs/superpowers/specs/2026-05-16-pick-chain-redesign-design.md``.

ROS2 parameters:
- ``cad_path`` (string): path to the FoundationPose CAD .obj.
- ``pose_source`` (string): ``"fp"`` (default) or ``"gt"``.

Subscribes:
- ``/cell/cam/{rgb,depth,info}``: cache latest cam frame (FP mode).
- ``/world/cube_pose`` (PoseStamped): GT mode pose feed.
- ``/cell/start_pick`` (String): trigger one pick by ``order_id``.

Publishes:
- ``/cell/pick_result`` (String): JSON-encoded ``PickResult``.
"""

from __future__ import annotations

import json
import threading
from typing import Any

import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from manipulation.motion_planning import ArmPlanner
from manipulation.pick_worker import PickRequest, PickResult, PickWorker
from manipulation.pose_estimation import PoseEstimator
from manipulation.pose_source import FoundationPosePoseSource, GroundTruthPoseSource

PLANNING_FRAME = "panda_link0"
TF_LOOKUP_TIMEOUT_S = 2.0


def _quat_to_rotation_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    n = qx * qx + qy * qy + qz * qz + qw * qw
    if n < 1e-12:
        return np.eye(3, dtype=np.float64)
    s = 2.0 / n
    return np.array(
        [
            [1 - s * (qy * qy + qz * qz), s * (qx * qy - qz * qw), s * (qx * qz + qy * qw)],
            [s * (qx * qy + qz * qw), 1 - s * (qx * qx + qz * qz), s * (qy * qz - qx * qw)],
            [s * (qx * qz - qy * qw), s * (qy * qz + qx * qw), 1 - s * (qx * qx + qy * qy)],
        ],
        dtype=np.float64,
    )


class PickCellOrchestrator(Node):
    def __init__(self) -> None:
        super().__init__("pick_cell_orchestrator")
        self.declare_parameter(
            "cad_path",
            "/opt/foundationpose/src/demo_data/mustard0/mesh/textured_simple.obj",
        )
        self.declare_parameter("pose_source", "fp")
        self._cad_path = self.get_parameter("cad_path").get_parameter_value().string_value
        pose_source_kind = self.get_parameter("pose_source").get_parameter_value().string_value

        # Latest cam state, accessed by both main thread (writers) and
        # worker (reader at request time). Protected by a lock.
        self._bridge = CvBridge()
        self._cam_lock = threading.Lock()
        self._latest_rgb: np.ndarray | None = None
        self._latest_depth: np.ndarray | None = None
        self._latest_K: np.ndarray | None = None

        # tf2 — owned by main thread; the worker calls the cached lookup
        # function below.
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._tf_cache: dict[str, np.ndarray] = {}

        # Build the pose source per the parameter.
        if pose_source_kind == "gt":
            self._gt_source = GroundTruthPoseSource()
            self._pose_source: Any = self._gt_source
            self.create_subscription(PoseStamped, "/world/cube_pose", self._on_cube_pose, 10)
            self.get_logger().info("pose_source=gt — subscribing to /world/cube_pose")
        else:
            self._gt_source = None
            self._pose_source = FoundationPosePoseSource(
                estimator=PoseEstimator(), frame_id="cell_cam_optical"
            )
            self.get_logger().info("pose_source=fp — FoundationPose live")

        # Camera subs (always wired so FP mode can switch in without relaunch).
        self.create_subscription(Image, "/cell/cam/rgb", self._on_rgb, 1)
        self.create_subscription(Image, "/cell/cam/depth", self._on_depth, 1)
        self.create_subscription(CameraInfo, "/cell/cam/info", self._on_info, 1)

        self._pub = self.create_publisher(String, "/cell/pick_result", 10)

        # Worker: separate rclpy Node + SingleThreadedExecutor so the
        # MoveGroup action client's spin_until_future_complete doesn't
        # race the main-thread spin. See pick_worker.py docstring.
        self._worker_node = rclpy.create_node("pick_worker_arm")
        self._worker_executor = SingleThreadedExecutor()
        self._worker_executor.add_node(self._worker_node)
        self._arm = ArmPlanner(
            parent_node=self._worker_node,
            planning_group="panda_arm",
            executor=self._worker_executor,
        )
        self._worker = PickWorker(
            pose_source=self._pose_source,
            arm_planner=self._arm,
            publish_result=self._publish_pick_result,
            tf_lookup=self._lookup_to_planning,
            cad_path=self._cad_path,
        )
        self._worker.start()

        # Start last so we don't enqueue requests before the worker is up.
        self.create_subscription(String, "/cell/start_pick", self._on_start, 1)
        self.get_logger().info("pick_cell_orchestrator ready")

    # --- main-thread callbacks ---

    def _on_rgb(self, msg: Image) -> None:
        rgb = self._bridge.imgmsg_to_cv2(msg, "rgb8")
        with self._cam_lock:
            self._latest_rgb = rgb

    def _on_depth(self, msg: Image) -> None:
        depth = self._bridge.imgmsg_to_cv2(msg, "32FC1")
        with self._cam_lock:
            self._latest_depth = depth

    def _on_info(self, msg: CameraInfo) -> None:
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        with self._cam_lock:
            self._latest_K = K

    def _on_cube_pose(self, msg: PoseStamped) -> None:
        if self._gt_source is None:
            return
        t = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            dtype=np.float64,
        )
        frame_id = msg.header.frame_id or "world"
        self._gt_source.set_latest(t, frame_id)

    def _on_start(self, msg: String) -> None:
        order_id = msg.data
        self.get_logger().info(f"start_pick received: {order_id}")
        with self._cam_lock:
            rgb = self._latest_rgb
            depth = self._latest_depth
            K = self._latest_K
        # Even in gt mode we forward cam state — pose_source.get_pose
        # ignores it. The tiny cost (3 references) keeps the request
        # shape uniform.
        self._worker.enqueue(PickRequest(order_id=order_id, rgb=rgb, depth=depth, camera_K=K))

    # --- shared helpers ---

    def _lookup_to_planning(self, source_frame: str) -> np.ndarray | None:
        if source_frame == PLANNING_FRAME:
            return np.eye(4, dtype=np.float64)
        cached = self._tf_cache.get(source_frame)
        if cached is not None:
            return cached
        try:
            t = self._tf_buffer.lookup_transform(
                target_frame=PLANNING_FRAME,
                source_frame=source_frame,
                time=Time(),
                timeout=Duration(seconds=TF_LOOKUP_TIMEOUT_S),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            self.get_logger().warn(f"TF lookup {source_frame}→{PLANNING_FRAME} failed: {exc}")
            return None
        q = t.transform.rotation
        tr = t.transform.translation
        R = _quat_to_rotation_matrix(q.x, q.y, q.z, q.w)
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R
        T[:3, 3] = [tr.x, tr.y, tr.z]
        self._tf_cache[source_frame] = T
        return T

    def _publish_pick_result(self, result: PickResult) -> None:
        msg = String()
        msg.data = json.dumps(
            {
                "order_id": result.order_id,
                "success": result.success,
                "attempts": result.attempts,
                "cycle_time_s": result.cycle_time_s,
                "failure_reason": result.failure_reason,
            }
        )
        self._pub.publish(msg)
        self.get_logger().info(f"pick_result: {msg.data}")


def _spin_worker_executor(executor: SingleThreadedExecutor) -> None:
    """Background thread target: spin the worker's executor.

    plan_to_pose calls spin_until_future_complete(..., executor=this).
    That spin-until-future call drives the executor itself, so we don't
    need a permanent spinner. BUT — between plan calls the executor must
    still process incoming action-feedback / cancellations, so spin in
    the background with a short timeout. Pattern from Karelics' writeup.
    """
    while rclpy.ok():
        executor.spin_once(timeout_sec=0.1)


def main() -> None:
    rclpy.init()
    node = PickCellOrchestrator()

    # Spin the worker's executor in its own thread so non-plan callbacks
    # on the worker node are still processed.
    worker_spin_thread = threading.Thread(
        target=_spin_worker_executor, args=(node._worker_executor,), daemon=True
    )
    worker_spin_thread.start()

    try:
        rclpy.spin(node)
    finally:
        node._worker.stop()
        node.destroy_node()
        rclpy.shutdown()
