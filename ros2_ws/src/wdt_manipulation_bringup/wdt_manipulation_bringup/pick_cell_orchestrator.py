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
import tf2_ros
from cv_bridge import CvBridge
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from manipulation.grasping import TopDownGrasp, TopDownGraspFromPose
from manipulation.motion_planning import ArmPlanner
from manipulation.pipeline import ManipulationPipeline
from manipulation.pose_estimation import PoseEstimator

PLANNING_FRAME = "panda_link0"
CAMERA_OPTICAL_FRAME = "cell_cam_optical"
TF_LOOKUP_TIMEOUT_S = 2.0


def _quat_to_rotation_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """ROS geometry_msgs Quaternion → 3x3 rotation matrix (right-handed)."""
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


class _TransformedPose:
    """Lightweight pose wrapper with a `.translation` field — what
    TopDownGraspFromPose duck-types against. Used to substitute the FP-output
    pose with one re-expressed in the planning (panda_link0) frame.
    """

    def __init__(self, translation: np.ndarray) -> None:
        self.translation = translation


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
        # /cell/start_pick uses a ReentrantCallbackGroup so the callback
        # can recursively spin the MoveGroup action client (send_goal_async
        # + get_result_async) without deadlocking. Default
        # MutuallyExclusiveCallbackGroup serializes all callbacks on a node
        # — even with MultiThreadedExecutor — and the action-server's
        # goal-acceptance response can't be dispatched while the start_pick
        # callback is blocked in spin_until_future_complete. Verified
        # M5 v21: MTExecutor alone didn't fix the deadlock.
        self._start_cb_group = ReentrantCallbackGroup()
        self.create_subscription(
            String,
            "/cell/start_pick",
            self._on_start,
            1,
            callback_group=self._start_cb_group,
        )
        self._pub = self.create_publisher(String, "/cell/pick_result", 10)

        self._pose_estimator = PoseEstimator()
        self._top_down = TopDownGrasp(standoff_m=0.05)
        # Pass self as parent_node so ArmPlanner reuses this node's
        # rclpy context + executor for the MoveGroup action client.
        # Without parent_node, ArmPlanner would create its own Node and
        # double-init rclpy.
        self._arm = ArmPlanner(parent_node=self, planning_group="panda_arm")
        # TF buffer + listener for transforming FP poses from the cell
        # camera's optical frame into the Franka planning frame
        # (panda_link0). Required because FoundationPose returns 6D poses in
        # the camera frame whose origin is published by /cell/cam/info, but
        # MoveIt's MoveGroup interprets goal poses in panda_link0 (set by
        # manipulation/motion_planning.py:PLANNING_FRAME). Without this
        # transform, the synthetic-noise pose was passed straight to
        # plan_to_pose() → unreachable target → exhausted_candidates (v12).
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self.get_logger().info("pick_cell_orchestrator ready")

    def _lookup_optical_to_planning(self) -> np.ndarray | None:
        """Return a 4x4 transform matrix from cell_cam_optical → panda_link0.

        Caller blocks on tf2 for up to TF_LOOKUP_TIMEOUT_S. Returns None on
        timeout. Static TF for world → cell_cam_optical is broadcast by
        run_scenario.py; world → panda_link0 comes from Isaac Sim's
        articulation TF publisher (the Franka is a fixed-base arm so this
        is effectively a static transform).
        """
        try:
            t = self._tf_buffer.lookup_transform(
                target_frame=PLANNING_FRAME,
                source_frame=CAMERA_OPTICAL_FRAME,
                time=Time(),
                timeout=Duration(seconds=TF_LOOKUP_TIMEOUT_S),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            self.get_logger().warn(
                f"TF lookup {CAMERA_OPTICAL_FRAME}→{PLANNING_FRAME} failed: {exc}"
            )
            return None

        q = t.transform.rotation
        tr = t.transform.translation
        R = _quat_to_rotation_matrix(q.x, q.y, q.z, q.w)
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R
        T[:3, 3] = [tr.x, tr.y, tr.z]
        return T

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

        # FoundationPose returns 6D poses in the camera optical frame
        # (frame_id = cell_cam_optical, set by run_scenario.py's
        # ROS2CameraInfoHelper). MoveIt plans in panda_link0. Transform the
        # pose's translation before composing the grasp.
        T_panda_from_optical = self._lookup_optical_to_planning()
        if T_panda_from_optical is None:
            self._publish_result(order_id, False, 0, perf_counter() - t0, "tf_lookup_failed")
            return
        cam_t = np.asarray(poses[0].translation, dtype=np.float64)
        t_homo = np.array([cam_t[0], cam_t[1], cam_t[2], 1.0], dtype=np.float64)
        panda_t = (T_panda_from_optical @ t_homo)[:3]
        transformed = _TransformedPose(translation=panda_t.astype(np.float32))
        # M5 v15 diagnostic: dump the pose chain so we can verify FP +
        # TF + grasp standoff produce a Franka-reachable goal. If "Unable
        # to sample any valid states for goal tree" persists, the printed
        # panda_t shows whether FP returned a sensible cube position.
        self.get_logger().info(
            f"pose chain: fp_optical={tuple(round(float(x), 3) for x in cam_t)} "
            f"-> panda_link0={tuple(round(float(x), 3) for x in panda_t)} "
            f"(grasp = +0.05 Z standoff above this)"
        )

        grasp_gen = TopDownGraspFromPose(inner=self._top_down, pose=transformed)
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
    # MultiThreadedExecutor so the MoveGroup action-client response can
    # be processed concurrently with the /cell/start_pick subscription
    # callback. With the default SingleThreadedExecutor, plan_to_pose's
    # spin_until_future_complete(send_future) deadlocked: the callback
    # was holding the executor, so the goal-acceptance message from
    # move_group couldn't be dispatched until ACTION_TIMEOUT_S=5s fired.
    # Verified in M5 v20: move_group logged "Motion plan was computed
    # successfully" 156 ms after request, but ArmPlanner returned
    # "goal_rejected handle=None" with cycle_time matching the 5 s
    # client timeout exactly.
    from rclpy.executors import MultiThreadedExecutor

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
