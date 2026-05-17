"""NavigateToPose action server backed by the pure-pursuit control law.

Drop-in replacement for ``nav2_bt_navigator`` on ``/<ns>/navigate_to_pose``:
the fleet_coordinator's existing ActionClient connects unchanged. Use
this OR the Nav2 stack, not both — they share the action name.

Why this exists: Phase 2 M1 smoke landed in "Nav2 lifecycle active, action
accepts goals, but Nova Carter doesn't move" (see
feedback-nav2-isaac-sim-gotchas). DWB's costmap depends on the front_3d
LIDAR publisher that doesn't fire under standalone-python Isaac Sim, so
every trajectory might be scored as obstructed. Skipping Nav2's controller
entirely takes the costmap out of the picture.

Control loop (20 Hz):
    1. Look up ``map → base_link`` via tf2 buffer.
    2. Compute ``(linear_x, angular_z, distance)`` via
       ``nav_drivers.pure_pursuit.compute_cmd_vel``.
    3. Publish ``geometry_msgs/Twist`` on ``cmd_vel``.
    4. If distance <= goal_tolerance → succeed.
    5. If elapsed > goal_timeout → abort.
    6. On cancel → publish zero Twist + abort.

Pose source: the pure-Python control law lives in ``nav_drivers/`` and is
imported here. The /work and project-root sys.path tweak mirrors what
fleet_coordinator does so this import works whether ros2_ws is the cwd or
deep under /work on vast.ai.
"""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

for candidate in ("/work", str(Path(__file__).resolve().parents[4])):
    if candidate not in sys.path:
        sys.path.insert(0, candidate)

from nav_drivers.pure_pursuit import PurePursuitConfig, compute_cmd_vel  # noqa: E402


def _yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    """ZYX yaw extraction — only the heading is meaningful for diff-drive."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class PurePursuitDriver(Node):
    def __init__(self) -> None:
        super().__init__("pure_pursuit_driver")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("action_name", "navigate_to_pose")
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("goal_timeout_s", 60.0)

        self.declare_parameter("k_linear", PurePursuitConfig.k_linear)
        self.declare_parameter("k_angular", PurePursuitConfig.k_angular)
        self.declare_parameter("max_linear", PurePursuitConfig.max_linear)
        self.declare_parameter("max_angular", PurePursuitConfig.max_angular)
        self.declare_parameter("heading_gate_rad", PurePursuitConfig.heading_gate_rad)
        self.declare_parameter("goal_tolerance_m", PurePursuitConfig.goal_tolerance_m)

        gp = self.get_parameter
        self._map_frame: str = gp("map_frame").get_parameter_value().string_value
        self._base_frame: str = gp("base_frame").get_parameter_value().string_value
        cmd_vel_topic: str = gp("cmd_vel_topic").get_parameter_value().string_value
        action_name: str = gp("action_name").get_parameter_value().string_value
        self._dt: float = 1.0 / float(gp("control_rate_hz").value)
        self._goal_timeout_s: float = float(gp("goal_timeout_s").value)
        self._config = PurePursuitConfig(
            k_linear=float(gp("k_linear").value),
            k_angular=float(gp("k_angular").value),
            max_linear=float(gp("max_linear").value),
            max_angular=float(gp("max_angular").value),
            heading_gate_rad=float(gp("heading_gate_rad").value),
            goal_tolerance_m=float(gp("goal_tolerance_m").value),
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self._cb_group = ReentrantCallbackGroup()
        self._tf_miss_count = 0

        self._action_server = ActionServer(
            self,
            NavigateToPose,
            action_name,
            execute_callback=self._execute,
            goal_callback=self._on_goal_request,
            cancel_callback=self._on_cancel_request,
            callback_group=self._cb_group,
        )

        self.get_logger().info(
            f"pure_pursuit_driver ready — action /{self.get_namespace().lstrip('/')}"
            f"/{action_name.lstrip('/')} cmd_vel→{cmd_vel_topic} "
            f"frames={self._map_frame}↔{self._base_frame}"
        )

    # ---- Goal lifecycle ---------------------------------------------------

    def _on_goal_request(self, goal_request) -> GoalResponse:  # noqa: ARG002
        return GoalResponse.ACCEPT

    def _on_cancel_request(self, goal_handle) -> CancelResponse:  # noqa: ARG002
        return CancelResponse.ACCEPT

    def _lookup_pose(self) -> tuple[float, float, float] | None:
        try:
            tf = self._tf_buffer.lookup_transform(self._map_frame, self._base_frame, Time())
        except Exception as e:  # tf2 raises various LookupException subclasses
            # Throttle: log on first miss then every 40 misses (~2s @ 20Hz).
            if self._tf_miss_count % 40 == 0:
                self.get_logger().warn(
                    f"TF lookup {self._map_frame}->{self._base_frame} failed: {e}"
                )
            self._tf_miss_count += 1
            return None
        self._tf_miss_count = 0
        t = tf.transform.translation
        q = tf.transform.rotation
        return float(t.x), float(t.y), _yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def _publish(self, linear_x: float, angular_z: float) -> None:
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self._cmd_pub.publish(twist)

    def _execute(self, goal_handle):
        goal_x = float(goal_handle.request.pose.pose.position.x)
        goal_y = float(goal_handle.request.pose.pose.position.y)
        self.get_logger().info(f"goal received: ({goal_x:.2f}, {goal_y:.2f}) in {self._map_frame}")

        start_t = self.get_clock().now()
        feedback_msg = NavigateToPose.Feedback()
        result = NavigateToPose.Result()

        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self._publish(0.0, 0.0)
                    goal_handle.canceled()
                    self.get_logger().info("goal canceled")
                    return result

                elapsed_s = (self.get_clock().now() - start_t).nanoseconds * 1e-9
                if elapsed_s > self._goal_timeout_s:
                    self._publish(0.0, 0.0)
                    goal_handle.abort()
                    self.get_logger().warn(
                        f"goal aborted: timeout after {elapsed_s:.1f}s "
                        f"(>{self._goal_timeout_s:.1f}s)"
                    )
                    return result

                pose = self._lookup_pose()
                if pose is None:
                    self._publish(0.0, 0.0)
                    time.sleep(self._dt)
                    continue

                x, y, yaw = pose
                lin, ang, dist = compute_cmd_vel(x, y, yaw, goal_x, goal_y, self._config)

                if dist <= self._config.goal_tolerance_m:
                    self._publish(0.0, 0.0)
                    goal_handle.succeed()
                    self.get_logger().info(
                        f"goal SUCCEEDED at ({x:.2f}, {y:.2f}) "
                        f"(dist={dist:.3f} m, elapsed={elapsed_s:.1f}s)"
                    )
                    return result

                self._publish(lin, ang)
                feedback_msg.distance_remaining = float(dist)
                goal_handle.publish_feedback(feedback_msg)
                time.sleep(self._dt)
        finally:
            self._publish(0.0, 0.0)

        if goal_handle.status == GoalStatus.STATUS_EXECUTING:
            goal_handle.abort()
        return result


def main() -> None:
    from rclpy.executors import ExternalShutdownException

    rclpy.init()
    node = PurePursuitDriver()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok():
            try:
                executor.spin()
                break  # spin returned cleanly — shutdown signaled
            except KeyError as exc:
                # rclpy ActionServer race on goal-timeout cleanup
                # (rclpy/action/server.py:357 looks up
                # self._result_futures[goal_uuid] after the future was
                # already removed by the abort path). Verified deterministic
                # under M7 steady_state load when pp_driver hits
                # goal_timeout_s and tries to publish the ABORTED result.
                # The goal is already terminal; resuming the executor is
                # safe and keeps the driver alive for the next goal.
                node.get_logger().warn(
                    f"executor.spin raised KeyError on goal-result cleanup "
                    f"(rclpy ActionServer race, goal_uuid={exc!r}); resuming"
                )
                continue
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
