"""Fleet coordinator: assigns orders via Hungarian, sends Nav2 goals, watches deadlocks.

Runs inside a ROS2 environment (on the vast.ai instance after colcon build).
At Phase 1 this is a skeleton — the NavigateToPose action server may not
exist if full Nav2 isn't wired up yet (see Task 26 simplification). When
that's the case the node logs warnings and the orders queue up; the
underlying Hungarian + deadlock logic is exercised regardless. Task 33
adds the TF listener for live pose updates.
"""

from __future__ import annotations

import sys
import threading
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

# Make the coordinator package importable regardless of which path the
# ROS2 entry point launches from. /work/ is where we untar ros2_ws on
# vast.ai; the project root is a fallback for local dev.
for candidate in ("/work", str(Path(__file__).resolve().parents[4])):
    if candidate not in sys.path:
        sys.path.insert(0, candidate)

from coordinator.assignment import hungarian_assign  # noqa: E402
from coordinator.deadlock import DeadlockMonitor  # noqa: E402


class FleetCoordinator(Node):
    def __init__(self):
        super().__init__("fleet_coordinator")
        self.declare_parameter("amr_ids", ["amr_0"])
        amr_ids: list[str] = self.get_parameter("amr_ids").value

        self.amr_ids = amr_ids
        self._poses: dict[str, tuple[float, float]] = {a: (0.0, 0.0) for a in amr_ids}
        self._busy: dict[str, bool] = {a: False for a in amr_ids}
        self._orders: list[tuple[str, tuple[float, float]]] = []
        self._lock = threading.Lock()
        self._deadlock = DeadlockMonitor(idle_radius_m=1.0, idle_secs=5.0)

        self._clients: dict[str, ActionClient] = {
            a: ActionClient(self, NavigateToPose, f"/{a}/navigate_to_pose") for a in amr_ids
        }
        self.create_subscription(PoseStamped, "/orders/enqueue", self._on_order, 10)
        self.create_timer(1.0, self._tick)

        self.get_logger().info(f"fleet_coordinator up — managing {len(amr_ids)} AMRs: {amr_ids}")

    def _on_order(self, msg: PoseStamped) -> None:
        oid = msg.header.frame_id or f"order_{len(self._orders)}"
        with self._lock:
            self._orders.append((oid, (msg.pose.position.x, msg.pose.position.y)))
        self.get_logger().info(
            f"enqueued order {oid} at ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})"
        )

    def _tick(self) -> None:
        # NOTE: pose updates come from the TF listener (Task 33). Until then
        # _poses stays at (0,0) for every AMR which is fine for exercising
        # the assignment + deadlock paths but not for real navigation.
        t = self.get_clock().now().nanoseconds * 1e-9
        self._deadlock.tick(t, self._poses)
        if self._deadlock.deadlocked():
            self.get_logger().warn(f"DEADLOCK detected: {self._deadlock.deadlocked()}")

        with self._lock:
            free_robots = {a: self._poses[a] for a in self.amr_ids if not self._busy[a]}
            if not free_robots or not self._orders:
                return
            assignment = hungarian_assign(free_robots, self._orders)

        for robot_id, order_id in assignment.items():
            order = next(o for o in self._orders if o[0] == order_id)
            self._send_goal(robot_id, order[1])
            self._busy[robot_id] = True
            self._orders.remove(order)

    def _send_goal(self, robot_id: str, xy: tuple[float, float]) -> None:
        client = self._clients[robot_id]
        if not client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(
                f"{robot_id} NavigateToPose action server not ready — Nav2 not running?"
            )
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x = xy[0]
        goal.pose.pose.position.y = xy[1]
        goal.pose.pose.orientation.w = 1.0
        send = client.send_goal_async(goal)
        send.add_done_callback(lambda f, rid=robot_id: self._on_done(rid, f))

    def _on_done(self, robot_id: str, _future) -> None:
        self._busy[robot_id] = False


def main() -> None:
    rclpy.init()
    rclpy.spin(FleetCoordinator())
    rclpy.shutdown()
