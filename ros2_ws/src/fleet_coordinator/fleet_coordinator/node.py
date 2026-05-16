"""Fleet coordinator with per-order state machine + real Nav2 result handling.

Phase 1 ran AMRs to the shelf and marked them free as soon as Nav2
*accepted* the goal (the action server didn't exist, so the goal-handle
future was the only signal available). Phase 2 wires the full action
result chain — accepted → completed — and adds a state machine per
order so the AMR navigates to the shelf, then to the pick cell, then
waits for ``/cell/pick_result`` from the manipulation orchestrator
before marking the order complete.

Order lifecycle:
    PENDING                  — enqueued, not yet assigned
    NAV_TO_SHELF             — Nav2 driving AMR to shelf_xy
    AT_SHELF (transient)     — magic-attach happens sim-side here
    NAV_TO_CELL              — Nav2 driving AMR to pick_cell_xy
    AT_CELL (transient)      — /cell/start_pick published
    PICKING                  — orchestrator running pipeline.pick()
    COMPLETED                — terminal: success
    FAILED                   — terminal: nav failure OR pick failure

The pick cell location is declared as a ROS2 parameter (``pick_cell_xy``)
so launch files can pass it through from the scenario / layout.
"""

from __future__ import annotations

import json
import sys
import threading
from dataclasses import dataclass
from enum import Enum, auto
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

for candidate in ("/work", str(Path(__file__).resolve().parents[4])):
    if candidate not in sys.path:
        sys.path.insert(0, candidate)

from coordinator.assignment import hungarian_assign  # noqa: E402
from coordinator.deadlock import DeadlockMonitor  # noqa: E402


class OrderState(Enum):
    PENDING = auto()
    NAV_TO_SHELF = auto()
    NAV_TO_CELL = auto()
    PICKING = auto()
    COMPLETED = auto()
    FAILED = auto()


@dataclass
class Order:
    id: str
    shelf_xy: tuple[float, float]
    state: OrderState = OrderState.PENDING
    robot_id: str | None = None
    failure_reason: str = ""
    attempts: int = 0


@dataclass
class _RobotState:
    nav_client: ActionClient
    active_goal_handle: object | None = None
    # If set, this lambda is invoked when the current Nav2 goal completes
    # successfully. Phase 2 chains shelf → cell navigation by setting this
    # to a closure that fires the next NavigateToPose.
    on_success: object | None = None
    free: bool = True


class FleetCoordinator(Node):
    def __init__(self) -> None:
        super().__init__("fleet_coordinator")
        self.declare_parameter("amr_ids", ["amr_0"])
        self.declare_parameter("pick_cell_xy", [16.0, 15.0])

        amr_ids: list[str] = list(self.get_parameter("amr_ids").value)
        pick_xy_param = list(self.get_parameter("pick_cell_xy").value)
        self._pick_cell_xy: tuple[float, float] = (
            float(pick_xy_param[0]),
            float(pick_xy_param[1]),
        )

        self.amr_ids = amr_ids
        self._poses: dict[str, tuple[float, float]] = {a: (0.0, 0.0) for a in amr_ids}
        self._robots: dict[str, _RobotState] = {
            a: _RobotState(
                nav_client=ActionClient(self, NavigateToPose, f"/{a}/navigate_to_pose"),
            )
            for a in amr_ids
        }
        self._pending: list[Order] = []
        self._active: dict[str, Order] = {}  # robot_id → Order
        self._completed: list[Order] = []
        self._lock = threading.Lock()
        self._deadlock = DeadlockMonitor(idle_radius_m=1.0, idle_secs=5.0)

        self.create_subscription(PoseStamped, "/orders/enqueue", self._on_order, 10)
        self._start_pick_pub = self.create_publisher(String, "/cell/start_pick", 10)
        self.create_subscription(String, "/cell/pick_result", self._on_pick_result, 10)
        self.create_timer(1.0, self._tick)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.get_logger().info(
            f"fleet_coordinator up — {len(amr_ids)} AMRs, "
            f"pick_cell at ({self._pick_cell_xy[0]:.2f}, {self._pick_cell_xy[1]:.2f})"
        )

    # ---- Order intake ------------------------------------------------

    def _on_order(self, msg: PoseStamped) -> None:
        oid = msg.header.frame_id or f"order_{len(self._pending) + len(self._completed)}"
        order = Order(id=oid, shelf_xy=(msg.pose.position.x, msg.pose.position.y))
        with self._lock:
            self._pending.append(order)
        self.get_logger().info(
            f"enqueued order {oid} at " f"({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})"
        )

    # ---- Pose tracking ----------------------------------------------

    def _refresh_poses(self) -> None:
        for a in self.amr_ids:
            try:
                tf = self._tf_buffer.lookup_transform("map", f"{a}/base_link", rclpy.time.Time())
            except Exception:
                continue
            self._poses[a] = (tf.transform.translation.x, tf.transform.translation.y)

    # ---- Main tick ---------------------------------------------------

    def _tick(self) -> None:
        self._refresh_poses()
        t = self.get_clock().now().nanoseconds * 1e-9
        self._deadlock.tick(t, self._poses)
        if self._deadlock.deadlocked():
            # Pre-existing bug: this fires false-positives because
            # `_refresh_poses` does `lookup_transform("map", "amr_X/base_link")`
            # on the GLOBAL tf2 buffer, but the AMR TFs are published under
            # the per-AMR /amr_X/tf topic (gotcha #13). All robots stay
            # pinned at (0,0) → DeadlockMonitor flags every tick after 5 s.
            # Suppress to keep logs readable until Option 3 (subscribe to
            # /amr_X/odom) lands. See docs/m5-expert-response.md §Q11.
            if not getattr(self, "_deadlock_warned_once", False):
                self.get_logger().warn(
                    f"DEADLOCK detected: {self._deadlock.deadlocked()} "
                    "(suppressing repeats — known false positive from broken TF lookup)"
                )
                self._deadlock_warned_once = True

        with self._lock:
            free_robots = {a: self._poses[a] for a in self.amr_ids if self._robots[a].free}
            if not free_robots or not self._pending:
                return
            shelf_xys = [(o.id, o.shelf_xy) for o in self._pending]
            assignment = hungarian_assign(free_robots, shelf_xys)
            kicked: list[tuple[str, Order]] = []
            for robot_id, order_id in assignment.items():
                order = next(o for o in self._pending if o.id == order_id)
                self._pending.remove(order)
                order.robot_id = robot_id
                order.state = OrderState.NAV_TO_SHELF
                self._active[robot_id] = order
                self._robots[robot_id].free = False
                kicked.append((robot_id, order))

        # Fire goals outside the lock — send_goal_async can callback before we'd
        # release otherwise.
        for robot_id, order in kicked:
            self._send_nav_goal(
                robot_id,
                order.shelf_xy,
                on_success=lambda rid=robot_id: self._on_arrived_at_shelf(rid),
            )

    # ---- Navigation chain --------------------------------------------

    def _send_nav_goal(
        self,
        robot_id: str,
        xy: tuple[float, float],
        on_success,
    ) -> None:
        client = self._robots[robot_id].nav_client
        if not client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(
                f"{robot_id} NavigateToPose action server not ready — Nav2 down?"
            )
            self._on_nav_failed(robot_id, "no_action_server")
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x = xy[0]
        goal.pose.pose.position.y = xy[1]
        goal.pose.pose.orientation.w = 1.0

        self._robots[robot_id].on_success = on_success
        future = client.send_goal_async(goal)
        future.add_done_callback(lambda f, rid=robot_id: self._on_goal_accepted(rid, f))

    def _on_goal_accepted(self, robot_id: str, future) -> None:
        handle = future.result()
        if handle is None or not handle.accepted:
            self._on_nav_failed(robot_id, "goal_rejected")
            return
        self._robots[robot_id].active_goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f, rid=robot_id: self._on_nav_result(rid, f)
        )

    def _on_nav_result(self, robot_id: str, future) -> None:
        result = future.result()
        status = result.status if result is not None else GoalStatus.STATUS_UNKNOWN
        self._robots[robot_id].active_goal_handle = None
        if status == GoalStatus.STATUS_SUCCEEDED:
            cb = self._robots[robot_id].on_success
            self._robots[robot_id].on_success = None
            if cb is not None:
                cb()
        else:
            self._on_nav_failed(robot_id, f"nav_status_{status}")

    # ---- State transitions -------------------------------------------

    def _on_arrived_at_shelf(self, robot_id: str) -> None:
        order = self._active.get(robot_id)
        if order is None:
            return
        self.get_logger().info(f"{robot_id} arrived at shelf for order {order.id}")
        # AT_SHELF is transient — magic-attach happens sim-side. Immediately
        # kick off NAV_TO_CELL.
        order.state = OrderState.NAV_TO_CELL
        self._send_nav_goal(
            robot_id,
            self._pick_cell_xy,
            on_success=lambda rid=robot_id: self._on_arrived_at_cell(rid),
        )

    def _on_arrived_at_cell(self, robot_id: str) -> None:
        order = self._active.get(robot_id)
        if order is None:
            return
        self.get_logger().info(
            f"{robot_id} arrived at pick cell for order {order.id} — starting pick"
        )
        order.state = OrderState.PICKING
        msg = String()
        msg.data = order.id
        self._start_pick_pub.publish(msg)

    def _on_pick_result(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"malformed /cell/pick_result: {msg.data}")
            return
        order_id = data.get("order_id")
        order = next(
            (o for o in self._active.values() if o.id == order_id),
            None,
        )
        if order is None:
            return
        order.attempts = int(data.get("attempts", 0))
        if data.get("success"):
            order.state = OrderState.COMPLETED
            self.get_logger().info(f"order {order_id} COMPLETED")
        else:
            order.state = OrderState.FAILED
            order.failure_reason = str(data.get("failure_reason", "unknown"))
            self.get_logger().warn(f"order {order_id} FAILED: {order.failure_reason}")
        self._finish_order(order.robot_id)

    def _on_nav_failed(self, robot_id: str, reason: str) -> None:
        order = self._active.get(robot_id)
        if order is None:
            # Robot was free already; nothing to fail.
            self._robots[robot_id].free = True
            return
        order.state = OrderState.FAILED
        order.failure_reason = reason
        self.get_logger().warn(f"order {order.id} FAILED during nav: {reason}")
        self._finish_order(robot_id)

    def _finish_order(self, robot_id: str | None) -> None:
        if robot_id is None:
            return
        order = self._active.pop(robot_id, None)
        if order is not None:
            self._completed.append(order)
        self._robots[robot_id].free = True
        self._robots[robot_id].on_success = None


def main() -> None:
    rclpy.init()
    rclpy.spin(FleetCoordinator())
    rclpy.shutdown()
