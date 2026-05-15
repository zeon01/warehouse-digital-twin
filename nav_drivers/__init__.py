"""Pure-Python AMR navigation control laws.

Mirrors the ``coordinator/`` and ``manipulation/`` pattern: the algorithm
lives here (testable with plain pytest, no ROS2 install needed); a thin
ROS2 wrapper in ``ros2_ws/src/wdt_pure_pursuit/`` imports from here.

Phase 2 fallback: when Nav2's controller_server doesn't drive Nova Carter
(Carter's LIDAR doesn't publish under standalone-python Isaac Sim, so
costmap obstacle observations are empty and DWB may score every trajectory
as bad), the pure-pursuit driver runs the AMR straight to the goal with a
simple go-to-goal control law. Defensible portfolio story: "Nav2 planner
+ costmap real; controller hand-rolled because the simulator's LIDAR
publisher is broken on this image."
"""

from nav_drivers.pure_pursuit import PurePursuitConfig, compute_cmd_vel, wrap_to_pi

__all__ = ["PurePursuitConfig", "compute_cmd_vel", "wrap_to_pi"]
