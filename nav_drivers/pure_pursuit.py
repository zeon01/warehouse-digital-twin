"""Go-to-goal control law for diff-drive AMRs.

Strictly speaking this is a *go-to-goal* controller, not pure-pursuit
(which has a lookahead distance along a path). For warehouse AMRs moving
between named waypoints — shelf, pick cell, dock — a single-target
controller is sufficient and the wider robotics ecosystem still calls
this family "pure pursuit", so the name sticks.

Control law (diff-drive):
    dx, dy            = goal - pose
    distance          = hypot(dx, dy)
    heading_error     = wrap_to_pi(atan2(dy, dx) - yaw)
    angular_z         = clip(K_w * heading_error, +/- max_w)
    if |heading_error| > heading_gate:     # too off-axis to drive forward
        linear_x      = 0.0                # turn in place
    else:
        linear_x      = clip(K_v * distance * cos(heading_error), 0, max_v)

The cosine gating makes the linear command roll off smoothly as the
robot drifts away from heading-aligned, instead of fighting the angular
controller. Past heading_gate the linear command drops to zero so the
robot pivots in place — important when the goal is behind the robot,
where a non-zero linear command would arc rather than turn.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class PurePursuitConfig:
    """Tuning constants for ``compute_cmd_vel``.

    Defaults are conservative for Nova Carter on the small warehouse map
    (max physical speed ~1.5 m/s linear, ~2 rad/s angular). Slow them
    down further in narrow aisles or speed up in straight corridors via
    constructor args.
    """

    k_linear: float = 0.6
    k_angular: float = 1.5
    max_linear: float = 0.5
    max_angular: float = 1.0
    heading_gate_rad: float = math.pi / 4  # 45 deg
    goal_tolerance_m: float = 0.25


def wrap_to_pi(angle: float) -> float:
    """Map any real-valued angle into (-pi, pi]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def compute_cmd_vel(
    pose_x: float,
    pose_y: float,
    pose_yaw: float,
    goal_x: float,
    goal_y: float,
    config: PurePursuitConfig | None = None,
) -> tuple[float, float, float]:
    """Compute ``(linear_x, angular_z, distance_to_goal)`` for one tick.

    Caller should treat ``distance_to_goal <= config.goal_tolerance_m`` as
    "arrived" and stop publishing — this function still returns the
    clamped commands at the goal (both zero), but the action-server
    wrapper is responsible for the termination decision.
    """
    cfg = config or PurePursuitConfig()

    dx = goal_x - pose_x
    dy = goal_y - pose_y
    distance = math.hypot(dx, dy)

    if distance <= cfg.goal_tolerance_m:
        return 0.0, 0.0, distance

    target_heading = math.atan2(dy, dx)
    heading_error = wrap_to_pi(target_heading - pose_yaw)

    angular_z = max(-cfg.max_angular, min(cfg.max_angular, cfg.k_angular * heading_error))

    if abs(heading_error) > cfg.heading_gate_rad:
        linear_x = 0.0
    else:
        raw_linear = cfg.k_linear * distance * math.cos(heading_error)
        linear_x = max(0.0, min(cfg.max_linear, raw_linear))

    return linear_x, angular_z, distance
