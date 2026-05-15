"""Unit tests for the go-to-goal control law."""

from __future__ import annotations

import math

import pytest

from nav_drivers.pure_pursuit import (
    PurePursuitConfig,
    compute_cmd_vel,
    wrap_to_pi,
)


def test_wrap_to_pi_identity_in_range():
    assert wrap_to_pi(0.0) == 0.0
    assert wrap_to_pi(1.0) == pytest.approx(1.0)
    assert wrap_to_pi(-1.0) == pytest.approx(-1.0)


def test_wrap_to_pi_wraps_above_pi():
    assert wrap_to_pi(math.pi + 0.1) == pytest.approx(-math.pi + 0.1)


def test_wrap_to_pi_wraps_below_neg_pi():
    assert wrap_to_pi(-math.pi - 0.1) == pytest.approx(math.pi - 0.1)


def test_aligned_far_drives_forward():
    # Robot at origin facing +x, goal 5 m straight ahead.
    lin, ang, dist = compute_cmd_vel(0.0, 0.0, 0.0, 5.0, 0.0)
    assert dist == pytest.approx(5.0)
    assert lin > 0.0
    assert lin <= PurePursuitConfig().max_linear
    assert abs(ang) < 1e-9


def test_aligned_far_clamps_to_max_linear():
    cfg = PurePursuitConfig(k_linear=1.0, max_linear=0.5)
    lin, _, _ = compute_cmd_vel(0.0, 0.0, 0.0, 100.0, 0.0, config=cfg)
    assert lin == pytest.approx(0.5)


def test_at_goal_within_tolerance_stops():
    cfg = PurePursuitConfig(goal_tolerance_m=0.25)
    lin, ang, dist = compute_cmd_vel(1.0, 1.0, 0.0, 1.1, 1.1, config=cfg)
    assert dist < 0.25
    assert lin == 0.0
    assert ang == 0.0


def test_perpendicular_goal_turns_in_place():
    # Robot at origin facing +x, goal at +y (90 deg off).
    lin, ang, _ = compute_cmd_vel(0.0, 0.0, 0.0, 0.0, 5.0)
    assert lin == 0.0  # heading_error > heading_gate -> turn in place
    assert ang > 0.0  # rotates CCW toward +y


def test_goal_behind_turns_in_place():
    # Robot at origin facing +x, goal at -x (180 deg off).
    lin, ang, _ = compute_cmd_vel(0.0, 0.0, 0.0, -5.0, 0.0)
    assert lin == 0.0
    # Either direction of rotation is valid for 180 deg; both saturate.
    assert abs(ang) == pytest.approx(PurePursuitConfig().max_angular)


def test_slightly_off_heading_drives_and_steers():
    # Robot facing +x, goal at 20 deg off — within heading_gate (45 deg).
    goal_angle = math.radians(20)
    lin, ang, _ = compute_cmd_vel(
        0.0, 0.0, 0.0, 5.0 * math.cos(goal_angle), 5.0 * math.sin(goal_angle)
    )
    assert lin > 0.0  # forward motion allowed
    assert ang > 0.0  # but also steering toward the goal


def test_negative_yaw_wrap_around():
    # Robot facing nearly -x (yaw = pi - 0.1), goal at -x.
    # heading_error should be ~+0.1, well within heading_gate.
    lin, ang, _ = compute_cmd_vel(0.0, 0.0, math.pi - 0.1, -5.0, 0.0)
    assert lin > 0.0
    assert ang > 0.0
    assert ang < PurePursuitConfig().max_angular


def test_angular_command_clamps_to_max():
    cfg = PurePursuitConfig(k_angular=10.0, max_angular=1.0)
    # Robot facing +x, goal at +y (90 deg off) → huge k_angular * err.
    _, ang, _ = compute_cmd_vel(0.0, 0.0, 0.0, 0.0, 5.0, config=cfg)
    assert ang == pytest.approx(1.0)


def test_custom_config_overrides_defaults():
    cfg = PurePursuitConfig(
        k_linear=2.0,
        k_angular=0.5,
        max_linear=10.0,
        max_angular=10.0,
        heading_gate_rad=math.pi,  # never turn in place
        goal_tolerance_m=0.01,
    )
    # Goal behind, would normally turn in place; with heading_gate=pi
    # the robot is allowed to drive (it'll back-arc, but the law
    # honors the relaxed gate).
    lin, ang, _ = compute_cmd_vel(0.0, 0.0, 0.0, -1.0, 0.0, config=cfg)
    # cos(pi) = -1, so raw_linear = 2.0 * 1.0 * -1.0 = -2.0,
    # clamped to [0, 10.0] → 0.0. Good: still no reverse driving.
    assert lin == 0.0
    # angular saturates at max_angular regardless of gate.
    assert abs(ang) == pytest.approx(10.0) or abs(ang) == pytest.approx(0.5 * math.pi)
