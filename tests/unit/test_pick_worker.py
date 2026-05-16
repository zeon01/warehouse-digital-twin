"""Unit tests for manipulation.pick_worker — threading + queue logic."""

from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np

from manipulation.pick_worker import PickRequest, PickWorker


@dataclass
class _StubArmResult:
    success: bool
    message: str = ""


class _StubArmPlanner:
    def __init__(self, results):
        self._results = list(results)  # consumed in order
        self.calls = []

    def plan_to_pose(self, translation, rotation):
        self.calls.append((np.asarray(translation).copy(), np.asarray(rotation).copy()))
        if not self._results:
            return _StubArmResult(False, "no_more_stub_results")
        return self._results.pop(0)


class _StubPoseSource:
    def __init__(self, pose):
        self._pose = pose  # tuple or None

    def get_pose(self, rgb, depth, camera_K, cad_path):
        return self._pose


def _identity_tf(_source_frame):
    return np.eye(4, dtype=np.float64)


def _wait_for_result(results, timeout=2.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if results:
            return results[0]
        time.sleep(0.01)
    raise AssertionError(f"no result in {timeout}s")


def test_worker_publishes_success_on_first_attempt():
    results = []
    worker = PickWorker(
        pose_source=_StubPoseSource((np.array([0.4, 0.0, -0.25]), "panda_link0")),
        arm_planner=_StubArmPlanner([_StubArmResult(True, "ok")]),
        publish_result=results.append,
        tf_lookup=_identity_tf,
        cad_path="x",
    )
    worker.start()
    try:
        worker.enqueue(PickRequest(order_id="o1", rgb=None, depth=None, camera_K=None))
        r = _wait_for_result(results)
        assert r.success is True
        assert r.order_id == "o1"
        assert r.attempts == 1
        assert r.failure_reason == ""
    finally:
        worker.stop()


def test_worker_publishes_no_pose_when_source_returns_none():
    results = []
    worker = PickWorker(
        pose_source=_StubPoseSource(None),
        arm_planner=_StubArmPlanner([]),
        publish_result=results.append,
        tf_lookup=_identity_tf,
        cad_path="x",
    )
    worker.start()
    try:
        worker.enqueue(PickRequest(order_id="o1", rgb=None, depth=None, camera_K=None))
        r = _wait_for_result(results)
        assert r.success is False
        assert r.failure_reason == "no_pose"
    finally:
        worker.stop()


def test_worker_publishes_tf_lookup_failed_when_tf_returns_none():
    results = []
    worker = PickWorker(
        pose_source=_StubPoseSource((np.array([0.0, 0.0, 1.0]), "cell_cam_optical")),
        arm_planner=_StubArmPlanner([]),
        publish_result=results.append,
        tf_lookup=lambda _src: None,
        cad_path="x",
    )
    worker.start()
    try:
        worker.enqueue(PickRequest(order_id="o1", rgb=None, depth=None, camera_K=None))
        r = _wait_for_result(results)
        assert r.success is False
        assert r.failure_reason == "tf_lookup_failed"
    finally:
        worker.stop()


def test_worker_retries_on_handle_none_race_condition():
    # First two attempts return handle=None (rclpy #1123 race); third succeeds.
    results = []
    worker = PickWorker(
        pose_source=_StubPoseSource((np.array([0.4, 0.0, -0.25]), "panda_link0")),
        arm_planner=_StubArmPlanner(
            [
                _StubArmResult(False, "goal_rejected handle=None"),
                _StubArmResult(False, "goal_rejected handle=None"),
                _StubArmResult(True, "ok"),
            ]
        ),
        publish_result=results.append,
        tf_lookup=_identity_tf,
        cad_path="x",
        race_retry_sleep_s=0.0,  # don't wait in tests
    )
    worker.start()
    try:
        worker.enqueue(PickRequest(order_id="o1", rgb=None, depth=None, camera_K=None))
        r = _wait_for_result(results, timeout=3.0)
        assert r.success is True
        assert r.attempts == 3
    finally:
        worker.stop()


def test_worker_publishes_plan_no_solution_on_real_planner_failure():
    # ArmPlanner says "status=4 error_code=-12" — a real planner failure,
    # NOT the race-condition signature. Should NOT retry.
    results = []
    arm = _StubArmPlanner([_StubArmResult(False, "status=4 error_code=-12")])
    worker = PickWorker(
        pose_source=_StubPoseSource((np.array([0.4, 0.0, -0.25]), "panda_link0")),
        arm_planner=arm,
        publish_result=results.append,
        tf_lookup=_identity_tf,
        cad_path="x",
    )
    worker.start()
    try:
        worker.enqueue(PickRequest(order_id="o1", rgb=None, depth=None, camera_K=None))
        r = _wait_for_result(results)
        assert r.success is False
        assert r.failure_reason.startswith("plan_no_solution")
        assert len(arm.calls) == 1  # NOT 3 — no retries on real failures
    finally:
        worker.stop()


def test_worker_publishes_plan_action_failed_after_max_retries():
    results = []
    worker = PickWorker(
        pose_source=_StubPoseSource((np.array([0.4, 0.0, -0.25]), "panda_link0")),
        arm_planner=_StubArmPlanner(
            [
                _StubArmResult(False, "goal_rejected handle=None"),
                _StubArmResult(False, "goal_rejected handle=None"),
                _StubArmResult(False, "goal_rejected handle=None"),
            ]
        ),
        publish_result=results.append,
        tf_lookup=_identity_tf,
        cad_path="x",
        race_retry_sleep_s=0.0,
    )
    worker.start()
    try:
        worker.enqueue(PickRequest(order_id="o1", rgb=None, depth=None, camera_K=None))
        r = _wait_for_result(results)
        assert r.success is False
        assert r.failure_reason == "plan_action_failed"
        assert r.attempts == 3
    finally:
        worker.stop()
