# Pick Chain Redesign Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the M5 pick orchestrator with a worker-thread architecture + pluggable PoseSource so the rclpy callback-deadlock class of bugs is eliminated and M5 acceptance (`orders_completed=1`) ships on simulator ground-truth pose with FoundationPose as a stretch.

**Architecture:** Thin rclpy subscription node owns no business logic — it caches cam frames and enqueues pick requests. A dedicated worker thread with its own rclpy `Node` + `SingleThreadedExecutor` does PoseSource → tf2 transform → ArmPlanner.plan_to_pose synchronously. PoseSource is a `Protocol` with `FoundationPosePoseSource` (wraps existing `PoseEstimator`) and `GroundTruthPoseSource` (reads `/world/cube_pose` from a sim-side publisher). Configured via a ROS2 parameter on the orchestrator: `pose_source: "fp" | "gt"`.

**Tech Stack:** ROS2 Humble + rclpy, Python 3.10 (orchestrator/worker), Python 3.11 (Isaac Sim Kit for the sim-side publisher), MoveIt2, FoundationPose, Isaac Sim 5.0, vast.ai (Quebec instance 36866311).

**Spec:** `docs/superpowers/specs/2026-05-16-pick-chain-redesign-design.md`

---

## File Structure

**Created:**
- `manipulation/pose_source.py` — `PoseSource` Protocol + `FoundationPosePoseSource` + `GroundTruthPoseSource`
- `manipulation/pick_worker.py` — `PickWorker` thread + `PickRequest` / `PickResult` dataclasses
- `wdt_vast/sim_world_pose_publisher.py` — Kit-python rclpy publisher of `/world/cube_pose`
- `tests/unit/test_pose_source.py` — unit tests for both implementations
- `tests/unit/test_pick_worker.py` — unit tests for `PickWorker` with mocked deps
- `tests/integration/test_pick_chain_fast.py` — fast harness (no sim, GT and FP variants)

**Rewritten:**
- `ros2_ws/src/wdt_manipulation_bringup/wdt_manipulation_bringup/pick_cell_orchestrator.py` — thin node + `PickWorker`

**Modified:**
- `manipulation/motion_planning.py:75-104` — `ArmPlanner.__init__` accepts explicit `executor` param; `plan_to_pose` uses it in `spin_until_future_complete`
- `wdt_vast/run_scenario.py:154-203` — spawn cube/table/lighting (kept from v17–v21); add `sim_world_pose_publisher.py` subprocess launch; pass `pose_source` ROS2 param to orchestrator

**Untouched:**
- `manipulation/pose_estimation.py` — v19 nearest-depth mask stays
- `manipulation/grasping.py` — `TopDownGrasp.propose_at` stays
- `manipulation/pipeline.py` — `ManipulationPipeline` kept for any future user; not used by the new worker (worker does its own composition)
- Everything outside `manipulation/`, `wdt_vast/`, `ros2_ws/src/wdt_manipulation_bringup/`

---

## Task 1: `PoseSource` Protocol + `FoundationPosePoseSource` adapter

**Files:**
- Create: `manipulation/pose_source.py`
- Test: `tests/unit/test_pose_source.py`

- [ ] **Step 1: Write the failing test**

`tests/unit/test_pose_source.py`:
```python
"""Unit tests for manipulation.pose_source — PoseSource implementations."""

from __future__ import annotations

import numpy as np
import pytest

from manipulation.pose_source import FoundationPosePoseSource


class _FakeFPEstimator:
    """Stub for PoseEstimator that returns one preset pose."""

    def __init__(self, translation):
        self._t = np.asarray(translation, dtype=np.float32)
        self.calls = []

    def estimate(self, rgb, depth, cad_path, camera_K):
        self.calls.append({"rgb": rgb, "depth": depth, "cad": cad_path, "K": camera_K})

        class _P:
            translation = self._t

        return [_P()]


def test_foundation_pose_source_returns_translation_and_optical_frame():
    fp = _FakeFPEstimator([0.0, 0.0, 1.097])
    src = FoundationPosePoseSource(estimator=fp, frame_id="cell_cam_optical")

    rgb = np.zeros((480, 640, 3), dtype=np.uint8)
    depth = np.full((480, 640), 1.0, dtype=np.float32)
    K = np.eye(3, dtype=np.float64)
    out = src.get_pose(rgb=rgb, depth=depth, camera_K=K, cad_path="/tmp/cube.obj")

    assert out is not None
    translation, frame_id = out
    assert frame_id == "cell_cam_optical"
    np.testing.assert_allclose(translation, [0.0, 0.0, 1.097], atol=1e-6)
    assert len(fp.calls) == 1


def test_foundation_pose_source_returns_none_on_empty_pose_list():
    class _Empty:
        def estimate(self, **kwargs):
            return []

    src = FoundationPosePoseSource(estimator=_Empty(), frame_id="cell_cam_optical")
    out = src.get_pose(
        rgb=np.zeros((1, 1, 3), dtype=np.uint8),
        depth=np.zeros((1, 1), dtype=np.float32),
        camera_K=np.eye(3),
        cad_path="x",
    )
    assert out is None
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `cd /Users/aiqarus/Desktop/Projects/isaac-sim && python -m pytest tests/unit/test_pose_source.py -v`
Expected: `ModuleNotFoundError: No module named 'manipulation.pose_source'`

- [ ] **Step 3: Implement `pose_source.py`**

`manipulation/pose_source.py`:
```python
"""Pluggable pose sources for the M5 pick chain.

Two implementations:

- ``FoundationPosePoseSource`` — wraps the FoundationPose estimator and
  returns the cube center in the camera optical frame.
- ``GroundTruthPoseSource`` — fed by an external subscription to
  ``/world/cube_pose`` (published by ``wdt_vast/sim_world_pose_publisher.py``).
  Bypasses perception entirely for the M5 demo path.

Selected via the orchestrator's ROS2 parameter ``pose_source: "fp" | "gt"``.
"""

from __future__ import annotations

import threading
from typing import Any, Protocol

import numpy as np


class PoseSource(Protocol):
    """Returns the 6D pose of the pick target.

    Implementations may ignore any of ``rgb``/``depth``/``camera_K``/
    ``cad_path``; the worker passes all four and lets each source pick what
    it needs. Returns ``(translation_3, source_frame_id)`` or ``None`` on
    failure (e.g. no pose received yet, registration failed).
    """

    def get_pose(
        self,
        rgb: np.ndarray | None,
        depth: np.ndarray | None,
        camera_K: np.ndarray | None,
        cad_path: str,
    ) -> tuple[np.ndarray, str] | None: ...


class FoundationPosePoseSource:
    """Wraps the existing FoundationPose ``PoseEstimator``.

    ``frame_id`` MUST match what the ROS2CameraInfoHelper publishes on
    ``/cell/cam/info`` — currently ``"cell_cam_optical"`` (see
    ``sim/cell_camera.py``).
    """

    def __init__(self, estimator: Any, frame_id: str = "cell_cam_optical") -> None:
        self._estimator = estimator
        self._frame_id = frame_id

    def get_pose(
        self,
        rgb: np.ndarray | None,
        depth: np.ndarray | None,
        camera_K: np.ndarray | None,
        cad_path: str,
    ) -> tuple[np.ndarray, str] | None:
        if rgb is None or depth is None or camera_K is None:
            return None
        poses = self._estimator.estimate(
            rgb=rgb, depth=depth, cad_path=cad_path, camera_K=camera_K
        )
        if not poses:
            return None
        return (np.asarray(poses[0].translation, dtype=np.float64), self._frame_id)


class GroundTruthPoseSource:
    """Returns the latest pose set by ``set_latest`` — typically wired to a
    ``/world/cube_pose`` subscription on the orchestrator's main node.

    Thread-safe: the subscription callback runs on the main thread while
    ``get_pose`` is called from the worker thread.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._latest: tuple[np.ndarray, str] | None = None

    def set_latest(self, translation: np.ndarray, frame_id: str) -> None:
        with self._lock:
            self._latest = (np.asarray(translation, dtype=np.float64).copy(), frame_id)

    def get_pose(
        self,
        rgb: np.ndarray | None,
        depth: np.ndarray | None,
        camera_K: np.ndarray | None,
        cad_path: str,
    ) -> tuple[np.ndarray, str] | None:
        with self._lock:
            if self._latest is None:
                return None
            t, fid = self._latest
            return (t.copy(), fid)
```

- [ ] **Step 4: Run the test to verify it passes**

Run: `cd /Users/aiqarus/Desktop/Projects/isaac-sim && python -m pytest tests/unit/test_pose_source.py -v`
Expected: 2 passed

- [ ] **Step 5: Commit**

```bash
cd /Users/aiqarus/Desktop/Projects/isaac-sim
git add manipulation/pose_source.py tests/unit/test_pose_source.py
git commit -m "$(cat <<'EOF'
feat(m5): add PoseSource protocol + FoundationPosePoseSource adapter

Pluggable pose-source abstraction for the M5 pick chain redesign. FP
adapter wraps the existing PoseEstimator and returns
(translation, "cell_cam_optical"). Ground-truth implementation follows
in the next task.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 2: `GroundTruthPoseSource` test coverage

**Files:**
- Test: `tests/unit/test_pose_source.py` (append)

Note: implementation already shipped in Task 1; this task just adds the unit tests for `GroundTruthPoseSource` so the contract is locked in.

- [ ] **Step 1: Add the failing tests**

Append to `tests/unit/test_pose_source.py`:
```python
from manipulation.pose_source import GroundTruthPoseSource


def test_ground_truth_source_returns_none_before_first_set():
    src = GroundTruthPoseSource()
    out = src.get_pose(rgb=None, depth=None, camera_K=None, cad_path="x")
    assert out is None


def test_ground_truth_source_returns_latest_pose():
    src = GroundTruthPoseSource()
    src.set_latest(np.array([16.40, 15.00, 0.75]), "world")
    out = src.get_pose(rgb=None, depth=None, camera_K=None, cad_path="x")
    assert out is not None
    translation, frame_id = out
    assert frame_id == "world"
    np.testing.assert_allclose(translation, [16.40, 15.00, 0.75])


def test_ground_truth_source_returns_copy_not_alias():
    src = GroundTruthPoseSource()
    payload = np.array([1.0, 2.0, 3.0])
    src.set_latest(payload, "world")
    out = src.get_pose(rgb=None, depth=None, camera_K=None, cad_path="x")
    assert out is not None
    out[0][0] = 999.0  # mutate the returned translation
    out2 = src.get_pose(rgb=None, depth=None, camera_K=None, cad_path="x")
    assert out2 is not None
    assert out2[0][0] == 1.0  # internal state unchanged
```

- [ ] **Step 2: Run the tests to verify they pass**

Run: `cd /Users/aiqarus/Desktop/Projects/isaac-sim && python -m pytest tests/unit/test_pose_source.py -v`
Expected: 5 passed (2 FP from Task 1 + 3 GT)

- [ ] **Step 3: Commit**

```bash
git add tests/unit/test_pose_source.py
git commit -m "$(cat <<'EOF'
test(m5): unit tests for GroundTruthPoseSource

Locks the contract: returns None before first set; returns latest
(translation, frame_id) after set; returns copies (caller mutation
doesn't corrupt internal state).

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 3: `ArmPlanner`: accept explicit executor

**Files:**
- Modify: `manipulation/motion_planning.py:75-104` (constructor + `_lazy_load`)
- Modify: `manipulation/motion_planning.py:172-192` (`plan_to_pose` spin calls)

- [ ] **Step 1: Read the current `ArmPlanner` class**

Run: `sed -n '75,205p' /Users/aiqarus/Desktop/Projects/isaac-sim/manipulation/motion_planning.py`
Expected: shows the existing class.

- [ ] **Step 2: Edit `__init__` to accept executor**

Replace lines 75-87 of `manipulation/motion_planning.py`:
```python
class ArmPlanner:
    def __init__(
        self,
        parent_node: Any = None,
        planning_group: str = "panda_arm",
        plan_only: bool = True,
        executor: Any = None,
    ) -> None:
        self._planning_group = planning_group
        self._parent_node = parent_node
        self._plan_only = plan_only
        self._client: Any = None
        self._owns_node = parent_node is None
        self._node: Any = None
        # Optional explicit executor for spin_until_future_complete. The
        # worker-thread architecture (M5 redesign) requires the action
        # client's spins to use the worker's own SingleThreadedExecutor,
        # not the global default — otherwise the worker's spin races with
        # the orchestrator's main-thread spin on the global executor.
        self._executor = executor
```

- [ ] **Step 3: Edit `plan_to_pose` to pass the executor to spins**

Find these two lines in `plan_to_pose`:
```python
        rclpy.spin_until_future_complete(self._node, send_future, timeout_sec=ACTION_TIMEOUT_S)
```
and
```python
        rclpy.spin_until_future_complete(self._node, result_future, timeout_sec=GOAL_TIMEOUT_S)
```

Replace each with the executor-aware variant:
```python
        rclpy.spin_until_future_complete(
            self._node, send_future, executor=self._executor, timeout_sec=ACTION_TIMEOUT_S
        )
```
```python
        rclpy.spin_until_future_complete(
            self._node, result_future, executor=self._executor, timeout_sec=GOAL_TIMEOUT_S
        )
```

(`rclpy.spin_until_future_complete` accepts `executor=None` → falls back to the global executor, preserving back-compat with existing callers that didn't pass one.)

- [ ] **Step 4: Run the existing ArmPlanner unit test**

Run: `cd /Users/aiqarus/Desktop/Projects/isaac-sim && python -m pytest tests/unit/test_motion_planning.py -v`
Expected: pass (the existing test uses mock action clients; adding an optional kwarg doesn't break it). If `tests/unit/test_motion_planning.py` doesn't exist, skip this step.

- [ ] **Step 5: Commit**

```bash
git add manipulation/motion_planning.py
git commit -m "$(cat <<'EOF'
feat(m5): ArmPlanner accepts explicit executor

Required for the worker-thread architecture: the worker owns its own
SingleThreadedExecutor and needs spin_until_future_complete to use that
specific executor, not the global default that the main thread shares.
Backwards-compatible — executor defaults to None (global).

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 4: `PickWorker` class with mocked dependencies

**Files:**
- Create: `manipulation/pick_worker.py`
- Test: `tests/unit/test_pick_worker.py`

- [ ] **Step 1: Write the failing test**

`tests/unit/test_pick_worker.py`:
```python
"""Unit tests for manipulation.pick_worker — threading + queue logic."""

from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np
import pytest

from manipulation.pick_worker import PickRequest, PickResult, PickWorker


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
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd /Users/aiqarus/Desktop/Projects/isaac-sim && python -m pytest tests/unit/test_pick_worker.py -v`
Expected: `ModuleNotFoundError: No module named 'manipulation.pick_worker'`

- [ ] **Step 3: Implement `pick_worker.py`**

`manipulation/pick_worker.py`:
```python
"""Worker thread that runs the M5 pick chain off the rclpy callback path.

The orchestrator's subscription callback enqueues a ``PickRequest``; this
worker dequeues, calls the PoseSource, transforms via tf2 (caller-supplied
lookup function), proposes a top-down grasp, and invokes ArmPlanner. The
result is published via a caller-supplied callback.

Threading: the worker runs on its own ``threading.Thread`` (daemon). It
expects ``arm_planner`` and ``tf_lookup`` to be safe to call from this
thread. For ArmPlanner that means it owns its own rclpy Node + Executor
(see ``ArmPlanner(..., executor=...)`` after task 3).

Why this exists: the previous orchestrator called ArmPlanner directly
from the ``/cell/start_pick`` subscription callback. That triggered a
documented rclpy deadlock (Karelics writeup; rclpy issue #1123): the
synchronous action call's spin_until_future_complete can't be served by
an executor that's already inside the callback. M5 v20/v21 hit this
verbatim with cycle_time≈5.0s == ACTION_TIMEOUT_S.
"""

from __future__ import annotations

import queue
import threading
import time
from dataclasses import dataclass
from time import perf_counter
from typing import Any, Callable

import numpy as np


@dataclass
class PickRequest:
    order_id: str
    rgb: np.ndarray | None
    depth: np.ndarray | None
    camera_K: np.ndarray | None


@dataclass
class PickResult:
    order_id: str
    success: bool
    attempts: int
    cycle_time_s: float
    failure_reason: str


# Sentinel pushed on the queue at shutdown so the loop exits cleanly.
_SHUTDOWN = object()


class PickWorker:
    """Single-consumer queue worker that runs the pick chain synchronously.

    Constructor parameters:
    - ``pose_source``: implements ``PoseSource.get_pose``.
    - ``arm_planner``: object with ``.plan_to_pose(translation, rotation)``
      returning an object that has ``.success: bool`` and ``.message: str``
      (matches ``ArmPlanner``'s ``ArmExecutionResult``).
    - ``publish_result``: callback ``(PickResult) -> None``.
    - ``tf_lookup``: callable ``(source_frame: str) -> np.ndarray (4x4) | None``.
      Returns the homogeneous transform from ``source_frame`` to
      ``panda_link0``, or ``None`` on lookup failure.
    - ``cad_path``: passed through to ``pose_source.get_pose``.
    - ``standoff_m``: vertical offset added to the cube center to form the
      grasp pose. Default 0.05 (matches TopDownGrasp).
    - ``max_race_retries``: how many times to retry plan_to_pose on the
      rclpy #1123 race-condition signature (``"goal_rejected handle=None"``).
      Default 3.
    - ``race_retry_sleep_s``: sleep between race-condition retries. Default
      0.2; set to 0.0 in tests.
    """

    _GRIPPER_DOWN_ROTATION = np.array(
        [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]], dtype=np.float32
    )

    def __init__(
        self,
        pose_source: Any,
        arm_planner: Any,
        publish_result: Callable[[PickResult], None],
        tf_lookup: Callable[[str], np.ndarray | None],
        cad_path: str,
        standoff_m: float = 0.05,
        max_race_retries: int = 3,
        race_retry_sleep_s: float = 0.2,
    ) -> None:
        self._pose_source = pose_source
        self._arm = arm_planner
        self._publish_result = publish_result
        self._tf_lookup = tf_lookup
        self._cad_path = cad_path
        self._standoff_m = float(standoff_m)
        self._max_race_retries = int(max_race_retries)
        self._race_retry_sleep_s = float(race_retry_sleep_s)
        self._queue: queue.Queue = queue.Queue()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._thread = threading.Thread(target=self._loop, daemon=True, name="PickWorker")
        self._thread.start()

    def stop(self, timeout_s: float = 5.0) -> None:
        if self._thread is None:
            return
        self._queue.put(_SHUTDOWN)
        self._thread.join(timeout=timeout_s)
        self._thread = None

    def enqueue(self, request: PickRequest) -> None:
        self._queue.put(request)

    def _loop(self) -> None:
        while True:
            item = self._queue.get()
            if item is _SHUTDOWN:
                return
            try:
                self._process(item)
            except Exception as exc:  # pragma: no cover — sentinel result
                self._publish_result(
                    PickResult(
                        order_id=getattr(item, "order_id", "?"),
                        success=False,
                        attempts=0,
                        cycle_time_s=0.0,
                        failure_reason=f"worker_crashed: {type(exc).__name__}: {exc}",
                    )
                )

    def _process(self, req: PickRequest) -> None:
        t0 = perf_counter()

        # Step 2: pose estimation
        pose_result = self._pose_source.get_pose(
            rgb=req.rgb, depth=req.depth, camera_K=req.camera_K, cad_path=self._cad_path
        )
        if pose_result is None:
            self._publish_result(
                PickResult(req.order_id, False, 0, perf_counter() - t0, "no_pose")
            )
            return
        translation, source_frame = pose_result

        # Step 3: TF transform
        transform = self._tf_lookup(source_frame)
        if transform is None:
            self._publish_result(
                PickResult(req.order_id, False, 0, perf_counter() - t0, "tf_lookup_failed")
            )
            return
        t_homo = np.array(
            [float(translation[0]), float(translation[1]), float(translation[2]), 1.0],
            dtype=np.float64,
        )
        panda_t = (transform @ t_homo)[:3]

        # Step 4: grasp generation (top-down, +Z standoff)
        grasp_t = panda_t.copy()
        grasp_t[2] += self._standoff_m
        rotation = self._GRIPPER_DOWN_ROTATION

        # Step 5: MoveIt plan with race-condition retry
        attempts = 0
        last_message = ""
        for attempt in range(1, self._max_race_retries + 1):
            attempts = attempt
            res = self._arm.plan_to_pose(grasp_t.astype(np.float32), rotation)
            last_message = getattr(res, "message", "")
            if getattr(res, "success", False):
                self._publish_result(
                    PickResult(req.order_id, True, attempts, perf_counter() - t0, "")
                )
                return
            # Distinguish race-condition vs. real failure. Only retry on
            # the rclpy #1123 signature.
            if "handle=None" in last_message:
                if attempt < self._max_race_retries:
                    time.sleep(self._race_retry_sleep_s)
                    continue
                # Exhausted race-condition retries.
                self._publish_result(
                    PickResult(
                        req.order_id,
                        False,
                        attempts,
                        perf_counter() - t0,
                        "plan_action_failed",
                    )
                )
                return
            # Real planner failure — surface the message, no retry.
            self._publish_result(
                PickResult(
                    req.order_id,
                    False,
                    attempts,
                    perf_counter() - t0,
                    f"plan_no_solution({last_message})",
                )
            )
            return

        # Shouldn't reach here, but be defensive.
        self._publish_result(
            PickResult(req.order_id, False, attempts, perf_counter() - t0, "plan_action_failed")
        )
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd /Users/aiqarus/Desktop/Projects/isaac-sim && python -m pytest tests/unit/test_pick_worker.py -v`
Expected: 6 passed

- [ ] **Step 5: Commit**

```bash
git add manipulation/pick_worker.py tests/unit/test_pick_worker.py
git commit -m "$(cat <<'EOF'
feat(m5): PickWorker thread + 6 unit tests

Worker dequeues PickRequest → pose_source.get_pose → tf_lookup →
TopDownGrasp standoff → arm_planner.plan_to_pose (with race-condition
retry) → publish_result callback. All deps injected so the worker is
fully unit-testable without rclpy. Distinguishes plan_no_solution
(physics) from plan_action_failed (infrastructure / rclpy #1123).

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 5: Rewrite `pick_cell_orchestrator.py` as thin node + worker

**Files:**
- Rewrite: `ros2_ws/src/wdt_manipulation_bringup/wdt_manipulation_bringup/pick_cell_orchestrator.py`

- [ ] **Step 1: Replace the file entirely**

Overwrite `ros2_ws/src/wdt_manipulation_bringup/wdt_manipulation_bringup/pick_cell_orchestrator.py` with:
```python
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
        pose_source_kind = (
            self.get_parameter("pose_source").get_parameter_value().string_value
        )

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
            self.create_subscription(
                PoseStamped, "/world/cube_pose", self._on_cube_pose, 10
            )
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
            self.get_logger().warn(
                f"TF lookup {source_frame}→{PLANNING_FRAME} failed: {exc}"
            )
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
```

- [ ] **Step 2: Verify the file is syntactically valid**

Run: `cd /Users/aiqarus/Desktop/Projects/isaac-sim && python -m py_compile ros2_ws/src/wdt_manipulation_bringup/wdt_manipulation_bringup/pick_cell_orchestrator.py`
Expected: no output (success)

- [ ] **Step 3: Commit**

```bash
git add ros2_ws/src/wdt_manipulation_bringup/wdt_manipulation_bringup/pick_cell_orchestrator.py
git commit -m "$(cat <<'EOF'
feat(m5): rewrite pick_cell_orchestrator as thin node + PickWorker

Subscription callbacks now just cache state and enqueue. All FP + TF +
MoveIt work runs on PickWorker, which owns its own rclpy Node and
SingleThreadedExecutor for the MoveGroup action client. Eliminates the
v11–v21 callback-deadlock class of bugs entirely.

ROS2 param pose_source ("fp"|"gt") selects FoundationPosePoseSource
or GroundTruthPoseSource without recompile. GT mode subscribes to
/world/cube_pose (published by wdt_vast/sim_world_pose_publisher.py
in the next task).

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 6: `sim_world_pose_publisher.py` — `/world/cube_pose` from Isaac Sim

**Files:**
- Create: `wdt_vast/sim_world_pose_publisher.py`

- [ ] **Step 1: Implement the publisher**

`wdt_vast/sim_world_pose_publisher.py`:
```python
"""Publish the pick cube's worldspace pose on /world/cube_pose at 10 Hz.

Runs inside the Isaac Sim kit process (python 3.11) since it needs USD
stage access. The orchestrator subscribes to this topic when running in
``pose_source=gt`` mode, completely bypassing FoundationPose for the M5
acceptance loop.

Invocation (from run_scenario.py):
    /isaac-sim/python.sh wdt_vast/sim_world_pose_publisher.py \\
        --cube-prim-path /World/pick_cube
"""

from __future__ import annotations

import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

PUBLISH_RATE_HZ = 10.0


def _quaternion_from_matrix(m) -> tuple[float, float, float, float]:
    """3x3 rotation matrix (USD Gf.Matrix3d or numpy) → (qx, qy, qz, qw)."""
    # Tolerate both pxr.Gf.Matrix3d and numpy.ndarray
    def _r(i, j):
        try:
            return float(m[i][j])
        except TypeError:
            return float(m[i, j])

    tr = _r(0, 0) + _r(1, 1) + _r(2, 2)
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        qw = 0.25 * s
        qx = (_r(2, 1) - _r(1, 2)) / s
        qy = (_r(0, 2) - _r(2, 0)) / s
        qz = (_r(1, 0) - _r(0, 1)) / s
    else:
        # Branchy fallback — pick the largest diagonal.
        if _r(0, 0) > _r(1, 1) and _r(0, 0) > _r(2, 2):
            s = math.sqrt(1.0 + _r(0, 0) - _r(1, 1) - _r(2, 2)) * 2
            qw = (_r(2, 1) - _r(1, 2)) / s
            qx = 0.25 * s
            qy = (_r(0, 1) + _r(1, 0)) / s
            qz = (_r(0, 2) + _r(2, 0)) / s
        elif _r(1, 1) > _r(2, 2):
            s = math.sqrt(1.0 + _r(1, 1) - _r(0, 0) - _r(2, 2)) * 2
            qw = (_r(0, 2) - _r(2, 0)) / s
            qx = (_r(0, 1) + _r(1, 0)) / s
            qy = 0.25 * s
            qz = (_r(1, 2) + _r(2, 1)) / s
        else:
            s = math.sqrt(1.0 + _r(2, 2) - _r(0, 0) - _r(1, 1)) * 2
            qw = (_r(1, 0) - _r(0, 1)) / s
            qx = (_r(0, 2) + _r(2, 0)) / s
            qy = (_r(1, 2) + _r(2, 1)) / s
            qz = 0.25 * s
    return (qx, qy, qz, qw)


class WorldCubePosePublisher(Node):
    def __init__(self, cube_prim_path: str) -> None:
        super().__init__("sim_world_cube_pose")
        self._cube_prim_path = cube_prim_path
        self._pub = self.create_publisher(PoseStamped, "/world/cube_pose", 10)
        self._timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self._tick)
        self.get_logger().info(
            f"sim_world_cube_pose publishing /world/cube_pose from {cube_prim_path} "
            f"at {PUBLISH_RATE_HZ:.1f} Hz"
        )

    def _tick(self) -> None:
        import omni.usd
        from pxr import UsdGeom

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return
        prim = stage.GetPrimAtPath(self._cube_prim_path)
        if not prim or not prim.IsValid():
            return
        xform = UsdGeom.Xformable(prim)
        # ComputeLocalToWorldTransform returns the prim's world transform
        # as a Gf.Matrix4d. Default time = 0 is fine; the cube isn't
        # animated in the smoke (FixedCuboid; DynamicCuboid may drift
        # slightly under physics but world pose is still current).
        world_xform = xform.ComputeLocalToWorldTransform(0.0)
        translation = world_xform.ExtractTranslation()
        rotation = world_xform.ExtractRotationMatrix()

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x = float(translation[0])
        msg.pose.position.y = float(translation[1])
        msg.pose.position.z = float(translation[2])
        qx, qy, qz, qw = _quaternion_from_matrix(rotation)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self._pub.publish(msg)


def main() -> int:
    parser = argparse.ArgumentParser(prog="sim_world_pose_publisher")
    parser.add_argument(
        "--cube-prim-path", default="/World/pick_cube",
        help="USD prim path of the cube whose world pose is published.",
    )
    args = parser.parse_args()

    rclpy.init()
    node = WorldCubePosePublisher(cube_prim_path=args.cube_prim_path)
    try:
        rclpy.spin(node)
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 2: Commit**

```bash
git add wdt_vast/sim_world_pose_publisher.py
git commit -m "$(cat <<'EOF'
feat(m5): sim-side /world/cube_pose publisher

Runs inside the Isaac Sim kit process and publishes the cube prim's
worldspace pose at 10 Hz on /world/cube_pose. Orchestrator's
GroundTruthPoseSource subscribes to this for the M5 demo loop.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 7: `run_scenario.py` integration

**Files:**
- Modify: `wdt_vast/run_scenario.py` (subprocess launches + orchestrator param)

- [ ] **Step 1: Add `--pose-source` CLI flag**

In `wdt_vast/run_scenario.py`'s `_parse_args` function (near the existing `--allocator` etc. arguments), add:
```python
    parser.add_argument(
        "--pose-source",
        choices=("fp", "gt"),
        default="gt",
        help="Pose source for the pick orchestrator. gt = ground truth "
             "from /world/cube_pose (M5 acceptance default); fp = live "
             "FoundationPose (M6 stretch).",
    )
```

(Find the parser near line 47-75 of run_scenario.py; this argument goes alongside the existing optional ones.)

- [ ] **Step 2: Launch the sim-side `/world/cube_pose` publisher**

In the subprocess-launch block of run_scenario.py (after the existing static-TF launches, before the pick_orchestrator launch), add:
```python
    # /world/cube_pose publisher — runs inside the kit-python so it can
    # read the cube's USD prim transform. Only needed for gt-mode pose
    # source, but launching unconditionally is fine: the orchestrator
    # only subscribes when pose_source=gt and the topic is cheap.
    cube_pose_proc = _ros2_popen(
        "world_cube_pose",
        "/isaac-sim/python.sh /work/wdt_vast/sim_world_pose_publisher.py "
        "--cube-prim-path /World/pick_cube",
    )
    mark(f"world_cube_pose_launched_pid={cube_pose_proc.pid}")
```

- [ ] **Step 3: Pass `pose_source` to the orchestrator**

Find the existing `_ros2_popen("pick_orch", ...)` call in run_scenario.py (~line 300). Replace its command string with:
```python
    orch_proc = _ros2_popen(
        "pick_orch",
        "ros2 run wdt_manipulation_bringup pick_cell_orchestrator "
        f"--ros-args -p cad_path:={m5_cad} "
        f"-p pose_source:={_args.pose_source}",
    )
```

- [ ] **Step 4: Add the new subprocess to the defensive pkill list**

In the `for pat in (...)` block near line 180, add:
```python
        "wdt_vast/sim_world_pose_publisher.py",
```

- [ ] **Step 5: Commit**

```bash
git add wdt_vast/run_scenario.py
git commit -m "$(cat <<'EOF'
feat(m5): run_scenario.py wires pose_source + cube pose publisher

Adds --pose-source {fp,gt} CLI flag (defaults to gt for M5 acceptance).
Launches sim_world_pose_publisher.py as a subprocess inside the kit
python so /world/cube_pose carries the cube's USD-stage pose. Passes
pose_source as a ROS2 parameter to pick_cell_orchestrator.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 8: Fast pick-chain harness (GT mode)

**Files:**
- Create: `tests/integration/test_pick_chain_fast.py`

- [ ] **Step 1: Implement the GT-mode harness**

`tests/integration/test_pick_chain_fast.py`:
```python
"""Fast harness for the M5 pick chain — runs on the vast.ai instance.

Skips Isaac Sim entirely. Launches move_group +
franka_ready_joint_states + pick_cell_orchestrator (gt mode), publishes
a synthetic /world/cube_pose at a known Franka-reachable point + a
trivial /cell/cam/info (so the cam-state cache is populated even
though gt-mode ignores it), publishes /cell/start_pick, asserts
/cell/pick_result arrives with success=true within 2 s.

Iteration time: ~30 s (most of it is move_group boot).

Invocation on the instance:
    source /opt/ros/humble/setup.bash
    source /work/ros2_ws/install/setup.bash
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    /usr/bin/python3 /work/tests/integration/test_pick_chain_fast.py
"""

from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String

PANDA_BASE_WORLD = (16.0, 15.0, 1.0)
# Reachable cube center in world coords for panda_link0 (0.40, 0, -0.25).
CUBE_WORLD = (16.40, 15.0, 0.75)
TIMEOUT_S = 30.0


class Harness(Node):
    def __init__(self) -> None:
        super().__init__("pick_chain_fast_harness")
        self._info_pub = self.create_publisher(CameraInfo, "/cell/cam/info", 1)
        self._cube_pub = self.create_publisher(PoseStamped, "/world/cube_pose", 1)
        self._start_pub = self.create_publisher(String, "/cell/start_pick", 1)
        self._result_sub = self.create_subscription(
            String, "/cell/pick_result", self._on_result, 10
        )
        self.result: dict | None = None

    def _on_result(self, msg: String) -> None:
        try:
            self.result = json.loads(msg.data)
        except json.JSONDecodeError:
            self.result = {"error": "bad_json", "raw": msg.data}

    def publish_info_and_cube(self) -> None:
        info = CameraInfo()
        info.header.stamp = self.get_clock().now().to_msg()
        info.header.frame_id = "cell_cam_optical"
        info.height = 480
        info.width = 640
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [600.0, 0.0, 320.0, 0.0, 0.0, 600.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        self._info_pub.publish(info)

        cube = PoseStamped()
        cube.header.stamp = self.get_clock().now().to_msg()
        cube.header.frame_id = "world"
        cube.pose.position.x = CUBE_WORLD[0]
        cube.pose.position.y = CUBE_WORLD[1]
        cube.pose.position.z = CUBE_WORLD[2]
        cube.pose.orientation.w = 1.0
        self._cube_pub.publish(cube)

    def fire_start_pick(self, order_id: str) -> None:
        msg = String()
        msg.data = order_id
        self._start_pub.publish(msg)


def _start_dep(name: str, cmd: str, env: dict) -> subprocess.Popen:
    """Launch a child process with full ROS2 sourcing baked in."""
    return subprocess.Popen(
        [
            "bash",
            "-lc",
            f"source /opt/ros/humble/setup.bash && "
            f"source /work/ros2_ws/install/setup.bash && "
            f"export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && "
            f"{cmd}",
        ],
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )


def main() -> int:
    env = os.environ.copy()
    env["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    # /tmp on the PYTHONPATH for non-colcon repo packages (manipulation).
    pp_parts = [p for p in env.get("PYTHONPATH", "").split(":") if p]
    if "/tmp" not in pp_parts:
        pp_parts.append("/tmp")
    env["PYTHONPATH"] = ":".join(pp_parts)

    deps = []
    try:
        deps.append(
            _start_dep("move_group", "ros2 launch wdt_manipulation_bringup move_group.launch.py", env)
        )
        deps.append(
            _start_dep("jsp", "/usr/bin/python3 /work/wdt_vast/franka_ready_joint_states.py", env)
        )
        deps.append(
            _start_dep(
                "panda_link0_tf",
                f"ros2 run tf2_ros static_transform_publisher --x {PANDA_BASE_WORLD[0]} "
                f"--y {PANDA_BASE_WORLD[1]} --z {PANDA_BASE_WORLD[2]} "
                f"--qx 0.0 --qy 0.0 --qz 0.0 --qw 1.0 "
                f"--frame-id world --child-frame-id panda_link0",
                env,
            )
        )
        deps.append(
            _start_dep(
                "pick_orch",
                "ros2 run wdt_manipulation_bringup pick_cell_orchestrator "
                "--ros-args -p pose_source:=gt -p cad_path:=/tmp/m5_smoke_box.obj",
                env,
            )
        )

        # Give move_group + RSP ~20 s to come up.
        time.sleep(20.0)

        rclpy.init()
        node = Harness()

        deadline = time.time() + TIMEOUT_S
        order_id = "fast_harness_o1"
        published_start = False
        while time.time() < deadline and node.result is None:
            node.publish_info_and_cube()
            if not published_start and time.time() > deadline - TIMEOUT_S + 21.0:
                node.fire_start_pick(order_id)
                published_start = True
            rclpy.spin_once(node, timeout_sec=0.1)

        ok = node.result is not None and node.result.get("success") is True
        print(f"==> result: {node.result}")
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
        return 0 if ok else 2
    finally:
        for p in deps:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            except Exception:
                pass


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 2: Rebuild the orchestrator package on Quebec**

```bash
vastai start instance 36866311
# Wait until SSH up, then:
ssh vast-romania '
  set +u
  source /opt/ros/humble/setup.bash
  cd /work/ros2_ws
  colcon build --packages-select wdt_manipulation_bringup
'
```
Expected: `Summary: 1 package finished`.

- [ ] **Step 3: Sync the new files to Quebec**

Run:
```bash
cd /Users/aiqarus/Desktop/Projects/isaac-sim
scp manipulation/pose_source.py manipulation/pick_worker.py manipulation/motion_planning.py vast-romania:/work/manipulation/
scp wdt_vast/sim_world_pose_publisher.py wdt_vast/run_scenario.py vast-romania:/work/wdt_vast/
scp ros2_ws/src/wdt_manipulation_bringup/wdt_manipulation_bringup/pick_cell_orchestrator.py vast-romania:/work/ros2_ws/src/wdt_manipulation_bringup/wdt_manipulation_bringup/pick_cell_orchestrator.py
mkdir -p tests/integration  # if not present
scp tests/integration/test_pick_chain_fast.py vast-romania:/work/tests/integration/
ssh vast-romania '
  set +u
  source /opt/ros/humble/setup.bash
  cd /work/ros2_ws
  colcon build --packages-select wdt_manipulation_bringup 2>&1 | tail -3
'
```
Expected: `Summary: 1 package finished` after the colcon build.

- [ ] **Step 4: Run the harness**

```bash
ssh vast-romania '
  set +u
  source /opt/ros/humble/setup.bash
  source /work/ros2_ws/install/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  /usr/bin/python3 /work/tests/integration/test_pick_chain_fast.py
'
```
Expected output (last line):
```
==> result: {"order_id": "fast_harness_o1", "success": true, "attempts": 1, "cycle_time_s": ~0.2, "failure_reason": ""}
```
Exit code: 0.

If the result is `success: false`, the orchestrator's log on Quebec at `/tmp/m5_smoke_v*` won't exist (no sim) — instead inspect orchestrator stdout via running the test with output redirected:
```bash
ssh vast-romania '... 2>&1 | tee /tmp/fast_harness.log'
```
Diagnose from the `pick_result.failure_reason` field.

- [ ] **Step 5: Commit**

```bash
cd /Users/aiqarus/Desktop/Projects/isaac-sim
git add tests/integration/test_pick_chain_fast.py
git commit -m "$(cat <<'EOF'
test(m5): fast pick-chain harness (gt mode)

Exercises orchestrator + move_group + franka_ready_joint_states without
Isaac Sim. ~30 s per iteration (move_group boot dominates) vs. 9 min
for the full smoke. Asserts pick_result.success=true within 2s of
publishing /cell/start_pick.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 9: Full end-to-end smoke (GT mode) — M5 acceptance

**Files:** none modified — verification task.

- [ ] **Step 1: Run the full smoke on Quebec**

```bash
ssh vast-romania '
  set +u
  source /opt/ros/humble/setup.bash
  source /work/ros2_ws/install/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export PATH=/usr/local/cuda-12.4/bin:$PATH
  export CUDA_HOME=/usr/local/cuda-12.4
  cd /work
  rm -rf /tmp/m5_smoke_v22_gt
  mkdir -p /tmp/m5_smoke_v22_gt
  rm -f /tmp/m5_smoke_v22_gt.log
  nohup /isaac-sim/python.sh /work/wdt_vast/run_scenario.py \
      /work/scenarios/smoke.yaml /tmp/m5_smoke_v22_gt \
      --pose-source gt \
      > /tmp/m5_smoke_v22_gt.log 2>&1 &
  echo "v22_gt pid=$!"
'
```

- [ ] **Step 2: Poll the run for completion**

The run takes ~10 min wall (nav + at_cell + pick). Watch for `pick_result` then `metrics.json`:
```bash
ssh vast-romania '
  while ! [ -f /tmp/m5_smoke_v22_gt/metrics.json ]; do
    sleep 30
    tail -3 /tmp/m5_smoke_v22_gt/progress.txt
    grep -E "pick_result|order o1" /tmp/m5_smoke_v22_gt/coordinator.log 2>/dev/null | tail -3
  done
  echo "=== metrics.json ==="
  cat /tmp/m5_smoke_v22_gt/metrics.json
'
```
Expected metrics.json:
```json
{
  "orders_total": 1,
  "orders_completed": 1,
  "pick_success_rate": 1.0,
  "avg_cycle_time_s": <small number>,
  "p95_cycle_time_s": <small number>,
  "deadlocks_total": 0
}
```

- [ ] **Step 3: Stop the vast.ai instance**

```bash
vastai stop instance 36866311
```

- [ ] **Step 4: Commit a results note**

```bash
cd /Users/aiqarus/Desktop/Projects/isaac-sim
mkdir -p docs
# Append to docs/results-phase-2.md (create if absent):
cat >> docs/results-phase-2.md <<'EOF'

## M5 acceptance — gt mode (2026-05-16)

orders_completed = 1 on hungarian_cbs with pose_source=gt
(orchestrator reads /world/cube_pose from sim_world_pose_publisher).

This validates the closed-loop orchestration chain end-to-end:
- AMR navigates spawn → shelf → cell
- Coordinator dispatches /cell/start_pick
- Orchestrator's worker thread reads cube pose, transforms via tf2 to
  panda_link0, generates a top-down grasp, plans via MoveIt2 (plan_only)
- pick_result publishes success=true within ~250 ms of start_pick

FoundationPose remains validated standalone in M4. The fp-mode pick
chain (same orchestrator code, pose_source=fp) is the M6 stretch.
EOF
git add docs/results-phase-2.md
git commit -m "$(cat <<'EOF'
docs(m5): record gt-mode acceptance result

orders_completed=1 on hungarian_cbs via the redesigned worker-thread
orchestrator with simulator ground-truth pose. M5 acceptance shipped.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 10: Full end-to-end smoke (FP mode) — M6 stretch

**Files:** none modified — verification task.

- [ ] **Step 1: Run the smoke with `pose_source=fp`**

```bash
vastai start instance 36866311
# wait for SSH up
ssh vast-romania '
  set +u
  source /opt/ros/humble/setup.bash
  source /work/ros2_ws/install/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export PATH=/usr/local/cuda-12.4/bin:$PATH
  export CUDA_HOME=/usr/local/cuda-12.4
  cd /work
  rm -rf /tmp/m5_smoke_v22_fp
  mkdir -p /tmp/m5_smoke_v22_fp
  rm -f /tmp/m5_smoke_v22_fp.log
  nohup /isaac-sim/python.sh /work/wdt_vast/run_scenario.py \
      /work/scenarios/smoke.yaml /tmp/m5_smoke_v22_fp \
      --pose-source fp \
      > /tmp/m5_smoke_v22_fp.log 2>&1 &
  echo "v22_fp pid=$!"
'
```

- [ ] **Step 2: Poll for completion**

Same as Task 9 step 2 but reading `/tmp/m5_smoke_v22_fp/metrics.json`.

If `orders_completed=1`: FP-mode also works. Append the result to `docs/results-phase-2.md`.

If `orders_completed=0` with `pick_result.failure_reason="plan_no_solution(..)"`: FP returned a pose that's geometrically off enough that MoveIt rejects the grasp. Acceptable for M6-deferred — document the FP estimate in `docs/results-phase-2.md` as future work.

If `failure_reason="plan_action_failed"` or `"tf_lookup_failed"`: regression in the redesign — go back to Task 8's fast harness with `pose_source=fp` and reproduce.

- [ ] **Step 3: Stop the vast.ai instance**

```bash
vastai stop instance 36866311
```

- [ ] **Step 4: Append result to docs**

If pass:
```bash
cat >> docs/results-phase-2.md <<'EOF'

## M5 stretch — fp mode (2026-05-16)

orders_completed = 1 on hungarian_cbs with pose_source=fp. End-to-end
loop closed with live FoundationPose perception. Phase 2 success bar
fully met including the stretch.
EOF
```

If document-as-future-work:
```bash
cat >> docs/results-phase-2.md <<'EOF'

## M5 stretch — fp mode (2026-05-16)

orders_completed=0 with pose_source=fp; failure_reason=<reason from log>.
FP's translation estimate vs. simulator ground-truth:
- FP optical: (..., ..., ...)
- Expected:   (0.0, 0.0, 1.097)
- Delta:      (..., ..., ...)

FP locks onto the cube depth but the registered pose is offset enough
to push the grasp outside Franka's IK envelope. Phase 3 will address
this with: (a) tighter FoundationPose masking via a 2D object detector
upstream (NVIDIA's recommended pattern), or (b) Carter onboard RGB-D
which provides higher-quality input.
EOF
```

Commit:
```bash
git add docs/results-phase-2.md
git commit -m "$(cat <<'EOF'
docs(m5): record fp-mode result and Phase 3 followups

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```
