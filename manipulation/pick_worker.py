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
from collections.abc import Callable
from dataclasses import dataclass
from time import perf_counter
from typing import Any

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
            self._publish_result(PickResult(req.order_id, False, 0, perf_counter() - t0, "no_pose"))
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
