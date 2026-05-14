"""Compose pose → grasp → motion plan with bounded retries.

The pipeline takes three injected components (any class that implements the
respective interfaces from `pose_estimation`, `grasping`, `motion_planning`)
and runs:

    1. Estimate 6D pose of the target object from RGB-D.
    2. Generate top-K grasp candidates from the depth image.
    3. Try each candidate in score order via the arm planner; first success
       returns immediately. After `max_retries` failures, return failure.

Injection makes the pipeline testable with mocks (see tests/unit/test_pipeline.py).
"""

from __future__ import annotations

from dataclasses import dataclass
from time import perf_counter

import numpy as np


@dataclass
class PickResult:
    success: bool
    attempts: int
    cycle_time_s: float
    failure_reason: str = ""


class ManipulationPipeline:
    def __init__(self, pose_estimator, grasp_generator, arm, max_retries: int = 3):
        self.pose_estimator = pose_estimator
        self.grasp_generator = grasp_generator
        self.arm = arm
        self.max_retries = max_retries

    def pick(
        self,
        rgb: np.ndarray,
        depth: np.ndarray,
        cad_path: str,
        camera_K: np.ndarray,
    ) -> PickResult:
        t0 = perf_counter()
        poses = self.pose_estimator.estimate(
            rgb=rgb, depth=depth, cad_path=cad_path, camera_K=camera_K
        )
        if not poses:
            return PickResult(False, 0, perf_counter() - t0, "no_pose")

        candidates = self.grasp_generator.propose(depth=depth, camera_K=camera_K)
        if not candidates:
            return PickResult(False, 0, perf_counter() - t0, "no_grasp")

        attempts = 0
        for cand in candidates[: self.max_retries]:
            attempts += 1
            res = self.arm.plan_to_pose(cand.translation, cand.rotation)
            if res.success:
                return PickResult(True, attempts, perf_counter() - t0, "")
        return PickResult(False, attempts, perf_counter() - t0, "exhausted_candidates")
