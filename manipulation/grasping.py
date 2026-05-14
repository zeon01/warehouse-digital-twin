"""Wrapper around AnyGrasp (graspnet-baseline) for top-K grasp candidates.

Like the FoundationPose wrapper, the upstream AnyGrasp package is loaded
lazily on first `.propose()` call so the test suite can import this on a
stock Mac without pulling the model weights or CUDA build.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class GraspCandidate:
    translation: np.ndarray  # (3,)
    rotation: np.ndarray  # (3, 3)
    width: float  # gripper width in meters
    score: float  # higher = better


class GraspGenerator:
    def __init__(self, model_dir: str, top_k: int = 5):
        self.model_dir = model_dir
        self.top_k = top_k
        self._impl = None

    def _lazy_load(self):
        if self._impl is not None:
            return
        from anygrasp import AnyGrasp  # type: ignore[import]

        self._impl = AnyGrasp(model_dir=self.model_dir, max_gripper_width=0.08)

    def propose(self, depth: np.ndarray, camera_K: np.ndarray) -> list[GraspCandidate]:
        self._lazy_load()
        result = self._impl.propose(depth=depth, K=camera_K)
        scored = sorted(result, key=lambda g: -g.score)[: self.top_k]
        return [
            GraspCandidate(
                translation=np.asarray(g.t, dtype=np.float32),
                rotation=np.asarray(g.R, dtype=np.float32),
                width=float(g.width),
                score=float(g.score),
            )
            for g in scored
        ]
