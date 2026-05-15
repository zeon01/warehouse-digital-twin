"""Grasp candidate generators.

Two implementations live here:

- ``GraspGenerator``: AnyGrasp wrapper (Phase 1). Lazy-loads the
  upstream AnyGrasp package on first ``.propose()``. Unused in Phase 2
  because the AnyGrasp license + CUDA build cost wasn't worth it (see
  ``docs/superpowers/specs/2026-05-15-warehouse-digital-twin-phase-2-design.md``
  §3 decisions log) but kept for Phase 3 swap-in.
- ``TopDownGrasp`` (Phase 2): deterministic grasp at a known object
  pose. Gripper points world-down, wrist offset by ``standoff_m``
  above the object. Defensible for warehouse SKUs whose pose is known
  from FoundationPose. Use the ``TopDownGraspFromPose`` adapter to
  satisfy the ``ManipulationPipeline.pick`` duck-typed interface.
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


class TopDownGrasp:
    """Deterministic top-down grasp at a known object pose.

    Returns one ``GraspCandidate`` with the gripper Z-axis pointing
    world-down and the wrist translated ``standoff_m`` above the
    object's pose. Phase 2 uses this in place of AnyGrasp — warehouse
    SKUs are constrained enough that a known-pose top-down grasp is
    defensible, and it sidesteps the AnyGrasp research-use license +
    CUDA build cost.

    The standoff is applied along world +Z. The gripper rotation is
    chosen with X = world X, Y = world -Y, Z = world -Z so the right-
    handed gripper frame ends up pointing down.
    """

    def __init__(self, standoff_m: float = 0.05, gripper_width: float = 0.08):
        self.standoff_m = standoff_m
        self.gripper_width = gripper_width

    def propose_at(
        self,
        translation: np.ndarray,
        depth: np.ndarray,  # unused; kept for interface symmetry with GraspGenerator
        camera_K: np.ndarray,  # unused; kept for interface symmetry
    ) -> list[GraspCandidate]:
        grasp_t = translation.astype(np.float32).copy()
        grasp_t[2] += self.standoff_m
        rotation = np.array(
            [
                [1.0, 0.0, 0.0],
                [0.0, -1.0, 0.0],
                [0.0, 0.0, -1.0],
            ],
            dtype=np.float32,
        )
        return [
            GraspCandidate(
                translation=grasp_t,
                rotation=rotation,
                width=self.gripper_width,
                score=1.0,
            )
        ]

    def propose(self, depth: np.ndarray, camera_K: np.ndarray) -> list[GraspCandidate]:
        # ManipulationPipeline expects propose(depth, K) — but TopDownGrasp
        # needs the object pose. Compose with TopDownGraspFromPose instead.
        raise NotImplementedError(
            "TopDownGrasp.propose() requires a pose; use propose_at() directly "
            "or wrap with TopDownGraspFromPose."
        )


class TopDownGraspFromPose:
    """Adapter that binds a pose to ``TopDownGrasp`` so it satisfies the
    ``ManipulationPipeline``'s duck-typed ``propose(depth, K) -> list``
    interface. Used by ``pick_cell_orchestrator``.
    """

    def __init__(self, inner: TopDownGrasp, pose):
        self._inner = inner
        self._pose = pose

    def propose(self, depth: np.ndarray, camera_K: np.ndarray) -> list[GraspCandidate]:
        return self._inner.propose_at(
            translation=self._pose.translation,
            depth=depth,
            camera_K=camera_K,
        )
