"""Wrapper around NVlabs/FoundationPose for zero-shot 6-DoF object pose.

Phase 1 used the Isaac-ROS-bundled wrapper (``isaac_ros_foundationpose``)
which exposed a different API. Phase 2 swaps to the raw upstream
package installed by ``wdt_vast/install_foundationpose.sh`` at commit
``a1b694b8`` (see ``manipulation/FOUNDATIONPOSE.md``).

Upstream's ``FoundationPose(model_pts, model_normals, mesh=, scorer=,
refiner=, glctx=)`` takes the mesh vertices + normals directly, NOT a
weights directory. We load the CAD mesh with trimesh on first call and
cache by path so subsequent estimates with the same SKU don't re-load
the rotation grid.

The wrapper stays import-safe on stock Macs: the upstream `foundationpose`
package only imports inside ``_lazy_load``. Pipeline unit tests inject
mocks and never hit this branch.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass
class PoseResult:
    translation: np.ndarray  # shape (3,)
    rotation: np.ndarray  # shape (3, 3)
    score: float


class PoseEstimator:
    """6-DoF pose estimator backed by FoundationPose.

    Construct once, call ``.estimate()`` per pick. If the CAD path
    changes between calls (different SKU), the impl is reset via
    ``FoundationPose.reset_object`` rather than re-constructed.
    """

    def __init__(self) -> None:
        self._scorer: Any = None
        self._refiner: Any = None
        self._glctx: Any = None
        self._impl: Any = None
        self._last_cad: str | None = None

    def _lazy_load(self) -> None:
        if self._scorer is not None:
            return
        import nvdiffrast.torch as dr  # type: ignore[import]
        from learning.training.predict_pose_refine import (  # type: ignore[import]
            PoseRefinePredictor,
        )
        from learning.training.predict_score import (  # type: ignore[import]
            ScorePredictor,
        )

        self._scorer = ScorePredictor()  # reads <src>/weights/<scorer_run>/
        self._refiner = PoseRefinePredictor()  # reads <src>/weights/<refiner_run>/
        self._glctx = dr.RasterizeCudaContext()

    def _set_object(self, cad_path: str) -> None:
        """Construct or reset the FoundationPose impl for a CAD mesh."""
        import trimesh  # type: ignore[import]
        from estimater import FoundationPose  # type: ignore[import]

        mesh = trimesh.load(cad_path)
        if self._impl is None:
            self._impl = FoundationPose(
                model_pts=mesh.vertices,
                model_normals=mesh.vertex_normals,
                mesh=mesh,
                scorer=self._scorer,
                refiner=self._refiner,
                glctx=self._glctx,
            )
        else:
            self._impl.reset_object(
                model_pts=mesh.vertices,
                model_normals=mesh.vertex_normals,
                mesh=mesh,
            )
        self._last_cad = cad_path

    def estimate(
        self,
        rgb: np.ndarray,
        depth: np.ndarray,
        cad_path: str,
        camera_K: np.ndarray,
    ) -> list[PoseResult]:
        self._lazy_load()
        if cad_path != self._last_cad:
            self._set_object(cad_path)

        # Full-image mask — the warehouse pick cell is empty except for the
        # target object, so we let FoundationPose register against the
        # entire frame rather than a tight bbox.
        mask = np.ones(depth.shape[:2], dtype=bool)
        pose = self._impl.register(
            K=camera_K,
            rgb=rgb,
            depth=depth,
            ob_mask=mask,
            iteration=5,
        )
        if pose is None:
            return []
        return [
            PoseResult(
                translation=np.asarray(pose[:3, 3], dtype=np.float32),
                rotation=np.asarray(pose[:3, :3], dtype=np.float32),
                score=1.0,
            )
        ]
