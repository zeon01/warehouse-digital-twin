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

    def __init__(
        self,
        auto_mask_from_nearest: bool = True,
        nearest_depth_window_m: float = 0.15,
    ) -> None:
        self._scorer: Any = None
        self._refiner: Any = None
        self._glctx: Any = None
        self._impl: Any = None
        self._last_cad: str | None = None
        # Mask strategy: with a cluttered scene (table + cube + walls), a
        # full-image mask makes FoundationPose register the CAD against
        # the dominant depth surface — which in M5 v18 was the table, not
        # the 8 cm cube on top of it. By construction the pick object is
        # the CLOSEST surface to the camera; mask all pixels within a
        # tight window above the minimum-depth pixel and FP focuses on
        # just the cube. Set auto_mask_from_nearest=False to fall back to
        # the legacy full-image mask (only safe when the depth image has
        # no other surfaces — e.g. M4's synthetic test fixture).
        self._auto_mask_from_nearest = auto_mask_from_nearest
        self._nearest_depth_window_m = nearest_depth_window_m

    def _lazy_load(self) -> None:
        if self._scorer is not None:
            return
        # FoundationPose isn't pip-installable — wdt_vast/install_foundationpose.sh
        # drops a .pth file pointing at /opt/foundationpose/src, but some
        # embedded interpreters skip .pth processing. Belt-and-suspenders:
        # ensure the src dir is on sys.path here too.
        import sys

        fp_src = "/opt/foundationpose/src"
        if fp_src not in sys.path:
            sys.path.insert(0, fp_src)

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

        if self._auto_mask_from_nearest:
            # Build a mask of pixels within `nearest_depth_window_m` of the
            # closest visible depth — i.e. the cube top + sides, excluding
            # the table around it and the floor behind. M5 v18 verified
            # without this: FP returned panda_link0 (0.39, 0.49, -0.71)
            # — the table surface, NOT the cube center at (0.40, 0, -0.25).
            valid = np.isfinite(depth) & (depth > 0)
            if not valid.any():
                return []
            dmin = float(depth[valid].min())
            mask = valid & (depth <= dmin + self._nearest_depth_window_m)
            # Sanity: need at least a few hundred pixels for FP's 160-px
            # crop to make sense.
            if int(mask.sum()) < 64:
                mask = np.ones(depth.shape[:2], dtype=bool)
        else:
            # Full-image mask — only safe when the depth image has a single
            # object on a clean background (e.g. M4's synthetic fixture).
            mask = np.ones(depth.shape[:2], dtype=bool)
        # trimesh loads mesh.vertices as float64 by default and FP's
        # internal pose math runs in double. Cast K to float64 to match
        # — without this FP's `K @ pts` raises
        # "RuntimeError: expected mat1 and mat2 to have the same
        # dtype, but got: float != double" (verified 2026-05-16 on
        # Quebec 36866311 with a 480x640 K from the smoke).
        K64 = np.asarray(camera_K, dtype=np.float64)
        pose = self._impl.register(
            K=K64,
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
