"""Wrapper around NVlabs/FoundationPose for zero-shot 6-DoF object pose.

Phase 1 used the Isaac-ROS-bundled wrapper (isaac_ros_foundationpose),
which doesn't ship on the vast.ai Isaac Sim image. Phase 2 switches to
the raw upstream package installed by
``wdt_vast/install_foundationpose.sh`` into /opt/foundationpose/.

The wrapper is import-safe on stock Macs because foundationpose is
imported inside ``_lazy_load()`` — pipeline unit tests inject mocks
and never hit this branch. See ``manipulation/FOUNDATIONPOSE.md`` for
the installation story.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass
class PoseResult:
    translation: np.ndarray  # shape (3,)
    rotation: np.ndarray  # shape (3, 3)
    score: float


class PoseEstimator:
    def __init__(self, model_dir: str = "/opt/foundationpose/checkpoints"):
        self.model_dir = model_dir
        self._impl = None

    def _lazy_load(self):
        if self._impl is not None:
            return
        # Raw NVlabs/FoundationPose. Installed on vast.ai by
        # wdt_vast/install_foundationpose.sh into /opt/foundationpose/src.
        from foundationpose import FoundationPose  # type: ignore[import]

        weights = Path(self.model_dir) / "model_best.pth"
        if not weights.exists():
            raise FileNotFoundError(
                f"FoundationPose weights not found at {weights}; run "
                f"wdt_vast/install_foundationpose.sh on this vast.ai instance"
            )
        self._impl = FoundationPose(model_pts=str(weights.parent))

    def estimate(
        self,
        rgb: np.ndarray,
        depth: np.ndarray,
        cad_path: str,
        camera_K: np.ndarray,
    ) -> list[PoseResult]:
        self._lazy_load()
        # Full-image mask — the warehouse pick cell is empty except for
        # the target object, so we let FoundationPose register against
        # the entire frame rather than a tight bbox.
        mask = np.ones(depth.shape, dtype=np.uint8) * 255
        pose = self._impl.register(
            rgb=rgb,
            depth=depth,
            K=camera_K,
            ob_in_cam=None,
            mask=mask,
            cad_path=cad_path,
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
