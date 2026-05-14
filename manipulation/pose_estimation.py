"""Wrapper around FoundationPose (Isaac ROS package) for zero-shot 6-DoF pose.

The wrapper is import-safe on systems without the FoundationPose package
installed — the dependency is loaded lazily on first `.estimate()` call.
This lets the test suite run on a stock Mac without pulling the full
Isaac ROS stack.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class PoseResult:
    translation: np.ndarray  # shape (3,)
    rotation: np.ndarray  # shape (3, 3)
    score: float


class PoseEstimator:
    def __init__(self, model_dir: str):
        self.model_dir = model_dir
        self._impl = None

    def _lazy_load(self):
        if self._impl is not None:
            return
        # FoundationPose ships as a Python module inside Isaac ROS.
        # If the upstream module name differs from the install you get,
        # check /opt/ros/humble/lib/python3.10/site-packages/ in the
        # container (ls | grep -i foundation) and adjust the import.
        from isaac_ros_foundationpose import FoundationPose  # type: ignore[import]

        self._impl = FoundationPose(model_dir=self.model_dir)

    def estimate(
        self,
        rgb: np.ndarray,
        depth: np.ndarray,
        cad_path: str,
        camera_K: np.ndarray,
    ) -> list[PoseResult]:
        self._lazy_load()
        result = self._impl.run(rgb=rgb, depth=depth, mesh_path=cad_path, K=camera_K)
        out: list[PoseResult] = []
        for pose, score in zip(result.poses, result.scores, strict=False):
            out.append(
                PoseResult(
                    translation=np.asarray(pose[:3, 3], dtype=np.float32),
                    rotation=np.asarray(pose[:3, :3], dtype=np.float32),
                    score=float(score),
                )
            )
        return out
