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
        poses = self._estimator.estimate(rgb=rgb, depth=depth, cad_path=cad_path, camera_K=camera_K)
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
