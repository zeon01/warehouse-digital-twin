"""Unit tests for manipulation.pose_source — PoseSource implementations."""

from __future__ import annotations

import numpy as np

from manipulation.pose_source import FoundationPosePoseSource, GroundTruthPoseSource


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
