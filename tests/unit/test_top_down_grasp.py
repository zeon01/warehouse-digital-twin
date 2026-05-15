"""Unit tests for the deterministic top-down grasp generator."""

from __future__ import annotations

import numpy as np


def test_top_down_grasp_at_pose():
    from manipulation.grasping import TopDownGrasp

    gen = TopDownGrasp(standoff_m=0.05)
    depth = np.ones((480, 640), dtype=np.float32) * 1.0
    K = np.array([[600.0, 0, 320], [0, 600.0, 240], [0, 0, 1]])

    pose_translation = np.array([0.1, 0.2, 0.3])
    candidates = gen.propose_at(translation=pose_translation, depth=depth, camera_K=K)

    assert len(candidates) == 1
    c = candidates[0]
    np.testing.assert_allclose(c.translation, [0.1, 0.2, 0.35], atol=1e-6)
    np.testing.assert_allclose(c.rotation[:, 2], [0, 0, -1], atol=1e-6)


def test_top_down_grasp_returns_one_candidate():
    from manipulation.grasping import TopDownGrasp

    gen = TopDownGrasp()
    cands = gen.propose_at(
        translation=np.zeros(3),
        depth=np.ones((10, 10), dtype=np.float32),
        camera_K=np.eye(3),
    )
    assert len(cands) == 1
    assert cands[0].score == 1.0
    assert cands[0].width == 0.08
