from pathlib import Path

import numpy as np
import pytest

from manipulation.grasping import GraspCandidate, GraspGenerator

FIXTURE = Path(__file__).parent.parent / "fixtures" / "cell_cam_rgbd"


@pytest.mark.skipif(
    not (FIXTURE / "depth.npy").exists(),
    reason="fixture missing — drop depth sample at tests/fixtures/cell_cam_rgbd/",
)
def test_grasp_generator_returns_topk_candidates():
    gen = GraspGenerator(model_dir="/vol/models/anygrasp", top_k=5)
    depth = np.load(FIXTURE / "depth.npy")
    cands = gen.propose(depth=depth, camera_K=np.eye(3))
    assert 0 < len(cands) <= 5
    assert all(isinstance(c, GraspCandidate) for c in cands)
