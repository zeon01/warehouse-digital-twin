from pathlib import Path

import numpy as np
import pytest

from manipulation.pose_estimation import PoseEstimator, PoseResult

FIXTURE = Path(__file__).parent.parent / "fixtures" / "cell_cam_rgbd"


@pytest.mark.skipif(
    not (FIXTURE / "rgb.npy").exists(),
    reason="fixture missing — drop RGB-D sample at tests/fixtures/cell_cam_rgbd/",
)
def test_pose_estimator_returns_one_pose_for_known_fixture():
    est = PoseEstimator(model_dir="/vol/models/foundationpose")
    rgb = np.load(FIXTURE / "rgb.npy")
    depth = np.load(FIXTURE / "depth.npy")
    cad = FIXTURE / "object.obj"
    results = est.estimate(rgb=rgb, depth=depth, cad_path=str(cad), camera_K=np.eye(3))
    assert results
    assert isinstance(results[0], PoseResult)
    assert results[0].translation.shape == (3,)
