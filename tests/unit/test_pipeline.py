from unittest.mock import MagicMock

import numpy as np

from manipulation.grasping import GraspCandidate
from manipulation.motion_planning import ArmExecutionResult
from manipulation.pipeline import ManipulationPipeline, PickResult
from manipulation.pose_estimation import PoseResult


def _make_pipeline(plan_results):
    pose_est = MagicMock()
    pose_est.estimate.return_value = [PoseResult(np.zeros(3), np.eye(3), 0.9)]
    grasp_gen = MagicMock()
    grasp_gen.propose.return_value = [
        GraspCandidate(np.zeros(3), np.eye(3), 0.05, 0.8),
        GraspCandidate(np.zeros(3), np.eye(3), 0.05, 0.7),
        GraspCandidate(np.zeros(3), np.eye(3), 0.05, 0.6),
    ]
    arm = MagicMock()
    arm.plan_to_pose.side_effect = plan_results
    return ManipulationPipeline(
        pose_estimator=pose_est, grasp_generator=grasp_gen, arm=arm, max_retries=3
    )


def test_pipeline_succeeds_on_first_try():
    p = _make_pipeline([ArmExecutionResult(True, "ok")])
    result = p.pick(
        rgb=np.zeros((10, 10, 3), dtype=np.uint8),
        depth=np.zeros((10, 10), dtype=np.float32),
        cad_path="x.obj",
        camera_K=np.eye(3),
    )
    assert isinstance(result, PickResult)
    assert result.success is True
    assert result.attempts == 1


def test_pipeline_retries_then_fails():
    p = _make_pipeline([ArmExecutionResult(False, "f")] * 3)
    result = p.pick(
        rgb=np.zeros((10, 10, 3), dtype=np.uint8),
        depth=np.zeros((10, 10), dtype=np.float32),
        cad_path="x.obj",
        camera_K=np.eye(3),
    )
    assert result.success is False
    assert result.attempts == 3
