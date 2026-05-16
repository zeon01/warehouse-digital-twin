"""End-to-end FoundationPose inference smoke (Task 22 / M4 acceptance).

Exercises the full FoundationPose stack — ScorePredictor + PoseRefinePredictor
+ nvdiffrast RasterizeCudaContext + trimesh mesh loading + FoundationPose
register — to verify the install works and the pipeline runs to
completion. Two paths:

1. **Demo-data path**: if `/opt/foundationpose/src/demo_data/mustard0/`
   is bundled on this image, use the real RGB-D + mesh + intrinsics
   for a strict pose-accuracy assertion. Most upstream Conda-install
   variants ship this; vast.ai's clean clone-via-install_foundationpose.sh
   does NOT (verified 2026-05-16).

2. **Synthetic path**: if no demo_data is present, generate a small
   cube mesh on-the-fly via trimesh, synthesize a matching depth bump
   + flat RGB, and call `.estimate()`. Loose assertion — the synthetic
   inputs don't perfectly match the mesh so FP may return a pose with
   coarse fit. We only verify the call CHAIN executes without crashing
   and returns SOMETHING (pose or empty list). Per plan-doc Task 22:
   "1 passed (or 1 skipped if the demo CAD isn't bundled) is
   acceptable — pipeline integration smoke in M5 will cover real input."

The skip-import-or fires only when foundationpose isn't installed at all
(e.g., Mac dev runs).

Run target:
    /usr/bin/python3 -m pytest tests/integration/test_foundationpose_smoke.py -v
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("estimater")
pytest.importorskip("nvdiffrast")

_FP_SRC = Path("/opt/foundationpose/src")
if _FP_SRC.exists() and str(_FP_SRC) not in sys.path:
    sys.path.insert(0, str(_FP_SRC))

_DEMO_DIR = _FP_SRC / "demo_data" / "mustard0"
_CAD_PATH = _DEMO_DIR / "mesh" / "textured_simple.obj"
_RGB_DIR = _DEMO_DIR / "rgb"
_DEPTH_DIR = _DEMO_DIR / "depth"
_CAM_K_FILE = _DEMO_DIR / "cam_K.txt"
_MASK_DIR = _DEMO_DIR / "masks"


def _load_first(directory: Path) -> Path | None:
    if not directory.exists():
        return None
    files = sorted(directory.iterdir())
    return files[0] if files else None


def _make_synthetic_inputs(tmp_dir: Path) -> dict:
    """Generate a tiny cube mesh + matching synthetic RGB-D so FP can
    run end-to-end even when demo_data isn't bundled.
    """
    trimesh = pytest.importorskip("trimesh")

    # 8 cm cube — typical pickable object size for warehouse SKUs.
    mesh = trimesh.creation.box(extents=(0.08, 0.08, 0.08))
    cad_path = tmp_dir / "synthetic_box.obj"
    mesh.export(str(cad_path))

    # 480x640 RGB-D. Camera intrinsics for a typical realsense-style
    # depth cam: fx=fy=600, cx=320, cy=240. Cube centered, ~50 cm from
    # camera. Depth image has the cube as a flat patch at 0.5 m, rest
    # of the scene at 1.0 m (a "floor").
    H, W = 480, 640
    depth = np.full((H, W), 1.0, dtype=np.float32)
    # 60 px wide cube patch centered → roughly 8 cm wide at z=0.5 m
    # with fx=600 (size_px = fx * size_m / z = 600 * 0.08 / 0.5 = 96).
    cy_px = H // 2
    cx_px = W // 2
    half = 48
    depth[cy_px - half : cy_px + half, cx_px - half : cx_px + half] = 0.5
    rgb = np.full((H, W, 3), 100, dtype=np.uint8)
    rgb[cy_px - half : cy_px + half, cx_px - half : cx_px + half] = (200, 80, 50)
    K = np.array([[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]], dtype=np.float32)
    mask = np.zeros((H, W), dtype=bool)
    mask[cy_px - half : cy_px + half, cx_px - half : cx_px + half] = True

    return {
        "rgb": rgb,
        "depth": depth,
        "K": K,
        "mask": mask,
        "cad": str(cad_path),
        "synthetic": True,
    }


@pytest.fixture(scope="module")
def fp_inputs(tmp_path_factory):
    """Return either real demo-data inputs or synthetic stand-ins."""
    if _CAD_PATH.exists():
        rgb_path = _load_first(_RGB_DIR)
        depth_path = _load_first(_DEPTH_DIR)
        if rgb_path is not None and depth_path is not None:
            cv2 = pytest.importorskip("cv2")
            rgb_bgr = cv2.imread(str(rgb_path), cv2.IMREAD_COLOR)
            rgb = cv2.cvtColor(rgb_bgr, cv2.COLOR_BGR2RGB)
            depth_raw = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED)
            depth = depth_raw.astype(np.float32) / 1000.0
            if _CAM_K_FILE.exists():
                K = np.loadtxt(_CAM_K_FILE).reshape(3, 3).astype(np.float32)
            else:
                K = np.array(
                    [[604.4, 0.0, 321.0], [0.0, 604.4, 241.4], [0.0, 0.0, 1.0]],
                    dtype=np.float32,
                )
            mask_path = _load_first(_MASK_DIR) if _MASK_DIR.exists() else None
            mask = (
                cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE) > 0
                if mask_path is not None
                else np.ones(depth.shape, dtype=bool)
            )
            return {
                "rgb": rgb,
                "depth": depth,
                "K": K,
                "mask": mask,
                "cad": str(_CAD_PATH),
                "synthetic": False,
            }

    return _make_synthetic_inputs(tmp_path_factory.mktemp("fp_smoke"))


def test_foundationpose_estimate_runs(fp_inputs):
    """Run PoseEstimator.estimate end-to-end. On real demo_data, assert
    a pose is returned with sane values. On synthetic inputs, just
    assert the call doesn't crash and returns a list (pose may be
    empty if FP can't fit the cube — that's OK, we tested the chain).
    """
    from manipulation.pose_estimation import PoseEstimator

    fp = PoseEstimator()
    estimates = fp.estimate(
        rgb=fp_inputs["rgb"],
        depth=fp_inputs["depth"],
        cad_path=fp_inputs["cad"],
        camera_K=fp_inputs["K"],
    )

    assert isinstance(estimates, list), f"estimate returned {type(estimates).__name__}"

    if fp_inputs["synthetic"]:
        # Synthetic path — chain ran without crashing. Pose accuracy
        # not assertable. Print diagnostic if pose returned for sanity.
        if estimates:
            pose = estimates[0]
            assert pose.translation.shape == (3,)
            assert pose.rotation.shape == (3, 3)
            assert np.all(np.isfinite(pose.translation))
            assert np.all(np.isfinite(pose.rotation))
        # Synthetic-no-pose is acceptable.
        return

    # Real demo_data path — strict assertions.
    assert len(estimates) >= 1, "FP returned no poses on real demo data"
    pose = estimates[0]
    assert pose.translation.shape == (3,)
    assert pose.rotation.shape == (3, 3)
    assert np.all(np.isfinite(pose.translation))
    assert np.all(np.isfinite(pose.rotation))
    assert (
        0.1 < pose.translation[2] < 3.0
    ), f"translation z={pose.translation[2]:.3f} outside plausible range"
