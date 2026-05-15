# FoundationPose Integration (Phase 2)

Phase 1 stubbed `manipulation.pose_estimation.PoseEstimator` against the
Isaac-ROS-bundled `isaac_ros_foundationpose` wrapper. Phase 2 swaps to
the raw upstream package from NVlabs/FoundationPose because that wrapper
isn't shipped on the vast.ai Isaac Sim image we deploy to.

## Pin

| Field | Value |
|---|---|
| Upstream | <https://github.com/NVlabs/FoundationPose> |
| Pinned commit | `a1b694b83e633c2cb6115b9063d940a687759392` (2026-04-29) |
| Pinned date | 2026-05-15 |
| Refiner weights folder | `2023-10-28-18-33-37/` |
| Scorer weights folder | `2024-01-11-20-02-45/` |

The pinned commit is the upstream HEAD as of audit time. It refactored
the local install flow to use conda + pybind11 for `mycpp`; the older
nvdiffrast-via-wheel and `mycpp/mycuda` CUDA build steps are no longer
needed (mycuda is optional and disabled by default).

## Runtime stack on vast.ai

- NVIDIA driver: 570.211 (Romania A5000 instance)
- CUDA toolkit: 12.4 (matches driver, bundled in the Isaac Sim 5.0 image)
- PyTorch: 2.1.0+cu124 (installed into Isaac Sim's `python.sh` env)
- Python: 3.10 (bundled with Isaac Sim 5.0)

## Distribution strategy

The upstream's `mycpp` extension now builds with cmake + pybind11 in
seconds, so the original "compile CUDA wheels on Modal, ship to vast.ai"
detour is gone — we install directly on vast.ai from source. Modal's
role shrinks to **only** caching the ~2 GB of model weights so the
vast.ai instance doesn't have to authenticate against Google Drive
each time.

| Step | Where | How | Plan task |
|---|---|---|---|
| Cache weights to Modal volume | Modal (CPU) | `modal run wdt_modal/stage_foundationpose_weights.py` (one-time) | Task 20 |
| Install on vast.ai (apt deps + source + mycpp build + pip + weights) | vast.ai instance | `bash wdt_vast/install_foundationpose.sh` | Task 21 |
| Wire into pipeline | Isaac Sim python | `manipulation.pose_estimation.PoseEstimator` (Task 22) | Task 22 |

### Modal weight cache

The weights live in two folders inside FoundationPose's repo-relative
`weights/` directory:

```
<src>/weights/
  2023-10-28-18-33-37/         # refiner network
    model_best.pth
    config.yml
  2024-01-11-20-02-45/         # scorer network
    model_best.pth
    config.yml
```

`wdt_modal/stage_foundationpose_weights.py` uses `gdown` to mirror the
upstream's "anyone with link" Google Drive folders to a Modal volume.
If gdown fails (Drive throttling, ToS changes), the fallback is
`modal volume put` from a local manual download — instructions are in
the script's docstring.

### vast.ai-side install

`install_foundationpose.sh` runs in this order:

1. **apt deps:** `cmake ninja-build build-essential libeigen3-dev libboost-all-dev`
2. **Clone + pin** the upstream at commit `a1b694b8` into `/opt/foundationpose/src`.
3. **Build mycpp** via cmake/pybind11 (~30s).
4. **pip install** PyTorch (cu124 index), `nvdiffrast`, FoundationPose's
   `requirements.txt`, and finally `pip install -e /opt/foundationpose/src`.
5. **Pull weights** from the Modal volume via `modal volume get`,
   extracting under `/opt/foundationpose/src/weights/`.
6. **Verify** with `from foundationpose.estimater import FoundationPose`.

## Wrapper API (`manipulation/pose_estimation.py`)

```python
class PoseEstimator:
    def __init__(self): ...
    def estimate(
        self,
        rgb: np.ndarray,      # HxWx3 uint8
        depth: np.ndarray,    # HxW float32 meters
        cad_path: str,        # path to .obj — loaded with trimesh, cached
        camera_K: np.ndarray, # 3x3 intrinsics
    ) -> list[PoseResult]
```

The wrapper:
- Lazy-loads ScorePredictor + PoseRefinePredictor + nvdiffrast context on first call.
- Loads the CAD mesh with trimesh and caches by path. If `cad_path`
  changes between calls (different SKU), calls `est.reset_object()`
  instead of constructing a new FoundationPose.
- Uses a full-image mask (the pick cell has only the target object
  visible) and the upstream's 5-iteration refinement.

## Fallback strategies

| Failure | Fallback |
|---|---|
| gdown rate-limited or Drive ACL changed | Manual: download both folders locally, then `modal volume put foundationpose-models weights/<run_name> /weights/<run_name>` |
| mycpp build fails (missing system pybind11) | `apt install python3-pybind11`, retry. Older Eigen also fine — the upstream uses conda's eigen but apt eigen3-dev works for our build. |
| PyTorch ABI mismatch with mycpp | Rebuild mycpp against the runtime python: `cd /opt/foundationpose/src/mycpp && rm -rf build && bash ../build_all_conda.sh` (the conda var override falls through to system) |
| FoundationPose CUDA OOM on A5000 (24 GB) | Reduce `iteration` argument from 5 to 3 in `pose_estimation.estimate()` |
