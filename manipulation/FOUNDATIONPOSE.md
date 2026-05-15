# FoundationPose Integration (Phase 2)

Phase 1 stubbed `manipulation.pose_estimation.PoseEstimator` against the
Isaac ROS bundled `isaac_ros_foundationpose` wrapper. Phase 2 swaps to
the raw upstream package from NVlabs/FoundationPose because that
wrapper isn't shipped on the vast.ai Isaac Sim image we deploy to.

## Pin

| Field | Value |
|---|---|
| Upstream | <https://github.com/NVlabs/FoundationPose> |
| Pinned commit | `4517f47b5e7e4a7e0d3b9e5d8f8c9e7b8a9d8c5e` *(placeholder — verify against the upstream HEAD before running Task 20 and update this file + `wdt_modal/build_foundationpose_wheels.py`)* |
| Pinned date | 2026-05-15 |
| Model weights | `2024-03-08-foundationpose-checkpoints.tar.gz` (~2 GB, NVIDIA-hosted) |

## Runtime stack on vast.ai

- NVIDIA driver: 570.211 (Romania A5000 instance)
- CUDA toolkit: 12.4 (matches driver)
- PyTorch: 2.1.0+cu124
- Python: 3.10 (Isaac Sim's bundled interpreter)

## Distribution strategy

CUDA extensions (`nvdiffrast`, `mycuda`) are notoriously fragile to
build inside a renderer container. Phase 2 compiles them **once on
Modal** (`nvidia/cuda:12.4.0-devel-ubuntu22.04` base, matches vast.ai
driver) and bundles the resulting wheels into a tarball on a Modal
Volume named `foundationpose-models`. The vast.ai instance pulls the
tarball + model weights via `modal volume get`, extracts under
`/opt/foundationpose/`, and `pip install`s the wheels into Isaac
Sim's `python.sh` environment.

| Step | Where | How | Plan task |
|---|---|---|---|
| Build CUDA wheels | Modal (L4) | `modal run wdt_modal/build_foundationpose_wheels.py` | Task 20 |
| Stage model weights | Modal (CPU) | `modal run wdt_modal/build_foundationpose_wheels.py::stage_weights` | Task 20 |
| Install on vast.ai | vast.ai instance | `bash wdt_vast/install_foundationpose.sh` | Task 21 |
| Wire into pipeline | Isaac Sim python | `manipulation.pose_estimation.PoseEstimator` | Task 22 |

## Prerequisites for the install script

The vast.ai instance must have the `modal` CLI installed and
authenticated against the same Modal account that holds the
`foundationpose-models` volume:

```bash
pip install --user modal
modal token new  # interactive — paste token from modal.com/settings/tokens
```

This is a one-time per-instance bootstrap. Pattern-3 stop/resume
preserves the auth token because it lives in `~/.modal/`.

## Fallback strategy

If the Modal CUDA build fails (driver/toolkit mismatch) or the wheels
don't load on vast.ai (PyTorch ABI mismatch), fall back to building
FoundationPose directly on vast.ai with `setup.py build_ext` — slower
(~25 min) but uses the exact runtime stack. Document the slow path
in `wdt_vast/install_foundationpose.sh` as a commented-out section.
