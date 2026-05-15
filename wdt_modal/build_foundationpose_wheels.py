"""Build FoundationPose CUDA wheels on Modal, store in a volume.

The runtime target is vast.ai (driver 570 / CUDA 12.4). We build inside
a Modal container that matches: nvidia/cuda:12.4.0-devel-ubuntu22.04
with PyTorch 2.1.0+cu124 and Python 3.10.

Output (on the `foundationpose-models` Modal volume):
    foundationpose-wheels-<commit-short>.tar.gz
        tarball of built wheels for nvdiffrast + mycuda CUDA extensions
    checkpoints/
        unpacked model weights from the NVIDIA-hosted checkpoint bundle

Run:
    # Build wheels (~15-25 min, watch for tzdata/PPA gotchas)
    modal run wdt_modal/build_foundationpose_wheels.py

    # Stage weights (~5-10 min, one-time)
    modal run wdt_modal/build_foundationpose_wheels.py::stage_weights

See manipulation/FOUNDATIONPOSE.md for the full integration story.
"""

from __future__ import annotations

import modal

# Pinned commit — verify against upstream HEAD before running.
# See manipulation/FOUNDATIONPOSE.md.
FP_COMMIT = "4517f47b5e7e4a7e0d3b9e5d8f8c9e7b8a9d8c5e"
FP_COMMIT_SHORT = FP_COMMIT[:8]

image = (
    modal.Image.from_registry(
        "nvidia/cuda:12.4.0-devel-ubuntu22.04",
        add_python="3.10",
    )
    .env({"DEBIAN_FRONTEND": "noninteractive", "TZ": "Etc/UTC"})
    .apt_install(
        "git",
        "build-essential",
        "ninja-build",
        "cmake",
        "libgl1",
        "libglib2.0-0",
    )
    .pip_install(
        "torch==2.1.0",
        "torchvision==0.16.0",
        "ninja",
        "setuptools",
        "wheel",
        index_url="https://download.pytorch.org/whl/cu124",
    )
    .run_commands(
        "git clone https://github.com/NVlabs/FoundationPose /workspace/fp && "
        f"cd /workspace/fp && git checkout {FP_COMMIT}",
    )
)

app = modal.App("wdt-foundationpose-wheel-builder")
vol = modal.Volume.from_name("foundationpose-models", create_if_missing=True)


@app.function(image=image, gpu="L4", volumes={"/weights": vol}, timeout=3600)
def build_wheels():
    """Compile nvdiffrast + mycuda CUDA extensions into pip wheels."""
    import shutil  # noqa: F401 — kept for parity with the install script's tarball
    import subprocess
    from pathlib import Path

    fp = Path("/workspace/fp")
    out = Path("/weights/wheels")
    out.mkdir(parents=True, exist_ok=True)

    # nvdiffrast bundled inside FoundationPose
    subprocess.run(
        ["pip", "wheel", "-w", str(out), "./bundled/nvdiffrast"],
        cwd=fp,
        check=True,
    )
    # mycuda extension (custom CUDA ops for ICP refinement)
    subprocess.run(
        ["pip", "wheel", "-w", str(out), "./mycpp/mycuda"],
        cwd=fp,
        check=True,
    )

    tarball = Path(f"/weights/foundationpose-wheels-{FP_COMMIT_SHORT}.tar.gz")
    subprocess.run(
        ["tar", "-czf", str(tarball), "-C", str(out), "."],
        check=True,
    )
    print(f"wheels built and tarred to {tarball}")
    print(f"contents: {list(out.iterdir())}")


@app.function(image=image, volumes={"/weights": vol}, timeout=3600)
def stage_weights():
    """Download + extract the FoundationPose model checkpoints.

    Skips if /weights/checkpoints/model_best.pth already exists, so
    re-running this function is cheap.
    """
    import subprocess
    from pathlib import Path

    out = Path("/weights/checkpoints")
    if (out / "model_best.pth").exists():
        print("weights already present, skipping")
        return
    out.mkdir(parents=True, exist_ok=True)
    # TODO: replace with the official NVIDIA-hosted FoundationPose
    # checkpoints URL from the upstream README. The current URL is a
    # placeholder — the upstream uses Google Drive which doesn't curl
    # cleanly. Options for the real fix: (a) a Hugging Face mirror,
    # (b) a Modal-hosted gdrive-rsync, (c) manual upload via
    # `modal volume put`.
    url = "https://example.invalid/2024-03-08-foundationpose-checkpoints.tar.gz"
    subprocess.run(["curl", "-L", url, "-o", "/weights/cp.tar.gz"], check=True)
    subprocess.run(["tar", "-xzf", "/weights/cp.tar.gz", "-C", str(out)], check=True)
    print(f"checkpoints extracted to {out}")


@app.local_entrypoint()
def main():
    """Default `modal run` entrypoint — builds the wheels."""
    build_wheels.remote()
