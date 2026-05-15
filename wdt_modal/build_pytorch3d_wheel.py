"""Build pytorch3d wheels on Modal — both cp311 (Isaac Sim) and cp310 (system).

Phase 2 ended up needing FoundationPose in system Python 3.10 (where
ROS2 Humble's rclpy lives) rather than Isaac Sim's Python 3.11 — the
pick_cell_orchestrator needs both rclpy and FoundationPose in one
process, and rclpy on Humble is py3.10-only.

We keep both Python targets so a Phase 3 with mixed orchestration (or a
ROS2 Jazzy upgrade) can use the cp311 wheel without rebuilding.

TORCH_CUDA_ARCH_LIST=8.6 targets RTX A5000 only; ~5x faster to build
than the default multi-arch list.

Run:
    modal run wdt_modal/build_pytorch3d_wheel.py
    # ~25-35 min for both builds (~12 min each + image cache hits).
"""

from __future__ import annotations

import modal

PT3D_REF = "v0.7.9"  # stable release tag, PyTorch 2.4 compatible


def _image(py: str) -> modal.Image:
    return (
        modal.Image.from_registry(
            "nvidia/cuda:12.4.0-devel-ubuntu22.04",
            add_python=py,
        )
        .env(
            {
                "DEBIAN_FRONTEND": "noninteractive",
                "TZ": "Etc/UTC",
                "TORCH_CUDA_ARCH_LIST": "8.6",
                "FORCE_CUDA": "1",
            }
        )
        .apt_install(
            "git",
            "build-essential",
            "ninja-build",
            "cmake",
            "clang",  # pytorch3d's setup.py shells out to `which clang++`
        )
        .pip_install(
            "torch==2.4.0",
            "torchvision==0.19.0",
            index_url="https://download.pytorch.org/whl/cu124",
        )
        .pip_install("fvcore", "iopath", "ninja", "setuptools", "wheel")
    )


image_py311 = _image("3.11")
image_py310 = _image("3.10")

app = modal.App("wdt-pytorch3d-wheel")
vol = modal.Volume.from_name("foundationpose-models", create_if_missing=True)


def _build(out_subdir: str) -> None:
    """Shared body: clone pytorch3d at PT3D_REF and pip-wheel into /weights/<out_subdir>."""
    import shutil
    import subprocess
    from pathlib import Path

    out_dir = Path("/weights") / out_subdir
    out_dir.mkdir(parents=True, exist_ok=True)

    existing = list(out_dir.glob("pytorch3d-*.whl"))
    if existing:
        print(f"[skip] wheel already present: {existing[0].name}")
        return

    src = Path("/tmp/pytorch3d")
    if src.exists():
        shutil.rmtree(src)
    subprocess.run(
        [
            "git",
            "clone",
            "--depth",
            "1",
            "--branch",
            PT3D_REF,
            "https://github.com/facebookresearch/pytorch3d.git",
            str(src),
        ],
        check=True,
    )
    subprocess.run(
        ["pip", "wheel", "--no-build-isolation", "-w", str(out_dir), "."],
        cwd=src,
        check=True,
    )
    wheels = list(out_dir.glob("pytorch3d-*.whl"))
    print(f"==> built {len(wheels)} wheel(s) in /weights/{out_subdir}:")
    for w in wheels:
        print(f"    {w.name}  ({w.stat().st_size // (1024 * 1024)} MB)")


@app.function(image=image_py311, gpu="L4", volumes={"/weights": vol}, timeout=3600)
def build_wheel_py311() -> None:
    _build("wheels")  # legacy path — preserves the cp311 wheel scp'd earlier


@app.function(image=image_py310, gpu="L4", volumes={"/weights": vol}, timeout=3600)
def build_wheel_py310() -> None:
    _build("wheels-py310")


@app.local_entrypoint()
def main() -> None:
    # cp311 already built and shipped; the default invocation just builds
    # cp310 for the system python on vast.ai. Use ::build_wheel_py311
    # explicitly to (re)build the Isaac-Sim-python wheel.
    build_wheel_py310.remote()
