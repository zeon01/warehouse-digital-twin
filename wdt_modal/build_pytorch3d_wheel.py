"""Build a pytorch3d wheel on Modal targeting vast.ai's runtime stack.

vast.ai runs Isaac Sim 5.0 with Python 3.11 and (after Phase 2 install)
PyTorch 2.4.0+cu124. pytorch3d isn't on PyPI for this combo, so we
build from source inside a CUDA-dev container and stash the .whl on
the foundationpose-models Modal volume. The vast.ai installer scp's
the wheel down and `pip install`s it.

TORCH_CUDA_ARCH_LIST=8.6 targets the RTX A5000 (compute capability 8.6)
only — about 5× faster to build than the default multi-arch list.

Run:
    modal run wdt_modal/build_pytorch3d_wheel.py
    # ~15-25 min depending on Modal queue + build parallelism.
"""

from __future__ import annotations

import modal

PT3D_REF = "v0.7.9"  # stable release tag, PyTorch 2.4 compatible

image = (
    modal.Image.from_registry(
        "nvidia/cuda:12.4.0-devel-ubuntu22.04",
        add_python="3.11",
    )
    .env(
        {
            "DEBIAN_FRONTEND": "noninteractive",
            "TZ": "Etc/UTC",
            "TORCH_CUDA_ARCH_LIST": "8.6",  # RTX A5000 only
            "FORCE_CUDA": "1",
        }
    )
    .apt_install(
        "git",
        "build-essential",
        "ninja-build",
        "cmake",
        # pytorch3d's setup.py shells out to `which clang++` and fails
        # if it's missing, even though gcc is the default compiler.
        "clang",
    )
    .pip_install(
        "torch==2.4.0",
        "torchvision==0.19.0",
        index_url="https://download.pytorch.org/whl/cu124",
    )
    .pip_install("fvcore", "iopath", "ninja", "setuptools", "wheel")
)

app = modal.App("wdt-pytorch3d-wheel")
vol = modal.Volume.from_name("foundationpose-models", create_if_missing=True)


@app.function(image=image, gpu="L4", volumes={"/weights": vol}, timeout=3600)
def build_wheel() -> None:
    """Clone pytorch3d at PT3D_REF and pip-wheel into the volume."""
    import shutil
    import subprocess
    from pathlib import Path

    out_dir = Path("/weights/wheels")
    out_dir.mkdir(parents=True, exist_ok=True)

    # Skip if a wheel already exists.
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

    # Pip wheel with --no-build-isolation so the build sees the installed
    # torch (pytorch3d's setup.py imports torch to detect CUDA arch).
    subprocess.run(
        [
            "pip",
            "wheel",
            "--no-build-isolation",
            "-w",
            str(out_dir),
            ".",
        ],
        cwd=src,
        check=True,
    )

    wheels = list(out_dir.glob("pytorch3d-*.whl"))
    print(f"==> built {len(wheels)} wheel(s) in /weights/wheels:")
    for w in wheels:
        print(f"    {w.name}  ({w.stat().st_size // (1024 * 1024)} MB)")


@app.local_entrypoint()
def main() -> None:
    build_wheel.remote()
