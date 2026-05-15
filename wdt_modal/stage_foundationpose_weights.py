"""Cache FoundationPose model weights on a Modal volume.

The upstream weights live in two public Google Drive folders:

    refiner: 2023-10-28-18-33-37/  (model_best.pth + config.yml)
    scorer:  2024-01-11-20-02-45/  (model_best.pth + config.yml)

vast.ai instances can't easily auth against Google Drive, so we mirror
the folders to a Modal volume named ``foundationpose-models`` once.
``wdt_vast/install_foundationpose.sh`` then pulls them via
``modal volume get``.

Run:
    modal run wdt_modal/stage_foundationpose_weights.py
    # ~5-10 minutes for ~2 GB total, depending on Drive throttling.

If gdown fails (rate-limited, ACL changes, ToS update), fall back to
manual upload:
    # On local Mac, after manually downloading the two folders:
    modal volume put foundationpose-models 2023-10-28-18-33-37 weights/2023-10-28-18-33-37
    modal volume put foundationpose-models 2024-01-11-20-02-45 weights/2024-01-11-20-02-45

The Modal Volume layout we maintain is:
    /weights/
        2023-10-28-18-33-37/
            model_best.pth
            config.yml
        2024-01-11-20-02-45/
            model_best.pth
            config.yml
"""

from __future__ import annotations

import modal

# Public Drive folder linked from FoundationPose's README — contains both
# the refiner and scorer subfolders.
WEIGHTS_DRIVE_FOLDER = "1DFezOAD0oD1BblsXVxqDsl8fj0qzB82i"
REFINER_RUN_NAME = "2023-10-28-18-33-37"
SCORER_RUN_NAME = "2024-01-11-20-02-45"

image = modal.Image.debian_slim(python_version="3.11").apt_install("curl").pip_install("gdown>=5.1")

app = modal.App("wdt-foundationpose-weights")
vol = modal.Volume.from_name("foundationpose-models", create_if_missing=True)


@app.function(image=image, volumes={"/weights": vol}, timeout=3600)
def stage_weights() -> None:
    """Mirror both weight subfolders from Google Drive to /weights/.

    Skips folders whose ``model_best.pth`` already exists so re-runs are
    cheap. If gdown returns no files, falls back to the manual-upload
    instructions in this module's docstring.
    """
    import subprocess
    from pathlib import Path

    out_root = Path("/weights")
    out_root.mkdir(parents=True, exist_ok=True)

    needs_download = []
    for run_name in (REFINER_RUN_NAME, SCORER_RUN_NAME):
        target = out_root / run_name
        if (target / "model_best.pth").exists():
            print(f"[skip] {target} already has model_best.pth")
        else:
            needs_download.append(run_name)

    if not needs_download:
        print("==> all weights present, nothing to download")
        return

    print(f"[gdown] downloading parent Drive folder for {needs_download}")
    # gdown --folder walks the entire Drive folder; we pull once and
    # cherry-pick the run subfolders we need.
    subprocess.run(
        [
            "gdown",
            "--folder",
            f"https://drive.google.com/drive/folders/{WEIGHTS_DRIVE_FOLDER}",
            "-O",
            "/tmp/fp_drive",
        ],
        check=True,
    )

    for run_name in needs_download:
        src = Path("/tmp/fp_drive") / run_name
        if not src.exists():
            raise RuntimeError(
                f"gdown didn't produce {src}; check Drive folder layout "
                f"or fall back to manual `modal volume put` per the module docstring"
            )
        target = out_root / run_name
        target.mkdir(parents=True, exist_ok=True)
        for f in src.iterdir():
            subprocess.run(["cp", str(f), str(target / f.name)], check=True)
        print(f"[ok] {target} populated")

    print("==> staging complete")


@app.local_entrypoint()
def main() -> None:
    stage_weights.remote()
