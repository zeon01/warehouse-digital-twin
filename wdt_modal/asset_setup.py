"""One-time asset pre-pull into the persistent volume."""

from __future__ import annotations

import os
import subprocess

from wdt_modal.app import app
from wdt_modal.volumes import (
    ASSETS_PATH,
    MODELS_PATH,
    RUNS_PATH,
    SCENES_PATH,
    VOLUME_MOUNT,
    isaac_volume,
)

ISAAC_ASSET_SOURCE = (
    "https://omniverse-content-production.s3.us-west-2.amazonaws.com/" "Assets/Isaac/5.0/Isaac"
)


@app.function(
    cpu=2.0,
    timeout=3600,
    volumes={VOLUME_MOUNT: isaac_volume},
)
def prepare_volume() -> dict[str, list[str]]:
    """Create directory layout and pre-pull a minimal Nova Carter + Franka asset set."""
    for path in (ASSETS_PATH, SCENES_PATH, MODELS_PATH, RUNS_PATH):
        os.makedirs(path, exist_ok=True)

    # NOTE: Paths updated from plan spec — in Isaac Sim 5.0 the S3 layout moved:
    #   Robots/NovaCarter/       -> Robots/NVIDIA/NovaCarter/
    #   Robots/Franka/           -> Robots/FrankaRobotics/FrankaPanda/
    #   Props/Shelves/shelf_basic.usd -> no shelves dir; substituted Sektion_Cabinet
    targets = [
        (
            "Robots/NVIDIA/NovaCarter/nova_carter.usd",
            f"{ASSETS_PATH}/Robots/NVIDIA/NovaCarter/nova_carter.usd",
        ),
        (
            "Robots/FrankaRobotics/FrankaPanda/franka.usd",
            f"{ASSETS_PATH}/Robots/FrankaRobotics/FrankaPanda/franka.usd",
        ),
        (
            "Props/Sektion_Cabinet/sektion_cabinet_instanceable.usd",
            f"{ASSETS_PATH}/Props/Sektion_Cabinet/sektion_cabinet_instanceable.usd",
        ),
    ]

    fetched: list[str] = []
    for source_rel, dest in targets:
        if os.path.exists(dest):
            fetched.append(f"cached:{dest}")
            continue
        os.makedirs(os.path.dirname(dest), exist_ok=True)
        url = f"{ISAAC_ASSET_SOURCE}/{source_rel}"
        subprocess.run(["curl", "-fL", url, "-o", dest], check=True)
        fetched.append(f"fetched:{dest}")

    isaac_volume.commit()
    return {"results": fetched}
