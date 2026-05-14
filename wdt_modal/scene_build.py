"""Build a warehouse USD scene on Modal and write to the persistent volume.

USD authoring is CPU-only (no rendering, no Vulkan) — runs fine on Modal
despite the Vulkan blocker for the actual render pipeline.

The function calls warehouse.generators.build_scene.build_from_yaml directly
in Modal's Python (3.11). pxr comes from the usd-core PyPI package which is
baked into the image — no /isaac-sim/python.sh subprocess needed.
"""

from __future__ import annotations

import os

from wdt_modal.app import app
from wdt_modal.volumes import SCENES_PATH, VOLUME_MOUNT, isaac_volume


@app.function(
    cpu=2.0,
    timeout=600,
    volumes={VOLUME_MOUNT: isaac_volume},
)
def build_scene_job(layout_name: str = "small") -> str:
    """Build USD scene from warehouse/layouts/<layout_name>.yaml, write to /vol/scenes/."""
    from warehouse.generators.build_scene import build_from_yaml

    os.makedirs(SCENES_PATH, exist_ok=True)
    layout_yaml = f"/root/warehouse/layouts/{layout_name}.yaml"
    out_usd = f"{SCENES_PATH}/{layout_name}.usd"

    path = build_from_yaml(layout_yaml, out_usd)
    isaac_volume.commit()
    return path
