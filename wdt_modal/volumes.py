"""Persistent Modal volumes for assets, scenes, models, and run outputs."""

from __future__ import annotations

import modal

isaac_volume = modal.Volume.from_name("isaac-volume", create_if_missing=True)

VOLUME_MOUNT = "/vol"
ASSETS_PATH = f"{VOLUME_MOUNT}/assets"
SCENES_PATH = f"{VOLUME_MOUNT}/scenes"
MODELS_PATH = f"{VOLUME_MOUNT}/models"
RUNS_PATH = f"{VOLUME_MOUNT}/runs"
