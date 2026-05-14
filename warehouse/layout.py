"""Pydantic models + loader for warehouse layout YAML configs."""

from __future__ import annotations

from pathlib import Path

import yaml
from pydantic import BaseModel, Field, PositiveFloat, PositiveInt


class WarehouseDims(BaseModel):
    width_m: PositiveFloat
    depth_m: PositiveFloat


class AMRSpawn(BaseModel):
    grid: tuple[PositiveInt, PositiveInt]
    origin_xy: tuple[float, float]
    spacing_m: PositiveFloat


class AMRConfig(BaseModel):
    count: PositiveInt
    spawn: AMRSpawn


class PickCell(BaseModel):
    position_xy: tuple[float, float]
    yaw_deg: float = 0.0


class Shelves(BaseModel):
    rows: PositiveInt
    cols: PositiveInt
    spacing_xy: tuple[PositiveFloat, PositiveFloat]
    origin_xy: tuple[float, float]


class LayoutConfig(BaseModel):
    name: str = Field(min_length=1)
    warehouse: WarehouseDims
    amrs: AMRConfig
    pick_cell: PickCell
    shelves: Shelves


def load_layout(path: str | Path) -> LayoutConfig:
    with open(path) as fh:
        raw = yaml.safe_load(fh)
    return LayoutConfig.model_validate(raw)
