"""Pydantic models + loader for warehouse layout YAML configs."""

from __future__ import annotations

from pathlib import Path

import yaml
from pydantic import BaseModel, Field, PositiveFloat, PositiveInt

# Wall thickness used by warehouse.generators.build_scene when extruding
# the four perimeter walls of the USD scene. Mirrored here so
# LayoutConfig.to_obstacle_boxes() can include the walls in the 2D
# occupancy grid handed to Nav2. Keep in sync with build_scene._main if
# the USD wall thickness ever changes.
WALL_THICKNESS_M = 0.2

# Shelf XY footprint (matches the Gf.Vec3d(1.0, 0.6, 2.0) scale in
# warehouse.generators.build_scene._add_shelves). Same sync caveat as
# WALL_THICKNESS_M.
SHELF_XY_M = (1.0, 0.6)

# Pick-cell base XY footprint (matches Gf.Vec3d(1.5, 1.5, 1.0) in
# warehouse.generators.build_scene._add_pick_cell).
PICK_CELL_XY_M = (1.5, 1.5)


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

    def to_obstacle_boxes(self) -> list[dict]:
        """Return all static obstacles as axis-aligned boxes for Nav2.

        Each box is a dict with x_min/x_max/y_min/y_max in meters, world
        frame. Includes the four perimeter walls, every shelf, and the
        pick-cell base. AMR spawn markers are excluded — they're visual
        debug aids, not collision geometry.
        """
        w = self.warehouse.width_m
        d = self.warehouse.depth_m
        t = WALL_THICKNESS_M
        boxes: list[dict] = [
            # South wall (y=0)
            {"x_min": 0.0, "x_max": w, "y_min": -t / 2, "y_max": t / 2},
            # North wall (y=d)
            {"x_min": 0.0, "x_max": w, "y_min": d - t / 2, "y_max": d + t / 2},
            # West wall (x=0)
            {"x_min": -t / 2, "x_max": t / 2, "y_min": 0.0, "y_max": d},
            # East wall (x=w)
            {"x_min": w - t / 2, "x_max": w + t / 2, "y_min": 0.0, "y_max": d},
        ]
        sx, sy = SHELF_XY_M
        ox, oy = self.shelves.origin_xy
        gx, gy = self.shelves.spacing_xy
        for row in range(self.shelves.rows):
            for col in range(self.shelves.cols):
                cx = ox + col * gx
                cy = oy + row * gy
                boxes.append(
                    {
                        "x_min": cx - sx / 2,
                        "x_max": cx + sx / 2,
                        "y_min": cy - sy / 2,
                        "y_max": cy + sy / 2,
                    }
                )
        px, py = self.pick_cell.position_xy
        pcx, pcy = PICK_CELL_XY_M
        boxes.append(
            {
                "x_min": px - pcx / 2,
                "x_max": px + pcx / 2,
                "y_min": py - pcy / 2,
                "y_max": py + pcy / 2,
            }
        )
        return boxes


def load_layout(path: str | Path) -> LayoutConfig:
    with open(path) as fh:
        raw = yaml.safe_load(fh)
    return LayoutConfig.model_validate(raw)
