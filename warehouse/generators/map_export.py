"""Rasterize procedural warehouse obstacles into a 2D occupancy grid.

The Nav2 ``map_server`` consumes a (PGM, YAML) pair where the PGM is a
grayscale image (0 = free, 100 = occupied, 255 = unknown) and the YAML
points to it plus declares resolution + origin in world coordinates.
We produce both from the procedural layout so Nav2's planner has a
consistent view of the warehouse.

Grid layout: row 0 is the bottom of the world (y=0); row H-1 is the top.
Column 0 is x=0; column W-1 is x=world_w_m. This matches the YAML
convention ``origin: [0.0, 0.0, 0.0]`` (world origin at bottom-left of
the grid).
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml

OCCUPIED = 100
FREE = 0


def rasterize_obstacles(
    world_w_m: float,
    world_h_m: float,
    resolution_m_per_px: float,
    obstacles: list[dict],
) -> np.ndarray:
    """Return a (H, W) uint8 grid where W = world_w_m/res, H = world_h_m/res.

    Each obstacle is a dict with keys x_min, x_max, y_min, y_max (meters).
    Obstacles clipping the world boundary are silently truncated.
    """
    res = resolution_m_per_px
    w = int(round(world_w_m / res))
    h = int(round(world_h_m / res))
    grid = np.zeros((h, w), dtype=np.uint8)

    for obs in obstacles:
        x0 = max(0, int(np.floor(obs["x_min"] / res)))
        x1 = min(w, int(np.ceil(obs["x_max"] / res)))
        y0 = max(0, int(np.floor(obs["y_min"] / res)))
        y1 = min(h, int(np.ceil(obs["y_max"] / res)))
        grid[y0:y1, x0:x1] = OCCUPIED

    return grid


def write_pgm(grid: np.ndarray, path: Path) -> None:
    """Write a Nav2-compatible PGM. 0 = free, 100 = occupied, 255 = unknown."""
    h, w = grid.shape
    # PGM rows go top-to-bottom in image space; flip so row 0 of our
    # bottom-up grid lands at the bottom of the PGM.
    img = np.flipud(grid)
    header = f"P5\n{w} {h}\n255\n".encode("ascii")
    path.write_bytes(header + img.tobytes())


def write_map_yaml(
    pgm_filename: str,
    resolution_m_per_px: float,
    origin_xy_yaw: tuple[float, float, float],
    path: Path,
) -> None:
    """Write the Nav2 map YAML next to the PGM.

    The PGM filename is stored *relative* to the YAML so Nav2's loader
    works regardless of absolute path on the runtime host.
    """
    data = {
        "image": pgm_filename,
        "resolution": resolution_m_per_px,
        "origin": list(origin_xy_yaw),
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
    }
    path.write_text(yaml.safe_dump(data, sort_keys=False))
