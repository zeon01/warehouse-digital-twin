"""Test that build_scene emits a PGM + YAML alongside the USD."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path


def test_build_scene_small_emits_map(tmp_path: Path):
    out_usd = tmp_path / "small.usd"
    out_map_dir = tmp_path / "maps"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "warehouse.generators.build_scene",
            "small",
            "--out-usd",
            str(out_usd),
            "--out-map-dir",
            str(out_map_dir),
            "--skip-usd",  # local Mac doesn't have pxr; skip USD authoring
        ],
        check=True,
    )
    assert (out_map_dir / "small.pgm").exists()
    assert (out_map_dir / "small.yaml").exists()
    # PGM header check
    content = (out_map_dir / "small.pgm").read_bytes()
    assert content.startswith(b"P5\n"), "expected P5 PGM header"


def test_layout_to_obstacle_boxes_small():
    from warehouse.layout import load_layout

    layout = load_layout(
        Path(__file__).resolve().parents[2] / "warehouse" / "layouts" / "small.yaml"
    )
    boxes = layout.to_obstacle_boxes()
    # 4 walls + 12 shelves (4 rows × 3 cols) + 1 pick cell = 17 boxes
    assert len(boxes) == 17
    # Every box has the four keys
    for b in boxes:
        assert {"x_min", "x_max", "y_min", "y_max"} <= b.keys()
        assert b["x_min"] < b["x_max"]
        assert b["y_min"] < b["y_max"]
