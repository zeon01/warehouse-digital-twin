import pytest
from pydantic import ValidationError

from warehouse.layout import LayoutConfig, load_layout


def test_load_minimal_layout(tmp_path):
    yml = tmp_path / "layout.yaml"
    yml.write_text("""
name: small
warehouse:
  width_m: 20
  depth_m: 30
amrs:
  count: 6
  spawn:
    grid: [3, 2]
    origin_xy: [2.0, 2.0]
    spacing_m: 1.5
pick_cell:
  position_xy: [16.0, 15.0]
  yaw_deg: 0
shelves:
  rows: 4
  cols: 3
  spacing_xy: [3.0, 4.0]
  origin_xy: [4.0, 8.0]
""")
    cfg = load_layout(yml)
    assert isinstance(cfg, LayoutConfig)
    assert cfg.name == "small"
    assert cfg.amrs.count == 6
    assert cfg.shelves.rows * cfg.shelves.cols == 12


def test_load_layout_rejects_negative_count(tmp_path):
    yml = tmp_path / "bad.yaml"
    yml.write_text("""
name: bad
warehouse: {width_m: 10, depth_m: 10}
amrs: {count: -1, spawn: {grid: [1,1], origin_xy: [0,0], spacing_m: 1}}
pick_cell: {position_xy: [5,5], yaw_deg: 0}
shelves: {rows: 0, cols: 0, spacing_xy: [1,1], origin_xy: [0,0]}
""")
    with pytest.raises(ValidationError):
        load_layout(yml)
