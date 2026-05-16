"""Spawn-pose consistency tests.

The multi-AMR launch + sim + smoke each carry a hardcoded copy of the
fleet spawn poses (DEFAULT_SPAWN_POSES / _SPAWN_POSES / SPAWN_POSES).
This test asserts they all agree with what ``warehouse/layouts/small.yaml``
actually produces via ``AMRConfig.spawn_poses()``, so drift is caught
before it bites a smoke run.
"""

from __future__ import annotations

import re
from pathlib import Path

from warehouse.layout import load_layout

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
_LAYOUT_PATH = _PROJECT_ROOT / "warehouse" / "layouts" / "small.yaml"

# Match a Python list-of-tuples assignment: anchor on `<NAME>: list...= [`
# then capture (x, y) pairs until the matching `]`. Avoids false matches
# on docstring mentions of "SPAWN_POSES".
_ASSIGN_RE = re.compile(r"^_?[A-Z_]*SPAWN_POSES[^=]*=\s*\[(.*?)\]", re.DOTALL | re.MULTILINE)
_PAIR_RE = re.compile(r"\(\s*([0-9.]+)\s*,\s*([0-9.]+)\s*\)")


def _extract_poses(path: Path) -> list[tuple[float, float]]:
    text = path.read_text()
    m = _ASSIGN_RE.search(text)
    assert m, f"{path.name}: couldn't find SPAWN_POSES list-assignment"
    body = m.group(1)
    return [(float(x), float(y)) for x, y in _PAIR_RE.findall(body)]


def test_amr_config_spawn_poses_small_layout():
    cfg = load_layout(_LAYOUT_PATH)
    poses = cfg.amrs.spawn_poses()
    expected = [
        (2.0, 2.0),
        (3.5, 2.0),
        (5.0, 2.0),
        (2.0, 3.5),
        (3.5, 3.5),
        (5.0, 3.5),
    ]
    assert poses == expected


def test_sim_fleet_poses_match_layout():
    expected = load_layout(_LAYOUT_PATH).amrs.spawn_poses()
    found = _extract_poses(_PROJECT_ROOT / "wdt_vast" / "sim_fleet.py")
    assert found == expected, f"sim_fleet.py drift — got {found}"


def test_smoke_poses_match_layout():
    expected = load_layout(_LAYOUT_PATH).amrs.spawn_poses()
    found = _extract_poses(_PROJECT_ROOT / "wdt_vast" / "pure_pursuit_multi_smoke.py")
    assert found == expected, f"pure_pursuit_multi_smoke.py drift — got {found}"


def test_launch_poses_match_layout():
    expected = load_layout(_LAYOUT_PATH).amrs.spawn_poses()
    found = _extract_poses(
        _PROJECT_ROOT / "ros2_ws" / "src" / "wdt_pure_pursuit" / "launch" / "multi_amr.launch.py"
    )
    assert found == expected, f"multi_amr.launch.py drift — got {found}"
