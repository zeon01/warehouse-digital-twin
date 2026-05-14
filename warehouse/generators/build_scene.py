"""Programmatic USD scene generator from a LayoutConfig.

Only runs in an environment that has `pxr` available — Isaac Sim's bundled
Python (via /isaac-sim/python.sh) or a venv with usd-core installed. Local
unit tests don't run this; the smoke test lives in the Modal job (Task 15)
where pxr is real.

USD authoring does NOT need rendering / Vulkan — only the pxr bindings —
so this can run on Modal even though full Isaac Sim render is broken there.
"""

from __future__ import annotations

from pathlib import Path

from warehouse.layout import LayoutConfig, load_layout


def build_scene(layout: LayoutConfig, out_usd: str | Path) -> str:
    """Compose a USD stage and write it to out_usd. Returns the written path."""
    from pxr import Gf, Usd, UsdGeom, UsdLux

    stage = Usd.Stage.CreateNew(str(out_usd))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    UsdGeom.Xform.Define(stage, "/World")

    # Floor — a thin cube centered on the warehouse footprint, sunk slightly into z<0
    floor = UsdGeom.Cube.Define(stage, "/World/Floor")
    floor.CreateSizeAttr(1.0)
    UsdGeom.Xformable(floor).AddTranslateOp().Set(
        Gf.Vec3d(layout.warehouse.width_m / 2, layout.warehouse.depth_m / 2, -0.05)
    )
    UsdGeom.Xformable(floor).AddScaleOp().Set(
        Gf.Vec3d(layout.warehouse.width_m, layout.warehouse.depth_m, 0.1)
    )

    # Walls — four thin cubes along the perimeter
    wall_h = 3.0
    wall_t = 0.2
    walls = [
        (
            "North",
            layout.warehouse.width_m / 2,
            layout.warehouse.depth_m,
            layout.warehouse.width_m,
            wall_t,
        ),
        ("South", layout.warehouse.width_m / 2, 0.0, layout.warehouse.width_m, wall_t),
        (
            "East",
            layout.warehouse.width_m,
            layout.warehouse.depth_m / 2,
            wall_t,
            layout.warehouse.depth_m,
        ),
        ("West", 0.0, layout.warehouse.depth_m / 2, wall_t, layout.warehouse.depth_m),
    ]
    for name, cx, cy, sx, sy in walls:
        prim = UsdGeom.Cube.Define(stage, f"/World/Walls/{name}")
        prim.CreateSizeAttr(1.0)
        UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(cx, cy, wall_h / 2))
        UsdGeom.Xformable(prim).AddScaleOp().Set(Gf.Vec3d(sx, sy, wall_h))

    # Distant light — sun-like, no specific direction set here
    light = UsdLux.DistantLight.Define(stage, "/World/SunLight")
    light.CreateIntensityAttr(3000.0)

    _add_shelves(stage, layout)
    _add_pick_cell(stage, layout)
    _add_amr_spawn_markers(stage, layout)

    stage.GetRootLayer().Save()
    return str(out_usd)


def _add_shelves(stage, layout: LayoutConfig) -> None:
    """Grid of shelf cubes per layout.shelves config."""
    from pxr import Gf, UsdGeom

    ox, oy = layout.shelves.origin_xy
    sx, sy = layout.shelves.spacing_xy
    for row in range(layout.shelves.rows):
        for col in range(layout.shelves.cols):
            cx = ox + col * sx
            cy = oy + row * sy
            prim = UsdGeom.Cube.Define(stage, f"/World/Shelves/Shelf_{row}_{col}")
            prim.CreateSizeAttr(1.0)
            UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(cx, cy, 1.0))
            UsdGeom.Xformable(prim).AddScaleOp().Set(Gf.Vec3d(1.0, 0.6, 2.0))


def _add_pick_cell(stage, layout: LayoutConfig) -> None:
    """Block placeholder for the manipulator's pick cell base."""
    from pxr import Gf, UsdGeom

    px, py = layout.pick_cell.position_xy
    base = UsdGeom.Cube.Define(stage, "/World/PickCell/Base")
    base.CreateSizeAttr(1.0)
    UsdGeom.Xformable(base).AddTranslateOp().Set(Gf.Vec3d(px, py, 0.5))
    UsdGeom.Xformable(base).AddScaleOp().Set(Gf.Vec3d(1.5, 1.5, 1.0))


def _add_amr_spawn_markers(stage, layout: LayoutConfig) -> list[tuple[float, float]]:
    """Small flat markers at each AMR spawn pose; returns the pose list."""
    from pxr import Gf, UsdGeom

    gx, gy = layout.amrs.spawn.grid
    ox, oy = layout.amrs.spawn.origin_xy
    spacing = layout.amrs.spawn.spacing_m
    poses: list[tuple[float, float]] = []
    idx = 0
    for r in range(gy):
        for c in range(gx):
            if idx >= layout.amrs.count:
                break
            x = ox + c * spacing
            y = oy + r * spacing
            poses.append((x, y))
            m = UsdGeom.Cube.Define(stage, f"/World/SpawnMarkers/AMR_{idx}")
            m.CreateSizeAttr(1.0)
            UsdGeom.Xformable(m).AddTranslateOp().Set(Gf.Vec3d(x, y, 0.05))
            UsdGeom.Xformable(m).AddScaleOp().Set(Gf.Vec3d(0.3, 0.3, 0.1))
            idx += 1
    return poses


def build_from_yaml(layout_path: str | Path, out_usd: str | Path) -> str:
    """Convenience: load layout YAML and build the scene in one call."""
    return build_scene(load_layout(layout_path), out_usd)
