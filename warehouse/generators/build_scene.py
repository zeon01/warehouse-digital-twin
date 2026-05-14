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

    stage.GetRootLayer().Save()
    return str(out_usd)


def build_from_yaml(layout_path: str | Path, out_usd: str | Path) -> str:
    """Convenience: load layout YAML and build the scene in one call."""
    return build_scene(load_layout(layout_path), out_usd)
