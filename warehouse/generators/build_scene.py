"""Programmatic USD scene generator from a LayoutConfig.

Only runs in an environment that has `pxr` available — Isaac Sim's bundled
Python (via /isaac-sim/python.sh) or a venv with usd-core installed. Local
unit tests don't run this; the smoke test lives in the Modal job (Task 15)
where pxr is real.

USD authoring does NOT need rendering / Vulkan — only the pxr bindings —
so this can run on Modal even though full Isaac Sim render is broken there.

Each prim type gets a colored UsdPreviewSurface material so a rendered
overhead view is interpretable: grey floor, white walls, blue shelves,
red pick cell, green AMR spawn markers.
"""

from __future__ import annotations

from pathlib import Path

from warehouse.layout import LayoutConfig, load_layout

# Material colors per prim type — chosen for high contrast in renders.
_COLORS = {
    "floor": (0.4, 0.4, 0.4),
    "wall": (0.85, 0.85, 0.85),
    "shelf": (0.2, 0.4, 0.8),
    "pick_cell": (0.85, 0.2, 0.2),
    "spawn_marker": (0.2, 0.8, 0.3),
}


def _make_material(stage, name: str, color: tuple[float, float, float], roughness: float = 0.6):
    """Define a UsdPreviewSurface material and return it for binding."""
    from pxr import Gf, Sdf, UsdShade

    mat_path = f"/World/Materials/{name}"
    material = UsdShade.Material.Define(stage, mat_path)
    shader = UsdShade.Shader.Define(stage, f"{mat_path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def _bind(prim, material) -> None:
    from pxr import UsdShade

    UsdShade.MaterialBindingAPI.Apply(prim.GetPrim()).Bind(material)


def build_scene(layout: LayoutConfig, out_usd: str | Path) -> str:
    """Compose a USD stage and write it to out_usd. Returns the written path."""
    from pxr import Gf, Usd, UsdGeom, UsdLux

    stage = Usd.Stage.CreateNew(str(out_usd))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    UsdGeom.Xform.Define(stage, "/World")
    UsdGeom.Scope.Define(stage, "/World/Materials")

    mat_floor = _make_material(stage, "Floor", _COLORS["floor"], roughness=0.8)
    mat_wall = _make_material(stage, "Wall", _COLORS["wall"], roughness=0.7)
    mat_shelf = _make_material(stage, "Shelf", _COLORS["shelf"], roughness=0.5)
    mat_pick = _make_material(stage, "PickCell", _COLORS["pick_cell"], roughness=0.4)
    mat_spawn = _make_material(stage, "Spawn", _COLORS["spawn_marker"], roughness=0.6)

    # Floor — a thin cube centered on the warehouse footprint, sunk slightly into z<0
    floor = UsdGeom.Cube.Define(stage, "/World/Floor")
    floor.CreateSizeAttr(1.0)
    UsdGeom.Xformable(floor).AddTranslateOp().Set(
        Gf.Vec3d(layout.warehouse.width_m / 2, layout.warehouse.depth_m / 2, -0.05)
    )
    UsdGeom.Xformable(floor).AddScaleOp().Set(
        Gf.Vec3d(layout.warehouse.width_m, layout.warehouse.depth_m, 0.1)
    )
    _bind(floor, mat_floor)

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
        _bind(prim, mat_wall)

    # Lighting — distant "sun" + dome for ambient fill. The previous render had
    # a single 3000-intensity DistantLight which produced a huge white hot spot;
    # this two-light setup gives even ambient illumination plus directional shadow.
    sun = UsdLux.DistantLight.Define(stage, "/World/SunLight")
    sun.CreateIntensityAttr(800.0)
    UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 0.0, 30.0))

    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.CreateIntensityAttr(500.0)

    _add_shelves(stage, layout, mat_shelf)
    _add_pick_cell(stage, layout, mat_pick)
    _add_amr_spawn_markers(stage, layout, mat_spawn)

    stage.GetRootLayer().Save()
    return str(out_usd)


def _add_shelves(stage, layout: LayoutConfig, material) -> None:
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
            _bind(prim, material)


def _add_pick_cell(stage, layout: LayoutConfig, material) -> None:
    """Block placeholder for the manipulator's pick cell base."""
    from pxr import Gf, UsdGeom

    px, py = layout.pick_cell.position_xy
    base = UsdGeom.Cube.Define(stage, "/World/PickCell/Base")
    base.CreateSizeAttr(1.0)
    UsdGeom.Xformable(base).AddTranslateOp().Set(Gf.Vec3d(px, py, 0.5))
    UsdGeom.Xformable(base).AddScaleOp().Set(Gf.Vec3d(1.5, 1.5, 1.0))
    _bind(base, material)


def _add_amr_spawn_markers(stage, layout: LayoutConfig, material) -> list[tuple[float, float]]:
    """Marker cubes at each AMR spawn pose; returns the pose list.

    Markers are sized 0.5x0.5x0.5m (was 0.3x0.3x0.1) so they're visible in an
    overhead render, not invisible specks.
    """
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
            UsdGeom.Xformable(m).AddTranslateOp().Set(Gf.Vec3d(x, y, 0.25))
            UsdGeom.Xformable(m).AddScaleOp().Set(Gf.Vec3d(0.5, 0.5, 0.5))
            _bind(m, material)
            idx += 1
    return poses


def build_from_yaml(layout_path: str | Path, out_usd: str | Path) -> str:
    """Convenience: load layout YAML and build the scene in one call."""
    return build_scene(load_layout(layout_path), out_usd)


def _main(argv: list[str] | None = None) -> int:
    """CLI: `python -m warehouse.generators.build_scene <layout> [out_usd]`.

    Resolves <layout> to warehouse/layouts/<layout>.yaml. If out_usd is
    omitted, writes to outputs/scenes/<layout>.usd alongside the repo.
    """
    import sys

    args = list(sys.argv[1:] if argv is None else argv)
    if not args or args[0] in {"-h", "--help"}:
        print("usage: python -m warehouse.generators.build_scene <layout-name> [out_usd]")
        return 2

    layout_name = args[0]
    layout_path = Path(__file__).resolve().parents[1] / "layouts" / f"{layout_name}.yaml"
    if not layout_path.exists():
        print(f"error: layout file not found: {layout_path}")
        return 1

    out_usd = Path(args[1]) if len(args) > 1 else Path("outputs/scenes") / f"{layout_name}.usd"
    out_usd.parent.mkdir(parents=True, exist_ok=True)

    path = build_from_yaml(layout_path, out_usd)
    print(f"wrote {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())
