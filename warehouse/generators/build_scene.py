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
    """CLI: ``python -m warehouse.generators.build_scene <layout> [flags]``.

    Resolves ``<layout>`` to ``warehouse/layouts/<layout>.yaml``. By
    default writes the USD to ``outputs/scenes/<layout>.usd``. Phase 2
    adds ``--out-map-dir`` to additionally emit a Nav2-compatible PGM +
    YAML occupancy grid from the same layout, and ``--skip-usd`` for
    environments without ``pxr`` (local Mac unit tests).
    """
    import argparse
    import sys

    parser = argparse.ArgumentParser(prog="build_scene")
    parser.add_argument("layout_name")
    parser.add_argument(
        "out_usd_pos",
        nargs="?",
        default=None,
        help="legacy positional out_usd path (Phase 1 compat)",
    )
    parser.add_argument(
        "--out-usd",
        default=None,
        help="output USD path; default outputs/scenes/<layout>.usd",
    )
    parser.add_argument(
        "--out-map-dir",
        default=None,
        help="if set, also write <layout>.pgm + <layout>.yaml here",
    )
    parser.add_argument(
        "--skip-usd",
        action="store_true",
        help="skip USD authoring (no pxr required); use with --out-map-dir",
    )
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    layout_path = Path(__file__).resolve().parents[1] / "layouts" / f"{args.layout_name}.yaml"
    if not layout_path.exists():
        print(f"error: layout file not found: {layout_path}")
        return 1

    layout = load_layout(layout_path)

    if not args.skip_usd:
        out_usd = Path(args.out_usd or args.out_usd_pos or f"outputs/scenes/{args.layout_name}.usd")
        out_usd.parent.mkdir(parents=True, exist_ok=True)
        path = build_scene(layout, out_usd)
        print(f"wrote {path}")

    if args.out_map_dir:
        from warehouse.generators.map_export import (
            rasterize_obstacles,
            write_map_yaml,
            write_pgm,
        )

        out_map_dir = Path(args.out_map_dir)
        out_map_dir.mkdir(parents=True, exist_ok=True)
        grid = rasterize_obstacles(
            world_w_m=layout.warehouse.width_m,
            world_h_m=layout.warehouse.depth_m,
            resolution_m_per_px=0.05,
            obstacles=layout.to_obstacle_boxes(),
        )
        pgm_path = out_map_dir / f"{args.layout_name}.pgm"
        yaml_path = out_map_dir / f"{args.layout_name}.yaml"
        write_pgm(grid, pgm_path)
        write_map_yaml(
            pgm_filename=pgm_path.name,
            resolution_m_per_px=0.05,
            origin_xy_yaw=(0.0, 0.0, 0.0),
            path=yaml_path,
        )
        print(f"wrote {pgm_path}")
        print(f"wrote {yaml_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(_main())
