"""Spawn helpers for Nova Carter (AMR) and Franka (manipulator) in Isaac Sim 5.0.

Module is only callable inside Isaac Sim's Python runtime
(/isaac-sim/kit/python/bin/python3 via /isaac-sim/python.sh).

Default asset paths point at NVIDIA's S3-hosted Isaac Sim 5.0 catalog so the
helpers work on a fresh vast.ai container without us pre-pulling anything.
Isaac Sim's USD loader resolves HTTPS URLs natively. Override `asset_usd`
to point at a local file or Modal Volume path if you've pre-pulled.
"""

from __future__ import annotations

from collections.abc import Sequence

# S3-hosted Isaac Sim 5.0 vendor-namespaced asset paths (per Task 8 discovery —
# the plan's `Robots/NovaCarter/...` and `Robots/Franka/...` paths 404 on the
# 5.0 catalog because NVIDIA reorganized under vendor folders).
_ISAAC_S3 = "https://omniverse-content-production.s3.us-west-2.amazonaws.com/Assets/Isaac/5.0/Isaac"

# Nova_Carter_ROS.usd wraps the bare nova_carter.usd and ADDS the OmniGraph
# action graphs that publish /tf, /odom, /clock and subscribe to /cmd_vel.
# The bare `Robots/NVIDIA/NovaCarter/nova_carter.usd` has NO ROS wiring — the
# bridge can be loaded but topics stay empty. Verified via the NVIDIA
# benchmark example at /isaac-sim/standalone_examples/benchmarks/
# benchmark_robots_nova_carter_ros2.py which references this exact path.
NOVA_CARTER_USD_S3 = f"{_ISAAC_S3}/Samples/ROS2/Robots/Nova_Carter_ROS.usd"
FRANKA_USD_S3 = f"{_ISAAC_S3}/Robots/FrankaRobotics/FrankaPanda/franka.usd"


def spawn_nova_carter(
    world,
    prim_path: str,
    name: str,
    position_xy: Sequence[float],
    asset_usd: str | None = None,
):
    """Spawn a Nova Carter AMR at (x, y, 0) under prim_path. Returns the robot."""
    import numpy as np
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.robot.wheeled_robots.robots import WheeledRobot

    asset_usd = asset_usd or NOVA_CARTER_USD_S3
    add_reference_to_stage(usd_path=asset_usd, prim_path=prim_path)
    # Actual joint names in the Isaac Sim 5.0 Nova Carter USD are
    # joint_wheel_left / joint_wheel_right — the plan listed
    # left_wheel_joint / right_wheel_joint which 404s with KeyError on
    # world.reset() (verified by traversing the USD and listing
    # RevoluteJoint prims).
    robot = WheeledRobot(
        prim_path=prim_path,
        name=name,
        wheel_dof_names=["joint_wheel_left", "joint_wheel_right"],
        create_robot=False,
        position=np.array([position_xy[0], position_xy[1], 0.0]),
    )
    world.scene.add(robot)
    return robot


def spawn_franka(
    world,
    prim_path: str,
    name: str,
    position_xyz: Sequence[float],
    asset_usd: str | None = None,
):
    """Spawn a Franka Panda arm at position_xyz under prim_path. Returns the arm."""
    import numpy as np
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.robot.manipulators.examples.franka import Franka

    asset_usd = asset_usd or FRANKA_USD_S3
    add_reference_to_stage(usd_path=asset_usd, prim_path=prim_path)
    arm = Franka(
        prim_path=prim_path,
        name=name,
        position=np.array(position_xyz),
    )
    world.scene.add(arm)
    return arm


def spawn_pick_table(
    world,
    prim_path: str = "/World/pick_table",
    name: str = "pick_table",
    center_xyz: Sequence[float] = (16.40, 15.0, 0.36),
    size_xyz: Sequence[float] = (0.6, 0.6, 0.7),
    color: Sequence[float] = (0.45, 0.30, 0.18),
):
    """Spawn a static fixed-cuboid 'workbench' the cube sits on.

    Default geometry (per M5 v13 reachability math): center (16.40, 15.0, 0.36),
    size 0.6 x 0.6 x 0.7. Top face at z=0.71 places the 8 cm cube's center at
    z=0.75, which lands in the Franka workspace at panda_link0 (0.4, 0, -0.25)
    — grasp + 5 cm standoff = (0.4, 0, -0.20), reachable.
    """
    import numpy as np
    from isaacsim.core.api.objects import FixedCuboid

    table = FixedCuboid(
        prim_path=prim_path,
        name=name,
        position=np.array(center_xyz, dtype=np.float32),
        scale=np.array(size_xyz, dtype=np.float32),
        color=np.array(color, dtype=np.float32),
    )
    world.scene.add(table)
    return table


def spawn_pick_cell_lighting(
    distant_intensity: float = 4000.0,
    dome_intensity: float = 1500.0,
    distant_path: str = "/World/cell_distant_light",
    dome_path: str = "/World/cell_dome_light",
):
    """Add a distant (sun) light + dome (sky/ambient) light over the cell.

    Without this, the camera_periodic-style USD scene the M5 smoke builds
    has no light source — depth renders correctly but RGB is essentially
    black. FoundationPose uses both channels; without RGB texture it
    returns near-uniform candidate scores and the chosen pose is random.
    Verified M5 v16 — depth showed cube + table cleanly, RGB was nearly
    black with all FP scores clustered within 0.3 of each other.

    Defaults sized for the 8 cm cube on the 0.6×0.6 m table at world
    (16.40, 15.0, 0.71 table-top). Distant light angles down from world
    +Z so the cube top is well-lit. Dome light fills the shadows so FP
    can see the cube sides during refinement.
    """
    import omni
    from pxr import Gf, UsdGeom, UsdLux

    stage = omni.usd.get_context().get_stage()

    # Distant light (directional, infinite source — like sunlight). Default
    # USD distant lights shine down -Z; rotate so it also tilts slightly
    # forward, matching natural overhead-sun feel.
    distant_prim = stage.DefinePrim(distant_path, "DistantLight")
    distant = UsdLux.DistantLight(distant_prim)
    distant.CreateIntensityAttr(distant_intensity)
    UsdGeom.XformCommonAPI(distant_prim).SetRotate(
        Gf.Vec3f(-30.0, 0.0, 0.0),
        UsdGeom.XformCommonAPI.RotationOrderXYZ,
    )

    # Dome light (image-based ambient). Provides soft fill so the cube
    # sides aren't pure shadow when only the top is lit by the distant.
    dome_prim = stage.DefinePrim(dome_path, "DomeLight")
    dome = UsdLux.DomeLight(dome_prim)
    dome.CreateIntensityAttr(dome_intensity)
    return distant_prim, dome_prim


def spawn_pick_cube(
    world,
    prim_path: str = "/World/pick_cube",
    name: str = "pick_cube",
    center_xyz: Sequence[float] = (16.40, 15.0, 0.75),
    edge_m: float = 0.08,
    color: Sequence[float] = (0.85, 0.30, 0.20),
    mass_kg: float = 0.10,
):
    """Spawn a dynamic 8 cm cube on top of the pick table for the M5 smoke.

    Default geometry: center (16.40, 15.0, 0.75) — sits on the 0.6x0.6x0.7
    table at world (16.40, 15.0, 0.36). 0.04 cube half-edge + 0.71 table top
    = 0.75 cube center. FoundationPose's input CAD ('m5_smoke_box.obj' from
    run_scenario.py) is the matching 0.08 m trimesh.creation.box.
    """
    import numpy as np
    from isaacsim.core.api.objects import DynamicCuboid

    cube = DynamicCuboid(
        prim_path=prim_path,
        name=name,
        position=np.array(center_xyz, dtype=np.float32),
        scale=np.array([edge_m, edge_m, edge_m], dtype=np.float32),
        color=np.array(color, dtype=np.float32),
        mass=mass_kg,
    )
    world.scene.add(cube)
    return cube
