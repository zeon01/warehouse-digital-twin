"""Isaac Sim 5.0 Camera + ROS2CameraHelper plumbing for the M5 pick cell.

Spawns a USD Camera prim looking at the pick table, then builds an on-demand
OmniGraph that wires it through `IsaacCreateViewport` + `ROS2CameraHelper`
nodes to publish:

    /cell/cam/rgb     sensor_msgs/Image       (rgb8)
    /cell/cam/depth   sensor_msgs/Image       (32FC1, meters)
    /cell/cam/info    sensor_msgs/CameraInfo

with `frame_id = "cell_cam_optical"`. The orchestrator does a tf2 lookup
from cell_cam_optical → panda_link0 before passing FoundationPose's pose to
TopDownGrasp.

Default geometry (per M5 v13 reachability math, see warehouse-digital-twin
plan):

    camera position: world (16.40, 14.20, 1.50)
    camera Euler XYZ: (46.8°, 0, 0)  — tilts default look (-Z) down-and-north
                                       to point at cube center (16.40, 15.0, 0.75)
    cube distance:   1.097 m
    cube projection: ~53 px in 640x480 with default fL=24mm, hA=21mm, vA=16mm

The static TF `world → cell_cam_optical` is broadcast by run_scenario.py
via a `static_transform_publisher` subprocess (not from this module) so the
TF doesn't compete with Isaac Sim's articulation TF tree.
"""

from __future__ import annotations

from collections.abc import Sequence

# These values are also encoded in run_scenario.py's static_transform_publisher
# call — keep them in sync if you change one.
DEFAULT_CAMERA_POS_WORLD: tuple[float, float, float] = (16.40, 14.20, 1.50)
DEFAULT_CAMERA_EULER_XYZ_DEG: tuple[float, float, float] = (46.8, 0.0, 0.0)
DEFAULT_CAMERA_RESOLUTION: tuple[int, int] = (640, 480)
# Default Isaac Sim Camera intrinsics. With 640x480: fx ≈ 731, fy ≈ 720.
DEFAULT_FOCAL_LENGTH_MM = 24.0
DEFAULT_HORIZONTAL_APERTURE_MM = 21.0
DEFAULT_VERTICAL_APERTURE_MM = 16.0

CAMERA_PRIM_PATH = "/World/cell_cam"
ROS_CAMERA_GRAPH_PATH = "/World/cell_cam_ros"
OPTICAL_FRAME_ID = "cell_cam_optical"


def spawn_cell_camera(
    position_xyz: Sequence[float] = DEFAULT_CAMERA_POS_WORLD,
    euler_xyz_deg: Sequence[float] = DEFAULT_CAMERA_EULER_XYZ_DEG,
    focal_length_mm: float = DEFAULT_FOCAL_LENGTH_MM,
    horizontal_aperture_mm: float = DEFAULT_HORIZONTAL_APERTURE_MM,
    vertical_aperture_mm: float = DEFAULT_VERTICAL_APERTURE_MM,
    prim_path: str = CAMERA_PRIM_PATH,
):
    """Create a USD Camera prim at the pick cell. Returns the UsdGeom.Camera."""
    import omni
    from pxr import Gf, UsdGeom

    stage = omni.usd.get_context().get_stage()
    cam_prim = stage.DefinePrim(prim_path, "Camera")
    camera = UsdGeom.Camera(cam_prim)

    xform_api = UsdGeom.XformCommonAPI(camera)
    xform_api.SetTranslate(Gf.Vec3d(*position_xyz))
    xform_api.SetRotate(
        Gf.Vec3f(*euler_xyz_deg),
        UsdGeom.XformCommonAPI.RotationOrderXYZ,
    )

    camera.GetProjectionAttr().Set("perspective")
    camera.GetFocalLengthAttr().Set(focal_length_mm)
    camera.GetHorizontalApertureAttr().Set(horizontal_aperture_mm)
    camera.GetVerticalApertureAttr().Set(vertical_aperture_mm)
    # focusDistance only matters for DOF rendering; pick a far value so the
    # workspace is fully in focus.
    camera.GetFocusDistanceAttr().Set(400.0)

    return camera


def build_ros2_camera_graph(
    resolution: Sequence[int] = DEFAULT_CAMERA_RESOLUTION,
    rgb_topic: str = "/cell/cam/rgb",
    depth_topic: str = "/cell/cam/depth",
    info_topic: str = "/cell/cam/info",
    frame_id: str = OPTICAL_FRAME_ID,
    camera_prim_path: str = CAMERA_PRIM_PATH,
    graph_path: str = ROS_CAMERA_GRAPH_PATH,
):
    """Build an on-demand OmniGraph that publishes the camera over ROS2.

    Follows NVIDIA's canonical pattern from
    /isaac-sim/standalone_examples/api/isaacsim.ros2.bridge/camera_periodic.py
    but parameterized for the M5 cell camera. Two ROS2CameraHelper nodes
    fan out off a shared IsaacCreateViewport → IsaacGetViewportRenderProduct
    → IsaacSetCameraOnRenderProduct chain so RGB and Depth share the same
    render product.
    """
    import omni.graph.core as og
    import usdrt.Sdf

    keys = og.Controller.Keys
    # NOTE: `resolution` parameter currently unused — IsaacCreateViewport in
    # Isaac Sim 5.0 doesn't accept width/height inputs. Default viewport
    # resolution applies. Kept in the signature for forward-compat.
    del resolution

    (graph, _, _, _) = og.Controller.edit(
        {
            "graph_path": graph_path,
            "evaluator_name": "push",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
        },
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),
                ("createViewport", "isaacsim.core.nodes.IsaacCreateViewport"),
                ("getRenderProduct", "isaacsim.core.nodes.IsaacGetViewportRenderProduct"),
                ("setCamera", "isaacsim.core.nodes.IsaacSetCameraOnRenderProduct"),
                ("camRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("camDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("camInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "createViewport.inputs:execIn"),
                ("createViewport.outputs:execOut", "getRenderProduct.inputs:execIn"),
                ("createViewport.outputs:viewport", "getRenderProduct.inputs:viewport"),
                ("getRenderProduct.outputs:execOut", "setCamera.inputs:execIn"),
                (
                    "getRenderProduct.outputs:renderProductPath",
                    "setCamera.inputs:renderProductPath",
                ),
                ("setCamera.outputs:execOut", "camRgb.inputs:execIn"),
                ("setCamera.outputs:execOut", "camDepth.inputs:execIn"),
                ("setCamera.outputs:execOut", "camInfo.inputs:execIn"),
                ("getRenderProduct.outputs:renderProductPath", "camRgb.inputs:renderProductPath"),
                ("getRenderProduct.outputs:renderProductPath", "camDepth.inputs:renderProductPath"),
                ("getRenderProduct.outputs:renderProductPath", "camInfo.inputs:renderProductPath"),
            ],
            keys.SET_VALUES: [
                # Bind a NEW viewport id so we don't collide with viewport 0
                # (the default fleet/sim viewport). Isaac Sim 5.0's
                # IsaacCreateViewport only exposes execIn/name/viewportId —
                # NO width/height/resolution. Default viewport resolution
                # is fine; FoundationPose's input crop is 160x160 from the
                # detected bounding box, so larger frames just mean a more
                # detailed source crop.
                ("createViewport.inputs:viewportId", 7),
                ("setCamera.inputs:cameraPrim", [usdrt.Sdf.Path(camera_prim_path)]),
                ("camRgb.inputs:frameId", frame_id),
                ("camRgb.inputs:topicName", rgb_topic),
                ("camRgb.inputs:type", "rgb"),
                ("camDepth.inputs:frameId", frame_id),
                ("camDepth.inputs:topicName", depth_topic),
                ("camDepth.inputs:type", "depth"),
                ("camInfo.inputs:frameId", frame_id),
                ("camInfo.inputs:topicName", info_topic),
            ],
        },
    )

    # Evaluate once so the SDGPipeline annotators attach and the ROS
    # publishers register.
    og.Controller.evaluate_sync(graph)
    return graph
