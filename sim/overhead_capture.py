"""Overhead camera + Replicator BasicWriter for warehouse video capture.

When `scenario.record_video=True`, run_scenario.py calls
`spawn_overhead_capture(...)` once after `world.reset()`. That spawns a
top-down camera at high altitude, attaches a Replicator render product
+ BasicWriter that drops `rgb_NNNN.png` frames into `<out_dir>/frames/`
each time the orchestrator steps. The caller is responsible for calling
`step_writer()` at the desired cadence inside the main loop — typically
every N world.step() calls to throttle wall-write IO.

Once the run finishes, `metrics.video.assemble_mp4(frame_dir, out_mp4)`
stitches the frames into a single MP4 via ffmpeg.

Design choice: Replicator's writer instead of a custom annotator + manual
PIL save. The writer handles file naming, output format, and per-tick
sync with the renderer for free. The cost is that we live with
Replicator's naming convention (`rgb_NNNN.png`); `metrics/video.py`'s
ffmpeg call uses that exact pattern.

Frame rate: at the default Isaac Sim 60 Hz render and a typical
sim/wall ratio of 0.15-0.2 on a multi-AMR scene, capturing every render
tick produces ~10-12 wall-fps video — usable. To reduce IO + storage,
call `step_writer()` every N frames (e.g. every 6 → ~2 wall-fps video).
"""

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path

# Oblique isometric-ish view of the warehouse for portfolio video.
# Camera sits south-west of the small-layout centroid (~10, 10) and
# elevated ~16 m, looking back at the centroid. Produces a tilted
# 3/4-perspective that shows shelves, robots, walls, and shadows —
# more visually informative than a flat top-down (nadir) view.
DEFAULT_OVERHEAD_POS_WORLD: tuple[float, float, float] = (-2.0, -3.0, 16.0)
DEFAULT_OVERHEAD_LOOK_AT: tuple[float, float, float] = (10.0, 10.0, 0.0)
DEFAULT_OVERHEAD_RESOLUTION: tuple[int, int] = (1920, 1080)


def spawn_overhead_capture(
    out_dir: str | Path,
    position_xyz: Sequence[float] = DEFAULT_OVERHEAD_POS_WORLD,
    look_at_xyz: Sequence[float] = DEFAULT_OVERHEAD_LOOK_AT,
    resolution: tuple[int, int] = DEFAULT_OVERHEAD_RESOLUTION,
):
    """Spawn the overhead camera + Replicator writer.

    Returns ``writer`` — call ``writer.detach()`` at shutdown if you want
    a clean teardown, otherwise it stops when the SimulationApp exits.

    Frames land at ``<out_dir>/frames/rgb_NNNN.png`` (Replicator naming).
    """
    import omni.replicator.core as rep

    frame_dir = Path(out_dir) / "frames"
    frame_dir.mkdir(parents=True, exist_ok=True)

    camera = rep.create.camera(position=tuple(position_xyz), look_at=tuple(look_at_xyz))
    render_product = rep.create.render_product(camera, resolution)
    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(output_dir=str(frame_dir), rgb=True)
    writer.attach([render_product])
    return writer


def step_writer() -> None:
    """Trigger a Replicator orchestrator step to flush one frame.

    Call this inside the main render loop at the desired cadence (e.g.
    every 6 world.step calls for ~2 wall-fps capture on a 12 wall-fps
    render). The function returns immediately; the writer's I/O runs in
    Replicator's own worker.
    """
    import omni.replicator.core as rep

    rep.orchestrator.step()
