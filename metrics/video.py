"""Assemble Replicator-emitted rgb_*.png frames into an MP4 via ffmpeg.

Replicator's BasicWriter dumps `rgb_0000.png`, `rgb_0001.png`, ... at a
configurable cadence (one frame per `rep.orchestrator.step()` call). Once
those frames are on disk, this module stitches them into a single MP4.

The function shells out to `ffmpeg` — assumes it's on PATH. On the
vast.ai instance our Task 7 image bootstrap installs ffmpeg as a
top-level apt dep (alongside curl, gnupg2, etc.). For local Mac dev,
`brew install ffmpeg`.
"""

from __future__ import annotations

import subprocess
from pathlib import Path


def assemble_mp4(frame_dir: str | Path, out_mp4: str | Path, fps: int = 30) -> str:
    """Stitch `<frame_dir>/rgb_%04d.png` into `out_mp4` at the given fps."""
    frame_dir = Path(frame_dir)
    out = str(out_mp4)
    pattern = str(frame_dir / "rgb_%04d.png")
    cmd = [
        "ffmpeg",
        "-y",
        "-framerate",
        str(fps),
        "-i",
        pattern,
        "-c:v",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        "-vf",
        "scale=1920:1080",
        out,
    ]
    subprocess.run(cmd, check=True)
    return out
