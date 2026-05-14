"""Modal app definition for the warehouse digital twin."""

from __future__ import annotations

import modal

app = modal.App("warehouse-digital-twin")


@app.function(gpu="L4", timeout=60)
def healthcheck() -> dict[str, str]:
    """Smoke-test that the Modal app boots and a GPU is attached."""
    import subprocess

    out = subprocess.run(
        ["nvidia-smi", "--query-gpu=name", "--format=csv,noheader"],
        capture_output=True,
        text=True,
        check=True,
    )
    return {"gpu": out.stdout.strip(), "status": "ok"}
