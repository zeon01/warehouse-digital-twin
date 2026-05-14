"""Pydantic schema for a scenario YAML.

A scenario describes ONE complete simulation run: which layout to load,
how many AMRs to spawn, which planner to use, what orders to inject and
when, and whether to record video. The runner (`run_scenario`) consumes
this schema and orchestrates the spawn → coordinator → manipulation
→ video pipeline.
"""

from __future__ import annotations

from pathlib import Path

import yaml
from pydantic import BaseModel, Field, PositiveFloat, PositiveInt


class OrderSpec(BaseModel):
    id: str
    shelf_xy: tuple[float, float]
    arrival_t: float = 0.0  # seconds into run


class Scenario(BaseModel):
    name: str = Field(min_length=1)
    layout: str = "small"
    duration_s: PositiveFloat = 600.0
    orders: list[OrderSpec]
    planner: str = "cbs"
    record_video: bool = True
    overhead_camera_only: bool = True
    fleet_size: PositiveInt = 6


def load_scenario(path: str | Path) -> Scenario:
    with open(path) as fh:
        return Scenario.model_validate(yaml.safe_load(fh))
