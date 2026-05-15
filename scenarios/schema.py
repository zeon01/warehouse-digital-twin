"""Pydantic schema for a scenario YAML.

A scenario describes ONE complete simulation run: which layout to load,
how many AMRs to spawn, which planner to use, what orders to inject and
when, and whether to record video. The runner (`run_scenario`) consumes
this schema and orchestrates the spawn → coordinator → manipulation
→ video pipeline.
"""

from __future__ import annotations

import random
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


def apply_seed_jitter(
    orders: list[OrderSpec],
    seed: int,
    jitter_s: float = 5.0,
) -> list[OrderSpec]:
    """Perturb each order's ``arrival_t`` by uniform noise in ±``jitter_s``.

    Used by the Phase 2 ablation runner to turn the deterministic
    ``steady_state.yaml`` into 5 stochastic variants — one per seed in
    ``{42, 43, 44, 45, 46}``. The arrival times never go negative.
    """
    rng = random.Random(seed)
    return [
        OrderSpec(
            id=o.id,
            shelf_xy=o.shelf_xy,
            arrival_t=max(0.0, o.arrival_t + rng.uniform(-jitter_s, jitter_s)),
        )
        for o in orders
    ]
