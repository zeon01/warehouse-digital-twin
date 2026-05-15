"""Aggregate per-run metrics.json files into a DataFrame + per-config summaries.

The Phase 2 ablation runner (``wdt_vast/run_ablation.py``) writes one
``<out_root>/<config>/<seed>/metrics.json`` per (config, seed) pair.
This module reads them all into a flat DataFrame and produces a
mean-±-std summary table for the Phase 2 results writeup.
"""

from __future__ import annotations

import json
from pathlib import Path

import pandas as pd

METRIC_COLS: list[str] = [
    "orders_completed",
    "pick_success_rate",
    "deadlocks",
    "mean_cycle_time_s",
]


def aggregate_runs(root: Path | str) -> pd.DataFrame:
    """Read every ``<config>/<seed>/metrics.json`` under ``root``.

    Returns a long DataFrame with columns: ``config``, ``seed``, plus
    every entry in :data:`METRIC_COLS`. Missing metric keys come through
    as ``None`` (DataFrame ``NaN``) so consumers can detect partial runs.

    An empty directory returns an empty DataFrame (no schema enforced).
    """
    rows: list[dict] = []
    for metrics_path in Path(root).glob("*/*/metrics.json"):
        config = metrics_path.parent.parent.name
        try:
            seed = int(metrics_path.parent.name)
        except ValueError:
            continue
        try:
            data = json.loads(metrics_path.read_text())
        except json.JSONDecodeError:
            continue
        row: dict = {"config": config, "seed": seed}
        for col in METRIC_COLS:
            row[col] = data.get(col)
        rows.append(row)
    return pd.DataFrame(rows)


def summarize_by_config(df: pd.DataFrame) -> pd.DataFrame:
    """Mean + std for each metric, grouped by config.

    Returns a wide DataFrame with one row per config and columns
    ``<metric>_mean`` / ``<metric>_std`` for every metric in
    :data:`METRIC_COLS`. Empty input → empty DataFrame.
    """
    if df.empty:
        return pd.DataFrame()
    agg = df.groupby("config")[METRIC_COLS].agg(["mean", "std"]).reset_index()
    # After reset_index, columns are a MultiIndex with the group-by key
    # at (name, ''). Flatten to plain strings, preserving 'config' as-is.
    flat: list[str] = []
    for c in agg.columns:
        if isinstance(c, str):
            flat.append(c)
        elif c[1] == "":
            flat.append(c[0])
        else:
            flat.append(f"{c[0]}_{c[1]}")
    agg.columns = flat
    return agg
