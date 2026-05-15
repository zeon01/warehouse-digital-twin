"""Unit tests for metrics.aggregate — Phase 2 ablation cross-run summary."""

from __future__ import annotations

from pathlib import Path


def _write_metrics(d: Path, payload: str) -> None:
    d.mkdir(parents=True, exist_ok=True)
    (d / "metrics.json").write_text(payload)


def test_aggregate_reads_metrics(tmp_path: Path):
    from metrics.aggregate import aggregate_runs

    for cfg in ["A", "B"]:
        for s in [1, 2]:
            _write_metrics(
                tmp_path / cfg / str(s),
                '{"orders_completed": 10, "pick_success_rate": 0.8, '
                '"deadlocks": 1, "mean_cycle_time_s": 30.0}',
            )
    df = aggregate_runs(tmp_path)
    assert len(df) == 4
    assert set(df["config"]) == {"A", "B"}
    assert set(df["seed"]) == {1, 2}


def test_aggregate_summarize_mean_std(tmp_path: Path):
    from metrics.aggregate import aggregate_runs, summarize_by_config

    for s in [1, 2, 3]:
        _write_metrics(
            tmp_path / "A" / str(s),
            '{"orders_completed": ' + str(10 + s) + ', "pick_success_rate": 0.8, '
            '"deadlocks": ' + str(s) + ', "mean_cycle_time_s": 30.0}',
        )
    df = aggregate_runs(tmp_path)
    summary = summarize_by_config(df)
    a = summary[summary["config"] == "A"].iloc[0]
    assert a["orders_completed_mean"] == 12.0  # (11+12+13)/3
    assert abs(a["deadlocks_std"] - 1.0) < 0.001  # std([1,2,3]) ≈ 1


def test_aggregate_empty_dir_returns_empty_frame(tmp_path: Path):
    from metrics.aggregate import aggregate_runs

    df = aggregate_runs(tmp_path)
    assert len(df) == 0
