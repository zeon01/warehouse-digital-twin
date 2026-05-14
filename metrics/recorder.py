"""Aggregates fleet events into a single metrics.json + events.log per run.

Hooks (`on_order_*`, `on_deadlock_detected`) are called by the coordinator
node throughout a run; `flush()` writes the summary at the end. Used by
the scenario runner and acceptance tests to compute throughput, pick
success rate, cycle time, and deadlock counts.
"""

from __future__ import annotations

import json
import statistics
from dataclasses import dataclass, field
from pathlib import Path


@dataclass
class _Order:
    enqueued_at: float = 0.0
    assigned_at: float | None = None
    completed_at: float | None = None
    pick_success: bool | None = None
    pick_attempts: int = 0


@dataclass
class MetricsRecorder:
    out_dir: Path
    _orders: dict[str, _Order] = field(default_factory=dict)
    _deadlocks: list[tuple[float, tuple[str, ...]]] = field(default_factory=list)
    _events: list[str] = field(default_factory=list)

    def on_order_enqueued(self, order_id: str, at: float) -> None:
        self._orders[order_id] = _Order(enqueued_at=at)
        self._events.append(f"{at:.3f} ENQ {order_id}")

    def on_order_assigned(self, order_id: str, robot_id: str, at: float) -> None:
        self._orders[order_id].assigned_at = at
        self._events.append(f"{at:.3f} ASN {order_id} -> {robot_id}")

    def on_order_completed(
        self, order_id: str, at: float, pick_success: bool, pick_attempts: int
    ) -> None:
        o = self._orders[order_id]
        o.completed_at = at
        o.pick_success = pick_success
        o.pick_attempts = pick_attempts
        self._events.append(
            f"{at:.3f} DONE {order_id} success={pick_success} attempts={pick_attempts}"
        )

    def on_deadlock_detected(self, robots: tuple[str, ...], at: float) -> None:
        self._deadlocks.append((at, robots))
        self._events.append(f"{at:.3f} DEADLOCK {' '.join(robots)}")

    def flush(self) -> None:
        Path(self.out_dir).mkdir(parents=True, exist_ok=True)
        completed = [o for o in self._orders.values() if o.completed_at is not None]
        cycle_times = [
            o.completed_at - o.enqueued_at for o in completed if o.completed_at is not None
        ]
        pick_results = [o.pick_success for o in completed if o.pick_success is not None]
        data = {
            "orders_total": len(self._orders),
            "orders_completed": len(completed),
            "pick_success_rate": (sum(pick_results) / len(pick_results) if pick_results else 0.0),
            "avg_cycle_time_s": statistics.mean(cycle_times) if cycle_times else 0.0,
            "p95_cycle_time_s": (
                statistics.quantiles(cycle_times, n=20)[-1] if len(cycle_times) >= 20 else 0.0
            ),
            "deadlocks_total": len(self._deadlocks),
        }
        (Path(self.out_dir) / "metrics.json").write_text(json.dumps(data, indent=2))
        (Path(self.out_dir) / "events.log").write_text("\n".join(self._events))
