import json

from metrics.recorder import MetricsRecorder


def test_recorder_aggregates_order_lifecycle(tmp_path):
    rec = MetricsRecorder(out_dir=tmp_path)
    rec.on_order_enqueued(order_id="o1", at=0.0)
    rec.on_order_assigned(order_id="o1", robot_id="a", at=1.0)
    rec.on_order_completed(order_id="o1", at=50.0, pick_success=True, pick_attempts=1)
    rec.on_deadlock_detected(robots=("a", "b"), at=20.0)
    rec.flush()

    data = json.loads((tmp_path / "metrics.json").read_text())
    assert data["orders_total"] == 1
    assert data["orders_completed"] == 1
    assert data["pick_success_rate"] == 1.0
    assert data["deadlocks_total"] == 1
    assert data["avg_cycle_time_s"] == 50.0
