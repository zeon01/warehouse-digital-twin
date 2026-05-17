# rclpy ActionServer KeyError — comment to post on ros2/rclpy#1236

The existing issue [ros2/rclpy#1236](https://github.com/ros2/rclpy/issues/1236) (Feb 2024) describes the same `KeyError` in `_execute_goal`. It was closed via PR #1363, but that PR only adjusted a log message from DEBUG to WARN — the race itself was not fixed, and the PR was reverted in October 2024 anyway. The bug is still reproducible.

This file holds a comment ready to paste on issue #1236 to reopen the discussion with new evidence.

---

## Comment body (paste verbatim on #1236)

Hitting this same `KeyError` deterministically on Humble (`rclpy 3.3.21`), but from a different trigger than the original sim-time-fast-clock report — and it looks like PR #1363 only adjusted a log message (DEBUG → WARN) and was reverted, so the underlying race is still live.

**Trigger:** real-time goal-timeout abort under sustained load. Our `NavigateToPose` action server runs a long-running control loop and aborts goals when wall time exceeds `goal_timeout_s = 1200.0`. Two out of six action servers crashed with this KeyError in a single 64-order simulation run:

```
[pure_pursuit_driver-6] [WARN] goal aborted: timeout after 1200.0s (>1200.0s)
[pure_pursuit_driver-6] Traceback (most recent call last):
  File ".../rclpy/executors.py", line 323, in spin
    self.spin_once()
  File ".../rclpy/executors.py", line 863, in spin_once
    self._spin_once_impl(timeout_sec)
  File ".../rclpy/executors.py", line 860, in _spin_once_impl
    future.result()
  File ".../rclpy/task.py", line 109, in result
    raise self.exception()
  File ".../rclpy/task.py", line 272, in _execute_coroutine_step
    result = coro.send(None)
  File ".../rclpy/action/server.py", line 357, in _execute_goal
    self._result_futures[bytes(goal_uuid)].set_result(result_response)
KeyError: b'S\xe2\xd7\xc6\xfd\xd7@\x02\x98^E\xc4\xe9\xcd4x'
```

Second crash same trace, different uuid (`b'\x95\x86\xafj\x8e\xe9E\xb5\x8e\xb7\x0bd/C\xe5\xa5'`). Full logs: [docs/runs/m7_steady_state_gt/pure_pursuit.log](https://github.com/zeon01/warehouse-digital-twin/blob/main/docs/runs/m7_steady_state_gt/pure_pursuit.log) in our repo.

**The race**: `_execute_expire_goals` removes the future from `self._result_futures` before `_execute_goal` runs `self._result_futures[bytes(goal_uuid)].set_result(...)`. Whoever wins the race scheduling-wise dictates whether the server crashes or not. Under steady load with periodic timeout-aborts, the loser case fires reliably.

**Suggested fix** (defensive lookup at `action/server.py:357`):

```python
# Before
self._result_futures[bytes(goal_uuid)].set_result(result_response)

# After
future = self._result_futures.get(bytes(goal_uuid))
if future is not None and not future.done():
    future.set_result(result_response)
else:
    self._logger.warn(
        f"_execute_goal: result_future for goal "
        f"{bytes(goal_uuid).hex()} unavailable (already expired or completed); "
        f"skipping set_result"
    )
```

This addresses the symptom. A real fix likely needs `_execute_expire_goals` and `_execute_goal` to coordinate on future ownership.

**Workaround we shipped (client side):** wrap `executor.spin()` in a `KeyError` retry loop ([commit](https://github.com/zeon01/warehouse-digital-twin/commit/9a909ef)). Keeps the node alive but masks the real bug.

Happy to PR the defensive-lookup change if there's interest in reopening this.

**Environment:**
- ROS 2 Humble Hawksbill
- `rclpy 3.3.21-1jammy.20260421.025639` (apt)
- Python 3.10.12
- `rmw_cyclonedds_cpp`
- Ubuntu 22.04 inside `nvcr.io/nvidia/isaac-sim:5.0.0` container, RTX 5090
