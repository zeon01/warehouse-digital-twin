# rclpy ActionServer KeyError — filed as ros2/rclpy#1667

**Filed:** https://github.com/ros2/rclpy/issues/1667 (2026-05-17)

This file holds the source body so we can iterate on follow-ups (PR description, additional repro evidence, etc.) without scraping it from github.

Related prior art (referenced inside the issue):
- [ros2/rclpy#1236](https://github.com/ros2/rclpy/issues/1236) — same KeyError, different trigger (sim-time fast clock), closed
- [ros2/rclpy#1363](https://github.com/ros2/rclpy/pull/1363) — DEBUG→WARN log adjustment that closed #1236, **reverted Oct 17 2024** — the underlying race was never addressed

---

## Title

`KeyError in ActionServer._execute_goal under goal-timeout abort (race on _result_futures)`

## Issue body

### Bug report

**Required Info:**

- Operating System: Ubuntu 22.04 (inside `nvcr.io/nvidia/isaac-sim:5.0.0` container)
- Installation type: binaries (`apt install ros-humble-rclpy`)
- Version or commit hash: `rclpy 3.3.21-1jammy.20260421.025639`
- DDS implementation: `rmw_cyclonedds_cpp`
- Client library: rclpy
- Python: 3.10.12

### Steps to reproduce

`rclpy/action/server.py:357` does `self._result_futures[bytes(goal_uuid)].set_result(result_response)` without checking whether the future is still in the dict. Under real-time load, the future can be removed by `_execute_expire_goals` before `_execute_goal` reaches that line, raising `KeyError` and crashing the executor.

Distinct trigger from #1236 (which was sim-time fast clock): **wall-clock goal-timeout abort under sustained ActionServer load**. Our `NavigateToPose` action server runs a 20 Hz control loop and aborts goals on `elapsed > goal_timeout_s = 1200.0`. Two out of six independent ActionServers crashed with this `KeyError` in a single 64-order multi-robot simulation run.

Minimal repro shape:
1. ActionServer running on `MultiThreadedExecutor`, `ReentrantCallbackGroup`
2. Long-running `execute_callback` that returns after `goal_handle.abort()` on timeout
3. Sustained load — many sequential goals per server, several servers in the process
4. Real time (`use_sim_time=False`)

### Expected behavior

Goal cleanup after abort should not race with the goal-result publish, or the publish should defensively check whether the future still exists before calling `set_result`.

### Actual behavior

```
[pure_pursuit_driver-6] [WARN] [pure_pursuit_driver]: goal aborted: timeout after 1200.0s (>1200.0s)
[pure_pursuit_driver-6] Traceback (most recent call last):
  File "/opt/ros/humble/lib/python3.10/site-packages/rclpy/executors.py", line 323, in spin
    self.spin_once()
  File "/opt/ros/humble/lib/python3.10/site-packages/rclpy/executors.py", line 863, in spin_once
    self._spin_once_impl(timeout_sec)
  File "/opt/ros/humble/lib/python3.10/site-packages/rclpy/executors.py", line 860, in _spin_once_impl
    future.result()
  File "/opt/ros/humble/lib/python3.10/site-packages/rclpy/task.py", line 109, in result
    raise self.exception()
  File "/opt/ros/humble/lib/python3.10/site-packages/rclpy/task.py", line 272, in _execute_coroutine_step
    result = coro.send(None)
  File "/opt/ros/humble/lib/python3.10/site-packages/rclpy/action/server.py", line 357, in _execute_goal
    self._result_futures[bytes(goal_uuid)].set_result(result_response)
KeyError: b'S\xe2\xd7\xc6\xfd\xd7@\x02\x98^E\xc4\xe9\xcd4x'
```

Second concurrent crash in same run, same trace, different uuid (`b'\x95\x86\xafj\x8e\xe9E\xb5\x8e\xb7\x0bd/C\xe5\xa5'`). Full logs: [docs/runs/m7_steady_state_gt/pure_pursuit.log](https://github.com/zeon01/warehouse-digital-twin/blob/main/docs/runs/m7_steady_state_gt/pure_pursuit.log).

### Suggested fix

Defensive lookup at `action/server.py:357`:

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

Addresses the symptom. A proper fix likely needs `_execute_expire_goals` and `_execute_goal` to coordinate on future ownership (e.g. atomic pop-and-handle rather than independent lookups).

### Prior art / related issues

- **#1236** reports the identical `KeyError` from a different trigger (`use_sim_time=True` with sim time accelerated 60x). It was closed as fixed by **PR #1363**.
- **PR #1363** only escalated the surrounding log line from DEBUG to WARN — it did not fix the race. It was [reverted on Oct 17, 2024](https://github.com/ros2/rclpy/pull/1363) (referenced as a revert PR on the rolling branch).
- The underlying race that produced #1236 is therefore still present and reproducible under different load shapes.

### Workaround we shipped (client side)

Wrapping `executor.spin()` in a `KeyError` retry loop keeps the node alive but masks the real bug. ([Commit](https://github.com/zeon01/warehouse-digital-twin/commit/9a909ef).)

```python
while rclpy.ok():
    try:
        executor.spin()
        break
    except KeyError as exc:
        node.get_logger().warn(
            f"executor.spin raised KeyError on goal-result cleanup "
            f"(rclpy ActionServer race, goal_uuid={exc!r}); resuming"
        )
        continue
```

Happy to put up a PR for the defensive-lookup change if there's interest.
