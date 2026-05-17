# Captured run artifacts from California RTX 5090 (vast.ai instance 36905209)

Pulled 2026-05-17 before instance destruction. Source: `/tmp/m5_smoke_v22*` and
`/tmp/m7_steady_state_gt/` plus parent `run_scenario.py` stdout for each run.

| Run | Mode | Result | Subdir |
|---|---|---|---|
| v22g | gt | orders_completed=1, pick=333 ms, metrics.json bug (recorder uncounted) | `m5_smoke_v22g_gt/` |
| v22h | gt | orders_completed=1, pick=161 ms, metrics.json clean — **v0.2.0 release evidence** | `m5_smoke_v22h_gt/` |
| v22_fp | fp | success=false, FP CUDA-no-kernel on sm_120 Blackwell — caught cleanly by PickWorker | `m5_smoke_v22_fp/` |
| M7 shakedown | gt | 8/64 picked, 4 nav-failed, 2 dead pp_drivers (rclpy ActionServer race) — M7 done | `m7_steady_state_gt/` |

Each run subdir contains `coordinator.log`, `pick_orch.log`, `pure_pursuit.log`,
`move_group.log`, `world_cube_pose.log`, `franka_ready_joint_states.log`,
`cell_cam_static_tf.log`, `panda_link0_static_tf.log`, `progress.txt`, and
(where the run finished cleanly) `metrics.json` + `events.log`.

The top-level `*.log` files are `run_scenario.py`'s combined stdout/stderr —
mostly Isaac Sim init noise + USD/Replicator warnings, useful as a record of
extension load order and Vulkan/rclpy interactions.

`bootstrap_phase{1,2}.log`, `install_fp.log`, `colcon_build.log` are the
host-bootstrap traces — proof the install chain reproduces on a fresh
California 5090 with NGC pull at host_id 155125.
