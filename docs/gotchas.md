# Gotchas Log

Running list of non-obvious failure modes hit during the warehouse digital
twin build. Every entry is a real bug we paid for with session time and
vast.ai compute, with the root cause and the fix. New entries appended at
the bottom; section headers correspond to milestone + iteration so you can
reconstruct chronology from `git log` if needed.

Earlier gotchas (#1-#24 — Phase 1 + M0 through M4) live in Claude's
auto-memory under `feedback-nav2-isaac-sim-gotchas.md` and
`feedback-foundationpose-install-gotchas.md` and are referenced inline in
the code with `gotcha #N`. This file picks up at #25 (M5 v11+).

---

## Phase 2 — M5 end-to-end pick chain (v11 → v15, 2026-05-16)

### #25 — `sys.path.insert(0, "/tmp")` requires /tmp to be populated

**Symptom (v11):** `ModuleNotFoundError: No module named 'sim'` at line 139
of `run_scenario.py` during Kit boot.

**Root cause:** `run_scenario.py` does `sys.path.insert(0, "/tmp")` on
startup (so it can import the non-colcon repo-root packages `sim`,
`manipulation`, `metrics`, `warehouse`). Those packages are expected to
exist at `/tmp/sim/`, `/tmp/manipulation/`, etc. After a vast.ai stop/start
cycle, `/tmp` is wiped (gotcha #12) so the symlinks/copies vanish.

**Fix:** After every fresh instance start, recreate the symlinks before
launching:

```bash
ln -sfn /work/sim         /tmp/sim
ln -sfn /work/manipulation /tmp/manipulation
ln -sfn /work/metrics     /tmp/metrics
ln -sfn /work/warehouse   /tmp/warehouse
ln -sfn /work/coordinator /tmp/coordinator
ln -sfn /work/nav_drivers /tmp/nav_drivers
```

### #26 — Subprocess PYTHONPATH must include `/tmp`

**Symptom (v12):** `pick_cell_orchestrator` died at startup with
`ModuleNotFoundError: No module named 'manipulation'` even though
`/tmp/manipulation` symlink existed.

**Root cause:** The orchestrator runs as a `ros2 run` subprocess. That
subprocess inherits its PYTHONPATH from the parent's `ros_env`
(`run_scenario.py:209`), which strips `/isaac-sim/` paths but does NOT
add `/tmp`. The parent's own `sys.path.insert(0, "/tmp")` only affects the
Kit process, not its children.

**Fix:** `run_scenario.py` now appends `/tmp` to the cleaned PYTHONPATH
before launching subprocesses (see comment block ~line 210).

### #27 — Isaac Sim 5.0 `IsaacCreateViewport` has no width/height inputs

**Symptom (v13):** `OmniGraphError: Failed trying to look up attribute with
(/World/cell_cam_ros/createViewport.inputs:width, node=None, graph=None)`
when building the camera ROS publisher graph.

**Root cause:** In Isaac Sim 5.0, `IsaacCreateViewport.ogn` exposes only
`execIn`, `name`, and `viewportId`. NVIDIA's older standalone examples
(camera_periodic.py etc.) don't set width/height either — viewport
resolution is controlled via the render product or the camera prim, not
via OG inputs.

**Fix:** Don't pass `width`/`height` in the OG SET_VALUES. Default viewport
resolution applies; FoundationPose's 160x160 crop tolerates a wide range
of source resolutions.

### #28 — MoveIt's "Found empty JointState message" is a benign warning

**Symptom (v12, v13, v14):** `move_group` log spammed with
`[ERROR] [moveit_robot_state.conversions]: Found empty JointState message`.

**Root cause:** Misleading error level. `MotionPlanRequest.start_state` is
a default-constructed message with an empty `joint_state` field; MoveIt
logs this as an ERROR but then falls back to the current state from its
planning_scene_monitor (which subscribes to `/joint_states`). The downstream
behavior depends entirely on whether the planning scene has a valid current
state from somewhere else.

**Fix:** Ignore this message during debugging. Look for the actual planner
outcome below it (`Skipping invalid start state`, `Unable to sample any
valid states for goal tree`, `Unable to find solution`, etc.).

### #29 — Isaac Sim's Franka articulation does NOT publish `/joint_states`

**Symptom (v13):** MoveIt rejected every plan with
`Found a contact between 'panda_link5' and 'panda_link7'` →
`fix_start_state_collision: Unable to find a valid state nearby` →
`Skipping invalid start state` → `Motion planning start tree could not be
initialized`.

**Root cause:** The Panda URDF's zero-position (all joints at 0) puts
panda_link5 and panda_link7 in self-collision (gotcha #20). MoveIt's
planning_scene_monitor needs a valid current state from `/joint_states`,
but Isaac Sim 5.0's Franka articulation does NOT auto-publish it. With no
publisher, MoveIt falls back to the zero-pose → invalid start state →
every plan fails.

**Fix:** Launch `wdt_vast/franka_ready_joint_states.py` as a subprocess.
Publishes the canonical Panda "ready" pose (all 7 arm joints + 2 finger
joints) at 10 Hz on `/joint_states`. Same approach as the M3 smoke. Should
be replaced once Isaac Sim's Franka articulation publishes live joint
states via an OmniGraph (Phase 3).

### #30 — `OrientationConstraint` 0.1 rad on all axes is too tight

**Symptom (v14):** `panda_arm: Unable to sample any valid states for goal
tree` → 5-second planning timeout → `exhausted_candidates`. Position
target was demonstrably reachable.

**Root cause:** `manipulation/motion_planning.py:_build_goal` constructed
the OrientationConstraint with `absolute_{x,y,z}_axis_tolerance = 0.1`
(5.7° per axis). For a top-down grasp pose (`R = [[1,0,0],[0,-1,0],[0,0,-1]]`,
gripper Z = world −Z), the Franka's IK needs more slack to find joint
configurations near the workspace edge.

**Fix:** Relax `absolute_z_axis_tolerance` to π (free rotation about the
gripper approach axis — fine for symmetric grippers + cubes), keep X/Y at
0.5 rad (29° tilt around the pointing axes). Verified in v15.

### #31 — `tf2_echo` CLI convention vs `lookup_transform` API convention

**Symptom (v13):** Hand-verifying TF math seemed to show wrong values from
the static_transform_publisher.

**Root cause:** Confusion about argument order semantics.
- `tf2_echo source target` (CLI) prints `T_source_from_target` — the pose
  of TARGET in SOURCE's frame.
- `tf_buffer.lookup_transform(target_frame, source_frame, time)` (API)
  returns `T_target_from_source` — applies to a point in SOURCE to give
  point in TARGET.

These two are inverses of each other. Easy to misread either one and
think the published static TF is wrong when it's actually correct.

**Fix:** When hand-verifying, always restate the conversion direction
explicitly: "I expect the position of cell_cam_optical's origin (the
camera) in panda_link0's frame to be (0.40, −0.80, 0.50). Which lookup
gives me that?"

### #32 — TF tree composition for the pick cell needs `world → panda_link0`

**Symptom (v13):** `move_group` log: `Unable to update multi-DOF joint
'virtual_joint': Failure to lookup transform between 'world' and
'panda_link0'`. Orchestrator's `lookup_transform(panda_link0,
cell_cam_optical)` worked only because both were anchored via static TFs.

**Root cause:** `robot_state_publisher` (launched via
`wdt_franka_description/franka_description.launch.py`) publishes the
Franka URDF's internal TF chain panda_link0 → panda_link1 → ... → panda_link8.
It does NOT publish `world → panda_link0` because the URDF doesn't define
a `<world>` link. MoveIt's panda_moveit_config defines a `virtual_joint`
expecting world → panda_link0 to exist.

**Fix:** Launch a separate `static_transform_publisher` for
`world → panda_link0` matching the Franka's spawn position (e.g., world
(16, 15, 1.0) with identity rotation). Added to `run_scenario.py` as the
`panda_link0_static_tf` subprocess.

### #33 — Broad `pkill` over SSH drops the session

**Symptom:** SSH connection terminated with exit 255 after running
`pkill -f franka_ready_joint_states` (and other patterns).

**Root cause:** `pkill -f <pattern>` matches the FULL command line of
every process, including SSH-forwarded shells whose command lines often
contain other process names as arguments. Killing those terminates the
SSH session.

**Fix:** Always use `pkill -x <exact_name>` over SSH (exact match on the
process name, no command-line scanning). For the few scripts that don't
have a clean process name (e.g., Python scripts where `argv[0]` is just
`/usr/bin/python3`), use `pgrep` first to find the PID list and `kill -KILL`
those specific PIDs.

### #34 — Killing the bash wrapper doesn't kill the Kit child

**Symptom:** After `kill -KILL <bash_wrapper_pid>` for v13's run, a fresh
v14 launch was still being throttled by leftover v13 ROS2 nodes. `ps aux`
showed `/isaac-sim/kit/python/bin/python3 /work/wdt_vast/run_scenario.py
... /tmp/m5_smoke_v13` still running.

**Root cause:** `/isaac-sim/python.sh` is a bash wrapper that `exec`s into
the kit python binary. After `exec`, the bash PID is replaced by the kit
python — but our `nohup bash ... &` recorded the BASH pid, not the post-exec
kit pid. Killing the bash PID kills nothing because bash already exited.
Meanwhile the kit python (which now lives under that PID's slot or its
own) is detached via `start_new_session=True` and survives our
`proc.terminate()` calls.

**Fix:** After killing the recorded parent PID, always grep `ps aux` for
`/isaac-sim/kit/python/bin/python3` and kill those orphans too. Better
long-term: capture the actual kit python PID via `pgrep -P <bash_pid>` and
kill that one instead.
