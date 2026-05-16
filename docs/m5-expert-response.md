# M5 expert response — pick chain redesign & TF blocker

**From:** Expert consultant
**To:** Claude session that wrote `docs/m5-expert-consultation.md`
**Re:** Pick chain redesign, AMR-nav TF blocker
**Date:** 2026-05-17

---

## Headline

The bug is well-known and not yours. `inputs:nodeNamespace` on `ROS2PublishTransformTree` namespaces **only the topic**, not the frame IDs. That's the opposite of what a multi-robot ROS2 user wants, it's documented by NVIDIA users as a longstanding request, and there is no built-in fix in Isaac Sim 5.0. Hypothesis A is **the wrong direction** — frame IDs are *not* prefixed, which is exactly why pp_driver's lookup of `(map, base_link)` should succeed if anything else is correct.

So the question becomes: why is `/amr_0/tf` empty when the OG is configured to publish to it?

---

## §8.1 — Isaac Sim ROS2 bridge

### Q1: What does `inputs:nodeNamespace` do?

It prefixes the **ROS2 node namespace**, which in turn prefixes the topic (`/tf` → `/amr_0/tf`). It does **not** touch `frame_id` or `child_frame_id` strings inside the `TFMessage`. This is confirmed by NVIDIA's developer forum: a user filed a feature request in Sept 2024 asking for a `frame_prefix` attribute analogous to ROS1's `tf_prefix`, explicitly because `nodeNamespace` only renames the topic. The accepted workaround as of now is an external `tf_relay` node that republishes `/$ROBOT/tf` → `/tf` while prepending the prefix to frame IDs. No first-party fix has shipped. See: forums.developer.nvidia.com/t/305843, and the related Dec 2025 thread "TF separated in Isaac Sim ROS 2" (tagged `isaac-sim-v5-0-0`) confirming the behavior is unchanged in 5.0.

**Implication:** the TF tree, when working, should publish `odom → base_link` (raw frame IDs, no `amr_0/` prefix) onto the topic `/amr_0/tf`. pp_driver listening on `/amr_0/tf` post-remap should see exactly the `base_link` frame it's looking for. Hypothesis A is eliminated; Option 2 (force `base_frame:=amr_0/base_link`) would have been a red herring.

### Q2: Canonical TF tree for multi-namespaced Nova Carter

The canonical pattern in NVIDIA's own multi-robot examples:

- Each Carter's OG publishes `odom → base_link → <chassis chain>` onto `/amr_X/tf` (topic-namespaced, frame IDs unprefixed)
- A per-AMR `static_transform_publisher` publishes `map → odom` onto `/amr_X/tf_static`
- Each AMR's nav stack lives entirely under `/amr_X/` and uses a TransformListener that subscribes (via `/tf` → `tf` remap) to `/amr_X/tf` + `/amr_X/tf_static`

This is what we're doing. The pattern is right. Something downstream is breaking it.

### Q3: Why `/amr_0/tf` registers but is empty

Most common causes, in frequency order:

1. **The OG's `execIn` is never ticked.** `ROS2PublishTransformTree` is downstream of an `OnPlaybackTick` or `OnTick` node. If the parent OG graph isn't ticking (wrong pipeline stage, evaluator misconfigured, or the OG is in a different graph from the one Isaac actually evaluates), the publisher registers its topic on `initialize()` but never produces a message. **Single most common cause in production.** A registered-but-empty topic is the smoking gun for "the publisher node exists but its `compute` never runs."
2. **`targetPrims` is empty or invalid.** If the OG's `inputs:targetPrims` doesn't resolve to actual valid Xform prims after spawn (e.g. namespaced subtree changed the prim paths but `targetPrims` still points at the un-namespaced path), the node ticks but produces nothing.
3. **`parentPrim` is invalid** for the same reason.
4. **The graph is in `GRAPH_PIPELINE_STAGE_PRE_RENDER`** but ROS publishers should be in `SIMULATION` (or `ON_DEMAND` with explicit ticks). Wrong stage → silent ticks.

M2 was green because something about the M2 setup kept the OG ticking and `targetPrims` resolving. M5 adds extra spawns (table, cube, lighting, camera, render product) **before sim warm-up completes**. Two failure modes from that:

- The camera spawn creates a new render product and may re-author the OG graph pipeline ordering. The `IsaacCreateRenderProduct` from §4.4 / gotcha #27 is in the chain of suspicion.
- Spawning entities after the fleet may change USD composition order such that `targetPrims` paths drift. This is Hypothesis C, and the strongest candidate after H-A is eliminated.

### Q4: Built-in Isaac Sim helper for multi-AMR namespacing

There isn't one that does what we want cleanly. `isaacsim.robot_setup.multi_robot` (and predecessors) only handle prim path uniqueness; they don't fix the topic-only-namespacing limitation. The custom `_namespace_subtree` is the right approach.

Cleaner pattern people are migrating to in 5.x: don't try to namespace at the OG level at all — publish everything to `/tf` from each Carter, and **prefix the frame IDs at author-time** by editing the OG's `frameId` / `childFrameId` string inputs (or by post-processing with a `tf_relay`). Then the whole fleet's TF lives on one global `/tf` with `amr_0/base_link`, `amr_1/base_link`, etc. RViz, Nav2 multi-robot, and tf2 all work natively with this.

That's a real architectural decision. More below.

---

## §8.2 — ROS2 / TF / tf2_ros internals

### Q5: Does `remappings=[("/tf", "tf"), ("/tf_static", "tf_static")]` route the absolute `/tf` topic that `TransformListener` hardcodes?

**Yes, for both C++ and Python TransformListener implementations.** The hardcoded `/tf` and `/tf_static` strings inside tf2 *are* subject to node-level remapping rules — that's by design and has been true since Foxy. If the remap is set on the Node directive, the listener inside that node subscribes to the remapped topic.

Verify empirically:

```bash
ros2 node info /amr_0/pure_pursuit_driver
# Look at "Subscribers:" — should show /amr_0/tf and /amr_0/tf_static, not /tf and /tf_static
```

If that shows `/tf` and `/tf_static`, the remap isn't taking — possible causes:

- The remap was set inside a `GroupAction` but not on the actual `Node()` — `PushRosNamespace` is a `GroupAction` and remappings must be on the `Node` itself (or applied via `SetRemap` in the group, which is different)
- The remap key uses `tf` instead of `/tf` (must be absolute on the left side)
- `tf2_ros::Buffer` was constructed with a separate node handle that didn't inherit the remap (rare but happens with custom buffer wrappers)

Gotcha #13 catches the common form. Verify with `ros2 node info`.

### Q6: Proven pattern for multi-namespaced Nav2

What `nav2_bringup/multirobot` does is the right pattern *for ROS2 nodes*. The breakage point in Isaac-Sim-backed setups is always the **bridge between Isaac and ROS2** — namely the OG publishers, which are not ROS2 nodes in the same sense (they're OmniGraph nodes that internally use rcl-equivalent publishers). The OG-side namespacing has the limitations described in Q1.

Proven full-stack pattern, as of 5.0:

1. OG publishes to `/amr_X/tf` (topic-namespaced)
2. Per-AMR `tf_relay` (Python or C++) republishes `/amr_X/tf` → `/tf` with frame_id `amr_X/<frame>`
3. All ROS2 nodes (Nav2, RViz, pp_driver) live globally and reference `amr_X/base_link`, `amr_X/odom`

The alternative — keeping per-AMR `/amr_X/tf` topics and namespaced nodes — works but requires the kind of remap discipline already attempted and breaks down whenever anyone forgets the `/tf` remap.

### Q7: `ros2 topic pub --once` retry / 47s duplicate

`--once` publishes a single message, waits ~1s for the publisher to be discovered by subscribers, and exits. It does *not* retry.

47s between identical messages from a single `--once` invocation means **`subprocess.Popen` is being called twice from `run_scenario.py`**. The mystery isn't ros2 topic pub; it's the outer loop. Two prior occurrences of this exact pattern:

- An outer `while` keyed on a sim-time counter that increments slower than wall-time (gotcha #22 — 0.23 sim/wall ratio). The loop body re-evaluates the "should I enqueue order o1?" predicate after the first message has been sent and consumed, but before the next-order-idx state has been persisted in a way the loop body checks. 47s wall ≈ 10.8s sim — plausible for a misordered increment.
- A try/except wrapping `Popen` that re-runs on a perceived failure when the subprocess exits non-zero, and the first `ros2 topic pub --once` *does* exit non-zero if the subscriber count was zero at publish time (which it might have been, since the coordinator's TF lookup was failing and its subscriber may not have been ready).

Look at `run_scenario.py`'s order-enqueue loop with a `print(f"about to popen {cmd}")` right above the `Popen` call. The second print proves the second invocation. Independent of the TF problem; only deserves a fix after that's solved.

---

## §8.3 — Architecture

### Q8: Is worker-thread + own-executor the right answer?

Yes, unambiguously. Three reasons:

1. **It eliminates the class of bug, not just the instance.** `ReentrantCallbackGroup + MultiThreadedExecutor` works *if* every callback ever written is reentrant-safe and never calls a synchronous future from inside any callback. That's a property of *every callback ever added*, not just the ones written today. Worker-thread + own-executor is structurally immune.
2. **rclpy #1123 is unfixed.** The MTE race is real and reproducible. The maintainers acknowledge it; no fix has landed. Building on top of MTE means inheriting a known race forever.
3. **Testability.** `PickWorker` is a plain Python class with mockable dependencies. The 6 unit tests pass without rclpy. Worth a lot.

The 161 ms fast-harness cycle on a real `move_group` is sufficient validation. Write this design up as a blog post — the rclpy callback-deadlock problem is widely encountered and this solution is cleaner than what most teams reach for. A published post about it is a much better artifact than a passing CI run.

Only refinement: name the worker's `Node` distinctly (e.g. `pick_worker_node`) and put it in a separate namespace from the orchestrator so logs are easy to grep. Minor.

### Q9: Defensible portfolio scope?

Scope question, not technical. Skip.

---

## §8.4 — Pragmatic

### Q10: Which of Options 1–7 first?

Reordered with reasoning:

1. **Option 1 (empirical diagnostic) — mandatory precondition.** Five commands. The two that matter most:
   - `ros2 topic echo /amr_0/tf --once` (confirms empty, and confirms `child_frame_id` is `base_link` not `amr_0/base_link` per Q1)
   - `ros2 node info /amr_0/pure_pursuit_driver` (confirms remap actually applied per Q5)
2. **Add an OG tick diagnostic.** From within the Isaac Sim Python (kit), inspect whether the Carter OG's `ROS2PublishTransformTree` node has been ticked recently:
   ```python
   import omni.graph.core as og
   node = og.Controller.node("/World/amr_0/.../ROS2PublishTransformTree")
   print(node.get_compute_count())  # if 0 after several seconds of play, the OG isn't ticking
   ```
   This is the test that distinguishes "OG ticking but producing nothing" (→ `targetPrims` issue) from "OG not ticking at all" (→ graph pipeline / evaluator / play state issue). If the OG isn't ticking, no amount of remap fiddling will help.
3. **Option 3 (coordinator subscribes to `/amr_X/odom`) for the coordinator-pose bug only.** This is a real bug regardless of the TF outcome — the coordinator's global TF listener cannot resolve namespaced frames. Deserves a separate fix. Don't conflate it with the pp_driver TF issue.
4. **Option 4 (tf_relay) is the strategic fix.** If the OG diagnostic in (2) shows the OG is ticking and `/amr_0/tf` populates after a fresh boot, done — it was startup ordering. If `/amr_0/tf` is reliably empty, switch to the "OG publishes to /tf, tf_relay prefixes frame IDs" pattern. This aligns with the broader Isaac/Nav2 multi-robot community and removes a class of bugs.
5. **Options 2, 5: ignore.** Both rest on Hypothesis A, which is wrong.
6. **Option 6: scope decision, not a fix.** Not my call.
7. **Option 7 (Nav2 + synthetic LIDAR): Phase 3.** Don't mix it into M5.
8. **Option 8 (rollback to v7): only as a last resort.** Bisection across a redesign is expensive; would lose a day.

### Q11: "Is the fleet TF wired correctly?" diagnostic sequence

Run these in order. Each one rules out a class of failure:

```bash
# 1. Topics exist?
ros2 topic list | grep -E "amr_[0-9]+/(tf|tf_static|odom)"

# 2. Static TF (map→odom) populated?
ros2 topic echo /amr_0/tf_static --once

# 3. Dynamic TF (odom→base_link) populated? THE critical one.
timeout 3 ros2 topic echo /amr_0/tf --once || echo "EMPTY — OG not ticking or targetPrims invalid"

# 4. Frame IDs unprefixed? (Per Q1, they should be 'odom', 'base_link', NOT 'amr_0/base_link')
ros2 topic echo /amr_0/tf --once --field transforms[0].child_frame_id

# 5. pp_driver listening on the right topic?
ros2 node info /amr_0/pure_pursuit_driver | grep -A 20 Subscribers

# 6. tf2 can resolve the chain?
ros2 run tf2_ros tf2_echo map base_link --timeout 5 --topic /amr_0/tf

# 7. Carter's odom publishing? (Sanity check that OG is alive at all)
ros2 topic hz /amr_0/odom

# 8. From inside Isaac kit, OG compute count > 0?
# (See Q10 step 2)
```

Wire this into `wdt_vast/scripts/diagnose_fleet_tf.sh` so future-you doesn't have to remember.

### Q12: Kit Python 3.11 vs Humble rclpy 3.10

Three patterns in increasing order of cleanliness:

1. **Current pattern (shell out to /usr/bin/python3)** — fine for static publishers and single-shot tools. It's what was done for `sim_world_pose_publisher.py` and it works.
2. **Use the Isaac Sim ROS2 bridge directly from kit Python.** The bridge exposes a `omni.isaac.ros2_bridge` (or `isaacsim.ros2.bridge` in 5.0) Python API that bypasses rclpy entirely — it uses the bridge's internal rcl wrapper. For publishing simple messages (the cube pose case), this works from kit Python 3.11 without needing rclpy at all. Specifically: there are `ROS2Publisher` OG nodes constructable programmatically from kit. This is what NVIDIA's own examples do.
3. **Build rclpy from source for Python 3.11.** Theoretically possible, practically painful. Don't.

For `sim_world_pose_publisher.py`, (1) is fine. For more complex bridge interactions (subscribing to ROS topics from kit Python, dynamic OG authoring with ROS messages), invest in (2). Don't go to (3).

---

## What to do tomorrow morning

1. Resume Spain. Run the §Q11 diagnostic sequence. Capture all 8 outputs in `/tmp/fleet_tf_diag.log`.
2. The diagnostic tells which of two worlds:
   - **World A: `/amr_0/tf` is reliably empty.** OG isn't ticking or `targetPrims` is invalid post-spawn. Fix the spawn order or the OG construction. Most likely culprit: something in the cell-camera or pick-table spawn is invalidating Carter's OG state. Bisect by commenting out spawns one at a time. Each iteration is ~2 minutes on a warm instance.
   - **World B: `/amr_0/tf` populates but pp_driver still fails the lookup.** Then it's the remap (Q5) or QoS mismatch (Hypothesis D). `ros2 node info` resolves it in one command.
3. Fix the coordinator's global-TF-listener bug (Option 3) regardless of which world. It's a real bug and it's masking signal in logs.
4. Once M5 is green: spend an afternoon rewriting the fleet to publish to global `/tf` with prefixed frame IDs (the tf_relay pattern). Removes the entire class of "namespace topic remap" bugs. The migration is mechanical and never debug it again.

---

## What's worth pushing back on in the consultation doc

Three things flagged on review:

1. **Proposed 8 options before running the empirical diagnostic.** The diagnostic is one command. The doc's bias toward "design alternatives" over "observe the system" cost several iterations on the rclpy deadlock (the v20 log already showed `cycle_time_s ≈ ACTION_TIMEOUT_S` — that's the deadlock signature, no design needed to spot it) and will keep costing. Observation cheaper than design, every time.
2. **Hypothesis A was treated as the prior despite being eliminable in 30 seconds.** The fix to this isn't "have better priors"; it's "test priors before building on them." The diagnostic for A is `ros2 topic echo /amr_0/tf --once --field transforms[0].child_frame_id`. One line. Resolves the entire branch.
3. **The "M2 worked, M5 doesn't" framing led to looking for what's *different*** about M5. The more productive frame is "what's *the same*" — both run on the same Carter USD, same `_namespace_subtree`, same launch file. The only meaningful M5-specific addition is the post-fleet spawning sequence (table, cube, lighting, camera). That's a much narrower search space than the 5-item list in §4.4.

None of these change the technical answer. They change the speed at which future-you reaches it.

The redesign is solid; the blocker is mundane and findable. Run the diagnostic.
