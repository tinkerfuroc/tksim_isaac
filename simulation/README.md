# Simulation integration boundary

`tinker_sim_core` is ROS-independent and contains deterministic scenario,
command-arbitration, parity-odometry, and evaluator logic. It never imports
the Tinker workspace.

Isaac owns physics and standard ROS endpoints only:

- CPU PhysX for every behavior-validation profile.
- `/isaac_joint_states` and `/isaac_joint_commands` as the articulation
  boundary.
- NVIDIA's `isaacsim.ros2.sim_control` implementation of
  `simulation_interfaces`.
- hardware-compatible camera, LiDAR, IMU, clock, and TF topics.

System-Humble processes own FJT, gripper and task actions, xArm services,
pan-tilt compatibility, Nav2, MoveIt/cuMotion, decision, audio, and VLA.
The sole `/isaac_joint_commands` publisher is the external command gateway.

Scenario seed is immutable launch metadata. `scenario_runner` compiles scenario
files into standard reset/spawn/state operations; no custom lifecycle aliases
exist. Only the evaluator consumes `/sim/truth/*`, and claimed task success is
always checked against hidden postconditions.

## Manipulation operator slice

The manipulation launch is an external-Humble operator surface. It resolves
the current content-addressed Tinker 2 artifact and starts topic-based
`ros2_control`, the command gateway, safety supervisor, xArm/gripper/pan-tilt
facades, contract guard, truth evaluator, and scenario runner. It deliberately
does not launch Nav2, MoveIt, cuMotion, vision, decision, or VLA nodes.

Build the Humble overlay before the first run:

```bash
cd /home/tinker/tinker-sim/6.0.1
./scripts/build-humble-overlay
```

For a development run, use two terminals. Terminal A must not source system
ROS or the Tinker workspace:

```bash
cd /home/tinker/tinker-sim/6.0.1
export TINKER_ACCEPT_OMNIVERSE_EULA=Y
./scripts/launch-isaac --sensor-profile manipulation-core --profile parity \
  --scenario pick-deliver-place --seed 7 --ros
```

In Terminal B, source the system-side environment through the wrapper:

```bash
cd /home/tinker/tinker-sim/6.0.1
./scripts/launch-humble manipulation scenario:=pick-deliver-place seed:=7 \
  qualification:=false
```

The scenario runner waits for `/reset_simulation`, `/spawn_entity`, and
`/set_simulation_state` from `simulation_interfaces` before issuing reset,
an explicit stopped state, all spawns, and the final playing state. The final
state operation is marked `PHYSICS_READY`. `attempt_dir:=...` writes its
operation report to `scenario-runner.json` under that directory.

Qualification uses the auditable development attempt runner. Manifest-only is safe for
preflight; live gates require explicit gate executors:

```bash
cd /home/tinker/tinker-sim/6.0.1
export TINKER_ACCEPT_OMNIVERSE_EULA=Y
./.venv/bin/python validation/manipulation_qualification.py \
  --scenario simulation/scenarios/qualification-free-space.json \
  --manifest-only
```

This creates a manifest and provenance record only. A live development run
requires one or more `--gate-command NAME=...` executors; without them the
runner preserves a `not-configured` failure. Successful external commands are
`executed-unverified` and cannot qualify a run. No live manipulation pass is
claimed.

Operator readiness and artifact retention are documented in
[`integration/MANIPULATION.md`](../integration/MANIPULATION.md). Preserve every
attempt directory, including failed startup and timeout attempts. Do not
overwrite reports, delete failed artifacts, teleport spawned objects, bypass
the command gateway, or treat an action result as proof of a world postcondition.

This remains development-only and is not release-qualified. MoveIt and cuMotion
are deferred until the manipulation core is physically evaluated; vision,
decision, and VLA vertical slices are deferred behind the same qualification
work.

## Changelog

- 2026-08-04 (integrated qualification Task 9, fix round 5 — "final narrow
  production-suite closure"): `simulation/tinker_sim_isaac/qualification_visual_capture.py`
  now expires a request sequence that a restarted consumer can no longer satisfy
  within the bounded capture-latency contract after a partial capture (camera-1
  durably captured, camera-2 missing, and latency now above
  `MAX_CAPTURE_LATENCY_FRAMES`): it records one deduplicated terminal error,
  marks the sequence terminal/handled via the durable `visual-terminal.json`
  marker so it never retries or grows duplicate errors, preserves camera-1's
  evidence, never fabricates camera-2, and never relaxes the latency bound
  (F5.4).  Each PNG is now persisted atomically and durably (temp file, fsync,
  atomic replace, parent-directory fsync) BEFORE its keyframe journal row is
  appended and fsynced, so a journal row never references a half-written image.
  The offline integrated evidence tests now cover stale-partial restart
  terminal/no-retry/no-fabrication, in-range partial restart still completing
  the missing camera, and image-persistence failure preventing any journal
  append.  This is offline production-suite closure only; no live
  Isaac/camera/rosbag/GPU/OMPL/cuMotion claim; Task 10 still owns the Gate-F
  wiring and a load-bearing live rosbag; `_image_stats` still requires live RTX
  calibration.

- 2026-08-04 (integrated qualification Task 9, fix round 4 — "align evidence with
  real capture artifacts"): `simulation/tinker_sim_isaac/qualification_visual_capture.py`
  is now restart-safe across a partial two-camera capture.  The durable
  completion seed is per `(request_sequence, camera)`: a request sequence is
  marked durably complete only once every configured camera has a durable
  keyframe, so a crash after camera-1's keyframe but before camera-2's re-captures
  only the missing camera on restart and never duplicates a completed camera.
  The offline integrated evidence tests now exercise real nonzero capture latency
  (`raw_frame_index - requested_physics_frame_index` in `[0, 4]`), the real
  nine-field Humble rosbag2 QoS profile, canonical production CLI sheet event
  ordering, verbatim `ompl-overlay-contract.json` artifacts with root-relative
  lock paths, and exact `(scenario_id, attempt_id)` visual closure.  `_image_stats`
  thresholds still require live RTX calibration; Task 10 must wire Gate F and
  launch/finalize a load-bearing integrated rosbag; no live Isaac/camera/rosbag/
  GPU run occurred in this repair; the future qualification tooling lock remains
  absent until after review-clean Task 10.

- 2026-08-04 (integrated qualification Task 9, fix round 3 — "make integrated
  evidence production-real"): `simulation/tinker_sim_isaac/qualification_visual_capture.py`
  now seeds its handled-request-sequence set from the durable
  `visual-keyframes.jsonl` journal at construction, so a restarted capture
  consumer never re-captures an already captured request sequence (durable
  at-most-once, F3.4).  The integrated evidence pipeline is now exercised
  end-to-end by tests that drive the real executor producer
  (`_append_visual_request` / `_append_visual_event`), the real capture consumer
  (fake app/backend), the validator's index/sheets/summary, and Gate F, reaching
  `verified-pass` only with semantically valid artifacts; a diagnostic-only
  journal test proves consumer skip + validator ignore + required-events
  fail-closed.  `_image_stats` thresholds still require live RTX calibration;
  Task 10 must wire Gate F and launch/finalize a load-bearing integrated rosbag;
  no live Isaac/camera/rosbag/GPU run occurred in this repair; the future
  qualification tooling lock remains absent until after review-clean Task 10.

- 2026-08-04 (integrated qualification Task 9, fix round 2 — "produce integrated
  visual evidence"): `simulation/tinker_sim_isaac/qualification_visual_capture.py`
  now co-tenants two request shapes in the same `visual-capture-requests.jsonl`:
  the canonical EventJournal sequence-shape capture request
  (`{schema_version, sequence, gate, event, simulated_timestamp,
  source_execution_event_sequence}`) is the only capture-driving schema, and the
  executor diagnostic record
  (`{schema_version, report_revision, scenario_id, phase, capture:{kind,target},
  diagnostic_only}`) is skipped silently (never capture-driving, never
  error-spam).  Malformed/non-object records are reported exactly once
  (deduplicated error keys) and capture freshness stays within the bounded
  `MAX_CAPTURE_LATENCY_FRAMES=4` physics-frame contract.  The capture consumer is
  enabled by `TINKER_SIM_VISUAL_EVIDENCE=1` with an exact scenario id in
  `TINKER_SIM_QUALIFICATION_GATE` (set for integrated Isaac children by
  `manipulation_qualification.py::_env`).  No build, no live Isaac/ROS/GPU run in
  this fix round; the producer changes are exercised by ROS-free tests and the
  Humble-sourced executor ROS tests.
