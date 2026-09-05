# Arena-findings fixes — design

- Date: 2026-08-19
- Status: approved design, pre-implementation
- Scope owner decision record: fix every defect surfaced by the 2026-08-18/19
  RoboCup arena validation wave; no command-protocol rework, no changes to
  validated manipulation defaults.

## 1. Context and findings

The 2026-08-18/19 validation pass ran the `rcw2026` arena end to end
(spawn, sensors, base control, scenarios, navigation safety-clear) and
surfaced seven defects, plus corrected three assumptions the earlier
handoff docs had gotten wrong:

- **Head effort starvation.** The URDF's `effort="1.0"` on `pan_joint` /
  `tilt_joint` is a hand-authored placeholder — the head has massless stub
  links and no `ros2_control` entry — not a validated hardware limit. At
  1.0 Nm the head cannot move against Isaac's default-assigned link
  inertias.
- **Obstructed default arena spawn.** The rcw2026 default spawn `(0, 0)`
  sits inside `shelf_02`'s rasterized footprint. A spawn inside furniture
  corrupts odometry, lidar, and AMCL from the first tick, silently.
- **Dev-lidar raycast-floor artifact.** When the sensor origin cell is
  itself occupied, the development lidar's raycast fallback returns the
  0.3 m minimum range for every ray, producing a fake ring of returns
  instead of no returns at all. Correction to the earlier handoff: there is
  no RTX lidar in this path to blame — this is the pure dev-lidar
  raycaster, and 0.3 m is its documented occupancy-raycast floor, not a
  sensor artifact.
- **Unbounded wheel-velocity steps.** Wheel velocity targets are applied
  verbatim, with no acceleration limit and no persisted "last applied"
  state; commanders can command step changes and the wheels teleport to
  the new target in one tick, and a coasting robot (no new commands) can
  drift on stale targets rather than decelerating.
- **Unimplemented scenario actor paths.** `actors[].path` was accepted by
  the scenario schema but nothing ever drove an actor along it — no node
  interpolated the path or issued `/set_entity_state` calls.
- **No procedural-world scenario poses.** The only committed scenarios
  used `world: {"mode": "current"}`, so nothing exercised
  `find-and-approach-person` / `pick-deliver-place` style tasks against
  the actual rcw2026 arena occupancy map with map-verified spawn/actor/
  object poses.
- **Missing navigation-profile safety-clear path.** `safety_supervisor`
  only knew how to gate `/sim/hardware/safety_stop` on manipulation's
  controller-lifecycle latches. The navigation profile has no
  `/controller_manager`, so the supervisor could never publish a clear,
  and operators were working around it with a manual CLI heartbeat.
- **Corrected finding:** an EKF + IMU yaw-fusion fix for odometry yaw slip
  was investigated and **rejected** — the sim IMU publishes only
  world-frame angular velocity, with orientation explicitly marked
  invalid, so fusing it would just duplicate the existing odom `vyaw`
  signal rather than adding information. Yaw slip remains a documented
  characteristic, not a bug to fix in this wave.

## 2. User decisions

- **Spawn validation:** fail closed on an obstructed spawn, and report the
  nearest free cell as a suggested `--spawn-xy` value, rather than silently
  clamping or auto-relocating the robot.
- **Wheel slew:** implement it, wire it into `step()`, and commit it — not
  a documentation-only fix.
- **Scope:** take all four scope groups (A/B/C/D below) in this wave, not
  a subset.
- **`tk25_ws` branch:** left as-is; out of scope for this wave.

## 3. Design A — sensors and spawn

- **Head effort override.** `IsaacWholeRobotBackend`'s `"head"`
  `ImplicitActuatorCfg` gains `effort_limit_sim=10.0`, mirroring the
  existing `"arm"` / `"wheels"` actuator-group overrides that already
  correct other placeholder URDF values. Stiffness/damping unchanged. This
  is sim actuator config only; the URDF and hardware defaults are
  untouched.
- **Fail-closed spawn with suggested free cell.** `OccupancyMap` gains two
  pure helpers: `free_with_clearance(x, y, clearance_m)` (true iff no
  occupied cell lies within `clearance_m` of the point) and
  `nearest_free_world(x, y, clearance_m, max_radius_m=5.0)` (ring search
  outward in grid steps, returns the nearest clear world point or `None`).
  `validation/run_sim.py` calls a new `validate_arena_spawn(arena_dir,
  spawn_xy)` immediately after each `resolve_arena_artifact` call in all
  three robot-backend launch branches. Clearance is fixed at
  `SPAWN_CLEARANCE_M = 0.35` (0.25 m robot inscribed radius + margin). An
  obstructed spawn raises `RuntimeError` naming the nearest free cell as a
  `--spawn-xy=` suggestion; a spawn with no free cell within
  `max_radius_m` raises `RuntimeError` without a suggestion rather than
  crashing on `None`.
- **Dev-lidar empty-cloud fix.** `_development_point_cloud` now computes
  the sensor origin once per tick and checks
  `occupancy.occupied_at_world(origin_x, origin_y)` before the ray loop.
  When the origin cell is occupied, the published `PointCloud2` is empty
  (`width == 0`) instead of a synthetic ring at the 0.3 m raycast floor.
  The existing no-occupancy fallback ring (used when `occupancy is None`)
  is preserved unchanged.

## 4. Design B — base velocity slew

- A module-level pure function `slew_velocity_target(current, target,
  max_delta) -> float` bounds one wheel's per-tick velocity change,
  raising on non-finite or boolean inputs and on negative `max_delta`.
- `step()`'s non-safety branch resolves the four named wheel joints
  (`WHEEL_JOINT_NAMES`) once at init into `self._wheel_indices`, and
  maintains persistent `self._applied_wheel_velocities` state across
  ticks — the ramp is driven by this authoritative applied-velocity state,
  not by re-deriving it from the (possibly stale) command buffer each
  tick.
- The slew rate is `WHEEL_VELOCITY_SLEW_RAD_S2 = 60.0` rad/s² (~3.1 m/s²
  linear at the 0.0525 m wheel radius), chosen **deliberately above**
  Nav2's `acc_lim` (~2.5 m/s²) so that planner-shaped velocity profiles
  pass through unmodified; this bound exists to catch non-planner
  commanders and stale-held-target transients, not to re-implement Nav2's
  acceleration limiting.
- `set_safety_stop` gains an idempotence guard at its very top — a
  repeated call with the same `active` value returns immediately, before
  touching any other state — and, on the `active=True` transition, resets
  `self._applied_wheel_velocities` to zero so a safety stop always starts
  the next motion from rest rather than resuming mid-ramp.
- This lands via the teammate's own TDD tests (`tests/test_base_velocity_
  slew.py`, `tests/test_chassis_ballast.py`) rather than tests authored in
  this wave; those files are taken as the specification for Task 6.
- Explicitly **not** reworked: the wider command-protocol gaps this
  validation surfaced (e.g. no acknowledgment/backpressure between
  commanders and the backend) are documented as known limitations, not
  redesigned in this wave.

## 5. Design C — scenario system

- **Actor path interpolation.** `tinker_sim_core.actor_path` provides
  `path_length(path)` and `path_pose_at(path, distance) ->
  (x, y, yaw)`, operating on a polyline of `[x, y]` waypoints, clamped to
  the path's end, rejecting fewer than two waypoints or any non-finite
  coordinate.
- **Tightened event validation.** `ScenarioDefinition` loading now
  rejects an `actor_path_start` event whose `actor` does not name a
  declared actor, and rejects a named actor whose `path` is not a list of
  at least two finite `[x, y]` pairs — both fail closed at scenario-load
  time rather than at run time.
- **`actor_path_driver` node.** A new, one-shot ROS 2 node (mirroring
  `scenario_runner`'s one-shot contract, since launch gates key off that
  node's exit) drives every `actor_path_start` actor along its path via
  repeated `/set_entity_state` calls, at a default speed of 0.3 m/s and a
  default 10 Hz update rate, exiting 0 after every actor has completed one
  full traversal and 1 on a service/timeout failure.
- **New arena-native scenarios.** `find-and-approach-person-rcw2026` and
  `pick-deliver-place-rcw2026` (ids equal to their filename stems, both
  excluded from `QUALIFICATION_SCENARIO_NAMES`) declare
  `world: {"mode": "arena", "arena": "rcw2026"}` and use robot/actor/
  object poses verified against the real derived occupancy map for the
  committed `rcw2026` artifact — spawn and path points are checked for
  clearance against `OccupancyMap.free_with_clearance` before being
  authored into the scenario JSON, and that check is re-asserted by a
  committed test so the poses cannot silently drift from the map.

## 6. Design D — safety supervisor navigation mode

- `tinker_sim_core.safety_gating.effective_stop(desired_stop,
  management_ready, startup_hold, restore_pending, manage_controllers)` is
  a pure function computing the published `/sim/hardware/safety_stop`
  value. In managed mode (`manage_controllers=True`, the manipulation
  default) it preserves every existing controller-lifecycle latch exactly
  as before. In **unmanaged mode** (`manage_controllers=False`) it reduces
  to `bool(desired_stop)` — a sources-only, fail-closed clear that never
  depends on a `/controller_manager` that doesn't exist on the navigation
  profile; the fail-closed source-freshness contract itself still lives
  upstream in `SafetySourceTracker`, which continues to assert
  `desired_stop` on any stale source.
- `safety_supervisor` gains two parameters: `manage_controllers` (bool,
  default `True` — manipulation/qualification behavior is byte-for-byte
  unchanged) and `required_sources` (string array, default `["xarm",
  "collision"]`). In unmanaged mode, `_reconcile` returns immediately
  after publishing the effective stop, before any controller-manager
  client is touched.
- `navigation.launch.py` now runs `safety_supervisor` itself, parameterized
  with `manage_controllers: False` and `required_sources: ["collision"]`
  — replacing the manual CLI safety-clear heartbeat operators previously
  needed for navigation runs.

## 7. Out of scope

- Any change to the manipulation/qualification safety-supervisor defaults
  or controller-lifecycle gating behavior.
- Reworking the base-motion command protocol (acknowledgment, backpressure,
  or other gaps the validation wave documented but did not redesign).
- An EKF/IMU yaw-fusion fix for odometry yaw slip (investigated and
  rejected — see Findings).
- Any change to the `tk25_ws` branch or its `test_provenance` behavior.
- Seeded/procedural placement generation, GPSR/LLM scenario generation, or
  any arena/asset work beyond the two new rcw2026 scenario variants.
- Live/release qualification of any of the above; this wave's live runs
  are development-validated evidence only (Task 12), not a release gate.
