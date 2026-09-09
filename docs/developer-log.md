# Developer log

Dated engineering notes: what was measured, what was ruled out, why a fix
took the shape it did. Operational instructions live in
`docs/gpsr-sim-runbook.md`; this file is the history behind them.

## 2026-09-09 — /livox/imu was a stub: four defects in one small message

Follow-up to the raycast lidar below, kept as a separate change because it is
a separate concern.

**What was wrong.** `/livox/imu` looked well-formed and was wrong four ways:

1. `linear_acceleration` was never assigned, so it published `(0, 0, 0)`. That
   is not a noise-free ideal reading. An accelerometer measures SPECIFIC FORCE,
   `f = a - g`, so a stationary sensor reads ~`+9.81 m/s^2` along its
   gravity-opposing axis; zero means freefall. FAST-LIO uses precisely this
   vector to find "down" before it will initialise, which is why the tk26_sim
   reference bothers to finite-difference an acceleration.
2. The angular velocity came from `root_ang_vel_w` -- the WORLD frame -- while
   the message was stamped `livox360`. On a planar base yawing about z the two
   coincide, so it was accidentally correct for ordinary driving and wrong the
   moment the robot pitched or rolled. That is why it survived this long.
3. It was sampled at the articulation ROOT with no lever-arm term. A real IMU
   0.195 m above the rotation centre feels centripetal and tangential
   acceleration whenever the base turns.
4. `angular_velocity_covariance` and `linear_acceleration_covariance` were left
   all-zero, which by REP-145 means "unknown"; a consumer that trusts it reads
   zero variance, i.e. a perfect sensor. (`orientation_covariance[0] = -1.0`
   was already right and is kept -- the sim reports no orientation, as the real
   driver does not.)

**What it does now.** `simulation/tinker_sim_isaac/imu_model.py` holds the
physics as pure functions -- no Isaac, no ROS import -- so specific force,
frame rotation and the lever arm are unit-testable on their own.
`backend.imu_state()` reads the simulator; the gateway assembles the message.

The sample body is resolved by preference `livox_frame` then `base_link`,
following the existing `_base_link_body_index` fail-soft pattern. When the
import keeps `livox_frame` as a distinct body, PhysX reports ITS acceleration
with the centripetal and tangential terms already included, so the lever arm is
intrinsic and no manual `omega x (omega x r)` is applied; the explicit
lever-arm path exists only for the welded case, using the URDF's livox_joint
origin (0.09, 0, 0.195).

Acceleration comes from `body_com_acc_w` (PhysX `get_link_accelerations()`),
not from differencing velocity: differencing lags half a step and amplifies
per-step solver jitter. A backend without that view falls back to a backward
difference rather than publishing zeros, and the first sample of that fallback
reports no acceleration rather than inventing one.

**Deliberately NOT modelled: noise and bias.** A real ICM-40609 has both, but
the simulator's value is determinism and the reference sim publishes a clean
signal too. If FAST-LIO later needs realistic noise, it belongs in
`imu_model.py` behind a spec flag, not smeared through the gateway.

**Lesson repeated from the lidar change.** Several suites build the gateway
with `object.__new__` to exercise `publish()` without a live backend, so every
attribute the publish path reads must tolerate a skipped `__init__` -- hence
the `getattr` defaults, matching how `_services_ready` is already read. The
legacy `root_state` compatibility path likewise uses `.get` with defaults: a
minimal test double supplied only the one field the old stub happened to read,
and a missing field must degrade the sample, never raise inside `publish()`.

## 2026-09-09 — /livox/lidar becomes a real sensor: the PhysX raycast lidar

**What was wrong.** `/livox/lidar` was never a sensor. `ros_gateway.
_development_point_cloud` traced 181 rays across the arena occupancy PGM at
1 deg spacing, every point at `z = 0`, from a point hard-coded 0.12 m ahead of
`base_link` with the height ignored. It read the MAP, so it could not see a
spawned object or the person capsule -- it re-published what Nav2 already holds
as `static_layer`. `"rtx_lidar"` in `sensor-rich.json` was inert metadata;
`SimulationProfile.modules` is parsed and never read by any code.

**What replaced it.** `simulation/tinker_sim_isaac/lidar_rig.py`: an
`isaacsim.sensors.experimental.physics` `RaycastSensor` on the robot,
57 channels x 349 azimuth columns = 19,893 rays at 10 Hz = 198,930 points/s,
against the Mid-360's 200,000, over its -7..+52 deg by 360 deg field. The
extension declares no RTX dependency, so it runs in `navigation-parity`
(`render=false`, CPU physics) as well as `sensor-rich`. Post-`play()` spawns
are hit with no registration -- confirmed by a cube created mid-run moving a
reading from 9.5 m to 2.75 m.

**Three findings that shaped it, all measured (throwaway probes, headless).**

*`depths` is broken in Isaac Sim 6.0.1.* Six axis-aligned rays into geometry at
hand-checked distances returned `depths = [3.5] * 6` -- ray 0's value
replicated across every ray -- while the same reading's `hit_positions` were
all correct (`[[3.5,0,0], [-5.5,0,0], [0,7.5,0], [0,-9.5,0], [0,0,10.5],
[0,0,-1.0]]`). The rays are cast correctly; only the depth field lies. The rig
reads `hit_positions` and derives range as its norm. This cost two probe rounds:
uniform depths look exactly like a collapsed ray pattern, and it took reading
the authored `rayDirections` back off the prim (128 distinct rows, matching
input) to rule that out.

*`ray_time_offsets` is a firing schedule, not just pose extrapolation.* The USD
schema documents only "the world transform is extrapolated to currentTime +
offset". Incomplete: the plugin also defers each ray to its offset instant. The
shipped example's `_generate_rotating_rays` docstring says so; the schema does
not. Spreading a frame's offsets across `1/tick_rate` measured 4.47 vs
27.89 ms/step at full scale. This is the ONLY rate control the sensor has --
`sensorPeriod` exists only on the contact and IMU sensors and is deprecated,
and toggling the inherited `enabled` attribute at runtime does not gate casting
(an earlier run where it appeared to had `initialize_physics` throwing in its
log and was degraded).

*A frame must be ACCUMULATED.* Because rays fire on their own step, the reading
buffer holds only that step's rays and is cleared each step -- populated counts
stay flat at ~500 of 19,893 across the window rather than growing. The union
over one window recovers the pattern exactly. `FrameAccumulator` folds
`window_steps` readings (12 at 120 Hz physics / 10 Hz lidar) into one frame and
publishes when the window closes, so the cadence is phase-locked to the scan
rather than to `_tick`. A miss is a ZERO VECTOR in `hit_positions`, which is
also how an unfired ray reads -- both contribute no point, so one test covers
both.

**Cost, measured with the real robot on the stage.** Against a no-sensor
baseline in the SAME scene, because the robot's ~200 convex-decomposition
collision shapes make every scene query dearer and dominate the absolute
number (21.5 ms/step with no sensor at all): marginal sensor cost is +0.75 s of
compute per simulated second at full scale, +0.50 at 32x360, +0.25 at 16x360,
+0.22 at 8x360. Nearly flat below 16x360, so trimming past that buys little.
An earlier empty-room measurement suggested ~0.35 s/s and was not
representative; a mid-analysis reading of ~1.9 s/s was wrong in the other
direction, from comparing a robot-present run against an empty-room baseline.
`max_range` is not a lever (8 m vs 40 m differed under 4%), and neither is
`min_range` (0.2 -> 0.7 m moved cost 3%): the expense is testing rays against
the robot's shapes, not hitting them.

**Self-hits.** 9,840 of 19,893 points -- 49.5% of a frame -- land on the robot
itself, far more than a real Mid-360 loses to its own chassis, because the
raycast sees the inflated convex-hull COLLISION geometry rather than the visual
shape. Filtering therefore moves the sim toward hardware behaviour, not away.
`report_hit_prim_paths` identifies them and is effectively free at this scale
(31.95 vs 31.91 ms/step, inside noise); only the ~3,300 live rays per step are
path-tested, since checking all 19,893 in Python every step would cost more
than the raycast. After filtering, frames carry ~11,000 points and the nearest
return moves from 0.20 m (the robot's own shell) to ~0.46 m.

**Mount and TF.** The sensor is created under the URDF's `livox_frame`, walking
up to the nearest `RigidBodyAPI` ancestor because the URDF importer welds
fixed-joint links onto their parent and the sensor's world transform comes from
a rigid body's pose (the same trap `camera_rig` documents for optical frames).
On the shipped artifact `livox_frame` carries the API itself, so the offset is
zero. The sim bridge's `base_link -> livox360` static TF moved from
(0.12, 0, 0.25) to the URDF's (0.09, 0, 0.195): it now matches both where the
sensor actually is and the height `arena_map.livox_scan_height()` slices the
AMCL map at. The old value predated any real sensor and agreed with neither.
The hardware launch files are untouched.

**pointcloud_to_laserscan now takes the hardware values verbatim** (min_height
0.0, max_height 2.0, angle -1.44..1.436, range 0.2..8.0). The previous
+/-180 deg, +/-0.05 m band existed only because the cloud was a planar ring.
Tabletops becoming scan hits and the robot being blind behind itself are the
PARITY TARGET, not regressions: they shape nav_back and spin recoveries on
hardware.

**A bug worth remembering.** The frame accumulator initially produced nothing
at all while looking healthy. Root cause: `SimulationManager`'s dispatcher
calls physics-step callbacks with `(step_dt, context)`, and the handler
declared one optional argument, so every invocation raised `TypeError` inside
the message bus, which swallowed it. The signature must match the sensor
extension's own `_SensorStepManager._on_physics_step(step_dt, context=None)`.
The `except Exception: return` in the read path hid it further; that path now
reports the first failure of each kind once.

**Not done here.** The synthetic `/livox/imu` is untouched and is NOT accurate:
`linear_acceleration` is never assigned (so it reads (0,0,0) -- permanent
freefall rather than ~9.81 m/s2 at rest), the angular velocity is world-frame
but stamped `livox360`, it is sampled at the articulation root so there is no
lever-arm term, and the angular/linear covariances are left all-zero. Nothing
in the sim stack consumes it today beyond a contract_guard existence check, but
it is a hard prerequisite for FAST-LIO in sim. AMCL parity evidence is pinned
bit-identical to the map raycast and has to be re-earned against this source,
with the spawn-mislocation flag off.

## 2026-09-08 — Task #33: the gripper's own effort-limit write silently reverted drive_joint's PhysX gains

**Finding.** On the grasp bench, `drive_joint` was not running the gains the
sim configures. Isaac Lab builds it from
`ImplicitActuatorCfg("gripper", stiffness=200, damping=20)`, and that is what
PhysX held — right up until the stack sent its first `GripperCommand`. From
that moment on the joint ran at stiffness 35809.86 with damping 0.0, for the
rest of the run. Every #33 clamp measurement taken after the first gripper
packet was therefore taken on a ~180x-stiff, completely undamped drive joint.

**Measurement 1 — bench round ahi (per-tick PhysX readback).** The instrument
built for this round reads `stiffness`/`damping`/`max_force` straight off the
PhysX articulation view every control tick, rather than off the Isaac Lab
buffers. It shows 200/20 on `drive_joint` for the whole pre-command window;
at the exact sample where `_set_gripper_effort_limit` performs its first
write (`max_force` 2.5 -> 1.25) the pair becomes 35809.86 / 0.0 and never
returns. The mimic followers (`left_finger_joint`,
`right_outer_knuckle_joint`) stay at 1500/55 throughout — only `drive_joint`
moves.

Where 35809.86/0 comes from: the asset. `artifacts/robot/tinker2/347aef…/
robot.usd` authors a `PhysicsDriveAPI:angular` on
`/tinker_full/joints/drive_joint` with stiffness 625.0 in USD *degree* units
(625 * 180/pi = 35809.86 in PhysX radian units), damping 0.0, maxForce 50 and
maxJointVelocity 114.59 deg/s (= 2.0 rad/s). The follower joints carry no
DriveAPI at all in the asset, which is exactly why they were untouched — and
the tell that the reverted values were being re-read from the stage rather
than computed by anything at runtime.

**Measurement 2 — headless one-variable control.** Two headless legs,
identical but for one line. Leg A calls `backend._set_gripper_effort_limit(5.0)`
as shipped: the PhysX gains flip to 35809.86/0 *inside the call*, before any
physics step is taken. Leg B replaces only
`backend._author_gripper_drive_usd_max_force` with a no-op: the gains stay
200/20 across 726 sampled rows, and `get_dof_max_forces` still reads 1.25 and
then 2.5 — so the direct tensor-view write (`set_dof_max_forces`) alone is
sufficient to make the cap bind, and the USD authoring buys nothing.

**Mechanism.** `_author_gripper_drive_usd_max_force` applied a
`UsdPhysics.DriveAPI` (both the "angular" and "linear" instances) on
`drive_joint`'s live prim and authored `physics:maxForce` on it. omni.physx
keeps a USD change listener on the stage; that edit makes it re-create the
joint's drive *from the stage*, which discards the runtime tensor-view gains
Isaac Lab had written and reinstates the asset's authored drive. Introduced
by 67cd278 on the PR #20 branch; neither `dev` nor `main` ever carried it.

**Why it hid for so long.** Isaac Lab's `data.joint_stiffness` and
`data.joint_damping` read 200/20 in *both* legs of the control — the Lab-side
buffers are never re-synced after omni.physx rebuilds the drive. Any check
written against the Isaac Lab API, which is the natural place to look, would
have reported the joint as correctly configured. Only a readback off the
PhysX view could see it, which is why this needed the round-ahi instrument
rather than another round of hypotheses.

**Fix.** Delete the runtime USD authoring at the source: the helper and its
call in `_write_gripper_drive_physx_max_force` are gone, and that method's
docstring plus the `#20 cap5-analysis` comment block now record the measured
facts — the tensor-view write is what binds the cap, USD authoring re-syncs
the drive from the stage and reverts the gains — instead of asserting the USD
write was required. The direct warp write, the Isaac Lab writer, the
actuator-model mirror, the readback and the dedup/latch logic are unchanged,
so the #20 effort-cap behaviour is otherwise intact.

Regression guard: `tests/test_manipulation_runtime.py
::test_gripper_effort_limit_never_authors_usd_drive` injects recording
`omni.usd`/`pxr` doubles into `sys.modules` — neither is importable in the
unit-test venv, so the old helper failed closed and no unit test ever
exercised it — and asserts zero `DriveAPI` applications and zero
`physics:maxForce` authoring while `set_dof_max_forces` still lands the
mapped cap.

**Caveat on prior numbers.** Any `validation/gripper_close_probe.py` run that
passed `--drive-effort-limit` before this fix went through the reverting path.
How much that invalidates depends on what the run did next, because the
effort-limit block runs before any later gain write. For the default
`--mirror-mode target` with 3-part configs, nothing rewrites `drive_joint`'s
gains afterwards, so those clamp figures describe a 35809.86/0 drive joint and
are not comparable with flag-off runs. For `--mirror-mode central`, or a 5-part
config whose `run_close` calls
`set_follower_gains(..., drive_stiffness, drive_damping)`, `drive_joint`'s k/d
were rewritten after the reversion — whether that write actually binds in PhysX
was never measured, so those runs are unknown rather than either clean or
invalid. Nothing was re-run for this change; it is code-only.

## 2026-09-07 — Task #41: pan_tilt facade keepalive timer thrashed stale_hold at low RTF

**Symptom.** A bench round logged 2835 `stale_hold`/`stale_hold_cleared`
pairs (~1.18 Hz cycle, ~0.85s period) from the mux's pan_tilt
`CommandSource` -- harmless that round because the mux's stale-hold
substitute happened to match what the facade was already sending (head
idle), but the churn is a symptom of a real mechanism: a real
`/pan_tilt_controller/cmd` sweep in progress could get clamped mid-motion
to a stale measured position.

**Root cause.** `pan_tilt_facade.py`'s `_hold_target` republish timer
(`self.create_timer(0.2, self._hold_target)`) had no explicit `clock=`
argument, so it ran on the node's default clock -- `ROS_TIME` under the
bridge's `use_sim_time=True` launch, i.e. paced in SIM seconds. The
consumer, `CommandGateway`'s pan_tilt `CommandSource` (0.5s timeout,
`command_gateway.py:99-101`), judges staleness in the mux against
`time.monotonic()` -- WALL seconds (same clock the gateway's own 150 Hz
publish timer and `_enforce_safety_deadline` are explicitly pinned to via
`Clock(clock_type=ClockType.STEADY_TIME)`, `command_gateway.py:135-139`).
At RTF < 0.4 a 0.2 sim-s republish costs more than 0.5 wall-s to land, so
every tick arrived after the deadline -- continuous churn. This is the same
family as Task #27 (facade dwell on the wall clock while the node runs sim
time) but the inverse direction: there the *deadline* logic ran on the
wrong clock, here the *keepalive* logic does.

**Fix.** `pan_tilt_facade.py`'s hold timer now passes
`clock=Clock(clock_type=ClockType.STEADY_TIME)`, mirroring
`command_gateway.py`'s own 150 Hz timer and `gripper_facade.py`'s 20 Hz
keepalive (`gripper_facade.py` was already correct here, confirmed by
reading it, not assumed). Header timestamps inside `_hold_target` still use
`self.get_clock().now()`, which stays sim time -- only the timer's own tick
cadence changed. Swept the rest of the bridge for the identical defect
(a keepalive/republish timer on a `use_sim_time` node feeding a
wall-clock-gated `CommandSource`): `base_facade.py` and `xarm_facade.py`
were already `STEADY_TIME`; `command_gateway.py`'s own 150 Hz tick and
`gripper_facade.py`'s two timers were already `STEADY_TIME`. `pan_tilt`
was the only one still on the node's default clock.

**Test.** `tests/test_pan_tilt_facade_keepalive.py` (new):
`test_hold_timer_created_with_steady_clock` patches `Node.create_timer`
with a recording double and asserts the `_hold_target` timer's `clock=`
kwarg has `clock_type == ClockType.STEADY_TIME`;
`test_hold_republish_cadence_is_wall_clock_bounded_at_low_rtf` drives a
synthetic `/clock` feed at RTF ~0.25 (`use_sim_time=True`) and asserts the
wall-clock gap between consecutive `/sim/controller/pan_tilt_commands`
publishes stays <= 0.35s. Pre-fix, both fail: the first with
`AssertionError: _hold_target timer must pass an explicit clock=`, the
second with only 3 republishes in 3 wall-s (~0.85s gaps, matching the
bench's 2835-pair measurement); post-fix, both pass with ~15 republishes in
3 wall-s. Full targeted run (python3.10, ROS-sourced,
`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`): `test_pan_tilt_facade_keepalive.py`,
`test_head_tf.py`, `test_head_initial_pose.py`,
`test_command_gateway_keepalive.py`, `test_gateway_simtime_deadlines.py`,
`test_xarm_safety_heartbeat.py`, `test_gripper_executor_humble.py` together:
47 passed. `tests/test_manipulation_runtime.py` (uv/lark-shim incantation):
132 passed, 3 subtests passed, unaffected.


## 2026-09-07 — Task #39: `/spawn_entity` advertised ~100s before it can be served

**Symptom.** A `/spawn_entity` (or sibling `simulation_interfaces`) call made
right after `wait_for_service()` returned true could still fail with
rclpy/`rmw_fastrtps_shared_cpp`'s "failed to send response (timeout): client
will not receive response" -- a middleware-level failure, not the caller's
own timeout budget expiring. In one dataset (`01-sim.log`) the extension
came up at wall 18.9s, then a 93s gap with nothing else logged, then the
failed-response warning at wall 117.7s; `scenario_runner`'s own six
boot-time spawns all round-tripped cleanly before that gap, ruling them out
as the failing call and pointing at a later, separate client (the GPSR
overlay's `tools/gpsr_spawn.py`, own budget `SPAWN_TIMEOUT_S = 120`).

**Root cause.** `validation/run_sim.py` called
`enable_extension("isaacsim.ros2.sim_control")` immediately after
`SimulationApp` construction, before the sensor profile branches ran at
all -- i.e. before the backend, camera rig, or `RosStandardGateway` existed.
`enable_extension` registers the extension's ROS services (`/spawn_entity`,
`/set_entity_state`, `/delete_entity`, `/set_simulation_state`,
`/load_world`, `/reset_simulation`) synchronously, but Kit only *serves*
them from its own asyncio loop, which nothing pumps regularly until each
profile's main loop starts. Backend construction
(`IsaacWholeRobotBackend.__init__`) has no `app.update()` call at all, and
camera warm-up (`CameraRig.initialize`, sensor-rich only) has only a
handful; together they can run for a couple of minutes on a cold shader
cache. A request that reaches the extension in that window queues past both
the caller's own timeout budget and, at the DDS layer, the point at which a
response can even be correlated back to it. `wait_for_service()` -- the only
readiness signal any client checked -- returns true the instant the
extension is enabled, long before any of this warm-up is done, so it
actively misleads a client into thinking a prompt response is possible.

**Fix.** `_enable_sim_control_services(app, gateway=None)` (`run_sim.py`)
now does the enable + one `app.update()`, and is called once per sensor
profile branch, at the same point the boot-config JSON prints today -- i.e.
after backend construction, camera-rig warm-up (sensor-rich), and gateway
construction are all behind it, right before that branch's main loop starts.
The pre-enable `_install_set_entity_state_physics` monkeypatch (Task #12/#20)
stays at the old early call site: it only patches a class method and does
not itself need the extension enabled or any long-running object to exist
yet, and must run before `enable_extension`'s `on_startup` captures the
unpatched bound method.

`RosStandardGateway` (`ros_gateway.py`) gained `services_ready` /
`services_ready_since` on `/sim/status/isaac` (a JSON string message):
`False`/`None` at gateway construction, flipped by the new
`mark_services_ready()` method (idempotent, timestamped with
`backend.simulation_time`) at the same call site as the extension enable.

`tools/gpsr_spawn.py`'s `_make_ros_service_client` now calls
`_wait_for_services_ready` (its own bounded wait, default
`SERVICES_READY_TIMEOUT_S = 300s`, logged) before the existing
`wait_for_service` 10s checks on `/spawn_entity`/`/delete_entity`, which
remain as a secondary check. A not-ready sim now fails fast and
diagnosably ("`/sim/status/isaac` never reported services_ready after
300s") instead of racing `SPAWN_TIMEOUT_S` and surfacing as an opaque DDS
timeout.

**How a client should gate a first `/spawn_entity`-family call going
forward:** wait for `/sim/status/isaac`'s `services_ready` to be `true`
first; `wait_for_service()` alone is necessary but not sufficient -- it only
proves the extension is enabled, not that Kit is being pumped regularly
enough to serve a request promptly.

### Review round (2026-09-07): test-double regression, scenario_runner gap, absent-status fallback

Three findings from review against the fix above, all addressed:

1. **`RosStandardGateway.publish()` read `self._services_ready` directly**,
   which broke `tests/test_manipulation_runtime.py`'s
   `test_finger_contact_wrench_publishes_every_tick_not_at_status_cadence`
   -- its gateway double is built via `object.__new__(RosStandardGateway)`
   (skips `__init__`, so `_services_ready`/`_services_ready_since` never
   get set) and exercises `publish()`'s status block. Fixed with
   `getattr(self, "_services_ready", False)` (and the `_since` sibling) so
   any double built this way degrades to "not ready" instead of raising;
   also added the two attributes directly to that test's double for
   belt-and-suspenders. `tests/test_manipulation_runtime.py`: 129 passed, 0
   failed (was 128 passed, 1 failed).

2. **`ros2_ws/src/tinker_sim_bridge/tinker_sim_bridge/scenario_runner.py`
   had no `services_ready` gate at all.** Its `ScenarioRunner.call()` bounds
   both the `wait_for_service` discovery and the response wait to a single
   `--timeout` (default 20.0s; `gpsr.launch.py` never raises it), with no
   retry beyond the very first `/reset_simulation`. Moving the sim's
   `_enable_sim_control_services` call point later (this same task) makes
   the ~100s warm-up gap it exists to describe worse for this client, not
   better. Added a duplicate of `tools/gpsr_spawn.py`'s
   `_wait_for_services_ready` (same fallback behavior as finding 3 below)
   directly in `scenario_runner.py` -- not imported, since
   `tinker_sim_bridge` is a separate `ament_python` package built by
   colcon and the top-level `tools/` tree is not installed anywhere colcon
   looks; a cross-package import would tie the bridge's build to a tree
   outside it. Wired into `main()` right after `ScenarioRunner`
   construction, ahead of `node.execute(operations)`. `tests/
   test_scenario_runner.py`: 14 passed (10 pre-existing + 4 new), run under
   `python3.10` (this file `pytest.importorskip`s real `rclpy`, whose
   Humble-built C extension does not load under the repo's Python 3.12 uv
   venv).

3. **`tools/gpsr_spawn.py`'s `_wait_for_services_ready` had no fallback for
   an older sim / absent `/sim/status/isaac`.** If the topic never
   published at all, or published without a `services_ready` key (a
   pre-#39 sim binary), the function spun for the full
   `SERVICES_READY_TIMEOUT_S` (300s) and then raised, turning what used to
   be a ~20-30s `wait_for_service`-only success into a 300s failure.
   Added `SERVICES_READY_GRACE_S = 10.0`: the function now waits only that
   long for *any* status sample; if none arrives, or the first one that
   does has no `services_ready` key, it logs a warning and returns
   immediately, leaving the caller's own `wait_for_service` checks (the
   pre-#39 path) to run unmodified. Once a sample carrying the key is seen
   -- `services_ready: false` included -- it commits to the full bounded
   wait exactly as before. Same fallback duplicated into
   `scenario_runner.py`'s copy (finding 2). `tests/test_gpsr_spawn_cli.py`:
   34 passed (31 pre-existing + 3 new, including the false-then-true
   commit-after-grace case).

**Tests.** `tests/test_run_sim_services_readiness.py` (new): unit tests on
`_enable_sim_control_services` itself (fakes for `enable_extension`/the
app/gateway, asserting the enable -> `app.update` -> `mark_services_ready`
order, and that a missing gateway is tolerated); source-order regression
tests per sensor-profile branch of `main()` (`inspect.getsource`, same
pattern as the existing structural tests in
`test_manipulation_gate_executor.py`) asserting backend construction, (on
sensor-rich) camera-rig warm-up, and gateway construction all precede the
`_enable_sim_control_services(...)` call in that branch's source, plus a
regression that the old early call site carries no `enable_extension(`
call. `tests/test_ros_gateway.py`: `/sim/status/isaac` carries
`services_ready: false`/`services_ready_since: null` before
`mark_services_ready()`, `true`/the recorded `backend.simulation_time`
after, and a second call does not move the timestamp.
`tests/test_gpsr_spawn_cli.py`: `_wait_for_services_ready` spins through
false samples and returns once a true sample arrives, cleans up its
subscription either way, and raises `ServiceUnavailable` once its own
bounded timeout elapses without ever seeing `services_ready: true` (fake
`/sim/status/isaac` node + injectable clock, no rclpy needed). Full
targeted run:
`tests/test_run_sim_services_readiness.py tests/test_ros_gateway.py
tests/test_gpsr_spawn_cli.py tests/test_set_entity_state_physics.py
tests/test_run_sim_arena_cli.py tests/test_run_sim_arena_wiring.py` --
83 passed. No GPU boot for this change (deferred: live confirmation that
the moved call site actually closes the wall-clock gap end to end).

### Second review round (2026-09-07): `spin_once`'s timeout is keyword-only

**Finding.** Both `_wait_for_services_ready` implementations
(`tools/gpsr_spawn.py`, `ros2_ws/src/tinker_sim_bridge/tinker_sim_bridge/
scenario_runner.py`) called `spin_once(node, 0.5)` -- a positional
`timeout_sec`. Real `rclpy.spin_once`'s signature is `spin_once(node, *,
executor=None, timeout_sec=None)`: `timeout_sec` is keyword-only. Against
real rclpy this raises `TypeError: spin_once() takes 1 positional argument
but 2 were given` on the very first loop iteration -- including the
"already ready" fast path this whole task exists to preserve. Every unit
test passed anyway because each test double's fake `spin_once(n,
timeout_sec)` accepts the positional call fine; the mismatch only fires
against the real function, which no test in the suite exercised for this
call site.

**Fix.** Both call sites now call `spin_once(node, timeout_sec=0.5)`.
Grepped the whole branch for other `rclpy.spin_once`/
`spin_until_future_complete` calls: every other call site in the repo
already uses the keyword form (`tools/gpsr_spawn.py`'s own
`spin_until_future_complete` calls, `scenario_runner.py`'s
`ScenarioRunner.call()`, `controller_reconciler.py`,
`ompl_plan_smoke.py`, etc.) -- these two were the only positional
holdouts. Also checked the two helpers' other injected-rclpy-shaped calls
(`node.create_subscription(msg_type, topic, callback, qos_profile)`,
`node.destroy_subscription(subscription)`) against real
`rclpy.node.Node`'s signatures (`inspect.signature` under `python3.10`)
-- both match; no `*` before the arguments these helpers pass.

**Test-double hardening.** Every fake `spin_once` in
`tests/test_gpsr_spawn_cli.py` and `tests/test_scenario_runner.py` is now
declared `def spin_once(n, *, timeout_sec=None)` (was `def spin_once(n,
timeout_sec)`), so a future regression back to the positional call form
raises `TypeError` inside the test itself instead of silently passing.
Verified directly: reverting either production call site back to
`spin_once(node, 0.5)` against the now-keyword-only doubles fails 6/34
(`test_gpsr_spawn_cli.py`) and 4/14 (`test_scenario_runner.py`) with
exactly that `TypeError`; restoring the keyword-argument call makes both
files pass again (34 passed; 14 passed).

`tests/test_scenario_runner.py` needs real `rclpy`/`simulation_interfaces`
(`pytest.importorskip`s them) and does not load under the repo's Python
3.12 uv venv; run under `python3.10` with `PYTHONPATH` carrying
`.ros-vendor/humble/opt/ros/humble/local/lib/python3.10/dist-packages`
(carries `simulation_interfaces`) plus the `ros2_ws/src/tinker_sim_bridge`
and `simulation` source trees.

## 2026-09-07 — Task #33: observability for the stale-hold / gripper-target / safety-gate chain

**Purpose.** Task #33's stale-hold repro (`$TMP/task33-stale-hold-repro.md`)
traces a lapsed 0.5 s gripper-command watchdog through `command_mux.py` (holds
the last measured `drive_joint` angle), into `backend.py`'s
`_apply_joint_command` (overwrites `_drive_command_target` from whatever
packet next names `drive_joint`), but none of it was visible on a live bench
log. This round adds observability only -- no packet, target, or publish
value this chain produces changes; every new branch either logs alongside an
existing decision or reads a pure function a second time purely to report an
edge.

**`simulation/tinker_sim_core/command_mux.py`** (plain library, no ROS
handle -- logs through `logging.getLogger(__name__)`):
- `stale_hold source=<src> joints=<names> hold_pos=<vals> age_s=<age>
  clock=steady` once when a source's composed packet first freezes at its
  watchdog transition (`_compose_packets`, the existing
  `_stale_position_holds.setdefault` site).
- `stale_hold_active source=<src> ...` (same fields) at most once per second
  while the hold continues -- a source held for minutes does not spam one
  line per composed frame (up to 150 Hz at the gateway).
- `stale_hold_cleared source=<src> stale_for_s=<duration>` when a fresh
  `accept()` replaces an active hold.
- `mux_stop state=engaged|released held=<joint>:<pos> ...` once per
  `stop(True)`/`stop(False)` transition (not per call -- the existing
  `if active and not self.safety_stop` / `elif not active and
  self.safety_stop` guards already gate real transitions only), listing the
  measured positions force-held by `_stopped_packets`.

**`simulation/tinker_sim_isaac/backend.py`** (module already logs via bare
`print(json.dumps(...))`/plain `print()`, no `logging` import -- this stays
consistent with that, not a JSON line, per the exact format requested):
`_apply_joint_command` now logs `gripper_command_target old=<old> new=<new>
effort=<effort> source_packet=<n> applied=<_position_targets[0,drive]>`
whenever the incoming packet actually changes `_drive_command_target`
(`old != new`; the very first command logs `old=None`). `applied` is the
ramp's current output at that instant -- the value `_ramp_drive_target()`
last wrote, i.e. what the facade would see this tick, not the new commanded
target, which only takes effect once the ramp catches up. `source_packet` is
a monotonic call-sequence number (`_apply_joint_command` carries no other
packet identity today). Rate-limited to
`GRIPPER_COMMAND_TARGET_LOG_MAX_PER_WINDOW` (5) lines per
`GRIPPER_COMMAND_TARGET_LOG_WINDOW_S` (1.0 s) via a sliding timestamp window,
so a fast oscillation (e.g. a source flapping stale/fresh) cannot spam the
log -- but the window ages out on its own, so the first line after any quiet
period always gets through.

**`ros2_ws/.../command_gateway.py`**: the mux's `logging` records never
reached the bridge log on their own -- `rclpy`'s `get_logger()` is a separate
sink from Python's `logging` module. `CommandGateway.__init__` now attaches a
`_RclpyLogForwarder` (a `logging.Handler` that calls
`self.get_logger().info/warning/error`) to the
`tinker_sim_core.command_mux` logger and sets `propagate = False` (keep it
out of an unrouted root-logger stdout copy). Also new in this file, same
principle -- log alongside an existing decision, never change it:
- `safety_gate armed reason=timeout gap_s=<since last sample>` /
  `safety_gate armed reason=sample` / `safety_gate cleared reason=sample
  gap_s=<duration held>` on every real `_safety_active` transition
  (`_enforce_safety_deadline`'s timeout arm, `_safety_stop`'s explicit
  sample arm/clear). `gap_s` means different things by design: for an
  armed-by-timeout line it is the heartbeat gap that caused it; for a
  cleared line it is how long the gate was held.
- `command_rejected source=<src> reason=<reason> count=<n since last line>`
  from `_accept`'s three existing `self._rejected[source] = ...` branches
  (safety-active-at-entry, safety-active-after-deadline-check, and the
  except-handler), rate-limited to at most one line per second per source;
  drops inside the window are still counted and folded into the next line's
  `count` once the window reopens, so nothing is silently lost, only
  batched.

**`ros2_ws/.../safety_supervisor.py`**: `_refresh_desired_stop` gained
`_log_source_transitions()`, which reads each tracker's `requires_stop()` a
second time (pure, no side effect) purely to detect a per-source edge the
existing `any(...)` OR cannot itself report, logging `safety_source
source=<name> state=expired|recovered age_s=<age> deadline_s=<deadline>`.
`_publish()` logs `safety_stop_published value=<bool>
reason=<comma-joined source names, or "none">` whenever the published value
actually changes (compared against `_published_stop` before it is
overwritten) -- the 0.25 s reconcile heartbeat republishes the *unchanged*
value far more often than it flips, so this is on-change, not on-publish.
`reason=none` on an active stop is itself informative: it means the trigger
was a controller-management hold (`startup_hold`/`restore_pending`/not
`management_ready`), not any of `xarm`/`collision`/`operator`.

**How to read a plateau against these lines.** A bench force/position
plateau with no `stale_hold` line in the same window did not stall because a
command source went stale -- look at `gripper_command_target` instead: if
`new` keeps changing while the pads have stopped moving, the *ramp* is the
bottleneck, not the mux. Conversely a `stale_hold` immediately followed by a
`gripper_command_target` restating the *same* frozen value confirms the mux
hold is what is driving `_drive_command_target`, not a live command. A
`safety_gate armed reason=timeout` immediately upstream of a run of
`command_rejected ... blocked by safety stop` lines confirms a lost
heartbeat, not a rejected-for-cause command, caused the gap.

**Tests.** `tests/test_command_mux.py`: two new `unittest.TestCase`s
(`JointCommandMuxStaleHoldLoggingTest`, `JointCommandMuxStopLoggingTest`)
using `self.assertLogs("tinker_sim_core.command_mux", level="INFO")` against
a fake steady clock passed straight to `accept()`/`compose()` -- no real
sleep. `tests/test_manipulation_runtime.py`: two new tests on
`_apply_joint_command` capturing `stdout` (`contextlib.redirect_stdout`,
following the module's existing print-capture pattern) confirming the
exact old/new/effort/applied fields and the 5-lines/s rate limit.
`tests/test_command_gateway_logging.py` (new) and
`tests/test_safety_supervisor_logging.py` (new): `object.__new__` test
doubles in the style of `test_command_gateway_keepalive.py`, with
`unittest.mock.patch` pinning `time.monotonic` to a dict-backed fake clock
so `gap_s`/`age_s` assertions are exact rather than wall-clock-flaky.

These two files import `rclpy`/`std_msgs` and are skipped in this sandbox
(`ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'` -- the
system ROS install is built for Python 3.10, this worktree's `.venv` is
3.12); confirmed pre-existing (`test_command_gateway_keepalive.py` skips
identically here) and not a regression. They were instead run and passed
under the system `python3.10` (which has a matching `rclpy`) with
`PYTHONPATH` extended to `ros2_ws/src/tinker_sim_bridge` and `simulation`:
10 passed (`test_command_gateway_logging.py` + `test_command_gateway_keepalive.py`),
12 passed (`test_safety_supervisor_logging.py` +
`test_command_gateway_logging.py` + `test_command_gateway_keepalive.py`).
`tests/test_manipulation_runtime.py` + `tests/test_command_mux.py` under the
regular `uv run` (3.12) incantation: 178 passed, 5 subtests passed.

## 2026-09-06 — Task #35: backend-only TCP/pad parity publisher

**Purpose.** A diagnostic for the grasp bench: the left pad's inner face was
observed meeting a can 29-30 mm from TF `link_tcp` at drive 0, while the USD
puts the pad faces at +-44.5 mm about the sim's `link_tcp` prim -- all URDFs
agree statically, so if the discrepancy is real it is runtime-only. To let a
bench diff the sim's PHYSICAL tool-centre-point and pad faces against the ROS
TF `link_tcp` in one recording, `ros_gateway.py` now publishes, every
`publish()` tick (same unconditional cadence as `/sim/internal/physics_truth`
and the #22 `/sim/parity/finger_contact` wrench), gated by
`TINKER_SIM_PARITY_TCP` (default `"1"`, `"0"` disables):

- `/sim/parity/tcp_pose` (`geometry_msgs/PoseStamped`, frame `world`): the
  articulation's `link_tcp` body world pose.
- `/sim/parity/tcp_pose_base` (`PoseStamped`, frame `base_link`): the same
  pose expressed in the robot root frame.
- `/sim/parity/pad_points` (`geometry_msgs/PolygonStamped`, frame
  `base_link`): `[left inner-face centre, right inner-face centre,
  midpoint]`.

**Read path.** `backend.py` gains `IsaacWholeRobotBackend.parity_tcp_frame()`,
which resolves `link_tcp`/`left_finger`/`right_finger` in
`data.body_names` and reads their world pose from the same
`body_pos_w`/`body_quat_w` tensors `_robot_truth_state()` already uses for
`tcp_pose` (view-free through the articulation data -- no separate PhysX
query; `IPhysx.get_rigidbody_transformation` is for non-articulated rigid
bodies, e.g. spawned objects via `_iter_spawned_bodies`, not articulation
links). Fails soft: if any of the three bodies is missing, it logs
`{"event": "parity_tcp_bodies_unresolved", "missing": [...]}` once (a
`_parity_tcp_bodies_missing_logged` latch, mirroring
`_contact_report_first_event_logged`) and returns `None`; the gateway then
skips publishing that tick without raising.

**Pad-inner-face constants and their provenance.** A live `pxr` probe of the
shipped robot USD (`artifacts/robot/tinker2/*/robot.usd`, re-verifying
`$TMP/task31-jaw-opening-findings.md`) gave, via
`UsdGeom.BBoxCache.ComputeLocalBound` on each finger's own `collisions` prim
(i.e. relative to that finger LINK's own origin, before its world transform):

```
              X (width)        Y (closing axis)      Z (reach axis)
left_finger   -16.0/+16.0 mm   -26.0/+5.9 mm         -5.9/+61.0 mm
right_finger  -16.0/+16.0 mm   -5.9/+26.0 mm         -5.9/+61.0 mm
```

Both finger links share one static rest orientation (confirmed via
`ComputeLocalToWorldTransform` on both prims: a ~180 deg rotation about
local X, matching that both `left_finger_joint`/`right_finger_joint` are
revolute about local X) -- the two collision meshes are mirror images of
one another, not the link frames. Each pad's INNER face (the surface that
meets a grasped object) is therefore the extreme 26.0 mm from the link
origin: local Y = -0.026 on the left finger, +0.026 on the right, both at
the reach-axis midpoint `PAD_MID_REACH_M = (-0.0059 + 0.0610) / 2 ~=
0.02755`. `backend.py` module-level constants `PAD_INNER_INSET_M`,
`PAD_MID_REACH_M`, `LEFT_FINGER_PAD_LOCAL_OFFSET`,
`RIGHT_FINGER_PAD_LOCAL_OFFSET` carry this exact derivation in a comment.
`finger_inner_face_world()` rotates the fixed link-local offset by that
link's CURRENT `body_quat_w` before adding it to the link's world position
-- correct through the whole open/close range because it is a point
painted on the rigid pad, not a world-frame constant; a live pxr check of
the rest pose confirmed the mirrored +-0.0445 m inner-face separation this
produces (from finger origins at +-0.0705 m) against the shipped USD.
`pose_in_frame()` (quaternion conjugate + Hamilton product, both
scalar-last like every other quaternion in this module) expresses a world
pose in an arbitrary frame's own frame, used both for `tcp_pose_base` and
for expressing the pad points in `base_link`.

**Tests** (`tests/test_manipulation_runtime.py`): pure pad-inner-face/
midpoint math against the rest orientation and under an added yaw (verifies
the mirrored separation "rotates with" the closing axis rather than staying
pinned to world Y); `pose_in_frame` against a known yawed root pose;
`IsaacWholeRobotBackend.parity_tcp_frame()` end to end (world + base_link
poses, pad points) and its fail-soft/log-once contract when
`left_finger`/`right_finger` are absent from `body_names`; the gateway's
publisher registration/topic/frame_id/env-gate strings; and a `publish()`
runtime test (mirroring the #22 finger-contact-wrench test) asserting all
three topics fire every tick and that `TINKER_SIM_PARITY_TCP=0` (or an
unresolved frame) fully skips publishing without raising. Full suite:
`tests/test_manipulation_runtime.py` 138 passed, 5 subtests passed, 0
failed. No GPU boot for this diagnostic-only change; a bench recording
against the live TF `link_tcp` is the follow-up that actually answers the
29-30 mm question this publisher exists for.

### Task #36 addendum — wrist colour camera optical-frame parity publisher

**Purpose.** The grasp bench sees a constant ~15 mm perception bias along
base X on two different objects and suspects the wrist camera extrinsic.
Same idea as #35's TCP/pad topics, same env gate (`TINKER_SIM_PARITY_TCP`),
same unconditional every-`publish()`-tick cadence: publish the sim's
ACTUAL rendered wrist colour camera pose so the bench can diff it against
the ROS TF frames `xarm_camera_color_optical_frame`/`_aimed` in one
recording.

- `/sim/parity/wrist_camera_pose` (`PoseStamped`, frame `world`): the wrist
  colour camera's ROS OPTICAL frame (x right, y down, z forward) pose in
  world.
- `/sim/parity/wrist_camera_pose_base` (`PoseStamped`, frame `base_link`):
  the same pose expressed in the robot root frame, via `backend.py`'s
  existing `pose_in_frame()` (reused as-is, no new backend method).

**Read path and convention.** `camera_rig.py` gains
`CameraRig.camera_optical_pose_world(name)`: it looks up the SAME
`rtx_camera` prim path `initialize()` created for that spec (cached in
`self._camera_prim_paths`, set at `camera_path = f"{mount_path}/rtx_camera"`
-- no separate mount-prim search or re-derivation of the un-corrected mount
frame), reads its live `UsdGeom.Xformable(prim).ComputeLocalToWorldTransform`
(the exact transform the renderer itself uses, at whatever pose the arm's
forward kinematics and any `TINKER_SIM_WRIST_CAMERA_AIM` preset put it at
this tick), and converts native USD camera convention (looks down -Z, +Y
up) to ROS optical (+Z forward, +Y down) via the new pure function
`usd_camera_pose_to_ros_optical()`. That conversion is exactly
`quaternion_wxyz (x) OPTICAL_TO_USD_CAMERA_WXYZ` -- the SAME module
constant `initialize()` already uses to go optical->usd for the `orient`
xform op (the current, Task #15-fixed value, `(0, 1, 0, 0)`, 180 deg about
X; the nearby code comment describing a `(0, 0, 1, 0)` "y-flip variant" for
this artifact is pre-#15/stale and was NOT used here), composed on the
RIGHT (Hamilton product) so the flip is about the camera's OWN current
local X, not a fixed world axis -- verified by a 90 deg-world-Z-yaw test
that a left-multiply ordering bug would fail (forward alone does not
discriminate the two orders, since it sits on the yaw's own rotation axis
either way; only a cross-axis vector like optical "down" does). Fails
soft, matching `parity_tcp_frame()`: if `initialize()` has not resolved
that camera's prim (or the prim later becomes invalid), it logs
`{"event": "camera_optical_pose_unresolved", "camera": ...}` once (a
`_optical_pose_missing_logged` latch) and returns `None`; the gateway skips
publishing that tick.

`ros_gateway.py`'s `publish()` reuses `pose_in_frame()` directly (imported
from `backend.py`) with `self.backend.root_state()`'s position/
`quaternion_wxyz` (converted to xyzw by reordering) as the frame -- no new
backend method needed, since a camera pose is not a backend/articulation
concept the way TCP/pad points are.

**Tests** (`tests/test_manipulation_runtime.py`): the pure
`usd_camera_pose_to_ros_optical()` conversion at identity (camera at the
origin looking down world -Z with +Y up must publish optical +Z along
world -Z and +Y along world -Y) and under a 90 deg world-Z yaw (the
order-discriminating case above); `CameraRig.camera_optical_pose_world()`'s
fail-soft/log-once contract before `initialize()` has run (no Kit/pxr
needed for this path); the gateway's publisher registration/topic/
frame_id/env-gate source strings; a `publish()` runtime test with a fake
camera rig and a non-trivial (180 deg-about-Z) root pose, asserting both
topics fire every tick with hand-derived-via-`pose_in_frame` world and
base_link values; and the disabled/unresolved/no-camera-rig skip contract
(no publish calls, no raise). Full suite: `tests/test_manipulation_runtime.py`
149 passed, 5 subtests passed, 0 failed (143 passed before this task's 6
new tests); `tests/test_camera_rig.py` and the other camera test files
unaffected (119 passed, 1 subtests passed). No GPU boot for this
diagnostic-only change; the bench recording against the live TF frames is
the follow-up that actually answers the 15 mm bias question this publisher
exists for.

### Task #36 follow-up — Fabric-safe pose (mount read once at init)

**The problem.** Review of the addendum above found that
`camera_optical_pose_world()`'s per-tick `UsdGeom.Xformable(prim).
ComputeLocalToWorldTransform()` reads the wrist camera's `rtx_camera`
prim -- a child of an articulation link, i.e. a physics-driven prim. Under
the default fabric-on config (`resolve_use_fabric()` in `backend.py`;
`/physics/updateToUsd=False`, see the Task #35 entry and the #11/#26/#30
history of this exact failure mode in this codebase) PhysX stops writing
rigid-body transforms back into USD every step, so a plain pxr read
returns the LAST-WRITTEN (e.g. spawn-time) pose, not the live one -- silently
stale during exactly the arm motion this publisher exists to diagnose.
`parity_tcp_frame()` (Task #35) avoids this by reading
`body_pos_w`/`body_quat_w` off the articulation data directly; the camera
code did not.

**The fix.** `camera_optical_pose_world()` no longer touches pxr per tick.
Instead, at `initialize()` time -- while PhysX has not yet stepped and a
plain USD read is still legitimate, and because the offset in question
never changes again after boot -- the rig resolves, for each robot-mounted
camera, the nearest ancestor prim of its `mount_prim` search result that
carries `UsdPhysics.RigidBodyAPI` (`_rigid_body_ancestor()`; a camera
spec's named mount, e.g. `xarm_camera_color_optical_frame`, is typically
several FIXED joints below the actual PhysX-simulated link the URDF
importer keeps as one rigid body -- exactly why the task description
suggested checking `xarm_camera_link`/`link_eef`/`link7` rather than
assuming the spec's own `mount_prim` name is a tensor body). It then reads,
ONCE, `camera_matrix * body_matrix.GetInverse()` (row-vector USD
convention: `world = local * parent_world`) to get the STATIC
`body_T_camera` offset, decomposed and cached as
`CameraRig._mount_local_pose[name]` (position, quaternion_wxyz) alongside
the resolved body's name in `_mount_body_names[name]`. World-fixed cameras
(the arena spectator; `mount_translation` set, no articulation body at
all) get their entire world pose cached directly at init instead
(`_camera_world_pose_static`), since it is static too and needs no body
lookup.

Every tick, `ros_gateway.py` now does: `body_name =
camera_rig.mount_body_name("wrist_camera")`, then `backend.body_pose_world
(body_name)` -- a new `IsaacWholeRobotBackend` method reading the SAME
`body_pos_w`/`body_quat_w` tensors as `parity_tcp_frame` (Fabric-independent,
scalar-last quaternion, `None` if the body is not in `data.body_names`) --
and passes that live pose into `camera_optical_pose_world(name,
body_pose_world)`, which composes it with the cached static offset
(`_compose_wxyz`, pure arithmetic: rotate the local offset by the link's
current world orientation, add; multiply the quaternions) before the same
`usd_camera_pose_to_ros_optical()` conversion as before. `None` on either
side (unresolved mount, or the body's tensor not found this tick) fails
soft exactly as before -- one `camera_optical_pose_unresolved` log latch,
no publish that tick.

**Tests** (`tests/test_manipulation_runtime.py`): two new composition
tests -- `_mount_local_pose`/`_mount_body_names` set directly (bypassing
`initialize()`, no Kit needed), a known link pose (position + scalar-last
quaternion, matching `body_pose_world`'s own convention) composed with a
known static mount offset, checked against a hand-derived (independently,
via plain rotation matrices, not this module's own quaternion helpers)
expectation for position and the optical +z/+y axes -- one with the link
at identity, one with the link yawed 90 deg about world Z and a mount
offset with an X component (discriminates a left- vs right-multiply
composition bug the boresight-only check cannot); and a regression test
that monkeypatches `UsdGeom.Xformable.ComputeLocalToWorldTransform` to
raise and confirms a normal per-tick call still succeeds -- proving the
per-tick path is pure Python. The existing publisher/env-gate/fail-soft
tests were adjusted for the new two-argument `camera_optical_pose_world`
signature and the new `mount_body_name`/`body_pose_world` plumbing
(fakes updated, same assertions). Full suite:
`tests/test_manipulation_runtime.py` 152 passed, 5 subtests passed, 0
failed (149 passed before this follow-up's 3 new tests);
`tests/test_camera_rig.py` and the other camera test files unaffected (119
passed, 1 subtests passed). No GPU boot for this diagnostic-only change;
the live bench recording against `updateToUsd=False` fabric-on is the
follow-up that actually validates the fix tracks the arm during motion.

### Task #33 addendum — PhysX-measured gripper joint torque parity publisher

**Purpose.** The #20 chain above needed the gripper joints' PhysX-MEASURED
torque to diagnose real clamp force, but `/isaac_joint_states`' `effort`
field is `data.applied_torque` -- the Isaac Lab actuator MODEL's own
post-clip COMMAND (`clamp(k*error - d*velocity)`) set INTO the sim, not
what the PhysX solver actually delivered (this is the same distinction
`validation/gripper_close_probe.py`'s `_read_physx_joint_forces()` draws
between its `tau`/`tau_drive` rows and its `physx_tau` row). Same idea as
#35/#36's parity topics, same env gate (`TINKER_SIM_PARITY_TCP`), same
unconditional every-`publish()`-tick cadence: publish the six gripper
joints' (`drive_joint`, `left_finger_joint`, `left_inner_knuckle_joint`,
`right_outer_knuckle_joint`, `right_inner_knuckle_joint`,
`right_finger_joint`) measured PhysX torque next to the existing
actuator-echo topic, so a bench recording of both agrees by construction
with the probe's own numbers.

- `/sim/parity/gripper_physx_tau` (`sensor_msgs/JointState`): `name` is the
  six gripper joints in that fixed order; `position`/`velocity` are the
  same `data.joint_pos`/`data.joint_vel` tensors `joint_state()` reads;
  `effort` is the PhysX-measured torque (see read path below), not the
  actuator echo.

**Read path.** `backend.py` gains a `PARITY_GRIPPER_JOINTS` class constant
(the six names, in publish order) and
`IsaacWholeRobotBackend.parity_gripper_torque()`. The six joints' DOF
indices are resolved once, at bind time, into
`self._parity_gripper_joint_indices` (alongside the existing
`_gripper_mimic_indices`/`_drive_joint_index` resolution), not re-looked-up
per publish tick. The torque itself is exactly the probe's read:
`root_view.get_dof_projected_joint_forces()` (`root_view`/`root_physx_view`,
whichever the articulation exposes) -- "projects the link's incoming joint
force[s] in the motion direction", i.e. the constraint solver's actual
output along each joint's motion axis, as opposed to
`get_dof_max_forces()`/`data.joint_effort_limits` (the CEILING, not the
delivered value) used by the #20 effort-limit read/write paths. Fails soft,
matching `parity_tcp_frame()`: any of the six joints missing from
`joint_names`, the view lacking `get_dof_projected_joint_forces`, or that
call raising, each log once (`parity_gripper_torque_joints_unresolved`/
`_view_unavailable`/`_read_error`, via two latches --
`_parity_gripper_torque_unresolved_logged` for the two static/joint-
resolution cases, `_parity_gripper_torque_error_logged` for a read
exception) and return `None`; the gateway skips publishing that tick
without raising.

**Tests** (`tests/test_manipulation_runtime.py`): the helper against a
scrambled (non-contiguous, includes a non-gripper joint) joint order with a
fake `root_view.get_dof_projected_joint_forces()` row DELIBERATELY
different from `data.applied_torque`, proving indices are resolved by name
and the torque column is the PhysX view's, not the actuator echo;
`parity_gripper_torque()`'s fail-soft/log-once contract when the view call
raises; the gateway's publisher registration/topic/env-gate source
strings; a `publish()` runtime test asserting the topic fires every tick
with `name`/`position`/`velocity`/`effort` taken straight from the backend
call and `header.stamp` matching that tick's `/clock` sample; and the
disabled/unresolved skip contract (no publish calls, no raise) --
mirroring the #35/#36 gateway test shape throughout. The four pre-existing
fake backends in this file that already define `parity_tcp_frame()` and
run with `_parity_tcp_enabled = True` needed a `parity_gripper_torque()`
returning `None` added alongside, since `publish()` now calls it
unconditionally in that same gated block. Full suite:
`tests/test_manipulation_runtime.py` 157 passed, 5 subtests passed, 0
failed (152 passed before this task's 5 new tests). No GPU boot for this
diagnostic-only change; a live bench recording of both `effort` fields side
by side against the probe's own `physx_tau` is the follow-up that confirms
they agree outside the unit-test fakes.

### Task #33 follow-up — sim-side safety-stop transitions, their trigger, and the drive-target snapshot

**Purpose.** The chain above covers the mux and the bridge nodes; the sim
process itself (`simulation/tinker_sim_isaac/backend.py`,
`simulation/tinker_sim_isaac/ros_gateway.py`) had no equivalent -- a
`_safety_stopped` flip on a live bench log was invisible, and there was no
way to tell which of the gateway's ~10 `backend.set_safety_stop(...)` call
sites (init, a direct sample, the heartbeat-timeout re-arm, a rejected
command baseline, the two-phase baseline preflight/commit, an epoch/session
adoption, a lost command stream, its recovery) triggered a given transition.
Observability only, same contract as the addendum above: reuses
`format_duration()` (`tinker_sim_core/observability.py`) and the
never-raise pattern, no packet/target/publish value changes.

**`backend.py`**: `set_safety_stop(active, reason=None)` gained an optional,
caller-named `reason` kwarg (default `None` -> renders `"unspecified"`; no
call site is required to pass it, so this is not a behaviour change). On
every *real* `_safety_stopped` flip (the existing repeated-identical-sample
early return is unchanged and still logs nothing) it prints
`sim_safety_stop state=engaged|released reason=<reason> sim_t=<simulation_time>
wall_age_s=n/a` -- `wall_age_s` is always `n/a` from here because the
backend has no wall clock of its own, only `simulation_time` (that's the
gateway's job, see below). An engage appends
`drive_snapshot=<_safety_snapshot[0, drive_index]>` (the frozen hold target
just latched); a release appends `drive_applied=<_position_targets[0,
drive_index]> drive_measured=<joint_pos[0, drive_index]>` (both equal
immediately after a release, since the fresh hold target IS the measured
joint position at that instant -- expected, not a bug). The whole body is
wrapped `try/except Exception: return` (review round 2, `2a305cb`): this
path is reached from `_adopt_command_epoch`/`_enter_command_stream_lost`
deep inside `spin_once()`'s unguarded loop, so a `print` `BrokenPipeError`
or a stale-view `RuntimeError` from a log-only read must never kill the sim
process. **`reason="init"` never actually appears in a log**: the backend
boots with `_safety_stopped=True` (`IsaacWholeRobotBackend.__init__`), so
the constructor's `set_safety_stop(True, reason="init")` hits the
repeated-identical-sample early return before reaching the print -- the
first visible transition on a live log is always `state=released
reason=sample_false` (the gateway's first genuine clear sample).

A second line, `applied_targets_reset source=<...> drive_before=<...>
drive_after=<...> measured=<...> sim_t=<...> dropped=<n>`, fires at the
FOUR places that replace the whole `_position_targets` tensor (not an
element-wise command write): `set_safety_stop`'s engage (snapshot clone)
and release (fresh `joint_pos.clone()` hold target); `step()`'s per-tick
`_position_targets.copy_(_safety_snapshot)` reassertion while stopped
(guarded by a `_safety_hold_reassert_logged` flag, armed `False` on every
engage, so it fires once on the first tick after the engage, not once per
physics tick for the whole duration of the stop); and
`_refresh_robot_handles`' `_position_targets = joint_pos.clone()` reseed on
every root-view identity change (review round 2 addition -- this one has
NO safety stop involved at all, the exact re-origination shape #33's
stale-hold/gripper-target chain is hunting; `source=refresh_robot_handles:
reset_rebind` for a genuine STOP -> spawn -> PLAY reset or
`:view_recovery` for `_maybe_recover_simulation_view`'s state-preserving
rebind, same `reapply_spawn_yaw` classification `_log_spawn_pose_trace`
already uses; `drive_before` is `n/a` on the very first boot bind, when
`_position_targets` doesn't exist yet). Every argument these four sites
pass is computed through `_safety_drive_scalar`/a new `_robot_joint_pos()`
getattr-chain helper (review round 2: NEVER a bare `self._robot.data.
joint_pos` attribute chain, which can raise before `_log_applied_targets_
reset`'s own try is even entered) and each call site is additionally
wrapped in its own local `try/except`, so nothing this line reads can
propagate. Rate-limited to `APPLIED_TARGETS_RESET_LOG_MAX_PER_WINDOW` (5)
lines per `APPLIED_TARGETS_RESET_LOG_WINDOW_S` (1.0 s) IN TOTAL -- one
shared window across all four sources, not five per source (the original
comment claimed "per source"; the implementation was always a single
counter, and review round 2 fixed the comment instead of the behaviour);
a suppressed line increments a counter that the next line to get through
reports as `dropped=<n>`, so nothing suppressed is unaccounted for. The
ros_gateway paths that only *call* `backend.set_safety_stop(...)` (baseline
preflight/commit, `_adopt_command_epoch`, `_enter_command_stream_lost`, its
recovery) don't duplicate this line themselves -- the backend is the single
source of truth for every wholesale `_position_targets` replace, and every
one of those gateway paths already routes through `set_safety_stop`.

**`ros_gateway.py`**: every one of the ~10 `backend.set_safety_stop(...)`
call sites now passes a distinct `reason=`: `"init"` (constructor),
`"sample_true"`/`"sample_false"` (a direct `/sim/hardware/safety_stop`
sample, both in `_apply_safety_stop`'s non-session-protocol branch and its
default when no caller overrides it), `"safety_stale"`
(`_enforce_safety_deadline`'s heartbeat-timeout re-arm, passed explicitly
into `_apply_safety_stop`), `"command_baseline_rejected"`
(`_reject_staged_baseline`), `"command_baseline_preflight"` /
`"command_baseline_commit"` (`_commit_staged_baseline`'s two-phase stop/
clear), `"session_reset"` / `"command_epoch_retired"`
(`_adopt_command_epoch`, keyed off its existing `new_session` bool),
`"command_stream_lost"` (`_enter_command_stream_lost`), and
`"command_stream_recovered"` (the snapshot-apply loop's mid-stream clear).

The staleness evaluation (`_sim_age_stale`) gained `source`/`wall_age`
kwargs (both observability only, default `"unspecified"`/`None`, no return-
value change -- refactored to compute `sim_age` once instead of three early
returns, same boolean result) and now logs through
`self.node.get_logger().info(...)`: `sim_safety_stale source=<source>
wall_age_s=<...> sim_age_s=<...> timeout_s=<...> stale=<bool>`. Rate-limited
to at most one line per second per `source` while `stale=True` stays true
(the deadline checks run every spin); a `stale=True -> False` transition
always gets one line regardless of the window so a recovery is never
silently swallowed by it; a source that has never gone stale stays silent.
Both callers (`_enforce_command_deadline`, `_enforce_safety_deadline`) pass
`source="command_stream"` / `"safety_heartbeat"` and their own already-
computed wall age. **Reading `stale=` on this line**: both callers only
reach `_sim_age_stale` AFTER their own wall-clock check already failed
(`now - last >= timeout`) -- the sample is already wall-stale by
construction whenever this line is emitted at all. `stale=` is therefore
the SIM-time verdict on top of that: `stale=True` means stale in BOTH
clocks (a genuinely dead publisher, or the sim genuinely outpacing it) and
the deadline actually fires; `stale=False` means wall-stale-but-sim-fresh
-- the stepping loop itself stalled (an RTX render stride, a loaded box)
while a healthy sample sat queued in DDS -- and the deadline is deliberately
NOT applied. It is never evidence of a fully healthy sample; it only
distinguishes "the loop stalled" from "the publisher died."

**Tests.** `tests/test_manipulation_runtime.py`: engage-then-release prints
both lines with the expected reason/state/drive fields; a repeated identical
sample logs nothing; a fresh backend (no `_drive_joint_index` yet,
`_safety_snapshot is None`) degrades every drive field to `n/a` without
raising; a release's `applied_targets_reset` reports the pre-replace
`_position_targets` value (the "ramp" value) as `drive_before` and the
joint_pos-derived fresh target as `drive_after`/`measured`; `step()`'s
per-tick reassertion logs exactly once across three consecutive ticks while
stopped; review round 2 added a rebind test (`_robot_view_identity` forced
to a new value, no safety stop involved) asserting
`source=refresh_robot_handles:reset_rebind` with the pre-rebind value as
`drive_before`. `tests/test_ros_gateway.py`: a fake backend recording
`(active, reason)` pairs confirms a direct sample carries `"sample_true"`,
`_enforce_safety_deadline` under a fake-clock timeout carries
`"safety_stale"`, and `_adopt_command_epoch` carries `"command_epoch_retired"`
or `"session_reset"` depending on `new_session`; a fake `node.get_logger()`
confirms the stale line is rate-limited to one per second, a stale->fresh
transition always logs, and a never-stale source stays silent. Full suite
under the repo's ROS-env pytest incantation (`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`
+ lark-shim `PYTHONPATH`, sourced `/opt/ros/humble/setup.bash` without
`set -u`, `uv run --frozen --no-sync`), review round 2 (`2a305cb` review
fixes, this commit):
`tests/test_manipulation_runtime.py` 165 passed / 5 subtests passed (157
baseline + 8 new, +1 over the first round's 164/7); `tests/test_ros_gateway.py`
27 passed (21 baseline + 6 new, unchanged this round);
`tests/test_gateway_simtime_deadlines.py` 7 passed, unaffected. No GPU boot
for this diagnostic-only change.

### Task #33 follow-up — gripper per-tick target parity publisher (bench round ahh)

**Context, not a fix.** Bench round ahh showed a mid-close stall with
`drive_joint` sitting at 0.341 rad while Isaac Lab's `data.applied_torque`
echo stayed pinned at +2.5 -- consistent with either PhysX's own drive
target already sitting near the measured position, or its stiffness/damping
gains reading near zero, but nothing on the live bench had ever recorded
the PhysX-side per-tick drive target to tell those two apart. Observability
only -- no fix, no behaviour change.

**New:** `/sim/parity/gripper_targets` (`sensor_msgs/JointState`, same
reliable QoS and `TINKER_SIM_PARITY_TCP` env gate as the sibling parity
topics, unconditional every `publish()` tick). For `drive_joint`,
`left_finger_joint`, and `right_outer_knuckle_joint`
(`PARITY_GRIPPER_TARGET_JOINTS` -- the drive, the pad readback #20's
bounded-lead clamp already uses, and a mimic follower on the opposite side
of the linkage), publishes nine `"<joint>/<field>"` names in `position`:
`py_target` (`backend._position_targets`, Python's own commanded target),
`lab_target` (`data.joint_pos_target`, Isaac Lab's actuator-model input
buffer), `physx_target` (`root_view.get_dof_position_targets()`, the PhysX
solver's own drive target -- the layer that had never been recorded),
`physx_k`/`physx_d`/`physx_max_force`/`physx_max_vel` (the live PhysX
gains/limits driving that target), `measured` (`data.joint_pos`), and
`lab_applied_effort` (`data.applied_torque`). Plus `articulation/
target_write` (1 if `step()`'s `TargetWriteGate` actually pushed changed
targets to PhysX that tick, else 0 -- a repeated identical-target tick
skips the write entirely, see `step()`'s `_write_targets` gate) and, if
cheaply resolvable, `articulation/is_sleeping` (`get_physx_simulation_
interface().is_sleeping(stage_id, prim_id)` on the articulation root,
ids resolved once per `_refresh_robot_handles` rebind, never per tick --
unverified against a live Kit process in this round, so a resolution
failure at any point just omits the field entirely rather than publishing
a placeholder).

**Backend:** `IsaacWholeRobotBackend.parity_gripper_targets()` mirrors
`validation/gripper_close_probe.py`'s existing `--arm-stream-hz` readback
rows exactly (same `_read_physx_dof_param`/`_read_physx_position_targets`/
`_read_lab_position_targets` sources, same one-call-per-parameter batching
across all three joints via a new `_parity_read_dof_param` static helper --
five PhysX view calls total per tick, not fifteen). Same two-flag fail-soft
shape as `parity_gripper_torque`: any of the three joints unresolved, the
PhysX view unavailable, or any read in the batch raising all return `None`
(logged once), unlike the probe's own per-field NaN degradation -- this
mirrors the OTHER parity publishers in this file (one bad read drops the
whole tick) rather than mixing real and NaN values in one message. The
gateway wraps its own publish call in a second try/except (log once, not
per tick) so a message-construction failure on that side can't interrupt
the `physics_truth` publish immediately after it either.

**Per-tick cost (not measured on a live bench this round -- no GPU/Kit run
in scope):** five small `(1, num_dofs)` PhysX tensor/array reads (one per
parameter, batched across all three joints) plus four already-in-memory
tensor slices (`_position_targets`/`joint_pos`/`joint_pos_target`/
`applied_torque`) and one already-resolved `is_sleeping` call -- the same
order of magnitude as the existing `parity_gripper_torque` publisher next
to it (one PhysX view call), roughly 5x that call count. Expected to be a
small fraction of the ~24 ms/physics-step budget measured for the
sensor-rich profile (`sim-rtf-sensor-rich-baseline.md`); a live bench
recording that confirms this is the follow-up.

**Also this round (both fixed while reading the surrounding code, still
observability only):**
- `step_profile_snapshot()`'s `changed_targets` used to keep only the
  top-8 most-changed `"<label>:<joint>"` keys by count. A low-traffic key
  like `pos:drive_joint` (the gripper closes far less often than the arm
  moves or the base drives) silently fell off that list whenever 8+ other
  joints changed more in the same window -- exactly the blind spot the
  mid-close stall investigation ran into. The key space is bounded by
  construction (at most 3 labels x `num_joints`, reset every snapshot
  call, never user-input-driven), so the cap is simply removed rather than
  raised to an arbitrary larger number.
- `_mirror_gripper_mimic_targets()`'s docstring claimed "robot.usd dropped
  every `<mimic>`" as the reason the coupling is restored in software.
  Review round 2 (2026-09-07): that claim conflated two different
  readings and was corrected to be exactly what was measured, no more:
  robot.usd authors `physxMimicJoint:rotX:*` attribute values (gearing
  -1.0, referenceJoint drive_joint) on the five followers WITHOUT applying
  `PhysxMimicJointAPI` in apiSchemas; the live Kit stage reports
  `PhysxMimicJointAPI:rotX` among the applied schemas; in both readings
  PhysX creates NO constraint (measured 2026-09-07: followers held at 0 ->
  drive still closes; followers driven -> drive does not move). The
  software mirror is the only coupling -- the operational conclusion is
  unchanged, only the file-vs-runtime claim was corrected. A second,
  near-identical claim in the `ImplicitActuatorCfg` comment block earlier
  in `__init__` (`backend.py` ~1508-1511: "the URDF->USD import dropped
  every `<mimic>`... no drive and no coupling") was deliberately left
  as-is per this round's brief (only the one flagged location was in
  scope), noted here for a follow-up pass.

**Tests** (`tests/test_manipulation_runtime.py`): the readback against a
scrambled (non-contiguous, includes a non-target joint) joint order with
tracked PhysX getter stubs, proving indices resolve by name and every
`get_dof_*` call fires exactly once (batched, not once per joint); a
double lacking a PhysX view returns `None`; the gateway publishes the
backend's `(names, values)` straight through with the tick's sim stamp;
the gateway skips cleanly (no publish, no raise) when
`TINKER_SIM_PARITY_TCP=0`, when the backend returns `None`, and when the
backend's `parity_gripper_targets()` raises -- the last case logging
exactly once across two ticks while `physics_truth` keeps publishing
every tick regardless; and `step_profile_snapshot()` with 9 other joints
changing every round and `drive_joint` changing once, proving all 10 keys
(not 8) come through. Full suite under the same ROS-env pytest incantation
as the round above: `tests/test_manipulation_runtime.py` 171 passed / 5
subtests passed (165 baseline + 6 new); `tests/test_ros_gateway.py` /
`tests/test_gateway_simtime_deadlines.py` unaffected (27 / 7 passed). No
GPU boot for this diagnostic-only change.

**Review round 2 (2026-09-07, live smoke on the round above): the
publisher itself was bench-safe (29/29 names, gains 200/20/2.5/2.0 and
1500/55/2.5/17453.29, 48.9 Hz wall at RTF 0.41, +0.5-0.8 ms/step, zero
Python tracebacks) but `articulation/is_sleeping` resolved the WRONG
prim.** `_resolve_articulation_sleep_ids` used `cfg.prim_path` itself
(`/World/Tinker`, a plain Xform with no `PhysicsRigidBodyAPI`) -- the
actual articulation/body root is `<prim_path>/base_link` (confirmed by
the headless coupling probe, `$TMP/task33-coupling-test.md`, which called
`get_physx_simulation_interface().is_sleeping(stage_id, prim_id)` on
`/World/Tinker/base_link` successfully). Querying the wrong prim id
produced a native `omni.physx.plugin [Error] Error executing isSleeping.`
on EVERY tick in the live smoke -- 11,843 lines in one boot, a C++-level
log Python's `except` cannot see at all -- while the field kept silently
publishing 0. Fixed: `_resolve_articulation_sleep_ids` now resolves
`<prim_path>/base_link` and only accepts it if `body_prim.HasAPI(
UsdPhysics.RigidBodyAPI)` at resolve time; anything else (wrong prim,
missing API, any exception) still omits the field rather than publishing
a placeholder. Also added a one-time self-check: if `is_sleeping()`'s
first successful return value isn't a real `bool`, the field disables
itself for the rest of the backend's life (not just until the next
rebind) and logs once, rather than silently coercing whatever came back
through `bool(...)` every tick forever.

Also this round: `articulation/target_write` used to latch right after
the write GATE's decision (`_target_write_gate.should_write()`), before
`write_data_to_sim()` had even been attempted -- so a swallowed
`write_data_to_sim()` failure (the existing `_maybe_recover_simulation_view`
path) would still publish `target_write=1` for a tick that never actually
reached PhysX. `_last_target_write` is now reset `False` at the very top
of `step()` (covering every return path, including the PHYSICS_READY
rebind branch that never attempts a write at all) and only set `True`
after `write_data_to_sim()` has returned without raising, so the
published value means "PhysX actually received this tick's targets," not
"the gate said yes." A one-line comment was also added at the `.numpy()`
conversion in `_parity_read_dof_param` noting it relies on this backend's
CPU physics pin (a CUDA tensor's `.numpy()` would raise into the batch
except and darken the topic after one log line) -- no code change, this
backend is CPU-physics-only today.

The `_mirror_gripper_mimic_targets()` mimic-comment fix from the round
above was further corrected to state exactly what was measured, no more
(see the bullet above this one) -- the coordinator's review caught that
the first pass's wording implied the shipped file itself carries the
applied `PhysxMimicJointAPI` schema, which is the OPPOSITE of what the
offline asset read found (attribute values authored, but the API not
applied in the file; the API only shows up applied on the live Kit
stage).

Tests added to `tests/test_manipulation_runtime.py`: a real in-memory USD
stage (`pxr.Usd.Stage.CreateInMemory()` -- this venv's bare `pxr` has
`UsdPhysics` but no `PhysicsSchemaTools`, so only that one call is faked
via `unittest.mock.patch`) with `base_link` carrying `RigidBodyAPI`
resolves; without the API, omits; a fake `omni.physx` interface returning
a non-`bool` from `is_sleeping()` disables the field after exactly one
log line, and a second call confirms it stays disabled without calling
the interface again; and `write_data_to_sim()` raising (swallowed by a
stubbed `_maybe_recover_simulation_view`) leaves `_last_target_write`
`False` even when a prior tick had left it `True`. Same ROS-env pytest
incantation: `tests/test_manipulation_runtime.py` 174 passed / 5 subtests
passed (171 baseline + 3 new); `tests/test_ros_gateway.py` /
`tests/test_gateway_simtime_deadlines.py` unaffected (27 / 7 passed). No
GPU boot for this diagnostic-only change (the wrong-prim finding above
came from the coordinator's own live smoke, not a run in this round).

## 2026-09-06 — Task #20: gripper joint effort limits at hardware scale (2.5 N*m), commanded effort mapped onto that ceiling

**The whole #20 chain, in brief.** The gripper's "creep" (an object tipping
along the finger arc and eventually escaping the jaw under a sustained
close) was chased through a long series of levers, each measured and
falsified in turn: object/pad friction magnitude and type, torsional
friction radius, contact/collision approximation (SDF vs convex hull),
follower stiffness/damping, follower effort-limit cap alone (5 through 180,
bit-identical 15 s hold physics -- proof `write_joint_effort_limit_to_sim_
index` alone does not guarantee a cap reaches the PhysX solver for these
mimic-coupled joints), drive-cap alone, mirror-mode (target vs measured),
and a stall-triggered position freeze (PR #19, `fix-gripper-stall-freeze-20`)
that arrested the creep in isolated bench legs but could not hold a
sustained clamp FORCE once the object had already yielded the freeze's lead
tolerance -- the bench acceptance round (`agy`) lost both a sugar box (force
relaxed 16 -> 1.8 N in 0.6 s once frozen) and a soup can (crept faster than
the freeze could arm, 0.08 rad/s). The actual mechanism, confirmed by a
freeze-all-six-targets-at-contact trace, is that the creep is the PD's
*continued advance* toward an unreachable close target -- the followers'
mimic control mirrors drive_joint's target every step, so as long as the
drive keeps asking to close further, the followers keep pushing the object
along the arc even after contact, regardless of any friction/material lever.
That reframes the freeze fix (PR #19) as a symptom patch, now superseded.

**The lever that actually holds: torque authority, not position.** A
real xArm gripper is position-controlled but torque-LIMITED -- it stalls at
its own clamp torque instead of continuing to overhaul the servo target
through the object. The sim's effort limits were nowhere near hardware
scale (drive 50 N*m, followers 180 N*m -- the follower cap's own comment
sized it only to keep drive_joint's 0 lower limit enforceable, not for
grip-force parity). A hardware-scale torque-cap bracket
(`validation/gripper_close_probe.py --drive-effort-limit`/
`--follower-effort-limit`, ported straight PhysX tensor-API write +
readback since the Isaac Lab writer alone was proven insufficient;
`$TMP/hwcap-result.md`, `$TMP/hwcap2-result.md`) swept drive+follower caps
together on a 15 s bottle side-pinch hold + lift:

| cap (N*m) | drive advance | tilt @15s | pad force (N) | lift |
|---|---|---|---|---|
| 1.5/1.5 | holds (-0.014 rad) | ~1° | 7-10 | retained |
| 2.0/2.0 | holds (-0.017 rad) | ~1° | 10-14 | retained (borderline tilt) |
| 2.5/2.5 | holds (flat, target never approached) | ~0.01° | 11-18 | retained (best) |
| 3.0/3.0 | creeps (+0.10 rad) | 16° | 18-23 | dropped |
| 50/180 (control) | runs away (+0.57 rad) | 55° | 0 | lost before lift |

2.5 N*m is the highest cap that holds; the threshold sits between 2.5 and
3.0. Raising the follower cap independently of the drive cap (2.0 drive /
5.0 follower) gave no force benefit -- follower `physx_tau` stayed well
under even the 2.5 cap, confirming the drive cap alone sets the stall
point. A sugar box at 2.0/2.0 held through 15 s with no one-pad dropout
(the uncapped control lost it at 11.6 s).

**Fix.** `simulation/tinker_sim_isaac/backend.py`:
- `GRIPPER_EFFORT_CEILING_NM = 2.5` (module constant, env-overridable via
  `TINKER_SIM_GRIPPER_EFFORT_CEILING_NM`) replaces the old
  `DEFAULT_GRIPPER_EFFORT_LIMIT = 80.0`. The "gripper" (drive_joint) and
  "gripper_mimic" (five followers) `ImplicitActuatorCfg` groups both set
  `effort_limit_sim=2.5` (kept as literal float, not a name reference, so
  the AST-based config test can read it via `ast.literal_eval` -- matches
  the module constant by construction/comment). The old "180 keeps
  drive_joint's limit enforceable" comment on the follower cap is replaced
  with the bracket's finding: that reasoning solved the wrong problem.
- **Mapping.** `GripperCommand.max_effort` (N, hardware fingertip
  convention) is no longer applied 1:1 as N*m on drive_joint. It is mapped
  proportionally onto the hardware ceiling: `gripper_effort_limit_nm(n,
  ceiling, full_scale) = ceiling * clamp(n / full_scale, 0, 1)`, with `n`
  <=0/unset/non-finite resolving to the full ceiling (today's default
  behaviour, not zero authority). `GRIPPER_EFFORT_FULL_SCALE_N = 10.0`
  (also env-overridable, `TINKER_SIM_GRIPPER_EFFORT_FULL_SCALE_N`) -- traced
  the real commands the manipulation stack sends: `gripper_facade.py`'s
  `_execute` forwards `request.command.max_effort` unmodified into
  `JointState.effort` (no substitution, confirmed by reading the code, not
  assuming from a log line -- see "facade forwarding" below), and
  `command_gateway.py`'s `_owned_command` projection is a pure passthrough
  too. `pick_and_place`'s grasp close and pre-open both send
  `native_gripper_max_effort` = 10 N (no launch override), so 10 N is "full
  commanded grip" hardware-side; `grasp_benchmark`'s pre-open sends 5 N; the
  bridge's own reopen sends 50 N (over-range, saturates at the ceiling).
  Mapped values: 10 N -> 2.5 N*m (ceiling), 5 N -> 1.25 N*m, 50 N -> 2.5
  N*m (clamped), 0/None -> 2.5 N*m.
- **Facade forwarding, verified not to need a change.** Re-read
  `gripper_facade.py` end to end for this task: `_execute` sets
  `max_effort = requested_effort` (the goal's own `max_effort`, no
  substitution to 50 anywhere) and publishes it verbatim as
  `command.effort = [max_effort]`. The "effort=50" seen in some bench logs
  is a *different* number -- `result.effort`/`feedback.effort` echo
  `self._effort`, which is populated from `joint_state()`'s
  `data.applied_torque` (the real, measured PD torque), not the commanded
  value -- so a stall reported "at 50" was the drive genuinely saturating at
  whatever ceiling `_set_gripper_effort_limit` last wrote, not proof the
  facade substituted 50 for the request. No bridge-side fix was needed;
  this is a verification finding, not a behavior change.
- **Runtime write reaches the solver.** `_set_gripper_effort_limit` still
  calls `write_joint_effort_limit_to_sim_index` (Isaac Lab buffer) and
  mirrors the mapped limit into the owning `ImplicitActuator`'s cached
  `effort_limit` tensor (issue #128 workaround, unchanged), but now also
  calls a new `_write_gripper_drive_physx_max_force`, ported from
  `validation/gripper_close_probe.py`'s `_write_physx_max_forces_direct` /
  `_author_usd_max_force` (the same direct PhysX tensor-view write +
  USD DriveAPI `maxForce` author the probe used to prove the Isaac Lab
  writer alone was insufficient for the follower joints in `cap5-analysis`).
  It clones the current full `joint_effort_limits` row, patches only
  drive_joint's column, and pushes the whole row back via
  `root_view.set_dof_max_forces` using warp arrays for both payload and
  indices (Task #12 precedent: RigidBodyView tensor-API writes need WARP
  arrays, not torch, to land), then reads it back
  (`get_dof_max_forces`) for the log line. Followers stay config-only at
  2.5 -- no runtime write targets them, matching the bracket recommendation
  (raising the follower cap independently gave no benefit). Every effective
  write now logs `{"event": "gripper_effort_limit", "commanded_n": ...,
  "limit_nm": ..., "physx_max_force": [...]}`.
- Tests: `tests/test_manipulation_runtime.py` gained
  `test_gripper_effort_limit_nm_mapping_monotone_and_capped`,
  `test_gripper_effort_ceiling_and_full_scale_env_overrides`,
  `test_gripper_joint_effort_limits_are_hardware_scale_in_config` (AST/eval
  config contract, fails on main: `gripper` has no `effort_limit_sim` at
  all and `gripper_mimic` resolves to 180.0, not 2.5),
  `test_set_gripper_effort_limit_maps_commanded_effort_proportionally`,
  `test_set_gripper_effort_limit_writes_direct_physx_max_force` (asserts
  the direct PhysX setter is called with the mapped value, via a new
  `_FakeGripperRootView` double), and
  `test_gripper_low_effort_pre_open_maps_above_zero`. Full suite: 115
  passed, 5 subtests passed (was 109 passed, 3 subtests on main).

**Open items.**
- **Pad force is still below hardware.** The bracket's best-holding cap
  (2.5 N*m) produces 11-18 N of pad force; the physical gripper's clamp is
  ~30 N. Pushing the cap higher (3.0 N*m) reliably creeps and drops the
  object in this contact model before that force is reachable -- the sim
  tips objects before it can match the hardware's clamp force. This is an
  open gap in the contact model, not something this fix resolves.
- **2.5 N*m needs pinning to a live hardware measurement.** The 2.5 N*m
  ceiling and the bracket data behind it (`$TMP/hwcap-result.md`,
  `hwcap2-result.md`) are simulation-only; the ceiling should be re-pinned
  to the user's measured hardware clamp force at commanded `max_effort` =
  10 N (native full scale) once that measurement is available. Both the
  ceiling and full-scale constants are env-overridable
  (`TINKER_SIM_GRIPPER_EFFORT_CEILING_NM`,
  `TINKER_SIM_GRIPPER_EFFORT_FULL_SCALE_N`) specifically so that
  recalibration doesn't need a code change.
- **Low-effort OPEN commands are unverified live.** A 5 N pre-open
  (`grasp_benchmark`) maps to 1.25 N*m, and 10 N (the native default used
  elsewhere) maps to the full 2.5 N*m ceiling -- both comfortably above the
  bracket's own 1.5 N*m leg, which held a grasped bottle, so opening in
  free air (much lower resistance than holding an object) should not be
  torque-starved. This worktree cannot launch the sim to confirm it
  directly; `test_gripper_low_effort_pre_open_maps_above_zero` is the
  unit-level floor (mapped limit for both real pre-open values is strictly
  positive), and the staged acceptance
  (`$TMP/effortcap_chain.sh`/`$TMP/effortcap_launch.sh`) exercises the
  mapped ceiling on a live close+hold+lift; a live free-air-open timing
  check is left to the bench round. No `TINKER_SIM_GRIPPER_OPEN_MIN_NM`
  floor was added -- with the corrected `GRIPPER_EFFORT_FULL_SCALE_N` =
  10.0 (not 50.0), the real low-effort commands map to 1.25-2.5 N*m, not
  the 0.25-0.5 N*m an assumed 50 N full scale would have produced, so the
  floor this task considered as a fallback is very likely unnecessary; add
  it only if the bench round finds an open genuinely torque-starved.

**Review fix round (`$TMP/task20-parity-review.md`, REQUEST-CHANGES on
67cd278).** Two findings, both closed:

1. `_write_gripper_drive_physx_max_force` called `wp.from_torch(...)`,
   which -- unlike `wp.array(...)` -- does not lazily initialize the Warp
   runtime (it reads `warp._src.context.runtime.cpu_device` directly and
   raises `AttributeError` if Warp has not been touched yet in-process,
   e.g. a gripper command arriving before the fused-actuator physics-step
   path has run at least once). The bare `except Exception` swallowed that
   with only a JSON diagnostic, while the caller still latched
   `_gripper_effort_limit_written = True`, so a later identical-effort
   command would be dedup-skipped forever without the mapped ceiling ever
   having reached PhysX. Reproduced exactly as the review predicted: running
   `test_set_gripper_effort_limit_writes_direct_physx_max_force` ALONE
   (`-k` that name only) failed `AssertionError: 0 != 1`
   (`set_dof_max_forces` never called) -- it only passed as part of the full
   file because an earlier test's Warp use had already initialized the
   runtime as a side effect. Fixed by building the payload via
   `wp.array(full_cpu.numpy(), dtype=wp.float32, device="cpu")` (the same
   self-initializing pattern `set_entity_pose_physics` already uses, #12)
   plus an explicit `wp.init()` guard, and by moving
   `_gripper_effort_limit_written = True` to fire only after the direct
   PhysX write returns successfully -- any exception now logs a
   `"level": "warning"` JSON line with the error text, leaves the flag
   False so the next identical command retries, and re-raises under
   `TINKER_SIM_STRICT_PHYSX_WRITES=1`. The now-isolated-safe test passes
   alone; two new tests cover the failure-does-not-latch/retry path and the
   strict re-raise path.
2. This branch conflicted with the then-open `feat-spawn-pose-assert-30`
   (PR #18) in `backend.py` (the `GRIPPER_EFFORT_*` constants block sits
   directly above #30's `_yaw_deg_from_quat_xyzw`/`format_spawn_pose_trace`
   helpers -- both are pure appends, no real overlap), the test file's
   import list, and `docs/developer-log.md` (independent same-day entries).
   Merged `origin/feat-spawn-pose-assert-30`; both features kept intact.
   Full suite after merge: `tests/test_manipulation_runtime.py` 130 passed,
   5 subtests passed (117 pre-merge + #30's 13 new).

## 2026-09-06 — Task #30: boot-time spawn-pose guard (a 171 deg, 1.3 m silent spawn miss)

**Symptom.** A GPSR run observed the robot base at `(-0.69, -2.19)` yaw
`+171 deg` in the sim's own `physics_truth` while the launch command was
`--spawn-xy=-2,-2` (yaw unset, i.e. commanded 0). Nothing in the sim log
flagged the ~1.3 m / ~171 deg miss; it only surfaced downstream, in
navigation.

**Refuted theory: a quaternion-convention bug in `TINKER_SIM_SPAWN_YAW_VIA_VIEW`.**
The first hypothesis was that PR #9's opt-in `_apply_spawn_yaw_via_view`
path wrote the root quaternion in the wrong element order. This does not
hold up: the vendored Isaac Lab 3.0.0 factory
(`isaaclab_physx/assets/articulation/articulation.py`,
`isaaclab/assets/articulation/base_articulation.py`) consistently documents
`(x, y, z, w)` for every `write_root_*_pose_to_sim_*` call, and
`backend.py`'s `_apply_spawn_yaw_via_view` already writes exactly that
order -- confirmed live in an earlier probe (commanded yaw=1.5708 read back
`quaternion_xyzw [0, 0, 0.70711, 0.70711]`, yaw error 0.0000 rad). More
decisively: **the flag is dead code for this exact failing run.** Its gate,
identical before and after PR #9,
is `_spawn_yaw_set = abs(resolve_spawn_yaw(TINKER_SIM_SPAWN_YAW)) > 1e-9`
(`backend.py:716-718`); with `TINKER_SIM_SPAWN_YAW` unset (as in the failing
launch), `resolve_spawn_yaw()` returns `0.0`, so `_spawn_yaw_set` is
`False` and `_apply_spawn_yaw_via_view`/`_reapply_spawn_yaw_after_rebind`
never run, `via_view` on or off. No yaw-authoring code of any kind (old USD
`xformOp:orient` path or the new view path) executes when spawn yaw is
unset. `robot.initial_pose` in the scenario JSON is also never read by this
repo's sim-side code (`grep -n "initial_pose"` across `scenario.py`,
`backend.py`, `ros_gateway.py` returns zero hits) -- it is descriptive only.
The likely actual source (out of scope for a tinker-sim-only fix) is an
external consumer (nav-sim-fix) re-posing the robot from its own
scan-to-map fit after reading `/physics_truth`; a ~171 deg heading error
with no matching geometric feature in this repo's arena/scenario files is
the classic signature of a front-back symmetric-room AMCL mismatch, not a
deterministic coordinate-transform bug in this repo.

**Guard, regardless of root cause.** Whoever eventually mis-poses the
robot, the sim should catch its own spawn being wrong instead of staying
silent. Added `IsaacWholeRobotBackend._check_spawn_pose`, run exactly once
per boot and once after any genuine reset rebind (`_refresh_robot_handles`,
only its `reapply_spawn_yaw=True` callers -- the boot bind and the standard
scenario STOP -> spawn -> PLAY cycle, not `_maybe_recover_simulation_view`'s
state-preserving recovery) via a new `_maybe_check_spawn_pose`, called from
`step()`. "Settled" is the same event the base-hold latch already uses: the
moment `_apply_base_hold` latches (`TINKER_SIM_FIX_BASE=1`) or, with the
base free, once `_base_hold_after_sim_s` (2.0s) of sim time has elapsed
since boot or the rebind -- `_spawn_pose_checked`/
`_spawn_pose_check_settle_from` are reset in lockstep with
`_base_hold_settle_from` on every genuine rebind. The check compares
`truth_state()['robot']['base_pose']` (read via the existing
`_robot_truth_state`) against the commanded spawn: xy is `self._spawn_x`/
`self._spawn_y`, the exact validated values `--spawn-xy` already threads
through `run_sim.py` -> `IsaacWholeRobotBackend.__init__(spawn_xy=...)`
(newly stored on `self` -- previously only local variables); yaw is
`self._spawn_yaw` (`resolve_spawn_yaw`, 0 when unset). It always prints one
`{"event": "spawn_pose_check", "commanded": [x,y,yaw], "actual":
[x,y,z,yaw], "dxy_m", "dyaw_deg", "ok"}` line (wrap-aware yaw delta via
`atan2(sin(d), cos(d))`); on a mismatch (`dxy_m > 0.05` or `|dyaw_deg| > 5`,
wrap-aware) it additionally prints a `spawn_pose_mismatch` line (same
fields plus `"level": "warning"`), and if `TINKER_SIM_SPAWN_POSE_ASSERT=strict`
raises `RuntimeError` to abort the boot. Default (unset or any other value)
is warn-only -- no behaviour change for existing launches.

**Tests** (`tests/test_manipulation_runtime.py::SpawnPoseCheckTest`, backend-double
pattern): matching pose emits only `spawn_pose_check` with `ok=True`; a pose
off by 1.3 m / 171 deg (the actual incident numbers) emits both lines with
`dxy_m≈1.324`, `dyaw_deg≈171.0`, `ok=False`, `level=warning`; a wrap case
(commanded 3.1 rad, actual -3.1 rad) reports the short way around (~4.77
deg), not the ~355 deg a naive subtraction would give, and is `ok=True`;
`TINKER_SIM_SPAWN_POSE_ASSERT=strict` raises. All 4 fail on main first with
`AttributeError: 'IsaacWholeRobotBackend' object has no attribute
'_maybe_check_spawn_pose'`. Full suite: `tests/test_manipulation_runtime.py`
113 passed, 3 subtests passed, 0 failed. No GPU boot for this change.

**Follow-up: live matrix on `gpsr-rcw2026-bench` narrows the trigger to Fabric,
not this repo's spawn-yaw code, and boot instrumentation was added to watch
the whole spawn path.** A live A/B on the `rcw2026` arena
(`gpsr-rcw2026-bench`, `--spawn-xy=-2,-2`, commanded yaw 0) across the
spawn-yaw/Fabric knobs:

| case | `TINKER_SIM_SPAWN_YAW` | via-view | Fabric | landed pose |
| --- | --- | --- | --- | --- |
| 1/2 | unset | — | ON (default) | `(-0.69, -2.19, 171°)` -- the original incident, reproduced |
| 4 | `1.5708` | ON | ON | exact |
| 6 | `1e-6` | ON | ON | `(-2.607, -2.032, 1.7°)` STATIC, 61 cm off in x |
| 7 | `1e-6` | OFF | OFF (forced, USD pre-reset write) | exact `(-2.000, -2.000)` |
| 8 | unset | — | OFF (`TINKER_SIM_USE_FABRIC=0`) | TUMBLES: `(-1.29, -2.17, 165°)` -> `(-0.55, -2.56, 63°)` |

Case 7 confirms the USD-authoritative path (13e4fdf's original fix) is sound
whenever it actually runs. Case 8 is the decisive new result: forcing Fabric
off with **no** `TINKER_SIM_SPAWN_YAW` set at all (so none of this repo's
yaw-write code runs, USD or view) still produces a bad spawn -- worse, a
*dynamic tumble* rather than a static offset. That rules out every write path
in `backend.py` as the proximate cause for the yaw-unset failure mode (case
1/2, the original incident) and points at default `InitialStateCfg`
placement / Fabric's own root-body ingestion at spawn time being broken in
this scenario independent of any TINKER_SIM_SPAWN_YAW code. Case 6 (via-view
ON, Fabric ON, yaw forced tiny-but-nonzero to exercise the view-write path)
lands statically wrong (61 cm off, not tumbling) -- a different, still
unexplained failure mode from case 8's dynamic tumble, so "Fabric on" alone
isn't a single clean explanation either; case 6 needs its own root-cause
pass. Full analysis, including the ruled-out quaternion/collider/spawn-yaw
hypotheses that predate this matrix, is in
`task30-spawn-path-findings.md` (background) and the live matrix above
(this session).

**Boot instrumentation: `spawn_pose_trace`.** Given the matrix above shows the
mislocation can already be present very early (case 8's first captured frame
is off before the ~150-frame settle burst even starts), `_check_spawn_pose`'s
single post-settle sample cannot show WHERE in the boot sequence the pose
goes wrong. Added a `{"event": "spawn_pose_trace", "stage": ..., "t":
sim_time, "root_pos": [x,y,z], "root_yaw_deg": ..., "base_link_pos": [...] if
resolvable, "via"/"reason" where applicable}` line at each stage of the boot
path, default-on and cheap (a handful of lines per boot, `backend.py`):

- `"initial_state"` -- the `ArticulationCfg.InitialStateCfg` pos/rot exactly
  as configured, before the spawner or the articulation object exist
  (`backend.py`, right before `self._robot = Articulation(robot_cfg)`).
- `"after_reset"` -- the truth root pose the instant `self._sim.reset()`
  returns, from `data.root_pos_w`/`root_quat_w`, before
  `_refresh_robot_handles` can touch it (right after the `reset()` call).
- `"after_spawn_yaw_write"` -- after whichever spawn-yaw write actually ran,
  with `"via": "usd"` or `"via": "view"`: the pre-reset USD `xformOp:orient`
  branch logs the just-authored values (no live tensor view yet); the
  post-reset `_apply_spawn_yaw_via_view` logs the tensor-view read-back
  right after `write_root_pose_to_sim_index`. Neither prints when
  `TINKER_SIM_SPAWN_YAW` is unset (matching case 8/1-2 above: no write of any
  kind runs on that path -- the trace makes that absence visible instead of
  silent).
- `"after_first_step"` -- once, right after the first successful
  `self._robot.update(self.dt)` following boot, from both `step()` call
  sites that can reach it (the normal path and the PHYSICS_READY-rebind
  early-return branch) -- guarded by `_spawn_pose_trace_first_step_logged`
  so it never re-fires on a later scenario rebind (that gets its own
  `after_rebind` stage instead).
- `"after_settle"` -- inside `_check_spawn_pose` itself, alongside the
  existing `spawn_pose_check` line, from the exact base pose that check
  already computed.
- `"after_rebind:<reason>"` -- inside `_refresh_robot_handles`, in the
  branch that already detects a genuine view-identity change
  (`self._robot_view_identity is not None`, i.e. not the initial bind).
  `<reason>` is `reset_rebind` for the scenario's own STOP -> spawn_entity(s)
  -> PLAY cycle (`reapply_spawn_yaw=True`) or `view_recovery` for
  `_maybe_recover_simulation_view`'s state-preserving mid-run recovery
  (`reapply_spawn_yaw=False`). This backend has no handle on which entity
  triggered a STOP/PLAY cycle -- that lives cross-process, in
  `ros2_ws/.../scenario_runner.py`'s `/spawn_entity` calls -- so only the
  reason classification is logged here; entity identity is covered by
  `scenario_spawn` below.

`base_link` vs the articulation root: `root_pos_w`/`root_quat_w` (used
everywhere in this file) report the ARTICULATION ROOT's pose
(`/World/Tinker`), which for a USD-referenced import need not be the same
prim as the URDF's own `base_link` link (`source-tinker-full.urdf` names
`base_link` as its actual root `<link>`). New helpers
`_base_link_body_index`/`_base_link_pose` look up `"base_link"` in
`data.body_names` and read `body_pos_w`/`body_quat_w` at that index when
present; `_log_spawn_pose_trace` includes `base_link_pos`/
`base_link_yaw_deg` whenever that lookup succeeds, so a divergence between
the two (or its absence) is visible in the trace instead of assumed away.
Not independently confirmed live in this session (no GPU boot) whether the
tinker2 USD's `body_names` actually contains a distinct `"base_link"` entry
separate from body index 0 -- the code degrades to omitting the field if it
doesn't, by design (`_log_spawn_pose_trace` never raises on a failed
lookup).

All of the above is formatted by a new pure/stateless `format_spawn_pose_trace`
function (`backend.py`, next to `resolve_use_fabric`) so the exact JSON shape
is unit-testable without a live sim.

**Scenario spawn sequence: `scenario_spawn`.** The `/spawn_entity` and
`/set_simulation_state` simulation_interfaces services are owned by the
out-of-repo `isaacsim.ros2.sim_control` extension, not by any code in this
repo -- `backend.py` never sees a spawn request directly, only its
after-the-fact effect (a view-identity change in `_refresh_robot_handles`,
see `after_rebind` above). The one place this repo issues those calls is
`ros2_ws/src/tinker_sim_bridge/tinker_sim_bridge/scenario_runner.py`'s
`ScenarioRunner.execute()`, which walks the `ScenarioOperation` tuple
`tinker_sim_core.orchestration.standard_operations` compiles. Added one
`{"event": "scenario_spawn", "name": ..., "pose": {"xyz": ...,
"quaternion_xyzw": ...}, "t": time.monotonic(), "stop_play_cycle": bool}`
line per `spawn_entity` operation, right after its service response
confirms the entity name. `stop_play_cycle` is tracked from the boundary
names on the `set_simulation_state` operations the same loop already walks:
`True` from the `"SPAWN_READY"` (STOPPED) boundary until the final
`"PHYSICS_READY"` (PLAYING) boundary -- `False` throughout for the opt-in
`--spawn-while-playing` mode, which skips that bracket entirely (see
`orchestration.standard_operations`'s own docstring on why that mode exists).
This `t` is wall-clock monotonic in the ROS-side runner process, a different
clock domain from `backend.py`'s sim-time `t` above (the two processes don't
share a clock) -- correlating a `scenario_spawn` line with a
`spawn_pose_trace`/`after_rebind` line needs matching by proximity/ordering
in the combined log, not by comparing `t` values directly.

**Tests.** `tests/test_manipulation_runtime.py::SpawnPoseTraceFormatTest`
(new): `format_spawn_pose_trace`'s exact JSON shape (fields present/absent
per optional argument); `_log_spawn_pose_trace` from a backend double (no
live sim) with a `base_link` body at a pose DIFFERENT from the root,
confirming both are logged and differ; and the omit-when-unresolvable case
(the stock backend double's `body_names` has no `"base_link"` entry).
`SpawnPoseCheckTest`'s existing assertions were updated to filter for
`spawn_pose_check`/`spawn_pose_mismatch` events specifically (the new
`after_settle` trace line now also prints during `_maybe_check_spawn_pose`,
in the same captured-stdout window those tests inspect) -- their pass/fail
logic is otherwise unchanged. Full suite:
`tests/test_manipulation_runtime.py` 117 passed, 3 subtests passed, 0
failed (up from 113 passed pre-change: 4 new `SpawnPoseTraceFormatTest`
cases). No GPU boot for this change; case 6's static 61 cm offset and case
8's dynamic tumble both remain open root-cause items the new trace is meant
to help localize on the next live run.

**Root cause found: `InitialStateCfg.rot` is scalar-last, backend built it
scalar-first -- the robot spawned upside down.** The `spawn_pose_trace`
instrumentation above, followed live, found the actual bug the case
1/2/6/8 matrix above was chasing without a mechanism. The vendored Isaac Lab
3.0.0 `AssetBaseCfg.InitialStateCfg` (`.deps/IsaacLab/source/isaaclab/
isaaclab/assets/asset_base_cfg.py:37-40`) documents its `rot` field as
scalar-last `(x, y, z, w)`, defaulting to `(0.0, 0.0, 0.0, 1.0)`. `backend.py`
built that keyword by hand as `(cos(yaw/2), 0, 0, sin(yaw/2))` -- scalar
FIRST. For the common case, an unset/zero spawn yaw, that expression
evaluates to `(1.0, 0.0, 0.0, 0.0)`, which under the real scalar-last
contract is not identity at all: it is a 180 degree rotation about world X.
`Articulation.reset()` *does* apply `init_state` (the old comment above the
construction, claiming "the spawner drops this rot" for a USD-referenced
articulation, was itself wrong/outdated for the vendored 3.0.0 spawner --
what it was actually observing was this order bug, not a dropped write) --
so every default-heading boot has been initializing the robot inverted.
Live trace numbers from the run that caught it: `after_reset` root `z=0.30`
sitting ABOVE `base_link z=0.25` (impossible right-side-up, trivial upside
down: the root offset that is normally below `base_link` flips to above it),
followed by a first-step ejection to `(-0.69, -2.19, 171°)` -- exactly case
1/2's original incident numbers. Two things masked this for non-zero yaw:
(a) `TINKER_SIM_SPAWN_YAW=1.5708` supplies `(cos(45°), 0, 0, sin(45°)) =
(0.707, 0, 0, 0.707)`, which the scalar-last contract reads as a 90 degree
ROLL about X, not the intended 90 degree yaw about Z -- rescued only because
the pre-reset USD `xformOp:orient` write (Fabric-off path, correctly
real-first for `Gf.Quatd`) or `_apply_spawn_yaw_via_view` (already
correctly scalar-last, post-reset tensor write) overwrite the orientation
before it matters (matrix cases 4 and 7); (b) case 8 (yaw unset, Fabric
forced off, neither write path engaged because the `abs(spawn_yaw) > 1e-9`
guard is false) tumbles instead of landing cleanly -- consistent with an
upside-down spawn correcting itself dynamically rather than being rescued by
either write path.

**Fix.** Added a pure `spawn_root_rot_xyzw(yaw) -> (0.0, 0.0, sin(yaw/2),
cos(yaw/2))` (`backend.py`, next to `resolve_spawn_yaw`) as the single
source of a correctly-ordered heading-about-Z quaternion, and routed the
`InitialStateCfg.rot` construction, the `"initial_state"`/
`"after_spawn_yaw_write"` trace payloads, and `_apply_spawn_yaw_via_view`'s
tensor-write quaternion through it (all previously duplicated the
by-hand `sin`/`cos` construction; now there is exactly one place the order
can go wrong). Left untouched, per the vendored contracts each site
actually uses: `_apply_spawn_yaw_via_view` (already scalar-last -- its own
docstring already warned not to copy the `Gf.Quatd` construction into it,
which turned out to be exactly backwards advice for the other direction:
the copying had happened into `InitialStateCfg.rot` instead) and the
pre-reset USD `xformOp:orient` authoring's `Gf.Quatd(cos, 0, 0, sin)` (`Gf`
quaternions are scalar-first by construction -- this one was already
correct and must stay real-first, not be "fixed" to scalar-last). Corrected
the stale "the spawner drops this rot" comment in the same edit.

**Tests** (`tests/test_manipulation_runtime.py`): `SpawnRootRotXyzwTest` --
`spawn_root_rot_xyzw(0.0) == (0.0, 0.0, 0.0, 1.0)` (the vendored identity
default) and `spawn_root_rot_xyzw(pi/2) ≈ (0, 0, 0.7071, 0.7071)`, plus an
explicit regression guard that a zero yaw must NOT reproduce the old
scalar-first bug's `(1, 0, 0, 0)`. `BackendInitStateRotConstructionTest` --
extracts the literal `rot=` keyword source off the
`*.InitialStateCfg(...)` call in `backend.py` via `ast` (no Isaac import
needed) and `eval`s it for an unset and a `1.5708` `TINKER_SIM_SPAWN_YAW`,
asserting it equals `spawn_root_rot_xyzw(spawn_yaw)` exactly -- this
exercises the backend's actual construction, not a hand-reimplementation of
it in the test. Confirmed both new construction-test cases FAIL against the
pre-fix (scalar-first) construction:
`AssertionError: Tuples differ: (1.0, 0.0, 0.0, 0.0) != (0.0, 0.0, 0.0, 1.0)`
(unset yaw) and the equivalent element-0/2/3 mismatch for yaw `1.5708`. Full
suite after the fix: `tests/test_manipulation_runtime.py` 122 passed, 3
subtests passed, 0 failed (up from 117: 5 new cases). No GPU boot for this
change; PR #18's boot-time `spawn_pose_check` assertion and the
`spawn_pose_trace` instrumentation above remain in place as the live guard
that would have caught this immediately had they existed before it did.
## 2026-09-06 — Task #11 follow-on: truth evaluator only republished the first object in `objects[]`

**Symptom.** Bench round `agw`: `/sim/truth/object_state` (the public topic
the bench's `truth_recorder.py` subscribes to) only ever carried
`delivery_object` across 10085 frames, even though the internal stream
(`/sim/internal/physics_truth`) carried `bench_soup_100_anygrasp_agw_a1` and
`bench_sugar_box_100_anygrasp_agw_a1` poses from t~309s onward, and
`ContactTruth` messages for those same bench objects reached the bench log
fine. So the backend-side spawned-entity truth work (#11, merged as PR #14,
`backend.py` `_spawned_object_states()`/`truth_state()`) was producing
correct data that a downstream hop was silently dropping.

**Root cause.** `truth_evaluator.py`'s `TruthFrame.from_mapping` (~line 151)
parsed only `payload.get("object")` (singular) and never touched
`payload.get("objects")` (plural) at all. `_on_truth` then published at most
one `ObjectTruth` per frame from `frame.object`. `backend.py`'s
`truth_state()` (~3417-3421) deliberately keeps `object` = `objects[0]`
"unchanged for existing consumers" and appends spawned entities *after* the
declared ones in `objects[]` (~2941-2944) — so any spawned body is always
`objects[1:]` and never reaches `frame.object`, and therefore never reaches
the public topic. Contacts were unaffected because `_on_truth` already
loops `for contact in frame.contacts` (~512-520); only the object republish
path lacked the equivalent loop. The internal `/sim/internal/physics_truth`
stream already carried the full `objects[]` array — this was purely a
bridge-side republish gap, requiring only a bridge rebuild (no backend or
scenario changes) to fix.

**Fix** (`ros2_ws/src/tinker_sim_bridge/tinker_sim_bridge/truth_evaluator.py`):
`TruthFrame` gained an `objects: tuple[Mapping[str, Any], ...]` field,
parsed from `payload["objects"]` when present, falling back to a
single-element tuple built from the legacy singular `payload["object"]`
otherwise (empty tuple when neither is present). `object` (singular) is
kept as a back-compat alias = `objects[0] if objects else None`, so nothing
that reads `frame.object` needed to change. `_on_truth` replaced its single
`if frame.object is not None: ... publish` block with
`for obj in frame.objects: ... self._object_pub.publish(...)`, mirroring
the existing contacts loop; `retained_by_gripper` is set only for the
element that `is frame.object` (i.e. `objects[0]`) since retention is
evaluated against a single task object, and `False` for every other
republished entity.

**Audit of `TruthEvaluatorCore.process`/`evaluate()` metrics.** These still
key off `frame.object` only (unchanged semantics, as required — this task
did not touch retention logic). `evaluate()` computes `lift_m`,
`translation_m`, `drift_m`/`drift_deg`, `stable_window_s`, and `retained`
against `frame.object`'s pose alone; if per-spawned-object retention
metrics are ever wanted (e.g. "is `bench_soup_...` also retained"), that is
a separate, not-yet-scoped change to `evaluate()`/`RetentionMetrics`, not
covered here.

**Tests** (`tests/test_manipulation_evaluator.py`): `test_objects_plural_parses_declared_and_spawned_entities`
feeds a payload with `objects = [declared, spawned]` and asserts
`TruthFrame.objects` has both entries in order with the spawned object's id
and pose intact, and that `frame.object` still aliases `objects[0]`.
`test_legacy_singular_object_still_yields_exactly_one_entry` and
`test_no_object_yields_empty_objects_tuple` cover the back-compat and
no-object cases. All three fail on the pre-fix code with
`AttributeError: 'TruthFrame' object has no attribute 'objects'`.
`test_on_truth_publishes_one_object_truth_per_object_in_frame` is an
AST-based structural check (rclpy isn't importable in this venv, matching
the existing test file's pattern for `TruthEvaluatorNode`) asserting
`_on_truth` contains a `for x in frame.objects:` loop that calls
`self._object_pub.publish(...)`; it fails on pre-fix code with
`AssertionError: _on_truth must loop over frame.objects ...`. Full file:
19 passed (was 15 before this round). No other test file in the repo
exercises `TruthFrame`/`ObjectTruth`/`_on_truth` behaviorally — the other
files matching `truth_evaluator` (`test_contract_guard.py`,
`test_integrated_evidence_index.py`, `test_integrated_static_contracts.py`,
`test_integrated_ompl_launch_contract.py`, `test_qualification_manifest.py`,
`test_integrated_qualification.py`) only assert static contract facts
(node/topic/message-type names, launch cardinality) unaffected by this
change.

## 2026-09-06 — Task #27: gripper facade false stall at low RTF (dwell ran on the wall clock)

**Symptom.** Bench round `agv`: a close goal reported
`Gripper close ok: position=0.013920 effort=9.992524 stalled=1
reached_goal=0` only ~0.45s wall after the goal started -- 4% of the drive's
stroke, no contact -- and `pick_and_place` immediately attached the object
and lifted; the arm lifted air. Bench RTF at the time was ~0.27.

**Root cause.** `gripper_facade.py`'s `_execute` runs with
`use_sim_time=True` and already uses `self.get_clock().now()` correctly for
`simulation_timeout`, but the no-progress stall dwell
(`stall_dwell_s`, default 0.3s) was timed with the free function
`time.monotonic()` -- WALL seconds -- via `last_progress_at`/`now_monotonic`
(and, since #23, `contact_stalled` inherits the same dwell). At RTF 0.27,
0.3s of WALL time is only ~0.08s of SIM time, well under the ~0.2s of SIM
actuation latency the drive needs before it even starts moving, so the
dwell fired before the drive had any chance to make progress -- confirmed
against the bench trace's timestamps and the truth-drive samples in
`/home/tinker/.claude/jobs/01ca17b4/tmp/task27-facade-false-stall-findings.md`.
`start_wall` and the 30s wall watchdog are correctly wall-clock (they bound
a mux/keepalive contract measured in wall seconds) and were left alone.

**Fix.** `last_progress_at` and the per-iteration `now_sim` (formerly
`now_monotonic`) now read `self.get_clock().now().nanoseconds * 1e-9`, the
same sim clock `simulation_timeout` already used; `contact_since` (and
therefore `contact_stalled`) now derives from the same `now_sim`, so both
stall paths share one clock. No parameter values changed.

**Observability.** The facade had zero log lines. Added
`_log_execute_outcome`, called once at every terminal `_execute` exit
(`aborted_safety_stop`, `canceled`, `reached_goal`/`stalled`, `timeout_sim`/
`timeout_wall`) with position, effort, stalled, reached_goal, and elapsed
sim/wall seconds -- what previously took cross-referencing the client's own
log line against the truth evaluator is now a single grep on
`tinker_sim_gripper_facade`.

**Test.** `tests/test_gripper_executor_humble.py::
test_position_stall_dwell_uses_sim_clock_not_wall_clock_at_low_rtf` drives a
synthetic `/clock` feed at RTF ~0.27 alongside `/isaac_joint_states` samples
that stay parked for 0.2s SIM, ramp for 0.15s SIM, then genuinely plateau.
On main it failed with:
`AssertionError: goal finished before the drive could have genuinely
stalled in SIM time -- the stall dwell is still running on WALL time`
(`assert not True`, at the 1.4s-wall checkpoint where sim time has only
reached ~0.38s). Getting this test running for real also surfaced an
unrelated test-harness trap worth recording: the file's existing
`_run_goal_with_timer` helper drives `_execute()` from a
`node.create_timer()` callback with no explicit `callback_group`, which
defaults to the node's default `MutuallyExclusiveCallbackGroup` -- the same
group rclpy's internal `TimeSource` uses for its own `/clock` subscription.
`_execute()`'s while-loop then monopolizes that group for the whole goal,
silently freezing `self.get_clock().now()` until the goal ends (reproduced
in isolation before diagnosing it). The real `ActionServer` doesn't have
this problem -- `execute_callback` runs in its own explicit
`ReentrantCallbackGroup` -- so the new test drives `_execute()` from a plain
background thread instead, matching that disjointness. Full targeted run:
`tests/test_gripper_executor_humble.py` 12 passed;
`tests/test_manipulation_runtime.py -k "facade or gripper"` 11 passed; full
`test_manipulation_runtime.py` 102 passed, 3 subtests passed, 0 failed.

**Remaining margin (follow-up, not fixed here).** The dwell (0.3s SIM) and
the observed actuation latency (~0.2s SIM) are close enough that a slower
actuation response, or a lower stall_epsilon combined with sensor noise,
could still trip a false-ish stall inside that 0.1s SIM margin. Not
reproduced live; flagged for whoever next tunes `stall_dwell_s` or
actuation timing.

## 2026-09-06 — Task #11: contact-report enablement was invisible in the sim log

**Symptom.** A whole bench round (`agv`) came back with `/sim/truth/contacts`
publishing nothing, `/sim/parity/finger_contact` flat zero on every axis for
its entire ~6100-sample force trace, and every physics-truth frame's
`contacts` field an empty list — a structurally-silent contact pipeline with
no error, no exception, and nothing distinguishing it in the sim's stdout
from a genuinely contact-free round.

**Diagnosis.** Not a regression in the round's own code path. PhysX contact
reporting is decided exactly once, at `IsaacWholeRobotBackend.__init__`
(`simulation/tinker_sim_isaac/backend.py`), from the `enable_contacts`
constructor argument — which the sensor-rich launch path derives from
`TINKER_SIM_SENSOR_RICH_CONTACTS` (`validation/run_sim.py`). There is no
later re-check: if that boot's environment lacked the flag, the
`subscribe_contact_report_events` call is skipped entirely and
`_on_contact_report_event` (the sole writer of the backend's contact-pairs
dict, which both `/sim/parity/finger_contact` and physics-truth's `contacts`
list read from) never fires for the rest of that process's life — no matter
what changes afterward in the launched task stack. Round `agv`'s bringup
(`restart_c_by_pid.sh`) explicitly restarts only the task-stack terminal
("Isaac + control plane stay up"), so whatever `TINKER_SIM_SENSOR_RICH_CONTACTS`
value the *already-running* Isaac Sim process booted with is the one that
silently governs contacts for every subsequent round until Isaac itself is
restarted — and nothing in the log said which value that was.

**Fix (observability only, no behavior change).** Two new one-line JSON
diagnostics in `backend.py`, alongside the existing `wheel_collider` /
`solver_iterations` boot lines:
- At `__init__`, right where `enable_contacts` is consumed:
  `{"event": "contact_report", "enabled": <bool>, "source":
  "TINKER_SIM_SENSOR_RICH_CONTACTS" | "constructor:enable_contacts",
  "monitored_bodies": <count>}` — printed unconditionally, so a stale
  contacts-off boot is visible in sim stdout without an `/proc/<pid>/environ`
  read or a live force probe. `source` names the env gate when it was set,
  falling back to naming the constructor argument for callers
  (`manipulation-core`, probes) that pass `enable_contacts` explicitly.
- In `_on_contact_report_event`, the first time a pair actually gets
  recorded (not merely reported and filtered out, and — after the #28 merge
  below — only counting a monitored ARM/GRASP/EXTRA pair, not a trace-only
  one): `{"event": "contact_report_first_event", "simulation_time": <t>,
  "pair": [body_a, body_b]}`, gated by a one-shot bool so it never repeats
  per-contact.

**Operational rule.** For any round that depends on contacts, restart Isaac
Sim itself — a task-stack-only restart keeps the prior process's
`enable_contacts` decision no matter what the next launch wrapper exports.
Before trusting a per-round env override, check the *running* process's own
environment (`cat /proc/<pid>/environ | tr '\0' '\n' | grep
TINKER_SIM_SENSOR_RICH_CONTACTS`), or now, simply read the `contact_report`
boot line the backend already prints.

Non-GPU coverage added (`tests/test_manipulation_runtime.py`,
`test_contact_report_first_event_logged_exactly_once`): two recorded contact
events (found, then persist) against a backend test double produce exactly
one `contact_report_first_event` line, carrying the expected body-pair and a
`simulation_time` field. Full suite (pre-#28): 103 passed, 3 subtests passed,
0 failed.

## 2026-09-06 — Task #28: contact reporting only sees the arm and the fingers, not the gripper base or the wrist camera

**Why.** A bench round on `agv` pushed the target object with the robot but
could not tell *who* pushed it: `_on_contact_report_event`
(`simulation/tinker_sim_isaac/backend.py`) only records a pair when one
actor's trailing prim name is in `ARM_CONTACT_BODIES` (`link1..link7`) or
`GRASP_CONTACT_BODIES` (`left_finger`, `right_finger`, `link_tcp`) — a shove
from the gripper base, the wrist-camera housing, or anything else past
`link7` was invisible on `contact_pairs()`/`contact_state()`, and there was
no way to get the raw per-sample contact points (only the force-weighted
average) to see where on an object it was pushed.

**Fix: two env-gated additions to the same monitored-pair path, no change
to default behavior.**

- `TINKER_SIM_CONTACT_EXTRA_BODIES` (comma-separated body names, default
  empty): adds these names to the boot-time monitored set alongside
  `ARM_CONTACT_BODIES`/`GRASP_CONTACT_BODIES`, so their pairs are recorded
  by `contact_pairs()` and aggregated by `contact_state()` under their own
  name key, the same mechanism `GRASP_CONTACT_BODIES` already use.
  `left_finger`/`right_finger` aggregation is untouched — this only adds new
  keys. Unset, `self._contact_extra_bodies == ()` and the monitored set is
  byte-identical to before.
- `TINKER_SIM_CONTACT_TRACE_BODIES` (comma-separated names, default empty,
  ported from a local validation branch, commit 045bf77): any pair where
  either actor's trailing name matches is recorded on a separate
  `contact_trace_pairs()` accessor regardless of the monitored set, keeping
  up to 4 raw per-sample points/normals (`points`, `normals`,
  `point_count`) instead of only the force-weighted average — for pointing
  a probe at one object and seeing every pair that touches it, with where.
  This does not add entries to `contact_pairs()`/`contact_state()`.

Both are logged once at boot as a `contact_bodies` JSON line (next to the
subscribe call, `enable_contacts` branch) so a live run can confirm which
bodies are covered without reading source: `{"event": "contact_bodies",
"arm": [...], "grasp": [...], "extra": [...], "trace": [...]}`.

**Resolved body names for the bench.** The robot's link names come straight
from `integration/model-bundle-r2/simulator_full_urdf/source-tinker-full.urdf`
(the sim's `/World/Tinker/{name}` prims are 1:1 with this URDF's `<link>`
names — `ARM_CONTACT_BODIES`/`GRASP_CONTACT_BODIES` are already exact
matches to `link1..link7`/`left_finger`/`right_finger`/`link_tcp` there).
Walking the fixed-joint chain past `link7`:
- Gripper base / palm: `xarm_gripper_base_link` (`link_eef` -> `gripper_fix`
  (fixed) -> `xarm_gripper_base_link`, line 556).
- Wrist camera housing / cam-stand: `xarm_camera_link` (`link_eef` ->
  `xarm_camera_joint` (fixed) -> `xarm_camera_bottom_screw_frame` ->
  `xarm_camera_link_joint` (fixed) -> `xarm_camera_link`, line 947) — this is
  the only body in that chain with collision geometry (a 0.02505 x 0.09 x
  0.025 m box), and it is what the `cam-stand` wrist-camera preset
  (`head_camera_aim.py`) repositions; `cam-stand` itself is a TF/render
  preset, not a separate rigid body.
- "Wrist housing" pushes proper (i.e. on `link7` itself) are already visible
  today via `ARM_CONTACT_BODIES` and need no new env value.

Recommended bench value:
`TINKER_SIM_CONTACT_EXTRA_BODIES=xarm_gripper_base_link,xarm_camera_link`.

**Tests.** `tests/test_manipulation_runtime.py`: unset-env parity
(`test_contact_extra_bodies_unset_matches_default_monitored_set`), extra
bodies recorded and aggregated like a grasp body without touching
`left_finger`/`right_finger`
(`test_contact_extra_bodies_env_records_pair_like_grasp_bodies`), the ported
trace-bodies tests
(`test_contact_report_drops_unmonitored_pair_without_trace_env`,
`test_contact_report_trace_env_records_unmonitored_pair_with_point_and_normal`),
and (added on the #11 merge, below) leaf-or-full-path matching for both
matchers against the bench's actual truth-pair shape, where a spawned
entity's rigid body IS its own scenario prim path
(`test_contact_trace_and_extra_bodies_match_full_scenario_prim_path`).
Full incantation from `pytest-suite-ros-env-incantation`: see below for the
merged suite's final count (both #11's and #28's tests included).

**Follow-up (same day, on the #11 merge).** `_on_contact_report_event`'s
`monitored`/`is_traced` checks originally matched `TINKER_SIM_CONTACT_EXTRA_BODIES`
only by reconstructing `/World/Tinker/{name}` and `TINKER_SIM_CONTACT_TRACE_BODIES`
only by the actor's own leaf. The bench's real truth pairs carry full prim
paths on both sides (e.g. `a="/World/Tinker/left_finger"`,
`b="/World/Scenario/bench_sugar_box_100_anygrasp_agt_a1"` — the spawned
entity prim itself is the rigid body), and sets
`TINKER_SIM_CONTACT_TRACE_BODIES`/`_EXTRA_BODIES` to bare leaf names. Added
`_contact_name_matches(actor_path, names)`: compares the actor's leaf against
each configured name, accepting either a bare leaf or a full path on the env
side (falling back to that entry's own leaf) — the same leaf resolution the
pre-existing `ARM_CONTACT_BODIES`/`GRASP_CONTACT_BODIES` matching already
relies on structurally. `contact_report_first_event` only fires for a
monitored (ARM/GRASP/EXTRA) pair — a trace-only pair does not count as the
backend's first contact.

## 2026-09-06 — Task #25: bound `controller_reconciler`'s post-success teardown so it cannot wedge the launch chain

**Symptom.** In a live GPSR run (`gpsr_stack_logs/20260905T230818`), the
`controller_reconciler` instance spawning `joint_state_broadcaster` logged
`controller joint_state_broadcaster is active` at t=148.65s and then never
logged anything else, never exited, and never wrote `process has finished
cleanly` or `process has died`, for the remaining ~26 minutes of the run
(final teardown at t~1788671093, ~114 min of Isaac Sim's own internal
clock). Because `whole_robot.launch.py` / `gpsr.launch.py` chain
`xarm7_traj_controller`'s reconciler off this process's `OnProcessExit`,
that controller was never loaded and every arm goal was rejected for the
whole run.

**Root cause (not proven; best-supported hypothesis).** Full analysis in
`/home/tinker/.claude/jobs/01ca17b4/tmp/task25-reconciler-hang-findings.md`.
Key evidence:
- Everything in `controller_reconciler.py` up to and including the logged
  "is active" line is provably bounded by `time.monotonic()` deadlines
  (`RosControllerManagerApi._call`, `set_remote_parameter`), independent of
  `/clock` — the log line proves that code already finished. Task #21's
  `/clock` re-zero mechanism is unrelated: this node has no `use_sim_time`
  parameter and no wait in it is gated on sim time.
- The launch arguments for this exact `Node(...)` (controllers, timeouts,
  `--ready-node` absence) are byte-identical across `whole_robot.launch.py`,
  `manipulation.launch.py`, and `gpsr.launch.py` — the healthy bench bridge
  and the failing GPSR composite run the identical code path with identical
  arguments, so whatever differs is external to this file.
- This was the *only* process in the entire 578-line bridge log that never
  responded to SIGINT at teardown — every sibling node (safety_supervisor,
  contract_guard, xarm_facade, base_facade, pan_tilt_facade, etc.) printed a
  Python traceback (`ExternalShutdownException` / `RCLError`) proving its
  interpreter resumed and ran signal-handling code; this pid has exactly one
  log line in the whole file and never appears again, including through the
  full SIGINT teardown cascade. That pattern — silent, SIGINT-immune,
  indefinite — is characteristic of a process blocked inside a
  non-Python-interruptible C call, not a logical/Python-level hang.
- The only unbounded, opaque segment left after the logged success is the
  `finally` block: `node.destroy_node()` / `rclpy.shutdown()`
  (`controller_reconciler.py:326-327` before this fix), which descends into
  rclpy's C extension (`rcl_node_fini` / `rcl_shutdown` / rmw/Fast DDS
  participant teardown). The leading hypothesis is that this hangs inside
  Fast DDS participant-deletion under the GPSR composite's much larger,
  near-simultaneous DDS discovery churn (~29 bridge nodes + Nav2 + vision +
  manipulation all standing up participants within a few seconds) — a known
  class of ROS 2 Humble / Fast DDS issue. **This is not proven**: no live
  process was inspected (the investigation was read-only, no sim/stack
  runs). The confirmation step for the next wedge is `py-spy dump --pid
  <pid>` against the stuck `controller_reconciler` process — if the frame is
  in `destroy_node`/`shutdown` inside `rmw_fastrtps_cpp`, that confirms it;
  if it's still inside `reconcile_controller`/`_call`, that falsifies this
  hypothesis and points to the `time.monotonic()` deadlines somehow not
  firing instead.

**Fix (mitigation, not a native-layer fix): bound the teardown.**
`ros2_ws/src/tinker_sim_bridge/tinker_sim_bridge/controller_reconciler.py`
now runs `node.destroy_node()` + `rclpy.shutdown()` (in that order, same as
before) inside a daemon thread and joins it with a timeout —
`bounded_teardown()`, new helper, wired through `_run()`'s `finally` block
for both the success (rc=0) and failure (rc=1) paths, since both already
shared this one teardown call. Default timeout 5.0s, configurable via
`--teardown-timeout-s` / `TINKER_RECONCILER_TEARDOWN_TIMEOUT_S` env var
(`_teardown_timeout_default()` reads the env var, falling back to the
default on unset/blank/invalid values so a bad env var cannot break
startup). If teardown completes inside the bound, behavior and exit code
are exactly as before. If it does not:
- logs one line, `controller_reconciler: teardown did not complete within
  Ns after {success|failure}; forcing exit (rc={0|1})`,
- flushes stdout/stderr,
- calls `os._exit(rc)` (preserving whichever exit code the run already
  earned) instead of waiting further.

Two more log lines bracket every teardown attempt (`controller_reconciler:
starting teardown (bound Ns)` before, `controller_reconciler: teardown
completed` after a normal finish) so the next wedge — if it recurs — is
visible from the log alone, without needing `py-spy` just to know where the
process is stuck.

Also added a one-line readiness marker (`{label} succeeded; starting next
stage`, logged at `info`) in `_process_exit_actions()` in
`whole_robot.launch.py`, `manipulation.launch.py`, and `gpsr.launch.py`,
emitted right before a successful `OnProcessExit` hands off to the next
chained stage (e.g. right before the `xarm7_traj_controller` reconciler is
launched). No restructuring of the chain — this only adds observability.

**Tests.** `tests/test_controller_reconciler.py` gained unit tests for
`bounded_teardown()` with a fake teardown callable: prompt completion
returns normally with no forced exit; a teardown that blocks forever
(`threading.Event().wait()`, never set) hits the timeout, calls a
monkeypatched `exit_fn` instead of really exiting (recording the code), and
emits the expected log line — checked for both the success (rc=0) and
failure (rc=1) exit-code cases. Also covered `_teardown_timeout_default()`
(env var present/invalid/unset). Existing tests in the file were unaffected
except one that asserts the full parsed-args dict, updated to include the
new `teardown_timeout_s` field.

**Not fixed by this change:** the underlying native hang, if hypothesis 1
is right, remains unconfirmed and unaddressed — this only stops it from
wedging the launch chain. The user's explicit choice was to land the bound
now rather than block on live-repro instrumentation first. `py-spy dump` on
the next live wedge (see above) remains the confirmation step for the root
cause.

## 2026-09-05 — /clock re-zero on a full sim-process restart: anchored to a boot epoch (task #21)

**Root cause (findings: `task21-clock-rezero-findings.md`, `.claude/jobs/01ca17b4/tmp/`).**
`767fb89` (2026-08-27) made `/clock` monotonic across an *in-process*
`ResetSimulation` STOP -> PLAY (`backend.py` `_refresh_robot_handles`,
`simulation_time` re-anchors `_clock_step_origin` to the elapsed step count
observed before the boundary instead of re-zeroing). That fix does not, and
was never meant to, cover a full Isaac Sim *process* restart: a fresh
`IsaacWholeRobotBackend` re-initializes `_clock_step_origin = 0` /
`_clock_elapsed_steps = 0` (backend.py, `__init__`), and `ros_gateway.py`
stands up a brand-new rclpy node/DDS participant, so the new process's first
`/clock` samples are near `0.0` again -- a real backward jump relative to
the prior process's last published sample. Long-lived ROS consumers that
keep running across the sim restart see it: Python `tf2_ros.Buffer` (used
by, e.g., AnyGrasp) has no clock-jump handling at all (unlike the C++
`tf2_ros::Buffer`, which registers `onTimeJump`), so a backward `/clock`
sample is either cached as stale "old data" and dropped, or produces
`ExtrapolationException`, until ~10 s of new sim time re-elapses past the
old cached entries (`tf2::BufferCore`'s 10 s default cache window). This is
the same class of wedge `767fb89` fixed for the in-process case, one level
up.

**Fix (option (a) from the findings, chosen over publishing a boot-id for
consumers to clear their own buffers on): anchor the *published* clock to a
boot epoch**, so a fresh process's `/clock` never appears to precede a
prior process's last sample -- matching hardware parity (real ROS time on
hardware is wall-clock and never goes backward).

- `simulation/tinker_sim_isaac/backend.py`: new `resolve_clock_epoch(value)`
  parses `TINKER_SIM_CLOCK_EPOCH`: unset/`"wall"` (new default) ->
  `time.time()` captured once in `__init__`; `"0"` -> the legacy zero-based
  clock; any other numeric string -> a pinned epoch (seconds), for a
  harness that wants a reproducible absolute clock. The resolved value is
  stored once as `self._clock_epoch_s`. A new `ros_clock_time` property
  returns `self.simulation_time + self._clock_epoch_s` -- this is the value
  to publish/stamp with. **`simulation_time` itself is unchanged**: it stays
  the small, process-relative elapsed-steps value that internal consumers
  already depend on starting near zero (run-duration gating in
  `validation/run_sim.py`'s `args.duration <= 0.0 or backend.simulation_time
  < args.duration` loop guards, the base-hold timers, truth-record `t`
  fields) -- only the externally published clock needed to change. The
  767fb89 in-process STOP -> PLAY re-anchoring is untouched and composes
  with the epoch (it still re-anchors `_clock_step_origin`, which
  `ros_clock_time` builds on through `simulation_time`).
- `simulation/tinker_sim_isaac/ros_gateway.py`'s `_stamp()` (used for
  `/clock` and every outgoing ROS message header stamp, including camera
  frames) now reads `backend.ros_clock_time` (falling back to
  `backend.simulation_time` for a backend double that predates the
  property), so every ROS-visible timestamp this gateway produces is
  consistently epoch-anchored, not just `/clock` itself.
- `ros2_ws/src/tinker_sim_bridge/tinker_sim_bridge/contract_guard.py`'s
  `evaluate_clock_domain` readiness gate treated `clock_now_ns <= 0` as "sim
  clock hasn't advanced past zero" -- correct under the old zero-based
  clock (rclpy's own `TimeSource` convention: "Zero time is a special value
  that means time is uninitialized"), but under the new wall-clock-anchored
  default a real running sim's first sample is already a large nonzero
  epoch value, so the gate no longer needs (or should assume) a genuine
  zero reading ever occurs in normal operation. `clock_now_ns` is now typed
  `int | None`: `None` explicitly means "no clock sample received yet" (a
  caller that tracks this itself, distinct from a raw `Clock.now()` read);
  the literal `0` case is kept and still treated as not-ready for backward
  compatibility with any caller that can only observe the numeric value.
  `joint_state_probe.py`'s call site needed no change -- it passes a raw
  `Clock.now().nanoseconds` read, which already reads exactly `0` before
  any `/clock` message has ever arrived (rclpy's own uninitialized-clock
  convention) and a large epoch value once the new anchored clock is live.

**Grepped for and ruled safe (no change needed), per the findings' "verify
this" list:** `validation/gripper_close_probe.py --record-s` (a duration
parameter, `steps = args.record_s / DT`, never touches `simulation_time`);
`validation/run_sim.py`'s profile-emission `sim_time=getattr(backend,
"simulation_time", None)` (pure telemetry; unaffected since
`simulation_time` didn't change); the 2026-08-21 profiling-attribution note
in this log about `/clock` vs `wall_time` drifting by the bridge attach
time (predates `767fb89`, describes historical pre-fix behavior, not a live
assumption). `_sim_receipt_time`/`_sim_age_stale` in `ros_gateway.py` (used
for same-process command-liveness deltas) were left reading the raw
`simulation_time`, not `ros_clock_time`: a constant epoch offset cancels
out of a same-process delta, so this is deliberately unchanged rather than
switched for consistency's sake.

**Known limitation, not fixed here:** the wall-clock default only
guarantees the new process's first sample is >= the old process's last one
when real wall-clock time elapses across the restart at least as fast as
sim time did within the old process (i.e. the sim was not running many
times faster than real time right up to the moment of the restart, and the
restart itself takes nonzero wall time) -- true in practice for an Isaac
Sim boot (tens of seconds) but not a mathematically airtight guarantee for
an arbitrarily fast headless sim killed and relaunched in well under a
second. A stronger guarantee would need cross-process persistence (a state
file) rather than a wall-clock anchor; deferred as the findings' option (a)
scoped it as wall-clock-only, matching hardware parity as the acceptance
bar rather than absolute mathematical monotonicity.

Tests: `tests/test_manipulation_runtime.py` (`resolve_clock_epoch`
defaults/legacy-zero/numeric/rejects-garbage; `ros_clock_time` adds the
epoch without perturbing `simulation_time`; two backends constructed
back-to-back with `time.time` monkeypatched to advance publish
non-decreasing clocks; the 767fb89 in-process reset still holds with an
epoch anchored) and `tests/test_integrated_joint_state_contract.py`
(`evaluate_clock_domain` treats `None` as not-ready and a large epoch value
as ready, alongside the pre-existing zero/missing-publisher cases).

## 2026-09-04 — Wrist camera blackout band: the camera was rendered from inside the gripper housing

**Symptom (grasp bench, sensor-rich profile, `TINKER_SIM_WRIST_CAMERA_AIM=tool-forward`).**
Every wrist frame carried a curved black band along the TOP edge, growing
toward the right: first non-black row 0 at x<=212, 40 at x=318, 56 at
x=424, 68 at x=530, 76 at x=636, 80 at x=742, 81 at x=847 — 9.4% of the
848x480 image, present in color and aligned depth alike, fixed relative to
the camera in every arm pose (close scan, wide scans, approach, carry).
Pixels were unlit (mean grey 1.1, half exactly 0), and the aligned depth
inside the band read a NEAR-RANGE 50–60 mm, not 0; deprojected through the
URDF optical frame it landed on `xarm_gripper_base_link` at z ≈ 0.07.
AnyGrasp's depth mask dropped those pixels (row 5: 241/848 valid). A
brighter patch on the right edge came with it. Task-list item #18.

**Reproduction without a GPU.** An offline USD frustum model (pure `pxr`,
no Kit: project every `/visuals/` mesh of `robot.usd` through the rig's
camera — 848x480, 69.4° HFOV, the rig's 0.05 m near clip — placed exactly
as `CameraRig.initialize` places it: mount prim xform · orient(correction ·
`mount_rotation_wxyz`)) reproduces the live band to within a row:

| | live (bench) | offline model |
|---|---|---|
| occluded fraction | 9.4% | 9.6% |
| first clear row @ x=318/424/530/636/742/847 | 40/56/68/76/80/81 | 40/57/69/77/81/82 |
| depth in band | 50–60 mm | 50–66 mm |
| occluder | (deprojects to gripper base) | `xarm_gripper_base_link/visuals/base_link/mesh` — the ONLY hit |

With no aim correction (parity) the model shows zero self-occlusion, so the
band is a product of the `tool-forward` tilt — but the tilt was only the
proximate cause.

**Root cause: the description mounts the wrist camera at a placeholder.**
`tk26_sim/src/isaac_bringup/urdf/tinker_full.urdf.xacro` (the sim
description the robot artifact is built from) layers Intel's
`sensor_d435` macro onto `link_eef` with `<origin xyz="0 0 0" rpy="0 0 0"/>`.
The Intel macro then adds its own bottom-screw → camera_link offset
(0.0106, 0.0175, 0.0125) and the colour frame's +0.015 in Y, so the colour
optical frame sits at link_eef + (0.0106, 0.0325, 0.0125): 12.5 mm above
the flange, 33 mm off the tool axis — i.e. on the surface of the gripper
base housing — looking along +X_eef, 90° off the tool. (That 90° is the
defect `tool-forward` was written for; see 2026-08-31.) The real robot
(`tinker_real.urdf`, xArm's `realsense_d435i.urdf.xacro`, "vendor
factory-nominal" extrinsics with the per-robot hand-eye override hook)
mounts `xarm_camera_link` on the D435 cam-stand bracket at link_eef + xyz
(0.06746, −0.0175, 0.0237) rpy (π, −π/2, 0): 67 mm out along +X_eef, 24 mm
up, looking straight down the tool axis with image-up radially outward.
Tilting the placeholder camera 60° toward the tool IN PLACE swings the
housing's far wall into the top of the frustum, just past the 0.05 m near
clip — hence "black, 50–60 mm". The bright right-edge patch is the lit
exterior of the same housing / left finger.

**Ruled out / not the cause.** Rendering settings (the head camera under
the same profile has no band); the annotator/CUDA-700 class of stale-read
faults (the band is geometrically stable and depth-consistent); the
`mount_rotation_wxyz` fix e2996f8 (parity pose shows no self-occlusion);
lighting (unlit because it is the inside of a closed mesh).

**What would also have "worked", and why not.** In the model, dollying the
tool-forward camera forward by as little as 10–20 mm along its view axis
already clears the housing (it was only 0–16 mm past the near plane), and a
50 mm shift up the optical −y does too. Both keep the render origin inside
the gripper assembly and keep the 60° in-place tilt that was itself a
sweep-picked compromise ("30° shy of the tool because a tool-aligned view
stares into the co-axial hand" — true only because the camera was
co-located with the hand).

**Fix: render from where the real camera is (`cam-stand`).** A second wrist
preset, `TINKER_SIM_WRIST_CAMERA_AIM=cam-stand`, places the render camera
at exactly the vendor cam-stand pose: `CAM_STAND_MOUNT_OFFSET_XYZ` =
(0.065, −0.0112, 0.05686) is the bracket translation and
`CAM_STAND_CORRECTION_WXYZ` = (0, 0, −√½, √½) the rotation, both in the
artifact's (placeholder) optical frame — `inv(T_artifact_optical) ·
T_vendor_optical`, which `tests/test_wrist_camera_aim.py` re-derives from
both URDF chains so the constants cannot drift from the geometry they
claim. Rendered from there the model shows ZERO housing pixels; only the
fingertips at the bottom edge (~750 px, 0.2%) — what a wrist camera sees.
Authoring the exact op sequence the rig produces onto `robot.usd`
(`AddTranslateOp(op0)`, `AddOrientOp`) lands the origin at link_eef +
(0.06746, −0.0325, 0.0237), view +Z_eef, image-up +X_eef, to 4e-8.

Mechanics: `CameraStreamSpec.mount_frame_offset_xyz` (new; default zero)
is a translation in the MOUNT prim's frame, authored as a translate op
listed BEFORE the orient op — USD applies the last-listed op to the
geometry first, so a translate before orient stays in the mount's axes
while the existing `view_axis_forward_offset_m` dolly (after orient) stays
in the camera's. Listing the bracket after the orient would rotate it with
the aim and put the camera straight back on the housing. `camera_xform_ops`
returns the op list as data so the order is unit-tested without Kit
(`tests/test_camera_rig.py`). `tool-forward` is kept as the A/B baseline;
`scripts/gpsr-stack` now hands the sim stage `cam-stand`.

**TF has to move with the pixels.** The artifact URDF still carries the
placeholder joint, so `robot_state_publisher` would put
`xarm_camera_color_optical_frame` 6 cm and 90° away from where the pixels
were rendered — every consumer deprojecting through TF (AnyGrasp's desk
plane fit, detections in base/map) would be wrong by construction, and the
bench had been compensating with a private `_aimed` alias frame.
`tinker_sim_deploy.runtime.sim_robot_description` = `topic_control_description`
plus a rewrite of `xarm_camera_joint`'s origin to (0.05496, 0, 0.0131) rpy
(π, −π/2, 0) — `T_vendor_camera_link · inv(T_intel_bottom_screw)`, which leaves
the Intel chain above it untouched and lands the colour optical frame on the
vendor pose. It is keyed on the SAME env value the sim stage reads
(`TINKER_SIM_WRIST_CAMERA_AIM=cam-stand`; any other value is a no-op), all
four bridge launches (gpsr, manipulation, whole_robot,
integrated_ompl_manipulation) call it, and `gpsr-stack` exports the value to
the bridge stage too. `tests/test_wrist_cam_stand_description.py` checks the
rewritten chain's FK equals the render pose exactly. A custom stack must set
the variable for BOTH the sim and the bridge or TF and pixels disagree.

**Live result (GPU0 relaunch of the bench recipe, same measurement tool on
both).** Relaunched the shared GPU0 sim from this branch with the bench's exact recipe (`launch_isaac_bench.sh`: sensor-rich, rcw2026, seed 7, spawn (−2.99, 3.80), `TINKER_SIM_CAMERA_HZ=4`) and only the wrist preset changed. Same subscriber-side metric on both (first non-black row per column, top-edge-connected black region, aligned-depth stats):

| | tool-forward (before) | cam-stand (after) |
|---|---|---|
| top-edge black band | 6.7% of the frame, 7/7 frames | 0 px, 8/8 frames |
| first non-black row @ x=318/636/742/847 | 39/76/80/81 | 0/0/0/0 |
| aligned depth inside the band | 50–64 mm | (no band) |
| camera_info frame / fx | `xarm_camera_color_optical_frame` / 612.3 | unchanged |

The after-capture is at the boot pose, where the tool-aligned camera looks at the robot's own chassis and lidar dome at close range (median depth 75 mm) — the home-pose view, not the housing; the housing band is pose-independent and gone. At the bench's four scan joint poses the URDF-FK-reposed frustum model gives tool-forward 9.6% (housing) vs cam-stand 0.1% (356 px of fingertip at 110–138 mm along the bottom edge) in every pose. Frames: `wrist_baseline_toolforward_color.png` / `wrist_camstand_color.png` in the job tmp dir; the grasp bench session is re-measuring at its scan poses against the plain optical frame.

**Second round (same day): the first cam-stand authoring froze the wrist render.**
The grasp bench's first live check of `cam-stand` at its wide scan pose
showed the robot's own chassis and lidar dome filling the wrist frame with
5–8 cm depths, which read as "the camera looks back along the tool". It
did not: that frame was pixel-for-pixel my boot-pose capture from 40
minutes earlier (mean difference 6.6 grey levels, 4% of pixels over 20 —
AA dither), while the spectator camera showed the arm at the scan pose;
the bench then moved to the close-scan pose and the wrist frame again
changed by dither only. The RTX render product had stopped tracking the
arm the moment `cam-stand` was on: every frame was the boot frame.

The one thing the first authoring changed on the prim was a SECOND,
suffix-named translate op — `xformOp:translate:op0` listed before
`xformOp:orient` (a distinct suffix is required to author two translates,
and the head's dolly already used the plain name after orient). Every
camera that tracks fine (head with its dolly, wrist under `tool-forward`)
carries only standard-named ops. The pose math was never wrong: the USD
`XformCache` result for the authored ops matched the vendor chain to 4e-8
both times — the static stage composes the suffixed op correctly, the
renderer's live hierarchy evidently does not. Suffix vs. non-standard
order was not separated (each test costs the shared sim a restart); the
fix removes both. `camera_xform_ops` now folds every offset into ONE
standard `xformOp:translate` listed before `xformOp:orient` (plain TRS
order): the mount-frame bracket offset as is, the view-axis dolly rotated
by the mount rotation into the mount frame first (`R · (0, 0, −d)`), which
is exactly what "translate listed after orient" composed to. Head
level-forward pose under the fold vs. the old `[orient, translate]`: 0.0
difference in USD. Rule for the rig from here on: standard op names only,
at most one translate, orient last.

Live after the second relaunch: boot-pose capture, 8/8 frames, top-edge band 0 px, 0.0% black pixels anywhere, depth 100% valid with median 0.568 m; the frame differs from the frozen one by a mean of 120 grey levels (70% of pixels over 20) and shows the desk ahead with the two fingertip pads just entering the bottom edge, the framing the frustum model predicted. Grasp bench verification at its two scan poses (dccfdff): the frame follows the arm (wide-scan vs. frozen frame: mean difference 121.6 grey levels, 70% of pixels over 20); first non-black row 0 at all nine sampled columns at both poses, 0.0% dark pixels, nothing dark along the bottom edge either; depth 100% valid, median 0.861 m (wide) / 0.571 m (close). Desk-plane fit deprojected through the plain vendor optical pose, no in-plane flip: wide pose 48% of desk-footprint points in a plane at z 0.7328 m, tilt 0.12°, residual 1.2 mm; close pose 72% at z 0.7334 m, tilt 0.08°, residual 0.8 mm (desk top is 0.734 m). Flipped variants fail (3–5% in a 40° tilted plane), so image rows/cols match the camera_info convention. Operator trap recorded by the bench: a stale +60° alias publisher from an older recovery script re-latched over the new alias and put the first fit 13 cm low — kill old static_transform_publishers before re-latching.

**Still open.** The durable fix is the description: give the `sensor_d435`
instantiation in `tinker_full.urdf.xacro` the cam-stand origin (or restore
xArm's `add_realsense_d435i` mount) and republish the robot artifact, after
which both `cam-stand` and the TF rewrite become no-ops and hardware parity
is restored rather than broken. Until then the preset is opt-in and
sim-only like the head's `level-forward`. The head camera has the same TF
gap (its `level-forward` correction is not in TF; the alias is identity) —
not addressed here.

## 2026-09-02 — bench retention follow-ons: knife and plate are grasp-geometry, not sim bugs

The mimic fix (below) turned the bench's first physically retained grasps
(bottle held 2/3 at pure defaults, the campaign's first). The two remaining
0/3 objects — knife and plate — were run down in the headless probe with the
fixed backend and contacts ON; neither is a gripper-physics fault.

**Aperture-vs-drive map (URDF FK, verified against the sim).** Pad-face gap by
drive angle: 0.00→89 mm, 0.22→69 mm (bottle), 0.43→99 mm finger-origin sep,
0.54→36 mm, 0.61→30 mm, 0.85→~6 mm. And the fingertips travel ~13 mm along the
tool axis over a close (parallelogram arc): 3 mm short of the TCP plane at open,
~9 mm PAST it near full close. So every top-down grasp height must add the arc:
the commanded TCP sits ~9 mm above where the fingertips actually end.

**Plate — infeasible geometry on the solid-disc asset (probe18).** The bench
`bench-plate.usda` was a solid cylinder r=0.10 h=0.025 lying flush on the desk.
A top-down radial rim pinch cannot grip it: with the jaw open 140 mm and closing
along the radius, the inner pad lands 74 mm IN from the near rim, flat on the
solid top face, and the arm's descent jams it there at ~120 N before the close
starts; the outer pad closes through air and stops 32 mm short of the rim (7 N
graze). The single-DOF linkage halts on the jammed knuckle at drive 0.36 and
nothing is pinched (lift retains 0 N). The bench read 0.54/zero-force because
`/sim/parity/finger_contact` is structurally silent in the sensor-rich profile,
so the 120 N jam was invisible; those descent jams are also the source of the
>120 N force-trace spikes (fingertip-on-rigid-surface strikes, not close
punches — the close itself never exceeds ~20 N with the mirror fix). A side/edge
pinch is equally blocked because the flush disc has no clearance beneath the
rim. A real deep-plate asset (foot ring raising a shallow bowl with a raised
rim, so a top-down rim pinch has clearance beneath) was authored and probed —
and it revealed a deeper truth: **the plate is not the asset, it's the gripper
model.** Across four rim geometries (6/16/25 mm walls, both jaw-centre biases)
and with compliant contact enabled, a top-down rim pinch loads only the
drive-side pad (~10 N) and the follower pad never engages (~1 N), so it never
holds. The reason is the single-DOF mirror doing exactly what the mechanism
says: the drive joint is the LEFT outer knuckle and the five followers track its
MEASURED angle through a rigid coupling, so the instant the left pad jams on the
rim's outer face the whole jaw freezes — before the right pad has closed the
last few mm onto the inner face. Small objects trapped symmetrically between the
pads (bottle 69 mm, knife 30 mm) load both sides and hold; a large object
gripped at a local off-centre rim loads one side. This is faithful to a rigid
single-motor gripper, not a bug (the OLD target mirror would load both pads here
precisely because it drove the followers independently — the same
independence that curled the pads and punched the bottle). Consequences: the
plate needs either a grasp that traps it symmetrically (hard for a 200 mm disc),
or the drive modeled as a CENTRAL actuator so a blocked finger doesn't halt its
partner (a URDF/backend change, a follow-up), or removal from the goalset. The
deep-plate asset and the four probes live in the session's evidence; the shipped
`bench-plate.usda` is unchanged pending that decision.

**Knife — graspable across the width; the failures were candidate + facade
(probe19).** A correct top-down centre pinch with the knife's 30 mm width across
the closing axis holds it 3/3: descent clean (pads straddle the width, no jam),
close stalls at drive 0.61 (=30 mm), pads bite the two sides (18 N / 8 N), lift
raises the knife +94 mm with the TCP at ~15 N hold. The bench's failures: (a)
candidate idx 0 sits 60 mm off-centre and closes on air — drive runs 0→0.825 at
full speed then sits dead flat, the exact signature in the bench mimic trace;
(b) the settled knife yaw is not guaranteed to land the 30 mm width across the
closing axis (a spawn-rotation quirk the benchmark owns); (c) the running bridge
facade on the main checkout (task50-stage-a-repair @ d8cc0ff) predates the
contact-free position-stall path (`stall_dwell`, added on task-sim-bugfixes at
a46d108), so an air-close never latches `stalled=true` and instead times out
three times as "native gripper: execution failed" (~111 s = 3 × the 5 s sim
timeout at sub-1.0 RTF). Benchmark-side geometry + a facade heal on the main
checkout; not the sim's gripper.

**Community cross-check.** Isaac Sim's own closed-loop tutorial (Robotiq 2F-85)
breaks the parallelogram loop and drives followers via the PhysX Mimic Joint API
referencing the drive joint's state — never by copying a commanded target — and
ros2_control's gripper action controller aborts on stall by default
(`allow_stalling: false`), which is the generic shape of the facade abort. Both
corroborate the fixes here.

## 2026-09-02 — close-phase punch root cause: the mimic mirror copied the TARGET, not the drive's angle

Closes Task #19 at the source. Five reactive/solver-level cycles (below) treated
the first-contact spike as a control or contact problem; it is a kinematics
problem. The xArm gripper (UFACTORY manual V1.11.0: one actuator, 84 mm stroke,
30 N max clamping force) is a single-DOF mechanism — one motor on the left
outer knuckle, the right knuckle gear-coupled, each finger on a parallelogram
that keeps the pad parallel. The URDF says exactly that: all five follower
joints carry `<mimic joint="drive_joint" multiplier="1" offset="0"/>` (finger
joints on `-x` axes = counter-rotation = parallel pads; the importer baked
those axes as 180° frame flips, so a uniform +1 mirror is kinematically right).
URDF mimic semantics are `q_follower = q_drive` — the driving joint's ACTUAL
angle. `_mirror_gripper_mimic_targets` copied drive_joint's commanded TARGET.
Identical in free motion; wrong the instant the object blocks the drive
knuckle, when the followers keep chasing the far target as five independent
k=1500 motors.

Measured in-process (headless probe, CPU PhysX, bench bottle centred at the
bench's recorded grasp pose, stock defaults): the k=200 drive side lags the
k=1500 followers 0.06 rad even in free motion, so the right pad always arrives
first; after the drive stalls at 0.43 rad the right outer knuckle runs on to
0.65 and shoves the bottle into the weak left pad (right 25 N vs left 5 N,
bottle tilted 12°, held only by hooking); and the finger joints run to 0.845
regardless — the pads CURL 0.2–0.4 rad about the finger axis. That curl is the
bench's "pads close through to 0.728, 18 mm past the knife"; on a desk-lying
knife it drives the fingertips into desk/knife, the 200+ N spikes.

Two earlier readings are corrected by the same data. (1) The d·v "preload"
hypothesis is real but not the ejector: free closes give k·lag = d·v within 3%
in every config (defaults: 73 N·m per follower entering contact; slew 0.75 →
37; d=20 → 22; k=500 → 60, i.e. lowering k only grows the lag), yet on a
centred bottle the stock jaw peaks at only 33 N and LOWER damping made
retention worse (d=10: 67 N, dropped). isaaclab's `applied_torque` reports
the net (spring − damper ≈ 0 in motion), which is why the preload was never
visible. (2) The dev-log's "drive joint is unloaded" premise is false:
`drive_joint` → `left_outer_knuckle` → `left_finger` carries the left pad.

Fix: followers target `q_drive + q̇_drive·dt` (measured angle plus one control
step of feed-forward so their one-step lag does not drag the drive; at stall
q̇ ≈ 0 so they hold the drive's angle exactly). A blocked knuckle now stops the
whole linkage, the pinch is symmetric, and drive_joint's actuator — its effort
limit, i.e. the facade's `max_effort` (50 → the USD maxForce; ≈20 N total pad
force, vs the 30 N spec) — is the true grip bound. The stall-gated lead clamp
(06cc2e1) is retired to default-off: with mimic-correct followers it has
nothing to bound, and its gate self-locks the close into a 0.1 rad/s crawl
(pads trail the drive by one step, so pad speed sits on the gate; a 30 mm knife
was not reached in 3 s). `TINKER_SIM_GRIPPER_MAX_LEAD_RAD` keeps it available.

Validation (same probe, bench objects, bench grasp poses, lift = joint2
−0.15 rad, retention = object rises with the TCP): bottle 7/7 retained
(peak = hold ≈ 20 N, tilt ≤ 6°, contact at 0.18 s) across slew 1.5/0.5,
follower k 1500/500, clamp on/off; knife (top-down, fingertips 15 mm above the
desk) 3/3 retained (peak 20 N, hold 19 N, no displacement) vs 0/5 with the
stock mirror (46–49 N peak, drive 0.42 while pads curl to 0.845, apparent
30 N "hold" that vanishes on lift — the bench's signature). Unit test:
`test_gripper_mimic_followers_track_measured_drive_angle` (red on the old
mirror, green now).

Probe lessons worth keeping: a static support column wider than the bottle
footprint spawned INTO the gripper hulls and exploded the articulation (drive
−32 rad) before any close — use a footprint-sized pedestal and gate every
trial on "no contact pairs, followers quiet, drive in range"; the kinematic
base hold latches at sim t=2 s, which on the bare ground plane caught the
0.2 m spawn drop mid-tumble (root z 0.218, 40° tilt) — latch after ~8 s;
`body_quat_w` comes through as XYZW here (the backend's `root_state` reorders
the same way) — decoding it as wxyz put the knife 139° off; the finger pad
runs 0–61 mm from the finger joint along the tool axis with the TCP plane at
64 mm, so a top-down pinch of a 25 mm object needs the fingertips within
~10 mm of the desk. One more, unexplained and worth its own round: on the
bare ground plane (no arena) the probe's boot LAUNCHES the robot — root +12 cm
in the first physics step, head tilt and finger joints at 30–70 rad/s against
their effort caps for ~0.3 s, base airborne to z=1.7 m and down 2 m away,
sometimes on its side — with the safety stop held or released, at spawn_z
0.20 or 0.09. The arena stack never shows it (the bench's base stays at its
spawn xy). The probe works around it (settle 8 s, re-latch the base hold
upright at the origin, then release the safety stop); the cause (a spawn-time
state violation — epsilon-mass frame links? the zero-mass link_tcp with an
undefined centre of mass? ground-plane placement?) is open. Follow-ups: the
exact model is a PhysX loop-closure joint
between inner knuckle and finger (NVIDIA: no native closed loops; the mimic
API is reported broken for parallel grippers on Isaac 5.1), which would make
the parallelogram passive; and sizing the drive effort limit to the 30 N spec.

## 2026-09-01 — close-phase punch, and why the first (open-loop) ramp made it worse

With the mimic coupling stiffened to k=1500 (the fingers finally grip), the
grasp-bench round found a new retention failure: every object — light ones
worst — was ejected from the jaw at *first pad contact*. The finger-contact
force trace showed a spike of 12–190 N on the first sample, before the grip
settled: the k=1500 followers applied the full commanded close target in one
step, so on first touch the position error (and thus `k*error` press) was at
its maximum. A punch, not a squeeze.

First attempt (`21d744c`): slew the applied drive target toward the command at
`_gripper_close_slew` rad/s instead of jumping to it, so the press builds
gradually. It made things worse on two axes, and the reason is the same on
both. An open-loop slew bounds `dF/dt` but never *stops* — past first contact
the target keeps advancing to the fully-closed command. (1) The follower press
therefore still climbs to the effort caps; the measured peak rose to 248 N,
worse than the unramped 190. (2) More subtly, it broke a previously-green path:
all three knife grasps began aborting as "native gripper: execution failed".
The gripper facade keys success on the *measured* joint position vs the fixed
goal (see the entry below): a close that stalls short of the goal is a success
only if it *stalls* — either fresh contact force, or measured position no
longer improving for `stall_dwell_s`. The open-loop ramp keeps the target
creeping, so the followers keep deepening and the measured position keeps
inching down by more than `stall_epsilon` every dwell window — the position-
stall detector never latches, contact-stall alone can't carry the thin knife,
and the close runs out its 5 s `simulation_timeout_s` and aborts. The ramp
defeated the very stall detector the entry below had just added.

Fix (`8cde40f`): make the ramp closed-loop — FREEZE the applied target once
finger-pad grip force reaches `_gripper_contact_halt_force`
(`TINKER_SIM_GRIPPER_CONTACT_HALT_N`, default 15 N). Freezing caps the press
near the halt force instead of the effort caps, and — because the target stops
moving — the measured position flatlines, so the facade's stall (contact and
position both) latches and the grasp reports success. The halt reads the same
quantity the facade sees on `/sim/parity/finger_contact` (the sum of the two
finger-pad normal forces, via `contact_state`), so the halt point and the
facade's contact threshold agree by construction — when the sim stops pressing,
the facade sees exactly that force. Only the closing stroke is force-bounded;
an opening command always slews freely so release stays prompt. `slew <= 0` and
`halt <= 0` each disable their own stage.

The 15 N default was sized off the force trace: good grips form in a 13–41 N
band before the runaway climbs 41 → 100 → 249, and the two grasps that *did*
hold settled at 31/36 N.

Cycle-2 (knife-only) showed the force halt was the wrong instrument, for a
reason that matters: a force threshold on the **contact-report** signal is
unreliable for exactly the object that needs it. Knife completion went 0/3 →
1/3 (the freeze-lets-the-facade-latch direction is right), but the one that
completed **froze at drive 0.148** — a single transient brush during approach
crossed 15 N and latched the halt ~73 mm open on a 30 mm knife, no real pinch —
while a 212 N peak persisted on the aborting closes. Both are the same defect:
a thin object reports contact **sparsely and spikily** (the knife baseline is 3
blips over an entire close), so a force latch both fires on a lone transient and
misses a fast spike that ejects the object within the one-step lag between
setting the target and reading the next contact report.

The fix (bounded lead) drops force sensing from the stopping decision entirely.
Instead of freezing on contact force, cap how far the applied target may lead
the **measured** pad position: `applied = min(slewed, pad_measured + max_lead)`
while closing, where `pad_measured` is the least-closed of the two finger
joints. Follower press is then `k * (target − pad_measured) <= k * max_lead` by
construction — a hard force bound with no runaway — and the moment the pads
stall on the object the target clamps at `pad + max_lead` and stops, so the
unloaded `drive_joint` the facade watches flatlines and its position-stall
latches. It reads measured position, not contact reports, so it is robust for a
thin object; and being a continuous clamp rather than a one-shot trigger, a
transient brush cannot latch it (the brush doesn't stall the pads). The drive
joint is unloaded — the pads carry the object, not the drive — so the drive's
own position can't see the stall; that is why the clamp reads the pad joints
specifically. `max_lead` (`TINKER_SIM_GRIPPER_MAX_LEAD_RAD`, default 0.015 rad)
is the grip-force knob: `k * lead ≈ 1500 * 0.015 ≈ 22 N`. The old force cap
(`TINKER_SIM_GRIPPER_CONTACT_HALT_N`) is kept but defaults **off**, since it was
the source of the transient latch.

Cycle-3 showed the naïve always-on clamp has a fatal flaw of its own: it went
0/3, every knife close **timed out with the drive back at 0.0 — jaw fully
open**. The clamp ratchets the target *backward*. The `lead ≈ press/k`
relationship only holds *quasi-statically*; while the pads are moving,
`target − pad` is the **dynamic tracking lag** (the peer measured 0.02–0.03 rad
in motion, vs ±0.004 rad settled), which is *larger* than the 0.015 lead. So
during the approach `pad + lead = target − 0.025 + 0.015 = target − 0.01`, and
`min(slew, pad+lead)` drives the target down 0.01 every step until the jaw sits
fully open and the facade times out. The unit test had masked this by faking
zero-lag pads — an unphysical `pad == target`.

The fix gates the clamp on stall: apply it only once the pads have nearly
stopped (`max pad speed ≤ _gripper_stall_speed`, default 0.1 rad/s;
`TINKER_SIM_GRIPPER_STALL_SPEED`). Free-close pad speed is ~the slew rate (1.5),
so the gate cleanly separates moving from stalled. While the pads move the clamp
is off and the target slews freely (no backward ratchet); the instant they
stall on the object, speed drops through the gate, the dynamic lag has decayed
to the settled ±0.004, and the clamp bounds the now-quasi-static press at
`k * lead`. `max(current, …)` additionally keeps the close monotonic so the
clamp can never retreat the target even at the moment the gate flips. `slew <= 0`
disables the ramp; `lead <= 0` disables the clamp. The `PhysxMaterialAPI`
compliant-contact spring (`TINKER_SIM_GRIPPER_COMPLIANT_STIFFNESS`) remains as a
softer-first-touch escalation, off by default. Not yet live-validated past
cycle-3 — reasoned safe (startup, slow legitimate motion, one-step velocity lag,
and missing-signal all degrade to bounded press or plain slew, none deadlock);
needs a live trace to confirm no >20 N peak, knife grasps complete without
abort, and the grip holds through the lift.

Cycle-4 (stall-gated clamp) is the decisive dataset, and it closes the reactive
approach entirely: 1/3 completed (drive 0.728, 18 mm *past* the knife), 2/3
timed out, and the force bound broke — three single-sample spikes of 118/228/
221 N against the ~22 N `k*lead` ceiling. The spikes land **during pad motion**,
i.e. while the stall gate is off by design, at first contact. At k=1500 and
1.5 rad/s the finger carries enough momentum that first contact with an 80 g
object is resolved impulsively **inside a single physics step** — the object is
ejected/displaced before the pads can slow, so no stall ever forms (the pads
close through to 0.728, or the facade times out on the disturbed geometry).

The conclusion across all four cycles: **any reactive stop scheme is
structurally one step too late for a light object.** The ejection happens inside
the very step the scheme is waiting to observe — a force latch (cycle-2) and a
velocity/stall gate (cycle-4) both see it only after the fact. The dominant term
is not the PD spring `k*error` but the rigid-contact **collision impulse**: the
solver resolving the moving finger's momentum into the light object in one step.

That moves the fix out of the control loop and into the solver step itself.
The mapped lever ladder, in order:
  1. **Compliant contact on the pads** — the primary. `PhysxMaterialAPI`
     compliant-contact spring (`TINKER_SIM_GRIPPER_COMPLIANT_STIFFNESS`, with
     `_DAMPING`; already wired in `_apply_gripper_friction_material`, off by
     default). It softens the *contact constraint* so the collision impulse is
     spread over several steps instead of one — the only place a one-step event
     can be tamed. It also buys the stall-gated clamp the steps it needs to
     latch, so the two are complementary (compliance softens the impact, the
     clamp bounds the steady hold). Acceleration-spring is on, so the stiffness
     is mass-normalized; a first-cut in the 1e5–1e6 range is the place to start
     an A/B, tuning down until the >20 N spike is gone and up until the grip
     still holds.
  2. **Soft-close k** — drop the mimic follower stiffness during the closing
     phase (k is the impulse multiplier) and restore k=1500 only after settled
     contact. k=200 alone fails the *hold* (e320d5b: the object extrudes at
     0.13–0.17 rad lag), so it must be phase-switched, not lowered outright. The
     runtime write path exists (`write_joint_stiffness_to_sim_index`, as the
     safety hold uses); the restore trigger is the same stall gate (not time-
     critical for the hold). Not yet implemented — it is runtime gain-switching
     against the actuator model and the mirror, so it wants a GPU round to
     develop, not a blind commit.
  3. **Slower slew through the contact band** — reduces the finger momentum at
     impact (`TINKER_SIM_GRIPPER_CLOSE_SLEW`). Bounded by the facade's 5 s
     timeout (a global 0.15 rad/s close would overrun it), so it is an adjunct
     to (1)/(2), not a standalone fix.

Reactive tuning is done: `21d744c/8cde40f/1f6f124/06cc2e1` are all on the branch
as the instrumented record, `06cc2e1` (stall-gated clamp) is the head and the
right *hold*-phase bound, but the *impact* must be solved at the solver level.

**Cycle-5 (compliant contact, VALIDATED lever).** The A/B ran at
`COMPLIANT_STIFFNESS=3e5` / `_DAMPING=1e3` (compliance confirmed engaged, no
`gripper_compliant_error` in the boot log). Peak first-contact force
**228.4 → 156.6 N (−31%)**, and — the important part — the trace *character*
changed: sustained mid-range samples (40/60/59/45/90 N) replaced cycle-4's
isolated 118–228 N extremes, i.e. the collision impulse is genuinely spreading
over steps rather than resolving in one. Lever 1 is the right one and the
response is monotonic — a two-point curve now exists (rigid 228 N
`force-trace-gate.txt` → 3e5 gives 157 N `force-trace-compliant.txt`, both in
`results/2026-09-01-retention-campaign/part3-evidence/`). Completion held at
1/3, held 0/3: 3e5 alone doesn't yet clear retention. Two follow-ons, both env
knobs (no code change — the levers are all exposed):
  - Go **softer**: `COMPLIANT_STIFFNESS=1e5` next (expect roughly proportional
    softening toward the <20 N target); add lower `CLOSE_SLEW` through the
    contact band if compliance alone stalls above 20 N.
  - **Retune the stall gate for compliant contact.** The one completing close
    overshot to drive 0.458 (~10 mm past the knife) because the hold-phase clamp
    latched late: under compliance the pads decelerate *gradually* rather than
    stalling sharply, so the rigid-sized `_gripper_stall_speed = 0.1 rad/s`
    catches the stall too late. Raise `TINKER_SIM_GRIPPER_STALL_SPEED` so the
    clamp latches earlier on the gentler deceleration (watch it does not latch
    during the free-close velocity ripple). This is the coupling between the two
    fixes: compliance changes the deceleration profile the hold-phase gate keys
    on.

Next window: `COMPLIANT_STIFFNESS=1e5` with the two-point curve as the guide,
then raise `STALL_SPEED` to kill the hold overshoot; reassess soft-close
(lever 2) only if compliance + slew can't reach <20 N with a holding grip.

## 2026-09-01 — contact-free gripper stall (grasps aborting in the sensor-rich profile)

The grasp benchmark's real closes were all aborting as "native gripper:
execution failed" — every close that physically stalled on an object timed
out instead of succeeding (run went 0/3 on bottles). Root cause is a
profile-parity gap, not a grasp-planning fault. The gripper facade
(`ros2_ws/src/tinker_sim_bridge/tinker_sim_bridge/gripper_facade.py`)
recognizes a successful grasp two ways: the finger reaches its commanded
position (`reached_goal`), or it stalls short of it against an object
(`stalled`). The stall test required a *fresh contact force* >=
`contact_force_n` on `/sim/parity/finger_contact`. But the sensor-rich
profile — the one that actually runs GPSR grasps — builds its backend with
`enable_contacts=False` (`validation/run_sim.py`), so that parity topic is
structurally silent there. `manipulation-core` *enforces* contacts
(`run_sim` raises "must enable contacts"); sensor-rich disables them, most
likely for RTF at camera cadence. So a real grasp in sensor-rich stalls the
fingers on the object (correct physics), produces no contact telemetry, and
the facade can only time out.

Why not just turn contacts on in sensor-rich: `activate_contact_sensors`
(`backend.py`) is an all-or-nothing flag on the whole `/World/Tinker` spawn
and the contact-report subscription is robot-global — there is no
finger-only contact path today, so it would mean paying robot-wide contact
reporting in the camera-bound loop. And it is unnecessary: the *only*
real-time consumer of `finger_contact` is the facade's stall test. The
qualification gate verifiers (`validation/integrated_gate_verifier.py`,
`manipulation_gate_verifier.py`) also read contact force, but they run under
`manipulation-core`, where contacts are on — so they are untouched.

Fix: give the facade a contact-free stall path, which is what a real gripper
driver does anyway — detect that the finger has stopped advancing toward its
target while still short of it. A close whose best distance-to-target has
not improved by `stall_epsilon` for `stall_dwell_s` (default 0.3 s) while
still outside `position_tolerance` is declared `stalled`. It is additive to
the contact path (`contact-stall OR position-stall`), so `manipulation-core`
keeps its contact semantics and the two coincide on a real grasp there; the
position path is what carries sensor-rich. Set `stall_dwell_s <= 0` to
disable it. Enabled by default because a profile that cannot complete a real
grasp is not a defensible default — this supersedes the interim
`TINKER_SIM_SENSOR_RICH_CONTACTS` env-gate the grasp-bench session used to
unblock. No launch change: the GPSR/manipulation launches pass no override,
so the default applies once the bridge is rebuilt. A free close still exits
via `reached_goal` (the finger reaches target before any dwell elapses); an
already-touching close reports stalled after one dwell. Verified with the
`test_gripper_executor_humble.py` suite under the Humble overlay (8 passed,
3 consecutive runs), including a new parked-finger-stalls-without-contact
test; the three tests that deliberately park the finger for an orthogonal
concern (cancel, safety, stale-contact) now disable the path explicitly.
`manipulation-core` qualification should get one confirmatory run since its
facade result now has a second success route (gate verdicts are unchanged —
the verifiers read the raw contact topic, not the facade result).

## 2026-08-31 — joint4 tuck stall, physics-less YCB objects, wall-clock safety deadlines

Root-cause round for the sim bugs blocking the GPSR battery and the grasp
benchmark (branch `task-sim-bugfixes`, commits 8e4caa4, 53bc502, 9b5fe4a,
e5d4312). All measurements from in-process manipulation-core probes (CPU
PhysX, contacts on, no bridge; probe scripts under the session job dir).

**joint4 blocked near tuck (every tuck trajectory aborting).** The elbow sat
pinned at its 50 Nm `effort_limit_sim` with zero velocity, up to 0.04 rad
short of the orchestrator's tuck target. Ruled out in order, one variable
per boot: link contact (contact reporting enabled: zero pairs at the
stall), PhysX joint friction (the USD authors `physxJoint:jointFriction=1.0`
on all arm joints and Isaac Lab's own `data.joint_friction_coeff` reads 0.0
— its new-style friction-properties write path never reaches this PhysX
build's live coefficient — but zeroing the live coefficient via
`set_dof_friction_coefficients` left the stall bit-identical), and the fused
actuator model (`TINKER_SIM_STOCK_ACTUATOR_MODEL=1` bit-identical). The
tell: `get_dof_projected_joint_forces` at the stall reads 50.0 Nm on joint4
— a genuine static load at the cap. `data.default_mass` shows why: the
URDF->USD importer applies `MassAPI` with inertia but no authored
`physics:mass` to every link the URDF declares without `<inertial>`, and
PhysX then defaults each to 1.0 kg. tinker2 uses 21 such links as pure
frames — ~11 kg hanging off the wrist (link_eef, link_tcp, nine
xarm-camera frames), ~10 kg on the head. Real elbow-downstream mass is
~3.6 kg (<= 15 Nm), matching the earlier estimate that had "ruled out"
gravity from the *authored* masses. Fix: `_apply_stub_link_masses` authors
1 g at spawn on exactly the links the artifact's colocated `robot.urdf`
declares inertial-less and collision-less (data-driven, no name list),
beside the ballast/wheel corrections. Verified: joint4 tuck error 0.039 ->
0.0019 rad (= real ~13 Nm gravity / 7000 stiffness), stable across cycles.

Two side findings. `/joint_states` effort is **stale telemetry**:
`applied_torque` refreshes only when the target-write gate writes, so the
published effort freezes at the last mid-transient value (a saturated 50.0,
or ~1e-16 after a hold) — judge tracking by position. And the "arm ignores
all trajectories for 240 s after ~3 aborts" degraded state was never a
drive fault: see the deadline item below.

**Every YCB object was static scenery.** `ycb_import` published each
`object.usd` with colliders but no `RigidBodyAPI` on the default prim
(violating the repo's own spawnable-asset contract,
`simulation/assets/primitives/task-object.usda`), and `spawn_entity` adds
no physics APIs. So no scenario object ever resolved a rigid-body view: no
ground-truth pose in physics-truth frames, ungraspable. Only `soup` errored
(~17k physx pattern-miss lines/session) because `create_rigid_body_view`
raises after logging and the discovery loop's broad `except` aborted the
whole pass — soup was merely first in dict order; mug/banana/bowl failed
silently behind it. The old developer-log claim that these messages were
"benign shutdown noise" was wrong (first hit is ~40 s into the run, when
scenario_runner spawns). Fixes: `author_object_rigid_body` in the importer
(mass stays density-derived from the collision hulls — the soup can
computes to ~0.35 kg vs 0.349 kg published); `ycb_import --repair-physics
[--root]` republishes the current artifact Kit-free (deterministic
identity 5117994887a1...) and repoints `asset-manifest.json`; scenarios
reference the repaired identity; the discovery loop is per-object, logs one
`rigid_body_missing` diagnosis, and backs off 20x. Verified in-process: a
repaired soup resolves, reports a truth pose, and settles under gravity; a
deliberately broken sibling logs once and blocks nothing. Behavior change
flagged to the battery/bench sessions: YCB objects now settle and can be
knocked over.

**Wall-clock liveness deadlines re-latched the limp hold.** The gateway's
safety-heartbeat (1.0 s) and command-stream (0.5 s) deadlines were pure
wall clock while the publishers live in separate processes: an RTX render
stride stalls the stepping loop for multiple wall seconds with healthy
samples queued in DDS, and the gateway then re-latched the limp safety
hold / invalidated the command stream on every stride (grasp-bench report;
the GPSR battery ran the 1.0/0.5 s defaults, making this the prime suspect
for its post-abort dead-arm state). Deadlines now require staleness in
BOTH wall and simulation time (`9b5fe4a`): sim time freezes exactly when
the loop stalls, so the loop cannot punish itself; a dead publisher still
trips within one simulated timeout while stepping; wall age still gates
faster-than-realtime runs. The 59b9d7e env overrides remain as escape
hatch.

**Operator traps (grasp-bench reports #4/#5, e5d4312).** World mode
`current` with no `--arena` plus declared spawns now prints a
`world_selection_warning` (a full benchmark run was lost to a silently
bare ground plane), and `pick-deliver-place` moved to the validated free
corridor — its old robot (0,0) and object (0.65, 0, 0.8) poses both sit
inside shelf_02's rasterized footprint in the rcw2026 arena map.

**Head DEPTH "freeze": not reproducible on the current build.** A live
discriminator probe (in-process rig, `level-forward` aim + 3 cm dolly
active, Kit pumped per capture, five pan/tilt poses through pan 180 deg)
shows head depth tracking every pose change — valid fraction 0.47 (level,
empty world) -> 0.97 (tilt 30 down) -> 0.00 (tilt 30 up: sky, all samples
out of range -> all-zero by the 16UC1 contract) -> 0.74 -> 0.90 — with
fresh buffers whenever the scene changes. No housing occlusion from the
dolly at any probed pose, no stale annotator. Two benign behaviors imitate
a freeze: an out-of-range aim yields constant all-zero frames while color
keeps rendering, and a static scene yields byte-identical depth while RGB
still dithers (AA sampling), so "depth unchanged, color changing" is the
*expected* static-scene signature. The 2026-08-27 field report also
predates the pan/tilt `/joint_states` fix (4e9c694): with pan/tilt
commands being rejected, the head physically never left its boot pose, so
its depth naturally never changed while the wrist camera (riding the
moving arm) looked alive. If a freeze recurs post-4e9c694, capture
`TINKER_SIM_CAMERA_DEBUG=1` (annotator buffer-pointer churn) plus a
per-frame depth valid-fraction before filing.

Probe hygiene note for future camera probes: RTX render products tick on
Kit `app.update()` (run_sim's `_pump_streaming_app_update`), NOT on
`SimulationContext.render()` — a probe that skips the Kit pump sees every
camera frozen at the boot frame, color included.

**Spawned objects pass through the gripper (grasp-bench run 5, and the
likely root of live-manip's 100% referee fallback).** Definitive
in-process discriminator (overlap probe, cube spawned intersecting
left_finger, contact pairs accumulated every physics step): on a timeline
that has NEVER been stopped, a mid-play `/spawn_entity` body pairs fully
with the articulation -- 261 N depenetration contact on the finger. After
any timeline STOP -> PLAY cycle, bodies spawned during or after the cycle
pair only with static geometry and free-fall straight through the fingers
with zero contact events. The stack always enters that poisoned regime at
boot: scenario_runner runs `reset_spawned` (itself a stop->play, per the
documented ResetSimulation scope bug) plus the SPAWN_READY state-0 stop
before spawning, so every later bench/command spawn is ungraspable. Fix
(`b37b67a`): `standard_operations(spawn_while_playing=True)` spawns
everything onto the still-playing first-run timeline (no reset_spawned, no
state-0; the final state-1 op stays as a no-op play carrying the
PHYSICS_READY payload); plumbed as `scenario_runner --spawn-while-playing`
/ `TINKER_SIM_SPAWN_WHILE_PLAYING=1`. Deliberately opt-in until the A/B
baselines that assume the old boot sequence are re-cut; end-to-end
validation is the battery's s2026-000 live-manip rerun. Caveats: earlier
probe iterations that judged collision by "object reached the floor" were
worthless (a cube bounces off the narrow gripper to the floor anyway, fast
falls tunnel, and 0.5 s contact sampling misses pairs removed on
CONTACT_LOST) -- accumulate per-step contact maxima and use overlap
spawns. Same defect family, still open: `/get_entity_state` returns the
frozen spawn pose for such bodies (sim_control's RigidPrim binds against
SimulationManager's cached warp sim view, which never re-binds; observed
stale even on a fresh timeline in-process). Consumers should read gateway
physics-truth instead; `force_load_physics_from_usd` as a repair is
destructive (invalidates every live tensor view) -- do not use it mid-run.

**Wrist camera aim: same description defect class as the head, exactly
90 deg (`e821e79`).** Found by the first live s2026-000 run with the
collision fix armed: the wrist frame at the table-scan pose renders the
ceiling. The artifact's robot.urdf FK proves it: the camera optical axis
sits 90 deg from the tool approach axis at every configuration (joint
zeros: gripper -90 deg, camera level; scan pose: TCP -48 deg, camera
+42 deg up). The wrist camera stub frames are among the same hand-authored
inertial-less links as the phantom-mass defect; the real robot survives
because hand-eye calibration, not URDF TF, supplies its grasp extrinsics.
Sim correction mirrors the head one: `TINKER_SIM_WRIST_CAMERA_AIM=
tool-forward` (+90 deg about the optical frame's own +X = render axis onto
the TCP forward), opt-in, parity-breaking, set by gpsr-stack. Watchpoint
for consumers: if a pipeline derives wrist extrinsics from URDF TF instead
of calibration, corrected images now disagree with that TF by 90 deg --
good detections at wrong map positions is the signature.

**MDL-bound spawns render as nothing (`f8e764e`).** The vanished
cmd_spam_0 — tracked physically rock-stable at its desk pose for 57 s
(`TINKER_SIM_TRACK_OBJECTS`, `bad2693`) while the correctly-aimed wrist
camera saw an empty desk — was a MATERIALS defect: a prim spawned onto a
playing stage renders as NOTHING when its material is MDL (any MDL;
textured, untextured, opaque, or the converter's own authored-transparent
`OmniPBR_Opacity` with `opacity_constant=0.0`), while the identical mesh
with no material or a `UsdPreviewSurface` network renders correctly,
textures included. Ruled out along the way, one boot each: the spawn pose
(settles to 0.1 mm), slot overlap (single-item manifest; though two
overlapping spawns DO skitter violently across a desk — a real hazard for
placement planning), the sim_control service pose write, delete/respawn,
and file format (usda vs crate identical). Boot-parse objects rendered
under the old stop-bracket boot, masking this for furniture and
pre-battery scenario objects. Fix: `author_preview_surface_material`
rewrites MDL materials in place to UsdPreviewSurface networks (Material
prim path kept, bindings stay valid); `--repair-physics` is now a
physics+materials repair; round-2 artifact identity `4b635c93c704...`.
Verified live: mid-play-spawned spam and soup cans render fully textured
in the wrist view. Probe lessons: compute camera-frame placement from the
aim geometry before trusting an "invisible" verdict (two boots went to
out-of-frame layouts), and `force_load_physics_from_usd` mid-run destroys
every live tensor view.

**Viewed-prim deletion kills the boot; the backend now recovers
(`90ad061`, `a078da7`, `2c6f587`).** Two whole-boot kills in one evening,
same class: deleting a prim that ANY live tensor view covers invalidates
the SHARED SimulationView ("prim ... was deleted while being used by a
tensor view class"), after which every articulation read/write raises and
/spawn_entity is dead. First kill: the spawn-attach healer's per-spawn
probe views (`a078da7`'s original form) at a routine multi-entity clear.
Second: the referee hand_object's cached write view at its post-run
clear. Measured facts that shape the fix: the physics.tensors views have
NO release API, and even a dropped, garbage-collected Python view leaves
the backend registration alive -- so create-use-drop is NOT safe, and
avoidance alone cannot protect against every component. Layered fix:
(1) everything that watches DELETABLE spawns is view-free via
``IPhysx.get_rigidbody_transformation`` (healer, tracker); discovery keeps
views for scenario-boot objects under the documented invariant that those
are never deleted mid-play; (2) the backend self-recovers from the
invalidation: de-initialize the articulation (its PHYSICS_READY handler
early-returns while it believes itself initialized), rebuild the
PhysxManager AND isaacsim SimulationManager views replicating only their
creation lines (their own warmup paths force_load the stage and snap
every body to its authored pose -- unusable), re-initialize directly (the
event-bus path stores callback exceptions silently), re-anchor the
monotonic /clock (the physics step counter resets with the views), force
the next target write; budget 5 per boot. Verified live: two consecutive
held-view deletes each recovered in one attempt with the arm holding its
commanded pose and a mid-fall can landing DURING recovery. Also fixed
along the way: the spawn-attach healer itself (about 1 in 3 mid-play
spawns never enters PhysX -- per-spawn nondeterministic omni.physx parse
race; active-toggle re-parse nudge, TINKER_SIM_HEAL_DETACHED_SPAWNS).

**Still open.** Head + wrist camera aim: the description-level defects
need measurement on the physical robot; the env-gated sim corrections
remain the sanctioned workarounds. `/get_entity_state` staleness above.

## 2026-08-29 — Arena camera RTF: Phase 0 measurement

Throwaway measurement (`scripts/arena-rtf-spike`, sim-only, GPU 1, no
bridge): six 120 s-sim variants with `TINKER_SIM_PROFILE=1`, profiled every
10 camera cycles, to attribute the arena observer camera's RTF hit (~0.8 ->
~0.24 seen live) to render cost, DLAA, resolution, or the ROS publish path.

Preflight (GPU 1 reserved by prior agreement): worktree had no other
`run_sim.py` on GPU 1 and ≥5 GB free —

```
nvidia-smi --query-gpu=index,memory.used,memory.total,utilization.gpu --format=csv
1, 2498 MiB, 11264 MiB, 21 %
```

```
$ cat /proc/loadavg
8.72 10.60 11.89 3/3232 2861952
```

Note: this worktree was missing the `.cache`, `.deps`, `.venv` symlinks that
sibling `.claude/worktrees/*` checkouts carry to the shared Isaac/uv caches
(it had a partial real `.cache/uv` and an empty `.venv` instead, both
artifacts of the harness's first, failed attempt). All six variants failed
in seconds with `Isaac Sim internal Humble libraries were not found in the
locked environment` until those three symlinks were recreated to point at
the main checkout's `.cache`, `.deps`, `.venv` (same targets the sibling
worktrees use); the measurement below is from the re-run after that fix.

Six logs, six clean completions — no crash signatures (`error 700`,
`illegal memory access`, `cudaErrorMemoryAllocation`, `Traceback`) in any
log, no non-zero variant exits, and every variant well past the ≥20-window
sanity floor after the 30 s warm-up (119-128 windows each):

| variant | windows | kit_pump ms | cameras ms | physics ms | wall ms | RTF |
|---|---|---|---|---|---|---|
| A_arena_off | 119 | 41.0 | 16.1 | 50.0 | 123.4 | 0.68 |
| B_arena_2hz | 128 | 103.1 | 18.3 | 51.8 | 190.3 | 0.44 |
| C_capture_skip | 127 | 104.0 | 16.2 | 53.5 | 190.4 | 0.44 |
| D_dlaa_only | 127 | 90.9 | 15.4 | 50.9 | 175.3 | 0.48 |
| E_arena_640 | 127 | 98.3 | 16.1 | 50.9 | 183.2 | 0.46 |
| F_arena_0p5hz | 127 | 90.6 | 17.5 | 51.8 | 178.1 | 0.47 |

Per-variant notes:

- **A_arena_off** — baseline, arena camera fully disabled. RTF 0.68, the
  best of the six as expected.
- **B_arena_2hz** — arena camera on at 2 Hz, default resolution. RTF drops
  to 0.44; `kit_pump` more than doubles over A (+62.1 ms).
- **C_capture_skip** — same as B but `TINKER_SIM_CAPTURE_SKIP=arena_camera`
  (render still runs, ROS publish of the frame is skipped). `kit_pump`
  104.0 ms, essentially identical to B (104.0 vs 103.1) and nowhere near A
  (41.0) — the publish/bridge path is not where the cost lives.
- **D_dlaa_only** — arena camera *off*, only `TINKER_SIM_STABLE_AA=1`
  forced. `kit_pump` 90.9 ms — most of B's hit reproduced with the camera
  disabled entirely.
- **E_arena_640** — arena camera on at 2 Hz, resolution cut to 640x360.
  `kit_pump` 98.3 ms, only 4.8 ms better than B — resolution is not the
  lever.
- **F_arena_0p5hz** — arena camera on at 0.5 Hz (quarter the capture rate
  of B). `kit_pump` 90.6 ms, close to B (within 12%) — capture rate is not
  the lever either, and it lands right next to D.

Decision rule (all deltas in `kit_pump` ms, skip-first-30s means):

- `B - A` = 103.1 - 41.0 = **62.1 ms**
- `D - A` = 90.9 - 41.0 = **49.9 ms** = **80.3% of (B - A)** — clears the
  ≥60% bar for "DLAA is the tax" outright.
- `E` recovery vs `B`: (B-A) - (E-A) = 62.1 - 57.3 = 4.8 ms, i.e. **7.7%**
  of the gap recovered by dropping resolution — far short of the ≥60% bar
  for "resolution is the lever."
- `F` vs `B`: |90.6 - 103.1| / 103.1 = **12.1%**, within the 15% "F ≈ B"
  band, but `D` is not small (it is 80.3% of the gap), so the "fixed
  per-product cost" reading does not hold either.
- `C` vs `A`: 104.0 vs 41.0 — not close; `C` sits with `B` (104.0 vs 103.1,
  0.9% apart) instead, so the cost is not in `publish_cameras`.

`D - A` clearing 60% of `B - A` fires first and on its own: turning on
`TINKER_SIM_STABLE_AA` alone, with the arena camera left off, reproduces
80% of the arena camera's RTF hit. Neither resolution (E) nor capture rate
(F) meaningfully recovers it, and skipping the publish step (C) recovers
none of it. DLAA is the tax.

Decision: Task 4a

### Phase 1a — scoping the DLAA pin

Task 4a: stop the global `/rtx/post/aa/op` DLAA pin from taxing the
hardware-parity (head/wrist) cameras' render cost while keeping the arena
camera's CUDA-700 workaround intact. Two sub-approaches, tried in order,
per-render-product override first.

**Step 1 — probe (04:11:50 EDT, sensor-rich, `TINKER_SIM_ARENA_CAMERA=1
TINKER_SIM_CAMERA_DEBUG=1`, GPU 1, 20 s sim duration, clean exit).** A
temporary print in `CameraRig.initialize` after each `CameraSensor` is
created, listing every `aa`/`dlss`-named attribute already authored on that
camera's `UsdRender.Product` prim:

```
[probe] head_camera /Render/OmniverseKit/HydraTextures/camera_sensor_7993988978446 ['omni:rtx:dlss:frameGeneration', 'omni:rtx:post:aa:autoExposureMode', 'omni:rtx:post:aa:exposure', 'omni:rtx:post:aa:exposureMultiplier', 'omni:rtx:post:aa:limitedOps', 'omni:rtx:post:aa:op', 'omni:rtx:post:aa:sharpness', 'omni:rtx:post:dlss:execMode', 'omni:rtx:post:dlss:manualScaling', 'omni:rtx:post:fxaa:quality:edgeThreshold', 'omni:rtx:post:fxaa:quality:edgeThresholdMin', 'omni:rtx:post:fxaa:quality:subPixel', 'omni:rtx:post:taa:alpha', 'omni:rtx:post:taa:colorBoxSigma', 'omni:rtx:post:taa:samples', 'omni:rtx:pt:dlss:enabled', 'omni:rtx:scene:GPUProceduralAABB:enabled']
[probe] wrist_camera /Render/OmniverseKit/HydraTextures/camera_sensor_7993988978335 ['omni:rtx:dlss:frameGeneration', 'omni:rtx:post:aa:autoExposureMode', 'omni:rtx:post:aa:exposure', 'omni:rtx:post:aa:exposureMultiplier', 'omni:rtx:post:aa:limitedOps', 'omni:rtx:post:aa:op', 'omni:rtx:post:aa:sharpness', 'omni:rtx:post:dlss:execMode', 'omni:rtx:post:dlss:manualScaling', 'omni:rtx:post:fxaa:quality:edgeThreshold', 'omni:rtx:post:fxaa:quality:edgeThresholdMin', 'omni:rtx:post:fxaa:quality:subPixel', 'omni:rtx:post:taa:alpha', 'omni:rtx:post:taa:colorBoxSigma', 'omni:rtx:post:taa:samples', 'omni:rtx:pt:dlss:enabled', 'omni:rtx:scene:GPUProceduralAABB:enabled']
[probe] arena_camera /Render/OmniverseKit/HydraTextures/camera_sensor_7993988977774 ['omni:rtx:dlss:frameGeneration', 'omni:rtx:post:aa:autoExposureMode', 'omni:rtx:post:aa:exposure', 'omni:rtx:post:aa:exposureMultiplier', 'omni:rtx:post:aa:limitedOps', 'omni:rtx:post:aa:op', 'omni:rtx:post:aa:sharpness', 'omni:rtx:post:dlss:execMode', 'omni:rtx:post:dlss:manualScaling', 'omni:rtx:post:fxaa:quality:edgeThreshold', 'omni:rtx:post:fxaa:quality:edgeThresholdMin', 'omni:rtx:post:fxaa:quality:subPixel', 'omni:rtx:post:taa:alpha', 'omni:rtx:post:taa:colorBoxSigma', 'omni:rtx:post:taa:samples', 'omni:rtx:pt:dlss:enabled', 'omni:rtx:scene:GPUProceduralAABB:enabled']
```

`omni:rtx:post:aa:op` is authored on every render product individually →
approach (i), per-render-product scoping, is possible. Probe removed
before commit.

**Approach chosen: (i), per-render-product override.** Added
`CameraRig.initialize(..., stable_aa_cameras: frozenset[str] | None = None)`:
when given, the global `carb.settings` call is skipped and
`omni:rtx:post:aa:op` is instead set directly on the named cameras' own
render-product prims (`run_sim.py` passes `frozenset({"arena_camera"})`).
`stable_aa_cameras=None` keeps the old global-setting path byte-for-byte,
so any caller that doesn't pass it (there are none left after this change)
sees no behaviour change.

**Implementation bug found and fixed along the way.** The first attempt
authored the attribute with the same int the global carb setting takes
(`AA_OP_DLAA = 4`) via `GetAttribute("omni:rtx:post:aa:op").Set(4)`. This
surfaced immediately, not as a CUDA-700 recurrence but as a Python
exception during the first crash-recipe attempt (`up start` 04:15:30 EDT,
`gpsr_stack_logs/20260829T041530/01-sim.log`):

```
pxr.Tf.ErrorException:
	Error in 'pxrInternal_v0_25_11__pxrReserved__::UsdStage::_SetValueImpl' at line 7058 in file /builds/omniverse/usd-ci/conan/src/0.25.11.kit.2/pxr/usd/usd/stage.cpp : 'Type mismatch for </Render/OmniverseKit/HydraTextures/camera_sensor_8499610349849.omni:rtx:post:aa:op>: expected 'TfToken', got 'int''
```

Unlike the global carb setting, the per-render-product `omni:rtx:post:aa:op`
USD attribute is token-typed, with a *five-token* enum
(`["none", "taa", "fxaa", "dlss", "rtxaa"]`, from
`omni.usd.schema.render_settings.rtx`'s `generatedSchema.usda`) that
doesn't literally spell "dlaa". That extension's own test
(`test_render_settings.py::test_carb_command_line`) asserts
`settings.get("/rtx/post/aa/op") == 4` maps to token `"rtxaa"` — confirmed
as the DLAA-equivalent token, added as `AA_OP_DLAA_TOKEN = "rtxaa"` in
`camera_rig.py` and used for the per-product `.Set()` call instead of the
int constant. A follow-up 20 s sanity run (04:26:11 EDT, same recipe as
the Step 1 probe) completed cleanly with the fix in place, before starting
the real crash recipe.

**Step 5 — crash recipe, 3×5 min, `./scripts/gpsr-stack up --scenario
gpsr-rcw2026-bench --sim-gpu 1` (arena camera on by default in this
recipe), teardown via `./scripts/gpsr-stack down` between runs, GPU 1
confirmed free before each:**

| run | up start (EDT) | bring-up | sim watch window | teardown | result |
|---|---|---|---|---|---|
| 1 | 04:28:29 | 72 s (rc=0) | 300 s total, clean | 04:33:28 → 04:33:39 | no `error 700` / illegal memory access / Traceback in `gpsr_stack_logs/20260829T042828/01-sim.log` |
| 2 | 04:34:04 | 72 s (rc=0) | 300 s total, clean | 04:39:04 → 04:39:17 | no `error 700` / illegal memory access / Traceback in `gpsr_stack_logs/20260829T043404/01-sim.log` |
| 3 | 04:39:36 | 86 s (rc=0) | 300 s total, clean | 04:44:36 → 04:44:47 | no `error 700` / illegal memory access / Traceback in `gpsr_stack_logs/20260829T043936/01-sim.log` |

Clean 3/3. All three sim logs show `[sim] arena camera enabled at 2 Hz,
960x540 -> /sim/arena_camera/image_raw` and run the full 300 s watch
window before teardown; the only errors present are unrelated benign
shutdown-time warnings (`Pattern '/World/Scenario/soup' did not match any
rigid bodies`, a physx-tensors cleanup message, present in all three runs
of Task 3's Phase 0 sweep too).

**Re-measure (sim-only, GPU 1, `TINKER_SIM_PROFILE=1`, variants A and B of
`scripts/arena-rtf-spike`, run by hand — `outputs/rtf-remeasure-4a/run_ab.sh`,
not committed):**

| variant | windows | kit_pump ms | cameras ms | physics ms | wall ms | RTF |
|---|---|---|---|---|---|---|
| A_arena_off | 118 | 37.1 | 13.6 | 49.0 | 115.9 | 0.72 |
| B_arena_2hz | 121 | 44.1 | 15.7 | 50.5 | 127.1 | 0.66 |

`B - A` = 44.1 - 37.1 = **7.0 ms**, down from Task 3's 62.1 ms — a shrink of
55.1 ms, exceeding the 49.9 ms (`D - A`) predicted shrink from Task 3's
decision rule (some of the gap is run-to-run variance: `A` itself came in
4 ms lower than Task 3's 41.0 ms baseline). RTF for the arena-camera-on
variant recovers from 0.44 (Task 3, global pin) to 0.66 (scoped pin),
close to the arena-camera-off baseline's 0.72. The scoped pin removes
essentially all of the DLAA tax from the parity cameras while the arena
camera itself still gets DLAA (crash workaround intact, confirmed by the
3/3 clean crash recipe above).

### Phase 2 — arena defaults

Task 5: lower the arena camera's defaults from 4 Hz / 960x540 to
`ARENA_CAMERA_DEFAULT_HZ = 2.0` and `ARENA_CAMERA_DEFAULT_SIZE = (640, 360)`
(`simulation/tinker_sim_isaac/arena_camera.py`). Justification is Phase 1a's
re-measure, not Task 3's `E`/`F` variants (those were Phase 0 measurements
taken under the *old* global DLAA pin, before the scoped-pin fix landed —
`F_arena_0p5hz` alone moved `kit_pump` by 12.5 ms, RTF 0.44 → 0.47, so they
are not evidence of a small effect post-fix). The real justification: with
the DLAA fix in place, Phase 1a's re-measure shows the arena camera's
*entire* residual cost is ~7 ms of `kit_pump` (post-fix `B - A`: 44.1 vs
37.1 ms), so its capture rate and resolution individually cannot move RTF
by more than that either way — lowering the defaults costs nothing
measurable to give up, not a real tradeoff. 640x360 was chosen because a
bird's-eye frame at that size still shows a recognisable person and table,
which is all a bird's-eye evidence frame needs. `tools/contact_sheet.py`
was checked (`grep -n "960|540|arena"`)
and left untouched: its `960` is the judge sheet's own 3-tile layout width
(`TILE_W = 320`), not a hard-coded arena frame size — each tile is resized
from the loaded frame's actual `width`/`height` (`contact_sheet.py:313`),
so it already adapts to any arena frame size. Step 3 (live visual check of
a 640x360 frame with a person and table recognisable, `TINKER_SIM_ARENA_CAMERA=1`
on GPU 1) is deferred to Task 7's stack run — GPU 1 was in use by another
session's benchmark stack at the time of this task.

### Phase 3 — bench policy

Task 6: `gpsr-stack up` now takes `--evidence` to opt into the arena camera; pass/fail batteries run arena-off by default.

### Phase 4 — verification on the bench stack

Task 7: verified the fix on the real hybrid stack (bridge + Nav2 + vision
attached, `--manipulation mock`), not just sim-only. `TINKER_WS`/GPU 1,
`gpsr-rcw2026-bench`, `scripts/gpsr-stack up/down`.

**Regression guard (Step 1).** Task 4 landed approach (i) (per-render-product
override), so the guard is a module constant:
`STABLE_AA_CAMERAS = frozenset({"arena_camera"})` in `validation/run_sim.py`,
used at the `camera_rig.initialize(...)` call site instead of the inline
literal, with `test_stable_aa_cameras_is_arena_only`
(`tests/test_run_sim_arena_wiring.py`) pinning it. All four camera test
files: 51 passed.

**Before/after, bridge-attached, idle (simulated-sec / wall-sec, 3 samples
each, ~1-2 min apart via `/clock`):**

| configuration | sample 1 | sample 2 | sample 3 | mean RTF | GPU 1 util (nvidia-smi) |
|---|---|---|---|---|---|
| before (bench session, earlier 2026-08-29) | 0.21 | — | 0.24 | 0.21-0.24 | 98% |
| after, arena camera on (`--evidence`) | 0.544 | 0.592 | 0.594 | **0.577** | 45-53% (~0.65 GB more VRAM than off) |
| after, arena camera off (control) | 0.643 | 0.692 | 0.594 | **0.643** | 48-53% |

Raw samples (arena on, `up --scenario gpsr-rcw2026-bench --sim-gpu 1
--evidence`, logs `gpsr_stack_logs/20260829T052441`): 233->244 sec / 20.225 s
wall = 0.544; 297->309 sec / 20.266 s = 0.592; 320->332 sec / 20.220 s =
0.594. Arena off (control, no `--evidence`, logs
`gpsr_stack_logs/20260829T053421`): 25->38 sec / 20.213 s = 0.643; 47->61
sec / 20.241 s = 0.692; 69->81 sec / 20.218 s = 0.594. (Method note: `grep
sec` also matches `nanosec:` lines, since `nanosec` contains `sec` as a
substring — anchored to `grep '^sec:'` before `sed -n '1p;$p'` so the "last"
line is actually the last `sec:` value, not the last message's `nanosec:`.)

Both configurations (4a arena-on-with-evidence and the arena-off control)
were taken; the brief's 4b variant (a lower-cost mitigation short of the
scoped-pin fix) was not needed — the scoped DLAA pin from Task 4a's Phase 1a
already recovers RTF from 0.21-0.24 to 0.58 with `--evidence` on the real
stack, well clear of the ≥0.5 "GPSR runnable" floor, so there was nothing
left for a second, weaker mitigation to buy.

GPU 1 utilisation stayed in the 45-53% band across every sample in both
configurations — nowhere near the 98% seen before the fix (arena-on no
longer saturates the card; the remaining draw is the vision stack sharing
GPU 1, not the arena render).

**Crash recipe.** No re-run needed for this task — Phase 1a's Step 5 already
recorded 3/3 clean 5-minute bring-up/watch/teardown cycles with the arena
camera on and the scoped DLAA pin in place (zero `error 700` / illegal
memory access / `Traceback` in any of the three `01-sim.log`s). This task's
own two stack-up logs (`20260829T052441`, arena-on-with-evidence;
`20260829T053421`, arena-off) add two more clean runs with no CUDA-700
signature, for a combined 5/5 across both tasks.

**Policy.** `--evidence` is the one-line policy: contact-sheet/evidence runs
opt into the arena camera (and pay the RTF cost above); pass/fail batteries
run arena-off by default and are unaffected.

**Visual check (deferred from Phase 2/Task 5).** With arena-on up:
`/sim/arena_camera/image_raw` reports `width=640`, `height=360`,
`encoding=rgb8` (matches Phase 2's 640x360 default). `image_view` is not
installed on this box, so a frame was saved via a small `rclpy` +
`sensor_msgs/Image` + PIL script (`cv_bridge` is broken in this env — built
against numpy 1.x, current numpy is 2.x) to `outputs/arena-640-check.jpg`
(gitignored, not committed). Visual inspection: a bird's-eye view of the
arena floor plan is legible, with a round dining table and two chairs
clearly recognisable, and two humanoid figures clearly recognisable — one
standing near the table, one standing near a doorway at the bottom of the
frame. Confirms Phase 2's 640x360 choice.

**Follow-up, not done.** The spec's Phase 2 item 3 (gating the head camera's
12 Hz publish on having a live subscriber) was not pursued here. Reason:
Phase 0's row `A_arena_off` — the baseline with the arena camera fully
disabled — already shows `kit_pump` (41.0 ms) is the largest per-cycle
bucket *after* physics (50.0 ms) and well ahead of `cameras` (16.1 ms);
subscriber-gating only touches the `cameras` bucket, which is not where the
arena-off budget is going, so the payback looked small relative to the
change's risk (a codepath that silently stops publishing when nothing is
subscribed) and it was left for a future task with its own measurement.

## 2026-08-26 — GPSR recorded sim battery bring-up

`scripts/gpsr-stack up/down/status` automates Stages 1-5 of
`docs/gpsr-sim-runbook.md` (Stage 6, the GPSR orchestrator, still lives in
the `tk25_ws` decision repo and is started separately). Getting a full
hybrid (`--manipulation mock`) bring-up to all-gates-green took nine
attempts across two repos; this entry is the fix narrative behind those
attempts (full blow-by-blow, if ever needed, was kept in this task's
now-deleted scratch notes — the summary below is complete on its own).

**Fix 1 — vision-stage headless crash: `waving_person_server`'s debug
window.** `vision_bringup.launch.py` starts `waving_person_server`
(`tk_vision_specialized`) with no parameter override, so it kept its
`show_window=True` default; on a headless GPU box with no `xcb` display and
no `offscreen` Qt plugin bundled in that venv, the node SIGABRTs at startup
and takes the whole vision stage down with it. `detect_waving.launch.py` (a
different, standalone launch entry point in the same `tk26_vision` package)
already forced `show_window:=False` for exactly this reason — the gap was
`vision_bringup.launch.py` not doing the same. Fixed upstream in
`tk26_vision` (commit `20b1a20`, "show_window off"); this repo's own
mitigation is `scripts/gpsr-stack`'s vision-stage env,
`QT_QPA_PLATFORM=offscreen`, which is a general belt-and-suspenders guard
against any node in that stage defaulting to an on-screen Qt window, not a
substitute for the upstream fix.

**Fix 2 — vision gate never satisfied despite every node self-reporting
ready: Fast DDS interface whitelist.** With the show_window crash fixed, all
6 vision-0 nodes logged successful startup (services, actions, and topics
all created, verified by reading node source directly against
`tools/gpsr_interface_census.py`'s expected names/types — exact matches),
yet the census's own `rclpy` discovery never saw them, across ~36
independent polls over a full 180 s gate timeout. Root cause: the vision
stage's Fast DDS profile's interface whitelist did not include this host's
current wired IP, so default-transport participants (the census subprocess,
the orchestrator) could not discover vision's participants at all — not a
name/type/namespace mismatch, a transport-level one. Fixed by adding the
host's wired IP to that whitelist; verified live (vision services became
discoverable to a fresh default-transport participant immediately).

**Fix 3 — `vision_bringup` needed a rebuild.** Between two attempts, the
`vision_bringup` package the earlier attempts launched against was stale in
`tk25_ws/install` (missing a fix already merged upstream); rebuilding it
into `tk25_ws/install` picked up the current source. Routine but worth
recording: a hybrid bring-up against a stale `tk25_ws/install` can silently
run old vision code.

**Fix 4 — CUDA error 700 (illegal memory access) during camera capture:
DLSS resize race, arena camera parked.** Once the vision gate cleared, sim
startup hit a Warp `wp_cuda_stream_synchronize` CUDA error 700 roughly 15-40 s
in, reproduced 3/3 with the world-fixed arena observer camera enabled
alongside the two hardware-parity cameras (head+wrist), 0/2 with only the
two hardware-parity cameras. Root cause (see
`simulation/tinker_sim_isaac/camera_rig.py`'s `CameraRig.initialize`
docstring for the full mechanism): DLSS's default anti-aliasing op
auto-picks an internal render resolution below the render product's
declared output resolution when that pick falls under DLSS's ~300 px
minimum input size (the 848x480 wrist camera's default-picked internal size
is 424x240, both under 300), then live-resizes up — and with 3+ concurrent
RTX render products alive, that resize raced this rig's Warp device-to-host
copy/synchronize. Fix: pin DLSS to its native-resolution `DLAA` op
(`stable_aa=True`, `AA_OP_DLAA`) whenever the arena camera pushes the
render-product count to 3+, scoped so hardware-parity-only runs (2 render
products) keep their previously-verified default AA path (commit `1ec9ade`).
**This fix was necessary but not sufficient**: error 700 recurred on the
*wrist* camera under the same `sensor-rich` (3-camera) profile even with the
DLAA pin in place, and a follow-up retest with only 2 render products
(head+wrist, arena off) still hit error 700 once — disproving the working
theory that render-product *count* (specifically, "3 is unstable, 2 is
stable") was the load-bearing variable
(`.superpowers/sdd/2026-08-25-gpsr-recorded-sim-battery/task-9-report.md`,
attempt 8). The controller ruling that actually stuck: park the arena
observer camera outright as a known issue (it is sim-only tooling, not
required for GPSR parity) and record head-camera only for the battery
(commit `1e730d4`). Zero error-700 hits across every subsequent attempt.
`TINKER_SIM_ARENA_CAMERA=1` still exists to re-enable it once someone picks
the CUDA-700 investigation back up; `TINKER_SIM_DISABLE_WRIST_CAMERA=1` (the
now-disproven 2-product mitigation) is retained only as a manual operator
escape hatch and is incompatible with `scripts/gpsr-stack`'s census gate
(see that flag's comment in `validation/run_sim.py`).

**Fix 5 — live-manipulation launch needed the model-bundle manifest wired
in.** `mobile_bringup manipulation_planning_task_only.launch.py` takes a
`model_bundle_manifest:=` argument `scripts/gpsr-stack` was not passing at
all; `resolve_model_bundle_manifest()` (`scripts/gpsr-stack:89-140`) now
resolves the canonical bundle produced per
`ros2_ws/src/tinker_sim_bridge/README.md` at
`outputs/ompl-overlay/model-bundle/model-bundle.json` and raises with that
README's exact generation recipe if it hasn't been produced yet — deliberately
not falling back to the robot-artifact `manifest.json` under `artifacts/`,
which does not satisfy `mobile_bringup`'s model-bundle schema and would
otherwise fail later, opaquely, inside the launch itself.

With all five fixes in place: hybrid (`--manipulation mock`) bring-up
reaches all 4 gates green with zero error-700 hits, and step 5's tier2
smoke corpus ran 2/2 PASS. Live-manipulation (`--manipulation live`)
bring-up reaches the manipulation stage (previously never reached) and is
blocked only on environment-local pieces out of this repo's scope (a
missing `anygrasp` checkpoint, `tk25_ws`-side Python dependencies for the
orchestrator) — not on anything `scripts/gpsr-stack` itself does.

## 2026-08-28 — Arena reskin: wood floor + tinted furniture (rcw2026)

The rendered arena read as stark-white: Isaac's default gray ground grid
under everything, and GLB furniture that renders bland/white because the
converted models' own material bindings do not resolve at render time (the
artifact ships `textures/<id>/*.png`, but they don't bind through). Fix is
in `tools/tinker_sim_deploy/arena_convert.py`, all at compose time — no GLB
re-conversion, no image textures (solid PBR keeps render cost ≈ current):

- **Floor**: `compose_arena` now authors `/World/Arena/Floor`, a thin
  (`2 cm`) visual cube covering the wall-footprint AABB (+10 cm margin,
  `floor_slab`), oak-tinted (`FLOOR_COLOR`). No collider — physics still
  rides the backend's `GroundPlaneCfg` plane at z=0; the slab's top face
  sits `2 mm` above it so it wins the depth test against the default grid.
- **Furniture**: a solid PBR from `FURNITURE_COLORS` (keyed by model_id,
  warm-neutral fallback) bound on each furniture wrapper at
  `strongerThanDescendants`, overriding the GLB subtree's bindings. Woods
  brown, fabrics muted, appliances off-white steel (intentional, not the
  untextured default), plants green, TV black.
- **Walls**: unchanged (gray `0.6`). `_bind_gray_material` now delegates to
  the shared `_bind_pbr_material`.

Pure geometry/color logic (`floor_slab`, `furniture_material`) is unit-tested
in `tests/test_arena_convert.py` under plain Python; the pxr authoring is
live-only as usual.

**Operator step — regenerate + re-pin (Isaac box, needs Kit + pinned SOBITS
checkout; not runnable in dev/CI):**

1. `python tools/arena_import.py --config config/arena-import.json`
   (add `--checkout <existing pinned checkout>` to skip the clone). This
   publishes a new content-addressed arena artifact and re-points
   `artifacts/arena/rcw2026/current.json`. Then register the new
   `arena.usd` path + sha256 under `generated_arena_usds` in
   `artifacts/asset-manifest.json` (the tool prints this reminder).
2. Eyeball the render: wood floor present, furniture tinted, walls unchanged.
3. Run the vision/detection smoke to confirm detection parity and that
   per-frame render time stayed close to the pre-change baseline (solid PBR
   should not move it). If a furniture tint still reads white, the override
   binding didn't win — check the `strongerThanDescendants` strength.

Design: `docs/superpowers/specs/2026-08-28-arena-wood-floor-furniture-tint-design.md`.

## 2026-08-22 — GPSR `goto_command_point` stall: two root causes in the sim, one residual

Starting point: `reports/gpsr-sim-2026-08-20/NAV-HANDOFF.md` — Nav2 never
left the first goal, `controller_server` aborted with `Failed to make
progress` every ~13 s (57×), recoveries ran 8×, `/amcl_pose` wandered in and
out of the 0.1 m tolerance. The handoff suspected `min_theta_velocity_threshold`
and the progress checker. Neither was the cause (`min_theta_velocity_threshold`
filters *odometry*, not commands). Everything below was measured on the same
stack (`sensor-rich`, `gpsr-rcw2026`, `--arena rcw2026 --spawn-xy=-2,-2`,
`navigation.launch.py`, domain 71) with `/sim/internal/physics_truth` as ground
truth; probe scripts lived in the job's tmp dir, numbers are in the text.

**Reproduced first.** Nav-only stack, goal = the scenario spawn (−2, −2, yaw 0):
same 13 s abort cadence, same recoveries. Truth vs estimate showed the
*physical* robot turning at −0.1…−0.2 rad/s for a −0.6 command while wheel
odometry reported +0.2…+1.3 rad/s, the estimate "moving" 0.5 m in 6 s, and
the robot physically 1.2 m off the goal while AMCL (cov 0.012) put it 0.1 m
away. Open-loop `/cmd_vel` without Nav2 isolated it: forward 0.2 m/s was fine
(truth 0.17, fronts 3.1 vs 3.8 rad/s target); **rotate ±0.5 rad/s gave truth
0.11 / −0.09 rad/s**, front wheels stalled and chattering (−0.24±0.66 vs
−1.19 target), odom yaw rate garbage.

**Root cause 1 — the rear casters were driven and held (fixed).** The URDF's
rear wheels (r = 0.03 m) sit on free swivel joints; `base_facade` commands
all four wheel joints with the front wheels' angular velocity and the
backend's `wheels` actuator group matched `rear_.*`, so (a) the caster wheels
got a damping-200 velocity drive at a target wrong by the radius ratio —
forward they were braked (the front drive saturated, 3.1 vs 3.8 rad/s), in a
turn they were skids — and (b) the caster *swivels* were caught by the same
group (damping 200 toward zero). Narrowing the group to the wheels only made
it worse: the URDF importer bakes a stiffness-625, unlimited-force position
drive (target 0) onto every continuous joint, so an unconfigured swivel is
rigidly held straight (swivel position stayed within ±0.0006 rad through an
8 s turn). Fix: drive only `front_.*_wheel_joint`; `rear_.*_swivel_joint`
and `rear_.*_wheel_joint` form an explicit zero-gain `casters` group
(`CASTER_JOINT_PATTERNS`). Result: forward 0.20 m/s at 3.76/3.80 rad/s (no
saturation), casters free-roll at 6.6 rad/s (= 0.2/0.03), **rotate ±0.5 →
0.28 / −0.27 rad/s and odom yaw rate now tracks truth** (0.29 / −0.33).
`tests/test_wheel_actuator_patterns.py`.

**Root cause 2 — AMCL was given the wrong map (fixed).** The `sensor-rich`
lidar is not a rendered sensor: `ros_gateway.py` raycasts 181 rays over ±90°
against the simulator's occupancy grid from the truth pose, i.e. against
`artifacts/arena/rcw2026/<current>/map.yaml`. `navigation.launch.py` and
`gpsr.launch.py` default `map_yaml` to the *robot artifact's* colocated
`map.yaml`, which the manifest traces to `0701_robocup_arena3` — the hardware
arena. The two maps share **zero** occupied cells in world coordinates (even
under a ±2 m shift search). The 2026-08-18 AMCL study passed the arena map
explicitly, which is why it found AMCL healthy. With the arena map and
passive casters, AMCL tracks truth: 0.03 m at rest, 0.13–0.26 m through
in-place rotation, 0.09 m after 1.5 m of driving, yaw within 0.10 rad. Fix:
`gpsr.launch.py` resolves the map from the scenario's `world.arena`;
`navigation.launch.py` takes `arena:=`; explicit `map_yaml:=` still wins
(`runtime.resolve_arena_map_yaml`, `scenario_arena_id`;
`tests/test_arena_map_resolution.py`, `test_navigation_launch_map.py`).

**Root cause 3 — the wheel colliders' line contact locked the turn (fixed).**
With the first two fixes the robot physically arrived within 0.16–0.21 m of
the goal and the estimate within 0.1 m, but the final yaw trim never
completed: DWB commanded ~0.09 rad/s and the base did not move, so
`SimpleProgressChecker` (XY only, 0.5 m in 10 s) aborted, the spin recovery
added ±1.57 rad, and it repeated. Measured truth yaw rate vs command:
**0.1 → 0.00, 0.2 → 0.07, 0.3 → 0.13, 0.5 → 0.27, 0.8 → 0.50**, wheel joints
reading exactly 0.0 under small commands with the drive at its cap; in a
turn the wheels lagged their targets by a near-constant 0.2–0.5 rad/s, a
dry-friction-like resistance of 40–80 N·m per wheel. Ruled out one variable
at a time: drive effort cap (10 vs 80 N·m: identical), articulation velocity
solver iterations (8 vs 1: identical), PGS instead of TGS (chatter gone,
0.5 → 0.36, deadband unchanged), articulation `sleep_threshold=0` (wheels
then read −0.03 rad/s — awake, still stuck), and the PhysX-side drive gains
(read back through the tensor view: stiffness 0, damping 200, max force 80,
force-type — the drive is what the config says). The casters do align
(swivels at −120°/−68°, the tangential trailing angles). What did move the
needle was the wheel *collider*: the importer's `Cylinder` prims are exact
custom-geometry cylinders (`/physics/collisionApproximateCylinders` is
already false; forcing convex hulls made it worse, 0.3 → 0.002), and a
cylinder's line contact across the 63 mm tread cannot roll on a 0.125 m
turn radius — the inner and outer edges need different speeds, so the patch
locks. Replacing the cylinder with a **sphere of the same radius** on the
drive wheels gave 0.2 → 0.20, 0.3 → 0.29, 0.5 → 0.49 (0.1 still dead: the
caster wheels had the same patch); on all four wheels, on the stock TGS
solver with default sleep settings, **0.1 → 0.099, 0.2 → 0.197, 0.3 →
0.298, 0.5 → 0.498, 0.8 → 0.795** with steady wheel speeds, forward driving
unchanged (0.20 m/s at 3.77 rad/s), `/clock` still ~48 Hz. Fix:
`_apply_wheel_sphere_colliders` (runtime override at spawn like the chassis
ballast — artifact untouched; deactivates `<wheel>/collisions/mesh_0`,
authors `<wheel>/collisions/sphere` with the cylinder's own radius, fails
closed on a missing wheel or collider); `TINKER_SIM_WHEEL_COLLIDER=cylinder`
restores the authored collider for A/B; `tests/test_wheel_colliders.py`.

**End to end.** Nav-only stack (`navigation.launch.py map_yaml:=<rcw2026
arena map>`), AMCL seeded at truth, robot 1.0 m from the goal and facing
away: `Reached the goal!` / `Goal succeeded` in 13 s with the stock
tk26_navigation parameters, truth 0.11 m / 0.12 rad from (−2, −2, 0),
estimate within 0.07 m of truth throughout, zero `Failed to make progress`.

**Nav2-side observations for the navigation owners** (tk26_navigation, not
this repo, no change needed now): `tracking_goal_checker` (`yaw_goal_tolerance:
3.14`) exists in `nav2_dwb_params.yaml` but is commented out of
`goal_checker_plugins`; the XY-only `SimpleProgressChecker` converts any
station-keeping yaw trim slower than 10 s into a recovery, which is what
amplified the base defect above into a 13-minute stall.

**Also learned.** A stack launched with `&` from a non-interactive shell
inherits SIGINT=ignored, so `ros2 launch` never sees a later SIGINT — reset
the disposition in a wrapper before exec. Tear the Nav2 launch down *before*
the simulator: its nodes run on sim time and hang in shutdown on a frozen
clock. The robot artifact's `map.yaml` is the hardware map by design; the
`(−2, −2)` scenario spawn cell is "unknown", not "free", in both maps.

## 2026-08-21 — Bridge attached while driving and while the arm moves

Exercise: `/cmd_vel` 0.15 m/s + 0.25 rad/s at 15 Hz for 45 s (Nav2's
controller rate, no localisation needed), then `JointTrajectory` goals to
`xarm7_traj_controller` alternating two poses every 5 s for 45 s; motion
verified from `/isaac_joint_states`. Bench scripts in the main checkout's gitignored
`outputs/bench/` (`run_with_bridge_exercise.sh`, `exercise.py`,
`record2.py`, `show_phases_wall.py`).

| bridge attached, control 60, cameras 12 Hz | old intake | main-thread intake + 60 Hz cap |
|---|---|---|
| idle | 0.71 | **0.81** |
| base driving | 0.59 | **0.68** |
| arm trajectories | 0.50 | **0.63** |

What the remaining cost is: driving pushes no new targets (the wheel
velocity targets are constant) — it is PhysX contact/rolling work, +15 ms
per camera cycle; a 45 s circle in the rcw2026 arena ends with the robot
against props, and that contact cost persists afterwards (physics 54 vs 41
ms/cycle), which is the scenario, not the bridge. Arm trajectories push
targets on ~70% of control steps (JTC rewrites the command each cycle) and
the moving drives cost PhysX ~3 ms/step more.

Three changes came out of this:

- **Main-thread intake.** The gateway's private executor thread is gone.
  `spin_once()` now takes messages straight from the two DDS readers
  (`_take_pending`, rclpy `handle.take_message`, up to 512 per reader per
  step). The thread could only take one message per wait-set pass and each
  pass had to win the GIL back from the simulation loop; removing it took
  `publish` from 24 back to 15 ms/cycle and idle RTF from 0.71 to 0.81.
  `TINKER_SIM_GATEWAY_EXECUTOR=1` restores the thread for comparison.
- **60 Hz cap on changed snapshots** in `command_gateway`
  (`MIN_PUBLISH_PERIOD_S`): ros2_control rewrites the arm command every
  150 Hz cycle during a trajectory; the simulator applies targets 60 times
  a second, so anything faster was pure overhead. Safety-stop snapshots and
  epoch bumps are never delayed.
- **Profile lines carry `wall_time` and `sim_time`.** Attribute phases by
  wall time. The simulator re-zeroes `/clock` when the bridge's
  `ResetSimulation` (STOP -> PLAY) lands, by design (Isaac Lab recreates the
  articulation view on PHYSICS_READY and that boundary is the new zero), so
  `/clock` and a step counter started at process launch differ by the
  attach time (~34 s here). Six runs of this investigation chased a
  "stale trajectory replayed 30 s late" that was entirely this offset: the
  snapshot ids received by the simulator matched the bridge's live
  publish counter once compared on wall time, and every message's DDS
  source timestamp was < 70 ms old. Ruled out on the way, each by
  measurement: DDS shared-memory transport (UDP-only identical), reliable
  repair (BEST_EFFORT identical), Kit worker-thread starvation (16 threads
  identical), a second publisher (one `command_gateway` process, one
  writer), a gateway replay path (none exists).

Side finding: `TINKER_SIM_CPU_THREADS=16` (Kit's default is 32 on this
32-thread host) gave idle 0.81 -> 0.82-0.84 and arm 0.63 -> 0.66 with
physics 44 -> 40 ms/cycle; the TBB workers otherwise spin at ~30% each.
Worth recommending for live-stack runs; not made the default.

## 2026-08-21 — ROS bridge attached: RTF 0.77 -> 0.23 -> 0.71


Measured on 2026-08-21 with `sensor-rich`, `TINKER_SIM_CONTROL_HZ=60`,
`TINKER_SIM_CAMERA_HZ=12`, the Stage 2 bridge attached ~50 s in and the
stack idle (no goals): RTF 0.77 standalone -> **0.23** with the bridge up ->
0.75 again once it was killed. The loss was entirely inside the simulator's
Python loop, not the host: scheduler wait stayed at 0.1%, GPU 0 idle, and
the sim process's own CPU share *fell* from 80% to 47% while its voluntary
context switches quadrupled. The profile attributed it:

| per camera cycle (5 control steps) | alone | bridge attached |
|---|---|---|
| `spin` (`gateway.spin_once`, inbound commands) | 0.2 ms | 100 ms |
| `publish` | 12 ms | 92 ms |
| physics / Kit pump / cameras | 47 / 31 / 14 ms | 78 / 52 / 34 ms |

Root cause: `command_gateway` re-sent **every** mux packet (base,
ros2_control, gripper, pan_tilt) as a fresh full snapshot on its 150 Hz
tick even when nothing changed, ~300-600 `JointState`/s into the simulator.
Each packet cost the simulator ~0.9 ms in `command_joints` (torch element
writes that each release the GIL to the busy executor thread), and the
executor thread's deserialisation work taxed every other GIL release in the
loop -- `imu` publish, which has no subscriber at all, went from 0.6 to
5.5 ms per call. Things that were **not** the cause (each measured): CPU
oversubscription, GPU sharing with vision (the bridge launches no vision
and tinker2-net's GPSR set has two BEST_EFFORT camera subscribers), Fast DDS
synchronous publish mode (`RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS`
changed nothing), and the gripper effort-limit write (never exercised).

Fixes, all result-neutral for the simulator's targets:

- **Bridge keepalive.** `command_gateway` still evaluates deadlines and
  composes the snapshot at 150 Hz but publishes it only when it differs
  from the last one sent or every 50 ms (`CommandGateway.KEEPALIVE_PERIOD_S`,
  10x inside the simulator's 0.5 s command-stream watchdog). Snapshots stay
  complete (the simulator zeroes velocity targets per snapshot, so partial
  snapshots are not an option). Idle inbound packets: 2141 -> 88 per
  100-step window. **Bridge-attached RTF 0.23 -> 0.71** (0.77 standalone in
  the same run). While the stack is driving, packets rise with the base
  facade's 50 Hz command rate; expect a partial regression that was not yet
  measured.
- **Batched target apply.** `backend._apply_joint_command` gathers a packet
  in Python and writes each target tensor once instead of per element
  (fewer GIL release points per packet; identical resulting targets).
- **Gripper effort-limit dedup.** An unchanged ceiling no longer reaches
  PhysX (harmless, but it was a plausible suspect and is now cheap to rule
  out via the `gripper_limit_writes` counter).
- **Profile.** `TINKER_SIM_PROFILE=1` now also reports `spin`, `unaccounted`
  and `wall` per cycle and a `spin_breakdown` (events, commands,
  `command_joints_ms`, `gripper_limit_writes`) -- if `spin` climbs again,
  count commands first.
- **Opt-in knobs** (default behaviour unchanged):
  `TINKER_SIM_GIL_SWITCH_INTERVAL_MS` (0.5 recovered 0.23 -> 0.29 with the
  *old* bridge; residual value with the keepalive is small) and
  `TINKER_SIM_CPU_THREADS` (caps Kit's worker pool, default min(cores, 32);
  not needed on this host -- scheduler wait was never the problem).

Why not a C++ bridge: the cost was packet *count* times simulator-side
Python per packet; a C++ sender of the same stream reproduces RTF 0.23
exactly. C++ would only matter on the simulator side (rclpy's GIL), which
is a large port of the safety-critical session/epoch state machine and is
not justified by the residual 0.06.

Follow-up runs on the same day (all bridge attached, idle, control 60,
cameras 12 Hz): keepalive alone 0.71; keepalive + `TINKER_SIM_GIL_SWITCH_INTERVAL_MS=0.5`
0.70 (no further gain, so the knob is not recommended); old bridge +
switch interval 0.5 alone 0.29. Residual gap to standalone (0.77-0.79) is
`publish` 12 -> 24 ms per cycle and `spin` 2 ms: the ~88 packets and ~64
safety heartbeats per 100-step window still cost a GIL hand-off each.
Unmeasured: the stack actively driving (base facade then emits changing
50 Hz commands, so packet count and the cost rise again).

## 2026-08-21 — Per-step costs removed (result-neutral)


- **Safety hold.** While stopped, the backend used to disable the arm's
  PhysX drive and push a gravity-compensated PD effort from Python every
  control step; at the control rate that PD limit-cycled (joint1 pinned at
  -100 Nm, arm never at rest) and cost ~8 ms per step. The hold is now
  PhysX's own drive at the latched pose (stiffness 600, damping 80, 100 Nm
  ceiling) with only the gravity term fed forward and refreshed every 30
  control steps. Stopped step 13.3 -> 4.5 ms; the arm sits within 0.005 rad
  at ~zero velocity, and the published joint efforts during a hold are now
  computed with the hold gains (they used to report the nominal 20 000
  stiffness).
- **Target pushes.** Isaac Lab applies actuator groups with two Warp launches
  per group; tinker2 has five, so every target push (arm trajectories, the
  wheel slew ramp, hold refreshes) paid ten launches, ~3.2 ms of a ~3.6 ms
  push. The backend now binds a fused `_apply_actuator_model` that launches
  each kernel once over all groups -- bit-identical staging and telemetry
  buffers, push 4.3-6.4 -> 1.8 ms. `TINKER_SIM_STOCK_ACTUATOR_MODEL=1`
  restores Isaac Lab's loop.
- **Camera conversions.** Frames are copied into pinned host buffers with one
  stream sync per cycle, RGB conversion writes into reused scratch buffers,
  and depth metres->16UC1 mm is a Warp kernel on the GPU (`wp.rint`, i.e.
  banker's rounding like `np.rint`); all byte-identical to the reference
  implementations kept in `camera_rig.py`, proven by
  `tests/test_camera_publish_equivalence.py` on synthetic edge cases and 120
  real frames. Camera stage 23 -> 13 ms per cycle. Note: with the optional
  `--camera-pointcloud` flag (off in this runbook) the cloud is now built
  from the millimetre depth, i.e. quantised to 1 mm.
  With `TINKER_SIM_PROFILE=1` the profile line now also carries a
  `camera_breakdown_ms` (capture / rgb_convert / depth_convert / image_fill /
  image_publish / info).

That breakdown is how the development-lidar ray-cast was found to cost
~35 ms per lidar frame (~350 ms per simulated second); it is now vectorised
and bit-identical (`OccupancyMap.raycast_many`), at ~2–5 ms per frame;
that alone took the default sensor-rich run from RTF 0.35 to 0.40.

## 2026-08-21 — Physics/control cadence, Kit pump and solver probes

Text moved from the runbook when it was reduced to current operation only.

### Camera cadence under a live stack

Export `TINKER_SIM_CAMERA_HZ=15` before Stage 1 for any run with the full
GPSR stack attached.

The simulator holds RTF ~0.30 on its own, but drops to ~0.06 once real
subscribers attach: Kit is pumped (both RTX cameras rendered) once per camera
stride, and the image payloads are serialised on the same beat, all inside the
step loop. At that speed Nav2 cannot service its lifecycle bonds and tears its
own stack down — `Switch controller timed out after 2.000000 seconds`, then
every navigation node deactivates, leaving no `map -> odom` and an unusable TF
tree. Halving the cadence halves both costs.

`simulation/sensors/hardware-parity.json` stays authoritative and unedited:
unset, its 30 Hz is used. The override may only *lower* the rate — publishing
faster than the real camera would be a parity violation, and is refused.

### Control cadence under a live stack (`TINKER_SIM_CONTROL_HZ`)

Export `TINKER_SIM_CONTROL_HZ=60` (with `TINKER_SIM_CAMERA_HZ=12`; 15 if the
vision stack needs it) before Stage 1 for live-stack runs. Do **not** use
`TINKER_SIM_PHYSICS_HZ=60` for that purpose any more. 12 Hz is exact at
control 120 and 60 (strides 10 and 5); at control 30 it rounds to 15 Hz.
Measured 2026-08-21 at control 60: cameras 15 Hz RTF 0.70, 12 Hz 0.77, and
0.80 once depth conversion moved to the GPU. Simulator VRAM on that run:
peak 2.6 GB, steady 2.3 GB (the 2026-08-20 code ran 2.7-3.6 GB).

The simulator has two rates. The *physics* rate (`TINKER_SIM_PHYSICS_HZ`,
default 120, may only be lowered) is PhysX's solver step — the thing every
contact/grasp result was validated at. The *control* rate
(`TINKER_SIM_CONTROL_HZ`, default = physics rate) is how often Isaac Lab,
the joint-target writes, the wheel slew, the gateway publish and `/clock`
run. With the control rate lowered, each control step runs
`physics_hz / control_hz` explicit solver substeps of the validated
1/120 s, so the solver trajectory is unchanged while every per-step wrapper
cost is paid 60 times a second instead of 120. The control rate must divide
the physics rate evenly and has a 30 Hz floor. Note omni.physx's
`IPhysxSimulation.simulate(elapsed)` does *not* substep on its own — it
integrates exactly `elapsed` — which is why the substeps are explicit and
why simply lowering `dt` was a fidelity change, not an optimisation.

Measured 2026-08-21 (gpsr-rcw2026, rcw2026 arena, GPU 1, RTF =
simulated / wall):

| run | physics-only (no ROS, no cameras) | sensor-rich + ROS, cameras 15 Hz, start of day | same, end of day (all fixes below) |
|---|---|---|---|
| default 120 / control 120 | 0.75 | 0.35 | **0.64** |
| `TINKER_SIM_CONTROL_HZ=60` | 1.18 | 0.44 | **0.70** |
| `TINKER_SIM_CONTROL_HZ=30` | 1.88 | 0.50 | **0.78** |
| `TINKER_SIM_PHYSICS_HZ=60` (old advice) | 1.61 | 0.51 | — (not needed any more) |

The sensor-rich numbers are with the robot safety-stopped (no bridge
attached), which is the state a run spends its start-up and every
command-loss interval in; the "end of day" column includes the lidar,
safety-hold, actuator-launch and camera fixes described further down.

Robot root position after 10 s idle agreed with the default to within
0.5 mm at control 60/30 and drifted 5 mm at physics 60 — the substepped runs
keep the validated solver trajectory, the lowered physics rate does not.
Lower control rates also lower the IMU (200 Hz parity) and base-state
(50 Hz) publish cadences, which are derived from the control step: at 60 Hz
they publish at 60 Hz, at 30 Hz they publish at 30 Hz.

What still bounds RTF under the live stack, per simulated second at control
60: the PhysX solve itself (~9 ms per control step, 32 authored position
iterations) and the Kit render pump for both RTX cameras (~30 ms per camera
frame). The pump was probed on 2026-08-21 and is *not* render-mode, GI,
async-rendering or readback bound: `RaytracedLighting` instead of the
default `RealTimePathTracing`, `/app/asyncRendering=true`, and
reflections/indirect-diffuse/AO off each changed it by <1 ms; an empty Kit
update is ~2 ms, and the cost scales with camera pixel count (~19 ms head,
~7 ms wrist). It is Kit's per-render-product frame pipeline at the parity
resolutions, and the only remaining levers are structural. Two further
opt-in knobs exist: `TINKER_SIM_SOLVER_POSITION_ITERATIONS` /
`TINKER_SIM_SOLVER_VELOCITY_ITERATIONS` override the robot USD's articulation
solver iteration counts (32 / 1); 8 position iterations measured RTF 0.52
at control 60 but *changes drive and contact convergence*, so it is not
recommended for anything that produces evidence. PhysX worker-thread count
(`--/persistent/physics/numThreads`, default 8) made no measurable
difference at 4 or 16.

With `TINKER_SIM_PROFILE=1`, every profile line now also carries
`physics_breakdown_ms.physx_substeps` and a `publish_breakdown_ms`
(clock / joint_state / imu / cloud / status / truth per `publish()` call).

## 2026-08-29: gpsr-spawn-spike — live scene-spawn viability

Ran `scripts/gpsr-spawn-spike` against the live stack (Nav2 + sim
PLAYING + bridge, `gpsr-stack up --scenario gpsr-rcw2026-bench --sim-gpu 1`)
per the command-driven-scene design
(docs/superpowers/specs/2026-08-28-command-driven-scene-and-sim-identity-design.md,
section 2.4): `tools/gpsr_spawn.py plan`+`apply` for "count the
pudding_box on the kitchen_table", checked `/clock` monotonicity, Nav2
health, and entity presence, then `clear`.

Results:
- plan: PASS
- apply (spawn_entity while PLAYING): PASS — `/get_entities` lists
  `/World/Scenario/cmd_pudding_box_0` afterwards; `GetEntityState` reads
  (2.5, -3.0, 0.734).
- arena-camera frame shows the pudding_box: INCONCLUSIVE — a 10 cm YCB
  object is ~3 px at the arena camera's 960 px; even the base scenario's
  soup/mug are not discernible. Entity-list check used instead.
- /clock monotonic across apply (20 samples before, 90 after; 65.43 -> 66.92 s): PASS
- Nav2 goto bedroom and back after the spawn: goals accepted, robot drove
  and ended at (-1.87, -1.78) ~ command point. Each leg hit the 170 s
  wall cap before reporting SUCCEEDED because RTF is ~0.2 with the arena
  camera on (sim clock 189 s after ~15 min wall) — a cadence limit, not a
  Nav2 fault.
- clear (delete_entity): PASS — entity absent from `/get_entities` afterwards.

Three things the spike caught that the unit tests could not:
1. `python3 tools/gpsr_spawn.py` failed with `ModuleNotFoundError: tools`
   when run as a script (fixed: repo root inserted on sys.path).
2. Presence-based (asset, spot) dedupe skipped instances 1-2 of a count
   command, and slot 0 coincides with the base scenario's soup pose at
   kitchen_table (fixed: count-aware dedupe + grid slots offset by the
   objects already at the spot).
3. `ros2 topic echo` via the ROS daemon intermittently dies with
   `!rclpy.ok()` — the spike uses `--no-daemon`.

**Decision:** outcome A — tier-2 uses runtime spawning per run:
`--spawn-cmd "scripts/gpsr-scene-apply --command {command} --seed {seed}
--plan {plan} --manifest {manifest}"` (plan+apply in one command) and
`--clear-cmd "python3 tools/gpsr_spawn.py clear --manifest {manifest}"`.
The bench environment must carry the vendored `simulation_interfaces`
Python overlay (`.ros-vendor/humble/opt/ros/humble/{local/lib,lib}/python3.10`)
on PYTHONPATH — the battery script already does.

Next: re-run `s2026-003`, `004`, `005` (old run dirs archived as
`*.attempt1-no-scene`). Acceptance: 003/004 no longer fail on absent
objects; 005 passes the `person_found` gate under
`GPSR_SIM_IDENTITY_RELAXED=1`.

## 2026-08-19 — Vision live-acceptance round-trip: recorded result + real tk26_vision defect

Ran the three-terminal live acceptance check (sim on `--sensor-profile
sensor-rich` with `--arena-colors`, the Humble vision overlay's `get_image`,
and `tests/ros_humble/test_vision_get_image_live.py` under
`TINKER_SIM_VISION_LIVE=1`) end to end on `ROS_DOMAIN_ID=42`. See "Live
acceptance runbook" under Vision hardware-parity cameras in the README for
the exact commands.

This was run and recorded (3/3 passed, `ROS_DOMAIN_ID=42`): direct RELIABLE
subscriptions decode both cameras with `tk26_vision` conventions, and the
wrist frame carried all six palette hues (45.3% chromatic pixels). Evidence
is under `reports/vision-roundtrip/` (gitignored, host-local).

**Real defect found in `~/tk25_ws/src/tk26_vision`** (documented here for the
record; out of scope to patch from this repo): `vision_util`'s `get_image`
and `get_point_cloud` register `async def` callbacks directly on
`message_filters`' `ApproximateTimeSynchronizer` (`get_image.py:49,65,77,81`,
`get_point_cloud.py:43,66,100,104`). Humble's `message_filters` invokes
callbacks synchronously, so those coroutines are never awaited, the node's
cached frames never update, and both services always answer "No camera
data" — on real hardware too, not only in sim. The sim run still proved that
delivery and stamp-pairing both work: the node's own "coroutine was never
awaited" `RuntimeWarning`s fired for both cameras. The acceptance test's
third case probes this service directly and will fail loudly — prompting a
test upgrade — once the node is fixed upstream.

Status: development-validated with a recorded live round-trip; **not
release-qualified**.

## 2026-08-17 through 2026-08-20 — RoboCup 2026 arena import: validation evidence and open findings

Validation performed on this branch (development-validated only):

- unit suites are green, with a stable failing/erroring name-set matched
  against this repo's pre-existing environmental failures (see "Developer
  verification" in the README);
- both artifacts hash-verify clean (`verify_asset_artifact(...) == []`) and
  re-import is a proven byte-identical no-op;
- visual/collision AABB agreement was spot-verified: within 1.4mm on three
  YCB objects (cracker box, mug, bowl); arena furniture bounds were checked
  against the upstream SDF within the configured 0.02m tolerance, with two
  documented per-model exceptions (`rcw26_door`, `rcw26_sink`) whose upstream
  SDFs deliberately under-size the collision box (a trimmed door-panel depth
  and a floor-anchored sink height, both for gripper-reach affordance, not
  data errors);
- a headless streaming smoke (`validation/run_sim.py --sensor-profile
  navigation-parity --profile parity --scenario empty --seed 7 --headless
  --livestream --arena rcw2026 --duration 45`) ran the full 45 simulated
  seconds to a clean exit, with only headless-windowing/driver diagnostic
  warnings in the log and no importer-scratch-path leakage;
- a live end-to-end `./scripts/launch-arena-streaming --arena rcw2026` run
  (2026-08-18) reached streaming readiness — ready file written, TCP 49100
  listening, `viewport_ready: true`, arena `robocup-arena3` with 38
  colliders loaded — and stayed up awaiting a client;
- sensor-rich camera imagery of the arena's furniture: head and wrist
  hardware-parity color/depth frames captured live against `--arena
  rcw2026` (shelf close-ups plus a base-rotation panorama showing the TV
  cabinet, trash bin, door, tiled floor, and plant), with per-frame content
  statistics — `reports/arena-sensor-rich-2026-08-18/`; the wrist camera's
  mount/intrinsics and arm-following viewpoint were verified separately in
  `reports/arena-arm-camera-2026-08-18/`;
- AMCL convergence on the derived map: with the spawn moved to a free cell
  (`--spawn-xy=-2.0,-2.0`) and the Humble stack pointed at the arena map
  (`map_yaml:=...`), AMCL locked to physics-truth within 0.07 m after
  seeding and, over truth-validated gentle-motion runs, contracted to
  position variance 0.035/0.050 m² (std ~0.2 m) at 0.14 rad yaw error —
  `reports/arena-amcl-2026-08-18/SUMMARY.md`, which also records two real
  findings: the default (0, 0) arena spawn sits inside `shelf_02`'s
  footprint (hence the new `--spawn-xy` override), and sustained in-place
  skid-steer rotation accumulates wheel-odometry yaw slip that drags the
  filter (a base-odometry characteristic, not a map defect).

- head pan/tilt effort override: with the backend's `effort_limit_sim`
  override extended to the `head` actuator group, commanded pan/tilt
  converged from spawn to within the 0.05 rad tolerance of a (1.0, -0.3)
  rad target — final (0.9506, -0.2853) rad at sim t=0.317 s — closing the
  2026-08-18 finding that pan/tilt drives inherited the URDF's 1.0 Nm
  effort cap. Live-proven:
  `reports/arena-fixes-2026-08-19/head-tracking.json`;
- physics interaction (an object resting on arena furniture): the
  pick-deliver-place `delivery_object` (0.08 m cube), spawned via the
  standard `spawn_entity` path at its declared z=0.8 pose, fell onto and
  came to rest statically (zero twist across 387 truth samples) on a 0.5 m
  board of `rcw26_shelf` —
  `reports/arena-scenario-spawn-2026-08-19/object-spawn-verification.md`.
  **Superseded 2026-08-20**: this evidence came from `/get_entity_state`
  polling, not physics truth; a later run under a profile that reports
  physics truth found no trace of the object in it at all. See "Scenario-
  spawned objects may not be physics-simulated" under Known arena
  limitations below — that finding is SUSPECTED, not confirmed, so treat
  this bullet as open again rather than either closed or refuted;
- scenario entity spawning in the arena: `scenario_runner` executed
  find-and-approach-person and pick-deliver-place against an `--arena
  rcw2026` sim with every operation accepted; the person capsule and task
  cube spawn at their exact declared world poses (verified via
  `/get_entities`/`/get_entity_state` and `expected_objects` truth
  correlation), and spawned entities are live rigid bodies
  (`/set_entity_state` round-trips) —
  `reports/arena-scenario-spawn-2026-08-19/`. Caveats: nothing implements
  scenario `events` (the person's `actor_path_start` walk never runs), and
  scenario poses were authored for the procedural world (the arena has no
  pedestal at the object spawn; the person's declared pose sits in
  furniture-dense space).

Not yet validated (open):

- textured-frame visual confirmation by a human viewer (the streaming
  session above is up for exactly this; connect with NVIDIA's client).

Known arena limitations (development findings, 2026-08-18 through 2026-08-20):

- the default robot spawn (0, 0) lies inside `shelf_02`'s physical and
  rasterized footprint. The launch now fails closed on this instead of
  spawning into it: `validate_arena_spawn()` exits non-zero and names the
  nearest free cell in the error (`arena spawn (0.0, 0.0) lacks 0.35 m
  clearance on the derived map; try --spawn-xy=-0.4,0.4`) — pass the
  suggested `--spawn-xy` for navigation work (see launch docs above).
  Live-proven: `reports/arena-fixes-2026-08-19/spawn-fail-closed.log`
  (exit 1, suggestion printed);
- under `sensor-rich`, an occupied dev-lidar sensor origin (for example the
  spawn-in-`shelf_02` case above) used to publish a dense ring at ~0.3 m
  for every ray. The 2026-08-18 note here misdescribed this as an RTX-lidar
  self-hit; there is no RTX lidar in this path, and the ring was the
  occupancy raycast's minimum-range floor being returned when every ray
  starts inside an occupied cell. An occupied ray origin now publishes an
  empty cloud instead. Unit-proven only, no live run has targeted this
  path: `tests/test_ros_gateway.py`
  (`RosDevelopmentLidarTest.test_development_lidar_empty_when_origin_occupied`);
- wheel-velocity commands are slew-limited to 60 rad/s² (≈3.1 m/s² linear
  at the 0.0525 m wheel radius), set deliberately ABOVE Nav2's `acc_lim`
  (~2.5 m/s²) so planner-shaped velocity profiles pass through unchanged —
  the bound exists to floor non-planner commanders and stale-target
  transients, not to shape Nav2 output. Live-proven: after a 30 s in-place
  rotation, an idle base commanded to coast drifted only 8.0e-05 m in XY
  over the following 30 s, far inside the 0.1 m bound —
  `reports/arena-fixes-2026-08-19/coast.json`;
- sustained in-place skid-steer rotation accumulates wheel-odometry yaw
  slip (a base-odometry characteristic, not a map defect — see the AMCL
  validation note above). This is a dead end for IMU fusion: the sim IMU
  publishes only world-frame angular velocity, marks
  `orientation_covariance[0] = -1.0` (REP-145 "orientation not provided"),
  and never populates linear acceleration, so fusing it into an EKF would
  only duplicate odom's own vyaw rather than correct it.

Found on 2026-08-20, while the live evidence wave was closing out the
fixes above:

1. **Scenario-spawned objects may not be physics-simulated.** Isaac logs
   `Physics tensor entity not valid for rigid body /World/Scenario/<id>`
   and the object was observed holding its exact spawn pose with
   fabricated zero velocities. This is SUSPECTED, not confirmed: it was
   seen through a run whose profile could not report objects at all
   (fixed since, commit `02d1785`), so it may prove to be an artifact of
   that. This supersedes the 2026-08-19 claim above that the "object
   rests on furniture" item was closed — that evidence came from
   `/get_entity_state`, not physics truth. Evidence:
   `reports/arena-fixes-2026-08-19/object-on-table.json`.
2. **`ROS_DOMAIN_ID` trap.** `.deployment.env` sets `ROS_DOMAIN_ID=25` and
   `scripts/launch-humble` defaults to `${ROS_DOMAIN_ID:-25}`, so sourcing
   `.deployment.env` silently overrides the 42 that live arena runs use.
   Export 42 AFTER sourcing, in every shell, including the one that runs
   `launch-humble`. `.deployment.env` must still be sourced — it carries
   the Isaac EULA acceptance variable.
3. **`scenario_runner` needs `PYTHONPATH` under a bare `ros2 run`.** It
   imports `tinker_sim_core` at module level (`scenario_runner.py:22`),
   so CLI invocations need `PYTHONPATH=$PWD/simulation:$PYTHONPATH`.
   `actor_path_driver` does NOT need this — it resolves
   `tinker_sim_core` from `--root` as of commit `bd4b553`.

Status: development-validated only, **not release-qualified**.

## 2026-09-05: fabric-off cost + opt-in spawn-yaw-via-view (#24, unvalidated)

Profiling flagged `TINKER_SIM_USE_FABRIC=0` (forced off by 13e4fdf whenever
`TINKER_SIM_SPAWN_YAW` is non-zero) as the single largest non-physics RTF
cost found: `updateToUsd` -- PhysX writing every rigid body's transform back
to USD every step, plus the Hydra scene-notice resync that write triggers --
profiles at roughly 1.1 s of wall per simulated second. 13e4fdf's own root
cause for forcing fabric off is narrower than that cost: `omni.physx.fabric`
resolves a *newly-spawned sibling body's* initial world transform against
the robot root's non-identity yaw when fabric is authoritative (i.e. when
`use_fabric=True` sets `/physics/updateToUsd=False`), so a scenario object
spawned after a yawed robot boot lands rotated by `-robot_yaw` about the
robot's origin in physics, even though USD/`get_entity_state` read back the
commanded pose correctly. Fabric-off makes USD authoritative for that
ingestion and removes the leak, at the `updateToUsd` cost above. A full
dependency audit (`/home/tinker/.claude/jobs/01ca17b4/tmp/fabric-off-dependencies.md`)
found this SPAWN_YAW mislocation is the *only* reason fabric is off anywhere
in this repo -- every other fabric-off-adjacent fix (arena/gripper/YCB
friction materials, #12's `set_entity_pose_physics`, `_apply_base_hold`) is
either a tensor-view write that is already fabric-independent by
construction, or a USD-authoring compensation for a second-order defect
(mesh colliders not inheriting the PhysicsScene default material under
`updateToUsd`) that is harmless under fabric-on.

**Hypothesis** (untested beyond static reading + CPU unit tests): the
pre-reset USD `xformOp:orient` write that authors the spawn yaw is not
itself required to be a USD write -- `_apply_base_hold` and #12's
`set_entity_pose_physics` already write the robot's root pose through the
Isaac Lab / PhysX tensor view (`write_root_pose_to_sim_index` /
`create_rigid_body_view(...).set_transforms`), which is fabric-independent.
If the spawn yaw is instead written through that same view, *after*
`sim.reset()` binds the articulation, fabric would not need to be forced
off at all for the SPAWN_YAW path. This is a **candidate fix, not a proven
one**: 13e4fdf's own diagnosis was that the defect's trigger is the robot
root carrying a non-identity yaw *in physics* at the moment a sibling body
spawns -- a condition switching the write mechanism does not obviously
change, since the robot root ends up at the same yawed pose in physics
either way, just via a different write path. It needs a live GPU A/B, not
more source reading.

**Shipped, default-off, current behaviour unchanged when unset**:
`TINKER_SIM_SPAWN_YAW_VIA_VIEW=1` (`simulation/tinker_sim_isaac/backend.py`).
When set and `TINKER_SIM_SPAWN_YAW` is non-zero: `use_fabric` stays as
`resolve_use_fabric` would otherwise compute it (normally `True`, i.e. NOT
forced off), the pre-reset USD orient authoring is skipped entirely, and
once the articulation is bound and reset, `_apply_spawn_yaw_via_view` writes
the commanded yaw as a root-view quaternion (`(x, y, z, w)` order -- NOT the
`(w, x, y, z)` order `InitialStateCfg.rot` and the USD `xformOp:orient` path
use; this tripped up the initial reading of the vendored IsaacLab source and
is called out explicitly in both the code and its tests) before the first
physics step, and seeds `_base_hold_pose`/`_base_hold_vel`/
`_base_hold_scene_sig` directly so `TINKER_SIM_FIX_BASE=1`, if also active,
holds the yawed pose immediately rather than only picking it up once its own
2 s settle-latch fires. A `{"event":"spawn_yaw","via":"view"|"usd",
"use_fabric":...}` boot-log line records which path ran, alongside the
pre-existing `{"use_fabric":...,"spawn_yaw_set":...}` line, so a live boot's
stdout says unambiguously which path it took.

**Fix round 1 (same day, code review `/home/tinker/.claude/jobs/01ca17b4/tmp/task24-yawview-review.md`, Finding 1 -- CONFIRMED)**:
the first cut of this flag only applied the yaw once, at the initial boot
bind (right after `sim.reset()`). Every standard scenario boot performs
reset -> STOP -> `spawn_entity`(s) -> PLAY (`simulation_interfaces`'
`/reset_simulation` + `/set_simulation_state`, the default
`spawn_while_playing=False` flow -- see `simulation/README.md` and
`simulation/tinker_sim_core/orchestration.py`), and Isaac Lab recreates the
articulation root view on that PLAY/PHYSICS_READY transition
(`_refresh_robot_handles` already detects and handles the new view identity
for every other piece of cached state). Since the pre-reset USD
`xformOp:orient` authoring is deliberately skipped under this flag, the
commanded yaw had no durable USD backing -- it lived only in the transient
root-view tensor buffer -- so that reset/rebind silently snapped the robot
back to the USD/`InitialStateCfg`-composed identity orientation, *right
after the spawn sequence this flag exists to make cheap*. With
`TINKER_SIM_FIX_BASE=0` (the nav-profile default) nothing ever re-applied
the yaw for the rest of the run; with `FIX_BASE=1`, `_apply_base_hold`'s
Python-cached target happened to survive and re-fix it, but only after one
physics step at identity yaw.

Fixed by moving the reapply into `_refresh_robot_handles` itself, via a new
`_reapply_spawn_yaw_after_rebind` helper: it runs on every genuine view-identity
change that method detects (the initial boot bind included, so the explicit
call the first cut made in `__init__` right after `sim.reset()` is now
redundant and was removed), covering all three call sites that can rebind
the articulation view -- `__init__`'s boot path, `step()`'s re-entrant
PHYSICS_READY rebind branch, and `_maybe_recover_simulation_view`'s manual
view recreation -- from one place instead of three. It re-reads the
*current* root position from the freshly (re)bound view rather than
replaying a cached boot-time position, so it stays correct even if the base
moved before the reset. Confirmed safe to call before any explicit
`Articulation.update(dt)`: IsaacLab's `root_link_pose_w` is a
timestamp-checked proxy that re-fetches from the physics view on every
access regardless of `.update()` (`.deps/IsaacLab/.../articulation_data.py:616-630`),
so there is no stale-buffer window. The one unavoidable residual from the
review (not fixed, and not fixable without changing Isaac's own
PHYSICS_READY dispatch order): `step()`'s rebind branch must still run one
`_sim.step()` at whatever orientation the just-recreated view reports
*before* it can call `_refresh_robot_handles()` again successfully -- so a
single physics step at the pre-reapply orientation is unavoidable on every
reset, same as it always was for every other piece of state that branch
re-synchronizes.

Non-GPU coverage (`tests/test_manipulation_runtime.py`,
`UseFabricDerivationTest` + `SpawnYawViaViewApplyTest` +
`SpawnYawViaViewRebindTest`): the `use_fabric` derivation truth table (yaw
set + flag off -> False, unchanged; yaw set + flag on -> True; no yaw ->
True regardless of flag; `TINKER_SIM_USE_FABRIC` override still wins over
the new flag), the view-write itself (issues the root-pose write exactly
once, position taken from the articulation's already-resolved `root_pos_w`,
quaternion matching a pure yaw in `(x, y, z, w)` order, and the base-hold
seeding only firing when `base_fixed` is set), and the rebind-durability
fix (`_refresh_robot_handles` reapplies the yaw and re-seeds the base-hold
target on a forced view-identity change, does so again on a second,
independent rebind, does nothing when the view identity is unchanged, and
does nothing at all when the flag is off or unset) -- all against a backend
test double. These cannot and do not exercise a real PhysX root view,
fabric's sibling-spawn ingestion, the actual PHYSICS_READY dispatch timing,
or whether the hypothesis above actually holds.

**Fix round 2 (same day, re-review `/home/tinker/.claude/jobs/01ca17b4/tmp/task24-yawview-review-r1.md`,
new finding -- CONFIRMED)**: fix round 1 closed Finding 1 correctly but
introduced a regression: `_reapply_spawn_yaw_after_rebind` ran on *every*
view-identity change `_refresh_robot_handles` detected, with no distinction
between a genuine reset (physics state reset to the initial pose anyway --
safe to reapply) and `_maybe_recover_simulation_view`'s mid-run,
state-PRESERVING view recovery (reachable any time via
`_heal_detached_scenario_bodies`, budgeted 5 uses/boot, whose own docstring
says it deliberately skips `force_load_physics_from_usd` specifically so
bodies keep their live/settled pose across the recovery). A nav-profile
robot (`FIX_BASE=0`) that had driven/turned to a live heading would get that
heading silently snapped back to the stale `TINKER_SIM_SPAWN_YAW` boot value
on the next such recovery; with `FIX_BASE=1` the base-hold reseed made the
wrong heading persistent rather than a one-off glitch. The review's point:
"is this a reset" cannot be inferred from the measured pose (a driven
robot's live heading is indistinguishable from a stale one by pose alone),
so the classification has to be explicit.

Fixed by giving `_refresh_robot_handles` an explicit
`reapply_spawn_yaw: bool = True` keyword, decided by the CALLER, never
inferred: the two genuine-reset callers (`__init__`'s boot bind and
`step()`'s PHYSICS_READY rebind branch) take the default and are unchanged;
`_maybe_recover_simulation_view`'s call now explicitly passes
`reapply_spawn_yaw=False`, so that recovery path never writes a root pose
and never touches `_spawn_yaw` or an existing base-hold target -- matching
what the USD-authoring path already does today (no code re-authors
`xformOp:orient` outside boot, so a state-preserving recovery already left a
flag-off robot's live heading alone; this makes the flag-on view-write path
behave the same way).

Non-GPU coverage added (`tests/test_manipulation_runtime.py`,
`SpawnYawViaViewRebindTest.test_boot_bind_reapplies_yaw` +
`SpawnYawViaViewRecoveryRebindTest`): boot (`_robot_view_identity=None`)
reapplies; a genuine-reset rebind (default `reapply_spawn_yaw=True`)
reapplies (already covered in round 1); a recovery-classified rebind
(`reapply_spawn_yaw=False`) issues no root-pose write at all and leaves a
pre-set `_spawn_yaw` and a driven, pre-latched `_base_hold_pose`/
`_base_hold_vel`/`_base_hold_scene_sig` completely byte-identical to what
they were before the call, while still performing the rest of the rebind
(joint index caches, clock re-anchoring, view-identity bookkeeping) exactly
as before. Full suite: 87 passed, 3 subtests passed, 0 failed.

This is a unit-tested invariant only -- no GPU harness in this repo can
currently trigger `_maybe_recover_simulation_view` on demand (it fires from
a caught tensor-view exception, not a controllable command), so there is no
live check analogous to the reset-survival one below for this path; see the
validation recipe's note on this.

**Not done here**: any GPU boot. The validation recipe --
`/home/tinker/.claude/jobs/01ca17b4/tmp/fabric-on-validation-recipe.md` --
lays out the exact A/B (spawn-yaw truth-pose comparison against 13e4fdf's
repro shape, robot root yaw from `/sim/internal/physics_truth`, and
`TINKER_SIM_PROFILE=1`'s `step_profile.kit_pump`/RTF numbers) for whoever
runs it next, now including a reset-survival check for Finding 1's fix
(spawn through the standard reset cycle, then re-read the robot's base yaw
from truth and confirm it is still the commanded value, not identity). Do
not treat this flag as validated, and do not flip any default based on this
entry alone.

## 2026-09-06: spawn-yaw-via-view + FIX_BASE held the base at its un-settled spawn height (#26)

**Symptom**: with `TINKER_SIM_SPAWN_YAW_VIA_VIEW=1` and `TINKER_SIM_FIX_BASE=1`
both active, the held base-frame z came out ~0.1954 instead of the ~0.0775
settled rest height the flag-off (USD-authored-yaw) path produces -- an
11.8 cm base-frame error, first caught on a bench round ("agu") comparing
base pose against the flag-off baseline.

**Root cause**: `_apply_spawn_yaw_via_view` (`simulation/tinker_sim_isaac/backend.py`)
runs once the articulation is bound and `self._sim.reset()` has returned,
*before the first physics step* -- exactly per its own docstring. The #24
fix round that added FIX_BASE seeding (see the 2026-09-05 entry above,
"seeds `_base_hold_pose`/`_base_hold_vel`/`_base_hold_scene_sig` directly")
took `data.root_pos_w` at that same pre-physics moment and latched it
straight into `_base_hold_pose`. That is the un-settled spawn height
(`InitialStateCfg.pos`'s z, ~0.20, minus a small articulation-resolve
offset) -- the free-base chassis has not yet had gravity drop it onto its
wheels/casters. Because `_base_hold_pose` was now non-None from step 0,
`_apply_base_hold`'s own settle-latch (`if self._base_hold_pose is None: if
self.simulation_time < self._base_hold_after_sim_s: return`, gated on a
2.0 s wait) never fired -- the branch that would otherwise have captured
the settled pose was permanently skipped, so the pre-settle height was held
for the entire run instead. The flag-off path never seeds anything here (it
is guarded out of `_apply_spawn_yaw_via_view` entirely, which flag-off never
calls), so it always went through the settle-latch and got the correct
settled height -- which is exactly why the two paths disagreed.

**Fix**: `_apply_spawn_yaw_via_view` no longer seeds `_base_hold_pose`'s
position at all. It now only remembers the commanded yaw quaternion in a
new pending field, `_base_hold_seed_quat`, and (when `base_fixed`) clears
any already-latched `_base_hold_pose`/`_base_hold_vel` back to `None` --
necessary on a genuine rebind (STOP -> spawn -> PLAY), since nothing else
re-drives the settle-latch once `_base_hold_pose` is non-None, and a
genuine rebind resets the chassis back to its unsettled spawn pose just
like boot did. `_apply_base_hold`'s existing settle-latch is otherwise
unchanged -- it still waits for `simulation_time >= _base_hold_after_sim_s`
-- except that when it fires, it now composes the freshly-read (and by then
settled) `root_pos_w` with `_base_hold_seed_quat` if one is pending, instead
of the measured `root_quat_w`, so the commanded yaw still survives into the
hold exactly as the #24 fix intended. Flag-off behaviour is unchanged
(`_base_hold_seed_quat` stays `None` for that path, so the latch falls back
to the measured orientation exactly as before). `_reapply_spawn_yaw_after_rebind`'s
rebind classification (`reapply_spawn_yaw`, #24 round 2) was not touched --
it still decides whether this whole method runs at all, independent of what
it does internally.

**Validation gap that let this ship**: the #24 GPU validation recipe
(`fabric-on-validation-recipe.md`) did exercise `FIX_BASE=1` and passed its
"spawn clean, reset survival" checks, but those checks read only
`robot.base_pose.quaternion_xyzw` -- orientation persistence across a
rebind -- never `base_pose.xyz`'s z-height against a settled baseline. It
was checking a different axis of correctness than the one that broke.

Non-GPU coverage (`tests/test_manipulation_runtime.py`,
`SpawnYawViaViewApplyTest.test_base_hold_latches_settled_height_with_seeded_yaw`,
new): seeds the yaw with `root_pos_w` z=0.20 (un-settled), mutates the mock's
`root_pos_w` to z=0.0775 (settled) and advances simulated time past
`_base_hold_after_sim_s`, then calls `_apply_base_hold` directly and asserts
the latched hold's z is 0.0775 (not 0.20) *and* its orientation is the
seeded yaw (not whatever `root_quat_w` happens to read at latch time). This
test fails against pre-fix `backend.py` (`_base_hold_pose` is not `None`
immediately after `_apply_spawn_yaw_via_view`, so the settle-latch branch
never runs). `test_seeds_base_hold_target_when_fix_base_active`,
`test_does_not_seed_base_hold_when_fix_base_inactive`, and
`test_reapplies_yaw_on_rebind_when_via_view_active` were updated to assert
the new contract (yaw seeded into `_base_hold_seed_quat`, `_base_hold_pose`
left/reset to `None`, not immediately re-latched). Full suite:
`tests/test_manipulation_runtime.py` 103 passed, 3 subtests passed, 0
failed. Still no GPU boot for this fix -- same caveat as #24 above.

**Round 2 (same day): the settle window is measured from process BOOT, not
from a rebind, so the fix above reproduced its own bug on every mid-run
reset.** Code review (`$TMP/task26-review.md`) caught it before a GPU gate:
`_apply_base_hold`'s settle-latch gates on `if self.simulation_time <
self._base_hold_after_sim_s: return` -- a one-shot, absolute 2.0 s deadline
from `simulation_time == 0`. But `_reapply_spawn_yaw_after_rebind` (->
`_apply_spawn_yaw_via_view`) runs on every genuine rebind, not just boot --
`step()`'s PHYSICS_READY branch calls it after every standard scenario
STOP -> spawn -> PLAY cycle -- and it unconditionally clears
`_base_hold_pose`/`_base_hold_vel` back to `None` each time (the round-1
fix above, needed so the settle-latch runs again after a respawn). Because
`simulation_time` is deliberately kept MONOTONIC across a rebind (#21's own
fix, `_clock_step_origin` re-anchored to the elapsed step count instead of
re-zeroed), a mid-run reset happening minutes into a live run finds
`simulation_time` already well past 2.0 s the instant the clear runs.
`_apply_base_hold`'s very next call therefore sees `_base_hold_pose is
None` *and* the boot-relative deadline already satisfied, skips the
settle-wait branch entirely, and re-latches immediately from
`data.root_pos_w` read right after the respawn -- the exact pre-settle
height bug the round-1 fix targeted, now recurring on every subsequent
reset for the rest of the run instead of once at boot. Flag-off has no
equivalent defect: it never clears `_base_hold_pose` on a rebind (its
branch of `_apply_spawn_yaw_via_view` is skipped entirely), so it just
keeps reasserting whatever was latched at boot and never re-enters the
`is None` branch.

**Fix**: track the settle deadline relative to the LAST CLEAR, not process
boot, on the via-view branch only. New field `_base_hold_settle_from`
(`__init__`, initialised to `0.0` next to `_base_hold_after_sim_s` -- boot
behaviour is unchanged, since `simulation_time - 0.0` is just
`simulation_time`). `_apply_spawn_yaw_via_view` now records
`self._base_hold_settle_from = self.simulation_time` in the same
`if getattr(self, "base_fixed", False):` block that clears
`_base_hold_pose`/`_base_hold_vel` -- so every clear (boot or a genuine
mid-run rebind) re-arms its own 2.0 s window. `_apply_base_hold`'s gate
branches on whether the via-view path owns the hold
(`_base_hold_seed_quat is not None`): when it does, the check becomes
`simulation_time - _base_hold_settle_from < _base_hold_after_sim_s`;
otherwise (flag-off, `seed_quat is None`) the original boot-relative
`simulation_time < _base_hold_after_sim_s` check runs unchanged, so
flag-off stays byte-identical.

New test (`SpawnYawViaViewRebindTest.test_rebind_after_boot_waits_for_settle_before_relatching`):
fakes 40.0 s of elapsed sim time (well past the 2.0 s deadline) *before*
triggering a rebind via `_refresh_robot_handles()` with `root_pos_w`
z=0.20 (pre-settle), then calls `_apply_base_hold()` immediately and
asserts `_base_hold_pose` is still `None` (not re-latched). It fails
against a495958 with:
```
E       AttributeError: 'IsaacWholeRobotBackend' object has no attribute '_base_hold_settle_from'. Did you mean: '_base_hold_resettle_s'?
```
(the field didn't exist pre-fix). Advancing simulated time by exactly 2.0 s
more with `root_pos_w` z=0.0775 and calling `_apply_base_hold()` again
asserts it now latches at the settled height with the seeded yaw intact.
A flag-off counterpart, `test_flag_off_rebind_keeps_previously_latched_hold`,
asserts a rebind with `_spawn_yaw_via_view = False` never touches an
already-latched `_base_hold_pose`/`_base_hold_vel` -- byte-identical to
pre-#26 behaviour, confirming this round's change doesn't give flag-off a
settle timer it never had. Full suite:
`tests/test_manipulation_runtime.py` 105 passed, 3 subtests passed, 0
failed. Still no GPU boot for either round of this fix.

## 2026-09-06: YCB object origins uncentered on the mesh -- soup can 8.5cm off (#32)

**Symptom**: bench truth for spawned YCB objects reads consistently off from
where the physical object actually sits -- reported worst case ~8.6cm for
the tomato soup can, with contacts (the only geometry-true signal) as the
tell. This is independent of the retention work in #17/#20: friction and
mass were already correctly authored on these assets, but the *origin*
PhysX reports as the object's rigid-body pose was never the origin the
object's own mesh is centered on.

**Root cause**: `tools/tinker_sim_deploy/arena_convert.py::_compose_object`
(via `convert_object_to_usd`) wraps each object's raw upstream DAE (visual)
and STL (collision) conversions under `/World/geom` and `/World/collision`
with only a unit-scale `xformOp:scale` -- no recentring translate is ever
authored. The module's own design rationale ("every allowlisted object's
SDF declares identity visual/collision poses, so this importer needs no
per-model scale/pose correction") only establishes that the mesh file isn't
offset *relative to its SDF link*; it says nothing about whether the raw
mesh's own vertex data is centered on that link's origin. It is not: the
upstream `tmc_wrs_gz_worlds` YCB scans carry over whatever the turntable/
scan reference frame happened to be. Measured directly against the
published `object.usd` files (`UsdGeom.BBoxCache` on the collision mesh,
in the object's own `/World` frame): the soup can's collision footprint is
`(-0.0429, 0.0508, 0.0003)..(0.0244, 0.1175, 0.1016)` -- its Y range does
not even contain Y=0, i.e. the tracked rigid-body origin sits ~5cm *outside*
the can's own body along Y (8.46cm full XY offset magnitude). Every
harness-spawned object's truth is read straight off that same rigid-body
origin (`backend.py::_iter_spawned_bodies` / `_spawned_object_states`, via
`physx.get_rigidbody_transformation` on the referenced object's `/World`
prim) with no correction, so the defect propagates end-to-end: PhysX
truth, `/sim/truth/object_state`, and `/spawn_entity` placement are all
that far from the physical mesh, independent of the collider geometry
itself (convex-decomposition, correctly sized) being fine. The Z
convention (base anchored at collision bbox min Z = 0) was already correct
by design and is preserved exactly.

**Fix**: `arena_convert.recenter_object_origin(stage)` -- a pure-pxr step,
mirroring the existing `author_object_*` authoring functions -- computes
the collision mesh's AABB in the object's own `/World` frame and authors a
corrective `xformOp:translate` on both the `geom` and `collision` wrapper
Xforms (same numeric offset on both, so they stay coincident) so the
collision bbox's XY centroid lands at (0, 0); the Z component of the
translate is `-bbox_min_z`, a no-op to floating-point noise for objects
already satisfying the base-anchor convention. Per this module's own
xformOp-ordering convention (top-of-file docstring: earlier-added ops end
up outermost), the translate has to be spliced in *ahead of* the existing
scale op rather than simply appended -- `_prepend_translate` reorders
`xformOpOrder` accordingly, and updates an existing `recenter` translate
in place rather than stacking a second one on a repair re-run.
`_compose_object` now calls it during import, right before the
rigid-body/preview-surface authoring; `ycb_import.recenter_physics`
(mirroring `repair_physics`) re-runs it standalone against an
already-published artifact -- no Kit, no GPU, no upstream checkout needed
-- and a new `--recenter` CLI flag wires it into `ycb_import.py`, alongside
the existing `--repair-physics`.

New tests (`tests/test_ycb_object_recenter.py`, pxr-importorskip'd): a
synthetic composed-object stage (mirroring `_compose_object`'s structure)
with a mesh offset by `(0.05, 0.08, 0.0)` under an already-base-anchored Z
-- asserts the recentred collision bbox centroid lands within 1e-6 of the
origin, min Z stays at 0, and the visual/collision wrappers receive the
identical translate (they stay coincident); plus idempotency and a
missing-collision-child failure-closed case. A second test,
`test_every_published_ycb_object_is_recentred_on_its_origin`, walks every
`object.usd` this checkout's `artifacts/asset-manifest.json` references
under `objects/ycb/` and asserts each collision bbox centroid is within
5mm of (0, 0, 0); it skips cleanly when no local `artifacts/` store is
present (the binaries are gitignored). Run directly against the live
artifact store at `/home/tinker/tinker-sim/6.0.1/artifacts/`: all 10
objects failed the 5mm check against the pre-fix identity `f342a496...`
(soup can 92.4mm 3D / 84.1mm XY, sugar box 18.2mm, ..., bowl 46.4mm) and
all 10 pass exactly (centroid and min Z reported as `0.000000` to six
decimals) against the republished identity below.

**Republish**: ran `tools/ycb_import.py --recenter --root
/home/tinker/tinker-sim/6.0.1` against the live artifact store (old
identity `f342a496fc34fa2a1d2721cef5e03cb629d74ec41fa0d8f68b5e6b1e42edc866`
preserved untouched for provenance -- confirmed no file under it newer than
the new `current.json`). New content-addressed identity
`b533a2e5dff60c79ab0f4ec9c6cee2f568735c036ac2b7dc684e12590ad41cca`; mass
and friction material authored by #17/#20 verified carried over unchanged
onto the recentred geometry (soup can: rigid body + mass 0.349kg + static/
dynamic friction 0.8/0.7, spot-checked). `current.json` and
`asset-manifest.json` (10 `generated_object_usds` entries) auto-repointed
by the CLI; scenario `asset_uris` in `simulation/scenarios/gpsr-rcw2026.json`
and `gpsr-rcw2026-bench.json` (4 objects each) manually repointed, same
flow as the #17 friction/mass migration.

Per-object `(dx, dy, dz)` translate applied (metres, printed by the CLI):

| object | dx | dy | dz |
|---|---|---|---|
| ycb_001_cheez-it | 0.0131 | 0.0149 | 0.0031 |
| ycb_002_sugar_box | 0.0074 | 0.0166 | -0.0001 |
| ycb_005_spam | 0.0333 | 0.0267 | 0.0030 |
| ycb_006_mustard_bottle | 0.0152 | 0.0234 | 0.0030 |
| ycb_008_pudding_box | -0.0006 | -0.0192 | 0.0004 |
| ycb_010_tomato_soup_can | 0.0092 | -0.0841 | -0.0003 |
| ycb_011_banana | -0.0115 | 0.0072 | 0.0002 |
| ycb_021_bleach_cleanser | 0.0216 | -0.0117 | 0.0007 |
| ycb_024_bowl | 0.0151 | 0.0436 | 0.0006 |
| ycb_025_mug | 0.0085 | -0.0179 | 0.0006 |

`tests/test_ycb_object_recenter.py` (new), `tests/test_ycb_physics_repair.py`,
`tests/test_arena_convert.py`, and `tests/test_ycb_import_cli.py` all green.
## 2026-09-06: contact-report force divided by the control-tick dt, not the physics dt (#29)

**Symptom**: bench facade `contact_force_n` reads 1.8-3.5 N/finger at
`TINKER_SIM_CONTROL_HZ=30` (agw run) vs 6-8 N/finger at the 120 Hz default
(agx run) on grasps that look otherwise identical. A clean A/B probe
(identical drive/pad/tilt trajectory, only `TINKER_SIM_CONTROL_HZ` changed)
reproduced it in isolation: 20.74 N at 120 Hz vs 5.12 N at 30 Hz --
`0.247 ~= 1/4`, and drive angle / pad position tracked within 1-2% of each
other at every sampled instant across the whole run, so the closure itself
was not different, only the reported force.

**Root cause**: `_on_contact_report_event` (`backend.py`, formerly line
3729) computed `normal_force = sum(impulse) / self.dt`. PhysX's
`subscribe_contact_report_events` callback fires once per solver SUBSTEP,
i.e. once per `physics_dt` (`1/physics_hz`), independent of `control_hz`.
`self.dt` is `1/control_hz` (`backend.py:574`), and
`physics_substeps = physics_hz/control_hz` substeps run per control tick
(`backend.py:573`, `1218-1227`). At the default `control_hz == physics_hz`
(120/120), `physics_substeps == 1` and `self.dt == self.physics_dt`, so the
formula happened to be correct -- masking the bug until `TINKER_SIM_CONTROL_HZ`
was lowered. At 30 Hz control / 120 Hz physics, `physics_substeps == 4`, so
every reported force was an impulse from one `physics_dt`-sized window
divided by a `self.dt` that is 4x too large: exactly the observed
20.74 N / 5.12 N ratio.

**Consumers of the bug**: everything that reads `contact_pairs()` or
`contact_state()` inherited the same rate-dependent 4x-at-30Hz
under-report -- the probe's `lf`/`rf` columns, `/sim/truth/contacts`,
`/sim/parity/finger_contact`, and (most importantly) the gripper facade's
`contact_force_n` gate: a force-based stall/success threshold at
`control_hz < physics_hz` would need up to `physics_substeps`x more real
force to cross the same nominal threshold than it does at 120 Hz, i.e. it
silently gets harder to satisfy exactly when control_hz is lowered for RTF.

**PR #16 was a wrong theory, now closed**: commit `75388ee` ("gripper mimic
mirror runs per physics substep") moved `_ramp_drive_target` /
`_mirror_gripper_mimic_targets` into the per-substep loop, theorizing the
follower PD's one-step velocity feed-forward was stale for the extra
substeps at low `control_hz`. It never touched `_on_contact_report_event`
or the force formula, and per the task's measurement record it left the
20.74 N / 5.12 N gap completely unchanged -- confirming the mimic-mirror
cadence and the contact-force formula are unrelated code paths, and that
`75388ee`'s theory does not explain this symptom. That commit is not on
this branch's history; this fix targets the actual formula bug instead.

**Fix**: `normal_force = sum(impulse) / self.physics_dt`. The callback
receives no per-step `dt`/`current_time` argument (only `contact_headers`,
`contact_data`), so there is no "actual step dt" to prefer over
`physics_dt` -- `physics_dt` is exactly the substep's own fixed integration
window and is the correct, and only available, divisor.

**Audit of other `self.dt` sites in `backend.py`** (grepped every
`/ self.dt`, `* self.dt`, and `self.dt` near contact/impulse/report/wrench;
`ros_gateway.py` has no dt-based contact math, it only reads
`contact_state()`):
- `backend.py:1180`, `2685`, `2756` (`self._robot.update(self.dt)`): called
  once per `step()` (one control tick), after `_step_simulation()` has
  already run all `physics_substeps` PhysX steps for that tick -- the
  correct elapsed time for that single buffer refresh is the control-tick
  duration, `self.dt`. No change.
- `backend.py:2252` (`_slew_wheel_targets`, `WHEEL_VELOCITY_SLEW_RAD_S2 *
  self.dt`) and `backend.py:2601` (`_ramp_drive_target`, `slew * self.dt`):
  both are called exactly once per `step()` (`step()`'s single call to
  `_slew_wheel_targets()`/`_ramp_drive_target()`, not inside the substep
  loop in `_step_simulation`), so the per-tick slew rate correctly uses the
  control-tick `self.dt`. No change.
- `backend.py:2685` region / `_mirror_gripper_mimic_targets`'s one-step
  velocity feed-forward (`joint_vel * self.dt`): also called once per
  `step()`, same reasoning -- the feed-forward spans one control tick. No
  change. (This is the code path `75388ee` theorized about and moved
  per-substep; that move is not present here and this fix does not
  reintroduce it -- no live evidence ties it to a real symptom.)
- `backend.py:754` (`SimulationCfg(dt=self.physics_dt, ...)`),
  `backend.py:1744` (`simulation_time`, `steps * self.physics_dt`), and
  `backend.py:2835` (`PhysxManager.update_simulation(get_physics_dt(), ...)`):
  already used `physics_dt` correctly; left untouched.

**Tests** (`tests/test_manipulation_runtime.py`): added
`test_contact_report_force_divides_by_physics_dt_not_control_dt` --
`physics_hz=120`, `control_hz=30` (`physics_dt=1/120`, `dt=1/30`,
`physics_substeps=4`), one contact event with impulse `0.05`, asserts the
recorded force is `6.0` (`0.05 * 120`) and explicitly not `1.5`
(`0.05 * 30`, the pre-fix value). Fails on the pre-fix formula with:
```
E       AssertionError: 1.5 != 6.0 within 7 places (4.5 difference)
```
Added `test_contact_report_force_at_matched_rates_is_byte_identical_path`
(`physics_hz == control_hz == 120`, `physics_substeps=1`) asserting the
120/120 case is unchanged by the fix (`self.dt == self.physics_dt` there,
so both formulas agree). Updated the four pre-existing contact-report
tests (`test_contact_report_uses_identified_bodies_and_reported_normal`,
`..._sums_normal_impulses_without_tangential_cancellation`,
`..._uses_deterministic_normal_for_degenerate_average`,
`..._first_event_logged_exactly_once`) to set `backend.physics_dt = 0.1`
instead of `backend.dt = 0.1`, since the divisor variable changed; their
expected `normal_force` values are unchanged because they set only one dt
field and it now maps to the divisor the formula actually uses. Full
suite: `tests/test_manipulation_runtime.py` 111 passed, 3 subtests passed,
0 failed (was 109 passed pre-change; +2 new tests).

**Not addressed here**: `$TMP/task29-measurement-check.md`'s bench
divergence (soup-can/sugar-box drive-angle and squeeze-depth differing
between 30 Hz and 120 Hz control) is a separate, unconfirmed
control_hz-dependent effect in the close/stall-detection cadence, not
reproduced by the clean side-pinch probe and not explained by this
measurement bug -- it needs its own live investigation.

## 2026-09-07 — TINKER_SIM_TRACK_OBJECTS log flood: blind per-tick PhysX queries of absent prims (#38)

**Symptom**: `_log_tracked_objects` (opt-in via `TINKER_SIM_TRACK_OBJECTS`,
built for the 2026-08-31 vanishing-spawn investigation) fires at its own
~4 Hz cadence and, on every fire, calls
`IPhysx.get_rigidbody_transformation(path)` for *every* path in
`_tracked_object_paths` -- parsed blind from the env var with no existence
check. A bench round logged 1424 bursts of the underlying carb/omni.physx
ERROR line ("did not locate any object" / "Error executing
getRigidBodyTransformation") because two of the tracked paths were spawned
late (or already removed) and every tick before/after that queried them
anyway. The Python side already handled the failure gracefully (`ret_val`
check, try/except), but that C++-level ERROR log line is emitted from
inside the PhysX call itself, before it ever returns to Python, so no
amount of Python-side error handling suppresses it.

**Cause**: unlike `_iter_spawned_bodies` (the "good pattern": re-derives
its candidate list from `/World/Scenario`'s stage children, filtered by
`RigidBodyAPI`, once per `_object_discovery_interval`, so it structurally
cannot blind-query a path for more than one interval), `_log_tracked_objects`
never re-derives or memoizes presence at all -- it queries the same static,
env-parsed path list forever, for the whole backend lifetime, regardless of
whether the prim ever existed or has since been removed.

**Fix**: added `_resolve_tracked_objects()`, mirroring `_iter_spawned_bodies`'s
discipline: at the `_object_discovery_interval` cadence (driven from
`step()`, same as `_refresh_object_views`/`_heal_detached_scenario_bodies`),
check each tracked path against the live stage
(`stage.GetPrimAtPath(path).IsValid()` + `HasAPI(UsdPhysics.RigidBodyAPI)`)
and maintain a `_tracked_object_resolved` set. `_log_tracked_objects` now
only calls `get_rigidbody_transformation` for paths in that set -- an
unresolved path is skipped every tick, not queried-then-caught. Logging is
transition-based (mirroring the single-shot guard pattern used elsewhere,
e.g. `_contact_report_first_event_logged`): "resolved" once when a path
first becomes present, "missing" once when a previously-present path
disappears, "unresolved" once for a path that has never resolved --
instead of a per-tick state print. Two in-repo code paths that confirm a
tracked prim's rigid body just went live get an immediate `force=True`
re-check instead of waiting out the discovery interval:
`_heal_detached_scenario_bodies` (when a watched spawn transitions to
attached) and `set_entity_pose_physics` (when a park view is first
resolved for a prim path). Output format for paths that do resolve is
unchanged.

**Tests** (`tests/test_manipulation_runtime.py`, net-new -- no prior test
covered `TINKER_SIM_TRACK_OBJECTS`/`_log_tracked_objects` at all): a fake
`omni.usd`/`pxr` stage stub (`_fake_omni_usd_and_pxr`) whose `GetPrimAtPath`
re-checks a live, test-mutable `present_paths` set, plus a fake
`omni.physx` module whose `get_rigidbody_transformation` asserts if called
with a path outside an allowed set.
- `test_log_tracked_objects_never_queries_unresolved_paths`: two present +
  two absent paths, several ticks -- zero PhysX queries reach the absent
  paths, present paths are queried every tick as before, and each absent
  path logs exactly one "unresolved" line (not once per tick).
- `test_tracked_object_resolves_on_spawn_without_waiting_for_interval`: a
  path absent at first (`_object_discovery_interval=1000`, so a plain tick
  would not naturally refresh) becomes present, and a forced re-check
  (`force=True`, the spawn/park hook path) resolves it immediately -- one
  "resolved" log line, and the very next `_log_tracked_objects()` call
  queries it.
- `test_tracked_object_stops_querying_after_despawn`: a present, queried
  path disappears; the next `_resolve_tracked_objects()` call notices,
  logs exactly one "missing" line, and no further PhysX query reaches that
  path afterward (repeat ticks do not re-log "missing").

Fail-first check: with the pre-fix `backend.py` (no
`_resolve_tracked_objects`), all three new tests fail with
`AttributeError: 'IsaacWholeRobotBackend' object has no attribute
'_resolve_tracked_objects'`. Full suite (ROS-sourced, lark-shim on
PYTHONPATH, `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`):
`tests/test_manipulation_runtime.py` **132 passed, 3 subtests passed** (129
baseline + 3 new).

**Not addressed here**: the design doc for this task also traced a related
"Physics tensor entity not valid ... velocities set to zero" warning to the
vendored `isaacsim.ros2.sim_control` extension's `/get_entity_state` handler
-- an out-of-tree call site this repo does not own and has no fix point
for; left open.
