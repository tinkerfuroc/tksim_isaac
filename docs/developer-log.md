# Developer log

Dated engineering notes: what was measured, what was ruled out, why a fix
took the shape it did. Operational instructions live in
`docs/gpsr-sim-runbook.md`; this file is the history behind them.

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

**Residual (not fixed) — low-speed yaw deadband.** With both fixes the robot
physically arrives within 0.16–0.21 m of the goal and the estimate within
0.1 m, but the final yaw trim never completes: estimated yaw error 0.12–0.16
rad against a 0.1 rad tolerance, DWB commands ~0.09 rad/s, and the base does
not move. Measured response (truth yaw rate vs command): **0.1 → 0.00, 0.2 →
0.07, 0.3 → 0.13, 0.5 → 0.27, 0.8 → 0.50**; under small commands the wheel
joints read exactly 0.0 with the drive effort at its cap. Then
`SimpleProgressChecker` (XY only, 0.5 m in 10 s) aborts, the spin recovery
adds ±1.57 rad, and it repeats — the loop the handoff described, now on top
of a real base deficiency. Ruled out, one variable at a time: drive effort
cap (10 N·m vs 80 N·m: identical response), articulation velocity solver
iterations (8 vs the authored 1: identical), caster swivel damping (above).
The deficit is independent of drive torque, so it is a contact-solution
effect, not a force limit. Next hypothesis, untested: PhysX stabilization /
sleep thresholds on the articulation freezing slow contacts
(`ArticulationRootPropertiesCfg(stabilization_threshold=0.0,
sleep_threshold=0.0)`); after that, the wheel collider (plain `Cylinder`
prims, no physics material).

**Nav2-side observations for the navigation owners** (tk26_navigation, not
this repo): `tracking_goal_checker` (`yaw_goal_tolerance: 3.14`) exists in
`nav2_dwb_params.yaml` but is commented out of `goal_checker_plugins`; with a
≥0.15 rad/s deadband a 0.1 rad yaw tolerance cannot be met in sim, and the
XY-only progress checker converts every station-keeping yaw trim into a
recovery.

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
