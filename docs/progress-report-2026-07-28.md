# Tinker Isaac Simulation Progress Report — 2026-07-28

## Purpose and evidence policy

This report records the work completed during the current Tinker whole-robot
simulation session, the validation evidence that exists, the limitations that
remain, and the reviewed plan for proceeding with end-to-end manipulation.

The authoritative isolated project is:

```text
/home/tinker/tinker-sim/6.0.1
```

The existing Tinker workspace remains a read-only source/reference:

```text
/home/tinker/tk25_ws
```

No claim in this report treats a unit test, action return code, configured
profile, or façade startup as proof that a physical simulation behavior passed.
Machine-generated reports and raw runtime observations are identified wherever
they exist. Missing live evidence is reported as unqualified work.

## Executive status

| Area | Current status | Evidence level |
| --- | --- | --- |
| Isolated deployment project | Implemented | Project and workspace provenance manifests |
| Frozen Python/Isaac environment | Implemented and locally reproduced offline | `uv.lock` and deployment reports |
| GPU/Isaac compatibility | Development pass | Compatibility, RTX, NVENC and streaming reports |
| CPU PhysX behavior runtime | Validated for smoke/navigation use | 10,000-step report and live navigation run |
| ROS 2 Humble boundary | Implemented and live-tested | Python-boundary and DDS observations |
| Standard simulation control | Implemented and live-tested | Standard services/action discovery, reset/pause/step tests |
| Navigation | Development validated | Live Nav2 and wheel-derived odometry report |
| Manipulation | Development runtime and live diagnostic qualification scaffold validated; physical task qualification pending | Unit, Humble, launch, headless, rosbag, exact-truth, and clean-teardown evidence; no physical manipulation qualification |
| Vision | Configured; live module qualification pending | Standalone RTX camera/LiDAR smoke only |
| Audio | Deterministic fixture contract ready | Contract/unit evidence |
| Decision | Pending vertical slice | No end-to-end result |
| VLA | Deferred until façades are qualified | No end-to-end result |
| Release qualification | Not achieved | Driver, RAM, calibration and clean-server blockers remain |

## Work completed during this session

### 1. Isolated, reproducible deployment

- Created and moved the standalone simulation project outside `tk25_ws`.
- Kept the source workspace read-only and recorded source locks/provenance.
- Defined the supported Ubuntu 22.04, x86_64 and GLIBC 2.35 deployment baseline.
- Pinned CPython 3.12.13 and `uv` 0.10.8.
- Pinned Isaac Sim 6.0.1.0.
- Pinned Isaac Lab to `v3.0.0-beta2.patch1`, exact commit
  `ffff603eafc6b74264a5261cc0183d6a65390d78`.
- Pinned PyTorch 2.11.0, TorchVision 0.26.0 and TorchAudio 2.11.0 using CUDA
  12.8 wheels, plus Pillow 12.2.0.
- Made `uv.lock` the single Python dependency authority. Miniconda remains a
  recovery bootstrap only.
- Added explicit Omniverse EULA acceptance through deployment configuration;
  scripts do not silently accept it.
- Added online/offline bootstrap tooling, frozen synchronization, cache audit,
  offline bundle creation/restoration and checksum manifests.
- Added machine-readable deployment reporting and content-addressed robot
  artifacts.

Relevant definitions:

- [`../pyproject.toml`](../pyproject.toml)
- [`../uv.lock`](../uv.lock)
- [`../release-manifest.json`](../release-manifest.json)
- [`../scripts/bootstrap`](../scripts/bootstrap)
- [`../scripts/create-offline-bundle`](../scripts/create-offline-bundle)
- [`../scripts/restore-offline-bundle`](../scripts/restore-offline-bundle)

### 2. Deployment and hardware validation

The host GPU was accessed successfully:

```text
NVIDIA GeForce RTX 5070 Ti
VRAM: 16303 MiB
Driver: 570.211.01
```

Completed validation includes:

- Isaac compatibility checker pass.
- Frozen offline environment synchronization without network fallback.
- Python 3.12/ROS Python-path boundary pass.
- 10,000 CPU PhysX steps with GPU physics disabled.
- RTX camera RGB output and RTX LiDAR output initialization.
- Real NVENC hardware encoding.
- Isaac cache prewarming.
- ROS overlay build.

The host is suitable for development experiments but is not release-qualified:
driver 570.211.01 is below the supported 595.58.03 baseline, and approximately
33.3 GB RAM is below the supported 64 GB whole-robot recommendation.

Primary evidence:

- [`../reports/deployment-20260725T074007Z.json`](../reports/deployment-20260725T074007Z.json)
- [`../reports/deployment-20260728T072313Z.json`](../reports/deployment-20260728T072313Z.json)
- [`../reports/headless-physx-latest.json`](../reports/headless-physx-latest.json)
- [`../reports/rtx-sensors-latest.json`](../reports/rtx-sensors-latest.json)

### 3. Native NVIDIA WebRTC validation

The streaming test was revised to comply with the requirement not to download
or use a browser. NVIDIA's Ubuntu Isaac Sim WebRTC Streaming Client 2.0.0 was
used as a native executable.

The post-relocation soak established:

- a real client/server connection;
- continuously advancing decoded video for 70.003403 seconds;
- NVENC activity during the connection;
- no external browser;
- tester-initiated teardown;
- terminated client and simulator processes;
- closed streaming ports and removed the viewer lock.

Evidence:

- [`../reports/webrtc-native-client-soak-relocation-20260725T074310Z.json`](../reports/webrtc-native-client-soak-relocation-20260725T074310Z.json)
- [`../reports/webrtc-native-client-soak-relocation-20260725T074310Z.client.log`](../reports/webrtc-native-client-soak-relocation-20260725T074310Z.client.log)

### 4. ROS 2 Humble and Isaac boundary

The integration was organized as two cooperating processes:

- Isaac runs Python 3.12 with Isaac's internal Humble libraries.
- Tinker, Nav2, ros2_control, hardware façades and other ROS processes run
  system Humble with Python 3.10.

DDS is the boundary. Both sides share ROS domain 25,
`rmw_fastrtps_cpp`, and the selected Fast DDS profile. Isaac launch scripts
reject accidental `/opt/ros/humble` or Python 3.10 paths in the Isaac process.

NVIDIA's `isaacsim.ros2.sim_control` provides the standard
`simulation_interfaces` lifecycle surface. Earlier custom `/sim/control/*` and
`/sim/scenario/*` aliases are intentionally forbidden.

Live validation observed:

- `/clock` and `/isaac_joint_states` across the Python boundary;
- 19 standard simulation services;
- `/simulate_steps`;
- successful standard reset with simulation clock reset;
- pause, forced step and resume behavior;
- no legacy custom-control services.

Relevant definitions:

- [`../contracts/simulation.yaml`](../contracts/simulation.yaml)
- [`../scripts/launch-isaac`](../scripts/launch-isaac)
- [`../scripts/launch-humble`](../scripts/launch-humble)
- [`../config/fastdds.xml`](../config/fastdds.xml)

### 5. Robot artifact and backend-neutral contracts

- Exported a content-addressed Tinker 2 URDF/USD/map artifact.
- Added deterministic scenario definitions for person approach,
  pick/deliver/place and reception/seat assignment.
- Added parity/truth separation and evaluator-only truth contracts.
- Added a deterministic command-owner model with one intended publisher of
  `/isaac_joint_commands`.
- Added the hardware-parity base, arm, gripper and pan-tilt interface contracts.
- Added standard scenario orchestration through `simulation_interfaces`.
- Added postcondition evaluation logic that rejects constructed claims when
  required world postconditions are false.

Artifact entry point:

- [`../artifacts/robot/tinker2/current.json`](../artifacts/robot/tinker2/current.json)

The evaluator logic and object-state acquisition are now connected to the
manipulation-core runtime. Live task arbitration and a qualified physical
pick/place remain incomplete; evaluator and runtime tests do not constitute a
successful simulated pick/place.

### 6. Navigation integration

Navigation reused Tinker's existing Gazebo/Nav2 foundation and adjusted only
the simulator-facing boundary for Isaac:

- Isaac publishes clock, joint state, Livox LiDAR and IMU surfaces.
- The base façade converts `/cmd_vel` to four wheel commands.
- Odometry is wheel-derived rather than copied from hidden truth.
- The point-cloud adapter supplies `/scan`.
- EKF owns `odom -> base_link`.
- AMCL owns `map -> odom`.
- A contract guard checks topic types, sole articulation command ownership and
  dynamic TF ownership.

Live development validation passed:

- AMCL initial localization;
- `map -> odom -> base_link` TF chain;
- Nav2 lifecycle activation;
- direct base command and command watchdog stop;
- two successful `NavigateToPose` goals;
- standard pause/step/resume control;
- clean ROS/Isaac shutdown with no residual processes.

This is a development pass, not release qualification. The current
`navigation-parity` point cloud uses deterministic CPU raycasting and is not an
RTX LiDAR parity claim. Hardware calibration is still missing.

Evidence and instructions:

- [`../integration/NAVIGATION.md`](../integration/NAVIGATION.md)
- [`../reports/navigation-integration-20260725T085604Z.json`](../reports/navigation-integration-20260725T085604Z.json)

### 7. Tinker module integration survey

A module-by-module sweep established the following integration state:

- **Navigation:** development validated using the existing Nav2 foundation.
- **Hardware gateway:** implemented around standard joint-state command/state
  topics and sole command ownership.
- **Manipulation:** external ros2_control FJT, xArm, gripper and pan-tilt façades
  exist, but have not passed live manipulation qualification.
- **Vision:** official RTX-facing configuration exists, but Tinker vision nodes
  have not completed a live vertical slice.
- **Audio:** deterministic non-acoustic dialogue fixtures implement the contract.
- **Decision:** intended to remain an unmodified external node and receive
  success only from postcondition evaluation; vertical slice pending.
- **VLA:** deferred until hardware façades and safety behavior are qualified.

Machine-readable summary:

- [`../integration/modules.json`](../integration/modules.json)
- [`../reports/module-integration-20260728.json`](../reports/module-integration-20260728.json)

## Implementation follow-up — 2026-07-28

The implementation work following the audit is recorded here. This section
supersedes the audit's implementation-status conclusions where it lists a
completed item, while retaining the audit and forward plan below as historical
review material. It does not claim live manipulation qualification.

Implemented:

- The command mux composes finite mixed base, arm and gripper commands and
  uses measured joint positions for holds. Layered safety stop handling now
  freezes held targets, aborts active gripper/FJT work, and bounds gripper
  effort. Command epochs reject delayed pre-stop packets, and safety is
  fail-safe from launch start.
- Safety sources publish heartbeats with expiry, consumers fail closed on
  stale input, the Isaac command stream times out independently, and
  gateway-authoritative session/generation epochs reject stale or replayed
  command snapshots across restarts.
- The manipulation-core profile runs CPU PhysX with the content-addressed
  actual Tinker artifact. Object views are acquired after scenario spawn, and
  body-identified PhysX contact reports cover the gripper, TCP and monitored
  arm links.
- Scenario orchestration follows reset -> `STOPPED` -> spawn-all -> `PLAYING`,
  and an external Humble launch provides the ROS-side controller and façade
  boundary.
- Graph ownership is explicit: there is one command publisher and one raw
  physics-truth evaluator subscriber. Task and planning nodes do not consume
  evaluator truth.
- The evaluator accepts object-free frames and computes retention in TCP
  coordinates with TCP-relative quaternion checks.
- Qualification scaffolding writes explicit approved bag topics, source and
  artifact provenance, and finalized evidence files. External gates are
  reported as executed-but-unverified and do not establish qualification.

Current verified evidence:

- Generic `.venv` suite: `194 passed, 3 skipped, 1 warning`.
- Sourced Humble manipulation partition: `54 passed`.
- Humble overlay: `2` packages built.
- Installed manipulation launch accepts `--show-args`.
- Manifest-only qualification exits `0`; the not-configured qualification
  path exits `1`.
- Clean headless no-ROS manipulation-core smoke exits `0`; a missing-scenario
  smoke exits `1`.
- Live two-process manipulation-core startup serialized, configured and
  activated both controllers. The scenario completed reset -> `STOPPED` ->
  spawn -> `PHYSICS_READY`, emitted measured object and evaluator output, the
  contract reported `pass`, and safety settled to `false`.
- The final live qualification-scaffold attempt was ROS domain `146`, recorded
  at `/tmp/tinker-sim-live-qualification18/20260728T131149.899568Z-343348-023c44d930`.
  Readiness passed the contract check with safety `false`,
  `xarm7_traj_controller` active, measured object truth present, and the
  scenario at `PHYSICS_READY`. It verified one effective safety publisher,
  one command publisher, and one raw-truth subscriber. The rosbag confirmed a
  recorder subscription for every approved topic with the expected QoS
  baselines. The `/bin/true` diagnostic gate remained
  `executed-unverified`; the overall result remained `unverified` with a
  nonzero exit. Post-gate Isaac, Humble, and rosbag health checks passed;
  planned SIGINT exits were all `0`; exactly `7149` raw/evaluator frames were
  correlated and drained with no mismatches or evaluator errors; no orphan
  cleanup was required; no Tinker process or GPU allocation survived; and the
  finalized evidence set was complete and readable.

This is not a live retention, FJT, gripper, collision or pick-place pass. The
diagnostic gate performed no physical task. Built-in gate executors and
independent metric recomputation remain absent. MoveIt and cuMotion remain
deferred, and release qualification has not been achieved.

## Manipulation-core milestone implementation — 2026-07-28

The manipulation-core plan was implemented after the diagnostic scaffold
described above. Qualification has not yet passed, so this section records an
implemented but unqualified milestone.

Implemented:

- Added built-in executors and independent verifiers for free-space FJT,
  safety stop, free and obstructed gripper, arm collision, and retention.
- Added strict gate-window clipping, raw/evaluator frame correlation, command
  ownership checks, bounded action cancellation, physical predicate checks,
  and fail-closed evidence parsing.
- Removed the executor's raw-truth ROS subscription. Physical predicates now
  read the evaluator-owned evidence stream, preserving one raw-truth
  subscriber.
- Added deterministic qualification fixtures, including a fixed pedestal and
  dynamic cube placement aligned with the predetermined grasp pose.
- Replaced the launch-time one-shot parameter race with a bounded ROS service
  handshake before scenario execution.
- Added bounded visual-capture latency, reframed overview and manipulation
  cameras, journal-bound keyframes, and separate user and agent contact sheets.
- Added per-attempt process and GPU ownership accounting. Cleanup compares
  per-GPU memory and graphics/compute process tables against the pre-attempt
  baseline and rejects owned survivors or unexplained memory.
- Hardened rosbag startup and finalization. Startup requires a live recorder,
  an open SQLite database, the exact approved topic set from the recorder's own
  subscription log, and a validated QoS override file. Finalization requires
  every approved topic in metadata with a positive message count and required
  QoS, plus a clean recorder exit. Fast DDS graph endpoint names are retained
  as diagnostics because they remained unresolved for otherwise confirmed
  recorder subscriptions.
- Added post-stop command baseline resynchronization that does not apply a
  partial multi-packet snapshot and does not release the fail-closed hold until
  a complete post-clear command baseline is accepted.
- Implemented a single explicit safety actuator path using PhysX gravity
  compensation plus bounded measured-state PD effort. Implicit gains are zero
  while stopped, non-arm explicit efforts are zero, the measured position
  target remains frozen, and nominal gains and limits are restored on clear.
  This follows the pinned Isaac Lab 3.0 beta PhysX API, including the floating
  base generalized-coordinate offset.

Current non-live verification:

- Generic suite: `309 passed, 3 skipped, 1 warning, 6 subtests passed`.
- Sourced Humble manipulation partition: `73 passed, 1 skipped`. The skipped
  scenario-runner test requires `simulation_interfaces`, which is supplied by
  the Isaac runtime rather than system Humble.
- Humble overlay rebuild: `tinker_sim_interfaces` and `tinker_sim_bridge`
  finished successfully.

Live evidence and current verdict:

- The first complete six-gate suite is preserved at
  `/tmp/tinker-manipulation-core-live-20260728/suite-20260728T143904.262540Z-871895-9c751697fe`.
  It correctly returned `evidence-invalid`; it exposed FJT timing, launch
  readiness, fixture, raw-truth ownership, visual skew, and safety-hold
  failures that were subsequently repaired in source.
- The latest gate-executing safety attempt is preserved at
  `/tmp/tinker-manipulation-core-final-safety6-20260728/20260728T155931.540056Z-1394404-ecf0452ae7`.
  The then-current implicit-only hold failed physically, reaching
  `0.2046 rad/s` and `0.0233 rad` drift. That controller was replaced; the
  failed evidence was not deleted or reclassified.
- Attempts using the final gravity-compensated controller did not reach the
  gate because an unrelated compute process identified by NVIDIA accounting as
  PID `1457672` occupied approximately `15.2 GiB` of VRAM. Warp could not
  create its CUDA stream even though manipulation physics is configured for
  CPU. The qualification runner did not kill or adopt that process.
- Every completed teardown reported no attempt-owned PID or GPU survivor. At
  idle, the observed GPU baseline and final usage were both `446 MiB`. During
  external contention, baseline and final usage were both `15674 MiB`, with
  the same unrelated process present before and after the attempt.
- The externally blocked launches did not terminate cleanly enough for
  qualification: Isaac exited nonzero after planned SIGINT and the runner
  forcibly removed an orphaned `omni.telemetry.transmitter` descendant.
  No descendant or GPU allocation survived cleanup, but forced cleanup remains
  a qualification failure and is not described as a clean process teardown.

The manipulation-core milestone is therefore **not yet qualified**. The code,
tests, fixtures, recorder contract, and visual evidence pipeline are in place,
but the final gravity-compensated safety controller still requires one clean
live gate. Free-space, gripper, collision, and retention then require a fresh
six-gate suite and independent artifact review. MoveIt, cuMotion, vision,
decision, and VLA remain outside this milestone.

Continuation details, the 2026-07-29 focused safety evidence, mirror-only
changes, and the ordered qualification path are recorded in
[`manipulation-core-handoff-2026-07-29.md`](manipulation-core-handoff-2026-07-29.md).
At the user's request, the manipulation qualification checksum/index layer
was removed on 2026-07-29 without weakening physical verification, rosbag
validation, truth correlation, ownership checks, or process/GPU cleanup.

Contact sheets:

- [`../reports/manipulation-core-20260728/contact-sheet-user.png`](../reports/manipulation-core-20260728/contact-sheet-user.png)
- [`../reports/manipulation-core-20260728/contact-sheet-agent.png`](../reports/manipulation-core-20260728/contact-sheet-agent.png)
- [`../reports/manipulation-core-20260728/safety-stop-diagnostic.png`](../reports/manipulation-core-20260728/safety-stop-diagnostic.png)

The suite sheets deliberately show `EVIDENCE-INVALID` and missing checkpoints.
The focused diagnostic sheet shows the last gate-executing safety attempt and
is not evidence for the final controller.

### Immediate next milestone

Implement and qualify the built-in free-space FJT gate before adding planning:

1. Execute a fixed outbound-and-return seven-joint trajectory only through the
   standard `FollowJointTrajectory` action.
2. Independently recompute final and RMS tracking error from recorded measured
   state, and reject missing/non-finite samples, collisions, safety stops, or
   gateway errors.
3. Inject a mid-trajectory safety stop and prove bounded velocity decay, frozen
   targets, unsuccessful action termination, and no automatic resume after
   clearing the stop.
4. Preserve the attempt-18 evidence, exact-drain, artifact-presence, and leak-free
   teardown requirements. Treat only core workflow, safety, evidence-integrity,
   and process/GPU leak findings as milestone blockers.

## Verification summary

| Test | Observed result | Artifact |
| --- | --- | --- |
| Generic Python suite | 194 passed, 3 skipped, 1 warning | Local `.venv` run |
| Sourced Humble manipulation partition | 54 passed | Sourced Humble test run |
| `uv lock --check` | 219 resolved packages | `module-integration-20260728.json` |
| Frozen offline bootstrap | Pass; 218 installed packages audited; no fallback | `deployment-20260728T072313Z.json` |
| ROS overlay | Two packages built; nine bridge executables | `module-integration-20260728.json` |
| CPU PhysX | 10,000 steps; `gpu_physics=false` | `headless-physx-latest.json` |
| RTX sensor smoke | Camera and LiDAR buffers valid | `rtx-sensors-latest.json` |
| Standard ROS control | Services/action discovered; live reset passed | `module-integration-20260728.json` |
| Native WebRTC soak | 70.003403 seconds decoded video; NVENC observed | `webrtc-native-client-soak-relocation-20260725T074310Z.json` |
| Navigation | Live development pass | `navigation-integration-20260725T085604Z.json` |
| Installed manipulation launch | `--show-args` passed | Installed launch smoke |
| Qualification scaffolding | Manifest-only exit `0`; not-configured exit `1` | Qualification runner smoke |
| Manipulation-core smoke | Clean headless no-ROS exit `0`; missing-scenario exit `1` | Runtime smoke |
| Live ROS manipulation startup | Controllers active; scenario reached `PHYSICS_READY`; measured object/evaluator output; contract `pass`; safety `false` | Two-process manipulation-core run |
| Live qualification scaffold | ROS domain `146`; readiness contract `pass`; safety `false`; `xarm7_traj_controller` active; object truth and `PHYSICS_READY`; sole effective safety/command publishers and raw-truth subscriber; every approved topic recorder subscription confirmed with explicit safety/contract QoS; post-gate Isaac/Humble/rosbag healthy; planned SIGINT exits all `0`; exactly `7149` raw/evaluator frames correlated and drained; no orphan cleanup or surviving GPU allocation; the finalized evidence set was complete and readable; `/bin/true` remained executed-unverified and overall unverified/nonzero | `/tmp/tinker-sim-live-qualification18/20260728T131149.899568Z-343348-023c44d930` |
| Physical manipulation task | Not a live retention, FJT, gripper, collision or pick-place pass; diagnostic gate performed no physical task | No release-qualification artifact |

## Historical independent manipulation audit — pre-implementation / superseded

A separate sub-agent conducted a read-only review specifically to prevent the
primary agent from grading its own conclusions. The audit captured the
pre-implementation state and is retained below for traceability. Its blocker
wording is superseded by the implementation follow-up above and must not be
read as the current runtime status. Its verdict was that the project could
not honestly pass an end-to-end manipulation test at that point; no live
qualification claim is inferred from that historical review.

### Confirmed blockers

1. **The supported command path currently rejects mixed base/arm commands.**
   The base façade continually publishes velocity-only wheel commands. The
   command mux combines these with arm position commands by inserting `NaN`
   positions for base joints, while `JointCommand.validate()` rejects every
   non-finite value. Bypassing the gateway and publishing directly to Isaac
   would invalidate the test.

2. **No manipulation runtime is wired into the Isaac launcher.**
   `run_sim.py` loads the Tinker robot only for `navigation-parity`, where
   contact sensors are explicitly disabled. `physics-only` and `sensor-rich`
   currently create a ground-plane world without the robot.

3. **Gripper effort is not applied.**
   The gripper façade accepts `max_effort`, but the Isaac backend consumes only
   commanded position and velocity.

4. **Safety stop is incomplete.**
   A controller can continue advancing the held target while stopped, and the
   active FJT action is not explicitly aborted.

5. **Contact coverage and live qualification are incomplete.**
   Finger/TCP contact sensor definitions exist but are disabled in the only
   robot runtime. There are no qualified arm collision sensors.

6. **Scenario loading is not part of whole-robot launch.**
   A standard `simulation_interfaces` scenario runner exists but is a separate
   executable and is not included by the whole-robot launch.

7. **No live object-truth or retention evaluator exists.**
   Object/contact/task truth messages are declarations. The live truth publisher
   contains robot and optional contact state, not dynamic object pose, twist,
   retained state or task arbitration. Existing evaluator tests use constructed
   dictionaries.

8. **MoveIt and cuMotion are not launched by the isolated project.**
   The current whole-robot launch stops at ros2_control and hardware façades.

What can be tested honestly now is graph discovery and controller/façade
startup. A complete physical pick/place cannot yet be claimed.

## Historical forward plan — pre-implementation / superseded

The following plan is retained as the audit-era qualification plan. It is
superseded as an implementation sequence by the follow-up and current
verification sections above. The live task requirements and independent review
requirements below remain unfulfilled unless explicitly stated otherwise.

All changes and generated evidence will remain inside the isolated project.
CPU PhysX is mandatory for behavior validation. GPU rendering or GPU planning
may be used, but must not alter the physics device.

### Gate 1 — repair and prove command/safety semantics

Before starting Isaac:

1. Repair heterogeneous position/velocity/effort command composition.
2. Unit-test simultaneous zero base velocity, seven-joint arm trajectory and
   gripper position/effort commands.
3. Require no `NaN`, missing fields, ownership conflicts or gateway rejection.
4. Freeze the position target when safety stop activates.
5. Reject subsequent target changes while stopped.
6. Abort the active FJT goal and prevent it from resuming after the stop clears.
7. Apply and bound gripper effort in the Isaac actuator.

Any failure is a stop condition for the live qualification.

### Gate 2 — add a real manipulation-core runtime

The runtime must:

- load the actual Tinker USD;
- assert `physics_device=cpu`, `/physics/useGpu=false` and
  `/physics/cudaDevice=-1`;
- enable validated finger, TCP and arm collision reporting;
- launch the external Humble controller manager and hardware façades;
- load `pick-deliver-place` through standard `simulation_interfaces`;
- spawn the existing dynamic cube without post-start teleportation;
- record cube pose, twist, contacts and gripper-relative transform directly
  from PhysX;
- prevent task/planning nodes from subscribing to evaluator truth.

### Gate 3 — free-space FJT and safety

Send a predetermined nontrivial outbound-and-return trajectory through:

```text
/xarm7_traj_controller/follow_joint_trajectory
```

Do not publish directly to `/isaac_joint_commands`.

The free-space trajectory passes only if:

- FJT is accepted and terminates successfully;
- all seven measured joints are continuously observed;
- final maximum joint error is at most 0.01 rad;
- RMS tracking error is at most 0.05 rad;
- there are no missing samples, non-finite values, collisions, gateway
  rejections or safety stops.

The mid-trajectory safety negative test passes only if:

- measured joint velocity falls below 0.02 rad/s within five physics frames;
- the target does not creep;
- the FJT does not report success;
- clearing the stop does not resume the interrupted goal.

### Gate 4 — gripper and contact physics

Run both:

1. A free-space open/close that produces real joint motion, bounded effort,
   `reached_goal=true` and `stalled=false`.
2. An obstructed close on the dynamic cube that produces bilateral
   finger-to-cube contacts, incomplete closure and `stalled=true`.

Action feedback alone is not contact evidence. PhysX records must identify both
colliding bodies.

### Gate 5 — collision negative control

Place a fixed obstacle in a predetermined arm trajectory. Require an identified
arm/obstacle contact, a non-successful trajectory, the configured safety
response, and no silent pass-through or continued target advancement.

### Gate 6 — physical object retention

Close on the cube, lift it at least 0.10 m, translate it at least 0.20 m and
hold it for at least one simulated second. Require bilateral contact and bound
object-to-gripper drift to 0.02 m and 5 degrees. A successful gripper action is
not proof of retention.

### Gate 7 — complete pick/place postcondition

Run MoveIt planning, FJT execution, physical grasp, transport, release and
settling. Pass only when hidden PhysX-derived state proves:

- the correct object is inside the 0.15 m target region;
- the object is released;
- object speed remains at or below 0.02 m/s for one continuous simulated second;
- no safety stop occurred.

A negative control will claim success without moving the object; the evaluator
must reject it. After the standard MoveIt path passes, cuMotion is tested as a
separate planner through the same execution and evaluation path, with no extra
truth access or relaxed thresholds.

## Qualification integrity rules

Before each live run, write an immutable manifest containing source, lock, USD
and scenario hashes; seed; commands; thresholds; ROS/RMW settings; CPU-physics
settings; and runtime versions.

Preserve every attempt, including failed startup, crash and timeout, with:

- raw ROS bag;
- Isaac Kit and ROS logs;
- process exit codes;
- ROS graph and publisher-ownership snapshots;
- controller commands, feedback and results;
- measured joint state and effort;
- body-identified contacts;
- raw object pose/twist and evaluator trace.

The following invalidate a qualification episode:

- publishing around the command gateway directly to `/isaac_joint_commands`;
- teleporting the task object after initial spawn;
- disabling gravity or collision;
- fabricating contact or object measurements;
- allowing task/planning nodes to consume `/sim/truth/*`;
- treating an action return code as proof of a world postcondition;
- changing thresholds after results are observed;
- deleting or silently replacing a failed attempt.

Video may support diagnosis but is not primary pass evidence.

### Independent result review

For the live qualification, a separate reviewer receives the completed artifact
directory read-only. The reviewer must independently recompute metrics from the
raw bag and truth trace, verify CPU physics and publisher ownership, check for
resets/teleports/timestamp discontinuities, account for every attempt, and
produce its own verdict. The runner's `pass` field is not authoritative. Any
runner/reviewer disagreement is a failed qualification pending investigation.

## Remaining release blockers

- NVIDIA driver 570.211.01 is below the supported 595.58.03 baseline.
- Host RAM meets the 32 GB hard minimum but is below the supported 64 GB
  whole-robot target.
- Synchronized Tinker 2 base, odometry, controller and sensor calibration is
  missing.
- Manipulation, Tinker vision, decision and VLA vertical slices lack live
  qualification.
- A fresh clean-server online bootstrap and a restored-bundle clean-server
  offline bootstrap remain required for release qualification.
- The host inotify instance limit was reported exhausted during Kit validation.

## Historical immediate execution order — pre-implementation / superseded

1. Fix and unit-test the command mux and safety stop.
2. Connect the manipulation-core runtime and scenario runner.
3. Add raw object/contact/evaluator instrumentation.
4. Run FJT and safety gates.
5. Run gripper, collision and retention gates.
6. Run MoveIt pick/place and its negative control.
7. Have the independent reviewer recompute and issue the final verdict.
8. Only then proceed to cuMotion, vision-assisted manipulation, decision and VLA
   vertical slices.
