# Tinker whole-robot simulation deployment

This is a standalone deployment project for x86_64 Tinker simulation servers.
It neither imports nor modifies the surrounding ROS workspace. For production,
copy this directory to a dedicated location such as
`/srv/tinker-sim/6.0.1`; do not overlay it on a sourced ROS workspace.

## Qualified baseline

| Component | Pinned value |
| --- | --- |
| Host | Ubuntu 22.04, x86_64, GLIBC 2.35 or newer |
| NVIDIA driver | 595.58.03 or newer |
| Python | CPython 3.12.13 |
| uv | 0.10.8 (0.10 lock format) |
| Isaac Sim | 6.0.1.0 |
| Isaac Lab | v3.0.0-beta2.patch1, commit `ffff603eafc6b74264a5261cc0183d6a65390d78` |
| Isaac Lab PhysX extension | 1.1.3 from the same pinned checkout |
| Isaac Lab Newton interface shim | 0.13.6 from the same pinned checkout |
| PyTorch | 2.11.0, CUDA 12.8 wheel |
| TorchVision / TorchAudio | 0.26.0 / 2.11.0, CUDA 12.8 wheels |
| Pillow | 12.2.0 |
| IsaacSim ROS workspaces | tag `IsaacSim-6.0.1`, commit `dd3eeede7912755996a18f4884285d9f50843f79` |
| ROS simulation interfaces | Humble 1.4.0, vendored Debian artifact |
| Topic-based ros2_control | Humble 0.2.0, vendored Debian artifact |

Provisioning requires at least 100 GB free disk, 32 GB RAM, and an RTX GPU
with 16 GB VRAM. Sixty-four GB RAM is the supported whole-robot target.
Drivers older than 595.58.03 produce an explicit experimental warning and are
not release-qualified.

`uv.lock` is authoritative for Python. The static `release-manifest.json` records the
lock, project, uv executable, managed Python, and Isaac Lab Git tree hashes.
`artifacts/provenance/ros-debs.json` is authoritative for the isolated Humble
runtime additions.
Each completed deployment also writes a machine-readable report under
`reports/`.

## Online bootstrap

Install uv 0.10.8 from its official release, start a fresh shell that has not
sourced `/opt/ros/humble`, then:

```bash
cd /srv/tinker-sim/6.0.1
cp deployment.env.example .deployment.env
# Read NVIDIA's Omniverse EULA, then set TINKER_ACCEPT_OMNIVERSE_EULA=Y.
set -a
source .deployment.env
set +a

./scripts/tinker-sim preflight
./scripts/bootstrap --mode online
```

EULA acceptance is never set by a script or repository default. Bootstrap
fails until the deployment configuration explicitly contains
`TINKER_ACCEPT_OMNIVERSE_EULA=Y`. After that explicit gate passes, bootstrap
sets Isaac Kit's required `OMNI_KIT_ACCEPT_EULA=YES` only for its child
processes.

Bootstrap is idempotent. It:

1. validates the OS, architecture, GLIBC, disk, RAM, GPU, VRAM, driver, and
   registered NVENC encoder;
2. installs CPython 3.12.13 into the project-local managed-Python directory;
3. fetches and verifies the exact Isaac Lab commit without patching it;
4. runs only `uv sync --frozen` and reconstructs a temporary environment with
   `uv sync --frozen --offline` to audit cache completeness;
5. runs Isaac's compatibility checker, 10,000 headless PhysX steps, requires
   non-empty RTX camera and LiDAR output, performs a real NVENC encode, and
   starts the headless WebRTC experience;
6. warms core, ROS, RTX-sensor, and URDF extension caches; and
7. vendors the exact ROS Debian artifacts and pins NVIDIA's IsaacSim ROS
   workspace without installing either into the host; and
8. writes a deployment report containing runtime hashes, package versions,
   ROS domain settings, and test results.

`--skip-preflight`, `--skip-validation`, and `--skip-prewarm` exist for tooling
tests only. A deployment made with any of them is not release-qualified.

No system CUDA toolkit is read or installed. CUDA runtime libraries come from
the Isaac Sim and PyTorch wheels.

## ROS 2 Humble boundary

There are two process environments:

- Isaac runs from this uv environment with Python 3.12 and Isaac's internal
  Humble libraries.
- Tinker gateways, Nav2, MoveIt/cuMotion, vision, decision, audio, VLA, FJT,
  and hardware-facade nodes run under system Humble with Python 3.10.

`scripts/launch-isaac` refuses to start if `PYTHONPATH`,
`AMENT_PREFIX_PATH`, `CMAKE_PREFIX_PATH`, `COLCON_PREFIX_PATH`,
`ROS_PACKAGE_PATH`, or `LD_LIBRARY_PATH` contains `/opt/ros/` or
`python3.10`. It then locates Isaac's internal
`isaacsim.ros2.* / humble / lib` directory and sets the shared ROS domain and
Fast DDS implementation. The default `--dds-profile local` leaves Fast DDS
shared memory enabled. `--dds-profile lan` selects the committed UDP-only
profile and must be used by every machine in that run.

Only the standard interfaces listed in `deployment.json` cross the Isaac DDS
boundary. NVIDIA's `isaacsim.ros2.sim_control` owns the standard
`simulation_interfaces` lifecycle API. The external overlay defines only
evaluator truth messages and hardware/task adapters; it must never be installed
in this uv environment.

Navigation is the first live module integration.  It reuses the existing
Tinker Humble Nav2/localization implementation without modifying the source
workspace, with a hardware-parity base facade, wheel-derived odometry, Livox
scan adapter, AMCL initialization, TF ownership guard, and public simulation
control gateway.  The exact build, two-terminal launch, interfaces, readiness
checks, and live results are documented in [`integration/NAVIGATION.md`](integration/NAVIGATION.md).

Example launch profiles:

```bash
./scripts/launch-isaac --sensor-profile physics-only --scenario empty
./scripts/launch-isaac --sensor-profile sensor-rich --profile parity \
  --scenario find-and-approach-person --seed 7
./scripts/launch-isaac --sensor-profile streaming --dds-profile lan
```

Streaming uses Isaac's headless WebRTC experience, explicitly disables GPU
physics, and uses a local process lock to
enforce one viewer per simulator instance. Restrict its ports to a trusted LAN
or VPN. Use SSH for installation, process management, logs, tests, and bags.

## Offline provisioning

Run a complete online bootstrap first. Add generated robot USDs and their
checksums to `artifacts/asset-manifest.json` using the tracked portable template
at `config/asset-manifest.example.json`; do not reference the surrounding ROS
workspace. Both generated-USD and warmed asset groups must be non-empty, present
on disk, and hash-correct. Whole-robot bundles also require a verified immutable
`artifacts/robot/tinker2/current.json` generation. The only intentional
artifact-free mode is the explicitly selected `bundle-restore --profile
physics_only` validation profile; whole-robot creation and restore fail closed
when the verified generation is absent.
Then:

```bash
./scripts/create-offline-bundle artifacts/tinker-sim-6.0.1.tar.gz
```

Bundle creation first creates a fresh temporary environment using
`uv sync --frozen --offline`. This proves that no package is missing from the
cache. It refuses to package an unverified Isaac Lab tree or an unwarmed Isaac
cache. The deterministic archive contains:

- the lock and environment definition;
- the exact uv executable and managed Python;
- the populated, versioned uv cache;
- the pinned Isaac Lab checkout;
- warmed Isaac caches and generated assets;
- this deployment tooling and ROS contract workspace; and
- a per-file SHA-256 manifest.

On an offline server:

```bash
mkdir -p /srv/tinker-sim/6.0.1
./scripts/restore-offline-bundle \
  /media/bundle/tinker-sim-6.0.1.tar.gz /srv/tinker-sim/6.0.1
cd /srv/tinker-sim/6.0.1
export PATH="$PWD/bin:$PATH"
export TINKER_ACCEPT_OMNIVERSE_EULA=Y
./scripts/bootstrap --mode offline
```

Restore requires an empty destination and verifies every file before use.
Offline bootstrap runs `uv sync --frozen --offline` and never falls back to
the network.

## Miniconda recovery only

Miniconda may create a Python 3.12.13 environment if uv's managed Python cannot
be bootstrapped. It is not a second dependency definition:

```bash
./scripts/tinker-sim conda-export artifacts/requirements-from-uv-lock.txt
conda create -n tinker-isaac-recovery python=3.12.13
conda activate tinker-isaac-recovery
python -m pip install -r artifacts/requirements-from-uv-lock.txt
```

Always regenerate the requirements file from the committed lock. Never edit
or maintain it independently.

## Simulation contract and rollout

`contracts/simulation.yaml` defines the standard `simulation_interfaces`
surface and evaluator-only `/sim/truth/*` topics. There are deliberately no
custom `/sim/control/*` or `/sim/scenario/*` aliases.
`simulation/tinker_sim_core` provides:

- a Gazebo/Isaac backend protocol;
- deterministic single-owner command arbitration;
- hardware-parity versus hidden-truth separation; and
- postcondition scoring for person approach, pick/deliver/place, and reception
  seat assignment.

The parity profile is the only input to navigation, perception,
manipulation, decision, and VLA software. Only the evaluator may receive
truth. The scenario evaluators reject a task server's claimed success when the
world postcondition is false.

The Tinker 2 USD/URDF/map export, navigation gateway, external FJT controller,
xArm/gripper/pan-tilt facades, deterministic scenario orchestration, and audio
fixtures are implemented as content-addressed artifacts under `artifacts/`.
The artifact exporter derives only the canonical URDF metadata needed by the
strict planning contract: it adds the zero `world -> base_link` mount, names
the existing `base_link -> link_base` mount, and records a state-only
`drive_joint` entry. The USD is copied byte-for-byte and remains the simulator
physics artifact. The canonical URDF bytes and canonicalizer algorithm version
are part of the artifact identity; the manifest, immutable per-artifact
source lock, and `current.json` record the complete generation and all payload
hashes. Publication fsyncs files/directories, atomically claims the immutable
artifact directory, and replaces `current.json` as the sole commit point; a
crash before that replacement can leave only an ignored orphan generation and
cannot mix a pointer with a mutable root lock. Export publication is serialized
by a nonblocking inter-process lock; a normal concurrent exporter fails without
recovering the active exporter's staging directory. The exporter reads the
external workspace once, derives the immutable lock in memory, and only accepts
an already captured shared lock when it exactly matches; a failed export never
creates or rewrites that shared lock. `workspace-lock` is the separate explicit
atomic capture operation.

### Portable deployment inputs

The simulator checkout is relocatable. Set `TINKER_SIM_ROOT` when invoking a
launch wrapper from an installed/copy deployment; launch files otherwise derive
the project root from their installed/source package location. Set `TINKER_WS`
to the external Humble workspace, or pass `tinker_workspace:=...` explicitly.
There is no host-specific workspace default. `scripts/launch-humble` fails
clearly when `TINKER_WS` is absent. The external workspace is read-only runtime
input and is never bundled as an absolute provenance path.

The source lock schema is portable and exact: it contains robot `tinker2`, a
SHA-256 source identity derived from the fixed source contract plus the ordered
relative path/size/SHA-256 records, and no fake repository revision. The
immutable per-artifact lock is authoritative for that generation; manifest
canonicalization, USD provenance, source records, payload hashes, and the
content-addressed identity are cross-checked against it by every consumer.
Use the project-managed interpreter, for example
`TINKER_WS=/path/to/tk25_ws ./.venv/bin/python tools/deploy.py artifact-export`,
rather than editing an artifact directory in place.

Vision, manipulation, decision, and VLA vertical slices still require live
qualification and must not be represented as release-qualified. Controller
gains and sensor/base noise still require synchronized robot calibration;
qualification fails explicitly while
`simulation/calibration/tinker2-missing.json` remains uncalibrated.

### OMPL overlay acceptance contract

`integration/ompl-overlay-contract.json` is the deterministic acceptance
contract for the reviewed OMPL overlay (Tasks 3-7): canonical model-bundle
producer/preflight, integrated eight-joint state contract, atomic fixture
PlanningScene adapter, staged integrated OMPL overlay with typed integrated
readiness, and the deterministic OMPL plan-only smoke.  The contract is
canonical JSON (schema version 1, sorted keys, minimal separators) with exact
repository/commit identities, the production overlay
(`mobile_bringup` / `manipulation_planning_task_only.launch.py`, 18-argument
contract, literal-false `use_cumotion_*` compatibility values), the provider
manifest (raw + canonical identities, distinct persistent/one-shot/controller/
publisher classes), the model-bundle schema/artifact hashes and full eight-link
touch set, the typed action/service/topic contract (including
`/isaac_joint_commands` depth 50 and the external future
`/tinker_integrated_gate_executor` publisher ownership), fixture/scenario
identities, Task 6/7 evidence, and build commands (`tkbuild` /
`scripts/build-humble-overlay`, never raw colcon).  It does not itself prove
live OMPL or authorize cuMotion.  `tests/test_provenance.py` recomputes every
derived hash/contract from the real source and fails on mutations.  The two
repository-local source-lock files are Task 9 only.  See
`integration/MANIPULATION.md` for the operator workflow.

## Developer verification

The non-GPU tests run with the Ubuntu system Python:

```bash
python3 -m unittest discover -s tests -v
uv lock --check
./scripts/build-humble-overlay
```

The focused OMPL-overlay provenance suite runs under the simulator venv.  ROS
plugin discovery may auto-load the Humble `launch_pytest` plugin, which can fail
collection with `ModuleNotFoundError: No module named 'lark'` on hosts that do
not provide that dependency; disabling plugin autoload is the defensive
reproducible invocation (it does not claim collection necessarily fails on any
specific host):

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ./.venv/bin/python -m pytest -q tests/test_provenance.py
```

This does not hide the pre-existing pinned-uv environment failure
(installed `uv 0.12.0` vs pinned `uv 0.10.8`), which is reported visibly.

Release qualification additionally requires a clean server, the full online
and offline bootstrap paths, ROS cross-process contract tests, RTX camera and
LiDAR initialization, and a 60-second continuous WebRTC connection using
NVIDIA's native Ubuntu Omniverse Streaming Client (never a downloaded browser).

References:

- [Isaac Sim Python installation](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_python.html)
- [Isaac Sim requirements](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html)
- [Isaac Sim ROS integration](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_ros.html)
- [Isaac Lab installation](https://isaac-sim.github.io/IsaacLab/develop/source/setup/installation/index.html)

## Changelog

- 2026-08-04 (integrated qualification Task 6, formal-review fix round 3 —
  "make Gate E temporal evidence deterministic"): Resolved the F3.1-F3.5
  consolidated findings in the Stage-E path, preserving every review-clean
  F1/F2 behavior.  F3.1 — every flagship E ordering/race/negative proof is now
  event-driven: `threading.Event` barriers gate the approach/transport/Place
  evidence injections strictly after the executor's corresponding observable
  state (goal acceptance + baseline capture, lift latch, Place baseline, exact
  Place cancel terminal), and the delayed Pick-result future is Event-released
  only after the transport latch.  Fixed `threading.Timer` offsets and
  `time.sleep` event-order margins are gone; the runner-level receipt-window
  "late" negatives pin the FJT `received_mono` strictly beyond the 2.0 s
  `E_FJT_CORRELATION_TIMEOUT_S` boundary and the settled-post-result negative
  seeds the transport FJT only after the bounded wait returns.  F3.2 —
  occupied-place now requires a STRICTLY fresh post-cancel PlanningScene
  observation: the runner records the pre-cancel `scene_sequence` + receipt,
  boundedly waits for a valid scene with `scene_sequence` strictly greater than
  the baseline AND a receipt time after the exact cancel terminal, and only that
  fresh scene may establish `post_cancel_target_attached=true`.  Timeout,
  unchanged sequence, malformed/provider-error newer scene, or detached target
  is `evidence-invalid`; baseline/post-cancel sequence, receipt delta,
  attachment state, and reason are recorded in the trigger and durable
  artifacts.  F3.3 — controller traffic is derived ONLY from observed FJT
  evidence: `controller_goal_sent` is true only when an actual FJT
  transaction/status/UUID was observed, `controller_endpoint` is None when no
  FJT was observed, no-goal cleanup is `None` (never `{}`), and accepting/
  canceling a task goal or attempting task-goal cleanup never implies a
  controller goal.  The truth is preserved consistently in returned records,
  `integrated-execution.jsonl/.json`, `controller-results.jsonl`,
  `moveit-plans.jsonl`, and goal artifacts.  F3.4 — the dead reason-collapsing
  `_e_post_grasp_lift_m()` helper is deleted (the detailed provider reason map
  stays in `_e_prepare`), and `integration/MANIPULATION.md` now carries the
  live-orchestrator latency obligation for `E_FJT_CORRELATION_TIMEOUT_S=2.0`,
  the safety-stop observation window, and fresh post-cancel PlanningScene
  publication/service latency.  Humble suite grows to 160 tests (pure suite
  stays 126); every ordering/race negative is barrier-bounded, and the full
  sourced-Humble suite passes 10 consecutive fresh clean processes with the
  flagship temporal subset passing 20 consecutive clean iterations.  No build
  or live run is required.

- 2026-08-04 (integrated qualification Task 6, formal-review fix round 2 —
  "seal Gate E runtime contracts"): Resolved the full F2.1-F2.8 consolidated
  SPEC/QUALITY findings in the Stage-E path.  F2.1 — the 10 cm physical
  threshold is preserved (Gate E never weakens `object_lift_m` to the 0.08 m
  production default); every E transport scenario (positive, occupied-place,
  cancel-transport, safety-transport) now requires an injected, fresh
  `post_grasp_lift_m_provider` runtime-parameter observation BEFORE any Pick
  traffic.  The observed production `pick_and_place` parameter must be finite
  and `>= object_lift_m` (0.10 m), with fresh identity/receipt metadata;
  missing/stale/provider-error/0.08 evidence fails immediately with a stable
  readiness reason (`no-post-grasp-lift-m-provider`,
  `post-grasp-lift-m-provider-{missing,stale,non-finite,unavailable}`,
  `post-grasp-lift-m-below-object-lift`) and zero action traffic — never a
  15 s transport timeout.  Accepted 0.10 keeps the lift latch
  `grasp_z + object_lift_m - tolerance` (0.81 m), physically reachable at the
  production TCP peak 0.82 m.  F2.2 — the positive/occupied-place/
  cancel-transport/safety-transport ordering tests now use controllably
  delayed Pick result futures (`result_ready_at=1.5`) and assert
  `transport_latched < pick_result_ready`; a runner-level negative proves that
  only settled post-result provider/history evidence fails closed
  (`evidence-invalid`, no Place, no release).  F2.3 — native-gripper
  rejection coverage is completed: a fresh nonzero baseline that stays
  unchanged passes, an increment-after-acceptance rejects the approach trigger
  (exact Pick cleaned up, no attachment/later goal, artifacts
  `evidence-invalid`), and missing/stale/provider-exception evidence fails
  closed.  F2.4 — runner-level receipt-window negatives prove the approach
  FJT before the acceptance baseline, the approach FJT later than 2.0 s, the
  transport FJT before the lift latch, the transport FJT later than 2.0 s
  after the lift latch, and a Place target-motion FJT outside its window are
  each bounded and `evidence-invalid` with no forbidden later goal/release.
  F2.5 — occupied-place re-observes a fresh PlanningScene after the exact
  Place cancel terminal and quiescence, proving `pick_and_place/object_mesh`
  remains attached; if open/detach wins the race Gate E fails
  `evidence-invalid`, and the post-cancel scene sequence/attachment state is
  recorded in the trigger artifact.  F2.6 — `_e_unexpected_exception` derives
  cleanup/goal identity/sent flags BEFORE durable writes; every durable
  artifact (`integrated-execution.jsonl/.json`, `moveit-plans.jsonl`,
  `controller-results.jsonl`, `goals/<id>.json`) truthfully preserves the
  accepted Pick/Place handle, goal IDs, goals sent, cleanup outcome, trigger,
  and `status=evidence-invalid` (no row claims no goal was sent when one was
  accepted).  F2.7 — blocked-approach/unreachable-grasp require production-real
  terminal consistency: action-client GoalStatus ABORTED (6) together with a
  non-success/non-canceled task result such as `planning_failed` (2); a
  contradictory SUCCEEDED-terminal/failure-Result (or canceled/safety Result)
  pair is rejected.  Humble suite grows to 155 tests and the pure suite to 126
  (F2.1 `_post_grasp_lift_m_observation` threshold + transport-kind set tests).
  No build or live run is required.

- 2026-08-03 (integrated qualification Task 6, formal-review fix round 1 —
  "make Gate E live-observable"): Resolved the full F1.1-F1.15 consolidated
  SPEC/QUALITY findings in the Stage-E path.  F1.1/F1.2 — the positive and
  occupied-place sequences now observe and latch lift-complete and transport
  **while the Pick goal remains executing** (production `stay=false` returns to
  `Q_OUTBOUND` before publishing the result); the transport trigger is a
  two-phase temporal predicate: `lift_complete` latches only after observed
  attachment + TCP z above the configured lift threshold + settled arm velocity
  (<= `settled_speed_m_s`) + two consecutive fresh normal-state samples, and
  `transport_started` then requires a **later** fresh FJT EXECUTING entry, target
  still attached, and fresh TCP speed >= the trigger limit, without re-requiring
  the settled condition while moving; receipt sequences/timestamps prove the
  transport evidence is later than the lift latch.  F1.3 — safety-transport uses
  the correct operator polarity (assert with `publish_operator(True)`, clear with
  `False` only after the effective stop) and the Humble test drives a causal fake
  publisher asserting the exact publication order `[True, False]`.  F1.4 — every
  accepted cancel/safety interruption boundedly awaits the exact goal result and
  records both status domains from the actual handle/result (canceled Pick/Place:
  action-client GoalStatus CANCELED=5 + `Result.status=4`; safety-transport:
  `Result.status=5` safety_stop + action-client ABORTED terminal per production
  `complete_pick` abort semantics).  F1.5 — the complete trigger object (join
  key, task/FJT UUIDs, receipt seq/time, TCP z/speed, arm velocity/normal
  samples, attachment, trigger kind, safety/cancel evidence) is persisted into
  `integrated-execution.jsonl/.json` and the per-scenario `goals/<id>.json`
  `geometry` is written from the validated dispatch spec (asserted exact fixed
  geometry), with trigger fields preserved in final downgrade rows.  F1.6 —
  `E_FJT_CORRELATION_TIMEOUT_S == 2.0` now bounds every FJT receipt window
  (approach vs goal-acceptance baseline, transport vs lift latch, Place
  target-motion vs Place baseline) and the actual receipt delta is recorded;
  stale/received-before-boundary entries are `None` and never satisfy a trigger
  (boundary tests just inside/outside the window).  F1.7 — every public E entry
  resets all per-attempt state (`_tcp_pose_samples`, native gripper seam,
  active-goal handle/state) before any provider sample; a reused-executor test
  proves no previous sample can satisfy the next attempt.  F1.8 — the shared
  `_fixture_scene_error` keeps Gate C/D exact fixture-only validation strict: a
  stray `pick_and_place/*` world object fails fixture readiness with the exact
  ordered "must equal" message, and only the exact task target is permitted via
  the explicit Stage-E `allow_e_target` argument.  F1.9 — every E public dispatch
  fails closed on any escaping exception (`reason_code="unexpected-exception"`)
  with bounded cleanup of the accepted goal and complete Gate-E artifacts, and an
  injected snapshot failure after Pick acceptance is verified end-to-end.  F1.10 —
  cancel-approach uses the injected receipt-sequenced
  `native_gripper_goal_count_provider` seam (not the fake-only `sent_goals`
  attribute); the fresh count must not increase and the target must stay
  unattached, with missing/stale/provider-error evidence failing closed.  F1.11 —
  sourced-Humble integration tests cover both `_run_e_blocked_or_unreachable`
  scenarios (real accepted Pick goal, terminal non-success in both status
  domains, exact short journal, no Place/later goal, complete fail-dominant
  artifacts, and success-status rejection).  F1.12 — `__all__` no longer lists
  the removed module-level `run_pick_place_*` stubs (`import *` succeeds) and
  malformed-back requires no TCP provider because it rejects before any motion.
  F1.13 — pre-goal evidence-invalid paths write canonical `planning-scene.json`
  even when the journal has no prior records.  Humble suite grows to 141 tests
  and the pure suite to 124 (import-star, receipt-window boundaries, C/D fixture
  strictness regression, blocked/unreachable, per-attempt reset, pre-goal
  artifacts, unexpected-exception cleanup).  No build or live run is required.

- 2026-08-03 (integrated qualification Task 6 — "add fixed-target Pick and Place
  controls"): Added the Stage-E fixed-target Pick/Place diagnostics executor to
  the same ROS-lazy `validation/integrated_gate_executor.py`.  A closed ROS-free
  `stage_e_dispatch` validates exactly the eight committed Stage-E scenarios
  (`qualification-pick-place-{positive,blocked-approach,unreachable-grasp,
  malformed-back,cancel-approach,cancel-transport,safety-transport,
  occupied-place}`) for exact id, `integrated.stage == "E"`,
  `integrated.execution_profile == "sim_ompl"`, exact declared polarity
  (positive/negative), the exact configured `expected_physical` list, the exact
  `expected_negative` required/forbidden contract, exact
  `forbidden_endpoints == ["/isaac_joint_commands"]`, exact declared
  `trigger_timeout_s`, and the pinned fixed-target geometry
  (`grasp_tcp_xyz [0.65, 0, 0.72]`, `object_root_xyz [0.65, 0, 0.60]`,
  `place_target_point` base_link `[0.85, 0, 0.72]`, identity orientation) plus
  the declared six-value malformed-back vector; unknown/C/D-stage, malformed, or
  mutated scenarios fail closed before any goal is created or sent.  The positive
  sequence uses only production `/pickup_action` then `/place_action` with the
  deterministic cube cloud and exact seven-joint `Q_OUTBOUND`, `use_mesh=True`,
  `stay=False`, and requires observed PlanningScene attachment before
  `scene-attach` and observed detach before `scene-detach` (never action-result
  inference).  The E journal branches per scenario: the positive order equals
  `POSITIVE_ORDER` exactly and each negative uses its own exact diagnostic event
  order with scenario-specific forbidden events, so Gate-C/D journal bytes stay
  unchanged.  Live TCP evidence reuses the existing injected
  `current_tcp_pose_provider` seam (never an embedded TF listener): a bounded
  per-attempt sample deque derives `tcp_z_m` and `tcp_speed_m_s = |Δxyz|/Δt`
  from the two newest fresh receipt-sequenced samples; fewer than two fresh
  samples means the trigger cannot fire and a timeout is evidence-invalid, never
  a pass.  FJT correlation is receipt-window only (first fresh FJT EXECUTING
  entry after the goal-acceptance baseline for approach, first later one for
  transport) with observed UUIDs recorded as evidence — the internal Pick/Place
  ExecuteTrajectory UUID is never claimed observable.
  cancel-approach/cancel-transport cancel the exact Pick handle at the trigger;
  safety-transport asserts operator safety, requires effective-stop evidence,
  clears only after the stop, and requires quiescence with no auto-resume;
  occupied-place sends Place and cancels at the first fresh Place target-motion
  trigger while the target remains attached (never waiting for the natural
  failure path that may open/detach).  malformed-back calls the real builder with
  the committed six-value back vector and requires a pre-send `ValueError` with
  zero action traffic and no PlanningScene mutation.  The fail-dominant artifact
  transaction mirrors Gate D with `event="gate-e"`, `stage="E"`, handler/polarity,
  pick/place goal-sent flags, both the action-client `GoalStatus` and the
  Pick/Place `Result.status` domains, task/FJT UUIDs, trigger record, and
  `isaac_joint_commands_published=false` across `integrated-execution.jsonl/.json`,
  `moveit-plans.jsonl`, `controller-results.jsonl`, visual-capture requests, and
  per-scenario goal artifacts; any journal/graph/artifact failure downgrades every
  status-bearing stream to `evidence-invalid`.  Task 6 records diagnostics only —
  raw contact/lift/release/collision verdicts remain Task 7 verifier work.
  Humble suite grows to 136 tests (fixed Pick/Place geometry, malformed-back
  pre-send rejection + zero traffic, positive full sequence + observed-attach
  guard, cancel-approach/cancel-transport/safety-transport/occupied-place,
  status-domain separation, receipt-window FJT selection, artifact
  fail-dominance); the pure suite grows to 121 tests (all eight dispatches +
  mutation rejection, per-kind journal orders/forbidden events, occupied-place
  fixture ownership, TCP speed/z derivation and undersampling).  No build or live
  run is required.

- 2026-08-03 (integrated qualification Task 5, formal-review fix round 2 —
  "seal Gate D evidence streams"): Resolved the formal SPEC/QUALITY residual
  findings F2.1-F2.7 in the Stage-D evidence path.  F2.1 — a D artifact-write
  downgrade now appends a final corrective row (`row_kind="final"`,
  `status="evidence-invalid"`) to ALL THREE status streams
  (`integrated-execution.jsonl`, `moveit-plans.jsonl`, `controller-results.jsonl`),
  preserving `planner_status`, `plan_applicable`, handler/action endpoint,
  UUID/digest/controller fields, `downgraded_from`, and the stable artifact
  error; no corrective row ever claims pass, and a failed corrective append is
  contained so the authoritative atomic summary stays fail-dominant.  F2.2 —
  `run_gripper_sequence(open_first=False)` (close→open) selects the close-first
  journal order `fixture-ready → gripper-close-terminal → gripper-open-terminal
  → teardown` via a fresh journal rebuild before its first record; an
  in-progress/nonempty journal is never mutated and the attempt fails closed
  with `journal-order-rebuild-refused`.  F2.3 — D scene-acquisition failures
  (`no-planning-scene`, `planning-scene-invalid`) route through the D evidence
  helper with `stage=D`, `event=gate-d`, the D handler label, and the D
  controller/artifact schema (never a Gate-C `gate-c-plan-only` record); Gate-C
  callers and bytes are unchanged.  F2.4 — a split-path execute whose accepted
  ExecuteTrajectory handle has an invalid/identical/non-normalizable UUID is
  cleaned up with exactly one bounded cancellation attempt before the evidence
  is finalized invalid, and the cleanup outcome is recorded without claiming a
  successful cancel unless the exact CancelGoal contract is met.  F2.5 —
  `_env_cloud_evidence` now requires structural PointCloud2 self-consistency:
  `row_step >= width*point_step` (valid row padding allowed) and
  `len(data) == row_step*height` (truncated and oversized buffers both rejected),
  plus a usable x/y/z FLOAT32 field layout when fields are advertised
  (unadvertised bytes are consumed as opaque payload and documented); invalid
  evidence fails closed before any action goal.  F2.6 — both MoveGroup builders
  explicitly pin `goal.planning_options.look_around = False`, the dead
  fail-open helpers `_safety_terminal_status` / `_safety_velocity_frames` and
  the unused `_d_journal_pass` are deleted, and `_wait_for_fjt_status` captures
  its matched entry in a single predicate result (no second lookup).  F2.7 —
  `controller_goal_sent` is pinned to the exact FJT semantic (a
  `follow_joint_trajectory` controller goal); retreat/gripper traffic is
  surfaced through explicit `action_goal_sent`/`action_endpoint` plus
  `cartesian_goal_sent`/`gripper_goal_sent` fields in `controller-results.jsonl`
  without pretending it was an FJT goal, and D visual-capture requests use
  `capture.kind="gate-d-diagnostic"` while Gate-C bytes keep `plan-only`.
  Humble suite expanded to 125 tests (all-stream corrective rows for
  execute/retreat/gripper, close-first pass and order-replacement guard,
  D-labeled scene-acquisition failures, accepted-UUID cleanup, truncated/
  oversized/undersized-row-step/valid-padded-row cloud structural probes,
  look_around pin, dead-helper removal, and execute/retreat/gripper action
  semantics artifacts); the pure suite stays 97 tests.  Red/green: 18 tests
  fail against the pre-fix base and pass at the fix.  No build is required.

- 2026-08-02 (integrated qualification Task 5, pre-review fix round 1 — "make
  Gate D runtime truthful"): Hardened the Stage-D lifecycle and evidence path
  against the real Humble rclpy action API.  F1.1 — the executor keeps a private
  `_owned_action_clients` collection of every real rclpy `ActionClient`, separate
  from the mutable public client map that tests may replace with fakes, and
  `shutdown()` now destroys every owned ActionClient exactly once (removing its
  waitable and C handle) before the node and private-context teardown, clearing
  lifecycle members so GC cannot double-finalize; a partial-constructor failure
  destroys already-created clients before node/context cleanup, and repeated
  construct/shutdown stays supported (this removes the leading in-process suspect
  for the coordinator's order-dependent full-suite SIGSEGV).  F1.2 — a cancel
  pass is impossible without the exact live ExecuteTrajectory `ClientGoalHandle`;
  raw UUID kwargs never substitute; `cancel_goal_async()` is called exactly once
  on that handle and requires the real-shape `CancelGoal.Response` contract
  (`return_code == ERROR_NONE` and `goals_canceling == [execute_goal_id]`), the
  ExecuteTrajectory result must be terminal CANCELED (5), and the joined FJT
  controller goal must reach CANCELED (5) with bounded quiescence — SUCCEEDED/
  ABORTED/unknown/empty/extra/malformed/exceptional/timed-out responses fail
  closed.  F1.3 — safety requires the `fjt_transaction_provider` evidence to
  validate (endpoint/UUID/digest/source/sequence/timestamp/status) and join to a
  fresh status-topic entry inside the current attempt window; provider exception,
  no-provider, stale, wrong-UUID/digest/source/endpoint/status-cache, and
  prior-ABORTED-cache entries fail closed (never swallowed, never an unrelated
  cached ABORTED goal).  F1.4 — FJT/status and joint-state evidence is windowed
  to the current attempt with internal receipt baselines; bounded spin/wait
  helpers cover the joined EXECUTING motion trigger, joined terminal status,
  no-active-goal quiescence, `safety_stop_frames` consecutive fresh bounded joint
  frames, and bounded post-clear stability using `safety_position_creep_rad`.
  F1.5 — an accepted ExecuteTrajectory goal that times out or fails early is
  cleaned up via bounded cancellation on the exact handle, recording the cleanup
  outcome without ever claiming cancel success unless the F1.2 contract was met.
  F1.6 — all six D handlers (execute joint/pose, cancel, safety, retreat,
  gripper) route success and failure through one fail-dominant
  `_finalize_d_attempt`/`_write_d_artifacts` path writing the complete
  authoritative set (`integrated-execution.jsonl/.json`,
  `moveit-plans.jsonl` with an explicit `plan_applicable=false`/
  `planner_status=null` for the non-MoveIt retreat/gripper handlers,
  `controller-results.jsonl`, truthful visual-capture-request rows, and
  scenario-specific `goals/<scenario_id>.json`); required goal/JSONL write
  failures propagate into the Task-4 transactional downgrade, and every D journal
  snapshot return is checked at its event boundary.  F1.7 — `run_cartesian_retreat`
  requires an explicit fresh `environment_cloud_provider` returning a real
  non-empty finite `base_link` `sensor_msgs/msg/PointCloud2`; that exact cloud is
  passed into `CartesianMove.Goal.env_points` and only then is
  `collision_checking` recorded true; missing/empty/malformed/stale/wrong-frame/
  provider-exception/serialization-failure fails closed before goal send.  F1.8 —
  Gate-C journal bytes and public policy are unchanged; D visual captures use
  real before/after chronology; real-shape `GetResultService.Response`
  (`status`/`result`) and `CancelGoal.Response` (`return_code`/`goals_canceling`)
  test doubles replace the legacy handle-conflation model; `execute_timeout_s`
  is in the test config.  Humble suite expanded to 109 tests (lifecycle destroy
  spies + repeated real-context stress, cancel/safety fail-closed matrices,
  windowed-evidence helper tests, timeout cleanup, goal-artifact/journal
  fail-dominant injection, retreat env-cloud validation, execute-pose coverage,
  visual chronology); the pure suite stays 97 tests.  No build is required.

- 2026-08-02 (integrated qualification Task 5): Added the Stage-D execution
  interruption gates to the same ROS-lazy executor.  A closed ROS-free
  `stage_d_dispatch` validates exactly the six Stage-D scenarios
  (`qualification-moveit-execute-joint`, `-execute-pose`, `-cartesian-retreat`,
  `-gripper`, `-cancel`, `-safety`) for exact id, `integrated.stage == "D"`,
  `execution_profile == "sim_ompl"`, exact declared polarity (`positive` /
  `cancel` / `safety`), the exact configured `expected_physical` list, and
  `forbidden_endpoints == ["/isaac_joint_commands"]`, failing closed before any
  goal on unknown/C/E-stage/malformed/mutated scenarios.  The mandatory split
  path (`run_execute_sequence`) sends exactly one OMPL plan-only `/move_action`
  goal (`xarm7`, `ompl`, attempts 3, allowed time 3.0, `plan_only=True`,
  `look_around=False`, `replan=False`), requires a non-empty
  `planned_trajectory`, assigns it unchanged to exactly one
  `moveit_msgs/action/ExecuteTrajectory.Goal` (`build_execute_trajectory_goal`;
  canonical ROS-serialized digest unchanged before/after), sends it to
  `/execute_trajectory`, and records distinct valid 16-byte plan/execute UUIDs.
  Terminal action statuses use action_msgs constants (SUCCEEDED=4, CANCELED=5,
  ABORTED=6; unknown/malformed never pass) via `_execute_status_name`.  FJT
  observation is truthful: only the real
  `/xarm7_traj_controller/follow_joint_trajectory/_action/status` subscription
  (`action_msgs/msg/GoalStatusArray`, RELIABLE/TRANSIENT_LOCAL/depth 1) exists —
  no `_action/goal` topic (the goal travels over the `send_goal` service) — with
  a bounded cache, and the executor requires an injected
  `fjt_transaction_provider` returning real controller-transaction evidence
  (endpoint exactly the FJT endpoint, normalized FJT goal UUID, canonical
  trajectory digest equal to the unchanged ExecuteTrajectory digest, finite
  timestamp/sequence, real source) that joins to the newest status entry;
  missing/stale/mismatched/extra/malformed/provider-exception evidence makes the
  D attempt `evidence-invalid`.  `run_cancel_sequence` calls
  `cancel_goal_async()` only on the ExecuteTrajectory `ClientGoalHandle` (never
  the completed MoveGroup planning handle), records exactly the execution UUID in
  `goals_canceling`, requires terminal CANCELED (5), requires no active joined
  FJT status before `quiescent=true`, and never sends a later stage.
  `run_safety_sequence` publishes operator True via the real
  `/sim/safety/operator` publisher, waits bounded for safety-stop True, requires
  the old ExecuteTrajectory terminal status ABORTED (6), requires
  `safety_stop_frames` consecutive fresh joint-state velocity frames bounded by
  `safety_stop_velocity_rad_s` (0.02), publishes operator False only after the
  effective-stop predicate, requires bounded post-clear stability, and sends no
  replacement/resume goal.  `run_cartesian_retreat` uses an injected
  `current_tcp_pose_provider` (no embedded TF listener), derives exactly
  `RETREAT_DISTANCE_M` (0.10 m) along `+Z` in `base_link` preserving orientation
  (`derive_retreat_target_pose`), and sends one collision-aware
  `/cartesian_move_action` goal with `command_gateway_bypassed=false` as a
  routing assertion.  `run_gripper_sequence` sends open (0.0) then close (0.85)
  `control_msgs/action/GripperCommand` goals to `/xarm_gripper/gripper_action`
  with max effort 10.0 (`build_gripper_goal`) and `native_action=true`.  A
  fail-closed `run_pick_place_negative` stub covers the two Gate-E cancellation
  identities (`cancel-approach`/`cancel-transport`) with `release_stage_started=false`,
  `released=false`, and zero goals, rejecting all others.  D-stage artifacts use
  Task 4's transactional/fail-dominant mechanics with a separate D record shape:
  `diagnostic_only=true`, `physical_verdict=None`, fail-dominant
  `status`/`planner_status` split, truthful `execute_trajectory_goal_sent` /
  `controller_goal_sent`, plan/execute/goals-canceling UUIDs, planned/executed/
  FJT trajectory digests, FJT goal UUID/status, execute result status/string,
  event log, elapsed, and `isaac_joint_commands_published=false`; required
  write/finalization failures downgrade every authoritative D artifact.  The
  journal uses scenario-specific D diagnostic event orders (execute/pose:
  `fixture-ready → execution-start → execution-terminal → teardown`; retreat:
  `fixture-ready → retreat-start → retreat-terminal → teardown`; gripper:
  `fixture-ready → gripper-open-terminal → gripper-close-terminal → teardown`;
  cancel: `fixture-ready → execution-start → cancel-requested → quiescent →
  teardown`; safety: `fixture-ready → execution-start → effective-stop →
  operator-clear → quiescent → teardown`) with the eight Task-4 forbidden
  manipulation events unchanged, the Task-3 graph projection byte-identical, and
  no attach/detach/release event in Gate D.  Humble suite expanded to 74 tests
  (split-path unchanged digest + distinct UUIDs, real FJT status QoS/type, no
  `_action/goal` subscription, gripper/cartesian goal construction, stubbed
  execute/cancel/safety/retreat/gripper flows, Gate-E stub zero traffic); the
  pure suite expanded to 97 tests (Stage-D dispatch mutations, status mapping,
  UUID normalization, retreat derivation, Gate-E stub, D-vs-C journal order,
  unchanged graph projection).  No build is required because no
  package-installed path changed.

- 2026-08-02 (integrated qualification Task 4): Added the ROS-lazy Gate-C OMPL
  plan-only executor.  `validation/integrated_gate_executor.py` is importable
  under the simulator CPython 3.12 venv without `rclpy` or any generated ROS
  message; it exposes the exact graph/endpoint/QoS contract, the real
  multi-operation canonical public report validation
  (`expected_physics_ready_report` / `validate_physics_ready_snapshot` built
  through `tinker_sim_bridge.integrated_readiness.build_canonical_report` with
  the one-key public `{"execution_profile": "sim_ompl"}` mapping and
  scenario-declaration-bound fixture descriptor digest), the readiness
  evaluator (`evaluate_executor_readiness`) over the genuine positive-ready
  baseline, the Task 3 journal graph projection
  (`build_journal_graph_projection` matching `planning_scene_journal.validate_graph_evidence`),
  ROS-lazy MoveGroup/Pick/Place/PointCloud2 builders, and the live
  `IntegratedGateExecutor` node `/tinker_integrated_gate_executor` that sends
  only plan-only `/move_action` goals (`group_name="xarm7"`, `pipeline_id="ompl"`,
  `num_planning_attempts=3`, `allowed_planning_time=3.0`,
  `planning_options.plan_only=True`, `replan=False`), never calls
  `/execute_trajectory` in Gate C, and never publishes `/isaac_joint_commands`.
  `validation/manipulation_qualification.py` gains only additive helper
  exposure (`QualificationProcessHelpers` plus module-level
  `qualification_ros_tooling_environment` / `qualification_write_json_atomic` /
  `qualification_new_suite_dir` / `qualification_source_inventory`) with the
  six-gate `run()` behavior, artifact schema, and command ordering unchanged.
  `tests/qualification_test_helpers.py` additively returns the complete
  seven-key `report_identities` and full `planning_scene_declaration`.
  New suites: `tests/test_integrated_gate_executor.py` (pure, 55 tests) and
  `tests/test_integrated_gate_executor_ros.py` (Humble generated-message, 13
  tests).  No build is required because no package-installed path changed.

- 2026-08-02 (integrated qualification Task 4 fix round 1): Made the Gate-C
  executor runnable and Task-4 evidence-complete.  The live
  `IntegratedGateExecutor` now constructs a valid isolated Humble node: a
  private `rclpy.context.Context` per executor initialized with the exact
  `ros_domain_id` in `[0,232]`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` required/
  verified, basename `tinker_integrated_gate_executor` + namespace `/` +
  `use_global_arguments=False` (FQN `/tinker_integrated_gate_executor`),
  dict-key (never `getattr`) typed action/service client creation with all nine
  action and eleven service clients asserted, a context-bound
  `SingleThreadedExecutor`, idempotent `shutdown()`, and construct -> shutdown ->
  construct reuse.  The journal recorder now subscribes to the real
  `moveit_msgs/msg/PlanningScene` type on `/planning_scene` and
  `/monitored_planning_scene` (reliable/transient-local/depth 1) with a
  ROS-only normalization helper deriving ordered world/attached/link/touch data,
  the exact fixture revision, an internal `scene_sequence`/`scene_timestamp`,
  and SHA-256 digests over ROS serialization of the full scene, ACM, and robot
  state.  The executor owns a real `PlanningSceneJournal` by default
  (loaded model touch contract, `pick_and_place/` task namespace,
  `pick_and_place/object_mesh` target, Stage-C explicit
  `("fixture-ready", "teardown")` event order, the eight forbidden
  manipulation events, fresh `planning-scene.jsonl` that fails closed when
  stale) and records `fixture-ready` via `record_diff` then `teardown` via
  `snapshot`, finalizing both journal artifacts with the exact observed graph
  projection.  `build_journal_graph_projection` now requires an explicit
  `observed_graph` input and fails closed on missing/extra interfaces, wrong
  type/QoS/source/cardinality, or an absent recorder subscriber/client; the
  executor requires `join_key_provider`, `readiness_snapshot_provider`, and
  `graph_observation_provider`, gates every goal on live readiness (config-
  authoritative operator freshness), and evaluates the three Stage-C scenario
  readiness baselines (their own revision, owned IDs, and descriptor digest).
  `run_gate_c_plan_only` dispatches joint/pose/blocked plan-only goals with
  non-empty-`planned_trajectory` enforcement (blocked expects planner
  non-success), uses separate bounded deadlines for server availability /
  acceptance / result / cancellation with `cancel_goal_async()` spun to
  completion, rejects non-Stage-C/non-plan-only scenarios before goal creation,
  and writes the complete Task-4 artifact set (`integrated-execution.jsonl/.json`,
  `moveit-plans.jsonl`, `controller-results.jsonl`, `goals/<scenario_id>.json`,
  `visual-capture-requests.jsonl`, `planning-scene.jsonl/.json`) with
  `diagnostic_only=true`, `execute_trajectory_goal_sent=false`, and
  `isaac_joint_commands_published=false`.  Humble suite expanded to 29 tests
  (real-class construction, typed subscriptions, scene normalization, stubbed
  three-scenario flow, nonempty-plan/blocked enforcement, readiness gating,
  bounded cancellation, repeated construct), and the documented sourced-Humble
  command FAILS (never skips) without the ROS runtime.  Pure suite expanded to
  66 tests (Stage-C readiness baselines, dispatch semantics, observed-graph
  fail-closed mutations, journal lifecycle/stale-jsonl/forbidden events).
  No build is required because no package-installed path changed.

- 2026-08-02 (integrated qualification Task 4 fix round 2): Made Gate-C evidence
  fail-dominant and aligned PlanningScene QoS with stock MoveIt2 Humble.  The
  authoritative final status is now computed after the plan outcome *and* every
  required evidence finalization step; any readiness/journal/graph/
  finalization/artifact-serialization/existence failure returns and persists
  `evidence-invalid` in `run_gate_c_plan_only` and `integrated-execution.json`,
  with the raw planner outcome preserved separately as `planner_status`, and
  `planning-scene.json` is always produced as a canonical failure artifact via a
  narrow `PlanningSceneJournal.finalize_failure` extension.  Every expected
  runtime/DDS/action failure (server wait, goal construction/serialization,
  `send_goal_async`, send-future spin/result, goal acceptance, result-future
  spin/result, cancellation, provider calls, artifact finalization) is converted
  into finite canonical diagnostic records with zero physical claim and exact
  zero-command/controller flags; once `fixture-ready` exists the executor always
  attempts teardown journal completion and failed finalization, and no exception
  escapes the public API.  The `/planning_scene` and `/monitored_planning_scene`
  subscriptions and the Task-3/4 observed-graph projection now use the stock
  MoveIt2 Humble RELIABLE/VOLATILE/depth-100 contract (the stale
  TRANSIENT_LOCAL/depth-1 claim is rejected); `/sim/status/planning_scene_fixture`
  stays RELIABLE/TRANSIENT_LOCAL/depth 1.  The blocked scenario now only passes
  on an explicit MoveIt planning-stage non-success allowlist (`PLANNING_FAILED`,
  `INVALID_MOTION_PLAN`, `NO_IK_SOLUTION`); request-level/unknown codes are
  `diagnostic-fail` with a recorded `error_code_classification`.  A bounded
  pre-goal scene-acquisition phase self-spins up to `scene_acquire_timeout_s`,
  `fixture-ready` requires the scene's ordered owned IDs to match the declared
  fixture contract (missing/extra/reordered/attached rejected before goal send),
  the `before` visual-capture request is durably flushed before the goal send and
  `after` only in the post-transaction phase, acceptance-timeout handling is
  truthful (canceling a client future is not proof of server-side cancellation),
  and the executor's atomic write now includes parent-directory fsync.  Humble
  suite expanded to 43 tests; pure suite to 68; journal suite to 184.  No build
  is required because no package-installed path changed.

- 2026-08-02 (integrated qualification Task 4 fix round 3): Sealed Gate-C
  artifact/scene-state consistency across the residual review findings.
  Artifact finalization is now transactional: `run_gate_c_plan_only` validates
  the graph through `PlanningSceneJournal.finalize(status, graph, json_path=None)`
  before any durable output, writes the non-journal artifacts first, and defers
  the successful `planning-scene.json` until every other required artifact is
  durable.  Any post-provisional-pass write/serialization/fsync/existence
  failure now invokes `_downgrade_persisted_evidence`, which rewrites
  `integrated-execution.json` fail-dominantly, appends `row_kind="final"`
  corrective rows to the JSONL lifecycle files (whose provisional/raw planner
  rows carry `row_kind="lifecycle"` so a consumer cannot mistake an early row
  for a completed pass), and writes a canonical failure `planning-scene.json`.
  `planner_status="diagnostic-pass"` remains available as diagnostic history
  while every authoritative `status` field is fail-dominant, and the
  `PlanningSceneJournal.finalize_failure` protection is unchanged.  Scene
  acquisition is no longer latch-permanent: a successfully normalized valid
  PlanningScene callback atomically caches the new scene and clears the invalid
  flag (recording `_scene_invalid_sequence` on failure), and acquisition
  requires a valid observation whose `scene_sequence` is after the last invalid
  one, so an invalid message never erases the last valid cached scene yet
  acquisition stays fail-closed while invalid is the newest observation.  The
  callback boundary catches the complete expected malformed-message exception
  set (AttributeError/IndexError/KeyError/TypeError/ValueError) without
  swallowing process-control exceptions.  `fixture-ready` now binds to the
  exact declared geometry and pose, not IDs only: `expected_fixture_geometry_digest`
  derives a deterministic projection via the bridge canonicalization helpers
  `fixture_to_specs`/`spec_geometry`/`readback_geometry`/`geometry_signature_sha256`
  covering the scenario-owned objects' exact ordered IDs, primitive geometry and
  dimensions, frame, and poses, and `_fixture_scene_error` compares the received
  scene's projected digest to the declaration, rejecting stale pose, wrong
  geometry/dimensions/frame, and duplicate IDs.  The blocked scenario now
  requires both an allowlisted planning-stage non-success code and an empty
  planned trajectory; an allowlisted code with a non-empty trajectory is
  classified `contradictory-nonempty-trajectory` and fails.  This entry
  supersedes the fix-round-2 "Humble suite expanded to 43 tests" count with the
  fresh 44-test baseline measured at `d911692` before fix round 3; the post-fix
  counts are Humble 58, pure 70, journal 184.  No build is required because no
  package-installed path changed.

- 2026-08-02 (integrated qualification Task 3 fix round 1): Bound the
  PlanningScene evidence semantics against the adversarial review findings.
  `validation/planning_scene_journal.py` now makes the public path fail closed:
  `record_diff` validates a real world-to-attached / attached-to-world
  transition for the exact target at `scene-attach` / `scene-detach` through a
  shared transition validator (also used by `assert_transition`), and
  `snapshot` rejects the transition labels `scene-attach`, `scene-detach`, and
  `task-cleanup` that cannot truthfully be represented without a scene diff.
  Object disappearance is gated for every diff, not just `task-cleanup`;
  `teardown` alone may additionally remove `sim_fixture/*`.  `finalize` rejects
  an empty journal and, when a required event order is declared, requires the
  recorded events to equal it exactly (rejecting extra, duplicate, out-of-order,
  and spurious negative-teardown events).  Returned records and final
  records/graph are deep-copied so caller mutation can never diverge the JSONL
  from final JSON.  Appending onto a pre-existing non-empty JSONL path fails
  closed.  Graph validation requires the exact topic/service key sets, the
  recorder node among topic subscribers and service clients, exact QoS key sets
  with a non-boolean integer `depth`, and exact (never string-coerced) fixture
  payload field types with unique `sim_fixture/*` owned ids; the exact canonical
  compact fixture payload string is retained as the payload evidence.  Nested
  scene types are strict (`ValueError`, no `str()` coercion, no uncaught
  `TypeError`), and `_append` independently re-validates the full scene.  The
  focused suite grew to 181 tests.

- 2026-08-02 (integrated qualification Task 3): Added the stable, durable
  PlanningScene journal consumed by the later executor/verifier/evidence tasks.
  `validation/planning_scene_journal.py` is the ROS-free pure record/transition
  journal: a model-contract loader (`load_model_touch_contract`) anchored to the
  committed `integration/ompl-overlay-contract.json` (`link_tcp`, the exact
  ordered eight-link SRDF touch set, and handoff
  `pick_and_place/object_mesh`, failing closed on drift), transactional
  validation (a rejected record/event leaves `_records`, `_last_scene`, JSONL
  bytes, and final JSON bytes unchanged), recursive rejection of forbidden
  physics fields (contact/force/object-pose/evaluator-metric/physical-verdict),
  append-only canonical compact sorted-key `planning-scene.jsonl` (flush +
  fsync before in-memory visibility), atomic final `planning-scene.json`
  (temp-file + file fsync + `os.replace` + directory fsync, no temp residue),
  graph-evidence validation (`validate_graph_evidence`) over a Task-4-supplied
  projection (recorder identity, topic/service types and QoS, real
  endpoint/provider metadata, exactly one `/fixture_planning_scene` publisher,
  and the exact canonical compact fixture-status payload with scalar
  `target_handoff="pick_and_place/object_mesh"`), and positive/negative event
  ordering with attach/detach/task-cleanup ownership semantics.  The journal
  records the exact `(frame_index, timestamp)` join key and never claims that
  scene attachment proves physical contact.  `tests/test_planning_scene_journal.py`
  covers every brief case plus adversarial digest/rollback/numeric/duplicate/
  physics-key/contract-drift/graph/durability/order coverage.

- 2026-08-02 (integrated qualification Task 2 fix round 3): Closed the residual
  semantic false-passes.  `validation/integrated_static_contracts.py`
  `_bind_run_post_close_pick` now aggregates every SimOmpl/Hardware profile
  block and validates the whole set (exactly one obstruction guard, one SimOmpl
  lift block, one Hardware lift block; per-block polarity; structural
  containment of every `execute_lift(...)` call by an accepted profile lift
  block) instead of checking only the last matching block, so a decoy block
  after a violating one can no longer pass.  Result-builder field checks use
  the exact assignment form `\bresult\s*->\s*<field>\s*=(?!=)` over sanitized
  executable code, so a differently-named member, a read, or a comparison never
  satisfies the write contract.  `_bind_bundle_artifact` binds a non-simulator
  artifact by its semantic source key plus exact path identity (bundle absolute
  path == `<repo_path>/<path_relative>`, overlay workspace path ==
  `src/<repo dir>/<path_relative>`) and requires the recorded digest to equal
  the verified source record; simulator-local acceptance is restricted to the
  declared overlay path under `outputs/` or `artifacts/`, and a shadow file
  with matching bytes at an undeclared path never reclassifies.  The Task 2
  report now records only independently reproduced test evidence.

- 2026-08-02 (integrated qualification Task 2 fix round 2): Strengthened the
  semantic source checks.  `validation/integrated_static_contracts.py` replaces
  comment-stripped string-presence C++ scans with a deliberately scoped
  lexical/structural layer: comments, string/character and raw-string literals
  are sanitized (preserving braces/newlines), conditional-preprocessor
  directives inside load-bearing function bodies are rejected (so `#if 0` /
  `#ifdef` dead-code anchors cannot satisfy a check), and each inspected
  function (result builders, `GraspNode::~GraspNode`,
  `GraspNode::clean_planning_scene`, `GraspNode::move_straight`,
  `run_post_close_pick`, `coordinator_main`, `~MotionRuntime`) is located by
  its actual signature and brace-matched.  The Sim/Hardware `execute_lift`
  boolean is bound to its own `ExecutionProfile` branch, the destructor
  deadline/shutdown/join/state-validity reset are bound to the destructor body,
  and each task result builder must write exactly its `.action` schema fields.
  The authoritative overlay `model_bundle.production_source_commits` is bound
  to immutable Git blobs (40-hex commit existence, blob digest, manipulation
  ancestry vs `implementation_head`; external `tk25_basic` via exact recorded
  commit/blob only) and drives both the model-artifact binding and the
  pinned-prerequisite source-identity check.  `fixture-ownership` now requires
  the overlay `scenarios` key set to equal the configured C/D/E set exactly.
  `source_lock_manifest.py` closes the observer gaps: qualification-tooling
  requires clean mode with empty evidence, `authorization.report_path` must
  resolve (no symlink/canonical escape) to a regular file inside the
  repository, `_normalize_posix` rejects `..` and canonicalizes absolute
  repository paths, the stale mtime-fallback docstring is removed, and the
  output manifest is written/fsynced/replaced exactly once from a single
  pre-write timestamp.

- 2026-08-02 (integrated qualification Task 2 fix round 1): Bound Gate B to
  real artifacts.  `validation/integrated_static_contracts.py` no longer reads
  the fabricated `qualification/records/*` layer: every production source
  (SRDF `_xarm7_macro.srdf.xacro`, `config/xarm7` and `config/xarm_gripper`
  `controllers.yaml`, the pick_and_place C++ runtime and result writers, the
  `.action` schemas, and the selected launch) is inspected as an immutable Git
  blob at the production `implementation_head` from the produced source-lock
  manifest.  The model-fingerprint check recomputes
  `structural_fingerprint == sha256(canonical bundle contract)`, binds every
  recorded model artifact SHA-256 and the `production_source_commits` blobs,
  and proves the ordered `xarm_gripper` members plus `link_tcp` end-effector
  parent.  `fixture-ownership` inspects exactly the 17 configured scenarios
  (C/D/E) and derives owned ids from `planning_scene.objects` in declared
  order versus `integrated.expected_scene.owned_ids`.  The transport-contract
  check reads `typed_contract.runtime_contract_sha256` from its sibling
  location, recomputes the canonical full runtime mapping, and preserves the
  exact public one-key mapping.  Provider checks recompute the
  `provider_manifest_sha256` canonical self-hash and raw bytes and reconcile
  the provider executable set with the overlay.  `source_lock_manifest.py`
  gained closed policy/authorization schemas, the resolved lock-commit scope
  rule (`{policy_path, docs/acceptance.md}`), the qualification HEAD exact-match
  requirement, authorization report existence/predate plus bogus/valid
  authorization-commit checks, canonical repository-relative `policy_path`,
  required ISO `started_at`, and a stable output-freshness boolean.  The
  static checker emits canonical finite JSON with fsync + atomic replace for
  `static-contract.json`, `model-fingerprint.json`, and
  `source-identities.json`, and propagates `evidence-invalid` /
  `verified-fail` / `verified-pass` per F4.  The real-root post-fix Gate B
  capture remains `evidence-invalid` solely because the future
  qualification-tooling policy is intentionally absent; eight of the nine
  static checks pass against the real simulator/production roots and only
  `source-identities` fails on that absent policy.

- 2026-08-02 (integrated qualification Task 2): Added the offline Gate B
  static-contract closure.  `validation/source_lock_manifest.py` observes the
  three immutable source-lock authorizations
  (`simulator_overlay` / `production` / `qualification_tooling`) with
  non-self-referential Git-history resolution, exact `LC_ALL=C` raw
  status/diff/index/untracked evidence, a canonical path-sorted untracked
  manifest, an atomic fsync+`os.replace` output manifest, and attempt
  freshness.  `validation/integrated_static_contracts.py` runs the nine
  semantic static checks (model, controllers, providers, fixture ownership,
  action lifecycle, scene/collision safety, source identities, transport
  contract) against the produced three-entry manifest.
  `simulation/qualification/integrated-ompl.json` now names exactly the three
  `source_lock_policies`.  The real qualification-tooling policy is
  intentionally absent during Task 2, so a real-root Gate B capture honestly
  returns `evidence-invalid` until the post-Task-10 lock-only phase; no
  existing source-lock policy was created or modified.
- 2026-08-02 (fix round 2): Separated static and runtime OMPL acceptance
  evidence.  The 16-check preflight is now self-contained: the real unmodified
  `preflight_manifest` runs against a reconstructed Task 3-compatible project
  root (committed source + pinned git objects + a legacy `current.json` selector
  pointing at the reproduced canonical URDF), so `ready=true` and the stable
  preflight hash reproduce with no gitignored `outputs/`/`artifacts/`
  dependency.  The real `current.json` is a separate provisioned-host runtime
  readiness diagnostic (stale selection fails; absent selection is reported
  `not_provisioned`, not a static failure).  Top-level `evidence.preflight` now
  carries the load-bearing `stable_manifest_sha256`/`stable_preflight_sha256`
  and nests the raw hashes under `host_snapshot`.  Task 7 action-client scope is
  AST-grounded in pinned Task 7 source (exactly one MoveGroup client on
  `/move_action`); task_range boundary subjects + exact Task 3-7 commit list are
  recorded and recomputed; fixture status publication is recomputed from Task 5
  source; every artifact `path_relative` follows the recorded source policy; a
  temporary copy-install/wheel test proves the symlinked contract + scenarios
  install as real byte-identical files; the pinned v1 canonicalizer is loaded by
  deterministic temporary-package materialization (no `exec` string surgery);
  joint-limit acceptance is semantic + toolchain-aware; and a clean-checkout
  regression seam (`git clone` of the tracked tree) requires every static test to
  pass with no gitignored trees.
- 2026-08-02 (fix round 3): Locked the clean-checkout acceptance coverage.  The
  regression seam now collects the exact 64-node `Task8OMPLOverlayProvenanceTest`
  class in the clone (canonical SHA-256 over the sorted node list, so a deleted
  or added test fails even if the count is preserved) and executes it under a
  machine-readable JUnit XML, requiring total=64, failures=0, errors=0, skipped=4,
  the exact four host-runtime diagnostic skip names, and their reason categories —
  an unrelated static assertion broadened into a skip is rejected.  The
  reconstructed Task 3 resolver (`tools/tinker_sim_deploy`) is now materialized
  from immutable git objects at the recorded simulator implementation identity
  (never copied from the live working tree, so local edits cannot influence
  acceptance), and a pinned-blob mismatch/missing file fails closed.  The
  fixture status field contract is asserted against an independent 12-field
  literal (not re-derived from the source function), and a stale
  `current.json` selector is proven to fail the preflight `artifact_identity`
  check.  The fix-1 baseline is documented as 2 failed + 1 skipped.
- 2026-08-02 (fix round 4): Isolated the pinned-resolver verification.  Because
  the test module pre-imports `tinker_sim_deploy` from the live working tree,
  Python's module cache masked the earlier in-process dirty live-tree probe
  (the proof did not actually execute the pinned materialized resolver).  The
  reconstruction proof now runs in a fresh isolated subprocess (`-I`, no
  inherited `tinker_sim_deploy*` cache) that loads `tinker_sim_deploy.runtime`
  from the materialized temp root, records its resolved `__file__`, runs the
  real Task 3 preflight, and emits one machine-readable JSON with
  ready/16-check/exact-stable-hash evidence; a temp decoy working-tree package
  proves the pinned path wins path precedence (positive) and is load-bearing
  (negative).  No test writes a tracked active-checkout file.  The clean-checkout
  seam's collection/JUnit acceptance checks are factored into one deterministic
  validator used by both the real clone output and realistic mutated fixtures
  (delete, rename+add, unrelated skip, removed skip, wrong reason, duplicate
  testcase, failure/error count, multiple suites), and the JUnit structure is
  tightened to exactly one `<testsuite>` with 64 unique `<testcase>` entries.
- 2026-08-02 (fix round 1): Made the OMPL-overlay acceptance evidence
  reproducible.  The production-overlay scope is split into the exact production
  allow-lists (from `launch_contract_helpers.py`, including
  `re`/`importlib`/`rviz2`) and a simulator-overlay provider set derived from
  the provider manifest; the provenance suite now recomputes the 18-argument
  contract, literal booleans, strict-sim keys, allow-lists, handoff, and
  action-client scope from immutable production git objects.  The model-bundle
  evidence is reproducible on a clean checkout: the canonical simulator full
  URDF source + provenance descriptor are committed under
  `integration/model-bundle-r2/simulator_full_urdf/`, and stable manifest /
  preflight hashes exclude host-absolute paths and `elapsed_ms`.  The
  acceptance contract and three qualification scenarios are installed under
  `share/tinker_sim_bridge/{integration,scenarios}/` with a deterministic
  launch package-share fallback.  `tests/test_provenance.py` fails (never
  skips) when required source evidence is absent, and the documented focused
  pytest invocation uses `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`.  The two
  source-lock files remain Task 9 only.
- 2026-08-02: Added the deterministic OMPL overlay acceptance contract
  (`integration/ompl-overlay-contract.json`) packaging the reviewed Tasks 3-7
  interfaces: exact repository/commit identities (simulator `f34de5f`;
  production runtime hardening `f3e2ce4..df702a5` and Task 2 launch
  `f7fea50..39d96a1` from actual git history), production overlay and
  literal-false `use_cumotion_*` compatibility values, provider-manifest raw +
  canonical identities, model-bundle schema/artifact hashes and full eight-link
  touch set, typed action/service/topic contract, fixture/scenario identities,
  Task 6/7 evidence, and `tkbuild`/`build-humble-overlay` build commands.
  `tests/test_provenance.py` recomputes every derived hash/contract.  The two
  source-lock files remain Task 9 only.
- 2026-07-30: Added deterministic exporter-side canonical URDF derivation with
  full-SHA immutable generation identity, exact portable source-lock identity,
  serialized crash-consistent publication, physical arm-joint validation,
  relocatable Humble launch inputs, shared runtime validation, and bundle
  admission checks. The USD physics bytes remain unchanged.
