# Manipulation operator workflow

This document describes the current development launch slice for Tinker 2.
It is an operator workflow and graph contract, not evidence of a live
manipulation pass or release qualification.

## Two-process development run

Use the isolated project at `/home/tinker/tinker-sim/6.0.1`. Keep
`/home/tinker/tk25_ws` read-only. Build the external Humble overlay once:

```bash
cd /home/tinker/tinker-sim/6.0.1
./scripts/build-humble-overlay
```

Terminal A runs Isaac. Do not source `/opt/ros/humble` in this terminal:

```bash
cd /home/tinker/tinker-sim/6.0.1
export TINKER_ACCEPT_OMNIVERSE_EULA=Y
./scripts/launch-isaac --sensor-profile manipulation-core --profile parity \
  --scenario pick-deliver-place --seed 7 --ros
```

Terminal B runs the system-Humble side:

```bash
cd /home/tinker/tinker-sim/6.0.1
export TINKER_WS=/home/tinker/tk25_ws               # required by launch-humble
./scripts/launch-humble manipulation scenario:=pick-deliver-place seed:=7 \
  qualification:=false attempt_dir:=outputs/manipulation-dev-7
```

The launch accepts `project_root`, `tinker_workspace`, `scenario`, `seed`,
`qualification`, and `attempt_dir`. The scenario runner uses only standard
`simulation_interfaces` services and waits for them before issuing operations.

## Readiness checks

Run these checks after both terminals are up:

```bash
ros2 topic list | rg '^/(clock|isaac_joint_states|isaac_joint_commands)$'
ros2 service list | rg '^/(get_simulation_state|set_simulation_state|reset_simulation|spawn_entity)$'
ros2 action list | rg '^/xarm7_traj_controller/follow_joint_trajectory$'
ros2 action list | rg '^/xarm_gripper/gripper_action$'
ros2 topic info -v /isaac_joint_commands
ros2 topic echo --once /sim/status/contract std_msgs/msg/String
ros2 topic echo --once /sim/status/command_gateway std_msgs/msg/String
```

Expected graph properties are:

- `/clock` and `/isaac_joint_states` are present.
- Standard simulation services are present; `/sim/control/*` and
  `/sim/scenario/*` aliases are absent.
- The FJT and gripper action surfaces are discoverable.
- `/isaac_joint_commands` has exactly one publisher, the command gateway.
- Contract status is `pass` after its startup grace period.
- The scenario runner reports `control_api: simulation_interfaces` and waits
  for service availability rather than using custom lifecycle services.

These checks establish launch readiness only. They do not prove joint tracking,
contact, safety response, object retention, or task success.

## Qualification command

Qualification uses the auditable development attempt runner. This command is
a manifest preflight only:

```bash
cd /home/tinker/tinker-sim/6.0.1
./.venv/bin/python validation/manipulation_qualification.py \
  --scenario simulation/scenarios/qualification-free-space.json \
  --manifest-only
```

This creates a manifest-only attempt and makes no live-pass claim. Without
gate executors the runner records `not-configured` and does not start
simulator processes. External `--gate-command NAME=...` commands are recorded
as `executed-unverified` when they exit zero; they can never produce a
qualification pass until built-in evidence recomputation exists.

## Deterministic OMPL plan-only smoke (Task 7)

`validation/ompl_plan_smoke.py` is deterministic plan-only qualification tooling
on top of the review-clean integrated readiness boundary.  It does **not**
begin cuMotion and does **not** claim a live plan without a running, qualified
graph: the client first requires a fresh `pass` on
`/sim/status/integrated_manipulation`, then verifies `/move_action` is exactly
one `moveit_msgs/action/MoveGroup` action server with observed
action-kind/type/cardinality/source metadata, then sends a goal with
`request.pipeline_id="ompl"` and `planning_options.plan_only=true` while
observing `/isaac_joint_commands` for zero command samples across the full
request/result window.  The MoveGroup action client is the only action client;
no execute-trajectory/controller/task action client is constructed.

- `validation/ompl_goal_builders.py` — ROS-free plain-data goal builders
  (`build_joint_goal`, `build_pose_goal`) that run under simulator Python 3.12.
- `validation/ompl_plan_smoke.py` — pure evaluator/serializer plus the live
  Humble client seam.  `rclpy`, `rclpy.action`, and `moveit_msgs` are imported
  only inside `main()` / the `OmplPlanSmokeClient` methods.
- `tests/test_ompl_plan_smoke.py` — pure CPython 3.12 contract tests.

Pure tests (simulator Python 3.12):

```bash
cd /home/tinker/tinker-sim/6.0.1
PYTHONPATH="$PWD/validation:$PWD/ros2_ws/src/tinker_sim_bridge" \
  ./.venv/bin/python -m pytest -q tests/test_ompl_plan_smoke.py
```

### Three-terminal live workflow

All three terminals use `ROS_DOMAIN_ID=25` and the **same**
`TINKER_SIM_DDS_PROFILE=local|lan`.

**Terminal A — Isaac Sim** (do not source `/opt/ros/humble` here):

```bash
cd /home/tinker/tinker-sim/6.0.1
export TINKER_ACCEPT_OMNIVERSE_EULA=Y
export ROS_DOMAIN_ID=25
export TINKER_SIM_DDS_PROFILE=local        # or lan
./scripts/launch-isaac --sensor-profile manipulation-core --profile parity \
  --scenario qualification-moveit-plan-joint --seed 7 --ros
```

**Terminal B — Humble overlay** (sourced system Humble; scenario must match the
smoke `--mode` below):

```bash
cd /home/tinker/tinker-sim/6.0.1
export TINKER_WS=/home/tinker/tk25_ws               # required by launch-humble
export ROS_DOMAIN_ID=25
export TINKER_SIM_DDS_PROFILE=local        # or lan, must match Terminal A
export TINKER_SIM_MODEL_BUNDLE_MANIFEST=outputs/ompl-overlay/model-bundle-r2/model-bundle.json
export TINKER_SIM_PROVIDER_MANIFEST=ros2_ws/src/tinker_sim_bridge/integration/provider-manifest.json
./scripts/launch-humble integrated-ompl scenario:=qualification-moveit-plan-joint \
  seed:=7 qualification:=false attempt_dir:=outputs/ompl-plan-smoke/attempt-joint
```

Wait for `/sim/status/integrated_manipulation` to publish `pass` before running
Terminal C.

**Terminal C — smoke client** (sourced system Humble Python 3.10):

```bash
cd /home/tinker/tinker-sim/6.0.1
source /opt/ros/humble/setup.bash
source /home/tinker/tk25_ws/install/setup.bash
export ROS_DOMAIN_ID=25
export TINKER_SIM_DDS_PROFILE=local        # or lan, must match
# The smoke imports the Task 6 canonical helpers (identity digests) from the
# tinker_sim_bridge source tree, plus the ROS-free goal builders in validation/.
export PYTHONPATH="$PWD/ros2_ws/src/tinker_sim_bridge:$PWD/validation:$PYTHONPATH"
python3 validation/ompl_plan_smoke.py --mode joint \
  --report outputs/ompl-plan-smoke/ompl-plan-smoke.json
```

### Scenario selection and expected terminal outcomes

| Mode | Terminal B `scenario:=` | Smoke `--mode` | Expected report |
|---|---|---|---|
| Joint | `qualification-moveit-plan-joint` | `joint` | `evaluation.ready=true`, `outcome.kind="success"`, `trajectory_point_count >= 1`, `command_observations.samples == 0` |
| Pose | `qualification-moveit-plan-pose` | `pose` | same as joint |
| Blocked | `qualification-moveit-plan-blocked` | `blocked` | `evaluation.ready=true`, `outcome.kind="non_success"`, accepted goal ending `STATUS_ABORTED` with an explicit MoveIt planning/collision/constraint failure code (e.g. `PLANNING_FAILED`/`GOAL_IN_COLLISION`), `command_observations.samples == 0` |

The smoke exits 0 on `evaluation.ready=true` and 1 otherwise.  A mode/scenario
mismatch (the scenario's `qualification_gate` is not `moveit-plan-<mode>`) is
rejected fail-closed before any goal is sent.  Joint mode plans to a small
reach from a vertical arm; pose mode targets a point
`POSE_APPROACH_Z_OFFSET` above the scenario's `target` object; blocked mode
targets the interior of the `blocker` object so every goal sample is in
collision, giving a deterministic non-success.  Blocked acceptance requires an
accepted goal ending `STATUS_ABORTED` (or a `STATUS_SUCCEEDED` terminal carrying
a non-success MoveIt result) with an explicit MoveIt planning/collision/
constraint failure code; a rejected goal, an empty/default (`0`/`None`) code, a
cancel, a result timeout, a transport/setup failure, or an unknown terminal
status is always rejected fail-closed.

If the readiness gate never publishes a fresh `pass` within
`--readiness-timeout`, the client writes a compact canonical fail-closed report
(`evaluation.ready=false` with an exact `blocker` reason) and exits nonzero.
This bounded fail-closed invocation can be verified without a live overlay:

```bash
cd /home/tinker/tinker-sim/6.0.1
source /opt/ros/humble/setup.bash
source /home/tinker/tk25_ws/install/setup.bash
export ROS_DOMAIN_ID=25
export TINKER_SIM_DDS_PROFILE=local
export PYTHONPATH="$PWD/ros2_ws/src/tinker_sim_bridge:$PWD/validation:$PYTHONPATH"
python3 validation/ompl_plan_smoke.py --mode joint --readiness-timeout 5 \
  --report outputs/ompl-plan-smoke/ompl-plan-smoke-failclosed.json
```

## Artifact policy

Use a unique `attempt_dir` for every run. Preserve successful and failed
attempts, including startup failures, timeouts, crashes, and contract failures.
An attempt directory may contain the scenario-runner report, launch logs, ROS
graph and publisher snapshots, bags, raw truth, controller feedback/results,
contact records, evaluator traces, manifests, and process exit codes.

Never overwrite or delete a failed attempt. Record the source and artifact
hashes, scenario and seed, thresholds, ROS/DDS settings, CPU-physics settings,
commands, and tool versions before interpreting results. The manifest and
evidence index use SHA256 checksums; filesystem permissions do not make an
attempt immutable. Do not bypass
`/isaac_joint_commands`, teleport an object after spawn, fabricate truth, or
use an action return code as a physical postcondition.

## Acceptance contract (Task 8)

`integration/ompl-overlay-contract.json` is the deterministic acceptance
contract for the reviewed OMPL overlay (Tasks 3-7).  It packages the reviewed
interfaces and writes the acceptance contract; it does not itself prove live
OMPL or authorize cuMotion.  It is a single canonical JSON document
(schema version 1, sorted keys, minimal separators, no timestamps, no
host-transient data, and no self-referential Task 8 commit hash).

The contract records, with exact values:

- **Repository identities.**  The simulator implementation identity is commit
  `f34de5f4cd472e2dbb50d65eb53e089bb1c84891` (clean baseline).  The production
  implementation identity is recorded from the actual git history, not a
  mutable concurrent HEAD: runtime hardening is the range
  `f3e2ce4f6e00b23f9b35fef14555ff48d8993058..df702a573f971bb3e2008789adc882c09567de7a`
  (canonical OMPL overlay consumer + pick_and_place/xarm_controller hardening),
  and the Task 2 production launch is the range
  `f7fea50b5e15ba22deb9d2ec401097056519bf97..39d96a176904c0b7966b11333c5517b3b54b6ae3`
  (mobile_bringup planning/task-only launch).  Clean/dirty policy is recorded
  for both repositories: the simulator identity is the clean tree; the
  production workspace is a read-only runtime input whose local modifications
  are not part of the recorded identity.
- **Production overlay.**  Package `mobile_bringup`, launch file
  `manipulation_planning_task_only.launch.py`
  (`src/mobile_bringup/launch/manipulation_planning_task_only.launch.py`), the
  exact 18-argument launch contract, and the literal-false
  `use_cumotion_object_attachment` / `use_cumotion_goalset` /
  `use_cumotion_straight_approach` / `esdf_freshness_wait_enabled`
  compatibility values (and literal-true `safety_required` /
  `fixture_revision_required` / `use_sim_time`).  The provider/import/action
  client allow-lists, the task-owned lifecycle
  (`pick_and_place` creates and owns `pick_and_place/object_mesh` after the
  hardening prerequisite), and the exact 7-step staging sequence are recorded.
- **ROS policy.**  Humble, `ROS_DOMAIN_ID=25`, `rmw_fastrtps_cpp`, and the
  `local`/`lan` Fast DDS profiles.
- **Provider manifest.**  The committed
  `ros2_ws/src/tinker_sim_bridge/integration/provider-manifest.json` is
  recorded verbatim with its canonical self-hash
  `4bc177890393b5b6d434e17aed3dc85889e55efce7cb9874a6d4c4575bc1362b` and the
  raw-byte digest.  Persistent nodes, one-shot processes/lifecycle, logical
  controller resources, and publishers are distinct sections and are never
  collapsed.
- **Model bundle.**  The canonical schema, the unchanged manifest path
  `outputs/ompl-overlay/model-bundle-r2/model-bundle.json`, the artifact
  hashes, the semantic kinematics/model contract, the normalized selected
  subgraph, the full eight-link SRDF touch set, and the preflight report
  (16 checks, `ready=true`).
- **Typed contract.**  Every typed action (including `/move_action` as
  `moveit_msgs/action/MoveGroup`, `/execute_trajectory`, gripper/FJT,
  pickup/place, Cartesian/Joint/Fold), the typed `/arm_joint_service`
  (`tinker_arm_msgs/srv/ArmJointService`), the controller-manager services,
  MoveIt scene services, gate services, and every typed publisher with exact
  type/source/cardinality/stamp/QoS policy — including the corrected
  `/isaac_joint_commands` depth 50 and the external future
  `/tinker_integrated_gate_executor` ownership of `/sim/safety/operator`.
  The public `scenario-runner.json` report carries the one-key
  `{"execution_profile": "sim_ompl"}` mapping exactly as the shipped
  `pick_and_place` canonical parser requires, while the full runtime contract
  is carried separately as `runtime_contract_sha256`.
- **Fixture/scenario identities.**  Exact `sim_fixture/*` ownership and parser
  encoding, canonical `target_source_id="sim_fixture/public_target"`,
  `target_handoff="pick_and_place/object_mesh"`, and the full Task 1
  17-scenario integrated matrix (three OMPL plan-only C scenarios, six D
  execute scenarios, and eight E pick-place scenarios) with their
  declaration/revision/digest/owned-ID/descriptor identities.  The plan-only
  and execute scenarios target `sim_fixture/public_target`; the pick-place
  scenarios target `sim_fixture/qualification_cube` and declare the source
  pedestal plus a place-support pedestal (`sim_fixture/place_pedestal`, top z
  0.60).  Each scenario is scenario-v2 and its scenario-v2 loader validates the
  `integrated` mapping; schema-v3 applies to the qualification config.  Fix
  round 1 aligned physical bottom-origin roots with PlanningScene center-origin
  poses (cube root z 0.60 / center z 0.64; blocker root z 0.70 / center z 0.85)
  so the declared target TCP is covered without initial target contact.  The
  public report carries only the one-key `{"execution_profile": "sim_ompl"}`
  mapping and the full per-scenario mapping is bound by the scenario declaration
  SHA-256.
- **Evidence.**  Task 6 runtime/public-report separation and Task 7 plan-only
  joint/pose/blocked plus zero-command evidence and the exact blocked-mode
  MoveIt failure-code allowlist.
- **Build commands.**  `MAKEFLAGS='-j2 -l2'
  /home/tinker/tk25_ws/tkbuild tk25_manipulation --parallel-workers 2` for the
  production workspace and `MAKEFLAGS='-j2 -l2'
  TINKER_WS=/home/tinker/tk25_ws ./scripts/build-humble-overlay` for the
  simulator overlay — never raw colcon.
- **Source locks.**  Both repository-local source-lock files are Task 9 only;
  Task 8 does not create or modify either.

`tests/test_provenance.py` recomputes every derived hash/contract from
immutable git objects and committed source and fails on mutations (including
argument order/count, literal compatibility booleans, strict-sim keys,
production import/node/executable allow-lists, simulator provider set, handoff,
Task 7 action-client scope, task-range boundary/commits, fixture status
publication, artifact path policy, model-bundle source evidence, top-level
stable hashes, stale current selection, and premature source-lock inclusion).
The static acceptance evidence is clean-checkout reproducible: the 16-check
preflight runs the real `preflight_manifest` against a self-contained
reconstructed Task 3-compatible project root (committed source + pinned git
objects + a legacy `current.json` selecting the reproduced canonical URDF),
requiring `ready=true`, all 16 checks, and the stable preflight hash with no
gitignored `outputs/`/`artifacts/` dependency.  The real `current.json` is a
separate provisioned-host runtime-readiness diagnostic (stale selection fails;
absent selection is reported `not_provisioned`).  A clean-checkout regression
seam (`git clone` of the tracked tree) collects the exact 64-node
`Task8OMPLOverlayProvenanceTest` class (canonical node-set SHA-256, so a
deleted/added test fails even at a preserved count) and executes it under a
machine-readable JUnit XML, requiring total=64, failures=0, errors=0, skipped=4,
the exact four host-runtime diagnostic skips, and their reason categories.  The
collection/JUnit acceptance checks are factored into one deterministic validator
applied to both the real clone output and realistic mutated fixtures (delete one
node, rename+add substitution, an unrelated skip, a removed expected skip, a
wrong skip reason, a duplicate testcase, a failure/error count, multiple
suites), and the JUnit structure is tightened to exactly one `<testsuite>` with
64 unique `<testcase>` entries.  The reconstructed `tools/tinker_sim_deploy`
resolver is materialized from immutable git objects at the recorded simulator
implementation identity (never the live working tree).  Because the test module
pre-imports `tinker_sim_deploy` from the live checkout, the pinned-resolver
proof runs in a fresh isolated subprocess (`-I`, no inherited module cache)
that records the loaded `tinker_sim_deploy.runtime.__file__` from the
materialized temp root and runs the real Task 3 preflight, with a temp decoy
working-tree package proving the pinned path wins precedence (positive) and is
load-bearing (negative); no test writes a tracked active-checkout file.  The
fixture status field contract is asserted against an independent 12-field
literal.  The pre-existing uv environment provenance failure
(installed `uv 0.12.0` vs pinned `uv 0.10.8`) remains an environment failure,
not a code failure.

The focused provenance invocation uses `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` as the
defensive reproducible form (ROS plugin discovery may auto-load the Humble
`launch_pytest` plugin, which can fail collection with
`ModuleNotFoundError: No module named 'lark'` on hosts that do not provide that
dependency):

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ./.venv/bin/python -m pytest -q tests/test_provenance.py
```

`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` is documented in the root `README.md` and the
bridge `README.md` verification blocks as the reproducible invocation; it does
not hide the pre-existing pinned-uv failure.

## Deferred work and status

This is development-only and not release-qualified. The current tree has no
live manipulation pass to report. MoveIt and cuMotion remain deferred until
the FJT, safety, gripper/contact, collision, and retention gates are backed by
raw physics evidence. Vision, decision, and VLA are also deferred until that
manipulation core is qualified.

## Changelog

- 2026-08-02 (integrated qualification Task 4): Added the ROS-lazy Gate-C OMPL
  plan-only executor `validation/integrated_gate_executor.py`.  The module
  imports cleanly under the simulator CPython 3.12 venv (all `rclpy` /
  generated-message imports are confined to `_load_ros()` and the ROS-only
  goal-builder call paths).  It carries the exact action/service/topic graph
  contract (types, cardinality, real provider nodes, QoS; `/joint_states`
  reliable/volatile/depth 10, fixture/operator/safety reliable/transient-local/
  depth 1, `/isaac_joint_commands` observation depth 50), the real
  multi-operation canonical public report validation over the one-key
  `{"execution_profile": "sim_ompl"}` integrated mapping with
  scenario-declaration-bound fixture descriptor digest, the readiness
  evaluator over the genuine positive-ready baseline, the Task 3 journal graph
  projection matching `planning_scene_journal.validate_graph_evidence`, and the
  live `/tinker_integrated_gate_executor` node whose Gate C flow sends only
  plan-only `/move_action` goals (`group_name="xarm7"`, `pipeline_id="ompl"`,
  `num_planning_attempts=3`, `allowed_planning_time=3.0`,
  `planning_options.plan_only=True`, `replan=False`), never calls
  `/execute_trajectory`, and never publishes `/isaac_joint_commands`; plan-only
  evidence stays `diagnostic_only=true`.  `validation/manipulation_qualification.py`
  only exposes additive process/recorder/provenance helpers
  (`QualificationProcessHelpers` and module-level `qualification_*` wrappers)
  around its existing mechanics; the six-gate `run()` behavior is unchanged.
  `tests/qualification_test_helpers.py` additively returns the complete
  seven-key `report_identities` and the full `planning_scene_declaration`.
  New `tests/test_integrated_gate_executor.py` (pure, 55 tests) and
  `tests/test_integrated_gate_executor_ros.py` (Humble generated-message, 13
  tests).  No package-installed path changed, so no build is required.

- 2026-08-02 (integrated qualification Task 4 fix round 1): Made the Gate-C
  executor runnable and Task-4 evidence-complete.  The live
  `IntegratedGateExecutor` now constructs a valid isolated Humble node through a
  private `rclpy.context.Context` per executor initialized with the exact
  `ros_domain_id` in `[0,232]`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` required/
  verified, node basename `tinker_integrated_gate_executor` + namespace `/` +
  `use_global_arguments=False` (FQN `/tinker_integrated_gate_executor`), dict-key
  (never `getattr`) typed action/service client creation (all nine action and
  eleven service clients asserted), a context-bound `SingleThreadedExecutor`,
  idempotent `shutdown()`, and construct -> shutdown -> construct reuse.  The
  journal recorder subscribes to the real `moveit_msgs/msg/PlanningScene` type on
  `/planning_scene` and `/monitored_planning_scene` (reliable/transient-local/
  depth 1) and normalizes real scenes (ordered owned/attached/link/touch data,
  exact fixture revision, internal `scene_sequence`/`scene_timestamp`, SHA-256
  digests over ROS serialization of the full scene, ACM, and robot state).  The
  executor owns a real `PlanningSceneJournal` by default (loaded model touch
  contract, `pick_and_place/` task namespace, `pick_and_place/object_mesh` target,
  Stage-C explicit `("fixture-ready", "teardown")` event order, the eight
  forbidden manipulation events, fresh `planning-scene.jsonl` that fails closed
  when stale) and records `fixture-ready` via `record_diff` then `teardown` via
  `snapshot`, finalizing both journal artifacts.  `build_journal_graph_projection`
  now requires an explicit `observed_graph` input and fails closed on missing/
  extra interfaces, wrong type/QoS/source/cardinality, or an absent recorder
  subscriber/client.  The executor requires `join_key_provider`,
  `readiness_snapshot_provider`, and `graph_observation_provider`, gates every
  goal on live readiness (config-authoritative operator freshness), and
  evaluates each Stage-C scenario's own readiness baseline (revision, owned IDs,
  descriptor digest).  `run_gate_c_plan_only` dispatches the joint/pose/blocked
  plan-only goals with non-empty-`planned_trajectory` enforcement (blocked
  expects planner non-success), uses separate bounded deadlines for server
  availability / acceptance / result / cancellation with `cancel_goal_async()`
  spun to completion, rejects non-Stage-C/non-plan-only scenarios before goal
  creation, and writes the complete Task-4 artifact set
  (`integrated-execution.jsonl/.json`, `moveit-plans.jsonl`,
  `controller-results.jsonl`, `goals/<scenario_id>.json`,
  `visual-capture-requests.jsonl`, `planning-scene.jsonl/.json`) with
  `diagnostic_only=true`, `execute_trajectory_goal_sent=false`, and
  `isaac_joint_commands_published=false`.  Humble suite expanded to 29 tests;
  the documented sourced-Humble command FAILS (never skips) without the ROS
  runtime.  Pure suite expanded to 66 tests.  No package-installed path changed,
  so no build is required.

- 2026-08-02 (integrated qualification Task 3 fix round 1): Bound PlanningScene
  evidence semantics.  `validation/planning_scene_journal.py` closes the
  public-path false passes: `record_diff` now requires a genuine world-to-
  attached / attached-to-world transition for the exact target at
  `scene-attach` / `scene-detach` (shared `_validate_transition` also used by
  `assert_transition`), `snapshot` rejects `scene-attach` / `scene-detach` /
  `task-cleanup`, and every diff gates object removal (only `task-cleanup` may
  remove `pick_and_place/*`; `teardown` alone may additionally remove
  `sim_fixture/*`).  `finalize` rejects an empty journal and, with a declared
  required order, requires exact event-list equality (extra/duplicate/
  out-of-order/spurious-teardown fail).  Returned records and final
  records/graph are deep-copied so caller mutation cannot diverge JSONL from
  final JSON, and a pre-existing non-empty JSONL path fails closed before the
  first append.  `validate_graph_evidence` requires the exact topic/service key
  sets, the recorder node among topic subscribers and service clients, exact QoS
  key sets with a non-boolean integer `depth`, and exact (never string-coerced)
  fixture payload field types with ordered unique `sim_fixture/*` owned ids,
  retaining the exact canonical compact payload string as evidence.  Nested
  scene types are strict (`ValueError`, no coercion), and `_append` independently
  re-validates the full scene.  `tests/test_planning_scene_journal.py` grew to
  181 tests.

- 2026-08-02 (integrated qualification Task 3): Added the stable PlanningScene
  journal for the integrated OMPL qualification.
  `validation/planning_scene_journal.py` is the ROS-free pure record/transition
  journal consumed by the later executor/verifier/evidence tasks.  It writes an
  append-only canonical compact `planning-scene.jsonl` (flush + fsync before the
  in-memory record becomes visible) and an atomic canonical final
  `planning-scene.json` (temp-file + file fsync + `os.replace` + directory
  fsync, no temp residue; a failed finalize never replaces an existing
  artifact).  `load_model_touch_contract` loads the attach/TCP link `link_tcp`,
  the ordered eight-link SRDF touch set, and handoff
  `pick_and_place/object_mesh` verbatim from
  `integration/ompl-overlay-contract.json`, failing closed on
  missing/malformed/non-eight/duplicate/permuted contract values.
  `validate_graph_evidence` validates the Task-4-supplied graph projection:
  recorder identity `node_name="/tinker_integrated_gate_executor"`,
  `namespace="/"`, `remap_table={}`, the `/planning_scene` and
  `/monitored_planning_scene` topics (`moveit_msgs/msg/PlanningScene`, reliable +
  transient-local, depth 1), `/get_planning_scene` and `/apply_planning_scene`
  services (`moveit_msgs/srv/GetPlanningScene` /
  `moveit_msgs/srv/ApplyPlanningScene`, reliable + volatile), real
  endpoint/provider metadata, and the `/sim/status/planning_scene_fixture`
  topic (`std_msgs/msg/String`, exactly one `/fixture_planning_scene` publisher,
  reliable + transient-local, depth 1) whose payload is independently validated
  as the exact canonical compact fixture-status JSON with scalar
  `target_handoff="pick_and_place/object_mesh"`; payload content never
  substitutes for graph ownership.  Records carry distinct journal
  (`journal_sequence`), raw/evaluator join (`frame_index`, `timestamp`), and
  diagnostic scene (`scene_sequence`, `scene_timestamp`,
  `scene_revision_digest`) identities; every digest matches
  `^(?!0{64}$)[0-9a-f]{64}$`.  Scene state remains diagnostic consistency
  evidence, never physical authority — no contact/force/object-pose/
  evaluator/physical-verdict fields are stored.  Positive event order is
  `fixture-ready → before-pick → scene-attach → lift-complete → transport →
  before-release → scene-detach → released-settled → teardown`; negative
  scenarios finalize a shorter required prefix, and duplicate/missing/
  out-of-order/forbidden events fail.  `tests/test_planning_scene_journal.py`
  provides 137 focused tests (brief cases plus adversarial coverage for each
  digest field, `_last_scene` rollback, bool/negative/nonfinite numerics,
  duplicate IDs, recursive physics-key leakage, model-contract drift, graph
  type/QoS/source/cardinality/remap/payload mismatch, JSONL/final durability
  and no temp residue, positive/negative order, duplicate/forbidden events, and
  attach/detach/cleanup ownership).

- 2026-08-02 (integrated qualification Task 2 fix round 3): Closed the residual
  semantic false-passes.  `validation/integrated_static_contracts.py`
  `_bind_run_post_close_pick` aggregates every SimOmpl/Hardware profile block
  and validates the whole set (exactly one obstruction guard, one SimOmpl lift
  block, one Hardware lift block; per-block polarity; offset-based structural
  containment of every `execute_lift(...)` call by an accepted profile lift
  block), so a decoy block after a violating one can no longer pass.  Result
  builders are checked by the exact assignment form
  `\bresult\s*->\s*<field>\s*=(?!=)` over sanitized executable code; a
  differently-named member, a read, or a comparison never satisfies the write
  contract.  `_bind_bundle_artifact` binds a non-simulator artifact by its
  semantic source key plus exact path identity (bundle absolute path ==
  `<repo_path>/<path_relative>`, overlay workspace path ==
  `src/<repo dir>/<path_relative>`, recorded digest == verified source record);
  simulator-local acceptance is restricted to the declared overlay path under
  `outputs/` or `artifacts/`, and a shadow file with matching bytes at an
  undeclared path never reclassifies.  The Task 2 report records only
  independently reproduced test evidence.

- 2026-08-02 (integrated qualification Task 2 fix round 2): Strengthened the
  semantic source checks.  `validation/integrated_static_contracts.py` replaces
  the comment-stripped string-presence C++ scans with a deliberately scoped
  lexical/structural layer: literals (string/char/raw-string) and comments are
  sanitized while braces/newlines are preserved, conditional-preprocessor
  directives inside each load-bearing function body are rejected (dead-code
  anchors cannot satisfy a check), and each inspected function is located by
  its actual signature and brace-matched.  `run_post_close_pick` binds
  `execute_lift(ctx, true/false, ...)` to its own Sim/Hardware `ExecutionProfile`
  branch; `GraspNode::~GraspNode` binds the bounded deadline,
  `motion_runtime_.shutdown`, the `executor_thread_` join and the
  state-validity client reset to the destructor body; `coordinator_main` binds
  the joined worker; `GraspNode::move_straight` binds
  `request.avoid_collisions = avoid_collisions`; and each task result builder
  must write exactly its `.action` result schema fields.  The overlay
  `model_bundle.production_source_commits` is content-verified against
  immutable Git blobs (commit existence, blob digest, manipulation ancestry vs
  `implementation_head`; external `tk25_basic` via exact recorded commit/blob
  only) and drives the model-artifact binding and the pinned-prerequisite
  source-identity check, so working-tree drift never affects an immutable
  `git show` result.  `fixture-ownership` requires the overlay `scenarios` set
  to equal the configured C/D/E set exactly (missing or extra fails).
  `validation/source_lock_manifest.py` closes the observer gaps:
  `qualification_tooling` requires clean mode with empty
  status/diff/index/untracked evidence, `authorization.report_path` must
  resolve to a regular file inside the repository (absolute/symlink escape
  fails), `_normalize_posix` rejects `..` and canonicalizes absolute
  repository-relative paths, the stale mtime-fallback docstring is removed,
  and the output manifest is written/fsynced/replaced exactly once from a
  single pre-write timestamp (no two-valid-file crash window).

- 2026-08-02 (integrated qualification Task 2 fix round 1): Bound Gate B to
  real artifacts.  `validation/integrated_static_contracts.py` dropped the
  fabricated `qualification/records/*` layer and inspects every production
  source as an immutable Git blob at the production `implementation_head`
  from the produced source-lock manifest: SRDF `_xarm7_macro.srdf.xacro`
  (ordered `xarm_gripper` members, `link_tcp` end-effector parent),
  `config/xarm7` and `config/xarm_gripper` `controllers.yaml`, the pick_and_place
  C++ runtime (`motion_runtime_.shutdown`, `executor_thread_.join`, no
  `.detach()`, result-field writes against the real `.action` schemas),
  `clean_planning_scene` SimOmpl early return with task-owned hardware cleanup,
  collision-aware `execute_lift(ctx, true, ...)`, and
  `request.avoid_collisions = avoid_collisions` forwarding.  The model bundle
  is recomputed (`structural_fingerprint`, artifact SHA-256,
  `production_source_commits`), `fixture-ownership` inspects exactly the 17
  configured C/D/E scenarios deriving owned ids from `planning_scene.objects`,
  and transport-contract reads the sibling `typed_contract.runtime_contract_sha256`
  (never nested in the public report) and recomputes the full runtime mapping.
  `source_lock_manifest.py` adds closed schemas, lock-commit scope
  (`{policy_path, docs/acceptance.md}`), qualification HEAD exact match,
  authorization report/commit checks, canonical repository-relative policy
  paths, required ISO `started_at`, and a stable output-freshness boolean.
  The static checker emits atomic fsync+`os.replace` evidence
  (`static-contract.json`, `model-fingerprint.json`, `source-identities.json`)
  and propagates `evidence-invalid` / `verified-fail` / `verified-pass` per F4.
  Real-root post-fix: the observer is `evidence-invalid` only because the
  future qualification-tooling policy is intentionally absent; eight of the
  nine static checks pass against the real simulator/production roots and only
  `source-identities` fails on that absent policy.

- 2026-08-02 (integrated qualification Task 2): Added the offline Gate B
  static-contract closure.  `validation/source_lock_manifest.py` is the
  canonical three-policy source-lock observer
  (`simulator_overlay` / `production` / `qualification_tooling`) with
  non-self-referential Git-history resolution (the containing/transition commit
  is resolved from history, never from `HEAD`/`HEAD^` or an in-file
  `policy_commit`), exact `LC_ALL=C` status/diff/index/untracked evidence,
  canonical path-sorted untracked manifest, atomic fsync+`os.replace` output,
  and attempt freshness.  `validation/integrated_static_contracts.py` runs the
  nine semantic static checks (model fingerprint, controller mapping, selected
  launch, provider cardinality, fixture ownership, action lifecycle,
  scene/collision safety, source identities, transport contract) against the
  produced three-entry manifest.  `simulation/qualification/integrated-ompl.json`
  now names exactly the three `source_lock_policies`.  The real
  qualification-tooling policy is intentionally absent during Task 2, so a
  real-root Gate B capture returns `evidence-invalid` until the post-Task-10
  lock-only phase; no existing source-lock policy was created or modified.
