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
  `92e3aad9b4d45c0583c32fac17fae1c4f5aec432d224c790d7ccd96482a9afe9` and the
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

## Known issues

- 2026-08-09 (Stage D qualification-moveit-cancel): the cancel scenario
  (`qualification-moveit-cancel`) does not pass live.  The `run_cancel_sequence`
  arbitration intermittently takes ~9.4 s (goal-time abort at ~9.39 s wins the
  race by ~20 ms, `cancel_response` return_code 3).  The safety scenario runs
  the identical presend + FJT-executing/motion-trigger waits in ~1.8 s, so the
  stall is intermittent.  **Deferred by decision: cancellation semantics are
  not required for the current OMPL-goal.**  The scenario remains in the suite
  and is tracked as a known issue, not a Stage-D blocking gate for the
  OMPL/cuMotion integration goal.

## Integrated OMPL qualification CLI (Task 10)

`validation/integrated_qualification.py` orchestrates the integrated OMPL
qualification Gates A-F.  It is an operator workflow and graph contract, not
evidence of a live manipulation pass or release qualification.  Task 10's own
tests and this documentation make no live Gate F/OMPL/cuMotion claim.

Use one consistent suite path variable:

```bash
SUITE_DIR=outputs/integrated/integrated-ompl-seed-7
```

`SUITE_DIR` is passed as the runner's `--attempt-root`.  It is the exact
integrated suite directory; nothing is silently appended below it.  A repeated
qualification run must choose a fresh `--attempt-root`; never merge a new run
into an old suite.  The six-gate Stage-A core suite runs in the sibling root
`<SUITE_DIR>-core/` (here `outputs/integrated/integrated-ompl-seed-7-core/`),
outside the integrated Gate-F index.

Gate A (six-gate core suite):

```bash
./.venv/bin/python validation/integrated_qualification.py \
  --attempt-root "$SUITE_DIR" --stage A
```

Gate B (offline static closure; `--offline` is an explicit compatibility flag
for the already-offline B implementation and must not make any live stage
offline or bypass checks):

```bash
./.venv/bin/python validation/integrated_qualification.py \
  --attempt-root "$SUITE_DIR" --stage B --offline
```

Gate C (three OMPL plan-only scenarios):

```bash
./.venv/bin/python validation/integrated_qualification.py \
  --attempt-root "$SUITE_DIR" --stage C
```

Gate D (six execute scenarios):

```bash
./.venv/bin/python validation/integrated_qualification.py \
  --attempt-root "$SUITE_DIR" --stage D
```

Gate E (eight pick-place scenarios):

```bash
./.venv/bin/python validation/integrated_qualification.py \
  --attempt-root "$SUITE_DIR" --stage E
```

Gate F (standalone offline rebuild/verify):

```bash
./.venv/bin/python validation/integrated_qualification.py \
  --attempt-root "$SUITE_DIR" --stage F
```

All stages (`--stage all`):

```bash
./.venv/bin/python validation/integrated_qualification.py \
  --attempt-root "$SUITE_DIR" --stage all
```

The standalone Gates A through F above and `--stage all` are alternatives.
`--stage all` must use a fresh suite path and must not be run against a suite
that already has write-once A-E stage records.

Independent verifier replay against the selected immutable C/D/E attempt
directory.  Every C/D/E scenario runs in a newly created immutable attempt
directory named `STAGE-<scenario>-<invocation>-<counter>`.  The selection
below finds the exact single immutable matching attempt under `$SUITE_DIR`,
fails unless exactly one match is found, and binds it to `ATTEMPT_DIR`:

```bash
ATTEMPT_DIR="$(find "$SUITE_DIR" -maxdepth 1 -type d \
  -name 'C-qualification-moveit-plan-joint-*' | sort)"
test "$(printf '%s\n' "$ATTEMPT_DIR" | sed '/^$/d' | wc -l)" -eq 1
./.venv/bin/python validation/integrated_gate_verifier.py \
  --scenario qualification-moveit-plan-joint \
  --attempt-dir "$ATTEMPT_DIR" \
  --config simulation/qualification/integrated-ompl.json
```

Integrated contact-sheet regeneration against `$SUITE_DIR`:

```bash
./.venv/bin/python validation/integrated_contact_sheets.py --suite-dir "$SUITE_DIR"
```

Standalone evidence-index rebuild and Gate-F validation (writes
`evidence-index.json` and `qualification-summary.json`):

```bash
./.venv/bin/python validation/integrated_evidence_index.py \
  --suite-dir "$SUITE_DIR" --summary "$SUITE_DIR/qualification-summary.json" \
  --validate
```

Failed-attempt retention/rerun rule: never delete or reuse a failed, stale, or
old attempt or suite.  Every C/D/E scenario runs in a freshly created immutable
attempt directory; repeated allocation yields distinct preserved paths.  To
rerun, choose a fresh suite path (a fresh `--attempt-root`); never merge a new
run into an old suite.

Bounded build command (never raw colcon).  The wrapper ignores CLI args and
internally executes colcon with `--parallel-workers 2`:

```bash
MAKEFLAGS='-j2 -l2' ./scripts/build-humble-overlay
```

Truthful scope:

- The runtime config has three source-lock roles
  (`simulator_overlay` / `production` / `qualification_tooling`).  The
  qualification-tooling source-lock role is created only after Task 10 is
  review-clean, in a separate lock-only commit, and only before live attempts.
- No live Gate F/OMPL/cuMotion claim comes from Task 10's offline tests.
- `_image_stats` thresholds still require live RTX calibration.
- cuMotion remains prohibited until Task 37's live OMPL qualification passes.

## Changelog

- 2026-08-09: Recorded the Stage-D `qualification-moveit-cancel` scenario as a
  deferred known issue (intermittent ~9.4 s cancel arbitration vs ~9.39 s
  goal-time abort).  Cancellation semantics are out of scope for the current
  OMPL-goal; the scenario remains tracked, not blocking.

- 2026-08-05 (integrated qualification Task 10 — "document integrated
  qualification CLI"): Documented the offline integrated OMPL qualification
  CLI with one consistent `SUITE_DIR` variable and exact standalone command
  blocks for Gates A-F (Gate B with the explicit `--offline` compatibility
  flag), `--stage all`, the deterministic single-match verifier replay
  (`ATTEMPT_DIR` selection that fails unless exactly one immutable matching
  attempt exists), the integrated contact-sheet regeneration, and the bounded
  `MAKEFLAGS='-j2 -l2' ./scripts/build-humble-overlay` build command (the
  wrapper ignores CLI args and internally executes colcon with
  `--parallel-workers 2`).  Standalone Gates A-F and `--stage all` are
  documented as alternatives; `--stage all` must use a fresh suite path and
  must not be run against a suite that already has write-once A-E stage
  records.  The evidence-index command writes `qualification-summary.json` via
  `--summary`.  The documentation states the fresh-suite retention rule (never
  delete/reuse; choose a fresh `--attempt-root`), the three source-lock roles
  with the qualification-tooling role created only after review-clean Task 10,
  and the live-only caveats (no live Gate F/OMPL/cuMotion claim from Task 10
  offline tests, `_image_stats` still requires live RTX calibration, and
  cuMotion remains prohibited until Task 37 live OMPL passes).  No build, no
  live Isaac/ROS/GPU/cuMotion, and no source-lock file changed; the future
  qualification-tooling source-lock role remains absent until Task 36.

- 2026-08-04 (integrated qualification Task 9, fix round 5 — "final narrow
  production-suite closure"): Closed the last offline production-suite residuals.
  `validation/integrated_contact_sheets.py::_all_bound_capture_entries` now
  tolerates shared cancel/safety event labels across multiple scenarios/attempts
  and selects exactly one deterministic representative capture per event by
  canonical event order then scenario/attempt/preferred-camera/path (F5.1).
  `validation/integrated_evidence_index.py` scopes the duplicate `(event, camera)`
  keyframe-identity check per attempt directory, requires `capture_latency_frames`
  and both `execution_event_sequence` / `source_execution_event_sequence` as
  mandatory equal positive integers with missing either side a critical
  diagnostic (retaining the real `raw = requested + latency` relation), and
  requires exactly one categorized overlay-contract artifact (F5.2/F5.3).
  `simulation/tinker_sim_isaac/qualification_visual_capture.py` expires a
  partially captured sequence that can no longer satisfy the bounded latency
  contract on a restarted consumer (one deduplicated terminal error, durable
  terminal marker, preserved camera-1 evidence, no camera-2 fabrication, no
  retry/error growth) and persists each PNG atomically and durably before its
  keyframe row (F5.4).  This is offline production-suite closure only; no live
  Isaac/camera/rosbag/GPU/OMPL/cuMotion claim; Task 10 still owns the Gate-F
  wiring and a load-bearing live rosbag; `_image_stats` still requires live RTX
  calibration.

- 2026-08-04 (integrated qualification Task 9, fix round 4 — "align evidence with
  real capture artifacts"): Aligned the offline Gate-F evidence contract with the
  real producer output.  `validation/integrated_evidence_index.py` validates the
  real capture-latency arithmetic (`requested_physics_frame_index` == the
  producer's rounded-frame calculation; `capture_latency_frames` integer in
  `[0, MAX]` equal to `raw_frame_index - requested_physics_frame_index`; primary
  key and timestamp tolerance retained at the captured frame), parses the real
  Humble nine-field rosbag2 QoS profile and matches the recorder override as a
  required-field subset, recognizes both `overlay-contract.json` and the real
  `ompl-overlay-contract.json` (any `*-overlay-contract.json`) as one
  authoritative overlay-contract identity (contradictory duplicates fail), and
  resolves a verbatim root-relative `source_locks.simulator_lock_path` against
  the evidence suite without escaping it or silently accepting an absent lock.
  `validation/integrated_contact_sheets.py` orders production CLI bound captures
  by the canonical required suite event sequence and rejects unknown/duplicate
  event identities.  Visual completeness keys by exact `(scenario_id,
  attempt_id)`, a nonempty valid GPU inventory is required when
  baseline/final `available=true`, and `qualification_visual_capture.py` is
  restart-safe across a partial two-camera capture (per-`(request_sequence,
  camera)` durable completion).  Offline production-shaped latency/QoS/CLI/
  overlay/restart tests pass; `_image_stats` thresholds still require live RTX
  calibration, Task 10 must wire Gate F and launch/finalize a load-bearing live
  rosbag, and no live Isaac/RTX/camera/rosbag/GPU run occurred in this repair.
  The future qualification tooling lock remains absent until after review-clean
  Task 10.

- 2026-08-04 (integrated qualification Task 9, fix round 3 — "make integrated
  evidence production-real"): Made the integrated evidence pipeline accept
  genuine live artifacts.  The evidence index (`validation/integrated_evidence_index.py`)
  now parses the real nonempty RTX GPU inventory in `resource-cleanup.json` as a
  physical inventory (F3.1), accepts the real nested
  `repositories.production`/.simulator + scalar `repositories.path_scope`
  overlay-contract shape (F3.2), requires required visual events per exact
  attempt/scenario with the complete suite event order embedded in contact
  sheets (F3.3), cross-binds keyframe request-time/source-sequence against the
  canonical request (F3.5), closes manifest/config/model/source/attempt/verdict
  identity binding with a missing-manifest failure (F3.6), rejects contact-sheet
  output equal to any indexed evidence artifact (F3.7), and parses real rosbag
  QoS YAML plus requires every metadata-listed storage file to exist and be
  nonempty (F3.8).  The executor (`validation/integrated_gate_executor.py`)
  routes every non-`"recorded"` `_append_visual_event` producer status to
  `evidence-invalid` through the fail-dominant D/E finalization paths (F3.9);
  `manipulation_qualification.py::_env` rejects noncanonical integrated scenario
  ids before launch (F3.10); `qualification_visual_capture.py` seeds
  at-most-once from durable keyframes (F3.4).  New tests drive the real runner
  env, real executor producer, real capture consumer, and a true integrated
  producer path (executor → consumer → index/sheets/summary → Gate F
  `verified-pass`) plus a diagnostic-only journal fail-closed test.  `_image_stats`
  thresholds still require live RTX calibration; Task 10 must wire Gate F and
  launch/finalize a load-bearing integrated rosbag; no live Isaac/camera/rosbag/
  GPU run occurred in this repair; the future qualification tooling lock remains
  absent until after review-clean Task 10.

- 2026-08-04 (integrated qualification Task 9, fix round 2 — "produce integrated
  visual evidence"): Produced the canonical visual-capture evidence end-to-end
  and closed the validator's semantic gaps.  The integrated executor
  (`validation/integrated_gate_executor.py`) now appends canonical
  sequence-shaped EventJournal capture requests at every required positive,
  cancel, and safety checkpoint (`_append_visual_event`), strictly after the
  durable planning-scene/journal checkpoint it binds, with a per-attempt
  sequence reset and duplicate-event rejection; the executor diagnostic records
  (`_append_visual_request`) remain a separate non-capture-driving shape.
  `manipulation_qualification.py::_env` enables the capture producer for
  integrated Isaac children only (exact canonical scenario id as
  `TINKER_SIM_QUALIFICATION_GATE`, `TINKER_SIM_VISUAL_EVIDENCE=1`, fail-closed
  on missing scenario id).  `qualification_visual_capture.py` co-tenants the two
  request shapes safely: executor diagnostics are skipped silently, malformed
  records are reported exactly once, and capture freshness stays within the
  bounded `MAX_CAPTURE_LATENCY_FRAMES=4` contract.  The evidence index
  (`validation/integrated_evidence_index.py`) binds captures only through the
  canonical request↔keyframe transaction and requires exactly one raw and one
  evaluator row per `(scenario_id, frame_index)` key within
  `max(1e-6, 0.5*physics_dt)`; any index diagnostic fails Gate F closed.
  F2.5-F2.9 close source/provenance, verdict, MoveIt/controller status domain,
  per-attempt PlanningScene, exact 11-topic rosbag, cleanup-recompute, and
  contact-sheet output-as-input semantics.  New tests: evidence-index mutations
  (16) + contact-sheet mutations (3) = 19 added (54 + 23 total in the two
  suites); affected ROS-free regressions pass; Humble-sourced
  `test_integrated_gate_executor_ros.py` passes.  No build, no live Isaac/ROS/
  GPU/cuMotion, and no rosbag/capture production ran; the future qualification
  source lock remains absent.

- 2026-08-04 (integrated qualification Task 9, fix round 1 — "bind evidence index
  to live artifacts"): The initial 44 tests were synthetic and did not exercise
  real Task 2-8 producer schemas, so this repair re-pinned every semantic parser
  to the actual producer contracts.  The evidence index now consumes the real
  executor `visual-capture-requests.jsonl` + capture-process
  `visual-keyframes.jsonl` two-journal transaction and binds each
  `visual/source/*.png` keyframe to exactly one request (via
  `request_sequence`/phase/event) and to an exact
  `physics_truth.jsonl`/`evaluator.jsonl` frame.  Scenario kinds come only from
  the canonical `integrated.acceptance.polarity`/`expected_negative` contract
  (never `integrated.kind`); canonical cancel/safety ids require their event
  groups.  `validate_gate_f` is semantic: source-lock/static-contract/model-
  fingerprint identities, per-scenario verified-pass gate verdicts, nonempty
  raw/evaluator/drain exactness, finalized MoveIt/controller/planning-scene
  journals, rosbag2 metadata + storage counts, cleanup/process/GPU leak checks,
  and contact-sheet PNG/metadata/parity.  F1.5 index integrity recomputes the
  canonical checksum, re-hashes current bytes, and compares the preserved-file
  set; the summary binds a pre-summary projection checksum (never a
  cryptographic cycle).  F1.6 embeds deterministic PNG text chunk metadata
  (role/ordered events/source capture records/reviewed state).  New tests:
  `tests/test_integrated_evidence_index.py` (38) +
  `tests/test_integrated_contact_sheets.py` (20) = 58 passed in the simulator
  venv, mutation-driven against production-shaped artifact bytes; affected
  ROS-free regressions pass (see the task-9-report).  Documentation/staging per
  `execution-corrections-2026-08-02.md` §6.  No build, no live Isaac/ROS/GPU/
  cuMotion, and no rosbag/capture production ran; immutables and the future
  qualification source lock remain untouched.

- 2026-08-04 (integrated qualification Task 9 — "close integrated evidence
  artifacts"): Added deterministic reproducibility indexing and integrated
  contact sheets.  `validation/integrated_evidence_index.py` (new) writes
  `evidence-index.json` covering config/scenario/overlay/model fingerprint,
  simulator and production HEAD/status/diff, dependency/source locks,
  commands/argv/environment allowlist, ROS domain/RMW/DDS, MoveIt
  plans/controller results/planning-scene journal, raw truth/evaluator/drain,
  rosbag metadata/QoS/counts, verdicts/cleanup/GPU/process reports, and contact
  sheets; it excludes only itself and repeated builds over unchanged bytes are
  identical.  `validate_gate_f` is fail-closed (missing commit identity, rosbag
  metadata/QoS/counts, planning-scene journal, required artifacts, unbound or
  missing required-event captures, and absent sheets) and never fabricates a
  verdict.  `validation/integrated_contact_sheets.py` (new) renders deterministic
  `contact-sheet-integrated-agent.png` / `contact-sheet-integrated-user.png`
  authorized only by captures already carrying exact path+digest+event/frame
  metadata in the evidence index; every visual event binds to exact
  scenario/attempt/execution-request plus `(frame_index,timestamp)`.  Agent and
  user sheets agree on the covered event set; screenshots and PlanningScene
  remain diagnostic only, never physical pass authority.  New tests: 28
  evidence-index + 16 contact-sheet = 44 passed; affected ROS-free regression
  369 passed.  Task 10 wires Gate F into the orchestrator.  No build, no live
  Isaac/GPU/cuMotion; immutables and the future qualification source lock
  untouched.

- 2026-08-04 (integrated qualification Task 8, fix round 5 — "stabilize controller
  evidence lifecycle"): Final repair round driven by the fix-round-4 coordinator
  result (positive D trio 11/17 fresh-process runs, one cancel transaction
  committed `evidence-invalid`, one interpreter-teardown crash, and five harness
  5 s readiness-budget failures).  Each required FJT state is one immutable
  captured status entry: `_wait_for_fjt_status`/`_wait_for_fjt_executing` capture
  and return a copy, `run_execute_sequence`/`run_cancel_sequence`/`run_safety_sequence`
  retain that entry and bind the provider through `_bind_and_call_fjt_provider`,
  and `_validate_fjt_evidence` validates only against the captured entry (never a
  fresh cache query); the provider returns UUID/status/sequence/timestamp/source
  from that exact entry and the exact recorded ExecuteTrajectory digest, and any
  post-capture second status for the same UUID cannot switch the transaction.
  Defaults are now 10.0 s for both `fjt_wait_timeout_s` and
  `motion_trigger_timeout_s` with fail-closed malformed overrides.  The presend
  goal uses a caller-generated action UUID and the driver owns an exact-UUID
  cancel path through the acceptance-response timeout (late-handle retention or a
  typed `CancelGoal` on `/execute_trajectory/_action/cancel_goal` with terminal
  evidence), so no accepted long-motion goal can be stranded by a delayed
  acceptance response.  The controlled graph teardown drains all goals/coroutines/
  futures/threads/nodes/contexts in bounded explicit order and the readiness
  budget is 30.0 s; a stale-but-valid join key waits a bounded 0.1 s for the next
  advancing physics-truth frame before failing closed (closes the round-4
  fresh-process `evidence-invalid` at the teardown journal snapshot).  Fresh
  counts: provider suite 38 passed / zero warnings in
  three consecutive fresh processes; each positive D execute/cancel/safety test
  4/4 alone; the three positive D tests together 30/30 consecutive fresh
  processes, zero warnings/crashes/timeouts; plus fresh delayed-status,
  second-post-capture, delayed-acceptance exact-cancel, cleanup-rejection/
  unavailable, and owner-QoS mutation tests.  No build, no live Isaac/GPU/cuMotion;
  all immutable production files and the future qualification source lock
  untouched.

- 2026-08-04 (integrated qualification Task 8, fix round 4 — "bind real
  controller transactions"): Bound every Stage-D controller transaction to the
  real controller FJT goal UUID — never the MoveIt ExecuteTrajectory UUID.
  Real Humble `UUID` message containers (numpy `uint8[16]`) are normalized by a
  strict 16-byte `_normalize_goal_uuid`; `_d_baseline()` records the known FJT
  goal UUID set and `_discover_new_fjt_goal()` requires exactly one distinct new
  controller UUID in the execution window (no-new and multiple-new fail closed,
  pre-baseline replays rejected); `_validate_fjt_evidence()` binds the provider
  to the discovered controller UUID and joins its status/sequence/timestamp to
  the exact fresh FJT status-topic entry.  The cancel/safety pre-send carries the
  real controller identity and the pre-send baseline, `run_driver` teardown
  guarantees presend cleanup plus an operator clear, and the journal graph
  observation selects owner-specific QoS.  `build_occupancy_from_planning_scene`
  rasterizes the oriented yaw-only box footprint for all 17 canonical scenarios.
  Offline regressions: executor 127 / driver 37 / ROS 165 passed; sourced-Humble
  provider suite 30 passed, zero warnings — the controlled C/D positive paths
  commit `diagnostic-pass` with a controller UUID distinct from the
  ExecuteTrajectory UUID, the D cancel/safety paths reach the real
  presend-provider sequence, and the transaction-real negatives fail closed.  No
  build, no live Isaac/GPU/cuMotion; all immutable production files and the
  future qualification source lock untouched.

- 2026-08-04 (integrated qualification Task 8, fix round 3 — "observe live
  integrated providers"): Made every executor provider a real live observation
  and adopted Option A+ for the qualification development LiDAR.  The driver
  now owns a private-context `_LiveProviderObserver` node registered on the
  executor's spinner; because rclpy delivers service responses to the spinner's
  wait set (not to the calling client's node), all controller/graph parameter
  and list-controllers queries go through `_call_service_with_spinner` polling
  an async future instead of the blocking `client.call()` that hangs on a shared
  private context.  Readiness, TF TCP pose, environment PointCloud2, native
  gripper goal count, FJT transaction digest, and graph introspection all derive
  from observed subscriptions/transforms; the driver never references
  `_observed_graph`/`_tf_lookup`/`_latest_environment_cloud`/
  `_native_gripper_goal_count`/`ParameterClient`/`server_is_available`.
  Long-motion goal UUIDs are normalized driver-side (`_goal_id_hex`) because the
  executor's `_normalize_goal_uuid` rejects real rclpy `UUID` messages; the
  operator baseline is re-published and age-refreshed inside the readiness
  snapshot; and `/pick_and_place.post_grasp_lift_m` is served by
  `declare_parameter` + `add_on_set_parameters_callback` (a manual service
  conflicts with rclpy's auto-created `/get_parameters`).  Option A+: committed
  scenario PlanningScene box footprints populate a pure deterministic occupancy
  map (`build_occupancy_from_planning_scene`, 0.05 m / 60 m half-extent) served
  as backend occupancy only under `manipulation-core --qualification`; the
  existing `/livox/lidar` stream is enabled only for `navigation-parity` or
  qualification; the integrated launch owns the exact `base_link -> livox360`
  static transform (`tf2_ros/static_transform_publisher` named
  `livox360_static_tf`, xyz 0.12/0.0/0.25, identity quaternion, matching
  navigation.launch.py), spelled `launch_ros.actions.Node` so the immutable
  Task-2 launch-graph allow-list still accepts the overlay.  Raw-verifier
  authority is unchanged; all provider success data is derived from real ROS
  traffic, never PlanningScene geometry.  Offline regressions: driver ROS-free
  suite grows to 36 tests, broad qualification batch 1220 passed + 2 skipped +
  9 subtests (sole pre-existing `uv` executable-hash provenance failure
  unchanged), launch contract 7 passed, and sourced-Humble
  `tests/ros_humble/` 54 passed + 1 skipped (22 new provider tests: live
  readiness, controllers, FJT digest, native-gripper, parameter set/read-back,
  negative-mutation fail-closed, and cancel-presend paths).  No build, no live
  Isaac/ROS, no GPU-process change, no cuMotion; executor/verifier/journal/
  scenarios/configs/locks/`ros_gateway.py` and all production files untouched.

- 2026-08-04 (integrated qualification Task 8, fix round 2 — "wire executor
  evidence producer"): Wired the live Humble executor evidence producer and
  sealed finalization.  The integrated C-E lifecycle is now three-child: Isaac +
  Humble overlay launch first; after canonical PHYSICS_READY the orchestrator
  atomically writes the already-validated `scenario-bundle.json` (scenario id/
  seed/declaration, planning-scene declaration + mapping, integrated mapping,
  report identities, current attempt id + resolved path) and launches the new
  source-run `validation/integrated_gate_executor_driver.py` as a third owned
  child (`qualification_start_process(runner, "executor", ...)`) under the same
  ROS domain, attempt dir, and ros-tooling environment
  (`/usr/bin/python3`, RMW/DDS/profile, Humble overlay
  PYTHONPATH/AMENT_PREFIX_PATH/LD_LIBRARY_PATH).  The driver loads the bundle
  unchanged, derives dispatch for exactly the 17 canonical scenario ids,
  constructs the real `IntegratedGateExecutor` with live readiness/join-key/
  graph providers, dispatches one run method, and atomically writes
  `execution-terminal.json` (cross-bound to scenario id + attempt id + attempt
  path, `marker: executor-driver`) only after executor artifact finalization
  (`integrated-execution.json` must exist); driver failures write a durable
  fail-closed terminal and exit nonzero.  The orchestrator waits within a
  config-derived terminal budget (`plan + 2*execute + cancel + scene +
  max(cancel,30)` = exactly 305.0 s for the committed thresholds; separate from
  the 30 s readiness budget) and fails immediately if the executor exits
  without a current-attempt marker; a wrong-identity or stale marker is
  rejected.  Every E transport scenario sets and reads back
  `/pick_and_place.post_grasp_lift_m = 0.10` on the live task server and feeds
  the observed read-back as the typed provider (fails closed otherwise).
  `_finalize_attempt` isolates every cleanup phase (executor stop → Isaac stop →
  exact raw/evaluator drain → Humble stop → rosbag → orphan → resource →
  settle) so an exception in one phase is recorded and all later phases still
  run; a whole-helper escape is guarded at both call sites and becomes durable
  per-scenario `evidence-invalid`, never an abort.  Integrated C-E rosbag stays
  a truthful non-load-bearing diagnostic (`not-recorded`; present valid bag
  validated, corrupt bag fails closed); Task 9/Gate F must add/index the
  intended recorder.  Offline regressions: 25 new driver tests + 55
  integrated-orchestration tests, 511 passed in the broader batch, and the
  sourced-Humble real executor/journal surface (164 + 184 passed) proving the
  executor finalizes `integrated-execution.json`, `moveit-plans.jsonl`,
  `controller-results.jsonl`, `planning-scene.jsonl`, and `planning-scene.json`.
  No build, no live Isaac/ROS, no GPU-process change, no cuMotion;
  executor/verifier/journal/overlay-launch/scenarios/configs/locks and all
  production files are untouched.

- 2026-08-04 (integrated qualification Task 8, fix round 1 — "execute
  integrated scenario lifecycle"): Made the integrated C-E lifecycle executable
  on the real manifest/launch path.  The orchestrator no longer passes a
  scenario id as a core `gate` (which crashed the core `_selected_gates`); it
  builds the manifest at the externally allocated attempt directory with
  `gate="integrated"` via the additive
  `QualificationRunner.prepare_manifest_at` and starts both the Isaac and Humble
  child wrappers through the real process lifecycle with the exact scenario id,
  seed, private domain, and `TINKER_SIM_ATTEMPT_DIR`/RMW/DDS environment applied
  to the subprocesses.  Each scenario runs in a newly created immutable attempt
  directory that did not previously exist (repeated allocation yields distinct
  preserved paths); a nonempty attempt directory is rejected, readiness requires
  current-attempt manifest provenance on top of the exact `scenario-runner.json`
  byte binding, and a zero-child-launch attempt can never false-pass on stale
  evidence.  Per scenario the producers are stopped and the evaluator/raw drain
  is required to correlate exactly before the independent
  `verify_integrated_attempt` runs; rosbag, orphan, and resource evidence are
  finalized before verification inside a single fail-dominant `try/finally`
  lifecycle, so every post-launch exception still runs bounded cleanup.  C/D/E
  stages carry a top-level fail-dominant status, standalone pass exits 0,
  `--stage all` always retains Stage A and never exits 0 on Gate-B failure, and
  a not-implemented Stage F reports non-success.  Gate B is per-invocation and
  cross-bound: source-lock `fail`/missing/stale/self-generated artifacts are
  `evidence-invalid`, producer exit 0 without newly written output is rejected,
  and `model-fingerprint.json` must equal the runtime model bundle fingerprint.
  Stage A requires the integrated config's `required_core_gates` to equal the
  core config gate list exactly.  A malformed scenario fails closed without
  skipping later controls.  The Task-7 L-A..L-D predicates remain delegated to
  the independent verifier; L-E cancel-approach quiescence remains an open
  live-only obligation (scenario/config files are immutable this round).  No
  build, no live Isaac/ROS, no cuMotion.

- 2026-08-04 (integrated qualification Task 8 — "orchestrate Gates A-F"):
  Added `validation/integrated_qualification.py`, the offline orchestration
  layer over the review-clean six-gate core suite, the Task 6 physics-ready
  gate, and the Task 7 independent verifier.  `IntegratedRunner` CLI stages
  `A`/`B`/`C`/`D`/`E`/`F`/`all`.  Stage A reuses the unchanged
  `manipulation_qualification.py --gate GATE_NAME` semantics and requires all
  six independent verdicts, exact raw/evaluator drains, valid rosbags, clean
  teardown, and existing contact sheets.  Gate B atomically writes
  `outputs/integrated/attempt-start.json` (UTC/monotonic start identities),
  then invokes the committed `source_lock_manifest.py` with the
  config-resolved authorization policy, validates the producer exit code and
  output schema, then invokes the offline static closure; it is fail-closed
  (missing/stale/self-generated/mismatched source-lock artifacts are
  `evidence-invalid`, never captured/trusted current state) and blocks C-F on
  any non-pass.  Stages C-E run every listed scenario in a unique child ROS
  domain in `[0,232]` with a unique immutable attempt directory.  Readiness
  requires the atomically written `physics-ready.json` to bind
  `scenario_report_sha256` to the exact external `scenario-runner.json` bytes
  and to carry the full committed identity (scenario id/seed,
  scenario_declaration_sha256, planning_scene_sha256, integrated_sha256, model
  fingerprint, provider-manifest digest, final `STATE_PLAYING`, and a final
  `state=1`/`boundary=PHYSICS_READY` operation); a transient
  `state=PHYSICS_READY` without that report-byte match is insufficient.
  Execution return codes never override the independent verifier verdict;
  teardown failures downgrade a scenario to `evidence-invalid`; every attempt
  is preserved.  Stage F is the explicit Tasks 9-10 extension point
  (`not-implemented`).  `validation/manipulation_qualification.py` gained
  additive thin helper exposures (source identity, record topics/QoS, process
  launch/readiness, truth/evaluator drain, rosbag finalization,
  termination/resource cleanup) with the six-gate `--gate` behavior and all
  existing tests unchanged.  `tests/test_integrated_qualification.py` (8
  deterministic orchestration-contract tests plus 7 real-runner offline
  contract tests) passes; focused suite 80 passed + 2 subtests; broader
  qualification regression batch 264 passed.  No build, no live Isaac/ROS, no
  cuMotion; production modules/scenarios/policies/executor/journal/config and
  the two source-lock policy files are untouched.

- 2026-08-04 (integrated qualification Task 7, fix round 2 — "verify terminal
  quiescence"): Verified terminal quiescence at the anchor instead of across the
  braking ramp (F2.1), closing the live-blocking cancel false-invalid on
  D-cancel and E cancel-transport: the verifier now proves rest from a bounded
  two-frame tail ending at `quiescent` (velocity <= `safety_stop_velocity_rad_s`
  + stable command target), so the arm's real deceleration after
  `cancel-requested`/`operator-clear` is allowed, while a ramp that does not
  settle, a new command target/goal in the subwindow, and later journal stages
  fail.  Fixtures carry production-real deceleration ramps.  Hardening: F2.2
  restores forbidden-token scanning over unpaired source/provider strings while
  keeping `env_cloud_evidence.source="observed-environment-cloud"` as semantic
  provenance, adds `goal_kind` to the provider scan, and keeps `pipeline_id`
  exact lowercase `"ompl"` (case variants evidence-invalid).  F2.3 adds a
  consistent non-success safety terminal requirement to D safety.  F2.4 makes
  CLI/API shape failures atomically write durable `evidence-invalid`
  `gate-verdict.json`.  F2.5 restricts raw target identity to the bare
  `qualification_cube`.  No build, no live Isaac/ROS, no cuMotion; production
  modules/scenarios/policies/executor/journal/config untouched.

- 2026-08-04 (integrated qualification Task 7, fix round 1 — "align verifier
  with production evidence"): Aligned the verifier with production
  executor/journal/backend artifact shapes and closed the review blockers and
  majors (F1.1-F1.9).  The pre-start `qualification_cube` requirement is now
  Stage-E-only (C/D production truth carries `objects: []`); the `scene-detach`
  record uses the committed after-state (target detached, matching the
  executor/journal); the endpoint/provider validator is scoped to endpoint
  evidence so `env_cloud_evidence.source` (cloud provenance) is accepted while
  wrong paired providers fail; contradictory terminal domains and malformed
  scalar evidence fail closed as `evidence-invalid` (never a crash or a
  permissive pass); forbidden cuMotion/AnyGrasp/start_grasp taint is enforced
  in provider/goal fields with `pipeline_id` required to be `"ompl"`;
  scenario-owned temporal predicates read only their Table-2 subwindows ending
  at `quiescent`/`released-settled` (post-terminal drain ignored);
  `scene_attached_after_place_failure` proves retained attachment through
  `quiescent`; fixture `goals_sent` is a production-shaped count and D retreat
  carries the real `env_cloud_evidence` shape; the CLI fails closed (exit 2) on
  a scenario filename/id mismatch.  36 tests (13 new), affected regression +
  qualification batch 622 + 2 subtests pass.  No build or live run required;
  production modules/scenarios/policies are unchanged.

- 2026-08-04 (integrated qualification Task 7 — "independent integrated
  raw-physics verifier"): Added the independent verifier
  `validation/integrated_gate_verifier.py` (ROS-free Python 3.12) whose
  physical verdicts derive only from raw physics truth (`physics_truth.jsonl`)
  plus the PlanningScene journal; action/executor/controller/PlanningScene
  results are diagnostic-only.  It implements the full 17-scenario contract
  (3 plan-only, 6 execute/retreat/gripper/cancel/safety, 8 pick-place
  positive/negative): Table 1 per-scenario terminal anchors and Table 2
  observation subwindows (ending at quiescent/released-settled, never at
  teardown), the integrated gate-window wrapper, physics.hz via `core_config`,
  the REQUIRED_ACTIONS ∪ REQUIRED_SERVICES endpoint allowlist, phase-aware
  attachment validation, exact raw/evaluator canonical equality with a distinct
  "raw/evaluator drain mismatch" reason code, the `gate-b-status.json`
  blocked-by-gate-b fail-closed marker, and verdict gate = scenario id with
  stage/polarity separated.  Test fixtures (`tests/integrated_verifier_fixtures.py`)
  build deterministic attempts for all 17 scenarios with fault injection;
  `tests/test_integrated_gate_verifier.py` carries the brief's eight acceptance
  tests verbatim plus adversarial coverage (full-matrix pass, per-class
  verified-fail, terminal-anchor drain exclusion, subwindow termination,
  marker handling, physics-hz, allowlist, verdict-gate identity, strict contact
  threshold, transport direction, lift baseline, attachment phases, obstacle
  exclusion of the grasped target, negative-after-terminal).  23 new tests
  pass; 480 regression tests pass.  No build or live run required; production
  modules/scenarios/policies are unchanged.

- 2026-08-04 (integrated qualification Task 6, formal-review fix round 5 —
  "harden final trajectory digest test"): Closed the last remaining D-side
  digest-padding flake path in the sourced-Humble acceptance suite.
  `test_executor_execute_uuid_mismatch_cleans_up_accepted_handle` is the only
  `run_execute_sequence` caller that reached the executor's own planned/
  executed CDR-digest comparison (`planned_digest_before` vs
  `executed_digest_after` in `run_execute_sequence`) without the F4.2
  deterministic serializer seam; it now installs the same test-local
  `_install_deterministic_serialize` seam so the canonical planned trajectory is
  serialized exactly once at setup and that byte/digest snapshot is reused for
  every digest computed inside the run window (plan, executed, FJT-join) and the
  provider's FJT evidence.  The test's UUID-mismatch purpose is preserved
  verbatim: the accepted ExecuteTrajectory handle has an invalid/UUID-identity
  mismatch, exactly one bounded cleanup attempt is made, and the final
  `execute_error` is the UUID reason — never a digest mismatch.  The actual
  sent `ExecuteTrajectory.Goal.trajectory` is now asserted semantically
  field-by-field against the setup snapshot
  (`_robot_trajectories_equivalent`).  A complete caller audit confirms every
  `run_execute_sequence` call in the Humble module is either seam-hardened or
  provably returns before the digest comparison (`fjt_transaction_provider is
  None` and `_acquire_scene` no-planning-scene negatives fail closed with zero
  goals, so they cannot reach the digest line).  Production serializer/digest
  code is unchanged, the digest checks are not weakened, and the raw serialized
  bytes are never altered, so Gate D/E runtime semantics are unaffected.

- 2026-08-04 (integrated qualification Task 6, formal-review fix round 4 —
  "preserve Gate E downgrade truth"): Preserved controller truth through the E
  fail-dominant downgrade path and eliminated the load-sensitive rclpy CDR-
  padding digest flake in the D-side qualification tests.  F4.1 — the E
  fail-dominant downgrade writers now carry `controller_goal_sent`/
  `controller_goal_uuid`/`controller_endpoint` from the pre-downgrade truthful
  record into the authoritative downgrade `integrated-execution.json` summary
  and the final `integrated-execution.jsonl`/`controller-results.jsonl`
  downgrade rows, so a late artifact failure never erases or fabricates the
  controller identity an attempt actually observed; two sourced-Humble tests
  force a late goal-artifact write failure after an observed approach FJT and
  on the no-controller path, asserting identical controller truth across every
  authoritative/final downgrade artifact and that no final row claims pass.
  F4.2 — the D-side qualification tests now serialize the canonical planned
  trajectory exactly once per setup and reuse that byte/digest snapshot for the
  provider's FJT evidence, making the plan/executed/FJT-join digests
  byte-identical inside the executor's run window (removing the documented
  rclpy CDR-padding nondeterminism under memory churn), while a separate
  field-by-field semantic identity check (`_robot_trajectories_equivalent`) and
  a mutation-negative test prove the unchanged-trajectory proof is not
  circular.  Production digest semantics and Gate D/E runtime behavior are
  unchanged (the wrapper returns the exact real serialized bytes); the executor
  truth rules (2.0 s FJT correlation, controller traffic only from observed FJT
  evidence) are unaffected.

- 2026-08-04 (integrated qualification Task 6, formal-review fix round 3 —
  "make Gate E temporal evidence deterministic"): Made the flagship Gate-E
  temporal proofs deterministic and sealed the fresh-replay and controller-truth
  gaps, preserving every review-clean F1/F2 behavior.  F3.1 — every flagship
  ordering/race/negative test is now event-driven: `threading.Event` barriers
  gate the approach/transport/Place evidence injections strictly after the
  executor's observable state (goal acceptance + baseline capture, lift latch,
  Place baseline, exact Place cancel terminal) and the delayed Pick-result future
  is Event-released only after the transport latch; fixed `threading.Timer`
  offsets and `time.sleep` event-order margins are gone, and the "late"
  receipt-window negatives pin the FJT `received_mono` strictly beyond the 2.0 s
  boundary.  F3.2 — occupied-place requires a STRICTLY fresh post-cancel
  PlanningScene observation (sequence strictly greater than the pre-cancel
  baseline AND receipt time after the exact cancel terminal); timeout, unchanged
  sequence, malformed/provider-error newer scene, or detached target is
  `evidence-invalid`, with baseline/post-cancel sequence, receipt delta,
  attachment state, and reason recorded in the trigger and durable artifacts.
  F3.3 — controller traffic is derived ONLY from observed FJT evidence:
  `controller_goal_sent` is true only when an actual FJT transaction/status/UUID
  was observed, `controller_endpoint` is None when no FJT was observed, no-goal
  cleanup is `None` (never `{}`), and accepting/canceling a task goal or
  attempting task-goal cleanup never implies a controller goal; the truth is
  preserved consistently in returned records, `integrated-execution.jsonl/.json`,
  `controller-results.jsonl`, `moveit-plans.jsonl`, and goal artifacts.  F3.4 —
  the dead reason-collapsing `_e_post_grasp_lift_m()` helper is deleted (the
  detailed provider reason map stays in `_e_prepare`).  Humble suite 160, pure
  suite 126.

  **Live orchestrator latency obligations (Task 8/10).**  Gate E intentionally
  fails `evidence-invalid` when a live MoveIt/supervisor latency exceeds these
  fixed budgets; Task 8/10 live evidence decides whether a later reviewed config
  change is needed (Task 6 changes no thresholds):

  - `E_FJT_CORRELATION_TIMEOUT_S == 2.0` bounds every FJT receipt window
    (approach vs goal-acceptance baseline, transport vs lift-latch boundary,
    Place target-motion vs Place acceptance baseline).  The executor polls the
    injected FJT status stream at ~spin frequency; the live controller's
    ExecuteTrajectory status must reach the FJT status topic within 2.0 s of the
    corresponding task boundary or the attempt fails closed.  The observed FJT
    `goal_uuid` is recorded as evidence and is never claimed to equal the
    internal Pick/Place `ExecuteTrajectory` goal UUID.
  - The safety-stop observation window (`safety_stop_wait_s`) bounds how long
    the executor waits for the operator assert publication to produce an
    effective stop before the safety-transport run fails closed.
  - The fresh post-cancel PlanningScene obligation requires the scene stream
    (or a fresh-scene service seam) to publish a STRICTLY newer scene after the
    exact Place cancel terminal; the executor waits `post_cancel_scene_wait_s`
    (default 2.0 s).  The live orchestrator must ensure the scene stream keeps
    publishing after a cancel so the post-cancel attachment re-observation is a
    genuinely fresh observation, not the last cached pre-cancel scene.

- 2026-08-04 (integrated qualification Task 6, formal-review fix round 2 —
  "seal Gate E runtime contracts"): Resolved the full F2.1-F2.8 consolidated
  findings.  F2.1 — Gate E now preserves the 10 cm physical threshold: every E
  transport scenario (positive, occupied-place, cancel-transport,
  safety-transport) requires an injected, fresh `post_grasp_lift_m_provider`
  runtime-parameter observation BEFORE any Pick traffic.  The observed
  production `pick_and_place` parameter must be finite and `>= object_lift_m`
  (0.10 m) with fresh identity/receipt metadata; missing/stale/provider-error/
  0.08 evidence fails immediately with a stable readiness reason and zero action
  traffic (never a 15 s transport timeout).  Accepted 0.10 keeps the lift latch
  `grasp_z + object_lift_m - tolerance` (0.81 m), physically reachable at the
  production TCP peak 0.82 m.  F2.2 — the transport ordering tests now use
  controllably delayed Pick result futures and assert the transport latched
  strictly before the result future completed; a settled post-result-only
  provider fails closed with no Place/release.  F2.3 — native-gripper rejection
  coverage is complete (nonzero unchanged baseline passes; increment-after-
  acceptance rejects the approach trigger with exact-Pick cleanup; missing/
  stale/exception fails closed).  F2.4 — runner-level receipt-window negatives
  cover the approach FJT before the acceptance baseline, the approach FJT later
  than 2.0 s, the transport FJT before the lift latch, the transport FJT later
  than 2.0 s after the lift latch, and a Place FJT outside its window — each
  bounded and `evidence-invalid` with no forbidden later goal/release.  F2.5 —
  occupied-place re-observes a fresh PlanningScene after the exact Place cancel
  terminal and quiescence, proving `pick_and_place/object_mesh` remains attached;
  an open/detach race-lost fails `evidence-invalid` with the post-cancel scene
  sequence/attachment recorded in the trigger.  F2.6 — the unexpected-exception
  path derives cleanup/goal identity/sent flags before durable writes so every
  artifact truthfully preserves the accepted-goal state with
  `status=evidence-invalid`.  F2.7 — blocked-approach/unreachable-grasp require
  production-real terminal consistency (GoalStatus ABORTED=6 together with a
  non-success/non-canceled task result such as `planning_failed=2`); a
  contradictory SUCCEEDED-terminal/failure-Result (or canceled/safety Result)
  pair is rejected.  Humble suite 155, pure suite 126.

  **Live `post_grasp_lift_m:=0.10` readback obligation (Task 8/10).**  The later
  live orchestrator MUST launch `pick_and_place` with the ROS parameter override
  `post_grasp_lift_m:=0.10` AND independently read back that value before Gate E
  (the production default is `post_grasp_lift_m=0.08`, which produces an attached
  lift peak of 0.80 m, 0.01 m below the 0.81 m Gate-E lift latch and therefore
  `evidence-invalid`).  The read-back must be provided to Gate E through the
  injected `post_grasp_lift_m_provider` seam with fresh identity/receipt
  metadata; Gate E fails immediately, zero-traffic, if the observed value is
  missing, stale, non-finite, or below `object_lift_m` (0.10 m).

- 2026-08-03 (integrated qualification Task 6, formal-review fix round 1 —
  "make Gate E live-observable"): Made Gate E live-observable per the full
  F1.1-F1.15 consolidated findings.  The positive and occupied-place sequences
  now observe lift-complete and transport **during** Pick execution (production
  `stay=false` returns to `Q_OUTBOUND` before the result, so post-result
  transport evidence can never exist).  The transport trigger is two-phase:
  `lift_complete` latches on observed attachment + TCP z above the lift
  threshold + settled arm velocity + two fresh normal samples; `transport_started`
  then requires a **later** fresh FJT EXECUTING entry + attached target + fresh
  TCP motion, never re-requiring the settled condition while moving.  Safety-
  transport asserts with `publish_operator(True)` and clears with `False` only
  after the effective stop.  Every cancel/safety interruption boundedly awaits
  the exact goal result and records both status domains (canceled: GoalStatus
  CANCELED=5 + `Result.status=4`; safety-transport: `Result.status=5` +
  ABORTED terminal).  The complete trigger object and validated-spec geometry
  are persisted into the authoritative artifacts and final downgrade rows.
  `E_FJT_CORRELATION_TIMEOUT_S == 2.0` bounds all FJT receipt windows and the
  actual receipt delta is recorded.  Every E attempt resets per-attempt state;
  the shared fixture-scene check keeps Gate C/D strict while permitting only the
  exact task target via the explicit E path; all E dispatches fail closed on any
  exception with accepted-goal cleanup; and pre-goal failures still write
  canonical `planning-scene.json`.  Humble suite 141, pure suite 124.

  **Live order obligation — transport observed during Pick execution.**  The
  live orchestrator (Task 7/10) must keep the Pick goal **executing** while the
  executor polls the two-phase transport predicate: production Pick with
  `stay=false` lifts, returns to `Q_OUTBOUND`, and only then publishes its
  result, so the executor latches the lift/transport checkpoints from the
  injected TCP/JJT/scene streams during that window.  A flow that awaits the
  Pick terminal before polling transport can never observe the transient return
  motion and will finalize `evidence-invalid` (the pre-fix defect).

  **Live native-gripper obligation (cancel-approach).**  cancel-approach now
  requires the injected, receipt-sequenced
  `native_gripper_goal_count_provider` seam instead of the fake-only
  `ActionClient.sent_goals` attribute.  The later live orchestrator must provide
  the native gripper action-goal count **from the real action stream**: a fresh
  non-negative integer count plus a fresh `age_s` gate, captured at baseline and
  at each trigger poll.  The trigger requires the count not to increase and the
  target to remain unattached; missing/stale/provider-error evidence fails
  closed with `evidence-invalid`.

  **Live scene-attach prerequisite.**  The journal attach transition requires
  the target world object to be present in the pre-attach scene (shared
  `_validate_transition`).  The live orchestrator must therefore **predeclare
  the task-owned world object** (`pick_and_place/object_mesh`) before the Pick
  attach transition (or record an intermediate world-appearance diff); the
  executor fails closed when the target world object is absent at fixture-ready
  and never fabricates the object.

- 2026-08-03 (integrated qualification Task 6 — "add fixed-target Pick and Place
  controls"): Added the Stage-E fixed-target Pick/Place diagnostics executor to
  the same ROS-lazy `validation/integrated_gate_executor.py`.  A closed ROS-free
  `stage_e_dispatch` validates exactly the eight committed Stage-E scenarios
  (`qualification-pick-place-{positive,blocked-approach,unreachable-grasp,
  malformed-back,cancel-approach,cancel-transport,safety-transport,
  occupied-place}`) for exact id, `integrated.stage == "E"`,
  `execution_profile == "sim_ompl"`, exact polarity, exact `expected_physical` /
  `expected_negative` contracts, exact `forbidden_endpoints ==
  ["/isaac_joint_commands"]`, exact `trigger_timeout_s`, and pinned fixed-target
  geometry (`grasp_tcp_xyz [0.65, 0, 0.72]`, `object_root_xyz [0.65, 0, 0.60]`,
  `place_target_point` base_link `[0.85, 0, 0.72]`, identity orientation) plus
  the declared six-value malformed-back vector; unknown/mutated/C/D-stage
  scenarios fail closed before any goal.  The positive sequence uses only
  production `/pickup_action` then `/place_action` with the deterministic cube
  cloud and exact seven-joint `Q_OUTBOUND` (`use_mesh=True`, `stay=False`) and
  records `scene-attach`/`scene-detach` only from observed PlanningScene
  transitions, never from action-result inference.  The E journal branches per
  scenario (positive equals `POSITIVE_ORDER`; each negative has its own exact
  diagnostic order and forbidden events), leaving Gate-C/D journal bytes and the
  Task-3 journal graph unchanged.

  **Live TCP provider obligation (later orchestration).**  Task 6 reuses the
  executor's injected `current_tcp_pose_provider` seam and owns no TF state: the
  executor keeps a bounded per-attempt sample deque and derives `tcp_z_m` plus
  `tcp_speed_m_s = |Δxyz|/Δt` from the two newest fresh receipt-sequenced
  `base_link` samples; fewer than two fresh samples means the trigger cannot
  fire and a timeout is evidence-invalid, never a pass.  The **live orchestrator
  (Task 7/10) must supply** a TF-backed provider in its own ROS node/context that
  reads `/tf` `world → base_link → link_tcp` from the sim/DDS, resolves the chain
  to `link_tcp`, gates freshness against the configured `tf_fresh_s`, and returns
  the normalized fresh `base_link` TCP-pose mapping.  The executor destroys no TF
  state and `shutdown()` need not touch TF; the provider is injected and freed by
  the orchestrator.  FJT correlation is receipt-window only: the executor captures
  the FJT/joint/start baseline at Pick/Place goal acceptance, uses the first fresh
  FJT EXECUTING entry after acceptance for the approach trigger, the first later
  fresh EXECUTING entry for transport, and records the observed FJT goal UUID as
  evidence — it never claims the internal Pick/Place ExecuteTrajectory UUID is
  observable (that UUID is private to production).

  **Gate E artifact and status semantics.**  Two status domains are kept separate
  and both recorded: the action-client `GoalStatus` (SUCCEEDED=4, CANCELED=5,
  ABORTED=6) and the Pick/Place `Result.status` (0 success, 1 invalid_goal,
  2 planning_failed, 3 execution_failed, 4 canceled, 5 safety_stop,
  6 scene_inconsistent, 7 postcondition_failed, 8 timeout, 9 internal_error).
  A canceled Pick/Place therefore records both `terminal_status="canceled"` and
  `task_result_status=4`/`task_result_status_string="canceled"`.  The fail-dominant
  artifact transaction mirrors Gate D with `event="gate-e"`, `stage="E"`,
  `diagnostic_only=true`, `physical_verdict=null`, handler/polarity, pick/place
  goal-sent flags, `goals_sent`, task/FJT UUIDs, trigger record, and
  `isaac_joint_commands_published=false` across `integrated-execution.jsonl/.json`,
  `moveit-plans.jsonl`, `controller-results.jsonl`, visual-capture requests
  (`capture.kind="gate-e-diagnostic"`), and per-scenario `goals/<scenario_id>.json`;
  any journal/graph/artifact write or finalization failure downgrades every
  status-bearing stream to a final `evidence-invalid` disposition.  Negative
  diagnostics may expose `diagnostic-pass` only when their exact short journal,
  graph, and artifacts are complete.  Task 6 records diagnostics only; raw
  contact/lift/release/collision verdicts remain Task 7 verifier work.  No build
  or live run is required.

- 2026-08-03 (integrated qualification Task 5, formal-review fix round 2 —
  "seal Gate D evidence streams"): Sealed the Stage-D evidence streams against
  the formal SPEC/QUALITY residual findings.  A D artifact-write downgrade
  appends a final `row_kind="final"`/`status="evidence-invalid"` corrective row
  to every status stream (`integrated-execution.jsonl`, `moveit-plans.jsonl`,
  `controller-results.jsonl`) preserving planner/plan/controller/action/UUID/
  digest fields and `downgraded_from`; gripper close→open is supported through a
  fresh close-first journal contract selected before its first record (never
  mutating a nonempty journal); D scene-acquisition failures carry D
  stage/handler/scenario labels and the D schema; an accepted execute UUID
  rejection performs one bounded cleanup; environment-cloud evidence requires
  structural PointCloud2 self-consistency (`row_step >= width*point_step`,
  `len(data) == row_step*height`, usable x/y/z FLOAT32 layout when fields are
  advertised); both MoveGroup builders pin `look_around=False`; the dead
  fail-open helpers are removed; and `controller_goal_sent` is documented as the
  exact FJT semantic with retreat/gripper traffic surfaced through
  `action_goal_sent`/`action_endpoint`/`cartesian_goal_sent`/`gripper_goal_sent`
  and D visual captures labeled `gate-d-diagnostic`.  Gate-C bytes, the Task-3
  journal graph, and all policy blobs are unchanged.

- 2026-08-02 (integrated qualification Task 5, pre-review fix round 1 — "make
  Gate D runtime truthful"): Hardened the Stage-D lifecycle and evidence path
  against the real Humble rclpy action API.  `shutdown()` now explicitly destroys
  every owned rclpy `ActionClient` (private `_owned_action_clients` collection,
  kept apart from the mutable public map) before node/context teardown, removing
  the action waitable/C-handle leak that was the leading in-process suspect for
  the coordinator's full-suite SIGSEGV; repeated construct/shutdown and partial
  constructor failure destroy clients exactly once.  Cancellation requires the
  exact live ExecuteTrajectory `ClientGoalHandle`, exactly one
  `cancel_goal_async()` on it, the real-shape `CancelGoal.Response`
  (`return_code == ERROR_NONE`, `goals_canceling == [execute_goal_id]`), terminal
  CANCELED (5) for both the ExecuteTrajectory result and the joined FJT
  controller goal, and bounded quiescence; raw UUIDs, rejected/unknown/terminated/
  empty/extra/malformed/timed-out responses, and SUCCEEDED/ABORTED terminals fail
  closed.  Safety requires provider evidence to validate and join to a fresh
  current-window status entry; provider exceptions, no-provider, stale, wrong
  UUID/digest/source/endpoint/status-cache, and prior-ABORTED-cache entries fail
  closed.  FJT/status and joint-state evidence is windowed to the current attempt
  with receipt baselines and bounded wait helpers (joined EXECUTING motion
  trigger, joined terminal status, quiescence, `safety_stop_frames` consecutive
  fresh bounded frames, post-clear `safety_position_creep_rad` stability);
  accepted ExecuteTrajectory goals are cleaned up on timeout/exception.  All six
  D handlers route success and failure through one fail-dominant artifact path
  writing `integrated-execution.jsonl/.json`, `moveit-plans.jsonl` (explicit
  `plan_applicable=false`/`planner_status=null` for the non-MoveIt retreat/
  gripper handlers), `controller-results.jsonl`, visual-capture rows, and
  `goals/<scenario_id>.json`; required write failures downgrade every
  authoritative artifact.  `run_cartesian_retreat` requires an explicit fresh
  non-empty `base_link` PointCloud2 provider passed into
  `CartesianMove.Goal.env_points` before `collision_checking=true`.  Humble suite
  expanded to 109 tests (red/green lifecycle, cancel/safety fail-closed matrices,
  windowed-evidence helpers, timeout cleanup, artifact/journal failure injection,
  retreat env-cloud validation, execute-pose, visual chronology).  No build is
  required.

- 2026-08-02 (integrated qualification Task 5): Extended the same
  `validation/integrated_gate_executor.py` with the Stage-D execution
  interruption gates.  A closed ROS-free `stage_d_dispatch` validates exactly
  the six Stage-D scenarios (`qualification-moveit-execute-joint`,
  `-execute-pose`, `-cartesian-retreat`, `-gripper`, `-cancel`, `-safety`) for
  exact id, `integrated.stage == "D"`, `execution_profile == "sim_ompl"`, exact
  declared polarity (`positive` / `cancel` / `safety`), the exact configured
  `expected_physical` list, and `forbidden_endpoints == ["/isaac_joint_commands"]`,
  failing closed before any goal on unknown/C/E-stage/malformed/mutated
  scenarios.  The split path sends exactly one OMPL plan-only `/move_action`
  goal, assigns the returned `planned_trajectory` unchanged to exactly one
  `moveit_msgs/action/ExecuteTrajectory.Goal` sent to `/execute_trajectory`
  (`build_execute_trajectory_goal`, canonical digest unchanged), records distinct
  valid 16-byte plan/execute UUIDs, and uses action_msgs terminal statuses
  (SUCCEEDED=4, CANCELED=5, ABORTED=6; unknown/malformed never pass).  FJT
  observation is truthful: only the real
  `/xarm7_traj_controller/follow_joint_trajectory/_action/status` subscription
  exists (`GoalStatusArray`, RELIABLE/TRANSIENT_LOCAL/depth 1; no `_action/goal`
  topic — the goal travels over the `send_goal` service), with a bounded cache
  and an injected `fjt_transaction_provider` whose evidence must join to the
  newest status entry with the unchanged trajectory digest; missing/stale/
  mismatched/malformed/provider-exception evidence is `evidence-invalid`.
  `run_cancel_sequence` cancels only the ExecuteTrajectory handle (never the
  completed MoveGroup planning handle), records exactly the execution UUID in
  `goals_canceling`, requires CANCELED (5), quiescence, and no later stage.
  `run_safety_sequence` publishes operator True, waits bounded for safety-stop
  True, requires the old ExecuteTrajectory terminal ABORTED (6),
  `safety_stop_frames` bounded joint-state velocity frames, operator False after
  the effective-stop, post-clear stability, and no replacement/resume goal.
  `run_cartesian_retreat` uses an injected `current_tcp_pose_provider` (no
  embedded TF listener), derives exactly 0.10 m along `+Z` in `base_link`
  preserving orientation, and sends one collision-aware `/cartesian_move_action`
  goal with `command_gateway_bypassed=false`.  `run_gripper_sequence` sends open
  (0.0) then close (0.85) `GripperCommand` goals to `/xarm_gripper/gripper_action`
  with max effort 10.0 and `native_action=true`.  A fail-closed
  `run_pick_place_negative` stub covers the two Gate-E cancellation identities
  with `release_stage_started=false`, `released=false`, and zero goals.  D-stage
  artifacts reuse Task 4's transactional/fail-dominant mechanics with a separate
  D record shape (`diagnostic_only=true`, `physical_verdict=None`, fail-dominant
  `status`/`planner_status`, truthful goal-sent booleans, UUIDs, digests, FJT
  UUID/status, execute result status/string, event log, elapsed,
  `isaac_joint_commands_published=false`); required write/finalization failures
  downgrade every authoritative D artifact.  The journal uses scenario-specific D
  diagnostic event orders (see README) with the eight Task-4 forbidden
  manipulation events unchanged, the Task-3 graph projection byte-identical, and
  no attach/detach/release event in Gate D.  Humble suite expanded to 74 tests;
  pure suite expanded to 97 tests.  No package-installed path changed, so no
  build is required.

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

- 2026-08-02 (integrated qualification Task 4 fix round 2): Made Gate-C evidence
  fail-dominant and aligned PlanningScene QoS with stock MoveIt2 Humble.
  `run_gate_c_plan_only` now computes one authoritative final status after the
  plan outcome *and* every required evidence finalization step; any
  readiness/journal/graph/finalization/artifact-serialization/existence failure
  returns and persists `evidence-invalid` (in the public record and
  `integrated-execution.json`), the raw planner outcome is preserved separately
  as `planner_status`, and `planning-scene.json` is always produced as a
  canonical failure artifact through a narrow `PlanningSceneJournal.finalize_failure`
  extension (records `evidence-invalid`, the failure reason, the existing journal
  records, and an invalid-graph diagnosis without pretending validation passed).
  Every expected runtime/DDS/action failure is converted into finite canonical
  diagnostic records with a stable reason code and zero physical/command claims;
  once `fixture-ready` exists the executor always attempts teardown journal
  completion and failed finalization, and no exception escapes the public API.
  The `/planning_scene` and `/monitored_planning_scene` subscriptions and the
  Task-3/4 observed-graph projection now use the stock MoveIt2 Humble
  RELIABLE/VOLATILE/depth-100 contract (the stale TRANSIENT_LOCAL/depth-1 claim
  is rejected by `validate_graph_evidence` and the projection builder); the
  fixture status topic stays RELIABLE/TRANSIENT_LOCAL/depth 1.  The blocked
  scenario only passes on an explicit MoveIt planning-stage non-success
  allowlist (`PLANNING_FAILED`, `INVALID_MOTION_PLAN`, `NO_IK_SOLUTION`);
  request-level/unknown codes are `diagnostic-fail` with a recorded
  `error_code_classification`.  A bounded pre-goal scene-acquisition phase
  self-spins up to `scene_acquire_timeout_s`; `fixture-ready` requires the scene
  ordered owned IDs to equal the declared fixture contract exactly
  (missing/extra/reordered/attached rejected before goal send); the `before`
  visual-capture request is durably flushed before the goal send and `after` only
  in the post-transaction phase; acceptance-timeout handling is truthful
  (canceling a client future is not proof of server-side cancellation); and the
  executor's atomic write now includes parent-directory fsync.  Humble suite
  expanded to 43 tests; pure suite to 68; journal suite to 184.  No
  package-installed path changed, so no build is required.

- 2026-08-02 (integrated qualification Task 4 fix round 3): Sealed Gate-C
  artifact/scene-state consistency across the residual review findings.
  `run_gate_c_plan_only` now finalizes transactionally: the graph is validated
  through `PlanningSceneJournal.finalize(status, graph, json_path=None)` before
  any durable output, non-journal artifacts are written first, and the
  successful `planning-scene.json` is deferred until every other required
  artifact is durable.  Any post-provisional-pass write failure invokes
  `_downgrade_persisted_evidence`, which rewrites `integrated-execution.json`
  fail-dominantly, appends `row_kind="final"` corrective rows to the JSONL
  lifecycle files (provisional/raw rows carry `row_kind="lifecycle"`), and
  writes a canonical failure `planning-scene.json`; `planner_status` stays
  available as diagnostic history while every authoritative `status` is
  fail-dominant.  A valid PlanningScene callback now atomically caches the new
  scene and clears the invalid latch; acquisition requires a valid observation
  whose `scene_sequence` is after the last invalid one (`_scene_invalid_sequence`),
  so invalid messages never erase the last valid cached scene yet acquisition
  stays fail-closed while invalid is newest.  The callback boundary catches the
  expected malformed-message exception set
  (AttributeError/IndexError/KeyError/TypeError/ValueError) without swallowing
  process-control exceptions.  `fixture-ready` now binds to the exact declared
  geometry and pose, not IDs only: `expected_fixture_geometry_digest` derives a
  deterministic projection (bridge helpers `fixture_to_specs`/`spec_geometry`/
  `readback_geometry`/`geometry_signature_sha256`) covering exact ordered owned
  IDs, geometry/dimensions, frame, and poses, and `_fixture_scene_error` rejects
  stale pose, wrong dimensions/frame, and duplicate IDs.  The blocked scenario
  now passes only on an allowlisted planning-stage non-success code *with* an
  empty planned trajectory; an allowlisted code plus a non-empty trajectory is
  `contradictory-nonempty-trajectory` and fails.  This entry supersedes the
  fix-round-2 "Humble suite expanded to 43 tests" count with the fresh 44-test
  baseline at `d911692` before fix round 3; post-fix counts are Humble 58, pure
  70, journal 184.  No package-installed path changed, so no build is required.

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
