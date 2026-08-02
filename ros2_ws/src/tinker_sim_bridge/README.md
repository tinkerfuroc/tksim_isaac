# tinker_sim_bridge

External Humble hardware-parity gateways and the canonical manipulation
model-bundle producer/preflight for the isolated Tinker simulation boundary.

## Canonical model bundle (Task 3)

`model_bundle` is the real producer of the canonical manipulation model-bundle
manifest consumed by the production `xarm_moveit_config` validator
(`launch/lib/tinker_model_bundle.py`).  `model_preflight` is the bounded,
pure manifest/provider-entry validator called synchronously before any
simulator provider is constructed.

The modules in this overlay are ROS-free at import time and run under both
simulator CPython 3.12 and system Humble CPython 3.10.

### Schema

One schema, version `1`, used by the simulator producer, production consumer,
preflight, readiness, and provenance report:

- `schema_version`: integer constant `1`.
- `producer`: exactly `{"name": "tinker_sim_bridge.model_bundle", "version": "1"}`.
- `artifacts`: exactly five entries named `simulator_full_urdf`, `planning_urdf`,
  `planning_srdf`, `joint_limits`, and `kinematics`.  Each entry has an
  absolute, existing regular-file `path` and a lowercase nonzero SHA-256.
- `normalization`: `prefix`, the exact zero `world -> base_link` `mount`, the
  exact group names `xarm7` and `xarm_gripper`, the ordered eight-joint list
  `joint1`..`joint7` followed by `drive_joint`, and the selected normalized
  link list.
- `contract`: `planning_frame=base_link`, `tcp_link=link_tcp`, ordered
  `arm_joints`, `gripper_joint=drive_joint`, recursively resolved `groups`, an
  end-effector record whose group is `xarm_gripper` and parent is `link_tcp`,
  the resolved eight-link `touch_links`, selected finite `joint_limits`,
  selected finite `collision_geometry`, semantic `kinematics`, and the declared
  fixed mount.
- `structural_fingerprint`: lowercase nonzero SHA-256 over canonical JSON of the
  complete normalized contract.

For the xArm7/gripper artifact the resolved end-effector touch set is exactly:
`xarm_gripper_base_link`, `left_outer_knuckle`, `left_finger`,
`left_inner_knuckle`, `right_inner_knuckle`, `right_outer_knuckle`,
`right_finger`, `link_tcp`.

### Producer CLI

```text
model_bundle --simulator-full-urdf PATH --planning-urdf PATH --planning-srdf PATH
  --joint-limits PATH --kinematics PATH --prefix PREFIX --mount-parent world
  --mount-child base_link --output PATH
```

The producer validates every input, parses the narrow manipulation subgraph,
computes exact byte hashes and the structural fingerprint, and atomically
renames the complete manifest into the output directory.  The simulator full
URDF should be resolved through the current content-addressed selector via
`resolve_simulator_full_urdf(project_root)`, which delegates to the one shared
authoritative resolver (see below) and never pins an artifact hash.

### Joint-limit synthesis (required for the canonical bundle)

The canonical schema requires all eight selected joints in `joint_limits`, but
the production arm file (`xarm_moveit_config/config/xarm7/joint_limits.yaml`)
defines only `joint1`..`joint7`.  `model_limits` deterministically synthesizes
the canonical eight-joint `joint_limits` artifact from the committed arm and
gripper source YAML files and writes it atomically, so the merged artifact is
itself the path+bytes hashed into the manifest and is reproducible:

```bash
ros2 run tinker_sim_bridge model_limits \
  --arm-joint-limits "$TINKER_WS/src/tk25_manipulation/src/xarm_ros2/xarm_moveit_config/config/xarm7/joint_limits.yaml" \
  --gripper-joint-limits "$TINKER_WS/src/tk25_manipulation/src/xarm_ros2/xarm_moveit_config/config/xarm_gripper/joint_limits.yaml" \
  --output "$PWD/outputs/ompl-overlay/model-bundle/joint_limits.yaml"
ros2 run tinker_sim_bridge model_bundle \
  --simulator-full-urdf "$(./scripts/model-bundle-sim-urdf.sh)" \
  --planning-urdf "$TINKER_WS/src/tk25_basic/src/cumotion_description/config/xarm7.urdf" \
  --planning-srdf "$TINKER_WS/src/tk25_basic/src/cumotion_description/config/xarm7.srdf" \
  --joint-limits "$PWD/outputs/ompl-overlay/model-bundle/joint_limits.yaml" \
  --kinematics "$TINKER_WS/src/tk25_manipulation/src/xarm_ros2/xarm_moveit_config/config/xarm7/kinematics.yaml" \
  --prefix "" --mount-parent world --mount-child base_link \
  --output "$PWD/outputs/ompl-overlay/model-bundle/model-bundle.json"
```

### Current-artifact resolution (one shared resolver)

`model_bundle`, `model_preflight`, and the runtime deployment tooling all
resolve `artifacts/robot/tinker2/current.json` through the single authoritative
resolver in `tools/tinker_sim_deploy/runtime.py`.  It explicitly dispatches and
validates both the currently deployed legacy selector (unversioned pointer +
schema-2 manifest) and the schema-4 publication shape; any other shape is
rejected.  The overlay accesses it through `tinker_sim_bridge.current_artifact`,
keeping the model modules ROS-free at import while sharing the resolver's full
integrity checks (schema, robot/artifact binding, safe contained paths, manifest
agreement, selected `robot.urdf`).  On migration to schema 4 the resolver
automatically enforces the stronger checks; no overlay-specific reader exists.

`current_artifact._shared_resolver` locates `tools/tinker_sim_deploy` first
through the authoritative project root supplied by the caller or derived from
the manifest simulator-artifact path, then falls back to module-tree/
environment/cwd discovery.  This keeps a genuine copied ROS install outside the
checkout able to reach the real resolver without `TINKER_SIM_ROOT`, and it
stays fail-closed when neither a project root nor an authoritative artifact
tree is available.

### Preflight CLI

```text
model_preflight --model-bundle-manifest PATH --report PATH --timeout SECONDS
```

The preflight verifies manifest schema, absolute paths, exact hashes, the
selected-subgraph contract, installed/source artifact identity, prefix, mount,
groups, end-effector parent, resolved touch links, limits, collision geometry,
and finite JSON output.  Artifact identity proves the manifest simulator
artifact byte-equals the authoritative `current.json` selection (identical
copied bytes pass; stale/different bytes fail) and fails closed whenever no
authoritative root can be resolved — never `ok=true`/`not applicable`.  It
returns a typed result for every mismatch, artifact/path state, timeout, or
safety classification and atomically writes a report only for the fully ready
result.

### Tests

```bash
cd /home/tinker/tinker-sim/6.0.1
PYTHONPATH="$PWD/ros2_ws/src/tinker_sim_bridge:$PWD/simulation" \
  ./.venv/bin/python -m pytest -q \
  tests/test_model_contract.py tests/test_model_bundle.py tests/test_model_preflight.py
```

The focused Task 8 provenance suite (deterministic OMPL-overlay acceptance
contract) uses `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` as the defensive reproducible
invocation (ROS plugin discovery may auto-load the Humble `launch_pytest`
plugin, which can fail collection on hosts without the `lark` dependency):

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ./.venv/bin/python -m pytest -q \
  tests/test_provenance.py
```

## Integrated eight-joint state contract (Task 4)

The integrated `/joint_states` contract is exactly eight joints — `joint1` ..
`joint7` followed by `drive_joint` — where `drive_joint` is state-only
(`position`, `velocity`, `effort`; zero command interfaces).  `drive_joint`
appears exactly once in the checked-in
`config/tinker_topic_control.ros2_control.xacro` and in the live controller
description produced by `tools/tinker_sim_deploy/runtime.py:
topic_control_description` from the complete canonical robot URDF.  The
`xarm7_traj_controller` keeps the seven arm joints, the gripper command path
remains `/sim/controller/gripper_commands`, and `joint_state_broadcaster`
remains the sole `/joint_states` publisher.

### Pure contract helpers (ROS-free)

`tinker_sim_bridge/contract_guard.py` exposes complete-input pure helpers used
by both the contract tests and the live probe:

- `evaluate_joint_state_sample(*, publisher_node, publisher_count, names,
  positions, velocities, header_stamp_ns, received_at_ns, now_ns)` — classifies
  one actual `sensor_msgs/msg/JointState` sample (exact names, cardinality one,
  source `joint_state_broadcaster`, nonzero stamp, finite arrays, bounded
  age/transport).  All times are explicit; no global clock or implicit topic.
  An epoch-scale header/now difference additionally records a probable
  `use_sim_time` clock-domain-mismatch reason (without replacing the ordinary
  stale/transport reasons).
- `evaluate_integrated_cardinality(*, joint_state_publishers)` — classifies
  publisher cardinality/source from ROS graph endpoint metadata.
- `evaluate_robot_description_contract(description)` — evaluates the same
  `drive_joint` state-only contract in the `robot_description` parameter
  received by `/controller_manager`.
- `evaluate_xacro_contract(xacro_text)` and
  `evaluate_joint_state_evidence_pair(...)` — parse the checked-in xacro source
  and compare source-xacro and live-parameter evidence together.
- `evaluate_clock_domain(...)` — verifies the probe and controller_manager agree
  on `use_sim_time` and that an active `/clock` has advanced past zero.
- `derive_logical_joint_state_publishers(...)` — derives the logical
  `joint_state_broadcaster` source only when the evidence proves it (a standalone
  exact-name node at the root namespace, or a controller-manager-hosted
  publisher with exactly one active controller of that exact name from
  `list_controllers`); otherwise it preserves the raw label and fails honestly.
  A standalone exact broadcaster satisfies attribution without any
  controller-manager list; only controller-manager-hosted endpoints require
  fresh exact active-controller proof.
- `evaluate_sample_freshness(...)` — wall-clock watchdog so a later sample
  loss/staleness produces FAIL.
- `evaluate_probe_verdict(...)` — fail-closed aggregation seam: sample readiness
  participates unconditionally, so no sample/stale sample/controller or graph
  failure after a prior PASS yields FAIL, replacing any latched PASS.
- `step_service(...)` — bounded, recoverable async ROS service state machine
  (discovery-pending, in-flight, timeout, exception, malformed response,
  recovery).  It accepts an explicit monotonic `now_s` plus a freshness `ttl_s`
  and an in-flight `timeout_s`: successful evidence is re-polled after its TTL
  (so a controller_manager restart or parameter change is re-verified on a
  bounded cadence), and an in-flight request older than `timeout_s` is abandoned
  and the client reset so the probe retries without leaking a client or keeping a
  stale future.  `succeeded_at` / `started_at` are tracked per state.

Each returns a complete mapping with `ready`, `reasons`, and the observed
values.  These modules stay import-time ROS-free and run under both simulator
CPython 3.12 and Humble CPython 3.10.

### ROS Humble live probe

`joint_state_probe` (`ros2 run tinker_sim_bridge joint_state_probe`) is the
Task 6 evidence path.  It reads the `/controller_manager` `robot_description`
and `use_sim_time` parameters (parsed with the real Humble
`rcl_interfaces.msg.ParameterType` constants), queries
`/controller_manager/list_controllers` to prove the publisher source, parses the
checked-in xacro, subscribes to `/joint_states`, reads `/joint_states` and
`/clock` publisher graph metadata, and evaluates all of the above pure helpers.
Both service reads are re-polled on a 30 s TTL with a 5 s in-flight timeout, so
a controller_manager restart or parameter change is re-verified on a bounded
cadence and latched PASS evidence cannot survive the restart.  The verdict is
fail-closed — no sample, stale/no-new sample, clock-domain mismatch, unproven
publisher source, stale/expired service evidence, or any service failure
produces FAIL with explicit evidence — and is published as compact JSON on
`/sim/status/joint_state_contract` (latched, reliable).  `controller_evidence`
and `parameter_evidence` carry `fresh`, `ttl_s`, `timeout_s`, and `succeeded_at`
so the freshness gate is observable.  It never counts source text as primary
evidence; `source_text_diagnostics` in the report is supplemental only.

The probe must run on the same clock as the controller_manager (sim time):

```bash
ros2 run tinker_sim_bridge joint_state_probe \
  --ros-args -p use_sim_time:=true \
  -p controller_manager:=/controller_manager \
  -p broadcaster:=joint_state_broadcaster
```

### Tests

```bash
cd /home/tinker/tinker-sim/6.0.1
PYTHONPATH="$PWD/ros2_ws/src/tinker_sim_bridge:$PWD/simulation" \
  ./.venv/bin/python -m pytest -q \
  tests/test_integrated_joint_state_contract.py \
  tests/test_manipulation_integration_contract.py \
  tests/test_contract_guard.py
```

The real selected production artifact is exercised by
`tests/test_integrated_joint_state_contract.py::test_real_selected_artifact_state_only_drive_joint_contract`
through the authoritative Task 3 resolver (it skips only when the gitignored
artifact tree is not provisioned).  Probe-node behavior under the Humble Python
3.10 runtime is covered by `tests/test_joint_state_probe_node.py`.

## Atomic fixture PlanningScene adapter (Task 5)

`fixture_planning_scene` is the live Humble node that installs the public
qualification fixtures into the MoveIt PlanningScene as exactly one atomic
replacement diff.  The fixture namespace is exclusively `sim_fixture/*`; the
task-owned handoff is exactly `pick_and_place/object_mesh`; no other PlanningScene
namespace is removed or replaced.  The integrated qualification matrix (Task 1)
defines 17 immutable scenarios under `simulation/scenarios/` and the schema-v3
config `simulation/qualification/integrated-ompl.json`.  The plan-only C-stage
scenarios (`qualification-moveit-plan-joint`, `qualification-moveit-plan-pose`,
`qualification-moveit-plan-blocked`) share the same pedestal and public-target
identity; the blocked scenario adds `sim_fixture/plan_blocker` while retaining
the same target source/handoff.  The D-stage execute scenarios
(`qualification-moveit-execute-*`, `-cartesian-retreat`, `-gripper`, `-cancel`,
`-safety`) reuse the same fixture geometry and declare no spawned physical task
object (they exercise arm/gripper execution against the declared fixtures
only).  The E-stage pick-place scenarios (`qualification-pick-place-*`) target
`sim_fixture/qualification_cube` and declare a source pedestal
(`sim_fixture/pedestal`) plus a place-support pedestal
(`sim_fixture/place_pedestal`) whose top (z 0.60) supports the placement object
bottom; the negative variants add only their own obstacle
(`sim_fixture/plan_blocker`) or occupant (`sim_fixture/place_occupant`).  Fix
round 1 aligned the physical (bottom-origin) roots with the PlanningScene
(center-origin) poses: the 0.08 m cube physical root is z 0.60 and its
PlanningScene center is z 0.64 (root + committed asset center offset 0.04); the
blocked-approach blocker physically rests at root z 0.70 with its PlanningScene
center at z 0.85 so the declared top-down target TCP z 0.72 lies inside the
blocker without initial target contact.  Every scenario is scenario-v2 and the
scenario-v2 loader validates its `integrated` mapping (`execution_profile`
`sim_ompl`, stage, `physics_truth` authority, acceptance polarity, race policy,
and terminal policy); schema-v3 applies to the qualification config
`simulation/qualification/integrated-ompl.json`.  The public
`scenario-runner.json` report carries only the one-key `integrated` mapping
`{"execution_profile": "sim_ompl"}`, and the full per-scenario mapping is bound
by the scenario declaration SHA-256.

### ROS-free contract (`fixture_contract.py`)

`fixture_contract.py` is import-time ROS-free and defines the typed helpers:

- `revision_digest(planning_scene)` — deterministic canonical digest over the
  full `planning_scene` declaration (excluding its own `revision_digest` key).
- `parse_required_fixture_owned_ids(value)` — parses the declared task-owned
  `sim_fixture/*` ids from a comma/JSON parameter, rejecting foreign or
  duplicate ids.
- `build_atomic_revision_diff(*, desired_objects, existing_ids)` — returns one
  `PlanningSceneDiffPlan`: every desired `sim_fixture/*` object as an ADD and
  every stale existing `sim_fixture/*` id as a REMOVE; foreign-namespace ids
  are never touched.  `apply_request` is exactly one canonical JSON-able
  PlanningScene diff.
- `spec_geometry(spec, resolve_mesh=...)` and `readback_geometry(obj)` — a
  deterministic canonical ROS-free geometry descriptor (declared-order id,
  frame, poses, primitive type+dimensions, mesh vertices+triangles normalized
  through the float32 wire representation, operation-independent).  The
  internal `geometry_signature_sha256` digests those descriptors and is used to
  prove the readback matches the declared fixture geometry; it is separate from
  the published 12-key `fixture_descriptor_sha256` (the declaration digest).
- `confirm_fixture_revision(*, service_result, scene_ids, status,
  expected_revision, expected_digest, expected_owned_ids, expected_geometry,
  observed_geometry)` — fail-closed readback/status confirmation covering
  owned-id presence, no foreign `sim_fixture/*` leakage, no duplicate
  `sim_fixture/*` id, exact readback-vs-declared geometry (frame, poses,
  primitive type/dimensions, mesh vertices/triangles), and canonical status
  consistency (schema version, ready state, owner, revision, digest, owned ids,
  target identity, monotonic sequence, finite `published_at`, descriptor
  digest).  Readback object order is normalized (not semantic).

The immutable data types `CollisionObjectSpec`, `PlanningSceneDiffPlan`, and
`Confirmation` carry every field the node and tests consume.

`fixture_planning_scene.py` (also ROS-free) bridges a scenario `planning_scene`
declaration into ADD specs (`fixture_to_specs`), the declared-order owned ids
(`fixture_owned_ids`), the shared fixture descriptor (`fixture_descriptor` +
`fixture_descriptor_sha256`), and the canonical shared fixture-status mapping
(`canonical_fixture_status`).  Diagnostic regions enter the collision-body set
only when explicitly marked `enter_collision_bodies: true`.

The touch-link set exported as `MODEL_CONTRACT_TOUCH_LINKS` is imported from
the validated Task 3 `model_contract.TOUCH_LINKS` (a single authoritative
source), never an independent literal.

### Mesh assets

Mesh-declared fixtures are fully supported: `parse_mesh_bytes` parses STL
(binary or ASCII) and OBJ into finite vertices and nondegenerate in-range
triangle indices with a narrow ROS-free stdlib-only parser (no heavyweight
dependency), and `load_mesh_asset` resolves a declared `uri` against the
project root, requires the file to exist, recomputes and compares its SHA-256
against the declaration, applies the positive scale, and emits real
`shape_msgs/Mesh` content.  Only `.stl` and `.obj` extensions are supported;
any other extension is rejected during scenario validation and mesh loading so
a mesh fixture can never become a silently-empty `Mesh()`.

### Scenario schema

Scenario schema version 2 gains an optional `planning_scene` object (no parallel
scenario format).  Strict validation requires a nonempty revision, a canonical
matching `revision_digest`, frame `base_link`, unique `sim_fixture/*` ids, finite
poses, positive primitive dimensions, a declared `target_source_id`, and the
exact scalar `target_handoff` of `pick_and_place/object_mesh`.  Mesh fixtures
must name an existing supported-format (`/\.(stl|obj)$/`) asset whose content
SHA-256 matches the declaration.  `target_source_id` must name a fixture that
enters the collision-body/owned set — a public object or a diagnostic explicitly
marked `enter_collision_bodies: true`; it cannot name a diagnostic excluded from
the collision-body set.

### Live node

`fixture_planning_scene_node.FixturePlanningScene` gates on the staged
`/sim/ready/physics` Trigger service, waits boundedly for `/apply_planning_scene`
and `/get_planning_scene`, pre-reads the current scene (requesting the explicit
`WORLD_OBJECT_NAMES | WORLD_OBJECT_GEOMETRY` PlanningScene components bitmask,
never the server-dependent `components=0` default) to discover stale
`sim_fixture/*` ids, applies exactly one atomic diff (with real mesh geometry
for mesh fixtures), reads the scene back with full `CollisionObject` geometry,
confirms readback ids, geometry, and status, and only then serves
`/sim/ready/fixture` (`std_srvs/srv/Trigger`).  The mesh loader and project root
are installed by scenario load and preserved for the node lifetime, so a real
schema-valid mesh scenario applies real geometry (never a misleading apply
timeout).  Foreign-namespace objects are outside fixture ownership: their raw
ids are preserved for namespace isolation/leak checks but their geometry is
never parsed, so a malformed foreign object cannot block fixture readiness
(while malformed `sim_fixture/*` objects still fail closed).  Startup
`heartbeat_period` and `start_deadline_s` are validated as finite positive
values during construction.  While alive the node publishes a reliable
transient-local 5 Hz compact JSON heartbeat on
`/sim/status/planning_scene_fixture` (`std_msgs/msg/String`) with exactly:
`schema_version=1`, `state`, `scenario`, `owner="sim_fixture"`, `revision`,
`revision_digest`, monotonic `sequence`, finite `published_at`, declared-order
`owned_ids`, `target_source_id`, scalar
`target_handoff="pick_and_place/object_mesh"`, and `fixture_descriptor_sha256`.
Any service failure, malformed readback, geometry mismatch, status mismatch, or
deadline exhaustion fails closed to `state="FIXTURE_FAILED"` and the ready
service returns failure.

The fixture adapter never owns task objects: the downstream hardening
reconciler receives the full SRDF-derived eight-link touch set
(`xarm_gripper_base_link`, `left_outer_knuckle`, `left_finger`,
`left_inner_knuckle`, `right_inner_knuckle`, `right_outer_knuckle`,
`right_finger`, `link_tcp`) from the validated model contract, and creates the
canonical task-owned object from the Pick goal's `object_points` using the
declared `target_source_id` and `target_handoff` as the shared identity.

Run:

```bash
ros2 run tinker_sim_bridge fixture_planning_scene \
  --ros-args -p scenario_file:=$PWD/simulation/scenarios/qualification-moveit-plan-joint.json
```

### Tests

```bash
cd /home/tinker/tinker-sim/6.0.1
PYTHONPATH="$PWD/ros2_ws/src/tinker_sim_bridge:$PWD/simulation" \
  ./.venv/bin/python -m pytest -q tests/test_fixture_planning_scene.py \
  tests/test_scenario_runner.py
```

Pure contract/scenario tests run under the simulator Python 3.12 venv; the node
is imported only in the Humble Python 3.10 tests through a local import after
sourcing system ROS.  The Humble tests cover real `FixturePlanningScene`
construction in an isolated ROS domain/context: scenario load, exact owned-id
parsing/guard, publisher topic + RELIABLE/TRANSIENT_LOCAL depth-1 QoS, the
`/sim/ready/fixture` service, physics/apply/get clients, the 5 Hz timer, clean
destroy/shutdown, and constructor failure paths.  The real
model-bundle/touch-link gate (`test_real_model_bundle_touch_links_match_exported_fixture_set`)
rebuilds the current provisioned manifest through the Task 3 producer and
asserts its exact eight touch links equal the exported fixture set (it skips
only when the artifact tree is absent).

## Staged integrated OMPL overlay and typed integrated readiness (Task 6)

`integrated_ompl_manipulation.launch.py` composes the Task 3 model, Task 4
joint-state, and Task 5 fixture providers into the first integrated
OMPL/readiness boundary.  It is the only path that installs the staged
production planning/task overlay, and it is reached from
`manipulation.launch.py` with `planning_overlay:=true` (the default `false`
path preserves the legacy launch exactly).

### Exact staging order

The launch resolves everything synchronously in one `OpaqueFunction`, then
chains the staged providers through `OnProcessExit` handlers:

1. Validate environment, model-bundle manifest (structural fingerprint is a
   nonzero lowercase SHA-256), provider manifest (raw-byte digest + canonical
   self-hash), scenario declaration, planning-scene revision/digest/owned
   `sim_fixture/*` IDs, integrated mapping, and attempt paths.  Provider
   actions are constructed only after this validation succeeds.
2. Start the simulator safety/controller/gateway/RSP providers exactly once:
   `safety_supervisor`, `ros2_control_node`, `controller_reconciler` (one
   process requesting both `joint_state_broadcaster` and
   `xarm7_traj_controller`, gated on the safety supervisor ready parameter),
   `command_gateway`, `xarm_facade`, `gripper_facade`, `pan_tilt_facade`,
   `contract_guard`, `truth_evaluator`, `robot_state_publisher`.
3. Start `scenario_runner` with the exact expected identities.  On nonzero
   exit the launch shuts down; on exit 0 it writes the canonical compact
   scenario report atomically (sibling temp file + `os.replace`, final bytes
   digested for `scenario_report_sha256`) and starts `physics_ready_gate`.
4. `physics_ready_gate` parses the atomic report (exact scenario ID/seed/
   declaration digest, planning-scene mapping, integrated mapping, model/
   provider identities, `final_simulation_state="STATE_PLAYING"`, an accepted
   operation with integer `state=1` and `boundary="PHYSICS_READY"`), atomically
   writes `physics-ready.json`, publishes transient reliable
   `state="PHYSICS_READY"` status, and serves `/sim/ready/physics`
   (`std_srvs/srv/Trigger`).  The launch waits for that typed service before
   starting the production planning-only launch.
5. After `/sim/ready/physics`, start the production planning-only overlay
   (`manipulation_planning_task_only.launch.py` with
   `start_move_group=true,start_task_server=false,
   execution_profile="sim_ompl"`) and the fixture adapter.  The fixture
   adapter gates on `/sim/ready/physics`, applies one atomic diff, confirms
   readback, and serves `/sim/ready/fixture`.
6. After `/sim/ready/fixture`, start the production task-only overlay
   (`start_move_group=false,start_task_server=true,safety_required=true`,
   exact fixture revision/digest/owned IDs, scenario status path, exact
   scenario identities, model fingerprint, fixture descriptor digest) and
   `integrated_readiness`.
7. `integrated_readiness` performs live graph/type/cardinality probes and
   fresh message checks, independently of status topics, and publishes the
   pass/fail JSON on `/sim/status/integrated_manipulation`.

### Provider manifest

`integration/provider-manifest.json` (schema version 1) records the four
explicit sections `persistent_nodes`, `one_shot_processes`,
`controller_resources`, and `publishers`, each entry carrying concrete
`owner`, package/executable, fully qualified node, `cardinality`, and
`evidence`.  The manifest records `provider_manifest_sha256` (the canonical
self-hash) and the exact `cardinality_source`.  The launch passes the raw-byte
SHA-256 digest of the manifest file separately to every consumer so a changed
provider set is detected against unchanged bytes.

### Readiness evaluator and Python split

`integrated_readiness.py` is ROS-free at import time.  It defines
`build_integrated_mapping()` (the full runtime readiness contract:
`report_revision`, typed `actions`, `services`, `publishers`, eight
`joint_names`, eight `touch_links`, `tf`, `controller_resources`,
`final_simulation_state`) and `public_integrated_mapping()`, which returns the
production-canonical public report `integrated` field
`{"execution_profile": "sim_ompl"}` exactly as the shipped `pick_and_place`
canonical parser requires.  The public `scenario-runner.json` report carries
the one-key `integrated` mapping and its exact digest; the full runtime
contract is carried separately as `runtime_contract_sha256` / `integrated_mapping`
evidence in the physics gate and readiness node.  The module also defines the
immutable `ReadinessReport` result type and
`evaluate_integrated_readiness(snapshot, contract) -> ReadinessReport`.  The
evaluator checks model preflight, the parsed shared-report `PHYSICS_READY`
evidence (including the external `scenario_report_sha256` and mapping identity
digests), exact joint-state content/stamp/age/source, the composed
`base_link -> link_tcp` TF chain, active trajectory-controller
logical-resource identity, fresh operator input and effective safety output,
every typed action (including Cartesian/Joint/Fold) with graph-observed
goal-service types, every typed MoveIt/controller/gate service, the typed
`/arm_joint_service` (`tinker_arm_msgs/srv/ArmJointService`) with exactly one
`/pick_and_place` server, exact canonical fixture status fields, full
scenario/planning-scene/integrated mapping and digest agreement,
publisher count/source/type/QoS metadata for every typed publisher
(including `/isaac_joint_commands` and `/sim/controller/gripper_commands`),
provider-manifest resolved/live agreement, semantic model/kinematics equality,
and initial collision state (published by `/tinker_isaac_gateway`).

`integrated_readiness_node.py` (`class IntegratedReadiness`, `main()`) is the
only module that imports `rclpy`/message types.  It probes actions via the
`{endpoint}/_action/send_goal` service pattern (Humble rclpy has no
action-introspection API), records graph-observed goal-service types, maps
services to their serving nodes, steps `/controller_manager/list_controllers`,
checks joint and Boolean sample content/freshness plus real publisher QoS
(reliability/durability from `PublishersInfo`; depth is compared when a
publisher actually reports it — the canonical command topic
`/isaac_joint_commands` expects RELIABLE/KEEP_LAST depth 50, matching the
gateway's actual QoS in `command_gateway.py`), composes the multi-hop TF chain with
`tf2_ros.Buffer` + `TransformListener`, reconciles the provider manifest
against the live graph, and publishes `std_msgs/msg/String` JSON on
`/sim/status/integrated_manipulation` at `check_period_s`; any check failure
publishes `fail` and (with `fail_exit_s>0`) exits nonzero.

`readiness_waiter.py` is the installed, testable readiness waiter used by the
launch for the `/sim/ready/physics` and `/sim/ready/fixture` gates
(`python3 -m tinker_sim_bridge.readiness_waiter`).  It bounds service
discovery, the call, the response, and the total process lifetime by the
deadline, services the `call_async` future with
`rclpy.spin_until_future_complete`, and exits 0 only for a typed Trigger
`success=true` response.

### Run

```bash
./scripts/launch-humble integrated-ompl \
  model_bundle_manifest:="$PWD/outputs/ompl-overlay/model-bundle/model-bundle.json" \
  provider_manifest_path:="$PWD/ros2_ws/src/tinker_sim_bridge/integration/provider-manifest.json"
```

### Tests

```bash
cd /home/tinker/tinker-sim/6.0.1
./scripts/build-humble-overlay
PYTHONPATH="$PWD/ros2_ws/src/tinker_sim_bridge:$PWD/simulation" \
  ./.venv/bin/python -m pytest -q \
  tests/test_integrated_readiness.py \
  tests/test_integrated_ompl_launch_contract.py \
  tests/test_qualification_scenario_schema.py
source /opt/ros/humble/setup.bash
source /home/tinker/tinker-sim/6.0.1/.ros-vendor/humble/local_setup.bash
source /home/tinker/tk25_ws/install/setup.bash
export PYTHONPATH="$PWD/ros2_ws/src/tinker_sim_bridge:$PWD/simulation:$PYTHONPATH"
python3 -m pytest -q \
  tests/ros_humble/test_readiness_waiter.py \
  tests/ros_humble/test_composed_tf.py \
  tests/ros_humble/test_graph_evidence.py \
  tests/ros_humble/test_live_graph_probe.py \
  tests/test_scenario_runner.py
```

The pure evaluator and AST-based launch-contract tests run under the simulator
CPython 3.12 venv (no ROS import needed).  The Humble node/graph tests
(real Trigger waiter, composed multi-hop TF, graph type/source/QoS observation,
legacy scenario-runner regression) run under system Python 3.10 with a sourced
Humble environment.  The live graph probe imports `rclpy` only inside the test
after a sourced Humble environment and skips cleanly when the integrated
overlay is not running; it uses a uniquely-named observer with no status
publisher so it never perturbs a running overlay's cardinality evidence.

## Deployment gateways

The remaining bridge nodes implement hardware-parity gateways and controllers
for the external Humble boundary (base, xArm, gripper, pan-tilt, command
gateway, safety supervisor, contract guard, truth evaluator, scenario runner,
audio fixtures).  See the repository root README for the deployment model.

## Acceptance contract provenance (Task 8)

`tests/test_provenance.py` was extended (not replaced) with a deterministic
acceptance-contract provenance suite that recomputes every derived hash and
contract in the committed `integration/ompl-overlay-contract.json` from the
real source and artifacts.  It fails on mutations: stale `setup.py`/`package.xml`
registrations, missing data files, provider-manifest drift, wrong model/current
artifact, wrong endpoint/type/source/cardinality/QoS, wrong fixture/order/
handoff, wrong compatibility booleans, Task 7 action-client scope, task-range
boundary/commits, fixture status publication, artifact path policy, top-level
stable hashes, raw-colcon text, dirty-policy violations, and source-lock files
being prematurely included.

The static acceptance evidence is clean-checkout reproducible with no
gitignored `outputs/`/`artifacts/` dependency: the 16-check preflight runs the
real unmodified `preflight_manifest` against a self-contained reconstructed
Task 3-compatible project root (committed source + pinned git objects + a
legacy `current.json` selecting the reproduced canonical URDF), requiring
`ready=true`, all 16 checks including `artifact_identity`, and the stable
preflight hash.  The real `artifacts/robot/tinker2/current.json` is a separate
provisioned-host runtime-readiness diagnostic (stale selection fails; absent
selection is reported `not_provisioned`).  A temporary copy-install/wheel test
proves the symlinked acceptance contract and scenarios install as real
byte-identical files, and a clean-checkout regression seam (`git clone` of the
tracked tree) collects the exact 64-node `Task8OMPLOverlayProvenanceTest` class
(canonical node-set SHA-256, so a deleted/added test fails even at a preserved
count) and executes it under a machine-readable JUnit XML, requiring total=64,
failures=0, errors=0, skipped=4, the exact four host-runtime diagnostic skips,
and their reason categories.  The collection/JUnit acceptance checks are
factored into one deterministic validator applied to both the real clone output
and realistic mutated fixtures (delete one node, rename+add substitution, an
unrelated skip, a removed expected skip, a wrong skip reason, a duplicate
testcase, a failure/error count, multiple suites), and the JUnit structure is
tightened to exactly one `<testsuite>` with 64 unique `<testcase>` entries.
The reconstructed `tools/tinker_sim_deploy` resolver is materialized from
immutable git objects at the recorded simulator implementation identity (never
the live working tree).  Because the test module pre-imports
`tinker_sim_deploy` from the live checkout, the pinned-resolver proof runs in a
fresh isolated subprocess (`-I`, no inherited module cache) that records the
loaded `tinker_sim_deploy.runtime.__file__` from the materialized temp root and
runs the real Task 3 preflight, with a temp decoy working-tree package proving
the pinned path wins precedence (positive) and is load-bearing (negative); no
test writes a tracked active-checkout file.  The fixture status field contract
is asserted against an independent 12-field literal.  The
pre-existing uv environment provenance failure (installed `uv 0.12.0` vs pinned
`uv 0.10.8`) is unchanged and is not masked: it is an environment failure, not a
code failure.

As part of this task the package metadata is completed:

- `setup.py` registers the `readiness_waiter` console script (the installed
  module already existed and is invoked as `python3 -m
  tinker_sim_bridge.readiness_waiter` by the integrated launch) and verifies
  every launch/config/integration asset and every `main()`-bearing module is
  registered.
- `package.xml` declares the direct message/service/action and runtime package
  dependencies with no transitive-import assumptions (`xarm_moveit_config` is
  deliberately not listed: it is consumed transitively through the
  `mobile_bringup` production launch, which declares it directly).  The missing
  direct `robot_localization` dependency was added: the shipped
  `launch/navigation.launch.py` launches `robot_localization/ekf_node`.
