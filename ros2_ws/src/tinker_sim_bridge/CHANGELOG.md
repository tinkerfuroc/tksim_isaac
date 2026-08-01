# Changelog

All notable changes to this package are documented here.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added (Task 6: staged integrated OMPL overlay + typed integrated readiness)

- `launch/integrated_ompl_manipulation.launch.py`: the staged overlay that
  composes the Task 3/4/5 providers into the first integrated OMPL/readiness
  boundary.  One `OpaqueFunction` synchronously validates the model-bundle
  manifest (64-hex structural fingerprint), provider manifest (raw-byte digest
  + canonical self-hash), scenario declaration, planning-scene
  revision/digest/owned `sim_fixture/*` IDs, and attempt paths, then chains
  the providers in the exact 7-step order via `OnProcessExit`: safety/
  controller/gateway/RSP providers -> `scenario_runner` -> `physics_ready_gate`
  -> `/sim/ready/physics` wait -> production planning-only launch (`start_move_
  group=true,start_task_server=false,execution_profile="sim_ompl"`) + fixture
  adapter -> `/sim/ready/fixture` wait -> production task-only launch
  (`start_task_server=true,safety_required=true` + exact fixture/scenario/
  model identities) + `integrated_readiness`.  Registers the launch in
  `data_files`.
- `tinker_sim_bridge/physics_ready_gate.py`: `PhysicsReadyGate` +
  `main()`.  After the scenario runner's successful exit it reads the atomic
  `scenario-runner.json`, computes `scenario_report_sha256` from the final
  bytes, parses the shared canonical report (exact scenario ID/seed/declaration
  digest, planning-scene mapping, integrated mapping, model/provider
  identities, `final_simulation_state="STATE_PLAYING"`, accepted operation
  with integer `state=1` and `boundary="PHYSICS_READY"`), atomically writes
  `physics-ready.json`, publishes transient reliable `state="PHYSICS_READY"`
  status on `/sim/status/physics_ready`, and serves `/sim/ready/physics`
  (`std_srvs/srv/Trigger`).  Registers the `physics_ready_gate` console
  script.
- `tinker_sim_bridge/integrated_readiness.py`: ROS-free at import time.
  `build_integrated_mapping()` returns the canonical composition mapping
  (report revision, typed actions/services/publishers, eight joint names,
  eight touch links, TF, controller resources, final simulation state); the
  immutable `ReadinessReport` dataclass and `evaluate_integrated_readiness(
  snapshot, contract)` implement the fail-closed typed readiness evaluator
  (model preflight, shared-report PHYSICS_READY evidence, joint content/stamp/
  age/source, TF, active trajectory-controller resource, operator/safety
  inputs, every typed action/service, `/arm_joint_service`, canonical fixture
  status, mapping/digest agreement, provider-manifest agreement, semantic
  model, initial collision state).
- `tinker_sim_bridge/integrated_readiness_node.py`: `IntegratedReadiness` +
  `main()`.  Live graph probes (actions via the `/_action/send_goal` service
  pattern, per-node service map, `list_controllers` step, joint and Boolean
  sample freshness, fixture status, provider-manifest bytes, semantic model,
  collision) and 5 Hz compact JSON publication on
  `/sim/status/integrated_manipulation` (reliable transient-local); publishes
  `fail` and exits nonzero on any failure.  Registers the
  `integrated_readiness` console script.
- `integration/provider-manifest.json`: schema-1 four-section manifest
  (`persistent_nodes`, `one_shot_processes`, `controller_resources`,
  `publishers`) recording every provider with concrete owner/package/
  executable/node/cardinality/evidence, the exact `cardinality_source`, and
  `provider_manifest_sha256` (canonical self-hash).  Registers the file in
  `data_files`.
- `tests/test_integrated_readiness.py`: 17 pure evaluator tests (ready pass and
  every fail-closed branch) with local `ready_snapshot()`,
  `provider_manifest()`, `contract()`, `mismatching_snapshot(**overrides)`.
- `tests/test_integrated_ompl_launch_contract.py`: AST-based launch contract
  tests with local `load_launch_source`/`resolve_launch_graph`/
  `assert_allowlisted_launch_graph` helpers: node/executable allow-list, exact
  staging sequence, production overlay inclusion, provider-manifest schema/
  self-hash, and `manipulation.launch.py` default preservation.
- `tests/ros_humble/test_live_graph_probe.py`: imports `rclpy` inside the test
  body; skips cleanly when the integrated overlay is not running and otherwise
  verifies every typed action/service/publisher endpoint against the live ROS
  graph.
- `scenario_runner` canonical report: accepts explicit expected
  scenario/planning-scene/integrated-mapping/model/provider identities,
  verifies them against the unchanged scenario bytes, and writes the canonical
  compact report atomically (sibling temp file + `os.replace`), printing
  `scenario_report_sha256`.  The legacy non-overlay path preserves the previous
  operation output.
- `scripts/launch-humble`: new `integrated-ompl` module mapping to
  `integrated_ompl_manipulation.launch.py`.
- Declared `tf2_msgs`, `tinker_arm_msgs`, `mobile_bringup`,
  `moveit_ros_move_group`, and `pick_and_place` exec dependencies used by the
  overlay/probe/readiness.

### Changed (Task 6)

- `manipulation.launch.py` gains a `planning_overlay` argument (default
  `false`): when true, `_resolve` returns the integrated overlay include; the
  default path is unchanged.  Also declares `model_bundle_manifest` and
  `provider_manifest_path` arguments.
- `scenario_runner.py` serializes the canonical report with compact
  `separators=(",", ":")` so digest and readiness consumers see canonical
  bytes.

### Added

- Atomic fixture PlanningScene adapter (`fixture_planning_scene`): applies one
  atomic replacement diff of all desired `sim_fixture/*` objects plus every
  stale existing `sim_fixture/*` id, gates on `/sim/ready/physics`, reads back
  the scene, confirms readback/status, serves `/sim/ready/fixture`
  (`std_srvs/srv/Trigger`), and publishes a reliable transient-local 5 Hz compact
  canonical JSON heartbeat on `/sim/status/planning_scene_fixture`.  Registers
  the console script and declares the direct interface dependencies used by the
  node (`moveit_msgs`, `shape_msgs`, `std_srvs`).
- Pure ROS-free fixture contract (`fixture_contract.py`): typed immutable
  `CollisionObjectSpec`, `PlanningSceneDiffPlan`, and `Confirmation` types plus
  `revision_digest`, `parse_required_fixture_owned_ids`,
  `build_atomic_revision_diff` (namespace-scoped ADD+REMOVE in one diff), and
  `confirm_fixture_revision` (fail-closed readback/status consistency).
- ROS-free adapter (`fixture_planning_scene.py`): `fixture_to_specs`,
  `fixture_owned_ids`, `fixture_descriptor`/`fixture_descriptor_sha256`, and the
  canonical shared fixture-status mapping with exactly `schema_version`,
  `state`, `scenario`, `owner`, `revision`, `revision_digest`, monotonic
  `sequence`, finite `published_at`, `owned_ids`, `target_source_id`, scalar
  `target_handoff="pick_and_place/object_mesh"`, and `fixture_descriptor_sha256`.
- Scenario schema version 2 optional `planning_scene` with strict validation
  (nonempty revision, canonical digest input, frame `base_link`, unique
  `sim_fixture/*` ids, finite poses, positive primitive dimensions, hashed
  absolute/declared mesh assets, declared target identity, exact handoff, and
  explicit `enter_collision_bodies` for diagnostics).
- Three public qualification scenarios
  (`qualification-moveit-plan-joint`, `qualification-moveit-plan-pose`,
  `qualification-moveit-plan-blocked`) sharing the pedestal and public-target
  identity, with the blocked scenario adding `sim_fixture/plan_blocker`.
- Pure contract/scenario tests (`tests/test_fixture_planning_scene.py`) and a
  planning-scene scenario compilation test in
  `tests/test_scenario_runner.py`.

### Changed

- Fixture readback now proves geometry, not just IDs: `_get_extract` preserves
  full `CollisionObject` payloads and the `GetPlanningScene` request explicitly
  requests `WORLD_OBJECT_NAMES | WORLD_OBJECT_GEOMETRY`; the node compares the
  readback geometry (frame, poses, primitive type/dimensions, mesh
  vertices/triangles) against the declared fixture before serving ready, and
  rejects duplicate `sim_fixture/*` ids.
- Mesh-declared fixtures are no longer applied as empty `Mesh()` payloads: a
  narrow ROS-free stdlib STL (binary/ASCII) and OBJ parser
  (`parse_mesh_bytes`), asset resolution + SHA-256 verification + scale
  application (`load_mesh_asset`), and real `shape_msgs/Mesh` emission are
  implemented; only `.stl`/`.obj` extensions are supported and any other
  extension is rejected at scenario validation.
- `MODEL_CONTRACT_TOUCH_LINKS` is now imported from the validated Task 3
  `model_contract.TOUCH_LINKS` (single authoritative source) instead of an
  independent literal.
- Scenario validation requires mesh assets to exist with content SHA-256
  matching the declaration and requires `target_source_id` to name a fixture
  that enters the collision-body/owned set (a diagnostic with
  `enter_collision_bodies=false` is rejected).
- Real `FixturePlanningScene` constructor tests run under Humble in an isolated
  ROS domain/context and verify scenario load, owned-id guard, publisher QoS,
  ready service, physics/apply/get clients, timer, clean shutdown, and
  constructor failure paths; a real model-bundle touch-link gate proves the
  provisioned contract's eight touch links equal the exported fixture set.
- The mesh loader and project root installed by scenario load are preserved for
  the node lifetime (the post-load reset that erased them is removed), so a real
  schema-valid mesh scenario applies real geometry; the mesh ready-loop now
  proves apply/readback/confirm to `FIXTURE_READY` and mesh constructor
  failures (missing/unsupported/hash-mismatch) fail immediately at
  construction instead of after a misleading apply timeout.
- `_get_extract` preserves raw foreign-namespace ids for isolation/leak checks
  but parses/validates canonical geometry only for `sim_fixture/*` objects, so a
  malformed foreign object can no longer block fixture readiness; malformed
  `sim_fixture/*` objects still fail closed.
- The `GetPlanningScene` component bitmask
  (`WORLD_OBJECT_NAMES | WORLD_OBJECT_GEOMETRY`) is asserted by tests instead of
  being left to the server default.
- `heartbeat_period` and `start_deadline_s` are validated as finite positive
  during real construction (NaN/Inf/nonpositive rejected rather than hanging
  phases), and ASCII STL parsing now respects facet boundaries (exactly three
  finite vertices per facet; vertices outside/unterminated/over-filled facets
  rejected).

- Integrated eight-joint state contract: `drive_joint` is now state-only
  (`position`/`velocity`/`effort`, zero command interfaces) in both the checked-in
  `config/tinker_topic_control.ros2_control.xacro` and the live controller
  description produced by `tools/tinker_sim_deploy/runtime.py:
  topic_control_description`, so the `robot_description` supplied to
  `controller_manager` and `robot_state_publisher` carries the same state-only
  joint.  `xarm7_traj_controller` keeps the seven arm joints and the gripper
  command path stays `/sim/controller/gripper_commands`.
- Pure ROS-free contract helpers in `contract_guard.py`:
  `evaluate_joint_state_sample` (exact eight names, cardinality one, source
  `joint_state_broadcaster`, nonzero stamp, finite arrays, bounded age),
  `evaluate_integrated_cardinality` (graph publisher cardinality/source),
  `evaluate_robot_description_contract`, `evaluate_xacro_contract`, and
  `evaluate_joint_state_evidence_pair` (compares checked-in xacro and live
  `robot_description` evidence).  All return complete `ready`/`reasons`/
  `observed` mappings with no global clock or implicit topic name.
- ROS Humble live probe executable `joint_state_probe`: reads the
  `/controller_manager` `robot_description` parameter, parses the checked-in
  xacro, subscribes to `/joint_states`, reads `/joint_states` publisher graph
  metadata, evaluates the pure contract helpers, and publishes compact JSON on
  `/sim/status/joint_state_contract` (latched, reliable).  Registers the
  console script and declares `ament_index_python` + `rcl_interfaces` exec
  dependencies.
- Probe fail-closed hardening: `evaluate_probe_verdict` aggregates evidence with
  sample readiness unconditional (no sample / stale / later loss → FAIL, and a
  latched PASS is replaced by the current failure status); `step_service`
  provides a bounded recoverable async service state machine for both the
  `GetParameters` and `ListControllers` calls (exception / malformed / pending →
  publish FAIL and retry); `derive_logical_joint_state_publishers` proves the
  publisher source through `/controller_manager/list_controllers` (exactly one
  active `joint_state_broadcaster`) instead of relabeling any controller-manager
  publisher; `evaluate_clock_domain` verifies probe/controller `use_sim_time`
  agreement plus active nonzero `/clock`; `evaluate_sample_freshness` adds a
  wall-clock sample watchdog; and `evaluate_joint_state_sample` records a
  probable `use_sim_time` clock-domain-mismatch reason for epoch-scale
  header/now differences.  Probe status is serialized with compact canonical
  separators.
- Real selected production artifact contract test
  (`test_real_selected_artifact_state_only_drive_joint_contract`) resolves
  `current.json` through the authoritative Task 3 resolver and feeds the actual
  `robot.urdf` bytes through `topic_control_description` and the contract
  evaluators (skips only when the gitignored artifact tree is absent); synthetic
  fixtures remain only for negative/unit cases.
- Probe-node tests under the Humble Python 3.10 runtime
  (`tests/test_joint_state_probe_node.py`): no sample → FAIL, fresh sample →
  PASS, prior PASS then lost/stale sample or graph/controller failure → FAIL,
  latched PASS replaced by current FAIL, clock-domain mismatch, and transient
  service failure publication.

- Canonical manipulation model-bundle producer (`model_bundle`) and bounded
  preflight validator (`model_preflight`), with pure ROS-free
  `model_contract` semantics matching the production `xarm_moveit_config`
  consumer.  Registers the `model_bundle` and `model_preflight` console
  scripts, declares the direct interface dependencies used by the model
  overlay, and documents the schema/producer/preflight contract in the README.
- Deterministic arm+gripper joint-limit synthesis (`model_limits`): merges the
  committed `xarm7/joint_limits.yaml` and `xarm_gripper/joint_limits.yaml`
  sources into the canonical eight-joint `joint_limits` artifact that is itself
  hashed into the manifest, making the plan-acceptance bundle reproducible from
  committed inputs.  Registers the `model_limits` console script and the
  `scripts/model-bundle-sim-urdf.sh` helper.
- Shared authoritative current-artifact resolver
  (`tinker_sim_bridge.current_artifact`) that dispatches both the legacy
  unversioned pointer/schema-2 manifest and the schema-4 publication shape in
  `tools/tinker_sim_deploy/runtime.py`, used identically by runtime selection,
  bundle resolution, and preflight identity.

### Fixed

- Probe parameter parsing now compares against the real Humble
  `rcl_interfaces.msg.ParameterType` constants (`PARAMETER_STRING` /
  `PARAMETER_BOOL`) instead of non-existent `ParameterValue.PARAMETER_*`
  attributes, so the live probe can actually read `robot_description` and
  `use_sim_time` from a real `GetParameters.Response`; wrong-type/missing-value
  mutations fail closed and recover on the next fresh response (covered by
  node tests that feed genuine `ParameterValue` objects through the production
  `_step_parameters` extraction path).
- Successful `GetParameters`/`ListControllers` evidence is no longer latched
  forever: `step_service` re-polls both services on a bounded wall-clock TTL
  (30 s default) and revokes the success latch once the evidence expires, so a
  controller_manager restart, renamed/inactive broadcaster, or changed
  `robot_description` is re-verified on a bounded cadence and cannot be masked by
  stale attribution/description evidence.  The probe treats stale parameter
  evidence as unavailable and reports typed FAIL until a fresh response arrives.
- `step_service` now tracks an in-flight request deadline and, on timeout,
  abandons the future, resets the client, and retries on a bounded cadence
  without busy-looping or leaking a client; an old generation can never satisfy a
  new request (`succeeded_at` / `started_at` tracked per state).
- `_endpoint_label_from_info` normalizes a root-namespace publisher to
  `/name` (no `//name`), matching the pure attribution helpers; the standalone
  exact `/joint_state_broadcaster` branch is now reachable in the probe (a
  controller-manager-hosted publisher still requires fresh exact active
  controller proof).
- Probe constructor no longer shadows rclpy's internal `Node._parameters`
  parameter store: the `joint_state_probe` service-state attributes are now
  `_parameters_state` / `_controllers_state`, so the installed probe constructs
  cleanly instead of exiting with `ParameterNotDeclaredException('xacro_path')`
  after its `declare_parameter` calls were silently clobbered mid-`__init__`
  (caught by the clean-environment `use_sim_time:=true` installed probe smoke).
- `test_runtime_transformer_is_shared_and_arm_only` updated to the eight-joint
  contract: the live transformer now emits exactly one state-only `drive_joint`
  alongside `joint1`..`joint7`, verified through the complete real robot URDF.
- Preflight artifact-identity gate now fails closed: every fully-ready report
  contains a successful `artifact_identity` check, derived from the explicit
  project root, the manifest simulator artifact path, or the environment, and
  it is never silently omitted.
- `validate_bundle_manifest` now validates `normalization.groups` content and
  the preflight recompute cross-checks exact selected-link ordering against the
  resolved graph, matching the production consumer.
- `scripts/build-humble-overlay` now forces `MAKEFLAGS='-j2 -l2'` so a preset
  higher value cannot escape the mandatory memory bound.
- Dropped inert package dependencies (`moveit_msgs`, `shape_msgs`,
  `tinker_arm_msgs`) not used by any source in this package.

### Fixed (copied-install boundary + selector/identity strictness)

- Shared-resolver discovery now prefers the authoritative simulator project
  root supplied by the caller or derived from the manifest artifact path
  (`current_artifact._shared_resolver`), so a genuine copied ROS install
  outside the checkout can locate `tools/tinker_sim_deploy/runtime.py` without
  `TINKER_SIM_ROOT`; discovery stays fail-closed when neither a project root
  nor an authoritative artifact tree is available.
- Legacy dispatch in `tools/tinker_sim_deploy/runtime.py` now accepts only the
  deployed legacy shapes (unversioned pointer plus absent/schema-2 manifest);
  hypothetical schema-1/3/other manifest values are rejected instead of being
  treated as legacy, while schema-4 strict validation is unchanged.
- Preflight artifact identity is proven against the authoritative
  `current.json` selection by SHA-256 bytes: an outside-tree simulator artifact
  with identical bytes passes, stale/different bytes fail, and an unresolvable
  authoritative root returns a typed not-ready identity check (never
  `ok=true`/`not applicable`).
