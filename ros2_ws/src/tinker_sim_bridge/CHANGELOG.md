# Changelog

All notable changes to this package are documented here.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

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
