# Changelog

All notable changes to this package are documented here.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Fixed (Task 48: joint4 live trajectory effort saturation)

- The simulator arm actuator now applies an explicit per-joint PhysX effort
  envelope of `[50, 50, 30, 50, 30, 20, 20]` Nm. This raises only joint4 from
  the inherited vendor-URDF 30 Nm cap to 50 Nm while preserving every other
  joint tier. The override addresses the live free-space FJT failure where
  joint4 remained saturated for about 72% of the action and alone produced
  0.180 rad RMS tracking error; the trajectory and its 0.01 rad terminal / 0.05
  rad RMS qualification thresholds remain unchanged.

### Fixed (Task 43: live OMPL actuator settling and truth teardown)

- Spawned Tinker articulations now override imported joint drives to force mode
  before Isaac Lab initializes the articulation, while preserving the existing
  arm, head, gripper, and wheel actuator gains and limits. This addresses the
  acceleration-drive gravity sag that exceeded the xArm trajectory controller's
  0.01 rad terminal tolerance during the live free-space FJT gate.
- Raw physics-truth persistence now belongs to the Humble truth evaluator: the
  Isaac gateway publishes strict finite canonical JSON only, the evaluator uses
  RELIABLE delivery and writes the validated raw frame plus evaluated record in
  the same callback, and both manipulation launches pass a separate
  `raw_jsonl_path`. This eliminates the cross-process final-frame teardown race
  without weakening exact raw/evaluator correlation.

### Added (integrated qualification executor driver — Task 8 fix round 3: observe live integrated providers)

- The integrated launch now owns the `base_link -> livox360` static transform
  for qualification runs only: one `tf2_ros/static_transform_publisher` named
  `livox360_static_tf` (xyz `0.12 0.0 0.25`, identity quaternion), matching
  `navigation.launch.py`.  It is spelled `launch_ros.actions.Node(...)` so the
  immutable Task-2 launch-graph allow-list (which permits `tf2_ros` only for
  the staging gates) keeps accepting the overlay; the launched executable is
  the literal `tf2_ros/static_transform_publisher`.  Ordinary
  `manipulation-core` runs leave the frame unowned.
- Qualification-only development LiDAR: `validation/run_sim.py` adds
  `build_occupancy_from_planning_scene` (pure deterministic 2-D occupancy map
  at 0.05 m / 60 m half-extent from committed scenario PlanningScene box
  footprints), `qualification_occupancy` (map only for scenarios with box
  fixtures; None for empty/free-space), and `gateway_lidar_enabled` (development
  lidar only for `navigation-parity` or `manipulation-core --qualification`).
- The executor driver (`validation/integrated_gate_executor_driver.py`) now
  observes live integrated providers: a driver-owned `_LiveProviderObserver`
  node on the executor's private-context spinner; `_call_service_with_spinner`
  (async future + explicit `_spin_once`) for every controller/graph service
  query because rclpy delivers responses to the spinner's wait set, not the
  calling client's node; driver-side `_goal_id_hex` normalization for real
  rclpy UUID goal handles; operator-baseline re-publish inside the readiness
  snapshot; and `declare_parameter` + `add_on_set_parameters_callback` for
  `/pick_and_place.post_grasp_lift_m`.  No reference to executor internals
  (`_observed_graph`/`_tf_lookup`/`_latest_environment_cloud`/
  `_native_gripper_goal_count`/`ParameterClient`/`server_is_available`).
- Tests: `tests/test_integrated_gate_executor_driver.py` grows to 36 ROS-free
  tests (hermetic double-parameter pure layer, Option A+ occupancy and gateway
  profile resolution, bundle committed-identity fail-closed); new
  `tests/ros_humble/test_integrated_gate_executor_driver_providers.py` (22
  sourced-Humble tests: live readiness, controllers, FJT digest, native
  gripper, parameter set/read-back, negative-mutation fail-closed, and cancel
  presend).  No build, no live Isaac/ROS, no GPU-process change, no cuMotion.

### Fixed (Task 1 fix round 1: align integrated fixture geometry)

- Aligned the physical (bottom-origin USD) roots with the PlanningScene
  (center-origin) poses for all eight E-stage pick-place scenarios: the 0.08 m
  cube physical root stays at z 0.60 while its PlanningScene center is now z
  0.64 (root + committed asset center offset 0.04); the occupied-place occupant
  physical root is now z 0.60 with its PlanningScene center at z 0.64; the
  pedestal root/center z 0.0/0.30 are unchanged.
- Added the declared physical + PlanningScene place-support pedestal
  (`qualification_place_pedestal` / `sim_fixture/place_pedestal`) to every
  E-stage scenario: fixed, owner `sim_fixture`, role `support`, region
  `place-region`, physical root `[0.85, 0.0, 0.0]`, PlanningScene box center
  `[0.85, 0.0, 0.30]`, dimensions `[0.12, 0.12, 0.60]`, reusing the exact
  qualification-pedestal asset bytes/hash.  The owned-id declared order is
  pedestal, cube, place_pedestal, then the scenario-specific obstacle/occupant.
- Reworked the blocked-approach blocker so it deterministically rejects before
  contact: physical root `[0.65, 0.0, 0.70]`, PlanningScene center
  `[0.65, 0.0, 0.85]` for the exact 0.30 m cube, leaving 0.02 m clearance above
  the source cube top 0.68 while the declared target TCP z 0.72 lies inside the
  blocker.  The blocked-approach integrated goal now carries an explicit
  `approach: "top-down"` and `target_tcp_xyz` contract for Task 4.
- The qualification config `simulation/qualification/integrated-ompl.json`
  geometry contract now distinguishes the bottom-origin `object_root_xyz`
  (0.60) from the explicit `object_center_xyz` (0.64) and carries the
  deterministic `object_local_center_z` / `object_half_extent_xyz` and the
  place-support root/center/dimensions.
- Hardened `ScenarioDefinition._validate_integrated` cross-field checks:
  positive polarity requires `expected_negative` null; negative requires a
  complete race/timeout/forbidden-after-terminal contract; goal,
  expected_scene, and expected_physical shapes are validated; a present
  `back_positions` vector is exactly seven finite elements except the
  explicitly identified malformed-back negative (exactly six, required
  predicate `goal_rejected_pre_send`).
- Recomputed every affected immutable identity: E-stage planning-scene
  revision digests, fixture descriptor hashes, scenario declaration hashes,
  and the `ompl-overlay-contract.json` scenario identities/owned-ids.
- Added tests: physical-root to PlanningScene-center parity, place-support top
  == placement-object bottom, blocked-approach deterministic-rejection
  geometry, public-report literal key-set anchor, and D-stage no-spawned-task
  object documentation; removed the unused `canonical_sha256` helper.

### Added (Task 1: integrated OMPL scenario matrix)

- Adds the schema-v3 integrated config `simulation/qualification/integrated-ompl.json`
  (source-lock policies, model, strict seven-value pick-place `back_positions`,
  geometry contract, thresholds, execution contract with cuMotion disabled, and
  stages A-F covering all 17 scenarios).
- Defines the full 17-scenario integrated matrix in `simulation/scenarios/`:
  the three plan-only C scenarios (`qualification-moveit-plan-joint` /
  `-pose` / `-blocked`), six D execute scenarios (`qualification-moveit-execute-joint`
  / `-pose` / `-cartesian-retreat` / `-gripper` / `-cancel` / `-safety`), and
  eight E pick-place scenarios (`qualification-pick-place-positive`,
  `-blocked-approach`, `-unreachable-grasp`, `-malformed-back`, `-cancel-approach`,
  `-cancel-transport`, `-safety-transport`, `-occupied-place`).  The three
  existing C scenarios retain their prior planning-scene identity, geometry,
  digests, and semantics; only the schema-v2-validated `integrated` mapping was
  added to each scenario declaration.
- `ScenarioDefinition` now loads and validates the `integrated` mapping and the
  identity-free `declaration`; `standard_operations` carries the immutable
  `scenario`, `planning_scene`, and `integrated` mappings into the final
  `PHYSICS_READY` operation; `scenario_runner` builds the canonical report via
  `build_canonical_report`.
- The public `scenario-runner.json` report carries exactly the one-key
  `integrated` mapping `{"execution_profile": "sim_ompl"}` (and its digest); the
  full per-scenario `integrated` mapping is bound by the scenario declaration
  SHA-256 and preserved in separate readiness/executor evidence.
- Registers all 14 new scenario package-share symlinks and `setup.py`
  `_scenario_sources` entries; the `ompl-overlay-contract.json` scenarios section
  now lists all 17 scenarios with recomputed declaration digests.
- Adds the ROS-free shared `tests/qualification_test_helpers.py`
  (`load_test_scenario`, `expected_physics_ready_report`) and the 50-test
  `tests/test_integrated_qualification_config.py`; `test_provenance.py` and
  `test_qualification_fixtures.py` cover the 17-scenario identities and
  physical spawn/planning-scene contracts.

### Added (Task 8 fix round 4: isolate pinned-resolver verification)

- The pinned-resolver independence proof is now executed in a fresh isolated
  subprocess (`python -I`, no inherited `tinker_sim_deploy*` module cache, no
  `PYTHONPATH`/user-site contamination).  The parent test module pre-imports
  `tinker_sim_deploy` from the live working tree, so Python's module cache
  masked the earlier in-process dirty live-tree probe; the previous claim that
  "a dirty live-tree resolver edit is proven not to affect the reconstruction"
  has been replaced by a module-origin proof.  The child loads
  `tinker_sim_deploy.runtime` from the materialized temp root, records its
  resolved `__file__`, runs the real unmodified Task 3 preflight, and emits one
  machine-readable JSON (ready / 16 checks / `artifact_identity` / exact stable
  preflight hash / loaded module paths); a temp decoy working-tree package with
  a sentinel `runtime.py` proves the pinned path wins path precedence
  (positive) and is load-bearing (negative, the decoy is detected and fails).
  No test writes a tracked active-checkout file.
- The clean-checkout seam's collection/JUnit acceptance checks are factored
  into one deterministic `_check_collection_acceptance` validator applied to
  both the real clone output and realistic mutated fixtures (delete one node,
  rename/delete+add while preserving count, an unrelated skip, a removed
  expected skip, a wrong skip reason, a duplicate testcase, a failure/error
  count, and multiple suites), so the negative assertions exercise the exact
  acceptance code path rather than direct set comparisons.
- The JUnit structure validation is tightened: exactly one `<testsuite>`
  (multiple suites are rejected rather than summed), exact suite counters, 64
  unique `<testcase>` children, and no unexpected `failure`/`error`/`xfailure`
  child statuses, while the canonical 64-node set/hash is unchanged.

### Added (Task 8 fix round 3: lock clean-checkout acceptance coverage)

- The clean-checkout regression seam now locks the exact executed suite and
  skip set, not just the exit code: it collects the exact 64-node
  `Task8OMPLOverlayProvenanceTest` class in the clone (canonical SHA-256 over
  the JSON-canonical sorted node list, so a deleted/added test fails even when
  the count is preserved) and executes it under a machine-readable pytest JUnit
  XML, asserting total=64, failures=0, errors=0, skipped=4, the exact four
  host-runtime diagnostic skip names, and their reason categories — an
  unrelated static assertion broadened into a skip is rejected.  An inline
  mutation self-check demonstrates a changed node-set hash or skip set is
  rejected.
- The reconstructed Task 3 project root now materializes its
  `tools/tinker_sim_deploy` resolver from immutable git objects at the recorded
  simulator implementation identity (`git ls-tree` + `git show <commit>:<path>`),
  never from the live working tree; the pinned-blob mismatch/missing file
  fails closed.
- `fixture_contract.status_publication.fields` is asserted against an
  independently documented 12-field literal (canonical order/encoding), with the
  Task 5 function's own key set validated separately so a coordinated
  source+contract key-set change fails.
- A stale `current.json` selector is proven to fail the reconstructed preflight
  `artifact_identity` check (absent selector -> `not_provisioned`; present/
  matching -> pass; present/stale -> fail).

### Added (Task 8 fix round 2: separate static and runtime OMPL evidence)

- The 16-check preflight is now self-contained on a clean checkout: the
  provenance suite reconstructs a Task 3-compatible project root (committed
  source + pinned git objects + a legacy `artifacts/robot/tinker2/<id>/`
  `current.json` selector pointing at the reproduced canonical URDF + a
  self-contained `tools/tinker_sim_deploy`) and runs the real unmodified
  `preflight_manifest` against it, requiring `ready=true`, all 16 checks
  including `artifact_identity`, and the stable preflight hash with no
  gitignored `outputs/`/`artifacts/` dependency.
- The real `artifacts/robot/tinker2/current.json` is a separate
  provisioned-host runtime-readiness diagnostic: when present it must select
  bytes equal to the reproducible derivation (stale selection fails); when
  absent the host is reported `not_provisioned` as a typed skip reason, not a
  static acceptance failure.
- Top-level `evidence.preflight` in the acceptance contract now carries the
  load-bearing `stable_manifest_sha256` / `stable_preflight_sha256`; the raw
  host-scoped hashes are nested under `evidence.preflight.host_snapshot` with a
  stated non-reproducible scope.
- Task 7 action-client scope is AST-grounded in pinned Task 7 source at the
  recorded simulator implementation identity (exactly one `ActionClient`
  construction on `MOVE_ACTION="/move_action"`, no execute-trajectory /
  controller / task client).
- `repositories.simulator.task_range` records `boundary_subjects` (Task 3
  canonical-model-bundle producer -> Task 7 OMPL-smoke adjudication) and the
  exact ordered 13-commit list; the suite recomputes both and rejects any other
  13-commit pair.
- `fixture_contract.status_publication` fields/topic/type/source/QoS/rate are
  recomputed from the Task 5 source and constants.
- Every model artifact `path_relative` is verified against the recorded
  source/`production_source_commits`/`source_evidence`/outputs policy.
- The provisioned-manifest cross-check and the real installed-prefix check emit
  explicit non-provisioned / build-command skip reasons instead of silently
  passing; the mandatory reconstruction and a new temporary copy-install/wheel
  test carry the portable gates.
- The pinned v1 canonicalizer is loaded by deterministic temporary-package
  materialization (no `exec` source-text surgery).
- Joint-limit acceptance is semantic and toolchain-aware:
  `model_bundle.joint_limits_semantic_sha256` is the format-independent identity
  over the canonical semantic mapping, while the raw YAML bytes remain a
  toolchain-scoped runtime artifact hash with the serialization policy recorded.
- A clean-checkout regression seam (`git clone` of the tracked tree) requires
  every Task 8 static test to pass with no gitignored trees (nested invocation
  skipped via `T8_SEAM_ACTIVE`).
- `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` is documented as the defensive reproducible
  invocation (plugin discovery may auto-load `launch_pytest`, which can fail on
  hosts without `lark`), not a claim that collection fails on this exact host.

### Added (Task 8 fix round 1: make OMPL acceptance evidence reproducible)

- The provenance suite now recomputes the production-overlay evidence from
  immutable production git objects (`git show <recorded-commit>:<path>`), never
  from contract literals alone: the exact ordered 18 `DeclareLaunchArgument`
  names, literal-false `sim_ompl` compatibility booleans, literal-true task
  parameters, the six `strict_sim_inputs` keys, the production import/node/
  executable/controller allow-lists (from `launch_contract_helpers.py`,
  including `re`/`importlib`/`rviz2`), the `pick_and_place/object_mesh` handoff,
  and the smoke action-client scope.
- `production_overlay.provider_allowlist` is split into
  `production_allowlists` (exact production-enforced lists) and
  `simulator_overlay_provider_set` (derived from the committed provider
  manifest), removing the previously mixed/scope-confused hand-synthesized list.
- The model-bundle acceptance evidence is reproducible on a clean checkout:
  `model_bundle` gains `source_evidence` (committed simulator full-URDF source +
  provenance descriptor under `integration/model-bundle-r2/simulator_full_urdf/`),
  `production_source_commits`, `stable_manifest_sha256`, and
  `preflight_report.stable_sha256` (deterministic projections excluding
  host-absolute paths and `elapsed_ms`).  The provenance tests reconstruct the
  manifest/preflight into a temporary directory from committed inputs and fail
  (never skip) when required source evidence is absent.
- `model_preflight.stable_preflight_evidence` and
  `model_bundle.stable_manifest_evidence` expose the deterministic projections.
- `tinker_sim_bridge.scenario_resolver` adds a ROS-free, deterministic
  package-share fallback for qualification scenarios (rejecting byte
  disagreement), wired into `launch/integrated_ompl_manipulation.launch.py`.
- `setup.py` registers `integration/ompl-overlay-contract.json` (via a tracked
  source symlink) and the three qualification scenarios under
  `share/tinker_sim_bridge/{integration,scenarios}/`; the provenance suite
  verifies installed bytes equal canonical source bytes.
- The focused provenance invocation is documented with
  `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` (the ROS `launch_pytest` plugin otherwise
  fails collection on the unavailable `lark` module).  The pre-existing pinned-uv
  environment failure remains and is not masked.

### Added (Task 8: OMPL overlay acceptance contract)

- `tests/test_provenance.py` extended (not replaced) with a deterministic
  acceptance-contract provenance suite (`Task8OMPLOverlayProvenanceTest`) that
  recomputes every derived hash/contract in the committed
  `integration/ompl-overlay-contract.json` from the real source and artifacts
  and fails on mutations: canonical-JSON determinism (no timestamps, no
  self-referential Task 8 commit), repository/commit identities from actual git
  history (simulator `f34de5f`; production runtime hardening
  `f3e2ce4..df702a5` and Task 2 launch `f7fea50..39d96a1`), production overlay
  18-argument contract and literal-false `use_cumotion_*` compatibility values,
  provider-manifest raw + canonical identity, model-bundle schema/artifact
  hashes/current-artifact identity, typed action/service/topic contract
  (including `/isaac_joint_commands` depth 50 and the external future
  `/tinker_integrated_gate_executor` publisher ownership), fixture/scenario
  identities, build commands (never raw colcon), and source-lock exclusion.
  The pre-existing uv environment provenance failure (installed `uv 0.12.0` vs
  pinned `uv 0.10.8`) remains and is not masked.
- `setup.py` now registers the `readiness_waiter` console script and the
  provenance suite verifies every launch/config/integration asset and every
  `main()`-bearing module is registered in `data_files`/`console_scripts`.
- `package.xml` direct-dependency coverage is asserted by the provenance suite
  with no transitive-import assumptions (`xarm_moveit_config` is intentionally
  left to the `mobile_bringup` production launch, which declares it directly).
  Added the missing direct `robot_localization` dependency (the shipped
  `launch/navigation.launch.py` launches `robot_localization/ekf_node`).

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

### Fixed (Task 6 fix round 1: make staged OMPL readiness executable)

- **Public report schema is now production-compatible.**  The canonical
  `scenario-runner.json` `integrated` field is exactly
  `{"execution_profile": "sim_ompl"}` (with exact digest and operation
  identities), matching the shipped `pick_and_place` canonical parser
  (`goal_validation.cpp::parse_scenario_status_json`); the full typed runtime
  readiness contract is carried separately as the
  `runtime_contract_sha256` / `integrated_mapping` evidence and never enters
  the public report.  `public_integrated_mapping()`,
  `build_integrated_mapping()`, `runtime_contract_sha256`, and the launch's
  `public_integrated_mapping`/`runtime_contract_sha256` parameters split the
  two identities.
- **Report validation now passes against a real fixture scenario.**
  `planning_scene_mapping()` derives `owned_ids` from the authoritative Task 5
  `fixture_owned_ids()` helper in declaration order; `_coerce_owned_ids`
  parses both in-memory sequences and JSON-array wire strings and rejects
  malformed values fail-closed.  `validate_report` accepts the public
  `integrated` mapping supplied via `public_integrated_mapping` with the
  full runtime contract falling back to `integrated_mapping`.
- **Launch-required identities now agree with the parsed report.**
  `planning_scene_sha256` is the digest of the four-key report planning-scene
  mapping (`sha256_json(planning_scene_mapping(planning_scene))`); the full
  Task 5 `revision_digest` remains the separate
  `planning_scene_revision_digest`/`required_fixture_revision_digest`.
- **Readiness waiters are bounded and executable.**  The embedded non-spinning
  waiter is replaced by the installed `readiness_waiter` module
  (`python3 -m tinker_sim_bridge.readiness_waiter`) which bounds discovery,
  call, response, and total lifetime by the deadline, spins the client future
  with `rclpy.spin_until_future_complete`, and exits 0 only for a typed
  `success=true` Trigger response.  Verified with a real Humble Trigger
  server/executor test.
- **TF readiness composes the real multi-hop chain.**  `IntegratedReadiness`
  uses `tf2_ros.Buffer` + `TransformListener` (consuming `/tf` and `/tf_static`)
  and records the composed lookup result with no incompatible clock
  comparison.
- **Type evidence is graph-observed, not self-stamped.**  Action backing
  `{endpoint}/_action/send_goal` and `_action/get_result` service types are
  recorded as `observed_types` and compared by the evaluator; wrong/missing/
  ambiguous observed types, duplicate endpoints, wrong sources, and missing
  result/goal services fail closed.
- **QoS is observed and enforced.**  `normalize_qos_value` maps Humble enum
  strings to short names; reliability and durability are compared strictly for
  every typed publisher (joint state, operator, effective safety, fixture,
  integrated status, command publishers).  Depth is compared when a publisher
  actually reports it (Humble `PublishersInfo` reports depth as 0/UNKNOWN, so
  depth is a documented soft dimension at runtime while the pure evaluator
  enforces it from reported values).
- **Command publishers and publisher metadata are probed.**  `/isaac_joint_commands`
  and `/sim/controller/gripper_commands` are probed for count/source/type/QoS,
  and `evaluate_publisher_metadata` compares every typed publisher against the
  contract.
- **Provider-manifest resolved/live agreement is enforced.**  The readiness
  node reports observed nodes/publishers/controllers; the evaluator's
  `_provider_manifest_agreement` reconciles persistent nodes, publishers, and
  typed controller resources against the manifest, excluding the intentionally
  later-provided `/sim/safety/operator` publisher.
- **Collision source corrected.**  `/sim/safety/collision` is asserted from the
  real `/tinker_isaac_gateway` publisher with type/cardinality/QoS/freshness
  and collision-free value.
- **Legacy scenario-runner behavior restored.**  The non-overlay
  (`integrated is None`) branch again writes the previous report shape to
  `--report` and prints the previous payload; the canonical compact report is
  produced only in integrated overlay mode.
- **Qualification reader is schema-tolerant.**  `manipulation_qualification.py`
  `_scenario_readiness` accepts both the legacy top-level
  `scenario` string + `seed` and the canonical `{id, seed, declaration}`
  object without weakening identity/digest validation.
- **Zero structural fingerprints are rejected** consistently with the
  production overlay and model contract.
- `IntegratedReadiness` initializes `_provider_manifest_path`/`_model_bundle_manifest`
  before building the contract and reads Humble's list-typed
  `get_service_names_and_types_by_node` result correctly.

### Changed (Task 6 fix round 2: command QoS depth contract alignment)

- `/isaac_joint_commands` expected QoS depth is aligned from the stale-declared
  `10` to the verified provider truth `50`.  `command_gateway.py` creates the
  topic with RELIABLE/KEEP_LAST `depth=50`; the Task 6 `INTEGRATED_PUBLISHERS`
  contract and `integration/provider-manifest.json` previously declared `10`.
  Humble reports depth 0/UNKNOWN (masking the mismatch), so an RMW that reports
  positive depth would have failed every readiness/smoke run.  No provider
  behavior was changed; the contract now matches the existing gateway QoS.
  The provider-manifest canonical self-hash was recomputed accordingly
  (raw-byte digest behavior is unchanged).

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
