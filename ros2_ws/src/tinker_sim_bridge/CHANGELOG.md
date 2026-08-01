# Changelog

All notable changes to this package are documented here.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

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
