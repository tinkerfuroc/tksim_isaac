# Changelog

All notable changes to this package are documented here.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

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
