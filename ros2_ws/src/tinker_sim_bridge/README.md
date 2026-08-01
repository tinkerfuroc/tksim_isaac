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
- `evaluate_integrated_cardinality(*, joint_state_publishers)` — classifies
  publisher cardinality/source from ROS graph endpoint metadata.
- `evaluate_robot_description_contract(description)` — evaluates the same
  `drive_joint` state-only contract in the `robot_description` parameter
  received by `/controller_manager`.
- `evaluate_xacro_contract(xacro_text)` and
  `evaluate_joint_state_evidence_pair(...)` — parse the checked-in xacro source
  and compare source-xacro and live-parameter evidence together.

Each returns a complete mapping with `ready`, `reasons`, and the observed
values.  These modules stay import-time ROS-free and run under both simulator
CPython 3.12 and Humble CPython 3.10.

### ROS Humble live probe

`joint_state_probe` (`ros2 run tinker_sim_bridge joint_state_probe`) is the
Task 6 evidence path.  It reads the `/controller_manager` `robot_description`
parameter, parses the checked-in xacro, subscribes to `/joint_states`, and reads
`/joint_states` publisher graph metadata, then evaluates all of the above pure
helpers and publishes compact JSON on `/sim/status/joint_state_contract`
(latched, reliable).  It never counts source text as primary evidence;
`source_text_diagnostics` in the report is supplemental only.

```bash
ros2 run tinker_sim_bridge joint_state_probe \
  --ros-args -p controller_manager:=/controller_manager -p broadcaster:=joint_state_broadcaster
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

## Deployment gateways

The remaining bridge nodes implement hardware-parity gateways and controllers
for the external Humble boundary (base, xArm, gripper, pan-tilt, command
gateway, safety supervisor, contract guard, truth evaluator, scenario runner,
audio fixtures).  See the repository root README for the deployment model.
