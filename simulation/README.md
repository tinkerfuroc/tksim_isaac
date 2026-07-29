# Simulation integration boundary

`tinker_sim_core` is ROS-independent and contains deterministic scenario,
command-arbitration, parity-odometry, and evaluator logic. It never imports
the Tinker workspace.

Isaac owns physics and standard ROS endpoints only:

- CPU PhysX for every behavior-validation profile.
- `/isaac_joint_states` and `/isaac_joint_commands` as the articulation
  boundary.
- NVIDIA's `isaacsim.ros2.sim_control` implementation of
  `simulation_interfaces`.
- hardware-compatible camera, LiDAR, IMU, clock, and TF topics.

System-Humble processes own FJT, gripper and task actions, xArm services,
pan-tilt compatibility, Nav2, MoveIt/cuMotion, decision, audio, and VLA.
The sole `/isaac_joint_commands` publisher is the external command gateway.

Scenario seed is immutable launch metadata. `scenario_runner` compiles scenario
files into standard reset/spawn/state operations; no custom lifecycle aliases
exist. Only the evaluator consumes `/sim/truth/*`, and claimed task success is
always checked against hidden postconditions.

## Manipulation operator slice

The manipulation launch is an external-Humble operator surface. It resolves
the current content-addressed Tinker 2 artifact and starts topic-based
`ros2_control`, the command gateway, safety supervisor, xArm/gripper/pan-tilt
facades, contract guard, truth evaluator, and scenario runner. It deliberately
does not launch Nav2, MoveIt, cuMotion, vision, decision, or VLA nodes.

Build the Humble overlay before the first run:

```bash
cd /home/tinker/tinker-sim/6.0.1
./scripts/build-humble-overlay
```

For a development run, use two terminals. Terminal A must not source system
ROS or the Tinker workspace:

```bash
cd /home/tinker/tinker-sim/6.0.1
export TINKER_ACCEPT_OMNIVERSE_EULA=Y
./scripts/launch-isaac --sensor-profile manipulation-core --profile parity \
  --scenario pick-deliver-place --seed 7 --ros
```

In Terminal B, source the system-side environment through the wrapper:

```bash
cd /home/tinker/tinker-sim/6.0.1
./scripts/launch-humble manipulation scenario:=pick-deliver-place seed:=7 \
  qualification:=false
```

The scenario runner waits for `/reset_simulation`, `/spawn_entity`, and
`/set_simulation_state` from `simulation_interfaces` before issuing reset,
an explicit stopped state, all spawns, and the final playing state. The final
state operation is marked `PHYSICS_READY`. `attempt_dir:=...` writes its
operation report to `scenario-runner.json` under that directory.

Qualification uses the auditable development attempt runner. Manifest-only is safe for
preflight; live gates require explicit gate executors:

```bash
cd /home/tinker/tinker-sim/6.0.1
export TINKER_ACCEPT_OMNIVERSE_EULA=Y
./.venv/bin/python validation/manipulation_qualification.py \
  --scenario simulation/scenarios/qualification-free-space.json \
  --manifest-only
```

This creates a manifest and provenance record only. A live development run
requires one or more `--gate-command NAME=...` executors; without them the
runner preserves a `not-configured` failure. Successful external commands are
`executed-unverified` and cannot qualify a run. No live manipulation pass is
claimed.

Operator readiness and artifact retention are documented in
[`integration/MANIPULATION.md`](../integration/MANIPULATION.md). Preserve every
attempt directory, including failed startup and timeout attempts. Do not
overwrite reports, delete failed artifacts, teleport spawned objects, bypass
the command gateway, or treat an action result as proof of a world postcondition.

This remains development-only and is not release-qualified. MoveIt and cuMotion
are deferred until the manipulation core is physically evaluated; vision,
decision, and VLA vertical slices are deferred behind the same qualification
work.
