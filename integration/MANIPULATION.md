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
export PYTHONPATH="$PWD/validation:$PYTHONPATH"
python3 validation/ompl_plan_smoke.py --mode joint \
  --report outputs/ompl-plan-smoke/ompl-plan-smoke.json
```

### Scenario selection and expected terminal outcomes

| Mode | Terminal B `scenario:=` | Smoke `--mode` | Expected report |
|---|---|---|---|
| Joint | `qualification-moveit-plan-joint` | `joint` | `evaluation.ready=true`, `outcome.kind="success"`, `trajectory_point_count >= 1`, `command_observations.samples == 0` |
| Pose | `qualification-moveit-plan-pose` | `pose` | same as joint |
| Blocked | `qualification-moveit-plan-blocked` | `blocked` | `evaluation.ready=true`, `outcome.kind="non_success"`, `error_code != 1` |

The smoke exits 0 on `evaluation.ready=true` and 1 otherwise.  A mode/scenario
mismatch (the scenario's `qualification_gate` is not `moveit-plan-<mode>`) is
rejected fail-closed before any goal is sent.  Joint mode plans to a small
reach from a vertical arm; pose mode targets a point
`POSE_APPROACH_Z_OFFSET` above the scenario's `target` object; blocked mode
targets the interior of the `blocker` object so every goal sample is in
collision, giving a deterministic non-success.

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
export PYTHONPATH="$PWD/validation:$PYTHONPATH"
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

## Deferred work and status

This is development-only and not release-qualified. The current tree has no
live manipulation pass to report. MoveIt and cuMotion remain deferred until
the FJT, safety, gripper/contact, collision, and retention gates are backed by
raw physics evidence. Vision, decision, and VLA are also deferred until that
manipulation core is qualified.
