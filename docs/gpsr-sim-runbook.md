# GPSR-in-Simulation Bring-up Runbook

Brings up all six stacks GPSR needs — Isaac sim, the tinker-sim ROS bridge,
tk26_vision, tk25_manipulation, tk26_navigation, and the GPSR orchestrator
itself — with **every subsystem real**. Nothing is mocked: Nav2 drives the
real base, the real detection/grasp servers run, and the sim publishes
hardware-parity camera topics in place of the physical Orbbec/RealSense
drivers. This is the design in
`docs/superpowers/specs/2026-08-20-gpsr-in-simulation-design.md`; read it
first if any of the "why" below is unclear.

Run each stage in its own terminal, in order, and leave it running — later
stages depend on earlier ones being up. Record the PID reported at the top of
each stage's output (or `echo $!` immediately after backgrounding it); Teardown
below needs that list.

## GPU pre-flight (before Stage 1, verbatim)

Never launch Isaac while another Isaac holds the GPU.

```bash
nvidia-smi --query-compute-apps=pid --format=csv,noheader
```

This **MUST be empty**. If it is not, wait — do not contend for the GPU.

## Conventions used in every stage below

- All six stacks share `ROS_DOMAIN_ID=42` (the plan's live-run domain; the
  repo's own `deployment.env.example` default of `25` is for other
  workflows — override it here).
- `TINKER_WS=/home/tinker/tk25_ws` is the external Humble workspace; it holds
  `tk25_decision`, `tk25_manipulation`, `tk26_vision`, and `tk26_navigation`
  as one built `install/`, so every stage 3–6 command sources the same
  `$TINKER_WS/install/setup.bash`.
- Every stage repeats this env preamble before its stack-specific sourcing,
  to guarantee a clean environment even if the terminal previously sourced a
  different workspace:

  ```bash
  set -a; source .deployment.env; set +a
  unset PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH LD_LIBRARY_PATH
  export ROS_DOMAIN_ID=42
  ```

  `.deployment.env` (gitignored; copy from `deployment.env.example`) carries
  `TINKER_ACCEPT_OMNIVERSE_EULA` and `RMW_IMPLEMENTATION`; never print or
  copy its contents.

## Prerequisites before Stage 3 — verified against a real 2026-08-20 bring-up

The `tk25_ws` install tree lags tk26_vision's source by months, and **every**
symptom below presents as "the interface simply isn't there" rather than as a
build error. Do all of this once, before the first live run.

### 1. Rebuild `tinker_vision_msgs_26`

The **installed** package (built 2026-05-31) predates the `.action` definitions
for `FeatureExtraction`/`DetectWaving` added to source on 2026-08-18 — it
exports neither as an action. tk26_vision serves both as ActionServers
(`feature_recognition.py`, `waving_person_server.py`), so `describe_person` and
waving detection hang against the stale install.

```bash
cd /home/tinker/tk25_ws
colcon build --packages-select tinker_vision_msgs_26
```

### 2. Build `camera_provider`, then rebuild the stale vision packages

`camera_provider` had **never** been built, and it is a dependency of six
packages (`object_detection_generalist`, `object_detection_new`,
`kimi_api`, `tk_vision_specialized`, `vision_util`, `monocular_depth`) plus
`vision_bringup`. Until it exists, rebuilding any of them fails with
`Failed to find .../camera_provider/share/camera_provider/package.sh`.

The installed `kimi_api`/`tk_vision_specialized` also still used
`create_service()` for `feature_extraction_service`/`detect_waving_persons`
— the srv→action conversion was never built in, so the census reports both
actions missing while the *services* show up. `vision_util` was likewise stale
(missing `action_queue`, which the converted servers import), and the installed
`object_detection_generalist` still defaulted `orbbec_depth_image_topic` to
`/camera/depth_to_color/image_raw` — a topic nothing publishes, since
tk26_vision commit `39de423` moved it to `/camera/depth/image_raw` (which is
what both the real driver bringup and the sim use). That mismatch surfaces as
`No orbbec camera data within sync threshold`, never as an error.

Use tk26_vision's own wrapper — plain `colcon build` writes `#!/usr/bin/python3`
shebangs that cannot see venv-only deps:

```bash
cd /home/tinker/tk25_ws/src/tk26_vision
./scripts/build.sh --packages-select camera_provider
./scripts/build.sh --packages-select kimi_api tk_vision_specialized vision_util \
    object_detection_generalist object_detection_new vision_track camera_server \
    --allow-overriding kimi_api tk_vision_specialized vision_util \
    object_detection_generalist object_detection_new vision_track
```

### 3. Install `py_trees_ros` (required by the GPSR orchestrator)

`GPSR/gpsr_orchestrator.py` imports `py_trees` and `py_trees_ros` at module
scope. Neither is present for `/usr/bin/python3`, which is the shebang of the
installed `gpsr-orchestrator` entry point. One apt package supplies both
(it depends on `ros-humble-py-trees`):

```bash
sudo apt install -y ros-humble-py-trees-ros
```

---

## Stage 1 — Sim

```bash
cd /home/tinker/tinker-sim/6.0.1
set -a; source .deployment.env; set +a
unset PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH LD_LIBRARY_PATH
export ROS_DOMAIN_ID=42

./scripts/tinker-sim launch \
  --sensor-profile sensor-rich \
  --scenario gpsr-rcw2026 \
  --seed 0 \
  --arena rcw2026 \
  --spawn-xy=-2.0,-2.0 \
  --ros \
  --headless
```

### Camera cadence under a live stack

Export `TINKER_SIM_CAMERA_HZ=15` before Stage 1 for any run with the full
GPSR stack attached.

The simulator holds RTF ~0.30 on its own, but drops to ~0.06 once real
subscribers attach: Kit is pumped (both RTX cameras rendered) once per camera
stride, and the image payloads are serialised on the same beat, all inside the
step loop. At that speed Nav2 cannot service its lifecycle bonds and tears its
own stack down — `Switch controller timed out after 2.000000 seconds`, then
every navigation node deactivates, leaving no `map -> odom` and an unusable TF
tree. Halving the cadence halves both costs.

`simulation/sensors/hardware-parity.json` stays authoritative and unedited:
unset, its 30 Hz is used. The override may only *lower* the rate — publishing
faster than the real camera would be a parity violation, and is refused.

### Control cadence under a live stack (`TINKER_SIM_CONTROL_HZ`)

Export `TINKER_SIM_CONTROL_HZ=60` (with `TINKER_SIM_CAMERA_HZ=15`) before
Stage 1 for live-stack runs. Do **not** use `TINKER_SIM_PHYSICS_HZ=60` for
that purpose any more.

The simulator has two rates. The *physics* rate (`TINKER_SIM_PHYSICS_HZ`,
default 120, may only be lowered) is PhysX's solver step — the thing every
contact/grasp result was validated at. The *control* rate
(`TINKER_SIM_CONTROL_HZ`, default = physics rate) is how often Isaac Lab,
the joint-target writes, the wheel slew, the gateway publish and `/clock`
run. With the control rate lowered, each control step runs
`physics_hz / control_hz` explicit solver substeps of the validated
1/120 s, so the solver trajectory is unchanged while every per-step wrapper
cost is paid 60 times a second instead of 120. The control rate must divide
the physics rate evenly and has a 30 Hz floor. Note omni.physx's
`IPhysxSimulation.simulate(elapsed)` does *not* substep on its own — it
integrates exactly `elapsed` — which is why the substeps are explicit and
why simply lowering `dt` was a fidelity change, not an optimisation.

Measured 2026-08-21 (gpsr-rcw2026, rcw2026 arena, GPU 1, RTF =
simulated / wall):

| run | physics-only (no ROS, no cameras) | sensor-rich + ROS, cameras 15 Hz, start of day | same, end of day (all fixes below) |
|---|---|---|---|
| default 120 / control 120 | 0.75 | 0.35 | **0.64** |
| `TINKER_SIM_CONTROL_HZ=60` | 1.18 | 0.44 | **0.70** |
| `TINKER_SIM_CONTROL_HZ=30` | 1.88 | 0.50 | **0.78** |
| `TINKER_SIM_PHYSICS_HZ=60` (old advice) | 1.61 | 0.51 | — (not needed any more) |

The sensor-rich numbers are with the robot safety-stopped (no bridge
attached), which is the state a run spends its start-up and every
command-loss interval in; the "end of day" column includes the lidar,
safety-hold, actuator-launch and camera fixes described further down.

Robot root position after 10 s idle agreed with the default to within
0.5 mm at control 60/30 and drifted 5 mm at physics 60 — the substepped runs
keep the validated solver trajectory, the lowered physics rate does not.
Lower control rates also lower the IMU (200 Hz parity) and base-state
(50 Hz) publish cadences, which are derived from the control step: at 60 Hz
they publish at 60 Hz, at 30 Hz they publish at 30 Hz.

What still bounds RTF under the live stack, per simulated second at control
60: the PhysX solve itself (~10 ms per control step, 32 authored position
iterations), the Kit render pump for both RTX cameras (~30 ms per camera
frame) and the image capture/publish (~22 ms per camera frame). Two further
opt-in knobs exist: `TINKER_SIM_SOLVER_POSITION_ITERATIONS` /
`TINKER_SIM_SOLVER_VELOCITY_ITERATIONS` override the robot USD's articulation
solver iteration counts (32 / 1); 8 position iterations measured RTF 0.52
at control 60 but *changes drive and contact convergence*, so it is not
recommended for anything that produces evidence. PhysX worker-thread count
(`--/persistent/physics/numThreads`, default 8) made no measurable
difference at 4 or 16.

With `TINKER_SIM_PROFILE=1`, every profile line now also carries
`physics_breakdown_ms.physx_substeps` and a `publish_breakdown_ms`
(clock / joint_state / imu / cloud / status / truth per `publish()` call).
### Per-step costs fixed on 2026-08-21 (no knobs, result-neutral)

- **Safety hold.** While stopped, the backend used to disable the arm's
  PhysX drive and push a gravity-compensated PD effort from Python every
  control step; at the control rate that PD limit-cycled (joint1 pinned at
  -100 Nm, arm never at rest) and cost ~8 ms per step. The hold is now
  PhysX's own drive at the latched pose (stiffness 600, damping 80, 100 Nm
  ceiling) with only the gravity term fed forward and refreshed every 30
  control steps. Stopped step 13.3 -> 4.5 ms; the arm sits within 0.005 rad
  at ~zero velocity, and the published joint efforts during a hold are now
  computed with the hold gains (they used to report the nominal 20 000
  stiffness).
- **Target pushes.** Isaac Lab applies actuator groups with two Warp launches
  per group; tinker2 has five, so every target push (arm trajectories, the
  wheel slew ramp, hold refreshes) paid ten launches, ~3.2 ms of a ~3.6 ms
  push. The backend now binds a fused `_apply_actuator_model` that launches
  each kernel once over all groups -- bit-identical staging and telemetry
  buffers, push 4.3-6.4 -> 1.8 ms. `TINKER_SIM_STOCK_ACTUATOR_MODEL=1`
  restores Isaac Lab's loop.
- **Camera conversions.** `rgb8_array`/`depth_to_16uc1_mm` reuse scratch
  buffers and write in place (byte-identical, proven by
  `tests/test_camera_publish_equivalence.py`); ~14% off the conversion time.
  With `TINKER_SIM_PROFILE=1` the profile line now also carries a
  `camera_breakdown_ms` (capture / rgb_convert / depth_convert / image_fill /
  image_publish / info).

Known model defect found on the way (not fixed here): the gripper's five
finger/knuckle joints carry no mimic constraint in `robot.usd` (the URDF's
`<mimic>` tags did not survive conversion), so they swing freely in normal
operation; grasp evidence should treat finger poses accordingly until the
conversion is fixed.

That breakdown is how the development-lidar ray-cast was found to cost
~35 ms per lidar frame (~350 ms per simulated second); it is now vectorised
and bit-identical (`OccupancyMap.raycast_many`), at ~2–5 ms per frame;
that alone took the default sensor-rich run from RTF 0.35 to 0.40.

Only the `sensor-rich` profile loads `simulation/sensors/hardware-parity.json`
and publishes the real-named camera topics GPSR's vision stack needs (valid
`--sensor-profile` values are `physics-only | sensor-rich | navigation-parity
| manipulation-core` — `hardware-parity` is a camera-spec file, not a profile
name). `--scenario gpsr-rcw2026` here spawns the scenario's YCB objects
directly into the Isaac world (`validation/run_sim.py`'s sensor-rich branch
passes `scenario=`/`task=` straight to `IsaacWholeRobotBackend`); it must match
the `scenario` used in Stage 2. `--spawn-xy=-2.0,-2.0` matches the scenario's
`robot.initial_pose` and clears shelf_02, which sits on the arena's default
`(0, 0)` origin. `--ros` is technically implied by `sensor-rich` even if
omitted (`run_sim.py`'s `sensor_rich_implies_ros`), but pass it explicitly for
clarity. Use the `=` form for `--spawn-xy`; a space-separated negative value
is parsed by argparse as a flag, not a value.

## Stage 2 — Composite bridge launch (`gpsr.launch.py`)

```bash
cd /home/tinker/tinker-sim/6.0.1
set -a; source .deployment.env; set +a
unset PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH LD_LIBRARY_PATH
export ROS_DOMAIN_ID=42

source /opt/ros/humble/setup.bash
source .ros-vendor/humble/local_setup.bash
source /home/tinker/tk25_ws/install/setup.bash
source ros2_ws/install/setup.bash

ros2 launch tinker_sim_bridge gpsr.launch.py \
  project_root:=/home/tinker/tinker-sim/6.0.1 \
  tinker_workspace:=/home/tinker/tk25_ws \
  scenario:=gpsr-rcw2026 \
  map_yaml:="" \
  seed:=0 \
  safety_source_deadline_s:=1.0
```

`gpsr.launch.py`'s declared arguments are exactly `project_root`,
`tinker_workspace`, `scenario` (default `gpsr-rcw2026`), `map_yaml` (default
`""`, which resolves to the current artifact's own `map.yaml`), `seed`
(default `0`), and `safety_source_deadline_s` (default `1.0`) — the values
above are the defaults spelled out explicitly. This is a **de-duplicating
merge** of `manipulation.launch.py` and `navigation.launch.py`, not an
`IncludeLaunchDescription` of either: both unconditionally start same-named
singleton nodes (`command_gateway`, `safety_supervisor`, `contract_guard`,
`robot_state_publisher`) with divergent parameters, so the composite is the
one process that owns each of those, plus `ros2_control_node`, the xArm/
gripper/pan-tilt facades, `base_facade`, `audio_fixtures`, AMCL, and Nav2
(`localization_no_ekf_launch.py` + `navigation_dwb_launch.py`) together. This
gives GPSR `/xarm_gripper/gripper_action`, `/pan_tilt_controller/*`,
`announce`/`listen_action`/`get_confirmation_service` (via `audio_fixtures`),
and `navigate_to_pose` + `map`→`base_link` TF (via Nav2/AMCL) in one process
group.

## Stage 3 — tk26_vision

```bash
cd /home/tinker/tinker-sim/6.0.1
set -a; source .deployment.env; set +a
unset PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH LD_LIBRARY_PATH
export ROS_DOMAIN_ID=42

source /opt/ros/humble/setup.bash
source /home/tinker/tk25_ws/install/setup.bash

ros2 launch vision_bringup vision_bringup.launch.py enable_gpsr:=true \
  use_sim_time:=true &

# The node NAME is load-bearing, not cosmetic -- see below.
ros2 run camera_server camera_server_node --ros-args \
  -r __node:=head_camera_server \
  -p use_sim_time:=true \
  -p color_topic:=/camera/color/image_raw \
  -p depth_topic:=/camera/depth/image_raw \
  -p color_info_topic:=/camera/color/camera_info \
  -p depth_info_topic:=/camera/color/camera_info
```

**Point the head before expecting anything from vision.** From a pristine
spawn the head camera looks at the robot's own deck, so `door_detection_srv`
returns `is_open=0` forever and the GPSR door gate never clears.

`camera_mount_joint` bakes a **-45.5 deg pitch** into the head
(`rpy="0.0406 -0.79457 3.0833"`). The head is a pan-*tilt* unit whose
`tilt_joint` runs -30..+90 deg, so the camera only points forward once
something commands roughly +45 deg of tilt. On hardware the pan-tilt controller
does that, and it lives in `vision_driver.launch.py` — the launch file this
runbook tells you not to start — so in simulation nothing ever commands it.

Verified 2026-08-20: a ~180 deg base rotation changed only 15% of the head
camera's pixels (the geometry rotates with the robot because it *is* the
robot), depth was 0.66–1.65 m across the frame with 39% valid pixels, and the
lidar map showed the 1.5 m ahead of the spawn as free. Publishing

```bash
ros2 topic pub -1 /pan_tilt_controller/cmd tinker_vision_msgs_26/msg/PanTiltCommand \
  '{mode: 0, pan_rad: 0.0, tilt_rad: 0.9}'
```

moved the head and took the valid depth fraction from 0.39 to 0.88 — the camera
started seeing a room instead of itself. The simulator already accepts these
commands through `pan_tilt_facade`; what the resting head pose should be, and
whether the GPSR stack or this bring-up owns setting it, is a robot-behaviour
decision — do not hardcode a guess here.

**Also start the pan/tilt state publisher.** Stage 1's sim publishes
`/pan_tilt_controller/state` exactly as the hardware pan-tilt driver does, but
nothing converts it into `/joint_states` unless this node runs:

```bash
ros2 run pan_tilt state_publisher --ros-args -p use_sim_time:=true
```

On hardware this node ships inside `vision_driver.launch.py`, which this
runbook tells you *not* to launch (it would start the physical Orbbec and
pan-tilt drivers). Skipping the whole launch file also skips this node, and
`pan_joint`/`tilt_joint` then never reach `robot_state_publisher` — the
`joint_state_broadcaster` only publishes the arm joints it claims interfaces
for. The entire head-camera subtree (`tilt_link -> camera_link ->
head_camera_*`) is then a **second, disconnected TF tree**: MoveIt logs
"Unable to transform object from frame 'camera_link' to planning frame
'world'" for every head frame, and any vision result that has to be expressed
in `map` fails to transform. Verified 2026-08-20: with this node running,
`base_link -> camera_link` resolves to `[-0.265, 0.029, 1.544]`.

This is a bring-up gap, not a simulator gap — do not "fix" it by changing
what the simulator publishes.

**Run the camera server as `head_camera_server`, not bare.** Every
service-backend vision consumer resolves frames through
`default_endpoint='/head_camera_server'` (e.g. `door_detection.py`), and
`camera_server/src/compat_bridge_node.cpp` "owns no camera subscriptions; all
payloads are forwarded to the per-camera C++ servers" at its `head_server`
parameter, which defaults to `/head_camera_server`. A bare
`ros2 run camera_server camera_server_node` registers as `/camera_server`, so
nothing serves that endpoint and every consumer starves — reporting
`No camera data or intrinsic.` rather than an error naming the real cause. The
real robot launches it under exactly this name and parameter set
(`vision_bringup/launch/vision_driver.launch.py`, node `head_camera_server`).

**Pass `use_sim_time:=true` to every Stage 3 node.** The sim publishes `/clock`
and stamps frames in simulation time, while a node left on wall-clock time sees
each frame as ~1.8 billion seconds stale and silently rejects all of them —
again surfacing as `No orbbec camera data within sync threshold` rather than a
clock error. Stages 4 and 5 already pass it; Stage 3 needs it too.

**Headless hosts:** `waving_person_server` defaults to `show_window:=true` and
opens a `cv2` window, aborting with `Available platform plugins are: xcb` when
there is no display. Start it with `-p show_window:=false` (or export
`QT_QPA_PLATFORM=offscreen`) on a headless run.

Do **not** additionally launch `vision_driver.launch.py` — that starts the
physical pan-tilt/Orbbec/FoundationStereo hardware drivers, and Stage 1's
sim already publishes the byte-identical camera topics those drivers would
otherwise produce.

`enable_gpsr:=true` starts the GPSR node set (`yolo_seg_node`,
`person_track_server`, `waving_person_server`, `feature_recognition`,
`get_image`) plus the always-on core (`generalist_node`, `door_detection`)
and the legacy `camera_compat_bridge`. **`camera_backend` defaults to
`"service"`** on every vision node that reads camera frames (declared
per-node, e.g. `grocery_categorize.py`, `waving_person_server.py`,
`object_match_all_server.py`) — so either `camera_server_node` runs (as
above) to serve that backend, or every vision node needs an explicit
`-p camera_backend:=subscription` override to read frames straight off the
topics instead. The camera server's topic defaults (`/camera/color/image_raw`,
`/camera/depth/image_raw`, `/camera/color/camera_info` for both color and
depth info) already match the head-camera topic set GPSR uses, but it must
still be started under the `head_camera_server` node name as shown above —
the parameters are spelled out there to mirror the real robot's launch
exactly.

## Stage 4 — tk25_manipulation

```bash
cd /home/tinker/tinker-sim/6.0.1
set -a; source .deployment.env; set +a
unset PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH LD_LIBRARY_PATH
export ROS_DOMAIN_ID=42

source /opt/ros/humble/setup.bash
source /home/tinker/tk25_ws/install/setup.bash

ros2 launch mobile_bringup manipulation_planning_task_only.launch.py \
  execution_profile:=sim_cumotion \
  use_sim_time:=true &

ros2 run arm_api grasp_action &

ros2 run anygrasp_ros2 anygrasp
```

`execution_profile:=sim_cumotion` is one of the launch file's three profiles
(`hardware` the default, `sim_ompl`, `sim_cumotion`) and requires
`use_sim_time:=true`. This gives GPSR `joint_move_action` (the real
`pick_and_place` task server) against the sim arm. `arm_api/grasp_action` is
a pure ROS orchestrator serving `start_grasp`; it needs no sim-specific
profile. `anygrasp_ros2`'s console script is named `anygrasp` (package
`anygrasp_ros2`, executable `anygrasp`) and supplies the grasp-pose proposals
`grasp_action` consumes. `/xarm_gripper/gripper_action` itself is already
served by Stage 2's `gripper_facade` — nothing in this stage duplicates it.

**Known gap, verify before relying on real grasp poses:** `anygrasp_ros2`
hardcodes its checkpoint at
`/home/tinker/anygrasp_sdk/grasp_detection/log/checkpoint_detection.tar`
(not a launch parameter). That path does not exist on this host as of this
writing — the node will fail at init until the AnyGrasp SDK/weights are
installed there. This is a real prerequisite gap, not a sim workaround to
route around.

## Stage 5 — tk26_navigation

```bash
cd /home/tinker/tinker-sim/6.0.1
set -a; source .deployment.env; set +a
unset PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH LD_LIBRARY_PATH
export ROS_DOMAIN_ID=42

source /opt/ros/humble/setup.bash
source /home/tinker/tk25_ws/install/setup.bash

ros2 launch approach_planner approach_planner.launch.py use_sim_time:=true &

ros2 run orientation_angle_service orientation_angle_server
```

These two are the only pieces Stage 2's Nav2/AMCL doesn't already provide:
`approach_planner`'s `planner_node` serves `go_to_approach` and
`find_approach_pose`; `orientation_angle_server` serves
`orientation_angle_service` (`tinker_nav_msgs/srv/OrientationAngle`), which
`BtNode_GetOrientationAngle` calls directly (not through an action).

## Stage 6 — GPSR

```bash
cd /home/tinker/tk25_ws
source install/setup.bash
export ROS_DOMAIN_ID=42
export GPSR_CONSTANTS_PATH=$PWD/src/tk25_decision/src/behavior_tree/behavior_tree/GPSR/constants.rcw2026.json
export BT_MOCK_CONFIG=$PWD/src/tk25_decision/src/behavior_tree/behavior_tree/mock_config.sim.json
set -a; source .env; set +a

ros2 run behavior_tree gpsr-orchestrator
```

`GPSR_CONSTANTS_PATH` selects the arena's own waypoint file
(`constants.rcw2026.json`, eight standing poses over the rcw2026 furniture)
in place of the competition-map `constants.json` the planner LLM is prompted
against by default; **leaving it unset keeps the competition default**, which
is wrong for this sim (its waypoints don't exist on the derived arena map).
`BT_MOCK_CONFIG` must point at `mock_config.sim.json` (this task, File 1) —
every subsystem `enabled: false`, so nothing routes through
`MockInputController` even though `mock_mode.enabled` is left at its shipped
`true`. `tk25_ws/.env` (not this repo's `.deployment.env`) carries the
LLM/API keys the planner and vision kimi_api nodes need; it is sourced last
so it doesn't get clobbered by an earlier `set -a` block, and its contents
are never read or printed here.

To drive a single command instead of the interactive REPL, also export
`BT_GPSR_CMD="<command text>"` and `BT_GPSR_NUM_COMMANDS=1` before the
`ros2 run` above.

---

## Teardown discipline (verbatim)

**On a shared host, tear down by domain, never by pattern.** Pattern-only
teardown (`pgrep -f tinker_sim_bridge`, bare `move_group`, `cumotion`,
`pick_and_place`) is domain-blind and will kill a concurrent session's nodes.
Every process is filtered by its own `ROS_DOMAIN_ID`:

```bash
scripts/kill-domain-nodes 42            # dry run: lists what WOULD be killed
scripts/kill-domain-nodes 42 --force    # SIGTERM, then SIGKILL survivors
```

Confirm other domains are untouched before and after (`scripts/kill-domain-nodes
<other-domain>` dry-run should report the same count both times).

**`cumotion_goal_set_planner_node` must be killed explicitly.** It holds
~6.6 GB of GPU and outlives a careless teardown, so the next run OOMs.

**Relaunch the decision stack after any dependency restarts.** The GPSR
orchestrator caches action-server goal handles; if the bridge, vision,
manipulation, or navigation stack restarts underneath it, it stalls on a stale
handle rather than reconnecting. Stage 6 always comes last, and always fresh.


SIGINT every recorded PID **in reverse bring-up order** (Stage 6 first, Stage
1 last), wait 10 s, SIGKILL survivors, then confirm the GPU is clear:

```bash
nvidia-smi --query-compute-apps=pid --format=csv,noheader
```

This must be empty again before starting another run. Never `pkill` a
pattern that can match the running shell — use explicit PIDs, or a bracketed
regex that cannot self-match (e.g. `pgrep -f 'run_sim[.]py'`, not
`pgrep -f run_sim.py`).
