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

## Automated bring-up: `scripts/gpsr-stack`

`scripts/gpsr-stack up/down/status` automates Stages 1-5 below end to end
(Stage 6, the GPSR orchestrator, still lives in the `tk25_ws` decision repo
and is started separately — see Stage 6). Prefer it over running each stage
by hand unless you're debugging a specific stage.

```bash
./scripts/gpsr-stack up --scenario gpsr-rcw2026-bench --seed 0 \
  --manipulation mock --sim-gpu 0                 # hybrid: manipulation mocked
./scripts/gpsr-stack up --scenario gpsr-rcw2026-bench --seed 0 \
  --manipulation live --sim-gpu 0 --manip-gpu 1    # live manipulation, separate GPU
./scripts/gpsr-stack status                        # one-shot interface census
./scripts/gpsr-stack down                          # tears down the newest run dir
./scripts/gpsr-stack up ... --evidence              # arena observer camera on (contact sheets); costs RTF, off for batteries
```

- `up` launches each stage as its own process group, waits on that stage's
  readiness gate (`tools/gpsr_interface_census.py`, polled every 2 s, 180 s
  timeout per stage — see the census-gating design note at the top of
  `scripts/gpsr-stack`), and writes `<log-dir>/<timestamp>/<stage>.log` +
  `.pgid` per process (default `--log-dir gpsr_stack_logs`, git-ignored). Any
  failure mid bring-up tears down everything already started before
  propagating the error.
- `down` SIGINTs every recorded PGID in reverse stage order, waits, then
  SIGKILLs survivors — then polls each PGID again and re-SIGKILLs anything
  still alive, printing survivor pid+name, which is what satisfies this
  runbook's "`cumotion_goal_set_planner_node` must be killed explicitly"
  requirement (below) via the process's own recorded PGID, never a name
  sweep.
- `--manipulation live` requires the canonical model bundle to already be
  produced at `outputs/ompl-overlay/model-bundle/model-bundle.json` (see
  Stage 4 and `ros2_ws/src/tinker_sim_bridge/README.md`); `up` raises with
  the exact two commands to produce it if it's missing.
- The arena observer camera (a sim-only overview stream, out of GPSR parity)
  is **opt-in and currently known-broken** under full-stack load — a
  CUDA-700 illegal-memory-access race — so `gpsr-stack` never enables it.
  See `docs/developer-log.md`, "2026-08-26 — GPSR recorded sim battery
  bring-up" for the full investigation and fix history; re-enable via
  `TINKER_SIM_ARENA_CAMERA=1` only once that's resolved.

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

### Camera cadence under a live stack (`TINKER_SIM_CAMERA_HZ`)

Export `TINKER_SIM_CAMERA_HZ=12` before Stage 1 for any run with the GPSR
stack attached (15 if the vision stack needs it). Kit renders both RTX
cameras and serialises the images once per camera stride inside the step
loop, so the camera rate is the single largest RTF lever.
`simulation/sensors/hardware-parity.json` stays authoritative and unedited:
unset, its 30 Hz is used. The override may only *lower* the rate —
publishing faster than the real camera would be a parity violation, and is
refused. 12 Hz is exact at control 120 and 60 (strides 10 and 5); at control
30 it rounds to 15 Hz.

If RTF falls too low, Nav2 cannot service its lifecycle bonds in wall time
and tears its own stack down — `Switch controller timed out after 2.000000
seconds`, then every navigation node deactivates, leaving no `map -> odom`
and an unusable TF tree. Treat that log line as "the simulator is too slow",
not as a navigation fault.

### Control cadence under a live stack (`TINKER_SIM_CONTROL_HZ`)

Export `TINKER_SIM_CONTROL_HZ=60` (with `TINKER_SIM_CAMERA_HZ=12`) before
Stage 1 for live-stack runs. Do **not** use `TINKER_SIM_PHYSICS_HZ=60` for
that purpose.

The simulator has two rates. The *physics* rate (`TINKER_SIM_PHYSICS_HZ`,
default 120, may only be lowered) is PhysX's solver step — the thing every
contact/grasp result was validated at. The *control* rate
(`TINKER_SIM_CONTROL_HZ`, default = physics rate) is how often Isaac Lab,
the joint-target writes, the wheel slew, the gateway publish and `/clock`
run. With the control rate lowered, each control step runs
`physics_hz / control_hz` explicit solver substeps of the validated 1/120 s,
so the solver trajectory is unchanged while every per-step wrapper cost is
paid fewer times per simulated second. The control rate must divide the
physics rate evenly and has a 30 Hz floor. Lower control rates also lower
the IMU (200 Hz parity) and base-state (50 Hz) publish cadences, which are
derived from the control step.

Expected RTF (simulated / wall; gpsr-rcw2026, rcw2026 arena, this host):

| setting | physics-only (no ROS, no cameras) | sensor-rich + ROS, cameras 12 Hz, robot safety-stopped | Stage 2 bridge attached: idle / base driving / arm trajectory |
|---|---|---|---|
| default 120 / control 120 | 0.75 | ~0.64 | — |
| `TINKER_SIM_CONTROL_HZ=60` | 1.18 | **0.77-0.80** | **0.81 / 0.68 / 0.63** |
| `TINKER_SIM_CONTROL_HZ=30` | 1.88 | ~0.85 | — |
| arena camera on (`--evidence`) | — | — | **~0.58 idle** (0.54-0.59, 2026-08-29 bench measurement) |

Adding `TINKER_SIM_CPU_THREADS=16` (Kit's worker pool; default 32 on this
host) is worth ~0.03-0.05 on every bridge-attached figure and is safe.

What bounds RTF at control 60 is the PhysX solve (~9 ms per control step)
and Kit's render pump for both RTX cameras (~30 ms per camera frame, scaling
with pixel count); neither has a safe knob left. Two opt-in knobs exist but
are **not recommended for anything that produces evidence**:
`TINKER_SIM_SOLVER_POSITION_ITERATIONS` / `TINKER_SIM_SOLVER_VELOCITY_ITERATIONS`
override the robot USD's articulation solver iteration counts (32 / 1) and
change drive and contact convergence.

### Profiling and actuator-model knobs

`TINKER_SIM_PROFILE=1` prints a `step_profile` line every
`TINKER_SIM_PROFILE_EVERY` camera cycles with per-cycle wall time split into
physics / publish / kit_pump / cameras / spin / unaccounted, plus
`physics_breakdown_ms`, `publish_breakdown_ms`, `camera_breakdown_ms` and
`spin_breakdown`. `TINKER_SIM_STOCK_ACTUATOR_MODEL=1` restores Isaac Lab's
stock actuator loop (slower; for A/B checks only). With the optional
`--camera-pointcloud` flag (off in this runbook) the cloud is built from the
millimetre depth, i.e. quantised to 1 mm.

Known model defect found on the way (not fixed here): the gripper's five
finger/knuckle joints carry no mimic constraint in `robot.usd` (the URDF's
`<mimic>` tags did not survive conversion), so they swing freely in normal
operation; grasp evidence should treat finger poses accordingly until the
conversion is fixed.

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

### Running with the Stage 2 bridge attached

Expected RTF with the bridge up at `TINKER_SIM_CONTROL_HZ=60`,
`TINKER_SIM_CAMERA_HZ=12`: ~0.81 idle, ~0.68 while the base drives, ~0.63
while the arm follows a trajectory (0.77-0.80 standalone). `command_gateway`
publishes a full snapshot only on change (at most 60 Hz,
`CommandGateway.MIN_PUBLISH_PERIOD_S`) or every 50 ms
(`KEEPALIVE_PERIOD_S`). The simulator takes inbound ROS messages on its
simulation thread inside `spin_once()`; `TINKER_SIM_GATEWAY_EXECUTOR=1`
restores the former executor thread for A/B checks only. If RTF is much
lower with the bridge attached, run with `TINKER_SIM_PROFILE=1` and read
`spin_breakdown.commands`: more than ~100 commands per 100-step window
while idle means the bridge is streaming unchanged snapshots. Attribute
profile windows by their `wall_time` field, not by `/clock`: the simulator
re-zeroes `/clock` when the bridge's `ResetSimulation` lands. Opt-in knobs,
defaults unchanged: `TINKER_SIM_CPU_THREADS` (16 recommended for live-stack
runs) and `TINKER_SIM_GIL_SWITCH_INTERVAL_MS`.

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

**Which map AMCL gets.** With `map_yaml` blank, `gpsr.launch.py` resolves the
map from the scenario's `world.arena` (`artifacts/arena/<arena>/current.json`
→ that artifact's `map.yaml`) — the same map the simulator raycasts its
synthetic lidar against. Only a scenario with no arena falls back to the
robot artifact's colocated `map.yaml`, which is the **hardware** arena
(`0701_robocup_arena3`) and shares no occupied cell with `rcw2026`.
`navigation.launch.py` has no scenario, so pass `arena:=rcw2026` (or an
explicit `map_yaml:=`) whenever Stage 1 ran with `--arena`. An explicit
`map_yaml:=` always wins.

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
