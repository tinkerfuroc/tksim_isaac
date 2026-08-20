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

SIGINT every recorded PID **in reverse bring-up order** (Stage 6 first, Stage
1 last), wait 10 s, SIGKILL survivors, then confirm the GPU is clear:

```bash
nvidia-smi --query-compute-apps=pid --format=csv,noheader
```

This must be empty again before starting another run. Never `pkill` a
pattern that can match the running shell — use explicit PIDs, or a bracketed
regex that cannot self-match (e.g. `pgrep -f 'run_sim[.]py'`, not
`pgrep -f run_sim.py`).
