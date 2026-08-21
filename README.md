# Tinker whole-robot simulation deployment

This is a standalone deployment project for x86_64 Tinker simulation servers.
It neither imports nor modifies the surrounding ROS workspace. For production,
copy this directory to a dedicated location such as
`/srv/tinker-sim/6.0.1`; do not overlay it on a sourced ROS workspace.

## Qualified baseline

| Component | Pinned value |
| --- | --- |
| Host | Ubuntu 22.04, x86_64, GLIBC 2.35 or newer |
| NVIDIA driver | 595.58.03 or newer |
| Python | CPython 3.12.13 |
| uv | 0.10.8 (0.10 lock format) |
| Isaac Sim | 6.0.1.0 |
| Isaac Lab | v3.0.0-beta2.patch1, commit `ffff603eafc6b74264a5261cc0183d6a65390d78` |
| Isaac Lab PhysX extension | 1.1.3 from the same pinned checkout |
| Isaac Lab Newton interface shim | 0.13.6 from the same pinned checkout |
| PyTorch | 2.11.0, CUDA 12.8 wheel |
| TorchVision / TorchAudio | 0.26.0 / 2.11.0, CUDA 12.8 wheels |
| Pillow | 12.2.0 |
| IsaacSim ROS workspaces | tag `IsaacSim-6.0.1`, commit `dd3eeede7912755996a18f4884285d9f50843f79` |
| ROS simulation interfaces | Humble 1.4.0, vendored Debian artifact |
| Topic-based ros2_control | Humble 0.2.0, vendored Debian artifact |

Provisioning requires at least 100 GB free disk, 32 GB RAM, and an RTX GPU
with 16 GB VRAM. Sixty-four GB RAM is the supported whole-robot target.
Drivers older than 595.58.03 produce an explicit experimental warning and are
not release-qualified.

`uv.lock` is authoritative for Python. The static `release-manifest.json` records the
lock, project, uv executable, managed Python, and Isaac Lab Git tree hashes.
`artifacts/provenance/ros-debs.json` is authoritative for the isolated Humble
runtime additions.
Each completed deployment also writes a machine-readable report under
`reports/`.

## Online bootstrap

Install uv 0.10.8 from its official release, start a fresh shell that has not
sourced `/opt/ros/humble`, then:

```bash
cd /srv/tinker-sim/6.0.1
cp deployment.env.example .deployment.env
# Read NVIDIA's Omniverse EULA, then set TINKER_ACCEPT_OMNIVERSE_EULA=Y.
set -a
source .deployment.env
set +a

./scripts/tinker-sim preflight
./scripts/bootstrap --mode online
```

EULA acceptance is never set by a script or repository default. Bootstrap
fails until the deployment configuration explicitly contains
`TINKER_ACCEPT_OMNIVERSE_EULA=Y`. After that explicit gate passes, bootstrap
sets Isaac Kit's required `OMNI_KIT_ACCEPT_EULA=YES` only for its child
processes.

Bootstrap is idempotent. It:

1. validates the OS, architecture, GLIBC, disk, RAM, GPU, VRAM, driver, and
   registered NVENC encoder;
2. installs CPython 3.12.13 into the project-local managed-Python directory;
3. fetches and verifies the exact Isaac Lab commit without patching it;
4. runs only `uv sync --frozen` and reconstructs a temporary environment with
   `uv sync --frozen --offline` to audit cache completeness;
5. runs Isaac's compatibility checker, 10,000 headless PhysX steps, requires
   non-empty RTX camera and LiDAR output, performs a real NVENC encode, and
   starts the headless WebRTC experience;
6. warms core, ROS, RTX-sensor, and URDF extension caches; and
7. vendors the exact ROS Debian artifacts and pins NVIDIA's IsaacSim ROS
   workspace without installing either into the host; and
8. writes a deployment report containing runtime hashes, package versions,
   ROS domain settings, and test results.

`--skip-preflight`, `--skip-validation`, and `--skip-prewarm` exist for tooling
tests only. A deployment made with any of them is not release-qualified.

No system CUDA toolkit is read or installed. CUDA runtime libraries come from
the Isaac Sim and PyTorch wheels.

## ROS 2 Humble boundary

There are two process environments:

- Isaac runs from this uv environment with Python 3.12 and Isaac's internal
  Humble libraries.
- Tinker gateways, Nav2, MoveIt/cuMotion, vision, decision, audio, VLA, FJT,
  and hardware-facade nodes run under system Humble with Python 3.10.

`scripts/launch-isaac` refuses to start if `PYTHONPATH`,
`AMENT_PREFIX_PATH`, `CMAKE_PREFIX_PATH`, `COLCON_PREFIX_PATH`,
`ROS_PACKAGE_PATH`, or `LD_LIBRARY_PATH` contains `/opt/ros/` or
`python3.10`. It then locates Isaac's internal
`isaacsim.ros2.* / humble / lib` directory and sets the shared ROS domain and
Fast DDS implementation. The default `--dds-profile local` leaves Fast DDS
shared memory enabled. `--dds-profile lan` selects the committed UDP-only
profile and must be used by every machine in that run.

Only the standard interfaces listed in `deployment.json` cross the Isaac DDS
boundary. NVIDIA's `isaacsim.ros2.sim_control` owns the standard
`simulation_interfaces` lifecycle API. The external overlay defines only
evaluator truth messages and hardware/task adapters; it must never be installed
in this uv environment.

Navigation is the first live module integration.  It reuses the existing
Tinker Humble Nav2/localization implementation without modifying the source
workspace, with a hardware-parity base facade, wheel-derived odometry, Livox
scan adapter, AMCL initialization, TF ownership guard, and public simulation
control gateway.  The exact build, two-terminal launch, interfaces, readiness
checks, and live results are documented in [`integration/NAVIGATION.md`](integration/NAVIGATION.md).

Example launch profiles:

```bash
./scripts/launch-isaac --sensor-profile physics-only --scenario empty
./scripts/launch-isaac --sensor-profile sensor-rich --profile parity \
  --scenario empty --seed 7 --ros
./scripts/launch-isaac --sensor-profile streaming --dds-profile lan
```

Streaming uses Isaac's headless WebRTC experience, explicitly disables GPU
physics, and uses a local process lock to
enforce one viewer per simulator instance. Restrict its ports to a trusted LAN
or VPN. Use SSH for installation, process management, logs, tests, and bags.
For the common remote-viewing case, `./scripts/launch-streaming` loads
`.deployment.env`, removes inherited system-ROS paths, and starts this streaming
profile in one command. Set `TINKER_SIM_DDS_PROFILE=local` to override its
default `lan` DDS profile, or `TINKER_SIM_ENV_FILE=/path/to/env` to load a
different deployment environment.

To stream the Tinker robot inside the committed RoboCup Arena 3 map instead of
opening an empty full Isaac UI, run:

```bash
./scripts/launch-arena-streaming
```

This standalone development viewer loads the current content-addressed
`robot.usd` and its colocated `map.yaml`, renders the map as visible collidable
walls, selects a deterministic arena overview camera, and listens on TCP 49100
and UDP 47998 for NVIDIA's WebRTC Streaming Client. The primary stream starts
at 1280x720 and uses Isaac Sim's supported dynamic-resize path to follow the
client window; spectator streams are not enabled. The launcher pumps Kit at a
bounded 10 Hz so WebRTC mouse, keyboard, and video are handled without coupling
the 120 Hz CPU-PhysX clock to ray-traced render latency; the guarded update
cannot step PhysX a second time. The occupancy-map cuboids are deliberately
kinematic static arena geometry, not loose props, so moving one in the stage
does not make it fall under gravity. It
also augments the robot's existing 20 kg low-mounted chassis ballast with 10 kg
(30 kg total, with proportionally scaled inertia). Base wheel velocity targets
are applied directly; navigation or another upstream controller is responsible
for acceleration and deceleration limits. It preserves Isaac's normal
single-session lifecycle: disconnecting the client
terminates the simulator and releases its ports. It
deliberately starts no external Humble/ROS processes; use the navigation
two-process workflow when ROS control is required. Optional launch arguments
such as `--duration 30` are forwarded to `launch-isaac`.

When the client network cannot return UDP packets directly to tkserver, carry
both WebRTC transports over the existing SSH connection. Keep the arena server
running in its tkserver shell, then run this on the GUI/client machine:

```bash
./scripts/connect-arena-streaming tinker@tkserver.example.net
```

The SSH destination is required and can be a hostname, SSH configuration alias,
or `user@host`; there is no client-machine-specific `tkserver` default. The
launcher needs only Bash, Python 3, and OpenSSH on Linux or macOS. It fetches the
matching relay helper from the authenticated server into a private temporary
directory, runs it locally, and removes it on exit, so the client does not need
a Tinker Sim checkout. To install the single launcher on another machine:

```bash
scp tinker@tkserver.example.net:/home/tinker/tinker-sim/6.0.1/scripts/connect-arena-streaming .
chmod +x connect-arena-streaming
./connect-arena-streaming tinker@tkserver.example.net
```

Use `--ssh-port` and `--identity-file` for connections not fully described by
the local SSH configuration. `--remote-root` changes the server checkout path;
it defaults to `/home/tinker/tinker-sim/6.0.1`. Key- or agent-based SSH access
is required because the connector deliberately uses non-interactive
`BatchMode=yes`.

The connector forwards TCP signaling and preserves UDP datagram boundaries
while framing media packets over the SSH byte stream. It waits up to 180
seconds for a process- and port-validated readiness marker written only after
the arena, robot, viewport, and Kit input loop have initialized; override this
with `--ready-timeout`. Do not open the NVIDIA client until the connector prints
`SSH WebRTC tunnel ready`.

Keep the connector running and configure NVIDIA's native client with Server
`2130706433`, Signal `49100`, and Stream `47998`. `2130706433` is the IPv4
numeric form of `127.0.0.1`: it never leaves the client machine, needs no
hosts-file entry, stays IPv4-only, and avoids a client 2.0 bug where literal
`localhost` or dotted `127.x` makes the client omit the explicit media endpoint
and bypass the UDP tunnel. The server also advertises `127.0.0.1` as its fixed
WebRTC media address, ensuring ICE uses the SSH relay instead of tkserver's
physical interface. Streaming ports and loopback values can be overridden with
`--client-host`, `--local-bind`, `--signal-port`, and `--media-port`. Close the
client first, then press Ctrl-C in the tunnel shell. Because UDP is encapsulated
in TCP, packet loss can produce head-of-line delay; this path favors reliable
access over minimum streaming latency.

After connecting with NVIDIA's native client, move the pointer completely
outside the streamed video once and then move it back in before clicking. The
2.0 client enables mouse/keyboard forwarding on the video's pointer-enter
event; if the pointer remains over the Connect button while that form is
replaced by the video, the first clicks can remain local to the client instead
of being sent to Isaac Sim.

## Vision hardware-parity cameras

The RTX camera graphs publish the same seven topics, encodings, and QoS that
`tk26_vision` expects from real RealSense drivers, so the existing vision
stack subscribes without modification. The contract's single source of truth
is [`simulation/sensors/hardware-parity.json`](simulation/sensors/hardware-parity.json)
(schema v2, `gateway_rtx_camera_publishers`), loaded fail-closed by
[`simulation/tinker_sim_isaac/camera_rig.py`](simulation/tinker_sim_isaac/camera_rig.py) —
including a per-camera `mount_rotation_wxyz`. The current artifact's xarm
optical frame is authored y-up (nonstandard), so the wrist camera uses the
rot_y(180) mount variant while the head camera uses rot_x(180).

| Topic | Content | Encoding |
| --- | --- | --- |
| `/camera/color/image_raw` | head color | rgb8 |
| `/camera/depth/image_raw` | head depth | 16UC1, millimetres |
| `/camera/color/camera_info` | head intrinsics | — |
| `/camera/xarm_camera/color/image_raw` | wrist color | rgb8 |
| `/camera/xarm_camera/aligned_depth_to_color/image_raw` | wrist depth | 16UC1, millimetres |
| `/camera/xarm_camera/color/camera_info` | wrist color intrinsics | — |
| `/camera/xarm_camera/aligned_depth_to_color/camera_info` | wrist depth intrinsics | — |
| `/camera/depth_registered/points` (optional, `--camera-pointcloud`) | organized head-camera cloud | point_step 16, x/y/z, real-driver layout |

Color and depth from one capture share one identical sim-time stamp. Every
publisher uses RELIABLE + VOLATILE + KEEP_LAST(10) QoS, matching the real
drivers after `tk26_vision`'s `realsense_qos.yaml` override: every
`tk26_vision` `CameraInfo` subscription is RELIABLE, so a best-effort
publisher would deliver zero messages to it, silently.

Two opt-in `launch-isaac` flags extend the stream under `--sensor-profile
sensor-rich`: `--arena-colors` paints the occupancy walls with a
deterministic six-hue palette from `tinker_sim_core.arena_palette` (used by
the live acceptance run below to prove hue delivery end to end), and
`--camera-pointcloud` additionally publishes the organized
`/camera/depth_registered/points` topic above, derived from the head camera.

Measured on this dev host (2x RTX 2080 Ti, driver 560.35.05 — below the
595.58.03 release baseline — with an arena-streaming session co-resident):
cameras hold 7.5–7.8 Hz wall-clock steady and `/clock` advances at roughly 30
steps/s, i.e. physics runs at about 25% of real time under `sensor-rich` —
the profile trades physics realtime for camera rate. The declared
`tick_rate_hz` of 30 in the contract is real-driver parity and is achieved in
full on a qualified host.

The head camera's spawn pose aims it at the sky (the tilt joint's default),
so its frames are uniform sky-gray at spawn — a correct render of that pose,
not a defect. Operational use commands the pan/tilt facade before reading
frames.

### Live acceptance runbook

Three terminals, all on `ROS_DOMAIN_ID=42` to isolate the run from the
default deployment domain (`25`):

Terminal A — launch the sim:

```bash
export ROS_DOMAIN_ID=42
./scripts/launch-isaac --sensor-profile sensor-rich --profile parity \
  --scenario empty --seed 7 --ros --arena-colors
```

(`--sensor-profile sensor-rich` implies `--ros` when it is omitted, printing
a note to that effect; it is passed explicitly above for clarity.)

Terminal B — the Humble vision overlay, same domain:

```bash
source /opt/ros/humble/setup.bash
source ~/tk25_ws/install/setup.bash
export ROS_DOMAIN_ID=42
ros2 run vision_util get_image   # optional — see the defect note below
```

Terminal C — the acceptance test:

```bash
TINKER_SIM_VISION_LIVE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  python3 -m pytest -v tests/ros_humble/test_vision_get_image_live.py
```

This was run and recorded (3/3 passed, `ROS_DOMAIN_ID=42`): direct RELIABLE
subscriptions decode both cameras with `tk26_vision` conventions, and the
wrist frame carried all six palette hues (45.3% chromatic pixels). Evidence
is under `reports/vision-roundtrip/` (gitignored, host-local).

**Real defect found in `~/tk25_ws/src/tk26_vision`** (documented here for the
record; out of scope to patch from this repo): `vision_util`'s `get_image`
and `get_point_cloud` register `async def` callbacks directly on
`message_filters`' `ApproximateTimeSynchronizer` (`get_image.py:49,65,77,81`,
`get_point_cloud.py:43,66,100,104`). Humble's `message_filters` invokes
callbacks synchronously, so those coroutines are never awaited, the node's
cached frames never update, and both services always answer "No camera
data" — on real hardware too, not only in sim. The sim run still proved that
delivery and stamp-pairing both work: the node's own "coroutine was never
awaited" `RuntimeWarning`s fired for both cameras. The acceptance test's
third case probes this service directly and will fail loudly — prompting a
test upgrade — once the node is fixed upstream.

Status: development-validated with a recorded live round-trip; **not
release-qualified**.

## RoboCup 2026 arena

`--arena rcw2026` replaces the procedurally generated occupancy world with an
imported RoboCup 2026 @Home-style arena: 20 furniture models across 32
placements (kitchen table, sofa, shelf, sink, washing machine, and so on)
plus the room's wall layout, converted from the real upstream Gazebo world.
A companion importer publishes ten graspable YCB tabletop objects (cracker
box, sugar box, potted meat can, mustard bottle, pudding box, tomato soup
can, banana, bleach cleanser, bowl, mug) for pick/place work. Both are
pulled from pinned upstream commits and published as immutable,
content-addressed asset artifacts, mirroring the existing robot-artifact
convention: `artifacts/arena/rcw2026/<identity>/` and
`artifacts/objects/ycb/<identity>/`, each with a `current.json` pointer, a
`manifest.json`, a `source-lock.json` (per consumed upstream file: relative
path, size, sha256), and an `ATTRIBUTION.md`. Re-running an importer against
unchanged upstream content and config is a byte-identical no-op (proven live
by two consecutive runs of each importer publishing under the same identity:
the second run reports `created=False` and atomically rewrites `current.json`
with identical bytes).

Provenance: the arena is converted from
[`TeamSOBITS/sobits_gazebo_worlds`](https://github.com/TeamSOBITS/sobits_gazebo_worlds)
at `feature/hri`, pinned to commit `293b4057d26a673c3f09ff7d8f3118234d42ba24`
(BSD-3-Clause; `ATTRIBUTION.md` carries the upstream `LICENSE` text plus the
per-file source records). The YCB objects are converted from
[`TeamSOBITS/tmc_wrs_gz`](https://github.com/TeamSOBITS/tmc_wrs_gz) at
`jazzy-devel`, pinned to commit `48157eec99bfc50f8d24ad95736d4d10bb344c14`
(`ATTRIBUTION.md` carries a CC BY 4.0 attribution block naming the
Yale-CMU-Berkeley (YCB) Object and Model Set, the license obligation for
that content, plus the Clear BSD text for the `tmc_wrs_gz` wrapper itself).
Both importers verify the checkout's `HEAD` against the pinned commit and
fail closed on a mismatch before converting anything.

Run either importer from the simulator venv, on a host with network access
(to clone the pinned commit) and a GPU (Isaac Sim Kit conversion), with the
same `TINKER_ACCEPT_OMNIVERSE_EULA=Y` gate as the rest of this README's
`.deployment.env`-sourcing bootstrap (see "Online bootstrap" above). Both
importers call `SimulationApp` directly rather than going through the
deploy CLI's `launch` command, so they bypass the one place
(`tools/tinker_sim_deploy/cli.py`) that bridges a confirmed
`TINKER_ACCEPT_OMNIVERSE_EULA=Y` into the `ACCEPT_EULA`/`OMNI_KIT_ACCEPT_EULA`
variables Kit itself reads; export those two explicitly, once the TINKER
gate is confirmed, or Kit prompts interactively and a headless run aborts
at EOF:

```bash
source .deployment.env
export ACCEPT_EULA=Y OMNI_KIT_ACCEPT_EULA=YES  # only after TINKER_ACCEPT_OMNIVERSE_EULA=Y above
./.venv/bin/python tools/arena_import.py --config config/arena-import.json
./.venv/bin/python tools/ycb_import.py --config config/ycb-import.json
```

`config/arena-import.json` carries the pin, the furniture allowlist, a
`model_skiplist` for benign non-furniture includes, `bounds_check_exceptions`
for the two models whose upstream SDF deliberately under-sizes the collision
box (see Status below), the placement-surface definitions, and
`bounds_tolerance_m`. `config/ycb-import.json` carries the pin, `models_root`,
and the object allowlist. Both accept `--checkout <path>` to reuse an
existing pinned checkout instead of cloning fresh; `tools/arena_import.py`
additionally accepts `--report-bounds` to print per-model measured bounds and
exit without publishing. Every Kit/pxr call is isolated behind a converter
adapter in
[`tools/tinker_sim_deploy/arena_convert.py`](tools/tinker_sim_deploy/arena_convert.py),
so the importers' own orchestration logic is unit-tested under plain system
Python with no GPU or Isaac Sim installed; the adapter itself is exercised
only by the live import.

The arena's `map.yaml`/`map.pgm` (ROS `map_server` PGM/YAML, resolution
0.05m, trinary mode) is derived, not hand-authored: it rasterizes the same
pinned world file's wall and furniture collision footprints, sliced at the
tinker2 Livox sensor's mounted height (read from the robot URDF's
`livox_joint` origin) — the pinned world file is the single source of truth
for both the 3D scene and the 2D navigation map. `placement.json` records
world-frame placement surfaces (for example `kitchen_table#top`, the
`rcw26_` model-id prefix stripped) for tabletop object spawning.

Select the arena on launch with `--arena rcw2026`, forwarded like any other
`validation/run_sim.py` flag through `./scripts/launch-isaac`. `--arena` is
mutually exclusive with `--map` (`--arena and --map are mutually exclusive`)
and with `--arena-colors` (`--arena-colors applies only to occupancy cuboid
walls` — that flag colors the procedurally generated occupancy walls, which
the imported arena's real geometry replaces), both enforced at
argument-parse time. A scenario selects the same arena declaratively via
`world: {"mode": "arena", "arena": "rcw2026"}`; validated fail-closed by
`tinker_sim_core.scenario.validate_world_selection`, which requires the
declared `arena` to be a non-empty string matching the launcher's `--arena`
value exactly (missing, mismatched, or combined with a `uri` key all raise
`ValueError`) — `mode: "current"`, or `mode` absent, is unaffected regardless
of `--arena`.

The robot's default spawn is world (0, 0), which in the rcw2026 arena lies
inside `shelf_02`'s physical and rasterized footprint: the robot stands under
the shelf plate, the planar lidar sees a self-hit/obstruction ring, wheel
odometry accumulates contact slip, and AMCL is initialized inside an occupied
map cell. Navigation work in this arena therefore passes
`--spawn-xy=X,Y` (world metres; use the `=` form — argparse treats a
separate `-2.0,-2.0` token as an option string) to place the robot on a free
map cell, e.g. `--spawn-xy=-2.0,-2.0` (1.0 m clearance on the derived map).
The override is validated fail-closed (two finite comma-separated numbers)
and requires a profile that loads the robot backend, like `--arena` itself.
On the Humble side, `./scripts/launch-humble navigation
map_yaml:=/abs/path/to/artifacts/arena/rcw2026/<identity>/map.yaml` points
AMCL's map server at the arena's derived map instead of the robot artifact's
colocated default; the override fails closed on a missing file.

The streaming wrapper `scripts/launch-arena-streaming` and `--livestream`
forwarding in the deploy CLI are **not** part of this branch — they live in
separate, uncommitted work. The headless streaming smoke recorded below was
therefore run by invoking `validation/run_sim.py` directly, not through
either wrapper.

Offline bundling does not pick up arena/object USDs automatically: register
`arena.usd`'s path and sha256 under the optional `generated_arena_usds`
array, and each object's `object.usd` path and sha256 under
`generated_object_usds`, in `artifacts/asset-manifest.json`
([`tools/tinker_sim_deploy/assets.py`](tools/tinker_sim_deploy/assets.py)).
Both groups are optional — an absent group is fine — but every entry is
hash-verified when either group is present.

Validation performed on this branch (development-validated only):

- unit suites are green, with a stable failing/erroring name-set matched
  against this repo's pre-existing environmental failures (see "Developer
  verification" below);
- both artifacts hash-verify clean (`verify_asset_artifact(...) == []`) and
  re-import is a proven byte-identical no-op;
- visual/collision AABB agreement was spot-verified: within 1.4mm on three
  YCB objects (cracker box, mug, bowl); arena furniture bounds were checked
  against the upstream SDF within the configured 0.02m tolerance, with two
  documented per-model exceptions (`rcw26_door`, `rcw26_sink`) whose upstream
  SDFs deliberately under-size the collision box (a trimmed door-panel depth
  and a floor-anchored sink height, both for gripper-reach affordance, not
  data errors);
- a headless streaming smoke (`validation/run_sim.py --sensor-profile
  navigation-parity --profile parity --scenario empty --seed 7 --headless
  --livestream --arena rcw2026 --duration 45`) ran the full 45 simulated
  seconds to a clean exit, with only headless-windowing/driver diagnostic
  warnings in the log and no importer-scratch-path leakage;
- a live end-to-end `./scripts/launch-arena-streaming --arena rcw2026` run
  (2026-08-18) reached streaming readiness — ready file written, TCP 49100
  listening, `viewport_ready: true`, arena `robocup-arena3` with 38
  colliders loaded — and stayed up awaiting a client;
- sensor-rich camera imagery of the arena's furniture: head and wrist
  hardware-parity color/depth frames captured live against `--arena
  rcw2026` (shelf close-ups plus a base-rotation panorama showing the TV
  cabinet, trash bin, door, tiled floor, and plant), with per-frame content
  statistics — `reports/arena-sensor-rich-2026-08-18/`; the wrist camera's
  mount/intrinsics and arm-following viewpoint were verified separately in
  `reports/arena-arm-camera-2026-08-18/`;
- AMCL convergence on the derived map: with the spawn moved to a free cell
  (`--spawn-xy=-2.0,-2.0`) and the Humble stack pointed at the arena map
  (`map_yaml:=...`), AMCL locked to physics-truth within 0.07 m after
  seeding and, over truth-validated gentle-motion runs, contracted to
  position variance 0.035/0.050 m² (std ~0.2 m) at 0.14 rad yaw error —
  `reports/arena-amcl-2026-08-18/SUMMARY.md`, which also records two real
  findings: the default (0, 0) arena spawn sits inside `shelf_02`'s
  footprint (hence the new `--spawn-xy` override), and sustained in-place
  skid-steer rotation accumulates wheel-odometry yaw slip that drags the
  filter (a base-odometry characteristic, not a map defect).

- physics interaction (an object resting on arena furniture): the
  pick-deliver-place `delivery_object` (0.08 m cube), spawned via the
  standard `spawn_entity` path at its declared z=0.8 pose, fell onto and
  came to rest statically (zero twist across 387 truth samples) on a 0.5 m
  board of `rcw26_shelf` —
  `reports/arena-scenario-spawn-2026-08-19/object-spawn-verification.md`;
- scenario entity spawning in the arena: `scenario_runner` executed
  find-and-approach-person and pick-deliver-place against an `--arena
  rcw2026` sim with every operation accepted; the person capsule and task
  cube spawn at their exact declared world poses (verified via
  `/get_entities`/`/get_entity_state` and `expected_objects` truth
  correlation), and spawned entities are live rigid bodies
  (`/set_entity_state` round-trips) —
  `reports/arena-scenario-spawn-2026-08-19/`. Caveats: nothing implements
  scenario `events` (the person's `actor_path_start` walk never runs), and
  scenario poses were authored for the procedural world (the arena has no
  pedestal at the object spawn; the person's declared pose sits in
  furniture-dense space).

Not yet validated (open):

- textured-frame visual confirmation by a human viewer (the streaming
  session above is up for exactly this; connect with NVIDIA's client).

Known arena limitations (development findings, 2026-08-18):

- the default robot spawn (0, 0) lies inside `shelf_02`'s physical and
  rasterized footprint — use `--spawn-xy` for navigation work (see launch
  docs above);
- head `pan_joint`/`tilt_joint` drives inherit the URDF's 1.0 Nm effort cap
  (the backend overrides `effort_limit_sim` for arm and wheels only), so
  commanded pan/tilt motion creeps at ~0.01 rad/s: re-aim the camera by
  rotating the base, or extend the backend's `head` actuator group;
- under `sensor-rich`, the planar lidar cloud includes a dense self-hit
  ring at ~0.3 m that dominates the `/scan` conversion; AMCL/navigation
  validation used `navigation-parity`'s deterministic raycaster instead.

Status: development-validated only, **not release-qualified**.

## Offline provisioning

Run a complete online bootstrap first. Add generated robot USDs and their
checksums to `artifacts/asset-manifest.json` using the tracked portable template
at `config/asset-manifest.example.json`; do not reference the surrounding ROS
workspace. Both generated-USD and warmed asset groups must be non-empty, present
on disk, and hash-correct. Whole-robot bundles also require a verified immutable
`artifacts/robot/tinker2/current.json` generation. The only intentional
artifact-free mode is the explicitly selected `bundle-restore --profile
physics_only` validation profile; whole-robot creation and restore fail closed
when the verified generation is absent.
Then:

```bash
./scripts/create-offline-bundle artifacts/tinker-sim-6.0.1.tar.gz
```

Bundle creation first creates a fresh temporary environment using
`uv sync --frozen --offline`. This proves that no package is missing from the
cache. It refuses to package an unverified Isaac Lab tree or an unwarmed Isaac
cache. The deterministic archive contains:

- the lock and environment definition;
- the exact uv executable and managed Python;
- the populated, versioned uv cache;
- the pinned Isaac Lab checkout;
- warmed Isaac caches and generated assets;
- this deployment tooling and ROS contract workspace; and
- a per-file SHA-256 manifest.

On an offline server:

```bash
mkdir -p /srv/tinker-sim/6.0.1
./scripts/restore-offline-bundle \
  /media/bundle/tinker-sim-6.0.1.tar.gz /srv/tinker-sim/6.0.1
cd /srv/tinker-sim/6.0.1
export PATH="$PWD/bin:$PATH"
export TINKER_ACCEPT_OMNIVERSE_EULA=Y
./scripts/bootstrap --mode offline
```

Restore requires an empty destination and verifies every file before use.
Offline bootstrap runs `uv sync --frozen --offline` and never falls back to
the network.

## Miniconda recovery only

Miniconda may create a Python 3.12.13 environment if uv's managed Python cannot
be bootstrapped. It is not a second dependency definition:

```bash
./scripts/tinker-sim conda-export artifacts/requirements-from-uv-lock.txt
conda create -n tinker-isaac-recovery python=3.12.13
conda activate tinker-isaac-recovery
python -m pip install -r artifacts/requirements-from-uv-lock.txt
```

Always regenerate the requirements file from the committed lock. Never edit
or maintain it independently.

## Simulation contract and rollout

`contracts/simulation.yaml` defines the standard `simulation_interfaces`
surface and evaluator-only `/sim/truth/*` topics. There are deliberately no
custom `/sim/control/*` or `/sim/scenario/*` aliases.
`simulation/tinker_sim_core` provides:

- a Gazebo/Isaac backend protocol;
- deterministic single-owner command arbitration;
- hardware-parity versus hidden-truth separation; and
- postcondition scoring for person approach, pick/deliver/place, and reception
  seat assignment.

The parity profile is the only input to navigation, perception,
manipulation, decision, and VLA software. Only the evaluator may receive
truth. The scenario evaluators reject a task server's claimed success when the
world postcondition is false.

The Tinker 2 USD/URDF/map export, navigation gateway, external FJT controller,
xArm/gripper/pan-tilt facades, deterministic scenario orchestration, and audio
fixtures are implemented as content-addressed artifacts under `artifacts/`.
The artifact exporter derives only the canonical URDF metadata needed by the
strict planning contract: it adds the zero `world -> base_link` mount, names
the existing `base_link -> link_base` mount, and records a state-only
`drive_joint` entry. The USD is copied byte-for-byte and remains the simulator
physics artifact. The canonical URDF bytes and canonicalizer algorithm version
are part of the artifact identity; the manifest, immutable per-artifact
source lock, and `current.json` record the complete generation and all payload
hashes. Publication fsyncs files/directories, atomically claims the immutable
artifact directory, and replaces `current.json` as the sole commit point; a
crash before that replacement can leave only an ignored orphan generation and
cannot mix a pointer with a mutable root lock. Export publication is serialized
by a nonblocking inter-process lock; a normal concurrent exporter fails without
recovering the active exporter's staging directory. The exporter reads the
external workspace once, derives the immutable lock in memory, and only accepts
an already captured shared lock when it exactly matches; a failed export never
creates or rewrites that shared lock. `workspace-lock` is the separate explicit
atomic capture operation.

### Portable deployment inputs

The simulator checkout is relocatable. Set `TINKER_SIM_ROOT` when invoking a
launch wrapper from an installed/copy deployment; launch files otherwise derive
the project root from their installed/source package location. Set `TINKER_WS`
to the external Humble workspace, or pass `tinker_workspace:=...` explicitly.
There is no host-specific workspace default. `scripts/launch-humble` fails
clearly when `TINKER_WS` is absent. The external workspace is read-only runtime
input and is never bundled as an absolute provenance path.

The source lock schema is portable and exact: it contains robot `tinker2`, a
SHA-256 source identity derived from the fixed source contract plus the ordered
relative path/size/SHA-256 records, and no fake repository revision. The
immutable per-artifact lock is authoritative for that generation; manifest
canonicalization, USD provenance, source records, payload hashes, and the
content-addressed identity are cross-checked against it by every consumer.
Use the project-managed interpreter, for example
`TINKER_WS=/path/to/tk25_ws ./.venv/bin/python tools/deploy.py artifact-export`,
rather than editing an artifact directory in place.

Vision, manipulation, decision, and VLA vertical slices still require live
qualification and must not be represented as release-qualified. Controller
gains and sensor/base noise still require synchronized robot calibration;
qualification fails explicitly while
`simulation/calibration/tinker2-missing.json` remains uncalibrated.

### OMPL overlay acceptance contract

`integration/ompl-overlay-contract.json` is the deterministic acceptance
contract for the reviewed OMPL overlay (Tasks 3-7): canonical model-bundle
producer/preflight, integrated eight-joint state contract, atomic fixture
PlanningScene adapter, staged integrated OMPL overlay with typed integrated
readiness, and the deterministic OMPL plan-only smoke.  The contract is
canonical JSON (schema version 1, sorted keys, minimal separators) with exact
repository/commit identities, the production overlay
(`mobile_bringup` / `manipulation_planning_task_only.launch.py`, 18-argument
contract, literal-false `use_cumotion_*` compatibility values), the provider
manifest (raw + canonical identities, distinct persistent/one-shot/controller/
publisher classes), the model-bundle schema/artifact hashes and full eight-link
touch set, the typed action/service/topic contract (including
`/isaac_joint_commands` depth 50 and the external future
`/tinker_integrated_gate_executor` publisher ownership), fixture/scenario
identities, Task 6/7 evidence, and build commands (`tkbuild` /
`scripts/build-humble-overlay`, never raw colcon).  It does not itself prove
live OMPL or authorize cuMotion.  `tests/test_provenance.py` recomputes every
derived hash/contract from the real source and fails on mutations.  The two
repository-local source-lock files are Task 9 only.  See
`integration/MANIPULATION.md` for the operator workflow.

## Integrated OMPL qualification CLI

`validation/integrated_qualification.py` orchestrates the integrated OMPL
qualification Gates A-F.  It is an offline orchestration layer over the
six-gate core suite, the offline static closure, and the live C/D/E scenario
attempts, with an offline Gate-F evidence rebuild/verify.  Task 10's own tests
and this documentation make no live Gate F/OMPL/cuMotion claim.

Use one consistent suite path variable:

```bash
SUITE_DIR=outputs/integrated/integrated-ompl-seed-7
```

`SUITE_DIR` is passed as the runner's `--attempt-root`.  It is the exact
integrated suite directory; nothing is silently appended below it.  A repeated
qualification run must choose a fresh `--attempt-root`; never merge a new run
into an old suite.  The six-gate Stage-A core suite runs in the sibling root
`<SUITE_DIR>-core/` (here `outputs/integrated/integrated-ompl-seed-7-core/`),
outside the integrated Gate-F index.

Gate A (six-gate core suite):

```bash
./.venv/bin/python validation/integrated_qualification.py \
  --attempt-root "$SUITE_DIR" --stage A
```

Gate B (offline static closure; `--offline` is an explicit compatibility flag
for the already-offline B implementation and must not make any live stage
offline or bypass checks):

```bash
./.venv/bin/python validation/integrated_qualification.py \
  --attempt-root "$SUITE_DIR" --stage B --offline
```

Gate C (three OMPL plan-only scenarios):

```bash
./.venv/bin/python validation/integrated_qualification.py \
  --attempt-root "$SUITE_DIR" --stage C
```

Gate D (six execute scenarios):

```bash
./.venv/bin/python validation/integrated_qualification.py \
  --attempt-root "$SUITE_DIR" --stage D
```

Gate E (eight pick-place scenarios):

```bash
./.venv/bin/python validation/integrated_qualification.py \
  --attempt-root "$SUITE_DIR" --stage E
```

Gate F (standalone offline rebuild/verify):

```bash
./.venv/bin/python validation/integrated_qualification.py \
  --attempt-root "$SUITE_DIR" --stage F
```

All stages (`--stage all`):

```bash
./.venv/bin/python validation/integrated_qualification.py \
  --attempt-root "$SUITE_DIR" --stage all
```

The standalone Gates A through F above and `--stage all` are alternatives.
`--stage all` must use a fresh suite path and must not be run against a suite
that already has write-once A-E stage records.

Independent verifier replay against the selected immutable C/D/E attempt
directory.  Every C/D/E scenario runs in a newly created immutable attempt
directory named `STAGE-<scenario>-<invocation>-<counter>`.  The selection
below finds the exact single immutable matching attempt under `$SUITE_DIR`,
fails unless exactly one match is found, and binds it to `ATTEMPT_DIR`:

```bash
ATTEMPT_DIR="$(find "$SUITE_DIR" -maxdepth 1 -type d \
  -name 'C-qualification-moveit-plan-joint-*' | sort)"
test "$(printf '%s\n' "$ATTEMPT_DIR" | sed '/^$/d' | wc -l)" -eq 1
./.venv/bin/python validation/integrated_gate_verifier.py \
  --scenario qualification-moveit-plan-joint \
  --attempt-dir "$ATTEMPT_DIR" \
  --config simulation/qualification/integrated-ompl.json
```

Integrated contact-sheet regeneration against `$SUITE_DIR`:

```bash
./.venv/bin/python validation/integrated_contact_sheets.py --suite-dir "$SUITE_DIR"
```

Standalone evidence-index rebuild and Gate-F validation (writes
`evidence-index.json` and `qualification-summary.json`):

```bash
./.venv/bin/python validation/integrated_evidence_index.py \
  --suite-dir "$SUITE_DIR" --summary "$SUITE_DIR/qualification-summary.json" \
  --validate
```

Failed-attempt retention/rerun rule: never delete or reuse a failed, stale, or
old attempt or suite.  Every C/D/E scenario runs in a freshly created immutable
attempt directory; repeated allocation yields distinct preserved paths.  To
rerun, choose a fresh suite path (a fresh `--attempt-root`); never merge a new
run into an old suite.

Bounded build command (never raw colcon).  The wrapper ignores CLI args and
internally executes colcon with `--parallel-workers 2`:

```bash
MAKEFLAGS='-j2 -l2' ./scripts/build-humble-overlay
```

Truthful scope:

- The runtime config has three source-lock roles
  (`simulator_overlay` / `production` / `qualification_tooling`).  The
  qualification-tooling source-lock role is created only after Task 10 is
  review-clean, in a separate lock-only commit, and only before live attempts.
- No live Gate F/OMPL/cuMotion claim comes from Task 10's offline tests.
- `_image_stats` thresholds still require live RTX calibration.
- cuMotion remains prohibited until Task 37's live OMPL qualification passes.

## Developer verification

The non-GPU tests run with the Ubuntu system Python:

```bash
python3 -m unittest discover -s tests -v
uv lock --check
./scripts/build-humble-overlay
```

The focused OMPL-overlay provenance suite runs under the simulator venv.  ROS
plugin discovery may auto-load the Humble `launch_pytest` plugin, which can fail
collection with `ModuleNotFoundError: No module named 'lark'` on hosts that do
not provide that dependency; disabling plugin autoload is the defensive
reproducible invocation (it does not claim collection necessarily fails on any
specific host):

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ./.venv/bin/python -m pytest -q tests/test_provenance.py
```

This does not hide the pre-existing pinned-uv environment failure
(installed `uv 0.12.0` vs pinned `uv 0.10.8`), which is reported visibly.

Release qualification additionally requires a clean server, the full online
and offline bootstrap paths, ROS cross-process contract tests, RTX camera and
LiDAR initialization, and a 60-second continuous WebRTC connection using
NVIDIA's native Ubuntu Omniverse Streaming Client (never a downloaded browser).

References:

- [Isaac Sim Python installation](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_python.html)
- [Isaac Sim requirements](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html)
- [Isaac Sim ROS integration](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_ros.html)
- [Isaac Lab installation](https://isaac-sim.github.io/IsaacLab/develop/source/setup/installation/index.html)

## Changelog

- 2026-08-21 (physics step cadence for live parity — "optimize its physics
  step frequency for more live parity"): Split the simulator's single rate
  into a PhysX solver rate (`TINKER_SIM_PHYSICS_HZ`, default 120, unchanged)
  and an opt-in control rate (`TINKER_SIM_CONTROL_HZ`, default = physics
  rate) at which Isaac Lab, target writes, wheel slew, gateway publish and
  `/clock` run; each control step runs explicit `physics_hz / control_hz`
  solver substeps of the validated 1/120 s. Measured (gpsr-rcw2026, RTF):
  physics-only 0.75 -> 1.18 at control 60 (1.88 at 30) with the robot root
  within 0.5 mm of the default after 10 s, versus 5 mm drift for the old
  `TINKER_SIM_PHYSICS_HZ=60` advice; sensor-rich with ROS and 15 Hz cameras
  0.35 -> 0.40 by default and 0.51 at control 60 (0.60 at 30) once combined
  with the lidar fix below. Found along the way via a new
  `publish_breakdown_ms` in the `TINKER_SIM_PROFILE=1` output that the
  development-lidar ray-cast cost ~35 ms per lidar frame (~350 ms per
  simulated second); `OccupancyMap.raycast_many` now casts all 181 rays
  vectorised with chunked early exit, proven bit-identical to the scalar
  loop (`tests/test_occupancy_raycast_vectorised.py`), ~2-5 ms per frame.
  Also opt-in `TINKER_SIM_SOLVER_POSITION_ITERATIONS` /
  `TINKER_SIM_SOLVER_VELOCITY_ITERATIONS` (robot USD authors 32 / 1;
  fidelity-affecting, not for evidence runs). Measured non-results: omni.physx
  `simulate(elapsed)` does not substep (a `timeStepsPerSecond` override
  silently integrates at the control dt — caught by a step-count check and
  replaced by explicit substeps), and PhysX worker-thread count (4/8/16)
  changes nothing. Runbook: "Control cadence under a live stack".

- 2026-08-19 (arena scenario entity spawning — "verify person and object
  spawn"): Verified the standard scenario spawn path inside `--arena
  rcw2026`: `scenario_runner` ran find-and-approach-person and
  pick-deliver-place with every operation accepted; the person capsule and
  delivery cube spawn at their exact declared world poses (checked via
  `/get_entities`, `/get_entity_state`, and the physics-truth
  `expected_objects` prim correlation), spawned entities are live rigid
  bodies (`/set_entity_state` round-trips), and the delivery cube fell from
  its declared z=0.8 onto a 0.5 m `rcw26_shelf` board and came to rest
  statically — closing the "object resting on arena furniture" physics
  item. Evidence: `reports/arena-scenario-spawn-2026-08-19/` (gitignored,
  dev host). Findings recorded there: scenario `events` (actor paths) are
  not implemented by any component, scenario poses were authored for the
  procedural world rather than the arena, and an idle base coasts ~1.5 m
  after a rotation sweep before settling.

- 2026-08-18 (RoboCup arena validation evidence + spawn/map overrides —
  "run the arena streaming launch and capture the outstanding validation
  evidence"): Ran `./scripts/launch-arena-streaming --arena rcw2026` end to
  end to streaming readiness; captured sensor-rich head/wrist camera
  evidence of the arena furniture
  (`reports/arena-sensor-rich-2026-08-18/`), verified the wrist camera's
  mount, intrinsics, and arm-following viewpoint
  (`reports/arena-arm-camera-2026-08-18/`), and validated AMCL on the
  derived arena map against `/sim/internal/physics_truth`
  (`reports/arena-amcl-2026-08-18/SUMMARY.md`). Two defects surfaced and
  were addressed: the default arena spawn (0, 0) sits inside `shelf_02`'s
  physical/rasterized footprint — fixed by an opt-in, fail-closed
  `--spawn-xy X,Y` override threaded `deploy CLI -> run_sim.py ->
  IsaacWholeRobotBackend.spawn_xy` (defaults unchanged;
  `tests/test_spawn_override.py`, 11 tests) — and
  `navigation.launch.py` hard-wired AMCL's map to the robot artifact's
  colocated `map.yaml` — fixed by an optional fail-closed `map_yaml:=`
  launch argument (`resolve_map_yaml`;
  `tests/test_navigation_launch_map.py`, 5 tests). Documented findings:
  head pan/tilt effort starvation (1.0 Nm URDF cap, no backend
  `effort_limit_sim` override), sensor-rich lidar self-hit ring, and
  skid-steer rotational odometry yaw slip. Focused arena/CLI suites pass
  78/78; full `unittest discover` failing set is environmental/external
  only (system PIL lacks `Image.Resampling`, `torch`/`tinker_sim_bridge`
  unavailable to system Python, and `test_provenance` pins a
  `tk25_manipulation` commit unreachable while that workspace sits on
  `collision-aware-grasp`), plus the pre-existing uncommitted
  `tests/test_base_velocity_slew.py`, which imports a
  `slew_velocity_target` that no commit implements (in-progress task50
  work). Textured-frame human confirmation remains the open item; the
  streaming session is left up for it.

- 2026-08-17 (RoboCup 2026 arena import, Tasks 1-11 — "import, launch, and
  document the RoboCup 2026 arena and YCB objects"): Added a RoboCup 2026
  arena importer (`tools/arena_import.py`, `config/arena-import.json`,
  pinned `TeamSOBITS/sobits_gazebo_worlds@feature/hri`
  `293b4057d26a673c3f09ff7d8f3118234d42ba24`) and a YCB tabletop-object
  importer (`tools/ycb_import.py`, `config/ycb-import.json`, pinned
  `TeamSOBITS/tmc_wrs_gz@jazzy-devel`
  `48157eec99bfc50f8d24ad95736d4d10bb344c14`), sharing pin-verification/
  clone/source-record helpers in new `tools/tinker_sim_deploy/import_common.py`
  and a new Kit conversion adapter `tools/tinker_sim_deploy/arena_convert.py`.
  New library modules parse the pinned Gazebo world and its model colliders
  (`tools/tinker_sim_deploy/arena_world.py`), rasterize a derived
  `map.yaml`/`map.pgm` from the wall/furniture collision footprints sliced
  at the tinker2 Livox scan height (`arena_map.py`), and record world-frame
  placement surfaces (`arena_surfaces.py`, `placement.json`). Publication
  reuses the existing content-addressed artifact machinery
  (`arena_artifact.py`) under `artifacts/arena/rcw2026/` and
  `artifacts/objects/ycb/`, each with a `current.json` pointer,
  `source-lock.json`, and `ATTRIBUTION.md` (arena: SOBITS BSD-3-Clause;
  YCB: CC BY 4.0 attribution to the Yale-CMU-Berkeley Object and Model Set
  plus the Toyota `tmc_wrs_gz` wrapper's Clear BSD).
  `validation/run_sim.py` gained a `--arena` flag (mutually exclusive with
  `--map` and with `--arena-colors`) and
  `simulation/tinker_sim_core/scenario.py` gained `validate_world_selection`,
  validating a scenario's `world: {"mode": "arena", "arena": "rcw2026"}`
  declaration fail-closed against the launcher's `--arena` value;
  `simulation/tinker_sim_isaac/backend.py` gained an opt-in `arena_artifact`
  construction path. `tools/tinker_sim_deploy/assets.py` gained optional,
  validated-when-present `generated_arena_usds`/`generated_object_usds`
  asset-manifest groups for offline bundling. Both importers were live-run
  on the dev host to a hash-verified artifact
  (`verify_asset_artifact(...) == []`) proven byte-identical on re-import;
  visual/collision AABB agreement was spot-verified (within 1.4mm on three
  YCB objects; arena furniture within the configured 0.02m of its
  SDF-declared collision box, with two documented per-model exceptions,
  `rcw26_door`/`rcw26_sink`); a headless streaming smoke with `--arena
  rcw2026` ran 45 simulated seconds to a clean exit, invoking
  `validation/run_sim.py` directly on this dev host — the
  `scripts/launch-arena-streaming` wrapper and the deploy-CLI's
  `--livestream` forwarding are separate, uncommitted work and are not part
  of this branch. Textured-frame visual confirmation by a human viewer,
  physics interaction with arena furniture, sensor-rich camera imagery of
  the arena, and AMCL convergence on the derived map remain unvalidated.
  New `README.md` "RoboCup 2026 arena" section documents provenance, import
  commands, launch usage, and this status. Status: development-validated
  only, **not release-qualified**.

- 2026-08-05 (integrated qualification Task 10 — "document integrated
  qualification CLI"): Documented the offline integrated OMPL qualification
  CLI with one consistent `SUITE_DIR` variable and exact standalone command
  blocks for Gates A-F (Gate B with the explicit `--offline` compatibility
  flag), `--stage all`, the deterministic single-match verifier replay
  (`ATTEMPT_DIR` selection that fails unless exactly one immutable matching
  attempt exists), the integrated contact-sheet regeneration, and the bounded
  `MAKEFLAGS='-j2 -l2' ./scripts/build-humble-overlay` build command (the
  wrapper ignores CLI args and internally executes colcon with
  `--parallel-workers 2`).  Standalone Gates A-F and `--stage all` are
  documented as alternatives; `--stage all` must use a fresh suite path and
  must not be run against a suite that already has write-once A-E stage
  records.  The evidence-index command writes `qualification-summary.json` via
  `--summary`.  The documentation states the fresh-suite retention rule (never
  delete/reuse; choose a fresh `--attempt-root`), the three source-lock roles
  with the qualification-tooling role created only after review-clean Task 10,
  and the live-only caveats (no live Gate F/OMPL/cuMotion claim from Task 10
  offline tests, `_image_stats` still requires live RTX calibration, and
  cuMotion remains prohibited until Task 37 live OMPL passes).  New
  `tests/test_integrated_acceptance_docs.py` asserts the exact command
  blocks/paths, fresh-suite retention wording, bounded build command,
  three-lock sequence, live-only caveats, and cuMotion prohibition verbatim in
  `docs/acceptance.md`, `integration/MANIPULATION.md`, and `README.md`.  No
  build, no live Isaac/ROS/GPU/cuMotion, and no source-lock file changed; the
  future qualification-tooling source-lock role remains absent until Task 36.

- 2026-08-04 (integrated qualification Task 9, fix round 5 — "final narrow
  production-suite closure"): Closed the last offline production-suite residuals
  so the contact-sheet and Gate-F tooling now supports the full 17-scenario
  production suite shape.  `validation/integrated_contact_sheets.py::_all_bound_capture_entries` now
  tolerates shared event labels (the same four cancel labels across three cancel
  scenarios and the same four safety labels across two safety scenarios) and
  selects exactly one deterministic representative capture per event by the rank
  canonical event order, then scenario, attempt, preferred camera
  (`overview` before `manipulation_closeup`, then other camera names), then path
  (F5.1).  `validation/integrated_evidence_index.py` scopes the duplicate
  `(event, camera)` keyframe-identity check per attempt directory so
  multi-scenario label sharing is normal while a duplicate within the same
  `(scenario, attempt, event, camera)` stays fail-closed; `capture_latency_frames`
  and both `execution_event_sequence` / `source_execution_event_sequence` are
  now mandatory positive integers that must be equal — missing either side is a
  critical diagnostic and the real `raw = requested + latency` relation is
  retained, never requested==raw (F5.2); and the Gate-F suite must contain
  exactly one categorized overlay-contract artifact, failing on more than one
  even with identical production/simulator identities while contradictory
  duplicates keep the sharper contradiction reason (F5.3).
  `simulation/tinker_sim_isaac/qualification_visual_capture.py` expires a
  partially captured sequence that can no longer satisfy the bounded latency
  contract on a restarted consumer: one deduplicated terminal error, a durable
  terminal-sequences marker, preserved camera-1 evidence, no camera-2
  fabrication, no retry/error growth across polls/restarts, and each PNG is
  atomically and durably persisted (temp + file fsync + atomic replace +
  parent-directory fsync) before its keyframe journal row (F5.4).  This is
  offline production-suite closure only; no live Isaac/camera/rosbag/GPU/OMPL/
  cuMotion claim; Task 10 still owns the Gate-F wiring and a load-bearing live
  rosbag; `_image_stats` still requires live RTX calibration.

- 2026-08-04 (integrated qualification Task 9, fix round 4 — "align evidence with
  real capture artifacts"): Closed the source-fixable residuals so Gate F accepts
  genuine live producer output.  `validation/integrated_evidence_index.py` now
  validates the real capture-latency arithmetic — `requested_physics_frame_index`
  must equal the producer's exact rounded-frame calculation from the keyframe's
  requested time and physics dt, `capture_latency_frames` must be an integer in
  `[0, MAX_CAPTURE_LATENCY_FRAMES]` equal to `raw_frame_index -
  requested_physics_frame_index`, and the raw/evaluator primary key and raw
  timestamp tolerance are retained at the captured frame (F4.1).  Rosbag
  `offered_qos_profiles` now parses the real Humble nine-field
  `rmw_qos_profile_t` (history/depth/reliability/durability plus deadline,
  lifespan, liveliness, liveliness_lease_duration,
  avoid_ros_namespace_conventions) and matches the recorder override as a subset
  on the required fields (F4.2).  `validation/integrated_contact_sheets.py` now
  orders the production CLI's bound captures by the canonical required suite
  event sequence (positive -> cancel -> safety), rejecting unknown/duplicate
  event identities instead of silently placing them (F4.3).  Both
  `overlay-contract.json` and the real `ompl-overlay-contract.json` (any
  legitimate `*-overlay-contract.json`) are categorized as overlay-contract with
  exactly one authoritative identity set (contradictory duplicates fail), and
  `source_locks.simulator_lock_path` resolves a verbatim root-relative
  `integration/source-locks.json` against the evidence suite without ever reading
  outside it or silently accepting an absent lock (F4.4).  Visual completeness
  keys by exact `(scenario_id, attempt_id)`; a nonempty valid GPU inventory is
  required when baseline/final report `available=true`; and
  `qualification_visual_capture.py` is restart-safe across a partial two-camera
  capture, seeding durable completion per `(request_sequence, camera)` so a
  crash after camera-1 re-captures only the missing camera on restart (F4.5).
  All offline production-shaped latency/QoS/CLI/overlay/restart tests pass;
  `_image_stats` thresholds still require live RTX calibration, Task 10 still
  owns the Gate-F wiring and a load-bearing live rosbag, and no live
  Isaac/RTX/camera/rosbag/GPU run occurred in this repair.  The future
  qualification tooling lock remains absent until after review-clean Task 10.

- 2026-08-04 (integrated qualification Task 9, fix round 3 — "make integrated
  evidence production-real"): Closed the two real-schema blockers so Gate F can
  accept genuine live artifacts and added the direct env/producer/consumer/
  end-to-end producer tests.  `validation/integrated_evidence_index.py` now
  parses the real nonempty RTX GPU inventory in `resource-cleanup.json`
  (baseline/final `gpus` are the physical inventory, not survivor lists) and
  recomputes cleanup `clean` from baseline/final availability, owned live PIDs,
  owned GPU survivors, unexplained memory, and termination state, with GPU
  topology invariance on the producer's stable `uuid`/`index` identity keys
  (F3.1); it accepts the real nested `repositories.production`/.simulator
  mapping plus scalar `repositories.path_scope` and the real source-lock status
  while still requiring lowercase 40-hex identities (F3.2); it computes required
  visual events per exact attempt/scenario so no cancel/safety sibling satisfies
  another and events split across siblings fail, with contact-sheet embedded
  events equal to the complete required suite sequence (F3.3); it cross-checks
  keyframe `requested_simulated_timestamp`/`requested_physics_frame_index`/
  `execution_event_sequence` against the canonical request within the strict
  numerical tolerance and keeps `capture_latency_frames` in
  `[0, MAX_CAPTURE_LATENCY_FRAMES]` (F3.5); it binds manifest/config/model/
  source/attempt/verdict identities and fails on a missing enclosing manifest
  (F3.6); `validation/integrated_contact_sheets.py` rejects output equal to any
  indexed evidence artifact before any overwrite (F3.7); rosbag QoS is parsed
  from the real YAML profiles (reliability/depth, recorder override) and every
  metadata-listed storage file must exist and be nonempty with no extra
  conflicting storage (F3.8).  `validation/integrated_gate_executor.py` makes
  every `_append_visual_event` producer failure fail-dominant: duplicate,
  no-join-key, invalid-timestamp, and rejected append all route the D/E attempt
  to `evidence-invalid` with the exact rejected event (F3.9), and
  `manipulation_qualification.py::_env` rejects a present-but-noncanonical
  integrated scenario id against `QUALIFICATION_SCENARIO_NAMES` before launch
  (F3.10).  `simulation/tinker_sim_isaac/qualification_visual_capture.py` seeds
  its handled-sequence set from the durable `visual-keyframes.jsonl` so a
  restarted consumer never re-captures (F3.4).  New direct tests drive the real
  runner env, real executor producer, real capture consumer, and one true
  integrated producer path (executor → consumer → index/sheets/summary → Gate F
  `verified-pass`) plus a diagnostic-only journal fail-closed test.  `_image_stats`
  thresholds still require live RTX calibration; Task 10 must wire Gate F and
  launch/finalize a load-bearing integrated rosbag; no live Isaac/camera/rosbag/
  GPU run occurred in this repair; the future qualification tooling lock remains
  absent until after review-clean Task 10.

- 2026-08-04 (integrated qualification Task 9, fix round 2 — "produce integrated
  visual evidence"): Produced the canonical visual-capture evidence end-to-end
  and closed the validator's semantic gaps.  The integrated executor
  (`validation/integrated_gate_executor.py`) now emits canonical sequence-shaped
  EventJournal capture requests (`{schema_version, sequence, gate, event,
  simulated_timestamp, source_execution_event_sequence}`) at every required
  checkpoint via `_append_visual_event`, strictly after the durable journal
  checkpoint it binds, with a per-attempt sequence reset and duplicate-event
  rejection; `manipulation_qualification.py::_env` enables the capture producer
  for integrated Isaac children by passing the exact canonical scenario id as
  `TINKER_SIM_QUALIFICATION_GATE` plus `TINKER_SIM_VISUAL_EVIDENCE=1`;
  `qualification_visual_capture.py` co-tenants executor diagnostic records
  safely (skipped silently, never capture-driving, never error-spam).
  `validation/integrated_evidence_index.py` binds captures only through the
  canonical request↔keyframe transaction (`request_sequence` + exact gate/event/
  capture path), keyframe truth by `(scenario_id, raw_frame_index)` primary key
  with a bounded `max(1e-6, 0.5*physics_dt)` timestamp window and exactly one raw
  and one evaluator row per key, and fails Gate F closed on any index diagnostic
  (duplicate request sequence / keyframe identity, orphan keyframe,
  request-without-image, malformed shape, duplicate physics/evaluator keys,
  frame/timestamp mismatch).  F2.5 closes source/provenance semantics
  (lowercase 40-hex commits, 64-hex status/diff/untracked digests,
  per-repository source-lock identities, nested overlay-contract
  `repositories`/`source_locks`); F2.6 closes scenario/evidence semantics
  (verdict↔manifest attempt cross-match, MoveIt/controller status domain
  `{diagnostic-pass, diagnostic-fail, evidence-invalid, blocked-by-gate-b,
  verified-pass}`, per-attempt PlanningScene journal+final, duplicate
  raw/evaluator rows); F2.7 makes the rosbag contract exact (all 11 approved
  topics with exact message types, per-topic nonzero counts, QoS, sqlite3
  storage); F2.8 recomputes cleanup `clean` against
  baseline/final/owned-pids/GPU-survivors/unexplained memory; F2.9 makes
  `build_contact_sheet` reject output equal to the sibling sheet or an indexed
  capture before any byte is written.  New tests: 16 evidence-index mutations +
  3 contact-sheet mutations (54 + 23 total); affected ROS-free regressions pass;
  Humble-sourced `test_integrated_gate_executor_ros.py` passes.  No build, no
  live Isaac/ROS/GPU/cuMotion, and no rosbag/capture production ran; the future
  qualification source lock remains absent.

- 2026-08-04 (integrated qualification Task 9, fix round 1 — "bind evidence index
  to live artifacts"): The initial 44 tests were synthetic and did not exercise
  real Task 2-8 producer schemas, so this repair re-pinned every semantic parser
  to the actual producer contracts.  `validation/integrated_evidence_index.py`
  now consumes the real executor `visual-capture-requests.jsonl` +
  capture-process `visual-keyframes.jsonl` two-journal transaction, joins each
  keyframe to exactly one request via `request_sequence`/phase/event identity,
  resolves `visual/source/*.png` under the attempt root, and cross-binds
  `(frame_index,timestamp)` to exact `physics_truth.jsonl`/`evaluator.jsonl`
  frames.  Scenario kinds come only from the canonical
  `integrated.acceptance.polarity`/`expected_negative` contract (never
  `integrated.kind`); the three canonical cancel ids and two safety ids require
  their event groups.  `validate_gate_f` is now semantic: source-lock/static-
  contract/model-fingerprint identities, per-scenario `gate-verdict.json`
  verified-pass, nonempty raw/evaluator/drain exactness, finalized MoveIt/
  controller/planning-scene journals, rosbag2 metadata + storage counts,
  cleanup/process/GPU leak checks, and contact-sheet PNG/metadata/parity.  F1.5
  index integrity recomputes the canonical checksum, re-hashes current bytes,
  and compares the preserved-file set; the summary binds a pre-summary projection
  checksum (never a cryptographic cycle).  F1.6 embeds deterministic PNG text
  chunk metadata (role/ordered events/source capture records/reviewed state).
  New tests: `tests/test_integrated_evidence_index.py` (38) +
  `tests/test_integrated_contact_sheets.py` (20) = 58 passed in the simulator
  venv, mutation-driven against production-shaped artifact bytes; affected
  ROS-free regressions pass (see the task-9-report).  Documentation/staging per
  `execution-corrections-2026-08-02.md` §6.  No build, no live Isaac/ROS/GPU/
  cuMotion, and no rosbag/capture production ran; immutables and the future
  qualification source lock remain untouched.

- 2026-08-04 (integrated qualification Task 9 — "close integrated evidence
  artifacts"): Added deterministic reproducibility indexing and integrated
  contact sheets.  `validation/integrated_evidence_index.py` (new) builds
  `evidence-index.json` from real preserved artifact bytes/metadata with
  canonical JSON (`sort_keys`, `("," ":" )`, `ensure_ascii=False`) and lowercase
  64-hex SHA-256 digests; the index excludes only itself and repeated builds over
  unchanged bytes are identical.  `validate_gate_f` fails closed on missing
  commit identity, rosbag metadata/QoS/counts, planning-scene journal, required
  artifacts, unbound captures, and absent contact sheets, and never fabricates a
  verdict.  `validation/integrated_contact_sheets.py` (new) renders deterministic
  `contact-sheet-integrated-agent.png` / `contact-sheet-integrated-user.png`
  authorized only by captures already carrying exact path+digest+event/frame
  metadata in the evidence index; blank/transparent/unindexed/stale/mismatched/
  unbound/out-of-suite/duplicate captures and output-as-input are rejected, and
  path traversal/symlink escape/files changing during hashing are rejected.  New
  tests: `tests/test_integrated_evidence_index.py` (28) +
  `tests/test_integrated_contact_sheets.py` (16) = 44 passed in the simulator
  venv; affected ROS-free regression 369 passed.  Task 10 wires Gate F into the
  orchestrator.  No build, no live Isaac/ROS/GPU/cuMotion; immutables and the
  future qualification source lock untouched.

- 2026-08-04 (integrated qualification Task 8, fix round 5 — "stabilize controller
  evidence lifecycle"): Repaired the four load-bearing defects found by fresh
  coordinator repetition of fix round 4 (the positive D trio passed only 11/17
  fresh-process runs, one real cancel transaction committed `evidence-invalid`,
  one process crashed at interpreter teardown, and five tests hit the 5 s
  harness readiness budget).  FJT terminal evidence is now a single immutable
  capture transaction: `_wait_for_fjt_status` captures one `dict(entry)` copy,
  the run methods bind the provider via `_bind_and_call_fjt_provider` and pass the
  exact captured entry into `_validate_fjt_evidence`, and a second post-capture
  status emission for the same controller UUID can no longer switch or race the
  transaction (provider UUID/status/sequence/timestamp must equal the captured
  entry exactly, source is the real
  `/xarm7_traj_controller/follow_joint_trajectory/_action/status` topic).  The
  production-real D wait defaults are now exactly `fjt_wait_timeout_s = 10.0` and
  `motion_trigger_timeout_s = 10.0` (`_threshold_timeout`); finite positive
  scenario overrides stay authoritative and malformed/non-finite/boolean/zero/
  negative overrides fail closed.  Presend ExecuteTrajectory goals now carry a
  preassigned action UUID (`send_goal_async(..., goal_uuid=...)`) retained before
  send; on acceptance-response timeout the driver enters a bounded cleanup phase
  (late-acceptance grace, then a typed exact-UUID `action_msgs/srv/CancelGoal` to
  `/execute_trajectory/_action/cancel_goal`, requiring cancellation/terminal
  evidence for that exact UUID) so no uncontrolled presend motion can survive
  driver exit, and it never cancels all/timestamp-wide/unrelated goals.  The
  controlled Humble harness now tracks every execute coroutine/goal/result/
  cancel future, hold/release event, thread, node, and context and drains them in
  explicit bounded order before shutdown (no daemon-thread reliance), and its
  readiness budget is exactly 30.0 s to match production `run_driver`.  A
  valid-but-non-advancing join key (two journal snapshots landing inside one
  truth frame observe the same frame_index/timestamp) now waits a bounded 0.1 s
  for the next advancing physics-truth frame instead of emitting a spurious
  `no-join-key`; malformed keys, a missing provider, or a genuinely stalled
  truth stream still fail closed (`JOIN_KEY_RETRY_S`).  An
  owner-QoS mutation creates an extra incompatible publisher/subscriber before
  the required endpoint and still selects `/move_group`,
  `/fixture_planning_scene`, and `/tinker_integrated_gate_executor`; missing or
  duplicate required owner endpoints return `{}` and fail closed.  Fresh
  verification: ROS-free executor + driver suites pass; sourced-Humble provider
  suite 38 passed with zero warnings across three consecutive fresh-process runs;
  each positive D execute/cancel/safety test passes 4 consecutive fresh-process
  runs alone; the three positive D tests together pass 30/30 consecutive
  fresh-process runs with fresh domain IDs and zero warnings/crashes/timeouts;
  the delayed-status (>1 s, <10 s), second-post-capture-status, delayed-
  acceptance exact-cancel (both late-handle and exact-UUID paths), cleanup
  rejection/unavailable fail-closed, and owner-QoS mutation tests pass fresh.
  No build, no live Isaac/GPU/cuMotion; executor/verifier/journal/orchestrator/
  scenarios/configs/locks/gateway/bridge/scripts and all production files
  untouched; the future qualification source lock remains absent.

- 2026-08-04 (integrated qualification Task 8, fix round 4 — "bind real
  controller transactions"): Bound every Stage-D controller transaction to the
  real FJT controller goal UUID, distinct from the MoveIt ExecuteTrajectory
  UUID.  `_normalize_goal_uuid` now accepts real Humble rclpy `UUID` messages
  (numpy `uint8[16]`) with a strict 16-byte rule; `_d_baseline()` tracks known
  controller FJT goal UUIDs and `_discover_new_fjt_goal()` discovers the unique
  new controller UUID after the ExecuteTrajectory result, failing closed on zero
  or multiple new UUIDs (pre-baseline replays and duplicate controller goals are
  rejected); `_validate_fjt_evidence()` gains `expected_fjt_goal_uuid` and joins
  the provider UUID/status/sequence/timestamp to the exact fresh FJT status-topic
  entry.  `run_execute_sequence`/`run_cancel_sequence`/`run_safety_sequence`
  never key FJT status on `execute_goal_id`: cancel/safety pre-send
  (`_presend_long_motion`) carries the real controller identity + pre-send
  baseline, `run_driver` teardown guarantees presend cleanup with an operator
  clear, and `_observe_journal_graph` selects owner-specific graph QoS rather
  than `infos[0]`.  `run_sim.build_occupancy_from_planning_scene` rasterizes the
  oriented (yaw-only) box footprint of all 17 canonical scenarios.  Offline
  regressions: `test_integrated_gate_executor.py` 127 passed,
  `test_integrated_gate_executor_driver.py` 37 passed,
  `test_integrated_gate_executor_ros.py` 165 passed; sourced-Humble
  `tests/ros_humble/test_integrated_gate_executor_driver_providers.py` 30 passed
  with zero warnings (positive C/D paths commit `diagnostic-pass`, controller
  UUID distinct from ExecuteTrajectory UUID, cancel/safety reach the real
  presend-provider sequence, and the transaction-real negatives fail closed).
  No build, no live Isaac/GPU/cuMotion; executor/verifier/journal/orchestrator/
  scenarios/configs/locks/gateway/bridge/scripts and all production files
  untouched; the future qualification source lock remains absent.

- 2026-08-04 (integrated qualification Task 8, fix round 3 — "observe live
  integrated providers"): Replaced every fabricated/placeholder provider with a
  driver-owned live observer and a real graph/service surface, and adopted
  Option A+ for the qualification development LiDAR.  The driver now constructs
  a private-context `_LiveProviderObserver` node added to the executor's spinner
  (multi-node private-context rclpy requires driving the shared spinner, so all
  controller/graph service calls go through `_call_service_with_spinner` polling
  an async future instead of the blocking `client.call()` that hangs when the
  spinner owns the response); readiness, TF TCP pose, PointCloud2 environment
  cloud, native gripper goal count, FJT transaction digest, and live graph
  introspection all come from observed subscriptions/transforms, never executor
  internals (`_observed_graph`/`_tf_lookup`/`_latest_environment_cloud`/
  `_native_gripper_goal_count`/`ParameterClient`/`server_is_available` are all
  unreferenced).  Long-motion goal UUIDs are normalized driver-side
  (`_goal_id_hex`) because `_normalize_goal_uuid` rejects real rclpy UUID
  messages.  For E transport scenarios the driver re-publishes the operator
  baseline and refreshes its age inside the readiness snapshot so a live fresh
  operator sample is actually observed.  The `/pick_and_place.post_grasp_lift_m`
  service uses `declare_parameter` + `add_on_set_parameters_callback` (rclpy
  auto-creates `/get_parameters`/`set_parameters` on every node; a manual
  service conflicts).  Option A+: `run_sim.build_occupancy_from_planning_scene`
  builds a pure deterministic 2-D occupancy map (0.05 m, 60 m half-extent) from
  committed scenario PlanningScene box footprints; `qualification_occupancy`
  returns it only for scenarios with box fixtures (None for empty/free-space);
  `gateway_lidar_enabled` enables the development lidar only for
  `navigation-parity` or `manipulation-core --qualification`; and the integrated
  launch owns the exact `base_link -> livox360` static transform
  (tf2_ros/static_transform_publisher named `livox360_static_tf`,
  xyz 0.12/0.0/0.25, quaternion identity), spelled qualified
  (`launch_ros.actions.Node`) so the immutable Task-2 launch-graph allow-list
  still accepts the overlay.  Raw-verifier authority is unchanged.  Offline
  regressions: `tests/test_integrated_gate_executor_driver.py` grows to 36
  ROS-free tests (hermetic double-parameter pure layer, Option A+ occupancy and
  gateway-profile resolution, bundle committed-identity fail-closed); broad
  ROS-free qualification batch 1220 passed + 2 skipped + 9 subtests with the
  sole pre-existing `uv` executable-hash provenance failure unchanged; launch
  contract 7 passed; sourced-Humble `tests/ros_humble/` 54 passed + 1 skipped
  (22 new provider tests incl. live readiness/controllers/FJT/native-gripper/
  parameter/negative-mutation/cancel-provider paths).  No build, no live
  Isaac/ROS, no GPU-process change, no cuMotion; executor/verifier/journal/
  scenarios/configs/locks/`ros_gateway.py` and all production files untouched.

- 2026-08-04 (integrated qualification Task 8, fix round 2 — "wire executor
  evidence producer"): Wired the live producer of the executor evidence and the
  per-scenario terminal marker that round 1 lacked, and sealed finalization so
  an exception can never escape the per-scenario boundary.  New source-run
  Humble driver `validation/integrated_gate_executor_driver.py` (ROS-lazy at
  import, runs under the existing `ros-tooling` environment as
  `/usr/bin/python3`) loads the orchestrator's atomically written
  `scenario-bundle.json`, derives dispatch for exactly the 17 canonical
  scenario ids from the committed executor constants, constructs the real
  `IntegratedGateExecutor` for the current immutable attempt with live
  readiness/join-key/graph providers, dispatches exactly one run method, and
  writes `execution-terminal.json` (scenario id + attempt id + resolved attempt
  path cross-bound, `marker: executor-driver`) only after the executor's own
  artifact finalization (`integrated-execution.json` must exist).  A driver
  setup/dispatch exception writes a durable fail-closed terminal
  (`evidence-invalid`) and exits nonzero; unknown scenario ids and stale or
  wrong-identity terminal markers fail closed before any ROS traffic.  The
  orchestrator now launches the driver as a third owned child
  (`qualification_start_process(runner, "executor", ...)`) only after canonical
  PHYSICS_READY (with the bundle atomically written at
  `<attempt_dir>/scenario-bundle.json`) and waits for a current-attempt
  cross-bound terminal within a config-derived budget —
  `plan + 2*execute + cancel + scene + max(cancel,30)` = exactly 305.0 s for
  the committed thresholds, separate from the 30 s readiness budget — failing
  immediately if the executor process exits without a marker.  For every E
  transport scenario the driver sets and reads back
  `/pick_and_place.post_grasp_lift_m = 0.10` on the live task server via an
  rclpy parameter client (rejected/unavailable/malformed/low read-back fails
  closed before Pick traffic) and supplies the observed read-back as the typed
  provider, never a default.  The additive `"executor": "ros-tooling"` role in
  `QualificationRunner._start` shares the Humble overlay's DDS/domain/attempt/
  PYTHONPATH/AMENT_PREFIX_PATH environment and never alters six-gate behavior.
  `_finalize_attempt` now isolates every cleanup phase (executor stop, Isaac
  stop, exact raw/evaluator drain while Humble is alive, Humble stop, rosbag
  handling, orphan termination, resource evidence, settle) with per-phase
  try/except so a single failing phase records a distinct failure reason while
  all later cleanup still runs; both `_execute_scenario` and `_teardown_scenario`
  guard a whole-helper escape and convert it to durable per-scenario
  `evidence-invalid`.  Integrated C-E rosbag remains a truthful non-load-bearing
  diagnostic (`not-recorded`; a present valid bag is validated, a corrupt one
  fails closed); Task 9/Gate F must add or index the intended recorder.  Live
  provider obligations (readiness snapshot liveness, TF TCP pose, environment
  cloud, native gripper goal count, long-motion UUIDs) are carried as live
  obligations and are never claimed by offline doubles.  Offline regressions:
  new `tests/test_integrated_gate_executor_driver.py` (25 ROS-free tests),
  55 integrated-orchestration tests (incl. third-child launch ordering, terminal
  budget/cross-binds, and per-phase finalize isolation with stage continuation),
  511 passed in the broader qualification batch, and the sourced-Humble real
  executor/journal surface (164 + 184 passed) proving the executor finalizes
  `integrated-execution.json`, `moveit-plans.jsonl`, `controller-results.jsonl`,
  `planning-scene.jsonl`, and `planning-scene.json`.  No build, no live
  Isaac/ROS, no GPU-process change, no cuMotion; executor/verifier/journal/
  overlay-launch/scenarios/configs/locks and all production files are untouched.

- 2026-08-04 (integrated qualification Task 8, fix round 1 — "execute
  integrated scenario lifecycle"): Made the integrated C-E lifecycle executable
  and fail-dominant on the real manifest/launch path.  Integrated scenario ids
  are never passed to the core six-gate `_selected_gates`; the orchestrator now
  builds the manifest at the externally allocated attempt directory via the new
  additive `QualificationRunner.prepare_manifest_at(... gate="integrated")` and
  starts the configured Isaac and Humble child wrappers through the real
  `qualification_start_process` lifecycle with the exact scenario id, seed,
  private domain, and `TINKER_SIM_ATTEMPT_DIR`/RMW/DDS environment applied to
  the subprocesses (previously the environment dict was computed and never
  applied, and no child was started).  Every scenario invocation gets a newly
  created immutable attempt directory (invocation id + monotonic counter,
  `mkdir(exist_ok=False)`), so repeated allocation yields distinct preserved
  paths and stale pre-existing evidence cannot satisfy readiness; readiness now
  additionally requires current-attempt manifest provenance, and a nonempty
  attempt directory is rejected before launch.  Per scenario the producers are
  stopped, the evaluator/raw drain is required to correlate exactly, rosbag
  evidence is finalized, and orphan/resource cleanup runs before the independent
  `verify_integrated_attempt` (single fail-dominant `try/finally` lifecycle,
  so every post-launch exception still runs bounded cleanup).  C/D/E stages now
  carry a top-level fail-dominant status (`evidence-invalid` > `verified-fail` >
  `verified-pass`), standalone pass exits 0, `--stage all` always retains Stage
  A and never exits 0 on Gate-B failure, and a not-implemented Stage F reports a
  non-success overall status.  Gate B evidence is per-invocation (fresh
  attempt-bound directory), source-lock `fail`/missing/stale/self-generated
  artifacts are `evidence-invalid` (never `verified-fail`), producer exit 0
  without newly written output is rejected, and Gate B's `model-fingerprint.json`
  is cross-bound to the runtime model bundle consumed by C-E.  Stage A requires
  the integrated config's `required_core_gates` to equal the core config gate
  list exactly.  A malformed scenario fails closed without skipping later
  controls.  Offline regressions: 35 integrated-orchestration tests (15 prior +
  20 new lifecycle/status/stale-evidence tests using process doubles) and the
  329-test broader qualification batch all pass.  No build, no live Isaac/ROS,
  no cuMotion; production modules/scenarios/policies/executor/journal/config and
  the two source-lock policy files are untouched.

- 2026-08-04 (integrated qualification Task 8 — "orchestrate Gates A-F"):
  Added the offline orchestration/lifecycle layer over the review-clean
  six-gate core suite, the Task 6 physics-ready gate, and the Task 7
  independent integrated verifier.  `validation/integrated_qualification.py`
  exposes `IntegratedRunner` with CLI stages `A`/`B`/`C`/`D`/`E`/`F`/`all`.
  Stage A calls the existing core suite through the unchanged
  `manipulation_qualification.py --gate GATE_NAME` semantics (six required
  gates, exact raw/evaluator drains, valid rosbags, clean teardown, existing
  contact sheets).  Before Gate B the runner atomically writes
  `outputs/integrated/attempt-start.json` with UTC/monotonic start identities,
  then invokes the committed `source_lock_manifest.py` producer with the
  config-resolved authorization policy and validates the exit code and output
  schema before invoking the offline static closure; Gate B is fail-closed and
  never captures/trusts current state, and blocks C-F on any non-pass.  Stages
  C-E run every listed scenario in a unique child ROS domain in `[0,232]` with
  a unique immutable attempt directory; readiness requires the overlay's
  atomically written `physics-ready.json` to bind its `scenario_report_sha256`
  to the exact bytes of the atomically written `scenario-runner.json` and to
  carry the full committed identity (scenario id/seed,
  scenario_declaration_sha256, planning_scene_sha256, integrated_sha256, model
  fingerprint, provider-manifest digest, final `STATE_PLAYING`, and a final
  `state=1`/`boundary=PHYSICS_READY` operation), so a transient
  `state=PHYSICS_READY` alone is insufficient.  Execution return codes never
  override the independent verifier verdict; teardown failures downgrade a
  scenario to `evidence-invalid` and every attempt is preserved.  Stage F is
  the explicit Tasks 9-10 extension point (`not-implemented`).  Reusable core
  helpers (source identity, record topics/QoS, process launch/readiness,
  truth/evaluator drain, rosbag finalization, termination/resource cleanup)
  were extracted as additive thin delegations in
  `validation/manipulation_qualification.py` with no behavior change to the
  six-gate runner.  `tests/test_integrated_qualification.py` (15 tests: 8
  deterministic orchestration-contract tests plus 7 real-runner offline
  contract tests) passes; focused suite is 80 passed + 2 subtests, and the
  broader qualification regression batch is 264 passed.  No build, no live
  Isaac/ROS, no cuMotion; production modules/scenarios/policies/executor/
  journal/config and the two source-lock policy files are untouched.

- 2026-08-04 (integrated qualification Task 7, fix round 2 — "verify terminal
  quiescence"): Made the integrated verifier production-safe for cancel and
  clear deceleration.  F2.1 — terminal quiescence is now proven from a bounded
  tail ending at the `quiescent` join key (at least two consecutive settled raw
  frames with max absolute arm velocity <= `safety_stop_velocity_rad_s` and a
  stable command target), never from max-speed over the whole
  `[cancel-requested, quiescent]` braking window; D-cancel and E
  cancel-transport pass fixtures now carry a production-real deceleration ramp
  that settles exactly at quiescent, a ramp that does not settle fails, and a
  new command target between cancel/clear and quiescent fails even if the
  velocity later settles.  `no_post_clear_resume`/forbidden `post_clear_resume`
  treat motion after clear as a resume only when it is a new target/goal, not
  the pre-existing command's deceleration.  Safety-specific
  `safety_stop_frames`/`safety_position_creep_rad` checks are preserved; a
  realistic D-safety braking-ramp test determines the contract truthfully
  (small ramp inside the creep bound passes, 0.012 rad drift fails
  `target_frozen`) and the E safety-transport ramp truthfully fails
  `velocity_below_stop_limit` while `no_post_clear_resume` stays green.  F2.2 —
  forbidden-token scanning is restored over unpaired source/provider strings
  while the committed semantic provenance
  `env_cloud_evidence.source="observed-environment-cloud"` stays accepted;
  `goal_kind` joins the provider/goal-field scan; `pipeline_id` remains exact
  lowercase `"ompl"` (case variants are evidence-invalid by intentional identity
  strictness).  F2.3 — the D-safety attempt now requires a consistent non-success
  terminal across every present status domain (`safety_terminal_non_success`);
  a terminal claiming success fails and a contradictory pair is evidence-invalid.
  F2.4 — the CLI identity/config-mismatch path atomically writes
  `gate-verdict.json` before exit 2, and `verify_integrated_attempt` resolves a
  stable fallback identity so malformed/missing bundle structures,
  bool/string/list/null seed, missing integrated mapping, and malformed report
  identities return durable `evidence-invalid` (never a traceback).  F2.5 — raw
  measured target identity is restricted to the backend-emitted bare
  `qualification_cube`; `sim_fixture/...` stays in the planning-scene diagnostic
  domain and the dead `_target_in_object_ids` helper is removed.  Test suite
  grows to 52 tests; affected regression + qualification batch passes 637 + 2
  subtests.  No build, no live Isaac/ROS, no cuMotion; production
  modules/scenarios/policies/executor/journal/config are untouched.

- 2026-08-04 (integrated qualification Task 7, fix round 1 — "align verifier
  with production evidence"): Aligned the independent integrated verifier with
  production raw-truth/executor/journal shapes and closed three blocker and
  three major defects found by specification and adversarial review.  F1.1 —
  the pre-start `qualification_cube` requirement is now Stage-E-only: C/D
  scenarios declare `objects: []` and real backend truth carries `objects:
  []`/`object: None`/`expected_objects: {}`, so C/D verify with production-real
  empty object sets (the fixture's unconditional cube injection is removed).
  F1.2 — the `scene-detach` journal record now uses the committed after-state
  (target detached), matching the executor's post-detachment snapshot and the
  journal transition rule; the fixture `detach_pending` exception is removed and
  attached-at-detach fails closed.  F1.3 — the endpoint/provider validator is
  scoped to endpoint evidence: paired `_REQUIRED_ENDPOINT_SOURCES` ownership is
  checked only when an endpoint and its provider coexist in the same mapping, so
  the real D cartesian-retreat `env_cloud_evidence.source ==
  "observed-environment-cloud"` (cloud provenance, not an endpoint provider) is
  accepted; wrong paired provider metadata still fails.  F1.4 — contradictory
  terminal domains (success string vs ABORTED numeric, or the reverse) and
  missing terminal evidence now fail closed as `evidence-invalid`, never a
  permissive selection; `diagnostic-pass` is no longer terminal proof; the same
  consistency rule applies to `_terminal_non_success`, D cancel, and E negative
  controller/place terminal checks.  F1.5 — all evidence-owned scalar/indices
  (`seed`, `raw_start_index`, `evaluator_start_index`, journal keys, result
  codes) are validated as non-boolean integers/indices; malformed values produce
  `evidence-invalid` and the public boundary/CLI always atomically writes
  `gate-verdict.json` (no traceback).  F1.6 — forbidden execution-provider taint
  now scans narrow provider/goal fields (`pipeline_id`, `provider`,
  `execution_profile`, planner fields) beyond endpoint/source values; a
  persisted `pipeline_id` must be `"ompl"`; semantic free-text fields stay out
  of the scan.  F1.7 — every scenario-owned temporal predicate reads only its
  exact Table-2 observation subwindow ending at `quiescent`/`released-settled`;
  post-terminal drain motion after `quiescent` is ignored, while the same motion
  between cancel/clear and `quiescent` fails.  F1.8 — `scene_attached_after_place_failure`
  now proves the target is attached in the journal records after the place
  failure through `quiescent`; fixture `goals_sent` is a production-shaped count
  (never a character list); the D retreat fixture carries the real
  `env_cloud_evidence` shape; verified-fail coverage added for
  `qualification-pick-place-cancel-approach` and `qualification-moveit-plan-pose`
  (all 17 scenarios now have a pass and a direct failing mutation).  F1.9 — the
  CLI bundle normalization compares a declaration id to the actual scenario
  path/bare-id and fails closed (exit 2) on a mismatched filename/id; the
  self-comparison dead check is removed.  Test suite grows to 36 tests (all
  prior 23 plus 13 new producer-shape/malformed/terminal/temporal/identity
  tests); affected regression + qualification batch passes 622 + 2 subtests.
  No build, no live Isaac/ROS, no cuMotion; production
  modules/scenarios/policies are untouched.

- 2026-08-04 (integrated qualification Task 7 — "independent integrated
  raw-physics verifier"): Added the independent, integrated raw-physics
  verifier for the 17-scenario manipulation-qualification matrix.
  `validation/integrated_gate_verifier.py` is a ROS-free Python 3.12 module
  whose physical verdicts derive only from `physics_truth.jsonl` raw truth and
  the PlanningScene journal; executor/action/controller/PlanningScene results
  are diagnostic-only.  It implements Table 1 per-scenario terminal anchors
  (gate_start = fixture-ready timestamp; gate_end = teardown for plan-only and
  malformed-back, execution/retreat/gripper-terminal, quiescent,
  released-settled, or pick-terminal otherwise), Table 2 observation subwindows
  that end at quiescent/released-settled and never at teardown, the
  `select_integrated_gate_window` wrapper over the verifier's nearest-pre-start
  + in-gate selection (no post-terminal drain, no duplicates), physics.hz
  resolution through `core_config` -> manipulation-core.json (120.0), the
  endpoint allowlist = REQUIRED_ACTIONS ∪ REQUIRED_SERVICES (forbidden
  direct-Isaac endpoints fail closed), phase-aware attachment validation,
  exact raw/evaluator canonical equality with a distinct
  "raw/evaluator drain mismatch" reason code, the `gate-b-status.json`
  `{"schema_version":1,"status":"blocked"}` blocked-by-gate-b marker
  (fail-closed), and verdict gate = scenario id with stage/polarity separated.
  Tests: `tests/integrated_verifier_fixtures.py` (deterministic 17-scenario
  attempt builder with fault injection) and `tests/test_integrated_gate_verifier.py`
  (the brief's eight acceptance tests verbatim plus adversarial coverage: full
  17-scenario pass and per-class verified-fail matrix, terminal-anchor
  exclusion of post-terminal drain, subwindows ending at quiescent,
  blocked-by-gate-b marker, physics-hz core_config resolution and rejection,
  endpoint allowlist, verdict-gate identity, drain-mismatch reason code, strict
  >1.0 N contact threshold, transport direction guard, lift baseline pinned to
  the pre-start frame, phase-aware attachment, obstacle-contact exclusion of
  the grasped target, and negative forbidden-after-terminal).  23 new tests
  pass; affected regression suites (Gate D/E, journal, static-contract, config,
  qualification fixtures) pass 480.  No build, no live Isaac/ROS, no cuMotion;
  production modules/scenarios/policies are untouched.

- 2026-08-04 (integrated qualification Task 6, formal-review fix round 5 —
  "harden final trajectory digest test"): Hardened the last remaining
  `run_execute_sequence` caller in the sourced-Humble D acceptance suite that
  reached the executor's own planned/executed CDR-digest comparison without the
  deterministic serializer seam.  F5.1 —
  `test_executor_execute_uuid_mismatch_cleans_up_accepted_handle` now installs
  the same test-local `_install_deterministic_serialize` seam as its seven F4.2
  siblings (the canonical planned trajectory is serialized EXACTLY ONCE at
  setup; that byte/digest snapshot is authoritative for the whole run window),
  so the rejection is deterministically the plan/execute UUID-identity failure
  and never a load-sensitive rclpy CDR-padding digest mismatch.  The test's real
  purpose is preserved: the accepted `ExecuteTrajectory` handle has a UUID
  identity mismatch, exactly one bounded cleanup attempt is made, and the final
  `execute_error` is the UUID reason.  The sent `ExecuteTrajectory.Goal`
  trajectory identity is now asserted semantically field-by-field
  (`_robot_trajectories_equivalent`) against the setup snapshot, and the
  provider's FJT evidence reuses the single canonical digest snapshot.  A
  complete caller audit confirms every `run_execute_sequence` call in
  `tests/test_integrated_gate_executor_ros.py` either installs the seam or
  returns before the digest comparison by direct control flow (the two
  no-scene/no-provider negatives fail closed in `_acquire_scene`/the
  `fjt_transaction_provider is None` gate before any plan/execute goal, so they
  cannot reach `planned_digest_before`).  Production serializer/digest code is
  unchanged and the digest checks are not weakened; the raw serialized bytes are
  never altered.  Humble suite stays 164 tests (pure suite stays 126); the
  UUID-mismatch test passes 100 consecutive fresh iterations, the affected
  D-digest batch passes 50 consecutive iterations, the concurrent stress passes
  all processes under load, the full sourced-Humble suite passes 10 consecutive
  fresh clean processes, and the flagship Gate-E temporal subset passes 20
  consecutive clean iterations.  No build or live run is required.

- 2026-08-04 (integrated qualification Task 6, formal-review fix round 4 —
  "preserve Gate E downgrade truth"): Preserved controller/action/task truth
  through the E fail-dominant downgrade path and removed the load-sensitive
  rclpy CDR-padding digest flake.  F4.1 — the E fail-dominant downgrade writers
  (`_write_e_fail_dominant_execution_json` and the final
  `integrated-execution.jsonl`/`controller-results.jsonl` downgrade rows) now
  preserve `controller_goal_sent`/`controller_goal_uuid`/`controller_endpoint`
  from the pre-downgrade truthful record, so a late required-artifact write
  failure never erases or fabricates the controller identity the attempt
  actually observed; two sourced-Humble downgrade tests force a late goal-
  artifact write failure after an observed approach FJT (identical controller
  truth across every authoritative/final downgrade artifact, no final row
  claims pass) and on the no-controller path (all three fields stay
  `False`/`None`, accepted-goal cleanup retained).  F4.2 — the D-side digest
  nondeterminism is eliminated by serializing the canonical planned trajectory
  EXACTLY ONCE per test setup (`_install_deterministic_serialize` caches the
  exact rclpy `serialize_message` bytes by object identity per executor,
  so the plan/executed/FJT-join digests are byte-identical inside the run
  window) while separately asserting full semantic trajectory identity
  (`_robot_trajectories_equivalent`) field-by-field on the actual
  `ExecuteTrajectory` goal, with a mutation-negative
  `test_semantic_trajectory_identity_detects_mutation` proving a mutated
  executed trajectory is caught even when the provider digest is reused.  The
  raw serialized bytes are never altered, so production digest semantics and
  Gate D/E runtime behavior are unchanged; every affected sibling D test now
  serializes once at setup and reuses the snapshot digest.  Humble suite grows
  to 164 tests (pure suite stays 126); the full sourced-Humble suite passes 10
  consecutive fresh clean processes, the affected D-digest tests pass 50
  consecutive iterations and 8 concurrent processes, and the flagship Gate-E
  temporal subset passes 20 consecutive clean iterations.  No build or live
  run is required.

- 2026-08-04 (integrated qualification Task 6, formal-review fix round 3 —
  "make Gate E temporal evidence deterministic"): Resolved the F3.1-F3.5
  consolidated findings in the Stage-E path, preserving every review-clean
  F1/F2 behavior.  F3.1 — every flagship E ordering/race/negative proof is now
  event-driven: `threading.Event` barriers gate the approach/transport/Place
  evidence injections strictly after the executor's corresponding observable
  state (goal acceptance + baseline capture, lift latch, Place baseline, exact
  Place cancel terminal), and the delayed Pick-result future is Event-released
  only after the transport latch.  Fixed `threading.Timer` offsets and
  `time.sleep` event-order margins are gone; the runner-level receipt-window
  "late" negatives pin the FJT `received_mono` strictly beyond the 2.0 s
  `E_FJT_CORRELATION_TIMEOUT_S` boundary and the settled-post-result negative
  seeds the transport FJT only after the bounded wait returns.  F3.2 —
  occupied-place now requires a STRICTLY fresh post-cancel PlanningScene
  observation: the runner records the pre-cancel `scene_sequence` + receipt,
  boundedly waits for a valid scene with `scene_sequence` strictly greater than
  the baseline AND a receipt time after the exact cancel terminal, and only that
  fresh scene may establish `post_cancel_target_attached=true`.  Timeout,
  unchanged sequence, malformed/provider-error newer scene, or detached target
  is `evidence-invalid`; baseline/post-cancel sequence, receipt delta,
  attachment state, and reason are recorded in the trigger and durable
  artifacts.  F3.3 — controller traffic is derived ONLY from observed FJT
  evidence: `controller_goal_sent` is true only when an actual FJT
  transaction/status/UUID was observed, `controller_endpoint` is None when no
  FJT was observed, no-goal cleanup is `None` (never `{}`), and accepting/
  canceling a task goal or attempting task-goal cleanup never implies a
  controller goal.  The truth is preserved consistently in returned records,
  `integrated-execution.jsonl/.json`, `controller-results.jsonl`,
  `moveit-plans.jsonl`, and goal artifacts.  F3.4 — the dead reason-collapsing
  `_e_post_grasp_lift_m()` helper is deleted (the detailed provider reason map
  stays in `_e_prepare`), and `integration/MANIPULATION.md` now carries the
  live-orchestrator latency obligation for `E_FJT_CORRELATION_TIMEOUT_S=2.0`,
  the safety-stop observation window, and fresh post-cancel PlanningScene
  publication/service latency.  Humble suite grows to 160 tests (pure suite
  stays 126); every ordering/race negative is barrier-bounded, and the full
  sourced-Humble suite passes 10 consecutive fresh clean processes with the
  flagship temporal subset passing 20 consecutive clean iterations.  No build
  or live run is required.

- 2026-08-04 (integrated qualification Task 6, formal-review fix round 2 —
  "seal Gate E runtime contracts"): Resolved the full F2.1-F2.8 consolidated
  SPEC/QUALITY findings in the Stage-E path.  F2.1 — the 10 cm physical
  threshold is preserved (Gate E never weakens `object_lift_m` to the 0.08 m
  production default); every E transport scenario (positive, occupied-place,
  cancel-transport, safety-transport) now requires an injected, fresh
  `post_grasp_lift_m_provider` runtime-parameter observation BEFORE any Pick
  traffic.  The observed production `pick_and_place` parameter must be finite
  and `>= object_lift_m` (0.10 m), with fresh identity/receipt metadata;
  missing/stale/provider-error/0.08 evidence fails immediately with a stable
  readiness reason (`no-post-grasp-lift-m-provider`,
  `post-grasp-lift-m-provider-{missing,stale,non-finite,unavailable}`,
  `post-grasp-lift-m-below-object-lift`) and zero action traffic — never a
  15 s transport timeout.  Accepted 0.10 keeps the lift latch
  `grasp_z + object_lift_m - tolerance` (0.81 m), physically reachable at the
  production TCP peak 0.82 m.  F2.2 — the positive/occupied-place/
  cancel-transport/safety-transport ordering tests now use controllably
  delayed Pick result futures (`result_ready_at=1.5`) and assert
  `transport_latched < pick_result_ready`; a runner-level negative proves that
  only settled post-result provider/history evidence fails closed
  (`evidence-invalid`, no Place, no release).  F2.3 — native-gripper
  rejection coverage is completed: a fresh nonzero baseline that stays
  unchanged passes, an increment-after-acceptance rejects the approach trigger
  (exact Pick cleaned up, no attachment/later goal, artifacts
  `evidence-invalid`), and missing/stale/provider-exception evidence fails
  closed.  F2.4 — runner-level receipt-window negatives prove the approach
  FJT before the acceptance baseline, the approach FJT later than 2.0 s, the
  transport FJT before the lift latch, the transport FJT later than 2.0 s
  after the lift latch, and a Place target-motion FJT outside its window are
  each bounded and `evidence-invalid` with no forbidden later goal/release.
  F2.5 — occupied-place re-observes a fresh PlanningScene after the exact
  Place cancel terminal and quiescence, proving `pick_and_place/object_mesh`
  remains attached; if open/detach wins the race Gate E fails
  `evidence-invalid`, and the post-cancel scene sequence/attachment state is
  recorded in the trigger artifact.  F2.6 — `_e_unexpected_exception` derives
  cleanup/goal identity/sent flags BEFORE durable writes; every durable
  artifact (`integrated-execution.jsonl/.json`, `moveit-plans.jsonl`,
  `controller-results.jsonl`, `goals/<id>.json`) truthfully preserves the
  accepted Pick/Place handle, goal IDs, goals sent, cleanup outcome, trigger,
  and `status=evidence-invalid` (no row claims no goal was sent when one was
  accepted).  F2.7 — blocked-approach/unreachable-grasp require production-real
  terminal consistency: action-client GoalStatus ABORTED (6) together with a
  non-success/non-canceled task result such as `planning_failed` (2); a
  contradictory SUCCEEDED-terminal/failure-Result (or canceled/safety Result)
  pair is rejected.  Humble suite grows to 155 tests and the pure suite to 126
  (F2.1 `_post_grasp_lift_m_observation` threshold + transport-kind set tests).
  No build or live run is required.

- 2026-08-03 (integrated qualification Task 6, formal-review fix round 1 —
  "make Gate E live-observable"): Resolved the full F1.1-F1.15 consolidated
  SPEC/QUALITY findings in the Stage-E path.  F1.1/F1.2 — the positive and
  occupied-place sequences now observe and latch lift-complete and transport
  **while the Pick goal remains executing** (production `stay=false` returns to
  `Q_OUTBOUND` before publishing the result); the transport trigger is a
  two-phase temporal predicate: `lift_complete` latches only after observed
  attachment + TCP z above the configured lift threshold + settled arm velocity
  (<= `settled_speed_m_s`) + two consecutive fresh normal-state samples, and
  `transport_started` then requires a **later** fresh FJT EXECUTING entry, target
  still attached, and fresh TCP speed >= the trigger limit, without re-requiring
  the settled condition while moving; receipt sequences/timestamps prove the
  transport evidence is later than the lift latch.  F1.3 — safety-transport uses
  the correct operator polarity (assert with `publish_operator(True)`, clear with
  `False` only after the effective stop) and the Humble test drives a causal fake
  publisher asserting the exact publication order `[True, False]`.  F1.4 — every
  accepted cancel/safety interruption boundedly awaits the exact goal result and
  records both status domains from the actual handle/result (canceled Pick/Place:
  action-client GoalStatus CANCELED=5 + `Result.status=4`; safety-transport:
  `Result.status=5` safety_stop + action-client ABORTED terminal per production
  `complete_pick` abort semantics).  F1.5 — the complete trigger object (join
  key, task/FJT UUIDs, receipt seq/time, TCP z/speed, arm velocity/normal
  samples, attachment, trigger kind, safety/cancel evidence) is persisted into
  `integrated-execution.jsonl/.json` and the per-scenario `goals/<id>.json`
  `geometry` is written from the validated dispatch spec (asserted exact fixed
  geometry), with trigger fields preserved in final downgrade rows.  F1.6 —
  `E_FJT_CORRELATION_TIMEOUT_S == 2.0` now bounds every FJT receipt window
  (approach vs goal-acceptance baseline, transport vs lift latch, Place
  target-motion vs Place baseline) and the actual receipt delta is recorded;
  stale/received-before-boundary entries are `None` and never satisfy a trigger
  (boundary tests just inside/outside the window).  F1.7 — every public E entry
  resets all per-attempt state (`_tcp_pose_samples`, native gripper seam,
  active-goal handle/state) before any provider sample; a reused-executor test
  proves no previous sample can satisfy the next attempt.  F1.8 — the shared
  `_fixture_scene_error` keeps Gate C/D exact fixture-only validation strict: a
  stray `pick_and_place/*` world object fails fixture readiness with the exact
  ordered "must equal" message, and only the exact task target is permitted via
  the explicit Stage-E `allow_e_target` argument.  F1.9 — every E public dispatch
  fails closed on any escaping exception (`reason_code="unexpected-exception"`)
  with bounded cleanup of the accepted goal and complete Gate-E artifacts, and an
  injected snapshot failure after Pick acceptance is verified end-to-end.  F1.10 —
  cancel-approach uses the injected receipt-sequenced
  `native_gripper_goal_count_provider` seam (not the fake-only `sent_goals`
  attribute); the fresh count must not increase and the target must stay
  unattached, with missing/stale/provider-error evidence failing closed.  F1.11 —
  sourced-Humble integration tests cover both `_run_e_blocked_or_unreachable`
  scenarios (real accepted Pick goal, terminal non-success in both status
  domains, exact short journal, no Place/later goal, complete fail-dominant
  artifacts, and success-status rejection).  F1.12 — `__all__` no longer lists
  the removed module-level `run_pick_place_*` stubs (`import *` succeeds) and
  malformed-back requires no TCP provider because it rejects before any motion.
  F1.13 — pre-goal evidence-invalid paths write canonical `planning-scene.json`
  even when the journal has no prior records.  Humble suite grows to 141 tests
  and the pure suite to 124 (import-star, receipt-window boundaries, C/D fixture
  strictness regression, blocked/unreachable, per-attempt reset, pre-goal
  artifacts, unexpected-exception cleanup).  No build or live run is required.

- 2026-08-03 (integrated qualification Task 6 — "add fixed-target Pick and Place
  controls"): Added the Stage-E fixed-target Pick/Place diagnostics executor to
  the same ROS-lazy `validation/integrated_gate_executor.py`.  A closed ROS-free
  `stage_e_dispatch` validates exactly the eight committed Stage-E scenarios
  (`qualification-pick-place-{positive,blocked-approach,unreachable-grasp,
  malformed-back,cancel-approach,cancel-transport,safety-transport,
  occupied-place}`) for exact id, `integrated.stage == "E"`,
  `integrated.execution_profile == "sim_ompl"`, exact declared polarity
  (positive/negative), the exact configured `expected_physical` list, the exact
  `expected_negative` required/forbidden contract, exact
  `forbidden_endpoints == ["/isaac_joint_commands"]`, exact declared
  `trigger_timeout_s`, and the pinned fixed-target geometry
  (`grasp_tcp_xyz [0.65, 0, 0.72]`, `object_root_xyz [0.65, 0, 0.60]`,
  `place_target_point` base_link `[0.85, 0, 0.72]`, identity orientation) plus
  the declared six-value malformed-back vector; unknown/C/D-stage, malformed, or
  mutated scenarios fail closed before any goal is created or sent.  The positive
  sequence uses only production `/pickup_action` then `/place_action` with the
  deterministic cube cloud and exact seven-joint `Q_OUTBOUND`, `use_mesh=True`,
  `stay=False`, and requires observed PlanningScene attachment before
  `scene-attach` and observed detach before `scene-detach` (never action-result
  inference).  The E journal branches per scenario: the positive order equals
  `POSITIVE_ORDER` exactly and each negative uses its own exact diagnostic event
  order with scenario-specific forbidden events, so Gate-C/D journal bytes stay
  unchanged.  Live TCP evidence reuses the existing injected
  `current_tcp_pose_provider` seam (never an embedded TF listener): a bounded
  per-attempt sample deque derives `tcp_z_m` and `tcp_speed_m_s = |Δxyz|/Δt`
  from the two newest fresh receipt-sequenced samples; fewer than two fresh
  samples means the trigger cannot fire and a timeout is evidence-invalid, never
  a pass.  FJT correlation is receipt-window only (first fresh FJT EXECUTING
  entry after the goal-acceptance baseline for approach, first later one for
  transport) with observed UUIDs recorded as evidence — the internal Pick/Place
  ExecuteTrajectory UUID is never claimed observable.
  cancel-approach/cancel-transport cancel the exact Pick handle at the trigger;
  safety-transport asserts operator safety, requires effective-stop evidence,
  clears only after the stop, and requires quiescence with no auto-resume;
  occupied-place sends Place and cancels at the first fresh Place target-motion
  trigger while the target remains attached (never waiting for the natural
  failure path that may open/detach).  malformed-back calls the real builder with
  the committed six-value back vector and requires a pre-send `ValueError` with
  zero action traffic and no PlanningScene mutation.  The fail-dominant artifact
  transaction mirrors Gate D with `event="gate-e"`, `stage="E"`, handler/polarity,
  pick/place goal-sent flags, both the action-client `GoalStatus` and the
  Pick/Place `Result.status` domains, task/FJT UUIDs, trigger record, and
  `isaac_joint_commands_published=false` across `integrated-execution.jsonl/.json`,
  `moveit-plans.jsonl`, `controller-results.jsonl`, visual-capture requests, and
  per-scenario goal artifacts; any journal/graph/artifact failure downgrades every
  status-bearing stream to `evidence-invalid`.  Task 6 records diagnostics only —
  raw contact/lift/release/collision verdicts remain Task 7 verifier work.
  Humble suite grows to 136 tests (fixed Pick/Place geometry, malformed-back
  pre-send rejection + zero traffic, positive full sequence + observed-attach
  guard, cancel-approach/cancel-transport/safety-transport/occupied-place,
  status-domain separation, receipt-window FJT selection, artifact
  fail-dominance); the pure suite grows to 121 tests (all eight dispatches +
  mutation rejection, per-kind journal orders/forbidden events, occupied-place
  fixture ownership, TCP speed/z derivation and undersampling).  No build or live
  run is required.

- 2026-08-03 (integrated qualification Task 5, formal-review fix round 2 —
  "seal Gate D evidence streams"): Resolved the formal SPEC/QUALITY residual
  findings F2.1-F2.7 in the Stage-D evidence path.  F2.1 — a D artifact-write
  downgrade now appends a final corrective row (`row_kind="final"`,
  `status="evidence-invalid"`) to ALL THREE status streams
  (`integrated-execution.jsonl`, `moveit-plans.jsonl`, `controller-results.jsonl`),
  preserving `planner_status`, `plan_applicable`, handler/action endpoint,
  UUID/digest/controller fields, `downgraded_from`, and the stable artifact
  error; no corrective row ever claims pass, and a failed corrective append is
  contained so the authoritative atomic summary stays fail-dominant.  F2.2 —
  `run_gripper_sequence(open_first=False)` (close→open) selects the close-first
  journal order `fixture-ready → gripper-close-terminal → gripper-open-terminal
  → teardown` via a fresh journal rebuild before its first record; an
  in-progress/nonempty journal is never mutated and the attempt fails closed
  with `journal-order-rebuild-refused`.  F2.3 — D scene-acquisition failures
  (`no-planning-scene`, `planning-scene-invalid`) route through the D evidence
  helper with `stage=D`, `event=gate-d`, the D handler label, and the D
  controller/artifact schema (never a Gate-C `gate-c-plan-only` record); Gate-C
  callers and bytes are unchanged.  F2.4 — a split-path execute whose accepted
  ExecuteTrajectory handle has an invalid/identical/non-normalizable UUID is
  cleaned up with exactly one bounded cancellation attempt before the evidence
  is finalized invalid, and the cleanup outcome is recorded without claiming a
  successful cancel unless the exact CancelGoal contract is met.  F2.5 —
  `_env_cloud_evidence` now requires structural PointCloud2 self-consistency:
  `row_step >= width*point_step` (valid row padding allowed) and
  `len(data) == row_step*height` (truncated and oversized buffers both rejected),
  plus a usable x/y/z FLOAT32 field layout when fields are advertised
  (unadvertised bytes are consumed as opaque payload and documented); invalid
  evidence fails closed before any action goal.  F2.6 — both MoveGroup builders
  explicitly pin `goal.planning_options.look_around = False`, the dead
  fail-open helpers `_safety_terminal_status` / `_safety_velocity_frames` and
  the unused `_d_journal_pass` are deleted, and `_wait_for_fjt_status` captures
  its matched entry in a single predicate result (no second lookup).  F2.7 —
  `controller_goal_sent` is pinned to the exact FJT semantic (a
  `follow_joint_trajectory` controller goal); retreat/gripper traffic is
  surfaced through explicit `action_goal_sent`/`action_endpoint` plus
  `cartesian_goal_sent`/`gripper_goal_sent` fields in `controller-results.jsonl`
  without pretending it was an FJT goal, and D visual-capture requests use
  `capture.kind="gate-d-diagnostic"` while Gate-C bytes keep `plan-only`.
  Humble suite expanded to 125 tests (all-stream corrective rows for
  execute/retreat/gripper, close-first pass and order-replacement guard,
  D-labeled scene-acquisition failures, accepted-UUID cleanup, truncated/
  oversized/undersized-row-step/valid-padded-row cloud structural probes,
  look_around pin, dead-helper removal, and execute/retreat/gripper action
  semantics artifacts); the pure suite stays 97 tests.  Red/green: 18 tests
  fail against the pre-fix base and pass at the fix.  No build is required.

- 2026-08-02 (integrated qualification Task 5, pre-review fix round 1 — "make
  Gate D runtime truthful"): Hardened the Stage-D lifecycle and evidence path
  against the real Humble rclpy action API.  F1.1 — the executor keeps a private
  `_owned_action_clients` collection of every real rclpy `ActionClient`, separate
  from the mutable public client map that tests may replace with fakes, and
  `shutdown()` now destroys every owned ActionClient exactly once (removing its
  waitable and C handle) before the node and private-context teardown, clearing
  lifecycle members so GC cannot double-finalize; a partial-constructor failure
  destroys already-created clients before node/context cleanup, and repeated
  construct/shutdown stays supported (this removes the leading in-process suspect
  for the coordinator's order-dependent full-suite SIGSEGV).  F1.2 — a cancel
  pass is impossible without the exact live ExecuteTrajectory `ClientGoalHandle`;
  raw UUID kwargs never substitute; `cancel_goal_async()` is called exactly once
  on that handle and requires the real-shape `CancelGoal.Response` contract
  (`return_code == ERROR_NONE` and `goals_canceling == [execute_goal_id]`), the
  ExecuteTrajectory result must be terminal CANCELED (5), and the joined FJT
  controller goal must reach CANCELED (5) with bounded quiescence — SUCCEEDED/
  ABORTED/unknown/empty/extra/malformed/exceptional/timed-out responses fail
  closed.  F1.3 — safety requires the `fjt_transaction_provider` evidence to
  validate (endpoint/UUID/digest/source/sequence/timestamp/status) and join to a
  fresh status-topic entry inside the current attempt window; provider exception,
  no-provider, stale, wrong-UUID/digest/source/endpoint/status-cache, and
  prior-ABORTED-cache entries fail closed (never swallowed, never an unrelated
  cached ABORTED goal).  F1.4 — FJT/status and joint-state evidence is windowed
  to the current attempt with internal receipt baselines; bounded spin/wait
  helpers cover the joined EXECUTING motion trigger, joined terminal status,
  no-active-goal quiescence, `safety_stop_frames` consecutive fresh bounded joint
  frames, and bounded post-clear stability using `safety_position_creep_rad`.
  F1.5 — an accepted ExecuteTrajectory goal that times out or fails early is
  cleaned up via bounded cancellation on the exact handle, recording the cleanup
  outcome without ever claiming cancel success unless the F1.2 contract was met.
  F1.6 — all six D handlers (execute joint/pose, cancel, safety, retreat,
  gripper) route success and failure through one fail-dominant
  `_finalize_d_attempt`/`_write_d_artifacts` path writing the complete
  authoritative set (`integrated-execution.jsonl/.json`,
  `moveit-plans.jsonl` with an explicit `plan_applicable=false`/
  `planner_status=null` for the non-MoveIt retreat/gripper handlers,
  `controller-results.jsonl`, truthful visual-capture-request rows, and
  scenario-specific `goals/<scenario_id>.json`); required goal/JSONL write
  failures propagate into the Task-4 transactional downgrade, and every D journal
  snapshot return is checked at its event boundary.  F1.7 — `run_cartesian_retreat`
  requires an explicit fresh `environment_cloud_provider` returning a real
  non-empty finite `base_link` `sensor_msgs/msg/PointCloud2`; that exact cloud is
  passed into `CartesianMove.Goal.env_points` and only then is
  `collision_checking` recorded true; missing/empty/malformed/stale/wrong-frame/
  provider-exception/serialization-failure fails closed before goal send.  F1.8 —
  Gate-C journal bytes and public policy are unchanged; D visual captures use
  real before/after chronology; real-shape `GetResultService.Response`
  (`status`/`result`) and `CancelGoal.Response` (`return_code`/`goals_canceling`)
  test doubles replace the legacy handle-conflation model; `execute_timeout_s`
  is in the test config.  Humble suite expanded to 109 tests (lifecycle destroy
  spies + repeated real-context stress, cancel/safety fail-closed matrices,
  windowed-evidence helper tests, timeout cleanup, goal-artifact/journal
  fail-dominant injection, retreat env-cloud validation, execute-pose coverage,
  visual chronology); the pure suite stays 97 tests.  No build is required.

- 2026-08-02 (integrated qualification Task 5): Added the Stage-D execution
  interruption gates to the same ROS-lazy executor.  A closed ROS-free
  `stage_d_dispatch` validates exactly the six Stage-D scenarios
  (`qualification-moveit-execute-joint`, `-execute-pose`, `-cartesian-retreat`,
  `-gripper`, `-cancel`, `-safety`) for exact id, `integrated.stage == "D"`,
  `execution_profile == "sim_ompl"`, exact declared polarity (`positive` /
  `cancel` / `safety`), the exact configured `expected_physical` list, and
  `forbidden_endpoints == ["/isaac_joint_commands"]`, failing closed before any
  goal on unknown/C/E-stage/malformed/mutated scenarios.  The mandatory split
  path (`run_execute_sequence`) sends exactly one OMPL plan-only `/move_action`
  goal (`xarm7`, `ompl`, attempts 3, allowed time 3.0, `plan_only=True`,
  `look_around=False`, `replan=False`), requires a non-empty
  `planned_trajectory`, assigns it unchanged to exactly one
  `moveit_msgs/action/ExecuteTrajectory.Goal` (`build_execute_trajectory_goal`;
  canonical ROS-serialized digest unchanged before/after), sends it to
  `/execute_trajectory`, and records distinct valid 16-byte plan/execute UUIDs.
  Terminal action statuses use action_msgs constants (SUCCEEDED=4, CANCELED=5,
  ABORTED=6; unknown/malformed never pass) via `_execute_status_name`.  FJT
  observation is truthful: only the real
  `/xarm7_traj_controller/follow_joint_trajectory/_action/status` subscription
  (`action_msgs/msg/GoalStatusArray`, RELIABLE/TRANSIENT_LOCAL/depth 1) exists —
  no `_action/goal` topic (the goal travels over the `send_goal` service) — with
  a bounded cache, and the executor requires an injected
  `fjt_transaction_provider` returning real controller-transaction evidence
  (endpoint exactly the FJT endpoint, normalized FJT goal UUID, canonical
  trajectory digest equal to the unchanged ExecuteTrajectory digest, finite
  timestamp/sequence, real source) that joins to the newest status entry;
  missing/stale/mismatched/extra/malformed/provider-exception evidence makes the
  D attempt `evidence-invalid`.  `run_cancel_sequence` calls
  `cancel_goal_async()` only on the ExecuteTrajectory `ClientGoalHandle` (never
  the completed MoveGroup planning handle), records exactly the execution UUID in
  `goals_canceling`, requires terminal CANCELED (5), requires no active joined
  FJT status before `quiescent=true`, and never sends a later stage.
  `run_safety_sequence` publishes operator True via the real
  `/sim/safety/operator` publisher, waits bounded for safety-stop True, requires
  the old ExecuteTrajectory terminal status ABORTED (6), requires
  `safety_stop_frames` consecutive fresh joint-state velocity frames bounded by
  `safety_stop_velocity_rad_s` (0.02), publishes operator False only after the
  effective-stop predicate, requires bounded post-clear stability, and sends no
  replacement/resume goal.  `run_cartesian_retreat` uses an injected
  `current_tcp_pose_provider` (no embedded TF listener), derives exactly
  `RETREAT_DISTANCE_M` (0.10 m) along `+Z` in `base_link` preserving orientation
  (`derive_retreat_target_pose`), and sends one collision-aware
  `/cartesian_move_action` goal with `command_gateway_bypassed=false` as a
  routing assertion.  `run_gripper_sequence` sends open (0.0) then close (0.85)
  `control_msgs/action/GripperCommand` goals to `/xarm_gripper/gripper_action`
  with max effort 10.0 (`build_gripper_goal`) and `native_action=true`.  A
  fail-closed `run_pick_place_negative` stub covers the two Gate-E cancellation
  identities (`cancel-approach`/`cancel-transport`) with `release_stage_started=false`,
  `released=false`, and zero goals, rejecting all others.  D-stage artifacts use
  Task 4's transactional/fail-dominant mechanics with a separate D record shape:
  `diagnostic_only=true`, `physical_verdict=None`, fail-dominant
  `status`/`planner_status` split, truthful `execute_trajectory_goal_sent` /
  `controller_goal_sent`, plan/execute/goals-canceling UUIDs, planned/executed/
  FJT trajectory digests, FJT goal UUID/status, execute result status/string,
  event log, elapsed, and `isaac_joint_commands_published=false`; required
  write/finalization failures downgrade every authoritative D artifact.  The
  journal uses scenario-specific D diagnostic event orders (execute/pose:
  `fixture-ready → execution-start → execution-terminal → teardown`; retreat:
  `fixture-ready → retreat-start → retreat-terminal → teardown`; gripper:
  `fixture-ready → gripper-open-terminal → gripper-close-terminal → teardown`;
  cancel: `fixture-ready → execution-start → cancel-requested → quiescent →
  teardown`; safety: `fixture-ready → execution-start → effective-stop →
  operator-clear → quiescent → teardown`) with the eight Task-4 forbidden
  manipulation events unchanged, the Task-3 graph projection byte-identical, and
  no attach/detach/release event in Gate D.  Humble suite expanded to 74 tests
  (split-path unchanged digest + distinct UUIDs, real FJT status QoS/type, no
  `_action/goal` subscription, gripper/cartesian goal construction, stubbed
  execute/cancel/safety/retreat/gripper flows, Gate-E stub zero traffic); the
  pure suite expanded to 97 tests (Stage-D dispatch mutations, status mapping,
  UUID normalization, retreat derivation, Gate-E stub, D-vs-C journal order,
  unchanged graph projection).  No build is required because no
  package-installed path changed.

- 2026-08-02 (integrated qualification Task 4): Added the ROS-lazy Gate-C OMPL
  plan-only executor.  `validation/integrated_gate_executor.py` is importable
  under the simulator CPython 3.12 venv without `rclpy` or any generated ROS
  message; it exposes the exact graph/endpoint/QoS contract, the real
  multi-operation canonical public report validation
  (`expected_physics_ready_report` / `validate_physics_ready_snapshot` built
  through `tinker_sim_bridge.integrated_readiness.build_canonical_report` with
  the one-key public `{"execution_profile": "sim_ompl"}` mapping and
  scenario-declaration-bound fixture descriptor digest), the readiness
  evaluator (`evaluate_executor_readiness`) over the genuine positive-ready
  baseline, the Task 3 journal graph projection
  (`build_journal_graph_projection` matching `planning_scene_journal.validate_graph_evidence`),
  ROS-lazy MoveGroup/Pick/Place/PointCloud2 builders, and the live
  `IntegratedGateExecutor` node `/tinker_integrated_gate_executor` that sends
  only plan-only `/move_action` goals (`group_name="xarm7"`, `pipeline_id="ompl"`,
  `num_planning_attempts=3`, `allowed_planning_time=3.0`,
  `planning_options.plan_only=True`, `replan=False`), never calls
  `/execute_trajectory` in Gate C, and never publishes `/isaac_joint_commands`.
  `validation/manipulation_qualification.py` gains only additive helper
  exposure (`QualificationProcessHelpers` plus module-level
  `qualification_ros_tooling_environment` / `qualification_write_json_atomic` /
  `qualification_new_suite_dir` / `qualification_source_inventory`) with the
  six-gate `run()` behavior, artifact schema, and command ordering unchanged.
  `tests/qualification_test_helpers.py` additively returns the complete
  seven-key `report_identities` and full `planning_scene_declaration`.
  New suites: `tests/test_integrated_gate_executor.py` (pure, 55 tests) and
  `tests/test_integrated_gate_executor_ros.py` (Humble generated-message, 13
  tests).  No build is required because no package-installed path changed.

- 2026-08-02 (integrated qualification Task 4 fix round 1): Made the Gate-C
  executor runnable and Task-4 evidence-complete.  The live
  `IntegratedGateExecutor` now constructs a valid isolated Humble node: a
  private `rclpy.context.Context` per executor initialized with the exact
  `ros_domain_id` in `[0,232]`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` required/
  verified, basename `tinker_integrated_gate_executor` + namespace `/` +
  `use_global_arguments=False` (FQN `/tinker_integrated_gate_executor`),
  dict-key (never `getattr`) typed action/service client creation with all nine
  action and eleven service clients asserted, a context-bound
  `SingleThreadedExecutor`, idempotent `shutdown()`, and construct -> shutdown ->
  construct reuse.  The journal recorder now subscribes to the real
  `moveit_msgs/msg/PlanningScene` type on `/planning_scene` and
  `/monitored_planning_scene` (reliable/transient-local/depth 1) with a
  ROS-only normalization helper deriving ordered world/attached/link/touch data,
  the exact fixture revision, an internal `scene_sequence`/`scene_timestamp`,
  and SHA-256 digests over ROS serialization of the full scene, ACM, and robot
  state.  The executor owns a real `PlanningSceneJournal` by default
  (loaded model touch contract, `pick_and_place/` task namespace,
  `pick_and_place/object_mesh` target, Stage-C explicit
  `("fixture-ready", "teardown")` event order, the eight forbidden
  manipulation events, fresh `planning-scene.jsonl` that fails closed when
  stale) and records `fixture-ready` via `record_diff` then `teardown` via
  `snapshot`, finalizing both journal artifacts with the exact observed graph
  projection.  `build_journal_graph_projection` now requires an explicit
  `observed_graph` input and fails closed on missing/extra interfaces, wrong
  type/QoS/source/cardinality, or an absent recorder subscriber/client; the
  executor requires `join_key_provider`, `readiness_snapshot_provider`, and
  `graph_observation_provider`, gates every goal on live readiness (config-
  authoritative operator freshness), and evaluates the three Stage-C scenario
  readiness baselines (their own revision, owned IDs, and descriptor digest).
  `run_gate_c_plan_only` dispatches joint/pose/blocked plan-only goals with
  non-empty-`planned_trajectory` enforcement (blocked expects planner
  non-success), uses separate bounded deadlines for server availability /
  acceptance / result / cancellation with `cancel_goal_async()` spun to
  completion, rejects non-Stage-C/non-plan-only scenarios before goal creation,
  and writes the complete Task-4 artifact set (`integrated-execution.jsonl/.json`,
  `moveit-plans.jsonl`, `controller-results.jsonl`, `goals/<scenario_id>.json`,
  `visual-capture-requests.jsonl`, `planning-scene.jsonl/.json`) with
  `diagnostic_only=true`, `execute_trajectory_goal_sent=false`, and
  `isaac_joint_commands_published=false`.  Humble suite expanded to 29 tests
  (real-class construction, typed subscriptions, scene normalization, stubbed
  three-scenario flow, nonempty-plan/blocked enforcement, readiness gating,
  bounded cancellation, repeated construct), and the documented sourced-Humble
  command FAILS (never skips) without the ROS runtime.  Pure suite expanded to
  66 tests (Stage-C readiness baselines, dispatch semantics, observed-graph
  fail-closed mutations, journal lifecycle/stale-jsonl/forbidden events).
  No build is required because no package-installed path changed.

- 2026-08-02 (integrated qualification Task 4 fix round 2): Made Gate-C evidence
  fail-dominant and aligned PlanningScene QoS with stock MoveIt2 Humble.  The
  authoritative final status is now computed after the plan outcome *and* every
  required evidence finalization step; any readiness/journal/graph/
  finalization/artifact-serialization/existence failure returns and persists
  `evidence-invalid` in `run_gate_c_plan_only` and `integrated-execution.json`,
  with the raw planner outcome preserved separately as `planner_status`, and
  `planning-scene.json` is always produced as a canonical failure artifact via a
  narrow `PlanningSceneJournal.finalize_failure` extension.  Every expected
  runtime/DDS/action failure (server wait, goal construction/serialization,
  `send_goal_async`, send-future spin/result, goal acceptance, result-future
  spin/result, cancellation, provider calls, artifact finalization) is converted
  into finite canonical diagnostic records with zero physical claim and exact
  zero-command/controller flags; once `fixture-ready` exists the executor always
  attempts teardown journal completion and failed finalization, and no exception
  escapes the public API.  The `/planning_scene` and `/monitored_planning_scene`
  subscriptions and the Task-3/4 observed-graph projection now use the stock
  MoveIt2 Humble RELIABLE/VOLATILE/depth-100 contract (the stale
  TRANSIENT_LOCAL/depth-1 claim is rejected); `/sim/status/planning_scene_fixture`
  stays RELIABLE/TRANSIENT_LOCAL/depth 1.  The blocked scenario now only passes
  on an explicit MoveIt planning-stage non-success allowlist (`PLANNING_FAILED`,
  `INVALID_MOTION_PLAN`, `NO_IK_SOLUTION`); request-level/unknown codes are
  `diagnostic-fail` with a recorded `error_code_classification`.  A bounded
  pre-goal scene-acquisition phase self-spins up to `scene_acquire_timeout_s`,
  `fixture-ready` requires the scene's ordered owned IDs to match the declared
  fixture contract (missing/extra/reordered/attached rejected before goal send),
  the `before` visual-capture request is durably flushed before the goal send and
  `after` only in the post-transaction phase, acceptance-timeout handling is
  truthful (canceling a client future is not proof of server-side cancellation),
  and the executor's atomic write now includes parent-directory fsync.  Humble
  suite expanded to 43 tests; pure suite to 68; journal suite to 184.  No build
  is required because no package-installed path changed.

- 2026-08-02 (integrated qualification Task 4 fix round 3): Sealed Gate-C
  artifact/scene-state consistency across the residual review findings.
  Artifact finalization is now transactional: `run_gate_c_plan_only` validates
  the graph through `PlanningSceneJournal.finalize(status, graph, json_path=None)`
  before any durable output, writes the non-journal artifacts first, and defers
  the successful `planning-scene.json` until every other required artifact is
  durable.  Any post-provisional-pass write/serialization/fsync/existence
  failure now invokes `_downgrade_persisted_evidence`, which rewrites
  `integrated-execution.json` fail-dominantly, appends `row_kind="final"`
  corrective rows to the JSONL lifecycle files (whose provisional/raw planner
  rows carry `row_kind="lifecycle"` so a consumer cannot mistake an early row
  for a completed pass), and writes a canonical failure `planning-scene.json`.
  `planner_status="diagnostic-pass"` remains available as diagnostic history
  while every authoritative `status` field is fail-dominant, and the
  `PlanningSceneJournal.finalize_failure` protection is unchanged.  Scene
  acquisition is no longer latch-permanent: a successfully normalized valid
  PlanningScene callback atomically caches the new scene and clears the invalid
  flag (recording `_scene_invalid_sequence` on failure), and acquisition
  requires a valid observation whose `scene_sequence` is after the last invalid
  one, so an invalid message never erases the last valid cached scene yet
  acquisition stays fail-closed while invalid is the newest observation.  The
  callback boundary catches the complete expected malformed-message exception
  set (AttributeError/IndexError/KeyError/TypeError/ValueError) without
  swallowing process-control exceptions.  `fixture-ready` now binds to the
  exact declared geometry and pose, not IDs only: `expected_fixture_geometry_digest`
  derives a deterministic projection via the bridge canonicalization helpers
  `fixture_to_specs`/`spec_geometry`/`readback_geometry`/`geometry_signature_sha256`
  covering the scenario-owned objects' exact ordered IDs, primitive geometry and
  dimensions, frame, and poses, and `_fixture_scene_error` compares the received
  scene's projected digest to the declaration, rejecting stale pose, wrong
  geometry/dimensions/frame, and duplicate IDs.  The blocked scenario now
  requires both an allowlisted planning-stage non-success code and an empty
  planned trajectory; an allowlisted code with a non-empty trajectory is
  classified `contradictory-nonempty-trajectory` and fails.  This entry
  supersedes the fix-round-2 "Humble suite expanded to 43 tests" count with the
  fresh 44-test baseline measured at `d911692` before fix round 3; the post-fix
  counts are Humble 58, pure 70, journal 184.  No build is required because no
  package-installed path changed.

- 2026-08-02 (integrated qualification Task 3 fix round 1): Bound the
  PlanningScene evidence semantics against the adversarial review findings.
  `validation/planning_scene_journal.py` now makes the public path fail closed:
  `record_diff` validates a real world-to-attached / attached-to-world
  transition for the exact target at `scene-attach` / `scene-detach` through a
  shared transition validator (also used by `assert_transition`), and
  `snapshot` rejects the transition labels `scene-attach`, `scene-detach`, and
  `task-cleanup` that cannot truthfully be represented without a scene diff.
  Object disappearance is gated for every diff, not just `task-cleanup`;
  `teardown` alone may additionally remove `sim_fixture/*`.  `finalize` rejects
  an empty journal and, when a required event order is declared, requires the
  recorded events to equal it exactly (rejecting extra, duplicate, out-of-order,
  and spurious negative-teardown events).  Returned records and final
  records/graph are deep-copied so caller mutation can never diverge the JSONL
  from final JSON.  Appending onto a pre-existing non-empty JSONL path fails
  closed.  Graph validation requires the exact topic/service key sets, the
  recorder node among topic subscribers and service clients, exact QoS key sets
  with a non-boolean integer `depth`, and exact (never string-coerced) fixture
  payload field types with unique `sim_fixture/*` owned ids; the exact canonical
  compact fixture payload string is retained as the payload evidence.  Nested
  scene types are strict (`ValueError`, no `str()` coercion, no uncaught
  `TypeError`), and `_append` independently re-validates the full scene.  The
  focused suite grew to 181 tests.

- 2026-08-02 (integrated qualification Task 3): Added the stable, durable
  PlanningScene journal consumed by the later executor/verifier/evidence tasks.
  `validation/planning_scene_journal.py` is the ROS-free pure record/transition
  journal: a model-contract loader (`load_model_touch_contract`) anchored to the
  committed `integration/ompl-overlay-contract.json` (`link_tcp`, the exact
  ordered eight-link SRDF touch set, and handoff
  `pick_and_place/object_mesh`, failing closed on drift), transactional
  validation (a rejected record/event leaves `_records`, `_last_scene`, JSONL
  bytes, and final JSON bytes unchanged), recursive rejection of forbidden
  physics fields (contact/force/object-pose/evaluator-metric/physical-verdict),
  append-only canonical compact sorted-key `planning-scene.jsonl` (flush +
  fsync before in-memory visibility), atomic final `planning-scene.json`
  (temp-file + file fsync + `os.replace` + directory fsync, no temp residue),
  graph-evidence validation (`validate_graph_evidence`) over a Task-4-supplied
  projection (recorder identity, topic/service types and QoS, real
  endpoint/provider metadata, exactly one `/fixture_planning_scene` publisher,
  and the exact canonical compact fixture-status payload with scalar
  `target_handoff="pick_and_place/object_mesh"`), and positive/negative event
  ordering with attach/detach/task-cleanup ownership semantics.  The journal
  records the exact `(frame_index, timestamp)` join key and never claims that
  scene attachment proves physical contact.  `tests/test_planning_scene_journal.py`
  covers every brief case plus adversarial digest/rollback/numeric/duplicate/
  physics-key/contract-drift/graph/durability/order coverage.

- 2026-08-02 (integrated qualification Task 2 fix round 3): Closed the residual
  semantic false-passes.  `validation/integrated_static_contracts.py`
  `_bind_run_post_close_pick` now aggregates every SimOmpl/Hardware profile
  block and validates the whole set (exactly one obstruction guard, one SimOmpl
  lift block, one Hardware lift block; per-block polarity; structural
  containment of every `execute_lift(...)` call by an accepted profile lift
  block) instead of checking only the last matching block, so a decoy block
  after a violating one can no longer pass.  Result-builder field checks use
  the exact assignment form `\bresult\s*->\s*<field>\s*=(?!=)` over sanitized
  executable code, so a differently-named member, a read, or a comparison never
  satisfies the write contract.  `_bind_bundle_artifact` binds a non-simulator
  artifact by its semantic source key plus exact path identity (bundle absolute
  path == `<repo_path>/<path_relative>`, overlay workspace path ==
  `src/<repo dir>/<path_relative>`) and requires the recorded digest to equal
  the verified source record; simulator-local acceptance is restricted to the
  declared overlay path under `outputs/` or `artifacts/`, and a shadow file
  with matching bytes at an undeclared path never reclassifies.  The Task 2
  report now records only independently reproduced test evidence.

- 2026-08-02 (integrated qualification Task 2 fix round 2): Strengthened the
  semantic source checks.  `validation/integrated_static_contracts.py` replaces
  comment-stripped string-presence C++ scans with a deliberately scoped
  lexical/structural layer: comments, string/character and raw-string literals
  are sanitized (preserving braces/newlines), conditional-preprocessor
  directives inside load-bearing function bodies are rejected (so `#if 0` /
  `#ifdef` dead-code anchors cannot satisfy a check), and each inspected
  function (result builders, `GraspNode::~GraspNode`,
  `GraspNode::clean_planning_scene`, `GraspNode::move_straight`,
  `run_post_close_pick`, `coordinator_main`, `~MotionRuntime`) is located by
  its actual signature and brace-matched.  The Sim/Hardware `execute_lift`
  boolean is bound to its own `ExecutionProfile` branch, the destructor
  deadline/shutdown/join/state-validity reset are bound to the destructor body,
  and each task result builder must write exactly its `.action` schema fields.
  The authoritative overlay `model_bundle.production_source_commits` is bound
  to immutable Git blobs (40-hex commit existence, blob digest, manipulation
  ancestry vs `implementation_head`; external `tk25_basic` via exact recorded
  commit/blob only) and drives both the model-artifact binding and the
  pinned-prerequisite source-identity check.  `fixture-ownership` now requires
  the overlay `scenarios` key set to equal the configured C/D/E set exactly.
  `source_lock_manifest.py` closes the observer gaps: qualification-tooling
  requires clean mode with empty evidence, `authorization.report_path` must
  resolve (no symlink/canonical escape) to a regular file inside the
  repository, `_normalize_posix` rejects `..` and canonicalizes absolute
  repository paths, the stale mtime-fallback docstring is removed, and the
  output manifest is written/fsynced/replaced exactly once from a single
  pre-write timestamp.

- 2026-08-02 (integrated qualification Task 2 fix round 1): Bound Gate B to
  real artifacts.  `validation/integrated_static_contracts.py` no longer reads
  the fabricated `qualification/records/*` layer: every production source
  (SRDF `_xarm7_macro.srdf.xacro`, `config/xarm7` and `config/xarm_gripper`
  `controllers.yaml`, the pick_and_place C++ runtime and result writers, the
  `.action` schemas, and the selected launch) is inspected as an immutable Git
  blob at the production `implementation_head` from the produced source-lock
  manifest.  The model-fingerprint check recomputes
  `structural_fingerprint == sha256(canonical bundle contract)`, binds every
  recorded model artifact SHA-256 and the `production_source_commits` blobs,
  and proves the ordered `xarm_gripper` members plus `link_tcp` end-effector
  parent.  `fixture-ownership` inspects exactly the 17 configured scenarios
  (C/D/E) and derives owned ids from `planning_scene.objects` in declared
  order versus `integrated.expected_scene.owned_ids`.  The transport-contract
  check reads `typed_contract.runtime_contract_sha256` from its sibling
  location, recomputes the canonical full runtime mapping, and preserves the
  exact public one-key mapping.  Provider checks recompute the
  `provider_manifest_sha256` canonical self-hash and raw bytes and reconcile
  the provider executable set with the overlay.  `source_lock_manifest.py`
  gained closed policy/authorization schemas, the resolved lock-commit scope
  rule (`{policy_path, docs/acceptance.md}`), the qualification HEAD exact-match
  requirement, authorization report existence/predate plus bogus/valid
  authorization-commit checks, canonical repository-relative `policy_path`,
  required ISO `started_at`, and a stable output-freshness boolean.  The
  static checker emits canonical finite JSON with fsync + atomic replace for
  `static-contract.json`, `model-fingerprint.json`, and
  `source-identities.json`, and propagates `evidence-invalid` /
  `verified-fail` / `verified-pass` per F4.  The real-root post-fix Gate B
  capture remains `evidence-invalid` solely because the future
  qualification-tooling policy is intentionally absent; eight of the nine
  static checks pass against the real simulator/production roots and only
  `source-identities` fails on that absent policy.

- 2026-08-02 (integrated qualification Task 2): Added the offline Gate B
  static-contract closure.  `validation/source_lock_manifest.py` observes the
  three immutable source-lock authorizations
  (`simulator_overlay` / `production` / `qualification_tooling`) with
  non-self-referential Git-history resolution, exact `LC_ALL=C` raw
  status/diff/index/untracked evidence, a canonical path-sorted untracked
  manifest, an atomic fsync+`os.replace` output manifest, and attempt
  freshness.  `validation/integrated_static_contracts.py` runs the nine
  semantic static checks (model, controllers, providers, fixture ownership,
  action lifecycle, scene/collision safety, source identities, transport
  contract) against the produced three-entry manifest.
  `simulation/qualification/integrated-ompl.json` now names exactly the three
  `source_lock_policies`.  The real qualification-tooling policy is
  intentionally absent during Task 2, so a real-root Gate B capture honestly
  returns `evidence-invalid` until the post-Task-10 lock-only phase; no
  existing source-lock policy was created or modified.
- 2026-08-02 (fix round 2): Separated static and runtime OMPL acceptance
  evidence.  The 16-check preflight is now self-contained: the real unmodified
  `preflight_manifest` runs against a reconstructed Task 3-compatible project
  root (committed source + pinned git objects + a legacy `current.json` selector
  pointing at the reproduced canonical URDF), so `ready=true` and the stable
  preflight hash reproduce with no gitignored `outputs/`/`artifacts/`
  dependency.  The real `current.json` is a separate provisioned-host runtime
  readiness diagnostic (stale selection fails; absent selection is reported
  `not_provisioned`, not a static failure).  Top-level `evidence.preflight` now
  carries the load-bearing `stable_manifest_sha256`/`stable_preflight_sha256`
  and nests the raw hashes under `host_snapshot`.  Task 7 action-client scope is
  AST-grounded in pinned Task 7 source (exactly one MoveGroup client on
  `/move_action`); task_range boundary subjects + exact Task 3-7 commit list are
  recorded and recomputed; fixture status publication is recomputed from Task 5
  source; every artifact `path_relative` follows the recorded source policy; a
  temporary copy-install/wheel test proves the symlinked contract + scenarios
  install as real byte-identical files; the pinned v1 canonicalizer is loaded by
  deterministic temporary-package materialization (no `exec` string surgery);
  joint-limit acceptance is semantic + toolchain-aware; and a clean-checkout
  regression seam (`git clone` of the tracked tree) requires every static test to
  pass with no gitignored trees.
- 2026-08-02 (fix round 3): Locked the clean-checkout acceptance coverage.  The
  regression seam now collects the exact 64-node `Task8OMPLOverlayProvenanceTest`
  class in the clone (canonical SHA-256 over the sorted node list, so a deleted
  or added test fails even if the count is preserved) and executes it under a
  machine-readable JUnit XML, requiring total=64, failures=0, errors=0, skipped=4,
  the exact four host-runtime diagnostic skip names, and their reason categories —
  an unrelated static assertion broadened into a skip is rejected.  The
  reconstructed Task 3 resolver (`tools/tinker_sim_deploy`) is now materialized
  from immutable git objects at the recorded simulator implementation identity
  (never copied from the live working tree, so local edits cannot influence
  acceptance), and a pinned-blob mismatch/missing file fails closed.  The
  fixture status field contract is asserted against an independent 12-field
  literal (not re-derived from the source function), and a stale
  `current.json` selector is proven to fail the preflight `artifact_identity`
  check.  The fix-1 baseline is documented as 2 failed + 1 skipped.
- 2026-08-02 (fix round 4): Isolated the pinned-resolver verification.  Because
  the test module pre-imports `tinker_sim_deploy` from the live working tree,
  Python's module cache masked the earlier in-process dirty live-tree probe
  (the proof did not actually execute the pinned materialized resolver).  The
  reconstruction proof now runs in a fresh isolated subprocess (`-I`, no
  inherited `tinker_sim_deploy*` cache) that loads `tinker_sim_deploy.runtime`
  from the materialized temp root, records its resolved `__file__`, runs the
  real Task 3 preflight, and emits one machine-readable JSON with
  ready/16-check/exact-stable-hash evidence; a temp decoy working-tree package
  proves the pinned path wins path precedence (positive) and is load-bearing
  (negative).  No test writes a tracked active-checkout file.  The clean-checkout
  seam's collection/JUnit acceptance checks are factored into one deterministic
  validator used by both the real clone output and realistic mutated fixtures
  (delete, rename+add, unrelated skip, removed skip, wrong reason, duplicate
  testcase, failure/error count, multiple suites), and the JUnit structure is
  tightened to exactly one `<testsuite>` with 64 unique `<testcase>` entries.
- 2026-08-02 (fix round 1): Made the OMPL-overlay acceptance evidence
  reproducible.  The production-overlay scope is split into the exact production
  allow-lists (from `launch_contract_helpers.py`, including
  `re`/`importlib`/`rviz2`) and a simulator-overlay provider set derived from
  the provider manifest; the provenance suite now recomputes the 18-argument
  contract, literal booleans, strict-sim keys, allow-lists, handoff, and
  action-client scope from immutable production git objects.  The model-bundle
  evidence is reproducible on a clean checkout: the canonical simulator full
  URDF source + provenance descriptor are committed under
  `integration/model-bundle-r2/simulator_full_urdf/`, and stable manifest /
  preflight hashes exclude host-absolute paths and `elapsed_ms`.  The
  acceptance contract and three qualification scenarios are installed under
  `share/tinker_sim_bridge/{integration,scenarios}/` with a deterministic
  launch package-share fallback.  `tests/test_provenance.py` fails (never
  skips) when required source evidence is absent, and the documented focused
  pytest invocation uses `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`.  The two
  source-lock files remain Task 9 only.
- 2026-08-02: Added the deterministic OMPL overlay acceptance contract
  (`integration/ompl-overlay-contract.json`) packaging the reviewed Tasks 3-7
  interfaces: exact repository/commit identities (simulator `f34de5f`;
  production runtime hardening `f3e2ce4..df702a5` and Task 2 launch
  `f7fea50..39d96a1` from actual git history), production overlay and
  literal-false `use_cumotion_*` compatibility values, provider-manifest raw +
  canonical identities, model-bundle schema/artifact hashes and full eight-link
  touch set, typed action/service/topic contract, fixture/scenario identities,
  Task 6/7 evidence, and `tkbuild`/`build-humble-overlay` build commands.
  `tests/test_provenance.py` recomputes every derived hash/contract.  The two
  source-lock files remain Task 9 only.
- 2026-07-30: Added deterministic exporter-side canonical URDF derivation with
  full-SHA immutable generation identity, exact portable source-lock identity,
  serialized crash-consistent publication, physical arm-joint validation,
  relocatable Humble launch inputs, shared runtime validation, and bundle
  admission checks. The USD physics bytes remain unchanged.
