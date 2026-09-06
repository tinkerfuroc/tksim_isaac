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

When the client network cannot reach the server directly over UDP, tunnel
both WebRTC transports over SSH instead with `./scripts/connect-arena-streaming
tinker@tkserver.example.net`. See
[`docs/streaming-guide.md`](docs/streaming-guide.md) for the arena-streaming
viewer's behavior, the SSH tunnel setup, and client-side connection gotchas.

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

Status: development-validated with a recorded live round-trip; **not
release-qualified**. See [`docs/developer-log.md`](docs/developer-log.md) for
the recorded result and a real `tk26_vision` defect this check found upstream.

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
inside `shelf_02`'s physical and rasterized footprint: the robot would stand
under the shelf plate, the dev lidar's occupied sensor origin would publish
an empty cloud rather than useful returns, wheel odometry would accumulate
contact slip, and AMCL would be initialized inside an occupied map cell. The
launch now fails closed on this instead of spawning into it — see "the
default robot spawn" under Known arena limitations below for the exact
error and its evidence. Navigation work in this arena therefore passes
`--spawn-xy=X,Y` (world metres; use the `=` form — argparse treats a
separate `-2.0,-2.0` token as an option string) to place the robot on a free
map cell, e.g. `--spawn-xy=-2.0,-2.0` (1.0 m clearance on the derived map).
The override is validated fail-closed (two finite comma-separated numbers)
and requires a profile that loads the robot backend, like `--arena` itself.
On the Humble side, `./scripts/launch-humble navigation
map_yaml:=/abs/path/to/artifacts/arena/rcw2026/<identity>/map.yaml` points
AMCL's map server at the arena's derived map instead of the robot artifact's
colocated default; the override fails closed on a missing file. See
"Navigation launch and scenario execution" immediately below for how
`navigation.launch.py` now manages its own safety source.

**Navigation launch and scenario execution.** `navigation.launch.py` now
starts its own `safety_supervisor` node (`manage_controllers:=false
required_sources:=["collision"]`), so the workflow that previously needed a
separate 10 Hz CLI heartbeat to keep `/sim/safety/operator` cleared no
longer applies when launching navigation this way. This path is
code-complete but not yet live-verified (see `docs/developer-log.md` for
why the verification attempt didn't count).

Two arena-native scenario variants exercise the arena directly:
`find-and-approach-person-rcw2026` and `pick-deliver-place-rcw2026`
(`simulation/scenarios/`), declaring `world: {"mode": "arena", "arena":
"rcw2026"}` with poses verified against the committed derived occupancy map.
Run either with `scenario_runner`:

```bash
PYTHONPATH=$PWD/simulation:$PYTHONPATH \
  ros2 run tinker_sim_bridge scenario_runner \
  --root $PWD --scenario find-and-approach-person-rcw2026 --seed 7
```

(`scenario_runner` imports `tinker_sim_core` at module level, so a bare
`ros2 run` needs the `PYTHONPATH` above — see Known arena limitations.) A
scenario's `actor_path_start` events (for example the person walking toward
the robot) are executed separately by `actor_path_driver`, a one-shot node
that drives actors along their declared paths via `/set_entity_state`:

```bash
ros2 run tinker_sim_bridge actor_path_driver \
  --root $PWD --scenario find-and-approach-person-rcw2026
```

`actor_path_driver` resolves `tinker_sim_core` from `--root` itself (commit
`bd4b553`) and does not need `PYTHONPATH` set. Both commands require
`ROS_DOMAIN_ID` to match the Isaac-side domain the arena sim was launched
on (`42` in the live acceptance runbook above). `actor_path_driver` itself
is NOT proven live: a prior crash on the first `/clock` message is fixed
(commit `de95b71`), but the fix hasn't been re-run live (see
`docs/developer-log.md`).

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

Status: development-validated only, **not release-qualified**. Import and
AABB/hash checks are green; live navigation, AMCL, camera, and scenario
runs against the arena were recorded in
[`docs/developer-log.md`](docs/developer-log.md) (2026-08-17 through
2026-08-20), including two open findings worth knowing before relying on
this arena: scenario-spawned objects may not be physics-simulated, and
`.deployment.env` silently resets `ROS_DOMAIN_ID` to 25 after it is sourced
(export the run's domain again afterward).

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

## Integrated OMPL qualification

`integration/ompl-overlay-contract.json` is the deterministic acceptance
contract for the reviewed OMPL overlay; `validation/integrated_qualification.py`
runs its Gate A-F qualification suite. Full contract details, the CLI's
`--attempt-root`/`--stage` usage, and the verifier/contact-sheet/evidence-index
companion tools are in
[`docs/integrated-ompl-qualification.md`](docs/integrated-ompl-qualification.md).
No live Gate F/OMPL/cuMotion claim is made by this repo's own tests; cuMotion
remains prohibited until Task 37's live OMPL qualification passes.

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

See [`docs/CHANGELOG.md`](docs/CHANGELOG.md) for the dated history of fixes
and features, and [`docs/developer-log.md`](docs/developer-log.md) for the
root-cause investigation narratives behind them.
