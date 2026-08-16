# Simulation vision stack — design

Date: 2026-08-16. Branch: `task50-stage-a-repair`. Status: approved design, pre-implementation.

## Problem

The project declares camera topics (`simulation/sensors/hardware-parity.json`) but publishes none:
no code loads that file, `ros_gateway.py` has no `Image`/`CameraInfo`, and the documented
`--sensor-profile sensor-rich` launch is accepted by argparse and then raises
`RuntimeError` at `validation/run_sim.py:805`. Meanwhile the real consumers in
`~/tk25_ws/src/tk26_vision` have a strict, verified contract that the declaration contradicts:

- Every `CameraInfo` subscription there is RELIABLE (`qos_profile=10`); several image
  subscriptions (`vision_util/get_image.py`, `get_point_cloud.py`, others) are RELIABLE.
  The declared `qos: sensor_data` (BEST_EFFORT) would deliver **zero messages, silently**.
  The real drivers publish RELIABLE + VOLATILE + KEEP_LAST(10)
  (`tk26_vision/config/realsense_qos.yaml`).
- Depth is decoded as raw `np.uint16` millimetres with no encoding check
  (`person_track_node.py:305`, `follow_head.py:912`, `seat_recommend_bbox.py:349`).
  32FC1 metres would produce silently wrong geometry.
- Color must be exactly 3 bytes/pixel; `person_track_node.py:611` ignores `msg.encoding`.
- Color+depth pairs must satisfy `ApproximateTimeSynchronizer` slop as tight as 0.05 s.

Feasibility is verified: the robot USD contains `head_camera_color_optical_frame` and
`xarm_camera_color_optical_frame` Xform prims; `isaacsim.sensors.experimental.rtx`
`CameraSensor` supports `rgb` and `distance_to_image_plane` annotators; the gateway already
publishes ROS 2 Humble messages from Isaac's Python 3.12 process; RTX rendering is proven
live on this host (`reports/rtx-sensors-latest.json`, `reports/arena-vision-latest.json`).

## Goal

Publish the head and wrist camera streams from Isaac on the real-driver contract, wire them
into a working `sensor-rich` profile, and prove the result end to end by driving the real,
unmodified `vision_util get_image` node from `~/tk25_ws` against the simulator.

**Acceptance:** a `GetImage` service call against the live sim returns `status=0` with a
synchronized color+depth pair of the declared encodings and resolutions; with wall coloring
enabled, the returned color frame contains the deterministic arena palette hues
(classified by the tested `hue_presence` helper). This is recorded as a Humble-side pytest
plus captured evidence.

## Non-goals

- IR/stereo streams, `realsense2_camera_msgs/Extrinsics`, FoundationStereo support.
- TF aliasing (`camera_color_optical_frame` ↔ URDF `head_camera_*` frames) and any
  `map`-frame TF work for TF-consuming vision nodes.
- `use_sim_time` rollout to tk26_vision nodes.
- Fixing tk26_vision bugs (e.g. `door_detection`'s 5-float point stride) — the sim
  reproduces real-driver behavior, it does not patch consumers.
- Release qualification. This work produces development-validated vision, honestly labeled.

## Architecture

Four units, each with one purpose:

### 1. Camera rig — `simulation/tinker_sim_isaac/camera_rig.py` (new)

- **Spec loading:** `hardware-parity.json` (schema bumped to v2) becomes the single source
  of truth. Frozen `CameraStreamSpec` per camera: topics, `camera_info_topics` (list),
  `frame_id`, `mount_prim`, `width`/`height`, `horizontal_fov_deg`, `tick_rate_hz`,
  encodings, QoS label. Malformed specs raise — never guess.
- **Cameras:** one `RtxCamera` per spec, created as a child prim of
  `/World/Tinker/<mount_prim>` so it tracks pan/tilt and arm motion through physics.
  Missing mount prim → hard error at init. Local orientation applies the fixed
  optical-frame→USD-camera rotation (optical +Z forward / +Y down vs USD −Z forward / +Y up).
  Focal length / apertures derived from `horizontal_fov_deg` with square pixels;
  `CameraInfo` uses the same math (`fx = width / (2·tan(hfov/2))`, `fy = fx`, centered
  principal point, `plumb_bob` with zero distortion, `P` from `K`, `R = I`), so render and
  intrinsics are consistent by construction. FOVs approximate the real cameras
  (head ≈ 90°, wrist ≈ 69.4°); consistency is the requirement, exact hardware match is not.
- **Capture:** `CameraSensor(camera, resolution=(H, W), annotators=["rgb",
  "distance_to_image_plane"])`, manual tick. One capture yields color+depth together.
- **Pure helpers** (unit-testable without Isaac): `depth_to_16uc1_mm` (metres→mm,
  NaN/Inf→0, clamp 65535), `rgb8_bytes` (strip alpha, enforce 3 channels),
  `camera_info_fields`, `pack_registered_cloud` (float32 xyz + 4-byte pad,
  `point_step=16`, organized `height×width`, NaN for invalid — matching the real driver
  layout, including its documented incompatibility with `door_detection`'s 5-float parser).

### 2. Gateway extension — `simulation/tinker_sim_isaac/ros_gateway.py`

- New optional constructor args: `camera_rig`, `camera_pointcloud: bool`.
- Publishers at **RELIABLE + VOLATILE + KEEP_LAST(10)** — the verified real-driver profile.
  Per head camera: color `Image`, depth `Image`, color `CameraInfo`. Per wrist camera:
  color `Image`, aligned-depth `Image`, color `CameraInfo`, **aligned-depth `CameraInfo`**
  (same intrinsics; the topic real consumers use that the old declaration omitted).
- `publish_cameras()` stamps every stream of one capture with the **same sim-time stamp**
  (`/clock` domain), making the 0.05 s sync slop trivially satisfied.
- A camera whose annotator returns no frame this tick is skipped and counted
  (`camera_skipped_frames` in the status payload) — fail-open per tick, visible in telemetry,
  never a fabricated frame.
- Point cloud (only when `camera_pointcloud`): `/camera/depth_registered/points` from the
  head depth + intrinsics, same stamp as its source capture.

### 3. `sensor-rich` branch — `validation/run_sim.py` (+ `tools/tinker_sim_deploy/cli.py`)

- Implements the currently-broken profile: whole-robot backend (`IsaacWholeRobotBackend`,
  current.json artifact, colocated arena3 `map.yaml`, scenario objects via
  `_expected_scenario_objects`, `enable_contacts=False`), gateway with `camera_rig`,
  development lidar enabled (`gateway_lidar_enabled` gains `sensor-rich → True`).
- `sensor-rich` **implies `--ros`** (forced with a printed note): the profile's purpose is
  hardware-parity topics.
- Camera cadence: target 15 Hz, decimated from the 120 Hz physics loop with the existing
  stride pattern (`max(1, round(1/(dt·hz)))`). Loop shape mirrors manipulation-core
  (spin, step, publish, `app.update()`); `publish_cameras()` runs every Nth frame.
  Achieved rate is measured during acceptance; resolution/rate knobs are the fallback if
  the 2080 Ti can't hold 15 Hz — the freshness floor for consumers is ~10 Hz.
- New flags, plumbed through `cli.py`: `--camera-pointcloud` (default off) and
  `--arena-colors` (default off; colors the occupancy walls with the deterministic
  6-hue palette for visually verifiable frames). The palette moves to
  `simulation/tinker_sim_core/arena_palette.py` (single source; `backend.py` gains an
  optional `wall_color_fn` parameter defaulting to the current uniform gray;
  `validation/arena_vision_smoke.py` imports instead of redefining).
- Startup JSON gains a `cameras` block (topics, resolutions, target rate, pointcloud,
  arena_colors) alongside the existing contract fields.

### 4. Contract + docs

- `hardware-parity.json` → schema v2: `implementation: gateway_rtx_camera_publishers`,
  `qos: reliable_volatile_keep_last_10`, explicit `color_encoding: rgb8`,
  `depth_encoding: 16UC1` + `depth_unit: millimeter`, `camera_info_topics` lists,
  `mount_prim`, FOV/resolution/rate, and the optional `point_cloud` block.
- `integration/modules.json` vision entry: mode updated to the real mechanism, status
  updated to reflect the live round-trip evidence, still not release-qualified.
- README: replace the broken `sensor-rich` example with the working launch and document
  the two-terminal acceptance procedure.

## Acceptance procedure (live round-trip)

1. Terminal A (Isaac env, no `/opt/ros` sourced): launch `sensor-rich` with `--ros
   --arena-colors` on an **isolated `ROS_DOMAIN_ID`** (the running arena session owns
   domain 25 and is not to be disturbed).
2. Terminal B (system Humble + `~/tk25_ws` overlay, same domain): `ros2 run vision_util
   get_image`, then run `tests/ros_humble/test_vision_get_image_live.py`, which calls
   `get_image_service` for the head (`orbbec`) and wrist (`realsense`) cameras and asserts:
   `status=0`; encodings `rgb8`/`16UC1`; declared resolutions; color/depth stamp delta
   = 0; depth values plausible (nonzero mm within arena scale); palette hues present in
   the head color frame (`hue_presence`).
3. Evidence: pytest output + returned frames saved under `reports/`, referenced from the
   README section.

The live test skips (does not fail) when the sim topics are absent, following the
`tests/ros_humble` importorskip convention, and is additionally gated by an env var so it
never fires in offline suites.

## Error handling

- Spec/prim/annotator failures at init are hard errors (fail-closed) — a sim that cannot
  provide the declared contract must not start under `sensor-rich`.
- Per-tick capture misses are skipped-and-counted (fail-open) — a transient render stall
  must not kill a long-running session; the counter makes it observable.
- All pure helpers reject malformed input with `ValueError` (house style).

## Testing

- `tests/test_camera_rig.py`: spec parsing against the committed v2 JSON, FOV→intrinsics
  math, depth conversion (NaN/Inf/clamp/rounding), rgb8 channel handling, cloud packing
  byte layout (`point_step=16`), fail-closed malformed specs. Plain `python3 -m unittest`,
  no Isaac.
- Extend arena/launcher tests: `sensor-rich` implies-ros rule, new CLI flags in
  `_launch_command`, stride math, palette module import, `backend` `wall_color_fn` default.
- `tests/ros_humble/test_vision_get_image_live.py` as above.
- Full existing suite must stay green (`python3 -m unittest discover -s tests -v`).

## Risks

- **Render throughput** on the below-baseline host: two RTX cameras at 15 Hz beside CPU
  PhysX. Mitigation: measured during acceptance; rate/resolution knobs; 720p+480p targets
  chosen to match consumer expectations, not maximums.
- **Annotator warmup**: first frames can be empty; init pumps render updates (existing
  pattern from `qualification_visual_capture.py`) before declaring ready.
- **Artifact drift**: a regenerated robot artifact without the mount prims fails loudly at
  init by design.
- **Point-cloud bandwidth** (toggle only): 720p organized cloud ≈ 14.7 MB/msg; Fast DDS
  SHM (default `local` profile) carries it; divisor knob if packing cost shows up.
