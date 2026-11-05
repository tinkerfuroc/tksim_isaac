# Top-Lidar Placeholder Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A separate, opt-in sim setup in which Tinker carries a floating Mid-360 at 1.2 m, upside down and tilted 45° nose-down, publishing `/livox/lidar_top` while a route driver drives a fixed route past the arena's tables, without changing anything about normal runs.

**Architecture:** A build script derives a robot artifact `artifacts/robot/tinker2-toplidar/` from the selected tinker2 artifact by adding one bare `livox_top` frame: a URDF link and fixed joint, and a matching USD Xform under `base_link`. The sim loads it through `run_sim`'s existing `--artifact` flag and a new `--lidar-config` flag pointing at `simulation/sensors/top-lidar.json`. The lidar rig learns to apply the mount link's rotation. A new launch plus wrapper script runs the bridge nodes (no Nav2), a static `map → odom`, and a route driver that steers by wheel odometry. An acceptance script checks, from sim truth, that the cloud lands on the table tops and the floor.

**Tech Stack:** Python 3.12 (`.venv`, with `pxr`, `numpy`), ROS 2 Humble (`rclpy`, `tf2_ros`, `robot_localization`), Isaac Sim 6.0.1 `isaacsim.sensors.experimental.physics.Raycast`, Pillow.

**Spec:** `docs/superpowers/specs/2026-10-22-top-lidar-placeholder-design.md`, Part 2. Starts only after `docs/superpowers/plans/2026-10-22-coacd-furniture-hulls.md` has shipped (the user chose that order).

## Global Constraints

- **Normal runs must not change.** With no new flags, `run_sim` loads the tinker2 artifact and `simulation/sensors/hardware-parity.json`, publishes `/livox/lidar` in `livox360`, and builds the base lidar exactly as before. `artifacts/robot/tinker2/current.json`, `hardware-parity.json` and every existing launch file are not edited.
- Mount defaults (spec): 1.2 m above the floor, i.e. joint `z = 1.2 − base_rest_height` (base_rest_height ≈ 0.0775, from `robot_geometry.base_rest_height(urdf)`); `x = 0.09`, `y = 0`; URDF `rpy = (π, π/4, 0)`, i.e. `R = Ry(45°)·Rx(180°)`. All are build-script parameters.
- Names (spec): frame and link `livox_top`, joint `livox_top_joint`, topic `/livox/lidar_top`, robot artifact id `tinker2-toplidar`, sensor config `simulation/sensors/top-lidar.json`.
- The cloud is in the **tilted** `livox_top` frame. The static transform comes from `robot_state_publisher` with the variant URDF.
- Same Mid-360 contract as the base lidar: −7…+52° × 360°, 57×349 contract, `TINKER_SIM_LIDAR_*` overrides, sweep, 0.2 m min range, 40 m max, self-filter.
- The route driver never reads `/sim/truth/*` (`simulation/README.md` reserves them for the evaluator). The acceptance script may.
- Tests: `scripts/pytest-clean <file> -q` (venv). ROS-node logic lives in pure functions tested under the venv; `rclpy` is imported only inside `main()`.
- Shared working tree: `git add <your paths>`, check `git diff --cached --name-only`, then `git commit`. Never `git commit -a` or `git commit -- <paths>`. Wait if `.git/index.lock` exists.
- Live runs (Task 7): GPU must be free (`nvidia-smi --query-compute-apps=pid --format=csv,noheader` prints nothing) or ask the user; private ROS domain (check `grep -l ROS_DOMAIN_ID=<n> /proc/*/environ`); `timeout -k 10 <s>` on every live command; tear down with `scripts/kill-domain-nodes <domain> --force`.

## Review Focus

- A mount with **identity** rotation (the base `livox_frame`) produces exactly today's `Raycast.create` kwargs: no `orientations` key at all. Pinned by `test_identity_mount_kwargs_unchanged` (Task 1).
- Running the variant build twice gives the same artifact identity and never touches tinker2's pointer. Pinned by `test_build_is_idempotent` and `test_tinker2_pointer_untouched` (Task 3).
- The build script run against a tinker2 URDF that already has `livox_top` (e.g. the real new robot later) fails loudly instead of adding a second link. Pinned by `test_existing_livox_top_rejected` (Task 3).
- A route segment that clips a wall or furniture would drive the robot into it, because there's no Nav2. Pinned by `test_default_route_clear_of_map` (Task 5).
- The route driver never spins forever at a waypoint it can't reach exactly: position and yaw tolerances and a per-waypoint timeout end it. Pinned by `test_waypoint_timeout_reports_failure` (Task 4).

---

### Task 1: The lidar rig applies the mount link's rotation

**Files:**
- Modify: `simulation/tinker_sim_isaac/lidar_rig.py` (`RaycastLidar.initialize`, `_resolve_mount`; new module-level helpers)
- Test: `tests/test_lidar_rig.py`, new `tests/test_lidar_mount_rotation.py`

**Interfaces:**
- Produces: `mount_offset(body_world, mount_world) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]`. Takes two `Gf.Matrix4d` and returns (translation, orientation wxyz) of the mount relative to the body, using the same `mount_world * body_world.GetInverse()` product `_resolve_mount` uses today, with `ExtractRotationQuat()` for rotation.
- Produces: `raycast_create_kwargs(spec: LidarSpec, origins, directions, offsets, translation, orientation_wxyz) -> dict[str, Any]`, the dict `initialize` passes to `Raycast.create`. It adds `"orientations": np.array([orientation_wxyz], dtype=np.float32)` **only** when the orientation differs from (1, 0, 0, 0) by more than 1e-9 in any component; otherwise the dict equals today's.
- `_resolve_mount` returns `(body_path, translation, orientation_wxyz)`. `RaycastLidar` gains `mount_orientation: tuple[float, float, float, float]` (default identity), and `build_lidar_rig`'s log line (run_sim.py) adds `orient=<wxyz>`.

- [ ] **Step 1: Write the failing tests**
- `test_identity_mount_kwargs_unchanged`: for the shipped spec, `raycast_create_kwargs(..., orientation_wxyz=(1,0,0,0))` has exactly the keys `min_range, max_range, ray_origins, ray_directions, output_frame, report_hit_prim_paths, translations, ray_time_offsets`; `output_frame == "SENSOR"`.
- `test_tilted_mount_adds_orientations`: orientation = quat of `Ry(π/4)·Rx(π)` → `"orientations"` shape (1, 4), equal to the input within 1e-6.
- `test_mount_offset_extracts_rotation` (`importorskip("pxr")`): body at (1, 2, 0) yaw 30°, mount = body · translate(0.09, 0, 1.12) · rotation `Ry(π/4)·Rx(π)` → translation (0.09, 0, 1.12) within 1e-9, rotation equal to that quat up to sign.
- `test_mount_offset_identity_for_welded_frame`: mount = body · translate(0.09, 0, 0.195) → orientation (1, 0, 0, 0) within 1e-9.

- [ ] **Step 2: Run them to see them fail**

Run: `scripts/pytest-clean tests/test_lidar_mount_rotation.py tests/test_lidar_rig.py -q`
Expected: new tests FAIL (names not defined); existing PASS.

- [ ] **Step 3: Implement.** Move the inline kwargs construction in `initialize` into `raycast_create_kwargs`, and the matrix maths in `_resolve_mount` into `mount_offset`.

- [ ] **Step 4: Run the tests**

Run: `scripts/pytest-clean tests/test_lidar_mount_rotation.py tests/test_lidar_rig.py tests/test_lidar_profile.py tests/test_raycast_lidar_gate.py -q`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add simulation/tinker_sim_isaac/lidar_rig.py validation/run_sim.py tests/test_lidar_mount_rotation.py
git commit -m "feat(lidar-rig): apply the mount link's rotation (identity unchanged)"
```

### Task 2: Selectable lidar config; the gateway publishes on the spec's topic

**Files:**
- Create: `simulation/sensors/top-lidar.json`
- Modify: `validation/run_sim.py` (`--lidar-config`, `build_lidar_rig`, new `resolve_lidar_config`), `simulation/tinker_sim_isaac/ros_gateway.py` (~l.215-221 cloud publisher)
- Test: `tests/test_lidar_rig.py`, `tests/test_ros_gateway.py`, new `tests/test_lidar_config_flag.py`

**Interfaces:**
- `simulation/sensors/top-lidar.json`: `{"schema_version": 2, "lidar": {...}}`. The lidar block is a copy of `hardware-parity.json`'s, with `pointcloud_topic: "/livox/lidar_top"`, `frame_id: "livox_top"`, `raycast.mount_prim: "livox_top"`, and `scan_topic: "/livox/scan_top"` (required by the loader, unused: no `pointcloud_to_laserscan` runs in top-lidar mode). Add a `"_comment"` saying this is the experimental top-lidar placeholder and naming the spec. It has no camera or IMU blocks, and cameras keep reading `hardware-parity.json`.
- `run_sim.py`: `--lidar-config PATH` (default `None`) and `resolve_lidar_config(root: Path, value: Path | None) -> Path`. `None` gives `root/"simulation/sensors/hardware-parity.json"`; a relative path is resolved against `root`; a missing file raises `FileNotFoundError`. `build_lidar_rig(root, app, config_path: Path)`. The log line adds `config=<path relative to root>`. (`main()` builds its parser inline, so the testable piece is `resolve_lidar_config`, like `parse_spawn_xy` in `tests/test_spawn_override.py`.)
- `ros_gateway.py`: module-level `lidar_cloud_topic(lidar_rig) -> str`, returning `lidar_rig.spec.pointcloud_topic` when a rig is given and `"/livox/lidar"` otherwise. The cloud publisher uses it. `_LIVOX_FRAME_ID`, the IMU and `_development_point_cloud` are untouched.

- [ ] **Step 1: Write the failing tests**
- `test_top_lidar_config_loads`: `load_lidar_spec("simulation/sensors/top-lidar.json")` → topic `/livox/lidar_top`, frame `livox_top`, mount `livox_top`, and channels/columns/elevations/ranges equal to the shipped contract's.
- In `tests/test_lidar_config_flag.py`: `test_default_lidar_config_is_hardware_parity`, `test_relative_lidar_config_resolved_against_root`, `test_missing_lidar_config_rejected`.
- `test_cloud_topic_follows_spec` and `test_cloud_topic_default_without_rig` (a stub object with `.spec.pointcloud_topic`).
- `test_shipped_contract_cloud_topic_unchanged`: `lidar_cloud_topic` for a rig built on `hardware-parity.json`'s spec is `/livox/lidar`.

- [ ] **Step 2: Run them to see them fail**

Run: `scripts/pytest-clean tests/test_lidar_rig.py tests/test_ros_gateway.py tests/test_lidar_config_flag.py -q`
Expected: new tests FAIL.

- [ ] **Step 3: Implement**

- [ ] **Step 4: Run the tests**

Run: `scripts/pytest-clean tests/test_lidar_rig.py tests/test_ros_gateway.py tests/test_lidar_config_flag.py tests/test_raycast_lidar_gate.py tests/test_ros_gateway_imu.py -q`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add simulation/sensors/top-lidar.json validation/run_sim.py simulation/tinker_sim_isaac/ros_gateway.py tests/test_lidar_rig.py tests/test_ros_gateway.py tests/test_lidar_config_flag.py
git commit -m "feat(sim): --lidar-config; cloud topic from the lidar spec"
```

### Task 3: The `tinker2-toplidar` robot artifact

**Files:**
- Create: `tools/tinker_sim_deploy/top_lidar.py`
- Create: `tools/build_toplidar_variant.py` (thin CLI)
- Test: `tests/test_top_lidar_variant.py`

**Interfaces:**
- Produces in `top_lidar.py`:
  ```python
  VARIANT_ID = "tinker2-toplidar"
  LINK = "livox_top"; JOINT = "livox_top_joint"

  @dataclass(frozen=True)
  class TopMount:
      height_above_floor_m: float = 1.2
      x: float = 0.09
      y: float = 0.0
      roll: float = math.pi
      pitch: float = math.pi / 4
      yaw: float = 0.0
      def joint_xyz(self, base_rest_height: float) -> tuple[float, float, float]: ...
      def quat_wxyz(self) -> tuple[float, float, float, float]: ...  # URDF rpy: Rz(yaw)·Ry(pitch)·Rx(roll)
      def as_record(self) -> dict[str, float]: ...

  def add_livox_top_link(urdf: bytes, mount: TopMount) -> bytes: ...
  def add_livox_top_prim(usd_src: Path, usd_dst: Path, mount: TopMount, base_rest_height: float) -> None: ...
  def build_variant(repo_root: Path, mount: TopMount) -> AssetPublication: ...
  def resolve_variant_artifact(repo_root: Path) -> Path: ...   # dir holding robot.usd + robot.urdf
  def ray_plane_hits(sensor_xyz_world, sensor_quat_wxyz, directions: np.ndarray, plane_z: float) -> np.ndarray: ...
  ```
- `add_livox_top_link` appends `<link name="livox_top"/>` and a fixed `livox_top_joint` (parent `base_link`, origin `xyz`/`rpy` from the mount, 9 significant digits) before `</robot>`. It raises `ValueError` if a link or joint with either name already exists. The rest of the bytes are untouched.
- `add_livox_top_prim` copies `usd_src` to `usd_dst`, finds the single prim named `base_link` carrying `UsdPhysics.RigidBodyAPI` (raise if not exactly one), defines child `UsdGeom.Xform` `livox_top` with a translate op (the joint xyz) and an orient op (the quat). No physics, collision, mass or visual. Re-exported fresh, like `arena_convert._resave_fresh`.
- `build_variant` resolves tinker2 through `runtime.resolve_current_artifact(repo_root)`, which verifies its hashes. It writes the variant's `robot.urdf` and `robot.usd` in a temp dir and publishes them with `publish_asset_artifact(repo_root, kind="robot", asset_id=VARIANT_ID, payload={"robot.urdf": ..., "robot.usd": ...}, source_lock={"derived_from": {"robot": "tinker2", "artifact_id": ..., "robot_urdf_sha256": ..., "robot_usd_sha256": ...}, "mount": mount.as_record(), "builder": "tools/build_toplidar_variant.py"})`, which selects the variant's own pointer.
- `resolve_variant_artifact` follows `artifacts/robot/tinker2-toplidar/current.json` → manifest → dir. It raises `FileNotFoundError` with the build command in the message if missing.
- `ray_plane_hits` rotates the sensor-frame directions by the quat, intersects rays from the sensor position with `z = plane_z`, and returns the (K, 2) XY hits of rays pointing down to that plane (`t > 0`).
- CLI: `.venv/bin/python tools/build_toplidar_variant.py [--height 1.2] [--x 0.09] [--y 0] [--roll 3.14159265] [--pitch 0.785398163] [--yaw 0]`. It prints the artifact dir and the joint origin.

- [ ] **Step 1: Write the failing tests** in `tests/test_top_lidar_variant.py`. URDF tests use a small inline URDF with `base_link` and a wheel.
- `test_link_and_joint_added_and_nothing_else`: the output parses; it has exactly one more link (`livox_top`, no inertial/visual/collision) and one more joint (fixed, parent `base_link`, origin `xyz="0.09 0 <1.2-rest>" rpy="3.14159265 0.785398163 0"`); removing those two elements gives XML equal to the input's.
- `test_existing_livox_top_rejected`.
- `test_forward_ray_points_45_down`: `quat_wxyz()` applied to (1, 0, 0) → z = −sin 45° within 1e-9.
- `test_cone_reaches_table_ahead`: `ray_plane_hits` for the shipped −7…+52° pattern (`lidar_pattern` on the shipped spec) from (0.09, 0, 1.2), plane z 0.75 (0.45 m below): the farthest-forward hit among those with |y| < 0.05 has x = 0.09 + 0.45/tan 38° ≈ 0.666 (± 0.03). This is the spec's "reaches a 0.75 m tabletop about 0.6 m ahead". (Don't assert that nothing lands behind the sensor: the sideways rays tilt down and back, so there are hits beside and behind it.)
- `test_prim_added_with_matching_transform` (`importorskip("pxr")`): a temp USD with a `base_link` rigid body and a child → after `add_livox_top_prim`, `livox_top` exists under `base_link`, has no physics/collision/mass API, and its local transform equals the URDF joint (translation within 1e-9, rotation within 1e-6). Every other prim's attributes are equal before and after (traverse both stages and compare `(path, attr, value)` sets minus the new prim).
- `test_build_is_idempotent` and `test_tinker2_pointer_untouched`: build into a temp repo root holding a fake tinker2 artifact (patch `resolve_current_artifact` to return it) twice → same identity; `artifacts/robot/tinker2/current.json` bytes unchanged.
- `test_real_variant_matches_real_tinker2` (skips when `artifacts/robot/tinker2/current.json` is absent, i.e. off this machine): build into a temp root from the real tinker2 files; the URDF differs only by the two elements and the USD only by the `livox_top` prim.

- [ ] **Step 2: Run them to see them fail**

Run: `scripts/pytest-clean tests/test_top_lidar_variant.py -q`
Expected: FAIL (module missing).

- [ ] **Step 3: Implement** `top_lidar.py` and the CLI.

- [ ] **Step 4: Run the tests**

Run: `scripts/pytest-clean tests/test_top_lidar_variant.py -q`
Expected: all PASS.

- [ ] **Step 5: Build the real variant**

Run: `.venv/bin/python tools/build_toplidar_variant.py`
Expected: prints `artifacts/robot/tinker2-toplidar/<identity>` and joint origin `0.09 0 1.1225` (with rest height 0.0775). `git status --short artifacts/` shows nothing (gitignored) and `artifacts/robot/tinker2/current.json` is unchanged.

- [ ] **Step 6: Commit**

```bash
git add tools/tinker_sim_deploy/top_lidar.py tools/build_toplidar_variant.py tests/test_top_lidar_variant.py
git commit -m "feat(top-lidar): build the tinker2-toplidar robot artifact"
```

### Task 4: Route driver

**Files:**
- Create: `ros2_ws/src/tinker_sim_bridge/tinker_sim_bridge/route_driver.py`
- Modify: `ros2_ws/src/tinker_sim_bridge/setup.py` (console script `route_driver = tinker_sim_bridge.route_driver:main`)
- Test: `tests/test_route_driver.py`

**Interfaces:**
- Route file (JSON):
  ```json
  {"schema_version": 1, "frame": "map", "arena": "rcw2026",
   "spawn": [-2.0, -2.0, 0.0],
   "waypoints": [{"xy": [x, y]}, {"xy": [x, y], "yaw": 1.57, "dwell_s": 3.0}]}
  ```
  `spawn` yaw must be 0.0, because a non-zero `TINKER_SIM_SPAWN_YAW` switches the backend's fabric mode (`backend.py:1386-1400`), which is a different sim. `yaw` and `dwell_s` are optional. A waypoint with `yaw` is a look stop: turn in place to it, then hold still for `dwell_s` of sim time (default 2.0) so the lidar collects frames.
- Pure functions:
  - `load_route(path: Path) -> Route` (frozen dataclass: `arena`, `spawn: tuple[float, float, float]`, `waypoints: tuple[Waypoint, ...]`). Raises `ValueError` on schema problems or non-zero spawn yaw.
  - `map_to_odom(route: Route) -> Route`: waypoints re-expressed in the odom frame, whose origin is the spawn pose.
  - `step(pose_odom: tuple[float, float, float], route: Route, state: DriverState, now_s: float, limits: Limits = Limits()) -> tuple[tuple[float, float], DriverState]`, returning `((v, w), new_state)`. `DriverState` holds the waypoint index, phase (`drive`, `turn`, `dwell`, `done`, `failed`), the phase start time and `failure: str`. `Limits(v_max=0.2, w_max=0.5, pos_tol=0.05, yaw_tol=0.05, waypoint_timeout_s=60.0)`.
- Control law (the signature doesn't fix it): in `drive`, turn toward the waypoint, and move forward only when the heading error is under 0.3 rad, with `v = min(v_max, 0.8·distance)` and `w = clamp(1.5·heading_error, ±w_max)`. Within `pos_tol`, go to `turn` (look stop) or the next waypoint. In `turn`, `w = clamp(1.5·yaw_error, ±w_max)` until within `yaw_tol`. In `dwell`, output zeros until `dwell_s` elapses. Exceeding `waypoint_timeout_s` in any phase → `failed` with the waypoint index in `failure`. `done` and `failed` output zeros.
- Node `main()`: params `route` (path), `use_sim_time` true. Subscribes `/tracer/odom` (pose from `nav_msgs/Odometry`), publishes `/cmd_vel` (`geometry_msgs/Twist`, reliable, as `base_facade` subscribes) at 20 Hz, and publishes `std_msgs/String` on `/sim/route/status` with JSON `{"phase", "waypoint", "of", "failure"}` at 2 Hz (latched with transient-local durability so a late subscriber sees `done`). It never subscribes to `/sim/truth/*`.

- [ ] **Step 1: Write the failing tests** (pure functions only):
- `test_load_route_rejects_nonzero_spawn_yaw`, `test_load_route_rejects_missing_waypoints`.
- `test_map_to_odom_identity_at_origin_spawn` and `test_map_to_odom_translates_spawn`: spawn (−2, −2, 0) → map (0, 0) is odom (2, 2).
- `test_drives_straight_to_waypoint`: simulate a unicycle (`x += v·cos θ·dt` etc., dt 0.05) from (0, 0, 0) to waypoint (1, 0) → reaches `done` within 10 s sim; |v| ≤ 0.2 and |w| ≤ 0.5 throughout.
- `test_look_stop_turns_then_dwells`: waypoint (0.5, 0) with yaw π/2, dwell 2.0 → ends at heading π/2 ± 0.05 and spends ≥ 2.0 s with zero output before `done`.
- `test_waypoint_timeout_reports_failure`: the simulated robot never moves → `failed` after 60 s, `failure` names waypoint 0.

- [ ] **Step 2: Run them to see them fail**

Run: `scripts/pytest-clean tests/test_route_driver.py -q`
Expected: FAIL (module missing). The test imports the module from `ros2_ws/src/tinker_sim_bridge` by inserting that path into `sys.path`, as other bridge tests do (e.g. `tests/test_contract_guard.py`).

- [ ] **Step 3: Implement** `route_driver.py` and the console script.

- [ ] **Step 4: Run the tests**

Run: `scripts/pytest-clean tests/test_route_driver.py -q`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add ros2_ws/src/tinker_sim_bridge/tinker_sim_bridge/route_driver.py ros2_ws/src/tinker_sim_bridge/setup.py tests/test_route_driver.py
git commit -m "feat(bridge): odometry route driver for top-lidar runs"
```

### Task 5: Default route past the visible surfaces

**Files:**
- Create: `simulation/routes/rcw2026-tabletops.json`
- Test: `tests/test_top_lidar_route.py`

**Interfaces:**
- Consumes: `load_route` (Task 4), `ray_plane_hits` and `TopMount` (Task 3), `lidar_pattern` and `load_lidar_spec` (`lidar_rig`), the selected rcw2026 arena's `map.pgm`, `map.yaml` and `placement.json` (resolved through `artifacts/arena/rcw2026/current.json`).
- Surfaces the route must cover: kitchen table top, laundry desk top, both side table tops, sofa seat. **Not the shelf plates.** The plate is at 1.06 m with the next board at 1.34 m above it, so a sensor at 1.2 m would have to be within ~0.18 m of the plate horizontally to look down onto it, closer than the robot's front can get. This deviates from the spec's "each of the five" (shelf is one of the five). Tell the user in the hand-off.
- Spawn is `[-2.0, -2.0, 0.0]`, the default scenarios' initial pose and the runbook's `--spawn-xy`.

- [ ] **Step 1: Write the failing tests**
- `test_default_route_loads`.
- `test_default_route_clear_of_map`: sample every segment (spawn → wp1 → … ) every 0.02 m. At each sample, the robot footprint `[[0.15,0.25],[0.15,-0.25],[-0.35,-0.25],[-0.35,0.25]]` (from the tinker2 artifact's `robot-profile.yaml`) inflated by 0.10 m, oriented along the segment (and through the turn at look stops: check headings every 0.1 rad), covers no occupied cell (value 0) of `map.pgm`. Report the first offending sample.
- `test_default_route_sees_each_surface`: for each listed surface, at its look stop (position + yaw), `ray_plane_hits` from the mount (sensor world pose = robot pose ∘ joint transform) on `plane_z = surface z` puts ≥ 100 hits inside the surface rectangle shrunk by `edge_margin`.

- [ ] **Step 2: Run them to see them fail**

Run: `scripts/pytest-clean tests/test_top_lidar_route.py -q`
Expected: FAIL (route file missing).

- [ ] **Step 3: Write the route.** Pick waypoints from the map and the surface positions (kitchen table (2.587, −3.058), side tables (−0.346, −5.045) and (0.43, 1.755), sofa (−2.184, −4.839), laundry desk (−2.988, 4.525)). Each surface gets a look stop that faces it with the robot's front about 0.3–0.4 m from its edge. Iterate until both tests pass.

- [ ] **Step 4: Run the tests**

Run: `scripts/pytest-clean tests/test_top_lidar_route.py -q`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add simulation/routes/rcw2026-tabletops.json tests/test_top_lidar_route.py
git commit -m "feat(top-lidar): default rcw2026 route past the tables and sofa"
```

### Task 6: Top-lidar launch and run script

**Files:**
- Create: `ros2_ws/src/tinker_sim_bridge/launch/top_lidar.launch.py`
- Create: `ros2_ws/src/tinker_sim_bridge/tinker_sim_bridge/top_lidar_nodes.py`
- Modify: `ros2_ws/src/tinker_sim_bridge/setup.py` (install the launch file next to the others, ~l.44-48)
- Create: `scripts/top-lidar`
- Test: `tests/test_top_lidar_launch.py`

**Interfaces:**
- `top_lidar.launch.py` arguments: `project_root` (default `$TINKER_SIM_ROOT`), `tinker_workspace` (default `$TINKER_WS`), `route` (default `simulation/routes/rcw2026-tabletops.json`), `scenario` (default `pick-deliver-place-rcw2026`: no actors, one object on the kitchen table). Nodes:
  1. `base_facade`, `command_gateway`, `safety_supervisor`: copied exactly from `navigation.launch.py:98-114` (same params, same calibration path).
  2. `robot_state_publisher` with `with_base_footprint(<variant robot.urdf text>)`, the variant dir coming from `resolve_variant_artifact(root)`.
  3. Static TF `base_link → livox360` (0.09, 0, 0.195), identity, as `navigation.launch.py` publishes it, because the base Mid-360's IMU still publishes in `livox360`.
  4. Static TF `map → odom` = the route's spawn pose.
  5. `robot_localization` `ekf_node` with `navigation_bringup`'s `params/ekf.yaml` and `use_sim_time`, as `navigation.launch.py:183-189` does (`odom → base_link`).
  6. `truth_evaluator` with the parameters of `manipulation.launch.py:171-180` (`scenario` and `task` = the scenario arg, `jsonl_path` under `reports/top-lidar/`), same `PYTHONPATH` env.
  7. `route_driver` with `route` = the route arg (absolute path).

  It must **not** start AMCL, map servers, Nav2, `pointcloud_to_laserscan` or `initial_pose`. The node list comes from `top_lidar_nodes(root: Path, nav_share: Path, bridge_share: Path, route: Path, scenario: str) -> list[NodeSpec]` in a new launch-free module `ros2_ws/src/tinker_sim_bridge/tinker_sim_bridge/top_lidar_nodes.py`. `NodeSpec` is a frozen dataclass `(package, executable, name, parameters: list, arguments: list, env: dict)`. The launch file's `OpaqueFunction` turns each into a `launch_ros.actions.Node`, so tests inspect `NodeSpec`s without importing `launch`.
- `scripts/top-lidar [--route PATH] [--domain N] [--duration SIM_S] [--force-gpu]`, bash, styled like `scripts/gpsr-stack`:
  1. Refuses to start if `nvidia-smi --query-compute-apps=pid --format=csv,noheader` prints anything (unless `--force-gpu`) or if `--domain` (required, no default) is in use by another process (`grep -l ROS_DOMAIN_ID=<n> /proc/*/environ`).
  2. Reads the route's spawn and requires the variant artifact (`resolve_variant_artifact`).
  3. Starts the sim (env cleaned as in `docs/gpsr-sim-runbook.md` Stage 1):
     ```
     timeout -k 10 <wall> ./scripts/tinker-sim launch --sensor-profile navigation-parity --scenario <scenario> --seed 0 --arena rcw2026 --spawn-xy=<x>,<y> --ros --headless --duration <SIM_S> -- --artifact <variant>/robot.usd --lidar-config simulation/sensors/top-lidar.json
     ```
  4. Waits up to 300 s wall for `/clock`.
  5. Starts `ros2 launch tinker_sim_bridge top_lidar.launch.py` (ROS env sourced as in runbook Stage 2), also under `timeout`.
  6. Exits when `/sim/route/status` reports `done` or `failed`, or at the deadline, and always tears down with `scripts/kill-domain-nodes <domain> --force` via `trap`.
  7. Logs go to `reports/top-lidar/<timestamp>/`.

- [ ] **Step 1: Write the failing tests** against `top_lidar_nodes` (import path set up like `tests/test_contract_guard.py`; pass temp dirs for the shares and a fake variant artifact in a temp repo root):
- `test_nodes_include_route_driver_and_variant_description`: the `robot_state_publisher` description contains `<link name="livox_top"/>`.
- `test_no_nav_nodes`: no `amcl`, `map_server`, `pointcloud_to_laserscan`, `initial_pose`, or Nav2 controller in the list.
- `test_map_to_odom_is_route_spawn`.
- `test_script_requires_domain`: `bash scripts/top-lidar` with no `--domain` exits non-zero with a usage message, without touching the GPU (run it with `PATH` stripped of `nvidia-smi` to prove it's checked after argument parsing).

- [ ] **Step 2: Run them to see them fail**

Run: `scripts/pytest-clean tests/test_top_lidar_launch.py -q`
Expected: FAIL.

- [ ] **Step 3: Implement** the launch file, `setup.py` entry and script (`chmod +x scripts/top-lidar`). Rebuild the bridge package the way `docs/gpsr-sim-runbook.md` does (`colcon build --packages-select tinker_sim_bridge` in `ros2_ws` with ROS sourced) so the launch file and `route_driver` are installed.

- [ ] **Step 4: Run the tests**

Run: `scripts/pytest-clean tests/test_top_lidar_launch.py -q`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add ros2_ws/src/tinker_sim_bridge/launch/top_lidar.launch.py ros2_ws/src/tinker_sim_bridge/tinker_sim_bridge/top_lidar_nodes.py ros2_ws/src/tinker_sim_bridge/setup.py scripts/top-lidar tests/test_top_lidar_launch.py
git commit -m "feat(top-lidar): launch file and run script (no Nav2)"
```

### Task 7: Acceptance check, live

**Files:**
- Create: `validation/top_lidar_acceptance.py`
- Test: `tests/test_top_lidar_acceptance.py`

**Interfaces:**
- Pure functions:
  - `sensor_to_world(points: np.ndarray, mount_xyz, mount_wxyz, base_xyz, base_wxyz) -> np.ndarray`
  - `voxel_downsample(points: np.ndarray, voxel_m: float = 0.02) -> np.ndarray`
  - `surface_hits(points: np.ndarray, surface: Mapping[str, object], tol_m: float = 0.03) -> int`: points inside the rectangle shrunk by `edge_margin` with |z − surface z| ≤ tol.
  - `floor_hits(points: np.ndarray, tol_m: float = 0.03) -> int`: points with |z| ≤ tol.
  - `verdict(points, surfaces, min_surface_points=50, min_floor_points=500) -> dict`: per-surface counts, floor count, overall `passed`.
- `main(argv)`: `--placement PATH --variant-urdf PATH --out DIR [--timeout-s 900]`. Subscribes `/livox/lidar_top`, `/sim/truth/robot_state` (`tinker_sim_interfaces/RobotTruth`, `base_pose`) and `/sim/route/status`. It pairs each cloud with the truth sample nearest its stamp (skipping clouds with no sample within 0.05 s), transforms with the `livox_top_joint` origin read from the variant URDF, accumulates and downsamples. When the route reports `done` (or `failed`, which fails the check), it writes `DIR/verdict.json` and `DIR/topdown.png` (accumulated points coloured by height, surfaces outlined, via Pillow) and exits 0 only if passed. The surfaces are the five covered by Task 5. Shelf plates are listed in the verdict as `"not expected: occluded by the board above"`.
- The floor check is what proves the tilt reached the cloud correctly. If `SENSOR`-frame hits weren't in the rotated frame (Task 1's assumption about `Raycast`), floor points would land at the wrong height and this fails.

- [ ] **Step 1: Write the failing tests**: `test_sensor_to_world_identity`, `test_sensor_to_world_tilted_forward_ray_hits_floor` (a sensor-frame point 1.2/sin45° along +x from mount (0.09, 0, 1.1225) on a robot at (0, 0, 0.0775) yaw 0 lands at z ≈ 0), `test_surface_hits_respects_yaw_and_margin`, `test_verdict_fails_without_floor_points`.

- [ ] **Step 2: Run them to see them fail, implement, run them to pass**

Run: `scripts/pytest-clean tests/test_top_lidar_acceptance.py -q`
Expected: FAIL before, PASS after.

- [ ] **Step 3: Commit**

```bash
git add validation/top_lidar_acceptance.py tests/test_top_lidar_acceptance.py
git commit -m "feat(validation): top-lidar acceptance from sim truth"
```

- [ ] **Step 4: Run it live** (Global Constraints: GPU, domain, timeouts). Shell 1:
```bash
scripts/top-lidar --domain <private> --duration 900
```
Shell 2 (ROS sourced, same domain), started once `/clock` is up:
```bash
timeout -k 10 1200 python3 validation/top_lidar_acceptance.py --placement <selected rcw2026 dir>/placement.json --variant-urdf <variant dir>/robot.urdf --out reports/top-lidar/acceptance-$(date +%F)
```
Expected: the sim log has `live raycast lidar:` with `config=simulation/sensors/top-lidar.json`, `mount=/World/Tinker/.../livox_raycast` and a non-identity `orient=`. The route reports `done`. `verdict.json` passes with every covered surface ≥ 50 points and floor ≥ 500. `topdown.png` shows table outlines.
If a surface has too few points, check the arm: the self-filter drops arm hits, but the arm still blocks rays. Report it with the PNG rather than moving the mount.

- [ ] **Step 5: Normal run still normal (live smoke).** With no new flags, run the runbook's Stage 1 sim for 60 sim seconds on a private domain. Its log shows `config=simulation/sensors/hardware-parity.json`, `orient=(1.0, 0.0, 0.0, 0.0)`, and `ros2 topic list` has `/livox/lidar` and not `/livox/lidar_top`. Tear down.

- [ ] **Step 6: Hand back to the user** the verdict, the PNG path, the `scripts/top-lidar` command, and the shelf deviation from Task 5.
