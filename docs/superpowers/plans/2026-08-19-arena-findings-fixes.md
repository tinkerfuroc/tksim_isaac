# Arena-Findings Fixes Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix the defects surfaced by the 2026-08-18/19 RoboCup arena validation: head effort starvation, obstructed default arena spawn, dev-lidar raycast-floor artifact, unbounded wheel-velocity steps/coasting, unimplemented scenario actor paths, procedural-world scenario poses, and the missing navigation-profile safety-clear path.

**Architecture:** Small fail-closed additions following existing repo patterns: actuator overrides in `IsaacWholeRobotBackend`, pure helpers in `tinker_sim_core` (occupancy search, velocity slew, safety gating, path interpolation) consumed by thin ROS nodes, two new scenario JSONs, and parameterized supervisor gating. No command-protocol rework; no changes to validated manipulation defaults.

**Tech Stack:** Python 3.10 (system) for unit tests; Isaac Sim / Isaac Lab (project venv) for live runs; ROS 2 Humble overlay (`ros2_ws`, rebuilt via `./scripts/build-humble-overlay`).

**Spec:** `docs/superpowers/specs/2026-08-19-arena-findings-fixes-design.md` (written and committed in Task 1; content inlined there).

## Global Constraints

- Evidence policy: no behavior claimed validated without a live run; evidence saved under `reports/` (gitignored by convention).
- All unit tests must pass under system python: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest <file> -q`.
- Default behavior of manipulation/qualification paths must not change (`manage_controllers` defaults True; head/arm/wheel changes are sim-side actuator config only).
- The streaming viewer currently running on TCP 49100 must not be killed (pending human textured-frame confirmation).
- Untracked teammate files other than the two test files being landed (`tests/test_base_velocity_slew.py`, `tests/test_chassis_ballast.py`) stay untracked.
- Live runs use ROS_DOMAIN_ID=42; launch env pattern: `set -a; source .deployment.env; set +a; unset PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH LD_LIBRARY_PATH`.
- Commit messages end with `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`.
- Arena artifact for poses/maps: `artifacts/arena/rcw2026/d2b559b43207c8d54ae2609f638dca1cc36ee8b7adc7e4d94aee86e7fb56729c/`.

---

### Task 1: Spec + plan docs

**Files:**
- Create: `docs/superpowers/specs/2026-08-19-arena-findings-fixes-design.md`
- Create: `docs/superpowers/plans/2026-08-19-arena-findings-fixes.md`

**Interfaces:** Produces the committed spec that later tasks cite.

- [ ] **Step 1: Write the spec doc** — condensed from the approved design sections (A: head effort 10 Nm override, fail-closed spawn with nearest-free-cell suggestion at 0.35 m clearance, empty-cloud dev-lidar fix for occupied ray origin; B: per-physics-tick wheel slew at 60 rad/s² with persistent applied-velocity state, safety-stop reset + idempotence guard, teammate TDD tests landed, protocol holes documented not reworked, slew constant deliberately above Nav2 acc_lim so planner profiles pass through; C: one-shot `actor_path_driver` node via `/set_entity_state` at 0.3 m/s default, tightened event validation, `find-and-approach-person-rcw2026` + `pick-deliver-place-rcw2026` scenarios with map-verified poses; D: supervisor `manage_controllers`/`required_sources` params, unmanaged mode = sources-only fail-closed clear, navigation launch runs it with `["collision"]`). Include the corrected findings (no RTX lidar exists; 0.3 m ring is the occupancy raycast floor; head 1.0 Nm is a placeholder; EKF+IMU yaw fusion rejected because the sim IMU publishes only world-frame angular velocity with orientation marked invalid). Record the user decisions (fail-closed + suggested cell; implement+wire+commit slew; all scope groups; tk25_ws branch left as-is).
- [ ] **Step 2: Copy this plan file** to `docs/superpowers/plans/2026-08-19-arena-findings-fixes.md`.
- [ ] **Step 3: Commit**

```bash
git add docs/superpowers/specs/2026-08-19-arena-findings-fixes-design.md docs/superpowers/plans/2026-08-19-arena-findings-fixes.md
git commit -m "docs: spec + plan for arena-findings fixes"
```

---

### Task 2: OccupancyMap clearance/nearest-free helpers

**Files:**
- Modify: `simulation/tinker_sim_core/occupancy.py` (append two methods to `OccupancyMap`)
- Test: `tests/test_occupancy.py` (append)

**Interfaces:**
- Produces: `OccupancyMap.free_with_clearance(x: float, y: float, clearance_m: float) -> bool` and `OccupancyMap.nearest_free_world(x: float, y: float, clearance_m: float, max_radius_m: float = 5.0) -> tuple[float, float] | None`. Task 3 consumes both.

- [ ] **Step 1: Write failing tests** (append to `tests/test_occupancy.py`, matching its existing construction style — build `OccupancyMap` directly with a small `occupied` grid):

```python
def _grid_map(rows):
    # rows: list of strings, '#' occupied, '.' free; row 0 = world-min y
    occupied = tuple(tuple(ch == "#" for ch in row) for row in rows)
    return OccupancyMap(
        width=len(rows[0]), height=len(rows), resolution=0.1,
        origin_x=0.0, origin_y=0.0, occupied=occupied,
    )


class OccupancyClearanceTest(unittest.TestCase):
    def test_free_with_clearance_true_in_open_space(self):
        grid = _grid_map(["........", "........", "........", "........"])
        self.assertTrue(grid.free_with_clearance(0.4, 0.2, 0.1))

    def test_free_with_clearance_false_near_obstacle(self):
        grid = _grid_map(["........", "...##...", "...##...", "........"])
        self.assertFalse(grid.free_with_clearance(0.4, 0.2, 0.15))

    def test_free_with_clearance_false_out_of_bounds(self):
        grid = _grid_map(["....", "...."])
        self.assertFalse(grid.free_with_clearance(-1.0, 0.0, 0.1))

    def test_nearest_free_world_finds_adjacent_cell(self):
        grid = _grid_map(["........", "..####..", "..####..", "........"])
        found = grid.nearest_free_world(0.4, 0.2, 0.1)
        self.assertIsNotNone(found)
        fx, fy = found
        self.assertTrue(grid.free_with_clearance(fx, fy, 0.1))

    def test_nearest_free_world_none_when_all_occupied(self):
        grid = _grid_map(["####", "####"])
        self.assertIsNone(grid.nearest_free_world(0.2, 0.1, 0.1, max_radius_m=0.5))

    def test_nearest_free_world_returns_input_when_already_clear(self):
        grid = _grid_map(["........", "........", "........", "........"])
        self.assertEqual(grid.nearest_free_world(0.4, 0.2, 0.1), (0.4, 0.2))
```

- [ ] **Step 2: Run** `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_occupancy.py -q` — expect the new tests FAIL (AttributeError).
- [ ] **Step 3: Implement** (append methods to `OccupancyMap`; module already avoids top-level math — keep the local-import style used by `raycast`):

```python
    def free_with_clearance(self, x: float, y: float, clearance_m: float) -> bool:
        if clearance_m < 0.0:
            raise ValueError("clearance must be non-negative")
        step = self.resolution
        steps = int(clearance_m / step) + 1
        for gx in range(-steps, steps + 1):
            for gy in range(-steps, steps + 1):
                dx = gx * step
                dy = gy * step
                if dx * dx + dy * dy > clearance_m * clearance_m:
                    continue
                if self.occupied_at_world(x + dx, y + dy):
                    return False
        return True

    def nearest_free_world(
        self, x: float, y: float, clearance_m: float, max_radius_m: float = 5.0
    ) -> "tuple[float, float] | None":
        if self.free_with_clearance(x, y, clearance_m):
            return (x, y)
        step = self.resolution
        ring = 1
        while ring * step <= max_radius_m:
            candidates = []
            for gx in range(-ring, ring + 1):
                for gy in range(-ring, ring + 1):
                    if max(abs(gx), abs(gy)) != ring:
                        continue
                    cx = x + gx * step
                    cy = y + gy * step
                    if self.free_with_clearance(cx, cy, clearance_m):
                        candidates.append((gx * gx + gy * gy, cx, cy))
            if candidates:
                _, cx, cy = min(candidates)
                return (round(cx, 3), round(cy, 3))
            ring += 1
        return None
```

- [ ] **Step 4: Run** the same pytest command — expect PASS (whole file).
- [ ] **Step 5: Commit** `git add simulation/tinker_sim_core/occupancy.py tests/test_occupancy.py && git commit -m "feat: occupancy clearance check and nearest-free-cell search"`

---

### Task 3: Fail-closed arena spawn validation in run_sim

**Files:**
- Modify: `validation/run_sim.py` (new helper + calls in the three robot-backend branches at ~582, ~775/783, ~912/919 — right after each `resolve_arena_artifact` call)
- Test: `tests/test_spawn_override.py` (append)

**Interfaces:**
- Consumes: `OccupancyMap.free_with_clearance`, `nearest_free_world` (Task 2); existing `backend._map_metadata(map_yaml)` (`simulation/tinker_sim_isaac/backend.py:~55-64`, returns `(pgm_path, resolution, origin_x, origin_y)`).
- Produces: `validate_arena_spawn(arena_dir: Path, spawn_xy: tuple[float, float]) -> None` in `run_sim.py` (module level, raises `RuntimeError` with the suggestion text). `SPAWN_CLEARANCE_M = 0.35` constant.

- [ ] **Step 1: Write failing tests** (append to `tests/test_spawn_override.py`; build a tiny arena dir fixture with a synthetic P5 PGM):

```python
import struct

def _write_arena_map(directory: Path, rows: list[str]) -> None:
    # '#' -> occupied (0), '.' -> free (254); row 0 = TOP of image (world max y)
    width, height = len(rows[0]), len(rows)
    header = f"P5\n{width} {height}\n255\n".encode()
    payload = bytes(0 if ch == "#" else 254 for row in rows for ch in row)
    (directory / "map.pgm").write_bytes(header + payload)
    (directory / "map.yaml").write_text(
        "image: map.pgm\nresolution: 0.1\norigin: [0.0, 0.0, 0]\n"
    )

class ValidateArenaSpawnTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.arena = Path(self.temporary.name)

    def test_clear_spawn_passes(self):
        _write_arena_map(self.arena, ["." * 20] * 20)
        from run_sim import validate_arena_spawn
        validate_arena_spawn(self.arena, (1.0, 1.0))  # no raise

    def test_occupied_spawn_fails_with_suggestion(self):
        rows = ["." * 20 for _ in range(20)]
        for r in range(8, 12):
            rows[r] = rows[r][:8] + "####" + rows[r][12:]
        _write_arena_map(self.arena, rows)
        from run_sim import validate_arena_spawn
        with self.assertRaisesRegex(RuntimeError, r"--spawn-xy="):
            validate_arena_spawn(self.arena, (1.0, 1.0))

    def test_fully_occupied_map_fails_without_suggestion_crash(self):
        _write_arena_map(self.arena, ["#" * 6] * 6)
        from run_sim import validate_arena_spawn
        with self.assertRaisesRegex(RuntimeError, "no free cell"):
            validate_arena_spawn(self.arena, (0.3, 0.3))
```

- [ ] **Step 2: Run** `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_spawn_override.py -q` — new tests FAIL (ImportError).
- [ ] **Step 3: Implement** in `validation/run_sim.py` (module level, near `parse_spawn_xy`):

```python
SPAWN_CLEARANCE_M = 0.35  # robot inscribed radius 0.25 m + margin


def validate_arena_spawn(arena_dir: Path, spawn_xy: tuple[float, float]) -> None:
    """Fail closed when the robot spawn lacks clearance on the arena map.

    The rcw2026 default spawn (0, 0) sits inside shelf_02's rasterized
    footprint; a spawn inside furniture corrupts odometry, lidar, and AMCL.
    The error names the nearest free cell so the user can retry.
    """
    sys.path.insert(0, str((arena_dir / "../../../..").resolve() / "simulation"))
    from tinker_sim_core.occupancy import OccupancyMap
    from tinker_sim_isaac.backend import _map_metadata

    pgm, resolution, origin_x, origin_y = _map_metadata(arena_dir / "map.yaml")
    occupancy = OccupancyMap.from_pgm(
        pgm, resolution=resolution, origin_x=origin_x, origin_y=origin_y
    )
    x, y = spawn_xy
    if occupancy.free_with_clearance(x, y, SPAWN_CLEARANCE_M):
        return
    suggestion = occupancy.nearest_free_world(x, y, SPAWN_CLEARANCE_M)
    if suggestion is None:
        raise RuntimeError(
            f"arena spawn ({x}, {y}) is obstructed and no free cell was found "
            f"within 5 m on the derived map"
        )
    raise RuntimeError(
        f"arena spawn ({x}, {y}) lacks {SPAWN_CLEARANCE_M} m clearance on the "
        f"derived map; try --spawn-xy={suggestion[0]},{suggestion[1]}"
    )
```

Note: `_map_metadata` lives in `tinker_sim_isaac.backend` (line ~55); confirm the exact name/signature when editing and use the same import path the backend uses internally. Then call the helper in all three robot-backend branches immediately after `arena_dir = resolve_arena_artifact(root, args.arena)`:

```python
            if arena_dir is not None:
                validate_arena_spawn(arena_dir, spawn_xy)
```

(The `spawn_xy` variable already exists from the `--spawn-xy` parsing added on 2026-08-18.)
- [ ] **Step 4: Run** `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_spawn_override.py tests/test_deploy_cli_launch.py tests/test_arena_streaming.py -q` — PASS.
- [ ] **Step 5: Commit** `git add validation/run_sim.py tests/test_spawn_override.py && git commit -m "feat: fail-closed arena spawn clearance check with nearest-free suggestion"`

---

### Task 4: Dev-lidar empty cloud from occupied origin

**Files:**
- Modify: `simulation/tinker_sim_isaac/ros_gateway.py:_development_point_cloud` (~1066)
- Test: `tests/test_ros_gateway.py` (append, reusing its `_cloud_gateway`/fake-backend harness at ~412-461)

**Interfaces:** none new; behavior change only (empty PointCloud2 when the ray origin cell is occupied).

- [ ] **Step 1: Write failing test** (append; follow the file's existing fake-backend pattern — the fake backend's `occupancy` object needs `occupied_at_world` and `raycast`):

```python
    def test_development_lidar_empty_when_origin_occupied(self):
        """A sensor origin inside an occupied cell has no valid returns; the
        cloud must be empty rather than a fake ring at the raycast floor."""
        class _OccupiedEverywhere:
            def occupied_at_world(self, x, y):
                return True
            def raycast(self, x, y, angle, minimum=0.3, maximum=40.0):
                return minimum
        gateway = _cloud_gateway(
            development_lidar=True, occupancy=_OccupiedEverywhere()
        )
        message = gateway._development_point_cloud(_stamp())
        self.assertEqual(message.width, 0)
```

(Adapt `_stamp()` to however existing tests in the file build the stamp argument.)
- [ ] **Step 2: Run** `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_ros_gateway.py -q` — new test FAILS (width == 181).
- [ ] **Step 3: Implement** — in `_development_point_cloud`, after computing `yaw` and `occupancy`, compute the sensor origin once and guard the loop:

```python
        origin_x = x + 0.12 * math.cos(yaw)
        origin_y = y + 0.12 * math.sin(yaw)
        origin_occupied = (
            occupancy is not None and occupancy.occupied_at_world(origin_x, origin_y)
        )
        points = []
        if not origin_occupied:
            for degrees in range(-90, 91):
                local = math.radians(degrees)
                if occupancy is not None:
                    distance = occupancy.raycast(origin_x, origin_y, yaw + local)
                else:
                    distance = _FALLBACK_LIDAR_RANGE_M
                if math.isfinite(distance):
                    points.append(
                        (distance * math.cos(local), distance * math.sin(local), 0.0)
                    )
```

(The existing no-occupancy fallback ring is preserved: `origin_occupied` is False when `occupancy is None`.)
- [ ] **Step 4: Run** the whole file — PASS, including the existing `test_development_lidar_publishes_without_occupancy`.
- [ ] **Step 5: Commit** `git add simulation/tinker_sim_isaac/ros_gateway.py tests/test_ros_gateway.py && git commit -m "fix: dev lidar publishes empty cloud from an occupied origin instead of a raycast-floor ring"`

---

### Task 5: Head pan/tilt effort override

**Files:**
- Modify: `simulation/tinker_sim_isaac/backend.py` (~292, `"head"` actuator group)

**Interfaces:** none; sim actuator config only.

- [ ] **Step 1: Edit the actuator group** (URDF's 1.0 Nm is a placeholder — massless stub links, head absent from ros2_control; this mirrors the existing `"arm"`/`"wheels"` overrides):

```python
                "head": ImplicitActuatorCfg(
                    joint_names_expr=["pan_joint", "tilt_joint"],
                    stiffness=500.0,
                    damping=50.0,
                    # The URDF's effort="1.0" is a hand-authored placeholder
                    # (massless stub links, no ros2_control entry); 1 Nm cannot
                    # move the head against default-assigned inertias.
                    effort_limit_sim=10.0,
                ),
```

- [ ] **Step 2: Sanity suites** `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_backend_arena_inputs.py tests/test_spawn_override.py -q` — PASS (config change has no unit-visible surface; live proof in Task 12).
- [ ] **Step 3: Commit** `git add simulation/tinker_sim_isaac/backend.py && git commit -m "fix: give the head actuator a usable effort limit (URDF 1 Nm placeholder)"`

---

### Task 6: slew_velocity_target + set_safety_stop idempotence (teammate TDD)

**Files:**
- Modify: `simulation/tinker_sim_isaac/backend.py`
- Test: `tests/test_base_velocity_slew.py` (EXISTS untracked — teammate's spec; do not edit it), `tests/test_chassis_ballast.py` (exists untracked, already passes)

**Interfaces:**
- Produces: module-level `slew_velocity_target(current: float, target: float, max_delta: float) -> float` in `tinker_sim_isaac.backend`. Task 7 consumes it.

- [ ] **Step 1: Run their test to see it fail** `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_base_velocity_slew.py -q` — ImportError.
- [ ] **Step 2: Implement the pure function** (module level, next to `chassis_ballast_target_properties`, same fail-closed style):

```python
def slew_velocity_target(current: float, target: float, max_delta: float) -> float:
    """Move a velocity target toward ``target`` by at most ``max_delta``.

    Wheel targets were previously applied verbatim (README: upstream owns
    acceleration limits — nothing upstream did). This bounds every wheel
    transient, including stale-held-target windows, without overshooting.
    """
    values = (current, target, max_delta)
    if any(isinstance(v, bool) or not math.isfinite(float(v)) for v in values):
        raise ValueError("slew inputs must be finite numbers")
    if max_delta < 0.0:
        raise ValueError("max_delta must be non-negative")
    delta = target - current
    if abs(delta) <= max_delta:
        return float(target)
    return float(current + math.copysign(max_delta, delta))
```

- [ ] **Step 3: Add the idempotence guard** at the very top of `set_safety_stop` (~452), before any other attribute access (their fourth test constructs the backend via `object.__new__` with only `_safety_stopped` set):

```python
    def set_safety_stop(self, active: bool) -> None:
        if bool(active) == self._safety_stopped:
            # A repeated identical sample must return before it clears the
            # acceleration-limited wheel state (see tests/test_base_velocity_slew.py).
            return
```

(Keep the existing body after the guard unchanged.)
- [ ] **Step 4: Run** `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_base_velocity_slew.py tests/test_chassis_ballast.py -q` — PASS.
- [ ] **Step 5: Land the teammate tests + implementation** `git add simulation/tinker_sim_isaac/backend.py tests/test_base_velocity_slew.py tests/test_chassis_ballast.py && git commit -m "feat: implement slew_velocity_target and safety-stop idempotence, land task50 TDD tests"`

---

### Task 7: Wire per-tick wheel slew in step()

**Files:**
- Modify: `simulation/tinker_sim_isaac/backend.py` (constants near the ballast constants; state init near `_velocity_targets` at ~401; `step()` at ~804; safety-stop activation path)

**Interfaces:**
- Consumes: `slew_velocity_target` (Task 6); `self._joint_index` (name→index, ~382); `self.dt`.
- Produces: constants `WHEEL_VELOCITY_SLEW_RAD_S2 = 60.0`, `WHEEL_JOINT_NAMES = frozenset({"front_left_wheel_joint", "front_right_wheel_joint", "rear_left_wheel_joint", "rear_right_wheel_joint"})`.

- [ ] **Step 1: Add constants** (module level):

```python
# Wheel radius 0.0525 m => 60 rad/s^2 ~= 3.1 m/s^2 linear. Deliberately above
# Nav2's acc_lim (~2.5 m/s^2) so planner-shaped profiles pass through; this is
# the floor-level bound for non-planner commanders and stale-target transients.
WHEEL_VELOCITY_SLEW_RAD_S2 = 60.0
WHEEL_JOINT_NAMES = frozenset(
    {
        "front_left_wheel_joint",
        "front_right_wheel_joint",
        "rear_left_wheel_joint",
        "rear_right_wheel_joint",
    }
)
```

- [ ] **Step 2: Init state** — where joint handles resolve (right after `self._joint_index` is built at ~382):

```python
        self._wheel_indices = tuple(
            self._joint_index[name]
            for name in sorted(WHEEL_JOINT_NAMES)
            if name in self._joint_index
        )
        self._applied_wheel_velocities = {index: 0.0 for index in self._wheel_indices}
```

- [ ] **Step 3: Slew in the non-safety `step()` branch** — replace the `else:` body (~825-827):

```python
        else:
            max_delta = WHEEL_VELOCITY_SLEW_RAD_S2 * self.dt
            for index in self._wheel_indices:
                commanded = float(self._velocity_targets[0, index])
                applied = slew_velocity_target(
                    self._applied_wheel_velocities[index], commanded, max_delta
                )
                self._applied_wheel_velocities[index] = applied
                self._velocity_targets[0, index] = applied
            self._robot.set_joint_velocity_target(self._velocity_targets)
            self._robot.set_joint_position_target(self._position_targets)
```

Note: writing the slewed value back into `_velocity_targets` is safe — the next gateway snapshot re-asserts the raw commanded value into the buffer before the next `step()`, and when snapshots stop arriving the buffer holds the applied (ramping-down-only-via-new-zeros) value; `_applied_wheel_velocities` is the authoritative ramp state either way.
- [ ] **Step 4: Reset on safety-stop activation** — inside the existing `set_safety_stop` body, in the `active=True` branch (after the Task 6 guard):

```python
            self._applied_wheel_velocities = {
                index: 0.0 for index in getattr(self, "_wheel_indices", ())
            }
```

- [ ] **Step 5: Run suites** `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_base_velocity_slew.py tests/test_backend_arena_inputs.py tests/test_arena_streaming.py -q` — PASS. (The wiring itself is proven live in Task 12's coast run.)
- [ ] **Step 6: Commit** `git add simulation/tinker_sim_isaac/backend.py && git commit -m "feat: per-tick wheel velocity slew bounds acceleration and stale-target transients"`

---

### Task 8: Scenario event validation + actor path helper

**Files:**
- Modify: `simulation/tinker_sim_core/scenario.py` (~89-91, event checks)
- Create: `simulation/tinker_sim_core/actor_path.py`
- Test: Create `tests/test_actor_paths.py`

**Interfaces:**
- Produces: `path_pose_at(path: Sequence[Sequence[float]], distance: float) -> tuple[float, float, float]` (x, y, yaw along a polyline, clamped to the end) and `path_length(path) -> float` in `tinker_sim_core.actor_path`. Task 9 consumes both. Scenario validation: `actor_path_start` events must name an existing actor whose `path` is a list of >=2 [x, y] pairs of finite floats.

- [ ] **Step 1: Write failing tests** (`tests/test_actor_paths.py`, header pattern from `tests/test_spawn_override.py`):

```python
from __future__ import annotations

import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))

from tinker_sim_core.actor_path import path_length, path_pose_at
from tinker_sim_core.scenario import ScenarioDefinition


class PathPoseTest(unittest.TestCase):
    PATH = [[0.0, 0.0], [2.0, 0.0], [2.0, 1.0]]

    def test_length(self):
        self.assertAlmostEqual(path_length(self.PATH), 3.0)

    def test_pose_on_first_segment(self):
        x, y, yaw = path_pose_at(self.PATH, 1.0)
        self.assertAlmostEqual(x, 1.0)
        self.assertAlmostEqual(y, 0.0)
        self.assertAlmostEqual(yaw, 0.0)

    def test_pose_on_second_segment(self):
        x, y, yaw = path_pose_at(self.PATH, 2.5)
        self.assertAlmostEqual(x, 2.0)
        self.assertAlmostEqual(y, 0.5)
        self.assertAlmostEqual(yaw, math.pi / 2)

    def test_pose_clamps_to_end(self):
        x, y, yaw = path_pose_at(self.PATH, 99.0)
        self.assertAlmostEqual(x, 2.0)
        self.assertAlmostEqual(y, 1.0)

    def test_rejects_short_or_nonfinite_paths(self):
        with self.assertRaises(ValueError):
            path_pose_at([[0.0, 0.0]], 0.0)
        with self assertRaises(ValueError):
            path_pose_at([[0.0, 0.0], [float("nan"), 1.0]], 0.0)


def _scenario_raw(events, actors):
    return {
        "schema_version": 2, "id": "x", "world": {"mode": "current"},
        "robot": {"id": "tinker2", "initial_pose": [0, 0, 0]},
        "actors": actors, "objects": [], "regions": [], "events": events,
        "dialogue": [], "postconditions": [{"name": "n", "path": "p", "operator": "equals", "value": True}],
    }


class EventValidationTest(unittest.TestCase):
    def _load(self, raw):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / "x.json"
            p.write_text(json.dumps(raw))
            return ScenarioDefinition.load(p)

    def test_actor_path_start_requires_known_actor(self):
        raw = _scenario_raw(
            [{"at_sim_time": 0.0, "type": "actor_path_start", "actor": "ghost"}], []
        )
        with self.assertRaisesRegex(ValueError, "unknown actor"):
            self._load(raw)

    def test_actor_path_start_requires_valid_path(self):
        actor = {"id": "a", "asset_uri": "x.usda", "path": [[0.0, 0.0]]}
        raw = _scenario_raw(
            [{"at_sim_time": 0.0, "type": "actor_path_start", "actor": "a"}], [actor]
        )
        with self.assertRaisesRegex(ValueError, "path"):
            self._load(raw)

    def test_valid_actor_path_event_loads(self):
        actor = {"id": "a", "asset_uri": "x.usda", "path": [[0.0, 0.0], [1.0, 0.0]]}
        raw = _scenario_raw(
            [{"at_sim_time": 0.0, "type": "actor_path_start", "actor": "a"}], [actor]
        )
        self._load(raw)  # no raise
```

(Fix the deliberate `with self assertRaises` typo to `self.assertRaises` when writing the file. Check `ScenarioDefinition.load`'s actual signature/entry point in `scenario.py` — if loading goes through a module function rather than a classmethod, use that; the existing `test_orchestration.py` shows the canonical loading call.)
- [ ] **Step 2: Run** — FAIL (no module `actor_path`; no event validation).
- [ ] **Step 3: Implement** `simulation/tinker_sim_core/actor_path.py`:

```python
from __future__ import annotations

import math
from typing import Sequence


def _validated(path: Sequence[Sequence[float]]) -> list[tuple[float, float]]:
    if len(path) < 2:
        raise ValueError("actor path requires at least two waypoints")
    points = []
    for waypoint in path:
        if len(waypoint) != 2:
            raise ValueError("actor path waypoints must be [x, y] pairs")
        x, y = float(waypoint[0]), float(waypoint[1])
        if not (math.isfinite(x) and math.isfinite(y)):
            raise ValueError("actor path waypoints must be finite")
        points.append((x, y))
    return points


def path_length(path: Sequence[Sequence[float]]) -> float:
    points = _validated(path)
    return sum(
        math.dist(points[i], points[i + 1]) for i in range(len(points) - 1)
    )


def path_pose_at(path: Sequence[Sequence[float]], distance: float) -> tuple[float, float, float]:
    """(x, y, yaw) at ``distance`` metres along the polyline, clamped to its end."""
    points = _validated(path)
    if not math.isfinite(distance) or distance < 0.0:
        distance = max(0.0, float(distance)) if math.isfinite(distance) else 0.0
    remaining = distance
    for start, end in zip(points, points[1:]):
        segment = math.dist(start, end)
        yaw = math.atan2(end[1] - start[1], end[0] - start[0])
        if remaining <= segment or (start, end) == (points[-2], points[-1]):
            fraction = min(1.0, remaining / segment) if segment > 0.0 else 1.0
            return (
                start[0] + (end[0] - start[0]) * fraction,
                start[1] + (end[1] - start[1]) * fraction,
                yaw,
            )
        remaining -= segment
    return (*points[-1], 0.0)
```

And in `scenario.py`, extend the existing event check (~89-91) after the `at_sim_time`/`trigger` requirement:

```python
        actor_ids = {str(record.get("id")) for record in actors}
        for event in events:
            if event.get("type") == "actor_path_start":
                actor_id = event.get("actor")
                if actor_id not in actor_ids:
                    raise ValueError(f"event names unknown actor: {actor_id!r}")
                actor = next(r for r in actors if str(r.get("id")) == actor_id)
                from .actor_path import _validated as _validated_path
                _validated_path(actor.get("path", ()))
```

(Place inside `ScenarioDefinition.load` where `actors`/`events` locals exist; adapt names to the real locals. If importing the private `_validated` reads poorly, export it as `validate_path` from `actor_path.py` and use that name in both places.)
- [ ] **Step 4: Run** `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_actor_paths.py tests/test_orchestration.py -q` — PASS (the existing `find-and-approach-person.json` already satisfies the tightened rule).
- [ ] **Step 5: Commit** `git add simulation/tinker_sim_core/actor_path.py simulation/tinker_sim_core/scenario.py tests/test_actor_paths.py && git commit -m "feat: actor path interpolation helper and fail-closed actor_path_start validation"`

---

### Task 9: actor_path_driver node

**Files:**
- Create: `ros2_ws/src/tinker_sim_bridge/tinker_sim_bridge/actor_path_driver.py`
- Modify: `ros2_ws/src/tinker_sim_bridge/setup.py` (console_scripts, after `scenario_runner`)

**Interfaces:**
- Consumes: `tinker_sim_core.scenario.load_named_scenario`, `tinker_sim_core.actor_path.path_length/path_pose_at`, `/clock`, `/set_entity_state` (simulation_interfaces).
- Produces: console script `actor_path_driver` with args `--root PATH --scenario NAME [--speed-mps 0.3] [--rate-hz 10.0] [--timeout 120.0]`. Exits 0 after one full traversal of every actor path; exits 1 on service/timeouts.

- [ ] **Step 1: Write the node** (mirror `scenario_runner.py`'s arg/env conventions — `--root` + PYTHONPATH-side import of `tinker_sim_core`):

```python
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from simulation_interfaces.srv import SetEntityState


class ActorPathDriver(Node):
    """Drive scenario actors along their declared paths via /set_entity_state.

    ScenarioRunner is one-shot by contract (launch gates key off its exit),
    so path execution lives in this separate, also one-shot, node.
    """

    def __init__(self, timeout_s: float) -> None:
        super().__init__("tinker_sim_actor_path_driver")
        self._sim_time: float | None = None
        self.create_subscription(Clock, "/clock", self._clock, 10)
        self._client = self.create_client(SetEntityState, "/set_entity_state")
        if not self._client.wait_for_service(timeout_sec=timeout_s):
            raise RuntimeError("/set_entity_state is not available")

    def _clock(self, message: Clock) -> None:
        self._sim_time = message.clock.sec + message.clock.nanosec * 1e-9

    def wait_sim_time(self, at_least: float, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._sim_time is not None and self._sim_time >= at_least:
                return True
        return False

    def place(self, prim_path: str, x: float, y: float, yaw: float, timeout_s: float) -> bool:
        request = SetEntityState.Request()
        request.entity = prim_path
        request.state.header.frame_id = "world"
        request.state.pose.position.x = x
        request.state.pose.position.y = y
        request.state.pose.orientation.z = math.sin(yaw / 2.0)
        request.state.pose.orientation.w = math.cos(yaw / 2.0)
        future = self._client.call_async(request)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        return future.done() and future.result() is not None


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--speed-mps", type=float, default=0.3)
    parser.add_argument("--rate-hz", type=float, default=10.0)
    parser.add_argument("--timeout", type=float, default=120.0)
    arguments = parser.parse_args(argv)
    if arguments.speed_mps <= 0.0 or arguments.rate_hz <= 0.0:
        raise SystemExit("--speed-mps and --rate-hz must be positive")

    sys.path.insert(0, str(arguments.root / "simulation"))
    from tinker_sim_core.actor_path import path_length, path_pose_at
    from tinker_sim_core.scenario import load_named_scenario

    scenario = load_named_scenario(arguments.root, arguments.scenario)
    actor_records = {str(record["id"]): record for record in scenario.actors}
    walks = []
    for event in scenario.events:
        if event.get("type") != "actor_path_start":
            continue
        record = actor_records[str(event["actor"])]
        walks.append(
            (
                float(event.get("at_sim_time", 0.0)),
                f"/World/Scenario/{record['id']}",
                list(record["path"]),
            )
        )
    if not walks:
        print("no actor_path_start events; nothing to drive")
        return

    rclpy.init()
    node = ActorPathDriver(timeout_s=arguments.timeout)
    try:
        for at_sim_time, prim_path, path in sorted(walks):
            if not node.wait_sim_time(at_sim_time, arguments.timeout):
                raise SystemExit(f"sim time {at_sim_time} not reached")
            total = path_length(path)
            start = time.monotonic()
            period = 1.0 / arguments.rate_hz
            while True:
                distance = min(total, (time.monotonic() - start) * arguments.speed_mps)
                x, y, yaw = path_pose_at(path, distance)
                if not node.place(prim_path, x, y, yaw, arguments.timeout):
                    raise SystemExit(f"set_entity_state failed for {prim_path}")
                if distance >= total:
                    break
                time.sleep(period)
            print(f"actor {prim_path} completed {total:.2f} m path")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Register the console script** in `setup.py` after the `scenario_runner` line:

```python
            "actor_path_driver = tinker_sim_bridge.actor_path_driver:main",
```

- [ ] **Step 3: Rebuild the overlay** `./scripts/build-humble-overlay` — expect a clean colcon build; `ros2_ws/install/tinker_sim_bridge/lib/tinker_sim_bridge/actor_path_driver` exists afterward.
- [ ] **Step 4: Commit** `git add ros2_ws/src/tinker_sim_bridge/tinker_sim_bridge/actor_path_driver.py ros2_ws/src/tinker_sim_bridge/setup.py && git commit -m "feat: actor_path_driver node executes scenario actor paths via set_entity_state"`

---

### Task 10: Arena scenario variants

**Files:**
- Create: `simulation/scenarios/find-and-approach-person-rcw2026.json`
- Create: `simulation/scenarios/pick-deliver-place-rcw2026.json`
- Test: append an arena-pose test to `tests/test_actor_paths.py`

**Interfaces:** scenario ids `find-and-approach-person-rcw2026`, `pick-deliver-place-rcw2026` (must equal filename stems; must NOT join `QUALIFICATION_SCENARIO_NAMES`). `tests/test_orchestration.py` globs and auto-validates compilation.

- [ ] **Step 1: Verify candidate poses against the real derived map** (read-only script; adjust coordinates below if any check fails):

```bash
python3 - <<'EOF'
import sys
sys.path.insert(0, "simulation")
from pathlib import Path
from tinker_sim_core.occupancy import OccupancyMap
art = Path("artifacts/arena/rcw2026/d2b559b43207c8d54ae2609f638dca1cc36ee8b7adc7e4d94aee86e7fb56729c")
occ = OccupancyMap.from_pgm(art/"map.pgm", resolution=0.05, origin_x=-5.05, origin_y=-6.0)
import math
def path_clear(a, b, clearance=0.4):
    steps = int(math.dist(a, b) / 0.05) + 1
    return all(occ.free_with_clearance(a[0]+(b[0]-a[0])*i/steps, a[1]+(b[1]-a[1])*i/steps, clearance) for i in range(steps+1))
print("robot spawn (-2,-2):", occ.free_with_clearance(-2.0, -2.0, 0.35))
print("person spawn (-2,-3.5):", occ.free_with_clearance(-2.0, -3.5, 0.4))
print("person path (-2,-3.5)->(-1,-3.0):", path_clear((-2.0,-3.5), (-1.0,-3.0)))
EOF
```

- [ ] **Step 2: Write the person scenario** (`find-and-approach-person-rcw2026.json`; structure copied from `find-and-approach-person.json`, only world/poses/path change):

```json
{
    "schema_version": 2,
    "id": "find-and-approach-person-rcw2026",
    "world": {"mode": "arena", "arena": "rcw2026"},
    "robot": {"id": "tinker2", "initial_pose": [-2.0, -2.0, 0.0]},
    "actors": [
        {
            "id": "target_person",
            "asset_uri": "simulation/assets/primitives/person.usda",
            "pose": {"xyz": [-2.0, -3.5, 0.0], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]},
            "path": [[-2.0, -3.5], [-1.0, -3.0]]
        }
    ],
    "objects": [],
    "regions": [],
    "events": [
        {"at_sim_time": 0.0, "type": "actor_path_start", "actor": "target_person"}
    ],
    "dialogue": [],
    "postconditions": "COPY VERBATIM from find-and-approach-person.json"
}
```

(Replace the postconditions placeholder with the original file's exact array.)
- [ ] **Step 3: Write the object scenario** (`pick-deliver-place-rcw2026.json`; object on `kitchen_table#top`, center [2.587, -3.058, 0.724], task cube is 0.08 m with origin at its base, so z = 0.724 + 0.01 settle margin; delivery region above `side_table_02#top` [0.43, 1.755, 0.602]):

```json
{
    "schema_version": 2,
    "id": "pick-deliver-place-rcw2026",
    "world": {"mode": "arena", "arena": "rcw2026"},
    "robot": {"id": "tinker2", "initial_pose": [-2.0, -2.0, 0.0]},
    "actors": [],
    "objects": [
        {
            "id": "delivery_object",
            "asset_uri": "simulation/assets/primitives/task-object.usda",
            "pose": {"xyz": [2.587, -3.058, 0.734], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]}
        }
    ],
    "regions": [
        {"id": "delivery_target", "center": [0.43, 1.755, 0.61], "radius": 0.15}
    ],
    "events": [],
    "dialogue": [],
    "postconditions": "COPY VERBATIM from pick-deliver-place.json"
}
```

- [ ] **Step 4: Add the map-consistency test** (append to `tests/test_actor_paths.py`; artifacts are committed, so this is repo-stable):

```python
class ArenaScenarioPoseTest(unittest.TestCase):
    ARTIFACT = ROOT / "artifacts/arena/rcw2026/d2b559b43207c8d54ae2609f638dca1cc36ee8b7adc7e4d94aee86e7fb56729c"

    def test_person_scenario_poses_are_free_on_derived_map(self):
        from tinker_sim_core.occupancy import OccupancyMap
        occupancy = OccupancyMap.from_pgm(
            self.ARTIFACT / "map.pgm", resolution=0.05, origin_x=-5.05, origin_y=-6.0
        )
        raw = json.loads(
            (ROOT / "simulation/scenarios/find-and-approach-person-rcw2026.json").read_text()
        )
        rx, ry, _ = raw["robot"]["initial_pose"]
        self.assertTrue(occupancy.free_with_clearance(rx, ry, 0.35))
        actor = raw["actors"][0]
        for x, y in actor["path"]:
            self.assertTrue(occupancy.free_with_clearance(x, y, 0.4))
```

- [ ] **Step 5: Run** `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_actor_paths.py tests/test_orchestration.py -q` — PASS.
- [ ] **Step 6: Commit** `git add simulation/scenarios/find-and-approach-person-rcw2026.json simulation/scenarios/pick-deliver-place-rcw2026.json tests/test_actor_paths.py && git commit -m "feat: arena-native scenario variants with map-verified poses"`

---

### Task 11: Safety supervisor navigation mode

**Files:**
- Create: `simulation/tinker_sim_core/safety_gating.py`
- Modify: `ros2_ws/src/tinker_sim_bridge/tinker_sim_bridge/safety_supervisor.py`
- Modify: `ros2_ws/src/tinker_sim_bridge/launch/navigation.launch.py`
- Test: Create `tests/test_safety_gating.py`

**Interfaces:**
- Produces: `effective_stop(desired_stop: bool, management_ready: bool, startup_hold: bool, restore_pending: bool, manage_controllers: bool) -> bool` in `tinker_sim_core.safety_gating` (system-python importable; the supervisor imports it). Supervisor params: `manage_controllers` (bool, default True), `required_sources` (string array, default `["xarm", "collision"]`).

- [ ] **Step 1: Write failing tests** (`tests/test_safety_gating.py`):

```python
from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))

from tinker_sim_core.safety_gating import effective_stop


class EffectiveStopTest(unittest.TestCase):
    def test_managed_mode_preserves_all_latches(self):
        self.assertTrue(effective_stop(False, False, False, False, True))
        self.assertTrue(effective_stop(False, True, True, False, True))
        self.assertTrue(effective_stop(False, True, False, True, True))
        self.assertFalse(effective_stop(False, True, False, False, True))
        self.assertTrue(effective_stop(True, True, False, False, True))

    def test_unmanaged_mode_reduces_to_desired(self):
        self.assertFalse(effective_stop(False, False, True, True, False))
        self.assertTrue(effective_stop(True, False, False, False, False))
```

- [ ] **Step 2: Run** — FAIL (module missing).
- [ ] **Step 3: Implement** `simulation/tinker_sim_core/safety_gating.py`:

```python
from __future__ import annotations


def effective_stop(
    desired_stop: bool,
    management_ready: bool,
    startup_hold: bool,
    restore_pending: bool,
    manage_controllers: bool,
) -> bool:
    """Effective /sim/hardware/safety_stop value.

    Managed mode (manipulation): the controller-lifecycle latches gate the
    clear exactly as before. Unmanaged mode (navigation — no
    /controller_manager exists): only the fused source state matters; the
    fail-closed source-freshness contract lives upstream in
    SafetySourceTracker and still asserts desired_stop on any stale source.
    """
    if not manage_controllers:
        return bool(desired_stop)
    return bool(
        desired_stop or not management_ready or startup_hold or restore_pending
    )
```

- [ ] **Step 4: Wire the supervisor** (`safety_supervisor.py`):
  - Imports: `from tinker_sim_core.safety_gating import effective_stop` (the nav launch already exports `PYTHONPATH` with `simulation/`; `launch-humble` sets it too).
  - `__init__`: `self.declare_parameter("manage_controllers", True)`; `self.declare_parameter("required_sources", list(self.REQUIRED_SOURCES))`; `self._manage_controllers = bool(self.get_parameter("manage_controllers").value)`; build `self._source_trackers`/`self._sources` from the parameter value instead of the class constant (validate every entry is a key of `self.SOURCES`, else `raise ValueError`). Optional sources handling unchanged.
  - `_publish_effective`: `active = effective_stop(self._desired_stop, self._management_ready, self._startup_hold, self._restore_pending, self._manage_controllers)`.
  - `_reconcile`: immediately after `self._publish_effective()`, add `if not self._manage_controllers: return` (before the `_management_ready` check), so unmanaged mode never touches controller-manager clients.
- [ ] **Step 5: Add the supervisor to `navigation.launch.py`** (in `_resolve`'s node list, near `command_gateway`):

```python
        Node(
            package="tinker_sim_bridge", executable="safety_supervisor", output="screen",
            parameters=[{
                "manage_controllers": False,
                "required_sources": ["collision"],
            }],
            additional_env=env,
        ),
```

(`env` already carries the `PYTHONPATH` pointing at `simulation/` for other bridge nodes in this launch.)
- [ ] **Step 6: Run** `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_safety_gating.py tests/test_navigation_launch_map.py -q` — PASS. Then rebuild: `./scripts/build-humble-overlay`.
- [ ] **Step 7: Commit** `git add simulation/tinker_sim_core/safety_gating.py ros2_ws/src/tinker_sim_bridge/tinker_sim_bridge/safety_supervisor.py ros2_ws/src/tinker_sim_bridge/launch/navigation.launch.py tests/test_safety_gating.py && git commit -m "feat: supervisor navigation mode with parameterized sources; nav launch runs it"`

---

### Task 12: Live evidence wave

**Files:** evidence under `reports/` (gitignored); no code changes expected — failures here reopen the corresponding task.

All runs: ROS_DOMAIN_ID=42, arena `rcw2026`, spawn `--spawn-xy=-2.0,-2.0`, launch env per Global Constraints. Do not touch the streaming viewer.

- [ ] **Step 1: Spawn fail-closed UX** — launch `./scripts/launch-isaac --sensor-profile navigation-parity --profile parity --scenario empty --seed 7 --headless --ros --arena rcw2026` (no `--spawn-xy`); expect a nonzero exit whose log tail contains `try --spawn-xy=`. Save the tail to `reports/arena-fixes-2026-08-19/spawn-fail-closed.log`.
- [ ] **Step 2: Head tracking** — launch sensor-rich with the arena + free spawn; start `command_gateway` + the 10 Hz safety-clear heartbeat is NOT needed anymore — instead run the supervisor path: this profile's launch is Isaac-only, so for the gateway clear use `ros2 run tinker_sim_bridge safety_supervisor --ros-args -p manage_controllers:=false -p 'required_sources:=["collision"]'` (plus the gateway). Command `/sim/controller/pan_tilt_commands` to tilt −0.3, pan ±1.0; assert `/isaac_joint_states` converges within 0.05 rad in <10 s sim. Save series to `reports/arena-fixes-2026-08-19/head-tracking.json`. Remove the README "head effort" limitation afterward (Task 13).
- [ ] **Step 3: Coast** — same stack: rotate at 0.5 rad/s for 30 s via `/sim/controller/base_commands` wheel velocities (or `/cmd_vel` + base_facade), stop commanding entirely, sample `/sim/internal/physics_truth` for 30 s; assert total XY drift < 0.1 m. Save to `reports/arena-fixes-2026-08-19/coast.json`, including a note on the documented protocol holes and the 0.5 s backstop.
- [ ] **Step 4: Person walks** — relaunch Isaac with `--scenario find-and-approach-person-rcw2026`; run `scenario_runner` (sourcing `.ros-vendor/humble` for `simulation_interfaces`), then `ros2 run tinker_sim_bridge actor_path_driver --root $PWD --scenario find-and-approach-person-rcw2026`; sample `/get_entity_state /World/Scenario/target_person` at 1 Hz during the walk; assert the pose traverses from (-2,-3.5) to (-1,-3.0) monotonically. Save series to `reports/arena-fixes-2026-08-19/person-walk.json`.
- [ ] **Step 5: Object on table** — relaunch with `--scenario pick-deliver-place-rcw2026`; run `scenario_runner`; sample physics truth 20 s; assert `delivery_object` rests (zero twist) at z ≈ 0.764 ± 0.02 (cube base on the 0.724 m tabletop) near (2.587, −3.058). Save to `reports/arena-fixes-2026-08-19/object-on-table.json`.
- [ ] **Step 6: Nav safety clear** — with the navigation-parity arena sim up, `TINKER_WS=/home/tinker/tk25_ws ./scripts/launch-humble navigation map_yaml:=$PWD/artifacts/arena/rcw2026/d2b559b43207c8d54ae2609f638dca1cc36ee8b7adc7e4d94aee86e7fb56729c/map.yaml`; with NO CLI heartbeat, assert `/sim/status/command_gateway` shows `"safety_stop": false` and the contract guard log shows `"state": "pass"` with empty `wrong_safety_stop_publishers`. Save excerpts to `reports/arena-fixes-2026-08-19/nav-safety.log`. Tear the stack down afterward (SIGINT the `ros2 launch` and `run_sim` PIDs explicitly; never pkill patterns that can match your own shell).

---

### Task 13: Docs, full discovery, wrap-up

**Files:**
- Modify: `README.md` (Known arena limitations, launch docs, changelog)
- Modify: `docs/simulation-handoff-2026-08-18.md` (append a dated update)

- [ ] **Step 1: README updates** — Known arena limitations: remove head-effort item (fixed, cite evidence); replace the lidar "self-hit ring" item with the corrected mechanism + fix ("dev-lidar raycast-floor artifact; occupied-origin clouds now publish empty"); document the fail-closed spawn (error text + `--spawn-xy` suggestion); add the wheel-slew bound and its relationship to Nav2 acc_lim; keep odometry yaw slip as a characteristic with the EKF/IMU dead-end note (sim IMU: orientation invalid, no accel — fusing it would duplicate odom vyaw). Launch docs: navigation launch now includes the supervisor (CLI heartbeat no longer needed — remove that guidance); arena scenario variants + `actor_path_driver` usage. Changelog: one 2026-08-19 entry summarizing all tasks with test counts and evidence paths.
- [ ] **Step 2: Full suites** — `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_deploy_cli_launch.py tests/test_arena_streaming.py tests/test_backend_arena_inputs.py tests/test_spawn_override.py tests/test_navigation_launch_map.py tests/test_occupancy.py tests/test_ros_gateway.py tests/test_actor_paths.py tests/test_safety_gating.py tests/test_base_velocity_slew.py tests/test_chassis_ballast.py tests/test_orchestration.py tests/test_arena_artifact.py -q` — all PASS. Then `python3 -m unittest discover -s tests` (full log to a file, not `| tail`): the failing set must now be exactly the environmental/external names (PIL `Resampling`, torch, `tinker_sim_bridge`-under-system-python loader errors, tk25_ws-branch `test_provenance`) — `test_base_velocity_slew` green.
- [ ] **Step 3: Commit docs** `git add README.md docs/simulation-handoff-2026-08-18.md && git commit -m "docs: record arena-findings fixes, corrected lidar mechanism, nav safety mode"`
- [ ] **Step 4: Report** — summarize per-fix evidence paths and any deviations.

## Self-review notes

- Spec coverage: A→Tasks 2-5, B→6-7, C→8-10, D→11; evidence→12; docs→13. tk25_ws stays untouched (user decision).
- Type consistency: `slew_velocity_target(current, target, max_delta)` used identically in Tasks 6/7; `free_with_clearance/nearest_free_world` signatures match between Tasks 2/3/10; `effective_stop` five-bool signature matches Tasks 11 usage.
- Known judgment points for the executor (verify, don't assume): exact name of `_map_metadata` in backend (Task 3), scenario loader entry point + local variable names in `scenario.py` (Task 8), stamp construction in `tests/test_ros_gateway.py` (Task 4), and whether `launch-humble`'s env already covers `simulation/` on PYTHONPATH for the supervisor import (Task 11) — each is a one-line adaptation at the named location.
