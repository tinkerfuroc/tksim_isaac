# Simulation Vision Stack Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Publish the head and wrist camera streams from Isaac on the real-driver ROS 2 contract, wire them into a working `sensor-rich` profile, and prove it by driving the real, unmodified `vision_util get_image` node from `~/tk25_ws` against the simulator.

**Architecture:** A new `camera_rig` module loads camera specs from `simulation/sensors/hardware-parity.json` (bumped to schema v2, becoming the single source of truth) and owns per-camera `RtxCamera`+`CameraSensor` pairs mounted on the robot's optical-frame prims. `ros_gateway.py` gains RELIABLE+VOLATILE image/CameraInfo publishers fed by the rig. A new `sensor-rich` branch in `run_sim.py` assembles backend + arena map + rig + gateway.

**Tech Stack:** Isaac Sim 6.0.1 (`isaacsim.sensors.experimental.rtx`), Isaac Lab spawners, Isaac-internal rclpy (Humble), numpy, plain `unittest` + Humble-side pytest.

**Spec:** `docs/superpowers/specs/2026-08-16-sim-vision-stack-design.md`

## Global Constraints

- Camera QoS: RELIABLE + VOLATILE + KEEP_LAST(10) — never `sensor_data`/BEST_EFFORT.
- Color encoding `rgb8`, exactly 3 bytes/pixel. Depth encoding `16UC1`, millimetres, NaN/Inf→0, clamp 65535.
- Color and depth of one capture share one identical sim-time stamp.
- Head camera: 1280×720, HFOV 90.0°, frame_id `camera_color_optical_frame`, mount `head_camera_color_optical_frame`. Wrist: 848×480, HFOV 69.4°, frame_id `xarm_camera_color_optical_frame`, mount `xarm_camera_color_optical_frame`. Target 15 Hz.
- Point cloud (flag-gated): `/camera/depth_registered/points`, organized, float32 xyz + 4-byte pad, `point_step=16`, NaN invalid.
- Never disturb the running arena session (PID 2967476, ROS domain 25, streaming lock). All live runs use `ROS_DOMAIN_ID=42`.
- Unit suite must stay green: `python3 -m unittest discover -s tests -v` (system python3, no Isaac, no rclpy).
- Isaac scripts run via `uv run --frozen --no-sync python …` from the project root with the env recipe in Task 9.
- `validation/run_sim.py` module stays importable under plain python3 (no top-level Isaac imports); Isaac imports stay inside `main()`/methods.
- Commits: small, per task, `Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>` trailer. Never `git add -A` (the worktree carries unrelated WIP: README/backend/cli/run_sim hunks for arena streaming + ballast — stage only files this plan names; for shared files stage selectively with `git add -p` equivalent care).

---

### Task 1: Arena palette core module

**Files:**
- Create: `simulation/tinker_sim_core/arena_palette.py`
- Modify: `validation/arena_vision_smoke.py` (drop local palette defs, import + re-export)
- Test: `tests/test_arena_vision_smoke.py` (already covers the functions via re-export; add one import-location test)

**Interfaces:**
- Produces: `WALL_PALETTE: tuple[tuple[str, tuple[float,float,float], float], ...]` (name, rgb 0–1, hue°); `wall_color(index: int) -> tuple[str, tuple[float,float,float], float]`; `expected_wall_colors(count: int) -> dict[str, int]`. Task 8 consumes `wall_color`; the smoke re-exports all three so existing tests keep working.

- [x] **Step 1: Write the failing test** — append to `tests/test_arena_vision_smoke.py`:

```python
class PaletteSourceTest(unittest.TestCase):
    def test_palette_is_shared_from_core(self) -> None:
        sys.path.insert(0, str(ROOT / "simulation"))
        from tinker_sim_core import arena_palette
        import arena_vision_smoke

        self.assertIs(arena_vision_smoke.WALL_PALETTE, arena_palette.WALL_PALETTE)
        self.assertIs(arena_vision_smoke.wall_color, arena_palette.wall_color)
```

- [x] **Step 2: Run test to verify it fails**

Run: `python3 -m unittest tests.test_arena_vision_smoke.PaletteSourceTest -v`
Expected: FAIL (`ModuleNotFoundError: tinker_sim_core.arena_palette`)

- [x] **Step 3: Create `simulation/tinker_sim_core/arena_palette.py`** with exactly the palette currently in `validation/arena_vision_smoke.py`:

```python
from __future__ import annotations

#: Saturated, evenly spaced hues for deterministic arena-wall coloring.
#: Sixty degrees of separation keeps hue classification unambiguous under
#: ray-traced lighting.  Entries are (name, diffuse rgb 0-1, hue degrees).
WALL_PALETTE = (
    ("red", (0.80, 0.02, 0.02), 0.0),
    ("yellow", (0.80, 0.80, 0.02), 60.0),
    ("green", (0.02, 0.80, 0.02), 120.0),
    ("cyan", (0.02, 0.80, 0.80), 180.0),
    ("blue", (0.02, 0.02, 0.80), 240.0),
    ("magenta", (0.80, 0.02, 0.80), 300.0),
)


def wall_color(index: int) -> tuple[str, tuple[float, float, float], float]:
    """Palette entry for wall ``index`` (modular, so adjacent walls differ)."""
    if index < 0:
        raise ValueError("wall index must not be negative")
    return WALL_PALETTE[index % len(WALL_PALETTE)]


def expected_wall_colors(count: int) -> dict[str, int]:
    """How many walls each palette color receives for ``count`` walls."""
    if count < 0:
        raise ValueError("wall count must not be negative")
    tally = {name: 0 for name, _rgb, _hue in WALL_PALETTE}
    for index in range(count):
        tally[wall_color(index)[0]] += 1
    return tally
```

- [x] **Step 4: Refactor `validation/arena_vision_smoke.py`** — delete its `WALL_PALETTE`, `wall_color`, `expected_wall_colors` definitions; directly under the existing `ROOT = Path(__file__).resolve().parents[1]` line add:

```python
sys.path.insert(0, str(ROOT / "simulation"))
from tinker_sim_core.arena_palette import (  # noqa: E402
    WALL_PALETTE,
    expected_wall_colors,
    wall_color,
)
```

Then remove the now-duplicate `sys.path.insert(0, str(ROOT / "simulation"))` inside `main()` (keep the `validation` insert).

- [x] **Step 5: Run the full test file**

Run: `python3 -m unittest tests.test_arena_vision_smoke -v`
Expected: all pass (existing 18 + new 1)

- [x] **Step 6: Commit**

```bash
git add simulation/tinker_sim_core/arena_palette.py validation/arena_vision_smoke.py tests/test_arena_vision_smoke.py
git commit -m "refactor: share the arena wall palette from tinker_sim_core"
```

---

### Task 2: hardware-parity.json v2 + spec loading

**Files:**
- Modify: `simulation/sensors/hardware-parity.json`
- Create: `simulation/tinker_sim_isaac/camera_rig.py`
- Test: `tests/test_camera_rig.py` (new)

**Interfaces:**
- Produces: `CameraStreamSpec` frozen dataclass with fields `name: str, color_topic: str, depth_topic: str, camera_info_topics: tuple[str, ...], frame_id: str, mount_prim: str, width: int, height: int, horizontal_fov_deg: float, tick_rate_hz: float`; `load_camera_specs(path: Path) -> tuple[CameraStreamSpec, ...]` (head first, then wrist); module constants `CAMERA_NAMES = ("head_camera", "wrist_camera")`, `COLOR_ANNOTATOR = "rgb"`, `DEPTH_ANNOTATOR = "distance_to_image_plane"`, `OPTICAL_TO_USD_CAMERA_WXYZ = (0.0, 1.0, 0.0, 0.0)`.

- [x] **Step 1: Rewrite `simulation/sensors/hardware-parity.json`** (full replacement; `clock`, `lidar`, `imu`, `contacts` blocks stay byte-identical to v1):

```json
{
  "schema_version": 2,
  "implementation": "gateway_rtx_camera_publishers",
  "clock": {"topic": "/clock", "reset_on_stop": false},
  "camera_qos": {
    "reliability": "reliable",
    "durability": "volatile",
    "history": "keep_last",
    "depth": 10,
    "rationale": "Matches the real drivers (tk26_vision config/realsense_qos.yaml). Every tk26_vision CameraInfo subscription and several image subscriptions are RELIABLE; a best-effort publisher delivers zero messages to them, silently."
  },
  "head_camera": {
    "color_topic": "/camera/color/image_raw",
    "depth_topic": "/camera/depth/image_raw",
    "camera_info_topics": ["/camera/color/camera_info"],
    "frame_id": "camera_color_optical_frame",
    "mount_prim": "head_camera_color_optical_frame",
    "width": 1280,
    "height": 720,
    "horizontal_fov_deg": 90.0,
    "tick_rate_hz": 15,
    "color_encoding": "rgb8",
    "depth_encoding": "16UC1",
    "depth_unit": "millimeter"
  },
  "wrist_camera": {
    "color_topic": "/camera/xarm_camera/color/image_raw",
    "depth_topic": "/camera/xarm_camera/aligned_depth_to_color/image_raw",
    "camera_info_topics": [
      "/camera/xarm_camera/color/camera_info",
      "/camera/xarm_camera/aligned_depth_to_color/camera_info"
    ],
    "frame_id": "xarm_camera_color_optical_frame",
    "mount_prim": "xarm_camera_color_optical_frame",
    "width": 848,
    "height": 480,
    "horizontal_fov_deg": 69.4,
    "tick_rate_hz": 15,
    "color_encoding": "rgb8",
    "depth_encoding": "16UC1",
    "depth_unit": "millimeter"
  },
  "point_cloud": {
    "topic": "/camera/depth_registered/points",
    "source": "head_camera",
    "enabled_by_flag": "--camera-pointcloud",
    "point_step": 16,
    "fields": ["x", "y", "z"],
    "organized": true
  },
  "lidar": {
    "pointcloud_topic": "/livox/lidar",
    "scan_topic": "/scan",
    "frame_id": "livox360",
    "tick_rate_hz": 10,
    "require_tick_rate_equal_scan_rate_base_hz": true,
    "qos": "sensor_data"
  },
  "imu": {
    "topic": "/livox/imu",
    "frame_id": "livox360",
    "tick_rate_hz": 200,
    "qos": "sensor_data"
  },
  "contacts": {
    "implementation": "isaacsim.sensors.experimental.physics.ContactSensor",
    "links": ["left_finger", "right_finger", "link_tcp"],
    "parity_topic": "/sim/parity/finger_contact",
    "truth_topic": "/sim/truth/contacts"
  }
}
```

- [x] **Step 2: Write the failing tests** — create `tests/test_camera_rig.py`:

```python
from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))

from tinker_sim_isaac.camera_rig import (
    CAMERA_NAMES,
    CameraStreamSpec,
    load_camera_specs,
)

CONTRACT = ROOT / "simulation/sensors/hardware-parity.json"


class LoadCameraSpecsTest(unittest.TestCase):
    def test_loads_committed_contract(self) -> None:
        specs = load_camera_specs(CONTRACT)
        self.assertEqual(tuple(spec.name for spec in specs), CAMERA_NAMES)
        head, wrist = specs
        self.assertEqual(head.color_topic, "/camera/color/image_raw")
        self.assertEqual(head.depth_topic, "/camera/depth/image_raw")
        self.assertEqual(head.camera_info_topics, ("/camera/color/camera_info",))
        self.assertEqual(head.frame_id, "camera_color_optical_frame")
        self.assertEqual(head.mount_prim, "head_camera_color_optical_frame")
        self.assertEqual((head.width, head.height), (1280, 720))
        self.assertEqual(head.horizontal_fov_deg, 90.0)
        self.assertEqual(head.tick_rate_hz, 15.0)
        self.assertEqual(
            wrist.camera_info_topics,
            (
                "/camera/xarm_camera/color/camera_info",
                "/camera/xarm_camera/aligned_depth_to_color/camera_info",
            ),
        )
        self.assertEqual((wrist.width, wrist.height), (848, 480))

    def _mutated(self, mutate) -> Path:
        raw = json.loads(CONTRACT.read_text(encoding="utf-8"))
        mutate(raw)
        handle = tempfile.NamedTemporaryFile(
            "w", suffix=".json", delete=False, encoding="utf-8"
        )
        json.dump(raw, handle)
        handle.close()
        self.addCleanup(Path(handle.name).unlink)
        return Path(handle.name)

    def test_rejects_wrong_schema_version(self) -> None:
        path = self._mutated(lambda raw: raw.update(schema_version=1))
        with self.assertRaisesRegex(ValueError, "schema_version"):
            load_camera_specs(path)

    def test_rejects_best_effort_qos(self) -> None:
        path = self._mutated(
            lambda raw: raw["camera_qos"].update(reliability="best_effort")
        )
        with self.assertRaisesRegex(ValueError, "reliable"):
            load_camera_specs(path)

    def test_rejects_wrong_depth_encoding(self) -> None:
        path = self._mutated(
            lambda raw: raw["head_camera"].update(depth_encoding="32FC1")
        )
        with self.assertRaisesRegex(ValueError, "16UC1"):
            load_camera_specs(path)

    def test_rejects_missing_camera(self) -> None:
        path = self._mutated(lambda raw: raw.pop("wrist_camera"))
        with self.assertRaisesRegex(ValueError, "wrist_camera"):
            load_camera_specs(path)

    def test_rejects_nonpositive_dimensions(self) -> None:
        path = self._mutated(lambda raw: raw["head_camera"].update(width=0))
        with self.assertRaisesRegex(ValueError, "positive"):
            load_camera_specs(path)


if __name__ == "__main__":
    unittest.main()
```

- [x] **Step 3: Run tests to verify they fail**

Run: `python3 -m unittest tests.test_camera_rig -v`
Expected: FAIL (`ModuleNotFoundError: tinker_sim_isaac.camera_rig`)

- [x] **Step 4: Create `simulation/tinker_sim_isaac/camera_rig.py`** (spec-loading half; Isaac class arrives in Task 6):

```python
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

#: Contract camera order: head first, wrist second.
CAMERA_NAMES = ("head_camera", "wrist_camera")
COLOR_ANNOTATOR = "rgb"
DEPTH_ANNOTATOR = "distance_to_image_plane"
#: Local rotation from a ROS optical frame (+Z forward, +Y down) to a USD
#: camera (-Z forward, +Y up): 180 degrees about X, as wxyz.
OPTICAL_TO_USD_CAMERA_WXYZ = (0.0, 1.0, 0.0, 0.0)
#: Default USD camera aperture the focal length is solved against (mm).
HORIZONTAL_APERTURE_MM = 20.955


@dataclass(frozen=True)
class CameraStreamSpec:
    name: str
    color_topic: str
    depth_topic: str
    camera_info_topics: tuple[str, ...]
    frame_id: str
    mount_prim: str
    width: int
    height: int
    horizontal_fov_deg: float
    tick_rate_hz: float


def _string(mapping: Mapping[str, Any], key: str, owner: str) -> str:
    value = mapping.get(key)
    if not isinstance(value, str) or not value:
        raise ValueError(f"{owner}.{key} must be a non-empty string")
    return value


def _positive_number(mapping: Mapping[str, Any], key: str, owner: str) -> float:
    value = mapping.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or value <= 0:
        raise ValueError(f"{owner}.{key} must be positive")
    return float(value)


def load_camera_specs(path: Path) -> tuple[CameraStreamSpec, ...]:
    """Load the camera contract; malformed declarations raise, never guess."""
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    if raw.get("schema_version") != 2:
        raise ValueError("hardware-parity sensor contract must be schema_version 2")
    qos = raw.get("camera_qos")
    if not isinstance(qos, Mapping) or qos.get("reliability") != "reliable":
        raise ValueError("camera_qos.reliability must be reliable")
    if qos.get("durability") != "volatile" or qos.get("history") != "keep_last":
        raise ValueError("camera_qos must be reliable + volatile + keep_last")
    specs = []
    for name in CAMERA_NAMES:
        camera = raw.get(name)
        if not isinstance(camera, Mapping):
            raise ValueError(f"sensor contract is missing {name}")
        if camera.get("color_encoding") != "rgb8":
            raise ValueError(f"{name}.color_encoding must be rgb8")
        if camera.get("depth_encoding") != "16UC1":
            raise ValueError(f"{name}.depth_encoding must be 16UC1")
        if camera.get("depth_unit") != "millimeter":
            raise ValueError(f"{name}.depth_unit must be millimeter")
        info_topics = camera.get("camera_info_topics")
        if (
            not isinstance(info_topics, list)
            or not info_topics
            or any(not isinstance(topic, str) or not topic for topic in info_topics)
        ):
            raise ValueError(f"{name}.camera_info_topics must be non-empty strings")
        specs.append(
            CameraStreamSpec(
                name=name,
                color_topic=_string(camera, "color_topic", name),
                depth_topic=_string(camera, "depth_topic", name),
                camera_info_topics=tuple(info_topics),
                frame_id=_string(camera, "frame_id", name),
                mount_prim=_string(camera, "mount_prim", name),
                width=int(_positive_number(camera, "width", name)),
                height=int(_positive_number(camera, "height", name)),
                horizontal_fov_deg=_positive_number(camera, "horizontal_fov_deg", name),
                tick_rate_hz=_positive_number(camera, "tick_rate_hz", name),
            )
        )
    return tuple(specs)
```

- [x] **Step 5: Run tests to verify they pass**

Run: `python3 -m unittest tests.test_camera_rig -v`
Expected: PASS (6 tests)

- [x] **Step 6: Commit**

```bash
git add simulation/sensors/hardware-parity.json simulation/tinker_sim_isaac/camera_rig.py tests/test_camera_rig.py
git commit -m "feat: camera contract v2 with fail-closed spec loading"
```

---

### Task 3: Optics and image conversion helpers

**Files:**
- Modify: `simulation/tinker_sim_isaac/camera_rig.py`
- Test: `tests/test_camera_rig.py`

**Interfaces:**
- Produces: `focal_from_fov(width_px: int, horizontal_fov_deg: float, horizontal_aperture_mm: float = HORIZONTAL_APERTURE_MM) -> float` (mm); `camera_info_fields(spec: CameraStreamSpec) -> dict` with keys `height:int, width:int, distortion_model:str, d:list[float], k:list[float] (9), r:list[float] (9), p:list[float] (12)`; `to_numpy(value) -> np.ndarray` (duck `.cpu()`/`.numpy()` then `asarray`); `rgb8_array(value, height: int, width: int) -> np.ndarray` uint8 `(H, W, 3)` C-contiguous; `depth_to_16uc1_mm(value) -> np.ndarray` uint16 `(H, W)`.

- [x] **Step 1: Write the failing tests** — append to `tests/test_camera_rig.py`:

```python
import math

import numpy as np

from tinker_sim_isaac.camera_rig import (
    HORIZONTAL_APERTURE_MM,
    camera_info_fields,
    depth_to_16uc1_mm,
    focal_from_fov,
    rgb8_array,
)


class OpticsTest(unittest.TestCase):
    def test_focal_matches_fov(self) -> None:
        focal = focal_from_fov(1280, 90.0)
        recovered = 2.0 * math.degrees(
            math.atan(HORIZONTAL_APERTURE_MM / (2.0 * focal))
        )
        self.assertAlmostEqual(recovered, 90.0, places=6)

    def test_camera_info_is_consistent_pinhole(self) -> None:
        head = load_camera_specs(CONTRACT)[0]
        fields = camera_info_fields(head)
        fx = head.width / (2.0 * math.tan(math.radians(head.horizontal_fov_deg) / 2))
        self.assertEqual((fields["height"], fields["width"]), (720, 1280))
        self.assertEqual(fields["distortion_model"], "plumb_bob")
        self.assertEqual(fields["d"], [0.0] * 5)
        self.assertAlmostEqual(fields["k"][0], fx, places=6)
        self.assertAlmostEqual(fields["k"][4], fx, places=6)  # square pixels
        self.assertAlmostEqual(fields["k"][2], 640.0)
        self.assertAlmostEqual(fields["k"][5], 360.0)
        self.assertEqual(fields["r"], [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        self.assertAlmostEqual(fields["p"][0], fx, places=6)
        self.assertEqual(len(fields["p"]), 12)


class DepthConversionTest(unittest.TestCase):
    def test_metres_become_rounded_millimetres(self) -> None:
        depth = np.array([[0.5, 1.2345]], dtype=np.float32)
        result = depth_to_16uc1_mm(depth)
        self.assertEqual(result.dtype, np.uint16)
        self.assertEqual(result.tolist(), [[500, 1234]])  # 1234.5 rounds to even

    def test_invalid_values_become_zero(self) -> None:
        depth = np.array([[np.nan, np.inf, -1.0, 0.0]], dtype=np.float32)
        self.assertEqual(depth_to_16uc1_mm(depth).tolist(), [[0, 0, 0, 0]])

    def test_clamps_to_uint16(self) -> None:
        self.assertEqual(depth_to_16uc1_mm(np.array([[70.0]])).tolist(), [[65535]])

    def test_squeezes_trailing_channel(self) -> None:
        depth = np.ones((2, 3, 1), dtype=np.float32)
        self.assertEqual(depth_to_16uc1_mm(depth).shape, (2, 3))

    def test_rejects_wrong_rank(self) -> None:
        with self.assertRaisesRegex(ValueError, "depth"):
            depth_to_16uc1_mm(np.ones(4, dtype=np.float32))


class Rgb8ArrayTest(unittest.TestCase):
    def test_strips_alpha_and_batch(self) -> None:
        frame = np.zeros((1, 2, 3, 4), dtype=np.uint8)
        result = rgb8_array(frame, 2, 3)
        self.assertEqual(result.shape, (2, 3, 3))
        self.assertTrue(result.flags["C_CONTIGUOUS"])

    def test_scales_unit_floats(self) -> None:
        frame = np.ones((2, 3, 3), dtype=np.float32)
        self.assertEqual(int(rgb8_array(frame, 2, 3).max()), 255)

    def test_rejects_wrong_resolution(self) -> None:
        with self.assertRaisesRegex(ValueError, "shape"):
            rgb8_array(np.zeros((4, 4, 3), dtype=np.uint8), 2, 3)
```

- [x] **Step 2: Run tests to verify they fail**

Run: `python3 -m unittest tests.test_camera_rig -v`
Expected: FAIL (ImportError on the new names)

- [x] **Step 3: Implement in `camera_rig.py`:**

```python
def focal_from_fov(
    width_px: int,
    horizontal_fov_deg: float,
    horizontal_aperture_mm: float = HORIZONTAL_APERTURE_MM,
) -> float:
    """USD focal length (mm) so the render matches the declared horizontal FOV."""
    if width_px <= 0 or not 0.0 < horizontal_fov_deg < 180.0:
        raise ValueError("focal_from_fov requires positive width and FOV in (0, 180)")
    return horizontal_aperture_mm / (
        2.0 * math.tan(math.radians(horizontal_fov_deg) / 2.0)
    )


def camera_info_fields(spec: CameraStreamSpec) -> dict[str, Any]:
    """Pinhole CameraInfo values consistent with the rendered FOV by construction."""
    fx = spec.width / (2.0 * math.tan(math.radians(spec.horizontal_fov_deg) / 2.0))
    cx = spec.width / 2.0
    cy = spec.height / 2.0
    return {
        "height": spec.height,
        "width": spec.width,
        "distortion_model": "plumb_bob",
        "d": [0.0] * 5,
        "k": [fx, 0.0, cx, 0.0, fx, cy, 0.0, 0.0, 1.0],
        "r": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        "p": [fx, 0.0, cx, 0.0, 0.0, fx, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
    }


def to_numpy(value: Any):
    """Normalize warp/torch/proxy buffers to a host numpy array."""
    import numpy as np

    candidate = value
    if hasattr(candidate, "cpu"):
        candidate = candidate.cpu()
    if hasattr(candidate, "numpy"):
        candidate = candidate.numpy()
    return np.asarray(candidate)


def rgb8_array(value: Any, height: int, width: int):
    """Normalize an RTX rgb annotator buffer to contiguous uint8 (H, W, 3)."""
    import numpy as np

    array = to_numpy(value)
    if array.ndim == 4 and array.shape[0] == 1:
        array = array[0]
    if array.ndim != 3 or array.shape[:2] != (height, width):
        raise ValueError(f"unexpected RGB frame shape: {array.shape}")
    if array.shape[2] < 3:
        raise ValueError(f"RGB frame has fewer than three channels: {array.shape}")
    array = array[:, :, :3]
    if array.dtype.kind == "f":
        if not np.isfinite(array).all():
            raise ValueError("RGB frame contains non-finite values")
        scale = 255.0 if float(array.max(initial=0.0)) <= 1.0 else 1.0
        array = np.clip(array * scale, 0.0, 255.0).astype(np.uint8)
    else:
        array = np.clip(array, 0, 255).astype(np.uint8)
    return np.ascontiguousarray(array)


def depth_to_16uc1_mm(value: Any):
    """Metres -> 16UC1 millimetres; NaN/Inf/non-positive -> 0; clamp 65535."""
    import numpy as np

    depth = to_numpy(value)
    if depth.ndim == 3 and depth.shape[2] == 1:
        depth = depth[:, :, 0]
    if depth.ndim != 2:
        raise ValueError(f"depth frame must be HxW: {depth.shape}")
    depth = depth.astype(np.float64, copy=False)
    millimetres = np.where(
        np.isfinite(depth) & (depth > 0.0), depth * 1000.0, 0.0
    )
    return np.clip(np.rint(millimetres), 0.0, 65535.0).astype(np.uint16)
```

- [x] **Step 4: Run tests to verify they pass**

Run: `python3 -m unittest tests.test_camera_rig -v`
Expected: PASS

- [x] **Step 5: Commit**

```bash
git add simulation/tinker_sim_isaac/camera_rig.py tests/test_camera_rig.py
git commit -m "feat: camera optics and hardware-parity image conversions"
```

---

### Task 4: Registered point-cloud packing

**Files:**
- Modify: `simulation/tinker_sim_isaac/camera_rig.py`
- Test: `tests/test_camera_rig.py`

**Interfaces:**
- Produces: `pack_registered_cloud(depth_value, *, fx: float, fy: float, cx: float, cy: float) -> bytes` — organized H×W cloud, per-point `x, y, z, pad` float32 little-endian (`point_step=16`), NaN xyz for invalid depth, optical-frame unprojection `x=(u−cx)/fx·z`, `y=(v−cy)/fy·z`.

- [x] **Step 1: Write the failing tests** — append to `tests/test_camera_rig.py`:

```python
from tinker_sim_isaac.camera_rig import pack_registered_cloud


class PackRegisteredCloudTest(unittest.TestCase):
    def test_unprojects_with_intrinsics(self) -> None:
        depth = np.array([[2.0, np.nan]], dtype=np.float32)
        data = pack_registered_cloud(depth, fx=100.0, fy=100.0, cx=0.5, cy=0.0)
        self.assertEqual(len(data), 2 * 16)
        cloud = np.frombuffer(data, dtype=np.float32).reshape(2, 4)
        # pixel (u=0, v=0): x = (0 - 0.5)/100 * 2 = -0.01, y = 0, z = 2
        np.testing.assert_allclose(cloud[0, :3], [-0.01, 0.0, 2.0], atol=1e-6)
        self.assertEqual(cloud[0, 3], 0.0)  # 4-byte pad
        self.assertTrue(np.isnan(cloud[1, :3]).all())  # invalid depth -> NaN

    def test_rejects_wrong_rank(self) -> None:
        with self.assertRaisesRegex(ValueError, "depth"):
            pack_registered_cloud(
                np.ones(3, dtype=np.float32), fx=1.0, fy=1.0, cx=0.0, cy=0.0
            )
```

- [x] **Step 2: Run tests to verify they fail**

Run: `python3 -m unittest tests.test_camera_rig.PackRegisteredCloudTest -v`
Expected: FAIL (ImportError)

- [x] **Step 3: Implement in `camera_rig.py`:**

```python
def pack_registered_cloud(
    depth_value: Any, *, fx: float, fy: float, cx: float, cy: float
) -> bytes:
    """Organized float32 xyz+pad cloud (point_step=16) from registered depth.

    Matches the real Orbbec driver layout: dense HxW rows, NaN xyz for invalid
    depth, optical-frame axes.  This deliberately reproduces the driver's
    4-float stride that ``door_detection``'s 5-float parser cannot read.
    """
    import numpy as np

    depth = to_numpy(depth_value)
    if depth.ndim == 3 and depth.shape[2] == 1:
        depth = depth[:, :, 0]
    if depth.ndim != 2:
        raise ValueError(f"depth frame must be HxW: {depth.shape}")
    depth = depth.astype(np.float32, copy=False)
    height, width = depth.shape
    z = np.where(np.isfinite(depth) & (depth > 0.0), depth, np.float32(np.nan))
    columns = np.arange(width, dtype=np.float32)[None, :]
    rows = np.arange(height, dtype=np.float32)[:, None]
    cloud = np.zeros((height, width, 4), dtype=np.float32)
    cloud[:, :, 0] = (columns - cx) / fx * z
    cloud[:, :, 1] = (rows - cy) / fy * z
    cloud[:, :, 2] = z
    return cloud.tobytes()
```

- [x] **Step 4: Run tests to verify they pass**

Run: `python3 -m unittest tests.test_camera_rig -v`
Expected: PASS

- [x] **Step 5: Commit**

```bash
git add simulation/tinker_sim_isaac/camera_rig.py tests/test_camera_rig.py
git commit -m "feat: organized registered point-cloud packing"
```

---

### Task 5: Backend wall_color_fn

**Files:**
- Modify: `simulation/tinker_sim_isaac/backend.py` (init signature near line 86; wall loop near line 167)
- Test: `tests/test_camera_rig.py` (module-level inspection only — backend cannot be instantiated without Isaac)

**Interfaces:**
- Consumes: nothing new.
- Produces: `IsaacWholeRobotBackend.__init__(..., wall_color_fn: Callable[[int], tuple[float, float, float]] | None = None)`; module constant `DEFAULT_WALL_COLOR = (0.35, 0.38, 0.42)`. Task 8 passes `wall_color_fn=lambda index: wall_color(index)[1]`.

- [x] **Step 1: Write the failing test** — append to `tests/test_camera_rig.py`:

```python
import inspect


class BackendWallColorTest(unittest.TestCase):
    def test_backend_accepts_wall_color_fn(self) -> None:
        from tinker_sim_isaac import backend

        self.assertEqual(backend.DEFAULT_WALL_COLOR, (0.35, 0.38, 0.42))
        parameter = inspect.signature(
            backend.IsaacWholeRobotBackend.__init__
        ).parameters["wall_color_fn"]
        self.assertIsNone(parameter.default)
```

- [x] **Step 2: Run test to verify it fails**

Run: `python3 -m unittest tests.test_camera_rig.BackendWallColorTest -v`
Expected: FAIL (`KeyError: 'wall_color_fn'`)

- [x] **Step 3: Implement in `backend.py`.** Add near the other module constants (below the `CHASSIS_BALLAST_*` block):

```python
#: Uniform wall material used when no palette override is supplied.
DEFAULT_WALL_COLOR = (0.35, 0.38, 0.42)
```

Add `Callable` to the existing `typing` import. Append to the `__init__` keyword parameters (after `task: str = ""`):

```python
        wall_color_fn: Callable[[int], tuple[float, float, float]] | None = None,
```

Replace the wall-spawn loop body (currently `visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.35, 0.38, 0.42))`):

```python
            for index, (x, y, sx, sy) in enumerate(self.occupancy.rectangles()):
                color = (
                    DEFAULT_WALL_COLOR
                    if wall_color_fn is None
                    else tuple(wall_color_fn(index))
                )
                box = sim_utils.CuboidCfg(
                    size=(sx, sy, 1.2),
                    rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
                    collision_props=sim_utils.CollisionPropertiesCfg(),
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color),
                )
                box.func(
                    f"/World/NavigationMap/occupied_{index:04d}",
                    box,
                    translation=(x, y, 0.6),
                )
```

- [x] **Step 4: Run test + full suite** (backend.py carries unrelated WIP; the suite guards against collateral damage)

Run: `python3 -m unittest discover -s tests -v 2>&1 | tail -5`
Expected: OK

- [x] **Step 5: Commit (stage the backend hunks for this change only — the file also holds uncommitted ballast WIP; use `git add -p simulation/tinker_sim_isaac/backend.py` and pick only the DEFAULT_WALL_COLOR/wall_color_fn hunks, or if interleaving makes that impractical, note in the commit body that ballast WIP hunks ride along and get amended out later by their owner — prefer the selective staging)**

```bash
git add -p simulation/tinker_sim_isaac/backend.py
git add tests/test_camera_rig.py
git commit -m "feat: optional deterministic wall palette for arena cuboids"
```

---

### Task 6: CameraRig Isaac class

**Files:**
- Modify: `simulation/tinker_sim_isaac/camera_rig.py`

**Interfaces:**
- Consumes: `CameraStreamSpec`, `focal_from_fov`, annotator constants (Tasks 2–3).
- Produces: `class CameraRig` with `__init__(self, specs: tuple[CameraStreamSpec, ...])`, attribute `specs`, `initialize(self, app) -> None` (creates cameras; raises on missing mounts), `capture(self) -> dict[str, tuple[Any | None, Any | None]]` (name → (rgb buffer, depth buffer), either may be None if the annotator has no frame). No unit test — Isaac-only; validated live in Task 9.

- [x] **Step 1: Append to `camera_rig.py`:**

```python
class CameraRig:
    """Robot-mounted RTX cameras serving the hardware-parity contract.

    Cameras are created as children of the robot's optical-frame prims so they
    track pan/tilt and arm motion through physics.  Initialization is
    fail-closed: a contract whose mounts or sensors cannot be realized must
    not produce a silently camera-less simulator.
    """

    def __init__(self, specs: tuple[CameraStreamSpec, ...]) -> None:
        if not specs:
            raise ValueError("camera rig requires at least one camera spec")
        self.specs = tuple(specs)
        self._sensors: dict[str, Any] = {}

    def initialize(self, app: Any) -> None:
        from isaacsim.core.utils.extensions import enable_extension

        enable_extension("isaacsim.sensors.experimental.rtx")
        for _ in range(4):
            app.update()

        import omni.usd
        from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera
        from pxr import Gf, Usd, UsdGeom

        stage = omni.usd.get_context().get_stage()
        robot = stage.GetPrimAtPath("/World/Tinker")
        if not robot.IsValid():
            raise RuntimeError("camera rig requires the spawned /World/Tinker robot")
        for spec in self.specs:
            mounts = [
                prim
                for prim in Usd.PrimRange(robot)
                if prim.GetName() == spec.mount_prim
            ]
            if len(mounts) != 1:
                raise RuntimeError(
                    f"expected exactly one {spec.mount_prim!r} prim under "
                    f"/World/Tinker, found {len(mounts)}"
                )
            camera_path = f"{mounts[0].GetPath().pathString}/rtx_camera"
            camera = RtxCamera(camera_path, tick_rate=float(spec.tick_rate_hz))
            prim = stage.GetPrimAtPath(camera_path)
            usd_camera = UsdGeom.Camera(prim)
            focal = focal_from_fov(spec.width, spec.horizontal_fov_deg)
            usd_camera.GetFocalLengthAttr().Set(focal)
            usd_camera.GetHorizontalApertureAttr().Set(HORIZONTAL_APERTURE_MM)
            usd_camera.GetVerticalApertureAttr().Set(
                HORIZONTAL_APERTURE_MM * spec.height / spec.width
            )
            usd_camera.GetClippingRangeAttr().Set(Gf.Vec2f(0.05, 200.0))
            # Identity translation; rotate the USD camera into the ROS optical
            # convention so the render looks along the frame's +Z axis.
            xform = UsdGeom.Xformable(prim)
            xform.ClearXformOpOrder()
            xform.AddOrientOp().Set(Gf.Quatf(*OPTICAL_TO_USD_CAMERA_WXYZ))
            self._sensors[spec.name] = CameraSensor(
                camera,
                resolution=(spec.height, spec.width),
                annotators=[COLOR_ANNOTATOR, DEPTH_ANNOTATOR],
            )
        for _ in range(4):
            app.update()

    def capture(self) -> dict[str, tuple[Any, Any]]:
        frames: dict[str, tuple[Any, Any]] = {}
        for name, sensor in self._sensors.items():
            rgb, _info = sensor.get_data(COLOR_ANNOTATOR)
            depth, _info = sensor.get_data(DEPTH_ANNOTATOR)
            frames[name] = (rgb, depth)
        return frames
```

- [x] **Step 2: Run the unit suite (module must stay importable without Isaac)**

Run: `python3 -m unittest tests.test_camera_rig -v`
Expected: PASS

- [x] **Step 3: Commit**

```bash
git add simulation/tinker_sim_isaac/camera_rig.py
git commit -m "feat: robot-mounted RTX camera rig"
```

---

### Task 7: Gateway camera publishers

**Files:**
- Modify: `simulation/tinker_sim_isaac/ros_gateway.py` (constructor at line 82; add `publish_cameras` near `publish` at line 819)

**Interfaces:**
- Consumes: `CameraRig.capture()`, `camera_info_fields`, `rgb8_array`, `depth_to_16uc1_mm`, `pack_registered_cloud` (Tasks 2–6); existing `self._stamp()`, `self.node`, QoS imports already present at lines 85–91.
- Produces: `RosStandardGateway.__init__(self, backend, *, development_lidar: bool = False, camera_rig=None, camera_pointcloud: bool = False)`; `publish_cameras(self) -> None`; counter attribute `camera_skipped_frames: int`. Task 8's loop calls `publish_cameras()` every camera stride.

- [x] **Step 1: Add the sibling import** at the top of `ros_gateway.py` (module level, next to the existing `tinker_sim_core` imports):

```python
from tinker_sim_isaac.camera_rig import (
    camera_info_fields,
    depth_to_16uc1_mm,
    pack_registered_cloud,
    rgb8_array,
)
```

- [x] **Step 2: Extend the constructor.** Change the signature at line 82 to:

```python
    def __init__(
        self,
        backend: Any,
        *,
        development_lidar: bool = False,
        camera_rig: Any | None = None,
        camera_pointcloud: bool = False,
    ) -> None:
```

After the existing publisher block (after `self.contact_pub = ...`), add:

```python
        self._camera_rig = camera_rig
        self.camera_skipped_frames = 0
        self._camera_streams: list[dict[str, Any]] = []
        self._camera_cloud_pub = None
        if camera_rig is not None:
            from sensor_msgs.msg import CameraInfo, Image

            self._Image = Image
            self._CameraInfo = CameraInfo
            # The real drivers publish RELIABLE + VOLATILE + KEEP_LAST(10)
            # (tk26_vision realsense_qos.yaml).  Every tk26_vision CameraInfo
            # subscription is RELIABLE; a best-effort publisher would deliver
            # zero messages to them, silently.
            camera_qos = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            for spec in camera_rig.specs:
                self._camera_streams.append(
                    {
                        "spec": spec,
                        "info_fields": camera_info_fields(spec),
                        "color_pub": self.node.create_publisher(
                            Image, spec.color_topic, camera_qos
                        ),
                        "depth_pub": self.node.create_publisher(
                            Image, spec.depth_topic, camera_qos
                        ),
                        "info_pubs": [
                            self.node.create_publisher(CameraInfo, topic, camera_qos)
                            for topic in spec.camera_info_topics
                        ],
                    }
                )
            if camera_pointcloud:
                self._camera_cloud_pub = self.node.create_publisher(
                    PointCloud2, "/camera/depth_registered/points", camera_qos
                )
```

(`QoSProfile`, `ReliabilityPolicy`, `DurabilityPolicy`, `PointCloud2`, `PointField` are already imported in this constructor — reuse them; keep this block after those imports.)

- [x] **Step 3: Add `publish_cameras`** after the existing `publish` method:

```python
    def publish_cameras(self) -> None:
        """Publish one same-stamp color+depth+info set per camera.

        A camera whose annotator has no frame this tick is skipped and counted
        rather than fabricated; the counter keeps stalls observable.
        """
        if self._camera_rig is None:
            return
        stamp = self._stamp()
        frames = self._camera_rig.capture()
        for entry in self._camera_streams:
            spec = entry["spec"]
            rgb, depth = frames.get(spec.name, (None, None))
            if rgb is None or depth is None:
                self.camera_skipped_frames += 1
                continue
            color_array = rgb8_array(rgb, spec.height, spec.width)
            depth_array = depth_to_16uc1_mm(depth)
            if depth_array.shape != (spec.height, spec.width):
                raise RuntimeError(
                    f"{spec.name} depth resolution {depth_array.shape} does not "
                    f"match the contract ({spec.height}, {spec.width})"
                )

            color = self._Image()
            color.header.stamp = stamp
            color.header.frame_id = spec.frame_id
            color.height = spec.height
            color.width = spec.width
            color.encoding = "rgb8"
            color.is_bigendian = 0
            color.step = spec.width * 3
            color.data = color_array.tobytes()
            entry["color_pub"].publish(color)

            depth_msg = self._Image()
            depth_msg.header.stamp = stamp
            depth_msg.header.frame_id = spec.frame_id
            depth_msg.height = spec.height
            depth_msg.width = spec.width
            depth_msg.encoding = "16UC1"
            depth_msg.is_bigendian = 0
            depth_msg.step = spec.width * 2
            depth_msg.data = depth_array.tobytes()
            entry["depth_pub"].publish(depth_msg)

            fields = entry["info_fields"]
            info = self._CameraInfo()
            info.header.stamp = stamp
            info.header.frame_id = spec.frame_id
            info.height = fields["height"]
            info.width = fields["width"]
            info.distortion_model = fields["distortion_model"]
            info.d = list(fields["d"])
            info.k = fields["k"]
            info.r = fields["r"]
            info.p = fields["p"]
            for publisher in entry["info_pubs"]:
                publisher.publish(info)

            if self._camera_cloud_pub is not None and spec.name == "head_camera":
                cloud = self._PointCloud2()
                cloud.header.stamp = stamp
                cloud.header.frame_id = spec.frame_id
                cloud.height = spec.height
                cloud.width = spec.width
                cloud.fields = [
                    self._PointField(
                        name=name,
                        offset=offset,
                        datatype=self._PointField.FLOAT32,
                        count=1,
                    )
                    for name, offset in (("x", 0), ("y", 4), ("z", 8))
                ]
                cloud.is_bigendian = False
                cloud.point_step = 16
                cloud.row_step = 16 * spec.width
                cloud.is_dense = False
                cloud.data = pack_registered_cloud(
                    depth,
                    fx=fields["k"][0],
                    fy=fields["k"][4],
                    cx=fields["k"][2],
                    cy=fields["k"][5],
                )
                self._camera_cloud_pub.publish(cloud)
```

- [x] **Step 4: Surface the skip counter in telemetry.** Locate the `/sim/status/isaac` payload construction (`grep -n "status_pub.publish\|sim/status" simulation/tinker_sim_isaac/ros_gateway.py`) and add `"camera_skipped_frames": self.camera_skipped_frames` to the JSON dict it serializes (only when `self._camera_rig is not None`, to leave other profiles' status payloads byte-identical).

- [x] **Step 5: Run the unit suite** (ros_gateway is not unit-imported without rclpy, but the sibling import must not break `tests/test_camera_rig.py`'s import of `camera_rig`)

Run: `python3 -m unittest discover -s tests -v 2>&1 | tail -5`
Expected: OK

- [x] **Step 6: Commit**

```bash
git add simulation/tinker_sim_isaac/ros_gateway.py
git commit -m "feat: hardware-parity camera publishers in the ROS gateway"
```

---

### Task 8: sensor-rich branch, flags, and CLI plumbing

**Files:**
- Modify: `validation/run_sim.py` (parser near line 396; guards near line 415; new `elif` before the `manipulation-core` branch at line 659; `gateway_lidar_enabled` at line 382)
- Modify: `tools/tinker_sim_deploy/cli.py` (launch parser near line 49; `_launch_command` near line 116)
- Test: `tests/test_arena_vision_smoke.py` (new test class; this file already imports `run_sim` helpers via the `validation` path — actually `tests/test_arena_streaming.py` does; add the new class THERE since it already stubs `run_sim` and `cli` imports)

**Interfaces:**
- Consumes: `IsaacWholeRobotBackend(wall_color_fn=...)` (Task 5), `CameraRig`/`load_camera_specs` (Tasks 2, 6), `RosStandardGateway(camera_rig=..., camera_pointcloud=...)` (Task 7), `wall_color` (Task 1), existing `_streaming_update_stride`, `_expected_scenario_objects`, `gateway_lidar_enabled`.
- Produces: `sensor_rich_implies_ros(sensor_profile: str, ros: bool) -> bool` in `run_sim.py` (pure, for tests); run_sim flags `--camera-pointcloud`, `--arena-colors` (each `parser.error`s unless `sensor-rich`); cli flags of the same names forwarded verbatim; `gateway_lidar_enabled("sensor-rich", anything) == True`.

- [x] **Step 1: Write the failing tests** — append to `tests/test_arena_streaming.py`:

```python
class SensorRichLaunchTest(unittest.TestCase):
    def test_sensor_rich_implies_ros(self) -> None:
        self.assertTrue(run_sim.sensor_rich_implies_ros("sensor-rich", False))
        self.assertFalse(run_sim.sensor_rich_implies_ros("sensor-rich", True))
        self.assertFalse(run_sim.sensor_rich_implies_ros("physics-only", False))

    def test_sensor_rich_enables_development_lidar(self) -> None:
        self.assertTrue(run_sim.gateway_lidar_enabled("sensor-rich", False))
        self.assertTrue(run_sim.gateway_lidar_enabled("sensor-rich", True))

    def test_launcher_forwards_camera_flags(self) -> None:
        args = _parse_cli_launch(
            [
                "launch",
                "--sensor-profile",
                "sensor-rich",
                "--ros",
                "--camera-pointcloud",
                "--arena-colors",
            ]
        )
        command = cli._launch_command(args)
        self.assertIn("--camera-pointcloud", command)
        self.assertIn("--arena-colors", command)

    def test_launcher_omits_camera_flags_by_default(self) -> None:
        args = _parse_cli_launch(
            ["launch", "--sensor-profile", "sensor-rich", "--ros"]
        )
        command = cli._launch_command(args)
        self.assertNotIn("--camera-pointcloud", command)
        self.assertNotIn("--arena-colors", command)
```

`tests/test_arena_streaming.py` already imports `run_sim` and `cli`; if it lacks a `_parse_cli_launch` helper, add one at module scope:

```python
def _parse_cli_launch(argv):
    return cli._parser().parse_args(argv)
```

(Check the file's existing import names first — it imports the modules under `run_sim` and `tinker_sim_deploy.cli`; reuse whatever aliases it already uses.)

- [x] **Step 2: Run tests to verify they fail**

Run: `python3 -m unittest tests.test_arena_streaming.SensorRichLaunchTest -v`
Expected: FAIL (`AttributeError: sensor_rich_implies_ros` / flag unrecognized)

- [x] **Step 3: Implement `run_sim.py` changes.**

3a. Pure helper next to `gateway_lidar_enabled`:

```python
def sensor_rich_implies_ros(sensor_profile: str, ros: bool) -> bool:
    """sensor-rich exists to serve hardware-parity topics; it forces --ros."""
    return sensor_profile == "sensor-rich" and not ros
```

3b. Extend `gateway_lidar_enabled`: change `if sensor_profile == "navigation-parity":` to

```python
    if sensor_profile in ("navigation-parity", "sensor-rich"):
        return True
```

3c. Parser additions (after `--livestream`):

```python
    parser.add_argument("--camera-pointcloud", action="store_true")
    parser.add_argument("--arena-colors", action="store_true")
```

Guards next to the livestream guards:

```python
    if args.camera_pointcloud and args.sensor_profile != "sensor-rich":
        parser.error("--camera-pointcloud is supported only with sensor-rich")
    if args.arena_colors and args.sensor_profile != "sensor-rich":
        parser.error("--arena-colors is supported only with sensor-rich")
    if sensor_rich_implies_ros(args.sensor_profile, args.ros):
        print("sensor-rich implies --ros; enabling the ROS gateway", flush=True)
        args.ros = True
```

(The implies-ros rewrite must run **before** the existing `if args.ros:` extension-enable block inside `try:` so `isaacsim.ros2.sim_control` still loads.)

3d. New branch, inserted between the `navigation-parity` branch and `elif args.sensor_profile == "manipulation-core":`:

```python
        elif args.sensor_profile == "sensor-rich":
            root = Path(__file__).resolve().parents[1]
            sys.path.insert(0, str(root / "simulation"))
            if args.artifact is None:
                current = json.loads(
                    (root / "artifacts/robot/tinker2/current.json").read_text()
                )
                manifest_path = Path(current["manifest"])
                if not manifest_path.is_absolute():
                    manifest_path = root / manifest_path
                args.artifact = manifest_path.parent / "robot.usd"
            if args.map_yaml is None:
                args.map_yaml = args.artifact.parent / "map.yaml"
            expected_objects = _expected_scenario_objects(root, args.scenario)
            wall_color_fn = None
            if args.arena_colors:
                from tinker_sim_core.arena_palette import wall_color

                wall_color_fn = lambda index: wall_color(index)[1]  # noqa: E731
            from tinker_sim_isaac.backend import IsaacWholeRobotBackend

            backend = IsaacWholeRobotBackend(
                usd_path=args.artifact,
                map_yaml=args.map_yaml,
                seed=args.seed,
                render=False,
                enable_contacts=False,
                add_ground_plane=True,
                expected_objects=expected_objects,
                scenario=args.scenario,
                task=args.scenario,
                wall_color_fn=wall_color_fn,
            )
            from tinker_sim_isaac.camera_rig import CameraRig, load_camera_specs

            camera_specs = load_camera_specs(
                root / "simulation/sensors/hardware-parity.json"
            )
            camera_rig = CameraRig(camera_specs)
            camera_rig.initialize(app)
            from tinker_sim_isaac.ros_gateway import RosStandardGateway

            gateway = RosStandardGateway(
                backend,
                development_lidar=gateway_lidar_enabled(
                    args.sensor_profile, args.qualification
                ),
                camera_rig=camera_rig,
                camera_pointcloud=args.camera_pointcloud,
            )
            camera_hz = min(spec.tick_rate_hz for spec in camera_specs)
            camera_stride = _streaming_update_stride(backend.dt, update_hz=camera_hz)
            print(
                json.dumps(
                    {
                        "artifact": str(args.artifact),
                        "map": str(args.map_yaml),
                        "arena_colors": args.arena_colors,
                        "cameras": {
                            spec.name: {
                                "color_topic": spec.color_topic,
                                "depth_topic": spec.depth_topic,
                                "camera_info_topics": list(spec.camera_info_topics),
                                "frame_id": spec.frame_id,
                                "resolution": [spec.width, spec.height],
                                "target_hz": camera_hz,
                            }
                            for spec in camera_specs
                        },
                        "camera_pointcloud": args.camera_pointcloud,
                        "physics_device": backend.physics_device,
                        "profile": "sensor-rich",
                        "ros": args.ros,
                        "scenario": args.scenario,
                        "seed": args.seed,
                        "simulation_control": "isaacsim.ros2.sim_control",
                    },
                    sort_keys=True,
                ),
                flush=True,
            )
            import omni.timeline

            # No external scenario runner drives this profile by default;
            # start the timeline so cameras and physics run immediately.
            # isaacsim.ros2.sim_control can still pause/resume it.
            omni.timeline.get_timeline_interface().play()
            next_step_wall = time.monotonic()
            next_collision_heartbeat = time.monotonic()
            collision_heartbeat_period_s = 0.1
            camera_frame_index = 0
            while (
                running
                and app.is_running()
                and (args.duration <= 0.0 or backend.simulation_time < args.duration)
            ):
                gateway.spin_once()
                if (
                    time.monotonic() - next_collision_heartbeat
                    >= collision_heartbeat_period_s
                ):
                    gateway.publish_safety_heartbeat()
                    next_collision_heartbeat = time.monotonic()
                if omni.timeline.get_timeline_interface().is_playing():
                    backend.step()
                    if not running:
                        break
                    try:
                        gateway.publish()
                        camera_frame_index += 1
                        if camera_frame_index % camera_stride == 0:
                            gateway.publish_cameras()
                    except BaseException:
                        if running:
                            raise
                        break
                    if not running:
                        break
                    app.update()
                    next_step_wall += backend.dt
                    remaining = next_step_wall - time.monotonic()
                    if remaining > 0.0:
                        time.sleep(remaining)
                    elif remaining < -1.0:
                        next_step_wall = time.monotonic()
                else:
                    app.update()
                    time.sleep(0.001)
```

- [x] **Step 4: Implement `cli.py` changes.** Launch parser (after `--dds-profile`, before `isaac_args`):

```python
    launch.add_argument(
        "--camera-pointcloud",
        action="store_true",
        help="publish /camera/depth_registered/points under sensor-rich",
    )
    launch.add_argument(
        "--arena-colors",
        action="store_true",
        help="color the occupancy walls with the deterministic palette",
    )
```

In `_launch_command`, after the `--qualification` append and before `--livestream`:

```python
        if args.camera_pointcloud:
            command.append("--camera-pointcloud")
        if args.arena_colors:
            command.append("--arena-colors")
```

- [x] **Step 5: Run tests to verify they pass, then the full suite** (the exact-list assertion in `test_launcher_builds_navigation_arena_stream_command` must still pass — the new flags default to False and append nothing)

Run: `python3 -m unittest tests.test_arena_streaming -v && python3 -m unittest discover -s tests -v 2>&1 | tail -3`
Expected: OK both

- [x] **Step 6: Commit (both files carry unrelated WIP hunks — stage selectively)**

```bash
git add -p validation/run_sim.py tools/tinker_sim_deploy/cli.py
git add tests/test_arena_streaming.py
git commit -m "feat: implement the sensor-rich profile with camera publishing"
```

---

### Task 9: Live sensor-rich smoke (integration checkpoint)

**Files:**
- No planned source changes — this task boots the real thing and fixes what reality disagrees with. Any fix commits reference this task.

**Interfaces:**
- Consumes: everything above.
- Produces: a verified-running sensor-rich sim; measured achieved camera rate; evidence notes for Task 11.

- [x] **Step 1: Launch the sim in the background** (domain 42; does not touch the live arena session on 25):

```bash
cd /home/tinker/tinker-sim/6.0.1
unset PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_CURRENT_PREFIX \
      COLCON_PREFIX_PATH ROS_PACKAGE_PATH ROS_PYTHON_VERSION ROS_VERSION \
      LD_LIBRARY_PATH PYTHONHOME VIRTUAL_ENV FASTRTPS_DEFAULT_PROFILES_FILE
set -a; source .deployment.env; set +a
export ACCEPT_EULA=Y OMNI_KIT_ACCEPT_EULA=YES
export UV_CACHE_DIR=$PWD/.cache/uv/0.10
export UV_PYTHON_INSTALL_DIR=$PWD/.cache/uv-python/3.12.13
export XDG_CACHE_HOME=$PWD/.cache/isaac/6.0.1
export OV_CACHE_ROOT=$PWD/.cache/isaac/6.0.1/ov
export ISAACSIM_CACHE_PATH=$PWD/.cache/isaac/6.0.1
export ROS_DOMAIN_ID=42
export LD_LIBRARY_PATH=$PWD/.venv/lib/python3.12/site-packages/isaacsim/exts/isaacsim.ros2.core/humble/lib
export PATH=$HOME/.local/bin:$PATH
uv run --frozen --no-sync python validation/run_sim.py \
  --sensor-profile sensor-rich --profile parity --scenario empty --seed 7 \
  --headless --ros --arena-colors
```

Expected: the startup JSON prints a `cameras` block; no traceback.

- [x] **Step 2: Verify topics from a Humble shell** (separate terminal env):

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 topic list | grep camera
ros2 topic info /camera/color/image_raw --verbose   # QoS: RELIABLE, VOLATILE
ros2 topic hz /camera/color/image_raw --window 30   # target ~15 Hz, floor 10
ros2 topic echo /camera/color/image_raw --once --field encoding   # rgb8
ros2 topic echo /camera/depth/image_raw --once --field encoding   # 16UC1
ros2 topic echo /camera/xarm_camera/color/image_raw --once --field width  # 848
```

Expected: 7 camera topics (no cloud — flag off), RELIABLE/VOLATILE, encodings and sizes per contract, rate ≥ 10 Hz.

- [x] **Step 3: Iterate on failures.** Likely first-run issues and their fixes: annotator returns None for the first ticks (acceptable — `camera_skipped_frames` counts them); camera looks the wrong way (fix `OPTICAL_TO_USD_CAMERA_WXYZ` usage in `CameraRig.initialize`); rate below 10 Hz (drop `tick_rate_hz` in the contract JSON and rerun — contract change + test update together). Commit each fix separately with a message naming the observed symptom.

- [x] **Step 4: Record the achieved rate** (from `ros2 topic hz`) for the README in Task 11. Stop the sim (Ctrl-C / SIGINT).

---

### Task 10: Humble acceptance test — live get_image round-trip

**Files:**
- Create: `tests/ros_humble/test_vision_get_image_live.py`
- Evidence: `reports/vision-roundtrip/` (gitignored, referenced from README)

**Interfaces:**
- Consumes: running sensor-rich sim (Task 9 recipe), `vision_util get_image` node from `~/tk25_ws`, `hue_presence` from `validation/arena_vision_smoke.py`.
- Produces: recorded pass evidence; the test skips cleanly offline.

- [x] **Step 1: Create `tests/ros_humble/test_vision_get_image_live.py`:**

```python
"""Live vision round-trip: the real vision_util get_image node against the sim.

Requires (see README "Vision hardware-parity cameras"):
  1. sensor-rich sim running with --ros --arena-colors on ROS_DOMAIN_ID=42;
  2. `ros2 run vision_util get_image` under system Humble + the tk25_ws
     overlay on the same domain;
  3. TINKER_SIM_VISION_LIVE=1 in this test's environment.
Skipped otherwise; never part of the offline suite.
"""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path

import pytest

if os.environ.get("TINKER_SIM_VISION_LIVE") != "1":
    pytest.skip(
        "set TINKER_SIM_VISION_LIVE=1 with the sensor-rich sim and get_image running",
        allow_module_level=True,
    )

rclpy = pytest.importorskip("rclpy", reason="requires Humble ROS Python runtime")
pytest.importorskip("sensor_msgs.msg", reason="requires Humble sensor_msgs")
srv_module = pytest.importorskip(
    "tinker_vision_msgs_26.srv", reason="requires the sourced tk25_ws overlay"
)

import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "validation"))
from arena_vision_smoke import hue_presence  # noqa: E402

EVIDENCE_DIR = ROOT / "reports/vision-roundtrip"
SERVICE_NAME = "get_image_service"


@pytest.fixture(scope="module")
def get_image_client():
    if not rclpy.ok():
        rclpy.init(args=[])
    node = rclpy.create_node("tinker_sim_vision_roundtrip_probe")
    client = node.create_client(srv_module.GetImage, SERVICE_NAME)
    assert client.wait_for_service(timeout_sec=15.0), (
        "get_image_service not available - is `ros2 run vision_util get_image` "
        "running on this ROS_DOMAIN_ID?"
    )
    yield node, client
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def _call(node, client, camera: str):
    request = srv_module.GetImage.Request()
    request.camera = camera
    request.depth = True
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=20.0)
    assert future.done(), f"get_image call for {camera!r} timed out"
    return future.result()


def _decode(response, width: int, height: int):
    rgb = response.rgb_image
    depth = response.depth_image
    assert response.status == 0, response.error_msg
    assert rgb.encoding == "rgb8"
    assert (rgb.height, rgb.width) == (height, width)
    assert depth.encoding == "16UC1"
    assert (depth.height, depth.width) == (height, width)
    # One capture -> one stamp: the pair the synchronizer matched is exact.
    assert (rgb.header.stamp.sec, rgb.header.stamp.nanosec) == (
        depth.header.stamp.sec,
        depth.header.stamp.nanosec,
    )
    image = np.frombuffer(bytes(rgb.data), dtype=np.uint8).reshape(height, width, 3)
    depth_mm = np.frombuffer(bytes(depth.data), dtype=np.uint16).reshape(height, width)
    return image, depth_mm


def _save_evidence(name: str, image, depth_mm, extra: dict) -> None:
    from PIL import Image as PilImage

    EVIDENCE_DIR.mkdir(parents=True, exist_ok=True)
    PilImage.fromarray(image, mode="RGB").save(EVIDENCE_DIR / f"{name}-color.png")
    normalized = np.clip(depth_mm.astype(np.float64) / 8000.0 * 255.0, 0, 255)
    PilImage.fromarray(normalized.astype(np.uint8), mode="L").save(
        EVIDENCE_DIR / f"{name}-depth.png"
    )
    (EVIDENCE_DIR / f"{name}.json").write_text(
        json.dumps(extra, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


def test_head_camera_round_trip(get_image_client) -> None:
    node, client = get_image_client
    image, depth_mm = _decode(_call(node, client, "orbbec"), 1280, 720)
    valid = depth_mm[depth_mm > 0]
    assert valid.size > 10_000, "head depth is almost entirely invalid"
    assert 100 < float(np.median(valid)) < 40_000, "head depth scale implausible"
    report = hue_presence(image)
    present = sorted(
        name for name, stats in report["colors"].items() if stats["present"]
    )
    # The head camera sees a subset of the colored arena from the robot pose;
    # several distinct palette hues prove authored-scene -> vision-node flow.
    assert len(present) >= 3, report
    _save_evidence(
        "head", image, depth_mm, {"hues_present": present, "report": report}
    )


def test_wrist_camera_round_trip(get_image_client) -> None:
    node, client = get_image_client
    image, depth_mm = _decode(_call(node, client, "realsense"), 848, 480)
    valid = depth_mm[depth_mm > 0]
    assert valid.size > 5_000, "wrist depth is almost entirely invalid"
    assert 50 < float(np.median(valid)) < 40_000, "wrist depth scale implausible"
    _save_evidence("wrist", image, depth_mm, {"note": "wrist pose has no hue gate"})
```

- [x] **Step 2: Verify offline skip behavior**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q tests/ros_humble/test_vision_get_image_live.py`
Expected: `1 skipped` (module-level skip; no Humble needed)

- [x] **Step 3: Run the live round-trip.** Terminal A: sim as in Task 9 Step 1. Terminal B:

```bash
source /opt/ros/humble/setup.bash
source /home/tinker/tk25_ws/install/setup.bash
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 run vision_util get_image
```

Terminal C (same env as B):

```bash
cd /home/tinker/tinker-sim/6.0.1
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
TINKER_SIM_VISION_LIVE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  python3 -m pytest -v tests/ros_humble/test_vision_get_image_live.py
```

Expected: `2 passed`; PNG + JSON evidence under `reports/vision-roundtrip/`.

- [x] **Step 4: If the head hue gate fails**, open `reports/vision-roundtrip/head-color.png` first — diagnose (camera aimed at floor/ceiling → orientation bug in Task 6; all-gray → `--arena-colors` not applied → Task 5/8 wiring) before touching thresholds. Thresholds move only with a written justification in the commit message.

- [x] **Step 5: Also run once with `--camera-pointcloud`** and verify from Terminal B: `ros2 topic info /camera/depth_registered/points --verbose` (RELIABLE), `ros2 topic echo /camera/depth_registered/points --once --field point_step` prints `16`. Stop everything afterward.

- [x] **Step 6: Commit**

```bash
git add tests/ros_humble/test_vision_get_image_live.py
git commit -m "test: live get_image round-trip acceptance for sim cameras"
```

---

### Task 11: Contract docs, README, module status

**Files:**
- Modify: `integration/modules.json` (vision entry)
- Modify: `README.md` (broken sensor-rich example ~line 120; new vision subsection near the navigation integration docs)
- Modify: `docs/superpowers/plans/2026-08-16-sim-vision-stack.md` (check boxes)

**Interfaces:** none — documentation of what Tasks 1–10 built and proved.

- [x] **Step 1: Update `integration/modules.json` vision entry** to:

```json
    "vision": {
      "mode": "gateway_published_rtx_camera_topics",
      "truth_input": false,
      "status": "development_validated_live_get_image_roundtrip",
      "release_blockers": ["pending_live_qualification"]
    },
```

- [x] **Step 2: Fix the README.** Replace the broken example (`./scripts/launch-isaac --sensor-profile sensor-rich --profile parity \ --scenario find-and-approach-person --seed 7`) with:

```bash
./scripts/launch-isaac --sensor-profile sensor-rich --profile parity \
  --scenario empty --seed 7 --ros
```

Add a `## Vision hardware-parity cameras` subsection containing: the topic table (7 topics + optional cloud), the QoS/encoding contract with the one-line rationale (RELIABLE because every tk26_vision CameraInfo subscription is RELIABLE), the `--camera-pointcloud` / `--arena-colors` flags, the measured achieved rate from Task 9, and the three-terminal acceptance runbook from Task 10 verbatim, noting `ROS_DOMAIN_ID=42` isolation and that `reports/vision-roundtrip/` holds the recorded evidence. State plainly: development-validated, not release-qualified.

- [x] **Step 3: Full verification**

Run: `python3 -m unittest discover -s tests -v 2>&1 | tail -3 && uv lock --check`
Expected: OK / lock up to date

- [x] **Step 4: Check all plan checkboxes, commit**

```bash
git add integration/modules.json README.md docs/superpowers/plans/2026-08-16-sim-vision-stack.md
git commit -m "docs: vision camera contract, runbook, and module status"
```
