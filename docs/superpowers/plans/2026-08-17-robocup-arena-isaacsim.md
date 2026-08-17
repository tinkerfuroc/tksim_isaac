# RoboCup 2026 Arena in Isaac Sim — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Import the TeamSOBITS `rcw2026` RoboCup@Home arena (walls + 22 GLB furniture models) and a YCB task-object subset as content-addressed artifacts, with an opt-in `--arena` path through backend/launcher and a scenario arena declaration.

**Architecture:** Pure-Python parsing/rasterizing/publication modules inside the existing `tools/tinker_sim_deploy` package (unit-tested, no GPU), a Kit-dependent conversion adapter (live-verified), and thin CLI orchestrators. Arena artifact = `arena.usd` + per-furniture USDs + derived `map.yaml`/`map.pgm` + `placement.json` + provenance, published with the same atomic content-addressed discipline as the robot artifact. Everything is strictly additive: no existing behavior changes when `--arena` is absent.

**Tech Stack:** Python 3 stdlib (ElementTree, hashlib, json), `unittest` (system Python), `pxr`/`omni.kit.asset_converter` (Kit, live-only), git (pinned upstream clone at import time).

**Spec:** `docs/superpowers/specs/2026-08-17-robocup-arena-isaacsim-design.md`

## Global Constraints

- Strictly additive: with `--arena` absent and `arena_artifact=None`, existing behavior is byte-for-byte unchanged. The Arena 3 map and robot artifact are never touched.
- Unit tests run with the Ubuntu **system Python**: `python3 -m unittest tests.test_<name> -v` (repo convention; no pytest, no GPU, no Isaac imports at module level in tested code paths).
- Kit/pxr imports only inside functions of the conversion adapter (`arena_convert.py`) and importer CLIs; never at module import time.
- Determinism: artifacts contain no timestamps; identities derive only from content hashes + algorithm version; re-running an importer against an unchanged pin is a no-op or byte-identical.
- Fail closed everywhere: unknown model URIs, missing pins, missing files, mismatched declarations, and both-set argument conflicts are errors, never warnings.
- Map raster: resolution `0.05`, `mode: trinary`, `negate: 0`, `occupied_thresh: 0.65`, `free_thresh: 0.25` (matches the existing robot-artifact `map.yaml`); PGM is binary `P5`, max 255, occupied pixel value 0, free 254, top pixel row = highest world y (see `simulation/tinker_sim_core/occupancy.py:from_pgm`, which treats value ≤ 100 as occupied and flips rows).
- Licensing: arena artifact `ATTRIBUTION.md` embeds upstream SOBITS `LICENSE` bytes (BSD-3-Clause); YCB artifact embeds a CC BY 4.0 attribution block naming the Yale-CMU-Berkeley Object and Model Set plus Toyota's Clear BSD wrapper. `gz_human_sim` content is never imported.
- `bounds_tolerance_m` default `0.01` for converted-USD vs SDF-declared bounds self-checks.
- Commit after every task with the trailer `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`.
- Test import bootstrap: before writing any test, open `tests/test_artifact_export.py` (first ~20 lines) and copy its exact import/sys.path pattern for reaching `tools.tinker_sim_deploy.*` and `tinker_sim_core.*`; use the same pattern in every new test file.

## File Structure

| File | Responsibility |
| --- | --- |
| `tools/tinker_sim_deploy/arena_world.py` (new) | Parse the SOBITS world XML → `ArenaLayout` (walls + furniture placements); parse `model.sdf` colliders |
| `tools/tinker_sim_deploy/arena_map.py` (new) | Livox scan height from robot URDF; rasterize layout → `map.pgm` + `map.yaml` bytes |
| `tools/tinker_sim_deploy/arena_surfaces.py` (new) | Placement-surface world-frame math → `placement.json` bytes |
| `tools/tinker_sim_deploy/arena_artifact.py` (new) | Generic content-addressed asset-artifact publication + verification + `ATTRIBUTION.md` |
| `tools/tinker_sim_deploy/assets.py` (modify) | Optional `generated_arena_usds` / `generated_object_usds` manifest groups |
| `tools/tinker_sim_deploy/arena_convert.py` (new) | Kit adapter: GLB→USD, collider authoring, `arena.usd` composition, bounds measurement (live-only) |
| `tools/arena_import.py` (new) | Arena importer CLI: clone pin → parse → convert → raster → publish |
| `tools/ycb_import.py` (new) | YCB importer CLI: clone pin → convert DAE/STL → publish |
| `config/arena-import.json` (new) | Pinned arena source + allowlist + surfaces config |
| `config/ycb-import.json` (new) | Pinned YCB source + object allowlist |
| `simulation/tinker_sim_core/scenario.py` (modify) | `validate_world_selection(scenario, arena_id)` |
| `simulation/tinker_sim_isaac/backend.py` (modify) | `resolve_arena_inputs()` + `arena_artifact` constructor path |
| `validation/run_sim.py` (modify) | `--arena` flag, `resolve_arena_artifact()`, wiring + scenario validation |
| `README.md` (modify) | Arena import + usage documentation |
| Tests | `tests/test_arena_world.py`, `tests/test_arena_map.py`, `tests/test_arena_surfaces.py`, `tests/test_arena_artifact.py`, `tests/test_assets.py` (extend), `tests/test_scenario_arena.py`, `tests/test_backend_arena_inputs.py`, `tests/test_run_sim_arena_cli.py`, `tests/test_arena_import_cli.py`, `tests/test_ycb_import_cli.py` |

---

### Task 1: Arena world parser (`arena_world.py`)

**Files:**
- Create: `tools/tinker_sim_deploy/arena_world.py`
- Test: `tests/test_arena_world.py`

**Interfaces:**
- Consumes: nothing (stdlib only).
- Produces (used by Tasks 2, 3, 9):

```python
@dataclass(frozen=True)
class WallBox:
    name: str
    size: tuple[float, float, float]     # metres, x/y/z
    center: tuple[float, float, float]   # world frame
    yaw: float                           # radians

@dataclass(frozen=True)
class FurniturePlacement:
    model_id: str                        # e.g. "rcw26_kitchen_table"
    position: tuple[float, float, float]
    yaw: float
    static: bool

@dataclass(frozen=True)
class BoxCollider:
    size: tuple[float, float, float]
    center: tuple[float, float, float]   # model-local frame
    yaw: float

@dataclass(frozen=True)
class MeshCollider:
    uri: str                             # mesh URI as written in the SDF

@dataclass(frozen=True)
class ArenaLayout:
    walls: tuple[WallBox, ...]
    furniture: tuple[FurniturePlacement, ...]

class ArenaWorldError(RuntimeError): ...

def parse_world(xml_bytes: bytes, model_allowlist: frozenset[str]) -> ArenaLayout
def parse_model_colliders(sdf_bytes: bytes) -> tuple[BoxCollider | MeshCollider, ...]
```

- [ ] **Step 1: Write the failing tests**

`tests/test_arena_world.py` (import bootstrap per Global Constraints). Fixture world snippet is embedded in the test module as a constant — it mirrors the real `rcw2026_arena.world.xacro` shapes: inline wall model with box-collision links, `<include>` with `model://` URI and 6-tuple pose, and a static flag.

```python
FIXTURE_WORLD = b"""<?xml version="1.0"?>
<sdf version="1.10">
  <world name="rcw2026_arena">
    <model name="arena_walls">
      <static>true</static>
      <pose>1 0 0 0 0 0</pose>
      <link name="wall_north">
        <pose>0 4.5 0.6 0 0 0</pose>
        <collision name="c"><geometry><box><size>9 0.1 1.2</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>9 0.1 1.2</size></box></geometry></visual>
      </link>
    </model>
    <include>
      <uri>model://rcw26_kitchen_table</uri>
      <name>kitchen_table</name>
      <pose>2.0 1.0 0 0 0 1.5707963267948966</pose>
      <static>true</static>
    </include>
  </world>
</sdf>"""

FIXTURE_MODEL_SDF = b"""<?xml version="1.0"?>
<sdf version="1.10">
  <model name="rcw26_kitchen_table">
    <link name="body">
      <collision name="top">
        <pose>0 0 0.72 0 0 0</pose>
        <geometry><box><size>1.2 0.6 0.04</size></box></geometry>
      </collision>
      <collision name="mesh_part">
        <geometry><mesh><uri>meshes/rcw26_kitchen_table.glb</uri></mesh></geometry>
      </collision>
    </link>
  </model>
</sdf>"""


class ParseWorldTest(unittest.TestCase):
    def test_walls_compose_model_and_link_pose(self):
        layout = arena_world.parse_world(
            FIXTURE_WORLD, frozenset({"rcw26_kitchen_table"})
        )
        self.assertEqual(len(layout.walls), 1)
        wall = layout.walls[0]
        self.assertEqual(wall.size, (9.0, 0.1, 1.2))
        # model pose (1,0,0) + link pose (0,4.5,0.6)
        self.assertEqual(wall.center, (1.0, 4.5, 0.6))
        self.assertEqual(wall.yaw, 0.0)

    def test_furniture_include_parsed(self):
        layout = arena_world.parse_world(
            FIXTURE_WORLD, frozenset({"rcw26_kitchen_table"})
        )
        self.assertEqual(len(layout.furniture), 1)
        item = layout.furniture[0]
        self.assertEqual(item.model_id, "rcw26_kitchen_table")
        self.assertEqual(item.position, (2.0, 1.0, 0.0))
        self.assertAlmostEqual(item.yaw, 1.5707963267948966)
        self.assertTrue(item.static)

    def test_unknown_model_fails_closed(self):
        with self.assertRaises(arena_world.ArenaWorldError):
            arena_world.parse_world(FIXTURE_WORLD, frozenset({"rcw26_sofa"}))

    def test_nonzero_roll_pitch_fails_closed(self):
        bad = FIXTURE_WORLD.replace(b"0 0 1.5707963267948966", b"0.3 0 0")
        with self.assertRaises(arena_world.ArenaWorldError):
            arena_world.parse_world(bad, frozenset({"rcw26_kitchen_table"}))

    def test_model_colliders(self):
        colliders = arena_world.parse_model_colliders(FIXTURE_MODEL_SDF)
        self.assertEqual(len(colliders), 2)
        box = colliders[0]
        self.assertEqual(box.size, (1.2, 0.6, 0.04))
        self.assertEqual(box.center, (0.0, 0.0, 0.72))
        self.assertIsInstance(colliders[1], arena_world.MeshCollider)
        self.assertEqual(colliders[1].uri, "meshes/rcw26_kitchen_table.glb")
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python3 -m unittest tests.test_arena_world -v`
Expected: FAIL/ERROR with `ModuleNotFoundError` or `AttributeError` (module does not exist yet).

- [ ] **Step 3: Implement `arena_world.py`**

```python
"""Parse the pinned SOBITS arena world XML into a neutral layout model.

The upstream ``.world.xacro`` files are plain XML (upstream itself parses
them with ElementTree); no xacro processing is performed or supported.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass

_POSE_EPSILON = 1e-6


class ArenaWorldError(RuntimeError):
    pass


# ... dataclasses exactly as in the Interfaces block above ...


def _parse_pose(text: str | None, label: str) -> tuple[float, float, float, float]:
    """Return (x, y, z, yaw); fail closed on non-zero roll/pitch."""
    if text is None or not text.strip():
        return (0.0, 0.0, 0.0, 0.0)
    parts = text.split()
    if len(parts) != 6:
        raise ArenaWorldError(f"{label}: pose must have 6 values, got {text!r}")
    x, y, z, roll, pitch, yaw = (float(part) for part in parts)
    if abs(roll) > _POSE_EPSILON or abs(pitch) > _POSE_EPSILON:
        raise ArenaWorldError(
            f"{label}: non-zero roll/pitch is not supported by the importer"
        )
    return (x, y, z, yaw)


def parse_world(xml_bytes: bytes, model_allowlist: frozenset[str]) -> ArenaLayout:
    try:
        root = ET.fromstring(xml_bytes)
    except ET.ParseError as error:
        raise ArenaWorldError(f"world XML is not parseable: {error}") from error
    world = root.find("world")
    if world is None:
        raise ArenaWorldError("world element missing")
    walls: list[WallBox] = []
    furniture: list[FurniturePlacement] = []
    for model in world.findall("model"):
        model_name = model.get("name", "model")
        mx, my, mz, myaw = _parse_pose(model.findtext("pose"), model_name)
        for link in model.findall("link"):
            link_name = link.get("name", "link")
            label = f"{model_name}/{link_name}"
            size_text = link.findtext("collision/geometry/box/size")
            if size_text is None:
                raise ArenaWorldError(f"{label}: wall link without box collision")
            sx, sy, sz = (float(part) for part in size_text.split())
            lx, ly, lz, lyaw = _parse_pose(link.findtext("pose"), label)
            if abs(myaw) > _POSE_EPSILON:
                cos_y, sin_y = math.cos(myaw), math.sin(myaw)
                lx, ly = mx + cos_y * lx - sin_y * ly, my + sin_y * lx + cos_y * ly
            else:
                lx, ly = mx + lx, my + ly
            walls.append(
                WallBox(name=label, size=(sx, sy, sz),
                        center=(lx, ly, mz + lz), yaw=myaw + lyaw)
            )
    for include in world.findall("include"):
        uri = (include.findtext("uri") or "").strip()
        if not uri.startswith("model://"):
            raise ArenaWorldError(f"include with unsupported uri: {uri!r}")
        model_id = uri[len("model://"):]
        if model_id not in model_allowlist:
            raise ArenaWorldError(f"model not on the import allowlist: {model_id}")
        x, y, z, yaw = _parse_pose(include.findtext("pose"), model_id)
        static = (include.findtext("static") or "false").strip().lower() == "true"
        furniture.append(
            FurniturePlacement(model_id=model_id, position=(x, y, z),
                               yaw=yaw, static=static)
        )
    if not walls:
        raise ArenaWorldError("world contains no wall links")
    return ArenaLayout(walls=tuple(walls), furniture=tuple(furniture))


def parse_model_colliders(sdf_bytes: bytes) -> tuple[BoxCollider | MeshCollider, ...]:
    try:
        root = ET.fromstring(sdf_bytes)
    except ET.ParseError as error:
        raise ArenaWorldError(f"model SDF is not parseable: {error}") from error
    colliders: list[BoxCollider | MeshCollider] = []
    for collision in root.iter("collision"):
        box_size = collision.findtext("geometry/box/size")
        mesh_uri = collision.findtext("geometry/mesh/uri")
        if box_size is not None:
            sx, sy, sz = (float(part) for part in box_size.split())
            cx, cy, cz, cyaw = _parse_pose(
                collision.findtext("pose"), collision.get("name", "collision")
            )
            colliders.append(
                BoxCollider(size=(sx, sy, sz), center=(cx, cy, cz), yaw=cyaw)
            )
        elif mesh_uri is not None:
            colliders.append(MeshCollider(uri=mesh_uri.strip()))
        else:
            raise ArenaWorldError("collision with unsupported geometry")
    return tuple(colliders)
```

Note for the implementer: the real `rcw2026_arena.world.xacro` may nest wall links differently (several models, or extra non-wall inline models such as ground planes). During Task 9's live run, extend `parse_world` only via new tests added here first — e.g., a skip-list parameter for known non-wall inline models — never by loosening the fail-closed default.

- [ ] **Step 4: Run tests to verify they pass**

Run: `python3 -m unittest tests.test_arena_world -v`
Expected: all 5 PASS.

- [ ] **Step 5: Run the full non-GPU suite to prove no regression**

Run: `python3 -m unittest discover -s tests -v 2>&1 | tail -5`
Expected: same pass/fail set as before this task (record the baseline first if unsure).

- [ ] **Step 6: Commit**

```bash
git add tools/tinker_sim_deploy/arena_world.py tests/test_arena_world.py
git commit -m "feat: arena world XML parser for SOBITS import"
```

---

### Task 2: Scan height + map rasterizer (`arena_map.py`)

**Files:**
- Create: `tools/tinker_sim_deploy/arena_map.py`
- Test: `tests/test_arena_map.py`

**Interfaces:**
- Consumes: `ArenaLayout`, `WallBox`, `BoxCollider`, `FurniturePlacement` from `tools.tinker_sim_deploy.arena_world` (Task 1).
- Produces (used by Task 9):

```python
class ArenaMapError(RuntimeError): ...

def livox_scan_height(robot_urdf_bytes: bytes) -> float
    # z of joint name="livox_joint" <origin xyz>; ArenaMapError if absent

def rasterize(
    layout: ArenaLayout,
    furniture_box_colliders: Mapping[str, tuple[BoxCollider, ...]],
    *,
    scan_height: float,
    resolution: float = 0.05,
    padding_m: float = 0.5,
) -> tuple[bytes, bytes]   # (pgm_bytes, map_yaml_bytes)
```

`furniture_box_colliders` maps *include order index as string* — no: it maps `FurniturePlacement.model_id` → box colliders in the model-local frame. Furniture whose colliders are mesh-only must be given a measured AABB `BoxCollider` by the caller (Task 9 does this from converted USD bounds); `rasterize` itself only consumes boxes.

- [ ] **Step 1: Write the failing tests**

```python
URDF_SNIPPET = b"""<robot name="t">
  <joint name="livox_joint" type="fixed">
    <parent link="base_link"/><child link="livox_frame"/>
    <origin rpy="0 0 0" xyz="0.09 0 0.195"/>
  </joint>
</robot>"""


class ScanHeightTest(unittest.TestCase):
    def test_reads_livox_joint_z(self):
        self.assertAlmostEqual(arena_map.livox_scan_height(URDF_SNIPPET), 0.195)

    def test_missing_joint_fails(self):
        with self.assertRaises(arena_map.ArenaMapError):
            arena_map.livox_scan_height(b"<robot name='t'/>")


def _square_room() -> arena_world.ArenaLayout:
    walls = []
    for name, center, size in (
        ("n", (0.0, 2.0, 0.6), (4.1, 0.1, 1.2)),
        ("s", (0.0, -2.0, 0.6), (4.1, 0.1, 1.2)),
        ("e", (2.0, 0.0, 0.6), (0.1, 4.1, 1.2)),
        ("w", (-2.0, 0.0, 0.6), (0.1, 4.1, 1.2)),
    ):
        walls.append(arena_world.WallBox(name=name, size=size, center=center, yaw=0.0))
    table = arena_world.FurniturePlacement(
        model_id="rcw26_kitchen_table", position=(1.0, 1.0, 0.0), yaw=0.0, static=True
    )
    return arena_world.ArenaLayout(walls=tuple(walls), furniture=(table,))


class RasterizeTest(unittest.TestCase):
    COLLIDERS = {
        "rcw26_kitchen_table": (
            # tabletop: entirely ABOVE the 0.195 scan plane, must not mark cells
            arena_world.BoxCollider(size=(0.8, 0.4, 0.04), center=(0.0, 0.0, 0.72), yaw=0.0),
            # leg/body box spanning z [0.0, 0.70]: crosses the scan plane, marks cells
            arena_world.BoxCollider(size=(0.7, 0.3, 0.70), center=(0.0, 0.0, 0.35), yaw=0.0),
        )
    }

    def _load(self, colliders=None):
        pgm, yaml_bytes = arena_map.rasterize(
            _square_room(), colliders if colliders is not None else self.COLLIDERS,
            scan_height=0.195,
        )
        import tempfile, pathlib
        with tempfile.TemporaryDirectory() as tmp:
            base = pathlib.Path(tmp)
            (base / "map.pgm").write_bytes(pgm)
            (base / "map.yaml").write_bytes(yaml_bytes)
            meta = {}
            for line in yaml_bytes.decode().splitlines():
                key, _, value = line.partition(":")
                meta[key.strip()] = value.strip()
            self.assertEqual(meta["image"], "map.pgm")
            self.assertEqual(meta["resolution"], "0.05")
            origin = json.loads(meta["origin"])
            return OccupancyMap.from_pgm(
                base / "map.pgm", resolution=0.05,
                origin_x=origin[0], origin_y=origin[1],
            )

    def test_walls_table_and_free_space(self):
        grid = self._load()
        self.assertTrue(grid.occupied_at_world(0.0, 2.0))    # north wall
        self.assertTrue(grid.occupied_at_world(1.0, 1.0))    # table top slice
        self.assertFalse(grid.occupied_at_world(0.0, 0.0))   # centre is free
        self.assertFalse(grid.occupied_at_world(-1.5, -1.5)) # corner floor free

    def test_outside_scan_plane_not_marked(self):
        for center_z, size_z in ((0.05, 0.10), (0.72, 0.04)):  # plinth below, top above
            colliders = {
                "rcw26_kitchen_table": (
                    arena_world.BoxCollider(size=(0.8, 0.4, size_z), center=(0.0, 0.0, center_z), yaw=0.0),
                )
            }
            grid = self._load(colliders)
            self.assertFalse(grid.occupied_at_world(1.0, 1.0))
```

(Import `OccupancyMap` from `tinker_sim_core.occupancy` using the same bootstrap the tests use for `tinker_sim_core` elsewhere — see `tests/test_command_mux.py` header if `test_artifact_export.py` does not import it.)

- [ ] **Step 2: Run tests to verify they fail**

Run: `python3 -m unittest tests.test_arena_map -v`
Expected: FAIL (`arena_map` missing).

- [ ] **Step 3: Implement `arena_map.py`**

```python
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from typing import Mapping

from .arena_world import ArenaLayout, BoxCollider

_OCCUPIED = 0
_FREE = 254


class ArenaMapError(RuntimeError):
    pass


def livox_scan_height(robot_urdf_bytes: bytes) -> float:
    root = ET.fromstring(robot_urdf_bytes)
    for joint in root.iter("joint"):
        if joint.get("name") == "livox_joint":
            origin = joint.find("origin")
            if origin is None or not origin.get("xyz"):
                break
            return float(origin.get("xyz").split()[2])
    raise ArenaMapError("livox_joint origin not found in robot URDF")


def _footprints_at(
    layout: ArenaLayout,
    colliders: Mapping[str, tuple[BoxCollider, ...]],
    scan_height: float,
) -> list[tuple[float, float, float, float, float]]:
    """(cx, cy, half_x, half_y, yaw) rectangles that intersect the scan plane."""
    rects = []
    for wall in layout.walls:
        z_low = wall.center[2] - wall.size[2] / 2.0
        z_high = wall.center[2] + wall.size[2] / 2.0
        if z_low <= scan_height <= z_high:
            rects.append((wall.center[0], wall.center[1],
                          wall.size[0] / 2.0, wall.size[1] / 2.0, wall.yaw))
    for item in layout.furniture:
        for box in colliders.get(item.model_id, ()):  
            world_z = item.position[2] + box.center[2]
            if not (world_z - box.size[2] / 2.0 <= scan_height <= world_z + box.size[2] / 2.0):
                continue
            cos_y, sin_y = math.cos(item.yaw), math.sin(item.yaw)
            cx = item.position[0] + cos_y * box.center[0] - sin_y * box.center[1]
            cy = item.position[1] + sin_y * box.center[0] + cos_y * box.center[1]
            rects.append((cx, cy, box.size[0] / 2.0, box.size[1] / 2.0,
                          item.yaw + box.yaw))
    if not rects:
        raise ArenaMapError("no geometry intersects the scan plane")
    return rects


def rasterize(
    layout: ArenaLayout,
    furniture_box_colliders: Mapping[str, tuple[BoxCollider, ...]],
    *,
    scan_height: float,
    resolution: float = 0.05,
    padding_m: float = 0.5,
) -> tuple[bytes, bytes]:
    rects = _footprints_at(layout, furniture_box_colliders, scan_height)
    corner_xs: list[float] = []
    corner_ys: list[float] = []
    for cx, cy, hx, hy, yaw in rects:
        for sx in (-1.0, 1.0):
            for sy in (-1.0, 1.0):
                corner_xs.append(cx + sx * hx * abs(math.cos(yaw)) + sy * hy * abs(math.sin(yaw)))
                corner_ys.append(cy + sx * hx * abs(math.sin(yaw)) + sy * hy * abs(math.cos(yaw)))
    origin_x = math.floor((min(corner_xs) - padding_m) / resolution) * resolution
    origin_y = math.floor((min(corner_ys) - padding_m) / resolution) * resolution
    width = int(math.ceil((max(corner_xs) + padding_m - origin_x) / resolution))
    height = int(math.ceil((max(corner_ys) + padding_m - origin_y) / resolution))

    def cell_occupied(x: float, y: float) -> bool:
        for cx, cy, hx, hy, yaw in rects:
            dx, dy = x - cx, y - cy
            cos_y, sin_y = math.cos(-yaw), math.sin(-yaw)
            lx = cos_y * dx - sin_y * dy
            ly = sin_y * dx + cos_y * dy
            if abs(lx) <= hx and abs(ly) <= hy:
                return True
        return False

    rows = bytearray()
    for row in range(height):                      # top row = highest world y
        world_y = origin_y + (height - row - 0.5) * resolution
        for col in range(width):
            world_x = origin_x + (col + 0.5) * resolution
            rows.append(_OCCUPIED if cell_occupied(world_x, world_y) else _FREE)
    pgm = b"P5\n%d %d\n255\n" % (width, height) + bytes(rows)
    map_yaml = (
        "image: map.pgm\n"
        "mode: trinary\n"
        f"resolution: {resolution}\n"
        f"origin: [{origin_x}, {origin_y}, 0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.25\n"
    ).encode("utf-8")
    return pgm, map_yaml
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python3 -m unittest tests.test_arena_map -v`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add tools/tinker_sim_deploy/arena_map.py tests/test_arena_map.py
git commit -m "feat: arena occupancy-map rasterizer with scan-plane slice"
```

---

### Task 3: Placement surfaces (`arena_surfaces.py`)

**Files:**
- Create: `tools/tinker_sim_deploy/arena_surfaces.py`
- Test: `tests/test_arena_surfaces.py`

**Interfaces:**
- Consumes: `FurniturePlacement` from `arena_world` (Task 1).
- Produces (used by Task 9):

```python
@dataclass(frozen=True)
class PlacementSurface:
    surface_id: str        # "<furniture short name>#<surface name>"
    furniture_id: str
    center_xyz: tuple[float, float, float]   # world frame
    size_xy: tuple[float, float]
    yaw: float
    edge_margin: float

def world_surface(
    furniture: FurniturePlacement,
    *,
    surface_name: str,
    local_center: tuple[float, float, float],
    size_xy: tuple[float, float],
    edge_margin: float,
) -> PlacementSurface

def placement_json(arena_id: str, surfaces: Sequence[PlacementSurface]) -> bytes
    # canonical JSON: sort_keys=True, separators=(",", ":"), schema per spec §6
```

- [ ] **Step 1: Write the failing tests**

```python
class WorldSurfaceTest(unittest.TestCase):
    def test_rotation_composition(self):
        furniture = arena_world.FurniturePlacement(
            model_id="rcw26_kitchen_table", position=(1.0, 2.0, 0.0),
            yaw=math.pi / 2.0, static=True,
        )
        surface = arena_surfaces.world_surface(
            furniture, surface_name="top",
            local_center=(0.5, 0.0, 0.74), size_xy=(1.2, 0.6), edge_margin=0.05,
        )
        self.assertAlmostEqual(surface.center_xyz[0], 1.0)
        self.assertAlmostEqual(surface.center_xyz[1], 2.5)
        self.assertAlmostEqual(surface.center_xyz[2], 0.74)
        self.assertAlmostEqual(surface.yaw, math.pi / 2.0)
        self.assertEqual(surface.surface_id, "kitchen_table#top")

    def test_placement_json_is_canonical_and_schema1(self):
        surface = ...  # as above
        payload = json.loads(arena_surfaces.placement_json("rcw2026", [surface]))
        self.assertEqual(payload["schema_version"], 1)
        self.assertEqual(payload["arena_id"], "rcw2026")
        self.assertEqual(len(payload["surfaces"]), 1)
        record = payload["surfaces"][0]
        self.assertEqual(
            set(record),
            {"surface_id", "furniture_id", "center_xyz", "size_xy", "yaw", "edge_margin"},
        )
        # byte-determinism
        self.assertEqual(
            arena_surfaces.placement_json("rcw2026", [surface]),
            arena_surfaces.placement_json("rcw2026", [surface]),
        )
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python3 -m unittest tests.test_arena_surfaces -v` — expected FAIL.

- [ ] **Step 3: Implement**

`surface_id` strips a leading `rcw26_` from `furniture_id` and appends `#<surface_name>`. `world_surface` composes `local_center` through the furniture yaw/translation exactly like the collider transform in Task 2. `placement_json` builds `{"schema_version": 1, "arena_id": ..., "surfaces": [...]}` with surfaces sorted by `surface_id`, dumped `json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()`.

- [ ] **Step 4: Run tests to verify they pass** — `python3 -m unittest tests.test_arena_surfaces -v`

- [ ] **Step 5: Commit**

```bash
git add tools/tinker_sim_deploy/arena_surfaces.py tests/test_arena_surfaces.py
git commit -m "feat: placement surface records and canonical placement.json"
```

---

### Task 4: Content-addressed asset artifact publication (`arena_artifact.py`)

**Files:**
- Create: `tools/tinker_sim_deploy/arena_artifact.py`
- Test: `tests/test_arena_artifact.py`

**Interfaces:**
- Consumes: `_atomic_write`, `_fsync_directory`, `_publication_lock`, `_recover_staging` from `tools.tinker_sim_deploy.workspace` (already exist; see `workspace.py:122,114,626,642`).
- Produces (used by Tasks 8, 9, 10):

```python
ALGORITHM_VERSION = "arena-artifact-1"

@dataclass(frozen=True)
class AssetPublication:
    artifact_dir: Path
    identity: str          # 64-hex
    created: bool          # False when identical artifact already existed

class AssetArtifactError(RuntimeError): ...

def publish_asset_artifact(
    repo_root: Path,
    *,
    kind: str,             # "arena" | "objects"
    asset_id: str,         # "rcw2026" | "ycb"
    payload: Mapping[str, bytes],       # relpath -> bytes; must include no manifest/source-lock
    source_lock: Mapping[str, object],
) -> AssetPublication
    # writes artifacts/<kind>/<asset_id>/<identity>/{payload..., source-lock.json, manifest.json}
    # then atomically replaces artifacts/<kind>/<asset_id>/current.json

def verify_asset_artifact(artifact_dir: Path) -> list[str]   # [] when clean

def attribution_markdown(sections: Sequence[tuple[str, str]]) -> bytes
    # deterministic "## <title>\n\n<body>\n" concatenation
```

`current.json` shape matches the robot pointer consumed at `validation/run_sim.py:520`: `{"schema_version": 1, "manifest": "artifacts/<kind>/<asset_id>/<identity>/manifest.json"}` (repo-root-relative). `manifest.json` is canonical JSON: `{"schema_version": 1, "kind": ..., "id": ..., "identity": ..., "algorithm": ALGORITHM_VERSION, "payload": {relpath: sha256hex}}`. Identity = sha256 of the canonical JSON of `{"algorithm": ALGORITHM_VERSION, "kind": ..., "id": ..., "payload": {...}, "source_lock_sha256": sha256(canonical source-lock bytes)}`.

- [ ] **Step 1: Write the failing tests**

```python
def _publish(self, tmp_root, payload=None):
    return arena_artifact.publish_asset_artifact(
        tmp_root, kind="arena", asset_id="rcw2026",
        payload=payload or {"arena.usd": b"usd-bytes", "map.yaml": b"image: map.pgm\n"},
        source_lock={"repository": "https://example/repo", "commit": "a" * 40,
                     "records": [{"path": "worlds/x.world.xacro", "size": 3, "sha256": "b" * 64}]},
    )

class PublicationTest(unittest.TestCase):
    def test_publish_creates_immutable_dir_and_pointer(self):
        pub = self._publish(root)
        self.assertRegex(pub.artifact_dir.name, r"^[0-9a-f]{64}$")
        self.assertEqual(pub.artifact_dir.parent,
                         root / "artifacts" / "arena" / "rcw2026")
        self.assertEqual((pub.artifact_dir / "arena.usd").read_bytes(), b"usd-bytes")
        pointer = json.loads(
            (root / "artifacts" / "arena" / "rcw2026" / "current.json").read_text()
        )
        self.assertEqual(
            pointer["manifest"],
            f"artifacts/arena/rcw2026/{pub.identity}/manifest.json",
        )
        manifest = json.loads((pub.artifact_dir / "manifest.json").read_text())
        self.assertEqual(manifest["identity"], pub.identity)

    def test_republish_is_idempotent(self):
        first = self._publish(root); second = self._publish(root)
        self.assertEqual(first.identity, second.identity)
        self.assertTrue(first.created); self.assertFalse(second.created)

    def test_payload_change_changes_identity(self): ...

    def test_crash_before_pointer_leaves_orphan_only(self):
        # unittest.mock.patch arena_artifact._atomic_write to raise on the
        # current.json write: pointer absent afterwards, orphan dir may exist,
        # re-publish succeeds and commits the pointer.

    def test_verify_detects_mutation(self):
        pub = self._publish(root)
        (pub.artifact_dir / "arena.usd").write_bytes(b"tampered")
        self.assertTrue(arena_artifact.verify_asset_artifact(pub.artifact_dir))

    def test_verify_clean(self):
        pub = self._publish(root)
        self.assertEqual(arena_artifact.verify_asset_artifact(pub.artifact_dir), [])

    def test_reserved_payload_names_rejected(self):
        with self.assertRaises(arena_artifact.AssetArtifactError):
            self._publish(root, payload={"manifest.json": b"x"})
```

- [ ] **Step 2: Run tests to verify they fail** — `python3 -m unittest tests.test_arena_artifact -v`

- [ ] **Step 3: Implement**

Implementation outline (follow `workspace.py:653-790` = `export_tinker2` / `_export_tinker2_locked` for the staging/claim/pointer choreography; reuse its helpers rather than re-implementing):

```python
from .workspace import (
    _atomic_write, _fsync_directory, _publication_lock, _recover_staging,
)

_RESERVED = {"manifest.json", "source-lock.json", "current.json"}

def _canonical(value) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":")).encode("utf-8")

def publish_asset_artifact(repo_root, *, kind, asset_id, payload, source_lock):
    for name in payload:
        if name in _RESERVED or name.startswith("/") or ".." in Path(name).parts:
            raise AssetArtifactError(f"invalid payload path: {name}")
    artifact_root = repo_root / "artifacts" / kind / asset_id
    artifact_root.mkdir(parents=True, exist_ok=True)
    lock_bytes = _canonical(source_lock)
    hashes = {name: hashlib.sha256(data).hexdigest() for name, data in payload.items()}
    identity = hashlib.sha256(_canonical({
        "algorithm": ALGORITHM_VERSION, "kind": kind, "id": asset_id,
        "payload": hashes,
        "source_lock_sha256": hashlib.sha256(lock_bytes).hexdigest(),
    })).hexdigest()
    manifest = _canonical({
        "schema_version": 1, "kind": kind, "id": asset_id,
        "identity": identity, "algorithm": ALGORITHM_VERSION, "payload": hashes,
    })
    pointer = _canonical({
        "schema_version": 1,
        "manifest": f"artifacts/{kind}/{asset_id}/{identity}/manifest.json",
    })
    with _publication_lock(artifact_root):
        final_dir = artifact_root / identity
        created = not final_dir.is_dir()
        if created:
            _recover_staging(artifact_root)
            staging = artifact_root / f".staging-{os.getpid()}"
            staging.mkdir()
            for name, data in payload.items():
                target = staging / name
                target.parent.mkdir(parents=True, exist_ok=True)
                _atomic_write(target, data)
            _atomic_write(staging / "source-lock.json", lock_bytes)
            _atomic_write(staging / "manifest.json", manifest)
            os.rename(staging, final_dir)
            _fsync_directory(artifact_root)
        _atomic_write(artifact_root / "current.json", pointer)
    return AssetPublication(artifact_dir=final_dir, identity=identity, created=created)
```

`verify_asset_artifact` loads `manifest.json`, recomputes the identity from payload hashes + `source-lock.json` bytes, re-hashes every payload file on disk, checks the directory name equals the identity, and reports every mismatch as a string in the returned list. `attribution_markdown` joins `f"## {title}\n\n{body.rstrip()}\n"` per section with `\n` and returns UTF-8 bytes.

Check `_recover_staging`'s actual signature/behavior at `workspace.py:642` before use; if its staging naming differs, mirror the robot exporter's staging name pattern instead of `.staging-<pid>`.

- [ ] **Step 4: Run tests to verify they pass** — `python3 -m unittest tests.test_arena_artifact -v`

- [ ] **Step 5: Commit**

```bash
git add tools/tinker_sim_deploy/arena_artifact.py tests/test_arena_artifact.py
git commit -m "feat: content-addressed asset artifact publication"
```

---

### Task 5: Optional asset-manifest groups (`assets.py`)

**Files:**
- Modify: `tools/tinker_sim_deploy/assets.py:29-33`
- Test: extend `tests/test_assets.py`

**Interfaces:**
- Produces: `verify_assets` additionally validates optional groups `generated_arena_usds` and `generated_object_usds` with the same entry schema (`{"path", "sha256"}`), **without** requiring them to be present or non-empty. The two original groups keep their non-empty requirement exactly.

- [ ] **Step 1: Write the failing tests** (extend the existing `tests/test_assets.py`, mirroring its fixture style)

```python
def test_optional_arena_group_validated_when_present(self):
    # manifest with valid required groups + generated_arena_usds entry whose
    # sha256 is wrong -> RuntimeError "asset checksum mismatch"

def test_optional_groups_absent_is_fine(self):
    # manifest without the optional groups still verifies

def test_optional_group_present_but_empty_is_fine(self):
    # empty list for generated_arena_usds does not raise
```

- [ ] **Step 2: Run to verify the new tests fail** — `python3 -m unittest tests.test_assets -v`

- [ ] **Step 3: Implement** — in `verify_assets`, after the existing required-group loop, add:

```python
    for group in ("generated_arena_usds", "generated_object_usds"):
        entries = manifest.get(group)
        if entries is None:
            continue
        if not isinstance(entries, list):
            raise RuntimeError(f"asset manifest group must be a list: {group}")
        for entry in entries:
            # identical per-entry validation to the loop above; factor the
            # entry check into a local function _verify_entries(group, entries)
            # and call it from both loops rather than duplicating it.
            ...
```

- [ ] **Step 4: Run tests** — `python3 -m unittest tests.test_assets -v` — all PASS, including pre-existing ones.

- [ ] **Step 5: Commit**

```bash
git add tools/tinker_sim_deploy/assets.py tests/test_assets.py
git commit -m "feat: optional arena/object asset-manifest groups"
```

---

### Task 6: Scenario world-mode validation (`scenario.py`)

**Files:**
- Modify: `simulation/tinker_sim_core/scenario.py` (append near `load_named_scenario`, `scenario.py:98`)
- Test: `tests/test_scenario_arena.py`

**Interfaces:**
- Produces (used by Task 8):

```python
def validate_world_selection(scenario: ScenarioDefinition, arena_id: str | None) -> None
```

Rules (spec §9): `world.mode` absent or `"current"` → always valid (arena-indifferent, so the streaming viewer's `--scenario empty` keeps working with `--arena`). `mode == "arena"` → `world["arena"]` must be a non-empty string equal to `arena_id`; `arena_id is None` (no `--arena`) or mismatch → `ValueError`. Any other mode → `ValueError`. `world` containing `uri` together with `mode: "arena"` → `ValueError` (the `load_world` path stays unactivated).

- [ ] **Step 1: Write the failing tests**

Build `ScenarioDefinition` instances directly (the dataclass is constructible without file I/O; copy the minimal-fields construction pattern from any existing scenario test, or construct via a temp JSON file + `ScenarioDefinition.load` with `postconditions` present — the loader requires them). Cases:

```python
def test_current_mode_ignores_arena(self):        # world={"mode": "current"}, arena "rcw2026" -> ok
def test_current_mode_without_arena(self):        # arena None -> ok
def test_arena_mode_matching(self):               # {"mode": "arena", "arena": "rcw2026"}, "rcw2026" -> ok
def test_arena_mode_without_launcher_arena(self): # arena None -> ValueError
def test_arena_mode_mismatch(self):               # selected "other" -> ValueError
def test_unknown_mode_rejected(self):             # {"mode": "gazebo"} -> ValueError
def test_arena_mode_with_uri_rejected(self):      # {"mode": "arena", "arena": "x", "uri": "y"} -> ValueError
```

- [ ] **Step 2: Run to verify failure** — `python3 -m unittest tests.test_scenario_arena -v`

- [ ] **Step 3: Implement** (append to `scenario.py`)

```python
def validate_world_selection(scenario: ScenarioDefinition, arena_id: str | None) -> None:
    """Fail closed when a scenario's declared world does not match the launch."""
    mode = scenario.world.get("mode", "current")
    if mode == "current":
        return
    if mode != "arena":
        raise ValueError(f"{scenario.scenario_id}: unsupported world mode {mode!r}")
    if "uri" in scenario.world:
        raise ValueError(f"{scenario.scenario_id}: world mode 'arena' must not carry uri")
    declared = scenario.world.get("arena")
    if not isinstance(declared, str) or not declared:
        raise ValueError(f"{scenario.scenario_id}: world mode 'arena' requires an arena id")
    if arena_id is None:
        raise ValueError(
            f"{scenario.scenario_id}: scenario requires arena {declared!r}; launch with --arena"
        )
    if declared != arena_id:
        raise ValueError(
            f"{scenario.scenario_id}: scenario requires arena {declared!r}, launcher selected {arena_id!r}"
        )
```

- [ ] **Step 4: Run tests** — `python3 -m unittest tests.test_scenario_arena -v`, then the full suite tail check.

- [ ] **Step 5: Commit**

```bash
git add simulation/tinker_sim_core/scenario.py tests/test_scenario_arena.py
git commit -m "feat: scenario arena world-mode declaration validation"
```

---

### Task 7: Backend arena path (`backend.py`)

**Files:**
- Modify: `simulation/tinker_sim_isaac/backend.py` (module-level helper above `IsaacWholeRobotBackend`; constructor `backend.py:90-188`)
- Test: `tests/test_backend_arena_inputs.py`

**Interfaces:**
- Produces (used by Task 8):

```python
def resolve_arena_inputs(
    arena_artifact: Path | None, map_yaml: Path | None
) -> tuple[Path | None, Path | None]:
    # returns (arena_usd, effective_map_yaml)
```

Rules: both set → `ValueError("--arena and --map are mutually exclusive")`-style message. Arena set → `arena_usd = arena_artifact / "arena.usd"`, `effective_map_yaml = arena_artifact / "map.yaml"`; either missing on disk → `FileNotFoundError`. Arena `None` → `(None, map_yaml)` unchanged.

Constructor changes:
- New keyword-only parameter `arena_artifact: Path | None = None` on `IsaacWholeRobotBackend.__init__` (the `IsaacNavigationBackend` name at `backend.py:1133` is an alias of the same class, so one change covers both).
- First lines of `__init__` (before `import torch`): call `resolve_arena_inputs`; also `if arena_artifact is not None and wall_color_fn is not None: raise ValueError(...)`.
- In the map/wall block (`backend.py:166-188`): keep building `self.occupancy` from the effective `map_yaml` exactly as today, but when `arena_artifact` is set, **skip the cuboid spawn loop** and instead reference the arena stage:

```python
        if arena_usd is not None:
            from isaacsim.core.utils.stage import add_reference_to_stage
            add_reference_to_stage(str(arena_usd.resolve()), "/World/Arena")
```

(`self.occupancy` stays populated for truth/raycast; only the visible `/World/NavigationMap` cuboids are replaced by the arena geometry.)

- [ ] **Step 1: Write the failing tests** (pure helper only — the constructor body needs Isaac and is live-verified in Task 9's smoke)

```python
class ResolveArenaInputsTest(unittest.TestCase):
    def test_none_passthrough(self):
        self.assertEqual(backend.resolve_arena_inputs(None, Path("m.yaml")),
                         (None, Path("m.yaml")))

    def test_both_set_rejected(self):
        with self.assertRaises(ValueError):
            backend.resolve_arena_inputs(Path("a"), Path("m.yaml"))

    def test_arena_resolves_colocated_files(self):
        # tmp dir with arena.usd + map.yaml -> returns both paths
    def test_missing_arena_usd_fails(self):
        # tmp dir with only map.yaml -> FileNotFoundError
    def test_missing_map_yaml_fails(self):
        # tmp dir with only arena.usd -> FileNotFoundError
```

Importing `simulation/tinker_sim_isaac/backend.py` at module level must not require Isaac (it already imports only stdlib + `tinker_sim_core.occupancy` at module scope — verify while editing, keep it that way).

- [ ] **Step 2: Run to verify failure** — `python3 -m unittest tests.test_backend_arena_inputs -v`

- [ ] **Step 3: Implement** helper + constructor changes as specified above.

- [ ] **Step 4: Run tests + regression tail**

`python3 -m unittest tests.test_backend_arena_inputs -v`, then `python3 -m unittest discover -s tests -v 2>&1 | tail -5` (backend-importing tests such as `tests/test_chassis_ballast.py` must still pass).

- [ ] **Step 5: Commit**

```bash
git add simulation/tinker_sim_isaac/backend.py tests/test_backend_arena_inputs.py
git commit -m "feat: opt-in arena artifact path in the Isaac backend"
```

---

### Task 8: `--arena` CLI wiring (`run_sim.py`)

**Files:**
- Modify: `validation/run_sim.py` (parser `run_sim.py:402-432`; `_expected_scenario_objects` `run_sim.py:172`; navigation-parity site `run_sim.py:519-531`; the two whole-robot sites near `run_sim.py:678-706` and `run_sim.py:820-836`)
- Test: `tests/test_run_sim_arena_cli.py`

**Interfaces:**
- Consumes: `resolve_arena_inputs` semantics (Task 7 — but run_sim passes `arena_artifact` through and lets the backend resolve), `validate_world_selection` (Task 6).
- Produces: module-level function in `run_sim.py`:

```python
def resolve_arena_artifact(root: Path, arena_id: str) -> Path:
    # reads artifacts/arena/<arena_id>/current.json (json "manifest" key,
    # relative paths resolved against root, mirroring run_sim.py:520-524),
    # returns the manifest's parent directory; FileNotFoundError/ValueError
    # fail closed on missing pointer, missing manifest, or missing
    # arena.usd / map.yaml in that directory.
```

Wiring changes:
1. `parser.add_argument("--arena")` after `--map` (`run_sim.py:413`).
2. Flag rules next to the existing ones (`run_sim.py:422-429`):
   ```python
   if args.arena and args.map_yaml is not None:
       parser.error("--arena and --map are mutually exclusive")
   if args.arena and args.arena_colors:
       parser.error("--arena-colors applies only to occupancy cuboid walls")
   ```
3. `_expected_scenario_objects(root, scenario_name)` → add parameter `arena_id: str | None`; after `scenario = load_named_scenario(...)` insert `validate_world_selection(scenario, arena_id)` (import alongside `load_named_scenario`); the `"empty"` short-circuit stays first. Update both call sites (`run_sim.py:688`, `run_sim.py:822`) to pass `args.arena`.
4. All three backend construction sites: when `args.arena` is set, compute `arena_dir = resolve_arena_artifact(root, args.arena)` and pass `arena_artifact=arena_dir` to the backend; skip the robot-artifact `map.yaml` defaulting (`args.map_yaml` stays `None`; the backend derives the map from the artifact). The `--artifact` robot-USD defaulting is unchanged.

- [ ] **Step 1: Write the failing tests** (import `validation.run_sim` module — its module level has no Isaac imports; verify and keep it that way)

```python
class ResolveArenaArtifactTest(unittest.TestCase):
    def test_resolves_via_pointer(self):
        # tmp root with artifacts/arena/rcw2026/current.json ->
        # {"schema_version":1,"manifest":"artifacts/arena/rcw2026/<id>/manifest.json"},
        # that dir containing arena.usd + map.yaml -> returns the dir
    def test_missing_pointer_fails(self): ...
    def test_missing_payload_fails(self): ...   # pointer ok, arena.usd absent
```

- [ ] **Step 2: Run to verify failure** — `python3 -m unittest tests.test_run_sim_arena_cli -v`

- [ ] **Step 3: Implement** all four wiring changes plus `resolve_arena_artifact`.

- [ ] **Step 4: Run tests + regression tail** — new tests PASS; `python3 -m unittest discover -s tests -v 2>&1 | tail -5` unchanged.

- [ ] **Step 5: Commit**

```bash
git add validation/run_sim.py tests/test_run_sim_arena_cli.py
git commit -m "feat: --arena launch flag resolving the arena artifact"
```

Note: `scripts/launch-arena-streaming` and `scripts/launch-isaac` forward `"$@"`, so `./scripts/launch-arena-streaming --arena rcw2026` works with no script change.

---

### Task 9: Kit conversion adapter + arena importer CLI + live import

**Files:**
- Create: `tools/tinker_sim_deploy/arena_convert.py`, `tools/arena_import.py`, `config/arena-import.json`
- Test: `tests/test_arena_import_cli.py`

**Interfaces:**
- Consumes: Tasks 1–5 modules.
- Produces:

```python
# arena_convert.py — every function imports Kit/pxr lazily; live-only
def convert_glb_to_usd(glb_path: Path, usd_path: Path) -> None
def author_model_colliders(usd_path: Path,
                           colliders: tuple[BoxCollider | MeshCollider, ...]) -> None
def compose_arena(arena_usd: Path, layout: ArenaLayout,
                  furniture_dir_name: str = "furniture") -> None
def measure_bounds(usd_path: Path) -> tuple[tuple[float, float, float],
                                            tuple[float, float, float]]

# tools/arena_import.py
def run_import(config: Mapping[str, object], repo_root: Path, checkout: Path,
               converter: ConverterHooks) -> AssetPublication
    # ConverterHooks is a small protocol/dataclass holding the four
    # arena_convert functions, injectable for tests.
def main(argv: list[str] | None = None) -> int
```

`config/arena-import.json` (committed; `commit` captured in Step 5):

```json
{
  "repository": "https://github.com/TeamSOBITS/sobits_gazebo_worlds",
  "branch": "feature/hri",
  "commit": "<filled in step 5>",
  "world": "worlds/rcw2026_arena.world.xacro",
  "arena_id": "rcw2026",
  "model_allowlist": ["<filled from models/rcw26_* listing in step 5>"],
  "surface_furniture": ["rcw26_kitchen_table", "rcw26_shelf", "rcw26_side_table",
                        "rcw26_laundry_desk", "rcw26_sofa"],
  "surfaces": [],
  "bounds_tolerance_m": 0.01
}
```

`run_import` pipeline (pure orchestration; every Kit call goes through `converter`):
1. Verify `checkout` HEAD equals `config["commit"]` (`git -C <checkout> rev-parse HEAD`), else `AssetArtifactError`.
2. `parse_world` on the world file with `frozenset(model_allowlist)`.
3. Per furniture model: read `models/<id>/model.sdf` → `parse_model_colliders`; `converter.convert_glb_to_usd` → scratch `furniture/<id>.usd`; `converter.author_model_colliders`; `converter.measure_bounds` vs SDF box-collider extents within `bounds_tolerance_m` (mesh-only models: bounds recorded, no box check); mesh-only models get their measured AABB injected as the map-slice `BoxCollider`.
4. `converter.compose_arena` → scratch `arena.usd` (walls as unit cubes scaled to size with gray `UsdPreviewSurface`, kinematic `RigidBodyAPI` + `CollisionAPI`, matching the existing cuboid convention; furniture as references `./furniture/<id>.usd` with translate + rotateZ).
5. `livox_scan_height` from the robot artifact's `robot.urdf` (resolved via `artifacts/robot/tinker2/current.json`), `rasterize` → `map.pgm`/`map.yaml`.
6. `world_surface` per `surfaces` config entry → `placement_json`.
7. `attribution_markdown` (upstream `LICENSE` bytes + per-model source paths/hashes section).
8. `publish_asset_artifact(repo_root, kind="arena", asset_id=config["arena_id"], payload=..., source_lock=...)` where source_lock = repository/branch/commit plus ordered `{path, size, sha256}` records of every consumed upstream file.
9. Print the identity and remind the operator to add `arena.usd` to `generated_arena_usds` in `artifacts/asset-manifest.json` for offline bundling.

`main` parses `--config config/arena-import.json --report-bounds`, clones the pin into the session scratchpad (`git init` + `git fetch <repo> <commit>` + `git checkout FETCH_HEAD` — works for any reachable commit), builds real `ConverterHooks` from `arena_convert`, and runs `run_import`. `--report-bounds` prints per-model measured bounds and exits without publishing (used to fill `surfaces`).

- [ ] **Step 1: Write the failing tests** — orchestration only, with stub hooks:

```python
class StubHooks:
    def convert_glb_to_usd(self, glb, usd):
        usd.parent.mkdir(parents=True, exist_ok=True)
        usd.write_bytes(b"stub-usd:" + glb.name.encode())
    def author_model_colliders(self, usd, colliders): pass
    def compose_arena(self, arena_usd, layout, furniture_dir_name="furniture"):
        arena_usd.write_bytes(b"stub-arena")
    def measure_bounds(self, usd):
        return ((-0.6, -0.3, 0.0), (0.6, 0.3, 0.74))

def test_import_publishes_artifact(self):
    # fake checkout dir: .git via real `git init`+commit of fixture files
    # (worlds/, models/rcw26_kitchen_table/{model.sdf, meshes/x.glb}, LICENSE),
    # config commit = that repo's HEAD; robot URDF fixture for scan height ->
    # run_import returns AssetPublication; artifact dir contains arena.usd,
    # furniture/rcw26_kitchen_table.usd, map.yaml, map.pgm, placement.json,
    # source-lock.json, manifest.json, ATTRIBUTION.md; verify_asset_artifact == []
def test_pin_mismatch_fails_closed(self):
    # config commit "f"*40 vs real HEAD -> AssetArtifactError, nothing published
def test_bounds_violation_fails_closed(self):
    # StubHooks.measure_bounds returning extents 5 cm off the SDF collider
    # with bounds_tolerance_m 0.01 -> AssetArtifactError
```

- [ ] **Step 2: Run to verify failure** — `python3 -m unittest tests.test_arena_import_cli -v`

- [ ] **Step 3: Implement** `tools/arena_import.py` (`run_import`, `main`, `ConverterHooks`) until the stub-hook tests pass. Keep every function under ~60 lines; factor bounds-checking and source-lock record collection into helpers.

- [ ] **Step 4: Implement `arena_convert.py`** (no unit tests — live-verified). Core patterns:

```python
def convert_glb_to_usd(glb_path: Path, usd_path: Path) -> None:
    import asyncio
    import omni.kit.asset_converter as asset_converter

    async def _run() -> None:
        instance = asset_converter.get_instance()
        task = instance.create_converter_task(str(glb_path), str(usd_path))
        if not await task.wait_until_finished():
            raise RuntimeError(
                f"asset conversion failed for {glb_path}: {task.get_error_message()}"
            )
    asyncio.get_event_loop().run_until_complete(_run())


def author_model_colliders(usd_path, colliders) -> None:
    from pxr import Gf, Usd, UsdGeom, UsdPhysics
    stage = Usd.Stage.Open(str(usd_path))
    scope = UsdGeom.Scope.Define(stage, "/Colliders")
    for index, collider in enumerate(colliders):
        if isinstance(collider, MeshCollider):
            # convex hull on the visual mesh prim(s)
            for prim in stage.Traverse():
                if prim.IsA(UsdGeom.Mesh):
                    api = UsdPhysics.MeshCollisionAPI.Apply(prim)
                    api.CreateApproximationAttr("convexHull")
                    UsdPhysics.CollisionAPI.Apply(prim)
            continue
        cube = UsdGeom.Cube.Define(stage, f"/Colliders/box_{index:02d}")
        cube.CreateSizeAttr(1.0)
        xform = UsdGeom.Xformable(cube.GetPrim())
        xform.AddTranslateOp().Set(Gf.Vec3d(*collider.center))
        xform.AddRotateZOp().Set(math.degrees(collider.yaw))
        xform.AddScaleOp().Set(Gf.Vec3f(*collider.size))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        UsdGeom.Imageable(cube.GetPrim()).MakeInvisible()
    stage.Save()
```

`compose_arena`: new stage, `/World/Arena` Xform as default prim; walls as `UsdGeom.Cube` size 1 scaled to `wall.size`, translate to `wall.center`, rotateZ `wall.yaw`, gray `UsdPreviewSurface` material, `UsdPhysics.CollisionAPI` + `UsdPhysics.RigidBodyAPI` with `CreateKinematicEnabledAttr(True)`; furniture prims `stage.DefinePrim(f"/World/Arena/Furniture/{model_id}").GetReferences().AddReference(f"./{furniture_dir_name}/{model_id}.usd")` + translate/rotateZ. `measure_bounds`: `UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"]).ComputeWorldBound(stage.GetDefaultPrim())` aligned range min/max.

These signatures/enums must be validated against Isaac Sim 6.0.1's bundled USD at live-run time; fix in place if an API name differs, and note any fix in the Task 9 commit message.

- [ ] **Step 5: Capture the pin and fill the config (live, network)**

```bash
git ls-remote https://github.com/TeamSOBITS/sobits_gazebo_worlds refs/heads/feature/hri
# -> commit hash into config/arena-import.json "commit"
git ls-remote --exit-code https://github.com/TeamSOBITS/sobits_gazebo_worlds  # sanity
```
Clone the pin into the scratchpad, list `models/rcw26_*` directory names into `model_allowlist`, and inspect `worlds/rcw2026_arena.world.xacro` for inline non-wall models (extend the Task 1 parser via new tests if needed, e.g. ground-plane skip entries).

- [ ] **Step 6: Live import run (Isaac venv, GPU host)**

```bash
./.venv/bin/python tools/arena_import.py --config config/arena-import.json --report-bounds
# fill config "surfaces" entries for each surface_furniture model from the
# reported bounds (local_center/size_xy per flat top; edge_margin 0.05)
./.venv/bin/python tools/arena_import.py --config config/arena-import.json
python3 - <<'EOF'
# verify: load current.json, run verify_asset_artifact, expect []
EOF
```
Then register `arena.usd` (path + sha256) under `generated_arena_usds` in `artifacts/asset-manifest.json`.

- [ ] **Step 7: Live smoke (dev host)** — record outputs, claim only development-validated:

```bash
./scripts/launch-arena-streaming --arena rcw2026        # walls + furniture render
# sensor-rich camera check and AMCL-on-derived-map check per the
# integration/NAVIGATION.md two-terminal workflow, with --arena rcw2026
```

- [ ] **Step 8: Commit**

```bash
git add tools/tinker_sim_deploy/arena_convert.py tools/arena_import.py \
        config/arena-import.json tests/test_arena_import_cli.py
git commit -m "feat: arena importer CLI with pinned SOBITS source and Kit conversion"
```
(Artifacts under `artifacts/` follow the existing tracking convention — commit only if the robot artifact generations are tracked; otherwise leave untracked and rely on the asset manifest, matching whatever `git check-ignore artifacts/robot/tinker2` shows.)

---

### Task 10: YCB object importer

**Files:**
- Create: `tools/ycb_import.py`, `config/ycb-import.json`
- Modify: `tools/tinker_sim_deploy/arena_convert.py` (add `convert_object_to_usd`)
- Test: `tests/test_ycb_import_cli.py`

**Interfaces:**
- Consumes: `publish_asset_artifact`, `attribution_markdown` (Task 4), converter-hooks pattern (Task 9).
- Produces: `artifacts/objects/ycb/<identity>/` with `<object_id>/object.usd` per allowlisted object, `source-lock.json`, `manifest.json`, `ATTRIBUTION.md`; `config/ycb-import.json`:

```json
{
  "repository": "https://github.com/TeamSOBITS/tmc_wrs_gz",
  "branch": "<default branch, checked at pin time>",
  "commit": "<filled at pin time>",
  "models_root": "tmc_wrs_gz_worlds/models",
  "object_allowlist": ["<ten ycb_* ids chosen from pick/place needs, filled at pin time>"]
}
```

`convert_object_to_usd(dae_path, stl_path, usd_path)`: convert the textured DAE via the same asset-converter task; apply `UsdPhysics.MeshCollisionAPI` with `CreateApproximationAttr("convexDecomposition")` to the collision geometry imported from the STL (reference both into one `object.usd`, visual mesh visible, collision mesh invisible).

`ATTRIBUTION.md` sections (exact bodies in code, not paraphrased at run time):
1. "YCB Object and Model Set — CC BY 4.0": states the objects are converted from the Yale-CMU-Berkeley (YCB) Object and Model Set, distributed under the Creative Commons Attribution 4.0 International license, with the per-object upstream paths and hashes.
2. "tmc_wrs_gz — Clear BSD": upstream `LICENSE` bytes (© Toyota Motor Corporation).

- [ ] **Step 1: Write the failing tests** — same structure as Task 9's: stub converter, fixture git repo with `tmc_wrs_gz_worlds/models/ycb_003_cracker_box/{model.config, meshes/textured.dae, meshes/nontextured.stl}` + `LICENSE`; assert publication layout, `verify_asset_artifact` clean, pin-mismatch failure, and that `ATTRIBUTION.md` contains both section titles and the string `CC BY 4.0`.

- [ ] **Step 2: Run to verify failure** — `python3 -m unittest tests.test_ycb_import_cli -v`

- [ ] **Step 3: Implement** `tools/ycb_import.py` (mirror `run_import`/`main`/hooks from Task 9; no world parse, no map, no surfaces) and `convert_object_to_usd` in `arena_convert.py`.

- [ ] **Step 4: Run tests** — `python3 -m unittest tests.test_ycb_import_cli -v`, full-suite tail check.

- [ ] **Step 5: Live pin + import (network + Isaac venv)** — `git ls-remote` the tmc_wrs_gz default branch, list `ycb_*` model ids, choose ten used by pick/place scenario needs, fill the config, run the importer, verify, register under `generated_object_usds` in `artifacts/asset-manifest.json`.

- [ ] **Step 6: Commit**

```bash
git add tools/ycb_import.py config/ycb-import.json \
        tools/tinker_sim_deploy/arena_convert.py tests/test_ycb_import_cli.py
git commit -m "feat: YCB object importer with CC-BY attribution"
```

---

### Task 11: Documentation

**Files:**
- Modify: `README.md` (new section after "Vision hardware-parity cameras"; changelog entry at the top of "Changelog")

**Interfaces:** none.

- [ ] **Step 1: Write the README section** — titled "RoboCup 2026 arena", covering: what the arena artifact is and its provenance (SOBITS `feature/hri`, pinned commit, BSD-3-Clause; YCB CC BY 4.0); the import commands (`tools/arena_import.py`, `tools/ycb_import.py`, venv + network requirements); `--arena rcw2026` usage on `launch-isaac` / `launch-arena-streaming`; the derived-map single-source-of-truth statement; the scenario `world: {"mode": "arena", "arena": "rcw2026"}` declaration; the `--arena`/`--map` and `--arena`/`--arena-colors` exclusions; and an explicit status line: development-validated only, **not release-qualified**. Match the README's existing tone: dense, factual, no marketing.

- [ ] **Step 2: Add the changelog entry** — dated, summarizing tasks 1–10, naming the new files and the truthful-scope caveats (no release qualification, live smoke on dev host only).

- [ ] **Step 3: Full suite + docs check**

Run: `python3 -m unittest discover -s tests -v 2>&1 | tail -5`
Expected: no regressions (docs-assertion tests such as `tests/test_integrated_acceptance_docs.py` must still pass — the new section must not disturb asserted blocks).

- [ ] **Step 4: Commit**

```bash
git add README.md
git commit -m "docs: RoboCup 2026 arena import, usage, and status"
```

---

## Plan Self-Review (completed at write time)

- **Spec coverage:** §4 importer → Task 9; §5 walls/map → Tasks 2, 9; §6 colliders/surfaces → Tasks 1, 3, 9; §7 artifact layout/publication + asset-manifest → Tasks 4, 5; §8 backend/launcher → Tasks 7, 8; §9 scenario schema → Tasks 6, 8; §10 YCB → Task 10; §11 licensing → Tasks 4, 9, 10; §12 testing → every task + Task 9 steps 6–7; §13 exclusions respected (no generator, no gz_human_sim, no door articulation).
- **Type consistency:** `BoxCollider`/`MeshCollider`/`ArenaLayout` names match across Tasks 1, 2, 9; `publish_asset_artifact(repo_root, *, kind, asset_id, payload, source_lock)` matches between Tasks 4, 9, 10; `validate_world_selection(scenario, arena_id)` matches Tasks 6, 8; `arena_artifact` kwarg matches Tasks 7, 8.
- **Known live-verification surface:** `arena_convert.py` Kit/pxr API names and the exact upstream world-file structure (Task 1 note, Task 9 steps 4–6) are validated on the GPU host during Task 9; unit-tested logic is Kit-free by construction.
