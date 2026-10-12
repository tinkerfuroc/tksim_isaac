"""Collision overrides for the arena importer (config ``collision_overrides``).

Pure spec expansion (``arena_world.expand_collision_override``), the
``run_import`` wiring that swaps a model's SDF colliders for the override, and
-- when pxr is importable (the repo venv has it) -- the real authoring/strip
round trip on a tiny mesh USD and the Kit-free ``ReuseFurnitureHooks``.
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import arena_import  # noqa: E402
from tinker_sim_deploy import arena_world  # noqa: E402
from tinker_sim_deploy.arena_world import (  # noqa: E402
    ArenaWorldError,
    BoxCollider,
    MeshCollider,
    collision_overrides_from_config,
    expand_collision_override,
)

SHELF_SPEC = {
    "kind": "open_shelf",
    "footprint_x": [-0.4274, 0.4277],
    "footprint_y": [-0.1463, 0.1507],
    "footprint_z": [0.0, 2.02],
    "boards_z": [[0.07, 0.09], [0.40, 0.43], [0.69, 0.72], [1.03, 1.06], [1.34, 1.38], [1.62, 1.66], [1.97, 2.02]],
    "side_panel_thickness": 0.04,
    "back_panel": {"side": "+y", "thickness": 0.045},
}


def _z_range(box: BoxCollider) -> tuple[float, float]:
    return round(box.center[2] - box.size[2] / 2, 4), round(box.center[2] + box.size[2] / 2, 4)


# ---------------------------------------------------------------- expansion

def test_open_shelf_expands_to_boards_side_panels_and_back_panel():
    boxes = expand_collision_override(SHELF_SPEC)
    assert len(boxes) == 7 + 2 + 1
    assert all(isinstance(b, BoxCollider) and b.yaw == 0.0 for b in boxes)
    boards = boxes[:7]
    assert [_z_range(b) for b in boards] == [tuple(b) for b in SHELF_SPEC["boards_z"]]
    for b in boards:  # full footprint in x/y
        assert b.size[0] == pytest.approx(0.8551) and b.size[1] == pytest.approx(0.297)
        assert b.center[0] == pytest.approx(0.00015) and b.center[1] == pytest.approx(0.0022)
    left, right = boxes[7], boxes[8]
    assert left.size[0] == pytest.approx(0.04) and left.center[0] == pytest.approx(-0.4274 + 0.02)
    assert right.size[0] == pytest.approx(0.04) and right.center[0] == pytest.approx(0.4277 - 0.02)
    for panel in (left, right):
        assert _z_range(panel) == (0.0, 2.02) and panel.size[1] == pytest.approx(0.297)
    back = boxes[9]
    assert back.size[1] == pytest.approx(0.045) and back.center[1] == pytest.approx(0.1507 - 0.0225)
    assert back.size[0] == pytest.approx(0.8551) and _z_range(back) == (0.0, 2.02)


def test_boards_are_sorted_and_panels_optional():
    spec = {"kind": "open_shelf", "footprint_x": [-1, 1], "footprint_y": [-0.5, 0.5],
            "footprint_z": [0, 2], "boards_z": [[1.0, 1.1], [0.2, 0.3]]}
    boxes = expand_collision_override(spec)
    assert [_z_range(b) for b in boxes] == [(0.2, 0.3), (1.0, 1.1)]


@pytest.mark.parametrize("side, expect_axis, expect_center", [
    ("-y", 1, -0.5 + 0.05), ("+x", 0, 1 - 0.05), ("-x", 0, -1 + 0.05),
])
def test_back_panel_sides(side, expect_axis, expect_center):
    spec = {"kind": "open_shelf", "footprint_x": [-1, 1], "footprint_y": [-0.5, 0.5],
            "footprint_z": [0, 2], "boards_z": [[0.2, 0.3]], "back_panel": {"side": side, "thickness": 0.1}}
    back = expand_collision_override(spec)[-1]
    assert back.size[expect_axis] == pytest.approx(0.1)
    assert back.center[expect_axis] == pytest.approx(expect_center)


@pytest.mark.parametrize("bad", [
    "not a dict",
    {"kind": "convex"},
    {**SHELF_SPEC, "boards_z": []},
    {**SHELF_SPEC, "boards_z": [[0.5, 0.4]]},
    {**SHELF_SPEC, "boards_z": [[1.9, 2.5]]},          # above footprint_z
    {**SHELF_SPEC, "footprint_x": [0.4, -0.4]},
    {**SHELF_SPEC, "side_panel_thickness": 0.6},       # >= half width
    {**SHELF_SPEC, "back_panel": {"side": "up", "thickness": 0.04}},
    {**SHELF_SPEC, "back_panel": {"side": "+y", "thickness": 0.0}},
])
def test_malformed_specs_fail_closed(bad):
    with pytest.raises(ArenaWorldError):
        expand_collision_override(bad)


def test_config_loader_skips_comment_keys_and_defaults_empty():
    assert collision_overrides_from_config({}) == {}
    assert collision_overrides_from_config({"collision_overrides": None}) == {}
    out = collision_overrides_from_config({"collision_overrides": {"_comment": "why", "rcw26_shelf": SHELF_SPEC}})
    assert list(out) == ["rcw26_shelf"] and len(out["rcw26_shelf"]) == 10
    with pytest.raises(ArenaWorldError):
        collision_overrides_from_config({"collision_overrides": ["nope"]})


def test_repo_config_shelf_override_matches_measured_boards():
    config = json.loads((ROOT / "config" / "arena-import.json").read_text())
    boxes = collision_overrides_from_config(config)["rcw26_shelf"]
    tops = [_z_range(b)[1] for b in boxes[:7]]
    assert tops == [0.09, 0.43, 0.72, 1.06, 1.38, 1.66, 2.02]
    # placements.json spot z for the shelf must sit just above a board top
    placements = json.loads((ROOT / "simulation" / "scenarios" / "rcw2026-placements.json").read_text())
    for spot in ("shelf", "shelf_02"):
        z = placements["spots"][spot]["surface_xyz"][2]
        assert any(0.0 <= z - top <= 0.02 for top in tops), (spot, z)


# -------------------------------------------------------------- run_import

_SHELF_SDF = b"""<?xml version="1.0"?>
<sdf version="1.10">
  <model name="rcw26_shelf">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <pose>0 0 1.01 1.5708 0 0</pose>
        <geometry><mesh><uri>model://rcw26_shelf/meshes/shelf.glb</uri><scale>2 2 2</scale></mesh></geometry>
      </collision>
      <visual name="visual">
        <pose>0 0 1.01 1.5708 0 0</pose>
        <geometry><mesh><uri>model://rcw26_shelf/meshes/shelf.glb</uri><scale>2 2 2</scale></mesh></geometry>
      </visual>
    </link>
  </model>
</sdf>"""

_WORLD = b"""<?xml version="1.0"?>
<sdf version="1.10">
  <world name="rcw2026_arena">
    <model name="arena_walls">
      <link name="wall_north">
        <pose>0 4.5 0.6 0 0 0</pose>
        <collision name="c"><geometry><box><size>9 0.1 1.2</size></box></geometry></collision>
      </link>
    </model>
    <include><uri>model://rcw26_shelf</uri><pose>2.0 1.0 0 0 0 0</pose><static>1</static></include>
  </world>
</sdf>"""

_URDF = b"""<robot name="t"><joint name="livox_joint" type="fixed">
<parent link="base_link"/><child link="livox_frame"/><origin rpy="0 0 0" xyz="0.09 0 0.195"/>
</joint></robot>"""


def _git(cwd: Path, *args: str) -> str:
    r = subprocess.run(["git", "-C", str(cwd), *args], capture_output=True, text=True,
                       env={**os.environ, "LC_ALL": "C"}, check=False)
    assert r.returncode == 0, r.stderr
    return r.stdout.strip()


def _checkout(root: Path) -> Path:
    checkout = root / "checkout"
    (checkout / "worlds").mkdir(parents=True)
    (checkout / "worlds" / "rcw2026_arena.world.xacro").write_bytes(_WORLD)
    model = checkout / "models" / "rcw26_shelf"
    (model / "meshes").mkdir(parents=True)
    (model / "model.sdf").write_bytes(_SHELF_SDF)
    (model / "meshes" / "shelf.glb").write_bytes(b"fake-glb")
    (checkout / "LICENSE").write_bytes(b"MIT\n")
    _git(checkout, "init", "-q")
    _git(checkout, "config", "user.email", "f@example.invalid")
    _git(checkout, "config", "user.name", "F")
    _git(checkout, "add", "-A")
    _git(checkout, "commit", "-q", "-m", "fixture")
    return checkout


def _repo_root(root: Path) -> Path:
    repo = root / "repo"
    robot = repo / "artifacts" / "robot" / "tinker2" / "deadbeef"
    robot.mkdir(parents=True)
    (robot / "robot.urdf").write_bytes(_URDF)
    (repo / "artifacts" / "robot" / "tinker2" / "current.json").write_text(
        json.dumps({"manifest": "artifacts/robot/tinker2/deadbeef/manifest.json"}))
    return repo


class _Hooks:
    def __init__(self):
        self.authored: dict[str, tuple] = {}

    def convert_glb_to_usd(self, glb, usd, *, mesh_scale=(1.0,) * 3, mesh_pose=(0.0,) * 6):
        usd.parent.mkdir(parents=True, exist_ok=True)
        usd.write_bytes(b"stub")

    def author_model_colliders(self, usd, colliders):
        self.authored[usd.stem] = tuple(colliders)

    def compose_arena(self, arena_usd, layout, furniture_dir_name="furniture"):
        arena_usd.write_bytes(b"stub-arena")

    def measure_bounds(self, usd):
        return ((-0.43, -0.15, 0.0), (0.43, 0.15, 2.02))


class RunImportOverrideTest(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        root = Path(self._tmp.name)
        self.checkout = _checkout(root)
        self.repo = _repo_root(root)
        self.config = {
            "repository": "https://example.invalid/x", "branch": "b", "commit": _git(self.checkout, "rev-parse", "HEAD"),
            "world": "worlds/rcw2026_arena.world.xacro", "arena_id": "rcw2026",
            "model_allowlist": ["rcw26_shelf"], "model_skiplist": [], "bounds_check_exceptions": [],
            "surfaces": [], "bounds_tolerance_m": 0.02,
        }

    def test_without_override_the_sdf_mesh_collider_is_authored(self):
        hooks = _Hooks()
        arena_import.run_import(self.config, self.repo, self.checkout, hooks)
        self.assertEqual(hooks.authored["rcw26_shelf"], (MeshCollider(uri="model://rcw26_shelf/meshes/shelf.glb"),))

    def test_override_replaces_sdf_colliders_and_keeps_measured_map_box(self):
        hooks = _Hooks()
        config = {**self.config, "collision_overrides": {"rcw26_shelf": SHELF_SPEC}}
        publication = arena_import.run_import(config, self.repo, self.checkout, hooks)
        authored = hooks.authored["rcw26_shelf"]
        self.assertEqual(len(authored), 10)
        self.assertTrue(all(isinstance(c, BoxCollider) for c in authored))
        self.assertEqual(authored, expand_collision_override(SHELF_SPEC))
        # The nav map still uses the measured AABB (the SDF collision is a mesh,
        # so the strict-box bounds check never applies); artifact verifies.
        self.assertTrue((publication.artifact_dir / "map.pgm").is_file())
        self.assertTrue((publication.artifact_dir / "furniture" / "rcw26_shelf.usd").is_file())

    def test_malformed_override_fails_closed_before_publishing(self):
        config = {**self.config, "collision_overrides": {"rcw26_shelf": {"kind": "open_shelf"}}}
        with self.assertRaises(ArenaWorldError):
            arena_import.run_import(config, self.repo, self.checkout, _Hooks())


# ------------------------------------------------------------------ pxr

def _mesh_usd(path: Path):
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.CreateNew(str(path))
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    mesh = UsdGeom.Mesh.Define(stage, "/World/geom/mesh")
    mesh.CreatePointsAttr([(-0.4, -0.15, 0), (0.4, -0.15, 0), (0.4, 0.15, 2.0), (-0.4, 0.15, 2.0)])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    stage.Save()


def test_pxr_author_boxes_then_strip_round_trip(tmp_path):
    pytest.importorskip("pxr")
    from pxr import Usd, UsdGeom, UsdPhysics
    from tinker_sim_deploy import arena_convert

    usd = tmp_path / "rcw26_shelf.usd"
    _mesh_usd(usd)
    boxes = expand_collision_override(SHELF_SPEC)
    arena_convert.author_model_colliders(usd, boxes)
    stage = Usd.Stage.Open(str(usd))
    cubes = [p for p in stage.Traverse() if p.IsA(UsdGeom.Cube)]
    assert len(cubes) == 10 and all(p.HasAPI(UsdPhysics.CollisionAPI) for p in cubes)
    mesh = stage.GetPrimAtPath("/World/geom/mesh")
    assert not mesh.HasAPI(UsdPhysics.MeshCollisionAPI) and not mesh.HasAPI(UsdPhysics.CollisionAPI)
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"], useExtentsHint=False)
    board4 = stage.GetPrimAtPath("/World/Colliders/box_03")
    r = cache.ComputeWorldBound(board4).ComputeAlignedRange()
    assert (round(r.GetMin()[2], 3), round(r.GetMax()[2], 3)) == (1.03, 1.06)
    del stage

    # Now the legacy convex-hull authoring, then strip -> pristine mesh again.
    arena_convert.author_model_colliders(usd, (MeshCollider(uri="x"),))
    stage = Usd.Stage.Open(str(usd))
    assert stage.GetPrimAtPath("/World/geom/mesh").HasAPI(UsdPhysics.MeshCollisionAPI)
    del stage
    arena_convert.strip_model_colliders(usd)
    stage = Usd.Stage.Open(str(usd))
    mesh = stage.GetPrimAtPath("/World/geom/mesh")
    assert not mesh.HasAPI(UsdPhysics.MeshCollisionAPI) and not mesh.HasAPI(UsdPhysics.CollisionAPI)
    assert not mesh.HasProperty("physics:approximation")
    assert not stage.GetPrimAtPath("/World/Colliders")
    assert len(mesh.GetAttribute("points").Get()) == 4


def test_pxr_reuse_furniture_hooks_copies_and_strips(tmp_path):
    pytest.importorskip("pxr")
    from pxr import Usd, UsdGeom, UsdPhysics
    from tinker_sim_deploy import arena_convert

    artifact = tmp_path / "artifact"
    (artifact / "furniture" / "textures" / "rcw26_shelf").mkdir(parents=True)
    (artifact / "furniture" / "textures" / "rcw26_shelf" / "t.png").write_bytes(b"png")
    source = artifact / "furniture" / "rcw26_shelf.usd"
    _mesh_usd(source)
    arena_convert.author_model_colliders(source, (MeshCollider(uri="x"),))

    hooks = arena_import.ReuseFurnitureHooks(artifact)
    out = tmp_path / "scratch" / "furniture" / "rcw26_shelf.usd"
    hooks.convert_glb_to_usd(tmp_path / "whatever.glb", out)
    stage = Usd.Stage.Open(str(out))
    mesh = stage.GetPrimAtPath("/World/geom/mesh")
    assert not mesh.HasAPI(UsdPhysics.MeshCollisionAPI)
    assert (tmp_path / "scratch" / "furniture" / "textures" / "rcw26_shelf" / "t.png").read_bytes() == b"png"
    # the source artifact is untouched
    src_stage = Usd.Stage.Open(str(source))
    assert src_stage.GetPrimAtPath("/World/geom/mesh").HasAPI(UsdPhysics.MeshCollisionAPI)
    with pytest.raises(arena_import.AssetArtifactError):
        hooks.convert_glb_to_usd(tmp_path / "x.glb", tmp_path / "scratch" / "furniture" / "rcw26_missing.usd")
    with pytest.raises(arena_import.AssetArtifactError):
        arena_import.ReuseFurnitureHooks(tmp_path / "nope")
