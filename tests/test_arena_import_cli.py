"""Task 9: arena importer CLI orchestration -- ``run_import`` with stub Kit hooks.

Covers only the pure-Python orchestration in ``tools/arena_import.py``: pin
verification, wiring Tasks 1-5 modules together, and publishing through
``arena_artifact``. Every Kit/pxr call is injected via ``ConverterHooks`` so
these tests need no GPU/Isaac Sim and run under plain ``unittest``. The real
``arena_convert`` implementation is exercised only by the live import (see
the Task 9 report), never here.
"""
from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import arena_import  # noqa: E402
from tinker_sim_deploy import arena_artifact, arena_map, arena_world  # noqa: E402
from tinker_sim_deploy.arena_coacd import ConvexPiece  # noqa: E402

_ROBOT_URDF = b"""<robot name="t">
  <joint name="livox_joint" type="fixed">
    <parent link="base_link"/><child link="livox_frame"/>
    <origin rpy="0 0 0" xyz="0.09 0 0.195"/>
  </joint>
</robot>"""

# Box collision + visual mesh centred/scaled so StubHooks.measure_bounds's
# fixed return value ((-0.6,-0.3,0.0), (0.6,0.3,0.74)) matches this SDF's
# declared box-collider extents exactly (size 1.2x0.6x0.74, centre
# (0,0,0.37)) -- the "happy path" bounds check must pass.
_MODEL_SDF = b"""<?xml version="1.0"?>
<sdf version="1.10">
  <model name="rcw26_kitchen_table">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <pose>0 0 0.37 0 0 0</pose>
        <geometry><box><size>1.2 0.6 0.74</size></box></geometry>
      </collision>
      <visual name="visual">
        <pose>0 0 0.37 1.5708 0 0</pose>
        <geometry>
          <mesh>
            <uri>model://rcw26_kitchen_table/meshes/x.glb</uri>
            <scale>0.5 0.5 0.5</scale>
          </mesh>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>"""

_WORLD_XACRO = b"""<?xml version="1.0"?>
<sdf version="1.10">
  <world name="rcw2026_arena">
    <model name="arena_walls">
      <link name="wall_north">
        <pose>0 4.5 0.6 0 0 0</pose>
        <collision name="c"><geometry><box><size>9 0.1 1.2</size></box></geometry></collision>
      </link>
    </model>
    <include>
      <uri>model://rcw26_kitchen_table</uri>
      <name>kitchen_table</name>
      <pose>2.0 1.0 0 0 0 0</pose>
      <static>1</static>
    </include>
  </world>
</sdf>"""

_LICENSE = b"MIT License\n\nCopyright upstream.\n"


def _git(cwd: Path, *args: str) -> subprocess.CompletedProcess[str]:
    result = subprocess.run(
        ["git", "-C", str(cwd), *args],
        capture_output=True,
        text=True,
        env={**os.environ, "LC_ALL": "C"},
        check=False,
    )
    if result.returncode != 0:
        raise AssertionError(f"git {' '.join(args)} failed: {result.stderr}")
    return result


def _git_head(cwd: Path) -> str:
    return _git(cwd, "rev-parse", "HEAD").stdout.strip()


def _build_checkout(root: Path) -> Path:
    checkout = root / "checkout"
    checkout.mkdir(parents=True)
    (checkout / "worlds").mkdir()
    (checkout / "worlds" / "rcw2026_arena.world.xacro").write_bytes(_WORLD_XACRO)
    model_dir = checkout / "models" / "rcw26_kitchen_table"
    (model_dir / "meshes").mkdir(parents=True)
    (model_dir / "model.sdf").write_bytes(_MODEL_SDF)
    (model_dir / "meshes" / "x.glb").write_bytes(b"fake-glb-bytes")
    (checkout / "LICENSE").write_bytes(_LICENSE)
    _git(checkout, "init", "-q")
    _git(checkout, "config", "user.email", "fixture@example.invalid")
    _git(checkout, "config", "user.name", "Fixture")
    _git(checkout, "add", "-A")
    _git(checkout, "commit", "-q", "-m", "fixture arena checkout")
    return checkout


def _build_repo_root(root: Path) -> Path:
    repo_root = root / "repo"
    robot_dir = repo_root / "artifacts" / "robot" / "tinker2" / "deadbeef"
    robot_dir.mkdir(parents=True)
    (robot_dir / "robot.urdf").write_bytes(_ROBOT_URDF)
    pointer = repo_root / "artifacts" / "robot" / "tinker2" / "current.json"
    pointer.parent.mkdir(parents=True, exist_ok=True)
    pointer.write_text(
        json.dumps({"manifest": "artifacts/robot/tinker2/deadbeef/manifest.json"})
    )
    return repo_root


def _base_config(commit: str) -> dict[str, object]:
    return {
        "repository": "https://example.invalid/sobits_gazebo_worlds",
        "branch": "feature/hri",
        "commit": commit,
        "world": "worlds/rcw2026_arena.world.xacro",
        "arena_id": "rcw2026",
        "model_allowlist": ["rcw26_kitchen_table"],
        "model_skiplist": [],
        "bounds_check_exceptions": [],
        "surfaces": [
            {
                "model_id": "rcw26_kitchen_table",
                "surface_name": "top",
                "local_center": [0.0, 0.0, 0.74],
                "size_xy": [1.2, 0.6],
                "edge_margin": 0.05,
            }
        ],
        "bounds_tolerance_m": 0.01,
        "coacd": {
            "threshold": 0.02,
            "max_convex_hull": -1,
            "preprocess_mode": "auto",
            "preprocess_resolution": 50,
            "resolution": 2000,
            "mcts_nodes": 20,
            "mcts_iterations": 150,
            "mcts_max_depth": 3,
            "pca": False,
            "merge": True,
            "max_ch_vertex": 64,
            "seed": 0,
            "plate_surfaces": [],
        },
    }


def _box_piece(size, center) -> ConvexPiece:
    """A box as a ConvexPiece: 8 corners, 12 triangles."""
    hx, hy, hz = (s / 2.0 for s in size)
    cx, cy, cz = center
    verts = np.array(
        [[cx + sx * hx, cy + sy * hy, cz + sz * hz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
    )
    faces = np.array(
        [[0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5], [0, 4, 5], [0, 5, 1],
         [2, 3, 7], [2, 7, 6], [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]],
        dtype=np.int64,
    )
    return ConvexPiece(vertices=verts, faces=faces)


class StubHooks:
    """Mirrors the brief's sketch, extended with the optional keyword-only
    ``mesh_scale``/``mesh_pose`` arguments the real ``convert_glb_to_usd``
    needs to correct for the upstream GLBs being Y-up and unit-normalized
    (see the Task 9 report) -- defaulted so a bare 2-positional-arg call
    behaves exactly like the brief's original stub.
    """

    def __init__(self, measure_bounds_return=((-0.6, -0.3, 0.0), (0.6, 0.3, 0.74)), write_texture=False,
                 pieces=None):
        self._measure_bounds_return = measure_bounds_return
        self._write_texture = write_texture
        self.convert_calls: list[dict[str, object]] = []
        self.author_coacd_calls: list[tuple[Path, object]] = []
        self.author_box_calls: list[tuple[Path, tuple]] = []
        # model_id -> pieces; default is one box equal to the kitchen table's map box
        self.pieces = pieces if pieces is not None else {
            "rcw26_kitchen_table": [_box_piece((1.2, 0.6, 0.74), (0.0, 0.0, 0.37))]
        }

    def convert_glb_to_usd(self, glb, usd, *, mesh_scale=(1.0, 1.0, 1.0), mesh_pose=(0.0,) * 6):
        self.convert_calls.append({"glb": glb, "usd": usd, "mesh_scale": mesh_scale, "mesh_pose": mesh_pose})
        usd.parent.mkdir(parents=True, exist_ok=True)
        usd.write_bytes(b"stub-usd:" + glb.name.encode())
        if self._write_texture:
            # Mirrors arena_convert._relocate_local_assets's real layout:
            # usd.parent / "textures" / <model_id> / <filename>.
            texture_dir = usd.parent / "textures" / usd.stem
            texture_dir.mkdir(parents=True, exist_ok=True)
            (texture_dir / "tex0.png").write_bytes(b"fake-png-bytes")

    def author_model_colliders(self, usd, colliders):
        self.author_box_calls.append((usd, tuple(colliders)))

    def author_coacd_colliders(self, usd, params):
        self.author_coacd_calls.append((usd, params))
        return len(self.pieces.get(usd.stem, ()))

    def read_collider_pieces(self, usd):
        return list(self.pieces.get(usd.stem, ()))

    def compose_arena(self, arena_usd, layout, furniture_dir_name="furniture"):
        arena_usd.write_bytes(b"stub-arena")

    def measure_bounds(self, usd):
        return self._measure_bounds_return


class RunImportTest(unittest.TestCase):
    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.root = Path(self._tmp.name)
        self.checkout = _build_checkout(self.root)
        self.repo_root = _build_repo_root(self.root)
        self.commit = _git_head(self.checkout)

    def test_import_publishes_artifact(self):
        config = _base_config(self.commit)
        hooks = StubHooks()

        publication = arena_import.run_import(config, self.repo_root, self.checkout, hooks).publication

        self.assertTrue(publication.created)
        artifact_dir = publication.artifact_dir
        self.assertTrue((artifact_dir / "arena.usd").is_file())
        self.assertTrue((artifact_dir / "furniture" / "rcw26_kitchen_table.usd").is_file())
        self.assertTrue((artifact_dir / "map.yaml").is_file())
        self.assertTrue((artifact_dir / "map.pgm").is_file())
        self.assertTrue((artifact_dir / "placement.json").is_file())
        self.assertTrue((artifact_dir / "source-lock.json").is_file())
        self.assertTrue((artifact_dir / "manifest.json").is_file())
        self.assertTrue((artifact_dir / "ATTRIBUTION.md").is_file())
        self.assertEqual(arena_artifact.verify_asset_artifact(artifact_dir), [])

        # the real GLB scale/pose from the model SDF must have reached the
        # converter hook, not just the default identity values
        self.assertEqual(len(hooks.convert_calls), 1)
        call = hooks.convert_calls[0]
        self.assertEqual(call["mesh_scale"], (0.5, 0.5, 0.5))
        self.assertEqual(call["mesh_pose"], (0.0, 0.0, 0.37, 1.5708, 0.0, 0.0))

        placement = json.loads((artifact_dir / "placement.json").read_bytes())
        self.assertEqual(placement["arena_id"], "rcw2026")
        self.assertEqual(len(placement["surfaces"]), 1)
        self.assertEqual(placement["surfaces"][0]["surface_id"], "kitchen_table#top")

        lock = json.loads((artifact_dir / "source-lock.json").read_bytes())
        self.assertEqual(lock["repository"], config["repository"])
        self.assertEqual(lock["branch"], config["branch"])
        self.assertEqual(lock["commit"], self.commit)
        recorded_paths = {record["path"] for record in lock["records"]}
        self.assertIn("worlds/rcw2026_arena.world.xacro", recorded_paths)
        self.assertIn("models/rcw26_kitchen_table/model.sdf", recorded_paths)
        self.assertIn("models/rcw26_kitchen_table/meshes/x.glb", recorded_paths)
        self.assertIn("LICENSE", recorded_paths)

    def test_import_does_not_select(self):
        pointer = self.repo_root / "artifacts" / "arena" / "rcw2026" / "current.json"
        pointer.parent.mkdir(parents=True, exist_ok=True)
        pointer.write_bytes(b'{"schema_version":1,"manifest":"old"}')

        arena_import.run_import(_base_config(self.commit), self.repo_root, self.checkout, StubHooks())

        self.assertEqual(pointer.read_bytes(), b'{"schema_version":1,"manifest":"old"}')

    def test_select_flag_selects_without_import(self):
        publication = arena_import.run_import(
            _base_config(self.commit), self.repo_root, self.checkout, StubHooks()
        ).publication
        pointer = self.repo_root / "artifacts" / "arena" / "rcw2026" / "current.json"
        self.assertFalse(pointer.exists())
        cfg = self.root / "arena-import.json"
        cfg.write_text(json.dumps(_base_config(self.commit)))

        with mock.patch.object(arena_import, "clone_pin", side_effect=AssertionError("cloned")):
            rc = arena_import.main(
                ["--config", str(cfg), "--select", publication.identity], repo_root=self.repo_root
            )

        self.assertEqual(rc, 0)
        self.assertEqual(
            json.loads(pointer.read_text())["manifest"],
            f"artifacts/arena/rcw2026/{publication.identity}/manifest.json",
        )

    def test_pin_mismatch_fails_closed(self):
        config = _base_config("f" * 40)
        hooks = StubHooks()

        with self.assertRaises(arena_artifact.AssetArtifactError):
            arena_import.run_import(config, self.repo_root, self.checkout, hooks)

        self.assertFalse((self.repo_root / "artifacts" / "arena").exists())

    def test_bounds_violation_fails_closed(self):
        config = _base_config(self.commit)
        # 5 cm off on the X axis vs the SDF-declared box collider (max 0.6)
        hooks = StubHooks(measure_bounds_return=((-0.6, -0.3, 0.0), (0.65, 0.3, 0.74)))

        with self.assertRaises(arena_artifact.AssetArtifactError):
            arena_import.run_import(config, self.repo_root, self.checkout, hooks)

        self.assertFalse((self.repo_root / "artifacts" / "arena").exists())

    def test_bounds_check_exception_skips_strict_check_for_named_model(self):
        # Real upstream box colliders can *deliberately* under-approximate
        # their visual mesh (e.g. rcw26_door's collision box explicitly
        # excludes a protruding doorknob per its own SDF comment) -- an
        # explicit, named config exception (mirroring model_skiplist) lets
        # the importer accept a known, documented mismatch instead of
        # failing the whole import, while still recording the *measured*
        # AABB (not the untrustworthy declared box) for the map slice.
        config = _base_config(self.commit)
        config["bounds_check_exceptions"] = ["rcw26_kitchen_table"]
        hooks = StubHooks(measure_bounds_return=((-0.6, -0.3, 0.0), (0.65, 0.3, 0.74)))

        publication = arena_import.run_import(config, self.repo_root, self.checkout, hooks).publication

        self.assertEqual(arena_artifact.verify_asset_artifact(publication.artifact_dir), [])

    def test_second_visual_mesh_fails_closed(self):
        # _visual_mesh_info returned on the *first* <visual> match found by
        # root.iter("visual") -- a model with two visual meshes would
        # silently lose the second mesh's geometry and its GLB would never
        # enter the source lock. Must fail closed instead.
        sdf_with_two_visuals = _MODEL_SDF.replace(
            b"</link>",
            b"""<visual name="visual2">
        <pose>0 0 0.1 1.5708 0 0</pose>
        <geometry><mesh><uri>model://rcw26_kitchen_table/meshes/extra.glb</uri></mesh></geometry>
      </visual>
    </link>""",
        )
        with self.assertRaises(arena_artifact.AssetArtifactError):
            arena_import._visual_mesh_info(sdf_with_two_visuals)

    def test_source_lock_glb_path_is_the_resolved_file_not_a_naming_guess(self):
        # _relative_glb_record_path used to fabricate "models/<id>/meshes/
        # <name>" instead of deriving the path from the file actually
        # resolved and read -- indistinguishable from ground truth in the
        # original fixture (which happens to live exactly one level under
        # meshes/), so this fixture nests the glb one level deeper to prove
        # the recorded path is derived from the real resolved location.
        nested_sdf = _MODEL_SDF.replace(
            b"model://rcw26_kitchen_table/meshes/x.glb",
            b"model://rcw26_kitchen_table/meshes/nested/x.glb",
        )
        root = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, root, ignore_errors=True)
        checkout = _build_checkout(root)
        model_dir = checkout / "models" / "rcw26_kitchen_table"
        (model_dir / "model.sdf").write_bytes(nested_sdf)
        (model_dir / "meshes" / "nested").mkdir()
        (model_dir / "meshes" / "nested" / "x.glb").write_bytes(b"fake-glb-bytes")
        _git(checkout, "add", "-A")
        _git(checkout, "commit", "-q", "-m", "nest the glb one level deeper")
        commit = _git_head(checkout)

        config = _base_config(commit)
        hooks = StubHooks()
        publication = arena_import.run_import(config, self.repo_root, checkout, hooks).publication

        lock = json.loads((publication.artifact_dir / "source-lock.json").read_bytes())
        recorded_paths = {record["path"] for record in lock["records"]}
        self.assertIn("models/rcw26_kitchen_table/meshes/nested/x.glb", recorded_paths)
        self.assertNotIn("models/rcw26_kitchen_table/meshes/x.glb", recorded_paths)

    def test_relocated_textures_are_published_alongside_their_usd(self):
        # arena_convert.convert_glb_to_usd relocates any texture/material
        # files Kit extracted to usd.parent/"textures"/<model_id>/ (see the
        # Task 9 fix report, Finding 1) -- run_import must republish those
        # alongside the referencing USD or the relative path it rewrote
        # them to resolves to nothing in the artifact.
        config = _base_config(self.commit)
        hooks = StubHooks(write_texture=True)

        publication = arena_import.run_import(config, self.repo_root, self.checkout, hooks).publication

        texture_path = publication.artifact_dir / "furniture" / "textures" / "rcw26_kitchen_table" / "tex0.png"
        self.assertTrue(texture_path.is_file())
        self.assertEqual(texture_path.read_bytes(), b"fake-png-bytes")
        self.assertEqual(arena_artifact.verify_asset_artifact(publication.artifact_dir), [])

    def test_duplicate_placements_get_distinct_surface_ids(self):
        # Two placements of the same model_id (e.g. two kitchen tables) must
        # not collide on surface_id -- world_surface's instance_suffix must
        # mirror arena_convert.compose_arena's own furniture prim-naming
        # scheme (bare model_id for the first instance, "_NN" for later
        # ones) so a surface_id always names the same instance as its
        # arena.usd prim (Final review Finding 2 -- live placement.json had
        # "shelf#plate" and "side_table#top" duplicated for real).
        root = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, root, ignore_errors=True)
        checkout = _build_checkout(root)
        world_with_two_tables = _WORLD_XACRO.replace(
            b"</world>",
            b"""<include>
      <uri>model://rcw26_kitchen_table</uri>
      <name>kitchen_table_2</name>
      <pose>-2.0 1.0 0 0 0 0</pose>
      <static>1</static>
    </include>
  </world>""",
        )
        (checkout / "worlds" / "rcw2026_arena.world.xacro").write_bytes(world_with_two_tables)
        _git(checkout, "add", "-A")
        _git(checkout, "commit", "-q", "-m", "duplicate kitchen table placement")
        commit = _git_head(checkout)

        config = _base_config(commit)
        hooks = StubHooks()
        publication = arena_import.run_import(config, self.repo_root, checkout, hooks).publication

        placement = json.loads((publication.artifact_dir / "placement.json").read_bytes())
        surface_ids = sorted(record["surface_id"] for record in placement["surfaces"])
        self.assertEqual(surface_ids, ["kitchen_table#top", "kitchen_table_02#top"])


class CoacdWiringTest(unittest.TestCase):
    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.root = Path(self._tmp.name)
        self.checkout = _build_checkout(self.root)
        self.repo_root = _build_repo_root(self.root)
        self.commit = _git_head(self.checkout)

    def _run(self, config=None, hooks=None):
        hooks = hooks or StubHooks()
        outcome = arena_import.run_import(
            config or _base_config(self.commit), self.repo_root, self.checkout, hooks
        )
        return outcome, hooks

    def test_coacd_authored_for_models_without_override(self):
        _, hooks = self._run()
        self.assertEqual(len(hooks.author_coacd_calls), 1)
        self.assertEqual(hooks.author_coacd_calls[0][0].stem, "rcw26_kitchen_table")
        self.assertEqual(hooks.author_box_calls, [])

    def test_override_models_keep_boxes(self):
        config = _base_config(self.commit)
        config["collision_overrides"] = {
            "rcw26_kitchen_table": {
                "kind": "open_shelf",
                "footprint_x": [-0.6, 0.6], "footprint_y": [-0.3, 0.3], "footprint_z": [0.0, 0.74],
                "boards_z": [[0.0, 0.04], [0.70, 0.74]],
            }
        }
        expected = arena_world.collision_overrides_from_config(config)["rcw26_kitchen_table"]
        _, hooks = self._run(config)
        self.assertEqual(hooks.author_coacd_calls, [])
        self.assertEqual(len(hooks.author_box_calls), 1)
        self.assertEqual(hooks.author_box_calls[0][1], tuple(expected))

    def test_plate_added_for_listed_surface(self):
        config = _base_config(self.commit)
        config["coacd"]["plate_surfaces"] = ["rcw26_kitchen_table"]
        _, hooks = self._run(config)
        self.assertEqual(len(hooks.author_coacd_calls), 1)
        self.assertEqual(len(hooks.author_box_calls), 1)
        plates = hooks.author_box_calls[0][1]
        self.assertEqual(len(plates), 1)
        plate = plates[0]
        self.assertAlmostEqual(plate.size[0], 1.2)
        self.assertAlmostEqual(plate.size[1], 0.6)
        self.assertAlmostEqual(plate.center[2] + plate.size[2] / 2.0, 0.74)

    def test_map_unchanged_by_coacd(self):
        seen = {}
        real = arena_map.rasterize

        def recorder(layout, colliders, **kwargs):
            seen.update(colliders)
            return real(layout, colliders, **kwargs)

        hooks = StubHooks(pieces={"rcw26_kitchen_table": [_box_piece((0.5, 0.5, 0.5), (0.0, 0.0, 0.25))]})
        with mock.patch.object(arena_import, "rasterize", recorder):
            self._run(hooks=hooks)
        boxes = seen["rcw26_kitchen_table"]
        self.assertEqual(len(boxes), 1)
        self.assertIsInstance(boxes[0], arena_world.BoxCollider)
        self.assertEqual(tuple(boxes[0].size), (1.2, 0.6, 0.74))
        self.assertEqual(tuple(boxes[0].center), (0.0, 0.0, 0.37))

    def test_outline_failure_reported_not_selected(self):
        hooks = StubHooks(pieces={
            "rcw26_kitchen_table": [_box_piece((1.2, 0.6, 0.74), (0.04, 0.0, 0.37))]})
        outcome, _ = self._run(hooks=hooks)
        self.assertFalse(outcome.outlines[0].passed)
        summary = json.loads(outcome.report.read_text())
        self.assertFalse(summary["passed"])
        self.assertEqual(
            outcome.report.parent,
            self.repo_root / "reports" / "arena-outline" / outcome.publication.identity[:12],
        )
        pointer = self.repo_root / "artifacts" / "arena" / "rcw2026" / "current.json"
        self.assertFalse(pointer.exists())

    def test_map_drawn_from_outlines_frame_unchanged(self):
        hooks = StubHooks(pieces={
            "rcw26_kitchen_table": [_box_piece((0.5, 0.3, 0.74), (0.0, 0.0, 0.37))]})
        box_maps = []
        real = arena_map.rasterize

        def recorder(layout, colliders, **kwargs):
            result = real(layout, colliders, **kwargs)
            if kwargs.get("furniture_outlines") is None:
                box_maps.append(result)
            return result

        with mock.patch.object(arena_import, "rasterize", recorder):
            outcome, _ = self._run(hooks=hooks)
        (box_pgm, box_yaml), = box_maps
        art = outcome.publication.artifact_dir
        self.assertEqual((art / "map.yaml").read_bytes(), box_yaml)
        self.assertNotEqual((art / "map.pgm").read_bytes(), box_pgm)
        self.assertTrue((outcome.report.parent / "map-diff.png").is_file())

    def test_outline_pass_report(self):
        outcome, _ = self._run()
        self.assertTrue(all(o.passed for o in outcome.outlines))
        self.assertTrue(json.loads(outcome.report.read_text())["passed"])

    def test_main_exits_zero_on_outline_failure(self):
        cfg = self.root / "arena-import.json"
        cfg.write_text(json.dumps(_base_config(self.commit)))
        hooks = StubHooks(pieces={
            "rcw26_kitchen_table": [_box_piece((1.2, 0.6, 0.74), (0.04, 0.0, 0.37))]})
        with mock.patch.object(arena_import, "_build_real_converter", return_value=hooks), \
                mock.patch.object(arena_import, "ReuseFurnitureHooks", return_value=hooks):
            art = self.root / "reuse"
            (art / "furniture").mkdir(parents=True)
            rc = arena_import.main(
                ["--config", str(cfg), "--checkout", str(self.checkout),
                 "--reuse-furniture-from", str(art)],
                repo_root=self.repo_root,
            )
        self.assertEqual(rc, 0)
        self.assertTrue(list((self.repo_root / "reports" / "arena-outline").glob("*/summary.json")))

    def test_collision_record_in_payload(self):
        config = _base_config(self.commit)
        outcome, _ = self._run(config)
        record = json.loads((outcome.publication.artifact_dir / "collision.json").read_bytes())
        self.assertEqual(record["coacd"], arena_import.CoacdParams.from_config(config["coacd"]).as_record())
        self.assertEqual(record["pieces"], {"rcw26_kitchen_table": 1})
        self.assertEqual(record["overrides"], [])
        self.assertEqual(record["plates"], [])

    def test_reimport_is_idempotent_with_coacd(self):
        first, _ = self._run()
        second, _ = self._run()
        self.assertEqual(first.publication.identity, second.publication.identity)


class FurnitureInstanceSuffixTest(unittest.TestCase):
    def test_first_instance_bare_later_instances_numbered(self):
        first = arena_world.FurniturePlacement(
            model_id="rcw26_side_table", position=(0.0, 0.0, 0.0), yaw=0.0, static=True,
        )
        second = arena_world.FurniturePlacement(
            model_id="rcw26_side_table", position=(1.0, 0.0, 0.0), yaw=0.0, static=True,
        )
        other = arena_world.FurniturePlacement(
            model_id="rcw26_shelf", position=(2.0, 0.0, 0.0), yaw=0.0, static=True,
        )
        suffixes = arena_import._furniture_instance_suffixes((first, second, other))
        self.assertEqual(suffixes[id(first)], "")
        self.assertEqual(suffixes[id(second)], "_02")
        self.assertEqual(suffixes[id(other)], "")


class ResolveGlbPathTest(unittest.TestCase):
    """Final review Finding 3: ``_resolve_glb_path`` must fail closed, before
    any read, on either shape of checkout escape -- an absolute URI
    remainder (``Path.__truediv__`` discards the joined prefix in favor of
    an absolute right operand) or a lexical ``..`` traversal.
    """

    def setUp(self) -> None:
        self.checkout = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, self.checkout, ignore_errors=True)

    def test_absolute_uri_remainder_is_rejected(self):
        # "model://rcw26_kitchen_table//etc/passwd" strips to the absolute
        # remainder "/etc/passwd" -- naive Path.__truediv__ joining collapses
        # checkout/models/<id>/"/etc/passwd" straight down to "/etc/passwd".
        outside = Path("/etc/passwd")
        with self.assertRaises(arena_artifact.AssetArtifactError):
            arena_import._resolve_glb_path(
                self.checkout, "rcw26_kitchen_table", f"model://rcw26_kitchen_table/{outside}"
            )

    def test_dotdot_traversal_is_rejected(self):
        with self.assertRaises(arena_artifact.AssetArtifactError):
            arena_import._resolve_glb_path(
                self.checkout, "rcw26_kitchen_table", "model://rcw26_kitchen_table/../../../etc/passwd"
            )

    def test_ordinary_relative_uri_still_resolves(self):
        model_dir = self.checkout / "models" / "rcw26_kitchen_table" / "meshes"
        model_dir.mkdir(parents=True)
        (model_dir / "x.glb").write_bytes(b"glb-bytes")
        resolved = arena_import._resolve_glb_path(
            self.checkout, "rcw26_kitchen_table", "model://rcw26_kitchen_table/meshes/x.glb"
        )
        self.assertEqual(resolved, (model_dir / "x.glb").resolve())


if __name__ == "__main__":
    unittest.main()
