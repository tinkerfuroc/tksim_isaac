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

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import arena_import  # noqa: E402
from tinker_sim_deploy import arena_artifact  # noqa: E402

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
        "surface_furniture": ["rcw26_kitchen_table"],
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
    }


class StubHooks:
    """Mirrors the brief's sketch, extended with the optional keyword-only
    ``mesh_scale``/``mesh_pose`` arguments the real ``convert_glb_to_usd``
    needs to correct for the upstream GLBs being Y-up and unit-normalized
    (see the Task 9 report) -- defaulted so a bare 2-positional-arg call
    behaves exactly like the brief's original stub.
    """

    def __init__(self, measure_bounds_return=((-0.6, -0.3, 0.0), (0.6, 0.3, 0.74)), write_texture=False):
        self._measure_bounds_return = measure_bounds_return
        self._write_texture = write_texture
        self.convert_calls: list[dict[str, object]] = []

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
        pass

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

        publication = arena_import.run_import(config, self.repo_root, self.checkout, hooks)

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

        publication = arena_import.run_import(config, self.repo_root, self.checkout, hooks)

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
        publication = arena_import.run_import(config, self.repo_root, checkout, hooks)

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

        publication = arena_import.run_import(config, self.repo_root, self.checkout, hooks)

        texture_path = publication.artifact_dir / "furniture" / "textures" / "rcw26_kitchen_table" / "tex0.png"
        self.assertTrue(texture_path.is_file())
        self.assertEqual(texture_path.read_bytes(), b"fake-png-bytes")
        self.assertEqual(arena_artifact.verify_asset_artifact(publication.artifact_dir), [])


if __name__ == "__main__":
    unittest.main()
