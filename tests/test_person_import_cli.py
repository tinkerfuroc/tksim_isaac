"""Person-model importer CLI orchestration -- ``run_import`` with stub hooks.

GPSR's rcw2026 scenario commands *"go to the kitchen table and find a
person"* while its own ``actors`` list is empty, and the repo's only person
asset is ``simulation/assets/primitives/person.usda`` -- a bare capsule
(r=0.25, h=1.2) that no open-vocabulary detector will label a person. GPSR
run12 arrived at the table and then scanned 3930 times for a person that was
not there.

``models/person_standing`` in the already-vendored ``sobits_gazebo_worlds``
checkout is a textured MakeHuman figure (Marina Kollmitz, Uni Freiburg) --
same repository, same pinned commit, same BSD-3 licence as the arena
furniture -- so it needs no new upstream source.

Covers only the pure-Python orchestration, exactly as
``test_ycb_import_cli.py`` does: pin verification, resolving the visual mesh
through the model SDF, the guard on upstream's declared visual pose, and
publishing through ``arena_artifact``. The real Kit conversion is exercised
only by the live import.
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import person_import  # noqa: E402
from tinker_sim_deploy import arena_artifact  # noqa: E402

_LICENSE = (
    b"BSD 3-Clause License\n\nCopyright (c) 2023, TeamSOBITS\nAll rights reserved.\n"
)

_MODEL_CONFIG = b"""<?xml version="1.0"?>
<model>
  <name>Standing person</name>
  <description>A standing person. Model created with Makehuman</description>
</model>
"""


def _model_sdf(mesh: str = "standing.dae", pose: str = "0 0 0.02 0.04 0 0") -> bytes:
    return f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="person_standing">
    <link name="link">
      <collision name="bottom">
        <pose>0 -0.1 0.01 0 0 0</pose>
        <geometry><box><size>0.5 0.35 0.02</size></box></geometry>
      </collision>
      <visual name="visual">
        <pose>{pose}</pose>
        <geometry>
          <mesh><uri>model://person_standing/meshes/{mesh}</uri></mesh>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
""".encode()


def _git(cwd: Path, *args: str) -> subprocess.CompletedProcess[str]:
    result = subprocess.run(
        ["git", "-C", str(cwd), *args],
        capture_output=True, text=True,
        env={**os.environ, "LC_ALL": "C"}, check=False,
    )
    if result.returncode != 0:
        raise AssertionError(f"git {' '.join(args)} failed: {result.stderr}")
    return result


def _build_checkout(root: Path, *, sdf: bytes | None = None, with_texture: bool = True) -> Path:
    checkout = root / "checkout"
    model_dir = checkout / "models" / "person_standing"
    (model_dir / "meshes").mkdir(parents=True)
    (model_dir / "model.config").write_bytes(_MODEL_CONFIG)
    (model_dir / "model.sdf").write_bytes(sdf if sdf is not None else _model_sdf())
    (model_dir / "meshes" / "standing.dae").write_bytes(b"fake-dae-bytes")
    if with_texture:
        tex = model_dir / "materials" / "textures"
        tex.mkdir(parents=True)
        (tex / "young_lightskinned_male_diffuse.png").write_bytes(b"fake-png")
        (tex / "jeans_basic_diffuse.png").write_bytes(b"fake-png-2")
    (checkout / "LICENSE").write_bytes(_LICENSE)
    _git(checkout, "init", "-q")
    _git(checkout, "config", "user.email", "fixture@example.invalid")
    _git(checkout, "config", "user.name", "Fixture")
    _git(checkout, "add", "-A")
    _git(checkout, "commit", "-q", "-m", "fixture person checkout")
    return checkout


def _base_config(commit: str) -> dict[str, object]:
    return {
        "repository": "https://example.invalid/sobits_gazebo_worlds",
        "branch": "feature/hri",
        "commit": commit,
        "models_root": "models",
        "person_allowlist": ["person_standing"],
        "collision_box_m": {
            "person_standing": {"min": [-0.28, -0.26, 0.0], "max": [0.27, 0.12, 1.91]}
        },
    }


class StubHooks:
    """Never imports Kit/pxr, so run_import is exercised without a GPU."""

    def __init__(self, write_texture: bool = False):
        self._write_texture = write_texture
        self.convert_calls: list[dict[str, object]] = []

    def convert_object_to_usd(self, dae_path: Path, stl_path: Path, usd_path: Path) -> None:
        self.convert_calls.append({"dae": dae_path, "stl": stl_path, "usd": usd_path})
        usd_path.parent.mkdir(parents=True, exist_ok=True)
        usd_path.write_bytes(b"stub-usd:" + dae_path.name.encode())
        if self._write_texture:
            person_id = usd_path.parent.name
            texture_dir = usd_path.parent / "textures" / person_id
            texture_dir.mkdir(parents=True, exist_ok=True)
            (texture_dir / "skin.png").write_bytes(b"fake-png-bytes")


class RunImportTest(unittest.TestCase):
    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.root = Path(self._tmp.name)
        self.checkout = _build_checkout(self.root)
        self.repo_root = self.root / "repo"
        self.commit = _git(self.checkout, "rev-parse", "HEAD").stdout.strip()

    def test_publishes_a_verifiable_artifact(self):
        publication = person_import.run_import(
            _base_config(self.commit), self.repo_root, self.checkout, StubHooks()
        )
        self.assertTrue(publication.created)
        d = publication.artifact_dir
        self.assertTrue((d / "person_standing" / "person.usd").is_file())
        self.assertTrue((d / "source-lock.json").is_file())
        self.assertTrue((d / "ATTRIBUTION.md").is_file())
        self.assertEqual(arena_artifact.verify_asset_artifact(d), [])

    def test_visual_mesh_is_resolved_through_the_model_sdf(self):
        """The mesh filename is upstream's to choose, not ours to hardcode."""
        hooks = StubHooks()
        person_import.run_import(_base_config(self.commit), self.repo_root, self.checkout, hooks)
        self.assertEqual(len(hooks.convert_calls), 1)
        call = hooks.convert_calls[0]
        self.assertEqual(Path(call["dae"]).name, "standing.dae")
        # Upstream ships no collision mesh, and the DAE cannot stand in for
        # one: arena_convert reads DAE and STL inputs under different unit
        # assumptions, so handing it the same file twice yielded a collider
        # exactly 100x the figure (caught live by _check_bounds_overlap).
        # The importer generates a metre-scale box proxy instead.
        self.assertNotEqual(call["dae"], call["stl"])
        self.assertEqual(Path(call["stl"]).suffix, ".stl")

    def test_a_renamed_upstream_mesh_is_followed(self):
        checkout = _build_checkout(self.root / "alt", sdf=_model_sdf(mesh="standing_v2.dae"))
        (checkout / "models" / "person_standing" / "meshes" / "standing_v2.dae").write_bytes(b"v2")
        _git(checkout, "add", "-A")
        _git(checkout, "commit", "-q", "-m", "rename mesh")
        commit = _git(checkout, "rev-parse", "HEAD").stdout.strip()
        hooks = StubHooks()
        person_import.run_import(_base_config(commit), self.repo_root / "alt", checkout, hooks)
        self.assertEqual(Path(hooks.convert_calls[0]["dae"]).name, "standing_v2.dae")

    def test_source_lock_records_mesh_textures_sdf_and_licence(self):
        publication = person_import.run_import(
            _base_config(self.commit), self.repo_root, self.checkout, StubHooks()
        )
        lock = json.loads((publication.artifact_dir / "source-lock.json").read_text())
        paths = {record["path"] for record in lock["records"]}
        self.assertIn("models/person_standing/meshes/standing.dae", paths)
        self.assertIn("models/person_standing/model.sdf", paths)
        self.assertIn(
            "models/person_standing/materials/textures/young_lightskinned_male_diffuse.png",
            paths,
        )
        self.assertIn("LICENSE", paths)
        self.assertEqual(lock["commit"], self.commit)

    def test_attribution_credits_the_model_author_and_licence(self):
        publication = person_import.run_import(
            _base_config(self.commit), self.repo_root, self.checkout, StubHooks()
        )
        text = (publication.artifact_dir / "ATTRIBUTION.md").read_text()
        self.assertIn("MakeHuman", text)
        self.assertIn("Marina Kollmitz", text)
        self.assertIn("BSD 3-Clause License", text)

    def test_relocated_textures_are_published_next_to_the_usd(self):
        publication = person_import.run_import(
            _base_config(self.commit), self.repo_root, self.checkout, StubHooks(write_texture=True)
        )
        self.assertTrue(
            (publication.artifact_dir / "person_standing" / "textures" / "person_standing" / "skin.png").is_file()
        )
        self.assertEqual(arena_artifact.verify_asset_artifact(publication.artifact_dir), [])


class CollisionProxyTest(unittest.TestCase):
    """Upstream ships no collision mesh for person_standing.

    Its SDF collides with the drawn mesh itself, but arena_convert's visual
    and collision inputs are read under different unit assumptions -- the DAE
    path trusts the stage's metersPerUnit, the STL path assumes metres -- so
    passing the same DAE for both produced a collider exactly 100x the
    figure. _check_bounds_overlap caught it. The importer therefore writes a
    metre-scale box STL sized to the figure's measured visual AABB.
    """

    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.root = Path(self._tmp.name)
        self.checkout = _build_checkout(self.root)
        self.repo_root = self.root / "repo"
        self.commit = _git(self.checkout, "rev-parse", "HEAD").stdout.strip()

    @staticmethod
    def _stl_bounds(stl_text: str):
        points = [
            tuple(float(value) for value in line.split()[1:4])
            for line in stl_text.splitlines()
            if line.strip().startswith("vertex")
        ]
        lo = tuple(min(point[axis] for point in points) for axis in range(3))
        hi = tuple(max(point[axis] for point in points) for axis in range(3))
        return lo, hi, len(points)

    def test_proxy_matches_the_configured_box(self):
        captured = {}

        class CapturingHooks(StubHooks):
            def convert_object_to_usd(self, dae_path, stl_path, usd_path):
                captured["stl"] = Path(stl_path).read_text(encoding="utf-8")
                super().convert_object_to_usd(dae_path, stl_path, usd_path)

        person_import.run_import(
            _base_config(self.commit), self.repo_root, self.checkout, CapturingHooks()
        )
        lo, hi, vertex_count = self._stl_bounds(captured["stl"])
        for axis, (expected_lo, expected_hi) in enumerate(
            zip((-0.28, -0.26, 0.0), (0.27, 0.12, 1.91))
        ):
            self.assertAlmostEqual(lo[axis], expected_lo, places=6)
            self.assertAlmostEqual(hi[axis], expected_hi, places=6)
        # A closed box is 12 triangles.
        self.assertEqual(vertex_count, 36)

    def test_proxy_is_a_parseable_ascii_stl(self):
        captured = {}

        class CapturingHooks(StubHooks):
            def convert_object_to_usd(self, dae_path, stl_path, usd_path):
                captured["stl"] = Path(stl_path).read_text(encoding="utf-8")
                super().convert_object_to_usd(dae_path, stl_path, usd_path)

        person_import.run_import(
            _base_config(self.commit), self.repo_root, self.checkout, CapturingHooks()
        )
        text = captured["stl"]
        self.assertTrue(text.startswith("solid "))
        self.assertTrue(text.rstrip().endswith("endsolid person_collision"))
        self.assertEqual(text.count("facet normal"), 12)
        self.assertEqual(text.count("endfacet"), 12)

    def test_a_model_without_a_collision_box_is_refused(self):
        config = _base_config(self.commit) | {"collision_box_m": {}}
        with self.assertRaises(arena_artifact.AssetArtifactError) as caught:
            person_import.run_import(config, self.repo_root, self.checkout, StubHooks())
        self.assertIn("collision_box_m", str(caught.exception))

    def test_an_inverted_box_is_refused(self):
        config = _base_config(self.commit) | {
            "collision_box_m": {
                "person_standing": {"min": [0.3, 0.3, 1.0], "max": [-0.3, -0.3, 0.0]}
            }
        }
        with self.assertRaises(arena_artifact.AssetArtifactError) as caught:
            person_import.run_import(config, self.repo_root, self.checkout, StubHooks())
        self.assertIn("collision box", str(caught.exception))

    def test_the_proxy_is_recorded_as_generated_not_upstream(self):
        """It is ours, so it must not appear in the upstream source-lock."""
        publication = person_import.run_import(
            _base_config(self.commit), self.repo_root, self.checkout, StubHooks()
        )
        lock = json.loads((publication.artifact_dir / "source-lock.json").read_text())
        self.assertFalse(
            [record for record in lock["records"] if str(record["path"]).endswith(".stl")]
        )


class UpstreamPoseGuardTest(unittest.TestCase):
    """Upstream declares a small visual pose this importer does not fold in.

    person_standing's SDF places its visual/collision at
    ``0 0 0.02 0.04 0 0`` -- 2 cm up and a 2.3 deg roll, a MakeHuman export
    artifact. Folding that into the USD needs Kit code that cannot be tested
    without a GPU, so the importer drops it and stands the figure upright.
    That is only defensible while the offset stays negligible, so the
    importer measures it and refuses anything larger rather than silently
    misplacing a person.
    """

    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.root = Path(self._tmp.name)
        self.repo_root = self.root / "repo"

    def _run(self, pose: str):
        checkout = _build_checkout(self.root, sdf=_model_sdf(pose=pose))
        commit = _git(checkout, "rev-parse", "HEAD").stdout.strip()
        return person_import.run_import(_base_config(commit), self.repo_root, checkout, StubHooks())

    def test_upstreams_small_offset_is_accepted(self):
        self.assertTrue(self._run("0 0 0.02 0.04 0 0").created)

    def test_identity_pose_is_accepted(self):
        self.assertTrue(self._run("0 0 0 0 0 0").created)

    def test_a_large_translation_is_refused(self):
        with self.assertRaises(arena_artifact.AssetArtifactError) as caught:
            self._run("0 0 0.9 0 0 0")
        self.assertIn("visual pose", str(caught.exception))

    def test_a_large_rotation_is_refused(self):
        with self.assertRaises(arena_artifact.AssetArtifactError) as caught:
            self._run("0 0 0 1.5708 0 0")
        self.assertIn("visual pose", str(caught.exception))


class MissingInputTest(unittest.TestCase):
    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.root = Path(self._tmp.name)
        self.repo_root = self.root / "repo"

    def test_missing_mesh_is_reported(self):
        checkout = _build_checkout(self.root)
        (checkout / "models" / "person_standing" / "meshes" / "standing.dae").unlink()
        _git(checkout, "add", "-A")
        _git(checkout, "commit", "-q", "-m", "drop mesh")
        commit = _git(checkout, "rev-parse", "HEAD").stdout.strip()
        with self.assertRaises(arena_artifact.AssetArtifactError) as caught:
            person_import.run_import(_base_config(commit), self.repo_root, checkout, StubHooks())
        self.assertIn("standing.dae", str(caught.exception))

    def test_empty_allowlist_is_refused(self):
        checkout = _build_checkout(self.root)
        commit = _git(checkout, "rev-parse", "HEAD").stdout.strip()
        config = _base_config(commit) | {"person_allowlist": []}
        with self.assertRaises(arena_artifact.AssetArtifactError):
            person_import.run_import(config, self.repo_root, checkout, StubHooks())


if __name__ == "__main__":
    unittest.main()
