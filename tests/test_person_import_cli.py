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

The glTF figures that joined it (``person_sitting``, ``owada_*``,
``standing_man_casual``, ``sitting_woman``) are Y-up, unit-normalised
meshes that only stand up at life size through their SDF's link pose and
mesh scale, so the importer now folds those into a ``visual_transform`` for
the converter (see ``GltfModelTransformTest``).

Covers only the pure-Python orchestration, exactly as
``test_ycb_import_cli.py`` does: pin verification, resolving the visual mesh
through the model SDF, dropping-vs-folding upstream's declared poses, and
publishing through ``arena_artifact``. The pxr side of the transform is in
``test_person_import_transform.py``; the real Kit conversion is exercised
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


def _glb_model_sdf(
    person_id: str,
    *,
    link_pose: str = "0 0 0.825 1.5708 0 1.57",
    visual_pose: str = "",
    scale: str = "1.65 1.65 1.65",
    leading_blank_line: bool = True,
) -> bytes:
    """The shape upstream's glTF person SDFs take (owada_*, sitting_woman,
    standing_man_casual, person_sitting): a <link><pose> standing the Y-up
    mesh upright, an optional <visual><pose> offset, a <mesh><scale> bringing
    the unit-normalised figure to life size -- and, for four of them, a
    blank line ahead of the XML declaration.
    """
    visual_pose_xml = f"<pose>{visual_pose}</pose>" if visual_pose else ""
    scale_xml = f"<scale>{scale}</scale>" if scale else ""
    text = f"""<?xml version="1.0"?>
<sdf version="1.10">
  <model name="{person_id}">
    <static>false</static>
    <link name="link">
      <pose>{link_pose}</pose>
      <visual name="visual">
        {visual_pose_xml}
        <geometry>
          <mesh>
            <uri>model://{person_id}/meshes/{person_id}.glb</uri>
            {scale_xml}
          </mesh>
        </geometry>
      </visual>
      <collision name="collision">
        <geometry>
          <mesh>
            <uri>model://{person_id}/meshes/{person_id}.glb</uri>
            {scale_xml}
          </mesh>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
"""
    return (("\n" if leading_blank_line else "") + text).encode()


_ACTOR_SDF = b"""<?xml version="1.0" ?>
<sdf version="1.8">
  <actor name="actor_walking">
    <skin>
      <filename>https://fuel.gazebosim.org/1.0/Mingfei/models/actor/tip/files/meshes/walk.dae</filename>
      <scale>1.0</scale>
    </skin>
    <animation name="walk">
      <filename>https://fuel.gazebosim.org/1.0/Mingfei/models/actor/tip/files/meshes/walk.dae</filename>
    </animation>
  </actor>
</sdf>
"""


def _placeholder_model_config(person_id: str) -> bytes:
    """Upstream ships the glTF figures with the SDF template's placeholders,
    and (like their SDFs) a blank line ahead of the XML declaration."""
    return f"""
<?xml version="1.0"?>
<model>
  <name>{person_id}</name>
  <version>1.0</version>
  <sdf version="1.10">model.sdf</sdf>
  <author>
    <name>Author Name</name>
    <email>author@example.com</email>
  </author>
  <description>
    A description of the {person_id} model.
  </description>
</model>
""".encode()


def _add_model(checkout: Path, person_id: str, sdf: bytes, *, mesh_name: str | None = None) -> None:
    """Add a second model directory to a fixture checkout and commit it."""
    model_dir = checkout / "models" / person_id
    (model_dir / "meshes").mkdir(parents=True)
    (model_dir / "model.config").write_bytes(_placeholder_model_config(person_id))
    (model_dir / "model.sdf").write_bytes(sdf)
    if mesh_name is not None:
        (model_dir / "meshes" / mesh_name).write_bytes(b"glTF-binary-bytes")
    _git(checkout, "add", "-A")
    _git(checkout, "commit", "-q", "-m", f"add {person_id}")


def _with_box(config: dict[str, object], person_id: str, minimum, maximum) -> dict[str, object]:
    boxes = dict(config["collision_box_m"])  # type: ignore[arg-type]
    boxes[person_id] = {"min": list(minimum), "max": list(maximum)}
    allowlist = list(config["person_allowlist"])  # type: ignore[arg-type]
    if person_id not in allowlist:
        allowlist.append(person_id)
    return config | {"collision_box_m": boxes, "person_allowlist": allowlist}


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

    def convert_object_to_usd(
        self, dae_path: Path, stl_path: Path, usd_path: Path, *, visual_transform=None
    ) -> None:
        self.convert_calls.append(
            {"dae": dae_path, "stl": stl_path, "usd": usd_path, "transform": visual_transform}
        )
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
            def convert_object_to_usd(self, dae_path, stl_path, usd_path, **kwargs):
                captured["stl"] = Path(stl_path).read_text(encoding="utf-8")
                super().convert_object_to_usd(dae_path, stl_path, usd_path, **kwargs)

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
            def convert_object_to_usd(self, dae_path, stl_path, usd_path, **kwargs):
                captured["stl"] = Path(stl_path).read_text(encoding="utf-8")
                super().convert_object_to_usd(dae_path, stl_path, usd_path, **kwargs)

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
    """Small SDF poses are dropped, large ones are folded.

    person_standing's SDF places its visual/collision at
    ``0 0 0.02 0.04 0 0`` -- 2 cm up and a 2.3 deg roll, a MakeHuman export
    artifact -- and the importer drops it so the figure stands exactly
    upright and the conversion stays byte-identical to the published
    artifact (the converter is handed ``visual_transform=None``). Anything
    beyond that bound is real placement (the glTF figures' 90 deg roll and
    seat heights) and is folded in as a matrix instead of refused.
    """

    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.root = Path(self._tmp.name)
        self.repo_root = self.root / "repo"

    def _run(self, pose: str) -> StubHooks:
        checkout = _build_checkout(self.root, sdf=_model_sdf(pose=pose))
        commit = _git(checkout, "rev-parse", "HEAD").stdout.strip()
        hooks = StubHooks()
        publication = person_import.run_import(_base_config(commit), self.repo_root, checkout, hooks)
        self.assertTrue(publication.created)
        return hooks

    def test_upstreams_small_offset_is_dropped(self):
        self.assertIsNone(self._run("0 0 0.02 0.04 0 0").convert_calls[0]["transform"])

    def test_identity_pose_is_dropped(self):
        self.assertIsNone(self._run("0 0 0 0 0 0").convert_calls[0]["transform"])

    def test_a_large_translation_is_folded(self):
        transform = self._run("0 0 0.9 0 0 0").convert_calls[0]["transform"]
        self.assertIsNotNone(transform)
        self.assertAlmostEqual(transform[2][3], 0.9, places=9)
        for row in range(3):
            self.assertAlmostEqual(transform[row][row], 1.0, places=9)

    def test_a_large_rotation_is_folded(self):
        transform = self._run("0 0 0 1.5708 0 0").convert_calls[0]["transform"]
        self.assertIsNotNone(transform)
        # roll 90 deg about X: mesh +Y becomes model +Z
        self.assertAlmostEqual(_apply(transform, (0.0, 1.0, 0.0))[2], 1.0, places=4)


def _apply(matrix, point):
    x, y, z = point
    return tuple(
        matrix[row][0] * x + matrix[row][1] * y + matrix[row][2] * z + matrix[row][3]
        for row in range(3)
    )


class GltfModelTransformTest(unittest.TestCase):
    """The glTF figures only look like people in Gazebo because of their
    SDF: a Y-up, unit-normalised mesh stood up by ``<link><pose>`` (roll 90
    deg plus a yaw), sometimes offset by ``<visual><pose>``, and brought to
    life size by ``<mesh><scale>``. The importer folds all of that into one
    matrix (``model = link_pose * visual_pose * scale * mesh``) for the
    converter, so ``collision_box_m`` is always expressed in the model
    frame. Numbers below are checked against the same transform applied
    with numpy/trimesh to the real pinned meshes.
    """

    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.root = Path(self._tmp.name)
        self.repo_root = self.root / "repo"
        self.checkout = _build_checkout(self.root)

    def _import(self, person_id: str, sdf: bytes, box=((-0.5, -0.5, 0.0), (0.5, 0.5, 1.65))) -> dict:
        _add_model(self.checkout, person_id, sdf, mesh_name=f"{person_id}.glb")
        commit = _git(self.checkout, "rev-parse", "HEAD").stdout.strip()
        config = _with_box(_base_config(commit), person_id, *box)
        hooks = StubHooks()
        person_import.run_import(config, self.repo_root, self.checkout, hooks)
        calls = {Path(call["usd"]).parent.name: call for call in hooks.convert_calls}
        return calls[person_id]

    def test_glb_mesh_is_resolved_and_a_leading_blank_line_is_tolerated(self):
        call = self._import("owada_left_hand_raising", _glb_model_sdf("owada_left_hand_raising"))
        self.assertEqual(Path(call["dae"]).name, "owada_left_hand_raising.glb")

    def test_link_roll_and_scale_stand_the_figure_up_at_life_size(self):
        # owada_*: link pose "0 0 0.825 1.5708 0 1.57", scale 1.65 -- the raw
        # mesh's Y extent [-0.5, 0.5] must land on model Z [0, 1.65].
        transform = self._import("owada_left_hand_raising", _glb_model_sdf("owada_left_hand_raising"))["transform"]
        self.assertIsNotNone(transform)
        top = _apply(transform, (0.0, 0.5, 0.0))
        bottom = _apply(transform, (0.0, -0.5, 0.0))
        self.assertAlmostEqual(top[2], 1.65, places=4)
        self.assertAlmostEqual(bottom[2], 0.0, places=4)
        # yaw 1.57: the mesh's +X (its right) turns to model +Y
        side = _apply(transform, (1.0, 0.0, 0.0))
        self.assertAlmostEqual(side[1], 1.65, places=2)
        self.assertAlmostEqual(side[0], 0.0, places=2)

    def test_visual_pose_offset_composes_after_scale(self):
        # standing_man_casual: link "0 0 0 1.5708 0 1.57", visual "0 0.4 0 0 0 0",
        # scale 1.8. Gazebo does not scale the visual offset, so the figure's
        # feet land at z = -0.9 + 0.4 = -0.5 and its head at z = 1.3 -- which is
        # why its configured collision box spans z [-0.5, 1.3], not [0, 1.8].
        transform = self._import(
            "standing_man_casual",
            _glb_model_sdf(
                "standing_man_casual",
                link_pose="0 0 0 1.5708 0 1.57",
                visual_pose="0 0.4 0 0 0 0",
                scale="1.8000 1.8000 1.8000",
            ),
            box=((-0.19, -0.28, -0.5), (0.19, 0.28, 1.3)),
        )["transform"]
        self.assertAlmostEqual(_apply(transform, (0.0, -0.5, 0.0))[2], -0.5, places=4)
        self.assertAlmostEqual(_apply(transform, (0.0, 0.5, 0.0))[2], 1.3, places=4)

    def test_person_sitting_scale_below_one_is_honoured(self):
        # person_sitting: link "0 0 0.675 1.57 0 0", scale 0.6872 (Blender
        # export, ~1.96 raw units tall -> 1.35 m).
        transform = self._import(
            "person_sitting",
            _glb_model_sdf("person_sitting", link_pose="0 0 0.675 1.57 0 0", scale="0.6872 0.6872 0.6872"),
            box=((-0.48, -0.29, 0.0), (0.45, 0.29, 1.35)),
        )["transform"]
        self.assertAlmostEqual(_apply(transform, (0.0, 0.9632, 0.0))[2], 0.675 + 0.9632 * 0.6872, places=3)

    def test_non_uniform_scale_is_refused(self):
        with self.assertRaises(arena_artifact.AssetArtifactError) as caught:
            self._import("squashed", _glb_model_sdf("squashed", scale="1 2 1"))
        self.assertIn("non-uniform", str(caught.exception))

    def test_a_gazebo_actor_is_refused_with_a_reason(self):
        """person_walking: an <actor> whose skin is a remote Fuel URL --
        nothing static in the checkout to convert."""
        with self.assertRaises(arena_artifact.AssetArtifactError) as caught:
            self._import("person_walking", _ACTOR_SDF)
        self.assertIn("<actor>", str(caught.exception))

    def test_person_standing_still_converts_without_a_transform(self):
        """Adding glTF figures must not disturb the DAE figure's conversion."""
        _add_model(self.checkout, "sitting_woman", _glb_model_sdf("sitting_woman"), mesh_name="sitting_woman.glb")
        commit = _git(self.checkout, "rev-parse", "HEAD").stdout.strip()
        config = _with_box(_base_config(commit), "sitting_woman", (-0.5, -0.5, 0.0), (0.5, 0.5, 1.3))
        hooks = StubHooks()
        publication = person_import.run_import(config, self.repo_root, self.checkout, hooks)
        calls = {Path(call["usd"]).parent.name: call for call in hooks.convert_calls}
        self.assertIsNone(calls["person_standing"]["transform"])
        self.assertIsNotNone(calls["sitting_woman"]["transform"])
        self.assertTrue((publication.artifact_dir / "sitting_woman" / "person.usd").is_file())
        self.assertTrue((publication.artifact_dir / "person_standing" / "person.usd").is_file())
        self.assertEqual(arena_artifact.verify_asset_artifact(publication.artifact_dir), [])


class MultiModelAttributionTest(unittest.TestCase):
    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.root = Path(self._tmp.name)
        self.repo_root = self.root / "repo"
        self.checkout = _build_checkout(self.root)
        _add_model(self.checkout, "sitting_woman", _glb_model_sdf("sitting_woman"), mesh_name="sitting_woman.glb")
        commit = _git(self.checkout, "rev-parse", "HEAD").stdout.strip()
        config = _with_box(_base_config(commit), "sitting_woman", (-0.5, -0.5, 0.0), (0.5, 0.5, 1.3))
        self.publication = person_import.run_import(config, self.repo_root, self.checkout, StubHooks())

    def test_each_figure_gets_its_own_section(self):
        text = (self.publication.artifact_dir / "ATTRIBUTION.md").read_text()
        self.assertIn("person_standing", text)
        self.assertIn("Marina Kollmitz", text)
        self.assertIn("sitting_woman", text)
        # Upstream's placeholder metadata is reproduced, not embellished.
        self.assertIn("Author Name", text)
        self.assertIn("A description of the sitting_woman model.", text)
        self.assertIn("BSD 3-Clause License", text)

    def test_model_config_is_in_the_source_lock(self):
        lock = json.loads((self.publication.artifact_dir / "source-lock.json").read_text())
        paths = {record["path"] for record in lock["records"]}
        self.assertIn("models/sitting_woman/model.config", paths)
        self.assertIn("models/sitting_woman/meshes/sitting_woman.glb", paths)
        self.assertIn("models/sitting_woman/model.sdf", paths)


class RealConfigTest(unittest.TestCase):
    """``config/person-import.json`` as committed: every allowlisted figure
    has a plausible collision box. The boxes are the figures' measured
    model-frame AABBs (trimesh over the pinned checkout with the SDF
    transform applied), so a person is between 1.2 m and 2.0 m tall and
    narrower than it is tall.
    """

    def setUp(self) -> None:
        self.config = json.loads((ROOT / "config" / "person-import.json").read_text(encoding="utf-8"))

    def test_every_allowlisted_person_has_a_box(self):
        for person_id in self.config["person_allowlist"]:
            minimum, maximum = person_import._collision_box(self.config, person_id)
            size = [hi - lo for lo, hi in zip(minimum, maximum)]
            self.assertGreaterEqual(size[2], 1.2, person_id)
            self.assertLessEqual(size[2], 2.0, person_id)
            self.assertLess(size[0], size[2], person_id)
            self.assertLess(size[1], size[2], person_id)

    def test_no_box_without_an_allowlisted_person(self):
        self.assertEqual(
            set(self.config["collision_box_m"]), set(self.config["person_allowlist"])
        )

    def test_the_gesture_and_pose_figures_are_allowlisted(self):
        """The keys gpsr_scene's pose/gesture fallback chains look up."""
        expected = {
            "person_standing",
            "person_sitting",
            "owada_left_hand_raising",
            "owada_right_hand_raising",
            "standing_man_casual",
            "sitting_woman",
        }
        self.assertTrue(expected <= set(self.config["person_allowlist"]))
        # person_walking is a Gazebo <actor> with a remote skin: not importable.
        self.assertNotIn("person_walking", self.config["person_allowlist"])


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
