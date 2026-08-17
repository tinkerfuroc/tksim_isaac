"""Pure-Python logic extracted from ``tinker_sim_deploy.arena_convert``.

``arena_convert`` is otherwise live-only (every Kit/pxr call is lazily
imported inside a function body and needs a running SimulationApp -- see
the Task 9 report for the live-run evidence). These two helpers have no
pxr dependency at all, so they are unit-tested here under plain system
Python: the review round that added ``_relocate_local_assets`` (Finding 1
-- published furniture USDs must never embed a random per-run scratch
path) split its path-selection/destination-naming logic out specifically
so it could be covered this way.
"""
from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from tinker_sim_deploy import arena_convert  # noqa: E402


class TextureRelocationTargetTest(unittest.TestCase):
    def test_relative_path_namespaced_under_textures_and_model_id(self):
        target = arena_convert._texture_relocation_target(
            "rcw26_kitchen_table", "kitchen_table_texture0.png"
        )
        self.assertEqual(target, "./textures/rcw26_kitchen_table/kitchen_table_texture0.png")

    def test_two_models_never_collide_even_with_the_same_filename(self):
        first = arena_convert._texture_relocation_target("rcw26_bed", "texture0.png")
        second = arena_convert._texture_relocation_target("rcw26_chair", "texture0.png")
        self.assertNotEqual(first, second)


class PatchDaeMissingUnitTest(unittest.TestCase):
    """Task 10: every allowlisted YCB object's ``textured.dae`` omits the
    optional COLLADA ``<unit>`` element, which segfaults Kit's asset
    converter live (confirmed against the real pinned ``tmc_wrs_gz``
    checkout -- see the Task 10 report). ``_patch_dae_missing_unit`` is the
    pure-Python decision logic for that workaround; no pxr/Kit dependency,
    so it is unit-tested here.
    """

    def test_missing_unit_gets_the_collada_spec_default_injected(self):
        dae = b'<COLLADA><asset><up_axis>Y_UP</up_axis></asset></COLLADA>'

        patched = arena_convert._patch_dae_missing_unit(dae)

        self.assertIsNotNone(patched)
        self.assertIn(b'<unit name="meter" meter="1"/>', patched)
        # inserted strictly before <up_axis>, inside the same <asset> block
        self.assertLess(patched.index(b"<unit"), patched.index(b"<up_axis>"))

    def test_existing_unit_element_is_left_untouched(self):
        dae = b'<COLLADA><asset><unit meter="0.01" name="centimeter"/><up_axis>Y_UP</up_axis></asset></COLLADA>'

        self.assertIsNone(arena_convert._patch_dae_missing_unit(dae))

    def test_no_up_axis_anchor_is_left_untouched(self):
        dae = b"<COLLADA><asset></asset></COLLADA>"

        self.assertIsNone(arena_convert._patch_dae_missing_unit(dae))


class PrepareDaeInputTest(unittest.TestCase):
    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.root = Path(self._tmp.name)

    def test_dae_with_unit_element_is_returned_unchanged(self):
        mesh_dir = self.root / "meshes"
        mesh_dir.mkdir()
        dae_path = mesh_dir / "textured.dae"
        dae_path.write_bytes(b'<COLLADA><asset><unit meter="1"/><up_axis>Y_UP</up_axis></asset></COLLADA>')

        result = arena_convert._prepare_dae_input(dae_path, self.root / "scratch")

        self.assertEqual(result, dae_path)

    def test_dae_missing_unit_is_mirrored_with_siblings_preserved(self):
        mesh_dir = self.root / "meshes"
        mesh_dir.mkdir()
        dae_path = mesh_dir / "textured.dae"
        dae_path.write_bytes(
            b'<COLLADA><asset><up_axis>Y_UP</up_axis></asset>'
            b'<library_images><image><init_from>tex.png</init_from></image></library_images></COLLADA>'
        )
        texture_path = mesh_dir / "tex.png"
        texture_path.write_bytes(b"fake-png-bytes")
        (mesh_dir / "nontextured.stl").write_bytes(b"fake-stl-bytes")

        scratch_dir = self.root / "scratch"
        result = arena_convert._prepare_dae_input(dae_path, scratch_dir)

        self.assertNotEqual(result, dae_path)
        self.assertEqual(result.parent, scratch_dir / "_dae_input")
        self.assertIn(b'<unit name="meter" meter="1"/>', result.read_bytes())
        # sibling texture (referenced by relative <init_from>) is mirrored
        # alongside the patched DAE so the relative reference still resolves
        self.assertTrue((scratch_dir / "_dae_input" / "tex.png").is_file())
        self.assertEqual((scratch_dir / "_dae_input" / "tex.png").read_bytes(), b"fake-png-bytes")
        # the original upstream file is never mutated in place
        self.assertNotIn(b"<unit", dae_path.read_bytes())


class RelocatableAssetSourceTest(unittest.TestCase):
    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.search_root = Path(self._tmp.name) / "scratch" / "furniture"
        self.search_root.mkdir(parents=True)

    def test_absolute_existing_file_under_search_root_is_relocated(self):
        texture = self.search_root / "textures" / "kitchen_table_texture0.png"
        texture.parent.mkdir(parents=True)
        texture.write_bytes(b"fake-png-bytes")

        result = arena_convert._relocatable_asset_source(str(texture), self.search_root)

        self.assertEqual(result, texture)

    def test_bare_mdl_library_reference_is_not_relocated(self):
        # e.g. a UsdShade shader's info:mdl:sourceAsset = "gltf/pbr.mdl" --
        # not an absolute filesystem path at authoring time; resolved via
        # Kit's own built-in MDL search path wherever the USD is opened.
        result = arena_convert._relocatable_asset_source("gltf/pbr.mdl", self.search_root)
        self.assertIsNone(result)

    def test_absolute_path_outside_search_root_is_not_relocated(self):
        outside = self.search_root.parent.parent / "somewhere-else" / "texture.png"
        outside.parent.mkdir(parents=True)
        outside.write_bytes(b"fake-png-bytes")

        result = arena_convert._relocatable_asset_source(str(outside), self.search_root)

        self.assertIsNone(result)

    def test_absolute_path_under_search_root_that_does_not_exist_is_not_relocated(self):
        missing = self.search_root / "textures" / "does_not_exist.png"

        result = arena_convert._relocatable_asset_source(str(missing), self.search_root)

        self.assertIsNone(result)

    def test_empty_or_none_path_is_not_relocated(self):
        self.assertIsNone(arena_convert._relocatable_asset_source("", self.search_root))
        self.assertIsNone(arena_convert._relocatable_asset_source(None, self.search_root))


if __name__ == "__main__":
    unittest.main()
