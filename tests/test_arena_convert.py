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
from tinker_sim_deploy.arena_artifact import AssetArtifactError  # noqa: E402


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

        patched = arena_convert._patch_dae_missing_unit(dae, "textured.dae")

        self.assertIsNotNone(patched)
        self.assertIn(b'<unit name="meter" meter="1"/>', patched)
        # inserted strictly before <up_axis>, inside the same <asset> block
        self.assertLess(patched.index(b"<unit"), patched.index(b"<up_axis>"))

    def test_existing_unit_element_is_left_untouched(self):
        dae = b'<COLLADA><asset><unit meter="0.01" name="centimeter"/><up_axis>Y_UP</up_axis></asset></COLLADA>'

        self.assertIsNone(arena_convert._patch_dae_missing_unit(dae, "textured.dae"))

    def test_no_up_axis_anchor_fails_closed(self):
        # Review fix (Finding 2): an earlier version returned None here,
        # silently falling through to feed the original, still
        # crash-triggering bytes straight to Kit -- a deferred native
        # segfault instead of an actionable Python exception. Must raise
        # instead, naming the file.
        dae = b"<COLLADA><asset></asset></COLLADA>"

        with self.assertRaises(AssetArtifactError) as ctx:
            arena_convert._patch_dae_missing_unit(dae, "models/ycb_999_widget/meshes/textured.dae")

        self.assertIn("models/ycb_999_widget/meshes/textured.dae", str(ctx.exception))
        self.assertIn("<unit>", str(ctx.exception))
        self.assertIn("<up_axis>", str(ctx.exception))


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


class CheckBoundsOverlapTest(unittest.TestCase):
    """Task 10 review round, Finding 1: ``_compose_object`` had no runtime
    check that ``_axis_correction``'s "STL scale=1.0, no rotation, trust
    DAE metersPerUnit" decision actually produced two co-located,
    similarly-sized meshes -- a future re-pin or allowlist addition whose
    STL is not already metre-expressed (or that genuinely needs a
    rotation) would previously have published a silently misaligned
    collision mesh. ``_check_bounds_overlap`` is the pure-Python decision
    logic for that guard (no pxr dependency), unit-tested here.
    """

    def test_in_range_ratio_on_every_axis_passes(self):
        geom_bound = ((-0.05, -0.10, -0.01), (0.02, 0.07, 0.21))
        collision_bound = ((-0.05, -0.10, -0.01), (0.02, 0.07, 0.21))

        arena_convert._check_bounds_overlap(geom_bound, collision_bound)  # must not raise

    def test_roughly_matching_but_not_identical_bounds_pass(self):
        # collision mesh is a coarser decimation of the same object -- not
        # byte-identical bounds, but well within the tolerance window
        geom_bound = ((-0.0488, -0.0962, -0.0032), (0.0230, 0.0679, 0.2102))
        collision_bound = ((-0.0488, -0.0962, -0.0031), (0.0226, 0.0664, 0.2099))

        arena_convert._check_bounds_overlap(geom_bound, collision_bound)  # must not raise

    def test_scale_error_out_of_range_fails_closed(self):
        # the exact ~100x collision-mesh shrink this task hit live before
        # the fix (STL wrongly scaled by the raw stage's irrelevant
        # metersPerUnit=0.01 stamp)
        geom_bound = ((-0.05, -0.10, -0.01), (0.02, 0.07, 0.21))
        collision_bound = ((-0.0005, -0.0010, -0.0001), (0.0002, 0.0007, 0.0021))

        with self.assertRaises(AssetArtifactError) as ctx:
            arena_convert._check_bounds_overlap(geom_bound, collision_bound)
        self.assertIn(str(geom_bound), str(ctx.exception))
        self.assertIn(str(collision_bound), str(ctx.exception))

    def test_axis_swap_on_an_elongated_object_fails_closed(self):
        # A per-axis size-ratio check only catches an axis-swap-style
        # rotation bug (like the live one this task hit on the DAE side)
        # when the swapped axes actually differ enough in size -- it is not
        # a full geometric alignment check (deliberately: the review asked
        # for exactly this ratio-based guard, not e.g. a centroid/IoU
        # check). A tall, thin object (Z much longer than X/Y) with a
        # spurious 90-degree rotation swapping Y and Z demonstrates the
        # guard catching that shape of error when it is large enough to
        # matter; see the Task 10 fix report for why this task's *actual*
        # live rotation bug on the cracker box was not itself reliably
        # distinguishable this way (its three axis extents were all within
        # ~3x of each other).
        geom_bound = ((-0.025, -0.025, -0.15), (0.025, 0.025, 0.15))  # sizes: 0.05, 0.05, 0.30
        collision_bound = ((-0.025, -0.15, -0.025), (0.025, 0.15, 0.025))  # Y/Z swapped: 0.05, 0.30, 0.05

        with self.assertRaises(AssetArtifactError):
            arena_convert._check_bounds_overlap(geom_bound, collision_bound)

    def test_degenerate_zero_extent_fails_closed(self):
        geom_bound = ((-0.05, -0.10, -0.01), (0.02, 0.07, 0.21))
        collision_bound = ((0.0, -0.10, -0.01), (0.0, 0.07, 0.21))  # zero X extent

        with self.assertRaises(AssetArtifactError):
            arena_convert._check_bounds_overlap(geom_bound, collision_bound)

    def test_custom_tolerance_is_respected(self):
        geom_bound = ((0.0, 0.0, 0.0), (1.0, 1.0, 1.0))
        collision_bound = ((0.0, 0.0, 0.0), (0.6, 0.6, 0.6))  # ratio 0.6

        with self.assertRaises(AssetArtifactError):
            arena_convert._check_bounds_overlap(geom_bound, collision_bound, min_ratio=0.8, max_ratio=1.2)
        arena_convert._check_bounds_overlap(geom_bound, collision_bound, min_ratio=0.5, max_ratio=2.0)


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


class FurnitureMaterialTest(unittest.TestCase):
    """Solid-PBR tint override for arena furniture. Pure lookup, no pxr:
    ``compose_arena`` binds the returned color over each GLB's own
    (evidently non-resolving) material so nothing renders stark-white.
    """

    #: The rcw2026 import allowlist (config/arena-import.json). Every one
    #: must resolve to a deliberate, non-white color.
    RCW2026_MODELS = (
        "rcw26_bed", "rcw26_chair", "rcw26_cushion", "rcw26_dishwasher_close",
        "rcw26_dishwasher_open", "rcw26_door", "rcw26_kitchen_table",
        "rcw26_laundry_basket", "rcw26_laundry_desk", "rcw26_plant_mid",
        "rcw26_plant_tall", "rcw26_refrigerator", "rcw26_shelf",
        "rcw26_side_table", "rcw26_sink", "rcw26_sofa", "rcw26_stand",
        "rcw26_trashbin", "rcw26_tv", "rcw26_tv_stand",
        "rcw26_washing_machine", "rcw26_washing_machine_open",
    )

    def test_every_allowlisted_model_has_an_explicit_entry(self):
        for model_id in self.RCW2026_MODELS:
            self.assertIn(model_id, arena_convert.FURNITURE_COLORS, model_id)

    def test_material_returns_rgb_and_roughness_in_range(self):
        for model_id in self.RCW2026_MODELS:
            (r, g, b), roughness = arena_convert.furniture_material(model_id)
            for channel in (r, g, b):
                self.assertGreaterEqual(channel, 0.0, model_id)
                self.assertLessEqual(channel, 1.0, model_id)
            self.assertGreaterEqual(roughness, 0.0, model_id)
            self.assertLessEqual(roughness, 1.0, model_id)

    def test_no_allowlisted_model_renders_stark_white(self):
        # "stark-white" == every channel above 0.92. Appliances are allowed
        # to be light/off-white but must stay below that ceiling so they read
        # as an intentional material, not an untextured default.
        for model_id in self.RCW2026_MODELS:
            (r, g, b), _ = arena_convert.furniture_material(model_id)
            self.assertFalse(
                r > 0.92 and g > 0.92 and b > 0.92,
                f"{model_id} is stark-white: {(r, g, b)}",
            )

    def test_unlisted_model_hits_the_non_white_fallback(self):
        (r, g, b), roughness = arena_convert.furniture_material("rcw26_not_a_real_model")
        self.assertFalse(r > 0.92 and g > 0.92 and b > 0.92)
        self.assertGreaterEqual(roughness, 0.0)
        self.assertLessEqual(roughness, 1.0)


class FloorSlabTest(unittest.TestCase):
    """Geometry of the visual wood floor slab authored into ``arena.usd``.
    Pure function of the wall boxes; no pxr.
    """

    @staticmethod
    def _wall(name, size, center):
        from tinker_sim_deploy.arena_world import WallBox

        return WallBox(name=name, size=size, center=center, yaw=0.0)

    def _layout(self, walls):
        from tinker_sim_deploy.arena_world import ArenaLayout

        return ArenaLayout(walls=tuple(walls), furniture=())

    @staticmethod
    def _room(cx, cy):
        # Four thin walls whose outer faces bound a 10 (x) by 6 (y) room
        # centered on (cx, cy); wall thickness 0.2. Outer AABB is exactly
        # x in [cx-5, cx+5], y in [cy-3, cy+3].
        return [
            FloorSlabTest._wall("top", (10.0, 0.2, 2.4), (cx, cy + 2.9, 1.2)),
            FloorSlabTest._wall("bottom", (10.0, 0.2, 2.4), (cx, cy - 2.9, 1.2)),
            FloorSlabTest._wall("right", (0.2, 6.0, 2.4), (cx + 4.9, cy, 1.2)),
            FloorSlabTest._wall("left", (0.2, 6.0, 2.4), (cx - 4.9, cy, 1.2)),
        ]

    def test_covers_wall_footprint_with_margin(self):
        margin = 0.10
        center, size = arena_convert.floor_slab(
            self._layout(self._room(0.0, 0.0)), margin=margin
        )
        # footprint AABB: x [-5,5] -> width 10, y [-3,3] -> depth 6.
        self.assertAlmostEqual(size[0], 10.0 + 2 * margin)
        self.assertAlmostEqual(size[1], 6.0 + 2 * margin)
        self.assertAlmostEqual(center[0], 0.0)
        self.assertAlmostEqual(center[1], 0.0)

    def test_top_face_sits_just_above_ground_plane(self):
        walls = [self._wall("w0", (4.0, 4.0, 2.4), (0.0, 0.0, 1.2))]
        lift, thickness = 0.002, 0.02
        center, size = arena_convert.floor_slab(
            self._layout(walls), lift=lift, thickness=thickness
        )
        self.assertAlmostEqual(size[2], thickness)
        # center z + half-thickness == lift (top face just above z=0)
        self.assertAlmostEqual(center[2] + size[2] / 2.0, lift)

    def test_offcenter_footprint_centers_slab_on_its_midpoint(self):
        # Same room translated to (5, 12); slab must recenter there.
        center, size = arena_convert.floor_slab(
            self._layout(self._room(5.0, 12.0)), margin=0.0
        )
        self.assertAlmostEqual(center[0], 5.0)
        self.assertAlmostEqual(center[1], 12.0)
        self.assertAlmostEqual(size[0], 10.0)
        self.assertAlmostEqual(size[1], 6.0)


if __name__ == "__main__":
    unittest.main()
