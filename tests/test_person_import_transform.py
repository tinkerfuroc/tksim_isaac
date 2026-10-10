"""pxr side of the person importer's SDF transform fold.

``tools/person_import.py`` composes a model SDF's link pose, visual pose and
mesh scale into one 4x4 (column-vector convention, stdlib maths) and hands
it to ``arena_convert.convert_object_to_usd`` as ``visual_transform``;
``arena_convert._author_visual_transform`` turns that into a single
``xformOp:transform`` on the composed object's visual wrapper, ahead of the
unit-scale op ``_compose_object`` always authors. The one thing that can go
quietly wrong at that boundary is the matrix convention (``Gf.Matrix4d`` is
row-vector, translation in the last row), so these tests pin it down by
measuring the world bound of a known mesh after the ops are applied --
exactly what ``_compose_object``'s bounds guard measures live.

Also covers the two other pxr/Kit-adjacent changes the glTF figures needed
in ``arena_convert``: ``_prepare_dae_input`` leaving a binary ``.glb``
untouched, and ``author_preview_surface_material`` keeping the albedo map
of a ``gltf/pbr.mdl`` shader (``base_color_texture``) the way it already
keeps OmniPBR's ``diffuse_texture``.

importorskip pxr, as ``test_ycb_object_recenter.py`` does: present in the
sim venv, absent in a plain system-python run.
"""
from __future__ import annotations

import math
import sys
import tempfile
import unittest
from pathlib import Path

import pytest

pytest.importorskip("pxr")

from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import person_import  # noqa: E402
from tinker_sim_deploy import arena_convert  # noqa: E402
from tinker_sim_deploy.arena_artifact import AssetArtifactError  # noqa: E402


def _define_box_mesh(stage: Usd.Stage, path: str, lo, hi) -> UsdGeom.Mesh:
    (xmin, ymin, zmin), (xmax, ymax, zmax) = lo, hi
    mesh = UsdGeom.Mesh.Define(stage, path)
    corners = [
        (xmin, ymin, zmin), (xmax, ymin, zmin), (xmax, ymax, zmin), (xmin, ymax, zmin),
        (xmin, ymin, zmax), (xmax, ymin, zmax), (xmax, ymax, zmax), (xmin, ymax, zmax),
    ]
    mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in corners])
    faces = [[0, 1, 2, 3], [4, 5, 6, 7], [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]]
    mesh.CreateFaceVertexCountsAttr([4] * len(faces))
    mesh.CreateFaceVertexIndicesAttr([index for face in faces for index in face])
    return mesh


def _world_bound(prim) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"], False, True)
    aligned = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    return tuple(aligned.GetMin()), tuple(aligned.GetMax())


def _geom_stage(transform, unit_scale: float, lo, hi):
    """``_compose_object``'s visual wrapper in miniature: /World/geom carries
    [visual_transform?, unit scale], and the raw mesh sits on a child."""
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    root = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(root.GetPrim())
    geom = UsdGeom.Xform.Define(stage, "/World/geom")
    xformable = UsdGeom.Xformable(geom.GetPrim())
    if transform is not None:
        arena_convert._author_visual_transform(xformable, transform)
    xformable.AddScaleOp().Set(Gf.Vec3f(unit_scale, unit_scale, unit_scale))
    _define_box_mesh(stage, "/World/geom/mesh", lo, hi)
    return stage, geom.GetPrim()


class AuthorVisualTransformTest(unittest.TestCase):
    def test_owada_link_pose_and_scale_stand_a_unit_mesh_up_at_life_size(self):
        # The real owada_* case: Kit's glTF import lands the raw points in
        # centimetres (metersPerUnit=0.01 -> unit scale 0.01), Y-up, unit
        # normalised. SDF: link "0 0 0.825 1.5708 0 1.57", scale 1.65.
        spec = person_import._VisualSpec(
            mesh_relative="meshes/x.glb",
            link_pose=(0.0, 0.0, 0.825, 1.5708, 0.0, 1.57),
            visual_pose=(0.0,) * 6,
            scale=1.65,
        )
        transform = person_import._model_transform(spec)
        # raw mesh in cm: x [-20, 20], y [-50, 50] (height), z [-28, 28]
        stage, geom = _geom_stage(transform, 0.01, (-20.0, -50.0, -28.0), (20.0, 50.0, 28.0))
        lo, hi = _world_bound(geom)
        self.assertAlmostEqual(lo[2], 0.0, places=3)
        self.assertAlmostEqual(hi[2], 1.65, places=3)
        # yaw ~90 deg: raw x (0.4 m wide) becomes model y, raw z becomes model x
        self.assertAlmostEqual(hi[0] - lo[0], 0.56 * 1.65, places=2)
        self.assertAlmostEqual(hi[1] - lo[1], 0.40 * 1.65, places=2)

    def test_translation_lands_in_the_last_column_not_the_last_row(self):
        """The convention trap: a column-vector matrix authored untransposed
        would put the translation into the projective row and shear the
        mesh instead of moving it."""
        transform = (
            (1.0, 0.0, 0.0, 0.3),
            (0.0, 1.0, 0.0, -0.2),
            (0.0, 0.0, 1.0, 0.9),
            (0.0, 0.0, 0.0, 1.0),
        )
        stage, geom = _geom_stage(transform, 1.0, (0.0, 0.0, 0.0), (1.0, 1.0, 1.0))
        lo, hi = _world_bound(geom)
        self.assertEqual(tuple(round(v, 6) for v in lo), (0.3, -0.2, 0.9))
        self.assertEqual(tuple(round(v, 6) for v in hi), (1.3, 0.8, 1.9))

    def test_op_order_is_transform_then_unit_scale(self):
        transform = person_import._pose_matrix((0.0, 0.0, 0.5, 0.0, 0.0, 0.0))
        stage, geom = _geom_stage(transform, 0.01, (0.0, 0.0, 0.0), (100.0, 100.0, 100.0))
        order = UsdGeom.Xformable(geom).GetXformOpOrderAttr().Get()
        self.assertEqual(list(order), ["xformOp:transform", "xformOp:scale"])
        lo, hi = _world_bound(geom)
        # scale first (cm -> m), then translate: [0.5, 1.5], not [50.5, ...]
        self.assertAlmostEqual(lo[2], 0.5, places=6)
        self.assertAlmostEqual(hi[2], 1.5, places=6)

    def test_no_transform_authors_no_op(self):
        stage, geom = _geom_stage(None, 0.01, (0.0, 0.0, 0.0), (100.0, 100.0, 100.0))
        order = UsdGeom.Xformable(geom).GetXformOpOrderAttr().Get()
        self.assertEqual(list(order), ["xformOp:scale"])

    def test_matrix_shape_is_checked(self):
        stage = Usd.Stage.CreateInMemory()
        xformable = UsdGeom.Xformable(UsdGeom.Xform.Define(stage, "/x").GetPrim())
        with self.assertRaises(AssetArtifactError):
            arena_convert._author_visual_transform(xformable, ((1.0, 0.0), (0.0, 1.0)))

    def test_pose_matrix_matches_usd_rotate_xyz(self):
        """Gazebo's roll/pitch/yaw is R = Rz * Ry * Rx, the same composition
        USD's RotateXYZ op uses -- the furniture path relies on that, and the
        person path must agree with it for the same SDF numbers."""
        pose = (0.1, -0.2, 0.3, 0.4, -0.5, 0.6)
        ours = person_import._pose_matrix(pose)
        stage = Usd.Stage.CreateInMemory()
        xformable = UsdGeom.Xformable(UsdGeom.Xform.Define(stage, "/x").GetPrim())
        xformable.AddTranslateOp().Set(Gf.Vec3d(*pose[:3]))
        xformable.AddRotateXYZOp().Set(Gf.Vec3f(*(math.degrees(a) for a in pose[3:])))
        theirs = xformable.GetLocalTransformation()  # row-vector
        for row in range(4):
            for column in range(4):
                self.assertAlmostEqual(ours[row][column], theirs[column][row], places=6)


class PrepareGlbInputTest(unittest.TestCase):
    def test_binary_glb_is_handed_over_untouched(self):
        with tempfile.TemporaryDirectory() as tmp:
            glb_path = Path(tmp) / "sitting_woman.glb"
            glb_path.write_bytes(b"glTF\x02\x00\x00\x00\xff\xfe\x00\x00")
            result = arena_convert._prepare_dae_input(glb_path, Path(tmp) / "scratch")
            self.assertEqual(result, glb_path)
            self.assertFalse((Path(tmp) / "scratch").exists())


class GltfMaterialRewriteTest(unittest.TestCase):
    def _material_stage(self, input_name: str):
        stage = Usd.Stage.CreateInMemory()
        root = stage.DefinePrim("/World", "Xform")
        stage.SetDefaultPrim(root)
        UsdShade.Material.Define(stage, "/World/Looks/mat")
        shader_prim = stage.DefinePrim("/World/Looks/mat/mat", "Shader")
        shader_prim.CreateAttribute("info:mdl:sourceAsset", Sdf.ValueTypeNames.Asset).Set(
            Sdf.AssetPath("gltf/pbr.mdl")
        )
        shader_prim.CreateAttribute(input_name, Sdf.ValueTypeNames.Asset).Set(
            Sdf.AssetPath("./textures/sitting_woman/albedo.png")
        )
        return stage

    def test_gltf_base_color_texture_survives_the_rewrite(self):
        stage = self._material_stage("inputs:base_color_texture")
        self.assertEqual(arena_convert.author_preview_surface_material(stage), 1)
        texture_shader = stage.GetPrimAtPath("/World/Looks/mat/diffuseTex")
        self.assertTrue(texture_shader)
        self.assertEqual(
            str(texture_shader.GetAttribute("inputs:file").Get().path),
            "./textures/sitting_woman/albedo.png",
        )

    def test_omnipbr_diffuse_texture_still_survives(self):
        stage = self._material_stage("inputs:diffuse_texture")
        self.assertEqual(arena_convert.author_preview_surface_material(stage), 1)
        self.assertTrue(stage.GetPrimAtPath("/World/Looks/mat/diffuseTex"))


if __name__ == "__main__":
    unittest.main()
