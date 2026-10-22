"""Authoring and reading CoACD hull colliders in converted furniture USDs."""
from __future__ import annotations

import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))


def _write_l_usd(path: Path) -> None:
    """Default prim /World; /World/geom (translate op) with an L from two meshes (one quads)."""
    from pxr import Gf, Usd, UsdGeom

    stage = Usd.Stage.CreateNew(str(path))
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    geom = UsdGeom.Xform.Define(stage, "/World/geom")
    geom.AddTranslateOp().Set(Gf.Vec3d(1.0, 2.0, 3.0))

    def box(name, lo, hi, quads):
        x0, y0, z0 = lo
        x1, y1, z1 = hi
        pts = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
               (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
        quad_faces = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
                      (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
        mesh = UsdGeom.Mesh.Define(stage, f"/World/geom/{name}")
        mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in pts])
        if quads:
            counts = [4] * 6
            idx = [i for f in quad_faces for i in f]
        else:
            tris = [t for f in quad_faces for t in ((f[0], f[1], f[2]), (f[0], f[2], f[3]))]
            counts = [3] * len(tris)
            idx = [i for t in tris for i in t]
        mesh.CreateFaceVertexCountsAttr(counts)
        mesh.CreateFaceVertexIndicesAttr(idx)

    box("base", (0, 0, 0), (1, 0.4, 0.2), True)
    box("wall", (0, 0, 0.2), (0.2, 0.4, 1.0), False)
    stage.GetRootLayer().Save()


class CoacdUsdTests(unittest.TestCase):
    def setUp(self):
        # pxr's native modules must load before coacd's, or the first
        # ``from pxr import Usd`` segfaults.
        pytest.importorskip("pxr.Usd")
        pytest.importorskip("coacd")
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.usd = Path(self._tmp.name) / "l.usd"
        _write_l_usd(self.usd)
        from tinker_sim_deploy import arena_convert
        from tinker_sim_deploy.arena_coacd import CoacdParams

        self.convert = arena_convert
        self.params = CoacdParams.from_config(
            json.load(open(ROOT / "config/arena-import.json"))["coacd"]
        )

    def test_read_visual_mesh_triangulates_and_merges(self):
        vertices, faces = self.convert.read_visual_mesh(self.usd)
        self.assertEqual(faces.shape[1], 3)
        self.assertEqual(len(vertices), 16)
        self.assertEqual(len(faces), 12 + 12)
        self.assertAlmostEqual(vertices[:, 0].min(), 1.0)
        self.assertAlmostEqual(vertices[:, 1].min(), 2.0)
        self.assertAlmostEqual(vertices[:, 2].max(), 4.0)

    def test_author_coacd_colliders_writes_invisible_convex_hulls(self):
        from pxr import Usd, UsdGeom, UsdPhysics

        count = self.convert.author_coacd_colliders(self.usd, self.params)
        self.assertGreaterEqual(count, 2)
        stage = Usd.Stage.Open(str(self.usd))
        hulls = [p for p in stage.GetPrimAtPath("/World/Colliders").GetChildren()]
        self.assertEqual([p.GetName() for p in hulls], [f"hull_{i:02d}" for i in range(count)])
        for prim in hulls:
            self.assertTrue(prim.HasAPI(UsdPhysics.CollisionAPI))
            self.assertEqual(
                UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get(), "convexHull"
            )
            self.assertEqual(
                UsdGeom.Imageable(prim).GetVisibilityAttr().Get(), UsdGeom.Tokens.invisible
            )
        for prim in stage.Traverse():
            if str(prim.GetPath()).startswith("/World/geom"):
                self.assertFalse(prim.HasAPI(UsdPhysics.MeshCollisionAPI))
                self.assertFalse(prim.HasAPI(UsdPhysics.CollisionAPI))

    def test_read_collider_pieces_round_trips_hulls_and_boxes(self):
        import numpy as np
        from tinker_sim_deploy.arena_world import BoxCollider

        count = self.convert.author_coacd_colliders(self.usd, self.params)
        box = BoxCollider(size=(0.2, 0.2, 0.01), center=(0, 0, 0.7), yaw=0.5)
        self.convert.author_model_colliders(self.usd, (box,))
        pieces = self.convert.read_collider_pieces(self.usd)
        self.assertEqual(len(pieces), count + 1)
        box_piece = pieces[-1]
        self.assertEqual(box_piece.faces.shape, (12, 3))
        c, s = math.cos(0.5), math.sin(0.5)
        expected = []
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    x, y, z = sx * 0.1, sy * 0.1, sz * 0.005
                    expected.append((c * x - s * y, s * x + c * y, z + 0.7))
        got = sorted(map(tuple, box_piece.vertices.tolist()))
        for g, e in zip(got, sorted(expected)):
            np.testing.assert_allclose(g, e, atol=1e-9)

    def test_strip_removes_coacd_hulls(self):
        from pxr import Usd

        self.convert.author_coacd_colliders(self.usd, self.params)
        self.convert.strip_model_colliders(self.usd)
        stage = Usd.Stage.Open(str(self.usd))
        self.assertFalse(stage.GetPrimAtPath("/World/Colliders"))


if __name__ == "__main__":
    unittest.main()
