"""Authoring and reading CoACD hull colliders in converted furniture USDs."""
from __future__ import annotations

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


def _box_pieces(*boxes):
    """Box-shaped ``ConvexPiece``s (8 corners, 12 triangles) for (lo, hi) corner pairs."""
    import numpy as np
    from tinker_sim_deploy.arena_coacd import ConvexPiece

    faces = np.array(
        [(0, 2, 3), (0, 3, 1), (4, 5, 7), (4, 7, 6), (0, 1, 5), (0, 5, 4),
         (2, 6, 7), (2, 7, 3), (0, 4, 6), (0, 6, 2), (1, 3, 7), (1, 7, 5)],
        dtype=np.int64,
    )
    pieces = []
    for lo, hi in boxes:
        verts = np.array(
            [(x, y, z) for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])],
            dtype=np.float64,
        )
        pieces.append(ConvexPiece(vertices=verts, faces=faces.copy()))
    return pieces


class CoacdUsdTests(unittest.TestCase):
    """pxr only: hull pieces come from ``arena_coacd.decompose_all`` in the
    importer, so authoring/reading is tested here with fixed box-shaped pieces
    (CoACD itself is covered by tests/test_arena_coacd.py)."""

    def setUp(self):
        pytest.importorskip("pxr.Usd")
        pytest.importorskip("numpy")
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.usd = Path(self._tmp.name) / "l.usd"
        _write_l_usd(self.usd)
        from tinker_sim_deploy import arena_convert

        self.convert = arena_convert
        # The L of _write_l_usd in the model frame (geom is translated by (1, 2, 3)).
        self.pieces = _box_pieces(
            ((1.0, 2.0, 3.0), (2.0, 2.4, 3.2)), ((1.0, 2.0, 3.2), (1.2, 2.4, 4.0))
        )

    def test_read_visual_mesh_triangulates_and_merges(self):
        vertices, faces = self.convert.read_visual_mesh(self.usd)
        self.assertEqual(faces.shape[1], 3)
        self.assertEqual(len(vertices), 16)
        self.assertEqual(len(faces), 12 + 12)
        self.assertAlmostEqual(vertices[:, 0].min(), 1.0)
        self.assertAlmostEqual(vertices[:, 1].min(), 2.0)
        self.assertAlmostEqual(vertices[:, 2].max(), 4.0)

    def test_author_collider_pieces_writes_invisible_convex_hulls(self):
        from pxr import Usd, UsdGeom, UsdPhysics

        self.convert.author_collider_pieces(self.usd, self.pieces)
        stage = Usd.Stage.Open(str(self.usd))
        hulls = [p for p in stage.GetPrimAtPath("/World/Colliders").GetChildren()]
        self.assertEqual([p.GetName() for p in hulls], ["hull_00", "hull_01"])
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

        self.convert.author_collider_pieces(self.usd, self.pieces)
        box = BoxCollider(size=(0.2, 0.2, 0.01), center=(0, 0, 0.7), yaw=0.5)
        self.convert.author_model_colliders(self.usd, (box,))
        pieces = self.convert.read_collider_pieces(self.usd)
        self.assertEqual(len(pieces), len(self.pieces) + 1)
        for got, want in zip(pieces, self.pieces):
            # Stored as float32 points.
            np.testing.assert_allclose(got.vertices, want.vertices, atol=1e-6)
            np.testing.assert_array_equal(got.faces, want.faces)
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

    def test_reauthoring_fewer_pieces_leaves_no_stale_colliders(self):
        from pxr import Usd
        from tinker_sim_deploy.arena_world import BoxCollider

        three = self.pieces + _box_pieces(((1.5, 2.0, 3.2), (1.7, 2.4, 3.4)))
        self.convert.author_collider_pieces(self.usd, three)
        plate = BoxCollider(size=(0.2, 0.2, 0.01), center=(0, 0, 0.7), yaw=0.0)
        self.convert.author_model_colliders(self.usd, (plate, plate))
        self.convert.author_collider_pieces(self.usd, self.pieces[:1])
        stage = Usd.Stage.Open(str(self.usd))
        names = [p.GetName() for p in stage.GetPrimAtPath("/World/Colliders").GetChildren()]
        self.assertEqual(names, ["hull_00"])
        self.assertEqual(len(self.convert.read_collider_pieces(self.usd)), 1)

    def test_same_pieces_give_byte_identical_usd(self):
        other = self.usd.with_name("other.usd")
        _write_l_usd(other)
        self.convert.author_collider_pieces(self.usd, self.pieces)
        self.convert.author_collider_pieces(other, self.pieces)
        self.assertEqual(self.usd.read_bytes(), other.read_bytes())

    def test_strip_removes_coacd_hulls(self):
        from pxr import Usd

        self.convert.author_collider_pieces(self.usd, self.pieces)
        self.convert.strip_model_colliders(self.usd)
        stage = Usd.Stage.Open(str(self.usd))
        self.assertFalse(stage.GetPrimAtPath("/World/Colliders"))


if __name__ == "__main__":
    unittest.main()
