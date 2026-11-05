"""Support-surface box override: legacy visual-mesh colliders only.

``support_surface_box_targets`` picks the prims the backend sets to
``boundingCube``; CoACD hulls under a furniture's ``Colliders`` scope must be
left alone. Kit-free: runs on an in-memory pxr stage.
"""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))

TABLE = "/World/Arena/Furniture/rcw26_kitchen_table"


def _stage():
    from pxr import Usd, UsdGeom, UsdPhysics

    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, TABLE)
    for path in (f"{TABLE}/visual", f"{TABLE}/Colliders/hull_00"):
        UsdGeom.Mesh.Define(stage, path)
        UsdPhysics.MeshCollisionAPI.Apply(stage.GetPrimAtPath(path))
    return stage


class SupportSurfaceBoxTargetsTest(unittest.TestCase):
    def test_legacy_visual_mesh_colliders_still_boxed(self):
        pytest.importorskip("pxr")
        from tinker_sim_isaac.arena_surface_colliders import (
            support_surface_box_targets,
        )

        paths = [
            str(p.GetPath()) for p in support_surface_box_targets(_stage(), [TABLE])
        ]
        self.assertIn(f"{TABLE}/visual", paths)

    def test_coacd_hulls_not_boxed(self):
        pytest.importorskip("pxr")
        from tinker_sim_isaac.arena_surface_colliders import (
            support_surface_box_targets,
        )

        paths = [
            str(p.GetPath()) for p in support_surface_box_targets(_stage(), [TABLE])
        ]
        self.assertNotIn(f"{TABLE}/Colliders/hull_00", paths)
        self.assertEqual(paths, [f"{TABLE}/visual"])

    def test_missing_furniture_path_skipped(self):
        pytest.importorskip("pxr")
        from tinker_sim_isaac.arena_surface_colliders import (
            support_surface_box_targets,
        )

        targets = support_surface_box_targets(
            _stage(), ["/World/Arena/Furniture/rcw26_nope", TABLE]
        )
        self.assertEqual(len(targets), 1)


if __name__ == "__main__":
    unittest.main()
