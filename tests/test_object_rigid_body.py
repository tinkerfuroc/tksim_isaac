"""Spawnable object assets must carry their own rigid-body physics.

The simulation spawns scenario objects by USD reference only
(``simulation_interfaces`` SpawnEntity adds no physics APIs), so an
``object.usd`` without ``RigidBodyAPI`` on its default prim composes in as
static scenery: the backend's rigid-body view for it never resolves, its
ground-truth pose is silently absent from every physics-truth frame, and a
grasp can never move it.  Observed live 2026-08-31 on the gpsr-rcw2026-bench
scenario as the ``/World/Scenario/soup`` physx-tensors pattern-miss loop
(~17k error lines per session).  ``author_object_rigid_body`` is the importer
step that closes this; these tests pin its contract.
"""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

try:
    from pxr import Usd, UsdGeom, UsdPhysics
except ImportError:  # pragma: no cover
    Usd = None

from tinker_sim_deploy.arena_convert import author_object_rigid_body  # noqa: E402


@unittest.skipIf(Usd is None, "pxr is unavailable")
class AuthorObjectRigidBody(unittest.TestCase):
    def _object_stage(self, *, with_collider: bool = True):
        stage = Usd.Stage.CreateInMemory()
        root = UsdGeom.Xform.Define(stage, "/World")
        stage.SetDefaultPrim(root.GetPrim())
        if with_collider:
            mesh = UsdGeom.Cube.Define(stage, "/World/collision/mesh_")
            UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        return stage

    def test_authors_rigid_body_on_default_prim(self) -> None:
        stage = self._object_stage()
        author_object_rigid_body(stage)
        self.assertTrue(stage.GetDefaultPrim().HasAPI(UsdPhysics.RigidBodyAPI))

    def test_requires_a_collider(self) -> None:
        # A dynamic body with no collider falls through the world; the
        # importer must refuse to publish one.
        stage = self._object_stage(with_collider=False)
        with self.assertRaises(RuntimeError):
            author_object_rigid_body(stage)
        self.assertFalse(stage.GetDefaultPrim().HasAPI(UsdPhysics.RigidBodyAPI))

    def test_requires_a_default_prim(self) -> None:
        stage = Usd.Stage.CreateInMemory()
        with self.assertRaises(RuntimeError):
            author_object_rigid_body(stage)

    def test_idempotent(self) -> None:
        stage = self._object_stage()
        author_object_rigid_body(stage)
        author_object_rigid_body(stage)
        self.assertTrue(stage.GetDefaultPrim().HasAPI(UsdPhysics.RigidBodyAPI))


@unittest.skipIf(Usd is None, "pxr is unavailable")
class PublishedYcbArtifactContract(unittest.TestCase):
    """Every object.usd the current scenarios reference must be dynamic."""

    def test_current_ycb_objects_have_rigid_bodies(self) -> None:
        import json

        pointer = ROOT / "artifacts/objects/ycb/current.json"
        if not pointer.is_file():
            self.skipTest("content-addressed YCB artifact not present")
        manifest_path = ROOT / json.loads(pointer.read_text())["manifest"]
        manifest = json.loads(manifest_path.read_text())
        checked = 0
        for name in manifest["payload"]:
            if not name.endswith("/object.usd"):
                continue
            stage = Usd.Stage.Open(str(manifest_path.parent / name))
            prim = stage.GetDefaultPrim()
            self.assertTrue(
                prim and prim.HasAPI(UsdPhysics.RigidBodyAPI),
                f"{name}: default prim lacks RigidBodyAPI",
            )
            checked += 1
        self.assertGreater(checked, 0)


if __name__ == "__main__":
    unittest.main()
