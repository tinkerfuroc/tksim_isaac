"""Unit tests for the YCB physics-repair authoring (mass + friction material).

These exercise pxr USD authoring directly, so they importorskip pxr (present in
the sim venv, absent in a plain system-python run). They guard the friction +
mass that spawned YCB objects were missing -- without which a closed grip cannot
hold them under fabric-off (the collider is effectively frictionless).
"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

pytest.importorskip("pxr")

from pxr import Usd, UsdGeom, UsdPhysics  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from tinker_sim_deploy.arena_convert import (  # noqa: E402
    author_object_friction_material,
    author_object_mass,
    author_object_rigid_body,
)


def _object_stage():
    """A minimal composed-object stage: default prim + one collider mesh."""
    stage = Usd.Stage.CreateInMemory()
    root = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(root.GetPrim())
    mesh = UsdGeom.Mesh.Define(stage, "/World/collision/mesh")
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    return stage


def test_author_object_mass_sets_physics_mass():
    stage = _object_stage()
    author_object_rigid_body(stage)
    author_object_mass(stage, 0.349)
    dp = stage.GetDefaultPrim()
    assert dp.HasAPI(UsdPhysics.MassAPI)
    assert abs(UsdPhysics.MassAPI(dp).GetMassAttr().Get() - 0.349) < 1e-6


def test_author_object_mass_rejects_nonpositive():
    stage = _object_stage()
    author_object_rigid_body(stage)
    for bad in (0.0, -1.0):
        with pytest.raises(RuntimeError):
            author_object_mass(stage, bad)


def test_author_friction_material_binds_physics_material_on_collider():
    stage = _object_stage()
    author_object_rigid_body(stage)
    bound = author_object_friction_material(stage, 0.8, 0.7)
    assert bound == 1

    materials = [
        p
        for p in stage.Traverse()
        if p.GetTypeName() == "Material"
        and UsdPhysics.MaterialAPI(p).GetStaticFrictionAttr().Get() is not None
    ]
    assert len(materials) == 1
    mat = UsdPhysics.MaterialAPI(materials[0])
    assert abs(mat.GetStaticFrictionAttr().Get() - 0.8) < 1e-6
    assert abs(mat.GetDynamicFrictionAttr().Get() - 0.7) < 1e-6
    assert abs(mat.GetRestitutionAttr().Get() - 0.0) < 1e-6

    collider = stage.GetPrimAtPath("/World/collision/mesh")
    rel = collider.GetRelationship("material:binding:physics")
    assert rel and rel.GetTargets() == [materials[0].GetPath()]


def test_author_friction_material_requires_a_collider():
    stage = Usd.Stage.CreateInMemory()
    root = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(root.GetPrim())
    with pytest.raises(RuntimeError):
        author_object_friction_material(stage, 0.8, 0.7)
