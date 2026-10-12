"""Authored spawn state can be registered without a live PhysX view."""
import math
import sys
from types import ModuleType, SimpleNamespace
import pytest
from pxr import Gf, Usd, UsdGeom, UsdPhysics
from tinker_sim_isaac.scoped_reset import PhysicsBaseline


def test_spawn_baseline_uses_world_pose_and_radian_angular_velocity(monkeypatch):
    stage=Usd.Stage.CreateInMemory()
    parent=UsdGeom.Xform.Define(stage,'/World/Spawn')
    parent.AddTranslateOp().Set(Gf.Vec3d(1.,2.,3.))
    parent.AddRotateZOp().Set(90.)
    body=UsdGeom.Xform.Define(stage,'/World/Spawn/Body')
    body.AddTranslateOp().Set(Gf.Vec3d(1.,0.,0.))
    rigid=UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
    rigid.CreateVelocityAttr().Set(Gf.Vec3f(1.,2.,3.))
    rigid.CreateAngularVelocityAttr().Set(Gf.Vec3f(0.,0.,180.))
    omni=ModuleType('omni');usd=ModuleType('omni.usd')
    usd.get_context=lambda:SimpleNamespace(get_stage=lambda:stage)
    omni.usd=usd
    monkeypatch.setitem(sys.modules,'omni',omni)
    monkeypatch.setitem(sys.modules,'omni.usd',usd)
    baseline=object.__new__(PhysicsBaseline);baseline.bodies={}
    state=baseline._authored_spawn_bodies()['/World/Spawn/Body']
    assert state[0][:3]==pytest.approx((1.,3.,3.))
    assert state[0][3:]==pytest.approx((0.,0.,math.sqrt(.5),math.sqrt(.5)))
    assert state[1]==pytest.approx((1.,2.,3.,0.,0.,math.pi))
    baseline.bodies['/World/Spawn/Body']=state
    parent.GetOrderedXformOps()[0].Set(Gf.Vec3d(9.,9.,9.))
    assert baseline._authored_spawn_bodies()=={}


def test_spawn_baseline_rotation_ignores_parent_scale(monkeypatch):
    stage = Usd.Stage.CreateInMemory()
    parent = UsdGeom.Xform.Define(stage, "/World/Spawn")
    parent.AddRotateZOp().Set(90.0)
    parent.AddScaleOp().Set(Gf.Vec3f(0.01, 0.01, 0.01))
    body = UsdGeom.Xform.Define(stage, "/World/Spawn/Body")
    body.AddTranslateOp().Set(Gf.Vec3d(100.0, 0.0, 0.0))
    UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
    omni = ModuleType("omni")
    usd = ModuleType("omni.usd")
    usd.get_context = lambda: SimpleNamespace(get_stage=lambda: stage)
    omni.usd = usd
    monkeypatch.setitem(sys.modules, "omni", omni)
    monkeypatch.setitem(sys.modules, "omni.usd", usd)
    baseline = object.__new__(PhysicsBaseline)
    baseline.bodies = {}
    state = baseline._authored_spawn_bodies()["/World/Spawn/Body"]
    assert state[0][:3] == pytest.approx((0.0, 1.0, 0.0), abs=1e-6)
    assert state[0][3:] == pytest.approx((0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)), abs=1e-6)
