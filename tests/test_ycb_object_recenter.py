"""Unit + artifact regression tests for YCB object origin recentring (Task 32).

Confirmed live (Task 32 forensics): the raw upstream YCB meshes carry an
uncentered origin all the way through the import pipeline -- for
``ycb_010_tomato_soup_can`` the collision mesh's own footprint does not even
contain the tracked rigid-body origin (~8.4cm outside the can along Y), so
PhysX rigid-body truth, ``TINKER_SIM_TRACK_OBJECTS``, and ``/spawn_entity``
placement are all that far off from where the physical object actually is.
``arena_convert.recenter_object_origin`` fixes this by authoring a corrective
``xformOp:translate`` on the composed object's ``geom``/``collision`` wrapper
Xforms so the collision bbox's XY centroid lands at (0, 0), while keeping the
existing base-anchored Z convention (bbox min Z = 0) intact.

These importorskip pxr (present in the sim venv, absent in a plain
system-python run) -- same rationale as ``test_ycb_physics_repair.py``.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

pytest.importorskip("pxr")

from pxr import Gf, Usd, UsdGeom, UsdPhysics  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from tinker_sim_deploy.arena_convert import recenter_object_origin  # noqa: E402

# 5mm tolerance for the artifact regression test -- generous relative to the
# 84mm/18mm real offsets this task fixes, tight enough to catch a regression.
_ARTIFACT_TOLERANCE_M = 0.005


def _define_box_mesh(
    stage: Usd.Stage,
    path: str,
    xmin: float,
    xmax: float,
    ymin: float,
    ymax: float,
    zmin: float,
    zmax: float,
) -> UsdGeom.Mesh:
    """A minimal axis-aligned box mesh, local-space extents as given."""
    mesh = UsdGeom.Mesh.Define(stage, path)
    corners = [
        (xmin, ymin, zmin), (xmax, ymin, zmin), (xmax, ymax, zmin), (xmin, ymax, zmin),
        (xmin, ymin, zmax), (xmax, ymin, zmax), (xmax, ymax, zmax), (xmin, ymax, zmax),
    ]
    mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in corners])
    faces = [
        [0, 1, 2, 3], [4, 5, 6, 7], [0, 1, 5, 4],
        [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7],
    ]
    mesh.CreateFaceVertexCountsAttr([4] * len(faces))
    mesh.CreateFaceVertexIndicesAttr([index for face in faces for index in face])
    return mesh


def _synthetic_object_stage() -> Usd.Stage:
    """Mirrors ``_compose_object``'s structure: /World default prim,
    ``geom``/``collision`` sibling Xforms (each carrying a unit-scale op,
    same as every real composed object), the visual mesh visible, the
    collision mesh invisible with ``PhysicsCollisionAPI``.

    The box's own local geometry already satisfies the base-anchor Z
    convention (z spans [0, 0.1] locally); a translate of (0.05, 0.08, 0.0)
    on each wrapper Xform models the real defect this task fixes: an
    uncentered mesh whose XY footprint straddles well clear of its own
    rigid-body origin.
    """
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(root.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())

    offset = Gf.Vec3d(0.05, 0.08, 0.0)

    geom = UsdGeom.Xform.Define(stage, "/World/geom")
    UsdGeom.Xformable(geom.GetPrim()).AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 1.0))
    visual_mesh = _define_box_mesh(stage, "/World/geom/mesh", -0.05, 0.05, -0.05, 0.05, 0.0, 0.1)
    UsdGeom.Xformable(visual_mesh.GetPrim()).AddTranslateOp().Set(offset)

    collision = UsdGeom.Xform.Define(stage, "/World/collision")
    UsdGeom.Xformable(collision.GetPrim()).AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 1.0))
    collision_mesh = _define_box_mesh(stage, "/World/collision/mesh", -0.05, 0.05, -0.05, 0.05, 0.0, 0.1)
    UsdGeom.Xformable(collision_mesh.GetPrim()).AddTranslateOp().Set(offset)
    UsdPhysics.CollisionAPI.Apply(collision_mesh.GetPrim())
    UsdGeom.Imageable(collision.GetPrim()).MakeInvisible()

    return stage


def _collision_centroid_and_min_z(stage: Usd.Stage) -> tuple[float, float, float]:
    default_prim = stage.GetDefaultPrim()
    collision_root = stage.GetPrimAtPath(default_prim.GetPath().AppendChild("collision"))
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"], False, True)
    aligned = cache.ComputeWorldBound(collision_root).ComputeAlignedRange()
    cmin, cmax = aligned.GetMin(), aligned.GetMax()
    return ((cmin[0] + cmax[0]) / 2.0, (cmin[1] + cmax[1]) / 2.0, cmin[2])


def test_recenter_object_origin_moves_collision_centroid_to_origin():
    stage = _synthetic_object_stage()

    # Sanity: before the fix, the collision footprint really is off-origin.
    cx, cy, min_z = _collision_centroid_and_min_z(stage)
    assert abs(cx - 0.05) < 1e-9
    assert abs(cy - 0.08) < 1e-9
    assert abs(min_z - 0.0) < 1e-9  # already base-anchored, must stay that way

    offset = recenter_object_origin(stage)
    assert abs(offset[0] - (-0.05)) < 1e-9
    assert abs(offset[1] - (-0.08)) < 1e-9
    assert abs(offset[2] - 0.0) < 1e-9

    cx, cy, min_z = _collision_centroid_and_min_z(stage)
    assert abs(cx) < 1e-6
    assert abs(cy) < 1e-6
    assert abs(min_z - 0.0) < 1e-6


def test_recenter_object_origin_keeps_visual_and_collision_coincident():
    stage = _synthetic_object_stage()
    offset = recenter_object_origin(stage)

    default_prim = stage.GetDefaultPrim()
    for child_name in ("geom", "collision"):
        child = stage.GetPrimAtPath(default_prim.GetPath().AppendChild(child_name))
        xformable = UsdGeom.Xformable(child)
        translate_ops = [
            op for op in xformable.GetOrderedXformOps()
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate
        ]
        assert len(translate_ops) == 1, f"{child_name}: expected exactly one translate op"
        value = translate_ops[0].Get()
        assert tuple(value) == pytest.approx(tuple(offset), abs=1e-9)

    # And the two wrapper meshes stay coincident in world space.
    geom_root = stage.GetPrimAtPath(default_prim.GetPath().AppendChild("geom"))
    collision_root = stage.GetPrimAtPath(default_prim.GetPath().AppendChild("collision"))
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"], False, True)
    geom_range = cache.ComputeWorldBound(geom_root).ComputeAlignedRange()
    collision_range = cache.ComputeWorldBound(collision_root).ComputeAlignedRange()
    assert tuple(geom_range.GetMin()) == pytest.approx(tuple(collision_range.GetMin()), abs=1e-9)
    assert tuple(geom_range.GetMax()) == pytest.approx(tuple(collision_range.GetMax()), abs=1e-9)


def test_recenter_object_origin_is_idempotent():
    """A second run against an already-recentred object must not move it
    further (a standalone repair re-run must not stack translates)."""
    stage = _synthetic_object_stage()
    recenter_object_origin(stage)
    second_offset = recenter_object_origin(stage)
    assert abs(second_offset[0]) < 1e-9
    assert abs(second_offset[1]) < 1e-9
    assert abs(second_offset[2]) < 1e-9

    default_prim = stage.GetDefaultPrim()
    collision_root = stage.GetPrimAtPath(default_prim.GetPath().AppendChild("collision"))
    translate_ops = [
        op for op in UsdGeom.Xformable(collision_root).GetOrderedXformOps()
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate
    ]
    assert len(translate_ops) == 1  # not two stacked translate ops


def test_recenter_object_origin_requires_collision_child():
    stage = Usd.Stage.CreateInMemory()
    root = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(root.GetPrim())
    with pytest.raises(RuntimeError):
        recenter_object_origin(stage)


# --------------------------------------------------------------------------- #
# artifact regression: every published YCB object.usd this checkout's
# scenarios actually reference must be recentred on its own rigid-body
# origin. Skips cleanly (rather than failing) when this checkout carries no
# local ``artifacts/`` store -- the binaries are gitignored / published
# out-of-band (see the Task #17/#20/#32 developer-log entries).
# --------------------------------------------------------------------------- #
def _generated_ycb_object_usds(root: Path) -> list[Path]:
    manifest_path = root / "artifacts" / "asset-manifest.json"
    if not manifest_path.is_file():
        return []
    data = json.loads(manifest_path.read_text(encoding="utf-8"))
    return [
        root / entry["path"]
        for entry in data.get("generated_object_usds", [])
        if "/objects/ycb/" in str(entry.get("path", ""))
    ]


def test_every_published_ycb_object_is_recentred_on_its_origin():
    usd_paths = _generated_ycb_object_usds(ROOT)
    if not usd_paths:
        pytest.skip("no artifacts/asset-manifest.json ycb entries in this checkout")
    for usd_path in usd_paths:
        if not usd_path.is_file():
            pytest.skip(f"{usd_path} missing from this checkout's artifact store")

    violations = []
    for usd_path in usd_paths:
        stage = Usd.Stage.Open(str(usd_path))
        cx, cy, min_z = _collision_centroid_and_min_z(stage)
        if abs(cx) > _ARTIFACT_TOLERANCE_M or abs(cy) > _ARTIFACT_TOLERANCE_M or abs(min_z) > _ARTIFACT_TOLERANCE_M:
            violations.append((usd_path.parent.name, round(cx, 4), round(cy, 4), round(min_z, 4)))
    assert not violations, (
        f"YCB objects not recentred on their rigid-body origin (centroid_x, centroid_y, min_z): {violations}"
    )
