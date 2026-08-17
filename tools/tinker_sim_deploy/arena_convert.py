"""Kit/pxr conversion adapter for the RoboCup arena importer.

Every function here is live-only: Kit/pxr are imported lazily inside each
function body so this module can be *imported* under plain system Python
(e.g. by ``tools/arena_import.py``'s ``--help``) without a running Isaac Sim
process, while actually *calling* any function requires an active
``SimulationApp`` with ``omni.kit.asset_converter`` enabled. There are no
unit tests for this module -- it is exercised by the live import only (see
the Task 9 report for the run log).

xformOp authoring convention used throughout: ops are added in the order
translate, then rotate, then scale. USD composes ``xformOpOrder`` so the
*last*-added op is applied first (innermost) to a point and the *first*-
added op is applied last (outermost); adding [translate, rotate, scale]
therefore yields the point transform ``translate(rotate(scale(p)))`` --
scale the local geometry first, rotate it, then place it in the parent
frame. This was verified directly against this Isaac Sim's bundled USD
before writing the rest of this module (see the Task 9 report).
"""
from __future__ import annotations

import math
from pathlib import Path

from .arena_world import ArenaLayout, BoxCollider, MeshCollider

_IDENTITY_SCALE = (1.0, 1.0, 1.0)
_IDENTITY_POSE = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def convert_glb_to_usd(
    glb_path: Path,
    usd_path: Path,
    *,
    mesh_scale: tuple[float, float, float] = _IDENTITY_SCALE,
    mesh_pose: tuple[float, float, float, float, float, float] = _IDENTITY_POSE,
) -> None:
    """Convert ``glb_path`` to ``usd_path`` via ``omni.kit.asset_converter``,
    then correct it to this codebase's metre/Z-up convention.

    Two real-world deviations from the brief's original sketch, confirmed
    live against Isaac Sim 6.0.1's bundled asset converter before writing
    this (see the Task 9 report for the raw measurements):

    1. Kit's glTF import leaves the stage Y-up with ``metersPerUnit=0.01``
       (its own default, unrelated to the source file) -- everywhere else
       in this codebase (e.g. ``robot.usd``) uses ``metersPerUnit=1.0`` /
       Z-up, so every converted furniture USD is re-based onto that
       convention here, not left to whatever Kit happened to default to.
    2. The upstream RCW26 GLBs are themselves unit-normalized and Y-up
       *on top of* that; Gazebo brings them to real size/orientation via
       the model SDF's ``<visual><mesh><scale>``/``<pose>`` (roll=90deg).
       ``mesh_scale``/``mesh_pose`` reproduce exactly that transform here
       -- combined with the raw stage's own ``metersPerUnit``, the
       corrected bounds line up with the SDF-declared box colliders to
       within a few millimetres (verified live; not just no-op-safe).

    Kit's converter has no per-asset scale/orientation knob, so the
    correction is authored as a *new* wrapping stage (translate, then
    rotate, then scale ops on a fresh default prim) referencing the raw
    conversion as a child, then flattened into ``usd_path`` so the
    published artifact never depends on the raw intermediate file.
    """
    import asyncio

    import omni.kit.asset_converter as asset_converter

    usd_path.parent.mkdir(parents=True, exist_ok=True)
    raw_path = usd_path.with_name(usd_path.stem + ".raw.usd")

    async def _run() -> None:
        instance = asset_converter.get_instance()
        task = instance.create_converter_task(str(glb_path), str(raw_path))
        if not await task.wait_until_finished():
            raise RuntimeError(
                f"asset conversion failed for {glb_path}: {task.get_error_message()}"
            )

    asyncio.get_event_loop().run_until_complete(_run())
    _rebase_converted_mesh(raw_path, usd_path, mesh_scale, mesh_pose)


def _rebase_converted_mesh(
    raw_path: Path,
    usd_path: Path,
    scale: tuple[float, float, float],
    pose: tuple[float, float, float, float, float, float],
) -> None:
    from pxr import Gf, Usd, UsdGeom

    raw_stage = Usd.Stage.Open(str(raw_path))
    if not raw_stage.GetDefaultPrim():
        raise RuntimeError(f"{raw_path}: converted USD has no default prim")
    raw_meters_per_unit = UsdGeom.GetStageMetersPerUnit(raw_stage)
    raw_stage = None  # release the layer before re-opening it via reference

    wrapper = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(wrapper, 1.0)
    UsdGeom.SetStageUpAxis(wrapper, UsdGeom.Tokens.z)
    root = UsdGeom.Xform.Define(wrapper, "/World")
    wrapper.SetDefaultPrim(root.GetPrim())

    x, y, z, roll, pitch, yaw = pose
    # Fold the raw stage's own metersPerUnit into the authored scale so the
    # flattened result is expressed directly in metres, matching the
    # wrapper's own metersPerUnit=1.0 declared above.
    effective_scale = tuple(component * raw_meters_per_unit for component in scale)
    xformable = UsdGeom.Xformable(root.GetPrim())
    xformable.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
    xformable.AddRotateXYZOp().Set(
        Gf.Vec3f(math.degrees(roll), math.degrees(pitch), math.degrees(yaw))
    )
    xformable.AddScaleOp().Set(Gf.Vec3f(*effective_scale))

    geom = wrapper.DefinePrim("/World/geom")
    geom.GetReferences().AddReference(str(raw_path))

    # Flatten: the published artifact only ever carries usd_path's bytes,
    # never the *.raw.usd sibling, so the reference above must not survive
    # into the final file.
    flattened = wrapper.Flatten()
    flattened.Export(str(usd_path))


def author_model_colliders(
    usd_path: Path, colliders: tuple[BoxCollider | MeshCollider, ...]
) -> None:
    """Author collider prims for one converted furniture USD, in place.

    Box colliders become invisible ``UsdGeom.Cube`` prims under a top-level
    ``/Colliders`` scope (a sibling of the converter's default prim, so it
    never inherits ``convert_glb_to_usd``'s mesh-correction xform and never
    contributes to ``measure_bounds``, which measures only the default
    prim's subtree). Mesh colliders get a convex-hull ``MeshCollisionAPI``
    applied directly to every imported ``UsdGeom.Mesh`` prim.
    """
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    stage = Usd.Stage.Open(str(usd_path))
    if any(isinstance(collider, MeshCollider) for collider in colliders):
        for prim in stage.Traverse():
            if prim.IsA(UsdGeom.Mesh):
                api = UsdPhysics.MeshCollisionAPI.Apply(prim)
                api.CreateApproximationAttr("convexHull")
                UsdPhysics.CollisionAPI.Apply(prim)

    box_colliders = [collider for collider in colliders if isinstance(collider, BoxCollider)]
    if box_colliders:
        UsdGeom.Scope.Define(stage, "/Colliders")
        for index, collider in enumerate(box_colliders):
            cube = UsdGeom.Cube.Define(stage, f"/Colliders/box_{index:02d}")
            cube.CreateSizeAttr(1.0)
            xform = UsdGeom.Xformable(cube.GetPrim())
            xform.AddTranslateOp().Set(Gf.Vec3d(*collider.center))
            xform.AddRotateZOp().Set(math.degrees(collider.yaw))
            xform.AddScaleOp().Set(Gf.Vec3f(*collider.size))
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
            UsdGeom.Imageable(cube.GetPrim()).MakeInvisible()
    stage.Save()


def _bind_gray_material(stage, prim, materials_scope: str) -> None:
    from pxr import Sdf, UsdShade

    material_path = f"{materials_scope}/Gray"
    material = UsdShade.Material.Get(stage, material_path)
    if not material:
        material = UsdShade.Material.Define(stage, material_path)
        shader = UsdShade.Shader.Define(stage, f"{material_path}/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set((0.6, 0.6, 0.6))
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.8)
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)


def compose_arena(
    arena_usd: Path, layout: ArenaLayout, furniture_dir_name: str = "furniture"
) -> None:
    """Compose the arena stage: authored wall cuboids + referenced furniture."""
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    arena_usd.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(arena_usd))
    # Usd.Stage.CreateNew() defaults to metersPerUnit=0.01 / upAxis=Y when
    # unauthored (confirmed live); every other stage in this codebase
    # (e.g. robot.usd) is metersPerUnit=1.0 / Z-up, so state it explicitly.
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    root = UsdGeom.Xform.Define(stage, "/World/Arena")
    stage.SetDefaultPrim(root.GetPrim())

    walls_scope = UsdGeom.Scope.Define(stage, "/World/Arena/Walls")
    for index, wall in enumerate(layout.walls):
        cube = UsdGeom.Cube.Define(stage, f"{walls_scope.GetPath()}/wall_{index:04d}")
        cube.CreateSizeAttr(1.0)
        xform = UsdGeom.Xformable(cube.GetPrim())
        xform.AddTranslateOp().Set(Gf.Vec3d(*wall.center))
        xform.AddRotateZOp().Set(math.degrees(wall.yaw))
        xform.AddScaleOp().Set(Gf.Vec3f(*wall.size))
        _bind_gray_material(stage, cube.GetPrim(), "/World/Arena/Materials")
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        rigid_body = UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
        rigid_body.CreateKinematicEnabledAttr(True)

    furniture_scope = UsdGeom.Scope.Define(stage, "/World/Arena/Furniture")
    instance_counts: dict[str, int] = {}
    for item in layout.furniture:
        instance_counts[item.model_id] = instance_counts.get(item.model_id, 0) + 1
        count = instance_counts[item.model_id]
        # Multiple placements of the same model_id (e.g. two side tables)
        # each need their own prim path; the first keeps the bare model_id
        # (matching the common, non-duplicated case) and later instances
        # get a disambiguating suffix.
        suffix = "" if count == 1 else f"_{count:02d}"
        prim_path = f"{furniture_scope.GetPath()}/{item.model_id}{suffix}"
        # The referenced furniture USD's own default prim may already carry
        # xformOps (convert_glb_to_usd's mesh_scale/mesh_pose correction);
        # referencing composes those onto whatever prim holds the
        # reference, so world-placement ops must live on a *wrapper* prim
        # with the reference on a child -- authoring translate/rotateZ
        # directly on the referencing prim itself collides with the
        # inherited xformOpOrder ("xformOp:translate already exists").
        wrapper = UsdGeom.Xform.Define(stage, prim_path)
        xform = UsdGeom.Xformable(wrapper.GetPrim())
        xform.AddTranslateOp().Set(Gf.Vec3d(*item.position))
        xform.AddRotateZOp().Set(math.degrees(item.yaw))
        geom_prim = stage.DefinePrim(f"{prim_path}/geom")
        geom_prim.GetReferences().AddReference(f"./{furniture_dir_name}/{item.model_id}.usd")

    stage.Save()


def measure_bounds(
    usd_path: Path,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Return the (min, max) world-space AABB of ``usd_path``'s default prim.

    Uses default ``ignoreVisibility=False`` so invisible collider prims
    authored by ``author_model_colliders`` never contribute (they also live
    outside the default prim's subtree, so this is a second, independent
    guard against the map slice / bounds check ever seeing collider
    geometry instead of the visual mesh).
    """
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.Open(str(usd_path))
    default_prim = stage.GetDefaultPrim()
    if not default_prim:
        raise RuntimeError(f"{usd_path}: no default prim to measure")
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
    bound = cache.ComputeWorldBound(default_prim)
    aligned = bound.ComputeAlignedRange()
    min_point = aligned.GetMin()
    max_point = aligned.GetMax()
    return (
        (float(min_point[0]), float(min_point[1]), float(min_point[2])),
        (float(max_point[0]), float(max_point[1]), float(max_point[2])),
    )
