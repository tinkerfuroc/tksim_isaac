"""Kit/pxr conversion adapter for the RoboCup arena importer.

Every function here that touches Kit/pxr is live-only: those imports are
lazy, inside each function body, so this module can be *imported* under
plain system Python (e.g. by ``tools/arena_import.py``'s ``--help``)
without a running Isaac Sim process, while actually *calling* one of them
requires an active ``SimulationApp`` with ``omni.kit.asset_converter``
enabled -- exercised by the live import only (see the Task 9 report for the
run log). A handful of pure-Python helpers with no pxr dependency
(``_texture_relocation_target``, ``_relocatable_asset_source``) are
extracted specifically so they can be unit-tested under system Python; see
``tests/test_arena_convert.py``.

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
    _relocate_local_assets(usd_path, raw_path.parent, usd_path.stem)


def _rebase_converted_mesh(
    raw_path: Path,
    usd_path: Path,
    scale: tuple[float, float, float],
    pose: tuple[float, float, float, float, float, float],
) -> None:
    """Re-anchor the raw Kit conversion onto this codebase's metre/Z-up
    convention, structured so both ``author_model_colliders`` (Finding 2)
    and further referencing (``compose_arena`` referencing this whole file)
    stay safe:

        /World            plain Xform, the default prim, no xformOps of
                           its own -- referencing *this* file elsewhere and
                           then authoring new world-placement ops on the
                           referencing prim must not collide with anything
                           already authored here.
        /World/geom       the mesh-correction ops (translate, then rotate,
                           then scale -- see the module docstring for why
                           that op order is correct).
        /World/geom/mesh  references the raw conversion. A further child,
                           not /World/geom itself: the raw stage's own
                           default prim already carries an authored
                           (identity) [translate, orient, scale] Kit stamps
                           on every glTF conversion, and referencing it
                           onto the *same* prim that already has our own
                           translate/rotate/scale ops raises
                           "xformOp:translate already exists in
                           xformOpOrder" (confirmed live).

    ``author_model_colliders`` later adds a sibling ``/World/Colliders``
    subtree -- also under the default prim (so a reference to this whole
    file composes it in too), but outside ``/World/geom`` (so colliders
    never inherit the mesh-correction xform).
    """
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

    geom = UsdGeom.Xform.Define(wrapper, "/World/geom")
    x, y, z, roll, pitch, yaw = pose
    # Fold the raw stage's own metersPerUnit into the authored scale so the
    # flattened result is expressed directly in metres, matching the
    # wrapper's own metersPerUnit=1.0 declared above.
    effective_scale = tuple(component * raw_meters_per_unit for component in scale)
    xformable = UsdGeom.Xformable(geom.GetPrim())
    xformable.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
    xformable.AddRotateXYZOp().Set(
        Gf.Vec3f(math.degrees(roll), math.degrees(pitch), math.degrees(yaw))
    )
    xformable.AddScaleOp().Set(Gf.Vec3f(*effective_scale))

    mesh_ref = wrapper.DefinePrim("/World/geom/mesh")
    mesh_ref.GetReferences().AddReference(str(raw_path))

    # Flatten: the published artifact only ever carries usd_path's bytes,
    # never the *.raw.usd sibling, so the reference above must not survive
    # into the final file.
    flattened = wrapper.Flatten()
    flattened.Export(str(usd_path))


def _texture_relocation_target(model_id: str, filename: str) -> str:
    """Deterministic, artifact-relative destination for a texture/material
    file Kit extracted alongside the raw GLB conversion -- relative to
    ``usd_path`` itself, since both end up siblings under the published
    ``furniture/`` directory (``furniture/<model_id>.usd`` next to
    ``furniture/textures/<model_id>/<filename>``). Namespaced per model_id
    so two different models' textures can never collide in the shared
    payload even if Kit ever reused a filename across sources.
    """
    return f"./textures/{model_id}/{filename}"


def _relocatable_asset_source(raw_asset_path: str | None, search_root: Path) -> Path | None:
    """Return the resolved source ``Path`` if ``raw_asset_path`` is a local
    file Kit extracted under ``search_root`` (the scratch conversion
    directory for this model), or ``None`` if it is not a relocation
    candidate -- e.g. a bare, non-absolute reference into Kit's built-in
    MDL shader library such as ``"gltf/pbr.mdl"``, which is not a real
    filesystem path at authoring time and must be left untouched (it
    resolves via Kit's own MDL search path wherever the USD is later
    opened, not something this importer controls or should bundle).
    """
    if not raw_asset_path:
        return None
    source = Path(raw_asset_path)
    if not source.is_absolute():
        return None
    try:
        source.relative_to(search_root)
    except ValueError:
        return None
    if not source.is_file():
        return None
    return source


def _resave_fresh(stage, usd_path: Path) -> None:
    """Replace ``usd_path`` with a freshly re-serialized export of
    ``stage``'s root layer, rather than an in-place ``Stage.Save()``.

    Confirmed live (see the Task 9 fix report, Finding 1): ``Save()``
    performs an incremental update of the existing crate file and does
    *not* purge its string pool -- a previously-authored-then-overwritten
    value (e.g. a texture path ``_relocate_local_assets`` rewrites from an
    absolute scratch path to a relative one) can remain physically present
    in the file's raw bytes even though the live scene graph reads back
    correctly. That both defeats grepping the published payload for a
    leaked scratch path and breaks publish-determinism (the stale bytes
    differ by the random per-run tempdir suffix even though nothing
    observable changed). A fresh ``Export()`` rebuilds the string pool
    from only the current field values, with nothing left over.
    """
    fresh_path = usd_path.with_name(usd_path.stem + ".fresh.usd")
    stage.GetRootLayer().Export(str(fresh_path))
    fresh_path.replace(usd_path)


def _relocate_local_assets(usd_path: Path, search_root: Path, model_id: str) -> None:
    """Copy any texture/material files Kit extracted as local files under
    ``search_root`` into the payload directory next to ``usd_path``, and
    rewrite the absolute scratch paths ``Stage.Flatten()`` bakes into
    asset-valued attributes (composition arcs get resolved away by
    ``Flatten()``, but asset-path-typed *values* like a shader's
    ``inputs:texture`` do not -- they get anchored absolute instead) to a
    path relative to ``usd_path``.

    Without this, every published furniture USD embeds the random per-run
    scratch tempdir path: textures resolve nowhere once that directory is
    deleted (confirmed live -- no PNG bytes embedded, no texture files in
    the payload, textures pointing into an already-deleted directory), and
    re-running the same import against the same pin/config produces
    different bytes (a different random tempdir suffix baked into the
    texture path) for what should be an identical, idempotent artifact.
    """
    from pxr import Sdf, Usd

    stage = Usd.Stage.Open(str(usd_path))
    dest_dir = usd_path.parent / "textures" / model_id
    relocated: dict[str, str] = {}
    changed = False
    for prim in stage.Traverse():
        for attr in prim.GetAttributes():
            if attr.GetTypeName() != Sdf.ValueTypeNames.Asset:
                continue
            value = attr.Get()
            if value is None:
                continue
            source = _relocatable_asset_source(value.path, search_root)
            if source is None:
                continue
            key = str(source)
            if key not in relocated:
                dest_dir.mkdir(parents=True, exist_ok=True)
                (dest_dir / source.name).write_bytes(source.read_bytes())
                relocated[key] = _texture_relocation_target(model_id, source.name)
            attr.Set(Sdf.AssetPath(relocated[key]))
            changed = True
    if changed:
        _resave_fresh(stage, usd_path)


def author_model_colliders(
    usd_path: Path, colliders: tuple[BoxCollider | MeshCollider, ...]
) -> None:
    """Author collider prims for one converted furniture USD, in place.

    Box colliders become invisible ``UsdGeom.Cube`` prims under
    ``<default prim>/Colliders`` -- a *sibling* of the mesh-correction
    ``.../geom`` subtree (colliders never inherit that xform), but still
    *under* the default prim itself, so a reference to this whole file
    (e.g. ``compose_arena`` referencing ``./furniture/<id>.usd``) composes
    in both the visual mesh and its colliders together.

    An earlier version put colliders at a root-level ``/Colliders``, a
    sibling of the default prim entirely: USD reference semantics only
    compose the *referenced prim's own subtree*, so anything outside the
    default prim is silently dropped by every reference. That left ~14 of
    20 furniture models (every strict-box model plus the cylinder/mesh
    ``kitchen_table``/``door``/``sink``) with no physics at all once
    referenced into the composed arena -- an object placed on
    ``kitchen_table#top`` would fall straight through. Found live via a
    structural pxr check on the composed ``arena.usd``, fixed here.

    Mesh colliders get a convex-hull ``MeshCollisionAPI`` applied directly
    to every imported ``UsdGeom.Mesh`` prim (works regardless of nesting
    depth -- ``Traverse()`` walks the whole stage).
    """
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    stage = Usd.Stage.Open(str(usd_path))
    default_prim = stage.GetDefaultPrim()
    if not default_prim:
        raise RuntimeError(f"{usd_path}: no default prim to author colliders under")

    if any(isinstance(collider, MeshCollider) for collider in colliders):
        for prim in stage.Traverse():
            if prim.IsA(UsdGeom.Mesh):
                api = UsdPhysics.MeshCollisionAPI.Apply(prim)
                api.CreateApproximationAttr("convexHull")
                UsdPhysics.CollisionAPI.Apply(prim)

    box_colliders = [collider for collider in colliders if isinstance(collider, BoxCollider)]
    if box_colliders:
        colliders_path = default_prim.GetPath().AppendChild("Colliders")
        UsdGeom.Scope.Define(stage, colliders_path)
        for index, collider in enumerate(box_colliders):
            cube = UsdGeom.Cube.Define(stage, colliders_path.AppendChild(f"box_{index:02d}"))
            cube.CreateSizeAttr(1.0)
            xform = UsdGeom.Xformable(cube.GetPrim())
            xform.AddTranslateOp().Set(Gf.Vec3d(*collider.center))
            xform.AddRotateZOp().Set(math.degrees(collider.yaw))
            xform.AddScaleOp().Set(Gf.Vec3f(*collider.size))
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
            UsdGeom.Imageable(cube.GetPrim()).MakeInvisible()
    _resave_fresh(stage, usd_path)


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
        # The referenced furniture USD's default prim (/World) is a plain,
        # op-free Xform by construction (see arena_convert's own
        # _rebase_converted_mesh), but world-placement ops still go on a
        # *wrapper* prim with the reference on a child ("content", not
        # "geom" -- the referenced file's own default prim already has a
        # "geom" child, and nesting "geom/geom" would be needlessly
        # confusing) rather than the wrapper itself, out of caution: this
        # is exactly the "xformOp:translate already exists" shape of bug
        # that hit _rebase_converted_mesh once already, and the reference
        # brings in the whole subtree either way.
        wrapper = UsdGeom.Xform.Define(stage, prim_path)
        xform = UsdGeom.Xformable(wrapper.GetPrim())
        xform.AddTranslateOp().Set(Gf.Vec3d(*item.position))
        xform.AddRotateZOp().Set(math.degrees(item.yaw))
        content_prim = stage.DefinePrim(f"{prim_path}/content")
        content_prim.GetReferences().AddReference(f"./{furniture_dir_name}/{item.model_id}.usd")

    stage.Save()


def measure_bounds(
    usd_path: Path,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Return the (min, max) world-space AABB of ``usd_path``'s default prim.

    Uses default ``ignoreVisibility=False``, which prunes invisible
    *descendants* from the computed bound regardless of nesting depth --
    confirmed live with a direct pxr check before relying on it -- so the
    invisible ``.../Colliders/box_NN`` cubes ``author_model_colliders``
    adds under the same default prim never contribute here, even though
    (per Finding 2's fix) they now live inside the default prim's subtree
    rather than outside it.
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
