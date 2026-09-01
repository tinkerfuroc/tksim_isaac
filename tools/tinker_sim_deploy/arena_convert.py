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

from .arena_artifact import AssetArtifactError
from .arena_world import ArenaLayout, BoxCollider, MeshCollider

_IDENTITY_SCALE = (1.0, 1.0, 1.0)
_IDENTITY_POSE = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

# --- arena appearance -------------------------------------------------------
#
# The GLB-converted furniture ships per-model textures but renders bland /
# white in the arena (the GLB's own material bindings do not resolve at
# render time). Rather than chase texture resolution, compose_arena binds a
# deterministic solid PBR color over each furniture prim. Colors are matte
# (roughness ~0.8) unless a material reads glossier in reality (appliances,
# TV). Values are UsdPreviewSurface diffuseColor, same convention as the
# wall gray below. Every rcw2026-allowlisted model has an explicit entry;
# anything unlisted falls back to a warm neutral so nothing can regress to
# stark-white.

#: Solid wood-oak floor slab authored into arena.usd (see ``floor_slab``).
FLOOR_COLOR = (0.50, 0.35, 0.20)
FLOOR_ROUGHNESS = 0.75

#: Warm neutral for any furniture model_id without an explicit color.
_FURNITURE_FALLBACK: tuple[tuple[float, float, float], float] = ((0.55, 0.50, 0.45), 0.8)

#: model_id -> (diffuse rgb 0..1, roughness). Keep in sync with the rcw2026
#: import allowlist in ``config/arena-import.json``.
FURNITURE_COLORS: dict[str, tuple[tuple[float, float, float], float]] = {
    # Wood -- medium
    "rcw26_kitchen_table": ((0.55, 0.38, 0.22), 0.8),
    "rcw26_side_table": ((0.60, 0.42, 0.25), 0.8),
    "rcw26_laundry_desk": ((0.56, 0.40, 0.24), 0.8),
    "rcw26_stand": ((0.52, 0.36, 0.21), 0.8),
    # Wood -- light
    "rcw26_shelf": ((0.62, 0.45, 0.28), 0.8),
    "rcw26_door": ((0.63, 0.46, 0.28), 0.8),
    # Wood -- dark
    "rcw26_tv_stand": ((0.45, 0.30, 0.18), 0.8),
    "rcw26_bed": ((0.46, 0.31, 0.19), 0.8),
    # Soft furnishings / fabric
    "rcw26_sofa": ((0.35, 0.40, 0.48), 0.85),
    "rcw26_cushion": ((0.70, 0.55, 0.45), 0.85),
    "rcw26_chair": ((0.30, 0.32, 0.35), 0.85),
    # Appliances -- off-white steel, a touch glossier
    "rcw26_refrigerator": ((0.86, 0.87, 0.89), 0.5),
    "rcw26_washing_machine": ((0.88, 0.88, 0.90), 0.5),
    "rcw26_washing_machine_open": ((0.88, 0.88, 0.90), 0.5),
    "rcw26_dishwasher_close": ((0.80, 0.82, 0.84), 0.5),
    "rcw26_dishwasher_open": ((0.80, 0.82, 0.84), 0.5),
    "rcw26_sink": ((0.78, 0.80, 0.82), 0.4),
    # Plastic / wicker
    "rcw26_trashbin": ((0.25, 0.28, 0.32), 0.7),
    "rcw26_laundry_basket": ((0.80, 0.75, 0.60), 0.8),
    # Plants
    "rcw26_plant_mid": ((0.20, 0.45, 0.20), 0.8),
    "rcw26_plant_tall": ((0.18, 0.42, 0.18), 0.8),
    # Electronics
    "rcw26_tv": ((0.05, 0.05, 0.06), 0.3),
}


def furniture_material(model_id: str) -> tuple[tuple[float, float, float], float]:
    """``(diffuse rgb, roughness)`` for a furniture ``model_id``.

    Falls back to a warm neutral for any unlisted id, so a newly
    allowlisted model can never regress to an untextured stark-white.
    """
    return FURNITURE_COLORS.get(model_id, _FURNITURE_FALLBACK)


def floor_slab(
    layout: ArenaLayout,
    *,
    margin: float = 0.10,
    thickness: float = 0.02,
    lift: float = 0.002,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """``(center_xyz, size_xyz)`` for the visual wood floor slab.

    The footprint is the axis-aligned bounding box of every wall box's XY
    extent, expanded by ``margin`` on each side. The slab is ``thickness``
    tall and its top face sits at z=``lift`` -- a hair above the physics
    ground plane at z=0 -- so it wins the depth test against Isaac's default
    ground grid without disturbing physics (the slab carries no collider).

    Wall yaw is 0 in rcw2026; a rotated wall contributes its axis-aligned
    half-extent, a conservative over-approximation that only ever grows the
    footprint the floor must cover.
    """
    if not layout.walls:
        raise ValueError("floor_slab requires at least one wall")
    min_x = min(wall.center[0] - wall.size[0] / 2.0 for wall in layout.walls)
    max_x = max(wall.center[0] + wall.size[0] / 2.0 for wall in layout.walls)
    min_y = min(wall.center[1] - wall.size[1] / 2.0 for wall in layout.walls)
    max_y = max(wall.center[1] + wall.size[1] / 2.0 for wall in layout.walls)
    size_x = (max_x - min_x) + 2.0 * margin
    size_y = (max_y - min_y) + 2.0 * margin
    center_x = (min_x + max_x) / 2.0
    center_y = (min_y + max_y) / 2.0
    center_z = lift - thickness / 2.0
    return (center_x, center_y, center_z), (size_x, size_y, thickness)


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


def _bind_pbr_material(
    stage,
    prim,
    *,
    materials_scope: str,
    name: str,
    rgb: tuple[float, float, float],
    roughness: float,
    override_descendants: bool = False,
) -> None:
    """Author (once) and bind a matte ``UsdPreviewSurface`` under
    ``materials_scope`` to ``prim``.

    ``override_descendants`` binds at ``strongerThanDescendants`` strength so
    the binding wins over any material the prim's own subtree carries -- used
    for referenced furniture whose GLB material bindings would otherwise
    render (see FURNITURE_COLORS for why they don't resolve cleanly).
    """
    from pxr import Sdf, UsdShade

    material_path = f"{materials_scope}/{name}"
    material = UsdShade.Material.Get(stage, material_path)
    if not material:
        material = UsdShade.Material.Define(stage, material_path)
        shader = UsdShade.Shader.Define(stage, f"{material_path}/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(tuple(rgb))
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(roughness))
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    binding = UsdShade.MaterialBindingAPI.Apply(prim)
    if override_descendants:
        binding.Bind(material, bindingStrength=UsdShade.Tokens.strongerThanDescendants)
    else:
        binding.Bind(material)


def _bind_gray_material(stage, prim, materials_scope: str) -> None:
    _bind_pbr_material(
        stage, prim, materials_scope=materials_scope, name="Gray",
        rgb=(0.6, 0.6, 0.6), roughness=0.8,
    )


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

    # Visual wood floor covering the arena footprint. No collider -- physics
    # still rides the backend's default ground plane at z=0; the slab's top
    # face sits a hair above it (see floor_slab) so it wins the depth test
    # against Isaac's default ground grid without touching physics/nav.
    floor_center, floor_size = floor_slab(layout)
    floor_cube = UsdGeom.Cube.Define(stage, f"{root.GetPath()}/Floor")
    floor_cube.CreateSizeAttr(1.0)
    floor_xform = UsdGeom.Xformable(floor_cube.GetPrim())
    floor_xform.AddTranslateOp().Set(Gf.Vec3d(*floor_center))
    floor_xform.AddScaleOp().Set(Gf.Vec3f(*floor_size))
    _bind_pbr_material(
        stage, floor_cube.GetPrim(), materials_scope="/World/Arena/Materials",
        name="Floor", rgb=FLOOR_COLOR, roughness=FLOOR_ROUGHNESS,
    )

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
        # Solid PBR tint bound on the wrapper at strongerThanDescendants so it
        # overrides the referenced GLB subtree's own (non-resolving) materials
        # -- without this the furniture renders stark-white. One material per
        # model_id, reused across repeated placements.
        rgb, roughness = furniture_material(item.model_id)
        _bind_pbr_material(
            stage, wrapper.GetPrim(), materials_scope="/World/Arena/Materials",
            name=f"furn_{item.model_id}", rgb=rgb, roughness=roughness,
            override_descendants=True,
        )

    stage.Save()


def _axis_correction(raw_path: Path, *, trust_stage_metadata: bool) -> float:
    """Return the uniform scale factor that re-anchors one raw single-source
    conversion's units onto this codebase's metre convention. No rotation
    is ever applied here -- see below for why that differs from
    ``convert_glb_to_usd``'s furniture path.

    Unlike ``convert_glb_to_usd`` (which always corrects for a *known*,
    single source convention -- Kit's own Y-up/cm glTF default, folded with
    an SDF-declared scale/pose the caller supplies), ``convert_object_to_usd``
    has two independent single-mesh sources (DAE, STL) with no SDF-declared
    scale/pose to lean on (every allowlisted YCB object's model SDF declares
    an identity visual/collision pose and no mesh scale -- confirmed against
    the real pinned checkout).

    ``trust_stage_metadata`` encodes a live-confirmed, format-specific split
    (see the Task 10 report for the full measurement, done against two
    different objects): Kit's COLLADA importer genuinely reads a DAE's own
    declared ``<unit>`` and reports it faithfully via
    ``GetStageMetersPerUnit`` on the raw conversion -- the resulting
    geometry, scaled by that reported value, lines up (to within a
    fraction of a millimetre, axis by axis) with the sibling STL's raw
    points. Kit's STL importer, by contrast, performs **no** unit
    reconciliation at all: STL as a format carries no unit metadata, the
    raw imported points pass through completely unmodified, and the raw
    stage is nonetheless stamped with the same nominal
    ``metersPerUnit=0.01`` default Kit stamps on *any* freshly created
    stage regardless of content. Applying that stamp as a scale correction
    to an STL conversion was confirmed live to shrink the collision mesh to
    roughly 1/100th of the matching visual mesh's size -- STL-sourced
    conversions must use scale=1.0 instead, trusting the raw STL point data
    as already expressed directly in metres.

    Rotation was tested and dropped: both raw conversions report
    ``upAxis=Y`` (again Kit's own default stamp, not a reflection of
    actual content orientation), but a live per-axis bounds comparison
    across two different objects (cracker box, mug) showed the raw DAE and
    STL points already agree axis-for-axis with **no** rotation applied at
    all -- both source pipelines in this dataset (unlike the furniture
    GLBs, which really are Y-up per their own SDF-declared roll) already
    author directly in this codebase's Z-up target frame, since the whole
    ``tmc_wrs_gz`` repo targets Gazebo (itself Z-up) natively. Applying the
    Y-up-implied 90-degree correction here, as the furniture path does,
    was confirmed live to *misalign* the two meshes (their Y/Z extents no
    longer matched at all), not fix them.
    """
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.Open(str(raw_path))
    if not stage.GetDefaultPrim():
        raise RuntimeError(f"{raw_path}: converted USD has no default prim")
    if not trust_stage_metadata:
        return 1.0
    return UsdGeom.GetStageMetersPerUnit(stage)


def _check_bounds_overlap(
    geom_bound: tuple[tuple[float, float, float], tuple[float, float, float]],
    collision_bound: tuple[tuple[float, float, float], tuple[float, float, float]],
    *,
    min_ratio: float = 0.5,
    max_ratio: float = 2.0,
) -> None:
    """Fail closed unless the composed visual and collision AABBs are
    plausibly co-located and similarly sized on every axis.

    Pure Python (no pxr) so it is unit-testable directly -- extracted from
    ``_compose_object`` specifically for that (Task 10 review round,
    Finding 1). Nothing in ``_axis_correction``'s "STL scale=1.0, no
    rotation, trust DAE metersPerUnit" decision was otherwise checked at
    runtime before this: a future re-pin or allowlist addition whose STL
    is not already metre-expressed (or that genuinely needs a rotation,
    unlike every object measured so far) would previously have published a
    misaligned or wildly mis-scaled collision mesh with no error at all.
    The 0.5x-2.0x per-axis size-ratio tolerance mirrors the same check this
    task's own report used to manually verify alignment live (cracker box,
    mug, bowl all agreed to within a fraction of a millimetre against that
    same ratio window) -- loose enough to tolerate the two meshes' genuine
    shape differences (STL is typically a coarser decimation of the same
    scan than the textured DAE) while still catching the ~100x scale error
    and ~90-degree rotation mismatch this task hit live during development.
    """
    (gmin, gmax), (cmin, cmax) = geom_bound, collision_bound
    for axis in range(3):
        gsize = gmax[axis] - gmin[axis]
        csize = cmax[axis] - cmin[axis]
        if gsize <= 0 or csize <= 0:
            raise AssetArtifactError(
                f"visual/collision bounds check failed: degenerate extent on axis "
                f"{axis} (geom_bound={geom_bound}, collision_bound={collision_bound})"
            )
        ratio = csize / gsize
        if not (min_ratio <= ratio <= max_ratio):
            raise AssetArtifactError(
                f"visual/collision bounds check failed: collision/visual size ratio "
                f"{ratio:.4f} on axis {axis} is outside [{min_ratio}, {max_ratio}] -- "
                f"geom_bound={geom_bound}, collision_bound={collision_bound}"
            )


def author_object_rigid_body(stage) -> None:
    """Author dynamic rigid-body physics on a composed object's default prim.

    Every spawnable asset in this codebase carries its own physics (see
    ``simulation/assets/primitives/task-object.usda``): ``RigidBodyAPI`` +
    ``PhysxRigidBodyAPI`` on the default prim, mass left to PhysX's
    density-derived computation over the collision hulls (scene default
    1000 kg/m^3 lands within ~2x of the published YCB masses).  Without
    this, a spawned object composes in as static scenery: the simulation's
    rigid-body view for it never resolves, its ground-truth pose is
    silently absent from every physics-truth frame, and a grasp can never
    move it.  Requires an already-authored collision subtree; fails closed
    otherwise, because a dynamic body with no collider falls through the
    world.
    """
    from pxr import Usd, UsdPhysics

    default_prim = stage.GetDefaultPrim()
    if not default_prim:
        raise RuntimeError("rigid-body authoring requires a default prim")
    if not any(
        prim.HasAPI(UsdPhysics.CollisionAPI) for prim in Usd.PrimRange(default_prim)
    ):
        raise RuntimeError(
            f"{default_prim.GetPath()}: rigid-body authoring requires an authored collider"
        )
    UsdPhysics.RigidBodyAPI.Apply(default_prim)
    try:
        from pxr import PhysxSchema

        PhysxSchema.PhysxRigidBodyAPI.Apply(default_prim)
    except ImportError:
        # Outside a Kit process the PhysX schema plugin may be absent; the
        # UsdPhysics rigid body alone is sufficient for simulation.
        pass


def _compose_object(raw_visual_path: Path, raw_collision_path: Path, usd_path: Path) -> None:
    """Wrap the two independent raw conversions (DAE visual, STL collision)
    into one flattened ``usd_path``, structured per the collider-placement
    rule confirmed in Task 9 (Finding 2 -- USD reference composition only
    ever pulls in the *referenced prim's own subtree*):

        /World             plain Xform, the default prim, no xformOps of
                            its own -- so a later reference to this whole
                            file composes in both children below together.
        /World/geom         visual: per-source unit-scale correction op
                             (no rotation -- see ``_axis_correction``),
                             referencing the raw DAE conversion on a child
                             (not itself, to avoid the "xformOp:translate
                             already exists" collision documented for
                             ``_rebase_converted_mesh``).
        /World/collision     collision: same shape, referencing the raw STL
                              conversion. A *sibling* of ``geom``, so making
                              it invisible never touches the visual mesh's
                              own visibility.

    Both children live under the default prim, satisfying the structure
    rule this whole importer follows: colliders and visual content both
    reachable from a single reference to the composed file.
    """
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    visual_scale = _axis_correction(raw_visual_path, trust_stage_metadata=True)
    collision_scale = _axis_correction(raw_collision_path, trust_stage_metadata=False)

    wrapper = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(wrapper, 1.0)
    UsdGeom.SetStageUpAxis(wrapper, UsdGeom.Tokens.z)
    root = UsdGeom.Xform.Define(wrapper, "/World")
    wrapper.SetDefaultPrim(root.GetPrim())

    geom = UsdGeom.Xform.Define(wrapper, "/World/geom")
    geom_xform = UsdGeom.Xformable(geom.GetPrim())
    geom_xform.AddScaleOp().Set(Gf.Vec3f(visual_scale, visual_scale, visual_scale))
    visual_ref = wrapper.DefinePrim("/World/geom/mesh")
    visual_ref.GetReferences().AddReference(str(raw_visual_path))

    collision = UsdGeom.Xform.Define(wrapper, "/World/collision")
    collision_xform = UsdGeom.Xformable(collision.GetPrim())
    collision_xform.AddScaleOp().Set(Gf.Vec3f(collision_scale, collision_scale, collision_scale))
    collision_ref = wrapper.DefinePrim("/World/collision/mesh")
    collision_ref.GetReferences().AddReference(str(raw_collision_path))

    # Flatten: the published artifact only ever carries usd_path's bytes,
    # never the *.raw.usd siblings, so the references above must not
    # survive into the final file.
    flattened = wrapper.Flatten()
    flattened.Export(str(usd_path))

    # Author collision physics + invisibility on the flattened result, then
    # re-export fresh (see _resave_fresh's docstring for why an in-place
    # Save() is unsafe: leftover string-pool bytes from the just-flattened
    # export would defeat both the scratch-path grep and publish
    # determinism).
    stage = Usd.Stage.Open(str(usd_path))
    default_prim = stage.GetDefaultPrim()
    if not default_prim:
        raise RuntimeError(f"{usd_path}: no default prim to author collision on")
    geom_root = stage.GetPrimAtPath(default_prim.GetPath().AppendChild("geom"))
    collision_root = stage.GetPrimAtPath(default_prim.GetPath().AppendChild("collision"))
    found_mesh = False
    for prim in Usd.PrimRange(collision_root):
        if prim.IsA(UsdGeom.Mesh):
            found_mesh = True
            api = UsdPhysics.MeshCollisionAPI.Apply(prim)
            api.CreateApproximationAttr("convexDecomposition")
            UsdPhysics.CollisionAPI.Apply(prim)
    if not found_mesh:
        raise RuntimeError(f"{usd_path}: no collision mesh geometry found under {collision_root.GetPath()}")
    UsdGeom.Imageable(collision_root).MakeInvisible()
    author_object_rigid_body(stage)

    # Fail-closed runtime guard (Task 10 review round, Finding 1): nothing
    # up to this point actually checks that "STL scale=1.0, no rotation,
    # trust DAE metersPerUnit" (_axis_correction's decision) produced two
    # meshes that are genuinely co-located and similarly sized -- a future
    # re-pin or allowlist addition whose STL is not already metre-expressed
    # (or that needs a rotation none of the objects measured so far needed)
    # would otherwise publish a silently misaligned collision mesh.
    # ignoreVisibility=True: collision_root was just made invisible above,
    # and its bound must still be measured regardless.
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"], False, True)
    geom_range = cache.ComputeWorldBound(geom_root).ComputeAlignedRange()
    collision_range = cache.ComputeWorldBound(collision_root).ComputeAlignedRange()
    geom_bound = (tuple(geom_range.GetMin()), tuple(geom_range.GetMax()))
    collision_bound = (tuple(collision_range.GetMin()), tuple(collision_range.GetMax()))
    _check_bounds_overlap(geom_bound, collision_bound)

    _resave_fresh(stage, usd_path)


def _patch_dae_missing_unit(dae_bytes: bytes, source_label: str) -> bytes | None:
    """Work around a live-confirmed native crash in Kit's asset-converter
    COLLADA importer: every allowlisted YCB object's ``textured.dae``
    (VCGLab/MeshLab-authored, confirmed against the real pinned
    ``tmc_wrs_gz`` checkout) declares an ``<asset>`` block with
    ``<up_axis>`` but no ``<unit>`` element, and feeding one of these files
    to ``omni.kit.asset_converter`` segfaults the whole Kit process --
    backtrace bottoms out in ``libusd_convert_asset.so`` calling into
    ``tinyxml2::XMLElement::FindAttribute`` (i.e. the importer looks up the
    ``<unit>`` element's ``meter`` attribute without null-checking that the
    optional element exists at all). COLLADA 1.4.1's own spec default for a
    missing ``<unit>`` is exactly ``meter="1.0"`` -- the value injected here
    -- so this only makes an already-implied default explicit; it changes
    no semantic content of the mesh. Confirmed live: the unpatched file
    segfaults the Kit process (exit 139) at ``wait_until_finished()``; the
    byte-patched copy converts cleanly to the identical geometry.

    ``source_label`` is only used to name the file in the error message
    below -- pass ``str(dae_path)`` (this function itself has no filesystem
    access, so it cannot discover a path on its own).

    Returns ``None`` (nothing to patch) if a ``<unit>`` element is already
    present. Raises ``AssetArtifactError`` (fail closed, not ``None``) if no
    ``<unit>`` is present *and* no ``<up_axis>`` anchor is found to insert
    one before -- an ``<asset>`` block shaped differently than every
    allowlisted object's real DAE, which this targeted workaround does not
    know how to fix. Review fix (Task 10 review round, Finding 2): an
    earlier version returned ``None`` here, silently falling through to
    feed the original, still crash-triggering bytes straight to Kit --
    exactly the native segfault this function exists to prevent, just
    deferred and with a far less actionable failure (a raw process crash
    instead of a Python exception naming the file and the missing anchor).
    """
    text = dae_bytes.decode("utf-8")
    if "<unit" in text:
        return None
    marker = "<up_axis>"
    if marker not in text:
        raise AssetArtifactError(
            f"{source_label}: DAE <asset> block has neither <unit> nor <up_axis> -- "
            "the missing-<unit> Kit-crash workaround (see _patch_dae_missing_unit) "
            "does not know how to patch this file; refusing to feed it to the "
            "converter unpatched"
        )
    return text.replace(marker, '<unit name="meter" meter="1"/>' + marker, 1).encode("utf-8")


def _prepare_dae_input(dae_path: Path, scratch_dir: Path) -> Path:
    """Return the path Kit's asset converter should actually read for
    ``dae_path``: the original file, unless ``_patch_dae_missing_unit``
    finds the crash-triggering missing ``<unit>`` element, in which case a
    sibling-preserving mirror of ``dae_path``'s directory is built under
    ``scratch_dir / "_dae_input"`` (so any relative ``<init_from>`` texture
    reference the DAE makes still resolves against a sibling file, exactly
    as it would next to the original) with only the DAE's own bytes
    patched. The mirror lives outside both ``usd_path`` itself and the
    ``textures/`` directory ``_relocate_local_assets`` populates, so it is
    never swept into the published payload.
    """
    dae_bytes = dae_path.read_bytes()
    patched = _patch_dae_missing_unit(dae_bytes, str(dae_path))
    if patched is None:
        return dae_path
    mirror_dir = scratch_dir / "_dae_input"
    mirror_dir.mkdir(parents=True, exist_ok=True)
    for sibling in dae_path.parent.iterdir():
        if sibling.is_file() and sibling.name != dae_path.name:
            (mirror_dir / sibling.name).write_bytes(sibling.read_bytes())
    patched_path = mirror_dir / dae_path.name
    patched_path.write_bytes(patched)
    return patched_path


def convert_object_to_usd(dae_path: Path, stl_path: Path, usd_path: Path) -> None:
    """Convert one YCB object's textured visual DAE and non-textured
    collision STL into a single composed ``usd_path``: visual mesh visible
    under ``<default prim>/geom``, collision mesh invisible under
    ``<default prim>/collision`` with ``UsdPhysics.MeshCollisionAPI``
    (``convexDecomposition`` approximation) applied, both reachable from a
    single reference to the whole file (see ``_compose_object``).

    Structurally the same two-phase shape as ``convert_glb_to_usd``: convert
    each source via the same ``omni.kit.asset_converter`` task used for
    furniture GLBs (it accepts COLLADA/``.dae`` and ``.stl`` sources too),
    then re-anchor onto this codebase's metre/Z-up convention -- except here
    there are *two* independent raw conversions to re-anchor (see
    ``_axis_correction``'s docstring for why each is corrected
    independently rather than assuming a shared convention), and no
    SDF-declared scale/pose to fold in (every allowlisted object's SDF
    declares identity visual/collision poses -- confirmed against the real
    pinned checkout, so unlike furniture this importer never reads a model
    SDF at all). See ``_patch_dae_missing_unit`` for a live-discovered Kit
    crash workaround applied to the DAE input before conversion.
    """
    import asyncio

    import omni.kit.asset_converter as asset_converter

    usd_path.parent.mkdir(parents=True, exist_ok=True)
    raw_visual_path = usd_path.with_name(usd_path.stem + ".visual.raw.usd")
    raw_collision_path = usd_path.with_name(usd_path.stem + ".collision.raw.usd")
    dae_input_path = _prepare_dae_input(dae_path, usd_path.parent)

    async def _run() -> None:
        instance = asset_converter.get_instance()
        visual_task = instance.create_converter_task(str(dae_input_path), str(raw_visual_path))
        if not await visual_task.wait_until_finished():
            raise RuntimeError(
                f"asset conversion failed for {dae_path}: {visual_task.get_error_message()}"
            )
        collision_task = instance.create_converter_task(str(stl_path), str(raw_collision_path))
        if not await collision_task.wait_until_finished():
            raise RuntimeError(
                f"asset conversion failed for {stl_path}: {collision_task.get_error_message()}"
            )

    asyncio.get_event_loop().run_until_complete(_run())
    _compose_object(raw_visual_path, raw_collision_path, usd_path)
    # object_id: usd_path is always scratch/<object_id>/object.usd (see
    # ycb_import.convert_ycb_object), so the parent directory name is the
    # object id -- namespaces any relocated texture files the same way
    # convert_glb_to_usd namespaces furniture textures by model_id.
    _relocate_local_assets(usd_path, raw_visual_path.parent, usd_path.parent.name)


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
