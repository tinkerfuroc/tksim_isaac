"""Isaac Sim side of design_import: URDF -> USD through isaacsim.asset.importer.urdf.

Ported from tk26_sim/src/isaac_bringup/scripts/verify_in_isaac.py, but that
script targets an older Isaac Sim API this hook no longer uses (see below).
Import config: no merged fixed joints requested, free base, URDF inertia
honoured, mimic tags left to the importer's default handling.

Isaac Sim 6.0.1 replaced the old ``omni.kit.commands.execute("URDFParseAndImportFile", ...)``
+ ``isaacsim.asset.importer.urdf._urdf.ImportConfig`` binding (still what
verify_in_isaac.py and the old tk26_sim tooling use) with a native Python
``URDFImporter``/``URDFImporterConfig`` API; the old ``_urdf`` module is kept
only as a deprecated shim with no ``ImportConfig`` attribute. This hook
targets the new API directly (found live, 2026-09-13 — see
docs/developer-log.md). ``URDFImporter.import_urdf()`` does not return a
prim path or leave a stage open in the shared USD context: it writes a
self-contained ``<robot_name>/<robot_name>.usda`` under the directory named
by ``config.usd_path`` and returns that file's path, so joints and the root
prim are read back by opening it, and the caller's exact ``usd_path`` is
produced by exporting that opened stage.

**Fixed-joint collapse is NOT reproducible on this importer, regardless of
``merge_fixed_joints``.** Measured live (2026-09-13): `designs/tinker2_ref`'s
source URDF has 47 joints (26 fixed, 21 movable); the tinker2 artifact's own
USD (built by the pre-6.0.1 importer) keeps 46 of them (25 fixed + 21
movable); this hook's import of the same URDF keeps only 27 (6 fixed + 21
movable) — 19 fewer fixed joints than the shipped baseline. Root cause,
read from ``urdf_usd_converter/_impl/link_hierarchy.py`` (``LinkHierarchy.
_ghost_links_chain``/``_check_remove_rigid_body_flag``) and ``_impl/link.py``
(``convert_link``): any link with no inertial, no visual, and no collision,
reached only through a chain of fixed joints, is a "ghost link" — the
importer skips ``RigidBodyAPI`` for it entirely (and for the whole ghost
chain feeding it) regardless of any config. There is no config knob to
disable this — checked both ``URDFImporterConfig``
(``isaacsim.asset.importer.urdf``) and ``Converter.Params``
(``urdf_usd_converter._impl.convert``); neither has a ghost-link,
rigid-body, or fixed-joint-retention field. The 19 links this drops are pure
massless fixed-joint frames (sensor/mount frames with no geometry); they are
retained as plain ``Xform`` prims (no ``RigidBodyAPI``/no ``Joint`` prim), so
nothing is silently missing from the USD, only the physics-joint count in
the importer's own log line differs from the old importer's output.
Structural parity with the pre-6.0.1-importer tinker2 USD is **not**
reproducible on Isaac 6.0.1's bundled ``urdf_usd_converter`` as of this
writing; whether that matters is explicitly deferred to M2's tinker2-vs-
tinker2_ref live smoke, not decided here.

Also found live, in ``urdf_usd_converter`` 0.1.3 (the pip package Isaac Sim
6.0.1 bundles for this importer, not our code): ``store_dae_material_data``
(``_impl/material.py:362``) sets a DAE material's identification name to
``material.id if use_material_id else material.name`` — it falls back to the
Collada ``id`` (always present) only when duplicate ``name``s force
disambiguation, never when a single material's ``name`` (always optional in
Collada) is simply absent. That leaves ``material_data.name = None``, which
later crashes ``MaterialCache.store_safe_names`` -> ``NameCache
.getPrimNames()`` (a stricter pybind11 signature that rejects ``None`` in
the batch of names) — tinker2_ref's ``realsense2_description/meshes/d435.dae``
has exactly one such material. ``_patch_dae_material_ids`` wraps
``store_dae_material_data`` at its call site (``conversion_collada.py``,
the only place it's invoked) to backfill any ``None`` name from the same
Collada material's ``.id``, i.e. do what the converter's own ``use_material_id``
branch already does, for the one case it misses. Version-guarded
(``urdf_usd_converter.__version__ == "0.1.3"``, skips with a log line
otherwise) since this is someone else's bug, not ours: **remove this patch
once ``urdf_usd_converter`` > 0.1.3 ships and handles an unnamed DAE
material without a duplicate; re-test against
``realsense2_description/meshes/d435.dae`` (tinker2_ref) when that happens.**

(An earlier hypothesis — that a `<visual>` with no `<material>` at all,
`xarm_camera_link` in tinker2_ref, fed the stray `None` — was tested live
and falsified: patching a default material onto that visual added a new,
correctly-named entry to the list but left the pre-existing `None` in place
on the next boot. That patch has been reverted; `import_urdf` no longer
mutates its input URDF.)
"""
from __future__ import annotations

from pathlib import Path


def _under(prim_path: str, candidate: str) -> bool:
    """True when candidate is prim_path itself or a descendant (segment-aware, not a string prefix)."""
    return candidate == prim_path or candidate.startswith(prim_path.rstrip("/") + "/")


def _patch_dae_material_ids() -> None:
    """Backfill unnamed DAE material names from their Collada id (see module docstring). Idempotent.

    No-op (with a log line) once urdf_usd_converter is no longer exactly 0.1.3 — see the
    module docstring's removal trigger.
    """
    import urdf_usd_converter

    if getattr(urdf_usd_converter, "__version__", None) != "0.1.3":
        print(f"design_convert: skipped the urdf_usd_converter DAE-material-name patch "
              f"(installed version {getattr(urdf_usd_converter, '__version__', None)!r} != '0.1.3'; "
              f"see tools/design_convert.py's module docstring)", flush=True)
        return

    from urdf_usd_converter._impl import conversion_collada, material

    original = material.store_dae_material_data
    if getattr(original, "_design_import_patched", False):
        return

    def _patched(mesh_file_path, _collada, data):
        before = len(data.material_data_list)
        original(mesh_file_path, _collada, data)
        appended = data.material_data_list[before:]
        for material_data, dae_material in zip(appended, _collada.materials):
            if material_data.name is None:
                material_data.name = dae_material.id

    _patched._design_import_patched = True
    material.store_dae_material_data = _patched
    conversion_collada.store_dae_material_data = _patched


class IsaacHooks:
    def import_urdf(self, urdf_path: Path, usd_path: Path) -> None:
        from isaacsim.core.utils.extensions import enable_extension

        enable_extension("isaacsim.asset.importer.urdf")
        from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
        from pxr import Usd

        _patch_dae_material_ids()
        usd_path.parent.mkdir(parents=True, exist_ok=True)
        # collision_type is read by URDFImporter only when collision_from_visuals=True
        # (isaacsim.asset.importer.urdf.impl.converter, ~line 211); we never set that, so it
        # is not passed here. The convex-hull collision on tinker2's meshes instead comes
        # unconditionally from urdf_usd_converter/_impl/geometry.py hard-coding
        # UsdPhysics.Tokens.convexHull for every mesh collider it builds.
        config = URDFImporterConfig(
            urdf_path=str(urdf_path),
            usd_path=str(usd_path.parent),
            merge_fixed_joints=False,
            fix_base=False,
        )
        final_path = URDFImporter(config).import_urdf()
        if not final_path or not Path(final_path).is_file():
            raise RuntimeError(f"URDFImporter.import_urdf() wrote no USD (returned {final_path!r})")
        stage = Usd.Stage.Open(final_path)
        if not stage:
            raise RuntimeError(f"failed to open imported stage at {final_path}")
        root_prim = stage.GetDefaultPrim()
        if not root_prim or not root_prim.IsValid():
            raise RuntimeError(f"imported stage {final_path} has no valid default prim")
        prim_path = str(root_prim.GetPath())
        joints = sum(1 for prim in stage.Traverse() if _under(prim_path, str(prim.GetPath())) and "Joint" in (prim.GetTypeName() or ""))
        if joints == 0:
            raise RuntimeError(f"imported {prim_path} has no joints")
        if not stage.Export(str(usd_path)):
            raise RuntimeError(f"stage.Export({usd_path}) returned False")
        print(f"design_convert: imported {prim_path} ({joints} joints) -> {usd_path}", flush=True)
