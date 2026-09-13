"""Isaac Sim side of design_import: URDF -> USD through isaacsim.asset.importer.urdf.

Ported from tk26_sim/src/isaac_bringup/scripts/verify_in_isaac.py. Import
config matches the tinker2 artifact: no merged fixed joints, free base,
URDF inertia honoured, mimic tags are left to the importer's default
handling. Convex decomposition is off for primitives (the box/cylinder
chassis) — meshes get the importer's default convex hull, which is what
tinker2's arm links use today.

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

Also found live, both in ``urdf_usd_converter`` 0.1.3 (the pip package Isaac
Sim 6.0.1 bundles for this importer, not our code):

1. ``MaterialCache.store_safe_names`` crashes ``NameCache.getPrimNames()``
   whenever any entry of ``data.material_data_list`` has ``name is None``, on
   the new (stricter) pybind11 signature. Root cause, read from
   ``urdf_usd_converter/_impl/material.py::store_dae_material_data``: a DAE
   mesh's embedded ``<material>`` always has a Collada ``id`` but ``name`` is
   optional; the converter only falls back to ``id`` when duplicate names
   force disambiguation (``use_material_id``), never for a single unnamed
   material — so one nameless embedded material in a DAE (tinker2_ref's
   ``realsense2_description/meshes/d435.dae`` has exactly this) leaves
   ``material_data.name = None`` and crashes the whole import.
   ``_patch_none_material_names`` monkeypatches ``store_safe_names`` to give
   any such entry a synthesized name before the real implementation runs —
   confirmed by first reproducing the exact crash with a URDF-level fix that
   left the ``None`` in place (a falsified hypothesis, see
   docs/developer-log.md), then tracing it to this DAE code path.
2. Separately, and harmlessly: a ``<visual>`` with no ``<material>`` element
   at all (tinker2_ref's ``xarm_camera_link``) also feeds an anonymous entry
   into the same list. ``_default_missing_materials`` patches the transient
   Isaac-only URDF copy (never the published one) to give every bare
   ``<visual>`` an explicit default material before import, independent of
   fix #1.
"""
from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path


def _under(prim_path: str, candidate: str) -> bool:
    """True when candidate is prim_path itself or a descendant (segment-aware, not a string prefix)."""
    return candidate == prim_path or candidate.startswith(prim_path.rstrip("/") + "/")


def _default_missing_materials(root: ET.Element) -> int:
    """Give every <visual> with no <material> a default one; return how many were patched."""
    patched = 0
    for visual in root.iter("visual"):
        if visual.find("material") is None:
            material = ET.SubElement(visual, "material", {"name": "_design_import_default"})
            ET.SubElement(material, "color", {"rgba": "0.6 0.6 0.6 1.0"})
            patched += 1
    return patched


def _patch_none_material_names() -> None:
    """Monkeypatch urdf_usd_converter's MaterialCache to name unnamed DAE materials before
    NameCache.getPrimNames() sees them (see module docstring, gotcha 1). Idempotent."""
    from urdf_usd_converter._impl import material_cache

    cls = material_cache.MaterialCache
    if getattr(cls.store_safe_names, "_design_import_patched", False):
        return
    original = cls.store_safe_names

    def _patched(self, data):
        for index, entry in enumerate(data.material_data_list):
            if entry.name is None:
                entry.name = f"unnamed_material_{index}"
        return original(self, data)

    _patched._design_import_patched = True
    cls.store_safe_names = _patched


class IsaacHooks:
    def import_urdf(self, urdf_path: Path, usd_path: Path) -> None:
        from isaacsim.core.utils.extensions import enable_extension

        enable_extension("isaacsim.asset.importer.urdf")
        from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
        from pxr import Usd

        _patch_none_material_names()
        usd_path.parent.mkdir(parents=True, exist_ok=True)
        urdf_root = ET.parse(urdf_path).getroot()
        patched = _default_missing_materials(urdf_root)
        if patched:
            urdf_path.write_bytes(ET.tostring(urdf_root, encoding="utf-8", xml_declaration=True))
            print(f"design_convert: patched {patched} <visual> element(s) with no <material> "
                  f"(urdf_usd_converter getPrimNames workaround) in {urdf_path}", flush=True)
        config = URDFImporterConfig(
            urdf_path=str(urdf_path),
            usd_path=str(usd_path.parent),
            merge_fixed_joints=False,
            fix_base=False,
            collision_type="Convex Hull",
        )
        final_path = URDFImporter(config).import_urdf()
        if not final_path or not Path(final_path).is_file():
            raise RuntimeError(f"URDFImporter.import_urdf() wrote no USD (returned {final_path!r})")
        stage = Usd.Stage.Open(final_path)
        if not stage:
            raise RuntimeError(f"failed to open imported stage at {final_path}")
        root_prim = stage.GetDefaultPrim()
        prim_path = str(root_prim.GetPath()) if root_prim and root_prim.IsValid() else "/"
        joints = sum(1 for prim in stage.Traverse() if _under(prim_path, str(prim.GetPath())) and "Joint" in (prim.GetTypeName() or ""))
        if joints == 0:
            raise RuntimeError(f"imported {prim_path} has no joints")
        if not stage.Export(str(usd_path)):
            raise RuntimeError(f"stage.Export({usd_path}) returned False")
        print(f"design_convert: imported {prim_path} ({joints} joints) -> {usd_path}", flush=True)
