"""Isaac Sim side of design_import: URDF -> USD through isaacsim.asset.importer.urdf.

Ported from tk26_sim/src/isaac_bringup/scripts/verify_in_isaac.py. Import
config matches the tinker2 artifact: no merged fixed joints, free base,
URDF inertia honoured, mimics parsed. Convex decomposition is off for
primitives (the box/cylinder chassis) — meshes get the importer's default
convex hull, which is what tinker2's arm links use today.
"""
from __future__ import annotations

from pathlib import Path


class IsaacHooks:
    def import_urdf(self, urdf_path: Path, usd_path: Path) -> None:
        import omni.kit.commands
        import omni.usd
        from isaacsim.core.utils.extensions import enable_extension

        enable_extension("isaacsim.asset.importer.urdf")
        from isaacsim.asset.importer.urdf import _urdf

        config = _urdf.ImportConfig()
        config.merge_fixed_joints = False
        config.convex_decomp = False
        config.replace_cylinders_with_capsules = False
        config.import_inertia_tensor = True
        config.fix_base = False
        config.distance_scale = 1.0
        config.density = 0.0
        status, prim_path = omni.kit.commands.execute(
            "URDFParseAndImportFile", urdf_path=str(urdf_path), import_config=config,
        )
        if not status or not prim_path:
            raise RuntimeError(f"URDFParseAndImportFile returned status={status!r} prim_path={prim_path!r}")
        stage = omni.usd.get_context().get_stage()
        joints = sum(1 for prim in stage.Traverse() if str(prim.GetPath()).startswith(prim_path) and "Joint" in (prim.GetTypeName() or ""))
        if joints == 0:
            raise RuntimeError(f"imported {prim_path} has no joints")
        usd_path.parent.mkdir(parents=True, exist_ok=True)
        if not stage.Export(str(usd_path)):
            raise RuntimeError(f"stage.Export({usd_path}) returned False")
        print(f"design_convert: imported {prim_path} ({joints} joints) -> {usd_path}", flush=True)
