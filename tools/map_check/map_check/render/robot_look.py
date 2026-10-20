"""Map Check's own colours for Tinker in renders, so the robot looks like the real one.

The shared robot artifact (also loaded by GPSR and tk27) is nearly all white: the base box
and its supporting beams are one white mesh (base_link), and the URDF's dark wheels and
lidar lose their colour on import. Renders recolour only Map Check's referenced copy, in
the render stage: black base and wheels, grey beams, dark lidar, white xArm. Links not
listed keep the artifact's materials; a link the artifact lacks is reported and skipped.

pxr is imported inside the functions, so importing this module needs only numpy.
"""
from __future__ import annotations

import numpy as np

COLOURS = {  # name: (rgb, roughness)
    "black": ((0.035, 0.035, 0.04), 0.6),
    "grey": ((0.42, 0.43, 0.45), 0.45),
    "lidar": ((0.07, 0.07, 0.075), 0.4),
    "white": ((0.92, 0.92, 0.9), 0.35),
}
LINK_LOOK = {
    "front_left_wheel": "black", "front_right_wheel": "black",
    "rear_left_wheel": "black", "rear_right_wheel": "black", "ballast": "black",
    "livox_frame": "lidar",
    "link_base": "white", "link1": "white", "link2": "white", "link3": "white",
    "link4": "white", "link5": "white", "link6": "white", "xarm_gripper_base_link": "white",
}
BASE_LINK = "base_link"  # one mesh: the base box below BASE_TOP_Z, the beams above it
BASE_TOP_Z = 0.215  # m in the base_link frame; the box's top face is at 0.21


def define_materials(stage, root: str) -> dict:
    """One UsdPreviewSurface material per COLOURS entry, under `root`."""
    return {name: preview_material(stage, f"{root}/{name}", rgb, rough)
            for name, (rgb, rough) in COLOURS.items()}


def preview_material(stage, path: str, rgb, rough: float):
    from pxr import Gf, Sdf, UsdShade

    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + "/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def apply(stage, robot_root, mats: dict) -> list[str]:
    """Recolour the robot referenced at `robot_root`; returns the links it could not find."""
    from pxr import UsdGeom, UsdShade

    missing = []
    for link in [BASE_LINK, *LINK_LOOK]:
        gprims = _visual_gprims(robot_root, link)
        if not gprims:
            missing.append(link)
            continue
        if link != BASE_LINK:
            for prim in gprims:
                UsdShade.MaterialBindingAPI.Apply(prim).Bind(mats[LINK_LOOK[link]])
            continue
        for prim in gprims:
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(mats["grey"])
            if prim.IsA(UsdGeom.Mesh):
                _split_base(UsdGeom.Mesh(prim), robot_root.GetChild(BASE_LINK), mats)
    return missing


def face_top_z(mesh, link_prim) -> np.ndarray:
    """Per face, the highest vertex z in the link's frame."""
    from pxr import Usd, UsdGeom

    t = Usd.TimeCode.Default()
    m = np.array(UsdGeom.Xformable(mesh).ComputeLocalToWorldTransform(t)
                 * UsdGeom.Xformable(link_prim).ComputeLocalToWorldTransform(t).GetInverse())
    pts = np.asarray(mesh.GetPointsAttr().Get(), dtype=float)
    z = (np.c_[pts, np.ones(len(pts))] @ m)[:, 2]  # USD matrices act on row vectors
    counts = np.asarray(mesh.GetFaceVertexCountsAttr().Get())
    idx = np.asarray(mesh.GetFaceVertexIndicesAttr().Get())
    return np.maximum.reduceat(z[idx], np.r_[0, np.cumsum(counts)[:-1]])


def _split_base(mesh, link_prim, mats) -> None:
    from pxr import UsdGeom, UsdShade

    top = face_top_z(mesh, link_prim)
    api = UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim())
    for name, faces, look in (("base", np.flatnonzero(top <= BASE_TOP_Z), "black"),
                              ("beams", np.flatnonzero(top > BASE_TOP_Z), "grey")):
        subset = api.CreateMaterialBindSubset(name, faces.tolist(), UsdGeom.Tokens.face)
        UsdShade.MaterialBindingAPI.Apply(subset.GetPrim()).Bind(mats[look])
    api.SetMaterialBindSubsetsFamilyType(UsdGeom.Tokens.partition)


def _visual_gprims(robot_root, link) -> list:
    """The link's rendered gprims, made editable and ready for their own bindings.

    The instanced visuals become local, and the artifact's bindings above each gprim (the
    importer binds the URDF material on the mesh's parent, strongerThanDescendants) are
    weakened in this stage, so a gprim's own binding and its subsets decide its look.
    """
    from pxr import Usd, UsdGeom, UsdShade

    link_prim = robot_root.GetChild(link)
    visuals = link_prim.GetChild("visuals") if link_prim else None
    if not visuals:
        return []
    if visuals.IsInstanceable():
        visuals.SetInstanceable(False)
    gprims = []
    for p in Usd.PrimRange(visuals):
        if p.IsA(UsdGeom.Gprim) and UsdGeom.Imageable(p).ComputePurpose() != UsdGeom.Tokens.guide:
            gprims.append(p)
        elif not p.IsA(UsdGeom.Gprim):
            rel = UsdShade.MaterialBindingAPI(p).GetDirectBindingRel()
            if rel and rel.HasAuthoredTargets():
                UsdShade.MaterialBindingAPI.SetMaterialBindingStrength(
                    rel, UsdShade.Tokens.weakerThanDescendants)
    return gprims
