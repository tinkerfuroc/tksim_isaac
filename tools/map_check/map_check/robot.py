"""Robot: artifact files, arm poses, URDF forward kinematics, link surface points.

Imports numpy and the standard library only at module level, because the Isaac
render script imports this module; pxr is imported inside link_points.
"""
from __future__ import annotations

import hashlib
import importlib.util
import json
import math
import os
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np

ARM_JOINTS = tuple(f"joint{i}" for i in range(1, 8))
PAN_JOINT = "pan_joint"
TILT_JOINT = "tilt_joint"
ROOT_LINK = "base_link"

# grounded_base_height(robot.urdf) for the current Tinker 2 artifact; used when
# simulation/tinker_sim_isaac/base_spawn.py is not present in the repo.
GROUNDED_Z = 0.0775

SAMPLE_SPACING = 0.02  # m, longest sub-triangle edge when sampling a face
_POINTS_CACHE_VERSION = 1
_CYLINDER_CHORD = 0.01  # m, tessellation step around a cylinder
_DRAWN_PURPOSES = ("default", "render")

# tools/map_check/map_check/robot.py -> the directory containing tools/
_REPO_ROOT = Path(__file__).resolve().parents[3]


@dataclass(frozen=True)
class RobotFiles:
    dir: Path
    urdf: Path
    usd: Path


def resolve_robot(pointer: Path) -> RobotFiles:
    """Follow <repo>/artifacts/robot/<name>/current.json to the artifact files."""
    pointer = Path(pointer)
    try:
        doc = json.loads(pointer.read_text())
        rel = doc["artifact_dir"]
    except (OSError, ValueError, KeyError, TypeError) as e:
        raise ValueError(f"{pointer}: cannot read artifact_dir: {e}") from e
    # artifact_dir is relative to the repo root, three levels above the pointer.
    d = Path(rel) if Path(rel).is_absolute() else pointer.parents[3] / rel
    files = RobotFiles(dir=d, urdf=d / "robot.urdf", usd=d / "robot.usd")
    for f in (files.urdf, files.usd):
        if not f.is_file():
            raise FileNotFoundError(f"robot artifact file missing: {f}")
    return files


def arm_poses(constants_json: Path) -> dict[str, list[float]]:
    """Every arm_pos_* entry (7 joint angles, degrees), in file order."""
    path = Path(constants_json)
    try:
        doc = json.loads(path.read_text())
    except (OSError, ValueError) as e:
        raise ValueError(f"{path}: cannot read: {e}") from e
    if not isinstance(doc, dict):
        raise ValueError(f"{path}: expected a JSON object")
    out = {}
    for k, v in doc.items():
        if not k.startswith("arm_pos_"):
            continue
        if (not isinstance(v, list) or len(v) != len(ARM_JOINTS)
                or not all(isinstance(a, (int, float)) and not isinstance(a, bool) for a in v)):
            raise ValueError(f"{path}: {k} is not a list of {len(ARM_JOINTS)} numbers")
        out[k] = [float(a) for a in v]
    return out


def joint_values(arm_deg: Sequence[float], pan_deg: float, tilt_deg: float) -> dict[str, float]:
    """Degrees in, radians out, keyed by URDF joint name."""
    if len(arm_deg) != len(ARM_JOINTS):
        raise ValueError(f"expected {len(ARM_JOINTS)} arm angles, got {len(arm_deg)}")
    q = {j: math.radians(a) for j, a in zip(ARM_JOINTS, arm_deg)}
    q[PAN_JOINT] = math.radians(pan_deg)
    q[TILT_JOINT] = math.radians(tilt_deg)
    return q


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _axis_angle(axis, q):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(q) * k + (1 - math.cos(q)) * k @ k


def link_transforms(urdf: Path, q: Mapping[str, float]) -> dict[str, np.ndarray]:
    """4x4 transform of every link relative to base_link; q in radians (missing = 0)."""
    q = q or {}
    tree = ET.parse(urdf).getroot()
    by_child = {}
    for j in tree.findall("joint"):
        o = j.find("origin")
        xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
        rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
        ax = j.find("axis")
        axis = [float(v) for v in (ax.get("xyz", "1 0 0") if ax is not None else "1 0 0").split()]
        mimic = j.find("mimic")
        by_child[j.find("child").get("link")] = dict(
            name=j.get("name"), type=j.get("type"), parent=j.find("parent").get("link"),
            xyz=xyz, rpy=rpy, axis=axis,
            mimic=None if mimic is None else (mimic.get("joint"), float(mimic.get("multiplier", 1)),
                                              float(mimic.get("offset", 0))))
    cache = {ROOT_LINK: np.eye(4)}

    def angle(j):
        if j["mimic"]:
            src, m, off = j["mimic"]
            return q.get(src, 0.0) * m + off
        return q.get(j["name"], 0.0)

    def T(link):
        if link in cache:
            return cache[link]
        j = by_child[link]
        o = np.eye(4)
        o[:3, :3] = _rpy(*j["rpy"])
        o[:3, 3] = j["xyz"]
        m = np.eye(4)
        if j["type"] in ("revolute", "continuous"):
            m[:3, :3] = _axis_angle(j["axis"], angle(j))
        elif j["type"] == "prismatic":
            m[:3, 3] = np.asarray(j["axis"], float) * angle(j)
        cache[link] = T(j["parent"]) @ o @ m
        return cache[link]

    return {link: T(link) for link in list(by_child) + [ROOT_LINK]}


def grounded_z(urdf: Path, repo_root: Path | None = None) -> float:
    """Height of base_link above the floor with all wheels touching it.

    Uses simulation/tinker_sim_isaac/base_spawn.py, loaded by file path because
    the package __init__ may import Isaac; falls back to GROUNDED_Z when absent.
    """
    path = Path(repo_root or _REPO_ROOT) / "simulation" / "tinker_sim_isaac" / "base_spawn.py"
    if not path.is_file():
        return GROUNDED_Z
    spec = importlib.util.spec_from_file_location("_map_check_base_spawn", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return float(mod.grounded_base_height(Path(urdf)))


def _sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def link_points(usd: Path, cache_dir: Path) -> dict[str, np.ndarray]:
    """Surface points (N x 3, link frame) of each link's drawn geometry.

    Every vertex plus points on each triangle so that no surface point is more
    than SAMPLE_SPACING from a sample. Cached in cache_dir by the USD's sha256.
    """
    usd, cache_dir = Path(usd), Path(cache_dir)
    cache = cache_dir / f"link_points-v{_POINTS_CACHE_VERSION}-{_sha256(usd)}.npz"
    if cache.is_file():
        with np.load(cache) as z:
            return {k: z[k] for k in z.files}
    points = _compute_link_points(usd)
    cache_dir.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=cache_dir, suffix=".tmp")
    try:
        with os.fdopen(fd, "wb") as f:
            np.savez(f, **points)
        os.replace(tmp, cache)
    except BaseException:
        os.unlink(tmp)
        raise
    return points


def _compute_link_points(usd: Path) -> dict[str, np.ndarray]:
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.Open(str(usd))
    root = stage.GetDefaultPrim()
    if not root:
        raise ValueError(f"{usd}: no default prim")
    xf = UsdGeom.XformCache()
    out = {}
    # Link prims are flat Xform children of the robot root.
    for link in root.GetChildren():
        if not link.IsA(UsdGeom.Xform):
            continue
        link_inv = np.array(xf.GetLocalToWorldTransform(link).GetInverse())
        chunks = []
        for prim in Usd.PrimRange(link, Usd.TraverseInstanceProxies()):
            if not prim.IsA(UsdGeom.Gprim):
                continue
            img = UsdGeom.Imageable(prim)
            if (img.ComputePurpose() not in _DRAWN_PURPOSES
                    or img.ComputeVisibility() == UsdGeom.Tokens.invisible):
                continue
            verts, tris = _triangles(prim)
            if not len(tris):
                continue
            # USD matrices act on row vectors: p_link = p_local . M_prim . inv(M_link)
            m = np.array(xf.GetLocalToWorldTransform(prim)) @ link_inv
            v = verts @ m[:3, :3] + m[3, :3]
            chunks.append(_sample_triangles(v, tris))
        if chunks:
            pts = np.concatenate(chunks)
            out[link.GetName()] = np.unique(np.round(pts, 6), axis=0)
    return out


def _triangles(prim):
    """(vertices V x 3, triangle indices T x 3) of a Mesh, Cube or Cylinder, local frame."""
    from pxr import UsdGeom

    if prim.IsA(UsdGeom.Mesh):
        mesh = UsdGeom.Mesh(prim)
        verts = np.array(mesh.GetPointsAttr().Get() or [], float).reshape(-1, 3)
        counts = np.array(mesh.GetFaceVertexCountsAttr().Get() or [], int)
        idx = np.array(mesh.GetFaceVertexIndicesAttr().Get() or [], int)
        if len(counts) and (counts == 3).all():
            return verts, idx.reshape(-1, 3)
        tris = []
        start = 0
        for c in counts:  # fan triangulation
            for k in range(1, c - 1):
                tris.append((idx[start], idx[start + k], idx[start + k + 1]))
            start += c
        return verts, np.array(tris, int).reshape(-1, 3)
    if prim.IsA(UsdGeom.Cube):
        h = UsdGeom.Cube(prim).GetSizeAttr().Get() / 2.0
        verts = np.array([[x, y, z] for x in (-h, h) for y in (-h, h) for z in (-h, h)], float)
        quads = [(0, 1, 3, 2), (4, 5, 7, 6), (0, 1, 5, 4), (2, 3, 7, 6), (0, 2, 6, 4), (1, 3, 7, 5)]
        tris = [(a, b, c) for a, b, c, d in quads] + [(a, c, d) for a, b, c, d in quads]
        return verts, np.array(tris, int)
    if prim.IsA(UsdGeom.Cylinder):
        cyl = UsdGeom.Cylinder(prim)
        r, h = cyl.GetRadiusAttr().Get(), cyl.GetHeightAttr().Get()
        n = max(24, math.ceil(2 * math.pi * r / _CYLINDER_CHORD))
        a = np.linspace(0.0, 2 * math.pi, n, endpoint=False)
        ring = np.stack([r * np.cos(a), r * np.sin(a)], 1)
        local = np.concatenate([np.c_[ring, np.full(n, -h / 2)], np.c_[ring, np.full(n, h / 2)],
                                [[0, 0, -h / 2], [0, 0, h / 2]]])
        i = np.arange(n)
        j = (i + 1) % n
        tris = np.concatenate([np.c_[i, j, n + j], np.c_[i, n + j, n + i],
                               np.c_[np.full(n, 2 * n), j, i], np.c_[np.full(n, 2 * n + 1), n + i, n + j]])
        # Cylinder axis is Z by default; permute coordinates for X or Y.
        perm = {"X": [2, 0, 1], "Y": [1, 2, 0], "Z": [0, 1, 2]}[cyl.GetAxisAttr().Get()]
        return local[:, perm], tris
    raise ValueError(f"{prim.GetPath()}: unsupported geometry type {prim.GetTypeName()}")


def _sample_triangles(verts: np.ndarray, tris: np.ndarray) -> np.ndarray:
    """Vertices plus a barycentric grid on each triangle with edge steps <= SAMPLE_SPACING."""
    a, b, c = verts[tris[:, 0]], verts[tris[:, 1]], verts[tris[:, 2]]
    longest = np.max([np.linalg.norm(b - a, axis=1), np.linalg.norm(c - b, axis=1),
                      np.linalg.norm(a - c, axis=1)], axis=0)
    steps = np.maximum(1, np.ceil(longest / SAMPLE_SPACING)).astype(int)
    out = [verts]
    for n in np.unique(steps):
        if n == 1:
            continue  # the vertices already cover it
        sel = steps == n
        ij = np.array([(i, j) for i in range(n + 1) for j in range(n + 1 - i)], float) / n
        u, v = ij[:, 0], ij[:, 1]
        w = 1.0 - u - v
        grid = (w[None, :, None] * a[sel][:, None, :] + u[None, :, None] * b[sel][:, None, :]
                + v[None, :, None] * c[sel][:, None, :])
        out.append(grid.reshape(-1, 3))
    return np.concatenate(out)
