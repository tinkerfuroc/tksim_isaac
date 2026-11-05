"""CoACD convex decomposition of arena furniture meshes.

Parameters come from the ``coacd`` block of ``config/arena-import.json`` and are
recorded with the artifact; ``decompose`` returns pieces in a stable order so
the authored USD is reproducible.

Multi-threaded, CoACD (its bundled libgomp) gives the same mesh different
pieces run to run, so every decomposition runs single-threaded:
``OMP_NUM_THREADS=1`` is put in the environment before coacd is first imported
(libgomp reads it once, when it loads). ``decompose_all`` gets the wall time
back by decomposing many meshes at once in a spawn-context process pool, one
single-threaded mesh per worker; the calling process (the importer's Kit
process) never imports coacd.

Nothing here imports pxr at module level: the importer loads this module before
``SimulationApp`` starts, and pxr must not load before it.
"""
from __future__ import annotations

import multiprocessing
import os
import sys
from dataclasses import asdict, dataclass, field, fields
from typing import Mapping

import numpy as np

#: Upper bound on ``decompose_all``'s worker processes (the host is shared).
MAX_WORKERS = 8


def _is_int(value: object) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _is_band(value: object) -> bool:
    return (
        isinstance(value, (list, tuple))
        and len(value) == 2
        and all(_is_int(v) or isinstance(v, float) for v in value)
        and bool(np.isfinite(value).all())
        and value[0] < value[1]
    )


# CoacdParams field annotation -> value check (annotations are strings here).
_TYPE_CHECKS = {
    "bool": lambda value: isinstance(value, bool),
    "int": _is_int,
    "float": lambda value: _is_int(value) or isinstance(value, float),
    "str": lambda value: isinstance(value, str),
    "tuple[str, ...]": lambda value: (
        isinstance(value, (list, tuple)) and all(isinstance(item, str) for item in value)
    ),
    "dict[str, tuple[float, float]]": lambda value: (
        isinstance(value, Mapping)
        and all(isinstance(k, str) and _is_band(v) for k, v in value.items())
    ),
}


@dataclass(frozen=True)
class CoacdParams:
    threshold: float
    max_convex_hull: int
    preprocess_mode: str
    preprocess_resolution: int
    resolution: int
    mcts_nodes: int
    mcts_iterations: int
    mcts_max_depth: int
    pca: bool
    merge: bool
    decimate: bool
    max_ch_vertex: int
    seed: int
    plate_surfaces: tuple[str, ...] = ()
    #: model id -> (z_bottom, z_top) model-local metres: that model's top slab is
    #: one flat convex piece (see ``single_piece_slab``), CoACD does the rest.
    single_piece_tops: dict[str, tuple[float, float]] = field(default_factory=dict)

    @classmethod
    def from_config(cls, config: Mapping[str, object]) -> "CoacdParams":
        if not isinstance(config, Mapping):
            raise ValueError("arena-import config has no 'coacd' block")
        keys = {k for k in config if not str(k).startswith("_")}
        names = {f.name for f in fields(cls)}
        missing = sorted(names - keys)
        extra = sorted(keys - names)
        if missing:
            raise ValueError(f"coacd config missing keys: {missing}")
        if extra:
            raise ValueError(f"coacd config has unknown keys: {extra}")
        for f in fields(cls):
            if not _TYPE_CHECKS[f.type](config[f.name]):
                raise ValueError(f"coacd config {f.name!r} must be {f.type}, got {config[f.name]!r}")
        values = {k: config[k] for k in names}
        values["threshold"] = float(values["threshold"])
        values["plate_surfaces"] = tuple(values["plate_surfaces"])
        values["single_piece_tops"] = {
            k: (float(v[0]), float(v[1])) for k, v in sorted(values["single_piece_tops"].items())
        }
        return cls(**values)

    def as_record(self) -> dict[str, object]:
        record = asdict(self)
        record["plate_surfaces"] = list(self.plate_surfaces)
        record["single_piece_tops"] = {k: list(v) for k, v in sorted(self.single_piece_tops.items())}
        return dict(sorted(record.items()))


@dataclass(frozen=True)
class ConvexPiece:
    vertices: np.ndarray  # (N, 3) float64, model-local metres
    faces: np.ndarray  # (M, 3) int64 triangle indices


def _import_coacd(*, load_pxr_first: bool = True):
    """Import coacd single-threaded (see the module docstring).

    The first import sets ``OMP_NUM_THREADS=1`` and, unless ``load_pxr_first``
    is false (pool workers never use pxr), loads pxr before coacd: coacd's
    native libs break a later first import of pxr (segfault in
    pxr.Tf.PreparePythonModule). Fails closed if coacd is already loaded
    without ``OMP_NUM_THREADS=1``.
    """
    if "coacd" not in sys.modules:
        os.environ["OMP_NUM_THREADS"] = "1"
        if load_pxr_first:
            try:
                from pxr import Usd, UsdGeom, UsdPhysics  # noqa: F401
            except ImportError:
                pass
    elif os.environ.get("OMP_NUM_THREADS") != "1":
        raise RuntimeError(
            "coacd was imported before OMP_NUM_THREADS=1 was set; its decompositions "
            "would not be reproducible"
        )
    import coacd

    return coacd


def _band_geometry(verts: np.ndarray, tris: np.ndarray, band: tuple[float, float]) -> np.ndarray:
    """Every mesh point inside ``band``: its vertices there plus the points where
    mesh edges cross the two band planes."""
    z_bottom, z_top = band
    points = [verts[(verts[:, 2] >= z_bottom) & (verts[:, 2] <= z_top)]]
    edges = np.unique(np.sort(np.concatenate([tris[:, [0, 1]], tris[:, [1, 2]], tris[:, [2, 0]]]), axis=1), axis=0)
    a, b = verts[edges[:, 0]], verts[edges[:, 1]]
    for z in band:
        crossing = (a[:, 2] - z) * (b[:, 2] - z) < 0
        t = (z - a[crossing, 2]) / (b[crossing, 2] - a[crossing, 2])
        points.append(a[crossing] + t[:, None] * (b[crossing] - a[crossing]))
    return np.concatenate(points)


def _reduce_outline(outline: np.ndarray, limit: int) -> np.ndarray:
    """Drop vertices of a convex counter-clockwise polygon, least area lost first, down to ``limit``."""
    outline = list(map(tuple, outline))
    while len(outline) > limit:
        n = len(outline)
        loss = []
        for i in range(n):
            (x0, y0), (x1, y1), (x2, y2) = outline[i - 1], outline[i], outline[(i + 1) % n]
            loss.append(abs((x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0)) / 2.0)
        del outline[int(np.argmin(loss))]
    return np.asarray(outline, dtype=np.float64)


def single_piece_slab(
    vertices: np.ndarray, faces: np.ndarray, band: tuple[float, float], max_vertices: int
) -> ConvexPiece:
    """One flat convex prism covering a mesh's geometry between two z planes.

    The prism's outline is the 2-D convex hull of the xy of every mesh point in
    ``band`` (vertices in it and edge crossings of its two planes), reduced to at
    most ``max_vertices // 2`` points by repeatedly dropping the vertex whose
    removal loses the least area; it is extruded from the lowest to the highest
    z of that geometry, so the top face is flat at the slab's real top. Pure
    numpy/scipy and deterministic. Raises ``ValueError`` for a band outside the
    mesh's z range or with no mesh geometry in it.
    """
    from scipy.spatial import ConvexHull, QhullError

    verts = np.asarray(vertices, dtype=np.float64)
    tris = np.asarray(faces, dtype=np.int64)
    z_bottom, z_top = band
    if not z_bottom < z_top:
        raise ValueError(f"single-piece band {band} needs z_bottom < z_top")
    if z_top <= verts[:, 2].min() or z_bottom >= verts[:, 2].max():
        raise ValueError(
            f"single-piece band {band} is outside the mesh's z range "
            f"[{verts[:, 2].min():.4f}, {verts[:, 2].max():.4f}]"
        )
    geometry = _band_geometry(verts, tris, (z_bottom, z_top))
    if len(geometry) == 0:
        raise ValueError(f"single-piece band {band} contains no mesh geometry")
    z_low, z_high = float(geometry[:, 2].min()), float(geometry[:, 2].max())
    if not z_low < z_high:
        raise ValueError(f"single-piece band {band} contains no mesh geometry with height")
    try:
        hull = ConvexHull(geometry[:, :2])
    except QhullError:
        raise ValueError(f"single-piece band {band}: mesh geometry in it has no area") from None
    outline = _reduce_outline(geometry[hull.vertices, :2], max(3, max_vertices // 2))
    n = len(outline)
    ring = np.arange(n)
    nxt = (ring + 1) % n
    prism = np.concatenate(
        [
            np.column_stack([outline, np.full(n, z_low)]),
            np.column_stack([outline, np.full(n, z_high)]),
        ]
    )
    # hull.vertices is counter-clockwise: the bottom fan must be reversed.
    tri = [(0, int(i + 1), int(i)) for i in range(1, n - 1)] + [
        (n, n + int(i), n + int(i + 1)) for i in range(1, n - 1)
    ]
    for i, j in zip(ring, nxt):
        tri.append((int(i), int(j), n + int(j)))
        tri.append((int(i), n + int(j), n + int(i)))
    return ConvexPiece(prism, np.asarray(tri, dtype=np.int64))


def _split_around_band(
    verts: np.ndarray, tris: np.ndarray, band: tuple[float, float]
) -> list[tuple[np.ndarray, np.ndarray]]:
    """The mesh parts below and above ``band`` (those with faces), cut at its planes.

    Cut open, not capped (capping needs shapely, which the importer's Python
    does not have); CoACD's preprocess remeshes the non-manifold result.
    """
    from trimesh.intersections import slice_faces_plane

    parts = []
    for normal, z in (((0.0, 0.0, -1.0), band[0]), ((0.0, 0.0, 1.0), band[1])):
        part_verts, part_tris, _ = slice_faces_plane(verts, tris, np.array(normal), np.array((0.0, 0.0, z)))
        if len(part_tris) >= 4:
            parts.append((np.asarray(part_verts, dtype=np.float64), np.asarray(part_tris, dtype=np.int64)))
    return parts


def decompose(
    vertices: np.ndarray,
    faces: np.ndarray,
    params: CoacdParams,
    band: tuple[float, float] | None = None,
) -> list[ConvexPiece]:
    """Decompose a triangle mesh into convex pieces sorted by (min z, min y, min x).

    Single-threaded, so the same mesh and parameters give the same pieces in
    any process. Fails closed if a piece has more than ``max_ch_vertex``
    vertices (``decimate`` is what enforces that cap; PhysX would otherwise
    re-cook the hull into a different shape).

    With a ``band`` (z_bottom, z_top), the geometry in it becomes one flat
    ``single_piece_slab`` and CoACD decomposes only the mesh below and above it.
    """
    verts = np.asarray(vertices, dtype=np.float64)
    tris = np.asarray(faces, dtype=np.int64)
    if verts.ndim != 2 or verts.shape[1] != 3 or len(verts) < 4:
        raise ValueError(f"decompose needs (N, 3) vertices with N >= 4, got {verts.shape}")
    if tris.ndim != 2 or tris.shape[1] != 3 or len(tris) < 4:
        raise ValueError(f"decompose needs (M, 3) faces with M >= 4, got {tris.shape}")
    if not np.isfinite(verts).all():
        raise ValueError("decompose vertices contain non-finite values")
    if tris.min() < 0 or tris.max() >= len(verts):
        raise ValueError("decompose face indices out of range")

    if band is None:
        return _sorted(_coacd_pieces(verts, tris, params), params)
    slab = single_piece_slab(verts, tris, band, params.max_ch_vertex)
    pieces = [slab]
    for part_verts, part_tris in _split_around_band(verts, tris, band):
        pieces.extend(_coacd_pieces(part_verts, part_tris, params))
    return _sorted(pieces, params)


def _coacd_pieces(verts: np.ndarray, tris: np.ndarray, params: CoacdParams) -> list[ConvexPiece]:
    coacd = _import_coacd()
    coacd.set_log_level("error")
    mesh = coacd.Mesh(verts, tris)
    parts = coacd.run_coacd(
        mesh,
        threshold=params.threshold,
        max_convex_hull=params.max_convex_hull,
        preprocess_mode=params.preprocess_mode,
        preprocess_resolution=params.preprocess_resolution,
        resolution=params.resolution,
        mcts_nodes=params.mcts_nodes,
        mcts_iterations=params.mcts_iterations,
        mcts_max_depth=params.mcts_max_depth,
        pca=params.pca,
        merge=params.merge,
        decimate=params.decimate,
        max_ch_vertex=params.max_ch_vertex,
        seed=params.seed,
    )
    if not parts:
        raise ValueError("CoACD returned no convex pieces")
    return [
        ConvexPiece(np.asarray(v, dtype=np.float64), np.asarray(f, dtype=np.int64))
        for v, f in parts
    ]


def _sorted(pieces: list[ConvexPiece], params: CoacdParams) -> list[ConvexPiece]:
    oversized = [len(p.vertices) for p in pieces if len(p.vertices) > params.max_ch_vertex]
    if oversized:
        raise ValueError(
            f"CoACD returned {len(oversized)} pieces over max_ch_vertex={params.max_ch_vertex} "
            f"vertices (largest {max(oversized)}; decimate={params.decimate})"
        )
    return sorted(pieces, key=lambda p: tuple(p.vertices.min(axis=0)[::-1]))


def _init_worker() -> None:
    _import_coacd(load_pxr_first=False)


def _decompose_task(name: str, vertices: np.ndarray, faces: np.ndarray, params: CoacdParams) -> list[ConvexPiece]:
    try:
        return decompose(vertices, faces, params, params.single_piece_tops.get(name))
    except ValueError as exc:
        raise ValueError(f"{name}: {exc}") from None


def decompose_all(
    meshes: Mapping[str, tuple[np.ndarray, np.ndarray]],
    params: CoacdParams,
    *,
    workers: int = MAX_WORKERS,
) -> dict[str, list[ConvexPiece]]:
    """``decompose`` every ``name -> (vertices, faces)`` mesh in a process pool.

    A spawn-context pool of at most ``workers`` (capped at ``MAX_WORKERS``)
    processes, each with ``OMP_NUM_THREADS=1`` set before it imports coacd;
    biggest meshes go first. Each result equals a ``decompose`` call in a
    fresh process. Returns ``{name: pieces}`` sorted by name.
    """
    names = sorted(meshes, key=lambda name: (-len(meshes[name][1]), name))
    if not names:
        return {}
    count = max(1, min(workers, MAX_WORKERS, len(names)))
    tasks = [(name, meshes[name][0], meshes[name][1], params) for name in names]
    with multiprocessing.get_context("spawn").Pool(count, initializer=_init_worker) as pool:
        results = pool.starmap(_decompose_task, tasks, chunksize=1)
    return dict(sorted(zip(names, results), key=lambda item: item[0]))
