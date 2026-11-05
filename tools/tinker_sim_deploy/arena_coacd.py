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
from dataclasses import asdict, dataclass, fields
from typing import Mapping

import numpy as np

#: Upper bound on ``decompose_all``'s worker processes (the host is shared).
MAX_WORKERS = 8


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
    max_ch_vertex: int
    seed: int
    plate_surfaces: tuple[str, ...] = ()

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
        values = {k: config[k] for k in names}
        values["plate_surfaces"] = tuple(str(s) for s in values["plate_surfaces"])
        return cls(**values)

    def as_record(self) -> dict[str, object]:
        record = asdict(self)
        record["plate_surfaces"] = list(self.plate_surfaces)
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


def decompose(vertices: np.ndarray, faces: np.ndarray, params: CoacdParams) -> list[ConvexPiece]:
    """Decompose a triangle mesh into convex pieces sorted by (min z, min y, min x).

    Single-threaded, so the same mesh and parameters give the same pieces in
    any process.
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
        max_ch_vertex=params.max_ch_vertex,
        seed=params.seed,
    )
    if not parts:
        raise ValueError("CoACD returned no convex pieces")
    pieces = [
        ConvexPiece(np.asarray(v, dtype=np.float64), np.asarray(f, dtype=np.int64))
        for v, f in parts
    ]
    pieces.sort(key=lambda p: tuple(p.vertices.min(axis=0)[::-1]))
    return pieces


def _init_worker() -> None:
    _import_coacd(load_pxr_first=False)


def _decompose_task(name: str, vertices: np.ndarray, faces: np.ndarray, params: CoacdParams) -> list[ConvexPiece]:
    try:
        return decompose(vertices, faces, params)
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
