"""CoACD convex decomposition of arena furniture meshes.

Parameters come from the ``coacd`` block of ``config/arena-import.json`` and are
recorded with the artifact; ``decompose`` returns pieces in a stable order so
the authored USD is reproducible.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, fields
from typing import Mapping

import numpy as np

# coacd's native libs break a later first import of pxr (segfault in
# pxr.Tf.PreparePythonModule), so pxr must be loaded before coacd is.
try:
    from pxr import Usd, UsdGeom, UsdPhysics  # noqa: F401
except ImportError:
    pass


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


def decompose(vertices: np.ndarray, faces: np.ndarray, params: CoacdParams) -> list[ConvexPiece]:
    """Decompose a triangle mesh into convex pieces sorted by (min z, min y, min x)."""
    import coacd

    coacd.set_log_level("error")
    mesh = coacd.Mesh(
        np.asarray(vertices, dtype=np.float64), np.asarray(faces, dtype=np.int64)
    )
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
    pieces = [
        ConvexPiece(np.asarray(v, dtype=np.float64), np.asarray(f, dtype=np.int64))
        for v, f in parts
    ]
    pieces.sort(key=lambda p: tuple(p.vertices.min(axis=0)[::-1]))
    return pieces
