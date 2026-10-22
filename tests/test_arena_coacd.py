"""CoACD decomposition module: config loading and the pinned decomposition call."""
from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from tinker_sim_deploy.arena_coacd import CoacdParams, decompose  # noqa: E402


def _config() -> dict:
    return json.loads((ROOT / "config" / "arena-import.json").read_text())["coacd"]


def _l_shape():
    """One watertight L (the union of a 1.0x0.2 and a 0.2x0.8 box), volume 0.072."""
    np = pytest.importorskip("numpy")
    trimesh = pytest.importorskip("trimesh")
    v2 = np.array([(-0.5, -0.1), (0.5, -0.1), (0.5, 0.9), (0.3, 0.9), (0.3, 0.1), (-0.5, 0.1)])
    f2 = np.array([(0, 1, 4), (0, 4, 5), (1, 2, 3), (1, 3, 4)])
    return trimesh.creation.extrude_triangulation(v2, f2, 0.2)


class CoacdParamsTests(unittest.TestCase):
    def test_from_config_reads_repo_config(self):
        params = CoacdParams.from_config(_config())
        self.assertEqual(params.max_ch_vertex, 64)
        self.assertEqual(params.seed, 0)
        self.assertEqual(params.plate_surfaces, ())

    def test_from_config_rejects_missing_key(self):
        cfg = _config()
        del cfg["threshold"]
        with self.assertRaises(ValueError):
            CoacdParams.from_config(cfg)

    def test_from_config_rejects_unknown_key(self):
        cfg = _config()
        cfg["bogus"] = 1
        with self.assertRaises(ValueError):
            CoacdParams.from_config(cfg)

    def test_from_config_ignores_comment_keys(self):
        cfg = _config()
        cfg["_other"] = "x"
        CoacdParams.from_config(cfg)


class DecomposeTests(unittest.TestCase):
    def test_l_shape_splits_into_convex_pieces(self):
        pytest.importorskip("coacd")
        trimesh = pytest.importorskip("trimesh")
        mesh = _l_shape()
        pieces = decompose(mesh.vertices, mesh.faces, CoacdParams.from_config(_config()))
        self.assertGreaterEqual(len(pieces), 2)
        total = 0.0
        for p in pieces:
            tm = trimesh.Trimesh(p.vertices, p.faces)
            self.assertTrue(tm.is_convex)
            self.assertLessEqual(len(p.vertices), 64)
            total += tm.volume
        import numpy as np

        allv = np.vstack([p.vertices for p in pieces])
        self.assertGreaterEqual(trimesh.proximity.signed_distance(mesh, allv).min(), -0.002)
        self.assertAlmostEqual(total / mesh.volume, 1.0, delta=0.05)

    def test_decompose_is_deterministic(self):
        pytest.importorskip("coacd")
        mesh = _l_shape()
        params = CoacdParams.from_config(_config())
        a = decompose(mesh.vertices, mesh.faces, params)
        b = decompose(mesh.vertices, mesh.faces, params)
        self.assertEqual(len(a), len(b))
        for x, y in zip(a, b):
            self.assertTrue(np_equal(x.vertices, y.vertices))
            self.assertTrue(np_equal(x.faces, y.faces))


def np_equal(a, b) -> bool:
    import numpy as np

    return np.array_equal(a, b)
