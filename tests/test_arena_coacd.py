"""CoACD decomposition module: config loading and the pinned decomposition call."""
from __future__ import annotations

import importlib.util
import json
import os
import subprocess
import sys
import unittest
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from tinker_sim_deploy.arena_coacd import CoacdParams, decompose  # noqa: E402


def _config() -> dict:
    return json.loads((ROOT / "config" / "arena-import.json").read_text())["coacd"]


def _require_coacd() -> None:
    """Skip without coacd -- but never import it here: ``decompose`` must be
    the first to load it (single-threaded, after pxr; see arena_coacd)."""
    if importlib.util.find_spec("coacd") is None:
        pytest.skip("coacd not installed")


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
        _require_coacd()
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
        _require_coacd()
        mesh = _l_shape()
        params = CoacdParams.from_config(_config())
        a = decompose(mesh.vertices, mesh.faces, params)
        b = decompose(mesh.vertices, mesh.faces, params)
        self.assertEqual(len(a), len(b))
        for x, y in zip(a, b):
            self.assertTrue(np_equal(x.vertices, y.vertices))
            self.assertTrue(np_equal(x.faces, y.faces))



class DecomposeFailClosedTests(unittest.TestCase):
    def test_rejects_malformed_input(self):
        import numpy as np

        params = CoacdParams.from_config(_config())
        v = np.array([(0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)], dtype=float)
        f = np.array([(0, 2, 1), (0, 1, 3), (1, 2, 3), (0, 3, 2)])
        for bad_v, bad_f in (
            (np.zeros((0, 3)), np.zeros((0, 3), dtype=int)),
            (v[:3], f),
            (v[:, :2], f),
            (v, f[:3]),
            (v, f[:, :2]),
            (v, f + 10),
            (v, f - 1),
        ):
            with self.assertRaises(ValueError):
                decompose(bad_v, bad_f, params)

    def test_zero_pieces_raises(self):
        _require_coacd()
        from tinker_sim_deploy import arena_coacd

        coacd = arena_coacd._import_coacd()
        mesh = _l_shape()
        with pytest.MonkeyPatch.context() as mp:
            mp.setattr(coacd, "run_coacd", lambda *a, **k: [])
            with self.assertRaises(ValueError):
                decompose(mesh.vertices, mesh.faces, CoacdParams.from_config(_config()))


# Run in fresh interpreters: argv = tools dir, coacd config JSON, "direct"|"pool".
_DECOMPOSE_SCRIPT = r"""
import hashlib, json, sys
sys.path.insert(0, sys.argv[1])
import numpy as np
import trimesh
from tinker_sim_deploy import arena_coacd

def digest(pieces):
    h = hashlib.sha256()
    for p in pieces:
        h.update(p.vertices.tobytes())
        h.update(p.faces.tobytes())
    return [len(pieces), h.hexdigest()]

params = arena_coacd.CoacdParams.from_config(json.loads(sys.argv[2]))
cup = trimesh.creation.revolve(
    [(0, 0), (0.1, 0), (0.1, 0.2), (0.09, 0.2), (0.09, 0.01), (0, 0.01)], sections=16)
if sys.argv[3] == "direct":
    out = {"cup": digest(arena_coacd.decompose(cup.vertices, cup.faces, params))}
else:
    v2 = np.array([(-0.5, -0.1), (0.5, -0.1), (0.5, 0.9), (0.3, 0.9), (0.3, 0.1), (-0.5, 0.1)])
    f2 = np.array([(0, 1, 4), (0, 4, 5), (1, 2, 3), (1, 3, 4)])
    ell = trimesh.creation.extrude_triangulation(v2, f2, 0.2)
    results = arena_coacd.decompose_all(
        {"cup": (cup.vertices, cup.faces), "ell": (ell.vertices, ell.faces)}, params, workers=2)
    out = {name: digest(pieces) for name, pieces in results.items()}
    out["parent_loaded"] = sorted(m for m in ("coacd", "pxr") if m in sys.modules)
print(json.dumps(out))
"""


class DecomposeDeterminismTest(unittest.TestCase):
    """Real CoACD in fresh processes. Multi-threaded, CoACD's bundled libgomp
    gives this cup different pieces run to run (four default-thread runs gave
    four different results, 33-36 pieces); single-threaded it is reproducible.
    Cheaper MCTS settings than the repo config keep each run to ~10 s."""

    @classmethod
    def setUpClass(cls):
        _require_coacd()
        pytest.importorskip("trimesh")
        cfg = {**_config(), "mcts_iterations": 20, "mcts_nodes": 8, "resolution": 500}
        # The code under test must force single-threading itself.
        env = {k: v for k, v in os.environ.items() if k != "OMP_NUM_THREADS"}
        procs = [
            subprocess.Popen(
                [sys.executable, "-c", _DECOMPOSE_SCRIPT, str(ROOT / "tools"), json.dumps(cfg), mode],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env=env, cwd=str(ROOT),
            )
            for mode in ("direct", "direct", "pool")
        ]
        cls.results = []
        for proc in procs:
            try:
                out, err = proc.communicate(timeout=240)
            except subprocess.TimeoutExpired:
                proc.kill()
                out, err = proc.communicate()
            cls.results.append((proc.returncode, out, err))

    def _output(self, index: int) -> dict:
        returncode, out, err = self.results[index]
        self.assertEqual(returncode, 0, err[-2000:])
        return json.loads(out.strip().splitlines()[-1])

    def test_fresh_processes_give_identical_pieces(self):
        self.assertEqual(self._output(0), self._output(1))

    def test_pool_gives_the_same_pieces_as_a_direct_decomposition(self):
        pool = self._output(2)
        self.assertEqual(pool["cup"], self._output(0)["cup"])
        self.assertGreaterEqual(pool["ell"][0], 2)
        # The pool's parent process loads neither coacd nor pxr.
        self.assertEqual(pool["parent_loaded"], [])

    def test_coacd_loaded_multi_threaded_fails_closed(self):
        script = (
            "import sys; sys.path.insert(0, sys.argv[1]); import coacd; "
            "from tinker_sim_deploy import arena_coacd; arena_coacd._import_coacd()"
        )
        env = {k: v for k, v in os.environ.items() if k != "OMP_NUM_THREADS"}
        result = subprocess.run(
            [sys.executable, "-c", script, str(ROOT / "tools")],
            capture_output=True, text=True, env=env, timeout=120, check=False,
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("OMP_NUM_THREADS", result.stderr)


def np_equal(a, b) -> bool:
    import numpy as np

    return np.array_equal(a, b)
