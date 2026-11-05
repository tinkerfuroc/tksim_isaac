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

from tinker_sim_deploy.arena_coacd import CoacdParams, decompose, single_piece_slab  # noqa: E402


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
        self.assertIs(params.decimate, True)
        self.assertEqual(params.seed, 0)
        self.assertEqual(params.plate_surfaces, ())
        self.assertEqual(params.threshold, 0.10)
        self.assertEqual(
            sorted(params.single_piece_tops),
            ["rcw26_laundry_desk", "rcw26_side_table", "rcw26_sink", "rcw26_tv_stand"],
        )
        self.assertEqual(params.single_piece_tops["rcw26_side_table"], (0.585, 0.602))
        self.assertEqual(params.as_record()["single_piece_tops"]["rcw26_sink"], [0.606, 0.639])
        self.assertIs(params.as_record()["decimate"], True)

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

    def test_from_config_rejects_wrongly_typed_values(self):
        for key, bad in (
            ("decimate", "false"),
            ("decimate", 0),
            ("pca", None),
            ("merge", "true"),
            ("seed", True),
            ("max_ch_vertex", 64.0),
            ("resolution", "2000"),
            ("threshold", "0.02"),
            ("threshold", False),
            ("preprocess_mode", 1),
            ("plate_surfaces", "rcw26_kitchen_table"),
            ("plate_surfaces", [1]),
            ("single_piece_tops", [["a", 0.1, 0.2]]),
            ("single_piece_tops", {"a": [0.1]}),
            ("single_piece_tops", {"a": [0.1, 0.2, 0.3]}),
            ("single_piece_tops", {"a": [0.2, 0.1]}),
            ("single_piece_tops", {"a": [0.1, 0.1]}),
            ("single_piece_tops", {"a": ["0.1", 0.2]}),
            ("single_piece_tops", {"a": [True, 0.2]}),
            ("single_piece_tops", {"a": [0.1, float("inf")]}),
            ("single_piece_tops", {1: [0.1, 0.2]}),
        ):
            cfg = _config()
            cfg[key] = bad
            with self.assertRaises(ValueError, msg=f"{key}={bad!r}"):
                CoacdParams.from_config(cfg)

    def test_from_config_accepts_an_integer_threshold_as_float(self):
        cfg = _config()
        cfg["threshold"] = 1
        params = CoacdParams.from_config(cfg)
        self.assertIsInstance(params.threshold, float)


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

    def test_decimate_caps_hull_vertices_on_a_curved_mesh(self):
        # A 96-section cylinder is one convex piece of 192 vertices unless
        # decimate enforces max_ch_vertex (PhysX's 64-vertex hull limit).
        _require_coacd()
        trimesh = pytest.importorskip("trimesh")
        mesh = trimesh.creation.cylinder(radius=0.2, height=0.3, sections=96)
        pieces = decompose(mesh.vertices, mesh.faces, CoacdParams.from_config(_config()))
        self.assertTrue(pieces)
        for p in pieces:
            self.assertLessEqual(len(p.vertices), 64)

    def test_pieces_over_the_vertex_cap_fail_closed(self):
        _require_coacd()
        trimesh = pytest.importorskip("trimesh")
        mesh = trimesh.creation.cylinder(radius=0.2, height=0.3, sections=96)
        cfg = _config()
        cfg["decimate"] = False
        with self.assertRaisesRegex(ValueError, "max_ch_vertex"):
            decompose(mesh.vertices, mesh.faces, CoacdParams.from_config(cfg))


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


def _xy_area(vertices) -> float:
    from scipy.spatial import ConvexHull

    return float(ConvexHull(vertices[:, :2]).volume)


def _table(slab_z=(0.585, 0.602)):
    """Watertight "table": a 1.0 x 0.6 top slab and four separate leg boxes below it."""
    np = pytest.importorskip("numpy")
    trimesh = pytest.importorskip("trimesh")
    z0, z1 = slab_z
    slab = trimesh.creation.box(
        extents=(1.0, 0.6, z1 - z0), transform=trimesh.transformations.translation_matrix((0, 0, (z0 + z1) / 2))
    )
    parts = [slab]
    for sx in (-0.45, 0.45):
        for sy in (-0.25, 0.25):
            parts.append(
                trimesh.creation.box(
                    extents=(0.05, 0.05, z0),
                    transform=trimesh.transformations.translation_matrix((sx, sy, z0 / 2)),
                )
            )
    table = trimesh.util.concatenate(parts)
    assert table.is_watertight
    return table


def _rounded_slab(z0=0.585, z1=0.602, radius=0.1, per_corner=14):
    """Convex slab 1.0 x 0.6 whose corners are arcs (4 * per_corner > 32 outline points)."""
    np = pytest.importorskip("numpy")
    trimesh = pytest.importorskip("trimesh")
    pts = []
    for cx, cy, a0 in ((0.4, 0.2, 0.0), (-0.4, 0.2, 90.0), (-0.4, -0.2, 180.0), (0.4, -0.2, 270.0)):
        for a in np.radians(np.linspace(a0, a0 + 90.0, per_corner)):
            pts.append((cx + radius * np.cos(a), cy + radius * np.sin(a)))
    pts = np.array(pts)
    return trimesh.convex.convex_hull(
        np.vstack([np.column_stack([pts, np.full(len(pts), z)]) for z in (z0, z1)])
    )


class SinglePieceTopTests(unittest.TestCase):
    BAND = (0.585, 0.602)

    def setUp(self):
        pytest.importorskip("numpy")
        pytest.importorskip("scipy")
        pytest.importorskip("trimesh")

    def test_slab_is_one_flat_prism_at_the_real_top(self):
        table = _table()
        slab = single_piece_slab(table.vertices, table.faces, self.BAND, 64)
        self.assertAlmostEqual(float(slab.vertices[:, 2].max()), 0.602, places=9)
        self.assertAlmostEqual(float(slab.vertices[:, 2].min()), 0.585, places=9)
        self.assertGreaterEqual(_xy_area(slab.vertices), 0.98 * 0.6)
        self.assertLessEqual(len(slab.vertices), 64)

    def test_slab_is_a_closed_convex_solid(self):
        import trimesh

        table = _table()
        slab = single_piece_slab(table.vertices, table.faces, self.BAND, 64)
        mesh = trimesh.Trimesh(slab.vertices, slab.faces)
        self.assertTrue(mesh.is_watertight)
        self.assertTrue(mesh.is_winding_consistent)
        self.assertTrue(mesh.is_convex)
        self.assertAlmostEqual(mesh.volume, 1.0 * 0.6 * 0.017, places=6)

    def test_rounded_top_is_reduced_to_32_outline_points_with_small_area_loss(self):
        top = _rounded_slab()
        self.assertGreater(len(set(map(tuple, top.vertices[:, :2].round(9)))), 32)
        slab = single_piece_slab(top.vertices, top.faces, self.BAND, 64)
        self.assertLessEqual(len(slab.vertices), 64)
        full = _xy_area(top.vertices)
        self.assertGreaterEqual(_xy_area(slab.vertices), 0.98 * full)
        self.assertAlmostEqual(float(slab.vertices[:, 2].max()), 0.602, places=9)

    def test_slab_is_deterministic(self):
        import numpy as np

        top = _rounded_slab()
        a = single_piece_slab(top.vertices, top.faces, self.BAND, 64)
        b = single_piece_slab(top.vertices, top.faces, self.BAND, 64)
        self.assertTrue(np.array_equal(a.vertices, b.vertices))
        self.assertTrue(np.array_equal(a.faces, b.faces))

    def test_band_cutting_through_a_slab_extrudes_between_the_band_planes(self):
        table = _table((0.55, 0.65))
        slab = single_piece_slab(table.vertices, table.faces, (0.6, 0.64), 64)
        self.assertAlmostEqual(float(slab.vertices[:, 2].min()), 0.6, places=9)
        self.assertAlmostEqual(float(slab.vertices[:, 2].max()), 0.64, places=9)

    def test_bad_bands_are_errors(self):
        table = _table()
        for band in ((0.7, 0.8), (-0.2, -0.1), (0.3, 0.2), (0.3, 0.3)):
            with self.assertRaises(ValueError, msg=str(band)):
                single_piece_slab(table.vertices, table.faces, band, 64)

    def test_band_over_a_gap_with_no_geometry_is_an_error(self):
        import trimesh

        low = _table()
        high = low.copy()
        high.apply_translation((0, 0, 1.0))
        two = trimesh.util.concatenate([low, high])
        with self.assertRaisesRegex(ValueError, "no mesh geometry"):
            single_piece_slab(two.vertices, two.faces, (0.7, 0.9), 64)

    def test_decompose_with_a_band_returns_one_top_piece_and_no_other_reaches_it(self):
        _require_coacd()
        table = _table()
        cfg = {**_config(), "mcts_iterations": 20, "mcts_nodes": 8, "resolution": 500}
        params = CoacdParams.from_config(cfg)
        pieces = decompose(table.vertices, table.faces, params, band=self.BAND)
        spanning = [p for p in pieces if p.vertices[:, 2].max() > 0.585 + 0.001]
        self.assertEqual(len(spanning), 1)
        top = spanning[0]
        self.assertAlmostEqual(float(top.vertices[:, 2].max()), 0.602, delta=0.001)
        self.assertGreaterEqual(_xy_area(top.vertices), 0.98 * 0.6)
        self.assertLessEqual(len(top.vertices), 64)
        self.assertGreaterEqual(len(pieces), 2)  # the legs are decomposed too
        again = decompose(table.vertices, table.faces, params, band=self.BAND)
        self.assertEqual(len(pieces), len(again))
        for x, y in zip(pieces, again):
            self.assertTrue(np_equal(x.vertices, y.vertices))
            self.assertTrue(np_equal(x.faces, y.faces))

    def test_decompose_all_applies_each_model_its_own_band(self):
        _require_coacd()
        from tinker_sim_deploy import arena_coacd

        table = _table()
        cfg = {**_config(), "mcts_iterations": 20, "mcts_nodes": 8, "resolution": 500,
               "single_piece_tops": {"with_band": [0.585, 0.602]}}
        params = CoacdParams.from_config(cfg)
        meshes = {"with_band": (table.vertices, table.faces), "plain": (table.vertices, table.faces)}
        results = arena_coacd.decompose_all(meshes, params, workers=2)
        def spanning(pieces):
            return [p for p in pieces if p.vertices[:, 2].max() > 0.586]
        self.assertEqual(len(spanning(results["with_band"])), 1)
        self.assertGreaterEqual(len(spanning(results["plain"])), 1)


def np_equal(a, b) -> bool:
    import numpy as np

    return np.array_equal(a, b)
