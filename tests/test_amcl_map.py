"""AMCL gets a surface-only copy of the arena map.

The arena map is rasterized from collider footprints, so every obstacle is a
SOLID block of occupied cells. AMCL's likelihood field only penalizes a hit by
its distance to the nearest occupied cell, so a hit pushed deep into a solid
block costs nothing -- with the 165 deg front-only scan the estimate slid
forward into walls/furniture (live 2026-10-10: likelihood optimum +0.07 m
ahead of ground truth; surface-only map: +0.02 m). A SLAM-built map, which
is what AMCL sees on the real robot, records only obstacle surfaces.
"""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/tinker_sim_bridge"))

from tinker_sim_bridge.nav_params_overlay import prior_map_costmap_overlay  # noqa: E402
from tinker_sim_bridge.amcl_map import (  # noqa: E402
    OCCUPIED,
    UNKNOWN,
    surface_only_pgm,
    write_amcl_map,
)

FREE = 254


def _pgm(rows: list[list[int]]) -> bytes:
    h, w = len(rows), len(rows[0])
    return b"P5\n%d %d\n255\n" % (w, h) + bytes(v for row in rows for v in row)


def _cells(pgm: bytes) -> list[list[int]]:
    parts = pgm.split(b"\n", 3)
    w, h = map(int, parts[1].split())
    data = parts[3]
    return [list(data[r * w:(r + 1) * w]) for r in range(h)]


class SurfaceOnlyTest(unittest.TestCase):
    def test_solid_block_keeps_only_its_outline(self):
        o, f = OCCUPIED, FREE
        rows = [[f] * 5] + [[f, o, o, o, f] for _ in range(3)] + [[f] * 5]
        out = _cells(surface_only_pgm(_pgm(rows)))
        self.assertEqual(out[2], [f, o, UNKNOWN, o, f])
        self.assertEqual(out[1], [f, o, o, o, f])
        self.assertEqual(out[0], [f] * 5)

    def test_interior_value_reads_as_unknown_to_map_server(self):
        # map_server trinary (negate 0): p = (255 - v) / 255; free if
        # p < free_thresh (0.25 here), occupied if p > occupied_thresh (0.65),
        # unknown in between. ROS map_saver's 205 is p = 0.196 -- FREE under
        # these thresholds, which would let AMCL's random-particle recovery
        # sample poses inside furniture.
        p = (255 - UNKNOWN) / 255.0
        self.assertGreater(p, 0.25)
        self.assertLess(p, 0.65)

    def test_thin_walls_and_free_space_are_unchanged(self):
        o, f = OCCUPIED, FREE
        rows = [[f, o, f], [f, o, f], [f, o, f]]
        self.assertEqual(_cells(surface_only_pgm(_pgm(rows))), rows)

    def test_existing_unknown_cells_stay_unknown(self):
        o, f, u = OCCUPIED, FREE, UNKNOWN
        rows = [[u, u, u], [u, o, f], [u, u, u]]
        self.assertEqual(_cells(surface_only_pgm(_pgm(rows)))[1], [u, o, f])

    def test_write_amcl_map_keeps_the_geometry_metadata(self):
        with tempfile.TemporaryDirectory() as tmp:
            src = Path(tmp) / "src"
            src.mkdir()
            o, f = OCCUPIED, FREE
            (src / "map.pgm").write_bytes(_pgm([[o, o, o], [o, o, o], [o, o, o]]))
            (src / "map.yaml").write_text(
                "image: map.pgm\nmode: trinary\nresolution: 0.05\n"
                "origin: [-5.05, -6.0, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n",
                encoding="utf-8",
            )
            out_yaml = write_amcl_map(src / "map.yaml", Path(tmp) / "out")
            import yaml

            meta = yaml.safe_load(out_yaml.read_text(encoding="utf-8"))
            self.assertEqual(meta["resolution"], 0.05)
            self.assertEqual(meta["origin"], [-5.05, -6.0, 0])
            self.assertEqual(meta["mode"], "trinary")
            image = Path(meta["image"])
            self.assertTrue(image.is_absolute() and image.is_file())
            self.assertEqual(_cells(image.read_bytes())[1], [o, UNKNOWN, o])
            # Source left alone.
            self.assertEqual(_cells((src / "map.pgm").read_bytes())[1], [o, o, o])


LAUNCH_DIR = ROOT / "ros2_ws/src/tinker_sim_bridge/launch"


def _node_calls(path: Path) -> list[dict]:
    import ast

    calls = []
    for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"))):
        if isinstance(node, ast.Call) and getattr(node.func, "id", "") == "Node":
            calls.append({kw.arg: kw.value for kw in node.keywords})
    return calls


class AmclMapWiringTest(unittest.TestCase):
    def test_overlay_points_amcl_at_its_own_map_topic(self):
        result = prior_map_costmap_overlay({
            "global_costmap": {"global_costmap": {"ros__parameters": {}}},
            "amcl": {"ros__parameters": {"scan_topic": "scan"}},
        })
        self.assertEqual(result["amcl"]["ros__parameters"]["map_topic"], "amcl_map")

    def test_sim_launches_serve_the_surface_map_to_amcl(self):
        import ast

        for name in ("gpsr", "navigation"):
            with self.subTest(launch=name):
                path = LAUNCH_DIR / f"{name}.launch.py"
                source = path.read_text(encoding="utf-8")
                self.assertIn("write_amcl_map(", source)
                servers = [
                    c for c in _node_calls(path)
                    if isinstance(c.get("executable"), ast.Constant)
                    and c["executable"].value == "map_server"
                    and isinstance(c.get("name"), ast.Constant)
                    and c["name"].value == "amcl_map_server"
                ]
                self.assertEqual(len(servers), 1)
                self.assertIn('"topic_name": "amcl_map"', source)
                managers = [
                    c for c in _node_calls(path)
                    if isinstance(c.get("name"), ast.Constant)
                    and c["name"].value == "lifecycle_manager_amcl_map"
                ]
                self.assertEqual(len(managers), 1)
                self.assertIn('"node_names": ["amcl_map_server"]', source)


if __name__ == "__main__":
    unittest.main()
