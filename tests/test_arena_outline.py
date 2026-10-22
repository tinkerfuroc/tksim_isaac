from __future__ import annotations

import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "simulation"))

from tinker_sim_deploy import arena_world

BAND = (0.2725, 2.2725)


def _deps():
    np = pytest.importorskip("numpy")
    trimesh = pytest.importorskip("trimesh")
    pytest.importorskip("scipy")
    from tinker_sim_deploy import arena_outline
    from tinker_sim_deploy.arena_coacd import ConvexPiece
    return np, trimesh, arena_outline, ConvexPiece


def _box_piece(size, center=(0.0, 0.0, 0.0)):
    np, trimesh, _, ConvexPiece = _deps()
    mesh = trimesh.creation.box(extents=size)
    mesh.apply_translation(center)
    return ConvexPiece(vertices=np.asarray(mesh.vertices, dtype=np.float64),
                       faces=np.asarray(mesh.faces, dtype=np.int64))


def _prism_piece(radius, height, sections=32):
    np, trimesh, _, ConvexPiece = _deps()
    mesh = trimesh.creation.cylinder(radius=radius, height=height, sections=sections)
    return ConvexPiece(vertices=np.asarray(mesh.vertices, dtype=np.float64),
                       faces=np.asarray(mesh.faces, dtype=np.int64))


def _item(pos=(0.0, 0.0, 0.0), yaw=0.0, model="m"):
    return arena_world.FurniturePlacement(model_id=model, position=pos, yaw=yaw, static=True)


def _check(piece, box, item=None):
    _, _, outline, _ = _deps()
    item = item or _item()
    layout = arena_world.ArenaLayout(walls=(), furniture=(item,))
    return outline.check_arena(layout, {"m": (box,)}, {"m": [piece]}, BAND)[0]


# A 0.72 m tall box centred at z=0.36 spans the band low edge (0.2725).
BOX = arena_world.BoxCollider(size=(0.8, 0.6, 0.72), center=(0.0, 0.0, 0.36), yaw=0.0)


class OutlineTest(unittest.TestCase):
    def test_identical_box_passes(self):
        r = _check(_box_piece((0.8, 0.6, 0.72), (0, 0, 0.36)), BOX)
        self.assertLess(r.bigger_mm, 1.5)
        self.assertLess(r.smaller_mm, 1.5)
        self.assertTrue(r.passed)

    def test_hull_10mm_bigger_fails(self):
        r = _check(_box_piece((0.81, 0.6, 0.72), (0.005, 0, 0.36)), BOX)
        self.assertTrue(9 <= r.bigger_mm <= 11, r.bigger_mm)
        self.assertFalse(r.passed)

    def test_hull_4mm_bigger_passes(self):
        r = _check(_box_piece((0.804, 0.6, 0.72), (0.002, 0, 0.36)), BOX)
        self.assertTrue(r.passed, (r.bigger_mm, r.smaller_mm))

    def test_round_table_inscribed_in_box_fails_smaller(self):
        piece = _prism_piece(0.4, 0.72)
        piece = type(piece)(vertices=piece.vertices + [0, 0, 0.36], faces=piece.faces)
        box = arena_world.BoxCollider(size=(0.8, 0.8, 0.72), center=(0, 0, 0.36), yaw=0.0)
        r = _check(piece, box)
        self.assertGreater(r.smaller_mm, 150)
        self.assertFalse(r.passed)

    def test_piece_outside_band_contributes_nothing(self):
        _, _, outline, _ = _deps()
        plinth = _box_piece((0.8, 0.6, 0.1), (0, 0, 0.05))
        self.assertEqual(outline.band_polygons([plinth], _item(), BAND), [])

    def test_yaw_and_position_applied(self):
        yaw = math.pi / 2
        item = _item((2.0, -3.0, 0.0), yaw)
        r = _check(_box_piece((0.8, 0.6, 0.72), (0, 0, 0.36)), BOX, item)
        self.assertTrue(r.passed, (r.bigger_mm, r.smaller_mm))
        self.assertLess(r.bigger_mm, 1.5)

    def test_duplicate_instances_reported_separately(self):
        _, _, outline, _ = _deps()
        layout = arena_world.ArenaLayout(
            walls=(), furniture=(_item((0, 0, 0)), _item((3, 0, 0))))
        res = outline.check_arena(
            layout, {"m": (BOX,)}, {"m": [_box_piece((0.8, 0.6, 0.72), (0, 0, 0.36))]}, BAND)
        self.assertEqual([r.instance for r in res], ["m", "m_02"])

    def test_report_writes_summary_and_pngs(self):
        _, _, outline, _ = _deps()
        layout = arena_world.ArenaLayout(
            walls=(), furniture=(_item((0, 0, 0)), _item((3, 0, 0))))
        res = outline.check_arena(
            layout, {"m": (BOX,)}, {"m": [_box_piece((0.81, 0.6, 0.72), (0.005, 0, 0.36))]}, BAND)
        with tempfile.TemporaryDirectory() as tmp:
            summary = outline.write_outline_report(res, Path(tmp))
            data = json.loads(summary.read_text())
            self.assertEqual(data["tolerance_mm"], 5.0)
            self.assertFalse(data["passed"])
            self.assertEqual(len(data["instances"]), 2)
            self.assertEqual(sorted(p.name for p in Path(tmp).glob("*.png")), ["m.png", "m_02.png"])


if __name__ == "__main__":
    unittest.main()
