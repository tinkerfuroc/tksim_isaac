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


def _check_pieces(pieces, box, item):
    _, _, outline, _ = _deps()
    layout = arena_world.ArenaLayout(walls=(), furniture=(item,))
    return outline.check_arena(layout, {"m": (box,)}, {"m": pieces}, BAND)[0]


# A 0.72 m tall box centred at z=0.36 spans the band low edge (0.2725).
BOX = arena_world.BoxCollider(size=(0.8, 0.6, 0.72), center=(0.0, 0.0, 0.36), yaw=0.0)


class OutlineTest(unittest.TestCase):
    def test_identical_box_passes(self):
        r = _check(_box_piece((0.8, 0.6, 0.72), (0, 0, 0.36)), BOX)
        self.assertLess(r.bigger_mm, 1.5)
        self.assertLess(r.smaller_mm, 1.5)
        self.assertTrue(r.passed)

    def test_hull_10mm_bigger_is_reported_but_passes(self):
        # Hull grows 10 mm on the +x side: bigger_mm is information now; the
        # bbox centre moved 5 mm, well inside the centre tolerance.
        r = _check(_box_piece((0.81, 0.6, 0.72), (0.005, 0, 0.36)), BOX)
        self.assertTrue(9 <= r.bigger_mm <= 11, r.bigger_mm)
        self.assertTrue(r.passed)

    def test_center_shift_gate(self):
        r = _check(_box_piece((0.8, 0.6, 0.72), (0.030, 0, 0.36)), BOX)
        self.assertAlmostEqual(r.center_shift_mm, 30.0, delta=1.0)
        self.assertFalse(r.passed)
        r = _check(_box_piece((0.8, 0.6, 0.72), (0.010, 0, 0.36)), BOX)
        self.assertAlmostEqual(r.center_shift_mm, 10.0, delta=1.0)
        self.assertAlmostEqual(r.bigger_mm, 10.0, delta=1.5)
        self.assertTrue(r.passed)

    def test_center_shift_is_measured_in_the_furniture_frame(self):
        np, trimesh, _, ConvexPiece = _deps()
        # L-shape: two boxes whose union bbox is 0.8 x 0.6, same as the map box.
        a = _box_piece((0.8, 0.2, 0.72), (0.0, -0.2, 0.36))
        b = _box_piece((0.2, 0.6, 0.72), (-0.3, 0.0, 0.36))
        for yaw in (0.0, math.pi / 3):
            r = _check_pieces([a, b], BOX, _item((1.0, 2.0, 0.0), yaw))
            self.assertLess(r.center_shift_mm, 1.0, yaw)
            self.assertTrue(r.passed, yaw)

    def test_hull_4mm_bigger_passes(self):
        r = _check(_box_piece((0.804, 0.6, 0.72), (0.002, 0, 0.36)), BOX)
        self.assertTrue(r.passed, (r.bigger_mm, r.smaller_mm))

    def test_round_table_inscribed_in_box_passes_centred(self):
        piece = _prism_piece(0.4, 0.72)
        piece = type(piece)(vertices=piece.vertices + [0, 0, 0.36], faces=piece.faces)
        box = arena_world.BoxCollider(size=(0.8, 0.8, 0.72), center=(0, 0, 0.36), yaw=0.0)
        r = _check(piece, box)
        self.assertGreater(r.smaller_mm, 150)
        self.assertLess(r.center_shift_mm, 1.0)
        self.assertTrue(r.passed)

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
            layout, {"m": (BOX,)}, {"m": [_box_piece((0.8, 0.6, 0.72), (0.03, 0, 0.36))]}, BAND)
        with tempfile.TemporaryDirectory() as tmp:
            summary = outline.write_outline_report(res, Path(tmp))
            data = json.loads(summary.read_text())
            self.assertEqual(data["tolerance_mm"], 5.0)
            self.assertEqual(data["center_tolerance_mm"], 25.0)
            self.assertFalse(data["passed"])
            self.assertAlmostEqual(data["instances"][0]["center_shift_mm"], 30.0, delta=1.0)
            self.assertEqual(len(data["instances"]), 2)
            self.assertEqual(sorted(p.name for p in Path(tmp).glob("*.png")), ["m.png", "m_02.png"])

    def test_thin_map_box_without_hull_fails(self):
        _, _, outline, _ = _deps()
        thin = arena_world.BoxCollider(size=(0.8, 0.006, 0.72), center=(0, 0, 0.36), yaw=0.0)
        layout = arena_world.ArenaLayout(walls=(), furniture=(_item(),))
        r = outline.check_arena(layout, {"m": (thin,)}, {"m": []}, BAND)[0]
        self.assertEqual(r.smaller_mm, float("inf"))
        self.assertFalse(r.passed)

    def test_missing_hull_fails_gate(self):
        _, _, outline, _ = _deps()
        layout = arena_world.ArenaLayout(walls=(), furniture=(_item(),))
        r = outline.check_arena(layout, {"m": (BOX,)}, {"m": []}, BAND)[0]
        self.assertEqual(r.center_shift_mm, float("inf"))
        self.assertFalse(r.passed)

    def test_both_shapes_empty_passes(self):
        _, _, outline, _ = _deps()
        layout = arena_world.ArenaLayout(walls=(), furniture=(_item(),))
        r = outline.check_arena(layout, {}, {"m": []}, BAND)[0]
        self.assertTrue(r.passed)

    def test_instance_outlines_keyed_like_check_arena(self):
        _, _, outline, _ = _deps()
        layout = arena_world.ArenaLayout(
            walls=(), furniture=(_item((0, 0, 0)), _item((3, 0, 0))))
        out = outline.instance_outlines(
            layout, {"m": [_box_piece((0.8, 0.6, 0.72), (0, 0, 0.36))]}, BAND)
        self.assertEqual(list(out), ["m", "m_02"])
        self.assertEqual(len(out["m_02"]), 1)
        self.assertAlmostEqual(float(out["m_02"][0][:, 0].mean()), 3.0, delta=1e-6)

    def test_hull_without_map_footprint_fails(self):
        _, _, outline, _ = _deps()
        layout = arena_world.ArenaLayout(walls=(), furniture=(_item(),))
        r = outline.check_arena(
            layout, {}, {"m": [_box_piece((0.8, 0.6, 0.72), (0, 0, 0.36))]}, BAND)[0]
        self.assertEqual(r.bigger_mm, float("inf"))
        self.assertFalse(r.passed)

    def test_report_writes_null_for_infinite_distance(self):
        _, _, outline, _ = _deps()
        layout = arena_world.ArenaLayout(walls=(), furniture=(_item(),))
        res = outline.check_arena(layout, {"m": (BOX,)}, {"m": []}, BAND)
        with tempfile.TemporaryDirectory() as tmp:
            summary = outline.write_outline_report(res, Path(tmp))
            data = json.loads(summary.read_text())
            self.assertIsNone(data["instances"][0]["smaller_mm"])
            self.assertFalse(data["passed"])


def _pgm(rows):
    h, w = len(rows), len(rows[0])
    return b"P5\n%d %d\n255\n" % (w, h) + bytes(0 if c == "#" else 254 for r in rows for c in r)


class MapDiffTest(unittest.TestCase):
    def test_write_map_diff_colours(self):
        _, _, outline, _ = _deps()
        from PIL import Image
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "map-diff.png"
            outline.write_map_diff(_pgm(["#.#.", "...."]), _pgm(["##..", "...."]), path)
            img = Image.open(path).convert("RGB")
            self.assertEqual(img.getpixel((0, 0)), (200, 200, 200))   # both
            self.assertEqual(img.getpixel((1, 0)), (220, 40, 40))     # new only
            self.assertEqual(img.getpixel((2, 0)), (40, 80, 220))     # old only
            self.assertEqual(img.getpixel((3, 0)), (255, 255, 255))   # free

    def test_write_map_diff_rejects_size_mismatch(self):
        _, _, outline, _ = _deps()
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(ValueError):
                outline.write_map_diff(_pgm(["##"]), _pgm(["###"]), Path(tmp) / "x.png")


if __name__ == "__main__":
    unittest.main()
