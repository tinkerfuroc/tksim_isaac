import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "validation"))

from placement_settle_check import judge_rest, spawn_poses  # noqa: E402

SURFACES = [
    {"center_xyz": [2.587, -3.058, 0.724], "edge_margin": 0.05,
     "furniture_id": "rcw26_kitchen_table", "size_xy": [0.86, 0.87],
     "surface_id": "kitchen_table#top", "yaw": 1.5708},
] + [
    {"center_xyz": [float(i), 0.5 * i, 0.4 + 0.1 * i], "edge_margin": 0.05,
     "furniture_id": f"f{i}", "size_xy": [0.6, 0.4],
     "surface_id": f"s{i}#top", "yaw": 0.0}
    for i in range(6)
]
PLACEMENT = {"arena_id": "rcw2026", "schema_version": 1, "surfaces": SURFACES}

# Wide along x before yaw; yawed 90 deg it is wide along y.
WIDE = {"center_xyz": [0.0, 0.0, 0.7], "edge_margin": 0.05,
        "size_xy": [1.0, 0.2], "surface_id": "w#top", "yaw": 0.0}
YAWED = dict(WIDE, yaw=1.5708)


class SpawnPosesTest(unittest.TestCase):
    def test_seven_spawns_5cm_above(self):
        poses = spawn_poses(PLACEMENT)
        self.assertEqual(len(poses), 7)
        self.assertEqual(poses[0][0], "kitchen_table#top")
        for (sid, xyz), surf in zip(poses, SURFACES):
            self.assertEqual(sid, surf["surface_id"])
            self.assertAlmostEqual(xyz[0], surf["center_xyz"][0])
            self.assertAlmostEqual(xyz[1], surf["center_xyz"][1])
            self.assertAlmostEqual(xyz[2], surf["center_xyz"][2] + 0.05)


class JudgeRestTest(unittest.TestCase):
    def test_rest_on_surface_passes(self):
        ok, why = judge_rest(WIDE, (0.1, 0.0, 0.70), (0.1, 0.0, 0.701))
        self.assertTrue(ok)
        self.assertEqual(why, "rests")

    def test_off_the_edge_fails_fell_off(self):
        ok, why = judge_rest(WIDE, (0.6, 0.0, 0.3), (0.6, 0.0, 0.3))
        self.assertFalse(ok)
        self.assertEqual(why, "fell off")

    def test_inside_margin_band_fails_fell_off(self):
        # 0.48 is inside the raw half-width 0.5 but outside 0.5 - 0.05.
        ok, why = judge_rest(WIDE, (0.48, 0.0, 0.7), (0.48, 0.0, 0.7))
        self.assertFalse(ok)
        self.assertEqual(why, "fell off")

    def test_below_surface_inside_rect_fails_sank(self):
        ok, why = judge_rest(WIDE, (0.0, 0.0, 0.6), (0.0, 0.0, 0.6))
        self.assertFalse(ok)
        self.assertEqual(why, "sank")

    def test_floating_fails(self):
        ok, why = judge_rest(WIDE, (0.0, 0.0, 0.9), (0.0, 0.0, 0.9))
        self.assertFalse(ok)
        self.assertEqual(why, "floating")

    def test_moving_fails(self):
        ok, why = judge_rest(WIDE, (0.0, 0.0, 0.70), (0.02, 0.0, 0.70))
        self.assertFalse(ok)
        self.assertEqual(why, "still moving")

    def test_yawed_rectangle_respected(self):
        point = (0.0, 0.4, 0.7)  # inside the yawed rect, outside the unrotated one
        self.assertEqual(judge_rest(WIDE, point, point), (False, "fell off"))
        self.assertEqual(judge_rest(YAWED, point, point), (True, "rests"))


if __name__ == "__main__":
    unittest.main()
