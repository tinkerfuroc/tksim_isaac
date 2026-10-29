import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "validation"))

from placement_settle_check import (  # noqa: E402
    WaitTimeout, judge_rest, spawn_poses, wait_fresh, wait_sim,
)  # noqa: E402

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


class FakeSim:
    """Fake wall + sim clocks; each spin advances wall by 0.1 s."""

    def __init__(self, sim_rate=1.0, first_clock_after=0.0, sim_t0=500.0):
        self.wall = 0.0
        self.rate = sim_rate
        self.first = first_clock_after
        self.t0 = sim_t0

    def monotonic(self):
        return self.wall

    def spin(self):
        self.wall += 0.1

    def read_clock(self):
        if self.wall < self.first:
            return None
        return self.t0 + (self.wall - self.first) * self.rate


class WaitTest(unittest.TestCase):
    def test_settle_waits_full_sim_duration_from_first_clock_message(self):
        sim = FakeSim(first_clock_after=2.0)
        wait_sim(sim.read_clock, sim.spin, sim.monotonic, 8.0)
        # 2 s with no clock, then 8 s of sim time (not 0 s).
        self.assertGreaterEqual(sim.wall, 10.0 - 1e-6)

    def test_stalled_clock_raises_after_wall_deadline(self):
        sim = FakeSim(sim_rate=0.0)
        with self.assertRaises(WaitTimeout):
            wait_sim(sim.read_clock, sim.spin, sim.monotonic, 8.0)
        self.assertGreaterEqual(sim.wall, 40.0 - 1e-6)  # max(30, 5 * 8)

    def test_no_clock_publisher_raises(self):
        sim = FakeSim(first_clock_after=1e9)
        with self.assertRaises(WaitTimeout):
            wait_sim(sim.read_clock, sim.spin, sim.monotonic, 1.0)

    def test_identical_stamp_samples_are_not_fresh(self):
        sim = FakeSim()
        stamps = {"a": (5, 0), "b": (5, 0)}
        fresh = wait_fresh(lambda: stamps, dict(stamps), ["a", "b"],
                           sim.spin, sim.monotonic, wall_timeout_s=2.0)
        self.assertEqual(fresh, set())

    def test_new_stamp_is_fresh(self):
        sim = FakeSim()
        stamps = {"a": (5, 0), "b": (5, 0)}
        baseline = dict(stamps)

        def spin():
            sim.spin()
            stamps["a"] = (6, 0)
            stamps["b"] = (6, 0)

        fresh = wait_fresh(lambda: stamps, baseline, ["a", "b"], spin, sim.monotonic)
        self.assertEqual(fresh, {"a", "b"})


if __name__ == "__main__":
    unittest.main()
