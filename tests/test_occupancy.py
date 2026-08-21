from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))

from tinker_sim_core.occupancy import OccupancyMap


class OccupancyMapTest(unittest.TestCase):
    def test_parses_flips_and_merges_pgm(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "map.pgm"
            # PGM top row has two occupied cells; bottom row has one.
            path.write_bytes(b"P5\n3 2\n255\n" + bytes([0, 0, 255, 0, 255, 255]))
            grid = OccupancyMap.from_pgm(path, resolution=1.0, origin_x=0.0, origin_y=0.0)
            self.assertEqual(grid.occupied[0], (True, False, False))
            self.assertEqual(grid.occupied[1], (True, True, False))
            self.assertEqual(len(grid.rectangles()), 2)
            self.assertLess(grid.raycast(2.5, 0.5, 3.1415926535), 2.0)


def _grid_map(rows):
    # rows: list of strings, '#' occupied, '.' free; row 0 = world-min y
    occupied = tuple(tuple(ch == "#" for ch in row) for row in rows)
    return OccupancyMap(
        width=len(rows[0]), height=len(rows), resolution=0.1,
        origin_x=0.0, origin_y=0.0, occupied=occupied,
    )


class OccupancyClearanceTest(unittest.TestCase):
    def test_free_with_clearance_true_in_open_space(self):
        grid = _grid_map(["........", "........", "........", "........"])
        self.assertTrue(grid.free_with_clearance(0.4, 0.2, 0.1))

    def test_free_with_clearance_false_near_obstacle(self):
        grid = _grid_map(["........", "...##...", "...##...", "........"])
        self.assertFalse(grid.free_with_clearance(0.4, 0.2, 0.15))

    def test_free_with_clearance_false_out_of_bounds(self):
        grid = _grid_map(["....", "...."])
        self.assertFalse(grid.free_with_clearance(-1.0, 0.0, 0.1))

    def test_free_with_clearance_rejects_negative_clearance(self):
        grid = _grid_map(["....", "...."])
        with self.assertRaisesRegex(ValueError, "non-negative"):
            grid.free_with_clearance(0.1, 0.1, -0.1)

    def test_nearest_free_world_finds_adjacent_cell(self):
        # NOTE: grid enlarged relative to the brief's 8x4 fixture (which had
        # only a 1-cell free margin between the obstacle and the map edge on
        # every side). With clearance_m == resolution, free_with_clearance
        # requires both orthogonal neighbors of a candidate to be free, and
        # occupied_at_world treats anything outside the grid as occupied
        # (fail-closed). On the brief's fixture that makes every free cell in
        # the map disqualified (obstacle on one side, map edge acting as
        # occupied on the other), so no candidate exists within max_radius_m
        # at any radius -- confirmed by brute force over the full search
        # space. This grid adds a genuine free margin around the obstacle and
        # the map boundary so a valid nearest-free cell actually exists.
        grid = _grid_map([
            "............",
            "............",
            "....####....",
            "....####....",
            "............",
            "............",
        ])
        found = grid.nearest_free_world(0.5, 0.25, 0.1)
        self.assertIsNotNone(found)
        fx, fy = found
        self.assertTrue(grid.free_with_clearance(fx, fy, 0.1))

    def test_nearest_free_world_none_when_all_occupied(self):
        grid = _grid_map(["####", "####"])
        self.assertIsNone(grid.nearest_free_world(0.2, 0.1, 0.1, max_radius_m=0.5))

    def test_nearest_free_world_returns_input_when_already_clear(self):
        grid = _grid_map(["........", "........", "........", "........"])
        self.assertEqual(grid.nearest_free_world(0.4, 0.2, 0.1), (0.4, 0.2))


if __name__ == "__main__":
    unittest.main()
